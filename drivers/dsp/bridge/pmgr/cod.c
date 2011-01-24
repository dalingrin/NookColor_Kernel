/*
 * cod.c
 *
 * DSP-BIOS Bridge driver support functions for TI OMAP processors.
 *
 * This module implements DSP code management for the DSP/BIOS Bridge
 * environment. It is mostly a thin wrapper.
 *
 * This module provides an interface for loading both static and
 * dynamic code objects onto DSP systems.
 *
 * Copyright (C) 2005-2006 Texas Instruments, Inc.
 *
 * This package is free software; you can redistribute it and/or modify
 * it under the terms of the GNU General Public License version 2 as
 * published by the Free Software Foundation.
 *
 * THIS PACKAGE IS PROVIDED ``AS IS'' AND WITHOUT ANY EXPRESS OR
 * IMPLIED WARRANTIES, INCLUDING, WITHOUT LIMITATION, THE IMPLIED
 * WARRANTIES OF MERCHANTIBILITY AND FITNESS FOR A PARTICULAR PURPOSE.
 */

/*  ----------------------------------- Host OS */
#include <dspbridge/host_os.h>
#include <linux/fs.h>
#include <linux/uaccess.h>

/*  ----------------------------------- DSP/BIOS Bridge */
#include <dspbridge/std.h>
#include <dspbridge/dbdefs.h>
#include <dspbridge/errbase.h>

/*  ----------------------------------- Trace & Debug */
#include <dspbridge/dbc.h>
#include <dspbridge/gt.h>

/*  ----------------------------------- OS Adaptation Layer */
#include <dspbridge/ldr.h>
#include <dspbridge/mem.h>

/*  ----------------------------------- Platform Manager */
/* Include appropriate loader header file */
#include <dspbridge/dbll.h>

/*  ----------------------------------- This */
#include <dspbridge/cod.h>

/* magic number for handle validation */
#define MAGIC	 0xc001beef

/* macro to validate COD manager handles */
#define IsValid(h)    ((h) != NULL && (h)->ulMagic == MAGIC)

/*
 *  ======== COD_MANAGER ========
 */
struct COD_MANAGER {
	struct DBLL_TarObj *target;
	struct DBLL_LibraryObj *baseLib;
	bool fLoaded;		/* Base library loaded? */
	u32 ulEntry;
	struct LDR_MODULE *hDll;
	struct DBLL_Fxns fxns;
	struct DBLL_Attrs attrs;
	char szZLFile[COD_MAXPATHLENGTH];
	u32 ulMagic;
} ;

/*
 *  ======== COD_LIBRARYOBJ ========
 */
struct COD_LIBRARYOBJ {
	struct DBLL_LibraryObj *dbllLib;
	struct COD_MANAGER *hCodMgr;
} ;

static u32 cRefs = 0L;

#if GT_TRACE
static struct GT_Mask COD_debugMask = { NULL, NULL };
#endif

static struct DBLL_Fxns dbllFxns = {
	(DBLL_CloseFxn) DBLL_close,
	(DBLL_CreateFxn) DBLL_create,
	(DBLL_DeleteFxn) DBLL_delete,
	(DBLL_ExitFxn) DBLL_exit,
	(DBLL_GetAttrsFxn) DBLL_getAttrs,
	(DBLL_GetAddrFxn) DBLL_getAddr,
	(DBLL_GetCAddrFxn) DBLL_getCAddr,
	(DBLL_GetSectFxn) DBLL_getSect,
	(DBLL_InitFxn) DBLL_init,
	(DBLL_LoadFxn) DBLL_load,
	(DBLL_LoadSectFxn) DBLL_loadSect,
	(DBLL_OpenFxn) DBLL_open,
	(DBLL_ReadSectFxn) DBLL_readSect,
	(DBLL_SetAttrsFxn) DBLL_setAttrs,
	(DBLL_UnloadFxn) DBLL_unload,
	(DBLL_UnloadSectFxn) DBLL_unloadSect,
};

static bool NoOp(void);

/*
 * File operations (originally were under kfile.c)
 */
static s32 COD_fClose(struct file *hFile)
{
	/* Check for valid handle */
	if (!hFile)
		return DSP_EHANDLE;

	filp_close(hFile, NULL);

	/* we can't use DSP_SOK here */
	return 0;
}

static struct file *COD_fOpen(CONST char *pszFileName, CONST char *pszMode)
{
	mm_segment_t fs;
	struct file *hFile;

	fs = get_fs();
	set_fs(get_ds());

	/* ignore given mode and open file as read-only */
	hFile = filp_open(pszFileName, O_RDONLY, 0);

	if (IS_ERR(hFile))
		hFile = NULL;

	set_fs(fs);

	return hFile;
}

static s32 COD_fRead(void __user *pBuffer, s32 cSize, s32 cCount,
		     struct file *hFile)
{
	/* check for valid file handle */
	if (!hFile)
		return DSP_EHANDLE;

	if ((cSize > 0) && (cCount > 0) && pBuffer) {
		u32 dwBytesRead;
		mm_segment_t fs;

		/* read from file */
		fs = get_fs();
		set_fs(get_ds());
		dwBytesRead = hFile->f_op->read(hFile, pBuffer, cSize * cCount,
						&(hFile->f_pos));
		set_fs(fs);

		if (!dwBytesRead)
			return DSP_EFREAD;

		return dwBytesRead / cSize;
	}

	return DSP_EINVALIDARG;
}

static s32 COD_fSeek(struct file *hFile, s32 lOffset, s32 cOrigin)
{
	u32 dwCurPos;

	/* check for valid file handle */
	if (!hFile)
		return DSP_EHANDLE;

	/* based on the origin flag, move the internal pointer */
	dwCurPos = hFile->f_op->llseek(hFile, lOffset, cOrigin);

	if ((s32)dwCurPos < 0)
		return DSP_EFAIL;

	/* we can't use DSP_SOK here */
	return 0;
}

static s32 COD_fTell(struct file *hFile)
{
	u32 dwCurPos;

	if (!hFile)
		return DSP_EHANDLE;

	/* Get current position */
	dwCurPos = hFile->f_op->llseek(hFile, 0, SEEK_CUR);

	if ((s32)dwCurPos < 0)
		return DSP_EFAIL;

	return dwCurPos;
}

/*
 *  ======== COD_Close ========
 */
void COD_Close(struct COD_LIBRARYOBJ *lib)
{
	struct COD_MANAGER *hMgr;

	DBC_Require(cRefs > 0);
	DBC_Require(lib != NULL);
	DBC_Require(IsValid(((struct COD_LIBRARYOBJ *)lib)->hCodMgr));

	hMgr = lib->hCodMgr;
	hMgr->fxns.closeFxn(lib->dbllLib);

	kfree(lib);
}

/*
 *  ======== COD_Create ========
 *  Purpose:
 *      Create an object to manage code on a DSP system.
 *      This object can be used to load an initial program image with
 *      arguments that can later be expanded with
 *      dynamically loaded object files.
 *
 */
DSP_STATUS COD_Create(OUT struct COD_MANAGER **phMgr, char *pstrDummyFile,
		     IN OPTIONAL CONST struct COD_ATTRS *attrs)
{
	struct COD_MANAGER *hMgrNew;
	struct DBLL_Attrs zlAttrs;
	DSP_STATUS status = DSP_SOK;

	DBC_Require(cRefs > 0);
	DBC_Require(phMgr != NULL);

	/* assume failure */
	*phMgr = NULL;

	/* we don't support non-default attrs yet */
	if (attrs != NULL)
		return DSP_ENOTIMPL;

	hMgrNew = MEM_Calloc(sizeof(struct COD_MANAGER), MEM_NONPAGED);
	if (hMgrNew == NULL)
		return DSP_EMEMORY;

	hMgrNew->ulMagic = MAGIC;

	/* Set up loader functions */
	hMgrNew->fxns = dbllFxns;

	/* initialize the ZL module */
	hMgrNew->fxns.initFxn();

	zlAttrs.alloc = (DBLL_AllocFxn)NoOp;
	zlAttrs.free = (DBLL_FreeFxn)NoOp;
	zlAttrs.fread = (DBLL_ReadFxn)COD_fRead;
	zlAttrs.fseek = (DBLL_SeekFxn)COD_fSeek;
	zlAttrs.ftell = (DBLL_TellFxn)COD_fTell;
	zlAttrs.fclose = (DBLL_FCloseFxn)COD_fClose;
	zlAttrs.fopen = (DBLL_FOpenFxn)COD_fOpen;
	zlAttrs.symLookup = NULL;
	zlAttrs.baseImage = true;
	zlAttrs.logWrite = NULL;
	zlAttrs.logWriteHandle = NULL;
	zlAttrs.write = NULL;
	zlAttrs.rmmHandle = NULL;
	zlAttrs.wHandle = NULL;
	zlAttrs.symHandle = NULL;
	zlAttrs.symArg = NULL;

	hMgrNew->attrs = zlAttrs;

	status = hMgrNew->fxns.createFxn(&hMgrNew->target, &zlAttrs);

	if (DSP_FAILED(status)) {
		COD_Delete(hMgrNew);
		return COD_E_ZLCREATEFAILED;
	}

	/* return the new manager */
	*phMgr = hMgrNew;

	return DSP_SOK;
}

/*
 *  ======== COD_Delete ========
 *  Purpose:
 *      Delete a code manager object.
 */
void COD_Delete(struct COD_MANAGER *hMgr)
{
	DBC_Require(cRefs > 0);
	DBC_Require(IsValid(hMgr));

	if (hMgr->baseLib) {
		if (hMgr->fLoaded)
			hMgr->fxns.unloadFxn(hMgr->baseLib, &hMgr->attrs);

		hMgr->fxns.closeFxn(hMgr->baseLib);
	}
	if (hMgr->target) {
		hMgr->fxns.deleteFxn(hMgr->target);
		hMgr->fxns.exitFxn();
	}
	hMgr->ulMagic = ~MAGIC;
	kfree(hMgr);
}

/*
 *  ======== COD_Exit ========
 *  Purpose:
 *      Discontinue usage of the COD module.
 *
 */
void COD_Exit(void)
{
	DBC_Require(cRefs > 0);

	cRefs--;

	DBC_Ensure(cRefs >= 0);
}

/*
 *  ======== COD_GetBaseLib ========
 *  Purpose:
 *      Get handle to the base image DBL library.
 */
DSP_STATUS COD_GetBaseLib(struct COD_MANAGER *hManager,
				struct DBLL_LibraryObj **plib)
{
	DSP_STATUS status = DSP_SOK;

	DBC_Require(cRefs > 0);
	DBC_Require(IsValid(hManager));
	DBC_Require(plib != NULL);

	*plib = (struct DBLL_LibraryObj *) hManager->baseLib;

	return status;
}

/*
 *  ======== COD_GetBaseName ========
 */
DSP_STATUS COD_GetBaseName(struct COD_MANAGER *hManager, char *pszName,
				u32 uSize)
{
	DSP_STATUS status = DSP_SOK;

	DBC_Require(cRefs > 0);
	DBC_Require(IsValid(hManager));
	DBC_Require(pszName != NULL);

	if (uSize <= COD_MAXPATHLENGTH)
		strncpy(pszName, hManager->szZLFile, uSize);
	else
		status = DSP_EFAIL;

	return status;
}

/*
 *  ======== COD_GetEntry ========
 *  Purpose:
 *      Retrieve the entry point of a loaded DSP program image
 *
 */
DSP_STATUS COD_GetEntry(struct COD_MANAGER *hManager, u32 *pulEntry)
{
	DBC_Require(cRefs > 0);
	DBC_Require(IsValid(hManager));
	DBC_Require(pulEntry != NULL);

	*pulEntry = hManager->ulEntry;

	return DSP_SOK;
}

/*
 *  ======== COD_GetLoader ========
 *  Purpose:
 *      Get handle to the DBLL loader.
 */
DSP_STATUS COD_GetLoader(struct COD_MANAGER *hManager,
				struct DBLL_TarObj **phLoader)
{
	DSP_STATUS status = DSP_SOK;

	DBC_Require(cRefs > 0);
	DBC_Require(IsValid(hManager));
	DBC_Require(phLoader != NULL);

	*phLoader = (struct DBLL_TarObj *)hManager->target;

	return status;
}

/*
 *  ======== COD_GetSection ========
 *  Purpose:
 *      Retrieve the starting address and length of a section in the COFF file
 *      given the section name.
 */
DSP_STATUS COD_GetSection(struct COD_LIBRARYOBJ *lib, IN char *pstrSect,
			  OUT u32 *puAddr, OUT u32 *puLen)
{
	struct COD_MANAGER *hManager;
	DSP_STATUS status = DSP_SOK;

	DBC_Require(cRefs > 0);
	DBC_Require(lib != NULL);
	DBC_Require(IsValid(lib->hCodMgr));
	DBC_Require(pstrSect != NULL);
	DBC_Require(puAddr != NULL);
	DBC_Require(puLen != NULL);

	*puAddr = 0;
	*puLen = 0;
	if (lib != NULL) {
		hManager = lib->hCodMgr;
		status = hManager->fxns.getSectFxn(lib->dbllLib, pstrSect,
						   puAddr, puLen);
	} else {
		status = COD_E_NOSYMBOLSLOADED;
	}

	DBC_Ensure(DSP_SUCCEEDED(status) || ((*puAddr == 0) && (*puLen == 0)));

	return status;
}

/*
 *  ======== COD_GetSymValue ========
 *  Purpose:
 *      Retrieve the value for the specified symbol. The symbol is first
 *      searched for literally and then, if not found, searched for as a
 *      C symbol.
 *
 */
DSP_STATUS COD_GetSymValue(struct COD_MANAGER *hMgr, char *pstrSym,
			   u32 *pulValue)
{
	struct DBLL_Symbol *pSym;

	DBC_Require(cRefs > 0);
	DBC_Require(IsValid(hMgr));
	DBC_Require(pstrSym != NULL);
	DBC_Require(pulValue != NULL);

	GT_3trace(COD_debugMask, GT_ENTER, "Entered COD_GetSymValue Args \t\n"
		  "hMgr: 0x%x\t\npstrSym: 0x%x\t\npulValue: 0x%x\n",
		  hMgr, pstrSym, pulValue);
	if (hMgr->baseLib) {
		if (!hMgr->fxns.getAddrFxn(hMgr->baseLib, pstrSym, &pSym)) {
			if (!hMgr->fxns.getCAddrFxn(hMgr->baseLib, pstrSym,
			    &pSym))
				return COD_E_SYMBOLNOTFOUND;
		}
	} else {
		return COD_E_NOSYMBOLSLOADED;
	}

	*pulValue = pSym->value;

	return DSP_SOK;
}

/*
 *  ======== COD_Init ========
 *  Purpose:
 *      Initialize the COD module's private state.
 *
 */
bool COD_Init(void)
{
	bool fRetVal = true;

	DBC_Require(cRefs >= 0);

	if (cRefs == 0) {
		DBC_Assert(!COD_debugMask.flags);
		GT_create(&COD_debugMask, "CO");
	}

	if (fRetVal)
		cRefs++;

	DBC_Ensure((fRetVal && cRefs > 0) || (!fRetVal && cRefs >= 0));
	return fRetVal;
}

/*
 *  ======== COD_LoadBase ========
 *  Purpose:
 *      Load the initial program image, optionally with command-line arguments,
 *      on the DSP system managed by the supplied handle. The program to be
 *      loaded must be the first element of the args array and must be a fully
 *      qualified pathname.
 *  Details:
 *      if nArgc doesn't match the number of arguments in the aArgs array, the
 *      aArgs array is searched for a NULL terminating entry, and argc is
 *      recalculated to reflect this.  In this way, we can support NULL
 *      terminating aArgs arrays, if nArgc is very large.
 */
DSP_STATUS COD_LoadBase(struct COD_MANAGER *hMgr, u32 nArgc, char *aArgs[],
			COD_WRITEFXN pfnWrite, void *pArb, char *envp[])
{
	DBLL_Flags flags;
	struct DBLL_Attrs saveAttrs;
	struct DBLL_Attrs newAttrs;
	DSP_STATUS status;
	u32 i;

	DBC_Require(cRefs > 0);
	DBC_Require(IsValid(hMgr));
	DBC_Require(nArgc > 0);
	DBC_Require(aArgs != NULL);
	DBC_Require(aArgs[0] != NULL);
	DBC_Require(pfnWrite != NULL);
	DBC_Require(hMgr->baseLib != NULL);

	/*
	 *  Make sure every argv[] stated in argc has a value, or change argc to
	 *  reflect true number in NULL terminated argv array.
	 */
	for (i = 0; i < nArgc; i++) {
		if (aArgs[i] == NULL) {
			nArgc = i;
			break;
		}
	}

	/* set the write function for this operation */
	hMgr->fxns.getAttrsFxn(hMgr->target, &saveAttrs);

	newAttrs = saveAttrs;
	newAttrs.write = (DBLL_WriteFxn)pfnWrite;
	newAttrs.wHandle = pArb;
	newAttrs.alloc = (DBLL_AllocFxn)NoOp;
	newAttrs.free = (DBLL_FreeFxn)NoOp;
	newAttrs.logWrite = NULL;
	newAttrs.logWriteHandle = NULL;

	/* Load the image */
	flags = DBLL_CODE | DBLL_DATA | DBLL_SYMB;
	status = hMgr->fxns.loadFxn(hMgr->baseLib, flags, &newAttrs,
		 &hMgr->ulEntry);
	if (DSP_FAILED(status))
		hMgr->fxns.closeFxn(hMgr->baseLib);

	if (DSP_SUCCEEDED(status))
		hMgr->fLoaded = true;
	else
		hMgr->baseLib = NULL;

	return status;
}

/*
 *  ======== COD_Open ========
 *      Open library for reading sections.
 */
DSP_STATUS COD_Open(struct COD_MANAGER *hMgr, IN char *pszCoffPath,
		    COD_FLAGS flags, struct COD_LIBRARYOBJ **pLib)
{
	DSP_STATUS status = DSP_SOK;
	struct COD_LIBRARYOBJ *lib = NULL;

	DBC_Require(cRefs > 0);
	DBC_Require(IsValid(hMgr));
	DBC_Require(pszCoffPath != NULL);
	DBC_Require(flags == COD_NOLOAD || flags == COD_SYMB);
	DBC_Require(pLib != NULL);

	*pLib = NULL;

	lib = MEM_Calloc(sizeof(struct COD_LIBRARYOBJ), MEM_NONPAGED);
	if (lib == NULL)
		status = DSP_EMEMORY;

	if (DSP_SUCCEEDED(status)) {
		lib->hCodMgr = hMgr;
		status = hMgr->fxns.openFxn(hMgr->target, pszCoffPath, flags,
					   &lib->dbllLib);
		if (DSP_SUCCEEDED(status))
			*pLib = lib;
	}

	if (DSP_FAILED(status))
		pr_err("%s: error status 0x%x, pszCoffPath: %s flags: 0x%x\n",
					__func__, status, pszCoffPath, flags);
	return status;
}

/*
 *  ======== COD_OpenBase ========
 *  Purpose:
 *      Open base image for reading sections.
 */
DSP_STATUS COD_OpenBase(struct COD_MANAGER *hMgr, IN char *pszCoffPath,
			DBLL_Flags flags)
{
	DSP_STATUS status = DSP_SOK;
	struct DBLL_LibraryObj *lib;

	DBC_Require(cRefs > 0);
	DBC_Require(IsValid(hMgr));
	DBC_Require(pszCoffPath != NULL);

	/* if we previously opened a base image, close it now */
	if (hMgr->baseLib) {
		if (hMgr->fLoaded) {
			hMgr->fxns.unloadFxn(hMgr->baseLib, &hMgr->attrs);
			hMgr->fLoaded = false;
		}
		hMgr->fxns.closeFxn(hMgr->baseLib);
		hMgr->baseLib = NULL;
	}
	status = hMgr->fxns.openFxn(hMgr->target, pszCoffPath, flags, &lib);
	if (DSP_SUCCEEDED(status)) {
		/* hang onto the library for subsequent sym table usage */
		hMgr->baseLib = lib;
		strncpy(hMgr->szZLFile, pszCoffPath, COD_MAXPATHLENGTH - 1);
		hMgr->szZLFile[COD_MAXPATHLENGTH - 1] = '\0';
	}

	if (DSP_FAILED(status))
		pr_err("%s: error status 0x%x pszCoffPath: %s\n", __func__,
							status, pszCoffPath);
	return status;
}

/*
 *  ======== COD_ReadSection ========
 *  Purpose:
 *      Retrieve the content of a code section given the section name.
 */
DSP_STATUS COD_ReadSection(struct COD_LIBRARYOBJ *lib, IN char *pstrSect,
			   OUT char *pstrContent, IN u32 cContentSize)
{
	DSP_STATUS status = DSP_SOK;

	DBC_Require(cRefs > 0);
	DBC_Require(lib != NULL);
	DBC_Require(IsValid(lib->hCodMgr));
	DBC_Require(pstrSect != NULL);
	DBC_Require(pstrContent != NULL);

	if (lib != NULL)
		status = lib->hCodMgr->fxns.readSectFxn(lib->dbllLib, pstrSect,
							pstrContent,
							cContentSize);
	else
		status = COD_E_NOSYMBOLSLOADED;

	return status;
}

/*
 *  ======== NoOp ========
 *  Purpose:
 *      No Operation.
 *
 */
static bool NoOp(void)
{
	return true;
}

