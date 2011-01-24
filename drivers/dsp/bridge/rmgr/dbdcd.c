/*
 * dbdcd.c
 *
 * DSP-BIOS Bridge driver support functions for TI OMAP processors.
 *
 * This file contains the implementation of the DSP/BIOS Bridge
 * Configuration Database (DCD).
 *
 * Notes:
 *   The fxn DCD_GetObjects can apply a callback fxn to each DCD object
 *   that is located in a specified COFF file.  At the moment,
 *   DCD_AutoRegister, DCD_AutoUnregister, and NLDR module all use
 *   DCD_GetObjects.
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

/*  ----------------------------------- DSP/BIOS Bridge */
#include <dspbridge/std.h>
#include <dspbridge/dbdefs.h>
#include <dspbridge/errbase.h>
/*  ----------------------------------- Trace & Debug */
#include <dspbridge/dbc.h>
#include <dspbridge/gt.h>

/*  ----------------------------------- OS Adaptation Layer */
#include <dspbridge/mem.h>
#include <dspbridge/reg.h>

/*  ----------------------------------- Platform Manager */
#include <dspbridge/cod.h>

/*  ----------------------------------- Others */
#include <dspbridge/uuidutil.h>

/*  ----------------------------------- This */
#include <dspbridge/dbdcd.h>

/*  ----------------------------------- Global defines. */
#define SIGNATURE       0x5f444344	/* "DCD_" (in reverse). */

#define IsValidHandle(h)	(((h) != NULL) && (h->dwSignature == SIGNATURE))

#define MAX_INT2CHAR_LENGTH     16   /* Maximum int2char len of 32 bit int. */

/* Name of section containing dependent libraries */
#define DEPLIBSECT		".dspbridge_deplibs"

/* DCD specific structures. */
struct DCD_MANAGER {
	u32 dwSignature;	/* Used for object validation.   */
	struct COD_MANAGER *hCodMgr;	/* Handle to COD manager object. */
};

/* Global reference variables. */
static u32 cRefs;
static u32 cEnumRefs;

extern struct GT_Mask curTrace;

/* Helper function prototypes. */
static s32 Atoi(char *pszBuf);
static DSP_STATUS GetAttrsFromBuf(char *pszBuf, u32 ulBufSize,
				  enum DSP_DCDOBJTYPE objType,
				  struct DCD_GENERICOBJ *pGenObj);
static void CompressBuf(char *pszBuf, u32 ulBufSize, s32 cCharSize);
static char DspChar2GppChar(char *pWord, s32 cDspCharSize);
static DSP_STATUS GetDepLibInfo(IN struct DCD_MANAGER *hDcdMgr,
				IN struct DSP_UUID *pUuid,
				IN OUT u16 *pNumLibs,
				OPTIONAL OUT u16 *pNumPersLibs,
				OPTIONAL OUT struct DSP_UUID *pDepLibUuids,
				OPTIONAL OUT bool *pPersistentDepLibs,
				IN enum NLDR_PHASE phase);

/*
 *  ======== DCD_AutoRegister ========
 *  Purpose:
 *      Parses the supplied image and resigsters with DCD.
 */
DSP_STATUS DCD_AutoRegister(IN struct DCD_MANAGER *hDcdMgr,
				IN char *pszCoffPath)
{
	DSP_STATUS status = DSP_SOK;

	DBC_Require(cRefs > 0);

	if (IsValidHandle(hDcdMgr))
		status = DCD_GetObjects(hDcdMgr, pszCoffPath,
					(DCD_REGISTERFXN)DCD_RegisterObject,
					(void *)pszCoffPath);
	else
		status = DSP_EHANDLE;

	return status;
}

/*
 *  ======== DCD_AutoUnregister ========
 *  Purpose:
 *      Parses the supplied DSP image and unresiters from DCD.
 */
DSP_STATUS DCD_AutoUnregister(IN struct DCD_MANAGER *hDcdMgr,
				IN char *pszCoffPath)
{
	DSP_STATUS status = DSP_SOK;

	DBC_Require(cRefs > 0);

	if (IsValidHandle(hDcdMgr))
		status = DCD_GetObjects(hDcdMgr, pszCoffPath,
				(DCD_REGISTERFXN)DCD_RegisterObject, NULL);
	else
		status = DSP_EHANDLE;

	return status;
}

/*
 *  ======== DCD_CreateManager ========
 *  Purpose:
 *      Creates DCD manager.
 */
DSP_STATUS DCD_CreateManager(IN char *pszZlDllName,
				OUT struct DCD_MANAGER **phDcdMgr)
{
	struct COD_MANAGER *hCodMgr;		/* COD manager handle */
	struct DCD_MANAGER *pDcdMgr = NULL;	/* DCD Manager pointer */
	DSP_STATUS status = DSP_SOK;

	DBC_Require(cRefs >= 0);
	DBC_Require(phDcdMgr);

	status = COD_Create(&hCodMgr, pszZlDllName, NULL);
	if (DSP_FAILED(status))
		goto func_end;

	/* Create a DCD object. */
	MEM_AllocObject(pDcdMgr, struct DCD_MANAGER, SIGNATURE);
	if (pDcdMgr != NULL) {
		/* Fill out the object. */
		pDcdMgr->hCodMgr = hCodMgr;

		/* Return handle to this DCD interface. */
		*phDcdMgr = pDcdMgr;
	} else {
		status = DSP_EMEMORY;

		/*
		 * If allocation of DcdManager object failed, delete the
		 * COD manager.
		 */
		COD_Delete(hCodMgr);
	}

	DBC_Ensure((DSP_SUCCEEDED(status)) || ((hCodMgr == NULL) &&
			(status == DSP_EFAIL)) || ((pDcdMgr == NULL) &&
			(status == DSP_EMEMORY)));

func_end:
	return status;
}

/*
 *  ======== DCD_DestroyManager ========
 *  Purpose:
 *      Frees DCD Manager object.
 */
DSP_STATUS DCD_DestroyManager(IN struct DCD_MANAGER *hDcdMgr)
{
	struct DCD_MANAGER *pDcdMgr = hDcdMgr;
	DSP_STATUS status = DSP_EHANDLE;

	DBC_Require(cRefs >= 0);

	if (IsValidHandle(hDcdMgr)) {
		/* Delete the COD manager. */
		COD_Delete(pDcdMgr->hCodMgr);

		/* Deallocate a DCD manager object. */
		MEM_FreeObject(pDcdMgr);

		status = DSP_SOK;
	}

	return status;
}

/*
 *  ======== DCD_EnumerateObject ========
 *  Purpose:
 *      Enumerates objects in the DCD.
 */
DSP_STATUS DCD_EnumerateObject(IN s32 cIndex, IN enum DSP_DCDOBJTYPE objType,
			       OUT struct DSP_UUID *pUuid)
{
	DSP_STATUS status = DSP_SOK;
	char szRegKey[REG_MAXREGPATHLENGTH];
	char szValue[REG_MAXREGPATHLENGTH];
	char szData[REG_MAXREGPATHLENGTH];
	u32 dwValueSize;
	u32 dwDataSize;
	struct DSP_UUID dspUuid;
	char szObjType[MAX_INT2CHAR_LENGTH];	/* str. rep. of objType. */
	u32 dwKeyLen = 0;

	DBC_Require(cRefs >= 0);
	DBC_Require(cIndex >= 0);
	DBC_Require(pUuid != NULL);

	if ((cIndex != 0) && (cEnumRefs == 0)) {
		/*
		 * If an enumeration is being performed on an index greater
		 * than zero, then the current cEnumRefs must have been
		 * incremented to greater than zero.
		 */
		status = DSP_ECHANGEDURINGENUM;
	} else {
		/* Enumerate a specific key in the registry by index. */
		dwValueSize = REG_MAXREGPATHLENGTH;
		dwDataSize = REG_MAXREGPATHLENGTH;

		/*
		 * Pre-determine final key length. It's length of DCD_REGKEY +
		 *  "_\0" + length of szObjType string + terminating NULL.
		 */
		dwKeyLen = strlen(DCD_REGKEY) + 1 + sizeof(szObjType) + 1;
		DBC_Assert(dwKeyLen < REG_MAXREGPATHLENGTH);

		/* Create proper REG key; concatenate DCD_REGKEY with
		 * objType. */
		strncpy(szRegKey, DCD_REGKEY, strlen(DCD_REGKEY) + 1);
		if ((strlen(szRegKey) + strlen("_\0")) <
		   REG_MAXREGPATHLENGTH) {
			strncat(szRegKey, "_\0", 2);
		} else {
			status = DSP_EFAIL;
		}

		/* This snprintf is guaranteed not to exceed max size of an
		 * integer. */
		status = snprintf(szObjType, MAX_INT2CHAR_LENGTH, "%d",
				 objType);

		if (status == -1) {
			status = DSP_EFAIL;
		} else {
			status = DSP_SOK;
			if ((strlen(szRegKey) + strlen(szObjType)) <
			   REG_MAXREGPATHLENGTH) {
				strncat(szRegKey, szObjType,
					strlen(szObjType) + 1);
			} else {
				status = DSP_EFAIL;
			}
		}

		if (DSP_SUCCEEDED(status)) {
			status = REG_EnumValue(cIndex, szRegKey, szValue,
					&dwValueSize, szData, &dwDataSize);
		}

		if (DSP_SUCCEEDED(status)) {
			/* Create UUID value using string retrieved from
			 * registry. */
			UUID_UuidFromString(szValue, &dspUuid);

			*pUuid = dspUuid;

			/* Increment cEnumRefs to update reference count. */
			cEnumRefs++;

			status = DSP_SOK;
		} else if (status == REG_E_NOMOREITEMS) {
			/* At the end of enumeration. Reset cEnumRefs. */
			cEnumRefs = 0;

			status = DSP_SENUMCOMPLETE;
		} else {
			status = DSP_EFAIL;
		}
	}

	DBC_Ensure(pUuid || (status == DSP_EFAIL));

	return status;
}

/*
 *  ======== DCD_Exit ========
 *  Purpose:
 *      Discontinue usage of the DCD module.
 */
void DCD_Exit(void)
{
	DBC_Require(cRefs > 0);

	cRefs--;
	if (cRefs == 0) {
		COD_Exit();
	}

	DBC_Ensure(cRefs >= 0);
}

/*
 *  ======== DCD_GetDepLibs ========
 */
DSP_STATUS DCD_GetDepLibs(IN struct DCD_MANAGER *hDcdMgr,
			 IN struct DSP_UUID *pUuid,
			 u16 numLibs, OUT struct DSP_UUID *pDepLibUuids,
			 OUT bool *pPersistentDepLibs, IN enum NLDR_PHASE phase)
{
	DSP_STATUS status = DSP_SOK;

	DBC_Require(cRefs > 0);
	DBC_Require(IsValidHandle(hDcdMgr));
	DBC_Require(pUuid != NULL);
	DBC_Require(pDepLibUuids != NULL);
	DBC_Require(pPersistentDepLibs != NULL);

	status = GetDepLibInfo(hDcdMgr, pUuid, &numLibs, NULL, pDepLibUuids,
			      pPersistentDepLibs, phase);

	return status;
}

/*
 *  ======== DCD_GetNumDepLibs ========
 */
DSP_STATUS DCD_GetNumDepLibs(IN struct DCD_MANAGER *hDcdMgr,
			    IN struct DSP_UUID *pUuid, OUT u16 *pNumLibs,
			    OUT u16 *pNumPersLibs, IN enum NLDR_PHASE phase)
{
	DSP_STATUS status = DSP_SOK;

	DBC_Require(cRefs > 0);
	DBC_Require(IsValidHandle(hDcdMgr));
	DBC_Require(pNumLibs != NULL);
	DBC_Require(pNumPersLibs != NULL);
	DBC_Require(pUuid != NULL);

	status = GetDepLibInfo(hDcdMgr, pUuid, pNumLibs, pNumPersLibs,
				NULL, NULL, phase);

	return status;
}

/*
 *  ======== DCD_GetObjectDef ========
 *  Purpose:
 *      Retrieves the properties of a node or processor based on the UUID and
 *      object type.
 */
DSP_STATUS DCD_GetObjectDef(IN struct DCD_MANAGER *hDcdMgr,
			IN struct DSP_UUID *pObjUuid,
			IN enum DSP_DCDOBJTYPE objType,
			OUT struct DCD_GENERICOBJ *pObjDef)
{
	struct DCD_MANAGER *pDcdMgr = hDcdMgr;	/* pointer to DCD manager */
	struct COD_LIBRARYOBJ *lib = NULL;
	DSP_STATUS status = DSP_SOK;
	u32 ulAddr = 0;	/* Used by COD_GetSection */
	u32 ulLen = 0;	/* Used by COD_GetSection */
	u32 dwBufSize;	/* Used by REG functions */
	char szRegKey[REG_MAXREGPATHLENGTH];
	char *szUuid;		/*[MAXUUIDLEN];*/
	char szRegData[REG_MAXREGPATHLENGTH];
	char szSectName[MAXUUIDLEN + 2];	/* ".[UUID]\0" */
	char *pszCoffBuf;
	u32 dwKeyLen;		/* Len of REG key. */
	char szObjType[MAX_INT2CHAR_LENGTH];	/* str. rep. of objType. */

	DBC_Require(cRefs > 0);
	DBC_Require(pObjDef != NULL);
	DBC_Require(pObjUuid != NULL);

	szUuid = (char *)MEM_Calloc(MAXUUIDLEN, MEM_PAGED);
	if (!szUuid) {
		status = DSP_EMEMORY;
		goto func_end;
	}

	if (!IsValidHandle(hDcdMgr)) {
		status = DSP_EHANDLE;
		goto func_end;
	}

	/* Pre-determine final key length. It's length of DCD_REGKEY +
	 *  "_\0" + length of szObjType string + terminating NULL */
	dwKeyLen = strlen(DCD_REGKEY) + 1 + sizeof(szObjType) + 1;
	DBC_Assert(dwKeyLen < REG_MAXREGPATHLENGTH);

	/* Create proper REG key; concatenate DCD_REGKEY with objType. */
	strncpy(szRegKey, DCD_REGKEY, strlen(DCD_REGKEY) + 1);

	if ((strlen(szRegKey) + strlen("_\0")) < REG_MAXREGPATHLENGTH)
		strncat(szRegKey, "_\0", 2);
	else
		status = DSP_EFAIL;

	status = snprintf(szObjType, MAX_INT2CHAR_LENGTH, "%d", objType);
	if (status == -1) {
		status = DSP_EFAIL;
	} else {
		status = DSP_SOK;

		if ((strlen(szRegKey) + strlen(szObjType)) <
		   REG_MAXREGPATHLENGTH) {
			strncat(szRegKey, szObjType, strlen(szObjType) + 1);
		} else {
			status = DSP_EFAIL;
		}

		/* Create UUID value to set in registry. */
		UUID_UuidToString(pObjUuid, szUuid, MAXUUIDLEN);

		if ((strlen(szRegKey) + MAXUUIDLEN) < REG_MAXREGPATHLENGTH)
			strncat(szRegKey, szUuid, MAXUUIDLEN);
		else
			status = DSP_EFAIL;

		/* Retrieve paths from the registry based on struct DSP_UUID */
		dwBufSize = REG_MAXREGPATHLENGTH;
	}
	if (DSP_SUCCEEDED(status))
		status = REG_GetValue(szRegKey, (u8 *)szRegData, &dwBufSize);

	if (DSP_FAILED(status)) {
		status = DSP_EUUID;
		goto func_end;
	}

	/* Open COFF file. */
	status = COD_Open(pDcdMgr->hCodMgr, szRegData, COD_NOLOAD, &lib);
	if (DSP_FAILED(status)) {
		status = DSP_EDCDLOADBASE;
		goto func_end;
	}

	/* Ensure szUuid + 1 is not greater than sizeof szSectName. */
	DBC_Assert((strlen(szUuid) + 1) < sizeof(szSectName));

	/* Create section name based on node UUID. A period is
	 * pre-pended to the UUID string to form the section name.
	 * I.e. ".24BC8D90_BB45_11d4_B756_006008BDB66F" */
	strncpy(szSectName, ".", 2);
	strncat(szSectName, szUuid, strlen(szUuid));

	/* Get section information. */
	status = COD_GetSection(lib, szSectName, &ulAddr, &ulLen);
	if (DSP_FAILED(status)) {
		status = DSP_EDCDGETSECT;
		goto func_end;
	}

	/* Allocate zeroed buffer. */
	pszCoffBuf = MEM_Calloc(ulLen + 4, MEM_PAGED);
#ifdef _DB_TIOMAP
	if (strstr(szRegData, "iva") == NULL) {
		/* Locate section by objectID and read its content. */
		status = COD_ReadSection(lib, szSectName, pszCoffBuf, ulLen);
	} else {
		status = COD_ReadSection(lib, szSectName, pszCoffBuf, ulLen);
		GT_0trace(curTrace, GT_4CLASS,
			 "Skipped Byte swap for IVA !!\n");
	}
#else
	status = COD_ReadSection(lib, szSectName, pszCoffBuf, ulLen);
#endif
	if (DSP_SUCCEEDED(status)) {
		/* Compres DSP buffer to conform to PC format. */
		if (strstr(szRegData, "iva") == NULL) {
			CompressBuf(pszCoffBuf, ulLen, DSPWORDSIZE);
		} else {
			CompressBuf(pszCoffBuf, ulLen, 1);
			GT_0trace(curTrace, GT_4CLASS, "Compressing IVA "
				 "COFF buffer by 1 for IVA !!\n");
		}

		/* Parse the content of the COFF buffer. */
		status = GetAttrsFromBuf(pszCoffBuf, ulLen, objType, pObjDef);
		if (DSP_FAILED(status))
			status = DSP_EDCDPARSESECT;
	} else {
		status = DSP_EDCDREADSECT;
	}

	/* Free the previously allocated dynamic buffer. */
	kfree(pszCoffBuf);
func_end:
	if (lib)
		COD_Close(lib);

	kfree(szUuid);

	return status;
}

/*
 *  ======== DCD_GetObjects ========
 */
DSP_STATUS DCD_GetObjects(IN struct DCD_MANAGER *hDcdMgr, IN char *pszCoffPath,
			 DCD_REGISTERFXN registerFxn, void *handle)
{
	struct DCD_MANAGER *pDcdMgr = hDcdMgr;	/* pointer to DCD manager */
	DSP_STATUS status = DSP_SOK;
	char *pszCoffBuf;
	char *pszCur;
	struct COD_LIBRARYOBJ *lib = NULL;
	u32 ulAddr = 0;	/* Used by COD_GetSection */
	u32 ulLen = 0;	/* Used by COD_GetSection */
	char seps[] = ":, ";
	char *pToken = NULL;
	struct DSP_UUID dspUuid;
	s32 cObjectType;

	DBC_Require(cRefs > 0);
	if (!IsValidHandle(hDcdMgr)) {
		status = DSP_EHANDLE;
		goto func_end;
	}

	/* Open DSP coff file, don't load symbols. */
	status = COD_Open(pDcdMgr->hCodMgr, pszCoffPath, COD_NOLOAD, &lib);
	if (DSP_FAILED(status)) {
		status = DSP_EDCDLOADBASE;
		goto func_cont;
	}

	/* Get DCD_RESIGER_SECTION section information. */
	status = COD_GetSection(lib, DCD_REGISTER_SECTION, &ulAddr, &ulLen);
	if (DSP_FAILED(status) ||  !(ulLen > 0)) {
		status = DSP_EDCDNOAUTOREGISTER;
		goto func_cont;
	}

	/* Allocate zeroed buffer. */
	pszCoffBuf = MEM_Calloc(ulLen + 4, MEM_PAGED);
#ifdef _DB_TIOMAP
	if (strstr(pszCoffPath, "iva") == NULL) {
		/* Locate section by objectID and read its content. */
		status = COD_ReadSection(lib, DCD_REGISTER_SECTION,
					pszCoffBuf, ulLen);
	} else {
		GT_0trace(curTrace, GT_4CLASS, "Skipped Byte swap for IVA!!\n");
		status = COD_ReadSection(lib, DCD_REGISTER_SECTION,
					pszCoffBuf, ulLen);
	}
#else
	status = COD_ReadSection(lib, DCD_REGISTER_SECTION, pszCoffBuf, ulLen);
#endif
	if (DSP_SUCCEEDED(status)) {
		/* Compress DSP buffer to conform to PC format. */
		if (strstr(pszCoffPath, "iva") == NULL) {
			CompressBuf(pszCoffBuf, ulLen, DSPWORDSIZE);
		} else {
			CompressBuf(pszCoffBuf, ulLen, 1);
			GT_0trace(curTrace, GT_4CLASS, "Compress COFF buffer "
				 "with 1 word for IVA !!\n");
		}

		/* Read from buffer and register object in buffer. */
		pszCur = pszCoffBuf;
		while ((pToken = strsep(&pszCur, seps)) && *pToken != '\0') {
			/*  Retrieve UUID string. */
			UUID_UuidFromString(pToken, &dspUuid);

			/*  Retrieve object type */
			pToken = strsep(&pszCur, seps);

			/*  Retrieve object type */
			cObjectType = Atoi(pToken);

			/*
			 *  Apply registerFxn to the found DCD object.
			 *  Possible actions include:
			 *
			 *  1) Register found DCD object.
			 *  2) Unregister found DCD object (when handle == NULL)
			 *  3) Add overlay node.
			 */
			status = registerFxn(&dspUuid, cObjectType, handle);
			if (DSP_FAILED(status)) {
				/* if error occurs, break from while loop. */
				break;
			}
		}
	} else {
		status = DSP_EDCDREADSECT;
	}

	/* Free the previously allocated dynamic buffer. */
	kfree(pszCoffBuf);
func_cont:
	if (lib)
		COD_Close(lib);

func_end:
	return status;
}

/*
 *  ======== DCD_GetLibraryName ========
 *  Purpose:
 *      Retrieves the library name for the given UUID.
 *
 */
DSP_STATUS DCD_GetLibraryName(IN struct DCD_MANAGER *hDcdMgr,
			IN struct DSP_UUID *pUuid,
			IN OUT char *pstrLibName, IN OUT u32 *pdwSize,
			enum NLDR_PHASE phase, OUT bool *fPhaseSplit)
{
	char szRegKey[REG_MAXREGPATHLENGTH];
	char szUuid[MAXUUIDLEN];
	u32 dwKeyLen;		/* Len of REG key. */
	char szObjType[MAX_INT2CHAR_LENGTH];	/* str. rep. of objType. */
	DSP_STATUS status = DSP_SOK;

	DBC_Require(pUuid != NULL);
	DBC_Require(pstrLibName != NULL);
	DBC_Require(pdwSize != NULL);
	DBC_Require(IsValidHandle(hDcdMgr));

	GT_4trace(curTrace, GT_ENTER,
		 "DCD_GetLibraryName: hDcdMgr 0x%x, pUuid 0x%x, "
		 " pstrLibName 0x%x, pdwSize 0x%x\n", hDcdMgr, pUuid,
		 pstrLibName, pdwSize);

	/*
	 *  Pre-determine final key length. It's length of DCD_REGKEY +
	 *  "_\0" + length of szObjType string + terminating NULL.
	 */
	dwKeyLen = strlen(DCD_REGKEY) + 1 + sizeof(szObjType) + 1;
	DBC_Assert(dwKeyLen < REG_MAXREGPATHLENGTH);

	/* Create proper REG key; concatenate DCD_REGKEY with objType. */
	strncpy(szRegKey, DCD_REGKEY, strlen(DCD_REGKEY) + 1);
	if ((strlen(szRegKey) + strlen("_\0")) < REG_MAXREGPATHLENGTH)
		strncat(szRegKey, "_\0", 2);
	else
		status = DSP_EFAIL;

	switch (phase) {
	case NLDR_CREATE:
		/* create phase type */
		sprintf(szObjType, "%d", DSP_DCDCREATELIBTYPE);
		break;
	case NLDR_EXECUTE:
		/* execute phase type */
		sprintf(szObjType, "%d", DSP_DCDEXECUTELIBTYPE);
		break;
	case NLDR_DELETE:
		/* delete phase type */
		sprintf(szObjType, "%d", DSP_DCDDELETELIBTYPE);
		break;
	case NLDR_NOPHASE:
		/* known to be a dependent library */
		sprintf(szObjType, "%d", DSP_DCDLIBRARYTYPE);
		break;
	default:
		status = DSP_EINVALIDARG;
		DBC_Assert(false);
	}
	if (DSP_SUCCEEDED(status)) {
		if ((strlen(szRegKey) + strlen(szObjType)) <
		   REG_MAXREGPATHLENGTH) {
			strncat(szRegKey, szObjType, strlen(szObjType) + 1);
		} else {
			status = DSP_EFAIL;
		}
		/* Create UUID value to find match in registry. */
		UUID_UuidToString(pUuid, szUuid, MAXUUIDLEN);
		if ((strlen(szRegKey) + MAXUUIDLEN) <
		   REG_MAXREGPATHLENGTH) {
			strncat(szRegKey, szUuid, MAXUUIDLEN);
		} else {
			status = DSP_EFAIL;
		}
	}
	if (DSP_SUCCEEDED(status))
		/* Retrieve path from the registry based on DSP_UUID */
		status = REG_GetValue(szRegKey, (u8 *)pstrLibName, pdwSize);

	/* If can't find, phases might be registered as generic LIBRARYTYPE */
	if (DSP_FAILED(status) && phase != NLDR_NOPHASE) {
		if (fPhaseSplit)
			*fPhaseSplit = false;

		strncpy(szRegKey, DCD_REGKEY, strlen(DCD_REGKEY) + 1);
		if ((strlen(szRegKey) + strlen("_\0")) <
		   REG_MAXREGPATHLENGTH) {
			strncat(szRegKey, "_\0", 2);
		} else {
			status = DSP_EFAIL;
		}
		sprintf(szObjType, "%d", DSP_DCDLIBRARYTYPE);
		if ((strlen(szRegKey) + strlen(szObjType))
		   < REG_MAXREGPATHLENGTH) {
			strncat(szRegKey, szObjType, strlen(szObjType) + 1);
		} else {
			status = DSP_EFAIL;
		}
		UUID_UuidToString(pUuid, szUuid, MAXUUIDLEN);
		if ((strlen(szRegKey) + MAXUUIDLEN) < REG_MAXREGPATHLENGTH)
			strncat(szRegKey, szUuid, MAXUUIDLEN);
		else
			status = DSP_EFAIL;

		status = REG_GetValue(szRegKey, (u8 *)pstrLibName, pdwSize);
	}

	return status;
}

/*
 *  ======== DCD_Init ========
 *  Purpose:
 *      Initialize the DCD module.
 */
bool DCD_Init(void)
{
	bool fInitCOD;
	bool fInit = true;

	DBC_Require(cRefs >= 0);

	if (cRefs == 0) {
		/* Initialize required modules. */
		fInitCOD = COD_Init();

		if (!fInitCOD) {
			fInit = false;
			/* Exit initialized modules. */
			if (fInitCOD)
				COD_Exit();
		}
	}

	if (fInit)
		cRefs++;

	DBC_Ensure((fInit && (cRefs > 0)) || (!fInit && (cRefs == 0)));

	return fInit;
}

/*
 *  ======== DCD_RegisterObject ========
 *  Purpose:
 *      Registers a node or a processor with the DCD.
 *      If pszPathName == NULL, unregister the specified DCD object.
 */
DSP_STATUS DCD_RegisterObject(IN struct DSP_UUID *pUuid,
			IN enum DSP_DCDOBJTYPE objType,
			IN char *pszPathName)
{
	DSP_STATUS status = DSP_SOK;
	char szRegKey[REG_MAXREGPATHLENGTH];
	char szUuid[MAXUUIDLEN + 1];
	u32 dwPathSize = 0;
	u32 dwKeyLen;				/* Len of REG key. */
	char szObjType[MAX_INT2CHAR_LENGTH];	/* str. rep. of objType. */

	DBC_Require(cRefs > 0);
	DBC_Require(pUuid != NULL);
	DBC_Require((objType == DSP_DCDNODETYPE) ||
			(objType == DSP_DCDPROCESSORTYPE) ||
			(objType == DSP_DCDLIBRARYTYPE) ||
			(objType == DSP_DCDCREATELIBTYPE) ||
			(objType == DSP_DCDEXECUTELIBTYPE) ||
			(objType == DSP_DCDDELETELIBTYPE));

	GT_3trace(curTrace, GT_ENTER, "DCD_RegisterObject: object UUID 0x%x, "
		 "objType %d, szPathName %s\n", pUuid, objType, pszPathName);

	/*
	 * Pre-determine final key length. It's length of DCD_REGKEY +
	 *  "_\0" + length of szObjType string + terminating NULL.
	 */
	dwKeyLen = strlen(DCD_REGKEY) + 1 + sizeof(szObjType) + 1;
	DBC_Assert(dwKeyLen < REG_MAXREGPATHLENGTH);

	/* Create proper REG key; concatenate DCD_REGKEY with objType. */
	strncpy(szRegKey, DCD_REGKEY, strlen(DCD_REGKEY) + 1);
	if ((strlen(szRegKey) + strlen("_\0")) < REG_MAXREGPATHLENGTH)
		strncat(szRegKey, "_\0", 2);
	else {
		status = DSP_EFAIL;
		goto func_end;
	}

	status = snprintf(szObjType, MAX_INT2CHAR_LENGTH, "%d", objType);
	if (status == -1) {
		status = DSP_EFAIL;
	} else {
		status = DSP_SOK;
		if ((strlen(szRegKey) + strlen(szObjType)) <
		   REG_MAXREGPATHLENGTH) {
			strncat(szRegKey, szObjType, strlen(szObjType) + 1);
		} else
			status = DSP_EFAIL;

		/* Create UUID value to set in registry. */
		UUID_UuidToString(pUuid, szUuid, MAXUUIDLEN);
		if ((strlen(szRegKey) + MAXUUIDLEN) < REG_MAXREGPATHLENGTH)
			strncat(szRegKey, szUuid, MAXUUIDLEN);
		else
			status = DSP_EFAIL;
	}

	if (DSP_FAILED(status))
		goto func_end;

	/*
	 * If pszPathName != NULL, perform registration, otherwise,
	 * perform unregistration.
	 */
	if (pszPathName) {
		/* Add new reg value (UUID+objType) with COFF path info. */
		dwPathSize = strlen(pszPathName) + 1;
		status = REG_SetValue(szRegKey, (u8 *)pszPathName, dwPathSize);
		GT_2trace(curTrace, GT_6CLASS, "REG_SetValue  "
			  "(u8 *)pszPathName=%s, dwPathSize=%d\n",
			  pszPathName, dwPathSize);
	} else {
		/* Deregister an existing object. */
		status = REG_DeleteValue(szRegKey);
	}

	if (DSP_SUCCEEDED(status)) {
		/*
		 *  Because the node database has been updated through a
		 *  successful object registration/de-registration operation,
		 *  we need to reset the object enumeration counter to allow
		 *  current enumerations to reflect this update in the node
		 *  database.
		 */
		cEnumRefs = 0;
	}
func_end:
	return status;
}

/*
 *  ======== DCD_UnregisterObject ========
 *  Call DCD_Register object with pszPathName set to NULL to
 *  perform actual object de-registration.
 */
DSP_STATUS DCD_UnregisterObject(IN struct DSP_UUID *pUuid,
				IN enum DSP_DCDOBJTYPE objType)
{
	DSP_STATUS status = DSP_SOK;

	DBC_Require(cRefs > 0);
	DBC_Require(pUuid != NULL);
	DBC_Require((objType == DSP_DCDNODETYPE) ||
		   (objType == DSP_DCDPROCESSORTYPE) ||
		   (objType == DSP_DCDLIBRARYTYPE) ||
		   (objType == DSP_DCDCREATELIBTYPE) ||
		   (objType == DSP_DCDEXECUTELIBTYPE) ||
		   (objType == DSP_DCDDELETELIBTYPE));

	/*
	 *  When DCD_RegisterObject is called with NULL as pathname,
	 *  it indicates an unregister object operation.
	 */
	status = DCD_RegisterObject(pUuid, objType, NULL);

	return status;
}

/*
 **********************************************************************
 * DCD Helper Functions
 **********************************************************************
 */

/*
 *  ======== Atoi ========
 *  Purpose:
 *      This function converts strings in decimal or hex format to integers.
 */
static s32 Atoi(char *pszBuf)
{
	char *pch = pszBuf;
	s32 base = 0;

	while (isspace(*pch))
		pch++;

	if (*pch == '-' || *pch == '+') {
		base = 10;
		pch++;
	} else if (*pch && tolower(pch[strlen(pch) - 1]) == 'h') {
		base = 16;
	}

	return simple_strtoul(pch, NULL, base);
}

/*
 *  ======== GetAttrsFromBuf ========
 *  Purpose:
 *      Parse the content of a buffer filled with DSP-side data and
 *      retrieve an object's attributes from it. IMPORTANT: Assume the
 *      buffer has been converted from DSP format to GPP format.
 */
static DSP_STATUS GetAttrsFromBuf(char *pszBuf, u32 ulBufSize,
				 enum DSP_DCDOBJTYPE objType,
				 struct DCD_GENERICOBJ *pGenObj)
{
	DSP_STATUS status = DSP_SOK;
	char seps[] = ", ";
	char *pszCur;
	char *token;
	s32 cLen = 0;
	u32 i = 0;
#ifdef _DB_TIOMAP
	s32 iEntry;
#endif

	DBC_Require(pszBuf != NULL);
	DBC_Require(ulBufSize != 0);
	DBC_Require((objType == DSP_DCDNODETYPE)
		    || (objType == DSP_DCDPROCESSORTYPE));
	DBC_Require(pGenObj != NULL);

	switch (objType) {
	case DSP_DCDNODETYPE:
		/*
		 * Parse COFF sect buffer to retrieve individual tokens used
		 * to fill in object attrs.
		 */
		pszCur = pszBuf;
		token = strsep(&pszCur, seps);

		/* u32 cbStruct */
		pGenObj->objData.nodeObj.ndbProps.cbStruct =
				(u32) Atoi(token);
		token = strsep(&pszCur, seps);

		/* DSP_UUID uiNodeID */
		UUID_UuidFromString(token,
				  &pGenObj->objData.nodeObj.ndbProps.uiNodeID);
		token = strsep(&pszCur, seps);

		/* acName */
		DBC_Require(token);
		cLen = strlen(token);
		if (cLen > DSP_MAXNAMELEN - 1)
			cLen = DSP_MAXNAMELEN - 1;

		strncpy(pGenObj->objData.nodeObj.ndbProps.acName,
			   token, cLen);
		pGenObj->objData.nodeObj.ndbProps.acName[cLen] = '\0';
		token = strsep(&pszCur, seps);
		/* u32 uNodeType */
		pGenObj->objData.nodeObj.ndbProps.uNodeType = Atoi(token);
		token = strsep(&pszCur, seps);
		/* u32 bCacheOnGPP */
		pGenObj->objData.nodeObj.ndbProps.bCacheOnGPP = Atoi(token);
		token = strsep(&pszCur, seps);
		/* DSP_RESOURCEREQMTS dspResourceReqmts */
		pGenObj->objData.nodeObj.ndbProps.dspResourceReqmts.cbStruct =
				(u32) Atoi(token);
		token = strsep(&pszCur, seps);

		pGenObj->objData.nodeObj.ndbProps.dspResourceReqmts.
			uStaticDataSize = Atoi(token);
		token = strsep(&pszCur, seps);
		pGenObj->objData.nodeObj.ndbProps.dspResourceReqmts.
			uGlobalDataSize = Atoi(token);
		token = strsep(&pszCur, seps);
		pGenObj->objData.nodeObj.ndbProps.dspResourceReqmts.
			uProgramMemSize = Atoi(token);
		token = strsep(&pszCur, seps);
		pGenObj->objData.nodeObj.ndbProps.dspResourceReqmts.
			uWCExecutionTime = Atoi(token);
		token = strsep(&pszCur, seps);
		pGenObj->objData.nodeObj.ndbProps.dspResourceReqmts.
			uWCPeriod = Atoi(token);
		token = strsep(&pszCur, seps);

		pGenObj->objData.nodeObj.ndbProps.dspResourceReqmts.
			uWCDeadline = Atoi(token);
		token = strsep(&pszCur, seps);

		pGenObj->objData.nodeObj.ndbProps.dspResourceReqmts.
			uAvgExectionTime = Atoi(token);
		token = strsep(&pszCur, seps);

		pGenObj->objData.nodeObj.ndbProps.dspResourceReqmts.
			uMinimumPeriod = Atoi(token);
		token = strsep(&pszCur, seps);

		/* s32 iPriority */
		pGenObj->objData.nodeObj.ndbProps.iPriority = Atoi(token);
		token = strsep(&pszCur, seps);

		/* u32 uStackSize */
		pGenObj->objData.nodeObj.ndbProps.uStackSize = Atoi(token);
		token = strsep(&pszCur, seps);

		/* u32 uSysStackSize */
		pGenObj->objData.nodeObj.ndbProps.uSysStackSize = Atoi(token);
		token = strsep(&pszCur, seps);

		/* u32 uStackSeg */
		pGenObj->objData.nodeObj.ndbProps.uStackSeg = Atoi(token);
		token = strsep(&pszCur, seps);

		/* u32 uMessageDepth */
		pGenObj->objData.nodeObj.ndbProps.uMessageDepth = Atoi(token);
		token = strsep(&pszCur, seps);

		/* u32 uNumInputStreams */
		pGenObj->objData.nodeObj.ndbProps.uNumInputStreams =
			Atoi(token);
		token = strsep(&pszCur, seps);

		/* u32 uNumOutputStreams */
		pGenObj->objData.nodeObj.ndbProps.uNumOutputStreams =
			Atoi(token);
		token = strsep(&pszCur, seps);

		/* u32 uTimeout */
		pGenObj->objData.nodeObj.ndbProps.uTimeout =
			Atoi(token);
		token = strsep(&pszCur, seps);

		/* char *pstrCreatePhaseFxn */
		DBC_Require(token);
		cLen = strlen(token);
		pGenObj->objData.nodeObj.pstrCreatePhaseFxn =
			MEM_Calloc(cLen + 1, MEM_PAGED);
		if (!pGenObj->objData.nodeObj.pstrCreatePhaseFxn) {
			status = DSP_EMEMORY;
			break;
		}
		strncpy(pGenObj->objData.nodeObj.pstrCreatePhaseFxn,
			token, cLen);
		pGenObj->objData.nodeObj.pstrCreatePhaseFxn[cLen] = '\0';
		token = strsep(&pszCur, seps);

		/* char *pstrExecutePhaseFxn */
		DBC_Require(token);
		cLen = strlen(token);
		pGenObj->objData.nodeObj.pstrExecutePhaseFxn =
			 MEM_Calloc(cLen + 1, MEM_PAGED);
		if (!pGenObj->objData.nodeObj.pstrExecutePhaseFxn) {
			status = DSP_EMEMORY;
			break;
		}
		strncpy(pGenObj->objData.nodeObj.pstrExecutePhaseFxn,
			token, cLen);
		pGenObj->objData.nodeObj.pstrExecutePhaseFxn[cLen] = '\0';
		token = strsep(&pszCur, seps);

		/* char *pstrDeletePhaseFxn */
		DBC_Require(token);
		cLen = strlen(token);
		pGenObj->objData.nodeObj.pstrDeletePhaseFxn =
			MEM_Calloc(cLen + 1, MEM_PAGED);
		if (!pGenObj->objData.nodeObj.pstrDeletePhaseFxn) {
			status = DSP_EMEMORY;
			break;
		}
		strncpy(pGenObj->objData.nodeObj.pstrDeletePhaseFxn,
			token, cLen);
		pGenObj->objData.nodeObj.pstrDeletePhaseFxn[cLen] = '\0';
		token = strsep(&pszCur, seps);

		/* Segment id for message buffers */
		pGenObj->objData.nodeObj.uMsgSegid = Atoi(token);
		token = strsep(&pszCur, seps);

		/* Message notification type */
		pGenObj->objData.nodeObj.uMsgNotifyType = Atoi(token);
		token = strsep(&pszCur, seps);

		/* char *pstrIAlgName */
		if (token) {
			cLen = strlen(token);
			pGenObj->objData.nodeObj.pstrIAlgName =
				MEM_Calloc(cLen + 1, MEM_PAGED);
			if (!pGenObj->objData.nodeObj.pstrIAlgName) {
				status = DSP_EMEMORY;
				break;
			}
			strncpy(pGenObj->objData.nodeObj.pstrIAlgName,
				token, cLen);
			pGenObj->objData.nodeObj.pstrIAlgName[cLen] = '\0';
			token = strsep(&pszCur, seps);
		}

		/* Load type (static, dynamic, or overlay) */
		if (token) {
			pGenObj->objData.nodeObj.usLoadType = Atoi(token);
			token = strsep(&pszCur, seps);
		}

		/* Dynamic load data requirements */
		if (token) {
			pGenObj->objData.nodeObj.ulDataMemSegMask = Atoi(token);
			token = strsep(&pszCur, seps);
		}

		/* Dynamic load code requirements */
		if (token) {
			pGenObj->objData.nodeObj.ulCodeMemSegMask = Atoi(token);
			token = strsep(&pszCur, seps);
		}

		/* Extract node profiles into node properties */
		if (token) {

			pGenObj->objData.nodeObj.ndbProps.uCountProfiles =
				Atoi(token);
			for (i = 0; i < pGenObj->objData.nodeObj.ndbProps.
			    uCountProfiles; i++) {
				token = strsep(&pszCur, seps);
				if (token) {
					/* Heap Size for the node */
					pGenObj->objData.nodeObj.ndbProps.
						aProfiles[i].ulHeapSize =
						Atoi(token);
				}
			}
		}
		token = strsep(&pszCur, seps);
		if (token) {
			pGenObj->objData.nodeObj.ndbProps.uStackSegName =
				(u32)(token);
		}

		break;

	case DSP_DCDPROCESSORTYPE:
		/*
		 * Parse COFF sect buffer to retrieve individual tokens used
		 * to fill in object attrs.
		 */
		pszCur = pszBuf;
		token = strsep(&pszCur, seps);

		pGenObj->objData.procObj.cbStruct = Atoi(token);
		token = strsep(&pszCur, seps);

		pGenObj->objData.procObj.uProcessorFamily = Atoi(token);
		token = strsep(&pszCur, seps);

		pGenObj->objData.procObj.uProcessorType = Atoi(token);
		token = strsep(&pszCur, seps);

		pGenObj->objData.procObj.uClockRate = Atoi(token);
		token = strsep(&pszCur, seps);

		pGenObj->objData.procObj.ulInternalMemSize = Atoi(token);
		token = strsep(&pszCur, seps);

		pGenObj->objData.procObj.ulExternalMemSize = Atoi(token);
		token = strsep(&pszCur, seps);

		pGenObj->objData.procObj.uProcessorID = Atoi(token);
		token = strsep(&pszCur, seps);

		pGenObj->objData.procObj.tyRunningRTOS = Atoi(token);
		token = strsep(&pszCur, seps);

		pGenObj->objData.procObj.nNodeMinPriority = Atoi(token);
		token = strsep(&pszCur, seps);

		pGenObj->objData.procObj.nNodeMaxPriority = Atoi(token);

#ifdef _DB_TIOMAP
		/* Proc object may contain additional(extended) attributes. */
		/* attr must match proc.hxx */
		for (iEntry = 0; iEntry < 7; iEntry++) {
			token = strsep(&pszCur, seps);
			pGenObj->objData.extProcObj.tyTlb[iEntry].ulGppPhys =
				Atoi(token);

			token = strsep(&pszCur, seps);
			pGenObj->objData.extProcObj.tyTlb[iEntry].ulDspVirt =
				Atoi(token);
		}
#endif

		break;

	default:
		status = DSP_EFAIL;
		break;
	}

	/* Check for Memory leak */
	if (status == DSP_EMEMORY) {
		kfree(pGenObj->objData.nodeObj.pstrCreatePhaseFxn);
		kfree(pGenObj->objData.nodeObj.pstrExecutePhaseFxn);
		kfree(pGenObj->objData.nodeObj.pstrDeletePhaseFxn);
	}

	return status;
}

/*
 *  ======== CompressBuffer ========
 *  Purpose:
 *      Compress the DSP buffer, if necessary, to conform to PC format.
 */
static void CompressBuf(char *pszBuf, u32 ulBufSize, s32 cCharSize)
{
	char *p;
	char ch;
	char *q;

	p = pszBuf;
	if (p == NULL)
		return;

	for (q = pszBuf; q < (pszBuf + ulBufSize);) {
		ch = DspChar2GppChar(q, cCharSize);
		if (ch == '\\') {
			q += cCharSize;
			ch = DspChar2GppChar(q, cCharSize);
			switch (ch) {
			case 't':
				*p = '\t';
				break;

			case 'n':
				*p = '\n';
				break;

			case 'r':
				*p = '\r';
				break;

			case '0':
				*p = '\0';
				break;

			default:
				*p = ch;
				break;
			}
		} else {
			*p = ch;
		}
		p++;
		q += cCharSize;
	}

	/* NULL out remainder of buffer. */
	while (p < q)
		*p++ = '\0';
}

/*
 *  ======== DspChar2GppChar ========
 *  Purpose:
 *      Convert DSP char to host GPP char in a portable manner
 */
static char DspChar2GppChar(char *pWord, s32 cDspCharSize)
{
	char ch = '\0';
	char *chSrc;
	s32 i;

	for (chSrc = pWord, i = cDspCharSize; i > 0; i--)
		ch |= *chSrc++;

	return ch;
}

/*
 *  ======== GetDepLibInfo ========
 */
static DSP_STATUS GetDepLibInfo(IN struct DCD_MANAGER *hDcdMgr,
				IN struct DSP_UUID *pUuid,
				IN OUT u16 *pNumLibs,
				OPTIONAL OUT u16 *pNumPersLibs,
				OPTIONAL OUT struct DSP_UUID *pDepLibUuids,
				OPTIONAL OUT bool *pPersistentDepLibs,
				enum NLDR_PHASE phase)
{
	struct DCD_MANAGER *pDcdMgr = hDcdMgr;	/* pointer to DCD manager */
	char *pszCoffBuf = NULL;
	char *pszCur;
	char *pszFileName = NULL;
	struct COD_LIBRARYOBJ *lib = NULL;
	u32 ulAddr = 0;				/* Used by COD_GetSection */
	u32 ulLen = 0;				/* Used by COD_GetSection */
	u32 dwDataSize = COD_MAXPATHLENGTH;
	char seps[] = ", ";
	char *pToken = NULL;
	bool fGetUuids = (pDepLibUuids != NULL);
	u16 nDepLibs = 0;
	DSP_STATUS status = DSP_SOK;

	DBC_Require(cRefs > 0);

	DBC_Require(IsValidHandle(hDcdMgr));
	DBC_Require(pNumLibs != NULL);
	DBC_Require(pUuid != NULL);

	/*  Initialize to 0 dependent libraries, if only counting number of
	 *  dependent libraries */
	if (!fGetUuids) {
		*pNumLibs = 0;
		*pNumPersLibs = 0;
	}

	/* Allocate a buffer for file name */
	pszFileName = MEM_Calloc(dwDataSize, MEM_PAGED);
	if (pszFileName == NULL) {
		status = DSP_EMEMORY;
	} else {
		/* Get the name of the library */
		status = DCD_GetLibraryName(hDcdMgr, pUuid, pszFileName,
			 &dwDataSize, phase, NULL);
	}

	/* Open the library */
	if (DSP_SUCCEEDED(status)) {
		status = COD_Open(pDcdMgr->hCodMgr, pszFileName,
				 COD_NOLOAD, &lib);
	}
	if (DSP_SUCCEEDED(status)) {
		/* Get dependent library section information. */
		status = COD_GetSection(lib, DEPLIBSECT, &ulAddr, &ulLen);

		if (DSP_FAILED(status)) {
			/* Ok, no dependent libraries */
			ulLen = 0;
			status = DSP_SNODEPENDENTLIBS;
		}
	}

	if (DSP_FAILED(status) || !(ulLen > 0))
		goto func_cont;

	/* Allocate zeroed buffer. */
	pszCoffBuf = MEM_Calloc(ulLen + 4, MEM_PAGED);
	if (pszCoffBuf == NULL)
		status = DSP_EMEMORY;

	/* Read section contents. */
	status = COD_ReadSection(lib, DEPLIBSECT, pszCoffBuf, ulLen);
	if (DSP_FAILED(status))
		goto func_cont;

	/* Compress and format DSP buffer to conform to PC format. */
	CompressBuf(pszCoffBuf, ulLen, DSPWORDSIZE);

	/* Read from buffer */
	pszCur = pszCoffBuf;
	while ((pToken = strsep(&pszCur, seps)) && *pToken != '\0') {
		if (fGetUuids) {
			if (nDepLibs >= *pNumLibs) {
				/* Gone beyond the limit */
				break;
			} else {
				/* Retrieve UUID string. */
				UUID_UuidFromString(pToken,
						 &(pDepLibUuids[nDepLibs]));
				/* Is this library persistent? */
				pToken = strsep(&pszCur, seps);
				pPersistentDepLibs[nDepLibs] = Atoi(pToken);
				nDepLibs++;
			}
		} else {
			/* Advanc to next token */
			pToken = strsep(&pszCur, seps);
			if (Atoi(pToken))
				(*pNumPersLibs)++;

			/* Just counting number of dependent libraries */
			(*pNumLibs)++;
		}
	}
func_cont:
	if (lib)
		COD_Close(lib);

	/* Free previously allocated dynamic buffers. */
	kfree(pszFileName);

	kfree(pszCoffBuf);

	return status;
}
