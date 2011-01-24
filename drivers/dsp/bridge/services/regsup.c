/*
 * regsup.c
 *
 * DSP-BIOS Bridge driver support functions for TI OMAP processors.
 *
 * Provide registry support functions.
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
#include <dspbridge/list.h>

/*  ----------------------------------- This */
#include <regsup.h>

struct RegValue {
	struct list_head link;	/* Make it linked to a list */
	char name[MAXREGPATHLENGTH];   /*  Name of a given value entry  */
	u32 dataSize;		/*  Size of the data  */
	void *pData;		/*  Pointer to the actual data  */
};

/*  Pointer to the registry support key  */
static struct LST_LIST regKey, *pRegKey = &regKey;

#if GT_TRACE
extern struct GT_Mask REG_debugMask;	/* GT trace var. */
#endif

/*
 *  ======== regsupInit ========
 *  Purpose:
 *      Initialize the Registry Support module's private state.
 */
bool regsupInit(void)
{
	INIT_LIST_HEAD(&pRegKey->head);
	return true;
}

/*
 *  ======== regsupExit ========
 *  Purpose:
 *      Release all registry support allocations.
 */
void regsupExit(void)
{
	struct RegValue *rv;
	/*  Now go through each entry and free all resources.  */
	while (!LST_IsEmpty(pRegKey)) {
		rv = (struct RegValue *) LST_GetHead(pRegKey);

		kfree(rv->pData);
		kfree(rv);
	}
}

/*
 *  ======== regsupGetValue ========
 *  Purpose:
 *      Get the value of the entry having the given name.
 */
DSP_STATUS regsupGetValue(char *valName, void *pBuf, u32 *dataSize)
{
	DSP_STATUS retVal = DSP_EFAIL;
	struct RegValue *rv = (struct RegValue *) LST_First(pRegKey);

	/*  Need to search through the entries looking for the right one.  */
	while (rv) {
		/*  See if the name matches.  */
		if (strncmp(rv->name, valName, MAXREGPATHLENGTH) == 0) {
			/*  We have a match!  Copy out the data.  */
			memcpy(pBuf, rv->pData, rv->dataSize);

			/*  Get the size for the caller.  */
			*dataSize = rv->dataSize;

			/*  Set our status to good and exit.  */
			retVal = DSP_SOK;
			break;
		}
		rv = (struct RegValue *) LST_Next(pRegKey,
						(struct list_head *) rv);
	}

	if (DSP_SUCCEEDED(retVal)) {
		GT_2trace(REG_debugMask, GT_2CLASS, "G %s DATA %x ", valName,
			  *(u32 *)pBuf);
	} else {
		GT_1trace(REG_debugMask, GT_3CLASS, "G %s FAILED\n", valName);
	}

	return retVal;
}

/*
 *  ======== regsupSetValue ========
 *  Purpose:
 *      Sets the value of the entry having the given name.
 */
DSP_STATUS regsupSetValue(char *valName, void *pBuf, u32 dataSize)
{
	DSP_STATUS retVal = DSP_EFAIL;
	struct RegValue *rv = (struct RegValue *) LST_First(pRegKey);

	GT_2trace(REG_debugMask, GT_2CLASS, "S %s DATA %x ", valName,
		  *(u32 *)pBuf);

	/*  Need to search through the entries looking for the right one.  */
	while (rv) {
		/*  See if the name matches.  */
		if (strncmp(rv->name, valName, MAXREGPATHLENGTH) == 0) {
			/*  Make sure the new data size is the same.  */
			if (dataSize != rv->dataSize) {
				/*  The caller needs a different data size!  */
				kfree(rv->pData);
				rv->pData = MEM_Alloc(dataSize, MEM_NONPAGED);
				if (rv->pData == NULL)
					break;
			}

			/*  We have a match!  Copy out the data.  */
			memcpy(rv->pData, pBuf, dataSize);

			/* Reset datasize - overwrite if new or same */
			rv->dataSize = dataSize;

			/*  Set our status to good and exit.  */
			retVal = DSP_SOK;
			break;
		}
	       rv = (struct RegValue *) LST_Next(pRegKey,
					(struct list_head *) rv);
	}

	/*  See if we found a match or if this is a new entry  */
	if (!rv) {
		/*  No match, need to make a new entry  */
		struct RegValue *new = MEM_Calloc(sizeof(struct RegValue),
						MEM_NONPAGED);

		if (new) {
			strncat(new->name, valName, MAXREGPATHLENGTH - 1);
			new->pData = MEM_Alloc(dataSize, MEM_NONPAGED);
			if (new->pData != NULL) {
				memcpy(new->pData, pBuf, dataSize);
				new->dataSize = dataSize;
				LST_PutTail(pRegKey, (struct list_head *) new);
				retVal = DSP_SOK;
			} else {
				kfree(new);
				retVal = DSP_EMEMORY;
			}
		} else {
			retVal = DSP_EMEMORY;
		}
	}

	return retVal;
}

/*
 *  ======== regsupEnumValue ========
 *  Purpose:
 *      Returns registry "values" and their "data" under a (sub)key.
 */
DSP_STATUS regsupEnumValue(IN u32 dwIndex, IN CONST char *pstrKey,
			   IN OUT char *pstrValue, IN OUT u32 *pdwValueSize,
			   IN OUT char *pstrData, IN OUT u32 *pdwDataSize)
{
	DSP_STATUS retVal = REG_E_INVALIDSUBKEY;
	struct RegValue *rv = (struct RegValue *) LST_First(pRegKey);
       u32 dwKeyLen;
	u32 count = 0;

       DBC_Require(pstrKey);
       dwKeyLen = strlen(pstrKey);

	/*  Need to search through the entries looking for the right one.  */
	while (rv) {
		/*  See if the name matches.  */
		if (strncmp(rv->name, pstrKey, dwKeyLen) == 0 &&
			count++ == dwIndex) {
			/*  We have a match!  Copy out the data.  */
			memcpy(pstrData, rv->pData, rv->dataSize);
			/*  Get the size for the caller.  */
			*pdwDataSize = rv->dataSize;
			*pdwValueSize = strlen(&(rv->name[dwKeyLen]));
			strncpy(pstrValue, &(rv->name[dwKeyLen]),
				    *pdwValueSize + 1);
			GT_3trace(REG_debugMask, GT_2CLASS,
				  "E Key %s, Value %s, Data %x ",
				  pstrKey, pstrValue, *(u32 *)pstrData);
			/*  Set our status to good and exit.  */
			retVal = DSP_SOK;
			break;
		}
	       rv = (struct RegValue *) LST_Next(pRegKey,
						(struct list_head *) rv);
	}

	if (count && DSP_FAILED(retVal))
		retVal = REG_E_NOMOREITEMS;

	return retVal;
}

/*
 *  ======== regsupDeleteValue ========
 */
DSP_STATUS regsupDeleteValue(IN CONST char *pstrValue)
{
	DSP_STATUS retVal = DSP_EFAIL;
	struct RegValue *rv = (struct RegValue *) LST_First(pRegKey);

	while (rv) {
		/*  See if the name matches.  */
		if (strncmp(rv->name, pstrValue, MAXREGPATHLENGTH) == 0) {
			/* We have a match!  Delete this key.  To delete a
			 * key, we free all resources associated with this
			 * key and, if we're not already the last entry in
			 * the array, we copy that entry into this deleted
			 * key.
			 */
			LST_RemoveElem(pRegKey, (struct list_head *)rv);
			kfree(rv->pData);
			kfree(rv);

			/*  Set our status to good and exit...  */
			retVal = DSP_SOK;
			break;
		}
		rv = (struct RegValue *)LST_Next(pRegKey,
				(struct list_head *)rv);
	}
	return retVal;

}

