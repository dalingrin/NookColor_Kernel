/*
 * dbdcd.h
 *
 * DSP-BIOS Bridge driver support functions for TI OMAP processors.
 *
 * Defines the DSP/BIOS Bridge Configuration Database (DCD) API.
 *
 * Copyright (C) 2008 Texas Instruments, Inc.
 *
 * This package is free software; you can redistribute it and/or modify
 * it under the terms of the GNU General Public License version 2 as
 * published by the Free Software Foundation.
 *
 * THIS PACKAGE IS PROVIDED ``AS IS'' AND WITHOUT ANY EXPRESS OR
 * IMPLIED WARRANTIES, INCLUDING, WITHOUT LIMITATION, THE IMPLIED
 * WARRANTIES OF MERCHANTIBILITY AND FITNESS FOR A PARTICULAR PURPOSE.
 */

#ifndef DBDCD_
#define DBDCD_

#include <dspbridge/dbdcddef.h>
#include <dspbridge/host_os.h>
#include <dspbridge/nldrdefs.h>

/*
 *  ======== DCD_AutoRegister ========
 *  Purpose:
 *      This function automatically registers DCD objects specified in a
 *      special COFF section called ".dcd_register"
 *  Parameters:
 *      hDcdMgr:                A DCD manager handle.
 *      pszCoffPath:            Pointer to name of COFF file containing DCD
 *                              objects to be registered.
 *  Returns:
 *      DSP_SOK:                Success.
 *      DSP_EDCDNOAUTOREGISTER: Unable to find auto-registration section.
 *      DSP_EDCDREADSECT:       Unable to read object code section.
 *      DSP_EDCDLOADBASE:       Unable to load code base.
 *      DSP_EHANDLE:            Invalid DCD_HMANAGER handle..
 *  Requires:
 *      DCD initialized.
 *  Ensures:
 *  Note:
 *      Due to the DCD database construction, it is essential for a DCD-enabled
 *      COFF file to contain the right COFF sections, especially
 *      ".dcd_register", which is used for auto registration.
 */
	extern DSP_STATUS DCD_AutoRegister(IN struct DCD_MANAGER *hDcdMgr,
					   IN char *pszCoffPath);

/*
 *  ======== DCD_AutoUnregister ========
 *  Purpose:
 *      This function automatically unregisters DCD objects specified in a
 *      special COFF section called ".dcd_register"
 *  Parameters:
 *      hDcdMgr:                A DCD manager handle.
 *      pszCoffPath:            Pointer to name of COFF file containing
 *                              DCD objects to be unregistered.
 *  Returns:
 *      DSP_SOK:                Success.
 *      DSP_EDCDNOAUTOREGISTER: Unable to find auto-registration section.
 *      DSP_EDCDREADSECT:       Unable to read object code section.
 *      DSP_EDCDLOADBASE:       Unable to load code base.
 *      DSP_EHANDLE:            Invalid DCD_HMANAGER handle..
 *  Requires:
 *      DCD initialized.
 *  Ensures:
 *  Note:
 *      Due to the DCD database construction, it is essential for a DCD-enabled
 *      COFF file to contain the right COFF sections, especially
 *      ".dcd_register", which is used for auto unregistration.
 */
	extern DSP_STATUS DCD_AutoUnregister(IN struct DCD_MANAGER *hDcdMgr,
					     IN char *pszCoffPath);

/*
 *  ======== DCD_CreateManager ========
 *  Purpose:
 *      This function creates a DCD module manager.
 *  Parameters:
 *      pszZlDllName:   Pointer to a DLL name string.
 *      phDcdMgr:       A pointer to a DCD manager handle.
 *  Returns:
 *      DSP_SOK:        Success.
 *      DSP_EMEMORY:    Unable to allocate memory for DCD manager handle.
 *      DSP_EFAIL:      General failure.
 *  Requires:
 *      DCD initialized.
 *      pszZlDllName is non-NULL.
 *      phDcdMgr is non-NULL.
 *  Ensures:
 *      A DCD manager handle is created.
 */
	extern DSP_STATUS DCD_CreateManager(IN char *pszZlDllName,
					    OUT struct DCD_MANAGER **phDcdMgr);

/*
 *  ======== DCD_DestroyManager ========
 *  Purpose:
 *      This function destroys a DCD module manager.
 *  Parameters:
 *      hDcdMgr:        A DCD manager handle.
 *  Returns:
 *      DSP_SOK:        Success.
 *      DSP_EHANDLE:    Invalid DCD manager handle.
 *  Requires:
 *      DCD initialized.
 *  Ensures:
 */
	extern DSP_STATUS DCD_DestroyManager(IN struct DCD_MANAGER *hDcdMgr);

/*
 *  ======== DCD_EnumerateObject ========
 *  Purpose:
 *      This function enumerates currently visible DSP/BIOS Bridge objects
 *      and returns the UUID and type of each enumerated object.
 *  Parameters:
 *      cIndex:             The object enumeration index.
 *      objType:            Type of object to enumerate.
 *      pUuid:              Pointer to a DSP_UUID object.
 *  Returns:
 *      DSP_SOK:            Success.
 *      DSP_EFAIL:          Unable to enumerate through the DCD database.
 *      DSP_SENUMCOMPLETE:  Enumeration completed. This is not an error code.
 *  Requires:
 *      DCD initialized.
 *      pUuid is a valid pointer.
 *  Ensures:
 *  Details:
 *      This function can be used in conjunction with DCD_GetObjectDef to
 *      retrieve object properties.
 */
	extern DSP_STATUS DCD_EnumerateObject(IN s32 cIndex,
					      IN enum DSP_DCDOBJTYPE objType,
					      OUT struct DSP_UUID *pUuid);

/*
 *  ======== DCD_Exit ========
 *  Purpose:
 *      This function cleans up the DCD module.
 *  Parameters:
 *  Returns:
 *  Requires:
 *      DCD initialized.
 *  Ensures:
 */
	extern void DCD_Exit(void);

/*
 *  ======== DCD_GetDepLibs ========
 *  Purpose:
 *      Given the uuid of a library and size of array of uuids, this function
 *      fills the array with the uuids of all dependent libraries of the input
 *      library.
 *  Parameters:
 *      hDcdMgr: A DCD manager handle.
 *      pUuid: Pointer to a DSP_UUID for a library.
 *      numLibs: Size of uuid array (number of library uuids).
 *      pDepLibUuids: Array of dependent library uuids to be filled in.
 *      pPersistentDepLibs: Array indicating if corresponding lib is persistent.
 *      phase: phase to obtain correct input library
 *  Returns:
 *      DSP_SOK: Success.
 *      DSP_EMEMORY: Memory allocation failure.
 *      DSP_EDCDREADSECT: Failure to read section containing library info.
 *      DSP_EFAIL: General failure.
 *  Requires:
 *      DCD initialized.
 *      Valid hDcdMgr.
 *      pUuid != NULL
 *      pDepLibUuids != NULL.
 *  Ensures:
 */
	extern DSP_STATUS DCD_GetDepLibs(IN struct DCD_MANAGER *hDcdMgr,
					 IN struct DSP_UUID *pUuid,
					 u16 numLibs,
					 OUT struct DSP_UUID *pDepLibUuids,
					 OUT bool *pPersistentDepLibs,
					 IN enum NLDR_PHASE phase);

/*
 *  ======== DCD_GetNumDepLibs ========
 *  Purpose:
 *      Given the uuid of a library, determine its number of dependent
 *      libraries.
 *  Parameters:
 *      hDcdMgr:        A DCD manager handle.
 *      pUuid:          Pointer to a DSP_UUID for a library.
 *      pNumLibs:       Size of uuid array (number of library uuids).
 *      pNumPersLibs:   number of persistent dependent library.
 *      phase:          Phase to obtain correct input library
 *  Returns:
 *      DSP_SOK: Success.
 *      DSP_EMEMORY: Memory allocation failure.
 *      DSP_EDCDREADSECT: Failure to read section containing library info.
 *      DSP_EFAIL: General failure.
 *  Requires:
 *      DCD initialized.
 *      Valid hDcdMgr.
 *      pUuid != NULL
 *      pNumLibs != NULL.
 *  Ensures:
 */
	extern DSP_STATUS DCD_GetNumDepLibs(IN struct DCD_MANAGER *hDcdMgr,
					    IN struct DSP_UUID *pUuid,
					    OUT u16 *pNumLibs,
					    OUT u16 *pNumPersLibs,
					    IN enum NLDR_PHASE phase);

/*
 *  ======== DCD_GetLibraryName ========
 *  Purpose:
 *      This function returns the name of a (dynamic) library for a given
 *      UUID.
 *  Parameters:
 *      hDcdMgr: A DCD manager handle.
 *      pUuid:          Pointer to a DSP_UUID that represents a unique DSP/BIOS
 *                      Bridge object.
 *      pstrLibName: Buffer to hold library name.
 *      pdwSize: Contains buffer size. Set to string size on output.
 *      phase:          Which phase to load
 *      fPhaseSplit:    Are phases in multiple libraries
 *  Returns:
 *      DSP_SOK: Success.
 *      DSP_EFAIL: General failure.
 *  Requires:
 *      DCD initialized.
 *      Valid hDcdMgr.
 *      pstrLibName != NULL.
 *      pUuid != NULL
 *      pdwSize != NULL.
 *  Ensures:
 */
	extern DSP_STATUS DCD_GetLibraryName(IN struct DCD_MANAGER *hDcdMgr,
					     IN struct DSP_UUID *pUuid,
					     IN OUT char *pstrLibName,
					     IN OUT u32 *pdwSize,
					     IN enum NLDR_PHASE phase,
					     OUT bool *fPhaseSplit);

/*
 *  ======== DCD_GetObjectDef ========
 *  Purpose:
 *      This function returns the properties/attributes of a DSP/BIOS Bridge
 *      object.
 *  Parameters:
 *      hDcdMgr:            A DCD manager handle.
 *      pUuid:              Pointer to a DSP_UUID that represents a unique
 *                          DSP/BIOS Bridge object.
 *      objType:            The type of DSP/BIOS Bridge object to be
 *                          referenced (node, processor, etc).
 *      pObjDef:            Pointer to an object definition structure. A
 *                          union of various possible DCD object types.
 *  Returns:
 *      DSP_SOK: Success.
 *      DSP_EDCDPARSESECT:  Unable to parse content of object code section.
 *      DSP_EDCDREADSECT:   Unable to read object code section.
 *      DSP_EDCDGETSECT:    Unable to access object code section.
 *      DSP_EDCDLOADBASE:   Unable to load code base.
 *      DSP_EFAIL:          General failure.
 *      DSP_EHANDLE:        Invalid DCD_HMANAGER handle.
 *  Requires:
 *      DCD initialized.
 *      pObjUuid is non-NULL.
 *      pObjDef is non-NULL.
 *  Ensures:
 */
	extern DSP_STATUS DCD_GetObjectDef(IN struct DCD_MANAGER *hDcdMgr,
					   IN struct DSP_UUID *pObjUuid,
					   IN enum DSP_DCDOBJTYPE objType,
					   OUT struct DCD_GENERICOBJ *pObjDef);

/*
 *  ======== DCD_GetObjects ========
 *  Purpose:
 *      This function finds all DCD objects specified in a special
 *      COFF section called ".dcd_register", and for each object,
 *      call a "register" function.  The "register" function may perform
 *      various actions, such as 1) register nodes in the node database, 2)
 *      unregister nodes from the node database, and 3) add overlay nodes.
 *  Parameters:
 *      hDcdMgr:                A DCD manager handle.
 *      pszCoffPath:            Pointer to name of COFF file containing DCD
 *                              objects.
 *      registerFxn:            Callback fxn to be applied on each located
 *                              DCD object.
 *      handle:                 Handle to pass to callback.
 *  Returns:
 *      DSP_SOK:                Success.
 *      DSP_EDCDNOAUTOREGISTER: Unable to find .dcd_register section.
 *      DSP_EDCDREADSECT:       Unable to read object code section.
 *      DSP_EDCDLOADBASE:       Unable to load code base.
 *      DSP_EHANDLE:            Invalid DCD_HMANAGER handle..
 *  Requires:
 *      DCD initialized.
 *  Ensures:
 *  Note:
 *      Due to the DCD database construction, it is essential for a DCD-enabled
 *      COFF file to contain the right COFF sections, especially
 *      ".dcd_register", which is used for auto registration.
 */
	extern DSP_STATUS DCD_GetObjects(IN struct DCD_MANAGER *hDcdMgr,
					 IN char *pszCoffPath,
					 DCD_REGISTERFXN registerFxn,
					 void *handle);

/*
 *  ======== DCD_Init ========
 *  Purpose:
 *      This function initializes DCD.
 *  Parameters:
 *  Returns:
 *      FALSE:  Initialization failed.
 *      TRUE:   Initialization succeeded.
 *  Requires:
 *  Ensures:
 *      DCD initialized.
 */
	extern bool DCD_Init(void);

/*
 *  ======== DCD_RegisterObject ========
 *  Purpose:
 *      This function registers a DSP/BIOS Bridge object in the DCD database.
 *  Parameters:
 *      pUuid:          Pointer to a DSP_UUID that identifies a DSP/BIOS
 *                      Bridge object.
 *      objType:        Type of object.
 *      pszPathName:    Path to the object's COFF file.
 *  Returns:
 *      DSP_SOK:        Success.
 *      DSP_EFAIL:      Failed to register object.
 *  Requires:
 *      DCD initialized.
 *      pUuid and szPathName are non-NULL values.
 *      objType is a valid type value.
 *  Ensures:
 */
	extern DSP_STATUS DCD_RegisterObject(IN struct DSP_UUID *pUuid,
					     IN enum DSP_DCDOBJTYPE objType,
					     IN char *pszPathName);

/*
 *  ======== DCD_UnregisterObject ========
 *  Purpose:
 *      This function de-registers a valid DSP/BIOS Bridge object from the DCD
 *      database.
 *  Parameters:
 *      pUuid:      Pointer to a DSP_UUID that identifies a DSP/BIOS Bridge
 *                  object.
 *      objType:    Type of object.
 *  Returns:
 *      DSP_SOK:    Success.
 *      DSP_EFAIL:  Unable to de-register the specified object.
 *  Requires:
 *      DCD initialized.
 *      pUuid is a non-NULL value.
 *      objType is a valid type value.
 *  Ensures:
 */
	extern DSP_STATUS DCD_UnregisterObject(IN struct DSP_UUID *pUuid,
					       IN enum DSP_DCDOBJTYPE objType);

#endif				/* _DBDCD_H */
