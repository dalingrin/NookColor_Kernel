/*
 * cfg.h
 *
 * DSP-BIOS Bridge driver support functions for TI OMAP processors.
 *
 * PM Configuration module.
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

#ifndef CFG_
#define CFG_
#include <dspbridge/host_os.h>
#include <dspbridge/cfgdefs.h>

/*
 *  ======== CFG_Exit ========
 *  Purpose:
 *      Discontinue usage of the CFG module.
 *  Parameters:
 *  Returns:
 *  Requires:
 *      CFG_Init(void) was previously called.
 *  Ensures:
 *      Resources acquired in CFG_Init(void) are freed.
 */
	extern void CFG_Exit(void);

/*
 *  ======== CFG_GetAutoStart ========
 *  Purpose:
 *      Retreive the autostart mask, if any, for this board.
 *  Parameters:
 *      hDevNode:       Handle to the DevNode who's WMD we are querying.
 *      pdwAutoStart:   Ptr to location for 32 bit autostart mask.
 *  Returns:
 *      DSP_SOK:                Success.
 *      CFG_E_INVALIDHDEVNODE:  hDevNode is invalid.
 *      CFG_E_RESOURCENOTAVAIL: Unable to retreive resource.
 *  Requires:
 *      CFG initialized.
 *  Ensures:
 *      DSP_SOK:        *pdwAutoStart contains autostart mask for this devnode.
 */
	extern DSP_STATUS CFG_GetAutoStart(IN struct CFG_DEVNODE *hDevNode,
					   OUT u32 *pdwAutoStart);

/*
 *  ======== CFG_GetCDVersion ========
 *  Purpose:
 *      Retrieves the version of the PM Class Driver.
 *  Parameters:
 *      pdwVersion: Ptr to u32 to contain version number upon return.
 *  Returns:
 *      DSP_SOK:    Success.  pdwVersion contains Class Driver version in
 *                  the form: 0xAABBCCDD where AABB is Major version and
 *                  CCDD is Minor.
 *      DSP_EFAIL:  Failure.
 *  Requires:
 *      CFG initialized.
 *  Ensures:
 *      DSP_SOK:    Success.
 *      else:       *pdwVersion is NULL.
 */
	extern DSP_STATUS CFG_GetCDVersion(OUT u32 *pdwVersion);

/*
 *  ======== CFG_GetDevObject ========
 *  Purpose:
 *      Retrieve the Device Object handle for a given devnode.
 *  Parameters:
 *      hDevNode:       Platform's DevNode handle from which to retrieve value.
 *      pdwValue:       Ptr to location to store the value.
 *  Returns:
 *      DSP_SOK:                Success.
 *      CFG_E_INVALIDHDEVNODE:  hDevNode is invalid.
 *      CFG_E_INVALIDPOINTER:   phDevObject is invalid.
 *      CFG_E_RESOURCENOTAVAIL: The resource is not available.
 *  Requires:
 *      CFG initialized.
 *  Ensures:
 *      DSP_SOK:    *pdwValue is set to the retrieved u32.
 *      else:       *pdwValue is set to 0L.
 */
	extern DSP_STATUS CFG_GetDevObject(IN struct CFG_DEVNODE *hDevNode,
					   OUT u32 *pdwValue);

/*
 *  ======== CFG_GetDSPResources ========
 *  Purpose:
 *      Get the DSP resources available to a given device.
 *  Parameters:
 *      hDevNode:       Handle to the DEVNODE who's resources we are querying.
 *      pDSPResTable:   Ptr to a location to store the DSP resource table.
 *  Returns:
 *      DSP_SOK:                On success.
 *      CFG_E_INVALIDHDEVNODE:  hDevNode is invalid.
 *      CFG_E_RESOURCENOTAVAIL: The DSP Resource information is not
 *                              available
 *  Requires:
 *      CFG initialized.
 *  Ensures:
 *      DSP_SOK:    pDSPResTable points to a filled table of resources allocated
 *                  for the specified WMD.
 */
	extern DSP_STATUS CFG_GetDSPResources(IN struct CFG_DEVNODE *hDevNode,
				      OUT struct CFG_DSPRES *pDSPResTable);


/*
 *  ======== CFG_GetExecFile ========
 *  Purpose:
 *      Retreive the default executable, if any, for this board.
 *  Parameters:
 *      hDevNode:       Handle to the DevNode who's WMD we are querying.
 *      cBufSize:       Size of buffer.
 *      pstrExecFile:   Ptr to character buf to hold ExecFile.
 *  Returns:
 *      DSP_SOK:                Success.
 *      CFG_E_INVALIDHDEVNODE:  hDevNode is invalid.
 *      CFG_E_INVALIDPOINTER:   pstrExecFile is invalid.
 *      CFG_E_RESOURCENOTAVAIL: The resource is not available.
 *  Requires:
 *      CFG initialized.
 *  Ensures:
 *      DSP_SOK:    Not more than cBufSize bytes were copied into pstrExecFile,
 *                  and *pstrExecFile contains default executable for this
 *                  devnode.
 */
	extern DSP_STATUS CFG_GetExecFile(IN struct CFG_DEVNODE *hDevNode,
					  IN u32 cBufSize,
					  OUT char *pstrExecFile);

/*
 *  ======== CFG_GetHostResources ========
 *  Purpose:
 *      Get the Host PC allocated resources assigned to a given device.
 *  Parameters:
 *      hDevNode:       Handle to the DEVNODE who's resources we are querying.
 *      pHostResTable:  Ptr to a location to store the host resource table.
 *  Returns:
 *      DSP_SOK:                On success.
 *      CFG_E_INVALIDPOINTER:   pHostResTable is invalid.
 *      CFG_E_INVALIDHDEVNODE:  hDevNode is invalid.
 *      CFG_E_RESOURCENOTAVAIL: The resource is not available.
 *  Requires:
 *      CFG initialized.
 *  Ensures:
 *      DSP_SOK:    pHostResTable points to a filled table of resources
 *                  allocated for the specified WMD.
 *
 */
	extern DSP_STATUS CFG_GetHostResources(IN struct CFG_DEVNODE *hDevNode,
				       OUT struct CFG_HOSTRES *pHostResTable);

/*
 *  ======== CFG_GetObject ========
 *  Purpose:
 *      Retrieve the Driver Object handle From the Registry
 *  Parameters:
 *      pdwValue:   Ptr to location to store the value.
 *      dwType      Type of Object to Get
 *  Returns:
 *      DSP_SOK:    Success.
 *  Requires:
 *      CFG initialized.
 *  Ensures:
 *      DSP_SOK:    *pdwValue is set to the retrieved u32(non-Zero).
 *      else:       *pdwValue is set to 0L.
 */
	extern DSP_STATUS CFG_GetObject(OUT u32 *pdwValue, u32 dwType);

/*
 *  ======== CFG_GetPerfValue ========
 *  Purpose:
 *      Retrieve a flag indicating whether PERF should log statistics for the
 *      PM class driver.
 *  Parameters:
 *      pfEnablePerf:   Location to store flag.  0 indicates the key was
 *                      not found, or had a zero value.  A nonzero value
 *                      means the key was found and had a nonzero value.
 *  Returns:
 *  Requires:
 *      pfEnablePerf != NULL;
 *  Ensures:
 */
	extern void CFG_GetPerfValue(OUT bool *pfEnablePerf);

/*
 *  ======== CFG_GetWMDFileName ========
 *  Purpose:
 *    Get the mini-driver file name for a given device.
 *  Parameters:
 *      hDevNode:       Handle to the DevNode who's WMD we are querying.
 *      cBufSize:       Size of buffer.
 *      pWMDFileName:   Ptr to a character buffer to hold the WMD filename.
 *  Returns:
 *      DSP_SOK:                On success.
 *      CFG_E_INVALIDHDEVNODE:  hDevNode is invalid.
 *      CFG_E_RESOURCENOTAVAIL: The filename is not available.
 *  Requires:
 *      CFG initialized.
 *  Ensures:
 *      DSP_SOK:        Not more than cBufSize bytes were copied
 *                      into pWMDFileName.
 *
 */
	extern DSP_STATUS CFG_GetWMDFileName(IN struct CFG_DEVNODE *hDevNode,
					     IN u32 cBufSize,
					     OUT char *pWMDFileName);

/*
 *  ======== CFG_GetZLFile ========
 *  Purpose:
 *      Retreive the ZLFile, if any, for this board.
 *  Parameters:
 *      hDevNode:       Handle to the DevNode who's WMD we are querying.
 *      cBufSize:       Size of buffer.
 *      pstrZLFileName: Ptr to character buf to hold ZLFileName.
 *  Returns:
 *      DSP_SOK:                Success.
 *      CFG_E_INVALIDPOINTER:   pstrZLFileName is invalid.
 *      CFG_E_INVALIDHDEVNODE:  hDevNode is invalid.
 *      CFG_E_RESOURCENOTAVAIL: couldn't find the ZLFileName.
 *  Requires:
 *      CFG initialized.
 *  Ensures:
 *      DSP_SOK:    Not more than cBufSize bytes were copied into
 *                  pstrZLFileName, and *pstrZLFileName contains ZLFileName
 *                  for this devnode.
 */
	extern DSP_STATUS CFG_GetZLFile(IN struct CFG_DEVNODE *hDevNode,
					IN u32 cBufSize,
					OUT char *pstrZLFileName);

/*
 *  ======== CFG_Init ========
 *  Purpose:
 *      Initialize the CFG module's private state.
 *  Parameters:
 *  Returns:
 *      TRUE if initialized; FALSE if error occured.
 *  Requires:
 *  Ensures:
 *      A requirement for each of the other public CFG functions.
 */
	extern bool CFG_Init(void);

/*
 *  ======== CFG_SetDevObject ========
 *  Purpose:
 *      Store the Device Object handle for a given devnode.
 *  Parameters:
 *      hDevNode:   Platform's DevNode handle we are storing value with.
 *      dwValue:    Arbitrary value to store.
 *  Returns:
 *      DSP_SOK:                Success.
 *      CFG_E_INVALIDHDEVNODE:  hDevNode is invalid.
 *      DSP_EFAIL:              Internal Error.
 *  Requires:
 *      CFG initialized.
 *  Ensures:
 *      DSP_SOK:    The Private u32 was successfully set.
 */
	extern DSP_STATUS CFG_SetDevObject(IN struct CFG_DEVNODE *hDevNode,
					   IN u32 dwValue);

/*
 *  ======== CFG_SetDrvObject ========
 *  Purpose:
 *      Store the Driver Object handle.
 *  Parameters:
 *      dwValue:        Arbitrary value to store.
 *      dwType          Type of Object to Store
 *  Returns:
 *      DSP_SOK:        Success.
 *      DSP_EFAIL:      Internal Error.
 *  Requires:
 *      CFG initialized.
 *  Ensures:
 *      DSP_SOK:        The Private u32 was successfully set.
 */
	extern DSP_STATUS CFG_SetObject(IN u32 dwValue, IN u32 dwType);

#endif				/* CFG_ */
