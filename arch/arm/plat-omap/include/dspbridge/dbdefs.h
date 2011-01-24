/*
 * dbdefs.h
 *
 * DSP-BIOS Bridge driver support functions for TI OMAP processors.
 *
 * Global definitions and constants for DSP/BIOS Bridge.
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

#ifndef DBDEFS_
#define DBDEFS_

#include <linux/types.h>

#include <dspbridge/dbtype.h>		/* GPP side type definitions */
#include <dspbridge/std.h>		/* DSP/BIOS type definitions */
#include <dspbridge/rms_sh.h>		/* Types shared between GPP and DSP */

#define PG_SIZE_4K 4096
#define PG_MASK(pg_size) (~((pg_size)-1))
#define PG_ALIGN_LOW(addr, pg_size) ((addr) & PG_MASK(pg_size))
#define PG_ALIGN_HIGH(addr, pg_size) (((addr)+(pg_size)-1) & PG_MASK(pg_size))

/* API return value and calling convention */
#define DBAPI                       DSP_STATUS

/* Infinite time value for the uTimeout parameter to DSPStream_Select() */
#define DSP_FOREVER                 (-1)

/* Maximum length of node name, used in DSP_NDBPROPS */
#define DSP_MAXNAMELEN              32

/* uNotifyType values for the RegisterNotify() functions. */
#define DSP_SIGNALEVENT             0x00000001

/* Types of events for processors */
#define DSP_PROCESSORSTATECHANGE    0x00000001
#define DSP_PROCESSORATTACH         0x00000002
#define DSP_PROCESSORDETACH         0x00000004
#define DSP_PROCESSORRESTART        0x00000008

/* DSP exception events (DSP/BIOS and DSP MMU fault) */
#define DSP_MMUFAULT                0x00000010
#define DSP_SYSERROR                0x00000020
#define DSP_EXCEPTIONABORT          0x00000300
#define DSP_PWRERROR                0x00000080
#define DSP_WDTOVERFLOW	0x00000040

/* IVA exception events (IVA MMU fault) */
#define IVA_MMUFAULT                0x00000040
/* Types of events for nodes */
#define DSP_NODESTATECHANGE         0x00000100
#define DSP_NODEMESSAGEREADY        0x00000200

/* Types of events for streams */
#define DSP_STREAMDONE              0x00001000
#define DSP_STREAMIOCOMPLETION      0x00002000

/* Handle definition representing the GPP node in DSPNode_Connect() calls */
#define DSP_HGPPNODE                0xFFFFFFFF

/* Node directions used in DSPNode_Connect() */
#define DSP_TONODE                  1
#define DSP_FROMNODE                2

/* Define Node Minimum and Maximum Priorities */
#define DSP_NODE_MIN_PRIORITY       1
#define DSP_NODE_MAX_PRIORITY       15

/* Pre-Defined Message Command Codes available to user: */
#define DSP_RMSUSERCODESTART RMS_USER	/* Start of RMS user cmd codes */
/* end of user codes */
#define DSP_RMSUSERCODEEND (RMS_USER + RMS_MAXUSERCODES);
#define DSP_RMSBUFDESC RMS_BUFDESC	/* MSG contains SM buffer description */

/* Shared memory identifier for MEM segment named "SHMSEG0" */
#define DSP_SHMSEG0     (u32)(-1)

/* Processor ID numbers */
#define DSP_UNIT    0
#define IVA_UNIT    1

#define DSPWORD       unsigned char
#define DSPWORDSIZE     sizeof(DSPWORD)

/* Success & Failure macros  */
#define DSP_SUCCEEDED(Status)      likely((s32)(Status) >= 0)
#define DSP_FAILED(Status)         unlikely((s32)(Status) < 0)

/* Power control enumerations */
#define PROC_PWRCONTROL             0x8070

#define PROC_PWRMGT_ENABLE          (PROC_PWRCONTROL + 0x3)
#define PROC_PWRMGT_DISABLE         (PROC_PWRCONTROL + 0x4)

/* Bridge Code Version */
#define BRIDGE_VERSION_CODE         333

#define    MAX_PROFILES     16

/* DSP chip type */
#define DSPTYPE_64	0x99

/* Types defined for 'Bridge API */
	typedef u32 DSP_STATUS;	/* API return code type         */


/* Handy Macros */
#define IsValidProcEvent(x) (((x) == 0) || (((x) & (DSP_PROCESSORSTATECHANGE | \
				DSP_PROCESSORATTACH | \
				DSP_PROCESSORDETACH | \
				DSP_PROCESSORRESTART | \
				DSP_NODESTATECHANGE | \
				DSP_STREAMDONE | \
				DSP_STREAMIOCOMPLETION | \
				DSP_MMUFAULT | \
				DSP_SYSERROR | \
				DSP_PWRERROR | \
				DSP_WDTOVERFLOW)) && \
				!((x) & ~(DSP_PROCESSORSTATECHANGE | \
				DSP_PROCESSORATTACH | \
				DSP_PROCESSORDETACH | \
				DSP_PROCESSORRESTART | \
				DSP_NODESTATECHANGE | \
				DSP_STREAMDONE | \
				DSP_STREAMIOCOMPLETION | \
				DSP_MMUFAULT | \
				DSP_SYSERROR | \
				DSP_PWRERROR | \
				DSP_WDTOVERFLOW))))

#define IsValidNodeEvent(x)    (((x) == 0) || (((x) & (DSP_NODESTATECHANGE | \
				DSP_NODEMESSAGEREADY)) && \
				!((x) & ~(DSP_NODESTATECHANGE | \
				DSP_NODEMESSAGEREADY))))

#define IsValidStrmEvent(x)     (((x) == 0) || (((x) & (DSP_STREAMDONE | \
				DSP_STREAMIOCOMPLETION)) && \
				!((x) & ~(DSP_STREAMDONE | \
				DSP_STREAMIOCOMPLETION))))

#define IsValidNotifyMask(x)   ((x) & DSP_SIGNALEVENT)

/* The Node UUID structure */
	struct DSP_UUID {
		u32 ulData1;
		u16 usData2;
		u16 usData3;
		u8 ucData4;
		u8 ucData5;
		u8 ucData6[6];
	};

/* DCD types */
	enum DSP_DCDOBJTYPE {
		DSP_DCDNODETYPE,
		DSP_DCDPROCESSORTYPE,
		DSP_DCDLIBRARYTYPE,
		DSP_DCDCREATELIBTYPE,
		DSP_DCDEXECUTELIBTYPE,
		DSP_DCDDELETELIBTYPE,
		/* DSP_DCDMAXOBJTYPE is meant to be the last DCD object type */
		DSP_DCDMAXOBJTYPE
	} ;

/* Processor states */
	enum DSP_PROCSTATE {
		PROC_STOPPED,
		PROC_LOADED,
		PROC_RUNNING,
		PROC_ERROR
	} ;

/*
 *  Node types: Message node, task node, xDAIS socket node, and
 *  device node. _NODE_GPP is used when defining a stream connection
 *  between a task or socket node and the GPP.
 *
 */
	enum NODE_TYPE {
		NODE_DEVICE,
		NODE_TASK,
		NODE_DAISSOCKET,
		NODE_MESSAGE,
		NODE_GPP
	} ;

/*
 *  ======== NODE_STATE ========
 *  Internal node states.
 */
	enum NODE_STATE {
		NODE_ALLOCATED,
		NODE_CREATED,
		NODE_RUNNING,
		NODE_PAUSED,
		NODE_DONE,
		NODE_CREATING,
		NODE_STARTING,
		NODE_PAUSING,
		NODE_TERMINATING,
		NODE_DELETING,
	} ;

/* Stream states */
	enum DSP_STREAMSTATE {
		STREAM_IDLE,
		STREAM_READY,
		STREAM_PENDING,
		STREAM_DONE
	} ;

/* Stream connect types */
	enum DSP_CONNECTTYPE {
		CONNECTTYPE_NODEOUTPUT,
		CONNECTTYPE_GPPOUTPUT,
		CONNECTTYPE_NODEINPUT,
		CONNECTTYPE_GPPINPUT
	} ;

/* Stream mode types */
	enum DSP_STRMMODE {
		STRMMODE_PROCCOPY, /* Processor(s) copy stream data payloads */
		STRMMODE_ZEROCOPY, /* Strm buffer ptrs swapped no data copied */
		STRMMODE_LDMA,	/* Local DMA : OMAP's System-DMA device */
		STRMMODE_RDMA	/* Remote DMA: OMAP's DSP-DMA device */
	} ;

/* Resource Types */
	enum DSP_RESOURCEINFOTYPE {
		DSP_RESOURCE_DYNDARAM = 0,
		DSP_RESOURCE_DYNSARAM,
		DSP_RESOURCE_DYNEXTERNAL,
		DSP_RESOURCE_DYNSRAM,
		DSP_RESOURCE_PROCLOAD
	} ;

/* Memory Segment Types */
	enum DSP_MEMTYPE {
		DSP_DYNDARAM = 0,
		DSP_DYNSARAM,
		DSP_DYNEXTERNAL,
		DSP_DYNSRAM
	} ;

/* Memory Flush Types */
       enum DSP_FLUSHTYPE {
		PROC_INVALIDATE_MEM = 0,
		PROC_WRITEBACK_MEM,
		PROC_WRITEBACK_INVALIDATE_MEM,
		PROC_WRBK_INV_ALL,
	} ;

/* Memory Segment Status Values */
	struct DSP_MEMSTAT {
		u32 ulSize;
		u32 ulTotalFreeSize;
		u32 ulLenMaxFreeBlock;
		u32 ulNumFreeBlocks;
		u32 ulNumAllocBlocks;
	} ;

/* Processor Load information Values */
	 struct DSP_PROCLOADSTAT {
		u32 uCurrLoad;
		u32 uPredictedLoad;
		u32 uCurrDspFreq;
		u32 uPredictedFreq;
	} ;

/* Attributes for STRM connections between nodes */
	struct DSP_STRMATTR {
		u32 uSegid;	/* Memory segment on DSP to allocate buffers */
		u32 uBufsize;	/* Buffer size (DSP words) */
		u32 uNumBufs;	/* Number of buffers */
		u32 uAlignment;	/* Buffer alignment */
		u32 uTimeout;	/* Timeout for blocking STRM calls */
		enum DSP_STRMMODE lMode;	/* mode of stream when opened */
		/* DMA chnl id if DSP_STRMMODE is LDMA or RDMA */
		u32 uDMAChnlId;
		u32 uDMAPriority;  /* DMA channel priority 0=lowest, >0=high */
	} ;

/* The DSP_CBDATA structure */
	struct DSP_CBDATA {
		u32 cbData;
		u8 cData[1];
	} ;

/* The DSP_MSG structure */
	struct DSP_MSG {
		u32 dwCmd;
		u32 dwArg1;
		u32 dwArg2;
	} ;

/* The DSP_RESOURCEREQMTS structure for node's resource requirements  */
	struct DSP_RESOURCEREQMTS {
		u32 cbStruct;
		u32 uStaticDataSize;
		u32 uGlobalDataSize;
		u32 uProgramMemSize;
		u32 uWCExecutionTime;
		u32 uWCPeriod;
		u32 uWCDeadline;
		u32 uAvgExectionTime;
		u32 uMinimumPeriod;
	} ;

/*
 * The DSP_STREAMCONNECT structure describes a stream connection
 * between two nodes, or between a node and the GPP
 */
	struct DSP_STREAMCONNECT {
		u32 cbStruct;
		enum DSP_CONNECTTYPE lType;
		u32 uThisNodeStreamIndex;
		void *hConnectedNode;
		struct DSP_UUID uiConnectedNodeID;
		u32 uConnectedNodeStreamIndex;
	} ;

	struct DSP_NODEPROFS {
		u32 ulHeapSize;
	} ;

/* The DSP_NDBPROPS structure reports the attributes of a node */
	struct DSP_NDBPROPS {
		u32 cbStruct;
		struct DSP_UUID uiNodeID;
		char acName[DSP_MAXNAMELEN];
		enum NODE_TYPE uNodeType;
		u32 bCacheOnGPP;
		struct DSP_RESOURCEREQMTS dspResourceReqmts;
		s32 iPriority;
		u32 uStackSize;
		u32 uSysStackSize;
		u32 uStackSeg;
		u32 uMessageDepth;
		u32 uNumInputStreams;
		u32 uNumOutputStreams;
		u32 uTimeout;
		u32 uCountProfiles;	/* Number of supported profiles */
		/* Array of profiles */
		struct DSP_NODEPROFS aProfiles[MAX_PROFILES];
		u32 uStackSegName; /* Stack Segment Name */
	} ;

	/* The DSP_NODEATTRIN structure describes the attributes of a
	 * node client */
	struct DSP_NODEATTRIN {
		u32 cbStruct;
		s32 iPriority;
		u32 uTimeout;
		u32    uProfileID;
		/* Reserved, for Bridge Internal use only */
		u32    uHeapSize;
		void *pGPPVirtAddr; /* Reserved, for Bridge Internal use only */
	} ;

	/* The DSP_NODEINFO structure is used to retrieve information
	 * about a node */
	struct DSP_NODEINFO {
		u32 cbStruct;
		struct DSP_NDBPROPS nbNodeDatabaseProps;
		u32 uExecutionPriority;
		enum NODE_STATE nsExecutionState;
		void *hDeviceOwner;
		u32 uNumberStreams;
		struct DSP_STREAMCONNECT scStreamConnection[16];
		u32 uNodeEnv;
	} ;

	/* The DSP_NODEATTR structure describes the attributes of a node */
	struct DSP_NODEATTR {
		u32 cbStruct;
		struct DSP_NODEATTRIN inNodeAttrIn;
		u32 uInputs;
		u32 uOutputs;
		struct DSP_NODEINFO iNodeInfo;
	} ;

/*
 *  Notification type: either the name of an opened event, or an event or
 *  window handle.
 */
	struct DSP_NOTIFICATION {
		char *psName;
		HANDLE handle;
	} ;

/* The DSP_PROCESSORATTRIN structure describes the attributes of a processor */
	struct DSP_PROCESSORATTRIN{
		u32 cbStruct;
		u32 uTimeout;
	} ;
/*
 * The DSP_PROCESSORINFO structure describes basic capabilities of a
 * DSP processor
 */
	struct DSP_PROCESSORINFO {
		u32 cbStruct;
		int uProcessorFamily;
		int uProcessorType;
		u32 uClockRate;
		u32 ulInternalMemSize;
		u32 ulExternalMemSize;
		u32 uProcessorID;
		int tyRunningRTOS;
		s32 nNodeMinPriority;
		s32 nNodeMaxPriority;
	} ;

/* Error information of last DSP exception signalled to the GPP */
	struct DSP_ERRORINFO {
		u32 dwErrMask;
		u32 dwVal1;
		u32 dwVal2;
		u32 dwVal3;
	} ;

/* The DSP_PROCESSORSTATE structure describes the state of a DSP processor */
	struct DSP_PROCESSORSTATE {
		u32 cbStruct;
		enum DSP_PROCSTATE iState;
		struct DSP_ERRORINFO errInfo;
	} ;

/*
 * The DSP_RESOURCEINFO structure is used to retrieve information about a
 * processor's resources
 */
	struct DSP_RESOURCEINFO {
		u32 cbStruct;
		enum DSP_RESOURCEINFOTYPE uResourceType;
		union {
			u32 ulResource;
			struct DSP_MEMSTAT memStat;
			struct DSP_PROCLOADSTAT procLoadStat;
		} result;
	} ;

/*
 * The DSP_STREAMATTRIN structure describes the attributes of a stream,
 * including segment and alignment of data buffers allocated with
 * DSPStream_AllocateBuffers(), if applicable
 */
	struct DSP_STREAMATTRIN {
		u32 cbStruct;
		u32 uTimeout;
		u32 uSegment;
		u32 uAlignment;
		u32 uNumBufs;
		enum DSP_STRMMODE lMode;
		u32 uDMAChnlId;
		u32 uDMAPriority;
	} ;

/* The DSP_BUFFERATTR structure describes the attributes of a data buffer */
	struct DSP_BUFFERATTR {
		u32 cbStruct;
		u32 uSegment;
		u32 uAlignment;
	} ;

/*
 *  The DSP_STREAMINFO structure is used to retrieve information
 *  about a stream.
 */
	struct DSP_STREAMINFO {
		u32 cbStruct;
		u32 uNumberBufsAllowed;
		u32 uNumberBufsInStream;
		u32 ulNumberBytes;
		HANDLE hSyncObjectHandle;
		enum DSP_STREAMSTATE ssStreamState;
	} ;

/* DMM MAP attributes
It is a bit mask with each bit value indicating a specific attribute
bit 0 - GPP address type (user virtual=0, physical=1)
bit 1 - MMU Endianism (Big Endian=1, Little Endian=0)
bit 2 - MMU mixed page attribute (Mixed/ CPUES=1, TLBES =0)
bit 3 - MMU element size = 8bit (valid only for non mixed page entries)
bit 4 - MMU element size = 16bit (valid only for non mixed page entries)
bit 5 - MMU element size = 32bit (valid only for non mixed page entries)
bit 6 - MMU element size = 64bit (valid only for non mixed page entries)

bit 14 - Input (read only) buffer
bit 15 - Output (writeable) buffer
*/

/* Types of mapping attributes */

/* MPU address is virtual and needs to be translated to physical addr */
#define DSP_MAPVIRTUALADDR          0x00000000
#define DSP_MAPPHYSICALADDR         0x00000001

/* Mapped data is big endian */
#define DSP_MAPBIGENDIAN            0x00000002
#define DSP_MAPLITTLEENDIAN         0x00000000

/* Element size is based on DSP r/w access size */
#define DSP_MAPMIXEDELEMSIZE        0x00000004

/*
 * Element size for MMU mapping (8, 16, 32, or 64 bit)
 * Ignored if DSP_MAPMIXEDELEMSIZE enabled
 */
#define DSP_MAPELEMSIZE8            0x00000008
#define DSP_MAPELEMSIZE16           0x00000010
#define DSP_MAPELEMSIZE32           0x00000020
#define DSP_MAPELEMSIZE64           0x00000040

#define DSP_MAPVMALLOCADDR         0x00000080

#define DSP_MAPDONOTLOCK	   0x00000100

#define DSP_MAP_DIR_MASK		0x3FFF

#define GEM_CACHE_LINE_SIZE     128
#define GEM_L1P_PREFETCH_SIZE   128

/*
 * Definitions from dbreg.h
 */

#define DSPPROCTYPE_C64		6410
#define IVAPROCTYPE_ARM7	470

#define REG_MGR_OBJECT	1
#define REG_DRV_OBJECT	2

/* registry */
#define DRVOBJECT	"DrvObject"
#define MGROBJECT	"MgrObject"

/* Max registry path length. Also the max registry value length. */
#define MAXREGPATHLENGTH	255

/* MiniDriver related definitions */
#define DEFEXEC		"DefaultExecutable"	/* Default executable */
#define AUTOSTART	"AutoStart"		/* Statically load flag */
#define CURRENTCONFIG	"CurrentConfig"		/* Current resources */
#define SHMSIZE		"SHMSize"		/* Size of SHM reservd on MPU */
#define TCWORDSWAP	"TCWordSwap"		/* Traffic Controller WordSwp */
#define DSPRESOURCES	"DspTMSResources"	/* C55 DSP resurces on OMAP */

#endif				/* DBDEFS_ */
