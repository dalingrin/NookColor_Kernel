/*
 * rms_sh.h
 *
 * DSP-BIOS Bridge driver support functions for TI OMAP processors.
 *
 * DSP/BIOS Bridge Resource Manager Server shared definitions (used on both
 * GPP and DSP sides).
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

#ifndef RMS_SH_
#define RMS_SH_

#include <dspbridge/rmstypes.h>

/* Node Types: */
#define RMS_TASK                1	/* Task node */
#define RMS_DAIS                2	/* xDAIS socket node */
#define RMS_MSG                 3	/* Message node */

/* Memory Types: */
#define RMS_CODE                0	/* Program space */
#define RMS_DATA                1	/* Data space */
#define RMS_IO                	2	/* I/O space */

/* RM Server Command and Response Buffer Sizes: */
#define RMS_COMMANDBUFSIZE     256	/* Size of command buffer */
#define RMS_RESPONSEBUFSIZE    16	/* Size of response buffer */

/* Pre-Defined Command/Response Codes: */
#define RMS_EXIT                0x80000000   /* GPP->Node: shutdown */
#define RMS_EXITACK             0x40000000   /* Node->GPP: ack shutdown */
#define RMS_BUFDESC             0x20000000   /* Arg1 SM buf, Arg2 is SM size */
#define RMS_KILLTASK            0x10000000  /* GPP->Node: Kill Task */
#define RMS_USER                0x0	/* Start of user-defined msg codes */
#define RMS_MAXUSERCODES        0xfff	/* Maximum user defined C/R Codes */


/* RM Server RPC Command Structure: */
	struct RMS_Command {
		RMS_WORD fxn;	/* Server function address */
		RMS_WORD arg1;	/* First argument */
		RMS_WORD arg2;	/* Second argument */
		RMS_WORD data;	/* Function-specific data array */
	} ;

/*
 *  The RMS_StrmDef structure defines the parameters for both input and output
 *  streams, and is passed to a node's create function.
 */
	struct RMS_StrmDef {
		RMS_WORD bufsize;	/* Buffer size (in DSP words) */
		RMS_WORD nbufs;	/* Max number of bufs in stream */
		RMS_WORD segid;	/* Segment to allocate buffers */
		RMS_WORD align;	/* Alignment for allocated buffers */
		RMS_WORD timeout;	/* Timeout (msec) for blocking calls */
		RMS_CHAR name[1];	/* Device Name (terminated by '\0') */
	} ;

/* Message node create args structure: */
	struct RMS_MsgArgs {
		RMS_WORD maxMessages;	/* Max # simultaneous msgs to node */
		RMS_WORD segid;	/* Mem segment for NODE_allocMsgBuf */
		RMS_WORD notifyType;	/* Type of message notification */
		RMS_WORD argLength;	/* Length (in DSP chars) of arg data */
		RMS_WORD argData;	/* Arg data for node */
	} ;

/* Partial task create args structure */
	struct RMS_MoreTaskArgs {
		RMS_WORD priority;	/* Task's runtime priority level */
		RMS_WORD stackSize;	/* Task's stack size */
		RMS_WORD sysstackSize;	/* Task's system stack size (55x) */
		RMS_WORD stackSeg;	/* Memory segment for task's stack */
		RMS_WORD heapAddr;   /* base address of the node memory heap in
				      * external memory (DSP virtual address) */
		RMS_WORD heapSize;   /* size in MAUs of the node memory heap in
				      * external memory */
		RMS_WORD misc;	/* Misc field.  Not used for 'normal'
				 * task nodes; for xDAIS socket nodes
				 * specifies the IALG_Fxn pointer.
				 */
		/* # input STRM definition structures */
		RMS_WORD numInputStreams;
	} ;

#endif				/* RMS_SH_ */

