/*
 * getsection.c
 *
 * DSP-BIOS Bridge driver support functions for TI OMAP processors.
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

#include <dspbridge/getsection.h>
#include "header.h"

/*
 * Error strings
 */
static const char E_READSTRM[] = { "Error reading %s from input stream" };
static const char E_SEEK[] = { "Set file position to %d failed" };
static const char E_ISIZ[] = { "Bad image packet size %d" };
static const char E_CHECKSUM[] = { "Checksum failed on %s" };
static const char E_RELOC[] = { "DLOAD_GetSection unable to read"
	"sections containing relocation entries"};
#if BITS_PER_AU > BITS_PER_BYTE
static const char E_ALLOC[] = { "Syms->Allocate( %d ) failed" };
static const char E_STBL[] = { "Bad string table offset " FMT_UI32 };
#endif

/*
 * we use the fact that DOFF section records are shaped just like
 * LDR_SECTION_INFO to reduce our section storage usage.  These macros
 * marks the places where that assumption is made
 */
#define DOFFSEC_IS_LDRSEC(pdoffsec) ((struct LDR_SECTION_INFO *)(pdoffsec))
#define LDRSEC_IS_DOFFSEC(ldrsec) ((struct doff_scnhdr_t *)(ldrsec))

/***************************************************************/
/********************* SUPPORT FUNCTIONS ***********************/
/***************************************************************/

#if BITS_PER_AU > BITS_PER_BYTE
/**************************************************************************
 * Procedure unpack_sec_name
 *
 * Parameters:
 *  dlthis		Handle from DLOAD_module_open for this module
 *	soffset	    Byte offset into the string table
 *  dst         Place to store the expanded string
 *
 * Effect:
 *	Stores a string from the string table into the destination, expanding
 * it in the process.  Returns a pointer just past the end of the stored
 * string on success, or NULL on failure.
 *
 *************************************************************************/
static char *unpack_sec_name(struct dload_state *dlthis,
			     u32 soffset, char *dst)
{
	u8 tmp, *src;

	if (soffset >= dlthis->dfile_hdr.df_scn_name_size) {
		dload_error(dlthis, E_STBL, soffset);
		return NULL;
	}
	src = (u8 *)dlthis->str_head +
	    (soffset >> (LOG_BITS_PER_AU - LOG_BITS_PER_BYTE));
	if (soffset & 1)
		*dst++ = *src++;	/* only 1 character in first word */
	do {
		tmp = *src++;
		*dst = (tmp >> BITS_PER_BYTE)
		if (!(*dst++))
			break;
	} while ((*dst++ = tmp & BYTE_MASK));

	return dst;
}

/**************************************************************************
 * Procedure expand_sec_names
 *
 * Parameters:
 *  dlthis		Handle from DLOAD_module_open for this module
 *
 * Effect:
 *    Allocates a buffer, unpacks and copies strings from string table into it.
 * Stores a pointer to the buffer into a state variable.
 **************************************************************************/
static void expand_sec_names(struct dload_state *dlthis)
{
	char *xstrings, *curr, *next;
	u32 xsize;
	u16 sec;
	struct LDR_SECTION_INFO *shp;
	/* assume worst-case size requirement */
	xsize = dlthis->dfile_hdr.df_max_str_len * dlthis->dfile_hdr.df_no_scns;
	xstrings = (char *)dlthis->mysym->Allocate(dlthis->mysym, xsize);
	if (xstrings == NULL) {
		dload_error(dlthis, E_ALLOC, xsize);
		return;
	}
	dlthis->xstrings = xstrings;
	/* For each sec, copy and expand its name */
	curr = xstrings;
	for (sec = 0; sec < dlthis->dfile_hdr.df_no_scns; sec++) {
		shp = DOFFSEC_IS_LDRSEC(&dlthis->sect_hdrs[sec]);
		next = unpack_sec_name(dlthis, *(u32 *) &shp->name, curr);
		if (next == NULL)
			break;	/* error */
		shp->name = curr;
		curr = next;
	}
}

#endif

/***************************************************************/
/********************* EXPORTED FUNCTIONS **********************/
/***************************************************************/

/**************************************************************************
 * Procedure DLOAD_module_open
 *
 * Parameters:
 *	module	The input stream that supplies the module image
 *	syms	Host-side malloc/free and error reporting functions.
 *			Other methods are unused.
 *
 * Effect:
 *	Reads header information from a dynamic loader module using the
    specified
 * stream object, and returns a handle for the module information.  This
 * handle may be used in subsequent query calls to obtain information
 * contained in the module.
 *
 * Returns:
 *	NULL if an error is encountered, otherwise a module handle for use
 * in subsequent operations.
 **************************************************************************/
DLOAD_module_info DLOAD_module_open(struct Dynamic_Loader_Stream *module,
				    struct Dynamic_Loader_Sym *syms)
{
	struct dload_state *dlthis;	/* internal state for this call */
	unsigned *dp, sz;
	u32 sec_start;
#if BITS_PER_AU <= BITS_PER_BYTE
	u16 sec;
#endif

	/* Check that mandatory arguments are present */
	if (!module || !syms) {
		if (syms != NULL)
			dload_syms_error(syms, "Required parameter is NULL");

		return NULL;
	}

	dlthis = (struct dload_state *)
		 syms->Allocate(syms, sizeof(struct dload_state));
	if (!dlthis) {
		/* not enough storage */
		dload_syms_error(syms, "Can't allocate module info");
		return NULL;
	}

	/* clear our internal state */
	dp = (unsigned *)dlthis;
	for (sz = sizeof(struct dload_state) / sizeof(unsigned);
	     sz > 0; sz -= 1)
		*dp++ = 0;

	dlthis->strm = module;
	dlthis->mysym = syms;

	/* read in the doff image and store in our state variable */
	dload_headers(dlthis);

	if (!dlthis->dload_errcount)
		dload_strings(dlthis, true);

	/* skip ahead past the unread portion of the string table */
	sec_start = sizeof(struct doff_filehdr_t) +
		    sizeof(struct doff_verify_rec_t) +
		    BYTE_TO_HOST(DOFF_ALIGN(dlthis->dfile_hdr.df_strtab_size));

	if (dlthis->strm->set_file_posn(dlthis->strm, sec_start) != 0) {
		dload_error(dlthis, E_SEEK, sec_start);
		return NULL;
	}

	if (!dlthis->dload_errcount)
		dload_sections(dlthis);

	if (dlthis->dload_errcount) {
		DLOAD_module_close(dlthis);	/* errors, blow off our state */
		dlthis = NULL;
		return NULL;
	}
#if BITS_PER_AU > BITS_PER_BYTE
	/* Expand all section names from the string table into the   */
	/* state variable, and convert section names from a relative */
	/* string table offset to a pointers to the expanded string. */
	expand_sec_names(dlthis);
#else
	/* Convert section names from a relative string table offset */
	/* to a pointer into the string table.                       */
	for (sec = 0; sec < dlthis->dfile_hdr.df_no_scns; sec++) {
		struct LDR_SECTION_INFO *shp =
		    DOFFSEC_IS_LDRSEC(&dlthis->sect_hdrs[sec]);
		shp->name = dlthis->str_head + *(u32 *)&shp->name;
	}
#endif

	return dlthis;
}

/***************************************************************************
 * Procedure DLOAD_GetSectionInfo
 *
 * Parameters:
 *  minfo		Handle from DLOAD_module_open for this module
 *	sectionName	Pointer to the string name of the section desired
 *	sectionInfo	Address of a section info structure pointer to be
 *			initialized
 *
 * Effect:
 *	Finds the specified section in the module information, and initializes
 * the provided struct LDR_SECTION_INFO pointer.
 *
 * Returns:
 *	true for success, false for section not found
 **************************************************************************/
int DLOAD_GetSectionInfo(DLOAD_module_info minfo, const char *sectionName,
		     const struct LDR_SECTION_INFO **const sectionInfo)
{
	struct dload_state *dlthis;
	struct LDR_SECTION_INFO *shp;
	u16 sec;

	dlthis = (struct dload_state *)minfo;
	if (!dlthis)
		return false;

	for (sec = 0; sec < dlthis->dfile_hdr.df_no_scns; sec++) {
		shp = DOFFSEC_IS_LDRSEC(&dlthis->sect_hdrs[sec]);
		if (strcmp(sectionName, shp->name) == 0) {
			*sectionInfo = shp;
			return true;
		}
	}

	return false;
}

#define IPH_SIZE (sizeof(struct image_packet_t) - sizeof(u32))
#define REVERSE_REORDER_MAP(rawmap) ((rawmap) ^ 0x3030303)

/**************************************************************************
 * Procedure DLOAD_GetSection
 *
 * Parameters:
 *  minfo		Handle from DLOAD_module_open for this module
 *	sectionInfo	Pointer to a section info structure for the desired
 *			section
 *	sectionData	Buffer to contain the section initialized data
 *
 * Effect:
 *	Copies the initialized data for the specified section into the
 * supplied buffer.
 *
 * Returns:
 *	true for success, false for section not found
 **************************************************************************/
int DLOAD_GetSection(DLOAD_module_info minfo,
		 const struct LDR_SECTION_INFO *sectionInfo, void *sectionData)
{
	struct dload_state *dlthis;
	u32 pos;
	struct doff_scnhdr_t *sptr = NULL;
	s32 nip;
	struct image_packet_t ipacket;
	s32 ipsize;
	u32 checks;
	s8 *dest = (s8 *)sectionData;

	dlthis = (struct dload_state *)minfo;
	if (!dlthis)
		return false;
	sptr = LDRSEC_IS_DOFFSEC(sectionInfo);
	if (sptr == NULL)
		return false;

	/* skip ahead to the start of the first packet */
	pos = BYTE_TO_HOST(DOFF_ALIGN((u32) sptr->ds_first_pkt_offset));
	if (dlthis->strm->set_file_posn(dlthis->strm, pos) != 0) {
		dload_error(dlthis, E_SEEK, pos);
		return false;
	}

	nip = sptr->ds_nipacks;
	while ((nip -= 1) >= 0) {	/* for each packet */
		/* get the fixed header bits */
		if (dlthis->strm->
		    read_buffer(dlthis->strm, &ipacket, IPH_SIZE) != IPH_SIZE) {
			dload_error(dlthis, E_READSTRM, "image packet");
			return false;
		}
		/* reorder the header if need be */
		if (dlthis->reorder_map)
			dload_reorder(&ipacket, IPH_SIZE, dlthis->reorder_map);

		/* Now read the packet image bits. Note: round the size up to
		 * the next multiple of 4 bytes; this is what checksum
		 * routines want. */
		ipsize = BYTE_TO_HOST(DOFF_ALIGN(ipacket.i_packet_size));
		if (ipsize > BYTE_TO_HOST(IMAGE_PACKET_SIZE)) {
			dload_error(dlthis, E_ISIZ, ipsize);
			return false;
		}
		if (dlthis->strm->read_buffer
		    (dlthis->strm, dest, ipsize) != ipsize) {
			dload_error(dlthis, E_READSTRM, "image packet");
			return false;
		}
		/* reorder the bytes if need be */
#if !defined(_BIG_ENDIAN) || (TARGET_AU_BITS > 16)
		if (dlthis->reorder_map)
			dload_reorder(dest, ipsize, dlthis->reorder_map);

		checks = dload_checksum(dest, ipsize);
#else
		if (dlthis->dfile_hdr.df_byte_reshuffle !=
		    TARGET_ORDER(REORDER_MAP(BYTE_RESHUFFLE_VALUE))) {
			/* put image bytes in big-endian order, not PC order */
			dload_reorder(dest, ipsize,
				      TARGET_ORDER(dlthis->dfile_hdr.
				      df_byte_reshuffle));
		}
#if TARGET_AU_BITS > 8
		checks = dload_reverse_checksum_16(dest, ipsize);
#else
		checks = dload_reverse_checksum(dest, ipsize);
#endif
#endif
		checks += dload_checksum(&ipacket, IPH_SIZE);

		/* NYI: unable to handle relocation entries here.  Reloc
		 * entries referring to fields that span the packet boundaries
		 * may result in packets of sizes that are not multiple of
		 * 4 bytes. Our checksum implementation works on 32-bit words
		 * only.     */
		if (ipacket.i_num_relocs != 0) {
			dload_error(dlthis, E_RELOC, ipsize);
			return false;
		}

		if (~checks) {
			dload_error(dlthis, E_CHECKSUM, "image packet");
			return false;
		}

		/*Advance destination ptr by the size of the just-read packet*/
		dest += ipsize;
	}

	return true;
}

/***************************************************************************
 * Procedure DLOAD_module_close
 *
 * Parameters:
 *  minfo		Handle from DLOAD_module_open for this module
 *
 * Effect:
 *	Releases any storage associated with the module handle.  On return,
 * the module handle is invalid.
 *
 * Returns:
 *	Zero for success. On error, the number of errors detected is returned.
 * Individual errors are reported using syms->Error_Report(), where syms was
 * an argument to DLOAD_module_open
 **************************************************************************/
void DLOAD_module_close(DLOAD_module_info minfo)
{
	struct dload_state *dlthis;

	dlthis = (struct dload_state *)minfo;
	if (!dlthis)
		return;

	if (dlthis->str_head)
		dlthis->mysym->Deallocate(dlthis->mysym, dlthis->str_head);

	if (dlthis->sect_hdrs)
		dlthis->mysym->Deallocate(dlthis->mysym, dlthis->sect_hdrs);

#if BITS_PER_AU > BITS_PER_BYTE
	if (dlthis->xstrings)
		dlthis->mysym->Deallocate(dlthis->mysym, dlthis->xstrings);

#endif

	dlthis->mysym->Deallocate(dlthis->mysym, dlthis);
}
