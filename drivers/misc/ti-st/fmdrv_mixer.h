/*
 *  FM Driver for Connectivity chip of Texas Instruments.
 *
 *  Copyright (C) 2009 Texas Instruments
 *
 *  This program is free software; you can redistribute it and/or modify
 *  it under the terms of the GNU General Public License version 2 as
 *  published by the Free Software Foundation.
 *
 *  This program is distributed in the hope that it will be useful,
 *  but WITHOUT ANY WARRANTY; without even the implied warranty of
 *  MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
 *  GNU General Public License for more details.
 *
 *  You should have received a copy of the GNU General Public License
 *  along with this program; if not, write to the Free Software
 *  Foundation, Inc., 59 Temple Place, Suite 330, Boston, MA  02111-1307  USA
 *
 */

#ifndef _FM_MIXER_H
#define _FM_MIXER_H

/* FM mixer name */
#define FM_DRV_MIXER_NAME   "tifm_mixer"

int fm_mixer_init(struct fmdrv_ops *fmdev);
int fm_mixer_deinit(struct fmdrv_ops *fmdev);

#endif
