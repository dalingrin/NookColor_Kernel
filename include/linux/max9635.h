/*
 * Copyright (c) 2010, Barnes & Noble. All rights reserved.
 * Written by David Bolcsfoldi <dbolcsfoldi@intrinsyc.com>
 *
 * This program is free software: you can redistribute it and/or modify
 * it under the terms of the GNU General Public License as published by
 * the Free Software Foundation, either version 3 of the License, or
 * (at your option) any later version.
 *
 * This program is distributed in the hope that it will be useful,
 * but WITHOUT ANY WARRANTY; without even the implied warranty of
 * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
 * GNU General Public License for more details.
 *
 * You should have received a copy of the GNU General Public License
 * along with this program.  If not, see <http://www.gnu.org/licenses/>.
 *
 */

#ifndef __MAX9635_H__
#define __MAX9635_H__

struct max9635_pdata {
    int gpio;
    int poll_interval; 
	int (*device_resource)(int);
};

#define MAX9635_PDATA(pdata) ((struct max9635_pdata *) (pdata))
#define MAX9635_NAME "max9635"
#define MAX9635_I2C_SLAVE_ADDRESS1 0x4a
#define MAX9635_I2C_SLAVE_ADDRESS2 0x4b

#endif /* __MAX9635_H__ */
