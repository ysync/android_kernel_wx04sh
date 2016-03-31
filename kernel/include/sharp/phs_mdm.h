/* include/sharp/phs_mdm.h
 *
 * Copyright (C) 2011-2013 SHARP CORPORATION
 *
 * This software is licensed under the terms of the GNU General Public
 * License version 2, as published by the Free Software Foundation, and
 * may be copied, distributed, and modified under those terms.
 *
 * This program is distributed in the hope that it will be useful,
 * but WITHOUT ANY WARRANTY; without even the implied warranty of
 * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
 * GNU General Public License for more details.
 *
 */

#ifndef _PHS_MDM_H
#define _PHS_MDM_H

#define SHPHS_MAGIC 'P'
#define SHPHS_TIOCMGET		_IOR(SHPHS_MAGIC, 1, unsigned int)
#define SHPHS_TIOCMSET		_IOW(SHPHS_MAGIC, 2, unsigned int)
#define	SHPHS_ENABLEDISP	_IOW(SHPHS_MAGIC, 3, unsigned int)
#define SHPHS_TIOCMIWAIT	_IOW(SHPHS_MAGIC, 5, unsigned int)
#define SHPHS_TIOCGICOUNT	_IOR(SHPHS_MAGIC, 6, struct phsmdm_icount)
#define SHPHS_TIOCMBIC		_IOW(SHPHS_MAGIC, 7, unsigned int)
#define SHPHS_TIOCMBIS		_IOW(SHPHS_MAGIC, 8, unsigned int)

#define	SHPHS_CLK_ONOFF		0x5555
#define	SHPHS_PURGE_SEND	0x5556

#define TIOCM_DISP1 0x10000
#define TIOCM_DISP2 0x20000
#define TIOCM_DISP3 0x40000

struct phsmdm_icount
{
	__u32	dcd;
	__u32	ri;
	__u32	disp1;
	__u32	disp2;
	__u32	disp3;
};

#endif
