/* include/sharp/snfc_ucc.h (NFC driver)
 *
 * Copyright (C) 2011 SHARP CORPORATION
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
#ifndef _LINUX_SNFC_UCC_H
#define _LINUX_SNFC_UCC_H

#define IOC_MAGIC 's'

#define SHSNFC_UCC_REQ_AUTOPOLL			_IOWR(IOC_MAGIC, 1, int)
#define SHSNFC_UCC_REQ_START_NFC		_IOWR(IOC_MAGIC, 2, int)
#define SHSNFC_UCC_REQ_END_PROC			_IOWR(IOC_MAGIC, 3, int)
#define SHSNFC_UCC_REQ_START_NFC_P2P	_IOWR(IOC_MAGIC, 4, int)

#define SHSNFC_UCC_RWS_NOCHECK	0		/* (0x00) no-check RWS */
#define SHSNFC_UCC_RWS_CHECK	1		/* (0x01) check RWS */
#define SHSNFC_UCC_RFS_CHECK	2		/* (0x02) check RFS for SHSNFC_UCC_REQ_START_NFC_P2P */

enum
{
	SHSNFC_UCC_RETVAL_OK = 0,
	SHSNFC_UCC_RETVAL_BUSY = -1,
	SHSNFC_UCC_RETVAL_ABNORMAL = -2
};

#endif /* _LINUX_SNFC_UCC_H */

