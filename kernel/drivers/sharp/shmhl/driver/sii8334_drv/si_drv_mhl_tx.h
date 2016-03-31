






/*
 SiI8334 Linux Driver

 Copyright (C) 2011 Silicon Image Inc.

 This program is free software; you can redistribute it and/or
 modify it under the terms of the GNU General Public License as
 published by the Free Software Foundation version 2.

 This program is distributed .as is. WITHOUT ANY WARRANTY of any
 kind, whether express or implied; without even the implied warranty
 of MERCHANTABILITY or FITNESS FOR A PARTICULAR
 PURPOSE.  See the
 GNU General Public License for more details.
*/
/*
   @file si_drv_mhl_tx.h
 */

// DEVCAP we will initialize to
#ifdef CONFIG_SII8334_MHL_TX
void shmhl_force_disconnect(void);
#define	MHL_LOGICAL_DEVICE_MAP		(MHL_DEV_LD_VIDEO | MHL_DEV_LD_GUI )
#else /* CONFIG_SII8334_MHL_TX */
#define	MHL_LOGICAL_DEVICE_MAP		(MHL_DEV_LD_AUDIO | MHL_DEV_LD_VIDEO | MHL_DEV_LD_MEDIA | MHL_DEV_LD_GUI )
#endif /* CONFIG_SII8334_MHL_TX */

