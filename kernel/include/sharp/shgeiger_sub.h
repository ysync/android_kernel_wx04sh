/* include/sharp/shgeiger_sub.h  (shgeiger sub Driver)
 *
 * Copyright (C) 2012 SHARP CORPORATION All rights reserved.
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

/*
 * Definitions for Shgeiger chip.
 */

/* ------------------------------------------------------------------------- */
/* SHARP GEIGER SENSOR SUB DRIVER FOR KERNEL MODE                             */
/* ------------------------------------------------------------------------- */

#ifndef SHGEIGERSUB_H
#define SHGEIGERSUB_H

/* ------------------------------------------------------------------------- */
/* MACROS                                                                    */
/* ------------------------------------------------------------------------- */

#define SH_GEIGERSUB_RESULT_FAILURE    -1
#define SH_GEIGERSUB_RESULT_SUCCESS    0

#define SH_GEIGERSUB_I2C_DEVNAME	"SH_GEIGERSUB"
#define SH_GEIGER_EPPROM_I2C_SLAVE		0x50

/*EEPROM Register*/
#define	BG_LSB			0x00
#define	BG_MSB			0x01
#define	MULTI_LSB		0x02
#define	MULTI_MSB		0x03
#define	MDULE_VER		0x20
#define	THERMAL_A		0x21
#define	THERMAL_B		0x22

int SHGEIGER_EEPROM_I2cRead(char *rxData, int length);

#endif /* SHGEIGERSUB_H */

/* ------------------------------------------------------------------------- */
/* END OF FILE                                                               */
/* ------------------------------------------------------------------------- */


