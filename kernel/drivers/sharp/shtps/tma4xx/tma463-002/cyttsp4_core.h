/*
 * Header file for:
 * Cypress TrueTouch(TM) Standard Product (TTSP) touchscreen drivers.
 * For use with Cypress Gen4 and Solo parts.
 * Supported parts include:
 * CY8CTMA884/616
 * CY8CTMA4XX
 *
 * Copyright (C) 2009-2012 Cypress Semiconductor, Inc.
 * Copyright (C) 2011 Motorola Mobility, Inc.
 *
 * This program is free software; you can redistribute it and/or
 * modify it under the terms of the GNU General Public License
 * version 2, and only version 2, as published by the
 * Free Software Foundation.
 *
 * This program is distributed in the hope that it will be useful,
 * but WITHOUT ANY WARRANTY; without even the implied warranty of
 * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
 * GNU General Public License for more details.
 *
 * You should have received a copy of the GNU General Public License along
 * with this program; if not, write to the Free Software Foundation, Inc.,
 * 51 Franklin Street, Fifth Floor, Boston, MA 02110-1301, USA.
 *
 * Contact Cypress Semiconductor at www.cypress.com <kev@cypress.com>
 *
 */

#ifndef __CYTTSP4_CORE_H__
#define __CYTTSP4_CORE_H__

#define CY_NUM_RETRY                10 /* max retries for rd/wr ops */

#define CY_I2C_NAME                 "cyttsp4-i2c"
//#define CY_SPI_NAME                 "SH_touchpanel"
#define CY_SPI_NAME                 "SH_touchpanel2"
#define CY_DRIVER_VERSION           "Rev4-2M-25"
#define CY_DRIVER_DATE              "2011-12-22"

/* use the following define if the device is a TMA400 family part
 */
#define CY_USE_TMA400

/* use the following define if the device is a TMA884/616 family part
#define CY_USE_TMA884
 */

/* use the following define if the firmware does not include bootloader or
 * if auto load is not desired
#define CY_NO_AUTO_LOAD
 */

/* use the following define to allow autoload for any version diffs;
 * otherwise only autoload if load version is greater than device image version
 */
#define CY_ANY_DIFF_NEW_VER

/* use the following define to include loader application
#define CY_USE_FORCE_LOAD
 */

/* use the following define to enable register peak/poke capability
#define CY_USE_REG_ACCESS
 */

/* use the following define to enable special debug tools for test only
 */
#define CY_USE_DEBUG_TOOLS

/* use the following define to use level interrupt method (else falling edge)
 * this method should only be used if the host processor misses edge interrupts
#define CY_USE_LEVEL_IRQ
 */

/* use the following define to enable driver watchdog timer
#define CY_USE_WATCHDOG
 */

/* system includes are here in order to allow DEBUG option */
/* enable this define to enable debug prints
#define DEBUG = y
 */

/* enable this define to enable debug prints
#define VERBOSE_DEBUG
 */

/* SH Custom -> */
/* use the following define to enable debug code
 */
#define CONFIG_TOUCHSCREEN_DEBUG

/* use the following define to allow auto sensor calibration on
 * startup
#define STARTUP_CALIBRATION
 */

/* use the following define to allow auto sensor calibration after
 * firmware update
 */
#define CALIBRATION_AFTER_FIRMWARE_UPDATE

/* use the following define to allow reset after calibration
 */
#define RESET_AFTER_CALIBRATION

/* use the following define to check mddata on startup
#define STARTUP_MDDATA_CHECK
 */

/* use the following define to reverse Y axis
#define REVERSE_Y_AXIS
 */

/* use the following define to keep compatibility with TMA443
 */
#define TMA443_COMPATIBILITY

/* use the following define to enable functions for diag mode
 */
#define SH_TPSIF_COMMAND
/* <- SH Custom */

#ifdef CONFIG_TOUCHSCREEN_DEBUG
#define CY_DBG_LVL_0			0
#define CY_DBG_LVL_1			1
#define CY_DBG_LVL_2			2
#define CY_DBG_LVL_3			3
#ifdef CY_USE_DEBUG_TOOLS
#define CY_DBG_SUSPEND			4
#define CY_DBG_RESUME			5
#define CY_DBG_SOFT_RESET		6
#define CY_DBG_RESET			7
#ifdef CY_USE_DEV_DEBUG_TOOLS
#define CY_DBG_PUT_ALL_PARAMS		8
#define CY_DBG_CHECK_MDDATA		9
#define CY_DBG_GET_MDDATA		10
#define CY_DBG_GET_IC_TCH_CRC		11
#define CY_DBG_GET_IC_TCH_ROW		12
#define CY_DBG_READ_IC_TCH_CRC		13
#define CY_DBG_CALC_IC_TCH_CRC		14
#define CY_DBG_READ_TABLE_TCH_CRC	15
#define CY_DBG_CALC_TABLE_TCH_CRC	16
#define CY_DBG_PUT_IC_TCH_ROW		17
#endif	/* --CY_USE_DEV_DEBUG_TOOLS */
#endif	/* --CY_USE_DEBUG_TOOLS */
#endif	/* --CONFIG_TOUCHSCREEN_DEBUG */

#include <linux/kernel.h>
#include <linux/device.h>
#include <linux/err.h>
#include <linux/input.h>
#include <linux/input/mt.h>

#ifdef CONFIG_HAS_EARLYSUSPEND
#include <linux/earlysuspend.h>
#endif
#define CY_MAX_PRBUF_SIZE       PIPE_BUF
#define CY_PR_TRUNCATED		"..."

struct cyttsp4_bus_ops {
	int (*write)(void *handle, u16 subaddr, size_t length,
		const void *values, int i2c_addr, bool use_subaddr);
	int (*read)(void *handle, u16 subaddr, size_t length,
		void *values, int i2c_addr, bool use_subaddr);
	struct device *dev;
#ifdef CONFIG_TOUCHSCREEN_DEBUG
	u8 tsdebug;
#endif /* --CONFIG_TOUCHSCREEN_DEBUG */
};

void *cyttsp4_core_init(struct cyttsp4_bus_ops *bus_ops,
	struct device *dev, int irq, char *name);

void cyttsp4_core_release(void *handle);

#if !defined(CONFIG_HAS_EARLYSUSPEND)
#if defined(CONFIG_PM_SLEEP)
extern const struct dev_pm_ops cyttsp4_pm_ops;
#elif defined(CONFIG_PM)
int cyttsp4_resume(void *handle);
int cyttsp4_suspend(void *handle);
#endif
#endif

#endif /* __CYTTSP4_CORE_H__ */
