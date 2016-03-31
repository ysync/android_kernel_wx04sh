/* include/sharp/shtps_dev_tma4xx.h
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
#ifndef __SHTPS_DEV_TMA4XX_H__
#define __SHTPS_DEV_TMA4XX_H__

#ifndef u8
#define u8 unsigned char
#endif
#ifndef u16
#define u16 unsigned short
#endif
#ifndef u32
#define u32 unsigned int
#endif

#define TPSIF_DEV_NAME		"shtpsif"
#define	TPSIF_DEV_FULLNAME	"/dev/shtpsif"

#if defined(CONFIG_SHTPS_TMA4XX_TMA463_002)
#define TPSIF_FW_IMG_SIZE       55607
#define TPSIF_TX_NUM		12
#define TPSIF_RX_NUM		21
#else
#define TPSIF_FW_IMG_SIZE       55607
#define TPSIF_TX_NUM		12
#define TPSIF_RX_NUM		21
#endif

#define TPSIF_RAW_DATA_NUM	(TPSIF_TX_NUM * TPSIF_RX_NUM)
#define TPSIF_IDAC_DATA_NUM	(TPSIF_TX_NUM * TPSIF_RX_NUM + 1)

#define TPSIF_IOC_MAGIC 0xE0

#define TPSIF_POWER_ON			_IO(TPSIF_IOC_MAGIC, 0)
#define TPSIF_POWER_OFF			_IO(TPSIF_IOC_MAGIC, 1)
#define TPSIF_HWRESET			_IO(TPSIF_IOC_MAGIC, 2)
#define TPSIF_SWRESET			_IO(TPSIF_IOC_MAGIC, 3)
#define TPSIF_SLEEP_ON			_IO(TPSIF_IOC_MAGIC, 4)
#define TPSIF_SLEEP_OFF			_IO(TPSIF_IOC_MAGIC, 5)
#define TPSIF_FW_VERSION		_IOR(TPSIF_IOC_MAGIC, 6, u32)
#define TPSIF_FW_UPDATE			_IOW(TPSIF_IOC_MAGIC, 7, u8)
#define TPSIF_DRV_STAT			_IO(TPSIF_IOC_MAGIC, 8)
#define TPSIF_DRV_VER			_IO(TPSIF_IOC_MAGIC, 9)
#define TPSIF_IRQ_STAT			_IO(TPSIF_IOC_MAGIC, 10)
#define TPSIF_CALIBRATION_IDAC		_IO(TPSIF_IOC_MAGIC, 11)
#define TPSIF_SET_REG_ADDR		_IOW(TPSIF_IOC_MAGIC, 12, u16)
#define TPSIF_READ_REG			_IOR(TPSIF_IOC_MAGIC, 13, u8)
#define TPSIF_WRITE_REG			_IOW(TPSIF_IOC_MAGIC, 14, u8)
#define TPSIF_SWITCH_MODE		_IOW(TPSIF_IOC_MAGIC, 15, int)
#define TPSIF_START_SCAN		_IOW(TPSIF_IOC_MAGIC, 16, int)
#define TPSIF_HWRESET_STARTUP		_IO(TPSIF_IOC_MAGIC, 17)
#define TPSIF_SWRESET_STARTUP		_IO(TPSIF_IOC_MAGIC, 18)
#define TPSIF_CALIBRATION_PARAM		_IOW(TPSIF_IOC_MAGIC, 19, u16)
#define TPSIF_SET_LOWPOWER_MODE		_IOW(TPSIF_IOC_MAGIC, 20, int)
#define TPSIF_SET_CONT_LOWPOWER_MODE	_IOW(TPSIF_IOC_MAGIC, 21, int)
#define TPSIF_SET_THINOUT_EVMODE	_IOW(TPSIF_IOC_MAGIC, 22, int)

/* controller mode */
enum {
	TPSIF_MODE_OPERATE = 0,
	TPSIF_MODE_SYSINFO,
	TPSIF_MODE_CONFIG,
	TPSIF_MODE_NUM,
};

/* calibration mode */
enum {
	TPSIF_CALIB_MODE_MUTUAL_FINE = 0,
	TPSIF_CALIB_MODE_MUTUAL_BUTTONS,
	TPSIF_CALIB_MODE_SELF,
	TPSIF_CALIB_MODE_BALANCED,
	TPSIF_CALIB_MODE_NUM,
};

/* panel scan type */
enum {
	TPSIF_SCAN_TYPE_MUTUAL_RAW = 0,
	TPSIF_SCAN_TYPE_MUTUAL_BASE,
	TPSIF_SCAN_TYPE_MUTUAL_DIFF,
	TPSIF_SCAN_TYPE_SELF_RAW,
	TPSIF_SCAN_TYPE_SELF_BASE,
	TPSIF_SCAN_TYPE_SELF_DIFF,
	TPSIF_SCAN_TYPE_MUTUAL_RAW_AND_BASE,
	TPSIF_SCAN_TYPE_SELF_RAW_AND_BASE,
	TPSIF_SCAN_TYPE_IDAC_MUTUAL,
	TPSIF_SCAN_TYPE_IDAC_SELF,
	TPSIF_SCAN_TYPE_NUM,
};

struct cyttsp4_scan_data {
	u8 buf[TPSIF_RAW_DATA_NUM];
	size_t buf_size;
	size_t data_size;
	u16 data_num;
	u8 matrix_mapping;	/* 0 = row major, 1 = column major */
#define TPSIF_SCAN_DATA_MATRIX_ROW_MAJOR	0
#define TPSIF_SCAN_DATA_MATRIX_COLUMN_MAJOR	1
	u8 endian;		/* 0 = big endian, 1 = little endian */
#define TPSIF_SCAN_DATA_ENDIAN_BIG	0
#define TPSIF_SCAN_DATA_ENDIAN_LITTLE	1
	u8 sign_type;		/* 0 = unsigned, 1 = 2's complement signed */
#define TPSIF_SCAN_DATA_SIGN_UNSIGNED	0
#define TPSIF_SCAN_DATA_SIGN_2COMP	1
	int scan_type;
};

struct cyttsp4_idac_data {
	u8 buf[TPSIF_IDAC_DATA_NUM];
	size_t buf_size;
	size_t data_size;
	u16 data_num;
	int scan_type;
};

extern void msm_tps_setsleep(int on);

#endif /* __SHTPS_DEV_TMA4XX_H__ */
