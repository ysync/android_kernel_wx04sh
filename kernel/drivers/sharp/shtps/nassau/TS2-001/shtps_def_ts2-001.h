/* drivers/sharp/shtps/nassau/xxx/shtps_def_xxx.h
 *
 * Copyright (C) 2012 SHARP CORPORATION
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
#ifndef __SHTPS_DEF_TS2_001_H__
#define __SHTPS_DEF_TS2_001_H__

/* -----------------------------------------------------------------------------------
 */
#define SHTPS_NASSAU_DEV_POS_OFFSET_X			355
#define SHTPS_NASSAU_DEV_POS_OFFSET_Y			1008
#define SHTPS_NASSAU_DEV_POS_MAX_X				24338
#define SHTPS_NASSAU_DEV_POS_MAX_Y				37992

#define SHTPS_NASSAU_SENSOR_NUM_RX				24
#define SHTPS_NASSAU_SENSOR_NUM_TX				39

#define SHTPS_NASSAU_MT_PRESSURE_MAX			0x1B58

/* -----------------------------------------------------------------------------------
 */
#define SHTPS_FINGER_MAX						10
#define SHTPS_FINGER_WIDTH_PALMDET				0x19

/* -----------------------------------------------------------------------------------
 */
#define SHTPS_NASSAU_SPIBLOCK_BUFSIZE			(5 + 8 * 2)
#define SHTPS_NASSAU_SPIRETRY_INTERVAL_US		1
#define SHTPS_NASSAU_RESET_AFTER_WAIT_MS		5

#define SHTPS_NASSAU_LOADER_WAIT_ACK_TMO		1000
#define SHTPS_NASSAU_LOADER_PAGE_SIZE			0x04
#define SHTPS_NASSAU_LOADER_BLOCK_SIZE			0x40

#define SHTPS_NASSAU_CMD_RESULT_MAX_SIZE		0x10
#define SHTPS_NASSAU_COMMAND_WAIT_ACK_TMO		3000

#define SHTPS_NASSAU_TMENTER_TIMEOUT_MS			3000
#define SHTPS_NASSAU_DCMAP_PAGE_SIZE			120

/* -----------------------------------------------------------------------------------
 */
#define SHTPS_FINGER_ID_INVALID					0xFF

#define SHTPS_FINGER_STATE_TD					0x06
#define SHTPS_FINGER_STATE_TU					0x08
#define SHTPS_FINGER_STATE_DR					0x00

/* -----------------------------------------------------------------------------------
 */
#define SHTPS_NAS_REG_HOSTINT					0x00
#define SHTPS_NAS_REG_BANK						0x02
#define SHTPS_NAS_REG_MICOM						0x03
#define SHTPS_NAS_REG_CMD						0x08

#define SHTPS_NAS_BANK_REPORT					0x00
#define SHTPS_NAS_BANK_CMD						0x04
#define SHTPS_NAS_BANK_CMDRESULT				0x06
#define SHTPS_NAS_BANK_ASIC						0x0D
#define SHTPS_NAS_BANK_DC1						0x30
#define SHTPS_NAS_BANK_DC2						0x31

#define SHTPS_NAS_INT_NONE						0x00
#define SHTPS_NAS_INT_MICOM						0x01
#define SHTPS_NAS_INT_RESET						0x02
#define SHTPS_NAS_INT_WAKEUP					0x04
#define SHTPS_NAS_INT_TOUCH						0x08
#define SHTPS_NAS_INT_DCMAP						0x20

/* -----------------------------------------------------------------------------------
 */
#define TPS_IOC_MAGIC					0xE0

#define TPSDEV_ENABLE						_IO  ( TPS_IOC_MAGIC,  1)
#define TPSDEV_DISABLE						_IO  ( TPS_IOC_MAGIC,  2)
#define TPSDEV_RESET						_IO  ( TPS_IOC_MAGIC,  3)
#define TPSDEV_GET_FW_VERSION				_IOR ( TPS_IOC_MAGIC,  4, struct shtps_ioctl_param)
#define TPSDEV_BL_ENTER						_IO  ( TPS_IOC_MAGIC,  5)
#define TPSDEV_BL_ERASE_SECTOR				_IO  ( TPS_IOC_MAGIC,  6)
#define TPSDEV_BL_WRITE_IMAGE				_IOW ( TPS_IOC_MAGIC,  7, struct shtps_ioctl_param)
#define TPSDEV_BL_EXIT						_IO  ( TPS_IOC_MAGIC,  8)
#define TPSDEV_GET_TOUCHINFO				_IOR ( TPS_IOC_MAGIC,  9, struct shtps_touch_info)
#define TPSDEV_GET_TOUCHINFO_UNTRANS		_IOR ( TPS_IOC_MAGIC, 10, struct shtps_touch_info)
#define TPSDEV_GET_TOUCHINFO_BLK			_IOR ( TPS_IOC_MAGIC, 11, struct shtps_touch_info)
#define TPSDEV_GET_TOUCHINFO_UNTRANS_BLK	_IOR ( TPS_IOC_MAGIC, 12, struct shtps_touch_info)
#define TPSDEV_GET_TOUCHINFO_CANCEL			_IO  ( TPS_IOC_MAGIC, 13)
#define TPSDEV_READ_REG						_IOWR( TPS_IOC_MAGIC, 14, struct shtps_ioctl_param)
#define TPSDEV_READ_ALL_REG					_IOR ( TPS_IOC_MAGIC, 15, struct shtps_ioctl_param)
#define TPSDEV_WRITE_REG					_IOW ( TPS_IOC_MAGIC, 16, struct shtps_ioctl_param)
#define TPSDEV_TESTMODE_ENTER				_IOR ( TPS_IOC_MAGIC, 17, unsigned char*)
#define TPSDEV_TESTMODE_EXIT				_IO  ( TPS_IOC_MAGIC, 18)
#define TPSDEV_TESTMODE_GETDATA				_IOR ( TPS_IOC_MAGIC, 19, struct shtps_ioctl_param)
#define TPSDEV_CALIBRATION_PARAM			_IOW ( TPS_IOC_MAGIC, 20, struct shtps_ioctl_param)
#define TPSDEV_DEBUG_REQEVENT				_IOW ( TPS_IOC_MAGIC, 21, int)
#define TPSDEV_GET_FW_VERSION_BUILTIN		_IOR ( TPS_IOC_MAGIC, 22, unsigned short)
#define TPSDEV_TESTMODE_GETDATA_NUM			_IOW ( TPS_IOC_MAGIC, 23, unsigned char*)
#define TPSDEV_BL_ERASE_SECTOR_LOADER		_IO  ( TPS_IOC_MAGIC, 24)
#define TPSDEV_BL_WRITE_IMAGE_LOADER		_IOW ( TPS_IOC_MAGIC, 25, struct shtps_ioctl_param)
#define TPSDEV_ENABLE_NO_ID					_IO  ( TPS_IOC_MAGIC, 26)
#define TPSDEV_FWUPDATE_CONTINUE			_IOW ( TPS_IOC_MAGIC, 27, int)
#define TPSDEV_BOOTLOADER_SELECT			_IOW ( TPS_IOC_MAGIC, 28, int)
#define TPSDEV_LOGOUTPUT_ENABLE				_IOW ( TPS_IOC_MAGIC, 29, int)
#define TPSDEV_SET_LOWPOWER_MODE			_IOW ( TPS_IOC_MAGIC, 30, int)
#define TPSDEV_SET_CONT_LOWPOWER_MODE		_IOW ( TPS_IOC_MAGIC, 31, int)
#define TPSDEV_SET_HIGHPOWER_MODE			_IOW ( TPS_IOC_MAGIC, 32, int)
#define TPSDEV_CALIBRATION_PEN_PARAM		_IOW ( TPS_IOC_MAGIC, 33, struct shtps_ioctl_param)

struct shtps_ioctl_param {
	int				size;
	unsigned char*	data;
};

struct shtps_touch_info {
	struct fingers{
		unsigned short	x;
		unsigned short	y;
		unsigned char	state;
		unsigned char	w;
		unsigned short	z;
		unsigned char	id;
		unsigned char	tool;
	} fingers[SHTPS_FINGER_MAX];
};

struct shtps_touch_state {
	u8				numOfFingers;
	u8				fingerStatus[SHTPS_FINGER_MAX];
	u8				dragStep[SHTPS_FINGER_MAX][2];
	u8				rezeroRequest;
	unsigned long	drag_timeout[SHTPS_FINGER_MAX][2];
	u16				dragStepReturnTime[SHTPS_FINGER_MAX][2];
};

#endif /* __SHTPS_DEF_TS2_001_H__ */
