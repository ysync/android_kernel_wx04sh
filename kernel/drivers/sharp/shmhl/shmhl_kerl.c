/* drivers/sharp/shmhl/shmhl_kerl.c (SHARP MHL Driver)
 *
 * Copyright (c) 2011, Sharp. All rights reserved.
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

// ____________________________________________________________________________________________________________________________
/**
 *	Include
 */
#include <asm/uaccess.h>
#include <asm/atomic.h>
#include <linux/kernel.h>
#include <linux/module.h>
#include <linux/init.h>
#include <linux/fs.h>
#include <linux/cdev.h>
#include <linux/device.h>
#include <linux/delay.h>
#include <linux/ioctl.h>
#include <mach/gpio.h>
#include <linux/interrupt.h>
#include <linux/sched.h>

#include <linux/wakelock.h>

#include <linux/input.h>
#include <linux/time.h>
#include <linux/workqueue.h>
#include <linux/delay.h>

#include <linux/i2c.h>
#include <linux/mfd/pm8xxx/pm8921.h>
#include <linux/usb/otg.h>

#include <linux/regulator/consumer.h> /* for regulator_xx() */

#include <sharp/shmhl_kerl.h>
#include "si_common.h"
#include "si_cra.h"
#include "si_c99support.h"
#include "si_mhl_tx_api.h"
#include "sii_hal.h"
#include "si_drv_mhl_tx.h"
#include "si_osdebug.h"

#include <linux/regulator/msm-gpio-regulator.h>
/* COORDINATOR Qualcomm1023 BUILDERR MODIFY start */
#if 0
#include "../../../arch/arm/mach-msm/board-msm8960.h"
#else
#include "../../../arch/arm/mach-msm/board-8960.h"
#endif
/* COORDINATOR Qualcomm1023 BUILDERR MODIFY end */
#include "../../video/msm/msm_fb.h"

#include "sharp/sh_smem.h"

// ____________________________________________________________________________________________________________________________
/**
 *	External variable
 */

/* Eye Pattern register value(REG_MHLTX_CTL4) set from sysfs
    0x00     : use default value
    not 0x00 : use this value
 */
uint8_t shmhl_reg_eye_pattern = 0x00;


// ____________________________________________________________________________________________________________________________
/**
 *	Internal Define
 */
#define SHMHL_DUMP_LOG(buf, size, print_size, args...)	if(NULL == buf){printk(args);}else{print_size = snprintf(buf, size, args);}
#define SHMHL_DUMP_BUF_FWD(buf, size)	if(NULL != buf){buf = buf + size;}
#define SHMHL_DEBUG_LOG_H(args...)	printk(KERN_ERR "[SHMHL] " args)
#define SHMHL_DEBUG_LOG_L(args...)	if('1' == shmhl_debug_flg[SHMHL_DEBUG_LOG_L]){printk(KERN_ERR "[SHMHL] " args);}
static char shmhl_debug_flg[SHMHL_DEBUG_LOG_MAX+1];

static char shmhl_reg_read_result[3] = {0, 0, 0};

#define SH_MHL_EDID_BLOCK_MAX	4
#define SH_MHL_EDID_OUT_BUF_SIZE	2047
static char* shmhl_edid_buf[SH_MHL_EDID_BLOCK_MAX] = {NULL, NULL, NULL, NULL};

#define SH_MHL_DEVICE_NAME		"shmhl"
#define SH_MHL_CLASS_NAME		"shmhl"
#define SH_MHL_KEY_DEVICE_NAME	"shmhl_key"
#define SH_MHL_MINORNUM_BASE	0
#define SH_MHL_DEVICE_COUNT		1

/* PMIC PORT No. */
#define SH_MHL_GPIO_REG12_EN	25
#define SH_MHL_GPIO_RESET_N		23
#define SH_MHL_MPP_5V_BOOST_EN	7

#define SH_MHL_GPIO_LO			0
#define SH_MHL_GPIO_HI			1

#define SH_MHL_REG_ID_VL23		"8921_l23"		/* for vcc18 */
#define SH_MHL_REG_VL23_VOL		1800000			/* uVol */

#define SH_MHL_DEV_ID_LEN		2

#define SH_MHL_RGND_OPEN		0x00
#define SH_MHL_RGND_1K			0x02	/* MHL */
#define SH_MHL_RGND_2K			0x01
#define SH_MHL_RGND_SHORT		0x03
#define SH_MHL_RGND_TIMEOUT_MS	500 /* ms */

#define SH_MHL_DISC_RETRY_MS	1000	/* ms */
#define SH_MHL_DISC_RETRY_MAX_CNT	150	/* times */

#define SH_MHL_DEV_CAT_MASK		0x0F
#define SH_MHL_DEV_CAT_NONE		0x00
#define SH_MHL_DEV_CAT_SINK		0x01
#define SH_MHL_DEV_CAT_SOURCE	0x02
#define SH_MHL_DEV_CAT_DONGLE	0x03

#define SH_MHL_CHG_CUR_SINK		500	/* mA */
#define SH_MHL_CHG_CUR_DONGLE	500	/* mA *//* because this is MAX value */
#define SH_MHL_CHG_CUR_USB_PC	500	/* mA */
#define SH_MHL_CHG_CUR_STOP		0	/* mA */

#define SH_MHL_RCP_KEYCODE_MAX	0x7F
#define SH_MHL_RCP_RELEASE_BIT	0x80
#define SH_MHL_RCP_HOLD_MS		470 /* ms */

// ____________________________________________________________________________________________________________________________
/**
 *	Internal decrare
 */
typedef enum
{
	SHMHL_RCP_STATE_NONE = 0,
	SHMHL_RCP_STATE_HOLD,
	SHMHL_RCP_STATE_MAX,
} shmhl_rcp_state_t;

typedef enum {
	SHMHL_BOOT_MODE_NORMAL = 0,
	SHMHL_BOOT_MODE_TESTMODE,
	SHMHL_BOOT_MODE_SOFTWARE_UPDATE,
} shmhl_boot_mode;

typedef struct
{
	struct workqueue_struct		*shmhl_connect_wq;
	struct work_struct			shmhl_init_swork;
	struct work_struct			shmhl_id_vbus_state_swork;
	struct delayed_work			shmhl_rgnd_dwork;
	struct delayed_work			shmhl_disc_retry_dwork;
	shmhl_con_state_t			con_state;
	bool						id_state;
	bool						vbus_state;
	uint16_t					rgndImpedance;
	uint8_t						peer_dev_cat;
	uint8_t						disc_retry_cnt;
	shmhl_cb_func_t				cb_func;
	void*						user_data;
} shmhl_connect_work_t;

typedef struct
{
	unsigned int		linux_key;
	bool				hold;
} shmhl_rcp_key_comvert_t;

typedef struct
{
	struct workqueue_struct		*shmhl_rcp_wq;
	struct delayed_work			shmhl_rcp_dwork;
	unsigned int				last_linux_key;
	shmhl_rcp_state_t			rcp_state;
} shmhl_rcp_work_t;

// ____________________________________________________________________________________________________________________________
/**
 *	Internal variable
 */
static int					shmhl_major = 0;
static struct cdev			shmhl_cdev;

static struct class			*shmhl_kdrv_class = NULL;
static dev_t				shmhl_kdrv_dev;
static struct input_dev*	shmhl_key = NULL;

static shmhl_connect_work_t shmhl_connect_work;
static struct mutex shmhl_connect_lock;

static shmhl_rcp_work_t shmhl_rcp_work;
static struct mutex shmhl_rcp_lock;

static char shmhl_dev_id[SH_MHL_DEV_ID_LEN];

static shmhl_rcp_key_comvert_t shmhl_rcp_key_comvert_tbl[SH_MHL_RCP_KEYCODE_MAX+1] =
{
	{ KEY_ENTER		,	false	},		/* 0x00 *//* Select */
	{ KEY_UP		,	true	},		/* 0x01 *//* Up */
	{ KEY_DOWN		,	true	},		/* 0x02 *//* Down */
	{ KEY_LEFT		,	true	},		/* 0x03 *//* Left */
	{ KEY_RIGHT		,	true	},		/* 0x04 *//* Right */
	{ KEY_RESERVED	,	true	},		/* 0x05 *//* Right-Up */
	{ KEY_RESERVED	,	true	},		/* 0x06 *//* Right-Down */
	{ KEY_RESERVED	,	true	},		/* 0x07 *//* Left-Up */
	{ KEY_RESERVED	,	true	},		/* 0x08 *//* Left-Down */
	{ KEY_HOME		,	false	},		/* 0x09 *//* Root Menu */
	{ KEY_RESERVED	,	false	},		/* 0x0A *//* Setup Menu */
	{ KEY_RESERVED	,	false	},		/* 0x0B *//* Contents Menu */
	{ KEY_RESERVED	,	false	},		/* 0x0C *//* Favorite Menu */
	{ KEY_BACK		,	false	},		/* 0x0D *//* Exit */
	{ KEY_RESERVED	,	false	},		/* 0x0E *//* Reserved */
	{ KEY_RESERVED	,	false	},		/* 0x0F *//* Reserved */
	{ KEY_RESERVED	,	false	},		/* 0x10 *//* Reserved */
	{ KEY_RESERVED	,	false	},		/* 0x11 *//* Reserved */
	{ KEY_RESERVED	,	false	},		/* 0x12 *//* Reserved */
	{ KEY_RESERVED	,	false	},		/* 0x13 *//* Reserved */
	{ KEY_RESERVED	,	false	},		/* 0x14 *//* Reserved */
	{ KEY_RESERVED	,	false	},		/* 0x15 *//* Reserved */
	{ KEY_RESERVED	,	false	},		/* 0x16 *//* Reserved */
	{ KEY_RESERVED	,	false	},		/* 0x17 *//* Reserved */
	{ KEY_RESERVED	,	false	},		/* 0x18 *//* Reserved */
	{ KEY_RESERVED	,	false	},		/* 0x19 *//* Reserved */
	{ KEY_RESERVED	,	false	},		/* 0x1A *//* Reserved */
	{ KEY_RESERVED	,	false	},		/* 0x1B *//* Reserved */
	{ KEY_RESERVED	,	false	},		/* 0x1C *//* Reserved */
	{ KEY_RESERVED	,	false	},		/* 0x1D *//* Reserved */
	{ KEY_RESERVED	,	false	},		/* 0x1E *//* Reserved */
	{ KEY_RESERVED	,	false	},		/* 0x1F *//* Reserved */
	{ KEY_0			,	false	},		/* 0x20 *//* Numeric 0 */
	{ KEY_1			,	false	},		/* 0x21 *//* Numeric 1 */
	{ KEY_2			,	false	},		/* 0x22 *//* Numeric 2 */
	{ KEY_3			,	false	},		/* 0x23 *//* Numeric 3 */
	{ KEY_4			,	false	},		/* 0x24 *//* Numeric 4 */
	{ KEY_5			,	false	},		/* 0x25 *//* Numeric 5 */
	{ KEY_6			,	false	},		/* 0x26 *//* Numeric 6 */
	{ KEY_7			,	false	},		/* 0x27 *//* Numeric 7 */
	{ KEY_8			,	false	},		/* 0x28 *//* Numeric 8 */
	{ KEY_9			,	false	},		/* 0x29 *//* Numeric 9 */
	{ KEY_RESERVED	,	false	},		/* 0x2A *//* Dot */
	{ KEY_ENTER		,	false	},		/* 0x2B *//* Enter */
	{ KEY_BACK		,	false	},		/* 0x2C *//* Clear */
	{ KEY_RESERVED	,	false	},		/* 0x2D *//* Reserved */
	{ KEY_RESERVED	,	false	},		/* 0x2E *//* Reserved */
	{ KEY_RESERVED	,	false	},		/* 0x2F *//* Reserved */
	{ KEY_RESERVED	,	true	},		/* 0x30 *//* Channel Up */
	{ KEY_RESERVED	,	true	},		/* 0x31 *//* Channel Down */
	{ KEY_RESERVED	,	false	},		/* 0x32 *//* Previous Channel */
	{ KEY_RESERVED	,	false	},		/* 0x33 *//* Sound Select */
	{ KEY_RESERVED	,	false	},		/* 0x34 *//* Input Select */
	{ KEY_RESERVED	,	false	},		/* 0x35 *//* Show Information */
	{ KEY_RESERVED	,	false	},		/* 0x36 *//* Help */
	{ KEY_RESERVED	,	true	},		/* 0x37 *//* Page Up */
	{ KEY_RESERVED	,	true	},		/* 0x38 *//* Page Down */
	{ KEY_RESERVED	,	false	},		/* 0x39 *//* Reserved */
	{ KEY_RESERVED	,	false	},		/* 0x3A *//* Reserved */
	{ KEY_RESERVED	,	false	},		/* 0x3B *//* Reserved */
	{ KEY_RESERVED	,	false	},		/* 0x3C *//* Reserved */
	{ KEY_RESERVED	,	false	},		/* 0x3D *//* Reserved */
	{ KEY_RESERVED	,	false	},		/* 0x3E *//* Reserved */
	{ KEY_RESERVED	,	false	},		/* 0x3F *//* Reserved */
	{ KEY_RESERVED	,	false	},		/* 0x40 *//* Reserved */
	{ KEY_RESERVED	,	true	},		/* 0x41 *//* Volume Up */
	{ KEY_RESERVED	,	true	},		/* 0x42 *//* Volume Down */
	{ KEY_RESERVED	,	false	},		/* 0x43 *//* Mute */
	{ KEY_PLAY		,	false	},		/* 0x44 *//* Play */
	{ KEY_STOP		,	false	},		/* 0x45 *//* Stop */
	{ KEY_PAUSE		,	false	},		/* 0x46 *//* Pause */
	{ KEY_RESERVED	,	false	},		/* 0x47 *//* Record */
	{ KEY_REWIND	,	true	},		/* 0x48 *//* Rewind */
	{ KEY_FASTFORWARD	,true	},		/* 0x49 *//* Fast Forward */
	{ KEY_RESERVED	,	false	},		/* 0x4A *//* Eject */
	{ KEY_NEXT		,	true	},		/* 0x4B *//* Forward */
	{ KEY_PREVIOUS	,	true	},		/* 0x4C *//* Backward */
	{ KEY_RESERVED	,	false	},		/* 0x4D *//* Reserved */
	{ KEY_RESERVED	,	false	},		/* 0x4E *//* Reserved */
	{ KEY_RESERVED	,	false	},		/* 0x4F *//* Reserved */
	{ KEY_RESERVED	,	false	},		/* 0x50 *//* Angle */
	{ KEY_RESERVED	,	false	},		/* 0x51 *//* Subpicture */
	{ KEY_RESERVED	,	false	},		/* 0x52 *//* Reserved */
	{ KEY_RESERVED	,	false	},		/* 0x53 *//* Reserved */
	{ KEY_RESERVED	,	false	},		/* 0x54 *//* Reserved */
	{ KEY_RESERVED	,	false	},		/* 0x55 *//* Reserved */
	{ KEY_RESERVED	,	false	},		/* 0x56 *//* Reserved */
	{ KEY_RESERVED	,	false	},		/* 0x57 *//* Reserved */
	{ KEY_RESERVED	,	false	},		/* 0x58 *//* Reserved */
	{ KEY_RESERVED	,	false	},		/* 0x59 *//* Reserved */
	{ KEY_RESERVED	,	false	},		/* 0x5A *//* Reserved */
	{ KEY_RESERVED	,	false	},		/* 0x5B *//* Reserved */
	{ KEY_RESERVED	,	false	},		/* 0x5C *//* Reserved */
	{ KEY_RESERVED	,	false	},		/* 0x5D *//* Reserved */
	{ KEY_RESERVED	,	false	},		/* 0x5E *//* Reserved */
	{ KEY_RESERVED	,	false	},		/* 0x5F *//* Reserved */
	{ KEY_PLAY		,	false	},		/* 0x60 *//* Play_Function */
	{ KEY_PLAYPAUSE	,	false	},		/* 0x61 *//* Play_Pause_Function */
	{ KEY_RESERVED	,	false	},		/* 0x62 *//* Record_Function */
	{ KEY_RESERVED	,	false	},		/* 0x63 *//* Pause_Record_Function */
	{ KEY_STOP		,	false	},		/* 0x64 *//* Stop_Function */
	{ KEY_RESERVED	,	false	},		/* 0x65 *//* Mute_Function */
	{ KEY_RESERVED	,	false	},		/* 0x66 *//* Restore_Volume_Function */
	{ KEY_RESERVED	,	false	},		/* 0x67 *//* Tune_Function */
	{ KEY_RESERVED	,	false	},		/* 0x68 *//* Select_Media_Function */
	{ KEY_RESERVED	,	false	},		/* 0x69 *//* Reserved */
	{ KEY_RESERVED	,	false	},		/* 0x6A *//* Reserved */
	{ KEY_RESERVED	,	false	},		/* 0x6B *//* Reserved */
	{ KEY_RESERVED	,	false	},		/* 0x6C *//* Reserved */
	{ KEY_RESERVED	,	false	},		/* 0x6D *//* Reserved */
	{ KEY_RESERVED	,	false	},		/* 0x6E *//* Reserved */
	{ KEY_RESERVED	,	false	},		/* 0x6F *//* Reserved */
	{ KEY_RESERVED	,	false	},		/* 0x70 *//* Reserved */
	{ KEY_BLUE		,	false	},		/* 0x71 *//* F1(Blue) */
	{ KEY_RED		,	false	},		/* 0x72 *//* F2(Red) */
	{ KEY_GREEN		,	false	},		/* 0x73 *//* F3(Green) */
	{ KEY_YELLOW	,	false	},		/* 0x74 *//* F4(Yellow) */
	{ KEY_RESERVED	,	false	},		/* 0x75 *//* F5 */
	{ KEY_RESERVED	,	false	},		/* 0x76 *//* Reserved */
	{ KEY_RESERVED	,	false	},		/* 0x77 *//* Reserved */
	{ KEY_RESERVED	,	false	},		/* 0x78 *//* Reserved */
	{ KEY_RESERVED	,	false	},		/* 0x79 *//* Reserved */
	{ KEY_RESERVED	,	false	},		/* 0x7A *//* Reserved */
	{ KEY_RESERVED	,	false	},		/* 0x7B *//* Reserved */
	{ KEY_RESERVED	,	false	},		/* 0x7C *//* Reserved */
	{ KEY_RESERVED	,	false	},		/* 0x7D *//* Reserved */
	{ KEY_RESERVED	,	false	},		/* 0x7E *//* Vendor_Specific */
	{ KEY_RESERVED	,	false	},		/* 0x7F *//* Reserved */
};

static char shmhl_shdiag_mhl_cmd[4] = SH_MHL_SHDIAG_CMD_NORMALMODE_DEFAULT;

static struct wake_lock shmhl_wake_lock;

static bool shmhl_is_usb_pc_charging = false;


// ____________________________________________________________________________________________________________________________
/**
 *	Internal prototypes
 */
static int shmhl_open(struct inode *inode, struct file *filp);
static int shmhl_close(struct inode *inode, struct file *filp);
static long shmhl_ioctl(struct file *filp, unsigned int cmd, unsigned long arg);

static int shmhl_pm_gpio_out(int port_no, int outvalue);
static int shmhl_reg_enable(const char *id, int min_uVol, int max_uVol);
static int shmhl_reg_disable(const char *id);

static void shmhl_init_comp_handler(struct work_struct *poWork);
static void shmhl_id_hi_vbus_hi_handler(struct work_struct *poWork);
static void shmhl_id_hi_vbus_low_handler(struct work_struct *poWork);
static void shmhl_id_low_vbus_hi_handler(struct work_struct *poWork);
static void shmhl_id_low_vbus_low_handler(struct work_struct *poWork);
static void shmhl_con_state_main(bool id, bool vbus);
static void shmhl_con_state_wait_plug(bool id, bool vbus);
static void shmhl_con_state_wait_unplug(bool id, bool vbus);
static void shmhl_con_state_rgnd_disc(bool id, bool vbus);
static void shmhl_con_state_mhl_est(bool id, bool vbus);
static void shmhl_set_con_state(shmhl_con_state_t con_state);
static void shmhl_rgnd_timer_handler(struct work_struct *poWork);
static void shmhl_disc_retry_handler(struct work_struct *poWork);
static void shmhl_cable_plug_unplug(bool plug);
static void shmhl_prepare_wait_plug_state(void);
#ifndef CONFIG_SII8334_MHL_TX_WITH_SWIC
static void shmhl_prepare_wait_unplug_state(void);
#endif /* CONFIG_SII8334_MHL_TX_WITH_SWIC */

static void shmhl_notify_charger_start(void);
static void shmhl_notify_charger_stop(void);

static void shmhl_rcp_start_delayed_work(unsigned int linux_key);
static void shmhl_rcp_stop_delayed_work(void);
static void shmhl_rcp_set_state(shmhl_rcp_state_t new_state);
static shmhl_rcp_state_t shmhl_rcp_get_state(void);
static void shmhl_rcp_delay_handler(struct work_struct *poWork);

static shmhl_boot_mode shmhl_get_boot_mode_from_smem(void);

static void shmhl_free_edid_out_buf(void);

static struct file_operations shmhl_fops = {
    .owner   = THIS_MODULE,
    .open    = shmhl_open,
    .release = shmhl_close,
    .unlocked_ioctl   = shmhl_ioctl,
};

// ____________________________________________________________________________________________________________________________
/**
 *	Main Functions
 */

void shmhl_notify_si_init_comp(void)
{
	SHMHL_DEBUG_LOG_L("shmhl_notify_si_init_comp() Start\n");
	mutex_lock(&shmhl_connect_lock);

	queue_work(shmhl_connect_work.shmhl_connect_wq, &shmhl_connect_work.shmhl_init_swork);

	mutex_unlock(&shmhl_connect_lock);
	SHMHL_DEBUG_LOG_L("shmhl_notify_si_init_comp() End\n");
}

int shmhl_set_port_vcc12_up(void)
{
	int ret = -EFAULT;

	ret = shmhl_pm_gpio_out( SH_MHL_GPIO_REG12_EN, SH_MHL_GPIO_HI );

	return ret;
}

int shmhl_set_port_vcc12_down(void)
{
	int ret = -EFAULT;

	ret = shmhl_pm_gpio_out( SH_MHL_GPIO_REG12_EN, SH_MHL_GPIO_LO );

	return ret;
}

int shmhl_set_port_vcc5_up(void)
{
	int ret = -EFAULT;

	return ret;
}

int shmhl_set_port_vcc5_down(void)
{
	int ret = -EFAULT;

	return ret;
}

int shmhl_set_port_vcc18_up(void)
{
	int ret = -EFAULT;

	ret = shmhl_reg_enable(SH_MHL_REG_ID_VL23, SH_MHL_REG_VL23_VOL, SH_MHL_REG_VL23_VOL);
	if (ret) {
		SHMHL_DEBUG_LOG_H("%s: ERR:%d\n", __func__, ret);
	}

	return ret;
}

int shmhl_set_port_vcc18_down(void)
{
	int ret = -EFAULT;

	ret = shmhl_reg_disable(SH_MHL_REG_ID_VL23);
	if (ret) {
		SHMHL_DEBUG_LOG_H("%s: ERR:%d\n", __func__, ret);
	}

	return ret;
}

void shmhl_set_dev_id(uint8_t low, uint8_t high)
{
	SHMHL_DEBUG_LOG_L("shmhl_set_dev_id() [0x%02x][0x%02x]\n", low, high);

	shmhl_dev_id[0] = low;
	shmhl_dev_id[1] = high;
}

void shmhl_detect_cb_regist(shmhl_cb_func_t cb_func, void* user_data)
{
	SHMHL_DEBUG_LOG_L("shmhl_detect_cb_regist() Start\n");

	if(NULL == cb_func) {
		SHMHL_DEBUG_LOG_H("shmhl_detect_cb_regist() NULL!\n");
	}

	mutex_lock(&shmhl_connect_lock);

	shmhl_connect_work.cb_func = cb_func;
	shmhl_connect_work.user_data = user_data;

	mutex_unlock(&shmhl_connect_lock);

	SHMHL_DEBUG_LOG_L("shmhl_detect_cb_regist() End\n");
}

void shmhl_notify_id_low(shmhl_vbus_status_t vbus)
{
	SHMHL_DEBUG_LOG_L("shmhl_notify_id_low() Start vbus[%d]\n", vbus);

	mutex_lock(&shmhl_connect_lock);

	if(SHMHL_VBUS_STATE_OFF == vbus) {
		PREPARE_WORK(&shmhl_connect_work.shmhl_id_vbus_state_swork, shmhl_id_low_vbus_low_handler);
	}
	else {
		PREPARE_WORK(&shmhl_connect_work.shmhl_id_vbus_state_swork, shmhl_id_low_vbus_hi_handler);
	}

	queue_work(shmhl_connect_work.shmhl_connect_wq, &shmhl_connect_work.shmhl_id_vbus_state_swork);

	mutex_unlock(&shmhl_connect_lock);

	SHMHL_DEBUG_LOG_L("shmhl_notify_id_low() End\n");
}

void shmhl_notify_id_vbus_state(bool id, bool vbus)
{
	void* handler_func = NULL;

	SHMHL_DEBUG_LOG_L("shmhl_notify_id_vbus_state() Start [id:%d] [vbus:%d]\n", id, vbus);

	mutex_lock(&shmhl_connect_lock);

	if(id) {
		if(vbus) {
			handler_func = (void*)shmhl_id_hi_vbus_hi_handler;
		}
		else {
			handler_func = (void*)shmhl_id_hi_vbus_low_handler;
		}
	}
	else {
		if(vbus) {
			handler_func = (void*)shmhl_id_low_vbus_hi_handler;
		}
		else {
			handler_func = (void*)shmhl_id_low_vbus_low_handler;
		}
	}

	PREPARE_WORK(&shmhl_connect_work.shmhl_id_vbus_state_swork, handler_func);
	queue_work(shmhl_connect_work.shmhl_connect_wq, &shmhl_connect_work.shmhl_id_vbus_state_swork);

	mutex_unlock(&shmhl_connect_lock);

	SHMHL_DEBUG_LOG_L("shmhl_notify_id_vbus_state() End\n");
}

static void shmhl_init_comp_handler(struct work_struct *poWork)
{
	SHMHL_DEBUG_LOG_L("shmhl_init_comp_handler() Start\n");

	if (HalAcquireIsrLock() == HAL_RET_SUCCESS) {
		/* If we haven't get dev_id yet */
		if((0 == shmhl_dev_id[0]) && (0 == shmhl_dev_id[1])) {
			/* Sii8334 Init */
			SiiMhlTxInitialize(0);
		}
		HalReleaseIsrLock();
	}

	SHMHL_DEBUG_LOG_L("shmhl_init_comp_handler() End\n");
}

static void shmhl_id_hi_vbus_hi_handler(struct work_struct *poWork)
{
	SHMHL_DEBUG_LOG_L("shmhl_id_hi_vbus_hi_handler() Start\n");

	shmhl_con_state_main(true, true);

	SHMHL_DEBUG_LOG_L("shmhl_id_hi_vbus_hi_handler() End\n");
}

static void shmhl_id_hi_vbus_low_handler(struct work_struct *poWork)
{
	SHMHL_DEBUG_LOG_L("shmhl_id_hi_vbus_low_handler() Start\n");

	shmhl_con_state_main(true, false);

	SHMHL_DEBUG_LOG_L("shmhl_id_hi_vbus_low_handler() End\n");
}

static void shmhl_id_low_vbus_hi_handler(struct work_struct *poWork)
{
	SHMHL_DEBUG_LOG_L("shmhl_id_low_vbus_hi_handler() Start\n");

	shmhl_con_state_main(false, true);

	SHMHL_DEBUG_LOG_L("shmhl_id_low_vbus_hi_handler() End\n");
}

static void shmhl_id_low_vbus_low_handler(struct work_struct *poWork)
{
	SHMHL_DEBUG_LOG_L("shmhl_id_low_vbus_low_handler() Start\n");

	shmhl_con_state_main(false, false);

	SHMHL_DEBUG_LOG_L("shmhl_id_low_vbus_low_handler() End\n");
}

static void shmhl_con_state_main(bool id, bool vbus)
{
	SHMHL_DEBUG_LOG_L("shmhl_con_state_main() Start\n");

	if(HalAcquireIsrLock() == HAL_RET_SUCCESS) {

		if((id == shmhl_connect_work.id_state) && (vbus == shmhl_connect_work.vbus_state)) {
			if(SHMHL_CON_STATE_WAIT_UNPLUG != shmhl_connect_work.con_state) {
				SHMHL_DEBUG_LOG_L("Already id=%d vbus=%d\n", id, vbus);
				HalReleaseIsrLock();
				return;
			}
		}

		switch(shmhl_connect_work.con_state) {
			case SHMHL_CON_STATE_WAIT_PLUG:
				SHMHL_DEBUG_LOG_L("Now SHMHL_CON_STATE_WAIT_PLUG\n");
				shmhl_con_state_wait_plug(id, vbus);
				break;

			case SHMHL_CON_STATE_WAIT_UNPLUG:
				SHMHL_DEBUG_LOG_L("Now SHMHL_CON_STATE_WAIT_UNPLUG\n");
				shmhl_con_state_wait_unplug(id, vbus);
				break;

			case SHMHL_CON_STATE_RGND:
			case SHMHL_CON_STATE_DISCOVERY:
				SHMHL_DEBUG_LOG_L("Now SHMHL_CON_STATE RGND/DISC\n");
				shmhl_con_state_rgnd_disc(id, vbus);
				break;

			case SHMHL_CON_STATE_MHL_EST:
			case SHMHL_CON_STATE_HAVE_DEV_CAT:
				SHMHL_DEBUG_LOG_L("Now SHMHL_CON_STATE MHL_EST/HAVE_DEV_CAT\n");
				shmhl_con_state_mhl_est(id, vbus);
				break;

			default:
				break;
		}
		HalReleaseIsrLock();
	}

	SHMHL_DEBUG_LOG_L("shmhl_con_state_main() End\n");
}

static void shmhl_con_state_wait_plug(bool id, bool vbus)
{
	SHMHL_DEBUG_LOG_L("shmhl_con_state_wait_plug() Start [id:%d] [vbus:%d]\n", id, vbus);

	/* Cable Pluged */
	if((shmhl_connect_work.id_state) && (!id)) {
		SHMHL_DEBUG_LOG_L("shmhl_con_state_wait_plug() ID HI --> LOW\n");

		shmhl_connect_work.id_state = false;
		shmhl_connect_work.vbus_state = vbus;

		/* Sii8334 Start */
		shmhl_cable_plug_unplug(true);
	}
	else {
		SHMHL_DEBUG_LOG_L("shmhl_con_state_wait_plug() ignore\n");
		/* CallBack */
		if(NULL != shmhl_connect_work.cb_func){
			SHMHL_DEBUG_LOG_L("%s() callback call start\n", __func__);
			shmhl_connect_work.cb_func(SHMHL_DEVICE_NONE, shmhl_connect_work.user_data);
			SHMHL_DEBUG_LOG_L("%s() callback call end\n", __func__);
		}
	}

	SHMHL_DEBUG_LOG_L("shmhl_con_state_wait_plug() End\n");
}

static void shmhl_con_state_wait_unplug(bool id, bool vbus)
{
	/* Dongle that is powerless or HOSIDEN DONGLE that is powered but HDMI cable is not connected, */
	/* stay in this state. */

	SHMHL_DEBUG_LOG_L("shmhl_con_state_wait_unplug() Start [id:%d] [vbus:%d]\n", id, vbus);

	/* Cable Unpluged */
	if((!shmhl_connect_work.id_state) && (id)) {
		SHMHL_DEBUG_LOG_L("shmhl_con_state_wait_unplug() ID LOW --> HI\n");

		wake_lock(&shmhl_wake_lock);

		/* Sii8334 Stop */
		shmhl_cable_plug_unplug(false);
	}
	/* When HDMI cable connected, HOSIDEN DONGLE toggles VBUS very quickly LO->HI once. */
	/* So in this state, we ignore last vbus_state. */
	/* If vbus(=2nd param) is HI, we do these steps. */
	else if(vbus) { 
		/* VBUS Powered */
		if(shmhl_connect_work.vbus_state) {
			SHMHL_DEBUG_LOG_L("shmhl_con_state_wait_unplug() VBUS HI --> HI\n");
		}
		else {
			SHMHL_DEBUG_LOG_L("shmhl_con_state_wait_unplug() VBUS LOW --> HI\n");
		}

		shmhl_connect_work.id_state = false;
		shmhl_connect_work.vbus_state = true;

		/* Sii8334 Start */
		shmhl_cable_plug_unplug(true);
	}
	/* If last vbus_state is Hi, and vbus(=2nd param) is Lo. */
	else if(shmhl_connect_work.vbus_state) {
		SHMHL_DEBUG_LOG_L("shmhl_con_state_wait_unplug() VBUS HI --> LOW\n");
		shmhl_connect_work.vbus_state = false;
	}
	else {
		SHMHL_DEBUG_LOG_L("shmhl_con_state_wait_unplug() ignore\n");
#ifdef CONFIG_SII8334_MHL_TX_WITH_SWIC
		/* CallBack */
		if(NULL != shmhl_connect_work.cb_func){
			SHMHL_DEBUG_LOG_L("%s() callback call start\n", __func__);
			shmhl_connect_work.cb_func(SHMHL_DEVICE_NONE, shmhl_connect_work.user_data);
			SHMHL_DEBUG_LOG_L("%s() callback call end\n", __func__);
		}
#endif /* CONFIG_SII8334_MHL_TX_WITH_SWIC */
	}

	SHMHL_DEBUG_LOG_L("shmhl_con_state_wait_unplug() End\n");
}

static void shmhl_con_state_rgnd_disc(bool id, bool vbus)
{
	SHMHL_DEBUG_LOG_L("shmhl_con_state_rgnd_disc() Start [id:%d] [vbus:%d]\n", id, vbus);

	/* Cable Unpluged */
	if((shmhl_connect_work.vbus_state) && (!vbus)) {
		SHMHL_DEBUG_LOG_L("shmhl_con_state_rgnd_disc() VBUS ON --> OFF\n");
		/* Sii8334 Stop */
		shmhl_cable_plug_unplug(false);
	}
	/* VBUS Powerd */
	else if((!shmhl_connect_work.vbus_state) && (vbus)) {
		SHMHL_DEBUG_LOG_L("shmhl_con_state_rgnd_disc() VBUS OFF --> ON\n");
		shmhl_connect_work.vbus_state = true;
	}
	else {
		SHMHL_DEBUG_LOG_L("shmhl_con_state_rgnd_disc() ignore\n");
#ifdef CONFIG_SII8334_MHL_TX_WITH_SWIC
		/* CallBack */
		if(NULL != shmhl_connect_work.cb_func){
			SHMHL_DEBUG_LOG_L("%s() callback call start\n", __func__);
			shmhl_connect_work.cb_func(SHMHL_DEVICE_NONE, shmhl_connect_work.user_data);
			SHMHL_DEBUG_LOG_L("%s() callback call end\n", __func__);
		}
#endif /* CONFIG_SII8334_MHL_TX_WITH_SWIC */
	}

	SHMHL_DEBUG_LOG_L("shmhl_con_state_rgnd_disc() End\n");
}

static void shmhl_con_state_mhl_est(bool id, bool vbus)
{
	SHMHL_DEBUG_LOG_L("shmhl_con_state_mhl_est() Start [id:%d] [vbus:%d]\n", id, vbus);

	/* Cable Unpluged */
	if((shmhl_connect_work.vbus_state) && (!vbus)) {
		SHMHL_DEBUG_LOG_L("shmhl_con_state_mhl_est() VBUS ON --> OFF\n");

		/* Sii8334 Stop */
		shmhl_cable_plug_unplug(false);
	}
	/* VBUS Powerd */
	else if((!shmhl_connect_work.vbus_state) && (vbus)) {
		SHMHL_DEBUG_LOG_L("shmhl_con_state_mhl_est() VBUS OFF --> ON\n");
		shmhl_connect_work.vbus_state = true;
		shmhl_notify_charger_start();
	}
	else {
		SHMHL_DEBUG_LOG_L("shmhl_con_state_mhl_est() ignore\n");
#ifdef CONFIG_SII8334_MHL_TX_WITH_SWIC
		/* CallBack */
		if(NULL != shmhl_connect_work.cb_func){
			SHMHL_DEBUG_LOG_L("%s() callback call start\n", __func__);
			shmhl_connect_work.cb_func(SHMHL_DEVICE_NONE, shmhl_connect_work.user_data);
			SHMHL_DEBUG_LOG_L("%s() callback call end\n", __func__);
		}
#endif /* CONFIG_SII8334_MHL_TX_WITH_SWIC */
	}

	SHMHL_DEBUG_LOG_L("shmhl_con_state_mhl_est() End\n");
}

static void shmhl_set_con_state(shmhl_con_state_t con_state)
{
	char from_string[48];
	char to_string[48];

	memset(from_string, 0, sizeof(from_string));
	memset(from_string, 0, sizeof(to_string));

	switch(shmhl_connect_work.con_state) {
			case SHMHL_CON_STATE_WAIT_PLUG:
				strncpy(from_string, "WAIT_PLUG", sizeof(from_string));
				break;

			case SHMHL_CON_STATE_WAIT_UNPLUG:
				strncpy(from_string, "WAIT_UNPLUG", sizeof(from_string));
				break;

			case SHMHL_CON_STATE_RGND:
				strncpy(from_string, "RGND", sizeof(from_string));
				break;

			case SHMHL_CON_STATE_DISCOVERY:
				strncpy(from_string, "DISCOVERY", sizeof(from_string));
				break;

			case SHMHL_CON_STATE_MHL_EST:
				strncpy(from_string, "MHL_EST", sizeof(from_string));
				break;

			case SHMHL_CON_STATE_HAVE_DEV_CAT:
				strncpy(from_string, "HAVE_DEV_CAT", sizeof(from_string));
				break;

			default:
				break;
	}

	switch(con_state) {
			case SHMHL_CON_STATE_WAIT_PLUG:
				strncpy(to_string, "WAIT_PLUG", sizeof(to_string));
				break;

			case SHMHL_CON_STATE_WAIT_UNPLUG:
				strncpy(to_string, "WAIT_UNPLUG", sizeof(to_string));
				break;

			case SHMHL_CON_STATE_RGND:
				strncpy(to_string, "RGND", sizeof(to_string));
				break;

			case SHMHL_CON_STATE_DISCOVERY:
				strncpy(to_string, "DISCOVERY", sizeof(to_string));
				break;

			case SHMHL_CON_STATE_MHL_EST:
				strncpy(to_string, "MHL_EST", sizeof(to_string));
				break;

			case SHMHL_CON_STATE_HAVE_DEV_CAT:
				strncpy(to_string, "HAVE_DEV_CAT", sizeof(to_string));
				break;

			default:
				break;
	}

	SHMHL_DEBUG_LOG_L("State Change from [%s] to [%s]\n", from_string, to_string);

	shmhl_connect_work.con_state = con_state;
}

static void shmhl_rgnd_timer_handler(struct work_struct *poWork)
{
	SHMHL_DEBUG_LOG_L("shmhl_rgnd_timer_handler() Start\n");

	if(HalAcquireIsrLock() == HAL_RET_SUCCESS) {

		if(shmhl_connect_work.con_state == SHMHL_CON_STATE_RGND) {
			/* Sii8334 Stop */
			shmhl_cable_plug_unplug(false);
		}

		HalReleaseIsrLock();
	}

	SHMHL_DEBUG_LOG_L("shmhl_rgnd_timer_handler() End\n");
}

static void shmhl_disc_retry_handler(struct work_struct *poWork)
{
	SHMHL_DEBUG_LOG_L("shmhl_disc_retry_handler() Start\n");

	if(HalAcquireIsrLock() == HAL_RET_SUCCESS) {
		if(shmhl_connect_work.con_state == SHMHL_CON_STATE_DISCOVERY) {
			/* Sii8334 Start */
			shmhl_cable_plug_unplug(true);
		}
		else {
			SHMHL_DEBUG_LOG_L("shmhl_disc_retry_handler() ignore\n");
		}
		HalReleaseIsrLock();
	}

	SHMHL_DEBUG_LOG_L("shmhl_disc_retry_handler() End\n");
}

static void shmhl_cable_plug_unplug(bool plug)
{
	if(plug) {
		SHMHL_DEBUG_LOG_L("shmhl_sii8334 Start\n");

		shmhl_set_con_state(SHMHL_CON_STATE_RGND);
		wake_lock(&shmhl_wake_lock);

		/* Sii8334 Start */
		SiiMhlTxInitialize(0);

		queue_delayed_work(shmhl_connect_work.shmhl_connect_wq,
							&shmhl_connect_work.shmhl_rgnd_dwork,
							msecs_to_jiffies(SH_MHL_RGND_TIMEOUT_MS));
	}
	else {
		SHMHL_DEBUG_LOG_L("shmhl_sii8334 Stop\n");

		/* Sii8334 Stop */
		shmhl_force_disconnect();
	}
}

static void shmhl_prepare_wait_plug_state(void)
{
	SHMHL_DEBUG_LOG_L("shmhl_prepare_wait_plug_state() Start\n");

	shmhl_connect_work.id_state = true;
	shmhl_connect_work.vbus_state = false;
	shmhl_connect_work.rgndImpedance = SH_MHL_RGND_OPEN;
	shmhl_connect_work.peer_dev_cat = SH_MHL_DEV_CAT_NONE;
	shmhl_connect_work.disc_retry_cnt = 0;

	SHMHL_DEBUG_LOG_L("shmhl_prepare_wait_plug_state() End\n");
}

#ifndef CONFIG_SII8334_MHL_TX_WITH_SWIC
static void shmhl_prepare_wait_unplug_state(void)
{
	SHMHL_DEBUG_LOG_L("shmhl_prepare_wait_unplug_state() Start\n");

	shmhl_connect_work.disc_retry_cnt = 0;

	SHMHL_DEBUG_LOG_L("shmhl_prepare_wait_unplug_state() End\n");
}
#endif /* CONFIG_SII8334_MHL_TX_WITH_SWIC */

void shmhl_notify_si_state(shmhl_si_status_t si_state, void* data)
{
	bool cb_flg = false;
	shmhl_detect_device_t cb_device;

	SHMHL_DEBUG_LOG_L("shmhl_notify_si_state() Start\n");

	mutex_lock(&shmhl_connect_lock);
	cancel_delayed_work(&shmhl_connect_work.shmhl_disc_retry_dwork);

	switch(si_state) {
		case SHMHL_SI_STATE_RGND:
			cancel_delayed_work(&shmhl_connect_work.shmhl_rgnd_dwork);

			shmhl_connect_work.rgndImpedance = *((uint8_t*)data);
			SHMHL_DEBUG_LOG_L("case SHMHL_SI_STATE_RGND Imp=0x%02x\n", shmhl_connect_work.rgndImpedance);

			if(shmhl_connect_work.rgndImpedance == 0x02) {
				shmhl_notify_usb_pc_charge(false);
			}

			shmhl_set_con_state(SHMHL_CON_STATE_DISCOVERY);

			break;
		
		case SHMHL_SI_STATE_MHL_EST:
			SHMHL_DEBUG_LOG_L("case SHMHL_SI_STATE_MHL_EST\n");

			if(shmhl_get_debug_flg(SHMHL_DEBUG_LOG_DUMP)) {
				printk("Reg_Dump(After MHL_EST) Start -------------------------------\n");
				shmhl_SiI8334_Reg_Dump_All_Offset(NULL, 0);
				printk("Reg_Dump(After MHL_EST) End -------------------------------\n");
			}

			if(0 == shmhl_connect_work.disc_retry_cnt) {
				cb_flg = true;
				cb_device = SHMHL_DEVICE_MHL;
			}
			/* during retrying */
			else {
				shmhl_connect_work.disc_retry_cnt = 0;
			}

			shmhl_set_con_state(SHMHL_CON_STATE_MHL_EST);

			break;

		case SHMHL_SI_STATE_DISC_FAIL:
			SHMHL_DEBUG_LOG_L("case SHMHL_SI_STATE_DISC_FAIL\n");

			/* Not MHL */
			if(SH_MHL_RGND_SHORT == shmhl_connect_work.rgndImpedance) {
				SHMHL_DEBUG_LOG_L("SH_MHL_RGND_SHORT [vbus:%d]\n", shmhl_connect_work.vbus_state);
				cb_flg = true;

				if(shmhl_connect_work.vbus_state) {
					cb_device = SHMHL_DEVICE_UNKNOWN;
				}
				else {
					cb_device = SHMHL_DEVICE_USB_B;
				}

				shmhl_prepare_wait_plug_state();
				shmhl_set_con_state(SHMHL_CON_STATE_WAIT_PLUG);
				wake_unlock(&shmhl_wake_lock);
			}
			/* MHL */
			else if(SH_MHL_RGND_1K == shmhl_connect_work.rgndImpedance) {
				SHMHL_DEBUG_LOG_L("SH_MHL_RGND_1K\n");

				if(SH_MHL_DISC_RETRY_MAX_CNT > shmhl_connect_work.disc_retry_cnt) {

					/* callback at once */
					if(0 == shmhl_connect_work.disc_retry_cnt) {
						cb_flg = true;
						cb_device = SHMHL_DEVICE_MHL;
					}

					/* retry */
					shmhl_connect_work.disc_retry_cnt++;
					SHMHL_DEBUG_LOG_L("Kick Discovery Retry [%dth]\n", shmhl_connect_work.disc_retry_cnt);
					queue_delayed_work(shmhl_connect_work.shmhl_connect_wq,
										&shmhl_connect_work.shmhl_disc_retry_dwork,
										msecs_to_jiffies(SH_MHL_DISC_RETRY_MS));
				}
				else {
					SHMHL_DEBUG_LOG_L("Discovery Retry counter Over!\n");
#ifdef CONFIG_SII8334_MHL_TX_WITH_SWIC
					/* With SWIC */
					/* SWIC can't detect ID Low when MHL is working. */
					/* So we set the MHL state to disconnected, and call back to SWIC driver. */
					cb_flg = true;
					cb_device = SHMHL_DEVICE_DISC_RETRY_OVER;
					shmhl_prepare_wait_plug_state();
					shmhl_set_con_state(SHMHL_CON_STATE_WAIT_PLUG);
#else /* CONFIG_SII8334_MHL_TX_WITH_SWIC */
					/* No SWIC */
					shmhl_prepare_wait_unplug_state();
					shmhl_set_con_state(SHMHL_CON_STATE_WAIT_UNPLUG);
#endif /* CONFIG_SII8334_MHL_TX_WITH_SWIC */
					wake_unlock(&shmhl_wake_lock);
				}
			}
			/* Not MHL *//* Open or 2K */
			else {
				SHMHL_DEBUG_LOG_L("SH_MHL_RGND_OPEN or SH_MHL_RGND_2K\n");
				cb_flg = true;
				cb_device = SHMHL_DEVICE_ACA_MCPC_CHG;

				shmhl_prepare_wait_plug_state();
				shmhl_set_con_state(SHMHL_CON_STATE_WAIT_PLUG);
				wake_unlock(&shmhl_wake_lock);
			}
			break;

		case SHMHL_SI_STATE_HAVE_DEV_CAT:
		{
			uint8_t peer_dev_cat = *((uint8_t*)data);

			SHMHL_DEBUG_LOG_L("case SHMHL_SI_STATE_HAVE_DEV_CAT\n");
			shmhl_connect_work.peer_dev_cat = (SH_MHL_DEV_CAT_MASK & peer_dev_cat);

			if(shmhl_connect_work.vbus_state) {
				shmhl_notify_charger_start();
			}

			shmhl_set_con_state(SHMHL_CON_STATE_HAVE_DEV_CAT);

			break;
		}

		case SHMHL_SI_STATE_DISCONNECTED:
			SHMHL_DEBUG_LOG_L("case SHMHL_SI_STATE_DISCONNECTED\n");
			shmhl_notify_charger_stop();

			cb_flg = true;
			cb_device = SHMHL_DEVICE_NONE;

			shmhl_prepare_wait_plug_state();
			shmhl_set_con_state(SHMHL_CON_STATE_WAIT_PLUG);
			wake_unlock(&shmhl_wake_lock);
			break;

		case SHMHL_SI_STATE_NULL:
		case SHMHL_SI_STATE_MAX:
		default:
			SHMHL_DEBUG_LOG_H("case Error!\n");
			break;
	}

	/* CallBack */
	if((cb_flg) && (NULL != shmhl_connect_work.cb_func)){
		SHMHL_DEBUG_LOG_L("shmhl_notify_si_state() callback call start\n");
		shmhl_connect_work.cb_func(cb_device, shmhl_connect_work.user_data);
		SHMHL_DEBUG_LOG_L("shmhl_notify_si_state() callback call end\n");
	}

	mutex_unlock(&shmhl_connect_lock);

	SHMHL_DEBUG_LOG_L("shmhl_notify_si_state() End\n");
}

shmhl_con_state_t shmhl_get_con_state(void)
{
	SHMHL_DEBUG_LOG_L("%s shmhl_con_state_t:%d\n", __func__, shmhl_connect_work.con_state);
	return shmhl_connect_work.con_state;
}

void shmhl_notify_usb_pc_charge(bool shmhl_usb_pc_charge_state)
{
	if(shmhl_usb_pc_charge_state) {
		if(!shmhl_is_usb_pc_charging) {
			SHMHL_DEBUG_LOG_L("Charge from SWIC for D+/- Open Charger mA=%u\n", SH_MHL_CHG_CUR_USB_PC);
			msm_notify_chg_info_from_mhl_irregular(SH_MHL_CHG_CUR_USB_PC);
			shmhl_is_usb_pc_charging = true;
		}
		else {
			SHMHL_DEBUG_LOG_L("Already Charge Started.\n");
		}
	}
	else {
		if(shmhl_is_usb_pc_charging) {
			SHMHL_DEBUG_LOG_L("Charge from SWIC for D+/- Open Charger mA=%u\n", SH_MHL_CHG_CUR_STOP);
			msm_notify_chg_info_from_mhl_irregular(SH_MHL_CHG_CUR_STOP);
			shmhl_is_usb_pc_charging = false;
		}
		else {
			SHMHL_DEBUG_LOG_L("Already Charge Stoped.\n");
		}
	}
}

static void shmhl_notify_charger_start(void)
{
	unsigned draw_mA = SH_MHL_CHG_CUR_STOP;

	SHMHL_DEBUG_LOG_L("shmhl_notify_charger_start() Start\n");

	if(shmhl_is_usb_pc_charging) {
		SHMHL_DEBUG_LOG_L("usb-pc charging STOP! \n");
		shmhl_notify_charger_stop();
	}

	if(SH_MHL_DEV_CAT_SINK == shmhl_connect_work.peer_dev_cat) {
		draw_mA = SH_MHL_CHG_CUR_SINK;
	}
	else if(SH_MHL_DEV_CAT_DONGLE == shmhl_connect_work.peer_dev_cat) {
		draw_mA = SH_MHL_CHG_CUR_DONGLE;
	}
	else {
		SHMHL_DEBUG_LOG_L("SH_MHL_DEV_CAT UNKNOWN\n");
		return;
	}

	SHMHL_DEBUG_LOG_L("Charge from MHL mA=%u\n", draw_mA);

	msm_notify_chg_info_from_mhl(draw_mA);

	SHMHL_DEBUG_LOG_L("shmhl_notify_charger_start() End\n");
}

static void shmhl_notify_charger_stop(void)
{
	SHMHL_DEBUG_LOG_L("Charge from MHL mA=%u\n", SH_MHL_CHG_CUR_STOP);

	msm_notify_chg_info_from_mhl(SH_MHL_CHG_CUR_STOP);
	shmhl_is_usb_pc_charging = false;
}

bool shmhl_rcp_key_support_judge(uint8_t rcp_data)
{
	if( KEY_RESERVED == shmhl_rcp_key_comvert_tbl[rcp_data & 0x7F].linux_key ) {
		SHMHL_DEBUG_LOG_L("shmhl_rcp_key_support_judge() [Not Support]\n");
		return false;
	}
	else {
		SHMHL_DEBUG_LOG_L("shmhl_rcp_key_support_judge() [Support]\n");
		return true;
	}
}

void shmhl_input_rcp_key(uint8_t rcp_data)
{
	bool press = true;
	uint8_t rcp_key = (rcp_data & 0x7F);
	uint16_t linux_key = shmhl_rcp_key_comvert_tbl[rcp_key].linux_key;
	bool hold = shmhl_rcp_key_comvert_tbl[rcp_key].hold;
	shmhl_rcp_state_t now_state = SHMHL_RCP_STATE_NONE;

	mutex_lock(&shmhl_rcp_lock);

	if( NULL == shmhl_key ) {
		SHMHL_DEBUG_LOG_H("key device not register!\n");
		mutex_unlock(&shmhl_rcp_lock);
		return;
	}

	if( (rcp_data & SH_MHL_RCP_RELEASE_BIT) == SH_MHL_RCP_RELEASE_BIT) {
		/* Key Release */
		press = 0;
	}

	SHMHL_DEBUG_LOG_L("shmhl_input_rcp_key() rcp_data[0x%x] press[%d] linux_key[%d(0x%x)] hold[%d]\n",
						rcp_data, press, linux_key, linux_key, hold);

	now_state = shmhl_rcp_get_state();

	switch(now_state) {
		case SHMHL_RCP_STATE_NONE:
			SHMHL_DEBUG_LOG_L("shmhl_input_rcp_key() STATE_NONE now\n");

			/* Press */
			if(press) {
				SHMHL_DEBUG_LOG_L("shmhl_input_rcp_key() Press\n");
				if(hold) {
					SHMHL_DEBUG_LOG_L("shmhl_input_rcp_key() Hold\n");
					input_report_key(shmhl_key, linux_key, 1);							/* Press new_key */
					input_sync(shmhl_key);
					shmhl_rcp_start_delayed_work(linux_key);							/* Start Timer for Release */
				}
				else {
					SHMHL_DEBUG_LOG_L("shmhl_input_rcp_key() AutoRelease\n");
					input_report_key(shmhl_key, linux_key, 1);							/* Press new_key */
					input_report_key(shmhl_key, linux_key, 0);							/* Release new_key */
					input_sync(shmhl_key);
				}
			}
			/* Release */
			else {
				SHMHL_DEBUG_LOG_L("shmhl_input_rcp_key() Release\n");
				if(hold) {
					SHMHL_DEBUG_LOG_L("shmhl_input_rcp_key() Hold\n");
					input_report_key(shmhl_key, linux_key, 0);							/* Release new_key */
					input_sync(shmhl_key);
				}
				else {
					SHMHL_DEBUG_LOG_L("shmhl_input_rcp_key() Ignore\n");
				}
			}
			break;

		case SHMHL_RCP_STATE_HOLD:
			SHMHL_DEBUG_LOG_L("shmhl_input_rcp_key() STATE_HOLD now\n");

			shmhl_rcp_stop_delayed_work();												/* Stop Timer */

			/* Press */
			if(press) {
				SHMHL_DEBUG_LOG_L("shmhl_input_rcp_key() Press\n");
				if(hold) {
					SHMHL_DEBUG_LOG_L("shmhl_input_rcp_key() Hold\n");
					if( shmhl_rcp_work.last_linux_key == linux_key ) {
						SHMHL_DEBUG_LOG_L("shmhl_input_rcp_key() SameKey ->Restart[%d(0x%x)]\n", linux_key, linux_key);
						shmhl_rcp_start_delayed_work(linux_key);						/* Restart Timer */
					}
					else {
						SHMHL_DEBUG_LOG_L("shmhl_input_rcp_key() DiffKey ->Release[%d(0x%x)] ->Start[%d(0x%x)]\n",
											shmhl_rcp_work.last_linux_key, shmhl_rcp_work.last_linux_key,
											linux_key, linux_key);
						input_report_key(shmhl_key, shmhl_rcp_work.last_linux_key, 0);	/* Release last_key */
						shmhl_rcp_work.last_linux_key = KEY_RESERVED;

						input_report_key(shmhl_key, linux_key, 1);						/* Press new_key */
    					input_sync(shmhl_key);
						shmhl_rcp_start_delayed_work(linux_key);						/* Start Timer for Release */
					}
				}
				else {
					SHMHL_DEBUG_LOG_L("shmhl_input_rcp_key() DiffKey ->Release[%d(0x%x)] ->Press and AutoRelease[%d(0x%x)]\n",
										shmhl_rcp_work.last_linux_key, shmhl_rcp_work.last_linux_key,
										linux_key, linux_key);
					input_report_key(shmhl_key, shmhl_rcp_work.last_linux_key, 0);		/* Release last_key */
					shmhl_rcp_work.last_linux_key = KEY_RESERVED;

					input_report_key(shmhl_key, linux_key, 1);							/* Press new_key */
					input_report_key(shmhl_key, linux_key, 0);							/* Release new_key */
					input_sync(shmhl_key);
				}
			}
			/* Release */
			else {
				SHMHL_DEBUG_LOG_L("shmhl_input_rcp_key() Release[0x%x]\n", shmhl_rcp_work.last_linux_key);
				input_report_key(shmhl_key, shmhl_rcp_work.last_linux_key, 0);			/* Release last_key */
				input_sync(shmhl_key);
				shmhl_rcp_work.last_linux_key = KEY_RESERVED;
			}
			break;
		default:
			SHMHL_DEBUG_LOG_H("shmhl_input_rcp_key() state err\n");
			break;
	}
	mutex_unlock(&shmhl_rcp_lock);
}

void shmhl_rcp_input_register_device(void)
{
	int i = 0;
	int ret = 0;

	SHMHL_DEBUG_LOG_L("%s: Start\n", __func__);

	mutex_lock(&shmhl_rcp_lock);

	if( NULL != shmhl_key ) {
		SHMHL_DEBUG_LOG_L("Already input_allocate_device error\n");
		mutex_unlock(&shmhl_rcp_lock);
		return;
	}

	shmhl_key = input_allocate_device();

	if (!shmhl_key) {
		SHMHL_DEBUG_LOG_H("%s line:%d \n", __func__, __LINE__);
		mutex_unlock(&shmhl_rcp_lock);
		return;
	}

	shmhl_key->name = SH_MHL_KEY_DEVICE_NAME;
	shmhl_key->id.vendor	= 0x0001;
	shmhl_key->id.product	= 1;
	shmhl_key->id.version	= 1;


	for(i=0; i<(SH_MHL_RCP_KEYCODE_MAX+1); i++) {
		if(KEY_RESERVED != shmhl_rcp_key_comvert_tbl[i].linux_key) {
			input_set_capability(shmhl_key, EV_KEY, shmhl_rcp_key_comvert_tbl[i].linux_key);
		}
	}

	ret = input_register_device(shmhl_key);
	if (ret) {
		SHMHL_DEBUG_LOG_H("%s line:%d \n", __func__, __LINE__);
		input_free_device(shmhl_key);
		shmhl_key = NULL;
		mutex_unlock(&shmhl_rcp_lock);
		return;
	}

	mutex_unlock(&shmhl_rcp_lock);

	SHMHL_DEBUG_LOG_L("%s: End\n", __func__);
}

void shmhl_rcp_input_unregister_device(void)
{
	SHMHL_DEBUG_LOG_L("%s: Start\n", __func__);

	mutex_lock(&shmhl_rcp_lock);

	if( NULL == shmhl_key ) {
		SHMHL_DEBUG_LOG_L("Already input_unregister_device error\n");
		mutex_unlock(&shmhl_rcp_lock);
		return;
	}

	if( SHMHL_RCP_STATE_HOLD ==  shmhl_rcp_get_state()) {
		SHMHL_DEBUG_LOG_L("shmhl_input_rcp_key() STATE_HOLD now\n");
		shmhl_rcp_stop_delayed_work();											/* Stop Timer */

		SHMHL_DEBUG_LOG_L("shmhl_input_rcp_key() Release[0x%x]\n", shmhl_rcp_work.last_linux_key);
		input_report_key(shmhl_key, shmhl_rcp_work.last_linux_key, 0);			/* Release last_key */
		input_sync(shmhl_key);
		shmhl_rcp_work.last_linux_key = KEY_RESERVED;
	}

	input_unregister_device(shmhl_key);

	shmhl_key = NULL;

	mutex_unlock(&shmhl_rcp_lock);

	SHMHL_DEBUG_LOG_L("%s: End\n", __func__);
	return;
}

void shmhl_get_shdiag_mhl_cmd(char* cmd)
{
	strncpy(cmd, shmhl_shdiag_mhl_cmd, sizeof(shmhl_shdiag_mhl_cmd));
}

static int shmhl_open(struct inode *inode, struct file *filp)
{
	return 0;
}

static int shmhl_close(struct inode *inode, struct file *filp)
{
	return 0;
}

static long shmhl_ioctl(struct file *filp, unsigned int cmd, unsigned long arg)
{
	int ret = -EFAULT;
	int sys_ret = -EFAULT;
	void __user *argp = (void __user*)arg;

	if( (SH_MHL_IOCTL_GET_DEV_ID == cmd) ||
		(SH_MHL_IOCTL_SET_SHDIAG_MHL_CMD == cmd) ||
		(SH_MHL_IOCTL_GET_CONNECT_STATE == cmd)) {
		if( NULL == argp ) {
			SHMHL_DEBUG_LOG_H("shmhl_ioctl() argp == NULL!!\n");
			return ret; 
		}
	}

	switch (cmd)
	{
		case SH_MHL_IOCTL_GET_DEV_ID:
			SHMHL_DEBUG_LOG_L("SH_MHL_IOCTL_GET_DEV_ID\n");

			sys_ret = copy_to_user(argp, shmhl_dev_id, sizeof(shmhl_dev_id));

			if( sys_ret < 0 ) {
				SHMHL_DEBUG_LOG_H("shmhl_ioctl() copy_to_user() fail!!\n");
				ret = -EFAULT;
			}
			else {
				ret = 0;
			}
			break;

		case SH_MHL_IOCTL_SET_SHDIAG_MHL_CMD:
			SHMHL_DEBUG_LOG_L("SH_MHL_IOCTL_SET_SHDIAG_MHL_CMD\n");
			sys_ret = copy_from_user(&shmhl_shdiag_mhl_cmd, argp, sizeof(shmhl_shdiag_mhl_cmd));
			
			if( sys_ret < 0 ) {
				SHMHL_DEBUG_LOG_H("shmhl_ioctl() argp == NULL!!\n");
				ret = -EFAULT;
			}
			else {
				ret = 0;
			}
			break;
			
		case SH_MHL_IOCTL_GET_CONNECT_STATE:
			SHMHL_DEBUG_LOG_L("SH_MHL_IOCTL_GET_CONNECT_STATE\n");
			
			sys_ret = copy_to_user(argp, &shmhl_connect_work.con_state, sizeof(shmhl_connect_work.con_state));
			
			if( sys_ret < 0 ) {
				SHMHL_DEBUG_LOG_H("shmhl_ioctl() copy_to_user() fail!!\n");
				ret = -EFAULT;
			}
			else {
				ret = 0;
			}
			break;

		default:
			SHMHL_DEBUG_LOG_H("shmhl_ioctl() default!!\n");
			ret = -EFAULT;
			break;
	}

	return ret;
}

static int shmhl_pm_gpio_out(int port_no, int outvalue)
{
	int ret = -EFAULT;
	int gpio_ret = 0;
	int pm_port_no = 0;
	struct pm_gpio *gpio_param = NULL;

	struct pm_gpio gpio25_param = {
		.direction      = PM_GPIO_DIR_OUT,
		.output_buffer  = PM_GPIO_OUT_BUF_CMOS,
		.output_value   = 1,
		.pull			= PM_GPIO_PULL_NO,
		.vin_sel		= PM_GPIO_VIN_S4,
		.out_strength	= PM_GPIO_STRENGTH_MED,
		.function		= PM_GPIO_FUNC_NORMAL,
		.inv_int_pol	= 0,
		.disable_pin	= 0,
	};

	struct pm_gpio gpio23_param = {
		.direction      = PM_GPIO_DIR_OUT,
		.output_buffer  = PM_GPIO_OUT_BUF_CMOS,
		.output_value   = 1,
		.pull			= PM_GPIO_PULL_NO,
		.vin_sel		= PM_GPIO_VIN_S4,
		.out_strength	= PM_GPIO_STRENGTH_MED,
		.function		= PM_GPIO_FUNC_NORMAL,
		.inv_int_pol	= 0,
		.disable_pin	= 0,
	};

	SHMHL_DEBUG_LOG_L("%s: Start [port_no=%d] [outvalue=%d]\n", __func__, port_no, outvalue);

	switch (port_no)
	{
		case SH_MHL_GPIO_REG12_EN:
			gpio_param = &gpio25_param;
			break;

		case SH_MHL_GPIO_RESET_N:
			gpio_param = &gpio23_param;
			break;

		default:
			break;
	}

	if (NULL == gpio_param) {
		SHMHL_DEBUG_LOG_H("%s: Err Param\n", __func__);
		return ret;
	}

	pm_port_no = PM8921_GPIO_PM_TO_SYS(port_no);

	gpio_ret = gpio_request(pm_port_no, "MHL_GPIO");

	if (gpio_ret) {
		SHMHL_DEBUG_LOG_H("%s: Failed to request gpio\n", __func__);
		gpio_free(pm_port_no);
		return ret;
	}

	gpio_ret = pm8xxx_gpio_config(pm_port_no, gpio_param);

	if (gpio_ret) {
		SHMHL_DEBUG_LOG_H("%s: Failed to config gpio\n", __func__);
		gpio_free(pm_port_no);
		return ret;
	}

	gpio_ret = gpio_direction_output(pm_port_no, outvalue);


	if (gpio_ret) {
		SHMHL_DEBUG_LOG_H("%s: Failed to output gpio\n", __func__);
		gpio_free(pm_port_no);
		return ret;
	}

	gpio_free(pm_port_no);

	SHMHL_DEBUG_LOG_L("%s: End\n", __func__);

	return 0;
}

static int shmhl_reg_enable(const char *id, int min_uVol, int max_uVol)
{
	int ret = 0;
	struct regulator *reg;
	struct device *dev = NULL;

	reg = regulator_get(dev, id);
	if (IS_ERR(reg)) {
		SHMHL_DEBUG_LOG_H("%s: Failed to get %s regulator\n", __func__, id);
		return -1;
	}

	ret = regulator_set_voltage(reg, min_uVol, max_uVol);
	if (ret) {
		SHMHL_DEBUG_LOG_H("%s: Failed to set %s regulator min_uV:%d, max_uV:%d\n", __func__, id, min_uVol, max_uVol);
		regulator_put(reg);
		return ret;
	}

	ret = regulator_is_enabled(reg);
	if (ret) {
		SHMHL_DEBUG_LOG_H("%s: %s regulator is already enabled\n", __func__, id);
		regulator_put(reg);
		return ret;
	}
	
	ret = regulator_enable(reg);
	if (ret) {
		SHMHL_DEBUG_LOG_H("%s: Failed to enable %s regulator\n", __func__, id);
	}

	regulator_put(reg);

	return ret;
}

static int shmhl_reg_disable(const char *id)
{
	int ret = 0;
	struct regulator *reg;
	struct device *dev = NULL;

	reg = regulator_get(dev, id);
	if (IS_ERR(reg)) {
		SHMHL_DEBUG_LOG_H("%s: Failed to get %s regulator\n", __func__, id);
		return -1;
	}

	ret = regulator_is_enabled(reg);
	if (!ret) {
		SHMHL_DEBUG_LOG_H("%s: %s regulator is already disabled\n", __func__, id);
		regulator_put(reg);
		return ret;
	}
	
	ret = regulator_disable(reg);
	if (ret) {
		SHMHL_DEBUG_LOG_H("%s: Failed to disable %s regulator\n", __func__, id);
	}

	regulator_put(reg);

	return ret;
}

static void shmhl_rcp_start_delayed_work(unsigned int linux_key)
{
	SHMHL_DEBUG_LOG_L("shmhl_rcp_start_delayed_work()\n");

	queue_delayed_work(shmhl_rcp_work.shmhl_rcp_wq, &shmhl_rcp_work.shmhl_rcp_dwork, msecs_to_jiffies(SH_MHL_RCP_HOLD_MS));
	shmhl_rcp_work.last_linux_key = linux_key;
	shmhl_rcp_set_state(SHMHL_RCP_STATE_HOLD);
}

static void shmhl_rcp_stop_delayed_work(void)
{
	SHMHL_DEBUG_LOG_L("shmhl_rcp_stop_delayed_work()\n");

	cancel_delayed_work(&shmhl_rcp_work.shmhl_rcp_dwork);
	shmhl_rcp_set_state(SHMHL_RCP_STATE_NONE);
}

static void shmhl_rcp_set_state(shmhl_rcp_state_t new_state)
{
	SHMHL_DEBUG_LOG_L("shmhl_rcp_set_state() [%d]-->[%d]\n", shmhl_rcp_work.rcp_state, new_state);
	shmhl_rcp_work.rcp_state = new_state;
}

static shmhl_rcp_state_t shmhl_rcp_get_state(void)
{
	return(shmhl_rcp_work.rcp_state);
}

static void shmhl_rcp_delay_handler(struct work_struct *poWork)
{
	mutex_lock(&shmhl_rcp_lock);

	if( NULL == shmhl_key ) {
		SHMHL_DEBUG_LOG_L("Already input_unregister_device error\n");
		mutex_unlock(&shmhl_rcp_lock);
		return;
	}

	if(shmhl_rcp_get_state() == SHMHL_RCP_STATE_HOLD) {
		SHMHL_DEBUG_LOG_L("shmhl_rcp_delay_handler() Release[%d(0x%x)]\n", shmhl_rcp_work.last_linux_key, shmhl_rcp_work.last_linux_key);

		shmhl_rcp_set_state(SHMHL_RCP_STATE_NONE);
		/* Release */
		input_report_key(shmhl_key, shmhl_rcp_work.last_linux_key, 0);
		input_sync(shmhl_key);
		shmhl_rcp_work.last_linux_key = KEY_RESERVED;
	}
	else {
		SHMHL_DEBUG_LOG_L("shmhl_rcp_delay_handler() Irregal State!\n");
	}

	mutex_unlock(&shmhl_rcp_lock);
}

static shmhl_boot_mode shmhl_get_boot_mode_from_smem(void)
{
	sharp_smem_common_type *p_smem_addr = NULL;
	int smem_boot_mode;
	int smem_softupdateflg = 0;
	shmhl_boot_mode boot_mode = SHMHL_BOOT_MODE_NORMAL;

	/* smem */
	p_smem_addr = sh_smem_get_common_address();
	if ( NULL == p_smem_addr ) {
		pr_err("%s: sh_smem_get_common_address failed\n", __func__);
		return -ENOMEM;
	}

	smem_boot_mode = p_smem_addr->sh_boot_mode;
	smem_softupdateflg = p_smem_addr->shusb_softupdate_mode_flag;

	/* judge boot mode */
	switch ( smem_boot_mode ) {
		case 0x40:
			if ( !smem_softupdateflg ) {
				boot_mode = SHMHL_BOOT_MODE_TESTMODE;
			} else {
				boot_mode = SHMHL_BOOT_MODE_SOFTWARE_UPDATE;
			}
			break;
		case 0x41:
		case 0x42:
			boot_mode = SHMHL_BOOT_MODE_TESTMODE;
			break;
		default :
			boot_mode = SHMHL_BOOT_MODE_NORMAL;
			break;
	}
	return boot_mode;
}

static ssize_t ShowDebugFlg(struct device *dev, struct device_attribute *attr, char *buf)
{
	size_t size = -EINVAL;

	SHMHL_DEBUG_LOG_L("ShowDebugFlg() Start\n");

	if(NULL == buf) {
		SHMHL_DEBUG_LOG_H("ShowDebugFlg() Param Err!!\n");
		return -EINVAL;
	}

	if(HalAcquireIsrLock() == HAL_RET_SUCCESS) {
		size = snprintf(buf, PAGE_SIZE, "%s\n", shmhl_debug_flg);

		SHMHL_DEBUG_LOG_L("ShowDebugFlg() size=[%d]\n", size);
		SHMHL_DEBUG_LOG_L("ShowDebugFlg() buf=[0x%X][0x%X][0x%X][0x%X][0x%X][0x%X][0x%X][0x%X]\n",
							buf[0], buf[1], buf[2], buf[3], buf[4], buf[5], buf[6], buf[7]);
		HalReleaseIsrLock();
	}

	return size;
}

static ssize_t StoreDebugFlg(struct device *dev, struct device_attribute *attr, const char *buf, size_t count)
{
	if(NULL == buf) {
		SHMHL_DEBUG_LOG_H("StoreDebugFlg() Param Err!!\n");
		return -EINVAL;
	}

	if(HalAcquireIsrLock() == HAL_RET_SUCCESS) {
		memset(shmhl_debug_flg, 0x00, sizeof(shmhl_debug_flg));

		if('1' == buf[SHMHL_DEBUG_LOG_L]) {
			SHMHL_DEBUG_LOG_H("SHMHL DEBUG LOG(L) ON\n");
			shmhl_debug_flg[SHMHL_DEBUG_LOG_L] = '1';
		}
		else if('0' == buf[SHMHL_DEBUG_LOG_L]) {
			SHMHL_DEBUG_LOG_H("SHMHL DEBUG LOG(L) OFF\n");
			shmhl_debug_flg[SHMHL_DEBUG_LOG_L] = '0';
		}

		if('1' == buf[SHMHL_DEBUG_LOG_TRACE]) {
			SHMHL_DEBUG_LOG_H("SII_OSAL_DEBUG_TRACE ON\n");
			shmhl_debug_flg[SHMHL_DEBUG_LOG_TRACE] = '1';
			SiiOsDebugChannelEnable(SII_OSAL_DEBUG_TRACE);
		}
		else if('0' == buf[SHMHL_DEBUG_LOG_TRACE]) {
			SHMHL_DEBUG_LOG_H("SII_OSAL_DEBUG_TRACE OFF\n");
			shmhl_debug_flg[SHMHL_DEBUG_LOG_TRACE] = '0';
			SiiOsDebugChannelDisable(SII_OSAL_DEBUG_TRACE);
		}

		if('1' == buf[SHMHL_DEBUG_LOG_TX]) {
			SHMHL_DEBUG_LOG_H("SII_OSAL_DEBUG_TX ON\n");
			shmhl_debug_flg[SHMHL_DEBUG_LOG_TX] = '1';
			SiiOsDebugChannelEnable(SII_OSAL_DEBUG_TX);
		}
		else if('0' == buf[SHMHL_DEBUG_LOG_TX]) {
			SHMHL_DEBUG_LOG_H("SII_OSAL_DEBUG_TX OFF\n");
			shmhl_debug_flg[SHMHL_DEBUG_LOG_TX] = '0';
			SiiOsDebugChannelDisable(SII_OSAL_DEBUG_TX);
		}

		if('1' == buf[SHMHL_DEBUG_LOG_CBUS]) {
			SHMHL_DEBUG_LOG_H("SII_OSAL_DEBUG_CBUS ON\n");
			shmhl_debug_flg[SHMHL_DEBUG_LOG_CBUS] = '1';
			SiiOsDebugChannelEnable(SII_OSAL_DEBUG_CBUS);
		}
		else if('0' == buf[SHMHL_DEBUG_LOG_CBUS]) {
			SHMHL_DEBUG_LOG_H("SII_OSAL_DEBUG_CBUS OFF\n");
			shmhl_debug_flg[SHMHL_DEBUG_LOG_CBUS] = '0';
			SiiOsDebugChannelDisable(SII_OSAL_DEBUG_CBUS);
		}

		if('1' == buf[SHMHL_DEBUG_LOG_DUMP]) {
			SHMHL_DEBUG_LOG_H("DUMP ON\n");
			shmhl_debug_flg[SHMHL_DEBUG_LOG_DUMP] = '1';
		}
		else if('0' == buf[SHMHL_DEBUG_LOG_DUMP]) {
			SHMHL_DEBUG_LOG_H("DUMP OFF\n");
			shmhl_debug_flg[SHMHL_DEBUG_LOG_DUMP] = '0';
		}

		if('1' == buf[SHMHL_DEBUG_LOG_EDID]) {
			SHMHL_DEBUG_LOG_H("EDID DUMP ON\n");
			shmhl_debug_flg[SHMHL_DEBUG_LOG_EDID] = '1';

		}
		else if('0' == buf[SHMHL_DEBUG_LOG_EDID]) {
			SHMHL_DEBUG_LOG_H("EDID DUMP OFF\n");
			shmhl_debug_flg[SHMHL_DEBUG_LOG_EDID] = '0';

			shmhl_free_edid_out_buf();
		}

		if('1' == buf[SHMHL_DEBUG_LOG_HDMI]) {
			SHMHL_DEBUG_LOG_H("HDMI LOG(L) ON\n");
			shmhl_debug_flg[SHMHL_DEBUG_LOG_HDMI] = '1';
		}
		else if('0' == buf[SHMHL_DEBUG_LOG_HDMI]) {
			SHMHL_DEBUG_LOG_H("HDMI LOG(L) OFF\n");
			shmhl_debug_flg[SHMHL_DEBUG_LOG_HDMI] = '0';
		}

		HalReleaseIsrLock();
	}

	return count;
}

static ssize_t ShowRegDump(struct device *dev, struct device_attribute *attr, char *buf)
{
	SHMHL_DEBUG_LOG_L("ShowRegDump() Start PAGE_SIZE[%u]\n", (unsigned int)PAGE_SIZE);

	if(NULL == buf) {
		SHMHL_DEBUG_LOG_H("ShowRegDump() Param Err!!\n");
		return -EINVAL;
	}

	if(HalAcquireIsrLock() == HAL_RET_SUCCESS) {
		shmhl_SiI8334_Reg_Dump_All_Offset(buf, (size_t)(PAGE_SIZE-1));

		HalReleaseIsrLock();
	}

	SHMHL_DEBUG_LOG_L("ShowRegDump() End\n");

	return (unsigned int)(PAGE_SIZE-1);
}

static ssize_t ShowRegReadResult(struct device *dev, struct device_attribute *attr, char *buf)
{
	size_t size = -EINVAL;

	SHMHL_DEBUG_LOG_L("ShowRegReadResult() Start [read_result=%s]\n", shmhl_reg_read_result);

	if(NULL == buf) {
		SHMHL_DEBUG_LOG_H("ShowRegReadResult() Param Err!!\n");
		return -EINVAL;
	}

	if (HalAcquireIsrLock() == HAL_RET_SUCCESS) {
		size = snprintf(buf, PAGE_SIZE, "%s\n", shmhl_reg_read_result);

		SHMHL_DEBUG_LOG_L("ShowRegReadResult() size=[%d]\n", size);
		SHMHL_DEBUG_LOG_L("ShowRegReadResult() buf=[0x%X][0x%X][0x%X]\n", buf[0], buf[1], buf[2]);
		SHMHL_DEBUG_LOG_L("ShowRegReadResult() buf=[0x%X] End\n", buf[3]);

		HalReleaseIsrLock();
	}

	return size;
}

static ssize_t ProcRegRead(struct device *dev, struct device_attribute *attr, const char *buf, size_t count)
{
	char work_str[3];
	unsigned long slv_addr = 0;
	unsigned long offset = 0;
	SiiResultCodes_t i2cret = SII_ERR_FAIL;
	uint8_t read_buf = 0;
	int rc = 0;

	SHMHL_DEBUG_LOG_L("ProcRegRead() Start count[%d]\n", count);

	if(NULL == buf) {
		SHMHL_DEBUG_LOG_H("ProcRegRead() Param Err!!\n");
		return -EINVAL;
	}

	if(HalAcquireIsrLock() == HAL_RET_SUCCESS) {
		memset(shmhl_reg_read_result, 0, sizeof(shmhl_reg_read_result));
		memset(work_str, 0, sizeof(work_str));

		work_str[0] = buf[0];
		work_str[1] = buf[1];
		rc = strict_strtoul(&work_str[0], 16, &slv_addr);
		SHMHL_DEBUG_LOG_L("ProcRegRead() slv[0x%02X]\n", (uint8_t)slv_addr);

		if(0 != rc) {
			SHMHL_DEBUG_LOG_H("ProcRegRead() Param Err slv[0x%X][0x%X]!!\n", buf[0], buf[1]);
			HalReleaseIsrLock();
			return -EINVAL;
		}

		work_str[0] = buf[2];
		work_str[1] = buf[3];
		rc = strict_strtoul(&work_str[0], 16, &offset);
		SHMHL_DEBUG_LOG_L("ProcRegRead() offset[0x%02X]\n", (uint8_t)offset);

		if(0 != rc) {
			SHMHL_DEBUG_LOG_H("ProcRegRead() Param Err offset[0x%X][0x%X]!!\n", buf[2], buf[3]);
			HalReleaseIsrLock();
			return -EINVAL;
		}

		i2cret = CraReadBlockI2c(DEV_I2C_0, (uint8_t)slv_addr, (uint8_t)offset, &read_buf, 1);

		if(SII_SUCCESS == i2cret) {
			snprintf(shmhl_reg_read_result, sizeof(shmhl_reg_read_result), "%02X", read_buf);
		}
		else {
			SHMHL_DEBUG_LOG_H("ProcRegRead() I2C Err!!\n");
			HalReleaseIsrLock();
			return -EINVAL;
		}

		HalReleaseIsrLock();
	}

	SHMHL_DEBUG_LOG_L("ProcRegRead() End [%s]\n", shmhl_reg_read_result);

	return count;
}

static ssize_t ProcRegWrite(struct device *dev, struct device_attribute *attr, const char *buf, size_t count)
{
	char work_str[3];
	unsigned long slv_addr = 0;
	unsigned long offset = 0;
	unsigned long value = 0;
	SiiResultCodes_t i2cret = SII_ERR_FAIL;
	int rc = 0;

	SHMHL_DEBUG_LOG_L("ProcRegWrite() Start\n");

	if(NULL == buf) {
		SHMHL_DEBUG_LOG_H("ProcRegWrite() Param Err!!\n");
		return -EINVAL;
	}

	if(HalAcquireIsrLock() == HAL_RET_SUCCESS) {

		memset(work_str, 0, sizeof(work_str));

		work_str[0] = buf[0];
		work_str[1] = buf[1];
		rc = strict_strtoul(&work_str[0], 16, &slv_addr);
		SHMHL_DEBUG_LOG_L("ProcRegWrite() slv[0x%02X]\n", (uint8_t)slv_addr);

		if(0 != rc) {
			SHMHL_DEBUG_LOG_H("ProcRegWrite() Param Err slv[0x%X][0x%X]!!\n", buf[0], buf[1]);
			HalReleaseIsrLock();
			return -EINVAL;
		}

		work_str[0] = buf[2];
		work_str[1] = buf[3];
		rc = strict_strtoul(&work_str[0], 16, &offset);
		SHMHL_DEBUG_LOG_L("ProcRegWrite() offset[0x%02X]\n", (uint8_t)offset);

		if(0 != rc) {
			SHMHL_DEBUG_LOG_H("ProcRegWrite() Param Err offset[0x%X][0x%X]!!\n", buf[2], buf[3]);
			HalReleaseIsrLock();
			return -EINVAL;
		}

		work_str[0] = buf[4];
		work_str[1] = buf[5];
		rc = strict_strtoul(&work_str[0], 16, &value);
		SHMHL_DEBUG_LOG_L("ProcRegWrite() value[0x%02X]\n", (uint8_t)value);

		if(0 != rc) {
			SHMHL_DEBUG_LOG_H("ProcRegWrite() Param Err value[0x%X][0x%X]!!\n", buf[4], buf[5]);
			HalReleaseIsrLock();
			return -EINVAL;
		}

		i2cret = CraWriteBlockI2c(DEV_I2C_0, (uint8_t)slv_addr, (uint8_t)offset, (uint8_t*)&value, 1);

		if(SII_SUCCESS == i2cret) {
			SHMHL_DEBUG_LOG_L("ProcRegWrite() I2C Success\n");
		}
		else {
			SHMHL_DEBUG_LOG_H("ProcRegWrite() I2C Err!!\n");
			HalReleaseIsrLock();
			return -EINVAL;
		}

		HalReleaseIsrLock();
	}

	SHMHL_DEBUG_LOG_L("ProcRegWrite() End\n");

	return count;
}

static ssize_t ShowRcpDevRegist(struct device *dev, struct device_attribute *attr, char *buf)
{
	SHMHL_DEBUG_LOG_L("ShowRcpDevRegist() Start [read_result=%s]\n", shmhl_reg_read_result);

	mutex_lock(&shmhl_rcp_lock);

	if(NULL == buf) {
		SHMHL_DEBUG_LOG_H("ShowRcpDevRegist() Param Err!!\n");
		mutex_unlock(&shmhl_rcp_lock);
		return -EINVAL;
	}

	if(NULL == shmhl_key) {
		SHMHL_DEBUG_LOG_H("ShowRcpDevRegist() Not Resited\n");
		strncpy(buf, "0\n", 3);
	}
	else {
		SHMHL_DEBUG_LOG_H("ShowRcpDevRegist() Already Resited\n");
		strncpy(buf, "1\n", 3);
	}

	mutex_unlock(&shmhl_rcp_lock);

	SHMHL_DEBUG_LOG_L("ShowRcpDevRegist() End buf=[0x%X][0x%X][0x%X]\n", buf[0], buf[1], buf[2]);

	return 2;
}

static ssize_t ProcRcpDevRegist(struct device *dev, struct device_attribute *attr, const char *buf, size_t count)
{
	SHMHL_DEBUG_LOG_L("ProcRcpDevRegist() Start\n");

	if(NULL == buf) {
		SHMHL_DEBUG_LOG_H("ProcRcpDevRegist() Param Err!!\n");
		return -EINVAL;
	}

	/* Regist */
	if('1' == buf[0]) {
		SHMHL_DEBUG_LOG_L("ProcRcpDevRegist() Regist\n");
		shmhl_rcp_input_register_device();
	}
	/* UnRegist */
	else if('0' == buf[0]) {
		SHMHL_DEBUG_LOG_L("ProcRcpDevRegist() UnRegist\n");
		shmhl_rcp_input_unregister_device();
	}
	else {
		SHMHL_DEBUG_LOG_H("ProcRcpDevRegist() Param Err!! [0x%X]\n", buf[0]);
		return -EINVAL;
	}

	SHMHL_DEBUG_LOG_L("ProcRcpDevRegist() End\n");

	return count;
}

static ssize_t ProcRcpInputKey(struct device *dev, struct device_attribute *attr, const char *buf, size_t count)
{
	char work_str[3];
	unsigned long rcp_data = 0;
	int rc = 0;

	SHMHL_DEBUG_LOG_L("ProcRcpInputKey() Start\n");

	if(NULL == buf) {
		SHMHL_DEBUG_LOG_H("ProcRcpInputKey() Param Err!!\n");
		return -EINVAL;
	}

	if(NULL == shmhl_key) {
		SHMHL_DEBUG_LOG_H("ProcRcpInputKey() Not Registed Err!!\n");
		return -EINVAL;
	}

	memset(work_str, 0, sizeof(work_str));

	work_str[0] = buf[0];
	work_str[1] = buf[1];
	rc = strict_strtoul(&work_str[0], 16, &rcp_data);

	if(0 != rc) {
		SHMHL_DEBUG_LOG_H("ProcRcpInputKey() Param Err buf[0x%X][0x%X]!!\n", buf[0], buf[1]);
		return -EINVAL;
	}

	SHMHL_DEBUG_LOG_L("ProcRcpInputKey() rcp_data[0x%02X]\n", (uint8_t)rcp_data);

	shmhl_input_rcp_key((uint8_t)rcp_data);

	SHMHL_DEBUG_LOG_L("ProcRcpInputKey() End\n");

	return count;
}

bool shmhl_get_debug_flg(shmhl_debug_log_type_t log_type)
{
	bool ret_val = FALSE;

	if('1' == shmhl_debug_flg[log_type]) {
		ret_val = TRUE;
	}
	return ret_val;
}

static ssize_t ShowEdidDump(struct device *dev, struct device_attribute *attr, char *buf)
{
	int copy_size = 0;
	int block = 0;

	SHMHL_DEBUG_LOG_L("ShowEdidDump() Start PAGE_SIZE[%u]\n", (unsigned int)PAGE_SIZE);

	if(NULL == buf) {
		SHMHL_DEBUG_LOG_H("ShowEdidDump() Param Err!!\n");
		return -EINVAL;
	}

	if(HalAcquireIsrLock() == HAL_RET_SUCCESS) {
		if(shmhl_get_debug_flg(SHMHL_DEBUG_LOG_EDID)) {
			for(block=0; block<SH_MHL_EDID_BLOCK_MAX; block++) {
				if(NULL != shmhl_edid_buf[block]) {
					SHMHL_DEBUG_LOG_L("ShowEdidDump() Edid Copy block[%d]\n", block);
					copy_size = snprintf(buf, (size_t)(SH_MHL_EDID_OUT_BUF_SIZE), "%s", shmhl_edid_buf[block]);
					buf = buf + copy_size;
				}
			}
		}
		else {
			SHMHL_DEBUG_LOG_L("ShowEdidDump() Debug Flg Err!!\n");
		}

		HalReleaseIsrLock();
	}

	SHMHL_DEBUG_LOG_L("ShowEdidDump() End\n");

	return (unsigned int)(PAGE_SIZE-1);
}

char* shmhl_get_edid_out_buf(int block)
{
	SHMHL_DEBUG_LOG_L("shmhl_get_edid_out_buf() Start block[%d]\n", block);

	if(SH_MHL_EDID_BLOCK_MAX <= block) {
		SHMHL_DEBUG_LOG_L("shmhl_get_edid_out_buf() Param Error!! \n");
		return NULL;
	}

	if(shmhl_get_debug_flg(SHMHL_DEBUG_LOG_EDID)) {
		if(NULL == shmhl_edid_buf[block]) {
			SHMHL_DEBUG_LOG_L("shmhl_get_edid_out_buf() Alloc Memory\n");
			shmhl_edid_buf[block] = (char*)kzalloc(SH_MHL_EDID_OUT_BUF_SIZE, GFP_KERNEL);
		}
		else {
			SHMHL_DEBUG_LOG_L("shmhl_get_edid_out_buf() Already Allocated\n");
		}
	}

	SHMHL_DEBUG_LOG_L("shmhl_get_edid_out_buf() End buf[%X]\n", (unsigned int)shmhl_edid_buf[block]);

	return shmhl_edid_buf[block];
}

static void shmhl_free_edid_out_buf(void)
{
	int block = 0;

	SHMHL_DEBUG_LOG_L("shmhl_free_edid_out_buf() Start\n");

	for(block=0; block<SH_MHL_EDID_BLOCK_MAX; block++) {
		if(NULL != shmhl_edid_buf[block]) {
			kfree(shmhl_edid_buf[block]);
			shmhl_edid_buf[block] = NULL;
			SHMHL_DEBUG_LOG_L("shmhl_free_edid_out_buf() free[block%d]\n", block);
		}
	}

	SHMHL_DEBUG_LOG_L("shmhl_free_edid_out_buf() End\n");

}

#define D_SHMHL_DUMP_SLAVE_NUM 4
#define D_SHMHL_DUMP_OFFSET_NUM 256
#define D_SHMHL_DUMP_TATE_MAX 0x100
#define D_SHMHL_DUMP_YOKO_MAX 16
/*****************************************************************************/
/* for RegisterDUMP */
/*****************************************************************************/
void shmhl_SiI8334_Reg_Dump_All_Offset(char *buf, size_t size)
{
	int slv_addr[D_SHMHL_DUMP_SLAVE_NUM] = {
		0x72, 0x7A, 0x9A, 0xC8
	};

	SiiResultCodes_t i2cret = 0;
	int print_size = 0;
	int i = 0;
	int tate = 0;
	int yoko = 0;
	uint8_t read_buf = 0;
	uint8_t dump[128];

	/* Output Image */
	/*
	   | 00 01 02 03 04 05 06 07 08 09 0A 0B 0C 0D 0E 0F
	------------------------------------------------------
	00 | FF FF FF FF FF FF FF FF 
	10 |
	20 |
	30 |
	40 |
	50 |
	60 |
	70 |
	80 |
	90 |
	A0 |
	B0 |
	C0 |
	D0 |
	E0 |
	F0 |
	*/

	SHMHL_DEBUG_LOG_L("shmhl_SiI8334_Reg_Dump_All_Offset() Start buf[0x%X] size[%u]\n", (unsigned int)buf, size);

	for( i=0; i<D_SHMHL_DUMP_SLAVE_NUM; i++ ) {
		SHMHL_DUMP_LOG(buf, size, print_size, "DUMP------------------\n");
		SHMHL_DUMP_BUF_FWD(buf, print_size);

		SHMHL_DUMP_LOG(buf, size, print_size, "DUMP   | SLAVE=[0x%02X]\n", slv_addr[i]);
		SHMHL_DUMP_BUF_FWD(buf, print_size);

		SHMHL_DUMP_LOG(buf, size, print_size, "DUMP   | 00 01 02 03 04 05 06 07 08 09 0A 0B 0C 0D 0E 0F\n");
		SHMHL_DUMP_BUF_FWD(buf, print_size);

		for(tate=0; tate<D_SHMHL_DUMP_TATE_MAX; tate=tate+0x10) {
			memset(dump, 0x00, sizeof(dump));

			for(yoko=0; yoko<D_SHMHL_DUMP_YOKO_MAX; yoko++) {
				read_buf = 0;

				i2cret = CraReadBlockI2c(DEV_I2C_0, slv_addr[i], (tate+yoko), &read_buf, 1);

				dump[yoko] = read_buf;
			}
			SHMHL_DUMP_LOG(buf, size, print_size, "DUMP%02X | %02X %02X %02X %02X %02X %02X %02X %02X %02X %02X %02X %02X %02X %02X %02X %02X\n"
										,tate
										,dump[0],dump[1],dump[2],dump[3],dump[4],dump[5],dump[6],dump[7]
										,dump[8],dump[9],dump[10],dump[11],dump[12],dump[13],dump[14],dump[15]);
			SHMHL_DUMP_BUF_FWD(buf, print_size);
		}
	}

	SHMHL_DEBUG_LOG_L("shmhl_SiI8334_Reg_Dump_All_Offset() End\n");
}

static ssize_t GetRegEyePattern(struct device *dev, struct device_attribute *attr, char *buf)
{
	size_t size = -EINVAL;

	SHMHL_DEBUG_LOG_L("%s Start\n", __func__);

	if(buf == NULL) {
		SHMHL_DEBUG_LOG_H("%s Param Err!!\n", __func__);
		return -EINVAL;
	}

	if(HalAcquireIsrLock() == HAL_RET_SUCCESS) {
		size = snprintf(buf, 3, "%02X\n", shmhl_reg_eye_pattern);

		SHMHL_DEBUG_LOG_L("%s size=%d\n", __func__, size);
		SHMHL_DEBUG_LOG_L("%s buf=0x%s\n", __func__, buf);
		HalReleaseIsrLock();
	}

	SHMHL_DEBUG_LOG_L("%s End\n", __func__);
	return size;
}

static uint8_t shmhl_hexchar_to_uint8(char hexchar)
{
	SHMHL_DEBUG_LOG_L("%s Start\n", __func__);
	
	switch(hexchar) {
	case '0'...'9':
		return hexchar - '0';
	case 'A'...'F':
		return 10 + (hexchar - 'A');
	case 'a'...'f':
		return 10 + (hexchar - 'a');
	default:
		return 0;
	}
	
	SHMHL_DEBUG_LOG_L("%s End\n", __func__);
}

static ssize_t SetRegEyePattern(struct device *dev, struct device_attribute *attr, const char *buf, size_t count)
{
	SHMHL_DEBUG_LOG_L("%s Start\n", __func__);

	if((buf == NULL) || (count != 3)) {
		SHMHL_DEBUG_LOG_H("%s Param Err!! buf=%x, count=%d\n", __func__, (uint)buf, count);
		return -EINVAL;
	}
	
	if(HalAcquireIsrLock() == HAL_RET_SUCCESS) {
		shmhl_reg_eye_pattern = shmhl_hexchar_to_uint8(buf[0]) << 4;
		shmhl_reg_eye_pattern += shmhl_hexchar_to_uint8(buf[1]);
		SHMHL_DEBUG_LOG_L("%s eye pattern:%02X\n", __func__, shmhl_reg_eye_pattern);
		HalReleaseIsrLock();
	}
	
	return count;


	SHMHL_DEBUG_LOG_L("%s End\n", __func__);
}

static ssize_t GetConnectState(struct device *dev, struct device_attribute *attr, char *buf)
{
	size_t size = -EINVAL;
	int connected = 0;

	SHMHL_DEBUG_LOG_L("%s Start\n", __func__);

	if(buf == NULL) {
		SHMHL_DEBUG_LOG_H("%s Param Err!!\n", __func__);
		return -EINVAL;
	}

	if(HalAcquireIsrLock() == HAL_RET_SUCCESS) {
		SHMHL_DEBUG_LOG_L("%s shmhl_connect_work.con_state=%d\n", __func__, shmhl_connect_work.con_state);
		switch (shmhl_connect_work.con_state) {
		case SHMHL_CON_STATE_MHL_EST:
		case SHMHL_CON_STATE_HAVE_DEV_CAT:
			connected = 1;
			break;
		default:
			connected = 0;
			break;
		}
		size = snprintf(buf, 2, "%1d\n", connected);
		SHMHL_DEBUG_LOG_L("%s size=%d\n", __func__, size);
		SHMHL_DEBUG_LOG_L("%s buf=%s\n", __func__, buf);
		HalReleaseIsrLock();
	}

	SHMHL_DEBUG_LOG_L("%s End\n", __func__);
	return size;
}

struct device_attribute shmhl_driver_attribs[] = {
		__ATTR(debug_flg, 0600, ShowDebugFlg, StoreDebugFlg),
		__ATTR(reg_dump, 0400, ShowRegDump, NULL),
		__ATTR(reg_read, 0600, ShowRegReadResult, ProcRegRead),
		__ATTR(reg_write, 0200, NULL, ProcRegWrite),
		__ATTR(rcp_dev_regist, 0600, ShowRcpDevRegist, ProcRcpDevRegist),
		__ATTR(rcp_input_key, 0200, NULL, ProcRcpInputKey),
		__ATTR(edid_dump, 0400, ShowEdidDump, NULL),
		__ATTR(reg_eye_pattern, 0600, GetRegEyePattern, SetRegEyePattern),
		__ATTR(state, 0444, GetConnectState, NULL),
		__ATTR_NULL
};

static int __init shmhl_init(void)
{
	int		ret = 0;
	dev_t	dev;

	shmhl_debug_flg[SHMHL_DEBUG_LOG_L]     = '1';
	shmhl_debug_flg[SHMHL_DEBUG_LOG_TRACE] = '0';
	shmhl_debug_flg[SHMHL_DEBUG_LOG_TX]    = '0';
	shmhl_debug_flg[SHMHL_DEBUG_LOG_CBUS]  = '0';
	shmhl_debug_flg[SHMHL_DEBUG_LOG_DUMP]  = '0';
	shmhl_debug_flg[SHMHL_DEBUG_LOG_EDID]  = '0';
	shmhl_debug_flg[SHMHL_DEBUG_LOG_HDMI]  = '1';
	shmhl_debug_flg[SHMHL_DEBUG_LOG_MAX]   = '0';

	SHMHL_DEBUG_LOG_L("shmhl_init() Start\n");

	dev = MKDEV(shmhl_major, 0);

	ret = alloc_chrdev_region(&dev, SH_MHL_MINORNUM_BASE, SH_MHL_DEVICE_COUNT, SH_MHL_DEVICE_NAME);
	
	if (ret < 0) {
		SHMHL_DEBUG_LOG_H("%s line:%d \n", __func__, __LINE__);
	}

	shmhl_major = MAJOR(dev);

	cdev_init(&shmhl_cdev, &shmhl_fops);
	shmhl_cdev.owner = THIS_MODULE;
	shmhl_cdev.ops = &shmhl_fops;
	
	ret = cdev_add(&shmhl_cdev, dev, SH_MHL_DEVICE_COUNT);

	if (ret < 0) {
		SHMHL_DEBUG_LOG_H("%s line:%d \n", __func__, __LINE__);
	}

	shmhl_kdrv_class = class_create( THIS_MODULE, SH_MHL_CLASS_NAME );
	if (IS_ERR(shmhl_kdrv_class)) {
		SHMHL_DEBUG_LOG_H("%s line:%d \n", __func__, __LINE__);
		return 0;
	}

	shmhl_kdrv_class->dev_attrs = shmhl_driver_attribs;

	shmhl_kdrv_dev = dev;
	device_create(shmhl_kdrv_class, NULL, shmhl_kdrv_dev, NULL, SH_MHL_DEVICE_NAME);

	memset(shmhl_dev_id, 0, sizeof(shmhl_dev_id));
	memset(&shmhl_connect_work, 0, sizeof(shmhl_connect_work));
	shmhl_prepare_wait_plug_state();
	shmhl_set_con_state(SHMHL_CON_STATE_WAIT_PLUG);

	memset(&shmhl_rcp_work, 0, sizeof(shmhl_rcp_work));

	shmhl_connect_work.shmhl_connect_wq = alloc_ordered_workqueue(SH_MHL_DEVICE_NAME, 0);
	shmhl_rcp_work.shmhl_rcp_wq = alloc_ordered_workqueue(SH_MHL_KEY_DEVICE_NAME, 0);

	INIT_WORK(&shmhl_connect_work.shmhl_init_swork, shmhl_init_comp_handler);
	INIT_WORK(&shmhl_connect_work.shmhl_id_vbus_state_swork, shmhl_id_hi_vbus_low_handler);
	INIT_DELAYED_WORK(&shmhl_connect_work.shmhl_rgnd_dwork, shmhl_rgnd_timer_handler);
	INIT_DELAYED_WORK(&shmhl_connect_work.shmhl_disc_retry_dwork, shmhl_disc_retry_handler);
	INIT_DELAYED_WORK(&shmhl_rcp_work.shmhl_rcp_dwork, shmhl_rcp_delay_handler);

	mutex_init(&shmhl_connect_lock);
	mutex_init(&shmhl_rcp_lock);

	if(SHMHL_BOOT_MODE_TESTMODE == shmhl_get_boot_mode_from_smem()) {
		strncpy(shmhl_shdiag_mhl_cmd, SH_MHL_SHDIAG_CMD_TESTMODE_DEFAULT, sizeof(shmhl_shdiag_mhl_cmd));
	}

	wake_lock_init(&shmhl_wake_lock, WAKE_LOCK_SUSPEND, "shmhl_wake_lock");

	SHMHL_DEBUG_LOG_L("shmhl_init() End\n");

	return ret;
}

static void __exit shmhl_exit(void)
{
	wake_lock_destroy(&shmhl_wake_lock);

	shmhl_free_edid_out_buf();

	device_destroy(shmhl_kdrv_class, shmhl_kdrv_dev);

	class_destroy(shmhl_kdrv_class);

	cdev_del(&shmhl_cdev);

	unregister_chrdev_region(shmhl_major, SH_MHL_DEVICE_COUNT);
}

module_init( shmhl_init );
module_exit( shmhl_exit );

MODULE_DESCRIPTION("SHARP HML DEVICE MODULE");
MODULE_LICENSE("GPL v2");
MODULE_AUTHOR("SHARP CORPORATION");
