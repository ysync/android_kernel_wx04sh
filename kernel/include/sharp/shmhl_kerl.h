/* include/sharp/shmhl_kerl.h (SHARP MHL Driver)
 *
 * Copyright (c) 2010, Sharp. All rights reserved.
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

#ifndef __SHMHL_KERL_HEADER__
#define __SHMHL_KERL_HEADER__

// ____________________________________________________________________________________________________________________________
/**
 *	Include
 */
#include <stdbool.h>
#include <linux/ioctl.h>

// ____________________________________________________________________________________________________________________________
/**
 *	Define
 */
#define SH_MHL_IOCTL_MAGIC 'h'

#define SH_MHL_IOCTL_GET_DEV_ID							_IO(SH_MHL_IOCTL_MAGIC, 0)
#define SH_MHL_IOCTL_SET_SHDIAG_MHL_CMD					_IO(SH_MHL_IOCTL_MAGIC, 1)
#define SH_MHL_IOCTL_GET_CONNECT_STATE					_IO(SH_MHL_IOCTL_MAGIC, 2)

/* SHDiag MHL COMMAND */
#define SH_MHL_SHDIAG_CMD_NORMALMODE_DEFAULT	"FFF"
#define SH_MHL_SHDIAG_CMD_TESTMODE_DEFAULT		"111"
#define SH_MHL_SHDIAG_CMD_OUTPUT_STOP			"000"

// ____________________________________________________________________________________________________________________________
/**
 *	Prototypes
 */
int shmhl_set_port_vcc12_up(void);
int shmhl_set_port_vcc12_down(void);
int shmhl_set_port_vcc5_up(void);
int shmhl_set_port_vcc5_down(void);
int shmhl_set_port_vcc18_up(void);
int shmhl_set_port_vcc18_down(void);
void shmhl_set_dev_id(uint8_t low, uint8_t high);

typedef enum
{
	SHMHL_VBUS_STATE_OFF = 0,
	SHMHL_VBUS_STATE_ON,
	SHMHL_VBUS_STATE_MAX,
}shmhl_vbus_status_t;

typedef enum
{
	SHMHL_SI_STATE_NULL = 0,
	SHMHL_SI_STATE_RGND,
	SHMHL_SI_STATE_MHL_EST,
	SHMHL_SI_STATE_DISC_FAIL,
	SHMHL_SI_STATE_HAVE_DEV_CAT,
	SHMHL_SI_STATE_DISCONNECTED,
	SHMHL_SI_STATE_MAX,
}shmhl_si_status_t;

typedef enum
{
	SHMHL_CON_STATE_WAIT_PLUG = 0,
	SHMHL_CON_STATE_RGND,
	SHMHL_CON_STATE_DISCOVERY,
	SHMHL_CON_STATE_MHL_EST,
	SHMHL_CON_STATE_HAVE_DEV_CAT,
	SHMHL_CON_STATE_WAIT_UNPLUG,
	SHMHL_CON_STATE_MAX,
} shmhl_con_state_t;

typedef enum
{
	SHMHL_DEVICE_UNKNOWN = 0,	/* Unknown */
	SHMHL_DEVICE_MHL,			/* MHL(SINK),MHL(Dongle), MHL(Standby) */
	SHMHL_DEVICE_ACA_MCPC_CHG,	/* ACA(CHG),MCPC */
	SHMHL_DEVICE_USB_A_CHG,		/* Not Use */
	SHMHL_DEVICE_STANDBY,		/* Not Use */
	SHMHL_DEVICE_USB_B_ACA,		/* USB_B(ACA) */
	SHMHL_DEVICE_USB_B,			/* USB_B,USB_B(ACA) */
	SHMHL_DEVICE_MCPC_ACA_OPN,	/* Not Use */
	SHMHL_DEVICE_NONE,			/* Disconnect */
	SHMHL_DEVICE_DISC_RETRY_OVER,	/* Discovery retry over */
	SHMHL_DEVICE_MAX,
}shmhl_detect_device_t;

typedef enum
{
	SHMHL_DEBUG_LOG_L = 0,
	SHMHL_DEBUG_LOG_TRACE,
	SHMHL_DEBUG_LOG_TX,
	SHMHL_DEBUG_LOG_CBUS,
	SHMHL_DEBUG_LOG_DUMP,
	SHMHL_DEBUG_LOG_EDID,
	SHMHL_DEBUG_LOG_HDMI,
	SHMHL_DEBUG_LOG_MAX,
}shmhl_debug_log_type_t;

void shmhl_notify_si_init_comp(void);
typedef void (*shmhl_cb_func_t)(shmhl_detect_device_t device, void* user_data);
void shmhl_detect_cb_regist(shmhl_cb_func_t cb_func, void* user_data);
void shmhl_notify_id_low(shmhl_vbus_status_t vbus);
void shmhl_notify_id_vbus_state(bool id, bool vbus);
void shmhl_notify_si_state(shmhl_si_status_t si_state, void* data);
shmhl_con_state_t shmhl_get_con_state(void);
void shmhl_notify_usb_pc_charge(bool shmhl_usb_pc_charge_state);

bool shmhl_rcp_key_support_judge(uint8_t rcp_data);
void shmhl_input_rcp_key(uint8_t rcp_data);
void shmhl_rcp_input_register_device(void);
void shmhl_rcp_input_unregister_device(void);
void shmhl_get_shdiag_mhl_cmd(char* cmd);

bool shmhl_get_debug_flg(shmhl_debug_log_type_t log_type);
void shmhl_SiI8334_Reg_Dump_All_Offset(char *buf, size_t size);
char *shmhl_get_edid_out_buf(int block);

#endif	//__SHMHL_KERL_HEADER__

