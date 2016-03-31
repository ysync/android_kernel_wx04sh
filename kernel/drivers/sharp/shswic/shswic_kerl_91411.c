/* kernel/drivers/sharp/shswic_kerl_91411.c  (SwitchingIC Driver)
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

/* ____________________________________________________________________________________________________________________________ */
/**
 *	Include
 */
#include <linux/kernel.h>
#include <linux/delay.h>
#include <linux/wakelock.h>
#include <sharp/shswic_kerl.h>
#ifdef CONFIG_SII8334_MHL_TX
#include <sharp/shmhl_kerl.h>
#endif /* CONFIG_SII8334_MHL_TX */
#include <linux/gpio.h>
#include <sharp/sh_boot_manager.h>

#include "shswic_kerl_local.h"
#include "shswic_kerl_91411.h"

#define SHSWIC_MHL_PASS_91411				(0x49)	/* (0x00)VBUS  :Open			*/
													/* (0x40)CBUS  :connect			*/
													/* (0x08)HDPR  :HDP2 connect	*/
													/* (0x01)HDML  :HDM2 connect	*/

/* INT_STAT */
#define SHSWIC_INT_STA_USBPORT_MASK_91411	(0xF4)
#define SHSWIC_INT_STA_VBUS_MASK_91411		(0x10)
#ifdef CONFIG_SII8334_MHL_TX
#define SHSWIC_INT_STA_CHGDET_MASK_91411	(0x80)
#endif /* CONFIG_SII8334_MHL_TX */

/* PWR_STAT */
#define SHSWIC_STATUS_CHGPORT_MASK_91411	(0x60)
#ifdef CONFIG_SII8334_MHL_TX
#define SHSWIC_STATUS_PUPDET_MASK_91411		(0x10)
#endif /*CONFIG_SII8334_MHL_TX */
#define SHSWIC_STATUS_DCDFAIL_MASK_91411	(0x80)

/* ID_STAT */
#define SHSWIC_ID_STA_IDRDET_MASK_91411		(0x10)
#define SHSWIC_ID_STA_INDO_MASK_91411		(0x0F)
#define SHSWIC_ID_STA_MASK_91411			(0x7F)
#ifdef CONFIG_HOST_SWIC
#define SHSWIC_ID_STA_HOST_MASK_91411		(0x1F)
#endif /* CONFIG_HOST_SWIC */

enum
{
	SHSWIC_RECHECK_NON_91411,
	SHSWIC_RECHECK_SECOND_91411,
	SHSWIC_RECHECK_HEADSET_91411,
	SHSWIC_RECHECK_CANCEL_91411,
};
	
/* ____________________________________________________________________________________________________________________________ */
/**
 *	Internal prototypes
 */
#ifdef CONFIG_SII8334_MHL_TX
static void shswic_call_mhl_func_91411(uint8_t id_status, uint8_t status, uint8_t int_status);
#endif /* CONFIG_SII8334_MHL_TX */
static bool shswic_is_connected_headset_91411(uint8_t int_status, uint8_t id_status);
static bool shswic_is_connected_switch_headset_91411(uint8_t int_status, uint8_t id_status);
static void shswic_update_headset_91411(uint8_t int_status);
static bool shswic_is_headset_indo_91411(uint8_t id_status);
static void shswic_setup_ucdcnt_91411(void);
static uint8_t shswic_detect_get_91411(uint8_t id_status, uint8_t status, uint8_t int_status);
static uint8_t shswic_detect_get_without_mhl_91411(uint8_t id_status, uint8_t status, uint8_t int_status);
static void shswic_send_register_recheck_signal_91411(long sig);
static void shswic_register_recheck_handler_91411(struct work_struct *poWork);
static void shswic_register_recheck_headset_sig_handler_91411(void);
static void shswic_register_recheck_sig_handler_91411(void);

/* ____________________________________________________________________________________________________________________________ */
/**
 *	Internal Value
 */
static uint8_t					shswic_last_detect_state_91411 = SHSWIC_STATE_NONE;
#ifdef CONFIG_CRADLE_SWIC
static int						shswic_last_cradle_status_91411 = SHSWIC_ID_CRADLE_UNKNOWN;
#endif /* CONFIG_CRADLE_SWIC */
static bool						shswic_interrupt_init_state_91411 = false;
static uint8_t					shswic_1th_check_detect_port_91411 = SHSWIC_ID_NONE;
static struct delayed_work		shswic_register_recheck_signal_pkt_91411;
static int						shswic_register_recheck_signal_pkt_state_91411 = SHSWIC_RECHECK_NON_91411;
static long						shswic_register_recheck_signal_pkt_waiting_sig_91411 = SHSWIC_NO_SIG;
static struct wake_lock 		shswic_delayed_work_wake_lock_91411;
static struct wake_lock			shswic_sleep_wake_lock_91411;
static struct wake_lock			shswic_detect_wake_lock_91411;
static int						shswic_dcdfail_retry_cnt = 0;

/* ____________________________________________________________________________________________________________________________ */
/**
 *	Internal Functions
 */
void shswic_detect_isr_sig_handler_91411(void)
{
	uint8_t int_status = 0x00;
	uint8_t status = 0x00;
	uint8_t id_status = 0x0d;
	uint8_t detect = SHSWIC_ID_NONE;
#ifdef CONFIG_CRADLE_SWIC
	int		cradle_status = SHSWIC_ID_CRADLE_NONE;
#endif /* CONFIG_CRADLE_SWIC */

	SHSWIC_DEBUG_LOG_L("shswic_detect_isr_sig_handler Start");

	if(wake_lock_active(&shswic_delayed_work_wake_lock_91411))
	{
		wake_unlock(&shswic_delayed_work_wake_lock_91411);
	}

	shswic_status_read(&id_status, &status, &int_status);
	shswic_status_read(&id_status, &status, &int_status);

#ifdef CONFIG_CRADLE_SWIC
	if(int_status & 0x08)
	{
		cradle_status = SHSWIC_ID_CRADLE;
	}
	else
	{
		cradle_status = SHSWIC_ID_CRADLE_NONE;
	}
	
	if(cradle_status != shswic_last_cradle_status_91411)
	{
		shswic_last_cradle_status_91411 = cradle_status;
		shswic_state_cradle_proc();
	}
#endif /* CONFIG_CRADLE_SWIC */

	detect = shswic_detect_get_91411(id_status, status, int_status);

	if(SHSWIC_ID_RECHECK != detect)
	{
		shswic_1th_check_detect_port_91411 = detect;
		shswic_send_register_recheck_signal_91411(SHSWIC_REG_RECHECK_SIG);
	}

	return;
}

uint8_t shswic_detect_get_91411(uint8_t id_status, uint8_t status, uint8_t int_status)
{
	uint8_t id_det = 0x00;

	if((status & SHSWIC_STATUS_DCDFAIL_MASK_91411) != 0x00)
	{
#ifdef CONFIG_SII8334_MHL_TX
		if(((int_status & SHSWIC_INT_STA_CHGDET_MASK_91411) == 0x00)
		&& ((status & SHSWIC_STATUS_CHGPORT_MASK_91411) == 0x20))
		{
			SHSWIC_DEBUG_LOG_L("shswic_detect_get:SHSWIC_ID_MHL");
			shswic_dcdfail_retry_cnt = 0;
			return SHSWIC_ID_MHL;
		}
#endif /* CONFIG_SII8334_MHL_TX */

		if (((status & SHSWIC_STATUS_CHGPORT_MASK_91411) == 0x60)
		|| ((status & SHSWIC_STATUS_CHGPORT_MASK_91411) == 0x40))
		{
			SHSWIC_DEBUG_LOG_L("shswic_detect_get:SHSWIC_ID_IRREGULAR_CHARGER");
			shswic_dcdfail_retry_cnt = 0;
			return SHSWIC_ID_IRREGULAR_CHARGER;
		}

		SHSWIC_DEBUG_LOG_L("DCDFAIL");

		if (shswic_dcdfail_retry_cnt < SHSWIC_DCDFAIL_RETRY_CNT_MAX)
		{
			uint8_t ucdcnt = 0x44;
			shswic_result_t retUcdcnt = SHSWIC_FAILURE;

			shswic_dcdfail_retry_cnt++;
			
			/* ADCRETRY */
			shswic_i2c_read(&id_det, 1, SHSWIC_REG_IDDET);
			id_det &= ~0x02;
			shswic_i2c_write(&id_det, 1, SHSWIC_REG_IDDET);

			/* BCSRETRY */
			retUcdcnt = shswic_i2c_read(&ucdcnt, 1, SHSWIC_REG_UCDCNT);
			if (retUcdcnt == SHSWIC_SUCCESS)
			{
				ucdcnt &= ~0x01;
				shswic_i2c_write(&ucdcnt, 1, SHSWIC_REG_UCDCNT);
			}

			wake_lock(&shswic_sleep_wake_lock_91411);
			usleep(30000);
			wake_unlock(&shswic_sleep_wake_lock_91411);

			/* ADCRETRY */
			id_det |= 0x12;
			shswic_i2c_write(&id_det, 1, SHSWIC_REG_IDDET);

			/* BCSRETRY */
			if (retUcdcnt == SHSWIC_SUCCESS)
			{
				ucdcnt |= 0x01;
				shswic_i2c_write(&ucdcnt, 1, SHSWIC_REG_UCDCNT);
			}
			
			SHSWIC_DEBUG_LOG_L("DCDFAIL retry : %d", shswic_dcdfail_retry_cnt);

			if(wake_lock_active(&shswic_delayed_work_wake_lock_91411) == false)
			{
				wake_lock(&shswic_delayed_work_wake_lock_91411);
			}

			return SHSWIC_ID_RECHECK;
		}

		SHSWIC_DEBUG_LOG_L("DCDFAIL retry over");
		return SHSWIC_ID_NONE;
	}

	shswic_dcdfail_retry_cnt = 0;

	if ((id_status & SHSWIC_ID_STA_IDRDET_MASK_91411) == 0x10)
	{
		SHSWIC_DEBUG_LOG_L("shswic_detect_isr_sig_handler IDRDET(1)");
#ifdef CONFIG_HOST_SWIC
		if ((shswic_last_detect_state_91411 == SHSWIC_STATE_USB_HOST_CABLE)
		&& ((id_status & SHSWIC_ID_STA_INDO_MASK_91411) == 0x00))
		{
			SHSWIC_DEBUG_LOG_L("shswic_detect_get:SHSWIC_ID_USB_HOST_CABLE");
			return SHSWIC_ID_USB_HOST_CABLE;
		}
#endif /* CONFIG_HOST_SWIC */

#ifdef CONFIG_SII8334_MHL_TX
		if ((id_status & SHSWIC_ID_STA_INDO_MASK_91411) == 0x0E)
		{
			SHSWIC_DEBUG_LOG_L("shswic_detect_get:SHSWIC_ID_MHL");
			return SHSWIC_ID_MHL;
		}
#endif /* CONFIG_SII8334_MHL_TX */
	}

	return shswic_detect_get_without_mhl_91411(id_status, status, int_status);
}

uint8_t shswic_detect_get_without_mhl_91411(uint8_t id_status, uint8_t status, uint8_t int_status)
{
	SHSWIC_DEBUG_LOG_L("shswic_detect_get INT = 0x%02x, STATUS = 0x%02x, ID = 0x%02x", int_status, status, id_status);

	if (shswic_is_connected_headset_91411(int_status, id_status))
	{
		SHSWIC_DEBUG_LOG_L("shswic_detect_get:SHSWIC_ID_HEADSET");
		return SHSWIC_ID_HEADSET;
	}
		
	if(shswic_is_connected_switch_headset_91411(int_status, id_status))
	{
		if((shswic_last_detect_state_91411 == SHSWIC_STATE_HEADSET)
		|| (shswic_last_detect_state_91411 == SHSWIC_STATE_HEADSET_SW))
		{
			SHSWIC_DEBUG_LOG_L("shswic_detect_get:SHSWIC_ID_HEADSET_SW");
			return SHSWIC_ID_HEADSET_SW;
		}

		SHSWIC_DEBUG_LOG_L("shswic_detect_get:SHSWIC_ID_HEADSET");
		return SHSWIC_ID_HEADSET;
	}

#ifdef CONFIG_HOST_SWIC
	if((id_status & SHSWIC_ID_STA_HOST_MASK_91411) == 0x10)
	{
		SHSWIC_DEBUG_LOG_L("shswic_detect_get:USB_HOST_CABLE");
		return SHSWIC_ID_USB_HOST_CABLE;
	}
#endif /* CONFIG_HOST_SWIC */

	if((status & SHSWIC_STATUS_CHGPORT_MASK_91411) == 0x20)
	{
		SHSWIC_DEBUG_LOG_L("shswic_detect_get:USB_CABLE");
		return SHSWIC_ID_USB_CABLE;
	}

	if ((status & SHSWIC_STATUS_CHGPORT_MASK_91411) == 0x60)
	{
		SHSWIC_DEBUG_LOG_L("shswic_detect_get:AC_ADAPTER");
		return SHSWIC_ID_AC_ADAPTER;
	}

	if ((status & SHSWIC_STATUS_CHGPORT_MASK_91411) == 0x00)
	{
		SHSWIC_DEBUG_LOG_L("shswic_detect_get:NONE");
		return SHSWIC_ID_NONE;
	}

	SHSWIC_DEBUG_LOG_L("shswic_detect_get:%d", shswic_detect_port);
	return shswic_detect_port;
}

void shswic_detect_init_91411(void)
{
	int ret = 0;
	uint8_t id_status = 0x0d;
	uint8_t status = 0x00;
	uint8_t int_status = 0x00;
	uint8_t detect = SHSWIC_ID_NONE;

	SHSWIC_DEBUG_LOG_L("shswic_detect_init Start");

	wake_lock_init(&shswic_delayed_work_wake_lock_91411, WAKE_LOCK_SUSPEND, "shswic_delayed_work_lock_91411");
	wake_lock_init(&shswic_sleep_wake_lock_91411, WAKE_LOCK_SUSPEND, "shswic_sleep_lock_91411");
	wake_lock_init(&shswic_detect_wake_lock_91411, WAKE_LOCK_SUSPEND, "shswic_detect_lock_91411");

#ifdef CONFIG_SII8334_MHL_TX
	shswic_read_data.shswic_mhl_result = SHMHL_DEVICE_MAX;
#endif /* CONFIG_SII8334_MHL_TX */

	shswic_setup_ucdcnt_91411();

	ret = gpio_request(SHSWIC_GPIO_INT, "swic_int");
	if (ret != 0)
	{
		SHSWIC_DEBUG_LOG_H("Error SHSWIC_GPIO_INT request");
	}

	ret = gpio_tlmm_config(GPIO_CFG(SHSWIC_GPIO_INT, 0, GPIO_CFG_INPUT, GPIO_CFG_PULL_UP, GPIO_CFG_2MA), GPIO_CFG_ENABLE);
	if (ret != 0)
	{
		SHSWIC_DEBUG_LOG_H("Error SHSWIC_GPIO_INT config");
	}

	ret = request_irq(MSM_GPIO_TO_INT(SHSWIC_GPIO_INT), shswic_detect_isr, IRQF_TRIGGER_FALLING, "shswic_irq", 0);
	if (ret != 0)
	{
		SHSWIC_DEBUG_LOG_H("Error USB-Int request_irq:Ret %d", ret);
	}
	else
	{
		shswic_interrupt_init_state_91411 = true;
		SHSWIC_DEBUG_LOG_L("Success USB-Int request_irq");
	}

	shswic_status_read(&id_status, &status, &int_status);
	shswic_status_read(&id_status, &status, &int_status);
	detect = shswic_detect_get_91411(id_status, status, int_status);

	if(SHSWIC_ID_RECHECK != detect)
	{
		shswic_1th_check_detect_port_91411 = detect;
		shswic_send_register_recheck_signal_91411(SHSWIC_REG_RECHECK_SIG);
	}

#ifdef CONFIG_CRADLE_SWIC
	if(int_status & 0x08)
	{
		shswic_last_cradle_status_91411 = SHSWIC_ID_CRADLE;
	}
	else
	{
		shswic_last_cradle_status_91411 = SHSWIC_ID_CRADLE_NONE;
	}

	shswic_state_cradle_proc();
#endif /* CONFIG_CRADLE_SWIC */

	return;
}

#ifdef CONFIG_SII8334_MHL_TX
void shswic_detect_mhl_proc_91411(void)
{
	uint8_t detect_port = SHSWIC_ID_NONE;
	uint8_t sw_ctl = 0x00;
	uint8_t id_det = 0x00;
	uint8_t ucdcnt = 0x44;
	shswic_result_t retUcdcnt = SHSWIC_FAILURE;

	SHSWIC_DEBUG_LOG_L("shswic_cb_detect_mhl_proc Called [device=%d]", shswic_read_data.shswic_mhl_result);

	shswic_detect_id = (shswic_read_data.shswic_id_status & SHSWIC_ID_STA_MASK_91411);

	switch(shswic_read_data.shswic_mhl_result)
	{
		case SHMHL_DEVICE_MHL:
			detect_port = SHSWIC_ID_MHL;
			shswic_detect_state = SHSWIC_STATE_MHL;
			SHSWIC_DEBUG_LOG_L("shswic_cb_detect_mhl_proc:MHL");
			break;
		case SHMHL_DEVICE_USB_B:
			if((shswic_read_data.shswic_status & SHSWIC_STATUS_DCDFAIL_MASK_91411) == 0x00)
			{
				detect_port = shswic_detect_get_without_mhl_91411(shswic_read_data.shswic_id_status, shswic_read_data.shswic_status, shswic_read_data.shswic_int_status);
			}
			else
			{
				detect_port = shswic_detect_port;
			}
			shswic_i2c_write(&sw_ctl, 1, SHSWIC_REG_MUXSW_CONTROL);
			SHSWIC_DEBUG_LOG_L("shswic_cb_detect_mhl_proc:SHMHL_DEVICE_USB_B");
			break;
		case SHMHL_DEVICE_UNKNOWN:
		case SHMHL_DEVICE_ACA_MCPC_CHG:
		case SHMHL_DEVICE_USB_A_CHG:
		case SHMHL_DEVICE_STANDBY:
		case SHMHL_DEVICE_USB_B_ACA:
		case SHMHL_DEVICE_MCPC_ACA_OPN:
			detect_port = shswic_detect_port;
			shswic_i2c_write(&sw_ctl, 1, SHSWIC_REG_MUXSW_CONTROL);
			SHSWIC_DEBUG_LOG_L("shswic_cb_detect_mhl_proc:OTHER");
			break;
		case SHMHL_DEVICE_NONE:
			shswic_i2c_write(&sw_ctl, 1, SHSWIC_REG_MUXSW_CONTROL);

			wake_lock(&shswic_sleep_wake_lock_91411);
			usleep(30000);
			wake_unlock(&shswic_sleep_wake_lock_91411);

			/* ADCRETRY */
			shswic_i2c_read(&id_det, 1, SHSWIC_REG_IDDET);
			id_det &= ~0x02;
			shswic_i2c_write(&id_det, 1, SHSWIC_REG_IDDET);

			/* BCSRETRY */
			retUcdcnt = shswic_i2c_read(&ucdcnt, 1, SHSWIC_REG_UCDCNT);
			if (retUcdcnt == SHSWIC_SUCCESS)
			{
				ucdcnt &= ~0x01;
				shswic_i2c_write(&ucdcnt, 1, SHSWIC_REG_UCDCNT);
			}

			wake_lock(&shswic_sleep_wake_lock_91411);
			usleep(30000);
			wake_unlock(&shswic_sleep_wake_lock_91411);

			/* ADCRETRY */
			id_det |= 0x12;
			shswic_i2c_write(&id_det, 1, SHSWIC_REG_IDDET);

			/* BCSRETRY */
			if (retUcdcnt == SHSWIC_SUCCESS)
			{
				ucdcnt |= 0x01;
				shswic_i2c_write(&ucdcnt, 1, SHSWIC_REG_UCDCNT);
			}

			SHSWIC_DEBUG_LOG_L("shswic_cb_detect_mhl_proc:NONE");
			return;
		case SHMHL_DEVICE_MAX:
			/* request MHL chipset check, now. */
			SHSWIC_DEBUG_LOG_L("shswic_cb_detect_mhl_proc:SHMHL_DEVICE_MAX");
			break;
		case SHMHL_DEVICE_DISC_RETRY_OVER:
			shswic_i2c_write(&sw_ctl, 1, SHSWIC_REG_MUXSW_CONTROL);
			SHSWIC_DEBUG_LOG_L("shswic_cb_detect_mhl_proc:SHMHL_DEVICE_DISC_RETRY_OVER");
			break;
		default:
			shswic_i2c_write(&sw_ctl, 1, SHSWIC_REG_MUXSW_CONTROL);
			SHSWIC_DEBUG_LOG_L("shswic_cb_detect_mhl_proc:default");
			break;
	}

	shswic_state_proc[shswic_last_detect_state_91411](detect_port);
	shswic_last_detect_state_91411 = shswic_detect_state;
}

static void shswic_call_mhl_func_91411(uint8_t id_status, uint8_t status, uint8_t int_status)
{
	uint8_t sw_ctl = SHSWIC_MHL_PASS_91411;
	/* idrdet flg 1 : id line low */
	/*            0 : id line hi  */
	bool id_is_hi = ((id_status & SHSWIC_ID_STA_IDRDET_MASK_91411) == 0x00);
	/* vbusdet flg 1 : vbus line hi  */
	/*             0 : vbus line low */
	bool vbas_is_hi = ((int_status & SHSWIC_INT_STA_VBUS_MASK_91411) != 0x00);

	shswic_read_data.shswic_id_status = id_status;
	shswic_read_data.shswic_status = status;
	shswic_read_data.shswic_int_status = int_status;

	shswic_i2c_write(&sw_ctl, 1, SHSWIC_REG_MUXSW_CONTROL);

	if(((int_status & SHSWIC_INT_STA_CHGDET_MASK_91411) == 0x00)
		&& ((status & SHSWIC_STATUS_CHGPORT_MASK_91411) == 0x20)
		&& ((id_status & SHSWIC_ID_STA_IDRDET_MASK_91411) == 0x00)) {
		SHSWIC_DEBUG_LOG_L("usb-pc charging start.");
		shmhl_notify_usb_pc_charge(true);
	}

	shmhl_detect_cb_regist(shswic_detect_cb_mhl, NULL);
	shmhl_notify_id_vbus_state(id_is_hi, vbas_is_hi);
}
#endif /* CONFIG_SII8334_MHL_TX */

shswic_result_t shswic_write_vbsw_reg_91411(uint8_t vbsw_ctl)
{
	uint8_t ovp_ctl = 0x00;
	shswic_result_t result = SHSWIC_FAILURE;
	
	result = shswic_i2c_read(&ovp_ctl, 1, SHSWIC_REG_VBSW_CONTROL);
	if(SHSWIC_SUCCESS != result)
	{
		return result;
	}

	SHSWIC_DEBUG_LOG_L("shswic_write_vbsw_reg_91411 ovp_ctl: 0x%x", ovp_ctl);
	
	switch(vbsw_ctl)
	{
		case SHSWIC_VBSW_OFF:
			ovp_ctl |= 0x01; /* VBSWDISEN FLG is ON. */
			break;
			
		case SHSWIC_VBSW_AUTO:
			ovp_ctl &= 0xFE; /* VBSWDISEN FLG is OFF. */
			break;
			
		default:
			return SHSWIC_PARAM_ERROR;
	}

	SHSWIC_DEBUG_LOG_L("shswic_write_vbsw_reg_91411 ovp_ctl: 0x%x", ovp_ctl);
	result = shswic_i2c_write(&ovp_ctl, 1, SHSWIC_REG_VBSW_CONTROL);
	return result;
}

shswic_result_t shswic_read_vbsw_reg_91411(uint8_t* vbsw_ctl)
{
	if (vbsw_ctl == NULL)
	{
		SHSWIC_DEBUG_LOG_H("shswic_read_vbsw_reg_91411 SHSWIC_PARAM_ERROR");
		return SHSWIC_PARAM_ERROR;
	}
	return shswic_i2c_read(vbsw_ctl, 1, SHSWIC_REG_VBSW_CONTROL);
}

void shswic_pre_detect_cb_call_91411(uint8_t detect)
{
#ifdef CONFIG_CRADLE_SWIC
	if((SHSWIC_ID_CRADLE == detect)
	|| (SHSWIC_ID_CRADLE_NONE == detect))
	{
		return;
	}
#endif /* CONFIG_CRADLE_SWIC */

#ifdef CONFIG_HOST_SWIC
	if (detect == SHSWIC_ID_USB_HOST_CABLE)
	{
		shswic_write_vbsw_reg(SHSWIC_VBSW_OFF);
	}
	else
	{
		shswic_write_vbsw_reg(SHSWIC_VBSW_AUTO);
	}
#endif /* CONFIG_HOST_SWIC */
}

void shswic_pre_do_exit_91411(void)
{
	if(shswic_interrupt_init_state_91411)
	{
		shswic_interrupt_init_state_91411 = false;
		disable_irq(MSM_GPIO_TO_INT(SHSWIC_GPIO_INT));
		free_irq(MSM_GPIO_TO_INT(SHSWIC_GPIO_INT), 0);
	}

	gpio_free(SHSWIC_GPIO_INT);
}

#ifdef CONFIG_CRADLE_SWIC
int shswic_cradle_state_91411(void)
{
	return shswic_last_cradle_status_91411;
}
#endif /* CONFIG_CRADLE_SWIC */

static bool shswic_is_connected_headset_91411(uint8_t int_status, uint8_t id_status)
{
	if((int_status & SHSWIC_INT_STA_USBPORT_MASK_91411) != 0x00)
	{
		/* not headset */
		SHSWIC_DEBUG_LOG_L("not headset: int_status != 0x00");
		return false;
	}

	return shswic_is_headset_indo_91411(id_status);
}

static bool shswic_is_connected_switch_headset_91411(uint8_t int_status, uint8_t id_status)
{
	if((id_status & SHSWIC_ID_STA_INDO_MASK_91411) == 0x02)
	{
		/* push headset button */
		return true;
	}

	if((int_status & SHSWIC_INT_STA_USBPORT_MASK_91411) != 0x20)
	{
		/* no COMPL flg */
		SHSWIC_DEBUG_LOG_L("not COMPL flag: int_status != 0x20");
		return false;
	}

	return shswic_is_headset_indo_91411(id_status);
}

static bool shswic_is_headset_indo_91411(uint8_t id_status)
{
	if((id_status & SHSWIC_ID_STA_INDO_MASK_91411) == 0x0C)
	{
		/* stereo */
		return true;
	}

	if((id_status & SHSWIC_ID_STA_INDO_MASK_91411) == 0x08)
	{
		/* mono */
		return true;
	}
	
	return false;
}

bool shswic_is_push_headset_button_91411(uint8_t int_status)
{
	return ((int_status & SHSWIC_INT_STA_USBPORT_MASK_91411) == 0x20);
}

static void shswic_update_headset_91411(uint8_t int_status)
{
	uint8_t sw_ctl = 0x00;

	if((shswic_last_detect_state_91411 != SHSWIC_STATE_HEADSET_SW)
	&& (shswic_last_detect_state_91411 != SHSWIC_STATE_HEADSET))
	{
		sw_ctl = SHSWIC_NONE_PASS;
		shswic_i2c_write(&sw_ctl, 1, SHSWIC_REG_MUXSW_CONTROL);
		
		wake_lock(&shswic_sleep_wake_lock_91411);
		usleep(100000);
		wake_unlock(&shswic_sleep_wake_lock_91411);

		sw_ctl = SHSWIC_HEADSET_PASS;
		shswic_i2c_write(&sw_ctl, 1, SHSWIC_REG_MUXSW_CONTROL);
		
	}
#ifdef CONFIG_HEADSET_BUTTON_SWIC
	else
	{
		shswic_send_headset_sw_button(int_status);
	}
#endif /* CONFIG_HEADSET_BUTTON_SWIC */
}

void shswic_setup_ucdcnt_91411(void)
{
	int ret = 0;
	uint8_t hwRevision = 0x00;
	uint8_t ucdcnt = 0x44;	/* 0x40 : INTBEN */
							/* 0x04 : USBDETCTRL */
	bool vbregdisen = true;
	
	hwRevision = sh_boot_get_hw_revision();

#ifdef CONFIG_HARDWARE_REV_TYPE_1_SWIC
	switch(hwRevision)
	{
		case 0x6: /* REV_PP1 */
		case 0x3: /* REV_PP15 */
		case 0x7: /* REV_PP2 */
			vbregdisen = true;
			break;
		
		default:
			vbregdisen = false;
			break;
	}
#endif /* CONFIG_HARDWARE_REV_TYPE_1_SWIC */

#ifdef CONFIG_HARDWARE_REV_TYPE_2_SWIC
	switch(hwRevision)
	{
		case 0x4: /* REV_ES0 */
			vbregdisen = false;
			break;
		
		default:
			vbregdisen = true;
			break;
	}
#endif /* CONFIG_HARDWARE_REV_TYPE_2_SWIC */

	if(vbregdisen)
	{
		ret = shswic_i2c_read(&ucdcnt, 1, SHSWIC_REG_UCDCNT);
		if (ret == SHSWIC_SUCCESS)
		{
			ucdcnt |= 0x04;	/* set USBDETCTRL FLG */
		}
		else
		{
			ucdcnt = 0x44;
		}
	}
	else
	{
		ret = shswic_i2c_read(&ucdcnt, 1, SHSWIC_REG_UCDCNT);
		if (ret == SHSWIC_SUCCESS)
		{
			ucdcnt &= 0xFE;	/* clear DCDRETRY FLG */
		}
		else
		{
			ucdcnt = 0x40;
		}
	}
	
	ret = shswic_i2c_write(&ucdcnt, 1, SHSWIC_REG_UCDCNT);
	SHSWIC_DEBUG_LOG_L("shswic_setup_ucdcnt_91411 ret(%d) ucdcnt(0x%x)", ret, ucdcnt);

	wake_lock(&shswic_sleep_wake_lock_91411);
	usleep(30000);
	wake_unlock(&shswic_sleep_wake_lock_91411);

	ucdcnt |= 0x01;	/* set DCDRETRY FLG */
	ret = shswic_i2c_write(&ucdcnt, 1, SHSWIC_REG_UCDCNT);
	SHSWIC_DEBUG_LOG_L("shswic_setup_ucdcnt_91411 ret(%d) ucdcnt(0x%x)", ret, ucdcnt);
}

static void shswic_register_recheck_sig_handler_91411(void)
{
	uint8_t id_status, status, int_status;
	uint8_t detect_port = shswic_detect_port;
	uint8_t sw_ctl = 0x00;

	SHSWIC_DEBUG_LOG_L("shswic_register_recheck_sig_handler_91411");

	shswic_status_read(&id_status, &status, &int_status);
	shswic_status_read(&id_status, &status, &int_status);

	detect_port = shswic_detect_get_91411(id_status, status, int_status);

	if(detect_port != shswic_1th_check_detect_port_91411)
	{
		SHSWIC_DEBUG_LOG_L("detect_port != shswic_1th_check_detect_port_91411");
		shswic_1th_check_detect_port_91411 = detect_port;
		shswic_send_register_recheck_signal_91411(SHSWIC_REG_RECHECK_SIG);
		return;
	}
	
	if((int_status & SHSWIC_INT_STA_VBUS_MASK_91411) == 0x00) {
		SHSWIC_DEBUG_LOG_L("usb-pc charging stop.");
		shmhl_notify_usb_pc_charge(false);
	}
	
	switch(detect_port)
	{
		case SHSWIC_ID_MHL:
#ifdef CONFIG_SII8334_MHL_TX
			/* mhl application without vbus */
			shswic_call_mhl_func_91411(id_status, status, int_status);
#endif /* CONFIG_SII8334_MHL_TX */
			return;

		case SHSWIC_ID_HEADSET:
			if(SHSWIC_ID_HEADSET_SW == shswic_last_detect_state_91411)
			{
				SHSWIC_DEBUG_LOG_L("shswic_register_recheck_sig_handler_91411 shswic_detect_get:HEADSET");
				shswic_detect_state = SHSWIC_STATE_HEADSET;
				shswic_update_headset_91411(int_status);
			}
			else
			{
				shswic_send_register_recheck_signal_91411(SHSWIC_REG_RECHECK_HEADSET_SIG);
				return;
			}
			break;
		
		case SHSWIC_ID_HEADSET_SW:
			SHSWIC_DEBUG_LOG_L("shswic_register_recheck_sig_handler_91411:HEADSET_SW");
			shswic_detect_state = SHSWIC_STATE_HEADSET_SW;
#ifdef CONFIG_HEADSET_BUTTON_SWIC
			shswic_switch_earphone = true;
#endif /* CONFIG_HEADSET_BUTTON_SWIC */
			shswic_update_headset_91411(int_status);
			break;
		
#ifdef CONFIG_HOST_SWIC
		case SHSWIC_ID_USB_HOST_CABLE:
			SHSWIC_DEBUG_LOG_L("shswic_register_recheck_sig_handler_91411:USB_HOST_CABLE");
			shswic_detect_state = SHSWIC_STATE_USB_HOST_CABLE;
#ifdef CONFIG_HEADSET_BUTTON_SWIC
			shswic_switch_earphone = false;
#endif /* CONFIG_HEADSET_BUTTON_SWIC */
			break;
#endif /* CONFIG_HOST_SWIC */

		case SHSWIC_ID_USB_CABLE:
			SHSWIC_DEBUG_LOG_L("shswic_register_recheck_sig_handler_91411:USB_CABLE");
			shswic_detect_state = SHSWIC_STATE_USB_CABLE;
#ifdef CONFIG_HEADSET_BUTTON_SWIC
			shswic_switch_earphone = false;
#endif /* CONFIG_HEADSET_BUTTON_SWIC */
			break;
		
		case SHSWIC_ID_AC_ADAPTER:
			SHSWIC_DEBUG_LOG_L("shswic_register_recheck_sig_handler_91411:AC_ADAPTER");
			shswic_detect_state = SHSWIC_STATE_AC_ADAPTER;
#ifdef CONFIG_HEADSET_BUTTON_SWIC
			shswic_switch_earphone = false;
#endif /* CONFIG_HEADSET_BUTTON_SWIC */
			break;

		case SHSWIC_ID_IRREGULAR_CHARGER:
			SHSWIC_DEBUG_LOG_L("shswic_register_recheck_sig_handler_91411:IRREGULAR_CHARGER");
			shswic_detect_state = SHSWIC_STATE_IRREGULAR_CHARGER;
#ifdef CONFIG_HEADSET_BUTTON_SWIC
			shswic_switch_earphone = false;
#endif /* CONFIG_HEADSET_BUTTON_SWIC */
			break;

		case SHSWIC_ID_NONE:
			SHSWIC_DEBUG_LOG_L("shswic_register_recheck_sig_handler_91411:NONE");
			shswic_detect_state = SHSWIC_STATE_NONE;
#ifdef CONFIG_HEADSET_BUTTON_SWIC
			shswic_switch_earphone = false;
#endif /* CONFIG_HEADSET_BUTTON_SWIC */
			shswic_i2c_write(&sw_ctl, 1, SHSWIC_REG_MUXSW_CONTROL);
			
			if((int_status & SHSWIC_INT_STA_VBUS_MASK_91411) == 0x10) {
				if(wake_lock_active(&shswic_detect_wake_lock_91411)) {
					wake_unlock(&shswic_detect_wake_lock_91411);
				}
				/* timeout after 1 sec */
				wake_lock_timeout(&shswic_detect_wake_lock_91411, 1 * HZ);
			}
			break;
		
		case SHSWIC_ID_RECHECK:
			return;
		
		default:
			break;
	}

	shswic_detect_id = (id_status & SHSWIC_ID_STA_MASK_91411);

	shswic_state_proc[shswic_last_detect_state_91411](detect_port);

	shswic_last_detect_state_91411 = shswic_detect_state;

	return;
}

static void shswic_register_recheck_headset_sig_handler_91411(void)
{
	uint8_t id_status, status, int_status;
	uint8_t detect_port = shswic_detect_port;

	SHSWIC_DEBUG_LOG_L("shswic_register_recheck_headset_sig_handler_91411");

	shswic_status_read(&id_status, &status, &int_status);
	shswic_status_read(&id_status, &status, &int_status);

	detect_port = shswic_detect_get_91411(id_status, status, int_status);

	if(detect_port != SHSWIC_ID_HEADSET)
	{
		SHSWIC_DEBUG_LOG_L("detect_port != SHSWIC_ID_HEADSET");
		return;
	}

	shswic_detect_state = SHSWIC_STATE_HEADSET;
	shswic_update_headset_91411(int_status);

	shswic_detect_id = (id_status & SHSWIC_ID_STA_MASK_91411);

	shswic_state_proc[shswic_last_detect_state_91411](detect_port);

	shswic_last_detect_state_91411 = shswic_detect_state;
}

static void shswic_send_register_recheck_signal_91411(long sig)
{
	int msec;
	struct delayed_work* pkt_p;
	bool non_delayed_work = false;

	SHSWIC_DEBUG_LOG_L("recheck_work_add_start! sig = %ld, shswic_register_recheck_signal_pkt_state_91411 = %d", sig, shswic_register_recheck_signal_pkt_state_91411);

	if(wake_lock_active(&shswic_delayed_work_wake_lock_91411))
	{
		wake_unlock(&shswic_delayed_work_wake_lock_91411);
	}

	pkt_p = &(shswic_register_recheck_signal_pkt_91411);

	if(SHSWIC_RECHECK_NON_91411 != shswic_register_recheck_signal_pkt_state_91411)
	{
		non_delayed_work = cancel_delayed_work(pkt_p);
		SHSWIC_DEBUG_LOG_L("cancel_delayed_work(%d)", non_delayed_work ? 1 : 0);
	}
	else
	{
		non_delayed_work = true;
	}

	shswic_register_recheck_signal_pkt_waiting_sig_91411 = SHSWIC_NO_SIG;

	switch(sig)
	{
		case SHSWIC_REG_RECHECK_SIG:
			shswic_register_recheck_signal_pkt_state_91411 = SHSWIC_RECHECK_SECOND_91411;
			msec = 50;
			break;
			
		case SHSWIC_REG_RECHECK_HEADSET_SIG:
			shswic_register_recheck_signal_pkt_state_91411 = SHSWIC_RECHECK_HEADSET_91411;
			msec = 300;
			break;
		
		default:
			/* SHSWIC_REG_RECHECK_NON_SIG */
			/* no supported SIG */
			return;
	}

	if(non_delayed_work == false)
	{
		SHSWIC_DEBUG_LOG_L("non_delayed_work == false");
		shswic_register_recheck_signal_pkt_waiting_sig_91411 = sig;
		shswic_register_recheck_signal_pkt_state_91411 = SHSWIC_RECHECK_CANCEL_91411;
		return;
	}
	
	INIT_DELAYED_WORK(pkt_p, shswic_register_recheck_handler_91411);
	wake_lock(&shswic_delayed_work_wake_lock_91411);
	queue_delayed_work(shswic_wq, pkt_p, msecs_to_jiffies(msec));
	SHSWIC_DEBUG_LOG_L("add workqueue");
	return;
}

static void shswic_register_recheck_handler_91411(struct work_struct *poWork)
{
	struct delayed_work* pkt_p;
	int state;
	
	if(wake_lock_active(&shswic_delayed_work_wake_lock_91411))
	{
		wake_unlock(&shswic_delayed_work_wake_lock_91411);
	}
	
	state = shswic_register_recheck_signal_pkt_state_91411;
	shswic_register_recheck_signal_pkt_state_91411 = SHSWIC_RECHECK_NON_91411;

	pkt_p = (struct delayed_work*)poWork;

	switch(state)
	{
		case SHSWIC_RECHECK_SECOND_91411:
			SHSWIC_DEBUG_LOG_L("SHSWIC_REG_RECHECK_SIG");
			shswic_register_recheck_sig_handler_91411();
			break;
		
		case SHSWIC_RECHECK_HEADSET_91411:
			SHSWIC_DEBUG_LOG_L("SHSWIC_REG_RECHECK_HEADSET_SIG");
			shswic_register_recheck_headset_sig_handler_91411();
			break;

		case SHSWIC_RECHECK_CANCEL_91411:
			SHSWIC_DEBUG_LOG_L("SHSWIC_RECHECK_CANCEL_91411");
			shswic_send_register_recheck_signal_91411(shswic_register_recheck_signal_pkt_waiting_sig_91411);
			break;
			
		default:
			break;
	}
}
