/* kernel/drivers/sharp/shswic_kerl_91401.c  (SwitchingIC Driver)
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

#include "shswic_kerl_local.h"
#include "shswic_kerl_91401.h"

#define SHSWIC_MHL_PASS_91401			(0x09)	/* (0x00)VBUS  :Open			*/
												/* (0x08)HDPR  :HDP2 connect	*/
												/* (0x01)HDML  :HDM2 connect	*/
#define SHSWIC_ID_DETECT_MASK_91401		(0xF0)

/* Register Mask */
#define SHSWIC_INT_STA_MASK_91401			(0xF0)
#define SHSWIC_STATUS_MASK_91401			(0xE0)
#define SHSWIC_ID_STA_MASK_91401			(0x0F)
#ifdef CONFIG_HOST_SWIC
#define SHSWIC_ID_HOST_MASK_91401			(0xFF)
#endif /* CONFIG_HOST_SWIC */
#define SHSWIC_ID_STA_IDRDET_MASK_91401		(0x10)
#define SHSWIC_INT_VBUS_MASK_91401			(0x10)

/* ____________________________________________________________________________________________________________________________ */
/**
 *	Internal prototypes
 */
#ifdef CONFIG_SII8334_MHL_TX
static void shswic_call_mhl_func_91401(uint8_t id_status, uint8_t status, uint8_t int_status, shmhl_vbus_status_t vbus_status);
#endif /* CONFIG_SII8334_MHL_TX */

/* ____________________________________________________________________________________________________________________________ */
/**
 *	Internal Value
 */
#ifdef CONFIG_SII8334_MHL_TX
static uint8_t					shswic_adc_enable_91401 = 0x12;
static uint8_t					shswic_adc_disable_91401 = 0x00;
static bool						shswic_id_detect_retry_flg_91401 = false;
#endif /* CONFIG_SII8334_MHL_TX */
#ifdef CONFIG_CRADLE_SWIC
static bool						shswic_cradle_interrupt_init_state_91401 = false;
#endif /* CONFIG_CRADLE_SWIC */
static uint8_t					shswic_last_detect_state_91401 = SHSWIC_STATE_NONE;
static uint8_t					shswic_last_status_91401 = 0x00;
static uint8_t					shswic_last_int_status_91401 = 0x10;
static bool						shswic_timer_flag_91401 = false;
static bool						shswic_interrupt_init_state_91401 = false;
static struct wake_lock 		shswic_sleep_wake_lock_91401;

/* ____________________________________________________________________________________________________________________________ */
/**
 *	Internal Functions
 */
void shswic_detect_isr_sig_handler_91401(void)
{
	uint8_t int_status = 0;
	uint8_t status = 0;
	uint8_t id_status = 0;
	uint8_t detect_port = SHSWIC_ID_NONE;

	SHSWIC_DEBUG_LOG_L("shswic_detect_isr_sig_handler Start");

	shswic_clear_timer_signal(SHSWIC_DETECT_RETRY_SIG);
	shswic_timer_flag_91401 = false;

	shswic_status_read(&id_status, &status, &int_status);
	shswic_status_read(&id_status, &status, &int_status);

	if ((int_status & SHSWIC_INT_VBUS_MASK_91401) == 0x00)
	{
		SHSWIC_DEBUG_LOG_L("shswic_detect_isr_sig_handler VBUS-OFF");
		if ((id_status & SHSWIC_ID_DETECT_MASK_91401) == 0x10)
		{
			SHSWIC_DEBUG_LOG_L("shswic_detect_isr_sig_handler IDRDET(1)");
#ifdef CONFIG_SII8334_MHL_TX
			if (shswic_regulator_power(true) != SHSWIC_REGULATOR_SUCCESS)
			{
				SHSWIC_DEBUG_LOG_H("shswic_regulator_power Error");
			}
#endif /* CONFIG_SII8334_MHL_TX */

			shswic_status_read(&id_status, &status, &int_status);
			shswic_status_read(&id_status, &status, &int_status);

#ifdef CONFIG_HOST_SWIC
			if ((shswic_last_detect_state_91401 == SHSWIC_STATE_USB_HOST_CABLE)
			&& ((id_status & SHSWIC_ID_DETECT_MASK_91401) == 0x10)
			&& ((id_status & SHSWIC_ID_STA_MASK_91401) == 0x00))
			{
				SHSWIC_DEBUG_LOG_L("shswic_detect_isr_sig_handler HOST Return");
				return;
			}
#endif /* CONFIG_HOST_SWIC */

#ifdef CONFIG_SII8334_MHL_TX
			if ((shswic_id_detect_retry_flg_91401 == false)
#ifndef CONFIG_HEADSET_BUTTON_SWIC
			 && (shswic_last_detect_state_91401 != SHSWIC_STATE_HEADSET)
			 && (shswic_last_detect_state_91401 != SHSWIC_STATE_HEADSET_SW)
#endif /* CONFIG_HEADSET_BUTTON_SWIC */
#ifdef CONFIG_HOST_SWIC
			 && (shswic_last_detect_state_91401 != SHSWIC_STATE_USB_HOST_CABLE)
#endif /* CONFIG_HOST_SWIC */
			 && ((id_status & SHSWIC_ID_DETECT_MASK_91401) == 0x10))
			{
				SHSWIC_DEBUG_LOG_L("shswic_detect_isr_sig_handler ID Detect Host[0x%02x]", id_status);

				shswic_i2c_write(&shswic_adc_disable_91401, 1, SHSWIC_REG_IDDET);

				wake_lock(&shswic_sleep_wake_lock_91401);
				usleep(30000);
				wake_unlock(&shswic_sleep_wake_lock_91401);

				shswic_i2c_write(&shswic_adc_enable_91401, 1, SHSWIC_REG_IDDET);

				shswic_id_detect_retry_flg_91401 = true;
				return;
			}
			shswic_id_detect_retry_flg_91401 = false;
#endif /* CONFIG_SII8334_MHL_TX */
#ifdef CONFIG_SII8334_MHL_TX
			if ((id_status & SHSWIC_ID_STA_MASK_91401) == 0x00)
			{
				SHSWIC_DEBUG_LOG_L("shswic_detect_isr_sig_handler call shswic_call_mhl_func_91401");
				shswic_call_mhl_func_91401(id_status, status, int_status, SHMHL_VBUS_STATE_OFF);
				return;
			}
#endif /* CONFIG_SII8334_MHL_TX */
		}
	}
	else if((int_status & SHSWIC_INT_VBUS_MASK_91401) == 0x10)
	{
		SHSWIC_DEBUG_LOG_L("shswic_detect_isr_sig_handler VBUS-ON");

#ifdef CONFIG_SII8334_MHL_TX
		if (shswic_regulator_power(false) != SHSWIC_REGULATOR_SUCCESS)
		{
			SHSWIC_DEBUG_LOG_H("shswic_regulator_power Error");
		}
#endif /* CONFIG_SII8334_MHL_TX */

		if ((id_status & SHSWIC_ID_DETECT_MASK_91401) == 0x10)
		{
			SHSWIC_DEBUG_LOG_L("shswic_detect_isr_sig_handler IDRDET(1)");

#ifdef CONFIG_HOST_SWIC
			if ((shswic_last_detect_state_91401 == SHSWIC_STATE_USB_HOST_CABLE)
			&& ((id_status & SHSWIC_ID_STA_MASK_91401) == 0x00))
			{
				SHSWIC_DEBUG_LOG_L("shswic_detect_isr_sig_handler HOST Return");
				return;
			}
#endif /* CONFIG_HOST_SWIC */

#ifdef CONFIG_SII8334_MHL_TX
			if ((id_status & SHSWIC_ID_STA_MASK_91401) == 0x00)
			{
				SHSWIC_DEBUG_LOG_L("shswic_detect_isr_sig_handler call shswic_call_mhl_func_91401");
				shswic_call_mhl_func_91401(id_status, status, int_status, SHMHL_VBUS_STATE_ON);
				return;
			}
#endif /* CONFIG_SII8334_MHL_TX */
		}
	}

	detect_port = shswic_detect_get_91401(id_status, status, int_status);

#ifdef CONFIG_SII8334_MHL_TX
	if ((detect_port != SHSWIC_ID_USB_HOST_CABLE)
#ifdef CONFIG_HEADSET_BUTTON_SWIC
	&&  (detect_port != SHSWIC_ID_HEADSET)
	&&  (detect_port != SHSWIC_ID_HEADSET_SW)
#endif /* CONFIG_HEADSET_BUTTON_SWIC */
	)
	{
		if (shswic_regulator_power(false) != SHSWIC_REGULATOR_SUCCESS)
		{
			SHSWIC_DEBUG_LOG_H("shswic_regulator_power Error");
		}
	}
#endif /* CONFIG_SII8334_MHL_TX */

	shswic_state_proc[shswic_last_detect_state_91401](detect_port);

	shswic_last_detect_state_91401 = shswic_detect_state;
	shswic_last_status_91401 = status; 
	shswic_last_int_status_91401 = int_status;

	return;
}

uint8_t shswic_detect_get_91401(uint8_t id_status, uint8_t status, uint8_t int_status)
{
	static uint8_t cnt = 0;
	uint8_t sw_ctl = 0x00;
	uint8_t dcd_en = 0x40;

	SHSWIC_DEBUG_LOG_L("shswic_detect_get INT = 0x%02x, STATUS = 0x%02x, ID = 0x%02x", int_status, status, id_status);
	
	shswic_detect_id = (id_status & SHSWIC_ID_STA_MASK_91401);

	if (((int_status & SHSWIC_INT_STA_MASK_91401) == 0x00)
	 && (((id_status & SHSWIC_ID_STA_MASK_91401) == 0x0C)
	  || ((id_status & SHSWIC_ID_STA_MASK_91401) == 0x08)))
	{
		shswic_detect_state = SHSWIC_STATE_HEADSET;
		
		if ((shswic_last_detect_state_91401 != SHSWIC_STATE_HEADSET_SW) && (shswic_last_detect_state_91401 != SHSWIC_STATE_HEADSET))
		{
			sw_ctl = SHSWIC_NONE_PASS;
			shswic_i2c_write(&sw_ctl, 1, SHSWIC_REG_MUXSW_CONTROL);

			wake_lock(&shswic_sleep_wake_lock_91401);
			usleep(100000);
			wake_unlock(&shswic_sleep_wake_lock_91401);

			sw_ctl = SHSWIC_HEADSET_PASS;
			shswic_i2c_write(&sw_ctl, 1, SHSWIC_REG_MUXSW_CONTROL);
			
		}
#ifdef CONFIG_HEADSET_BUTTON_SWIC
		else
		{
			SHSWIC_DEBUG_LOG_L("shswic_detect_get button 1[int_status:0x%x]", int_status);
			shswic_send_headset_sw_button(int_status);
		}
#endif /* CONFIG_HEADSET_BUTTON_SWIC */

		cnt = 0;
		SHSWIC_DEBUG_LOG_L("shswic_detect_get:HEADSET");
		return SHSWIC_ID_HEADSET;
	}
	else if (((id_status   & SHSWIC_ID_STA_MASK_91401)  == 0x02)
	      || (((int_status & SHSWIC_INT_STA_MASK_91401) == 0x20)
	       && (((id_status & SHSWIC_ID_STA_MASK_91401)  == 0x0C)
	        || ((id_status & SHSWIC_ID_STA_MASK_91401)  == 0x08))))
	{
		shswic_detect_state = SHSWIC_STATE_HEADSET_SW;
#ifdef CONFIG_HEADSET_BUTTON_SWIC
		shswic_switch_earphone = true;
#endif /* CONFIG_HEADSET_BUTTON_SWIC */

		if ((shswic_last_detect_state_91401 != SHSWIC_STATE_HEADSET_SW) && (shswic_last_detect_state_91401 != SHSWIC_STATE_HEADSET))
		{
			sw_ctl = SHSWIC_NONE_PASS;
			shswic_i2c_write(&sw_ctl, 1, SHSWIC_REG_MUXSW_CONTROL);
			
			wake_lock(&shswic_sleep_wake_lock_91401);
			usleep(100000);
			wake_unlock(&shswic_sleep_wake_lock_91401);

			sw_ctl = SHSWIC_HEADSET_PASS;
			shswic_i2c_write(&sw_ctl, 1, SHSWIC_REG_MUXSW_CONTROL);
			
		}
#ifdef CONFIG_HEADSET_BUTTON_SWIC
		else
		{
			SHSWIC_DEBUG_LOG_L("shswic_detect_get button 2[int_status:0x%x]", int_status);
			shswic_send_headset_sw_button(int_status);
		}
#endif /* CONFIG_HEADSET_BUTTON_SWIC */

		cnt = 0;
		SHSWIC_DEBUG_LOG_L("shswic_detect_get:HEADSET_SW");
		return SHSWIC_ID_HEADSET_SW;
	}
#ifdef CONFIG_HOST_SWIC
	else if ((id_status & SHSWIC_ID_HOST_MASK_91401) == 0x10)
	{
		shswic_detect_state = SHSWIC_STATE_USB_HOST_CABLE;
#ifdef CONFIG_HEADSET_BUTTON_SWIC
		shswic_switch_earphone = false;
#endif /* CONFIG_HEADSET_BUTTON_SWIC */
		SHSWIC_DEBUG_LOG_L("shswic_detect_get:USB_HOST_CABLE");
		return SHSWIC_ID_USB_HOST_CABLE;
	}
#endif /* CONFIG_HOST_SWIC */
	else if ((status & SHSWIC_STATUS_MASK_91401) == 0x20)
	{
		shswic_detect_state = SHSWIC_STATE_USB_CABLE;
#ifdef CONFIG_HEADSET_BUTTON_SWIC
		shswic_switch_earphone = false;
#endif /* CONFIG_HEADSET_BUTTON_SWIC */
		cnt = 0;
		SHSWIC_DEBUG_LOG_L("shswic_detect_get:USB_CABLE");
		return SHSWIC_ID_USB_CABLE;
	}
	else if ((status & SHSWIC_STATUS_MASK_91401) == 0x60)
	{
		shswic_detect_state = SHSWIC_STATE_AC_ADAPTER;
#ifdef CONFIG_HEADSET_BUTTON_SWIC
		shswic_switch_earphone = false;
#endif /* CONFIG_HEADSET_BUTTON_SWIC */
		cnt = 0;
		SHSWIC_DEBUG_LOG_L("shswic_detect_get:AC_ADAPTER");
		return SHSWIC_ID_AC_ADAPTER;
	}
	else if ((status & SHSWIC_STATUS_MASK_91401) == 0x00)
	{
		shswic_detect_state = SHSWIC_STATE_NONE;
#ifdef CONFIG_HEADSET_BUTTON_SWIC
		shswic_switch_earphone = false;
#endif /* CONFIG_HEADSET_BUTTON_SWIC */
		shswic_i2c_write(&sw_ctl, 1, SHSWIC_REG_MUXSW_CONTROL);

		if ((int_status & SHSWIC_INT_STA_MASK_91401) == 0x10)
		{
			if ((cnt < SHSWIC_DETECT_RETRY_CNT)
			 && (((shswic_last_status_91401     & SHSWIC_STATUS_MASK_91401)  == 0x00)
			  && ((shswic_last_int_status_91401 & SHSWIC_INT_STA_MASK_91401) == 0x10)))
			{
				SHSWIC_DEBUG_LOG_L("shswic_detect_get retry detect");
				cnt++;
				shswic_timer_flag_91401 = true;
				shswic_i2c_write(&dcd_en, 1, SHSWIC_REG_UCDCNT);
				shswic_send_timer_signal(SHSWIC_DETECT_RETRY_SIG, SHSWIC_DETECT_RETRY_TIME);
			}
		}
		else
		{
			cnt = 0;
		}
		if (((int_status & SHSWIC_INT_STA_MASK_91401) == 0x10)
		 || ((int_status & SHSWIC_INT_STA_MASK_91401) == 0x00))
		{
			if ((((shswic_last_int_status_91401 & SHSWIC_INT_STA_MASK_91401) == 0x10) || ((shswic_last_int_status_91401 & SHSWIC_INT_STA_MASK_91401) == 0x00)) &&
			  (shswic_last_detect_state_91401 == SHSWIC_STATE_NONE))
			{
				wake_lock(&shswic_sleep_wake_lock_91401);
				usleep(SHSWIC_DETECT_DELAY * 1000);
				wake_unlock(&shswic_sleep_wake_lock_91401);
			}
		}
		SHSWIC_DEBUG_LOG_L("shswic_detect_get:NONE");
		return SHSWIC_ID_NONE;
	}
	else
	{
		cnt = 0;
		SHSWIC_DEBUG_LOG_L("shswic_detect_get:%d", shswic_detect_port);
		return shswic_detect_port;
	}
}

void shswic_detect_init_91401(void)
{
	int ret = 0;
	uint8_t id_status = 0;
	uint8_t status = 0;
	uint8_t int_status = 0;
#ifndef CONFIG_SII8334_MHL_TX
	uint8_t detect_port = SHSWIC_ID_NONE;
#endif /* CONFIG_SII8334_MHL_TX */

	SHSWIC_DEBUG_LOG_L("shswic_detect_init Start");

	wake_lock_init(&shswic_sleep_wake_lock_91401, WAKE_LOCK_SUSPEND, "shswic_sleep_lock_91401");

#ifdef CONFIG_SII8334_MHL_TX
	shswic_read_data.shswic_mhl_result = SHMHL_DEVICE_MAX;
#endif /* CONFIG_SII8334_MHL_TX */

	gpio_tlmm_config(GPIO_CFG(SHSWIC_GPIO_INT, 0, GPIO_CFG_INPUT, GPIO_CFG_PULL_UP, GPIO_CFG_2MA), GPIO_CFG_ENABLE);
	ret = request_irq(MSM_GPIO_TO_INT(SHSWIC_GPIO_INT), shswic_detect_isr, IRQF_TRIGGER_FALLING, "shswic_irq", 0);
	if (ret != 0)
	{
		SHSWIC_DEBUG_LOG_H("Error USB-Int request_irq:Ret %d", ret);
	}
	else
	{
		shswic_interrupt_init_state_91401 = true;
		SHSWIC_DEBUG_LOG_L("Success USB-Int request_irq");
	}
#ifdef CONFIG_SII8334_MHL_TX
	shswic_status_read(&id_status, &status, &int_status);

	if ((id_status & SHSWIC_ID_DETECT_MASK_91401) == 0x10)
	{
		if (shswic_regulator_power(true) != SHSWIC_REGULATOR_SUCCESS)
		{
			SHSWIC_DEBUG_LOG_H("shswic_regulator_power Error");
		}

		shswic_i2c_write(&shswic_adc_disable_91401, 1, SHSWIC_REG_IDDET);

		wake_lock(&shswic_sleep_wake_lock_91401);
		usleep(30000);
		wake_unlock(&shswic_sleep_wake_lock_91401);

		shswic_i2c_write(&shswic_adc_enable_91401, 1, SHSWIC_REG_IDDET);
		shswic_id_detect_retry_flg_91401 = true;
	}
#else /* CONFIG_SII8334_MHL_TX */
	shswic_status_read(&id_status, &status, &int_status);
	shswic_status_read(&id_status, &status, &int_status);

	shswic_detect_get_91401(id_status, status, int_status);

	shswic_state_proc[shswic_last_detect_state_91401](detect_port);

	shswic_last_detect_state_91401 = shswic_detect_state;
	shswic_last_status_91401 = status; 
	shswic_last_int_status_91401 = int_status;
#endif /* CONFIG_SII8334_MHL_TX */
#ifdef CONFIG_CRADLE_SWIC
	gpio_tlmm_config(GPIO_CFG(SHSWIC_GPIO_VCDET, 0, GPIO_CFG_INPUT, GPIO_CFG_PULL_UP, GPIO_CFG_2MA), GPIO_CFG_ENABLE);
	ret = request_irq(MSM_GPIO_TO_INT(SHSWIC_GPIO_VCDET), shswic_cradle_isr, IRQF_TRIGGER_RISING | IRQF_TRIGGER_FALLING, "shswic_irq", 0);
	if (ret != 0)
	{
		SHSWIC_DEBUG_LOG_H("Error CRADLE-Int request_irq:Ret %d", ret);
	}
	else
	{
		shswic_cradle_interrupt_init_state_91401 = true;
		SHSWIC_DEBUG_LOG_L("Success CRADLE-Int request_irq");
	}
	shswic_state_cradle_proc();
#endif /* CONFIG_CRADLE_SWIC */

	return;
}

void shswic_detect_retry_sig_handler_91401(void)
{
	uint8_t dcd_en = 0x41;
	
	if (shswic_timer_flag_91401 == true)
	{
		shswic_i2c_write(&dcd_en, 1, SHSWIC_REG_UCDCNT);
	}
}

#ifdef CONFIG_SII8334_MHL_TX
void shswic_detect_mhl_proc_91401(void)
{
	uint8_t detect_port = SHSWIC_ID_NONE;
	uint8_t sw_ctl = 0x00;

	SHSWIC_DEBUG_LOG_L("shswic_cb_detect_mhl_proc Called [device=%d]", shswic_read_data.shswic_mhl_result);

	shswic_detect_id = (shswic_read_data.shswic_id_status & SHSWIC_ID_STA_MASK_91401);

	switch(shswic_read_data.shswic_mhl_result)
	{
		case SHMHL_DEVICE_MHL:
			detect_port = SHSWIC_ID_MHL;
			shswic_detect_state = SHSWIC_STATE_MHL;
			SHSWIC_DEBUG_LOG_L("shswic_cb_detect_mhl_proc:MHL");
			break;
		case SHMHL_DEVICE_USB_B:
			detect_port = shswic_detect_get_91401(shswic_read_data.shswic_id_status, shswic_read_data.shswic_status, shswic_read_data.shswic_int_status);
			shswic_i2c_write(&sw_ctl, 1, SHSWIC_REG_MUXSW_CONTROL);
			SHSWIC_DEBUG_LOG_L("shswic_cb_detect_mhl_proc:USB_HOST_CABLE");
			break;
		case SHMHL_DEVICE_UNKNOWN:
		case SHMHL_DEVICE_ACA_MCPC_CHG:
		case SHMHL_DEVICE_USB_A_CHG:
		case SHMHL_DEVICE_STANDBY:
		case SHMHL_DEVICE_USB_B_ACA:
		case SHMHL_DEVICE_MCPC_ACA_OPN:
		case SHMHL_DEVICE_NONE:
			detect_port = shswic_detect_port;
			shswic_i2c_write(&sw_ctl, 1, SHSWIC_REG_MUXSW_CONTROL);
			SHSWIC_DEBUG_LOG_L("shswic_cb_detect_mhl_proc:OTHER");
			break;
		case SHMHL_DEVICE_MAX:
			/* request MHL chipset check, now. */
			break;
		default:
			shswic_i2c_write(&sw_ctl, 1, SHSWIC_REG_MUXSW_CONTROL);
			break;
	}

	if ((detect_port != SHSWIC_ID_USB_HOST_CABLE)
#ifdef CONFIG_HEADSET_BUTTON_SWIC
	&&  (detect_port != SHSWIC_ID_HEADSET)
	&&  (detect_port != SHSWIC_ID_HEADSET_SW)
#endif /* CONFIG_HEADSET_BUTTON_SWIC */
	)
	{
		if (shswic_regulator_power(false) != SHSWIC_REGULATOR_SUCCESS)
		{
			SHSWIC_DEBUG_LOG_H("shswic_regulator_power Error");
		}
	}

	shswic_state_proc[shswic_last_detect_state_91401](detect_port);
	shswic_last_detect_state_91401 = shswic_detect_state;
	shswic_last_status_91401 = shswic_read_data.shswic_status; 
	shswic_last_int_status_91401 = shswic_read_data.shswic_int_status;
}

static void shswic_call_mhl_func_91401(uint8_t id_status, uint8_t status, uint8_t int_status, shmhl_vbus_status_t vbus_status)
{
	uint8_t sw_ctl = SHSWIC_MHL_PASS_91401;
	/* idrdet flg 1 : id line low */
	/*            0 : id line hi  */
	bool id_is_hi = ((id_status & SHSWIC_ID_STA_IDRDET_MASK_91401) == 0x00);

	shswic_read_data.shswic_id_status = id_status;
	shswic_read_data.shswic_status = status;
	shswic_read_data.shswic_int_status = int_status;

	shswic_i2c_write(&sw_ctl, 1, SHSWIC_REG_MUXSW_CONTROL);
	shmhl_detect_cb_regist(shswic_detect_cb_mhl, NULL);
	shmhl_notify_id_vbus_state(id_is_hi, vbus_status);
}
#endif /* CONFIG_SII8334_MHL_TX */

shswic_result_t shswic_write_vbsw_reg_91401(uint8_t vbsw_ctl)
{
	shswic_result_t result = SHSWIC_FAILURE;

	result = shswic_i2c_write(&vbsw_ctl, 1, SHSWIC_REG_VBSW_CONTROL);

	return result;
}

shswic_result_t shswic_read_vbsw_reg_91401(uint8_t* vbsw_ctl)
{
	if (vbsw_ctl == NULL)
	{
		SHSWIC_DEBUG_LOG_H("shswic_read_vbsw_reg_91411 SHSWIC_PARAM_ERROR");
		return SHSWIC_PARAM_ERROR;
	}
	return shswic_i2c_read(vbsw_ctl, 1, SHSWIC_REG_VBSW_CONTROL);
}

void shswic_pre_detect_cb_call_91401(uint8_t detect)
{
#ifdef CONFIG_HOST_SWIC
	if(detect == SHSWIC_ID_USB_HOST_CABLE)
	{
#ifdef CONFIG_SII8334_MHL_TX
		if (shswic_regulator_power(true) != SHSWIC_REGULATOR_SUCCESS)
		{
			SHSWIC_DEBUG_LOG_H("shswic_regulator_power Error");
		}
#endif /* CONFIG_SII8334_MHL_TX */
		shswic_write_vbsw_reg(SHSWIC_VBSW_OFF);
	}
	else
	{
#ifdef CONFIG_SII8334_MHL_TX
#ifdef CONFIG_HEADSET_BUTTON_SWIC
		if((detect != SHSWIC_ID_HEADSET)
		&& (detect != SHSWIC_ID_HEADSET_SW))
#endif /* CONFIG_HEADSET_BUTTON_SWIC */
		{
			if (shswic_regulator_power(false) != SHSWIC_REGULATOR_SUCCESS)
			{
				SHSWIC_DEBUG_LOG_H("shswic_regulator_power Error");
			}
		}
#endif /* CONFIG_SII8334_MHL_TX */
		shswic_write_vbsw_reg(SHSWIC_VBSW_AUTO);
	}
#endif /* CONFIG_HOST_SWIC */
}

void shswic_pre_do_exit_91401(void)
{
	if(shswic_interrupt_init_state_91401)
	{
		shswic_interrupt_init_state_91401 = false;
		disable_irq(MSM_GPIO_TO_INT(SHSWIC_GPIO_INT));
		free_irq(MSM_GPIO_TO_INT(SHSWIC_GPIO_INT), 0);
	}

#ifdef CONFIG_CRADLE_SWIC
	if(shswic_cradle_interrupt_init_state_91401)
	{
		shswic_cradle_interrupt_init_state_91401 = false;
		disable_irq(MSM_GPIO_TO_INT(SHSWIC_GPIO_VCDET));
		free_irq(MSM_GPIO_TO_INT(SHSWIC_GPIO_VCDET), 0);
	}
#endif /* CONFIG_CRADLE_SWIC */
}

#ifdef CONFIG_CRADLE_SWIC
int shswic_cradle_state_91401(void)
{
	if(!shswic_cradle_interrupt_init_state_91401)
	{
		return SHSWIC_ID_CRADLE_UNKNOWN;
	}
	
	if(gpio_get_value(SHSWIC_GPIO_VCDET) != SHSWIC_GPIO_HIGH)
	{
		return SHSWIC_ID_CRADLE;
	}

	return SHSWIC_ID_CRADLE_NONE;
}
#endif /* CONFIG_CRADLE_SWIC */

bool shswic_is_push_headset_button_91401(uint8_t int_status)
{
	return ((int_status & SHSWIC_INT_STA_MASK_91401) == 0x20);
}

