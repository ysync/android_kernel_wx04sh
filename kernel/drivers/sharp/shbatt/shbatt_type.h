/*
 * Copyright (C) 2011 SHARP CORPORATION All rights reserved.
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

#ifndef SHBATT_TYPE_H
#define SHBATT_TYPE_H

/*+-----------------------------------------------------------------------------+*/
/*| @ INCLUDE FILE :															|*/
/*+-----------------------------------------------------------------------------+*/

/*+-----------------------------------------------------------------------------+*/
/*| @ VALUE DEFINE DECLARE :													|*/
/*+-----------------------------------------------------------------------------+*/

#include <sharp/shbatt_kerl.h>

#define SHBATT_IOCTL_MAGIC 'b'

#define SHBATT_DRV_IOCTL_CMD_GET_CHARGER_CABLE_STATUS				_IOR( SHBATT_IOCTL_MAGIC,  0, shbatt_cable_status_t*)
#define SHBATT_DRV_IOCTL_CMD_GET_BATTERY_CHARGING_STATUS			_IOR( SHBATT_IOCTL_MAGIC,  1, shbatt_chg_status_t*)
#define SHBATT_DRV_IOCTL_CMD_GET_BATTERY_CAPACITY					_IOR( SHBATT_IOCTL_MAGIC,  2, int*)
#define SHBATT_DRV_IOCTL_CMD_GET_BATTERY_VOLTAGE					_IOR( SHBATT_IOCTL_MAGIC,  3, int*)
#define SHBATT_DRV_IOCTL_CMD_GET_BATTERY_PRESENT					_IOR( SHBATT_IOCTL_MAGIC,  4, shbatt_present_t*)
#define SHBATT_DRV_IOCTL_CMD_GET_BATTERY_HEALTH 					_IOR( SHBATT_IOCTL_MAGIC,  5, shbatt_present_t*)
#define SHBATT_DRV_IOCTL_CMD_GET_BATTERY_TECHNOLOGY 				_IOR( SHBATT_IOCTL_MAGIC,  6, shbatt_technology_t*)
#define SHBATT_DRV_IOCTL_CMD_GET_BATTERY_CURRENT					_IOR( SHBATT_IOCTL_MAGIC,  7, int*)
#define SHBATT_DRV_IOCTL_CMD_READ_ADC_CHANNEL						_IOWR(SHBATT_IOCTL_MAGIC,  8, shbatt_adc_t*)
#define SHBATT_DRV_IOCTL_CMD_SET_VBATT_CALIBRATION_DATA 			_IOW( SHBATT_IOCTL_MAGIC,  9, shbatt_vbatt_cal_t)
#define SHBATT_DRV_IOCTL_CMD_REFRESH_VBATT_CALIBRATION_DATA 		_IO(  SHBATT_IOCTL_MAGIC, 10)
#define SHBATT_DRV_IOCTL_CMD_SET_PMIC_CHARGER_TRANSISTOR_SWITCH		_IOW( SHBATT_IOCTL_MAGIC, 11, int)
#define SHBATT_DRV_IOCTL_CMD_SET_PMIC_BATTERY_TRANSISTOR_SWITCH		_IOW( SHBATT_IOCTL_MAGIC, 12, int)
#define SHBATT_DRV_IOCTL_CMD_SET_PMIC_VMAXSEL						_IOW( SHBATT_IOCTL_MAGIC, 13, int)
#define SHBATT_DRV_IOCTL_CMD_SET_PMIC_IMAXSEL						_IOW( SHBATT_IOCTL_MAGIC, 14, int)
#define SHBATT_DRV_IOCTL_CMD_SET_PMIC_VTRICKLE						_IOW( SHBATT_IOCTL_MAGIC, 15, int)
#define SHBATT_DRV_IOCTL_CMD_SET_PMIC_ITRICKLE						_IOW( SHBATT_IOCTL_MAGIC, 16, int)
#define SHBATT_DRV_IOCTL_CMD_CHECK_STARTUP_VOLTAGE					_IOR( SHBATT_IOCTL_MAGIC, 17, int*)
#define SHBATT_DRV_IOCTL_CMD_CHECK_STARTUP_BATTERY_PRESENT_CHECK	_IO(  SHBATT_IOCTL_MAGIC, 18)
#define SHBATT_DRV_IOCTL_CMD_GET_BATTERY_TEMPERATURE				_IOR( SHBATT_IOCTL_MAGIC, 19, int*)
#define SHBATT_DRV_IOCTL_CMD_GET_CHARGER_TEMPERATURE				_IOR( SHBATT_IOCTL_MAGIC, 20, int*)
#define SHBATT_DRV_IOCTL_CMD_GET_CAMERA_TEMPERATURE 				_IOR( SHBATT_IOCTL_MAGIC, 21, int*)
#define SHBATT_DRV_IOCTL_CMD_GET_PA_TEMPERATURE 					_IOR( SHBATT_IOCTL_MAGIC, 22, int*)
#define SHBATT_DRV_IOCTL_CMD_GET_XO_TEMPERATURE 					_IOR( SHBATT_IOCTL_MAGIC, 23, int*)
#define SHBATT_DRV_IOCTL_CMD_GET_PMIC_TEMPERATURE					_IOR( SHBATT_IOCTL_MAGIC, 24, int*)
#define SHBATT_DRV_IOCTL_CMD_SET_SMBC_CHARGER						_IOW( SHBATT_IOCTL_MAGIC, 25, shbatt_smbc_chg_t)
#define SHBATT_DRV_IOCTL_CMD_SET_MEAS_FREQ							_IOW( SHBATT_IOCTL_MAGIC, 26, int)
#define SHBATT_DRV_IOCTL_CMD_READ_OCV_FOR_RBATT						_IOR( SHBATT_IOCTL_MAGIC, 27, uint*)
#define SHBATT_DRV_IOCTL_CMD_READ_VSENSE_FOR_RBATT					_IOR( SHBATT_IOCTL_MAGIC, 28, shbatt_adc_conv_uint_t*)
#define SHBATT_DRV_IOCTL_CMD_READ_VBATT_FOR_RBATT					_IOR( SHBATT_IOCTL_MAGIC, 29, uint*)
#define SHBATT_DRV_IOCTL_CMD_READ_CC								_IOR( SHBATT_IOCTL_MAGIC, 30, shbatt_adc_conv_offset_t*)
#define SHBATT_DRV_IOCTL_CMD_READ_LAST_GOOD_OCV						_IOR( SHBATT_IOCTL_MAGIC, 31, uint*)
#define SHBATT_DRV_IOCTL_CMD_READ_VSENSE_AVG						_IOR( SHBATT_IOCTL_MAGIC, 32, shbatt_adc_conv_int_t*)
#define SHBATT_DRV_IOCTL_CMD_READ_VBATT_AVG							_IOR( SHBATT_IOCTL_MAGIC, 33, int*)
#define SHBATT_DRV_IOCTL_CMD_AUTO_ENABLE							_IOW( SHBATT_IOCTL_MAGIC, 34, int)
#define SHBATT_DRV_IOCTL_CMD_RECALIB_ADC_DEVICE						_IO(  SHBATT_IOCTL_MAGIC, 35)
#define SHBATT_DRV_IOCTL_CMD_GET_CALIBRATE_BATTERY_VOLTAGE			_IOR( SHBATT_IOCTL_MAGIC, 36, int*)
#define SHBATT_DRV_IOCTL_CMD_GET_WIRELESS_STATUS					_IOR( SHBATT_IOCTL_MAGIC, 37, int*)
#define SHBATT_DRV_IOCTL_CMD_SET_WIRELESS_SWITCH					_IOW( SHBATT_IOCTL_MAGIC, 38, int)
#define SHBATT_DRV_IOCTL_CMD_SET_VSENSE_AVG_CALIBRATION_DATA 		_IOW( SHBATT_IOCTL_MAGIC, 39, shbatt_vsense_avg_cal_t)
#define SHBATT_DRV_IOCTL_CMD_REFRESH_VSENSE_AVG_CALIBRATION_DATA 	_IO(  SHBATT_IOCTL_MAGIC, 40)
#define SHBATT_DRV_IOCTL_CMD_GET_BACKLIGHT_TEMPERATURE 				_IOR( SHBATT_IOCTL_MAGIC, 41, int*)
#define SHBATT_DRV_IOCTL_CMD_GET_BMS_CONTROL 						_IOR( SHBATT_IOCTL_MAGIC, 42, int*)

#define SHBATT_PS_VALUE_FULL_VOLTAGE			4310
#define SHBATT_PS_VALUE_EMPTYE_VOLTAGE			3400
#define SHBATT_PS_VALUE_FULL_CAPACITY			1800
/* TODO New API add point */

/*+-----------------------------------------------------------------------------+*/
/*| @ ENUMERATION DECLARE : 													|*/
/*+-----------------------------------------------------------------------------+*/
/*attributes.*/
/*cable(ac,usb)status attributes.*/
enum
{
	SHBATT_PS_PROPERTY_ONLINE,
};
/*battery attributes.*/
enum
{
	SHBATT_PS_PROPERTY_STATUS,
	SHBATT_PS_PROPERTY_HEALTH,
	SHBATT_PS_PROPERTY_PRESENT,
	SHBATT_PS_PROPERTY_CAPACITY,
	SHBATT_PS_PROPERTY_BATT_VOL,
	SHBATT_PS_PROPERTY_BATT_TEMP,
	SHBATT_PS_PROPERTY_TECHNOLOGY,
	SHBATT_PS_PROPERTY_SAFETY,
	SHBATT_PS_PROPERTY_CAMERA_TEMP,
	SHBATT_PS_PROPERTY_TERMINAL_TEMP,
	SHBATT_PS_PROPERTY_MODEM_TEMP,
	SHBATT_PS_PROPERTY_CURRENT_NOW,
	SHBATT_PS_PROPERTY_VOLTAGE_NOW,
	SHBATT_PS_PROPERTY_VOLTAGE_MAX_DESIGN,
	SHBATT_PS_PROPERTY_VOLTAGE_MIN_DESIGN,
	SHBATT_PS_PROPERTY_ENERGY_FULL,
};

/*fuelgauge attributes.*/
enum
{
	SHBATT_PS_PROPERTY_CURRENT,
	SHBATT_PS_PROPERTY_VOLTAGE,
	SHBATT_PS_PROPERTY_DEVICE_ID,
	SHBATT_PS_PROPERTY_MODE,
	SHBATT_PS_PROPERTY_ACCUMULATE_CURRENT,
	SHBATT_PS_PROPERTY_FGIC_TEMP,
	SHBATT_PS_PROPERTY_CURRENT_AD,
};
/*pmic attributes.*/
enum
{
	SHBATT_PS_PROPERTY_CHARGER_TRANSISTOR_SWITCH,
	SHBATT_PS_PROPERTY_VMAXSEL,
	SHBATT_PS_PROPERTY_IMAXSEL,
};
/*adc attributes.*/
enum
{
	SHBATT_PS_PROPERTY_GPADC_IN0,
	SHBATT_PS_PROPERTY_GPADC_IN1,
	SHBATT_PS_PROPERTY_GPADC_IN2,
	SHBATT_PS_PROPERTY_GPADC_IN3,
	SHBATT_PS_PROPERTY_GPADC_IN4,
	SHBATT_PS_PROPERTY_GPADC_IN5,
	SHBATT_PS_PROPERTY_GPADC_IN6,
	SHBATT_PS_PROPERTY_VBAT,
	SHBATT_PS_PROPERTY_VBKP,
	SHBATT_PS_PROPERTY_VAC,
	SHBATT_PS_PROPERTY_VBUS,
	SHBATT_PS_PROPERTY_ICHG,
	SHBATT_PS_PROPERTY_HOTDIE1,
	SHBATT_PS_PROPERTY_HOTDIE2,
	SHBATT_PS_PROPERTY_ID,
	SHBATT_PS_PROPERTY_TESTV,
	SHBATT_PS_PROPERTY_CHRG_LED_TEST,
	SHBATT_PS_PROPERTY_GPADC_IN0_BUF,
	SHBATT_PS_PROPERTY_GPADC_IN1_BUF,
	SHBATT_PS_PROPERTY_GPADC_IN2_BUF,
	SHBATT_PS_PROPERTY_GPADC_IN3_BUF,
	SHBATT_PS_PROPERTY_GPADC_IN4_BUF,
	SHBATT_PS_PROPERTY_GPADC_IN5_BUF,
	SHBATT_PS_PROPERTY_GPADC_IN6_BUF,
	SHBATT_PS_PROPERTY_VBAT_BUF,
	SHBATT_PS_PROPERTY_VBKP_BUF,
	SHBATT_PS_PROPERTY_VAC_BUF,
	SHBATT_PS_PROPERTY_VBUS_BUF,
	SHBATT_PS_PROPERTY_ICHG_BUF,
	SHBATT_PS_PROPERTY_HOTDIE1_BUF,
	SHBATT_PS_PROPERTY_HOTDIE2_BUF,
	SHBATT_PS_PROPERTY_ID_BUF,
	SHBATT_PS_PROPERTY_TESTV_BUF,
	SHBATT_PS_PROPERTY_CHRG_LED_TEST_BUF,
	SHBATT_PS_PROPERTY_VBATT_CALIBRATION,
	SHBATT_PS_PROPERTY_CPU_TEMP,
};
typedef enum shbatt_api_to_tsk_command_tag
{
	SHBATT_TASK_CMD_INVALID,
	SHBATT_TASK_CMD_GET_CHARGER_CABLE_STATUS,
	SHBATT_TASK_CMD_GET_BATTERY_CHARGING_STATUS,
	SHBATT_TASK_CMD_GET_BATTERY_CAPACITY,
	SHBATT_TASK_CMD_GET_BATTERY_VOLTAGE,
	SHBATT_TASK_CMD_GET_BATTERY_HEALTH,
	SHBATT_TASK_CMD_GET_BATTERY_TECHNOLOGY,
	SHBATT_TASK_CMD_GET_BATTERY_CURRENT,
	SHBATT_TASK_CMD_READ_ADC_CHANNEL,
	SHBATT_TASK_CMD_SET_VBATT_CALIBRATION_DATA,
	SHBATT_TASK_CMD_REFRESH_VBATT_CALIBRATION_DATA,
	SHBATT_TASK_CMD_SET_PMIC_CHARGER_TRANSISTOR_SWITCH,
	SHBATT_TASK_CMD_SET_PMIC_BATTERY_TRANSISTOR_SWITCH,
	SHBATT_TASK_CMD_SET_PMIC_VMAXSEL,
	SHBATT_TASK_CMD_SET_PMIC_IMAXSEL,
	SHBATT_TASK_CMD_SET_PMIC_VTRICKLE,
	SHBATT_TASK_CMD_SET_PMIC_ITRICKLE,
	SHBATT_TASK_CMD_CHECK_STARTUP_VOLTAGE,
	SHBATT_TASK_CMD_CHECK_STARTUP_BATTERY_PRESENT_CHECK,
	SHBATT_TASK_CMD_GET_BATTERY_TEMPERATURE,
	SHBATT_TASK_CMD_GET_CHARGER_TEMPERATURE,
	SHBATT_TASK_CMD_GET_CAMERA_TEMPERATURE,
	SHBATT_TASK_CMD_GET_PA_TEMPERATURE,
	SHBATT_TASK_CMD_GET_XO_TEMPERATURE,
	SHBATT_TASK_CMD_GET_PMIC_TEMPERATURE,
	SHBATT_TASK_CMD_SET_SMBC_CHARGER,
	SHBATT_TASK_CMD_SET_MEAS_FREQ,
	SHBATT_TASK_CMD_READ_OCV_FOR_RBATT,
	SHBATT_TASK_CMD_READ_VSENSE_FOR_RBATT,
	SHBATT_TASK_CMD_READ_VBATT_FOR_RBATT,
	SHBATT_TASK_CMD_READ_CC,
	SHBATT_TASK_CMD_READ_LAST_GOOD_OCV,
	SHBATT_TASK_CMD_READ_VSENSE_AVG,
	SHBATT_TASK_CMD_READ_VBATT_AVG,
	SHBATT_TASK_CMD_AUTO_ENABLE,
	SHBATT_TASK_CMD_RECALIB_ADC_DEVICE,
	SHBATT_TASK_CMD_GET_TERMINAL_TEMPERATURE,
	SHBATT_TASK_CMD_GET_MODEM_TEMPERATURE,
	SHBATT_TASK_CMD_GET_FUELGAUGE_VOLTAGE,
	SHBATT_TASK_CMD_GET_FUELGAUGE_DEVICE_ID,
	SHBATT_TASK_CMD_SET_FUELGAUGE_MODE,
	SHBATT_TASK_CMD_GET_FUELGAUGE_ACCUMULATE_CURRENT,
	SHBATT_TASK_CMD_CLR_FUELGAUGE_ACCUMULATE_CURRENT,
	SHBATT_TASK_CMD_GET_FUELGAUGE_TEMPERATURE,
	SHBATT_TASK_CMD_GET_FUELGAUGE_CURRENT_AD,
	SHBATT_TASK_CMD_READ_ADC_CHANNEL_BUFFERED,
	SHBATT_TASK_CMD_GET_BATTERY_LOG_INFO,
	SHBATT_TASK_CMD_NOTIFY_CHARGER_CABLE_STATUS,
	SHBATT_TASK_CMD_NOTIFY_BATTERY_CHARGING_STATUS,
	SHBATT_TASK_CMD_NOTIFY_BATTERY_CAPACITY,
	SHBATT_TASK_CMD_NOTIFY_CHARGING_STATE_MACHINE_ENABLE,
	SHBATT_TASK_CMD_GET_BATTERY_SAFETY,
	SHBATT_TASK_CMD_INITIALIZE,
	SHBATT_TASK_CMD_EXEC_LOW_BATTERY_CHECK_SEQUENCE,
	SHBATT_TASK_CMD_POST_BATTERY_LOG_INFO,
	SHBATT_TASK_CMD_EXEC_BATTERY_PRESENT_CHECK_SEQUENCE,
	SHBATT_TASK_CMD_EXEC_OVERCURR_CHECK_SEQUENCE,
	SHBATT_TASK_CMD_EXEC_FUELGAUGE_SOC_POLL_SEQUENCE,
	SHBATT_TASK_CMD_GET_CALIBRATE_BATTERY_VOLTAGE,
	SHBATT_TASK_CMD_GET_WIRELESS_STATUS,
	SHBATT_TASK_CMD_SET_WIRELESS_SWITCH,
	SHBATT_TASK_CMD_SET_VSENSE_AVG_CALIBRATION_DATA,
	SHBATT_TASK_CMD_REFRESH_VSENSE_AVG_CALIBRATION_DATA,
	SHBATT_TASK_CMD_GET_BATTERY_PRESENT,
	SHBATT_TASK_CMD_NOTIFY_WIRELESS_CHARGER_STATE_CHANGED,
	SHBATT_TASK_CMD_SET_LOG_ENABLE,
	SHBATT_TASK_CMD_GET_REAL_BATTERY_CAPACITY,
	SHBATT_TASK_CMD_GET_BACKLIGHT_TEMPERATURE,
	SHBATT_TASK_CMD_NOTIFY_CHARGE_SWITCH_STATUS,
	SHBATT_TASK_CMD_GET_DEPLETED_CAPACITY,
	SHBATT_TASK_CMD_SET_DEPLETED_BATTERY_FLG,
	SHBATT_TASK_CMD_GET_DEPLETED_BATTERY_FLG,
	SHBATT_TASK_CMD_SET_BATTERY_HEALTH,
	SHBATT_TASK_CMD_SET_DEPLETED_CALC_ENABLE,
/* TODO New API add point */
	NUM_SHBATT_TASK_CMD,
} shbatt_api_to_tsk_command_t;

typedef enum
{
	SHBATT_CMD_INVALID,
	SHBATT_CMD_GET_CHARGER_CABLE_STATUS,
	SHBATT_CMD_GET_BATTERY_CHARGING_STATUS,
	SHBATT_CMD_GET_BATTERY_HEALTH,
	SHBATT_CMD_GET_BATTERY_PRESENT,
	SHBATT_CMD_GET_BATTERY_CAPACITY,
	SHBATT_CMD_GET_BATTERY_VOLTAGE,
	SHBATT_CMD_GET_BATTERY_TEMPERATURE,
	SHBATT_CMD_GET_BATTERY_TECHNOLOGY,
	SHBATT_CMD_GET_BATTERY_SAFETY,
	SHBATT_CMD_GET_CAMERA_TEMPERATURE,
	SHBATT_CMD_GET_TERMINAL_TEMPERATURE,
	SHBATT_CMD_GET_MODEM_TEMPERATURE,
	SHBATT_CMD_GET_FUELGAUGE_VOLTAGE,
	SHBATT_CMD_GET_FUELGAUGE_DEVICE_ID,
	SHBATT_CMD_SET_FUELGAUGE_MODE,
	SHBATT_CMD_GET_FUELGAUGE_ACCUMULATE_CURRENT,
	SHBATT_CMD_CLR_FUELGAUGE_ACCUMULATE_CURRENT,
	SHBATT_CMD_GET_FUELGAUGE_TEMPERATURE,
	SHBATT_CMD_GET_FUELGAUGE_CURRENT_AD,
	SHBATT_CMD_SET_PMIC_VMAXSEL,
	SHBATT_CMD_SET_PMIC_IMAXSEL,
	SHBATT_CMD_READ_ADC_CHANNEL,
	SHBATT_CMD_READ_ADC_CHANNEL_BUFFERED,
	SHBATT_CMD_SET_VBATT_CALIBRATION_DATA,
	SHBATT_CMD_REFRESH_VBATT_CALIBRATION_DATA,
	SHBATT_CMD_CHECK_STARTUP_VOLTAGE,
	SHBATT_CMD_GET_BATTERY_LOG_INFO,
	SHBATT_CMD_POST_BATTERY_LOG_INFO,
	SHBATT_CMD_NOTIFY_CHARGER_CABLE_STATUS,
	SHBATT_CMD_NOTIFY_BATTERY_CHARGING_STATUS,
	SHBATT_CMD_NOTIFY_BATTERY_CAPACITY,
	SHBATT_CMD_NOTIFY_CHARGING_STATE_MACHINE_ENABLE,
	SHBATT_CMD_GET_CALIBRATE_BATTERY_VOLTAGE,
	SHBATT_CMD_INITIALIZE,
	SHBATT_CMD_EXEC_FUELGAUGE_SOC_POLL_SEQUENCE,
	SHBATT_CMD_EXEC_LOW_BATTERY_CHECK_SEQUENCE,
	SHBATT_CMD_EXEC_OVERCURR_CHECK_SEQUENCE,
	SHBATT_CMD_EXEC_BATTERY_PRESENT_CHECK_SEQUENCE,
	SHBATT_CMD_NOTIFY_WIRELESS_CHARGER_STATE_CHANGED,
	SHBATT_CMD_SET_LOG_ENABLE,
	SHBATT_CMD_GET_REAL_BATTERY_CAPACITY,
	SHBATT_CMD_NOTIFY_CHARGE_SWITCH_STATUS,
	SHBATT_CMD_GET_DEPLETED_CAPACITY,
	SHBATT_CMD_SET_DEPLETED_BATTERY_FLG,
	SHBATT_CMD_GET_DEPLETED_BATTERY_FLG,
	SHBATT_CMD_SET_BATTERY_HEALTH,
	SHBATT_CMD_SET_DEPLETED_CALC_ENABLE,
	NUM_SHBATT_CMD,
} shbatt_kernel_to_user_command_t;


typedef enum shbatt_low_battery_event_tag
{
  SHBATT_LOW_BATTERY_EVENT_LOW_INTERRUPT,
  SHBATT_LOW_BATTERY_EVENT_FATAL_INTERRUPT,
  SHBATT_LOW_BATTERY_EVENT_LOW_TIMER,
  SHBATT_LOW_BATTERY_EVENT_FATAL_TIMER,
  SHBATT_LOW_BATTERY_EVENT_CHARGING_FATAL_INTERRUPT,
  SHBATT_LOW_BATTERY_EVENT_CHARGING_FATAL_TIMER,
  SHBATT_LOW_BATTERY_EVENT_LOW_SHUTDOWN,
  SHBATT_LOW_BATTERY_EVENT_CHARGER_CONNECT,
  SHBATT_LOW_BATTERY_EVENT_CHARGER_DISCONNECT,
  NUM_SHBATT_LOW_BATTERY_EVENT

} shbatt_low_battery_event_t;

typedef enum shbatt_timer_type_tag
{
  SHBATT_TIMER_TYPE_0,
  SHBATT_TIMER_TYPE_1,
  NUM_SHBATT_TIMER_TYPE

} shbatt_timer_type_t;

typedef enum shbatt_timer_sleep_type_tag
{
  SHBATT_TIMER_TYPE_WAKEUP,
  SHBATT_TIMER_TYPE_SLEEP,
  NUM_SHBATT_TIMER_SLEEP_TYPE

} shbatt_timer_sleep_type_t;

/*+-----------------------------------------------------------------------------+*/
/*| @ STRUCT & UNION DECLARE :													|*/
/*+-----------------------------------------------------------------------------+*/

typedef struct shbatt_soc_tag
{
  shbatt_timer_type_t type;
  shbatt_timer_sleep_type_t sleep;

} shbatt_soc_t;

typedef struct shbatt_packet_hdr_tag
{
	shbatt_api_to_tsk_command_t	cmd;
	void* 					cb_p;
	struct completion*		cmp_p;
	shbatt_result_t* 		ret_p;

} shbatt_packet_hdr_t;
typedef union shbatt_packet_prm_tag
{
	shbatt_cable_status_t*		cbs_p;
	shbatt_chg_status_t*		cgs_p;
	shbatt_health_t*			hea_p;
	shbatt_present_t*			pre_p;
	int*						cap_p;
	int*						vol_p;
	int*						tmp_p;
	shbatt_technology_t*		tec_p;
	int*						cur_p;
	shbatt_adc_t*				adc_p;
	int*						chk_p;
	shbatt_adc_conv_uint_t*		ocv_p;
	shbatt_adc_conv_uint_t*		vsense_p;
	shbatt_adc_conv_uint_t*		vbatt_p;
	int*						cc_p;
	shbatt_adc_conv_int_t*		vsense_avg_p;
	shbatt_adc_conv_int_t*		vbatt_avg_p;
	shbatt_adc_conv_offset_t*	cc_mah_p;
	shbatt_vbatt_cal_t			cal;
	shbatt_cable_status_t		cbs;
	shbatt_chg_status_t			cgs;
	int							cap;
	int							val;
	shbatt_smbc_chg_t			chg;
	shbatt_vsense_avg_cal_t		vac;

	unsigned int* dev_p;				
	shbatt_fuelgauge_mode_t mode;
	shbatt_accumulate_current_t* acc_p;
	int* raw_p;
	shbatt_batt_log_info_t* bli_p;
	shbatt_boolean_t ena;
	shbatt_safety_t* saf_p;
	int evt;
	shbatt_batt_log_info_t bli;

	int seq;
	shbatt_soc_t soc;
	shbatt_chg_switch_status_t switch_status;
	int*						depleted_capacity_per;
	int depleted_battery_flg;
	int* depleted_battery_flg_p;
	shbatt_health_t				hea;
} shbatt_packet_prm_t;
typedef struct shbatt_packet_tag
{
	struct work_struct		work;
	shbatt_packet_hdr_t 	hdr;
	shbatt_packet_prm_t 	prm;
	bool is_used;	
} shbatt_packet_t;


typedef struct shbatt_pm_device_info_tag
{
  struct device* dev_p;

} shbatt_pm_device_info_t;
typedef enum shbatt_ps_category_tag
{
  SHBATT_PS_CATEGORY_BATTERY,
  SHBATT_PS_CATEGORY_USB,
  SHBATT_PS_CATEGORY_AC,
  SHBATT_PS_CATEGORY_FUELGAUGE,
  SHBATT_PS_CATEGORY_PMIC,
  SHBATT_PS_CATEGORY_ADC,
  NUM_SHBATT_POWER_SUPPLY_CAT

} shbatt_ps_category_t;
typedef enum shbatt_poll_timer_type_tag
{
  SHBATT_POLL_TIMER_TYPE_FUELGAUGE_SOC,
  SHBATT_POLL_TIMER_TYPE_FUELGAUGE_SOC_SLEEP,
  SHBATT_POLL_TIMER_TYPE_FUELGAUGE_SOC_MULTI,
  SHBATT_POLL_TIMER_TYPE_FUELGAUGE_SOC_SLEEP_MULTI,
  SHBATT_POLL_TIMER_TYPE_LOW_BATTERY,
  SHBATT_POLL_TIMER_TYPE_FATAL_BATTERY,
  SHBATT_POLL_TIMER_TYPE_CHARGING_FATAL_BATTERY,
  SHBATT_POLL_TIMER_TYPE_LOW_BATTERY_SHUTDOWN,
  SHBATT_POLL_TIMER_TYPE_OVERCURR,
  SHBATT_POLL_TIMER_TYPE_BATTERY_PRESENT,
  NUM_SHBATT_POLL_TIMER_TYPE

} shbatt_poll_timer_type_t;

typedef enum shbatt_voltage_alarm_type_tag
{
  SHBATT_VOLTAGE_ALARM_TYPE_LOW_BATTERY,
  SHBATT_VOLTAGE_ALARM_TYPE_FATAL_BATTERY,
  SHBATT_VOLTAGE_ALARM_TYPE_CHARGING_FATAL_BATTERY,
  NUM_SHBATT_VOLTAGE_ALARM_TYPE

} shbatt_voltage_alarm_type_t;

typedef struct shbatt_usse_packet_hdr_tag
{
  shbatt_kernel_to_user_command_t cmd;
  shbatt_result_t ret;

} shbatt_usse_packet_hdr_t;

typedef union shbatt_usse_packet_prm_tag
{
  shbatt_cable_status_t cbs;
  shbatt_chg_status_t cgs;
  shbatt_health_t hea;
  shbatt_present_t pre;
  int cap;
  int vol;
  int tmp;
  shbatt_technology_t tec;
  shbatt_safety_t saf;
  int cur;
  unsigned int dev;
  shbatt_fuelgauge_mode_t mode;
  shbatt_accumulate_current_t acc;
  int raw;
  shbatt_adc_t adc;
  shbatt_vbatt_cal_t cal;
  shbatt_batt_log_info_t bli;
  shbatt_boolean_t ena;
  int evt;
  int seq;
  int val;
  shbatt_boolean_t chk;
  shbatt_vsense_avg_cal_t vac;
  shbatt_soc_t soc;
  shbatt_chg_switch_status_t switch_status;
  int depleted_capacity_per;
	int depleted_battery_flg;
} shbatt_usse_packet_prm_t;

typedef struct shbatt_usse_packet_tag
{
  shbatt_usse_packet_hdr_t hdr;
  shbatt_usse_packet_prm_t prm;

} shbatt_usse_packet_t;

typedef struct shbatt_timer_tag
{
  struct alarm alm;
  int prm;

} shbatt_timer_t;

typedef struct shbatt_pm_reg_info_tag
{
  int mod;
  int reg;
  unsigned char buf[8];
  int len;

} shbatt_pm_reg_info_t;

typedef struct shbatt_fg_reg_info_tag
{
  int slv;
  int reg;
  unsigned char buf[8];
  int len;

} shbatt_fg_reg_info_t;

typedef struct shbatt_adc_conv_info_tag
{
  shbatt_adc_channel_t channel;
  int raw;
  int mv;
  int degc;

} shbatt_adc_conv_info_t;

typedef struct shbatt_poll_timer_info_tag
{
  shbatt_poll_timer_type_t ptt;
  int ms;
  int prm;

} shbatt_poll_timer_info_t;

typedef struct shbatt_voltage_alarm_info_tag
{
  shbatt_voltage_alarm_type_t vat;
  int max;
  int min;

} shbatt_voltage_alarm_info_t;

typedef struct
{
	int cur;
	int vol;
} shbatt_current_voltage_t;

typedef struct
{
	int request_num;
	shbatt_adc_t channels[NUM_SHBATT_ADC_CHANNEL];
} shbatt_adc_read_request_t;

/*+-----------------------------------------------------------------------------+*/
/*| @ CALLBACK FUNCTION TYPE DECLARE :                                          |*/
/*+-----------------------------------------------------------------------------+*/

typedef void (*shbatt_cb_func_t1)( shbatt_result_t result );

typedef void (*shbatt_cb_func_t2)( int val, shbatt_result_t result );

typedef void (*shbatt_cb_func_t3)( shbatt_present_t val, shbatt_result_t result );

typedef void (*shbatt_cb_func_t4)( shbatt_health_t val, shbatt_result_t result );

typedef void (*shbatt_cb_func_t5)( shbatt_technology_t val, shbatt_result_t result );

typedef void (*shbatt_cb_func_t6)( shbatt_safety_t val, shbatt_result_t result );

typedef void (*shbatt_cb_func_t7)( shbatt_cable_status_t val, shbatt_result_t result );

typedef void (*shbatt_cb_func_t8)( shbatt_chg_status_t val, shbatt_result_t result );

typedef void (*shbatt_cb_func_t9)( shbatt_adc_t val, shbatt_result_t result );

typedef void (*shbatt_cb_func_t10)( unsigned int val, shbatt_result_t result );

typedef void (*shbatt_cb_func_t11)( shbatt_accumulate_current_t val, shbatt_result_t result );

typedef void (*shbatt_cb_func_t12)( shbatt_batt_log_info_t val, shbatt_result_t result );

typedef void (*shbatt_cb_func_t13)( void* arg_p );

typedef void (*shbatt_cb_func_t14)( shbatt_boolean_t chk, shbatt_result_t result );


/*+-----------------------------------------------------------------------------+*/
/*| @ THIS FILE END :															|*/
/*+-----------------------------------------------------------------------------+*/

#endif	/* SHBATT_TYPE_H */
