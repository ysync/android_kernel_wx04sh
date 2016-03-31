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

#ifndef SHBATT_KERL_H
#define SHBATT_KERL_H

/*+-----------------------------------------------------------------------------+*/
/*| @ INCLUDE FILE :															|*/
/*+-----------------------------------------------------------------------------+*/

  /* NONE.. */

/*+-----------------------------------------------------------------------------+*/
/*| @ VALUE DEFINE DECLARE :													|*/
/*+-----------------------------------------------------------------------------+*/

  /* NONE.. */

/*+-----------------------------------------------------------------------------+*/
/*| @ ENUMERATION DECLARE : 													|*/
/*+-----------------------------------------------------------------------------+*/

typedef enum shbatt_result_tag
{
	SHBATT_RESULT_SUCCESS,
	SHBATT_RESULT_FAIL,
	SHBATT_RESULT_REJECTED,
	NUM_SHBATT_RESULT

} shbatt_result_t;



typedef enum shbatt_cable_status_tag
{
	SHBATT_CABLE_STATUS_UNKNOWN,
	SHBATT_CABLE_STATUS_USB,
	SHBATT_CABLE_STATUS_AC,
	SHBATT_CABLE_STATUS_BOTH,
	SHBATT_CABLE_STATUS_NONE,

	NUM_SHBATT_CABLE_STATUS

} shbatt_cable_status_t;

typedef enum shbatt_chg_status_tag
{
	SHBATT_CHG_STATUS_UNKNOWN,
	SHBATT_CHG_STATUS_CHARGING,
	SHBATT_CHG_STATUS_DISCHARGING,
	SHBATT_CHG_STATUS_NOT_CHARGING,
	SHBATT_CHG_STATUS_FULL,
	NUM_SHBATT_CHG_STATUS

} shbatt_chg_status_t;

typedef enum shbatt_health_tag
{
	SHBATT_HEALTH_BATT_UNKNOWN,
	SHBATT_HEALTH_BATT_GOOD,
	SHBATT_HEALTH_BATT_OVERHEAT,
	SHBATT_HEALTH_BATT_DEAD,
	SHBATT_HEALTH_BATT_OVERVOLTAGE,
	SHBATT_HEALTH_BATT_UNSPEC_FAILURE,
	SHBATT_HEALTH_BATT_COLD,
	NUM_SHBATT_HEALTH

} shbatt_health_t;

typedef enum shbatt_present_tag
{
	SHBATT_PRESENT_BATT_NONE,
	SHBATT_PRESENT_BATT_CONNECTED,
	NUM_SHBATT_PRESEN

} shbatt_present_t;

typedef enum shbatt_technology_tag
{
	SHBATT_TECHNOLOGY_BATT_UNKNOWN,
	SHBATT_TECHNOLOGY_BATT_NIMH,
	SHBATT_TECHNOLOGY_BATT_LION,
	SHBATT_TECHNOLOGY_BATT_LIPO,
	SHBATT_TECHNOLOGY_BATT_LIFE,
	SHBATT_TECHNOLOGY_BATT_NICD,
	SHBATT_TECHNOLOGY_BATT_LIMN,
	NUM_SHBATT_TECHNOLOGY

} shbatt_technology_t;

typedef enum shbatt_fuelgauge_mode_tag
{
  SHBATT_FUELGAUGE_MODE_STANDBY,
  SHBATT_FUELGAUGE_MODE_OPERATING,
  NUM_SHBATT_FUELGAUGE_MODE
} shbatt_fuelgauge_mode_t;
typedef enum shbatt_boolean_tag
{
  SHBATT_BOOLEAN_FALSE,
  SHBATT_BOOLEAN_TRUE,
  NUM_SHBATT_BOOLEAN
} shbatt_boolean_t;
typedef enum shbatt_adc_channel_tag
{
	SHBATT_ADC_CHANNEL_VCOIN,
	SHBATT_ADC_CHANNEL_VBAT,
	SHBATT_ADC_CHANNEL_DCIN,
	SHBATT_ADC_CHANNEL_ICHG,
	SHBATT_ADC_CHANNEL_VPH_PWR,
	SHBATT_ADC_CHANNEL_IBAT,
	SHBATT_ADC_CHANNEL_CAM_TEMP,
	SHBATT_ADC_CHANNEL_BAT_THERM,
	SHBATT_ADC_CHANNEL_BAT_ID,
	SHBATT_ADC_CHANNEL_USBIN,
	SHBATT_ADC_CHANNEL_PMIC_TEMP,
	SHBATT_ADC_CHANNEL_625MV,
	SHBATT_ADC_CHANNEL_125V,
	SHBATT_ADC_CHANNEL_CHG_TEMP,
	SHBATT_ADC_CHANNEL_XO_THERM,
	SHBATT_ADC_CHANNEL_PA_THERM,
	SHBATT_ADC_CHANNEL_USB_ID,
	SHBATT_ADC_CHANNEL_BAT_THERM_NO_MOVING_AVERAGE,
	SHBATT_ADC_CHANNEL_BACKLIGHT_TEMP,
	NUM_SHBATT_ADC_CHANNEL
} shbatt_adc_channel_t;
typedef enum shbatt_drv_adc_channel_tag
{
	SHBATT_DRV_ADC_CHANNEL_VCOIN = 0,
	SHBATT_DRV_ADC_CHANNEL_VBAT,
	SHBATT_DRV_ADC_CHANNEL_DCIN,
	SHBATT_DRV_ADC_CHANNEL_ICHG = 66,
	SHBATT_DRV_ADC_CHANNEL_VPH_PWR = 4,
	SHBATT_DRV_ADC_CHANNEL_IBAT,
	SHBATT_DRV_ADC_CHANNEL_CAM_TEMP = 26,	//ADC_MPP_1_AMUX6
	SHBATT_DRV_ADC_CHANNEL_BAT_THERM = 8,
	SHBATT_DRV_ADC_CHANNEL_BAT_ID,
	SHBATT_DRV_ADC_CHANNEL_USBIN,
	SHBATT_DRV_ADC_CHANNEL_PMIC_TEMP,
	SHBATT_DRV_ADC_CHANNEL_625MV,
	SHBATT_DRV_ADC_CHANNEL_125V,
	SHBATT_DRV_ADC_CHANNEL_CHG_TEMP,
	SHBATT_DRV_ADC_CHANNEL_XO_THERM,		//CHANNEL_MUXOFF
	SHBATT_DRV_ADC_CHANNEL_PA_THERM = 23,	//ADC_MPP_1_AMUX3
	SHBATT_DRV_ADC_CHANNEL_USB_ID = 86,		//USB_ID
	SHBATT_DRV_ADC_CHANNEL_BAT_THERM_NO_MOVING_AVERAGE = 99,	// BAT_THERM
	SHBATT_DRV_ADC_CHANNEL_BACKLIGHT_TEMP = 106, //ADC_MPP_1_BACKLIGHT_AMUX6
	NUM_DRV_SHBATT_ADC_CHANNEL = 19
} shbatt_drv_adc_channel_t;

typedef enum shbatt_cpu_clock_level_tag
{
	SHBATT_CPU_CLOCK_LEVEL0,
	SHBATT_CPU_CLOCK_LEVEL1,
	SHBATT_CPU_CLOCK_LEVEL2,
	SHBATT_CPU_CLOCK_LEVEL3,
	SHBATT_CPU_CLOCK_LEVEL4,
	SHBATT_CPU_CLOCK_LEVEL5,
	NUM_SHBATT_CPU_CLOCK_LEVEL,
} shbatt_cpu_clock_level_t;

typedef enum shbatt_chg_switch_status_tag
{
	SHBATT_CHG_SWITCH_ON,
	SHBATT_CHG_SWITCH_OFF,
	SHBATT_CHG_SWITCH_WAIT,
	NUM_SHBATT_CHG_SWITCH_STATUS
} shbatt_chg_switch_status_t;

/*+-----------------------------------------------------------------------------+*/
/*| @ STRUCT & UNION DECLARE :													|*/
/*+-----------------------------------------------------------------------------+*/

typedef struct shbatt_adc_tag
{
	shbatt_adc_channel_t	channel;
	int32_t					adc_code;
	int64_t					measurement;
	int64_t					physical;
} shbatt_adc_t;

typedef struct shbatt_vbatt_cal_tag
{
	int min;
	int max;
	int vmin;
	int vmax;

} shbatt_vbatt_cal_t;

typedef struct shbatt_smbc_chg_tag
{
	int vol;
	int cur;

} shbatt_smbc_chg_t;

typedef struct shbatt_adc_conv_long_tag
{
	int    adc_code;
	int64_t measurement;

} shbatt_adc_conv_long_t;

typedef struct shbatt_adc_conv_uint_tag
{
	uint adc_code;
	uint measurement;

} shbatt_adc_conv_uint_t;

typedef struct shbatt_adc_conv_int_tag
{
	int adc_code;
	int  measurement;

} shbatt_adc_conv_int_t;

typedef struct shbatt_adc_conv_offset_tag
{
	int offset;
	shbatt_adc_conv_long_t adc_convl;

} shbatt_adc_conv_offset_t;

typedef struct shbatt_accumulate_current_tag
{
  int cur;
  int cnt;
  int raw;
} shbatt_accumulate_current_t;
typedef struct shbatt_batt_log_info_tag
{
  int event_num;
  int bat_vol;
  int chg_vol;
  int chg_cur;
  int bat_temp;
  int chg_temp;
  int cam_temp;
  int pmic_temp;
  int pa_temp;
  int avg_cur;
  int avg_vol;
  int latest_cur;
  int acc_cur;
  int vol_per;
  int cur_dep_per;
  int avg_dep_per;
} shbatt_batt_log_info_t;
typedef enum shbatt_safety_tag
{
  SHBATT_SAFETY_BATT_UNKNOWN,
  SHBATT_SAFETY_BATT_GOOD,
  SHBATT_SAFETY_BATT_OVERCHARGING,
  NUM_SHBATT_SAFETY
} shbatt_safety_t;

typedef struct shbatt_vsense_avg_cal_tag
{
	int adjmin;
	int adjmax;
	int curmin;
	int curmax;

} shbatt_vsense_avg_cal_t;

typedef struct shbatt_smem_info_tag
{
  int battery_present;
  int battery_voltage;
  int battery_temperature;
  int xo_temperature;
  int cable_status;
  int product_flg;
  int softupdate_flg;
  int abnormal_flg;
  int restrain_flg;
  unsigned char fullchg_flg;
  unsigned char charge_th_high_array[8];
  unsigned char charge_th_low_array[8];
  int shbatt_fuel_data[4];
  int shbatt_vbat_data[4];

} shbatt_smem_info_t;

/*+-----------------------------------------------------------------------------+*/
/*| @ PUBLIC VARIABLE : 														|*/
/*+-----------------------------------------------------------------------------+*/

  /* NONE.. */

/*+-----------------------------------------------------------------------------+*/
/*| @ PUBLIC FUNCTION PROTO TYPE DECLARE :										|*/
/*+-----------------------------------------------------------------------------+*/
shbatt_result_t shbatt_api_get_charger_cable_status( shbatt_cable_status_t* cbs_p );
shbatt_result_t shbatt_api_get_battery_charging_status( shbatt_chg_status_t* cgs_p );
shbatt_result_t shbatt_api_get_battery_capacity( int* cap_p );
shbatt_result_t shbatt_api_get_battery_voltage( int* vol_p );
shbatt_result_t shbatt_api_get_battery_present( shbatt_present_t* pre_p );
shbatt_result_t shbatt_api_get_battery_health( shbatt_health_t* hea_p );
shbatt_result_t shbatt_api_get_battery_technology( shbatt_technology_t* tec_p );
shbatt_result_t shbatt_api_get_battery_current( int* cur_p );
shbatt_result_t shbatt_api_read_adc_channel( shbatt_adc_t* adc_p );
shbatt_result_t shbatt_api_set_vbatt_calibration_data( shbatt_vbatt_cal_t cal );
shbatt_result_t shbatt_api_refresh_vbatt_calibration_data( void );
shbatt_result_t shbatt_api_set_pmic_charger_transistor_switch( int val );
shbatt_result_t shbatt_api_set_pmic_battery_transistor_switch( int val );
shbatt_result_t shbatt_api_set_pmic_vmaxsel( int val );
shbatt_result_t shbatt_api_set_pmic_imaxsel( int val );
shbatt_result_t shbatt_api_set_pmic_vtrickle( int val );
shbatt_result_t shbatt_api_set_pmic_itrickle( int val );
shbatt_result_t shbatt_api_check_startup_voltage( int* chk_p );
shbatt_result_t shbatt_api_start_battery_present_check( void );
shbatt_result_t shbatt_api_get_battery_temperature( int* tmp_p );
shbatt_result_t shbatt_api_get_charger_temperature( int* tmp_p );
shbatt_result_t shbatt_api_get_camera_temperature( int* tmp_p );
shbatt_result_t shbatt_api_get_pa_temperature( int* tmp_p );
shbatt_result_t shbatt_api_get_xo_temperature( int* tmp_p );
shbatt_result_t shbatt_api_get_pmic_temperature( int* tmp_p );
shbatt_result_t shbatt_api_set_smbc_charger( shbatt_smbc_chg_t chg );
shbatt_result_t shbatt_api_set_meas_freq( int val );
shbatt_result_t shbatt_api_read_ocv_for_rbatt( shbatt_adc_conv_uint_t* ocv_p );
shbatt_result_t shbatt_api_read_vsense_for_rbatt( shbatt_adc_conv_uint_t* adc_convu_p );
shbatt_result_t shbatt_api_read_vbatt_for_rbatt( shbatt_adc_conv_uint_t* vbatt_p );
shbatt_result_t shbatt_api_read_cc( shbatt_adc_conv_long_t* adc_convl_p );
shbatt_result_t shbatt_api_read_cc_offset( shbatt_adc_conv_long_t* adc_convl_p, int offset );
shbatt_result_t shbatt_api_read_last_good_ocv( shbatt_adc_conv_uint_t* ocv_p );
shbatt_result_t shbatt_api_read_vsense_avg( shbatt_adc_conv_int_t* adc_convi_p );
shbatt_result_t shbatt_api_read_vbatt_avg( shbatt_adc_conv_int_t* vbatt_avg_p );
shbatt_result_t shbatt_api_auto_enable( int val );
shbatt_result_t shbatt_api_recalib_adc_device( void );
shbatt_result_t shbatt_api_get_calibrate_battery_voltage( int* vol_p );
shbatt_result_t shbatt_api_get_wireless_status( int* vol_p );
shbatt_result_t shbatt_api_set_wireless_switch( int val );
shbatt_result_t shbatt_api_set_vsense_avg_calibration_data( shbatt_vsense_avg_cal_t vac );
shbatt_result_t shbatt_api_refresh_vsense_avg_calibration_data( void );
/* TODO New API add point */
shbatt_result_t shbatt_api_get_terminal_temperature( int* tmp_p );
shbatt_result_t shbatt_api_get_modem_temperature( int* tmp_p );
shbatt_result_t shbatt_api_get_fuelgauge_current( int* cur_p );	
shbatt_result_t shbatt_api_get_fuelgauge_voltage( int* vol_p );
shbatt_result_t shbatt_api_get_fuelgauge_device_id( unsigned int* dev_p );
shbatt_result_t shbatt_api_set_fuelgauge_mode( shbatt_fuelgauge_mode_t mode );
shbatt_result_t shbatt_api_get_fuelgauge_accumulate_current( shbatt_accumulate_current_t* acc_p );
shbatt_result_t shbatt_api_clr_fuelgauge_accumulate_current( void );
shbatt_result_t shbatt_api_get_fuelgauge_temperature( int* tmp_p );
shbatt_result_t shbatt_api_get_fuelgauge_current_ad( int* raw_p );
shbatt_result_t shbatt_api_read_adc_channel_buffered( shbatt_adc_t* adc_p );
shbatt_result_t shbatt_api_get_battery_log_info( shbatt_batt_log_info_t* bli_p );
shbatt_result_t shbatt_api_post_battery_log_info( shbatt_batt_log_info_t bli );
shbatt_result_t shbatt_api_notify_charger_cable_status( shbatt_cable_status_t cbs );
shbatt_result_t shbatt_api_notify_battery_charging_status( shbatt_chg_status_t cgs );
shbatt_result_t shbatt_api_notify_battery_capacity( int cap );
shbatt_result_t shbatt_api_notify_charging_state_machine_enable( shbatt_boolean_t ena );
shbatt_result_t shbatt_api_get_calibrate_battery_voltage( int* vol_p );
shbatt_result_t shbatt_api_get_battery_safety( shbatt_safety_t* saf_p );
shbatt_result_t shbatt_api_read_adc_channel_no_conversion( shbatt_adc_t* adc_p );
shbatt_result_t shbatt_api_get_hw_revision( uint * p_hw_rev );
shbatt_result_t shbatt_api_get_smem_info( shbatt_smem_info_t * p_smem_info );
shbatt_result_t shbatt_api_get_backlight_temperature( int* tmp_p );
shbatt_result_t shbatt_api_notify_charge_switch_status( shbatt_chg_switch_status_t switch_status );
shbatt_result_t shbatt_api_get_bms_control( int* val );

/*+-----------------------------------------------------------------------------+*/
/*| @ PRIVATE FUNCTION PROTO TYPE DECLARE : 									|*/
/*+-----------------------------------------------------------------------------+*/


/*+-----------------------------------------------------------------------------+*/
/*| @ THIS FILE END :															|*/
/*+-----------------------------------------------------------------------------+*/

#endif	/* SHBATT_KERL_H */
