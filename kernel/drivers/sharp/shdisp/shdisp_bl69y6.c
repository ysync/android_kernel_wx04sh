/* drivers/sharp/shdisp/shdisp_bl69y6.c  (Display Driver)
 *
 * Copyright (C) 2011-2012 SHARP CORPORATION
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

/* ------------------------------------------------------------------------- */
/* SHARP DISPLAY DRIVER FOR KERNEL MODE                                      */
/* ------------------------------------------------------------------------- */

/* ------------------------------------------------------------------------- */
/* INCLUDE FILES                                                             */
/* ------------------------------------------------------------------------- */

#include <linux/compiler.h>
#include <linux/types.h>
#include <linux/kernel.h>
#include <linux/module.h>
#include <linux/fs.h>
#include <linux/platform_device.h>
#include <linux/device.h>
#include <linux/cdev.h>
#include <linux/interrupt.h>
#include <linux/init.h>
#include <linux/poll.h>
#include <linux/mm.h>
#include <linux/idr.h>
#include <linux/kobject.h>
#include <linux/string.h>
#include <linux/ioctl.h>
#include <linux/gpio.h>
#include <linux/hrtimer.h>
#include <linux/irq.h>
#include <linux/i2c.h>
#include <linux/input.h>
#include <linux/slab.h>
#include <linux/delay.h>
#include <sharp/shdisp_kerl.h>
#include "shdisp_system.h"
#include "shdisp_type.h"
#include "shdisp_bl69y6.h"


#if defined(CONFIG_MACH_DECKARD_GP4)
#include "./data/shdisp_bl69y6_data_gp4.h"
#elif defined(CONFIG_MACH_DECKARD_AS70)
#include "./data/shdisp_bl69y6_data_as70.h"
#elif defined(CONFIG_MACH_TOR)
#include "./data/shdisp_bl69y6_data_tori.h"
#elif defined(CONFIG_MACH_DECKARD_AS38)
#include "./data/shdisp_bl69y6_data_as38.h"
#elif defined(CONFIG_MACH_LYNX_DL15)
#include "./data/shdisp_bl69y6_data_dl15.h"
#elif defined(CONFIG_MACH_DECKARD_AF33)
#include "./data/shdisp_bl69y6_data_af33.h"
#elif defined(CONFIG_MACH_LYNX_DL12)
#include "./data/shdisp_bl69y6_data_dl12.h"
#elif defined(CONFIG_MACH_LYNX_DL10)
#include "./data/shdisp_bl69y6_data_dl10.h"
#elif defined(CONFIG_MACH_BLT)
#include "./data/shdisp_bl69y6_data_bolt.h"
#elif defined(CONFIG_MACH_MNB)
#include "./data/shdisp_bl69y6_data_mnb.h"
#elif defined(CONFIG_MACH_ARS)
#include "./data/shdisp_bl69y6_data_ars.h"
#else /* CONFIG_MACH_DEFAULT */
#include "./data/shdisp_bl69y6_data_default.h"
#endif /* CONFIG_MACH_ */

/* ------------------------------------------------------------------------- */
/* MACROS                                                                    */
/* ------------------------------------------------------------------------- */
#define SHDISP_BDIC_BKL_MODE_OFF            0
#define SHDISP_BDIC_BKL_MODE_FIX            1
#define SHDISP_BDIC_BKL_MODE_AUTO           2

#define SHDISP_BDIC_BKL_PWM_DISABLE         0
#define SHDISP_BDIC_BKL_PWM_ENABLE          1

#define SHDISP_BDIC_BKL_DTV_OFF             0
#define SHDISP_BDIC_BKL_DTV_ON              1

#define SHDISP_BDIC_BKL_EMG_OFF             0
#define SHDISP_BDIC_BKL_EMG_ON              1

#define SHDISP_BDIC_BKL_ECO_OFF             0
#define SHDISP_BDIC_BKL_ECO_ON              1

#define SHDISP_BDIC_BKL_CHG_OFF             0
#define SHDISP_BDIC_BKL_CHG_ON              1

#define SHDISP_BDIC_TRI_LED_MODE_NORMAL     0
#define SHDISP_BDIC_TRI_LED_MODE_BLINK      1
#define SHDISP_BDIC_TRI_LED_MODE_FIREFLY    2

#define SHDISP_BDIC_I2C_IS_NOT_SELECTED     0
#define SHDISP_BDIC_I2C_IS_SELECTED         1

#define SHDISP_BDIC_I2C_W_START             0x01
#define SHDISP_BDIC_I2C_R_START             0x02
#define SHDISP_BDIC_I2C_R_TIMRE_ST          0x10
#define SHDISP_BDIC_I2C_ADO_UPDATE_MASK     0x60
#define SHDISP_BDIC_I2C_SEND_WAIT           1000
#define SHDISP_BDIC_I2C_SEND_RETRY          200

#define SHDISP_BDIC_I2C_SEND_RETRY_WAIT     100

#define SHDISP_BDIC_DISABLE_BKL_CHG_MODE_FOR_FIX

/* ------------------------------------------------------------------------- */
/* PROTOTYPES                                                                */
/* ------------------------------------------------------------------------- */

static void shdisp_bdic_LD_GPIO_control(unsigned char symbol, unsigned char status);
static void shdisp_bdic_LD_LCD_BKL_active(void);
static void shdisp_bdic_LD_LCD_BKL_standby(void);
static void shdisp_bdic_LD_LCD_BKL_off(void);
static void shdisp_bdic_LD_LCD_BKL_fix_on(int param);
static void shdisp_bdic_LD_LCD_BKL_auto_on(int param);
static void shdisp_bdic_LD_LCD_PWR_active(void);
static void shdisp_bdic_LD_LCD_PWR_standby(void);
static void shdisp_bdic_LD_LCD_PWR_on(void);
static void shdisp_bdic_LD_LCD_PWR_off(void);
static void shdisp_bdic_LD_LCD_M_PWR_on(void);
static void shdisp_bdic_LD_LCD_M_PWR_off(void);
static void shdisp_bdic_LD_LCD_VO2_ON(void);
static void shdisp_bdic_LD_LCD_VO2_OFF(void);
static void shdisp_bdic_LD_TRI_LED_active(void);
static void shdisp_bdic_LD_TRI_LED_standby(void);
static void shdisp_bdic_LD_TRI_LED_off(void);
static void shdisp_bdic_LD_TRI_LED_normal_on(unsigned char color);
static void shdisp_bdic_LD_TRI_LED_blink_on(unsigned char color, int ontime, int interval);
static void shdisp_bdic_LD_TRI_LED_firefly_on(unsigned char color, int ontime, int interval);
static void shdisp_bdic_LD_TRI_LED_ANIME_active(void);
static void shdisp_bdic_LD_TRI_LED_ANIME_standby(void);
static void shdisp_bdic_LD_PHOTO_SENSOR_active(unsigned long dev_type);
static void shdisp_bdic_LD_PHOTO_SENSOR_standby(unsigned long dev_type);
static int shdisp_bdic_LD_PHOTO_SENSOR_get_val(unsigned short *value);
static int shdisp_bdic_LD_PHOTO_SENSOR_get_lux(unsigned short *value, unsigned long *lux);
static void shdisp_bdic_LD_PHOTO_SENSOR_bias_als_adj(unsigned char als_in, unsigned char *als_out);
static int shdisp_bdic_LD_i2c_write(struct shdisp_bdic_i2c_msg *msg);
static int shdisp_bdic_LD_i2c_read_mode0(struct shdisp_bdic_i2c_msg *msg);
static int shdisp_bdic_LD_i2c_read_mode1(struct shdisp_bdic_i2c_msg *msg);
static int shdisp_bdic_LD_i2c_read_mode2(struct shdisp_bdic_i2c_msg *msg);
static int shdisp_bdic_LD_i2c_read_mode3(struct shdisp_bdic_i2c_msg *msg);
static void shdisp_bdic_LD_PROX_SENSOR_active(unsigned long dev_type);
static void shdisp_bdic_LD_PROX_SENSOR_standby(unsigned long dev_type);
static void shdisp_bdic_LD_SENSOR_get_state(int *state);
static void shdisp_bdic_LD_LCD_BKL_pwm_disable(void);
static void shdisp_bdic_LD_LCD_BKL_pwm_enable(void);
static void shdisp_bdic_LD_LCD_BKL_dtv_on(void);
static void shdisp_bdic_LD_LCD_BKL_dtv_off(void);
static void shdisp_bdic_LD_LCD_BKL_emg_on(void);
static void shdisp_bdic_LD_LCD_BKL_emg_off(void);
static void shdisp_bdic_LD_LCD_BKL_get_mode(int *mode);
static void shdisp_bdic_LD_LCD_BKL_eco_on(void);
static void shdisp_bdic_LD_LCD_BKL_eco_off(void);
static void shdisp_bdic_LD_LCD_BKL_chg_on(void);
static void shdisp_bdic_LD_LCD_BKL_chg_off(void);

static void shdisp_bdic_PD_BDIC_init(void);
static void shdisp_bdic_PD_BDIC_active(void);
static void shdisp_bdic_PD_BDIC_standby(void);
static void shdisp_bdic_PD_BKL_control(unsigned char request, int param);
static void shdisp_bdic_PD_GPIO_control(unsigned char port, unsigned char status);
static void shdisp_bdic_PD_TRI_LED_control(unsigned char request, int param);
static void shdisp_bdic_PD_TRI_LED_set_anime(void);
static void shdisp_bdic_PD_TRI_LED_set_chdig(void);
static void shdisp_bdic_PD_REG_M1LED_set_value(void);
static void shdisp_bdic_PD_REG_OPT_set_value( int table );
static void shdisp_bdic_PD_REG_OPT_temp_set( unsigned char value );
static int  shdisp_bdic_PHOTO_SENSOR_lux_init(void);
static void shdisp_bdic_PD_REG_ADO_get_opt(unsigned short *value);
static void shdisp_bdic_PD_PHOTO_SENSOR_control(unsigned long device, unsigned char request);
static void shdisp_bdic_PD_PROX_SENSOR_control(unsigned long device, unsigned char request);
static void shdisp_bdic_PD_SENSOR_control(unsigned char request, unsigned char sensor);
static unsigned char shdisp_bdic_get_opt_value(int table);

static int shdisp_bdic_IO_write_reg(unsigned char reg, unsigned char val);
static int shdisp_bdic_IO_read_reg(unsigned char reg, unsigned char *val);
static int shdisp_bdic_IO_read_no_check_reg(unsigned char reg, unsigned char *val);
static int shdisp_bdic_IO_read_check_reg(unsigned char reg, unsigned char *val);
static int shdisp_bdic_IO_multi_read_reg(unsigned char reg, unsigned char *val, int size);
static int shdisp_bdic_IO_set_bit_reg(unsigned char reg, unsigned char val);
static int shdisp_bdic_IO_clr_bit_reg(unsigned char reg, unsigned char val);
static int shdisp_bdic_IO_msk_bit_reg(unsigned char reg, unsigned char val, unsigned char msk);
static int shdisp_photo_sensor_change_range( int bkl_opt );
static int shdisp_photo_sensor_IO_write_reg(unsigned char reg, unsigned char val);
static int shdisp_photo_sensor_IO_read_reg(unsigned char reg, unsigned char *val);
static int shdisp_photo_sensor_IO_burst_write_reg(unsigned char *wval, unsigned char dataNum);
static int shdisp_photo_sensor_IO_write_threshold(void);

static int shdisp_bdic_ps_req_mask( unsigned char *remask );
static int shdisp_bdic_ps_req_restart( unsigned char remask );

/* ------------------------------------------------------------------------- */
/* VARIABLES                                                                 */
/* ------------------------------------------------------------------------- */

static struct shdisp_bdic_state_str s_state_str;

static int shdisp_bdic_bkl_mode;
static int shdisp_bdic_bkl_param;

static int shdisp_bdic_pwm;

static int shdisp_bdic_dtv;

static int shdisp_bdic_emg;

static int shdisp_bdic_eco;

static int shdisp_bdic_chg;

static unsigned char shdisp_bdic_tri_led_color;
static int shdisp_bdic_tri_led_mode;
static int shdisp_bdic_tri_led_ontime;
static int shdisp_bdic_tri_led_interval;

static int shdisp_bdic_i2c_errflag;

static struct shdisp_main_bkl_ctl shdisp_bkl_priority_table[NUM_SHDISP_MAIN_BKL_DEV_TYPE] = {
    { SHDISP_MAIN_BKL_MODE_OFF      , SHDISP_MAIN_BKL_PARAM_0   },
    { SHDISP_MAIN_BKL_MODE_AUTO     , SHDISP_MAIN_BKL_PARAM_0   }
};

#if defined(CONFIG_SHDISP_PANEL_SWITCH)
static unsigned char shdisp_bdic_reg_data[4];
#endif

static unsigned int shdisp_bdic_irq_fac = 0;
static unsigned int shdisp_bdic_irq_fac_exe = 0;

static int  shdisp_bdic_irq_prioriy[SHDISP_IRQ_MAX_KIND];

static unsigned char shdisp_backup_irq_reg[6];
static unsigned char shdisp_backup_irq_photo_req[3];	

static int shdisp_bdic_irq_det_flag = 0;

static int shdisp_bdic_bkl_off_flg;

/* ------------------------------------------------------------------------- */
/* DEBUG MACRAOS                                                             */
/* ------------------------------------------------------------------------- */
//#define SHDISP_SW_BDIC_I2C_RWLOG
//#define SHDISP_SW_BDIC_POW_CTLLOG
//#define SHDISP_SW_BDIC_ADJUST_DATALOG
//#define SHDISP_SW_BDIC_IRQ_LOG

#define SHDISP_BL69Y6_FILE "shdisp_bl69y6.c"
#define SHDISP_ERR(fmt, args...) \
        printk("[SHDISP_ERROR][%s][%s] " fmt, SHDISP_BL69Y6_FILE, __func__, ## args);

/* ------------------------------------------------------------------------- */
/* FUNCTIONS                                                                 */
/* ------------------------------------------------------------------------- */
/* ------------------------------------------------------------------------- */
/* API                                                                       */
/* ------------------------------------------------------------------------- */
/* ------------------------------------------------------------------------- */
/* shdisp_bdic_API_boot_init                                                 */
/* ------------------------------------------------------------------------- */

int shdisp_bdic_API_boot_init( void )
{
    int ret;
    unsigned char dummy;

    shdisp_bdic_bkl_mode  = SHDISP_BDIC_BKL_MODE_OFF;
    shdisp_bdic_bkl_param = 0;

    shdisp_bdic_pwm = SHDISP_BDIC_BKL_PWM_DISABLE;

    shdisp_bdic_dtv = SHDISP_BDIC_BKL_DTV_OFF;

    shdisp_bdic_emg = SHDISP_BDIC_BKL_EMG_OFF;

    shdisp_bdic_eco = SHDISP_BDIC_BKL_ECO_OFF;

    shdisp_bdic_chg = SHDISP_BDIC_BKL_CHG_OFF;

    shdisp_bdic_tri_led_color    = 0;
    shdisp_bdic_tri_led_mode     = SHDISP_BDIC_TRI_LED_MODE_NORMAL;
    shdisp_bdic_tri_led_ontime   = 0;
    shdisp_bdic_tri_led_interval = 0;

    shdisp_SYS_set_Host_gpio(SHDISP_GPIO_NUM_BL_RST_N, SHDISP_GPIO_CTL_LOW);
    shdisp_SYS_delay_us(15000);
    shdisp_SYS_set_Host_gpio(SHDISP_GPIO_NUM_BL_RST_N, SHDISP_GPIO_CTL_HIGH);
    shdisp_SYS_delay_us(1000);

    ret = shdisp_SYS_bdic_i2c_read(BDIC_REG_SYSTEM1, &dummy);
    if (ret != SHDISP_RESULT_SUCCESS){
        return SHDISP_BDIC_IS_NOT_EXIST;
    }

    return SHDISP_BDIC_IS_EXIST;
}


/* ------------------------------------------------------------------------- */
/* shdisp_bdic_API_bdic_exist                                                */
/* ------------------------------------------------------------------------- */

void shdisp_bdic_API_bdic_exist(int* bdic_is_exist)
{
    *bdic_is_exist = s_state_str.bdic_is_exist;
    return;
}


/* ------------------------------------------------------------------------- */
/* shdisp_bdic_API_initialize                                                */
/* ------------------------------------------------------------------------- */

void shdisp_bdic_API_initialize(struct shdisp_bdic_state_str* state_str)
{
    int idx;

    s_state_str.bdic_power_status = state_str->bdic_power_status;

    for (idx = 0; idx < NUM_SHDISP_BDIC_DEV_TYPE; idx++) {
        s_state_str.bdic_power_status_tbl[idx] = state_str->bdic_power_status_tbl[idx];
    }

    for (idx = 0; idx < NUM_SHDISP_BDIC_DEV_TYPE; idx++) {
        s_state_str.bdic_power_resume_tbl[idx] = SHDISP_BDIC_DEV_PWR_OFF;
    }

    s_state_str.bdic_is_exist = state_str->bdic_is_exist;
    s_state_str.bdic_main_bkl_opt_mode_output = SHDISP_BDIC_MAIN_BKL_OPT_LOW;
    s_state_str.bdic_main_bkl_opt_mode_ado = SHDISP_BDIC_MAIN_BKL_OPT_LOW;
    s_state_str.shdisp_lux_change_level1 = SHDISP_LUX_CHANGE_LEVEL1;
    s_state_str.shdisp_lux_change_level2 = SHDISP_LUX_CHANGE_LEVEL2;

    s_state_str.als_adj0_low  = state_str->als_adj0_low;
    s_state_str.als_adj0_high = state_str->als_adj0_high;
    s_state_str.als_adj1_low  = state_str->als_adj1_low;
    s_state_str.als_adj1_high = state_str->als_adj1_high;
    s_state_str.als_shift     = state_str->als_shift;
    s_state_str.clear_offset  = state_str->clear_offset;
    s_state_str.ir_offset     = state_str->ir_offset;
    s_state_str.bkl_adjust_val = state_str->bkl_adjust_val;
    memset(&(s_state_str.shdisp_bdic_prox_params), 0, sizeof( struct shdisp_prox_params));
    
#if defined(CONFIG_SHDISP_PANEL_SWITCH)
    if(shdisp_api_get_panel_info() == 0) {
        shdisp_bdic_reg_data[0] = BDIC_REG_DCDC1_VLIM_VAL;
        shdisp_bdic_reg_data[1] = BDIC_REG_DCDC_SYS_VAL;
        shdisp_bdic_reg_data[2] = BDIC_REG_SYSTEM2_BKL;
        shdisp_bdic_reg_data[3] = 0;
    }
    else {
        shdisp_bdic_reg_data[0] = 0x78;
        shdisp_bdic_reg_data[1] = 0x20;
        shdisp_bdic_reg_data[2] = 0x03;
        shdisp_bdic_reg_data[3] = 0;
    }
#endif
    
    if((s_state_str.bdic_is_exist == SHDISP_BDIC_IS_EXIST)
    && (s_state_str.bdic_power_status == SHDISP_BDIC_PWR_STATUS_OFF)) {
        shdisp_bdic_PD_BDIC_init();
        s_state_str.bdic_power_status = SHDISP_BDIC_PWR_STATUS_STANDBY;
    }
    
    return;
}


/* ------------------------------------------------------------------------- */
/* shdisp_bdic_API_set_power_mode                                            */
/* ------------------------------------------------------------------------- */

int shdisp_bdic_API_set_power_mode(unsigned char dev_type, unsigned char onoff)
{

#ifdef SHDISP_SW_BDIC_POW_CTLLOG
    printk("[SHDISP] shdisp_bdic_API_set_power_mode(dev_type:%d, power:%d)\n", dev_type, onoff);
#endif /* SHDISP_SW_BDIC_POW_CTLLOG */
    if (dev_type >= NUM_SHDISP_BDIC_DEV_TYPE || 
        onoff    >= NUM_SHDISP_BDIC_DEV_PWR) {
        SHDISP_ERR("<INVALID_VALUE> dev_type(%d), onoff(%d).\n", dev_type, onoff);
        return SHDISP_RESULT_FAILURE;
    }

    if (s_state_str.bdic_power_status_tbl[dev_type] == onoff) {
#ifdef SHDISP_SW_BDIC_POW_CTLLOG
        printk("[SHDISP] bdic set power mode : same request.\n");
#endif /* SHDISP_SW_BDIC_POW_CTLLOG */
        return SHDISP_RESULT_SUCCESS;
    }

    if (onoff == SHDISP_BDIC_DEV_PWR_ON) {
        if (s_state_str.bdic_power_status == SHDISP_BDIC_PWR_STATUS_OFF) {
            shdisp_bdic_PD_BDIC_init();
            s_state_str.bdic_power_status = SHDISP_BDIC_PWR_STATUS_STANDBY;
#ifdef SHDISP_SW_BDIC_POW_CTLLOG
        printk("[SHDISP] bdic set power mode : OFF -> STANDBY.\n");
#endif /* SHDISP_SW_BDIC_POW_CTLLOG */
        }
        if (s_state_str.bdic_power_status == SHDISP_BDIC_PWR_STATUS_STANDBY) {
            shdisp_bdic_PD_BDIC_active();
            s_state_str.bdic_power_status = SHDISP_BDIC_PWR_STATUS_ACTIVE;
#ifdef SHDISP_SW_BDIC_POW_CTLLOG
        printk("[SHDISP] bdic set power mode : STANDBY -> ACTIVE.\n");
#endif /* SHDISP_SW_BDIC_POW_CTLLOG */
        }
    }

    switch (dev_type) {
    case SHDISP_BDIC_DEV_TYPE_LCD_BKL:
        if (onoff == SHDISP_BDIC_DEV_PWR_ON) {
            shdisp_bdic_LD_LCD_BKL_active();
        }
        else {
            shdisp_bdic_LD_LCD_BKL_standby();
        }
        break;

    case SHDISP_BDIC_DEV_TYPE_LCD_PWR:
        if (onoff == SHDISP_BDIC_DEV_PWR_ON) {
            shdisp_bdic_LD_LCD_PWR_active();
        }
        else {
            shdisp_bdic_LD_LCD_PWR_standby();
        }
        break;

    case SHDISP_BDIC_DEV_TYPE_TRI_LED:
        if (onoff == SHDISP_BDIC_DEV_PWR_ON) {
            shdisp_bdic_LD_TRI_LED_active();
        }
        else {
            shdisp_bdic_LD_TRI_LED_standby();
        }
        break;

    case SHDISP_BDIC_DEV_TYPE_TRI_LED_ANIME:
        if (onoff == SHDISP_BDIC_DEV_PWR_ON) {
            shdisp_bdic_LD_TRI_LED_ANIME_active();
        }
        else {
            shdisp_bdic_LD_TRI_LED_ANIME_standby();
        }
        break;
        
    case SHDISP_BDIC_DEV_TYPE_PHOTO_SENSOR_APP:
        if (onoff == SHDISP_BDIC_DEV_PWR_ON) {
            shdisp_bdic_LD_PHOTO_SENSOR_active(SHDISP_BDIC_DEVICE_PHOTO_SENSOR_APP);
        }
        else {
            shdisp_bdic_LD_PHOTO_SENSOR_standby(SHDISP_BDIC_DEVICE_PHOTO_SENSOR_APP);
        }
        break;
        
    case SHDISP_BDIC_DEV_TYPE_PHOTO_SENSOR_LUX:
        if (onoff == SHDISP_BDIC_DEV_PWR_ON) {
            shdisp_bdic_LD_PHOTO_SENSOR_active(SHDISP_BDIC_DEVICE_PHOTO_SENSOR_LUX);
        }
        else {
            shdisp_bdic_LD_PHOTO_SENSOR_standby(SHDISP_BDIC_DEVICE_PHOTO_SENSOR_LUX);
        }
        break;
        
    case SHDISP_BDIC_DEV_TYPE_PHOTO_SENSOR_BKL:
        if (onoff == SHDISP_BDIC_DEV_PWR_ON) {
            shdisp_bdic_LD_PHOTO_SENSOR_active(SHDISP_BDIC_DEVICE_PHOTO_SENSOR_BKL);
        }
        else {
            shdisp_bdic_LD_PHOTO_SENSOR_standby(SHDISP_BDIC_DEVICE_PHOTO_SENSOR_BKL);
        }
        break;
        
    case SHDISP_BDIC_DEV_TYPE_PROX_SENSOR:
        if (onoff == SHDISP_BDIC_DEV_PWR_ON) {
            shdisp_bdic_LD_PROX_SENSOR_active(SHDISP_BDIC_DEVICE_PROX_SENSOR);
        }
        else {
            shdisp_bdic_LD_PROX_SENSOR_standby(SHDISP_BDIC_DEVICE_PROX_SENSOR);
        }
        break;
        
    default:
        break;
    }

    s_state_str.bdic_power_status_tbl[dev_type] = onoff;

    if ((s_state_str.bdic_power_status_tbl[SHDISP_BDIC_DEV_TYPE_LCD_BKL]            == SHDISP_BDIC_DEV_PWR_OFF) &&
        (s_state_str.bdic_power_status_tbl[SHDISP_BDIC_DEV_TYPE_LCD_PWR]            == SHDISP_BDIC_DEV_PWR_OFF) &&
        (s_state_str.bdic_power_status_tbl[SHDISP_BDIC_DEV_TYPE_TRI_LED]            == SHDISP_BDIC_DEV_PWR_OFF) &&
        (s_state_str.bdic_power_status_tbl[SHDISP_BDIC_DEV_TYPE_PHOTO_SENSOR_APP]   == SHDISP_BDIC_DEV_PWR_OFF) &&
        (s_state_str.bdic_power_status_tbl[SHDISP_BDIC_DEV_TYPE_PHOTO_SENSOR_LUX]   == SHDISP_BDIC_DEV_PWR_OFF) &&
        (s_state_str.bdic_power_status_tbl[SHDISP_BDIC_DEV_TYPE_PHOTO_SENSOR_BKL]   == SHDISP_BDIC_DEV_PWR_OFF) &&
        (s_state_str.bdic_power_status_tbl[SHDISP_BDIC_DEV_TYPE_PROX_SENSOR]        == SHDISP_BDIC_DEV_PWR_OFF)
                                                                                                               ) {

        if (s_state_str.bdic_power_status == SHDISP_BDIC_PWR_STATUS_ACTIVE) {
            shdisp_bdic_PD_BDIC_standby();

            if (s_state_str.bdic_power_status_tbl[SHDISP_BDIC_DEV_TYPE_TRI_LED_ANIME] == SHDISP_BDIC_DEV_PWR_OFF) {
                shdisp_bdic_IO_clr_bit_reg(BDIC_REG_LPOSC, 0x01);
                }

            s_state_str.bdic_power_status = SHDISP_BDIC_PWR_STATUS_STANDBY;
#ifdef SHDISP_SW_BDIC_POW_CTLLOG
            printk("[SHDISP] bdic set power mode : ACTIVE -> STANDBY.\n");
#endif /* SHDISP_SW_BDIC_POW_CTLLOG */
        }
    }

    return SHDISP_RESULT_SUCCESS;
}


/* ------------------------------------------------------------------------- */
/* shdisp_bdic_API_bdic_shutdown                                             */
/* ------------------------------------------------------------------------- */

void shdisp_bdic_API_bdic_shutdown(void)
{
    if (s_state_str.bdic_power_status_tbl[SHDISP_BDIC_DEV_TYPE_LCD_BKL] == SHDISP_BDIC_DEV_PWR_ON) {
        shdisp_bdic_API_set_power_mode(SHDISP_BDIC_DEV_TYPE_LCD_BKL, SHDISP_BDIC_DEV_PWR_OFF);
        s_state_str.bdic_power_resume_tbl[SHDISP_BDIC_DEV_TYPE_LCD_BKL] = SHDISP_BDIC_DEV_PWR_ON;
    }
    if (s_state_str.bdic_power_status_tbl[SHDISP_BDIC_DEV_TYPE_LCD_PWR] == SHDISP_BDIC_DEV_PWR_ON) {
        shdisp_bdic_API_set_power_mode(SHDISP_BDIC_DEV_TYPE_LCD_PWR, SHDISP_BDIC_DEV_PWR_OFF);
        s_state_str.bdic_power_resume_tbl[SHDISP_BDIC_DEV_TYPE_LCD_PWR] = SHDISP_BDIC_DEV_PWR_ON;
    }
    if (s_state_str.bdic_power_status_tbl[SHDISP_BDIC_DEV_TYPE_TRI_LED] == SHDISP_BDIC_DEV_PWR_ON) {
        shdisp_bdic_API_set_power_mode(SHDISP_BDIC_DEV_TYPE_TRI_LED, SHDISP_BDIC_DEV_PWR_OFF);
        s_state_str.bdic_power_resume_tbl[SHDISP_BDIC_DEV_TYPE_TRI_LED] = SHDISP_BDIC_DEV_PWR_ON;
    }
    if (s_state_str.bdic_power_status_tbl[SHDISP_BDIC_DEV_TYPE_PHOTO_SENSOR_APP] == SHDISP_BDIC_DEV_PWR_ON) {
        shdisp_bdic_API_set_power_mode(SHDISP_BDIC_DEV_TYPE_PHOTO_SENSOR_APP, SHDISP_BDIC_DEV_PWR_OFF);
        s_state_str.bdic_power_resume_tbl[SHDISP_BDIC_DEV_TYPE_PHOTO_SENSOR_APP] = SHDISP_BDIC_DEV_PWR_ON;
    }
    if (s_state_str.bdic_power_status_tbl[SHDISP_BDIC_DEV_TYPE_PHOTO_SENSOR_LUX] == SHDISP_BDIC_DEV_PWR_ON) {
        shdisp_bdic_API_set_power_mode(SHDISP_BDIC_DEV_TYPE_PHOTO_SENSOR_LUX, SHDISP_BDIC_DEV_PWR_OFF);
        s_state_str.bdic_power_resume_tbl[SHDISP_BDIC_DEV_TYPE_PHOTO_SENSOR_LUX] = SHDISP_BDIC_DEV_PWR_ON;
    }
    if (s_state_str.bdic_power_status_tbl[SHDISP_BDIC_DEV_TYPE_PHOTO_SENSOR_BKL] == SHDISP_BDIC_DEV_PWR_ON) {
        shdisp_bdic_API_set_power_mode(SHDISP_BDIC_DEV_TYPE_PHOTO_SENSOR_BKL, SHDISP_BDIC_DEV_PWR_OFF);
        s_state_str.bdic_power_resume_tbl[SHDISP_BDIC_DEV_TYPE_PHOTO_SENSOR_BKL] = SHDISP_BDIC_DEV_PWR_ON;
    }
    if (s_state_str.bdic_power_status_tbl[SHDISP_BDIC_DEV_TYPE_PROX_SENSOR] == SHDISP_BDIC_DEV_PWR_ON) {
        shdisp_bdic_API_set_power_mode(SHDISP_BDIC_DEV_TYPE_PROX_SENSOR, SHDISP_BDIC_DEV_PWR_OFF);
        s_state_str.bdic_power_resume_tbl[SHDISP_BDIC_DEV_TYPE_PROX_SENSOR] = SHDISP_BDIC_DEV_PWR_ON;
    }
    
    shdisp_SYS_set_Host_gpio(SHDISP_GPIO_NUM_BL_RST_N, SHDISP_GPIO_CTL_LOW);
    shdisp_SYS_delay_us(15000);
    
    s_state_str.bdic_power_status = SHDISP_BDIC_PWR_STATUS_OFF;
    
    return;
}


/* ------------------------------------------------------------------------- */
/* shdisp_bdic_API_set_prox_sensor_param                                     */
/* ------------------------------------------------------------------------- */

void shdisp_bdic_API_set_prox_sensor_param( struct shdisp_prox_params *prox_params)
{
    memcpy( &s_state_str.shdisp_bdic_prox_params, prox_params, sizeof( struct shdisp_prox_params));
}


/* ------------------------------------------------------------------------- */
/* shdisp_bdic_API_i2c_error_power_off                                       */
/* ------------------------------------------------------------------------- */

void shdisp_bdic_API_i2c_error_power_off(void)
{
    if (s_state_str.bdic_power_status_tbl[SHDISP_BDIC_DEV_TYPE_PHOTO_SENSOR_APP] == SHDISP_BDIC_DEV_PWR_ON) {
        shdisp_bdic_API_set_power_mode(SHDISP_BDIC_DEV_TYPE_PHOTO_SENSOR_APP, SHDISP_BDIC_DEV_PWR_OFF);
        s_state_str.bdic_power_resume_tbl[SHDISP_BDIC_DEV_TYPE_PHOTO_SENSOR_APP] = SHDISP_BDIC_DEV_PWR_ON;
    }
    if (s_state_str.bdic_power_status_tbl[SHDISP_BDIC_DEV_TYPE_PHOTO_SENSOR_LUX] == SHDISP_BDIC_DEV_PWR_ON) {
        shdisp_bdic_API_set_power_mode(SHDISP_BDIC_DEV_TYPE_PHOTO_SENSOR_LUX, SHDISP_BDIC_DEV_PWR_OFF);
        s_state_str.bdic_power_resume_tbl[SHDISP_BDIC_DEV_TYPE_PHOTO_SENSOR_LUX] = SHDISP_BDIC_DEV_PWR_ON;
    }
    if (s_state_str.bdic_power_status_tbl[SHDISP_BDIC_DEV_TYPE_PHOTO_SENSOR_BKL] == SHDISP_BDIC_DEV_PWR_ON) {
        shdisp_bdic_API_set_power_mode(SHDISP_BDIC_DEV_TYPE_PHOTO_SENSOR_BKL, SHDISP_BDIC_DEV_PWR_OFF);
        s_state_str.bdic_power_resume_tbl[SHDISP_BDIC_DEV_TYPE_PHOTO_SENSOR_BKL] = SHDISP_BDIC_DEV_PWR_ON;
    }
    if (s_state_str.bdic_power_status_tbl[SHDISP_BDIC_DEV_TYPE_PROX_SENSOR] == SHDISP_BDIC_DEV_PWR_ON) {
        shdisp_bdic_API_set_power_mode(SHDISP_BDIC_DEV_TYPE_PROX_SENSOR, SHDISP_BDIC_DEV_PWR_OFF);
        s_state_str.bdic_power_resume_tbl[SHDISP_BDIC_DEV_TYPE_PROX_SENSOR] = SHDISP_BDIC_DEV_PWR_ON;
    }
}


/* ------------------------------------------------------------------------- */
/* shdisp_bdic_API_i2c_error_power_recovery                                  */
/* ------------------------------------------------------------------------- */

int shdisp_bdic_API_i2c_error_power_recovery(void)
{
    if (s_state_str.bdic_power_resume_tbl[SHDISP_BDIC_DEV_TYPE_PHOTO_SENSOR_APP] == SHDISP_BDIC_DEV_PWR_ON) {
        shdisp_bdic_API_set_power_mode(SHDISP_BDIC_DEV_TYPE_PHOTO_SENSOR_APP, SHDISP_BDIC_DEV_PWR_ON);
        s_state_str.bdic_power_resume_tbl[SHDISP_BDIC_DEV_TYPE_PHOTO_SENSOR_APP] = SHDISP_BDIC_DEV_PWR_OFF;
    }
    if (s_state_str.bdic_power_resume_tbl[SHDISP_BDIC_DEV_TYPE_PHOTO_SENSOR_LUX] == SHDISP_BDIC_DEV_PWR_ON) {
        shdisp_bdic_API_set_power_mode(SHDISP_BDIC_DEV_TYPE_PHOTO_SENSOR_LUX, SHDISP_BDIC_DEV_PWR_ON);
        s_state_str.bdic_power_resume_tbl[SHDISP_BDIC_DEV_TYPE_PHOTO_SENSOR_LUX] = SHDISP_BDIC_DEV_PWR_OFF;
    }
    if (s_state_str.bdic_power_resume_tbl[SHDISP_BDIC_DEV_TYPE_PHOTO_SENSOR_BKL] == SHDISP_BDIC_DEV_PWR_ON) {
        shdisp_bdic_API_set_power_mode(SHDISP_BDIC_DEV_TYPE_PHOTO_SENSOR_BKL, SHDISP_BDIC_DEV_PWR_ON);
        s_state_str.bdic_power_resume_tbl[SHDISP_BDIC_DEV_TYPE_PHOTO_SENSOR_BKL] = SHDISP_BDIC_DEV_PWR_OFF;
    }
    if (s_state_str.bdic_power_resume_tbl[SHDISP_BDIC_DEV_TYPE_PROX_SENSOR] == SHDISP_BDIC_DEV_PWR_ON) {
        shdisp_bdic_API_set_power_mode(SHDISP_BDIC_DEV_TYPE_PROX_SENSOR, SHDISP_BDIC_DEV_PWR_ON);
        s_state_str.bdic_power_resume_tbl[SHDISP_BDIC_DEV_TYPE_PROX_SENSOR] = SHDISP_BDIC_DEV_PWR_OFF;
    }
    return SHDISP_RESULT_SUCCESS;
}


/* ------------------------------------------------------------------------- */
/* shdisp_bdic_API_check_sensor_param                                        */
/* ------------------------------------------------------------------------- */

void shdisp_bdic_API_check_sensor_param(struct shdisp_photo_sensor_adj *adj_in, struct shdisp_photo_sensor_adj *adj_out)
{
    struct shdisp_photo_sensor_adj tmp_adj;
    int err_flg=0;
    unsigned long chksum;
    unsigned long tmp_param1=0;
    unsigned long tmp_param2=0;
    
    memcpy(&tmp_adj, adj_in, sizeof(struct shdisp_photo_sensor_adj));
    
    if (tmp_adj.status != 0x90) {
        err_flg = 1;
#ifdef SHDISP_SW_BDIC_ADJUST_DATALOG
        printk("[SHDISP] shdisp_bdic_API_check_sensor_param : status check error.");
#endif /* SHDISP_SW_BDIC_ADJUST_DATALOG */
    }
    else if ((tmp_adj.als_adj0 < 0x0000) || (tmp_adj.als_adj0 > 0xFFFF)) {
        err_flg = 2;
#ifdef SHDISP_SW_BDIC_ADJUST_DATALOG
        printk("[SHDISP] shdisp_bdic_API_check_sensor_param : als_adj0 check error.");
#endif /* SHDISP_SW_BDIC_ADJUST_DATALOG */
    }
    else if ((tmp_adj.als_adj1 < 0x0000) || (tmp_adj.als_adj1 > 0xFFFF)) {
        err_flg = 3;
#ifdef SHDISP_SW_BDIC_ADJUST_DATALOG
        printk("[SHDISP] shdisp_bdic_API_check_sensor_param : als_adj1 check error.");
#endif /* SHDISP_SW_BDIC_ADJUST_DATALOG */
    }
    else if ((tmp_adj.als_shift < 0x00) || (tmp_adj.als_shift > 0x1F)) {
        err_flg = 4;
#ifdef SHDISP_SW_BDIC_ADJUST_DATALOG
        printk("[SHDISP] shdisp_bdic_API_check_sensor_param : als_shift check error.");
#endif /* SHDISP_SW_BDIC_ADJUST_DATALOG */
    }
    else if ((tmp_adj.clear_offset < 0x00) || (tmp_adj.clear_offset > 0xFF)) {
        err_flg = 5;
#ifdef SHDISP_SW_BDIC_ADJUST_DATALOG
        printk("[SHDISP] shdisp_bdic_API_check_sensor_param : clear_offset check error.");
#endif /* SHDISP_SW_BDIC_ADJUST_DATALOG */
    }
    else if ((tmp_adj.ir_offset < 0x00) || (tmp_adj.ir_offset > 0xFF)) {
        err_flg = 6;
#ifdef SHDISP_SW_BDIC_ADJUST_DATALOG
        printk("[SHDISP] shdisp_bdic_API_check_sensor_param : ir_offset check error.");
#endif /* SHDISP_SW_BDIC_ADJUST_DATALOG */
    }
    else if ((tmp_adj.adc_lclip < 0x00) || (tmp_adj.adc_lclip > 0x7F)) {
        err_flg = 7;
#ifdef SHDISP_SW_BDIC_ADJUST_DATALOG
        printk("[SHDISP] shdisp_bdic_API_check_sensor_param : adc_lclip check error.");
#endif /* SHDISP_SW_BDIC_ADJUST_DATALOG */
    }
    else if ((tmp_adj.key_backlight < 0x00) || (tmp_adj.key_backlight > 0xFF)) {
        err_flg = 8;
#ifdef SHDISP_SW_BDIC_ADJUST_DATALOG
        printk("[SHDISP] shdisp_bdic_API_check_sensor_param : key_backlight check error.");
#endif /* SHDISP_SW_BDIC_ADJUST_DATALOG */
    }
    else {
        chksum = (unsigned long)tmp_adj.status +  tmp_adj.als_adj0 + tmp_adj.als_adj1 + (unsigned long)tmp_adj.als_shift + 
                 (unsigned long)tmp_adj.clear_offset + (unsigned long)tmp_adj.ir_offset + (unsigned long)tmp_adj.adc_lclip + (unsigned long)tmp_adj.key_backlight;
        if (tmp_adj.chksum != chksum) {
            err_flg = 9;
#ifdef SHDISP_SW_BDIC_ADJUST_DATALOG
            printk("[SHDISP] shdisp_bdic_API_check_sensor_param : chksum check error.\n");
            printk("[SHDISP] chksum = 0x%08x\n", (unsigned int)tmp_adj.chksum);
            printk("[SHDISP] result = 0x%08x\n", (unsigned int)chksum);
            
            printk("[SHDISP] status        = 0x%02x\n", tmp_adj.status);
            printk("[SHDISP] als_adj0      = 0x%04x\n", (unsigned short)tmp_adj.als_adj0);
            printk("[SHDISP] als_adj1      = 0x%04x\n", (unsigned short)tmp_adj.als_adj1);
            printk("[SHDISP] als_shift     = 0x%02x\n", tmp_adj.als_shift);
            printk("[SHDISP] clear_offset  = 0x%02x\n", tmp_adj.clear_offset);
            printk("[SHDISP] ir_offset     = 0x%02x\n", tmp_adj.ir_offset);
            printk("[SHDISP] adc_lclip     = 0x%02x\n", tmp_adj.adc_lclip);
            printk("[SHDISP] key_backlight = 0x%02x\n", tmp_adj.key_backlight);
#endif /* SHDISP_SW_BDIC_ADJUST_DATALOG */
        }
    }
    
    if (err_flg == 0) {
        memcpy(adj_out, &tmp_adj, sizeof(struct shdisp_photo_sensor_adj));
    }
    else {
        tmp_adj.status = (unsigned char)err_flg;
        tmp_param1 = ((unsigned long)BDIC_REG_ALS_ADJ0_H_DEFAULT << 8);
        tmp_param2 = (unsigned long)BDIC_REG_ALS_ADJ0_L_DEFAULT;
        tmp_adj.als_adj0     = tmp_param1 | tmp_param2;
        tmp_param1 = ((unsigned long)BDIC_REG_ALS_ADJ1_H_DEFAULT << 8);
        tmp_param2 = (unsigned long)BDIC_REG_ALS_ADJ1_L_DEFAULT;
        tmp_adj.als_adj1     = tmp_param1 | tmp_param2;
        tmp_adj.als_shift    = BDIC_REG_ALS_SHIFT_DEFAULT;
        tmp_adj.clear_offset = BDIC_REG_CLEAR_OFFSET_DEFAULT;
        tmp_adj.ir_offset    = BDIC_REG_IR_OFFSET_DEFAULT;
        
        memcpy(adj_out, &tmp_adj, sizeof(struct shdisp_photo_sensor_adj));
    }
    
    return;
}


/* ------------------------------------------------------------------------- */
/* shdisp_bdic_API_check_bkl_adj_param                                       */
/* ------------------------------------------------------------------------- */

void shdisp_bdic_API_check_bkl_adj_param(unsigned short dma_status, int *bkl_adjust)
{
    unsigned char dma_status_tmp;
    
    dma_status_tmp = (unsigned char)(dma_status & 0x0003);
    
    switch (dma_status_tmp) {
    case 0x00:
        *bkl_adjust = SHDISP_MAIN_BKL_ADJ_RETRY0;
        break;
    case 0x01:
        *bkl_adjust = SHDISP_MAIN_BKL_ADJ_RETRY1;
        break;
    case 0x02:
        *bkl_adjust = SHDISP_MAIN_BKL_ADJ_RETRY2;
        break;
    case 0x03:
        *bkl_adjust = SHDISP_MAIN_BKL_ADJ_RETRY3;
        break;
    default:
        *bkl_adjust = SHDISP_MAIN_BKL_ADJ_RETRY0;
        break;
    }
    
    return;
}


/* ------------------------------------------------------------------------- */
/* shdisp_bdic_API_LCD_release_hw_reset                                      */
/* ------------------------------------------------------------------------- */

void shdisp_bdic_API_LCD_release_hw_reset(void)
{
    shdisp_bdic_LD_GPIO_control(SHDISP_BDIC_GPIO_COG_RESET, SHDISP_BDIC_GPIO_HIGH);

    return;
}


/* ------------------------------------------------------------------------- */
/* shdisp_bdic_API_LCD_set_hw_reset                                          */
/* ------------------------------------------------------------------------- */

void shdisp_bdic_API_LCD_set_hw_reset(void)
{
    shdisp_bdic_LD_GPIO_control(SHDISP_BDIC_GPIO_COG_RESET, SHDISP_BDIC_GPIO_LOW);

    return;
}


/* ------------------------------------------------------------------------- */
/* shdisp_bdic_API_LCD_power_on                                              */
/* ------------------------------------------------------------------------- */

void shdisp_bdic_API_LCD_power_on(void)
{
    shdisp_bdic_LD_LCD_PWR_on();

    return;
}


/* ------------------------------------------------------------------------- */
/* shdisp_bdic_API_LCD_power_off                                             */
/* ------------------------------------------------------------------------- */

void shdisp_bdic_API_LCD_power_off(void)
{
    shdisp_bdic_LD_LCD_PWR_off();

    return;
}


/* ------------------------------------------------------------------------- */
/* shdisp_bdic_API_LCD_m_power_on                                            */
/* ------------------------------------------------------------------------- */

void shdisp_bdic_API_LCD_m_power_on(void)
{
    shdisp_bdic_LD_LCD_M_PWR_on();

    return;
}


/* ------------------------------------------------------------------------- */
/* shdisp_bdic_API_LCD_m_power_off                                           */
/* ------------------------------------------------------------------------- */

void shdisp_bdic_API_LCD_m_power_off(void)
{
    shdisp_bdic_LD_LCD_M_PWR_off();

    return;
}


/* ------------------------------------------------------------------------- */
/* shdisp_bdic_API_LCD_vo2_on                                                */
/* ------------------------------------------------------------------------- */

void shdisp_bdic_API_LCD_vo2_on(void)
{
    shdisp_bdic_LD_LCD_VO2_ON();
    
    return;
}


/* ------------------------------------------------------------------------- */
/* shdisp_bdic_API_LCD_vo2_off                                               */
/* ------------------------------------------------------------------------- */

void shdisp_bdic_API_LCD_vo2_off(void)
{
    shdisp_bdic_LD_LCD_VO2_OFF();
    
    return;
}


/* ------------------------------------------------------------------------- */
/* shdisp_bdic_API_LCD_BKL_off                                               */
/* ------------------------------------------------------------------------- */

void shdisp_bdic_API_LCD_BKL_off(void)
{
    shdisp_bdic_LD_LCD_BKL_off();

    return;
}


/* ------------------------------------------------------------------------- */
/* shdisp_bdic_API_LCD_BKL_fix_on                                            */
/* ------------------------------------------------------------------------- */

void shdisp_bdic_API_LCD_BKL_fix_on(int param)
{
    shdisp_bdic_LD_LCD_BKL_fix_on(param);

    return;
}


/* ------------------------------------------------------------------------- */
/* shdisp_bdic_API_LCD_BKL_auto_on                                           */
/* ------------------------------------------------------------------------- */

void shdisp_bdic_API_LCD_BKL_auto_on(int param)
{
    shdisp_bdic_LD_LCD_BKL_auto_on(param);

    return;
}


/* ------------------------------------------------------------------------- */
/* shdisp_bdic_API_LCD_BKL_get_param                                         */
/* ------------------------------------------------------------------------- */

void shdisp_bdic_API_LCD_BKL_get_param(unsigned long int* param)
{
    int mode = 0;

    switch (shdisp_bdic_bkl_mode) {
    case SHDISP_BDIC_BKL_MODE_FIX:
        shdisp_bdic_LD_LCD_BKL_get_mode(&mode);
        
        if (shdisp_bdic_dtv == SHDISP_BDIC_BKL_DTV_ON) {
            *param = shdisp_main_dtv_bkl_tbl[shdisp_bdic_bkl_param][mode] + shdisp_main_bkl_adj_tbl[s_state_str.bkl_adjust_val];
        }
        else {
            *param = shdisp_main_bkl_tbl[shdisp_bdic_bkl_param][mode] + shdisp_main_bkl_adj_tbl[s_state_str.bkl_adjust_val];
        }
        break;

    case SHDISP_BDIC_BKL_MODE_AUTO:
        *param = 0x100;
        break;
        
    default:
        *param = 0;
        break;
    }

    return;
}


/* ------------------------------------------------------------------------- */
/* shdisp_bdic_API_LCD_BKL_set_request                                       */
/* ------------------------------------------------------------------------- */
void shdisp_bdic_API_LCD_BKL_set_request(int type, struct shdisp_main_bkl_ctl *tmp)
{
    shdisp_bkl_priority_table[type].mode  = tmp->mode;
    shdisp_bkl_priority_table[type].param = tmp->param;
    
    shdisp_bdic_bkl_mode   = tmp->mode;
    shdisp_bdic_bkl_param  = tmp->param;
}


/* ------------------------------------------------------------------------- */
/* shdisp_bdic_API_TRI_LED_set_request                                       */
/* ------------------------------------------------------------------------- */
void shdisp_bdic_API_TRI_LED_set_request(struct shdisp_tri_led *tmp)
{
    int color = 0x00;

    color = (tmp->blue << 2) | (tmp->green << 1) | tmp->red;

    shdisp_bdic_tri_led_mode     = tmp->led_mode;
    shdisp_bdic_tri_led_color    = color;
    shdisp_bdic_tri_led_ontime   = tmp->ontime;
    shdisp_bdic_tri_led_interval = tmp->interval;
}


/* ------------------------------------------------------------------------- */
/* shdisp_bdic_API_LCD_BKL_get_request                                       */
/* ------------------------------------------------------------------------- */
void shdisp_bdic_API_LCD_BKL_get_request(int type, struct shdisp_main_bkl_ctl *tmp, struct shdisp_main_bkl_ctl *req)
{
    shdisp_bkl_priority_table[type].mode  = tmp->mode;
    shdisp_bkl_priority_table[type].param = tmp->param;
    
    
    if (shdisp_bkl_priority_table[SHDISP_MAIN_BKL_DEV_TYPE_APP].mode == SHDISP_MAIN_BKL_MODE_OFF) {
        req->mode  = shdisp_bkl_priority_table[SHDISP_MAIN_BKL_DEV_TYPE_APP].mode;
        req->param = shdisp_bkl_priority_table[SHDISP_MAIN_BKL_DEV_TYPE_APP].param;
    }
    else if (((shdisp_bkl_priority_table[SHDISP_MAIN_BKL_DEV_TYPE_APP].mode == SHDISP_MAIN_BKL_MODE_FIX) &&
                 (shdisp_bkl_priority_table[SHDISP_MAIN_BKL_DEV_TYPE_APP].param == SHDISP_MAIN_BKL_PARAM_1))) {
        req->mode  = shdisp_bkl_priority_table[SHDISP_MAIN_BKL_DEV_TYPE_APP].mode;
        req->param = shdisp_bkl_priority_table[SHDISP_MAIN_BKL_DEV_TYPE_APP].param;
    }
    else if (shdisp_bkl_priority_table[SHDISP_MAIN_BKL_DEV_TYPE_APP_AUTO].param == SHDISP_MAIN_BKL_PARAM_1) {
        req->mode  = shdisp_bkl_priority_table[SHDISP_MAIN_BKL_DEV_TYPE_APP_AUTO].mode;
        req->param = shdisp_bkl_priority_table[SHDISP_MAIN_BKL_DEV_TYPE_APP_AUTO].param;
    }
    else {
        req->mode  = shdisp_bkl_priority_table[SHDISP_MAIN_BKL_DEV_TYPE_APP].mode;
        req->param = shdisp_bkl_priority_table[SHDISP_MAIN_BKL_DEV_TYPE_APP].param;
    }
    
    return;
}


/* ------------------------------------------------------------------------- */
/* shdisp_bdic_API_LCD_BKL_pwm_disable                                       */
/* ------------------------------------------------------------------------- */

void shdisp_bdic_API_LCD_BKL_pwm_disable(void)
{
    shdisp_bdic_LD_LCD_BKL_pwm_disable();
    
    return;
}


/* ------------------------------------------------------------------------- */
/* shdisp_bdic_API_LCD_BKL_pwm_enable                                        */
/* ------------------------------------------------------------------------- */

void shdisp_bdic_API_LCD_BKL_pwm_enable(void)
{
    shdisp_bdic_LD_LCD_BKL_pwm_enable();
    
    return;
}


/* ------------------------------------------------------------------------- */
/* shdisp_bdic_API_LCD_BKL_dtv_on                                            */
/* ------------------------------------------------------------------------- */

void shdisp_bdic_API_LCD_BKL_dtv_on(void)
{
    shdisp_bdic_LD_LCD_BKL_dtv_on();
    
    return;
}


/* ------------------------------------------------------------------------- */
/* shdisp_bdic_API_LCD_BKL_dtv_off                                           */
/* ------------------------------------------------------------------------- */

void shdisp_bdic_API_LCD_BKL_dtv_off(void)
{
    shdisp_bdic_LD_LCD_BKL_dtv_off();
    
    return;
}


/* ------------------------------------------------------------------------- */
/* shdisp_bdic_API_LCD_BKL_emg_on                                            */
/* ------------------------------------------------------------------------- */

void shdisp_bdic_API_LCD_BKL_emg_on(void)
{
    shdisp_bdic_LD_LCD_BKL_emg_on();
    
    return;
}


/* ------------------------------------------------------------------------- */
/* shdisp_bdic_API_LCD_BKL_emg_off                                           */
/* ------------------------------------------------------------------------- */

void shdisp_bdic_API_LCD_BKL_emg_off(void)
{
    shdisp_bdic_LD_LCD_BKL_emg_off();
    
    return;
}


/* ------------------------------------------------------------------------- */
/* shdisp_bdic_API_LCD_BKL_eco_on                                            */
/* ------------------------------------------------------------------------- */

void shdisp_bdic_API_LCD_BKL_eco_on(void)
{
    shdisp_bdic_LD_LCD_BKL_eco_on();
    
    return;
}


/* ------------------------------------------------------------------------- */
/* shdisp_bdic_API_LCD_BKL_emg_off                                           */
/* ------------------------------------------------------------------------- */

void shdisp_bdic_API_LCD_BKL_eco_off(void)
{
    shdisp_bdic_LD_LCD_BKL_eco_off();
    
    return;
}


/* ------------------------------------------------------------------------- */
/* shdisp_bdic_API_LCD_BKL_chg_on                                            */
/* ------------------------------------------------------------------------- */

void shdisp_bdic_API_LCD_BKL_chg_on(void)
{
    shdisp_bdic_LD_LCD_BKL_chg_on();
    
    return;
}


/* ------------------------------------------------------------------------- */
/* shdisp_bdic_API_LCD_BKL_chg_off                                           */
/* ------------------------------------------------------------------------- */

void shdisp_bdic_API_LCD_BKL_chg_off(void)
{
    shdisp_bdic_LD_LCD_BKL_chg_off();
    
    return;
}


/* ------------------------------------------------------------------------- */
/* shdisp_bdic_API_TRI_LED_off                                               */
/* ------------------------------------------------------------------------- */

void shdisp_bdic_API_TRI_LED_off(void)
{
    shdisp_bdic_LD_TRI_LED_off();
    
    return;
}


/* ------------------------------------------------------------------------- */
/* shdisp_bdic_API_TRI_LED_normal_on                                         */
/* ------------------------------------------------------------------------- */

void shdisp_bdic_API_TRI_LED_normal_on(unsigned char color)
{
    shdisp_bdic_LD_TRI_LED_normal_on(color);

    return;
}


/* ------------------------------------------------------------------------- */
/* shdisp_bdic_API_TRI_LED_blink_on                                          */
/* ------------------------------------------------------------------------- */

void shdisp_bdic_API_TRI_LED_blink_on(unsigned char color, int ontime, int interval)
{
    shdisp_bdic_LD_TRI_LED_blink_on(color, ontime, interval);

    return;
}


/* ------------------------------------------------------------------------- */
/* shdisp_bdic_API_TRI_LED_firefly_on                                        */
/* ------------------------------------------------------------------------- */

void shdisp_bdic_API_TRI_LED_firefly_on(unsigned char color, int ontime, int interval)
{
    shdisp_bdic_LD_TRI_LED_firefly_on(color, ontime, interval);

    return;
}


/* ------------------------------------------------------------------------- */
/* shdisp_bdic_API_PHOTO_SENSOR_get_val                                      */
/* ------------------------------------------------------------------------- */

int shdisp_bdic_API_PHOTO_SENSOR_get_val(unsigned short *value)
{
    int ret;
    
    ret = shdisp_bdic_LD_PHOTO_SENSOR_get_val(value);
    
    return ret;
}


/* ------------------------------------------------------------------------- */
/* shdisp_bdic_API_PHOTO_SENSOR_get_lux                                      */
/* ------------------------------------------------------------------------- */

int  shdisp_bdic_API_PHOTO_SENSOR_get_lux(unsigned short *value, unsigned long *lux)
{
    int ret;
    
    ret = shdisp_bdic_LD_PHOTO_SENSOR_get_lux(value, lux);
    
    return ret;
}


/* ------------------------------------------------------------------------- */
/* shdisp_bdic_API_PHOTO_SENSOR_lux_change_ind                               */
/* ------------------------------------------------------------------------- */

int  shdisp_bdic_API_PHOTO_SENSOR_lux_change_ind(int *mode)
{
    if(s_state_str.bdic_main_bkl_opt_mode_ado == SHDISP_BDIC_MAIN_BKL_OPT_LOW ){
        *mode = SHDISP_LUX_MODE_LOW;
    }
    else {
        *mode = SHDISP_LUX_MODE_HIGH;
    }

    return SHDISP_RESULT_SUCCESS;
}


/* ------------------------------------------------------------------------- */
/* shdisp_bdic_API_PHOTO_SENSOR_lux_judge                                    */
/* ------------------------------------------------------------------------- */

int  shdisp_bdic_API_PHOTO_SENSOR_lux_judge(void)
{
    int           ret;
    unsigned char val;
    unsigned short ADO_dat;
    
    shdisp_bdic_IO_read_reg(BDIC_REG_ADO_INDEX, &val);
    val &= 0x1F;
    shdisp_bdic_PD_REG_ADO_get_opt(&ADO_dat);

    if( s_state_str.bdic_main_bkl_opt_mode_output == SHDISP_BDIC_MAIN_BKL_OPT_LOW ){
        if(val >= s_state_str.shdisp_lux_change_level1 ){
#ifdef SHDISP_SW_BDIC_IRQ_LOG
            printk("[SHDISP DBG] lux_judge: ADO_INDEX(=%d) >= level1(=%d) (ADO=0x%04X) -> JUDGE_OUT\n", (int)val, (int)s_state_str.shdisp_lux_change_level1, ADO_dat);
#endif
            shdisp_photo_sensor_change_range( SHDISP_BDIC_MAIN_BKL_OPT_HIGH );

            shdisp_bdic_IO_clr_bit_reg( BDIC_REG_GIMR2, 0x01 );
            shdisp_bdic_IO_clr_bit_reg( BDIC_REG_GIMF2, 0x01 );
            shdisp_bdic_IO_clr_bit_reg( BDIC_REG_GIMR3, 0x20 );

#if defined(CONFIG_MACH_DECKARD_AS70)
            shdisp_bdic_PD_REG_OPT_temp_set(0x8E);
#else
            shdisp_bdic_PD_REG_OPT_temp_set(0x7D);
#endif

            shdisp_bdic_IO_clr_bit_reg( BDIC_REG_ADO_SYS, 0x01 );
            shdisp_bdic_IO_write_reg(BDIC_REG_ADOL, 0x80);
            shdisp_bdic_IO_write_reg(BDIC_REG_ADOH, 0x05);
            shdisp_bdic_IO_set_bit_reg(BDIC_REG_SENSOR, 0x01);
            shdisp_bdic_PD_REG_OPT_set_value(SHDISP_BDIC_MAIN_BKL_OPT_HIGH);

            shdisp_bdic_IO_set_bit_reg( BDIC_REG_GIMF2, 0x01 );
            shdisp_bdic_IO_set_bit_reg( BDIC_REG_GIMR3, 0x20 );

            ret = SHDISP_BDIC_LUX_JUDGE_OUT;
        }
        else{
#ifdef SHDISP_SW_BDIC_IRQ_LOG
            printk("[SHDISP DBG] lux_judge: ADO_INDEX(=%d) < level1(=%d) (ADO=0x%04X) -> JUDGE_IN_CONTI\n", (int)val, (int)s_state_str.shdisp_lux_change_level1, ADO_dat);
#endif
            shdisp_bdic_IO_set_bit_reg( BDIC_REG_GIMR2, 0x01 );
            
            ret = SHDISP_BDIC_LUX_JUDGE_IN_CONTI;
        }
    }
    else{
        if(val < s_state_str.shdisp_lux_change_level2 ){
#ifdef SHDISP_SW_BDIC_IRQ_LOG
            printk("[SHDISP DBG] lux_judge: ADO_INDEX(=%d) < level2(=%d) (ADO=0x%04X) -> JUDGE_IN\n", (int)val, (int)s_state_str.shdisp_lux_change_level2, ADO_dat);
#endif
            shdisp_photo_sensor_change_range( SHDISP_BDIC_MAIN_BKL_OPT_LOW );

            shdisp_bdic_IO_clr_bit_reg( BDIC_REG_GIMR2, 0x01 );
            shdisp_bdic_IO_clr_bit_reg( BDIC_REG_GIMF2, 0x01 );
            shdisp_bdic_IO_clr_bit_reg( BDIC_REG_GIMR3, 0x20 );

#if defined(CONFIG_MACH_DECKARD_AS70)
            shdisp_bdic_PD_REG_OPT_temp_set(0x4F);
#else
            shdisp_bdic_PD_REG_OPT_temp_set(0x78);
#endif

            shdisp_bdic_IO_write_reg(BDIC_REG_ADOL, 0x80);
#if defined(CONFIG_MACH_DECKARD_AS70)
            shdisp_bdic_IO_write_reg(BDIC_REG_ADOH, 0x40);
#elif defined(CONFIG_MACH_TOR) || defined(CONFIG_MACH_MNB) || defined(CONFIG_MACH_ARS)
            shdisp_bdic_IO_write_reg(BDIC_REG_ADOH, 0x08);
#elif defined(CONFIG_MACH_DECKARD_GP4)
            shdisp_bdic_IO_write_reg(BDIC_REG_ADOH, 0x10);
#else
            shdisp_bdic_IO_write_reg(BDIC_REG_ADOH, 0x04);
#endif
            shdisp_bdic_IO_set_bit_reg(BDIC_REG_SENSOR, 0x01);

            shdisp_bdic_IO_set_bit_reg( BDIC_REG_ADO_SYS, 0x01 );

            shdisp_bdic_PD_REG_OPT_set_value(SHDISP_BDIC_MAIN_BKL_OPT_LOW);

            shdisp_bdic_IO_set_bit_reg( BDIC_REG_GIMR2, 0x01 );
            
            ret = SHDISP_BDIC_LUX_JUDGE_IN;
        }
        else{
#ifdef SHDISP_SW_BDIC_IRQ_LOG
            printk("[SHDISP DBG] lux_judge: ADO_INDEX(=%d) >= level2(=%d) (ADO=0x%04X) -> JUDGE_OUT_CONTI\n", (int)val, (int)s_state_str.shdisp_lux_change_level2, ADO_dat);
#endif
            shdisp_bdic_IO_set_bit_reg( BDIC_REG_GIMF2, 0x01 );
            shdisp_bdic_IO_set_bit_reg( BDIC_REG_GIMR3, 0x20 );
            
            ret = SHDISP_BDIC_LUX_JUDGE_OUT_CONTI;
        }
    }
    return ret;
}


/* ------------------------------------------------------------------------- */
/* shdisp_bdic_API_change_level_lut                                          */
/* ------------------------------------------------------------------------- */

int shdisp_bdic_API_change_level_lut(void)
{

    int level_lut = SHDISP_MAIN_DISP_CABC_LUT0;

    unsigned char val;
    
    shdisp_bdic_IO_read_reg(BDIC_REG_ADO_INDEX, &val);
    val &= 0x1F;

    if( s_state_str.bdic_main_bkl_opt_mode_output == SHDISP_BDIC_MAIN_BKL_OPT_LOW ){
        level_lut = SHDISP_MAIN_DISP_CABC_LUT0;
    }
    else if(val >= CABC_LUX_LEVEL_LUT4_5)
    {
        level_lut = SHDISP_MAIN_DISP_CABC_LUT5;
    }
    else if(val >= CABC_LUX_LEVEL_LUT3_4)
    {
        level_lut = SHDISP_MAIN_DISP_CABC_LUT4;
    }
    else if(val >= CABC_LUX_LEVEL_LUT2_3)
    {
        level_lut = SHDISP_MAIN_DISP_CABC_LUT3;
    }
    else if(val >= CABC_LUX_LEVEL_LUT1_2)
    {
        level_lut = SHDISP_MAIN_DISP_CABC_LUT2;
    }
    else if(val >= CABC_LUX_LEVEL_LUT0_1)
    {
        level_lut = SHDISP_MAIN_DISP_CABC_LUT1;
    }
    else
    {
        level_lut = SHDISP_MAIN_DISP_CABC_LUT0;
    }

    return level_lut;
}

/* ------------------------------------------------------------------------- */
/* shdisp_bdic_PHOTO_SENSOR_lux_init                                         */
/* ------------------------------------------------------------------------- */

static int  shdisp_bdic_PHOTO_SENSOR_lux_init(void)
{
    int           ret= SHDISP_RESULT_SUCCESS;

#ifdef SHDISP_SW_BDIC_IRQ_LOG
    printk("[SHDISP DBG] shdisp_bdic_PHOTO_SENSOR_lux_init : start\n");
#endif
    
    shdisp_bdic_API_PHOTO_lux_timer( SHDISP_BDIC_PHOTO_LUX_TIMER_OFF );

    shdisp_photo_sensor_change_range( SHDISP_BDIC_MAIN_BKL_OPT_LOW );

    shdisp_bdic_IO_clr_bit_reg( BDIC_REG_GIMR2, 0x01 );
    shdisp_bdic_IO_clr_bit_reg( BDIC_REG_GIMF2, 0x03 );
    shdisp_bdic_IO_clr_bit_reg( BDIC_REG_GIMR3, 0x20 );

    shdisp_bdic_IO_set_bit_reg( BDIC_REG_ADO_SYS, 0x01 );

    shdisp_bdic_PD_REG_OPT_set_value(SHDISP_BDIC_MAIN_BKL_OPT_LOW);
    
    shdisp_bdic_IO_set_bit_reg( BDIC_REG_GIMR2, 0x01 );
    shdisp_bdic_IO_set_bit_reg( BDIC_REG_GIMR3, 0x08 );
    if (SHDISP_INT_ENABLE_GFAC & SHDISP_BDIC_INT_GFAC_PS2) {
        shdisp_bdic_IO_set_bit_reg( BDIC_REG_GIMF2, 0x02 );
    }

    shdisp_bdic_API_PHOTO_lux_timer( SHDISP_BDIC_PHOTO_LUX_TIMER_ON );

    shdisp_bdic_API_SENSOR_start(SHDISP_LUX_MODE_LOW);

    return ret;
}


/* ------------------------------------------------------------------------- */
/* shdisp_bdic_API_i2c_transfer                                              */
/* ------------------------------------------------------------------------- */

int shdisp_bdic_API_i2c_transfer(struct shdisp_bdic_i2c_msg *msg)
{
    int ret = SHDISP_RESULT_SUCCESS;
    unsigned char value = 0x00;
    int i2c_rtimer_restart_flg = 0;
    
    if (msg == NULL) {
        SHDISP_ERR("<NULL_POINTER> msg.\n");
        return SHDISP_RESULT_FAILURE;
    }
    
    if (msg->mode >= NUM_SHDISP_BDIC_I2C_M) {
        SHDISP_ERR("<INVALID_VALUE> msg->mode(%d).\n", msg->mode);
        return SHDISP_RESULT_FAILURE;
    }
    
    shdisp_bdic_IO_read_reg(BDIC_REG_I2C_START, &value);
    
    if (value & SHDISP_BDIC_I2C_R_START) {
        SHDISP_ERR("<OTHER> i2c read start.\n");
        return SHDISP_RESULT_FAILURE;
    }
    
    if (value & SHDISP_BDIC_I2C_R_TIMRE_ST) {
#ifdef SHDISP_SW_BDIC_I2C_RWLOG
        printk("[SHDISP] bdic i2c transfer : stop i2c rtimer.\n");
#endif
        i2c_rtimer_restart_flg = 1;
        shdisp_bdic_IO_clr_bit_reg(BDIC_REG_I2C_START, 0x10);
        shdisp_bdic_IO_set_bit_reg(BDIC_REG_ALS_DATA0_SET, SHDISP_BDIC_I2C_ADO_UPDATE_MASK);
        shdisp_SYS_delay_us(1000);
    }
    
    switch (msg->mode) {
    case SHDISP_BDIC_I2C_M_W:
        ret = shdisp_bdic_LD_i2c_write(msg);
        break;
    case SHDISP_BDIC_I2C_M_R:
        ret = shdisp_bdic_LD_i2c_read_mode0(msg);
        break;
    case SHDISP_BDIC_I2C_M_R_MODE1:
        ret = shdisp_bdic_LD_i2c_read_mode1(msg);
        break;
    case SHDISP_BDIC_I2C_M_R_MODE2:
        ret = shdisp_bdic_LD_i2c_read_mode2(msg);
        break;
    case SHDISP_BDIC_I2C_M_R_MODE3:
        ret = shdisp_bdic_LD_i2c_read_mode3(msg);
        break;
    default:
        SHDISP_ERR("<INVALID_VALUE> msg->mode(%d).\n", msg->mode);
        ret = SHDISP_RESULT_FAILURE;
        break;
    }
    
    if (i2c_rtimer_restart_flg == 1) {
#ifdef SHDISP_SW_BDIC_I2C_RWLOG
        printk("[SHDISP] bdic i2c transfer : restart i2c rtimer.\n");
#endif
        shdisp_bdic_IO_clr_bit_reg(BDIC_REG_ALS_DATA0_SET, SHDISP_BDIC_I2C_ADO_UPDATE_MASK);
        shdisp_bdic_IO_write_reg(BDIC_REG_I2C_DATA6, 0x0C);
        if (SHDISP_INT_ENABLE_GFAC & SHDISP_BDIC_INT_GFAC_PS2) {
            shdisp_bdic_IO_write_reg(BDIC_REG_I2C_SET, 0x50);
        }
        else {
            shdisp_bdic_IO_write_reg(BDIC_REG_I2C_SET, 0x30);
        }
        shdisp_bdic_IO_set_bit_reg(BDIC_REG_I2C_START, 0x10);
    }
    else {
        if (SHDISP_INT_ENABLE_GFAC & SHDISP_BDIC_INT_GFAC_PS2) {
            shdisp_bdic_IO_write_reg(BDIC_REG_I2C_SET, 0x50);
        }
    }
    
    return ret;
}


/* ------------------------------------------------------------------------- */
/* shdisp_bdic_API_DIAG_write_reg                                            */
/* ------------------------------------------------------------------------- */

void shdisp_bdic_API_DIAG_write_reg(unsigned char reg, unsigned char val)
{
    shdisp_bdic_IO_write_reg(reg, val);

    return;
}


/* ------------------------------------------------------------------------- */
/* shdisp_bdic_API_DIAG_read_reg                                             */
/* ------------------------------------------------------------------------- */

void shdisp_bdic_API_DIAG_read_reg(unsigned char reg, unsigned char *val)
{
    shdisp_bdic_IO_read_reg(reg, val);

    return;
}


/* ------------------------------------------------------------------------- */
/* shdisp_bdic_API_DIAG_multi_read_reg                                       */
/* ------------------------------------------------------------------------- */

int shdisp_bdic_API_DIAG_multi_read_reg(unsigned char reg, unsigned char *val, int size)
{
    int ret = 0;
    
    ret = shdisp_bdic_IO_multi_read_reg(reg, val, size);
    
    return ret;
}


/* ------------------------------------------------------------------------- */
/* shdisp_bdic_API_RECOVERY_check_restoration                                */
/* ------------------------------------------------------------------------- */

int shdisp_bdic_API_RECOVERY_check_restoration(void)
{
    unsigned char dummy=0;
    
    shdisp_bdic_IO_read_reg(BDIC_REG_GINF3, &dummy);
    
    if (dummy & 0x04) {
        return SHDISP_RESULT_SUCCESS;
    }
    else {
        return SHDISP_RESULT_FAILURE;
    }
}


/* ------------------------------------------------------------------------- */
/* shdisp_bdic_API_RECOVERY_check_bdic_practical                             */
/* ------------------------------------------------------------------------- */
int  shdisp_bdic_API_RECOVERY_check_bdic_practical(void)
{
    if (s_state_str.bdic_power_status_tbl[SHDISP_BDIC_DEV_TYPE_PROX_SENSOR] == SHDISP_BDIC_DEV_PWR_ON) {
        return SHDISP_RESULT_FAILURE;
    }
    
    return SHDISP_RESULT_SUCCESS;
}


/* ------------------------------------------------------------------------- */
/* shdisp_bdic_API_RECOVERY_resume_bdic_status                               */
/* ------------------------------------------------------------------------- */

int  shdisp_bdic_API_RECOVERY_resume_bdic_status(void)
{
    shdisp_SYS_set_Host_gpio(SHDISP_GPIO_NUM_BL_RST_N, SHDISP_GPIO_CTL_HIGH);
    shdisp_SYS_delay_us(1000);
    
    if (s_state_str.bdic_power_resume_tbl[SHDISP_BDIC_DEV_TYPE_LCD_BKL] == SHDISP_BDIC_DEV_PWR_ON) {
        shdisp_bdic_API_set_power_mode(SHDISP_BDIC_DEV_TYPE_LCD_BKL, SHDISP_BDIC_DEV_PWR_ON);
        s_state_str.bdic_power_resume_tbl[SHDISP_BDIC_DEV_TYPE_LCD_BKL] = SHDISP_BDIC_DEV_PWR_OFF;
    }
    if (s_state_str.bdic_power_resume_tbl[SHDISP_BDIC_DEV_TYPE_LCD_PWR] == SHDISP_BDIC_DEV_PWR_ON) {
        shdisp_bdic_API_set_power_mode(SHDISP_BDIC_DEV_TYPE_LCD_PWR, SHDISP_BDIC_DEV_PWR_ON);
        s_state_str.bdic_power_resume_tbl[SHDISP_BDIC_DEV_TYPE_LCD_PWR] = SHDISP_BDIC_DEV_PWR_OFF;
    }
    if (s_state_str.bdic_power_resume_tbl[SHDISP_BDIC_DEV_TYPE_TRI_LED] == SHDISP_BDIC_DEV_PWR_ON) {
        shdisp_bdic_API_set_power_mode(SHDISP_BDIC_DEV_TYPE_TRI_LED, SHDISP_BDIC_DEV_PWR_ON);
        s_state_str.bdic_power_resume_tbl[SHDISP_BDIC_DEV_TYPE_TRI_LED] = SHDISP_BDIC_DEV_PWR_OFF;
    }
    if (s_state_str.bdic_power_resume_tbl[SHDISP_BDIC_DEV_TYPE_PHOTO_SENSOR_APP] == SHDISP_BDIC_DEV_PWR_ON) {
        shdisp_bdic_API_set_power_mode(SHDISP_BDIC_DEV_TYPE_PHOTO_SENSOR_APP, SHDISP_BDIC_DEV_PWR_ON);
        s_state_str.bdic_power_resume_tbl[SHDISP_BDIC_DEV_TYPE_PHOTO_SENSOR_APP] = SHDISP_BDIC_DEV_PWR_OFF;
    }
    if (s_state_str.bdic_power_resume_tbl[SHDISP_BDIC_DEV_TYPE_PHOTO_SENSOR_LUX] == SHDISP_BDIC_DEV_PWR_ON) {
        shdisp_bdic_API_set_power_mode(SHDISP_BDIC_DEV_TYPE_PHOTO_SENSOR_LUX, SHDISP_BDIC_DEV_PWR_ON);
        s_state_str.bdic_power_resume_tbl[SHDISP_BDIC_DEV_TYPE_PHOTO_SENSOR_LUX] = SHDISP_BDIC_DEV_PWR_OFF;
    }
    if (s_state_str.bdic_power_resume_tbl[SHDISP_BDIC_DEV_TYPE_PHOTO_SENSOR_BKL] == SHDISP_BDIC_DEV_PWR_ON) {
        shdisp_bdic_API_set_power_mode(SHDISP_BDIC_DEV_TYPE_PHOTO_SENSOR_BKL, SHDISP_BDIC_DEV_PWR_ON);
        s_state_str.bdic_power_resume_tbl[SHDISP_BDIC_DEV_TYPE_PHOTO_SENSOR_BKL] = SHDISP_BDIC_DEV_PWR_OFF;
    }
    if (s_state_str.bdic_power_resume_tbl[SHDISP_BDIC_DEV_TYPE_PROX_SENSOR] == SHDISP_BDIC_DEV_PWR_ON) {
        shdisp_bdic_API_set_power_mode(SHDISP_BDIC_DEV_TYPE_PROX_SENSOR, SHDISP_BDIC_DEV_PWR_ON);
        s_state_str.bdic_power_resume_tbl[SHDISP_BDIC_DEV_TYPE_PROX_SENSOR] = SHDISP_BDIC_DEV_PWR_OFF;
    }
    
    if (shdisp_bdic_tri_led_color != 0) {
        switch (shdisp_bdic_tri_led_mode) {
        case SHDISP_BDIC_TRI_LED_MODE_NORMAL:
            shdisp_bdic_API_TRI_LED_normal_on(shdisp_bdic_tri_led_color);
            break;
        case SHDISP_BDIC_TRI_LED_MODE_BLINK:
            shdisp_bdic_API_TRI_LED_blink_on(shdisp_bdic_tri_led_color, shdisp_bdic_tri_led_ontime, shdisp_bdic_tri_led_interval);
            break;
        case SHDISP_BDIC_TRI_LED_MODE_FIREFLY:
            shdisp_bdic_API_TRI_LED_firefly_on(shdisp_bdic_tri_led_color, shdisp_bdic_tri_led_ontime, shdisp_bdic_tri_led_interval);
            break;
        default:
            break;
        }
    }
    
    if (shdisp_bdic_bkl_mode != SHDISP_BDIC_BKL_MODE_OFF) {
        switch (shdisp_bdic_bkl_mode) {
        case SHDISP_BDIC_BKL_MODE_FIX:
            shdisp_bdic_API_LCD_BKL_fix_on(shdisp_bdic_bkl_param);
            break;
        case SHDISP_BDIC_BKL_MODE_AUTO:
            shdisp_bdic_API_LCD_BKL_auto_on(shdisp_bdic_bkl_param);
            break;
        default:
            break;
        }
    }
    
    return SHDISP_RESULT_SUCCESS;
}


/* ------------------------------------------------------------------------- */
/* shdisp_bdic_API_DBG_INFO_output                                           */
/* ------------------------------------------------------------------------- */

void shdisp_bdic_API_DBG_INFO_output(void)
{
    int idx;
    
    printk("[SHDISP] BL69Y6 INFO ->>\n");
    printk("[SHDISP] s_state_str.bdic_power_status          = %d.\n", s_state_str.bdic_power_status);
    for ( idx = 0; idx < NUM_SHDISP_BDIC_DEV_TYPE; idx++ ) {
        printk("[SHDISP] s_state_str.bdic_power_status_tbl[%d]   = %d.\n", idx, s_state_str.bdic_power_status_tbl[idx]);
    }
    printk("[SHDISP] s_state_str.bdic_is_exist              = %d.\n", s_state_str.bdic_is_exist);
    printk("[SHDISP] s_state_str.photo_sensor_user          = 0x%08X.\n", (unsigned int)s_state_str.photo_sensor_user);
    printk("[SHDISP] s_state_str.prox_sensor_user           = 0x%08X.\n", (unsigned int)s_state_str.prox_sensor_user);
    printk("[SHDISP] s_state_str.bdic_main_bkl_opt_mode_output = %d. (0:low/1:high)\n", s_state_str.bdic_main_bkl_opt_mode_output);
    printk("[SHDISP] s_state_str.bdic_main_bkl_opt_mode_ado = %d. (0:low/1:high)\n", s_state_str.bdic_main_bkl_opt_mode_ado);
    printk("[SHDISP] s_state_str.shdisp_lux_change_level1   = %d.\n", s_state_str.shdisp_lux_change_level1);
    printk("[SHDISP] s_state_str.shdisp_lux_change_level2   = %d.\n", s_state_str.shdisp_lux_change_level2);
    printk("[SHDISP] s_state_str.als_adj0_low               = 0x%02X.\n", s_state_str.als_adj0_low);
    printk("[SHDISP] s_state_str.als_adj0_high              = 0x%02X.\n", s_state_str.als_adj0_high);
    printk("[SHDISP] s_state_str.als_adj1_low               = 0x%02X.\n", s_state_str.als_adj1_low);
    printk("[SHDISP] s_state_str.als_adj1_high              = 0x%02X.\n", s_state_str.als_adj1_high);
    printk("[SHDISP] s_state_str.als_shift                  = 0x%02X.\n", s_state_str.als_shift);
    printk("[SHDISP] s_state_str.clear_offset               = 0x%02X.\n", s_state_str.clear_offset);
    printk("[SHDISP] s_state_str.ir_offset                  = 0x%02X.\n", s_state_str.ir_offset);
    printk("[SHDISP] s_state_str.bkl_adjust_val             = %d.\n", s_state_str.bkl_adjust_val);
    printk("[SHDISP] s_state_str.shdisp_bdic_prox_params    = (threshold_high:0x%08X ,threshold_low:0x%08X).\n",
             s_state_str.shdisp_bdic_prox_params.threshold_high, s_state_str.shdisp_bdic_prox_params.threshold_low);
    printk("[SHDISP] shdisp_bdic_bkl_mode                   = %d.\n", shdisp_bdic_bkl_mode);
    printk("[SHDISP] shdisp_bdic_bkl_param                  = %d.\n", shdisp_bdic_bkl_param);
    printk("[SHDISP] shdisp_bdic_pwm                        = %d.\n", shdisp_bdic_pwm);
    printk("[SHDISP] shdisp_bdic_dtv                        = %d.\n", shdisp_bdic_dtv);
    printk("[SHDISP] shdisp_bdic_emg                        = %d.\n", shdisp_bdic_emg);
    printk("[SHDISP] shdisp_bdic_eco                        = %d.\n", shdisp_bdic_eco);
    printk("[SHDISP] shdisp_bdic_chg                        = %d.\n", shdisp_bdic_chg);
    printk("[SHDISP] shdisp_bdic_tri_led_color              = %d.\n", (int)shdisp_bdic_tri_led_color);
    printk("[SHDISP] shdisp_bdic_tri_led_mode               = %d.\n", shdisp_bdic_tri_led_mode);
    printk("[SHDISP] shdisp_bdic_tri_led_ontime             = %d.\n", shdisp_bdic_tri_led_ontime);
    printk("[SHDISP] shdisp_bdic_tri_led_interval           = %d.\n", shdisp_bdic_tri_led_interval);
    
    for ( idx = 0; idx < NUM_SHDISP_MAIN_BKL_DEV_TYPE; idx++ ) {
        printk("[SHDISP] shdisp_bkl_priority_table[%d]       = (mode:%d, param:%d).\n", idx, shdisp_bkl_priority_table[idx].mode, shdisp_bkl_priority_table[idx].param);
    }
    
    printk("[SHDISP] BL69Y6 INFO <<-\n");
    return;
}


/* ------------------------------------------------------------------------- */
/* shdisp_bdic_API_IRQ_set_reg                                               */
/* ------------------------------------------------------------------------- */

void shdisp_bdic_API_IRQ_set_reg( int irq_type, int onoff )
{
    unsigned char   val1, val2,val3,val4,val5;
    
    switch ( irq_type ) {
    case SHDISP_IRQ_TYPE_ALS:
        if( SHDISP_INT_ENABLE_GFAC & SHDISP_BDIC_INT_GFAC_ALS ){
            val1 = 0x01;
            if( onoff == SHDISP_IRQ_NO_MASK ){
                shdisp_bdic_PHOTO_SENSOR_lux_init();
            }
            else{
                shdisp_bdic_IO_clr_bit_reg( BDIC_REG_GIMR2, val1 );
                shdisp_bdic_IO_clr_bit_reg( BDIC_REG_GIMF2, val1 );
                shdisp_bdic_IO_clr_bit_reg( BDIC_REG_GIMR3, 0x20 );
                shdisp_bdic_IO_write_reg(BDIC_REG_ALS_INT,  0x14);
            }
        }

        

        break;
        
    case SHDISP_IRQ_TYPE_PS:
        if( SHDISP_INT_ENABLE_GFAC & SHDISP_BDIC_INT_GFAC_PS ){
            val1 = 0x08;
            if( onoff == SHDISP_IRQ_NO_MASK ){
                shdisp_bdic_IO_set_bit_reg( BDIC_REG_GIMR1, val1 );
                shdisp_bdic_IO_set_bit_reg( BDIC_REG_GIMF1, val1 );
                shdisp_bdic_IO_set_bit_reg( BDIC_REG_GIMR3, 0x08 );
                if (SHDISP_INT_ENABLE_GFAC & SHDISP_BDIC_INT_GFAC_PS2) {
                    shdisp_bdic_IO_set_bit_reg( BDIC_REG_GIMF2, 0x02 );
                }
            }
            else{
                shdisp_bdic_IO_clr_bit_reg( BDIC_REG_GIMR1, val1 );
                shdisp_bdic_IO_clr_bit_reg( BDIC_REG_GIMF1, val1 );
            }
        }
        break;

    case SHDISP_IRQ_TYPE_DET:
        if( SHDISP_INT_ENABLE_GFAC & SHDISP_BDIC_INT_GFAC_DET ){
            val1 = 0x04;
            if( onoff == SHDISP_IRQ_NO_MASK){
                shdisp_bdic_IO_set_bit_reg( BDIC_REG_GIMF3, val1 );
            }
            else{
                shdisp_bdic_IO_clr_bit_reg( BDIC_REG_GIMF3, val1 );
            }
        }
        break;
    default:
        return;
    }

    if( onoff == SHDISP_IRQ_MASK ){
        val1 = 0;
        val2 = 0;
        val3 = 0;
        val4 = 0;
        val5 = 0;
        shdisp_bdic_IO_read_reg( BDIC_REG_GIMR2, &val1);
        val1 &= 0x01;
        shdisp_bdic_IO_read_reg( BDIC_REG_GIMF2, &val2);
        val2 &= 0x01;
        shdisp_bdic_IO_read_reg( BDIC_REG_GIMR3, &val3);
        val3 &= 0x20;
        shdisp_bdic_IO_read_reg( BDIC_REG_GIMR1, &val4);
        val4 &= 0x08;
        shdisp_bdic_IO_read_reg( BDIC_REG_GIMF1, &val5);
        val5 &= 0x08;
        if((val1 == 0) && (val2 == 0) && (val3 == 0) && (val4 == 0) && (val5 == 0)){
            shdisp_bdic_IO_clr_bit_reg( BDIC_REG_GIMR3, 0x08 );
            if (SHDISP_INT_ENABLE_GFAC & SHDISP_BDIC_INT_GFAC_PS2) {
                shdisp_bdic_IO_clr_bit_reg( BDIC_REG_GIMF2, 0x02 );
            }
#ifdef SHDISP_SW_BDIC_IRQ_LOG
            printk("[SHDISP]I2C_ERR Interrupt Mask\n");
#endif /* SHDISP_SW_BDIC_IRQ_LOG */
        }
    }
}


/* ------------------------------------------------------------------------- */
/* shdisp_bdic_API_IRQ_check_type                                            */
/* ------------------------------------------------------------------------- */

int shdisp_bdic_API_IRQ_check_type( int irq_type )
{
    if(( irq_type < SHDISP_IRQ_TYPE_ALS ) || ( irq_type >= NUM_SHDISP_IRQ_TYPE ))
        return SHDISP_RESULT_FAILURE;
    
    if( irq_type == SHDISP_IRQ_TYPE_DET ){
        if( !( SHDISP_INT_ENABLE_GFAC & SHDISP_BDIC_INT_GFAC_DET )){
           return SHDISP_RESULT_FAILURE;
        }
    }
    
    return SHDISP_RESULT_SUCCESS;
}


/* ------------------------------------------------------------------------- */
/* shdisp_bdic_API_IRQ_save_fac                                              */
/* ------------------------------------------------------------------------- */

void shdisp_bdic_API_IRQ_save_fac(void)
{
    unsigned char value1=0, value2=0, value3=0;

    shdisp_bdic_IO_read_reg(BDIC_REG_GFAC1, &value1);
    value1 &= 0x1F;
    shdisp_bdic_IO_read_reg(BDIC_REG_GFAC2, &value2);
    shdisp_bdic_IO_read_reg(BDIC_REG_GFAC3, &value3);
#ifdef SHDISP_SW_BDIC_IRQ_LOG
    printk("[SHDISP] shdisp_bdic_AIP_save_irq_fac:GFAC3=%02x GFAC2=%02x GFAC1=%02x\n", value3, value2, value1);
#endif /* SHDISP_SW_BDIC_IRQ_LOG */
    shdisp_bdic_irq_fac = (unsigned int)value1 | ((unsigned int)value2 << 8 ) | ((unsigned int)value3 << 16);
    
    if( shdisp_bdic_irq_fac & SHDISP_BDIC_INT_GFAC_DET ){
        shdisp_bdic_IO_clr_bit_reg( BDIC_REG_GIMF3, 0x04 );
    }

    if( shdisp_bdic_irq_fac & SHDISP_BDIC_INT_GFAC_PS ){
        shdisp_bdic_IO_clr_bit_reg( BDIC_REG_GIMR1, 0x08 );
        shdisp_bdic_IO_clr_bit_reg( BDIC_REG_GIMF1, 0x08 );
    }


    if( shdisp_bdic_irq_fac & SHDISP_BDIC_INT_GFAC_PS2 ){
        shdisp_bdic_IO_clr_bit_reg( BDIC_REG_GIMF2, 0x02 );
    }

    if( shdisp_bdic_irq_fac & SHDISP_BDIC_INT_GFAC_I2C_ERR ){
        shdisp_bdic_IO_clr_bit_reg( BDIC_REG_GIMR3, 0x08 );
    }

	
    if( shdisp_bdic_irq_fac & (SHDISP_BDIC_INT_GFAC_DET | SHDISP_BDIC_INT_GFAC_PS2 | SHDISP_BDIC_INT_GFAC_I2C_ERR) ){
        shdisp_bdic_IO_read_reg( BDIC_REG_GIMR2, &shdisp_backup_irq_photo_req[0]);
        shdisp_bdic_IO_read_reg( BDIC_REG_GIMF2, &shdisp_backup_irq_photo_req[1]);
        shdisp_bdic_IO_read_reg( BDIC_REG_GIMR3, &shdisp_backup_irq_photo_req[2]);
    }
	
 
    if((shdisp_bdic_irq_fac & (SHDISP_BDIC_INT_GFAC_ALS | SHDISP_BDIC_INT_GFAC_OPTSEL)) != 0 ){
        shdisp_bdic_IO_clr_bit_reg( BDIC_REG_GIMR2, 0x01 );
        shdisp_bdic_IO_clr_bit_reg( BDIC_REG_GIMF2, 0x01 );
        shdisp_bdic_IO_clr_bit_reg( BDIC_REG_GIMR3, 0x20 );
    }

    
    if(shdisp_bdic_irq_det_flag == 1) {
        shdisp_bdic_irq_fac |= SHDISP_BDIC_INT_GFAC_DET;
        shdisp_bdic_irq_det_flag = 2;
    }
    
    return;
}


/* ------------------------------------------------------------------------- */
/* shdisp_bdic_API_IRQ_check_DET                                              */
/* ------------------------------------------------------------------------- */

int shdisp_bdic_API_IRQ_check_DET(void)
{
    if( shdisp_bdic_irq_fac & SHDISP_BDIC_INT_GFAC_DET ){
        return SHDISP_BDIC_IRQ_TYPE_DET;
    }
    else{
        return SHDISP_BDIC_IRQ_TYPE_NONE;
    }
}


/* ------------------------------------------------------------------------- */
/* shdisp_bdic_API_IRQ_check_fac                                              */
/* ------------------------------------------------------------------------- */

int shdisp_bdic_API_IRQ_check_fac(void)
{
    int i;
    if( shdisp_bdic_irq_fac == 0 ){
        return SHDISP_RESULT_FAILURE;
    }

    shdisp_bdic_irq_fac_exe = (shdisp_bdic_irq_fac & SHDISP_INT_ENABLE_GFAC);
    if( shdisp_bdic_irq_fac_exe == 0 ){
        return SHDISP_RESULT_FAILURE;
    }

    for(i=0; i<SHDISP_IRQ_MAX_KIND ; i++){
        shdisp_bdic_irq_prioriy[i] = SHDISP_BDIC_IRQ_TYPE_NONE;
    }
    
    i=0;
    if(( shdisp_bdic_irq_fac_exe & (SHDISP_BDIC_INT_GFAC_PS | SHDISP_BDIC_INT_GFAC_I2C_ERR | SHDISP_BDIC_INT_GFAC_PS2 )) == SHDISP_BDIC_INT_GFAC_PS ){
        shdisp_bdic_irq_prioriy[i] = SHDISP_BDIC_IRQ_TYPE_PS;
        i++;
    }
    
    if(( shdisp_bdic_irq_fac_exe & SHDISP_BDIC_INT_GFAC_DET ) != 0 ){
        shdisp_bdic_irq_prioriy[i] = SHDISP_BDIC_IRQ_TYPE_DET;
        i++;
    }
    else if(( shdisp_bdic_irq_fac_exe & SHDISP_BDIC_INT_GFAC_I2C_ERR ) != 0 
         || ( shdisp_bdic_irq_fac_exe & SHDISP_BDIC_INT_GFAC_PS2 ) != 0){
        shdisp_bdic_irq_prioriy[i] = SHDISP_BDIC_IRQ_TYPE_I2C_ERR;
        i++;
    }
    else if((shdisp_bdic_irq_fac_exe & (SHDISP_BDIC_INT_GFAC_ALS | SHDISP_BDIC_INT_GFAC_OPTSEL)) != 0 ){
        shdisp_bdic_irq_prioriy[i] = SHDISP_BDIC_IRQ_TYPE_ALS;
        i++;
    }
    
    return SHDISP_RESULT_SUCCESS;
}


/* ------------------------------------------------------------------------- */
/* shdisp_bdic_API_IRQ_get_fac                                                   */
/* ------------------------------------------------------------------------- */

int shdisp_bdic_API_IRQ_get_fac( int iQueFac )
{
    if( iQueFac >= SHDISP_IRQ_MAX_KIND )
        return SHDISP_BDIC_IRQ_TYPE_NONE;

    return shdisp_bdic_irq_prioriy[iQueFac];
}


/* ------------------------------------------------------------------------- */
/* shdisp_bdic_API_IRQ_Clear                                                 */
/* ------------------------------------------------------------------------- */

void shdisp_bdic_API_IRQ_Clear(void)
{
    unsigned char out1,out2,out3;

    
    if( shdisp_bdic_irq_fac == 0 )
        return;

    out1 = (unsigned char)(shdisp_bdic_irq_fac & 0x000000FF);
    out2 = (unsigned char)((shdisp_bdic_irq_fac >> 8 ) & 0x000000FF);
    out3 = (unsigned char)((shdisp_bdic_irq_fac >> 16 ) & 0x000000FF);
    
    shdisp_bdic_IO_write_reg(BDIC_REG_GSCR1, out1);
    shdisp_bdic_IO_write_reg(BDIC_REG_GSCR2, out2);
    shdisp_bdic_IO_write_reg(BDIC_REG_GSCR3, out3);
    
    shdisp_bdic_IO_write_reg(BDIC_REG_GSCR1, 0x00);
    shdisp_bdic_IO_write_reg(BDIC_REG_GSCR2, 0x00);
    shdisp_bdic_IO_write_reg(BDIC_REG_GSCR3, 0x00);
    
    
    if(( shdisp_bdic_irq_fac & SHDISP_BDIC_INT_GFAC_DET ) && ( SHDISP_INT_ENABLE_GFAC & SHDISP_BDIC_INT_GFAC_DET )){
        shdisp_bdic_IO_set_bit_reg( BDIC_REG_GIMF3, 0x04 );
    }

    if(( shdisp_bdic_irq_fac & SHDISP_BDIC_INT_GFAC_PS ) && ( SHDISP_INT_ENABLE_GFAC & SHDISP_BDIC_INT_GFAC_PS )){
        shdisp_bdic_IO_set_bit_reg( BDIC_REG_GIMR1, 0x08 );
        shdisp_bdic_IO_set_bit_reg( BDIC_REG_GIMF1, 0x08 );
    }


    if(( shdisp_bdic_irq_fac & SHDISP_BDIC_INT_GFAC_PS2 ) && ( SHDISP_INT_ENABLE_GFAC & SHDISP_BDIC_INT_GFAC_PS2 )){
        shdisp_bdic_IO_set_bit_reg( BDIC_REG_GIMF2, 0x02 );
    }

    
    if(( shdisp_bdic_irq_fac & SHDISP_BDIC_INT_GFAC_I2C_ERR ) && ( SHDISP_INT_ENABLE_GFAC & SHDISP_BDIC_INT_GFAC_I2C_ERR)){
        shdisp_bdic_IO_set_bit_reg( BDIC_REG_GIMR3, 0x08 );
    }
    
	
    if((( shdisp_bdic_irq_fac & SHDISP_BDIC_INT_GFAC_DET ) && ( SHDISP_INT_ENABLE_GFAC & SHDISP_BDIC_INT_GFAC_DET )) ||
       (( shdisp_bdic_irq_fac & SHDISP_BDIC_INT_GFAC_PS2 ) && ( SHDISP_INT_ENABLE_GFAC & SHDISP_BDIC_INT_GFAC_PS2 )) ||
       (( shdisp_bdic_irq_fac & SHDISP_BDIC_INT_GFAC_I2C_ERR ) && ( SHDISP_INT_ENABLE_GFAC & SHDISP_BDIC_INT_GFAC_I2C_ERR))){
        if( shdisp_backup_irq_photo_req[0] & 0x01 ){
            shdisp_bdic_IO_set_bit_reg( BDIC_REG_GIMR2, 0x01 );
        }
        if( shdisp_backup_irq_photo_req[1] & 0x01 ){
            shdisp_bdic_IO_set_bit_reg( BDIC_REG_GIMF2, 0x01 );
        }
        if( shdisp_backup_irq_photo_req[2] & 0x20 ){
            shdisp_bdic_IO_set_bit_reg( BDIC_REG_GIMR3, 0x20 );
        }
    }
	
    
}

/* ------------------------------------------------------------------------- */
/* shdisp_bdic_API_IRQ_i2c_error_Clear                                       */
/* ------------------------------------------------------------------------- */

void shdisp_bdic_API_IRQ_i2c_error_Clear(void)
{
    unsigned char out2=0,out3=0;

    if( SHDISP_INT_ENABLE_GFAC & SHDISP_BDIC_INT_GFAC_I2C_ERR ){
        out3 = 0x08;
        shdisp_bdic_IO_write_reg(BDIC_REG_GSCR3, out3);
        shdisp_bdic_IO_write_reg(BDIC_REG_GSCR3, 0x00);
    }

    if( SHDISP_INT_ENABLE_GFAC & SHDISP_BDIC_INT_GFAC_PS2 ){
        out2 = 0x02;
        shdisp_bdic_IO_write_reg(BDIC_REG_GSCR2, out2);
        shdisp_bdic_IO_write_reg(BDIC_REG_GSCR2, 0x00);
    }
}


/* ------------------------------------------------------------------------- */
/* shdisp_bdic_API_IRQ_i2c_error_Clear                                       */
/* ------------------------------------------------------------------------- */

void shdisp_bdic_API_IRQ_det_fac_Clear(void)
{
    unsigned char out3=0;

    if( SHDISP_INT_ENABLE_GFAC & SHDISP_BDIC_INT_GFAC_DET ){
        out3 = 0x04;
        shdisp_bdic_IO_write_reg(BDIC_REG_GSCR3, out3);
        shdisp_bdic_IO_write_reg(BDIC_REG_GSCR3, 0x00);
    }
}


/* ------------------------------------------------------------------------- */
/* shdisp_bdic_API_IRQ_backup_irq_reg                                        */
/* ------------------------------------------------------------------------- */

void shdisp_bdic_API_IRQ_backup_irq_reg(void)
{
    memset(&shdisp_backup_irq_reg, 0, sizeof(shdisp_backup_irq_reg));
    shdisp_bdic_IO_read_reg( BDIC_REG_GIMR1, &shdisp_backup_irq_reg[0]);
    shdisp_bdic_IO_read_reg( BDIC_REG_GIMR2, &shdisp_backup_irq_reg[1]);
    shdisp_bdic_IO_read_reg( BDIC_REG_GIMR3, &shdisp_backup_irq_reg[2]);
    shdisp_bdic_IO_read_reg( BDIC_REG_GIMF1, &shdisp_backup_irq_reg[3]);
    shdisp_bdic_IO_read_reg( BDIC_REG_GIMF2, &shdisp_backup_irq_reg[4]);
    shdisp_bdic_IO_read_reg( BDIC_REG_GIMF3, &shdisp_backup_irq_reg[5]);
#ifdef SHDISP_SW_BDIC_IRQ_LOG
    printk("[SHDISP DBG] backup_irq_reg: R1=0x%02X,R2=0x%02X,R3=0x%02X,F1=0x%02X,F2=0x%02X,F3=0x%02X\n",
        shdisp_backup_irq_reg[0], shdisp_backup_irq_reg[1], shdisp_backup_irq_reg[2],
        shdisp_backup_irq_reg[3], shdisp_backup_irq_reg[4], shdisp_backup_irq_reg[5]);
#endif
}

/* ------------------------------------------------------------------------- */
/* shdisp_bdic_API_IRQ_return_irq_reg                                        */
/* ------------------------------------------------------------------------- */

void shdisp_bdic_API_IRQ_return_irq_reg(void)
{
    shdisp_bdic_IO_write_reg( BDIC_REG_GIMR1, shdisp_backup_irq_reg[0]);
    shdisp_bdic_IO_write_reg( BDIC_REG_GIMR2, shdisp_backup_irq_reg[1]);
    shdisp_bdic_IO_write_reg( BDIC_REG_GIMR3, shdisp_backup_irq_reg[2]);
    shdisp_bdic_IO_write_reg( BDIC_REG_GIMF1, shdisp_backup_irq_reg[3]);
    shdisp_bdic_IO_write_reg( BDIC_REG_GIMF2, shdisp_backup_irq_reg[4]);
    shdisp_bdic_IO_write_reg( BDIC_REG_GIMF3, shdisp_backup_irq_reg[5]);
}

/* ------------------------------------------------------------------------- */
/* shdisp_bdic_API_IRQ_return_irq_reg_no_det                                 */
/* ------------------------------------------------------------------------- */

void shdisp_bdic_API_IRQ_return_irq_reg_no_det(void)
{
    shdisp_bdic_IO_write_reg( BDIC_REG_GIMR1, shdisp_backup_irq_reg[0]);
    shdisp_bdic_IO_write_reg( BDIC_REG_GIMR2, shdisp_backup_irq_reg[1]);
    shdisp_bdic_IO_write_reg( BDIC_REG_GIMR3, shdisp_backup_irq_reg[2]);
    shdisp_bdic_IO_write_reg( BDIC_REG_GIMF1, shdisp_backup_irq_reg[3]);
    shdisp_bdic_IO_write_reg( BDIC_REG_GIMF2, shdisp_backup_irq_reg[4]);
    shdisp_bdic_IO_write_reg( BDIC_REG_GIMF3, (shdisp_backup_irq_reg[5] & (~0x04)));
}

/* ------------------------------------------------------------------------- */
/* shdisp_bdic_API_IRQ_irq_reg_i2c_mask                                      */
/* ------------------------------------------------------------------------- */

void shdisp_bdic_API_IRQ_irq_reg_i2c_mask(void)
{
    if( SHDISP_INT_ENABLE_GFAC & SHDISP_BDIC_INT_GFAC_I2C_ERR ){
        shdisp_bdic_IO_clr_bit_reg( BDIC_REG_GIMR3, 0x08);
    }

    if( SHDISP_INT_ENABLE_GFAC & SHDISP_BDIC_INT_GFAC_PS2 ){
        shdisp_bdic_IO_clr_bit_reg( BDIC_REG_GIMF2, 0x02);
    }
}


/* ------------------------------------------------------------------------- */
/* shdisp_bdic_API_IRQ_det_irq_mask                                          */
/* ------------------------------------------------------------------------- */

void shdisp_bdic_API_IRQ_det_irq_mask(void)
{
    if( SHDISP_INT_ENABLE_GFAC & SHDISP_BDIC_INT_GFAC_DET ){
        shdisp_bdic_IO_clr_bit_reg( BDIC_REG_GIMF3, 0x04 );
    }
}


/* ------------------------------------------------------------------------- */
/* shdisp_bdic_API_IRQ_set_det_flag                                          */
/* ------------------------------------------------------------------------- */
void shdisp_bdic_API_IRQ_set_det_flag(void)
{
    shdisp_bdic_irq_det_flag = 1;
}


/* ------------------------------------------------------------------------- */
/* shdisp_bdic_API_IRQ_clr_det_flag                                          */
/* ------------------------------------------------------------------------- */
void shdisp_bdic_API_IRQ_clr_det_flag(void)
{
    if(shdisp_bdic_irq_det_flag != 1) {
        shdisp_bdic_irq_det_flag = 0;
    }
}


/* ------------------------------------------------------------------------- */
/* shdisp_bdic_API_IRQ_get_det_flag                                          */
/* ------------------------------------------------------------------------- */
int shdisp_bdic_API_IRQ_get_det_flag(void)
{
    return shdisp_bdic_irq_det_flag;
}


/* ------------------------------------------------------------------------- */
/* shdisp_bdic_API_IRQ_dbg_Clear_All                                         */
/* ------------------------------------------------------------------------- */

void shdisp_bdic_API_IRQ_dbg_Clear_All(void)
{
    unsigned char out1,out2,out3;
    
    out1 = (unsigned char)(SHDISP_INT_ENABLE_GFAC & 0x000000FF);
    out2 = (unsigned char)((SHDISP_INT_ENABLE_GFAC >> 8 ) & 0x000000FF);
    out3 = (unsigned char)((SHDISP_INT_ENABLE_GFAC >> 16 ) & 0x000000FF);
    
    shdisp_bdic_IO_write_reg(BDIC_REG_GSCR1, out1);
    shdisp_bdic_IO_write_reg(BDIC_REG_GSCR2, out2);
    shdisp_bdic_IO_write_reg(BDIC_REG_GSCR3, out3);
    
    shdisp_bdic_IO_write_reg(BDIC_REG_GSCR1, 0x00);
    shdisp_bdic_IO_write_reg(BDIC_REG_GSCR2, 0x00);
    shdisp_bdic_IO_write_reg(BDIC_REG_GSCR3, 0x00);
}


/* ------------------------------------------------------------------------- */
/* shdisp_bdic_API_IRQ_dbg_set_fac                                           */
/* ------------------------------------------------------------------------- */

void shdisp_bdic_API_IRQ_dbg_set_fac(unsigned int nGFAC)
{
    shdisp_bdic_irq_fac = nGFAC;
    
    if( shdisp_bdic_irq_fac & SHDISP_BDIC_INT_GFAC_DET ){
        shdisp_bdic_IO_clr_bit_reg( BDIC_REG_GIMF3, 0x04 );
    }

    if( shdisp_bdic_irq_fac & SHDISP_BDIC_INT_GFAC_PS ){
        shdisp_bdic_IO_clr_bit_reg( BDIC_REG_GIMR1, 0x08 );
        shdisp_bdic_IO_clr_bit_reg( BDIC_REG_GIMF1, 0x08 );
    }

    if( shdisp_bdic_irq_fac & SHDISP_BDIC_INT_GFAC_ALS ){
        shdisp_bdic_IO_clr_bit_reg( BDIC_REG_GIMR2, 0x01 );
        shdisp_bdic_IO_clr_bit_reg( BDIC_REG_GIMF2, 0x01 );
    }

    if( shdisp_bdic_irq_fac & SHDISP_BDIC_INT_GFAC_PS2 ){
        shdisp_bdic_IO_clr_bit_reg( BDIC_REG_GIMF2, 0x02 );
    }

    if( shdisp_bdic_irq_fac & SHDISP_BDIC_INT_GFAC_OPTSEL ){
        shdisp_bdic_IO_clr_bit_reg( BDIC_REG_GIMR3, 0x20 );
    }
    
    if( shdisp_bdic_irq_fac & SHDISP_BDIC_INT_GFAC_I2C_ERR ){
        shdisp_bdic_IO_clr_bit_reg( BDIC_REG_GIMR3, 0x08 );
    }
}


/* ------------------------------------------------------------------------- */
/* shdisp_bdic_API_IRQ_dbg_photo_param                                       */
/* ------------------------------------------------------------------------- */

void shdisp_bdic_API_IRQ_dbg_photo_param( int level1, int level2)
{
    s_state_str.shdisp_lux_change_level1 = (unsigned char)( level1 & 0x00FF );
    s_state_str.shdisp_lux_change_level2 = (unsigned char)( level2 & 0x00FF );
}


/* ------------------------------------------------------------------------- */
/* shdisp_bdic_API_PHOTO_lux_timer                                           */
/* ------------------------------------------------------------------------- */

int shdisp_bdic_API_PHOTO_lux_timer( int onoff )
{
    int ret = SHDISP_RESULT_SUCCESS;
    unsigned char   remask;
    
    if(( s_state_str.photo_sensor_user & ( SHDISP_BDIC_DEVICE_PHOTO_SENSOR_BKL | SHDISP_BDIC_DEVICE_PHOTO_SENSOR_LUX | SHDISP_BDIC_DEVICE_PHOTO_SENSOR_APP )) == 0 )
        return SHDISP_RESULT_FAILURE;

    if( onoff == SHDISP_BDIC_PHOTO_LUX_TIMER_OFF ){
        shdisp_bdic_IO_clr_bit_reg(BDIC_REG_I2C_START, 0x10);
        shdisp_bdic_IO_set_bit_reg(BDIC_REG_ALS_DATA0_SET, SHDISP_BDIC_I2C_ADO_UPDATE_MASK);
        shdisp_SYS_delay_us(1000);
#if defined(CONFIG_MACH_TOR) || defined(CONFIG_MACH_MNB) || defined(CONFIG_MACH_ARS)
        shdisp_bdic_IO_write_reg(BDIC_REG_TEST0, 0x00);
#endif
        shdisp_bdic_ps_req_mask( &remask );
    }
    else{
        shdisp_bdic_IO_clr_bit_reg(BDIC_REG_ALS_DATA0_SET, SHDISP_BDIC_I2C_ADO_UPDATE_MASK);
        shdisp_bdic_IO_write_reg(BDIC_REG_I2C_DATA6, 0x0C);
        shdisp_bdic_IO_set_bit_reg(BDIC_REG_I2C_START, 0x10);
        shdisp_SYS_delay_us(1000);
    }
    
    return ret;
}


/* ------------------------------------------------------------------------- */
/* shdisp_bdic_API_SENSOR_start                                              */
/* ------------------------------------------------------------------------- */

int shdisp_bdic_API_SENSOR_start( int new_opt_mode )
{
    int ret;
    
    ret = shdisp_bdic_IO_clr_bit_reg(BDIC_REG_SENSOR, 0x01);
    s_state_str.bdic_main_bkl_opt_mode_ado = new_opt_mode;
    shdisp_bdic_ps_req_restart(1);
#if defined(CONFIG_MACH_TOR) || defined(CONFIG_MACH_MNB) || defined(CONFIG_MACH_ARS)
    if(s_state_str.bdic_main_bkl_opt_mode_ado == SHDISP_BDIC_MAIN_BKL_OPT_LOW){
        shdisp_bdic_IO_write_reg(BDIC_REG_TEST0, 0x04);
    }
#endif
    return ret;
}


/* ------------------------------------------------------------------------- */
/* Logical Driver                                                            */
/* ------------------------------------------------------------------------- */
/* ------------------------------------------------------------------------- */
/* shdisp_bdic_LD_GPIO_control                                               */
/* ------------------------------------------------------------------------- */

static void shdisp_bdic_LD_GPIO_control(unsigned char symbol, unsigned char status)
{
    unsigned char port;

    switch (symbol) {
    case SHDISP_BDIC_GPIO_COG_RESET:
        port = SHDISP_BDIC_GPIO_GPOD4;
        break;
        
    default:
        return;;
    }

    shdisp_bdic_PD_GPIO_control(port, status);

    return;
}


/* ------------------------------------------------------------------------- */
/* shdisp_bdic_LD_LCD_BKL_active                                             */
/* ------------------------------------------------------------------------- */

static void shdisp_bdic_LD_LCD_BKL_active(void)
{

    return;
}


/* ------------------------------------------------------------------------- */
/* shdisp_bdic_LD_LCD_BKL_standby                                            */
/* ------------------------------------------------------------------------- */

static void shdisp_bdic_LD_LCD_BKL_standby(void)
{

    return;
}


/* ------------------------------------------------------------------------- */
/* shdisp_bdic_LD_LCD_BKL_off                                                */
/* ------------------------------------------------------------------------- */

static void shdisp_bdic_LD_LCD_BKL_off(void)
{
    shdisp_bdic_PD_BKL_control(SHDISP_BDIC_REQ_BKL_SET_MODE_OFF, 0);

    shdisp_bdic_PD_BKL_control(SHDISP_BDIC_REQ_STOP, 0);

    return;
}


/* ------------------------------------------------------------------------- */
/* shdisp_bdic_LD_LCD_BKL_fix_on                                             */
/* ------------------------------------------------------------------------- */

static void shdisp_bdic_LD_LCD_BKL_fix_on(int param)
{
    shdisp_bdic_PD_BKL_control(SHDISP_BDIC_REQ_BKL_SET_MODE_FIX, param);

    shdisp_bdic_PD_BKL_control(SHDISP_BDIC_REQ_START, 0);

    return;
}


/* ------------------------------------------------------------------------- */
/* shdisp_bdic_LD_LCD_BKL_auto_on                                            */
/* ------------------------------------------------------------------------- */

static void shdisp_bdic_LD_LCD_BKL_auto_on(int param)
{
    shdisp_bdic_PD_BKL_control(SHDISP_BDIC_REQ_BKL_SET_MODE_AUTO, param);

    shdisp_bdic_PD_BKL_control(SHDISP_BDIC_REQ_PRE_START, 0);

    shdisp_bdic_API_set_power_mode(SHDISP_BDIC_DEV_TYPE_PHOTO_SENSOR_BKL, SHDISP_BDIC_DEV_PWR_ON);

    shdisp_bdic_PD_BKL_control(SHDISP_BDIC_REQ_POST_START, 0);

	return;
}


/* ------------------------------------------------------------------------- */
/* shdisp_bdic_LD_LCD_PWR_active                                             */
/* ------------------------------------------------------------------------- */

static void shdisp_bdic_LD_LCD_PWR_active(void)
{
    return;
}


/* ------------------------------------------------------------------------- */
/* shdisp_bdic_LD_LCD_PWR_standby                                            */
/* ------------------------------------------------------------------------- */

static void shdisp_bdic_LD_LCD_PWR_standby(void)
{
    return;
}


/* ------------------------------------------------------------------------- */
/* shdisp_bdic_LD_LCD_PWR_on                                                 */
/* ------------------------------------------------------------------------- */

static void shdisp_bdic_LD_LCD_PWR_on(void)
{
    shdisp_bdic_IO_set_bit_reg(BDIC_REG_DCDC2_SYS, 0x40);
    
    
    return;
}


/* ------------------------------------------------------------------------- */
/* shdisp_bdic_LD_LCD_PWR_off                                                */
/* ------------------------------------------------------------------------- */

static void shdisp_bdic_LD_LCD_PWR_off(void)
{
    shdisp_bdic_IO_clr_bit_reg(BDIC_REG_DCDC2_SYS, 0x40);
    shdisp_SYS_delay_us(2000);
    
    return;
}


/* ------------------------------------------------------------------------- */
/* shdisp_bdic_LD_LCD_M_PWR_on                                               */
/* ------------------------------------------------------------------------- */

static void shdisp_bdic_LD_LCD_M_PWR_on(void)
{
    shdisp_bdic_IO_set_bit_reg(BDIC_REG_CPM, 0x01);
    
    return;
}


/* ------------------------------------------------------------------------- */
/* shdisp_bdic_LD_LCD_M_PWR_off                                              */
/* ------------------------------------------------------------------------- */

static void shdisp_bdic_LD_LCD_M_PWR_off(void)
{
    shdisp_bdic_IO_clr_bit_reg(BDIC_REG_CPM, 0x01);
    shdisp_SYS_delay_us(1000);

    return;
}


/* ------------------------------------------------------------------------- */
/* shdisp_bdic_LD_LCD_VO2_ON                                                 */
/* ------------------------------------------------------------------------- */

static void shdisp_bdic_LD_LCD_VO2_ON(void)
{
    shdisp_bdic_IO_write_reg(BDIC_REG_DCDC2_VO, BDIC_REG_DCDC2_VO_VAL);
    
    return;
}


/* ------------------------------------------------------------------------- */
/* shdisp_bdic_LD_LCD_VO2_OFF                                                */
/* ------------------------------------------------------------------------- */

static void shdisp_bdic_LD_LCD_VO2_OFF(void)
{
    shdisp_bdic_IO_write_reg(BDIC_REG_DCDC2_VO, 0x00);
    
    return;
}


/* ------------------------------------------------------------------------- */
/* shdisp_bdic_LD_TRI_LED_active                                             */
/* ------------------------------------------------------------------------- */

static void shdisp_bdic_LD_TRI_LED_active(void)
{
    return;
}


/* ------------------------------------------------------------------------- */
/* shdisp_bdic_LD_TRI_LED_standby                                            */
/* ------------------------------------------------------------------------- */

static void shdisp_bdic_LD_TRI_LED_standby(void)
{
    return;
}


/* ------------------------------------------------------------------------- */
/* shdisp_bdic_LD_TRI_LED_off                                                */
/* ------------------------------------------------------------------------- */

static void shdisp_bdic_LD_TRI_LED_off(void)
{
    shdisp_bdic_PD_TRI_LED_control(SHDISP_BDIC_REQ_STOP, 0);

    return;
}


/* ------------------------------------------------------------------------- */
/* shdisp_bdic_LD_TRI_LED_normal_on                                          */
/* ------------------------------------------------------------------------- */

static void shdisp_bdic_LD_TRI_LED_normal_on(unsigned char color)
{
    shdisp_bdic_PD_TRI_LED_control(SHDISP_BDIC_REQ_TRI_LED_SET_MODE_NORMAL, color);

    shdisp_bdic_PD_TRI_LED_control(SHDISP_BDIC_REQ_START, 0);

    return;
}


/* ------------------------------------------------------------------------- */
/* shdisp_bdic_LD_TRI_LED_blink_on                                           */
/* ------------------------------------------------------------------------- */

static void shdisp_bdic_LD_TRI_LED_blink_on(unsigned char color, int ontime, int interval)
{
    shdisp_bdic_PD_TRI_LED_control(SHDISP_BDIC_REQ_TRI_LED_SET_MODE_BLINK,   color);

    shdisp_bdic_PD_TRI_LED_control(SHDISP_BDIC_REQ_TRI_LED_SET_ONTIME,       ontime);

    shdisp_bdic_PD_TRI_LED_control(SHDISP_BDIC_REQ_TRI_LED_SET_INTERVAL,     interval);

    shdisp_bdic_PD_TRI_LED_control(SHDISP_BDIC_REQ_START, 0);

    return;
}


/* ------------------------------------------------------------------------- */
/* shdisp_bdic_LD_TRI_LED_firefly_on                                         */
/* ------------------------------------------------------------------------- */

static void shdisp_bdic_LD_TRI_LED_firefly_on(unsigned char color, int ontime, int interval)
{
    shdisp_bdic_PD_TRI_LED_control(SHDISP_BDIC_REQ_TRI_LED_SET_MODE_FIREFLY, color);

    shdisp_bdic_PD_TRI_LED_control(SHDISP_BDIC_REQ_TRI_LED_SET_ONTIME,       ontime);

    shdisp_bdic_PD_TRI_LED_control(SHDISP_BDIC_REQ_TRI_LED_SET_INTERVAL,     interval);

    shdisp_bdic_PD_TRI_LED_control(SHDISP_BDIC_REQ_START, 0);

    return;
}


/* ------------------------------------------------------------------------- */
/* shdisp_bdic_LD_TRI_LED_ANIME_active                                       */
/* ------------------------------------------------------------------------- */

static void shdisp_bdic_LD_TRI_LED_ANIME_active(void)
{
    return;
}


/* ------------------------------------------------------------------------- */
/* shdisp_bdic_LD_TRI_LED_ANIME_standby                                      */
/* ------------------------------------------------------------------------- */

static void shdisp_bdic_LD_TRI_LED_ANIME_standby(void)
{
    return;
}


/* ------------------------------------------------------------------------- */
/* shdisp_bdic_LD_PHOTO_SENSOR_active                                        */
/* ------------------------------------------------------------------------- */

static void shdisp_bdic_LD_PHOTO_SENSOR_active(unsigned long dev_type)
{
    shdisp_bdic_PD_PHOTO_SENSOR_control(dev_type, SHDISP_BDIC_REQ_ACTIVE);
    
    shdisp_bdic_PD_PHOTO_SENSOR_control(dev_type, SHDISP_BDIC_REQ_START);
    
    return;
}


/* ------------------------------------------------------------------------- */
/* shdisp_bdic_LD_PHOTO_SENSOR_standby                                       */
/* ------------------------------------------------------------------------- */

static void shdisp_bdic_LD_PHOTO_SENSOR_standby(unsigned long dev_type)
{
    shdisp_bdic_PD_PHOTO_SENSOR_control(dev_type, SHDISP_BDIC_REQ_STOP);
    
    shdisp_bdic_PD_PHOTO_SENSOR_control(dev_type, SHDISP_BDIC_REQ_STANDBY);
    
    return;
}


/* ------------------------------------------------------------------------- */
/* shdisp_bdic_LD_PHOTO_SENSOR_get_val                                       */
/* ------------------------------------------------------------------------- */

static int shdisp_bdic_LD_PHOTO_SENSOR_get_val(unsigned short *value)
{
    if (s_state_str.photo_sensor_user == SHDISP_BDIC_DEVICE_NONE) {
        SHDISP_ERR("<OTHER> photo sensor user none.\n");
        return SHDISP_RESULT_FAILURE;
    }
    
    shdisp_bdic_PD_REG_ADO_get_opt(value);
    
    return SHDISP_RESULT_SUCCESS;
}


/* ------------------------------------------------------------------------- */
/* shdisp_bdic_LD_PHOTO_SENSOR_get_lux                                       */
/* ------------------------------------------------------------------------- */

static int shdisp_bdic_LD_PHOTO_SENSOR_get_lux(unsigned short *value, unsigned long *lux)
{
    int mode = s_state_str.bdic_main_bkl_opt_mode_ado;
    unsigned long ret_lux;
    int ext_flag, ok_lux;
    int i,j;
    
    if (s_state_str.photo_sensor_user == SHDISP_BDIC_DEVICE_NONE) {
        SHDISP_ERR("<OTHER> photo sensor user none.\n");
        return SHDISP_RESULT_FAILURE;
    }
    
    shdisp_bdic_PD_REG_ADO_get_opt(value);

    ret_lux = shdisp_bdic_bkl_lux_tbl[1][15].lux;
    for(j=0; j<2 ; j++){
        ok_lux = 0;
        for( i=0; i<16 ; i++) {
            if (*value <= shdisp_bdic_bkl_lux_tbl[mode][i].ado_range) {
                ext_flag = shdisp_bdic_bkl_lux_tbl[mode][i].ext_flag;
                if( ext_flag == mode ){
                    ret_lux = shdisp_bdic_bkl_lux_tbl[mode][i].lux;
                    ok_lux = 1;
                }
                else{
                    mode = ext_flag;
                }
                break;
            }
        }
        if(( ok_lux ) || ( i >= 16 ))
            break;
    }
    
    *lux = ret_lux;

#ifdef SHDISP_SW_BDIC_IRQ_LOG
    if( s_state_str.bdic_main_bkl_opt_mode_ado == mode )
        printk("[SHDISP] shdisp_bdic_LD_PHOTO_SENSOR_get_lux : mode=%d / ado=0x%04X, lux=%lu\n", mode, *value, ret_lux);
    else
        printk("[SHDISP] shdisp_bdic_LD_PHOTO_SENSOR_get_lux : ext_mode=%d / ado=0x%04X, lux=%lu\n", mode, *value, ret_lux);
#endif /* SHDISP_SW_BDIC_IRQ_LOG */
    
    return SHDISP_RESULT_SUCCESS;
}


/* ------------------------------------------------------------------------- */
/* shdisp_bdic_LD_PHOTO_SENSOR_bias_als_adj                                  */
/* ------------------------------------------------------------------------- */

static void shdisp_bdic_LD_PHOTO_SENSOR_bias_als_adj(unsigned char als_in, unsigned char *als_out)
{
    unsigned char als_tmp;
    als_tmp = als_in & 0x1F;
    
    switch (als_tmp) {
    case 0x1E:
        *als_out = 0x00;
        break;
    case 0x1F:
        *als_out = 0x01;
        break;
    case 0x0E:
        *als_out = 0x0F;
        break;
    case 0x0F:
        *als_out = 0x0F;
        break;
    default:
        *als_out = als_tmp + 2;
        break;
    }
    
    return;
}


/* ------------------------------------------------------------------------- */
/* shdisp_bdic_LD_i2c_write                                                  */
/* ------------------------------------------------------------------------- */

static int shdisp_bdic_LD_i2c_write(struct shdisp_bdic_i2c_msg *msg)
{
    unsigned char value = 0x00;
    int i;
    
#ifdef SHDISP_SW_BDIC_I2C_RWLOG
    int t;
    if (msg->wlen <= 0) {
        printk("[SHDISP] bdic i2c Write(addr=0x%02X, size=%d)\n", msg->addr, msg->wlen);
    }
    else {
        printk("[SHDISP] bdic i2c Write(addr=0x%02X, size=%d Wbuf=", msg->addr, msg->wlen);
        for (t=0; t<(msg->wlen-1); t++) {
            printk("0x%02X,", (msg->wbuf[t] & 0xFF));
        }
        printk("0x%02X)\n", (msg->wbuf[t] & 0xFF));
    }
#endif
    
    if (msg->wbuf == NULL) {
        SHDISP_ERR("<NULL_POINTER> msg->wbuf.\n");
        return SHDISP_RESULT_FAILURE;
    }
    
    if (msg->wlen > SHDISP_BDIC_I2C_WBUF_MAX) {
        SHDISP_ERR("<INVALID_VALUE> msg->wlen(%d).\n", msg->wlen);
        return SHDISP_RESULT_FAILURE;
    }
    
    shdisp_bdic_IO_set_bit_reg(BDIC_REG_ALS_DATA0_SET, SHDISP_BDIC_I2C_ADO_UPDATE_MASK);
    shdisp_SYS_delay_us(1000);
    
    shdisp_bdic_IO_write_reg(BDIC_REG_I2C_DATA0, (unsigned char)((msg->addr & 0x007F) << 1));
    
    for (i=0; i<msg->wlen; i++) {
        shdisp_bdic_IO_write_reg((BDIC_REG_I2C_DATA1 + i), msg->wbuf[i]);
    }
    
    shdisp_bdic_IO_write_reg(BDIC_REG_I2C_SET, msg->wlen);
    
    shdisp_bdic_i2c_errflag = 0;
    
    shdisp_bdic_IO_write_reg(BDIC_REG_I2C_START, SHDISP_BDIC_I2C_W_START);
    shdisp_SYS_delay_us(SHDISP_BDIC_I2C_SEND_WAIT);
    
    for (i=0; i<=SHDISP_BDIC_I2C_SEND_RETRY; i++) {
        if (shdisp_bdic_i2c_errflag != 0) {
            SHDISP_ERR("<OTHER> i2c write isr.\n");
            return SHDISP_RESULT_FAILURE_I2C_TMO;
        }
        else {
            shdisp_bdic_IO_read_reg(BDIC_REG_I2C_START, &value);
            if ((value & SHDISP_BDIC_I2C_W_START) == 0x00) {
                return SHDISP_RESULT_SUCCESS;
            }
            else {
                shdisp_SYS_delay_us(SHDISP_BDIC_I2C_SEND_RETRY_WAIT);
            }
        }
    }
    
    SHDISP_ERR("<OTHER> i2c write timeout.\n");
    return SHDISP_RESULT_FAILURE_I2C_TMO;
}


/* ------------------------------------------------------------------------- */
/* shdisp_bdic_LD_i2c_read_mode0                                             */
/* ------------------------------------------------------------------------- */

static int shdisp_bdic_LD_i2c_read_mode0(struct shdisp_bdic_i2c_msg *msg)
{
    unsigned char value = 0x00;
    int i, t;
    
    if (msg->wbuf == NULL) {
        SHDISP_ERR("<NULL_POINTER> msg->wbuf.\n");
        return SHDISP_RESULT_FAILURE;
    }
    
    if (msg->wlen != 0x01) {
        SHDISP_ERR("<INVALID_VALUE> msg->wlen(%d).\n", msg->wlen);
        return SHDISP_RESULT_FAILURE;
    }
    
    if (msg->rbuf == NULL) {
        SHDISP_ERR("<NULL_POINTER> msg->rbuf.\n");
        return SHDISP_RESULT_FAILURE;
    }
    
    if (msg->rlen > SHDISP_BDIC_I2C_RBUF_MAX) {
        SHDISP_ERR("<INVALID_VALUE> msg->rlen(%d).\n", msg->rlen);
        return SHDISP_RESULT_FAILURE;
    }
    
    shdisp_bdic_IO_set_bit_reg(BDIC_REG_ALS_DATA0_SET, SHDISP_BDIC_I2C_ADO_UPDATE_MASK);
    shdisp_SYS_delay_us(1000);
    
    shdisp_bdic_IO_write_reg(BDIC_REG_I2C_SYS, 0x00);
    
    shdisp_bdic_IO_write_reg(BDIC_REG_I2C_DATA0, (unsigned char)((msg->addr & 0x007F) << 1));
    
    shdisp_bdic_IO_write_reg(BDIC_REG_I2C_DATA6, msg->wbuf[0]);
    
    shdisp_bdic_IO_write_reg(BDIC_REG_I2C_SET, (unsigned char)(msg->rlen << 4));
    
    shdisp_bdic_i2c_errflag = 0;
    
    shdisp_bdic_IO_write_reg(BDIC_REG_I2C_START, SHDISP_BDIC_I2C_R_START);
    shdisp_SYS_delay_us(SHDISP_BDIC_I2C_SEND_WAIT);
    
    for (i=0; i<=SHDISP_BDIC_I2C_SEND_RETRY; i++) {
        if (shdisp_bdic_i2c_errflag != 0) {
            SHDISP_ERR("<OTHER> i2c write isr.\n");
            return SHDISP_RESULT_FAILURE_I2C_TMO;
        }
        else {
            shdisp_bdic_IO_read_reg(BDIC_REG_I2C_START, &value);
            if ((value & SHDISP_BDIC_I2C_R_START) == 0x00) {
#ifdef SHDISP_SW_BDIC_I2C_RWLOG
                printk("[SHDISP] bdic i2c Read(addr=0x%02x, reg=0x%02X, size=%d, ", msg->addr, msg->wbuf[0], msg->rlen);
#endif
                for (t=0; t<msg->rlen; t++) {
                    shdisp_bdic_IO_read_reg((BDIC_REG_I2C_READDATA0 + t), &value);
                    msg->rbuf[t] = value;
#ifdef SHDISP_SW_BDIC_I2C_RWLOG
                    if (t < (msg->rlen - 1)) {
                        printk("0x%02X,", value);
                    }
                    else {
                        printk("0x%02X)\n", value);
                    }
#endif
                }
                return SHDISP_RESULT_SUCCESS;
            }
            else {
                shdisp_SYS_delay_us(SHDISP_BDIC_I2C_SEND_RETRY_WAIT);
            }
        }
    }
    
    SHDISP_ERR("<OTHER> i2c write timeout.\n");
    return SHDISP_RESULT_FAILURE_I2C_TMO;
}


/* ------------------------------------------------------------------------- */
/* shdisp_bdic_LD_i2c_read_mode1                                             */
/* ------------------------------------------------------------------------- */

static int shdisp_bdic_LD_i2c_read_mode1(struct shdisp_bdic_i2c_msg *msg)
{
    return SHDISP_RESULT_SUCCESS;
}


/* ------------------------------------------------------------------------- */
/* shdisp_bdic_LD_i2c_read_mode2                                             */
/* ------------------------------------------------------------------------- */

static int shdisp_bdic_LD_i2c_read_mode2(struct shdisp_bdic_i2c_msg *msg)
{
    return SHDISP_RESULT_SUCCESS;
}


/* ------------------------------------------------------------------------- */
/* shdisp_bdic_LD_i2c_read_mode3                                             */
/* ------------------------------------------------------------------------- */

static int shdisp_bdic_LD_i2c_read_mode3(struct shdisp_bdic_i2c_msg *msg)
{
    return SHDISP_RESULT_SUCCESS;
}


/* ------------------------------------------------------------------------- */
/* shdisp_bdic_LD_PROX_SENSOR_active                                         */
/* ------------------------------------------------------------------------- */

static void shdisp_bdic_LD_PROX_SENSOR_active(unsigned long dev_type)
{
    shdisp_bdic_PD_PROX_SENSOR_control(dev_type, SHDISP_BDIC_REQ_ACTIVE);
    
    shdisp_bdic_PD_PROX_SENSOR_control(dev_type, SHDISP_BDIC_REQ_START);
    
    return;
}


/* ------------------------------------------------------------------------- */
/* shdisp_bdic_LD_PROX_SENSOR_standby                                        */
/* ------------------------------------------------------------------------- */

static void shdisp_bdic_LD_PROX_SENSOR_standby(unsigned long dev_type)
{
    shdisp_bdic_PD_PROX_SENSOR_control(dev_type, SHDISP_BDIC_REQ_STOP);
    
    shdisp_bdic_PD_PROX_SENSOR_control(dev_type, SHDISP_BDIC_REQ_STANDBY);
    
    return;
}


/* ------------------------------------------------------------------------- */
/* shdisp_bdic_LD_SENSOR_get_state                                           */
/* ------------------------------------------------------------------------- */

static void shdisp_bdic_LD_SENSOR_get_state(int *state)
{
    if (s_state_str.prox_sensor_user == SHDISP_BDIC_DEVICE_NONE) {
        if (s_state_str.photo_sensor_user == SHDISP_BDIC_DEVICE_NONE) {
            *state = SHDISP_SENSOR_STATE_PROX_OFF_ALC_OFF;
        }
        else {
            *state = SHDISP_SENSOR_STATE_PROX_OFF_ALC_ON;
        }
    }
    else {
        if (s_state_str.photo_sensor_user == SHDISP_BDIC_DEVICE_NONE) {
            *state = SHDISP_SENSOR_STATE_PROX_ON_ALC_OFF;
        }
        else {
            *state = SHDISP_SENSOR_STATE_PROX_ON_ALC_ON;
        }
    }
    
    return;
}


/* ------------------------------------------------------------------------- */
/* shdisp_bdic_LD_LCD_BKL_pwm_disable                                        */
/* ------------------------------------------------------------------------- */

static void shdisp_bdic_LD_LCD_BKL_pwm_disable(void)
{
    if (shdisp_bdic_pwm == SHDISP_BDIC_BKL_PWM_DISABLE) {
        return;
    }
    
    shdisp_bdic_PD_BKL_control(SHDISP_BDIC_REQ_BKL_PWM_DISABLE, 0);
    
    
    return;
}


/* ------------------------------------------------------------------------- */
/* shdisp_bdic_LD_LCD_BKL_pwm_enable                                         */
/* ------------------------------------------------------------------------- */

static void shdisp_bdic_LD_LCD_BKL_pwm_enable(void)
{
    if (shdisp_bdic_pwm == SHDISP_BDIC_BKL_PWM_ENABLE) {
        return;
    }
    
    shdisp_bdic_PD_BKL_control(SHDISP_BDIC_REQ_BKL_PWM_ENABLE, 0);
    
    
    return;
}


/* ------------------------------------------------------------------------- */
/* shdisp_bdic_LD_LCD_BKL_dtv_on                                             */
/* ------------------------------------------------------------------------- */

static void shdisp_bdic_LD_LCD_BKL_dtv_on(void)
{
    if (shdisp_bdic_dtv == SHDISP_BDIC_BKL_DTV_ON) {
        return;
    }
    
    shdisp_bdic_PD_BKL_control(SHDISP_BDIC_REQ_BKL_DTV_ON, 0);
    
    if (shdisp_bdic_bkl_mode != SHDISP_BDIC_BKL_MODE_OFF) {
        shdisp_bdic_PD_BKL_control(SHDISP_BDIC_REQ_START, 0);
    }
    
    return;
}


/* ------------------------------------------------------------------------- */
/* shdisp_bdic_LD_LCD_BKL_dtv_off                                            */
/* ------------------------------------------------------------------------- */

static void shdisp_bdic_LD_LCD_BKL_dtv_off(void)
{
    if (shdisp_bdic_dtv == SHDISP_BDIC_BKL_DTV_OFF) {
        return;
    }
    
    shdisp_bdic_PD_BKL_control(SHDISP_BDIC_REQ_BKL_DTV_OFF, 0);
    
    if (shdisp_bdic_bkl_mode != SHDISP_BDIC_BKL_MODE_OFF) {
        shdisp_bdic_PD_BKL_control(SHDISP_BDIC_REQ_START, 0);
    }
    
    return;
}


/* ------------------------------------------------------------------------- */
/* shdisp_bdic_LD_LCD_BKL_emg_on                                             */
/* ------------------------------------------------------------------------- */

static void shdisp_bdic_LD_LCD_BKL_emg_on(void)
{
    if (shdisp_bdic_emg == SHDISP_BDIC_BKL_EMG_ON) {
        return;
    }
    
    shdisp_bdic_PD_BKL_control(SHDISP_BDIC_REQ_BKL_EMG_ON, 0);
    
    if (shdisp_bdic_bkl_mode != SHDISP_BDIC_BKL_MODE_OFF) {
        shdisp_bdic_PD_BKL_control(SHDISP_BDIC_REQ_START, 0);
    }
    
    return;
}


/* ------------------------------------------------------------------------- */
/* shdisp_bdic_LD_LCD_BKL_emg_off                                            */
/* ------------------------------------------------------------------------- */

static void shdisp_bdic_LD_LCD_BKL_emg_off(void)
{
    if (shdisp_bdic_emg == SHDISP_BDIC_BKL_EMG_OFF) {
        return;
    }
    
    shdisp_bdic_PD_BKL_control(SHDISP_BDIC_REQ_BKL_EMG_OFF, 0);
    
    if (shdisp_bdic_bkl_mode != SHDISP_BDIC_BKL_MODE_OFF) {
        shdisp_bdic_PD_BKL_control(SHDISP_BDIC_REQ_START, 0);
    }
    
    return;
}


/* ------------------------------------------------------------------------- */
/* shdisp_bdic_LD_LCD_BKL_get_mode                                           */
/* ------------------------------------------------------------------------- */

static void shdisp_bdic_LD_LCD_BKL_get_mode(int *mode)
{
    
    if (shdisp_bdic_emg == SHDISP_BDIC_BKL_EMG_ON) {
        *mode = SHDISP_BKL_TBL_MODE_EMERGENCY;
    }
#ifndef SHDISP_BDIC_DISABLE_BKL_CHG_MODE_FOR_FIX
    else if (shdisp_bdic_chg == SHDISP_BDIC_BKL_CHG_ON) {
        *mode = SHDISP_BKL_TBL_MODE_CHARGE;
    }
#else   /* SHDISP_BDIC_DISABLE_BKL_CHG_MODE_FOR_FIX */
    else if ((shdisp_bdic_bkl_mode != SHDISP_BDIC_BKL_MODE_FIX) && (shdisp_bdic_chg == SHDISP_BDIC_BKL_CHG_ON)) {
        *mode = SHDISP_BKL_TBL_MODE_CHARGE;
    }
#endif  /* SHDISP_BDIC_DISABLE_BKL_CHG_MODE_FOR_FIX */
    else if (shdisp_bdic_eco == SHDISP_BDIC_BKL_ECO_ON) {
        *mode = SHDISP_BKL_TBL_MODE_ECO;
    }
    else {
        *mode = SHDISP_BKL_TBL_MODE_NORMAL;
    }
    
    if (*mode >= NUM_SHDISP_BKL_TBL_MODE) {
        *mode = SHDISP_BKL_TBL_MODE_NORMAL;
    }
    
    return;
}


/* ------------------------------------------------------------------------- */
/* shdisp_bdic_LD_LCD_BKL_eco_on                                             */
/* ------------------------------------------------------------------------- */

static void shdisp_bdic_LD_LCD_BKL_eco_on(void)
{
    if (shdisp_bdic_eco == SHDISP_BDIC_BKL_ECO_ON) {
        return;
    }
    
    shdisp_bdic_PD_BKL_control(SHDISP_BDIC_REQ_BKL_ECO_ON, 0);
    
    if (shdisp_bdic_bkl_mode != SHDISP_BDIC_BKL_MODE_OFF) {
        shdisp_bdic_PD_BKL_control(SHDISP_BDIC_REQ_START, 0);
    }
    
    return;
}


/* ------------------------------------------------------------------------- */
/* shdisp_bdic_LD_LCD_BKL_eco_off                                            */
/* ------------------------------------------------------------------------- */

static void shdisp_bdic_LD_LCD_BKL_eco_off(void)
{
    if (shdisp_bdic_eco == SHDISP_BDIC_BKL_ECO_OFF) {
        return;
    }
    
    shdisp_bdic_PD_BKL_control(SHDISP_BDIC_REQ_BKL_ECO_OFF, 0);
    
    if (shdisp_bdic_bkl_mode != SHDISP_BDIC_BKL_MODE_OFF) {
        shdisp_bdic_PD_BKL_control(SHDISP_BDIC_REQ_START, 0);
    }
    
    return;
}


/* ------------------------------------------------------------------------- */
/* shdisp_bdic_LD_LCD_BKL_chg_on                                             */
/* ------------------------------------------------------------------------- */

static void shdisp_bdic_LD_LCD_BKL_chg_on(void)
{
    if (shdisp_bdic_chg == SHDISP_BDIC_BKL_CHG_ON) {
        return;
    }
    
    shdisp_bdic_PD_BKL_control(SHDISP_BDIC_REQ_BKL_CHG_ON, 0);
    
    if (shdisp_bdic_bkl_mode != SHDISP_BDIC_BKL_MODE_OFF) {
        shdisp_bdic_PD_BKL_control(SHDISP_BDIC_REQ_START, 0);
    }
    
    return;
}


/* ------------------------------------------------------------------------- */
/* shdisp_bdic_LD_LCD_BKL_chg_off                                            */
/* ------------------------------------------------------------------------- */

static void shdisp_bdic_LD_LCD_BKL_chg_off(void)
{
    if (shdisp_bdic_chg == SHDISP_BDIC_BKL_CHG_OFF) {
        return;
    }
    
    shdisp_bdic_PD_BKL_control(SHDISP_BDIC_REQ_BKL_CHG_OFF, 0);
    
    if (shdisp_bdic_bkl_mode != SHDISP_BDIC_BKL_MODE_OFF) {
        shdisp_bdic_PD_BKL_control(SHDISP_BDIC_REQ_START, 0);
    }
    
    return;
}


/* ------------------------------------------------------------------------- */
/* Phygical Driver                                                           */
/* ------------------------------------------------------------------------- */
/* ------------------------------------------------------------------------- */
/* shdisp_bdic_PD_BDIC_init                                                  */
/* ------------------------------------------------------------------------- */

static void shdisp_bdic_PD_BDIC_init(void)
{
    int i;
    unsigned char value;
    unsigned char data_buf[16];
    unsigned char detector_val;
    unsigned char adj0_2_l_val;
    unsigned char adj0_2_h_val;
    
    adj0_2_l_val = (unsigned char)(s_state_str.als_adj0_low * 1/5);
    adj0_2_h_val = (unsigned char)(s_state_str.als_adj0_high * 1/5);
    
    shdisp_bdic_IO_write_reg(BDIC_REG_SYSTEM2,                      0x80);
    shdisp_SYS_delay_us(11000);
    
    shdisp_bdic_IO_read_reg(BDIC_REG_ETC, &value);
    if ((value & 0xF0) == 0x00 ) {
        detector_val = 0x21;
        shdisp_bdic_IO_write_reg(BDIC_REG_TEST60,       0x10);
        shdisp_bdic_IO_write_reg(BDIC_REG_TEST_B5,      0x08);
        shdisp_bdic_IO_write_reg(BDIC_REG_TEST_B0,      0x05);
        shdisp_bdic_IO_write_reg(BDIC_REG_TEST_B1,      0x00);
        shdisp_bdic_IO_write_reg(BDIC_REG_TEST_B0,      0x04);
        shdisp_bdic_IO_write_reg(BDIC_REG_TEST_B1,      0x00);
        
        for (i=0; i<=15; i++) {
            value = 0;
            shdisp_bdic_IO_read_reg(BDIC_REG_TEST_B3,   &value);
            shdisp_SYS_delay_us(200);
            data_buf[i] = value;
        }
        
        shdisp_bdic_IO_write_reg(BDIC_REG_TRIM0,         data_buf[0]);
        shdisp_bdic_IO_write_reg(BDIC_REG_TRIM1,         data_buf[1]);
        shdisp_bdic_IO_write_reg(BDIC_REG_TRIM2,         data_buf[2]);
        shdisp_bdic_IO_write_reg(BDIC_REG_TRIM3,         data_buf[3]);
        shdisp_bdic_IO_write_reg(BDIC_REG_TRIM4,         data_buf[4]);
        shdisp_bdic_IO_write_reg(BDIC_REG_TRIM5,         data_buf[5]);
        shdisp_bdic_IO_write_reg(BDIC_REG_TRIM6,         data_buf[6]);
        shdisp_bdic_IO_write_reg(BDIC_REG_TRIM7,         data_buf[7]);
        shdisp_bdic_IO_write_reg(BDIC_REG_TRIM8,         data_buf[8]);
        shdisp_bdic_IO_write_reg(BDIC_REG_TRIM9,         data_buf[9]);
        shdisp_bdic_IO_write_reg(BDIC_REG_TRIM10,        data_buf[10]);
        shdisp_bdic_IO_write_reg(BDIC_REG_TRIM11,        data_buf[11]);
        shdisp_bdic_IO_write_reg(BDIC_REG_TRIM12,        data_buf[12]);
        shdisp_bdic_IO_write_reg(BDIC_REG_TRIM13,        data_buf[13]);
        shdisp_bdic_IO_write_reg(BDIC_REG_TRIM14,        data_buf[14]);
        shdisp_bdic_IO_write_reg(BDIC_REG_TRIM15,        data_buf[15]);
        
        shdisp_bdic_IO_write_reg(BDIC_REG_TEST_B5,      0x00);
        shdisp_bdic_IO_write_reg(BDIC_REG_TEST_B0,      0x00);
        shdisp_bdic_IO_write_reg(BDIC_REG_TEST60,       0x00);
    }
    else {
        detector_val = 0x01;
    }
    
    shdisp_bdic_IO_write_reg(BDIC_REG_SYSTEM1,                                  0x00);
    shdisp_bdic_IO_write_reg(BDIC_REG_SYSTEM2,                                  0x00);
    shdisp_bdic_IO_write_reg(BDIC_REG_SYSTEM3,                                  0x00);
    shdisp_bdic_IO_write_reg(BDIC_REG_SYSTEM4,                  BDIC_REG_SYSTEM4_VAL);
    
    shdisp_bdic_IO_write_reg(BDIC_REG_OPT_MODE,                                 0x00);
    shdisp_bdic_IO_write_reg(BDIC_REG_SLOPE,                      BDIC_REG_SLOPE_VAL);
    shdisp_bdic_IO_write_reg(BDIC_REG_MODE_M1,                                  0x03);
    shdisp_bdic_IO_write_reg(BDIC_REG_ADO_SYS,                                  0x05);
    shdisp_bdic_IO_write_reg(BDIC_REG_ALS_DATA0_SET,                            0xE0);
    shdisp_bdic_IO_write_reg(BDIC_REG_TEST_ADC1,                                0xFD);
    shdisp_bdic_IO_write_reg(BDIC_REG_TEST_ADC2,                        adj0_2_l_val);
    shdisp_bdic_IO_write_reg(BDIC_REG_TEST_ADC4,                        adj0_2_h_val);
    shdisp_bdic_IO_write_reg(BDIC_REG_TEST_ADC5,                                0x00);
    shdisp_bdic_IO_write_reg(BDIC_REG_TEST_ADC6,                                0x00);
    shdisp_bdic_IO_write_reg(BDIC_REG_TEST_ADC7,               s_state_str.als_shift);
    shdisp_bdic_IO_write_reg(BDIC_REG_ALS_ADJ0_L,           s_state_str.als_adj0_low);
    shdisp_bdic_IO_write_reg(BDIC_REG_ALS_ADJ0_H,          s_state_str.als_adj0_high);
    shdisp_bdic_IO_write_reg(BDIC_REG_ALS_DATA1_SET,                            0x01);
    shdisp_bdic_IO_write_reg(BDIC_REG_ALS_ADJ1_L,           s_state_str.als_adj1_low);
    shdisp_bdic_IO_write_reg(BDIC_REG_ALS_ADJ1_H,          s_state_str.als_adj1_high);
    shdisp_bdic_IO_write_reg(BDIC_REG_ALS_SHIFT,               s_state_str.als_shift);
    shdisp_bdic_IO_write_reg(BDIC_REG_ALS_INT,                                  0x14);
    shdisp_bdic_IO_write_reg(BDIC_REG_PSDATA_SET,                               0x02);
    shdisp_bdic_IO_write_reg(BDIC_REG_PS_HT_LSB,                                0x10);
    shdisp_bdic_IO_write_reg(BDIC_REG_PS_HT_MSB,                                0x00);
    shdisp_bdic_IO_write_reg(BDIC_REG_PS_LT_LSB,                                0x0F);
    shdisp_bdic_IO_write_reg(BDIC_REG_PS_LT_MSB,                                0x00);
    shdisp_bdic_IO_write_reg(BDIC_REG_I2C_TIMER,                                0xFF);
    shdisp_bdic_IO_write_reg(BDIC_REG_TEST0,                                    0x04);
    shdisp_bdic_IO_write_reg(BDIC_REG_I2C_SYS,                                  0x00);
    shdisp_bdic_IO_write_reg(BDIC_REG_I2C_DATA0,                                0x72);
    shdisp_bdic_IO_write_reg(BDIC_REG_SENSOR,                                   0x20);
    shdisp_bdic_IO_write_reg(BDIC_REG_SENSOR2,                                  0x00);
    shdisp_bdic_IO_write_reg(BDIC_REG_ADC_LCLIP,                                0x00);
    shdisp_bdic_IO_write_reg(BDIC_REG_CLEAR_OFFSET,         s_state_str.clear_offset);
    shdisp_bdic_IO_write_reg(BDIC_REG_IR_OFFSET,               s_state_str.ir_offset);

    shdisp_bdic_IO_write_reg(BDIC_REG_OPT0_HT_LSB,  shdisp_main_bkl_chg_high_tbl[0][1]);
    shdisp_bdic_IO_write_reg(BDIC_REG_OPT0_HT_MSB,  shdisp_main_bkl_chg_high_tbl[0][0]);
    shdisp_bdic_IO_write_reg(BDIC_REG_OPT1_LT_LSB,  shdisp_main_bkl_chg_high_tbl[1][1]);
    shdisp_bdic_IO_write_reg(BDIC_REG_OPT1_LT_MSB,  shdisp_main_bkl_chg_high_tbl[1][0]);
    shdisp_bdic_IO_write_reg(BDIC_REG_OPT1_HT_LSB,  shdisp_main_bkl_chg_high_tbl[2][1]);
    shdisp_bdic_IO_write_reg(BDIC_REG_OPT1_HT_MSB,  shdisp_main_bkl_chg_high_tbl[2][0]);
    shdisp_bdic_IO_write_reg(BDIC_REG_OPT2_LT_LSB,  shdisp_main_bkl_chg_high_tbl[3][1]);
    shdisp_bdic_IO_write_reg(BDIC_REG_OPT2_LT_MSB,  shdisp_main_bkl_chg_high_tbl[3][0]);
    shdisp_bdic_IO_write_reg(BDIC_REG_OPT2_HT_LSB,  shdisp_main_bkl_chg_high_tbl[4][1]);
    shdisp_bdic_IO_write_reg(BDIC_REG_OPT2_HT_MSB,  shdisp_main_bkl_chg_high_tbl[4][0]);
    shdisp_bdic_IO_write_reg(BDIC_REG_OPT3_LT_LSB,  shdisp_main_bkl_chg_high_tbl[5][1]);
    shdisp_bdic_IO_write_reg(BDIC_REG_OPT3_LT_MSB,  shdisp_main_bkl_chg_high_tbl[5][0]);
    shdisp_bdic_IO_write_reg(BDIC_REG_OPT3_HT_LSB,  shdisp_main_bkl_chg_high_tbl[6][1]);
    shdisp_bdic_IO_write_reg(BDIC_REG_OPT3_HT_MSB,  shdisp_main_bkl_chg_high_tbl[6][0]);
    shdisp_bdic_IO_write_reg(BDIC_REG_OPT4_LT_LSB,  shdisp_main_bkl_chg_high_tbl[7][1]);
    shdisp_bdic_IO_write_reg(BDIC_REG_OPT4_LT_MSB,  shdisp_main_bkl_chg_high_tbl[7][0]);
    shdisp_bdic_IO_write_reg(BDIC_REG_OPT4_HT_LSB,  shdisp_main_bkl_chg_high_tbl[8][1]);
    shdisp_bdic_IO_write_reg(BDIC_REG_OPT4_HT_MSB,  shdisp_main_bkl_chg_high_tbl[8][0]);
    shdisp_bdic_IO_write_reg(BDIC_REG_OPT5_LT_LSB,  shdisp_main_bkl_chg_high_tbl[9][1]);
    shdisp_bdic_IO_write_reg(BDIC_REG_OPT5_LT_MSB,  shdisp_main_bkl_chg_high_tbl[9][0]);
    shdisp_bdic_IO_write_reg(BDIC_REG_OPT5_HT_LSB, shdisp_main_bkl_chg_high_tbl[10][1]);
    shdisp_bdic_IO_write_reg(BDIC_REG_OPT5_HT_MSB, shdisp_main_bkl_chg_high_tbl[10][0]);
    shdisp_bdic_IO_write_reg(BDIC_REG_OPT6_LT_LSB, shdisp_main_bkl_chg_high_tbl[11][1]);
    shdisp_bdic_IO_write_reg(BDIC_REG_OPT6_LT_MSB, shdisp_main_bkl_chg_high_tbl[11][0]);
    shdisp_bdic_IO_write_reg(BDIC_REG_OPT6_HT_LSB, shdisp_main_bkl_chg_high_tbl[12][1]);
    shdisp_bdic_IO_write_reg(BDIC_REG_OPT6_HT_MSB, shdisp_main_bkl_chg_high_tbl[12][0]);
    shdisp_bdic_IO_write_reg(BDIC_REG_OPT7_LT_LSB, shdisp_main_bkl_chg_high_tbl[13][1]);
    shdisp_bdic_IO_write_reg(BDIC_REG_OPT7_LT_MSB, shdisp_main_bkl_chg_high_tbl[13][0]);

    shdisp_bdic_IO_write_reg(BDIC_REG_DETECTOR,                         detector_val);
#if defined(CONFIG_SHDISP_PANEL_SWITCH)
    shdisp_bdic_IO_write_reg(BDIC_REG_DCDC1_VLIM,            shdisp_bdic_reg_data[0]);
    shdisp_bdic_IO_write_reg(BDIC_REG_DCDC_SYS,              shdisp_bdic_reg_data[1]);
#else
    shdisp_bdic_IO_write_reg(BDIC_REG_DCDC1_VLIM,            BDIC_REG_DCDC1_VLIM_VAL);
    shdisp_bdic_IO_write_reg(BDIC_REG_DCDC_SYS,                BDIC_REG_DCDC_SYS_VAL);
#endif
    shdisp_bdic_IO_write_reg(BDIC_REG_DCDC2_VO,                BDIC_REG_DCDC2_VO_VAL);
    shdisp_bdic_IO_write_reg(BDIC_REG_DCDC2_SYS,                                0x0C);
    shdisp_bdic_IO_write_reg(BDIC_REG_TEST61,                                   0x00);
    shdisp_bdic_IO_write_reg(BDIC_REG_DCDC2_RESERVE,                            0x00);
    shdisp_bdic_IO_write_reg(BDIC_REG_CPM,                                      0x0E);
    shdisp_bdic_IO_write_reg(BDIC_REG_GPOD1,                                    0x00);
    shdisp_bdic_IO_write_reg(BDIC_REG_GDIR1,                                    0x33);
    shdisp_bdic_IO_write_reg(BDIC_REG_GPPE1,                                    0x00);
    shdisp_bdic_IO_write_reg(BDIC_REG_GIMR1,                                    0x00);
    shdisp_bdic_IO_write_reg(BDIC_REG_GIMR2,                                    0x00);
    shdisp_bdic_IO_write_reg(BDIC_REG_GIMR3,                                    0x00);
    shdisp_bdic_IO_write_reg(BDIC_REG_GIMF1,                                    0x00);
    shdisp_bdic_IO_write_reg(BDIC_REG_GIMF2,                                    0x00);
    shdisp_bdic_IO_write_reg(BDIC_REG_GIMF3,                                    0x00);
    shdisp_bdic_IO_write_reg(BDIC_REG_GPOMODE1,                                 0x00);
    shdisp_bdic_IO_write_reg(BDIC_REG_GPOMODE2,                                 0x00);
    shdisp_bdic_IO_write_reg(BDIC_REG_GPIMSK1,                                  0x01);
    shdisp_bdic_IO_write_reg(BDIC_REG_GPIMSK2,                                  0x00);
    shdisp_bdic_IO_write_reg(BDIC_REG_INT_CTRL,                                 0x04);
    
    return;
}


/* ------------------------------------------------------------------------- */
/* shdisp_bdic_PD_BDIC_active                                                */
/* ------------------------------------------------------------------------- */

static void shdisp_bdic_PD_BDIC_active(void)
{
    shdisp_bdic_IO_set_bit_reg(BDIC_REG_LPOSC, 0x01);
    
    shdisp_bdic_IO_set_bit_reg(BDIC_REG_SYSTEM2, 0x80);
    shdisp_SYS_delay_us(1000);
    
    return;
}


/* ------------------------------------------------------------------------- */
/* shdisp_bdic_PD_BDIC_standby                                               */
/* ------------------------------------------------------------------------- */

static void shdisp_bdic_PD_BDIC_standby(void)
{
    shdisp_bdic_IO_clr_bit_reg(BDIC_REG_SYSTEM2, 0x80);
    shdisp_SYS_delay_us(1000);
    
    return;
}


/* ------------------------------------------------------------------------- */
/* shdisp_bdic_PD_BKL_control                                                */
/* ------------------------------------------------------------------------- */

static void shdisp_bdic_PD_BKL_control(unsigned char request, int param)
{

    switch (request) {
    case SHDISP_BDIC_REQ_ACTIVE:
        break;
        
    case SHDISP_BDIC_REQ_STANDBY:
        break;
        
    case SHDISP_BDIC_REQ_START:
        shdisp_bdic_PD_REG_M1LED_set_value();
        shdisp_bdic_bkl_off_flg = 0;
        
        if (shdisp_bdic_pwm == SHDISP_BDIC_BKL_PWM_ENABLE) {
            shdisp_bdic_IO_msk_bit_reg(BDIC_REG_SYSTEM4, 0x05, 0x55);
        }
        else {
            shdisp_bdic_IO_msk_bit_reg(BDIC_REG_SYSTEM4, 0x01, 0x55);
        }
        
        if ( shdisp_bdic_bkl_mode == SHDISP_BDIC_BKL_MODE_FIX ) {
            shdisp_bdic_IO_set_bit_reg(BDIC_REG_OPT_MODE, 0x01);
        }
        
#if defined(CONFIG_SHDISP_PANEL_SWITCH)
        shdisp_bdic_IO_set_bit_reg(BDIC_REG_SYSTEM2, shdisp_bdic_reg_data[2]);
#else
        shdisp_bdic_IO_set_bit_reg(BDIC_REG_SYSTEM2, BDIC_REG_SYSTEM2_BKL);
#endif
        
        if ( shdisp_bdic_bkl_mode == SHDISP_BDIC_BKL_MODE_AUTO ) {
            shdisp_bdic_PD_REG_OPT_set_value(s_state_str.bdic_main_bkl_opt_mode_output);
            shdisp_bdic_IO_clr_bit_reg(BDIC_REG_SYSTEM4, 0x01);
            shdisp_bdic_IO_clr_bit_reg(BDIC_REG_OPT_MODE, 0x01);
        }
        break;

    case SHDISP_BDIC_REQ_PRE_START:
        shdisp_bdic_PD_REG_M1LED_set_value();
        shdisp_bdic_bkl_off_flg = 0;
        if (shdisp_bdic_pwm == SHDISP_BDIC_BKL_PWM_ENABLE) {
            shdisp_bdic_IO_msk_bit_reg(BDIC_REG_SYSTEM4, 0x05, 0x55);
        }
        else {
            shdisp_bdic_IO_msk_bit_reg(BDIC_REG_SYSTEM4, 0x01, 0x55);
        }
        
        if ( shdisp_bdic_bkl_mode == SHDISP_BDIC_BKL_MODE_FIX ) {
            shdisp_bdic_IO_set_bit_reg(BDIC_REG_OPT_MODE, 0x01);
        }
        shdisp_bdic_IO_set_bit_reg(BDIC_REG_SYSTEM2, BDIC_REG_SYSTEM2_BKL);
        
        if ( shdisp_bdic_bkl_mode == SHDISP_BDIC_BKL_MODE_AUTO ) {
            shdisp_bdic_PD_REG_OPT_set_value(s_state_str.bdic_main_bkl_opt_mode_output);
        }
        break;

    case SHDISP_BDIC_REQ_POST_START:
        if ( shdisp_bdic_bkl_mode == SHDISP_BDIC_BKL_MODE_AUTO ) {
            shdisp_bdic_IO_clr_bit_reg(BDIC_REG_SYSTEM4, 0x01);
            shdisp_bdic_IO_clr_bit_reg(BDIC_REG_OPT_MODE, 0x01);
        }
        break;
        
    case SHDISP_BDIC_REQ_STOP:
        shdisp_bdic_bkl_mode  = SHDISP_BDIC_BKL_MODE_OFF;
        shdisp_bdic_bkl_param = 0;
        shdisp_bdic_PD_REG_M1LED_set_value();
        shdisp_bdic_IO_set_bit_reg(BDIC_REG_OPT_MODE, 0x01);
#if defined(CONFIG_SHDISP_PANEL_SWITCH)
        shdisp_bdic_IO_clr_bit_reg(BDIC_REG_SYSTEM2, shdisp_bdic_reg_data[2]);
#else
        shdisp_bdic_IO_clr_bit_reg(BDIC_REG_SYSTEM2, BDIC_REG_SYSTEM2_BKL);
#endif
        shdisp_bdic_bkl_off_flg = 1;
        break;
        
    case SHDISP_BDIC_REQ_BKL_SET_MODE_OFF:
        shdisp_bdic_bkl_mode  = SHDISP_BDIC_BKL_MODE_OFF;
        shdisp_bdic_bkl_param = param;
        break;
        
    case SHDISP_BDIC_REQ_BKL_SET_MODE_FIX:
        shdisp_bdic_bkl_mode  = SHDISP_BDIC_BKL_MODE_FIX;
        shdisp_bdic_bkl_param = param;
        break;
        
    case SHDISP_BDIC_REQ_BKL_SET_MODE_AUTO:
        shdisp_bdic_bkl_mode  = SHDISP_BDIC_BKL_MODE_AUTO;
        shdisp_bdic_bkl_param = param;
        break;
        
    case SHDISP_BDIC_REQ_BKL_PWM_DISABLE:
        shdisp_bdic_pwm = SHDISP_BDIC_BKL_PWM_DISABLE;
        switch(shdisp_bdic_bkl_mode){
        case SHDISP_BDIC_BKL_MODE_OFF:
            break;
        case SHDISP_BDIC_BKL_MODE_FIX:
            shdisp_bdic_IO_msk_bit_reg(BDIC_REG_SYSTEM4, 0x01, 0x05);
            break;
        case SHDISP_BDIC_BKL_MODE_AUTO:
            shdisp_bdic_IO_msk_bit_reg(BDIC_REG_SYSTEM4, 0x00, 0x05);
            break;
        default:
            break;
        }
        break;
        
    case SHDISP_BDIC_REQ_BKL_PWM_ENABLE:
        shdisp_bdic_pwm = SHDISP_BDIC_BKL_PWM_ENABLE;
        switch(shdisp_bdic_bkl_mode){
        case SHDISP_BDIC_BKL_MODE_OFF:
            break;
        case SHDISP_BDIC_BKL_MODE_FIX:
            shdisp_bdic_IO_msk_bit_reg(BDIC_REG_SYSTEM4, 0x05, 0x05);
            break;
        case SHDISP_BDIC_BKL_MODE_AUTO:
            shdisp_bdic_IO_msk_bit_reg(BDIC_REG_SYSTEM4, 0x04, 0x05);
            break;
        default:
            break;
        }
        break;
        
    case SHDISP_BDIC_REQ_BKL_DTV_OFF:
        shdisp_bdic_dtv = SHDISP_BDIC_BKL_DTV_OFF;
        break;
        
    case SHDISP_BDIC_REQ_BKL_DTV_ON:
        shdisp_bdic_dtv = SHDISP_BDIC_BKL_DTV_ON;
        break;
        
    case SHDISP_BDIC_REQ_BKL_EMG_OFF:
        shdisp_bdic_emg = SHDISP_BDIC_BKL_EMG_OFF;
        break;
        
    case SHDISP_BDIC_REQ_BKL_EMG_ON:
        shdisp_bdic_emg = SHDISP_BDIC_BKL_EMG_ON;
        break;
        
    case SHDISP_BDIC_REQ_BKL_ECO_OFF:
        shdisp_bdic_eco = SHDISP_BDIC_BKL_ECO_OFF;
        break;
        
    case SHDISP_BDIC_REQ_BKL_ECO_ON:
        shdisp_bdic_eco = SHDISP_BDIC_BKL_ECO_ON;
        break;
        
    case SHDISP_BDIC_REQ_BKL_CHG_OFF:
        shdisp_bdic_chg = SHDISP_BDIC_BKL_CHG_OFF;
        break;
        
    case SHDISP_BDIC_REQ_BKL_CHG_ON:
        shdisp_bdic_chg = SHDISP_BDIC_BKL_CHG_ON;
        break;
        
    default:
        break;
    }
    
    return;
}


/* ------------------------------------------------------------------------- */
/* shdisp_bdic_PD_GPIO_control                                               */
/* ------------------------------------------------------------------------- */

static void shdisp_bdic_PD_GPIO_control(unsigned char port, unsigned char status)
{
    unsigned char    reg;
    unsigned char    bit;

    switch (port) {
    case SHDISP_BDIC_GPIO_GPOD0:
        reg = BDIC_REG_GPOD1;
        bit = 0x01;
        break;
    case SHDISP_BDIC_GPIO_GPOD1:
        reg = BDIC_REG_GPOD1;
        bit = 0x02;
        break;
    case SHDISP_BDIC_GPIO_GPOD2:
        reg = BDIC_REG_GPOD1;
        bit = 0x04;
        break;
    case SHDISP_BDIC_GPIO_GPOD3:
        reg = BDIC_REG_GPOD1;
        bit = 0x08;
        break;
    case SHDISP_BDIC_GPIO_GPOD4:
        reg = BDIC_REG_GPOD1;
        bit = 0x10;
        break;
    case SHDISP_BDIC_GPIO_GPOD5:
        reg = BDIC_REG_GPOD1;
        bit = 0x20;
        break;
    default:
        return;
    }
    if ( status == SHDISP_BDIC_GPIO_HIGH ) {
        shdisp_bdic_IO_set_bit_reg( reg, bit );
    }
    else
    {
        shdisp_bdic_IO_clr_bit_reg( reg, bit );
    }

    return;
}


/* ------------------------------------------------------------------------- */
/* shdisp_bdic_PD_TRI_LED_control                                            */
/* ------------------------------------------------------------------------- */

static void shdisp_bdic_PD_TRI_LED_control(unsigned char request, int param)
{

    switch (request) {
    case SHDISP_BDIC_REQ_ACTIVE:
        break;
        
    case SHDISP_BDIC_REQ_STANDBY:
        break;
        
    case SHDISP_BDIC_REQ_START:
        shdisp_bdic_IO_clr_bit_reg(BDIC_REG_TIMERSTART, 0x01);
        shdisp_bdic_IO_clr_bit_reg(BDIC_REG_SYSTEM3, 0x01);
        shdisp_SYS_delay_us(5500);
        shdisp_bdic_PD_TRI_LED_set_chdig();
        shdisp_bdic_PD_TRI_LED_set_anime();
        shdisp_bdic_IO_set_bit_reg(BDIC_REG_SYSTEM3, 0x01);
        if (shdisp_bdic_tri_led_mode == SHDISP_BDIC_TRI_LED_MODE_NORMAL) {
            shdisp_bdic_IO_clr_bit_reg(BDIC_REG_TIMERSTART, 0x01);
        }
        else {
            shdisp_bdic_IO_set_bit_reg(BDIC_REG_TIMERSTART, 0x01);
            shdisp_SYS_delay_us(6000);
        }
        break;
        
    case SHDISP_BDIC_REQ_STOP:
        shdisp_bdic_IO_clr_bit_reg(BDIC_REG_TIMERSTART, 0x01);
        shdisp_bdic_IO_clr_bit_reg(BDIC_REG_SYSTEM3, 0x01);
        shdisp_SYS_delay_us(5500);
        shdisp_bdic_tri_led_mode     = SHDISP_BDIC_TRI_LED_MODE_NORMAL;
        shdisp_bdic_tri_led_color    = 0;
        shdisp_bdic_tri_led_ontime   = 0;
        shdisp_bdic_tri_led_interval = 0;
        break;
        
    case SHDISP_BDIC_REQ_TRI_LED_SET_MODE_NORMAL:
        shdisp_bdic_tri_led_mode  = SHDISP_BDIC_TRI_LED_MODE_NORMAL;
        shdisp_bdic_tri_led_color = (unsigned char)param;
        break;
        
    case SHDISP_BDIC_REQ_TRI_LED_SET_MODE_BLINK:
        shdisp_bdic_tri_led_mode  = SHDISP_BDIC_TRI_LED_MODE_BLINK;
        shdisp_bdic_tri_led_color = (unsigned char)param;
        break;
        
    case SHDISP_BDIC_REQ_TRI_LED_SET_MODE_FIREFLY:
        shdisp_bdic_tri_led_mode  = SHDISP_BDIC_TRI_LED_MODE_FIREFLY;
        shdisp_bdic_tri_led_color = (unsigned char)param;
        break;
        
    case SHDISP_BDIC_REQ_TRI_LED_SET_ONTIME:
        shdisp_bdic_tri_led_ontime = param;
        break;
        
    case SHDISP_BDIC_REQ_TRI_LED_SET_INTERVAL:
        shdisp_bdic_tri_led_interval = param;
        break;
        
    default:
        break;
    }
    
    return;
}


/* ------------------------------------------------------------------------- */
/* shdisp_bdic_PD_TRI_LED_set_anime                                          */
/* ------------------------------------------------------------------------- */

static void shdisp_bdic_PD_TRI_LED_set_anime(void)
{
    unsigned char timeer1_val;
    unsigned char ch_set1_val;
    unsigned char ch_set2_val;
    
    switch (shdisp_bdic_tri_led_mode) {
    case SHDISP_BDIC_TRI_LED_MODE_NORMAL:
        shdisp_bdic_IO_set_bit_reg(BDIC_REG_CH0_SET1, 0x20);
        shdisp_bdic_IO_set_bit_reg(BDIC_REG_CH1_SET1, 0x20);
        shdisp_bdic_IO_set_bit_reg(BDIC_REG_CH2_SET1, 0x20);
        break;
        
    case SHDISP_BDIC_TRI_LED_MODE_BLINK:
        timeer1_val = (unsigned char)(shdisp_bdic_tri_led_interval << 4);
        ch_set1_val = 0x46;
        ch_set2_val = (unsigned char)(shdisp_bdic_tri_led_ontime);
        shdisp_bdic_IO_msk_bit_reg(BDIC_REG_CH0_SET1, ch_set1_val, 0x6F);
        shdisp_bdic_IO_write_reg(BDIC_REG_CH0_SET2, ch_set2_val);
        shdisp_bdic_IO_msk_bit_reg(BDIC_REG_CH1_SET1, ch_set1_val, 0x6F);
        shdisp_bdic_IO_write_reg(BDIC_REG_CH1_SET2, ch_set2_val);
        shdisp_bdic_IO_msk_bit_reg(BDIC_REG_CH2_SET1, ch_set1_val, 0x6F);
        shdisp_bdic_IO_write_reg(BDIC_REG_CH2_SET2, ch_set2_val);
        shdisp_bdic_IO_msk_bit_reg(BDIC_REG_TIMEER1, (unsigned char)timeer1_val, 0xF7);
        break;
        
    case SHDISP_BDIC_TRI_LED_MODE_FIREFLY:
        timeer1_val = (unsigned char)(shdisp_bdic_tri_led_interval << 4);
        ch_set1_val = 0x06;
        ch_set2_val = (unsigned char)(shdisp_bdic_tri_led_ontime);
        shdisp_bdic_IO_msk_bit_reg(BDIC_REG_CH0_SET1, ch_set1_val, 0x6F);
        shdisp_bdic_IO_write_reg(BDIC_REG_CH0_SET2, ch_set2_val);
        shdisp_bdic_IO_msk_bit_reg(BDIC_REG_CH1_SET1, ch_set1_val, 0x6F);
        shdisp_bdic_IO_write_reg(BDIC_REG_CH1_SET2, ch_set2_val);
        shdisp_bdic_IO_msk_bit_reg(BDIC_REG_CH2_SET1, ch_set1_val, 0x6F);
        shdisp_bdic_IO_write_reg(BDIC_REG_CH2_SET2, ch_set2_val);
        shdisp_bdic_IO_msk_bit_reg(BDIC_REG_TIMEER1, (unsigned char)timeer1_val, 0xF7);
        break;
        
    default:
        break;
    }
    
    return;
}


/* ------------------------------------------------------------------------- */
/* shdisp_bdic_PD_TRI_LED_set_chdig                                          */
/* ------------------------------------------------------------------------- */

static void shdisp_bdic_PD_TRI_LED_set_chdig(void)
{
    switch (shdisp_bdic_tri_led_mode) {
    case SHDISP_BDIC_TRI_LED_MODE_NORMAL:
        shdisp_bdic_IO_write_reg(BDIC_REG_CH0_A, shdisp_triple_led_tbl[shdisp_bdic_tri_led_color][0]);
        shdisp_bdic_IO_write_reg(BDIC_REG_CH1_A, shdisp_triple_led_tbl[shdisp_bdic_tri_led_color][1]);
        shdisp_bdic_IO_write_reg(BDIC_REG_CH2_A, shdisp_triple_led_tbl[shdisp_bdic_tri_led_color][2]);
        shdisp_bdic_IO_write_reg(BDIC_REG_CH0_B, 0x00);
        shdisp_bdic_IO_write_reg(BDIC_REG_CH1_B, 0x00);
        shdisp_bdic_IO_write_reg(BDIC_REG_CH2_B, 0x00);
        shdisp_bdic_IO_write_reg(BDIC_REG_CH0_C, 0x00);
        shdisp_bdic_IO_write_reg(BDIC_REG_CH1_C, 0x00);
        shdisp_bdic_IO_write_reg(BDIC_REG_CH2_C, 0x00);
        break;
        
    case SHDISP_BDIC_TRI_LED_MODE_BLINK:
    case SHDISP_BDIC_TRI_LED_MODE_FIREFLY:
        shdisp_bdic_IO_write_reg(BDIC_REG_CH0_A, shdisp_triple_led_anime_tbl[0][shdisp_bdic_tri_led_color][0]);
        shdisp_bdic_IO_write_reg(BDIC_REG_CH1_A, shdisp_triple_led_anime_tbl[0][shdisp_bdic_tri_led_color][1]);
        shdisp_bdic_IO_write_reg(BDIC_REG_CH2_A, shdisp_triple_led_anime_tbl[0][shdisp_bdic_tri_led_color][2]);
        shdisp_bdic_IO_write_reg(BDIC_REG_CH0_B, shdisp_triple_led_anime_tbl[1][shdisp_bdic_tri_led_color][0]);
        shdisp_bdic_IO_write_reg(BDIC_REG_CH1_B, shdisp_triple_led_anime_tbl[1][shdisp_bdic_tri_led_color][1]);
        shdisp_bdic_IO_write_reg(BDIC_REG_CH2_B, shdisp_triple_led_anime_tbl[1][shdisp_bdic_tri_led_color][2]);
        shdisp_bdic_IO_write_reg(BDIC_REG_CH0_C, shdisp_triple_led_anime_tbl[0][shdisp_bdic_tri_led_color][0]);
        shdisp_bdic_IO_write_reg(BDIC_REG_CH1_C, shdisp_triple_led_anime_tbl[0][shdisp_bdic_tri_led_color][1]);
        shdisp_bdic_IO_write_reg(BDIC_REG_CH2_C, shdisp_triple_led_anime_tbl[0][shdisp_bdic_tri_led_color][2]);
        break;
        
    default:
        break;
    }
    
    return;
}


/* ------------------------------------------------------------------------- */
/* shdisp_bdic_PD_REG_M1LED_set_value                                        */
/* ------------------------------------------------------------------------- */

static void shdisp_bdic_PD_REG_M1LED_set_value(void)
{
    unsigned char value = 0x00;
    int mode = 0;

    switch (shdisp_bdic_bkl_mode) {
    case SHDISP_BDIC_BKL_MODE_OFF:
        value = 0x00;
        break;
    case SHDISP_BDIC_BKL_MODE_FIX:
        shdisp_bdic_LD_LCD_BKL_get_mode(&mode);
        
        if (shdisp_bdic_dtv == SHDISP_BDIC_BKL_DTV_ON) {
            value = shdisp_main_dtv_bkl_tbl[shdisp_bdic_bkl_param][mode] + shdisp_main_bkl_adj_tbl[s_state_str.bkl_adjust_val];
        }
        else {
            value = shdisp_main_bkl_tbl[shdisp_bdic_bkl_param][mode] + shdisp_main_bkl_adj_tbl[s_state_str.bkl_adjust_val];
        }
        break;
    case SHDISP_BDIC_BKL_MODE_AUTO:
        if( shdisp_bdic_bkl_off_flg == 0){
            value = shdisp_bdic_get_opt_value(s_state_str.bdic_main_bkl_opt_mode_output);
            break;
        }
        
        shdisp_bdic_LD_LCD_BKL_get_mode(&mode);
        
        if (shdisp_bdic_dtv == SHDISP_BDIC_BKL_DTV_ON) {
            value = shdisp_main_dtv_bkl_tbl[SHDISP_MAIN_BKL_PARAM_11][mode] + shdisp_main_bkl_adj_tbl[s_state_str.bkl_adjust_val];
        }
        else {
            value = shdisp_main_bkl_tbl[SHDISP_MAIN_BKL_PARAM_11][mode] + shdisp_main_bkl_adj_tbl[s_state_str.bkl_adjust_val];
        }
        break;
    default:
        break;
    }
    
    shdisp_bdic_IO_write_reg(BDIC_REG_M1LED , value);
#ifndef CONFIG_MACH_LYNX_DL15
    shdisp_bdic_IO_write_reg(BDIC_REG_M2LED , value);
#endif /* CONFIG_MACH_LYNX_DL15 */
    
    return;
}

static unsigned char shdisp_bdic_get_opt_value(int table)
{
    unsigned char  M1REG_value=0;
    unsigned char ado_idx=0;
    int mode = 0;

    shdisp_bdic_IO_read_reg(BDIC_REG_ADO_INDEX, &ado_idx);
    ado_idx = ado_idx & 0x1F;
#ifdef SHDISP_SW_BDIC_IRQ_LOG
    printk ("[BDIC DBG] get_opt_value:ado_idx == %x\n",ado_idx);
#endif
    shdisp_bdic_LD_LCD_BKL_get_mode(&mode);

    if (ado_idx > 0x0F){
        ado_idx = 0x0F;
    }

    if( table == SHDISP_BDIC_MAIN_BKL_OPT_LOW ) {
        if (shdisp_bdic_dtv == SHDISP_BDIC_BKL_DTV_ON) 
            M1REG_value = shdisp_main_dtv_bkl_opt_low_tbl[ado_idx][1+mode];
        else
            M1REG_value = shdisp_main_bkl_opt_low_tbl[ado_idx][1+mode];
    }
    else {
        if (shdisp_bdic_dtv == SHDISP_BDIC_BKL_DTV_ON) 
            M1REG_value = shdisp_main_dtv_bkl_opt_high_tbl[ado_idx][1+mode];
        else
            M1REG_value = shdisp_main_bkl_opt_high_tbl[ado_idx][1+mode];
    }
#ifdef SHDISP_SW_BDIC_IRQ_LOG
    printk("[BDIC DBG] get_opt_value:M1REG_value=%x\n",M1REG_value);
#endif
    return M1REG_value;
}


/* ------------------------------------------------------------------------- */
/* shdisp_bdic_PD_REG_OPT_set_value                                          */
/* ------------------------------------------------------------------------- */

static void shdisp_bdic_PD_REG_OPT_set_value( int table )
{
    int idx;
    int mode = 0;
    unsigned char shdisp_bkl_tbl[SHDISP_BKL_AUTO_TBL_NUM][1+NUM_SHDISP_BKL_TBL_MODE];

    shdisp_bdic_LD_LCD_BKL_get_mode(&mode);
    
    if( table == SHDISP_BDIC_MAIN_BKL_OPT_LOW ){
#ifdef SHDISP_SW_BDIC_IRQ_LOG
        printk("[SHDISP DBG]opt low table write(mode=%d)\n", mode);
#endif
        s_state_str.bdic_main_bkl_opt_mode_output = SHDISP_BDIC_MAIN_BKL_OPT_LOW;
        shdisp_bdic_IO_write_reg(BDIC_REG_ALS_INT, s_state_str.shdisp_lux_change_level1);
        if (shdisp_bdic_dtv == SHDISP_BDIC_BKL_DTV_ON)
            memcpy( shdisp_bkl_tbl, shdisp_main_dtv_bkl_opt_low_tbl, sizeof( shdisp_main_dtv_bkl_opt_low_tbl ));
        else
            memcpy( shdisp_bkl_tbl, shdisp_main_bkl_opt_low_tbl, sizeof( shdisp_main_bkl_opt_low_tbl ));
    }
    else if( table == SHDISP_BDIC_MAIN_BKL_OPT_HIGH ){
#ifdef SHDISP_SW_BDIC_IRQ_LOG
        printk("[SHDISP DBG]opt high table write(mode=%d)\n", mode);
#endif
        s_state_str.bdic_main_bkl_opt_mode_output = SHDISP_BDIC_MAIN_BKL_OPT_HIGH;
        shdisp_bdic_IO_write_reg(BDIC_REG_ALS_INT, s_state_str.shdisp_lux_change_level2);
        if (shdisp_bdic_dtv == SHDISP_BDIC_BKL_DTV_ON)
            memcpy( shdisp_bkl_tbl, shdisp_main_dtv_bkl_opt_high_tbl, sizeof( shdisp_main_dtv_bkl_opt_high_tbl ));
        else
            memcpy( shdisp_bkl_tbl, shdisp_main_bkl_opt_high_tbl, sizeof( shdisp_main_bkl_opt_high_tbl ));
    }
    else
        return;


    for( idx = 0; idx < SHDISP_BKL_AUTO_TBL_NUM; idx++ ){
        shdisp_bdic_IO_write_reg( shdisp_bkl_tbl[idx][0],
                                  shdisp_bkl_tbl[idx][1+mode]);
    }
    
    return;
}


/* ------------------------------------------------------------------------- */
/* shdisp_bdic_PD_REG_OPT_temp_set                                           */
/* ------------------------------------------------------------------------- */

static void shdisp_bdic_PD_REG_OPT_temp_set( unsigned char value )
{
    unsigned char idx;
    unsigned char adr = BDIC_REG_OPT0;

    for( idx = 0; idx < 16; idx++ ){
        shdisp_bdic_IO_write_reg( adr + idx, value);
    }
    return;
}


/* ------------------------------------------------------------------------- */
/* shdisp_bdic_PD_REG_ADO_get_opt                                            */
/* ------------------------------------------------------------------------- */

static void shdisp_bdic_PD_REG_ADO_get_opt(unsigned short *value)
{
    unsigned char rdata[2];
    unsigned short tmp_data[2];
    unsigned char  reg_sensor;
    static unsigned short before_val = 0x0000;
    
    rdata[0] = 0x00;
    rdata[1] = 0x00;
    
    
    shdisp_bdic_IO_read_reg(BDIC_REG_SENSOR, &reg_sensor);
    if( reg_sensor & 0x01 ){
#ifdef SHDISP_SW_BDIC_IRQ_LOG
        printk("[SHDISP DBG] ADO_get_opt XEN_SENSOR=ON before_ADO=0x%04X\n", before_val);
#endif
    }
    else{
        shdisp_bdic_IO_multi_read_reg(BDIC_REG_ADOL, rdata, 2);
    
        tmp_data[0] = (unsigned short)rdata[0];
        tmp_data[1] = (unsigned short)rdata[1];
    
        before_val = (tmp_data[1] << 8) | (tmp_data[0]);
#ifdef SHDISP_SW_BDIC_IRQ_LOG
        printk("[SHDISP DBG] ADO_get_opt XEN_SENSOR=OFF renew_ADO=0x%04X\n", before_val);
#endif
    }
    *value = before_val;
    return;
}


/* ------------------------------------------------------------------------- */
/* shdisp_bdic_PD_PHOTO_SENSOR_control                                       */
/* ------------------------------------------------------------------------- */

static void shdisp_bdic_PD_PHOTO_SENSOR_control(unsigned long device, unsigned char request)
{
    int state = 0;
    unsigned char   sensor = SHDISP_BDIC_SENSOR_TYPE_PHOTO;
    unsigned long   temp_user;
    unsigned char   remask;

    switch (request) {
    case SHDISP_BDIC_REQ_ACTIVE:
        shdisp_bdic_PD_SENSOR_control(SHDISP_BDIC_REQ_ACTIVE, sensor);
        break;
    case SHDISP_BDIC_REQ_STANDBY:
        shdisp_bdic_PD_SENSOR_control(SHDISP_BDIC_REQ_STANDBY, sensor);
        break;
    case SHDISP_BDIC_REQ_START:
        if (s_state_str.photo_sensor_user != SHDISP_BDIC_DEVICE_NONE){
            s_state_str.photo_sensor_user |= device;
            break;
        }
        shdisp_bdic_ps_req_mask( &remask );
        shdisp_bdic_PD_SENSOR_control(SHDISP_BDIC_REQ_START, sensor);
        s_state_str.photo_sensor_user |= device;
        shdisp_bdic_IO_clr_bit_reg(BDIC_REG_ALS_DATA0_SET, SHDISP_BDIC_I2C_ADO_UPDATE_MASK);
        shdisp_bdic_IO_write_reg(BDIC_REG_I2C_DATA6, 0x0C);
        if (SHDISP_INT_ENABLE_GFAC & SHDISP_BDIC_INT_GFAC_PS2) {
            shdisp_bdic_IO_write_reg(BDIC_REG_I2C_SET, 0x50);
        }
        else {
            shdisp_bdic_IO_write_reg(BDIC_REG_I2C_SET, 0x30);
        }
        shdisp_bdic_IO_write_reg(BDIC_REG_TEST0, 0x00);
        shdisp_bdic_IO_set_bit_reg(BDIC_REG_I2C_START, 0x10);
        shdisp_SYS_delay_us(150000);
        shdisp_bdic_IO_write_reg(BDIC_REG_TEST0, 0x04);
        shdisp_bdic_ps_req_restart( remask );
        break;
    case SHDISP_BDIC_REQ_STOP:
        if (s_state_str.photo_sensor_user == SHDISP_BDIC_DEVICE_NONE)
            break;
            
        temp_user = s_state_str.photo_sensor_user;
        temp_user &= ~device;
        if( temp_user != SHDISP_BDIC_DEVICE_NONE ){
            s_state_str.photo_sensor_user &= ~device;
            break;
        }
        
        shdisp_bdic_PD_SENSOR_control(SHDISP_BDIC_REQ_STOP, sensor);
        s_state_str.photo_sensor_user &= ~device;
        if (SHDISP_INT_ENABLE_GFAC & SHDISP_BDIC_INT_GFAC_PS2) {
            shdisp_bdic_LD_SENSOR_get_state(&state);
            if (state == SHDISP_SENSOR_STATE_PROX_OFF_ALC_OFF) {
                shdisp_bdic_IO_clr_bit_reg(BDIC_REG_I2C_START, 0x10);
                shdisp_bdic_IO_set_bit_reg(BDIC_REG_ALS_DATA0_SET, SHDISP_BDIC_I2C_ADO_UPDATE_MASK);
                shdisp_SYS_delay_us(1000);
            }
        }
        else {
            if (s_state_str.photo_sensor_user == SHDISP_BDIC_DEVICE_NONE) {
                shdisp_bdic_IO_clr_bit_reg(BDIC_REG_I2C_START, 0x10);
                shdisp_bdic_IO_set_bit_reg(BDIC_REG_ALS_DATA0_SET, SHDISP_BDIC_I2C_ADO_UPDATE_MASK);
                shdisp_SYS_delay_us(1000);
            }
        }
        break;
    case SHDISP_BDIC_REQ_PHOTO_SENSOR_CONFIG:
        break;
    default:
        break;
    }
    return;
}


/* ------------------------------------------------------------------------- */
/* shdisp_bdic_PD_PROX_SENSOR_control                                        */
/* ------------------------------------------------------------------------- */

static void shdisp_bdic_PD_PROX_SENSOR_control(unsigned long device, unsigned char request)
{
    int state = 0;
    unsigned char   sensor = SHDISP_BDIC_SENSOR_TYPE_PROX;
    unsigned long   temp_user;
    unsigned char   remask;

    switch (request) {
    case SHDISP_BDIC_REQ_ACTIVE:
        shdisp_bdic_PD_SENSOR_control(SHDISP_BDIC_REQ_ACTIVE, sensor);
        break;
    case SHDISP_BDIC_REQ_STANDBY:
        shdisp_bdic_PD_SENSOR_control(SHDISP_BDIC_REQ_STANDBY, sensor);
        break;
    case SHDISP_BDIC_REQ_START:
        if (s_state_str.prox_sensor_user != SHDISP_BDIC_DEVICE_NONE){
            s_state_str.prox_sensor_user |= device;
            break;
        }
        if (SHDISP_INT_ENABLE_GFAC & SHDISP_BDIC_INT_GFAC_PS2) {
            shdisp_bdic_ps_req_mask( &remask );
        }
        shdisp_bdic_PD_SENSOR_control(SHDISP_BDIC_REQ_START, sensor);
        s_state_str.prox_sensor_user |= device;
        if (SHDISP_INT_ENABLE_GFAC & SHDISP_BDIC_INT_GFAC_PS2) {
            shdisp_bdic_IO_write_reg(BDIC_REG_I2C_DATA6, 0x0C);
            shdisp_bdic_IO_write_reg(BDIC_REG_I2C_SET, 0x50);
            shdisp_bdic_IO_set_bit_reg(BDIC_REG_I2C_START, 0x10);
            shdisp_bdic_ps_req_restart( remask );
        }
        shdisp_SYS_delay_us(20000);
        break;
    case SHDISP_BDIC_REQ_STOP:
        if (s_state_str.prox_sensor_user == SHDISP_BDIC_DEVICE_NONE)
            break;
        temp_user = s_state_str.prox_sensor_user;
        temp_user &= ~device;
        if( temp_user != SHDISP_BDIC_DEVICE_NONE ){
            s_state_str.prox_sensor_user &= ~device;
            break;
        }
        shdisp_bdic_PD_SENSOR_control(SHDISP_BDIC_REQ_STOP, sensor);
        s_state_str.prox_sensor_user &= ~device;
        shdisp_bdic_LD_SENSOR_get_state(&state);
        if (SHDISP_INT_ENABLE_GFAC & SHDISP_BDIC_INT_GFAC_PS2 && 
            state == SHDISP_SENSOR_STATE_PROX_OFF_ALC_OFF) {
            shdisp_bdic_IO_clr_bit_reg(BDIC_REG_I2C_START, 0x10);
            shdisp_bdic_IO_set_bit_reg(BDIC_REG_ALS_DATA0_SET, SHDISP_BDIC_I2C_ADO_UPDATE_MASK);
            shdisp_SYS_delay_us(1000);
        }

        break;
    default:
        break;
    }
    return;
}


/* ------------------------------------------------------------------------- */
/* shdisp_bdic_PD_SENSOR_control                                             */
/* ------------------------------------------------------------------------- */

static void shdisp_bdic_PD_SENSOR_control(unsigned char request, unsigned char sensor)
{
    int state = 0;
    unsigned char als_shift = 0x00;
    unsigned char als_range = 0x00;
    unsigned char remask;
    unsigned char wBuf[SHDISP_BDIC_I2C_WBUF_MAX];
    
    shdisp_bdic_LD_SENSOR_get_state(&state);
    shdisp_bdic_LD_PHOTO_SENSOR_bias_als_adj(s_state_str.als_shift ,&als_shift);

    if( s_state_str.bdic_main_bkl_opt_mode_output == SHDISP_BDIC_MAIN_BKL_OPT_LOW )
        als_range = 0x04;
    else
        als_range = 0x06;

#ifdef SHDISP_SW_BDIC_IRQ_LOG
    printk("[SHDISP DBG] shdisp_bdic_PD_SENSOR_control : request=%d, state=%d, sensor=%d als_range=0x%02X\n", (int)request, state, (int)sensor, als_range);
#endif
    
    switch (request) {
    case SHDISP_BDIC_REQ_ACTIVE:
        switch (state) {
        case SHDISP_SENSOR_STATE_PROX_OFF_ALC_OFF:
            shdisp_bdic_IO_set_bit_reg(BDIC_REG_SENSOR, 0x10);
            if (SHDISP_INT_ENABLE_GFAC & SHDISP_BDIC_INT_GFAC_PS2) {
                shdisp_bdic_ps_req_mask( &remask );
                wBuf[0] = SENSOR_REG_COMMAND1;
                wBuf[1] = 0x00;
                wBuf[2] = 0x00;
                wBuf[3] = 0x10;
                wBuf[4] = 0xEC;
                shdisp_photo_sensor_IO_burst_write_reg(wBuf, 5);
                shdisp_photo_sensor_IO_write_reg(SENSOR_REG_COMMAND1, 0xE0);
                shdisp_bdic_IO_write_reg(BDIC_REG_I2C_DATA6, 0x0C);
                shdisp_bdic_IO_write_reg(BDIC_REG_I2C_SET, 0x50);
                shdisp_bdic_IO_set_bit_reg(BDIC_REG_I2C_START, 0x10);
                shdisp_SYS_delay_us(20000);
                shdisp_bdic_ps_req_restart( remask );
                shdisp_bdic_IO_clr_bit_reg(BDIC_REG_I2C_START, 0x10);
                shdisp_photo_sensor_IO_write_reg(SENSOR_REG_COMMAND1, 0x00);
                shdisp_bdic_IO_write_reg(BDIC_REG_PS_HT_LSB, 0x01);
                shdisp_bdic_IO_write_reg(BDIC_REG_PS_HT_MSB, 0x00);
                shdisp_bdic_IO_write_reg(BDIC_REG_PS_LT_LSB, 0x00);
                shdisp_bdic_IO_write_reg(BDIC_REG_PS_LT_MSB, 0x00);
                shdisp_photo_sensor_IO_write_threshold();
            }
            break;
        case SHDISP_SENSOR_STATE_PROX_ON_ALC_OFF:
        case SHDISP_SENSOR_STATE_PROX_OFF_ALC_ON:
        case SHDISP_SENSOR_STATE_PROX_ON_ALC_ON:
        default:
            break;
        }
        break;
    case SHDISP_BDIC_REQ_STANDBY:
        switch (state) {
        case SHDISP_SENSOR_STATE_PROX_OFF_ALC_OFF:
            shdisp_bdic_IO_clr_bit_reg(BDIC_REG_SENSOR, 0x10);
            break;
        case SHDISP_SENSOR_STATE_PROX_ON_ALC_OFF:
        case SHDISP_SENSOR_STATE_PROX_OFF_ALC_ON:
        case SHDISP_SENSOR_STATE_PROX_ON_ALC_ON:
        default:
            break;
        }
        break;
        
    case SHDISP_BDIC_REQ_START:
        switch (state) {
        case SHDISP_SENSOR_STATE_PROX_OFF_ALC_OFF:
            if( sensor == SHDISP_BDIC_SENSOR_TYPE_PROX ){
                shdisp_bdic_IO_write_reg(BDIC_REG_ALS_SHIFT, s_state_str.als_shift);
                wBuf[0] = SENSOR_REG_COMMAND1;
                wBuf[1] = 0x00;
                wBuf[2] = 0x00;
                wBuf[3] = 0x10;
                wBuf[4] = 0xEC;
                shdisp_photo_sensor_IO_burst_write_reg(wBuf, 5);
                shdisp_photo_sensor_IO_write_threshold();
                shdisp_photo_sensor_IO_write_reg(SENSOR_REG_COMMAND1, 0xE0);
            }
            else{
                shdisp_bdic_IO_write_reg(BDIC_REG_ALS_SHIFT, s_state_str.als_shift);
                wBuf[0] = SENSOR_REG_COMMAND1;
                wBuf[1] = 0x00;
                wBuf[2] = (0x18 | als_range);
                wBuf[3] = 0x00;
                wBuf[4] = 0x00;
                shdisp_photo_sensor_IO_burst_write_reg(wBuf, 5);
                shdisp_photo_sensor_IO_write_reg(SENSOR_REG_COMMAND1, 0xD0);
            }
            break;
        case SHDISP_SENSOR_STATE_PROX_ON_ALC_OFF:
            if( sensor == SHDISP_BDIC_SENSOR_TYPE_PROX ){
#ifdef SHDISP_SW_BDIC_I2C_RWLOG
                printk("[SHDISP BDIC] shdisp_bdic_PD_SENSOR_control: prox sensor started\n");
#endif
            }
            else{
                shdisp_bdic_IO_write_reg(BDIC_REG_ALS_SHIFT, als_shift);
                wBuf[0] = SENSOR_REG_COMMAND1;
                wBuf[1] = 0x0C;
                wBuf[2] = (0x20 | als_range);
                wBuf[3] = 0x10;
                wBuf[4] = 0xEC;
                shdisp_photo_sensor_IO_burst_write_reg(wBuf, 5);
                shdisp_photo_sensor_IO_write_threshold();
                shdisp_photo_sensor_IO_write_reg(SENSOR_REG_COMMAND1, 0xCC);
            }
            break;
        case SHDISP_SENSOR_STATE_PROX_OFF_ALC_ON:
            if( sensor == SHDISP_BDIC_SENSOR_TYPE_PROX ){
                shdisp_bdic_IO_write_reg(BDIC_REG_ALS_SHIFT, als_shift);
                wBuf[0] = SENSOR_REG_COMMAND1;
                wBuf[1] = 0x00;
                wBuf[2] = (0x20 | als_range);
                wBuf[3] = 0x10;
                wBuf[4] = 0xEC;
                shdisp_photo_sensor_IO_burst_write_reg(wBuf, 5);
                shdisp_photo_sensor_IO_write_threshold();
                shdisp_photo_sensor_IO_write_reg(SENSOR_REG_COMMAND1, 0xCC);
            }
            else{
#ifdef SHDISP_SW_BDIC_I2C_RWLOG
                printk("[SHDISP BDIC] shdisp_bdic_PD_SENSOR_control: photo sensor started\n");
#endif
            }
            break;
        case SHDISP_SENSOR_STATE_PROX_ON_ALC_ON:
            if( sensor == SHDISP_BDIC_SENSOR_TYPE_PROX ){
#ifdef SHDISP_SW_BDIC_I2C_RWLOG
                printk("[SHDISP BDIC] shdisp_bdic_PD_SENSOR_control: prox sensor started\n");
#endif
            }
            else{
#ifdef SHDISP_SW_BDIC_I2C_RWLOG
                printk("[SHDISP BDIC] shdisp_bdic_PD_SENSOR_control: photo sensor started\n");
#endif
            }
            break;
            
        default:
            break;
        }
        break;
        
    case SHDISP_BDIC_REQ_STOP:
        switch (state) {
        case SHDISP_SENSOR_STATE_PROX_OFF_ALC_OFF:
            if( sensor == SHDISP_BDIC_SENSOR_TYPE_PROX ){
#ifdef SHDISP_SW_BDIC_I2C_RWLOG
                printk("[SHDISP BDIC] shdisp_bdic_PD_SENSOR_control: prox sensor stoped\n");
#endif
            }
            else{
#ifdef SHDISP_SW_BDIC_I2C_RWLOG
                printk("[SHDISP BDIC] shdisp_bdic_PD_SENSOR_control: photo sensor stoped\n");
#endif
            }
            break;
        case SHDISP_SENSOR_STATE_PROX_ON_ALC_OFF:
            if( sensor == SHDISP_BDIC_SENSOR_TYPE_PHOTO ){
#ifdef SHDISP_SW_BDIC_I2C_RWLOG
                printk("[SHDISP BDIC] shdisp_bdic_PD_SENSOR_control: photo sensor stoped\n");
#endif
            }
            else{
                shdisp_photo_sensor_IO_write_reg(SENSOR_REG_COMMAND1, 0x00);
            }
            break;
        case SHDISP_SENSOR_STATE_PROX_OFF_ALC_ON:
            if( sensor == SHDISP_BDIC_SENSOR_TYPE_PROX ){
#ifdef SHDISP_SW_BDIC_I2C_RWLOG
                printk("[SHDISP BDIC] shdisp_bdic_PD_SENSOR_control: prox sensor stoped\n");
#endif
                break;
            }
            else{
                shdisp_photo_sensor_IO_write_reg(SENSOR_REG_COMMAND1, 0x00);
            }
            break;
        case SHDISP_SENSOR_STATE_PROX_ON_ALC_ON:
            if( sensor == SHDISP_BDIC_SENSOR_TYPE_PHOTO ){
                shdisp_bdic_IO_write_reg(BDIC_REG_ALS_SHIFT, s_state_str.als_shift);
                wBuf[0] = SENSOR_REG_COMMAND1;
                wBuf[1] = 0x0C;
                wBuf[2] = 0x00;
                wBuf[3] = 0x10;
                wBuf[4] = 0xEC;
                shdisp_photo_sensor_IO_burst_write_reg(wBuf, 5);
                shdisp_photo_sensor_IO_write_reg(SENSOR_REG_COMMAND1, 0xEC);
            }
            else{
                shdisp_bdic_IO_write_reg(BDIC_REG_ALS_SHIFT, s_state_str.als_shift);
                wBuf[0] = SENSOR_REG_COMMAND1;
                wBuf[1] = 0x00;
                wBuf[2] = (0x18 | als_range);
                wBuf[3] = 0x00;
                wBuf[4] = 0x00;
                shdisp_photo_sensor_IO_burst_write_reg(wBuf, 5);
                shdisp_photo_sensor_IO_write_reg(SENSOR_REG_COMMAND1, 0xD0);
            }
            break;
        default:
            break;
        }
        break;
    default:
        break;
    }

    return;
}


/* ------------------------------------------------------------------------- */
/* shdisp_bdic_ps_req_mask                                                   */
/* ------------------------------------------------------------------------- */

static int shdisp_bdic_ps_req_mask( unsigned char *remask )
{
    int ret = SHDISP_RESULT_SUCCESS;
    unsigned char val;

    if ((SHDISP_INT_ENABLE_GFAC & SHDISP_BDIC_INT_GFAC_PS2) == 0){
        *remask = 0;
        return ret;
    }
    
    ret = shdisp_bdic_IO_read_reg(BDIC_REG_GIMF2, &val);
    if( val & 0x02 ){
        *remask = 1;
        val &= 0xFD;
        ret = shdisp_bdic_IO_write_reg(BDIC_REG_GIMF2, val);
#ifdef SHDISP_SW_BDIC_IRQ_LOG
        printk("[SHDISP DBG] shdisp_bdic_ps_req_mask\n");
#endif
    }
    else
        *remask = 0;
        
    return ret;
}


/* ------------------------------------------------------------------------- */
/* shdisp_bdic_ps_req_restart                                                */
/* ------------------------------------------------------------------------- */

static int shdisp_bdic_ps_req_restart( unsigned char remask )
{
    int ret = SHDISP_RESULT_SUCCESS;

    if ((SHDISP_INT_ENABLE_GFAC & SHDISP_BDIC_INT_GFAC_PS2) == 0)
        return ret;

    if( !remask )
        return ret;

    ret = shdisp_bdic_IO_set_bit_reg( BDIC_REG_GIMF2, 0x02 );
#ifdef SHDISP_SW_BDIC_IRQ_LOG
    printk("[SHDISP DBG] shdisp_bdic_ps_req_restart\n");
#endif
    return ret;
}


/* ------------------------------------------------------------------------- */
/* Input/Output                                                              */
/* ------------------------------------------------------------------------- */
/* ------------------------------------------------------------------------- */
/* shdisp_bdic_IO_write_reg                                                  */
/* ------------------------------------------------------------------------- */

static int shdisp_bdic_IO_write_reg(unsigned char reg, unsigned char val)
{
    int ret;
    
    if (s_state_str.bdic_is_exist != SHDISP_BDIC_IS_EXIST) {
        SHDISP_ERR("<ACCESS_ERR> shdisp_bdic_IO_write_reg.\n");
        return SHDISP_RESULT_SUCCESS;
    }
    
    ret = shdisp_SYS_bdic_i2c_write(reg, val);
    
    if (ret == SHDISP_RESULT_SUCCESS) {
        return SHDISP_RESULT_SUCCESS;
    }
    else if (ret == SHDISP_RESULT_FAILURE_I2C_TMO) {
        SHDISP_ERR("<TIME_OUT> shdisp_SYS_bdic_i2c_write.\n");
        return SHDISP_RESULT_FAILURE_I2C_TMO;
    }
    
    SHDISP_ERR("<RESULT_FAILURE> shdisp_SYS_bdic_i2c_write.\n");
    return SHDISP_RESULT_FAILURE;
}


/* ------------------------------------------------------------------------- */
/* shdisp_bdic_IO_read_reg                                                  */
/* ------------------------------------------------------------------------- */

static int shdisp_bdic_IO_read_reg(unsigned char reg, unsigned char *val)
{
    int ret;
    
    if (val == NULL) {
        return SHDISP_RESULT_FAILURE;
    }
    
    if (s_state_str.bdic_is_exist != SHDISP_BDIC_IS_EXIST) {
        SHDISP_ERR("<ACCESS_ERR> shdisp_bdic_IO_read_reg.\n");
        return SHDISP_RESULT_SUCCESS;
    }
    
    if ((reg == BDIC_REG_TEST_B3)
    ||  (reg == BDIC_REG_I2C_START)) {
        ret = shdisp_bdic_IO_read_no_check_reg(reg, val);
    }
    else {
        ret = shdisp_bdic_IO_read_check_reg(reg, val);
    }
    
    return ret;
}


/* ------------------------------------------------------------------------- */
/* shdisp_bdic_IO_read_no_check_reg                                          */
/* ------------------------------------------------------------------------- */

static int shdisp_bdic_IO_read_no_check_reg(unsigned char reg, unsigned char *val)
{
    int ret;
    
    ret = shdisp_SYS_bdic_i2c_read(reg, val);
    if (ret == SHDISP_RESULT_SUCCESS) {
        return SHDISP_RESULT_SUCCESS;
    }
    else if (ret == SHDISP_RESULT_FAILURE_I2C_TMO) {
        return SHDISP_RESULT_FAILURE_I2C_TMO;
    }
    
    return SHDISP_RESULT_FAILURE;
}


/* ------------------------------------------------------------------------- */
/* shdisp_bdic_IO_read_check_reg                                             */
/* ------------------------------------------------------------------------- */

static int shdisp_bdic_IO_read_check_reg(unsigned char reg, unsigned char *val)
{
    int ret;
    int retry = 0;
    unsigned char try_1st, try_2nd;
    try_1st = 0;
    try_2nd = 0;
    
    for (retry = 0; retry < 3; retry++) {
        ret = shdisp_SYS_bdic_i2c_read(reg, &try_1st);
        
        if (ret == SHDISP_RESULT_FAILURE_I2C_TMO) {
            SHDISP_ERR("<TIME_OUT> shdisp_SYS_bdic_i2c_read.\n");
            return SHDISP_RESULT_FAILURE_I2C_TMO;
        }
        else if (ret != SHDISP_RESULT_SUCCESS) {
            SHDISP_ERR("<RESULT_FAILURE> shdisp_SYS_bdic_i2c_read.\n");
            return SHDISP_RESULT_FAILURE;
        }
        
        ret = shdisp_SYS_bdic_i2c_read(reg, &try_2nd);
        
        if (ret == SHDISP_RESULT_FAILURE_I2C_TMO) {
            SHDISP_ERR("<TIME_OUT> shdisp_SYS_bdic_i2c_read.\n");
            return SHDISP_RESULT_FAILURE_I2C_TMO;
        }
        else if (ret != SHDISP_RESULT_SUCCESS) {
            SHDISP_ERR("<RESULT_FAILURE> shdisp_SYS_bdic_i2c_read.\n");
            return SHDISP_RESULT_FAILURE;
        }
        
        if (try_1st == try_2nd) {
            *val = try_1st;
            return SHDISP_RESULT_SUCCESS;
        }
        else if (retry == 2) {
            SHDISP_ERR("<OTHER> i2c read retry over! addr:0x%02X val:(1st:0x%02X, 2nd:0x%02X).\n", reg, try_1st, try_2nd);
            *val = try_1st;
            return SHDISP_RESULT_SUCCESS;
        }
        else {
            SHDISP_ERR("<OTHER> i2c read retry (%d)! addr:0x%02X val:(1st:0x%02X, 2nd:0x%02X).\n", retry, reg, try_1st, try_2nd);
        }
    }
    
    SHDISP_ERR("<RESULT_FAILURE> shdisp_SYS_bdic_i2c_read.\n");
    return SHDISP_RESULT_FAILURE;
}


/* ------------------------------------------------------------------------- */
/* shdisp_bdic_IO_multi_read_reg                                             */
/* ------------------------------------------------------------------------- */

static int shdisp_bdic_IO_multi_read_reg(unsigned char reg, unsigned char *val, int size)
{
    int ret;
    
    if (val == NULL) {
        SHDISP_ERR("<NULL_POINTER> val.\n");
        return SHDISP_RESULT_FAILURE;
    }
    
    if ((size < 1) || (size > 8)) {
        SHDISP_ERR("<INVALID_VALUE> size(%d).\n", size);
        return SHDISP_RESULT_FAILURE;
    }
    
    if ((reg + (unsigned char)(size-1)) > BDIC_REG_TEST6) {
        SHDISP_ERR("<OTHER> register address overflow.\n");
        return SHDISP_RESULT_FAILURE;
    }
    
    ret = shdisp_SYS_bdic_i2c_multi_read(reg, val, size);
    
    if (ret == SHDISP_RESULT_SUCCESS) {
        return SHDISP_RESULT_SUCCESS;
    }
    else if (ret == SHDISP_RESULT_FAILURE_I2C_TMO) {
        SHDISP_ERR("<TIME_OUT> shdisp_SYS_bdic_i2c_multi_read.\n");
        return SHDISP_RESULT_FAILURE_I2C_TMO;
    }
    
    SHDISP_ERR("<RESULT_FAILURE> shdisp_SYS_bdic_i2c_multi_read.\n");
    return SHDISP_RESULT_FAILURE;
}


/* ------------------------------------------------------------------------- */
/* shdisp_bdic_IO_set_bit_reg                                                */
/* ------------------------------------------------------------------------- */

static int shdisp_bdic_IO_set_bit_reg(unsigned char reg, unsigned char val)
{
    int             ret;
    unsigned char   set_bit = 0;

    ret = shdisp_bdic_IO_read_reg(reg, &set_bit);

    if (ret != SHDISP_RESULT_SUCCESS) {
        return ret;
    }

    set_bit |= val;

    return shdisp_bdic_IO_write_reg(reg, set_bit);
}


/* ------------------------------------------------------------------------- */
/* shdisp_bdic_IO_clr_bit_reg                                                */
/* ------------------------------------------------------------------------- */

static int shdisp_bdic_IO_clr_bit_reg(unsigned char reg, unsigned char val)
{
    int             ret;
    unsigned char   clr_bit = 0;

    ret = shdisp_bdic_IO_read_reg(reg, &clr_bit);

    if (ret != SHDISP_RESULT_SUCCESS) {
        return ret;
    }

    clr_bit &= (unsigned char)~val;

    return shdisp_bdic_IO_write_reg(reg, clr_bit);
}


/* ------------------------------------------------------------------------- */
/* shdisp_photo_sensor_change_range                                          */
/* ------------------------------------------------------------------------- */

static int shdisp_photo_sensor_change_range( int bkl_opt )
{
    int           ret;
    unsigned char val=0;

    ret = shdisp_photo_sensor_IO_read_reg(SENSOR_REG_COMMAND2, &val);
    val &= 0xF8;

    if( bkl_opt == SHDISP_BDIC_MAIN_BKL_OPT_LOW )
        val |= 0x04;
    else
        val |= 0x06;
    ret = shdisp_photo_sensor_IO_write_reg(SENSOR_REG_COMMAND2, val);
    return ret;
}


/* ------------------------------------------------------------------------- */
/* shdisp_bdic_IO_msk_bit_reg                                                */
/* ------------------------------------------------------------------------- */

static int shdisp_bdic_IO_msk_bit_reg(unsigned char reg, unsigned char val, unsigned char msk)
{
    int             ret;
    unsigned char   src_bit = 0;
    unsigned char   dst_bit = 0;

    ret = shdisp_bdic_IO_read_reg(reg, &src_bit);

    if (ret != SHDISP_RESULT_SUCCESS) {
        return ret;
    }

    dst_bit = ((unsigned char)~msk & src_bit) | (msk & val);

    return shdisp_bdic_IO_write_reg(reg, dst_bit);
}


/* ------------------------------------------------------------------------- */
/* shdisp_photo_sensor_IO_write_reg                                          */
/* ------------------------------------------------------------------------- */

static int shdisp_photo_sensor_IO_write_reg(unsigned char reg, unsigned char val)
{
    int ret = SHDISP_RESULT_SUCCESS;
    struct shdisp_bdic_i2c_msg msg;
    unsigned char wbuf[2];
    unsigned char rbuf[1];
    
    wbuf[0] = reg;
    wbuf[1] = val;
    rbuf[0] = 0x00;
    
    msg.addr = 0x39;
    msg.mode = SHDISP_BDIC_I2C_M_W;
    msg.wlen = 2;
    msg.rlen = 0;
    msg.wbuf = &wbuf[0];
    msg.rbuf = NULL;
    
    ret = shdisp_bdic_API_i2c_transfer(&msg);
    
    
    return ret;
}

#if 1
/* ------------------------------------------------------------------------- */
/* shdisp_photo_sensor_IO_read_reg                                           */
/* ------------------------------------------------------------------------- */

static int shdisp_photo_sensor_IO_read_reg(unsigned char reg, unsigned char *val)
{
    int ret = SHDISP_RESULT_SUCCESS;
    struct shdisp_bdic_i2c_msg msg;
    unsigned char wbuf[1];
    unsigned char rbuf[1];
    
    wbuf[0] = reg;
    rbuf[0] = 0x00;
    
    msg.addr = 0x39;
    msg.mode = SHDISP_BDIC_I2C_M_R;
    msg.wlen = 1;
    msg.rlen = 1;
    msg.wbuf = &wbuf[0];
    msg.rbuf = &rbuf[0];
    
    ret = shdisp_bdic_API_i2c_transfer(&msg);
    if (ret == SHDISP_RESULT_SUCCESS) {
        *val = rbuf[0];
    }
    
    return ret;
}
#endif


/* ------------------------------------------------------------------------- */
/* shdisp_photo_sensor_IO_burst_write_reg                                    */
/* ------------------------------------------------------------------------- */

static int shdisp_photo_sensor_IO_burst_write_reg(unsigned char *wval, unsigned char dataNum)
{
    int ret = SHDISP_RESULT_SUCCESS;
    struct shdisp_bdic_i2c_msg msg;
    
    msg.addr = 0x39;
    msg.mode = SHDISP_BDIC_I2C_M_W;
    msg.wlen = dataNum;
    msg.rlen = 0;
    msg.wbuf = wval;
    msg.rbuf = NULL;
    
    ret = shdisp_bdic_API_i2c_transfer(&msg);
    
    return ret;
}


/* ------------------------------------------------------------------------- */
/* shdisp_photo_sensor_IO_write_threshold                                    */
/* ------------------------------------------------------------------------- */

static int shdisp_photo_sensor_IO_write_threshold(void)
{
    int ret = SHDISP_RESULT_SUCCESS;
    unsigned char wBuf[SHDISP_BDIC_I2C_WBUF_MAX];
    
    wBuf[0] = SENSOR_REG_PS_LT_LSB;
    wBuf[1] = (unsigned char)(s_state_str.shdisp_bdic_prox_params.threshold_low & 0x00FF);
    wBuf[2] = (unsigned char)((s_state_str.shdisp_bdic_prox_params.threshold_low >> 8) & 0x00FF);
    wBuf[3] = (unsigned char)(s_state_str.shdisp_bdic_prox_params.threshold_high & 0x00FF);
    wBuf[4] = (unsigned char)((s_state_str.shdisp_bdic_prox_params.threshold_high >> 8) & 0x00FF);
    
    ret = shdisp_photo_sensor_IO_burst_write_reg(wBuf, 5);
    
    return ret;
}


MODULE_DESCRIPTION("SHARP DISPLAY DRIVER MODULE");
MODULE_LICENSE("GPL v2");
MODULE_AUTHOR("SHARP CORPORATION");
MODULE_VERSION("1.00");

/* ------------------------------------------------------------------------- */
/* END OF FILE                                                               */
/* ------------------------------------------------------------------------- */
