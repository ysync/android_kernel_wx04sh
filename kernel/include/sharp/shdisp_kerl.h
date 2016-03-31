/* include/sharp/shdisp_kerl.h  (Display Driver)
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

#ifndef SHDISP_KERN_H
#define SHDISP_KERN_H

/* ------------------------------------------------------------------------- */
/* INCLUDE FILES                                                             */
/* ------------------------------------------------------------------------- */



/* ------------------------------------------------------------------------- */
/* IOCTL                                                                     */
/* ------------------------------------------------------------------------- */

#define SHDISP_IOC_MAGIC 's'
#define SHDISP_IOCTL_GET_CONTEXT            _IOR  (SHDISP_IOC_MAGIC,  0, struct shdisp_kernel_context)
#define SHDISP_IOCTL_SET_HOST_GPIO          _IOW  (SHDISP_IOC_MAGIC,  1, struct shdisp_host_gpio)
#define SHDISP_IOCTL_TRI_LED_SET_COLOR      _IOW  (SHDISP_IOC_MAGIC,  2, struct shdisp_tri_led)
#define SHDISP_IOCTL_BDIC_WRITE_REG         _IOW  (SHDISP_IOC_MAGIC,  3, struct shdisp_diag_bdic_reg)
#define SHDISP_IOCTL_BDIC_READ_REG          _IOWR (SHDISP_IOC_MAGIC,  4, struct shdisp_diag_bdic_reg)
#define SHDISP_IOCTL_GET_LUX                _IOWR (SHDISP_IOC_MAGIC,  5, struct shdisp_photo_sensor_val)
#define SHDISP_IOCTL_PWM_CONFIG             _IOW  (SHDISP_IOC_MAGIC,  6, int)
#define SHDISP_IOCTL_PWM_STOP               _IO   (SHDISP_IOC_MAGIC,  7)
#define SHDISP_IOCTL_PHOTO_SENSOR_POW_CTL   _IOW  (SHDISP_IOC_MAGIC,  8, struct shdisp_photo_sensor_power_ctl)
#define SHDISP_IOCTL_LCDDR_WRITE_REG        _IOW  (SHDISP_IOC_MAGIC,  9, struct shdisp_lcddr_reg)
#define SHDISP_IOCTL_LCDDR_READ_REG         _IOWR (SHDISP_IOC_MAGIC, 10, struct shdisp_lcddr_reg)
#define SHDISP_IOCTL_SET_FLICKER_PARAM      _IOW  (SHDISP_IOC_MAGIC, 11, unsigned short)
#define SHDISP_IOCTL_GET_FLICKER_PARAM      _IOWR (SHDISP_IOC_MAGIC, 12, unsigned short)
#define SHDISP_IOCTL_BKL_SET_AUTO_MODE      _IOW  (SHDISP_IOC_MAGIC, 13, int)
#define SHDISP_IOCTL_BDIC_MULTI_READ_REG    _IOWR (SHDISP_IOC_MAGIC, 14, struct shdisp_diag_bdic_reg_multi)
#define SHDISP_IOCTL_BKL_SET_DTV_MODE       _IOW  (SHDISP_IOC_MAGIC, 15, int)
#define SHDISP_IOCTL_BKL_SET_EMG_MODE       _IOW  (SHDISP_IOC_MAGIC, 16, int)
#define SHDISP_IOCTL_LEDC_POWER_ON          _IO   (SHDISP_IOC_MAGIC, 17)
#define SHDISP_IOCTL_LEDC_POWER_OFF         _IO   (SHDISP_IOC_MAGIC, 18)
#define SHDISP_IOCTL_LEDC_SET_RGB           _IOW  (SHDISP_IOC_MAGIC, 19, struct shdisp_ledc_rgb)
#define SHDISP_IOCTL_LEDC_SET_COLOR         _IOW  (SHDISP_IOC_MAGIC, 20, struct shdisp_ledc_req)
#define SHDISP_IOCTL_LEDC_WRITE_REG         _IOW  (SHDISP_IOC_MAGIC, 21, struct shdisp_diag_ledc_reg)
#define SHDISP_IOCTL_LEDC_READ_REG          _IOWR (SHDISP_IOC_MAGIC, 22, struct shdisp_diag_ledc_reg)
#define SHDISP_IOCTL_LUX_CHANGE_IND         _IOWR (SHDISP_IOC_MAGIC, 23, struct shdisp_photo_sensor_val)
#define SHDISP_IOCTL_BKL_SET_ECO_MODE       _IOW  (SHDISP_IOC_MAGIC, 24, int)
#define SHDISP_IOCTL_SET_CABC               _IOWR (SHDISP_IOC_MAGIC, 25, struct shdisp_main_dbc)
#define SHDISP_IOCTL_BKL_SET_CHG_MODE       _IOW  (SHDISP_IOC_MAGIC, 26, int)
#define SHDISP_IOCTL_GET_FLICKER_LOW_PARAM  _IOWR (SHDISP_IOC_MAGIC, 27, unsigned short)
#define SHDISP_IOCTL_SET_CABC_UPDATE_PARAM  _IOWR (SHDISP_IOC_MAGIC, 28, struct shdisp_main_dbc)

/* ------------------------------------------------------------------------- */
/* MACROS                                                                    */
/* ------------------------------------------------------------------------- */

#define DEV_NAME    ("shdisp")
#define SHDISP_DEV    DEV_NAME

#define SHDISP_NOT_ADJUST_VCOM_VAL  0xFFFF

/* ------------------------------------------------------------------------- */
/* TYPES                                                                     */
/* ------------------------------------------------------------------------- */

enum {
    SHDISP_BDIC_I2C_M_W,
    SHDISP_BDIC_I2C_M_R,
    SHDISP_BDIC_I2C_M_R_MODE1,
    SHDISP_BDIC_I2C_M_R_MODE2,
    SHDISP_BDIC_I2C_M_R_MODE3,
    NUM_SHDISP_BDIC_I2C_M
};

enum {
    SHDISP_PROX_SENSOR_POWER_OFF,
    SHDISP_PROX_SENSOR_POWER_ON,
    NUM_SHDISP_PROX_SENSOR_POWER
};

enum {
    SHDISP_IRQ_TYPE_ALS,
    SHDISP_IRQ_TYPE_PS,
    SHDISP_IRQ_TYPE_DET,
    NUM_SHDISP_IRQ_TYPE
};

enum {
    SHDISP_LEDC_PWR_STATUS_OFF,
    SHDISP_LEDC_PWR_STATUS_ON,
    NUM_SHDISP_LEDC_PWR_STATUS
};

enum {
    SHDISP_RESULT_SUCCESS,
    SHDISP_RESULT_FAILURE,
    SHDISP_RESULT_FAILURE_I2C_TMO,
    NUM_SHDISP_RESULT
};

enum {
    SHDISP_UPPER_UNIT_IS_NOT_CONNECTED,
    SHDISP_UPPER_UNIT_IS_CONNECTED,
    NUM_UPPER_UNIT_STATUS
};

enum {
    SHDISP_BDIC_IS_NOT_EXIST,
    SHDISP_BDIC_IS_EXIST,
    NUM_BDIC_EXIST_STATUS
};

enum {
    SHDISP_MAIN_BKL_MODE_OFF,
    SHDISP_MAIN_BKL_MODE_FIX,
    SHDISP_MAIN_BKL_MODE_AUTO,
    SHDISP_MAIN_BKL_MODE_AUTO_ECO,
    SHDISP_MAIN_BKL_MODE_DTV_OFF,
    SHDISP_MAIN_BKL_MODE_DTV_FIX,
    SHDISP_MAIN_BKL_MODE_DTV_AUTO,
    NUM_SHDISP_MAIN_BKL_MODE
};

enum {
    SHDISP_MAIN_BKL_PARAM_0,
    SHDISP_MAIN_BKL_PARAM_1,
    SHDISP_MAIN_BKL_PARAM_2,
    SHDISP_MAIN_BKL_PARAM_3,
    SHDISP_MAIN_BKL_PARAM_4,
    SHDISP_MAIN_BKL_PARAM_5,
    SHDISP_MAIN_BKL_PARAM_6,
    SHDISP_MAIN_BKL_PARAM_7,
    SHDISP_MAIN_BKL_PARAM_8,
    SHDISP_MAIN_BKL_PARAM_9,
    SHDISP_MAIN_BKL_PARAM_10,
    SHDISP_MAIN_BKL_PARAM_11,
    SHDISP_MAIN_BKL_PARAM_12,
    SHDISP_MAIN_BKL_PARAM_13,
    SHDISP_MAIN_BKL_PARAM_14,
    SHDISP_MAIN_BKL_PARAM_15,
    SHDISP_MAIN_BKL_PARAM_16,
    SHDISP_MAIN_BKL_PARAM_17,
    SHDISP_MAIN_BKL_PARAM_18,
    SHDISP_MAIN_BKL_PARAM_19,
    SHDISP_MAIN_BKL_PARAM_20,
    SHDISP_MAIN_BKL_PARAM_21,
    SHDISP_MAIN_BKL_PARAM_22,
    NUM_SHDISP_MAIN_BKL_PARAM
};

enum {
    SHDISP_MAIN_BKL_AUTO_OFF,
    SHDISP_MAIN_BKL_AUTO_ON,
    SHDISP_MAIN_BKL_AUTO_ECO_ON,
    NUM_SHDISP_MAIN_BKL_AUTO
};

enum {
    SHDISP_MAIN_BKL_DTV_OFF,
    SHDISP_MAIN_BKL_DTV_ON,
    NUM_SHDISP_MAIN_BKL_DTV
};

enum {
    SHDISP_MAIN_BKL_EMG_OFF,
    SHDISP_MAIN_BKL_EMG_ON,
    NUM_SHDISP_MAIN_BKL_EMG
};

enum {
    SHDISP_MAIN_BKL_ECO_OFF,
    SHDISP_MAIN_BKL_ECO_ON,
    NUM_SHDISP_MAIN_BKL_ECO
};

enum {
    SHDISP_MAIN_BKL_CHG_OFF,
    SHDISP_MAIN_BKL_CHG_ON,
    NUM_SHDISP_MAIN_BKL_CHG
};

enum {
    SHDISP_PHOTO_SENSOR_DISABLE,
    SHDISP_PHOTO_SENSOR_ENABLE,
    NUM_SHDISP_PHOTO_SENSOR
};

enum {
    SHDISP_TRI_LED_EXT_MODE_DISABLE,
    SHDISP_TRI_LED_EXT_MODE_ENABLE,
    NUM_SHDISP_TRI_LED_EXT_MODE
};

enum {
    SHDISP_TRI_LED_MODE_NORMAL,
    SHDISP_TRI_LED_MODE_BLINK,
    SHDISP_TRI_LED_MODE_FIREFLY,
    NUM_SHDISP_TRI_LED_MODE
};

enum {
    SHDISP_TRI_LED_ONTIME_TYPE0,
    SHDISP_TRI_LED_ONTIME_TYPE1,
    SHDISP_TRI_LED_ONTIME_TYPE2,
    SHDISP_TRI_LED_ONTIME_TYPE3,
    SHDISP_TRI_LED_ONTIME_TYPE4,
    SHDISP_TRI_LED_ONTIME_TYPE5,
    SHDISP_TRI_LED_ONTIME_TYPE6,
    SHDISP_TRI_LED_ONTIME_TYPE7,
    NUM_SHDISP_TRI_LED_ONTIME
};

enum {
    SHDISP_TRI_LED_INTERVAL_NONE,
    SHDISP_TRI_LED_INTERVAL_1S,
    SHDISP_TRI_LED_INTERVAL_2S,
    SHDISP_TRI_LED_INTERVAL_3S,
    SHDISP_TRI_LED_INTERVAL_4S,
    SHDISP_TRI_LED_INTERVAL_5S,
    SHDISP_TRI_LED_INTERVAL_6S,
    SHDISP_TRI_LED_INTERVAL_7S,
    SHDISP_TRI_LED_INTERVAL_8S,
    SHDISP_TRI_LED_INTERVAL_9S,
    SHDISP_TRI_LED_INTERVAL_10S,
    SHDISP_TRI_LED_INTERVAL_11S,
    SHDISP_TRI_LED_INTERVAL_12S,
    SHDISP_TRI_LED_INTERVAL_13S,
    SHDISP_TRI_LED_INTERVAL_14S,
    SHDISP_TRI_LED_INTERVAL_15S,
    NUM_SHDISP_TRI_LED_INTERVAL
};

enum {
    SHDISP_PHOTO_SENSOR_TYPE_APP,
    SHDISP_PHOTO_SENSOR_TYPE_LUX,
    NUM_SHDISP_PHOTO_SENSOR_TYPE
};

enum {
    SHDISP_LUX_MODE_LOW,
    SHDISP_LUX_MODE_HIGH,
    NUM_SHDISP_LUX_MODE
};

enum {
    SHDISP_LEDC_RGB_MODE_NORMAL,
    SHDISP_LEDC_RGB_MODE_PATTERN1,
    SHDISP_LEDC_RGB_MODE_PATTERN2,
    SHDISP_LEDC_RGB_MODE_PATTERN3,
    SHDISP_LEDC_RGB_MODE_PATTERN4,
    SHDISP_LEDC_RGB_MODE_PATTERN5,
    SHDISP_LEDC_RGB_MODE_PATTERN6,
    SHDISP_LEDC_RGB_MODE_ANIMATION1,
    SHDISP_LEDC_RGB_MODE_ANIMATION2,
    SHDISP_LEDC_RGB_MODE_ANIMATION3,
    SHDISP_LEDC_RGB_MODE_ANIMATION4,
    SHDISP_LEDC_RGB_MODE_ANIMATION5,
    SHDISP_LEDC_RGB_MODE_ANIMATION6,
    SHDISP_LEDC_RGB_MODE_ANIMATION7,
    SHDISP_LEDC_RGB_MODE_ANIMATION8,
    SHDISP_LEDC_RGB_MODE_ANIMATION9,
    SHDISP_LEDC_RGB_MODE_ANIMATION10,
    NUM_SHDISP_LEDC_RGB_MODE
};

enum {
    SHDISP_LEDC_ONCOUNT_REPEAT,
    SHDISP_LEDC_ONCOUNT_1SHOT,
    NUM_SHDISP_LEDC_ONCOUNT
};

enum {
    SHDISP_LEDC_IS_NOT_EXIST,
    SHDISP_LEDC_IS_EXIST,
    NUM_LEDC_EXIST_STATUS
};


enum {
    SHDISP_MAIN_DISP_CABC_MODE_OFF,
    SHDISP_MAIN_DISP_CABC_MODE_DBC,
    SHDISP_MAIN_DISP_CABC_MODE_ACC,
    SHDISP_MAIN_DISP_CABC_MODE_DBC_ACC,
    NUM_SHDISP_CABC_MODE
};

enum {
    SHDISP_MAIN_DISP_CABC_LUT0,
    SHDISP_MAIN_DISP_CABC_LUT1,
    SHDISP_MAIN_DISP_CABC_LUT2,
    SHDISP_MAIN_DISP_CABC_LUT3,
    SHDISP_MAIN_DISP_CABC_LUT4,
    SHDISP_MAIN_DISP_CABC_LUT5,
    NUM_SHDISP_CABC_LUT
};


struct shdisp_host_gpio {
    int num;
    int value;
};

struct shdisp_procfs {
    int id;
    int par[4];
};

struct shdisp_main_bkl_ctl {
    int mode;
    int param;
};

struct shdisp_tri_led {
    unsigned long red;
    unsigned long green;
    unsigned long blue;
    int ext_mode;
    int led_mode;
    int ontime;
    int interval;
};

struct shdisp_diag_bdic_reg {
    unsigned char reg;
    unsigned char val;
};

struct shdisp_diag_bdic_reg_multi {
    unsigned char reg;
    unsigned char val[8];
    unsigned char size;
};

#define SHDISP_LCDDR_BUF_MAX    64
struct shdisp_lcddr_reg {
    unsigned char address;
    unsigned char size;
    unsigned char buf[SHDISP_LCDDR_BUF_MAX];
};

#define SHDISP_LCDDR_PHY_GAMMA_BUF_MAX      72
struct shdisp_lcddr_phy_gamma_reg {
    unsigned char status;
    unsigned char buf[SHDISP_LCDDR_PHY_GAMMA_BUF_MAX];
    short chksum;
};

struct shdisp_photo_sensor_val {
    unsigned short value;
    unsigned long  lux;
    int mode;
    int result;
};

struct shdisp_photo_sensor_power_ctl {
    int type;
    int power;
};

struct shdisp_pharaoh_reg {
    unsigned char address;
    unsigned char size;
    unsigned char buf[32];
};

struct shdisp_photo_sensor_adj {
    unsigned char status;
    unsigned short als_adj0;
    unsigned short als_adj1;
    unsigned char als_shift;
    unsigned char clear_offset;
    unsigned char ir_offset;
    unsigned char adc_lclip;
    unsigned char key_backlight;
    unsigned long chksum;
};

struct dma_abl_color {
    unsigned char blue;
    unsigned char green;
    unsigned char red;
};

struct dma_abl_lut_gamma {
    struct dma_abl_color abl_color[256];
    unsigned long chksum;
};

struct dma_argc_lut_gamma{
    unsigned short red[16][4];
    unsigned long red_chksum;
    unsigned short green[16][4];
    unsigned long green_chksum;
    unsigned short blue[16][4];
    unsigned long blue_chksum;
};

struct shdisp_ledc_rgb {
    unsigned long mode;
    unsigned long red[2];
    unsigned long green[2];
    unsigned long blue[2];
};

struct shdisp_ledc_req {
    unsigned long red;
    unsigned long green;
    unsigned long blue;
    int led_mode;
    int on_count;
};

struct shdisp_diag_ledc_reg {
    unsigned char reg;
    unsigned char val;
};

struct shdisp_ledc_mono {
    unsigned long led;
    int led_mode;
    int on_count;
};

struct shdisp_bdic_i2c_msg {
    unsigned short addr;
    unsigned char mode;
    unsigned char wlen;
    unsigned char rlen;
    const unsigned char *wbuf;
    unsigned char *rbuf;
};

struct shdisp_panel_param_str{
    unsigned short vcom_alpha;
    unsigned short vcom_alpha_low;
    unsigned int   shdisp_lcd;
};

struct shdisp_main_update {
    void *buf;
    unsigned short src_width;
    unsigned short src_xps;
    unsigned short src_yps;
    unsigned short upd_xsz;
    unsigned short upd_ysz;
    unsigned short dst_xps;
    unsigned short dst_yps;
};

struct shdisp_main_clear {
    unsigned long color;
    unsigned short start_xps;
    unsigned short start_yps;
    unsigned short end_xps;
    unsigned short end_yps;
};

struct shdisp_subscribe {
    int   irq_type;
    void (*callback)(void);
};

struct shdisp_prox_params {
    unsigned int threshold_low;
    unsigned int threshold_high;
};

struct shdisp_ledc_status {
    int ledc_is_exist;
    int power_status;
    struct shdisp_ledc_req ledc_req;
};


struct shdisp_panel_operations {
    int (*init_io) (void);
    int (*exit_io) (void);
    int (*init_isr) (void);
    void (*set_param) (struct shdisp_panel_param_str *param_str);
    int (*power_on) (void);
    int (*power_off) (void);
    int (*init_1st) (void);
    int (*init_2nd) (void);
    int (*disp_on) (void);
    int (*sleep) (void);
    int (*deep_standby) (void);
    int (*check_upper_unit) (void);
    int (*check_flicker) (unsigned short alpha_in, unsigned short *alpha_out);
    int (*write_reg) (unsigned char addr, unsigned char *write_data, unsigned char size);
    int (*read_reg) (unsigned char addr, unsigned char *read_data, unsigned char size);
    int (*set_flicker) (unsigned short alpha);
    int (*get_flicker) (unsigned short *alpha);
    int (*check_recovery) (void);
    int (*recovery_type) (int *type);
    int (*set_abl_lut) (struct dma_abl_lut_gamma *abl_lut_gammma);
    int (*disp_update) (struct shdisp_main_update *update);
    int (*clear_screen) (struct shdisp_main_clear *clear);
    int (*get_flicker_low) (unsigned short *alpha);
};


struct shdisp_boot_context {
    int driver_is_initialized;
    unsigned short hw_revision;
    int handset_color;
    int upper_unit_is_connected;
    int bdic_is_exist;
    int main_disp_status;
    struct shdisp_main_bkl_ctl main_bkl;
    struct shdisp_tri_led tri_led;
    unsigned short alpha;
    unsigned short alpha_low;
    struct shdisp_photo_sensor_adj photo_sensor_adj;
    unsigned short dma_lut_status;
    struct dma_argc_lut_gamma argc_lut_gamma;
    struct dma_abl_lut_gamma abl_lut_gamma;
    struct shdisp_lcddr_phy_gamma_reg lcddr_phy_gamma;
    struct shdisp_lcddr_phy_gamma_reg lcddr_rom_gamma;
    struct shdisp_ledc_status ledc_status;
    unsigned int shdisp_lcd;
};

struct shdisp_kernel_context {
    int driver_is_initialized;
    unsigned short hw_revision;
    int handset_color;
    int upper_unit_is_connected;
    int bdic_is_exist;
    int main_disp_status;
    struct shdisp_main_bkl_ctl main_bkl;
    struct shdisp_tri_led tri_led;
    unsigned short alpha;
    unsigned short alpha_low;
    struct shdisp_photo_sensor_adj photo_sensor_adj;
    unsigned short dma_lut_status;
    struct dma_argc_lut_gamma argc_lut_gamma;
    struct dma_abl_lut_gamma abl_lut_gamma;
    struct shdisp_lcddr_phy_gamma_reg lcddr_phy_gamma;
    struct shdisp_lcddr_phy_gamma_reg lcddr_rom_gamma;
    int dtv_status;
    int thermal_status;
    int eco_bkl_status;
    int usb_chg_status;
    struct shdisp_ledc_status ledc_status;
    unsigned int shdisp_lcd;
    unsigned char dbgTraceF;
};


struct shdisp_main_dbc {
    int mode;
    int auto_mode;
};

struct shdisp_check_cabc_val {
    int old_mode;
    int old_lut;
    int mode;
    int lut;
    int change;
};


/* ------------------------------------------------------------------------- */
/* PROTOTYPES                                                                */
/* ------------------------------------------------------------------------- */

int shdisp_api_get_fbcon_status(void);
int shdisp_api_main_lcd_power_on(void);
int shdisp_api_main_lcd_reset(void);
int shdisp_api_main_lcd_power_off(void);
int shdisp_api_main_disp_init_1st(void);
int shdisp_api_main_disp_init_2nd(void);
int shdisp_api_main_disp_on(void);
int shdisp_api_main_disp_off(void);
int shdisp_api_main_bkl_on(struct shdisp_main_bkl_ctl *bkl);
int shdisp_api_main_bkl_off(void);
int shdisp_api_shutdown(void);
int shdisp_api_write_bdic_i2c(struct shdisp_bdic_i2c_msg *i2c_msg);
int shdisp_api_read_bdic_i2c(struct shdisp_bdic_i2c_msg *i2c_msg);
int shdisp_api_prox_sensor_pow_ctl(int power_mode, struct shdisp_prox_params *prox_params);
void shdisp_api_get_boot_context(void);
int shdisp_api_get_panel_info(void);
int shdisp_api_get_clock_info(void);
int shdisp_api_get_clock_sw(void);
int shdisp_api_get_boot_disp_status(void);
int shdisp_api_get_upper_unit(void);
int shdisp_api_set_upper_unit(int mode);
unsigned short shdisp_api_get_hw_revision(void);
unsigned short shdisp_api_get_alpha(void);
unsigned short shdisp_api_get_alpha_low(void);
struct shdisp_lcddr_phy_gamma_reg* shdisp_api_get_lcddr_phy_gamma(void);
int shdisp_api_check_recovery (void);
int shdisp_api_do_recovery (void);
int shdisp_api_event_subscribe(struct shdisp_subscribe *subscribe);
int shdisp_api_event_unsubscribe(int irq_type);
void shdisp_api_do_mipi_dsi_det_recovery(void);
void shdisp_api_clr_mipi_dsi_det_recovery_flag(void);
int  shdisp_api_get_mipi_dsi_det_recovery_flag(void);
void shdisp_api_init_cabc_state(int mode, int lut);

int shdisp_api_check_cabc(struct shdisp_check_cabc_val *value);
int shdisp_api_pwm_enable(void);
int shdisp_api_pwm_disable(void);

int shdisp_api_photo_sensor_pow_ctl(void);
int shdisp_api_get_shutdown_mode(void);
int shdisp_api_get_recovery_mode(void);

#endif /* SHDISP_KERN_H */

/* ------------------------------------------------------------------------- */
/* END OF FILE                                                               */
/* ------------------------------------------------------------------------- */
