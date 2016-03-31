/* drivers/sharp/shdisp/shdisp_bd2802gu.c  (Display Driver)
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
#include "shdisp_bd2802gu.h"
#include "shdisp_system.h"
#include "./data/shdisp_bd2802gu_data_default.h"


/* ------------------------------------------------------------------------- */
/* DEBUG MACRAOS                                                             */
/* ------------------------------------------------------------------------- */
//#define SHDISP_LCDC_SW_REG_BKUP
//#define SHDISP_LCDC_SW_I2C_WLOG

#define SHDISP_BD2802GU_FILE "shdisp_bd2802gu.c"
#define SHDISP_ERR(fmt, args...) \
        printk("[SHDISP_ERROR][%s][%s] " fmt, SHDISP_BD2802GU_FILE, __func__, ## args);

/* ------------------------------------------------------------------------- */
/* MACROS                                                                    */
/* ------------------------------------------------------------------------- */
#define SHDISP_LEDC_REG_MAX             0x16

#define SHDISP_LEDC_RET_ERR_NULL        -1
#define SHDISP_LEDC_RET_ERR_NOIC        -2

/* ------------------------------------------------------------------------- */
/* PROTOTYPES                                                                */
/* ------------------------------------------------------------------------- */
static int shdisp_ledc_LD_api_check(void);
static int shdisp_ledc_LD_power_on(void);
static int shdisp_ledc_LD_power_off(void);
static void shdisp_ledc_LD_set_color(struct shdisp_ledc_req *ledc_req);
static void shdisp_ledc_PD_init(void);
static int shdisp_ledc_PD_power_on(void);
static int shdisp_ledc_PD_power_off(void);
static void shdisp_ledc_PD_LED_off(void);
static void shdisp_ledc_PD_set_rgb(struct shdisp_ledc_rgb *ledc_rgb);
static void shdisp_ledc_PD_LED_on_color(unsigned char color1, unsigned char color2, int mode, int count);
static int shdisp_ledc_IO_write_reg(unsigned char reg, unsigned char val);
static int shdisp_ledc_IO_read_reg(unsigned char reg, unsigned char *val);

/* ------------------------------------------------------------------------- */
/* VARIABLES                                                                 */
/* ------------------------------------------------------------------------- */
static struct shdisp_ledc_status *plcdc_info = NULL;
#ifdef SHDISP_LCDC_SW_REG_BKUP
static unsigned char shdisp_ledc_reg[SHDISP_LEDC_REG_MAX];
#endif

/* ------------------------------------------------------------------------- */
/* FUNCTIONS                                                                 */
/* ------------------------------------------------------------------------- */
/* ------------------------------------------------------------------------- */
/* API                                                                       */
/* ------------------------------------------------------------------------- */
/* ------------------------------------------------------------------------- */
/* shdisp_ledc_API_set_status_addr                                           */
/* ------------------------------------------------------------------------- */

void shdisp_ledc_API_set_status_addr(struct shdisp_ledc_status* state_str)
{
    plcdc_info = state_str;
    return;
}


/* ------------------------------------------------------------------------- */
/* shdisp_ledc_API_init                                                      */
/* ------------------------------------------------------------------------- */

int shdisp_ledc_API_init(void)
{
    int ret;
    
    shdisp_ledc_PD_init();
    
    ret = shdisp_SYS_ledc_i2c_init();
    if(ret != SHDISP_RESULT_SUCCESS) {
        return ret;
    }
    
    return SHDISP_RESULT_SUCCESS;
}


/* ------------------------------------------------------------------------- */
/* shdisp_ledc_API_exit                                                      */
/* ------------------------------------------------------------------------- */

int shdisp_ledc_API_exit(void)
{
    int ret;
    
    ret = shdisp_SYS_ledc_i2c_exit();
    if(ret != SHDISP_RESULT_SUCCESS) {
        return ret;
    }
    
    return SHDISP_RESULT_SUCCESS;
}


/* ------------------------------------------------------------------------- */
/* shdisp_ledc_API_power_on                                                  */
/* ------------------------------------------------------------------------- */

int shdisp_ledc_API_power_on(void)
{
    int ret;
    
    ret = shdisp_ledc_LD_api_check();
    if(ret != SHDISP_RESULT_SUCCESS) {
        SHDISP_ERR("<OTHER> status error(%d).\n", ret);
    }
    
    if(plcdc_info->power_status != SHDISP_LEDC_PWR_STATUS_OFF) {
        SHDISP_ERR("<OTHER> power status error(%d).\n", plcdc_info->power_status);
        return SHDISP_RESULT_FAILURE;
    }
    
    return shdisp_ledc_LD_power_on();
}


/* ------------------------------------------------------------------------- */
/* shdisp_ledc_API_power_off                                                 */
/* ------------------------------------------------------------------------- */

int shdisp_ledc_API_power_off(void)
{
    int ret;
    
    ret = shdisp_ledc_LD_api_check();
    if(ret != SHDISP_RESULT_SUCCESS) {
        SHDISP_ERR("<OTHER> status error(%d).\n", ret);
    }
    
    if(plcdc_info->power_status != SHDISP_LEDC_PWR_STATUS_ON) {
        SHDISP_ERR("<OTHER> power status error(%d).\n", plcdc_info->power_status);
        return SHDISP_RESULT_FAILURE;
    }
    
    return shdisp_ledc_LD_power_off();
}


/* ------------------------------------------------------------------------- */
/* shdisp_ledc_API_set_rgb                                                   */
/* ------------------------------------------------------------------------- */

void shdisp_ledc_API_set_rgb(struct shdisp_ledc_rgb *ledc_rgb)
{
    int ret;
    
    ret = shdisp_ledc_LD_api_check();
    if(ret != SHDISP_RESULT_SUCCESS) {
        SHDISP_ERR("<OTHER> status error(%d).\n", ret);
    }
    
    shdisp_ledc_PD_set_rgb(ledc_rgb);
    
    return;
}


/* ------------------------------------------------------------------------- */
/* shdisp_ledc_API_set_color                                                 */
/* ------------------------------------------------------------------------- */

void shdisp_ledc_API_set_color(struct shdisp_ledc_req *ledc_req)
{
    int ret;
    
    ret = shdisp_ledc_LD_api_check();
    if(ret != SHDISP_RESULT_SUCCESS) {
        SHDISP_ERR("<OTHER> status error(%d).\n", ret);
    }
    
    if((plcdc_info->ledc_req.red == ledc_req->red)
    && (plcdc_info->ledc_req.green == ledc_req->green)
    && (plcdc_info->ledc_req.blue == ledc_req->blue)
    && (plcdc_info->ledc_req.led_mode == ledc_req->led_mode)
    && (plcdc_info->ledc_req.on_count == ledc_req->on_count)) {
        return;
    }
    
    shdisp_ledc_LD_set_color(ledc_req);
    
    return;
}


/* ------------------------------------------------------------------------- */
/* shdisp_ledc_API_DIAG_write_reg                                            */
/* ------------------------------------------------------------------------- */

void shdisp_ledc_API_DIAG_write_reg(unsigned char reg, unsigned char val)
{
    int ret;
    
    ret = shdisp_ledc_LD_api_check();
    if(ret != SHDISP_RESULT_SUCCESS) {
        SHDISP_ERR("<OTHER> status error(%d).\n", ret);
    }
    
    if(plcdc_info->power_status != SHDISP_LEDC_PWR_STATUS_ON) {
        SHDISP_ERR("<OTHER> power status error(%d).\n", plcdc_info->power_status);
        return;
    }
    
    shdisp_ledc_IO_write_reg(reg, val);
    
    return;
}


/* ------------------------------------------------------------------------- */
/* shdisp_ledc_API_DIAG_read_reg                                             */
/* ------------------------------------------------------------------------- */

void shdisp_ledc_API_DIAG_read_reg(unsigned char reg, unsigned char *val)
{
    int ret;
    
    ret = shdisp_ledc_LD_api_check();
    if(ret != SHDISP_RESULT_SUCCESS) {
        SHDISP_ERR("<OTHER> status error(%d).\n", ret);
    }
    
    if(plcdc_info->power_status != SHDISP_LEDC_PWR_STATUS_ON) {
        SHDISP_ERR("<OTHER> power status error(%d).\n", plcdc_info->power_status);
        return;
    }
    
    shdisp_ledc_IO_read_reg(reg, val);
    
    return;
}


/* ------------------------------------------------------------------------- */
/* Logical Driver                                                            */
/* ------------------------------------------------------------------------- */
/* ------------------------------------------------------------------------- */
/* shdisp_ledc_LD_api_check                                                  */
/* ------------------------------------------------------------------------- */

static int shdisp_ledc_LD_api_check(void)
{
    if(plcdc_info == NULL) {
        return SHDISP_LEDC_RET_ERR_NULL;
    }
    
#if 0
    if(plcdc_info->ledc_is_exist != SHDISP_LEDC_IS_EXIST) {
        return SHDISP_LEDC_RET_ERR_NOIC;
    }
#endif
    
    return SHDISP_RESULT_SUCCESS;
}


/* ------------------------------------------------------------------------- */
/* shdisp_ledc_LD_power_on                                                   */
/* ------------------------------------------------------------------------- */

static int shdisp_ledc_LD_power_on(void)
{
    return shdisp_ledc_PD_power_on();
}


/* ------------------------------------------------------------------------- */
/* shdisp_ledc_LD_power_off                                                  */
/* ------------------------------------------------------------------------- */

static int shdisp_ledc_LD_power_off(void)
{
    return shdisp_ledc_PD_power_off();
}


/* ------------------------------------------------------------------------- */
/* shdisp_ledc_LD_set_color                                                  */
/* ------------------------------------------------------------------------- */

static void shdisp_ledc_LD_set_color(struct shdisp_ledc_req *ledc_req)
{
    unsigned char color1, color2;
    
    color1 = ((((ledc_req->blue  & 0x01) << 2)
             | ((ledc_req->green & 0x01) << 1)
             | ((ledc_req->red   & 0x01)     )) & 0x07);
    
    color2 = ((((ledc_req->blue  & 0x02) << 1)
             | ((ledc_req->green & 0x02)     )
             | ((ledc_req->red   & 0x02) >> 1)) & 0x07);
    
    if((color1 == 0x00) && (color2 == 0x00)) {
        shdisp_ledc_PD_LED_off();
        
        shdisp_ledc_LD_power_off();
    } else {
        shdisp_ledc_LD_power_on();
        
        shdisp_ledc_PD_LED_on_color(color1, color2, ledc_req->led_mode, ledc_req->on_count);
    }
    
    plcdc_info->ledc_req.red   = ledc_req->red;
    plcdc_info->ledc_req.green = ledc_req->green;
    plcdc_info->ledc_req.blue  = ledc_req->blue;
    plcdc_info->ledc_req.led_mode = ledc_req->led_mode;
    plcdc_info->ledc_req.on_count = ledc_req->on_count;
    
    return;
}


/* ------------------------------------------------------------------------- */
/* Phygical Driver                                                           */
/* ------------------------------------------------------------------------- */
/* ------------------------------------------------------------------------- */
/* shdisp_ledc_PD_init                                                       */
/* ------------------------------------------------------------------------- */

static void shdisp_ledc_PD_init(void)
{
#ifdef SHDISP_LCDC_SW_REG_BKUP
    memset(shdisp_ledc_reg, 0, sizeof(shdisp_ledc_reg));
    shdisp_ledc_reg[LEDC_REG_R1_PTRN] = 0x07;
    shdisp_ledc_reg[LEDC_REG_G1_PTRN] = 0x07;
    shdisp_ledc_reg[LEDC_REG_B1_PTRN] = 0x07;
    shdisp_ledc_reg[LEDC_REG_R2_PTRN] = 0x07;
    shdisp_ledc_reg[LEDC_REG_G2_PTRN] = 0x07;
    shdisp_ledc_reg[LEDC_REG_B2_PTRN] = 0x07;
#endif
    return;
}


/* ------------------------------------------------------------------------- */
/* shdisp_ledc_PD_power_on                                                   */
/* ------------------------------------------------------------------------- */

static int shdisp_ledc_PD_power_on(void)
{
    if (plcdc_info->power_status == SHDISP_LEDC_PWR_STATUS_ON)
        return SHDISP_RESULT_SUCCESS;
    
    shdisp_SYS_set_Host_gpio(SHDISP_GPIO_NUM_LEDC_RST_N, SHDISP_GPIO_CTL_LOW);
    shdisp_SYS_delay_us(100);
    shdisp_SYS_set_Host_gpio(SHDISP_GPIO_NUM_LEDC_RST_N, SHDISP_GPIO_CTL_HIGH);
    shdisp_SYS_delay_us(100);
    
    plcdc_info->power_status = SHDISP_LEDC_PWR_STATUS_ON;
    
    shdisp_ledc_PD_init();
    
    return SHDISP_RESULT_SUCCESS;
}


/* ------------------------------------------------------------------------- */
/* shdisp_ledc_PD_power_off                                                  */
/* ------------------------------------------------------------------------- */

static int shdisp_ledc_PD_power_off(void)
{
    if (plcdc_info->power_status == SHDISP_LEDC_PWR_STATUS_OFF)
        return SHDISP_RESULT_SUCCESS;
    
    shdisp_SYS_set_Host_gpio(SHDISP_GPIO_NUM_LEDC_RST_N, SHDISP_GPIO_CTL_LOW);
    shdisp_SYS_delay_us(100);
    
    plcdc_info->power_status = SHDISP_LEDC_PWR_STATUS_OFF;
    
    shdisp_ledc_PD_init();
    
    return SHDISP_RESULT_SUCCESS;
}


/* ------------------------------------------------------------------------- */
/* shdisp_ledc_PD_LED_off                                                    */
/* ------------------------------------------------------------------------- */

static void shdisp_ledc_PD_LED_off(void)
{
    if (plcdc_info->power_status == SHDISP_LEDC_PWR_STATUS_OFF)
        return;
    
    shdisp_ledc_IO_write_reg(LEDC_REG_RGB_CTRL, 0x00);
    
    return;
}


/* ------------------------------------------------------------------------- */
/* shdisp_ledc_PD_set_rgb                                                    */
/* ------------------------------------------------------------------------- */

static void shdisp_ledc_PD_set_rgb(struct shdisp_ledc_rgb *ledc_rgb)
{
    shdisp_ledc_IO_write_reg(LEDC_REG_RGB_CTRL,  0x00);
    shdisp_ledc_IO_write_reg(LEDC_REG_RGB1_TIME, (unsigned char)((ledc_rgb->mode >>  8) & 0xFF));
    shdisp_ledc_IO_write_reg(LEDC_REG_RGB2_TIME, (unsigned char)((ledc_rgb->mode      ) & 0xFF));
    shdisp_ledc_IO_write_reg(LEDC_REG_R1_VAL1,   (unsigned char)((ledc_rgb->red[0]   >> 16) & 0xFF));
    shdisp_ledc_IO_write_reg(LEDC_REG_R1_VAL2,   (unsigned char)((ledc_rgb->red[0]   >>  8) & 0xFF));
    shdisp_ledc_IO_write_reg(LEDC_REG_R1_PTRN,   (unsigned char)((ledc_rgb->red[0]        ) & 0xFF));
    shdisp_ledc_IO_write_reg(LEDC_REG_G1_VAL1,   (unsigned char)((ledc_rgb->green[0] >> 16) & 0xFF));
    shdisp_ledc_IO_write_reg(LEDC_REG_G1_VAL2,   (unsigned char)((ledc_rgb->green[0] >>  8) & 0xFF));
    shdisp_ledc_IO_write_reg(LEDC_REG_G1_PTRN,   (unsigned char)((ledc_rgb->green[0]      ) & 0xFF));
    shdisp_ledc_IO_write_reg(LEDC_REG_B1_VAL1,   (unsigned char)((ledc_rgb->blue[0]  >> 16) & 0xFF));
    shdisp_ledc_IO_write_reg(LEDC_REG_B1_VAL2,   (unsigned char)((ledc_rgb->blue[0]  >>  8) & 0xFF));
    shdisp_ledc_IO_write_reg(LEDC_REG_B1_PTRN,   (unsigned char)((ledc_rgb->blue[0]       ) & 0xFF));
    shdisp_ledc_IO_write_reg(LEDC_REG_R2_VAL1,   (unsigned char)((ledc_rgb->red[1]   >> 16) & 0xFF));
    shdisp_ledc_IO_write_reg(LEDC_REG_R2_VAL2,   (unsigned char)((ledc_rgb->red[1]   >>  8) & 0xFF));
    shdisp_ledc_IO_write_reg(LEDC_REG_R2_PTRN,   (unsigned char)((ledc_rgb->red[1]        ) & 0xFF));
    shdisp_ledc_IO_write_reg(LEDC_REG_G2_VAL1,   (unsigned char)((ledc_rgb->green[1] >> 16) & 0xFF));
    shdisp_ledc_IO_write_reg(LEDC_REG_G2_VAL2,   (unsigned char)((ledc_rgb->green[1] >>  8) & 0xFF));
    shdisp_ledc_IO_write_reg(LEDC_REG_G2_PTRN,   (unsigned char)((ledc_rgb->green[1]      ) & 0xFF));
    shdisp_ledc_IO_write_reg(LEDC_REG_B2_VAL1,   (unsigned char)((ledc_rgb->blue[1]  >> 16) & 0xFF));
    shdisp_ledc_IO_write_reg(LEDC_REG_B2_VAL2,   (unsigned char)((ledc_rgb->blue[1]  >>  8) & 0xFF));
    shdisp_ledc_IO_write_reg(LEDC_REG_B2_PTRN,   (unsigned char)((ledc_rgb->blue[1]       ) & 0xFF));
    shdisp_ledc_IO_write_reg(LEDC_REG_RGB_CTRL,  (unsigned char)((ledc_rgb->mode >> 16) & 0xFF));
    
    plcdc_info->ledc_req.red   = 0x01;
    plcdc_info->ledc_req.green = 0x01;
    plcdc_info->ledc_req.blue  = 0x01;
    plcdc_info->ledc_req.led_mode = NUM_SHDISP_LEDC_RGB_MODE;
    plcdc_info->ledc_req.on_count = NUM_SHDISP_LEDC_ONCOUNT;
    
    return;
}


/* ------------------------------------------------------------------------- */
/* shdisp_ledc_PD_LED_on_color                                               */
/* ------------------------------------------------------------------------- */

static void shdisp_ledc_PD_LED_on_color(unsigned char color1, unsigned char color2, int mode, int count)
{
    unsigned char rgb_ctrl = 0x00;
    unsigned char val1 = 0x00;
    unsigned char val2 = 0x00;
    int i, ptn_no;
    
    switch (mode) {
    case SHDISP_LEDC_RGB_MODE_NORMAL:
    case SHDISP_LEDC_RGB_MODE_PATTERN1:
    case SHDISP_LEDC_RGB_MODE_PATTERN2:
    case SHDISP_LEDC_RGB_MODE_PATTERN3:
    case SHDISP_LEDC_RGB_MODE_PATTERN4:
    case SHDISP_LEDC_RGB_MODE_PATTERN5:
    case SHDISP_LEDC_RGB_MODE_PATTERN6:
        shdisp_ledc_IO_write_reg(LEDC_REG_RGB_CTRL, 0x00);
        shdisp_ledc_IO_write_reg(LEDC_REG_RGB1_TIME, shdisp_ledc_ptn_tbl[mode][0]);
        for (i=0; i<3; i++) {
            val1 = 0x00;
            val2 = 0x00;
            if(shdisp_ledc_rgb1_tbl[color1][i] != 0) {
                val1 = shdisp_ledc_ptn_tbl[mode][1];
                val2 = shdisp_ledc_ptn_tbl[mode][2];
            }
            shdisp_ledc_IO_write_reg(LEDC_REG_R1_VAL1 + (i * 3) + 0, val1);
            shdisp_ledc_IO_write_reg(LEDC_REG_R1_VAL1 + (i * 3) + 1, val2);
            shdisp_ledc_IO_write_reg(LEDC_REG_R1_VAL1 + (i * 3) + 2, shdisp_ledc_ptn_tbl[mode][3]);
        }
        shdisp_ledc_IO_write_reg(LEDC_REG_RGB2_TIME, shdisp_ledc_ptn_tbl[mode][4]);
        for (i=0; i<3; i++) {
            val1 = 0x00;
            val2 = 0x00;
            if(shdisp_ledc_rgb1_tbl[color2][i] != 0) {
                val1 = shdisp_ledc_ptn_tbl[mode][5];
                val2 = shdisp_ledc_ptn_tbl[mode][6];
            }
            shdisp_ledc_IO_write_reg(LEDC_REG_R2_VAL1 + (i * 3) + 0, val1);
            shdisp_ledc_IO_write_reg(LEDC_REG_R2_VAL1 + (i * 3) + 1, val2);
            shdisp_ledc_IO_write_reg(LEDC_REG_R2_VAL1 + (i * 3) + 2, shdisp_ledc_ptn_tbl[mode][7]);
        }
        if(count != SHDISP_LEDC_ONCOUNT_1SHOT) {
            rgb_ctrl  = (color1 == 0) ? 0 : 0x01;
            rgb_ctrl |= (color2 == 0) ? 0 : 0x10;
        } else {
            rgb_ctrl  = (color1 == 0) ? 0 : 0x02;
            rgb_ctrl |= (color2 == 0) ? 0 : 0x20;
        }
        shdisp_ledc_IO_write_reg(LEDC_REG_RGB_CTRL, rgb_ctrl);
        break;
    case SHDISP_LEDC_RGB_MODE_ANIMATION1:
    case SHDISP_LEDC_RGB_MODE_ANIMATION2:
    case SHDISP_LEDC_RGB_MODE_ANIMATION3:
    case SHDISP_LEDC_RGB_MODE_ANIMATION4:
    case SHDISP_LEDC_RGB_MODE_ANIMATION5:
    case SHDISP_LEDC_RGB_MODE_ANIMATION6:
    case SHDISP_LEDC_RGB_MODE_ANIMATION7:
    case SHDISP_LEDC_RGB_MODE_ANIMATION8:
    case SHDISP_LEDC_RGB_MODE_ANIMATION9:
    case SHDISP_LEDC_RGB_MODE_ANIMATION10:
        ptn_no = mode - SHDISP_LEDC_RGB_MODE_ANIMATION1;
        shdisp_ledc_IO_write_reg(LEDC_REG_RGB_CTRL, 0x00);
        shdisp_ledc_IO_write_reg(LEDC_REG_RGB1_TIME, shdisp_ledc_anime1_tbl[ptn_no][0]);
        shdisp_ledc_IO_write_reg(LEDC_REG_R1_VAL1,   shdisp_ledc_anime1_tbl[ptn_no][1]);
        shdisp_ledc_IO_write_reg(LEDC_REG_R1_VAL2,   shdisp_ledc_anime1_tbl[ptn_no][2]);
        shdisp_ledc_IO_write_reg(LEDC_REG_R1_PTRN,   shdisp_ledc_anime1_tbl[ptn_no][3]);
        shdisp_ledc_IO_write_reg(LEDC_REG_G1_VAL1,   shdisp_ledc_anime1_tbl[ptn_no][4]);
        shdisp_ledc_IO_write_reg(LEDC_REG_G1_VAL2,   shdisp_ledc_anime1_tbl[ptn_no][5]);
        shdisp_ledc_IO_write_reg(LEDC_REG_G1_PTRN,   shdisp_ledc_anime1_tbl[ptn_no][6]);
        shdisp_ledc_IO_write_reg(LEDC_REG_B1_VAL1,   shdisp_ledc_anime1_tbl[ptn_no][7]);
        shdisp_ledc_IO_write_reg(LEDC_REG_B1_VAL2,   shdisp_ledc_anime1_tbl[ptn_no][8]);
        shdisp_ledc_IO_write_reg(LEDC_REG_B1_PTRN,   shdisp_ledc_anime1_tbl[ptn_no][9]);
        shdisp_ledc_IO_write_reg(LEDC_REG_RGB2_TIME, shdisp_ledc_anime2_tbl[ptn_no][0]);
        shdisp_ledc_IO_write_reg(LEDC_REG_R2_VAL1,   shdisp_ledc_anime2_tbl[ptn_no][1]);
        shdisp_ledc_IO_write_reg(LEDC_REG_R2_VAL2,   shdisp_ledc_anime2_tbl[ptn_no][2]);
        shdisp_ledc_IO_write_reg(LEDC_REG_R2_PTRN,   shdisp_ledc_anime2_tbl[ptn_no][3]);
        shdisp_ledc_IO_write_reg(LEDC_REG_G2_VAL1,   shdisp_ledc_anime2_tbl[ptn_no][4]);
        shdisp_ledc_IO_write_reg(LEDC_REG_G2_VAL2,   shdisp_ledc_anime2_tbl[ptn_no][5]);
        shdisp_ledc_IO_write_reg(LEDC_REG_G2_PTRN,   shdisp_ledc_anime2_tbl[ptn_no][6]);
        shdisp_ledc_IO_write_reg(LEDC_REG_B2_VAL1,   shdisp_ledc_anime2_tbl[ptn_no][7]);
        shdisp_ledc_IO_write_reg(LEDC_REG_B2_VAL2,   shdisp_ledc_anime2_tbl[ptn_no][8]);
        shdisp_ledc_IO_write_reg(LEDC_REG_B2_PTRN,   shdisp_ledc_anime2_tbl[ptn_no][9]);
        if(count != SHDISP_LEDC_ONCOUNT_1SHOT) {
            rgb_ctrl  = (color1 == 0) ? 0 : 0x01;
            rgb_ctrl |= (color2 == 0) ? 0 : 0x10;
        } else {
            rgb_ctrl  = (color1 == 0) ? 0 : 0x02;
            rgb_ctrl |= (color2 == 0) ? 0 : 0x20;
        }
        shdisp_ledc_IO_write_reg(LEDC_REG_RGB_CTRL, rgb_ctrl);
        break;
    default:
        break;
    }
    
    return;
}


/* ------------------------------------------------------------------------- */
/* Input/Output                                                              */
/* ------------------------------------------------------------------------- */
/* ------------------------------------------------------------------------- */
/* shdisp_ledc_IO_write_reg                                                  */
/* ------------------------------------------------------------------------- */

static int shdisp_ledc_IO_write_reg(unsigned char reg, unsigned char val)
{
    int ret;
    
#ifdef SHDISP_LCDC_SW_I2C_WLOG
    printk("[SHDISP] ledc i2c Write(reg=0x%02x, val=0x%02X)", reg, val);
#endif
    
    ret = shdisp_SYS_ledc_i2c_write(reg, val);
    
    if (ret == SHDISP_RESULT_SUCCESS) {
#ifdef SHDISP_LCDC_SW_REG_BKUP
        if(reg < SHDISP_LEDC_REG_MAX) {
            shdisp_ledc_reg[reg] = val;
        }
#endif
        return SHDISP_RESULT_SUCCESS;
    } else if (ret == SHDISP_RESULT_FAILURE_I2C_TMO) {
        SHDISP_ERR("<TIME_OUT> shdisp_SYS_ledc_i2c_write.\n");
        return SHDISP_RESULT_FAILURE_I2C_TMO;
    }
    
    return SHDISP_RESULT_FAILURE;
}


/* ------------------------------------------------------------------------- */
/* shdisp_ledc_IO_read_reg                                                  */
/* ------------------------------------------------------------------------- */

static int shdisp_ledc_IO_read_reg(unsigned char reg, unsigned char *val)
{
#ifdef SHDISP_LCDC_SW_REG_BKUP
    if(reg < SHDISP_LEDC_REG_MAX) {
        *val = shdisp_ledc_reg[reg];
    }
#endif
    return SHDISP_RESULT_SUCCESS;
}


MODULE_DESCRIPTION("SHARP DISPLAY DRIVER MODULE");
MODULE_LICENSE("GPL v2");
MODULE_AUTHOR("SHARP CORPORATION");
MODULE_VERSION("1.00");

/* ------------------------------------------------------------------------- */
/* END OF FILE                                                               */
/* ------------------------------------------------------------------------- */
