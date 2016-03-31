/* drivers/sharp/shdisp/shdisp_flute.c  (Display Driver)
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
#include "shdisp_flute.h"
#include "shdisp_system.h"
#include "shdisp_bl69y6.h"
#include "../../video/msm/msm_fb_panel.h"
#include "../../video/msm/mipi_sharp.h"
#include "../../video/msm/mipi_sharp_flute.h"
#include "shdisp_type.h"
#include "../../video/msm/mipi_dsi.h"


/* ------------------------------------------------------------------------- */
/* MACROS                                                                    */
/* ------------------------------------------------------------------------- */

#define SHDISP_FLUTE_1V_WAIT                 18000
#define SHDISP_FLUTE_VCOM_MIN                0x0000
#define SHDISP_FLUTE_VCOM_MAX                0x0096
#define SHDISP_FLUTE_ALPHA_DEFAULT           0x0054

#define SHDISP_FLUTE_TPIN_CFG                0xFA0182F0
#define SHDISP_FLUTE_TPIN_OUT                0xFA0182F4
#define SHDISP_FLUTE_TPIN_VAL                0xFA01A064

/* ------------------------------------------------------------------------- */
/* PROTOTYPES                                                                */
/* ------------------------------------------------------------------------- */

static int shdisp_flute_API_init_io(void);
static int shdisp_flute_API_exit_io(void);
static int shdisp_flute_API_init_isr(void);
static void shdisp_flute_API_set_param(struct shdisp_panel_param_str *param_str);
static int shdisp_flute_API_power_on(void);
static int shdisp_flute_API_disp_init_1st(void);
static int shdisp_flute_API_disp_init_2nd(void);
static int shdisp_flute_API_disp_on(void);
static int shdisp_flute_API_sleep(void);
static int shdisp_flute_API_deep_standby(void);
static int shdisp_flute_API_power_off(void);
static int shdisp_flute_API_check_upper_unit(void);
static int shdisp_flute_API_check_flicker_param(unsigned short alpha_in, unsigned short *alpha_out);
static int shdisp_flute_API_diag_write_reg(unsigned char addr, unsigned char *write_data, unsigned char size);
static int shdisp_flute_API_diag_read_reg(unsigned char addr, unsigned char *read_data, unsigned char size);
static int shdisp_flute_API_diag_set_flicker_param(unsigned short alpha);
static int shdisp_flute_API_diag_get_flicker_param(unsigned short *alpha);
static int shdisp_flute_API_diag_get_flicker_low_param(unsigned short *alpha);
static int shdisp_flute_API_check_recovery(void);
static int shdisp_flute_API_recovery_type(int *type);
static int shdisp_flute_API_set_abl_lut(struct dma_abl_lut_gamma *abl_lut_gammma);
static int shdisp_flute_API_disp_update(struct shdisp_main_update *update);
static int shdisp_flute_API_disp_clear_screen(struct shdisp_main_clear *clear);


/* ------------------------------------------------------------------------- */
/* VARIABLES                                                                 */
/* ------------------------------------------------------------------------- */

static struct shdisp_panel_param_str flute_param_str;
static unsigned char flute_wdata[8];
static unsigned char flute_rdata[8];


static struct shdisp_panel_operations shdisp_flute_fops = {
    shdisp_flute_API_init_io,
    shdisp_flute_API_exit_io,
    shdisp_flute_API_init_isr,
    shdisp_flute_API_set_param,
    shdisp_flute_API_power_on,
    shdisp_flute_API_power_off,
    shdisp_flute_API_disp_init_1st,
    shdisp_flute_API_disp_init_2nd,
    shdisp_flute_API_disp_on,
    shdisp_flute_API_sleep,
    shdisp_flute_API_deep_standby,
    shdisp_flute_API_check_upper_unit,
    shdisp_flute_API_check_flicker_param,
    shdisp_flute_API_diag_write_reg,
    shdisp_flute_API_diag_read_reg,
    shdisp_flute_API_diag_set_flicker_param,
    shdisp_flute_API_diag_get_flicker_param,
    shdisp_flute_API_check_recovery,
    shdisp_flute_API_recovery_type,
    shdisp_flute_API_set_abl_lut,
    shdisp_flute_API_disp_update,
    shdisp_flute_API_disp_clear_screen,
    shdisp_flute_API_diag_get_flicker_low_param,
};

/* ------------------------------------------------------------------------- */
/* DEBUG MACRAOS                                                             */
/* ------------------------------------------------------------------------- */
#define SHDISP_FLUTE_SW_CHK_UPPER_UNIT

#define SHDISP_FLUTE_FILE "shdisp_flute.c"
#define SHDISP_ERR(fmt, args...) \
        printk("[SHDISP_ERROR][%s][%s] " fmt, SHDISP_FLUTE_FILE, __func__, ## args);

/* ------------------------------------------------------------------------- */
/* FUNCTIONS                                                                 */
/* ------------------------------------------------------------------------- */
/* ------------------------------------------------------------------------- */
/* API                                                                       */
/* ------------------------------------------------------------------------- */
/* ------------------------------------------------------------------------- */
/* shdisp_flute_API_create                                                    */
/* ------------------------------------------------------------------------- */
struct shdisp_panel_operations *shdisp_flute_API_create(void)
{
    return &shdisp_flute_fops;
}


/* ------------------------------------------------------------------------- */
/* shdisp_flute_API_init_io                                                   */
/* ------------------------------------------------------------------------- */
static int shdisp_flute_API_init_io(void)
{
    return SHDISP_RESULT_SUCCESS;
}


/* ------------------------------------------------------------------------- */
/* shdisp_flute_API_exit_io                                                   */
/* ------------------------------------------------------------------------- */
static int shdisp_flute_API_exit_io(void)
{
    return SHDISP_RESULT_SUCCESS;
}


/* ------------------------------------------------------------------------- */
/* shdisp_flute_API_init_isr                                                  */
/* ------------------------------------------------------------------------- */
static int shdisp_flute_API_init_isr(void)
{
    return SHDISP_RESULT_SUCCESS;
}


/* ------------------------------------------------------------------------- */
/* shdisp_flute_API_set_param                                                 */
/* ------------------------------------------------------------------------- */
static void shdisp_flute_API_set_param(struct shdisp_panel_param_str *param_str)
{
    flute_param_str.vcom_alpha = param_str->vcom_alpha;
    
    return;
}


/* ------------------------------------------------------------------------- */
/* shdisp_flute_API_power_on                                                  */
/* ------------------------------------------------------------------------- */
static int shdisp_flute_API_power_on(void)
{
    shdisp_bdic_API_LCD_power_on();
    shdisp_SYS_delay_us(10000);
    
    return SHDISP_RESULT_SUCCESS;
}


/* ------------------------------------------------------------------------- */
/* shdisp_flute_API_disp_init_1st                                             */
/* ------------------------------------------------------------------------- */
static int shdisp_flute_API_disp_init_1st(void)
{
    return SHDISP_RESULT_SUCCESS;
}


/* ------------------------------------------------------------------------- */
/* shdisp_flute_API_disp_init_2nd                                             */
/* ------------------------------------------------------------------------- */
static int shdisp_flute_API_disp_init_2nd(void)
{
    return SHDISP_RESULT_SUCCESS;
}


/* ------------------------------------------------------------------------- */
/* shdisp_flute_API_disp_on                                                   */
/* ------------------------------------------------------------------------- */
static int shdisp_flute_API_disp_on(void)
{
    return SHDISP_RESULT_SUCCESS;
}


/* ------------------------------------------------------------------------- */
/* shdisp_flute_API_sleep                                                     */
/* ------------------------------------------------------------------------- */
static int shdisp_flute_API_sleep(void)
{
    return SHDISP_RESULT_SUCCESS;
}


/* ------------------------------------------------------------------------- */
/* shdisp_flute_API_deep_standby                                              */
/* ------------------------------------------------------------------------- */
static int shdisp_flute_API_deep_standby(void)
{
    return SHDISP_RESULT_SUCCESS;
}


/* ------------------------------------------------------------------------- */
/* shdisp_flute_API_power_off                                                 */
/* ------------------------------------------------------------------------- */
static int shdisp_flute_API_power_off(void)
{
    if ((shdisp_api_get_shutdown_mode()) || (shdisp_api_get_recovery_mode())) {
        shdisp_bdic_API_LCD_set_hw_reset();
        shdisp_SYS_delay_us(1000);
    }
    
    shdisp_bdic_API_LCD_power_off();
    shdisp_SYS_delay_us(10000);
    
    mipi_dsi_clkin_force_ctl(0);
    
    return SHDISP_RESULT_SUCCESS;
}


/* ------------------------------------------------------------------------- */
/* shdisp_flute_API_check_upper_unit                                          */
/* ------------------------------------------------------------------------- */
static int shdisp_flute_API_check_upper_unit(void)
{
#ifndef SHDISP_FLUTE_SW_CHK_UPPER_UNIT
    unsigned cfg_bk, out_bk;
    unsigned val;
    
    cfg_bk = __raw_readl(SHDISP_FLUTE_TPIN_CFG);
    out_bk = __raw_readl(SHDISP_FLUTE_TPIN_OUT);
    
    __raw_writel(SHDISP_FLUTE_TPIN_CFG, 0x00000003);
    __raw_writel(SHDISP_FLUTE_TPIN_OUT, 0x00000000);
    
    shdisp_SYS_delay_us(5);
    
    val = __raw_readl(SHDISP_FLUTE_TPIN_VAL);
    val = (val & 0x00008000) ? 1 : 0;
    
    __raw_writel(SHDISP_FLUTE_TPIN_OUT, out_bk);
    __raw_writel(SHDISP_FLUTE_TPIN_CFG, cfg_bk);
    
    if(!val) {
        SHDISP_ERR("<OTHER> Upper unit does not exist.\n");
        return SHDISP_RESULT_FAILURE;
    }
#endif
    return SHDISP_RESULT_SUCCESS;
}


/* ------------------------------------------------------------------------- */
/* shdisp_flute_API_check_flicker_param                                       */
/* ------------------------------------------------------------------------- */
static int shdisp_flute_API_check_flicker_param(unsigned short alpha_in, unsigned short *alpha_out)
{
    unsigned short tmp_alpha = alpha_in;
    
    if (alpha_out == NULL){
        SHDISP_ERR("<NULL_POINTER> alpha_out.\n");
        return SHDISP_RESULT_FAILURE;
    }
    
    if ((tmp_alpha & 0xF000) != 0x9000) {
        *alpha_out = SHDISP_FLUTE_ALPHA_DEFAULT;
        return SHDISP_RESULT_SUCCESS;
    }
    
    tmp_alpha = tmp_alpha & 0x0FFF;
    if (tmp_alpha > SHDISP_FLUTE_VCOM_MAX) {
        *alpha_out = SHDISP_FLUTE_ALPHA_DEFAULT;
        return SHDISP_RESULT_SUCCESS;
    }
    
    *alpha_out = tmp_alpha;
    
    return SHDISP_RESULT_SUCCESS;
}


/* ------------------------------------------------------------------------- */
/* shdisp_flute_API_diag_write_reg                                            */
/* ------------------------------------------------------------------------- */
static int shdisp_flute_API_diag_write_reg(unsigned char addr, unsigned char *write_data, unsigned char size)
{
    int ret = 0;
    
    ret = mipi_sharp_flute_diag_write_reg(addr, write_data, size);
    if(ret) {
        return SHDISP_RESULT_FAILURE;
    }
    return SHDISP_RESULT_SUCCESS;
}


/* ------------------------------------------------------------------------- */
/* shdisp_flute_API_diag_read_reg                                             */
/* ------------------------------------------------------------------------- */
static int shdisp_flute_API_diag_read_reg(unsigned char addr, unsigned char *read_data, unsigned char size)
{
    int ret = 0;
    
    ret = mipi_sharp_flute_diag_read_reg(addr, read_data, size);
    if(ret) {
        return SHDISP_RESULT_FAILURE;
    }
    return SHDISP_RESULT_SUCCESS;
}


/* ------------------------------------------------------------------------- */
/* shdisp_flute_API_diag_set_flicker_param                                    */
/* ------------------------------------------------------------------------- */
static int shdisp_flute_API_diag_set_flicker_param(unsigned short alpha)
{
    int i;
    int ret = 0;
    
    if (alpha > SHDISP_FLUTE_VCOM_MAX) {
        SHDISP_ERR("<INVALID_VALUE> alpha(0x%04X).\n", alpha);
        return SHDISP_RESULT_FAILURE;
    }
    
    for (i=1; i<=7; i++) {
        flute_rdata[i] = 0;
    }
    
    ret = mipi_sharp_flute_diag_read_reg(0xF5, flute_rdata, 6);
    if(ret) {
        SHDISP_ERR("<RESULT_FAILURE> mipi_sharp_diag_read_reg.\n");
    }
    flute_wdata[0] = flute_rdata[0];
    flute_wdata[1] = flute_rdata[1];
    flute_wdata[2] = (unsigned char) (alpha & 0xFF);
    flute_wdata[3] = flute_rdata[3];
    flute_wdata[4] = flute_rdata[4];
    flute_wdata[5] = flute_rdata[5];
    
    ret = mipi_sharp_flute_diag_write_reg(0xF5, flute_wdata, 6);
    if(ret) {
        SHDISP_ERR("<RESULT_FAILURE> mipi_sharp_diag_write_reg.\n");
    }
    flute_param_str.vcom_alpha = alpha;
    return SHDISP_RESULT_SUCCESS;
}


/* ------------------------------------------------------------------------- */
/* shdisp_flute_API_diag_get_flicker_param                                    */
/* ------------------------------------------------------------------------- */
static int shdisp_flute_API_diag_get_flicker_param(unsigned short *alpha)
{
    int i;
    int ret = 0;
    
    if (alpha == NULL){
        SHDISP_ERR("<NULL_POINTER> alpha.\n");
        return SHDISP_RESULT_FAILURE;
    }
    
    for (i=1; i<=7; i++) {
        flute_rdata[i] = 0;
    }
    
    ret = mipi_sharp_flute_diag_read_reg(0xF5, flute_rdata, 6);
    if(ret) {
        SHDISP_ERR("<RESULT_FAILURE> mipi_sharp_diag_read_reg.\n");
        return SHDISP_RESULT_FAILURE;
    }
    
    *alpha = flute_rdata[2] & 0xFF;
    
    return SHDISP_RESULT_SUCCESS;
}


/* ------------------------------------------------------------------------- */
/* shdisp_flute_API_diag_get_flicker_low_param                               */
/* ------------------------------------------------------------------------- */
static int shdisp_flute_API_diag_get_flicker_low_param(unsigned short *alpha)
{
    return SHDISP_RESULT_SUCCESS;
}


/* ------------------------------------------------------------------------- */
/* shdisp_flute_API_check_recovery                                            */
/* ------------------------------------------------------------------------- */
static int shdisp_flute_API_check_recovery(void)
{
    int ret;
    
    ret = shdisp_bdic_API_RECOVERY_check_restoration();
    
    if (ret != SHDISP_RESULT_SUCCESS) {
        SHDISP_ERR("<RESULT_FAILURE> shdisp_bdic_API_RECOVERY_check_restoration.\n");
        return SHDISP_RESULT_FAILURE;
    }
    
    return SHDISP_RESULT_SUCCESS;
}


/* ------------------------------------------------------------------------- */
/* shdisp_flute_API_recovery_type                                             */
/* ------------------------------------------------------------------------- */
static int shdisp_flute_API_recovery_type(int *type)
{
    *type = SHDISP_SUBSCRIBE_TYPE_INT;

    return SHDISP_RESULT_SUCCESS;
}


/* ------------------------------------------------------------------------- */
/* shdisp_flute_API_set_abl_lut                                               */
/* ------------------------------------------------------------------------- */
static int shdisp_flute_API_set_abl_lut(struct dma_abl_lut_gamma *abl_lut_gammma)
{
    return SHDISP_RESULT_SUCCESS;
}


/* ------------------------------------------------------------------------- */
/* shdisp_flute_API_disp_update                                               */
/* ------------------------------------------------------------------------- */
static int shdisp_flute_API_disp_update(struct shdisp_main_update *update)
{
    return SHDISP_RESULT_SUCCESS;
}


/* ------------------------------------------------------------------------- */
/* shdisp_flute_API_disp_clear_screen                                         */
/* ------------------------------------------------------------------------- */
static int shdisp_flute_API_disp_clear_screen(struct shdisp_main_clear *clear)
{
    return SHDISP_RESULT_SUCCESS;
}


MODULE_DESCRIPTION("SHARP DISPLAY DRIVER MODULE");
MODULE_LICENSE("GPL v2");
MODULE_AUTHOR("SHARP CORPORATION");
MODULE_VERSION("1.00");

/* ------------------------------------------------------------------------- */
/* END OF FILE                                                               */
/* ------------------------------------------------------------------------- */
