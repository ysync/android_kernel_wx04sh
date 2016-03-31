/* drivers/sharp/shdisp/shdisp_takt.c  (Display Driver)
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
#include "shdisp_takt.h"
#include "shdisp_system.h"
#include "shdisp_type.h"

//#ifdef SHDISP_BL68Y6
//#include "shdisp_bdic.h"
//#else /* SHDISP_BL68Y6 */
#include "shdisp_bl69y6.h"
//#endif /* SHDISP_BL68Y6 */


#if defined(CONFIG_MACH_DECKARD_AF33)
#include "./data/shdisp_takt_data_af33.h"
#else  /* CONFIG_MACH_DEFAULT */
#include "./data/shdisp_takt_data_default.h"
#endif /* CONFIG_MACH_ */


/* ------------------------------------------------------------------------- */
/* MACROS                                                                    */
/* ------------------------------------------------------------------------- */

#define SHDISP_TAKT_1V_WAIT                 18000
#define SHDISP_TAKT_ALPHA_DEFAULT           0x0028

/* ------------------------------------------------------------------------- */
/* PROTOTYPES                                                                */
/* ------------------------------------------------------------------------- */

static int shdisp_takt_API_init_io(void);
static int shdisp_takt_API_exit_io(void);
static int shdisp_takt_API_init_isr(void);
static void shdisp_takt_API_set_param(struct shdisp_panel_param_str *param_str);
static int shdisp_takt_API_power_on(void);
static int shdisp_takt_API_disp_init_1st(void);
static int shdisp_takt_API_disp_init_2nd(void);
static int shdisp_takt_API_disp_on(void);
static int shdisp_takt_API_sleep(void);
static int shdisp_takt_API_deep_standby(void);
static int shdisp_takt_API_power_off(void);
static int shdisp_takt_API_check_upper_unit(void);
static int shdisp_takt_API_check_flicker_param(unsigned short alpha_in, unsigned short *alpha_out);
static int shdisp_takt_API_diag_write_reg(unsigned char addr, unsigned char *write_data, unsigned char size);
static int shdisp_takt_API_diag_read_reg(unsigned char addr, unsigned char *read_data, unsigned char size);
static int shdisp_takt_API_diag_set_flicker_param(unsigned short alpha);
static int shdisp_takt_API_diag_get_flicker_param(unsigned short *alpha);
static int shdisp_takt_API_check_recovery(void);
static int shdisp_takt_API_recovery_type(int *type);
static int shdisp_takt_API_set_abl_lut(struct dma_abl_lut_gamma *abl_lut_gammma);
static int shdisp_takt_API_disp_update(struct shdisp_main_update *update);
static int shdisp_takt_API_disp_clear_screen(struct shdisp_main_clear *clear);

static int shdisp_takt_IO_write_reg(unsigned char reg, unsigned char val);
static int shdisp_takt_IO_read_reg(unsigned char reg, unsigned char *val);
static int shdisp_takt_set_gam_tbl(void);


/* ------------------------------------------------------------------------- */
/* VARIABLES                                                                 */
/* ------------------------------------------------------------------------- */

static struct shdisp_panel_param_str s_param_str;


static struct shdisp_panel_operations shdisp_takt_fops = {
    shdisp_takt_API_init_io,
    shdisp_takt_API_exit_io,
    shdisp_takt_API_init_isr,
    shdisp_takt_API_set_param,
    shdisp_takt_API_power_on,
    shdisp_takt_API_power_off,
    shdisp_takt_API_disp_init_1st,
    shdisp_takt_API_disp_init_2nd,
    shdisp_takt_API_disp_on,
    shdisp_takt_API_sleep,
    shdisp_takt_API_deep_standby,
    shdisp_takt_API_check_upper_unit,
    shdisp_takt_API_check_flicker_param,
    shdisp_takt_API_diag_write_reg,
    shdisp_takt_API_diag_read_reg,
    shdisp_takt_API_diag_set_flicker_param,
    shdisp_takt_API_diag_get_flicker_param,
    shdisp_takt_API_check_recovery,
    shdisp_takt_API_recovery_type,
    shdisp_takt_API_set_abl_lut,
    shdisp_takt_API_disp_update,
    shdisp_takt_API_disp_clear_screen,
};

/* ------------------------------------------------------------------------- */
/* DEBUG MACRAOS                                                             */
/* ------------------------------------------------------------------------- */
#define SHDISP_SW_DISABLE_CHK_UPPER_UNIT


#define SHDISP_TAKT_FILE "shdisp_takt.c"
#define SHDISP_ERR(fmt, args...) \
        printk("[SHDISP_ERROR][%s][%s] " fmt, SHDISP_TAKT_FILE, __func__, ## args);

/* ------------------------------------------------------------------------- */
/* FUNCTIONS                                                                 */
/* ------------------------------------------------------------------------- */
/* ------------------------------------------------------------------------- */
/* API                                                                       */
/* ------------------------------------------------------------------------- */
/* ------------------------------------------------------------------------- */
/* shdisp_takt_API_create                                                    */
/* ------------------------------------------------------------------------- */
struct shdisp_panel_operations *shdisp_takt_API_create(void)
{
    return &shdisp_takt_fops;
}


/* ------------------------------------------------------------------------- */
/* shdisp_takt_API_init_io                                                   */
/* ------------------------------------------------------------------------- */
static int shdisp_takt_API_init_io(void)
{
    int ret = 0;
    
    ret = shdisp_SYS_takt_i2c_init();
    
    if (ret != SHDISP_RESULT_SUCCESS) {
        SHDISP_ERR("<RESULT_FAILURE> shdisp_SYS_takt_i2c_init.\n");
        return SHDISP_RESULT_FAILURE;
    }
    
    return SHDISP_RESULT_SUCCESS;
}


/* ------------------------------------------------------------------------- */
/* shdisp_takt_API_exit_io                                                   */
/* ------------------------------------------------------------------------- */
static int shdisp_takt_API_exit_io(void)
{
    shdisp_SYS_takt_i2c_exit();
    
    return SHDISP_RESULT_SUCCESS;
}


/* ------------------------------------------------------------------------- */
/* shdisp_takt_API_init_isr                                                  */
/* ------------------------------------------------------------------------- */
static int shdisp_takt_API_init_isr(void)
{
    return SHDISP_RESULT_SUCCESS;
}


/* ------------------------------------------------------------------------- */
/* shdisp_takt_API_set_param                                                 */
/* ------------------------------------------------------------------------- */
static void shdisp_takt_API_set_param(struct shdisp_panel_param_str *param_str)
{
    s_param_str.vcom_alpha = param_str->vcom_alpha;
    
    return;
}


/* ------------------------------------------------------------------------- */
/* shdisp_takt_API_power_on                                                  */
/* ------------------------------------------------------------------------- */
static int shdisp_takt_API_power_on(void)
{
    shdisp_SYS_delay_us(1000);
    shdisp_SYS_set_Host_gpio(SHDISP_GPIO_NUM_LCD_SCS_N, SHDISP_GPIO_CTL_HIGH);
    shdisp_SYS_delay_us(1000);
    shdisp_SYS_set_Host_gpio(SHDISP_GPIO_NUM_LCD_SCS_N, SHDISP_GPIO_CTL_LOW);
    shdisp_SYS_delay_us(1000);
    shdisp_SYS_set_Host_gpio(SHDISP_GPIO_NUM_LCD_SCS_N, SHDISP_GPIO_CTL_HIGH);
    shdisp_SYS_delay_us(1000);

    shdisp_bdic_API_LCD_release_hw_reset();
    shdisp_SYS_delay_us(1000);

    shdisp_takt_IO_write_reg(TAKT_REG_RESDSTB,         0x01);
    shdisp_SYS_delay_us(10);
    shdisp_takt_IO_write_reg(TAKT_REG_SWRES,           0xC0);
    shdisp_SYS_delay_us(10000);
    shdisp_bdic_API_LCD_power_on();
    shdisp_SYS_delay_us(5000);
    
    return SHDISP_RESULT_SUCCESS;
}

/* ------------------------------------------------------------------------- */
/* shdisp_takt_API_disp_init_1st                                             */
/* ------------------------------------------------------------------------- */
static int shdisp_takt_API_disp_init_1st(void)
{
/* 1 */
    shdisp_takt_IO_write_reg(TAKT_REG_BANKCTL,         0x00);
    /* version read */
/* 2 */
    shdisp_takt_IO_write_reg(TAKT_REG_NVMACS,          0x01);
    shdisp_takt_IO_write_reg(TAKT_REG_NVOPEN,          0x07);
    shdisp_takt_IO_write_reg(TAKT_REG_CLKSEL,          0x11);
    shdisp_takt_IO_write_reg(TAKT_REG_CLKCTL,          0x13);
    shdisp_SYS_delay_us(200);
    shdisp_takt_IO_write_reg(TAKT_REG_I2CNC,           0x01);
    shdisp_SYS_delay_us(1000);
    shdisp_takt_IO_write_reg(TAKT_REG_HRDCTL,          0x00);
/* 3 */
    shdisp_takt_IO_write_reg(TAKT_REG_TEST0,           0x02);
    shdisp_takt_IO_write_reg(TAKT_REG_VREFV,           0x0E);
    shdisp_takt_IO_write_reg(TAKT_REG_TEST1,           0x0F);
/* 4 */
    shdisp_takt_IO_write_reg(TAKT_REG_OSCSET,          0x1F);
    shdisp_takt_IO_write_reg(TAKT_REG_RXDIV,           0x42);
    shdisp_takt_IO_write_reg(TAKT_REG_BSTMDIV,         0x16);
    shdisp_takt_IO_write_reg(TAKT_REG_VGHLDIV,         0xB0);
    shdisp_takt_IO_write_reg(TAKT_REG_SWRDIV,          0x00);
    shdisp_takt_IO_write_reg(TAKT_REG_SWNVMDIV,        0x0F);
    shdisp_takt_IO_write_reg(TAKT_REG_NCOSCSET,        0x07);
    shdisp_takt_IO_write_reg(TAKT_REG_DFIM,            0x01);
    shdisp_takt_IO_write_reg(TAKT_REG_SEQSEL,          0x00);
    shdisp_takt_IO_write_reg(TAKT_REG_EXTDIV,          0x00);
/* 5 */
    shdisp_takt_IO_write_reg(TAKT_REG_IMGSET1,         0x00);
    shdisp_takt_IO_write_reg(TAKT_REG_IMGSET2,         0x00);
    shdisp_takt_IO_write_reg(TAKT_REG_IVBP_A,          0x03);
    shdisp_takt_IO_write_reg(TAKT_REG_IVNUMH_A,        0x03);
    shdisp_takt_IO_write_reg(TAKT_REG_IVNUML_A,        0xBF);
    shdisp_takt_IO_write_reg(TAKT_REG_IHNUMH_A,        0x02);
    shdisp_takt_IO_write_reg(TAKT_REG_IHNUML_A,        0x1B);
    shdisp_takt_IO_write_reg(TAKT_REG_IVBP_B,          0x03);
    shdisp_takt_IO_write_reg(TAKT_REG_IVNUMH_B,        0x03);
    shdisp_takt_IO_write_reg(TAKT_REG_IVNUML_B,        0xBF);
    shdisp_takt_IO_write_reg(TAKT_REG_IHNUMH_B,        0x02);
    shdisp_takt_IO_write_reg(TAKT_REG_IHNUML_B,        0x1B);
    shdisp_takt_IO_write_reg(TAKT_REG_ISVFP,           0x0A);
/* 6 */
    shdisp_takt_IO_write_reg(TAKT_REG_RXONDLY,         0x00);
    shdisp_takt_IO_write_reg(TAKT_REG_VDSYNCTL,        0x00);
    shdisp_takt_IO_write_reg(TAKT_REG_VDSET,           0x10);
    shdisp_takt_IO_write_reg(TAKT_REG_VDMODE_A,        0x05);
    shdisp_takt_IO_write_reg(TAKT_REG_VDERRCTL,        0x01);
    shdisp_takt_IO_write_reg(TAKT_REG_VFCODE,          0x03);
    shdisp_takt_IO_write_reg(TAKT_REG_VECODE,          0x0A);
    shdisp_takt_IO_write_reg(TAKT_REG_HFCODE,          0x05);
    shdisp_takt_IO_write_reg(TAKT_REG_HECODE,          0x09);
    shdisp_takt_IO_write_reg(TAKT_REG_SYNCODE1,        0xFF);
    shdisp_takt_IO_write_reg(TAKT_REG_SYNCODE2,        0x00);
    shdisp_takt_IO_write_reg(TAKT_REG_SYNCODE3,        0x00);
/* 7 */
    shdisp_takt_IO_write_reg(TAKT_REG_SLPOCTL,         0x00);
    shdisp_takt_IO_write_reg(TAKT_REG_DSPCTL1 ,        0x00);
    shdisp_takt_IO_write_reg(TAKT_REG_DSPCTL2,         0x01);
    shdisp_takt_IO_write_reg(TAKT_REG_PNLCTL1,         0x00);
    shdisp_takt_IO_write_reg(TAKT_REG_PNLCTL2,         0x00);
    shdisp_takt_IO_write_reg(TAKT_REG_MODECTL,         0x00);
    shdisp_takt_IO_write_reg(TAKT_REG_FRCCTL,          0x11);
    shdisp_takt_IO_write_reg(TAKT_REG_LPCTL,           0x00);
    shdisp_takt_IO_write_reg(TAKT_REG_SDCTL,           0x00);
    shdisp_takt_IO_write_reg(TAKT_REG_SDSET,           0x10);
    shdisp_takt_IO_write_reg(TAKT_REG_CMISET,          0x00);
    shdisp_takt_IO_write_reg(TAKT_REG_SD2NUM,          0xFF);
    shdisp_takt_IO_write_reg(TAKT_REG_SD3NUM,          0x00);
    shdisp_takt_IO_write_reg(TAKT_REG_RECCTL,          0x00);
    shdisp_takt_IO_write_reg(TAKT_REG_CHOPC,           0x00);
    shdisp_takt_IO_write_reg(TAKT_REG_TGSET1,          0x04);
    shdisp_takt_IO_write_reg(TAKT_REG_TGSET2,          0x00);
    shdisp_takt_IO_write_reg(TAKT_REG_GSPSET,          0x00);
    shdisp_takt_IO_write_reg(TAKT_REG_GCKSET,          0x05);
    shdisp_takt_IO_write_reg(TAKT_REG_SSDSET,          0x30);
    shdisp_takt_IO_write_reg(TAKT_REG_DREPSET,         0x00);
    shdisp_takt_IO_write_reg(TAKT_REG_TGOUTSET1,       0xF3);
    shdisp_takt_IO_write_reg(TAKT_REG_TGOUTSET2,       0x00);
    shdisp_takt_IO_write_reg(TAKT_REG_GSPDLY,          0x50);
    shdisp_takt_IO_write_reg(TAKT_REG_HVBSTDIV,        0x03);
    shdisp_takt_IO_write_reg(TAKT_REG_HVBSTST,         0x24);
    shdisp_takt_IO_write_reg(TAKT_REG_HVBSTWD,         0x2F);
    shdisp_takt_IO_write_reg(TAKT_REG_GCKST_SLF,       0x28);
    shdisp_takt_IO_write_reg(TAKT_REG_GCKWD_SLF,       0xA2);
    shdisp_takt_IO_write_reg(TAKT_REG_SSDST_SLF,       0x3A);
    shdisp_takt_IO_write_reg(TAKT_REG_SSDWD_SLF,       0x86);
    shdisp_takt_IO_write_reg(TAKT_REG_SSDITV_SLF,      0x12);
    shdisp_takt_IO_write_reg(TAKT_REG_CMICH_SLF,       0x26);
    shdisp_takt_IO_write_reg(TAKT_REG_PCHGST_SLF,      0x00);
    shdisp_takt_IO_write_reg(TAKT_REG_PCHGWD_SLF,      0x00);

#if 0
    shdisp_takt_IO_write_reg(TAKT_REG_GCKST_A, shdisp_takt_timing_tbl[SHDISP_MAIN_DISP_VDLINK_FREQ_149_6MHZ][0]);
    shdisp_takt_IO_write_reg(TAKT_REG_GCKWD_A, shdisp_takt_timing_tbl[SHDISP_MAIN_DISP_VDLINK_FREQ_149_6MHZ][1]);
    shdisp_takt_IO_write_reg(TAKT_REG_SSDST_A, shdisp_takt_timing_tbl[SHDISP_MAIN_DISP_VDLINK_FREQ_149_6MHZ][2]);
    shdisp_takt_IO_write_reg(TAKT_REG_SSDWD_A, shdisp_takt_timing_tbl[SHDISP_MAIN_DISP_VDLINK_FREQ_149_6MHZ][3]);
    shdisp_takt_IO_write_reg(TAKT_REG_SSDITV_A, shdisp_takt_timing_tbl[SHDISP_MAIN_DISP_VDLINK_FREQ_149_6MHZ][4]);
    shdisp_takt_IO_write_reg(TAKT_REG_CMICH_A, shdisp_takt_timing_tbl[SHDISP_MAIN_DISP_VDLINK_FREQ_149_6MHZ][5]);
    shdisp_takt_IO_write_reg(TAKT_REG_PCHGST_A, shdisp_takt_timing_tbl[SHDISP_MAIN_DISP_VDLINK_FREQ_149_6MHZ][6]);
    shdisp_takt_IO_write_reg(TAKT_REG_PCHGWD_A, shdisp_takt_timing_tbl[SHDISP_MAIN_DISP_VDLINK_FREQ_149_6MHZ][7]);
    shdisp_takt_IO_write_reg(TAKT_REG_HSLIMIT_A, shdisp_takt_timing_tbl[SHDISP_MAIN_DISP_VDLINK_FREQ_149_6MHZ][8]);
    shdisp_takt_IO_write_reg(TAKT_REG_SXDLY12_A, shdisp_takt_timing_tbl[SHDISP_MAIN_DISP_VDLINK_FREQ_149_6MHZ][9]);
    shdisp_takt_IO_write_reg(TAKT_REG_SXDLY34_A, shdisp_takt_timing_tbl[SHDISP_MAIN_DISP_VDLINK_FREQ_149_6MHZ][10]);
    shdisp_takt_IO_write_reg(TAKT_REG_SXDLY56_A, shdisp_takt_timing_tbl[SHDISP_MAIN_DISP_VDLINK_FREQ_149_6MHZ][11]);
    shdisp_takt_IO_write_reg(TAKT_REG_SXLWD12_A, shdisp_takt_timing_tbl[SHDISP_MAIN_DISP_VDLINK_FREQ_149_6MHZ][12]);
    shdisp_takt_IO_write_reg(TAKT_REG_SXLWD34_A, shdisp_takt_timing_tbl[SHDISP_MAIN_DISP_VDLINK_FREQ_149_6MHZ][13]);
    shdisp_takt_IO_write_reg(TAKT_REG_SXLWD56_A, shdisp_takt_timing_tbl[SHDISP_MAIN_DISP_VDLINK_FREQ_149_6MHZ][14]);
    shdisp_takt_IO_write_reg(TAKT_REG_SXBIAS_A, shdisp_takt_timing_tbl[SHDISP_MAIN_DISP_VDLINK_FREQ_149_6MHZ][15]);

    shdisp_takt_IO_write_reg(TAKT_REG_GCKST_B, shdisp_takt_timing_tbl[SHDISP_MAIN_DISP_VDLINK_FREQ_154_2MHZ][0]);
    shdisp_takt_IO_write_reg(TAKT_REG_GCKWD_B, shdisp_takt_timing_tbl[SHDISP_MAIN_DISP_VDLINK_FREQ_154_2MHZ][1]);
    shdisp_takt_IO_write_reg(TAKT_REG_SSDST_B, shdisp_takt_timing_tbl[SHDISP_MAIN_DISP_VDLINK_FREQ_154_2MHZ][2]);
    shdisp_takt_IO_write_reg(TAKT_REG_SSDWD_B, shdisp_takt_timing_tbl[SHDISP_MAIN_DISP_VDLINK_FREQ_154_2MHZ][3]);
    shdisp_takt_IO_write_reg(TAKT_REG_SSDITV_B, shdisp_takt_timing_tbl[SHDISP_MAIN_DISP_VDLINK_FREQ_154_2MHZ][4]);
    shdisp_takt_IO_write_reg(TAKT_REG_CMICH_B, shdisp_takt_timing_tbl[SHDISP_MAIN_DISP_VDLINK_FREQ_154_2MHZ][5]);
    shdisp_takt_IO_write_reg(TAKT_REG_PCHGST_B, shdisp_takt_timing_tbl[SHDISP_MAIN_DISP_VDLINK_FREQ_154_2MHZ][6]);
    shdisp_takt_IO_write_reg(TAKT_REG_PCHGWD_B, shdisp_takt_timing_tbl[SHDISP_MAIN_DISP_VDLINK_FREQ_154_2MHZ][7]);
    shdisp_takt_IO_write_reg(TAKT_REG_HSLIMIT_B, shdisp_takt_timing_tbl[SHDISP_MAIN_DISP_VDLINK_FREQ_154_2MHZ][8]);
    shdisp_takt_IO_write_reg(TAKT_REG_SXDLY12_B, shdisp_takt_timing_tbl[SHDISP_MAIN_DISP_VDLINK_FREQ_154_2MHZ][9]);
    shdisp_takt_IO_write_reg(TAKT_REG_SXDLY34_B, shdisp_takt_timing_tbl[SHDISP_MAIN_DISP_VDLINK_FREQ_154_2MHZ][10]);
    shdisp_takt_IO_write_reg(TAKT_REG_SXDLY56_B, shdisp_takt_timing_tbl[SHDISP_MAIN_DISP_VDLINK_FREQ_154_2MHZ][11]);
    shdisp_takt_IO_write_reg(TAKT_REG_SXLWD12_B, shdisp_takt_timing_tbl[SHDISP_MAIN_DISP_VDLINK_FREQ_154_2MHZ][12]);
    shdisp_takt_IO_write_reg(TAKT_REG_SXLWD34_B, shdisp_takt_timing_tbl[SHDISP_MAIN_DISP_VDLINK_FREQ_154_2MHZ][13]);
    shdisp_takt_IO_write_reg(TAKT_REG_SXLWD56_B, shdisp_takt_timing_tbl[SHDISP_MAIN_DISP_VDLINK_FREQ_154_2MHZ][14]);
    shdisp_takt_IO_write_reg(TAKT_REG_SXBIAS_B, shdisp_takt_timing_tbl[SHDISP_MAIN_DISP_VDLINK_FREQ_154_2MHZ][15]);
#endif
/* 8 */
    shdisp_takt_IO_write_reg(TAKT_REG_GCKST_A,         0x24);
    shdisp_takt_IO_write_reg(TAKT_REG_GCKWD_A,         0x9D);
    shdisp_takt_IO_write_reg(TAKT_REG_SSDST_A,         0x38);
    shdisp_takt_IO_write_reg(TAKT_REG_SSDWD_A,         0xC8);
    shdisp_takt_IO_write_reg(TAKT_REG_SSDITV_A,        0x13);
    shdisp_takt_IO_write_reg(TAKT_REG_CMICH_A,         0x1B);
    shdisp_takt_IO_write_reg(TAKT_REG_PCHGST_A,        0x00);
    shdisp_takt_IO_write_reg(TAKT_REG_PCHGWD_A,        0x00);
    shdisp_takt_IO_write_reg(TAKT_REG_HSLIMIT_A,       0x2F);
    shdisp_takt_IO_write_reg(TAKT_REG_SXDLY12_A,       0x00);
    shdisp_takt_IO_write_reg(TAKT_REG_SXDLY34_A,       0x00);
    shdisp_takt_IO_write_reg(TAKT_REG_SXDLY56_A,       0x00);
    shdisp_takt_IO_write_reg(TAKT_REG_SXLWD12_A,       0x66);
    shdisp_takt_IO_write_reg(TAKT_REG_SXLWD34_A,       0x66);
    shdisp_takt_IO_write_reg(TAKT_REG_SXLWD56_A,       0x66);
    shdisp_takt_IO_write_reg(TAKT_REG_SXBIAS_A,        0x30);
/* 9 */
    shdisp_takt_IO_write_reg(TAKT_REG_GCKST_B,         0x24);
    shdisp_takt_IO_write_reg(TAKT_REG_GCKWD_B,         0x9D);
    shdisp_takt_IO_write_reg(TAKT_REG_SSDST_B,         0x38);
    shdisp_takt_IO_write_reg(TAKT_REG_SSDWD_B,         0xC8);
    shdisp_takt_IO_write_reg(TAKT_REG_SSDITV_B,        0x13);
    shdisp_takt_IO_write_reg(TAKT_REG_CMICH_B,         0x1B);
    shdisp_takt_IO_write_reg(TAKT_REG_PCHGST_B,        0x00);
    shdisp_takt_IO_write_reg(TAKT_REG_PCHGWD_B,        0x00);
    shdisp_takt_IO_write_reg(TAKT_REG_HSLIMIT_B,       0x7F);
    shdisp_takt_IO_write_reg(TAKT_REG_SXDLY12_B,       0x00);
    shdisp_takt_IO_write_reg(TAKT_REG_SXDLY34_B,       0x00);
    shdisp_takt_IO_write_reg(TAKT_REG_SXDLY56_B,       0x00);
    shdisp_takt_IO_write_reg(TAKT_REG_SXLWD12_B,       0x66);
    shdisp_takt_IO_write_reg(TAKT_REG_SXLWD34_B,       0x66);
    shdisp_takt_IO_write_reg(TAKT_REG_SXLWD56_B,       0x66);
    shdisp_takt_IO_write_reg(TAKT_REG_SXBIAS_B,        0x30);
/* 10 */
    shdisp_takt_IO_write_reg(TAKT_REG_POWCTL1,         0x00);
    shdisp_takt_IO_write_reg(TAKT_REG_POWCTL2,         0x00);
    shdisp_takt_IO_write_reg(TAKT_REG_SWRSET1,         0x01);
    shdisp_takt_IO_write_reg(TAKT_REG_SWRSET2,         0x00);
    shdisp_takt_IO_write_reg(TAKT_REG_DCAPBBS,         0x00);
    shdisp_takt_IO_write_reg(TAKT_REG_VRGMV,           0x07);
    shdisp_takt_IO_write_reg(TAKT_REG_VRCOMDCV,        0x4A);
    shdisp_takt_IO_write_reg(TAKT_REG_BIASSET,         0x05);
    shdisp_takt_IO_write_reg(TAKT_REG_GMCOMBS,         0x22);
    shdisp_takt_IO_write_reg(TAKT_REG_VGHPWR,          0x00);
    shdisp_takt_IO_write_reg(TAKT_REG_DSIVC,           0x00);
    shdisp_takt_IO_write_reg(TAKT_REG_DSISET,          0x23);
    shdisp_takt_IO_write_reg(TAKT_REG_DSIICONT,        0x07);
/* 11 */
    shdisp_takt_IO_write_reg(TAKT_REG_PVCH_A,          0x6E);
    shdisp_takt_IO_write_reg(TAKT_REG_PVCL_A,          0x31);
    shdisp_takt_IO_write_reg(TAKT_REG_NVCH_A,          0x67);
    shdisp_takt_IO_write_reg(TAKT_REG_NVCL_A,          0x2B);
    shdisp_takt_IO_write_reg(TAKT_REG_COMDCV_A,        (SHDISP_TAKT_VCOM_DEFAULT - SHDISP_TAKT_VCOM_OFFSET + s_param_str.vcom_alpha));
    shdisp_takt_IO_write_reg(TAKT_REG_PVCH_B,          0x6E);
    shdisp_takt_IO_write_reg(TAKT_REG_PVCL_B,          0x31);
    shdisp_takt_IO_write_reg(TAKT_REG_NVCH_B,          0x67);
    shdisp_takt_IO_write_reg(TAKT_REG_NVCL_B,          0x2B);
    shdisp_takt_IO_write_reg(TAKT_REG_COMDCV_B,        (SHDISP_TAKT_VCOM_DEFAULT - SHDISP_TAKT_VCOM_OFFSET + s_param_str.vcom_alpha));
    shdisp_takt_IO_write_reg(TAKT_REG_VCCOPY,          0x01);
    shdisp_takt_IO_write_reg(TAKT_REG_BKVCH,           0x00);
    shdisp_takt_IO_write_reg(TAKT_REG_BKVCL,           0x7F);
    shdisp_takt_IO_write_reg(TAKT_REG_BKNVCH,          0x00);
    shdisp_takt_IO_write_reg(TAKT_REG_BKNVCL,          0x7F);
/* 12 */
    shdisp_takt_IO_write_reg(TAKT_REG_INTCTL,          0x08);
    shdisp_takt_IO_write_reg(TAKT_REG_DSIMSK1,         0xFF);
    shdisp_takt_IO_write_reg(TAKT_REG_DSIMSK2,         0x3F);
#if 0
    shdisp_takt_IO_write_reg(TAKT_REG_BANKCTL,         0x08);
    shdisp_takt_IO_write_reg(TAKT_CSTM_REG_CPFTRVPOS,  0x91);
    shdisp_takt_IO_write_reg(TAKT_CSTM_REG_CPFCTRL,    0x10);
    shdisp_takt_IO_write_reg(TAKT_CSTM_REG_CPFMODE,    0x11);
    shdisp_takt_IO_write_reg(TAKT_CSTM_REG_CPFSTRDAT0, 0xBA);
    shdisp_takt_IO_write_reg(TAKT_CSTM_REG_CPFSTRDAT1, 0xF2);
    shdisp_takt_IO_write_reg(TAKT_CSTM_REG_CPFSTRDAT2, 0xF2);
    shdisp_takt_IO_write_reg(TAKT_CSTM_REG_CPFSTRDAT3, 0xF4);
    shdisp_takt_IO_write_reg(TAKT_CSTM_REG_CPFSTRDAT4, 0x00);
    shdisp_takt_IO_write_reg(TAKT_REG_BANKCTL,         0x00);
#endif
/* 13 */

    return SHDISP_RESULT_SUCCESS;
}

/* ------------------------------------------------------------------------- */
/* shdisp_takt_API_disp_init_2nd                                             */
/* ------------------------------------------------------------------------- */
static int shdisp_takt_API_disp_init_2nd(void)
{
    return SHDISP_RESULT_SUCCESS;
}


/* ------------------------------------------------------------------------- */
/* shdisp_takt_API_disp_on                                                   */
/* ------------------------------------------------------------------------- */
static int shdisp_takt_API_disp_on(void)
{
/* 1 */
    shdisp_takt_IO_write_reg(TAKT_REG_VGHLDIV,         0xB0);
    shdisp_takt_IO_write_reg(TAKT_REG_VGHPWR,          0x00);
    shdisp_takt_IO_write_reg(TAKT_REG_POWCTL1,         0x01);
    shdisp_SYS_delay_us(1000);
    shdisp_takt_IO_write_reg(TAKT_REG_POWCTL1,         0x41);
    shdisp_SYS_delay_us(15000);
    shdisp_takt_IO_write_reg(TAKT_REG_POWCTL1,         0x61);
    shdisp_SYS_delay_us(1000);
    shdisp_takt_IO_write_reg(TAKT_REG_VGHLDIV,         0xBB);
    shdisp_SYS_delay_us(15000);
    shdisp_takt_IO_write_reg(TAKT_REG_VGHPWR,          0x03);
    shdisp_SYS_delay_us(1000);
    shdisp_takt_IO_write_reg(TAKT_REG_VGHPWR,          0x07);
    shdisp_takt_IO_write_reg(TAKT_REG_POWCTL1,         0x63);
    shdisp_SYS_delay_us(1000);
    shdisp_takt_IO_write_reg(TAKT_REG_SDCTL,           0x05);
    shdisp_SYS_delay_us(1000);
    shdisp_takt_IO_write_reg(TAKT_REG_SDCTL,           0x0F);
    shdisp_SYS_delay_us(1000);
    shdisp_takt_IO_write_reg(TAKT_REG_PNLCTL1,         0x01);
    shdisp_takt_IO_write_reg(TAKT_REG_POWCTL1,         0x67);
    shdisp_takt_IO_write_reg(TAKT_REG_POWCTL2,         0x01);
    shdisp_SYS_delay_us(18000);
    shdisp_takt_set_gam_tbl();
    shdisp_takt_IO_write_reg(TAKT_REG_PNLCTL1,         0x00);
    shdisp_SYS_delay_us(3);
#if 0
    shdisp_takt_IO_write_reg(TAKT_REG_BANKCTL,         0x08);
    shdisp_takt_IO_write_reg(TAKT_CSTM_REG_CPFON,      0x01);
    shdisp_takt_IO_write_reg(TAKT_REG_BANKCTL,         0x00);
#endif
    shdisp_takt_IO_write_reg(TAKT_REG_DSPCTL1,         0x03);
    shdisp_SYS_delay_us(18000);
#if 0
    shdisp_takt_IO_write_reg(TAKT_REG_BANKCTL,         0x08);
    shdisp_takt_IO_write_reg(TAKT_CSTM_REG_CPFCTRL,    0x22);
    shdisp_takt_IO_write_reg(TAKT_CSTM_REG_CPFFIXLUT1, 0xFF);
    shdisp_takt_IO_write_reg(TAKT_CSTM_REG_CPFFIXLUT2, 0x00);
    shdisp_takt_IO_write_reg(TAKT_CSTM_REG_CPFTRVPOS,  0x00);
    shdisp_takt_IO_write_reg(TAKT_CSTM_REG_CPFON,      0x00);
    shdisp_takt_IO_write_reg(TAKT_REG_BANKCTL,         0x00);
#endif
    shdisp_takt_IO_write_reg(TAKT_REG_VALTRAN,         0x01);

#if 0
    unsigned char data;

    shdisp_SYS_delay_us(100000);
    shdisp_takt_IO_read_reg(CC, &data);
    shdisp_takt_IO_read_reg(CD, &data);
#endif

    return SHDISP_RESULT_SUCCESS;
}


/* ------------------------------------------------------------------------- */
/* shdisp_takt_API_sleep                                                     */
/* ------------------------------------------------------------------------- */
static int shdisp_takt_API_sleep(void)
{
#if 0
    shdisp_takt_IO_write_reg(TAKT_REG_BANKCTL,         0x08);
    shdisp_takt_IO_write_reg(TAKT_CSTM_REG_CPFCTRL,    0x10);
    shdisp_takt_IO_write_reg(TAKT_CSTM_REG_CPFTRVPOS,  0x90);
    shdisp_takt_IO_write_reg(TAKT_CSTM_REG_CPFON,      0x01);
    shdisp_takt_IO_write_reg(TAKT_REG_BANKCTL,         0x00);
    shdisp_takt_IO_write_reg(TAKT_REG_VALTRAN,         0x01);
    shdisp_SYS_delay_us(SHDISP_TAKT_1V_WAIT);
#endif
    shdisp_takt_IO_write_reg(TAKT_REG_CMISET,          0x04);
    shdisp_takt_IO_write_reg(TAKT_REG_POWCTL2,         0x00);
    shdisp_takt_IO_write_reg(TAKT_REG_POWCTL1,         0x61);
    shdisp_takt_IO_write_reg(TAKT_REG_VALTRAN,         0x01);
    shdisp_SYS_delay_us(SHDISP_TAKT_1V_WAIT);
    shdisp_takt_IO_write_reg(TAKT_REG_SDCTL,           0x00);
    shdisp_takt_IO_write_reg(TAKT_REG_DSPCTL1,         0x00);
    shdisp_SYS_delay_us(SHDISP_TAKT_1V_WAIT);
    shdisp_takt_IO_write_reg(TAKT_REG_POWCTL1,         0x21);
    shdisp_takt_IO_write_reg(TAKT_REG_POWCTL1,         0x01);
    shdisp_takt_IO_write_reg(TAKT_REG_POWCTL1,         0x00);
    shdisp_SYS_delay_us(SHDISP_TAKT_1V_WAIT);

    return SHDISP_RESULT_SUCCESS;
}


/* ------------------------------------------------------------------------- */
/* shdisp_takt_API_deep_standby                                              */
/* ------------------------------------------------------------------------- */
static int shdisp_takt_API_deep_standby(void)
{
  
    return SHDISP_RESULT_SUCCESS;
}


/* ------------------------------------------------------------------------- */
/* shdisp_takt_API_power_off                                                 */
/* ------------------------------------------------------------------------- */
static int shdisp_takt_API_power_off(void)
{
/* 1 */
    shdisp_bdic_API_LCD_power_off();
    shdisp_SYS_delay_us(1000);
/* 2 */
    shdisp_takt_IO_write_reg(TAKT_REG_RESDSTB, 0x00);
    shdisp_SYS_delay_us(1000);
/* 3 */
    shdisp_SYS_set_Host_gpio(SHDISP_GPIO_NUM_LCD_SCS_N, SHDISP_GPIO_CTL_HIGH);
    shdisp_SYS_delay_us(1000);
    
    return SHDISP_RESULT_SUCCESS;
}


/* ------------------------------------------------------------------------- */
/* shdisp_takt_API_check_upper_unit                                          */
/* ------------------------------------------------------------------------- */
static int shdisp_takt_API_check_upper_unit(void)
{
#ifndef SHDISP_SW_DISABLE_CHK_UPPER_UNIT
    int connection = -1;
    unsigned char dummy;
       
    connection = shdisp_takt_IO_read_reg(TAKT_REG_VER_MAJOR, &dummy);
    
    if (connection != SHDISP_RESULT_SUCCESS) {
        SHDISP_ERR("<OTHER> Upper unit does not exist.\n");
        return SHDISP_RESULT_FAILURE;
    }
#endif  /* SHDISP_SW_DISABLE_CHK_UPPER_UNIT */
    return SHDISP_RESULT_SUCCESS;
}


/* ------------------------------------------------------------------------- */
/* shdisp_takt_API_check_flicker_param                                       */
/* ------------------------------------------------------------------------- */
static int shdisp_takt_API_check_flicker_param(unsigned short alpha_in, unsigned short *alpha_out)
{
    if (alpha_out == NULL){
        SHDISP_ERR("<NULL_POINTER> alpha_out.\n");
        return SHDISP_RESULT_FAILURE;
    }
    
    *alpha_out = SHDISP_TAKT_ALPHA_DEFAULT;
    
    return SHDISP_RESULT_SUCCESS;
}


/* ------------------------------------------------------------------------- */
/* shdisp_takt_API_diag_write_reg                                            */
/* ------------------------------------------------------------------------- */
static int shdisp_takt_API_diag_write_reg(unsigned char addr, unsigned char *write_data, unsigned char size)
{
    shdisp_takt_IO_write_reg(addr,         *write_data);    
    return SHDISP_RESULT_SUCCESS;
}


/* ------------------------------------------------------------------------- */
/* shdisp_takt_API_diag_read_reg                                             */
/* ------------------------------------------------------------------------- */
static int shdisp_takt_API_diag_read_reg(unsigned char addr, unsigned char *read_data, unsigned char size)
{
    if (read_data == NULL){
        SHDISP_ERR("<NULL_POINTER> read_data.\n");
        return SHDISP_RESULT_FAILURE;
    }
     
    shdisp_takt_IO_read_reg(addr, read_data);
    
    return SHDISP_RESULT_SUCCESS;
}


/* ------------------------------------------------------------------------- */
/* shdisp_takt_API_diag_set_flicker_param                                    */
/* ------------------------------------------------------------------------- */
static int shdisp_takt_API_diag_set_flicker_param(unsigned short alpha)
{
    unsigned char tmp_alpha = 0;
    
    if (alpha > SHDISP_TAKT_VCOM_MAX) {
        SHDISP_ERR("<INVALID_VALUE> alpha(0x%04X).\n", alpha);
        return SHDISP_RESULT_FAILURE;
    }
    
    tmp_alpha = SHDISP_TAKT_VCOM_DEFAULT - SHDISP_TAKT_VCOM_OFFSET + (unsigned short)alpha;
    
    shdisp_takt_IO_write_reg(TAKT_REG_COMDCV_A, tmp_alpha);
    shdisp_takt_IO_write_reg(TAKT_REG_COMDCV_B, tmp_alpha);
    
    s_param_str.vcom_alpha = alpha;
    
    return SHDISP_RESULT_SUCCESS;
}


/* ------------------------------------------------------------------------- */
/* shdisp_takt_API_diag_get_flicker_param                                    */
/* ------------------------------------------------------------------------- */
static int shdisp_takt_API_diag_get_flicker_param(unsigned short *alpha)
{
    unsigned char tmp_alpha = 0;
    
    if (alpha == NULL){
        SHDISP_ERR("<NULL_POINTER> alpha.\n");
        return SHDISP_RESULT_FAILURE;
    }
    
    shdisp_takt_IO_read_reg(TAKT_REG_COMDCV_A, &tmp_alpha);
    
    *alpha = (unsigned short)(tmp_alpha - (SHDISP_TAKT_VCOM_DEFAULT - SHDISP_TAKT_VCOM_OFFSET));
    
    return SHDISP_RESULT_SUCCESS;
}


/* ------------------------------------------------------------------------- */
/* shdisp_takt_API_check_recovery                                            */
/* ------------------------------------------------------------------------- */
static int shdisp_takt_API_check_recovery(void)
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
/* shdisp_takt_API_recovery_type                                             */
/* ------------------------------------------------------------------------- */
static int shdisp_takt_API_recovery_type(int *type)
{
    *type = SHDISP_SUBSCRIBE_TYPE_INT;

    return SHDISP_RESULT_SUCCESS;
}


/* ------------------------------------------------------------------------- */
/* shdisp_takt_API_set_abl_lut                                               */
/* ------------------------------------------------------------------------- */
static int shdisp_takt_API_set_abl_lut(struct dma_abl_lut_gamma *abl_lut_gammma)
{
    return SHDISP_RESULT_SUCCESS;
}


/* ------------------------------------------------------------------------- */
/* shdisp_takt_API_disp_update                                               */
/* ------------------------------------------------------------------------- */
static int shdisp_takt_API_disp_update(struct shdisp_main_update *update)
{
    return SHDISP_RESULT_SUCCESS;
}


/* ------------------------------------------------------------------------- */
/* shdisp_takt_API_disp_clear_screen                                         */
/* ------------------------------------------------------------------------- */
static int shdisp_takt_API_disp_clear_screen(struct shdisp_main_clear *clear)
{
    return SHDISP_RESULT_SUCCESS;
}


/* ------------------------------------------------------------------------- */
/* shdisp_takt_IO_write_reg                                                  */
/* ------------------------------------------------------------------------- */

static int shdisp_takt_IO_write_reg(unsigned char reg, unsigned char val)
{
    int ret;
    
    ret = shdisp_SYS_takt_i2c_write(reg, &val, 1);
    
    return ret;
}


/* ------------------------------------------------------------------------- */
/* shdisp_takt_IO_read_reg                                                   */
/* ------------------------------------------------------------------------- */

static int shdisp_takt_IO_read_reg(unsigned char reg, unsigned char *val)
{
    unsigned char read_val[2];
    int ret;

    read_val[0] = TAKT_REG_SRDADR;
    ret = shdisp_SYS_takt_i2c_read(reg, read_val, 1);
    *val = read_val[0];

    return ret;
}


/* ------------------------------------------------------------------------- */
/* shdisp_takt_set_gam_tbl                                                   */
/* ------------------------------------------------------------------------- */
static int shdisp_takt_set_gam_tbl(void)
{

    int i;
/* 1 */
    shdisp_takt_IO_write_reg(TAKT_REG_BANKCTL,         0x00);
    shdisp_takt_IO_write_reg(TAKT_REG_GAMCTL,          0x10);
    shdisp_SYS_delay_us(18000);
    shdisp_takt_IO_write_reg(TAKT_REG_GAMSET,          0x21);
    shdisp_takt_IO_write_reg(TAKT_REG_GAMVALPH,        0x00);
    shdisp_takt_IO_write_reg(TAKT_REG_GAMVALPL,        0x00);
    shdisp_takt_IO_write_reg(TAKT_REG_GAMVALNH,        0x03);
    shdisp_takt_IO_write_reg(TAKT_REG_GAMVALNL,        0xFF);

    shdisp_takt_IO_write_reg(TAKT_REG_BANKCTL,         0x04);
    for(i=0; i<SHDISP_GAM_BANK_REG_NUM; i++) {
        shdisp_takt_IO_write_reg(TAKT_GAM_REG_GMP0 + i,gamma_red_tbl[0][i]);
    }

    shdisp_takt_IO_write_reg(TAKT_REG_BANKCTL,         0x05);
    for(i=0; i<SHDISP_GAM_BANK_REG_NUM; i++) {
        shdisp_takt_IO_write_reg(TAKT_GAM_REG_GMP0 + i,gamma_red_tbl[1][i]);
    }

    shdisp_takt_IO_write_reg(TAKT_REG_BANKCTL,         0x06);
    for(i=0; i<SHDISP_GAM_BANK_REG_NUM; i++) {
        shdisp_takt_IO_write_reg(TAKT_GAM_REG_GMP0 + i,gamma_red_tbl[2][i]);
    }

    shdisp_takt_IO_write_reg(TAKT_REG_BANKCTL,         0x07);
    for(i=0; i<SHDISP_GAM_BANK_REG_NUM; i++) {
        shdisp_takt_IO_write_reg(TAKT_GAM_REG_GMP0 + i,gamma_red_tbl[3][i]);
    }
/* 2 */
    shdisp_takt_IO_write_reg(TAKT_REG_BANKCTL,         0x00);
    shdisp_takt_IO_write_reg(TAKT_REG_GAMSET,          0x22);
    shdisp_takt_IO_write_reg(TAKT_REG_GAMVALPH,        0x00);
    shdisp_takt_IO_write_reg(TAKT_REG_GAMVALPL,        0x00);
    shdisp_takt_IO_write_reg(TAKT_REG_GAMVALNH,        0x03);
    shdisp_takt_IO_write_reg(TAKT_REG_GAMVALNL,        0xFF);

    shdisp_takt_IO_write_reg(TAKT_REG_BANKCTL,         0x04);
    for(i=0; i<SHDISP_GAM_BANK_REG_NUM; i++) {
        shdisp_takt_IO_write_reg(TAKT_GAM_REG_GMP0 + i,gamma_green_tbl[0][i]);
    }

    shdisp_takt_IO_write_reg(TAKT_REG_BANKCTL,         0x05);
    for(i=0; i<SHDISP_GAM_BANK_REG_NUM; i++) {
        shdisp_takt_IO_write_reg(TAKT_GAM_REG_GMP0 + i,gamma_green_tbl[1][i]);
    }

    shdisp_takt_IO_write_reg(TAKT_REG_BANKCTL,         0x06);
    for(i=0; i<SHDISP_GAM_BANK_REG_NUM; i++) {
        shdisp_takt_IO_write_reg(TAKT_GAM_REG_GMP0 + i,gamma_green_tbl[2][i]);
    }

    shdisp_takt_IO_write_reg(TAKT_REG_BANKCTL,         0x07);
    for(i=0; i<SHDISP_GAM_BANK_REG_NUM; i++) {
        shdisp_takt_IO_write_reg(TAKT_GAM_REG_GMP0 + i,gamma_green_tbl[3][i]);
    }
/* 3 */
    shdisp_takt_IO_write_reg(TAKT_REG_BANKCTL,         0x00);
    shdisp_takt_IO_write_reg(TAKT_REG_GAMSET,          0x23);
    shdisp_takt_IO_write_reg(TAKT_REG_GAMVALPH,        0x00);
    shdisp_takt_IO_write_reg(TAKT_REG_GAMVALPL,        0x00);
    shdisp_takt_IO_write_reg(TAKT_REG_GAMVALNH,        0x03);
    shdisp_takt_IO_write_reg(TAKT_REG_GAMVALNL,        0xFF);

    shdisp_takt_IO_write_reg(TAKT_REG_BANKCTL,         0x04);
    for(i=0; i<SHDISP_GAM_BANK_REG_NUM; i++) {
        shdisp_takt_IO_write_reg(TAKT_GAM_REG_GMP0 + i,gamma_blue_tbl[0][i]);
    }

    shdisp_takt_IO_write_reg(TAKT_REG_BANKCTL,         0x05);
    for(i=0; i<SHDISP_GAM_BANK_REG_NUM; i++) {
        shdisp_takt_IO_write_reg(TAKT_GAM_REG_GMP0 + i,gamma_blue_tbl[1][i]);
    }

    shdisp_takt_IO_write_reg(TAKT_REG_BANKCTL,         0x06);
    for(i=0; i<SHDISP_GAM_BANK_REG_NUM; i++) {
        shdisp_takt_IO_write_reg(TAKT_GAM_REG_GMP0 + i,gamma_blue_tbl[2][i]);
    }

    shdisp_takt_IO_write_reg(TAKT_REG_BANKCTL,         0x07);
    for(i=0; i<SHDISP_GAM_BANK_REG_NUM; i++) {
        shdisp_takt_IO_write_reg(TAKT_GAM_REG_GMP0 + i,gamma_blue_tbl[3][i]);
    }
/* 4 */
    shdisp_takt_IO_write_reg(TAKT_REG_BANKCTL,         0x00);
    shdisp_takt_IO_write_reg(TAKT_REG_GAMSET,          0x24);
    shdisp_takt_IO_write_reg(TAKT_REG_GAMCTL,          0x11);

    return SHDISP_RESULT_SUCCESS;
}


MODULE_DESCRIPTION("SHARP DISPLAY DRIVER MODULE");
MODULE_LICENSE("GPL v2");
MODULE_AUTHOR("SHARP CORPORATION");
MODULE_VERSION("1.00");

/* ------------------------------------------------------------------------- */
/* END OF FILE                                                               */
/* ------------------------------------------------------------------------- */
