/* drivers/sharp/shdisp/shdisp_nicole.c  (Display Driver)
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
#include "shdisp_nicole.h"

//#ifdef SHDISP_BL68Y6
//#include "shdisp_bdic.h"
//#else /* SHDISP_BL68Y6 */
#include "shdisp_bl69y6.h"
//#endif /* SHDISP_BL68Y6 */

#include "shdisp_system.h"
#include "shdisp_type.h"



/* ------------------------------------------------------------------------- */
/* MACROS                                                                    */
/* ------------------------------------------------------------------------- */

#define SHDISP_NICOLE_1V_WAIT               18000
#define SHDISP_NICOLE_ALPHA_DEFAULT         0x0010



/* ------------------------------------------------------------------------- */
/* PROTOTYPES                                                                */
/* ------------------------------------------------------------------------- */

static int shdisp_nicole_API_init_io(void);
static int shdisp_nicole_API_exit_io(void);
static int shdisp_nicole_API_init_isr(void);
static void shdisp_nicole_API_set_param(struct shdisp_panel_param_str *param_str);
static int shdisp_nicole_API_power_on(void);
static int shdisp_nicole_API_disp_init_1st(void);
static int shdisp_nicole_API_disp_init_2nd(void);
static int shdisp_nicole_API_disp_on(void);
static int shdisp_nicole_API_sleep(void);
static int shdisp_nicole_API_deep_standby(void);
static int shdisp_nicole_API_power_off(void);
static int shdisp_nicole_API_check_upper_unit(void);
static int shdisp_nicole_API_check_flicker_param(unsigned short alpha_in, unsigned short *alpha_out);
static int shdisp_nicole_API_diag_write_reg(unsigned char addr, unsigned char *write_data, unsigned char size);
static int shdisp_nicole_API_diag_read_reg(unsigned char addr, unsigned char *read_data, unsigned char size);
static int shdisp_nicole_API_diag_set_flicker_param(unsigned short alpha);
static int shdisp_nicole_API_diag_get_flicker_param(unsigned short *alpha);
static int shdisp_nicole_API_check_recovery(void);
static int shdisp_nicole_API_recovery_type(int *type);
static int shdisp_nicole_API_set_abl_lut(struct dma_abl_lut_gamma *abl_lut_gammma);
static int shdisp_nicole_API_disp_update(struct shdisp_main_update *update);
static int shdisp_nicole_API_disp_clear_screen(struct shdisp_main_clear *clear);

static int shdisp_nicole_IO_write_reg(unsigned char reg, unsigned char *list, unsigned char size);
static int shdisp_nicole_IO_read_reg(unsigned char reg, unsigned char *list, unsigned char size);

static unsigned char nicole_wdata[NICOLE_NUM_I2C_CONTROL_MAX_WRITE];
static unsigned char nicole_rdata[NICOLE_NUM_I2C_CONTROL_MAX_READ];


/* ------------------------------------------------------------------------- */
/* VARIABLES                                                                 */
/* ------------------------------------------------------------------------- */

static struct shdisp_panel_param_str s_param_str;


static struct shdisp_panel_operations shdisp_nicole_fops = {
    shdisp_nicole_API_init_io,
    shdisp_nicole_API_exit_io,
    shdisp_nicole_API_init_isr,
    shdisp_nicole_API_set_param,
    shdisp_nicole_API_power_on,
    shdisp_nicole_API_power_off,
    shdisp_nicole_API_disp_init_1st,
    shdisp_nicole_API_disp_init_2nd,
    shdisp_nicole_API_disp_on,
    shdisp_nicole_API_sleep,
    shdisp_nicole_API_deep_standby,
    shdisp_nicole_API_check_upper_unit,
    shdisp_nicole_API_check_flicker_param,
    shdisp_nicole_API_diag_write_reg,
    shdisp_nicole_API_diag_read_reg,
    shdisp_nicole_API_diag_set_flicker_param,
    shdisp_nicole_API_diag_get_flicker_param,
    shdisp_nicole_API_check_recovery,
    shdisp_nicole_API_recovery_type,
    shdisp_nicole_API_set_abl_lut,
    shdisp_nicole_API_disp_update,
    shdisp_nicole_API_disp_clear_screen,
};

/* ------------------------------------------------------------------------- */
/* DEBUG MACRAOS                                                             */
/* ------------------------------------------------------------------------- */
#define SHDISP_SW_DISABLE_CHK_UPPER_UNIT

#define SHDISP_NICOLE_FILE "shdisp_nicole.c"
#define SHDISP_ERR(fmt, args...) \
        printk("[SHDISP_ERROR][%s][%s] " fmt, SHDISP_NICOLE_FILE, __func__, ## args);


/* ------------------------------------------------------------------------- */
/* FUNCTIONS                                                                 */
/* ------------------------------------------------------------------------- */
/* ------------------------------------------------------------------------- */
/* API                                                                       */
/* ------------------------------------------------------------------------- */
/* ------------------------------------------------------------------------- */
/* shdisp_nicole_API_create                                                  */
/* ------------------------------------------------------------------------- */
struct shdisp_panel_operations *shdisp_nicole_API_create(void)
{
    return &shdisp_nicole_fops;
}


/* ------------------------------------------------------------------------- */
/* shdisp_nicole_API_init_io                                                 */
/* ------------------------------------------------------------------------- */
static int shdisp_nicole_API_init_io(void)
{
    int ret = 0;
    
    ret = shdisp_SYS_nicole_i2c_init();
    
    if (ret != SHDISP_RESULT_SUCCESS) {
        SHDISP_ERR("<RESULT_FAILURE> shdisp_SYS_nicole_sio_init.\n");
        return SHDISP_RESULT_FAILURE;
    }
    
    return SHDISP_RESULT_SUCCESS;
}


/* ------------------------------------------------------------------------- */
/* shdisp_nicole_API_exit_io                                                 */
/* ------------------------------------------------------------------------- */
static int shdisp_nicole_API_exit_io(void)
{
    shdisp_SYS_nicole_i2c_exit();
    
    return SHDISP_RESULT_SUCCESS;
}


/* ------------------------------------------------------------------------- */
/* shdisp_nicole_API_init_isr                                                */
/* ------------------------------------------------------------------------- */
static int shdisp_nicole_API_init_isr(void)
{
    /* T.B.D */

    return SHDISP_RESULT_SUCCESS;
}


/* ------------------------------------------------------------------------- */
/* shdisp_nicole_API_set_param                                               */
/* ------------------------------------------------------------------------- */
static void shdisp_nicole_API_set_param(struct shdisp_panel_param_str *param_str)
{
    s_param_str.vcom_alpha = param_str->vcom_alpha;
    
    return;
}


/* ------------------------------------------------------------------------- */
/* shdisp_nicole_API_power_on                                                */
/* ------------------------------------------------------------------------- */
static int shdisp_nicole_API_power_on(void)
{
    shdisp_bdic_API_LCD_set_hw_reset();
    shdisp_SYS_delay_us(1000);
    shdisp_bdic_API_LCD_power_on();
    shdisp_SYS_delay_us(5000);
    shdisp_bdic_API_LCD_release_hw_reset();
    shdisp_SYS_delay_us(1);
    
    return SHDISP_RESULT_SUCCESS;
}


/* ------------------------------------------------------------------------- */
/* shdisp_nicole_API_disp_init_1st                                           */
/* ------------------------------------------------------------------------- */
static int shdisp_nicole_API_disp_init_1st(void)
{
    return SHDISP_RESULT_SUCCESS;
}


/* ------------------------------------------------------------------------- */
/* shdisp_nicole_API_disp_init_2nd                                           */
/* ------------------------------------------------------------------------- */
static int shdisp_nicole_API_disp_init_2nd(void)
{
    nicole_wdata[0]  = 0x00;
    shdisp_nicole_IO_write_reg(NICOLE_REG_BANK, nicole_wdata, 1);

    nicole_rdata[0] = 0x00;
    shdisp_nicole_IO_read_reg(NICOLE_REG_CHIPID, nicole_rdata, 1);

    nicole_wdata[0]  = 0x02;
    shdisp_nicole_IO_write_reg(NICOLE_REG_BANK, nicole_wdata, 1);

    nicole_wdata[0]  = 0x05;
    shdisp_nicole_IO_write_reg(NICOLE_REG_OTP_BIAS, nicole_wdata, 1);

    nicole_wdata[0]  = 0xA9;
    shdisp_nicole_IO_write_reg(NICOLE_REG_PASSWORD_SETTINGS1, nicole_wdata, 1);

    nicole_wdata[0]  = 0x55;
    shdisp_nicole_IO_write_reg(NICOLE_REG_PASSWORD_SETTINGS2, nicole_wdata, 1);

    nicole_wdata[0]  = 0x01;
    nicole_wdata[1]  = 0x00;
    nicole_wdata[2]  = 0x01;
    nicole_wdata[3]  = 0x00;
    nicole_wdata[4]  = 0x00;
    shdisp_nicole_IO_write_reg(NICOLE_REG_MANUALON, nicole_wdata, 5);

    nicole_wdata[0]  = 0x03;
    shdisp_nicole_IO_write_reg(NICOLE_REG_MIPI_DSI_LANE, nicole_wdata, 1);

    nicole_wdata[0]  = 0x01;
    shdisp_nicole_IO_write_reg(NICOLE_REG_OTP_LOAD, nicole_wdata, 1);

    nicole_wdata[0]  = 0x00;
    shdisp_nicole_IO_write_reg(NICOLE_REG_SLEEPOUT, nicole_wdata, 1);
    shdisp_SYS_delay_us(20000);

    nicole_wdata[0]  = 0x01;
    shdisp_nicole_IO_write_reg(NICOLE_REG_BANK, nicole_wdata, 1);

    nicole_wdata[0]  = 0x1A;
    shdisp_nicole_IO_write_reg(NICOLE_REG_RESOL, nicole_wdata, 1);

    nicole_wdata[0]  = 0x02;
    shdisp_nicole_IO_write_reg(NICOLE_REG_BANK, nicole_wdata, 1);

    nicole_wdata[0]  = SHDISP_NICOLE_GAMRP0;
    nicole_wdata[1]  = SHDISP_NICOLE_GAMRP1;
    nicole_wdata[2]  = SHDISP_NICOLE_GAMRP2;
    nicole_wdata[3]  = SHDISP_NICOLE_GAMRP3;
    nicole_wdata[4]  = SHDISP_NICOLE_GAMRP4;
    nicole_wdata[5]  = SHDISP_NICOLE_GAMRP5;
    nicole_wdata[6]  = SHDISP_NICOLE_GAMRP6;
    nicole_wdata[7]  = SHDISP_NICOLE_GAMRP7;
    shdisp_nicole_IO_write_reg(NICOLE_REG_GAMADJRP, nicole_wdata, 8);

    nicole_wdata[0]  = SHDISP_NICOLE_GAMRN0;
    nicole_wdata[1]  = SHDISP_NICOLE_GAMRN1;
    nicole_wdata[2]  = SHDISP_NICOLE_GAMRN2;
    nicole_wdata[3]  = SHDISP_NICOLE_GAMRN3;
    nicole_wdata[4]  = SHDISP_NICOLE_GAMRN4;
    nicole_wdata[5]  = SHDISP_NICOLE_GAMRN5;
    nicole_wdata[6]  = SHDISP_NICOLE_GAMRN6;
    nicole_wdata[7]  = SHDISP_NICOLE_GAMRN7;
    shdisp_nicole_IO_write_reg(NICOLE_REG_GAMADJRN, nicole_wdata, 8);

    nicole_wdata[0]  = SHDISP_NICOLE_GAMGP0;
    nicole_wdata[1]  = SHDISP_NICOLE_GAMGP1;
    nicole_wdata[2]  = SHDISP_NICOLE_GAMGP2;
    nicole_wdata[3]  = SHDISP_NICOLE_GAMGP3;
    nicole_wdata[4]  = SHDISP_NICOLE_GAMGP4;
    nicole_wdata[5]  = SHDISP_NICOLE_GAMGP5;
    nicole_wdata[6]  = SHDISP_NICOLE_GAMGP6;
    nicole_wdata[7]  = SHDISP_NICOLE_GAMGP7;
    shdisp_nicole_IO_write_reg(NICOLE_REG_GAMADJGP, nicole_wdata, 8);

    nicole_wdata[0]  = SHDISP_NICOLE_GAMGN0;
    nicole_wdata[1]  = SHDISP_NICOLE_GAMGN1;
    nicole_wdata[2]  = SHDISP_NICOLE_GAMGN2;
    nicole_wdata[3]  = SHDISP_NICOLE_GAMGN3;
    nicole_wdata[4]  = SHDISP_NICOLE_GAMGN4;
    nicole_wdata[5]  = SHDISP_NICOLE_GAMGN5;
    nicole_wdata[6]  = SHDISP_NICOLE_GAMGN6;
    nicole_wdata[7]  = SHDISP_NICOLE_GAMGN7;
    shdisp_nicole_IO_write_reg(NICOLE_REG_GAMADJGN, nicole_wdata, 8);

    nicole_wdata[0]  = SHDISP_NICOLE_GAMBP0;
    nicole_wdata[1]  = SHDISP_NICOLE_GAMBP1;
    nicole_wdata[2]  = SHDISP_NICOLE_GAMBP2;
    nicole_wdata[3]  = SHDISP_NICOLE_GAMBP3;
    nicole_wdata[4]  = SHDISP_NICOLE_GAMBP4;
    nicole_wdata[5]  = SHDISP_NICOLE_GAMBP5;
    nicole_wdata[6]  = SHDISP_NICOLE_GAMBP6;
    nicole_wdata[7]  = SHDISP_NICOLE_GAMBP7;
    shdisp_nicole_IO_write_reg(NICOLE_REG_GAMADJBP, nicole_wdata, 8);

    nicole_wdata[0]  = SHDISP_NICOLE_GAMBN0;
    nicole_wdata[1]  = SHDISP_NICOLE_GAMBN1;
    nicole_wdata[2]  = SHDISP_NICOLE_GAMBN2;
    nicole_wdata[3]  = SHDISP_NICOLE_GAMBN3;
    nicole_wdata[4]  = SHDISP_NICOLE_GAMBN4;
    nicole_wdata[5]  = SHDISP_NICOLE_GAMBN5;
    nicole_wdata[6]  = SHDISP_NICOLE_GAMBN6;
    nicole_wdata[7]  = SHDISP_NICOLE_GAMBN7;
    shdisp_nicole_IO_write_reg(NICOLE_REG_GAMADJBN, nicole_wdata, 8);

    nicole_wdata[0]  = 0x00;
    shdisp_nicole_IO_write_reg(NICOLE_REG_SELMODE, nicole_wdata, 1);

    nicole_wdata[0]  = 0x08;
    nicole_wdata[1]  = 0x44;
    nicole_wdata[2]  = 0x06;
    nicole_wdata[3]  = 0x2E;
    nicole_wdata[4]  = 0x00;
    nicole_wdata[5]  = 0x00;
    nicole_wdata[6]  = 0x30;
    nicole_wdata[7]  = 0x33;
    shdisp_nicole_IO_write_reg(NICOLE_REG_SET_DDVDHP, nicole_wdata, 8);

    nicole_wdata[0]  = 0x1F;
    nicole_wdata[1]  = 0x44;
    nicole_wdata[2]  = 0x10;
    nicole_wdata[3]  = 0x2E;
    nicole_wdata[4]  = 0x1F;
    nicole_wdata[5]  = 0x00;
    nicole_wdata[6]  = 0x30;
    nicole_wdata[7]  = 0x33;
    shdisp_nicole_IO_write_reg(NICOLE_REG_SET_DDVDHM, nicole_wdata, 8);

    nicole_wdata[0]  = 0x48;
    nicole_wdata[1]  = 0x11;
    nicole_wdata[2]  = 0x01;
    nicole_wdata[3]  = 0x00;
    nicole_wdata[4]  = 0x30;
    shdisp_nicole_IO_write_reg(NICOLE_REG_SET_VGH, nicole_wdata, 5);

    nicole_wdata[0]  = 0x4F;
    nicole_wdata[1]  = 0x11;
    nicole_wdata[2]  = 0x00;
    nicole_wdata[3]  = 0x00;
    nicole_wdata[4]  = 0x30;
    shdisp_nicole_IO_write_reg(NICOLE_REG_SET_VGL, nicole_wdata, 5);

    nicole_wdata[0]  = 0x11;
    nicole_wdata[1]  = 0x01;
    nicole_wdata[2]  = 0x00;
    nicole_wdata[3]  = 0x30;
    shdisp_nicole_IO_write_reg(NICOLE_REG_SET_VCL, nicole_wdata, 4);

    nicole_wdata[0]  = 0x01;
    nicole_wdata[1]  = 0x01;

#if 1
    nicole_wdata[2]  = SHDISP_NICOLE_COMDC;
#else
    nicole_wdata[2]  = (unsigned char)(s_param_str.vcom_alpha);
#endif
    shdisp_nicole_IO_write_reg(NICOLE_REG_VCSET, nicole_wdata, 3);

    nicole_wdata[0]  = SHDISP_NICOLE_SETVGMPM;
    shdisp_nicole_IO_write_reg(NICOLE_REG_SETVGMPM, nicole_wdata, 1);

    nicole_wdata[0]  = 0x32;
    nicole_wdata[1]  = 0x03;
    shdisp_nicole_IO_write_reg(NICOLE_REG_RBIAS, nicole_wdata, 2);

#if 1
    nicole_wdata[0]  = 0x04;
#else
    nicole_wdata[0]  = 0x03;
#endif
    shdisp_nicole_IO_write_reg(NICOLE_REG_TVBP, nicole_wdata, 1);

    nicole_wdata[0]  = 0x80;
    shdisp_nicole_IO_write_reg(NICOLE_REG_THDEHBP, nicole_wdata, 1);

    nicole_wdata[0]  = 0x01;
    shdisp_nicole_IO_write_reg(NICOLE_REG_BANK, nicole_wdata, 1);

    nicole_wdata[0]  = SHDISP_NICOLE_PNLCTL0;
    shdisp_nicole_IO_write_reg(NICOLE_REG_PANELCTL0, nicole_wdata, 1);

    nicole_wdata[0]  = SHDISP_NICOLE_PNLCTL1;
    shdisp_nicole_IO_write_reg(NICOLE_REG_PANELCTL1, nicole_wdata, 1);

    nicole_wdata[0]  = SHDISP_NICOLE_PNLCTL2;
    shdisp_nicole_IO_write_reg(NICOLE_REG_PANELCTL2, nicole_wdata, 1);

    nicole_wdata[0]  = SHDISP_NICOLE_PNLCTL3;
    shdisp_nicole_IO_write_reg(NICOLE_REG_PANELCTL3, nicole_wdata, 1);

    nicole_wdata[0]  = SHDISP_NICOLE_PNLCTL4;
    shdisp_nicole_IO_write_reg(NICOLE_REG_PANELCTL4, nicole_wdata, 1);

    nicole_wdata[0]  = SHDISP_NICOLE_PNLCTL5;
    shdisp_nicole_IO_write_reg(NICOLE_REG_PANELCTL5, nicole_wdata, 1);

    nicole_wdata[0]  = SHDISP_NICOLE_PNLCTL6;
    shdisp_nicole_IO_write_reg(NICOLE_REG_PANELCTL6, nicole_wdata, 1);

    nicole_wdata[0]  = 0x01;
    shdisp_nicole_IO_write_reg(0x96, nicole_wdata, 1);


    nicole_wdata[0]  = SHDISP_NICOLE_PNLCTL8;
    shdisp_nicole_IO_write_reg(NICOLE_REG_PANELCTL8, nicole_wdata, 1);

    nicole_wdata[0]  = SHDISP_NICOLE_PNLCTL9;
    shdisp_nicole_IO_write_reg(NICOLE_REG_SSDST, nicole_wdata, 1);

    nicole_wdata[0]  = SHDISP_NICOLE_PNLCTL10;
    shdisp_nicole_IO_write_reg(NICOLE_REG_SSDWD, nicole_wdata, 1);

    nicole_wdata[0]  = SHDISP_NICOLE_PNLCTL11;
    shdisp_nicole_IO_write_reg(NICOLE_REG_SSDITV, nicole_wdata, 1);

    nicole_wdata[0]  = 0x07;
    shdisp_nicole_IO_write_reg(NICOLE_REG_PCHLVL, nicole_wdata, 1);

    nicole_wdata[0]  = 0x08;
    shdisp_nicole_IO_write_reg(NICOLE_REG_PCHST, nicole_wdata, 1);

    nicole_wdata[0]  = 0x00;
    shdisp_nicole_IO_write_reg(NICOLE_REG_PCHWD, nicole_wdata, 1);

    nicole_wdata[0]  = 0x06;
    shdisp_nicole_IO_write_reg(NICOLE_REG_EQ1, nicole_wdata, 1);

    nicole_wdata[0]  = 0x06;
    shdisp_nicole_IO_write_reg(NICOLE_REG_EQ2, nicole_wdata, 1);

    nicole_wdata[0]  = 0x00;
    shdisp_nicole_IO_write_reg(NICOLE_REG_BLNKLB, nicole_wdata, 1);

    nicole_wdata[0]  = 0x00;
    shdisp_nicole_IO_write_reg(NICOLE_REG_SLINV, nicole_wdata, 1);

    nicole_wdata[0]  = 0x11;
    shdisp_nicole_IO_write_reg(NICOLE_REG_SLINVIDL, nicole_wdata, 1);

    nicole_wdata[0]  = 0x00;
    shdisp_nicole_IO_write_reg(NICOLE_REG_PB_CYCLE, nicole_wdata, 1);

    nicole_wdata[0]  = 0x00;
    shdisp_nicole_IO_write_reg(NICOLE_REG_PB_DD, nicole_wdata, 1);

    nicole_wdata[0]  = 0x0A;
    shdisp_nicole_IO_write_reg(NICOLE_REG_LS_CYCLE, nicole_wdata, 1);

    nicole_wdata[0]  = 0x00;
    shdisp_nicole_IO_write_reg(NICOLE_REG_LINEINV, nicole_wdata, 1);

    nicole_wdata[0]  = 0x00;
    shdisp_nicole_IO_write_reg(NICOLE_REG_PIXARRANGE, nicole_wdata, 1);

    nicole_wdata[0]  = 0x00;
    shdisp_nicole_IO_write_reg(NICOLE_REG_STRIPE, nicole_wdata, 1);

    nicole_wdata[0]  = 0x00;
    shdisp_nicole_IO_write_reg(NICOLE_REG_ZOOM, nicole_wdata, 1);

    nicole_wdata[0]  = 0x00;
    shdisp_nicole_IO_write_reg(NICOLE_REG_SXDLY1, nicole_wdata, 1);

    nicole_wdata[0]  = 0x00;
    shdisp_nicole_IO_write_reg(NICOLE_REG_SXDLY2, nicole_wdata, 1);

    nicole_wdata[0]  = 0x00;
    shdisp_nicole_IO_write_reg(NICOLE_REG_BANK, nicole_wdata, 1);

    nicole_wdata[0]  = 0x05;
    shdisp_nicole_IO_write_reg(NICOLE_REG_MIPI_ALTERNATIVE_CLK1, nicole_wdata, 1);

    nicole_wdata[0]  = 0x00;
    shdisp_nicole_IO_write_reg(NICOLE_REG_MIPI_ALTERNATIVE_CLK2, nicole_wdata, 1);

    nicole_wdata[0]  = 0x00;
    shdisp_nicole_IO_write_reg(NICOLE_REG_MIPI_ALTERNATIVE_CLK3, nicole_wdata, 1);

    nicole_wdata[0]  = 0x80;
    shdisp_nicole_IO_write_reg(NICOLE_REG_MIPI_ALTERNATIVE_CLK4, nicole_wdata, 1);

    return SHDISP_RESULT_SUCCESS;
}


/* ------------------------------------------------------------------------- */
/* shdisp_nicole_API_disp_on                                                 */
/* ------------------------------------------------------------------------- */
static int shdisp_nicole_API_disp_on(void)
{
    nicole_wdata[0]  = 0x00;
    shdisp_nicole_IO_write_reg(NICOLE_REG_BANK, nicole_wdata, 1);

    nicole_wdata[0]  = 0x01;
    nicole_wdata[1]  = 0x01;
    nicole_wdata[2]  = 0x01;
    nicole_wdata[3]  = 0x00;
    nicole_wdata[4]  = 0x00;
    shdisp_nicole_IO_write_reg(NICOLE_REG_MANUALON, nicole_wdata, 5);

    shdisp_SYS_delay_us(3000);

    nicole_wdata[0]  = 0x01;
    nicole_wdata[1]  = 0x03;
    nicole_wdata[2]  = 0x01;
    nicole_wdata[3]  = 0x00;
    nicole_wdata[4]  = 0x00;
    shdisp_nicole_IO_write_reg(NICOLE_REG_MANUALON, nicole_wdata, 5);

    shdisp_SYS_delay_us(1000);

    /* T.B.D -power */
    shdisp_bdic_API_LCD_m_power_on();
    shdisp_SYS_delay_us(3000);
#if 0
    nicole_wdata[0]  = 0x01;
    nicole_wdata[1]  = 0x0B;
    nicole_wdata[2]  = 0x01;
    nicole_wdata[3]  = 0x00;
    nicole_wdata[4]  = 0x00;
    shdisp_nicole_IO_write_reg(NICOLE_REG_MANUALON, nicole_wdata, 5);
    shdisp_SYS_delay_us(3000);
#endif

    nicole_wdata[0]  = 0x01;
    nicole_wdata[1]  = 0x23;
    nicole_wdata[2]  = 0x01;
    nicole_wdata[3]  = 0x00;
    nicole_wdata[4]  = 0x00;
    shdisp_nicole_IO_write_reg(NICOLE_REG_MANUALON, nicole_wdata, 5);
    shdisp_SYS_delay_us(7000);

    nicole_wdata[0]  = 0x01;
    nicole_wdata[1]  = 0x63;
    nicole_wdata[2]  = 0x01;
    nicole_wdata[3]  = 0x00;
    nicole_wdata[4]  = 0x00;
    shdisp_nicole_IO_write_reg(NICOLE_REG_MANUALON, nicole_wdata, 5);
    shdisp_SYS_delay_us(1000);

    nicole_wdata[0]  = 0x01;
    nicole_wdata[1]  = 0xE3;
    nicole_wdata[2]  = 0x01;
    nicole_wdata[3]  = 0x00;
    nicole_wdata[4]  = 0x00;
    shdisp_nicole_IO_write_reg(NICOLE_REG_MANUALON, nicole_wdata, 5);
    shdisp_SYS_delay_us(1000);

    nicole_wdata[0]  = 0x01;
    nicole_wdata[1]  = 0xE3;
    nicole_wdata[2]  = 0x31;
    nicole_wdata[3]  = 0x00;
    nicole_wdata[4]  = 0x00;
    shdisp_nicole_IO_write_reg(NICOLE_REG_MANUALON, nicole_wdata, 5);
    shdisp_SYS_delay_us(1000);

    nicole_wdata[0]  = 0x01;
    nicole_wdata[1]  = 0xE3;
    nicole_wdata[2]  = 0x33;
    nicole_wdata[3]  = 0x00;
    nicole_wdata[4]  = 0x02;
    shdisp_nicole_IO_write_reg(NICOLE_REG_MANUALON, nicole_wdata, 5);
    shdisp_SYS_delay_us(3000);

    nicole_wdata[0]  = 0x01;
    nicole_wdata[1]  = 0xE3;
    nicole_wdata[2]  = 0x31;
    nicole_wdata[3]  = 0x00;
    nicole_wdata[4]  = 0x03;
    shdisp_nicole_IO_write_reg(NICOLE_REG_MANUALON, nicole_wdata, 5);

    nicole_wdata[0]  = 0x01;
    nicole_wdata[1]  = 0xE3;
    nicole_wdata[2]  = 0x3D;
    nicole_wdata[3]  = 0x00;
    nicole_wdata[4]  = 0x00;
    shdisp_nicole_IO_write_reg(NICOLE_REG_MANUALON, nicole_wdata, 5);

    nicole_wdata[0]  = 0x01;
    nicole_wdata[1]  = 0xE3;
    nicole_wdata[2]  = 0x3D;
    nicole_wdata[3]  = 0x00;
    nicole_wdata[4]  = 0x02;
    shdisp_nicole_IO_write_reg(NICOLE_REG_MANUALON, nicole_wdata, 5);
    shdisp_SYS_delay_us(20000);

    nicole_wdata[0]  = 0x02;
    shdisp_nicole_IO_write_reg(NICOLE_REG_BANK, nicole_wdata, 1);

    nicole_wdata[0]  = 0x00;
    shdisp_nicole_IO_write_reg(NICOLE_REG_PASSWORD_SETTINGS1, nicole_wdata, 1);

    nicole_wdata[0]  = 0x00;
    shdisp_nicole_IO_write_reg(NICOLE_REG_PASSWORD_SETTINGS2, nicole_wdata, 1);

    nicole_wdata[0]  = 0x00;
    shdisp_nicole_IO_write_reg(NICOLE_REG_DISPON, nicole_wdata, 1);

    return SHDISP_RESULT_SUCCESS;
}


/* ------------------------------------------------------------------------- */
/* shdisp_nicole_API_sleep                                                   */
/* ------------------------------------------------------------------------- */
static int shdisp_nicole_API_sleep(void)
{
    nicole_wdata[0]  = 0x00;
    shdisp_nicole_IO_write_reg(NICOLE_REG_BANK, nicole_wdata, 1);

    nicole_wdata[0]  = 0x00;
    shdisp_nicole_IO_write_reg(NICOLE_REG_POFSEQB, nicole_wdata, 1);

    nicole_wdata[0]  = 0x00;
    shdisp_nicole_IO_write_reg(NICOLE_REG_DISPOFF, nicole_wdata, 1);

    shdisp_SYS_delay_us(SHDISP_NICOLE_1V_WAIT);

    nicole_wdata[0]  = 0x02;
    shdisp_nicole_IO_write_reg(NICOLE_REG_BANK, nicole_wdata, 1);

    nicole_wdata[0]  = 0x01;
    nicole_wdata[1]  = 0xE3;
    nicole_wdata[2]  = 0x30;
    nicole_wdata[3]  = 0x00;
    nicole_wdata[4]  = 0x00;
    shdisp_nicole_IO_write_reg(NICOLE_REG_MANUALON, nicole_wdata, 5);

    shdisp_SYS_delay_us(1000);

    nicole_wdata[0]  = 0x01;
    nicole_wdata[1]  = 0x03;
    nicole_wdata[2]  = 0x00;
    nicole_wdata[3]  = 0x00;
    nicole_wdata[4]  = 0x00;
    shdisp_nicole_IO_write_reg(NICOLE_REG_MANUALON, nicole_wdata, 5);

    shdisp_SYS_delay_us(2400);

    /* T.B.D -power */
    shdisp_bdic_API_LCD_m_power_off();
    shdisp_SYS_delay_us(2000);

    nicole_wdata[0]  = 0x01;
    nicole_wdata[1]  = 0x00;
    nicole_wdata[2]  = 0x00;
    nicole_wdata[3]  = 0x00;
    nicole_wdata[4]  = 0x00;
    shdisp_nicole_IO_write_reg(NICOLE_REG_MANUALON, nicole_wdata, 5);

    shdisp_SYS_delay_us(2000);

    nicole_wdata[0]  = 0x00;
    shdisp_nicole_IO_write_reg(NICOLE_REG_SLEEPIN, nicole_wdata, 1);

    return SHDISP_RESULT_SUCCESS;
}


/* ------------------------------------------------------------------------- */
/* shdisp_nicole_API_deep_standby                                            */
/* ------------------------------------------------------------------------- */
static int shdisp_nicole_API_deep_standby(void)
{
    nicole_wdata[0]  = 0x00;
    shdisp_nicole_IO_write_reg(NICOLE_REG_DEEP_STANDBY, nicole_wdata, 1);

    shdisp_SYS_delay_us(SHDISP_NICOLE_1V_WAIT + 1000);

    return SHDISP_RESULT_SUCCESS;
}


/* ------------------------------------------------------------------------- */
/* shdisp_nicole_API_power_off                                               */
/* ------------------------------------------------------------------------- */
static int shdisp_nicole_API_power_off(void)
{
    shdisp_bdic_API_LCD_power_off();
    shdisp_SYS_delay_us(10000);

    return SHDISP_RESULT_SUCCESS;
}


/* ------------------------------------------------------------------------- */
/* shdisp_nicole_API_check_upper_unit                                        */
/* ------------------------------------------------------------------------- */
static int shdisp_nicole_API_check_upper_unit(void)
{
#ifndef SHDISP_SW_DISABLE_CHK_UPPER_UNIT
    unsigned char chk_data = 0x11;
    
    nicole_rdata[0] = 0x00;
    shdisp_nicole_IO_read_reg(NICOLE_REG_CHIPID, nicole_rdata, 1);
    
    if (chk_data != nicole_rdata[0]) {
        SHDISP_ERR("<OTHER> Upper unit does not exist.\n");
        return SHDISP_RESULT_FAILURE;
    }
#endif  /* SHDISP_SW_DISABLE_CHK_UPPER_UNIT */
    return SHDISP_RESULT_SUCCESS;
}


/* ------------------------------------------------------------------------- */
/* shdisp_nicole_API_check_flicker_param                                     */
/* ------------------------------------------------------------------------- */
static int shdisp_nicole_API_check_flicker_param(unsigned short alpha_in, unsigned short *alpha_out)
{
    unsigned short tmp_alpha = alpha_in;
    
    if (alpha_out == NULL){
        SHDISP_ERR("<NULL_POINTER> alpha_out.\n");
        return SHDISP_RESULT_FAILURE;
    }
    
    if ((tmp_alpha & 0xF000) != 0x9000) {
        *alpha_out = SHDISP_NICOLE_ALPHA_DEFAULT;
        return SHDISP_RESULT_SUCCESS;
    }
    
    tmp_alpha = tmp_alpha & 0x00FF;
    if (tmp_alpha > SHDISP_NICOLE_VCOM_MAX) {
        *alpha_out = SHDISP_NICOLE_ALPHA_DEFAULT;
        return SHDISP_RESULT_SUCCESS;
    }
    
    *alpha_out = tmp_alpha;
    
    return SHDISP_RESULT_SUCCESS;
}


/* ------------------------------------------------------------------------- */
/* shdisp_nicole_API_diag_write_reg                                          */
/* ------------------------------------------------------------------------- */
static int shdisp_nicole_API_diag_write_reg(unsigned char addr, unsigned char *write_data, unsigned char size)
{
    if (write_data == NULL){
        SHDISP_ERR("<NULL_POINTER> write_data.\n");
        return SHDISP_RESULT_FAILURE;
    }
    
    if (size < NICOLE_NUM_I2C_CONTROL_MIN_WRITE || size > NICOLE_NUM_I2C_CONTROL_MAX_WRITE){
        SHDISP_ERR("<INVALID_VALUE> size(%d).\n", size);
        return SHDISP_RESULT_FAILURE;
    }
    
    shdisp_nicole_IO_write_reg(addr, write_data, size);

    return SHDISP_RESULT_SUCCESS;
}


/* ------------------------------------------------------------------------- */
/* shdisp_nicole_API_diag_read_reg                                           */
/* ------------------------------------------------------------------------- */
static int shdisp_nicole_API_diag_read_reg(unsigned char addr, unsigned char *read_data, unsigned char size)
{
    if (read_data == NULL){
        SHDISP_ERR("<NULL_POINTER> read_data.\n");
        return SHDISP_RESULT_FAILURE;
    }
    
    if (size < NICOLE_NUM_I2C_CONTROL_MIN_READ || size > NICOLE_NUM_I2C_CONTROL_MAX_READ){
        SHDISP_ERR("<INVALID_VALUE> size(%d).\n", size);
        return SHDISP_RESULT_FAILURE;
    }
    
    shdisp_nicole_IO_read_reg(addr, read_data, size);

    return SHDISP_RESULT_SUCCESS;
}


/* ------------------------------------------------------------------------- */
/* shdisp_nicole_API_diag_set_flicker_param                                  */
/* ------------------------------------------------------------------------- */
static int shdisp_nicole_API_diag_set_flicker_param(unsigned short alpha)
{
    if (alpha > SHDISP_NICOLE_VCOM_MAX) {
        SHDISP_ERR("<INVALID_VALUE> alpha(0x%04X).\n", alpha);
        return SHDISP_RESULT_FAILURE;
    }
    
    nicole_wdata[0]  = 0x02;
    shdisp_nicole_IO_write_reg(NICOLE_REG_BANK, nicole_wdata, 1);

    nicole_wdata[0]  = 0x01;
    nicole_wdata[1]  = 0x01;
    nicole_wdata[2]  = (unsigned char)alpha;
    shdisp_nicole_IO_write_reg(NICOLE_REG_VCSET, nicole_wdata, 3);
    
    s_param_str.vcom_alpha = alpha;

    return SHDISP_RESULT_SUCCESS;
}


/* ------------------------------------------------------------------------- */
/* shdisp_nicole_API_diag_get_flicker_param                                  */
/* ------------------------------------------------------------------------- */
static int shdisp_nicole_API_diag_get_flicker_param(unsigned short *alpha)
{
    int i;
    
    if (alpha == NULL){
        SHDISP_ERR("<NULL_POINTER> alpha.\n");
        return SHDISP_RESULT_FAILURE;
    }

    nicole_wdata[0]  = 0x02;
    shdisp_nicole_IO_write_reg(NICOLE_REG_BANK, nicole_wdata, 1);

    for (i=0; i<3; i++) {
        nicole_rdata[i] = 0;
    }
    
    shdisp_nicole_IO_read_reg(NICOLE_REG_VCSET, nicole_rdata, 3);
    
    *alpha = nicole_rdata[2];

    return SHDISP_RESULT_SUCCESS;
}


/* ------------------------------------------------------------------------- */
/* shdisp_nicole_API_check_recovery                                          */
/* ------------------------------------------------------------------------- */
static int shdisp_nicole_API_check_recovery(void)
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
/* shdisp_nicole_API_recovery_type                                           */
/* ------------------------------------------------------------------------- */
static int shdisp_nicole_API_recovery_type(int *type)
{
    *type = SHDISP_SUBSCRIBE_TYPE_INT;

    return SHDISP_RESULT_SUCCESS;
}


/* ------------------------------------------------------------------------- */
/* shdisp_nicole_API_set_abl_lut                                             */
/* ------------------------------------------------------------------------- */
static int shdisp_nicole_API_set_abl_lut(struct dma_abl_lut_gamma *abl_lut_gammma)
{
    return SHDISP_RESULT_SUCCESS;
}


/* ------------------------------------------------------------------------- */
/* shdisp_nicole_API_disp_update                                            */
/* ------------------------------------------------------------------------- */
static int shdisp_nicole_API_disp_update(struct shdisp_main_update *update)
{
    return SHDISP_RESULT_SUCCESS;
}


/* ------------------------------------------------------------------------- */
/* shdisp_nicole_API_disp_clear_screen                                            */
/* ------------------------------------------------------------------------- */
static int shdisp_nicole_API_disp_clear_screen(struct shdisp_main_clear *clear)
{
    return SHDISP_RESULT_SUCCESS;
}


/* ------------------------------------------------------------------------- */
/* shdisp_nicole_IO_write_reg                                                */
/* ------------------------------------------------------------------------- */

static int shdisp_nicole_IO_write_reg(unsigned char reg, unsigned char *list, unsigned char size)
{
    int ret;
    
    ret = shdisp_SYS_nicole_i2c_write(reg, list, (int)size);
    if (ret != SHDISP_RESULT_SUCCESS) {
        return SHDISP_RESULT_FAILURE;
    }
    
    return SHDISP_RESULT_SUCCESS;
}


/* ------------------------------------------------------------------------- */
/* shdisp_nicole_IO_read_reg                                                 */
/* ------------------------------------------------------------------------- */

static int shdisp_nicole_IO_read_reg(unsigned char reg, unsigned char *list, unsigned char size)
{
    int ret;
    
    nicole_wdata[0] = reg;
    ret = shdisp_nicole_IO_write_reg(NICOLE_REG_READCOMMAND, nicole_wdata , 1);
    if (ret == SHDISP_RESULT_SUCCESS) {
        shdisp_SYS_nicole_i2c_read(reg, list, (int)size);
    }
    
    return SHDISP_RESULT_SUCCESS;
}


MODULE_DESCRIPTION("SHARP DISPLAY DRIVER MODULE");
MODULE_LICENSE("GPL v2");
MODULE_AUTHOR("SHARP CORPORATION");
MODULE_VERSION("1.00");

/* ------------------------------------------------------------------------- */
/* END OF FILE                                                               */
/* ------------------------------------------------------------------------- */
