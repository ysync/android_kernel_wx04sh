/* drivers/sharp/shdisp/shdisp_pharaoh.c  (Display Driver)
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
#include "shdisp_pharaoh.h"
#include "shdisp_system.h"
#include "shdisp_type.h"

//#ifdef SHDISP_BL68Y6
//#include "shdisp_bdic.h"
//#else /* SHDISP_BL68Y6 */
#include "shdisp_bl69y6.h"
//#endif /* SHDISP_BL68Y6 */



/* ------------------------------------------------------------------------- */
/* MACROS                                                                    */
/* ------------------------------------------------------------------------- */

#define SHDISP_PHARAOH_1V_WAIT              18000
#if defined(CONFIG_MACH_LYNX_DL12)
    #define SHDISP_PHARAOH_ALPHA_DEFAULT        0x0151
#else
    #define SHDISP_PHARAOH_ALPHA_DEFAULT        0x0149
#endif
#define SHDISP_PHARAOH_CLOCK                13500000


/* ------------------------------------------------------------------------- */
/* PROTOTYPES                                                                */
/* ------------------------------------------------------------------------- */

static int shdisp_pharaoh_API_init_io(void);
static int shdisp_pharaoh_API_exit_io(void);
static int shdisp_pharaoh_API_init_isr(void);
static void shdisp_pharaoh_API_set_param(struct shdisp_panel_param_str *param_str);
static int shdisp_pharaoh_API_power_on(void);
static int shdisp_pharaoh_API_disp_init_1st(void);
static int shdisp_pharaoh_API_disp_init_2nd(void);
static int shdisp_pharaoh_API_disp_on(void);
static int shdisp_pharaoh_API_sleep(void);
static int shdisp_pharaoh_API_deep_standby(void);
static int shdisp_pharaoh_API_power_off(void);
static int shdisp_pharaoh_API_check_upper_unit(void);
static int shdisp_pharaoh_API_check_flicker_param(unsigned short alpha_in, unsigned short *alpha_out);
static int shdisp_pharaoh_API_diag_write_reg(unsigned char addr, unsigned char *write_data, unsigned char size);
static int shdisp_pharaoh_API_diag_read_reg(unsigned char addr, unsigned char *read_data, unsigned char size);
static int shdisp_pharaoh_API_diag_set_flicker_param(unsigned short alpha);
static int shdisp_pharaoh_API_diag_get_flicker_param(unsigned short *alpha);
static int shdisp_pharaoh_API_check_recovery(void);
static int shdisp_pharaoh_API_recovery_type(int *type);
static int shdisp_pharaoh_API_set_abl_lut(struct dma_abl_lut_gamma *abl_lut_gammma);
static int shdisp_pharaoh_API_disp_update(struct shdisp_main_update *update);
static int shdisp_pharaoh_API_disp_clear_screen(struct shdisp_main_clear *clear);

static int shdisp_pharaoh_IO_write_reg(unsigned char reg, unsigned char *list, unsigned char size);
static int shdisp_pharaoh_IO_read_reg(unsigned char reg, unsigned char *list, unsigned char size);

static unsigned char pharaoh_wdata[PHAR_NUM_SIO_CONTROL_MAX_WRITE];
static unsigned char pharaoh_rdata[PHAR_NUM_SIO_CONTROL_MAX_READ];


/* ------------------------------------------------------------------------- */
/* VARIABLES                                                                 */
/* ------------------------------------------------------------------------- */

static struct shdisp_panel_param_str s_param_str;


static struct shdisp_panel_operations shdisp_pharaoh_fops = {
    shdisp_pharaoh_API_init_io,
    shdisp_pharaoh_API_exit_io,
    shdisp_pharaoh_API_init_isr,
    shdisp_pharaoh_API_set_param,
    shdisp_pharaoh_API_power_on,
    shdisp_pharaoh_API_power_off,
    shdisp_pharaoh_API_disp_init_1st,
    shdisp_pharaoh_API_disp_init_2nd,
    shdisp_pharaoh_API_disp_on,
    shdisp_pharaoh_API_sleep,
    shdisp_pharaoh_API_deep_standby,
    shdisp_pharaoh_API_check_upper_unit,
    shdisp_pharaoh_API_check_flicker_param,
    shdisp_pharaoh_API_diag_write_reg,
    shdisp_pharaoh_API_diag_read_reg,
    shdisp_pharaoh_API_diag_set_flicker_param,
    shdisp_pharaoh_API_diag_get_flicker_param,
    shdisp_pharaoh_API_check_recovery,
    shdisp_pharaoh_API_recovery_type,
    shdisp_pharaoh_API_set_abl_lut,
    shdisp_pharaoh_API_disp_update,
    shdisp_pharaoh_API_disp_clear_screen,
};

/* ------------------------------------------------------------------------- */
/* DEBUG MACRAOS                                                             */
/* ------------------------------------------------------------------------- */
#define SHDISP_SW_DISABLE_CHK_UPPER_UNIT

#define SHDISP_PHARAOH_FILE "shdisp_pharaoh.c"
#define SHDISP_ERR(fmt, args...) \
        printk("[SHDISP_ERROR][%s][%s] " fmt, SHDISP_PHARAOH_FILE, __func__, ## args);

/* ------------------------------------------------------------------------- */
/* FUNCTIONS                                                                 */
/* ------------------------------------------------------------------------- */
/* ------------------------------------------------------------------------- */
/* API                                                                       */
/* ------------------------------------------------------------------------- */
/* ------------------------------------------------------------------------- */
/* shdisp_pharaoh_API_create                                                 */
/* ------------------------------------------------------------------------- */
struct shdisp_panel_operations *shdisp_pharaoh_API_create(void)
{
    return &shdisp_pharaoh_fops;
}


/* ------------------------------------------------------------------------- */
/* shdisp_pharaoh_API_init_io                                                */
/* ------------------------------------------------------------------------- */
static int shdisp_pharaoh_API_init_io(void)
{
    int ret = 0;
    
    if (shdisp_api_get_boot_disp_status() == SHDISP_MAIN_DISP_ON) {
        shdisp_SYS_Host_control(SHDISP_HOST_CTL_CMD_LCD_CLK_START, SHDISP_PHARAOH_CLOCK);
    }
    
    ret = shdisp_SYS_pharaoh_sio_init();
    
    if (ret != SHDISP_RESULT_SUCCESS) {
        SHDISP_ERR("<RESULT_FAILURE> shdisp_SYS_pharaoh_sio_init.\n");
        return SHDISP_RESULT_FAILURE;
    }
    
    return SHDISP_RESULT_SUCCESS;
}


/* ------------------------------------------------------------------------- */
/* shdisp_pharaoh_API_exit_io                                                */
/* ------------------------------------------------------------------------- */
static int shdisp_pharaoh_API_exit_io(void)
{
    shdisp_SYS_pharaoh_sio_exit();
    
    return SHDISP_RESULT_SUCCESS;
}


/* ------------------------------------------------------------------------- */
/* shdisp_pharaoh_API_init_isr                                               */
/* ------------------------------------------------------------------------- */
static int shdisp_pharaoh_API_init_isr(void)
{
    return SHDISP_RESULT_SUCCESS;
}



/* ------------------------------------------------------------------------- */
/* shdisp_pharaoh_API_set_param                                              */
/* ------------------------------------------------------------------------- */
static void shdisp_pharaoh_API_set_param(struct shdisp_panel_param_str *param_str)
{
    s_param_str.vcom_alpha = param_str->vcom_alpha;
    s_param_str.shdisp_lcd = param_str->shdisp_lcd;
    
    return;
}


/* ------------------------------------------------------------------------- */
/* shdisp_pharaoh_API_power_on                                               */
/* ------------------------------------------------------------------------- */
static int shdisp_pharaoh_API_power_on(void)
{
    shdisp_bdic_API_LCD_set_hw_reset();
    shdisp_SYS_delay_us(1000);
    shdisp_bdic_API_LCD_power_on();
    shdisp_bdic_API_LCD_m_power_on();
    shdisp_SYS_delay_us(4000);
    shdisp_bdic_API_LCD_release_hw_reset();
    shdisp_SYS_delay_us(5000);
    shdisp_SYS_Host_control(SHDISP_HOST_CTL_CMD_LCD_CLK_START, SHDISP_PHARAOH_CLOCK);
    
    return SHDISP_RESULT_SUCCESS;
}


/* ------------------------------------------------------------------------- */
/* shdisp_pharaoh_API_disp_init_1st                                          */
/* ------------------------------------------------------------------------- */
static int shdisp_pharaoh_API_disp_init_1st(void)
{
    return SHDISP_RESULT_SUCCESS;
}


/* ------------------------------------------------------------------------- */
/* shdisp_pharaoh_API_disp_init_2nd                                          */
/* ------------------------------------------------------------------------- */
static int shdisp_pharaoh_API_disp_init_2nd(void)
{
    pharaoh_wdata[0]  = 0x00;
    shdisp_pharaoh_IO_write_reg(PHAR_REG_MANUFACTURE_COMMAND_ACCESS_PROJECT, pharaoh_wdata, 1);
    
    pharaoh_rdata[0] = 0x00;
    pharaoh_rdata[1] = 0x00;
    pharaoh_rdata[2] = 0x00;
    pharaoh_rdata[3] = 0x00;
    pharaoh_rdata[4] = 0x00;
    shdisp_pharaoh_IO_read_reg(PHAR_REG_DEVICE_CODE_READ, pharaoh_rdata, 5);
    
    pharaoh_wdata[0]  = 0x00;
    shdisp_pharaoh_IO_write_reg(PHAR_REG_AUTO_COMMAND_ACCESS_PROJECT, pharaoh_wdata, 1);

    pharaoh_wdata[0]  = 0x09;
    shdisp_pharaoh_IO_write_reg(PHAR_REG_CLOCK_AND_INTERFACE_SETTING, pharaoh_wdata, 1);

    pharaoh_wdata[0]  = 0x02;
    shdisp_pharaoh_IO_write_reg(PHAR_REG_PIXEL_FORMAT_SETTING, pharaoh_wdata, 1);

    pharaoh_wdata[0]  = 0x51;
    pharaoh_wdata[1]  = 0xE3;
    shdisp_pharaoh_IO_write_reg(PHAR_REG_DSI_CONTROL, pharaoh_wdata, 2);


    pharaoh_wdata[0]  = 0x00;
    shdisp_pharaoh_IO_write_reg(PHAR_REG_CABC_ON_OFF, pharaoh_wdata, 1);


#if defined(CONFIG_MACH_LYNX_DL12)
    pharaoh_wdata[0]  = 0x41;
#else
    pharaoh_wdata[0]  = 0x40;
#endif
    pharaoh_wdata[1]  = 0x02;
    pharaoh_wdata[2]  = 0x7F;
    pharaoh_wdata[3]  = 0xC5;
    pharaoh_wdata[4]  = 0x13;
    shdisp_pharaoh_IO_write_reg(PHAR_REG_PANNEL_DRIVING_SETTING, pharaoh_wdata, 5);

    pharaoh_wdata[0]  = 0x00;
    pharaoh_wdata[1]  = 0xB0;
    pharaoh_wdata[2]  = 0x00;
    pharaoh_wdata[3]  = 0x00;
    pharaoh_wdata[4]  = 0xA5;
    pharaoh_wdata[5]  = 0x00;
    pharaoh_wdata[6]  = 0x00;
    pharaoh_wdata[7]  = 0x9F;
    pharaoh_wdata[8]  = 0x09;
    pharaoh_wdata[9]  = 0x20;
    pharaoh_wdata[10] = 0x09;
    pharaoh_wdata[11] = 0x00;
    pharaoh_wdata[12] = 0x00;
    pharaoh_wdata[13] = 0x00;
    pharaoh_wdata[14] = 0x01;
    shdisp_pharaoh_IO_write_reg(PHAR_REG_DISPLAY_H_TIMING_SETTING, pharaoh_wdata, 15);

    pharaoh_wdata[0]  = 0x00;
    pharaoh_wdata[1]  = 0x09;
    pharaoh_wdata[2]  = 0x09;
    pharaoh_wdata[3]  = 0x00;
    pharaoh_wdata[4]  = 0x00;
    shdisp_pharaoh_IO_write_reg(PHAR_REG_SOURCE_OUTPUT_SETTING, pharaoh_wdata, 5);

    pharaoh_wdata[0]  = 0x04;
    shdisp_pharaoh_IO_write_reg(PHAR_REG_GATE_DRIVE_IF_SETTING, pharaoh_wdata, 1);

    pharaoh_wdata[0]  = (unsigned char) (s_param_str.shdisp_lcd & 0x00FF);
    pharaoh_wdata[1]  = (unsigned char) ((s_param_str.shdisp_lcd >> 8) & 0x00FF);
    shdisp_pharaoh_IO_write_reg(PHAR_REG_LTPS_INTERFACE_MODE, pharaoh_wdata, 2);

    pharaoh_wdata[0]  = 0x00;
    pharaoh_wdata[1]  = 0x04;
    shdisp_pharaoh_IO_write_reg(PHAR_REG_PBCTRL_CONTROL, pharaoh_wdata, 2);

    pharaoh_wdata[0]  = 0x13;
    pharaoh_wdata[1]  = 0x20;
    pharaoh_wdata[2]  = 0x20;
    pharaoh_wdata[3]  = 0x7C;
    pharaoh_wdata[4]  = 0x00;
    pharaoh_wdata[5]  = 0x00;
    pharaoh_wdata[6]  = 0x00;
    pharaoh_wdata[7]  = 0x00;
    pharaoh_wdata[8]  = 0x00;
    pharaoh_wdata[9]  = 0x00;
    shdisp_pharaoh_IO_write_reg(PHAR_REG_DISPLAY_RGB_SWITCH_ORDER, pharaoh_wdata, 10);

#if defined(CONFIG_MACH_LYNX_DL12)
    pharaoh_wdata[0]  = 0x00;
#else
    pharaoh_wdata[0]  = 0x20;
#endif
    shdisp_pharaoh_IO_write_reg(PHAR_REG_LTPS_INTERFACE_CONTROL, pharaoh_wdata, 1);

    pharaoh_wdata[0]  = SHDISP_PHAR_CGMP00;
    pharaoh_wdata[1]  = SHDISP_PHAR_CGMP01;
    pharaoh_wdata[2]  = SHDISP_PHAR_CGMN00;
    pharaoh_wdata[3]  = SHDISP_PHAR_CGMN01;
    shdisp_pharaoh_IO_write_reg(PHAR_REG_GAMMA_CONTROL, pharaoh_wdata, 4);

    pharaoh_wdata[0]  = SHDISP_PHAR_RP00;
    pharaoh_wdata[1]  = SHDISP_PHAR_RP01;
    pharaoh_wdata[2]  = SHDISP_PHAR_RP02;
    pharaoh_wdata[3]  = SHDISP_PHAR_RP03;
    pharaoh_wdata[4]  = SHDISP_PHAR_RP04;
    pharaoh_wdata[5]  = SHDISP_PHAR_RP05;
    pharaoh_wdata[6]  = SHDISP_PHAR_RP06;
    pharaoh_wdata[7]  = SHDISP_PHAR_RP07;
    pharaoh_wdata[8]  = SHDISP_PHAR_RP08;
    pharaoh_wdata[9]  = SHDISP_PHAR_RP09;
    pharaoh_wdata[10] = SHDISP_PHAR_RP10;
    pharaoh_wdata[11] = SHDISP_PHAR_RP11;
    pharaoh_wdata[12] = SHDISP_PHAR_RP12;
    shdisp_pharaoh_IO_write_reg(PHAR_REG_GAMMA_CONTROL_OF_SET_A_POSITIVE, pharaoh_wdata, 13);

    pharaoh_wdata[0]  = SHDISP_PHAR_RN00;
    pharaoh_wdata[1]  = SHDISP_PHAR_RN01;
    pharaoh_wdata[2]  = SHDISP_PHAR_RN02;
    pharaoh_wdata[3]  = SHDISP_PHAR_RN03;
    pharaoh_wdata[4]  = SHDISP_PHAR_RN04;
    pharaoh_wdata[5]  = SHDISP_PHAR_RN05;
    pharaoh_wdata[6]  = SHDISP_PHAR_RN06;
    pharaoh_wdata[7]  = SHDISP_PHAR_RN07;
    pharaoh_wdata[8]  = SHDISP_PHAR_RN08;
    pharaoh_wdata[9]  = SHDISP_PHAR_RN09;
    pharaoh_wdata[10] = SHDISP_PHAR_RN10;
    pharaoh_wdata[11] = SHDISP_PHAR_RN11;
    pharaoh_wdata[12] = SHDISP_PHAR_RN12;
    shdisp_pharaoh_IO_write_reg(PHAR_REG_GAMMA_CONTROL_OF_SET_A_NEGATIVE, pharaoh_wdata, 13);

    pharaoh_wdata[0]  = SHDISP_PHAR_GP00;
    pharaoh_wdata[1]  = SHDISP_PHAR_GP01;
    pharaoh_wdata[2]  = SHDISP_PHAR_GP02;
    pharaoh_wdata[3]  = SHDISP_PHAR_GP03;
    pharaoh_wdata[4]  = SHDISP_PHAR_GP04;
    pharaoh_wdata[5]  = SHDISP_PHAR_GP05;
    pharaoh_wdata[6]  = SHDISP_PHAR_GP06;
    pharaoh_wdata[7]  = SHDISP_PHAR_GP07;
    pharaoh_wdata[8]  = SHDISP_PHAR_GP08;
    pharaoh_wdata[9]  = SHDISP_PHAR_GP09;
    pharaoh_wdata[10] = SHDISP_PHAR_GP10;
    pharaoh_wdata[11] = SHDISP_PHAR_GP11;
    pharaoh_wdata[12] = SHDISP_PHAR_GP12;
    shdisp_pharaoh_IO_write_reg(PHAR_REG_GAMMA_CONTROL_OF_SET_B_POSITIVE, pharaoh_wdata, 13);

    pharaoh_wdata[0]  = SHDISP_PHAR_GN00;
    pharaoh_wdata[1]  = SHDISP_PHAR_GN01;
    pharaoh_wdata[2]  = SHDISP_PHAR_GN02;
    pharaoh_wdata[3]  = SHDISP_PHAR_GN03;
    pharaoh_wdata[4]  = SHDISP_PHAR_GN04;
    pharaoh_wdata[5]  = SHDISP_PHAR_GN05;
    pharaoh_wdata[6]  = SHDISP_PHAR_GN06;
    pharaoh_wdata[7]  = SHDISP_PHAR_GN07;
    pharaoh_wdata[8]  = SHDISP_PHAR_GN08;
    pharaoh_wdata[9]  = SHDISP_PHAR_GN09;
    pharaoh_wdata[10] = SHDISP_PHAR_GN10;
    pharaoh_wdata[11] = SHDISP_PHAR_GN11;
    pharaoh_wdata[12] = SHDISP_PHAR_GN12;
    shdisp_pharaoh_IO_write_reg(PHAR_REG_GAMMA_CONTROL_OF_SET_B_NEGATIVE, pharaoh_wdata, 13);

    pharaoh_wdata[0]  = SHDISP_PHAR_BP00;
    pharaoh_wdata[1]  = SHDISP_PHAR_BP01;
    pharaoh_wdata[2]  = SHDISP_PHAR_BP02;
    pharaoh_wdata[3]  = SHDISP_PHAR_BP03;
    pharaoh_wdata[4]  = SHDISP_PHAR_BP04;
    pharaoh_wdata[5]  = SHDISP_PHAR_BP05;
    pharaoh_wdata[6]  = SHDISP_PHAR_BP06;
    pharaoh_wdata[7]  = SHDISP_PHAR_BP07;
    pharaoh_wdata[8]  = SHDISP_PHAR_BP08;
    pharaoh_wdata[9]  = SHDISP_PHAR_BP09;
    pharaoh_wdata[10] = SHDISP_PHAR_BP10;
    pharaoh_wdata[11] = SHDISP_PHAR_BP11;
    pharaoh_wdata[12] = SHDISP_PHAR_BP12;
    shdisp_pharaoh_IO_write_reg(PHAR_REG_GAMMA_CONTROL_OF_SET_C_POSITIVE, pharaoh_wdata, 13);

    pharaoh_wdata[0]  = SHDISP_PHAR_BN00;
    pharaoh_wdata[1]  = SHDISP_PHAR_BN01;
    pharaoh_wdata[2]  = SHDISP_PHAR_BN02;
    pharaoh_wdata[3]  = SHDISP_PHAR_BN03;
    pharaoh_wdata[4]  = SHDISP_PHAR_BN04;
    pharaoh_wdata[5]  = SHDISP_PHAR_BN05;
    pharaoh_wdata[6]  = SHDISP_PHAR_BN06;
    pharaoh_wdata[7]  = SHDISP_PHAR_BN07;
    pharaoh_wdata[8]  = SHDISP_PHAR_BN08;
    pharaoh_wdata[9]  = SHDISP_PHAR_BN09;
    pharaoh_wdata[10] = SHDISP_PHAR_BN10;
    pharaoh_wdata[11] = SHDISP_PHAR_BN11;
    pharaoh_wdata[12] = SHDISP_PHAR_BN12;
    shdisp_pharaoh_IO_write_reg(PHAR_REG_GAMMA_CONTROL_OF_SET_C_NEGATIVE, pharaoh_wdata, 13);

    pharaoh_wdata[0]  = SHDISP_PHAR_SETVGH;
    pharaoh_wdata[1]  = SHDISP_PHAR_SETVGL;
    pharaoh_wdata[2]  = SHDISP_PHAR_SETVCLI;
    pharaoh_wdata[3]  = 0x18;
    pharaoh_wdata[4]  = 0x53;
    pharaoh_wdata[5]  = 0x00;
    pharaoh_wdata[6]  = 0x13;
    shdisp_pharaoh_IO_write_reg(PHAR_REG_POWER_SETTING_1, pharaoh_wdata, 7);

    pharaoh_wdata[0]  = SHDISP_PHAR_SETVSPN;
    pharaoh_wdata[1]  = 0xD4;
    shdisp_pharaoh_IO_write_reg(PHAR_REG_POWER_SETTING_2, pharaoh_wdata, 2);

    pharaoh_wdata[0]  = 0x33;
    shdisp_pharaoh_IO_write_reg(PHAR_REG_POWER_SETTING_FOR_INTERNAL, pharaoh_wdata, 1);

    pharaoh_wdata[0]  = SHDISP_PHAR_SETPVH;
    pharaoh_wdata[1]  = SHDISP_PHAR_SETLVH;
    shdisp_pharaoh_IO_write_reg(PHAR_REG_VPLVL_VNLVL_SETTING, pharaoh_wdata, 2);

    pharaoh_wdata[0]  = 0x00;
    pharaoh_wdata[1]  = 0x00;
    pharaoh_wdata[2]  = (unsigned char) (((s_param_str.vcom_alpha >> 8) & 0x01) | 0x30);
    pharaoh_wdata[3]  = (unsigned char) s_param_str.vcom_alpha;
    pharaoh_wdata[4]  = 0x00;
    pharaoh_wdata[5]  = 0x00;
    pharaoh_wdata[6]  = 0x00;
    pharaoh_wdata[7]  = 0x00;
    pharaoh_wdata[8]  = 0x00;
    pharaoh_wdata[9]  = 0x00;
    pharaoh_wdata[10] = 0x00;
    shdisp_pharaoh_IO_write_reg(PHAR_REG_VCOMDC_SETTING, pharaoh_wdata, 11);

    return SHDISP_RESULT_SUCCESS;
}


/* ------------------------------------------------------------------------- */
/* shdisp_pharaoh_API_disp_on                                                */
/* ------------------------------------------------------------------------- */
static int shdisp_pharaoh_API_disp_on(void)
{
    shdisp_pharaoh_IO_write_reg(PHAR_REG_EXIT_SLEEP_MODE, pharaoh_wdata, 0);
    shdisp_SYS_delay_us(SHDISP_PHARAOH_1V_WAIT*3);
    shdisp_pharaoh_IO_write_reg(PHAR_REG_SET_DISPLAY_ON, pharaoh_wdata, 0);

    return SHDISP_RESULT_SUCCESS;
}


/* ------------------------------------------------------------------------- */
/* shdisp_pharaoh_API_sleep                                                  */
/* ------------------------------------------------------------------------- */
static int shdisp_pharaoh_API_sleep(void)
{
    shdisp_pharaoh_IO_write_reg(PHAR_REG_ENTER_SLEEP_MODE, pharaoh_wdata, 0);
    shdisp_SYS_delay_us(80000);
    shdisp_SYS_Host_control(SHDISP_HOST_CTL_CMD_LCD_CLK_STOP, SHDISP_PHARAOH_CLOCK);

    return SHDISP_RESULT_SUCCESS;
}


/* ------------------------------------------------------------------------- */
/* shdisp_pharaoh_API_deep_standby                                           */
/* ------------------------------------------------------------------------- */
static int shdisp_pharaoh_API_deep_standby(void)
{
    unsigned char data[2];
    
    data[0]  = 0x01;
    shdisp_pharaoh_IO_write_reg(PHAR_REG_LOW_POWER_MODE_CONTROL, data, 1);
    shdisp_SYS_delay_us(1000);

    return SHDISP_RESULT_SUCCESS;
}


/* ------------------------------------------------------------------------- */
/* shdisp_pharaoh_API_power_off                                              */
/* ------------------------------------------------------------------------- */
static int shdisp_pharaoh_API_power_off(void)
{
    shdisp_bdic_API_LCD_power_off();
    shdisp_SYS_delay_us(2000);
    shdisp_bdic_API_LCD_m_power_off();
    
    return SHDISP_RESULT_SUCCESS;
}


/* ------------------------------------------------------------------------- */
/* shdisp_pharaoh_API_check_upper_unit                                       */
/* ------------------------------------------------------------------------- */
static int shdisp_pharaoh_API_check_upper_unit(void)
{
#ifndef SHDISP_SW_DISABLE_CHK_UPPER_UNIT
    int i;
    unsigned char chk_data[4];
    
    chk_data[0] = 0x01;
    chk_data[1] = 0x22;
    chk_data[2] = 0x33;
    chk_data[3] = 0x06;
    
    for (i=0; i<4; i++) {
        pharaoh_rdata[i] = 0;
    }
    
    shdisp_pharaoh_IO_read_reg(PHAR_REG_DEVICE_CODE_READ, pharaoh_rdata, 4);
    
    for (i=0; i<4; i++) {
        if (chk_data[i] != pharaoh_rdata[i]) {
            SHDISP_ERR("<OTHER> Upper unit does not exist.\n");
            return SHDISP_RESULT_FAILURE;
        }
    }
#endif  /* SHDISP_SW_DISABLE_CHK_UPPER_UNIT */
    return SHDISP_RESULT_SUCCESS;
}


/* ------------------------------------------------------------------------- */
/* shdisp_pharaoh_API_check_flicker_param                                    */
/* ------------------------------------------------------------------------- */
static int shdisp_pharaoh_API_check_flicker_param(unsigned short alpha_in, unsigned short *alpha_out)
{
    unsigned short tmp_alpha = alpha_in;
    
    if (alpha_out == NULL){
        SHDISP_ERR("<NULL_POINTER> alpha_out.\n");
        return SHDISP_RESULT_FAILURE;
    }
    
    if ((tmp_alpha & 0xF000) != 0x9000) {
        *alpha_out = SHDISP_PHARAOH_ALPHA_DEFAULT;
        return SHDISP_RESULT_SUCCESS;
    }
    
    tmp_alpha = tmp_alpha & 0x01FF;
    if (tmp_alpha > SHDISP_PHARAOH_VCOM_MAX) {
        *alpha_out = SHDISP_PHARAOH_ALPHA_DEFAULT;
        return SHDISP_RESULT_SUCCESS;
    }
    
    *alpha_out = tmp_alpha;
    
    return SHDISP_RESULT_SUCCESS;
}


/* ------------------------------------------------------------------------- */
/* shdisp_pharaoh_API_diag_write_reg                                         */
/* ------------------------------------------------------------------------- */
static int shdisp_pharaoh_API_diag_write_reg(unsigned char addr, unsigned char *write_data, unsigned char size)
{
    int i;
    
    if (write_data == NULL){
        SHDISP_ERR("<NULL_POINTER> write_data.\n");
        return SHDISP_RESULT_FAILURE;
    }
    
    if (size < PHAR_NUM_SIO_CONTROL_MIN_WRITE || size > PHAR_NUM_SIO_CONTROL_MAX_WRITE){
        SHDISP_ERR("<INVALID_VALUE> size(%d).\n", size);
        return SHDISP_RESULT_FAILURE;
    }
    
    for (i = 0; i< size; i++) {
        pharaoh_wdata[i] = write_data[i];
    }
    
    shdisp_pharaoh_IO_write_reg(addr, pharaoh_wdata, size);
    
    return SHDISP_RESULT_SUCCESS;
}


/* ------------------------------------------------------------------------- */
/* shdisp_pharaoh_API_diag_read_reg                                          */
/* ------------------------------------------------------------------------- */
static int shdisp_pharaoh_API_diag_read_reg(unsigned char addr, unsigned char *read_data, unsigned char size)
{
    if (read_data == NULL){
        SHDISP_ERR("<NULL_POINTER> read_data.\n");
        return SHDISP_RESULT_FAILURE;
    }
    
    if (size < PHAR_NUM_SIO_CONTROL_MIN_READ || size > PHAR_NUM_SIO_CONTROL_MAX_READ){
        SHDISP_ERR("<INVALID_VALUE> size(%d).\n", size);
        return SHDISP_RESULT_FAILURE;
    }
    
    shdisp_pharaoh_IO_read_reg(addr, read_data, size);
    
    return SHDISP_RESULT_SUCCESS;
}


/* ------------------------------------------------------------------------- */
/* shdisp_pharaoh_API_diag_set_flicker_param                                 */
/* ------------------------------------------------------------------------- */
static int shdisp_pharaoh_API_diag_set_flicker_param(unsigned short alpha)
{
    int i;
    
    if (alpha > SHDISP_PHARAOH_VCOM_MAX) {
        SHDISP_ERR("<INVALID_VALUE> alpha(0x%04X).\n", alpha);
        return SHDISP_RESULT_FAILURE;
    }
    
    for (i=1; i<=4; i++) {
        pharaoh_rdata[i] = 0;
    }
    
    shdisp_pharaoh_IO_read_reg(PHAR_REG_VCOMDC_SETTING, pharaoh_rdata, 4);
    
    pharaoh_wdata[0]  = pharaoh_rdata[0];
    pharaoh_wdata[1]  = pharaoh_rdata[1];
    pharaoh_wdata[2]  = (unsigned char) ((alpha >> 8) & 0x01) | (pharaoh_rdata[2] & ~0x01);
    pharaoh_wdata[3]  = (unsigned char) alpha;
    
    shdisp_pharaoh_IO_write_reg(PHAR_REG_VCOMDC_SETTING, pharaoh_wdata, 4);
    
    s_param_str.vcom_alpha = alpha;
    
    return SHDISP_RESULT_SUCCESS;
}


/* ------------------------------------------------------------------------- */
/* shdisp_pharaoh_API_diag_get_flicker_param                                 */
/* ------------------------------------------------------------------------- */
static int shdisp_pharaoh_API_diag_get_flicker_param(unsigned short *alpha)
{
    int i;
    
    if (alpha == NULL){
        SHDISP_ERR("<NULL_POINTER> alpha.\n");
        return SHDISP_RESULT_FAILURE;
    }
    
    for (i=1; i<=4; i++) {
        pharaoh_rdata[i] = 0;
    }
    
    shdisp_pharaoh_IO_read_reg(PHAR_REG_VCOMDC_SETTING, pharaoh_rdata, 4);
    
    *alpha = ((pharaoh_rdata[2] & 0x01) << 8) | pharaoh_rdata[3];
    
    return SHDISP_RESULT_SUCCESS;
}


/* ------------------------------------------------------------------------- */
/* shdisp_pharaoh_API_check_recovery                                         */
/* ------------------------------------------------------------------------- */
static int shdisp_pharaoh_API_check_recovery(void)
{
    int i;
    
    for (i=0; i<3; i++) {
        pharaoh_rdata[i] = 0xFF;
    }
    
    shdisp_pharaoh_IO_read_reg(PHAR_REG_READ_CHECKSUM_AND_ECC_ERR_COUNT, pharaoh_rdata, 3);
    
    for (i=0; i<3; i++) {
        if (pharaoh_rdata[i] != 0x00) {
            SHDISP_ERR("<OTHER> detect error(b5).\n");
            return SHDISP_RESULT_FAILURE;
        }
    }
    
    pharaoh_rdata[0] = 0;
    
    shdisp_pharaoh_IO_read_reg(PHAR_REG_DEVICE_CODE_READ, pharaoh_rdata, 1);
    
    if (pharaoh_rdata[0] != 0x01) {
        SHDISP_ERR("<OTHER> detect error(bf).\n");
        return SHDISP_RESULT_FAILURE;
    }
    
    return SHDISP_RESULT_SUCCESS;
}


/* ------------------------------------------------------------------------- */
/* shdisp_pharaoh_API_recovery_type                                          */
/* ------------------------------------------------------------------------- */
static int shdisp_pharaoh_API_recovery_type(int *type)
{
    *type = SHDISP_SUBSCRIBE_TYPE_POL;
    
    return SHDISP_RESULT_SUCCESS;
}


/* ------------------------------------------------------------------------- */
/* shdisp_pharaoh_API_set_abl_lut                                            */
/* ------------------------------------------------------------------------- */
static int shdisp_pharaoh_API_set_abl_lut(struct dma_abl_lut_gamma *abl_lut_gammma)
{
    return SHDISP_RESULT_SUCCESS;
}


/* ------------------------------------------------------------------------- */
/* shdisp_pharaoh_API_disp_update                                            */
/* ------------------------------------------------------------------------- */
static int shdisp_pharaoh_API_disp_update(struct shdisp_main_update *update)
{
    return SHDISP_RESULT_SUCCESS;
}


/* ------------------------------------------------------------------------- */
/* shdisp_pharaoh_API_disp_clear_screen                                      */
/* ------------------------------------------------------------------------- */
static int shdisp_pharaoh_API_disp_clear_screen(struct shdisp_main_clear *clear)
{
    return SHDISP_RESULT_SUCCESS;
}


/* ------------------------------------------------------------------------- */
/* shdisp_pharaoh_IO_write_reg                                               */
/* ------------------------------------------------------------------------- */

static int shdisp_pharaoh_IO_write_reg(unsigned char reg, unsigned char *list, unsigned char size)
{
    shdisp_SYS_pharaoh_sio_transfer((unsigned short)reg, (unsigned char *)list, (int)size, NULL, 0);
    
    return SHDISP_RESULT_SUCCESS;
}


/* ------------------------------------------------------------------------- */
/* shdisp_pharaoh_IO_read_reg                                                */
/* ------------------------------------------------------------------------- */

static int shdisp_pharaoh_IO_read_reg(unsigned char reg, unsigned char *list, unsigned char size)
{
    shdisp_SYS_pharaoh_sio_transfer(0xF5, NULL, 0, NULL, 0);
    
    shdisp_SYS_pharaoh_sio_transfer((unsigned short)reg, NULL, 0, (unsigned char *)list, (int)size);
    
    shdisp_SYS_pharaoh_sio_transfer(0xF6, NULL, 0, NULL, 0);
    
    return SHDISP_RESULT_SUCCESS;
}


MODULE_DESCRIPTION("SHARP DISPLAY DRIVER MODULE");
MODULE_LICENSE("GPL v2");
MODULE_AUTHOR("SHARP CORPORATION");
MODULE_VERSION("1.00");

/* ------------------------------------------------------------------------- */
/* END OF FILE                                                               */
/* ------------------------------------------------------------------------- */
