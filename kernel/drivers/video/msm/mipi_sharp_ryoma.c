/* drivers/video/msm/mipi_sharp_ryoma.c  (Display Driver)
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

#include "msm_fb.h"
#include "mipi_dsi.h"
#include "mdp.h"
#include "mdp4.h"
#include "mipi_sharp.h"
#include <sharp/shdisp_kerl.h>
#include "mipi_sharp_ryoma.h"

#include <linux/gpio.h>
#include <linux/mfd/pm8xxx/pm8921.h>
#include <mach/irqs.h>

#if defined(CONFIG_MACH_DECKARD_GP4)
    #include "../../sharp/shdisp/data/shdisp_cabc_ryoma_data_gp4.h"
#else
    #include "../../sharp/shdisp/data/shdisp_cabc_ryoma_data_default.h"
#endif


/* ------------------------------------------------------------------------- */
/* DEBUG MACRAOS                                                             */
/* ------------------------------------------------------------------------- */

#define MIPI_SHARP_RW_MAX_SIZE          47
#define SHDISP_RYOMA_VCOM_MIN           0x10
#define SHDISP_LCDDR_GAMMA_OFFSET       12
#define SHDISP_LCDDR_GAMMA_STATUS_OK    0x96
#define SHDISP_LCDDR_GAMMA_STATUS_OK_2  0x98

#define SHDISP_RYOMA_1V_WAIT            18000

static struct dsi_buf sharp_tx_buf;
static struct dsi_buf sharp_rx_buf;
static int ch_used[3];
static int lcd_deviceid_out = 0;
static char setting_val_gamma_a[25];
static char setting_val_gamma_b[25];
static char setting_val_gamma_c[25];
static char setting_val_cabc_lut1_gamma_a[CABC_LUT_TABLE_SIZE];
static char setting_val_cabc_lut1_gamma_b[CABC_LUT_TABLE_SIZE];
static char setting_val_cabc_lut1_gamma_c[CABC_LUT_TABLE_SIZE];
static char setting_val_cabc_lut2_gamma_a[CABC_LUT_TABLE_SIZE];
static char setting_val_cabc_lut2_gamma_b[CABC_LUT_TABLE_SIZE];
static char setting_val_cabc_lut2_gamma_c[CABC_LUT_TABLE_SIZE];
static char setting_val_cabc_lut3_gamma_a[CABC_LUT_TABLE_SIZE];
static char setting_val_cabc_lut3_gamma_b[CABC_LUT_TABLE_SIZE];
static char setting_val_cabc_lut3_gamma_c[CABC_LUT_TABLE_SIZE];
static char setting_val_cabc_lut4_gamma_a[CABC_LUT_TABLE_SIZE];
static char setting_val_cabc_lut4_gamma_b[CABC_LUT_TABLE_SIZE];
static char setting_val_cabc_lut4_gamma_c[CABC_LUT_TABLE_SIZE];
static char setting_val_cabc_lut5_gamma_a[CABC_LUT_TABLE_SIZE];
static char setting_val_cabc_lut5_gamma_b[CABC_LUT_TABLE_SIZE];
static char setting_val_cabc_lut5_gamma_c[CABC_LUT_TABLE_SIZE];

static int mipi_sharp_force_60hz_move_flg = 0;
static int lcd_deviceid_read_flg = 0;
static int set_lpf_enable_flg = 0;

static char mipi_sharp_cmd_protect[2] = {0xB0, 0x00};
static char mipi_sharp_cmd_manufacture_id[2] = {0xBF, 0x00};
static char mipi_sharp_cmd_init1[7] = {0xB3, 
    0x0C,0xC0,0x00,0x00,0x00,0x00};
static char mipi_sharp_cmd_init2[3] = {0xB4, 0x0C,0x12};
static char mipi_sharp_cmd_init3[3] = {0xB6, 0x39,0xB3};
static char mipi_sharp_cmd_init4[2] = {0xB7, 0x00};
static char mipi_sharp_cmd_panel_pin[12] = {0xCB, 
    0x65,0x26,0xC0,0x19,0x0A,0x00,0x00,0x00,0x00,0xC0,0x00};
static char mipi_sharp_cmd_panel_if[2] = {0xCC, 0x00};
static char mipi_sharp_cmd_timing1[39] = {0xC1, 
    0x0C,0x60,0x40,0xAD,0x01,0x00,0x00,0x00,0x00,0x00,0x02,0xD8,
    0x06,0x02,0x09,0x0A,0x0B,0x0C,0x00,0x00,0x00,0x00,0x62,0x30,
    0x40,0xA5,0x0F,0x04,0x00,0x20,0x00,0x00,0x00,0x00,0x00,0x00,
    0x00,0x00};
static char mipi_sharp_cmd_timing2[8] = {0xC2, 
    0x30,0xF5,0x00,0x08,0x0C,0x00,0x00};
static char mipi_sharp_cmd_timing3[4] = {0xC3, 0x01,0x06,0x08};
static char mipi_sharp_cmd_timing4[14] = {0xC4, 
    0x70,0x00,0x00,0x00,0x00,0x0A,0x00,0x00,0x00,0x00,0x00,0x0A,
    0x00};
static char mipi_sharp_cmd_timing5[7] = {0xC6, 
    0x7C,0x69,0x69,0x7C,0x69,0x69};
static char mipi_sharp_cmd_gamma_a[25] = {0xC7, 
    0x0D,0x14,0x1A,0x21,0x2C,0x3E,0x2C,0x39,0x44,0x52,0x5C,0x64,
    0x0B,0x12,0x18,0x1F,0x2A,0x3E,0x2C,0x39,0x44,0x52,0x5C,0x64};
static char mipi_sharp_cmd_gamma_b[25] = {0xC8, 
    0x0D,0x14,0x1A,0x21,0x2C,0x3E,0x2C,0x39,0x44,0x52,0x5C,0x64,
    0x0B,0x12,0x18,0x1F,0x2A,0x3E,0x2C,0x39,0x44,0x52,0x5C,0x64};
static char mipi_sharp_cmd_gamma_c[25] = {0xC9, 
    0x0D,0x14,0x1A,0x21,0x2C,0x3E,0x2C,0x39,0x44,0x52,0x5C,0x64,
    0x0B,0x12,0x18,0x1F,0x2A,0x3E,0x2C,0x39,0x44,0x52,0x5C,0x64};
static char mipi_sharp_cmd_power1[17] = {0xD0, 
    0x00,0x10,0x19,0x18,0x99,0x98,0x18,0x00,0x88,0x01,0xBB,0x0C,
    0x8F,0x0E,0x21,0x20};
static char mipi_sharp_cmd_power15[2] = {0xD2, 0x9C};
static char mipi_sharp_cmd_power2[25] = {0xD3, 
    0x1B,0xB3,0xBB,0xBB,0x33,0x33,0x33,0x33,0x55,0x01,0x00,0xA0,
    0xA8,0xA0,0x07,0xCF,0xB7,0x33,0xA2,0x73,0xCF,0x00,0x00,0x00};
static char mipi_sharp_cmd_vcom[9] = {0xD5, 
    0x06,0x00,0x00,0x2A,0x2A,0x2A,0x2A,0x2A};
static char mipi_sharp_cmd_sleepout[2] = {0xD6, 0x01};
static char mipi_sharp_cmd_power3[48] = {0xD7, 
    0x44,0x01,0xFF,0xFF,0x3F,0xFC,0x51,0x9D,0x71,0xF0,0x0F,0x00,
    0xE0,0xFF,0x01,0xF0,0x03,0x00,0x1E,0x00,0x08,0x94,0x40,0xF0,
    0x73,0x7f,0x78,0x08,0xC0,0x07,0x0A,0x55,0x15,0x28,0x54,0x55,
    0xA0,0xF0,0xFF,0x00,0x40,0x55,0x05,0x00,0x20,0x20,0x01};
static char mipi_sharp_cmd_power4[17] = {0xD9, 
    0x00,0x68,0x4F,0x07,0x00,0x10,0x00,0xC0,0x00,0x05,0x33,0x33,
    0x00,0xF0,0x33,0x33};
static char mipi_sharp_cmd_set_sync1[3] = {0xEC, 0x01,0x00};
static char mipi_sharp_cmd_set_sync2[6] = {0xED, 0x00,0x00,0x00,0x00,0x00};
static char mipi_sharp_cmd_set_sync3[2] = {0xEE, 0x40};
static char mipi_sharp_cmd_set_sync4[14] = {0xEF, 
    0x02,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,
    0x00};
static char mipi_sharp_cmd_exit_sleep[2] = {0x11, 0x00};
static char mipi_sharp_cmd_column_addr[5] = {0x2A, 0x00,0x00,0x03,0x1F};
static char mipi_sharp_cmd_page_addr[5] = {0x2B, 0x00,0x00,0x04,0xFF};
static char mipi_sharp_cmd_tear_on[2] = {0x35, 0x00};
static char mipi_sharp_cmd_display_on[2] = {0x29, 0x00};
static struct dsi_cmd_desc mipi_sharp_cmds_protect[] = {
    {DTYPE_GEN_WRITE2, 1, 0, 0, 0, sizeof(mipi_sharp_cmd_protect), mipi_sharp_cmd_protect}
};
static struct dsi_cmd_desc mipi_sharp_cmds_manufacture_id[] = {
    {DTYPE_GEN_READ1, 1, 0, 1, 0, sizeof(mipi_sharp_cmd_manufacture_id), mipi_sharp_cmd_manufacture_id}
};
static struct dsi_cmd_desc mipi_sharp_cmds_init[] = {
    {DTYPE_GEN_LWRITE, 1, 0, 0, 0, sizeof(mipi_sharp_cmd_init1), mipi_sharp_cmd_init1},
    {DTYPE_GEN_LWRITE, 1, 0, 0, 0, sizeof(mipi_sharp_cmd_init2), mipi_sharp_cmd_init2},
    {DTYPE_GEN_LWRITE, 1, 0, 0, 0, sizeof(mipi_sharp_cmd_init3), mipi_sharp_cmd_init3},
    {DTYPE_GEN_WRITE2, 1, 0, 0, 0, sizeof(mipi_sharp_cmd_init4), mipi_sharp_cmd_init4},
    {DTYPE_GEN_LWRITE, 1, 0, 0, 0, sizeof(mipi_sharp_cmd_panel_pin), mipi_sharp_cmd_panel_pin},
    {DTYPE_GEN_WRITE2, 1, 0, 0, 0, sizeof(mipi_sharp_cmd_panel_if), mipi_sharp_cmd_panel_if}
};
static struct dsi_cmd_desc mipi_sharp_cmds_timing1[] = {
    {DTYPE_GEN_LWRITE, 1, 0, 0, 0, sizeof(mipi_sharp_cmd_timing1), mipi_sharp_cmd_timing1},
    {DTYPE_GEN_LWRITE, 1, 0, 0, 0, sizeof(mipi_sharp_cmd_timing2), mipi_sharp_cmd_timing2},
    {DTYPE_GEN_LWRITE, 1, 0, 0, 0, sizeof(mipi_sharp_cmd_timing3), mipi_sharp_cmd_timing3},
    {DTYPE_GEN_LWRITE, 1, 0, 0, 0, sizeof(mipi_sharp_cmd_timing4), mipi_sharp_cmd_timing4},
    {DTYPE_GEN_LWRITE, 1, 0, 0, 0, sizeof(mipi_sharp_cmd_timing5), mipi_sharp_cmd_timing5}
};
static struct dsi_cmd_desc mipi_sharp_cmds_gamma[] = {
    {DTYPE_GEN_LWRITE, 1, 0, 0, 0, sizeof(setting_val_gamma_a), setting_val_gamma_a},
    {DTYPE_GEN_LWRITE, 1, 0, 0, 0, sizeof(setting_val_gamma_b), setting_val_gamma_b},
    {DTYPE_GEN_LWRITE, 1, 0, 0, 0, sizeof(setting_val_gamma_c), setting_val_gamma_c}
};
static struct dsi_cmd_desc mipi_sharp_cmds_power1[] = {
    {DTYPE_GEN_LWRITE, 1, 0, 0, 0, sizeof(mipi_sharp_cmd_power1), mipi_sharp_cmd_power1},
    {DTYPE_GEN_WRITE2, 1, 0, 0, 0, sizeof(mipi_sharp_cmd_power15), mipi_sharp_cmd_power15},
    {DTYPE_GEN_LWRITE, 1, 0, 0, 0, sizeof(mipi_sharp_cmd_power2), mipi_sharp_cmd_power2}
};
static struct dsi_cmd_desc mipi_sharp_cmds_power2[] = {
    {DTYPE_GEN_WRITE2, 1, 0, 0, 0, sizeof(mipi_sharp_cmd_sleepout), mipi_sharp_cmd_sleepout},
    {DTYPE_GEN_LWRITE, 1, 0, 0, 0, sizeof(mipi_sharp_cmd_power3), mipi_sharp_cmd_power3},
    {DTYPE_GEN_LWRITE, 1, 0, 0, 0, sizeof(mipi_sharp_cmd_power4), mipi_sharp_cmd_power4}
};
static struct dsi_cmd_desc mipi_sharp_cmds_sync[] = {
    {DTYPE_GEN_LWRITE, 1, 0, 0, 0, sizeof(mipi_sharp_cmd_set_sync1), mipi_sharp_cmd_set_sync1},
    {DTYPE_GEN_LWRITE, 1, 0, 0, 0, sizeof(mipi_sharp_cmd_set_sync2), mipi_sharp_cmd_set_sync2},
    {DTYPE_GEN_WRITE2, 1, 0, 0, 0, sizeof(mipi_sharp_cmd_set_sync3), mipi_sharp_cmd_set_sync3},
    {DTYPE_GEN_LWRITE, 1, 0, 0, 0, sizeof(mipi_sharp_cmd_set_sync4), mipi_sharp_cmd_set_sync4}
};
static struct dsi_cmd_desc mipi_sharp_cmds_display_on[] = {
    {DTYPE_DCS_LWRITE, 1, 0, 0, 0, sizeof(mipi_sharp_cmd_column_addr), mipi_sharp_cmd_column_addr},
    {DTYPE_DCS_LWRITE, 1, 0, 0, 0, sizeof(mipi_sharp_cmd_page_addr), mipi_sharp_cmd_page_addr},
    {DTYPE_DCS_WRITE1, 1, 0, 0, 0, sizeof(mipi_sharp_cmd_tear_on), mipi_sharp_cmd_tear_on},
    {DTYPE_DCS_WRITE,  1, 0, 0, 0, sizeof(mipi_sharp_cmd_display_on), mipi_sharp_cmd_display_on}
};
static struct dsi_cmd_desc mipi_sharp_cmds_display_on_start[] = {
    {DTYPE_DCS_WRITE, 1, 0, 0, 90, sizeof(mipi_sharp_cmd_exit_sleep), mipi_sharp_cmd_exit_sleep}
};

static char mipi_sharp_cmd_low_power[2] = {0xD9, 0x00};
static char mipi_sharp_cmd_disp_off[2] = {0x28, 0x00};
static char mipi_sharp_cmd_vcom_off[9] = {0xD5, 
    0x06,0x00,0x00,0x11,0x11,0x11,0x11,0x11};
static char mipi_sharp_cmd_sleep_on[2] = {0x10, 0x00};
static char mipi_sharp_cmd_standby[2] = {0xB1, 0x01};
static struct dsi_cmd_desc mipi_sharp_cmds_display_off[] = {
    {DTYPE_GEN_LWRITE, 1, 0, 0, 18, sizeof(mipi_sharp_cmd_low_power), mipi_sharp_cmd_low_power},
    {DTYPE_DCS_WRITE, 1, 0, 0, 18, sizeof(mipi_sharp_cmd_disp_off), mipi_sharp_cmd_disp_off}
};
static struct dsi_cmd_desc mipi_sharp_cmds_display_off2[] = {
    {DTYPE_DCS_WRITE, 1, 0, 0, 108, sizeof(mipi_sharp_cmd_sleep_on), mipi_sharp_cmd_sleep_on},
    {DTYPE_GEN_WRITE2, 1, 0, 0, 18, sizeof(mipi_sharp_cmd_standby), mipi_sharp_cmd_standby}
};


static struct dsi_cmd_desc mipi_sharp_cmds_cabc_init_set_pwm[] = {
    {DTYPE_GEN_LWRITE, 1, 0, 0, 0, sizeof(mipi_sharp_cmd_cabc_ryoma_init_set_pwm), mipi_sharp_cmd_cabc_ryoma_init_set_pwm}
};
static struct dsi_cmd_desc mipi_sharp_cmds_cabc_init_set_para[] = {
    {DTYPE_DCS_LWRITE, 1, 0, 0, 0, sizeof(mipi_sharp_cmd_cabc_ryoma_init_set_param), mipi_sharp_cmd_cabc_ryoma_init_set_param}
};
static struct dsi_cmd_desc mipi_sharp_cmds_cabc_init_set_flicker[] = {
    {DTYPE_GEN_LWRITE, 1, 0, 0, 0, sizeof(mipi_sharp_cmd_cabc_ryoma_init_set_flicker), mipi_sharp_cmd_cabc_ryoma_init_set_flicker}
};
static struct dsi_cmd_desc mipi_sharp_cmds_cabc_init_set_connector[] = {
    {DTYPE_DCS_WRITE1, 1, 0, 0, 0, sizeof(mipi_sharp_cmd_cabc_ryoma_init_set_connector), mipi_sharp_cmd_cabc_ryoma_init_set_connector}
};

static struct dsi_cmd_desc mipi_sharp_cmds_cabc_indoor_set_image_exrate_lut0[] = {
    {DTYPE_GEN_LWRITE, 1, 0, 0, 0, sizeof(mipi_sharp_cmd_cabc_ryoma_indoor_set_image_exrate_lut0), mipi_sharp_cmd_cabc_ryoma_indoor_set_image_exrate_lut0}
};
static struct dsi_cmd_desc mipi_sharp_cmds_cabc_indoor_set_flicker[] = {
    {DTYPE_GEN_LWRITE, 1, 0, 0, 0, sizeof(mipi_sharp_cmd_cabc_ryoma_indoor_set_flicker), mipi_sharp_cmd_cabc_ryoma_indoor_set_flicker}
};
static struct dsi_cmd_desc mipi_sharp_cmds_cabc_indoor_set_on[] = {
    {DTYPE_DCS_WRITE1, 1, 0, 0, 0, sizeof(mipi_sharp_cmd_cabc_ryoma_indoor_set_on), mipi_sharp_cmd_cabc_ryoma_indoor_set_on}
};

static struct dsi_cmd_desc mipi_sharp_cmds_cabc_outdoor_set_image_exrate_lut1[] = {
    {DTYPE_GEN_LWRITE, 1, 0, 0, 0, sizeof(mipi_sharp_cmd_cabc_ryoma_outdoor_set_image_exrate_lut1), mipi_sharp_cmd_cabc_ryoma_outdoor_set_image_exrate_lut1}
};
static struct dsi_cmd_desc mipi_sharp_cmds_cabc_outdoor_set_image_exrate_lut2[] = {
    {DTYPE_GEN_LWRITE, 1, 0, 0, 0, sizeof(mipi_sharp_cmd_cabc_ryoma_outdoor_set_image_exrate_lut2), mipi_sharp_cmd_cabc_ryoma_outdoor_set_image_exrate_lut2}
};
static struct dsi_cmd_desc mipi_sharp_cmds_cabc_outdoor_set_image_exrate_lut3[] = {
    {DTYPE_GEN_LWRITE, 1, 0, 0, 0, sizeof(mipi_sharp_cmd_cabc_ryoma_outdoor_set_image_exrate_lut3), mipi_sharp_cmd_cabc_ryoma_outdoor_set_image_exrate_lut3}
};
static struct dsi_cmd_desc mipi_sharp_cmds_cabc_outdoor_set_image_exrate_lut4[] = {
    {DTYPE_GEN_LWRITE, 1, 0, 0, 0, sizeof(mipi_sharp_cmd_cabc_ryoma_outdoor_set_image_exrate_lut4), mipi_sharp_cmd_cabc_ryoma_outdoor_set_image_exrate_lut4}
};
static struct dsi_cmd_desc mipi_sharp_cmds_cabc_outdoor_set_image_exrate_lut5[] = {
    {DTYPE_GEN_LWRITE, 1, 0, 0, 0, sizeof(mipi_sharp_cmd_cabc_ryoma_outdoor_set_image_exrate_lut5), mipi_sharp_cmd_cabc_ryoma_outdoor_set_image_exrate_lut5}
};
static struct dsi_cmd_desc mipi_sharp_cmds_cabc_outdoor_set_on[] = {
    {DTYPE_DCS_WRITE1, 1, 0, 0, 0, sizeof(mipi_sharp_cmd_cabc_ryoma_outdoor_set_on), mipi_sharp_cmd_cabc_ryoma_outdoor_set_on}
};
static struct dsi_cmd_desc mipi_sharp_cmds_cabc_outdoor_set_flicker[] = {
    {DTYPE_GEN_LWRITE, 1, 0, 0, 0, sizeof(mipi_sharp_cmd_cabc_ryoma_outdoor_set_flicker), mipi_sharp_cmd_cabc_ryoma_outdoor_set_flicker}
};

static struct dsi_cmd_desc mipi_sharp_cmds_cabc_off_set_image_exrate_lut0[] = {
    {DTYPE_GEN_LWRITE, 1, 0, 0, 0, sizeof(mipi_sharp_cmd_cabc_ryoma_off_set_image_exrate_lut0), mipi_sharp_cmd_cabc_ryoma_off_set_image_exrate_lut0}
};
static struct dsi_cmd_desc mipi_sharp_cmds_cabc_off_set_off[] = {
    {DTYPE_DCS_WRITE1, 1, 0, 0, 0, sizeof(mipi_sharp_cmd_cabc_ryoma_off_set_off), mipi_sharp_cmd_cabc_ryoma_off_set_off}
};
static struct dsi_cmd_desc mipi_sharp_cmds_cabc_transition_set_image_exrate_lut1[] = {
    {DTYPE_GEN_LWRITE, 1, 0, 0, 0, sizeof(mipi_sharp_cmd_cabc_ryoma_transition_set_image_exrate_lut1), mipi_sharp_cmd_cabc_ryoma_transition_set_image_exrate_lut1}
};
static struct dsi_cmd_desc mipi_sharp_cmds_cabc_transition_set_image_exrate_lut2[] = {
    {DTYPE_GEN_LWRITE, 1, 0, 0, 0, sizeof(mipi_sharp_cmd_cabc_ryoma_transition_set_image_exrate_lut2), mipi_sharp_cmd_cabc_ryoma_transition_set_image_exrate_lut2}
};
static struct dsi_cmd_desc mipi_sharp_cmds_cabc_transition_set_image_exrate_lut3[] = {
    {DTYPE_GEN_LWRITE, 1, 0, 0, 0, sizeof(mipi_sharp_cmd_cabc_ryoma_transition_set_image_exrate_lut3), mipi_sharp_cmd_cabc_ryoma_transition_set_image_exrate_lut3}
};
static struct dsi_cmd_desc mipi_sharp_cmds_cabc_transition_set_image_exrate_lut4[] = {
    {DTYPE_GEN_LWRITE, 1, 0, 0, 0, sizeof(mipi_sharp_cmd_cabc_ryoma_transition_set_image_exrate_lut4), mipi_sharp_cmd_cabc_ryoma_transition_set_image_exrate_lut4}
};
static struct dsi_cmd_desc mipi_sharp_cmds_cabc_transition_set_image_exrate_lut5[] = {
    {DTYPE_GEN_LWRITE, 1, 0, 0, 0, sizeof(mipi_sharp_cmd_cabc_ryoma_transition_set_image_exrate_lut5), mipi_sharp_cmd_cabc_ryoma_transition_set_image_exrate_lut5}
};

static struct dsi_cmd_desc mipi_sharp_cmds_cabc_bank_lock[] = {
    {DTYPE_GEN_WRITE2, 1, 0, 0, 0, sizeof(mipi_sharp_cmd_cabc_bank_lock), mipi_sharp_cmd_cabc_bank_lock}
};
static struct dsi_cmd_desc mipi_sharp_cmds_cabc_bank_01[] = {
    {DTYPE_GEN_WRITE2, 1, 0, 0, 0, sizeof(mipi_sharp_cmd_cabc_bank_01), mipi_sharp_cmd_cabc_bank_01}
};
static struct dsi_cmd_desc mipi_sharp_cmds_cabc_trv_lock0[] = {
    {DTYPE_GEN_WRITE2, 1, 0, 0, 0, sizeof(mipi_sharp_cmd_cabc_trv_lock0), mipi_sharp_cmd_cabc_trv_lock0}
};
static struct dsi_cmd_desc mipi_sharp_cmds_cabc_trv_lock1[] = {
    {DTYPE_GEN_WRITE2, 1, 0, 0, 0, sizeof(mipi_sharp_cmd_cabc_trv_lock1), mipi_sharp_cmd_cabc_trv_lock1}
};
static struct dsi_cmd_desc mipi_sharp_cmds_cabc_bank_00[] = {
    {DTYPE_GEN_WRITE2, 1, 0, 0, 0, sizeof(mipi_sharp_cmd_cabc_bank_00), mipi_sharp_cmd_cabc_bank_00}
};
static struct dsi_cmd_desc mipi_sharp_cmds_cabc_fntn_s[] = {
    {DTYPE_GEN_WRITE2, 1, 0, 0, 0, sizeof(mipi_sharp_cmd_cabc_fntn_s), mipi_sharp_cmd_cabc_fntn_s}
};
static struct dsi_cmd_desc mipi_sharp_cmds_cabc_trv_mode_set[] = {
    {DTYPE_GEN_WRITE2, 1, 0, 0, 0, sizeof(mipi_sharp_cmd_cabc_trv_mode_set), mipi_sharp_cmd_cabc_trv_mode_set}
};
static struct dsi_cmd_desc mipi_sharp_cmds_cabc_val_go[] = {
    {DTYPE_GEN_WRITE2, 1, 0, 0, 0, sizeof(mipi_sharp_cmd_cabc_val_go), mipi_sharp_cmd_cabc_val_go}
};

static struct dsi_cmd_desc mipi_sharp_cmds_cabc_trv_on[] = {
    {DTYPE_GEN_WRITE2, 1, 0, 0, 0, sizeof(mipi_sharp_cmd_cabc_trv_on), mipi_sharp_cmd_cabc_trv_on}
};

static struct dsi_cmd_desc mipi_sharp_cmds_cabc_trv_off[] = {
    {DTYPE_GEN_WRITE2, 1, 0, 0, 0, sizeof(mipi_sharp_cmd_cabc_trv_off), mipi_sharp_cmd_cabc_trv_off}
};

static struct dsi_cmd_desc mipi_sharp_cmds_cabc_trv_lut_set_on[] = {
    {DTYPE_GEN_WRITE2, 1, 0, 0, 0, sizeof(mipi_sharp_cmd_cabc_trv_lut_set_on), mipi_sharp_cmd_cabc_trv_lut_set_on}
};
static struct dsi_cmd_desc mipi_sharp_cmds_cabc_bank_07[] = {
    {DTYPE_GEN_WRITE2, 1, 0, 0, 0, sizeof(mipi_sharp_cmd_cabc_bank_07), mipi_sharp_cmd_cabc_bank_07}
};
static struct dsi_cmd_desc mipi_sharp_cmds_cabc_bank_08[] = {
    {DTYPE_GEN_WRITE2, 1, 0, 0, 0, sizeof(mipi_sharp_cmd_cabc_bank_08), mipi_sharp_cmd_cabc_bank_08}
};
static struct dsi_cmd_desc mipi_sharp_cmds_cabc_bank_09[] = {
    {DTYPE_GEN_WRITE2, 1, 0, 0, 0, sizeof(mipi_sharp_cmd_cabc_bank_09), mipi_sharp_cmd_cabc_bank_09}
};
static struct dsi_cmd_desc mipi_sharp_cmds_cabc_bank_0A[] = {
    {DTYPE_GEN_WRITE2, 1, 0, 0, 0, sizeof(mipi_sharp_cmd_cabc_bank_0A), mipi_sharp_cmd_cabc_bank_0A}
};
static struct dsi_cmd_desc mipi_sharp_cmds_cabc_trv_lut_set_off[] = {
    {DTYPE_GEN_WRITE2, 1, 0, 0, 0, sizeof(mipi_sharp_cmd_cabc_trv_lut_set_off), mipi_sharp_cmd_cabc_trv_lut_set_off}
};
static struct dsi_cmd_desc mipi_sharp_cmds_cabc_pwmoff_unset_connector[] = {
    {DTYPE_DCS_WRITE1, 1, 0, 0, 0, sizeof(mipi_sharp_cmd_cabc_pwmoff_unset_connector), mipi_sharp_cmd_cabc_pwmoff_unset_connector}
};


static char mipi_sharp_clk_state[16] = {
    0x00,0x68,0x4F,0x07,0x00,0x00,0x00,0x80,0x00,0x76,0x33,0x33,
    0x00,0xF0,0x33,0x33};
static char mipi_sharp_panel_sync[2] = {0x01,0x00};


static int mipi_sharp_ryoma_lcd_on(struct platform_device *pdev);
static int mipi_sharp_ryoma_lcd_off(struct platform_device *pdev);
static void mipi_sharp_ryoma_lcd_set_backlight(struct msm_fb_data_type *mfd);

static int mipi_sharp_clk_write(struct msm_fb_data_type *mfd, int length);
static int mipi_sharp_panel_sync_write(struct msm_fb_data_type *mfd);
#if defined(CONFIG_SHDISP_USE_CABC)
static int mipi_sharp_cabc_lcd_backlight_on(void);
static int mipi_sharp_ryoma_cabc_pre_disp_on_ctrl(struct msm_fb_data_type *mfd, struct mipi_panel_info *mipi);
static int mipi_sharp_cabc_disp_on_ctrl(struct msm_fb_data_type *mfd, struct mipi_panel_info *mipi);
#endif

static int mipi_sharp_cmd_sqe_cabc_init(struct msm_fb_data_type *mfd, struct mipi_panel_info *mipi);
static int mipi_sharp_cmd_sqe_cabc_indoor_on(struct msm_fb_data_type *mfd, struct mipi_panel_info *mipi);
static int mipi_sharp_cmd_sqe_cabc_outdoor_on(struct msm_fb_data_type *mfd, struct mipi_panel_info *mipi, int lut_level);
static int mipi_sharp_cmd_sqe_cabc_off(struct msm_fb_data_type *mfd, struct mipi_panel_info *mipi, int wait_on, int pwm_disable);
static int mipi_sharp_cmd_sqe_cabc_outdoor_move(struct msm_fb_data_type *mfd, struct mipi_panel_info *mipi, int lut_level);

static int mipi_sharp_ryoma_cmd_sqe_cabc_trv_primary_setting(struct msm_fb_data_type *mfd);
static int mipi_sharp_ryoma_cmd_sqe_cabc_trv_lut_write(struct msm_fb_data_type *mfd, char addr, char val);
static int mipi_sharp_ryoma_cmd_sqe_cabc_trv_lut_setting(struct msm_fb_data_type *mfd, int lut_level);
static int mipi_sharp_ryoma_cmd_sqe_cabc_trv_on(struct msm_fb_data_type *mfd);
static int mipi_sharp_ryoma_cmd_sqe_cabc_trv_off(struct msm_fb_data_type *mfd);

static int mipi_sharp_ryoma_cabc_ctrl(struct platform_device *pdev);

static void mipi_sharp_cabc_acc_gamma_init(void);

#define PM8921_GPIO_BASE        NR_GPIO_IRQS
#define PM8921_GPIO_PM_TO_SYS(pm_gpio)  (pm_gpio - 1 + PM8921_GPIO_BASE)

static char mipi_sharp_manufacture_id(struct msm_fb_data_type *mfd)
{
    struct dsi_buf *rp, *tp;
    struct dsi_cmd_desc *cmd;
    char *rdata;
    
    tp = &sharp_tx_buf;
    rp = &sharp_rx_buf;
    
    cmd = &mipi_sharp_cmds_manufacture_id[0];
    mipi_dsi_cmds_rx(mfd, tp, rp, cmd, 6);
    rdata = rp->data;
    if (lcd_deviceid_out == 0) {
        pr_info("[SHDISP]%s: BFh=%02x,%02x,%02x,%02x,%02x,%02x",
            __func__, *rdata,*(rdata+1),*(rdata+2),*(rdata+3),*(rdata+4),*(rdata+5));
        lcd_deviceid_out = 1;
    }
    
    return *(rdata+4);
}

static int mipi_sharp_tx_vcom_write(struct msm_fb_data_type *mfd)
{
    struct dsi_cmd_desc cmd[1];
    char cmd_buf[13];
    unsigned short alpha;
    unsigned short alpha_low;
    
    alpha = shdisp_api_get_alpha();
    alpha_low = shdisp_api_get_alpha_low();
    
    if( (alpha == SHDISP_NOT_ADJUST_VCOM_VAL) || (alpha_low == SHDISP_NOT_ADJUST_VCOM_VAL) ) {
        memcpy(&cmd_buf[0], mipi_sharp_cmd_vcom, 4);
        cmd[0].dlen = 4;
    }
    else {
        memcpy(&cmd_buf[0], mipi_sharp_cmd_vcom, 9);
        cmd[0].dlen = 9;
        cmd_buf[4]  = (unsigned char) (alpha & 0xFF);
        cmd_buf[5]  = (unsigned char) (alpha & 0xFF);
        alpha = alpha_low;
        alpha = (alpha + 1) / 2;
        if(alpha < SHDISP_RYOMA_VCOM_MIN){
            alpha = SHDISP_RYOMA_VCOM_MIN;
        }
        cmd_buf[6]  = (unsigned char) (alpha & 0xFF);
        cmd_buf[7]  = (unsigned char) (alpha_low & 0xFF);
        cmd_buf[8]  = (unsigned char) (alpha_low & 0xFF);
    }
    
    cmd[0].dtype = DTYPE_GEN_LWRITE;
    cmd[0].last = 0x01;
    cmd[0].vc = 0x00;
    cmd[0].ack = 0x00;
    cmd[0].wait = 0x00;
    cmd[0].payload = cmd_buf;

    mipi_dsi_cmds_tx(&sharp_tx_buf, cmd, ARRAY_SIZE(cmd));

    return 0;
}

static int mipi_sharp_tx_vcom_off_write(struct msm_fb_data_type *mfd)
{
    struct dsi_cmd_desc cmd[1];
    char cmd_buf[13];
    int paramCnt;
    char data;
    unsigned char ryoma_rdata[9];
    int ret;
    
    ret = mipi_sharp_normal_read_reg(0xD5, ryoma_rdata, 8);
    if(ret) {
        pr_err("%s: mipi_sharp_diag_read_reg fail.\n", __func__);
        return ret;
    }
    memcpy(&cmd_buf[0], mipi_sharp_cmd_vcom_off, 4);
    for(paramCnt = 4; paramCnt < 9; paramCnt++){
        data = (ryoma_rdata[paramCnt - 1] + 1) / 2;
        if(data < SHDISP_RYOMA_VCOM_MIN){
            data = SHDISP_RYOMA_VCOM_MIN;
        }
        cmd_buf[paramCnt] = data;
    }
    
    cmd[0].dtype = DTYPE_GEN_LWRITE;
    cmd[0].last = 0x01;
    cmd[0].vc = 0x00;
    cmd[0].ack = 0x00;
    cmd[0].wait = 18;
    cmd[0].dlen = 9;
    cmd[0].payload = cmd_buf;
    
    mipi_dsi_cmds_tx(&sharp_tx_buf, cmd, ARRAY_SIZE(cmd));
    
    return 0;
}

static int mipi_sharp_cmd_lcd_on(struct msm_fb_data_type *mfd, struct mipi_panel_info *mipi)
{
    char device_id = 0x00;
    
    if(shdisp_api_get_upper_unit() != SHDISP_RESULT_SUCCESS) {
        return -ENODEV;
    }
    
    mipi_dsi_cmds_tx(&sharp_tx_buf, mipi_sharp_cmds_protect,
                     ARRAY_SIZE(mipi_sharp_cmds_protect));
    device_id = mipi_sharp_manufacture_id(mfd);
    mipi_dsi_cmds_tx(&sharp_tx_buf, mipi_sharp_cmds_init,
                     ARRAY_SIZE(mipi_sharp_cmds_init));
    mipi_dsi_cmds_tx(&sharp_tx_buf, mipi_sharp_cmds_timing1,
                     ARRAY_SIZE(mipi_sharp_cmds_timing1));
    mipi_dsi_cmds_tx(&sharp_tx_buf, mipi_sharp_cmds_gamma,
                     ARRAY_SIZE(mipi_sharp_cmds_gamma));
    mipi_dsi_cmds_tx(&sharp_tx_buf, mipi_sharp_cmds_power1,
                     ARRAY_SIZE(mipi_sharp_cmds_power1));
    mipi_sharp_tx_vcom_write(mfd);
    mipi_dsi_cmds_tx(&sharp_tx_buf, mipi_sharp_cmds_power2,
                     ARRAY_SIZE(mipi_sharp_cmds_power2));

    memcpy(&mipi_sharp_clk_state[0], &mipi_sharp_cmd_power4[1], 16);

    mipi_dsi_cmds_tx(&sharp_tx_buf, mipi_sharp_cmds_sync,
                     ARRAY_SIZE(mipi_sharp_cmds_sync));

    memcpy(&mipi_sharp_panel_sync[0], &mipi_sharp_cmd_set_sync1[1], 2);

    mipi_dsi_cmds_tx(&sharp_tx_buf, mipi_sharp_cmds_display_on,
                     ARRAY_SIZE(mipi_sharp_cmds_display_on));
    mipi_dsi_cmd_bta_sw_trigger();
    
#if defined(CONFIG_SHDISP_USE_CABC)
    mipi_sharp_ryoma_cabc_pre_disp_on_ctrl(mfd, mipi);
#endif
    return 0;
}

#if defined(CONFIG_SHDISP_USE_CABC) 
static int mipi_sharp_cabc_lcd_backlight_on(void)
{
	int rc;

	static struct pm_gpio pwm_mode = {
		.direction        = PM_GPIO_DIR_OUT,
		.output_buffer    = PM_GPIO_OUT_BUF_CMOS,
		.output_value     = 1,
		.pull             = PM_GPIO_PULL_NO,
		.vin_sel          = PM_GPIO_VIN_S4,
		.out_strength     = PM_GPIO_STRENGTH_HIGH,
		.function         = PM_GPIO_FUNC_NORMAL,
		.inv_int_pol      = 0,
		.disable_pin      = 0,
	};

	

	rc = gpio_request(PM8921_GPIO_PM_TO_SYS(24), "shspamp");
	if (rc) {
		pr_err("%s: Failed to request gpio\n", __func__);
		goto err_free_gpio;
	}

	rc = pm8xxx_gpio_config(PM8921_GPIO_PM_TO_SYS(24), &pwm_mode);
	if (rc) {
		pr_err("%s: pwm_mode failed\n", __func__);
		goto err_free_gpio;
	}
	gpio_direction_output(PM8921_GPIO_PM_TO_SYS(24), 1);
	
err_free_gpio:
    gpio_free(PM8921_GPIO_PM_TO_SYS(24));

	
	return 0;
}
#endif


static int mipi_sharp_cmd_lcd_off(struct msm_fb_data_type *mfd, struct mipi_panel_info *mipi)
{
    if(shdisp_api_get_upper_unit() != SHDISP_RESULT_SUCCESS) {
        return 0;
    }
    
    mipi_dsi_cmds_tx(&sharp_tx_buf, mipi_sharp_cmds_display_off,
                     ARRAY_SIZE(mipi_sharp_cmds_display_off));

    mipi_sharp_clk_state[0] = mipi_sharp_cmd_low_power[1];

    mipi_sharp_tx_vcom_off_write(mfd);
    mipi_dsi_cmds_tx(&sharp_tx_buf, mipi_sharp_cmds_display_off2,
                     ARRAY_SIZE(mipi_sharp_cmds_display_off2));

    return 0;
}

static int mipi_sharp_ryoma_lcd_on(struct platform_device *pdev)
{
    struct msm_fb_data_type *mfd;
    struct mipi_panel_info *mipi;
    int ret = 0;
    
    mfd = platform_get_drvdata(pdev);
    if (!mfd) {
        return -ENODEV;
    }
    if (mfd->key != MFD_KEY) {
        return -EINVAL;
    }
    
    mipi = &mfd->panel_info.mipi;
    if (mipi->mode == DSI_CMD_MODE) {
        ret = mipi_sharp_cmd_lcd_on(mfd, mipi);
    }
    return ret;
}

static int mipi_sharp_ryoma_lcd_off(struct platform_device *pdev)
{
    struct msm_fb_data_type *mfd;
    struct mipi_panel_info *mipi;
    int ret = 0;
    
    mfd = platform_get_drvdata(pdev);
    if (!mfd) {
        return -ENODEV;
    }
    if (mfd->key != MFD_KEY) {
        return -EINVAL;
    }
    
    mipi = &mfd->panel_info.mipi;
    if (mipi->mode == DSI_CMD_MODE) {
        ret = mipi_sharp_cmd_lcd_off(mfd, mipi);
    }
    shdisp_api_main_lcd_power_off();
    return 0;
}


static int mipi_sharp_cmd_lcd_display_on(struct msm_fb_data_type *mfd, struct mipi_panel_info *mipi)
{
	if(shdisp_api_get_upper_unit() != SHDISP_RESULT_SUCCESS) {
		return -ENODEV;
	}
	mipi_dsi_cmds_tx(&sharp_tx_buf, mipi_sharp_cmds_display_on_start,
						 ARRAY_SIZE(mipi_sharp_cmds_display_on_start));
	mipi_dsi_cmd_bta_sw_trigger();
	return 0;
}

static int mipi_sharp_ryoma_lcd_start_display(struct msm_fb_data_type *mfd)
{
	struct mipi_panel_info *mipi;
	int ret = 0;

	if (mfd->key != MFD_KEY) {
		return -EINVAL;
	}
	mipi = &mfd->panel_info.mipi;
	if (mipi->mode == DSI_CMD_MODE) {
		mutex_lock(&mfd->dma->ov_mutex);
		mipi_dsi_mdp_busy_wait();
		ret = mipi_sharp_cmd_lcd_display_on(mfd, mipi);
		mutex_unlock(&mfd->dma->ov_mutex);
	}
	return ret;
}

static void mipi_sharp_ryoma_lcd_set_backlight(struct msm_fb_data_type *mfd)
{
	struct shdisp_main_bkl_ctl bkl;

	if (mfd->bl_level == 0) {
		shdisp_api_main_bkl_off();
	}else{
		bkl.mode = SHDISP_MAIN_BKL_MODE_FIX;
		bkl.param = mfd->bl_level;
		shdisp_api_main_bkl_on(&bkl);
	}
}


#if defined(CONFIG_SHDISP_USE_CABC)
static int mipi_sharp_ryoma_cabc_pre_disp_on_ctrl(struct msm_fb_data_type *mfd, struct mipi_panel_info *mipi)
{
    mipi_sharp_cabc_lcd_backlight_on();

    mipi_sharp_cmd_sqe_cabc_off(mfd, mipi, 0, 1);

    shdisp_api_init_cabc_state(SHDISP_MAIN_DISP_CABC_MODE_OFF, SHDISP_MAIN_DISP_CABC_LUT0);

    return 0;
}
#endif


#if defined(CONFIG_SHDISP_USE_CABC)
static int mipi_sharp_cabc_disp_on_ctrl(struct msm_fb_data_type *mfd, struct mipi_panel_info *mipi)
{
    struct shdisp_check_cabc_val value;


    shdisp_api_photo_sensor_pow_ctl();

    shdisp_api_check_cabc(&value);

    if(value.change == 0)
    {
        return 0;
    }

    if(value.old_mode == SHDISP_MAIN_DISP_CABC_MODE_OFF && value.mode == SHDISP_MAIN_DISP_CABC_MODE_DBC )
    {
        mipi_sharp_cmd_sqe_cabc_init(mfd, mipi);

        mipi_sharp_cmd_sqe_cabc_indoor_on(mfd, mipi);
    }
    else if(value.old_mode == SHDISP_MAIN_DISP_CABC_MODE_OFF && value.mode == SHDISP_MAIN_DISP_CABC_MODE_ACC )
    {
        if(value.lut == SHDISP_MAIN_DISP_CABC_LUT0)
        {
        }
        else
        {
            mipi_sharp_cmd_sqe_cabc_init(mfd, mipi);


            mipi_sharp_cmd_sqe_cabc_outdoor_on(mfd, mipi, value.lut);
        }
    }
    else if(value.old_mode == SHDISP_MAIN_DISP_CABC_MODE_OFF && value.mode == SHDISP_MAIN_DISP_CABC_MODE_DBC_ACC )
    {
        if(value.lut == SHDISP_MAIN_DISP_CABC_LUT0)
        {

            mipi_sharp_cmd_sqe_cabc_init(mfd, mipi);

            mipi_sharp_cmd_sqe_cabc_indoor_on(mfd, mipi);
        }
        else
        {

            mipi_sharp_cmd_sqe_cabc_init(mfd, mipi);


            mipi_sharp_cmd_sqe_cabc_outdoor_on(mfd, mipi, value.lut);
        }
    }

    else if(value.mode == SHDISP_MAIN_DISP_CABC_MODE_OFF )
    {
        mipi_sharp_cmd_sqe_cabc_off(mfd, mipi, 0, 1);
    }

    else if(value.old_mode == SHDISP_MAIN_DISP_CABC_MODE_DBC && value.mode == SHDISP_MAIN_DISP_CABC_MODE_ACC )
    {
        if(value.lut == SHDISP_MAIN_DISP_CABC_LUT0)
        {

            mipi_sharp_cmd_sqe_cabc_off(mfd, mipi, 0, 1);
        }
        else
        {

#if defined(CONFIG_SHDISP_PANEL_RYOMA)
#else
            mipi_sharp_cmd_sqe_cabc_off(mfd, mipi, 1, 0);
#endif /* CONFIG_MACH_*** */


            mipi_sharp_cmd_sqe_cabc_outdoor_on(mfd, mipi, value.lut);
        }
    }
    else if(value.old_mode == SHDISP_MAIN_DISP_CABC_MODE_DBC && value.mode == SHDISP_MAIN_DISP_CABC_MODE_DBC_ACC )
    {
        if(value.lut == SHDISP_MAIN_DISP_CABC_LUT0)
        {
        }
        else
        {

#if defined(CONFIG_SHDISP_PANEL_RYOMA)
#else
            mipi_sharp_cmd_sqe_cabc_off(mfd, mipi, 1, 0);
#endif /* CONFIG_MACH_*** */


            mipi_sharp_cmd_sqe_cabc_outdoor_on(mfd, mipi, value.lut);
        }
    }

    else if(value.old_mode == SHDISP_MAIN_DISP_CABC_MODE_ACC && value.mode == SHDISP_MAIN_DISP_CABC_MODE_DBC )
    {
        if(value.old_lut == SHDISP_MAIN_DISP_CABC_LUT0)
        {

            mipi_sharp_cmd_sqe_cabc_init(mfd, mipi);

            mipi_sharp_cmd_sqe_cabc_indoor_on(mfd, mipi);
        }
        else
        {

            mipi_sharp_cmd_sqe_cabc_off(mfd, mipi, 1, 0);

            mipi_sharp_cmd_sqe_cabc_indoor_on(mfd, mipi);
        }
    }
    else if(value.old_mode == SHDISP_MAIN_DISP_CABC_MODE_ACC && value.mode == SHDISP_MAIN_DISP_CABC_MODE_ACC)
    {
        if(value.lut == SHDISP_MAIN_DISP_CABC_LUT0)
        {

            mipi_sharp_cmd_sqe_cabc_off(mfd, mipi, 0, 1);
        }
        else
        {
            if(value.old_lut == SHDISP_MAIN_DISP_CABC_LUT0)
            {

                mipi_sharp_cmd_sqe_cabc_init(mfd, mipi);


                mipi_sharp_cmd_sqe_cabc_outdoor_on(mfd, mipi, value.lut);
            }
            else
            {

                mipi_sharp_cmd_sqe_cabc_outdoor_move(mfd, mipi, value.lut);
            }
        }
    }
    else if(value.old_mode == SHDISP_MAIN_DISP_CABC_MODE_ACC && value.mode == SHDISP_MAIN_DISP_CABC_MODE_DBC_ACC)
    {
        if(value.old_lut == SHDISP_MAIN_DISP_CABC_LUT0)
        {
            if(value.lut == SHDISP_MAIN_DISP_CABC_LUT0)
            {

                mipi_sharp_cmd_sqe_cabc_init(mfd, mipi);

                mipi_sharp_cmd_sqe_cabc_indoor_on(mfd, mipi);
            }
            else
            {

                mipi_sharp_cmd_sqe_cabc_init(mfd, mipi);


                mipi_sharp_cmd_sqe_cabc_outdoor_on(mfd, mipi, value.lut);
            }
        }
        else
        {
            if(value.lut == SHDISP_MAIN_DISP_CABC_LUT0)
            {

                mipi_sharp_cmd_sqe_cabc_off(mfd, mipi, 1, 0);

                mipi_sharp_cmd_sqe_cabc_indoor_on(mfd, mipi);
            }
            else
            {

                mipi_sharp_cmd_sqe_cabc_outdoor_move(mfd, mipi, value.lut);
            }
        }
    }
    else if(value.old_mode == SHDISP_MAIN_DISP_CABC_MODE_DBC_ACC && value.mode == SHDISP_MAIN_DISP_CABC_MODE_DBC )
    {
        if(value.old_lut == SHDISP_MAIN_DISP_CABC_LUT0)
        {
        }
        else
        {

            mipi_sharp_cmd_sqe_cabc_off(mfd, mipi, 1, 0);

            mipi_sharp_cmd_sqe_cabc_indoor_on(mfd, mipi);
        }
    }
    else if(value.old_mode == SHDISP_MAIN_DISP_CABC_MODE_DBC_ACC && value.mode == SHDISP_MAIN_DISP_CABC_MODE_ACC )
    {
        if(value.lut == SHDISP_MAIN_DISP_CABC_LUT0)
        {

            mipi_sharp_cmd_sqe_cabc_off(mfd, mipi, 0, 1);
        }
        else
        {
            if(value.old_lut == SHDISP_MAIN_DISP_CABC_LUT0)
            {

#if defined(CONFIG_SHDISP_PANEL_RYOMA)
#else
                mipi_sharp_cmd_sqe_cabc_off(mfd, mipi, 1, 0);
#endif /* CONFIG_MACH_*** */


                mipi_sharp_cmd_sqe_cabc_outdoor_on(mfd, mipi, value.lut);
            }
            else
            {

                mipi_sharp_cmd_sqe_cabc_outdoor_move(mfd, mipi, value.lut);
            }
        }
    }
    else if(value.old_mode == SHDISP_MAIN_DISP_CABC_MODE_DBC_ACC && value.mode == SHDISP_MAIN_DISP_CABC_MODE_DBC_ACC )
    {
        if(value.old_lut == SHDISP_MAIN_DISP_CABC_LUT0)
        {

#if defined(CONFIG_SHDISP_PANEL_RYOMA)
#else
            mipi_sharp_cmd_sqe_cabc_off(mfd, mipi, 1, 0);
#endif /* CONFIG_MACH_*** */


            mipi_sharp_cmd_sqe_cabc_outdoor_on(mfd, mipi, value.lut);
        }
        else
        {
            if(value.lut == SHDISP_MAIN_DISP_CABC_LUT0)
            {

                mipi_sharp_cmd_sqe_cabc_off(mfd, mipi, 1, 0);

                mipi_sharp_cmd_sqe_cabc_indoor_on(mfd, mipi);
            }
            else
            {

                mipi_sharp_cmd_sqe_cabc_outdoor_move(mfd, mipi, value.lut);
            }
        }
    }

    return 0;
}
#endif


static int mipi_sharp_clk_write(struct msm_fb_data_type *mfd, int length)
{
    struct dsi_cmd_desc cmd[1];
    char cmd_buf[length + 1];
    
    cmd_buf[0] = 0xD9;
    memcpy(&cmd_buf[1], mipi_sharp_clk_state, length);
    cmd[0].dlen = length + 1;

    if(length > 1)
    {
        cmd[0].dtype = DTYPE_GEN_LWRITE;
    }
    else
    {
        cmd[0].dtype = DTYPE_GEN_WRITE2;
    }
    cmd[0].last = 0x01;
    cmd[0].vc = 0x00;
    cmd[0].ack = 0x00;
    cmd[0].wait = 0x00;
    cmd[0].payload = cmd_buf;

    mipi_dsi_cmds_tx(&sharp_tx_buf, cmd, ARRAY_SIZE(cmd));
    
    return 0;
}

static int mipi_sharp_panel_sync_write(struct msm_fb_data_type *mfd)
{
    struct dsi_cmd_desc cmd[1];
    char cmd_buf[13];
    
    cmd_buf[0] = 0xEC;
    memcpy(&cmd_buf[1], mipi_sharp_panel_sync, 2);
    cmd[0].dlen = 3;

    cmd[0].dtype = DTYPE_GEN_LWRITE;
    cmd[0].last = 0x01;
    cmd[0].vc = 0x00;
    cmd[0].ack = 0x00;
    cmd[0].wait = 0x00;
    cmd[0].payload = cmd_buf;

    mipi_dsi_cmds_tx(&sharp_tx_buf, cmd, ARRAY_SIZE(cmd));
    
    return 0;
}

static int mipi_sharp_ryoma_set_lpf(struct msm_fb_data_type *mfd, struct mipi_panel_info *mipi, unsigned char *param)
{
    struct dsi_cmd_desc cmd[1];
    char cmd_buf[3];
	char device_id = 0x00;
	
	if (lcd_deviceid_read_flg == 0){
		device_id = mipi_sharp_manufacture_id(mfd);
		if(device_id == 0x03){
			set_lpf_enable_flg = 1;
		}
		lcd_deviceid_read_flg = 1;
	}

    if (set_lpf_enable_flg == 0){
       return 0;
	}
	
    cmd_buf[0] = 0xD9;
    if(param[0] == 0xFF){
        if(param[1] == 1){
            cmd_buf[1] = 0x00;
            cmd_buf[2] = 0x68;
            mipi_sharp_force_60hz_move_flg = 1;
        }
        else {
            mipi_sharp_force_60hz_move_flg = 0;
            return 0;
        }
    }
    else {
        if(mipi_sharp_force_60hz_move_flg == 1) {
            return 0;
        }
        else {
            cmd_buf[1] = param[0];
            cmd_buf[2] = param[1];
        }
    }

    mipi_sharp_clk_state[0] = cmd_buf[1];
    mipi_sharp_clk_state[1] = cmd_buf[2];

    cmd[0].dlen = 3;

    cmd[0].dtype = DTYPE_GEN_LWRITE;
    cmd[0].last = 0x01;
    cmd[0].vc = 0x00;
    cmd[0].ack = 0x00;
    cmd[0].wait = 0x00;
    cmd[0].payload = cmd_buf;

    mipi_dsi_cmds_tx(&sharp_tx_buf, cmd, ARRAY_SIZE(cmd));
    
    return 0;
}

static int mipi_sharp_cmd_sqe_cabc_init(struct msm_fb_data_type *mfd, struct mipi_panel_info *mipi)
{
    mipi_sharp_clk_state[3] &= (unsigned char)~0x03;
    mipi_sharp_clk_write(mfd,4);
    
    mipi_dsi_cmds_tx(&sharp_tx_buf, mipi_sharp_cmds_cabc_init_set_pwm,
                     ARRAY_SIZE(mipi_sharp_cmds_cabc_init_set_pwm));
    mipi_dsi_cmds_tx(&sharp_tx_buf, mipi_sharp_cmds_cabc_init_set_para,
                     ARRAY_SIZE(mipi_sharp_cmds_cabc_init_set_para));
    mipi_dsi_cmds_tx(&sharp_tx_buf, mipi_sharp_cmds_cabc_init_set_flicker,
                     ARRAY_SIZE(mipi_sharp_cmds_cabc_init_set_flicker));
    mipi_dsi_cmds_tx(&sharp_tx_buf, mipi_sharp_cmds_cabc_init_set_connector,
                     ARRAY_SIZE(mipi_sharp_cmds_cabc_init_set_connector));
    
    
    mipi_sharp_panel_sync[1] = ((unsigned char)~0x70 & mipi_sharp_panel_sync[1]) | (0x70 & 0x40);
    mipi_sharp_panel_sync_write(mfd);

    mipi_sharp_ryoma_cmd_sqe_cabc_trv_primary_setting(mfd);
    
    mipi_sharp_delay_us(SHDISP_RYOMA_1V_WAIT);

    mipi_sharp_clk_state[3] |= 0x03;
    mipi_sharp_clk_write(mfd,4);
    
    shdisp_api_pwm_enable();

    return 0;
}


static int mipi_sharp_cmd_sqe_cabc_indoor_on(struct msm_fb_data_type *mfd, struct mipi_panel_info *mipi)
{
    int low_power_mode = 0;

    shdisp_api_pwm_enable();

    if((mipi_sharp_clk_state[0] & 0x01 ) == 0x01)
    {
        low_power_mode = 1;
    }

    if( low_power_mode == 1 )
    {
        mipi_sharp_clk_state[0] &= (unsigned char)~0x01;
        mipi_sharp_clk_write(mfd, 1);

        mipi_sharp_delay_us(SHDISP_RYOMA_1V_WAIT);
    }
    
    #if 0
    
    mipi_dsi_cmds_tx(&sharp_tx_buf, mipi_sharp_cmds_cabc_indoor_set_on,
                     ARRAY_SIZE(mipi_sharp_cmds_cabc_indoor_set_on));
    #endif
    mipi_dsi_cmds_tx(&sharp_tx_buf, mipi_sharp_cmds_cabc_indoor_set_image_exrate_lut0,
                     ARRAY_SIZE(mipi_sharp_cmds_cabc_indoor_set_image_exrate_lut0));
    
    mipi_dsi_cmds_tx(&sharp_tx_buf, mipi_sharp_cmds_cabc_indoor_set_flicker,
                     ARRAY_SIZE(mipi_sharp_cmds_cabc_indoor_set_flicker));

    mipi_dsi_cmds_tx(&sharp_tx_buf, mipi_sharp_cmds_cabc_indoor_set_on,
                     ARRAY_SIZE(mipi_sharp_cmds_cabc_indoor_set_on));
    
    mipi_sharp_delay_us(SHDISP_RYOMA_1V_WAIT);
    
    if( low_power_mode == 1 )
    {
        mipi_sharp_clk_state[0] |= 0x01;
        mipi_sharp_clk_write(mfd, 1);
    }

    return 0;
}


static int mipi_sharp_cmd_sqe_cabc_outdoor_on(struct msm_fb_data_type *mfd, struct mipi_panel_info *mipi, int lut_level)
{
    int low_power_mode = 0;

    if((mipi_sharp_clk_state[0] & 0x01 ) == 0x01)
    {
        low_power_mode = 1;
    }

    if( low_power_mode == 1 )
    {
        mipi_sharp_clk_state[0] &= (unsigned char)~0x01;
        mipi_sharp_clk_write(mfd, 1);

        mipi_sharp_delay_us(SHDISP_RYOMA_1V_WAIT);
    }


    
    switch(lut_level)
    {
    case SHDISP_MAIN_DISP_CABC_LUT1:
        mipi_dsi_cmds_tx(&sharp_tx_buf, mipi_sharp_cmds_cabc_outdoor_set_image_exrate_lut1,
                         ARRAY_SIZE(mipi_sharp_cmds_cabc_outdoor_set_image_exrate_lut1));

        break;
    case SHDISP_MAIN_DISP_CABC_LUT2:
        mipi_dsi_cmds_tx(&sharp_tx_buf, mipi_sharp_cmds_cabc_outdoor_set_image_exrate_lut2,
                         ARRAY_SIZE(mipi_sharp_cmds_cabc_outdoor_set_image_exrate_lut2));

        break;
    case SHDISP_MAIN_DISP_CABC_LUT3:
        mipi_dsi_cmds_tx(&sharp_tx_buf, mipi_sharp_cmds_cabc_outdoor_set_image_exrate_lut3,
                         ARRAY_SIZE(mipi_sharp_cmds_cabc_outdoor_set_image_exrate_lut3));

        break;
    case SHDISP_MAIN_DISP_CABC_LUT4:
        mipi_dsi_cmds_tx(&sharp_tx_buf, mipi_sharp_cmds_cabc_outdoor_set_image_exrate_lut4,
                         ARRAY_SIZE(mipi_sharp_cmds_cabc_outdoor_set_image_exrate_lut4));
        break;
    case SHDISP_MAIN_DISP_CABC_LUT5:
        mipi_dsi_cmds_tx(&sharp_tx_buf, mipi_sharp_cmds_cabc_outdoor_set_image_exrate_lut5,
                         ARRAY_SIZE(mipi_sharp_cmds_cabc_outdoor_set_image_exrate_lut5));
        break;
    }
    mipi_sharp_ryoma_cmd_sqe_cabc_trv_lut_setting(mfd, lut_level);
    
    mipi_dsi_cmds_tx(&sharp_tx_buf, mipi_sharp_cmds_cabc_outdoor_set_flicker,
                     ARRAY_SIZE(mipi_sharp_cmds_cabc_outdoor_set_flicker));
    
    
    mipi_dsi_cmds_tx(&sharp_tx_buf, mipi_sharp_cmds_cabc_outdoor_set_on,
                     ARRAY_SIZE(mipi_sharp_cmds_cabc_outdoor_set_on));
    
    mipi_sharp_ryoma_cmd_sqe_cabc_trv_on(mfd);

    mipi_sharp_delay_us(SHDISP_RYOMA_1V_WAIT);

    if( low_power_mode == 1 )
    {
        mipi_sharp_clk_state[0] |= 0x01;
        mipi_sharp_clk_write(mfd, 1);
    }
    
    return 0;
}


static int mipi_sharp_cmd_sqe_cabc_off(struct msm_fb_data_type *mfd, struct mipi_panel_info *mipi, int wait_on, int pwm_disable)
{
    int low_power_mode = 0;

    if((mipi_sharp_clk_state[0] & 0x01 ) == 0x01)
    {
        low_power_mode = 1;
    }

    if( low_power_mode == 1 )
    {
        mipi_sharp_clk_state[0] &= (unsigned char)~0x01;
        mipi_sharp_clk_write(mfd, 1);

        mipi_sharp_delay_us(SHDISP_RYOMA_1V_WAIT);
    }


    mipi_sharp_ryoma_cmd_sqe_cabc_trv_off(mfd);

    mipi_dsi_cmds_tx(&sharp_tx_buf, mipi_sharp_cmds_cabc_off_set_image_exrate_lut0,
             ARRAY_SIZE(mipi_sharp_cmds_cabc_off_set_image_exrate_lut0));

    mipi_dsi_cmds_tx(&sharp_tx_buf, mipi_sharp_cmds_cabc_off_set_off,
                     ARRAY_SIZE(mipi_sharp_cmds_cabc_off_set_off));


    mipi_sharp_delay_us(SHDISP_RYOMA_1V_WAIT);

    if( low_power_mode == 1 )
    {
        mipi_sharp_clk_state[0] |= 0x01;
        mipi_sharp_clk_write(mfd, 1);
    }

   if( pwm_disable == 1 )
   {
        shdisp_api_pwm_disable();

        if( low_power_mode == 1 )
        {
			mipi_sharp_clk_state[3] &= (unsigned char)~0x03;
            mipi_sharp_clk_write(mfd,4);
        }

        mipi_dsi_cmds_tx(&sharp_tx_buf, mipi_sharp_cmds_cabc_pwmoff_unset_connector,
                         ARRAY_SIZE(mipi_sharp_cmds_cabc_pwmoff_unset_connector));


        mipi_sharp_delay_us(SHDISP_RYOMA_1V_WAIT);

        if( low_power_mode == 1 )
        {
            mipi_sharp_clk_state[3] |= 0x03;
            mipi_sharp_clk_write(mfd,4);
        }
   }

    return 0;
}


static int mipi_sharp_cmd_sqe_cabc_outdoor_move(struct msm_fb_data_type *mfd, struct mipi_panel_info *mipi, int lut_level)
{
    int low_power_mode = 0;
    char mipi_sh_ryoma_clk_state_bak[2];

    if((mipi_sharp_clk_state[0] & 0x01 ) == 0x01)
    {
        low_power_mode = 1;
    }

    if( low_power_mode == 1 )
    {
        mipi_sharp_clk_state[0] &= (unsigned char)~0x01;
        mipi_sharp_clk_write(mfd, 1);

        mipi_sharp_delay_us(SHDISP_RYOMA_1V_WAIT);
    }

    mipi_sh_ryoma_clk_state_bak[0] = mipi_sharp_clk_state[0];
    mipi_sh_ryoma_clk_state_bak[1] = mipi_sharp_clk_state[1];

    mipi_sharp_clk_state[1] = 0x68;
    mipi_sharp_clk_write(mfd, 2);

    mipi_sharp_clk_state[0] |= 0x01;
    mipi_sharp_clk_state[2] = 0x43;
    mipi_sharp_clk_write(mfd, 3);

    mipi_sharp_delay_us(SHDISP_RYOMA_1V_WAIT);

    mipi_sharp_clk_state[2] = 0x03;
    mipi_sharp_clk_write(mfd, 3);

    mipi_sharp_delay_us(SHDISP_RYOMA_1V_WAIT);


    switch(lut_level)
    {
    case SHDISP_MAIN_DISP_CABC_LUT1:
        mipi_dsi_cmds_tx(&sharp_tx_buf, mipi_sharp_cmds_cabc_transition_set_image_exrate_lut1,
                         ARRAY_SIZE(mipi_sharp_cmds_cabc_transition_set_image_exrate_lut1));
        break;
    case SHDISP_MAIN_DISP_CABC_LUT2:
        mipi_dsi_cmds_tx(&sharp_tx_buf, mipi_sharp_cmds_cabc_transition_set_image_exrate_lut2,
                         ARRAY_SIZE(mipi_sharp_cmds_cabc_transition_set_image_exrate_lut2));
        break;
    case SHDISP_MAIN_DISP_CABC_LUT3:
        mipi_dsi_cmds_tx(&sharp_tx_buf, mipi_sharp_cmds_cabc_transition_set_image_exrate_lut3,
                         ARRAY_SIZE(mipi_sharp_cmds_cabc_transition_set_image_exrate_lut3));
        break;
    case SHDISP_MAIN_DISP_CABC_LUT4:
        mipi_dsi_cmds_tx(&sharp_tx_buf, mipi_sharp_cmds_cabc_transition_set_image_exrate_lut4,
                         ARRAY_SIZE(mipi_sharp_cmds_cabc_transition_set_image_exrate_lut4));
        break;
    case SHDISP_MAIN_DISP_CABC_LUT5:
        mipi_dsi_cmds_tx(&sharp_tx_buf, mipi_sharp_cmds_cabc_transition_set_image_exrate_lut5,
                         ARRAY_SIZE(mipi_sharp_cmds_cabc_transition_set_image_exrate_lut5));
        break;
    }
    
    mipi_sharp_ryoma_cmd_sqe_cabc_trv_lut_setting(mfd, lut_level);

    mipi_sharp_ryoma_cmd_sqe_cabc_trv_on(mfd);

    if( low_power_mode == 1 )
    {
        mipi_sharp_clk_state[0] |= 0x01;
    }
    else
    {
        mipi_sharp_clk_state[0] &= (unsigned char)~0x01;
    }

    mipi_sharp_clk_state[1] = mipi_sh_ryoma_clk_state_bak[1];
    mipi_sharp_clk_state[2] = 0x4F;
    mipi_sharp_clk_write(mfd, 3);

    return 0;
}

static int mipi_sharp_ryoma_cmd_sqe_cabc_trv_primary_setting(struct msm_fb_data_type *mfd)
{
    mipi_dsi_cmds_tx(&sharp_tx_buf, mipi_sharp_cmds_cabc_bank_lock,
                     ARRAY_SIZE(mipi_sharp_cmds_cabc_bank_lock));
    mipi_dsi_cmds_tx(&sharp_tx_buf, mipi_sharp_cmds_cabc_bank_01,
                     ARRAY_SIZE(mipi_sharp_cmds_cabc_bank_01));
    mipi_dsi_cmds_tx(&sharp_tx_buf, mipi_sharp_cmds_cabc_trv_lock0,
                     ARRAY_SIZE(mipi_sharp_cmds_cabc_trv_lock0));
    mipi_dsi_cmds_tx(&sharp_tx_buf, mipi_sharp_cmds_cabc_trv_lock1,
                     ARRAY_SIZE(mipi_sharp_cmds_cabc_trv_lock1));
    mipi_dsi_cmds_tx(&sharp_tx_buf, mipi_sharp_cmds_cabc_bank_00,
                     ARRAY_SIZE(mipi_sharp_cmds_cabc_bank_00));
    mipi_dsi_cmds_tx(&sharp_tx_buf, mipi_sharp_cmds_cabc_fntn_s,
                     ARRAY_SIZE(mipi_sharp_cmds_cabc_fntn_s));
    mipi_dsi_cmds_tx(&sharp_tx_buf, mipi_sharp_cmds_cabc_bank_01,
                     ARRAY_SIZE(mipi_sharp_cmds_cabc_bank_01));
    mipi_dsi_cmds_tx(&sharp_tx_buf, mipi_sharp_cmds_cabc_trv_mode_set,
                     ARRAY_SIZE(mipi_sharp_cmds_cabc_trv_mode_set));
    mipi_dsi_cmds_tx(&sharp_tx_buf, mipi_sharp_cmds_cabc_bank_00,
                     ARRAY_SIZE(mipi_sharp_cmds_cabc_bank_00));
    mipi_dsi_cmds_tx(&sharp_tx_buf, mipi_sharp_cmds_cabc_val_go,
                     ARRAY_SIZE(mipi_sharp_cmds_cabc_val_go));

    return 0;
}
static void mipi_sharp_cabc_acc_gamma_init(void)
{
    memcpy(setting_val_cabc_lut1_gamma_a, setting_val_gamma_a, CABC_LUT_TABLE_SIZE);
    memcpy(setting_val_cabc_lut1_gamma_b, setting_val_gamma_b, CABC_LUT_TABLE_SIZE);
    memcpy(setting_val_cabc_lut1_gamma_c, setting_val_gamma_c, CABC_LUT_TABLE_SIZE);
    memcpy(setting_val_cabc_lut2_gamma_a, setting_val_gamma_a, CABC_LUT_TABLE_SIZE);
    memcpy(setting_val_cabc_lut2_gamma_b, setting_val_gamma_b, CABC_LUT_TABLE_SIZE);
    memcpy(setting_val_cabc_lut2_gamma_c, setting_val_gamma_c, CABC_LUT_TABLE_SIZE);
    memcpy(setting_val_cabc_lut3_gamma_a, setting_val_gamma_a, CABC_LUT_TABLE_SIZE);
    memcpy(setting_val_cabc_lut3_gamma_b, setting_val_gamma_b, CABC_LUT_TABLE_SIZE);
    memcpy(setting_val_cabc_lut3_gamma_c, setting_val_gamma_c, CABC_LUT_TABLE_SIZE);
    memcpy(setting_val_cabc_lut4_gamma_a, setting_val_gamma_a, CABC_LUT_TABLE_SIZE);
    memcpy(setting_val_cabc_lut4_gamma_b, setting_val_gamma_b, CABC_LUT_TABLE_SIZE);
    memcpy(setting_val_cabc_lut4_gamma_c, setting_val_gamma_c, CABC_LUT_TABLE_SIZE);
    memcpy(setting_val_cabc_lut5_gamma_a, setting_val_gamma_a, CABC_LUT_TABLE_SIZE);
    memcpy(setting_val_cabc_lut5_gamma_b, setting_val_gamma_b, CABC_LUT_TABLE_SIZE);
    memcpy(setting_val_cabc_lut5_gamma_c, setting_val_gamma_c, CABC_LUT_TABLE_SIZE);

}


static int mipi_sharp_ryoma_cmd_sqe_cabc_trv_lut_write(struct msm_fb_data_type *mfd, char addr, char val)
{
    struct dsi_cmd_desc cmd[1];
    char cmd_buf[2];
    
    cmd_buf[0] = addr;
    cmd_buf[1] = val;
    cmd[0].dlen = 2;

    cmd[0].dtype = DTYPE_GEN_WRITE2;
    cmd[0].last = 0x01;
    cmd[0].vc = 0x00;
    cmd[0].ack = 0x00;
    cmd[0].wait = 0x00;
    cmd[0].payload = cmd_buf;

    mipi_dsi_cmds_tx(&sharp_tx_buf, cmd, ARRAY_SIZE(cmd));
    
    return 0;
}

static int mipi_sharp_ryoma_cmd_sqe_cabc_trv_lut_setting(struct msm_fb_data_type *mfd, int lut_level)
{
    int i;


    mipi_dsi_cmds_tx(&sharp_tx_buf, mipi_sharp_cmds_cabc_trv_lut_set_on,
                     ARRAY_SIZE(mipi_sharp_cmds_cabc_trv_lut_set_on));
    mipi_dsi_cmds_tx(&sharp_tx_buf, mipi_sharp_cmds_cabc_bank_07,
                     ARRAY_SIZE(mipi_sharp_cmds_cabc_bank_07));

    for(i = 0 ; i < CABC_TRVLUT_QUARTER_TABLE_SIZE ; i++ )
    {
        switch(lut_level)
        {
        case SHDISP_MAIN_DISP_CABC_LUT1:
            mipi_sharp_ryoma_cmd_sqe_cabc_trv_lut_write(mfd, mipi_sharp_cabc_trvlut_addr[i], mipi_sharp_cabc_trvlut1_q1[i]);
            break;
        case SHDISP_MAIN_DISP_CABC_LUT2:
            mipi_sharp_ryoma_cmd_sqe_cabc_trv_lut_write(mfd, mipi_sharp_cabc_trvlut_addr[i], mipi_sharp_cabc_trvlut2_q1[i]);
            break;
        case SHDISP_MAIN_DISP_CABC_LUT3:
            mipi_sharp_ryoma_cmd_sqe_cabc_trv_lut_write(mfd, mipi_sharp_cabc_trvlut_addr[i], mipi_sharp_cabc_trvlut3_q1[i]);
            break;
        case SHDISP_MAIN_DISP_CABC_LUT4:
            mipi_sharp_ryoma_cmd_sqe_cabc_trv_lut_write(mfd, mipi_sharp_cabc_trvlut_addr[i], mipi_sharp_cabc_trvlut4_q1[i]);
            break;
        case SHDISP_MAIN_DISP_CABC_LUT5:
            mipi_sharp_ryoma_cmd_sqe_cabc_trv_lut_write(mfd, mipi_sharp_cabc_trvlut_addr[i], mipi_sharp_cabc_trvlut5_q1[i]);
            break;
        }
    }

    mipi_dsi_cmds_tx(&sharp_tx_buf, mipi_sharp_cmds_cabc_bank_08,
                     ARRAY_SIZE(mipi_sharp_cmds_cabc_bank_08));

    for(i = 0 ; i < CABC_TRVLUT_QUARTER_TABLE_SIZE ; i++ )
    {
        switch(lut_level)
        {
        case SHDISP_MAIN_DISP_CABC_LUT1:
            mipi_sharp_ryoma_cmd_sqe_cabc_trv_lut_write(mfd, mipi_sharp_cabc_trvlut_addr[i], mipi_sharp_cabc_trvlut1_q2[i]);
            break;
        case SHDISP_MAIN_DISP_CABC_LUT2:
            mipi_sharp_ryoma_cmd_sqe_cabc_trv_lut_write(mfd, mipi_sharp_cabc_trvlut_addr[i], mipi_sharp_cabc_trvlut2_q2[i]);
            break;
        case SHDISP_MAIN_DISP_CABC_LUT3:
            mipi_sharp_ryoma_cmd_sqe_cabc_trv_lut_write(mfd, mipi_sharp_cabc_trvlut_addr[i], mipi_sharp_cabc_trvlut3_q2[i]);
            break;
        case SHDISP_MAIN_DISP_CABC_LUT4:
            mipi_sharp_ryoma_cmd_sqe_cabc_trv_lut_write(mfd, mipi_sharp_cabc_trvlut_addr[i], mipi_sharp_cabc_trvlut4_q2[i]);
            break;
        case SHDISP_MAIN_DISP_CABC_LUT5:
            mipi_sharp_ryoma_cmd_sqe_cabc_trv_lut_write(mfd, mipi_sharp_cabc_trvlut_addr[i], mipi_sharp_cabc_trvlut5_q2[i]);
            break;
        }
    }

    mipi_dsi_cmds_tx(&sharp_tx_buf, mipi_sharp_cmds_cabc_bank_09,
                     ARRAY_SIZE(mipi_sharp_cmds_cabc_bank_09));

    for(i = 0 ; i < CABC_TRVLUT_QUARTER_TABLE_SIZE ; i++ )
    {
        switch(lut_level)
        {
        case SHDISP_MAIN_DISP_CABC_LUT1:
            mipi_sharp_ryoma_cmd_sqe_cabc_trv_lut_write(mfd, mipi_sharp_cabc_trvlut_addr[i], mipi_sharp_cabc_trvlut1_q3[i]);
            break;
        case SHDISP_MAIN_DISP_CABC_LUT2:
            mipi_sharp_ryoma_cmd_sqe_cabc_trv_lut_write(mfd, mipi_sharp_cabc_trvlut_addr[i], mipi_sharp_cabc_trvlut2_q3[i]);
            break;
        case SHDISP_MAIN_DISP_CABC_LUT3:
            mipi_sharp_ryoma_cmd_sqe_cabc_trv_lut_write(mfd, mipi_sharp_cabc_trvlut_addr[i], mipi_sharp_cabc_trvlut3_q3[i]);
            break;
        case SHDISP_MAIN_DISP_CABC_LUT4:
            mipi_sharp_ryoma_cmd_sqe_cabc_trv_lut_write(mfd, mipi_sharp_cabc_trvlut_addr[i], mipi_sharp_cabc_trvlut4_q3[i]);
            break;
        case SHDISP_MAIN_DISP_CABC_LUT5:
            mipi_sharp_ryoma_cmd_sqe_cabc_trv_lut_write(mfd, mipi_sharp_cabc_trvlut_addr[i], mipi_sharp_cabc_trvlut5_q3[i]);
            break;
        }
    }

    mipi_dsi_cmds_tx(&sharp_tx_buf, mipi_sharp_cmds_cabc_bank_0A,
                     ARRAY_SIZE(mipi_sharp_cmds_cabc_bank_0A));

    for(i = 0 ; i < CABC_TRVLUT_QUARTER_TABLE_SIZE ; i++ )
    {
        switch(lut_level)
        {
        case SHDISP_MAIN_DISP_CABC_LUT1:
            mipi_sharp_ryoma_cmd_sqe_cabc_trv_lut_write(mfd, mipi_sharp_cabc_trvlut_addr[i], mipi_sharp_cabc_trvlut1_q4[i]);
            break;
        case SHDISP_MAIN_DISP_CABC_LUT2:
            mipi_sharp_ryoma_cmd_sqe_cabc_trv_lut_write(mfd, mipi_sharp_cabc_trvlut_addr[i], mipi_sharp_cabc_trvlut2_q4[i]);
            break;
        case SHDISP_MAIN_DISP_CABC_LUT3:
            mipi_sharp_ryoma_cmd_sqe_cabc_trv_lut_write(mfd, mipi_sharp_cabc_trvlut_addr[i], mipi_sharp_cabc_trvlut3_q4[i]);
            break;
        case SHDISP_MAIN_DISP_CABC_LUT4:
            mipi_sharp_ryoma_cmd_sqe_cabc_trv_lut_write(mfd, mipi_sharp_cabc_trvlut_addr[i], mipi_sharp_cabc_trvlut4_q4[i]);
            break;
        case SHDISP_MAIN_DISP_CABC_LUT5:
            mipi_sharp_ryoma_cmd_sqe_cabc_trv_lut_write(mfd, mipi_sharp_cabc_trvlut_addr[i], mipi_sharp_cabc_trvlut5_q4[i]);
            break;
        }
    }

    mipi_dsi_cmds_tx(&sharp_tx_buf, mipi_sharp_cmds_cabc_bank_00,
                     ARRAY_SIZE(mipi_sharp_cmds_cabc_bank_00));
    mipi_dsi_cmds_tx(&sharp_tx_buf, mipi_sharp_cmds_cabc_trv_lut_set_off,
                     ARRAY_SIZE(mipi_sharp_cmds_cabc_trv_lut_set_off));

    return 0;
}

static int mipi_sharp_ryoma_cmd_sqe_cabc_trv_on(struct msm_fb_data_type *mfd)
{

    mipi_dsi_cmds_tx(&sharp_tx_buf, mipi_sharp_cmds_cabc_bank_01,
                     ARRAY_SIZE(mipi_sharp_cmds_cabc_bank_01));
    mipi_dsi_cmds_tx(&sharp_tx_buf, mipi_sharp_cmds_cabc_trv_on,
                     ARRAY_SIZE(mipi_sharp_cmds_cabc_trv_on));
    mipi_dsi_cmds_tx(&sharp_tx_buf, mipi_sharp_cmds_cabc_bank_00,
                     ARRAY_SIZE(mipi_sharp_cmds_cabc_bank_00));
    mipi_dsi_cmds_tx(&sharp_tx_buf, mipi_sharp_cmds_cabc_val_go,
                     ARRAY_SIZE(mipi_sharp_cmds_cabc_val_go));

    return 0;
}

static int mipi_sharp_ryoma_cmd_sqe_cabc_trv_off(struct msm_fb_data_type *mfd)
{

    mipi_dsi_cmds_tx(&sharp_tx_buf, mipi_sharp_cmds_cabc_bank_01,
                     ARRAY_SIZE(mipi_sharp_cmds_cabc_bank_01));
    mipi_dsi_cmds_tx(&sharp_tx_buf, mipi_sharp_cmds_cabc_trv_off,
                     ARRAY_SIZE(mipi_sharp_cmds_cabc_trv_off));
    mipi_dsi_cmds_tx(&sharp_tx_buf, mipi_sharp_cmds_cabc_bank_00,
                     ARRAY_SIZE(mipi_sharp_cmds_cabc_bank_00));
    mipi_dsi_cmds_tx(&sharp_tx_buf, mipi_sharp_cmds_cabc_val_go,
                     ARRAY_SIZE(mipi_sharp_cmds_cabc_val_go));

    return 0;
}



#if defined(CONFIG_SHDISP_USE_CABC)
static int mipi_sharp_ryoma_cabc_ctrl(struct platform_device *pdev)
{
    struct msm_fb_data_type *mfd;
    struct mipi_panel_info *mipi;
    int ret = 0;
    
    mfd = platform_get_drvdata(pdev);
    if (!mfd) {
        return -ENODEV;
    }
    if (mfd->key != MFD_KEY) {
        return -EINVAL;
    }
    
    mipi = &mfd->panel_info.mipi;
    if (mipi->mode == DSI_CMD_MODE) {
        ret = mipi_sharp_cabc_disp_on_ctrl(mfd, mipi);
    }

    return 0;
}
#endif
static int __devinit mipi_sharp_ryoma_lcd_probe(struct platform_device *pdev)
{
	struct mipi_dsi_panel_platform_data *platform_data;
	struct msm_fb_panel_data            *pdata;
	struct msm_panel_info               *pinfo;

	static u32 width, height;

	if (pdev->id == 0) {
		platform_data = pdev->dev.platform_data;

		if (platform_data) {
			width  = (platform_data->width_in_mm);
			height = (platform_data->height_in_mm);
		}

		return 0;
	}

	pdata = (struct msm_fb_panel_data *)pdev->dev.platform_data;
	pinfo = &(pdata->panel_info);

	pinfo->width_in_mm  = width;
	pinfo->height_in_mm = height;

	msm_fb_add_device(pdev);

	return 0;
}

static struct platform_driver this_driver = {
	.probe  = mipi_sharp_ryoma_lcd_probe,
	.driver = {
		.name   = "mipi_sharp_ryoma",
	},
};

static struct msm_fb_panel_data sharp_ryoma_panel_data = {
	.on		= mipi_sharp_ryoma_lcd_on,
	.off	= mipi_sharp_ryoma_lcd_off,
	.set_backlight = mipi_sharp_ryoma_lcd_set_backlight,
	.start_display = mipi_sharp_ryoma_lcd_start_display,

	.cabc_init			= mipi_sharp_cmd_sqe_cabc_init,
	.cabc_indoor_on		= mipi_sharp_cmd_sqe_cabc_indoor_on,
	.cabc_outdoor_on	= mipi_sharp_cmd_sqe_cabc_outdoor_on,
	.cabc_off			= mipi_sharp_cmd_sqe_cabc_off,
	.cabc_outdoor_move	= mipi_sharp_cmd_sqe_cabc_outdoor_move,
    .cabc_ctrl			= mipi_sharp_ryoma_cabc_ctrl,
	.set_lpf			= mipi_sharp_ryoma_set_lpf,
};

int mipi_sharp_ryoma_device_register(struct msm_panel_info *pinfo,
					u32 channel, u32 panel)
{
	struct platform_device *pdev = NULL;
	int ret;

	
	if ((channel >= 3) || ch_used[channel])
		return -ENODEV;

	
	ch_used[channel] = TRUE;

	
	pdev = platform_device_alloc("mipi_sharp_ryoma", (panel << 8)|channel);
	if (!pdev)
		return -ENOMEM;

	sharp_ryoma_panel_data.panel_info = *pinfo;

	
	ret = platform_device_add_data(pdev, &sharp_ryoma_panel_data,
		sizeof(sharp_ryoma_panel_data));
	if (ret) {
		printk(KERN_ERR
		  "%s: platform_device_add_data failed!\n", __func__);
		goto err_device_put;
	}

	ret = platform_device_add(pdev);
	if (ret) {
		printk(KERN_ERR
		  "%s: platform_device_register failed!\n", __func__);
		goto err_device_put;
	}

	return 0;

err_device_put:
	platform_device_put(pdev);
	return ret;
}

int mipi_sharp_ryoma_diag_write_reg(unsigned char addr, unsigned char *write_data, unsigned char size)
{
    int ret;

    if(size > MIPI_SHARP_RW_MAX_SIZE) {
        pr_err("%s: size over, -EINVAL\n", __func__);
        ret = -EINVAL;
    }
    else {
        ret = mipi_sharp_diag_write_reg(addr, write_data, size);
    }

    return ret;
}

int mipi_sharp_ryoma_diag_read_reg(unsigned char addr, unsigned char *read_data, unsigned char size)
{
    int ret;

    if(size > MIPI_SHARP_RW_MAX_SIZE) {
        pr_err("%s: size over, -EINVAL\n", __func__);
        ret = -EINVAL;
    }
    else {
        ret = mipi_sharp_diag_read_reg(addr, read_data, size);
    }

    return ret;
}

int mipi_sharp_ryoma_init_phy_gamma(struct shdisp_lcddr_phy_gamma_reg *phy_gamma)
{
    int ret = 0;
    int cnt;
    int checksum;

    setting_val_gamma_a[0] = mipi_sharp_cmd_gamma_a[0];
    setting_val_gamma_b[0] = mipi_sharp_cmd_gamma_b[0];
    setting_val_gamma_c[0] = mipi_sharp_cmd_gamma_c[0];
    if(phy_gamma == NULL) {
        printk("%s: phy_gamma is NULL.\n", __func__);
        ret = -1;
    }
    else if((phy_gamma->status != SHDISP_LCDDR_GAMMA_STATUS_OK) && 
            (phy_gamma->status != SHDISP_LCDDR_GAMMA_STATUS_OK_2) ) {
        printk("%s: gamma status invalid. status=%02x\n", __func__, phy_gamma->status);
        ret = -1;
    }
    else {
        checksum = phy_gamma->status;
        for(cnt = 0; cnt < SHDISP_LCDDR_PHY_GAMMA_BUF_MAX; cnt++) {
            checksum = checksum + phy_gamma->buf[cnt];
        }
        if((checksum & 0xFFFF) != phy_gamma->chksum) {
            printk("%s: gamma chksum NG. chksum=%04x calc_chksum=%04x\n", __func__, phy_gamma->chksum, (checksum & 0xFFFF));
            ret = -1;
        }
        else {
            for(cnt = 0; cnt < SHDISP_LCDDR_GAMMA_OFFSET; cnt++) {
                setting_val_gamma_a[cnt + 1]                             = phy_gamma->buf[cnt];
                setting_val_gamma_a[cnt + SHDISP_LCDDR_GAMMA_OFFSET + 1] = phy_gamma->buf[cnt + SHDISP_LCDDR_GAMMA_OFFSET];
                setting_val_gamma_b[cnt + 1]                             = phy_gamma->buf[cnt + SHDISP_LCDDR_GAMMA_OFFSET * 2];
                setting_val_gamma_b[cnt + SHDISP_LCDDR_GAMMA_OFFSET + 1] = phy_gamma->buf[cnt + SHDISP_LCDDR_GAMMA_OFFSET * 3];
                setting_val_gamma_c[cnt + 1]                             = phy_gamma->buf[cnt + SHDISP_LCDDR_GAMMA_OFFSET * 4];
                setting_val_gamma_c[cnt + SHDISP_LCDDR_GAMMA_OFFSET + 1] = phy_gamma->buf[cnt + SHDISP_LCDDR_GAMMA_OFFSET * 5];
            }
        }
    }
    if(ret < 0) {
        for(cnt = 1; cnt < SHDISP_LCDDR_GAMMA_OFFSET + 1; cnt++) {
            setting_val_gamma_a[cnt]                             = mipi_sharp_cmd_gamma_a[cnt];
            setting_val_gamma_a[cnt + SHDISP_LCDDR_GAMMA_OFFSET] = mipi_sharp_cmd_gamma_a[cnt + SHDISP_LCDDR_GAMMA_OFFSET];
            setting_val_gamma_b[cnt]                             = mipi_sharp_cmd_gamma_b[cnt];
            setting_val_gamma_b[cnt + SHDISP_LCDDR_GAMMA_OFFSET] = mipi_sharp_cmd_gamma_b[cnt + SHDISP_LCDDR_GAMMA_OFFSET];
            setting_val_gamma_c[cnt]                             = mipi_sharp_cmd_gamma_c[cnt];
            setting_val_gamma_c[cnt + SHDISP_LCDDR_GAMMA_OFFSET] = mipi_sharp_cmd_gamma_c[cnt + SHDISP_LCDDR_GAMMA_OFFSET];
        }
    }
    mipi_sharp_cabc_acc_gamma_init();
    return ret;
}

static int __init mipi_sharp_ryoma_lcd_init(void)
{
    int ret;
    
    mipi_dsi_buf_alloc(&sharp_tx_buf, DSI_BUF_SIZE);
    mipi_dsi_buf_alloc(&sharp_rx_buf, DSI_BUF_SIZE);
    
    ret = platform_driver_register(&this_driver);
    if( ret < 0 )
    {
        goto mipi_init_err_1;
    }
    return 0;

mipi_init_err_1:
    return -1;
}
module_init(mipi_sharp_ryoma_lcd_init);

static void mipi_sharp_ryoma_lcd_exit(void)
{
    platform_driver_unregister(&this_driver);
    return;
}
module_exit(mipi_sharp_ryoma_lcd_exit);
