/* drivers/video/msm/mipi_sharp_kite.c  (Display Driver)
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
#include "mipi_sharp_kite.h"
#include <sharp/shdisp_kerl.h>

#if !defined(CONFIG_MACH_TOR)
    #include <linux/gpio.h>
    #include <linux/mfd/pm8xxx/pm8921.h>
    #include <mach/irqs.h>
#endif

#if defined(CONFIG_MACH_LYNX_DL10)
    #include "../../sharp/shdisp/data/shdisp_cabc_data_dl10.h"
#elif defined(CONFIG_MACH_BLT)
    #include "../../sharp/shdisp/data/shdisp_cabc_data_bolt.h"
#elif defined(CONFIG_MACH_DECKARD_AS70)
    #include "../../sharp/shdisp/data/shdisp_cabc_data_as70.h"
#elif defined(CONFIG_MACH_TOR)
    #include "../../sharp/shdisp/data/shdisp_cabc_data_tori.h"
#else
    #include "../../sharp/shdisp/data/shdisp_cabc_data_default.h"
#endif
/* ------------------------------------------------------------------------- */
/* DEBUG MACRAOS                                                             */
/* ------------------------------------------------------------------------- */

#define MIPI_SHARP_RW_MAX_SIZE          42

static struct dsi_buf sharp_tx_buf;
static struct dsi_buf sharp_rx_buf;
static int ch_used[3];
static int lcd_deviceid_out = 0;

static char mipi_sharp_cmd_protect[2] = {0xB0, 0x04};
static char mipi_sharp_cmd_manufacture_id[2] = {0xBF, 0x00};
static char mipi_sharp_cmd_ts1[4] = {0xFE, 0x00,0x40,0x2D};
static char mipi_sharp_cmd_ts3[4] = {0xFE, 0x00,0x00,0x0D};
static char mipi_sharp_cmd_init1[6] = {0xB3, 0x00,0x00,0x22,0x00,0x00};
static char mipi_sharp_cmd_init2[3] = {0xB4, 0x0C,0x12};
static char mipi_sharp_cmd_init3[3] = {0xB6, 0x31,0xB5};
static char mipi_sharp_cmd_panel[2] = {0xCC, 0x06};
static char mipi_sharp_cmd_timing1[28] = {0xC1, 
    0x04,0x64,0x10,0x41,0x00,0x00,0x8E,0x29,0xEF,0xBD,0xF7,0xDE,
    0x7B,0xEF,0xBD,0xF7,0xDE,0x7B,0xC5,0x1C,0x02,0x86,0x08,0x22,
    0x22,0x00,0x20};
static char mipi_sharp_cmd_timing2[7] = {0xC2, 
    0x20,0xF5,0x00,0x14,0x08,0x00};
static char mipi_sharp_cmd_timing3[4] = {0xC3, 0x00,0x00,0x00};
static char mipi_sharp_cmd_timing4_clk[14] = {0xC4, 
    0x00,0x00,0x00,0x00,0x00,0x00,0x08,0x00,0x00,0x00,0x00,0x00,
    0x08};
static char mipi_sharp_cmd_timing5_clk[41] = {0xC6, 
    0xAC,0x00,0xAB,0x05,0xA0,0x00,0x00,0x00,0x00,0x00,0x00,0x00,
    0x00,0x00,0x00,0x00,0x00,0x0B,0x20,0x0F,0xAC,0x00,0xAB,0x05,
    0xA0,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,
    0x00,0x0B,0x20,0x0F};
static char mipi_sharp_cmd_timing4_osc[14] = {0xC4, 
    0x00,0x00,0x00,0x00,0x00,0x00,0x09,0x00,0x00,0x00,0x00,0x00,
    0x09};
static char mipi_sharp_cmd_timing5_osc[41] = {0xC6, 
    0xB2,0x00,0xB1,0x05,0xA6,0x00,0x00,0x00,0x00,0x00,0x00,0x00,
    0x00,0x00,0x00,0x00,0x00,0x0B,0x21,0x10,0xB2,0x00,0xB1,0x05,
    0xA6,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,
    0x00,0x0B,0x21,0x10};
static char mipi_sharp_cmd_gamma_a[25] = {0xC7, 
    0x00,0x0A,0x12,0x1B,0x2A,0x3F,0x2F,0x3C,0x48,0x56,0x63,0x6C,
    0x00,0x0A,0x12,0x1B,0x2A,0x3F,0x2F,0x3C,0x48,0x56,0x63,0x6C};
static char mipi_sharp_cmd_gamma_b[25] = {0xC8, 
    0x0B,0x12,0x18,0x20,0x2C,0x40,0x2F,0x3C,0x48,0x56,0x63,0x6C,
    0x0B,0x12,0x18,0x20,0x2C,0x40,0x2F,0x3C,0x48,0x56,0x63,0x6C};
static char mipi_sharp_cmd_gamma_c[25] = {0xC9, 
    0x12,0x17,0x1C,0x23,0x2E,0x41,0x2F,0x3C,0x49,0x56,0x63,0x6C,
    0x12,0x17,0x1C,0x23,0x2E,0x41,0x2F,0x3C,0x49,0x56,0x63,0x6C};
static char mipi_sharp_cmd_power1[13] = {0xD0, 
    0x10,0x4C,0x18,0xCC,0xDA,0x5A,0x01,0x8A,0x01,0xBB,0x58,0x4A};
static char mipi_sharp_cmd_power2[2] = {0xD2, 0xB8};
static char mipi_sharp_cmd_power3[22] = {0xD3, 
    0x1A,0xB3,0xBB,0xFF,0x77,0x33,0x33,0x33,0x00,0x01,0x00,0xA0,
    0x38,0xA0,0x00,0xDB,0xB7,0x33,0xA2,0x72,0xDB};
static char mipi_sharp_cmd_vcom[8] = {0xD5, 
    0x06,0x00,0x00,0x01,0x00,0x01,0x00};
static char mipi_sharp_cmd_power4[2] = {0xD6, 0x01};
static char mipi_sharp_cmd_power5[23] = {0xD7, 
    0x20,0x80,0xFC,0xFF,0x7F,0x22,0xA2,0xA2,0x80,0x0A,0xF0,0x60,
    0x7E,0x00,0x3C,0x18,0x40,0x05,0x7E,0x00,0x00,0x00};
static char mipi_sharp_cmd_setram[11] = {0xE4, 
    0x04,0x00,0x00,0x00,0x00,0x00,0x03,0x00,0x00,0x00};
static char mipi_sharp_cmd_column_addr[5] = {0x2A, 0x00,0x00,0x02,0xCF};
static char mipi_sharp_cmd_page_addr[5] = {0x2B, 0x00,0x00,0x04,0xFF};
static char mipi_sharp_cmd_tear_on[2] = {0x35, 0x00};
static char mipi_sharp_cmd_display_on[2] = {0x29, 0x00};
static char mipi_sharp_cmd_exit_sleep[2] = {0x11, 0x00};
static struct dsi_cmd_desc mipi_sharp_cmds_protect[] = {
    {DTYPE_GEN_WRITE2, 1, 0, 0, 0, sizeof(mipi_sharp_cmd_protect), mipi_sharp_cmd_protect}
};
static struct dsi_cmd_desc mipi_sharp_cmds_manufacture_id[] = {
    {DTYPE_GEN_READ1, 1, 0, 1, 0, sizeof(mipi_sharp_cmd_manufacture_id), mipi_sharp_cmd_manufacture_id}
};
static struct dsi_cmd_desc mipi_sharp_cmds_testmode1[] = {
    {DTYPE_GEN_LWRITE, 1, 0, 0, 0, sizeof(mipi_sharp_cmd_ts1), mipi_sharp_cmd_ts1}
};
static struct dsi_cmd_desc mipi_sharp_cmds_testmode3[] = {
    {DTYPE_GEN_LWRITE, 1, 0, 0, 0, sizeof(mipi_sharp_cmd_ts3), mipi_sharp_cmd_ts3}
};
static struct dsi_cmd_desc mipi_sharp_cmds_init[] = {
    {DTYPE_GEN_LWRITE, 1, 0, 0, 0, sizeof(mipi_sharp_cmd_init1), mipi_sharp_cmd_init1},
    {DTYPE_GEN_LWRITE, 1, 0, 0, 0, sizeof(mipi_sharp_cmd_init2), mipi_sharp_cmd_init2},
    {DTYPE_GEN_LWRITE, 1, 0, 0, 0, sizeof(mipi_sharp_cmd_init3), mipi_sharp_cmd_init3},
    {DTYPE_GEN_WRITE2, 1, 0, 0, 0, sizeof(mipi_sharp_cmd_panel), mipi_sharp_cmd_panel}
};
static struct dsi_cmd_desc mipi_sharp_cmds_timing1[] = {
    {DTYPE_GEN_LWRITE, 1, 0, 0, 0, sizeof(mipi_sharp_cmd_timing1), mipi_sharp_cmd_timing1},
    {DTYPE_GEN_LWRITE, 1, 0, 0, 0, sizeof(mipi_sharp_cmd_timing2), mipi_sharp_cmd_timing2},
    {DTYPE_GEN_LWRITE, 1, 0, 0, 0, sizeof(mipi_sharp_cmd_timing3), mipi_sharp_cmd_timing3}
};
static struct dsi_cmd_desc mipi_sharp_cmds_timing2_clk[] = {
    {DTYPE_GEN_LWRITE, 1, 0, 0, 0, sizeof(mipi_sharp_cmd_timing4_clk), mipi_sharp_cmd_timing4_clk},
    {DTYPE_GEN_LWRITE, 1, 0, 0, 0, sizeof(mipi_sharp_cmd_timing5_clk), mipi_sharp_cmd_timing5_clk}
};
static struct dsi_cmd_desc mipi_sharp_cmds_timing2_osc[] = {
    {DTYPE_GEN_LWRITE, 1, 0, 0, 0, sizeof(mipi_sharp_cmd_timing4_osc), mipi_sharp_cmd_timing4_osc},
    {DTYPE_GEN_LWRITE, 1, 0, 0, 0, sizeof(mipi_sharp_cmd_timing5_osc), mipi_sharp_cmd_timing5_osc}
};
static struct dsi_cmd_desc mipi_sharp_cmds_gamma[] = {
    {DTYPE_GEN_LWRITE, 1, 0, 0, 0, sizeof(mipi_sharp_cmd_gamma_a), mipi_sharp_cmd_gamma_a},
    {DTYPE_GEN_LWRITE, 1, 0, 0, 0, sizeof(mipi_sharp_cmd_gamma_b), mipi_sharp_cmd_gamma_b},
    {DTYPE_GEN_LWRITE, 1, 0, 0, 0, sizeof(mipi_sharp_cmd_gamma_c), mipi_sharp_cmd_gamma_c}
};
static struct dsi_cmd_desc mipi_sharp_cmds_power1[] = {
    {DTYPE_GEN_LWRITE, 1, 0, 0, 0, sizeof(mipi_sharp_cmd_power1), mipi_sharp_cmd_power1},
    {DTYPE_GEN_WRITE2, 1, 0, 0, 0, sizeof(mipi_sharp_cmd_power2), mipi_sharp_cmd_power2},
    {DTYPE_GEN_LWRITE, 1, 0, 0, 0, sizeof(mipi_sharp_cmd_power3), mipi_sharp_cmd_power3}
};
static struct dsi_cmd_desc mipi_sharp_cmds_power2[] = {
    {DTYPE_GEN_WRITE2, 1, 0, 0, 0, sizeof(mipi_sharp_cmd_power4), mipi_sharp_cmd_power4},
    {DTYPE_GEN_LWRITE, 1, 0, 0, 0, sizeof(mipi_sharp_cmd_power5), mipi_sharp_cmd_power5},
    {DTYPE_GEN_LWRITE, 1, 0, 0, 0, sizeof(mipi_sharp_cmd_setram), mipi_sharp_cmd_setram}
};
static struct dsi_cmd_desc mipi_sharp_cmds_display_on[] = {
    {DTYPE_DCS_LWRITE, 1, 0, 0, 0, sizeof(mipi_sharp_cmd_column_addr), mipi_sharp_cmd_column_addr},
    {DTYPE_DCS_LWRITE, 1, 0, 0, 0, sizeof(mipi_sharp_cmd_page_addr), mipi_sharp_cmd_page_addr},
    {DTYPE_DCS_WRITE1, 1, 0, 0, 0, sizeof(mipi_sharp_cmd_tear_on), mipi_sharp_cmd_tear_on},
    {DTYPE_DCS_WRITE, 1, 0, 0, 0, sizeof(mipi_sharp_cmd_display_on), mipi_sharp_cmd_display_on}
};
static struct dsi_cmd_desc mipi_sharp_cmds_display_on_start[] = {
    {DTYPE_DCS_WRITE, 1, 0, 0, 60, sizeof(mipi_sharp_cmd_exit_sleep), mipi_sharp_cmd_exit_sleep}
};

static char mipi_sharp_cmd_disp_off[2] = {0x28, 0x00};
static char mipi_sharp_cmd_sleep_on[2] = {0x10, 0x00};
static char mipi_sharp_cmd_standby[2] = {0xB1, 0x01};
static struct dsi_cmd_desc mipi_sharp_cmds_display_off[] = {
    {DTYPE_DCS_WRITE, 1, 0, 0, 20, sizeof(mipi_sharp_cmd_disp_off), mipi_sharp_cmd_disp_off},
    {DTYPE_DCS_WRITE, 1, 0, 0, 60, sizeof(mipi_sharp_cmd_sleep_on), mipi_sharp_cmd_sleep_on},
    {DTYPE_GEN_WRITE2, 1, 0, 0, 20, sizeof(mipi_sharp_cmd_standby), mipi_sharp_cmd_standby}
};

static struct dsi_cmd_desc mipi_sharp_cmds_cabc_init_set_pwm_clk[] = {
    {DTYPE_GEN_LWRITE, 1, 0, 0, 0, sizeof(mipi_sharp_cmd_cabc_init_set_pwm_clk), mipi_sharp_cmd_cabc_init_set_pwm_clk}
};
static struct dsi_cmd_desc mipi_sharp_cmds_cabc_init_set_pwm_osc[] = {
    {DTYPE_GEN_LWRITE, 1, 0, 0, 0, sizeof(mipi_sharp_cmd_cabc_init_set_pwm_osc), mipi_sharp_cmd_cabc_init_set_pwm_osc}
};
static struct dsi_cmd_desc mipi_sharp_cmds_cabc_init_set_para[] = {
    {DTYPE_DCS_LWRITE, 1, 0, 0, 0, sizeof(mipi_sharp_cmd_cabc_init_set_param), mipi_sharp_cmd_cabc_init_set_param}
};
static struct dsi_cmd_desc mipi_sharp_cmds_cabc_init_set_flicker[] = {
    {DTYPE_GEN_LWRITE, 1, 0, 0, 0, sizeof(mipi_sharp_cmd_cabc_init_set_flicker), mipi_sharp_cmd_cabc_init_set_flicker}
};
static struct dsi_cmd_desc mipi_sharp_cmds_cabc_init_set_connector[] = {
    {DTYPE_DCS_WRITE1, 1, 0, 0, 0, sizeof(mipi_sharp_cmd_cabc_init_set_connector), mipi_sharp_cmd_cabc_init_set_connector}
};
static struct dsi_cmd_desc mipi_sharp_cmds_cabc_init_unset_connector[] = {
    {DTYPE_DCS_WRITE1, 1, 0, 0, 0, sizeof(mipi_sharp_cmd_cabc_init_unset_connector), mipi_sharp_cmd_cabc_init_unset_connector}
};

static struct dsi_cmd_desc mipi_sharp_cmds_cabc_indoor_set_image_exrate_lut0[] = {
    {DTYPE_GEN_LWRITE, 1, 0, 0, 0, sizeof(mipi_sharp_cmd_cabc_indoor_set_image_exrate_lut0), mipi_sharp_cmd_cabc_indoor_set_image_exrate_lut0}
};
static struct dsi_cmd_desc mipi_sharp_cmds_cabc_indoor_set_on[] = {
    {DTYPE_DCS_WRITE1, 1, 0, 0, 0, sizeof(mipi_sharp_cmd_cabc_indoor_set_on), mipi_sharp_cmd_cabc_indoor_set_on}
};

static struct dsi_cmd_desc mipi_sharp_cmds_cabc_outdoor_set_image_exrate_lut1[] = {
    {DTYPE_GEN_LWRITE, 1, 0, 0, 0, sizeof(mipi_sharp_cmd_cabc_outdoor_set_image_exrate_lut1), mipi_sharp_cmd_cabc_outdoor_set_image_exrate_lut1}
};
static struct dsi_cmd_desc mipi_sharp_cmds_cabc_outdoor_set_image_exrate_lut2[] = {
    {DTYPE_GEN_LWRITE, 1, 0, 0, 0, sizeof(mipi_sharp_cmd_cabc_outdoor_set_image_exrate_lut2), mipi_sharp_cmd_cabc_outdoor_set_image_exrate_lut2}
};
static struct dsi_cmd_desc mipi_sharp_cmds_cabc_outdoor_set_image_exrate_lut3[] = {
    {DTYPE_GEN_LWRITE, 1, 0, 0, 0, sizeof(mipi_sharp_cmd_cabc_outdoor_set_image_exrate_lut3), mipi_sharp_cmd_cabc_outdoor_set_image_exrate_lut3}
};
static struct dsi_cmd_desc mipi_sharp_cmds_cabc_outdoor_set_image_exrate_lut4[] = {
    {DTYPE_GEN_LWRITE, 1, 0, 0, 0, sizeof(mipi_sharp_cmd_cabc_outdoor_set_image_exrate_lut4), mipi_sharp_cmd_cabc_outdoor_set_image_exrate_lut4}
};
static struct dsi_cmd_desc mipi_sharp_cmds_cabc_outdoor_set_image_exrate_lut5[] = {
    {DTYPE_GEN_LWRITE, 1, 0, 0, 0, sizeof(mipi_sharp_cmd_cabc_outdoor_set_image_exrate_lut5), mipi_sharp_cmd_cabc_outdoor_set_image_exrate_lut5}
};
static struct dsi_cmd_desc mipi_sharp_cmds_cabc_outdoor_set_on[] = {
    {DTYPE_DCS_WRITE1, 1, 0, 0, 0, sizeof(mipi_sharp_cmd_cabc_outdoor_set_on), mipi_sharp_cmd_cabc_outdoor_set_on}
};

static struct dsi_cmd_desc mipi_sharp_cmds_cabc_off_set_image_exrate_lut0[] = {
    {DTYPE_GEN_LWRITE, 1, 0, 0, 0, sizeof(mipi_sharp_cmd_cabc_off_set_image_exrate_lut0), mipi_sharp_cmd_cabc_off_set_image_exrate_lut0}
};
static struct dsi_cmd_desc mipi_sharp_cmds_cabc_off_set_on[] = {
    {DTYPE_DCS_WRITE1, 1, 0, 0, 0, sizeof(mipi_sharp_cmd_cabc_off_set_on), mipi_sharp_cmd_cabc_off_set_on}
};

static struct dsi_cmd_desc mipi_sharp_cmds_cabc_transition_set_image_exrate_lut0[] = {
    {DTYPE_GEN_LWRITE, 1, 0, 0, 0, sizeof(mipi_sharp_cmd_cabc_transition_set_image_exrate_lut0), mipi_sharp_cmd_cabc_transition_set_image_exrate_lut0}
};
static struct dsi_cmd_desc mipi_sharp_cmds_cabc_transition_set_image_exrate_lut1[] = {
    {DTYPE_GEN_LWRITE, 1, 0, 0, 0, sizeof(mipi_sharp_cmd_cabc_transition_set_image_exrate_lut1), mipi_sharp_cmd_cabc_transition_set_image_exrate_lut1}
};
static struct dsi_cmd_desc mipi_sharp_cmds_cabc_transition_set_image_exrate_lut2[] = {
    {DTYPE_GEN_LWRITE, 1, 0, 0, 0, sizeof(mipi_sharp_cmd_cabc_transition_set_image_exrate_lut2), mipi_sharp_cmd_cabc_transition_set_image_exrate_lut2}
};
static struct dsi_cmd_desc mipi_sharp_cmds_cabc_transition_set_image_exrate_lut3[] = {
    {DTYPE_GEN_LWRITE, 1, 0, 0, 0, sizeof(mipi_sharp_cmd_cabc_transition_set_image_exrate_lut3), mipi_sharp_cmd_cabc_transition_set_image_exrate_lut3}
};
static struct dsi_cmd_desc mipi_sharp_cmds_cabc_transition_set_image_exrate_lut4[] = {
    {DTYPE_GEN_LWRITE, 1, 0, 0, 0, sizeof(mipi_sharp_cmd_cabc_transition_set_image_exrate_lut4), mipi_sharp_cmd_cabc_transition_set_image_exrate_lut4}
};
static struct dsi_cmd_desc mipi_sharp_cmds_cabc_transition_set_image_exrate_lut5[] = {
    {DTYPE_GEN_LWRITE, 1, 0, 0, 0, sizeof(mipi_sharp_cmd_cabc_transition_set_image_exrate_lut5), mipi_sharp_cmd_cabc_transition_set_image_exrate_lut5}
};

static struct dsi_cmd_desc mipi_sharp_cmds_cabc_lut0_gamma[] = {
    {DTYPE_GEN_LWRITE, 1, 0, 0, 0, sizeof(mipi_sharp_cmd_cabc_lut0_gamma_a), mipi_sharp_cmd_cabc_lut0_gamma_a},
    {DTYPE_GEN_LWRITE, 1, 0, 0, 0, sizeof(mipi_sharp_cmd_cabc_lut0_gamma_b), mipi_sharp_cmd_cabc_lut0_gamma_b},
    {DTYPE_GEN_LWRITE, 1, 0, 0, 0, sizeof(mipi_sharp_cmd_cabc_lut0_gamma_c), mipi_sharp_cmd_cabc_lut0_gamma_c}
};
static struct dsi_cmd_desc mipi_sharp_cmds_cabc_lut1_gamma[] = {
    {DTYPE_GEN_LWRITE, 1, 0, 0, 0, sizeof(mipi_sharp_cmd_cabc_lut1_gamma_a), mipi_sharp_cmd_cabc_lut1_gamma_a},
    {DTYPE_GEN_LWRITE, 1, 0, 0, 0, sizeof(mipi_sharp_cmd_cabc_lut1_gamma_b), mipi_sharp_cmd_cabc_lut1_gamma_b},
    {DTYPE_GEN_LWRITE, 1, 0, 0, 0, sizeof(mipi_sharp_cmd_cabc_lut1_gamma_c), mipi_sharp_cmd_cabc_lut1_gamma_c}
};
static struct dsi_cmd_desc mipi_sharp_cmds_cabc_lut2_gamma[] = {
    {DTYPE_GEN_LWRITE, 1, 0, 0, 0, sizeof(mipi_sharp_cmd_cabc_lut2_gamma_a), mipi_sharp_cmd_cabc_lut2_gamma_a},
    {DTYPE_GEN_LWRITE, 1, 0, 0, 0, sizeof(mipi_sharp_cmd_cabc_lut2_gamma_b), mipi_sharp_cmd_cabc_lut2_gamma_b},
    {DTYPE_GEN_LWRITE, 1, 0, 0, 0, sizeof(mipi_sharp_cmd_cabc_lut2_gamma_c), mipi_sharp_cmd_cabc_lut2_gamma_c}
};
static struct dsi_cmd_desc mipi_sharp_cmds_cabc_lut3_gamma[] = {
    {DTYPE_GEN_LWRITE, 1, 0, 0, 0, sizeof(mipi_sharp_cmd_cabc_lut3_gamma_a), mipi_sharp_cmd_cabc_lut3_gamma_a},
    {DTYPE_GEN_LWRITE, 1, 0, 0, 0, sizeof(mipi_sharp_cmd_cabc_lut3_gamma_b), mipi_sharp_cmd_cabc_lut3_gamma_b},
    {DTYPE_GEN_LWRITE, 1, 0, 0, 0, sizeof(mipi_sharp_cmd_cabc_lut3_gamma_c), mipi_sharp_cmd_cabc_lut3_gamma_c}
};
static struct dsi_cmd_desc mipi_sharp_cmds_cabc_lut4_gamma[] = {
    {DTYPE_GEN_LWRITE, 1, 0, 0, 0, sizeof(mipi_sharp_cmd_cabc_lut4_gamma_a), mipi_sharp_cmd_cabc_lut4_gamma_a},
    {DTYPE_GEN_LWRITE, 1, 0, 0, 0, sizeof(mipi_sharp_cmd_cabc_lut4_gamma_b), mipi_sharp_cmd_cabc_lut4_gamma_b},
    {DTYPE_GEN_LWRITE, 1, 0, 0, 0, sizeof(mipi_sharp_cmd_cabc_lut4_gamma_c), mipi_sharp_cmd_cabc_lut4_gamma_c}
};
static struct dsi_cmd_desc mipi_sharp_cmds_cabc_lut5_gamma[] = {
    {DTYPE_GEN_LWRITE, 1, 0, 0, 0, sizeof(mipi_sharp_cmd_cabc_lut5_gamma_a), mipi_sharp_cmd_cabc_lut5_gamma_a},
    {DTYPE_GEN_LWRITE, 1, 0, 0, 0, sizeof(mipi_sharp_cmd_cabc_lut5_gamma_b), mipi_sharp_cmd_cabc_lut5_gamma_b},
    {DTYPE_GEN_LWRITE, 1, 0, 0, 0, sizeof(mipi_sharp_cmd_cabc_lut5_gamma_c), mipi_sharp_cmd_cabc_lut5_gamma_c}
};

static int mipi_sharp_kite_lcd_on(struct platform_device *pdev);
static int mipi_sharp_kite_lcd_off(struct platform_device *pdev);
static void mipi_sharp_kite_lcd_set_backlight(struct msm_fb_data_type *mfd);

#if defined(CONFIG_SHDISP_USE_CABC)
#if !defined(CONFIG_MACH_TOR)
static int mipi_sharp_cabc_lcd_backlight_on(void);
#endif
static int mipi_sharp_kite_cabc_pre_disp_on_ctrl(struct msm_fb_data_type *mfd, struct mipi_panel_info *mipi);
static int mipi_sharp_cabc_disp_on_ctrl(struct msm_fb_data_type *mfd, struct mipi_panel_info *mipi);
#endif
static int mipi_sharp_cmd_sqe_cabc_init(struct msm_fb_data_type *mfd, struct mipi_panel_info *mipi);
static int mipi_sharp_cmd_sqe_cabc_indoor_on(struct msm_fb_data_type *mfd, struct mipi_panel_info *mipi);
static int mipi_sharp_cmd_sqe_cabc_outdoor_on(struct msm_fb_data_type *mfd, struct mipi_panel_info *mipi, int lut_level);
static int mipi_sharp_cmd_sqe_cabc_off(struct msm_fb_data_type *mfd, struct mipi_panel_info *mipi, int wait_on, int pwm_disable);
static int mipi_sharp_cmd_sqe_cabc_outdoor_move(struct msm_fb_data_type *mfd, struct mipi_panel_info *mipi, int lut_level);
static int mipi_sharp_kite_cabc_ctrl(struct platform_device *pdev);

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
    char cmd_buf[12];
    unsigned short alpha;
    
    memcpy(&cmd_buf[0], mipi_sharp_cmd_vcom, 8);
    
    alpha = shdisp_api_get_alpha();
    
    cmd_buf[4]  = (unsigned char) ((alpha >> 8) & 0x01);
    cmd_buf[5]  = (unsigned char) (alpha & 0xFF);
    cmd_buf[6]  = (unsigned char) ((alpha >> 8) & 0x01);
    cmd_buf[7]  = (unsigned char) (alpha & 0xFF);
    
    cmd[0].dtype = DTYPE_GEN_LWRITE;
    cmd[0].last = 0x01;
    cmd[0].vc = 0x00;
    cmd[0].ack = 0x00;
    cmd[0].wait = 0x00;
    cmd[0].dlen = 8;
    cmd[0].payload = cmd_buf;
    
    mipi_dsi_cmds_tx(&sharp_tx_buf, cmd, ARRAY_SIZE(cmd));
    
    return 0;
}

static int mipi_sharp_cmd_lcd_on(struct msm_fb_data_type *mfd, struct mipi_panel_info *mipi)
{
    char device_id = 0x00;
    int clk_check;
    
    if(shdisp_api_get_upper_unit() != SHDISP_RESULT_SUCCESS) {
        return -ENODEV;
    }
    
    mipi_dsi_cmds_tx(&sharp_tx_buf, mipi_sharp_cmds_protect,
                     ARRAY_SIZE(mipi_sharp_cmds_protect));
    device_id = mipi_sharp_manufacture_id(mfd);
    if((device_id == 0x00) || (device_id == 0x01)) {
        mipi_dsi_cmds_tx(&sharp_tx_buf, mipi_sharp_cmds_testmode1,
                     ARRAY_SIZE(mipi_sharp_cmds_testmode1));
    } else {
        mipi_dsi_cmds_tx(&sharp_tx_buf, mipi_sharp_cmds_testmode3,
                     ARRAY_SIZE(mipi_sharp_cmds_testmode3));
    }
    mipi_dsi_cmds_tx(&sharp_tx_buf, mipi_sharp_cmds_init,
                     ARRAY_SIZE(mipi_sharp_cmds_init));
    mipi_dsi_cmds_tx(&sharp_tx_buf, mipi_sharp_cmds_timing1,
                     ARRAY_SIZE(mipi_sharp_cmds_timing1));
    clk_check = shdisp_api_get_clock_info();
    if (clk_check == 0) {
        mipi_dsi_cmds_tx(&sharp_tx_buf, mipi_sharp_cmds_timing2_clk,
                         ARRAY_SIZE(mipi_sharp_cmds_timing2_clk));
    } else {
        mipi_dsi_cmds_tx(&sharp_tx_buf, mipi_sharp_cmds_timing2_osc,
                         ARRAY_SIZE(mipi_sharp_cmds_timing2_osc));
    }
    mipi_dsi_cmds_tx(&sharp_tx_buf, mipi_sharp_cmds_gamma,
                     ARRAY_SIZE(mipi_sharp_cmds_gamma));
    mipi_dsi_cmds_tx(&sharp_tx_buf, mipi_sharp_cmds_power1,
                     ARRAY_SIZE(mipi_sharp_cmds_power1));
    mipi_sharp_tx_vcom_write(mfd);
    mipi_dsi_cmds_tx(&sharp_tx_buf, mipi_sharp_cmds_power2,
                     ARRAY_SIZE(mipi_sharp_cmds_power2));
    mipi_dsi_cmds_tx(&sharp_tx_buf, mipi_sharp_cmds_display_on,
                     ARRAY_SIZE(mipi_sharp_cmds_display_on));

    mipi_dsi_cmd_bta_sw_trigger();
    

#if defined(CONFIG_SHDISP_USE_CABC)
    mipi_sharp_kite_cabc_pre_disp_on_ctrl(mfd, mipi);
#endif
    return 0;
}


#if defined(CONFIG_SHDISP_USE_CABC) && !defined(CONFIG_MACH_TOR)
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
    return 0;
}

static int mipi_sharp_kite_lcd_on(struct platform_device *pdev)
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

static int mipi_sharp_kite_lcd_off(struct platform_device *pdev)
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

static int mipi_sharp_kite_lcd_start_display(struct msm_fb_data_type *mfd)
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
		mdp4_dsi_cmd_busy_wait();
		ret = mipi_sharp_cmd_lcd_display_on(mfd, mipi);
		mutex_unlock(&mfd->dma->ov_mutex);
	}
	return ret;
}

static void mipi_sharp_kite_lcd_set_backlight(struct msm_fb_data_type *mfd)
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
static int mipi_sharp_kite_cabc_pre_disp_on_ctrl(struct msm_fb_data_type *mfd, struct mipi_panel_info *mipi)
{
#ifndef CONFIG_MACH_TOR
	mipi_sharp_cabc_lcd_backlight_on();
#endif

    mipi_sharp_cmd_sqe_cabc_off(mfd, mipi, 0, 1);

    shdisp_api_init_cabc_state(SHDISP_MAIN_DISP_CABC_MODE_OFF, SHDISP_MAIN_DISP_CABC_LUT0);

    return 0;
}
#endif


#if defined(CONFIG_SHDISP_USE_CABC)
static int mipi_sharp_cabc_disp_on_ctrl(struct msm_fb_data_type *mfd, struct mipi_panel_info *mipi)
{
    struct shdisp_check_cabc_val value;
#ifndef CONFIG_MACH_TOR
	mipi_sharp_cabc_lcd_backlight_on();
#endif
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



static int mipi_sharp_cmd_sqe_cabc_init(struct msm_fb_data_type *mfd, struct mipi_panel_info *mipi)
{
    int clk_check;
    
    clk_check = shdisp_api_get_clock_info();
    if (clk_check == 0) {
        mipi_dsi_cmds_tx(&sharp_tx_buf, mipi_sharp_cmds_cabc_init_set_pwm_clk,
                         ARRAY_SIZE(mipi_sharp_cmds_cabc_init_set_pwm_clk));
    } else {
        mipi_dsi_cmds_tx(&sharp_tx_buf, mipi_sharp_cmds_cabc_init_set_pwm_osc,
                         ARRAY_SIZE(mipi_sharp_cmds_cabc_init_set_pwm_osc));
    }
    mipi_dsi_cmds_tx(&sharp_tx_buf, mipi_sharp_cmds_cabc_init_set_para,
                     ARRAY_SIZE(mipi_sharp_cmds_cabc_init_set_para));
    mipi_dsi_cmds_tx(&sharp_tx_buf, mipi_sharp_cmds_cabc_init_set_flicker,
                     ARRAY_SIZE(mipi_sharp_cmds_cabc_init_set_flicker));
    mipi_dsi_cmds_tx(&sharp_tx_buf, mipi_sharp_cmds_cabc_init_set_connector,
                     ARRAY_SIZE(mipi_sharp_cmds_cabc_init_set_connector));

    mipi_sharp_delay_us(20000);

    shdisp_api_pwm_enable();

    return 0;
}

static int mipi_sharp_cmd_sqe_cabc_indoor_on(struct msm_fb_data_type *mfd, struct mipi_panel_info *mipi)
{
    shdisp_api_pwm_enable();

    mipi_dsi_cmds_tx(&sharp_tx_buf, mipi_sharp_cmds_cabc_indoor_set_image_exrate_lut0,
                     ARRAY_SIZE(mipi_sharp_cmds_cabc_indoor_set_image_exrate_lut0));
    mipi_dsi_cmds_tx(&sharp_tx_buf, mipi_sharp_cmds_cabc_indoor_set_on,
                     ARRAY_SIZE(mipi_sharp_cmds_cabc_indoor_set_on));

    return 0;
}


static int mipi_sharp_cmd_sqe_cabc_outdoor_on(struct msm_fb_data_type *mfd, struct mipi_panel_info *mipi, int lut_level)
{
    shdisp_api_pwm_enable();

    switch(lut_level)
    {
    case SHDISP_MAIN_DISP_CABC_LUT1:
        mipi_dsi_cmds_tx(&sharp_tx_buf, mipi_sharp_cmds_cabc_outdoor_set_image_exrate_lut1,
                         ARRAY_SIZE(mipi_sharp_cmds_cabc_outdoor_set_image_exrate_lut1));
        mipi_dsi_cmds_tx(&sharp_tx_buf, mipi_sharp_cmds_cabc_outdoor_set_on,
                         ARRAY_SIZE(mipi_sharp_cmds_cabc_outdoor_set_on));
        mipi_dsi_cmds_tx(&sharp_tx_buf, mipi_sharp_cmds_cabc_lut1_gamma,
                         ARRAY_SIZE(mipi_sharp_cmds_cabc_lut1_gamma));
        break;
    case SHDISP_MAIN_DISP_CABC_LUT2:
        mipi_dsi_cmds_tx(&sharp_tx_buf, mipi_sharp_cmds_cabc_outdoor_set_image_exrate_lut2,
                         ARRAY_SIZE(mipi_sharp_cmds_cabc_outdoor_set_image_exrate_lut2));
        mipi_dsi_cmds_tx(&sharp_tx_buf, mipi_sharp_cmds_cabc_outdoor_set_on,
                         ARRAY_SIZE(mipi_sharp_cmds_cabc_outdoor_set_on));
        mipi_dsi_cmds_tx(&sharp_tx_buf, mipi_sharp_cmds_cabc_lut2_gamma,
                         ARRAY_SIZE(mipi_sharp_cmds_cabc_lut2_gamma));
        break;
    case SHDISP_MAIN_DISP_CABC_LUT3:
        mipi_dsi_cmds_tx(&sharp_tx_buf, mipi_sharp_cmds_cabc_outdoor_set_image_exrate_lut3,
                         ARRAY_SIZE(mipi_sharp_cmds_cabc_outdoor_set_image_exrate_lut3));
        mipi_dsi_cmds_tx(&sharp_tx_buf, mipi_sharp_cmds_cabc_outdoor_set_on,
                         ARRAY_SIZE(mipi_sharp_cmds_cabc_outdoor_set_on));
        mipi_dsi_cmds_tx(&sharp_tx_buf, mipi_sharp_cmds_cabc_lut3_gamma,
                         ARRAY_SIZE(mipi_sharp_cmds_cabc_lut3_gamma));
        break;
    case SHDISP_MAIN_DISP_CABC_LUT4:
        mipi_dsi_cmds_tx(&sharp_tx_buf, mipi_sharp_cmds_cabc_outdoor_set_image_exrate_lut4,
                         ARRAY_SIZE(mipi_sharp_cmds_cabc_outdoor_set_image_exrate_lut4));
        mipi_dsi_cmds_tx(&sharp_tx_buf, mipi_sharp_cmds_cabc_outdoor_set_on,
                         ARRAY_SIZE(mipi_sharp_cmds_cabc_outdoor_set_on));
        mipi_dsi_cmds_tx(&sharp_tx_buf, mipi_sharp_cmds_cabc_lut4_gamma,
                         ARRAY_SIZE(mipi_sharp_cmds_cabc_lut4_gamma));
        break;
    case SHDISP_MAIN_DISP_CABC_LUT5:
        mipi_dsi_cmds_tx(&sharp_tx_buf, mipi_sharp_cmds_cabc_outdoor_set_image_exrate_lut5,
                         ARRAY_SIZE(mipi_sharp_cmds_cabc_outdoor_set_image_exrate_lut5));
        mipi_dsi_cmds_tx(&sharp_tx_buf, mipi_sharp_cmds_cabc_outdoor_set_on,
                         ARRAY_SIZE(mipi_sharp_cmds_cabc_outdoor_set_on));
        mipi_dsi_cmds_tx(&sharp_tx_buf, mipi_sharp_cmds_cabc_lut5_gamma,
                         ARRAY_SIZE(mipi_sharp_cmds_cabc_lut5_gamma));
        break;
	}

    return 0;
}


static int mipi_sharp_cmd_sqe_cabc_off(struct msm_fb_data_type *mfd, struct mipi_panel_info *mipi, int wait_on, int pwm_disable)
{

    mipi_dsi_cmds_tx(&sharp_tx_buf, mipi_sharp_cmds_cabc_lut0_gamma,
                     ARRAY_SIZE(mipi_sharp_cmds_cabc_lut0_gamma));
    
    if(wait_on == 0){
        mipi_dsi_cmds_tx(&sharp_tx_buf, mipi_sharp_cmds_cabc_off_set_image_exrate_lut0,
                 ARRAY_SIZE(mipi_sharp_cmds_cabc_off_set_image_exrate_lut0));
    }
    
    mipi_dsi_cmds_tx(&sharp_tx_buf, mipi_sharp_cmds_cabc_off_set_on,
                     ARRAY_SIZE(mipi_sharp_cmds_cabc_off_set_on));
    

    if(pwm_disable == 1) {
    	shdisp_api_pwm_disable();

    	mipi_dsi_cmds_tx(&sharp_tx_buf, mipi_sharp_cmds_cabc_init_unset_connector,
                     ARRAY_SIZE(mipi_sharp_cmds_cabc_init_unset_connector));
    	mipi_sharp_delay_us(20000);
    }

    if(wait_on == 1)
        usleep(300 * 1000);

    return 0;
}

static int mipi_sharp_cmd_sqe_cabc_outdoor_move(struct msm_fb_data_type *mfd, struct mipi_panel_info *mipi, int lut_level)
{
    switch(lut_level)
    {
    case SHDISP_MAIN_DISP_CABC_LUT0:
        mipi_dsi_cmds_tx(&sharp_tx_buf, mipi_sharp_cmds_cabc_transition_set_image_exrate_lut0,
                         ARRAY_SIZE(mipi_sharp_cmds_cabc_transition_set_image_exrate_lut0));
        mipi_dsi_cmds_tx(&sharp_tx_buf, mipi_sharp_cmds_cabc_lut0_gamma,
                         ARRAY_SIZE(mipi_sharp_cmds_cabc_lut0_gamma));
        break;
    case SHDISP_MAIN_DISP_CABC_LUT1:
        mipi_dsi_cmds_tx(&sharp_tx_buf, mipi_sharp_cmds_cabc_transition_set_image_exrate_lut1,
                         ARRAY_SIZE(mipi_sharp_cmds_cabc_transition_set_image_exrate_lut1));
        mipi_dsi_cmds_tx(&sharp_tx_buf, mipi_sharp_cmds_cabc_lut1_gamma,
                         ARRAY_SIZE(mipi_sharp_cmds_cabc_lut1_gamma));
        break;
    case SHDISP_MAIN_DISP_CABC_LUT2:
        mipi_dsi_cmds_tx(&sharp_tx_buf, mipi_sharp_cmds_cabc_transition_set_image_exrate_lut2,
                         ARRAY_SIZE(mipi_sharp_cmds_cabc_transition_set_image_exrate_lut2));
        mipi_dsi_cmds_tx(&sharp_tx_buf, mipi_sharp_cmds_cabc_lut2_gamma,
                         ARRAY_SIZE(mipi_sharp_cmds_cabc_lut2_gamma));
        break;
    case SHDISP_MAIN_DISP_CABC_LUT3:
        mipi_dsi_cmds_tx(&sharp_tx_buf, mipi_sharp_cmds_cabc_transition_set_image_exrate_lut3,
                         ARRAY_SIZE(mipi_sharp_cmds_cabc_transition_set_image_exrate_lut3));
        mipi_dsi_cmds_tx(&sharp_tx_buf, mipi_sharp_cmds_cabc_lut3_gamma,
                         ARRAY_SIZE(mipi_sharp_cmds_cabc_lut3_gamma));
        break;
    case SHDISP_MAIN_DISP_CABC_LUT4:
        mipi_dsi_cmds_tx(&sharp_tx_buf, mipi_sharp_cmds_cabc_transition_set_image_exrate_lut4,
                         ARRAY_SIZE(mipi_sharp_cmds_cabc_transition_set_image_exrate_lut4));
        mipi_dsi_cmds_tx(&sharp_tx_buf, mipi_sharp_cmds_cabc_lut4_gamma,
                         ARRAY_SIZE(mipi_sharp_cmds_cabc_lut4_gamma));
        break;
    case SHDISP_MAIN_DISP_CABC_LUT5:
        mipi_dsi_cmds_tx(&sharp_tx_buf, mipi_sharp_cmds_cabc_transition_set_image_exrate_lut5,
                         ARRAY_SIZE(mipi_sharp_cmds_cabc_transition_set_image_exrate_lut5));
        mipi_dsi_cmds_tx(&sharp_tx_buf, mipi_sharp_cmds_cabc_lut5_gamma,
                         ARRAY_SIZE(mipi_sharp_cmds_cabc_lut5_gamma));
        break;
	}

    return 0;
}

#if defined(CONFIG_SHDISP_USE_CABC)
static int mipi_sharp_kite_cabc_ctrl(struct platform_device *pdev)
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


static int __devinit mipi_sharp_kite_lcd_probe(struct platform_device *pdev)
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
	.probe  = mipi_sharp_kite_lcd_probe,
	.driver = {
		.name   = "mipi_sharp_kite",
	},
};

static struct msm_fb_panel_data sharp_kite_panel_data = {
	.on		= mipi_sharp_kite_lcd_on,
	.off	= mipi_sharp_kite_lcd_off,
	.set_backlight = mipi_sharp_kite_lcd_set_backlight,
	.start_display = mipi_sharp_kite_lcd_start_display,

	.cabc_init			= mipi_sharp_cmd_sqe_cabc_init,
	.cabc_indoor_on		= mipi_sharp_cmd_sqe_cabc_indoor_on,
	.cabc_outdoor_on	= mipi_sharp_cmd_sqe_cabc_outdoor_on,
	.cabc_off			= mipi_sharp_cmd_sqe_cabc_off,
	.cabc_outdoor_move	= mipi_sharp_cmd_sqe_cabc_outdoor_move,
	.cabc_ctrl			= mipi_sharp_kite_cabc_ctrl,
};

int mipi_sharp_kite_device_register(struct msm_panel_info *pinfo,
					u32 channel, u32 panel)
{
	struct platform_device *pdev = NULL;
	int ret;

	
	if ((channel >= 3) || ch_used[channel])
		return -ENODEV;

	
	ch_used[channel] = TRUE;

	
	pdev = platform_device_alloc("mipi_sharp_kite", (panel << 8)|channel);
	if (!pdev)
		return -ENOMEM;

	sharp_kite_panel_data.panel_info = *pinfo;

	
	ret = platform_device_add_data(pdev, &sharp_kite_panel_data,
		sizeof(sharp_kite_panel_data));
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

int mipi_sharp_kite_diag_write_reg(unsigned char addr, unsigned char *write_data, unsigned char size)
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

int mipi_sharp_kite_diag_read_reg(unsigned char addr, unsigned char *read_data, unsigned char size)
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

static int __init mipi_sharp_kite_lcd_init(void)
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
module_init(mipi_sharp_kite_lcd_init);

static void mipi_sharp_kite_lcd_exit(void)
{
    platform_driver_unregister(&this_driver);
    return;
}
module_exit(mipi_sharp_kite_lcd_exit);
