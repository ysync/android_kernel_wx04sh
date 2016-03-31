/* drivers/sharp/shdisp/data/shdisp_bl69y6_data_ars.h  (Display Driver)
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

#ifndef SHDISP_BL69Y6_DATA_ARS_H
#define SHDISP_BL69Y6_DATA_ARS_H

/* ------------------------------------------------------------------------- */
/* INCLUDE FILES                                                             */
/* ------------------------------------------------------------------------- */

#include "../shdisp_bl69y6.h"

/* ------------------------------------------------------------------------- */
/* MACROS                                                                    */
/* ------------------------------------------------------------------------- */

#define SHDISP_BKL_FIX_TBL_NUM          23
#define SHDISP_BKL_AUTO_TBL_NUM         16
#define SHDISP_TRI_LED_COLOR_TBL_NUM    8
#define NUM_SHDISP_BKL_TBL_MODE         (SHDISP_BKL_TBL_MODE_CHARGE + 1)

#define SHDISP_INT_ENABLE_GFAC          0x002C0308
#define SHDISP_LUX_CHANGE_LEVEL1        0x0C
#define SHDISP_LUX_CHANGE_LEVEL2        0x01

#define CABC_LUX_LEVEL_LUT0_1           0x01
#define CABC_LUX_LEVEL_LUT1_2           0x02
#define CABC_LUX_LEVEL_LUT2_3           0x03
#define CABC_LUX_LEVEL_LUT3_4           0x04
#define CABC_LUX_LEVEL_LUT4_5           0x05


/* ------------------------------------------------------------------------- */
/* MACROS(Register Value)                                                    */
/* ------------------------------------------------------------------------- */

#define BDIC_REG_SYSTEM4_VAL            0x28
#define BDIC_REG_SLOPE_VAL              0xCB
#define BDIC_REG_DCDC1_VLIM_VAL         0x78
#define BDIC_REG_DCDC_SYS_VAL           0x20
#define BDIC_REG_DCDC2_VO_VAL           0xE8
#define BDIC_REG_SYSTEM2_BKL            0x03
#define BDIC_REG_ALS_ADJ0_L_DEFAULT     0x5C
#define BDIC_REG_ALS_ADJ0_H_DEFAULT     0x3F
#define BDIC_REG_ALS_ADJ1_L_DEFAULT     0xAA
#define BDIC_REG_ALS_ADJ1_H_DEFAULT     0x4C
#define BDIC_REG_ALS_SHIFT_DEFAULT      0x03
#define BDIC_REG_CLEAR_OFFSET_DEFAULT   0x00
#define BDIC_REG_IR_OFFSET_DEFAULT      0x00

/* ------------------------------------------------------------------------- */
/* VARIABLES                                                                 */
/* ------------------------------------------------------------------------- */

static const unsigned char shdisp_main_bkl_tbl[SHDISP_BKL_FIX_TBL_NUM][NUM_SHDISP_BKL_TBL_MODE] = {
    { 0x00,   0x00,   0x00,   0x00 },
    { 0x04,   0x04,   0x04,   0x04 },
    { 0x04,   0x04,   0x04,   0x04 },
    { 0x05,   0x05,   0x05,   0x05 },
    { 0x07,   0x06,   0x06,   0x07 },
    { 0x0A,   0x07,   0x07,   0x0A },
    { 0x0C,   0x09,   0x09,   0x0C },
    { 0x0F,   0x0B,   0x0B,   0x0F },
    { 0x12,   0x0C,   0x0C,   0x12 },
    { 0x16,   0x0E,   0x0E,   0x16 },
    { 0x1A,   0x10,   0x10,   0x1A },
    { 0x1E,   0x12,   0x12,   0x1E },
    { 0x23,   0x15,   0x15,   0x23 },
    { 0x29,   0x17,   0x15,   0x29 },
    { 0x2E,   0x1B,   0x15,   0x2E },
    { 0x35,   0x1E,   0x15,   0x35 },
    { 0x3C,   0x22,   0x15,   0x3C },
    { 0x44,   0x26,   0x15,   0x44 },
    { 0x4C,   0x2B,   0x15,   0x4C },
    { 0x56,   0x31,   0x15,   0x56 },
    { 0x60,   0x37,   0x15,   0x60 },
    { 0x6B,   0x3E,   0x15,   0x6B },
    { 0x78,   0x45,   0x15,   0x78 }
};

static const unsigned char shdisp_main_dtv_bkl_tbl[SHDISP_BKL_FIX_TBL_NUM][NUM_SHDISP_BKL_TBL_MODE] = {
    { 0x00,   0x00,   0x00,   0x00 },
    { 0x04,   0x04,   0x04,   0x04 },
    { 0x04,   0x04,   0x04,   0x04 },
    { 0x05,   0x05,   0x05,   0x05 },
    { 0x07,   0x06,   0x06,   0x07 },
    { 0x0A,   0x07,   0x07,   0x0A },
    { 0x0C,   0x09,   0x09,   0x0C },
    { 0x0F,   0x0B,   0x0B,   0x0F },
    { 0x12,   0x0C,   0x0C,   0x12 },
    { 0x16,   0x0E,   0x0E,   0x16 },
    { 0x1A,   0x10,   0x10,   0x1A },
    { 0x1E,   0x12,   0x12,   0x1E },
    { 0x23,   0x15,   0x15,   0x23 },
    { 0x29,   0x17,   0x15,   0x29 },
    { 0x2E,   0x1B,   0x15,   0x2E },
    { 0x35,   0x1E,   0x15,   0x35 },
    { 0x3C,   0x22,   0x15,   0x3C },
    { 0x44,   0x26,   0x15,   0x44 },
    { 0x4C,   0x2B,   0x15,   0x4C },
    { 0x56,   0x31,   0x15,   0x56 },
    { 0x60,   0x37,   0x15,   0x60 },
    { 0x6B,   0x3E,   0x15,   0x6B },
    { 0x78,   0x45,   0x15,   0x78 }
};  



static const unsigned char shdisp_main_bkl_opt_low_tbl[SHDISP_BKL_AUTO_TBL_NUM][1+NUM_SHDISP_BKL_TBL_MODE] = {
    { BDIC_REG_OPT0,    0x06,   0x03,   0x03,   0x11 },
    { BDIC_REG_OPT1,    0x08,   0x05,   0x05,   0x15 },
    { BDIC_REG_OPT2,    0x0B,   0x07,   0x07,   0x1B },
    { BDIC_REG_OPT3,    0x0F,   0x09,   0x09,   0x22 },
    { BDIC_REG_OPT4,    0x13,   0x0B,   0x0B,   0x29 },
    { BDIC_REG_OPT5,    0x18,   0x0E,   0x0E,   0x31 },
    { BDIC_REG_OPT6,    0x1C,   0x10,   0x10,   0x39 },
    { BDIC_REG_OPT7,    0x21,   0x13,   0x13,   0x43 },
    { BDIC_REG_OPT8,    0x25,   0x16,   0x16,   0x4E },
    { BDIC_REG_OPT9,    0x2B,   0x19,   0x16,   0x64 },
    { BDIC_REG_OPT10,   0x32,   0x1D,   0x16,   0x79 },
    { BDIC_REG_OPT11,   0x38,   0x21,   0x16,   0x7F },
    { BDIC_REG_OPT12,   0x3F,   0x25,   0x16,   0x84 },
    { BDIC_REG_OPT13,   0x45,   0x29,   0x16,   0x87 },
    { BDIC_REG_OPT14,   0x4B,   0x2C,   0x16,   0x89 },
    { BDIC_REG_OPT15,   0x78,   0x36,   0x16,   0x89 }
};

static const unsigned char shdisp_main_dtv_bkl_opt_low_tbl[SHDISP_BKL_AUTO_TBL_NUM][1+NUM_SHDISP_BKL_TBL_MODE] = {
    { BDIC_REG_OPT0,    0x06,   0x03,   0x03,   0x11 },
    { BDIC_REG_OPT1,    0x08,   0x05,   0x05,   0x15 },
    { BDIC_REG_OPT2,    0x0B,   0x07,   0x07,   0x1B },
    { BDIC_REG_OPT3,    0x0F,   0x09,   0x09,   0x22 },
    { BDIC_REG_OPT4,    0x13,   0x0B,   0x0B,   0x29 },
    { BDIC_REG_OPT5,    0x18,   0x0E,   0x0E,   0x31 },
    { BDIC_REG_OPT6,    0x1C,   0x10,   0x10,   0x39 },
    { BDIC_REG_OPT7,    0x21,   0x13,   0x13,   0x43 },
    { BDIC_REG_OPT8,    0x25,   0x16,   0x16,   0x4E },
    { BDIC_REG_OPT9,    0x2B,   0x19,   0x16,   0x64 },
    { BDIC_REG_OPT10,   0x32,   0x1D,   0x16,   0x79 },
    { BDIC_REG_OPT11,   0x38,   0x21,   0x16,   0x7F },
    { BDIC_REG_OPT12,   0x3F,   0x25,   0x16,   0x84 },
    { BDIC_REG_OPT13,   0x45,   0x29,   0x16,   0x87 },
    { BDIC_REG_OPT14,   0x4B,   0x2C,   0x16,   0x89 },
    { BDIC_REG_OPT15,   0x78,   0x36,   0x16,   0x89 }
};

static const unsigned char shdisp_main_bkl_opt_high_tbl[SHDISP_BKL_AUTO_TBL_NUM][1+NUM_SHDISP_BKL_TBL_MODE] = {
    { BDIC_REG_OPT0,    0x3D,   0x24,   0x16,   0x75 },
    { BDIC_REG_OPT1,    0x41,   0x26,   0x16,   0x7B },
    { BDIC_REG_OPT2,    0x46,   0x29,   0x16,   0x7F },
    { BDIC_REG_OPT3,    0x49,   0x2B,   0x16,   0x82 },
    { BDIC_REG_OPT4,    0x4D,   0x2D,   0x16,   0x84 },
    { BDIC_REG_OPT5,    0x51,   0x2F,   0x16,   0x86 },
    { BDIC_REG_OPT6,    0x55,   0x32,   0x16,   0x88 },
    { BDIC_REG_OPT7,    0x78,   0x36,   0x16,   0x89 },
    { BDIC_REG_OPT8,    0x78,   0x36,   0x16,   0x89 },
    { BDIC_REG_OPT9,    0x78,   0x36,   0x16,   0x89 },
    { BDIC_REG_OPT10,   0x78,   0x36,   0x16,   0x89 },
    { BDIC_REG_OPT11,   0x78,   0x36,   0x16,   0x89 },
    { BDIC_REG_OPT12,   0x78,   0x36,   0x16,   0x89 },
    { BDIC_REG_OPT13,   0x78,   0x36,   0x16,   0x89 },
    { BDIC_REG_OPT14,   0x78,   0x36,   0x16,   0x89 },
    { BDIC_REG_OPT15,   0x78,   0x36,   0x16,   0x89 }
};

static const unsigned char shdisp_main_dtv_bkl_opt_high_tbl[SHDISP_BKL_AUTO_TBL_NUM][1+NUM_SHDISP_BKL_TBL_MODE] = {
    { BDIC_REG_OPT0,    0x3D,   0x24,   0x16,   0x75 },
    { BDIC_REG_OPT1,    0x41,   0x26,   0x16,   0x7B },
    { BDIC_REG_OPT2,    0x46,   0x29,   0x16,   0x7F },
    { BDIC_REG_OPT3,    0x49,   0x2B,   0x16,   0x82 },
    { BDIC_REG_OPT4,    0x4D,   0x2D,   0x16,   0x84 },
    { BDIC_REG_OPT5,    0x51,   0x2F,   0x16,   0x86 },
    { BDIC_REG_OPT6,    0x55,   0x32,   0x16,   0x88 },
    { BDIC_REG_OPT7,    0x78,   0x36,   0x16,   0x89 },
    { BDIC_REG_OPT8,    0x78,   0x36,   0x16,   0x89 },
    { BDIC_REG_OPT9,    0x78,   0x36,   0x16,   0x89 },
    { BDIC_REG_OPT10,   0x78,   0x36,   0x16,   0x89 },
    { BDIC_REG_OPT11,   0x78,   0x36,   0x16,   0x89 },
    { BDIC_REG_OPT12,   0x78,   0x36,   0x16,   0x89 },
    { BDIC_REG_OPT13,   0x78,   0x36,   0x16,   0x89 },
    { BDIC_REG_OPT14,   0x78,   0x36,   0x16,   0x89 },
    { BDIC_REG_OPT15,   0x78,   0x36,   0x16,   0x89 }
};


static const unsigned char shdisp_main_bkl_adj_tbl[NUM_SHDISP_MAIN_BKL_ADJ] = {
    0x00,
    0x00,
    0x00,
    0x00
};

static const unsigned char shdisp_triple_led_tbl[SHDISP_TRI_LED_COLOR_TBL_NUM][3] = {
    { 0x00, 0x00, 0x00 },
    { 0x0C, 0x00, 0x00 },
    { 0x00, 0x0D, 0x00 },
    { 0x07, 0x0E, 0x00 },
    { 0x00, 0x00, 0x17 },
    { 0x0A, 0x00, 0x09 },
    { 0x00, 0x12, 0x09 },
    { 0x06, 0x0C, 0x0F }
};

static const unsigned char shdisp_triple_led_anime_tbl[2][SHDISP_TRI_LED_COLOR_TBL_NUM][3] = {
    {
        { 0x00, 0x00, 0x00 },
        { 0x00, 0x00, 0x00 },
        { 0x00, 0x00, 0x00 },
        { 0x00, 0x00, 0x00 },
        { 0x00, 0x00, 0x00 },
        { 0x00, 0x00, 0x00 },
        { 0x00, 0x00, 0x00 },
        { 0x00, 0x00, 0x00 }
    },
    {
        { 0x00, 0x00, 0x00 },
        { 0x0C, 0x00, 0x00 },
        { 0x00, 0x0D, 0x00 },
        { 0x07, 0x0E, 0x00 },
        { 0x00, 0x00, 0x17 },
        { 0x0A, 0x00, 0x09 },
        { 0x00, 0x12, 0x09 },
        { 0x06, 0x0C, 0x0F }
    }
};


static const unsigned char shdisp_main_bkl_chg_high_tbl[14][2] = {
    { 0x00, 0xA3 },
    { 0x00, 0x95 },
    { 0x00, 0xAF },
    { 0x00, 0xA9 },
    { 0x00, 0xBB },
    { 0x00, 0xB7 },
    { 0x00, 0xC3 },
    { 0x00, 0xC0 },
    { 0x00, 0xCE },
    { 0x00, 0xCA },
    { 0x00, 0xD8 },
    { 0x00, 0xD5 },
    { 0x00, 0xE4 },
    { 0x00, 0xE1 }
};

static const struct shdisp_bdic_bkl_lux_str shdisp_bdic_bkl_lux_tbl[2][16] = {
    {
        { 0, 0x000F,      5 },
        { 0, 0x001F,     10 },
        { 0, 0x002F,     20 },
        { 0, 0x003F,     30 },
        { 0, 0x004F,     60 },
        { 0, 0x005F,    100 },
        { 0, 0x006F,    180 },
        { 0, 0x007F,    300 },
        { 0, 0x008F,    600 },
        { 0, 0x009F,   1100 },
        { 0, 0x00AF,   2200 },
        { 0, 0x00BF,   4400 },
        { 0, 0x00CF,   8000 },
        { 0, 0x00DF,  15000 },
        { 1, 0x00EF,  35000 },
        { 1, 0x00FF,  75000 }
    },
    {
        { 0, 0x008F,   4800 },
        { 1, 0x009A,   7800 },
        { 1, 0x00A5,  11500 },
        { 1, 0x00AD,  16500 },
        { 1, 0x00B7,  24000 },
        { 1, 0x00C0,  34000 },
        { 1, 0x00CB,  40000 },
        { 1, 0x00FF,  42000 },
        { 1, 0x00FF,  42000 },
        { 1, 0x00FF,  42000 },
        { 1, 0x00FF,  42000 },
        { 1, 0x00FF,  42000 },
        { 1, 0x00FF,  42000 },
        { 1, 0x00FF,  42000 },
        { 1, 0x00FF,  42000 },
        { 1, 0x00FF,  42000 }
    }
};

/* ------------------------------------------------------------------------- */
/* TYPES                                                                     */
/* ------------------------------------------------------------------------- */



/* ------------------------------------------------------------------------- */
/* PROTOTYPES                                                                */
/* ------------------------------------------------------------------------- */



#endif /* SHDISP_BL69Y6_DATA_ARS_H */

/* ------------------------------------------------------------------------- */
/* END OF FILE                                                               */
/* ------------------------------------------------------------------------- */
