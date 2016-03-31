/* drivers/sharp/shdisp/shdisp_bdic_data.h  (Display Driver)
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

#ifndef SHDISP_BDIC_DATA_H
#define SHDISP_BDIC_DATA_H

/* ------------------------------------------------------------------------- */
/* INCLUDE FILES                                                             */
/* ------------------------------------------------------------------------- */



/* ------------------------------------------------------------------------- */
/* MACROS                                                                    */
/* ------------------------------------------------------------------------- */

#define SHDISP_BKL_FIX_TBL_NUM          23
#define SHDISP_BKL_AUTO_TBL_NUM         16
#define SHDISP_TRI_LED_COLOR_TBL_NUM    8

/* ------------------------------------------------------------------------- */
/* VARIABLES                                                                 */
/* ------------------------------------------------------------------------- */

static const unsigned char shdisp_main_bkl_tbl[SHDISP_BKL_FIX_TBL_NUM] = {
    0x00,
    0x08,
    0x08,
    0x0D,
    0x12,
    0x17,
    0x1C,
    0x22,
    0x28,
    0x2C,
    0x30,
    0x32,
    0x35,
    0x3A,
    0x3F,
    0x46,
    0x4E,
    0x5A,
    0x67,
    0x78,
    0x8A,
    0x9B,
    0xAD
};

static const unsigned char shdisp_main_dtv_bkl_tbl[SHDISP_BKL_FIX_TBL_NUM] = {
    0x00,
    0x08,
    0x08,
    0x0D,
    0x12,
    0x17,
    0x1C,
    0x22,
    0x28,
    0x2C,
    0x30,
    0x32,
    0x35,
    0x3A,
    0x3F,
    0x46,
    0x4E,
    0x5A,
    0x67,
    0x78,
    0x8A,
    0x9B,
    0xAD
};

static const unsigned char shdisp_main_bkl_opt_tbl[SHDISP_BKL_AUTO_TBL_NUM][3] = {
    { BDIC_REG_OPT0,  0x09, 0x09 },
    { BDIC_REG_OPT1,  0x11, 0x11 },
    { BDIC_REG_OPT2,  0x17, 0x17 },
    { BDIC_REG_OPT3,  0x1C, 0x1C },
    { BDIC_REG_OPT4,  0x22, 0x22 },
    { BDIC_REG_OPT5,  0x28, 0x28 },
    { BDIC_REG_OPT6,  0x2E, 0x2E },
    { BDIC_REG_OPT7,  0x35, 0x35 },
    { BDIC_REG_OPT8,  0x39, 0x39 },
    { BDIC_REG_OPT9,  0x3D, 0x3A },
    { BDIC_REG_OPT10, 0x44, 0x3A },
    { BDIC_REG_OPT11, 0x53, 0x3A },
    { BDIC_REG_OPT12, 0x6C, 0x3A },
    { BDIC_REG_OPT13, 0x8A, 0x3A },
    { BDIC_REG_OPT14, 0xAD, 0x3A },
    { BDIC_REG_OPT15, 0xAD, 0x3A }
};

static const unsigned char shdisp_triple_led_tbl[SHDISP_TRI_LED_COLOR_TBL_NUM][3] = {
    { 0x00, 0x00, 0x00 },
    { 0x0C, 0x00, 0x00 },
    { 0x00, 0x0D, 0x00 },
    { 0x0A, 0x0F, 0x00 },
    { 0x00, 0x00, 0x17 },
    { 0x0A, 0x00, 0x09 },
    { 0x00, 0x12, 0x09 },
    { 0x0A, 0x0F, 0x09 }
};

static const unsigned char shdisp_triple_led_anime_tbl[2][SHDISP_TRI_LED_COLOR_TBL_NUM][3] = {
    {
        { 0x00, 0x00, 0x00 },
        { 0xF4, 0x00, 0x00 },
        { 0x00, 0xF3, 0x00 },
        { 0xF6, 0xF1, 0x00 },
        { 0x00, 0x00, 0xE9 },
        { 0xF6, 0x00, 0xF7 },
        { 0x00, 0xEE, 0xF7 },
        { 0xF6, 0xF1, 0xF7 }
    },
    {
        { 0x00, 0x00, 0x00 },
        { 0x0C, 0x00, 0x00 },
        { 0x00, 0x0D, 0x00 },
        { 0x0A, 0x0F, 0x00 },
        { 0x00, 0x00, 0x17 },
        { 0x0A, 0x00, 0x09 },
        { 0x00, 0x12, 0x09 },
        { 0x0A, 0x0F, 0x09 }
    }
};

/* ------------------------------------------------------------------------- */
/* TYPES                                                                     */
/* ------------------------------------------------------------------------- */



/* ------------------------------------------------------------------------- */
/* PROTOTYPES                                                                */
/* ------------------------------------------------------------------------- */



#endif /* SHDISP_BDIC_DATA_H */

/* ------------------------------------------------------------------------- */
/* END OF FILE                                                               */
/* ------------------------------------------------------------------------- */
