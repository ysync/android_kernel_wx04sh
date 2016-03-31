/* drivers/sharp/shdisp/shdisp_nicole.h  (Display Driver)
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

#ifndef SHDISP_NICOLE_H
#define SHDISP_NICOLE_H

/* ------------------------------------------------------------------------- */
/* MACROS                                                                    */
/* ------------------------------------------------------------------------- */

#define SHDISP_NICOLE_VCOM_MAX              0xFF

#define NICOLE_NUM_I2C_CONTROL_MAX_WRITE    8
#define NICOLE_NUM_I2C_CONTROL_MIN_WRITE    1
#define NICOLE_NUM_I2C_CONTROL_MAX_READ     8
#define NICOLE_NUM_I2C_CONTROL_MIN_READ     1

#define NICOLE_REG_SLEEPIN                  0x10
#define NICOLE_REG_SLEEPOUT                 0x11
#define NICOLE_REG_DISPOFF                  0x28
#define NICOLE_REG_DISPON                   0x29
#define NICOLE_REG_DEEP_STANDBY             0x70
#define NICOLE_REG_SCL_L_STAT               0x7D
#define NICOLE_REG_POFSEQB                  0x9C
#define NICOLE_REG_MANUALON                 0x9D
#define NICOLE_REG_GAMADJRP                 0xA2
#define NICOLE_REG_GAMADJRN                 0xA3
#define NICOLE_REG_GAMADJGP                 0xA4
#define NICOLE_REG_GAMADJGN                 0xA5
#define NICOLE_REG_GAMADJBP                 0xA6
#define NICOLE_REG_GAMADJBN                 0xA7

#define NICOLE_REG_READCOMMAND              0xAD
#define NICOLE_REG_WRITECOMMAND             0xAF

#define NICOLE_REG_BANK                     0xB0
#define NICOLE_REG_VCSET                    0xB3
#define NICOLE_REG_SETVGMPM                 0xB4
#define NICOLE_REG_RBIAS                    0xB5
#define NICOLE_REG_SELMODE                  0xB6
#define NICOLE_REG_SET_DDVDHP               0xB7
#define NICOLE_REG_SET_DDVDHM               0xB8
#define NICOLE_REG_SET_VGH                  0xB9
#define NICOLE_REG_SET_VGL                  0xBA
#define NICOLE_REG_SET_VCL                  0xBB
#define NICOLE_REG_TVBP                     0xBC
#define NICOLE_REG_CHIPID                   0xBD
#define NICOLE_REG_THDEHBP                  0xBF

#define NICOLE_REG_MIPI_ALTERNATIVE_CLK1    0xE7
#define NICOLE_REG_MIPI_ALTERNATIVE_CLK2    0xF5
#define NICOLE_REG_MIPI_ALTERNATIVE_CLK3    0xF6
#define NICOLE_REG_MIPI_ALTERNATIVE_CLK4    0xF8

#define NICOLE_REG_PANELCTL0                0xC0
#define NICOLE_REG_PANELCTL1                0xC1
#define NICOLE_REG_PANELCTL2                0xC2
#define NICOLE_REG_PANELCTL3                0xC3
#define NICOLE_REG_PANELCTL4                0xC4
#define NICOLE_REG_PANELCTL5                0xC5
#define NICOLE_REG_PANELCTL6                0xC6
#define NICOLE_REG_PANELCTL7                0xC7
#define NICOLE_REG_PANELCTL8                0xC8
#define NICOLE_REG_SSDST                    0xCA
#define NICOLE_REG_SSDWD                    0xCB
#define NICOLE_REG_SSDITV                   0xCC
#define NICOLE_REG_PCHLVL                   0xCD
#define NICOLE_REG_PCHST                    0xCE
#define NICOLE_REG_PCHWD                    0xCF

#define NICOLE_REG_EQ1                      0xD0
#define NICOLE_REG_EQ2                      0xD1
#define NICOLE_REG_BLNKLB                   0xD4
#define NICOLE_REG_SLINV                    0xD5
#define NICOLE_REG_SLINVIDL                 0xD6
#define NICOLE_REG_PB_CYCLE                 0xD7
#define NICOLE_REG_PB_DD                    0xD8
#define NICOLE_REG_LS_CYCLE                 0xD9
#define NICOLE_REG_LINEINV                  0xDA
#define NICOLE_REG_RESOL                    0xDB
#define NICOLE_REG_PIXARRANGE               0xDC
#define NICOLE_REG_STRIPE                   0xDD
#define NICOLE_REG_ZOOM                     0xDE
#define NICOLE_REG_SXDLY1                   0xF0
#define NICOLE_REG_SXDLY2                   0xF1

#define NICOLE_REG_MIPI_DSI_LANE            0x7D
#define NICOLE_REG_OTP_LOAD                 0xC4
#define NICOLE_REG_OTP_BIAS                 0xC6
#define NICOLE_REG_PASSWORD_SETTINGS1       0xFE
#define NICOLE_REG_PASSWORD_SETTINGS2       0xFF

/* PARAM */
#define SHDISP_NICOLE_SETVGMPM              0x55
#define SHDISP_NICOLE_COMDC                 0x9B

#define SHDISP_NICOLE_GAMRP0                0x00
#define SHDISP_NICOLE_GAMRP1                0x15
#define SHDISP_NICOLE_GAMRP2                0x13
#define SHDISP_NICOLE_GAMRP3                0x08
#define SHDISP_NICOLE_GAMRP4                0xC9
#define SHDISP_NICOLE_GAMRP5                0x9A
#define SHDISP_NICOLE_GAMRP6                0xA9
#define SHDISP_NICOLE_GAMRP7                0x0C

#define SHDISP_NICOLE_GAMRN0                0x00
#define SHDISP_NICOLE_GAMRN1                0x15
#define SHDISP_NICOLE_GAMRN2                0x13
#define SHDISP_NICOLE_GAMRN3                0x08
#define SHDISP_NICOLE_GAMRN4                0xC9
#define SHDISP_NICOLE_GAMRN5                0x9A
#define SHDISP_NICOLE_GAMRN6                0xA9
#define SHDISP_NICOLE_GAMRN7                0x0C

#define SHDISP_NICOLE_GAMGP0                0x16
#define SHDISP_NICOLE_GAMGP1                0x0C
#define SHDISP_NICOLE_GAMGP2                0x00
#define SHDISP_NICOLE_GAMGP3                0x09
#define SHDISP_NICOLE_GAMGP4                0x88
#define SHDISP_NICOLE_GAMGP5                0x9A
#define SHDISP_NICOLE_GAMGP6                0xA9
#define SHDISP_NICOLE_GAMGP7                0x12

#define SHDISP_NICOLE_GAMGN0                0x16
#define SHDISP_NICOLE_GAMGN1                0x0C
#define SHDISP_NICOLE_GAMGN2                0x00
#define SHDISP_NICOLE_GAMGN3                0x09
#define SHDISP_NICOLE_GAMGN4                0x88
#define SHDISP_NICOLE_GAMGN5                0x9A
#define SHDISP_NICOLE_GAMGN6                0xA9
#define SHDISP_NICOLE_GAMGN7                0x12

#define SHDISP_NICOLE_GAMBP0                0x16
#define SHDISP_NICOLE_GAMBP1                0x0C
#define SHDISP_NICOLE_GAMBP2                0x00
#define SHDISP_NICOLE_GAMBP3                0x09
#define SHDISP_NICOLE_GAMBP4                0x88
#define SHDISP_NICOLE_GAMBP5                0x9A
#define SHDISP_NICOLE_GAMBP6                0xA9
#define SHDISP_NICOLE_GAMBP7                0x12

#define SHDISP_NICOLE_GAMBN0                0x16
#define SHDISP_NICOLE_GAMBN1                0x0C
#define SHDISP_NICOLE_GAMBN2                0x00
#define SHDISP_NICOLE_GAMBN3                0x09
#define SHDISP_NICOLE_GAMBN4                0x88
#define SHDISP_NICOLE_GAMBN5                0x9A
#define SHDISP_NICOLE_GAMBN6                0xA9
#define SHDISP_NICOLE_GAMBN7                0x12

#define SHDISP_NICOLE_PNLCTL0               0xC8
#define SHDISP_NICOLE_PNLCTL1               0x00
#define SHDISP_NICOLE_PNLCTL2               0x00
#define SHDISP_NICOLE_PNLCTL3               0x00
#define SHDISP_NICOLE_PNLCTL4               0x18
#define SHDISP_NICOLE_PNLCTL5               0x30
#define SHDISP_NICOLE_PNLCTL6               0x60
#define SHDISP_NICOLE_PNLCTL7               0x00
#define SHDISP_NICOLE_PNLCTL8               0x00
#define SHDISP_NICOLE_PNLCTL9               0x18
#define SHDISP_NICOLE_PNLCTL10              0x61
#define SHDISP_NICOLE_PNLCTL11              0x18

/* ------------------------------------------------------------------------- */
/* TYPES                                                                     */
/* ------------------------------------------------------------------------- */


/* ------------------------------------------------------------------------- */
/* VARIABLES                                                                 */
/* ------------------------------------------------------------------------- */



/* ------------------------------------------------------------------------- */
/* PROTOTYPES                                                                */
/* ------------------------------------------------------------------------- */

struct shdisp_panel_operations *shdisp_nicole_API_create(void);

#endif /* SHDISP_NICOLE_H */

/* ------------------------------------------------------------------------- */
/* END OF FILE                                                               */
/* ------------------------------------------------------------------------- */

