/* drivers/sharp/shdisp/shdisp_takt.h  (Display Driver)
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

#ifndef SHDISP_TAKT_H
#define SHDISP_TAKT_H

/* ------------------------------------------------------------------------- */
/* MACROS                                                                    */
/* ------------------------------------------------------------------------- */

#define SHDISP_TAKT_VCOM_MAX        0x50
#define SHDISP_TAKT_VCOM_OFFSET     0x28
#define SHDISP_TAKT_VCOM_DEFAULT    0xA5

#define TAKT_REG_VER_MAJOR          0x00
#define TAKT_REG_CLKSEL             0x02
#define TAKT_REG_CLKCTL             0x03
#define TAKT_REG_OSCSET             0x04
#define TAKT_REG_RXDIV              0x05
#define TAKT_REG_SUBCLKDIV          0x06
#define TAKT_REG_BSTMDIV            0x07
#define TAKT_REG_VGHLDIV            0x08
#define TAKT_REG_SWRDIV             0x09
#define TAKT_REG_SWNVMDIV           0x0A
#define TAKT_REG_DIMDIV             0x0B
#define TAKT_REG_NCOSCSET           0x0C
#define TAKT_REG_DFIM               0x0D
#define TAKT_REG_SEQSEL             0x0E
#define TAKT_REG_EXTDIV             0x0F
#define TAKT_REG_SLPOCTL            0x10
#define TAKT_REG_VER_MINOR          0x1F
#define TAKT_REG_IMGSET1            0x20
#define TAKT_REG_IMGSET2            0x21
#define TAKT_REG_IVBP_A             0x22
#define TAKT_REG_IVNUMH_A           0x23
#define TAKT_REG_IVNUML_A           0x24
#define TAKT_REG_IHNUMH_A           0x25
#define TAKT_REG_IHNUML_A           0x26
#define TAKT_REG_IVBP_B             0x27
#define TAKT_REG_IVNUMH_B           0x28
#define TAKT_REG_IVNUML_B           0x29
#define TAKT_REG_IHNUMH_B           0x2A
#define TAKT_REG_IHNUML_B           0x2B
#define TAKT_REG_ISVFP              0x2C
#define TAKT_REG_SUBIFCTL           0x2D
#define TAKT_REG_SUBIFOSEL          0x2E
#define TAKT_REG_RXCTL1             0x30
#define TAKT_REG_RXCTL2             0x31
#define TAKT_REG_RXONDLY            0x32
#define TAKT_REG_VDSYNCTL           0x33
#define TAKT_REG_VDSET              0x34
#define TAKT_REG_VDMODE_A           0x35
#define TAKT_REG_VDMODE_B           0x36
#define TAKT_REG_VDERRCTL           0x37
#define TAKT_REG_VFCODE             0x39
#define TAKT_REG_VECODE             0x3A
#define TAKT_REG_HFCODE             0x3B
#define TAKT_REG_HECODE             0x3C
#define TAKT_REG_SYNCODE1           0x3D
#define TAKT_REG_SYNCODE2           0x3E
#define TAKT_REG_SYNCODE3           0x3F
#define TAKT_REG_DSPCTL1            0x40
#define TAKT_REG_VALTRAN            0x41
#define TAKT_REG_DSPCTL2            0x42
#define TAKT_REG_PNLCTL1            0x43
#define TAKT_REG_PNLCTL2            0x44
#define TAKT_REG_MODECTL            0x45
#define TAKT_REG_FRCCTL             0x46
#define TAKT_REG_LPCTL              0x47
#define TAKT_REG_SDCTL              0x48
#define TAKT_REG_SDSET              0x49
#define TAKT_REG_CMISET             0x4A
#define TAKT_REG_SD2NUM             0x4B
#define TAKT_REG_SD3NUM             0x4C
#define TAKT_REG_RECCTL             0x4D
#define TAKT_REG_CHOPC              0x4E
#define TAKT_REG_TGSET1             0x50
#define TAKT_REG_TGSET2             0x51
#define TAKT_REG_GSPSET             0x52
#define TAKT_REG_GCKSET             0x53
#define TAKT_REG_SSDSET             0x54
#define TAKT_REG_INTMITCTL          0x55
#define TAKT_REG_INTMITSET_A        0x56
#define TAKT_REG_INTMITSET_B        0x57
#define TAKT_REG_DREPSET            0x58
#define TAKT_REG_TGOUTSET1          0x59
#define TAKT_REG_TGOUTSET2          0x5A
#define TAKT_REG_GSPDLY             0x5B
#define TAKT_REG_HVBSTDIV           0x5C
#define TAKT_REG_HVBSTST            0x5D
#define TAKT_REG_HVBSTWD            0x5E
#define TAKT_REG_GCKST_SLF          0x60
#define TAKT_REG_GCKWD_SLF          0x61
#define TAKT_REG_SSDST_SLF          0x62
#define TAKT_REG_SSDWD_SLF          0x63
#define TAKT_REG_SSDITV_SLF         0x64
#define TAKT_REG_CMICH_SLF          0x65
#define TAKT_REG_PCHGST_SLF         0x66
#define TAKT_REG_PCHGWD_SLF         0x67
#define TAKT_REG_USR_H              0x68
#define TAKT_REG_USR_L              0x69
#define TAKT_REG_GCKST_A            0x70
#define TAKT_REG_GCKWD_A            0x71
#define TAKT_REG_SSDST_A            0x72
#define TAKT_REG_SSDWD_A            0x73
#define TAKT_REG_SSDITV_A           0x74
#define TAKT_REG_CMICH_A            0x75
#define TAKT_REG_PCHGST_A           0x76
#define TAKT_REG_PCHGWD_A           0x77
#define TAKT_REG_HSLIMIT_A          0x78
#define TAKT_REG_SXDLY12_A          0x79
#define TAKT_REG_SXDLY34_A          0x7A
#define TAKT_REG_SXDLY56_A          0x7B
#define TAKT_REG_SXLWD12_A          0x7C
#define TAKT_REG_SXLWD34_A          0x7D
#define TAKT_REG_SXLWD56_A          0x7E
#define TAKT_REG_SXBIAS_A           0x7F
#define TAKT_REG_GCKST_B            0x80
#define TAKT_REG_GCKWD_B            0x81
#define TAKT_REG_SSDST_B            0x82
#define TAKT_REG_SSDWD_B            0x83
#define TAKT_REG_SSDITV_B           0x84
#define TAKT_REG_CMICH_B            0x85
#define TAKT_REG_PCHGST_B           0x86
#define TAKT_REG_PCHGWD_B           0x87
#define TAKT_REG_HSLIMIT_B          0x88
#define TAKT_REG_SXDLY12_B          0x89
#define TAKT_REG_SXDLY34_B          0x8A
#define TAKT_REG_SXDLY56_B          0x8B
#define TAKT_REG_SXLWD12_B          0x8C
#define TAKT_REG_SXLWD34_B          0x8D
#define TAKT_REG_SXLWD56_B          0x8E
#define TAKT_REG_SXBIAS_B           0x8F
#define TAKT_REG_POWCTL1            0x90
#define TAKT_REG_POWCTL2            0x91
#define TAKT_REG_SWRSET1            0x92
#define TAKT_REG_SWRSET2            0x93
#define TAKT_REG_DCAPBBS            0x94
#define TAKT_REG_VREFV              0x95
#define TAKT_REG_VRGMV              0x96
#define TAKT_REG_VRCOMDCV           0x97
#define TAKT_REG_BIASSET            0x98
#define TAKT_REG_GMCOMBS            0x99
#define TAKT_REG_VGHPWR             0x9A
#define TAKT_REG_DSIVC              0x9B
#define TAKT_REG_DSISET             0x9C
#define TAKT_REG_DSIICONT           0x9D
#define TAKT_REG_DSIMSK1            0x9E
#define TAKT_REG_DSIMSK2            0x9F
#define TAKT_REG_PVCH_A             0xA0
#define TAKT_REG_PVCL_A             0xA1
#define TAKT_REG_NVCH_A             0xA2
#define TAKT_REG_NVCL_A             0xA3
#define TAKT_REG_COMDCV_A           0xA4
#define TAKT_REG_PVCH_B             0xA5
#define TAKT_REG_PVCL_B             0xA6
#define TAKT_REG_NVCH_B             0xA7
#define TAKT_REG_NVCL_B             0xA8
#define TAKT_REG_COMDCV_B           0xA9
#define TAKT_REG_VCCOPY             0xAA
#define TAKT_REG_BKVCH              0xAB
#define TAKT_REG_BKVCL              0xAC
#define TAKT_REG_BKNVCH             0xAD
#define TAKT_REG_BKNVCL             0xAE
#define TAKT_REG_INTCTL             0xB0
#define TAKT_REG_SWRES              0xC0
#define TAKT_REG_SRDADR             0xC1
#define TAKT_REG_I2CNC              0xC3
#define TAKT_REG_HRDCTL             0xC4
#define TAKT_REG_NVMACS             0xC6
#define TAKT_REG_NVOPEN             0xC9
#define TAKT_REG_RESDSTB            0xCB
#define TAKT_REG_GAMFIXON           0xD0
#define TAKT_REG_GAMFIX_RPH         0xD1
#define TAKT_REG_GAMFIX_RPL         0xD2
#define TAKT_REG_GAMFIX_GPH         0xD3
#define TAKT_REG_GAMFIX_GPL         0xD4
#define TAKT_REG_GAMFIX_BPH         0xD5
#define TAKT_REG_GAMFIX_BPL         0xD6
#define TAKT_REG_GAMFIX_RNH         0xD7
#define TAKT_REG_GAMFIX_RNL         0xD8
#define TAKT_REG_GAMFIX_GNH         0xD9
#define TAKT_REG_GAMFIX_GNL         0xDA
#define TAKT_REG_GAMFIX_BNH         0xDB
#define TAKT_REG_GAMFIX_BNL         0xDC
#define TAKT_REG_TRVMEMSET          0xDD
#define TAKT_REG_TRVRDADR           0xDE
#define TAKT_REG_TRVRDDAT           0xDF
#define TAKT_REG_GAMCTL             0xE0
#define TAKT_REG_GAMSET             0xE1
#define TAKT_REG_GAMVALPH           0xE2
#define TAKT_REG_GAMVALPL           0xE3
#define TAKT_REG_GAMVALNH           0xE4
#define TAKT_REG_GAMVALNL           0xE5
#define TAKT_REG_GAMRDADR           0xE6
#define TAKT_REG_GAMRDDATH          0xE7
#define TAKT_REG_GAMRDDATL          0xE8
#define TAKT_REG_GAMASSET           0xE9
#define TAKT_REG_BANKCTL            0xEF
#define TAKT_REG_TEST0              0xF0
#define TAKT_REG_TEST1              0xF1

#define TAKT_CSTM_REG_CPFON         0x40
#define TAKT_CSTM_REG_CPFTRVPOS     0x41
#define TAKT_CSTM_REG_CPFCTRL       0x42
#define TAKT_CSTM_REG_CPFMODE       0x43
#define TAKT_CSTM_REG_CPFSTRDAT0    0x50
#define TAKT_CSTM_REG_CPFSTRDAT1    0x51
#define TAKT_CSTM_REG_CPFSTRDAT2    0x52
#define TAKT_CSTM_REG_CPFSTRDAT3    0x53
#define TAKT_CSTM_REG_CPFSTRDAT4    0x54
#define TAKT_CSTM_REG_CPFFIXLUT1    0x5C
#define TAKT_CSTM_REG_CPFFIXLUT2    0x5D

#define TAKT_GAM_REG_GMP0           0x00

#define SHDISP_GAM_COLOR_NUM        3
#define SHDISP_GAM_IDX_NUM          256
#define SHDISP_GAM_ADJ_POINTS_NUM   8

#define SHDISP_GAM_BANK_REG_NUM     128

/* ------------------------------------------------------------------------- */
/* TYPES                                                                     */
/* ------------------------------------------------------------------------- */
struct shdisp_gam_adj_point {
    unsigned char gam_lvl;
    unsigned short gam_val;
};


struct shdisp_diag_set_gam_tbl {
    unsigned short status;
    unsigned char def_pos_tbl[SHDISP_GAM_COLOR_NUM][SHDISP_GAM_IDX_NUM];
    unsigned short def_init_pos[SHDISP_GAM_COLOR_NUM];
    struct shdisp_gam_adj_point point[SHDISP_GAM_COLOR_NUM][SHDISP_GAM_ADJ_POINTS_NUM];
};

/* ------------------------------------------------------------------------- */
/* VARIABLES                                                                 */
/* ------------------------------------------------------------------------- */



/* ------------------------------------------------------------------------- */
/* PROTOTYPES                                                                */
/* ------------------------------------------------------------------------- */

struct shdisp_panel_operations *shdisp_takt_API_create(void);

#endif /* SHDISP_TAKT_H */

/* ------------------------------------------------------------------------- */
/* END OF FILE                                                               */
/* ------------------------------------------------------------------------- */

