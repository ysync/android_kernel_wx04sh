/* sound/soc/codecs/ymu828b/sh_mc_asoc.c
 *
 * Copyright (C) 2013 SHARP CORPORATION
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
/* CONFIG_SH_AUDIO_DRIVER newly created */

#include <linux/module.h>
#include <linux/i2c.h>
#include <linux/mutex.h>
#include <sound/soc.h>
#include <sound/tlv.h>
#include "mcdriver.h"
#include "sh_mc_asoc.h"




static int sh_mc_asoc_no_init = 1;
static int sh_mc_asoc_cdsp_registered = 0;
static struct mutex sh_mc_asoc_mutex;




static struct i2c_client *sh_mc_asoc_i2c_d;
static struct i2c_client *sh_mc_asoc_i2c_a;

struct i2c_client* sh_mc_asoc_get_i2c_client(int slave)
{
	if (slave == 0x11){
		return sh_mc_asoc_i2c_d;
	}
	if (slave == 0x3a){
		return sh_mc_asoc_i2c_a;
	}
	pr_err("%s: i2c slave address err (0x%x)\n", __func__, slave);
	return NULL;
}




static const MCDRV_PLL_INFO sh_mc_asoc_hw_pll_info = {
	0x10, /* bMode0 */
#ifdef SH_MC_ASOC_TEST_BOARD
	11, /* bPrevDiv0 */
	71, /* wFbDiv0 */
	54836, /* wFrac0 */
#else
	27, /* bPrevDiv0 */
	73, /* wFbDiv0 */
	47710, /* wFrac0 */
#endif
	0x10, /* bMode1 */
#ifdef SH_MC_ASOC_TEST_BOARD
	11, /* bPrevDiv1 */
	71, /* wFbDiv1 */
	54836 /* wFrac1 */
#else
	27, /* bPrevDiv1 */
	73, /* wFbDiv1 */
	47710 /* wFrac1 */
#endif
};

static const MCDRV_INIT_INFO sh_mc_asoc_hw_init_info = {
#ifdef SH_MC_ASOC_TEST_BOARD
	MCDRV_CKSEL_CMOS, /* bCkSel */
#else
	MCDRV_CKSEL_TCXO, /* bCkSel */
#endif
	0, /* bDivR0 */
	0, /* bDivF0 */
	0, /* bDivR1 */
	0, /* bDivF1 */
	0, /* bRange0 */
	0, /* bRange1 */
	0, /* bBypass */
	MCDRV_DAHIZ_LOW, /* bDioSdo0Hiz */
	MCDRV_DAHIZ_LOW, /* bDioSdo1Hiz */
	MCDRV_DAHIZ_LOW, /* bDioSdo2Hiz */
	MCDRV_DAHIZ_LOW, /* bDioClk0Hiz */
	MCDRV_DAHIZ_LOW, /* bDioClk1Hiz */
	MCDRV_DAHIZ_LOW, /* bDioClk2Hiz */
	MCDRV_PCMHIZ_LOW, /* bPcmHiz */
	MCDRV_LINE_DIF, /* bLineIn1Dif */
	MCDRV_LINE_STEREO, /* bLineIn2Dif */
	MCDRV_LINE_STEREO, /* bLineOut1Dif */
	MCDRV_LINE_STEREO, /* bLineOut2Dif */
	MCDRV_SPMN_ON, /* bSpmn */
	MCDRV_MIC_DIF, /* bMic1Sng */
	MCDRV_MIC_DIF, /* bMic2Sng */
	MCDRV_MIC_DIF, /* bMic3Sng */
	MCDRV_POWMODE_FULL, /* bPowerMode */
	MCDRV_SPHIZ_PULLDOWN, /* bSpHiz */
	MCDRV_LDO_AON_DON, /* bLdo */
	MCDRV_PAD_GPIO, /* bPad0Func */
	MCDRV_PAD_GPIO, /* bPad1Func */
	MCDRV_PAD_GPIO, /* bPad2Func */
	MCDRV_OUTLEV_6, /* bAvddLev */
	0, /* bVrefLev */
	MCDRV_DCLGAIN_12, /* bDclGain */
	MCDRV_DCLLIMIT_0, /* bDclLimit */
	0, /* bCpMod */
	MCDRV_SD_DS_LLL, /* bSdDs */
	MCDRV_HPIDLE_OFF, /* bHpIdle */
	MCDRV_CLKI_NORMAL, /* bCLKI1 */
	MCDRV_MBSDISCH_000, /* bMbsDisch */
	0, /* bReserved */
	{ /* sWaitTime */
		0, /* dAdHpf */
		0, /* dMic1Cin */
		0, /* dMic2Cin */
		0, /* dMic3Cin */
		40000, /* dLine1Cin */
		40000, /* dLine2Cin */
		5000, /* dVrefRdy1 */
		15000, /* dVrefRdy2 */
		9000, /* dHpRdy */
		13000, /* dSpRdy */
		0, /* dPdm */
		1000, /* dAnaRdyInterval */
		1000, /* dSvolInterval */
		100, /* dAnaRdyTimeOut */
		100, /* dSvolTimeOut */
	}
};

static const MCDRV_DIO_INFO sh_mc_asoc_hw_dio_info = {
	{
		{ /* asPortInfo[0] */
			{ /* sDioCommon */
				MCDRV_DIO_SLAVE, /* bMasterSlave */
				MCDRV_AUTOFS_ON, /* bAutoFs */
				MCDRV_FS_8000, /* bFs */
				MCDRV_BCKFS_256, /* bBckFs */
				MCDRV_DIO_PCM, /* bInterface */
				MCDRV_BCLK_NORMAL, /* bBckInvert */
				MCDRV_PCMHIZTIM_FALLING, /* bPcmHizTim */
				MCDRV_PCM_CLKDOWN_OFF, /* bPcmClkDown */
				MCDRV_PCM_SHORTFRAME, /* bPcmFrame */
				0, /* bPcmHighPeriod */
			},
			{ /* sDir */
				0, /* wSrcRate */
				{ /* sDaFormat */
					MCDRV_BITSEL_16, /* bBitSel */
					MCDRV_DAMODE_HEADALIGN, /* bMode */
				},
				{ /* sPcmFormat */
					MCDRV_PCM_MONO, /* bMono */
					MCDRV_PCM_MSB_FIRST, /* bOrder */
					MCDRV_PCM_LINEAR, /* bLaw */
					MCDRV_PCM_BITSEL_16, /* bBitSel */
				},
				{0, 1}, /* abSlot[DIO_CHANNELS] */
			},
			{ /* sDit */
				0, /* wSrcRate */
				{ /* sDaFormat */
					MCDRV_BITSEL_16, /* bBitSel */
					MCDRV_DAMODE_HEADALIGN, /* bMode */
				},
				{ /* sPcmFormat */
					MCDRV_PCM_MONO, /* bMono */
					MCDRV_PCM_MSB_FIRST, /* bOrder */
					MCDRV_PCM_LINEAR, /* bLaw */
					MCDRV_PCM_BITSEL_16, /* bBitSel */
				},
				{0, 1}, /* abSlot[DIO_CHANNELS] */
			},
		},
		{ /* asPortInfo[1] */
			{ /* sDioCommon */
				MCDRV_DIO_SLAVE, /* bMasterSlave */
				MCDRV_AUTOFS_ON, /* bAutoFs */
				MCDRV_FS_8000, /* bFs */
				MCDRV_BCKFS_32, /* bBckFs */
				MCDRV_DIO_PCM, /* bInterface */
				MCDRV_BCLK_NORMAL, /* bBckInvert */
				MCDRV_PCMHIZTIM_FALLING, /* bPcmHizTim */
				MCDRV_PCM_CLKDOWN_OFF, /* bPcmClkDown */
				MCDRV_PCM_SHORTFRAME, /* bPcmFrame */
				0, /* bPcmHighPeriod */
			},
			{ /* sDir */
				0, /* wSrcRate */
				{ /* sDaFormat */
					MCDRV_BITSEL_16, /* bBitSel */
					MCDRV_DAMODE_HEADALIGN, /* bMode */
				},
				{ /* sPcmFormat */
					MCDRV_PCM_MONO, /* bMono */
					MCDRV_PCM_LSB_FIRST, /* bOrder */
					MCDRV_PCM_LINEAR, /* bLaw */
					MCDRV_PCM_BITSEL_16, /* bBitSel */
				},
				{0, 1}, /* abSlot[DIO_CHANNELS] */
			},
			{ /* sDit */
				0, /* wSrcRate */
				{ /* sDaFormat */
					MCDRV_BITSEL_16, /* bBitSel */
					MCDRV_DAMODE_HEADALIGN, /* bMode */
				},
				{ /* sPcmFormat */
					MCDRV_PCM_MONO, /* bMono */
					MCDRV_PCM_LSB_FIRST, /* bOrder */
					MCDRV_PCM_LINEAR, /* bLaw */
					MCDRV_PCM_BITSEL_16, /* bBitSel */
				},
				{0, 1}, /* abSlot[DIO_CHANNELS] */
			},
		},
		{ /* asPortInfo[2] */
			{ /* sDioCommon */
				MCDRV_DIO_SLAVE, /* bMasterSlave */
				MCDRV_AUTOFS_ON, /* bAutoFs */
				MCDRV_FS_8000, /* bFs */
				MCDRV_BCKFS_48, /* bBckFs */
				MCDRV_DIO_PCM, /* bInterface */
				MCDRV_BCLK_NORMAL, /* bBckInvert */
				MCDRV_PCMHIZTIM_FALLING, /* bPcmHizTim */
				MCDRV_PCM_CLKDOWN_OFF, /* bPcmClkDown */
				MCDRV_PCM_LONGFRAME, /* bPcmFrame */
				0, /* bPcmHighPeriod */
			},
			{ /* sDir */
				0, /* wSrcRate */
				{ /* sDaFormat */
					MCDRV_BITSEL_16, /* bBitSel */
					MCDRV_DAMODE_HEADALIGN, /* bMode */
				},
				{ /* sPcmFormat */
					MCDRV_PCM_MONO, /* bMono */
					MCDRV_PCM_MSB_FIRST, /* bOrder */
					MCDRV_PCM_MULAW, /* bLaw */
					MCDRV_PCM_BITSEL_8, /* bBitSel */
				},
				{0, 1}, /* abSlot[DIO_CHANNELS] */
			},
			{ /* sDit */
				0, /* wSrcRate */
				{ /* sDaFormat */
					MCDRV_BITSEL_16, /* bBitSel */
					MCDRV_DAMODE_HEADALIGN, /* bMode */
				},
				{ /* sPcmFormat */
					MCDRV_PCM_MONO, /* bMono */
					MCDRV_PCM_MSB_FIRST, /* bOrder */
					MCDRV_PCM_MULAW, /* bLaw */
					MCDRV_PCM_BITSEL_8, /* bBitSel */
				},
				{0, 1} /* abSlot[DIO_CHANNELS] */
			}
		}
	}
};

static const MCDRV_ADC_INFO sh_mc_asoc_hw_adc_info = {
	MCDRV_AGCADJ_0, /* bAgcAdjust */
	MCDRV_AGC_OFF, /* bAgcOn */
	MCDRV_ADC_STEREO /* bMono */
};




enum {
	SH_MC_ASOC_SRC_DIR0 = 0,
	SH_MC_ASOC_SRC_DIR2,
	SH_MC_ASOC_SRC_DIR2_DIRECT,
	SH_MC_ASOC_SRC_ADC0,
	SH_MC_ASOC_SRC_LINE1_M,
	SH_MC_ASOC_SRC_LINE1_L,
	SH_MC_ASOC_SRC_LINE1_R,
	SH_MC_ASOC_SRC_CDSP,
	SH_MC_ASOC_SRC_CDSP_DIRECT,
	SH_MC_ASOC_SRC_MIX,
	SH_MC_ASOC_SRC_DTMF,

	SH_MC_ASOC_SRC_MAX
};

#define SH_MC_ASOC_SRC_BIT(src)  (1 << (src))

struct sh_mc_asoc_src_onoff_info {
	u8 block;
	u8 on;
	u8 off;
};

static const struct sh_mc_asoc_src_onoff_info sh_mc_asoc_src_onoff_tbl[SH_MC_ASOC_SRC_MAX] = {
	{MCDRV_SRC_DIR0_BLOCK, MCDRV_SRC3_DIR0_ON, MCDRV_SRC3_DIR0_OFF},
	{MCDRV_SRC_DIR2_BLOCK, MCDRV_SRC3_DIR2_ON, MCDRV_SRC3_DIR2_OFF},
	{MCDRV_SRC_DIR2_DIRECT_BLOCK, MCDRV_SRC3_DIR2_DIRECT_ON, MCDRV_SRC3_DIR2_DIRECT_OFF},
	{MCDRV_SRC_ADC0_BLOCK, MCDRV_SRC4_ADC0_ON, MCDRV_SRC4_ADC0_OFF},
	{MCDRV_SRC_LINE1_M_BLOCK, MCDRV_SRC1_LINE1_M_ON, MCDRV_SRC1_LINE1_M_OFF},
	{MCDRV_SRC_LINE1_L_BLOCK, MCDRV_SRC1_LINE1_L_ON, MCDRV_SRC1_LINE1_L_OFF},
	{MCDRV_SRC_LINE1_R_BLOCK, MCDRV_SRC1_LINE1_R_ON, MCDRV_SRC1_LINE1_R_OFF},
	{MCDRV_SRC_CDSP_BLOCK, MCDRV_SRC6_CDSP_ON, MCDRV_SRC6_CDSP_OFF},
	{MCDRV_SRC_CDSP_DIRECT_BLOCK, MCDRV_SRC6_CDSP_DIRECT_ON, MCDRV_SRC6_CDSP_DIRECT_OFF},
	{MCDRV_SRC_MIX_BLOCK, MCDRV_SRC6_MIX_ON, MCDRV_SRC6_MIX_OFF},
	{MCDRV_SRC_DTMF_BLOCK, MCDRV_SRC4_DTMF_ON, MCDRV_SRC4_DTMF_OFF}
};




/* virtual registers */
enum {
	SH_MC_ASOC_REG_LOG_ONOFF = 0, /* for debug */

	SH_MC_ASOC_REG_HW_REG_TYPE, /* for debug */
	SH_MC_ASOC_REG_HW_REG_ADDR, /* for debug */
	SH_MC_ASOC_REG_HW_REG_DATA, /* for debug */
	SH_MC_ASOC_REG_HW_CMD, /* for debug */

	SH_MC_ASOC_REG_CDSP_PARAM_CMD, /* for debug */
	SH_MC_ASOC_REG_CDSP_PARAM_00_01, /* for debug */
	SH_MC_ASOC_REG_CDSP_PARAM_02_03, /* for debug */
	SH_MC_ASOC_REG_CDSP_PARAM_04_05, /* for debug */
	SH_MC_ASOC_REG_CDSP_PARAM_06_07, /* for debug */
	SH_MC_ASOC_REG_CDSP_PARAM_08_09, /* for debug */
	SH_MC_ASOC_REG_CDSP_PARAM_10_11, /* for debug */
	SH_MC_ASOC_REG_CDSP_PARAM_12_13, /* for debug */
	SH_MC_ASOC_REG_CDSP_PARAM_14_15, /* for debug */

	SH_MC_ASOC_REG_CDSP_PARAM_ID,

	__SH_MC_ASOC_REG_READONLY_FIRST,
	SH_MC_ASOC_REG_SRCONOFF_DIT0_HW = __SH_MC_ASOC_REG_READONLY_FIRST,
	SH_MC_ASOC_REG_SRCONOFF_DIT2_HW,
	SH_MC_ASOC_REG_SRCONOFF_ADC0L_HW,
	SH_MC_ASOC_REG_SRCONOFF_ADC0R_HW,
	SH_MC_ASOC_REG_SRCONOFF_CDSP0_HW,
	SH_MC_ASOC_REG_SRCONOFF_CDSP1_HW,
	SH_MC_ASOC_REG_SRCONOFF_CDSP2_HW,
	SH_MC_ASOC_REG_SRCONOFF_CDSP3_HW,
	SH_MC_ASOC_REG_SRCONOFF_MIX_HW,
	__SH_MC_ASOC_REG_READONLY_LAST = SH_MC_ASOC_REG_SRCONOFF_MIX_HW,

	__SH_MC_ASOC_REG_SRCONOFF_FIRST,
	SH_MC_ASOC_REG_SRCONOFF_DIT0 = __SH_MC_ASOC_REG_SRCONOFF_FIRST,
	SH_MC_ASOC_REG_SRCONOFF_DIT2,
	SH_MC_ASOC_REG_SRCONOFF_ADC0L,
	SH_MC_ASOC_REG_SRCONOFF_ADC0R,
	SH_MC_ASOC_REG_SRCONOFF_CDSP0,
	SH_MC_ASOC_REG_SRCONOFF_CDSP1,
	SH_MC_ASOC_REG_SRCONOFF_CDSP2,
	SH_MC_ASOC_REG_SRCONOFF_CDSP3,
	SH_MC_ASOC_REG_SRCONOFF_MIX,
	__SH_MC_ASOC_REG_SRCONOFF_LAST = SH_MC_ASOC_REG_SRCONOFF_MIX,

	__SH_MC_ASOC_REG_VOL_FIRST,
	SH_MC_ASOC_REG_DVOL_DIR0 = __SH_MC_ASOC_REG_VOL_FIRST,
	SH_MC_ASOC_REG_DVOL_DIR0_MUTE,
	SH_MC_ASOC_REG_DVOL_DIT0,
	SH_MC_ASOC_REG_DVOL_DIT0_MUTE,
	__SH_MC_ASOC_REG_VOL_LAST = SH_MC_ASOC_REG_DVOL_DIT0_MUTE,

	SH_MC_ASOC_REG_KEEP_PATH,
	SH_MC_ASOC_REG_KEEP_PM_SRCONOFF,

	SH_MC_ASOC_REG_PM_SRCONOFF,

	SH_MC_ASOC_REG_MAX
};

static const u16 sh_mc_asoc_reg_default[SH_MC_ASOC_REG_MAX] = {
	0,
	0,
	0,
	0,
	0,
	/*----- REG_CDSP ----->*/
	0,
	0,
	0,
	0,
	0,
	0,
	0,
	0,
	0,
	0, /* REG_CDSP_PARAM_ID */
	/*<----- REG_CDSP -----*/
	/*----- REG_SRCONOFF_HW ----->*/
	0, /* DIT0 */
	0, /* DIT2 */
	0, /* ADC0L */
	0, /* ADC0R */
	0, /* CDSP0 */
	0, /* CDSP1 */
	0, /* CDSP2 */
	0, /* CDSP3 */
	0, /* MIX */
	/*<----- REG_SRCONOFF_HW -----*/
	/*----- REG_SRCONOFF ----->*/
#if 1
#ifndef SH_MC_ASOC_CDSP_DISABLE
	SH_MC_ASOC_SRC_BIT(SH_MC_ASOC_SRC_MIX), /* DIT0 */
	SH_MC_ASOC_SRC_BIT(SH_MC_ASOC_SRC_CDSP_DIRECT), /* DIT2 */
	0, /* ADC0L */
	SH_MC_ASOC_SRC_BIT(SH_MC_ASOC_SRC_LINE1_R), /* ADC0R */
	SH_MC_ASOC_SRC_BIT(SH_MC_ASOC_SRC_DIR0), /* CDSP0 */
	SH_MC_ASOC_SRC_BIT(SH_MC_ASOC_SRC_DIR0), /* CDSP1 */
	SH_MC_ASOC_SRC_BIT(SH_MC_ASOC_SRC_DIR2_DIRECT), /* CDSP2 */
	SH_MC_ASOC_SRC_BIT(SH_MC_ASOC_SRC_MIX), /* CDSP3 */
	SH_MC_ASOC_SRC_BIT(SH_MC_ASOC_SRC_CDSP), /* MIX */
#else
	SH_MC_ASOC_SRC_BIT(SH_MC_ASOC_SRC_MIX), /* DIT0 */
	SH_MC_ASOC_SRC_BIT(SH_MC_ASOC_SRC_DIR0), /* DIT2 */
	0, /* ADC0L */
	SH_MC_ASOC_SRC_BIT(SH_MC_ASOC_SRC_LINE1_R), /* ADC0R */
	0, /* CDSP0 */
	0, /* CDSP1 */
	0, /* CDSP2 */
	0, /* CDSP3 */
	SH_MC_ASOC_SRC_BIT(SH_MC_ASOC_SRC_DIR2), /* MIX */
#endif
#else
	/*++++++++++ loopback ++++++++++*/
	SH_MC_ASOC_SRC_BIT(SH_MC_ASOC_SRC_DIR0), /* DIT0 */
	0, /* DIT2 */
	0, /* ADC0L */
	0, /* ADC0R */
	0, /* CDSP0 */
	0, /* CDSP1 */
	0, /* CDSP2 */
	0, /* CDSP3 */
	0, /* MIX */
#endif
	/*<----- REG_SRCONOFF -----*/
	/*----- REG_VOL ----->*/
	0, /* DIR0 */
	0, /* DIR0_MUTE */
	0, /* DIT0 */
	0, /* DIT0_MUTE */
	/*<----- REG_VOL -----*/
	0,
	0,
	0
};

#define SH_MC_ASOC_SRCONOFF_SHIFT(reg) ((reg) - __SH_MC_ASOC_REG_SRCONOFF_FIRST)

#define SH_MC_ASOC_SRCONOFF_CDSP_MASK	( \
	(1 << SH_MC_ASOC_SRCONOFF_SHIFT(SH_MC_ASOC_REG_SRCONOFF_CDSP0)) | \
	(1 << SH_MC_ASOC_SRCONOFF_SHIFT(SH_MC_ASOC_REG_SRCONOFF_CDSP1)) | \
	(1 << SH_MC_ASOC_SRCONOFF_SHIFT(SH_MC_ASOC_REG_SRCONOFF_CDSP2)) | \
	(1 << SH_MC_ASOC_SRCONOFF_SHIFT(SH_MC_ASOC_REG_SRCONOFF_CDSP3)))

#define SH_MC_ASOC_SRCONOFF_IS_CDSP_SHIFT(shift)	( \
	((shift) == SH_MC_ASOC_SRCONOFF_SHIFT(SH_MC_ASOC_REG_SRCONOFF_CDSP0)) || \
	((shift) == SH_MC_ASOC_SRCONOFF_SHIFT(SH_MC_ASOC_REG_SRCONOFF_CDSP1)) || \
	((shift) == SH_MC_ASOC_SRCONOFF_SHIFT(SH_MC_ASOC_REG_SRCONOFF_CDSP2)) || \
	((shift) == SH_MC_ASOC_SRCONOFF_SHIFT(SH_MC_ASOC_REG_SRCONOFF_CDSP3)))




struct sh_mc_asoc_set_vol_info {
	int src;
	int reg;
	int reg_mute;
	SINT16 fixed_value;
	size_t offset;
	int num;
};

static const struct sh_mc_asoc_set_vol_info sh_mc_asoc_set_vol_tbl_dit0[] = {
	{SH_MC_ASOC_SRC_DIR0, SH_MC_ASOC_REG_DVOL_DIT0, SH_MC_ASOC_REG_DVOL_DIT0_MUTE, 0,
		offsetof(MCDRV_VOL_INFO, aswD_Dit0), DIO0_VOL_CHANNELS},
	{SH_MC_ASOC_SRC_DIR0, SH_MC_ASOC_REG_DVOL_DIR0, SH_MC_ASOC_REG_DVOL_DIR0_MUTE, 0,
		offsetof(MCDRV_VOL_INFO, aswD_Dir0), DIO0_VOL_CHANNELS},
	{SH_MC_ASOC_SRC_DIR2, SH_MC_ASOC_REG_DVOL_DIT0, SH_MC_ASOC_REG_DVOL_DIT0_MUTE, 0,
		offsetof(MCDRV_VOL_INFO, aswD_Dit0), DIO0_VOL_CHANNELS},
	{SH_MC_ASOC_SRC_DIR2, -1, -1, 0,
		offsetof(MCDRV_VOL_INFO, aswD_Dir2), DIO2_VOL_CHANNELS},
	{SH_MC_ASOC_SRC_CDSP, SH_MC_ASOC_REG_DVOL_DIT0, SH_MC_ASOC_REG_DVOL_DIT0_MUTE, 0,
		offsetof(MCDRV_VOL_INFO, aswD_Dit0), DIO0_VOL_CHANNELS},
	{SH_MC_ASOC_SRC_CDSP, -1, -1, 0xA000,
		offsetof(MCDRV_VOL_INFO, aswD_Dir2[0]), 1},
	{SH_MC_ASOC_SRC_CDSP, -1, -1, 0,
		offsetof(MCDRV_VOL_INFO, aswD_Dir2[1]), 1},
	{SH_MC_ASOC_SRC_MIX, SH_MC_ASOC_REG_DVOL_DIT0, SH_MC_ASOC_REG_DVOL_DIT0_MUTE, 0,
		offsetof(MCDRV_VOL_INFO, aswD_Dit0), DIO0_VOL_CHANNELS},
	{SH_MC_ASOC_SRC_DTMF, SH_MC_ASOC_REG_DVOL_DIT0, SH_MC_ASOC_REG_DVOL_DIT0_MUTE, 0,
		offsetof(MCDRV_VOL_INFO, aswD_Dit0), DIO0_VOL_CHANNELS},
	{SH_MC_ASOC_SRC_DTMF, -1, -1, 0,
		offsetof(MCDRV_VOL_INFO, aswD_Aeng6), AENG6_VOL_CHANNELS},
	{SH_MC_ASOC_SRC_DTMF, -1, -1, 0,
		offsetof(MCDRV_VOL_INFO, aswD_Dtmfb), DTMF_VOL_CHANNELS},
	{-1, -1, -1, 0, 0, 0} /* table end */
};

static const struct sh_mc_asoc_set_vol_info sh_mc_asoc_set_vol_tbl_dit2[] = {
	{SH_MC_ASOC_SRC_DIR0, -1, -1, 0,
		offsetof(MCDRV_VOL_INFO, aswD_Dit2), DIO2_VOL_CHANNELS},
	{SH_MC_ASOC_SRC_DIR0, SH_MC_ASOC_REG_DVOL_DIR0, SH_MC_ASOC_REG_DVOL_DIR0_MUTE, 0,
		offsetof(MCDRV_VOL_INFO, aswD_Dir0), DIO0_VOL_CHANNELS},
	{SH_MC_ASOC_SRC_ADC0, -1, -1, 0,
		offsetof(MCDRV_VOL_INFO, aswD_Dit2), DIO2_VOL_CHANNELS},
	{SH_MC_ASOC_SRC_ADC0, -1, -1, 0,
		offsetof(MCDRV_VOL_INFO, aswD_Aeng6), AENG6_VOL_CHANNELS},
	{SH_MC_ASOC_SRC_ADC0, -1, -1, 0,
		offsetof(MCDRV_VOL_INFO, aswD_Ad0), AD0_VOL_CHANNELS},
	{SH_MC_ASOC_SRC_CDSP_DIRECT, -1, -1, 0,
		offsetof(MCDRV_VOL_INFO, aswD_Dit2), DIO2_VOL_CHANNELS},
	{SH_MC_ASOC_SRC_CDSP_DIRECT, SH_MC_ASOC_REG_DVOL_DIR0, SH_MC_ASOC_REG_DVOL_DIR0_MUTE, 0,
		offsetof(MCDRV_VOL_INFO, aswD_Dir0), DIO0_VOL_CHANNELS},
	{SH_MC_ASOC_SRC_DTMF, -1, -1, 0,
		offsetof(MCDRV_VOL_INFO, aswD_Dit2), DIO2_VOL_CHANNELS},
	{SH_MC_ASOC_SRC_DTMF, -1, -1, 0,
		offsetof(MCDRV_VOL_INFO, aswD_Aeng6), AENG6_VOL_CHANNELS},
	{SH_MC_ASOC_SRC_DTMF, -1, -1, 0,
		offsetof(MCDRV_VOL_INFO, aswD_Dtmfb), DTMF_VOL_CHANNELS},
	{-1, -1, -1, 0, 0, 0} /* table end */
};

static const struct sh_mc_asoc_set_vol_info sh_mc_asoc_set_vol_tbl_adc0[] = {
	{SH_MC_ASOC_SRC_LINE1_M, -1, -1, 0, offsetof(MCDRV_VOL_INFO, aswA_Ad0), AD0_VOL_CHANNELS},
	{SH_MC_ASOC_SRC_LINE1_L, -1, -1, 0, offsetof(MCDRV_VOL_INFO, aswA_Ad0[0]), 1},
	{SH_MC_ASOC_SRC_LINE1_R, -1, -1, 0, offsetof(MCDRV_VOL_INFO, aswA_Ad0[1]), 1},
	{-1, -1, -1, 0, 0, 0} /* table end */
};

static const struct sh_mc_asoc_set_vol_info sh_mc_asoc_set_vol_tbl_cdsp[] = {
	{SH_MC_ASOC_SRC_MIX, -1, -1, 0, offsetof(MCDRV_VOL_INFO, aswD_Dit21), DIO2_VOL_CHANNELS},
	{-1, -1, -1, 0, 0, 0} /* table end */
};

static const struct sh_mc_asoc_set_vol_info sh_mc_asoc_set_vol_tbl_mix[] = {
	{SH_MC_ASOC_SRC_DIR0, -1, -1, 0, offsetof(MCDRV_VOL_INFO, aswD_Dir0Att), DIO0_VOL_CHANNELS},
	{SH_MC_ASOC_SRC_DIR0, SH_MC_ASOC_REG_DVOL_DIR0, SH_MC_ASOC_REG_DVOL_DIR0_MUTE, 0,
		offsetof(MCDRV_VOL_INFO, aswD_Dir0), DIO0_VOL_CHANNELS},
	{SH_MC_ASOC_SRC_DIR2, -1, -1, 0, offsetof(MCDRV_VOL_INFO, aswD_Dir2Att), DIO2_VOL_CHANNELS},
	{SH_MC_ASOC_SRC_DIR2, -1, -1, 0, offsetof(MCDRV_VOL_INFO, aswD_Dir2), DIO2_VOL_CHANNELS},
	{SH_MC_ASOC_SRC_ADC0, -1, -1, 0, offsetof(MCDRV_VOL_INFO, aswD_Ad0Att), AD0_VOL_CHANNELS},
	{SH_MC_ASOC_SRC_ADC0, -1, -1, 0, offsetof(MCDRV_VOL_INFO, aswD_Aeng6), AENG6_VOL_CHANNELS},
	{SH_MC_ASOC_SRC_ADC0, -1, -1, 0, offsetof(MCDRV_VOL_INFO, aswD_Ad0), AD0_VOL_CHANNELS},
	{SH_MC_ASOC_SRC_CDSP, -1, -1, 0, offsetof(MCDRV_VOL_INFO, aswD_Dir2Att), DIO2_VOL_CHANNELS},
	{SH_MC_ASOC_SRC_CDSP, -1, -1, 0xA000, offsetof(MCDRV_VOL_INFO, aswD_Dir2[0]), 1},
	{SH_MC_ASOC_SRC_CDSP, -1, -1, 0, offsetof(MCDRV_VOL_INFO, aswD_Dir2[1]), 1},
	{SH_MC_ASOC_SRC_DTMF, -1, -1, 0, offsetof(MCDRV_VOL_INFO, aswD_Ad0Att), AD0_VOL_CHANNELS},
	{SH_MC_ASOC_SRC_DTMF, -1, -1, 0, offsetof(MCDRV_VOL_INFO, aswD_Aeng6), AENG6_VOL_CHANNELS},
	{SH_MC_ASOC_SRC_DTMF, -1, -1, 0, offsetof(MCDRV_VOL_INFO, aswD_Dtmfb), DTMF_VOL_CHANNELS},
	{-1, -1, -1, 0, 0, 0} /* table end */
};

struct sh_mc_asoc_set_path_info {
	int src_onoff_reg;
	int src_onoff_reg_hw;
	int src_onoff_shift;
	size_t channel_offset;
	int channel_num;
	const struct sh_mc_asoc_set_vol_info *set_vol_info;
};

static const struct sh_mc_asoc_set_path_info sh_mc_asoc_set_path_tbl[] = {
	{SH_MC_ASOC_REG_SRCONOFF_DIT0, SH_MC_ASOC_REG_SRCONOFF_DIT0_HW, SH_MC_ASOC_SRCONOFF_SHIFT(SH_MC_ASOC_REG_SRCONOFF_DIT0),
		offsetof(MCDRV_PATH_INFO, asDit0), DIT0_PATH_CHANNELS, sh_mc_asoc_set_vol_tbl_dit0},
	{SH_MC_ASOC_REG_SRCONOFF_DIT2, SH_MC_ASOC_REG_SRCONOFF_DIT2_HW, SH_MC_ASOC_SRCONOFF_SHIFT(SH_MC_ASOC_REG_SRCONOFF_DIT2),
		offsetof(MCDRV_PATH_INFO, asDit2), DIT2_PATH_CHANNELS, sh_mc_asoc_set_vol_tbl_dit2},
	{SH_MC_ASOC_REG_SRCONOFF_ADC0L, SH_MC_ASOC_REG_SRCONOFF_ADC0L_HW, SH_MC_ASOC_SRCONOFF_SHIFT(SH_MC_ASOC_REG_SRCONOFF_ADC0L),
		offsetof(MCDRV_PATH_INFO, asAdc0[0]), 1, sh_mc_asoc_set_vol_tbl_adc0},
	{SH_MC_ASOC_REG_SRCONOFF_ADC0R, SH_MC_ASOC_REG_SRCONOFF_ADC0R_HW, SH_MC_ASOC_SRCONOFF_SHIFT(SH_MC_ASOC_REG_SRCONOFF_ADC0R),
		offsetof(MCDRV_PATH_INFO, asAdc0[1]), 1, sh_mc_asoc_set_vol_tbl_adc0},
	{SH_MC_ASOC_REG_SRCONOFF_CDSP0, SH_MC_ASOC_REG_SRCONOFF_CDSP0_HW, SH_MC_ASOC_SRCONOFF_SHIFT(SH_MC_ASOC_REG_SRCONOFF_CDSP0),
		offsetof(MCDRV_PATH_INFO, asCdsp[0]), 1, sh_mc_asoc_set_vol_tbl_cdsp},
	{SH_MC_ASOC_REG_SRCONOFF_CDSP1, SH_MC_ASOC_REG_SRCONOFF_CDSP1_HW, SH_MC_ASOC_SRCONOFF_SHIFT(SH_MC_ASOC_REG_SRCONOFF_CDSP1),
		offsetof(MCDRV_PATH_INFO, asCdsp[1]), 1, sh_mc_asoc_set_vol_tbl_cdsp},
	{SH_MC_ASOC_REG_SRCONOFF_CDSP2, SH_MC_ASOC_REG_SRCONOFF_CDSP2_HW, SH_MC_ASOC_SRCONOFF_SHIFT(SH_MC_ASOC_REG_SRCONOFF_CDSP2),
		offsetof(MCDRV_PATH_INFO, asCdsp[2]), 1, sh_mc_asoc_set_vol_tbl_cdsp},
	{SH_MC_ASOC_REG_SRCONOFF_CDSP3, SH_MC_ASOC_REG_SRCONOFF_CDSP3_HW, SH_MC_ASOC_SRCONOFF_SHIFT(SH_MC_ASOC_REG_SRCONOFF_CDSP3),
		offsetof(MCDRV_PATH_INFO, asCdsp[3]), 1, sh_mc_asoc_set_vol_tbl_cdsp},
	{SH_MC_ASOC_REG_SRCONOFF_MIX, SH_MC_ASOC_REG_SRCONOFF_MIX_HW, SH_MC_ASOC_SRCONOFF_SHIFT(SH_MC_ASOC_REG_SRCONOFF_MIX),
		offsetof(MCDRV_PATH_INFO, asMix), MIX_PATH_CHANNELS, sh_mc_asoc_set_vol_tbl_mix},
	{-1, -1, 0, 0, 0} /* table end */
};




static const DECLARE_TLV_DB_SCALE(sh_mc_asoc_tlv_digital_volume, 0, 1, 1);

static const struct snd_kcontrol_new sh_mc_asoc_snd_controls[] = {
	SOC_SINGLE_S8_TLV("MC DIR0 Volume", SH_MC_ASOC_REG_DVOL_DIR0, -74, 18, sh_mc_asoc_tlv_digital_volume),
	SOC_SINGLE("MC DIR0 Mute", SH_MC_ASOC_REG_DVOL_DIR0_MUTE, 0, 1, 0),
	SOC_SINGLE_S8_TLV("MC DIT0 Volume", SH_MC_ASOC_REG_DVOL_DIT0, -74, 18, sh_mc_asoc_tlv_digital_volume),
	SOC_SINGLE("MC DIT0 Mute", SH_MC_ASOC_REG_DVOL_DIT0_MUTE, 0, 1, 0),
	SOC_SINGLE("MC CDSP ParamID", SH_MC_ASOC_REG_CDSP_PARAM_ID, 0, 0xffff, 0),
	SOC_SINGLE("MC Keep Path", SH_MC_ASOC_REG_KEEP_PATH, 0, 1, 0),
};

static const struct snd_kcontrol_new sh_mc_asoc_dapm_ctrl_dit0_src_onoff[] = {
	SOC_DAPM_SINGLE("DIR0", SH_MC_ASOC_REG_SRCONOFF_DIT0, SH_MC_ASOC_SRC_DIR0, 1, 0),
	SOC_DAPM_SINGLE("DIR2", SH_MC_ASOC_REG_SRCONOFF_DIT0, SH_MC_ASOC_SRC_DIR2, 1, 0),
	SOC_DAPM_SINGLE("MIX", SH_MC_ASOC_REG_SRCONOFF_DIT0, SH_MC_ASOC_SRC_MIX, 1, 0),
#ifndef SH_MC_ASOC_CDSP_DISABLE
	SOC_DAPM_SINGLE("CDSP", SH_MC_ASOC_REG_SRCONOFF_DIT0, SH_MC_ASOC_SRC_CDSP, 1, 0),
#else
	SOC_DAPM_SINGLE("CDSP", SH_MC_ASOC_REG_SRCONOFF_DIT0, SH_MC_ASOC_SRC_DIR2, 1, 0),
#endif
	SOC_DAPM_SINGLE("DTMF", SH_MC_ASOC_REG_SRCONOFF_DIT0, SH_MC_ASOC_SRC_DTMF, 1, 0),
};

static const struct snd_kcontrol_new sh_mc_asoc_dapm_ctrl_dit2_src_onoff[] = {
	SOC_DAPM_SINGLE("DIR0", SH_MC_ASOC_REG_SRCONOFF_DIT2, SH_MC_ASOC_SRC_DIR0, 1, 0),
	SOC_DAPM_SINGLE("ADC0", SH_MC_ASOC_REG_SRCONOFF_DIT2, SH_MC_ASOC_SRC_ADC0, 1, 0),
#ifndef SH_MC_ASOC_CDSP_DISABLE
	SOC_DAPM_SINGLE("CDSP", SH_MC_ASOC_REG_SRCONOFF_DIT2, SH_MC_ASOC_SRC_CDSP_DIRECT, 1, 0),
#else
	SOC_DAPM_SINGLE("CDSP", SH_MC_ASOC_REG_SRCONOFF_DIT2, SH_MC_ASOC_SRC_DIR0, 1, 0),
#endif
	SOC_DAPM_SINGLE("DTMF", SH_MC_ASOC_REG_SRCONOFF_DIT2, SH_MC_ASOC_SRC_DTMF, 1, 0),
};

static const struct snd_kcontrol_new sh_mc_asoc_dapm_ctrl_adc0L_src_onoff[] = {
	SOC_DAPM_SINGLE("LINE1", SH_MC_ASOC_REG_SRCONOFF_ADC0L, SH_MC_ASOC_SRC_LINE1_M, 1, 0),
	SOC_DAPM_SINGLE("LINE1L", SH_MC_ASOC_REG_SRCONOFF_ADC0L, SH_MC_ASOC_SRC_LINE1_L, 1, 0),
};

static const struct snd_kcontrol_new sh_mc_asoc_dapm_ctrl_adc0R_src_onoff[] = {
	SOC_DAPM_SINGLE("LINE1", SH_MC_ASOC_REG_SRCONOFF_ADC0R, SH_MC_ASOC_SRC_LINE1_M, 1, 0),
	SOC_DAPM_SINGLE("LINE1R", SH_MC_ASOC_REG_SRCONOFF_ADC0R, SH_MC_ASOC_SRC_LINE1_R, 1, 0),
};

static const struct snd_kcontrol_new sh_mc_asoc_dapm_ctrl_cdsp0_src_onoff[] = {
	SOC_DAPM_SINGLE("DIR0", SH_MC_ASOC_REG_SRCONOFF_CDSP0, SH_MC_ASOC_SRC_DIR0, 1, 0),
};

static const struct snd_kcontrol_new sh_mc_asoc_dapm_ctrl_cdsp1_src_onoff[] = {
	SOC_DAPM_SINGLE("DIR0", SH_MC_ASOC_REG_SRCONOFF_CDSP1, SH_MC_ASOC_SRC_DIR0, 1, 0),
};

static const struct snd_kcontrol_new sh_mc_asoc_dapm_ctrl_cdsp2_src_onoff[] = {
	SOC_DAPM_SINGLE("DIR2", SH_MC_ASOC_REG_SRCONOFF_CDSP2, SH_MC_ASOC_SRC_DIR2_DIRECT, 1, 0),
};

static const struct snd_kcontrol_new sh_mc_asoc_dapm_ctrl_cdsp3_src_onoff[] = {
	SOC_DAPM_SINGLE("MIX", SH_MC_ASOC_REG_SRCONOFF_CDSP3, SH_MC_ASOC_SRC_MIX, 1, 0),
};

static const struct snd_kcontrol_new sh_mc_asoc_dapm_ctrl_mix_src_onoff[] = {
	SOC_DAPM_SINGLE("DIR0", SH_MC_ASOC_REG_SRCONOFF_MIX, SH_MC_ASOC_SRC_DIR0, 1, 0),
	SOC_DAPM_SINGLE("DIR2", SH_MC_ASOC_REG_SRCONOFF_MIX, SH_MC_ASOC_SRC_DIR2, 1, 0),
	SOC_DAPM_SINGLE("ADC0", SH_MC_ASOC_REG_SRCONOFF_MIX, SH_MC_ASOC_SRC_ADC0, 1, 0),
#ifndef SH_MC_ASOC_CDSP_DISABLE
	SOC_DAPM_SINGLE("CDSP", SH_MC_ASOC_REG_SRCONOFF_MIX, SH_MC_ASOC_SRC_CDSP, 1, 0),
#else
	SOC_DAPM_SINGLE("CDSP", SH_MC_ASOC_REG_SRCONOFF_MIX, SH_MC_ASOC_SRC_DIR2, 1, 0),
#endif
	SOC_DAPM_SINGLE("DTMF", SH_MC_ASOC_REG_SRCONOFF_MIX, SH_MC_ASOC_SRC_DTMF, 1, 0),
};

static const struct snd_soc_dapm_widget sh_mc_asoc_dapm_widgets[] = {
	SND_SOC_DAPM_AIF_IN("MC DIR0", "MC Playback", 0, SND_SOC_NOPM, 0, 0),
	SND_SOC_DAPM_AIF_OUT("MC DIT0", "MC Capture", 0, SND_SOC_NOPM, 0, 0),

	SND_SOC_DAPM_MIXER("MC DIT0 SrcOnOff",
		SH_MC_ASOC_REG_PM_SRCONOFF, SH_MC_ASOC_SRCONOFF_SHIFT(SH_MC_ASOC_REG_SRCONOFF_DIT0), 0,
		sh_mc_asoc_dapm_ctrl_dit0_src_onoff, ARRAY_SIZE(sh_mc_asoc_dapm_ctrl_dit0_src_onoff)),
	SND_SOC_DAPM_MIXER("MC DIT2 SrcOnOff",
		SH_MC_ASOC_REG_PM_SRCONOFF, SH_MC_ASOC_SRCONOFF_SHIFT(SH_MC_ASOC_REG_SRCONOFF_DIT2), 0,
		sh_mc_asoc_dapm_ctrl_dit2_src_onoff, ARRAY_SIZE(sh_mc_asoc_dapm_ctrl_dit2_src_onoff)),
	SND_SOC_DAPM_MIXER("MC ADC0L SrcOnOff",
		SH_MC_ASOC_REG_PM_SRCONOFF, SH_MC_ASOC_SRCONOFF_SHIFT(SH_MC_ASOC_REG_SRCONOFF_ADC0L), 0,
		sh_mc_asoc_dapm_ctrl_adc0L_src_onoff, ARRAY_SIZE(sh_mc_asoc_dapm_ctrl_adc0L_src_onoff)),
	SND_SOC_DAPM_MIXER("MC ADC0R SrcOnOff",
		SH_MC_ASOC_REG_PM_SRCONOFF, SH_MC_ASOC_SRCONOFF_SHIFT(SH_MC_ASOC_REG_SRCONOFF_ADC0R), 0,
		sh_mc_asoc_dapm_ctrl_adc0R_src_onoff, ARRAY_SIZE(sh_mc_asoc_dapm_ctrl_adc0R_src_onoff)),
	SND_SOC_DAPM_MIXER("MC CDSP0 SrcOnOff",
		SH_MC_ASOC_REG_PM_SRCONOFF, SH_MC_ASOC_SRCONOFF_SHIFT(SH_MC_ASOC_REG_SRCONOFF_CDSP0), 0,
		sh_mc_asoc_dapm_ctrl_cdsp0_src_onoff, ARRAY_SIZE(sh_mc_asoc_dapm_ctrl_cdsp0_src_onoff)),
	SND_SOC_DAPM_MIXER("MC CDSP1 SrcOnOff",
		SH_MC_ASOC_REG_PM_SRCONOFF, SH_MC_ASOC_SRCONOFF_SHIFT(SH_MC_ASOC_REG_SRCONOFF_CDSP1), 0,
		sh_mc_asoc_dapm_ctrl_cdsp1_src_onoff, ARRAY_SIZE(sh_mc_asoc_dapm_ctrl_cdsp1_src_onoff)),
	SND_SOC_DAPM_MIXER("MC CDSP2 SrcOnOff",
		SH_MC_ASOC_REG_PM_SRCONOFF, SH_MC_ASOC_SRCONOFF_SHIFT(SH_MC_ASOC_REG_SRCONOFF_CDSP2), 0,
		sh_mc_asoc_dapm_ctrl_cdsp2_src_onoff, ARRAY_SIZE(sh_mc_asoc_dapm_ctrl_cdsp2_src_onoff)),
	SND_SOC_DAPM_MIXER("MC CDSP3 SrcOnOff",
		SH_MC_ASOC_REG_PM_SRCONOFF, SH_MC_ASOC_SRCONOFF_SHIFT(SH_MC_ASOC_REG_SRCONOFF_CDSP3), 0,
		sh_mc_asoc_dapm_ctrl_cdsp3_src_onoff, ARRAY_SIZE(sh_mc_asoc_dapm_ctrl_cdsp3_src_onoff)),
	SND_SOC_DAPM_MIXER("MC MIX SrcOnOff",
		SH_MC_ASOC_REG_PM_SRCONOFF, SH_MC_ASOC_SRCONOFF_SHIFT(SH_MC_ASOC_REG_SRCONOFF_MIX), 0,
		sh_mc_asoc_dapm_ctrl_mix_src_onoff, ARRAY_SIZE(sh_mc_asoc_dapm_ctrl_mix_src_onoff)),

	SND_SOC_DAPM_INPUT("MC DTMF"),
	SND_SOC_DAPM_INPUT("MC LineIn1 L"),
	SND_SOC_DAPM_INPUT("MC LineIn1 R"),
	SND_SOC_DAPM_INPUT("MC DIR2"),
	SND_SOC_DAPM_OUTPUT("MC DIT2"),
};

static const struct snd_soc_dapm_route sh_mc_asoc_intercon[] = {
	{"MC CDSP0 SrcOnOff", "DIR0", "MC DIR0"},
	{"MC CDSP1 SrcOnOff", "DIR0", "MC DIR0"},
	{"MC DIT2 SrcOnOff", "DIR0", "MC DIR0"},
	{"MC DIT2 SrcOnOff", "ADC0", "MC ADC0L SrcOnOff"},
	{"MC DIT2 SrcOnOff", "ADC0", "MC ADC0R SrcOnOff"},
#ifndef SH_MC_ASOC_CDSP_DISABLE
	{"MC DIT2 SrcOnOff", "CDSP", "MC CDSP0 SrcOnOff"},
	{"MC DIT2 SrcOnOff", "CDSP", "MC CDSP1 SrcOnOff"},
#else
	{"MC DIT2 SrcOnOff", "CDSP", "MC DIR0"},
#endif
	{"MC DIT2 SrcOnOff", "DTMF", "MC DTMF"},
	{"MC DIT2", NULL, "MC DIT2 SrcOnOff"},

	{"MC DIT0", NULL, "MC DIT0 SrcOnOff"},
	{"MC DIT0 SrcOnOff", "DIR0", "MC DIR0"},
	{"MC DIT0 SrcOnOff", "DIR2", "MC DIR2"},
	{"MC DIT0 SrcOnOff", "MIX", "MC MIX SrcOnOff"},
#ifndef SH_MC_ASOC_CDSP_DISABLE
	{"MC DIT0 SrcOnOff", "CDSP", "MC CDSP2 SrcOnOff"},
	{"MC DIT0 SrcOnOff", "CDSP", "MC CDSP3 SrcOnOff"},
#else
	{"MC DIT0 SrcOnOff", "CDSP", "MC DIR2"},
#endif
	{"MC DIT0 SrcOnOff", "DTMF", "MC DTMF"},
	{"MC MIX SrcOnOff", "DIR0", "MC DIR0"},
	{"MC MIX SrcOnOff", "DIR2", "MC DIR2"},
	{"MC MIX SrcOnOff", "ADC0", "MC ADC0L SrcOnOff"},
	{"MC MIX SrcOnOff", "ADC0", "MC ADC0R SrcOnOff"},
#ifndef SH_MC_ASOC_CDSP_DISABLE
	{"MC MIX SrcOnOff", "CDSP", "MC CDSP2 SrcOnOff"},
	{"MC MIX SrcOnOff", "CDSP", "MC CDSP3 SrcOnOff"},
#else
	{"MC MIX SrcOnOff", "CDSP", "MC DIR2"},
#endif
	{"MC MIX SrcOnOff", "DTMF", "MC DTMF"},
	{"MC ADC0L SrcOnOff", "LINE1", "MC LineIn1 L"},
	{"MC ADC0L SrcOnOff", "LINE1L", "MC LineIn1 L"},
	{"MC ADC0R SrcOnOff", "LINE1", "MC LineIn1 R"},
	{"MC ADC0R SrcOnOff", "LINE1R", "MC LineIn1 R"},
	{"MC CDSP2 SrcOnOff", "DIR2", "MC DIR2"},
	{"MC CDSP3 SrcOnOff", "MIX", "MC MIX SrcOnOff"},
};




static int sh_mc_asoc_reg_cache_read(struct snd_soc_codec *codec, unsigned int reg, unsigned int *val)
{
	int ret = 0;
	if(reg >= SH_MC_ASOC_REG_MAX) {
		dev_err(codec->dev, "invalid (reg=0x%x)\n", reg);
		return -EINVAL;
	}
	ret = snd_soc_cache_read(codec, reg, val);
	if(ret < 0) {
		dev_err(codec->dev, "snd_soc_cache_read failed (reg=0x%x, err=%d)\n", reg, ret);
	}
	return ret;
}

static int sh_mc_asoc_reg_cache_write(struct snd_soc_codec *codec, unsigned int reg, unsigned int value)
{
	int ret = 0;
	if(reg >= SH_MC_ASOC_REG_MAX) {
		dev_err(codec->dev, "invalid (reg=0x%x)\n", reg);
		return -EINVAL;
	}
	dev_dbg(codec->dev, "reg_write  reg=0x%x, value=0x%x\n", reg, value);
	ret = snd_soc_cache_write(codec, reg, value);
	if(ret < 0) {
		dev_err(codec->dev, "snd_soc_cache_write failed (reg=0x%x, err=%d)\n", reg, ret);
	}
	return ret;
}




static void sh_mc_asoc_hw_cmd_log(struct snd_soc_codec *codec, UINT32 dCmd, void* pvPrm, UINT32 dPrm)
{
	unsigned int log_onoff_flags;
	if(sh_mc_asoc_reg_cache_read(codec, SH_MC_ASOC_REG_LOG_ONOFF, &log_onoff_flags) < 0) {
		return;
	}
	if(!log_onoff_flags) {
		return;
	}

	switch(dCmd)
	{
		case MCDRV_GET_PATH:
		case MCDRV_SET_PATH:
			if(log_onoff_flags & 0x0001) {
				MCDRV_CHANNEL *channels = (MCDRV_CHANNEL *)pvPrm;
				int i;
				for(i = 0; i < (sizeof(MCDRV_PATH_INFO) / sizeof(MCDRV_CHANNEL)); i++) {
					dev_dbg(codec->dev, "src_onoff[%d] {0x%02x,0x%02x,0x%02x,0x%02x,0x%02x,0x%02x,0x%02x}\n", i,
						channels[i].abSrcOnOff[0], channels[i].abSrcOnOff[1], channels[i].abSrcOnOff[2],
						channels[i].abSrcOnOff[3], channels[i].abSrcOnOff[4], channels[i].abSrcOnOff[5],
						channels[i].abSrcOnOff[6]);
				}
			}
			break;

		case MCDRV_GET_VOLUME:
		case MCDRV_SET_VOLUME:
			if(log_onoff_flags & 0x0002) {
				SINT16 *vols = (SINT16 *)pvPrm;
				int i;
				for(i = 0; i < (sizeof(MCDRV_VOL_INFO) / sizeof(SINT16)) / 4; i++) {
					dev_dbg(codec->dev, "vol[%2d-%2d] {0x%04x,0x%04x,0x%04x,0x%04x}\n",
						i * 4, (i * 4) + 3, (UINT16)vols[0], (UINT16)vols[1], (UINT16)vols[2], (UINT16)vols[3]);
					vols += 4;
				}
				switch((sizeof(MCDRV_VOL_INFO) / sizeof(SINT16)) % 4) {
					case 3:
						dev_dbg(codec->dev, "vol[%2d-%2d] {0x%04x,0x%04x,0x%04x}\n",
							i * 4, (i * 4) + 2, (UINT16)vols[0], (UINT16)vols[1], (UINT16)vols[2]);
						break;
					case 2:
						dev_dbg(codec->dev, "vol[%2d-%2d] {0x%04x,0x%04x}\n",
							i * 4, (i * 4) + 1, (UINT16)vols[0], (UINT16)vols[1]);
						break;
					case 1:
						dev_dbg(codec->dev, "vol[%2d-%2d] {0x%04x}\n",
							i * 4, (i * 4) + 0, (UINT16)vols[0]);
						break;
				}
			}
			break;

		case MCDRV_SET_CDSP_PARAM:
			if(log_onoff_flags & 0x0004) {
				MCDRV_CDSPPARAM *cdsp_param = (MCDRV_CDSPPARAM *)pvPrm;
				dev_dbg(codec->dev,
					"cdsp {command=0x%02x, "
					"param={0x%02x,0x%02x,0x%02x,0x%02x,0x%02x,0x%02x,0x%02x,0x%02x,"
					"0x%02x,0x%02x,0x%02x,0x%02x,0x%02x,0x%02x,0x%02x,0x%02x}}\n",
					cdsp_param->bCommand,
					cdsp_param->abParam[0], cdsp_param->abParam[1], cdsp_param->abParam[2], cdsp_param->abParam[3],
					cdsp_param->abParam[4], cdsp_param->abParam[5], cdsp_param->abParam[6], cdsp_param->abParam[7],
					cdsp_param->abParam[8], cdsp_param->abParam[9], cdsp_param->abParam[10], cdsp_param->abParam[11],
					cdsp_param->abParam[12], cdsp_param->abParam[13], cdsp_param->abParam[14], cdsp_param->abParam[15]);
			}
			break;

		case MCDRV_GET_DIGITALIO:
		case MCDRV_SET_DIGITALIO:
			if(log_onoff_flags & 0x8008) {
				UINT32 uf_com[IOPORT_NUM] = {MCDRV_DIO0_COM_UPDATE_FLAG, MCDRV_DIO1_COM_UPDATE_FLAG, MCDRV_DIO2_COM_UPDATE_FLAG};
				UINT32 uf_dir[IOPORT_NUM] = {MCDRV_DIO0_DIR_UPDATE_FLAG, MCDRV_DIO1_DIR_UPDATE_FLAG, MCDRV_DIO2_DIR_UPDATE_FLAG};
				UINT32 uf_dit[IOPORT_NUM] = {MCDRV_DIO0_DIT_UPDATE_FLAG, MCDRV_DIO1_DIT_UPDATE_FLAG, MCDRV_DIO2_DIT_UPDATE_FLAG};
				MCDRV_DIO_INFO *dio = (MCDRV_DIO_INFO *)pvPrm;
				int i;
				for(i = 0; i < IOPORT_NUM; i++) {
					if(dPrm & uf_com[i]) {
						dev_dbg(codec->dev,
							"dio[%d] COM {0x%02x,0x%02x,0x%02x,0x%02x,0x%02x,0x%02x,0x%02x,0x%02x,0x%02x,0x%02x}\n", i,
							dio->asPortInfo[i].sDioCommon.bMasterSlave, dio->asPortInfo[i].sDioCommon.bAutoFs,
							dio->asPortInfo[i].sDioCommon.bFs, dio->asPortInfo[i].sDioCommon.bBckFs,
							dio->asPortInfo[i].sDioCommon.bInterface, dio->asPortInfo[i].sDioCommon.bBckInvert,
							dio->asPortInfo[i].sDioCommon.bPcmHizTim, dio->asPortInfo[i].sDioCommon.bPcmClkDown,
							dio->asPortInfo[i].sDioCommon.bPcmFrame, dio->asPortInfo[i].sDioCommon.bPcmHighPeriod);
					}
					if(dPrm & uf_dir[i]) {
						dev_dbg(codec->dev,
							"dio[%d] DIR {0x%04x,{0x%02x,0x%02x},{0x%02x,0x%02x,0x%02x,0x%02x},0x%02x,0x%02x}\n", i,
							dio->asPortInfo[i].sDir.wSrcRate,
							dio->asPortInfo[i].sDir.sDaFormat.bBitSel, dio->asPortInfo[i].sDir.sDaFormat.bMode,
							dio->asPortInfo[i].sDir.sPcmFormat.bMono, dio->asPortInfo[i].sDir.sPcmFormat.bOrder,
							dio->asPortInfo[i].sDir.sPcmFormat.bLaw, dio->asPortInfo[i].sDir.sPcmFormat.bBitSel,
							dio->asPortInfo[i].sDir.abSlot[0], dio->asPortInfo[i].sDir.abSlot[1]);
					}
					if(dPrm & uf_dit[i]) {
						dev_dbg(codec->dev,
							"dio[%d] DIT {0x%04x,{0x%02x,0x%02x},{0x%02x,0x%02x,0x%02x,0x%02x},0x%02x,0x%02x}\n", i,
							dio->asPortInfo[i].sDit.wSrcRate,
							dio->asPortInfo[i].sDit.sDaFormat.bBitSel, dio->asPortInfo[i].sDit.sDaFormat.bMode,
							dio->asPortInfo[i].sDit.sPcmFormat.bMono, dio->asPortInfo[i].sDit.sPcmFormat.bOrder,
							dio->asPortInfo[i].sDit.sPcmFormat.bLaw, dio->asPortInfo[i].sDit.sPcmFormat.bBitSel,
							dio->asPortInfo[i].sDit.abSlot[0], dio->asPortInfo[i].sDit.abSlot[1]);
					}
				}
			}
			break;

		case MCDRV_SET_PLL:
			if(log_onoff_flags & 0x8000) {
				MCDRV_PLL_INFO *pll = (MCDRV_PLL_INFO *)pvPrm;
				dev_dbg(codec->dev, "pll {0x%02x,0x%02x,0x%04x,0x%04x,0x%02x,0x%02x,0x%04x,0x%04x}\n",
					pll->bMode0, pll->bPrevDiv0, pll->wFbDiv0, pll->wFrac0,
					pll->bMode1, pll->bPrevDiv1, pll->wFbDiv1, pll->wFrac1);
			}
			break;

		case MCDRV_INIT:
			if(log_onoff_flags & 0x8000) {
				MCDRV_INIT_INFO *ini = (MCDRV_INIT_INFO *)pvPrm;
				dev_dbg(codec->dev, "init {0x%02x,0x%02x,0x%02x,0x%02x,0x%02x,0x%02x,0x%02x,0x%02x,0x%02x,0x%02x,\n",
					ini->bCkSel, ini->bDivR0, ini->bDivF0, ini->bDivR1, ini->bDivF1,
					ini->bRange0, ini->bRange1, ini->bBypass, ini->bDioSdo0Hiz, ini->bDioSdo1Hiz);
				dev_dbg(codec->dev, "      0x%02x,0x%02x,0x%02x,0x%02x,0x%02x,0x%02x,0x%02x,0x%02x,0x%02x,0x%02x,\n",
					ini->bDioSdo2Hiz, ini->bDioClk0Hiz, ini->bDioClk1Hiz, ini->bDioClk2Hiz, ini->bPcmHiz,
					ini->bLineIn1Dif, ini->bLineIn2Dif, ini->bLineOut1Dif, ini->bLineOut2Dif, ini->bSpmn);
				dev_dbg(codec->dev, "      0x%02x,0x%02x,0x%02x,0x%02x,0x%02x,0x%02x,0x%02x,0x%02x,0x%02x,0x%02x,\n",
					ini->bMic1Sng, ini->bMic2Sng, ini->bMic3Sng, ini->bPowerMode, ini->bSpHiz,
					ini->bLdo, ini->bPad0Func, ini->bPad1Func, ini->bPad2Func, ini->bAvddLev);
				dev_dbg(codec->dev, "      0x%02x,0x%02x,0x%02x,0x%02x,0x%02x,0x%02x,0x%02x,0x%02x,0x%02x,\n",
					ini->bVrefLev, ini->bDclGain, ini->bDclLimit, ini->bCpMod, ini->bSdDs,
					ini->bHpIdle, ini->bCLKI1, ini->bMbsDisch, ini->bReserved);
				dev_dbg(codec->dev, "      {0x%08x,0x%08x,0x%08x,0x%08x,0x%08x,\n",
					(unsigned int)ini->sWaitTime.dAdHpf, (unsigned int)ini->sWaitTime.dMic1Cin,
					(unsigned int)ini->sWaitTime.dMic2Cin, (unsigned int)ini->sWaitTime.dMic3Cin,
					(unsigned int)ini->sWaitTime.dLine1Cin);
				dev_dbg(codec->dev, "       0x%08x,0x%08x,0x%08x,0x%08x,0x%08x,\n",
					(unsigned int)ini->sWaitTime.dLine2Cin, (unsigned int)ini->sWaitTime.dVrefRdy1,
					(unsigned int)ini->sWaitTime.dVrefRdy2, (unsigned int)ini->sWaitTime.dHpRdy,
					(unsigned int)ini->sWaitTime.dSpRdy);
				dev_dbg(codec->dev, "       0x%08x,0x%08x,0x%08x,0x%08x,0x%08x}}\n",
					(unsigned int)ini->sWaitTime.dPdm, (unsigned int)ini->sWaitTime.dAnaRdyInterval,
					(unsigned int)ini->sWaitTime.dSvolInterval, (unsigned int)ini->sWaitTime.dAnaRdyTimeOut,
					(unsigned int)ini->sWaitTime.dSvolTimeOut);
			}
			break;

		case MCDRV_SET_ADC:
			if(log_onoff_flags & 0x8000) {
				MCDRV_ADC_INFO *adc = (MCDRV_ADC_INFO *)pvPrm;
				dev_dbg(codec->dev, "adc {0x%02x,0x%02x,0x%02x}\n", adc->bAgcAdjust, adc->bAgcOn, adc->bMono);
			}
			break;
	}
}




static int sh_mc_asoc_hw_setup(struct snd_soc_codec *codec)
{
	signed long hw_ret;

	dev_dbg(codec->dev, "----- setup -----\n");

	hw_ret = McDrv_Ctrl(MCDRV_SET_PLL, (void *)&sh_mc_asoc_hw_pll_info, 0);

	sh_mc_asoc_hw_cmd_log(codec, MCDRV_SET_PLL, (void *)&sh_mc_asoc_hw_pll_info, 0);

	if(hw_ret != MCDRV_SUCCESS) {
		dev_err(codec->dev, "McDrv_Ctrl(MCDRV_SET_PLL) failed (ret=%d)\n", (int)hw_ret);
		return -EIO;
	}


	hw_ret = McDrv_Ctrl(MCDRV_INIT, (void *)&sh_mc_asoc_hw_init_info, 0);

	sh_mc_asoc_hw_cmd_log(codec, MCDRV_INIT, (void *)&sh_mc_asoc_hw_init_info, 0);

	if(hw_ret != MCDRV_SUCCESS) {
		dev_err(codec->dev, "McDrv_Ctrl(MCDRV_INIT) failed (ret=%d)\n", (int)hw_ret);
		return -EIO;
	}


	hw_ret = McDrv_Ctrl(MCDRV_SET_DIGITALIO, (void *)&sh_mc_asoc_hw_dio_info,
		MCDRV_DIO0_COM_UPDATE_FLAG | MCDRV_DIO0_DIR_UPDATE_FLAG | MCDRV_DIO0_DIT_UPDATE_FLAG |
		MCDRV_DIO2_COM_UPDATE_FLAG | MCDRV_DIO2_DIR_UPDATE_FLAG | MCDRV_DIO2_DIT_UPDATE_FLAG);

	sh_mc_asoc_hw_cmd_log(codec, MCDRV_SET_DIGITALIO, (void *)&sh_mc_asoc_hw_dio_info,
		MCDRV_DIO0_COM_UPDATE_FLAG | MCDRV_DIO0_DIR_UPDATE_FLAG | MCDRV_DIO0_DIT_UPDATE_FLAG |
		MCDRV_DIO2_COM_UPDATE_FLAG | MCDRV_DIO2_DIR_UPDATE_FLAG | MCDRV_DIO2_DIT_UPDATE_FLAG);

	if(hw_ret != MCDRV_SUCCESS) {
		dev_err(codec->dev, "McDrv_Ctrl(MCDRV_SET_DIGITALIO) failed (ret=%d)\n", (int)hw_ret);
		McDrv_Ctrl(MCDRV_TERM, NULL, 0);
		return -EIO;
	}


	hw_ret = McDrv_Ctrl(MCDRV_SET_ADC, (void *)&sh_mc_asoc_hw_adc_info, 0);

	sh_mc_asoc_hw_cmd_log(codec, MCDRV_SET_ADC, (void *)&sh_mc_asoc_hw_adc_info, 0);

	if(hw_ret != MCDRV_SUCCESS) {
		dev_err(codec->dev, "McDrv_Ctrl(MCDRV_SET_ADC) failed (ret=%d)\n", (int)hw_ret);
		McDrv_Ctrl(MCDRV_TERM, NULL, 0);
		return -EIO;
	}

	sh_mc_asoc_no_init = 0;

	return 0;
}

static void sh_mc_asoc_hw_reset(struct snd_soc_codec *codec)
{
	const struct sh_mc_asoc_set_path_info *path_info = sh_mc_asoc_set_path_tbl;
	signed long hw_ret;

	dev_dbg(codec->dev, "----- reset -----\n");

	sh_mc_asoc_no_init = 1;
	sh_mc_asoc_cdsp_registered = 0;

	hw_ret = McDrv_Ctrl(MCDRV_TERM, NULL, 0);
	if(hw_ret != MCDRV_SUCCESS) {
		dev_err(codec->dev, "McDrv_Ctrl(MCDRV_TERM) failed (ret=%d)\n", (int)hw_ret);
	}

	while(path_info->src_onoff_reg >= 0)
	{
		sh_mc_asoc_reg_cache_write(codec, path_info->src_onoff_reg_hw, 0);
		path_info++;
	}

	sh_mc_asoc_reg_cache_write(codec, SH_MC_ASOC_REG_PM_SRCONOFF, 0);
}

static int sh_mc_asoc_hw_rw_reg(struct snd_soc_codec *codec, unsigned int rw_type)
{
	int ret = 0;

	signed long hw_ret;
	unsigned int reg_val;

	MCDRV_REG_INFO hw_reg_info;
	memset(&hw_reg_info, 0, sizeof(MCDRV_REG_INFO));

	if((ret = sh_mc_asoc_reg_cache_read(codec, SH_MC_ASOC_REG_HW_REG_TYPE, &reg_val)) < 0) {
		return ret;
	}
	hw_reg_info.bRegType = (UINT8)reg_val;

	if((ret = sh_mc_asoc_reg_cache_read(codec, SH_MC_ASOC_REG_HW_REG_ADDR, &reg_val)) < 0) {
		return ret;
	}
	hw_reg_info.bAddress = (UINT8)reg_val;

	if(rw_type) {
		if((ret = sh_mc_asoc_reg_cache_read(codec, SH_MC_ASOC_REG_HW_REG_DATA, &reg_val)) < 0) {
			return ret;
		}
		hw_reg_info.bData = (UINT8)reg_val;
	}

	hw_ret = McDrv_Ctrl((rw_type)? MCDRV_WRITE_REG: MCDRV_READ_REG, &hw_reg_info, 0);
	if(hw_ret != MCDRV_SUCCESS) {
		dev_err(codec->dev, "McDrv_Ctrl(%s) failed (ret=%d)\n",
			((rw_type)? "MCDRV_WRITE_REG": "MCDRV_READ_REG"), (int)hw_ret);
		return -EIO;
	}

	if(!rw_type) {
		reg_val = hw_reg_info.bData;
		if((ret = sh_mc_asoc_reg_cache_write(codec, SH_MC_ASOC_REG_HW_REG_DATA, reg_val)) < 0) {
			return ret;
		}
	}

	return ret;
}

static int sh_mc_asoc_hw_get_dio(struct snd_soc_codec *codec)
{
	signed long hw_ret;

	MCDRV_DIO_INFO hw_dio_info;
	memset(&hw_dio_info, 0, sizeof(MCDRV_DIO_INFO));

	dev_dbg(codec->dev, "----- get_dio -----\n");

	hw_ret = McDrv_Ctrl(MCDRV_GET_DIGITALIO, (void *)&hw_dio_info, 0);

	sh_mc_asoc_hw_cmd_log(codec, MCDRV_GET_DIGITALIO, (void *)&hw_dio_info, MCDRV_DIO0_DIT_UPDATE_FLAG);

	if(hw_ret != MCDRV_SUCCESS) {
		dev_err(codec->dev, "McDrv_Ctrl(MCDRV_GET_DIGITALIO) failed (ret=%d)\n", (int)hw_ret);
		return -EIO;
	}

	return 0;
}

static int sh_mc_asoc_hw_set_dio(struct snd_soc_codec *codec, int dit0_swap)
{
	signed long hw_ret;

	MCDRV_DIO_INFO hw_dio_info;
	memset(&hw_dio_info, 0, sizeof(MCDRV_DIO_INFO));

	dev_dbg(codec->dev, "----- set_dio (dit0_swap=%d) -----\n", dit0_swap);

	hw_dio_info.asPortInfo[0].sDit.wSrcRate = 0;
	hw_dio_info.asPortInfo[0].sDit.sPcmFormat.bMono = (dit0_swap)? MCDRV_PCM_STEREO: MCDRV_PCM_MONO;
	hw_dio_info.asPortInfo[0].sDit.sPcmFormat.bOrder = MCDRV_PCM_MSB_FIRST;
	hw_dio_info.asPortInfo[0].sDit.sPcmFormat.bLaw = MCDRV_PCM_LINEAR;
	hw_dio_info.asPortInfo[0].sDit.sPcmFormat.bBitSel = MCDRV_PCM_BITSEL_16;
	hw_dio_info.asPortInfo[0].sDit.abSlot[0] = (dit0_swap)? 1: 0;
	hw_dio_info.asPortInfo[0].sDit.abSlot[1] = (dit0_swap)? 0: 1;

	hw_ret = McDrv_Ctrl(MCDRV_SET_DIGITALIO, (void *)&hw_dio_info, MCDRV_DIO0_DIT_UPDATE_FLAG);

	sh_mc_asoc_hw_cmd_log(codec, MCDRV_SET_DIGITALIO, (void *)&hw_dio_info, MCDRV_DIO0_DIT_UPDATE_FLAG);

	if(hw_ret != MCDRV_SUCCESS) {
		dev_err(codec->dev, "McDrv_Ctrl(MCDRV_SET_DIGITALIO) failed (ret=%d)\n", (int)hw_ret);
		return -EIO;
	}

	return 0;
}

static int sh_mc_asoc_hw_get_path(struct snd_soc_codec *codec)
{
	signed long hw_ret;

	MCDRV_PATH_INFO hw_path_info;
	memset(&hw_path_info, 0, sizeof(MCDRV_PATH_INFO));

	dev_dbg(codec->dev, "----- get_path -----\n");

	hw_ret = McDrv_Ctrl(MCDRV_GET_PATH, (void *)&hw_path_info, 0);

	sh_mc_asoc_hw_cmd_log(codec, MCDRV_GET_PATH, (void *)&hw_path_info, 0);

	if(hw_ret != MCDRV_SUCCESS) {
		dev_err(codec->dev, "McDrv_Ctrl(MCDRV_GET_PATH) failed (ret=%d)\n", (int)hw_ret);
		return -EIO;
	}

	return 0;
}

static int sh_mc_asoc_hw_set_path(struct snd_soc_codec *codec, MCDRV_PATH_INFO *hw_path_info)
{
	signed long hw_ret;

	dev_dbg(codec->dev, "----- set_path -----\n");

	hw_ret = McDrv_Ctrl(MCDRV_SET_PATH, hw_path_info, 0);

	sh_mc_asoc_hw_cmd_log(codec, MCDRV_SET_PATH, hw_path_info, 0);

	if(hw_ret != MCDRV_SUCCESS) {
		dev_err(codec->dev, "McDrv_Ctrl(MCDRV_SET_PATH) failed (ret=%d)\n", (int)hw_ret);
		return -EIO;
	}

	return 0;
}

static int sh_mc_asoc_hw_get_volume(struct snd_soc_codec *codec)
{
	signed long hw_ret;

	MCDRV_VOL_INFO hw_vol_info;
	memset(&hw_vol_info, 0, sizeof(MCDRV_VOL_INFO));

	dev_dbg(codec->dev, "----- get_volume -----\n");

	hw_ret = McDrv_Ctrl(MCDRV_GET_VOLUME, (void *)&hw_vol_info, 0);

	sh_mc_asoc_hw_cmd_log(codec, MCDRV_GET_VOLUME, (void *)&hw_vol_info, 0);

	if(hw_ret != MCDRV_SUCCESS) {
		dev_err(codec->dev, "McDrv_Ctrl(MCDRV_GET_VOLUME) failed (ret=%d)\n", (int)hw_ret);
		return -EIO;
	}

	return 0;
}

static int sh_mc_asoc_hw_set_volume(struct snd_soc_codec *codec, MCDRV_VOL_INFO *hw_vol_info)
{
	signed long hw_ret;

	dev_dbg(codec->dev, "----- set_volume -----\n");

	hw_ret = McDrv_Ctrl(MCDRV_SET_VOLUME, hw_vol_info, 0);

	sh_mc_asoc_hw_cmd_log(codec, MCDRV_SET_VOLUME, hw_vol_info, 0);

	if(hw_ret != MCDRV_SUCCESS) {
		dev_err(codec->dev, "McDrv_Ctrl(MCDRV_SET_VOLUME) failed (ret=%d)\n", (int)hw_ret);
		return -EIO;
	}

	return 0;
}

static int sh_mc_asoc_hw_set_dtmf(struct snd_soc_codec *codec, int dtmf_onoff)
{
	signed long hw_ret;

	MCDRV_DTMF_INFO hw_dtmf_info;
	memset(&hw_dtmf_info, 0, sizeof(MCDRV_DTMF_INFO));

	dev_dbg(codec->dev, "----- set_dtmf (dtmf_onoff=%d) -----\n", dtmf_onoff);

	hw_dtmf_info.bSinGenHost = MCDRV_DTMFHOST_REG;
	hw_dtmf_info.bOnOff = (dtmf_onoff)? MCDRV_DTMF_ON: MCDRV_DTMF_OFF;
	hw_dtmf_info.sParam.wSinGen0Freq = 1000;
	hw_dtmf_info.sParam.wSinGen1Freq = 1000;
	hw_dtmf_info.sParam.swSinGen0Vol = 0x0000;
	hw_dtmf_info.sParam.swSinGen1Vol = 0xA000;
	hw_dtmf_info.sParam.bSinGenGate = 25;
	hw_dtmf_info.sParam.bSinGenLoop = MCDRV_SINGENLOOP_ON;

	hw_ret = McDrv_Ctrl(MCDRV_SET_DTMF, (void *)&hw_dtmf_info,
		MCDRV_DTMFHOST_UPDATE_FLAG | MCDRV_DTMFONOFF_UPDATE_FLAG | MCDRV_DTMFFREQ0_UPDATE_FLAG | MCDRV_DTMFFREQ1_UPDATE_FLAG |
		MCDRV_DTMFVOL0_UPDATE_FLAG | MCDRV_DTMFVOL1_UPDATE_FLAG | MCDRV_DTMFGATE_UPDATE_FLAG | MCDRV_DTMFLOOP_UPDATE_FLAG);
	if(hw_ret != MCDRV_SUCCESS) {
		dev_err(codec->dev, "McDrv_Ctrl(MCDRV_SET_DTMF) failed (ret=%d)\n", (int)hw_ret);
		return -EIO;
	}

	return 0;
}

static int sh_mc_asoc_hw_set_cdsp(struct snd_soc_codec *codec, int onoff)
{
	signed long hw_ret;

	dev_dbg(codec->dev, "----- set_cdsp (onoff=%d) -----\n", onoff);

	if(onoff)
	{
		hw_ret = McDrv_Ctrl(
			MCDRV_SET_CDSP,
			(void *)sh_mc_asoc_get_cdsp_firmware(),
			sh_mc_asoc_get_cdsp_firmware_size());
		if(hw_ret != MCDRV_SUCCESS) {
			dev_err(codec->dev, "McDrv_Ctrl(MCDRV_SET_CDSP) failed (ret=%d)\n", (int)hw_ret);
			return -EIO;
		}

		sh_mc_asoc_cdsp_registered = 1;
	}
	else
	{
		sh_mc_asoc_cdsp_registered = 0;

		hw_ret = McDrv_Ctrl(MCDRV_SET_CDSP, NULL, 0);
		if(hw_ret != MCDRV_SUCCESS) {
			dev_err(codec->dev, "McDrv_Ctrl(MCDRV_SET_CDSP(NULL)) failed (ret=%d)\n", (int)hw_ret);
			return -EIO;
		}
	}

	return 0;
}

static int sh_mc_asoc_hw_set_cdsp_param(struct snd_soc_codec *codec, int cdsp_is_active, int cdsp_debug)
{
	int ret = 0;

	signed long hw_ret;

	MCDRV_CDSPPARAM hw_cdsp_param;
	const UINT8 *param;
	UINT8 param_debug[SH_MC_ASOC_CDSP_PARAM_LEN];
	UINT32 param_size;

	dev_dbg(codec->dev, "----- set_cdsp_param (active=%d, debug=%d) -----\n", cdsp_is_active, cdsp_debug);


	if(cdsp_debug)
	{
		unsigned int reg_val;

		if((ret = sh_mc_asoc_reg_cache_read(codec, SH_MC_ASOC_REG_CDSP_PARAM_CMD, &reg_val)) < 0) {
			return ret;
		}
		param_debug[0] = (UINT8)(reg_val & 0xff);

		if((ret = sh_mc_asoc_reg_cache_read(codec, SH_MC_ASOC_REG_CDSP_PARAM_00_01, &reg_val)) < 0) {
			return ret;
		}
		param_debug[1] = (UINT8)(reg_val & 0xff);
		param_debug[2] = (UINT8)((reg_val >> 8) & 0xff);

		if((ret = sh_mc_asoc_reg_cache_read(codec, SH_MC_ASOC_REG_CDSP_PARAM_02_03, &reg_val)) < 0) {
			return ret;
		}
		param_debug[3] = (UINT8)(reg_val & 0xff);
		param_debug[4] = (UINT8)((reg_val >> 8) & 0xff);

		if((ret = sh_mc_asoc_reg_cache_read(codec, SH_MC_ASOC_REG_CDSP_PARAM_04_05, &reg_val)) < 0) {
			return ret;
		}
		param_debug[5] = (UINT8)(reg_val & 0xff);
		param_debug[6] = (UINT8)((reg_val >> 8) & 0xff);

		if((ret = sh_mc_asoc_reg_cache_read(codec, SH_MC_ASOC_REG_CDSP_PARAM_06_07, &reg_val)) < 0) {
			return ret;
		}
		param_debug[7] = (UINT8)(reg_val & 0xff);
		param_debug[8] = (UINT8)((reg_val >> 8) & 0xff);

		if((ret = sh_mc_asoc_reg_cache_read(codec, SH_MC_ASOC_REG_CDSP_PARAM_08_09, &reg_val)) < 0) {
			return ret;
		}
		param_debug[9] = (UINT8)(reg_val & 0xff);
		param_debug[10] = (UINT8)((reg_val >> 8) & 0xff);

		if((ret = sh_mc_asoc_reg_cache_read(codec, SH_MC_ASOC_REG_CDSP_PARAM_10_11, &reg_val)) < 0) {
			return ret;
		}
		param_debug[11] = (UINT8)(reg_val & 0xff);
		param_debug[12] = (UINT8)((reg_val >> 8) & 0xff);

		if((ret = sh_mc_asoc_reg_cache_read(codec, SH_MC_ASOC_REG_CDSP_PARAM_12_13, &reg_val)) < 0) {
			return ret;
		}
		param_debug[13] = (UINT8)(reg_val & 0xff);
		param_debug[14] = (UINT8)((reg_val >> 8) & 0xff);

		if((ret = sh_mc_asoc_reg_cache_read(codec, SH_MC_ASOC_REG_CDSP_PARAM_14_15, &reg_val)) < 0) {
			return ret;
		}
		param_debug[15] = (UINT8)(reg_val & 0xff);
		param_debug[16] = (UINT8)((reg_val >> 8) & 0xff);

		param = param_debug;
		param_size = SH_MC_ASOC_CDSP_PARAM_LEN;
	}
	else
	{
		unsigned int param_id = 0;

		if((ret = sh_mc_asoc_reg_cache_read(codec, SH_MC_ASOC_REG_CDSP_PARAM_ID, &param_id)) < 0) {
			return ret;
		}

		param = sh_mc_asoc_get_cdsp_param((int)param_id);
		if(param == NULL) {
			dev_err(codec->dev, "get_cdsp_param failed (param_id=%d)\n", (int)param_id);
			return -EINVAL;
		}

		param_size = sh_mc_asoc_get_cdsp_param_size((int)param_id);

		dev_dbg(codec->dev, "cdsp_param (id=%d, size=%d)\n", (int)param_id, (int)param_size);
	}


	while(param_size >= SH_MC_ASOC_CDSP_PARAM_LEN)
	{
		memset(&hw_cdsp_param, 0, sizeof(MCDRV_CDSPPARAM));
		hw_cdsp_param.bCommand = param[0];
		memcpy(hw_cdsp_param.abParam, &param[1], sizeof(hw_cdsp_param.abParam));

		if(!sh_mc_asoc_ignore_cdsp_cmd(hw_cdsp_param.bCommand, cdsp_is_active))
		{
			hw_ret = McDrv_Ctrl(MCDRV_SET_CDSP_PARAM, (void *)&hw_cdsp_param, 0);

			sh_mc_asoc_hw_cmd_log(codec, MCDRV_SET_CDSP_PARAM, (void *)&hw_cdsp_param, 0);

			if(hw_ret != MCDRV_SUCCESS) {
				dev_err(codec->dev, "McDrv_Ctrl(MCDRV_SET_CDSP_PARAM) failed (ret=%d)\n", (int)hw_ret);
				return -EIO;
			}
		}
		else
		{
			dev_warn(codec->dev, "ignore cdsp cmd (0x%02x)\n", hw_cdsp_param.bCommand);
		}

		param += SH_MC_ASOC_CDSP_PARAM_LEN;
		param_size -= SH_MC_ASOC_CDSP_PARAM_LEN;
	}

	return ret;
}




static void sh_mc_asoc_set_src_onoff(
	struct snd_soc_codec *codec, const struct sh_mc_asoc_set_path_info *path_info,
	MCDRV_PATH_INFO *hw_path_info, unsigned int src_onoff_update, unsigned int src_onoff_new)
{
	int i, j;
	MCDRV_CHANNEL *channels;

	channels = (MCDRV_CHANNEL *)((void *)hw_path_info + path_info->channel_offset);
	for(i = 0; i < path_info->channel_num; i++)
	{
		for(j = 0; j < SH_MC_ASOC_SRC_MAX; j++) {
			if(src_onoff_update & SH_MC_ASOC_SRC_BIT(j)) {
				if(src_onoff_new & SH_MC_ASOC_SRC_BIT(j)) {
					channels[i].abSrcOnOff[sh_mc_asoc_src_onoff_tbl[j].block] |= sh_mc_asoc_src_onoff_tbl[j].on;
				} else {
					channels[i].abSrcOnOff[sh_mc_asoc_src_onoff_tbl[j].block] |= sh_mc_asoc_src_onoff_tbl[j].off;
				}
			}
		}
	}
}

static int sh_mc_asoc_set_vol_onoff(
	struct snd_soc_codec *codec, const struct sh_mc_asoc_set_path_info *path_info,
	MCDRV_VOL_INFO *hw_vol_on, MCDRV_VOL_INFO *hw_vol_off, int *hw_vol_on_update, int *hw_vol_off_update,
	unsigned int src_onoff_update, unsigned int src_onoff_new)
{
	int ret = 0;

	int i, j;
	unsigned int reg_val;
	SINT16 vol_val;
	SINT16 *vols;

	const struct sh_mc_asoc_set_vol_info *vol_info;

	for(i = 0; i < SH_MC_ASOC_SRC_MAX; i++)
	{
		if(src_onoff_update & SH_MC_ASOC_SRC_BIT(i))
		{
			if(src_onoff_new & SH_MC_ASOC_SRC_BIT(i))
			{
				vol_info = path_info->set_vol_info;

				while(vol_info->src >= 0)
				{
					if(i == vol_info->src)
					{
						reg_val = 0;
						if(vol_info->reg_mute >= 0) {
							if((ret = sh_mc_asoc_reg_cache_read(codec, vol_info->reg_mute, &reg_val)) < 0) {
								return ret;
							}
						}
						if(reg_val) {
							vol_val = 0xA000;
						} else {
							if(vol_info->reg >= 0) {
								if((ret = sh_mc_asoc_reg_cache_read(codec, vol_info->reg, &reg_val)) < 0) {
									return ret;
								}
								vol_val = (SINT16)((reg_val & 0xff) << 8);
							} else {
								vol_val = vol_info->fixed_value;
							}
						}
						vols = (SINT16 *)((void *)hw_vol_on + vol_info->offset);
						for(j = 0; j < vol_info->num; j++) {
							vols[j] = vol_val | MCDRV_VOL_UPDATE;
						}
						*hw_vol_on_update = 1;
					}

					vol_info++;
				}
			}
			else
			{
				vol_info = path_info->set_vol_info;

				while(vol_info->src >= 0)
				{
					if(i == vol_info->src)
					{
						vols = (SINT16 *)((void *)hw_vol_off + vol_info->offset);
						for(j = 0; j < vol_info->num; j++) {
							vols[j] = 0xA000 | MCDRV_VOL_UPDATE;
						}
						*hw_vol_off_update = 1;
					}

					vol_info++;
				}
			}
		}
	}

	return ret;
}

static int sh_mc_asoc_update_path(struct snd_soc_codec *codec, unsigned int pm_src_onoff_new, unsigned int pm_src_onoff_old)
{
	int ret = 0;

	unsigned int pm_src_onoff_update;
	unsigned int val_src_onoff;
	unsigned int val_src_onoff_hw;
	unsigned int val_src_onoff_update;

	int hw_path_update = 0;
	int hw_vol_on_update = 0;
	int hw_vol_off_update = 0;
	int cdsp_update = 0;
	int cdsp_onoff = 0;
	int dit0_swap = 0;
	int dit0_onoff = 0;
	int dtmf_update = 0;
	int dtmf_onoff = 0;

	const struct sh_mc_asoc_set_path_info *path_info = sh_mc_asoc_set_path_tbl;

	static MCDRV_PATH_INFO hw_path_info;
	static MCDRV_VOL_INFO hw_vol_info_off;
	static MCDRV_VOL_INFO hw_vol_info_on;
	memset(&hw_path_info, 0, sizeof(MCDRV_PATH_INFO));
	memset(&hw_vol_info_off, 0, sizeof(MCDRV_VOL_INFO));
	memset(&hw_vol_info_on, 0, sizeof(MCDRV_VOL_INFO));


	pm_src_onoff_update = pm_src_onoff_old | pm_src_onoff_new;

	dev_dbg(codec->dev, "update_path (new=0x%x, old=0x%x, update=0x%x)\n",
		pm_src_onoff_new, pm_src_onoff_old, pm_src_onoff_update);

	if(!pm_src_onoff_update) {
		return ret;
	}


	if((pm_src_onoff_new & SH_MC_ASOC_SRCONOFF_CDSP_MASK) != 0)
	{
		dit0_swap = 1;
	}

	if((pm_src_onoff_new & SH_MC_ASOC_SRCONOFF_CDSP_MASK) == SH_MC_ASOC_SRCONOFF_CDSP_MASK)
	{
		cdsp_onoff = 1;
	}
	else
	{
		pm_src_onoff_new &= ~SH_MC_ASOC_SRCONOFF_CDSP_MASK;
		pm_src_onoff_update |= SH_MC_ASOC_SRCONOFF_CDSP_MASK;
	}

	dev_dbg(codec->dev, "cdsp_onoff=%d, dit0_swap=%d (new=0x%x, old=0x%x, update=0x%x)\n",
		cdsp_onoff, dit0_swap, pm_src_onoff_new, pm_src_onoff_old, pm_src_onoff_update);


	while(path_info->src_onoff_reg >= 0)
	{
		if(pm_src_onoff_update & (1 << path_info->src_onoff_shift))
		{
			if((ret = sh_mc_asoc_reg_cache_read(codec, (unsigned int)path_info->src_onoff_reg_hw, &val_src_onoff_hw)) < 0) {
				return ret;
			}
			if((path_info->src_onoff_shift == SH_MC_ASOC_SRCONOFF_SHIFT(SH_MC_ASOC_REG_SRCONOFF_DIT0)) && val_src_onoff_hw) {
				dit0_onoff = 1;
			}

			if(pm_src_onoff_new & (1 << path_info->src_onoff_shift)) {
				if((ret = sh_mc_asoc_reg_cache_read(codec, (unsigned int)path_info->src_onoff_reg, &val_src_onoff)) < 0) {
					return ret;
				}
				if(val_src_onoff &
					(SH_MC_ASOC_SRC_BIT(SH_MC_ASOC_SRC_CDSP) | SH_MC_ASOC_SRC_BIT(SH_MC_ASOC_SRC_CDSP_DIRECT)))
				{
					if(!cdsp_onoff) {
						val_src_onoff &= ~SH_MC_ASOC_SRC_BIT(SH_MC_ASOC_SRC_CDSP);
						val_src_onoff &= ~SH_MC_ASOC_SRC_BIT(SH_MC_ASOC_SRC_CDSP_DIRECT);
						dev_dbg(codec->dev, "cdsp_src_off (reg=%d, src_onoff=0x%x)\n", path_info->src_onoff_reg, val_src_onoff);
					}
				}
			} else {
				val_src_onoff = 0;
			}

			val_src_onoff_update = val_src_onoff ^ val_src_onoff_hw;

			dev_dbg(codec->dev, "src_onoff[reg=%d] (new=0x%x, old=0x%x, update=0x%x)\n",
				path_info->src_onoff_reg, val_src_onoff, val_src_onoff_hw, val_src_onoff_update);

			if(val_src_onoff_update)
			{
				ret = sh_mc_asoc_set_vol_onoff(
					codec, path_info,
					&hw_vol_info_on, &hw_vol_info_off, &hw_vol_on_update, &hw_vol_off_update,
					val_src_onoff_update, val_src_onoff);
				if(ret < 0) {
					return ret;
				}

				sh_mc_asoc_set_src_onoff(codec, path_info, &hw_path_info, val_src_onoff_update, val_src_onoff);

				val_src_onoff_hw = val_src_onoff;

				if((ret = sh_mc_asoc_reg_cache_write(codec, path_info->src_onoff_reg_hw, val_src_onoff_hw)) < 0) {
					return ret;
				}

				if(SH_MC_ASOC_SRCONOFF_IS_CDSP_SHIFT(path_info->src_onoff_shift))
				{
					cdsp_update = 1;
				}
				if(val_src_onoff_update &
					(SH_MC_ASOC_SRC_BIT(SH_MC_ASOC_SRC_CDSP) | SH_MC_ASOC_SRC_BIT(SH_MC_ASOC_SRC_CDSP_DIRECT)))
				{
					cdsp_update = 1;
				}

				if(val_src_onoff_update & SH_MC_ASOC_SRC_BIT(SH_MC_ASOC_SRC_DTMF))
				{
					dtmf_update = 1;
					if(val_src_onoff & SH_MC_ASOC_SRC_BIT(SH_MC_ASOC_SRC_DTMF)) {
						dtmf_onoff = 1;
					}
				}

				hw_path_update = 1;
			}
		}

		path_info++;
	}


	dev_dbg(codec->dev, "update flg (cdsp=%d, vol_on=%d, vol_off=%d, path=%d, dtmf=%d, dtmf_onoff=%d, dit0_onoff=%d)\n",
		cdsp_update, hw_vol_on_update, hw_vol_off_update, hw_path_update, dtmf_update, dtmf_onoff, dit0_onoff);

	if(cdsp_update && cdsp_onoff && (!sh_mc_asoc_cdsp_registered)) {
		ret = sh_mc_asoc_hw_set_cdsp(codec, cdsp_onoff);
		if(ret >= 0) {
			ret = sh_mc_asoc_hw_set_cdsp_param(codec, 0, 0);
		}
		if(ret < 0) {
			return ret;
		}
	}
	if(hw_vol_off_update) {
		ret = sh_mc_asoc_hw_set_volume(codec, &hw_vol_info_off);
		if(ret < 0) {
			return ret;
		}
	}
	if(dtmf_update && (!dtmf_onoff)) {
		ret = sh_mc_asoc_hw_set_dtmf(codec, dtmf_onoff);
		if(ret < 0) {
			return ret;
		}
	}
	if(hw_path_update) {
		if((pm_src_onoff_update & (1 << SH_MC_ASOC_SRCONOFF_SHIFT(SH_MC_ASOC_REG_SRCONOFF_DIT0))) && !dit0_onoff) {
			ret = sh_mc_asoc_hw_set_dio(codec, dit0_swap);
		}
		if(ret >= 0) {
			ret = sh_mc_asoc_hw_set_path(codec, &hw_path_info);
		}
		if(ret < 0) {
			return ret;
		}
	}
	if(hw_vol_on_update) {
		ret = sh_mc_asoc_hw_set_volume(codec, &hw_vol_info_on);
		if(ret < 0) {
			return ret;
		}
	}
	if(dtmf_update && dtmf_onoff) {
		ret = sh_mc_asoc_hw_set_dtmf(codec, dtmf_onoff);
		if(ret < 0) {
			return ret;
		}
	}
	if(cdsp_update && (!cdsp_onoff)) {
		ret = sh_mc_asoc_hw_set_cdsp(codec, cdsp_onoff);
		if(ret < 0) {
			return ret;
		}
	}

	return ret;
}

static int sh_mc_asoc_update_volume(struct snd_soc_codec *codec, unsigned int reg, unsigned int pm_src_onoff)
{
	int ret = 0;

	unsigned int val_src_onoff;
	unsigned int val_src_onoff_hw;

	int hw_vol_update = 0;

	const struct sh_mc_asoc_set_path_info *path_info = sh_mc_asoc_set_path_tbl;
	const struct sh_mc_asoc_set_vol_info *vol_info;

	MCDRV_VOL_INFO hw_vol_info;
	memset(&hw_vol_info, 0, sizeof(MCDRV_VOL_INFO));


	dev_dbg(codec->dev, "update_volume (pm_src_onoff=0x%x)\n", pm_src_onoff);

	if(!pm_src_onoff) {
		return ret;
	}


	while(path_info->src_onoff_reg >= 0)
	{
		if(pm_src_onoff & (1 << path_info->src_onoff_shift))
		{
			if((ret = sh_mc_asoc_reg_cache_read(codec, (unsigned int)path_info->src_onoff_reg_hw, &val_src_onoff_hw)) < 0) {
				return ret;
			}

			dev_dbg(codec->dev, "src_onoff[reg=%d] (0x%x)\n", path_info->src_onoff_reg_hw, val_src_onoff_hw);

			if(val_src_onoff_hw)
			{
				val_src_onoff = 0;

				vol_info = path_info->set_vol_info;
				while(vol_info->src >= 0) {
					if((vol_info->reg == (int)reg) || (vol_info->reg_mute == (int)reg)) {
						if(val_src_onoff_hw & SH_MC_ASOC_SRC_BIT(vol_info->src)) {
							val_src_onoff = SH_MC_ASOC_SRC_BIT(vol_info->src);
							break;
						}
					}
					vol_info++;
				}

				if(val_src_onoff) {
					ret = sh_mc_asoc_set_vol_onoff(
						codec, path_info,
						&hw_vol_info, &hw_vol_info, &hw_vol_update, &hw_vol_update,
						val_src_onoff, val_src_onoff);
					if(ret < 0) {
						return ret;
					}
					ret = sh_mc_asoc_hw_set_volume(codec, &hw_vol_info);
					break;
				}
			}
		}

		path_info++;
	}

	return ret;
}

static int sh_mc_asoc_update_cdsp_param(struct snd_soc_codec *codec, unsigned int pm_src_onoff)
{
	int ret = 0;

	unsigned int src_onoff_dit2_hw;
	unsigned int src_onoff_mix_hw;
	unsigned int src_onoff_dit2;
	unsigned int src_onoff_mix;


	dev_dbg(codec->dev, "update_cdsp_param (pm_src_onoff=0x%x)\n", pm_src_onoff);

	if((pm_src_onoff & SH_MC_ASOC_SRCONOFF_CDSP_MASK) != SH_MC_ASOC_SRCONOFF_CDSP_MASK) {
		return ret;
	}


	if((ret = sh_mc_asoc_reg_cache_read(codec, SH_MC_ASOC_REG_SRCONOFF_DIT2_HW, &src_onoff_dit2_hw)) < 0) {
		return ret;
	}
	if((ret = sh_mc_asoc_reg_cache_read(codec, SH_MC_ASOC_REG_SRCONOFF_MIX_HW, &src_onoff_mix_hw)) < 0) {
		return ret;
	}
	if((ret = sh_mc_asoc_reg_cache_read(codec, SH_MC_ASOC_REG_SRCONOFF_DIT2, &src_onoff_dit2)) < 0) {
		return ret;
	}
	if((ret = sh_mc_asoc_reg_cache_read(codec, SH_MC_ASOC_REG_SRCONOFF_MIX, &src_onoff_mix)) < 0) {
		return ret;
	}


	if((src_onoff_dit2_hw & SH_MC_ASOC_SRC_BIT(SH_MC_ASOC_SRC_CDSP_DIRECT)) &&
		(src_onoff_mix_hw & SH_MC_ASOC_SRC_BIT(SH_MC_ASOC_SRC_CDSP)))
	{
		unsigned int pm_src_onoff_temp = pm_src_onoff & (~SH_MC_ASOC_SRCONOFF_CDSP_MASK);

		src_onoff_dit2_hw &= ~SH_MC_ASOC_SRC_BIT(SH_MC_ASOC_SRC_CDSP_DIRECT);
		src_onoff_mix_hw &= ~SH_MC_ASOC_SRC_BIT(SH_MC_ASOC_SRC_CDSP);
		src_onoff_dit2_hw |= SH_MC_ASOC_SRC_BIT(SH_MC_ASOC_SRC_DIR0);
		src_onoff_mix_hw |= SH_MC_ASOC_SRC_BIT(SH_MC_ASOC_SRC_DIR2);

		ret = sh_mc_asoc_reg_cache_write(codec, SH_MC_ASOC_REG_SRCONOFF_DIT2, src_onoff_dit2_hw);
		if(ret >= 0) {
			ret = sh_mc_asoc_reg_cache_write(codec, SH_MC_ASOC_REG_SRCONOFF_MIX, src_onoff_mix_hw);
		}
		if(ret >= 0) {
			ret = sh_mc_asoc_update_path(codec, pm_src_onoff_temp, pm_src_onoff);
		}

		src_onoff_dit2_hw &= ~SH_MC_ASOC_SRC_BIT(SH_MC_ASOC_SRC_DIR0);
		src_onoff_mix_hw &= ~SH_MC_ASOC_SRC_BIT(SH_MC_ASOC_SRC_DIR2);
		src_onoff_dit2_hw |= SH_MC_ASOC_SRC_BIT(SH_MC_ASOC_SRC_CDSP_DIRECT);
		src_onoff_mix_hw |= SH_MC_ASOC_SRC_BIT(SH_MC_ASOC_SRC_CDSP);

		if(ret >= 0) {
			ret = sh_mc_asoc_reg_cache_write(codec, SH_MC_ASOC_REG_SRCONOFF_DIT2, src_onoff_dit2_hw);
		}
		if(ret >= 0) {
			ret = sh_mc_asoc_reg_cache_write(codec, SH_MC_ASOC_REG_SRCONOFF_MIX, src_onoff_mix_hw);
		}
		if(ret >= 0) {
			ret = sh_mc_asoc_update_path(codec, pm_src_onoff, pm_src_onoff_temp);
		}

		sh_mc_asoc_reg_cache_write(codec, SH_MC_ASOC_REG_SRCONOFF_DIT2, src_onoff_dit2);
		sh_mc_asoc_reg_cache_write(codec, SH_MC_ASOC_REG_SRCONOFF_MIX, src_onoff_mix);
	}
	else
	{
		ret = sh_mc_asoc_hw_set_cdsp_param(codec, 1, 0);
	}

	return ret;
}




static struct snd_soc_dai_ops sh_mc_asoc_dai_ops = {
/*	.hw_params	= sh_mc_asoc_dai_hw_params,*/
};

struct snd_soc_dai_driver sh_mc_asoc_dai = {
	.name = "mc_asoc-dio0",
	.id = 1,
	.playback = {
		.stream_name  = "MC Playback",
		.channels_min = 1,
		.channels_max = 1,
		.rates   = SNDRV_PCM_RATE_8000,
		.formats = SNDRV_PCM_FMTBIT_S16_LE,
	},
	.capture = {
		.stream_name  = "MC Capture",
		.channels_min = 1,
		.channels_max = 1,
		.rates   = SNDRV_PCM_RATE_8000,
		.formats = SNDRV_PCM_FMTBIT_S16_LE,
	},
	.ops = &sh_mc_asoc_dai_ops,
};




static unsigned int sh_mc_asoc_codec_read_reg(struct snd_soc_codec *codec, unsigned int reg)
{
	int ret = 0;
	unsigned int val;

	if(codec == NULL) {
		pr_err("invalid codec\n");
		return -EINVAL;
	}

	mutex_lock(&sh_mc_asoc_mutex);
	ret = sh_mc_asoc_reg_cache_read(codec, reg, &val);
	mutex_unlock(&sh_mc_asoc_mutex);

	if(ret < 0) {
		return -1;
	}
	return val;
}

static int sh_mc_asoc_codec_write_reg(struct snd_soc_codec *codec, unsigned int reg, unsigned int value)
{
	int ret = 0;
	int reg_w_ret = 0;

	unsigned int pm_src_onoff = 0;
	unsigned int keep_path = 0;

	if(codec == NULL) {
		pr_err("invalid codec\n");
		return -EINVAL;
	}
	if(reg >= __SH_MC_ASOC_REG_READONLY_FIRST && reg <= __SH_MC_ASOC_REG_READONLY_LAST) {
		dev_err(codec->dev, "readonly (reg=0x%x)\n", reg);
		return -EINVAL;
	}

	mutex_lock(&sh_mc_asoc_mutex);

	if(sh_mc_asoc_no_init) {
		ret = sh_mc_asoc_hw_setup(codec);
	}

	if(ret >= 0) {
		ret = sh_mc_asoc_reg_cache_read(codec, SH_MC_ASOC_REG_KEEP_PATH, &keep_path);
	}
	if(ret >= 0) {
		if(keep_path) {
			ret = sh_mc_asoc_reg_cache_read(codec, SH_MC_ASOC_REG_KEEP_PM_SRCONOFF, &pm_src_onoff);
			dev_dbg(codec->dev, "keep_pm_src_onoff=0x%x\n", pm_src_onoff);
		} else {
			ret = sh_mc_asoc_reg_cache_read(codec, SH_MC_ASOC_REG_PM_SRCONOFF, &pm_src_onoff);
		}
	}

	if(ret >= 0) {
		switch(reg) {
			case SH_MC_ASOC_REG_PM_SRCONOFF:
				if(!keep_path) {
					ret = sh_mc_asoc_update_path(codec, value, pm_src_onoff);
				}
				break;
			case SH_MC_ASOC_REG_KEEP_PATH:
				dev_dbg(codec->dev, "keep_path=%d\n", (int)value);
				if(value) {
					ret = sh_mc_asoc_reg_cache_write(codec, SH_MC_ASOC_REG_KEEP_PM_SRCONOFF, pm_src_onoff);
				} else {
					unsigned int keep_pm_src_onoff = pm_src_onoff;
					ret = sh_mc_asoc_reg_cache_read(codec, SH_MC_ASOC_REG_PM_SRCONOFF, &pm_src_onoff);
					if(ret >= 0) {
						ret = sh_mc_asoc_update_path(codec, pm_src_onoff, keep_pm_src_onoff);
					}
				}
				break;
			case SH_MC_ASOC_REG_HW_CMD:
				switch(value) {
					case 0: /* hw reg read */
					case 1: /* hw reg write */
						ret = sh_mc_asoc_hw_rw_reg(codec, value);
						break;
					case 10:
						ret = sh_mc_asoc_hw_get_dio(codec);
						break;
					case 11:
						ret = sh_mc_asoc_hw_get_path(codec);
						break;
					case 12:
						ret = sh_mc_asoc_hw_get_volume(codec);
						break;
					case 50: /* hw cdsp param */
						if(!sh_mc_asoc_cdsp_registered) {
							ret = sh_mc_asoc_hw_set_cdsp(codec, 1);
						}
						if(ret >= 0) {
							ret = sh_mc_asoc_hw_set_cdsp_param(
								codec, ((pm_src_onoff & SH_MC_ASOC_SRCONOFF_CDSP_MASK) == SH_MC_ASOC_SRCONOFF_CDSP_MASK), 1);
						}
						break;
					case 99: /* hw reset */
						sh_mc_asoc_hw_reset(codec);
						break;
				}
				break;
			default:
				break;
		}
	}

	reg_w_ret = sh_mc_asoc_reg_cache_write(codec, reg, value);
	if(ret >= 0) {
		ret = reg_w_ret;
	}

	if(ret >= 0) {
		if(reg >= __SH_MC_ASOC_REG_SRCONOFF_FIRST && reg <= __SH_MC_ASOC_REG_SRCONOFF_LAST) {
			if(!keep_path) {
				ret = sh_mc_asoc_update_path(codec, pm_src_onoff, pm_src_onoff);
			}
		} else if(reg >= __SH_MC_ASOC_REG_VOL_FIRST && reg <= __SH_MC_ASOC_REG_VOL_LAST) {
			ret = sh_mc_asoc_update_volume(codec, reg, pm_src_onoff);
		} else if(reg == SH_MC_ASOC_REG_CDSP_PARAM_ID) {
			ret = sh_mc_asoc_update_cdsp_param(codec, pm_src_onoff);
		}
	}

	if(ret < 0) {
		if(!sh_mc_asoc_no_init) {
			sh_mc_asoc_hw_reset(codec);
		}
	}

	mutex_unlock(&sh_mc_asoc_mutex);

	return ret;
}

static int sh_mc_asoc_codec_probe(struct snd_soc_codec *codec)
{
	return 0;
}

static int sh_mc_asoc_codec_remove(struct snd_soc_codec *codec)
{
	mutex_lock(&sh_mc_asoc_mutex);
	if(!sh_mc_asoc_no_init) {
		sh_mc_asoc_hw_reset(codec);
	}
	mutex_unlock(&sh_mc_asoc_mutex);
	return 0;
}




struct snd_soc_codec_driver sh_mc_asoc_codec_dev = {
	.probe             = sh_mc_asoc_codec_probe,
	.remove            = sh_mc_asoc_codec_remove,
	.read              = sh_mc_asoc_codec_read_reg,
	.write             = sh_mc_asoc_codec_write_reg,
	.reg_cache_size    = SH_MC_ASOC_REG_MAX,
	.reg_word_size     = sizeof(u16),
	.reg_cache_default = sh_mc_asoc_reg_default,
	.reg_cache_step    = 1,
	.controls          = sh_mc_asoc_snd_controls,
	.num_controls      = ARRAY_SIZE(sh_mc_asoc_snd_controls),
	.dapm_widgets      = sh_mc_asoc_dapm_widgets,
	.num_dapm_widgets  = ARRAY_SIZE(sh_mc_asoc_dapm_widgets),
	.dapm_routes       = sh_mc_asoc_intercon,
	.num_dapm_routes   = ARRAY_SIZE(sh_mc_asoc_intercon),
};




static int sh_mc_asoc_i2c_probe(struct i2c_client *client, const struct i2c_device_id *id)
{
	int err;

	if ((err = snd_soc_register_codec(&client->dev, &sh_mc_asoc_codec_dev, &sh_mc_asoc_dai, 1)) < 0) {
		dev_err(&client->dev, "snd_soc_register_codec failed (err=%d)\n", err);
		return err;
	}

	sh_mc_asoc_i2c_a = client;

	return 0;
}

static int sh_mc_asoc_i2c_remove(struct i2c_client *client)
{
	snd_soc_unregister_codec(&client->dev);
	return 0;
}

static int sh_mc_asoc_i2c_probe_d(struct i2c_client *client, const struct i2c_device_id *id)
{
	sh_mc_asoc_i2c_d = client;
	return 0;
}

static int sh_mc_asoc_i2c_remove_d(struct i2c_client *client)
{
	return 0;
}

static const struct i2c_device_id sh_mc_asoc_i2c_id_a[] = {
	{"mc_asoc_a", 0},
	{},
};
MODULE_DEVICE_TABLE(i2c, sh_mc_asoc_i2c_id_a);

static const struct i2c_device_id sh_mc_asoc_i2c_id_d[] = {
	{"mc_asoc_d", 0},
	{},
};
MODULE_DEVICE_TABLE(i2c, sh_mc_asoc_i2c_id_d);

static struct i2c_driver sh_mc_asoc_i2c_driver_a = {
	.driver = {
		.name  = "mc_asoc_a",
		.owner = THIS_MODULE,
	},
	.probe     = sh_mc_asoc_i2c_probe,
	.remove    = sh_mc_asoc_i2c_remove,
	.id_table  = sh_mc_asoc_i2c_id_a,
};

static struct i2c_driver sh_mc_asoc_i2c_driver_d = {
	.driver = {
		.name  = "mc_asoc_d",
		.owner = THIS_MODULE,
	},
	.probe     = sh_mc_asoc_i2c_probe_d,
	.remove    = sh_mc_asoc_i2c_remove_d,
	.id_table  = sh_mc_asoc_i2c_id_d,
};

static int __init sh_mc_asoc_module_init(void)
{
	int err = 0;
	err = i2c_add_driver(&sh_mc_asoc_i2c_driver_a);
	if (err >= 0) {
		err = i2c_add_driver(&sh_mc_asoc_i2c_driver_d);
	}
	sh_mc_asoc_no_init = 1;
	sh_mc_asoc_cdsp_registered = 0;
	mutex_init(&sh_mc_asoc_mutex);
	return err;
}
module_init(sh_mc_asoc_module_init);

static void __exit sh_mc_asoc_module_exit(void)
{
	mutex_destroy(&sh_mc_asoc_mutex);
	i2c_del_driver(&sh_mc_asoc_i2c_driver_a);
	i2c_del_driver(&sh_mc_asoc_i2c_driver_d);
}
module_exit(sh_mc_asoc_module_exit);

MODULE_DESCRIPTION("SH MC ALSA SoC codec driver");
MODULE_LICENSE("GPL");
