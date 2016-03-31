/* sound/soc/codecs/ymu828b/sh_mc_asoc_cdsp.c
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

#include "mcdriver.h"
#include "sh_mc_asoc.h"

#ifndef SH_MC_ASOC_CDSP_DISABLE
#include "ladonhf31.h"
#endif

#include "sh_mc_asoc_cdsp_param.h"


static const UINT8 sh_mc_asoc_hw_cdsp_param_default[] = {
	0x22,
	10, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0,
	0x44,
	0, 0, 0, 0, 0, 64, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0,
};


/*--------------------------------------------------------------------*/
/* CDSP setting parameters table */
/*--------------------------------------------------------------------*/
struct sh_mc_asoc_cdsp_param_info {
	int id;
	const UINT8 *data;
	UINT32 size;
};

static const struct sh_mc_asoc_cdsp_param_info sh_mc_asoc_cdsp_param_tbl[] = {
	{0, sh_mc_asoc_hw_cdsp_param_default, sizeof(sh_mc_asoc_hw_cdsp_param_default)},
	{1, HS134, sizeof(HS134)},
	{2, Hs302, sizeof(Hs302)},
	{3, HF267, sizeof(HF267)},
	{4, BT501, sizeof(BT501)},
	{5, Hs405, sizeof(Hs405)},
	{-1, NULL, 0} /* table end */
};
/*--------------------------------------------------------------------*/




static const UINT8 sh_mc_asoc_ignore_cdsp_cmd_tbl_hf31[] = {0x22,0x3f,0x40,0x41,0x42,0x43,0x44};




const UINT8* sh_mc_asoc_get_cdsp_firmware(void)
{
#ifndef SH_MC_ASOC_CDSP_DISABLE
	return gabLadonHf31;
#else
	return NULL;
#endif
}

UINT32 sh_mc_asoc_get_cdsp_firmware_size(void)
{
#ifndef SH_MC_ASOC_CDSP_DISABLE
	return sizeof(gabLadonHf31);
#else
	return 0;
#endif
}

const UINT8* sh_mc_asoc_get_cdsp_param(int param_id)
{
	const struct sh_mc_asoc_cdsp_param_info *param_info = sh_mc_asoc_cdsp_param_tbl;
	while(param_info->id >= 0) {
		if(param_info->id == param_id) {
			return param_info->data;
		}
		param_info++;
	}
	return NULL;
}

UINT32 sh_mc_asoc_get_cdsp_param_size(int param_id)
{
	const struct sh_mc_asoc_cdsp_param_info *param_info = sh_mc_asoc_cdsp_param_tbl;
	while(param_info->id >= 0) {
		if(param_info->id == param_id) {
			return param_info->size;
		}
		param_info++;
	}
	return 0;
}

int sh_mc_asoc_ignore_cdsp_cmd(UINT8 cdsp_cmd, int is_cdsp_active)
{
	if(is_cdsp_active) {
		int i;
		for(i = 0; i < sizeof(sh_mc_asoc_ignore_cdsp_cmd_tbl_hf31); i++) {
			if(cdsp_cmd == sh_mc_asoc_ignore_cdsp_cmd_tbl_hf31[i]) {
				return 1;
			}
		}
	}
	return 0;
}
