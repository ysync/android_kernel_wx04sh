/* sound/soc/codecs/ymu828b/sh_mc_asoc.h
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

#ifndef __SH_MC_ASOC_H__
#define __SH_MC_ASOC_H__


/*#define SH_MC_ASOC_TEST_BOARD*/
/*#define SH_MC_ASOC_CDSP_DISABLE*/


#include <linux/i2c.h>
#include "mcdriver.h"


struct i2c_client* sh_mc_asoc_get_i2c_client(int slave);


#define SH_MC_ASOC_CDSP_PARAM_LEN 17

const UINT8* sh_mc_asoc_get_cdsp_firmware(void);
UINT32 sh_mc_asoc_get_cdsp_firmware_size(void);
const UINT8* sh_mc_asoc_get_cdsp_param(int param_id);
UINT32 sh_mc_asoc_get_cdsp_param_size(int param_id);
int sh_mc_asoc_ignore_cdsp_cmd(UINT8 cdsp_cmd, int is_cdsp_active);


#endif /* __SH_MC_ASOC_H__ */
