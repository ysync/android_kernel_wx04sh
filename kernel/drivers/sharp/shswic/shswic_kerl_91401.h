/* kernel/drivers/sharp/shswic_kerl_91401.h  (SwitchingIC Driver)
 *
 * Copyright (C) 2011 SHARP CORPORATION
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

/* ____________________________________________________________________________________________________________________________ */

#ifndef _SHSWIC_KERL_91401_
#define _SHSWIC_KERL_91401_

#include "shswic_def.h"

void shswic_detect_isr_sig_handler_91401(void);
uint8_t shswic_detect_get_91401(uint8_t id_status, uint8_t status, uint8_t int_status);
void shswic_detect_init_91401(void);
void shswic_detect_retry_sig_handler_91401(void);
void shswic_detect_mhl_proc_91401(void);
shswic_result_t shswic_write_vbsw_reg_91401(uint8_t vbsw_ctl);
shswic_result_t shswic_read_vbsw_reg_91401(uint8_t* vbsw_ctl);
void shswic_pre_detect_cb_call_91401(uint8_t detect);
void shswic_pre_do_exit_91401(void);
#ifdef CONFIG_CRADLE_SWIC
int shswic_cradle_state_91401(void);
#endif /* CONFIG_CRADLE_SWIC */
bool shswic_is_push_headset_button_91401(uint8_t int_status);

#endif /* _SHSWIC_KERL_91401_ */
