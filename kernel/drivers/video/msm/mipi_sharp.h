/* drivers/video/msm/mipi_sharp.h  (Display Driver)
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

#ifndef MIPI_SHARP_H
#define MIPI_SHARP_H

#include "msm_fb_panel.h"

enum {
    SHDISP_STAT_INC_ERR_DET,
    SHDISP_STAT_INC_RECOVERY
};

int mipi_sharp_diag_write_reg(unsigned char addr, 
					unsigned char *write_data, unsigned char size);
int mipi_sharp_diag_read_reg(unsigned char addr, 
					unsigned char *read_data, unsigned char size);
int mipi_sharp_normal_read_reg(unsigned char addr, 
                    unsigned char *read_data, unsigned char size);

void lock_recovery(void);
void unlock_recovery(void);
void msm_lcd_recovery(void);
void msm_lcd_recovery_subscribe(void);
void msm_lcd_recovery_unsubscribe(void);
void mipi_dsi_det_recovery(void);
void mipi_sharp_delay_us(unsigned long usec);
int mipi_sharp_is_recovery(void);
int mipi_sharp_get_suspended_recovery_info(void);
void mipi_dsi_det_recovery_flag_clr(void);
void mipi_sharp_stat_inc(int mode);

int mipi_sharp_api_cabc_init(void);
int mipi_sharp_api_cabc_indoor_on(void);
int mipi_sharp_api_cabc_outdoor_on(int lut_level);
int mipi_sharp_api_cabc_off(int wait_on,int pwm_disable);
int mipi_sharp_api_cabc_outdoor_move(int lut_level);
int mipi_sharp_api_set_lpf(unsigned char *param);

#endif  /* MIPI_SHARP_H */
