/* drivers/video/msm/mipi_sharp_ryoma.h  (Display Driver)
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

#ifndef MIPI_SHARP_RYOMA_H
#define MIPI_SHARP_RYOMA_H

int mipi_sharp_ryoma_device_register(struct msm_panel_info *pinfo,
					u32 channel, u32 panel);
int mipi_sharp_ryoma_diag_write_reg(unsigned char addr, unsigned char *write_data, unsigned char size);
int mipi_sharp_ryoma_diag_read_reg(unsigned char addr, unsigned char *read_data, unsigned char size);
int mipi_sharp_ryoma_init_phy_gamma(struct shdisp_lcddr_phy_gamma_reg *phy_gamma);

#endif  /* MIPI_SHARP_RYOMA_H */
