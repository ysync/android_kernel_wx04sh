/* drivers/video/msm/mipi_sharp_cmd_kite_hd_pt.c  (Display Driver)
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

#include "msm_fb.h"
#include "mipi_dsi.h"
#include "mipi_sharp_kite.h"
#include <sharp/shdisp_kerl.h>

static struct msm_panel_info pinfo;

static struct mipi_dsi_phy_ctrl dsi_cmd_mode_phy_db = {
#ifdef CONFIG_MACH_DECKARD_AS70
	/* regulator */
	{0x03, 0x01, 0x01, 0x00, 0x20},
	/* timing    */
	{0x6B, 0x23, 0x1A, 0x00, 0x5E, 0x33, 0x1E, 0x25, 0x1D, 0x03, 0x04, 0x00},
	/* phy ctrl  */
	{0x7F, 0x00, 0x00, 0x10},
	/* strength  */
	{0xEE, 0x02, 0x86, 0x00},
	/* pll control */
	{0x00, 0x5C, 0x30, 0xC5, 0x00, 0x50, 0x48, 0x63,
	 0x31, 0x0F, 0x03, /*  --> Four lane configuration */
	 0x05, 0x14, 0x03, 0x00, 0x02, 0x54, 0x06, 0x10, 0x04, 0x00},
#else
	/* regulator */
	{0x03, 0x01, 0x01, 0x00, 0x20},
	/* timing    */
	{0x65, 0x21, 0x19, 0x00, 0x5D, 0x2E, 0x1C, 0x23, 0x1C, 0x03, 0x04, 0x00},
	/* phy ctrl  */
	{0x7F, 0x00, 0x00, 0x10},
	/* strength  */
	{0xEE, 0x02, 0x86, 0x00},
	/* pll control */
	{0x00, 0x62, 0x31, 0xD7, 0x00, 0x50, 0x48, 0x63,
	 0x31, 0x0F, 0x03, /*  --> Four lane configuration */
	 0x05, 0x14, 0x03, 0x00, 0x02, 0x54, 0x06, 0x10, 0x04, 0x00},
#endif /* CONFIG_MACH_DECKARD_AS70 */
};

static int __init mipi_cmd_sharp_kite_hd_pt_init(void)
{
	int ret;

#if defined(CONFIG_SHDISP_PANEL_SWITCH)
    if (shdisp_api_get_panel_info() == 1) {
        return 0;
    }
#endif

#ifdef CONFIG_FB_MSM_MIPI_PANEL_DETECT
	if (msm_fb_detect_client("mipi_cmd_sharp_kite_hd"))
		return 0;
#endif

#if defined(CONFIG_SHDISP_PANEL_SWITCH)
	printk(KERN_INFO "[SHDISP]%s\n", __func__);
#endif

	pinfo.xres = 720;
	pinfo.yres = 1280;
	pinfo.type = MIPI_CMD_PANEL;
	pinfo.pdest = DISPLAY_1;
	pinfo.wait_cycle = 0;
	pinfo.bpp = 24;
	pinfo.bl_max = 255;
	pinfo.bl_min = 1;
	pinfo.fb_num = 2;
#ifdef CONFIG_MACH_DECKARD_AS70
    pinfo.clk_rate = 418500000;
    pinfo.clk_min  = 418500000;
    pinfo.clk_max  = 418500000;
#else
    pinfo.clk_rate = 399380000;
    pinfo.clk_min  = 399380000;
    pinfo.clk_max  = 399380000;
#endif /* CONFIG_MACH_DECKARD_AS70 */
	pinfo.is_3d_panel = 0;
	pinfo.lcd.vsync_enable = TRUE;
	pinfo.lcd.refx100 = 6000;
	pinfo.lcd.hw_vsync_mode = TRUE;
#ifdef CONFIG_MACH_DECKARD_AS70
	pinfo.lcdc.h_back_porch = 20;
	pinfo.lcdc.h_front_porch = 138;
	pinfo.lcdc.h_pulse_width = 12;

	if (shdisp_api_get_clock_info() == 0) {
		pinfo.lcdc.v_back_porch = 2;
		pinfo.lcdc.v_front_porch = 18;
	} else {
		pinfo.lcdc.v_back_porch = 18;	/* v timing adjusted for internal CR clock */
		pinfo.lcdc.v_front_porch = 8;	/* v timing adjusted for internal CR clock */
	}
	pinfo.lcdc.v_pulse_width = 2;
#else
	pinfo.lcdc.h_back_porch = 20;
	pinfo.lcdc.h_front_porch = 96;
	pinfo.lcdc.h_pulse_width = 12;
	if (shdisp_api_get_clock_info() == 0) {
		pinfo.lcdc.v_back_porch = 2;
		pinfo.lcdc.v_front_porch = 18;
	} else {
		pinfo.lcdc.v_back_porch = 18;	/* v timing adjusted for internal CR clock */
		pinfo.lcdc.v_front_porch = 8;	/* v timing adjusted for internal CR clock */
	}
	pinfo.lcdc.v_pulse_width = 2;
#endif /* CONFIG_MACH_DECKARD_AS70 */
	pinfo.lcdc.border_clr = 0;
	pinfo.lcdc.underflow_clr = 0xff;
	pinfo.lcdc.hsync_skew = 0;
	pinfo.mipi.mode = DSI_CMD_MODE;
	pinfo.mipi.dst_format = DSI_CMD_DST_FORMAT_RGB888;
	pinfo.mipi.data_lane0 = TRUE;
	pinfo.mipi.data_lane1 = TRUE;
	pinfo.mipi.data_lane2 = TRUE;
	pinfo.mipi.data_lane3 = TRUE;
	pinfo.mipi.esc_byte_ratio = 4;
	pinfo.mipi.rgb_swap = DSI_RGB_SWAP_RGB;
	pinfo.mipi.tx_eot_append = TRUE;
	pinfo.mipi.t_clk_post = 0x00;
#ifdef CONFIG_MACH_DECKARD_AS70
	pinfo.mipi.t_clk_pre = 0x17;
#else
	pinfo.mipi.t_clk_pre = 0x16;
#endif /* CONFIG_MACH_DECKARD_AS70 */
	pinfo.mipi.vc = 0;
	pinfo.mipi.dsi_phy_db = &dsi_cmd_mode_phy_db;
	pinfo.mipi.pulse_mode_hsa_he = FALSE;
	pinfo.mipi.hfp_power_stop = TRUE;
	pinfo.mipi.hbp_power_stop = FALSE;
	pinfo.mipi.hsa_power_stop = FALSE;
	pinfo.mipi.eof_bllp_power_stop = TRUE;
	pinfo.mipi.bllp_power_stop = TRUE;
	pinfo.mipi.traffic_mode = DSI_NON_BURST_SYNCH_EVENT;
	pinfo.mipi.frame_rate = 60;
	pinfo.mipi.interleave_max = 1;
	pinfo.mipi.insert_dcs_cmd = TRUE;
	pinfo.mipi.wr_mem_continue = 0x3c;
	pinfo.mipi.wr_mem_start = 0x2c;
	pinfo.mipi.te_sel = 1;
	pinfo.mipi.stream = 0;
	pinfo.mipi.mdp_trigger = DSI_CMD_TRIGGER_NONE;
	pinfo.mipi.dma_trigger = DSI_CMD_TRIGGER_SW;
	pinfo.mipi.force_clk_lane_hs = FALSE;

	ret = mipi_sharp_kite_device_register(&pinfo, MIPI_DSI_PRIM,
						MIPI_DSI_PANEL_720P_PT);
	if (ret)
		pr_err("%s: failed to register device!\n", __func__);

	return ret;
}

module_init(mipi_cmd_sharp_kite_hd_pt_init);
