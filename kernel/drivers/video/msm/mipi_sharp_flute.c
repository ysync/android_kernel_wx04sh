/* drivers/video/msm/mipi_sharp_flute.c  (Display Driver)
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
#include "mdp.h"
#include "mdp4.h"
#include "mipi_sharp.h"
#include "mipi_sharp_flute.h"
#include <sharp/shdisp_kerl.h>

#include <linux/gpio.h>
#include <linux/mfd/pm8xxx/pm8921.h>
#include <mach/irqs.h>

/* ------------------------------------------------------------------------- */
/* DEBUG MACROS                                                              */
/* ------------------------------------------------------------------------- */

#define MIPI_SHARP_RW_MAX_SIZE          48
#define MIPI_SHARP_FLUTE_GPARA_ADDR     0xB0

static struct dsi_buf sharp_tx_buf;
static struct dsi_buf sharp_rx_buf;
static int ch_used[3];
extern boolean msm_fb_shutdown_in_progress;
extern boolean lcdon_cmd_send;

static char mipi_sharp_cmd_passwd1_l2_unmask[3] = {0xF0, 0x5A,0x5A};
static char mipi_sharp_cmd_passwd1_l2_mask[3] = {0xF0, 0xA5,0xA5};
static char mipi_sharp_cmd_passwd1_l3_unmask[3] = {0xFC, 0x5A,0x5A};
static char mipi_sharp_cmd_passwd1_l3_mask[3] = {0xFC, 0xA5,0xA5};
static char mipi_sharp_cmd_crclk_ctl_low[3] = {0xB2, 0x00,0x1B};
static char mipi_sharp_cmd_crclk_ctl_high[3] = {0xB2, 0x01,0x1B};

static struct dsi_cmd_desc mipi_sharp_cmds_passwd1_l2_unmask[] = {
    {DTYPE_GEN_LWRITE, 1, 0, 0, 0, sizeof(mipi_sharp_cmd_passwd1_l2_unmask), mipi_sharp_cmd_passwd1_l2_unmask}
};
static struct dsi_cmd_desc mipi_sharp_cmds_passwd1_l2_mask[] = {
    {DTYPE_GEN_LWRITE, 1, 0, 0, 0, sizeof(mipi_sharp_cmd_passwd1_l2_mask), mipi_sharp_cmd_passwd1_l2_mask}
};
static struct dsi_cmd_desc mipi_sharp_cmds_passwd1_l3_unmask[] = {
    {DTYPE_GEN_LWRITE, 1, 0, 0, 0, sizeof(mipi_sharp_cmd_passwd1_l3_unmask), mipi_sharp_cmd_passwd1_l3_unmask}
};
static struct dsi_cmd_desc mipi_sharp_cmds_passwd1_l3_mask[] = {
    {DTYPE_GEN_LWRITE, 1, 0, 0, 0, sizeof(mipi_sharp_cmd_passwd1_l3_mask), mipi_sharp_cmd_passwd1_l3_mask}
};
static struct dsi_cmd_desc mipi_sharp_cmds_crclk_ctl_low[] = {
    {DTYPE_GEN_LWRITE, 1, 0, 0, 0, sizeof(mipi_sharp_cmd_crclk_ctl_low), mipi_sharp_cmd_crclk_ctl_low}
};
static struct dsi_cmd_desc mipi_sharp_cmds_crclk_ctl_high[] = {
    {DTYPE_GEN_LWRITE, 1, 0, 0, 0, sizeof(mipi_sharp_cmd_crclk_ctl_high), mipi_sharp_cmd_crclk_ctl_high}
};

static char mipi_sharp_cmd_sleepout[2] = {0x11, 0x00};
static char mipi_sharp_cmd_resol[2] = {0xB3, 0x21};
static char mipi_sharp_cmd_panelctl1[9] = {0xB4, 0x00,0x06,0x07,0x09,0x0A,0x08,0x00,0x06};
static char mipi_sharp_cmd_dispctl_for_video[8] = {0xF2, 0x19,0x00,0x0A,0x16,0x1E,0x1C,0x08};
static char mipi_sharp_cmd_manpwr[8] = {0xF3, 0x01,0x00,0x00,0x08,0x08,0x02,0x10};
static char mipi_sharp_cmd_pwrctl1[11] = {0xF4, 0x00,0x00,0x00,0x00,0x55,0x44,0x05,0xDD,0x4B,0x4B};
static char mipi_sharp_cmd_pwrctl2[7] = {0xF5, 0x69,0x69,0x54,0x48,0x36,0x08};
static char mipi_sharp_cmd_srcctl[7] = {0xF6, 0x03,0x07,0x8A,0x00,0x01,0x16};

static char mipi_sharp_cmd_panelctl[38] = {0xF7, 
    0x00,
    0x14,0x13,0x12,0x11,0x01,0x10,0x0C,0x0F,0x0B,0x0E,0x0A,0x0D,0x09,0x01,0x08,0x07,0x06,0x05,
    0x14,0x13,0x12,0x11,0x01,0x10,0x0C,0x0F,0x0B,0x0E,0x0A,0x0D,0x09,0x01,0x08,0x07,0x06,0x05};
static char mipi_sharp_cmd_pgammactl[49] = {0xFA,
    0x0A,0x00,0x00,0x02,0x00,0x00,0x00,0x0A,0x08,0x0D,0x14,0x14,
    0x17,0x1E,0x12,0x0A,0x0A,0x00,0x00,0x02,0x00,0x00,0x00,0x0A,
    0x08,0x0D,0x14,0x14,0x17,0x1E,0x12,0x0A,0x0A,0x00,0x00,0x02,
    0x00,0x00,0x00,0x0A,0x08,0x0D,0x14,0x14,0x17,0x1E,0x12,0x0A};
static char mipi_sharp_cmd_ngammactl[49] = {0xFB,
    0x00,0x06,0x06,0x07,0x05,0x04,0x02,0x0C,0x0A,0x0E,0x12,0x11,
    0x12,0x18,0x0A,0x01,0x00,0x06,0x06,0x07,0x05,0x04,0x02,0x0C,
    0x0A,0x0E,0x12,0x11,0x12,0x18,0x0A,0x01,0x00,0x06,0x06,0x07,
    0x05,0x04,0x02,0x0C,0x0E,0x0A,0x12,0x11,0x12,0x0A,0x18,0x01};
static char mipi_sharp_cmd_logictest[4] = {0xFE, 0x00,0x0B,0x00};

static struct dsi_cmd_desc mipi_sharp_cmds_sleepout[] = {
    {DTYPE_DCS_WRITE, 1, 0, 0, 120, sizeof(mipi_sharp_cmd_sleepout), mipi_sharp_cmd_sleepout}
};
static struct dsi_cmd_desc mipi_sharp_cmds_resol[] = {
    {DTYPE_GEN_WRITE2, 1, 0, 0, 0, sizeof(mipi_sharp_cmd_resol), mipi_sharp_cmd_resol}
};
static struct dsi_cmd_desc mipi_sharp_cmds_panelctl1_for_video[] = {
    {DTYPE_GEN_LWRITE, 1, 0, 0, 0, sizeof(mipi_sharp_cmd_panelctl1), mipi_sharp_cmd_panelctl1}
};
static struct dsi_cmd_desc mipi_sharp_cmds_dispctl_for_video[] = {
    {DTYPE_GEN_LWRITE, 1, 0, 0, 0, sizeof(mipi_sharp_cmd_dispctl_for_video), mipi_sharp_cmd_dispctl_for_video}
};
static struct dsi_cmd_desc mipi_sharp_cmds_power[] = {
    {DTYPE_GEN_LWRITE, 1, 0, 0, 0, sizeof(mipi_sharp_cmd_manpwr), mipi_sharp_cmd_manpwr},
    {DTYPE_GEN_LWRITE, 1, 0, 0, 0, sizeof(mipi_sharp_cmd_pwrctl1), mipi_sharp_cmd_pwrctl1}
};
static struct dsi_cmd_desc mipi_sharp_cmds_srcctl[] = {
    {DTYPE_GEN_LWRITE, 1, 0, 0, 0, sizeof(mipi_sharp_cmd_srcctl), mipi_sharp_cmd_srcctl}
};
static struct dsi_cmd_desc mipi_sharp_cmds_panelctl[] = {
    {DTYPE_GEN_LWRITE, 1, 0, 0, 0, sizeof(mipi_sharp_cmd_panelctl), mipi_sharp_cmd_panelctl}
};
static struct dsi_cmd_desc mipi_sharp_cmds_pgammactl[] = {
    {DTYPE_GEN_LWRITE, 1, 0, 0, 0, sizeof(mipi_sharp_cmd_pgammactl), mipi_sharp_cmd_pgammactl}
};
static struct dsi_cmd_desc mipi_sharp_cmds_ngammactl[] = {
    {DTYPE_GEN_LWRITE, 1, 0, 0, 0, sizeof(mipi_sharp_cmd_ngammactl), mipi_sharp_cmd_ngammactl}
};
static struct dsi_cmd_desc mipi_sharp_cmds_logictest[] = {
    {DTYPE_GEN_LWRITE, 1, 0, 0, 0, sizeof(mipi_sharp_cmd_logictest), mipi_sharp_cmd_logictest}
};

static char mipi_sharp_cmd_batctl[5] = {0xBC, 0x01,0x29,0x00,0x43};
static char mipi_sharp_cmd_disp_on[2] = {0x29, 0x00};

static struct dsi_cmd_desc mipi_sharp_cmds_disp_on_for_video[] = {
    {DTYPE_GEN_LWRITE, 1, 0, 0, 0, sizeof(mipi_sharp_cmd_passwd1_l2_unmask), mipi_sharp_cmd_passwd1_l2_unmask},
    {DTYPE_GEN_LWRITE, 1, 0, 0, 0, sizeof(mipi_sharp_cmd_batctl), mipi_sharp_cmd_batctl},
    {DTYPE_GEN_LWRITE, 1, 0, 0, 0, sizeof(mipi_sharp_cmd_crclk_ctl_low), mipi_sharp_cmd_crclk_ctl_low},
    {DTYPE_GEN_LWRITE, 1, 0, 0, 0, sizeof(mipi_sharp_cmd_passwd1_l2_mask), mipi_sharp_cmd_passwd1_l2_mask},
    {DTYPE_DCS_WRITE, 1, 0, 0, 0, sizeof(mipi_sharp_cmd_disp_on), mipi_sharp_cmd_disp_on}
};

static char mipi_sharp_cmd_disp_off[2] = {0x28, 0x00};

static struct dsi_cmd_desc mipi_sharp_cmds_disp_off_for_video[] = {
    {DTYPE_DCS_WRITE, 1, 0, 0, 0, sizeof(mipi_sharp_cmd_disp_off), mipi_sharp_cmd_disp_off},
    {DTYPE_GEN_LWRITE, 1, 0, 0, 0, sizeof(mipi_sharp_cmd_passwd1_l2_unmask), mipi_sharp_cmd_passwd1_l2_unmask},
    {DTYPE_GEN_LWRITE, 1, 0, 0, 0, sizeof(mipi_sharp_cmd_crclk_ctl_high), mipi_sharp_cmd_crclk_ctl_high},
    {DTYPE_GEN_LWRITE, 1, 0, 0, 0, sizeof(mipi_sharp_cmd_passwd1_l2_mask), mipi_sharp_cmd_passwd1_l2_mask}
};

static char mipi_sharp_cmd_sleepin[2] = {0x10, 0x00};
static char mipi_sharp_cmd_standby[2] = {0xBD, 0x01};

static struct dsi_cmd_desc mipi_sharp_cmds_sleepin[] = {
    {DTYPE_GEN_LWRITE, 1, 0, 0, 0, sizeof(mipi_sharp_cmd_passwd1_l2_unmask), mipi_sharp_cmd_passwd1_l2_unmask},
    {DTYPE_DCS_WRITE, 1, 0, 0, 120, sizeof(mipi_sharp_cmd_sleepin), mipi_sharp_cmd_sleepin}
};

static struct dsi_cmd_desc mipi_sharp_cmds_standby[] = {
    {DTYPE_GEN_WRITE2, 1, 0, 0, 100, sizeof(mipi_sharp_cmd_standby), mipi_sharp_cmd_standby}
};

#if defined(CONFIG_SHDISP_USE_CABC)
static struct dsi_cmd_desc mipi_sharp_cmds_cabc_init_set_pwm_clk[] = {
    {DTYPE_GEN_LWRITE, 1, 0, 0, 0, sizeof(mipi_sharp_cmd_cabc_init_set_pwm_clk), mipi_sharp_cmd_cabc_init_set_pwm_clk}
};
static struct dsi_cmd_desc mipi_sharp_cmds_cabc_init_set_pwm_osc[] = {
    {DTYPE_GEN_LWRITE, 1, 0, 0, 0, sizeof(mipi_sharp_cmd_cabc_init_set_pwm_osc), mipi_sharp_cmd_cabc_init_set_pwm_osc}
};
static struct dsi_cmd_desc mipi_sharp_cmds_cabc_init_set_para[] = {
    {DTYPE_DCS_LWRITE, 1, 0, 0, 0, sizeof(mipi_sharp_cmd_cabc_init_set_param), mipi_sharp_cmd_cabc_init_set_param}
};
static struct dsi_cmd_desc mipi_sharp_cmds_cabc_init_set_flicker[] = {
    {DTYPE_GEN_LWRITE, 1, 0, 0, 0, sizeof(mipi_sharp_cmd_cabc_init_set_flicker), mipi_sharp_cmd_cabc_init_set_flicker}
};
static struct dsi_cmd_desc mipi_sharp_cmds_cabc_init_set_connector[] = {
    {DTYPE_DCS_WRITE1, 1, 0, 0, 0, sizeof(mipi_sharp_cmd_cabc_init_set_connector), mipi_sharp_cmd_cabc_init_set_connector}
};

static struct dsi_cmd_desc mipi_sharp_cmds_cabc_indoor_set_image_exrate_lut0[] = {
    {DTYPE_GEN_LWRITE, 1, 0, 0, 0, sizeof(mipi_sharp_cmd_cabc_indoor_set_image_exrate_lut0), mipi_sharp_cmd_cabc_indoor_set_image_exrate_lut0}
};
static struct dsi_cmd_desc mipi_sharp_cmds_cabc_indoor_set_on[] = {
    {DTYPE_DCS_WRITE1, 1, 0, 0, 0, sizeof(mipi_sharp_cmd_cabc_indoor_set_on), mipi_sharp_cmd_cabc_indoor_set_on}
};

static struct dsi_cmd_desc mipi_sharp_cmds_cabc_outdoor_set_image_exrate_lut1[] = {
    {DTYPE_GEN_LWRITE, 1, 0, 0, 0, sizeof(mipi_sharp_cmd_cabc_outdoor_set_image_exrate_lut1), mipi_sharp_cmd_cabc_outdoor_set_image_exrate_lut1}
};
static struct dsi_cmd_desc mipi_sharp_cmds_cabc_outdoor_set_image_exrate_lut2[] = {
    {DTYPE_GEN_LWRITE, 1, 0, 0, 0, sizeof(mipi_sharp_cmd_cabc_outdoor_set_image_exrate_lut2), mipi_sharp_cmd_cabc_outdoor_set_image_exrate_lut2}
};
static struct dsi_cmd_desc mipi_sharp_cmds_cabc_outdoor_set_image_exrate_lut3[] = {
    {DTYPE_GEN_LWRITE, 1, 0, 0, 0, sizeof(mipi_sharp_cmd_cabc_outdoor_set_image_exrate_lut3), mipi_sharp_cmd_cabc_outdoor_set_image_exrate_lut3}
};
static struct dsi_cmd_desc mipi_sharp_cmds_cabc_outdoor_set_image_exrate_lut4[] = {
    {DTYPE_GEN_LWRITE, 1, 0, 0, 0, sizeof(mipi_sharp_cmd_cabc_outdoor_set_image_exrate_lut4), mipi_sharp_cmd_cabc_outdoor_set_image_exrate_lut4}
};
static struct dsi_cmd_desc mipi_sharp_cmds_cabc_outdoor_set_image_exrate_lut5[] = {
    {DTYPE_GEN_LWRITE, 1, 0, 0, 0, sizeof(mipi_sharp_cmd_cabc_outdoor_set_image_exrate_lut5), mipi_sharp_cmd_cabc_outdoor_set_image_exrate_lut5}
};
static struct dsi_cmd_desc mipi_sharp_cmds_cabc_outdoor_set_on[] = {
    {DTYPE_DCS_WRITE1, 1, 0, 0, 0, sizeof(mipi_sharp_cmd_cabc_outdoor_set_on), mipi_sharp_cmd_cabc_outdoor_set_on}
};

static struct dsi_cmd_desc mipi_sharp_cmds_cabc_off_set_image_exrate_lut0[] = {
    {DTYPE_GEN_LWRITE, 1, 0, 0, 0, sizeof(mipi_sharp_cmd_cabc_off_set_image_exrate_lut0), mipi_sharp_cmd_cabc_off_set_image_exrate_lut0}
};
static struct dsi_cmd_desc mipi_sharp_cmds_cabc_off_set_on[] = {
    {DTYPE_DCS_WRITE1, 1, 0, 0, 0, sizeof(mipi_sharp_cmd_cabc_off_set_on), mipi_sharp_cmd_cabc_off_set_on}
};

static struct dsi_cmd_desc mipi_sharp_cmds_cabc_transition_set_image_exrate_lut0[] = {
    {DTYPE_GEN_LWRITE, 1, 0, 0, 0, sizeof(mipi_sharp_cmd_cabc_transition_set_image_exrate_lut0), mipi_sharp_cmd_cabc_transition_set_image_exrate_lut0}
};
static struct dsi_cmd_desc mipi_sharp_cmds_cabc_transition_set_image_exrate_lut1[] = {
    {DTYPE_GEN_LWRITE, 1, 0, 0, 0, sizeof(mipi_sharp_cmd_cabc_transition_set_image_exrate_lut1), mipi_sharp_cmd_cabc_transition_set_image_exrate_lut1}
};
static struct dsi_cmd_desc mipi_sharp_cmds_cabc_transition_set_image_exrate_lut2[] = {
    {DTYPE_GEN_LWRITE, 1, 0, 0, 0, sizeof(mipi_sharp_cmd_cabc_transition_set_image_exrate_lut2), mipi_sharp_cmd_cabc_transition_set_image_exrate_lut2}
};
static struct dsi_cmd_desc mipi_sharp_cmds_cabc_transition_set_image_exrate_lut3[] = {
    {DTYPE_GEN_LWRITE, 1, 0, 0, 0, sizeof(mipi_sharp_cmd_cabc_transition_set_image_exrate_lut3), mipi_sharp_cmd_cabc_transition_set_image_exrate_lut3}
};
static struct dsi_cmd_desc mipi_sharp_cmds_cabc_transition_set_image_exrate_lut4[] = {
    {DTYPE_GEN_LWRITE, 1, 0, 0, 0, sizeof(mipi_sharp_cmd_cabc_transition_set_image_exrate_lut4), mipi_sharp_cmd_cabc_transition_set_image_exrate_lut4}
};
static struct dsi_cmd_desc mipi_sharp_cmds_cabc_transition_set_image_exrate_lut5[] = {
    {DTYPE_GEN_LWRITE, 1, 0, 0, 0, sizeof(mipi_sharp_cmd_cabc_transition_set_image_exrate_lut5), mipi_sharp_cmd_cabc_transition_set_image_exrate_lut5}
};

static struct dsi_cmd_desc mipi_sharp_cmds_cabc_lut0_gamma[] = {
    {DTYPE_GEN_LWRITE, 1, 0, 0, 0, sizeof(mipi_sharp_cmd_cabc_lut0_gamma_a), mipi_sharp_cmd_cabc_lut0_gamma_a},
    {DTYPE_GEN_LWRITE, 1, 0, 0, 0, sizeof(mipi_sharp_cmd_cabc_lut0_gamma_b), mipi_sharp_cmd_cabc_lut0_gamma_b},
    {DTYPE_GEN_LWRITE, 1, 0, 0, 0, sizeof(mipi_sharp_cmd_cabc_lut0_gamma_c), mipi_sharp_cmd_cabc_lut0_gamma_c}
};
static struct dsi_cmd_desc mipi_sharp_cmds_cabc_lut1_gamma[] = {
    {DTYPE_GEN_LWRITE, 1, 0, 0, 0, sizeof(mipi_sharp_cmd_cabc_lut1_gamma_a), mipi_sharp_cmd_cabc_lut1_gamma_a},
    {DTYPE_GEN_LWRITE, 1, 0, 0, 0, sizeof(mipi_sharp_cmd_cabc_lut1_gamma_b), mipi_sharp_cmd_cabc_lut1_gamma_b},
    {DTYPE_GEN_LWRITE, 1, 0, 0, 0, sizeof(mipi_sharp_cmd_cabc_lut1_gamma_c), mipi_sharp_cmd_cabc_lut1_gamma_c}
};
static struct dsi_cmd_desc mipi_sharp_cmds_cabc_lut2_gamma[] = {
    {DTYPE_GEN_LWRITE, 1, 0, 0, 0, sizeof(mipi_sharp_cmd_cabc_lut2_gamma_a), mipi_sharp_cmd_cabc_lut2_gamma_a},
    {DTYPE_GEN_LWRITE, 1, 0, 0, 0, sizeof(mipi_sharp_cmd_cabc_lut2_gamma_b), mipi_sharp_cmd_cabc_lut2_gamma_b},
    {DTYPE_GEN_LWRITE, 1, 0, 0, 0, sizeof(mipi_sharp_cmd_cabc_lut2_gamma_c), mipi_sharp_cmd_cabc_lut2_gamma_c}
};
static struct dsi_cmd_desc mipi_sharp_cmds_cabc_lut3_gamma[] = {
    {DTYPE_GEN_LWRITE, 1, 0, 0, 0, sizeof(mipi_sharp_cmd_cabc_lut3_gamma_a), mipi_sharp_cmd_cabc_lut3_gamma_a},
    {DTYPE_GEN_LWRITE, 1, 0, 0, 0, sizeof(mipi_sharp_cmd_cabc_lut3_gamma_b), mipi_sharp_cmd_cabc_lut3_gamma_b},
    {DTYPE_GEN_LWRITE, 1, 0, 0, 0, sizeof(mipi_sharp_cmd_cabc_lut3_gamma_c), mipi_sharp_cmd_cabc_lut3_gamma_c}
};
static struct dsi_cmd_desc mipi_sharp_cmds_cabc_lut4_gamma[] = {
    {DTYPE_GEN_LWRITE, 1, 0, 0, 0, sizeof(mipi_sharp_cmd_cabc_lut4_gamma_a), mipi_sharp_cmd_cabc_lut4_gamma_a},
    {DTYPE_GEN_LWRITE, 1, 0, 0, 0, sizeof(mipi_sharp_cmd_cabc_lut4_gamma_b), mipi_sharp_cmd_cabc_lut4_gamma_b},
    {DTYPE_GEN_LWRITE, 1, 0, 0, 0, sizeof(mipi_sharp_cmd_cabc_lut4_gamma_c), mipi_sharp_cmd_cabc_lut4_gamma_c}
};
static struct dsi_cmd_desc mipi_sharp_cmds_cabc_lut5_gamma[] = {
    {DTYPE_GEN_LWRITE, 1, 0, 0, 0, sizeof(mipi_sharp_cmd_cabc_lut5_gamma_a), mipi_sharp_cmd_cabc_lut5_gamma_a},
    {DTYPE_GEN_LWRITE, 1, 0, 0, 0, sizeof(mipi_sharp_cmd_cabc_lut5_gamma_b), mipi_sharp_cmd_cabc_lut5_gamma_b},
    {DTYPE_GEN_LWRITE, 1, 0, 0, 0, sizeof(mipi_sharp_cmd_cabc_lut5_gamma_c), mipi_sharp_cmd_cabc_lut5_gamma_c}
};
#endif /* CONFIG_SHDISP_USE_CABC */

static int mipi_sharp_flute_lcd_on(struct platform_device *pdev);
static int mipi_sharp_flute_lcd_off(struct platform_device *pdev);
static void mipi_sharp_flute_lcd_set_backlight(struct msm_fb_data_type *mfd);

#if defined(CONFIG_SHDISP_USE_CABC)
static int mipi_sharp_cabc_lcd_backlight_on(void);
static int mipi_sharp_cabc_disp_on_ctrl(struct msm_fb_data_type *mfd, struct mipi_panel_info *mipi);
static int mipi_sharp_cmd_sqe_cabc_init(struct msm_fb_data_type *mfd, struct mipi_panel_info *mipi);
static int mipi_sharp_cmd_sqe_cabc_indoor_on(struct msm_fb_data_type *mfd, struct mipi_panel_info *mipi);
static int mipi_sharp_cmd_sqe_cabc_outdoor_on(struct msm_fb_data_type *mfd, struct mipi_panel_info *mipi, int lut_level);
static int mipi_sharp_cmd_sqe_cabc_off(struct msm_fb_data_type *mfd, struct mipi_panel_info *mipi, int wait_on);
static int mipi_sharp_cmd_sqe_cabc_outdoor_move(struct msm_fb_data_type *mfd, struct mipi_panel_info *mipi, int lut_level);
#endif /* CONFIG_SHDISP_USE_CABC */

#define PM8921_GPIO_BASE        NR_GPIO_IRQS
#define PM8921_GPIO_PM_TO_SYS(pm_gpio)  (pm_gpio - 1 + PM8921_GPIO_BASE)

static int mipi_sharp_flute_read_reg_byte(struct msm_fb_data_type *mfd,
										 unsigned char addr, unsigned char *rdata)
{
	struct dsi_buf *rp, *tp;
	struct dsi_cmd_desc cmd[1];
	char cmd_buf[2];

	tp = &sharp_tx_buf;
	rp = &sharp_rx_buf;

	cmd_buf[0] = addr;
	cmd_buf[1] = 0x00;

	cmd[0].dtype = DTYPE_GEN_READ1;
	cmd[0].last = 0x01;
	cmd[0].vc = 0x00;
	cmd[0].ack = 0x01;
	cmd[0].wait = 0x00;
	cmd[0].dlen = 1;
	cmd[0].payload = cmd_buf;

	mipi_dsi_cmds_rx(mfd, tp, rp, cmd, 2);



    *rdata = *(rp->data);

	return 0;
}

static int mipi_sharp_flute_write_reg_byte(struct msm_fb_data_type *mfd,
										 unsigned char addr, unsigned char wdata, int dtype)
{
	struct dsi_cmd_desc cmd[1];
	char cmd_buf[2];

	cmd_buf[0] = addr;
	cmd_buf[1] = wdata;

	cmd[0].dtype = dtype;
	cmd[0].last = 0x01;
	cmd[0].vc = 0x00;
	cmd[0].ack = 0x00;
	cmd[0].wait = 0x00;
	cmd[0].dlen = 2;
	cmd[0].payload = cmd_buf;

	mipi_dsi_cmds_tx(&sharp_tx_buf, cmd, ARRAY_SIZE(cmd));

	return 0;
}

static int mipi_sharp_flute_read_reg(unsigned char addr, unsigned char *read_data, unsigned char size)
{
	struct msm_fb_data_type *mfd;
	struct mipi_panel_info *mipi;
	struct fb_info *info = NULL;
	int i = 0;
	uint32 data;
	unsigned char rdata = 0, gpara = 0;
	unsigned char waddr = MIPI_SHARP_FLUTE_GPARA_ADDR;


	if (!num_registered_fb){
		pr_err("%s: num_registered_fb == NULL, -ENODEV\n", __func__);
		return -ENODEV;
	}

	info = registered_fb[0];
	if (!info){
		pr_err("%s: registered_fb[0] == NULL, -ENODEV\n", __func__);
		return -ENODEV;
	}

	mfd = (struct msm_fb_data_type *)info->par;
	if (!mfd) {
		pr_err("%s: mfd == NULL, -ENODEV\n", __func__);
		return -ENODEV;
	}

	if (!mfd->panel_power_on) {
		pr_err("%s: power off, -ENODEV\n", __func__);
		return -ENODEV;
	}

	mipi = &mfd->panel_info.mipi;

	data = MIPI_INP(MIPI_DSI_BASE + 0x38);
	data &= BIT(26);

	mdp_pipe_ctrl(MDP_CMD_BLOCK, MDP_BLOCK_POWER_ON, FALSE);
	MDP_OUTP(MDP_BASE + 0xE0000, 0);
	mdp_pipe_ctrl(MDP_CMD_BLOCK, MDP_BLOCK_POWER_OFF, FALSE);

	mipi_dsi_controller_cfg(0);

	mipi_dsi_op_mode_config(DSI_CMD_MODE);

	mipi_dsi_sw_reset();

	if (data)
		mipi_set_tx_power_mode(0);

	mipi_dsi_cmds_tx(&sharp_tx_buf, mipi_sharp_cmds_passwd1_l2_unmask,
					ARRAY_SIZE(mipi_sharp_cmds_passwd1_l2_unmask));

	mipi_dsi_cmds_tx(&sharp_tx_buf, mipi_sharp_cmds_crclk_ctl_high,
					ARRAY_SIZE(mipi_sharp_cmds_crclk_ctl_high));

	mipi_dsi_cmds_tx(&sharp_tx_buf, mipi_sharp_cmds_passwd1_l2_mask,
					ARRAY_SIZE(mipi_sharp_cmds_passwd1_l2_mask));
	mipi_sharp_delay_us(1000);

	mipi_set_tx_power_mode(1);



	mipi_dsi_cmds_tx(&sharp_tx_buf, mipi_sharp_cmds_passwd1_l2_unmask,
					ARRAY_SIZE(mipi_sharp_cmds_passwd1_l2_unmask));

	for (i=0; i<size; i++) {
		mipi_sharp_flute_read_reg_byte(mfd, addr, &rdata);
		*read_data++ = rdata;
		if (size == 1)
			break;
		if (i == size - 1)
			gpara = 0;
		else
			gpara++;
		mipi_sharp_flute_write_reg_byte(mfd, waddr, gpara, DTYPE_GEN_WRITE2);
	}

	mipi_dsi_cmds_tx(&sharp_tx_buf, mipi_sharp_cmds_passwd1_l2_mask,
						ARRAY_SIZE(mipi_sharp_cmds_passwd1_l2_mask));


	mipi_set_tx_power_mode(0);

	mipi_dsi_cmds_tx(&sharp_tx_buf, mipi_sharp_cmds_passwd1_l2_unmask,
					ARRAY_SIZE(mipi_sharp_cmds_passwd1_l2_unmask));

	mipi_dsi_cmds_tx(&sharp_tx_buf, mipi_sharp_cmds_crclk_ctl_low,
					ARRAY_SIZE(mipi_sharp_cmds_crclk_ctl_low));

	mipi_dsi_cmds_tx(&sharp_tx_buf, mipi_sharp_cmds_passwd1_l2_mask,
					ARRAY_SIZE(mipi_sharp_cmds_passwd1_l2_mask));
	mipi_sharp_delay_us(1000);

	mipi_set_tx_power_mode(data);

	mipi_dsi_op_mode_config(DSI_VIDEO_MODE);

	mdp_pipe_ctrl(MDP_CMD_BLOCK, MDP_BLOCK_POWER_ON, FALSE);
	MDP_OUTP(MDP_BASE + 0xE0000, 1);
	mdp_pipe_ctrl(MDP_CMD_BLOCK, MDP_BLOCK_POWER_OFF, FALSE);


    return 0;
}

static int mipi_sharp_flute_write_reg(unsigned char addr, unsigned char *write_data, unsigned char size)
{
    struct msm_fb_data_type *mfd;
    struct fb_info *info = NULL;
    struct dsi_cmd_desc cmd[1];
    char cmd_buf[MIPI_SHARP_RW_MAX_SIZE+1];
    
    
    if (!num_registered_fb){
        pr_err("%s: num_registered_fb == NULL, -ENODEV\n", __func__);
        return -ENODEV;
    }
    
    info = registered_fb[0];
    if (!info){
        pr_err("%s: registered_fb[0] == NULL, -ENODEV\n", __func__);
        return -ENODEV;
    }
    
    mfd = (struct msm_fb_data_type *)info->par;
    if (!mfd) {
        pr_err("%s: mfd == NULL, -ENODEV\n", __func__);
        return -ENODEV;
    }
    
    if (!mfd->panel_power_on) {
        pr_err("%s: power off, -ENODEV\n", __func__);
        return -ENODEV;
    }
    
    cmd_buf[0] = addr;
    cmd_buf[1] = 0x00;
    memcpy(&cmd_buf[1], write_data, size);
    
    if(size == 0) {
        cmd[0].dtype = DTYPE_DCS_WRITE;
    } else if(size == 1) {
        cmd[0].dtype = DTYPE_GEN_WRITE2;
    } else{
        cmd[0].dtype = DTYPE_GEN_LWRITE;
    }
    
    cmd[0].last = 0x01;
    cmd[0].vc = 0x00;
    cmd[0].ack = 0x00;
    cmd[0].wait = 0x00;
    cmd[0].dlen = size + 1;
    cmd[0].payload = cmd_buf;
    
    mutex_lock(&mfd->dma->ov_mutex);
    
    if(addr == 0xFE){
        mipi_dsi_cmds_tx(&sharp_tx_buf, mipi_sharp_cmds_passwd1_l3_unmask,
                    ARRAY_SIZE(mipi_sharp_cmds_passwd1_l3_unmask));
    }
    else{
        mipi_dsi_cmds_tx(&sharp_tx_buf, mipi_sharp_cmds_passwd1_l2_unmask,
                    ARRAY_SIZE(mipi_sharp_cmds_passwd1_l2_unmask));
    }

    mipi_dsi_cmds_tx(&sharp_tx_buf, cmd, ARRAY_SIZE(cmd));
    
    if(addr == 0xFE){
        mipi_dsi_cmds_tx(&sharp_tx_buf, mipi_sharp_cmds_passwd1_l3_mask,
                        ARRAY_SIZE(mipi_sharp_cmds_passwd1_l3_mask));
    }
    else{
        mipi_dsi_cmds_tx(&sharp_tx_buf, mipi_sharp_cmds_passwd1_l2_mask,
                        ARRAY_SIZE(mipi_sharp_cmds_passwd1_l2_mask));
    }

    mutex_unlock(&mfd->dma->ov_mutex);
    
    return 0;
}

static int mipi_sharp_tx_vcom_write(struct msm_fb_data_type *mfd)
{
    struct dsi_cmd_desc cmd[1];
    char cmd_buf[12];
    unsigned short alpha;
    
    memcpy(&cmd_buf[0], mipi_sharp_cmd_pwrctl2, 7);
    
    alpha = shdisp_api_get_alpha();
    
    cmd_buf[3]  = (unsigned char) (alpha & 0xFF);
    
    cmd[0].dtype = DTYPE_GEN_LWRITE;
    cmd[0].last = 0x01;
    cmd[0].vc = 0x00;
    cmd[0].ack = 0x00;
    cmd[0].wait = 0x00;
    cmd[0].dlen = 7;
    cmd[0].payload = cmd_buf;
    
    mipi_dsi_cmds_tx(&sharp_tx_buf, cmd, ARRAY_SIZE(cmd));
    
    return 0;
}

static int mipi_sharp_cmd_lcd_on(struct msm_fb_data_type *mfd, struct mipi_panel_info *mipi)
{

    if(shdisp_api_get_upper_unit() != SHDISP_RESULT_SUCCESS) {
        return -ENODEV;
    }

    if(lcdon_cmd_send) {
        return 0;
    }

    shdisp_api_main_lcd_reset();

    mipi_dsi_cmds_tx(&sharp_tx_buf, mipi_sharp_cmds_sleepout,
                     ARRAY_SIZE(mipi_sharp_cmds_sleepout));


    mipi_dsi_cmds_tx(&sharp_tx_buf, mipi_sharp_cmds_passwd1_l2_unmask,
                     ARRAY_SIZE(mipi_sharp_cmds_passwd1_l2_unmask));
    mipi_dsi_cmds_tx(&sharp_tx_buf, mipi_sharp_cmds_resol,
                     ARRAY_SIZE(mipi_sharp_cmds_resol));
    mipi_dsi_cmds_tx(&sharp_tx_buf, mipi_sharp_cmds_panelctl1_for_video,
                     ARRAY_SIZE(mipi_sharp_cmds_panelctl1_for_video));
    mipi_dsi_cmds_tx(&sharp_tx_buf, mipi_sharp_cmds_dispctl_for_video,
                     ARRAY_SIZE(mipi_sharp_cmds_dispctl_for_video));
    mipi_dsi_cmds_tx(&sharp_tx_buf, mipi_sharp_cmds_power,
                     ARRAY_SIZE(mipi_sharp_cmds_power));
    mipi_sharp_tx_vcom_write(mfd);
    mipi_dsi_cmds_tx(&sharp_tx_buf, mipi_sharp_cmds_srcctl,
                     ARRAY_SIZE(mipi_sharp_cmds_srcctl));
    mipi_dsi_cmds_tx(&sharp_tx_buf, mipi_sharp_cmds_panelctl,
                     ARRAY_SIZE(mipi_sharp_cmds_panelctl));
    mipi_dsi_cmds_tx(&sharp_tx_buf, mipi_sharp_cmds_pgammactl,
                     ARRAY_SIZE(mipi_sharp_cmds_pgammactl));
    mipi_dsi_cmds_tx(&sharp_tx_buf, mipi_sharp_cmds_ngammactl,
                     ARRAY_SIZE(mipi_sharp_cmds_ngammactl));
    mipi_dsi_cmds_tx(&sharp_tx_buf, mipi_sharp_cmds_passwd1_l2_mask,
                     ARRAY_SIZE(mipi_sharp_cmds_passwd1_l2_mask));
    mipi_dsi_cmds_tx(&sharp_tx_buf, mipi_sharp_cmds_passwd1_l3_unmask,
                     ARRAY_SIZE(mipi_sharp_cmds_passwd1_l3_unmask));
    mipi_dsi_cmds_tx(&sharp_tx_buf, mipi_sharp_cmds_logictest,
                     ARRAY_SIZE(mipi_sharp_cmds_logictest));
    mipi_dsi_cmds_tx(&sharp_tx_buf, mipi_sharp_cmds_passwd1_l3_mask,
                     ARRAY_SIZE(mipi_sharp_cmds_passwd1_l3_mask));



#if defined(CONFIG_SHDISP_USE_CABC)
    mipi_sharp_cabc_disp_on_ctrl(mfd, mipi);
#endif
    return 0;
}

static int mipi_sharp_cmd_lcd_display_on(struct msm_fb_data_type *mfd, struct mipi_panel_info *mipi)
{
	if(shdisp_api_get_upper_unit() != SHDISP_RESULT_SUCCESS) {
		return -ENODEV;
	}

    if(lcdon_cmd_send) {
        return 0;
    }

    mipi_dsi_cmds_tx(&sharp_tx_buf, mipi_sharp_cmds_disp_on_for_video,
                     ARRAY_SIZE(mipi_sharp_cmds_disp_on_for_video));
	return 0;
}

static int mipi_sharp_cmd_lcd_display_off(struct msm_fb_data_type *mfd, struct mipi_panel_info *mipi)
{
    if(shdisp_api_get_upper_unit() != SHDISP_RESULT_SUCCESS) {
        return 0;
    }

    mipi_dsi_cmds_tx(&sharp_tx_buf, mipi_sharp_cmds_disp_off_for_video,
                     ARRAY_SIZE(mipi_sharp_cmds_disp_off_for_video));
    mipi_sharp_delay_us(1000);

    mipi_dsi_cmds_tx(&sharp_tx_buf, mipi_sharp_cmds_sleepin,
                     ARRAY_SIZE(mipi_sharp_cmds_sleepin));

    return 0;
}

static int mipi_sharp_cmd_lcd_deep_stanby(struct msm_fb_data_type *mfd, struct mipi_panel_info *mipi)
{
    if(shdisp_api_get_upper_unit() != SHDISP_RESULT_SUCCESS) {
        return 0;
    }

    mipi_dsi_cmds_tx(&sharp_tx_buf, mipi_sharp_cmds_standby,
                     ARRAY_SIZE(mipi_sharp_cmds_standby));

    return 0;
}

static int mipi_sharp_flute_lcd_on_sleepout(void)
{
	mipi_dsi_cmds_tx(&sharp_tx_buf, mipi_sharp_cmds_passwd1_l2_unmask,
					ARRAY_SIZE(mipi_sharp_cmds_passwd1_l2_unmask));

	mipi_dsi_cmds_tx(&sharp_tx_buf, mipi_sharp_cmds_crclk_ctl_high,
					ARRAY_SIZE(mipi_sharp_cmds_crclk_ctl_high));
    mipi_sharp_delay_us(1000);

    mipi_dsi_cmds_tx(&sharp_tx_buf, mipi_sharp_cmds_sleepout,
                     ARRAY_SIZE(mipi_sharp_cmds_sleepout));

	mipi_dsi_cmds_tx(&sharp_tx_buf, mipi_sharp_cmds_crclk_ctl_low,
					ARRAY_SIZE(mipi_sharp_cmds_crclk_ctl_low));

	mipi_dsi_cmds_tx(&sharp_tx_buf, mipi_sharp_cmds_passwd1_l2_mask,
					ARRAY_SIZE(mipi_sharp_cmds_passwd1_l2_mask));

    return 0;
}

static int mipi_sharp_flute_lcd_on(struct platform_device *pdev)
{
    struct msm_fb_data_type *mfd;
    struct mipi_panel_info *mipi;
    int ret = 0;
    
    mfd = platform_get_drvdata(pdev);
    if (!mfd) {
        return -ENODEV;
    }
    if (mfd->key != MFD_KEY) {
        return -EINVAL;
    }
    
    mipi = &mfd->panel_info.mipi;
    ret = mipi_sharp_cmd_lcd_on(mfd, mipi);
    
    return ret;
}

static int mipi_sharp_flute_lcd_off(struct platform_device *pdev)
{
    struct msm_fb_data_type *mfd;
    struct mipi_panel_info *mipi;

    mfd = platform_get_drvdata(pdev);
    if (!mfd) {
        return -ENODEV;
    }
    if (mfd->key != MFD_KEY) {
        return -EINVAL;
	}

    mipi = &mfd->panel_info.mipi;

	if (!(msm_fb_shutdown_in_progress || (mipi_sharp_is_recovery()))){
		mipi_sharp_cmd_lcd_deep_stanby(mfd, mipi);
		mipi_set_tx_power_mode(1);
	}

    return 0;
}

static int mipi_sharp_flute_lcd_start_display(struct msm_fb_data_type *mfd)
{
	struct mipi_panel_info *mipi;
	int ret = 0;

	if (mfd->key != MFD_KEY) {
		return -EINVAL;
	}
	mipi = &mfd->panel_info.mipi;
	if (mipi->mode == DSI_VIDEO_MODE) {
		mutex_lock(&mfd->dma->ov_mutex);
		ret = mipi_sharp_cmd_lcd_display_on(mfd, mipi);
		mutex_unlock(&mfd->dma->ov_mutex);
	}

	return ret;
}

static void mipi_sharp_flute_lcd_set_backlight(struct msm_fb_data_type *mfd)
{
	struct shdisp_main_bkl_ctl bkl;

	if (mfd->bl_level == 0) {
		shdisp_api_main_bkl_off();
	}else{
		bkl.mode = SHDISP_MAIN_BKL_MODE_FIX;
		bkl.param = mfd->bl_level;
		shdisp_api_main_bkl_on(&bkl);
	}
}

static int mipi_sharp_flute_lcd_stop_display(struct msm_fb_data_type *mfd)
{
	struct mipi_panel_info *mipi;
	int ret = 0;

	if (mfd->key != MFD_KEY) {
		return -EINVAL;
	}
	mipi = &mfd->panel_info.mipi;
	if (mipi->mode == DSI_VIDEO_MODE) {
		mutex_lock(&mfd->dma->ov_mutex);
		ret = mipi_sharp_cmd_lcd_display_off(mfd, mipi);
		mutex_unlock(&mfd->dma->ov_mutex);
	}

	return ret;
}

int mipi_sharp_flute_lcd_check_display(void)
{
	int ret, retry = 0;

	if (mipi_sharp_is_recovery()){
	    return 0;
	}

	if(shdisp_api_get_upper_unit() != SHDISP_RESULT_SUCCESS) {
		return -ENODEV;
	}

    for (retry = 0; retry < 5; retry++) {
		ret = shdisp_api_check_recovery();
		if (ret == 0) {
			break;
		} else {
			mipi_sharp_flute_lcd_on_sleepout();
		}
	}

    return 0;
}
#if defined(CONFIG_SHDISP_USE_CABC)
static int mipi_sharp_cabc_lcd_backlight_on(void)
{
	int rc;

	static struct pm_gpio pwm_mode = {
		.direction        = PM_GPIO_DIR_OUT,
		.output_buffer    = PM_GPIO_OUT_BUF_CMOS,
		.output_value     = 1,
		.pull             = PM_GPIO_PULL_NO,
		.vin_sel          = PM_GPIO_VIN_S4,
		.out_strength     = PM_GPIO_STRENGTH_HIGH,
		.function         = PM_GPIO_FUNC_NORMAL,
		.inv_int_pol      = 0,
		.disable_pin      = 0,
	};

	

	rc = gpio_request(PM8921_GPIO_PM_TO_SYS(24), "shspamp");
	if (rc) {
		pr_err("%s: Failed to request gpio\n", __func__);
		goto err_free_gpio;
	}

	rc = pm8xxx_gpio_config(PM8921_GPIO_PM_TO_SYS(24), &pwm_mode);
	if (rc) {
		pr_err("%s: pwm_mode failed\n", __func__);
		goto err_free_gpio;
	}
	gpio_direction_output(PM8921_GPIO_PM_TO_SYS(24), 1);
	
err_free_gpio:
    gpio_free(PM8921_GPIO_PM_TO_SYS(24));

	
	return 0;
}
#endif


#if defined(CONFIG_SHDISP_USE_CABC)
static int mipi_sharp_cabc_disp_on_ctrl(struct msm_fb_data_type *mfd, struct mipi_panel_info *mipi)
{
    struct shdisp_check_cabc_val value;

	mipi_sharp_cabc_lcd_backlight_on();

    shdisp_api_photo_sensor_pow_ctl();

    shdisp_api_check_cabc(&value);

    if(value.mode == SHDISP_MAIN_DISP_CABC_MODE_OFF)
    {
        
        mipi_sharp_cmd_sqe_cabc_off(mfd, mipi, 0);
    }
    else if(value.mode == SHDISP_MAIN_DISP_CABC_MODE_DBC)
    {
        
        mipi_sharp_cmd_sqe_cabc_init(mfd, mipi);

        mipi_sharp_cmd_sqe_cabc_indoor_on(mfd, mipi);
    }
    else if(value.mode == SHDISP_MAIN_DISP_CABC_MODE_ACC)
    {
        
        if(value.lut == SHDISP_MAIN_DISP_CABC_LUT0)
        {
            mipi_sharp_cmd_sqe_cabc_off(mfd, mipi, 0);
        }
        else
        {
            mipi_sharp_cmd_sqe_cabc_init(mfd, mipi);

            mipi_sharp_cmd_sqe_cabc_outdoor_on(mfd, mipi, value.lut);
        }
    }
    else if(value.mode == SHDISP_MAIN_DISP_CABC_MODE_DBC_ACC)
    {

        mipi_sharp_cmd_sqe_cabc_init(mfd, mipi);

        if(value.lut > 0)
        {
            mipi_sharp_cmd_sqe_cabc_outdoor_on(mfd, mipi, value.lut);
        }
        else
        {
            mipi_sharp_cmd_sqe_cabc_indoor_on(mfd, mipi);
        }

        
    }

    return 0;
}


static int mipi_sharp_cmd_sqe_cabc_init(struct msm_fb_data_type *mfd, struct mipi_panel_info *mipi)
{
    int clk_check;
    
    clk_check = shdisp_api_get_clock_info();
    if (clk_check == 0) {
        mipi_dsi_cmds_tx(&sharp_tx_buf, mipi_sharp_cmds_cabc_init_set_pwm_clk,
                         ARRAY_SIZE(mipi_sharp_cmds_cabc_init_set_pwm_clk));
    } else {
        mipi_dsi_cmds_tx(&sharp_tx_buf, mipi_sharp_cmds_cabc_init_set_pwm_osc,
                         ARRAY_SIZE(mipi_sharp_cmds_cabc_init_set_pwm_osc));
    }
    mipi_dsi_cmds_tx(&sharp_tx_buf, mipi_sharp_cmds_cabc_init_set_para,
                     ARRAY_SIZE(mipi_sharp_cmds_cabc_init_set_para));
    mipi_dsi_cmds_tx(&sharp_tx_buf, mipi_sharp_cmds_cabc_init_set_flicker,
                     ARRAY_SIZE(mipi_sharp_cmds_cabc_init_set_flicker));
    mipi_dsi_cmds_tx(&sharp_tx_buf, mipi_sharp_cmds_cabc_init_set_connector,
                     ARRAY_SIZE(mipi_sharp_cmds_cabc_init_set_connector));
    shdisp_api_pwm_enable();

    return 0;
}

static int mipi_sharp_cmd_sqe_cabc_indoor_on(struct msm_fb_data_type *mfd, struct mipi_panel_info *mipi)
{
    mipi_dsi_cmds_tx(&sharp_tx_buf, mipi_sharp_cmds_cabc_indoor_set_image_exrate_lut0,
                     ARRAY_SIZE(mipi_sharp_cmds_cabc_indoor_set_image_exrate_lut0));
    mipi_dsi_cmds_tx(&sharp_tx_buf, mipi_sharp_cmds_cabc_indoor_set_on,
                     ARRAY_SIZE(mipi_sharp_cmds_cabc_indoor_set_on));

    return 0;
}

static int mipi_sharp_cmd_sqe_cabc_outdoor_on(struct msm_fb_data_type *mfd, struct mipi_panel_info *mipi, int lut_level)
{
    switch(lut_level)
    {
    case SHDISP_MAIN_DISP_CABC_LUT1:
        mipi_dsi_cmds_tx(&sharp_tx_buf, mipi_sharp_cmds_cabc_outdoor_set_image_exrate_lut1,
                         ARRAY_SIZE(mipi_sharp_cmds_cabc_outdoor_set_image_exrate_lut1));
        mipi_dsi_cmds_tx(&sharp_tx_buf, mipi_sharp_cmds_cabc_outdoor_set_on,
                         ARRAY_SIZE(mipi_sharp_cmds_cabc_outdoor_set_on));
        mipi_dsi_cmds_tx(&sharp_tx_buf, mipi_sharp_cmds_cabc_lut1_gamma,
                         ARRAY_SIZE(mipi_sharp_cmds_cabc_lut1_gamma));
        break;
    case SHDISP_MAIN_DISP_CABC_LUT2:
        mipi_dsi_cmds_tx(&sharp_tx_buf, mipi_sharp_cmds_cabc_outdoor_set_image_exrate_lut2,
                         ARRAY_SIZE(mipi_sharp_cmds_cabc_outdoor_set_image_exrate_lut2));
        mipi_dsi_cmds_tx(&sharp_tx_buf, mipi_sharp_cmds_cabc_outdoor_set_on,
                         ARRAY_SIZE(mipi_sharp_cmds_cabc_outdoor_set_on));
        mipi_dsi_cmds_tx(&sharp_tx_buf, mipi_sharp_cmds_cabc_lut2_gamma,
                         ARRAY_SIZE(mipi_sharp_cmds_cabc_lut2_gamma));
        break;
    case SHDISP_MAIN_DISP_CABC_LUT3:
        mipi_dsi_cmds_tx(&sharp_tx_buf, mipi_sharp_cmds_cabc_outdoor_set_image_exrate_lut3,
                         ARRAY_SIZE(mipi_sharp_cmds_cabc_outdoor_set_image_exrate_lut3));
        mipi_dsi_cmds_tx(&sharp_tx_buf, mipi_sharp_cmds_cabc_outdoor_set_on,
                         ARRAY_SIZE(mipi_sharp_cmds_cabc_outdoor_set_on));
        mipi_dsi_cmds_tx(&sharp_tx_buf, mipi_sharp_cmds_cabc_lut3_gamma,
                         ARRAY_SIZE(mipi_sharp_cmds_cabc_lut3_gamma));
        break;
    case SHDISP_MAIN_DISP_CABC_LUT4:
        mipi_dsi_cmds_tx(&sharp_tx_buf, mipi_sharp_cmds_cabc_outdoor_set_image_exrate_lut4,
                         ARRAY_SIZE(mipi_sharp_cmds_cabc_outdoor_set_image_exrate_lut4));
        mipi_dsi_cmds_tx(&sharp_tx_buf, mipi_sharp_cmds_cabc_outdoor_set_on,
                         ARRAY_SIZE(mipi_sharp_cmds_cabc_outdoor_set_on));
        mipi_dsi_cmds_tx(&sharp_tx_buf, mipi_sharp_cmds_cabc_lut4_gamma,
                         ARRAY_SIZE(mipi_sharp_cmds_cabc_lut4_gamma));
        break;
    case SHDISP_MAIN_DISP_CABC_LUT5:
        mipi_dsi_cmds_tx(&sharp_tx_buf, mipi_sharp_cmds_cabc_outdoor_set_image_exrate_lut5,
                         ARRAY_SIZE(mipi_sharp_cmds_cabc_outdoor_set_image_exrate_lut5));
        mipi_dsi_cmds_tx(&sharp_tx_buf, mipi_sharp_cmds_cabc_outdoor_set_on,
                         ARRAY_SIZE(mipi_sharp_cmds_cabc_outdoor_set_on));
        mipi_dsi_cmds_tx(&sharp_tx_buf, mipi_sharp_cmds_cabc_lut5_gamma,
                         ARRAY_SIZE(mipi_sharp_cmds_cabc_lut5_gamma));
        break;
	}

    return 0;
}

static int mipi_sharp_cmd_sqe_cabc_off(struct msm_fb_data_type *mfd, struct mipi_panel_info *mipi, int wait_on)
{
    mipi_dsi_cmds_tx(&sharp_tx_buf, mipi_sharp_cmds_cabc_lut0_gamma,
                     ARRAY_SIZE(mipi_sharp_cmds_cabc_lut0_gamma));
    
    if(wait_on == 0){
        mipi_dsi_cmds_tx(&sharp_tx_buf, mipi_sharp_cmds_cabc_off_set_image_exrate_lut0,
                 ARRAY_SIZE(mipi_sharp_cmds_cabc_off_set_image_exrate_lut0));
    }
    
    mipi_dsi_cmds_tx(&sharp_tx_buf, mipi_sharp_cmds_cabc_off_set_on,
                     ARRAY_SIZE(mipi_sharp_cmds_cabc_off_set_on));
    
    shdisp_api_pwm_disable();
    
    if(wait_on == 1){
		mipi_sharp_delay_us(300000);
    }

    return 0;
}

static int mipi_sharp_cmd_sqe_cabc_outdoor_move(struct msm_fb_data_type *mfd, struct mipi_panel_info *mipi, int lut_level)
{
    switch(lut_level)
    {
    case SHDISP_MAIN_DISP_CABC_LUT0:
        mipi_dsi_cmds_tx(&sharp_tx_buf, mipi_sharp_cmds_cabc_transition_set_image_exrate_lut0,
                         ARRAY_SIZE(mipi_sharp_cmds_cabc_transition_set_image_exrate_lut0));
        mipi_dsi_cmds_tx(&sharp_tx_buf, mipi_sharp_cmds_cabc_lut0_gamma,
                         ARRAY_SIZE(mipi_sharp_cmds_cabc_lut0_gamma));
        break;
    case SHDISP_MAIN_DISP_CABC_LUT1:
        mipi_dsi_cmds_tx(&sharp_tx_buf, mipi_sharp_cmds_cabc_transition_set_image_exrate_lut1,
                         ARRAY_SIZE(mipi_sharp_cmds_cabc_transition_set_image_exrate_lut1));
        mipi_dsi_cmds_tx(&sharp_tx_buf, mipi_sharp_cmds_cabc_lut1_gamma,
                         ARRAY_SIZE(mipi_sharp_cmds_cabc_lut1_gamma));
        break;
    case SHDISP_MAIN_DISP_CABC_LUT2:
        mipi_dsi_cmds_tx(&sharp_tx_buf, mipi_sharp_cmds_cabc_transition_set_image_exrate_lut2,
                         ARRAY_SIZE(mipi_sharp_cmds_cabc_transition_set_image_exrate_lut2));
        mipi_dsi_cmds_tx(&sharp_tx_buf, mipi_sharp_cmds_cabc_lut2_gamma,
                         ARRAY_SIZE(mipi_sharp_cmds_cabc_lut2_gamma));
        break;
    case SHDISP_MAIN_DISP_CABC_LUT3:
        mipi_dsi_cmds_tx(&sharp_tx_buf, mipi_sharp_cmds_cabc_transition_set_image_exrate_lut3,
                         ARRAY_SIZE(mipi_sharp_cmds_cabc_transition_set_image_exrate_lut3));
        mipi_dsi_cmds_tx(&sharp_tx_buf, mipi_sharp_cmds_cabc_lut3_gamma,
                         ARRAY_SIZE(mipi_sharp_cmds_cabc_lut3_gamma));
        break;
    case SHDISP_MAIN_DISP_CABC_LUT4:
        mipi_dsi_cmds_tx(&sharp_tx_buf, mipi_sharp_cmds_cabc_transition_set_image_exrate_lut4,
                         ARRAY_SIZE(mipi_sharp_cmds_cabc_transition_set_image_exrate_lut4));
        mipi_dsi_cmds_tx(&sharp_tx_buf, mipi_sharp_cmds_cabc_lut4_gamma,
                         ARRAY_SIZE(mipi_sharp_cmds_cabc_lut4_gamma));
        break;
    case SHDISP_MAIN_DISP_CABC_LUT5:
        mipi_dsi_cmds_tx(&sharp_tx_buf, mipi_sharp_cmds_cabc_transition_set_image_exrate_lut5,
                         ARRAY_SIZE(mipi_sharp_cmds_cabc_transition_set_image_exrate_lut5));
        mipi_dsi_cmds_tx(&sharp_tx_buf, mipi_sharp_cmds_cabc_lut5_gamma,
                         ARRAY_SIZE(mipi_sharp_cmds_cabc_lut5_gamma));
        break;
	}

    return 0;
}
#endif /* CONFIG_SHDISP_USE_CABC */

static int __devinit mipi_sharp_flute_lcd_probe(struct platform_device *pdev)
{
	struct mipi_dsi_panel_platform_data *platform_data;
	struct msm_fb_panel_data            *pdata;
	struct msm_panel_info               *pinfo;

	static u32 width, height;

	if (pdev->id == 0) {
		platform_data = pdev->dev.platform_data;

		if (platform_data) {
			width  = (platform_data->width_in_mm);
			height = (platform_data->height_in_mm);
		}

		return 0;
	}

	pdata = (struct msm_fb_panel_data *)pdev->dev.platform_data;
	pinfo = &(pdata->panel_info);

	pinfo->width_in_mm  = width;
	pinfo->height_in_mm = height;

	msm_fb_add_device(pdev);

	return 0;
}

static struct platform_driver this_driver = {
	.probe  = mipi_sharp_flute_lcd_probe,
	.driver = {
		.name   = "mipi_sharp_flute",
	},
};

static struct msm_fb_panel_data sharp_flute_panel_data = {
	.on		= mipi_sharp_flute_lcd_on,
	.off	= mipi_sharp_flute_lcd_off,
	.set_backlight = mipi_sharp_flute_lcd_set_backlight,
	.start_display = mipi_sharp_flute_lcd_start_display,
	.stop_display = mipi_sharp_flute_lcd_stop_display,
	.check_display = mipi_sharp_flute_lcd_check_display,

#if defined(CONFIG_SHDISP_USE_CABC)
	.cabc_init			= mipi_sharp_cmd_sqe_cabc_init,
	.cabc_indoor_on		= mipi_sharp_cmd_sqe_cabc_indoor_on,
	.cabc_outdoor_on	= mipi_sharp_cmd_sqe_cabc_outdoor_on,
	.cabc_off			= mipi_sharp_cmd_sqe_cabc_off,
	.cabc_outdoor_move	= mipi_sharp_cmd_sqe_cabc_outdoor_move,
#endif /* CONFIG_SHDISP_USE_CABC */
};

int mipi_sharp_flute_device_register(struct msm_panel_info *pinfo,
					u32 channel, u32 panel)
{
	struct platform_device *pdev = NULL;
	int ret;

	
	if ((channel >= 3) || ch_used[channel])
		return -ENODEV;

	
	ch_used[channel] = TRUE;

	
	pdev = platform_device_alloc("mipi_sharp_flute", (panel << 8)|channel);
	if (!pdev)
		return -ENOMEM;

	sharp_flute_panel_data.panel_info = *pinfo;

	
	ret = platform_device_add_data(pdev, &sharp_flute_panel_data,
		sizeof(sharp_flute_panel_data));
	if (ret) {
		printk(KERN_ERR
		  "%s: platform_device_add_data failed!\n", __func__);
		goto err_device_put;
	}

	ret = platform_device_add(pdev);
	if (ret) {
		printk(KERN_ERR
		  "%s: platform_device_register failed!\n", __func__);
		goto err_device_put;
	}

	return 0;

err_device_put:
	platform_device_put(pdev);
	return ret;
}

int mipi_sharp_flute_diag_write_reg(unsigned char addr, unsigned char *write_data, unsigned char size)
{
    int ret;

    if(size > MIPI_SHARP_RW_MAX_SIZE) {
        pr_err("%s: size over, -EINVAL\n", __func__);
        ret = -EINVAL;
    }
    else {
        ret = mipi_sharp_flute_write_reg(addr, write_data, size);
    }

    return ret;
}

int mipi_sharp_flute_diag_read_reg(unsigned char addr, unsigned char *read_data, unsigned char size)
{
    int ret;

    if(size > MIPI_SHARP_RW_MAX_SIZE) {
        pr_err("%s: size over, -EINVAL\n", __func__);
        ret = -EINVAL;
    }
    else {
        ret = mipi_sharp_flute_read_reg(addr, read_data, size);
    }

    return ret;
}

static int __init mipi_sharp_flute_lcd_init(void)
{
    int ret;
    
    mipi_dsi_buf_alloc(&sharp_tx_buf, DSI_BUF_SIZE);
    mipi_dsi_buf_alloc(&sharp_rx_buf, DSI_BUF_SIZE);
    
    ret = platform_driver_register(&this_driver);
    if( ret < 0 )
    {
        goto mipi_init_err_1;
    }
    return 0;

mipi_init_err_1:
    return -1;
}
module_init(mipi_sharp_flute_lcd_init);

static void mipi_sharp_flute_lcd_exit(void)
{
    platform_driver_unregister(&this_driver);
    return;
}
module_exit(mipi_sharp_flute_lcd_exit);
