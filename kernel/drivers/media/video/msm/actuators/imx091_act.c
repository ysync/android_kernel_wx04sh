/* Copyright (c) 2012, Code Aurora Forum. All rights reserved.
 *
 * This program is free software; you can redistribute it and/or modify
 * it under the terms of the GNU General Public License version 2 and
 * only version 2 as published by the Free Software Foundation.
 *
 * This program is distributed in the hope that it will be useful,
 * but WITHOUT ANY WARRANTY; without even the implied warranty of
 * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
 * GNU General Public License for more details.
 *
 */

#include <linux/module.h>
#include "msm_actuator.h"
#include <linux/debugfs.h>
#include <linux/gpio.h>
#include <mach/gpio.h>
#include <mach/board.h>
#include <mach/gpiomux.h>
#include <sharp/sh_smem.h>

#define IMX091_TOTAL_STEPS_NEAR_TO_FAR_MAX		40

int32_t imx091_actuator_write_focus(
	struct msm_actuator_ctrl_t *a_ctrl,
	uint16_t curr_lens_pos,
	struct damping_params_t *damping_params,
	int8_t sign_direction,
	int16_t code_boundary);

int32_t imx091_actuator_i2c_write(struct msm_actuator_ctrl_t *a_ctrl,
	int16_t next_lens_position, uint32_t hw_params);

int32_t imx091_actuator_init_step_table(struct msm_actuator_ctrl_t *a_ctrl,
	struct msm_actuator_set_info_t *set_info);

int32_t imx091_actuator_set_ois_init(void);
int32_t imx091_actuator_set_ois_still(void);
int32_t imx091_actuator_set_ois_video(void);
int32_t imx091_actuator_set_ois_off(void);

static int16_t virtual_vcm_dreg = 0;

static struct msm_actuator_ctrl_t imx091_act_t;

static struct msm_actuator imx091_vcm_actuator_table = {
	.act_type = ACTUATOR_VCM,
	.func_tbl = {
		.actuator_init_step_table  = imx091_actuator_init_step_table,
		.actuator_move_focus = msm_actuator_move_focus,
		.actuator_write_focus = imx091_actuator_write_focus,
		.actuator_set_default_focus = msm_actuator_set_default_focus,
		.actuator_init_focus = msm_actuator_init_focus,
		.actuator_i2c_write = imx091_actuator_i2c_write,
	},
};

static struct msm_actuator *actuators[] = {
	&imx091_vcm_actuator_table,
};

#define OIS_STATE_POWEROFF       0
#define OIS_STATE_OFF            1
#define OIS_STATE_STILL          2
#define OIS_STATE_VIDEO          3

static struct{
	uint8_t ois_adjustflg;
	uint8_t ois_state;
	uint16_t curdat;
	uint16_t halofs;
	uint16_t pstxof;
	uint16_t pstyof;
	int16_t hx_ofs;
	int16_t hy_ofs;
	int16_t gx_ofs;
	int16_t gy_ofs;
	int16_t kgxhg;
	int16_t kgyhg;
	uint16_t kgxg;
	uint16_t kgyg;
}imx091_ois_parameter;

#define IMX091_ACT_OIS_CMD_PER	0x82
#define IMX091_ACT_OIS_CMD_MEM	0x84
#define IMX091_ACT_OIS_CMD_SPE	0x8C

struct imx091_act_ois_reg_conf {
	uint8_t type;
	uint8_t addr;
	uint16_t data;
};

static struct imx091_act_ois_reg_conf ois_init_setting1[] = {
	{IMX091_ACT_OIS_CMD_SPE, 0x01, 0x0000},
	{IMX091_ACT_OIS_CMD_PER, 0x05, 0x0005},
	{IMX091_ACT_OIS_CMD_MEM, 0xF3, 0x7FFF},
};

static struct imx091_act_ois_reg_conf ois_init_setting2[] = {
	{IMX091_ACT_OIS_CMD_PER, 0x57, 0x0C00},
	{IMX091_ACT_OIS_CMD_PER, 0x22, 0x0080},
	{IMX091_ACT_OIS_CMD_PER, 0x5A, 0x0001},
	{IMX091_ACT_OIS_CMD_PER, 0x3D, 0xC08C},
	{IMX091_ACT_OIS_CMD_MEM, 0xF3, 0x6666},
};

static struct imx091_act_ois_reg_conf ois_init_setting3[] = {
	{IMX091_ACT_OIS_CMD_MEM, 0x48, 0x0484},
	{IMX091_ACT_OIS_CMD_MEM, 0x49, 0x6AD0},
	{IMX091_ACT_OIS_CMD_MEM, 0x4A, 0x9852},
	{IMX091_ACT_OIS_CMD_MEM, 0x4B, 0x7FFF},
	{IMX091_ACT_OIS_CMD_MEM, 0x4C, 0x7FFF},
	{IMX091_ACT_OIS_CMD_MEM, 0xC8, 0x0484},
	{IMX091_ACT_OIS_CMD_MEM, 0xC9, 0x6AD0},
	{IMX091_ACT_OIS_CMD_MEM, 0xCA, 0x9852},
	{IMX091_ACT_OIS_CMD_MEM, 0xCB, 0x7FFF},
	{IMX091_ACT_OIS_CMD_MEM, 0xCC, 0x7FFF},
};

static struct imx091_act_ois_reg_conf ois_off_setting[] = {
	{IMX091_ACT_OIS_CMD_MEM, 0x7F, 0x8000},
	{IMX091_ACT_OIS_CMD_PER, 0x30, 0x0000},
};

static struct imx091_act_ois_reg_conf ois_still_setting[] = {
	{IMX091_ACT_OIS_CMD_MEM, 0x7F, 0x0C0C},
	{IMX091_ACT_OIS_CMD_MEM, 0x10, 0x2000},
	{IMX091_ACT_OIS_CMD_MEM, 0x11, 0x7800},
	{IMX091_ACT_OIS_CMD_MEM, 0x12, 0x1000},
	{IMX091_ACT_OIS_CMD_MEM, 0x13, 0x1000},
	{IMX091_ACT_OIS_CMD_MEM, 0x18, 0x2000},
	{IMX091_ACT_OIS_CMD_MEM, 0x19, 0x7800},
	{IMX091_ACT_OIS_CMD_MEM, 0x1A, 0x1000},
	{IMX091_ACT_OIS_CMD_MEM, 0x1B, 0x1000},
	{IMX091_ACT_OIS_CMD_MEM, 0x36, 0x7FC0},
	{IMX091_ACT_OIS_CMD_MEM, 0x40, 0x1B80},
	{IMX091_ACT_OIS_CMD_MEM, 0x43, 0x3E80},
	{IMX091_ACT_OIS_CMD_MEM, 0x90, 0x2000},
	{IMX091_ACT_OIS_CMD_MEM, 0x91, 0x7800},
	{IMX091_ACT_OIS_CMD_MEM, 0x92, 0x1000},
	{IMX091_ACT_OIS_CMD_MEM, 0x93, 0x1000},
	{IMX091_ACT_OIS_CMD_MEM, 0x98, 0x2000},
	{IMX091_ACT_OIS_CMD_MEM, 0x99, 0x7800},
	{IMX091_ACT_OIS_CMD_MEM, 0x9A, 0x1000},
	{IMX091_ACT_OIS_CMD_MEM, 0x9B, 0x1000},
	{IMX091_ACT_OIS_CMD_MEM, 0xB6, 0x7FC0},
	{IMX091_ACT_OIS_CMD_MEM, 0xC0, 0x1B80},
	{IMX091_ACT_OIS_CMD_MEM, 0xC3, 0x3E80},
	{IMX091_ACT_OIS_CMD_MEM, 0x14, 0x0000},
	{IMX091_ACT_OIS_CMD_MEM, 0x1C, 0x0000},
	{IMX091_ACT_OIS_CMD_MEM, 0x3E, 0x7FFF},
	{IMX091_ACT_OIS_CMD_MEM, 0x3C, 0x0000},
	{IMX091_ACT_OIS_CMD_MEM, 0x94, 0x0000},
	{IMX091_ACT_OIS_CMD_MEM, 0x9C, 0x0000},
	{IMX091_ACT_OIS_CMD_MEM, 0xBE, 0x7FFF},
	{IMX091_ACT_OIS_CMD_MEM, 0xBC, 0x0000},
	{IMX091_ACT_OIS_CMD_MEM, 0x7F, 0x0D0D},
};

static struct imx091_act_ois_reg_conf ois_video_setting[] = {
	{IMX091_ACT_OIS_CMD_MEM, 0x7F, 0x0C0C},
	{IMX091_ACT_OIS_CMD_MEM, 0x10, 0x4000},
	{IMX091_ACT_OIS_CMD_MEM, 0x11, 0x7000},
	{IMX091_ACT_OIS_CMD_MEM, 0x12, 0x1000},
	{IMX091_ACT_OIS_CMD_MEM, 0x13, 0x1000},
	{IMX091_ACT_OIS_CMD_MEM, 0x18, 0x4000},
	{IMX091_ACT_OIS_CMD_MEM, 0x19, 0x7000},
	{IMX091_ACT_OIS_CMD_MEM, 0x1A, 0x1000},
	{IMX091_ACT_OIS_CMD_MEM, 0x1B, 0x1000},
	{IMX091_ACT_OIS_CMD_MEM, 0x36, 0x7F40},
	{IMX091_ACT_OIS_CMD_MEM, 0x40, 0x2800},
	{IMX091_ACT_OIS_CMD_MEM, 0x43, 0x6D60},
	{IMX091_ACT_OIS_CMD_MEM, 0x90, 0x4000},
	{IMX091_ACT_OIS_CMD_MEM, 0x91, 0x7000},
	{IMX091_ACT_OIS_CMD_MEM, 0x92, 0x1000},
	{IMX091_ACT_OIS_CMD_MEM, 0x93, 0x1000},
	{IMX091_ACT_OIS_CMD_MEM, 0x98, 0x4000},
	{IMX091_ACT_OIS_CMD_MEM, 0x99, 0x7000},
	{IMX091_ACT_OIS_CMD_MEM, 0x9A, 0x1000},
	{IMX091_ACT_OIS_CMD_MEM, 0x9B, 0x1000},
	{IMX091_ACT_OIS_CMD_MEM, 0xB6, 0x7F40},
	{IMX091_ACT_OIS_CMD_MEM, 0xC0, 0x2800},
	{IMX091_ACT_OIS_CMD_MEM, 0xC3, 0x6D60},
	{IMX091_ACT_OIS_CMD_MEM, 0x14, 0x0000},
	{IMX091_ACT_OIS_CMD_MEM, 0x1C, 0x0000},
	{IMX091_ACT_OIS_CMD_MEM, 0x3E, 0x7FFF},
	{IMX091_ACT_OIS_CMD_MEM, 0x3C, 0x0000},
	{IMX091_ACT_OIS_CMD_MEM, 0x94, 0x0000},
	{IMX091_ACT_OIS_CMD_MEM, 0x9C, 0x0000},
	{IMX091_ACT_OIS_CMD_MEM, 0xBE, 0x7FFF},
	{IMX091_ACT_OIS_CMD_MEM, 0xBC, 0x0000},
	{IMX091_ACT_OIS_CMD_MEM, 0x7F, 0x0D0D},
};

static uint8_t imx091_act_init_fw_settings1[] = {
	0x80,0x00,0xDE,0x61,0x00,0x03,0x90,0x00,0x00,0x03,0x00,0x90,0x00,0x84,0x00,0x00,
	0x01,0x10,0x7E,0x84,0x50,0x00,0x08,0x40,0xFF,0x90,0x40,0x7E,0xA0,0x70,0x06,0x07,
	0x10,0x00,0x08,0x00,0x01,0x11,0x80,0x00,0xA0,0x04,0xFF,0x84,0x00,0x03,0x00,0x3F,
	0xEE,0x07,0x10,0xFD,0x84,0x10,0x00,0x08,0x40,0xFD,0xA0,0x10,0xFE,0x84,0x40,0x00,
	0x08,0x40,0xFE,0xA0,0x10,0xFC,0x84,0x30,0x00,0x08,0x00,0x02,0x07,0x60,0x20,0x08,
	0x30,0x00,0x08,0x10,0xFF,0x84,0x60,0x05,0x07,0x40,0xFC,0xA0,0x00,0x01,0x11,0x10,
	0x00,0x08,0x40,0xFE,0x90,0x40,0xFF,0xA0,0x00,0x03,0x00,0x40,0x20,0x08,0x00,0x04,
	0x11,0x70,0x20,0x08,0x80,0x00,0x11,0x20,0x0A,0x08,0x42,0x00,0xA0,0x20,0x0F,0x08,
	0x00,0x03,0x00,0x00,0x14,0x43,0x90,0x05,0x84,0x20,0x8F,0x08,0x80,0x02,0x90,0x80,
	0x04,0xA0,0x20,0x0F,0x08,0x80,0x03,0x90,0x90,0x04,0x84,0x00,0x20,0x08,0x40,0xFF,
	0x90,0x00,0x20,0x08,0x40,0xFE,0x90,0xFF,0xFF,0x01,0x8F,0x02,0x07,0x00,0x01,0x01,
	0x30,0x04,0x07,0x00,0x00,0x01,0x10,0x10,0x08,0x10,0xFE,0x84,0x20,0x0F,0x08,0x80,
	0x01,0x90,0x80,0x04,0xA0,0x10,0xFF,0x84,0x20,0x0F,0x08,0x80,0x00,0x90,0x00,0x03,
	0x00,0x00,0x14,0x43,0x90,0x17,0x84,0x20,0x0F,0x08,0x80,0x47,0xA0,0x00,0x00,0x11,
	0x30,0x02,0x07,0x80,0x41,0x90,0x50,0x00,0x08,0x40,0xFC,0x90,0x88,0x2F,0x84,0x00,
	0x00,0x11,0x30,0x02,0x07,0x40,0xFF,0x90,0x50,0x00,0x08,0x40,0xFD,0x90,0x40,0x7F,
	0xA0,0x10,0xFF,0x84,0x20,0x2C,0x08,0x7F,0xFF,0x11,0x20,0x0D,0x08,0x80,0x0F,0x90,
	0x80,0x26,0xA0,0x90,0x2E,0x84,0x00,0x20,0x08,0x80,0x1F,0x90,0x90,0x26,0x84,0x20,
	0x8F,0x08,0x7F,0xFF,0x11,0x80,0x1F,0xA0,0x20,0x0F,0x08,0x80,0x0E,0x90,0x00,0x00,
	0x21,0x30,0x02,0x07,0x20,0x09,0x60,0x50,0x00,0x08,0x40,0xFE,0x90,0x40,0x7F,0xA0,
	0x90,0x1F,0x84,0x10,0x20,0x08,0x80,0x17,0x90,0x90,0x3F,0x84,0x20,0x1E,0x08,0x80,
	0x46,0xA0,0x10,0x00,0x08,0x80,0x1E,0x90,0x40,0xFF,0xA0,0x10,0xFC,0x84,0x08,0xFD,
	0x84,0x04,0xFE,0x84,0x00,0x03,0x00,0x00,0x14,0x43,0x20,0x0D,0x08,0x40,0xFF,0x90,
	0x20,0x00,0x21,0x30,0x02,0x07,0x40,0xEE,0xA0,0x50,0x00,0x08,0x40,0xFE,0x90,0x40,
	0x7F,0xA0,0x10,0xFF,0x84,0x00,0x00,0x08,0x80,0x37,0x90,0x00,0x00,0x21,0x30,0x02,
	0x07,0x80,0x4F,0xA0,0x50,0x00,0x08,0x40,0x7F,0xA0,0x04,0xFE,0x84,0x00,0x03,0x00,
	0x00,0x01,0x00,0x04,0x41,0x44,0x04,0x28,0x44,0x00,0x00,0x21,0x00,0x04,0x00,0x10,
	0x42,0x44,0x00,0x43,0x43,0x00,0x00,0x08,0x40,0xFF,0x90,0x10,0x78,0x84,0x50,0x00,
};

static uint8_t imx091_act_init_fw_settings2[] = {
	0x80,
	0x08,0x10,0x00,0x20,0x00,0x10,0x08,0x40,0x78,0xA0,0x50,0x00,0x08,0x00,0xFF,0x11,
	0x10,0xFF,0x84,0x50,0x00,0x08,0x7F,0x00,0x11,0x10,0x00,0x20,0x20,0x0F,0x08,0x2A,
	0xAA,0x11,0x40,0x61,0xA0,0x00,0x00,0x00,0x10,0xF0,0x44,0x50,0x00,0x08,0x00,0x7F,
	0x11,0x20,0xF0,0x60,0x01,0x43,0x43,0x00,0x42,0x42,0x04,0x5C,0x84,0x04,0x61,0x84,
	0x04,0x57,0x84,0x00,0x00,0x21,0x04,0x74,0x84,0x00,0x40,0x21,0x00,0x0D,0x07,0x05,
	0x41,0x43,0x04,0x74,0x84,0x00,0x00,0x21,0x10,0x57,0x84,0x30,0x20,0x08,0x00,0x04,
	0x11,0x50,0x00,0x08,0x03,0xFF,0x11,0x20,0xFA,0x60,0x10,0xF0,0x44,0x50,0x20,0x08,
	0x00,0x7F,0x11,0x60,0x20,0x08,0x40,0x00,0x08,0x00,0x08,0x11,0x30,0x1D,0x07,0x50,
	0x00,0x08,0x07,0x00,0x11,0x9F,0x20,0x07,0x8B,0x21,0x07,0x9C,0x16,0x07,0x20,0xF8,
	0x60,0x87,0x18,0x07,0x40,0xF7,0xA0,0x81,0x26,0x07,0x20,0x48,0x60,0x10,0x0A,0x44,
	0x10,0xF4,0x84,0x3B,0xEA,0x00,0xE0,0x14,0x43,0x40,0xAE,0xA0,0x00,0x02,0x07,0x40,
	0x2E,0xA0,0x00,0x04,0x07,0x40,0x7C,0xA0,0x89,0x05,0x07,0x81,0x04,0x07,0x40,0x7F,
	0xA0,0x00,0x00,0x00,0x10,0x7E,0x84,0x00,0x00,0x08,0x40,0x00,0x21,0x8F,0x04,0x07,
	0x40,0x7F,0xA0,0x30,0x06,0x07,0x70,0x10,0x08,0x80,0x00,0x21,0x00,0x07,0x07,0x40,
	0x00,0x21,0x9F,0x0B,0x07,0x40,0x7F,0xA0,0x30,0x05,0x07,0x50,0x00,0x08,0xC0,0x00,
	0x21,0x40,0x7E,0x90,0x00,0x00,0x00,0x10,0xCF,0x84,0x20,0x1B,0x08,0x7F,0xFF,0x21,
	0x3C,0x09,0x00,0xC8,0x14,0x43,0x10,0x00,0x20,0x00,0x00,0x08,0x40,0xB5,0x90,0x40,
	0xAD,0xA0,0x00,0x0A,0x07,0x10,0x4F,0x84,0x20,0x1B,0x08,0x7F,0xFF,0x21,0x3C,0x13,
	0x00,0x48,0x14,0x43,0x10,0x00,0x20,0x00,0x00,0x08,0x40,0x35,0x90,0x40,0x2D,0xA0,
	0x82,0x14,0x07,0x81,0x0C,0x07,0x80,0x16,0x07,0x04,0x50,0x84,0x10,0x00,0x20,0x20,
	0x02,0x07,0x00,0x08,0x21,0x40,0x00,0x08,0x00,0x01,0x11,0x40,0x50,0xA0,0x00,0x04,
	0x00,0x3C,0x16,0x00,0x10,0x00,0x20,0x20,0x0D,0x08,0x7F,0xFF,0x11,0x20,0x37,0x60,
	0x68,0x14,0x43,0x20,0x96,0x00,0x80,0x14,0x43,0x01,0x00,0x01,0x04,0x00,0x11,0x02,
	0x00,0x21,0xFF,0x36,0xC5,0x20,0x9C,0x00,0x00,0x14,0x43,0x00,0x01,0x01,0x00,0x04,
	0x11,0x00,0x02,0x21,0xFF,0x34,0xC5,0x00,0x04,0x00,0x04,0x1C,0x44,0x04,0x1B,0x44,
	0x00,0x01,0x07,0x4C,0x00,0x21,0x04,0x7B,0x84,0x40,0x7A,0xA0,0x10,0x7E,0x84,0x50,
	0x00,0x08,0x3F,0xFF,0x21,0x00,0x04,0x00,0x04,0xBE,0x84,0x04,0x3E,0x84,0x7F,0xFF,
	0x21,0x04,0xBC,0x84,0x04,0x9C,0x84,0x04,0x94,0x84,0x04,0x8C,0x84,0x04,0x84,0x84,
};

static uint8_t imx091_act_init_fw_settings3[] = {
	0x80,
	0x04,0x87,0x84,0x04,0x3C,0x84,0x04,0x1C,0x84,0x04,0x14,0x84,0x04,0x0C,0x84,0x04,
	0x04,0x84,0x04,0x07,0x84,0x00,0x00,0x21,0x30,0x12,0x07,0x70,0x10,0x08,0x80,0x00,
	0x21,0x50,0x00,0x08,0xC0,0x00,0x21,0x00,0x1E,0x07,0x4C,0x02,0x21,0x10,0x7E,0x84,
	0x60,0x10,0x08,0x80,0x00,0x21,0x50,0x00,0x08,0x3F,0xFF,0x21,0x40,0x7E,0x90,0x30,
	0x09,0x07,0x70,0x10,0x08,0x40,0x00,0x21,0x00,0x29,0x07,0xFF,0xFF,0x21,0x10,0x42,
	0x84,0x20,0x1F,0x08,0x20,0x00,0x21,0x20,0x1F,0x08,0x40,0x63,0xA0,0x10,0x16,0x84,
	0x10,0x20,0x08,0x40,0x1D,0x90,0x10,0x07,0x84,0x20,0x2C,0x08,0x7F,0xFF,0x11,0x10,
	0x20,0x08,0x40,0x06,0x90,0x10,0x55,0x84,0x70,0x00,0x08,0x20,0x1B,0x60,0x80,0x00,
	0x11,0x10,0xC2,0x84,0x20,0x1F,0x08,0x20,0x00,0x21,0x20,0x1F,0x08,0x40,0x66,0xA0,
	0x10,0x96,0x84,0x10,0x20,0x08,0x40,0x9D,0x90,0x10,0x87,0x84,0x20,0x2C,0x08,0x7F,
	0xFF,0x11,0x10,0x20,0x08,0x40,0x86,0x90,0x10,0x56,0x84,0x70,0x00,0x08,0x20,0x1C,
	0x60,0x80,0x00,0x11,0x00,0x23,0x07,0x10,0x7B,0x84,0x10,0x00,0x08,0x00,0x01,0x11,
	0x20,0x05,0x07,0x60,0x00,0x08,0x00,0x00,0x11,0x40,0x7B,0xA0,0x30,0x2D,0x07,0x50,
	0x00,0x08,0xC0,0x00,0x21,0x40,0x7E,0x90,0x00,0x00,0x00,0x04,0x6F,0x84,0x01,0x00,
	0x21,0x10,0x7E,0x84,0x60,0x00,0x08,0x04,0x00,0x11,0x40,0x7E,0xA0,0x10,0xC1,0x84,
	0x20,0x0F,0x08,0x70,0x09,0x07,0x10,0x10,0x08,0x40,0xC0,0xA0,0x20,0x0F,0x08,0x7F,
	0xFF,0x11,0x8F,0x02,0x07,0x80,0x00,0x11,0x04,0xC1,0x84,0x40,0xBD,0xA0,0x04,0x6E,
	0x84,0x01,0x00,0x21,0x10,0x7E,0x84,0x60,0x00,0x08,0x00,0x04,0x11,0x40,0x7E,0xA0,
	0x10,0x41,0x84,0x20,0x0F,0x08,0x70,0x09,0x07,0x10,0x10,0x08,0x40,0x40,0xA0,0x20,
	0x0F,0x08,0x7F,0xFF,0x11,0x8F,0x02,0x07,0x80,0x00,0x11,0x04,0x41,0x84,0x40,0x3D,
	0xA0,0x10,0x2D,0x44,0x21,0x0E,0x00,0x08,0x00,0x11,0x10,0x00,0x21,0x80,0x14,0x43,
	0x10,0x2C,0x44,0x21,0x13,0x00,0x00,0x08,0x11,0x00,0x10,0x21,0x00,0x14,0x43,0x00,
	0x04,0x00,0x10,0xF5,0x84,0x00,0x0C,0x01,0x01,0x43,0x43,0x00,0x42,0x42,0x04,0xF7,
	0x84,0x01,0x84,0x21,0x30,0x06,0x07,0x10,0x00,0x08,0x00,0x01,0x11,0x40,0xF5,0xA0,
	0x60,0x09,0x07,0x10,0x00,0x08,0x40,0xF3,0x90,0x40,0x6D,0xA0,0x20,0x0D,0x07,0x70,
	0x00,0x08,0x01,0x84,0x11,0x40,0xF7,0xA0,0x3C,0xC3,0x00,0x10,0x00,0x20,0xA8,0x14,
	0x43,0x04,0xA7,0x84,0x20,0x8E,0x08,0x40,0xC4,0x90,0x40,0x73,0xA0,0x20,0x0F,0x08,
	0x40,0xC5,0x90,0x40,0xA7,0xA0,0x3C,0xCD,0x00,0x10,0x00,0x20,0x28,0x14,0x43,0x04,
};

static uint8_t imx091_act_init_fw_settings4[] = {
	0x80,
	0x27,0x84,0x20,0x8E,0x08,0x40,0x44,0x90,0x40,0x71,0xA0,0x20,0x0F,0x08,0x40,0x45,
	0x90,0x40,0x27,0xA0,0x04,0x73,0x84,0x40,0xAF,0xA0,0x04,0x71,0x84,0x40,0x2F,0xA0,
	0x30,0x05,0x07,0x60,0x00,0x08,0x40,0x51,0x90,0x00,0x00,0x21,0x3D,0x00,0x00,0x60,
	0x00,0x08,0x72,0x14,0x43,0x40,0xAF,0x90,0x00,0x00,0x21,0x3D,0x05,0x00,0x60,0x00,
	0x08,0x70,0x14,0x43,0x40,0x2F,0x90,0x00,0x00,0x21,0x00,0x04,0x00,0x10,0xF0,0x44,
	0x50,0x00,0x08,0xFF,0xF7,0x11,0x93,0x04,0x07,0x20,0xF0,0x60,0x04,0x61,0x84,0x04,
	0x5C,0x84,0x40,0x57,0xA0,0x00,0x04,0x00,0x10,0x61,0x84,0x00,0x20,0x08,0x20,0x1F,
	0x08,0x40,0xFF,0xA0,0x10,0x00,0x08,0x40,0x5C,0x90,0x40,0x57,0xA0,0x04,0x74,0x84,
	0x10,0x00,0x20,0x00,0x00,0x08,0x40,0x74,0xA0,0x00,0x01,0x11,0x20,0x05,0x07,0x00,
	0x40,0x21,0x70,0x00,0x08,0x7F,0xFF,0x21,0x08,0xFF,0x84,0x42,0x00,0x90,0x85,0x02,
	0x07,0x20,0xF0,0x60,0x7F,0xFF,0x11,0x60,0x20,0x08,0x00,0xD0,0x11,0x40,0x00,0x08,
	0x00,0x02,0x11,0x86,0x1B,0x07,0x40,0x74,0xA0,0x08,0x66,0x84,0x7F,0xFF,0x11,0x8A,
	0x02,0x07,0x89,0x03,0x07,0x40,0x7E,0xA0,0x00,0x00,0x11,0x08,0x6F,0x84,0x00,0x00,
	0x11,0x04,0xF1,0x84,0x00,0x40,0x21,0x10,0x7E,0x84,0x60,0x00,0x08,0x40,0x7E,0xA0,
	0x02,0x00,0x11,0x60,0x09,0x07,0x10,0x20,0x08,0x40,0xC3,0x90,0x20,0x2F,0x08,0x80,
	0x00,0x11,0x70,0x02,0x07,0x7F,0xFF,0x11,0x10,0x00,0x08,0x40,0x9D,0x90,0x40,0x85,
	0xA0,0x08,0x63,0x84,0x7F,0xFF,0x11,0x82,0x02,0x07,0x81,0x03,0x07,0x40,0x7E,0xA0,
	0x00,0x00,0x11,0x08,0x6E,0x84,0x00,0x00,0x11,0x04,0xF0,0x84,0x00,0x40,0x21,0x10,
	0x7E,0x84,0x60,0x00,0x08,0x40,0x7E,0xA0,0x00,0x02,0x11,0x60,0x09,0x07,0x10,0x20,
	0x08,0x40,0x43,0x90,0x20,0x2F,0x08,0x80,0x00,0x11,0x70,0x02,0x07,0x7F,0xFF,0x11,
	0x10,0x00,0x08,0x40,0x1D,0x90,0x40,0x05,0xA0,0x00,0x04,0x00,0x22,0x21,0x00,0xFB,
	0xFF,0x21,0x6F,0x14,0x43,0x22,0x24,0x00,0xFF,0xFB,0x21,0x6E,0x14,0x43,0x22,0x27,
	0x00,0xFF,0xFF,0x21,0x51,0x14,0x43,0x3D,0x48,0x00,0x40,0x73,0xA0,0xA0,0x14,0x43,
	0x3D,0x4B,0x00,0x40,0x71,0xA0,0x20,0x14,0x43,0x00,0x04,0x00,0x3D,0x4F,0x00,0x40,
	0xA5,0xA0,0xB0,0x14,0x43,0x3D,0x52,0x00,0x40,0x25,0xA0,0x30,0x14,0x43,0x22,0x01,
	0x00,0xB8,0x14,0x43,0x40,0xC2,0xA0,0x22,0x04,0x00,0x38,0x14,0x43,0x40,0x42,0xA0,
	0x00,0x04,0x00,0x22,0x3E,0x00,0xFD,0xFF,0x21,0xF1,0x14,0x43,0x22,0x41,0x00,0xFF,
	0xFD,0x21,0xF0,0x14,0x43,0x22,0x0E,0x00,0x40,0x87,0xA0,0x80,0x14,0x43,0x22,0x11,
	0x00,0x40,0x07,0xA0,0x00,0x14,0x43,0x00,0x04,0x00,0x10,0xB9,0x84,0x20,0x0F,0x08,
};

static uint8_t imx091_act_init_fw_settings5[] = {
	0x80,
	0x40,0xBE,0xA0,0x40,0xB6,0x90,0x08,0xBE,0x84,0x70,0x02,0x07,0x10,0x20,0x08,0x7F,
	0xC0,0x11,0x10,0xBE,0x84,0x00,0x00,0x08,0x40,0x53,0xA0,0x7F,0xC0,0x11,0x04,0x51,
	0x84,0x00,0x1F,0x21,0x10,0xBC,0x84,0x20,0x0F,0x08,0x70,0x00,0x11,0x40,0xBC,0xA0,
	0x10,0x9C,0x84,0x10,0x94,0x84,0x20,0x0D,0x08,0x7F,0xFF,0x21,0x40,0x8D,0x90,0x99,
	0x0D,0x07,0x40,0x7E,0xA0,0x40,0xBE,0x90,0x10,0x9C,0x84,0x10,0x94,0x84,0x20,0x0D,
	0x08,0x7F,0xFF,0x21,0x40,0x8D,0x90,0x08,0x53,0x84,0xFF,0xFF,0x11,0x9A,0x0A,0x07,
	0x40,0xBE,0x90,0x08,0x53,0x84,0x00,0x01,0x11,0x40,0x7E,0xA0,0x10,0x39,0x84,0x20,
	0x0F,0x08,0x40,0x3E,0xA0,0x40,0x36,0x90,0x08,0x3E,0x84,0x70,0x02,0x07,0x10,0x20,
	0x08,0x7F,0xC0,0x11,0x10,0x3E,0x84,0x00,0x00,0x08,0x40,0x52,0xA0,0x7F,0xC0,0x11,
	0x04,0x51,0x84,0x00,0x1F,0x21,0x10,0x3C,0x84,0x20,0x0F,0x08,0x70,0x00,0x11,0x40,
	0x3C,0xA0,0x10,0x1C,0x84,0x10,0x14,0x84,0x20,0x0D,0x08,0x7F,0xFF,0x21,0x40,0x0D,
	0x90,0x91,0x0D,0x07,0x40,0x7E,0xA0,0x40,0x3E,0x90,0x10,0x1C,0x84,0x10,0x14,0x84,
	0x20,0x0D,0x08,0x7F,0xFF,0x21,0x40,0x0D,0x90,0x08,0x52,0x84,0xFF,0xFF,0x11,0x92,
	0x0A,0x07,0x40,0x3E,0x90,0x08,0x52,0x84,0x00,0x01,0x11,0x40,0x7E,0xA0,0x00,0x04,
	0x00,0x10,0xEE,0x84,0x00,0x20,0x08,0x20,0x1F,0x08,0x40,0xEF,0xA0,0x10,0x10,0x08,
	0x40,0xE7,0xA0,0x20,0x0D,0x08,0x40,0xED,0xA0,0x40,0xEE,0x90,0x3D,0xBF,0x00,0x20,
	0x37,0x60,0xE8,0x14,0x43,0x22,0x6E,0x00,0x40,0x85,0xA0,0x88,0x14,0x43,0x22,0x71,
	0x00,0x40,0x05,0xA0,0x08,0x14,0x43,0x00,0x04,0x00,0x22,0x75,0x00,0x40,0x8D,0xA0,
	0x90,0x14,0x43,0x22,0x78,0x00,0x40,0x0D,0xA0,0x10,0x14,0x43,0x00,0x04,0x00,0x22,
	0x7C,0x00,0x40,0x95,0xA0,0x98,0x14,0x43,0x22,0x7F,0x00,0x40,0x15,0xA0,0x18,0x14,
	0x43,0x00,0x04,0x00,0x00,0x01,0x07,0x00,0x02,0x07,0x00,0x03,0x07,0x00,0x05,0x07,
	0x00,0x0D,0x07,0x00,0x15,0x07,0x00,0x29,0x07,0x00,0x77,0x07,0x00,0x85,0x07,0x00,
	0x93,0x07,0x00,0xA4,0x07,0x00,0xFA,0x07,0x01,0x34,0x07,0x01,0xBD,0x07,0x01,0xD1,
	0x07,0x00,0x10,0x07,0x1F,0xFF,0x07,0x20,0x03,0x60,0x0F,0x72,0x07,0x08,0xFA,0x84,
	0x04,0x11,0x44,0xFF,0x00,0x11,0x2F,0x76,0x07,0x70,0x10,0x08,0x00,0xF0,0x21,0x50,
	0x00,0x08,0x00,0xF0,0x21,0x40,0xF9,0x90,0x00,0x00,0x00,0x0D,0xAE,0x07,0x22,0x57,
	0x00,0x04,0xFA,0x84,0x48,0x00,0x21,0x04,0x11,0x44,0x00,0x0E,0x07,0x3E,0x88,0x00,
	0x0F,0x84,0x07,0x04,0x51,0x84,0x00,0x1F,0x21,0x3F,0x87,0x07,0x70,0x20,0x08,0x00,
	0x1E,0x11,0x50,0x00,0x08,0x20,0x05,0x07,0x70,0x00,0x08,0x00,0x7F,0x11,0x40,0xF8,
};

static uint8_t imx091_act_init_fw_settings6[] = {
	0x80,
	0xA0,0x08,0xFA,0x84,0x04,0x11,0x44,0x3E,0x6E,0x00,0x00,0x1E,0x07,0x00,0x1F,0x07,
	0x48,0x80,0x11,0x08,0x09,0x44,0x40,0x75,0x90,0x7F,0xEF,0x07,0x10,0x00,0x20,0x00,
	0x10,0x08,0xFF,0xFF,0x21,0x06,0x00,0x84,0x40,0xFB,0xA0,0x60,0x00,0x08,0x00,0x00,
	0x21,0x40,0xFF,0x90,0x10,0xFB,0x84,0x60,0x20,0x08,0x40,0xFB,0x90,0x30,0x00,0x08,
	0x00,0x08,0x11,0x3E,0x05,0x00,0x04,0xFB,0x84,0x3E,0x07,0x00,0x04,0xFF,0x84,0x40,
	0xF8,0xA0,0x04,0x11,0x44,0x8F,0xFF,0x07,0x20,0x19,0x60,0x22,0x86,0x00,0x0F,0xF0,
	0x07,0x00,0x00,0x00,0x00,0x05,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x3F,0x07,
	0x48,0xA0,0x11,0x40,0xF8,0xA0,0x3E,0x13,0x00,0x04,0x11,0x44,0x40,0xF8,0xA0,0x3E,
	0x16,0x00,0x04,0x11,0x44,0x40,0xF9,0xA0,0x3E,0x19,0x00,0x08,0xF8,0x84,0x04,0xF9,
	0x84,0x00,0x02,0x00,0x00,0x00,0x00,0x00,0x05,0x00,0x00,0x00,0x00,0x00,0x00,0x00,
	0x00,0x01,0x00,0x90,0x13,0x07,0x20,0x48,0x60,0x60,0x10,0x08,0x40,0xF9,0xA0,0x30,
	0x00,0x08,0x00,0x08,0x11,0x40,0xF8,0xA0,0x04,0x11,0x44,0x04,0xF9,0x84,0x3E,0x2B,
	0x00,0x04,0x11,0x44,0x00,0x5C,0x07,0x00,0x01,0x00,0x00,0x5E,0x07,0x00,0x02,0x00,
	0x04,0x28,0x44,0x00,0x99,0x21,0x10,0x00,0x44,0x60,0x00,0x08,0x40,0x54,0x90,0x20,
	0x00,0x60,0x00,0x66,0x07,0x81,0x0A,0x07,0x80,0x03,0x07,0x40,0xF8,0xA0,0x48,0xC0,
	0x11,0x00,0x63,0x07,0x00,0x64,0x07,0x00,0x65,0x07,0x00,0x04,0x07,0x00,0x67,0x07,
	0x00,0x15,0x07,0x00,0x69,0x07,0x00,0x39,0x07,0x00,0x6B,0x07,0x00,0x56,0x07,0x00,
	0x6D,0x07,0x00,0x59,0x07,0x00,0x6F,0x07,0x00,0x69,0x07,0x00,0x71,0x07,0x00,0x6D,
	0x07,0x0F,0xEF,0x07,0x1F,0xFF,0x07,0xFF,0x00,0x11,0x10,0x00,0x20,0x30,0x10,0x08,
	0x00,0x01,0x21,0x50,0x00,0x08,0x00,0x0F,0x21,0x40,0xF9,0x90,0x30,0x7C,0x07,0x70,
	0x10,0x08,0x00,0x80,0x21,0x50,0x00,0x08,0x00,0xF0,0x21,0x08,0xF9,0x84,0x04,0xF8,
	0x84,0x2F,0xFF,0x07,0x3E,0x58,0x00,0x04,0xF3,0x84,0x7F,0xFF,0x21,0x04,0xF5,0x84,
	0x00,0x0C,0x21,0x04,0x78,0x84,0x04,0xF1,0x84,0x04,0xF0,0x84,0x04,0x5A,0x84,0x00,
	0x00,0x21,0x04,0xF6,0x84,0x06,0xD0,0x21,0x04,0xFA,0x84,0x40,0x00,0x21,0x04,0xF7,
	0x84,0x01,0x04,0x21,0x00,0x42,0x42,0x01,0x43,0x43,0x04,0x3B,0x44,0x04,0x3A,0x44,
	0x04,0x39,0x44,0x04,0x38,0x44,0x00,0x80,0x21,0x04,0x34,0x44,0x00,0x00,0x21,0x04,
	0x3C,0x44,0x2A,0x0F,0x21,0x04,0x2F,0x44,0x00,0x44,0x21,0x04,0x2A,0x44,0x42,0x02,
	0x21,0x02,0x2B,0x43,0x04,0x29,0x44,0x00,0x00,0x21,0x4C,0x19,0x43,0x1F,0x18,0x43,
	0x75,0x00,0x43,0x00,0x00,0x42,0xC0,0x3D,0x42,0x80,0x3D,0x43,0x0A,0x59,0x43,0x04,
	0x35,0x44,0x00,0x00,0x21,0x04,0x05,0x43,0x42,0xBB,0x00,
};

static uint8_t imx091_act_init_mem_settings[] = {
    0x88,0xEF,0x00,0x08,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x40,0x00,0x40,0x00,0x40,
    0x00,0x40,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x80,0x00,0x00,0x78,0x82,0xBF,0x7E,
    0x3C,0x01,0xFF,0x7F,0xCC,0x7C,0x50,0x78,0x82,0x70,0x78,0x68,0x2B,0x63,0xEE,0x5E,
    0x25,0x54,0xDC,0x48,0x3E,0x3E,0x33,0x32,0x70,0x27,0x4F,0x1E,0x86,0x18,0xF4,0x17,
    0x74,0x12,0x00,0x00,0x00,0x00,0x00,0x00,0xFF,0x7F,0x60,0xA0,0x52,0x98,0x10,0x56,
    0x7D,0x72,0x75,0xE6,0x00,0x40,0x00,0x89,0x00,0x40,0x00,0x19,0x00,0x7F,0xFF,0x7F,
    0x00,0x0F,0x00,0x00,0xFF,0x7F,0x00,0x00,0x00,0x00,0x00,0xCC,0x70,0x74,0xF2,0x7F,
    0xEE,0x27,0x00,0x00,0xF2,0x7F,0x00,0x00,0x00,0x00,0x51,0xE2,0x35,0x59,0x40,0x7E,
    0x00,0x15,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x51,0xE2,0x73,0x60,0x80,0x56,
    0xFF,0x7F,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x40,0x00,0x40,0x00,0x48,
    0x00,0x38,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x10,0x00,0x10,0x85,0x7C,
    0xEE,0x0D,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x10,0x00,0x10,0x85,0x7C,
    0xEE,0x0D,0x00,0x20,0xFF,0x7F,0x00,0x00,0x00,0x00,0x00,0x40,0x00,0x40,0x00,0x60,
    0x00,0x20,0x00,0x00,0x10,0x00,0x00,0x00,0x00,0x00,0x00,0x40,0x00,0x40,0x00,0x60,
    0x00,0x20,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0xD9,0x07,0x00,0x00,
    0x00,0x00,0x00,0x32,0x45,0x40,0x00,0x0F,0x10,0x00,0x00,0x00,0x00,0x10,0x00,0x00,
    0x00,0x10,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x40,0x00,0x40,0x00,0x70,
    0x00,0x10,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x02,0x00,0x00,0x00,0x00,0x00,0x00,
    0x00,0x00,0xB7,0x2D,0x00,0x00,0x00,0x40,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,
    0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x0B,0x01,0x00,0x01,0x00,0x00,0x00,
    0x08,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0xFF,0x7F,0x60,0xA0,0x52,0x98,0x10,0x56,
    0x7D,0x72,0x8B,0x19,0x00,0x40,0x00,0x89,0x00,0x40,0x00,0x19,0x00,0x7F,0xFF,0x7F,
    0x00,0x0F,0x00,0x00,0xFF,0x7F,0x00,0x00,0x00,0x00,0x00,0xCC,0x70,0x74,0xF2,0x7F,
    0xEE,0x27,0x00,0x00,0xF2,0x7F,0x00,0x00,0x00,0x00,0x51,0xE2,0x35,0x59,0x40,0x7E,
    0x00,0x15,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x51,0xE2,0x73,0x60,0x80,0x56,
    0xFF,0x7F,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x40,0x00,0x40,0x00,0x48,
    0x00,0x38,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x10,0x00,0x10,0x85,0x7C,
    0xEE,0x0D,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x10,0x00,0x10,0x85,0x7C,
    0xEE,0x0D,0x00,0x20,0xFF,0x7F,0x00,0x00,0x00,0x00,0x00,0x40,0x00,0x40,0x00,0x60,
    0x00,0x20,0x00,0x00,0x10,0x00,0x00,0x00,0x00,0x00,0x00,0x40,0x00,0x40,0x00,0x60,
    0x00,0x20
};

static int32_t imx091_actuator_write_ois(uint8_t cmd_type, uint8_t addr, uint16_t data)
{
	int32_t rc = -EFAULT;
	struct i2c_msg msg;
	uint16_t saddr = imx091_act_t.i2c_client.client->addr >> 1;
	int i = 0;
	uint8_t trans_buf[4] = {0};

	CDBG("%s L.%d cmd:0x%2x adr:0x%2x", __func__, __LINE__, cmd_type, addr);

	if((cmd_type != IMX091_ACT_OIS_CMD_SPE) && (cmd_type != IMX091_ACT_OIS_CMD_MEM) &&
		(cmd_type != IMX091_ACT_OIS_CMD_PER)) {
		pr_err("%s L.%d invalid cmd 0x%x", __func__, __LINE__, cmd_type);
		return rc;
	}

	if (cmd_type == IMX091_ACT_OIS_CMD_SPE) {
		msg.flags = 0;
		msg.len = 2;
		msg.addr = saddr;
		msg.buf = &trans_buf[0];
		trans_buf[i] = cmd_type;
		trans_buf[++i] = addr;
	} else {
		msg.flags = 0;
		msg.len = 4;
		msg.addr = saddr;
		msg.buf = &trans_buf[0];
		trans_buf[i] = cmd_type;
		trans_buf[++i] = addr;
		trans_buf[++i] = (uint8_t)(data & 0x00FF);
		trans_buf[++i] = (uint8_t)((data & 0xFF00) >> 8);
	}

	CDBG("%s L.%d data: 0x%2x 0x%2x 0x%2x 0x%2x", __func__, __LINE__, 
		trans_buf[0], trans_buf[1], trans_buf[2], trans_buf[3]);

	rc = i2c_transfer(imx091_act_t.i2c_client.client->adapter, &msg, 1);
	if(rc < 0){
		pr_err("%s L.%d failed rc:%d saddr:0x%x", __func__, __LINE__, rc, saddr);
		return rc;
	}

	CDBG("%s L.%d rc:%d", __func__, __LINE__, rc);

	return rc;
}

static int32_t imx091_act_status_check(struct msm_actuator_ctrl_t *a_ctrl,
										int retry_max)
{
	unsigned char	sts[4];
	uint16_t saddr = 0x1c >> 1;
	int32_t rc = 0;
	struct i2c_msg msg[] = {
		{
			.flags = 0,
		 },
		{
			.flags = I2C_M_RD,
		},
	};
	int r_max = retry_max;

	while(r_max) {
		msg[0].len = 1;
		msg[0].addr = saddr;
		msg[0].buf =&sts[0];
		msg[1].len = 1;
		msg[1].addr = saddr;
		msg[1].buf =&sts[0];
		sts[0] = 0xF0;
		rc = i2c_transfer(a_ctrl->i2c_client.client->adapter, msg, 2);
		CDBG("rc = %d : Actuator Status %02x\n", rc, sts[0]);
		if((sts[0] & 0x88) == 0x00) { return 0; }

		msg[0].len = 2;
		msg[0].addr = saddr;
		msg[0].buf =&sts[0];
		msg[1].len = 2;
		msg[1].addr = saddr;
		msg[1].buf =&sts[0];
		sts[0] = 0x84;
		sts[1] = 0xF7;
		rc = i2c_transfer(a_ctrl->i2c_client.client->adapter, msg, 2);
		CDBG("rc = %d : Ois Status %02x, %02x\n", rc, sts[0],sts[1]);
		if(rc > 0) {
			if(sts[1] == 0x84) {
				msg[0].flags = 0;
				msg[0].len = 4;
				msg[0].addr = saddr;
				msg[0].buf =&sts[0];
				sts[0] = 0x84;
				sts[1] = 0xF7;
				sts[2] = 0x04;
				sts[3] = 0x01;
				rc = i2c_transfer(a_ctrl->i2c_client.client->adapter, msg, 1);
				if (rc < 0){
					pr_err("msm_camera_i2c_txdata faild 0x%x\n", saddr);
				}
			}
		}

		msleep(1);
		--r_max;
	}

	return -EFAULT;
}

int32_t imx091_actuator_i2c_write(struct msm_actuator_ctrl_t *a_ctrl,
	int16_t next_lens_position, uint32_t hw_params)
{
	int32_t rc = 0;
	unsigned char	data[5];
	uint16_t saddr = 0x1c >> 1;
	struct i2c_msg msg[] = {
		{
			.flags = 0,
			.len = 5,
		 },
	};

	CDBG("%s: lens_position=%d\n", __func__, next_lens_position);
	if(imx091_act_status_check(a_ctrl, 10) < 0) {
		pr_err("%s: cannot i2c_txdata 0x%x\n", __func__, saddr);
		kfree(data);
		return -EFAULT;
	}

	data[0] = 0xF0;
	data[1] = 0x90;
	data[2] = 0x00;
	data[3] = (next_lens_position & 0xFF00) >> 8;
	data[4] = (next_lens_position & 0x00FF);
	msg[0].addr=saddr;
	msg[0].buf=&data[0];

	rc = i2c_transfer(a_ctrl->i2c_client.client->adapter, msg, 1);
	if(rc > 0) {
		rc = 0;
		virtual_vcm_dreg = next_lens_position;
	}

	CDBG("%s: exit rc=%d\n", __func__, rc);
	return rc;
}

int32_t imx091_actuator_write_focus(
	struct msm_actuator_ctrl_t *a_ctrl,
	uint16_t curr_lens_pos,
	struct damping_params_t *damping_params,
	int8_t sign_direction,
	int16_t code_boundary)
{
	int32_t rc = 0;
	int16_t next_lens_pos = 0;
	uint16_t damping_code_step = 0;
	uint16_t wait_time = 0;

	damping_code_step = damping_params->damping_step;
	wait_time = damping_params->damping_delay;

	if(a_ctrl->func_tbl) {
		rc = a_ctrl->func_tbl->actuator_i2c_write(a_ctrl, code_boundary,damping_params->hw_params);
	}
	++next_lens_pos;		//dummy code

	CDBG("%s: [L:%d]  Exit %d\n", __func__, __LINE__, rc);
	return rc;
}

int32_t imx091_actuator_power_down(struct msm_actuator_ctrl_t *a_ctrl)
{
	int32_t rc = 0;
	if (a_ctrl->vcm_enable) {
		rc = gpio_direction_output(a_ctrl->vcm_pwd, 0);
		if (!rc)
			gpio_free(a_ctrl->vcm_pwd);
	}

	if(a_ctrl->step_position_table)
		kfree(a_ctrl->step_position_table);
	a_ctrl->step_position_table = NULL;
	imx091_actuator_set_ois_off();
	imx091_ois_parameter.ois_state = OIS_STATE_POWEROFF;
	
	return rc;
}

int32_t imx091_actuator_init(struct msm_actuator_ctrl_t *a_ctrl,
	struct msm_actuator_set_info_t *set_info) {
	struct reg_settings_t *init_settings = NULL;
	int32_t rc = -EFAULT;
	uint16_t i = 0;
	CDBG("%s: IN\n", __func__);

	for (i = 0; i < ARRAY_SIZE(actuators); i++) {
		if (set_info->actuator_params.act_type ==
			actuators[i]->act_type) {
			a_ctrl->func_tbl = &actuators[i]->func_tbl;
			rc = 0;
		}
	}

	if (rc < 0) {
		pr_err("%s: Actuator function table not found\n", __func__);
		return rc;
	}

	a_ctrl->region_size = set_info->af_tuning_params.region_size;
	if (a_ctrl->region_size > MAX_ACTUATOR_REGION) {
		pr_err("%s: MAX_ACTUATOR_REGION is exceeded.\n", __func__);
		return -EFAULT;
	}
	a_ctrl->total_steps = set_info->af_tuning_params.total_steps;
	a_ctrl->pwd_step = set_info->af_tuning_params.pwd_step;
	a_ctrl->total_steps = set_info->af_tuning_params.total_steps;

	if (copy_from_user(&a_ctrl->region_params,
		(void *)set_info->af_tuning_params.region_params,
		a_ctrl->region_size * sizeof(struct region_params_t)))
		return -EFAULT;

	a_ctrl->i2c_data_type = set_info->actuator_params.i2c_data_type;
	a_ctrl->i2c_client.client->addr = set_info->actuator_params.i2c_addr;
	a_ctrl->i2c_client.addr_type = set_info->actuator_params.i2c_addr_type;
	a_ctrl->reg_tbl_size = set_info->actuator_params.reg_tbl_size;
	if (a_ctrl->reg_tbl_size > MAX_ACTUATOR_REG_TBL_SIZE) {
		pr_err("%s: MAX_ACTUATOR_REG_TBL_SIZE is exceeded.\n",
			__func__);
		return -EFAULT;
	}
	if (copy_from_user(&a_ctrl->reg_tbl,
		(void *)set_info->actuator_params.reg_tbl_params,
		a_ctrl->reg_tbl_size *
		sizeof(struct msm_actuator_reg_params_t)))
		return -EFAULT;

	if (set_info->actuator_params.init_setting_size) {
		if (a_ctrl->func_tbl->actuator_init_focus) {
			init_settings = kmalloc(sizeof(struct reg_settings_t) *
				(set_info->actuator_params.init_setting_size),
				GFP_KERNEL);
			if (init_settings == NULL) {
				pr_err("%s Error allocating memory for init_settings\n",
					__func__);
				return -EFAULT;
			}
			if (copy_from_user(init_settings,
				(void *)set_info->actuator_params.init_settings,
				set_info->actuator_params.init_setting_size *
				sizeof(struct reg_settings_t))) {
				kfree(init_settings);
				pr_err("%s Error copying init_settings\n",
					__func__);
				return -EFAULT;
			}
			rc = a_ctrl->func_tbl->actuator_init_focus(a_ctrl,
				set_info->actuator_params.init_setting_size,
				a_ctrl->i2c_data_type,
				init_settings);
			kfree(init_settings);
			if (rc < 0) {
				pr_err("%s Error actuator_init_focus\n",
					__func__);
				return -EFAULT;
			}
		}
	}

	a_ctrl->initial_code = set_info->af_tuning_params.initial_code;
	a_ctrl->terminal_code =  set_info->af_tuning_params.terminal_code;
	CDBG("%s%d : a_ctrl->initial_code = %d\n, a_ctrl->terminal_code=%d\n",__func__ ,__LINE__ ,a_ctrl->initial_code, a_ctrl->terminal_code);
	if (a_ctrl->func_tbl->actuator_init_step_table)
		rc = a_ctrl->func_tbl->
			actuator_init_step_table(a_ctrl, set_info);

	a_ctrl->curr_step_pos = 0;
	a_ctrl->curr_region_index = 0;

	return rc;
}

int32_t imx091_actuator_init_step_table(struct msm_actuator_ctrl_t *a_ctrl,
	struct msm_actuator_set_info_t *set_info)
{
	int32_t rc = 0;
	int16_t cur_code = 0;
	int16_t end_code = 0;
	int16_t step_index = 0, region_index = 0;
    int16_t code_per_step = 0;
	uint16_t step_boundary = 0;
	uint32_t max_code_size = 1;
	uint32_t total_steps = 0;
	uint16_t data_size = set_info->actuator_params.data_size;
	CDBG("%s called\n", __func__);

	for (; data_size > 0; data_size--)
		max_code_size *= 2;

	if(a_ctrl->step_position_table)
		kfree(a_ctrl->step_position_table);

	a_ctrl->step_position_table = NULL;

	a_ctrl->step_position_table =
		kmalloc(sizeof(uint16_t) *
		(set_info->af_tuning_params.total_steps + 1), GFP_KERNEL);

	if (a_ctrl->step_position_table == NULL)
		return -EFAULT;

	if(imx091_act_t.af_adjustflg == 0x55){
		
		
		cur_code = a_ctrl->initial_code;
		end_code = a_ctrl->terminal_code;
		total_steps = a_ctrl->total_steps;

		CDBG("cur_code= %d, end_code =%d, total_steps=%d\n",cur_code, end_code, total_steps);

		for ( step_index = 0; step_index < total_steps; step_index++)
		{
			a_ctrl->step_position_table[step_index] = cur_code + step_index * ( end_code - cur_code ) / (total_steps - 1);
			CDBG("step_position_table[%d]= %d\n",step_index,a_ctrl->step_position_table[step_index]);
		}
	}else
	{
		cur_code = 200;
		a_ctrl->step_position_table[step_index++] = cur_code;
		for (region_index = 0;
			region_index < a_ctrl->region_size;
			region_index++) {
			code_per_step =
				a_ctrl->region_params[region_index].code_per_step;
			step_boundary =
				a_ctrl->region_params[region_index].
				step_bound[MOVE_NEAR];
			for (; step_index <= step_boundary;
				step_index++) {
				cur_code += code_per_step;
				if (cur_code < max_code_size)
					a_ctrl->step_position_table[step_index] =
						cur_code;
				else {
					for (; step_index <
						set_info->af_tuning_params.total_steps;
						step_index++)
						a_ctrl->
							step_position_table[
							step_index] =
							max_code_size;

				return rc;
				}
			}
		}
		
	}
	return rc;
}

int32_t imx091_actuator_set_ois_init(void)
{
	struct i2c_msg msg[2];
	int32_t rc = 0;
	uint16_t saddr = imx091_act_t.i2c_client.client->addr >> 1;
	unsigned char	sts[4];
       int i;
	
	CDBG("%s:%d",__func__,__LINE__);
	
	if(imx091_ois_parameter.ois_state == OIS_STATE_POWEROFF &&
	   imx091_ois_parameter.ois_adjustflg == 0x55)
	{
		for(i = 0; i < ARRAY_SIZE(ois_init_setting2); i++) {
			rc = imx091_actuator_write_ois(ois_init_setting2[i].type,
				ois_init_setting2[i].addr, ois_init_setting2[i].data);
			if (rc < 0){
				pr_err("msm_camera_i2c_txdata faild No.%d 0x%x\n", i, saddr);
				return -EFAULT;
			}
		}

		CDBG("CURDAT %d\n", imx091_ois_parameter.curdat);
		CDBG("HALOFS %d\n", imx091_ois_parameter.halofs);
		CDBG("PSTXOF %d\n", imx091_ois_parameter.pstxof);
		CDBG("PSTYOF %d\n", imx091_ois_parameter.pstyof);
		CDBG("HX_OFS %d\n", imx091_ois_parameter.hx_ofs);
		CDBG("HY_OFS %d\n", imx091_ois_parameter.hy_ofs);
		CDBG("HX_OFS %d\n", imx091_ois_parameter.gx_ofs);
		CDBG("HY_OFS %d\n", imx091_ois_parameter.gy_ofs);
		CDBG("KGxHG  %d\n", imx091_ois_parameter.kgxhg);
		CDBG("KGyHG  %d\n", imx091_ois_parameter.kgyhg);
		CDBG("KGXG   %d\n", imx091_ois_parameter.kgxg);
		CDBG("KGKY   %d\n", imx091_ois_parameter.kgyg);

		msg[0].flags = 0;
		msg[0].len = 4;
		msg[0].addr = saddr;
		msg[0].buf =&sts[0];
		sts[0]=0x82;
		sts[1]=0x30;
		sts[2]=(char)(imx091_ois_parameter.curdat & 0x00ff);
		sts[3]=(char)(imx091_ois_parameter.curdat >> 8);
		rc = i2c_transfer(imx091_act_t.i2c_client.client->adapter, msg, 1);
		if (rc < 0){
			pr_err("msm_camera_i2c_txdata faild 0x%x\n", saddr);
			return -EFAULT;
		}
	
		msg[0].flags = 0;
		msg[0].len = 4;
		msg[0].addr = saddr;
		msg[0].buf =&sts[0];
		sts[0]=0x82;
		sts[1]=0x31;
		sts[2]=(char)(imx091_ois_parameter.halofs & 0x00ff);
		sts[3]=(char)(imx091_ois_parameter.halofs >> 8);
		rc = i2c_transfer(imx091_act_t.i2c_client.client->adapter, msg, 1);
		if (rc < 0){
			pr_err("msm_camera_i2c_txdata faild 0x%x\n", saddr);
			return -EFAULT;
		}

		msg[0].flags = 0;
		msg[0].len = 4;
		msg[0].addr = saddr;
		msg[0].buf =&sts[0];
		sts[0]=0x84;
		sts[1]=0x1e;
		sts[2]=(char)(imx091_ois_parameter.hx_ofs & 0x00ff);
		sts[3]=(char)(imx091_ois_parameter.hx_ofs >> 8);
		rc = i2c_transfer(imx091_act_t.i2c_client.client->adapter, msg, 1);
		if (rc < 0){
			pr_err("msm_camera_i2c_txdata faild 0x%x\n", saddr);
			return -EFAULT;
		}

		msg[0].flags = 0;
		msg[0].len = 4;
		msg[0].addr = saddr;
		msg[0].buf =&sts[0];
		sts[0]=0x84;
		sts[1]=0x9e;
		sts[2]=(char)(imx091_ois_parameter.hy_ofs & 0x00ff);
		sts[3]=(char)(imx091_ois_parameter.hy_ofs >> 8);
		rc = i2c_transfer(imx091_act_t.i2c_client.client->adapter, msg, 1);
		if (rc < 0){
			pr_err("msm_camera_i2c_txdata faild 0x%x\n", saddr);
			return -EFAULT;
		}

		msg[0].flags = 0;
		msg[0].len = 4;
		msg[0].addr = saddr;
		msg[0].buf =&sts[0];
		sts[0]=0x82;
		sts[1]=0x39;
		sts[2]=(char)(imx091_ois_parameter.pstxof & 0x00ff);
		sts[3]=(char)(imx091_ois_parameter.pstxof >> 8);
		rc = i2c_transfer(imx091_act_t.i2c_client.client->adapter, msg, 1);
		if (rc < 0){
			pr_err("msm_camera_i2c_txdata faild 0x%x\n", saddr);
			return -EFAULT;
		}

		msg[0].flags = 0;
		msg[0].len = 4;
		msg[0].addr = saddr;
		msg[0].buf =&sts[0];
		sts[0]=0x82;
		sts[1]=0x3b;
		sts[2]=(char)(imx091_ois_parameter.pstyof & 0x00ff);
		sts[3]=(char)(imx091_ois_parameter.pstyof >> 8);
		rc = i2c_transfer(imx091_act_t.i2c_client.client->adapter, msg, 1);
		if (rc < 0){
			pr_err("msm_camera_i2c_txdata faild 0x%x\n", saddr);
			return -EFAULT;
		}

		msg[0].flags = 0;
		msg[0].len = 4;
		msg[0].addr = saddr;
		msg[0].buf =&sts[0];
		sts[0]=0x84;
		sts[1]=0x06;
		sts[2]=(char)(imx091_ois_parameter.gx_ofs & 0x00ff);
		sts[3]=(char)(imx091_ois_parameter.gx_ofs >> 8);
		rc = i2c_transfer(imx091_act_t.i2c_client.client->adapter, msg, 1);
		if (rc < 0){
			pr_err("msm_camera_i2c_txdata faild 0x%x\n", saddr);
			return -EFAULT;
		}

		msg[0].flags = 0;
		msg[0].len = 4;
		msg[0].addr = saddr;
		msg[0].buf =&sts[0];
		sts[0]=0x84;
		sts[1]=0x86;
		sts[2]=(char)(imx091_ois_parameter.gy_ofs & 0x00ff);
		sts[3]=(char)(imx091_ois_parameter.gy_ofs >> 8);
		rc = i2c_transfer(imx091_act_t.i2c_client.client->adapter, msg, 1);
		if (rc < 0){
			pr_err("msm_camera_i2c_txdata faild 0x%x\n", saddr);
			return -EFAULT;
		}

		msg[0].flags = 0;
		msg[0].len = 4;
		msg[0].addr = saddr;
		msg[0].buf =&sts[0];
		sts[0]=0x84;
		sts[1]=0x46;
		sts[2]=(char)(imx091_ois_parameter.kgxhg & 0x00ff);
		sts[3]=(char)(imx091_ois_parameter.kgxhg >> 8);
		rc = i2c_transfer(imx091_act_t.i2c_client.client->adapter, msg, 1);
		if (rc < 0){
			pr_err("msm_camera_i2c_txdata faild 0x%x\n", saddr);
			return -EFAULT;
		}

		msg[0].flags = 0;
		msg[0].len = 4;
		msg[0].addr = saddr;
		msg[0].buf =&sts[0];
		sts[0]=0x84;
		sts[1]=0xC6;
		sts[2]=(char)(imx091_ois_parameter.kgyhg & 0x00ff);
		sts[3]=(char)(imx091_ois_parameter.kgyhg >> 8);
		rc = i2c_transfer(imx091_act_t.i2c_client.client->adapter, msg, 1);
		if (rc < 0){
			pr_err("msm_camera_i2c_txdata faild 0x%x\n", saddr);
			return -EFAULT;
		}

		msg[0].flags = 0;
		msg[0].len = 4;
		msg[0].addr = saddr;
		msg[0].buf =&sts[0];
		sts[0]=0x84;
		sts[1]=0x0f;
		sts[2]=(char)(imx091_ois_parameter.kgxg & 0x00ff);
		sts[3]=(char)(imx091_ois_parameter.kgxg >> 8);
		rc = i2c_transfer(imx091_act_t.i2c_client.client->adapter, msg, 1);
		if (rc < 0){
			pr_err("msm_camera_i2c_txdata faild 0x%x\n", saddr);
			return -EFAULT;
		}

		msg[0].flags = 0;
		msg[0].len = 4;
		msg[0].addr = saddr;
		msg[0].buf =&sts[0];
		sts[0]=0x84;
		sts[1]=0x8f;
		sts[2]=(char)(imx091_ois_parameter.kgyg & 0x00ff);
		sts[3]=(char)(imx091_ois_parameter.kgyg >> 8);
		rc = i2c_transfer(imx091_act_t.i2c_client.client->adapter, msg, 1);
		if (rc < 0){
			pr_err("msm_camera_i2c_txdata faild 0x%x\n", saddr);
			return -EFAULT;
		}

		for(i = 0; i < ARRAY_SIZE(ois_init_setting3); i++) {
			rc = imx091_actuator_write_ois(ois_init_setting3[i].type,
				ois_init_setting3[i].addr, ois_init_setting3[i].data);
			if (rc < 0){
				pr_err("msm_camera_i2c_txdata faild No.%d 0x%x\n", i, saddr);
				return -EFAULT;
			}
		}
	}else
	{
		CDBG("ois_state is not OIS_STATE_POWEROFF\n");
	}
	return rc;
}

int32_t imx091_actuator_set_ois_still(void)
{
	struct i2c_msg msg[2];
	int32_t rc = 0;
	uint16_t saddr = imx091_act_t.i2c_client.client->addr >> 1;
	unsigned char	sts[4];
	int i;
	
	CDBG("%s:%d",__func__,__LINE__);
	
	if(imx091_ois_parameter.ois_adjustflg == 0x55 &&
	   imx091_ois_parameter.ois_state != OIS_STATE_STILL )
	{
		imx091_actuator_set_ois_off();
		
		msg[0].flags = 0;
		msg[0].len = 4;
		msg[0].addr = saddr;
		msg[0].buf =&sts[0];
		sts[0]=0x82;
		sts[1]=0x30;
		sts[2]=(char)(imx091_ois_parameter.curdat & 0x00ff);
		sts[3]=(char)(imx091_ois_parameter.curdat >> 8);
		rc = i2c_transfer(imx091_act_t.i2c_client.client->adapter, msg, 1);
		if (rc < 0){
			pr_err("msm_camera_i2c_txdata faild 0x%x\n", saddr);
			return -EFAULT;
		}
		
		for(i = 0; i < ARRAY_SIZE(ois_still_setting); i++) {
			rc = imx091_actuator_write_ois(ois_still_setting[i].type,
				ois_still_setting[i].addr, ois_still_setting[i].data);
			if (rc < 0){
				LINFO("msm_camera_i2c_txdata faild No.%d 0x%x\n", i, saddr);
				return -EFAULT;
			}
		}
		
		imx091_ois_parameter.ois_state = OIS_STATE_STILL;
		
	}else
	{
		if(imx091_ois_parameter.ois_adjustflg != 0x55)
			CDBG("imx091_act_t.ois_adjustflg != 0x55\n");
		else
			CDBG("ois_state is not OIS_STATE_STILL\n");
	}
	return rc;
}

int32_t imx091_actuator_set_ois_video(void)
{
	struct i2c_msg msg[2];
	int32_t rc = 0;
	uint16_t saddr = imx091_act_t.i2c_client.client->addr >> 1;
	unsigned char	sts[4];
	int i;
	
	CDBG("%s:%d",__func__,__LINE__);
	
	if(imx091_ois_parameter.ois_adjustflg == 0x55 &&
	   imx091_ois_parameter.ois_state != OIS_STATE_VIDEO )
	{
		imx091_actuator_set_ois_off();
		
		msg[0].flags = 0;
		msg[0].len = 4;
		msg[0].addr = saddr;
		msg[0].buf =&sts[0];
		sts[0]=0x82;
		sts[1]=0x30;
		sts[2]=(char)(imx091_ois_parameter.curdat & 0x00ff);
		sts[3]=(char)(imx091_ois_parameter.curdat >> 8);
		rc = i2c_transfer(imx091_act_t.i2c_client.client->adapter, msg, 1);
		if (rc < 0){
			pr_err("msm_camera_i2c_txdata faild 0x%x\n", saddr);
			return -EFAULT;
		}
		
		for(i = 0; i < ARRAY_SIZE(ois_video_setting); i++) {
			rc = imx091_actuator_write_ois(ois_video_setting[i].type,
				ois_video_setting[i].addr, ois_video_setting[i].data);
			if (rc < 0){
				LINFO("msm_camera_i2c_txdata faild No.%d 0x%x\n", i, saddr);
				return -EFAULT;
			}
		}
		
		imx091_ois_parameter.ois_state = OIS_STATE_VIDEO;
		
	}else
	{
		if(imx091_ois_parameter.ois_adjustflg != 0x55)
			CDBG("imx091_act_t.ois_adjustflg != 0x55\n");
		else
			CDBG("ois_state is not OIS_STATE_VIDEO\n");
	}
	return rc;
}

int32_t imx091_actuator_set_ois_off(void)
{
	int32_t rc = 0;
	uint16_t saddr = imx091_act_t.i2c_client.client->addr >> 1;
	int i;

	CDBG("%s:%d",__func__,__LINE__);

	if( imx091_ois_parameter.ois_state != OIS_STATE_OFF )
	{
		for(i = 0; i < ARRAY_SIZE(ois_off_setting); i++) {
			rc = imx091_actuator_write_ois(ois_off_setting[i].type,
				ois_off_setting[i].addr, ois_off_setting[i].data);
			if (rc < 0){
				LINFO("msm_camera_i2c_txdata faild No.%d 0x%x\n", i, saddr);
				return -EFAULT;
			}
		}
		imx091_ois_parameter.ois_state = OIS_STATE_OFF;
	}
	
	return rc;
}

DEFINE_MUTEX(imx091_act_event_mutex);
int32_t imx091_actuator_power_up(struct msm_actuator_ctrl_t *a_ctrl);
int16_t imx091_lens_pos = 0;
int8_t  imx091_ois_mode = 0;
int32_t imx091_act_store_event = 0;
#define IMX091_ACT_EVENT_FW_DL			0x00000001 
#define IMX091_ACT_EVENT_SET_OIS		0x00000002 
#define IMX091_ACT_EVENT_INIT_LENSPOS	0x00000004 
#define IMX091_ACT_EVENT_MASK			(IMX091_ACT_EVENT_FW_DL | \
										 IMX091_ACT_EVENT_SET_OIS | \
										 IMX091_ACT_EVENT_INIT_LENSPOS)

int32_t imx091_actuator_set_ois(int8_t mode)
{
	int32_t rc = 0;

	rc = imx091_actuator_set_ois_init();
	if (rc < 0){
		pr_err("imx091_actuator_set_ois_init faild\n");
		rc = -EFAULT;
	}

	if( imx091_ois_parameter.ois_state == OIS_STATE_POWEROFF ) {
		rc = imx091_actuator_set_ois_off();
		if (rc < 0){
			pr_err("imx091_actuator_set_ois_off faild\n");
			rc = -EFAULT;
		}
	}

	if( mode == SHCAM_OIS_MODE_STILL ) {
		rc = imx091_actuator_set_ois_still();
		if (rc < 0){
			pr_err("imx091_actuator_set_ois_still faild\n");
			rc = -EFAULT;
		}
	}
	else if( mode == SHCAM_OIS_MODE_VIDEO ){
		rc = imx091_actuator_set_ois_video();
		if (rc < 0){
			pr_err("imx091_actuator_set_ois_video faild\n");
			rc = -EFAULT;
		}
	}
	else {
		rc = imx091_actuator_set_ois_off();
		if (rc < 0){
			pr_err("imx091_actuator_set_ois_off faild\n");
			rc = -EFAULT;
		}
	}
	return rc;
}

int32_t imx091_actuator_powerup_seq(struct msm_actuator_ctrl_t *a_ctrl,
									struct msm_actuator_cfg_data *cdata)
{
	int32_t rc = 0;

	mutex_lock(&imx091_act_event_mutex);

	if((imx091_act_store_event & IMX091_ACT_EVENT_FW_DL) == 0) {
		rc = imx091_actuator_power_up(a_ctrl);
		usleep_range(1000, 2000);
		if (rc < 0)
			pr_err("%s move focus failed %d\n", __func__, rc);
		rc = a_ctrl->func_tbl->actuator_set_default_focus(a_ctrl,
			&cdata->cfg.move);
		if (rc < 0)
			pr_err("%s move focus failed %d\n", __func__, rc);

		imx091_act_store_event &= IMX091_ACT_EVENT_MASK;
		if( imx091_act_store_event != 0 ) {
			if(imx091_act_store_event & IMX091_ACT_EVENT_SET_OIS) {
				CDBG("%s actioc IMX091_ACT_EVENT_SET_OIS mode = %d\n", __func__, imx091_ois_mode);
				rc = imx091_actuator_set_ois(imx091_ois_mode);
				if (rc < 0){
					pr_err("imx091_actuator_set_ois faild\n");
				}
			}
			if(imx091_act_store_event & IMX091_ACT_EVENT_INIT_LENSPOS) {
				CDBG("%s action IMX091_ACT_EVENT_INIT_LENSPOS lenspos = %d\n", __func__, imx091_lens_pos);
				imx091_actuator_i2c_write(a_ctrl, imx091_lens_pos, 0);
			}
		}
		imx091_act_store_event = IMX091_ACT_EVENT_FW_DL;
	}
	else {
		CDBG("%s actuator started \n", __func__);
	}

	mutex_unlock(&imx091_act_event_mutex);
	return rc;
}

int32_t imx091_actuator_config(struct msm_actuator_ctrl_t *a_ctrl,
							void __user *argp)
{
	struct msm_actuator_cfg_data cdata;
	int32_t rc = 0;
	uint16_t saddr = imx091_act_t.i2c_client.client->addr >> 1;
	if (copy_from_user(&cdata,
		(void *)argp,
		sizeof(struct msm_actuator_cfg_data)))
		return -EFAULT;
	mutex_lock(a_ctrl->actuator_mutex);
	CDBG("%s called, type %d\n", __func__, cdata.cfgtype);
	switch (cdata.cfgtype) {
	case CFG_SH_ACTUATOR_FW_DL:
		{
			rc = imx091_actuator_powerup_seq(a_ctrl, &cdata);
			if (rc < 0)
				pr_err("%s actuator_power up failed %d\n", __func__, rc);
		}
		break;
	case CFG_SHDIAG_GET_I2C_DATA:
		{
			rc = imx091_actuator_powerup_seq(a_ctrl, &cdata);
			if (rc < 0)
				pr_err("%s actuator_power up failed %d\n", __func__, rc);

			if(cdata.cfg.i2c_info.length == 3) {
				void *data;
				unsigned char * pd;
				data = kmalloc(cdata.cfg.i2c_info.length, GFP_KERNEL);
				if(data == NULL){
					rc = -EFAULT;
					break;
				}
				pd = (unsigned char*)data;
				pd[0] = (unsigned char)((virtual_vcm_dreg & 0xFF00) >> 8);
				pd[1] = (unsigned char)(virtual_vcm_dreg & 0x00FF);
				pd[2] = 0;
				CDBG("%s:%d i2c_read data=0x%02x%02x%02x\n",__func__,__LINE__,*(unsigned char *)data,*((unsigned char *)data + 1),*((unsigned char *)data +2));
				if (copy_to_user((void *)cdata.cfg.i2c_info.data,
					data,
					cdata.cfg.i2c_info.length)){
					kfree(data);
					pr_err("%s copy_to_user error\n",__func__);
					rc = -EFAULT;
					break;
				}
				kfree(data);
				if (copy_to_user((void *)argp,
					&cdata,
					sizeof(struct sensor_cfg_data))) {
					rc = -EFAULT;
					break;
				}
			}
			else if(cdata.cfg.i2c_info.length == 4) {
				uint8_t *data;
				uint8_t read_addr[2];
				uint8_t	sts[2];

				struct i2c_msg msg[] = {
					{
						.flags = 0,
					 },
					{
						.flags = I2C_M_RD,
					},
				};

				data = (uint8_t *)kmalloc(cdata.cfg.i2c_info.length, GFP_KERNEL);
				if(data == NULL){
					rc = -EFAULT;
					break;
				}
				if (copy_from_user(data,
					(uint8_t *)cdata.cfg.i2c_info.data,
					cdata.cfg.i2c_info.length)){
					kfree(data);
					pr_err("%s copy_to_user error\n",__func__);
					rc = -EFAULT;
					break;
				}

				memcpy(read_addr, (uint8_t *)data, 2);
				msg[0].len = ARRAY_SIZE(read_addr);
				msg[0].addr = saddr;
				msg[0].buf =&read_addr[0];
				msg[1].len = 2;
				msg[1].addr = saddr;
				msg[1].buf =&sts[0];
				sts[0] = 0x00;
				sts[1] = 0x00;
				rc = i2c_transfer(a_ctrl->i2c_client.client->adapter, msg, 2);
				if (rc < 0){
					pr_err("msm_camera_i2c_rxdata failed 0x%x\n", saddr);
					kfree(data);
					rc = -EFAULT;
					break;
				}
				CDBG("msg[0].read_addr = %02x read_addr = %02x\n", read_addr[0], read_addr[1]);
				CDBG("msg[0].len = %d addr = %02x buf = %p\n", msg[0].len, msg[0].addr, msg[0].buf);
				CDBG("msg[1].len = %d addr = %02x buf = %p\n", msg[1].len, msg[1].addr, msg[1].buf);
				CDBG("rc = %d : Ois i2c_read data= %02x, %02x\n", rc, sts[0],sts[1]);
				memcpy(&data[2], sts, 2);
				CDBG("rc = %d : debug Ois i2c_read data= %02x, %02x\n", rc, data[2],data[3]);

				if (copy_to_user((uint8_t *)cdata.cfg.i2c_info.data,
					data,
					cdata.cfg.i2c_info.length)){
					kfree(data);
					pr_err("%s copy_to_user error\n",__func__);
					rc = -EFAULT;
					break;
				}
				kfree(data);
				if (copy_to_user((void *)argp,
					&cdata,
					sizeof(struct sensor_cfg_data))) {
					rc = -EFAULT;
					break;
				}
			}
			else if(cdata.cfg.i2c_info.length == 1) {
				uint8_t *data;
				uint8_t	sts[2];

				struct i2c_msg msg[] = {
					{
						.flags = 0,
					 },
					{
						.flags = I2C_M_RD,
					},
				};

				data = (uint8_t *)kmalloc(cdata.cfg.i2c_info.length, GFP_KERNEL);
				if(data == NULL){
					rc = -EFAULT;
					break;
				}
				if (copy_from_user(data,
					(uint8_t *)cdata.cfg.i2c_info.data,
					cdata.cfg.i2c_info.length)){
					kfree(data);
					pr_err("%s copy_to_user error\n",__func__);
					rc = -EFAULT;
					break;
				}

				msg[0].len = 1;
				msg[0].addr = saddr;
				msg[0].buf = &sts[0];
				msg[1].len = 1;
				msg[1].addr = saddr;
				msg[1].buf =&sts[0];
				sts[0] = 0xF0;
				rc = i2c_transfer(a_ctrl->i2c_client.client->adapter, msg, 2);
				CDBG("msg[0].len = %d addr = %02x buf = %p\n", msg[0].len, msg[0].addr, msg[0].buf);
				CDBG("msg[1].len = %d addr = %02x buf = %p\n", msg[1].len, msg[1].addr, msg[1].buf);
				CDBG("rc = %d : Actuator i2c_read data= %02x\n", rc, sts[0]);
				memcpy(&data[0], sts, 1);
				CDBG("rc = %d : Actuator i2c_read data= %02x\n", rc, data[0]);

				if (copy_to_user((uint8_t *)cdata.cfg.i2c_info.data,
					data,
					cdata.cfg.i2c_info.length)){
					kfree(data);
					pr_err("%s copy_to_user error\n",__func__);
					rc = -EFAULT;
					break;
				}
				kfree(data);
				if (copy_to_user((void *)argp,
					&cdata,
					sizeof(struct sensor_cfg_data))) {
					rc = -EFAULT;
					break;
				}
			} else {
				void *data;
				struct i2c_msg msgs[] = {
					{
						.flags = I2C_M_RD,
						.len   = 3,
					},
				};

				data = kmalloc(cdata.cfg.i2c_info.length, GFP_KERNEL);
				if(data == NULL){
					rc = -EFAULT;
					break;
				}
				CDBG("%s:%d saddr=0x%0x\n",__func__,__LINE__, saddr);
				
				msgs[0].len = cdata.cfg.i2c_info.length;
				msgs[0].addr=saddr;
				msgs[0].buf=data;

				rc = i2c_transfer( a_ctrl->i2c_client.client->adapter, msgs, 1);
				if (rc < 0){
					CDBG("msm_camera_i2c_rxdata failed 0x%x\n", saddr);
					kfree(data);
					rc = -EFAULT;
					break;
				}

				CDBG("%s:%d i2c_read data=0x%02x\n",__func__,__LINE__,*(unsigned char *)data);
				if (copy_to_user((void *)cdata.cfg.i2c_info.data,
					data,
					cdata.cfg.i2c_info.length)){
					kfree(data);
					pr_err("%s copy_to_user error\n",__func__);
					rc = -EFAULT;
					break;
				}
				kfree(data);
				if (copy_to_user((void *)argp,
					&cdata,
					sizeof(struct sensor_cfg_data))) {
					rc = -EFAULT;
					break;
				}
			}
		}
		break;
	case CFG_SHDIAG_SET_I2C_DATA:
		{
			int16_t vcm_dreg;
			unsigned char ch[2];
			void *data;
			struct i2c_msg msg[] = {
				{
					.flags = 0,
					.len = 5,
				 },
			};

			rc = imx091_actuator_powerup_seq(a_ctrl, &cdata);
			if (rc < 0)
				pr_err("%s actuator_power up failed %d\n", __func__, rc);

			data = kmalloc(cdata.cfg.i2c_info.length, GFP_KERNEL);
			if(data == NULL){
					rc = -EFAULT;
					break;
			}
			if (copy_from_user(data,
				(void *)cdata.cfg.i2c_info.data,
				cdata.cfg.i2c_info.length)){
				kfree(data);
				pr_err("%s copy_to_user error\n",__func__);
					rc = -EFAULT;
					break;
			}

			if(imx091_act_status_check(a_ctrl, 10) < 0) {
				pr_err("cannot i2c_txdata 0x%x\n", saddr);
				kfree(data);
					rc = -EFAULT;
					break;
			}

			CDBG("%s:%d i2c_write data=0x%02x%02x%02x%02x%02x\n",__func__,__LINE__,*(unsigned char *)data,*((unsigned char *)data + 1),*((unsigned char *)data +2),*((unsigned char *)data +3),*((unsigned char *)data +4));
			
			msg[0].len = cdata.cfg.i2c_info.length;
			msg[0].addr=saddr;
			msg[0].buf=data;

			rc = i2c_transfer(a_ctrl->i2c_client.client->adapter, msg, 1);
			if (rc < 0){
				pr_err("msm_camera_i2c_txdata faild 0x%x\n", saddr);
				kfree(data);
					rc = -EFAULT;
					break;
			}
			if(rc > 0) {
				rc = 0;
				ch[0] = *((unsigned char *)data +3);
				ch[1] = *((unsigned char *)data +4);
				vcm_dreg = (int16_t)( (ch[0] & 0x03) << 8);
				vcm_dreg += (int16_t) (ch[1] & 0x03);
				virtual_vcm_dreg = vcm_dreg;
			}

			kfree(data);
			if (copy_to_user((void *)argp,
				&cdata,
				sizeof(struct sensor_cfg_data))) {
				rc = -EFAULT;
				break;
			}
		}
		break;
	case CFG_SHDIAG_ACTUATOR_SET_TABLE:
		{
			uint16_t lens_start, lens_end, search_point;
			int16_t step_index = 0;

			lens_start = cdata.cfg.af_info.lens_start;
			lens_end = cdata.cfg.af_info.lens_end;
			search_point = cdata.cfg.af_info.search_point - 1;
			CDBG("%s lens_start = %d lens_end = %d search_point = %d\n", __func__, lens_start, lens_end, search_point );
			
			if(lens_start < 0){
				rc = -EFAULT;
				break;
			}
			if(lens_end > 0x3ff){
				rc = -EFAULT;
				break;
			}
			if(search_point > ( lens_end - lens_start )){
				rc = -EFAULT;
				break;
			}
			if(a_ctrl->step_position_table != NULL){
				CDBG("%s free default step_position_table\n", __func__ );
				kfree(a_ctrl->step_position_table);
				a_ctrl->step_position_table = NULL;
			}
			a_ctrl->step_position_table = kmalloc(sizeof(uint16_t) * (search_point + 1) , GFP_KERNEL);
			if(a_ctrl->step_position_table == NULL){
				rc = -EFAULT;
				break;
			}
			
			a_ctrl->step_position_table[step_index++] = lens_start;
			
			for (; step_index <= search_point; step_index++) {
				a_ctrl->step_position_table[step_index] = lens_start + ( step_index * ( lens_end - lens_start ) + search_point / 2) / search_point;
			}
			
			for (step_index = 0; step_index <= search_point; step_index++) {
				CDBG("step_position_table[%d]= 0x%0x\n", step_index, a_ctrl->step_position_table[step_index]);
			}
			a_ctrl->curr_step_pos = 0;
			a_ctrl->curr_region_index = 0;
			a_ctrl->total_steps = search_point + 1;
			a_ctrl->region_params[0].step_bound[0] = search_point + 1;

			if (copy_to_user((void *)cdata.cfg.af_info.p_step_position_table,
				a_ctrl->step_position_table,
				sizeof(uint16_t) * (search_point + 1))) {
				rc = -EFAULT;
				break;
			}
			{
				mutex_lock(&imx091_act_event_mutex);
				if(imx091_act_store_event & IMX091_ACT_EVENT_FW_DL) {
					rc = imx091_actuator_i2c_write(a_ctrl,a_ctrl->step_position_table[0], 0);
				}
 				else {
					CDBG("%s stored IMX091_ACT_EVENT_INIT_LENSPOS lenspos = %d\n", __func__, imx091_lens_pos);
					imx091_lens_pos = a_ctrl->step_position_table[0];
					imx091_act_store_event |= IMX091_ACT_EVENT_INIT_LENSPOS;
				}
				mutex_unlock(&imx091_act_event_mutex);
			}
		}
		break;
		case CFG_SH_ACTUATOR_INIT_LENSPOS:
#if 1
		{
			int16_t *lens;
			void *data;

			lens = &imx091_lens_pos;

			data = kmalloc(cdata.cfg.i2c_info.length, GFP_KERNEL);
			if(data == NULL){
				rc = -EFAULT;
				break;
			}
			if(copy_from_user(data,
				(void *)cdata.cfg.i2c_info.data,
				cdata.cfg.i2c_info.length)){
				LINFO("%s copy_to_user error\n",__func__);
				rc = -EFAULT;
				break;
			}

			memcpy(lens, data, cdata.cfg.i2c_info.length);
			LINFO("%s imx091_lens_pos : %d\n", __func__ , imx091_lens_pos);

			if(imx091_lens_pos >= imx091_act_t.initial_code && 
			   imx091_lens_pos <= imx091_act_t.terminal_code )
			{
				mutex_lock(&imx091_act_event_mutex);
				if(imx091_act_store_event & IMX091_ACT_EVENT_FW_DL) {
					imx091_actuator_i2c_write(a_ctrl, imx091_lens_pos, 0);
				}
 				else {
					CDBG("%s stored IMX091_ACT_EVENT_INIT_LENSPOS lenspos = %d\n", __func__, imx091_lens_pos);
					imx091_act_store_event |= IMX091_ACT_EVENT_INIT_LENSPOS;
				}
				mutex_unlock(&imx091_act_event_mutex);
			}

			kfree(data);
		}
#endif
		break;
	case CFG_SHDIAG_ACTUATOR_SET_SMEM:
		{
			void *data;
			unsigned char *wk_data;
			data = kmalloc(cdata.cfg.i2c_info.length, GFP_KERNEL);
			if(data == NULL){
				rc = -EFAULT;
				break;
			}
			if (copy_from_user(data,
				(void *)cdata.cfg.i2c_info.data,
				cdata.cfg.i2c_info.length)){
				kfree(data);
				CDBG("%s copy_to_user error\n",__func__);
				rc = -EFAULT;
				break;
			}
			CDBG("cdata.cfg.i2c_info.addr = 0x%x\n", cdata.cfg.i2c_info.addr);

			wk_data = (unsigned char *)data;
			if(cdata.cfg.i2c_info.addr == 0x00) {
				imx091_act_t.af_adjustflg = wk_data[0];
				if(imx091_act_t.af_adjustflg == 0x55){
					imx091_act_t.step_100_up =   ((wk_data[2] & 0x30) << 4) + wk_data[3];
					imx091_act_t.step_500_up =   ((wk_data[2] & 0x0c) << 6) + wk_data[4];
					imx091_act_t.step_shido_up =    ((wk_data[2] & 0x03) << 8) + wk_data[5];
					imx091_act_t.step_shido = ((wk_data[2] & 0xc0) << 2) + wk_data[6];
					CDBG("sh_camOtpData step_100_up=%d, step_500_up=%d, step_shido=%d, step_shido_up=%d\n",
					         imx091_act_t.step_100_up, imx091_act_t.step_500_up, imx091_act_t.step_shido, imx091_act_t.step_shido_up);
					CDBG("sh_camOtpData [0]=%02x, [2]=%02x, [3]=%02x, [4]=%02x [5]=%02x, [6]=%02x\n",
					         wk_data[0], wk_data[2], wk_data[3],
					         wk_data[4], wk_data[5], wk_data[6]);
				}
				else{
					CDBG("AF adjust value is not enable\n");
				}
			}
			else if(cdata.cfg.i2c_info.addr == 0x10) {
				imx091_ois_parameter.ois_adjustflg = wk_data[0];
				imx091_ois_parameter.ois_state = OIS_STATE_POWEROFF;
				if(imx091_ois_parameter.ois_adjustflg == 0x55){
					imx091_ois_parameter.curdat = (uint16_t)(wk_data[1]  << 8 | wk_data[2]);
					imx091_ois_parameter.halofs = (uint16_t)(wk_data[3]  << 8 | wk_data[4]);
					imx091_ois_parameter.pstxof = (uint16_t)(wk_data[5]  << 8 | wk_data[6]);
					imx091_ois_parameter.pstyof = (uint16_t)(wk_data[7]  << 8 | wk_data[8]);
					imx091_ois_parameter.hx_ofs = (int16_t )(wk_data[9]  << 8 | wk_data[10]);
					imx091_ois_parameter.hy_ofs = (int16_t )(wk_data[11] << 8 | wk_data[12]);
					imx091_ois_parameter.gx_ofs = (int16_t )(wk_data[13] << 8 | wk_data[14]);
					imx091_ois_parameter.gy_ofs = (int16_t )(wk_data[15] << 8 | wk_data[16]);
					imx091_ois_parameter.kgxhg  = (int16_t )(wk_data[17] << 8 | wk_data[18]);
					imx091_ois_parameter.kgyhg  = (int16_t )(wk_data[19] << 8 | wk_data[20]);
					imx091_ois_parameter.kgxg   = (uint16_t)(wk_data[21] << 8 | wk_data[22]);
					imx091_ois_parameter.kgyg   = (uint16_t)(wk_data[23] << 8 | wk_data[24]);
					CDBG("sh_camOtpData curdat=%d, halofs=%d, pstxof=%d, pstyof=%d, hx_ofs=%d, hy_ofs=%d, gx_ofs=%d, gy_ofs=%d, kgxhg=%d, kgyhg=%d, kgxg=%d, kgyg=%d\n"
					         ,imx091_ois_parameter.curdat, imx091_ois_parameter.halofs, imx091_ois_parameter.pstxof, imx091_ois_parameter.pstyof
					         ,imx091_ois_parameter.hx_ofs, imx091_ois_parameter.hy_ofs, imx091_ois_parameter.gx_ofs, imx091_ois_parameter.gy_ofs
					         ,imx091_ois_parameter.kgxhg,  imx091_ois_parameter.kgyhg,  imx091_ois_parameter.kgxg,   imx091_ois_parameter.kgyg);
				}
				else{
					CDBG("OIS adjust value is not enable\n");
				}
			}
			else {
				;
			}

		    kfree(data);
			if (copy_to_user((void *)argp,
				&cdata,
				sizeof(struct sensor_cfg_data))) {
				rc = -EFAULT;
				break;
			}
		}
		break;
	case CFG_SHDIAG_SET_OIS:
		{
			void *data;
			data = kmalloc(cdata.cfg.i2c_info.length, GFP_KERNEL);
			if(data == NULL){
				rc = -EFAULT;
				break;
			}
			if(copy_from_user(data,
				(void *)cdata.cfg.i2c_info.data,
				cdata.cfg.i2c_info.length)){
				LINFO("%s copy_to_user error\n",__func__);
				kfree(data);
				rc = -EFAULT;
				break;
			}
			memcpy(&imx091_ois_mode, data, cdata.cfg.i2c_info.length);

			mutex_lock(&imx091_act_event_mutex);
			if(imx091_act_store_event & IMX091_ACT_EVENT_FW_DL) {
				rc = imx091_actuator_set_ois(imx091_ois_mode);
				if (rc < 0){
					pr_err("imx091_actuator_set_ois faild\n");
					rc = -EFAULT;
				}
			}
			else {
				CDBG("%s stored IMX091_ACT_EVENT_SET_OIS mode = %d\n", __func__, imx091_ois_mode);
				imx091_act_store_event |= IMX091_ACT_EVENT_SET_OIS;
			}
			mutex_unlock(&imx091_act_event_mutex);
			kfree(data);
		}
		break;
	case CFG_GET_ACTUATOR_INFO:
		cdata.cfg.get_info.step_shido = imx091_act_t.step_shido;
		cdata.cfg.get_info.step_100_up = imx091_act_t.step_100_up;
		cdata.cfg.get_info.step_500_up = imx091_act_t.step_500_up;
		cdata.cfg.get_info.step_shido_up = imx091_act_t.step_shido_up;
		if (copy_to_user((void *)argp,
				 &cdata,
				 sizeof(struct msm_actuator_cfg_data)))
			rc = -EFAULT;
		break;
	case CFG_SET_ACTUATOR_INFO:
		rc = imx091_actuator_init(a_ctrl, &cdata.cfg.set_info);
		if (rc < 0)
			pr_err("%s init table failed %d\n", __func__, rc);
		break;

	case CFG_SET_DEFAULT_FOCUS:
		mutex_lock(&imx091_act_event_mutex);
		if(imx091_act_store_event & IMX091_ACT_EVENT_FW_DL) {
			rc = a_ctrl->func_tbl->actuator_set_default_focus(a_ctrl,
				&cdata.cfg.move);
			if (rc < 0)
				pr_err("%s move focus failed %d\n", __func__, rc);
		}
		else {
			CDBG("%s stored CFG_SET_DEFAULT_FOCUS\n", __func__);
		}
		mutex_unlock(&imx091_act_event_mutex);
		break;

	case CFG_MOVE_FOCUS:
		rc = imx091_actuator_powerup_seq(a_ctrl, &cdata);
		if (rc < 0)
			pr_err("%s actuator_power up failed %d\n", __func__, rc);
		rc = a_ctrl->func_tbl->actuator_move_focus(a_ctrl,
			&cdata.cfg.move);
		if (rc < 0)
			pr_err("%s move focus failed %d\n", __func__, rc);
		break;

	default:
		break;
	}
	mutex_unlock(a_ctrl->actuator_mutex);
	return rc;
}


int32_t imx091_actuator_i2c_probe(
	struct i2c_client *client,
	const struct i2c_device_id *id)
{
	int rc = 0;
	struct msm_actuator_ctrl_t *act_ctrl_t = NULL;
	sharp_smem_common_type *p_sh_smem_common_type = NULL;
	CDBG("%s called\n", __func__);

	if (!i2c_check_functionality(client->adapter, I2C_FUNC_I2C)) {
		pr_err("i2c_check_functionality failed\n");
		goto probe_failure;
	}

	act_ctrl_t = (struct msm_actuator_ctrl_t *)(id->driver_data);
	CDBG("%s client = %x\n",
		__func__, (unsigned int) client);
	act_ctrl_t->i2c_client.client = client;

	snprintf(act_ctrl_t->sdev.name, sizeof(act_ctrl_t->sdev.name),
			 "%s", act_ctrl_t->i2c_driver->driver.name);

	v4l2_i2c_subdev_init(&act_ctrl_t->sdev,
		act_ctrl_t->i2c_client.client,
		act_ctrl_t->act_v4l2_subdev_ops);

	p_sh_smem_common_type = sh_smem_get_common_address();

	if(p_sh_smem_common_type != NULL){
		
		imx091_act_t.af_adjustflg = p_sh_smem_common_type->sh_camOtpData[0];
		
		if(imx091_act_t.af_adjustflg == 0x55){

			imx091_act_t.step_100_up =   ((p_sh_smem_common_type->sh_camOtpData[2] & 0x30) << 4) + p_sh_smem_common_type->sh_camOtpData[3];
			imx091_act_t.step_500_up =   ((p_sh_smem_common_type->sh_camOtpData[2] & 0x0c) << 6) + p_sh_smem_common_type->sh_camOtpData[4];
			imx091_act_t.step_shido_up =    ((p_sh_smem_common_type->sh_camOtpData[2] & 0x03) << 8) + p_sh_smem_common_type->sh_camOtpData[5];
			imx091_act_t.step_shido = ((p_sh_smem_common_type->sh_camOtpData[2] & 0xc0) << 2) + p_sh_smem_common_type->sh_camOtpData[6];

			CDBG("sh_camOtpData step_100_up=%d, step_500_up=%d, step_shido=%d, step_shido_up=%d\n",
			         imx091_act_t.step_100_up, imx091_act_t.step_500_up, imx091_act_t.step_shido, imx091_act_t.step_shido_up);
			CDBG("sh_camOtpData [0]=%02x, [2]=%02x, [3]=%02x, [4]=%02x [5]=%02x, [6]=%02x\n",
			         p_sh_smem_common_type->sh_camOtpData[0], p_sh_smem_common_type->sh_camOtpData[2], p_sh_smem_common_type->sh_camOtpData[3],
			         p_sh_smem_common_type->sh_camOtpData[4], p_sh_smem_common_type->sh_camOtpData[5], p_sh_smem_common_type->sh_camOtpData[6]);
		}
		else{
			CDBG("AF adjust value is not enable\n");
		}
		
		imx091_ois_parameter.ois_adjustflg = p_sh_smem_common_type->sh_camOtpData[16];
		imx091_ois_parameter.ois_state = OIS_STATE_POWEROFF;
		
		if(imx091_ois_parameter.ois_adjustflg == 0x55){
			imx091_ois_parameter.curdat = (uint16_t)(p_sh_smem_common_type->sh_camOtpData[17] << 8 | p_sh_smem_common_type->sh_camOtpData[18]);
			imx091_ois_parameter.halofs = (uint16_t)(p_sh_smem_common_type->sh_camOtpData[19] << 8 | p_sh_smem_common_type->sh_camOtpData[20]);
			imx091_ois_parameter.pstxof = (uint16_t)(p_sh_smem_common_type->sh_camOtpData[21] << 8 | p_sh_smem_common_type->sh_camOtpData[22]);
			imx091_ois_parameter.pstyof = (uint16_t)(p_sh_smem_common_type->sh_camOtpData[23] << 8 | p_sh_smem_common_type->sh_camOtpData[24]);
			imx091_ois_parameter.hx_ofs = (int16_t )(p_sh_smem_common_type->sh_camOtpData[25] << 8 | p_sh_smem_common_type->sh_camOtpData[26]);
			imx091_ois_parameter.hy_ofs = (int16_t )(p_sh_smem_common_type->sh_camOtpData[27] << 8 | p_sh_smem_common_type->sh_camOtpData[28]);
			imx091_ois_parameter.gx_ofs = (int16_t )(p_sh_smem_common_type->sh_camOtpData[29] << 8 | p_sh_smem_common_type->sh_camOtpData[30]);
			imx091_ois_parameter.gy_ofs = (int16_t )(p_sh_smem_common_type->sh_camOtpData[31] << 8 | p_sh_smem_common_type->sh_camOtpData[32]);
			imx091_ois_parameter.kgxhg  = (int16_t )(p_sh_smem_common_type->sh_camOtpData[33] << 8 | p_sh_smem_common_type->sh_camOtpData[34]);
			imx091_ois_parameter.kgyhg  = (int16_t )(p_sh_smem_common_type->sh_camOtpData[35] << 8 | p_sh_smem_common_type->sh_camOtpData[36]);
			imx091_ois_parameter.kgxg   = (uint16_t)(p_sh_smem_common_type->sh_camOtpData[37] << 8 | p_sh_smem_common_type->sh_camOtpData[38]);
			imx091_ois_parameter.kgyg   = (uint16_t)(p_sh_smem_common_type->sh_camOtpData[39] << 8 | p_sh_smem_common_type->sh_camOtpData[40]);
			CDBG("sh_camOtpData curdat=%d, halofs=%d, pstxof=%d, pstyof=%d, hx_ofs=%d, hy_ofs=%d, gx_ofs=%d, gy_ofs=%d, kgxhg=%d, kgyhg=%d, kgxg=%d, kgyg=%d\n"
			         ,imx091_ois_parameter.curdat, imx091_ois_parameter.halofs, imx091_ois_parameter.pstxof, imx091_ois_parameter.pstyof
			         ,imx091_ois_parameter.hx_ofs, imx091_ois_parameter.hy_ofs, imx091_ois_parameter.gx_ofs, imx091_ois_parameter.gy_ofs
			         ,imx091_ois_parameter.kgxhg,  imx091_ois_parameter.kgyhg,  imx091_ois_parameter.kgxg,   imx091_ois_parameter.kgyg);
		}
		else{
			CDBG("OIS adjust value is not enable\n");
		}

		
	}else{
		CDBG("%s:%d:p_sh_smem_common_type != NULL!!!!\n", __func__, __LINE__);
	}

	CDBG("%s succeeded\n", __func__);
	return rc;

probe_failure:
	pr_err("%s failed! rc = %d\n", __func__, rc);
	return rc;

}

int32_t imx091_actuator_power_up(struct msm_actuator_ctrl_t *a_ctrl)
{
	struct i2c_msg msg[2];
	int32_t rc = 0;
	uint16_t saddr;
	int i;

	CDBG("%s: %d\n", __func__, __LINE__);

	msg[0].flags = 0;
	msg[0].len = ARRAY_SIZE(imx091_act_init_fw_settings1);
	saddr = imx091_act_t.i2c_client.client->addr >> 1;
	msg[0].addr = saddr;
	msg[0].buf =&imx091_act_init_fw_settings1[0];
	rc = i2c_transfer(imx091_act_t.i2c_client.client->adapter, msg, 1);
	if (rc < 0){
		LINFO("msm_camera_i2c_txdata faild 0x%x\n", saddr);
		return -EFAULT;
	}

	msleep(10);

	msg[0].flags = 0;
	msg[0].len = ARRAY_SIZE(imx091_act_init_fw_settings2);
	saddr = imx091_act_t.i2c_client.client->addr >> 1;
	msg[0].addr = saddr;
	msg[0].buf =&imx091_act_init_fw_settings2[0];
	rc = i2c_transfer(imx091_act_t.i2c_client.client->adapter, msg, 1);
	if (rc < 0){
		LINFO("msm_camera_i2c_txdata faild 0x%x\n", saddr);
		return -EFAULT;
	}

	msleep(10);

	msg[0].flags = 0;
	msg[0].len = ARRAY_SIZE(imx091_act_init_fw_settings3);
	saddr = imx091_act_t.i2c_client.client->addr >> 1;
	msg[0].addr = saddr;
	msg[0].buf =&imx091_act_init_fw_settings3[0];
	rc = i2c_transfer(imx091_act_t.i2c_client.client->adapter, msg, 1);
	if (rc < 0){
		LINFO("msm_camera_i2c_txdata faild 0x%x\n", saddr);
		return -EFAULT;
	}

	msleep(10);

	msg[0].flags = 0;
	msg[0].len = ARRAY_SIZE(imx091_act_init_fw_settings4);
	saddr = imx091_act_t.i2c_client.client->addr >> 1;
	msg[0].addr = saddr;
	msg[0].buf =&imx091_act_init_fw_settings4[0];
	rc = i2c_transfer(imx091_act_t.i2c_client.client->adapter, msg, 1);
	if (rc < 0){
		LINFO("msm_camera_i2c_txdata faild 0x%x\n", saddr);
		return -EFAULT;
	}

	msleep(10);

	msg[0].flags = 0;
	msg[0].len = ARRAY_SIZE(imx091_act_init_fw_settings5);
	saddr = imx091_act_t.i2c_client.client->addr >> 1;
	msg[0].addr = saddr;
	msg[0].buf =&imx091_act_init_fw_settings5[0];
	rc = i2c_transfer(imx091_act_t.i2c_client.client->adapter, msg, 1);
	if (rc < 0){
		LINFO("msm_camera_i2c_txdata faild 0x%x\n", saddr);
		return -EFAULT;
	}

	msleep(10);

	msg[0].flags = 0;
	msg[0].len = ARRAY_SIZE(imx091_act_init_fw_settings6);
	saddr = imx091_act_t.i2c_client.client->addr >> 1;
	msg[0].addr = saddr;
	msg[0].buf =&imx091_act_init_fw_settings6[0];
	rc = i2c_transfer(imx091_act_t.i2c_client.client->adapter, msg, 1);
	if (rc < 0){
		LINFO("msm_camera_i2c_txdata faild 0x%x\n", saddr);
		return -EFAULT;
	}

	msleep(10);

	CDBG("%s: %d\n", __func__, __LINE__);
	msg[0].flags = 0;
	msg[0].len = ARRAY_SIZE(imx091_act_init_mem_settings);
	msg[0].addr = saddr;
	msg[0].buf =&imx091_act_init_mem_settings[0];
	rc = i2c_transfer(imx091_act_t.i2c_client.client->adapter, msg, 1);
	if (rc < 0){
		LINFO("msm_camera_i2c_txdata faild 0x%x\n", saddr);
		return -EFAULT;
	}

	msleep(10);

	for(i = 0; i < ARRAY_SIZE(ois_init_setting1); i++) {
		rc = imx091_actuator_write_ois(ois_init_setting1[i].type,
			ois_init_setting1[i].addr, ois_init_setting1[i].data);
		if (rc < 0){
			LINFO("msm_camera_i2c_txdata faild No.%d 0x%x\n", i, saddr);
			return -EFAULT;
		}
	}

	CDBG("%s: %d\n", __func__, __LINE__);

	return 0;
}

DEFINE_MUTEX(imx091_act_mutex);

static const struct i2c_device_id imx091_act_i2c_id[] = {
	{"imx091_act", (kernel_ulong_t)&imx091_act_t},
	{ }
};
static struct i2c_driver imx091_act_i2c_driver = {
	.id_table = imx091_act_i2c_id,
	.probe  = imx091_actuator_i2c_probe,
	.remove = __exit_p(imx091_act_i2c_remove),
	.driver = {
		.name = "imx091_act",
	},
};

static int __init imx091_act_i2c_add_driver(
	void)
{
	CDBG("%s called\n", __func__);
	return i2c_add_driver(imx091_act_t.i2c_driver);
}

long imx091_actuator_subdev_ioctl(struct v4l2_subdev *sd,
			unsigned int cmd, void *arg)
{
	struct msm_actuator_ctrl_t *a_ctrl = get_actrl(sd);
	void __user *argp = (void __user *)arg;
	switch (cmd) {
	case VIDIOC_MSM_ACTUATOR_CFG:
		return imx091_actuator_config(a_ctrl, argp);
	default:
		return -ENOIOCTLCMD;
	}
}

int32_t imx091_actuator_power(struct v4l2_subdev *sd, int on)
{
	int rc = 0;
	struct msm_actuator_ctrl_t *a_ctrl = get_actrl(sd);
	mutex_lock(a_ctrl->actuator_mutex);
	if (on)
	{
		CDBG("%s imx091_actuator_power mutex lock! \n", __func__);
		mutex_lock(&imx091_act_event_mutex);
		imx091_act_store_event = 0;
		mutex_unlock(&imx091_act_event_mutex);
	}
	else
		rc = imx091_actuator_power_down(a_ctrl);
	mutex_unlock(a_ctrl->actuator_mutex);
	return rc;
}

static struct v4l2_subdev_core_ops imx091_act_subdev_core_ops= {
	.ioctl = imx091_actuator_subdev_ioctl,
	.s_power = imx091_actuator_power,
};

static struct v4l2_subdev_ops imx091_act_subdev_ops = {
	.core = &imx091_act_subdev_core_ops,
};

static struct msm_actuator_ctrl_t imx091_act_t = {
	.i2c_driver = &imx091_act_i2c_driver,
	.act_v4l2_subdev_ops = &imx091_act_subdev_ops,

	.i2c_client = {
		.addr_type = MSM_CAMERA_I2C_BYTE_ADDR,
	},
	.total_steps = IMX091_TOTAL_STEPS_NEAR_TO_FAR_MAX,
	.step_shido    = 0,
	.step_100_up   = 0,
	.step_500_up   = 0,
	.step_shido_up = 0,
	.curr_step_pos = 0,
	.curr_region_index = 0,
	.initial_code = 0,
	.terminal_code = 0,
	.actuator_mutex = &imx091_act_mutex,

};

subsys_initcall(imx091_act_i2c_add_driver);
MODULE_DESCRIPTION("IMX091 actuator");
MODULE_LICENSE("GPL v2");
