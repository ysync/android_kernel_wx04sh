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

#include "msm_sensor.h"
#include "msm.h"
#include "msm_ispif.h"
#include <sharp/sh_smem.h>
#define SENSOR_NAME "imx111"
#define PLATFORM_DRIVER_NAME "msm_camera_imx111"
#define imx111_obj imx111_##obj
#define imx111_REC_SETTINGS

#if defined(CONFIG_MACH_MNB) 
static struct camera_vreg_t msm_8960_imx111_vreg_1[] = {
	{"cam_vaf", REG_LDO, 3000000, 3000000, 300000},
};
struct regulator **imx111_reg_ptr;
#endif /* CONFIG_MACH_MNB */

//the issue of white blink when camera starts.
#define P_INIT_LINE 1902  //Bv3.5
#define P_INIT_GAIN ((uint16_t)(256 - 256/ 1.23 +0.5)) //reg = 256 - 256/gain
#define P_INIT_FLEN 0x090A  //2314
#define P_INIT_DGAIN 0x0100  //256
#define PIL_H ((0xFF00 & P_INIT_LINE) >> 8)
#define PIL_L (0x00FF & P_INIT_LINE)
#define PIG_H ((0xFF00 & P_INIT_GAIN) >> 8)
#define PIG_L (0x00FF & P_INIT_GAIN)
#define PIF_H ((0xFF00 & P_INIT_FLEN) >> 8)
#define PIF_L (0x00FF & P_INIT_FLEN)

#define FHD_INIT_LINE 951 //Bv3.5
#define FHD_INIT_GAIN ((uint16_t)(256 - 256/ 1.23 +0.5)) //reg = 256 - 256/gain
#define FHD_INIT_FLEN 0x4B2  //1202 (30.03fps)
//#define FHD_INIT_FLEN 0x04A0  //1184 is constraint violation!!

#define IMX111_SENSOR_MCLK_6P75HZ  6750000

DEFINE_MUTEX(imx111_mut);
static struct msm_sensor_ctrl_t imx111_s_ctrl;
static uint8_t imx111_curr_mode = 10;
static int imx111_ae_lock=0;
#define IMX111_EEPROM_BANK_SEL_REG 0x34C9

static struct msm_camera_i2c_reg_conf imx111_start_settings[] = {
	{0x0100, 0x01},
};

static struct msm_camera_i2c_reg_conf imx111_stop_settings[] = {
	{0x0100, 0x00},
};

static struct msm_camera_i2c_reg_conf imx111_groupon_settings[] = {
	{0x104, 0x01},
};

static struct msm_camera_i2c_reg_conf imx111_groupoff_settings[] = {
	{0x104, 0x00},
};

static struct msm_camera_i2c_reg_conf imx111_prev_settings[] = {

#ifdef imx111_REC_SETTINGS

	{0x0307, 0x60}, /*PLL Multiplier-K*/
#if 0
	{0x0340, 0x04}, /*FLL*/
	{0x0341, 0xFC}, /*FLL*/
#endif
	{0x0342, 0x07},
	{0x0343, 0x2C},

#else
	{0x0307, 0x60}, /*PLL Multiplier-D*/
#if 0
	{0x0340, 0x04}, /*FLL*/
	{0x0341, 0xFC}, /*FLL*/
#endif
	{0x0342, 0x0D},
	{0x0343, 0x74},

#endif
	{0x0202,  PIL_H},
	{0x0203,  PIL_L},
	{0x0204,  PIG_H},
	{0x0205,  PIG_L},
	{0x0340,  PIF_H},
	{0x0341,  PIF_L},
	{0x0305, 0x01},
	{0x30A4, 0x02},
	{0x303C, 0x15},
	{0x0344, 0x00},
	{0x0345, 0x08},
	{0x0346, 0x00},
	{0x0347, 0x30},
	{0x0348, 0x0C},
	{0x0349, 0xD7},
	{0x034A, 0x09},
	{0x034B, 0xCF},
	{0x034C, 0x06},
	{0x034D, 0x68},
	{0x034E, 0x04},
	{0x034F, 0xD0},
	{0x0381, 0x01},
	{0x0383, 0x01},
	{0x0385, 0x01},
	{0x0387, 0x03},
	{0x3033, 0x87},
	{0x303D, 0x10},
	{0x303E, 0x40},
	{0x3048, 0x01},
	{0x304C, 0xB7},
	{0x304D, 0x01},
	{0x309B, 0x28},
	{0x30A1, 0x09},
	{0x30AA, 0x00},
	{0x30B2, 0x05},
	{0x30D5, 0x04},
	{0x30D6, 0x85},
	{0x30D7, 0x2A},
	{0x30DE, 0x00},
	{0x30DF, 0x20},
	{0x3102, 0x08},
	{0x3103, 0x22},
	{0x3104, 0x20},
	{0x3105, 0x00},
	{0x3106, 0x87},
	{0x3107, 0x00},
	{0x3108, 0x03},
	{0x3109, 0x02},
	{0x310A, 0x03},
	{0x315C, 0x3A},
	{0x315D, 0x39},
	{0x316E, 0x3B},
	{0x316F, 0x3A},
	{0x3301, 0x00},
	{0x3318, 0x63},
};


static struct msm_camera_i2c_reg_conf imx111_video_VGA_120fps_settings[] = {

#ifdef imx111_REC_SETTINGS

	{0x0307, 0x5C}, /*PLL Multiplier-K*/
	{0x0340, 0x02}, /*FLL*/
	{0x0341, 0x4C}, /*FLL*/

#else
    {0x0307, 0x60}, /*PLL Multiplier-D*/
	{0x0340, 0x02}, /*FLL*/
	{0x0341, 0x64}, /*FLL*/

#endif
	{0x0305, 0x01},
	{0x30A4, 0x02},
	{0x303C, 0x15},
	{0x0342, 0x06},
	{0x0343, 0xE0},
	{0x0344, 0x00},
	{0x0345, 0x08},
	{0x0346, 0x00},
	{0x0347, 0xA0},
	{0x0348, 0x0C},
	{0x0349, 0xD7},
	{0x034A, 0x09},
	{0x034B, 0x5F},
	{0x034C, 0x06},
	{0x034D, 0x68},
	{0x034E, 0x02},
	{0x034F, 0x30},
	{0x0381, 0x01},
	{0x0383, 0x01},
	{0x0385, 0x05},
	{0x0387, 0x03},
	{0x3033, 0x87},
	{0x303D, 0x10},
	{0x303E, 0x40},
	{0x3048, 0x01},
	{0x304C, 0xB7},
	{0x304D, 0x01},
	{0x309B, 0x28},
	{0x30A1, 0x09},
	{0x30AA, 0x00},
	{0x30B2, 0x03},
	{0x30D5, 0x04},
	{0x30D6, 0x85},
	{0x30D7, 0x2A},
	{0x30DE, 0x00},
	{0x30DF, 0x20},
	{0x3102, 0x08},
	{0x3103, 0x22},
	{0x3104, 0x20},
	{0x3105, 0x00},
	{0x3106, 0x87},
	{0x3107, 0x00},
	{0x3108, 0x03},
	{0x3109, 0x02},
	{0x310A, 0x03},
	{0x315C, 0x3A},
	{0x315D, 0x39},
	{0x316E, 0x3B},
	{0x316F, 0x3A},
	{0x3301, 0x00},
	{0x3318, 0x63},
};

static struct msm_camera_i2c_reg_conf imx111_video_1080P_30fps_settings[] = {

#ifdef imx111_REC_SETTINGS

	{0x0307, 0x60}, /*PLL Multiplier-K*/
	{0x0340, 0x04}, /*FLL*/
	{0x0341, 0xB2}, /*FLL*/

#else
    {0x0307, 0x60}, /*PLL Multiplier-D*/
	{0x0340, 0x04}, /*FLL*/
	{0x0341, 0xE6}, /*FLL*/

#endif
	{0x0305, 0x01},
	{0x30A4, 0x02},
	{0x303C, 0x15},
	{0x0342, 0x0E},
	{0x0343, 0x06},
	{0x0344, 0x02},
	{0x0345, 0x36},
	{0x0346, 0x02},
	{0x0347, 0xBA},
	{0x0348, 0x0A},
	{0x0349, 0xA9},
	{0x034A, 0x07},
	{0x034B, 0x45},
	{0x034C, 0x08},
	{0x034D, 0x74},
	{0x034E, 0x04},
	{0x034F, 0x8C},
	{0x0381, 0x01},
	{0x0383, 0x01},
	{0x0385, 0x01},
	{0x0387, 0x01},
	{0x3033, 0x00},
	{0x303D, 0x10},
	{0x303E, 0x40},
	{0x3048, 0x00},
	{0x304C, 0x57},
	{0x304D, 0x03},
	{0x309B, 0x20},
	{0x30A1, 0x08},
	{0x30AA, 0x02},
	{0x30B2, 0x07},
	{0x30D5, 0x00},
	{0x30D6, 0x85},
	{0x30D7, 0x2A},
	{0x30DE, 0x00},
	{0x30DF, 0x20},
	{0x3102, 0x08},
	{0x3103, 0x22},
	{0x3104, 0x20},
	{0x3105, 0x00},
	{0x3106, 0x87},
	{0x3107, 0x00},
	{0x3108, 0x03},
	{0x3109, 0x02},
	{0x310A, 0x03},
	{0x315C, 0x9D},
	{0x315D, 0x9C},
	{0x316E, 0x9E},
	{0x316F, 0x9D},
	{0x3301, 0x00},
	{0x3318, 0x63},
};

static struct msm_camera_i2c_reg_conf imx111_video_60fps_settings[] = {

#ifdef imx111_REC_SETTINGS

	{0x0307, 0x5C}, /*PLL Multiplier-K*/
	{0x0340, 0x04}, /*FLL*/
	{0x0341, 0x98}, /*FLL*/

#else
    {0x0307, 0x60}, /*PLL Multiplier-D*/
	{0x0340, 0x04}, /*FLL*/
	{0x0341, 0xCA}, /*FLL*/

#endif
	/* 60 fps */
	{0x0305, 0x01},
	{0x30A4, 0x02},
	{0x303C, 0x15},
	{0x0342, 0x06},
	{0x0343, 0xE0},
	{0x0344, 0x00},
	{0x0345, 0x08},
	{0x0346, 0x00},
	{0x0347, 0x88},
	{0x0348, 0x0C},
	{0x0349, 0xD7},
	{0x034A, 0x09},
	{0x034B, 0x77},
	{0x034C, 0x06},
	{0x034D, 0x68},
	{0x034E, 0x04},
	{0x034F, 0x78},
	{0x0381, 0x01},
	{0x0383, 0x01},
	{0x0385, 0x01},
	{0x0387, 0x03},
	{0x3033, 0x87},
	{0x303D, 0x10},
	{0x303E, 0x40},
	{0x3048, 0x01},
	{0x304C, 0xB7},
	{0x304D, 0x01},
	{0x309B, 0x28},
	{0x30A1, 0x09},
	{0x30AA, 0x00},
	{0x30B2, 0x05},
	{0x30D5, 0x04},
	{0x30D6, 0x85},
	{0x30D7, 0x2A},
	{0x30DE, 0x00},
	{0x30DF, 0x20},
	{0x3102, 0x08},
	{0x3103, 0x22},
	{0x3104, 0x20},
	{0x3105, 0x00},
	{0x3106, 0x87},
	{0x3107, 0x00},
	{0x3108, 0x03},
	{0x3109, 0x02},
	{0x310A, 0x03},
	{0x315C, 0x3A},
	{0x315D, 0x39},
	{0x316E, 0x3B},
	{0x316F, 0x3A},
	{0x3301, 0x00},
	{0x3318, 0x63},
};


static struct msm_camera_i2c_reg_conf imx111_video_90fps_settings[] = {
	/* 90 fps */
	{0x0305, 0x01},
	{0x0307, 0x60},
	{0x30A4, 0x02},
	{0x303C, 0x15},
	{0x0340, 0x03},
	{0x0341, 0x32},
	{0x0342, 0x06},
	{0x0343, 0xE0},
	{0x0344, 0x00},
	{0x0345, 0x08},
	{0x0346, 0x00},
	{0x0347, 0x88},
	{0x0348, 0x0C},
	{0x0349, 0xD7},
	{0x034A, 0x09},
	{0x034B, 0x77},
	{0x034C, 0x06},
	{0x034D, 0x68},
	{0x034E, 0x04},
	{0x034F, 0x78},
	{0x0381, 0x01},
	{0x0383, 0x01},
	{0x0385, 0x01},
	{0x0387, 0x03},
	{0x3033, 0x87},
	{0x303D, 0x10},
	{0x303E, 0x40},
	{0x3048, 0x01},
	{0x304C, 0xB7},
	{0x304D, 0x01},
	{0x309B, 0x28},
	{0x30A1, 0x09},
	{0x30AA, 0x00},
	{0x30B2, 0x05},
	{0x30D5, 0x04},
	{0x30D6, 0x85},
	{0x30D7, 0x2A},
	{0x30DE, 0x00},
	{0x30DF, 0x20},
	{0x3102, 0x08},
	{0x3103, 0x22},
	{0x3104, 0x20},
	{0x3105, 0x00},
	{0x3106, 0x87},
	{0x3107, 0x00},
	{0x3108, 0x03},
	{0x3109, 0x02},
	{0x310A, 0x03},
	{0x315C, 0x3A},
	{0x315D, 0x39},
	{0x316E, 0x3B},
	{0x316F, 0x3A},
	{0x3301, 0x00},
	{0x3318, 0x63},
};

static struct msm_camera_i2c_reg_conf imx111_snap_settings[] = {

#ifdef imx111_REC_SETTINGS

	{0x0307, 0x60}, /*PLL Multiplier*/
	{0x0340, 0x09}, /*FLL*/
	{0x0341, 0xC8}, /*FLL*/

#else
    {0x0307, 0x60}, /*PLL Multiplier*/
	{0x0340, 0x0A}, /*FLL*/
	{0x0341, 0x34}, /*FLL*/

#endif
	/* 14.5 fps */
	{0x0305, 0x01},
	{0x30A4, 0x02},
	{0x303C, 0x15},
	{0x0342, 0x0E},
	{0x0343, 0x06},
	{0x0344, 0x00},
	{0x0345, 0x08},
	{0x0346, 0x00},
	{0x0347, 0x30},
	{0x0348, 0x0C},
	{0x0349, 0xD7},
	{0x034A, 0x09},
	{0x034B, 0xCF},
	{0x034C, 0x0C},
	{0x034D, 0xD0},
	{0x034E, 0x09},
	{0x034F, 0xA0},
	{0x0381, 0x01},
	{0x0383, 0x01},
	{0x0385, 0x01},
	{0x0387, 0x01},
	{0x3033, 0x00},
	{0x303D, 0x10},
	{0x303E, 0x40},
	{0x3048, 0x00},
	{0x304C, 0x57},
	{0x304D, 0x03},
	{0x309B, 0x20},
	{0x30A1, 0x08},
	{0x30AA, 0x02},
	{0x30B2, 0x07},
	{0x30D5, 0x00},
	{0x30D6, 0x85},
	{0x30D7, 0x2A},
	{0x30DE, 0x00},
	{0x30DF, 0x20},
	{0x3102, 0x08},
	{0x3103, 0x22},
	{0x3104, 0x20},
	{0x3105, 0x00},
	{0x3106, 0x87},
	{0x3107, 0x00},
	{0x3108, 0x03},
	{0x3109, 0x02},
	{0x310A, 0x03},
	{0x315C, 0x9D},
	{0x315D, 0x9C},
	{0x316E, 0x9E},
	{0x316F, 0x9D},
	{0x3301, 0x00},
	{0x3318, 0x63},

};

static struct msm_camera_i2c_reg_conf imx111_recommend_settings[] = {
	{0x3080,0x50},
	{0x3087,0x53},
	{0x309D,0x94},
	{0x30C6,0x00},
	{0x30C7,0x00},
	{0x3115,0x0B},
	{0x3118,0x30},
	{0x311D,0x25},
	{0x3121,0x0A},
	{0x3212,0xF2},
	{0x3213,0x0F},
	{0x3215,0x0F},
	{0x3217,0x0B},
	{0x3219,0x0B},
	{0x321B,0x0D},
	{0x321D,0x0D},
	{0x32AA,0x11},
	/* black level setting */
	{0x3032, 0x40},
};

static struct msm_camera_i2c_reg_conf imx111_comm1_settings[] = {
	{0x3035, 0x10},
	{0x303B, 0x14},
	{0x3312, 0x45},
	{0x3313, 0xC0},
	{0x3310, 0x20},
	{0x3310, 0x00},
	{0x303B, 0x04},
	{0x303D, 0x00},
	{0x0100, 0x10},
	{0x3035, 0x00},
};

static struct msm_camera_i2c_reg_conf imx111_comm2_part1_settings[] = {
	{0x0340, 0x0A},
	{0x0341, 0x34},
	{0x0342, 0x0D},
	{0x0343, 0x70},
	{0x034B, 0xCF},
	{0x034C, 0x0C},
	{0x034D, 0xD0},
	{0x034E, 0x09},
	{0x034F, 0xA0},
	{0x0387, 0x01},
	{0x3033, 0x00},
	{0x3048, 0x00},
	{0x304C, 0x57},
	{0x304D, 0x03},
	{0x309B, 0x20},
	{0x30A1, 0x08},
	{0x30AA, 0x02},
	{0x30B2, 0x07},
	{0x30D5, 0x00},
	{0x315C, 0x9D},
	{0x315D, 0x9C},
	{0x316E, 0x9E},
	{0x316F, 0x9D},

};

static struct msm_camera_i2c_reg_conf imx111_comm2_part2_settings[] = {
	{0x30B1, 0x43},
	/*{0x3311, 0x80},
	{0x3311, 0x00},*/
};

static struct msm_camera_i2c_conf_array imx111_comm_confs[] = {
	{&imx111_comm1_settings[0],
	ARRAY_SIZE(imx111_comm1_settings), 0, MSM_CAMERA_I2C_BYTE_DATA},
	{&imx111_comm2_part1_settings[0],
	ARRAY_SIZE(imx111_comm2_part1_settings), 0, MSM_CAMERA_I2C_BYTE_DATA},
	{&imx111_comm2_part2_settings[0],
	ARRAY_SIZE(imx111_comm2_part2_settings), 0, MSM_CAMERA_I2C_BYTE_DATA},
};

static struct v4l2_subdev_info imx111_subdev_info[] = {
	{
	.code   = V4L2_MBUS_FMT_SBGGR10_1X10,
	.colorspace = V4L2_COLORSPACE_JPEG,
	.fmt    = 1,
	.order    = 0,
	},
	/* more can be supported, to be added later */
};

static struct msm_camera_i2c_conf_array imx111_init_conf[] = {
	{&imx111_recommend_settings[0],
	ARRAY_SIZE(imx111_recommend_settings), 0, MSM_CAMERA_I2C_BYTE_DATA}
};

static struct msm_camera_i2c_conf_array imx111_confs[] = {
    {&imx111_snap_settings[0],
	ARRAY_SIZE(imx111_snap_settings), 0, MSM_CAMERA_I2C_BYTE_DATA},
	{&imx111_prev_settings[0],
	ARRAY_SIZE(imx111_prev_settings), 0, MSM_CAMERA_I2C_BYTE_DATA},
	{&imx111_video_60fps_settings[0],
	ARRAY_SIZE(imx111_video_60fps_settings), 0, MSM_CAMERA_I2C_BYTE_DATA},
	{&imx111_video_90fps_settings[0],
	ARRAY_SIZE(imx111_video_90fps_settings), 0, MSM_CAMERA_I2C_BYTE_DATA},
	{&imx111_video_VGA_120fps_settings[0],
	ARRAY_SIZE(imx111_video_VGA_120fps_settings), 0, MSM_CAMERA_I2C_BYTE_DATA},
	{&imx111_video_1080P_30fps_settings[0],
	ARRAY_SIZE(imx111_video_1080P_30fps_settings), 0, MSM_CAMERA_I2C_BYTE_DATA},
	{&imx111_snap_settings[0],
	ARRAY_SIZE(imx111_snap_settings), 0, MSM_CAMERA_I2C_BYTE_DATA},
	
};

static struct msm_sensor_output_info_t imx111_dimensions[] = {
	{
		/* 14.41 fps */
		.x_output = 0x0CD0, /* 3280 */
		.y_output = 0x9A0, /* 2464 */
		.line_length_pclk = 0xE06, /* 3590 */
#ifdef imx111_REC_SETTINGS
        .frame_length_lines = 0x9C8, /* 2504 */
		.vt_pixel_clk = 129600000,
		.op_pixel_clk = 129600000,
#else
		.frame_length_lines = 0xA34, /* 2612 */
		.vt_pixel_clk = 129600000,
		.op_pixel_clk = 129600000,
#endif
		.binning_factor = 1,
	},
	{
		/* 30.5 fps preview */
		.x_output = 0x668, /* 1640 */
		.y_output = 0x4D0, /* 1232 */
		
#ifdef imx111_REC_SETTINGS
		.line_length_pclk = 0x72C, /* 1836 */
/* SHLOCAL_CAMERA_DRIVER_IQ -> */ 
    	.frame_length_lines = P_INIT_FLEN, // 2314
/* SHLOCAL_CAMERA_DRIVER_IQ <- */ 
		.vt_pixel_clk = 129600000,
		.op_pixel_clk = 129600000,
#else
		.line_length_pclk = 0xD74, /* 3444 */
		.frame_length_lines = 0x4FC, /* 1276 */
		.vt_pixel_clk = 129600000,
		.op_pixel_clk = 129600000,
#endif
		.binning_factor = 1,
	},
	{
		/* 720p 60  fps video */
		.x_output = 0x668, /* 1640 */
		.y_output = 0x478, /* 1144 */
		.line_length_pclk = 0x6E0, /* 1760*/

#ifdef imx111_REC_SETTINGS
        .frame_length_lines = 0x498, /* 1176*/
		.vt_pixel_clk = 124200000,
		.op_pixel_clk = 124200000,
#else
		.frame_length_lines = 0x4CA, /* 1226 */
		.vt_pixel_clk = 129600000,
		.op_pixel_clk = 129600000,
#endif
		.binning_factor = 1,
	},
	{
		/* 720p 90  fps video */
		.x_output = 0x668, /* 1640 */
		.y_output = 0x478, /* 1144 */
		.line_length_pclk = 0x6E0, /* 1760*/
		.frame_length_lines = 0x332, /* 818 */
		.vt_pixel_clk = 129600000,
		.op_pixel_clk = 129600000,
		.binning_factor = 1,
	},
	{
		/* VGA 120  fps video */
		.x_output = 0x668, /* 1640 */
		.y_output = 0x230, /* 560 */
		.line_length_pclk = 0x6E0, /* 1760*/

#ifdef imx111_REC_SETTINGS
        .frame_length_lines = 0x24C, /* 588 */
		.vt_pixel_clk = 124200000,
		.op_pixel_clk = 124200000,
#else
		.frame_length_lines = 0x264, /* 612 */
		.vt_pixel_clk = 129600000,
		.op_pixel_clk = 129600000,
#endif
		.binning_factor = 1,
	},
	{
		/* 1080p 30  fps video */
		.x_output = 0x874, /* 2164 */
		.y_output = 0x48C, /* 1164 */
		.line_length_pclk = 0xE06, /* 3590*/

#ifdef imx111_REC_SETTINGS
        .frame_length_lines = FHD_INIT_FLEN, /* 1184 */
		.vt_pixel_clk = 129600000,
		.op_pixel_clk = 129600000,
#else
		.frame_length_lines = 0x4E6, /* 1254 */
		.vt_pixel_clk = 129600000,
		.op_pixel_clk = 129600000,
#endif
		.binning_factor = 1,
	},

	{
		/* 14.42 fps */
		.x_output = 0x0CD0, /* 3280 */
		.y_output = 0x9A0, /* 2464 */
		.line_length_pclk = 0xE06, /* 3590 */

#ifdef imx111_REC_SETTINGS
        .frame_length_lines = 0x9C8, /* 2504 */
		.vt_pixel_clk = 129600000,
		.op_pixel_clk = 129600000,
#else
		.frame_length_lines = 0xA34, /* 2612 */
		.vt_pixel_clk = 129600000,
		.op_pixel_clk = 129600000,
#endif
		.binning_factor = 1,
	},
	
};

static struct msm_camera_csid_vc_cfg imx111_cid_cfg[] = {
	{0, CSI_RAW10, CSI_DECODE_10BIT},
	{1, CSI_EMBED_DATA, CSI_DECODE_8BIT},
//	{2, CSI_RESERVED_DATA, CSI_DECODE_8BIT},
	{2, CSI_RESERVED_DATA_0, CSI_DECODE_8BIT},
};

static struct msm_camera_csi2_params imx111_csi_params = {
	.csid_params = {
		.lane_assign = 0xe4,
		.lane_cnt = 2,
		.lut_params = {
			.num_cid = 3,
			.vc_cfg = imx111_cid_cfg,
		},
	},
	.csiphy_params = {
		.lane_cnt = 2,
		.settle_cnt = 0x14,
	},
};

static struct msm_camera_csi2_params *imx111_csi_params_array[] = {
	&imx111_csi_params,
	&imx111_csi_params,
	&imx111_csi_params,
	&imx111_csi_params,
	&imx111_csi_params,
	&imx111_csi_params,
	&imx111_csi_params,
	
};

static struct msm_sensor_output_reg_addr_t imx111_reg_addr = {
	.x_output = 0x34C,
	.y_output = 0x34E,
	.line_length_pclk = 0x342,
	.frame_length_lines = 0x340,
};

static struct msm_sensor_id_info_t imx111_id_info = {
	.sensor_id_reg_addr = 0x0,
	.sensor_id = 0x0111,
};

static struct msm_sensor_exp_gain_info_t imx111_exp_gain_info = {
	.coarse_int_time_addr = 0x202,
	.global_gain_addr = 0x204,
	.vert_offset = 5,
};

static const struct i2c_device_id imx111_i2c_id[] = {
	{SENSOR_NAME, (kernel_ulong_t)&imx111_s_ctrl},
	{ }
};

unsigned char *p_imx111_shdiag_data = NULL;

typedef struct {
	uint32_t fl_lines;
	uint32_t line;
	uint16_t gain;
	uint16_t digital_gain;
} imx111_exp_gain_t;

imx111_exp_gain_t imx111_exp_gain[MSM_SENSOR_INVALID_RES] ={
	{ 0x09C8, 0x0028, 0x0005, 0x0100},
	{ P_INIT_FLEN, P_INIT_LINE, P_INIT_GAIN, 0x0100},
	{ 0x0498, 0x0041, 0x0003, 0x0100},
	{ 0x0332, 0x0041, 0x0003, 0x0100},
	{ 0x024C, 0x0041, 0x0003, 0x0100},
	{ FHD_INIT_FLEN, FHD_INIT_LINE, FHD_INIT_GAIN, 0x0100},
	{ 0x09C8, 0x0028, 0x0005, 0x0100},
};

int32_t imx111_sensor_i2c_probe(struct i2c_client *client,
	const struct i2c_device_id *id)
{
	int i;
	sharp_smem_common_type *p_sh_smem_common_type = NULL;
	CDBG("CFG_SHDIAG_GET_OTP_DATA IN\n");
	p_sh_smem_common_type = sh_smem_get_common_address();

	CDBG("%s shdiag_data_size = %d\n", __func__, sizeof(p_sh_smem_common_type->sh_camOtpData));
	for(i = 0; i < sizeof(p_sh_smem_common_type->sh_camOtpData) ;i+=16){
		CDBG("p_sh_smem_common_type->sh_camOtpData[%d] = %02x %02x %02x %02x %02x %02x %02x %02x %02x %02x %02x %02x %02x %02x %02x %02x", i, 
																p_sh_smem_common_type->sh_camOtpData[i],
																p_sh_smem_common_type->sh_camOtpData[i+1],
																p_sh_smem_common_type->sh_camOtpData[i+2],
																p_sh_smem_common_type->sh_camOtpData[i+3],
																p_sh_smem_common_type->sh_camOtpData[i+4],
																p_sh_smem_common_type->sh_camOtpData[i+5],
																p_sh_smem_common_type->sh_camOtpData[i+6],
																p_sh_smem_common_type->sh_camOtpData[i+7],
																p_sh_smem_common_type->sh_camOtpData[i+8],
																p_sh_smem_common_type->sh_camOtpData[i+9],
																p_sh_smem_common_type->sh_camOtpData[i+10],
																p_sh_smem_common_type->sh_camOtpData[i+11],
																p_sh_smem_common_type->sh_camOtpData[i+12],
																p_sh_smem_common_type->sh_camOtpData[i+13],
																p_sh_smem_common_type->sh_camOtpData[i+14],
																p_sh_smem_common_type->sh_camOtpData[i+15]);
	}

	p_imx111_shdiag_data = kzalloc(sizeof(p_sh_smem_common_type->sh_camOtpData), GFP_KERNEL);
	
	memcpy(p_imx111_shdiag_data, p_sh_smem_common_type->sh_camOtpData, sizeof(p_sh_smem_common_type->sh_camOtpData));

	return msm_sensor_i2c_probe(client, id);
}
static struct i2c_driver imx111_i2c_driver = {
	.id_table = imx111_i2c_id,
	.probe  = imx111_sensor_i2c_probe,
	.driver = {
		.name = SENSOR_NAME,
	},
};

static struct msm_camera_i2c_client imx111_sensor_i2c_client = {
	.addr_type = MSM_CAMERA_I2C_WORD_ADDR,
};

int32_t imx111_sensor_setting(struct msm_sensor_ctrl_t *s_ctrl,
			int update_type, int res)
{
	int32_t rc = 0;
	long fps = 0;
	uint16_t ll_pclk;
	uint16_t fl_lines;
	uint32_t delay = 0;

	CDBG("%s res = %d\n", __func__, res);

	if (update_type == MSM_SENSOR_REG_INIT) {
		s_ctrl->curr_csi_params = NULL;
		msm_sensor_enable_debugfs(s_ctrl);
		msm_sensor_write_init_settings(s_ctrl);
		imx111_exp_gain[6].fl_lines = 0x09C8;
		imx111_exp_gain[6].line = 0x0028;
		imx111_exp_gain[6].gain = 0x0005;
		imx111_exp_gain[6].digital_gain = 0x0100;
		imx111_exp_gain[1].fl_lines = P_INIT_FLEN;
		imx111_exp_gain[1].line = P_INIT_LINE;
		imx111_exp_gain[1].gain = P_INIT_GAIN;
		imx111_exp_gain[1].digital_gain = P_INIT_DGAIN;
		imx111_exp_gain[5].fl_lines = FHD_INIT_FLEN;
		imx111_exp_gain[5].line = FHD_INIT_LINE;
		imx111_exp_gain[5].gain = FHD_INIT_GAIN;
		imx111_exp_gain[5].digital_gain = 0x0100;
	} else if (update_type == MSM_SENSOR_UPDATE_PERIODIC) {
		if(s_ctrl->curr_res != MSM_SENSOR_INVALID_RES){

			s_ctrl->func_tbl->sensor_stop_stream(s_ctrl);
			rc = msm_camera_i2c_read(
					s_ctrl->sensor_i2c_client,
					s_ctrl->sensor_output_reg_addr->line_length_pclk, &ll_pclk,
					MSM_CAMERA_I2C_WORD_DATA);
			rc = msm_camera_i2c_read(
					s_ctrl->sensor_i2c_client,
					s_ctrl->sensor_output_reg_addr->frame_length_lines, &fl_lines,
					MSM_CAMERA_I2C_WORD_DATA);
			if((ll_pclk != 0) && (fl_lines != 0)){
				fps = s_ctrl->msm_sensor_reg->
					output_settings[s_ctrl->curr_res].vt_pixel_clk /
					fl_lines / ll_pclk;
				if(fps != 0)
					delay = 1000 / fps;
			}
			CDBG("%s fps = %ld, frame time = %d\n", __func__, fps, delay);
//			delay += 10;
			CDBG("%s delay = %d\n", __func__, delay);
			msleep(delay);
		}

		if (res == 0) {
			msm_camera_i2c_write_tbl(s_ctrl->sensor_i2c_client,
				(struct msm_camera_i2c_reg_conf *)
				imx111_comm_confs[0].conf,
				imx111_comm_confs[0].size,
				imx111_comm_confs[0].data_type);
		} else {
			msm_sensor_write_res_settings(s_ctrl, res);
			if (s_ctrl->curr_csi_params != s_ctrl->csi_params[res]) {
				s_ctrl->curr_csi_params = s_ctrl->csi_params[res];
				s_ctrl->curr_csi_params->csid_params.lane_assign =
					s_ctrl->sensordata->sensor_platform_info->
					csi_lane_params->csi_lane_assign;
				s_ctrl->curr_csi_params->csiphy_params.lane_mask =
					s_ctrl->sensordata->sensor_platform_info->
					csi_lane_params->csi_lane_mask;
				v4l2_subdev_notify(&s_ctrl->sensor_v4l2_subdev,
					NOTIFY_CSID_CFG,
					&s_ctrl->curr_csi_params->csid_params);
				mb();
				v4l2_subdev_notify(&s_ctrl->sensor_v4l2_subdev,
					NOTIFY_CSIPHY_CFG,
					&s_ctrl->curr_csi_params->csiphy_params);
				mb();
				msleep(20);
			}

			v4l2_subdev_notify(&s_ctrl->sensor_v4l2_subdev,
				NOTIFY_PCLK_CHANGE, &s_ctrl->msm_sensor_reg->
				output_settings[res].op_pixel_clk);
			s_ctrl->func_tbl->sensor_start_stream(s_ctrl);
			if(s_ctrl->curr_res != MSM_SENSOR_INVALID_RES){
				msleep(30);
			}
		}
	}

	return rc;
}

int32_t imx111_sensor_write_exp_gain1(struct msm_sensor_ctrl_t *s_ctrl,
		uint16_t gain, uint32_t line)
{

	uint32_t fl_lines;
	uint8_t offset;
	fl_lines = s_ctrl->curr_frame_length_lines;
	fl_lines = (fl_lines * s_ctrl->fps_divider) / Q10;
	offset = s_ctrl->sensor_exp_gain_info->vert_offset;
	if (line > (fl_lines - offset))
		fl_lines = line + offset;

	CDBG("\n%s:Gain:%d, Linecount:%d\n", __func__, gain, line);
	if (s_ctrl->curr_res == 0) {
		msm_camera_i2c_write_tbl(s_ctrl->sensor_i2c_client,
			(struct msm_camera_i2c_reg_conf *)
			imx111_comm_confs[1].conf,
			imx111_comm_confs[1].size,
			imx111_comm_confs[1].data_type);

		msm_camera_i2c_write(s_ctrl->sensor_i2c_client,
			s_ctrl->sensor_output_reg_addr->frame_length_lines,
			fl_lines, MSM_CAMERA_I2C_WORD_DATA);
		msm_camera_i2c_write(s_ctrl->sensor_i2c_client,
			s_ctrl->sensor_exp_gain_info->coarse_int_time_addr,
			line, MSM_CAMERA_I2C_WORD_DATA);
		msm_camera_i2c_write(s_ctrl->sensor_i2c_client,
			s_ctrl->sensor_exp_gain_info->global_gain_addr, gain,
			MSM_CAMERA_I2C_WORD_DATA);

		msm_camera_i2c_write_tbl(s_ctrl->sensor_i2c_client,
			(struct msm_camera_i2c_reg_conf *)
			imx111_comm_confs[2].conf,
			imx111_comm_confs[2].size,
			imx111_comm_confs[2].data_type);

		if (s_ctrl->curr_csi_params !=
			s_ctrl->csi_params[s_ctrl->curr_res]) {
			s_ctrl->curr_csi_params =
				s_ctrl->csi_params[s_ctrl->curr_res];
			v4l2_subdev_notify(&s_ctrl->sensor_v4l2_subdev,
				NOTIFY_CSID_CFG,
				&s_ctrl->curr_csi_params->csid_params);
			mb();
			v4l2_subdev_notify(&s_ctrl->sensor_v4l2_subdev,
				NOTIFY_CSIPHY_CFG,
				&s_ctrl->curr_csi_params->csiphy_params);
			mb();
			msleep(20);
		}

		v4l2_subdev_notify(&s_ctrl->sensor_v4l2_subdev,
			NOTIFY_PCLK_CHANGE, &s_ctrl->msm_sensor_reg->
			output_settings[s_ctrl->curr_res].op_pixel_clk);
		s_ctrl->func_tbl->sensor_start_stream(s_ctrl);
	} else {
		s_ctrl->func_tbl->sensor_group_hold_on(s_ctrl);
		msm_camera_i2c_write(s_ctrl->sensor_i2c_client,
			s_ctrl->sensor_output_reg_addr->frame_length_lines,
			fl_lines, MSM_CAMERA_I2C_WORD_DATA);
		msm_camera_i2c_write(s_ctrl->sensor_i2c_client,
			s_ctrl->sensor_exp_gain_info->coarse_int_time_addr,
			line, MSM_CAMERA_I2C_WORD_DATA);
		msm_camera_i2c_write(s_ctrl->sensor_i2c_client,
			s_ctrl->sensor_exp_gain_info->global_gain_addr, gain,
			MSM_CAMERA_I2C_WORD_DATA);
		msm_camera_i2c_write(s_ctrl->sensor_i2c_client,
			0x020E, imx111_exp_gain[s_ctrl->curr_res].digital_gain,
			MSM_CAMERA_I2C_WORD_DATA);
		msm_camera_i2c_write(s_ctrl->sensor_i2c_client,
			0x0210, imx111_exp_gain[s_ctrl->curr_res].digital_gain,
			MSM_CAMERA_I2C_WORD_DATA);
		msm_camera_i2c_write(s_ctrl->sensor_i2c_client,
			0x0212, imx111_exp_gain[s_ctrl->curr_res].digital_gain,
			MSM_CAMERA_I2C_WORD_DATA);
		msm_camera_i2c_write(s_ctrl->sensor_i2c_client,
			0x0214, imx111_exp_gain[s_ctrl->curr_res].digital_gain,
			MSM_CAMERA_I2C_WORD_DATA);
		s_ctrl->func_tbl->sensor_group_hold_off(s_ctrl);
	}

	return 0;
}

/* peaking alpha2.1 patch mod -> */
int32_t imx111_sensor_write_exp_gain(struct msm_sensor_ctrl_t *s_ctrl,
		uint32_t gain, uint32_t line)
{

	uint32_t fl_lines;
	uint8_t offset;
	fl_lines = s_ctrl->curr_frame_length_lines;
	fl_lines = (fl_lines * s_ctrl->fps_divider) / Q10;
	offset = s_ctrl->sensor_exp_gain_info->vert_offset;
	if (line > (fl_lines - offset))
		fl_lines = line + offset;

	CDBG("\n%s:Gain:%d, Linecount:%d\n", __func__, gain, line);
	if (s_ctrl->curr_res == 0) {
		msm_camera_i2c_write_tbl(s_ctrl->sensor_i2c_client,
			(struct msm_camera_i2c_reg_conf *)
			imx111_comm_confs[1].conf,
			imx111_comm_confs[1].size,
			imx111_comm_confs[1].data_type);

		msm_camera_i2c_write(s_ctrl->sensor_i2c_client,
			s_ctrl->sensor_output_reg_addr->frame_length_lines,
			fl_lines, MSM_CAMERA_I2C_WORD_DATA);
		msm_camera_i2c_write(s_ctrl->sensor_i2c_client,
			s_ctrl->sensor_exp_gain_info->coarse_int_time_addr,
			line, MSM_CAMERA_I2C_WORD_DATA);
		msm_camera_i2c_write(s_ctrl->sensor_i2c_client,
			s_ctrl->sensor_exp_gain_info->global_gain_addr, gain,
			MSM_CAMERA_I2C_WORD_DATA);

		msm_camera_i2c_write_tbl(s_ctrl->sensor_i2c_client,
			(struct msm_camera_i2c_reg_conf *)
			imx111_comm_confs[2].conf,
			imx111_comm_confs[2].size,
			imx111_comm_confs[2].data_type);

		if (s_ctrl->curr_csi_params !=
			s_ctrl->csi_params[s_ctrl->curr_res]) {
			s_ctrl->curr_csi_params =
				s_ctrl->csi_params[s_ctrl->curr_res];
			v4l2_subdev_notify(&s_ctrl->sensor_v4l2_subdev,
				NOTIFY_CSID_CFG,
				&s_ctrl->curr_csi_params->csid_params);
			mb();
			v4l2_subdev_notify(&s_ctrl->sensor_v4l2_subdev,
				NOTIFY_CSIPHY_CFG,
				&s_ctrl->curr_csi_params->csiphy_params);
			mb();
			msleep(20);
		}

		v4l2_subdev_notify(&s_ctrl->sensor_v4l2_subdev,
			NOTIFY_PCLK_CHANGE, &s_ctrl->msm_sensor_reg->
			output_settings[s_ctrl->curr_res].op_pixel_clk);
		s_ctrl->func_tbl->sensor_start_stream(s_ctrl);
	} else {
		s_ctrl->func_tbl->sensor_group_hold_on(s_ctrl);
		msm_camera_i2c_write(s_ctrl->sensor_i2c_client,
			s_ctrl->sensor_output_reg_addr->frame_length_lines,
			fl_lines, MSM_CAMERA_I2C_WORD_DATA);
		msm_camera_i2c_write(s_ctrl->sensor_i2c_client,
			s_ctrl->sensor_exp_gain_info->coarse_int_time_addr,
			line, MSM_CAMERA_I2C_WORD_DATA);
		msm_camera_i2c_write(s_ctrl->sensor_i2c_client,
			s_ctrl->sensor_exp_gain_info->global_gain_addr, gain,
			MSM_CAMERA_I2C_WORD_DATA);
		msm_camera_i2c_write(s_ctrl->sensor_i2c_client,
			0x020E, imx111_exp_gain[s_ctrl->curr_res].digital_gain,
			MSM_CAMERA_I2C_WORD_DATA);
		msm_camera_i2c_write(s_ctrl->sensor_i2c_client,
			0x0210, imx111_exp_gain[s_ctrl->curr_res].digital_gain,
			MSM_CAMERA_I2C_WORD_DATA);
		msm_camera_i2c_write(s_ctrl->sensor_i2c_client,
			0x0212, imx111_exp_gain[s_ctrl->curr_res].digital_gain,
			MSM_CAMERA_I2C_WORD_DATA);
		msm_camera_i2c_write(s_ctrl->sensor_i2c_client,
			0x0214, imx111_exp_gain[s_ctrl->curr_res].digital_gain,
			MSM_CAMERA_I2C_WORD_DATA);
		s_ctrl->func_tbl->sensor_group_hold_off(s_ctrl);
	}

	return 0;
}
/* peaking alpha2.1 patch mod <- */

int32_t imx111_sensor_adjust_frame_lines(struct msm_sensor_ctrl_t *s_ctrl,
	uint16_t res)
{
	if(imx111_ae_lock == 1){
		CDBG("%s imx111_ae_lock=%d", __func__, imx111_ae_lock);
		return 0;
	}

//	if((res == MSM_SENSOR_RES_QTR) || (res == MSM_SENSOR_RES_5)){
		s_ctrl->func_tbl->sensor_group_hold_on(s_ctrl);
		msm_camera_i2c_write(s_ctrl->sensor_i2c_client,
			s_ctrl->sensor_output_reg_addr->frame_length_lines, imx111_exp_gain[res].fl_lines,
			MSM_CAMERA_I2C_WORD_DATA);
		msm_camera_i2c_write(s_ctrl->sensor_i2c_client,
			s_ctrl->sensor_exp_gain_info->coarse_int_time_addr, imx111_exp_gain[res].line,
			MSM_CAMERA_I2C_WORD_DATA);
		msm_camera_i2c_write(s_ctrl->sensor_i2c_client,
			s_ctrl->sensor_exp_gain_info->global_gain_addr, imx111_exp_gain[res].gain,
			MSM_CAMERA_I2C_WORD_DATA);
		msm_camera_i2c_write(s_ctrl->sensor_i2c_client,
			0x020E, imx111_exp_gain[res].digital_gain,
			MSM_CAMERA_I2C_WORD_DATA);
		msm_camera_i2c_write(s_ctrl->sensor_i2c_client,
			0x0210, imx111_exp_gain[res].digital_gain,
			MSM_CAMERA_I2C_WORD_DATA);
		msm_camera_i2c_write(s_ctrl->sensor_i2c_client,
			0x0212, imx111_exp_gain[res].digital_gain,
			MSM_CAMERA_I2C_WORD_DATA);
		msm_camera_i2c_write(s_ctrl->sensor_i2c_client,
			0x0214, imx111_exp_gain[res].digital_gain,
			MSM_CAMERA_I2C_WORD_DATA);
		s_ctrl->func_tbl->sensor_group_hold_off(s_ctrl);
		
		CDBG("%s fl_lines=%d line=%d gain=%d digital_gain=%d", __func__, imx111_exp_gain[res].fl_lines, imx111_exp_gain[res].line, imx111_exp_gain[res].gain, imx111_exp_gain[res].digital_gain);
//	}
	return 0;
}

static int32_t imx111_sensor_power(struct v4l2_subdev *sd, int on)
{
	int rc = 0;
	struct msm_sensor_ctrl_t *s_ctrl = get_sctrl(sd);
	mutex_lock(s_ctrl->msm_sensor_mutex);
	if (on) {
		rc = s_ctrl->func_tbl->sensor_power_up(s_ctrl);
		if (rc < 0) {
			pr_err("%s: %s power_up failed rc = %d\n", __func__,
				s_ctrl->sensordata->sensor_name, rc);
		}
	} else {
		rc = s_ctrl->func_tbl->sensor_power_down(s_ctrl);
	}
	mutex_unlock(s_ctrl->msm_sensor_mutex);
	return rc;
}

static int __init msm_sensor_init_module(void)
{
	return i2c_add_driver(&imx111_i2c_driver);
}

static struct v4l2_subdev_core_ops imx111_subdev_core_ops = {
	.ioctl = msm_sensor_subdev_ioctl,
	.s_power = imx111_sensor_power,
};
static struct v4l2_subdev_video_ops imx111_subdev_video_ops = {
	.enum_mbus_fmt = msm_sensor_v4l2_enum_fmt,
};

static struct v4l2_subdev_ops imx111_subdev_ops = {
	.core = &imx111_subdev_core_ops,
	.video  = &imx111_subdev_video_ops,
};

#if 0
void imx111_set_dev_addr(struct msm_camera_eeprom_client *ectrl,
			uint32_t *reg_addr) {
	 int32_t rc = 0;
	 rc = msm_camera_i2c_write(ectrl->i2c_client, 0x34C9,
	  (*reg_addr) >> 16, MSM_CAMERA_I2C_BYTE_DATA);
	  if (rc != 0) {
	  CDBG("%s: Page write error\n", __func__);
	  return;
  }
  (*reg_addr) = (*reg_addr) & 0xFFFF;

  /*rc = msm_camera_i2c_poll(ectrl->i2c_client, 0x34C8, 0x01,
		MSM_CAMERA_I2C_SET_BYTE_MASK);
	if (rc != 0) {
		CDBG("%s: Read Status2 error\n", __func__);
		return;
	}*/
}
#endif /* if 0 */

static struct msm_cam_clk_info cam_clk_info[] = {
	{"cam_clk", IMX111_SENSOR_MCLK_6P75HZ},
};

static int32_t imx111_sensor_power_up(struct msm_sensor_ctrl_t *s_ctrl)
{
	int32_t rc = 0;
	struct msm_camera_sensor_info *data = s_ctrl->sensordata;

	CDBG("%s: %d\n", __func__, __LINE__);
	s_ctrl->reg_ptr = kzalloc(sizeof(struct regulator *)
			* data->sensor_platform_info->num_vreg, GFP_KERNEL);
	if (!s_ctrl->reg_ptr) {
		pr_err("%s: could not allocate mem for regulators\n",
			__func__);
		return -ENOMEM;
	}

	CDBG("%s: %d\n", __func__, __LINE__);
	rc = msm_camera_request_gpio_table(data, 1);
	if (rc < 0) {
		pr_err("%s: request gpio failed\n", __func__);
		goto request_gpio_failed;
	}

#if defined(CONFIG_MACH_MNB) 
	imx111_reg_ptr = kzalloc(sizeof(struct regulator *)
			* ARRAY_SIZE(msm_8960_imx111_vreg_1), GFP_KERNEL);
	if (!imx111_reg_ptr) {
		pr_err("%s: could not allocate mem for imx111_reg_ptr\n",
			__func__);
		kfree(s_ctrl->reg_ptr);
		return -ENOMEM;
	}
#endif /* CONFIG_MACH_MNB */

	CDBG("%s: %d\n", __func__, __LINE__);
	rc = msm_camera_config_vreg(&s_ctrl->sensor_i2c_client->client->dev,
			s_ctrl->sensordata->sensor_platform_info->cam_vreg,
			s_ctrl->sensordata->sensor_platform_info->num_vreg,
			s_ctrl->reg_ptr, 1);
	if (rc < 0) {
		pr_err("%s: regulator on failed\n", __func__);
		goto config_vreg_failed;
	}

	CDBG("%s: %d\n", __func__, __LINE__);
	rc = msm_camera_enable_vreg(&s_ctrl->sensor_i2c_client->client->dev,
			s_ctrl->sensordata->sensor_platform_info->cam_vreg,
			s_ctrl->sensordata->sensor_platform_info->num_vreg,
			s_ctrl->reg_ptr, 1);
	if (rc < 0) {
		pr_err("%s: enable regulator failed\n", __func__);
		goto enable_vreg_failed;
	}

#if defined(CONFIG_MACH_MNB) 
	usleep_range(800,1800);

	rc = msm_camera_config_vreg(&s_ctrl->sensor_i2c_client->client->dev,
			msm_8960_imx111_vreg_1,
			ARRAY_SIZE(msm_8960_imx111_vreg_1),
			imx111_reg_ptr, 1);
	if (rc < 0) {
		pr_err("%s: regulator on failed\n", __func__);
		goto config_vreg_failed;
	}

	rc = msm_camera_enable_vreg(&s_ctrl->sensor_i2c_client->client->dev,
			msm_8960_imx111_vreg_1,
			ARRAY_SIZE(msm_8960_imx111_vreg_1),
			imx111_reg_ptr, 1);
	if (rc < 0) {
		pr_err("%s: enable regulator failed\n", __func__);
		goto enable_vreg_failed;
	}
#else /* CONFIG_MACH_MNB */
	usleep(1500);
#endif /* CONFIG_MACH_MNB */

	CDBG("%s: %d\n", __func__, __LINE__);
	rc = msm_camera_config_gpio_table(data, 1);
	if (rc < 0) {
		pr_err("%s: config gpio failed\n", __func__);
		goto config_gpio_failed;
	}

#if !defined(CONFIG_MACH_MNB) 
	usleep(1100);
#endif /* CONFIG_MACH_MNB */

	CDBG("%s: %d\n", __func__, __LINE__);
	if (s_ctrl->clk_rate != 0)
		cam_clk_info->clk_rate = s_ctrl->clk_rate;

	CDBG("%s: %d\n", __func__, __LINE__);
	rc = msm_cam_clk_enable(&s_ctrl->sensor_i2c_client->client->dev,
		cam_clk_info, &s_ctrl->cam_clk, ARRAY_SIZE(cam_clk_info), 1);
	if (rc < 0) {
		pr_err("%s: clk enable failed\n", __func__);
		goto enable_clk_failed;
	}

	CDBG("%s: %d\n", __func__, __LINE__);

#if defined(CONFIG_MACH_MNB) 
	usleep_range(300,1300);
#endif /* CONFIG_MACH_MNB */

//	if (data->sensor_platform_info->ext_power_ctrl != NULL)
//		data->sensor_platform_info->ext_power_ctrl(1);

    CDBG("%s: rc value %d\n", __func__, rc);
    CDBG("%s: %d\n", __func__, __LINE__);
	imx111_curr_mode = 10;
	imx111_ae_lock = 0;

	return rc;
enable_clk_failed:
		msm_camera_config_gpio_table(data, 0);
config_gpio_failed:
	msm_camera_enable_vreg(&s_ctrl->sensor_i2c_client->client->dev,
			s_ctrl->sensordata->sensor_platform_info->cam_vreg,
			s_ctrl->sensordata->sensor_platform_info->num_vreg,
			s_ctrl->reg_ptr, 0);

	CDBG("%s: %d\n", __func__, __LINE__);

enable_vreg_failed:
	msm_camera_config_vreg(&s_ctrl->sensor_i2c_client->client->dev,
		s_ctrl->sensordata->sensor_platform_info->cam_vreg,
		s_ctrl->sensordata->sensor_platform_info->num_vreg,
		s_ctrl->reg_ptr, 0);

	CDBG("%s: %d\n", __func__, __LINE__);
config_vreg_failed:
	msm_camera_request_gpio_table(data, 0);
request_gpio_failed:
	kfree(s_ctrl->reg_ptr);
#if defined(CONFIG_MACH_MNB) 
	kfree(imx111_reg_ptr);
	imx111_reg_ptr = NULL;
#endif /* CONFIG_MACH_MNB */
    CDBG("%s: rc value 2nd time %d\n", __func__, rc);
	CDBG("%s: %d\n", __func__, __LINE__);
	return rc;
}

static int32_t imx111_sensor_power_down(struct msm_sensor_ctrl_t *s_ctrl)
{
	struct msm_camera_sensor_info *data = s_ctrl->sensordata;
	long fps = 0;
	uint16_t ll_pclk;
	uint16_t fl_lines;
	uint32_t delay = 0;
	CDBG("%s\n", __func__);

	s_ctrl->func_tbl->sensor_stop_stream(s_ctrl);

	msm_camera_i2c_read(
			s_ctrl->sensor_i2c_client,
			s_ctrl->sensor_output_reg_addr->line_length_pclk, &ll_pclk,
			MSM_CAMERA_I2C_WORD_DATA);
	msm_camera_i2c_read(
			s_ctrl->sensor_i2c_client,
			s_ctrl->sensor_output_reg_addr->frame_length_lines, &fl_lines,
			MSM_CAMERA_I2C_WORD_DATA);
	CDBG("%s ll_pclk = %d, frame fl_lines = %d\n", __func__, ll_pclk, fl_lines);

	if((ll_pclk != 0) && (fl_lines != 0) && (s_ctrl->curr_res != MSM_SENSOR_INVALID_RES)){
		fps = s_ctrl->msm_sensor_reg->
			output_settings[s_ctrl->curr_res].vt_pixel_clk /
			fl_lines / ll_pclk;
		if(fps != 0)
			delay = 1000 / fps;
	}
	CDBG("%s fps = %ld, frame time = %d\n", __func__, fps, delay);
	msleep(delay);

	usleep(50);

//	if (data->sensor_platform_info->ext_power_ctrl != NULL)
//		data->sensor_platform_info->ext_power_ctrl(0);

	msm_cam_clk_enable(&s_ctrl->sensor_i2c_client->client->dev,
		cam_clk_info, &s_ctrl->cam_clk, ARRAY_SIZE(cam_clk_info), 0);

	usleep(20);

	msm_camera_config_gpio_table(data, 0);

	usleep(1500);

	msm_camera_enable_vreg(&s_ctrl->sensor_i2c_client->client->dev,
		s_ctrl->sensordata->sensor_platform_info->cam_vreg,
		s_ctrl->sensordata->sensor_platform_info->num_vreg,
		s_ctrl->reg_ptr, 0);

	msm_camera_config_vreg(&s_ctrl->sensor_i2c_client->client->dev,
		s_ctrl->sensordata->sensor_platform_info->cam_vreg,
		s_ctrl->sensordata->sensor_platform_info->num_vreg,
		s_ctrl->reg_ptr, 0);

#if defined(CONFIG_MACH_MNB) 
	msm_camera_enable_vreg(&s_ctrl->sensor_i2c_client->client->dev,
		msm_8960_imx111_vreg_1,
		ARRAY_SIZE(msm_8960_imx111_vreg_1),
		imx111_reg_ptr, 0);

	msm_camera_config_vreg(&s_ctrl->sensor_i2c_client->client->dev,
		msm_8960_imx111_vreg_1,
		ARRAY_SIZE(msm_8960_imx111_vreg_1),
		imx111_reg_ptr, 0);
#endif /* CONFIG_MACH_MNB */

	msm_camera_request_gpio_table(data, 0);

	kfree(s_ctrl->reg_ptr);

#if defined(CONFIG_MACH_MNB) 
	kfree(imx111_reg_ptr);
	imx111_reg_ptr = NULL;
#endif /* CONFIG_MACH_MNB */

	return 0;
}

static int imx111_sensor_config(struct msm_sensor_ctrl_t *s_ctrl, void __user *argp)
{
	struct sensor_cfg_data cdata;
	long   rc = 0;
	if (copy_from_user(&cdata,
		(void *)argp,
		sizeof(struct sensor_cfg_data)))
		return -EFAULT;
	mutex_lock(&imx111_mut);
	CDBG("imx111_sensor_config: cfgtype = %d\n",
	cdata.cfgtype);
	switch (cdata.cfgtype) {
	case CFG_SHDIAG_GET_I2C_DATA:
		{
			void *data;
			data = kmalloc(cdata.cfg.i2c_info.length, GFP_KERNEL);
			if(data == NULL){
				rc = -EFAULT;
				break;
			}
			CDBG("%s:%d i2c_read addr=0x%0x\n",__func__,__LINE__,cdata.cfg.i2c_info.addr);
			rc = msm_camera_i2c_read_seq(s_ctrl->sensor_i2c_client, cdata.cfg.i2c_info.addr , data, cdata.cfg.i2c_info.length);
			CDBG("%s:%d i2c_read data=0x%0x\n",__func__,__LINE__,*(unsigned char *)data);
			if (copy_to_user((void *)cdata.cfg.i2c_info.data,
				data,
				cdata.cfg.i2c_info.length)){
				kfree(data);
				CDBG("%s copy_to_user error\n",__func__);
				rc = -EFAULT;
				break;
			}
			kfree(data);
			if (copy_to_user((void *)argp,
				&cdata,
				sizeof(struct sensor_cfg_data))){
				rc = -EFAULT;
				break;
			}
		}
			break;
	case CFG_SHDIAG_SET_I2C_DATA:
		{
			void *data;
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
			rc = msm_camera_i2c_write_seq(s_ctrl->sensor_i2c_client, cdata.cfg.i2c_info.addr, data, cdata.cfg.i2c_info.length);
			kfree(data);
			if (copy_to_user((void *)argp,
				&cdata,
				sizeof(struct sensor_cfg_data))){
				rc = -EFAULT;
				break;
			}
		}
			break;
	case CFG_SHDIAG_GET_OTP_DATA:
		{
			if(p_imx111_shdiag_data[48] != 0x55){
				CDBG("ISO/LB adjust value is not enable\n");
				
				if (copy_to_user((void *)cdata.cfg.otp_info.data,
					&p_imx111_shdiag_data[48],1)){
					pr_err("%s copy_to_user error\n",__func__);
					rc = -EFAULT;
					break;
				}
				
			}else{
			
				if(cdata.cfg.otp_info.length >= 7105){
					pr_err("error cdata.cfg.otp_info.length\n");
					rc = -EFAULT;
					break;
				}
				
				if (copy_to_user((void *)cdata.cfg.otp_info.data,
					&p_imx111_shdiag_data[48],
					cdata.cfg.otp_info.length)){
					pr_err("%s copy_to_user error\n",__func__);
					rc = -EFAULT;
					break;
				}
			}
			if (copy_to_user((void *)argp,
				&cdata,
				sizeof(struct sensor_cfg_data))){
				rc = -EFAULT;
				break;
			}
		}
		break;

	case CFG_SHDIAG_SET_SMEM:
		{
			void *data;
			CDBG("%s cdata.cfg.i2c_info.length=%d", __func__, cdata.cfg.i2c_info.length);
			CDBG("cdata.cfg.i2c_info.addr = 0x%x\n", cdata.cfg.i2c_info.addr);
			if(cdata.cfg.i2c_info.length >= 7105){
				pr_err("error cdata.cfg.otp_info.length\n");
				rc = -EFAULT;
				break;
			}
			data = kmalloc(cdata.cfg.i2c_info.length, GFP_KERNEL);
			if(data == NULL){
				rc = -EFAULT;
				break;
			}
			if (copy_from_user(data,
				(void *)cdata.cfg.i2c_info.data,
				cdata.cfg.i2c_info.length)){
				kfree(data);
				CDBG("%s copy_from_user error\n",__func__);
				rc = -EFAULT;
				break;
			}
			if(p_imx111_shdiag_data != NULL){
				CDBG("cdata.cfg.i2c_info.addr = 0x%x\n", cdata.cfg.i2c_info.addr);
				if(cdata.cfg.i2c_info.addr == 0x30) {
					memcpy(&p_imx111_shdiag_data[48], (unsigned char *)data, cdata.cfg.i2c_info.length);
				}
			}
			else{
				CDBG("%s:%d:p_imx111_shdiag_data != NULL!!!!\n", __func__, __LINE__);
			    kfree(data);
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
		break;
	case CFG_SH_SET_EXP_GAIN:
		{
			uint32_t fl_lines;
			uint8_t offset;
			long fps = 0;
			uint16_t ll_pclk;
			uint16_t cur_fl_lines;
			uint32_t delay = 0;

			fl_lines = s_ctrl->curr_frame_length_lines;
			fl_lines = (fl_lines * s_ctrl->fps_divider) / Q10;
			offset = s_ctrl->sensor_exp_gain_info->vert_offset;
			if (cdata.cfg.exp_gain.line > (fl_lines - offset))
				fl_lines = cdata.cfg.exp_gain.line + offset;

			if(imx111_ae_lock == 1){
				CDBG("%s imx111_ae_lock=%d", __func__, imx111_ae_lock);
				break;
			}

			if( imx111_curr_mode == 0 ) {
				rc = msm_camera_i2c_read(
						s_ctrl->sensor_i2c_client,
						s_ctrl->sensor_output_reg_addr->line_length_pclk, &ll_pclk,
						MSM_CAMERA_I2C_WORD_DATA);
				rc = msm_camera_i2c_read(
						s_ctrl->sensor_i2c_client,
						s_ctrl->sensor_output_reg_addr->frame_length_lines, &cur_fl_lines,
						MSM_CAMERA_I2C_WORD_DATA);
				CDBG("%s ll_pclk = %d, frame fl_lines = %d\n", __func__, ll_pclk, cur_fl_lines);
				if((ll_pclk != 0) && (cur_fl_lines != 0) && (s_ctrl->curr_res != MSM_SENSOR_INVALID_RES)){
					fps = s_ctrl->msm_sensor_reg->
						output_settings[s_ctrl->curr_res].vt_pixel_clk /
						cur_fl_lines / ll_pclk;

					if(fps != 0)
						delay = 1000 / fps;
				}
				CDBG("%s fps = %ld, delay = %d\n", __func__, fps, delay);
			}

			s_ctrl->func_tbl->sensor_group_hold_on(s_ctrl);
			msm_camera_i2c_write(s_ctrl->sensor_i2c_client,
				s_ctrl->sensor_output_reg_addr->frame_length_lines, fl_lines,
				MSM_CAMERA_I2C_WORD_DATA);
			msm_camera_i2c_write(s_ctrl->sensor_i2c_client,
				s_ctrl->sensor_exp_gain_info->coarse_int_time_addr, cdata.cfg.exp_gain.line,
				MSM_CAMERA_I2C_WORD_DATA);
			msm_camera_i2c_write(s_ctrl->sensor_i2c_client,
				s_ctrl->sensor_exp_gain_info->global_gain_addr, cdata.cfg.exp_gain.gain,
				MSM_CAMERA_I2C_WORD_DATA);
			msm_camera_i2c_write(s_ctrl->sensor_i2c_client,
				0x020E, cdata.cfg.exp_gain.digital_gain,
				MSM_CAMERA_I2C_WORD_DATA);
			msm_camera_i2c_write(s_ctrl->sensor_i2c_client,
				0x0210, cdata.cfg.exp_gain.digital_gain,
				MSM_CAMERA_I2C_WORD_DATA);
			msm_camera_i2c_write(s_ctrl->sensor_i2c_client,
				0x0212, cdata.cfg.exp_gain.digital_gain,
				MSM_CAMERA_I2C_WORD_DATA);
			msm_camera_i2c_write(s_ctrl->sensor_i2c_client,
				0x0214, cdata.cfg.exp_gain.digital_gain,
				MSM_CAMERA_I2C_WORD_DATA);
			s_ctrl->func_tbl->sensor_group_hold_off(s_ctrl);
			
			if(s_ctrl->curr_res != MSM_SENSOR_INVALID_RES){
				imx111_exp_gain[s_ctrl->curr_res].fl_lines = fl_lines;
				imx111_exp_gain[s_ctrl->curr_res].line = cdata.cfg.exp_gain.line;
				imx111_exp_gain[s_ctrl->curr_res].gain = cdata.cfg.exp_gain.gain;
				imx111_exp_gain[s_ctrl->curr_res].digital_gain = cdata.cfg.exp_gain.digital_gain;
			}

			CDBG("%s fl_lines=%d line=%d gain=%d digital_gain=%d", __func__, fl_lines, cdata.cfg.exp_gain.line, cdata.cfg.exp_gain.gain, cdata.cfg.exp_gain.digital_gain);
			if( imx111_curr_mode == 0 ) {
				msleep(delay);
			}
		}
		break;
	case CFG_SHDIAG_AE_LOCK:
			imx111_ae_lock = cdata.cfg.ae_mode;
		break;
	default:
		mutex_unlock(&imx111_mut);
		return msm_sensor_config(s_ctrl, argp);
		break;
	}
	
	mutex_unlock(&imx111_mut);
	return rc;
}

static int32_t imx111_sensor_set_sensor_mode(struct msm_sensor_ctrl_t *s_ctrl,
	int mode, int res)
{
	int32_t rc = 0;
	uint32_t fl_lines;
	uint8_t offset;

	CDBG("%s mode=%d \n", __func__, mode);
	if(((imx111_curr_mode != 9) && (mode == 9)) ||
	   ((imx111_curr_mode == 9) && (mode == 2)) ||
	   ((s_ctrl->curr_res == MSM_SENSOR_RES_5) && (res != MSM_SENSOR_RES_5)) ||
	   ((s_ctrl->curr_res != MSM_SENSOR_RES_5) && (res == MSM_SENSOR_RES_5))){
		CDBG("%s mode==SENSOR_MODE_ZSL or SENSOR_MODE_ZSL -> SENSOR_MODE_PREVIEW\n", __func__);

		imx111_curr_mode = mode;
		imx111_exp_gain[6].fl_lines = 0x09C8;
		imx111_exp_gain[6].line = 0x0028;
		imx111_exp_gain[6].gain = 0x0005;
		imx111_exp_gain[6].digital_gain = 0x0100;
		imx111_exp_gain[1].fl_lines = P_INIT_FLEN;
		imx111_exp_gain[1].line = P_INIT_LINE;
		imx111_exp_gain[1].gain = P_INIT_GAIN;
		imx111_exp_gain[1].digital_gain = P_INIT_DGAIN;
		imx111_exp_gain[5].fl_lines = FHD_INIT_FLEN;
		imx111_exp_gain[5].line = FHD_INIT_LINE;
		imx111_exp_gain[5].gain = FHD_INIT_GAIN;
		imx111_exp_gain[5].digital_gain = 0x0100;

		if (s_ctrl->curr_res != res) {

			s_ctrl->curr_frame_length_lines =
				s_ctrl->msm_sensor_reg->
				output_settings[res].frame_length_lines;

			s_ctrl->curr_line_length_pclk =
				s_ctrl->msm_sensor_reg->
				output_settings[res].line_length_pclk;

			CDBG("%s s_ctrl->curr_res = %d\n", __func__, s_ctrl->curr_res);
			if(s_ctrl->curr_res != MSM_SENSOR_INVALID_RES){
				CDBG("%s line[%d]=%d gain[%d]=%d digital_gain[%d]=%d", __func__, 
					s_ctrl->curr_res, imx111_exp_gain[s_ctrl->curr_res].line, 
					s_ctrl->curr_res, imx111_exp_gain[s_ctrl->curr_res].gain, 
					s_ctrl->curr_res, imx111_exp_gain[s_ctrl->curr_res].digital_gain);
				fl_lines = s_ctrl->curr_frame_length_lines;
				fl_lines = (fl_lines * s_ctrl->fps_divider) / Q10;
				offset = s_ctrl->sensor_exp_gain_info->vert_offset;
				if (imx111_exp_gain[s_ctrl->curr_res].line > (fl_lines - offset))
					fl_lines = imx111_exp_gain[s_ctrl->curr_res].line + offset;
		
				imx111_exp_gain[res].fl_lines = fl_lines;
				imx111_exp_gain[res].line = imx111_exp_gain[s_ctrl->curr_res].line;
				imx111_exp_gain[res].gain = imx111_exp_gain[s_ctrl->curr_res].gain;
				imx111_exp_gain[res].digital_gain = imx111_exp_gain[s_ctrl->curr_res].digital_gain;
			}
			CDBG("%s line[%d]=%d gain[%d]=%d digital_gain[%d]=%d", __func__, 
																res, imx111_exp_gain[res].line, 
																res, imx111_exp_gain[res].gain, 
																res, imx111_exp_gain[res].digital_gain);

			if (s_ctrl->is_csic ||
				!s_ctrl->sensordata->csi_if)
				rc = s_ctrl->func_tbl->sensor_csi_setting(s_ctrl,
					MSM_SENSOR_UPDATE_PERIODIC, res);
			else
				rc = s_ctrl->func_tbl->sensor_setting(s_ctrl,
					MSM_SENSOR_UPDATE_PERIODIC, res);
			if (rc < 0)
				return rc;
			s_ctrl->curr_res = res;
		} else {
			long fps = 0;
			uint16_t ll_pclk;
			uint16_t fl_lines;
			uint32_t delay = 0;

			CDBG("%s res=%d s_ctrl->curr_res=%d\n", __func__, res, s_ctrl->curr_res);
			CDBG("%s line[%d]=%d gain[%d]=%d digital_gain[%d]=%d", __func__, 
																res, imx111_exp_gain[res].line, 
																res, imx111_exp_gain[res].gain, 
																res, imx111_exp_gain[res].digital_gain);

			rc = msm_camera_i2c_read(
					s_ctrl->sensor_i2c_client,
					s_ctrl->sensor_output_reg_addr->line_length_pclk, &ll_pclk,
					MSM_CAMERA_I2C_WORD_DATA);
			rc = msm_camera_i2c_read(
					s_ctrl->sensor_i2c_client,
					s_ctrl->sensor_output_reg_addr->frame_length_lines, &fl_lines,
					MSM_CAMERA_I2C_WORD_DATA);

			if (s_ctrl->func_tbl->sensor_adjust_frame_lines)
				rc = s_ctrl->func_tbl->sensor_adjust_frame_lines(s_ctrl, res);

			if((ll_pclk != 0) && (fl_lines != 0)){
				fps = s_ctrl->msm_sensor_reg->
					output_settings[s_ctrl->curr_res].vt_pixel_clk /
					fl_lines / ll_pclk;
				if(fps != 0)
					delay = 1000 / fps;
			}
			CDBG("%s fps = %ld, frame time = %d\n", __func__, fps, delay);
			delay += 10;
			CDBG("%s delay = %d\n", __func__, delay);
			msleep(delay);

		}
	} else {
		CDBG("%s mode!=SENSOR_MODE_ZSL \n", __func__);
		imx111_curr_mode = mode;
		if (s_ctrl->curr_res != res) {
			s_ctrl->curr_frame_length_lines =
				s_ctrl->msm_sensor_reg->
				output_settings[res].frame_length_lines;

			s_ctrl->curr_line_length_pclk =
				s_ctrl->msm_sensor_reg->
				output_settings[res].line_length_pclk;

			if (s_ctrl->is_csic ||
				!s_ctrl->sensordata->csi_if)
				rc = s_ctrl->func_tbl->sensor_csi_setting(s_ctrl,
					MSM_SENSOR_UPDATE_PERIODIC, res);
			else
				rc = s_ctrl->func_tbl->sensor_setting(s_ctrl,
					MSM_SENSOR_UPDATE_PERIODIC, res);
			if (rc < 0)
				return rc;
			s_ctrl->curr_res = res;
		}
	}

	return rc;
}

extern int shcamled_pmic_set_torch_led_1_current(unsigned mA);

int imx111_pmic_set_flash_led_current(enum pmic8058_leds id, unsigned mA)
{
	int ret = 0;
	ret = shcamled_pmic_set_torch_led_1_current(mA);
	return ret;
}
EXPORT_SYMBOL(imx111_pmic_set_flash_led_current);

static struct msm_sensor_fn_t imx111_func_tbl = {
	.sensor_start_stream = msm_sensor_start_stream,
	.sensor_stop_stream = msm_sensor_stop_stream,
	.sensor_group_hold_on = msm_sensor_group_hold_on,
	.sensor_group_hold_off = msm_sensor_group_hold_off,
	.sensor_set_fps = msm_sensor_set_fps,
	.sensor_write_exp_gain = imx111_sensor_write_exp_gain1,
	.sensor_write_snapshot_exp_gain = imx111_sensor_write_exp_gain1,
	.sensor_setting = imx111_sensor_setting,
//	.sensor_set_sensor_mode = msm_sensor_set_sensor_mode,
	.sensor_set_sensor_mode = imx111_sensor_set_sensor_mode,
	.sensor_mode_init = msm_sensor_mode_init,
	.sensor_get_output_info = msm_sensor_get_output_info,
	.sensor_config = imx111_sensor_config,
	.sensor_power_up = imx111_sensor_power_up,
	.sensor_power_down = imx111_sensor_power_down,
	.sensor_adjust_frame_lines = imx111_sensor_adjust_frame_lines,
	.sensor_get_csi_params = msm_sensor_get_csi_params,
};

static struct msm_sensor_reg_t imx111_regs = {
	.default_data_type = MSM_CAMERA_I2C_BYTE_DATA,
	.start_stream_conf = imx111_start_settings,
	.start_stream_conf_size = ARRAY_SIZE(imx111_start_settings),
	.stop_stream_conf = imx111_stop_settings,
	.stop_stream_conf_size = ARRAY_SIZE(imx111_stop_settings),
	.group_hold_on_conf = imx111_groupon_settings,
	.group_hold_on_conf_size = ARRAY_SIZE(imx111_groupon_settings),
	.group_hold_off_conf = imx111_groupoff_settings,
	.group_hold_off_conf_size =
		ARRAY_SIZE(imx111_groupoff_settings),
	.init_settings = &imx111_init_conf[0],
	.init_size = ARRAY_SIZE(imx111_init_conf),
	.mode_settings = &imx111_confs[0],
	.output_settings = &imx111_dimensions[0],
	.num_conf = ARRAY_SIZE(imx111_confs),
};

static struct msm_sensor_ctrl_t imx111_s_ctrl = {
	.msm_sensor_reg = &imx111_regs,
	.sensor_i2c_client = &imx111_sensor_i2c_client,
	.sensor_i2c_addr = 0x34,
	.sensor_output_reg_addr = &imx111_reg_addr,
	.sensor_id_info = &imx111_id_info,
	.sensor_exp_gain_info = &imx111_exp_gain_info,
	.cam_mode = MSM_SENSOR_MODE_INVALID,
	.csi_params = &imx111_csi_params_array[0],
	.msm_sensor_mutex = &imx111_mut,
	.sensor_i2c_driver = &imx111_i2c_driver,
	.sensor_v4l2_subdev_info = imx111_subdev_info,
	.sensor_v4l2_subdev_info_size = ARRAY_SIZE(imx111_subdev_info),
	.sensor_v4l2_subdev_ops = &imx111_subdev_ops,
	.func_tbl = &imx111_func_tbl,
	.clk_rate = IMX111_SENSOR_MCLK_6P75HZ,
};

module_init(msm_sensor_init_module);
MODULE_DESCRIPTION("Sony 8MP Bayer sensor driver");
MODULE_LICENSE("GPL v2");
