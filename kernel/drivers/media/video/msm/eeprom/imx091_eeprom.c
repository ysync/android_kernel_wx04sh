/* Copyright (c) 2012, The Linux Foundation. All rights reserved.
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
#include "msm_camera_eeprom.h"
#include "msm_camera_i2c.h"

DEFINE_MUTEX(imx091_eeprom_mutex);
static struct msm_eeprom_ctrl_t imx091_eeprom_t;
#define DPC_SIZE 20

static const struct i2c_device_id imx091_eeprom_i2c_id[] = {
	{"imx091_eeprom", (kernel_ulong_t)&imx091_eeprom_t},
	{ }
};

static struct i2c_driver imx091_eeprom_i2c_driver = {
	.id_table = imx091_eeprom_i2c_id,
	.probe  = msm_eeprom_i2c_probe,
	.remove = __exit_p(imx091_eeprom_i2c_remove),
	.driver = {
		.name = "imx091_eeprom",
	},
};

static int __init imx091_eeprom_i2c_add_driver(void)
{
	int rc = 0;
	rc = i2c_add_driver(imx091_eeprom_t.i2c_driver);
	return rc;
}

static struct v4l2_subdev_core_ops imx091_eeprom_subdev_core_ops = {
	.ioctl = msm_eeprom_subdev_ioctl,
};

static struct v4l2_subdev_ops imx091_eeprom_subdev_ops = {
	.core = &imx091_eeprom_subdev_core_ops,
};

uint8_t imx091_afcalib_data[4];
uint8_t imx091_wbcalib_data[5];
uint8_t imx091_dpccalib_data[84];
struct msm_calib_af imx091_af_data;
struct msm_calib_wb imx091_wb_data;
struct msm_calib_dpc imx091_dpc_data;

static struct msm_camera_eeprom_info_t imx091_calib_supp_info = {
	{FALSE, 0, 0, 1},
	{FALSE, 0, 0, 1},
	{FALSE, 0, 0, 1},
	{TRUE, sizeof(struct msm_calib_dpc), 2, 1},

};

static struct msm_camera_eeprom_read_t imx091_eeprom_read_tbl[] = {
	{0x003503, &imx091_afcalib_data[0], sizeof(imx091_afcalib_data), 0},
	{0x003507, &imx091_wbcalib_data[0], sizeof(imx091_wbcalib_data)-4, 0},
	{0x013508, &imx091_wbcalib_data[1], sizeof(imx091_wbcalib_data)-1, 0},	
	{0x01350C, &imx091_dpccalib_data[0], 4, 0},
	{0x023510, &imx091_dpccalib_data[4], 8, 0},
	{0x033518, &imx091_dpccalib_data[12], 8, 0},
	{0x043520, &imx091_dpccalib_data[20], 8, 0},
	{0x053528, &imx091_dpccalib_data[28], 8, 0},
	{0x063530, &imx091_dpccalib_data[36], 8, 0},
	{0x073538, &imx091_dpccalib_data[44], 8, 0},
	{0x083540, &imx091_dpccalib_data[52], 8, 0},
	{0x093548, &imx091_dpccalib_data[60], 8, 0},
	{0x0A3550, &imx091_dpccalib_data[68], 8, 0},
	{0x0B3558, &imx091_dpccalib_data[76], 8, 0},
};


static struct msm_camera_eeprom_data_t imx091_eeprom_data_tbl[] = {
	{&imx091_af_data, sizeof(struct msm_calib_af)},
	{&imx091_wb_data, sizeof(struct msm_calib_wb)},
	{&imx091_dpc_data, sizeof(struct msm_calib_dpc)},
};

static void imx091_format_dpcdata(void)
{
	uint16_t i;
	uint16_t msb_xcord, lsb_xcord, msb_ycord, lsb_ycord, count = 0;

	for (i = 0; i < DPC_SIZE; i++) {
		msb_xcord = (imx091_dpccalib_data[i*4] & 0x001F);
		lsb_xcord = imx091_dpccalib_data[i*4+1];
		msb_ycord = (imx091_dpccalib_data[i*4+2] & 0x000F);
		lsb_ycord = imx091_dpccalib_data[i*4+3];
				
		imx091_dpc_data.snapshot_coord[i].x =
			(msb_xcord << 8) | lsb_xcord;
		imx091_dpc_data.snapshot_coord[i].y =
			(msb_ycord << 8) | lsb_ycord;
	}

	for (i = 0; i < DPC_SIZE; i++)
		if (!((imx091_dpc_data.snapshot_coord[i].x == 0) &&
			(imx091_dpc_data.snapshot_coord[i].y == 0)))
				count++;

	imx091_dpc_data.validcount = count;

	CDBG("%s count = %d\n", __func__, count);
	for (i = 0; i < count; i++){
		CDBG("snapshot_dpc_cord[%d] X= %d Y = %d\n",
		i, imx091_dpc_data.snapshot_coord[i].x,
		imx091_dpc_data.snapshot_coord[i].y);
	}

	for (i = 0; i < count; i++){
		imx091_dpc_data.snapshot_coord[i].x = 4208 - 1 - imx091_dpc_data.snapshot_coord[i].x;
		imx091_dpc_data.snapshot_coord[i].y = 3120 - 1 - imx091_dpc_data.snapshot_coord[i].y;
		CDBG("snapshot_dpc_cord(rotated)[%d] X= %d Y = %d\n",
		i, imx091_dpc_data.snapshot_coord[i].x,
		imx091_dpc_data.snapshot_coord[i].y);
	}

	for (i = 0; i < count; i++) {
		imx091_dpc_data.preview_coord[i].x =
			imx091_dpc_data.snapshot_coord[i].x;
		imx091_dpc_data.preview_coord[i].y =
			((imx091_dpc_data.snapshot_coord[i].y / 4) * 2)
		+ (imx091_dpc_data.snapshot_coord[i].y % 2);
		CDBG("prview_dpc_cord[%d] X= %d Y = %d\n",
		i, imx091_dpc_data.preview_coord[i].x,
		imx091_dpc_data.preview_coord[i].y);
	}

	for (i = 0; i < count; i++) {
		imx091_dpc_data.video60_coord[i].x =
			imx091_dpc_data.snapshot_coord[i].x;
		imx091_dpc_data.video60_coord[i].y =
			(imx091_dpc_data.snapshot_coord[i].y - 390) / 3;
		CDBG("video60_dpc_cord[%d] X= %d Y = %d\n",
		i, imx091_dpc_data.video60_coord[i].x,
		imx091_dpc_data.video60_coord[i].y);
	}

	for (i = 0; i < count; i++) {
		imx091_dpc_data.video120_coord[i].x =
			((imx091_dpc_data.snapshot_coord[i].x / 4) * 2)
		+ (imx091_dpc_data.snapshot_coord[i].x % 2);
		imx091_dpc_data.video120_coord[i].y =
			(imx091_dpc_data.snapshot_coord[i].y / 6) +
			(((imx091_dpc_data.snapshot_coord[i].y % 12)
			== 5) ? 1 : 0);
		CDBG("video120_dpc_cord[%d] X= %d Y = %d\n",
		i, imx091_dpc_data.video120_coord[i].x,
		imx091_dpc_data.video120_coord[i].y);
	}

}

static void imx091_format_wbdata(void)
{
	uint16_t wb_msb;
	uint32_t r, gr, gb, b;
	wb_msb = imx091_wbcalib_data[0];
	r = ((wb_msb & 0x00C0) << 2) | imx091_wbcalib_data[1];
	gr = ((wb_msb & 0x0030) << 4) | imx091_wbcalib_data[2];
	b = ((wb_msb & 0x000C) << 6) | imx091_wbcalib_data[3];
	gb = ((wb_msb & 0x0003) << 8) | imx091_wbcalib_data[4];

	CDBG("%s imx091_wbcalib_data[0] =%d r =%d gr = %d gb =%d, b=%d\n",
	__func__, wb_msb, r, gr, gb, b);

	imx091_wb_data.r_over_g = (r * 256) / gr;
	imx091_wb_data.b_over_g = (b * 256) / gb;
	imx091_wb_data.gr_over_gb = (gr * 256) / gb;
	CDBG("%s r_over_g =%d b_over_g =%d gr_over_gb = %d\n", __func__,
		imx091_wb_data.r_over_g, imx091_wb_data.b_over_g,
		imx091_wb_data.gr_over_gb);
}

static void imx091_format_afdata(void)
{
	uint16_t vcmcode_msb;
	vcmcode_msb = imx091_afcalib_data[0];
	imx091_af_data.macro_dac =
		((vcmcode_msb & 0x0030) << 4) | imx091_afcalib_data[1];
	imx091_af_data.start_dac =
		((vcmcode_msb & 0x0003) << 8) | imx091_afcalib_data[3];
	imx091_af_data.inf_dac =
		((vcmcode_msb & 0x000C) << 6) | imx091_afcalib_data[2];

	CDBG("%s startDac =%d MacroDac =%d 50cm = %d\n", __func__,
		imx091_af_data.start_dac, imx091_af_data.macro_dac,
		imx091_af_data.inf_dac);
}

void imx091_format_calibrationdata(void)
{
	imx091_format_afdata();
	imx091_format_wbdata();
	imx091_format_dpcdata();
}

void imx091_set_dev_addr(struct msm_eeprom_ctrl_t *ectrl,
	uint32_t *reg_addr) {
	int32_t rc = 0;

	rc = msm_camera_i2c_write(&ectrl->i2c_client, 0x34C9,
		(*reg_addr) >> 16, MSM_CAMERA_I2C_BYTE_DATA);
	if (rc != 0)
		pr_err("%s: Page write error\n", __func__);
}

static struct msm_eeprom_ctrl_t imx091_eeprom_t = {
	.i2c_driver = &imx091_eeprom_i2c_driver,
	.i2c_addr = 0x34,
	.eeprom_v4l2_subdev_ops = &imx091_eeprom_subdev_ops,

	.i2c_client = {
		.addr_type = MSM_CAMERA_I2C_WORD_ADDR,
	},

	.eeprom_mutex = &imx091_eeprom_mutex,

	.func_tbl = {
		.eeprom_init = NULL,
		.eeprom_release = NULL,
		.eeprom_get_info = msm_camera_eeprom_get_info,
		.eeprom_get_data = msm_camera_eeprom_get_data,
		.eeprom_set_dev_addr = imx091_set_dev_addr,
		.eeprom_format_data = imx091_format_calibrationdata,
	},
	.info = &imx091_calib_supp_info,
	.info_size = sizeof(struct msm_camera_eeprom_info_t),
	.read_tbl = imx091_eeprom_read_tbl,
	.read_tbl_size = ARRAY_SIZE(imx091_eeprom_read_tbl),
	.data_tbl = imx091_eeprom_data_tbl,
	.data_tbl_size = ARRAY_SIZE(imx091_eeprom_data_tbl),
};

subsys_initcall(imx091_eeprom_i2c_add_driver);
MODULE_DESCRIPTION("imx091 EEPROM");
MODULE_LICENSE("GPL v2");
