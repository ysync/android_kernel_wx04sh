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
#include "msm_camera_eeprom.h"
#include "msm_camera_i2c.h"

DEFINE_MUTEX(imx111_eeprom_mutex);
static struct msm_eeprom_ctrl_t imx111_eeprom_t;

static const struct i2c_device_id imx111_eeprom_i2c_id[] = {
	{"imx111_eeprom", (kernel_ulong_t)&imx111_eeprom_t},
	{ }
};

static struct i2c_driver imx111_eeprom_i2c_driver = {
	.id_table = imx111_eeprom_i2c_id,
	.probe  = msm_eeprom_i2c_probe,
	.remove = __exit_p(imx111_eeprom_i2c_remove),
	.driver = {
		.name = "imx111_eeprom",
	},
};

static int __init imx111_eeprom_i2c_add_driver(void)
{
	int rc = 0;
	rc = i2c_add_driver(imx111_eeprom_t.i2c_driver);
	return rc;
}

static struct v4l2_subdev_core_ops imx111_eeprom_subdev_core_ops = {
	.ioctl = msm_eeprom_subdev_ioctl,
};

static struct v4l2_subdev_ops imx111_eeprom_subdev_ops = {
	.core = &imx111_eeprom_subdev_core_ops,
};

uint8_t imx111_act_data[1];
uint8_t imx111_afcalib_data[4];
uint8_t imx111_dpccalib_data[18];
struct msm_calib_af imx111_af_data;
struct msm_calib_dpc imx111_dpc_data;

static struct msm_camera_eeprom_info_t imx111_calib_supp_info = {
//	{TRUE, sizeof(struct msm_calib_af), 0, 1},
	{FALSE, 0, 0, 1},
	{FALSE, 0, 0, 1},
	{FALSE, 0, 0, 1},
	{TRUE, sizeof(struct msm_calib_dpc), 1, 1},
};

static struct msm_camera_eeprom_read_t imx111_eeprom_read_tbl[] = {
	{0x003500, &imx111_act_data[0], sizeof(imx111_act_data), 0},
	{0x003503, &imx111_afcalib_data[0], sizeof(imx111_afcalib_data), 0},
	{0x01350C, &imx111_dpccalib_data[0], 4, 0},
	{0x023510, &imx111_dpccalib_data[4], 8, 0},
	{0x033518, &imx111_dpccalib_data[12], 6, 0},
};


static struct msm_camera_eeprom_data_t imx111_eeprom_data_tbl[] = {
	{&imx111_af_data, sizeof(struct msm_calib_af)},
	{&imx111_dpc_data, sizeof(struct msm_calib_dpc)},
};

static void imx111_format_afdata(void)
{
	uint16_t vcmcode_msb;

	if((imx111_act_data[0] & 0x03) != 0x01){
		CDBG("%s imx111_act_data[0] = 0x%0x\n", __func__, imx111_act_data[0]);
		imx111_af_data.macro_dac = 0;
		imx111_af_data.start_dac = 0;
		imx111_af_data.inf_dac = 0;
		return;
	}

	vcmcode_msb = imx111_afcalib_data[0];
	imx111_af_data.macro_dac =
		(( vcmcode_msb & 0x0030) << 4) | imx111_afcalib_data[1];
	imx111_af_data.start_dac =
		(( vcmcode_msb & 0x0003) << 8)| imx111_afcalib_data[3];
	imx111_af_data.inf_dac =
		(( vcmcode_msb & 0x000C) << 6)| imx111_afcalib_data[2];

	CDBG("%s startDac =%d MacroDac =%d 50cm = %d\n", __func__,
		imx111_af_data.start_dac, imx111_af_data.macro_dac,
		imx111_af_data.inf_dac);
}

static void imx111_format_dpcdata(void)
{
	uint16_t dpc_count = 0;
	int i;
/*
	if((imx111_act_data[0] & 0x30) != 0x10){
		CDBG("%s imx111_act_data[0] = 0x%0x\n", __func__, imx111_act_data[0]);
		imx111_dpc_data.validcount = 0;
		return;
	}
*/
	for(i = 0; i < 5; i++){
		if(imx111_dpccalib_data[0] & (0x01 << i)){
			imx111_dpc_data.snapshot_coord[dpc_count].x = ((imx111_dpccalib_data[3 + 3*i] & 0xF0) << 4) + imx111_dpccalib_data[3 + 3*i + 1];
			imx111_dpc_data.snapshot_coord[dpc_count].y = ((imx111_dpccalib_data[3 + 3*i] & 0x0F) << 8) + imx111_dpccalib_data[3 + 3*i + 2];

			CDBG("%s imx111_dpc_data.snapshot_coord[%d].x=0x%0x\n", __func__, dpc_count, imx111_dpc_data.snapshot_coord[dpc_count].x);
			CDBG("%s imx111_dpc_data.snapshot_coord[%d].y=0x%0x\n", __func__, dpc_count, imx111_dpc_data.snapshot_coord[dpc_count].y);

			dpc_count++;
		}
	}
	
	for(i = 0; i < dpc_count; i++){
		imx111_dpc_data.preview_coord[i].x = ((imx111_dpc_data.snapshot_coord[i].x / 4) * 2 ) + (imx111_dpc_data.snapshot_coord[i].x % 2);
		imx111_dpc_data.preview_coord[i].y = ((imx111_dpc_data.snapshot_coord[i].y / 4) * 2 ) + (imx111_dpc_data.snapshot_coord[i].y % 2);

		imx111_dpc_data.video_coord[i].x = ((imx111_dpc_data.snapshot_coord[i].x / 4) * 2 ) + (imx111_dpc_data.snapshot_coord[i].x % 2);
		imx111_dpc_data.video_coord[i].y = ((imx111_dpc_data.snapshot_coord[i].y / 4) * 2 ) + (imx111_dpc_data.snapshot_coord[i].y % 2);

		imx111_dpc_data.video60_coord[i].x = imx111_dpc_data.snapshot_coord[i].x - 558;
		imx111_dpc_data.video60_coord[i].y = imx111_dpc_data.snapshot_coord[i].y - 650;
	}
	imx111_dpc_data.validcount = dpc_count;
	CDBG("%s dpc_count =%d \n", __func__, dpc_count);
}

void imx111_format_calibrationdata(void)
{
	imx111_format_afdata();
	imx111_format_dpcdata();
}

void imx111_set_dev_addr(struct msm_eeprom_ctrl_t *ectrl,
	uint32_t* reg_addr) {
	int32_t rc = 0;

	rc = msm_camera_i2c_write(&ectrl->i2c_client, 0x34C9,
		(*reg_addr) >> 16, MSM_CAMERA_I2C_BYTE_DATA);
	if (rc != 0)
		pr_err("%s: Page write error\n", __func__);

  (*reg_addr) = (*reg_addr) & 0xFFFF;

}

static struct msm_eeprom_ctrl_t imx111_eeprom_t = {
	.i2c_driver = &imx111_eeprom_i2c_driver,
	.i2c_addr = 0x34,
	.eeprom_v4l2_subdev_ops = &imx111_eeprom_subdev_ops,

	.i2c_client = {
		.addr_type = MSM_CAMERA_I2C_WORD_ADDR,
	},

	.eeprom_mutex = &imx111_eeprom_mutex,

	.func_tbl = {
		.eeprom_init = NULL,
		.eeprom_release = NULL,
		.eeprom_get_info = msm_camera_eeprom_get_info,
		.eeprom_get_data = msm_camera_eeprom_get_data,
		.eeprom_set_dev_addr = imx111_set_dev_addr,
		.eeprom_format_data = imx111_format_calibrationdata,
	},
	.info = &imx111_calib_supp_info,
	.info_size = sizeof(struct msm_camera_eeprom_info_t),
	.read_tbl = imx111_eeprom_read_tbl,
	.read_tbl_size = ARRAY_SIZE(imx111_eeprom_read_tbl),
	.data_tbl = imx111_eeprom_data_tbl,
	.data_tbl_size = ARRAY_SIZE(imx111_eeprom_data_tbl),
};

subsys_initcall(imx111_eeprom_i2c_add_driver);
MODULE_DESCRIPTION("imx111 EEPROM");
MODULE_LICENSE("GPL v2");
