/* Copyright (c) 2011-2012, The Linux Foundation. All rights reserved.
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

#include <asm/mach-types.h>
#include <linux/gpio.h>
#include <mach/board.h>
#include <mach/msm_bus_board.h>
#include <mach/gpiomux.h>
#include "devices.h"
#include "board-8960.h"

#ifdef CONFIG_MSM_CAMERA

#if (defined(CONFIG_GPIO_SX150X) || defined(CONFIG_GPIO_SX150X_MODULE)) && \
	defined(CONFIG_I2C)

#if 0
static struct i2c_board_info cam_expander_i2c_info[] = {
	{
		I2C_BOARD_INFO("sx1508q", 0x22),
		.platform_data = &msm8960_sx150x_data[SX150X_CAM]
	},
};

static struct msm_cam_expander_info cam_expander_info[] = {
	{
		cam_expander_i2c_info,
		MSM_8960_GSBI4_QUP_I2C_BUS_ID,
	},
};
#endif
#endif

#if !defined(CONFIG_IMX111)
static struct gpiomux_setting cam_settings[] = {
#if 0
	{
		.func = GPIOMUX_FUNC_GPIO, /*suspend*/
		.drv = GPIOMUX_DRV_2MA,
		.pull = GPIOMUX_PULL_DOWN,
	},

	{
		.func = GPIOMUX_FUNC_1, /*active 1*/
		.drv = GPIOMUX_DRV_2MA,
		.pull = GPIOMUX_PULL_NONE,
	},

	{
		.func = GPIOMUX_FUNC_GPIO, /*active 2*/
		.drv = GPIOMUX_DRV_2MA,
		.pull = GPIOMUX_PULL_NONE,
	},

	{
		.func = GPIOMUX_FUNC_1, /*active 3*/
		.drv = GPIOMUX_DRV_8MA,
		.pull = GPIOMUX_PULL_NONE,
	},

	{
		.func = GPIOMUX_FUNC_5, /*active 4*/
		.drv = GPIOMUX_DRV_8MA,
		.pull = GPIOMUX_PULL_UP,
	},

	{
		.func = GPIOMUX_FUNC_6, /*active 5*/
		.drv = GPIOMUX_DRV_8MA,
		.pull = GPIOMUX_PULL_UP,
	},

	{
		.func = GPIOMUX_FUNC_2, /*active 6*/
		.drv = GPIOMUX_DRV_2MA,
		.pull = GPIOMUX_PULL_UP,
	},

	{
		.func = GPIOMUX_FUNC_3, /*active 7*/
		.drv = GPIOMUX_DRV_8MA,
		.pull = GPIOMUX_PULL_UP,
	},

	{
		.func = GPIOMUX_FUNC_GPIO, /*i2c suspend*/
		.drv = GPIOMUX_DRV_2MA,
		.pull = GPIOMUX_PULL_KEEPER,
	},
#else
	{
		.func = GPIOMUX_FUNC_2, /* active */
		.drv = GPIOMUX_DRV_6MA,
		.pull = GPIOMUX_PULL_NONE,
	},

	{
		.func = GPIOMUX_FUNC_GPIO, /* suspend */
		.drv = GPIOMUX_DRV_6MA,
		.pull = GPIOMUX_PULL_NONE,
		.dir = GPIOMUX_OUT_LOW,
	},

	{
		.func = GPIOMUX_FUNC_1, /* active */
		.drv = GPIOMUX_DRV_6MA,
		.pull = GPIOMUX_PULL_NONE,
	},

	{
		.func = GPIOMUX_FUNC_GPIO, /* suspend */
		.drv = GPIOMUX_DRV_6MA,
		.pull = GPIOMUX_PULL_NONE,
		.dir = GPIOMUX_OUT_LOW,
	},
#if defined(CONFIG_S5K4EC)
	{
		.func = GPIOMUX_FUNC_GPIO,
		.drv = GPIOMUX_DRV_6MA,
		.pull = GPIOMUX_PULL_NONE,
		.dir = GPIOMUX_OUT_HIGH,
	},
	{
		.func = GPIOMUX_FUNC_GPIO,
		.drv = GPIOMUX_DRV_2MA,
		.pull = GPIOMUX_PULL_DOWN,
		.dir = GPIOMUX_IN,
	},
#endif /* CONFIG_S5K4EC */
#endif
};
#endif /* !defined(CONFIG_IMX111) */

#if 0
static struct msm_gpiomux_config msm8960_cdp_flash_configs[] = {
	{
		.gpio = 3,
		.settings = {
			[GPIOMUX_ACTIVE]    = &cam_settings[1],
			[GPIOMUX_SUSPENDED] = &cam_settings[0],
		},
	},
};
#endif

static struct msm_gpiomux_config msm8960_cam_common_configs[] = {
#if 0
	{
		.gpio = 2,
		.settings = {
			[GPIOMUX_ACTIVE]    = &cam_settings[2],
			[GPIOMUX_SUSPENDED] = &cam_settings[0],
		},
	},
	{
		.gpio = 3,
		.settings = {
			[GPIOMUX_ACTIVE]    = &cam_settings[2],
			[GPIOMUX_SUSPENDED] = &cam_settings[0],
		},
	},
	{
		.gpio = 4,
		.settings = {
			[GPIOMUX_ACTIVE]    = &cam_settings[1],
			[GPIOMUX_SUSPENDED] = &cam_settings[0],
		},
	},
	{
		.gpio = 5,
		.settings = {
			[GPIOMUX_ACTIVE]    = &cam_settings[3],
			[GPIOMUX_SUSPENDED] = &cam_settings[0],
		},
	},
	{
		.gpio = 76,
		.settings = {
			[GPIOMUX_ACTIVE]    = &cam_settings[2],
			[GPIOMUX_SUSPENDED] = &cam_settings[0],
		},
	},
	{
		.gpio = 107,
		.settings = {
			[GPIOMUX_ACTIVE]    = &cam_settings[2],
			[GPIOMUX_SUSPENDED] = &cam_settings[0],
		},
	},
#endif
};

static struct msm_gpiomux_config msm8960_cam_2d_configs[] = {
#if 0
	{
		.gpio = 18,
		.settings = {
			[GPIOMUX_ACTIVE]    = &cam_settings[3],
			[GPIOMUX_SUSPENDED] = &cam_settings[8],
		},
	},
	{
		.gpio = 19,
		.settings = {
			[GPIOMUX_ACTIVE]    = &cam_settings[3],
			[GPIOMUX_SUSPENDED] = &cam_settings[8],
		},
	},
	{
		.gpio = 20,
		.settings = {
			[GPIOMUX_ACTIVE]    = &cam_settings[3],
			[GPIOMUX_SUSPENDED] = &cam_settings[8],
		},
	},
	{
		.gpio = 21,
		.settings = {
			[GPIOMUX_ACTIVE]    = &cam_settings[3],
			[GPIOMUX_SUSPENDED] = &cam_settings[8],
		},
	},
#else
#if defined(CONFIG_S5K4EC)
	{
		.gpio = 3,
		.settings = {
			[GPIOMUX_ACTIVE]    = &cam_settings[4],
			[GPIOMUX_SUSPENDED] = &cam_settings[5],
		},
	},
#endif /* CONFIG_S5K4EC */
#if !defined(CONFIG_IMX111)
	{
		.gpio = 4,
		.settings = {
			[GPIOMUX_ACTIVE]    = &cam_settings[0],
			[GPIOMUX_SUSPENDED] = &cam_settings[1],
		},
	},
	{
		.gpio = 5,
		.settings = {
			[GPIOMUX_ACTIVE]    = &cam_settings[2],
			[GPIOMUX_SUSPENDED] = &cam_settings[3],
		},
	},
#endif
#endif
};

#define VFE_CAMIF_TIMER1_GPIO 2
#define VFE_CAMIF_TIMER2_GPIO 3
#define VFE_CAMIF_TIMER3_GPIO_INT 4
#ifdef CONFIG_IMX074
static struct msm_camera_sensor_strobe_flash_data strobe_flash_xenon = {
	.flash_trigger = VFE_CAMIF_TIMER2_GPIO,
	.flash_charge = VFE_CAMIF_TIMER1_GPIO,
	.flash_charge_done = VFE_CAMIF_TIMER3_GPIO_INT,
	.flash_recharge_duration = 50000,
	.irq = MSM_GPIO_TO_INT(VFE_CAMIF_TIMER3_GPIO_INT),
};
#endif /* CONFIG_IMX074 */

#ifdef CONFIG_MSM_CAMERA_FLASH
#if defined(CONFIG_IMX091)
int imx091_pmic_set_flash_led_current(enum pmic8058_leds id, unsigned mA);

static struct msm_camera_sensor_flash_src msm_flash_src = {
	.flash_sr_type = MSM_CAMERA_FLASH_SRC_PMIC,
	._fsrc.pmic_src.num_of_src = 1,
	._fsrc.pmic_src.low_current  = 36,
	._fsrc.pmic_src.high_current = 36,
	._fsrc.pmic_src.led_src_1 = PM8XXX_ID_LED_1,
	._fsrc.pmic_src.led_src_2 = 0,
	._fsrc.pmic_src.pmic_set_current = imx091_pmic_set_flash_led_current,
};
#elif defined(CONFIG_IMX111)
int imx111_pmic_set_flash_led_current(enum pmic8058_leds id, unsigned mA);

static struct msm_camera_sensor_flash_src msm_flash_src = {
	.flash_sr_type = MSM_CAMERA_FLASH_SRC_PMIC,
	._fsrc.pmic_src.num_of_src = 1,
	._fsrc.pmic_src.low_current  = 36,
	._fsrc.pmic_src.high_current = 36,
	._fsrc.pmic_src.led_src_1 = PM8XXX_ID_LED_1,
	._fsrc.pmic_src.led_src_2 = 0,
	._fsrc.pmic_src.pmic_set_current = imx111_pmic_set_flash_led_current,
};
#elif defined(CONFIG_S5K4EC)
int s5k4ec_pmic_set_flash_led_current(enum pmic8058_leds id, unsigned mA);

static struct msm_camera_sensor_flash_src msm_flash_src = {
	.flash_sr_type = MSM_CAMERA_FLASH_SRC_PMIC,
	._fsrc.pmic_src.num_of_src = 1,
	._fsrc.pmic_src.low_current  = 36,
	._fsrc.pmic_src.high_current = 36,
	._fsrc.pmic_src.led_src_1 = PM8XXX_ID_LED_1,
	._fsrc.pmic_src.led_src_2 = 0,
	._fsrc.pmic_src.pmic_set_current = s5k4ec_pmic_set_flash_led_current,
};
#else
static struct msm_camera_sensor_flash_src msm_flash_src = {
	.flash_sr_type = MSM_CAMERA_FLASH_SRC_EXT,
	._fsrc.ext_driver_src.led_en = VFE_CAMIF_TIMER1_GPIO,
	._fsrc.ext_driver_src.led_flash_en = VFE_CAMIF_TIMER2_GPIO,
	._fsrc.ext_driver_src.flash_id = MAM_CAMERA_EXT_LED_FLASH_SC628A,
};
#endif
#endif

static struct msm_bus_vectors cam_init_vectors[] = {
	{
		.src = MSM_BUS_MASTER_VFE,
		.dst = MSM_BUS_SLAVE_EBI_CH0,
		.ab  = 0,
		.ib  = 0,
	},
	{
		.src = MSM_BUS_MASTER_VPE,
		.dst = MSM_BUS_SLAVE_EBI_CH0,
		.ab  = 0,
		.ib  = 0,
	},
	{
		.src = MSM_BUS_MASTER_JPEG_ENC,
		.dst = MSM_BUS_SLAVE_EBI_CH0,
		.ab  = 0,
		.ib  = 0,
	},
	{
		.src = MSM_BUS_MASTER_JPEG_ENC,
		.dst = MSM_BUS_SLAVE_MM_IMEM,
		.ab  = 0,
		.ib  = 0,
	},
	{
		.src = MSM_BUS_MASTER_VFE,
		.dst = MSM_BUS_SLAVE_MM_IMEM,
		.ab  = 0,
		.ib  = 0,
	},
};

static struct msm_bus_vectors cam_preview_vectors[] = {
	{
		.src = MSM_BUS_MASTER_VFE,
		.dst = MSM_BUS_SLAVE_EBI_CH0,
		.ab  = 27648000,
		.ib  = 110592000,
	},
	{
		.src = MSM_BUS_MASTER_VPE,
		.dst = MSM_BUS_SLAVE_EBI_CH0,
		.ab  = 0,
		.ib  = 0,
	},
	{
		.src = MSM_BUS_MASTER_JPEG_ENC,
		.dst = MSM_BUS_SLAVE_EBI_CH0,
		.ab  = 0,
		.ib  = 0,
	},
	{
		.src = MSM_BUS_MASTER_JPEG_ENC,
		.dst = MSM_BUS_SLAVE_MM_IMEM,
		.ab  = 0,
		.ib  = 0,
	},
	{
		.src = MSM_BUS_MASTER_VFE,
		.dst = MSM_BUS_SLAVE_MM_IMEM,
		.ab  = 0,
		.ib  = 0,
	},
};

static struct msm_bus_vectors cam_video_vectors[] = {
	{
		.src = MSM_BUS_MASTER_VFE,
		.dst = MSM_BUS_SLAVE_EBI_CH0,
		.ab  = 154275840,
		.ib  = 617103360,
	},
	{
		.src = MSM_BUS_MASTER_VPE,
		.dst = MSM_BUS_SLAVE_EBI_CH0,
		.ab  = 206807040,
		.ib  = 488816640,
	},
	{
		.src = MSM_BUS_MASTER_JPEG_ENC,
		.dst = MSM_BUS_SLAVE_EBI_CH0,
		.ab  = 0,
		.ib  = 0,
	},
	{
		.src = MSM_BUS_MASTER_JPEG_ENC,
		.dst = MSM_BUS_SLAVE_MM_IMEM,
		.ab  = 0,
		.ib  = 0,
	},
	{
		.src = MSM_BUS_MASTER_VFE,
		.dst = MSM_BUS_SLAVE_MM_IMEM,
		.ab  = 0,
		.ib  = 0,
	},
};

static struct msm_bus_vectors cam_snapshot_vectors[] = {
	{
		.src = MSM_BUS_MASTER_VFE,
		.dst = MSM_BUS_SLAVE_EBI_CH0,
		.ab  = 274423680,
		.ib  = 1097694720,
	},
	{
		.src = MSM_BUS_MASTER_VPE,
		.dst = MSM_BUS_SLAVE_EBI_CH0,
		.ab  = 0,
		.ib  = 0,
	},
	{
		.src = MSM_BUS_MASTER_JPEG_ENC,
		.dst = MSM_BUS_SLAVE_EBI_CH0,
		.ab  = 540000000,
		.ib  = 1350000000,
	},
	{
		.src = MSM_BUS_MASTER_JPEG_ENC,
		.dst = MSM_BUS_SLAVE_MM_IMEM,
		.ab  = 43200000,
		.ib  = 69120000,
	},
	{
		.src = MSM_BUS_MASTER_VFE,
		.dst = MSM_BUS_SLAVE_MM_IMEM,
		.ab  = 43200000,
		.ib  = 69120000,
	},
};

static struct msm_bus_vectors cam_zsl_vectors[] = {
	{
		.src = MSM_BUS_MASTER_VFE,
		.dst = MSM_BUS_SLAVE_EBI_CH0,
		.ab  = 302071680,
		.ib  = 1208286720,
	},
	{
		.src = MSM_BUS_MASTER_VPE,
		.dst = MSM_BUS_SLAVE_EBI_CH0,
		.ab  = 0,
		.ib  = 0,
	},
	{
		.src = MSM_BUS_MASTER_JPEG_ENC,
		.dst = MSM_BUS_SLAVE_EBI_CH0,
		.ab  = 540000000,
		.ib  = 1350000000,
	},
	{
		.src = MSM_BUS_MASTER_JPEG_ENC,
		.dst = MSM_BUS_SLAVE_MM_IMEM,
		.ab  = 43200000,
		.ib  = 69120000,
	},
	{
		.src = MSM_BUS_MASTER_VFE,
		.dst = MSM_BUS_SLAVE_MM_IMEM,
		.ab  = 43200000,
		.ib  = 69120000,
	},
};

static struct msm_bus_paths cam_bus_client_config[] = {
	{
		ARRAY_SIZE(cam_init_vectors),
		cam_init_vectors,
	},
	{
		ARRAY_SIZE(cam_preview_vectors),
		cam_preview_vectors,
	},
	{
		ARRAY_SIZE(cam_video_vectors),
		cam_video_vectors,
	},
	{
		ARRAY_SIZE(cam_snapshot_vectors),
		cam_snapshot_vectors,
	},
	{
		ARRAY_SIZE(cam_zsl_vectors),
		cam_zsl_vectors,
	},
};

static struct msm_bus_scale_pdata cam_bus_client_pdata = {
		cam_bus_client_config,
		ARRAY_SIZE(cam_bus_client_config),
		.name = "msm_camera",
};

static struct msm_camera_device_platform_data msm_camera_csi_device_data[] = {
	{
		.csid_core = 0,
		.is_vpe    = 1,
		.cam_bus_scale_table = &cam_bus_client_pdata,
	},
	{
		.csid_core = 1,
		.is_vpe    = 1,
		.cam_bus_scale_table = &cam_bus_client_pdata,
	},
	{
		.csid_core = 2,
		.is_vpe    = 1,
		.cam_bus_scale_table = &cam_bus_client_pdata,
	},
};

#ifdef CONFIG_IMX074
static struct camera_vreg_t msm_8960_back_cam_vreg[] = {
	{"cam_vdig", REG_LDO, 1200000, 1200000, 105000},
	{"cam_vio", REG_VS, 0, 0, 0},
	{"cam_vana", REG_LDO, 2800000, 2850000, 85600},
	{"cam_vaf", REG_LDO, 2800000, 2800000, 300000},
};
#endif /* CONFIG_IMX074 */

#if defined(CONFIG_WEBCAM_OV7692)
static struct camera_vreg_t msm_8960_front_cam_vreg[] = {
#if defined(CONFIG_MACH_TOR) || defined(CONFIG_MACH_MNB)
	{"cam_vio2", REG_VS, 0, 0, 0},
	{"cam_vana3", REG_LDO, 2800000, 2800000, 85600},
#else /* CONFIG_MACH_TOR */
	{"cam_vio", REG_VS, 0, 0, 0},
	{"cam_vio2", REG_VS, 0, 0, 0},
	{"cam_vana3", REG_LDO, 2800000, 2800000, 85600},
#endif /* CONFIG_MACH_TOR */
};
#elif defined(CONFIG_WEBCAM_S5K8AAYX)
static struct camera_vreg_t msm_8960_front_cam_vreg[] = {
	{"cam_vana3", REG_LDO, 2800000, 2800000, 85600},
	{"cam_vdig", REG_LDO, 1225000, 1225000, 105000},
	{"cam_vio2", REG_VS, 0, 0, 0},
};
#else
#if 0
static struct camera_vreg_t msm_8960_front_cam_vreg[] = {
	{"cam_vio", REG_VS, 0, 0, 0},
	{"cam_vana", REG_LDO, 2800000, 2850000, 85600},
	{"cam_vdig", REG_LDO, 1200000, 1200000, 105000},
};
#endif
#endif

static struct gpio msm8960_common_cam_gpio[] = {
#if defined(CONFIG_WEBCAM_OV7692) || defined(CONFIG_WEBCAM_S5K8AAYX)
	{4, GPIOF_DIR_IN, "CAM2_MCLK"},
#endif
	{5, GPIOF_DIR_IN, "CAMIF_MCLK"},
	{20, GPIOF_DIR_IN, "CAMIF_I2C_DATA"},
	{21, GPIOF_DIR_IN, "CAMIF_I2C_CLK"},
};

#if defined(CONFIG_WEBCAM_OV7692) || defined(CONFIG_WEBCAM_S5K8AAYX)
static struct gpio msm8960_front_cam_gpio[] = {
	{PM8921_GPIO_PM_TO_SYS(38), GPIOF_DIR_OUT, "CAM_RESET"},
};
#else
#if 0
static struct gpio msm8960_front_cam_gpio[] = {
	{76, GPIOF_DIR_OUT, "CAM_RESET"},
};
#endif
#endif

#if defined(CONFIG_IMX091) || defined(CONFIG_S5K3L1YX) || defined(CONFIG_IMX111) || defined(CONFIG_S5K4EC)
static struct gpio msm8960_back_cam_gpio[] = {
#if defined(CONFIG_S5K4EC)
	{3, GPIOF_DIR_OUT, "CAM_STBY"},
#endif /* CONFIG_S5K4EC */
	{PM8921_GPIO_PM_TO_SYS(37), GPIOF_DIR_OUT, "CAM_RESET"},
};
#else
static struct gpio msm8960_back_cam_gpio[] = {
	{107, GPIOF_DIR_OUT, "CAM_RESET"},
};
#endif

#if defined(CONFIG_WEBCAM_OV7692) || defined(CONFIG_WEBCAM_S5K8AAYX)
static struct msm_gpio_set_tbl msm8960_front_cam_gpio_set_tbl[] = {
	{PM8921_GPIO_PM_TO_SYS(38), GPIOF_OUT_INIT_LOW, 1000},
	{PM8921_GPIO_PM_TO_SYS(38), GPIOF_OUT_INIT_HIGH, 4000},
};
#else
#if 0
static struct msm_gpio_set_tbl msm8960_front_cam_gpio_set_tbl[] = {
	{76, GPIOF_OUT_INIT_LOW, 1000},
	{76, GPIOF_OUT_INIT_HIGH, 4000},
};
#endif
#endif

#if defined(CONFIG_IMX091) || defined(CONFIG_S5K3L1YX) || defined(CONFIG_IMX111) || defined(CONFIG_S5K4EC)
static struct msm_gpio_set_tbl msm8960_back_cam_gpio_set_tbl[] = {
#if defined(CONFIG_S5K4EC)
	{3, GPIOF_OUT_INIT_LOW, 1000},
	{3, GPIOF_OUT_INIT_HIGH, 4000},
#endif /* CONFIG_S5K4EC */
	{PM8921_GPIO_PM_TO_SYS(37), GPIOF_OUT_INIT_LOW, 1000},
	{PM8921_GPIO_PM_TO_SYS(37), GPIOF_OUT_INIT_HIGH, 4000},
};
#else
static struct msm_gpio_set_tbl msm8960_back_cam_gpio_set_tbl[] = {
	{107, GPIOF_OUT_INIT_LOW, 1000},
	{107, GPIOF_OUT_INIT_HIGH, 4000},
};
#endif
#if defined(CONFIG_WEBCAM_OV7692) || defined(CONFIG_WEBCAM_S5K8AAYX)
static struct msm_camera_gpio_conf msm_8960_front_cam_gpio_conf = {
	.cam_gpiomux_conf_tbl = msm8960_cam_2d_configs,
	.cam_gpiomux_conf_tbl_size = ARRAY_SIZE(msm8960_cam_2d_configs),
	.cam_gpio_common_tbl = msm8960_common_cam_gpio,
	.cam_gpio_common_tbl_size = ARRAY_SIZE(msm8960_common_cam_gpio),
	.cam_gpio_req_tbl = msm8960_front_cam_gpio,
	.cam_gpio_req_tbl_size = ARRAY_SIZE(msm8960_front_cam_gpio),
	.cam_gpio_set_tbl = msm8960_front_cam_gpio_set_tbl,
	.cam_gpio_set_tbl_size = ARRAY_SIZE(msm8960_front_cam_gpio_set_tbl),
};
#endif
static struct msm_camera_gpio_conf msm_8960_back_cam_gpio_conf = {
	.cam_gpiomux_conf_tbl = msm8960_cam_2d_configs,
	.cam_gpiomux_conf_tbl_size = ARRAY_SIZE(msm8960_cam_2d_configs),
	.cam_gpio_common_tbl = msm8960_common_cam_gpio,
	.cam_gpio_common_tbl_size = ARRAY_SIZE(msm8960_common_cam_gpio),
	.cam_gpio_req_tbl = msm8960_back_cam_gpio,
	.cam_gpio_req_tbl_size = ARRAY_SIZE(msm8960_back_cam_gpio),
	.cam_gpio_set_tbl = msm8960_back_cam_gpio_set_tbl,
	.cam_gpio_set_tbl_size = ARRAY_SIZE(msm8960_back_cam_gpio_set_tbl),
};

#ifdef CONFIG_IMX074
static struct i2c_board_info msm_act_main_cam_i2c_info = {
	I2C_BOARD_INFO("msm_actuator", 0x11),
};

static struct msm_actuator_info msm_act_main_cam_0_info = {
	.board_info     = &msm_act_main_cam_i2c_info,
	.cam_name   = MSM_ACTUATOR_MAIN_CAM_0,
	.bus_id         = MSM_8960_GSBI4_QUP_I2C_BUS_ID,
	.vcm_pwd        = 0,
	.vcm_enable     = 0,
};
#endif /* CONFIG_IMX074 */

#ifdef CONFIG_IMX091
#if defined(CONFIG_BU64291_ACT)
static struct i2c_board_info msm_act_bu64291_i2c_info = {
	I2C_BOARD_INFO("bu64291_act", 0x18),
};

static struct msm_actuator_info imx091_actuator_bu64291_info = {
	.board_info     = &msm_act_bu64291_i2c_info,
	.cam_name       = MSM_ACTUATOR_MAIN_CAM_2,
	.bus_id         = MSM_8960_GSBI4_QUP_I2C_BUS_ID,
	.vcm_pwd        = 0,
	.vcm_enable     = 0,
};
#elif defined(CONFIG_IMX091_ACT)
static struct i2c_board_info imx091_actuator_i2c_info = {
	I2C_BOARD_INFO("imx091_act", 0x1c),
};

static struct msm_actuator_info imx091_actuator_info = {
	.board_info     = &imx091_actuator_i2c_info,
	.cam_name       = MSM_ACTUATOR_MAIN_CAM_0,
	.bus_id         = MSM_8960_GSBI4_QUP_I2C_BUS_ID,
	.vcm_pwd        = 0,
	.vcm_enable     = 0,
};

#else

static struct i2c_board_info msm_act_main_cam1_i2c_info = {
	I2C_BOARD_INFO("msm_actuator", 0x18),
};

static struct msm_actuator_info msm_act_main_cam_1_info = {
	.board_info     = &msm_act_main_cam1_i2c_info,
	.cam_name       = MSM_ACTUATOR_MAIN_CAM_1,
	.bus_id         = MSM_8960_GSBI4_QUP_I2C_BUS_ID,
	.vcm_pwd        = 0,
	.vcm_enable     = 0,
};
#endif
#endif /* CONFIG_IMX091 */

#ifdef CONFIG_IMX074
static struct msm_camera_sensor_flash_data flash_imx074 = {
	.flash_type	= MSM_CAMERA_FLASH_LED,
#ifdef CONFIG_MSM_CAMERA_FLASH
	.flash_src	= &msm_flash_src
#endif
};

static struct msm_camera_csi_lane_params imx074_csi_lane_params = {
	.csi_lane_assign = 0xE4,
	.csi_lane_mask = 0xF,
};

static struct msm_camera_sensor_platform_info sensor_board_info_imx074 = {
	.mount_angle	= 90,
	.cam_vreg = msm_8960_back_cam_vreg,
	.num_vreg = ARRAY_SIZE(msm_8960_back_cam_vreg),
	.gpio_conf = &msm_8960_back_cam_gpio_conf,
	.csi_lane_params = &imx074_csi_lane_params,
};

static struct i2c_board_info imx074_eeprom_i2c_info = {
	I2C_BOARD_INFO("imx074_eeprom", 0x34 << 1),
};

static struct msm_eeprom_info imx074_eeprom_info = {
	.board_info     = &imx074_eeprom_i2c_info,
	.bus_id         = MSM_8960_GSBI4_QUP_I2C_BUS_ID,
};

static struct msm_camera_sensor_info msm_camera_sensor_imx074_data = {
	.sensor_name	= "imx074",
	.pdata	= &msm_camera_csi_device_data[0],
	.flash_data	= &flash_imx074,
	.strobe_flash_data = &strobe_flash_xenon,
	.sensor_platform_info = &sensor_board_info_imx074,
	.csi_if	= 1,
	.camera_type = BACK_CAMERA_2D,
	.sensor_type = BAYER_SENSOR,
	.actuator_info = &msm_act_main_cam_0_info,
	.eeprom_info = &imx074_eeprom_info,
};
#endif /* CONFIG_IMX074 */

#ifdef CONFIG_MT9M114
static struct camera_vreg_t msm_8960_mt9m114_vreg[] = {
	{"cam_vio", REG_VS, 0, 0, 0},
	{"cam_vdig", REG_LDO, 1200000, 1200000, 105000},
	{"cam_vana", REG_LDO, 2800000, 2850000, 85600},
	{"cam_vaf", REG_LDO, 2800000, 2800000, 300000},
};

static struct msm_camera_sensor_flash_data flash_mt9m114 = {
	.flash_type = MSM_CAMERA_FLASH_NONE
};

static struct msm_camera_csi_lane_params mt9m114_csi_lane_params = {
	.csi_lane_assign = 0xE4,
	.csi_lane_mask = 0x1,
};

static struct msm_camera_sensor_platform_info sensor_board_info_mt9m114 = {
	.mount_angle = 90,
	.cam_vreg = msm_8960_mt9m114_vreg,
	.num_vreg = ARRAY_SIZE(msm_8960_mt9m114_vreg),
	.gpio_conf = &msm_8960_front_cam_gpio_conf,
	.csi_lane_params = &mt9m114_csi_lane_params,
};

static struct msm_camera_sensor_info msm_camera_sensor_mt9m114_data = {
	.sensor_name = "mt9m114",
	.pdata = &msm_camera_csi_device_data[1],
	.flash_data = &flash_mt9m114,
	.sensor_platform_info = &sensor_board_info_mt9m114,
	.csi_if = 1,
	.camera_type = FRONT_CAMERA_2D,
	.sensor_type = YUV_SENSOR,
};
#endif /* CONFIG_MT9M114 */

#ifdef CONFIG_OV2720
static struct msm_camera_sensor_flash_data flash_ov2720 = {
	.flash_type	= MSM_CAMERA_FLASH_NONE,
};

static struct msm_camera_csi_lane_params ov2720_csi_lane_params = {
	.csi_lane_assign = 0xE4,
	.csi_lane_mask = 0x3,
};

static struct msm_camera_sensor_platform_info sensor_board_info_ov2720 = {
	.mount_angle	= 0,
	.cam_vreg = msm_8960_front_cam_vreg,
	.num_vreg = ARRAY_SIZE(msm_8960_front_cam_vreg),
	.gpio_conf = &msm_8960_front_cam_gpio_conf,
	.csi_lane_params = &ov2720_csi_lane_params,
};

static struct msm_camera_sensor_info msm_camera_sensor_ov2720_data = {
	.sensor_name	= "ov2720",
	.pdata	= &msm_camera_csi_device_data[1],
	.flash_data	= &flash_ov2720,
	.sensor_platform_info = &sensor_board_info_ov2720,
	.csi_if	= 1,
	.camera_type = FRONT_CAMERA_2D,
	.sensor_type = BAYER_SENSOR,
};
#endif /* CONFIG_OV2720 */

#ifdef CONFIG_S5K3L1YX
#if 1
static struct camera_vreg_t msm_8960_s5k3l1yx_vreg[] = {
	{"cam_vdig", REG_LDO, 1250000, 1250000, 105000},
	{"cam_vana", REG_LDO, 2800000, 2850000, 85600},
	{"cam_vaf", REG_LDO, 3000000, 3000000, 300000},
	{"cam_vio", REG_VS, 0, 0, 0},
};

static struct msm_camera_sensor_flash_data flash_s5k3l1yx = {
	.flash_type	= MSM_CAMERA_FLASH_LED,
#ifdef CONFIG_MSM_CAMERA_FLASH
	.flash_src	= &msm_flash_src
#endif
};
#else
static struct camera_vreg_t msm_8960_s5k3l1yx_vreg[] = {
	{"cam_vdig", REG_LDO, 1200000, 1200000, 105000},
	{"cam_vana", REG_LDO, 2800000, 2850000, 85600},
	{"cam_vio", REG_VS, 0, 0, 0},
	{"cam_vaf", REG_LDO, 2800000, 2800000, 300000},
};

static struct msm_camera_sensor_flash_data flash_s5k3l1yx = {
	.flash_type = MSM_CAMERA_FLASH_NONE,
};
#endif
static struct msm_camera_csi_lane_params s5k3l1yx_csi_lane_params = {
	.csi_lane_assign = 0xE4,
	.csi_lane_mask = 0xF,
};

static struct msm_camera_sensor_platform_info sensor_board_info_s5k3l1yx = {
	.mount_angle  = 0,
	.cam_vreg = msm_8960_s5k3l1yx_vreg,
	.num_vreg = ARRAY_SIZE(msm_8960_s5k3l1yx_vreg),
	.gpio_conf = &msm_8960_back_cam_gpio_conf,
	.csi_lane_params = &s5k3l1yx_csi_lane_params,
};

static struct msm_actuator_info msm_act_main_cam_2_info = {
	.board_info     = &msm_act_main_cam_i2c_info,
	.cam_name   = MSM_ACTUATOR_MAIN_CAM_2,
	.bus_id         = MSM_8960_GSBI4_QUP_I2C_BUS_ID,
	.vcm_pwd        = 0,
	.vcm_enable     = 0,
};

static struct msm_camera_sensor_info msm_camera_sensor_s5k3l1yx_data = {
	.sensor_name          = "s5k3l1yx",
	.pdata                = &msm_camera_csi_device_data[0],
	.flash_data           = &flash_s5k3l1yx,
	.sensor_platform_info = &sensor_board_info_s5k3l1yx,
	.csi_if               = 1,
	.camera_type          = BACK_CAMERA_2D,
	.sensor_type          = BAYER_SENSOR,
	.actuator_info    = &msm_act_main_cam_2_info,
};
#endif /* CONFIG_S5K3L1YX */

#ifdef CONFIG_IMX091
static struct msm_camera_csi_lane_params imx091_csi_lane_params = {
	.csi_lane_assign = 0xE4,
	.csi_lane_mask = 0xF,
};

static struct camera_vreg_t msm_8960_imx091_vreg[] = {
#if 1
#if defined(CONFIG_BU64291_ACT)
	{"cam_vana", REG_LDO, 2800000, 2850000, 85600},
	{"cam_vdd", REG_VS, 0, 0, 0},
	{"cam_vio", REG_VS, 0, 0, 0},
#else /* CONFIG_BU64291_ACT */
	{"cam_vaf", REG_LDO, 3000000, 3000000, 300000},
	{"cam_vana", REG_LDO, 2800000, 2850000, 85600},
	{"cam_vdd", REG_VS, 0, 0, 0},
	{"cam_vio", REG_VS, 0, 0, 0},
#endif /* CONFIG_BU64291_ACT */
#else
	{"cam_vana", REG_LDO, 2800000, 2850000, 85600},
	{"cam_vaf", REG_LDO, 2800000, 2800000, 300000},
	{"cam_vdig", REG_LDO, 1200000, 1200000, 105000},
	{"cam_vio", REG_VS, 0, 0, 0},
#endif
};

static struct msm_camera_sensor_flash_data flash_imx091 = {
	.flash_type	= MSM_CAMERA_FLASH_LED,
#ifdef CONFIG_MSM_CAMERA_FLASH
	.flash_src	= &msm_flash_src
#endif
};

static struct msm_camera_sensor_platform_info sensor_board_info_imx091 = {
	.mount_angle	= 90,
	.cam_vreg = msm_8960_imx091_vreg,
	.num_vreg = ARRAY_SIZE(msm_8960_imx091_vreg),
	.gpio_conf = &msm_8960_back_cam_gpio_conf,
	.csi_lane_params = &imx091_csi_lane_params,
};

static struct i2c_board_info imx091_eeprom_i2c_info = {
	I2C_BOARD_INFO("imx091_eeprom", 0x21),
};

static struct msm_eeprom_info imx091_eeprom_info = {
	.board_info     = &imx091_eeprom_i2c_info,
	.bus_id         = MSM_8960_GSBI4_QUP_I2C_BUS_ID,
};

static struct msm_camera_sensor_info msm_camera_sensor_imx091_data = {
	.sensor_name	= "imx091",
	.pdata	= &msm_camera_csi_device_data[0],
	.flash_data	= &flash_imx091,
	.sensor_platform_info = &sensor_board_info_imx091,
	.csi_if	= 1,
	.camera_type = BACK_CAMERA_2D,
	.sensor_type = BAYER_SENSOR,
#if defined(CONFIG_BU64291_ACT)
	.actuator_info = &imx091_actuator_bu64291_info,
#elif defined(CONFIG_IMX091_ACT)
	.actuator_info = &imx091_actuator_info,
#else
	.actuator_info = &msm_act_main_cam_1_info,
#endif
	.eeprom_info = &imx091_eeprom_info,
};
#endif /* CONFIG_IMX091 */

#if defined(CONFIG_IMX111)
#if defined(CONFIG_IMX111_ACT)
static struct i2c_board_info imx111_actuator_i2c_info = {
	I2C_BOARD_INFO("imx111_act", 0x18),
};

static struct msm_actuator_info imx111_actuator_info = {
	.board_info     = &imx111_actuator_i2c_info,
	.cam_name       = MSM_ACTUATOR_MAIN_CAM_1,
	.bus_id         = MSM_8960_GSBI4_QUP_I2C_BUS_ID,
	.vcm_pwd        = 0,
	.vcm_enable     = 0,
};
#endif /* CONFIG_IMX111_ACT */

static struct camera_vreg_t msm_8960_imx111_vreg[] = {
#if defined(CONFIG_MACH_MNB)
	{"cam_vana", REG_LDO, 2800000, 2850000, 85600},
	{"cam_vdd", REG_VS, 0, 0, 0},
	{"cam_vio", REG_VS, 0, 0, 0},
#else
	{"cam_vana", REG_LDO, 2850000, 2850000, 85600},
	{"cam_vdig", REG_LDO, 1225000, 1225000, 105000},
	{"cam_vio", REG_VS, 0, 0, 0},
	{"cam_vaf", REG_LDO, 3000000, 3000000, 300000},
#endif
};

static struct msm_camera_sensor_flash_data flash_imx111 = {
	.flash_type	= MSM_CAMERA_FLASH_LED,
#ifdef CONFIG_MSM_CAMERA_FLASH
	.flash_src	= &msm_flash_src
#endif
};

static struct msm_camera_csi_lane_params imx111_csi_lane_params = {
	.csi_lane_assign = 0xE4,
	.csi_lane_mask = 0x3,
};

static struct msm_camera_sensor_platform_info sensor_board_info_imx111 = {
	.mount_angle = 90,
	.cam_vreg    = msm_8960_imx111_vreg,
	.num_vreg    = ARRAY_SIZE(msm_8960_imx111_vreg),
	.gpio_conf   = &msm_8960_back_cam_gpio_conf,
	.csi_lane_params = &imx111_csi_lane_params,
};

#ifdef CONFIG_IMX111_EEPROM
static struct i2c_board_info imx111_eeprom_i2c_info = {
	I2C_BOARD_INFO("imx111_eeprom", 0x21),
};

static struct msm_eeprom_info imx111_eeprom_info = {
	.board_info     = &imx111_eeprom_i2c_info,
	.bus_id         = MSM_8960_GSBI4_QUP_I2C_BUS_ID,
};
#endif

static struct msm_camera_sensor_info msm_camera_sensor_imx111_data = {
	.sensor_name          = "imx111",
	.pdata                = &msm_camera_csi_device_data[0],
	.flash_data           = &flash_imx111,
	.sensor_platform_info = &sensor_board_info_imx111,
	.csi_if               = 1,
	.camera_type          = BACK_CAMERA_2D,
	.sensor_type          = BAYER_SENSOR,
#ifdef CONFIG_IMX111_ACT
	.actuator_info = &imx111_actuator_info,
#endif
#ifdef CONFIG_IMX111_EEPROM
	.eeprom_info = &imx111_eeprom_info,
#endif
};
#endif /* CONFIG_IMX111 */

#if defined(CONFIG_S5K4EC)

static struct camera_vreg_t msm_8960_s5k4ec_vreg[] = {
	{"cam_vana", REG_LDO, 2800000, 2850000, 85600},
	{"cam_vdd", REG_VS, 0, 0, 0},
	{"cam_vaf", REG_LDO, 3000000, 3000000, 300000},
	{"cam_vio", REG_VS, 0, 0, 0},
};

static struct msm_camera_sensor_flash_data flash_s5k4ec = {
	.flash_type	= MSM_CAMERA_FLASH_LED,
#ifdef CONFIG_MSM_CAMERA_FLASH
	.flash_src	= &msm_flash_src
#endif
};

static struct msm_camera_csi_lane_params s5k4ec_csi_lane_params = {
	.csi_lane_assign = 0xE4,
	.csi_lane_mask = 0x3,
};

static struct msm_camera_sensor_platform_info sensor_board_info_s5k4ec = {
	.mount_angle = 90,
	.cam_vreg    = msm_8960_s5k4ec_vreg,
	.num_vreg    = ARRAY_SIZE(msm_8960_s5k4ec_vreg),
	.gpio_conf   = &msm_8960_back_cam_gpio_conf,
	.csi_lane_params = &s5k4ec_csi_lane_params,
};

static struct msm_camera_sensor_info msm_camera_sensor_s5k4ec_data = {
	.sensor_name          = "s5k4ec",
	.pdata                = &msm_camera_csi_device_data[0],
	.flash_data           = &flash_s5k4ec,
	.sensor_platform_info = &sensor_board_info_s5k4ec,
	.csi_if               = 1,
	.camera_type          = BACK_CAMERA_2D,
	.sensor_type          = BAYER_SENSOR,

};
#endif /* CONFIG_S5K4EC */

#ifdef CONFIG_WEBCAM_OV7692
static struct msm_camera_sensor_flash_data flash_ov7692 = {
	.flash_type	= MSM_CAMERA_FLASH_NONE,
};

static struct msm_camera_csi_lane_params ov7692_csi_lane_params = {
	.csi_lane_assign = 0xE4,
	.csi_lane_mask = 0x1,
};

static struct msm_camera_sensor_platform_info sensor_board_info_ov7692 = {
	.mount_angle	= 270,
	.cam_vreg = msm_8960_front_cam_vreg,
	.num_vreg = ARRAY_SIZE(msm_8960_front_cam_vreg),
	.gpio_conf = &msm_8960_front_cam_gpio_conf,
	.csi_lane_params = &ov7692_csi_lane_params,
};

static struct msm_camera_sensor_info msm_camera_sensor_ov7692_data = {
	.sensor_name	= "ov7692",
	.pdata	= &msm_camera_csi_device_data[1],
	.flash_data	= &flash_ov7692,
	.sensor_platform_info = &sensor_board_info_ov7692,
	.csi_if	= 1,
	.camera_type = FRONT_CAMERA_2D,
	.sensor_type = YUV_SENSOR,
};
#endif /* CONFIG_WEBCAM_OV7692 */

#ifdef CONFIG_WEBCAM_S5K8AAYX
static struct msm_camera_csi_lane_params s5k8aayx_csi_lane_params = {
	.csi_lane_assign = 0xE4,
	.csi_lane_mask = 0x1,
};

static struct msm_camera_sensor_flash_data flash_s5k8aayx = {
	.flash_type	= MSM_CAMERA_FLASH_NONE,
};

static struct msm_camera_sensor_platform_info sensor_board_info_s5k8aayx = {
	.mount_angle	= 270,
	.cam_vreg = msm_8960_front_cam_vreg,
	.num_vreg = ARRAY_SIZE(msm_8960_front_cam_vreg),
	.gpio_conf = &msm_8960_front_cam_gpio_conf,
	.csi_lane_params = &s5k8aayx_csi_lane_params,
};

static struct msm_camera_sensor_info msm_camera_sensor_s5k8aayx_data = {
	.sensor_name	= "s5k8aayx",
	.pdata	= &msm_camera_csi_device_data[1],
	.flash_data	= &flash_s5k8aayx,
	.sensor_platform_info = &sensor_board_info_s5k8aayx,
	.csi_if	= 1,
	.camera_type = FRONT_CAMERA_2D,
	.sensor_type = YUV_SENSOR,
};
#endif /* CONFIG_WEBCAM_S5K8AAYX */

#if 0
static struct pm8xxx_mpp_config_data privacy_light_on_config = {
	.type		= PM8XXX_MPP_TYPE_SINK,
	.level		= PM8XXX_MPP_CS_OUT_5MA,
	.control	= PM8XXX_MPP_CS_CTRL_MPP_LOW_EN,
};

static struct pm8xxx_mpp_config_data privacy_light_off_config = {
	.type		= PM8XXX_MPP_TYPE_SINK,
	.level		= PM8XXX_MPP_CS_OUT_5MA,
	.control	= PM8XXX_MPP_CS_CTRL_DISABLE,
};

static int32_t msm_camera_8960_ext_power_ctrl(int enable)
{
	int rc = 0;
	if (enable) {
		rc = pm8xxx_mpp_config(PM8921_MPP_PM_TO_SYS(12),
			&privacy_light_on_config);
	} else {
		rc = pm8xxx_mpp_config(PM8921_MPP_PM_TO_SYS(12),
			&privacy_light_off_config);
	}
	return rc;
}
#endif

static struct platform_device msm_camera_server = {
	.name = "msm_cam_server",
	.id = 0,
};

void __init msm8960_init_cam(void)
{
	msm_gpiomux_install(msm8960_cam_common_configs,
			ARRAY_SIZE(msm8960_cam_common_configs));

	if (machine_is_msm8960_cdp()) {
#if 0
		msm_gpiomux_install(msm8960_cdp_flash_configs,
			ARRAY_SIZE(msm8960_cdp_flash_configs));
		msm_flash_src._fsrc.ext_driver_src.led_en =
			GPIO_CAM_GP_LED_EN1;
		msm_flash_src._fsrc.ext_driver_src.led_flash_en =
			GPIO_CAM_GP_LED_EN2;
		#if defined(CONFIG_I2C) && (defined(CONFIG_GPIO_SX150X) || \
		defined(CONFIG_GPIO_SX150X_MODULE))
		msm_flash_src._fsrc.ext_driver_src.expander_info =
			cam_expander_info;
		#endif
#endif
	}

	if (machine_is_msm8960_liquid()) {
#if 0
		struct msm_camera_sensor_info *s_info;
		s_info = &msm_camera_sensor_imx074_data;
		s_info->sensor_platform_info->mount_angle = 180;
		s_info = &msm_camera_sensor_ov2720_data;
		s_info->sensor_platform_info->ext_power_ctrl =
			msm_camera_8960_ext_power_ctrl;
#endif
	}

	if (machine_is_msm8960_fluid()) {
#if 0
		msm_camera_sensor_imx091_data.sensor_platform_info->
			mount_angle = 270;
#endif
	}

	platform_device_register(&msm_camera_server);
	platform_device_register(&msm8960_device_csiphy0);
	platform_device_register(&msm8960_device_csiphy1);
	platform_device_register(&msm8960_device_csiphy2);
	platform_device_register(&msm8960_device_csid0);
	platform_device_register(&msm8960_device_csid1);
	platform_device_register(&msm8960_device_csid2);
	platform_device_register(&msm8960_device_ispif);
	platform_device_register(&msm8960_device_vfe);
	platform_device_register(&msm8960_device_vpe);
}

#ifdef CONFIG_I2C
static struct i2c_board_info msm8960_camera_i2c_boardinfo[] = {
#ifdef CONFIG_IMX074
	{
	I2C_BOARD_INFO("imx074", 0x1A),
	.platform_data = &msm_camera_sensor_imx074_data,
	},
#endif
#ifdef CONFIG_OV2720
	{
	I2C_BOARD_INFO("ov2720", 0x6C),
	.platform_data = &msm_camera_sensor_ov2720_data,
	},
#endif
#ifdef CONFIG_MT9M114
	{
	I2C_BOARD_INFO("mt9m114", 0x48),
	.platform_data = &msm_camera_sensor_mt9m114_data,
	},
#endif
#ifdef CONFIG_S5K3L1YX
	{
	I2C_BOARD_INFO("s5k3l1yx", 0x20),
	.platform_data = &msm_camera_sensor_s5k3l1yx_data,
	},
#endif
#ifdef CONFIG_WEBCAM_OV7692
	{
	I2C_BOARD_INFO("ov7692", 0x78),
	.platform_data = &msm_camera_sensor_ov7692_data,
	},
#endif
#ifdef CONFIG_MSM_CAMERA_FLASH_SC628A
	{
	I2C_BOARD_INFO("sc628a", 0x6E),
	},
#endif
#ifdef CONFIG_IMX091
	{
	I2C_BOARD_INFO("imx091", 0x34),
	.platform_data = &msm_camera_sensor_imx091_data,
	},
#endif
#ifdef CONFIG_WEBCAM_S5K8AAYX
	{
	I2C_BOARD_INFO("s5k8aayx", 0x78),
	.platform_data = &msm_camera_sensor_s5k8aayx_data,
	},
#endif
#if defined(CONFIG_IMX111)
	{
	I2C_BOARD_INFO("imx111", 0x34),
	.platform_data = &msm_camera_sensor_imx111_data,
	},
#endif /* defined(CONFIG_IMX111) */
#if defined(CONFIG_S5K4EC)
	{
	I2C_BOARD_INFO("s5k4ec", 0x7A),
	.platform_data = &msm_camera_sensor_s5k4ec_data,
	},
#endif /* defined(CONFIG_S5K4EC) */

};

struct msm_camera_board_info msm8960_camera_board_info = {
	.board_info = msm8960_camera_i2c_boardinfo,
	.num_i2c_board_info = ARRAY_SIZE(msm8960_camera_i2c_boardinfo),
};
#endif
#endif
