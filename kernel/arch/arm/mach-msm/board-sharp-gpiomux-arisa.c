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

#include <asm/mach-types.h>
#include <mach/gpio.h>
#include <mach/gpiomux.h>
#include <mach/socinfo.h>
#include "devices.h"
#include "board-8960.h"

/* suspended */
static struct gpiomux_setting sus_func1_keeper_6ma_cfg = {
	.func = GPIOMUX_FUNC_1,
	.drv = GPIOMUX_DRV_6MA,
	.pull = GPIOMUX_PULL_KEEPER,
};
static struct gpiomux_setting sus_func1_np_2ma_cfg = {
	.func = GPIOMUX_FUNC_1,
	.drv = GPIOMUX_DRV_2MA,
	.pull = GPIOMUX_PULL_NONE,
};
static struct gpiomux_setting sus_func1_np_6ma_cfg = {
	.func = GPIOMUX_FUNC_1,
	.drv = GPIOMUX_DRV_6MA,
	.pull = GPIOMUX_PULL_NONE,
};
static struct gpiomux_setting sus_func1_pd_2ma_cfg = {
	.func = GPIOMUX_FUNC_1,
	.drv = GPIOMUX_DRV_2MA,
	.pull = GPIOMUX_PULL_DOWN,
};
static struct gpiomux_setting sus_func1_pd_6ma_cfg = {
	.func = GPIOMUX_FUNC_1,
	.drv = GPIOMUX_DRV_6MA,
	.pull = GPIOMUX_PULL_DOWN,
};
static struct gpiomux_setting sus_func2_np_2ma_cfg = {
	.func = GPIOMUX_FUNC_2,
	.drv = GPIOMUX_DRV_2MA,
	.pull = GPIOMUX_PULL_NONE,
};
static struct gpiomux_setting sus_func2_np_6ma_cfg = {
	.func = GPIOMUX_FUNC_2,
	.drv = GPIOMUX_DRV_6MA,
	.pull = GPIOMUX_PULL_NONE,
};
static struct gpiomux_setting sus_func2_pd_2ma_cfg = {
	.func = GPIOMUX_FUNC_2,
	.drv = GPIOMUX_DRV_2MA,
	.pull = GPIOMUX_PULL_DOWN,
};
static struct gpiomux_setting sus_func2_pu_2ma_cfg = {
	.func = GPIOMUX_FUNC_2,
	.drv = GPIOMUX_DRV_2MA,
	.pull = GPIOMUX_PULL_UP,
};
static struct gpiomux_setting sus_gpio_np_2ma_in_cfg = {
	.func = GPIOMUX_FUNC_GPIO,
	.drv = GPIOMUX_DRV_2MA,
	.pull = GPIOMUX_PULL_NONE,
	.dir = GPIOMUX_IN,
};
static struct gpiomux_setting sus_gpio_np_2ma_out_low_cfg = {
	.func = GPIOMUX_FUNC_GPIO,
	.drv = GPIOMUX_DRV_2MA,
	.pull = GPIOMUX_PULL_NONE,
	.dir = GPIOMUX_OUT_LOW,
};
static struct gpiomux_setting sus_gpio_np_6ma_out_high_cfg = {
	.func = GPIOMUX_FUNC_GPIO,
	.drv = GPIOMUX_DRV_6MA,
	.pull = GPIOMUX_PULL_NONE,
	.dir = GPIOMUX_OUT_HIGH,
};
static struct gpiomux_setting sus_gpio_np_6ma_out_low_cfg = {
	.func = GPIOMUX_FUNC_GPIO,
	.drv = GPIOMUX_DRV_6MA,
	.pull = GPIOMUX_PULL_NONE,
	.dir = GPIOMUX_OUT_LOW,
};
static struct gpiomux_setting sus_gpio_pd_2ma_in_cfg = {
	.func = GPIOMUX_FUNC_GPIO,
	.drv = GPIOMUX_DRV_2MA,
	.pull = GPIOMUX_PULL_DOWN,
	.dir = GPIOMUX_IN,
};

static struct gpiomux_setting sus_gpio_pu_2ma_in_cfg = {
	.func = GPIOMUX_FUNC_GPIO,
	.drv = GPIOMUX_DRV_2MA,
	.pull = GPIOMUX_PULL_UP,
	.dir = GPIOMUX_IN,
};

/* active */
static struct gpiomux_setting act_func1_keeper_6ma_cfg = {
	.func = GPIOMUX_FUNC_1,
	.drv = GPIOMUX_DRV_6MA,
	.pull = GPIOMUX_PULL_KEEPER,
};
static struct gpiomux_setting act_func1_np_2ma_cfg = {
	.func = GPIOMUX_FUNC_1,
	.drv = GPIOMUX_DRV_2MA,
	.pull = GPIOMUX_PULL_NONE,
};
static struct gpiomux_setting act_func1_np_6ma_cfg = {
	.func = GPIOMUX_FUNC_1,
	.drv = GPIOMUX_DRV_6MA,
	.pull = GPIOMUX_PULL_NONE,
};
static struct gpiomux_setting act_func1_pd_2ma_cfg = {
	.func = GPIOMUX_FUNC_1,
	.drv = GPIOMUX_DRV_2MA,
	.pull = GPIOMUX_PULL_DOWN,
};
static struct gpiomux_setting act_func1_pd_6ma_cfg = {
	.func = GPIOMUX_FUNC_1,
	.drv = GPIOMUX_DRV_6MA,
	.pull = GPIOMUX_PULL_DOWN,
};
static struct gpiomux_setting act_func2_np_2ma_cfg = {
	.func = GPIOMUX_FUNC_2,
	.drv = GPIOMUX_DRV_2MA,
	.pull = GPIOMUX_PULL_NONE,
};
static struct gpiomux_setting act_func2_np_6ma_cfg = {
	.func = GPIOMUX_FUNC_2,
	.drv = GPIOMUX_DRV_6MA,
	.pull = GPIOMUX_PULL_NONE,
};
static struct gpiomux_setting act_func2_pd_2ma_cfg = {
	.func = GPIOMUX_FUNC_2,
	.drv = GPIOMUX_DRV_2MA,
	.pull = GPIOMUX_PULL_DOWN,
};
static struct gpiomux_setting act_func2_pu_2ma_cfg = {
	.func = GPIOMUX_FUNC_2,
	.drv = GPIOMUX_DRV_2MA,
	.pull = GPIOMUX_PULL_UP,
};
static struct gpiomux_setting act_gpio_np_2ma_in_cfg = {
	.func = GPIOMUX_FUNC_GPIO,
	.drv = GPIOMUX_DRV_2MA,
	.pull = GPIOMUX_PULL_NONE,
	.dir = GPIOMUX_IN,
};
static struct gpiomux_setting act_gpio_np_2ma_out_low_cfg = {
	.func = GPIOMUX_FUNC_GPIO,
	.drv = GPIOMUX_DRV_2MA,
	.pull = GPIOMUX_PULL_NONE,
	.dir = GPIOMUX_OUT_LOW,
};
static struct gpiomux_setting act_gpio_np_6ma_out_high_cfg = {
	.func = GPIOMUX_FUNC_GPIO,
	.drv = GPIOMUX_DRV_6MA,
	.pull = GPIOMUX_PULL_NONE,
	.dir = GPIOMUX_OUT_HIGH,
};
static struct gpiomux_setting act_gpio_np_6ma_out_low_cfg = {
	.func = GPIOMUX_FUNC_GPIO,
	.drv = GPIOMUX_DRV_6MA,
	.pull = GPIOMUX_PULL_NONE,
	.dir = GPIOMUX_OUT_LOW,
};
static struct gpiomux_setting act_gpio_pd_2ma_in_cfg = {
	.func = GPIOMUX_FUNC_GPIO,
	.drv = GPIOMUX_DRV_2MA,
	.pull = GPIOMUX_PULL_DOWN,
	.dir = GPIOMUX_IN,
};
static struct gpiomux_setting act_gpio_pu_2ma_in_cfg = {
	.func = GPIOMUX_FUNC_GPIO,
	.drv = GPIOMUX_DRV_2MA,
	.pull = GPIOMUX_PULL_UP,
	.dir = GPIOMUX_IN,
};

static struct msm_gpiomux_config msm8960_gpio_configs[] __initdata = {
	{
		.gpio = 0,
		.settings = {
			[GPIOMUX_ACTIVE]    = &act_gpio_pd_2ma_in_cfg,
			[GPIOMUX_SUSPENDED] = &sus_gpio_pd_2ma_in_cfg,
		},
	},
	{
		.gpio = 1,
		.settings = {
			[GPIOMUX_ACTIVE]    = &act_gpio_np_6ma_out_low_cfg,
			[GPIOMUX_SUSPENDED] = &sus_gpio_np_6ma_out_low_cfg,
		},
	},
	{
		.gpio = 2,
		.settings = {
			[GPIOMUX_ACTIVE]    = &act_gpio_pd_2ma_in_cfg,
			[GPIOMUX_SUSPENDED] = &sus_gpio_pd_2ma_in_cfg,
		},
	},
	{
		.gpio = 3,
		.settings = {
			[GPIOMUX_ACTIVE]    = &act_gpio_np_6ma_out_low_cfg,
			[GPIOMUX_SUSPENDED] = &sus_gpio_np_6ma_out_low_cfg,
		},
	},
	{
		.gpio = 4,
		.settings = {
			[GPIOMUX_ACTIVE]    = &act_gpio_np_6ma_out_low_cfg,
			[GPIOMUX_SUSPENDED] = &sus_gpio_np_6ma_out_low_cfg,
		},
	},
	{
		.gpio = 5,
		.settings = {
			[GPIOMUX_ACTIVE]    = &act_gpio_np_6ma_out_low_cfg,
			[GPIOMUX_SUSPENDED] = &sus_gpio_np_6ma_out_low_cfg,
		},
	},
	{
		.gpio = 7,
		.settings = {
			[GPIOMUX_ACTIVE]    = &act_gpio_pd_2ma_in_cfg,
			[GPIOMUX_SUSPENDED] = &sus_gpio_pd_2ma_in_cfg,
		},
	},
	{
		.gpio = 8,
		.settings = {
			[GPIOMUX_ACTIVE]    = &act_gpio_pd_2ma_in_cfg,
			[GPIOMUX_SUSPENDED] = &sus_gpio_pd_2ma_in_cfg,
		},
	},
	{
		.gpio = 9,
		.settings = {
			[GPIOMUX_ACTIVE]    = &act_gpio_pd_2ma_in_cfg,
			[GPIOMUX_SUSPENDED] = &sus_gpio_pd_2ma_in_cfg,
		},
	},
	{
		.gpio = 10,
		.settings = {
			[GPIOMUX_ACTIVE]    = &act_func1_np_6ma_cfg,
			[GPIOMUX_SUSPENDED] = &sus_func1_np_6ma_cfg,
		},
	},
	{
		.gpio = 11,
		.settings = {
			[GPIOMUX_ACTIVE]    = &act_func1_np_2ma_cfg,
			[GPIOMUX_SUSPENDED] = &sus_func1_np_2ma_cfg,
		},
	},
	{
		.gpio = 12,
		.settings = {
			[GPIOMUX_ACTIVE]    = &act_func1_np_6ma_cfg,
			[GPIOMUX_SUSPENDED] = &sus_func1_np_6ma_cfg,
		},
	},
	{
		.gpio = 13,
		.settings = {
			[GPIOMUX_ACTIVE]    = &act_func1_np_6ma_cfg,
			[GPIOMUX_SUSPENDED] = &sus_func1_np_6ma_cfg,
		},
	},
	{
		.gpio = 14,
		.settings = {
			[GPIOMUX_ACTIVE]    = &act_gpio_pu_2ma_in_cfg,
			[GPIOMUX_SUSPENDED] = &sus_gpio_pu_2ma_in_cfg,
		},
	},
	{
		.gpio = 15,
		.settings = {
			[GPIOMUX_ACTIVE]    = &act_gpio_pd_2ma_in_cfg,
			[GPIOMUX_SUSPENDED] = &sus_gpio_pd_2ma_in_cfg,
		},
	},
	{
		.gpio = 16,
		.settings = {
			[GPIOMUX_ACTIVE]    = &act_gpio_np_6ma_out_low_cfg,
			[GPIOMUX_SUSPENDED] = &sus_gpio_np_6ma_out_high_cfg,
		},
	},
	{
		.gpio = 17,
		.settings = {
			[GPIOMUX_ACTIVE]    = &act_gpio_pd_2ma_in_cfg,
			[GPIOMUX_SUSPENDED] = &sus_gpio_pd_2ma_in_cfg,
		},
	},
	{
		.gpio = 18,
		.settings = {
			[GPIOMUX_ACTIVE]    = &act_gpio_pd_2ma_in_cfg,
			[GPIOMUX_SUSPENDED] = &sus_gpio_pd_2ma_in_cfg,
		},
	},
	{
		.gpio = 19,
		.settings = {
			[GPIOMUX_ACTIVE]    = &act_gpio_pd_2ma_in_cfg,
			[GPIOMUX_SUSPENDED] = &sus_gpio_pd_2ma_in_cfg,
		},
	},
	{
		.gpio = 20,
		.settings = {
			[GPIOMUX_ACTIVE]    = &act_func1_np_6ma_cfg,
			[GPIOMUX_SUSPENDED] = &sus_func1_np_6ma_cfg,
		},
	},
	{
		.gpio = 21,
		.settings = {
			[GPIOMUX_ACTIVE]    = &act_func1_np_6ma_cfg,
			[GPIOMUX_SUSPENDED] = &sus_func1_np_6ma_cfg,
		},
	},
	{
		.gpio = 22,
		.settings = {
			[GPIOMUX_ACTIVE]    = &act_func1_np_6ma_cfg,
			[GPIOMUX_SUSPENDED] = &sus_gpio_pd_2ma_in_cfg,
		},
	},
	{
		.gpio = 23,
		.settings = {
			[GPIOMUX_ACTIVE]    = &act_func1_np_2ma_cfg,
			[GPIOMUX_SUSPENDED] = &sus_gpio_pd_2ma_in_cfg,
		},
	},
	{
		.gpio = 24,
		.settings = {
			[GPIOMUX_ACTIVE]    = &act_func1_np_2ma_cfg,
			[GPIOMUX_SUSPENDED] = &sus_gpio_pd_2ma_in_cfg,
		},
	},
	{
		.gpio = 25,
		.settings = {
			[GPIOMUX_ACTIVE]    = &act_func1_np_6ma_cfg,
			[GPIOMUX_SUSPENDED] = &sus_gpio_pd_2ma_in_cfg,
		},
	},
	{
		.gpio = 26,
		.settings = {
			[GPIOMUX_ACTIVE]    = &act_gpio_pd_2ma_in_cfg,
			[GPIOMUX_SUSPENDED] = &sus_gpio_pd_2ma_in_cfg,
		},
	},
	{
		.gpio = 27,
		.settings = {
			[GPIOMUX_ACTIVE]    = &act_gpio_pd_2ma_in_cfg,
			[GPIOMUX_SUSPENDED] = &sus_gpio_pd_2ma_in_cfg,
		},
	},
	{
		.gpio = 28,
		.settings = {
			[GPIOMUX_ACTIVE]    = &act_func1_np_6ma_cfg,
			[GPIOMUX_SUSPENDED] = &sus_func1_np_6ma_cfg,
		},
	},
	{
		.gpio = 29,
		.settings = {
			[GPIOMUX_ACTIVE]    = &act_func1_np_6ma_cfg,
			[GPIOMUX_SUSPENDED] = &sus_func1_pd_6ma_cfg,
		},
	},
	{
		.gpio = 30,
		.settings = {
			[GPIOMUX_ACTIVE]    = &act_func1_np_2ma_cfg,
			[GPIOMUX_SUSPENDED] = &sus_func1_np_2ma_cfg,
		},
	},
	{
		.gpio = 31,
		.settings = {
			[GPIOMUX_ACTIVE]    = &act_func1_np_2ma_cfg,
			[GPIOMUX_SUSPENDED] = &sus_func1_np_2ma_cfg,
		},
	},
	{
		.gpio = 32,
		.settings = {
			[GPIOMUX_ACTIVE]    = &act_gpio_pd_2ma_in_cfg,
			[GPIOMUX_SUSPENDED] = &sus_gpio_pd_2ma_in_cfg,
		},
	},
	{
		.gpio = 33,
		.settings = {
			[GPIOMUX_ACTIVE]    = &act_gpio_pd_2ma_in_cfg,
			[GPIOMUX_SUSPENDED] = &sus_gpio_pd_2ma_in_cfg,
		},
	},
	{
		.gpio = 34,
		.settings = {
			[GPIOMUX_ACTIVE]    = &act_gpio_pd_2ma_in_cfg,
			[GPIOMUX_SUSPENDED] = &sus_gpio_pd_2ma_in_cfg,
		},
	},
	{
		.gpio = 35,
		.settings = {
			[GPIOMUX_ACTIVE]    = &act_gpio_pd_2ma_in_cfg,
			[GPIOMUX_SUSPENDED] = &sus_gpio_pd_2ma_in_cfg,
		},
	},
	{
		.gpio = 36,
		.settings = {
			[GPIOMUX_ACTIVE]    = &act_gpio_pd_2ma_in_cfg,
			[GPIOMUX_SUSPENDED] = &sus_gpio_pd_2ma_in_cfg,
		},
	},
	{
		.gpio = 37,
		.settings = {
			[GPIOMUX_ACTIVE]    = &act_gpio_np_6ma_out_low_cfg,
			[GPIOMUX_SUSPENDED] = &sus_gpio_np_6ma_out_low_cfg,
		},
	},
	{
		.gpio = 38,
		.settings = {
			[GPIOMUX_ACTIVE]    = &act_gpio_pd_2ma_in_cfg,
			[GPIOMUX_SUSPENDED] = &sus_gpio_pd_2ma_in_cfg,
		},
	},
	{
		.gpio = 39,
		.settings = {
			[GPIOMUX_ACTIVE]    = &act_gpio_pd_2ma_in_cfg,
			[GPIOMUX_SUSPENDED] = &sus_gpio_pd_2ma_in_cfg,
		},
	},
	{
		.gpio = 40,
		.settings = {
			[GPIOMUX_ACTIVE]    = &act_gpio_pd_2ma_in_cfg,
			[GPIOMUX_SUSPENDED] = &sus_gpio_pd_2ma_in_cfg,
		},
	},
	{
		.gpio = 41,
		.settings = {
			[GPIOMUX_ACTIVE]    = &act_gpio_pd_2ma_in_cfg,
			[GPIOMUX_SUSPENDED] = &sus_gpio_pd_2ma_in_cfg,
		},
	},
	{
		.gpio = 42,
		.settings = {
			[GPIOMUX_ACTIVE]    = &act_gpio_pd_2ma_in_cfg,
			[GPIOMUX_SUSPENDED] = &sus_gpio_pd_2ma_in_cfg,
		},
	},
	{
		.gpio = 43,
		.settings = {
			[GPIOMUX_ACTIVE]    = &act_gpio_pd_2ma_in_cfg,
			[GPIOMUX_SUSPENDED] = &sus_gpio_pd_2ma_in_cfg,
		},
	},
	{
		.gpio = 44,
		.settings = {
			[GPIOMUX_ACTIVE]    = &act_func1_np_6ma_cfg,
			[GPIOMUX_SUSPENDED] = &sus_func1_np_6ma_cfg,
		},
	},
	{
		.gpio = 45,
		.settings = {
			[GPIOMUX_ACTIVE]    = &act_func1_np_6ma_cfg,
			[GPIOMUX_SUSPENDED] = &sus_func1_np_6ma_cfg,
		},
	},
	{
		.gpio = 46,
		.settings = {
			[GPIOMUX_ACTIVE]    = &act_gpio_pu_2ma_in_cfg,
			[GPIOMUX_SUSPENDED] = &sus_gpio_pu_2ma_in_cfg,
		},
	},
	{
		.gpio = 47,
		.settings = {
			[GPIOMUX_ACTIVE]    = &act_gpio_pu_2ma_in_cfg,
			[GPIOMUX_SUSPENDED] = &sus_gpio_pu_2ma_in_cfg,
		},
	},
	{
		.gpio = 48,
		.settings = {
			[GPIOMUX_ACTIVE]    = &act_gpio_np_6ma_out_low_cfg,
			[GPIOMUX_SUSPENDED] = &sus_gpio_np_6ma_out_low_cfg,
		},
	},
	{
		.gpio = 49,
		.settings = {
			[GPIOMUX_ACTIVE]    = &act_gpio_pu_2ma_in_cfg,
			[GPIOMUX_SUSPENDED] = &sus_gpio_pu_2ma_in_cfg,
		},
	},
	{
		.gpio = 50,
		.settings = {
			[GPIOMUX_ACTIVE]    = &act_gpio_pd_2ma_in_cfg,
			[GPIOMUX_SUSPENDED] = &sus_gpio_pd_2ma_in_cfg,
		},
	},
	{
		.gpio = 51,
		.settings = {
			[GPIOMUX_ACTIVE]    = &act_gpio_pd_2ma_in_cfg,
			[GPIOMUX_SUSPENDED] = &sus_gpio_pd_2ma_in_cfg,
		},
	},
	{
		.gpio = 52,
		.settings = {
			[GPIOMUX_ACTIVE]    = &act_gpio_np_2ma_in_cfg,
			[GPIOMUX_SUSPENDED] = &sus_gpio_np_2ma_in_cfg,
		},
	},
	{
		.gpio = 53,
		.settings = {
			[GPIOMUX_ACTIVE]    = &act_gpio_pd_2ma_in_cfg,
			[GPIOMUX_SUSPENDED] = &sus_gpio_pd_2ma_in_cfg,
		},
	},
	{
		.gpio = 54,
		.settings = {
			[GPIOMUX_ACTIVE]    = &act_func2_np_6ma_cfg,
			[GPIOMUX_SUSPENDED] = &sus_func2_np_6ma_cfg,
		},
	},
	{
		.gpio = 55,
		.settings = {
			[GPIOMUX_ACTIVE]    = &act_gpio_pd_2ma_in_cfg,
			[GPIOMUX_SUSPENDED] = &sus_gpio_pd_2ma_in_cfg,
		},
	},
	{
		.gpio = 56,
		.settings = {
			[GPIOMUX_ACTIVE]    = &act_gpio_pd_2ma_in_cfg,
			[GPIOMUX_SUSPENDED] = &sus_gpio_pd_2ma_in_cfg,
		},
	},
	{
		.gpio = 57,
		.settings = {
			[GPIOMUX_ACTIVE]    = &act_gpio_np_2ma_out_low_cfg,
			[GPIOMUX_SUSPENDED] = &sus_gpio_np_2ma_out_low_cfg,
		},
	},
	{
		.gpio = 58,
		.settings = {
			[GPIOMUX_ACTIVE]    = &act_gpio_pu_2ma_in_cfg,
			[GPIOMUX_SUSPENDED] = &sus_gpio_pu_2ma_in_cfg,
		},
	},
	{
		.gpio = 59,
		.settings = {
			[GPIOMUX_ACTIVE]    = &act_func1_np_6ma_cfg,
			[GPIOMUX_SUSPENDED] = &sus_func1_np_6ma_cfg,
		},
	},
	{
		.gpio = 60,
		.settings = {
			[GPIOMUX_ACTIVE]    = &act_func1_keeper_6ma_cfg,
			[GPIOMUX_SUSPENDED] = &sus_func1_keeper_6ma_cfg,
		},
	},
	{
		.gpio = 61,
		.settings = {
			[GPIOMUX_ACTIVE]    = &act_func1_keeper_6ma_cfg,
			[GPIOMUX_SUSPENDED] = &sus_func1_keeper_6ma_cfg,
		},
	},
	{
		.gpio = 62,
		.settings = {
			[GPIOMUX_ACTIVE]    = &act_gpio_np_2ma_in_cfg,
			[GPIOMUX_SUSPENDED] = &sus_gpio_np_2ma_in_cfg,
		},
	},
	{
		.gpio = 63,
		.settings = {
			[GPIOMUX_ACTIVE]    = &act_func1_np_6ma_cfg,
			[GPIOMUX_SUSPENDED] = &sus_gpio_np_6ma_out_low_cfg,
		},
	},
	{
		.gpio = 64,
		.settings = {
			[GPIOMUX_ACTIVE]    = &act_func1_np_2ma_cfg,
			[GPIOMUX_SUSPENDED] = &sus_gpio_np_2ma_in_cfg,
		},
	},
	{
		.gpio = 65,
		.settings = {
			[GPIOMUX_ACTIVE]    = &act_func1_np_6ma_cfg,
			[GPIOMUX_SUSPENDED] = &sus_gpio_np_6ma_out_low_cfg,
		},
	},
	{
		.gpio = 66,
		.settings = {
			[GPIOMUX_ACTIVE]    = &act_func1_np_6ma_cfg,
			[GPIOMUX_SUSPENDED] = &sus_gpio_np_6ma_out_low_cfg,
		},
	},
	{
		.gpio = 67,
		.settings = {
			[GPIOMUX_ACTIVE]    = &act_gpio_pd_2ma_in_cfg,
			[GPIOMUX_SUSPENDED] = &sus_gpio_pd_2ma_in_cfg,
		},
	},
	{
		.gpio = 68,
		.settings = {
			[GPIOMUX_ACTIVE]    = &act_gpio_pu_2ma_in_cfg,
			[GPIOMUX_SUSPENDED] = &sus_gpio_pu_2ma_in_cfg,
		},
	},
	{
		.gpio = 69,
		.settings = {
			[GPIOMUX_ACTIVE]    = &act_gpio_pd_2ma_in_cfg,
			[GPIOMUX_SUSPENDED] = &sus_gpio_pd_2ma_in_cfg,
		},
	},
	{
		.gpio = 70,
		.settings = {
			[GPIOMUX_ACTIVE]    = &act_gpio_pd_2ma_in_cfg,
			[GPIOMUX_SUSPENDED] = &sus_gpio_pd_2ma_in_cfg,
		},
	},
	{
		.gpio = 71,
		.settings = {
			[GPIOMUX_ACTIVE]    = &act_func2_np_6ma_cfg,
			[GPIOMUX_SUSPENDED] = &sus_func2_np_6ma_cfg,
		},
	},
	{
		.gpio = 72,
		.settings = {
			[GPIOMUX_ACTIVE]    = &act_func2_pu_2ma_cfg,
			[GPIOMUX_SUSPENDED] = &sus_func2_pu_2ma_cfg,
		},
	},
	{
		.gpio = 73,
		.settings = {
			[GPIOMUX_ACTIVE]    = &act_func2_np_6ma_cfg,
			[GPIOMUX_SUSPENDED] = &sus_func2_np_6ma_cfg,
		},
	},
	{
		.gpio = 74,
		.settings = {
			[GPIOMUX_ACTIVE]    = &act_func2_np_6ma_cfg,
			[GPIOMUX_SUSPENDED] = &sus_func2_np_6ma_cfg,
		},
	},
	{
		.gpio = 75,
		.settings = {
			[GPIOMUX_ACTIVE]    = &act_gpio_np_2ma_in_cfg,
			[GPIOMUX_SUSPENDED] = &sus_gpio_np_2ma_in_cfg,
		},
	},
	{
		.gpio = 76,
		.settings = {
			[GPIOMUX_ACTIVE]    = &act_gpio_np_2ma_in_cfg,
			[GPIOMUX_SUSPENDED] = &sus_gpio_np_2ma_in_cfg,
		},
	},
	{
		.gpio = 77,
		.settings = {
			[GPIOMUX_ACTIVE]    = &act_gpio_np_2ma_in_cfg,
			[GPIOMUX_SUSPENDED] = &sus_gpio_np_2ma_in_cfg,
		},
	},
	{
		.gpio = 78,
		.settings = {
			[GPIOMUX_ACTIVE]    = &act_gpio_pd_2ma_in_cfg,
			[GPIOMUX_SUSPENDED] = &sus_gpio_pd_2ma_in_cfg,
		},
	},
	{
		.gpio = 79,
		.settings = {
			[GPIOMUX_ACTIVE]    = &act_gpio_pd_2ma_in_cfg,
			[GPIOMUX_SUSPENDED] = &sus_gpio_pd_2ma_in_cfg,
		},
	},
	{
		.gpio = 80,
		.settings = {
			[GPIOMUX_ACTIVE]    = &act_gpio_pd_2ma_in_cfg,
			[GPIOMUX_SUSPENDED] = &sus_gpio_pd_2ma_in_cfg,
		},
	},
	{
		.gpio = 81,
		.settings = {
			[GPIOMUX_ACTIVE]    = &act_gpio_pd_2ma_in_cfg,
			[GPIOMUX_SUSPENDED] = &sus_gpio_pd_2ma_in_cfg,
		},
	},
	{
		.gpio = 82,
		.settings = {
			[GPIOMUX_ACTIVE]    = &act_gpio_pu_2ma_in_cfg,
			[GPIOMUX_SUSPENDED] = &sus_gpio_pd_2ma_in_cfg,
		},
	},
	{
		.gpio = 83,
		.settings = {
			[GPIOMUX_ACTIVE]    = &act_func1_pd_6ma_cfg,
			[GPIOMUX_SUSPENDED] = &sus_func1_pd_6ma_cfg,
		},
	},
	{
		.gpio = 84,
		.settings = {
			[GPIOMUX_ACTIVE]    = &act_func1_pd_6ma_cfg,
			[GPIOMUX_SUSPENDED] = &sus_func1_pd_6ma_cfg,
		},
	},
	{
		.gpio = 85,
		.settings = {
			[GPIOMUX_ACTIVE]    = &act_func1_pd_6ma_cfg,
			[GPIOMUX_SUSPENDED] = &sus_func1_pd_6ma_cfg,
		},
	},
	{
		.gpio = 86,
		.settings = {
			[GPIOMUX_ACTIVE]    = &act_func1_pd_6ma_cfg,
			[GPIOMUX_SUSPENDED] = &sus_func1_pd_6ma_cfg,
		},
	},
	{
		.gpio = 87,
		.settings = {
			[GPIOMUX_ACTIVE]    = &act_func1_np_6ma_cfg,
			[GPIOMUX_SUSPENDED] = &sus_func1_np_6ma_cfg,
		},
	},
	{
		.gpio = 88,
		.settings = {
			[GPIOMUX_ACTIVE]    = &act_func1_np_6ma_cfg,
			[GPIOMUX_SUSPENDED] = &sus_func1_np_6ma_cfg,
		},
	},
	{
		.gpio = 89,
		.settings = {
			[GPIOMUX_ACTIVE]    = &act_gpio_pd_2ma_in_cfg,
			[GPIOMUX_SUSPENDED] = &sus_gpio_pd_2ma_in_cfg,
		},
	},
	{
		.gpio = 90,
		.settings = {
			[GPIOMUX_ACTIVE]    = &act_gpio_np_2ma_in_cfg,
			[GPIOMUX_SUSPENDED] = &sus_gpio_pd_2ma_in_cfg,
		},
	},
	{
		.gpio = 91,
		.settings = {
			[GPIOMUX_ACTIVE]    = &act_gpio_pd_2ma_in_cfg,
			[GPIOMUX_SUSPENDED] = &sus_gpio_pd_2ma_in_cfg,
		},
	},
	{
		.gpio = 92,
		.settings = {
			[GPIOMUX_ACTIVE]    = &act_gpio_np_6ma_out_low_cfg,
			[GPIOMUX_SUSPENDED] = &sus_gpio_pd_2ma_in_cfg,
		},
	},
	{
		.gpio = 93,
		.settings = {
			[GPIOMUX_ACTIVE]    = &act_func2_np_6ma_cfg,
			[GPIOMUX_SUSPENDED] = &sus_func2_np_6ma_cfg,
		},
	},
	{
		.gpio = 94,
		.settings = {
			[GPIOMUX_ACTIVE]    = &act_func2_pd_2ma_cfg,
			[GPIOMUX_SUSPENDED] = &sus_func2_pd_2ma_cfg,
		},
	},
	{
		.gpio = 95,
		.settings = {
			[GPIOMUX_ACTIVE]    = &act_func2_np_6ma_cfg,
			[GPIOMUX_SUSPENDED] = &sus_func2_np_6ma_cfg,
		},
	},
	{
		.gpio = 96,
		.settings = {
			[GPIOMUX_ACTIVE]    = &act_func2_np_6ma_cfg,
			[GPIOMUX_SUSPENDED] = &sus_func2_np_6ma_cfg,
		},
	},
	{
		.gpio = 97,
		.settings = {
			[GPIOMUX_ACTIVE]    = &act_gpio_pd_2ma_in_cfg,
			[GPIOMUX_SUSPENDED] = &sus_gpio_pd_2ma_in_cfg,
		},
	},
	{
		.gpio = 98,
		.settings = {
			[GPIOMUX_ACTIVE]    = &act_gpio_pd_2ma_in_cfg,
			[GPIOMUX_SUSPENDED] = &sus_gpio_pd_2ma_in_cfg,
		},
	},
	{
		.gpio = 99,
		.settings = {
			[GPIOMUX_ACTIVE]    = &act_gpio_np_2ma_in_cfg,
			[GPIOMUX_SUSPENDED] = &sus_gpio_pd_2ma_in_cfg,
		},
	},
	{
		.gpio = 100,
		.settings = {
			[GPIOMUX_ACTIVE]    = &act_gpio_np_2ma_in_cfg,
			[GPIOMUX_SUSPENDED] = &sus_gpio_pd_2ma_in_cfg,
		},
	},
	{
		.gpio = 101,
		.settings = {
			[GPIOMUX_ACTIVE]    = &act_gpio_np_2ma_in_cfg,
			[GPIOMUX_SUSPENDED] = &sus_gpio_pd_2ma_in_cfg,
		},
	},
	{
		.gpio = 102,
		.settings = {
			[GPIOMUX_ACTIVE]    = &act_gpio_pd_2ma_in_cfg,
			[GPIOMUX_SUSPENDED] = &sus_gpio_pd_2ma_in_cfg,
		},
	},
	{
		.gpio = 103,
		.settings = {
			[GPIOMUX_ACTIVE]    = &act_gpio_pu_2ma_in_cfg,
			[GPIOMUX_SUSPENDED] = &sus_gpio_pu_2ma_in_cfg,
		},
	},
	{
		.gpio = 104,
		.settings = {
			[GPIOMUX_ACTIVE]    = &act_gpio_pu_2ma_in_cfg,
			[GPIOMUX_SUSPENDED] = &sus_gpio_pu_2ma_in_cfg,
		},
	},
	{
		.gpio = 105,
		.settings = {
			[GPIOMUX_ACTIVE]    = &act_gpio_pu_2ma_in_cfg,
			[GPIOMUX_SUSPENDED] = &sus_gpio_pu_2ma_in_cfg,
		},
	},
	{
		.gpio = 106,
		.settings = {
			[GPIOMUX_ACTIVE]    = &act_gpio_pu_2ma_in_cfg,
			[GPIOMUX_SUSPENDED] = &sus_gpio_pu_2ma_in_cfg,
		},
	},
	{
		.gpio = 108,
		.settings = {
			[GPIOMUX_ACTIVE]    = &act_func1_np_2ma_cfg,
			[GPIOMUX_SUSPENDED] = &sus_func1_np_2ma_cfg,
		},
	},
	{
		.gpio = 109,
		.settings = {
			[GPIOMUX_ACTIVE]    = &act_gpio_pd_2ma_in_cfg,
			[GPIOMUX_SUSPENDED] = &sus_gpio_pd_2ma_in_cfg,
		},
	},
	{
		.gpio = 110,
		.settings = {
			[GPIOMUX_ACTIVE]    = &act_gpio_pd_2ma_in_cfg,
			[GPIOMUX_SUSPENDED] = &sus_gpio_pd_2ma_in_cfg,
		},
	},
	{
		.gpio = 111,
		.settings = {
			[GPIOMUX_ACTIVE]    = &act_gpio_pd_2ma_in_cfg,
			[GPIOMUX_SUSPENDED] = &sus_gpio_pd_2ma_in_cfg,
		},
	},
	{
		.gpio = 112,
		.settings = {
			[GPIOMUX_ACTIVE]    = &act_gpio_np_2ma_in_cfg,
			[GPIOMUX_SUSPENDED] = &sus_gpio_np_2ma_in_cfg,
		},
	},
	{
		.gpio = 113,
		.settings = {
			[GPIOMUX_ACTIVE]    = &act_gpio_pd_2ma_in_cfg,
			[GPIOMUX_SUSPENDED] = &sus_gpio_pd_2ma_in_cfg,
		},
	},
	{
		.gpio = 114,
		.settings = {
			[GPIOMUX_ACTIVE]    = &act_gpio_pd_2ma_in_cfg,
			[GPIOMUX_SUSPENDED] = &sus_gpio_pd_2ma_in_cfg,
		},
	},
	{
		.gpio = 115,
		.settings = {
			[GPIOMUX_ACTIVE]    = &act_gpio_pd_2ma_in_cfg,
			[GPIOMUX_SUSPENDED] = &sus_gpio_pd_2ma_in_cfg,
		},
	},
	{
		.gpio = 116,
		.settings = {
			[GPIOMUX_ACTIVE]    = &act_func1_pd_2ma_cfg,
			[GPIOMUX_SUSPENDED] = &sus_func1_pd_2ma_cfg,
		},
	},
	{
		.gpio = 117,
		.settings = {
			[GPIOMUX_ACTIVE]    = &act_func1_pd_2ma_cfg,
			[GPIOMUX_SUSPENDED] = &sus_func1_pd_2ma_cfg,
		},
	},
	{
		.gpio = 118,
		.settings = {
			[GPIOMUX_ACTIVE]    = &act_func1_pd_2ma_cfg,
			[GPIOMUX_SUSPENDED] = &sus_func1_pd_2ma_cfg,
		},
	},
	{
		.gpio = 119,
		.settings = {
			[GPIOMUX_ACTIVE]    = &act_func1_pd_2ma_cfg,
			[GPIOMUX_SUSPENDED] = &sus_func1_pd_2ma_cfg,
		},
	},
	{
		.gpio = 120,
		.settings = {
			[GPIOMUX_ACTIVE]    = &act_gpio_pd_2ma_in_cfg,
			[GPIOMUX_SUSPENDED] = &sus_gpio_pd_2ma_in_cfg,
		},
	},
	{
		.gpio = 121,
		.settings = {
			[GPIOMUX_ACTIVE]    = &act_gpio_np_6ma_out_high_cfg,
			[GPIOMUX_SUSPENDED] = &sus_gpio_np_6ma_out_high_cfg,
		},
	},
	{
		.gpio = 122,
		.settings = {
			[GPIOMUX_ACTIVE]    = &act_func1_np_2ma_cfg,
			[GPIOMUX_SUSPENDED] = &sus_func1_np_2ma_cfg,
		},
	},
	{
		.gpio = 123,
		.settings = {
			[GPIOMUX_ACTIVE]    = &act_func1_np_2ma_cfg,
			[GPIOMUX_SUSPENDED] = &sus_func1_np_2ma_cfg,
		},
	},
	{
		.gpio = 124,
		.settings = {
			[GPIOMUX_ACTIVE]    = &act_gpio_np_6ma_out_low_cfg,
			[GPIOMUX_SUSPENDED] = &sus_gpio_np_6ma_out_low_cfg,
		},
	},
	{
		.gpio = 125,
		.settings = {
			[GPIOMUX_ACTIVE]    = &act_func1_np_2ma_cfg,
			[GPIOMUX_SUSPENDED] = &sus_func1_np_2ma_cfg,
		},
	},
	{
		.gpio = 126,
		.settings = {
			[GPIOMUX_ACTIVE]    = &act_func1_np_2ma_cfg,
			[GPIOMUX_SUSPENDED] = &sus_func1_np_2ma_cfg,
		},
	},
	{
		.gpio = 127,
		.settings = {
			[GPIOMUX_ACTIVE]    = &act_func1_np_2ma_cfg,
			[GPIOMUX_SUSPENDED] = &sus_func1_np_2ma_cfg,
		},
	},
	{
		.gpio = 128,
		.settings = {
			[GPIOMUX_ACTIVE]    = &act_func1_np_2ma_cfg,
			[GPIOMUX_SUSPENDED] = &sus_func1_np_2ma_cfg,
		},
	},
	{
		.gpio = 129,
		.settings = {
			[GPIOMUX_ACTIVE]    = &act_gpio_pd_2ma_in_cfg,
			[GPIOMUX_SUSPENDED] = &sus_gpio_pd_2ma_in_cfg,
		},
	},
	{
		.gpio = 130,
		.settings = {
			[GPIOMUX_ACTIVE]    = &act_gpio_pd_2ma_in_cfg,
			[GPIOMUX_SUSPENDED] = &sus_gpio_pd_2ma_in_cfg,
		},
	},
	{
		.gpio = 131,
		.settings = {
			[GPIOMUX_ACTIVE]    = &act_gpio_pd_2ma_in_cfg,
			[GPIOMUX_SUSPENDED] = &sus_gpio_pd_2ma_in_cfg,
		},
	},
	{
		.gpio = 132,
		.settings = {
			[GPIOMUX_ACTIVE]    = &act_func1_np_2ma_cfg,
			[GPIOMUX_SUSPENDED] = &sus_func1_np_2ma_cfg,
		},
	},
	{
		.gpio = 133,
		.settings = {
			[GPIOMUX_ACTIVE]    = &act_func1_np_2ma_cfg,
			[GPIOMUX_SUSPENDED] = &sus_func1_np_2ma_cfg,
		},
	},
	{
		.gpio = 134,
		.settings = {
			[GPIOMUX_ACTIVE]    = &act_func1_np_2ma_cfg,
			[GPIOMUX_SUSPENDED] = &sus_func1_np_2ma_cfg,
		},
	},
	{
		.gpio = 135,
		.settings = {
			[GPIOMUX_ACTIVE]    = &act_gpio_pd_2ma_in_cfg,
			[GPIOMUX_SUSPENDED] = &sus_gpio_pd_2ma_in_cfg,
		},
	},
	{
		.gpio = 136,
		.settings = {
			[GPIOMUX_ACTIVE]    = &act_func1_np_2ma_cfg,
			[GPIOMUX_SUSPENDED] = &sus_func1_np_2ma_cfg,
		},
	},
	{
		.gpio = 137,
		.settings = {
			[GPIOMUX_ACTIVE]    = &act_func2_np_2ma_cfg,
			[GPIOMUX_SUSPENDED] = &sus_func2_np_2ma_cfg,
		},
	},
	{
		.gpio = 138,
		.settings = {
			[GPIOMUX_ACTIVE]    = &act_gpio_pd_2ma_in_cfg,
			[GPIOMUX_SUSPENDED] = &sus_gpio_pd_2ma_in_cfg,
		},
	},
	{
		.gpio = 139,
		.settings = {
			[GPIOMUX_ACTIVE]    = &act_gpio_pd_2ma_in_cfg,
			[GPIOMUX_SUSPENDED] = &sus_gpio_pd_2ma_in_cfg,
		},
	},
	{
		.gpio = 140,
		.settings = {
			[GPIOMUX_ACTIVE]    = &act_gpio_pd_2ma_in_cfg,
			[GPIOMUX_SUSPENDED] = &sus_gpio_pd_2ma_in_cfg,
		},
	},
	{
		.gpio = 141,
		.settings = {
			[GPIOMUX_ACTIVE]    = &act_func1_pd_2ma_cfg,
			[GPIOMUX_SUSPENDED] = &sus_func1_pd_2ma_cfg,
		},
	},
	{
		.gpio = 142,
		.settings = {
			[GPIOMUX_ACTIVE]    = &act_func1_pd_2ma_cfg,
			[GPIOMUX_SUSPENDED] = &sus_func1_pd_2ma_cfg,
		},
	},
	{
		.gpio = 143,
		.settings = {
			[GPIOMUX_ACTIVE]    = &act_func1_pd_2ma_cfg,
			[GPIOMUX_SUSPENDED] = &sus_func1_pd_2ma_cfg,
		},
	},
	{
		.gpio = 144,
		.settings = {
			[GPIOMUX_ACTIVE]    = &act_gpio_pd_2ma_in_cfg,
			[GPIOMUX_SUSPENDED] = &sus_gpio_pd_2ma_in_cfg,
		},
	},
	{
		.gpio = 145,
		.settings = {
			[GPIOMUX_ACTIVE]    = &act_gpio_pd_2ma_in_cfg,
			[GPIOMUX_SUSPENDED] = &sus_gpio_pd_2ma_in_cfg,
		},
	},
	{
		.gpio = 146,
		.settings = {
			[GPIOMUX_ACTIVE]    = &act_func1_np_2ma_cfg,
			[GPIOMUX_SUSPENDED] = &sus_func1_np_2ma_cfg,
		},
	},
	{
		.gpio = 147,
		.settings = {
			[GPIOMUX_ACTIVE]    = &act_func1_np_2ma_cfg,
			[GPIOMUX_SUSPENDED] = &sus_func1_np_2ma_cfg,
		},
	},
	{
		.gpio = 148,
		.settings = {
			[GPIOMUX_ACTIVE]    = &act_func1_np_2ma_cfg,
			[GPIOMUX_SUSPENDED] = &sus_func1_np_2ma_cfg,
		},
	},
	{
		.gpio = 149,
		.settings = {
			[GPIOMUX_ACTIVE]    = &act_gpio_pd_2ma_in_cfg,
			[GPIOMUX_SUSPENDED] = &sus_gpio_pd_2ma_in_cfg,
		},
	},
	{
		.gpio = 150,
		.settings = {
			[GPIOMUX_ACTIVE]    = &act_gpio_pd_2ma_in_cfg,
			[GPIOMUX_SUSPENDED] = &sus_gpio_pd_2ma_in_cfg,
		},
	},
	{
		.gpio = 151,
		.settings = {
			[GPIOMUX_ACTIVE]    = &act_gpio_pd_2ma_in_cfg,
			[GPIOMUX_SUSPENDED] = &sus_gpio_pd_2ma_in_cfg,
		},
	},
};

struct msm_gpio_out_config {
	unsigned gpio;
	int val;
};

static struct msm_gpio_out_config msm8960_gpio_out_configs[] __initdata = {
	{1, 0},
	{3, 0},
	{4, 0},
	{5, 0},
	{16, 0},
	{37, 0},
	{48, 0},
	{57, 0},
	{92, 0},
	{121, 1},
	{124, 0},
};

static void __init msm_gpio_set_out(struct msm_gpio_out_config *cfgs, unsigned ncfgs)
{
	unsigned n;
	int rc;

	for (n = 0; n < ncfgs; ++n) {
		rc = gpio_request(cfgs[n].gpio, NULL);
		if (!rc) {
			rc = gpio_direction_output(cfgs[n].gpio, cfgs[n].val);
			gpio_free(cfgs[n].gpio);
		}

		if (rc) {
			pr_err("%s: failed to set gpio as output %d: %d\n",
					__func__, cfgs[n].gpio, rc);
		}
	}
}

int __init msm8960_init_gpiomux(void)
{
	int rc = msm_gpiomux_init(NR_GPIO_IRQS);
	if (rc) {
		pr_err(KERN_ERR "msm_gpiomux_init failed %d\n", rc);
		return rc;
	}

	msm_gpiomux_install(msm8960_gpio_configs, ARRAY_SIZE(msm8960_gpio_configs));

	msm_gpio_set_out(msm8960_gpio_out_configs, ARRAY_SIZE(msm8960_gpio_out_configs));

	return 0;
}
