/* drivers/usb/gadget/msm_otg.c
 *
 * Copyright (c) 2009-2012, The Linux Foundation. All rights reserved.
 * Copyright (C) 2013 SHARP CORPORATION
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
#include <linux/device.h>
#include <linux/platform_device.h>
#include <linux/clk.h>
#include <linux/slab.h>
#include <linux/interrupt.h>
#include <linux/err.h>
#include <linux/delay.h>
#include <linux/io.h>
#include <linux/ioport.h>
#include <linux/uaccess.h>
#include <linux/debugfs.h>
#include <linux/seq_file.h>
#include <linux/pm_runtime.h>
#include <linux/of.h>
#include <linux/dma-mapping.h>

#include <linux/usb.h>
#include <linux/usb/otg.h>
#include <linux/usb/ulpi.h>
#include <linux/usb/gadget.h>
#include <linux/usb/hcd.h>
#include <linux/usb/quirks.h>
#include <linux/usb/msm_hsusb.h>
#include <linux/usb/msm_hsusb_hw.h>
#include <linux/regulator/consumer.h>
#include <linux/mfd/pm8xxx/pm8921-charger.h>
#include <linux/mfd/pm8xxx/misc.h>
#include <linux/power_supply.h>

#include <mach/clk.h>
#ifdef CONFIG_USB_SWIC
	#include <sharp/shswic_kerl.h>
#endif /* CONFIG_USB_SWIC */
#ifdef CONFIG_USB_MSM_OTG_SH_CUST_BATTERYLOG
	#include <sharp/shterm_k.h>
#endif /* CONFIG_USB_MSM_OTG_SH_CUST_BATTERYLOG */
#include <mach/mpm.h>
#include <mach/msm_xo.h>
#include <mach/msm_bus.h>
#include <mach/rpm-regulator.h>

#ifdef CONFIG_USB_MSM_OTG_EXT5V_EXCLUSIVE
	#include <sharp/tps61029.h>
#endif /* CONFIG_USB_MSM_OTG_EXT5V_EXCLUSIVE */

#ifdef CONFIG_USB_MSM_OTG_SH_CUST_MHL
	#include <sharp/shmhl_kerl.h>
#endif /* CONFIG_USB_MSM_OTG_SH_CUST_MHL */

#ifdef CONFIG_USB_MSM_OTG_SH_CUST_CHARGE
	#include <sharp/shchg_kerl.h>
#endif /* CONFIG_USB_MSM_OTG_SH_CUST_CHARGE */
#ifdef CONFIG_USB_MSM_OTG_SH_CUST
	#include <sharp/shbatt_kerl.h>
	#include <linux/kobject.h>
#endif /* CONFIG_USB_MSM_OTG_SH_CUST */
#ifdef CONFIG_USB_SH_CUST_NON_STANDARD_CHARGE
	#include <linux/spinlock.h>
#endif /* CONFIG_USB_SH_CUST_NON_STANDARD_CHARGE */

#define MSM_USB_BASE	(motg->regs)
#define DRIVER_NAME	"msm_otg"

#define ID_TIMER_FREQ		(jiffies + msecs_to_jiffies(500))
#define ULPI_IO_TIMEOUT_USEC	(10 * 1000)
#define USB_PHY_3P3_VOL_MIN	3050000 /* uV */
#define USB_PHY_3P3_VOL_MAX	3300000 /* uV */
#define USB_PHY_3P3_HPM_LOAD	50000	/* uA */
#define USB_PHY_3P3_LPM_LOAD	4000	/* uA */

#define USB_PHY_1P8_VOL_MIN	1800000 /* uV */
#define USB_PHY_1P8_VOL_MAX	1800000 /* uV */
#define USB_PHY_1P8_HPM_LOAD	50000	/* uA */
#define USB_PHY_1P8_LPM_LOAD	4000	/* uA */

#define USB_PHY_VDD_DIG_VOL_NONE	0 /*uV */
#define USB_PHY_VDD_DIG_VOL_MIN	1045000 /* uV */
#define USB_PHY_VDD_DIG_VOL_MAX	1320000 /* uV */

#ifdef CONFIG_USB_SH_CUST_NON_STANDARD_CHARGE
#define OTG_CHARGE_TIMER_TIMEOUT (2000) /*ms*/
#endif /* CONFIG_USB_SH_CUST_NON_STANDARD_CHARGE */
static DECLARE_COMPLETION(pmic_vbus_init);
static struct msm_otg *the_msm_otg;
static bool debug_aca_enabled;
static bool debug_bus_voting_enabled;

static struct regulator *hsusb_3p3;
static struct regulator *hsusb_1p8;
static struct regulator *hsusb_vddcx;
static struct regulator *vbus_otg;
static struct regulator *mhl_usb_hs_switch;
static struct power_supply *psy;

#ifdef CONFIG_USB_MSM_OTG_SH_CUST
static u32	shvbus_usb_force_disconnect = 0;
static void msm_sh_wait_cancel(void);
static struct delayed_work sh_usb_wait_work;
static unsigned sh_disconnect_wait_flg = 0;
static unsigned shvbus_charge_wait_val = 0;
static bool shusb_id_interrupts=false;
static struct mutex msm_otg_sm_work_lock;
static int queue_work_cnt = 0;
static int sh_cable_type = 2;
#endif /* CONFIG_USB_MSM_OTG_SH_CUST */
#ifdef CONFIG_USB_SH_CUST_NON_STANDARD_CHARGE
static struct timer_list charge_timer;
static struct work_struct charge_timer_work;
static DEFINE_SPINLOCK(charge_timer_lock);
#endif /* CONFIG_USB_SH_CUST_NON_STANDARD_CHARGE */

#ifdef CONFIG_USB_SWIC

#ifdef CONFIG_USB_MSM_OTG_SH_CUST_SWIC_HOST
	#define USB_SWIC_INTERRUPTS_HOST  SHSWIC_ID_USB_HOST_CABLE
#else /* CONFIG_USB_MSM_OTG_SH_CUST_SWIC_HOST */
	#define USB_SWIC_INTERRUPTS_HOST  0
#endif /* CONFIG_USB_MSM_OTG_SH_CUST_SWIC_HOST */

#ifdef CONFIG_USB_MSM_OTG_SH_CUST_SWIC
	#define USB_SWIC_INTERRUPTS_BYPASS  (SHSWIC_ID_USB_CABLE | SHSWIC_ID_AC_ADAPTER | SHSWIC_ID_IRREGULAR_CHARGER)
#else /* CONFIG_USB_MSM_OTG_SH_CUST_SWIC */
	#define USB_SWIC_INTERRUPTS_BYPASS  0
#endif /* CONFIG_USB_MSM_OTG_SH_CUST_SWIC */

#define USB_SWIC_INTERRUPTS_VALUE	(USB_SWIC_INTERRUPTS_BYPASS | USB_SWIC_INTERRUPTS_HOST | SHSWIC_ID_MHL)

static u8 sh_swic_device = SHSWIC_ID_NONE;
static void msm_otg_swic_irq_start(void *data);
static void msm_otg_swic_ch_inputs(struct msm_otg *motg);

#ifdef CONFIG_USB_MSM_OTG_SH_CUST_SWIC
	static void msm_otg_swic_ch_status(struct msm_otg *motg);
#endif /* CONFIG_USB_MSM_OTG_SH_CUST_SWIC */

#endif /* CONFIG_USB_SWIC */
#ifdef CONFIG_USB_MSM_OTG_SH_CUST_MHL
typedef struct shusb_mhl_status
{
	bool					shusb_mhl_status;
	shmhl_detect_device_t	shusb_mhl_result;
}shusb_mhl_status_t;

static void msm_otg_mhl_irq_start(shmhl_detect_device_t device);
static void msm_otg_mhl_callback(shmhl_detect_device_t device, void* user_data);
static void msm_otg_mhl_power(struct msm_otg *motg, bool on);
static void msm_otg_mhl_irq(void);
static bool msm_otg_mhl_sm_work(struct work_struct *w);

static bool sh_usb_mhl_flag = false;
static shusb_mhl_status_t shusb_read_data;
#endif /* CONFIG_USB_MSM_OTG_SH_CUST_MHL */

#ifdef CONFIG_USB_SH_CUST_NON_STANDARD_CHARGE
static void msm_otg_start_charge_timer(void);
static void msm_otg_stop_charge_timer(void);
#endif /* CONFIG_USB_SH_CUST_NON_STANDARD_CHARGE */

#ifdef CONFIG_USB_MSM_OTG_SH_CUST
static bool msm_otg_id_check(void);
#define OTG_ID_CHECK_RETRY_COUNT     (2)
#define OTG_ID_CHECK_READ_COUNT     (10)
#define OTG_ID_CHECK_SUCCESS_COUNT   (4)
#define OTG_ID_CHECK_MIN_THRESHOLD   (0)
#define OTG_ID_CHECK_MAX_THRESHOLD (373)
#define OTG_ID_CHECK_WAIT_TIME       (1)
#endif /* CONFIG_USB_MSM_OTG_SH_CUST */
#ifdef CONFIG_USB_MSM_OTG_SH_CUST_HW_CHECK
static bool msm_otg_hw_check_init(void);
static void msm_otg_hw_check(void);
static bool shusb_swic_enable=false;
static bool swic_hw_init = false;
#endif /* CONFIG_USB_MSM_OTG_SH_CUST_HW_CHECK */

#ifdef CONFIG_USB_MSM_OTG_SH_CUST_EYEPATTERN
static void msm_otg_set_override(struct otg_ulpi_modify *ulpi_tbl);

#define OTG_ULPI_MODIFY_MAX (255)
static struct otg_ulpi_modify ulpi_val_override[] = {
#ifdef CONFIG_USB_MSM_OTG_SH_CUST_OVERRIDE_HS
	/* PARAMETER_OVERRIDE_B * 1011: + 16% */
	{
		.reg =0x81,
		.mask=0x0F,
		.val =0x0B,
	},
	/* PARAMETER_OVERRIDE_C * 11: - 10% */
	{
		.reg =0x82,
		.mask=0x0C,
		.val =0x0C,
	},
	/* PARAMETER_OVERRIDE_D * 11: Source impedance is decreased by approximately 4 */
	{
		.reg =0x83,
		.mask=0x30,
		.val =0x30,
	},
#endif /* CONFIG_USB_MSM_OTG_SH_CUST_OVERRIDE_HS */
#ifdef CONFIG_USB_MSM_OTG_SH_CUST_OVERRIDE_HS_TYPE_B
	/* PARAMETER_OVERRIDE_B * 1110: + 22% */
	{
		.reg =0x81,
		.mask=0x0F,
		.val =0x0E,
	},
	/* PARAMETER_OVERRIDE_C * 11: - 10% */
	{
		.reg =0x82,
		.mask=0x0C,
		.val =0x0C,
	},
	/* PARAMETER_OVERRIDE_D * 11: Source impedance is decreased by approximately 4 */
	{
		.reg =0x83,
		.mask=0x30,
		.val =0x30,
	},
#endif /* CONFIG_USB_MSM_OTG_SH_CUST_OVERRIDE_HS_TYPE_B */
#ifdef CONFIG_USB_MSM_OTG_SH_CUST_OVERRIDE_HS_TYPE_C
	/* PARAMETER_OVERRIDE_B * 1111: + 24% */
	{
		.reg =0x81,
		.mask=0x0F,
		.val =0x0F,
	},
	/* PARAMETER_OVERRIDE_C * 11: - 10% */
	{
		.reg =0x82,
		.mask=0x0C,
		.val =0x0C,
	},
	/* PARAMETER_OVERRIDE_D * 11: Source impedance is decreased by approximately 4 */
	{
		.reg =0x83,
		.mask=0x30,
		.val =0x30,
	},
#endif /* CONFIG_USB_MSM_OTG_SH_CUST_OVERRIDE_HS_TYPE_C */
	{
		.reg = OTG_ULPI_MODIFY_MAX,
		.mask= OTG_ULPI_MODIFY_MAX,
		.val = OTG_ULPI_MODIFY_MAX,
	},
};
#endif /* CONFIG_USB_MSM_OTG_SH_CUST_EYEPATTERN */

static bool aca_id_turned_on;
static inline bool aca_enabled(void)
{
#ifdef CONFIG_USB_MSM_ACA
	return true;
#else
	return debug_aca_enabled;
#endif
}

#ifdef CONFIG_USB_MSM_OTG_SH_CUST_HW_CHECK
static inline bool swic_enabled(void)
{
	return shusb_swic_enable;
}
#endif /* CONFIG_USB_MSM_OTG_SH_CUST_HW_CHECK */

#ifdef CONFIG_USB_MSM_OTG_SH_CUST
static inline bool is_id_line_low(void)
{
#ifdef CONFIG_USB_SWIC
	if( sh_swic_device == SHSWIC_ID_USB_HOST_CABLE)
		return true;
#endif /* CONFIG_USB_SWIC */

#ifdef CONFIG_USB_MSM_OTG_SH_CUST_HW_CHECK
	if( swic_enabled() ){
		return false;
	}
#endif /* CONFIG_USB_MSM_OTG_SH_CUST_HW_CHECK */
	return msm_otg_id_check();
}
static void msm_otg_sm_work_lockinit(void)
{
	mutex_init(&msm_otg_sm_work_lock);
	queue_work_cnt=0;

}
static bool msm_otg_sm_work_lockinc(void)
{
	bool flg=true;
	mutex_lock(&msm_otg_sm_work_lock);
	if(queue_work_cnt++){
		pr_info("inc check=%d ", queue_work_cnt);
		flg=false;
	}
	mutex_unlock(&msm_otg_sm_work_lock);
	return flg;
}

static bool msm_otg_sm_work_lockdec(void)
{
	bool flg=true;

	mutex_lock(&msm_otg_sm_work_lock);
	if(--queue_work_cnt){
		pr_info("dec check=%d ", queue_work_cnt);
		flg=false;
		queue_work_cnt = 0;
	}
	mutex_unlock(&msm_otg_sm_work_lock);

	return flg;
}
#endif /* CONFIG_USB_MSM_OTG_SH_CUST */

static const int vdd_val[VDD_TYPE_MAX][VDD_VAL_MAX] = {
		{  /* VDD_CX CORNER Voting */
			[VDD_NONE]	= RPM_VREG_CORNER_NONE,
			[VDD_MIN]	= RPM_VREG_CORNER_NOMINAL,
			[VDD_MAX]	= RPM_VREG_CORNER_HIGH,
		},
		{ /* VDD_CX Voltage Voting */
			[VDD_NONE]	= USB_PHY_VDD_DIG_VOL_NONE,
			[VDD_MIN]	= USB_PHY_VDD_DIG_VOL_MIN,
			[VDD_MAX]	= USB_PHY_VDD_DIG_VOL_MAX,
		},
};

static int msm_hsusb_ldo_init(struct msm_otg *motg, int init)
{
	int rc = 0;

	if (init) {
		hsusb_3p3 = devm_regulator_get(motg->phy.dev, "HSUSB_3p3");
		if (IS_ERR(hsusb_3p3)) {
			dev_err(motg->phy.dev, "unable to get hsusb 3p3\n");
			return PTR_ERR(hsusb_3p3);
		}

		rc = regulator_set_voltage(hsusb_3p3, USB_PHY_3P3_VOL_MIN,
				USB_PHY_3P3_VOL_MAX);
		if (rc) {
			dev_err(motg->phy.dev, "unable to set voltage level for"
					"hsusb 3p3\n");
			return rc;
		}
		hsusb_1p8 = devm_regulator_get(motg->phy.dev, "HSUSB_1p8");
		if (IS_ERR(hsusb_1p8)) {
			dev_err(motg->phy.dev, "unable to get hsusb 1p8\n");
			rc = PTR_ERR(hsusb_1p8);
			goto put_3p3_lpm;
		}
		rc = regulator_set_voltage(hsusb_1p8, USB_PHY_1P8_VOL_MIN,
				USB_PHY_1P8_VOL_MAX);
		if (rc) {
			dev_err(motg->phy.dev, "unable to set voltage level for"
					"hsusb 1p8\n");
			goto put_1p8;
		}

		return 0;
	}

put_1p8:
	regulator_set_voltage(hsusb_1p8, 0, USB_PHY_1P8_VOL_MAX);
put_3p3_lpm:
	regulator_set_voltage(hsusb_3p3, 0, USB_PHY_3P3_VOL_MAX);
	return rc;
}

static int msm_hsusb_config_vddcx(int high)
{
	struct msm_otg *motg = the_msm_otg;
	enum usb_vdd_type vdd_type = motg->vdd_type;
	int max_vol = vdd_val[vdd_type][VDD_MAX];
	int min_vol;
	int ret;

	min_vol = vdd_val[vdd_type][!!high];
	ret = regulator_set_voltage(hsusb_vddcx, min_vol, max_vol);
	if (ret) {
		pr_err("%s: unable to set the voltage for regulator "
			"HSUSB_VDDCX\n", __func__);
		return ret;
	}

	pr_debug("%s: min_vol:%d max_vol:%d\n", __func__, min_vol, max_vol);

	return ret;
}

static int msm_hsusb_ldo_enable(struct msm_otg *motg, int on)
{
	int ret = 0;

	if (IS_ERR(hsusb_1p8)) {
		pr_err("%s: HSUSB_1p8 is not initialized\n", __func__);
		return -ENODEV;
	}

	if (IS_ERR(hsusb_3p3)) {
		pr_err("%s: HSUSB_3p3 is not initialized\n", __func__);
		return -ENODEV;
	}

	if (on) {
		ret = regulator_set_optimum_mode(hsusb_1p8,
				USB_PHY_1P8_HPM_LOAD);
		if (ret < 0) {
			pr_err("%s: Unable to set HPM of the regulator:"
				"HSUSB_1p8\n", __func__);
			return ret;
		}

		ret = regulator_enable(hsusb_1p8);
		if (ret) {
			dev_err(motg->phy.dev, "%s: unable to enable the hsusb 1p8\n",
				__func__);
			regulator_set_optimum_mode(hsusb_1p8, 0);
			return ret;
		}

		ret = regulator_set_optimum_mode(hsusb_3p3,
				USB_PHY_3P3_HPM_LOAD);
		if (ret < 0) {
			pr_err("%s: Unable to set HPM of the regulator:"
				"HSUSB_3p3\n", __func__);
			regulator_set_optimum_mode(hsusb_1p8, 0);
			regulator_disable(hsusb_1p8);
			return ret;
		}

		ret = regulator_enable(hsusb_3p3);
		if (ret) {
			dev_err(motg->phy.dev, "%s: unable to enable the hsusb 3p3\n",
				__func__);
			regulator_set_optimum_mode(hsusb_3p3, 0);
			regulator_set_optimum_mode(hsusb_1p8, 0);
			regulator_disable(hsusb_1p8);
			return ret;
		}

	} else {
		ret = regulator_disable(hsusb_1p8);
		if (ret) {
			dev_err(motg->phy.dev, "%s: unable to disable the hsusb 1p8\n",
				__func__);
			return ret;
		}

		ret = regulator_set_optimum_mode(hsusb_1p8, 0);
		if (ret < 0)
			pr_err("%s: Unable to set LPM of the regulator:"
				"HSUSB_1p8\n", __func__);

		ret = regulator_disable(hsusb_3p3);
		if (ret) {
			dev_err(motg->phy.dev, "%s: unable to disable the hsusb 3p3\n",
				 __func__);
			return ret;
		}
		ret = regulator_set_optimum_mode(hsusb_3p3, 0);
		if (ret < 0)
			pr_err("%s: Unable to set LPM of the regulator:"
				"HSUSB_3p3\n", __func__);
	}

	pr_debug("reg (%s)\n", on ? "HPM" : "LPM");
	return ret < 0 ? ret : 0;
}

static void msm_hsusb_mhl_switch_enable(struct msm_otg *motg, bool on)
{
	struct msm_otg_platform_data *pdata = motg->pdata;

	if (!pdata->mhl_enable)
		return;

	if (!mhl_usb_hs_switch) {
		pr_err("%s: mhl_usb_hs_switch is NULL.\n", __func__);
		return;
	}

	if (on) {
		if (regulator_enable(mhl_usb_hs_switch))
			pr_err("unable to enable mhl_usb_hs_switch\n");
	} else {
		regulator_disable(mhl_usb_hs_switch);
	}
}

static int ulpi_read(struct usb_phy *phy, u32 reg)
{
	struct msm_otg *motg = container_of(phy, struct msm_otg, phy);
	int cnt = 0;

	/* initiate read operation */
	writel(ULPI_RUN | ULPI_READ | ULPI_ADDR(reg),
	       USB_ULPI_VIEWPORT);

	/* wait for completion */
	while (cnt < ULPI_IO_TIMEOUT_USEC) {
		if (!(readl(USB_ULPI_VIEWPORT) & ULPI_RUN))
			break;
		udelay(1);
		cnt++;
	}

	if (cnt >= ULPI_IO_TIMEOUT_USEC) {
		dev_err(phy->dev, "ulpi_read: timeout %08x\n",
			readl(USB_ULPI_VIEWPORT));
		return -ETIMEDOUT;
	}
	return ULPI_DATA_READ(readl(USB_ULPI_VIEWPORT));
}

static int ulpi_write(struct usb_phy *phy, u32 val, u32 reg)
{
	struct msm_otg *motg = container_of(phy, struct msm_otg, phy);
	int cnt = 0;

	/* initiate write operation */
	writel(ULPI_RUN | ULPI_WRITE |
	       ULPI_ADDR(reg) | ULPI_DATA(val),
	       USB_ULPI_VIEWPORT);

	/* wait for completion */
	while (cnt < ULPI_IO_TIMEOUT_USEC) {
		if (!(readl(USB_ULPI_VIEWPORT) & ULPI_RUN))
			break;
		udelay(1);
		cnt++;
	}

	if (cnt >= ULPI_IO_TIMEOUT_USEC) {
		dev_err(phy->dev, "ulpi_write: timeout\n");
		return -ETIMEDOUT;
	}
	return 0;
}

static struct usb_phy_io_ops msm_otg_io_ops = {
	.read = ulpi_read,
	.write = ulpi_write,
};

static void ulpi_init(struct msm_otg *motg)
{
	struct msm_otg_platform_data *pdata = motg->pdata;
	int *seq = pdata->phy_init_seq;

	if (!seq)
		return;

	while (seq[0] >= 0) {
		dev_vdbg(motg->phy.dev, "ulpi: write 0x%02x to 0x%02x\n",
				seq[0], seq[1]);
		ulpi_write(&motg->phy, seq[0], seq[1]);
		seq += 2;
	}
}

static int msm_otg_link_clk_reset(struct msm_otg *motg, bool assert)
{
	int ret;

	if (IS_ERR(motg->clk))
		return 0;

	if (assert) {
		ret = clk_reset(motg->clk, CLK_RESET_ASSERT);
		if (ret)
			dev_err(motg->phy.dev, "usb hs_clk assert failed\n");
	} else {
		ret = clk_reset(motg->clk, CLK_RESET_DEASSERT);
		if (ret)
			dev_err(motg->phy.dev, "usb hs_clk deassert failed\n");
	}
	return ret;
}

static int msm_otg_phy_clk_reset(struct msm_otg *motg)
{
	int ret;

	if (IS_ERR(motg->phy_reset_clk))
		return 0;

	ret = clk_reset(motg->phy_reset_clk, CLK_RESET_ASSERT);
	if (ret) {
		dev_err(motg->phy.dev, "usb phy clk assert failed\n");
		return ret;
	}
	usleep_range(10000, 12000);
	ret = clk_reset(motg->phy_reset_clk, CLK_RESET_DEASSERT);
	if (ret)
		dev_err(motg->phy.dev, "usb phy clk deassert failed\n");
	return ret;
}

static int msm_otg_phy_reset(struct msm_otg *motg)
{
	u32 val;
	int ret;
	int retries;

	ret = msm_otg_link_clk_reset(motg, 1);
	if (ret)
		return ret;
	ret = msm_otg_phy_clk_reset(motg);
	if (ret)
		return ret;
	ret = msm_otg_link_clk_reset(motg, 0);
	if (ret)
		return ret;

	val = readl(USB_PORTSC) & ~PORTSC_PTS_MASK;
	writel(val | PORTSC_PTS_ULPI, USB_PORTSC);

	for (retries = 3; retries > 0; retries--) {
		ret = ulpi_write(&motg->phy, ULPI_FUNC_CTRL_SUSPENDM,
				ULPI_CLR(ULPI_FUNC_CTRL));
		if (!ret)
			break;
		ret = msm_otg_phy_clk_reset(motg);
		if (ret)
			return ret;
	}
	if (!retries)
		return -ETIMEDOUT;

	/* This reset calibrates the phy, if the above write succeeded */
	ret = msm_otg_phy_clk_reset(motg);
	if (ret)
		return ret;

	for (retries = 3; retries > 0; retries--) {
		ret = ulpi_read(&motg->phy, ULPI_DEBUG);
		if (ret != -ETIMEDOUT)
			break;
		ret = msm_otg_phy_clk_reset(motg);
		if (ret)
			return ret;
	}
	if (!retries)
		return -ETIMEDOUT;

#ifdef CONFIG_USB_DEBUG_SH_LOG
	dev_info(motg->phy.dev, "phy_reset: success\n");
#endif /* CONFIG_USB_DEBUG_SH_LOG */
	return 0;
}

#define LINK_RESET_TIMEOUT_USEC		(250 * 1000)
static int msm_otg_link_reset(struct msm_otg *motg)
{
	int cnt = 0;

	writel_relaxed(USBCMD_RESET, USB_USBCMD);
	while (cnt < LINK_RESET_TIMEOUT_USEC) {
		if (!(readl_relaxed(USB_USBCMD) & USBCMD_RESET))
			break;
		udelay(1);
		cnt++;
	}
	if (cnt >= LINK_RESET_TIMEOUT_USEC)
		return -ETIMEDOUT;

	/* select ULPI phy */
	writel_relaxed(0x80000000, USB_PORTSC);
	writel_relaxed(0x0, USB_AHBBURST);
	writel_relaxed(0x08, USB_AHBMODE);

	return 0;
}

static int msm_otg_reset(struct usb_phy *phy)
{
	struct msm_otg *motg = container_of(phy, struct msm_otg, phy);
	struct msm_otg_platform_data *pdata = motg->pdata;
	int ret;
	u32 val = 0;
	u32 ulpi_val = 0;

	/*
	 * USB PHY and Link reset also reset the USB BAM.
	 * Thus perform reset operation only once to avoid
	 * USB BAM reset on other cases e.g. USB cable disconnections.
	 */
	if (pdata->disable_reset_on_disconnect) {
		if (motg->reset_counter)
			return 0;
		else
			motg->reset_counter++;
	}

	if (!IS_ERR(motg->clk))
		clk_prepare_enable(motg->clk);
	ret = msm_otg_phy_reset(motg);
	if (ret) {
		dev_err(phy->dev, "phy_reset failed\n");
		return ret;
	}

	aca_id_turned_on = false;
	ret = msm_otg_link_reset(motg);
	if (ret) {
		dev_err(phy->dev, "link reset failed\n");
		return ret;
	}
	msleep(100);

	ulpi_init(motg);

	/* Ensure that RESET operation is completed before turning off clock */
	mb();

	if (!IS_ERR(motg->clk))
		clk_disable_unprepare(motg->clk);

	if (pdata->otg_control == OTG_PHY_CONTROL) {
		val = readl_relaxed(USB_OTGSC);
		if (pdata->mode == USB_OTG) {
#ifdef CONFIG_USB_MSM_OTG_SH_CUST_SWIC_HOST
			ulpi_val = ULPI_INT_SESS_VALID;
			val |= OTGSC_BSVIE;
#else /* CONFIG_USB_MSM_OTG_SH_CUST_SWIC_HOST */
			ulpi_val = ULPI_INT_IDGRD | ULPI_INT_SESS_VALID;
			val |= OTGSC_IDIE | OTGSC_BSVIE;
#endif /* CONFIG_USB_MSM_OTG_SH_CUST_SWIC_HOST */
		} else if (pdata->mode == USB_PERIPHERAL) {
			ulpi_val = ULPI_INT_SESS_VALID;
			val |= OTGSC_BSVIE;
		}
		writel_relaxed(val, USB_OTGSC);
		ulpi_write(phy, ulpi_val, ULPI_USB_INT_EN_RISE);
		ulpi_write(phy, ulpi_val, ULPI_USB_INT_EN_FALL);
	} else if (pdata->otg_control == OTG_PMIC_CONTROL) {
		ulpi_write(phy, OTG_COMP_DISABLE,
			ULPI_SET(ULPI_PWR_CLK_MNG_REG));
#ifndef CONFIG_USB_MSM_OTG_SH_CUST_SWIC
		/* Enable PMIC pull-up */
		pm8xxx_usb_id_pullup(1);
#endif /* CONFIG_USB_MSM_OTG_SH_CUST_SWIC */
	}

#ifdef CONFIG_USB_MSM_OTG_SH_CUST_EYEPATTERN
	if( motg->ulpi_write_modify)
		msm_otg_set_override(motg->ulpi_write_modify );
#endif /* CONFIG_USB_MSM_OTG_SH_CUST_EYEPATTERN */

	return 0;
}

static const char *timer_string(int bit)
{
	switch (bit) {
	case A_WAIT_VRISE:		return "a_wait_vrise";
	case A_WAIT_VFALL:		return "a_wait_vfall";
	case B_SRP_FAIL:		return "b_srp_fail";
	case A_WAIT_BCON:		return "a_wait_bcon";
	case A_AIDL_BDIS:		return "a_aidl_bdis";
	case A_BIDL_ADIS:		return "a_bidl_adis";
	case B_ASE0_BRST:		return "b_ase0_brst";
	case A_TST_MAINT:		return "a_tst_maint";
	case B_TST_SRP:			return "b_tst_srp";
	case B_TST_CONFIG:		return "b_tst_config";
	default:			return "UNDEFINED";
	}
}

static enum hrtimer_restart msm_otg_timer_func(struct hrtimer *hrtimer)
{
	struct msm_otg *motg = container_of(hrtimer, struct msm_otg, timer);

	switch (motg->active_tmout) {
	case A_WAIT_VRISE:
		/* TODO: use vbus_vld interrupt */
		set_bit(A_VBUS_VLD, &motg->inputs);
		break;
	case A_TST_MAINT:
		/* OTG PET: End session after TA_TST_MAINT */
		set_bit(A_BUS_DROP, &motg->inputs);
		break;
	case B_TST_SRP:
		/*
		 * OTG PET: Initiate SRP after TB_TST_SRP of
		 * previous session end.
		 */
		set_bit(B_BUS_REQ, &motg->inputs);
		break;
	case B_TST_CONFIG:
		clear_bit(A_CONN, &motg->inputs);
		break;
	default:
		set_bit(motg->active_tmout, &motg->tmouts);
	}

	pr_debug("expired %s timer\n", timer_string(motg->active_tmout));
	queue_work(system_nrt_wq, &motg->sm_work);
	return HRTIMER_NORESTART;
}

static void msm_otg_del_timer(struct msm_otg *motg)
{
	int bit = motg->active_tmout;

	pr_debug("deleting %s timer. remaining %lld msec\n", timer_string(bit),
			div_s64(ktime_to_us(hrtimer_get_remaining(
					&motg->timer)), 1000));
	hrtimer_cancel(&motg->timer);
	clear_bit(bit, &motg->tmouts);
}

static void msm_otg_start_timer(struct msm_otg *motg, int time, int bit)
{
	clear_bit(bit, &motg->tmouts);
	motg->active_tmout = bit;
	pr_debug("starting %s timer\n", timer_string(bit));
	hrtimer_start(&motg->timer,
			ktime_set(time / 1000, (time % 1000) * 1000000),
			HRTIMER_MODE_REL);
}

static void msm_otg_init_timer(struct msm_otg *motg)
{
	hrtimer_init(&motg->timer, CLOCK_MONOTONIC, HRTIMER_MODE_REL);
	motg->timer.function = msm_otg_timer_func;
}

static int msm_otg_start_hnp(struct usb_otg *otg)
{
	struct msm_otg *motg = container_of(otg->phy, struct msm_otg, phy);

	if (otg->phy->state != OTG_STATE_A_HOST) {
		pr_err("HNP can not be initiated in %s state\n",
				otg_state_string(otg->phy->state));
		return -EINVAL;
	}

	pr_debug("A-Host: HNP initiated\n");
	clear_bit(A_BUS_REQ, &motg->inputs);
	queue_work(system_nrt_wq, &motg->sm_work);
	return 0;
}

static int msm_otg_start_srp(struct usb_otg *otg)
{
	struct msm_otg *motg = container_of(otg->phy, struct msm_otg, phy);
	u32 val;
	int ret = 0;

	if (otg->phy->state != OTG_STATE_B_IDLE) {
		pr_err("SRP can not be initiated in %s state\n",
				otg_state_string(otg->phy->state));
		ret = -EINVAL;
		goto out;
	}

	if ((jiffies - motg->b_last_se0_sess) < msecs_to_jiffies(TB_SRP_INIT)) {
		pr_debug("initial conditions of SRP are not met. Try again"
				"after some time\n");
		ret = -EAGAIN;
		goto out;
	}

	pr_debug("B-Device SRP started\n");

	/*
	 * PHY won't pull D+ high unless it detects Vbus valid.
	 * Since by definition, SRP is only done when Vbus is not valid,
	 * software work-around needs to be used to spoof the PHY into
	 * thinking it is valid. This can be done using the VBUSVLDEXTSEL and
	 * VBUSVLDEXT register bits.
	 */
	ulpi_write(otg->phy, 0x03, 0x97);
	/*
	 * Harware auto assist data pulsing: Data pulse is given
	 * for 7msec; wait for vbus
	 */
	val = readl_relaxed(USB_OTGSC);
	writel_relaxed((val & ~OTGSC_INTSTS_MASK) | OTGSC_HADP, USB_OTGSC);

	/* VBUS plusing is obsoleted in OTG 2.0 supplement */
out:
	return ret;
}

static void msm_otg_host_hnp_enable(struct usb_otg *otg, bool enable)
{
	struct usb_hcd *hcd = bus_to_hcd(otg->host);
	struct usb_device *rhub = otg->host->root_hub;

	if (enable) {
		pm_runtime_disable(&rhub->dev);
		rhub->state = USB_STATE_NOTATTACHED;
		hcd->driver->bus_suspend(hcd);
		clear_bit(HCD_FLAG_HW_ACCESSIBLE, &hcd->flags);
	} else {
		usb_remove_hcd(hcd);
		msm_otg_reset(otg->phy);
		usb_add_hcd(hcd, hcd->irq, IRQF_SHARED);
	}
}

static int msm_otg_set_suspend(struct usb_phy *phy, int suspend)
{
	struct msm_otg *motg = container_of(phy, struct msm_otg, phy);

	if (aca_enabled())
		return 0;

	if (atomic_read(&motg->in_lpm) == suspend)
		return 0;

	if (suspend) {
		switch (phy->state) {
		case OTG_STATE_A_WAIT_BCON:
			if (TA_WAIT_BCON > 0)
				break;
			/* fall through */
		case OTG_STATE_A_HOST:
			pr_debug("host bus suspend\n");
			clear_bit(A_BUS_REQ, &motg->inputs);
			queue_work(system_nrt_wq, &motg->sm_work);
			break;
		case OTG_STATE_B_PERIPHERAL:
			pr_debug("peripheral bus suspend\n");
			if (!(motg->caps & ALLOW_LPM_ON_DEV_SUSPEND))
				break;
			set_bit(A_BUS_SUSPEND, &motg->inputs);
			queue_work(system_nrt_wq, &motg->sm_work);
			break;

		default:
			break;
		}
	} else {
		switch (phy->state) {
		case OTG_STATE_A_SUSPEND:
			/* Remote wakeup or resume */
			set_bit(A_BUS_REQ, &motg->inputs);
			phy->state = OTG_STATE_A_HOST;

			/* ensure hardware is not in low power mode */
			pm_runtime_resume(phy->dev);
			break;
		case OTG_STATE_B_PERIPHERAL:
			pr_debug("peripheral bus resume\n");
			if (!(motg->caps & ALLOW_LPM_ON_DEV_SUSPEND))
				break;
			clear_bit(A_BUS_SUSPEND, &motg->inputs);
			queue_work(system_nrt_wq, &motg->sm_work);
			break;
		default:
			break;
		}
	}
	return 0;
}

#define PHY_SUSPEND_TIMEOUT_USEC	(500 * 1000)
#define PHY_RESUME_TIMEOUT_USEC	(100 * 1000)

#ifdef CONFIG_USB_MSM_OTG_SH_CUST_SWIC
static irqreturn_t msm_otg_irq(int irq, void *data);
#endif /* CONFIG_USB_MSM_OTG_SH_CUST_SWIC */

#ifdef CONFIG_PM_SLEEP
static int msm_otg_suspend(struct msm_otg *motg)
{
	struct usb_phy *phy = &motg->phy;
	struct usb_bus *bus = phy->otg->host;
	struct msm_otg_platform_data *pdata = motg->pdata;
	int cnt = 0;
	bool host_bus_suspend, device_bus_suspend, dcp;
	u32 phy_ctrl_val = 0, cmd_val;
	unsigned ret;
	u32 portsc;

	if (atomic_read(&motg->in_lpm))
		return 0;

#ifndef CONFIG_USB_MSM_OTG_SH_CUST_SWIC
	disable_irq(motg->irq);
#endif /* CONFIG_USB_MSM_OTG_SH_CUST_SWIC*/
	host_bus_suspend = phy->otg->host && !test_bit(ID, &motg->inputs);
	device_bus_suspend = phy->otg->gadget && test_bit(ID, &motg->inputs) &&
		test_bit(A_BUS_SUSPEND, &motg->inputs) &&
		motg->caps & ALLOW_LPM_ON_DEV_SUSPEND;
	dcp = motg->chg_type == USB_DCP_CHARGER;

	/* charging detection in progress due to cable plug-in */
	if (test_bit(B_SESS_VLD, &motg->inputs) && !device_bus_suspend &&
		!dcp) {
		enable_irq(motg->irq);
		return -EBUSY;
	}

	/*
	 * Chipidea 45-nm PHY suspend sequence:
	 *
	 * Interrupt Latch Register auto-clear feature is not present
	 * in all PHY versions. Latch register is clear on read type.
	 * Clear latch register to avoid spurious wakeup from
	 * low power mode (LPM).
	 *
	 * PHY comparators are disabled when PHY enters into low power
	 * mode (LPM). Keep PHY comparators ON in LPM only when we expect
	 * VBUS/Id notifications from USB PHY. Otherwise turn off USB
	 * PHY comparators. This save significant amount of power.
	 *
	 * PLL is not turned off when PHY enters into low power mode (LPM).
	 * Disable PLL for maximum power savings.
	 */

	if (motg->pdata->phy_type == CI_45NM_INTEGRATED_PHY) {
		ulpi_read(phy, 0x14);
		if (pdata->otg_control == OTG_PHY_CONTROL)
			ulpi_write(phy, 0x01, 0x30);
		ulpi_write(phy, 0x08, 0x09);
	}


	/* Set the PHCD bit, only if it is not set by the controller.
	 * PHY may take some time or even fail to enter into low power
	 * mode (LPM). Hence poll for 500 msec and reset the PHY and link
	 * in failure case.
	 */
	portsc = readl_relaxed(USB_PORTSC);
	if (!(portsc & PORTSC_PHCD)) {
		writel_relaxed(portsc | PORTSC_PHCD,
				USB_PORTSC);
		while (cnt < PHY_SUSPEND_TIMEOUT_USEC) {
			if (readl_relaxed(USB_PORTSC) & PORTSC_PHCD)
				break;
			udelay(1);
			cnt++;
		}
	}

	if (cnt >= PHY_SUSPEND_TIMEOUT_USEC) {
		dev_err(phy->dev, "Unable to suspend PHY\n");
		msm_otg_reset(phy);
#ifndef CONFIG_USB_MSM_OTG_SH_CUST_SWIC
		enable_irq(motg->irq);
#endif /* CONFIG_USB_MSM_OTG_SH_CUST_SWIC */
		return -ETIMEDOUT;
	}

	/*
	 * PHY has capability to generate interrupt asynchronously in low
	 * power mode (LPM). This interrupt is level triggered. So USB IRQ
	 * line must be disabled till async interrupt enable bit is cleared
	 * in USBCMD register. Assert STP (ULPI interface STOP signal) to
	 * block data communication from PHY.
	 *
	 * PHY retention mode is disallowed while entering to LPM with wall
	 * charger connected.  But PHY is put into suspend mode. Hence
	 * enable asynchronous interrupt to detect charger disconnection when
	 * PMIC notifications are unavailable.
	 */
	cmd_val = readl_relaxed(USB_USBCMD);
	if (host_bus_suspend || device_bus_suspend ||
		(motg->pdata->otg_control == OTG_PHY_CONTROL && dcp))
		cmd_val |= ASYNC_INTR_CTRL | ULPI_STP_CTRL;
	else
		cmd_val |= ULPI_STP_CTRL;
	writel_relaxed(cmd_val, USB_USBCMD);

	/*
	 * BC1.2 spec mandates PD to enable VDP_SRC when charging from DCP.
	 * PHY retention and collapse can not happen with VDP_SRC enabled.
	 */
	if (motg->caps & ALLOW_PHY_RETENTION && !host_bus_suspend &&
		!device_bus_suspend && !dcp) {
		phy_ctrl_val = readl_relaxed(USB_PHY_CTRL);
		if (motg->pdata->otg_control == OTG_PHY_CONTROL)
			/* Enable PHY HV interrupts to wake MPM/Link */
			phy_ctrl_val |=
				(PHY_IDHV_INTEN | PHY_OTGSESSVLDHV_INTEN);

		writel_relaxed(phy_ctrl_val & ~PHY_RETEN, USB_PHY_CTRL);
		motg->lpm_flags |= PHY_RETENTIONED;
	}

	/* Ensure that above operation is completed before turning off clocks */
	mb();
	if (!motg->pdata->core_clk_always_on_workaround) {
		clk_disable_unprepare(motg->pclk);
		clk_disable_unprepare(motg->core_clk);
	}

	/* usb phy no more require TCXO clock, hence vote for TCXO disable */
	if (!host_bus_suspend) {
		ret = msm_xo_mode_vote(motg->xo_handle, MSM_XO_MODE_OFF);
		if (ret)
			dev_err(phy->dev, "%s failed to devote for "
				"TCXO D0 buffer%d\n", __func__, ret);
		else
			motg->lpm_flags |= XO_SHUTDOWN;
	}

	if (motg->caps & ALLOW_PHY_POWER_COLLAPSE &&
			!host_bus_suspend && !dcp) {
		msm_hsusb_ldo_enable(motg, 0);
		motg->lpm_flags |= PHY_PWR_COLLAPSED;
	}

	if (motg->lpm_flags & PHY_RETENTIONED) {
		msm_hsusb_config_vddcx(0);
		msm_hsusb_mhl_switch_enable(motg, 0);
	}

	if (device_may_wakeup(phy->dev)) {
		enable_irq_wake(motg->irq);
		if (motg->pdata->pmic_id_irq)
			enable_irq_wake(motg->pdata->pmic_id_irq);
		if (pdata->otg_control == OTG_PHY_CONTROL &&
			pdata->mpm_otgsessvld_int)
			msm_mpm_set_pin_wake(pdata->mpm_otgsessvld_int, 1);
	}
	if (bus) {
		clear_bit(HCD_FLAG_HW_ACCESSIBLE, &(bus_to_hcd(bus))->flags);
#ifdef CONFIG_USB_MSM_OTG_SH_CUST_SWIC
		dev_dbg(phy->dev, "request_irq msm_otg_irq\n");
		ret = request_irq(motg->irq, msm_otg_irq, IRQF_SHARED,
						"msm_otg", motg);
		if (ret) {
			dev_err(phy->dev, "request irq failed\n");
		}
#endif /* CONFIG_USB_MSM_OTG_SH_CUST_SWIC */
	}

#ifdef CONFIG_USB_MSM_OTG_SH_CUST_MHL
	if( shusb_read_data.shusb_mhl_result != SHMHL_DEVICE_MHL)
		msm_otg_mhl_power( motg, false );
#endif /* CONFIG_USB_MSM_OTG_SH_CUST_MHL */

	atomic_set(&motg->in_lpm, 1);
#ifndef CONFIG_USB_MSM_OTG_SH_CUST_SWIC
	enable_irq(motg->irq);
#endif /* CONFIG_USB_MSM_OTG_SH_CUST_SWIC */
	wake_unlock(&motg->wlock);

	dev_info(phy->dev, "USB in low power mode\n");

	return 0;
}

static int msm_otg_resume(struct msm_otg *motg)
{
	struct usb_phy *phy = &motg->phy;
	struct usb_bus *bus = phy->otg->host;
	struct msm_otg_platform_data *pdata = motg->pdata;
	int cnt = 0;
	unsigned temp;
	u32 phy_ctrl_val = 0;
	unsigned ret;

	if (!atomic_read(&motg->in_lpm))
		return 0;

	wake_lock(&motg->wlock);

	/* Vote for TCXO when waking up the phy */
	if (motg->lpm_flags & XO_SHUTDOWN) {
		ret = msm_xo_mode_vote(motg->xo_handle, MSM_XO_MODE_ON);
		if (ret)
			dev_err(phy->dev, "%s failed to vote for "
				"TCXO D0 buffer%d\n", __func__, ret);
		motg->lpm_flags &= ~XO_SHUTDOWN;
	}

	if (!motg->pdata->core_clk_always_on_workaround) {
		clk_prepare_enable(motg->core_clk);
		clk_prepare_enable(motg->pclk);
	}

	if (motg->lpm_flags & PHY_PWR_COLLAPSED) {
		msm_hsusb_ldo_enable(motg, 1);
		motg->lpm_flags &= ~PHY_PWR_COLLAPSED;
	}

	if (motg->lpm_flags & PHY_RETENTIONED) {
		msm_hsusb_mhl_switch_enable(motg, 1);
		msm_hsusb_config_vddcx(1);
		phy_ctrl_val = readl_relaxed(USB_PHY_CTRL);
		phy_ctrl_val |= PHY_RETEN;
		if (motg->pdata->otg_control == OTG_PHY_CONTROL)
			/* Disable PHY HV interrupts */
			phy_ctrl_val &=
				~(PHY_IDHV_INTEN | PHY_OTGSESSVLDHV_INTEN);
		writel_relaxed(phy_ctrl_val, USB_PHY_CTRL);
		motg->lpm_flags &= ~PHY_RETENTIONED;
	}

	temp = readl(USB_USBCMD);
	temp &= ~ASYNC_INTR_CTRL;
	temp &= ~ULPI_STP_CTRL;
	writel(temp, USB_USBCMD);

	/*
	 * PHY comes out of low power mode (LPM) in case of wakeup
	 * from asynchronous interrupt.
	 */
	if (!(readl(USB_PORTSC) & PORTSC_PHCD))
		goto skip_phy_resume;

	writel(readl(USB_PORTSC) & ~PORTSC_PHCD, USB_PORTSC);
	while (cnt < PHY_RESUME_TIMEOUT_USEC) {
		if (!(readl(USB_PORTSC) & PORTSC_PHCD))
			break;
		udelay(1);
		cnt++;
	}

	if (cnt >= PHY_RESUME_TIMEOUT_USEC) {
		/*
		 * This is a fatal error. Reset the link and
		 * PHY. USB state can not be restored. Re-insertion
		 * of USB cable is the only way to get USB working.
		 */
		dev_err(phy->dev, "Unable to resume USB."
				"Re-plugin the cable\n");
		msm_otg_reset(phy);
	}

skip_phy_resume:
	if (device_may_wakeup(phy->dev)) {
		disable_irq_wake(motg->irq);
		if (motg->pdata->pmic_id_irq)
			disable_irq_wake(motg->pdata->pmic_id_irq);
		if (pdata->otg_control == OTG_PHY_CONTROL &&
			pdata->mpm_otgsessvld_int)
			msm_mpm_set_pin_wake(pdata->mpm_otgsessvld_int, 0);
	}
	if (bus) {
#ifdef CONFIG_USB_MSM_OTG_SH_CUST_SWIC
		free_irq(motg->irq, motg);
#endif /* CONFIG_USB_MSM_OTG_SH_CUST_SWIC */
		set_bit(HCD_FLAG_HW_ACCESSIBLE, &(bus_to_hcd(bus))->flags);
	}

#ifdef CONFIG_USB_MSM_OTG_SH_CUST_MHL
	msm_otg_mhl_power( motg, true );
#endif /* CONFIG_USB_MSM_OTG_SH_CUST_MHL */

	atomic_set(&motg->in_lpm, 0);

	if (motg->async_int) {
		motg->async_int = 0;
		enable_irq(motg->irq);
	}

	dev_info(phy->dev, "USB exited from low power mode\n");

	return 0;
}
#endif

static int msm_otg_notify_host_mode(struct msm_otg *motg, bool host_mode)
{
	if (!psy)
		goto psy_not_supported;

	if (host_mode)
		power_supply_set_scope(psy, POWER_SUPPLY_SCOPE_SYSTEM);
	else
		power_supply_set_scope(psy, POWER_SUPPLY_SCOPE_DEVICE);

psy_not_supported:
	dev_dbg(motg->phy.dev, "Power Supply doesn't support USB charger\n");
	return -ENXIO;
}

#ifdef CONFIG_USB_MSM_OTG_SH_CUST_CHARGE
#ifdef CONFIG_USB_MSM_OTG_SH_CUST_BATTERYLOG
static void msm_otg_set_battery_log(struct usb_otg *otg)
{
	struct msm_otg *motg = container_of(otg->phy, struct msm_otg, phy);
	int ret = 0;
	shbattlog_info_t info_t;

	/* charge type */
	if (motg->chg_type == USB_PROPRIETARY_CHARGER) {
		if (test_bit(B_SESS_VLD, &motg->inputs))
			info_t.event_num = SHBATTLOG_EVENT_CHG_INSERT_PROP_CHGR;
		else
			info_t.event_num = SHBATTLOG_EVENT_CHG_REMOVE_PROP_CHGR;
	} else {
		dev_dbg(motg->phy.dev, "%s: not put battery log chg_type = %d\n", __func__, (int)motg->chg_type);
		return;
	}

	/* put battery log */
	ret = shterm_k_set_event(&info_t);
	dev_dbg(motg->phy.dev, "%s: put battery log chg_type = %d, ret = %d\n", __func__, (int)motg->chg_type, ret);

	return;
}
#endif /* CONFIG_USB_MSM_OTG_SH_CUST_BATTERYLOG */

static int msm_otg_notify_chg_info(struct usb_phy *phy, unsigned mA)
{
	struct msm_otg *motg = container_of(phy, struct msm_otg, phy);
#ifdef CONFIG_USB_MSM_OTG_SH_CUST_BATTERYLOG
	struct usb_otg *otg = motg->phy.otg;
#endif /* CONFIG_USB_MSM_OTG_SH_CUST_BATTERYLOG */

	shchg_device_t type = SHCHG_DEVICE_USB_HOST;

	/* set charge type */
	if (motg->chg_type == USB_SDP_CHARGER)
		type = SHCHG_DEVICE_USB_HOST;
	else if (motg->chg_type == USB_DCP_CHARGER)
		type = SHCHG_DEVICE_USB_CHARGER;
	else if (motg->chg_type == USB_PROPRIETARY_CHARGER)
		type = SHCHG_DEVICE_USB_HOST;
	else {
		pr_err("%s: invalid charge type  chg_type = %d\n", __func__, (int)motg->chg_type);
		return 0;
	}


	/* check vbus */
	if (test_bit(B_SESS_VLD, &motg->inputs))
		shchg_api_notify_usb_charger_connected(type);
	else
		shchg_api_notify_usb_charger_disconnected();

	/* is charge */
	if (mA > 0)
		shchg_api_notify_usb_charger_i_is_available(mA);
	else
		shchg_api_notify_usb_charger_i_is_not_available();

#ifdef CONFIG_USB_MSM_OTG_SH_CUST_BATTERYLOG
	msm_otg_set_battery_log(otg);
#endif /* CONFIG_USB_MSM_OTG_SH_CUST_BATTERYLOG */

	dev_info(motg->phy.dev, "msm_otg_notify_chg_info chg_type=%d mA=%d vbus=%d \n"
				, (int)motg->chg_type, mA, test_bit(B_SESS_VLD, &motg->inputs));

	return 0;
}

void msm_notify_chg_info_from_mhl(unsigned mA)
{
	struct msm_otg *motg = the_msm_otg;

	motg->chg_type = USB_MHL_CHARGER;
	dev_info(motg->phy.dev, "msm_notify_chg_info_from_mhl chg_type=%d mA=%d vbus=%d \n"
				, (int)motg->chg_type, mA, test_bit(B_SESS_VLD, &motg->inputs));


	/* 0:disconnect other:connect */
	if (mA > 0) {
		shchg_api_notify_usb_charger_connected(SHCHG_DEVICE_MHL);
		shchg_api_notify_usb_charger_i_is_available(mA);
	}
	else {
		shchg_api_notify_usb_charger_disconnected();
		shchg_api_notify_usb_charger_i_is_not_available();
	}
}

void msm_notify_chg_info_from_mhl_irregular(unsigned mA)
{
	struct msm_otg *motg = the_msm_otg;
#ifdef CONFIG_USB_MSM_OTG_SH_CUST_BATTERYLOG
	struct usb_otg *otg = motg->phy.otg;
#endif /* CONFIG_USB_MSM_OTG_SH_CUST_BATTERYLOG */

	motg->chg_type = USB_PROPRIETARY_CHARGER;
	dev_info(motg->phy.dev, "msm_notify_chg_info_from_mhl_irregular chg_type=%d mA=%d vbus=%d \n"
				, (int)motg->chg_type, mA, test_bit(B_SESS_VLD, &motg->inputs));

	/* 0:disconnect other:connect */
	if (mA > 0) {
		shchg_api_notify_usb_charger_connected(SHCHG_DEVICE_USB_HOST);
		shchg_api_notify_usb_charger_i_is_available(mA);
	}
	else {
		shchg_api_notify_usb_charger_disconnected();
		shchg_api_notify_usb_charger_i_is_not_available();
	}

#ifdef CONFIG_USB_MSM_OTG_SH_CUST_BATTERYLOG
	msm_otg_set_battery_log(otg);
#endif /* CONFIG_USB_MSM_OTG_SH_CUST_BATTERYLOG */
}
#else /* CONFIG_USB_MSM_OTG_SH_CUST_CHARGE */
static int msm_otg_notify_chg_type(struct msm_otg *motg)
{
	static int charger_type;
	/*
	 * TODO
	 * Unify OTG driver charger types and power supply charger types
	 */
	if (charger_type == motg->chg_type)
		return 0;

	if (motg->chg_type == USB_SDP_CHARGER)
		charger_type = POWER_SUPPLY_TYPE_USB;
	else if (motg->chg_type == USB_CDP_CHARGER)
		charger_type = POWER_SUPPLY_TYPE_USB_CDP;
	else if (motg->chg_type == USB_DCP_CHARGER ||
			motg->chg_type == USB_PROPRIETARY_CHARGER)
		charger_type = POWER_SUPPLY_TYPE_USB_DCP;
	else if ((motg->chg_type == USB_ACA_DOCK_CHARGER ||
		motg->chg_type == USB_ACA_A_CHARGER ||
		motg->chg_type == USB_ACA_B_CHARGER ||
		motg->chg_type == USB_ACA_C_CHARGER))
		charger_type = POWER_SUPPLY_TYPE_USB_ACA;
	else
		charger_type = POWER_SUPPLY_TYPE_BATTERY;

	return pm8921_set_usb_power_supply_type(charger_type);
}

static int msm_otg_notify_power_supply(struct msm_otg *motg, unsigned mA)
{

	if (!psy)
		goto psy_not_supported;

	if (motg->cur_power == 0 && mA > 0) {
		/* Enable charging */
		if (power_supply_set_online(psy, true))
			goto psy_not_supported;
	} else if (motg->cur_power > 0 && mA == 0) {
		/* Disable charging */
		if (power_supply_set_online(psy, false))
			goto psy_not_supported;
		return 0;
	}
	/* Set max current limit */
	if (power_supply_set_current_limit(psy, 1000*mA))
		goto psy_not_supported;

	return 0;

psy_not_supported:
	dev_dbg(motg->phy.dev, "Power Supply doesn't support USB charger\n");
	return -ENXIO;
}
#endif /* CONFIG_USB_MSM_OTG_SH_CUST_CHARGE */

static void msm_otg_notify_charger(struct msm_otg *motg, unsigned mA)
{
#ifndef CONFIG_USB_MSM_OTG_SH_CUST_CHARGE
	struct usb_gadget *g = motg->phy.otg->gadget;

	if (g && g->is_a_peripheral)
		return;

	if ((motg->chg_type == USB_ACA_DOCK_CHARGER ||
		motg->chg_type == USB_ACA_A_CHARGER ||
		motg->chg_type == USB_ACA_B_CHARGER ||
		motg->chg_type == USB_ACA_C_CHARGER) &&
			mA > IDEV_ACA_CHG_LIMIT)
		mA = IDEV_ACA_CHG_LIMIT;

	if (msm_otg_notify_chg_type(motg))
		dev_err(motg->phy.dev,
			"Failed notifying %d charger type to PMIC\n",
							motg->chg_type);

	if (motg->cur_power == mA)
		return;

	dev_info(motg->phy.dev, "Avail curr from USB = %u\n", mA);

	/*
	 *  Use Power Supply API if supported, otherwise fallback
	 *  to legacy pm8921 API.
	 */
	if (msm_otg_notify_power_supply(motg, mA))
		pm8921_charger_vbus_draw(mA);

	motg->cur_power = mA;
#endif /* CONFIG_USB_MSM_OTG_SH_CUST_CHARGE */
}

static int msm_otg_set_power(struct usb_phy *phy, unsigned mA)
{
	struct msm_otg *motg = container_of(phy, struct msm_otg, phy);

	/*
	 * Gadget driver uses set_power method to notify about the
	 * available current based on suspend/configured states.
	 *
	 * IDEV_CHG can be drawn irrespective of suspend/un-configured
	 * states when CDP/ACA is connected.
	 */
#ifdef CONFIG_USB_MSM_OTG_SH_CUST
	if (motg->chg_type == USB_SDP_CHARGER){

		if(sh_disconnect_wait_flg != 0){
			shvbus_charge_wait_val = mA;
			return 0;
		}
		msm_otg_notify_charger(motg, mA);
	}
#else /* CONFIG_USB_MSM_OTG_SH_CUST */
	if (motg->chg_type == USB_SDP_CHARGER)
		msm_otg_notify_charger(motg, mA);
#endif /* CONFIG_USB_MSM_OTG_SH_CUST */
	return 0;
}



static void msm_otg_start_host(struct usb_otg *otg, int on)
{
	struct msm_otg *motg = container_of(otg->phy, struct msm_otg, phy);
	struct msm_otg_platform_data *pdata = motg->pdata;
	struct usb_hcd *hcd;

	if (!otg->host)
		return;

	hcd = bus_to_hcd(otg->host);

	if (on) {
		dev_dbg(otg->phy->dev, "host on\n");

		if (pdata->otg_control == OTG_PHY_CONTROL)
			ulpi_write(otg->phy, OTG_COMP_DISABLE,
				ULPI_SET(ULPI_PWR_CLK_MNG_REG));

		/*
		 * Some boards have a switch cotrolled by gpio
		 * to enable/disable internal HUB. Enable internal
		 * HUB before kicking the host.
		 */
		if (pdata->setup_gpio)
			pdata->setup_gpio(OTG_STATE_A_HOST);
		usb_add_hcd(hcd, hcd->irq, IRQF_SHARED);
	} else {
		dev_dbg(otg->phy->dev, "host off\n");

		usb_remove_hcd(hcd);
		/* HCD core reset all bits of PORTSC. select ULPI phy */
		writel_relaxed(0x80000000, USB_PORTSC);

		if (pdata->setup_gpio)
			pdata->setup_gpio(OTG_STATE_UNDEFINED);

		if (pdata->otg_control == OTG_PHY_CONTROL)
			ulpi_write(otg->phy, OTG_COMP_DISABLE,
				ULPI_CLR(ULPI_PWR_CLK_MNG_REG));
	}
}

static int msm_otg_usbdev_notify(struct notifier_block *self,
			unsigned long action, void *priv)
{
	struct msm_otg *motg = container_of(self, struct msm_otg, usbdev_nb);
	struct usb_otg *otg = motg->phy.otg;
	struct usb_device *udev = priv;

	if (action == USB_BUS_ADD || action == USB_BUS_REMOVE)
		goto out;

	if (udev->bus != otg->host)
		goto out;
	/*
	 * Interested in devices connected directly to the root hub.
	 * ACA dock can supply IDEV_CHG irrespective devices connected
	 * on the accessory port.
	 */
	if (!udev->parent || udev->parent->parent ||
			motg->chg_type == USB_ACA_DOCK_CHARGER)
		goto out;

	switch (action) {
	case USB_DEVICE_ADD:
		if (aca_enabled())
			usb_disable_autosuspend(udev);
		if (otg->phy->state == OTG_STATE_A_WAIT_BCON) {
			pr_debug("B_CONN set\n");
			set_bit(B_CONN, &motg->inputs);
			msm_otg_del_timer(motg);
			otg->phy->state = OTG_STATE_A_HOST;
			/*
			 * OTG PET: A-device must end session within
			 * 10 sec after PET enumeration.
			 */
			if (udev->quirks & USB_QUIRK_OTG_PET)
				msm_otg_start_timer(motg, TA_TST_MAINT,
						A_TST_MAINT);
		}
		/* fall through */
	case USB_DEVICE_CONFIG:
		if (udev->actconfig)
			motg->mA_port = udev->actconfig->desc.bMaxPower * 2;
		else
			motg->mA_port = IUNIT;
		if (otg->phy->state == OTG_STATE_B_HOST)
			msm_otg_del_timer(motg);
		break;
	case USB_DEVICE_REMOVE:
		if ((otg->phy->state == OTG_STATE_A_HOST) ||
			(otg->phy->state == OTG_STATE_A_SUSPEND)) {
			pr_debug("B_CONN clear\n");
			clear_bit(B_CONN, &motg->inputs);
			/*
			 * OTG PET: A-device must end session after
			 * PET disconnection if it is enumerated
			 * with bcdDevice[0] = 1. USB core sets
			 * bus->otg_vbus_off for us. clear it here.
			 */
			if (udev->bus->otg_vbus_off) {
				udev->bus->otg_vbus_off = 0;
				set_bit(A_BUS_DROP, &motg->inputs);
			}
			queue_work(system_nrt_wq, &motg->sm_work);
		}
	default:
		break;
	}
	if (test_bit(ID_A, &motg->inputs))
		msm_otg_notify_charger(motg, IDEV_ACA_CHG_MAX -
				motg->mA_port);
out:
	return NOTIFY_OK;
}

static void msm_hsusb_vbus_power(struct msm_otg *motg, bool on)
{
	int ret;
	static bool vbus_is_on;

	if (vbus_is_on == on)
		return;

	if (motg->pdata->vbus_power) {
		ret = motg->pdata->vbus_power(on);
		if (!ret)
			vbus_is_on = on;
		return;
	}

	if (!vbus_otg) {
		pr_err("vbus_otg is NULL.");
		return;
	}

	/*
	 * if entering host mode tell the charger to not draw any current
	 * from usb before turning on the boost.
	 * if exiting host mode disable the boost before enabling to draw
	 * current from the source.
	 */
	if (on) {
#ifdef CONFIG_USB_MSM_OTG_EXT5V_EXCLUSIVE
		ret = tps61029_register_user(TPS61029_USER_USB);
		if (ret != TPS61029_SUCCESS) {
			pr_err("unable to enable ext_5v\n");
			return;
		}
#endif /* CONFIG_USB_MSM_OTG_EXT5V_EXCLUSIVE */
		msm_otg_notify_host_mode(motg, on);
		ret = regulator_enable(vbus_otg);
		if (ret) {
			pr_err("unable to enable vbus_otg\n");
			return;
		}
		vbus_is_on = true;
	} else {
		ret = regulator_disable(vbus_otg);
		if (ret) {
			pr_err("unable to disable vbus_otg\n");
			return;
		}
#ifdef CONFIG_USB_MSM_OTG_EXT5V_EXCLUSIVE
		ret = tps61029_unregister_user(TPS61029_USER_USB);
		if (ret != TPS61029_SUCCESS) {
			pr_err("unable to disable ext_5v\n");
			return;
		}
#endif /* CONFIG_USB_MSM_OTG_EXT5V_EXCLUSIVE */
		msm_otg_notify_host_mode(motg, on);
		vbus_is_on = false;
	}
}

static int msm_otg_set_host(struct usb_otg *otg, struct usb_bus *host)
{
	struct msm_otg *motg = container_of(otg->phy, struct msm_otg, phy);
	struct usb_hcd *hcd;

	/*
	 * Fail host registration if this board can support
	 * only peripheral configuration.
	 */
	if (motg->pdata->mode == USB_PERIPHERAL) {
		dev_info(otg->phy->dev, "Host mode is not supported\n");
		return -ENODEV;
	}

	if (!motg->pdata->vbus_power && host) {
		vbus_otg = devm_regulator_get(motg->phy.dev, "vbus_otg");
		if (IS_ERR(vbus_otg)) {
			pr_err("Unable to get vbus_otg\n");
			return -ENODEV;
		}
	}

	if (!host) {
#ifdef CONFIG_USB_MSM_OTG_SH_CUST
		msm_sh_wait_cancel();
#endif /* CONFIG_USB_MSM_OTG_SH_CUST */
		if (otg->phy->state == OTG_STATE_A_HOST) {
			pm_runtime_get_sync(otg->phy->dev);
			usb_unregister_notify(&motg->usbdev_nb);
			msm_otg_start_host(otg, 0);
			msm_hsusb_vbus_power(motg, 0);
			otg->host = NULL;
			otg->phy->state = OTG_STATE_UNDEFINED;
			queue_work(system_nrt_wq, &motg->sm_work);
		} else {
			otg->host = NULL;
		}

		return 0;
	}

	hcd = bus_to_hcd(host);
	hcd->power_budget = motg->pdata->power_budget;

#ifdef CONFIG_USB_OTG
	host->otg_port = 1;
#endif
	motg->usbdev_nb.notifier_call = msm_otg_usbdev_notify;
	usb_register_notify(&motg->usbdev_nb);
	otg->host = host;
	dev_dbg(otg->phy->dev, "host driver registered w/ tranceiver\n");

	/*
	 * Kick the state machine work, if peripheral is not supported
	 * or peripheral is already registered with us.
	 */
	if (motg->pdata->mode == USB_HOST || otg->gadget) {
		pm_runtime_get_sync(otg->phy->dev);
		queue_work(system_nrt_wq, &motg->sm_work);
	}

	return 0;
}

#ifdef CONFIG_USB_SH_CUST_NON_STANDARD_CHARGE
static int msm_otg_start_peripheral(struct usb_otg *otg, int on)
#else /* CONFIG_USB_SH_CUST_NON_STANDARD_CHARGE */
static void msm_otg_start_peripheral(struct usb_otg *otg, int on)
#endif /* CONFIG_USB_SH_CUST_NON_STANDARD_CHARGE */
{
	int ret;
	struct msm_otg *motg = container_of(otg->phy, struct msm_otg, phy);
	struct msm_otg_platform_data *pdata = motg->pdata;
#ifdef CONFIG_USB_SH_CUST_NON_STANDARD_CHARGE
	int result = 0;
#endif /* CONFIG_USB_SH_CUST_NON_STANDARD_CHARGE */

	if (!otg->gadget)
#ifdef CONFIG_USB_SH_CUST_NON_STANDARD_CHARGE
		return -ENODEV;
#else /* CONFIG_USB_SH_CUST_NON_STANDARD_CHARGE */
		return;
#endif /* CONFIG_USB_SH_CUST_NON_STANDARD_CHARGE */

	if (on) {
		dev_dbg(otg->phy->dev, "gadget on\n");
		/*
		 * Some boards have a switch cotrolled by gpio
		 * to enable/disable internal HUB. Disable internal
		 * HUB before kicking the gadget.
		 */
		if (pdata->setup_gpio)
			pdata->setup_gpio(OTG_STATE_B_PERIPHERAL);

		/* Configure BUS performance parameters for MAX bandwidth */
		if (motg->bus_perf_client && debug_bus_voting_enabled) {
			ret = msm_bus_scale_client_update_request(
					motg->bus_perf_client, 1);
			if (ret)
				dev_err(motg->phy.dev, "%s: Failed to vote for "
					   "bus bandwidth %d\n", __func__, ret);
		}

#ifdef CONFIG_USB_SH_CUST_NON_STANDARD_CHARGE
		result = usb_gadget_vbus_connect(otg->gadget);
#else /* CONFIG_USB_SH_CUST_NON_STANDARD_CHARGE */
		usb_gadget_vbus_connect(otg->gadget);
#endif /* CONFIG_USB_SH_CUST_NON_STANDARD_CHARGE */
	} else {
		dev_dbg(otg->phy->dev, "gadget off\n");
#ifdef CONFIG_USB_SH_CUST_NON_STANDARD_CHARGE
		result = usb_gadget_vbus_disconnect(otg->gadget);
#else /* CONFIG_USB_SH_CUST_NON_STANDARD_CHARGE */
		usb_gadget_vbus_disconnect(otg->gadget);
#endif /* CONFIG_USB_SH_CUST_NON_STANDARD_CHARGE */
		/* Configure BUS performance parameters to default */
		if (motg->bus_perf_client) {
			ret = msm_bus_scale_client_update_request(
					motg->bus_perf_client, 0);
			if (ret)
				dev_err(motg->phy.dev, "%s: Failed to devote "
					   "for bus bw %d\n", __func__, ret);
		}
		if (pdata->setup_gpio)
			pdata->setup_gpio(OTG_STATE_UNDEFINED);
	}

#ifdef CONFIG_USB_SH_CUST_NON_STANDARD_CHARGE
	if(0 != result)
		pr_err("%s: start peripheral error = %d\n", __func__, result);

	return result;
#endif /* CONFIG_USB_SH_CUST_NON_STANDARD_CHARGE */
}

static int msm_otg_set_peripheral(struct usb_otg *otg,
			struct usb_gadget *gadget)
{
	struct msm_otg *motg = container_of(otg->phy, struct msm_otg, phy);

	/*
	 * Fail peripheral registration if this board can support
	 * only host configuration.
	 */
	if (motg->pdata->mode == USB_HOST) {
		dev_info(otg->phy->dev, "Peripheral mode is not supported\n");
		return -ENODEV;
	}

	if (!gadget) {
#ifdef CONFIG_USB_MSM_OTG_SH_CUST
		msm_sh_wait_cancel();
#endif /* CONFIG_USB_MSM_OTG_SH_CUST */
		if (otg->phy->state == OTG_STATE_B_PERIPHERAL) {
			pm_runtime_get_sync(otg->phy->dev);
			msm_otg_start_peripheral(otg, 0);
			otg->gadget = NULL;
			otg->phy->state = OTG_STATE_UNDEFINED;
			queue_work(system_nrt_wq, &motg->sm_work);
		} else {
			otg->gadget = NULL;
		}

		return 0;
	}
	otg->gadget = gadget;
	dev_dbg(otg->phy->dev, "peripheral driver registered w/ tranceiver\n");

	/*
	 * Kick the state machine work, if host is not supported
	 * or host is already registered with us.
	 */
	if (motg->pdata->mode == USB_PERIPHERAL || otg->host) {
		pm_runtime_get_sync(otg->phy->dev);
		queue_work(system_nrt_wq, &motg->sm_work);
	}

	return 0;
}

#ifdef CONFIG_USB_MSM_OTG_SH_CUST_EYEPATTERN
static void msm_otg_set_override(struct otg_ulpi_modify *ulpi_tbl)
{
	u32 reg_val=0;
	u32 set_val=0;
	struct msm_otg *motg = the_msm_otg;
	struct usb_phy *phy = &motg->phy;

	if( !ulpi_tbl )
		return ;

	while( ulpi_tbl->reg != OTG_ULPI_MODIFY_MAX ){

		reg_val = ulpi_read(phy , ulpi_tbl->reg );

		set_val = (reg_val & ~ulpi_tbl->mask ) | ulpi_tbl->val ;
		ulpi_write(phy, set_val, (u32)ulpi_tbl->reg);

		pr_debug("set override reg=0x%02x val=0x%02x " , ulpi_tbl->reg , set_val);
		ulpi_tbl++;

	}

}
#endif /* CONFIG_USB_MSM_OTG_SH_CUST_EYEPATTERN */

static bool msm_chg_aca_detect(struct msm_otg *motg)
{
	struct usb_phy *phy = &motg->phy;
	u32 int_sts;
	bool ret = false;

	if (!aca_enabled())
		goto out;

	if (motg->pdata->phy_type == CI_45NM_INTEGRATED_PHY)
		goto out;

	int_sts = ulpi_read(phy, 0x87);
	switch (int_sts & 0x1C) {
	case 0x08:
		if (!test_and_set_bit(ID_A, &motg->inputs)) {
			dev_dbg(phy->dev, "ID_A\n");
			motg->chg_type = USB_ACA_A_CHARGER;
			motg->chg_state = USB_CHG_STATE_DETECTED;
			clear_bit(ID_B, &motg->inputs);
			clear_bit(ID_C, &motg->inputs);
			set_bit(ID, &motg->inputs);
			ret = true;
		}
		break;
	case 0x0C:
		if (!test_and_set_bit(ID_B, &motg->inputs)) {
			dev_dbg(phy->dev, "ID_B\n");
			motg->chg_type = USB_ACA_B_CHARGER;
			motg->chg_state = USB_CHG_STATE_DETECTED;
			clear_bit(ID_A, &motg->inputs);
			clear_bit(ID_C, &motg->inputs);
			set_bit(ID, &motg->inputs);
			ret = true;
		}
		break;
	case 0x10:
		if (!test_and_set_bit(ID_C, &motg->inputs)) {
			dev_dbg(phy->dev, "ID_C\n");
			motg->chg_type = USB_ACA_C_CHARGER;
			motg->chg_state = USB_CHG_STATE_DETECTED;
			clear_bit(ID_A, &motg->inputs);
			clear_bit(ID_B, &motg->inputs);
			set_bit(ID, &motg->inputs);
			ret = true;
		}
		break;
	case 0x04:
		if (test_and_clear_bit(ID, &motg->inputs)) {
			dev_dbg(phy->dev, "ID_GND\n");
			motg->chg_type = USB_INVALID_CHARGER;
			motg->chg_state = USB_CHG_STATE_UNDEFINED;
			clear_bit(ID_A, &motg->inputs);
			clear_bit(ID_B, &motg->inputs);
			clear_bit(ID_C, &motg->inputs);
			ret = true;
		}
		break;
	default:
		ret = test_and_clear_bit(ID_A, &motg->inputs) |
			test_and_clear_bit(ID_B, &motg->inputs) |
			test_and_clear_bit(ID_C, &motg->inputs) |
			!test_and_set_bit(ID, &motg->inputs);
		if (ret) {
			dev_dbg(phy->dev, "ID A/B/C/GND is no more\n");
			motg->chg_type = USB_INVALID_CHARGER;
			motg->chg_state = USB_CHG_STATE_UNDEFINED;
		}
	}
out:
	return ret;
}

static void msm_chg_enable_aca_det(struct msm_otg *motg)
{
	struct usb_phy *phy = &motg->phy;

	if (!aca_enabled())
		return;

	switch (motg->pdata->phy_type) {
	case SNPS_28NM_INTEGRATED_PHY:
		/* Disable ID_GND in link and PHY */
		writel_relaxed(readl_relaxed(USB_OTGSC) & ~(OTGSC_IDPU |
				OTGSC_IDIE), USB_OTGSC);
		ulpi_write(phy, 0x01, 0x0C);
		ulpi_write(phy, 0x10, 0x0F);
		ulpi_write(phy, 0x10, 0x12);
#ifndef CONFIG_USB_MSM_OTG_SH_CUST_SWIC
		/* Disable PMIC ID pull-up */
		pm8xxx_usb_id_pullup(0);
#endif /* CONFIG_USB_MSM_OTG_SH_CUST_SWIC */
		/* Enable ACA ID detection */
		ulpi_write(phy, 0x20, 0x85);
		aca_id_turned_on = true;
		break;
	default:
		break;
	}
}

static void msm_chg_enable_aca_intr(struct msm_otg *motg)
{
	struct usb_phy *phy = &motg->phy;

	if (!aca_enabled())
		return;

	switch (motg->pdata->phy_type) {
	case SNPS_28NM_INTEGRATED_PHY:
		/* Enable ACA Detection interrupt (on any RID change) */
		ulpi_write(phy, 0x01, 0x94);
		break;
	default:
		break;
	}
}

static void msm_chg_disable_aca_intr(struct msm_otg *motg)
{
	struct usb_phy *phy = &motg->phy;

	if (!aca_enabled())
		return;

	switch (motg->pdata->phy_type) {
	case SNPS_28NM_INTEGRATED_PHY:
		ulpi_write(phy, 0x01, 0x95);
		break;
	default:
		break;
	}
}

static bool msm_chg_check_aca_intr(struct msm_otg *motg)
{
	struct usb_phy *phy = &motg->phy;
	bool ret = false;

	if (!aca_enabled())
		return ret;

	switch (motg->pdata->phy_type) {
	case SNPS_28NM_INTEGRATED_PHY:
		if (ulpi_read(phy, 0x91) & 1) {
			dev_dbg(phy->dev, "RID change\n");
			ulpi_write(phy, 0x01, 0x92);
			ret = msm_chg_aca_detect(motg);
		}
	default:
		break;
	}
	return ret;
}

static void msm_otg_id_timer_func(unsigned long data)
{
	struct msm_otg *motg = (struct msm_otg *) data;

	if (!aca_enabled())
		return;

	if (atomic_read(&motg->in_lpm)) {
		dev_dbg(motg->phy.dev, "timer: in lpm\n");
		return;
	}

	if (motg->phy.state == OTG_STATE_A_SUSPEND)
		goto out;

	if (msm_chg_check_aca_intr(motg)) {
		dev_dbg(motg->phy.dev, "timer: aca work\n");
		queue_work(system_nrt_wq, &motg->sm_work);
	}

out:
	if (!test_bit(ID, &motg->inputs) || test_bit(ID_A, &motg->inputs))
		mod_timer(&motg->id_timer, ID_TIMER_FREQ);
}

static bool msm_chg_check_secondary_det(struct msm_otg *motg)
{
	struct usb_phy *phy = &motg->phy;
	u32 chg_det;
	bool ret = false;

	switch (motg->pdata->phy_type) {
	case CI_45NM_INTEGRATED_PHY:
		chg_det = ulpi_read(phy, 0x34);
		ret = chg_det & (1 << 4);
		break;
	case SNPS_28NM_INTEGRATED_PHY:
		chg_det = ulpi_read(phy, 0x87);
		ret = chg_det & 1;
		break;
	default:
		break;
	}
	return ret;
}

static void msm_chg_enable_secondary_det(struct msm_otg *motg)
{
	struct usb_phy *phy = &motg->phy;
	u32 chg_det;

	switch (motg->pdata->phy_type) {
	case CI_45NM_INTEGRATED_PHY:
		chg_det = ulpi_read(phy, 0x34);
		/* Turn off charger block */
		chg_det |= ~(1 << 1);
		ulpi_write(phy, chg_det, 0x34);
		udelay(20);
		/* control chg block via ULPI */
		chg_det &= ~(1 << 3);
		ulpi_write(phy, chg_det, 0x34);
		/* put it in host mode for enabling D- source */
		chg_det &= ~(1 << 2);
		ulpi_write(phy, chg_det, 0x34);
		/* Turn on chg detect block */
		chg_det &= ~(1 << 1);
		ulpi_write(phy, chg_det, 0x34);
		udelay(20);
		/* enable chg detection */
		chg_det &= ~(1 << 0);
		ulpi_write(phy, chg_det, 0x34);
		break;
	case SNPS_28NM_INTEGRATED_PHY:
		/*
		 * Configure DM as current source, DP as current sink
		 * and enable battery charging comparators.
		 */
		ulpi_write(phy, 0x8, 0x85);
		ulpi_write(phy, 0x2, 0x85);
		ulpi_write(phy, 0x1, 0x85);
		break;
	default:
		break;
	}
}

static bool msm_chg_check_primary_det(struct msm_otg *motg)
{
	struct usb_phy *phy = &motg->phy;
	u32 chg_det;
	bool ret = false;

	switch (motg->pdata->phy_type) {
	case CI_45NM_INTEGRATED_PHY:
		chg_det = ulpi_read(phy, 0x34);
		ret = chg_det & (1 << 4);
		break;
	case SNPS_28NM_INTEGRATED_PHY:
		chg_det = ulpi_read(phy, 0x87);
		ret = chg_det & 1;
		/* Turn off VDP_SRC */
		ulpi_write(phy, 0x3, 0x86);
		msleep(20);
		break;
	default:
		break;
	}
	return ret;
}

static void msm_chg_enable_primary_det(struct msm_otg *motg)
{
	struct usb_phy *phy = &motg->phy;
	u32 chg_det;

	switch (motg->pdata->phy_type) {
	case CI_45NM_INTEGRATED_PHY:
		chg_det = ulpi_read(phy, 0x34);
		/* enable chg detection */
		chg_det &= ~(1 << 0);
		ulpi_write(phy, chg_det, 0x34);
		break;
	case SNPS_28NM_INTEGRATED_PHY:
		/*
		 * Configure DP as current source, DM as current sink
		 * and enable battery charging comparators.
		 */
		ulpi_write(phy, 0x2, 0x85);
		ulpi_write(phy, 0x1, 0x85);
		break;
	default:
		break;
	}
}

static bool msm_chg_check_dcd(struct msm_otg *motg)
{
	struct usb_phy *phy = &motg->phy;
	u32 line_state;
	bool ret = false;

	switch (motg->pdata->phy_type) {
	case CI_45NM_INTEGRATED_PHY:
		line_state = ulpi_read(phy, 0x15);
		ret = !(line_state & 1);
		break;
	case SNPS_28NM_INTEGRATED_PHY:
		line_state = ulpi_read(phy, 0x87);
		ret = line_state & 2;
		break;
	default:
		break;
	}
	return ret;
}

static void msm_chg_disable_dcd(struct msm_otg *motg)
{
	struct usb_phy *phy = &motg->phy;
	u32 chg_det;

	switch (motg->pdata->phy_type) {
	case CI_45NM_INTEGRATED_PHY:
		chg_det = ulpi_read(phy, 0x34);
		chg_det &= ~(1 << 5);
		ulpi_write(phy, chg_det, 0x34);
		break;
	case SNPS_28NM_INTEGRATED_PHY:
		ulpi_write(phy, 0x10, 0x86);
		break;
	default:
		break;
	}
}

static void msm_chg_enable_dcd(struct msm_otg *motg)
{
	struct usb_phy *phy = &motg->phy;
	u32 chg_det;

	switch (motg->pdata->phy_type) {
	case CI_45NM_INTEGRATED_PHY:
		chg_det = ulpi_read(phy, 0x34);
		/* Turn on D+ current source */
		chg_det |= (1 << 5);
		ulpi_write(phy, chg_det, 0x34);
		break;
	case SNPS_28NM_INTEGRATED_PHY:
		/* Data contact detection enable */
		ulpi_write(phy, 0x10, 0x85);
		break;
	default:
		break;
	}
}

static void msm_chg_block_on(struct msm_otg *motg)
{
	struct usb_phy *phy = &motg->phy;
	u32 func_ctrl, chg_det;

	/* put the controller in non-driving mode */
	func_ctrl = ulpi_read(phy, ULPI_FUNC_CTRL);
	func_ctrl &= ~ULPI_FUNC_CTRL_OPMODE_MASK;
	func_ctrl |= ULPI_FUNC_CTRL_OPMODE_NONDRIVING;
	ulpi_write(phy, func_ctrl, ULPI_FUNC_CTRL);

	switch (motg->pdata->phy_type) {
	case CI_45NM_INTEGRATED_PHY:
		chg_det = ulpi_read(phy, 0x34);
		/* control chg block via ULPI */
		chg_det &= ~(1 << 3);
		ulpi_write(phy, chg_det, 0x34);
		/* Turn on chg detect block */
		chg_det &= ~(1 << 1);
		ulpi_write(phy, chg_det, 0x34);
		udelay(20);
		break;
	case SNPS_28NM_INTEGRATED_PHY:
		/* disable DP and DM pull down resistors */
		ulpi_write(phy, 0x6, 0xC);
		/* Clear charger detecting control bits */
		ulpi_write(phy, 0x1F, 0x86);
		/* Clear alt interrupt latch and enable bits */
		ulpi_write(phy, 0x1F, 0x92);
		ulpi_write(phy, 0x1F, 0x95);
		udelay(100);
		break;
	default:
		break;
	}
}

static void msm_chg_block_off(struct msm_otg *motg)
{
	struct usb_phy *phy = &motg->phy;
	u32 func_ctrl, chg_det;

	switch (motg->pdata->phy_type) {
	case CI_45NM_INTEGRATED_PHY:
		chg_det = ulpi_read(phy, 0x34);
		/* Turn off charger block */
		chg_det |= ~(1 << 1);
		ulpi_write(phy, chg_det, 0x34);
		break;
	case SNPS_28NM_INTEGRATED_PHY:
		/* Clear charger detecting control bits */
		ulpi_write(phy, 0x3F, 0x86);
		/* Clear alt interrupt latch and enable bits */
		ulpi_write(phy, 0x1F, 0x92);
		ulpi_write(phy, 0x1F, 0x95);
		break;
	default:
		break;
	}

	/* put the controller in normal mode */
	func_ctrl = ulpi_read(phy, ULPI_FUNC_CTRL);
	func_ctrl &= ~ULPI_FUNC_CTRL_OPMODE_MASK;
	func_ctrl |= ULPI_FUNC_CTRL_OPMODE_NORMAL;
	ulpi_write(phy, func_ctrl, ULPI_FUNC_CTRL);
}

static const char *chg_to_string(enum usb_chg_type chg_type)
{
	switch (chg_type) {
	case USB_SDP_CHARGER:		return "USB_SDP_CHARGER";
	case USB_DCP_CHARGER:		return "USB_DCP_CHARGER";
	case USB_CDP_CHARGER:		return "USB_CDP_CHARGER";
	case USB_ACA_A_CHARGER:		return "USB_ACA_A_CHARGER";
	case USB_ACA_B_CHARGER:		return "USB_ACA_B_CHARGER";
	case USB_ACA_C_CHARGER:		return "USB_ACA_C_CHARGER";
	case USB_ACA_DOCK_CHARGER:	return "USB_ACA_DOCK_CHARGER";
	case USB_PROPRIETARY_CHARGER:	return "USB_PROPRIETARY_CHARGER";
	default:			return "INVALID_CHARGER";
	}
}

#ifdef CONFIG_USB_MSM_OTG_SH_CUST
#define MSM_CHG_BEFORE_WAIT_TIME	(350 * HZ/1000) /* before detection wait time */
#endif /* CONFIG_USB_MSM_OTG_SH_CUST */
#define MSM_CHG_DCD_POLL_TIME		(100 * HZ/1000) /* 100 msec */
#define MSM_CHG_DCD_MAX_RETRIES		6 /* Tdcd_tmout = 6 * 100 msec */
#define MSM_CHG_PRIMARY_DET_TIME	(50 * HZ/1000) /* TVDPSRC_ON */
#define MSM_CHG_SECONDARY_DET_TIME	(50 * HZ/1000) /* TVDMSRC_ON */
static void msm_chg_detect_work(struct work_struct *w)
{
	struct msm_otg *motg = container_of(w, struct msm_otg, chg_work.work);
	struct usb_phy *phy = &motg->phy;
	bool is_dcd = false, tmout, vout, is_aca;
	u32 line_state, dm_vlgc;
	unsigned long delay;

	dev_dbg(phy->dev, "chg detection work\n");
	switch (motg->chg_state) {
	case USB_CHG_STATE_UNDEFINED:
		msm_chg_block_on(motg);
		msm_chg_enable_dcd(motg);
		msm_chg_enable_aca_det(motg);
		motg->chg_state = USB_CHG_STATE_WAIT_FOR_DCD;
		motg->dcd_retries = 0;
#ifdef CONFIG_USB_MSM_OTG_SH_CUST
		delay = MSM_CHG_BEFORE_WAIT_TIME;
#else /* CONFIG_USB_MSM_OTG_SH_CUST */
		delay = MSM_CHG_DCD_POLL_TIME;
#endif /* CONFIG_USB_MSM_OTG_SH_CUST */
		break;
	case USB_CHG_STATE_WAIT_FOR_DCD:
		is_aca = msm_chg_aca_detect(motg);
		if (is_aca) {
			/*
			 * ID_A can be ACA dock too. continue
			 * primary detection after DCD.
			 */
			if (test_bit(ID_A, &motg->inputs)) {
				motg->chg_state = USB_CHG_STATE_WAIT_FOR_DCD;
			} else {
				delay = 0;
				break;
			}
		}
		is_dcd = msm_chg_check_dcd(motg);
		tmout = ++motg->dcd_retries == MSM_CHG_DCD_MAX_RETRIES;
		if (is_dcd || tmout) {
			msm_chg_disable_dcd(motg);
			msm_chg_enable_primary_det(motg);
			delay = MSM_CHG_PRIMARY_DET_TIME;
			motg->chg_state = USB_CHG_STATE_DCD_DONE;
		} else {
			delay = MSM_CHG_DCD_POLL_TIME;
		}
		break;
	case USB_CHG_STATE_DCD_DONE:
		vout = msm_chg_check_primary_det(motg);
		line_state = readl_relaxed(USB_PORTSC) & PORTSC_LS;
		dm_vlgc = line_state & PORTSC_LS_DM;
		if (vout && !dm_vlgc) { /* VDAT_REF < DM < VLGC */
			if (test_bit(ID_A, &motg->inputs)) {
				motg->chg_type = USB_ACA_DOCK_CHARGER;
				motg->chg_state = USB_CHG_STATE_DETECTED;
				delay = 0;
				break;
			}
			if (line_state) { /* DP > VLGC */
				motg->chg_type = USB_PROPRIETARY_CHARGER;
				motg->chg_state = USB_CHG_STATE_DETECTED;
				delay = 0;
			} else {
				msm_chg_enable_secondary_det(motg);
				delay = MSM_CHG_SECONDARY_DET_TIME;
				motg->chg_state = USB_CHG_STATE_PRIMARY_DONE;
			}
		} else { /* DM < VDAT_REF || DM > VLGC */
			if (test_bit(ID_A, &motg->inputs)) {
				motg->chg_type = USB_ACA_A_CHARGER;
				motg->chg_state = USB_CHG_STATE_DETECTED;
				delay = 0;
				break;
			}

			if (line_state) /* DP > VLGC or/and DM > VLGC */
				motg->chg_type = USB_PROPRIETARY_CHARGER;
			else
				motg->chg_type = USB_SDP_CHARGER;

			motg->chg_state = USB_CHG_STATE_DETECTED;
			delay = 0;
		}
		break;
	case USB_CHG_STATE_PRIMARY_DONE:
		vout = msm_chg_check_secondary_det(motg);
		if (vout)
			motg->chg_type = USB_DCP_CHARGER;
		else
			motg->chg_type = USB_CDP_CHARGER;
		motg->chg_state = USB_CHG_STATE_SECONDARY_DONE;
		/* fall through */
	case USB_CHG_STATE_SECONDARY_DONE:
		motg->chg_state = USB_CHG_STATE_DETECTED;
	case USB_CHG_STATE_DETECTED:
		msm_chg_block_off(motg);
		msm_chg_enable_aca_det(motg);
		/*
		 * Spurious interrupt is seen after enabling ACA detection
		 * due to which charger detection fails in case of PET.
		 * Add delay of 100 microsec to avoid that.
		 */
		udelay(100);
		msm_chg_enable_aca_intr(motg);
		dev_dbg(phy->dev, "chg_type = %s\n",
			chg_to_string(motg->chg_type));
		queue_work(system_nrt_wq, &motg->sm_work);
		return;
	default:
		return;
	}

	queue_delayed_work(system_nrt_wq, &motg->chg_work, delay);
}

/*
 * We support OTG, Peripheral only and Host only configurations. In case
 * of OTG, mode switch (host-->peripheral/peripheral-->host) can happen
 * via Id pin status or user request (debugfs). Id/BSV interrupts are not
 * enabled when switch is controlled by user and default mode is supplied
 * by board file, which can be changed by userspace later.
 */
static void msm_otg_init_sm(struct msm_otg *motg)
{
	struct msm_otg_platform_data *pdata = motg->pdata;
	u32 otgsc = readl(USB_OTGSC);

	switch (pdata->mode) {
	case USB_OTG:
		if (pdata->otg_control == OTG_USER_CONTROL) {
			if (pdata->default_mode == USB_HOST) {
				clear_bit(ID, &motg->inputs);
			} else if (pdata->default_mode == USB_PERIPHERAL) {
				set_bit(ID, &motg->inputs);
				set_bit(B_SESS_VLD, &motg->inputs);
			} else {
				set_bit(ID, &motg->inputs);
				clear_bit(B_SESS_VLD, &motg->inputs);
			}
		} else if (pdata->otg_control == OTG_PHY_CONTROL) {
			if (otgsc & OTGSC_ID) {
				set_bit(ID, &motg->inputs);
			} else {
				clear_bit(ID, &motg->inputs);
				set_bit(A_BUS_REQ, &motg->inputs);
			}
			if (otgsc & OTGSC_BSV)
				set_bit(B_SESS_VLD, &motg->inputs);
			else
				clear_bit(B_SESS_VLD, &motg->inputs);
		} else if (pdata->otg_control == OTG_PMIC_CONTROL) {
#ifndef CONFIG_USB_MSM_OTG_SH_CUST
			if (pdata->pmic_id_irq) {
				unsigned long flags;
				local_irq_save(flags);
				if (irq_read_line(pdata->pmic_id_irq))
					set_bit(ID, &motg->inputs);
				else
					clear_bit(ID, &motg->inputs);
				local_irq_restore(flags);
			}
#endif /* CONFIG_USB_MSM_OTG_SH_CUST */

#ifndef CONFIG_USB_MSM_OTG_SH_CUST_SWIC
			/*
			 * VBUS initial state is reported after PMIC
			 * driver initialization. Wait for it.
			 */
			wait_for_completion(&pmic_vbus_init);
#endif /* CONFIG_USB_MSM_OTG_SH_CUST_SWIC */
#ifdef CONFIG_USB_MSM_OTG_SH_CUST
			if (!is_id_line_low())
				set_bit(ID, &motg->inputs);
			else
				clear_bit(ID, &motg->inputs);
#endif /* CONFIG_USB_MSM_OTG_SH_CUST */
		}
		break;
	case USB_HOST:
		clear_bit(ID, &motg->inputs);
		break;
	case USB_PERIPHERAL:
		set_bit(ID, &motg->inputs);
		if (pdata->otg_control == OTG_PHY_CONTROL) {
			if (otgsc & OTGSC_BSV)
				set_bit(B_SESS_VLD, &motg->inputs);
			else
				clear_bit(B_SESS_VLD, &motg->inputs);
		} else if (pdata->otg_control == OTG_PMIC_CONTROL) {
			/*
			 * VBUS initial state is reported after PMIC
			 * driver initialization. Wait for it.
			 */
			wait_for_completion(&pmic_vbus_init);
		}
		break;
	default:
		break;
	}
}

#ifdef CONFIG_USB_SWIC
static void msm_otg_swic_ch_inputs(struct msm_otg *motg)
{
	u8 device  = 0;

	device = sh_swic_device;

	switch(device) {
	case SHSWIC_ID_USB_CABLE:
		set_bit(ID, &motg->inputs);
		set_bit(B_SESS_VLD, &motg->inputs);
		break;
	case SHSWIC_ID_AC_ADAPTER:
		set_bit(ID, &motg->inputs);
		set_bit(B_SESS_VLD, &motg->inputs);
		break;
	case SHSWIC_ID_USB_HOST_CABLE:
		clear_bit(ID, &motg->inputs);
		clear_bit(B_SESS_VLD, &motg->inputs);
		break;
	case SHSWIC_ID_MHL:
		clear_bit(ID, &motg->inputs);
		set_bit(B_SESS_VLD, &motg->inputs);
		break;
	case SHSWIC_ID_IRREGULAR_CHARGER:
		set_bit(ID, &motg->inputs);
		set_bit(B_SESS_VLD, &motg->inputs);
		break;
	case SHSWIC_ID_NONE:
		set_bit(ID, &motg->inputs);
		clear_bit(B_SESS_VLD, &motg->inputs);
		break;
	default:
		pr_err("%s: swic err state = %d \n" 
			, __func__ , device);
		break;
	}
}

#ifdef CONFIG_USB_MSM_OTG_SH_CUST_SWIC
static void msm_otg_swic_ch_status(struct msm_otg *motg)
{
	u8 device  = 0;
	device = sh_swic_device;

	switch(device) {
	case SHSWIC_ID_USB_CABLE:
		motg->chg_state = USB_CHG_STATE_DETECTED;
		motg->chg_type  = USB_SDP_CHARGER;
		break;
	case SHSWIC_ID_AC_ADAPTER:
		motg->chg_state = USB_CHG_STATE_DETECTED;
		motg->chg_type  = USB_DCP_CHARGER;
		break;
	case SHSWIC_ID_USB_HOST_CABLE:
	case SHSWIC_ID_MHL:
		break;
	case SHSWIC_ID_IRREGULAR_CHARGER:
		motg->chg_state = USB_CHG_STATE_DETECTED;
		motg->chg_type  = USB_PROPRIETARY_CHARGER;
		break;
	case SHSWIC_ID_NONE:
		motg->chg_state = USB_CHG_STATE_UNDEFINED;
		motg->chg_type  = USB_INVALID_CHARGER;
		/* THROUGH */
		break;
	default:
		pr_err("%s: swic err state = %d \n" 
			, __func__ , device);
		break;
	}

	return ;
}
#endif /* CONFIG_USB_MSM_OTG_SH_CUST_SWIC */
#endif /* CONFIG_USB_SWIC */

#ifdef CONFIG_USB_MSM_OTG_SH_CUST
static int get_cable_type(void)
{
	struct msm_otg *dev = the_msm_otg;
	int cable_type;

	if (!test_bit(ID, &dev->inputs)) {
		/* host */
		cable_type = 0;
	} else if (test_bit(B_SESS_VLD, &dev->inputs)) {
		/* peripehral */
		cable_type = 1;
	} else {
		/* not connect */
		cable_type = 2;
	}

	return cable_type;
}

static void msm_otg_change_cable_notify(void)
{
	struct msm_otg *dev = the_msm_otg;
	char *uevent_envp[]={"CABLE_CHANGE",NULL};
	int cable_type;

	cable_type = get_cable_type();
	if (sh_cable_type != cable_type) {
		kobject_uevent_env(&dev->phy.dev->kobj,KOBJ_CHANGE, uevent_envp);
#ifdef CONFIG_USB_DEBUG_SH_LOG
		pr_info("%s: sent uevent %s type %d->%d\n", __func__, uevent_envp[0], sh_cable_type, cable_type);
#endif /* CONFIG_USB_DEBUG_SH_LOG */
	}
	sh_cable_type = cable_type;
}
#endif /* CONFIG_USB_MSM_OTG_SH_CUST */

static void msm_otg_sm_work(struct work_struct *w)
{
	struct msm_otg *motg = container_of(w, struct msm_otg, sm_work);
	struct usb_otg *otg = motg->phy.otg;
	bool work = 0, srp_reqd;
#ifdef CONFIG_USB_SH_CUST_NON_STANDARD_CHARGE
	int ret = 0;
#endif /* CONFIG_USB_SH_CUST_NON_STANDARD_CHARGE */
#ifdef CONFIG_USB_MSM_OTG_SH_CUST
	int chg_notify = 0;

	pr_info("msm_otg_sm_work inputs=%d\n" , (int)motg->inputs);

	if(!msm_otg_sm_work_lockinc()){
		pr_debug("return sm_work");
		return ;
	}
#endif /* CONFIG_USB_MSM_OTG_SH_CUST */

#ifdef CONFIG_USB_MSM_OTG_SH_CUST_HW_CHECK
	if(!swic_hw_init)
		msm_otg_hw_check();
#endif /* CONFIG_USB_MSM_OTG_SH_CUST_HW_CHECK */

#ifdef CONFIG_USB_MSM_OTG_SH_CUST_MHL
	if( !msm_otg_mhl_sm_work(w)){
		if(!msm_otg_sm_work_lockdec())
			queue_work(system_nrt_wq, &motg->sm_work);
		return ;
	}
#else /* CONFIG_USB_MSM_OTG_SH_CUST_MHL */
#ifdef CONFIG_USB_MSM_OTG_SH_CUST
	if(shusb_id_interrupts){
		if (!is_id_line_low())
			set_bit(ID, &motg->inputs);
		else
			clear_bit(ID, &motg->inputs);
		pr_debug("msm_otg_sm_work shusb_id_interrupts %d \n" ,(int)motg->inputs);
		shusb_id_interrupts = false;
	}
#endif /* CONFIG_USB_MSM_OTG_SH_CUST */
#endif /* CONFIG_USB_MSM_OTG_SH_CUST_MHL */

	pm_runtime_resume(otg->phy->dev);
	pr_debug("%s work\n", otg_state_string(otg->phy->state));
	switch (otg->phy->state) {
	case OTG_STATE_UNDEFINED:
		msm_otg_reset(otg->phy);
		msm_otg_init_sm(motg);
		psy = power_supply_get_by_name("usb");
		if (!psy)
			pr_err("couldn't get usb power supply\n");
		otg->phy->state = OTG_STATE_B_IDLE;
		if (!test_bit(B_SESS_VLD, &motg->inputs) &&
				test_bit(ID, &motg->inputs)) {
			pm_runtime_put_noidle(otg->phy->dev);
			pm_runtime_suspend(otg->phy->dev);
#ifdef CONFIG_USB_MSM_OTG_SH_CUST
			chg_notify = 1;
#endif /* CONFIG_USB_MSM_OTG_SH_CUST */
			break;
		}
#ifdef CONFIG_USB_MSM_OTG_SH_CUST_MHL
		if( !test_bit(ID, &motg->inputs)) {
			shusb_read_data.shusb_mhl_status = true;
			work = 1;
			break;
		}
#endif /* CONFIG_USB_MSM_OTG_SH_CUST_MHL */
		/* FALL THROUGH */
	case OTG_STATE_B_IDLE:
#ifdef CONFIG_USB_MSM_OTG_SH_CUST
		if(shvbus_usb_force_disconnect) {
			cancel_delayed_work_sync(&motg->chg_work);
#ifdef CONFIG_USB_SH_CUST_NON_STANDARD_CHARGE
			msm_otg_stop_charge_timer();
#endif /* CONFIG_USB_SH_CUST_NON_STANDARD_CHARGE */
#ifdef CONFIG_USB_MSM_OTG_SH_CUST_CHARGE
			msm_otg_notify_chg_info(otg->phy, 0);
#else /* CONFIG_USB_MSM_OTG_SH_CUST_CHARGE */
			msm_otg_notify_charger(motg, 0);
#endif /* CONFIG_USB_MSM_OTG_SH_CUST_CHARGE */
			motg->chg_state = USB_CHG_STATE_UNDEFINED;
			motg->chg_type = USB_INVALID_CHARGER;
			msm_otg_reset(otg->phy);
			pm_runtime_put_noidle(otg->phy->dev);
			pm_runtime_suspend(otg->phy->dev);
			chg_notify = 1;
			break;
		}
#endif /* CONFIG_USB_MSM_OTG_SH_CUST */
		if ((!test_bit(ID, &motg->inputs) ||
				test_bit(ID_A, &motg->inputs)) && otg->host) {
#ifdef CONFIG_USB_MSM_OTG_SH_CUST
			chg_notify = 1;
			if (test_bit(B_SESS_VLD, &motg->inputs)) {
				cancel_delayed_work_sync(&motg->chg_work);
				motg->chg_state = USB_CHG_STATE_UNDEFINED;
				motg->chg_type = USB_INVALID_CHARGER;
				msm_otg_reset(otg->phy);
				pm_runtime_put_noidle(otg->phy->dev);
				pm_runtime_suspend(otg->phy->dev);
				break;
			}
#endif /* CONFIG_USB_MSM_OTG_SH_CUST */
			pr_debug("!id || id_A\n");
			clear_bit(B_BUS_REQ, &motg->inputs);
			set_bit(A_BUS_REQ, &motg->inputs);
			otg->phy->state = OTG_STATE_A_IDLE;
			work = 1;
		} else if (test_bit(B_SESS_VLD, &motg->inputs)) {
			pr_debug("b_sess_vld\n");
#ifdef CONFIG_USB_MSM_OTG_SH_CUST
			chg_notify = 1;
#endif /* CONFIG_USB_MSM_OTG_SH_CUST */
			switch (motg->chg_state) {
			case USB_CHG_STATE_UNDEFINED:
#ifdef CONFIG_USB_MSM_OTG_SH_CUST_SWIC
				msm_otg_swic_ch_status(motg);
				if(motg->chg_state == USB_CHG_STATE_DETECTED){
					work = 1;
					break;
				}
#else /* CONFIG_USB_MSM_OTG_SH_CUST_SWIC */
				msm_chg_detect_work(&motg->chg_work.work);
#endif /* CONFIG_USB_MSM_OTG_SH_CUST_SWIC */
				break;
			case USB_CHG_STATE_DETECTED:
				switch (motg->chg_type) {
				case USB_DCP_CHARGER:
					/* Enable VDP_SRC */
					ulpi_write(otg->phy, 0x2, 0x85);
#ifdef CONFIG_USB_MSM_OTG_SH_CUST_CHARGE
					msm_otg_notify_chg_info(otg->phy, IDEV_CHG_MAX);
#else /* CONFIG_USB_MSM_OTG_SH_CUST_CHARGE */
					/* fall through */
				case USB_PROPRIETARY_CHARGER:
					msm_otg_notify_charger(motg,
							IDEV_CHG_MAX);
#endif /* CONFIG_USB_MSM_OTG_SH_CUST_CHARGE */
					pm_runtime_put_noidle(otg->phy->dev);
					pm_runtime_suspend(otg->phy->dev);
					break;
#ifdef CONFIG_USB_MSM_OTG_SH_CUST_CHARGE
				case USB_PROPRIETARY_CHARGER:
					/* Enable VDP_SRC */
					ulpi_write(otg->phy, 0x2, 0x85);
					msm_otg_notify_chg_info(otg->phy, IDEV_CHG_MIN);
					pm_runtime_put_noidle(otg->phy->dev);
					pm_runtime_suspend(otg->phy->dev);
					break;
#endif /* CONFIG_USB_MSM_OTG_SH_CUST_CHARGE */
				case USB_ACA_B_CHARGER:
					msm_otg_notify_charger(motg,
							IDEV_ACA_CHG_MAX);
					/*
					 * (ID_B --> ID_C) PHY_ALT interrupt can
					 * not be detected in LPM.
					 */
					break;
				case USB_CDP_CHARGER:
					msm_otg_notify_charger(motg,
							IDEV_CHG_MAX);
					msm_otg_start_peripheral(otg, 1);
					otg->phy->state =
						OTG_STATE_B_PERIPHERAL;
					break;
				case USB_ACA_C_CHARGER:
					msm_otg_notify_charger(motg,
							IDEV_ACA_CHG_MAX);
					msm_otg_start_peripheral(otg, 1);
					otg->phy->state =
						OTG_STATE_B_PERIPHERAL;
					break;
				case USB_SDP_CHARGER:
#ifdef CONFIG_USB_SH_CUST_NON_STANDARD_CHARGE
					msm_otg_start_charge_timer();
					ret = msm_otg_start_peripheral(otg, 1);
					if(0 != ret)
						msm_otg_stop_charge_timer();
#else /* CONFIG_USB_SH_CUST_NON_STANDARD_CHARGE */
					msm_otg_start_peripheral(otg, 1);
#endif /* CONFIG_USB_SH_CUST_NON_STANDARD_CHARGE */
					otg->phy->state =
						OTG_STATE_B_PERIPHERAL;
					break;
				default:
					break;
				}
				break;
			default:
				break;
			}
		} else if (test_bit(B_BUS_REQ, &motg->inputs)) {
			pr_debug("b_sess_end && b_bus_req\n");
			if (msm_otg_start_srp(otg) < 0) {
				clear_bit(B_BUS_REQ, &motg->inputs);
				work = 1;
				break;
			}
			otg->phy->state = OTG_STATE_B_SRP_INIT;
			msm_otg_start_timer(motg, TB_SRP_FAIL, B_SRP_FAIL);
			break;
		} else {
			pr_debug("chg_work cancel");
			cancel_delayed_work_sync(&motg->chg_work);
#ifdef CONFIG_USB_SH_CUST_NON_STANDARD_CHARGE
			msm_otg_stop_charge_timer();
#endif /* CONFIG_USB_SH_CUST_NON_STANDARD_CHARGE */
#ifdef CONFIG_USB_MSM_OTG_SH_CUST_CHARGE
			if (motg->chg_state != USB_CHG_STATE_UNDEFINED)
				msm_otg_notify_chg_info(otg->phy, 0);
			motg->chg_state = USB_CHG_STATE_UNDEFINED;
			motg->chg_type = USB_INVALID_CHARGER;
#else /* CONFIG_USB_MSM_OTG_SH_CUST_CHARGE */
			motg->chg_state = USB_CHG_STATE_UNDEFINED;
			motg->chg_type = USB_INVALID_CHARGER;
			msm_otg_notify_charger(motg, 0);
#endif /* CONFIG_USB_MSM_OTG_SH_CUST_CHARGE */
			msm_otg_reset(otg->phy);
			pm_runtime_put_noidle(otg->phy->dev);
			pm_runtime_suspend(otg->phy->dev);
#ifdef CONFIG_USB_MSM_OTG_SH_CUST
			chg_notify = 1;
#endif /* CONFIG_USB_MSM_OTG_SH_CUST */
		}
		break;
	case OTG_STATE_B_SRP_INIT:
		if (!test_bit(ID, &motg->inputs) ||
				test_bit(ID_A, &motg->inputs) ||
				test_bit(ID_C, &motg->inputs) ||
				(test_bit(B_SESS_VLD, &motg->inputs) &&
				!test_bit(ID_B, &motg->inputs))) {
			pr_debug("!id || id_a/c || b_sess_vld+!id_b\n");
			msm_otg_del_timer(motg);
			otg->phy->state = OTG_STATE_B_IDLE;
			/*
			 * clear VBUSVLDEXTSEL and VBUSVLDEXT register
			 * bits after SRP initiation.
			 */
			ulpi_write(otg->phy, 0x0, 0x98);
			work = 1;
		} else if (test_bit(B_SRP_FAIL, &motg->tmouts)) {
			pr_debug("b_srp_fail\n");
			pr_info("A-device did not respond to SRP\n");
			clear_bit(B_BUS_REQ, &motg->inputs);
			clear_bit(B_SRP_FAIL, &motg->tmouts);
			otg_send_event(OTG_EVENT_NO_RESP_FOR_SRP);
			ulpi_write(otg->phy, 0x0, 0x98);
			otg->phy->state = OTG_STATE_B_IDLE;
			motg->b_last_se0_sess = jiffies;
			work = 1;
		}
		break;
	case OTG_STATE_B_PERIPHERAL:
		if (!test_bit(ID, &motg->inputs) ||
				test_bit(ID_A, &motg->inputs) ||
#ifdef CONFIG_USB_MSM_OTG_SH_CUST
				(sh_disconnect_wait_flg == 1) ||
#endif /* CONFIG_USB_MSM_OTG_SH_CUST */
				test_bit(ID_B, &motg->inputs) ||
				!test_bit(B_SESS_VLD, &motg->inputs)) {
#ifdef CONFIG_USB_MSM_OTG_SH_CUST
			if( sh_disconnect_wait_flg == 0 &&
				 test_bit(ID, &motg->inputs) ){
				queue_delayed_work(system_nrt_wq, &sh_usb_wait_work, 0);
				break;
			}

			if( cancel_delayed_work_sync(&sh_usb_wait_work) &&
				test_bit(B_SESS_VLD, &motg->inputs) ){
				dev_info(otg->phy->dev, "sh_usb_wait_work cancel OK mA=%d\n", shvbus_charge_wait_val);
				sh_disconnect_wait_flg = 0;
				if(shvbus_charge_wait_val)
					msm_otg_notify_charger(motg, shvbus_charge_wait_val);
				shvbus_charge_wait_val = 0;
				break;
			}

			msm_sh_wait_cancel();
#endif /* CONFIG_USB_MSM_OTG_SH_CUST */
			pr_debug("!id  || id_a/b || !b_sess_vld\n");
#ifndef CONFIG_USB_MSM_OTG_SH_CUST
			motg->chg_state = USB_CHG_STATE_UNDEFINED;
			motg->chg_type = USB_INVALID_CHARGER;
#endif /* CONFIG_USB_MSM_OTG_SH_CUST */
			msm_otg_notify_charger(motg, 0);
			srp_reqd = otg->gadget->otg_srp_reqd;
			msm_otg_start_peripheral(otg, 0);
			if (test_bit(ID_B, &motg->inputs))
				clear_bit(ID_B, &motg->inputs);
			clear_bit(B_BUS_REQ, &motg->inputs);
			otg->phy->state = OTG_STATE_B_IDLE;
			motg->b_last_se0_sess = jiffies;
			if (srp_reqd)
				msm_otg_start_timer(motg,
					TB_TST_SRP, B_TST_SRP);
			else
				work = 1;
		} else if (test_bit(B_BUS_REQ, &motg->inputs) &&
				otg->gadget->b_hnp_enable &&
				test_bit(A_BUS_SUSPEND, &motg->inputs)) {
			pr_debug("b_bus_req && b_hnp_en && a_bus_suspend\n");
			msm_otg_start_timer(motg, TB_ASE0_BRST, B_ASE0_BRST);
			/* D+ pullup should not be disconnected within 4msec
			 * after A device suspends the bus. Otherwise PET will
			 * fail the compliance test.
			 */
			udelay(1000);
			msm_otg_start_peripheral(otg, 0);
			otg->phy->state = OTG_STATE_B_WAIT_ACON;
			/*
			 * start HCD even before A-device enable
			 * pull-up to meet HNP timings.
			 */
			otg->host->is_b_host = 1;
			msm_otg_start_host(otg, 1);
		} else if (test_bit(A_BUS_SUSPEND, &motg->inputs) &&
				   test_bit(B_SESS_VLD, &motg->inputs)) {
			pr_debug("a_bus_suspend && b_sess_vld\n");
			if (motg->caps & ALLOW_LPM_ON_DEV_SUSPEND) {
				pm_runtime_put_noidle(otg->phy->dev);
				pm_runtime_suspend(otg->phy->dev);
			}
		} else if (test_bit(ID_C, &motg->inputs)) {
			msm_otg_notify_charger(motg, IDEV_ACA_CHG_MAX);
		}
		break;
	case OTG_STATE_B_WAIT_ACON:
		if (!test_bit(ID, &motg->inputs) ||
				test_bit(ID_A, &motg->inputs) ||
				test_bit(ID_B, &motg->inputs) ||
				!test_bit(B_SESS_VLD, &motg->inputs)) {
			pr_debug("!id || id_a/b || !b_sess_vld\n");
			msm_otg_del_timer(motg);
			/*
			 * A-device is physically disconnected during
			 * HNP. Remove HCD.
			 */
			msm_otg_start_host(otg, 0);
			otg->host->is_b_host = 0;

			clear_bit(B_BUS_REQ, &motg->inputs);
			clear_bit(A_BUS_SUSPEND, &motg->inputs);
			motg->b_last_se0_sess = jiffies;
			otg->phy->state = OTG_STATE_B_IDLE;
			msm_otg_reset(otg->phy);
			work = 1;
		} else if (test_bit(A_CONN, &motg->inputs)) {
			pr_debug("a_conn\n");
			clear_bit(A_BUS_SUSPEND, &motg->inputs);
			otg->phy->state = OTG_STATE_B_HOST;
			/*
			 * PET disconnects D+ pullup after reset is generated
			 * by B device in B_HOST role which is not detected by
			 * B device. As workaorund , start timer of 300msec
			 * and stop timer if A device is enumerated else clear
			 * A_CONN.
			 */
			msm_otg_start_timer(motg, TB_TST_CONFIG,
						B_TST_CONFIG);
		} else if (test_bit(B_ASE0_BRST, &motg->tmouts)) {
			pr_debug("b_ase0_brst_tmout\n");
			pr_info("B HNP fail:No response from A device\n");
			msm_otg_start_host(otg, 0);
			msm_otg_reset(otg->phy);
			otg->host->is_b_host = 0;
			clear_bit(B_ASE0_BRST, &motg->tmouts);
			clear_bit(A_BUS_SUSPEND, &motg->inputs);
			clear_bit(B_BUS_REQ, &motg->inputs);
			otg_send_event(OTG_EVENT_HNP_FAILED);
			otg->phy->state = OTG_STATE_B_IDLE;
			work = 1;
		} else if (test_bit(ID_C, &motg->inputs)) {
			msm_otg_notify_charger(motg, IDEV_ACA_CHG_MAX);
		}
		break;
	case OTG_STATE_B_HOST:
		if (!test_bit(B_BUS_REQ, &motg->inputs) ||
				!test_bit(A_CONN, &motg->inputs) ||
				!test_bit(B_SESS_VLD, &motg->inputs)) {
			pr_debug("!b_bus_req || !a_conn || !b_sess_vld\n");
			clear_bit(A_CONN, &motg->inputs);
			clear_bit(B_BUS_REQ, &motg->inputs);
			msm_otg_start_host(otg, 0);
			otg->host->is_b_host = 0;
			otg->phy->state = OTG_STATE_B_IDLE;
			msm_otg_reset(otg->phy);
			work = 1;
		} else if (test_bit(ID_C, &motg->inputs)) {
			msm_otg_notify_charger(motg, IDEV_ACA_CHG_MAX);
		}
		break;
	case OTG_STATE_A_IDLE:
		otg->default_a = 1;
		if (test_bit(ID, &motg->inputs) &&
			!test_bit(ID_A, &motg->inputs)) {
			pr_debug("id && !id_a\n");
			otg->default_a = 0;
			clear_bit(A_BUS_DROP, &motg->inputs);
			otg->phy->state = OTG_STATE_B_IDLE;
			del_timer_sync(&motg->id_timer);
			msm_otg_link_reset(motg);
			msm_chg_enable_aca_intr(motg);
			msm_otg_notify_charger(motg, 0);
			work = 1;
		} else if (!test_bit(A_BUS_DROP, &motg->inputs) &&
				(test_bit(A_SRP_DET, &motg->inputs) ||
				 test_bit(A_BUS_REQ, &motg->inputs))) {
			pr_debug("!a_bus_drop && (a_srp_det || a_bus_req)\n");

			clear_bit(A_SRP_DET, &motg->inputs);
			/* Disable SRP detection */
			writel_relaxed((readl_relaxed(USB_OTGSC) &
					~OTGSC_INTSTS_MASK) &
					~OTGSC_DPIE, USB_OTGSC);

			otg->phy->state = OTG_STATE_A_WAIT_VRISE;
			/* VBUS should not be supplied before end of SRP pulse
			 * generated by PET, if not complaince test fail.
			 */
			usleep_range(10000, 12000);
			/* ACA: ID_A: Stop charging untill enumeration */
			if (test_bit(ID_A, &motg->inputs))
				msm_otg_notify_charger(motg, 0);
			else
				msm_hsusb_vbus_power(motg, 1);
			msm_otg_start_timer(motg, TA_WAIT_VRISE, A_WAIT_VRISE);
		} else {
			pr_debug("No session requested\n");
			clear_bit(A_BUS_DROP, &motg->inputs);
			if (test_bit(ID_A, &motg->inputs)) {
					msm_otg_notify_charger(motg,
							IDEV_ACA_CHG_MAX);
			} else if (!test_bit(ID, &motg->inputs)) {
				msm_otg_notify_charger(motg, 0);
				/*
				 * A-device is not providing power on VBUS.
				 * Enable SRP detection.
				 */
				writel_relaxed(0x13, USB_USBMODE);
				writel_relaxed((readl_relaxed(USB_OTGSC) &
						~OTGSC_INTSTS_MASK) |
						OTGSC_DPIE, USB_OTGSC);
				mb();
			}
		}
		break;
	case OTG_STATE_A_WAIT_VRISE:
		if ((test_bit(ID, &motg->inputs) &&
				!test_bit(ID_A, &motg->inputs)) ||
				test_bit(A_BUS_DROP, &motg->inputs) ||
				test_bit(A_WAIT_VRISE, &motg->tmouts)) {
			pr_debug("id || a_bus_drop || a_wait_vrise_tmout\n");
			clear_bit(A_BUS_REQ, &motg->inputs);
			msm_otg_del_timer(motg);
			msm_hsusb_vbus_power(motg, 0);
			otg->phy->state = OTG_STATE_A_WAIT_VFALL;
			msm_otg_start_timer(motg, TA_WAIT_VFALL, A_WAIT_VFALL);
		} else if (test_bit(A_VBUS_VLD, &motg->inputs)) {
			pr_debug("a_vbus_vld\n");
			otg->phy->state = OTG_STATE_A_WAIT_BCON;
			if (TA_WAIT_BCON > 0)
				msm_otg_start_timer(motg, TA_WAIT_BCON,
					A_WAIT_BCON);
			msm_otg_start_host(otg, 1);
			msm_chg_enable_aca_det(motg);
			msm_chg_disable_aca_intr(motg);
			mod_timer(&motg->id_timer, ID_TIMER_FREQ);
			if (msm_chg_check_aca_intr(motg))
				work = 1;
		}
		break;
	case OTG_STATE_A_WAIT_BCON:
		if ((test_bit(ID, &motg->inputs) &&
				!test_bit(ID_A, &motg->inputs)) ||
				test_bit(A_BUS_DROP, &motg->inputs) ||
				test_bit(A_WAIT_BCON, &motg->tmouts)) {
			pr_debug("(id && id_a/b/c) || a_bus_drop ||"
					"a_wait_bcon_tmout\n");
			if (test_bit(A_WAIT_BCON, &motg->tmouts)) {
				pr_info("Device No Response\n");
				otg_send_event(OTG_EVENT_DEV_CONN_TMOUT);
			}
			msm_otg_del_timer(motg);
			clear_bit(A_BUS_REQ, &motg->inputs);
			clear_bit(B_CONN, &motg->inputs);
			msm_otg_start_host(otg, 0);
			/*
			 * ACA: ID_A with NO accessory, just the A plug is
			 * attached to ACA: Use IDCHG_MAX for charging
			 */
			if (test_bit(ID_A, &motg->inputs))
				msm_otg_notify_charger(motg, IDEV_CHG_MIN);
			else
				msm_hsusb_vbus_power(motg, 0);
			otg->phy->state = OTG_STATE_A_WAIT_VFALL;
			msm_otg_start_timer(motg, TA_WAIT_VFALL, A_WAIT_VFALL);
		} else if (!test_bit(A_VBUS_VLD, &motg->inputs)) {
			pr_debug("!a_vbus_vld\n");
			clear_bit(B_CONN, &motg->inputs);
			msm_otg_del_timer(motg);
			msm_otg_start_host(otg, 0);
			otg->phy->state = OTG_STATE_A_VBUS_ERR;
			msm_otg_reset(otg->phy);
		} else if (test_bit(ID_A, &motg->inputs)) {
			msm_hsusb_vbus_power(motg, 0);
		} else if (!test_bit(A_BUS_REQ, &motg->inputs)) {
			/*
			 * If TA_WAIT_BCON is infinite, we don;t
			 * turn off VBUS. Enter low power mode.
			 */
			if (TA_WAIT_BCON < 0)
				pm_runtime_put_sync(otg->phy->dev);
		} else if (!test_bit(ID, &motg->inputs)) {
			msm_hsusb_vbus_power(motg, 1);
		}
		break;
	case OTG_STATE_A_HOST:
		if ((test_bit(ID, &motg->inputs) &&
				!test_bit(ID_A, &motg->inputs)) ||
				test_bit(A_BUS_DROP, &motg->inputs)) {
			pr_debug("id_a/b/c || a_bus_drop\n");
			clear_bit(B_CONN, &motg->inputs);
			clear_bit(A_BUS_REQ, &motg->inputs);
			msm_otg_del_timer(motg);
			otg->phy->state = OTG_STATE_A_WAIT_VFALL;
			msm_otg_start_host(otg, 0);
			if (!test_bit(ID_A, &motg->inputs))
				msm_hsusb_vbus_power(motg, 0);
			msm_otg_start_timer(motg, TA_WAIT_VFALL, A_WAIT_VFALL);
		} else if (!test_bit(A_VBUS_VLD, &motg->inputs)) {
			pr_debug("!a_vbus_vld\n");
			clear_bit(B_CONN, &motg->inputs);
			msm_otg_del_timer(motg);
			otg->phy->state = OTG_STATE_A_VBUS_ERR;
			msm_otg_start_host(otg, 0);
			msm_otg_reset(otg->phy);
		} else if (!test_bit(A_BUS_REQ, &motg->inputs)) {
			/*
			 * a_bus_req is de-asserted when root hub is
			 * suspended or HNP is in progress.
			 */
			pr_debug("!a_bus_req\n");
			msm_otg_del_timer(motg);
			otg->phy->state = OTG_STATE_A_SUSPEND;
			if (otg->host->b_hnp_enable)
				msm_otg_start_timer(motg, TA_AIDL_BDIS,
						A_AIDL_BDIS);
			else
				pm_runtime_put_sync(otg->phy->dev);
		} else if (!test_bit(B_CONN, &motg->inputs)) {
			pr_debug("!b_conn\n");
			msm_otg_del_timer(motg);
			otg->phy->state = OTG_STATE_A_WAIT_BCON;
			if (TA_WAIT_BCON > 0)
				msm_otg_start_timer(motg, TA_WAIT_BCON,
					A_WAIT_BCON);
			if (msm_chg_check_aca_intr(motg))
				work = 1;
		} else if (test_bit(ID_A, &motg->inputs)) {
			msm_otg_del_timer(motg);
			msm_hsusb_vbus_power(motg, 0);
			if (motg->chg_type == USB_ACA_DOCK_CHARGER)
				msm_otg_notify_charger(motg,
						IDEV_ACA_CHG_MAX);
			else
				msm_otg_notify_charger(motg,
						IDEV_CHG_MIN - motg->mA_port);
		} else if (!test_bit(ID, &motg->inputs)) {
			motg->chg_state = USB_CHG_STATE_UNDEFINED;
			motg->chg_type = USB_INVALID_CHARGER;
			msm_otg_notify_charger(motg, 0);
			msm_hsusb_vbus_power(motg, 1);
		}
		break;
	case OTG_STATE_A_SUSPEND:
		if ((test_bit(ID, &motg->inputs) &&
				!test_bit(ID_A, &motg->inputs)) ||
				test_bit(A_BUS_DROP, &motg->inputs) ||
				test_bit(A_AIDL_BDIS, &motg->tmouts)) {
			pr_debug("id_a/b/c || a_bus_drop ||"
					"a_aidl_bdis_tmout\n");
			msm_otg_del_timer(motg);
			clear_bit(B_CONN, &motg->inputs);
			otg->phy->state = OTG_STATE_A_WAIT_VFALL;
			msm_otg_start_host(otg, 0);
			msm_otg_reset(otg->phy);
			if (!test_bit(ID_A, &motg->inputs))
				msm_hsusb_vbus_power(motg, 0);
			msm_otg_start_timer(motg, TA_WAIT_VFALL, A_WAIT_VFALL);
		} else if (!test_bit(A_VBUS_VLD, &motg->inputs)) {
			pr_debug("!a_vbus_vld\n");
			msm_otg_del_timer(motg);
			clear_bit(B_CONN, &motg->inputs);
			otg->phy->state = OTG_STATE_A_VBUS_ERR;
			msm_otg_start_host(otg, 0);
			msm_otg_reset(otg->phy);
		} else if (!test_bit(B_CONN, &motg->inputs) &&
				otg->host->b_hnp_enable) {
			pr_debug("!b_conn && b_hnp_enable");
			otg->phy->state = OTG_STATE_A_PERIPHERAL;
			msm_otg_host_hnp_enable(otg, 1);
			otg->gadget->is_a_peripheral = 1;
			msm_otg_start_peripheral(otg, 1);
		} else if (!test_bit(B_CONN, &motg->inputs) &&
				!otg->host->b_hnp_enable) {
			pr_debug("!b_conn && !b_hnp_enable");
			/*
			 * bus request is dropped during suspend.
			 * acquire again for next device.
			 */
			set_bit(A_BUS_REQ, &motg->inputs);
			otg->phy->state = OTG_STATE_A_WAIT_BCON;
			if (TA_WAIT_BCON > 0)
				msm_otg_start_timer(motg, TA_WAIT_BCON,
					A_WAIT_BCON);
		} else if (test_bit(ID_A, &motg->inputs)) {
			msm_hsusb_vbus_power(motg, 0);
			msm_otg_notify_charger(motg,
					IDEV_CHG_MIN - motg->mA_port);
		} else if (!test_bit(ID, &motg->inputs)) {
			msm_otg_notify_charger(motg, 0);
			msm_hsusb_vbus_power(motg, 1);
		}
		break;
	case OTG_STATE_A_PERIPHERAL:
		if ((test_bit(ID, &motg->inputs) &&
				!test_bit(ID_A, &motg->inputs)) ||
				test_bit(A_BUS_DROP, &motg->inputs)) {
			pr_debug("id _f/b/c || a_bus_drop\n");
			/* Clear BIDL_ADIS timer */
			msm_otg_del_timer(motg);
			otg->phy->state = OTG_STATE_A_WAIT_VFALL;
			msm_otg_start_peripheral(otg, 0);
			otg->gadget->is_a_peripheral = 0;
			msm_otg_start_host(otg, 0);
			msm_otg_reset(otg->phy);
			if (!test_bit(ID_A, &motg->inputs))
				msm_hsusb_vbus_power(motg, 0);
			msm_otg_start_timer(motg, TA_WAIT_VFALL, A_WAIT_VFALL);
		} else if (!test_bit(A_VBUS_VLD, &motg->inputs)) {
			pr_debug("!a_vbus_vld\n");
			/* Clear BIDL_ADIS timer */
			msm_otg_del_timer(motg);
			otg->phy->state = OTG_STATE_A_VBUS_ERR;
			msm_otg_start_peripheral(otg, 0);
			otg->gadget->is_a_peripheral = 0;
			msm_otg_start_host(otg, 0);
		} else if (test_bit(A_BIDL_ADIS, &motg->tmouts)) {
			pr_debug("a_bidl_adis_tmout\n");
			msm_otg_start_peripheral(otg, 0);
			otg->gadget->is_a_peripheral = 0;
			otg->phy->state = OTG_STATE_A_WAIT_BCON;
			set_bit(A_BUS_REQ, &motg->inputs);
			msm_otg_host_hnp_enable(otg, 0);
			if (TA_WAIT_BCON > 0)
				msm_otg_start_timer(motg, TA_WAIT_BCON,
					A_WAIT_BCON);
		} else if (test_bit(ID_A, &motg->inputs)) {
			msm_hsusb_vbus_power(motg, 0);
			msm_otg_notify_charger(motg,
					IDEV_CHG_MIN - motg->mA_port);
		} else if (!test_bit(ID, &motg->inputs)) {
			msm_otg_notify_charger(motg, 0);
			msm_hsusb_vbus_power(motg, 1);
		}
		break;
	case OTG_STATE_A_WAIT_VFALL:
#ifdef CONFIG_USB_MSM_OTG_SH_CUST
		if ((!test_bit(ID, &motg->inputs) ||
				test_bit(ID_A, &motg->inputs)) && otg->host) {
			pr_debug("id_low || id_A\n");
			msm_otg_del_timer(motg);
			clear_bit(A_VBUS_VLD, &motg->inputs);
			clear_bit(A_BUS_DROP, &motg->inputs);
			otg->phy->state = OTG_STATE_B_IDLE;
			del_timer_sync(&motg->id_timer);
			msm_otg_link_reset(motg);
			msm_chg_enable_aca_intr(motg);
			msm_otg_notify_charger(motg, 0);
			work = 1;
		} else if (test_bit(A_WAIT_VFALL, &motg->tmouts)) {
#else /* CONFIG_USB_MSM_OTG_SH_CUST */
		if (test_bit(A_WAIT_VFALL, &motg->tmouts)) {
#endif /* CONFIG_USB_MSM_OTG_SH_CUST */
			clear_bit(A_VBUS_VLD, &motg->inputs);
			otg->phy->state = OTG_STATE_A_IDLE;
			work = 1;
		}
		break;
	case OTG_STATE_A_VBUS_ERR:
		if ((test_bit(ID, &motg->inputs) &&
				!test_bit(ID_A, &motg->inputs)) ||
				test_bit(A_BUS_DROP, &motg->inputs) ||
				test_bit(A_CLR_ERR, &motg->inputs)) {
			otg->phy->state = OTG_STATE_A_WAIT_VFALL;
			if (!test_bit(ID_A, &motg->inputs))
				msm_hsusb_vbus_power(motg, 0);
			msm_otg_start_timer(motg, TA_WAIT_VFALL, A_WAIT_VFALL);
			motg->chg_state = USB_CHG_STATE_UNDEFINED;
			motg->chg_type = USB_INVALID_CHARGER;
			msm_otg_notify_charger(motg, 0);
		}
		break;
	default:
		break;
	}
#ifdef CONFIG_USB_MSM_OTG_SH_CUST
	if(!msm_otg_sm_work_lockdec())
		work = 1;

	if (chg_notify)
		msm_otg_change_cable_notify();
#endif /* CONFIG_USB_MSM_OTG_SH_CUST */
	if (work)
		queue_work(system_nrt_wq, &motg->sm_work);
}

#ifdef CONFIG_USB_MSM_OTG_SH_CUST
#ifdef CONFIG_USB_MSM_OTG_SH_CUST_SWIC
	#define SH_DISCON_WAIT_TIME		(720 * HZ/1000) /* 720 msec */
#else /* CONFIG_USB_MSM_OTG_SH_CUST_SWIC */
	#define SH_DISCON_WAIT_TIME		(300 * HZ/1000) /* 300 msec */
#endif /* CONFIG_USB_MSM_OTG_SH_CUST_SWIC */
static void msm_sh_wait_work(struct work_struct *w)
{
	struct msm_otg *motg = the_msm_otg;
#ifdef CONFIG_USB_ANDROID_SH_CUST
	struct usb_otg *otg = motg->phy.otg;
#endif /* CONFIG_USB_ANDROID_SH_CUST */

#ifdef CONFIG_USB_DEBUG_SH_LOG
	dev_info(otg->phy->dev, "sh_wait_work state=%d inputs=%d chg_state=%d chg_type=%d sh_wait=%d \n"
	,(int)otg->phy->state ,(int)motg->inputs , (int)motg->chg_state , (int)motg->chg_type, (int)sh_disconnect_wait_flg);
#endif /* CONFIG_USB_DEBUG_SH_LOG */

	if(sh_disconnect_wait_flg == 0){
		sh_disconnect_wait_flg = 1;
#ifdef CONFIG_USB_ANDROID_SH_CUST
		/* Pullup OFF */
		usb_gadget_disconnect_internal(otg->gadget);
#endif /* CONFIG_USB_ANDROID_SH_CUST */
		queue_delayed_work(system_nrt_wq, &sh_usb_wait_work,  SH_DISCON_WAIT_TIME);

	}else{
		queue_work(system_nrt_wq, &motg->sm_work);

	}

}

static void msm_sh_wait_cancel(void)
{
	cancel_delayed_work_sync(&sh_usb_wait_work);
	sh_disconnect_wait_flg = 0;
	shvbus_charge_wait_val = 0;
}

#endif /* CONFIG_USB_MSM_OTG_SH_CUST */

#ifdef CONFIG_USB_SWIC
static void msm_otg_swic_irq(u8 device, void *data)
{
	struct msm_otg *motg = the_msm_otg;
	struct usb_phy *phy = &motg->phy;

	dev_info(phy->dev, "msm_otg_swic_irq device=%d \n", device);

	sh_swic_device = device;

	if(!data){
		pr_err("%s: data is NULL\n", __func__);
		return;
	}

	msm_otg_swic_irq_start(data);

	return;
}

static void msm_otg_swic_irq_start(void *data)
{
	struct msm_otg *motg = data;

	msm_otg_swic_ch_inputs(motg);

	queue_work(system_nrt_wq, &motg->sm_work);
}

#endif /* CONFIG_USB_SWIC */

#ifdef CONFIG_USB_MSM_OTG_SH_CUST_HW_CHECK
static bool msm_otg_hw_check_init(void)
{
	int swic_set;
	if(swic_hw_init)
		return swic_hw_init;

	swic_set = shswic_is_implement();
	if(swic_set == SHSWIC_INIT)
		return swic_hw_init;

	swic_hw_init = true;
	shusb_swic_enable = (swic_set == SHSWIC_IMPLEMENT)? true: false ;

#ifdef CONFIG_USB_DEBUG_SH_LOG
	pr_info("%s hw=%d enable=%d \n",__func__,(int)swic_set ,(int)shusb_swic_enable);
#endif /* CONFIG_USB_DEBUG_SH_LOG */
	return swic_hw_init;
}

#define POLL_WAIT_TIME  150
#define POLL_COUNT      10
static void msm_otg_hw_check(void)
{
	int cnt = POLL_COUNT;

	/* wait time */
	while( !msm_otg_hw_check_init() && (cnt > 0))
	{
		msleep(POLL_WAIT_TIME); /* wait time */
		cnt--;
		pr_debug("%s hw check=%d \n",__func__,(int)cnt);
	}
}
#endif /* CONFIG_USB_MSM_OTG_SH_CUST_HW_CHECK */

#ifdef CONFIG_USB_MSM_OTG_SH_CUST_MHL

static void msm_otg_mhl_irq_start(shmhl_detect_device_t device)
{
	struct msm_otg *motg = the_msm_otg;

#ifdef CONFIG_USB_MSM_OTG_SH_CUST_HW_CHECK
	if (swic_enabled())
		return;
#endif /* CONFIG_USB_MSM_OTG_SH_CUST_HW_CHECK */

#ifdef CONFIG_USB_DEBUG_SH_LOG
	pr_info("%s %d\n" ,__func__, (int)device);
#endif /* CONFIG_USB_DEBUG_SH_LOG */

	switch(device)
	{
	case SHMHL_DEVICE_MHL:
		sh_usb_mhl_flag = false;
		shusb_read_data.shusb_mhl_status = false;
		shusb_read_data.shusb_mhl_result = device;
		break;
	case SHMHL_DEVICE_NONE:
		msm_otg_mhl_power( motg, false );

		sh_usb_mhl_flag = false;
		if(!is_id_line_low() ) {
			set_bit(ID, &motg->inputs);
			shusb_read_data.shusb_mhl_status = false;
		} else {
			clear_bit(ID, &motg->inputs);
			shusb_read_data.shusb_mhl_status = true;
		}
		shusb_read_data.shusb_mhl_result = device;
		break;
	case SHMHL_DEVICE_USB_B:
	case SHMHL_DEVICE_UNKNOWN:
	case SHMHL_DEVICE_ACA_MCPC_CHG:
	case SHMHL_DEVICE_USB_A_CHG:
	case SHMHL_DEVICE_STANDBY:
	case SHMHL_DEVICE_USB_B_ACA:
	case SHMHL_DEVICE_MCPC_ACA_OPN:
	case SHMHL_DEVICE_MAX:
	default :
		sh_usb_mhl_flag = false;
		shusb_read_data.shusb_mhl_status = false;
		shusb_read_data.shusb_mhl_result = device;
		break;
	}

	queue_work(system_nrt_wq, &motg->sm_work);
}

static void msm_otg_mhl_callback(shmhl_detect_device_t device, void* user_data)
{
	msm_otg_mhl_irq_start(device);
}

static void msm_otg_mhl_callback_setting(bool on)
{
	sh_usb_mhl_flag = false;
	shusb_read_data.shusb_mhl_result = SHMHL_DEVICE_UNKNOWN;
	shusb_read_data.shusb_mhl_status = false;

#ifdef CONFIG_USB_MSM_OTG_SH_CUST_HW_CHECK
	if (swic_enabled())
		return;
#endif /* CONFIG_USB_MSM_OTG_SH_CUST_HW_CHECK */
	pr_debug("%s %d \n" ,__func__, (int)on);

	if(on)
		shmhl_detect_cb_regist(msm_otg_mhl_callback, NULL);
	else
		shmhl_detect_cb_regist(NULL, NULL);


}

static void msm_otg_mhl_irq(void)
{
	struct msm_otg *motg = the_msm_otg;
	bool id_value = true;
	bool vbus_value = false;

#ifdef CONFIG_USB_MSM_OTG_SH_CUST_HW_CHECK
	if (swic_enabled())
		return;
#endif /* CONFIG_USB_MSM_OTG_SH_CUST_HW_CHECK */

	msm_otg_mhl_callback_setting(true);

	if(!test_bit(ID, &motg->inputs))
			id_value = false;

	if(test_bit(B_SESS_VLD, &motg->inputs))
		vbus_value = SHMHL_VBUS_STATE_ON;

	msm_otg_mhl_power( motg, true );

	sh_usb_mhl_flag = true;

	shmhl_notify_id_vbus_state(id_value, vbus_value);
}

static void msm_otg_mhl_power(struct msm_otg *motg, bool on)
{
	static struct regulator *msm_otg_mhl_regulator;
	static bool shusb_mhl_on = false;

#ifdef CONFIG_USB_MSM_OTG_SH_CUST_HW_CHECK
	if (swic_enabled())
		return;
#endif /* CONFIG_USB_MSM_OTG_SH_CUST_HW_CHECK */

#ifdef CONFIG_USB_DEBUG_SH_LOG
	pr_info("%s %d \n" ,__func__, (int)on);
#endif /* CONFIG_USB_DEBUG_SH_LOG */

	if (shusb_mhl_on == on)
		return;

	if(!on)
		goto msm_otg_mhl_power_disable;

	msm_otg_mhl_regulator = regulator_get(motg->phy.dev, "vbus_shusb_mhl");
	if (IS_ERR(msm_otg_mhl_regulator))
	{
		pr_err("%s: regulator_get err \n", __func__);
		return ;
	}

	if (regulator_enable(msm_otg_mhl_regulator))
	{
		pr_err("%s: regulator_enable ON err \n", __func__);
		regulator_put(msm_otg_mhl_regulator);
		msm_otg_mhl_regulator = NULL;
		return ;
	}

	shusb_mhl_on = on;
	return ;

msm_otg_mhl_power_disable:
	regulator_disable(msm_otg_mhl_regulator);
	regulator_put(msm_otg_mhl_regulator);
	shusb_mhl_on = false;

}

static void msm_otg_mhl_init(void)
{
	sh_usb_mhl_flag = false;
	shusb_read_data.shusb_mhl_status = false;
	shusb_read_data.shusb_mhl_result = SHMHL_DEVICE_UNKNOWN;
}


static bool msm_otg_mhl_sm_work(struct work_struct *w)
{
	struct msm_otg *motg = container_of(w, struct msm_otg, sm_work);
	struct usb_phy *phy = &motg->phy;

	if( sh_usb_mhl_flag ){
#ifdef CONFIG_USB_DEBUG_SH_LOG
		pr_info("msm_otg_mhl_sm_work wait mhl callback \n");
#endif /* CONFIG_USB_DEBUG_SH_LOG */
		return false;
	}

	if( shusb_read_data.shusb_mhl_result  == SHMHL_DEVICE_MHL){
		bool id_value = true;
		bool vbus_value = false;
#ifdef CONFIG_USB_DEBUG_SH_LOG
		pr_info("msm_otg_mhl_sm_work mhl work \n");
#endif /* CONFIG_USB_DEBUG_SH_LOG */

		if (!is_id_line_low()){
			id_value = true;
			set_bit(ID, &motg->inputs);
		} else {
			id_value = false;
			clear_bit(ID, &motg->inputs);
		}

		if(test_bit(B_SESS_VLD, &motg->inputs))
			vbus_value = true;

		shmhl_notify_id_vbus_state(id_value, vbus_value);
		msm_otg_change_cable_notify();
		return false;
	}

	if( shusb_id_interrupts ){
		if (!is_id_line_low()){
			shusb_read_data.shusb_mhl_status = false;
			set_bit(ID, &motg->inputs);
		} else {
			if( test_bit(ID, &motg->inputs) )
				shusb_read_data.shusb_mhl_status = true;
			clear_bit(ID, &motg->inputs);
		}
		shusb_id_interrupts = false;
	}

	if( shusb_read_data.shusb_mhl_status && (phy->state == OTG_STATE_B_IDLE)){
		shusb_read_data.shusb_mhl_status = false;
#ifdef CONFIG_USB_DEBUG_SH_LOG
		pr_info("msm_otg_mhl_sm_work start \n");
#endif /* CONFIG_USB_DEBUG_SH_LOG */
		msm_otg_mhl_irq();
		return false;
	}

	return true;
}
#endif /* CONFIG_USB_MSM_OTG_SH_CUST_MHL */
#ifdef CONFIG_USB_MSM_OTG_SH_CUST
static bool msm_otg_id_check(void)
{
	shbatt_adc_t adc;
	shbatt_result_t bat_res = SHBATT_RESULT_SUCCESS;
	bool tmp_H_L[OTG_ID_CHECK_READ_COUNT];
	bool chk_H_L = 0;
	bool chk_res = false;
	bool ret_val =false;
	int i = 0, j = 0, success_cnt = 0;
	int L_cnt = 0;
	struct msm_otg *dev = the_msm_otg;

	/* channel set to USB_ID */
	adc.channel = SHBATT_ADC_CHANNEL_USB_ID;

	/* id check */
	for (i = 0; i < OTG_ID_CHECK_RETRY_COUNT ; i++) {
		if (wake_lock_active(&dev->wlock)) {
			pr_debug("msm_otg_id_check: while wake_lock \n");
			msleep(OTG_ID_CHECK_WAIT_TIME);
		} else {
			pr_debug("msm_otg_id_check: no wake_lock \n");
			wake_lock(&dev->wlock);
			/* wait ms */
			msleep(OTG_ID_CHECK_WAIT_TIME);
			wake_unlock(&dev->wlock);
		}

		/* get USB_ID from shbatt */
		for (j = 0; j < OTG_ID_CHECK_READ_COUNT ; j++) {
			bat_res = shbatt_api_read_adc_channel_no_conversion( &adc );
			if ( (bat_res == SHBATT_RESULT_SUCCESS)
			  && (adc.physical >= OTG_ID_CHECK_MIN_THRESHOLD && adc.physical <= OTG_ID_CHECK_MAX_THRESHOLD) ) {
				tmp_H_L[j] = true;
				L_cnt++;
			} else
				tmp_H_L[j] = false;
		}
		/* id check consider to chattering */
		chk_H_L = tmp_H_L[OTG_ID_CHECK_READ_COUNT-1];
		for (j = OTG_ID_CHECK_READ_COUNT-2; j >= 0 ; j--) {
			success_cnt = (chk_H_L == tmp_H_L[j])? success_cnt+1:0;
			chk_H_L = tmp_H_L[j];
			/* detect check */
			if (success_cnt >= OTG_ID_CHECK_SUCCESS_COUNT-1)
				break;
		}
		/* if success, check end. if fail, retry check */
		if (success_cnt >= OTG_ID_CHECK_SUCCESS_COUNT-1){
			chk_res = true;
			break;
		}
		L_cnt = 0;
	}

	/* check success */
	if (chk_res) {
		if (chk_H_L)
			ret_val = true;
	/* check fail */
	} else {
		/* adopt more ones */
		if (L_cnt > (OTG_ID_CHECK_READ_COUNT / 2))
			ret_val = true;
	}

	pr_info("%s id_check_result= %d adc.physical=%d\n", __func__, ret_val, (int)adc.physical);
	return ret_val;
}
#endif /* CONFIG_USB_MSM_OTG_SH_CUST */

static irqreturn_t msm_otg_irq(int irq, void *data)
{
	struct msm_otg *motg = data;
	struct usb_otg *otg = motg->phy.otg;
	u32 otgsc = 0, usbsts, pc;
	bool work = 0;
	irqreturn_t ret = IRQ_HANDLED;

	if (atomic_read(&motg->in_lpm)) {
		pr_debug("OTG IRQ: in LPM\n");
		disable_irq_nosync(irq);
		motg->async_int = 1;
		if (!atomic_read(&motg->pm_suspended))
			pm_request_resume(otg->phy->dev);
		return IRQ_HANDLED;
	}

	usbsts = readl(USB_USBSTS);
	otgsc = readl(USB_OTGSC);

	if (!(otgsc & OTG_OTGSTS_MASK) && !(usbsts & OTG_USBSTS_MASK))
		return IRQ_NONE;

	if ((otgsc & OTGSC_IDIS) && (otgsc & OTGSC_IDIE)) {
		if (otgsc & OTGSC_ID) {
			pr_debug("Id set\n");
			set_bit(ID, &motg->inputs);
		} else {
			pr_debug("Id clear\n");
			/*
			 * Assert a_bus_req to supply power on
			 * VBUS when Micro/Mini-A cable is connected
			 * with out user intervention.
			 */
			set_bit(A_BUS_REQ, &motg->inputs);
			clear_bit(ID, &motg->inputs);
			msm_chg_enable_aca_det(motg);
		}
		writel_relaxed(otgsc, USB_OTGSC);
		work = 1;
	} else if (otgsc & OTGSC_DPIS) {
		pr_debug("DPIS detected\n");
		writel_relaxed(otgsc, USB_OTGSC);
		set_bit(A_SRP_DET, &motg->inputs);
		set_bit(A_BUS_REQ, &motg->inputs);
		work = 1;
	} else if (otgsc & OTGSC_BSVIS) {
		writel_relaxed(otgsc, USB_OTGSC);
		/*
		 * BSV interrupt comes when operating as an A-device
		 * (VBUS on/off).
		 * But, handle BSV when charger is removed from ACA in ID_A
		 */
		if ((otg->phy->state >= OTG_STATE_A_IDLE) &&
			!test_bit(ID_A, &motg->inputs))
			return IRQ_HANDLED;
		if (otgsc & OTGSC_BSV) {
			pr_debug("BSV set\n");
			set_bit(B_SESS_VLD, &motg->inputs);
		} else {
			pr_debug("BSV clear\n");
			clear_bit(B_SESS_VLD, &motg->inputs);
			clear_bit(A_BUS_SUSPEND, &motg->inputs);

			msm_chg_check_aca_intr(motg);
		}
		work = 1;
	} else if (usbsts & STS_PCI) {
		pc = readl_relaxed(USB_PORTSC);
		pr_debug("portsc = %x\n", pc);
		ret = IRQ_NONE;
		/*
		 * HCD Acks PCI interrupt. We use this to switch
		 * between different OTG states.
		 */
		work = 1;
		switch (otg->phy->state) {
		case OTG_STATE_A_SUSPEND:
			if (otg->host->b_hnp_enable && (pc & PORTSC_CSC) &&
					!(pc & PORTSC_CCS)) {
				pr_debug("B_CONN clear\n");
				clear_bit(B_CONN, &motg->inputs);
				msm_otg_del_timer(motg);
			}
			break;
		case OTG_STATE_A_PERIPHERAL:
			/*
			 * A-peripheral observed activity on bus.
			 * clear A_BIDL_ADIS timer.
			 */
			msm_otg_del_timer(motg);
			work = 0;
			break;
		case OTG_STATE_B_WAIT_ACON:
			if ((pc & PORTSC_CSC) && (pc & PORTSC_CCS)) {
				pr_debug("A_CONN set\n");
				set_bit(A_CONN, &motg->inputs);
				/* Clear ASE0_BRST timer */
				msm_otg_del_timer(motg);
			}
			break;
		case OTG_STATE_B_HOST:
			if ((pc & PORTSC_CSC) && !(pc & PORTSC_CCS)) {
				pr_debug("A_CONN clear\n");
				clear_bit(A_CONN, &motg->inputs);
				msm_otg_del_timer(motg);
			}
			break;
		case OTG_STATE_A_WAIT_BCON:
			if (TA_WAIT_BCON < 0)
				set_bit(A_BUS_REQ, &motg->inputs);
		default:
			work = 0;
			break;
		}
	} else if (usbsts & STS_URI) {
		ret = IRQ_NONE;
		switch (otg->phy->state) {
		case OTG_STATE_A_PERIPHERAL:
			/*
			 * A-peripheral observed activity on bus.
			 * clear A_BIDL_ADIS timer.
			 */
			msm_otg_del_timer(motg);
			work = 0;
			break;
		default:
			work = 0;
			break;
		}
	} else if (usbsts & STS_SLI) {
		ret = IRQ_NONE;
		work = 0;
		switch (otg->phy->state) {
		case OTG_STATE_B_PERIPHERAL:
			if (otg->gadget->b_hnp_enable) {
				set_bit(A_BUS_SUSPEND, &motg->inputs);
				set_bit(B_BUS_REQ, &motg->inputs);
				work = 1;
			}
			break;
		case OTG_STATE_A_PERIPHERAL:
			msm_otg_start_timer(motg, TA_BIDL_ADIS,
					A_BIDL_ADIS);
			break;
		default:
			break;
		}
	} else if ((usbsts & PHY_ALT_INT)) {
		writel_relaxed(PHY_ALT_INT, USB_USBSTS);
		if (msm_chg_check_aca_intr(motg))
			work = 1;
		ret = IRQ_HANDLED;
	}
	if (work)
		queue_work(system_nrt_wq, &motg->sm_work);

	return ret;
}

#ifndef CONFIG_USB_MSM_OTG_SH_CUST_SWIC
static void msm_otg_set_vbus_state(int online)
{
	static bool init;
	struct msm_otg *motg = the_msm_otg;
	struct usb_otg *otg = motg->phy.otg;

	/* In A Host Mode, ignore received BSV interrupts */
	if (otg->phy->state >= OTG_STATE_A_IDLE)
		return;

	if (online) {
		pr_debug("PMIC: BSV set\n");
		set_bit(B_SESS_VLD, &motg->inputs);
	} else {
		pr_debug("PMIC: BSV clear\n");
		clear_bit(B_SESS_VLD, &motg->inputs);
	}

	if (!init) {
		init = true;
		complete(&pmic_vbus_init);
		pr_debug("PMIC: BSV init complete\n");
		return;
	}

	if (atomic_read(&motg->pm_suspended))
		motg->sm_work_pending = true;
	else
		queue_work(system_nrt_wq, &motg->sm_work);
}

static void msm_pmic_id_status_w(struct work_struct *w)
{
	struct msm_otg *motg = container_of(w, struct msm_otg,
						pmic_id_status_work.work);
	int work = 0;
	unsigned long flags;

	local_irq_save(flags);
#ifdef CONFIG_USB_MSM_OTG_SH_CUST
	shusb_id_interrupts = true;
	if (irq_read_line(motg->pdata->pmic_id_irq))
		pr_debug("PMIC: ID set\n");
	else
		pr_debug("PMIC: ID clear\n");
	work = 1;
#else /* CONFIG_USB_MSM_OTG_SH_CUST */
	if (irq_read_line(motg->pdata->pmic_id_irq)) {
		if (!test_and_set_bit(ID, &motg->inputs)) {
			pr_debug("PMIC: ID set\n");
			work = 1;
		}
	} else {
		if (test_and_clear_bit(ID, &motg->inputs)) {
			pr_debug("PMIC: ID clear\n");
			set_bit(A_BUS_REQ, &motg->inputs);
			work = 1;
		}
	}
#endif /* CONFIG_USB_MSM_OTG_SH_CUST */

	if (work && (motg->phy.state != OTG_STATE_UNDEFINED)) {
		if (atomic_read(&motg->pm_suspended))
			motg->sm_work_pending = true;
		else
			queue_work(system_nrt_wq, &motg->sm_work);
	}
	local_irq_restore(flags);

}

#define MSM_PMIC_ID_STATUS_DELAY	5 /* 5msec */
static irqreturn_t msm_pmic_id_irq(int irq, void *data)
{
	struct msm_otg *motg = data;

	if (!aca_id_turned_on)
		/*schedule delayed work for 5msec for ID line state to settle*/
		queue_delayed_work(system_nrt_wq, &motg->pmic_id_status_work,
				msecs_to_jiffies(MSM_PMIC_ID_STATUS_DELAY));

	return IRQ_HANDLED;
}
#endif /* CONFIG_USB_MSM_OTG_SH_CUST_SWIC */

static int msm_otg_mode_show(struct seq_file *s, void *unused)
{
	struct msm_otg *motg = s->private;
	struct usb_phy *phy = &motg->phy;

	switch (phy->state) {
	case OTG_STATE_A_HOST:
		seq_printf(s, "host\n");
		break;
	case OTG_STATE_B_PERIPHERAL:
		seq_printf(s, "peripheral\n");
		break;
	default:
		seq_printf(s, "none\n");
		break;
	}

	return 0;
}

static int msm_otg_mode_open(struct inode *inode, struct file *file)
{
	return single_open(file, msm_otg_mode_show, inode->i_private);
}

static ssize_t msm_otg_mode_write(struct file *file, const char __user *ubuf,
				size_t count, loff_t *ppos)
{
	struct seq_file *s = file->private_data;
	struct msm_otg *motg = s->private;
	char buf[16];
	struct usb_phy *phy = &motg->phy;
	int status = count;
	enum usb_mode_type req_mode;

	memset(buf, 0x00, sizeof(buf));

	if (copy_from_user(&buf, ubuf, min_t(size_t, sizeof(buf) - 1, count))) {
		status = -EFAULT;
		goto out;
	}

	if (!strncmp(buf, "host", 4)) {
		req_mode = USB_HOST;
	} else if (!strncmp(buf, "peripheral", 10)) {
		req_mode = USB_PERIPHERAL;
	} else if (!strncmp(buf, "none", 4)) {
		req_mode = USB_NONE;
	} else {
		status = -EINVAL;
		goto out;
	}

	switch (req_mode) {
	case USB_NONE:
		switch (phy->state) {
		case OTG_STATE_A_HOST:
		case OTG_STATE_B_PERIPHERAL:
			set_bit(ID, &motg->inputs);
			clear_bit(B_SESS_VLD, &motg->inputs);
			break;
		default:
			goto out;
		}
		break;
	case USB_PERIPHERAL:
		switch (phy->state) {
		case OTG_STATE_B_IDLE:
		case OTG_STATE_A_HOST:
			set_bit(ID, &motg->inputs);
			set_bit(B_SESS_VLD, &motg->inputs);
			break;
		default:
			goto out;
		}
		break;
	case USB_HOST:
		switch (phy->state) {
		case OTG_STATE_B_IDLE:
		case OTG_STATE_B_PERIPHERAL:
			clear_bit(ID, &motg->inputs);
			break;
		default:
			goto out;
		}
		break;
	default:
		goto out;
	}

	pm_runtime_resume(phy->dev);
	queue_work(system_nrt_wq, &motg->sm_work);
out:
	return status;
}

const struct file_operations msm_otg_mode_fops = {
	.open = msm_otg_mode_open,
	.read = seq_read,
	.write = msm_otg_mode_write,
	.llseek = seq_lseek,
	.release = single_release,
};

static int msm_otg_show_otg_state(struct seq_file *s, void *unused)
{
	struct msm_otg *motg = s->private;
	struct usb_phy *phy = &motg->phy;

	seq_printf(s, "%s\n", otg_state_string(phy->state));
	return 0;
}

static int msm_otg_otg_state_open(struct inode *inode, struct file *file)
{
	return single_open(file, msm_otg_show_otg_state, inode->i_private);
}

const struct file_operations msm_otg_state_fops = {
	.open = msm_otg_otg_state_open,
	.read = seq_read,
	.llseek = seq_lseek,
	.release = single_release,
};

static int msm_otg_show_chg_type(struct seq_file *s, void *unused)
{
	struct msm_otg *motg = s->private;

	seq_printf(s, "%s\n", chg_to_string(motg->chg_type));
	return 0;
}

static int msm_otg_chg_open(struct inode *inode, struct file *file)
{
	return single_open(file, msm_otg_show_chg_type, inode->i_private);
}

const struct file_operations msm_otg_chg_fops = {
	.open = msm_otg_chg_open,
	.read = seq_read,
	.llseek = seq_lseek,
	.release = single_release,
};

static int msm_otg_aca_show(struct seq_file *s, void *unused)
{
	if (debug_aca_enabled)
		seq_printf(s, "enabled\n");
	else
		seq_printf(s, "disabled\n");

	return 0;
}

static int msm_otg_aca_open(struct inode *inode, struct file *file)
{
	return single_open(file, msm_otg_aca_show, inode->i_private);
}

static ssize_t msm_otg_aca_write(struct file *file, const char __user *ubuf,
				size_t count, loff_t *ppos)
{
	char buf[8];

	memset(buf, 0x00, sizeof(buf));

	if (copy_from_user(&buf, ubuf, min_t(size_t, sizeof(buf) - 1, count)))
		return -EFAULT;

	if (!strncmp(buf, "enable", 6))
		debug_aca_enabled = true;
	else
		debug_aca_enabled = false;

	return count;
}

const struct file_operations msm_otg_aca_fops = {
	.open = msm_otg_aca_open,
	.read = seq_read,
	.write = msm_otg_aca_write,
	.llseek = seq_lseek,
	.release = single_release,
};

static int msm_otg_bus_show(struct seq_file *s, void *unused)
{
	if (debug_bus_voting_enabled)
		seq_printf(s, "enabled\n");
	else
		seq_printf(s, "disabled\n");

	return 0;
}

static int msm_otg_bus_open(struct inode *inode, struct file *file)
{
	return single_open(file, msm_otg_bus_show, inode->i_private);
}

static ssize_t msm_otg_bus_write(struct file *file, const char __user *ubuf,
				size_t count, loff_t *ppos)
{
	char buf[8];
	int ret;
	struct seq_file *s = file->private_data;
	struct msm_otg *motg = s->private;

	memset(buf, 0x00, sizeof(buf));

	if (copy_from_user(&buf, ubuf, min_t(size_t, sizeof(buf) - 1, count)))
		return -EFAULT;

	if (!strncmp(buf, "enable", 6)) {
		/* Do not vote here. Let OTG statemachine decide when to vote */
		debug_bus_voting_enabled = true;
	} else {
		debug_bus_voting_enabled = false;
		if (motg->bus_perf_client) {
			ret = msm_bus_scale_client_update_request(
					motg->bus_perf_client, 0);
			if (ret)
				dev_err(motg->phy.dev, "%s: Failed to devote "
					   "for bus bw %d\n", __func__, ret);
		}
	}

	return count;
}

const struct file_operations msm_otg_bus_fops = {
	.open = msm_otg_bus_open,
	.read = seq_read,
	.write = msm_otg_bus_write,
	.llseek = seq_lseek,
	.release = single_release,
};

static struct dentry *msm_otg_dbg_root;

static int msm_otg_debugfs_init(struct msm_otg *motg)
{
	struct dentry *msm_otg_dentry;

	msm_otg_dbg_root = debugfs_create_dir("msm_otg", NULL);

	if (!msm_otg_dbg_root || IS_ERR(msm_otg_dbg_root))
		return -ENODEV;

	if (motg->pdata->mode == USB_OTG &&
		motg->pdata->otg_control == OTG_USER_CONTROL) {

		msm_otg_dentry = debugfs_create_file("mode", S_IRUGO |
			S_IWUSR, msm_otg_dbg_root, motg,
			&msm_otg_mode_fops);

		if (!msm_otg_dentry) {
			debugfs_remove(msm_otg_dbg_root);
			msm_otg_dbg_root = NULL;
			return -ENODEV;
		}
	}

	msm_otg_dentry = debugfs_create_file("chg_type", S_IRUGO,
		msm_otg_dbg_root, motg,
		&msm_otg_chg_fops);

	if (!msm_otg_dentry) {
		debugfs_remove_recursive(msm_otg_dbg_root);
		return -ENODEV;
	}

	msm_otg_dentry = debugfs_create_file("aca", S_IRUGO | S_IWUSR,
		msm_otg_dbg_root, motg,
		&msm_otg_aca_fops);

	if (!msm_otg_dentry) {
		debugfs_remove_recursive(msm_otg_dbg_root);
		return -ENODEV;
	}

	msm_otg_dentry = debugfs_create_file("bus_voting", S_IRUGO | S_IWUSR,
		msm_otg_dbg_root, motg,
		&msm_otg_bus_fops);

	if (!msm_otg_dentry) {
		debugfs_remove_recursive(msm_otg_dbg_root);
		return -ENODEV;
	}

	msm_otg_dentry = debugfs_create_file("otg_state", S_IRUGO,
				msm_otg_dbg_root, motg, &msm_otg_state_fops);

	if (!msm_otg_dentry) {
		debugfs_remove_recursive(msm_otg_dbg_root);
		return -ENODEV;
	}
	return 0;
}

static void msm_otg_debugfs_cleanup(void)
{
	debugfs_remove_recursive(msm_otg_dbg_root);
}

static u64 msm_otg_dma_mask = DMA_BIT_MASK(64);
static struct platform_device *msm_otg_add_pdev(
		struct platform_device *ofdev, const char *name)
{
	struct platform_device *pdev;
	const struct resource *res = ofdev->resource;
	unsigned int num = ofdev->num_resources;
	int retval;

	pdev = platform_device_alloc(name, -1);
	if (!pdev) {
		retval = -ENOMEM;
		goto error;
	}

	pdev->dev.coherent_dma_mask = DMA_BIT_MASK(32);
	pdev->dev.dma_mask = &msm_otg_dma_mask;

	if (num) {
		retval = platform_device_add_resources(pdev, res, num);
		if (retval)
			goto error;
	}

	retval = platform_device_add(pdev);
	if (retval)
		goto error;

	return pdev;

error:
	platform_device_put(pdev);
	return ERR_PTR(retval);
}

static int msm_otg_setup_devices(struct platform_device *ofdev,
		enum usb_mode_type mode, bool init)
{
	const char *gadget_name = "msm_hsusb";
	const char *host_name = "msm_hsusb_host";
	static struct platform_device *gadget_pdev;
	static struct platform_device *host_pdev;
	int retval = 0;

	if (!init) {
		if (gadget_pdev)
			platform_device_unregister(gadget_pdev);
		if (host_pdev)
			platform_device_unregister(host_pdev);
		return 0;
	}

	switch (mode) {
	case USB_OTG:
		/* fall through */
	case USB_PERIPHERAL:
		gadget_pdev = msm_otg_add_pdev(ofdev, gadget_name);
		if (IS_ERR(gadget_pdev)) {
			retval = PTR_ERR(gadget_pdev);
			break;
		}
		if (mode == USB_PERIPHERAL)
			break;
		/* fall through */
	case USB_HOST:
		host_pdev = msm_otg_add_pdev(ofdev, host_name);
		if (IS_ERR(host_pdev)) {
			retval = PTR_ERR(host_pdev);
			if (mode == USB_OTG)
				platform_device_unregister(gadget_pdev);
		}
		break;
	default:
		break;
	}

	return retval;
}

struct msm_otg_platform_data *msm_otg_dt_to_pdata(struct platform_device *pdev)
{
	struct device_node *node = pdev->dev.of_node;
	struct msm_otg_platform_data *pdata;
	int len = 0;

	pdata = devm_kzalloc(&pdev->dev, sizeof(*pdata), GFP_KERNEL);
	if (!pdata) {
		pr_err("unable to allocate platform data\n");
		return NULL;
	}
	of_get_property(node, "qcom,hsusb-otg-phy-init-seq", &len);
	if (len) {
		pdata->phy_init_seq = devm_kzalloc(&pdev->dev, len, GFP_KERNEL);
		if (!pdata->phy_init_seq)
			return NULL;
		of_property_read_u32_array(node, "qcom,hsusb-otg-phy-init-seq",
				pdata->phy_init_seq,
				len/sizeof(*pdata->phy_init_seq));
	}
	of_property_read_u32(node, "qcom,hsusb-otg-power-budget",
				&pdata->power_budget);
	of_property_read_u32(node, "qcom,hsusb-otg-mode",
				&pdata->mode);
	of_property_read_u32(node, "qcom,hsusb-otg-otg-control",
				&pdata->otg_control);
	of_property_read_u32(node, "qcom,hsusb-otg-default-mode",
				&pdata->default_mode);
	of_property_read_u32(node, "qcom,hsusb-otg-phy-type",
				&pdata->phy_type);
	of_property_read_u32(node, "qcom,hsusb-otg-pmic-id-irq",
				&pdata->pmic_id_irq);
	return pdata;
}

#ifdef CONFIG_USB_MSM_OTG_SH_CUST
static ssize_t
show_usb_force_disconnect(struct device *dev, struct device_attribute *attr,
			char *buf)
{
	size_t val;

	val = sprintf(buf, "%d", (int)shvbus_usb_force_disconnect);

	return val;
}

static ssize_t
store_usb_force_disconnect(struct device *_dev, struct device_attribute *attr,
			const char *buf, size_t size)
{
	struct msm_otg *dev = the_msm_otg;
	size_t value = -EINVAL;
#ifdef CONFIG_USB_SWIC
	shswic_result_t swic_ret;
#endif /* CONFIG_USB_SWIC */
#ifndef CONFIG_USB_MSM_OTG_SH_CUST_SWIC
	struct msm_otg *motg = dev;
	int ret = 0;
#endif /* CONFIG_USB_MSM_OTG_SH_CUST_SWIC  */

	if ((size == 0) || (size > 2) || (buf == NULL)){
		pr_err("%s: invalid parmeter\n", __func__);
		return -EINVAL;
	}

	if ((size == 2) && (*(buf+1) != 0x0a) && (*(buf+1) != 0x00))
		return -EINVAL;

	switch(*buf){
	case '0':
		if (shvbus_usb_force_disconnect == 1) {
#ifndef CONFIG_USB_MSM_OTG_SH_CUST_SWIC
			if (dev->pdata->otg_control == OTG_PMIC_CONTROL) {
				if (dev->pdata->pmic_id_irq) {
					ret = request_irq(dev->pdata->pmic_id_irq,
								msm_pmic_id_irq,
								IRQF_TRIGGER_RISING |
								IRQF_TRIGGER_FALLING,
								"msm_otg", dev);
					if (ret) {
						dev_err(_dev, "request irq failed for PMIC ID\n");
					}
				} else {
					ret = -ENODEV;
					dev_err(_dev, "PMIC IRQ for ID notifications doesn't exist\n");
				}
			}
			if (dev->pdata->otg_control == OTG_PMIC_CONTROL)
				pm8921_charger_register_vbus_sn(&msm_otg_set_vbus_state);
#endif /* CONFIG_USB_MSM_OTG_SH_CUST_SWIC */

#ifdef CONFIG_USB_SWIC
			swic_ret = shswic_detect_cb_regist(SHSWIC_VBUS_DEVICE, 
				USB_SWIC_INTERRUPTS_VALUE, 
				(void*)msm_otg_swic_irq, dev);
			if (swic_ret) {
				pr_err("%s: shswic_detect_cb_regist err\n",
					__func__);
				value = -EINVAL;
				break;
			}
#endif /* CONFIG_USB_SWIC */
#ifndef CONFIG_USB_MSM_OTG_SH_CUST_SWIC
			ret = request_irq(dev->irq, msm_otg_irq, IRQF_SHARED,
							"msm_otg", dev);
			if (ret) {
				dev_err(_dev, "request irq failed\n");
				value = ret;
				break;
			}

			writel(readl(USB_OTGSC) | OTGSC_BSVIE, USB_OTGSC);
#endif /* CONFIG_USB_MSM_OTG_SH_CUST_SWIC */

			msm_otg_resume(dev);
			msm_otg_init_sm(dev);
			wake_lock(&dev->wlock);
			shvbus_usb_force_disconnect = 0;
			queue_work(system_nrt_wq, &dev->sm_work);
		}
		value = size;
		break;
	case '1':
		if (shvbus_usb_force_disconnect == 0) {
#ifdef CONFIG_USB_SWIC
			swic_ret = shswic_detect_cb_regist(SHSWIC_VBUS_DEVICE
						, 0, (void*)NULL, NULL);
			if (swic_ret) {
				pr_err("%s: shswic_detect_cb_regist err\n",
					__func__);
				value = -EINVAL;
				break;
			}
#endif /* CONFIG_USB_SWIC */

#ifndef CONFIG_USB_MSM_OTG_SH_CUST_SWIC
			writel(readl(USB_OTGSC) & ~OTGSC_BSVIE, USB_OTGSC);

			free_irq(dev->irq, dev);

			if (dev->pdata->otg_control == OTG_PMIC_CONTROL)
				pm8921_charger_unregister_vbus_sn(0);
			
			if (dev->pdata->pmic_id_irq)
				free_irq(dev->pdata->pmic_id_irq, dev);
#endif /* CONFIG_USB_MSM_OTG_SH_CUST_SWIC */

			cancel_delayed_work_sync(&dev->chg_work);
			cancel_work_sync(&dev->sm_work);
			
			if (atomic_read(&dev->in_lpm))
				msm_otg_resume(dev);
			
			clear_bit(B_SESS_VLD, &dev->inputs);
			
			wake_lock(&dev->wlock);
			sh_disconnect_wait_flg = 1;
			shvbus_usb_force_disconnect = 1;
			queue_work(system_nrt_wq, &dev->sm_work);
		}
		value = size;
		break;
	default:
		value = -EINVAL;
		pr_err("%s: invalid parmeter = %d\n", __func__, (int)*buf);
		break;
	}
	
	return value;
}

static DEVICE_ATTR(usb_force_disconnect, 0664
		, show_usb_force_disconnect, store_usb_force_disconnect);

static ssize_t
show_usb_cable_type(struct device *sdev, struct device_attribute *attr, char *buf)
{
	size_t val;
	char *cable_str[] = {"host", "peripheral", "none"};
	int cable_type;

	cable_type = get_cable_type();
	val = snprintf(buf, PAGE_SIZE, "%s\n", cable_str[cable_type]);

	return val;
}

static DEVICE_ATTR(usb_cable_type, 0444, show_usb_cable_type, NULL);

#endif /* CONFIG_USB_MSM_OTG_SH_CUST */

#ifdef CONFIG_USB_SH_CUST_NON_STANDARD_CHARGE
static int msm_otg_notify_enumeration_started(struct usb_phy *phy)
{
	queue_work(system_nrt_wq, &charge_timer_work);
	return 0;
}

static void msm_otg_charge_timer_work_handler(struct work_struct *work)
{
	msm_otg_stop_charge_timer();
}

static void msm_otg_charge_timer_handler(unsigned long data)
{
	struct msm_otg *motg = (struct msm_otg *)data;
	struct usb_otg *otg = motg->phy.otg;
	int is_selfpowered = 0;
	unsigned power = 0;

	dev_info(otg->phy->dev, "charge timer expired\n");
	is_selfpowered = usb_gadget_is_selfpowered(otg->gadget);
	if(is_selfpowered)
		power = 0;
	else
#ifdef CONFIG_USB_MSM_OTG_SH_CUST_CHARGE
		power = IDEV_CHG_MIN;
#else /* CONFIG_USB_MSM_OTG_SH_CUST_CHARGE */
		power = IDEV_CHG_MAX;
#endif /* CONFIG_USB_MSM_OTG_SH_CUST_CHARGE */

#ifdef CONFIG_USB_MSM_OTG_SH_CUST_CHARGE
	msm_otg_notify_chg_info(otg->phy, power);
#else /* CONFIG_USB_MSM_OTG_SH_CUST_CHARGE */
	msm_otg_notify_charger(motg, power);
#endif /* CONFIG_USB_MSM_OTG_SH_CUST_CHARGE */
}

static void msm_otg_init_charge_timer(struct msm_otg *motg)
{
	charge_timer.function = msm_otg_charge_timer_handler;
	charge_timer.expires  = 0;
	charge_timer.data     = (unsigned long)motg;

	init_timer(&charge_timer);
}

static void msm_otg_start_charge_timer(void)
{
	spin_lock(&charge_timer_lock);

	if(timer_pending(&charge_timer)){
		del_timer_sync(&charge_timer);
	}

	charge_timer.expires  = jiffies + msecs_to_jiffies(OTG_CHARGE_TIMER_TIMEOUT);
	add_timer(&charge_timer);

	spin_unlock(&charge_timer_lock);
}

static void msm_otg_stop_charge_timer(void)
{
	spin_lock(&charge_timer_lock);

	if(timer_pending(&charge_timer)){
		del_timer_sync(&charge_timer);
	}

	spin_unlock(&charge_timer_lock);
}
#endif /* CONFIG_USB_SH_CUST_NON_STANDARD_CHARGE */

static int __init msm_otg_probe(struct platform_device *pdev)
{
	int ret = 0;
	struct resource *res;
	struct msm_otg *motg;
	struct usb_phy *phy;
	struct msm_otg_platform_data *pdata;
#ifdef CONFIG_USB_SWIC
	shswic_result_t swic_ret;
#endif /* CONFIG_USB_SWIC */

	dev_info(&pdev->dev, "msm_otg probe\n");

	if (pdev->dev.of_node) {
		dev_dbg(&pdev->dev, "device tree enabled\n");
		pdata = msm_otg_dt_to_pdata(pdev);
		if (!pdata)
			return -ENOMEM;
		ret = msm_otg_setup_devices(pdev, pdata->mode, true);
		if (ret) {
			dev_err(&pdev->dev, "devices setup failed\n");
			return ret;
		}
	} else if (!pdev->dev.platform_data) {
		dev_err(&pdev->dev, "No platform data given. Bailing out\n");
		return -ENODEV;
	} else {
		pdata = pdev->dev.platform_data;
	}

	motg = kzalloc(sizeof(struct msm_otg), GFP_KERNEL);
	if (!motg) {
		dev_err(&pdev->dev, "unable to allocate msm_otg\n");
		return -ENOMEM;
	}

	motg->phy.otg = kzalloc(sizeof(struct usb_otg), GFP_KERNEL);
	if (!motg->phy.otg) {
		dev_err(&pdev->dev, "unable to allocate usb_otg\n");
		ret = -ENOMEM;
		goto free_motg;
	}

	the_msm_otg = motg;
	motg->pdata = pdata;
	phy = &motg->phy;
	phy->dev = &pdev->dev;

	/*
	 * ACA ID_GND threshold range is overlapped with OTG ID_FLOAT.  Hence
	 * PHY treat ACA ID_GND as float and no interrupt is generated.  But
	 * PMIC can detect ACA ID_GND and generate an interrupt.
	 */
	if (aca_enabled() && motg->pdata->otg_control != OTG_PMIC_CONTROL) {
		dev_err(&pdev->dev, "ACA can not be enabled without PMIC\n");
		ret = -EINVAL;
		goto free_otg;
	}

	/* initialize reset counter */
	motg->reset_counter = 0;

	/* Some targets don't support PHY clock. */
	motg->phy_reset_clk = clk_get(&pdev->dev, "phy_clk");
	if (IS_ERR(motg->phy_reset_clk))
		dev_err(&pdev->dev, "failed to get phy_clk\n");

	/*
	 * Targets on which link uses asynchronous reset methodology,
	 * free running clock is not required during the reset.
	 */
	motg->clk = clk_get(&pdev->dev, "alt_core_clk");
	if (IS_ERR(motg->clk))
		dev_dbg(&pdev->dev, "alt_core_clk is not present\n");
	else
		clk_set_rate(motg->clk, 60000000);

	/*
	 * USB Core is running its protocol engine based on CORE CLK,
	 * CORE CLK  must be running at >55Mhz for correct HSUSB
	 * operation and USB core cannot tolerate frequency changes on
	 * CORE CLK. For such USB cores, vote for maximum clk frequency
	 * on pclk source
	 */
	motg->core_clk = clk_get(&pdev->dev, "core_clk");
	if (IS_ERR(motg->core_clk)) {
		motg->core_clk = NULL;
		dev_err(&pdev->dev, "failed to get core_clk\n");
		ret = PTR_ERR(motg->core_clk);
		goto put_clk;
	}
	clk_set_rate(motg->core_clk, INT_MAX);

	motg->pclk = clk_get(&pdev->dev, "iface_clk");
	if (IS_ERR(motg->pclk)) {
		dev_err(&pdev->dev, "failed to get iface_clk\n");
		ret = PTR_ERR(motg->pclk);
		goto put_core_clk;
	}

	res = platform_get_resource(pdev, IORESOURCE_MEM, 0);
	if (!res) {
		dev_err(&pdev->dev, "failed to get platform resource mem\n");
		ret = -ENODEV;
		goto put_pclk;
	}

	motg->regs = ioremap(res->start, resource_size(res));
	if (!motg->regs) {
		dev_err(&pdev->dev, "ioremap failed\n");
		ret = -ENOMEM;
		goto put_pclk;
	}
#ifdef CONFIG_USB_DEBUG_SH_LOG
	dev_info(&pdev->dev, "OTG regs = %p\n", motg->regs);
#endif /* CONFIG_USB_DEBUG_SH_LOG */

	motg->irq = platform_get_irq(pdev, 0);
	if (!motg->irq) {
		dev_err(&pdev->dev, "platform_get_irq failed\n");
		ret = -ENODEV;
		goto free_regs;
	}

	motg->xo_handle = msm_xo_get(MSM_XO_TCXO_D0, "usb");
	if (IS_ERR(motg->xo_handle)) {
		dev_err(&pdev->dev, "%s not able to get the handle "
			"to vote for TCXO D0 buffer\n", __func__);
		ret = PTR_ERR(motg->xo_handle);
		goto free_regs;
	}

	ret = msm_xo_mode_vote(motg->xo_handle, MSM_XO_MODE_ON);
	if (ret) {
		dev_err(&pdev->dev, "%s failed to vote for TCXO "
			"D0 buffer%d\n", __func__, ret);
		goto free_xo_handle;
	}

	clk_prepare_enable(motg->pclk);

	motg->vdd_type = VDDCX_CORNER;
	hsusb_vddcx = devm_regulator_get(motg->phy.dev, "hsusb_vdd_dig");
	if (IS_ERR(hsusb_vddcx)) {
		hsusb_vddcx = devm_regulator_get(motg->phy.dev, "HSUSB_VDDCX");
		if (IS_ERR(hsusb_vddcx)) {
			dev_err(motg->phy.dev, "unable to get hsusb vddcx\n");
			ret = PTR_ERR(hsusb_vddcx);
			goto devote_xo_handle;
		}
		motg->vdd_type = VDDCX;
	}

	ret = msm_hsusb_config_vddcx(1);
	if (ret) {
		dev_err(&pdev->dev, "hsusb vddcx configuration failed\n");
		goto devote_xo_handle;
	}

	ret = regulator_enable(hsusb_vddcx);
	if (ret) {
		dev_err(&pdev->dev, "unable to enable the hsusb vddcx\n");
		goto free_config_vddcx;
	}

	ret = msm_hsusb_ldo_init(motg, 1);
	if (ret) {
		dev_err(&pdev->dev, "hsusb vreg configuration failed\n");
		goto free_hsusb_vddcx;
	}

	if (pdata->mhl_enable) {
		mhl_usb_hs_switch = devm_regulator_get(motg->phy.dev,
							"mhl_usb_hs_switch");
		if (IS_ERR(mhl_usb_hs_switch)) {
			dev_err(&pdev->dev, "Unable to get mhl_usb_hs_switch\n");
			ret = PTR_ERR(mhl_usb_hs_switch);
			goto free_ldo_init;
		}
	}

	ret = msm_hsusb_ldo_enable(motg, 1);
	if (ret) {
		dev_err(&pdev->dev, "hsusb vreg enable failed\n");
		goto free_ldo_init;
	}
	clk_prepare_enable(motg->core_clk);

	writel(0, USB_USBINTR);
	writel(0, USB_OTGSC);
	/* Ensure that above STOREs are completed before enabling interrupts */
	mb();

	wake_lock_init(&motg->wlock, WAKE_LOCK_SUSPEND, "msm_otg");
	msm_otg_init_timer(motg);
	INIT_WORK(&motg->sm_work, msm_otg_sm_work);
	INIT_DELAYED_WORK(&motg->chg_work, msm_chg_detect_work);
#ifndef CONFIG_USB_MSM_OTG_SH_CUST_SWIC
	INIT_DELAYED_WORK(&motg->pmic_id_status_work, msm_pmic_id_status_w);
#endif /* CONFIG_USB_MSM_OTG_SH_CUST_SWIC */
	setup_timer(&motg->id_timer, msm_otg_id_timer_func,
				(unsigned long) motg);
#ifdef CONFIG_USB_MSM_OTG_SH_CUST
	INIT_DELAYED_WORK(&sh_usb_wait_work, msm_sh_wait_work);
	msm_otg_sm_work_lockinit();
#endif /* CONFIG_USB_MSM_OTG_SH_CUST */
#ifdef CONFIG_USB_SH_CUST_NON_STANDARD_CHARGE
	INIT_WORK(&charge_timer_work, msm_otg_charge_timer_work_handler);
#endif /* CONFIG_USB_SH_CUST_NON_STANDARD_CHARGE */

#ifdef CONFIG_USB_SWIC
	swic_ret = shswic_detect_cb_regist(SHSWIC_VBUS_DEVICE, 
				USB_SWIC_INTERRUPTS_VALUE, 
				(void*)msm_otg_swic_irq, motg);
	if (swic_ret) {
		dev_err(&pdev->dev, "shswic_detect_cb_regist failed\n");
		ret = -EINVAL;
		goto destroy_wlock;
	}
#endif /* CONFIG_USB_SWIC */
#ifndef CONFIG_USB_MSM_OTG_SH_CUST_SWIC
	ret = request_irq(motg->irq, msm_otg_irq, IRQF_SHARED,
					"msm_otg", motg);
	if (ret) {
		dev_err(&pdev->dev, "request irq failed\n");
		goto destroy_wlock;
	}
#endif /* CONFIG_USB_MSM_OTG_SH_CUST_SWIC */

	if (pdata->otg_control == OTG_PHY_CONTROL && pdata->mpm_otgsessvld_int)
		msm_mpm_enable_pin(pdata->mpm_otgsessvld_int, 1);

	phy->init = msm_otg_reset;
	phy->set_power = msm_otg_set_power;
	phy->set_suspend = msm_otg_set_suspend;
#ifdef CONFIG_USB_MSM_OTG_SH_CUST_CHARGE
	phy->notify_chg_info = msm_otg_notify_chg_info;
#endif /* CONFIG_USB_MSM_OTG_SH_CUST_CHARGE */
#ifdef CONFIG_USB_SH_CUST_NON_STANDARD_CHARGE
	phy->notify_enumeration_started = msm_otg_notify_enumeration_started;
#endif /* CONFIG_USB_SH_CUST_NON_STANDARD_CHARGE */

	phy->io_ops = &msm_otg_io_ops;

	phy->otg->phy = &motg->phy;
	phy->otg->set_host = msm_otg_set_host;
	phy->otg->set_peripheral = msm_otg_set_peripheral;
	phy->otg->start_hnp = msm_otg_start_hnp;
	phy->otg->start_srp = msm_otg_start_srp;

	ret = usb_set_transceiver(&motg->phy);
	if (ret) {
		dev_err(&pdev->dev, "usb_set_transceiver failed\n");
		goto free_irq;
	}

#ifndef CONFIG_USB_MSM_OTG_SH_CUST_SWIC
	if (motg->pdata->mode == USB_OTG &&
		motg->pdata->otg_control == OTG_PMIC_CONTROL) {
		if (motg->pdata->pmic_id_irq) {
			ret = request_irq(motg->pdata->pmic_id_irq,
						msm_pmic_id_irq,
						IRQF_TRIGGER_RISING |
						IRQF_TRIGGER_FALLING,
						"msm_otg", motg);
			if (ret) {
				dev_err(&pdev->dev, "request irq failed for PMIC ID\n");
				goto remove_phy;
			}
		} else {
			ret = -ENODEV;
			dev_err(&pdev->dev, "PMIC IRQ for ID notifications doesn't exist\n");
			goto remove_phy;
		}
	}
#endif /* CONFIG_USB_MSM_OTG_SH_CUST_SWIC */

	msm_hsusb_mhl_switch_enable(motg, 1);

#ifdef CONFIG_USB_SH_CUST_NON_STANDARD_CHARGE
	msm_otg_init_charge_timer(motg);
#endif /* CONFIG_USB_SH_CUST_NON_STANDARD_CHARGE */

#ifdef CONFIG_USB_MSM_OTG_SH_CUST_HW_CHECK
	msm_otg_hw_check_init();
#endif /* CONFIG_USB_MSM_OTG_SH_CUST_HW_CHECK */

#ifdef CONFIG_USB_MSM_OTG_SH_CUST_MHL
	msm_otg_mhl_init();
	msm_otg_mhl_power( motg, true );
#endif /* CONFIG_USB_MSM_OTG_SH_CUST_MHL */


	platform_set_drvdata(pdev, motg);
	device_init_wakeup(&pdev->dev, 1);
	motg->mA_port = IUNIT;

	ret = msm_otg_debugfs_init(motg);
	if (ret)
		dev_dbg(&pdev->dev, "mode debugfs file is"
			"not available\n");
#ifdef CONFIG_USB_MSM_OTG_SH_CUST
	ret = device_create_file(&pdev->dev, &dev_attr_usb_force_disconnect);
	if (ret < 0) {
		pr_err("%s: Failed to create the file entry\n", __func__);
		goto remove_phy;
	}

	ret = device_create_file(&pdev->dev, &dev_attr_usb_cable_type);
	if (ret < 0) {
		device_remove_file(&pdev->dev, &dev_attr_usb_force_disconnect);
		pr_err("%s: Failed to create the file entry(usb_cable_type)\n", __func__);
		goto remove_phy;
	}
#endif /* CONFIG_USB_MSM_OTG_SH_CUST */

#ifdef CONFIG_USB_MSM_OTG_SH_CUST_SWIC
	motg->caps = ALLOW_PHY_POWER_COLLAPSE |
		ALLOW_PHY_RETENTION;
#else /* CONFIG_USB_MSM_OTG_SH_CUST_SWIC */
	if (motg->pdata->otg_control == OTG_PMIC_CONTROL)
		pm8921_charger_register_vbus_sn(&msm_otg_set_vbus_state);

	if (motg->pdata->phy_type == SNPS_28NM_INTEGRATED_PHY) {
		if (motg->pdata->otg_control == OTG_PMIC_CONTROL &&
			(!(motg->pdata->mode == USB_OTG) ||
			 motg->pdata->pmic_id_irq))
			motg->caps = ALLOW_PHY_POWER_COLLAPSE |
				ALLOW_PHY_RETENTION;

		if (motg->pdata->otg_control == OTG_PHY_CONTROL)
			motg->caps = ALLOW_PHY_RETENTION;
	}
#endif /* CONFIG_USB_MSM_OTG_SH_CUST_SWIC */
#ifdef CONFIG_USB_MSM_OTG_SH_CUST_EYEPATTERN
	motg->ulpi_write_modify = ulpi_val_override;
#endif /* CONFIG_USB_MSM_OTG_SH_CUST_EYEPATTERN */

	if (motg->pdata->enable_lpm_on_dev_suspend)
		motg->caps |= ALLOW_LPM_ON_DEV_SUSPEND;

	wake_lock(&motg->wlock);
	pm_runtime_set_active(&pdev->dev);
	pm_runtime_enable(&pdev->dev);

	if (motg->pdata->bus_scale_table) {
		motg->bus_perf_client =
		    msm_bus_scale_register_client(motg->pdata->bus_scale_table);
		if (!motg->bus_perf_client)
			dev_err(motg->phy.dev, "%s: Failed to register BUS "
						"scaling client!!\n", __func__);
		else
			debug_bus_voting_enabled = true;
	}

	return 0;

remove_phy:
#ifndef CONFIG_USB_MSM_OTG_SH_CUST_SWIC
	usb_set_transceiver(NULL);
#endif /* CONFIG_USB_MSM_OTG_SH_CUST_SWIC */
#ifdef CONFIG_USB_MSM_OTG_SH_CUST
	msm_otg_debugfs_cleanup();
#endif /* CONFIG_USB_MSM_OTG_SH_CUST */
free_irq:
#ifdef CONFIG_USB_SWIC
	swic_ret = shswic_detect_cb_regist(SHSWIC_VBUS_DEVICE, 
					0, (void*)NULL, NULL);
#endif /* CONFIG_USB_SWIC */
#ifndef CONFIG_USB_MSM_OTG_SH_CUST_SWIC
	free_irq(motg->irq, motg);
#endif /* CONFIG_USB_MSM_OTG_SH_CUST_SWIC */
destroy_wlock:
	wake_lock_destroy(&motg->wlock);
	clk_disable_unprepare(motg->core_clk);
	msm_hsusb_ldo_enable(motg, 0);
free_ldo_init:
	msm_hsusb_ldo_init(motg, 0);
free_hsusb_vddcx:
	regulator_disable(hsusb_vddcx);
free_config_vddcx:
	regulator_set_voltage(hsusb_vddcx,
		vdd_val[motg->vdd_type][VDD_NONE],
		vdd_val[motg->vdd_type][VDD_MAX]);
devote_xo_handle:
	clk_disable_unprepare(motg->pclk);
	msm_xo_mode_vote(motg->xo_handle, MSM_XO_MODE_OFF);
free_xo_handle:
	msm_xo_put(motg->xo_handle);
free_regs:
	iounmap(motg->regs);
put_pclk:
	clk_put(motg->pclk);
put_core_clk:
	clk_put(motg->core_clk);
put_clk:
	if (!IS_ERR(motg->clk))
		clk_put(motg->clk);
	if (!IS_ERR(motg->phy_reset_clk))
		clk_put(motg->phy_reset_clk);
free_otg:
	kfree(motg->phy.otg);
free_motg:
	kfree(motg);
	return ret;
}

static int __devexit msm_otg_remove(struct platform_device *pdev)
{
	struct msm_otg *motg = platform_get_drvdata(pdev);
	struct usb_otg *otg = motg->phy.otg;
	int cnt = 0;

	if (otg->host || otg->gadget)
		return -EBUSY;

	if (pdev->dev.of_node)
		msm_otg_setup_devices(pdev, motg->pdata->mode, false);
#ifndef CONFIG_USB_MSM_OTG_SH_CUST_SWIC
	if (motg->pdata->otg_control == OTG_PMIC_CONTROL)
		pm8921_charger_unregister_vbus_sn(0);
#endif /* CONFIG_USB_MSM_OTG_SH_CUST_SWIC */
	msm_otg_debugfs_cleanup();
#ifdef CONFIG_USB_SH_CUST_NON_STANDARD_CHARGE
	msm_otg_stop_charge_timer();
#endif /* CONFIG_USB_SH_CUST_NON_STANDARD_CHARGE */
#ifdef CONFIG_USB_MSM_OTG_SH_CUST
	cancel_delayed_work_sync(&sh_usb_wait_work);
	device_remove_file(&pdev->dev, &dev_attr_usb_cable_type);
	device_remove_file(&pdev->dev, &dev_attr_usb_force_disconnect);
#endif /* CONFIG_USB_MSM_OTG_SH_CUST */

	cancel_delayed_work_sync(&motg->chg_work);
	cancel_delayed_work_sync(&motg->pmic_id_status_work);
	cancel_work_sync(&motg->sm_work);

	pm_runtime_resume(&pdev->dev);

	device_init_wakeup(&pdev->dev, 0);
	pm_runtime_disable(&pdev->dev);
	wake_lock_destroy(&motg->wlock);

#ifdef CONFIG_USB_MSM_OTG_SH_CUST_MHL
	msm_otg_mhl_callback_setting(false);
	msm_otg_mhl_power( motg, false );
#endif /* CONFIG_USB_MSM_OTG_SH_CUST_MHL */

	msm_hsusb_mhl_switch_enable(motg, 0);
#ifndef CONFIG_USB_MSM_OTG_SH_CUST_SWIC
	if (motg->pdata->pmic_id_irq)
		free_irq(motg->pdata->pmic_id_irq, motg);
#endif /* CONFIG_USB_MSM_OTG_SH_CUST_SWIC */
	usb_set_transceiver(NULL);
	free_irq(motg->irq, motg);

	if (motg->pdata->otg_control == OTG_PHY_CONTROL &&
		motg->pdata->mpm_otgsessvld_int)
		msm_mpm_enable_pin(motg->pdata->mpm_otgsessvld_int, 0);

	/*
	 * Put PHY in low power mode.
	 */
	ulpi_read(otg->phy, 0x14);
	ulpi_write(otg->phy, 0x08, 0x09);

	writel(readl(USB_PORTSC) | PORTSC_PHCD, USB_PORTSC);
	while (cnt < PHY_SUSPEND_TIMEOUT_USEC) {
		if (readl(USB_PORTSC) & PORTSC_PHCD)
			break;
		udelay(1);
		cnt++;
	}
	if (cnt >= PHY_SUSPEND_TIMEOUT_USEC)
		dev_err(otg->phy->dev, "Unable to suspend PHY\n");

	clk_disable_unprepare(motg->pclk);
	clk_disable_unprepare(motg->core_clk);
	msm_xo_put(motg->xo_handle);
	msm_hsusb_ldo_enable(motg, 0);
	msm_hsusb_ldo_init(motg, 0);
	regulator_disable(hsusb_vddcx);
	regulator_set_voltage(hsusb_vddcx,
		vdd_val[motg->vdd_type][VDD_NONE],
		vdd_val[motg->vdd_type][VDD_MAX]);

	iounmap(motg->regs);
	pm_runtime_set_suspended(&pdev->dev);

	if (!IS_ERR(motg->phy_reset_clk))
		clk_put(motg->phy_reset_clk);
	clk_put(motg->pclk);
	if (!IS_ERR(motg->clk))
		clk_put(motg->clk);
	clk_put(motg->core_clk);

	if (motg->bus_perf_client)
		msm_bus_scale_unregister_client(motg->bus_perf_client);

	kfree(motg->phy.otg);
	kfree(motg);
	return 0;
}

#ifdef CONFIG_PM_RUNTIME
static int msm_otg_runtime_idle(struct device *dev)
{
	struct msm_otg *motg = dev_get_drvdata(dev);
	struct usb_phy *phy = &motg->phy;

	dev_dbg(dev, "OTG runtime idle\n");

	if (phy->state == OTG_STATE_UNDEFINED)
		return -EAGAIN;
	else
		return 0;
}

static int msm_otg_runtime_suspend(struct device *dev)
{
	struct msm_otg *motg = dev_get_drvdata(dev);

	dev_dbg(dev, "OTG runtime suspend\n");
	return msm_otg_suspend(motg);
}

static int msm_otg_runtime_resume(struct device *dev)
{
	struct msm_otg *motg = dev_get_drvdata(dev);

	dev_dbg(dev, "OTG runtime resume\n");
	pm_runtime_get_noresume(dev);
	return msm_otg_resume(motg);
}
#endif

#ifdef CONFIG_PM_SLEEP
static int msm_otg_pm_suspend(struct device *dev)
{
	int ret = 0;
	struct msm_otg *motg = dev_get_drvdata(dev);

	dev_dbg(dev, "OTG PM suspend\n");

	atomic_set(&motg->pm_suspended, 1);
	ret = msm_otg_suspend(motg);
	if (ret)
		atomic_set(&motg->pm_suspended, 0);

	return ret;
}

static int msm_otg_pm_resume(struct device *dev)
{
	int ret = 0;
	struct msm_otg *motg = dev_get_drvdata(dev);

	dev_dbg(dev, "OTG PM resume\n");

	atomic_set(&motg->pm_suspended, 0);
	if (motg->async_int || motg->sm_work_pending) {
		pm_runtime_get_noresume(dev);
		ret = msm_otg_resume(motg);

		/* Update runtime PM status */
		pm_runtime_disable(dev);
		pm_runtime_set_active(dev);
		pm_runtime_enable(dev);

		if (motg->sm_work_pending) {
			motg->sm_work_pending = false;
			queue_work(system_nrt_wq, &motg->sm_work);
		}
	}

	return ret;
}
#endif

#ifdef CONFIG_PM
static const struct dev_pm_ops msm_otg_dev_pm_ops = {
	SET_SYSTEM_SLEEP_PM_OPS(msm_otg_pm_suspend, msm_otg_pm_resume)
	SET_RUNTIME_PM_OPS(msm_otg_runtime_suspend, msm_otg_runtime_resume,
				msm_otg_runtime_idle)
};
#endif

static struct of_device_id msm_otg_dt_match[] = {
	{	.compatible = "qcom,hsusb-otg",
	},
	{}
};

static struct platform_driver msm_otg_driver = {
	.remove = __devexit_p(msm_otg_remove),
	.driver = {
		.name = DRIVER_NAME,
		.owner = THIS_MODULE,
#ifdef CONFIG_PM
		.pm = &msm_otg_dev_pm_ops,
#endif
		.of_match_table = msm_otg_dt_match,
	},
};

static int __init msm_otg_init(void)
{
	return platform_driver_probe(&msm_otg_driver, msm_otg_probe);
}

static void __exit msm_otg_exit(void)
{
	platform_driver_unregister(&msm_otg_driver);
}

module_init(msm_otg_init);
module_exit(msm_otg_exit);

MODULE_LICENSE("GPL v2");
MODULE_DESCRIPTION("MSM USB transceiver driver");
