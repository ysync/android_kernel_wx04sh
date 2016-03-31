/* drivers/sharp/shspamp/shspamp_yda168b.c
 *
 * Copyright (C) 2011 SHARP CORPORATION
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
/* CONFIG_SH_AUDIO_DRIVER newly created */

#include <linux/module.h>
#include <linux/miscdevice.h>
#include <linux/gpio.h>
#include <linux/mfd/pm8xxx/pm8921.h>
#include <sharp/shspamp.h>
#include <mach/irqs.h>
#include <linux/delay.h>

/*SH_AUDIO_DRIVER->*//*02-150*/
#include <sharp/sh_boot_manager.h>
/*SH_AUDIO_DRIVER<-*//*02-150*/


static int is_on;
static DEFINE_MUTEX(spk_amp_lock);
static int shspamp_opened;

/*SH_AUDIO_DRIVER->*//*02-150*/
static int shspamp_sppd_n;
/*SH_AUDIO_DRIVER<-*//*02-150*/

#define PM8921_GPIO_BASE        NR_GPIO_IRQS
#define PM8921_GPIO_PM_TO_SYS(pm_gpio)  (pm_gpio - 1 + PM8921_GPIO_BASE)

/*SH_AUDIO_DRIVER->*//*02-150*/
#define SHSPAMP_PM8921_GPIO_SPPD_N          35
#define SHSPAMP_PM8921_GPIO_SPPD_N_PRO      31
/*SH_AUDIO_DRIVER<-*//*02-150*/


static int shspamp_open(struct inode *inode, struct file *file)
{
    int rc = 0;
    pr_debug("{shspamp} %s\n", __func__);

    mutex_lock(&spk_amp_lock);

    if (shspamp_opened) {
        pr_err("%s: busy\n", __func__);
        rc = -EBUSY;
        goto done;
    }

    shspamp_opened = 1;
done:
    mutex_unlock(&spk_amp_lock);
    return rc;
}
static int shspamp_release(struct inode *inode, struct file *file)
{

    pr_debug("{shspamp} %s\n", __func__);

    mutex_lock(&spk_amp_lock);
    shspamp_opened = 0;
    mutex_unlock(&spk_amp_lock);
    return 0;
}
static struct file_operations shspamp_fops = {
    .owner   = THIS_MODULE,
    .open    = shspamp_open,
    .release = shspamp_release,
};
static struct miscdevice shspamp_device = {
    .minor = MISC_DYNAMIC_MINOR,
    .name = "shspamp",
    .fops = &shspamp_fops,
};
void shspamp_set_speaker_amp(int on)
{
    int ret =0;
    struct pm_gpio param = {
        .direction      = PM_GPIO_DIR_OUT,
        .output_buffer  = PM_GPIO_OUT_BUF_CMOS,
        .output_value   = 1,
        .pull           = PM_GPIO_PULL_NO,
        .vin_sel        = PM_GPIO_VIN_S4,
        .out_strength   = PM_GPIO_STRENGTH_MED,
        .function       = PM_GPIO_FUNC_NORMAL,
    };

    pr_debug("{shspamp} %s \n", __func__);

    mutex_lock(&spk_amp_lock);
    if (on && !is_on) {
        ret = gpio_request(PM8921_GPIO_PM_TO_SYS(shspamp_sppd_n), "shspamp"); /*02-150*/
        if (ret) {
            pr_err("%s: Failed to request gpio %d\n", __func__,
                    PM8921_GPIO_PM_TO_SYS(shspamp_sppd_n)); /*02-150*/
            goto err_free_gpio;
        }
        ret = pm8xxx_gpio_config(PM8921_GPIO_PM_TO_SYS(shspamp_sppd_n), &param); /*02-150*/
        if (ret){
            pr_err("%s: Failed to configure gpio %d\n", __func__,
                    PM8921_GPIO_PM_TO_SYS(shspamp_sppd_n)); /*02-150*/
            goto err_free_gpio;
        }
        else{
            gpio_direction_output(PM8921_GPIO_PM_TO_SYS(shspamp_sppd_n), 1); /*02-150*/
        }
        is_on = 1;
        pr_debug("%s: ON\n", __func__);
    } else if (!on && is_on) {
        is_on = 0;
        ret = gpio_request(PM8921_GPIO_PM_TO_SYS(shspamp_sppd_n), "shspamp"); /*02-150*/
        if (ret) {
            pr_err("%s: Failed to request gpio %d\n", __func__,
                PM8921_GPIO_PM_TO_SYS(shspamp_sppd_n)); /*02-150*/
            goto err_free_gpio;
        }
        ret = pm8xxx_gpio_config(PM8921_GPIO_PM_TO_SYS(shspamp_sppd_n), &param); /*02-150*/
        if (ret){
            pr_err("%s: Failed to configure gpio %d\n", __func__,
                PM8921_GPIO_PM_TO_SYS(shspamp_sppd_n)); /*02-150*/
            goto err_free_gpio;
        }
        else{
            gpio_direction_output(PM8921_GPIO_PM_TO_SYS(shspamp_sppd_n), 0); /*02-150*/
        }
        msleep(20);
        pr_debug("%s: OFF\n", __func__);
    }

err_free_gpio:
    mutex_unlock(&spk_amp_lock);
    gpio_free(PM8921_GPIO_PM_TO_SYS(shspamp_sppd_n)); /*02-150*/
    return;
}

static int __init shspamp_init(void)
{
	int ret = 0;
    pr_debug("{shspamp} %s\n", __func__);

/*SH_AUDIO_DRIVER->*//*02-150*/
    shspamp_sppd_n = SHSPAMP_PM8921_GPIO_SPPD_N;

#if (CONFIG_SH_AUDIO_DRIVER_MODEL_NUMBER == 303)
    if (sh_boot_get_hw_revision() == 7) {
        shspamp_sppd_n = SHSPAMP_PM8921_GPIO_SPPD_N_PRO;
    } 
#endif
/*SH_AUDIO_DRIVER<-*//*02-150*/

    ret = misc_register(&shspamp_device);
    if (ret) {
        pr_err("%s: shspamp_device register failed\n", __func__);
    }

    return 0;
}

module_init(shspamp_init);

MODULE_DESCRIPTION("shspamp speaker amp driver");
MODULE_LICENSE("GPL");
