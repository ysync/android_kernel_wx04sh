/* drivers/sharp/shreceiver/shreceiver_lm48560.c
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
#include <linux/init.h>
#include <linux/device.h>
#include <linux/fs.h>
#include <linux/i2c.h>
#include <linux/miscdevice.h>
#include <linux/gpio.h>
#include <asm/uaccess.h>
#include <linux/delay.h>
#include <linux/mutex.h>
#include <linux/slab.h>
#include <linux/mfd/pm8xxx/pm8921.h>
#include <sharp/shspamp.h>

static struct i2c_client *this_client;
static struct shreceiver_platform_data *pdata;

static DEFINE_MUTEX(receiver_amp_lock);
static int shreceiver_opened;

static int shreceiver_i2c_write(int reg, u8 data)
{
    int rc;
    u8 buf[2];
    struct i2c_msg msg[] = {
    {
        .addr = this_client->addr,
        .flags= 0,
        .len  = 2,
        .buf  = buf,
        },
    };

    buf[0] = reg;
    buf[1] = data;
    rc = i2c_transfer(this_client->adapter, msg, 1);
    if(rc != 1){
        dev_err(&this_client->dev,
                "shreceiver_i2c_write FAILED: writing to reg %d\n", reg);
        rc = -1;
    }
    return rc;
}

static int shreceiver_open(struct inode *inode, struct file *file)
{
    int rc = 0;
    pr_debug("{shreceiver} %s\n", __func__);

    mutex_lock(&receiver_amp_lock);

    if (shreceiver_opened) {
        pr_err("%s: busy\n", __func__);
        rc = -EBUSY;
        goto done;
    }

    shreceiver_opened = 1;
done:
    mutex_unlock(&receiver_amp_lock);
    return rc;
}
static int shreceiver_release(struct inode *inode, struct file *file)
{

    pr_debug("{shreceiver} %s\n", __func__);

    mutex_lock(&receiver_amp_lock);
    shreceiver_opened = 0;
    mutex_unlock(&receiver_amp_lock);
    return 0;
}
static struct file_operations shreceiver_fops = {
    .owner   = THIS_MODULE,
    .open    = shreceiver_open,
    .release = shreceiver_release,
};
static struct miscdevice shreceiver_device = {
    .minor = MISC_DYNAMIC_MINOR,
    .name = "shreceiver",
    .fops = &shreceiver_fops,
};

void shreceiver_set_receiver(int on){
    static int is_on;
    int ret = 0;
    struct pm_gpio param = {
        .direction      = PM_GPIO_DIR_OUT,
        .output_buffer  = PM_GPIO_OUT_BUF_CMOS,
        .output_value   = 1,
        .pull           = PM_GPIO_PULL_NO,
        .vin_sel        = PM_GPIO_VIN_S4,
        .out_strength   = PM_GPIO_STRENGTH_MED,
        .function       = PM_GPIO_FUNC_NORMAL,
    };

    pr_debug("%s: \n", __func__);

    if (!pdata) {
        pr_err("%s: no platform data!\n", __func__);
        return;
    }
    mutex_lock(&receiver_amp_lock);
    if (on && !is_on) {
        msleep(30);
        ret = gpio_request(pdata->gpio_shreceiver_receiver_en, "shreceiver");
        if (ret) {
            pr_err("%s: Failed to request gpio %d\n", __func__,
                    pdata->gpio_shreceiver_receiver_en);
            goto err_free_gpio;
        }
        ret = pm8xxx_gpio_config(pdata->gpio_shreceiver_receiver_en, &param);
        if (ret){
            pr_err("%s: Failed to configure gpio %d\n", __func__,
                    pdata->gpio_shreceiver_receiver_en);
            goto err_free_gpio;
        }
        else{
            gpio_direction_output(pdata->gpio_shreceiver_receiver_en, 1);
        }
        shreceiver_i2c_write(0x00, 0x07);   /* [TURN_ON] Normal turn on time, tWU = 15ms */
                                            /* [IN_SEL]  Input 2 selected */
                                            /* [BOOST_EN]Boost enabled */
                                            /* [SHDN] Device enabled */
        shreceiver_i2c_write(0x01, 0x06);   /* [PLEV2 (B2) PLEV1 (B1)PLEV0 (B0)] VTH(VLIM) = 28VP-P */
        shreceiver_i2c_write(0x02, 0x01);   /* [GAIN1(B1) GAIN0 (B0)] 24dB */
        
        is_on = 1;
        
        pr_debug("%s: ON\n", __func__);
    } else if (!on && is_on){
        if (is_on) {
            shreceiver_i2c_write(0x00, 0x00);   /* [SHDN] Device shutdown */
            ret = gpio_request(pdata->gpio_shreceiver_receiver_en, "shreceiver");
            if (ret) {
                pr_err("%s: Failed to request gpio %d\n", __func__,
                        pdata->gpio_shreceiver_receiver_en);
                goto err_free_gpio;
            }
            ret = pm8xxx_gpio_config(pdata->gpio_shreceiver_receiver_en, &param);
            if (ret){
                pr_err("%s: Failed to configure gpio %d\n", __func__,
                        pdata->gpio_shreceiver_receiver_en);
                goto err_free_gpio;
            }
            else{
                gpio_direction_output(pdata->gpio_shreceiver_receiver_en, 0);
            }
            is_on = 0;
            msleep(10);
            pr_debug("%s: OFF\n", __func__);
        }
    }

err_free_gpio:
    mutex_unlock(&receiver_amp_lock);
    gpio_free(pdata->gpio_shreceiver_receiver_en);
    return ;
}

static int shreceiver_probe(struct i2c_client *client, const struct i2c_device_id *id)
{
    int ret = 0;
    pr_debug("{shreceiver} %s\n", __func__);
    pdata = client->dev.platform_data;

    if (!pdata) {
        ret = -EINVAL;
        pr_err("%s: platform data is NULL\n", __func__);
        goto err_no_pdata;
    }

    this_client = client;

    ret = misc_register(&shreceiver_device);
    if (ret) {
        pr_err("%s: shreceiver_device register failed\n", __func__);
    }

    return 0;

err_no_pdata:
    return ret;
}

static int shreceiver_remove(struct i2c_client *client)
{
    this_client = i2c_get_clientdata(client);
    return 0;
}

static const struct i2c_device_id shreceiver_id[] = {
    { SHRECEIVER_I2C_NAME, 0 },
    { }
};

static struct i2c_driver shreceiver_driver = {
    .driver = {
        .owner = THIS_MODULE,
        .name  = SHRECEIVER_I2C_NAME,
    },
    .class    = I2C_CLASS_HWMON,
    .probe    = shreceiver_probe,
    .id_table = shreceiver_id,
    .remove   = shreceiver_remove,
};

static int __init shreceiver_init(void)
{
    pr_debug("{shreceiver} %s\n", __func__);
    return i2c_add_driver(&shreceiver_driver);
}

module_init(shreceiver_init);

MODULE_DESCRIPTION("shreceiver driver");
MODULE_LICENSE("GPL");
