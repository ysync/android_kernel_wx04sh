/* drivers/sharp/shdiag/sdc3_clk.c
 *
 * Copyright (C) 2012 Sharp Corporation
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

#include <linux/module.h>
#include <linux/miscdevice.h>
#include <linux/fs.h>
#include <linux/poll.h>

#include <../../../arch/arm/mach-msm/include/mach/peripheral-loader.h>


static int load_image_modem_open(struct inode *inode, struct file *filp)
{
    return 0;
}

static ssize_t load_image_modem_read(struct file *filp, char __user *buf,size_t count, loff_t *ppos)
{
    void *pil = NULL;

    printk("%s start pil(%p)\n", __func__, pil);
    pil = pil_get("modem");
    printk("%s end pil(%p)\n", __func__, pil);

    return 0;
}

static ssize_t load_image_modem_write(struct file *filp, const char __user *buf,size_t count, loff_t *ppos)
{
    return count;
}

static int load_image_modem_release(struct inode *inode, struct file *filp)
{
    return 0;
}

static struct file_operations load_image_modem_fops = {
    .owner   = THIS_MODULE,
    .read    = load_image_modem_read,
    .write   = load_image_modem_write,
    .open    = load_image_modem_open,
    .release = load_image_modem_release,
};

static struct miscdevice load_image_modem_dev = {
    .minor = MISC_DYNAMIC_MINOR,
    .name = "load_image_modem",
    .fops = &load_image_modem_fops,
};

static int __init load_image_modem_init( void )
{
    int ret;

    ret = misc_register(&load_image_modem_dev);
    if (ret) {
        printk("%s: misc_register failed %d\n", __func__, ret);
        return ret;
    }

    return 0;
}

module_init(load_image_modem_init);

MODULE_DESCRIPTION("load_image_modem");
MODULE_LICENSE("GPL v2");

