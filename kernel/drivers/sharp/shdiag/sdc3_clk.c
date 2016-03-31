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

#include "../../mmc/host/msm_sdcc.h"

static int sdc3_clk_open(struct inode *inode, struct file *filp)
{
    return 0;
}

static ssize_t sdc3_clk_read(struct file *filp, char __user *buf,size_t count, loff_t *ppos)
{
    return 0;
}

static ssize_t sdc3_clk_write(struct file *filp, const char __user *buf,size_t count, loff_t *ppos)
{
    int ret;
    bool enable;
    int temp;

    ret = copy_from_user((void *)&temp, buf, sizeof(int));
    if (ret) {
        printk("%s: copy_from_user failed %d\n", __func__, ret);
        return -EFAULT;
    }

    if (temp) {
        enable = true;
    }
    else {
        enable = false;
    }
#if 0
    ret = msmsdcc_force_setup_sdc3_clk(enable);
    if (ret) {
        printk("%s: msmsdcc_force_setup_sdc3_clk failed %d\n", __func__, ret);
        return -EFAULT;
    }
#endif

    return count;
}

static int sdc3_clk_release(struct inode *inode, struct file *filp)
{
    return 0;
}

static struct file_operations sdc3_clk_fops = {
    .owner   = THIS_MODULE,
    .read    = sdc3_clk_read,
    .write   = sdc3_clk_write,
    .open    = sdc3_clk_open,
    .release = sdc3_clk_release,
};

static struct miscdevice sdc3_clk_dev = {
    .minor = MISC_DYNAMIC_MINOR,
    .name = "sdc3_clk",
    .fops = &sdc3_clk_fops,
};

static int __init sdc3_clk_init( void )
{
    int ret;

    ret = misc_register(&sdc3_clk_dev);
    if (ret) {
        printk("%s: misc_register failed %d\n", __func__, ret);
        return ret;
    }

    return 0;
}

module_init(sdc3_clk_init);

MODULE_DESCRIPTION("sdc3_clk");
MODULE_LICENSE("GPL v2");

