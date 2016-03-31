/* drivers/sharp/shuim/shuim.c
 *
 * Copyright (C) 2012 SHARP CORPORATION
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

#include <linux/kernel.h>
#include <linux/init.h>
#include <linux/mutex.h>
#include <linux/input.h>
#include <linux/cdev.h>
#include <linux/sched.h>
#include <linux/types.h>
#include <linux/fs.h>
#include <linux/unistd.h>
#include <linux/slab.h>
#include <linux/module.h>
#include <linux/irq.h>
#include <linux/delay.h>
#include <linux/major.h>
#include <linux/gpio.h>
#include <linux/device.h>
#include <linux/interrupt.h>
#include <linux/uaccess.h>

#include "shuim.h"

#define D_SHUIM_GPIO_NO 			(58)
#define D_SHUIM_DEV_LOW 			(0)
#define D_SHUIM_DEV_HIGH 			(1)
#define D_SHUIM_DEV_NAME 			("shuimif")

#define D_SHUIM_POLL_SLEEP_NUM		(10)
#define D_SHUIM_POLL_SLEEP_USEC		(10000)

#define D_SHUIM_UIM_STATE_NORMAL	(0)
#define D_SHUIM_UIM_STATE_HOTSWAP	(1)

#define D_UIM_DEVS					(1)
#define D_UIM_GET_HOTSWAP_INFO		_IOR ( 0xE0, 0, int )

/*
 * prototype
 */
static __init int shuim_init(void);
static void __exit shuim_exit(void);

/*
 * global variable
 */
static int		g_shuim_hotswap	= D_SHUIM_UIM_STATE_NORMAL;
static struct class	*shuim_class		= NULL;
static struct		cdev uim_cdev;

static struct poll_data g_shuim_data;
static struct poll_data *g_shuim_d = &g_shuim_data;

static irqreturn_t shuim_irq_handler(int irq, void *dev_id)
{
	SHUIM_DRV_DBG_LOG("START irq = %d", irq);
	
	disable_irq_nosync(gpio_to_irq(D_SHUIM_GPIO_NO));
	
	g_shuim_hotswap = D_SHUIM_UIM_STATE_HOTSWAP;

	SHUIM_DRV_DBG_LOG("END");
	
	return IRQ_HANDLED;
}

static int shuim_ioctl_get_gpio_info(void)
{
	struct poll_data *shuim_d = g_shuim_d;
	int i, ret = 0, read_value = 0, old_value = 0;
	unsigned long irqflag = 0;

	SHUIM_DRV_DBG_LOG("START");

	old_value = shuim_d->device_status;

	/* anti-chattering */
	for (i = 0; i < D_SHUIM_POLL_SLEEP_NUM; i++) {
		read_value = gpio_get_value(D_SHUIM_GPIO_NO);
		if ((read_value < 0) || (read_value == old_value)) {
			break;
		}
		usleep(D_SHUIM_POLL_SLEEP_USEC);
	}

	SHUIM_DRV_DBG_LOG("read_value = %d, old_value = %d", read_value, old_value);
	
	if (read_value < 0) {
		/* read error */
		shuim_d->read_error = read_value;
		ret = -EFAULT;
	} else if (read_value == old_value) {
		/* anti-chattering */
		irqflag = (read_value == D_SHUIM_DEV_LOW) ? IRQF_TRIGGER_HIGH : IRQF_TRIGGER_LOW;
		irqflag |= IRQF_SHARED;

		if (irq_set_irq_type(gpio_to_irq(D_SHUIM_GPIO_NO), irqflag)) {
			SHUIM_DRV_ERR_LOG("set_irq_type irqflag = %ld", irqflag);
		}
		/* enable irq handler */
		enable_irq(gpio_to_irq(D_SHUIM_GPIO_NO));
		ret = -EFAULT;
	} else {
		/* read changed data */
		shuim_d->device_status = read_value;
		shuim_d->read_error = 0;
	}
	
	SHUIM_DRV_DBG_LOG("END read_value = %d, old_value = %d, shuim_d->read_error = %d",
						read_value, old_value, shuim_d->read_error);

	return ret;
}

static long shuim_ioctl_get_hotswap_info(unsigned long arg)
{
	struct poll_data *shuim_d = g_shuim_d;

	SHUIM_DRV_DBG_LOG("START");

	if(g_shuim_hotswap != D_SHUIM_UIM_STATE_HOTSWAP) {
		SHUIM_DRV_DBG_LOG("uim state normal");
		return 0;
	}

	if(shuim_ioctl_get_gpio_info()) {
		SHUIM_DRV_ERR_LOG("get_gpio_info failure");
		return -EFAULT;
	}

	if(copy_to_user((u8*)arg, (u8*)&(shuim_d->device_status), sizeof(shuim_d->device_status))){
		SHUIM_DRV_ERR_LOG("copy_to_user failure");
	}
	
	SHUIM_DRV_DBG_LOG("END");

	return -EFAULT;
}

static long shuim_ioctl(struct file *filp, unsigned int cmd, unsigned long arg)
{
	int ret = 0;

	SHUIM_DRV_DBG_LOG("START cmd = %d", cmd);

	switch(cmd){
	case D_UIM_GET_HOTSWAP_INFO:
		ret = shuim_ioctl_get_hotswap_info(arg);
		if(ret) {
			SHUIM_DRV_ERR_LOG("get_hotswap_info failure ret = %d", ret);
			return -EFAULT;
		}
		break;
	default:
		SHUIM_DRV_ERR_LOG("cmd = %d", cmd);
		return -ENOIOCTLCMD;
	}
	
	return 0;
}

static int shuim_open(struct inode *inode, struct file *file)
{
	SHUIM_DRV_DBG_LOG("START");
	SHUIM_DRV_DBG_LOG("END");
	return 0;
}

static int shuim_release(struct inode *inode, struct file *file)
{
	SHUIM_DRV_DBG_LOG("START");
	SHUIM_DRV_DBG_LOG("END");
	return 0;
}

static const struct file_operations shuim_fileops = {
	.owner          = THIS_MODULE,
	.open           = shuim_open,
	.release        = shuim_release,
	.unlocked_ioctl = shuim_ioctl,
};

/*
 * shuim_init
 */
static __init int shuim_init(void)
{
	struct poll_data *shuim_d = g_shuim_d;
	int ret;
	struct device *class_dev;
	unsigned long irqflag = 0;
	dev_t dev = MKDEV(MISC_MAJOR, 0);

	SHUIM_DRV_DBG_LOG("START");
	
	ret = alloc_chrdev_region(&dev, 0, D_UIM_DEVS, D_SHUIM_DEV_NAME);
	if(ret < 0){
		SHUIM_DRV_ERR_LOG("alloc_chrdev_region error\n");
		return ret;
	}

	shuim_class = class_create(THIS_MODULE, D_SHUIM_DEV_NAME);
	if (IS_ERR(shuim_class)) {
		unregister_chrdev_region(dev, D_UIM_DEVS);
		ret = PTR_ERR(shuim_class);
		SHUIM_DRV_ERR_LOG("shuim_create error\n");
		return ret;
	}

	class_dev = device_create(shuim_class, NULL, dev, NULL, D_SHUIM_DEV_NAME);
	if (IS_ERR(class_dev)) {
		unregister_chrdev_region(dev, D_UIM_DEVS);
		ret = PTR_ERR(class_dev);
		SHUIM_DRV_ERR_LOG("device_create ret = %d", ret);
		return ret;
	}

	cdev_init(&uim_cdev, &shuim_fileops);
	uim_cdev.owner = THIS_MODULE;

	ret = cdev_add(&uim_cdev, dev, D_UIM_DEVS);
	if (ret) {
		unregister_chrdev_region(dev, D_UIM_DEVS);
		SHUIM_DRV_ERR_LOG("cdev_add ret = %d", ret);
		return ret;
	}

	/* initialize poll_data */
	memset(g_shuim_d, 0x00, sizeof(struct poll_data));
	
	ret = gpio_get_value(D_SHUIM_GPIO_NO);
	if (ret < 0) {
		SHUIM_DRV_ERR_LOG("gpio_get_value ret = %d", ret);
		return -EIO;
	}

	shuim_d->device_status = ret;

	if (shuim_d->device_status == D_SHUIM_DEV_HIGH) {
		irqflag = IRQF_TRIGGER_LOW | IRQF_SHARED;
	}
	else {
		irqflag = IRQF_TRIGGER_HIGH | IRQF_SHARED;
	}

	if (request_irq(gpio_to_irq(D_SHUIM_GPIO_NO),
					shuim_irq_handler,
					irqflag,
					D_SHUIM_DEV_NAME,
					(void*)shuim_d)) {
		SHUIM_DRV_ERR_LOG("request_irq irqflag = %ld", irqflag);
		return -EIO;
	}
		
	if(enable_irq_wake(gpio_to_irq(D_SHUIM_GPIO_NO))) {
		SHUIM_DRV_ERR_LOG("enable_irq_wake");
		free_irq(gpio_to_irq(D_SHUIM_GPIO_NO), (void *)shuim_d);
		return -EIO;
	}

	shuim_d->irq_handler_done = 0;
	shuim_d->open_flag = 1;
	
	SHUIM_DRV_DBG_LOG("END");

	return 0;
}


/*
 * shuim_exit
 */
static void __exit shuim_exit(void)
{
	struct poll_data *shuim_d = g_shuim_d;

	dev_t dev = MKDEV(MISC_MAJOR, 0);
	
	SHUIM_DRV_DBG_LOG("START");

	cdev_del(&uim_cdev);
	class_destroy( shuim_class );
	unregister_chrdev_region(dev, D_UIM_DEVS);
	
	/* clear workqueue */
	if(shuim_d->open_flag) {
		if(disable_irq_wake(gpio_to_irq(D_SHUIM_GPIO_NO)))
			SHUIM_DRV_ERR_LOG("disable_irq_wake");
		
		free_irq(gpio_to_irq(D_SHUIM_GPIO_NO), (void *)shuim_d);
	}
	
	SHUIM_DRV_DBG_LOG("END");
}

MODULE_LICENSE("GPL v2");

module_init(shuim_init);
module_exit(shuim_exit);

