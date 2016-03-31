/* drivers/sharp/mfc/mvdd.c (NFC driver)
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

#ifdef CONFIG_SHFELICA

/***************header***************/
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
#include "mfc.h"

#ifdef CONFIG_SHSNFC_BATTERY_FIXATION

#include <sharp/sh_boot_manager.h>

/* for snfc_output_disable */
#include <linux/mfd/pm8xxx/pm8921.h>

#define D_MVDD_DEVS 			(1)
#define D_MVDD_GPIO_NO 			CONFIG_SHSNFC_MVDD_GPIO
#define D_MVDD_DEV_LOW 			(0)
#define D_MVDD_DEV_HIGH 		(1)
#define D_MVDD_DEV_NAME 		("snfc_mvdd")
#define D_MVDD_DELAY_MSEC 		(0)

#define D_MVDD_ENABLE 			0x01
#define D_MVDD_DISABLE 			0x00

#if (CONFIG_SHSNFC_MVDD_GPIO == 58)
#define D_MVDD_ENABLE_SP 		0x02
#endif	//(CONFIG_SHSNFC_MVDD_GPIO == 58).

/* for snfc_output_disable */
#define PM8921_GPIO_BASE				NR_GPIO_IRQS
#define PM8921_GPIO_PM_TO_SYS(pm_gpio)	(pm_gpio - 1 + PM8921_GPIO_BASE)
#define D_HSEL_GPIO_NO					PM8921_GPIO_PM_TO_SYS(14)
#define D_PON_GPIO_NO					(48)
#define D_UART_TX_GPIO_NO				(10)

#if (CONFIG_SHSNFC_MVDD_GPIO == 58)
/* for anti-chattering */
#define D_MVDD_POLL_SLEEP_NUM			(10)
#define D_MVDD_POLL_SLEEP_USEC			(10000)
#endif	//(CONFIG_SHSNFC_MVDD_GPIO == 58).

/*
 * prototype
 */
static __init int mvdd_init(void);
static void __exit mvdd_exit(void);

/*
 * global variable
 */
static struct poll_data g_mvdd_data;
static struct poll_data *g_mvdd_d = &g_mvdd_data;

static unsigned int g_snfc_status = D_MVDD_DISABLE;
#if (CONFIG_SHSNFC_MVDD_GPIO == 58)
static unsigned int g_hw_revision = 0x00;
#endif	//(CONFIG_SHSNFC_MVDD_GPIO == 58).

#endif	//CONFIG_SHSNFC_BATTERY_FIXATION.

unsigned int snfc_available(void)
{
#ifdef CONFIG_SHSNFC_BATTERY_FIXATION

	MFC_DRV_DBG_LOG("g_snfc_status = 0x%02X", g_snfc_status);
	return g_snfc_status;

#else	//CONFIG_SHSNFC_BATTERY_FIXATION.

	return 0x01;

#endif	//CONFIG_SHSNFC_BATTERY_FIXATION.
}


unsigned int snfc_available_wake_up(void)
{
#ifdef CONFIG_SHSNFC_BATTERY_FIXATION

	unsigned int ret;
	ret = (D_MVDD_DEV_HIGH == gpio_get_value(D_MVDD_GPIO_NO)) ? D_MVDD_ENABLE : D_MVDD_DISABLE;

	MFC_DRV_DBG_LOG("gpio_get_value = %d",ret);

#if (CONFIG_SHSNFC_MVDD_GPIO == 58)
	if((0x07 == g_hw_revision) && (g_snfc_status & D_MVDD_ENABLE_SP)) {
		MFC_DRV_DBG_LOG("g_snfc_status = 0x%02X", g_snfc_status);
		ret = D_MVDD_ENABLE;
	}
#endif	//(CONFIG_SHSNFC_MVDD_GPIO == 58).

	return ret;

#else	//CONFIG_SHSNFC_BATTERY_FIXATION.

	return 0x01;

#endif	//CONFIG_SHSNFC_BATTERY_FIXATION.
}

#ifdef CONFIG_SHSNFC_BATTERY_FIXATION
void snfc_output_disable(void)
{
	int ret = 0;
	
	MFC_DRV_DBG_LOG("START");
	
	/* UART */
	ret = gpio_tlmm_config(GPIO_CFG(D_UART_TX_GPIO_NO, 0, GPIO_CFG_OUTPUT, GPIO_CFG_PULL_DOWN, GPIO_CFG_2MA), GPIO_CFG_ENABLE);
	
	/* HSEL */
	gpio_set_value_cansleep(D_HSEL_GPIO_NO, D_MVDD_DEV_LOW);
	
	/* PON */
	gpio_set_value(D_PON_GPIO_NO, D_MVDD_DEV_LOW);

	MFC_DRV_DBG_LOG("END ret = %d", ret);
}

void mvdd_work_func(struct work_struct *work)
{
	struct poll_data *mvdd_d = g_mvdd_d;
	int read_value = 0, old_value = 0;
	
#if (CONFIG_SHSNFC_MVDD_GPIO == 58)
	int i;
	unsigned long irqflag = 0;
#endif	//(CONFIG_SHSNFC_MVDD_GPIO == 58).

	MFC_DRV_DBG_LOG("START");
	
	old_value = mvdd_d->device_status;

#if (CONFIG_SHSNFC_MVDD_GPIO == 58)
	/* anti-chattering */
	for (i = 0; i < D_MVDD_POLL_SLEEP_NUM; i++) {
		read_value = gpio_get_value(D_MVDD_GPIO_NO);
		if ((read_value < 0) || (read_value == old_value)) {
			break;
		}
		usleep(D_MVDD_POLL_SLEEP_USEC);
	}
#else	//(CONFIG_SHSNFC_MVDD_GPIO == 58).
	read_value = gpio_get_value(D_MVDD_GPIO_NO);
#endif	//(CONFIG_SHSNFC_MVDD_GPIO == 58).

	MFC_DRV_DBG_LOG("read_value = %d, old_value = %d, g_snfc_status = 0x%02X", read_value, old_value, g_snfc_status);
	
	/* read error */
	if (read_value < 0) {
		mvdd_d->read_error = read_value;
	/* read changed data */
	} else {
		if((read_value == D_MVDD_DEV_LOW) && (old_value == D_MVDD_DEV_HIGH)) {
			snfc_output_disable();
			mvdd_d->device_status = read_value;
			mvdd_d->read_error = 0;
			g_snfc_status &= D_MVDD_DISABLE;
		}

#if (CONFIG_SHSNFC_MVDD_GPIO == 58)
		/* anti-chattering */
		if(read_value == old_value) {
			irqflag = (read_value == D_MVDD_DEV_LOW) ? IRQF_TRIGGER_HIGH : IRQF_TRIGGER_LOW;
			irqflag |= IRQF_SHARED;
		} else {
			/* check hw revision */
			if(0x07 == g_hw_revision) {
				if(read_value == D_MVDD_DEV_HIGH) {
					irqflag = IRQF_TRIGGER_LOW | IRQF_SHARED;
					g_snfc_status &= ~D_MVDD_ENABLE_SP;
				}
			}
		}
		
		if(irqflag) {
			if (irq_set_irq_type(gpio_to_irq(D_MVDD_GPIO_NO), irqflag))
				MFC_DRV_ERR_LOG("set_irq_type irqflag = %ld", irqflag);
			/* enable irq handler */
			enable_irq(gpio_to_irq(D_MVDD_GPIO_NO));
		}
		
		mvdd_d->device_status = read_value;
		mvdd_d->read_error = 0;

#endif	//(CONFIG_SHSNFC_MVDD_GPIO == 58).
	}
	
	MFC_DRV_DBG_LOG("END read_value = %d, old_value = %d, mvdd_d->read_error = %d, g_snfc_status = 0x%02X"
					, read_value, old_value, mvdd_d->read_error, g_snfc_status);
}


static irqreturn_t mvdd_irq_handler(int irq, void *dev_id)
{	
	MFC_DRV_DBG_LOG("START irq = %d", irq);
	
	disable_irq_nosync(gpio_to_irq(D_MVDD_GPIO_NO));
	/* set workqueue */
	schedule_delayed_work(&g_mvdd_d->work, msecs_to_jiffies(D_MVDD_DELAY_MSEC));
	
	MFC_DRV_DBG_LOG("END");
	
	return IRQ_HANDLED;
}

/*
 * mvdd_init
 */
static __init int mvdd_init(void)
{
	int ret;
	struct poll_data *mvdd_d = g_mvdd_d;
	unsigned long irqflag = 0;
	
	MFC_DRV_DBG_LOG("START");
	
	/* initialize poll_data */
	memset(g_mvdd_d, 0x00, sizeof(struct poll_data));
	/* initialize workqueue */
	INIT_DELAYED_WORK(&g_mvdd_d->work, mvdd_work_func);
	/* initialize waitqueue */
	init_waitqueue_head(&g_mvdd_d->read_wait);
	
	ret = gpio_get_value(D_MVDD_GPIO_NO);
	if (ret < 0) {
		MFC_DRV_ERR_LOG("gpio_get_value ret = %d", ret);
		return -EIO;
	}

	mvdd_d->device_status = ret;

#if (CONFIG_SHSNFC_MVDD_GPIO == 58)
	g_hw_revision = sh_boot_get_hw_revision();

	/* check hw revision */
	if(0x07 == g_hw_revision) {
		MFC_DRV_DBG_LOG("hw_revision = 0x07");
	
		if(mvdd_d->device_status == D_MVDD_DEV_LOW) {
			irqflag = IRQF_TRIGGER_HIGH | IRQF_SHARED;
			g_snfc_status = (D_MVDD_ENABLE | D_MVDD_ENABLE_SP);
		}
	}
#endif	//(CONFIG_SHSNFC_MVDD_GPIO == 58).

	if (mvdd_d->device_status == D_MVDD_DEV_HIGH) {
		irqflag = IRQF_TRIGGER_LOW | IRQF_SHARED;
		g_snfc_status = D_MVDD_ENABLE;

#if (CONFIG_SHSNFC_MVDD_GPIO == 78)
		if(0x06 == sh_boot_get_hw_revision()) {
			if((gpio_get_value(49) == 0) && (gpio_get_value(46) == 0)) {
				MFC_DRV_ERR_LOG("NFC DISABLE");
				g_snfc_status = D_MVDD_DISABLE;
				mvdd_d->device_status = D_MVDD_DEV_LOW;
				irqflag = 0;
			}
		}
#endif

	}

	if(irqflag) {
		if (request_irq(gpio_to_irq(D_MVDD_GPIO_NO),
		                mvdd_irq_handler,
		                irqflag,
		                D_MVDD_DEV_NAME,
		                (void*)mvdd_d)) {

			MFC_DRV_ERR_LOG("request_irq irqflag = %ld", irqflag);
			return -EIO;
		}
		
		if(enable_irq_wake(gpio_to_irq(D_MVDD_GPIO_NO))) {
			MFC_DRV_ERR_LOG("enable_irq_wake");
			free_irq(gpio_to_irq(D_MVDD_GPIO_NO), (void *)mvdd_d);
			return -EIO;
		}
		mvdd_d->irq_handler_done = 0;
		mvdd_d->open_flag = 1;
	}
	
	MFC_DRV_DBG_LOG("END");
	
	return 0;
}


/*
 * mvdd_exit
 */
static void __exit mvdd_exit(void)
{
	struct poll_data *mvdd_d = g_mvdd_d;
	
	MFC_DRV_DBG_LOG("START");
	
	/* clear workqueue */
	cancel_delayed_work(&mvdd_d->work);
	
	if(mvdd_d->open_flag) {
		if(disable_irq_wake(gpio_to_irq(D_MVDD_GPIO_NO)))
			MFC_DRV_ERR_LOG("disable_irq_wake");
		
		free_irq(gpio_to_irq(D_MVDD_GPIO_NO), (void *)mvdd_d);
	}
	
	MFC_DRV_DBG_LOG("END");
}

MODULE_LICENSE("GPL v2");

module_init(mvdd_init);
module_exit(mvdd_exit);

#endif	//CONFIG_SHSNFC_BATTERY_FIXATION.

#endif	//CONFIG_SHFELICA

