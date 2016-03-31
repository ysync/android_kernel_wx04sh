/* drivers/sharp/shcamled/shcamled.c
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

/* ------------------------------------------------------------------------- */
/* INCLUDE FILES                                                             */
/* ------------------------------------------------------------------------- */
#include <linux/module.h>
#include <linux/platform_device.h>
#include <linux/device.h>
#include <linux/ctype.h>
#include <linux/leds-pm8xxx.h>
#include "linux/leds.h"
#if defined(CONFIG_MACH_DECKARD_AF33) || defined(CONFIG_MACH_DECKARD_AS38)
#include <sharp/tps61029.h>
#endif

/* ------------------------------------------------------------------------- */
/* PROTOTYPES                                                                */
/* ------------------------------------------------------------------------- */
int shcamled_pmic_set_torch_led_1_current(unsigned mA);
static int shcamled_pmic_set_torch_led_2_current(unsigned mA);
static ssize_t shcamled_torch_store(struct device *dev, struct device_attribute *attr, const char *buf, size_t size);
static int shcamled_torch_probe(struct platform_device *pdev);
static int shcamled_torch_remove(struct platform_device *pdev);

static int shcamled_pmic_set_red_led_current(unsigned mA);
static ssize_t shcamled_red_store(struct device *dev, struct device_attribute *attr, const char *buf, size_t size);
static int shcamled_red_probe(struct platform_device *pdev);
static int shcamled_red_remove(struct platform_device *pdev);

/* ------------------------------------------------------------------------- */
/* MACROS                                                                    */
/* ------------------------------------------------------------------------- */
#define FALSE 0
#define TRUE 1

#define SHCAMLED_TORCH_DRV_NAME	"shcamled_torch"
#define SHCAMLED_TORCH_PERMISSION	(S_IRUSR | S_IWUSR) | (S_IRGRP | S_IWGRP)

#define SHCAMLED_RED_DRV_NAME	"shcamled_red"
#define SHCAMLED_RED_PERMISSION	(S_IRUSR | S_IWUSR) | (S_IRGRP | S_IWGRP)

//#define SHCAMLED_ENABLE_DEBUG

#ifdef SHCAMLED_ENABLE_DEBUG
#define SHCAMLED_INFO(x...)	printk(KERN_INFO "[SHCAMLED] " x)
#define SHCAMLED_TRACE(x...)	printk(KERN_DEBUG "[SHCAMLED] " x)
#define SHCAMLED_ERROR(x...)	printk(KERN_ERR "[SHCAMLED] " x)
#else
#define SHCAMLED_INFO(x...)	do {} while(0)
#define SHCAMLED_TRACE(x...)	do {} while(0)
#define SHCAMLED_ERROR(x...)	printk(KERN_ERR "[SHCAMLED] " x)
#endif

/* ------------------------------------------------------------------------- */
/* VARIABLES                                                                 */
/* ------------------------------------------------------------------------- */
static struct platform_device shcamled_torch_dev = {
	.name   = "shcamled_torch",
};
static struct platform_device shcamled_red_dev = {
	.name   = "shcamled_red",
};

static struct platform_driver shcamled_torch_driver = {
	.probe = shcamled_torch_probe,
	.remove = shcamled_torch_remove,
	.shutdown = NULL,
	.driver = {
		.name = SHCAMLED_TORCH_DRV_NAME,
		.owner = THIS_MODULE,
	},
};

static struct platform_driver shcamled_red_driver = {
	.probe = shcamled_red_probe,
	.remove = shcamled_red_remove,
	.shutdown = NULL,
	.driver = {
		.name = SHCAMLED_RED_DRV_NAME,
		.owner = THIS_MODULE,
	},
};

static DEVICE_ATTR(shcamled_torch, SHCAMLED_TORCH_PERMISSION, NULL, shcamled_torch_store);
static DEVICE_ATTR(shcamled_red, SHCAMLED_RED_PERMISSION, NULL, shcamled_red_store);

/* LED trigger (declare cam_led_x_trigger) */
#if defined(CONFIG_MACH_DECKARD_AF33)
DEFINE_LED_TRIGGER(cam_torch_led_1_trigger);
DEFINE_LED_TRIGGER(cam_red_led_trigger);
#else
DEFINE_LED_TRIGGER(cam_torch_led_1_trigger);
DEFINE_LED_TRIGGER(cam_torch_led_2_trigger);
DEFINE_LED_TRIGGER(cam_red_led_trigger);
#endif
/* mutex */
static DEFINE_MUTEX(shcamled_torch_mut);
static DEFINE_MUTEX(shcamled_red_mut);

/* user info */
static int shcamled_use_torch_led_cam = FALSE;
static int shcamled_use_torch_led_comm = FALSE;

/* ------------------------------------------------------------------------- */
/* CODE                                                                      */
/* ------------------------------------------------------------------------- */

/* ------------------------------------------------------------------------- */
/* shcamled_pmic_set_torch_led_1_current                                     */
/* ------------------------------------------------------------------------- */
int shcamled_pmic_set_torch_led_1_current(unsigned mA)
{
#if defined(CONFIG_MACH_DECKARD_AF33) || defined(CONFIG_MACH_DECKARD_AS38)
	int ret;
#endif

	mutex_lock(&shcamled_torch_mut);

	SHCAMLED_TRACE("%s in mA:%d ucam:%d ucom:%d\n", __FUNCTION__, mA,
				   shcamled_use_torch_led_cam, shcamled_use_torch_led_comm);

	if(mA == 0){
		if((shcamled_use_torch_led_cam == FALSE && shcamled_use_torch_led_comm == TRUE) ||
		   (shcamled_use_torch_led_cam == TRUE && shcamled_use_torch_led_comm == FALSE)) {
			/* request turning off */
#if defined(CONFIG_MACH_DECKARD_AF33)
			led_trigger_event(cam_torch_led_1_trigger, LED_OFF);
#else
			led_trigger_event(cam_torch_led_1_trigger, LED_OFF);
			led_trigger_event(cam_torch_led_2_trigger, LED_OFF);
#endif
#if defined(CONFIG_MACH_DECKARD_AF33) || defined(CONFIG_MACH_DECKARD_AS38)
			ret = tps61029_unregister_user(TPS61029_USER_CAMLED);
#endif
			shcamled_use_torch_led_cam = FALSE;
		}
	} else {
		if(shcamled_use_torch_led_cam == FALSE && shcamled_use_torch_led_comm == FALSE) {
			/* request turning on */
#if defined(CONFIG_MACH_DECKARD_AF33) || defined(CONFIG_MACH_DECKARD_AS38)
			ret = tps61029_register_user(TPS61029_USER_CAMLED);
#endif
#if defined(CONFIG_MACH_DECKARD_AF33)
			led_trigger_event(cam_torch_led_1_trigger, LED_FULL);
#else
			led_trigger_event(cam_torch_led_1_trigger, LED_FULL);
			led_trigger_event(cam_torch_led_2_trigger, LED_FULL);
#endif
			shcamled_use_torch_led_cam = TRUE;
		} else if(shcamled_use_torch_led_cam == FALSE && shcamled_use_torch_led_comm == TRUE) {
			/* change user info only */
			shcamled_use_torch_led_comm = FALSE;
			shcamled_use_torch_led_cam = TRUE;
		}
	}

	SHCAMLED_TRACE("%s done ucam:%d ucom:%d\n", __FUNCTION__,
				   shcamled_use_torch_led_cam, shcamled_use_torch_led_comm);

	mutex_unlock(&shcamled_torch_mut);

	return 0;
}
EXPORT_SYMBOL(shcamled_pmic_set_torch_led_1_current);

/* ------------------------------------------------------------------------- */
/* shcamled_pmic_set_torch_led_2_current                                     */
/* ------------------------------------------------------------------------- */
static int shcamled_pmic_set_torch_led_2_current(unsigned mA)
{
#if defined(CONFIG_MACH_DECKARD_AF33) || defined(CONFIG_MACH_DECKARD_AS38)
	int ret;
#endif

	mutex_lock(&shcamled_torch_mut);

	SHCAMLED_TRACE("%s in mA:%d ucam:%d ucom:%d\n", __FUNCTION__, mA,
				   shcamled_use_torch_led_cam, shcamled_use_torch_led_comm);

	if(mA == 0) {
		if(shcamled_use_torch_led_cam == FALSE && shcamled_use_torch_led_comm == TRUE) {
			/* request turning off */
#if defined(CONFIG_MACH_DECKARD_AF33)
			led_trigger_event(cam_torch_led_1_trigger, LED_OFF);
#else
			led_trigger_event(cam_torch_led_1_trigger, LED_OFF);
			led_trigger_event(cam_torch_led_2_trigger, LED_OFF);
#endif
#if defined(CONFIG_MACH_DECKARD_AF33) || defined(CONFIG_MACH_DECKARD_AS38)
			ret = tps61029_unregister_user(TPS61029_USER_CAMLED);
#endif
			shcamled_use_torch_led_comm = FALSE;
		}
	} else {
		if(shcamled_use_torch_led_cam == FALSE && shcamled_use_torch_led_comm == FALSE){
			/* request turning on */
#if defined(CONFIG_MACH_DECKARD_AF33) || defined(CONFIG_MACH_DECKARD_AS38)
			ret = tps61029_register_user(TPS61029_USER_CAMLED);
#endif
#if defined(CONFIG_MACH_DECKARD_AF33)
			led_trigger_event(cam_torch_led_1_trigger, LED_FULL);
#else
			led_trigger_event(cam_torch_led_1_trigger, LED_FULL);
			led_trigger_event(cam_torch_led_2_trigger, LED_FULL);
#endif
			shcamled_use_torch_led_comm = TRUE;
		}
	}

	SHCAMLED_TRACE("%s done ucam:%d ucom:%d\n", __FUNCTION__,
				   shcamled_use_torch_led_cam, shcamled_use_torch_led_comm);

	mutex_unlock(&shcamled_torch_mut);

	return 0;
}

/* ------------------------------------------------------------------------- */
/* shcamled_pmic_req_torch_led_current                                       */
/* ------------------------------------------------------------------------- */
int shcamled_pmic_req_torch_led_current_off(void)
{

	int ret = 0;

	SHCAMLED_TRACE("%s in ucam:%d ucom:%d\n", __FUNCTION__,
				   shcamled_use_torch_led_cam, shcamled_use_torch_led_comm);

	if ((shcamled_use_torch_led_cam == FALSE && shcamled_use_torch_led_comm == TRUE) ||
		(shcamled_use_torch_led_cam == TRUE && shcamled_use_torch_led_comm == FALSE)) {
		ret = shcamled_pmic_set_torch_led_1_current(0);
		ret = shcamled_pmic_set_torch_led_2_current(0);
	} else {
		// NOP
	}

	SHCAMLED_TRACE("%s done\n", __FUNCTION__);
	return ret;
}
EXPORT_SYMBOL(shcamled_pmic_req_torch_led_current_off);

/* ------------------------------------------------------------------------- */
/* shcamled_pmic_set_red_led_current                                         */
/* ------------------------------------------------------------------------- */
static int shcamled_pmic_set_red_led_current(unsigned mA)
{

	mutex_lock(&shcamled_red_mut);

	if(mA == 0) {
		/* request turning off */
		led_trigger_event(cam_red_led_trigger, LED_OFF);
	} else {
		/* request turning on */
		led_trigger_event(cam_red_led_trigger, LED_FULL);
	}

	mutex_unlock(&shcamled_red_mut);

	return 0;
}

/* ------------------------------------------------------------------------- */
/* shcamled_torch_store                                                      */
/* ------------------------------------------------------------------------- */
static ssize_t shcamled_torch_store(struct device *dev,
		struct device_attribute *attr, const char *buf, size_t size)
{

	ssize_t ret = -EINVAL;
	int proc_ret;
	char *after;
	unsigned long val = simple_strtoul(buf, &after, 10);
	size_t count = after - buf;

	SHCAMLED_TRACE("%s in\n", __FUNCTION__);

	/* count written bytes size and check size */
	if (isspace(*after))
		count++;

	if(count == size) {
		ret = count;
		proc_ret = shcamled_pmic_set_torch_led_2_current(val);
		if(proc_ret) {
			ret = -EFAULT;
			SHCAMLED_ERROR("%s failed ret:%d\n", __FUNCTION__, proc_ret);
		}
		SHCAMLED_TRACE("%s done ret:%d\n", __FUNCTION__, proc_ret);
	}

	SHCAMLED_TRACE("%s done ret:%d\n", __FUNCTION__, ret);
	return ret;
}

/* ------------------------------------------------------------------------- */
/* shcamled_red_store                                                        */
/* ------------------------------------------------------------------------- */
static ssize_t shcamled_red_store(struct device *dev,
		struct device_attribute *attr, const char *buf, size_t size)
{

	ssize_t ret = -EINVAL;
	int proc_ret;
	char *after;
	unsigned long val = simple_strtoul(buf, &after, 10);
	size_t count = after - buf;

	SHCAMLED_TRACE("%s in\n", __FUNCTION__);

	/* count written bytes size and check size */
	if (isspace(*after))
		count++;

	if(count == size) {
		ret = count;
		proc_ret = shcamled_pmic_set_red_led_current(val);
		if(proc_ret) {
			ret = -EFAULT;
			SHCAMLED_ERROR("%s failed ret:%d\n", __FUNCTION__, proc_ret);
		}
		SHCAMLED_TRACE("%s done ret:%d\n", __FUNCTION__, proc_ret);
	}

	SHCAMLED_TRACE("%s done ret:%d\n", __FUNCTION__, ret);
	return ret;
}

/* ------------------------------------------------------------------------- */
/* shcamled_torch_prove                                                      */
/* ------------------------------------------------------------------------- */
static int shcamled_torch_probe(struct platform_device *pdev)
{

	int ret;

	SHCAMLED_TRACE("%s in\n", __FUNCTION__);

#if defined(CONFIG_MACH_DECKARD_AF33)
	led_trigger_register_simple("cam-led1", &cam_torch_led_1_trigger);
#else
	led_trigger_register_simple("cam-led1", &cam_torch_led_1_trigger);
	led_trigger_register_simple("cam-led2", &cam_torch_led_2_trigger);
#endif

	ret = device_create_file(&pdev->dev, &dev_attr_shcamled_torch);
	if (ret) {
		SHCAMLED_ERROR("%s failed create file \n", __FUNCTION__);
		return ret;
	}

	SHCAMLED_TRACE("%s done ret:%d\n", __FUNCTION__, ret);
	return ret;
}

/* ------------------------------------------------------------------------- */
/* shcamled_torch_remove                                                     */
/* ------------------------------------------------------------------------- */
static int shcamled_torch_remove(struct platform_device *pdev)
{

	SHCAMLED_TRACE("%s in\n", __FUNCTION__);

#if defined(CONFIG_MACH_DECKARD_AF33)
	led_trigger_unregister_simple(cam_torch_led_1_trigger);
#else
	led_trigger_unregister_simple(cam_torch_led_1_trigger);
	led_trigger_unregister_simple(cam_torch_led_2_trigger);
#endif

	device_remove_file(&pdev->dev, &dev_attr_shcamled_torch);

	SHCAMLED_TRACE("%s done\n", __FUNCTION__);
	return 0;
}

/* ------------------------------------------------------------------------- */
/* shcamled_red_prove                                                        */
/* ------------------------------------------------------------------------- */
static int shcamled_red_probe(struct platform_device *pdev)
{
	int ret;

	SHCAMLED_TRACE("%s in\n", __FUNCTION__);

	led_trigger_register_simple("cam-red-led", &cam_red_led_trigger);

	ret = device_create_file(&pdev->dev, &dev_attr_shcamled_red);
	if (ret) {
		SHCAMLED_ERROR("%s failed create file \n", __FUNCTION__);
		return ret;
	}

	SHCAMLED_TRACE("%s done ret:%d\n", __FUNCTION__, ret);
	return ret;
}

/* ------------------------------------------------------------------------- */
/* shcamled_red_remove                                                       */
/* ------------------------------------------------------------------------- */
static int shcamled_red_remove(struct platform_device *pdev)
{
	SHCAMLED_TRACE("%s in\n", __FUNCTION__);

	led_trigger_unregister_simple(cam_red_led_trigger);

	device_remove_file(&pdev->dev, &dev_attr_shcamled_red);

	SHCAMLED_TRACE("%s done\n", __FUNCTION__);
	return 0;
}

/* ------------------------------------------------------------------------- */
/* shcamled_driver_init                                                      */
/* ------------------------------------------------------------------------- */
static int __init shcamled_torch_driver_init(void)
{
	int ret;

	SHCAMLED_TRACE("%s in\n", __FUNCTION__);

	/* torch 1. register platform-driver for device */
	ret = platform_driver_register(&shcamled_torch_driver);
	if (ret) {
		SHCAMLED_ERROR("%s failed! L.%d ret=%d\n", __FUNCTION__, __LINE__, ret);
		return ret;
	}

	/* torch 2. register platform-device */
	ret = platform_device_register(&shcamled_torch_dev);
	if (ret) {
		SHCAMLED_ERROR("%s failed! L.%d ret=%d\n", __FUNCTION__, __LINE__, ret);
		platform_driver_unregister(&shcamled_torch_driver);
		return ret;
	}

	/* red 1. register platform-driver for device */
	ret = platform_driver_register(&shcamled_red_driver);
	if (ret) {
		SHCAMLED_ERROR("%s failed! L.%d ret=%d\n", __FUNCTION__, __LINE__, ret);
		return ret;
	}

	/* red 2. register platform-device */
	ret = platform_device_register(&shcamled_red_dev);
	if (ret) {
		SHCAMLED_ERROR("%s failed! L.%d ret=%d\n", __FUNCTION__, __LINE__, ret);
		platform_driver_unregister(&shcamled_red_driver);
		return ret;
	}

	SHCAMLED_TRACE("%s done\n", __FUNCTION__);
	return 0;
}
module_init(shcamled_torch_driver_init);

/* ------------------------------------------------------------------------- */
/* shcamled_driver_exit                                                      */
/* ------------------------------------------------------------------------- */
static void __exit shcamled_torch_driver_exit(void)
{
	SHCAMLED_TRACE("%s in\n", __FUNCTION__);

	platform_driver_unregister(&shcamled_torch_driver);
	platform_device_unregister(&shcamled_torch_dev);

	platform_driver_unregister(&shcamled_red_driver);
	platform_device_unregister(&shcamled_red_dev);

	SHCAMLED_TRACE("%s done\n", __FUNCTION__);
}
module_exit(shcamled_torch_driver_exit);

MODULE_DESCRIPTION("SHARP CAMERA TORCH LED DRIVER MODULE");
MODULE_LICENSE("GPL v2");
MODULE_AUTHOR("SHARP CORPORATION");
MODULE_VERSION("1.0");

/* ------------------------------------------------------------------------- */
/* END OF FILE                                                               */
/* ------------------------------------------------------------------------- */
