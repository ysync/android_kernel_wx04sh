/* drivers/sharp/shcamled/tps61029.c
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
#include <linux/regulator/consumer.h>
#include <linux/leds-pmic8058.h>
#include <linux/leds-pm8xxx.h>
#include <linux/err.h>
#include <linux/module.h>
#include <sharp/tps61029.h>

/* ------------------------------------------------------------------------- */
/* PROTOTYPES                                                                */
/* ------------------------------------------------------------------------- */
static int tps61029_control_port(int isEnable);
extern int shcamled_pmic_req_torch_led_current_off(void);

/* ------------------------------------------------------------------------- */
/* MACROS                                                                    */
/* ------------------------------------------------------------------------- */
#define FALSE 0
#define TRUE 1

//#define TPS61029_ENABLE_DEBUG

#ifdef TPS61029_ENABLE_DEBUG
#define TPS61029_INFO(x...)	printk(KERN_INFO "[TPS61029] " x)
#define TPS61029_TRACE(x...)	printk(KERN_DEBUG "[TPS61029] " x)
#define TPS61029_ERROR(x...)	printk(KERN_ERR "[TPS61029] " x)
#else
#define TPS61029_INFO(x...)	do {} while(0)
#define TPS61029_TRACE(x...)	do {} while(0)
#define TPS61029_ERROR(x...)	printk(KERN_ERR "[TPS61029] " x)
#endif

/* ------------------------------------------------------------------------- */
/* VARIABLES                                                                 */
/* ------------------------------------------------------------------------- */
static int tps61029_use_usb = FALSE;
static int tps61029_use_camled = FALSE;
static int tps61029_use_kbled = FALSE;
static struct regulator *ext_5v = NULL;
DEFINE_MUTEX(tps61029_mut);

/* ------------------------------------------------------------------------- */
/* tps61029_control_port                                                     */
/* ------------------------------------------------------------------------- */
static int tps61029_control_port(int isEnable)
{

	TPS61029_TRACE("%s req:%d \n", __FUNCTION__, isEnable);

	if(isEnable == TRUE) {
		if(ext_5v == NULL) {
			ext_5v = regulator_get(NULL, "ext_5v");
			if (IS_ERR(ext_5v)) {
				pr_err("%s: ext_5v get failed\n", __func__);
				ext_5v = NULL;
				return TPS61029_FAILURE;
			}
			if(regulator_enable(ext_5v)) {
				TPS61029_ERROR("%s: ext_5v enable failed\n", __FUNCTION__);
				return TPS61029_FAILURE;
			}
		}
	} else {
		regulator_disable(ext_5v);
		regulator_put(ext_5v);
		ext_5v = NULL;
	}

	TPS61029_TRACE("%s req:%d proc done\n", __FUNCTION__, isEnable);
	return TPS61029_SUCCESS;
}

/* ------------------------------------------------------------------------- */
/* tps61029_register_user                                                    */
/* ------------------------------------------------------------------------- */
int tps61029_register_user(int user)
{

	int rc = TPS61029_FAILURE;
	int ctrl_rc = FALSE;

	if(user < TPS61029_USER_USB || user > TPS61029_USER_CAMLED) {
		TPS61029_ERROR("[E] %s L.%d\n", __FUNCTION__, __LINE__);
		return TPS61029_FAILURE;
	}

	mutex_lock(&tps61029_mut);

	if((user == TPS61029_USER_USB && tps61029_use_usb == TRUE) ||
	   (user == TPS61029_USER_KBLED && tps61029_use_kbled == TRUE) ||
	   (user == TPS61029_USER_CAMLED && tps61029_use_camled == TRUE)) {
		TPS61029_TRACE("%s %d user registered already\n", __FUNCTION__, user);
		mutex_unlock(&tps61029_mut);
		return TPS61029_SUCCESS;
	}

	if(user == TPS61029_USER_CAMLED && tps61029_use_usb == TRUE) {
		TPS61029_TRACE("%s block %d user\n", __FUNCTION__, user);
		mutex_unlock(&tps61029_mut);
		return TPS61029_FAILURE;
	}

	TPS61029_TRACE("%s %d user U:%d C:%d\n",
				   __FUNCTION__, user,tps61029_use_usb, tps61029_use_camled);

	switch (user) {
	case TPS61029_USER_USB:
#if defined(CONFIG_MACH_DECKARD_AF33)
		TPS61029_ERROR("[E] %s L.%d NOP\n", __FUNCTION__, __LINE__);
#else
		if(tps61029_use_usb == FALSE && tps61029_use_camled == FALSE) {
			ctrl_rc = tps61029_control_port(TRUE);
			if(ctrl_rc == TPS61029_SUCCESS) {
				tps61029_use_usb = TRUE;
				rc = TPS61029_SUCCESS;
			} else {
				rc = TPS61029_FAILURE;
			}
		} else if(tps61029_use_usb == FALSE && tps61029_use_camled == TRUE) {
			rc = shcamled_pmic_req_torch_led_current_off();
			TPS61029_TRACE("%s proc turning off LED L.%d\n",  __FUNCTION__, __LINE__);
			tps61029_use_usb = TRUE;
			rc = TPS61029_SUCCESS;
		} else {
			TPS61029_ERROR("[E] %s L.%d\n", __FUNCTION__, __LINE__);
		}
#endif
		break;

	case TPS61029_USER_CAMLED:
#if defined(CONFIG_MACH_DECKARD_AF33)
		if(tps61029_use_kbled == FALSE && tps61029_use_camled == FALSE) {
			ctrl_rc = tps61029_control_port(TRUE);
			if(ctrl_rc == TPS61029_SUCCESS) {
				tps61029_use_camled = TRUE;
				rc = TPS61029_SUCCESS;
			} else {
				rc = TPS61029_FAILURE;
			}
		} else if(tps61029_use_kbled == TRUE && tps61029_use_camled == FALSE) {
			tps61029_use_camled = TRUE;
			rc = TPS61029_SUCCESS;
		} else {
			TPS61029_ERROR("[E] %s L.%d\n", __FUNCTION__, __LINE__);
		}
#else
		if(tps61029_use_usb == FALSE && tps61029_use_camled == FALSE) {
			ctrl_rc = tps61029_control_port(TRUE);
			if(ctrl_rc == TPS61029_SUCCESS) {
				tps61029_use_camled = TRUE;
				rc = TPS61029_SUCCESS;
			} else {
				rc = TPS61029_FAILURE;
			}
		} else {
			TPS61029_ERROR("[E] %s L.%d\n", __FUNCTION__, __LINE__);
		}
#endif
		break;

	case TPS61029_USER_KBLED:
#if defined(CONFIG_MACH_DECKARD_AF33)
		if(tps61029_use_kbled == FALSE && tps61029_use_camled == FALSE) {
			ctrl_rc = tps61029_control_port(TRUE);
			if(ctrl_rc == TPS61029_SUCCESS) {
				tps61029_use_kbled = TRUE;
				rc = TPS61029_SUCCESS;
			} else {
				rc = TPS61029_FAILURE;
			}
		} else if(tps61029_use_kbled == FALSE && tps61029_use_camled == TRUE) {
			tps61029_use_kbled = TRUE;
			rc = TPS61029_SUCCESS;
		}else {
			TPS61029_ERROR("[E] %s L.%d\n", __FUNCTION__, __LINE__);
		}
#else
		TPS61029_ERROR("[E] %s L.%d NOP\n", __FUNCTION__, __LINE__);
#endif
		break;

	default:
		break;
	}

	TPS61029_TRACE("%s done\n", __FUNCTION__);

	mutex_unlock(&tps61029_mut);
	return rc;
}
EXPORT_SYMBOL(tps61029_register_user);

/* ------------------------------------------------------------------------- */
/* tps61029_unregister_user                                                  */
/* ------------------------------------------------------------------------- */
int tps61029_unregister_user(int user)
{

	int rc = TPS61029_FAILURE;
	int ctrl_rc = FALSE;

	if(user < TPS61029_USER_USB || user > TPS61029_USER_CAMLED) {
		TPS61029_ERROR("[E] %s L.%d\n", __FUNCTION__, __LINE__);
		return TPS61029_FAILURE;
	}

	mutex_lock(&tps61029_mut);

	if((user == TPS61029_USER_USB && tps61029_use_usb == FALSE) ||
	   (user == TPS61029_USER_KBLED &&  tps61029_use_kbled == FALSE) ||
	   (user == TPS61029_USER_CAMLED && tps61029_use_camled == FALSE)) {
		TPS61029_TRACE("%s %d user unregistered already\n", __FUNCTION__, user);
		mutex_unlock(&tps61029_mut);
		return TPS61029_SUCCESS;
	}

	TPS61029_TRACE("%s %d user U:%d C:%d\n",
				   __FUNCTION__, user,tps61029_use_usb, tps61029_use_camled);

	switch (user) {
	case TPS61029_USER_USB:
#if defined(CONFIG_MACH_DECKARD_AF33)
		TPS61029_ERROR("[E] %s L.%d NOP\n", __FUNCTION__, __LINE__);
#else
		if(tps61029_use_usb == TRUE && tps61029_use_camled == FALSE) {
			ctrl_rc = tps61029_control_port(FALSE);
			if(ctrl_rc == TPS61029_SUCCESS) {
				rc = TPS61029_SUCCESS;
				tps61029_use_usb = FALSE;
			} else {
				rc = TPS61029_FAILURE;
			}
		}
		if(tps61029_use_usb == TRUE && tps61029_use_camled == TRUE) {
			tps61029_use_usb = FALSE;
			rc = TPS61029_SUCCESS;
		}
#endif
		break;

	case TPS61029_USER_CAMLED:
#if defined(CONFIG_MACH_DECKARD_AF33)
		if(tps61029_use_kbled == FALSE && tps61029_use_camled == TRUE) {
			ctrl_rc = tps61029_control_port(FALSE);
			if(ctrl_rc == TPS61029_SUCCESS) {
				rc = TPS61029_SUCCESS;
				tps61029_use_camled = FALSE;
			} else {
				rc = TPS61029_FAILURE;
			}
		} else if(tps61029_use_kbled == TRUE && tps61029_use_camled == TRUE) {
			tps61029_use_camled = FALSE;
			rc = TPS61029_SUCCESS;
		}
#else
		if(tps61029_use_usb == FALSE && tps61029_use_camled == TRUE) {
			ctrl_rc = tps61029_control_port(FALSE);
			if(ctrl_rc == TPS61029_SUCCESS) {
				rc = TPS61029_SUCCESS;
				tps61029_use_camled = FALSE;
			} else {
				rc = TPS61029_FAILURE;
			}
		} else if(tps61029_use_usb == TRUE && tps61029_use_camled == TRUE) {
			tps61029_use_camled = FALSE;
			rc = TPS61029_SUCCESS;
		}
#endif
		break;

	case TPS61029_USER_KBLED:
#if defined(CONFIG_MACH_DECKARD_AF33)
		if(tps61029_use_kbled == TRUE && tps61029_use_camled == FALSE) {
			ctrl_rc = tps61029_control_port(FALSE);
			if(ctrl_rc == TPS61029_SUCCESS) {
				rc = TPS61029_SUCCESS;
				tps61029_use_kbled = FALSE;
			} else {
				rc = TPS61029_FAILURE;
			}
		} else if(tps61029_use_kbled == TRUE && tps61029_use_camled == TRUE) {
			rc = TPS61029_SUCCESS;
			tps61029_use_kbled = FALSE;
		}
#else
		TPS61029_ERROR("[E] %s L.%d NOP\n", __FUNCTION__, __LINE__);
#endif
		break;

	default:
		break;
	}

	TPS61029_TRACE("%s done\n", __FUNCTION__);

	mutex_unlock(&tps61029_mut);
	return rc;
}
EXPORT_SYMBOL(tps61029_unregister_user);

/* ------------------------------------------------------------------------- */
/* tps61029_get_user                                                         */
/* ------------------------------------------------------------------------- */
int tps61029_get_user(void)
{

	if(tps61029_use_usb == TRUE) {
		if (tps61029_use_camled == FALSE)
			return TPS61029_USER_USB;
		else
			return TPS61029_USER_BOTH;
	} else if(tps61029_use_kbled == TRUE) {
		if (tps61029_use_camled == FALSE)
			return TPS61029_USER_KBLED;
		else
			return TPS61029_USER_BOTH;
	} else {
		if (tps61029_use_camled == FALSE)
			return TPS61029_USER_NONE;
		else
			return TPS61029_USER_CAMLED;
	}
}
EXPORT_SYMBOL(tps61029_get_user);

/* ------------------------------------------------------------------------- */
/* END OF FILE                                                               */
/* ------------------------------------------------------------------------- */
