/* drivers/sharp/shdisp/shdisp_pwm.c  (Display Driver)
 *
 * Copyright (C) 2011-2012 SHARP CORPORATION
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
/* SHARP DISPLAY DRIVER FOR KERNEL MODE                                      */
/* ------------------------------------------------------------------------- */

/* ------------------------------------------------------------------------- */
/* INCLUDE FILES                                                             */
/* ------------------------------------------------------------------------- */

#include <linux/compiler.h>
#include <linux/types.h>
#include <linux/kernel.h>
#include <linux/module.h>
#include <linux/fs.h>
#include <linux/platform_device.h>
#include <linux/device.h>
#include <linux/cdev.h>
#include <linux/interrupt.h>
#include <linux/init.h>
#include <linux/poll.h>
#include <linux/mm.h>
#include <linux/idr.h>
#include <linux/kobject.h>
#include <linux/string.h>
#include <linux/ioctl.h>
#include <linux/gpio.h>
#include <linux/hrtimer.h>
#include <linux/irq.h>
#include <linux/i2c.h>
#include <linux/input.h>
#include <linux/slab.h>
#include <linux/delay.h>
#include <linux/spi/spi.h>
#include <linux/pwm.h>
#include <linux/mfd/pm8xxx/pwm.h>
#include <mach/gpio.h>
#include <mach/msm_iomap.h>
#include <asm/io.h>

#include "shdisp_pwm.h"

/* ------------------------------------------------------------------------- */
/* MACROS                                                                    */
/* ------------------------------------------------------------------------- */



/* ------------------------------------------------------------------------- */
/* PROTOTYPES                                                                */
/* ------------------------------------------------------------------------- */



/* ------------------------------------------------------------------------- */
/* TYPES                                                                     */
/* ------------------------------------------------------------------------- */



/* ------------------------------------------------------------------------- */
/* VARIABLES                                                                 */
/* ------------------------------------------------------------------------- */

static struct pwm_device *dbc_pwm = NULL;

/* ------------------------------------------------------------------------- */
/* DEBUG MACRAOS                                                             */
/* ------------------------------------------------------------------------- */
#define SHDISP_PWM_FILE "shdisp_pwm.c"
#define SHDISP_ERR(fmt, args...) \
        printk("[SHDISP_ERROR][%s][%s] " fmt, SHDISP_PWM_FILE, __func__, ## args);

/* ------------------------------------------------------------------------- */
/* FUNCTIONS                                                                 */
/* ------------------------------------------------------------------------- */
/* ------------------------------------------------------------------------- */
/* API                                                                       */
/* ------------------------------------------------------------------------- */
/* ------------------------------------------------------------------------- */
/* shdisp_PWM_request                                                        */
/* ------------------------------------------------------------------------- */

int  shdisp_PWM_request(void)
{
    if ( dbc_pwm == NULL ) {
        dbc_pwm = pwm_request(SHDISP_PWM_CHANNEL_01, "dbc-lpg");
        if (dbc_pwm == NULL || IS_ERR(dbc_pwm)) {
            SHDISP_ERR("<RESULT_FAILURE> pwm_request.\n");
            dbc_pwm = NULL;
            return SHDISP_RESULT_FAILURE;
        }
        return SHDISP_RESULT_SUCCESS;
    } else {
        return SHDISP_RESULT_SUCCESS;
    }
}


/* ------------------------------------------------------------------------- */
/* shdisp_PWM_release                                                        */
/* ------------------------------------------------------------------------- */

void shdisp_PWM_release(void)
{
    if ( dbc_pwm != NULL ) {
        pwm_free(dbc_pwm);
        dbc_pwm = NULL;
    }
    return;
}


/* ------------------------------------------------------------------------- */
/* shdisp_PWM_enable                                                         */
/* ------------------------------------------------------------------------- */

int  shdisp_PWM_enable(void)
{
    int ret = 0;
    if ( dbc_pwm != NULL ) {
        ret = pwm_enable(dbc_pwm);
        if (ret < 0) {
            SHDISP_ERR("<RESULT_FAILURE> pwm_enable.\n");
            return SHDISP_RESULT_FAILURE;
        }
        return SHDISP_RESULT_SUCCESS;
    } else {
        SHDISP_ERR("<OTHER> dbc_pwm is not requested.\n");
        return SHDISP_RESULT_FAILURE;
    }
}


/* ------------------------------------------------------------------------- */
/* shdisp_PWM_disable                                                        */
/* ------------------------------------------------------------------------- */

void shdisp_PWM_disable(void)
{
    if ( dbc_pwm != NULL ) {
        pwm_disable(dbc_pwm);
    }
    return;
}


/* ------------------------------------------------------------------------- */
/* shdisp_PWM_config                                                         */
/* ------------------------------------------------------------------------- */

int  shdisp_PWM_config(int duty)
{
    int ret = 0;
    int duty_us = 0;
    
    if ( (duty <= 0) || (duty > 100) ) {
        SHDISP_ERR("<INVALID_VALUE> duty(%d).\n", duty);
        return SHDISP_RESULT_FAILURE;
    }
    
    duty_us = PWM_PERIOD * duty / 100;
    
    if (duty_us == 0) {
        duty_us = 1;
    }
    
    if ( dbc_pwm != NULL ) {
        ret = pwm_config(dbc_pwm, duty_us, PWM_PERIOD);
        if (ret < 0) {
            SHDISP_ERR("<RESULT_FAILURE> pwm_config.\n");
            return SHDISP_RESULT_FAILURE;
        }
        return SHDISP_RESULT_SUCCESS;
    } else {
        SHDISP_ERR("<OTHER> dbc_pwm is not requested.\n");
        return SHDISP_RESULT_FAILURE;
    }
}

/* ------------------------------------------------------------------------- */
/* END OF FILE                                                               */
/* ------------------------------------------------------------------------- */
