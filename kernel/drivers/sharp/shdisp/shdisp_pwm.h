/* drivers/sharp/shdisp/shdisp_pwm.h  (Display Driver)
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

#ifndef SHDISP_PWM_H
#define SHDISP_PWM_H

/* ------------------------------------------------------------------------- */
/* INCLUDE FILES                                                             */
/* ------------------------------------------------------------------------- */

#include <sharp/shdisp_kerl.h>

/* ------------------------------------------------------------------------- */
/* MACROS                                                                    */
/* ------------------------------------------------------------------------- */

#define SHDISP_PWM_CHANNEL_01   0
#define SHDISP_PWM_CHANNEL_02   1
#define SHDISP_PWM_CHANNEL_03   2
#define SHDISP_PWM_CHANNEL_04   3
#define SHDISP_PWM_CHANNEL_05   4
#define SHDISP_PWM_CHANNEL_06   5
#define SHDISP_PWM_CHANNEL_07   6
#define SHDISP_PWM_CHANNEL_08   7

#define PWM_PERIOD 53

/* ------------------------------------------------------------------------- */
/* TYPES                                                                     */
/* ------------------------------------------------------------------------- */






/* ------------------------------------------------------------------------- */
/* PROTOTYPES                                                                */
/* ------------------------------------------------------------------------- */

int  shdisp_PWM_request(void);
void shdisp_PWM_release(void);
int  shdisp_PWM_enable(void);
void shdisp_PWM_disable(void);
int  shdisp_PWM_config(int duty);

#endif /* SHDISP_PWM_H */

/* ------------------------------------------------------------------------- */
/* END OF FILE                                                               */
/* ------------------------------------------------------------------------- */
