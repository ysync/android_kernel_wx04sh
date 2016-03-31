/* drivers/sharp/shdisp/shdisp_bd2802gu.h  (Display Driver)
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

#ifndef SHDISP_BD2802GU_H
#define SHDISP_BD2802GU_H

/* ------------------------------------------------------------------------- */
/* INCLUDE FILES                                                             */
/* ------------------------------------------------------------------------- */

#include <sharp/shdisp_kerl.h>

/* ------------------------------------------------------------------------- */
/* MACROS                                                                    */
/* ------------------------------------------------------------------------- */

#define LEDC_REG_IC_CTRL                            0x00
#define LEDC_REG_RGB_CTRL                           0x01
#define LEDC_REG_RGB1_TIME                          0x02
#define LEDC_REG_R1_VAL1                            0x03
#define LEDC_REG_R1_VAL2                            0x04
#define LEDC_REG_R1_PTRN                            0x05
#define LEDC_REG_G1_VAL1                            0x06
#define LEDC_REG_G1_VAL2                            0x07
#define LEDC_REG_G1_PTRN                            0x08
#define LEDC_REG_B1_VAL1                            0x09
#define LEDC_REG_B1_VAL2                            0x0A
#define LEDC_REG_B1_PTRN                            0x0B
#define LEDC_REG_RGB2_TIME                          0x0C
#define LEDC_REG_R2_VAL1                            0x0D
#define LEDC_REG_R2_VAL2                            0x0E
#define LEDC_REG_R2_PTRN                            0x0F
#define LEDC_REG_G2_VAL1                            0x10
#define LEDC_REG_G2_VAL2                            0x11
#define LEDC_REG_G2_PTRN                            0x12
#define LEDC_REG_B2_VAL1                            0x13
#define LEDC_REG_B2_VAL2                            0x14
#define LEDC_REG_B2_PTRN                            0x15


/* ------------------------------------------------------------------------- */
/* TYPES                                                                     */
/* ------------------------------------------------------------------------- */





/* ------------------------------------------------------------------------- */
/* PROTOTYPES                                                                */
/* ------------------------------------------------------------------------- */

void shdisp_ledc_API_set_status_addr(struct shdisp_ledc_status* state_str);
int shdisp_ledc_API_init(void);
int shdisp_ledc_API_exit(void);
int shdisp_ledc_API_power_on(void);
int shdisp_ledc_API_power_off(void);
void shdisp_ledc_API_exist(int* ledc_is_exist);
void shdisp_ledc_API_set_rgb(struct shdisp_ledc_rgb *ledc_rgb);
void shdisp_ledc_API_set_color(struct shdisp_ledc_req *ledc_req);
void shdisp_ledc_API_DIAG_write_reg(unsigned char reg, unsigned char val);
void shdisp_ledc_API_DIAG_read_reg(unsigned char reg, unsigned char *val);

#endif /* SHDISP_BD2802GU_H */

/* ------------------------------------------------------------------------- */
/* END OF FILE                                                               */
/* ------------------------------------------------------------------------- */
