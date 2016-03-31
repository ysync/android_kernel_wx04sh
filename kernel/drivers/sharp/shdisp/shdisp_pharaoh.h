/* drivers/sharp/shdisp/shdisp_pharaoh.h  (Display Driver)
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

#ifndef SHDISP_PHARAOH_H
#define SHDISP_PHARAOH_H

/* ------------------------------------------------------------------------- */
/* MACROS                                                                    */
/* ------------------------------------------------------------------------- */

#define SHDISP_PHARAOH_VCOM_MAX                         0x1FF

#define PHAR_NUM_SIO_CONTROL_MAX_WRITE                  15
#define PHAR_NUM_SIO_CONTROL_MIN_WRITE                  0
#define PHAR_NUM_SIO_CONTROL_MAX_READ                   15
#define PHAR_NUM_SIO_CONTROL_MIN_READ                   1

#define PHAR_REG_ENTER_SLEEP_MODE                       0x10
#define PHAR_REG_EXIT_SLEEP_MODE                        0x11

#define PHAR_REG_SET_DISPLAY_ON                         0x29

#define PHAR_REG_MANUFACTURE_COMMAND_ACCESS_PROJECT     0xB0
#define PHAR_REG_LOW_POWER_MODE_CONTROL                 0xB1
#define PHAR_REG_AUTO_COMMAND_ACCESS_PROJECT            0xB2
#define PHAR_REG_CLOCK_AND_INTERFACE_SETTING            0xB3
#define PHAR_REG_PIXEL_FORMAT_SETTING                   0xB4
#define PHAR_REG_READ_CHECKSUM_AND_ECC_ERR_COUNT        0xB5
#define PHAR_REG_DSI_CONTROL                            0xB6
#define PHAR_REG_PFM_PWM_CONTROL                        0xB9
#define PHAR_REG_CABC_ON_OFF                            0xBB
#define PHAR_REG_CABC_USER_PARAMETER                    0xBE
#define PHAR_REG_DEVICE_CODE_READ                       0xBF

#define PHAR_REG_PANNEL_DRIVING_SETTING                 0xC0
#define PHAR_REG_DISPLAY_H_TIMING_SETTING               0xC1
#define PHAR_REG_SOURCE_OUTPUT_SETTING                  0xC2
#define PHAR_REG_GATE_DRIVE_IF_SETTING                  0xC3
#define PHAR_REG_LTPS_INTERFACE_MODE                    0xC4
#define PHAR_REG_PBCTRL_CONTROL                         0xC5
#define PHAR_REG_DISPLAY_RGB_SWITCH_ORDER               0xC6
#define PHAR_REG_LTPS_INTERFACE_CONTROL                 0xC7
#define PHAR_REG_GAMMA_CONTROL                          0xC8
#define PHAR_REG_GAMMA_CONTROL_OF_SET_A_POSITIVE        0xC9
#define PHAR_REG_GAMMA_CONTROL_OF_SET_A_NEGATIVE        0xCA
#define PHAR_REG_GAMMA_CONTROL_OF_SET_B_POSITIVE        0xCB
#define PHAR_REG_GAMMA_CONTROL_OF_SET_B_NEGATIVE        0xCC
#define PHAR_REG_GAMMA_CONTROL_OF_SET_C_POSITIVE        0xCD
#define PHAR_REG_GAMMA_CONTROL_OF_SET_C_NEGATIVE        0xCE

#define PHAR_REG_POWER_SETTING_1                        0xD0
#define PHAR_REG_POWER_SETTING_2                        0xD1
#define PHAR_REG_POWER_SETTING_FOR_INTERNAL             0xD3
#define PHAR_REG_VPLVL_VNLVL_SETTING                    0xD5
#define PHAR_REG_VCOMDC_SETTING                         0xDE

#define SHDISP_PHAR_SETVGH      0x6D
#define SHDISP_PHAR_SETVGL      0x65
#define SHDISP_PHAR_SETVCLI     0x4C
#define SHDISP_PHAR_SETVSPN     0x77
#define SHDISP_PHAR_SETPVH      0x0F
#define SHDISP_PHAR_SETLVH      0x0F
#define SHDISP_PHAR_COMDCV_A    0x31
#define SHDISP_PHAR_COMDCV_B    0x51

#if defined(CONFIG_MACH_LYNX_DL12)
#define SHDISP_PHAR_CGMP00      0x00
#define SHDISP_PHAR_CGMP01      0x13
#define SHDISP_PHAR_CGMN00      0x00
#define SHDISP_PHAR_CGMN01      0x13

#define SHDISP_PHAR_RP00        0x04
#define SHDISP_PHAR_RP01        0x00
#define SHDISP_PHAR_RP02        0x07
#define SHDISP_PHAR_RP03        0x12
#define SHDISP_PHAR_RP04        0x14
#define SHDISP_PHAR_RP05        0x0E
#define SHDISP_PHAR_RP06        0x16
#define SHDISP_PHAR_RP07        0x1B
#define SHDISP_PHAR_RP08        0x10
#define SHDISP_PHAR_RP09        0x0E
#define SHDISP_PHAR_RP10        0x24
#define SHDISP_PHAR_RP11        0x28
#define SHDISP_PHAR_RP12        0x73

#define SHDISP_PHAR_RN00        0x3B
#define SHDISP_PHAR_RN01        0x3F
#define SHDISP_PHAR_RN02        0x58
#define SHDISP_PHAR_RN03        0x4D
#define SHDISP_PHAR_RN04        0x4B
#define SHDISP_PHAR_RN05        0x51
#define SHDISP_PHAR_RN06        0x49
#define SHDISP_PHAR_RN07        0x44
#define SHDISP_PHAR_RN08        0x4F
#define SHDISP_PHAR_RN09        0x51
#define SHDISP_PHAR_RN10        0x3B
#define SHDISP_PHAR_RN11        0x17
#define SHDISP_PHAR_RN12        0x4C

#define SHDISP_PHAR_GP00        0x04
#define SHDISP_PHAR_GP01        0x00
#define SHDISP_PHAR_GP02        0x07
#define SHDISP_PHAR_GP03        0x12
#define SHDISP_PHAR_GP04        0x14
#define SHDISP_PHAR_GP05        0x0E
#define SHDISP_PHAR_GP06        0x16
#define SHDISP_PHAR_GP07        0x1B
#define SHDISP_PHAR_GP08        0x10
#define SHDISP_PHAR_GP09        0x0E
#define SHDISP_PHAR_GP10        0x24
#define SHDISP_PHAR_GP11        0x28
#define SHDISP_PHAR_GP12        0x73

#define SHDISP_PHAR_GN00        0x3B
#define SHDISP_PHAR_GN01        0x3F
#define SHDISP_PHAR_GN02        0x58
#define SHDISP_PHAR_GN03        0x4D
#define SHDISP_PHAR_GN04        0x4B
#define SHDISP_PHAR_GN05        0x51
#define SHDISP_PHAR_GN06        0x49
#define SHDISP_PHAR_GN07        0x44
#define SHDISP_PHAR_GN08        0x4F
#define SHDISP_PHAR_GN09        0x51
#define SHDISP_PHAR_GN10        0x3B
#define SHDISP_PHAR_GN11        0x17
#define SHDISP_PHAR_GN12        0x4C

#define SHDISP_PHAR_BP00        0x04
#define SHDISP_PHAR_BP01        0x00
#define SHDISP_PHAR_BP02        0x07
#define SHDISP_PHAR_BP03        0x12
#define SHDISP_PHAR_BP04        0x14
#define SHDISP_PHAR_BP05        0x0E
#define SHDISP_PHAR_BP06        0x16
#define SHDISP_PHAR_BP07        0x1B
#define SHDISP_PHAR_BP08        0x10
#define SHDISP_PHAR_BP09        0x0E
#define SHDISP_PHAR_BP10        0x24
#define SHDISP_PHAR_BP11        0x28
#define SHDISP_PHAR_BP12        0x73

#define SHDISP_PHAR_BN00        0x3B
#define SHDISP_PHAR_BN01        0x3F
#define SHDISP_PHAR_BN02        0x58
#define SHDISP_PHAR_BN03        0x4D
#define SHDISP_PHAR_BN04        0x4B
#define SHDISP_PHAR_BN05        0x51
#define SHDISP_PHAR_BN06        0x49
#define SHDISP_PHAR_BN07        0x44
#define SHDISP_PHAR_BN08        0x4F
#define SHDISP_PHAR_BN09        0x51
#define SHDISP_PHAR_BN10        0x3B
#define SHDISP_PHAR_BN11        0x17
#define SHDISP_PHAR_BN12        0x4C

#else

#define SHDISP_PHAR_CGMP00      0x00
#define SHDISP_PHAR_CGMP01      0x1E
#define SHDISP_PHAR_CGMN00      0x00
#define SHDISP_PHAR_CGMN01      0x1E

#define SHDISP_PHAR_RP00        0x00
#define SHDISP_PHAR_RP01        0x01
#define SHDISP_PHAR_RP02        0x0C
#define SHDISP_PHAR_RP03        0x1B
#define SHDISP_PHAR_RP04        0x1E
#define SHDISP_PHAR_RP05        0x18
#define SHDISP_PHAR_RP06        0x21
#define SHDISP_PHAR_RP07        0x27
#define SHDISP_PHAR_RP08        0x1D
#define SHDISP_PHAR_RP09        0x1C
#define SHDISP_PHAR_RP10        0x35
#define SHDISP_PHAR_RP11        0x37
#define SHDISP_PHAR_RP12        0x7F

#define SHDISP_PHAR_RN00        0x3F
#define SHDISP_PHAR_RN01        0x3E
#define SHDISP_PHAR_RN02        0x53
#define SHDISP_PHAR_RN03        0x44
#define SHDISP_PHAR_RN04        0x41
#define SHDISP_PHAR_RN05        0x47
#define SHDISP_PHAR_RN06        0x3E
#define SHDISP_PHAR_RN07        0x38
#define SHDISP_PHAR_RN08        0x42
#define SHDISP_PHAR_RN09        0x43
#define SHDISP_PHAR_RN10        0x2A
#define SHDISP_PHAR_RN11        0x08
#define SHDISP_PHAR_RN12        0x40

#define SHDISP_PHAR_GP00        0x00
#define SHDISP_PHAR_GP01        0x01
#define SHDISP_PHAR_GP02        0x0C
#define SHDISP_PHAR_GP03        0x1B
#define SHDISP_PHAR_GP04        0x1E
#define SHDISP_PHAR_GP05        0x18
#define SHDISP_PHAR_GP06        0x21
#define SHDISP_PHAR_GP07        0x27
#define SHDISP_PHAR_GP08        0x1D
#define SHDISP_PHAR_GP09        0x1C
#define SHDISP_PHAR_GP10        0x35
#define SHDISP_PHAR_GP11        0x37
#define SHDISP_PHAR_GP12        0x7F

#define SHDISP_PHAR_GN00        0x3F
#define SHDISP_PHAR_GN01        0x3E
#define SHDISP_PHAR_GN02        0x53
#define SHDISP_PHAR_GN03        0x44
#define SHDISP_PHAR_GN04        0x41
#define SHDISP_PHAR_GN05        0x47
#define SHDISP_PHAR_GN06        0x3E
#define SHDISP_PHAR_GN07        0x38
#define SHDISP_PHAR_GN08        0x42
#define SHDISP_PHAR_GN09        0x43
#define SHDISP_PHAR_GN10        0x2A
#define SHDISP_PHAR_GN11        0x08
#define SHDISP_PHAR_GN12        0x40

#define SHDISP_PHAR_BP00        0x21
#define SHDISP_PHAR_BP01        0x1B
#define SHDISP_PHAR_BP02        0x20
#define SHDISP_PHAR_BP03        0x25
#define SHDISP_PHAR_BP04        0x25
#define SHDISP_PHAR_BP05        0x1E
#define SHDISP_PHAR_BP06        0x23
#define SHDISP_PHAR_BP07        0x28
#define SHDISP_PHAR_BP08        0x1F
#define SHDISP_PHAR_BP09        0x1D
#define SHDISP_PHAR_BP10        0x3A
#define SHDISP_PHAR_BP11        0x3F
#define SHDISP_PHAR_BP12        0x7F

#define SHDISP_PHAR_BN00        0x1E
#define SHDISP_PHAR_BN01        0x24
#define SHDISP_PHAR_BN02        0x3F
#define SHDISP_PHAR_BN03        0x3A
#define SHDISP_PHAR_BN04        0x3A
#define SHDISP_PHAR_BN05        0x41
#define SHDISP_PHAR_BN06        0x3C
#define SHDISP_PHAR_BN07        0x37
#define SHDISP_PHAR_BN08        0x40
#define SHDISP_PHAR_BN09        0x42
#define SHDISP_PHAR_BN10        0x25
#define SHDISP_PHAR_BN11        0x00
#define SHDISP_PHAR_BN12        0x40
#endif
/* ------------------------------------------------------------------------- */
/* TYPES                                                                     */
/* ------------------------------------------------------------------------- */


/* ------------------------------------------------------------------------- */
/* VARIABLES                                                                 */
/* ------------------------------------------------------------------------- */



/* ------------------------------------------------------------------------- */
/* PROTOTYPES                                                                */
/* ------------------------------------------------------------------------- */

struct shdisp_panel_operations *shdisp_pharaoh_API_create(void);

#endif /* SHDISP_PHARAOH_H */

/* ------------------------------------------------------------------------- */
/* END OF FILE                                                               */
/* ------------------------------------------------------------------------- */

