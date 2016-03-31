/* drivers/sharp/shboot/sh_boot_manager.c
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

/*===========================================================================
INCLUDE
===========================================================================*/
#include <linux/module.h>
#include <sharp/sh_smem.h>

/*===========================================================================
DEFINE
===========================================================================*/

/*===========================================================================
PROTOTYPES
===========================================================================*/
static int sh_boot_get_bootmode_to_user(char *buffer, const struct kernel_param *kp);

/*===========================================================================
GLOBAL VARIABLES
===========================================================================*/
static unsigned long boot_mode = 0;
static struct kernel_param_ops param_ops_bootmode = {
	.get = sh_boot_get_bootmode_to_user,
};

/*=============================================================================
FUNCTION
=============================================================================*/
unsigned short sh_boot_get_hw_revision(void)
{
	sharp_smem_common_type *p_sharp_smem_common_type;

	p_sharp_smem_common_type = sh_smem_get_common_address();
	if( p_sharp_smem_common_type != 0 )
	{
		return p_sharp_smem_common_type->sh_hw_revision;
	}else{
		return 0xFF;
	}
}

unsigned short sh_boot_get_bootmode(void)
{
	sharp_smem_common_type *p_sharp_smem_common_type;

	p_sharp_smem_common_type  = sh_smem_get_common_address();
	if( p_sharp_smem_common_type != 0 )
	{
		return p_sharp_smem_common_type->sh_boot_mode;
	}else{
		return 0;
	}
}

MODULE_DESCRIPTION("SH Boot Manager");
MODULE_LICENSE("GPL v2");
MODULE_AUTHOR("SHARP CORPORATION");
MODULE_VERSION("1.0");

/*=============================================================================

FUNCTION sh_boot_get_bootmode_to_user

DESCRIPTION

DEPENDENCIES
  None

RETURN VALUE

SIDE EFFECTS
  None

NOTE
  None

=============================================================================*/
static int sh_boot_get_bootmode_to_user(char *buffer, const struct kernel_param *kp)
{
	int ret = 0;

	boot_mode = sh_boot_get_bootmode();
	ret = param_get_int(buffer, kp);

	return ret;
}
module_param_cb(boot_mode, &param_ops_bootmode, &boot_mode, 0644);
