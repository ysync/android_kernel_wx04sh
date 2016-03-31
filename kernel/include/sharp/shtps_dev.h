/* include/sharp/shtps_dev.h
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
#ifndef __SHTPS_DEV_H__
#define __SHTPS_DEV_H__

#include <sharp/shtps_dev_cfg.h>

#if defined(CONFIG_SHTPS_TMA4XX_DEV)
#include <sharp/shtps_dev_tma4xx.h>
#elif defined(CONFIG_SHTPS_SY3X00_DEV)
#include <sharp/shtps_dev_sy3x00.h>
#elif defined(CONFIG_SHTPS_NASSAU_DEV)
#include <sharp/shtps_dev_nassau.h>
#endif

#endif /* __SHTPS_DEV_H__ */
