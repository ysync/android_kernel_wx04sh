/* include/sharp/shtps_dev_sy3x00.h
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
#ifndef __SHTPS_DEV_SY3X00_H__
#define __SHTPS_DEV_SY3X00_H__

#define SHTPS_SY3X00_GPIO_RST	16
#define SHTPS_SY3X00_GPIO_IRQ	106
#define SHTPS_SY3X00_SPI_CLOCK	1100000

#if defined( CONFIG_SHTPS_SY3000_TM2215_001 )
	#include <sharp/shtps_sy3000_tm2215-001.h>
	#undef	SHTPS_SY3X00_SPI_CLOCK
	#define SHTPS_SY3X00_SPI_CLOCK	400000
#endif	/* CONFIG_SHTPS_SY3000_TM2215_001 */

#if defined( CONFIG_SHTPS_SY3000_TM2314_001 )
	#include <sharp/shtps_sy3000_tm2314-001.h>
	#undef	SHTPS_SY3X00_SPI_CLOCK
	#define SHTPS_SY3X00_SPI_CLOCK	400000
#endif	/* CONFIG_SHTPS_SY3000_TM2314_001 */

#if defined( CONFIG_SHTPS_SY3000_TM2376_001 )
	#include <sharp/shtps_sy3000_tm2376-001.h>
	#undef	SHTPS_SY3X00_SPI_CLOCK
	#define SHTPS_SY3X00_SPI_CLOCK	400000
#endif	/* CONFIG_SHTPS_SY3000_TM2376_001 */

#if defined( CONFIG_SHTPS_SY3000_TM2731_001 )
    #include <sharp/shtps_sy3000_tm2731-001.h>
    #undef  SHTPS_SY3X00_SPI_CLOCK
    #define SHTPS_SY3X00_SPI_CLOCK  400000
#endif  /* CONFIG_SHTPS_SY3000_TM2731_001 */

#endif /* __SHTPS_DEV_SY3X00_H__ */
