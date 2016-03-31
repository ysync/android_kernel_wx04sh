/* drivers/sharp/phsmdm/phsmdm_def.h
 *
 * Copyright (C) 2011-2013 SHARP CORPORATION
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

#ifndef _PHSMDM_DEF_H
#define _PHSMDM_DEF_H

#include <linux/mutex.h>
#include <linux/remote_spinlock.h>
#include <linux/device.h>

#define PHSMDM_PHS_DTR 92
#define PHSMDM_PHS_PWRCONT_PP1 140
#define PHSMDM_PHS_PWRCONT_PP2 55
#define PHSMDM_PHS_PWRRESET 124
#define SHPHS_TIOCM_DTR		92
#define SHPHS_TIOCM_CAR		90
#define SHPHS_TIOCM_RI		14
#define SHPHS_DISP1			99
#define SHPHS_DISP2			100
#define SHPHS_DISP3			101

#define PHSMDM_OPEN_TIME 250

#define PHSMDM_PP1 1
struct phsmdm_irq
{
	int	dcd;
	int	ri;
	int	disp1;
	int	disp2;
	int disp3;
};

struct msm_phsmdm_platform_data
{
	struct device *dev;
	struct mutex phsmdm_lock;
	int num_open;
	int is_not_first_open;
	spinlock_t lock;
	wait_queue_head_t phsmdm_wait;
	struct phsmdm_irq phs_irq;
	struct phsmdm_icount icount;
	uint8_t pwrcont_num;
	int enable_report_disp;
#ifdef	DEBUG_LOG
	unsigned char *membase;
#endif	/* DEBUG_LOG */
	int report_resume;
};

#endif /* _PHSMDM_DEF_H */
