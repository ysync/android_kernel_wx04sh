/* drivers/sharp/shuim/shuim.h
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

#ifndef SHUIM_H
#define SHUIM_H

/* LOG */
#ifdef DEBUG_SHUIM_DRV
#define SHUIM_DRV_DBG_LOG(fmt, args...) printk(KERN_INFO "[SHUIM][%s]" fmt "\n", __func__, ## args)
#else
#define SHUIM_DRV_DBG_LOG(fmt, args...)
#endif

#define SHUIM_DRV_ERR_LOG(fmt, args...) printk(KERN_ERR "[SHUIM][%s]ERR " fmt "\n", __func__, ## args)

/* prototype */
struct poll_data
{
	wait_queue_head_t read_wait;
	int irq_handler_done;
	struct delayed_work work;
	int device_status;
	int read_error;
	int open_flag;
};

#endif /* SHUIM_H */

