/* Copyright (c) 2009-2012, The Linux Foundation. All rights reserved.
 *
 * This program is free software; you can redistribute it and/or modify
 * it under the terms of the GNU General Public License version 2 and
 * only version 2 as published by the Free Software Foundation.
 *
 * This program is distributed in the hope that it will be useful,
 * but WITHOUT ANY WARRANTY; without even the implied warranty of
 * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
 * GNU General Public License for more details.
 *
 */
#include <linux/module.h>
#include <linux/kernel.h>
#include <linux/sched.h>
#include <linux/time.h>
#include <linux/init.h>
#include <linux/interrupt.h>
#include <linux/spinlock.h>
#include <linux/hrtimer.h>
#include <linux/clk.h>
#include <linux/io.h>
#include <linux/debugfs.h>
#include <linux/semaphore.h>
#include <linux/uaccess.h>
#include <asm/system.h>
#include <asm/mach-types.h>
#include <mach/hardware.h>

#include "mdp.h"
#include "msm_fb.h"
#ifdef CONFIG_FB_MSM_MDP40
#include "mdp4.h"
#endif
#include "mddihosti.h"
#include "tvenc.h"
#ifdef CONFIG_FB_MSM_HDMI_MSM_PANEL
#include "hdmi_msm.h"
#endif

#ifdef CONFIG_SHLCDC_BOARD /* CUST_ID_00039 */
#include "mipi_dsi.h"
#endif /* CONFIG_SHLCDC_BOARD */

#ifdef CONFIG_SHLCDC_BOARD /* CUST_ID_00124 */
extern struct timespec mdp4_dmap_max_time;
#endif /* CONFIG_SHLCDC_BOARD */

#define MDP_DEBUG_BUF	2048

#ifdef CONFIG_SHLCDC_BOARD /* CUST_ID_00095 */
#define MDP_REQ_DEBUG_OUT_STAT          0x00000001
#define MDP_REQ_DEBUG_OUT_PAN_DISPLAY   0x00000002
#define MDP_REQ_DEBUG_OUT_OVERLAY_SET   0x00000004
#define MDP_REQ_DEBUG_OUT_OVERLAY_PLAY  0x00000008
#define MDP_REQ_DEBUG_OUT_KICKOFF_PIPE  0x00000010

#define MDP_REQ_PAN_DISPLAY_MAX_NUM     2
#define MDP_REQ_OVERLAY_SET_MAX_NUM     2
#define MDP_REQ_OVERLAY_PLAY_MAX_NUM    2
#define MDP_REQ_KICKOFF_PIPE_MAX_NUM    2
#define MDP_REQ_KICKOFF_PIPE_MAX_USED   4

struct st_mdp_req_time {
    unsigned long sec;
    unsigned long usec;
};

struct st_mdp_req_pipe {
    int flag;
    struct mdp4_overlay_pipe pipe[MDP_REQ_KICKOFF_PIPE_MAX_USED];
};

static int mdp_req_pan_display_num = 0;
static struct st_mdp_req_time mdp_req_pan_display_time[MDP_REQ_PAN_DISPLAY_MAX_NUM];
static struct fb_var_screeninfo mdp_req_pan_display_info[MDP_REQ_PAN_DISPLAY_MAX_NUM];
static int mdp_req_overlay_set_num = 0;
static struct st_mdp_req_time mdp_req_overlay_set_time[MDP_REQ_OVERLAY_SET_MAX_NUM];
static struct mdp_overlay mdp_req_overlay_set_info[MDP_REQ_OVERLAY_SET_MAX_NUM];
static int mdp_req_overlay_id_num = 0;
static int mdp_req_overlay_id_info[MDP_REQ_OVERLAY_SET_MAX_NUM];
static int mdp_req_overlay_play_num = 0;
static struct st_mdp_req_time mdp_req_overlay_play_time[MDP_REQ_OVERLAY_PLAY_MAX_NUM];
static struct msmfb_overlay_data mdp_req_overlay_play_info[MDP_REQ_OVERLAY_PLAY_MAX_NUM];
static int mdp_req_kickoff_pipe_num = 0;
static struct st_mdp_req_time mdp_req_kickoff_pipe_time[MDP_REQ_KICKOFF_PIPE_MAX_NUM];
static struct st_mdp_req_pipe mdp_req_kickoff_pipe_info[MDP_REQ_KICKOFF_PIPE_MAX_NUM];
static int mdp_req_kickoff_pipe_used[OVERLAY_PIPE_MAX - MDP_REQ_KICKOFF_PIPE_MAX_USED];
void mdp_req_debug_init(void);
void mdp_req_trace_dump(int mode);
#endif /* CONFIG_SHLCDC_BOARD */

static uint32	mdp_offset;
static uint32	mdp_count;

static char	debug_buf[MDP_DEBUG_BUF];

/*
 * MDP4
 *
 */

static int mdp_offset_open(struct inode *inode, struct file *file)
{
	/* non-seekable */
	file->f_mode &= ~(FMODE_LSEEK | FMODE_PREAD | FMODE_PWRITE);
	return 0;
}

static int mdp_offset_release(struct inode *inode, struct file *file)
{
	return 0;
}

static ssize_t mdp_offset_write(
	struct file *file,
	const char __user *buff,
	size_t count,
	loff_t *ppos)
{
	uint32 off, cnt;

	if (count >= sizeof(debug_buf))
		return -EFAULT;

	if (copy_from_user(debug_buf, buff, count))
		return -EFAULT;

	debug_buf[count] = 0;	/* end of string */

	sscanf(debug_buf, "%x %d", &off, &cnt);

	if (cnt <= 0)
		cnt = 1;

	mdp_offset = off;
	mdp_count = cnt;

	printk(KERN_INFO "%s: offset=%x cnt=%d\n", __func__,
				mdp_offset, mdp_count);

	return count;
}

static ssize_t mdp_offset_read(
	struct file *file,
	char __user *buff,
	size_t count,
	loff_t *ppos)
{
	int len = 0;


	if (*ppos)
		return 0;	/* the end */

	len = snprintf(debug_buf, sizeof(debug_buf), "0x%08x %d\n",
					mdp_offset, mdp_count);
	if (len < 0)
		return 0;

	if (copy_to_user(buff, debug_buf, len))
		return -EFAULT;

	*ppos += len;	/* increase offset */

	return len;
}

static const struct file_operations mdp_off_fops = {
	.open = mdp_offset_open,
	.release = mdp_offset_release,
	.read = mdp_offset_read,
	.write = mdp_offset_write,
};

static int mdp_reg_open(struct inode *inode, struct file *file)
{
	/* non-seekable */
	file->f_mode &= ~(FMODE_LSEEK | FMODE_PREAD | FMODE_PWRITE);
	return 0;
}

static int mdp_reg_release(struct inode *inode, struct file *file)
{
	return 0;
}

static ssize_t mdp_reg_write(
	struct file *file,
	const char __user *buff,
	size_t count,
	loff_t *ppos)
{
	uint32 off, data;
	int cnt;

	if (count >= sizeof(debug_buf))
		return -EFAULT;

	if (copy_from_user(debug_buf, buff, count))
		return -EFAULT;

	debug_buf[count] = 0;	/* end of string */

	cnt = sscanf(debug_buf, "%x %x", &off, &data);

	mdp_pipe_ctrl(MDP_CMD_BLOCK, MDP_BLOCK_POWER_ON, FALSE);
#ifdef CONFIG_SHLCDC_BOARD /*CUST_ID_00131*/
	mdp_clk_ctrl(1);
#endif
	outpdw(MDP_BASE + off, data);
#ifdef CONFIG_SHLCDC_BOARD /*CUST_ID_00131*/
	mdp_clk_ctrl(0);
#endif
	mdp_pipe_ctrl(MDP_CMD_BLOCK, MDP_BLOCK_POWER_OFF, FALSE);

	printk(KERN_INFO "%s: addr=%x data=%x\n", __func__, off, data);

	return count;
}

static ssize_t mdp_reg_read(
	struct file *file,
	char __user *buff,
	size_t count,
	loff_t *ppos)
{
	int len = 0;
	uint32 data;
	int i, j, off, dlen, num;
	char *bp, *cp;
	int tot = 0;


	if (*ppos)
		return 0;	/* the end */

	j = 0;
	num = 0;
	bp = debug_buf;
	cp = MDP_BASE + mdp_offset;
	dlen = sizeof(debug_buf);
	mdp_pipe_ctrl(MDP_CMD_BLOCK, MDP_BLOCK_POWER_ON, FALSE);
#ifdef CONFIG_SHLCDC_BOARD /*CUST_ID_00131*/
	mdp_clk_ctrl(1);
#endif
	while (j++ < 8) {
		len = snprintf(bp, dlen, "0x%08x: ", (int)cp);
		tot += len;
		bp += len;
		dlen -= len;
		off = 0;
		i = 0;
		while (i++ < 4) {
			data = inpdw(cp + off);
			len = snprintf(bp, dlen, "%08x ", data);
			tot += len;
			bp += len;
			dlen -= len;
			off += 4;
			num++;
			if (num >= mdp_count)
				break;
		}
		*bp++ = '\n';
		--dlen;
		tot++;
		cp += off;
		if (num >= mdp_count)
			break;
	}
#ifdef CONFIG_SHLCDC_BOARD /*CUST_ID_00131*/
	mdp_clk_ctrl(0);
#endif
	mdp_pipe_ctrl(MDP_CMD_BLOCK, MDP_BLOCK_POWER_OFF, FALSE);
	*bp = 0;
	tot++;

	if (copy_to_user(buff, debug_buf, tot))
		return -EFAULT;

	*ppos += tot;	/* increase offset */

	return tot;
}


static const struct file_operations mdp_reg_fops = {
	.open = mdp_reg_open,
	.release = mdp_reg_release,
	.read = mdp_reg_read,
	.write = mdp_reg_write,
};

#ifdef CONFIG_FB_MSM_MDP40
static int mdp_stat_open(struct inode *inode, struct file *file)
{
	/* non-seekable */
	file->f_mode &= ~(FMODE_LSEEK | FMODE_PREAD | FMODE_PWRITE);
	return 0;
}

static int mdp_stat_release(struct inode *inode, struct file *file)
{
	return 0;
}

static ssize_t mdp_stat_write(
	struct file *file,
	const char __user *buff,
	size_t count,
	loff_t *ppos)
{
	unsigned long flag;

	if (count > sizeof(debug_buf))
		return -EFAULT;

	spin_lock_irqsave(&mdp_spin_lock, flag);
	memset((char *)&mdp4_stat, 0 , sizeof(mdp4_stat));	/* reset */
	spin_unlock_irqrestore(&mdp_spin_lock, flag);

	return count;
}

static ssize_t mdp_stat_read(
	struct file *file,
	char __user *buff,
	size_t count,
	loff_t *ppos)
{
	int len = 0;
	int tot = 0;
	int dlen;
	char *bp;
#ifdef CONFIG_SHLCDC_BOARD /* CUST_ID_00124 */
	u64 dmap_ms, dmap_us;
#endif /* CONFIG_SHLCDC_BOARD */


	if (*ppos)
		return 0;	/* the end */

	bp = debug_buf;
	dlen = sizeof(debug_buf);

	len = snprintf(bp, dlen, "\nmdp:\n");
	bp += len;
	dlen -= len;

	len = snprintf(bp, dlen, "int_total: %08lu\t",
					mdp4_stat.intr_tot);
	bp += len;
	dlen -= len;

	len = snprintf(bp, dlen, "int_overlay0: %08lu\t",
					mdp4_stat.intr_overlay0);
	bp += len;
	dlen -= len;
	len = snprintf(bp, dlen, "int_overlay1: %08lu\n",
					mdp4_stat.intr_overlay1);
	bp += len;
	dlen -= len;
#ifdef CONFIG_SHLCDC_BOARD /* CUST_ID_00126 */
	len = snprintf(bp, dlen, "int_overlay2: %08lu\n",
					mdp4_stat.intr_overlay2);
#else
	len = snprintf(bp, dlen, "int_overlay1: %08lu\n",
					mdp4_stat.intr_overlay2);
#endif /* CONFIG_SHLCDC_BOARD */
	bp += len;
	dlen -= len;

	len = snprintf(bp, dlen, "int_dmap: %08lu\t",
					mdp4_stat.intr_dma_p);
	bp += len;
	dlen -= len;
	len = snprintf(bp, dlen, "int_dmas: %08lu\t",
					mdp4_stat.intr_dma_s);
	bp += len;
	dlen -= len;
	len = snprintf(bp, dlen, "int_dmae:  %08lu\n",
					mdp4_stat.intr_dma_e);
	bp += len;
	dlen -= len;

	len = snprintf(bp, dlen, "primary:   vsync: %08lu\t",
					mdp4_stat.intr_vsync_p);
	bp += len;
	dlen -= len;
	len = snprintf(bp, dlen, "underrun: %08lu\n",
					mdp4_stat.intr_underrun_p);
	bp += len;
	dlen -= len;
	len = snprintf(bp, dlen, "external:  vsync: %08lu\t",
					mdp4_stat.intr_vsync_e);
	bp += len;
	dlen -= len;
	len = snprintf(bp, dlen, "underrun: %08lu\n",
					mdp4_stat.intr_underrun_e);

	bp += len;
	dlen -= len;
	len = snprintf(bp, dlen, "histogram: %08lu\t",
					mdp4_stat.intr_histogram);
	bp += len;
	dlen -= len;
	len = snprintf(bp, dlen, "read_ptr: %08lu\n\n",
					mdp4_stat.intr_rdptr);
	bp += len;
	dlen -= len;
	len = snprintf(bp, dlen, "dsi:\n");
	bp += len;
	dlen -= len;
	len = snprintf(bp, dlen, "int_total: %08lu\tmdp_start: %08lu\n",
			mdp4_stat.intr_dsi, mdp4_stat.dsi_mdp_start);
	bp += len;
	dlen -= len;
	len = snprintf(bp, dlen, "int_cmd: %08lu\t",
					mdp4_stat.intr_dsi_cmd);

	bp += len;
	dlen -= len;
	len = snprintf(bp, dlen, "int_mdp: %08lu\t",
					mdp4_stat.intr_dsi_mdp);

	bp += len;
	dlen -= len;

	len = snprintf(bp, dlen, "int_err: %08lu\n",
					mdp4_stat.intr_dsi_err);

	bp += len;
	dlen -= len;
	len = snprintf(bp, dlen, "clk_on : %08lu\t",
					mdp4_stat.dsi_clk_on);

	bp += len;
	dlen -= len;
	len = snprintf(bp, dlen, "clk_off: %08lu\n\n",
					mdp4_stat.dsi_clk_off);

	bp += len;
	dlen -= len;
	len = snprintf(bp, dlen, "kickoff:\n");
	bp += len;
	dlen -= len;

	len = snprintf(bp, dlen, "overlay0: %08lu\t",
					mdp4_stat.kickoff_ov0);
	bp += len;
	dlen -= len;
	len = snprintf(bp, dlen, "dmap: %08lu\t",
					mdp4_stat.kickoff_dmap);
	bp += len;
	dlen -= len;
	len = snprintf(bp, dlen, "dmas: %08lu\n",
					mdp4_stat.kickoff_dmas);

	bp += len;
	dlen -= len;
	len = snprintf(bp, dlen, "overlay1: %08lu\t",
					mdp4_stat.kickoff_ov1);
	bp += len;
	dlen -= len;
	len = snprintf(bp, dlen, "dmae: %08lu\n\n",
					mdp4_stat.kickoff_dmae);

	bp += len;
	dlen -= len;

	len = snprintf(bp, dlen, "overlay0_play:\n");
	bp += len;
	dlen -= len;

	len = snprintf(bp, dlen, "set:   %08lu\t",
					mdp4_stat.overlay_set[0]);
	bp += len;
	dlen -= len;
	len = snprintf(bp, dlen, "unset: %08lu\t",
					mdp4_stat.overlay_unset[0]);
	bp += len;
	dlen -= len;
	len = snprintf(bp, dlen, "play:  %08lu\t",
					mdp4_stat.overlay_play[0]);
	bp += len;
	dlen -= len;
	len = snprintf(bp, dlen, "commit:  %08lu\n",
					mdp4_stat.overlay_commit[0]);
	bp += len;
	dlen -= len;

	len = snprintf(bp, dlen, "overlay1_play:\n");
	bp += len;
	dlen -= len;
	len = snprintf(bp, dlen, "set:   %08lu\t",
					mdp4_stat.overlay_set[1]);
	bp += len;
	dlen -= len;
	len = snprintf(bp, dlen, "unset: %08lu\t",
					mdp4_stat.overlay_unset[1]);
	bp += len;
	dlen -= len;
	len = snprintf(bp, dlen, "play:  %08lu\t",
					mdp4_stat.overlay_play[1]);

	bp += len;
	dlen -= len;
	len = snprintf(bp, dlen, "commit:  %08lu\n\n",
					mdp4_stat.overlay_commit[1]);
	bp += len;
	dlen -= len;

	len = snprintf(bp, dlen, "frame_push:\n");
	bp += len;
	dlen -= len;
	len = snprintf(bp, dlen, "vg1 :   %08lu\t", mdp4_stat.pipe[0]);
	bp += len;
	dlen -= len;
	len = snprintf(bp, dlen, "vg2 :   %08lu\t", mdp4_stat.pipe[1]);
	bp += len;
	dlen -= len;
	len = snprintf(bp, dlen, "vg3 :   %08lu\n", mdp4_stat.pipe[5]);
	bp += len;
	dlen -= len;
	len = snprintf(bp, dlen, "rgb1:   %08lu\t", mdp4_stat.pipe[2]);
	bp += len;
	dlen -= len;
	len = snprintf(bp, dlen, "rgb2:   %08lu\t", mdp4_stat.pipe[3]);
	bp += len;
	dlen -= len;
	len = snprintf(bp, dlen, "rgb3:   %08lu\n\n", mdp4_stat.pipe[4]);
	bp += len;
	dlen -= len;
	len = snprintf(bp, dlen, "wait4vsync: ");
	bp += len;
	dlen -= len;
	len = snprintf(bp, dlen, "mixer0 : %08lu\t", mdp4_stat.wait4vsync0);
	bp += len;
	dlen -= len;
	len = snprintf(bp, dlen, "mixer1: %08lu\n\n", mdp4_stat.wait4vsync1);
	bp += len;
	dlen -= len;
	len = snprintf(bp, dlen, "iommu: ");
	bp += len;
	dlen -= len;
	len = snprintf(bp, dlen, "map : %08lu\t", mdp4_stat.iommu_map);
	bp += len;
	dlen -= len;
	len = snprintf(bp, dlen, "unmap: %08lu\t", mdp4_stat.iommu_unmap);
	bp += len;
	dlen -= len;
	len = snprintf(bp, dlen, "drop: %08lu\n\n", mdp4_stat.iommu_drop);
	bp += len;
	dlen -= len;
	len = snprintf(bp, dlen, "err_mixer : %08lu\t", mdp4_stat.err_mixer);
	bp += len;
	dlen -= len;
	len = snprintf(bp, dlen, "err_size  : %08lu\n", mdp4_stat.err_size);
	bp += len;
	dlen -= len;
	len = snprintf(bp, dlen, "err_scale : %08lu\t", mdp4_stat.err_scale);
	bp += len;
	dlen -= len;
	len = snprintf(bp, dlen, "err_format: %08lu\n", mdp4_stat.err_format);
	bp += len;
	dlen -= len;
	len = snprintf(bp, dlen, "err_play  : %08lu\t", mdp4_stat.err_play);
	bp += len;
	dlen -= len;
	len = snprintf(bp, dlen, "err_stage : %08lu\n", mdp4_stat.err_stage);
	bp += len;
	dlen -= len;
	len = snprintf(bp, dlen, "err_underflow: %08lu\n\n",
		       mdp4_stat.err_underflow);
	bp += len;
	dlen -= len;

	len = snprintf(bp, dlen, "writeback:\n");
	bp += len;
	dlen -= len;

	len = snprintf(bp, dlen, "dsi_cmd: %08lu\t",
					mdp4_stat.blt_dsi_cmd);
	bp += len;
	dlen -= len;

	len = snprintf(bp, dlen, "dsi_video: %08lu\n",
					mdp4_stat.blt_dsi_video);
	bp += len;
	dlen -= len;

	len = snprintf(bp, dlen, "lcdc: %08lu\t",
					mdp4_stat.blt_lcdc);
	bp += len;
	dlen -= len;

	len = snprintf(bp, dlen, "dtv: %08lu\t",
					mdp4_stat.blt_dtv);
	bp += len;
	dlen -= len;

	len = snprintf(bp, dlen, "mddi: %08lu\n\n",
					mdp4_stat.blt_mddi);
	bp += len;
	dlen -= len;

#ifdef CONFIG_SHLCDC_BOARD /* CUST_ID_00128 */
	len = snprintf(bp, dlen, "Recovery:\n");
	bp += len;
	dlen -= len;

	len = snprintf(bp, dlen, "err_det: %08lu\t",
					mdp4_stat.err_det);
	bp += len;
	dlen -= len;

	len = snprintf(bp, dlen, "recovery: %08lu\n\n",
					mdp4_stat.recovery);
	bp += len;
	dlen -= len;
#endif /* CONFIG_SHLCDC_BOARD */

#ifdef CONFIG_SHLCDC_BOARD /* CUST_ID_00124 */
	dmap_ms = timespec_to_ns(&mdp4_dmap_max_time);
	do_div(dmap_ms, NSEC_PER_USEC);
	dmap_us = do_div(dmap_ms, USEC_PER_MSEC);

	len = snprintf(bp, dlen, "dmap: %lu.%03lums\n\n",
					(unsigned long)dmap_ms,
					(unsigned long)dmap_us);
	bp += len;
	dlen -= len;
#endif /* CONFIG_SHLCDC_BOARD */

	tot = (uint32)bp - (uint32)debug_buf;
	*bp = 0;
	tot++;

	if (tot < 0)
		return 0;
	if (copy_to_user(buff, debug_buf, tot))
		return -EFAULT;

	*ppos += tot;	/* increase offset */

	return tot;
}

static const struct file_operations mdp_stat_fops = {
	.open = mdp_stat_open,
	.release = mdp_stat_release,
	.read = mdp_stat_read,
	.write = mdp_stat_write,
};
#endif

/*
 * MDDI
 *
 */

struct mddi_reg {
	char *name;
	int off;
};

static struct mddi_reg mddi_regs_list[] = {
	{"MDDI_CMD", MDDI_CMD},	 	/* 0x0000 */
	{"MDDI_VERSION", MDDI_VERSION},  /* 0x0004 */
	{"MDDI_PRI_PTR", MDDI_PRI_PTR},  /* 0x0008 */
	{"MDDI_BPS",  MDDI_BPS}, 	/* 0x0010 */
	{"MDDI_SPM", MDDI_SPM}, 	/* 0x0014 */
	{"MDDI_INT", MDDI_INT}, 	/* 0x0018 */
	{"MDDI_INTEN", MDDI_INTEN},	/* 0x001c */
	{"MDDI_REV_PTR", MDDI_REV_PTR},	/* 0x0020 */
	{"MDDI_	REV_SIZE", MDDI_REV_SIZE},/* 0x0024 */
	{"MDDI_STAT", MDDI_STAT},	/* 0x0028 */
	{"MDDI_REV_RATE_DIV", MDDI_REV_RATE_DIV}, /* 0x002c */
	{"MDDI_REV_CRC_ERR", MDDI_REV_CRC_ERR}, /* 0x0030 */
	{"MDDI_TA1_LEN", MDDI_TA1_LEN}, /* 0x0034 */
	{"MDDI_TA2_LEN", MDDI_TA2_LEN}, /* 0x0038 */
	{"MDDI_TEST", MDDI_TEST}, 	/* 0x0040 */
	{"MDDI_REV_PKT_CNT", MDDI_REV_PKT_CNT}, /* 0x0044 */
	{"MDDI_DRIVE_HI", MDDI_DRIVE_HI},/* 0x0048 */
	{"MDDI_DRIVE_LO", MDDI_DRIVE_LO},	/* 0x004c */
	{"MDDI_DISP_WAKE", MDDI_DISP_WAKE},/* 0x0050 */
	{"MDDI_REV_ENCAP_SZ", MDDI_REV_ENCAP_SZ}, /* 0x0054 */
	{"MDDI_RTD_VAL", MDDI_RTD_VAL}, /* 0x0058 */
	{"MDDI_PAD_CTL", MDDI_PAD_CTL},	 /* 0x0068 */
	{"MDDI_DRIVER_START_CNT", MDDI_DRIVER_START_CNT}, /* 0x006c */
	{"MDDI_CORE_VER", MDDI_CORE_VER}, /* 0x008c */
	{"MDDI_FIFO_ALLOC", MDDI_FIFO_ALLOC}, /* 0x0090 */
	{"MDDI_PAD_IO_CTL", MDDI_PAD_IO_CTL}, /* 0x00a0 */
	{"MDDI_PAD_CAL", MDDI_PAD_CAL},  /* 0x00a4 */
	{0, 0}
};

static int mddi_reg_open(struct inode *inode, struct file *file)
{
	/* non-seekable */
	file->f_mode &= ~(FMODE_LSEEK | FMODE_PREAD | FMODE_PWRITE);
	return 0;
}

static int mddi_reg_release(struct inode *inode, struct file *file)
{
	return 0;
}

static void mddi_reg_write(int ndx, uint32 off, uint32 data)
{
	char *base;

	if (ndx)
		base = (char *)msm_emdh_base;
	else
		base = (char *)msm_pmdh_base;

	mdp_pipe_ctrl(MDP_CMD_BLOCK, MDP_BLOCK_POWER_ON, FALSE);
	writel(data, base + off);
	mdp_pipe_ctrl(MDP_CMD_BLOCK, MDP_BLOCK_POWER_OFF, FALSE);

	printk(KERN_INFO "%s: addr=%x data=%x\n",
			__func__, (int)(base+off), (int)data);
}

static int mddi_reg_read(int ndx)
{
	struct mddi_reg *reg;
	unsigned char *base;
	int data;
	char *bp;
	int len = 0;
	int tot = 0;
	int dlen;

	if (ndx)
		base = msm_emdh_base;
	else
		base = msm_pmdh_base;

	reg = mddi_regs_list;
	bp = debug_buf;
	dlen = sizeof(debug_buf);

	mdp_pipe_ctrl(MDP_CMD_BLOCK, MDP_BLOCK_POWER_ON, FALSE);
	while (reg->name) {
		data = readl((u32)base + reg->off);
		len = snprintf(bp, dlen, "%s:0x%08x\t\t= 0x%08x\n",
					reg->name, reg->off, data);
		tot += len;
		bp += len;
		dlen -= len;
		reg++;
	}
	mdp_pipe_ctrl(MDP_CMD_BLOCK, MDP_BLOCK_POWER_OFF, FALSE);
	*bp = 0;
	tot++;

	return tot;
}

static ssize_t pmdh_reg_write(
	struct file *file,
	const char __user *buff,
	size_t count,
	loff_t *ppos)
{
	uint32 off, data;
	int cnt;

	if (count >= sizeof(debug_buf))
		return -EFAULT;

	if (copy_from_user(debug_buf, buff, count))
		return -EFAULT;

	debug_buf[count] = 0;	/* end of string */

	cnt = sscanf(debug_buf, "%x %x", &off, &data);

	mddi_reg_write(0, off, data);

	return count;
}

static ssize_t pmdh_reg_read(
	struct file *file,
	char __user *buff,
	size_t count,
	loff_t *ppos)
{
	int tot = 0;

	if (*ppos)
		return 0;	/* the end */

	tot = mddi_reg_read(0);	/* pmdh */

	if (tot < 0)
		return 0;
	if (copy_to_user(buff, debug_buf, tot))
		return -EFAULT;

	*ppos += tot;	/* increase offset */

	return tot;
}


static const struct file_operations pmdh_fops = {
	.open = mddi_reg_open,
	.release = mddi_reg_release,
	.read = pmdh_reg_read,
	.write = pmdh_reg_write,
};



#if defined(CONFIG_FB_MSM_OVERLAY) && defined(CONFIG_FB_MSM_MDDI)
static int vsync_reg_open(struct inode *inode, struct file *file)
{
	/* non-seekable */
	file->f_mode &= ~(FMODE_LSEEK | FMODE_PREAD | FMODE_PWRITE);
	return 0;
}

static int vsync_reg_release(struct inode *inode, struct file *file)
{
	return 0;
}

static ssize_t vsync_reg_write(
	struct file *file,
	const char __user *buff,
	size_t count,
	loff_t *ppos)
{
	uint32 enable;
	int cnt;

	if (count >= sizeof(debug_buf))
		return -EFAULT;

	if (copy_from_user(debug_buf, buff, count))
		return -EFAULT;

	debug_buf[count] = 0;	/* end of string */

	cnt = sscanf(debug_buf, "%x", &enable);

	mdp_dmap_vsync_set(enable);

	return count;
}

static ssize_t vsync_reg_read(
	struct file *file,
	char __user *buff,
	size_t count,
	loff_t *ppos)
{
	char *bp;
	int len = 0;
	int tot = 0;
	int dlen;

	if (*ppos)
		return 0;	/* the end */

	bp = debug_buf;
	dlen = sizeof(debug_buf);
	len = snprintf(bp, dlen, "%x\n", mdp_dmap_vsync_get());
	tot += len;
	bp += len;
	*bp = 0;
	tot++;

	if (copy_to_user(buff, debug_buf, tot))
		return -EFAULT;

	*ppos += tot;	/* increase offset */

	return tot;
}


static const struct file_operations vsync_fops = {
	.open = vsync_reg_open,
	.release = vsync_reg_release,
	.read = vsync_reg_read,
	.write = vsync_reg_write,
};
#endif

static ssize_t emdh_reg_write(
	struct file *file,
	const char __user *buff,
	size_t count,
	loff_t *ppos)
{
	uint32 off, data;
	int cnt;

	if (count >= sizeof(debug_buf))
		return -EFAULT;

	if (copy_from_user(debug_buf, buff, count))
		return -EFAULT;

	debug_buf[count] = 0;	/* end of string */

	cnt = sscanf(debug_buf, "%x %x", &off, &data);

	mddi_reg_write(1, off, data);

	return count;
}

static ssize_t emdh_reg_read(
	struct file *file,
	char __user *buff,
	size_t count,
	loff_t *ppos)
{
	int tot = 0;

	if (*ppos)
		return 0;	/* the end */

	tot = mddi_reg_read(1);	/* emdh */

	if (tot < 0)
		return 0;
	if (copy_to_user(buff, debug_buf, tot))
		return -EFAULT;

	*ppos += tot;	/* increase offset */

	return tot;
}

static const struct file_operations emdh_fops = {
	.open = mddi_reg_open,
	.release = mddi_reg_release,
	.read = emdh_reg_read,
	.write = emdh_reg_write,
};


uint32 dbg_offset;
uint32 dbg_count;
char *dbg_base;


static int dbg_open(struct inode *inode, struct file *file)
{
	/* non-seekable */
	file->f_mode &= ~(FMODE_LSEEK | FMODE_PREAD | FMODE_PWRITE);
	return 0;
}

static int dbg_release(struct inode *inode, struct file *file)
{
	return 0;
}

static ssize_t dbg_base_write(
	struct file *file,
	const char __user *buff,
	size_t count,
	loff_t *ppos)
{
	return count;
}

static ssize_t dbg_base_read(
	struct file *file,
	char __user *buff,
	size_t count,
	loff_t *ppos)
{
	int len = 0;
	int tot = 0;
	int dlen;
	char *bp;


	if (*ppos)
		return 0;	/* the end */


	bp = debug_buf;
	dlen = sizeof(debug_buf);

	len = snprintf(bp, dlen, "mdp_base  :    %08x\n",
				(int)msm_mdp_base);
	bp += len;
	dlen -= len;
	len = snprintf(bp, dlen, "mddi_base :    %08x\n",
				(int)msm_pmdh_base);
	bp += len;
	dlen -= len;
	len = snprintf(bp, dlen, "emdh_base :    %08x\n",
				(int)msm_emdh_base);
	bp += len;
	dlen -= len;
#ifdef CONFIG_FB_MSM_TVOUT
	len = snprintf(bp, dlen, "tvenv_base:    %08x\n",
				(int)tvenc_base);
	bp += len;
	dlen -= len;
#endif

#ifdef CONFIG_FB_MSM_MIPI_DSI
	len = snprintf(bp, dlen, "mipi_dsi_base: %08x\n",
				(int)mipi_dsi_base);
	bp += len;
	dlen -= len;
#endif

	tot = (uint32)bp - (uint32)debug_buf;
	*bp = 0;
	tot++;

	if (tot < 0)
		return 0;
	if (copy_to_user(buff, debug_buf, tot))
		return -EFAULT;

	*ppos += tot;	/* increase offset */

	return tot;
}

static const struct file_operations dbg_base_fops = {
	.open = dbg_open,
	.release = dbg_release,
	.read = dbg_base_read,
	.write = dbg_base_write,
};

static ssize_t dbg_offset_write(
	struct file *file,
	const char __user *buff,
	size_t count,
	loff_t *ppos)
{
	uint32 off, cnt, num, base;

	if (count >= sizeof(debug_buf))
		return -EFAULT;

	if (copy_from_user(debug_buf, buff, count))
		return -EFAULT;

	debug_buf[count] = 0;	/* end of string */

	cnt = sscanf(debug_buf, "%x %d %x", &off, &num, &base);

	if (cnt < 0)
		cnt = 0;

	if (cnt >= 1)
		dbg_offset = off;
	if (cnt >= 2)
		dbg_count = num;
	if (cnt >= 3)
		dbg_base = (char *)base;

	printk(KERN_INFO "%s: offset=%x cnt=%d base=%x\n", __func__,
				dbg_offset, dbg_count, (int)dbg_base);

	return count;
}

static ssize_t dbg_offset_read(
	struct file *file,
	char __user *buff,
	size_t count,
	loff_t *ppos)
{
	int len = 0;


	if (*ppos)
		return 0;	/* the end */

	len = snprintf(debug_buf, sizeof(debug_buf), "0x%08x %d 0x%08x\n",
				dbg_offset, dbg_count, (int)dbg_base);
	if (len < 0)
		return 0;

	if (copy_to_user(buff, debug_buf, len))
		return -EFAULT;

	*ppos += len;	/* increase offset */

	return len;
}

static const struct file_operations dbg_off_fops = {
	.open = dbg_open,
	.release = dbg_release,
	.read = dbg_offset_read,
	.write = dbg_offset_write,
};


static ssize_t dbg_reg_write(
	struct file *file,
	const char __user *buff,
	size_t count,
	loff_t *ppos)
{
	uint32 off, data;
	int cnt;

	if (count >= sizeof(debug_buf))
		return -EFAULT;

	if (copy_from_user(debug_buf, buff, count))
		return -EFAULT;

	debug_buf[count] = 0;	/* end of string */

	cnt = sscanf(debug_buf, "%x %x", &off, &data);

	writel(data, dbg_base + off);

	printk(KERN_INFO "%s: addr=%x data=%x\n",
			__func__, (int)(dbg_base+off), (int)data);

	return count;
}

static ssize_t dbg_reg_read(
	struct file *file,
	char __user *buff,
	size_t count,
	loff_t *ppos)
{
	int len = 0;
	uint32 data;
	int i, j, off, dlen, num;
	char *bp, *cp;
	int tot = 0;


	if (*ppos)
		return 0;	/* the end */

	if (dbg_base == 0)
		return 0;	/* nothing to read */

	j = 0;
	num = 0;
	bp = debug_buf;
	cp = (char *)(dbg_base + dbg_offset);
	dlen = sizeof(debug_buf);
	while (j++ < 16) {
		len = snprintf(bp, dlen, "0x%08x: ", (int)cp);
		tot += len;
		bp += len;
		dlen -= len;
		off = 0;
		i = 0;
		while (i++ < 4) {
			data = readl(cp + off);
			len = snprintf(bp, dlen, "%08x ", data);
			tot += len;
			bp += len;
			dlen -= len;
			off += 4;
			num++;
			if (num >= dbg_count)
				break;
		}
		data = readl((u32)cp + off);
		*bp++ = '\n';
		--dlen;
		tot++;
		cp += off;
		if (num >= dbg_count)
			break;
	}
	*bp = 0;
	tot++;

	if (copy_to_user(buff, debug_buf, tot))
		return -EFAULT;

	*ppos += tot;	/* increase offset */

	return tot;
}


static const struct file_operations dbg_reg_fops = {
	.open = dbg_open,
	.release = dbg_release,
	.read = dbg_reg_read,
	.write = dbg_reg_write,
};

u32 dbg_force_ov0_blt;
u32 dbg_force_ov1_blt;

static ssize_t dbg_force_ov0_blt_read(
	struct file *file,
	char __user *buff,
	size_t count,
	loff_t *ppos) {
	int len;

	if (*ppos)
		return 0;

	len = snprintf(debug_buf, sizeof(debug_buf),
		       "%d\n", dbg_force_ov0_blt);

	if (len < 0)
		return 0;

	if (copy_to_user(buff, debug_buf, len))
		return -EFAULT;

	*ppos += len;

	return len;
}

static ssize_t dbg_force_ov0_blt_write(
	struct file *file,
	const char __user *buff,
	size_t count,
	loff_t *ppos)
{
	u32 cnt;

	if (count >= sizeof(debug_buf))
		return -EFAULT;

	if (copy_from_user(debug_buf, buff, count))
		return -EFAULT;

	debug_buf[count] = 0;	/* end of string */

	cnt = sscanf(debug_buf, "%x", &dbg_force_ov0_blt);

	pr_info("%s: dbg_force_ov0_blt = %x\n",
		__func__, dbg_force_ov0_blt);

	if ((dbg_force_ov0_blt & 0x0f) > 2)
		pr_err("%s: invalid dbg_force_ov0_blt = %d\n",
			__func__, dbg_force_ov0_blt);

	if ((dbg_force_ov0_blt >> 4) > 2)
		pr_err("%s: invalid dbg_force_ov0_blt = %d\n",
			__func__, dbg_force_ov0_blt);

	return count;
}

static const struct file_operations dbg_force_ov0_blt_fops = {
	.open = dbg_open,
	.release = dbg_release,
	.read = dbg_force_ov0_blt_read,
	.write = dbg_force_ov0_blt_write,
};

static ssize_t dbg_force_ov1_blt_read(
	struct file *file,
	char __user *buff,
	size_t count,
	loff_t *ppos) {
	int len;

	if (*ppos)
		return 0;

	len = snprintf(debug_buf, sizeof(debug_buf),
		       "%x\n", dbg_force_ov1_blt);

	if (len < 0)
		return 0;

	if (copy_to_user(buff, debug_buf, len))
		return -EFAULT;

	*ppos += len;

	return len;
}

static ssize_t dbg_force_ov1_blt_write(
	struct file *file,
	const char __user *buff,
	size_t count,
	loff_t *ppos)
{
	u32 cnt;

	if (count >= sizeof(debug_buf))
		return -EFAULT;

	if (copy_from_user(debug_buf, buff, count))
		return -EFAULT;

	debug_buf[count] = 0;	/* end of string */

	cnt = sscanf(debug_buf, "%x", &dbg_force_ov1_blt);

	pr_info("%s: dbg_force_ov1_blt = %x\n",
		__func__, dbg_force_ov1_blt);

	if ((dbg_force_ov1_blt & 0x0f) > 2)
		pr_err("%s: invalid dbg_force_ov1_blt = %x\n",
			__func__, dbg_force_ov1_blt);

	if ((dbg_force_ov1_blt >> 4) > 2)
		pr_err("%s: invalid dbg_force_ov1_blt = %d\n",
			__func__, dbg_force_ov1_blt);

	return count;
}

static const struct file_operations dbg_force_ov1_blt_fops = {
	.open = dbg_open,
	.release = dbg_release,
	.read = dbg_force_ov1_blt_read,
	.write = dbg_force_ov1_blt_write,
};

#ifdef CONFIG_FB_MSM_HDMI_MSM_PANEL
static uint32 hdmi_offset;
static uint32 hdmi_count;

static int hdmi_open(struct inode *inode, struct file *file)
{
	/* non-seekable */
	file->f_mode &= ~(FMODE_LSEEK | FMODE_PREAD | FMODE_PWRITE);
	return 0;
}

static int hdmi_release(struct inode *inode, struct file *file)
{
	return 0;
}

static ssize_t hdmi_offset_write(
	struct file *file,
	const char __user *buff,
	size_t count,
	loff_t *ppos)
{
	uint32 off, cnt, num;

	if (count >= sizeof(debug_buf))
		return -EFAULT;

	if (copy_from_user(debug_buf, buff, count))
		return -EFAULT;

	debug_buf[count] = 0;	/* end of string */

	cnt = sscanf(debug_buf, "%x %d", &off, &num);

	if (cnt < 0)
		cnt = 0;

	if (cnt >= 1)
		hdmi_offset = off;
	if (cnt >= 2)
		hdmi_count = num;

	printk(KERN_INFO "%s: offset=%x cnt=%d\n", __func__,
				hdmi_offset, hdmi_count);

	return count;
}

static ssize_t hdmi_offset_read(
	struct file *file,
	char __user *buff,
	size_t count,
	loff_t *ppos)
{
	int len = 0;


	if (*ppos)
		return 0;	/* the end */

	len = snprintf(debug_buf, sizeof(debug_buf), "0x%08x %d\n",
				hdmi_offset, hdmi_count);
	if (len < 0)
		return 0;

	if (copy_to_user(buff, debug_buf, len))
		return -EFAULT;

	*ppos += len;	/* increase offset */

	return len;
}

static const struct file_operations hdmi_off_fops = {
	.open = hdmi_open,
	.release = hdmi_release,
	.read = hdmi_offset_read,
	.write = hdmi_offset_write,
};


static ssize_t hdmi_reg_write(
	struct file *file,
	const char __user *buff,
	size_t count,
	loff_t *ppos)
{
	uint32 off, data, base;
	int cnt;

	if (count >= sizeof(debug_buf))
		return -EFAULT;

	if (copy_from_user(debug_buf, buff, count))
		return -EFAULT;

	base = hdmi_msm_get_io_base();
	if (base == 0)
		return -EFAULT;

	debug_buf[count] = 0;	/* end of string */

	cnt = sscanf(debug_buf, "%x %x", &off, &data);

	writel(data, base + off);

	printk(KERN_INFO "%s: addr=%x data=%x\n",
			__func__, (int)(base+off), (int)data);

	return count;
}

static ssize_t hdmi_reg_read(
	struct file *file,
	char __user *buff,
	size_t count,
	loff_t *ppos)
{
	int len = 0;
	uint32 data;
	int i, j, off, dlen, num;
	char *bp, *cp;
	int tot = 0;


	if (*ppos)
		return 0;	/* the end */

	if (hdmi_msm_get_io_base() == 0)
		return 0;	/* nothing to read */

	j = 0;
	num = 0;
	bp = debug_buf;
	cp = (char *)(hdmi_msm_get_io_base() + hdmi_offset);
	dlen = sizeof(debug_buf);
	while (j++ < 16) {
		len = snprintf(bp, dlen, "0x%08x: ", (int)cp);
		tot += len;
		bp += len;
		dlen -= len;
		off = 0;
		i = 0;
		while (i++ < 4) {
			data = readl(cp + off);
			len = snprintf(bp, dlen, "%08x ", data);
			tot += len;
			bp += len;
			dlen -= len;
			off += 4;
			num++;
			if (num >= hdmi_count)
				break;
		}
		data = readl((u32)cp + off);
		*bp++ = '\n';
		--dlen;
		tot++;
		cp += off;
		if (num >= hdmi_count)
			break;
	}
	*bp = 0;
	tot++;

	if (copy_to_user(buff, debug_buf, tot))
		return -EFAULT;

	*ppos += tot;	/* increase offset */

	return tot;
}


static const struct file_operations hdmi_reg_fops = {
	.open = hdmi_open,
	.release = hdmi_release,
	.read = hdmi_reg_read,
	.write = hdmi_reg_write,
};
#endif

#ifdef CONFIG_SHLCDC_BOARD /* CUST_ID_00039 */
static uint32 dsi_offset;
static uint32 dsi_count;

static int dsi_open(struct inode *inode, struct file *file)
{
	/* non-seekable */
	file->f_mode &= ~(FMODE_LSEEK | FMODE_PREAD | FMODE_PWRITE);
	return 0;
}

static int dsi_release(struct inode *inode, struct file *file)
{
	return 0;
}

static ssize_t dsi_offset_write(
	struct file *file,
	const char __user *buff,
	size_t count,
	loff_t *ppos)
{
	uint32 off, cnt, num;

	if (count >= sizeof(debug_buf))
		return -EFAULT;

	if (copy_from_user(debug_buf, buff, count))
		return -EFAULT;

	debug_buf[count] = 0;	/* end of string */

	cnt = sscanf(debug_buf, "%x %d", &off, &num);

	if (cnt < 0)
		cnt = 0;

	if (cnt >= 1)
		dsi_offset = off;
	if (cnt >= 2)
		dsi_count = num;

	printk(KERN_INFO "%s: offset=%x cnt=%d\n", __func__,
				dsi_offset, dsi_count);

	return count;
}

static ssize_t dsi_offset_read(
	struct file *file,
	char __user *buff,
	size_t count,
	loff_t *ppos)
{
	int len = 0;


	if (*ppos)
		return 0;	/* the end */

	len = snprintf(debug_buf, sizeof(debug_buf), "0x%08x %d\n",
				dsi_offset, dsi_count);
	if (len < 0)
		return 0;

	if (copy_to_user(buff, debug_buf, len))
		return -EFAULT;

	*ppos += len;	/* increase offset */

	return len;
}

static const struct file_operations dsi_off_fops = {
	.open = dsi_open,
	.release = dsi_release,
	.read = dsi_offset_read,
	.write = dsi_offset_write,
};


static ssize_t dsi_reg_write(
	struct file *file,
	const char __user *buff,
	size_t count,
	loff_t *ppos)
{
	uint32 off, data;
	int cnt;

	if (count >= sizeof(debug_buf))
		return -EFAULT;

	if (copy_from_user(debug_buf, buff, count))
		return -EFAULT;

	debug_buf[count] = 0;	/* end of string */

	cnt = sscanf(debug_buf, "%x %x", &off, &data);

	writel(data, MIPI_DSI_BASE + off);

	printk(KERN_INFO "%s: addr=%x data=%x\n",
			__func__, (int)(MIPI_DSI_BASE + off), (int)data);

	return count;
}

static ssize_t dsi_reg_read(
	struct file *file,
	char __user *buff,
	size_t count,
	loff_t *ppos)
{
	int len = 0;
	uint32 data;
	int i, j, off, dlen, num;
	char *bp, *cp;
	int tot = 0;


	if (*ppos)
		return 0;	/* the end */

	j = 0;
	num = 0;
	bp = debug_buf;
	cp = (char *)(MIPI_DSI_BASE + dsi_offset);
	dlen = sizeof(debug_buf);
	while (j++ < 16) {
		len = snprintf(bp, dlen, "0x%08x: ", (int)cp);
		tot += len;
		bp += len;
		dlen -= len;
		off = 0;
		i = 0;
		while (i++ < 4) {
			data = readl(cp + off);
			len = snprintf(bp, dlen, "%08x ", data);
			tot += len;
			bp += len;
			dlen -= len;
			off += 4;
			num++;
			if (num >= dsi_count)
				break;
		}
		data = readl((u32)cp + off);
		*bp++ = '\n';
		--dlen;
		tot++;
		cp += off;
		if (num >= dsi_count)
			break;
	}
	*bp = 0;
	tot++;

	if (copy_to_user(buff, debug_buf, tot))
		return -EFAULT;

	*ppos += tot;	/* increase offset */

	return tot;
}


static const struct file_operations dsi_reg_fops = {
	.open = dsi_open,
	.release = dsi_release,
	.read = dsi_reg_read,
	.write = dsi_reg_write,
};
#endif /* CONFIG_SHLCDC_BOARD */

#ifdef CONFIG_SHLCDC_BOARD /* CUST_ID_00095 */
static int mdp_trace_open(struct inode *inode, struct file *file)
{
	file->f_mode &= ~(FMODE_LSEEK | FMODE_PREAD | FMODE_PWRITE);
	return 0;
}

static int mdp_trace_release(struct inode *inode, struct file *file)
{
	return 0;
}

static ssize_t mdp_trace_write(
	struct file *file,
	const char __user *buff,
	size_t count,
	loff_t *ppos)
{
	return count;
}

static ssize_t mdp_trace_read(
	struct file *file,
	char __user *buff,
	size_t count,
	loff_t *ppos)
{
	mdp_req_trace_dump(0x1F);
	return 0;
}

static const struct file_operations mdp_trace_fops = {
	.open = mdp_trace_open,
	.release = mdp_trace_release,
	.read = mdp_trace_read,
	.write = mdp_trace_write,
};
#endif /* CONFIG_SHLCDC_BOARD */

/*
 * debugfs
 *
 */

int mdp_debugfs_init(void)
{
	struct dentry *dent = debugfs_create_dir("mdp", NULL);

	if (IS_ERR(dent)) {
		printk(KERN_ERR "%s(%d): debugfs_create_dir fail, error %ld\n",
			__FILE__, __LINE__, PTR_ERR(dent));
		return -1;
	}

	if (debugfs_create_file("off", 0644, dent, 0, &mdp_off_fops)
			== NULL) {
		printk(KERN_ERR "%s(%d): debugfs_create_file: index fail\n",
			__FILE__, __LINE__);
		return -1;
	}

	if (debugfs_create_file("reg", 0644, dent, 0, &mdp_reg_fops)
			== NULL) {
		printk(KERN_ERR "%s(%d): debugfs_create_file: debug fail\n",
			__FILE__, __LINE__);
		return -1;
	}

#ifdef CONFIG_FB_MSM_MDP40
	if (debugfs_create_file("stat", 0644, dent, 0, &mdp_stat_fops)
			== NULL) {
		printk(KERN_ERR "%s(%d): debugfs_create_file: debug fail\n",
			__FILE__, __LINE__);
		return -1;
	}
#endif

#ifdef CONFIG_SHLCDC_BOARD /* CUST_ID_00095 */
	if (debugfs_create_file("trace", 0644, dent, 0, &mdp_trace_fops)
			== NULL) {
		printk(KERN_ERR "%s(%d): debugfs_create_file: trace fail\n",
			__FILE__, __LINE__);
		return -1;
	}
#endif /* CONFIG_SHLCDC_BOARD */

	if (debugfs_create_file("force_ov0_blt", 0644, dent, 0,
				&dbg_force_ov0_blt_fops)
			== NULL) {
		pr_err("%s(%d): debugfs_create_file: debug fail\n",
			__FILE__, __LINE__);
		return -EFAULT;
	}

	if (debugfs_create_file("force_ov1_blt", 0644, dent, 0,
				&dbg_force_ov1_blt_fops)
			== NULL) {
		pr_err("%s(%d): debugfs_create_file: debug fail\n",
			__FILE__, __LINE__);
		return -EFAULT;
	}

	dent = debugfs_create_dir("mddi", NULL);

	if (IS_ERR(dent)) {
		printk(KERN_ERR "%s(%d): debugfs_create_dir fail, error %ld\n",
			__FILE__, __LINE__, PTR_ERR(dent));
		return -1;
	}

	if (debugfs_create_file("reg", 0644, dent, 0, &pmdh_fops)
			== NULL) {
		printk(KERN_ERR "%s(%d): debugfs_create_file: debug fail\n",
			__FILE__, __LINE__);
		return -1;
	}

#if defined(CONFIG_FB_MSM_OVERLAY) && defined(CONFIG_FB_MSM_MDDI)
	if (debugfs_create_file("vsync", 0644, dent, 0, &vsync_fops)
			== NULL) {
		printk(KERN_ERR "%s(%d): debugfs_create_file: debug fail\n",
			__FILE__, __LINE__);
		return -1;
	}
#endif

	dent = debugfs_create_dir("emdh", NULL);

	if (IS_ERR(dent)) {
		printk(KERN_ERR "%s(%d): debugfs_create_dir fail, error %ld\n",
			__FILE__, __LINE__, PTR_ERR(dent));
		return -1;
	}

	if (debugfs_create_file("reg", 0644, dent, 0, &emdh_fops)
			== NULL) {
		printk(KERN_ERR "%s(%d): debugfs_create_file: debug fail\n",
			__FILE__, __LINE__);
		return -1;
	}

	dent = debugfs_create_dir("mdp-dbg", NULL);

	if (IS_ERR(dent)) {
		printk(KERN_ERR "%s(%d): debugfs_create_dir fail, error %ld\n",
			__FILE__, __LINE__, PTR_ERR(dent));
		return -1;
	}

	if (debugfs_create_file("base", 0644, dent, 0, &dbg_base_fops)
			== NULL) {
		printk(KERN_ERR "%s(%d): debugfs_create_file: index fail\n",
			__FILE__, __LINE__);
		return -1;
	}

	if (debugfs_create_file("off", 0644, dent, 0, &dbg_off_fops)
			== NULL) {
		printk(KERN_ERR "%s(%d): debugfs_create_file: index fail\n",
			__FILE__, __LINE__);
		return -1;
	}

	if (debugfs_create_file("reg", 0644, dent, 0, &dbg_reg_fops)
			== NULL) {
		printk(KERN_ERR "%s(%d): debugfs_create_file: debug fail\n",
			__FILE__, __LINE__);
		return -1;
	}

#ifdef CONFIG_FB_MSM_HDMI_MSM_PANEL
	dent = debugfs_create_dir("hdmi", NULL);

	if (IS_ERR(dent)) {
		printk(KERN_ERR "%s(%d): debugfs_create_dir fail, error %ld\n",
			__FILE__, __LINE__, PTR_ERR(dent));
		return PTR_ERR(dent);
	}

	if (debugfs_create_file("off", 0644, dent, 0, &hdmi_off_fops)
			== NULL) {
		printk(KERN_ERR "%s(%d): debugfs_create_file: 'off' fail\n",
			__FILE__, __LINE__);
		return -ENOENT;
	}

	if (debugfs_create_file("reg", 0644, dent, 0, &hdmi_reg_fops)
			== NULL) {
		printk(KERN_ERR "%s(%d): debugfs_create_file: 'reg' fail\n",
			__FILE__, __LINE__);
		return -ENOENT;
	}
#endif

#ifdef CONFIG_SHLCDC_BOARD /* CUST_ID_00039 */
	dent = debugfs_create_dir("dsi", NULL);

	if (IS_ERR(dent)) {
		printk(KERN_ERR "%s(%d): debugfs_create_dir fail, error %ld\n",
			__FILE__, __LINE__, PTR_ERR(dent));
		return PTR_ERR(dent);
	}

	if (debugfs_create_file("off", 0644, dent, 0, &dsi_off_fops)
			== NULL) {
		printk(KERN_ERR "%s(%d): debugfs_create_file: 'off' fail\n",
			__FILE__, __LINE__);
		return -ENOENT;
	}

	if (debugfs_create_file("reg", 0644, dent, 0, &dsi_reg_fops)
			== NULL) {
		printk(KERN_ERR "%s(%d): debugfs_create_file: 'reg' fail\n",
			__FILE__, __LINE__);
		return -ENOENT;
	}
#endif /* CONFIG_SHLCDC_BOARD */

#ifdef CONFIG_SHLCDC_BOARD /* CUST_ID_00095 */
	mdp_req_debug_init();
#endif /* CONFIG_SHLCDC_BOARD */

	return 0;
}
#ifdef CONFIG_SHLCDC_BOARD /* CUST_ID_00095 */
void mdp_req_debug_init(void)
{
    int i;
    
    mdp_req_pan_display_num = 0;
    for (i=0; i<MDP_REQ_PAN_DISPLAY_MAX_NUM; i++) {
        memset(&mdp_req_pan_display_time[i], 0x00, sizeof(struct st_mdp_req_time));
        memset(&mdp_req_pan_display_info[i], 0x00, sizeof(struct fb_var_screeninfo));
    }
    mdp_req_overlay_set_num = 0;
    mdp_req_overlay_id_num = 0;
    for (i=0; i<MDP_REQ_OVERLAY_SET_MAX_NUM; i++) {
        memset(&mdp_req_overlay_set_time[i], 0x00, sizeof(struct st_mdp_req_time));
        memset(&mdp_req_overlay_set_info[i], 0x00, sizeof(struct mdp_overlay));
        mdp_req_overlay_id_info[i] = 0;
    }
    mdp_req_overlay_play_num = 0;
    for (i=0; i<MDP_REQ_OVERLAY_PLAY_MAX_NUM; i++) {
        memset(&mdp_req_overlay_play_time[i], 0x00, sizeof(struct st_mdp_req_time));
        memset(&mdp_req_overlay_play_info[i], 0x00, sizeof(struct msmfb_overlay_data));
    }
    mdp_req_kickoff_pipe_num = 0;
    for (i=0; i<MDP_REQ_KICKOFF_PIPE_MAX_NUM; i++) {
        memset(&mdp_req_kickoff_pipe_time[i], 0x00, sizeof(struct st_mdp_req_time));
        memset(&mdp_req_kickoff_pipe_info[i], 0x00, sizeof(struct st_mdp_req_pipe));
    }
    for (i=MDP_REQ_KICKOFF_PIPE_MAX_USED; i<OVERLAY_PIPE_MAX; i++) {
        mdp_req_kickoff_pipe_used[i-MDP_REQ_KICKOFF_PIPE_MAX_USED] = 0;
    }
}

void mdp_req_pan_display_set_debug(struct fb_var_screeninfo *var)
{
    unsigned long long t;
    unsigned long nanosec_rem;
    int idx;
    
    t = cpu_clock(smp_processor_id());
    nanosec_rem = do_div(t, 1000000000);
    idx = mdp_req_pan_display_num;
    mdp_req_pan_display_num ++;
    if(mdp_req_pan_display_num >= MDP_REQ_PAN_DISPLAY_MAX_NUM) {
        mdp_req_pan_display_num = 0;
    }
    
    mdp_req_pan_display_time[idx].sec = (unsigned long)t;
    mdp_req_pan_display_time[idx].usec = (unsigned long)(nanosec_rem / 1000);
    
    memcpy(&mdp_req_pan_display_info[idx], var, sizeof(struct fb_var_screeninfo));
}

void mdp_req_overlay_set_debug(struct mdp_overlay *req)
{
    unsigned long long t;
    unsigned long nanosec_rem;
    int idx;
    
    t = cpu_clock(smp_processor_id());
    nanosec_rem = do_div(t, 1000000000);
    idx = mdp_req_overlay_set_num;
    mdp_req_overlay_set_num ++;
    if(mdp_req_overlay_set_num >= MDP_REQ_OVERLAY_SET_MAX_NUM) {
        mdp_req_overlay_set_num = 0;
    }
    
    mdp_req_overlay_set_time[idx].sec = (unsigned long)t;
    mdp_req_overlay_set_time[idx].usec = (unsigned long)(nanosec_rem / 1000);
    
    memcpy(&mdp_req_overlay_set_info[idx], req, sizeof(struct mdp_overlay));
}

void mdp_req_overlay_id_debug(int id)
{
    int idx;
    
    idx = mdp_req_overlay_id_num;
    mdp_req_overlay_id_num ++;
    if(mdp_req_overlay_id_num >= MDP_REQ_OVERLAY_SET_MAX_NUM) {
        mdp_req_overlay_id_num = 0;
    }
    
    mdp_req_overlay_id_info[idx] = id;
}

void mdp_req_overlay_play_debug(struct msmfb_overlay_data *req)
{
    unsigned long long t;
    unsigned long nanosec_rem;
    int idx;
    
    t = cpu_clock(smp_processor_id());
    nanosec_rem = do_div(t, 1000000000);
    idx = mdp_req_overlay_play_num;
    mdp_req_overlay_play_num ++;
    if(mdp_req_overlay_play_num >= MDP_REQ_OVERLAY_PLAY_MAX_NUM) {
        mdp_req_overlay_play_num = 0;
    }
    
    mdp_req_overlay_play_time[idx].sec = (unsigned long)t;
    mdp_req_overlay_play_time[idx].usec = (unsigned long)(nanosec_rem / 1000);
    
    memcpy(&mdp_req_overlay_play_info[idx], req, sizeof(struct msmfb_overlay_data));
}

void mdp_req_kickoff_pipe_debug(int flag)
{
    unsigned long long t;
    unsigned long nanosec_rem;
    struct mdp4_overlay_pipe *pipe;
    int idx;
    int i;
    
    t = cpu_clock(smp_processor_id());
    nanosec_rem = do_div(t, 1000000000);
    idx = mdp_req_kickoff_pipe_num;
    mdp_req_kickoff_pipe_num ++;
    if(mdp_req_kickoff_pipe_num >= MDP_REQ_KICKOFF_PIPE_MAX_NUM) {
        mdp_req_kickoff_pipe_num = 0;
    }
    
    mdp_req_kickoff_pipe_time[idx].sec = (unsigned long)t;
    mdp_req_kickoff_pipe_time[idx].usec = (unsigned long)(nanosec_rem / 1000);
    
    mdp_req_kickoff_pipe_info[idx].flag = flag;
    for (i=0; i<MDP_REQ_KICKOFF_PIPE_MAX_USED; i++) {
        pipe = mdp4_overlay_ndx2pipe(i+1);
        if (!pipe) {
            mdp_req_kickoff_pipe_info[idx].pipe[i].pipe_used = 0;
        } else if (pipe->pipe_used == 0) {
            mdp_req_kickoff_pipe_info[idx].pipe[i].pipe_used = 0;
        } else {
            memcpy(&mdp_req_kickoff_pipe_info[idx].pipe[i], pipe, sizeof(struct mdp4_overlay_pipe));
        }
    }
    for (i=MDP_REQ_KICKOFF_PIPE_MAX_USED; i<OVERLAY_PIPE_MAX; i++) {
        pipe = mdp4_overlay_ndx2pipe(i+1);
        if (!pipe) {
            mdp_req_kickoff_pipe_used[i-MDP_REQ_KICKOFF_PIPE_MAX_USED] = 0;
        } else if (pipe->pipe_used == 0) {
            mdp_req_kickoff_pipe_used[i-MDP_REQ_KICKOFF_PIPE_MAX_USED] = 0;
        } else {
            mdp_req_kickoff_pipe_used[i-MDP_REQ_KICKOFF_PIPE_MAX_USED] = pipe->pipe_used;
        }
    }
}

static void mdp_req_stat_dump(void)
{
    u64 dmap_ms, dmap_us;

    pr_err("[***]stat\n");
    pr_err("[ nmdp - 1   ] int_total=%lu,int_overlay0=%lu,int_overlay1=%lu,int_overlay2=%lu,int_dmap=%lu,int_dmas=%lu,int_dmae=%lu\n",
            (unsigned long)mdp4_stat.intr_tot,
            (unsigned long)mdp4_stat.intr_overlay0,
            (unsigned long)mdp4_stat.intr_overlay1,
            (unsigned long)mdp4_stat.intr_overlay2,
            (unsigned long)mdp4_stat.intr_dma_p,
            (unsigned long)mdp4_stat.intr_dma_s,
            (unsigned long)mdp4_stat.intr_dma_e
    );
    pr_err("[ nmdp - 2   ] primary=(vsync=%lu,underrun=%lu),external=(vsync=%lu,underrun=%lu),histogram=%lu,read_ptr=%lu\n",
            (unsigned long)mdp4_stat.intr_vsync_p,
            (unsigned long)mdp4_stat.intr_underrun_p,
            (unsigned long)mdp4_stat.intr_vsync_e,
            (unsigned long)mdp4_stat.intr_underrun_e,
            (unsigned long)mdp4_stat.intr_histogram,
            (unsigned long)mdp4_stat.intr_rdptr
    );
    pr_err("[ dsi        ] int_total=%lu,mdp_start=%lu,int_cmd=%lu,int_mdp=%lu,int_err=%lu,clk_on=%lu,clk_off=%lu\n",
            (unsigned long)mdp4_stat.intr_dsi,
            (unsigned long)mdp4_stat.dsi_mdp_start,
            (unsigned long)mdp4_stat.intr_dsi_cmd,
            (unsigned long)mdp4_stat.intr_dsi_mdp,
            (unsigned long)mdp4_stat.intr_dsi_err,
            (unsigned long)mdp4_stat.dsi_clk_on,
            (unsigned long)mdp4_stat.dsi_clk_off
    );
    pr_err("[ kickoff    ] overlay0=%lu,dmap=%lu,dmas=%lu,overlay1=%lu,dmae=%lu\n",
            (unsigned long)mdp4_stat.kickoff_ov0,
            (unsigned long)mdp4_stat.kickoff_dmap,
            (unsigned long)mdp4_stat.kickoff_dmas,
            (unsigned long)mdp4_stat.kickoff_ov1,
            (unsigned long)mdp4_stat.kickoff_dmae
    );
    pr_err("[ overlay0   ] set=%lu,unset=%lu,play=%lu,commit=%lu\n",
            (unsigned long)mdp4_stat.overlay_set[0],
            (unsigned long)mdp4_stat.overlay_unset[0],
            (unsigned long)mdp4_stat.overlay_play[0],
            (unsigned long)mdp4_stat.overlay_commit[0]
    );
    pr_err("[ overlay1   ] set=%lu,unset=%lu,play=%lu,commit=%lu\n",
            (unsigned long)mdp4_stat.overlay_set[1],
            (unsigned long)mdp4_stat.overlay_unset[1],
            (unsigned long)mdp4_stat.overlay_play[1],
            (unsigned long)mdp4_stat.overlay_commit[1]
    );
    pr_err("[ frame_push ] vg1=%lu,vg2=%lu,vg3=%lu,rgb1=%lu,rgb2=%lu,rgb3=%lu\n",
            (unsigned long)mdp4_stat.pipe[0],
            (unsigned long)mdp4_stat.pipe[1],
            (unsigned long)mdp4_stat.pipe[5],
            (unsigned long)mdp4_stat.pipe[2],
            (unsigned long)mdp4_stat.pipe[3],
            (unsigned long)mdp4_stat.pipe[4]
    );
    pr_err("[ wait4vsync ] mixer0=%lu,mixer1=%lu\n",
            (unsigned long)mdp4_stat.wait4vsync0,
            (unsigned long)mdp4_stat.wait4vsync1
    );
    pr_err("[ iommu      ] map=%lu,unmap=%lu,drop=%lu\n",
            (unsigned long)mdp4_stat.iommu_map,
            (unsigned long)mdp4_stat.iommu_unmap,
            (unsigned long)mdp4_stat.iommu_drop
    );
    pr_err("[ error      ] err_mixer=%lu,err_zorder=%lu,err_size=%lu,err_scale=%lu,err_format=%lu,err_play=%lu,err_stage=%lu,err_underflow=%lu\n",
            (unsigned long)mdp4_stat.err_mixer,
            (unsigned long)mdp4_stat.err_zorder,
            (unsigned long)mdp4_stat.err_size,
            (unsigned long)mdp4_stat.err_scale,
            (unsigned long)mdp4_stat.err_format,
            (unsigned long)mdp4_stat.err_play,
            (unsigned long)mdp4_stat.err_stage,
            (unsigned long)mdp4_stat.err_underflow
    );
    pr_err("[ writeback  ] dsi_cmd=%lu,dsi_video=%lu,lcdc=%lu,dtv=%lu,mddi=%lu\n",
            (unsigned long)mdp4_stat.blt_dsi_cmd,
            (unsigned long)mdp4_stat.blt_dsi_video,
            (unsigned long)mdp4_stat.blt_lcdc,
            (unsigned long)mdp4_stat.blt_dtv,
            (unsigned long)mdp4_stat.blt_mddi
    );
    pr_err("[ Recovery   ] err_det=%lu,recovery=%lu\n",
            (unsigned long)mdp4_stat.err_det,
            (unsigned long)mdp4_stat.recovery
    );

    dmap_ms = timespec_to_ns(&mdp4_dmap_max_time);
    do_div(dmap_ms, NSEC_PER_USEC);
    dmap_us = do_div(dmap_ms, USEC_PER_MSEC);
    
    pr_err("[ max_time   ] dmap=%lu.%03lums\n",
            (unsigned long)dmap_ms,
            (unsigned long)dmap_us
    );
}

static void mdp_req_pan_display_dump(void)
{
    int i;
    
    pr_err("[***]pan_display(num=%d)\n", mdp_req_pan_display_num);
    for (i=0; i<MDP_REQ_PAN_DISPLAY_MAX_NUM; i++) {
        pr_err("[%5lu.%06lu] x=%d,y=%d,virtual=(x=%d,y=%d),offset=(x=%d,y=%d),bits_per_pixel=%d,grayscale=%d,nonstd=%d,activate=%d,height=%d,width=%d,accel_flags=%d\n",
                mdp_req_pan_display_time[i].sec,
                mdp_req_pan_display_time[i].usec,
                (int)mdp_req_pan_display_info[i].xres,
                (int)mdp_req_pan_display_info[i].yres,
                (int)mdp_req_pan_display_info[i].xres_virtual,
                (int)mdp_req_pan_display_info[i].yres_virtual,
                (int)mdp_req_pan_display_info[i].xoffset,
                (int)mdp_req_pan_display_info[i].yoffset,
                (int)mdp_req_pan_display_info[i].bits_per_pixel,
                (int)mdp_req_pan_display_info[i].grayscale,
                (int)mdp_req_pan_display_info[i].nonstd,
                (int)mdp_req_pan_display_info[i].activate,
                (int)mdp_req_pan_display_info[i].height,
                (int)mdp_req_pan_display_info[i].width,
                (int)mdp_req_pan_display_info[i].accel_flags
        );
        pr_err("[------------] pixclock=%d,left_margin=%d,right_margin=%d,upper_margin=%d,lower_margin=%d,hsync_len=%d,vsync_len=%d,sync=%d,vmode=%d,rotate=%d,reserved=[%d,%d,%d,%d,%d]\n",
                (int)mdp_req_pan_display_info[i].pixclock,
                (int)mdp_req_pan_display_info[i].left_margin,
                (int)mdp_req_pan_display_info[i].right_margin,
                (int)mdp_req_pan_display_info[i].upper_margin,
                (int)mdp_req_pan_display_info[i].lower_margin,
                (int)mdp_req_pan_display_info[i].hsync_len,
                (int)mdp_req_pan_display_info[i].vsync_len,
                (int)mdp_req_pan_display_info[i].sync,
                (int)mdp_req_pan_display_info[i].vmode,
                (int)mdp_req_pan_display_info[i].rotate,
                (int)mdp_req_pan_display_info[i].reserved[0],
                (int)mdp_req_pan_display_info[i].reserved[1],
                (int)mdp_req_pan_display_info[i].reserved[2],
                (int)mdp_req_pan_display_info[i].reserved[3],
                (int)mdp_req_pan_display_info[i].reserved[4]
        );
    }
}

static void mdp_req_overlay_set_dump(void)
{
    int i;
    
    pr_err("[***]overlay_set(set_num=%d, id_num=%d)\n", mdp_req_overlay_set_num, mdp_req_overlay_id_num);
    for (i=0; i<MDP_REQ_OVERLAY_SET_MAX_NUM; i++) {
        pr_err("[%5lu.%06lu] id=%d(%d),src=(w=%d,h=%d,f=%d),src_rect=(x=%d,y=%d,w=%d,h=%d),dst_rect=(x=%d,y=%d,w=%d,h=%d)\n",
                mdp_req_overlay_set_time[i].sec,
                mdp_req_overlay_set_time[i].usec,
                (int)mdp_req_overlay_set_info[i].id,
                (int)mdp_req_overlay_id_info[i],
                (int)mdp_req_overlay_set_info[i].src.width,
                (int)mdp_req_overlay_set_info[i].src.height,
                (int)mdp_req_overlay_set_info[i].src.format,
                (int)mdp_req_overlay_set_info[i].src_rect.x,
                (int)mdp_req_overlay_set_info[i].src_rect.y,
                (int)mdp_req_overlay_set_info[i].src_rect.w,
                (int)mdp_req_overlay_set_info[i].src_rect.h,
                (int)mdp_req_overlay_set_info[i].dst_rect.x,
                (int)mdp_req_overlay_set_info[i].dst_rect.y,
                (int)mdp_req_overlay_set_info[i].dst_rect.w,
                (int)mdp_req_overlay_set_info[i].dst_rect.h
        );
        pr_err("[------------] z_order=%d,is_fg=%d,alpha=%d,transp_mask=%d,flags=0x%lx\n",
                (int)mdp_req_overlay_set_info[i].z_order,
                (int)mdp_req_overlay_set_info[i].is_fg,
                (int)mdp_req_overlay_set_info[i].alpha,
                (int)mdp_req_overlay_set_info[i].transp_mask,
                (unsigned long)mdp_req_overlay_set_info[i].flags
        );
    }
}

static void mdp_req_overlay_play_dump(void)
{
    int i;
    
    pr_err("[***]overlay_play(num=%d)\n", mdp_req_overlay_play_num);
    for (i=0; i<MDP_REQ_OVERLAY_PLAY_MAX_NUM; i++) {
        pr_err("[%5lu.%06lu] id=%d,version_key=0x%lx,data=(offset=0x%lx,memory_id=%d,id=%d,flags=0x%lx,priv=0x%lx,iova=0x%lx)\n",
                mdp_req_overlay_play_time[i].sec,
                mdp_req_overlay_play_time[i].usec,
                (int)mdp_req_overlay_play_info[i].id,
                (unsigned long)mdp_req_overlay_play_info[i].version_key,
                (unsigned long)mdp_req_overlay_play_info[i].data.offset,
                (int)mdp_req_overlay_play_info[i].data.memory_id,
                (int)mdp_req_overlay_play_info[i].data.id,
                (unsigned long)mdp_req_overlay_play_info[i].data.flags,
                (unsigned long)mdp_req_overlay_play_info[i].data.priv,
                (unsigned long)mdp_req_overlay_play_info[i].data.iova
        );
    }
}

static void mdp_req_kickoff_pipe_dump(void)
{
    int i, t;
    
    pr_err("[***]kickoff_pipe(num=%d)\n", mdp_req_kickoff_pipe_num);
    for (i=0; i<MDP_REQ_KICKOFF_PIPE_MAX_NUM; i++) {
        pr_err("[%5lu.%06lu] mode=%d\n",
                mdp_req_pan_display_time[i].sec,
                mdp_req_pan_display_time[i].usec,
                (int)mdp_req_kickoff_pipe_info[i].flag
        );
        for (t=0; t<MDP_REQ_KICKOFF_PIPE_MAX_USED; t++) {
            if(mdp_req_kickoff_pipe_info[i].pipe[t].pipe_used != 0) {
                pr_err("[-- pipe%d  --] used=%d,type=%d,num=%d,ndx=%d,share=%d,mixer=(num=%d,stage=%d),src_format=%d,src_width=%d,src_height=%d\n",
                        (int)t+1,
                        (int)mdp_req_kickoff_pipe_info[i].pipe[t].pipe_used,
                        (int)mdp_req_kickoff_pipe_info[i].pipe[t].pipe_type,
                        (int)mdp_req_kickoff_pipe_info[i].pipe[t].pipe_num,
                        (int)mdp_req_kickoff_pipe_info[i].pipe[t].pipe_ndx,
                        (int)mdp_req_kickoff_pipe_info[i].pipe[t].pipe_share,
                        (int)mdp_req_kickoff_pipe_info[i].pipe[t].mixer_num,
                        (int)mdp_req_kickoff_pipe_info[i].pipe[t].mixer_stage,
                        (int)mdp_req_kickoff_pipe_info[i].pipe[t].src_format,
                        (int)mdp_req_kickoff_pipe_info[i].pipe[t].src_width,
                        (int)mdp_req_kickoff_pipe_info[i].pipe[t].src_height
                );
                pr_err("[------------] src=(x=%d,y=%d,w=%d,h=%d),dst=(x=%d,y=%d,w=%d,h=%d),flags=0x%lx,op_mode=%d,transp=%d,blend_op=%d,alpha=%d,is_fg=%d,rotated_90=%d\n",
                        (int)mdp_req_kickoff_pipe_info[i].pipe[t].src_x,
                        (int)mdp_req_kickoff_pipe_info[i].pipe[t].src_y,
                        (int)mdp_req_kickoff_pipe_info[i].pipe[t].src_w,
                        (int)mdp_req_kickoff_pipe_info[i].pipe[t].src_h,
                        (int)mdp_req_kickoff_pipe_info[i].pipe[t].dst_x,
                        (int)mdp_req_kickoff_pipe_info[i].pipe[t].dst_y,
                        (int)mdp_req_kickoff_pipe_info[i].pipe[t].dst_w,
                        (int)mdp_req_kickoff_pipe_info[i].pipe[t].dst_h,
                        (unsigned long)mdp_req_kickoff_pipe_info[i].pipe[t].flags,
                        (int)mdp_req_kickoff_pipe_info[i].pipe[t].op_mode,
                        (int)mdp_req_kickoff_pipe_info[i].pipe[t].transp,
                        (int)mdp_req_kickoff_pipe_info[i].pipe[t].blend_op,
                        (int)mdp_req_kickoff_pipe_info[i].pipe[t].alpha,
                        (int)mdp_req_kickoff_pipe_info[i].pipe[t].is_fg,
                        (int)mdp_req_kickoff_pipe_info[i].pipe[t].rotated_90
                );
                pr_err("[------------] ov_blt_addr=0x%lx,dma_blt_addr=0x%lx,blt_base=0x%lx,blt_offset=0x%lx,blt_cnt=%d,ov_cnt=%d,dmap_cnt=%d,dmae_cnt=%d,blt_end=%d,luma_align_size=%d\n",
                        (unsigned long)mdp_req_kickoff_pipe_info[i].pipe[t].ov_blt_addr,
                        (unsigned long)mdp_req_kickoff_pipe_info[i].pipe[t].dma_blt_addr,
                        (unsigned long)mdp_req_kickoff_pipe_info[i].pipe[t].blt_base,
                        (unsigned long)mdp_req_kickoff_pipe_info[i].pipe[t].blt_offset,
                        (int)mdp_req_kickoff_pipe_info[i].pipe[t].blt_cnt,
                        (int)mdp_req_kickoff_pipe_info[i].pipe[t].ov_cnt,
                        (int)mdp_req_kickoff_pipe_info[i].pipe[t].dmap_cnt,
                        (int)mdp_req_kickoff_pipe_info[i].pipe[t].dmae_cnt,
                        (int)mdp_req_kickoff_pipe_info[i].pipe[t].blt_end,
                        (int)mdp_req_kickoff_pipe_info[i].pipe[t].luma_align_size
                );
            }
        }
        for (t=MDP_REQ_KICKOFF_PIPE_MAX_USED; t<OVERLAY_PIPE_MAX; t++) {
            if (mdp_req_kickoff_pipe_used[t-MDP_REQ_KICKOFF_PIPE_MAX_USED] != 0) {
                pr_err("[-- pipe%d  --] used=%d, Error\n", (int)t+1, (int)mdp_req_kickoff_pipe_used[t-MDP_REQ_KICKOFF_PIPE_MAX_USED] );
            }
        }
    }
}

void mdp_req_trace_dump(int mode)
{
    if (mode & MDP_REQ_DEBUG_OUT_STAT) {
        mdp_req_stat_dump();
    }
    
    if (mode & MDP_REQ_DEBUG_OUT_PAN_DISPLAY) {
        mdp_req_pan_display_dump();
    }
    
    if (mode & MDP_REQ_DEBUG_OUT_OVERLAY_SET) {
        mdp_req_overlay_set_dump();
    }
    
    if (mode & MDP_REQ_DEBUG_OUT_OVERLAY_PLAY) {
        mdp_req_overlay_play_dump();
    }
    
    if (mode & MDP_REQ_DEBUG_OUT_KICKOFF_PIPE) {
        mdp_req_kickoff_pipe_dump();
    }
}
#endif /* CONFIG_SHLCDC_BOARD */

