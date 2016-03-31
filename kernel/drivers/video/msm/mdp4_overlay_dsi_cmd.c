/* Copyright (c) 2011-2013, The Linux Foundation. All rights reserved.
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
#include <linux/delay.h>
#include <linux/io.h>
#include <linux/semaphore.h>
#include <linux/spinlock.h>
#include <linux/fb.h>
#include <asm/system.h>
#include <asm/mach-types.h>
#include <mach/hardware.h>

#include "mdp.h"
#include "msm_fb.h"
#include "mipi_dsi.h"
#include "mdp4.h"
#ifdef CONFIG_SHLCDC_BOARD /* CUST_ID_00051 */ /* CUST_ID_00051 */ /* CUST_ID_00089 */ /* CUST_ID_00095 */ /* CUST_ID_00096 */
#include "mipi_sharp.h"
extern int fps_low_mode;
extern int base_fps_low_mode;
extern void mdp_req_kickoff_pipe_debug(int flag);
extern void mdp_req_trace_dump(int mode);
#endif /* CONFIG_SHLCDC_BOARD */

#ifdef CONFIG_SHLCDC_BOARD /* CUST_ID_00120 */
static int mdp4_dsi_cmd_overlay_err_flg = 0;
#endif /* CONFIG_SHLCDC_BOARD */

#ifdef CONFIG_SHLCDC_BOARD /* CUST_ID_00047 */
static int vsync_start_y_adjust = 767;
#else
static int vsync_start_y_adjust = 4;
#endif /* CONFIG_SHLCDC_BOARD */

#ifdef CONFIG_SHLCDC_BOARD /* CUST_ID_00124 */
struct timespec mdp4_dmap_max_time;
#endif /* CONFIG_SHLCDC_BOARD */

#define MAX_CONTROLLER	1

/*
 * VSYNC_EXPIRE_TICK == 0 means clock always on
 * VSYNC_EXPIRE_TICK == 4 is recommended
 */
#define VSYNC_EXPIRE_TICK 4

static struct vsycn_ctrl {
	struct device *dev;
	int inited;
	int update_ndx;
	int expire_tick;
	int blt_wait;
	u32 ov_koff;
	u32 ov_done;
	u32 dmap_koff;
	u32 dmap_done;
	u32 pan_display;
	uint32 rdptr_intr_tot;
	uint32 rdptr_sirq_tot;
	atomic_t suspend;
	int wait_vsync_cnt;
	int blt_change;
	int blt_free;
	int blt_end;
	int sysfs_created;
	struct mutex update_lock;
	struct completion ov_comp;
	struct completion dmap_comp;
	struct completion vsync_comp;
	spinlock_t spin_lock;
	struct msm_fb_data_type *mfd;
	struct mdp4_overlay_pipe *base_pipe;
	struct vsync_update vlist[2];
	int vsync_enabled;
	int clk_enabled;
	int clk_control;
	ktime_t vsync_time;
	struct work_struct clk_work;
#ifdef CONFIG_SHLCDC_BOARD  /* CUST_ID_00107 */
	int wait_show_cnt;
	struct completion show_comp;
#endif  /* CONFIG_SHLCDC_BOARD */
#ifdef	CONFIG_SHLCDC_BOARD /* CUST_ID_00121 */
	int fpslow_count;
#endif /* CUST_ID_00121 */
#ifdef CONFIG_SHLCDC_BOARD /* CUST_ID_00130 */
	int timeout_counter_dmap;
	int timeout_counter_ov;
#endif /* CUST_ID_00130 */
} vsync_ctrl_db[MAX_CONTROLLER];

#ifdef CONFIG_SHLCDC_BOARD /* CUST_ID_00096 */
void mdp4_dsi_vsync_ctrl_db_out(void)
{
	struct vsycn_ctrl *vctrl;
	vctrl = &vsync_ctrl_db[0];

	pr_err("[ vsycn_ctrl ] inited=%d,update_ndx=%d,expire_tick=%d,blt_wait=%d,ov_koff=%lu,ov_done=%lu,dmap_koff=%lu,dmap_done=%lu\n", 
			vctrl->inited,
			vctrl->update_ndx,
			vctrl->expire_tick,
			vctrl->blt_wait,
			(unsigned long)vctrl->ov_koff,
			(unsigned long)vctrl->ov_done,
			(unsigned long)vctrl->dmap_koff,
			(unsigned long)vctrl->dmap_done
	);
	pr_err("[ ---------- ] rdptr_intr_tot=%lu,rdptr_sirq_tot=%lu,suspend=%d,wait_vsync_cnt=%d,blt_change=%d,blt_free=%d,blt_end=%d,sysfs_created=%d\n", 
			(unsigned long)vctrl->rdptr_intr_tot,
			(unsigned long)vctrl->rdptr_sirq_tot,
			(int)atomic_read(&vctrl->suspend),
			vctrl->wait_vsync_cnt,
			vctrl->blt_change,
			vctrl->blt_free,
			vctrl->blt_end,
			vctrl->sysfs_created
	);
	pr_err("[ ---------- ] ov_comp=0x%lx,dmap_comp=0x%lx,vsync_comp=0x%lx,show_comp=0x%lx\n", 
			(unsigned long)vctrl->ov_comp.done,
			(unsigned long)vctrl->dmap_comp.done,
			(unsigned long)vctrl->vsync_comp.done,
			(unsigned long)vctrl->show_comp.done
	);
	pr_err("[ ---------- ] vsync_enabled=%d,clk_enabled=%d,clk_control=%d,pan_display=%d,wait_show_cnt=%d\n", 
			vctrl->vsync_enabled,
			vctrl->clk_enabled,
			vctrl->clk_control,
			vctrl->pan_display,
			vctrl->wait_show_cnt
	);
}
#endif /* CONFIG_SHLCDC_BOARD */

#ifdef CONFIG_SHLCDC_BOARD /* CUST_ID_00130 */
static void clear_vctrl_koff_done_cnt(void)
{
	struct vsycn_ctrl *vctrl;
	vctrl = &vsync_ctrl_db[0];
	vctrl->ov_koff = 0;
	vctrl->ov_done = 0;
	vctrl->dmap_koff = 0;
	vctrl->dmap_done = 0;
}
#endif /* CUST_ID_00130 */

#ifdef CONFIG_SHLCDC_BOARD /* CUST_ID_00146 */
static int mdp_qos_deny_collapse = 0;
static struct workqueue_struct    *mdp_qos_wq;
static struct work_struct         mdp_qos_work;
static DEFINE_MUTEX(mdp_qos_lock);

static void mdp_qos_work_handler(struct work_struct *work)
{
	struct vsycn_ctrl *vctrl;
	int cndx = 0;

	vctrl = &vsync_ctrl_db[cndx];

	if (atomic_read(&vctrl->suspend) > 0)
		return;
	
	mutex_lock(&mdp_qos_lock);
	if(!mdp_qos_deny_collapse) {
		mipi_dsi_latency_allow_collapse();
	}
	mutex_unlock(&mdp_qos_lock);
}
#endif /* CONFIG_SHLCDC_BOARD */

static void vsync_irq_enable(int intr, int term)
{
	unsigned long flag;

	spin_lock_irqsave(&mdp_spin_lock, flag);
	/* no need to clrear other interrupts for comamnd mode */
	mdp_intr_mask |= intr;
	outp32(MDP_INTR_ENABLE, mdp_intr_mask);
	mdp_enable_irq(term);
	spin_unlock_irqrestore(&mdp_spin_lock, flag);
}

static void vsync_irq_disable(int intr, int term)
{
	unsigned long flag;

	spin_lock_irqsave(&mdp_spin_lock, flag);
	/* no need to clrear other interrupts for comamnd mode */
	mdp_intr_mask &= ~intr;
	outp32(MDP_INTR_ENABLE, mdp_intr_mask);
	mdp_disable_irq_nosync(term);
	spin_unlock_irqrestore(&mdp_spin_lock, flag);
}

static void mdp4_dsi_cmd_blt_ov_update(struct mdp4_overlay_pipe *pipe)
{
	uint32 off, addr;
	int bpp;
	char *overlay_base;

	if (pipe->ov_blt_addr == 0)
		return;

#ifdef BLT_RGB565
	bpp = 2; /* overlay ouput is RGB565 */
#else
	bpp = 3; /* overlay ouput is RGB888 */
#endif
	off = 0;
	if (pipe->ov_cnt & 0x01)
		off = pipe->src_height * pipe->src_width * bpp;
	addr = pipe->ov_blt_addr + off;
	/* overlay 0 */
	overlay_base = MDP_BASE + MDP4_OVERLAYPROC0_BASE;/* 0x10000 */
	outpdw(overlay_base + 0x000c, addr);
	outpdw(overlay_base + 0x001c, addr);
}

static void mdp4_dsi_cmd_blt_dmap_update(struct mdp4_overlay_pipe *pipe)
{
	uint32 off, addr;
	int bpp;

	if (pipe->ov_blt_addr == 0)
		return;

#ifdef BLT_RGB565
	bpp = 2; /* overlay ouput is RGB565 */
#else
	bpp = 3; /* overlay ouput is RGB888 */
#endif
	off = 0;
	if (pipe->dmap_cnt & 0x01)
		off = pipe->src_height * pipe->src_width * bpp;
	addr = pipe->dma_blt_addr + off;

	/* dmap */
	MDP_OUTP(MDP_BASE + 0x90008, addr);
}

static void mdp4_dsi_cmd_wait4dmap(int cndx);
static void mdp4_dsi_cmd_wait4ov(int cndx);

static void mdp4_dsi_cmd_do_blt(struct msm_fb_data_type *mfd, int enable)
{
	unsigned long flags;
	int cndx = 0;
	struct vsycn_ctrl *vctrl;
	struct mdp4_overlay_pipe *pipe;
	int need_wait;

	vctrl = &vsync_ctrl_db[cndx];
	pipe = vctrl->base_pipe;

	mdp4_allocate_writeback_buf(mfd, MDP4_MIXER0);

	if (mfd->ov0_wb_buf->write_addr == 0) {
		pr_info("%s: no blt_base assigned\n", __func__);
		return;
	}

	spin_lock_irqsave(&vctrl->spin_lock, flags);
	if (enable && pipe->ov_blt_addr == 0) {
		vctrl->blt_change++;
		if (vctrl->dmap_koff != vctrl->dmap_done) {
			INIT_COMPLETION(vctrl->dmap_comp);
			need_wait = 1;
		}
	} else if (enable == 0 && pipe->ov_blt_addr) {
		vctrl->blt_change++;
		if (vctrl->ov_koff != vctrl->dmap_done) {
			INIT_COMPLETION(vctrl->dmap_comp);
			need_wait = 1;
		}
	}
	spin_unlock_irqrestore(&vctrl->spin_lock, flags);

	if (need_wait)
		mdp4_dsi_cmd_wait4dmap(0);

	spin_lock_irqsave(&vctrl->spin_lock, flags);
	if (enable && pipe->ov_blt_addr == 0) {
		pipe->ov_blt_addr = mfd->ov0_wb_buf->write_addr;
		pipe->dma_blt_addr = mfd->ov0_wb_buf->read_addr;
		pipe->ov_cnt = 0;
		pipe->dmap_cnt = 0;
		vctrl->ov_koff = vctrl->dmap_koff;
		vctrl->ov_done = vctrl->dmap_done;
		vctrl->blt_free = 0;
		vctrl->blt_wait = 0;
		vctrl->blt_end = 0;
#ifdef CONFIG_SHLCDC_BOARD /* CUST_ID_00127 */
		mdp4_stat.blt_dsi_cmd++;
#else
		mdp4_stat.blt_dsi_video++;
#endif /* CONFIG_SHLCDC_BOARD */
	} else if (enable == 0 && pipe->ov_blt_addr) {
		pipe->ov_blt_addr = 0;
		pipe->dma_blt_addr =  0;
		vctrl->blt_end = 1;
		vctrl->blt_free = 4;	/* 4 commits to free wb buf */
	}

	pr_info("%s: changed=%d enable=%d ov_blt_addr=%x\n", __func__,
		vctrl->blt_change, enable, (int)pipe->ov_blt_addr);

	spin_unlock_irqrestore(&vctrl->spin_lock, flags);
}

/*
 * mdp4_dsi_cmd_do_update:
 * called from thread context
 */
void mdp4_dsi_cmd_pipe_queue(int cndx, struct mdp4_overlay_pipe *pipe)
{
	struct vsycn_ctrl *vctrl;
	struct vsync_update *vp;
	struct mdp4_overlay_pipe *pp;
	int undx;

	if (cndx >= MAX_CONTROLLER) {
		pr_err("%s: out or range: cndx=%d\n", __func__, cndx);
		return;
	}

	vctrl = &vsync_ctrl_db[cndx];

	if (atomic_read(&vctrl->suspend)) {
		pr_err("%s: suspended, no more pipe queue\n", __func__);
		return;
	}

	mutex_lock(&vctrl->update_lock);
	undx =  vctrl->update_ndx;
	vp = &vctrl->vlist[undx];

	pp = &vp->plist[pipe->pipe_ndx - 1];	/* ndx start form 1 */

	pr_debug("%s: vndx=%d pipe_ndx=%d expire=%x pid=%d\n", __func__,
		undx, pipe->pipe_ndx, vctrl->expire_tick, current->pid);

	*pp = *pipe;	/* clone it */
	vp->update_cnt++;

	mutex_unlock(&vctrl->update_lock);
	mdp4_stat.overlay_play[pipe->mixer_num]++;
}

static void mdp4_dsi_cmd_blt_ov_update(struct mdp4_overlay_pipe *pipe);

int mdp4_dsi_cmd_pipe_commit(int cndx, int wait)
{
	int  i, undx;
	int mixer = 0;
	struct vsycn_ctrl *vctrl;
	struct vsync_update *vp;
	struct mdp4_overlay_pipe *pipe;
	struct mdp4_overlay_pipe *real_pipe;
	unsigned long flags;
	int need_dmap_wait = 0;
	int need_ov_wait = 0;
	int cnt = 0;
#ifdef CONFIG_SHLCDC_BOARD /* CUST_ID_00120 */
	mdp4_dsi_cmd_overlay_err_flg = 0;
#endif /* CONFIG_SHLCDC_BOARD */

	vctrl = &vsync_ctrl_db[0];

	mutex_lock(&vctrl->update_lock);
	undx =  vctrl->update_ndx;
	vp = &vctrl->vlist[undx];
	pipe = vctrl->base_pipe;
	mixer = pipe->mixer_num;

	if (vp->update_cnt == 0) {
		mutex_unlock(&vctrl->update_lock);
#ifdef CONFIG_SHLCDC_BOARD /* CUST_ID_00120 */
		mdp4_dsi_cmd_overlay_err_flg = 1;
#endif /* CONFIG_SHLCDC_BOARD */
		return cnt;
	}

	vctrl->update_ndx++;
	vctrl->update_ndx &= 0x01;
	vp->update_cnt = 0;     /* reset */
	if (vctrl->blt_free) {
		vctrl->blt_free--;
		if (vctrl->blt_free == 0)
			mdp4_free_writeback_buf(vctrl->mfd, mixer);
	}
	mutex_unlock(&vctrl->update_lock);

	/* free previous committed iommu back to pool */
	mdp4_overlay_iommu_unmap_freelist(mixer);

	spin_lock_irqsave(&vctrl->spin_lock, flags);
	if (pipe->ov_blt_addr) {
		/* Blt */
		if (vctrl->blt_wait)
			need_dmap_wait = 1;
		if (vctrl->ov_koff != vctrl->ov_done) {
			INIT_COMPLETION(vctrl->ov_comp);
			need_ov_wait = 1;
		}
	} else {
		/* direct out */
		if (vctrl->dmap_koff != vctrl->dmap_done) {
			INIT_COMPLETION(vctrl->dmap_comp);
			pr_debug("%s: wait, ok=%d od=%d dk=%d dd=%d cpu=%d\n",
			 __func__, vctrl->ov_koff, vctrl->ov_done,
			vctrl->dmap_koff, vctrl->dmap_done, smp_processor_id());
			need_dmap_wait = 1;
		}
	}
	spin_unlock_irqrestore(&vctrl->spin_lock, flags);

	if (need_dmap_wait) {
		pr_debug("%s: wait4dmap\n", __func__);
		mdp4_dsi_cmd_wait4dmap(0);
	}

	if (need_ov_wait) {
		pr_debug("%s: wait4ov\n", __func__);
		mdp4_dsi_cmd_wait4ov(0);
	}

	if (pipe->ov_blt_addr) {
		if (vctrl->blt_end) {
			vctrl->blt_end = 0;
			pipe->ov_blt_addr = 0;
			pipe->dma_blt_addr =  0;
		}
	}

	if (vctrl->blt_change) {
		mdp4_overlayproc_cfg(pipe);
		mdp4_overlay_dmap_xy(pipe);
		vctrl->blt_change = 0;
	}

	pipe = vp->plist;
	for (i = 0; i < OVERLAY_PIPE_MAX; i++, pipe++) {
		if (pipe->pipe_used) {
			cnt++;
			real_pipe = mdp4_overlay_ndx2pipe(pipe->pipe_ndx);
			if (real_pipe && real_pipe->pipe_used) {
				/* pipe not unset */
				mdp4_overlay_vsync_commit(pipe);
			}
			/* free previous iommu to freelist
			* which will be freed at next
			* pipe_commit
			*/
			mdp4_overlay_iommu_pipe_free(pipe->pipe_ndx, 0);
			pipe->pipe_used = 0; /* clear */
		}
	}

	/* tx dcs command if had any */
	mipi_dsi_cmdlist_commit(1);

	mdp4_mixer_stage_commit(mixer);

	pipe = vctrl->base_pipe;

#ifdef CONFIG_SHLCDC_BOARD /* CUST_ID_00117 */
	mdp4_dma_convert_setup_cmd_mode();
	mdp4_dma_pcc_setup_cmd_mode();
	mdp_lut_enable();
#endif	/* CONFIG_SHLCDC_BOARD */

#ifdef CONFIG_SHLCDC_BOARD /* CUST_ID_00146 */
	mutex_lock(&mdp_qos_lock);
	if (mdp_qos_wq) {
		mipi_dsi_latency_deny_collapse();
		mdp_qos_deny_collapse = 1;
	}
#endif /* CONFIG_SHLCDC_BOARD */

	spin_lock_irqsave(&vctrl->spin_lock, flags);
	if (pipe->ov_blt_addr) {
		mdp4_dsi_cmd_blt_ov_update(pipe);
		pipe->ov_cnt++;
		vctrl->ov_koff++;
		INIT_COMPLETION(vctrl->ov_comp);
		vsync_irq_enable(INTR_OVERLAY0_DONE, MDP_OVERLAY0_TERM);
	} else {
#ifdef CONFIG_SHLCDC_BOARD /* CUST_ID_00123 */
		mipi_dsi_cmd_mdp_start();
#endif /* CONFIG_SHLCDC_BOARD */
		INIT_COMPLETION(vctrl->dmap_comp);
		vsync_irq_enable(INTR_DMA_P_DONE, MDP_DMAP_TERM);
		vctrl->dmap_koff++;
	}
	pr_debug("%s: kickoff, pid=%d\n", __func__, current->pid);
	/* kickoff overlay engine */
	mdp4_stat.kickoff_ov0++;
#ifdef CONFIG_SHLCDC_BOARD /* CUST_ID_00095 */
	mdp_req_kickoff_pipe_debug(0);
#endif /* CONFIG_SHLCDC_BOARD */
	outpdw(MDP_BASE + 0x0004, 0);
	mb();
	spin_unlock_irqrestore(&vctrl->spin_lock, flags);

#ifdef CONFIG_SHLCDC_BOARD /* CUST_ID_00146 */
	mutex_unlock(&mdp_qos_lock);
#endif /* CONFIG_SHLCDC_BOARD */

	mdp4_stat.overlay_commit[pipe->mixer_num]++;

	if (wait) {
		long long tick;

		mdp4_dsi_cmd_wait4vsync(cndx, &tick);
	}

	return cnt;
}

#ifdef CONFIG_SHLCDC_BOARD /* CUST_ID_00100 */
int mdp4_dsi_cmd_vsync_get_status(void)
{
	struct vsycn_ctrl *vctrl;
	
	vctrl = &vsync_ctrl_db[0];
	return vctrl->vsync_enabled;
}
#endif /* CONFIG_SHLCDC_BOARD */

static void mdp4_overlay_update_dsi_cmd(struct msm_fb_data_type *mfd);

void mdp4_dsi_cmd_vsync_ctrl(struct fb_info *info, int enable)
{
	struct vsycn_ctrl *vctrl;
	unsigned long flags;
	int cndx = 0;
	int clk_set_on = 0;

	vctrl = &vsync_ctrl_db[cndx];

	mutex_lock(&vctrl->update_lock);
	pr_debug("%s: clk_enabled=%d vsync_enabled=%d req=%d\n", __func__,
		vctrl->clk_enabled, vctrl->vsync_enabled, enable);

	if (vctrl->vsync_enabled == enable) {
		mutex_unlock(&vctrl->update_lock);
		return;
	}

	vctrl->vsync_enabled = enable;

	if (enable) {
		spin_lock_irqsave(&vctrl->spin_lock, flags);
		vctrl->clk_control = 0;
		vctrl->expire_tick = 0;
		spin_unlock_irqrestore(&vctrl->spin_lock, flags);
		if (vctrl->clk_enabled == 0) {
			pr_debug("%s: SET_CLK_ON\n", __func__);
			mipi_dsi_clk_cfg(1);
			mdp_clk_ctrl(1);
			vctrl->clk_enabled = 1;
			clk_set_on = 1;
		}
		if (clk_set_on) {
			vsync_irq_enable(INTR_PRIMARY_RDPTR,
						MDP_PRIM_RDPTR_TERM);
		}
	} else {
		spin_lock_irqsave(&vctrl->spin_lock, flags);
		vctrl->expire_tick = VSYNC_EXPIRE_TICK;
		spin_unlock_irqrestore(&vctrl->spin_lock, flags);
	}
	mutex_unlock(&vctrl->update_lock);
}

void mdp4_dsi_cmd_wait4vsync(int cndx, long long *vtime)
{
	struct vsycn_ctrl *vctrl;
	struct mdp4_overlay_pipe *pipe;
	unsigned long flags;
#ifdef CONFIG_SHLCDC_BOARD /* CUST_ID_00096 */
	int wait_ret;
	int dsi_clk_info;
	int mdp_clk_info;
	unsigned long int_cnt;
#endif /* CONFIG_SHLCDC_BOARD */

	if (cndx >= MAX_CONTROLLER) {
		pr_err("%s: out or range: cndx=%d\n", __func__, cndx);
		return;
	}

	vctrl = &vsync_ctrl_db[cndx];
	pipe = vctrl->base_pipe;

	if (atomic_read(&vctrl->suspend) > 0) {
		*vtime = -1;
		return;
	}

	spin_lock_irqsave(&vctrl->spin_lock, flags);
	if (vctrl->wait_vsync_cnt == 0)
		INIT_COMPLETION(vctrl->vsync_comp);
	vctrl->wait_vsync_cnt++;
	spin_unlock_irqrestore(&vctrl->spin_lock, flags);

#ifdef CONFIG_SHLCDC_BOARD /* CUST_ID_00096 */
	dsi_clk_info = mipi_dsi_clk_on;
	mdp_clk_info = mdp_get_clk_cnt();
	int_cnt = mdp4_stat.intr_rdptr;
	wait_ret = wait_for_completion_timeout(&vctrl->vsync_comp, msecs_to_jiffies(100));
	if (wait_ret <= 0) {
		pr_err("%s %d  TIMEOUT_\n", __func__, __LINE__);
		pr_err("dsi_clk=%d->%d, mdp_clk=%d->%d, isr=%d, count=%lu->%lu\n",
				dsi_clk_info, mipi_dsi_clk_on, mdp_clk_info, mdp_get_clk_cnt(), mdp_is_in_isr, int_cnt, mdp4_stat.intr_rdptr);
	}
#else
	if (!wait_for_completion_timeout(
			&vctrl->vsync_comp, msecs_to_jiffies(100)))
		pr_err("%s %d  TIMEOUT_\n", __func__, __LINE__);
#endif /* CONFIG_SHLCDC_BOARD */
	mdp4_stat.wait4vsync0++;

	*vtime = ktime_to_ns(vctrl->vsync_time);
}

#ifdef CONFIG_SHLCDC_BOARD /* CUST_ID_00115 */
void mdp4_dsi_cmd_busy_wait(void)
{
	struct vsycn_ctrl *vctrl;
	unsigned long flags;
	struct mdp4_overlay_pipe *pipe;
	int need_dmap_wait = 0;
	int need_ov_wait = 0;

	vctrl = &vsync_ctrl_db[0];

	mutex_lock(&vctrl->update_lock);
	pipe = vctrl->base_pipe;
	mutex_unlock(&vctrl->update_lock);

	spin_lock_irqsave(&vctrl->spin_lock, flags);

	pr_debug("%s: wait, ok=%d od=%d dk=%d dd=%d cpu=%d\n",
			__func__, vctrl->ov_koff, vctrl->ov_done,
			vctrl->dmap_koff, vctrl->dmap_done, smp_processor_id());

	if (pipe && (pipe->ov_blt_addr)) {
		/* Blt */
		if (vctrl->blt_wait)
			need_dmap_wait = 1;
		else if (vctrl->ov_koff != vctrl->ov_done) {
			INIT_COMPLETION(vctrl->ov_comp);
			need_ov_wait = 1;
		}
	} else {
		/* direct out */
		if (vctrl->dmap_koff != vctrl->dmap_done) {
			INIT_COMPLETION(vctrl->dmap_comp);
			need_dmap_wait = 1;
		}
	}

	spin_unlock_irqrestore(&vctrl->spin_lock, flags);

	if (need_dmap_wait) {
		pr_debug("%s: wait4dmap\n", __func__);
		mdp4_dsi_cmd_wait4dmap(0);
	}

	if (need_ov_wait) {
		pr_debug("%s: wait4ov\n", __func__);
		mdp4_dsi_cmd_wait4ov(0);
	}

	return;
}
#endif /* CONFIG_SHLCDC_BOARD*/

static void mdp4_dsi_cmd_wait4dmap(int cndx)
{
	struct vsycn_ctrl *vctrl;
#ifdef CONFIG_SHLCDC_BOARD /* CUST_ID_00096 */ /* CUST_ID_00124 */
	struct timespec start, stop, df;
	int wait_ret;
	int dsi_clk_info;
	int mdp_clk_info;
	unsigned long int_cnt;
#endif /* CONFIG_SHLCDC_BOARD */

	if (cndx >= MAX_CONTROLLER) {
		pr_err("%s: out or range: cndx=%d\n", __func__, cndx);
		return;
	}

	vctrl = &vsync_ctrl_db[cndx];

	if (atomic_read(&vctrl->suspend) > 0)
		return;

#ifdef CONFIG_SHLCDC_BOARD /* CUST_ID_00096 */ /* CUST_ID_00124 */ /* CUST_ID_00130 */
	dsi_clk_info = mipi_dsi_clk_on;
	mdp_clk_info = mdp_get_clk_cnt();
	int_cnt = mdp4_stat.intr_dma_p;
	getnstimeofday(&start);
	wait_ret = wait_for_completion_timeout(&vctrl->dmap_comp, msecs_to_jiffies(100));
	if (wait_ret > 0) {
		getnstimeofday(&stop);
		df = timespec_sub(stop, start);
		if (timespec_compare(&df, &mdp4_dmap_max_time) > 0) {
			mdp4_dmap_max_time = df;
		}
		if(vctrl->timeout_counter_dmap > 0){
			/* notice ... */
			pr_info("%s:[notice] timeout_counter_dmap=%d\n", __func__, vctrl->timeout_counter_dmap);
			pr_info("%s %d  TIMEOUT_\n", __func__, __LINE__);
			pr_info("dsi_clk=%d->%d, mdp_clk=%d->%d, isr=%d, count=%lu->%lu\n",
				dsi_clk_info, mipi_dsi_clk_on, mdp_clk_info, mdp_get_clk_cnt(), mdp_is_in_isr, int_cnt, mdp4_stat.intr_dma_p);
			clear_vctrl_koff_done_cnt();
		}
		vctrl->timeout_counter_dmap=0;
	}
	else if(wait_ret == 0){
		/* rescue from System freeze : carefully and carefully */
		if(vctrl->timeout_counter_dmap >= 100){
			pr_info("%s:**** [error] timeout consecution 100 times **** timeout_counter_dmap=%d\n", __func__, vctrl->timeout_counter_dmap);
			pr_err("%s %d  TIMEOUT_\n", __func__, __LINE__);
			pr_err("dsi_clk=%d->%d, mdp_clk=%d->%d, isr=%d, count=%lu->%lu\n",
					dsi_clk_info, mipi_dsi_clk_on, mdp_clk_info, mdp_get_clk_cnt(), mdp_is_in_isr, int_cnt, mdp4_stat.intr_dma_p);
			mdp_clk_info_out();
			mdp4_dsi_vsync_ctrl_db_out();
			mdp_req_trace_dump(0x1F);
			WARN(1, "%s: timeout Err!\n", __func__);
			panic("%s: timeout Err!\n", __func__);
		}
		vctrl->timeout_counter_dmap ++;
		clear_vctrl_koff_done_cnt();
	}
#else
	if (!wait_for_completion_timeout(
			&vctrl->dmap_comp, msecs_to_jiffies(100)))
		pr_err("%s %d  TIMEOUT_\n", __func__, __LINE__);
#endif /* CONFIG_SHLCDC_BOARD */
}

static void mdp4_dsi_cmd_wait4ov(int cndx)
{
	struct vsycn_ctrl *vctrl;
#ifdef CONFIG_SHLCDC_BOARD /* CUST_ID_00096 */
	int wait_ret;
	int dsi_clk_info;
	int mdp_clk_info;
	unsigned long int_cnt;
#endif /* CONFIG_SHLCDC_BOARD */

	if (cndx >= MAX_CONTROLLER) {
		pr_err("%s: out or range: cndx=%d\n", __func__, cndx);
		return;
	}

	vctrl = &vsync_ctrl_db[cndx];

	if (atomic_read(&vctrl->suspend) > 0)
		return;

#ifdef CONFIG_SHLCDC_BOARD /* CUST_ID_00096 */ /* CUST_ID_00130 */
	dsi_clk_info = mipi_dsi_clk_on;
	mdp_clk_info = mdp_get_clk_cnt();
	int_cnt = mdp4_stat.intr_overlay0;
	wait_ret = wait_for_completion_timeout(&vctrl->ov_comp, msecs_to_jiffies(100));
	if(wait_ret > 0){
		if(vctrl->timeout_counter_ov > 0){
			/* notice ... */
			pr_info("%s:[notice] timeout_counter_ov=%d\n", __func__, vctrl->timeout_counter_ov);
			pr_info("%s %d  TIMEOUT_\n", __func__, __LINE__);
			pr_info("dsi_clk=%d->%d, mdp_clk=%d->%d, isr=%d, count=%lu->%lu\n",
				dsi_clk_info, mipi_dsi_clk_on, mdp_clk_info, mdp_get_clk_cnt(), mdp_is_in_isr, int_cnt, mdp4_stat.intr_overlay0);
			clear_vctrl_koff_done_cnt();
		}
		vctrl->timeout_counter_ov=0;
	}
	else if(wait_ret == 0) {
		/* rescue from System freeze : carefully and carefully */
		if(vctrl->timeout_counter_ov >= 100){
			pr_err("%s:**** [error] timeout consecution 100 times **** timeout_counter_ov=%d\n", __func__, vctrl->timeout_counter_ov);
			pr_err("%s %d  TIMEOUT_\n", __func__, __LINE__);
			pr_err("dsi_clk=%d->%d, mdp_clk=%d->%d, isr=%d, count=%lu->%lu\n",
				dsi_clk_info, mipi_dsi_clk_on, mdp_clk_info, mdp_get_clk_cnt(), mdp_is_in_isr, int_cnt, mdp4_stat.intr_overlay0);
			mdp_clk_info_out();
			mdp4_dsi_vsync_ctrl_db_out();
			mdp_req_trace_dump(0x1F);
			WARN(1, "%s: timeout Err!\n", __func__);
			panic("%s: timeout Err!\n", __func__);
		}
		vctrl->timeout_counter_ov ++;
		clear_vctrl_koff_done_cnt();
	}
#else
	if (!wait_for_completion_timeout(
			&vctrl->ov_comp, msecs_to_jiffies(100)))
		pr_err("%s %d  TIMEOUT_\n", __func__, __LINE__);
#endif /* CONFIG_SHLCDC_BOARD */
}

/*
 * primary_rdptr_isr:
 * called from interrupt context
 */

static void primary_rdptr_isr(int cndx)
{
	struct vsycn_ctrl *vctrl;

	vctrl = &vsync_ctrl_db[cndx];
	pr_debug("%s: ISR, tick=%d pan=%d cpu=%d\n", __func__,
		vctrl->expire_tick, vctrl->pan_display, smp_processor_id());
	vctrl->rdptr_intr_tot++;
#ifdef CONFIG_SHLCDC_BOARD /* CUST_ID_00121 */
	if ((fps_low_mode || base_fps_low_mode)) {
		vctrl->fpslow_count = (vctrl->fpslow_count + 1) & 1;
		if (vctrl->fpslow_count) {
			pr_debug("%s: return....\n", __func__);
			return;
		}
	}
#endif /* CUST_ID_00121 */

	spin_lock(&vctrl->spin_lock);
	vctrl->vsync_time = ktime_get();

	complete_all(&vctrl->vsync_comp);
	vctrl->wait_vsync_cnt = 0;

#ifdef CONFIG_SHLCDC_BOARD  /* CUST_ID_00107 */
	complete_all(&vctrl->show_comp);
	vctrl->wait_show_cnt = 0;
#endif  /* CONFIG_SHLCDC_BOARD */
	if (vctrl->expire_tick) {
		vctrl->expire_tick--;
		if (vctrl->expire_tick == 0) {
			if (vctrl->pan_display <= 0) {
				vctrl->clk_control = 1;
				schedule_work(&vctrl->clk_work);
			} else {
				/* wait one more vsycn */
				vctrl->expire_tick += 1;
			}
		}
	}
	spin_unlock(&vctrl->spin_lock);
}

void mdp4_dmap_done_dsi_cmd(int cndx)
{
	struct vsycn_ctrl *vctrl;
	struct mdp4_overlay_pipe *pipe;
	int diff;

	vctrl = &vsync_ctrl_db[cndx];
	pipe = vctrl->base_pipe;

	 /* blt enabled */
	spin_lock(&vctrl->spin_lock);
	vsync_irq_disable(INTR_DMA_P_DONE, MDP_DMAP_TERM);
	vctrl->dmap_done++;

	if (vctrl->pan_display)
		vctrl->pan_display--;

	diff = vctrl->ov_done - vctrl->dmap_done;
	pr_debug("%s: ov_koff=%d ov_done=%d dmap_koff=%d dmap_done=%d cpu=%d\n",
		__func__, vctrl->ov_koff, vctrl->ov_done, vctrl->dmap_koff,
		vctrl->dmap_done, smp_processor_id());
	complete(&vctrl->dmap_comp);
	if (diff <= 0) {
		if (vctrl->blt_wait)
			vctrl->blt_wait = 0;
		spin_unlock(&vctrl->spin_lock);
#ifdef CONFIG_SHLCDC_BOARD /* CUST_ID_00146 */
		if (mdp_qos_wq) {
			mdp_qos_deny_collapse = 0;
			queue_work(mdp_qos_wq, &mdp_qos_work);
		}
#endif /* CONFIG_SHLCDC_BOARD */
		return;
	}

	/* kick dmap */
	mdp4_dsi_cmd_blt_dmap_update(pipe);
	pipe->dmap_cnt++;
	mdp4_stat.kickoff_dmap++;
	vctrl->dmap_koff++;
	vsync_irq_enable(INTR_DMA_P_DONE, MDP_DMAP_TERM);
	outpdw(MDP_BASE + 0x000c, 0); /* kickoff dmap engine */
	mb();
#ifdef CONFIG_SHLCDC_BOARD /* CUST_ID_00123 */
	mipi_dsi_cmd_mdp_start();
#endif /* CONFIG_SHLCDC_BOARD */
	spin_unlock(&vctrl->spin_lock);
}

/*
 * mdp4_overlay0_done_dsi_cmd: called from isr
 */
void mdp4_overlay0_done_dsi_cmd(int cndx)
{
	struct vsycn_ctrl *vctrl;
	struct mdp4_overlay_pipe *pipe;
	int diff;

	vctrl = &vsync_ctrl_db[cndx];
	pipe = vctrl->base_pipe;

	spin_lock(&vctrl->spin_lock);
	vsync_irq_disable(INTR_OVERLAY0_DONE, MDP_OVERLAY0_TERM);
	vctrl->ov_done++;
	complete(&vctrl->ov_comp);
	diff = vctrl->ov_done - vctrl->dmap_done;

	pr_debug("%s: ov_koff=%d ov_done=%d dmap_koff=%d dmap_done=%d cpu=%d\n",
		__func__, vctrl->ov_koff, vctrl->ov_done, vctrl->dmap_koff,
		vctrl->dmap_done, smp_processor_id());

	if (pipe->ov_blt_addr == 0) {
		/* blt disabled */
		spin_unlock(&vctrl->spin_lock);
		return;
	}

	if (diff > 1) {
		/*
		 * two overlay_done and none dmap_done yet
		 * let dmap_done kickoff dmap
		 * and put pipe_commit to wait
		 */
		vctrl->blt_wait = 1;
		pr_debug("%s: blt_wait set\n", __func__);
		spin_unlock(&vctrl->spin_lock);
		return;
	}
	mdp4_dsi_cmd_blt_dmap_update(pipe);
	pipe->dmap_cnt++;
	mdp4_stat.kickoff_dmap++;
	vctrl->dmap_koff++;
	vsync_irq_enable(INTR_DMA_P_DONE, MDP_DMAP_TERM);
	outpdw(MDP_BASE + 0x000c, 0); /* kickoff dmap engine */
	mb();
#ifdef CONFIG_SHLCDC_BOARD /* CUST_ID_00123 */
	mipi_dsi_cmd_mdp_start();
#endif /* CONFIG_SHLCDC_BOARD */
	spin_unlock(&vctrl->spin_lock);
}

static void clk_ctrl_work(struct work_struct *work)
{
	unsigned long flags;
	struct vsycn_ctrl *vctrl =
		container_of(work, typeof(*vctrl), clk_work);

	mutex_lock(&vctrl->update_lock);
	spin_lock_irqsave(&vctrl->spin_lock, flags);
	if (vctrl->clk_control && vctrl->clk_enabled) {
		vsync_irq_disable(INTR_PRIMARY_RDPTR, MDP_PRIM_RDPTR_TERM);
		vctrl->clk_enabled = 0;
		vctrl->clk_control = 0;
		spin_unlock_irqrestore(&vctrl->spin_lock, flags);
		/* make sure dsi link is idle */
		mipi_dsi_mdp_busy_wait();
		mipi_dsi_clk_cfg(0);
		mdp_clk_ctrl(0);
		pr_debug("%s: SET_CLK_OFF, pid=%d\n", __func__, current->pid);
	} else {
		spin_unlock_irqrestore(&vctrl->spin_lock, flags);
	}
	mutex_unlock(&vctrl->update_lock);
}

ssize_t mdp4_dsi_cmd_show_event(struct device *dev,
		struct device_attribute *attr, char *buf)
{
	int cndx;
	struct vsycn_ctrl *vctrl;
	ssize_t ret = 0;
	unsigned long flags;
	u64 vsync_tick;

	cndx = 0;
	vctrl = &vsync_ctrl_db[0];

	if (atomic_read(&vctrl->suspend) > 0)
		return 0;

	spin_lock_irqsave(&vctrl->spin_lock, flags);
#ifdef CONFIG_SHLCDC_BOARD  /* CUST_ID_00107 */ /* CUST_ID_00135 */
	if (vctrl->wait_show_cnt == 0)
		INIT_COMPLETION(vctrl->show_comp);
	vctrl->wait_show_cnt++;
	spin_unlock_irqrestore(&vctrl->spin_lock, flags);

	if(fps_low_mode || base_fps_low_mode){
		ret = wait_for_completion_interruptible_timeout(&vctrl->show_comp,
			msecs_to_jiffies(VSYNC_PERIOD * 4));
	}
	else{
		ret = wait_for_completion_interruptible_timeout(&vctrl->show_comp,
			msecs_to_jiffies(VSYNC_PERIOD * 2));
	}
#else
	if (vctrl->wait_vsync_cnt == 0)
		INIT_COMPLETION(vctrl->vsync_comp);
	vctrl->wait_vsync_cnt++;
	spin_unlock_irqrestore(&vctrl->spin_lock, flags);

	ret = wait_for_completion_interruptible_timeout(&vctrl->vsync_comp,
		msecs_to_jiffies(VSYNC_PERIOD * 4));
#endif /* CONFIG_SHLCDC_BOARD */
	if (ret <= 0) {
		vctrl->wait_vsync_cnt = 0;
		vsync_tick = ktime_to_ns(ktime_get());
		ret = snprintf(buf, PAGE_SIZE, "VSYNC=%llu", vsync_tick);
		buf[strlen(buf) + 1] = '\0';
		return ret;
	}

	spin_lock_irqsave(&vctrl->spin_lock, flags);
	vsync_tick = ktime_to_ns(vctrl->vsync_time);
	spin_unlock_irqrestore(&vctrl->spin_lock, flags);

	ret = snprintf(buf, PAGE_SIZE, "VSYNC=%llu", vsync_tick);
	pr_debug("%s: UEVENT\n", __func__);

	buf[strlen(buf) + 1] = '\0';
	return ret;
}

void mdp4_dsi_rdptr_init(int cndx)
{
	struct vsycn_ctrl *vctrl;

	if (cndx >= MAX_CONTROLLER) {
		pr_err("%s: out or range: cndx=%d\n", __func__, cndx);
		return;
	}

	vctrl = &vsync_ctrl_db[cndx];
	if (vctrl->inited)
		return;

	vctrl->inited = 1;
	vctrl->update_ndx = 0;
	mutex_init(&vctrl->update_lock);
	init_completion(&vctrl->ov_comp);
	init_completion(&vctrl->dmap_comp);
	init_completion(&vctrl->vsync_comp);
#ifdef CONFIG_SHLCDC_BOARD  /* CUST_ID_00107 */
	init_completion(&vctrl->show_comp);
#endif  /* CONFIG_SHLCDC_BOARD */
	spin_lock_init(&vctrl->spin_lock);
	atomic_set(&vctrl->suspend, 1);
	INIT_WORK(&vctrl->clk_work, clk_ctrl_work);
#ifdef	CONFIG_SHLCDC_BOARD /* CUST_ID_00121 */
	vctrl->fpslow_count = 0;
#endif /* CUST_ID_00121 */
}

void mdp4_primary_rdptr(void)
{
	primary_rdptr_isr(0);
}

static __u32 msm_fb_line_length(__u32 fb_index, __u32 xres, int bpp)
{
	/*
	 * The adreno GPU hardware requires that the pitch be aligned to
	 * 32 pixels for color buffers, so for the cases where the GPU
	 * is writing directly to fb0, the framebuffer pitch
	 * also needs to be 32 pixel aligned
	 */

	if (fb_index == 0)
		return ALIGN(xres, 32) * bpp;
	else
		return xres * bpp;
}

void mdp4_mipi_vsync_enable(struct msm_fb_data_type *mfd,
		struct mdp4_overlay_pipe *pipe, int which)
{
	uint32 start_y, data, tear_en;

	tear_en = (1 << which);

	if ((mfd->use_mdp_vsync) && (mfd->ibuf.vsync_enable) &&
		(mfd->panel_info.lcd.vsync_enable)) {

		if (vsync_start_y_adjust <= pipe->dst_y)
			start_y = pipe->dst_y - vsync_start_y_adjust;
		else
			start_y = (mfd->total_lcd_lines - 1) -
				(vsync_start_y_adjust - pipe->dst_y);
		if (which == 0)
			MDP_OUTP(MDP_BASE + 0x210, start_y);	/* primary */
		else
			MDP_OUTP(MDP_BASE + 0x214, start_y);	/* secondary */

		data = inpdw(MDP_BASE + 0x20c);
		data |= tear_en;
		MDP_OUTP(MDP_BASE + 0x20c, data);
	} else {
		data = inpdw(MDP_BASE + 0x20c);
		data &= ~tear_en;
		MDP_OUTP(MDP_BASE + 0x20c, data);
	}
}

void mdp4_dsi_cmd_base_swap(int cndx, struct mdp4_overlay_pipe *pipe)
{
	struct vsycn_ctrl *vctrl;

	if (cndx >= MAX_CONTROLLER) {
		pr_err("%s: out or range: cndx=%d\n", __func__, cndx);
		return;
	}

	vctrl = &vsync_ctrl_db[cndx];
	vctrl->base_pipe = pipe;
}

static void mdp4_overlay_setup_pipe_addr(struct msm_fb_data_type *mfd,
			struct mdp4_overlay_pipe *pipe)
{
	MDPIBUF *iBuf = &mfd->ibuf;
	struct fb_info *fbi;
	int bpp;
	uint8 *src;

	/* whole screen for base layer */
	src = (uint8 *) iBuf->buf;
	fbi = mfd->fbi;

	if (pipe->is_3d) {
		bpp = fbi->var.bits_per_pixel / 8;
		pipe->src_height = pipe->src_height_3d;
		pipe->src_width = pipe->src_width_3d;
		pipe->src_h = pipe->src_height_3d;
		pipe->src_w = pipe->src_width_3d;
		pipe->dst_h = pipe->src_height_3d;
		pipe->dst_w = pipe->src_width_3d;
		pipe->srcp0_ystride = msm_fb_line_length(0,
						pipe->src_width, bpp);
	} else {
		 /* 2D */
		pipe->src_height = fbi->var.yres;
		pipe->src_width = fbi->var.xres;
		pipe->src_h = fbi->var.yres;
		pipe->src_w = fbi->var.xres;
		pipe->dst_h = fbi->var.yres;
		pipe->dst_w = fbi->var.xres;
		pipe->srcp0_ystride = fbi->fix.line_length;
	}
	pipe->src_y = 0;
	pipe->src_x = 0;
	pipe->dst_y = 0;
	pipe->dst_x = 0;
	pipe->srcp0_addr = (uint32)src;
}

static void mdp4_overlay_update_dsi_cmd(struct msm_fb_data_type *mfd)
{
	int ptype;
	struct mdp4_overlay_pipe *pipe;
	int ret;
	int cndx = 0;
	struct vsycn_ctrl *vctrl;


	if (mfd->key != MFD_KEY)
		return;

	vctrl = &vsync_ctrl_db[cndx];

	if (vctrl->base_pipe == NULL) {
		ptype = mdp4_overlay_format2type(mfd->fb_imgType);
		if (ptype < 0)
			printk(KERN_INFO "%s: format2type failed\n", __func__);
		pipe = mdp4_overlay_pipe_alloc(ptype, MDP4_MIXER0);
		if (pipe == NULL) {
			printk(KERN_INFO "%s: pipe_alloc failed\n", __func__);
			return;
		}
		pipe->pipe_used++;
		pipe->mixer_stage  = MDP4_MIXER_STAGE_BASE;
		pipe->mixer_num  = MDP4_MIXER0;
		pipe->src_format = mfd->fb_imgType;
		mdp4_overlay_panel_mode(pipe->mixer_num, MDP4_PANEL_DSI_CMD);
		ret = mdp4_overlay_format2pipe(pipe);
		if (ret < 0)
			printk(KERN_INFO "%s: format2type failed\n", __func__);

		vctrl->base_pipe = pipe; /* keep it */
		mdp4_init_writeback_buf(mfd, MDP4_MIXER0);
		pipe->ov_blt_addr = 0;
		pipe->dma_blt_addr = 0;
	} else {
		pipe = vctrl->base_pipe;
	}

#ifdef	CONFIG_SHLCDC_BOARD /* CUST_ID_00122 */
	/* 548 = ((mfd->total_lcd_lines - 1) - vsync_start_y_adjust + 14) % (mfd->total_lcd_lines - 1) */
	MDP_OUTP(MDP_BASE + 0x021c, 548);
#else
	MDP_OUTP(MDP_BASE + 0x021c, 10); /* read pointer */
#endif

	/*
	 * configure dsi stream id
	 * dma_p = 0, dma_s = 1
	 */
	MDP_OUTP(MDP_BASE + 0x000a0, 0x10);
	/* disable dsi trigger */
	MDP_OUTP(MDP_BASE + 0x000a4, 0x00);

	mdp4_overlay_setup_pipe_addr(mfd, pipe);

	mdp4_overlay_rgb_setup(pipe);

	mdp4_overlay_reg_flush(pipe, 1);

	mdp4_mixer_stage_up(pipe, 0);

	mdp4_overlayproc_cfg(pipe);

	mdp4_overlay_dmap_xy(pipe);

	mdp4_overlay_dmap_cfg(mfd, 0);

#ifdef CONFIG_SHLCDC_BOARD /* CUST_ID_00147 */
	mdp4_overlay_mdp_pipe_req(pipe,mfd);

	mdp4_overlay_mdp_perf_req(mfd,pipe);
#endif /* CONFIG_SHLCDC_BOARD */

	wmb();
}

/* 3D side by side */
void mdp4_dsi_cmd_3d_sbys(struct msm_fb_data_type *mfd,
				struct msmfb_overlay_3d *r3d)
{
	struct fb_info *fbi;
	int bpp;
	uint8 *src = NULL;
	int cndx = 0;
	struct vsycn_ctrl *vctrl;
	struct mdp4_overlay_pipe *pipe;

	vctrl = &vsync_ctrl_db[cndx];
	pipe = vctrl->base_pipe;

	if (pipe == NULL)
		return;

	if (pipe->pipe_used == 0 ||
			pipe->mixer_stage != MDP4_MIXER_STAGE_BASE) {
		pr_err("%s: NOT baselayer\n", __func__);
		return;
	}

	pipe->is_3d = r3d->is_3d;
	pipe->src_height_3d = r3d->height;
	pipe->src_width_3d = r3d->width;

	if (pipe->is_3d)
		mdp4_overlay_panel_3d(pipe->mixer_num, MDP4_3D_SIDE_BY_SIDE);
	else
		mdp4_overlay_panel_3d(pipe->mixer_num, MDP4_3D_NONE);

	fbi = mfd->fbi;
	if (pipe->is_3d) {
		bpp = fbi->var.bits_per_pixel / 8;
		pipe->src_height = pipe->src_height_3d;
		pipe->src_width = pipe->src_width_3d;
		pipe->src_h = pipe->src_height_3d;
		pipe->src_w = pipe->src_width_3d;
		pipe->dst_h = pipe->src_height_3d;
		pipe->dst_w = pipe->src_width_3d;
		pipe->srcp0_ystride = msm_fb_line_length(0,
					pipe->src_width, bpp);
	} else {
		 /* 2D */
		pipe->src_height = fbi->var.yres;
		pipe->src_width = fbi->var.xres;
		pipe->src_h = fbi->var.yres;
		pipe->src_w = fbi->var.xres;
		pipe->dst_h = fbi->var.yres;
		pipe->dst_w = fbi->var.xres;
		pipe->srcp0_ystride = fbi->fix.line_length;
	}
	pipe->src_y = 0;
	pipe->src_x = 0;
	pipe->dst_y = 0;
	pipe->dst_x = 0;
	pipe->srcp0_addr = (uint32)src;

	mdp_pipe_ctrl(MDP_CMD_BLOCK, MDP_BLOCK_POWER_ON, FALSE);
	mdp_clk_ctrl(1);

	mdp4_overlay_rgb_setup(pipe);

	mdp4_overlay_reg_flush(pipe, 1);

	mdp4_mixer_stage_up(pipe, 0);

	mdp4_overlayproc_cfg(pipe);

	mdp4_overlay_dmap_xy(pipe);

	mdp4_overlay_dmap_cfg(mfd, 0);

	mdp4_mixer_stage_commit(pipe->mixer_num);
	/* MDP cmd block disable */
	mdp_clk_ctrl(0);
	mdp_pipe_ctrl(MDP_CMD_BLOCK, MDP_BLOCK_POWER_OFF, FALSE);
}

void mdp4_dsi_cmd_blt_start(struct msm_fb_data_type *mfd)
{
	mdp4_dsi_cmd_do_blt(mfd, 1);
}

void mdp4_dsi_cmd_blt_stop(struct msm_fb_data_type *mfd)
{
	mdp4_dsi_cmd_do_blt(mfd, 0);
}

void mdp4_dsi_cmd_overlay_blt(struct msm_fb_data_type *mfd,
					struct msmfb_overlay_blt *req)
{
	mdp4_dsi_cmd_do_blt(mfd, req->enable);
}

int mdp4_dsi_cmd_on(struct platform_device *pdev)
{
	int ret = 0;
	int cndx = 0;
	struct msm_fb_data_type *mfd;
	struct vsycn_ctrl *vctrl;

	pr_debug("%s+: pid=%d\n", __func__, current->pid);

	mfd = (struct msm_fb_data_type *)platform_get_drvdata(pdev);
#ifdef CONFIG_SHLCDC_BOARD /* CUST_ID_00153 */
	if (!(mfd->cont_splash_done)) {
		mdp_clk_ctrl(0);
	}
#endif /* CONFIG_SHLCDC_BOARD */

	if (!(mfd->cont_splash_done)) {
		/* Clks are enabled in probe.
		   Disabling clocks now */
		mdp_clk_ctrl(0);
		mfd->cont_splash_done = 1;
	}

	vctrl = &vsync_ctrl_db[cndx];
	vctrl->mfd = mfd;
	vctrl->dev = mfd->fbi->dev;

	mdp_clk_ctrl(1);
	mdp4_overlay_update_dsi_cmd(mfd);
	mdp_clk_ctrl(0);

	mdp4_iommu_attach();

	atomic_set(&vctrl->suspend, 0);

#ifdef CONFIG_SHLCDC_BOARD /* CUST_ID_00130 */
	clear_vctrl_koff_done_cnt();
#endif /* CUST_ID_00130 */

	pr_debug("%s-:\n", __func__);
	return ret;
}

int mdp4_dsi_cmd_off(struct platform_device *pdev)
{
	int ret = 0;
	int cndx = 0;
	struct msm_fb_data_type *mfd;
	struct vsycn_ctrl *vctrl;
	struct mdp4_overlay_pipe *pipe;
	struct vsync_update *vp;
	int undx;
	int need_wait, cnt;
#ifdef CONFIG_SHLCDC_BOARD  /* CUST_ID_00153 */
	unsigned long flags;
#endif /* CONFIG_SHLCDC_BOARD */

	pr_debug("%s+: pid=%d\n", __func__, current->pid);

	mfd = (struct msm_fb_data_type *)platform_get_drvdata(pdev);

	vctrl = &vsync_ctrl_db[cndx];
	pipe = vctrl->base_pipe;
	if (pipe == NULL) {
		pr_err("%s: NO base pipe\n", __func__);
		return ret;
	}

	need_wait = 0;
	mutex_lock(&vctrl->update_lock);
	atomic_set(&vctrl->suspend, 1);

	complete_all(&vctrl->vsync_comp);

#ifdef	CONFIG_SHLCDC_BOARD /* CUST_ID_00121 */
	vctrl->fpslow_count = 0;
#endif /* CUST_ID_00121 */
#ifdef CONFIG_SHLCDC_BOARD  /* CUST_ID_00107 */
	complete_all(&vctrl->show_comp);
#endif  /* CONFIG_SHLCDC_BOARD */

	pr_debug("%s: clk=%d pan=%d\n", __func__,
			vctrl->clk_enabled, vctrl->pan_display);
	if (vctrl->clk_enabled)
		need_wait = 1;
	mutex_unlock(&vctrl->update_lock);

	cnt = 0;
	if (need_wait) {
		while (vctrl->clk_enabled) {
#ifdef CONFIG_SHLCDC_BOARD /* CUST_ID_00136 */
			mipi_sharp_delay_us(20000);
#else
			msleep(20);
#endif /* CUST_ID_00136 */
			cnt++;
			if (cnt > 10)
				break;
		}
	}

#ifdef CONFIG_SHLCDC_BOARD  /* CUST_ID_00153 */
	if (cnt > 10) {
		spin_lock_irqsave(&vctrl->spin_lock, flags);
		vctrl->clk_control = 0;
		vctrl->clk_enabled = 0;
		vctrl->expire_tick = 0;
		spin_unlock_irqrestore(&vctrl->spin_lock, flags);
		mipi_dsi_clk_cfg(0);
		mdp_clk_ctrl(0);
		pr_err("%s: Error, SET_CLK_OFF by force\n", __func__);
	}
#else
	/* message for system suspnded */
	if (cnt > 10)
		pr_err("%s:Error,  mdp clocks NOT off\n", __func__);
	else
		pr_debug("%s: mdp clocks off at cnt=%d\n", __func__, cnt);
#endif /* CONFIG_SHLCDC_BOARD */

#ifdef CONFIG_SHLCDC_BOARD /* CUST_ID_00116 */
	if(!mipi_sharp_is_recovery()){
		/* sanity check, free pipes besides base layer */
		mdp4_overlay_unset_mixer(pipe->mixer_num);
		mdp4_mixer_stage_down(pipe, 1);
		mdp4_overlay_pipe_free(pipe);
		vctrl->base_pipe = NULL;
	}
#else
	/* sanity check, free pipes besides base layer */
	mdp4_overlay_unset_mixer(pipe->mixer_num);
	mdp4_mixer_stage_down(pipe, 1);
	mdp4_overlay_pipe_free(pipe);
	vctrl->base_pipe = NULL;
#endif

#ifdef CONFIG_SHLCDC_BOARD  /* CUST_ID_00153 */
	if (vctrl->vsync_enabled) {
		vsync_irq_disable(INTR_PRIMARY_RDPTR, MDP_PRIM_RDPTR_TERM);
		vctrl->vsync_enabled = 0;
	}
#endif /* CONFIG_SHLCDC_BOARD */
	undx =  vctrl->update_ndx;
	vp = &vctrl->vlist[undx];
	if (vp->update_cnt) {
		/*
		 * pipe's iommu will be freed at next overlay play
		 * and iommu_drop statistic will be increased by one
		 */
		vp->update_cnt = 0;     /* empty queue */
	}
#ifdef CONFIG_SHLCDC_BOARD /* CUST_ID_00130 */
	clear_vctrl_koff_done_cnt();
#endif /* CUST_ID_00130 */

#ifdef CONFIG_SHLCDC_BOARD /* CUST_ID_00146 */
	mipi_dsi_latency_allow_collapse();
	mdp_qos_deny_collapse = 0;
#endif /* CONFIG_SHLCDC_BOARD */

	pr_debug("%s-:\n", __func__);
	return ret;
}

void mdp_dsi_cmd_overlay_suspend(struct msm_fb_data_type *mfd)
{
	int cndx = 0;
	struct vsycn_ctrl *vctrl;
	struct mdp4_overlay_pipe *pipe;

	vctrl = &vsync_ctrl_db[cndx];
	pipe = vctrl->base_pipe;
	/* dis-engage rgb0 from mixer0 */
	if (pipe) {
		if (mfd->ref_cnt == 0) {
			/* adb stop */
			if (pipe->pipe_type == OVERLAY_TYPE_BF)
				mdp4_overlay_borderfill_stage_down(pipe);

			/* pipe == rgb1 */
			mdp4_overlay_unset_mixer(pipe->mixer_num);
			vctrl->base_pipe = NULL;
		} else {
			mdp4_mixer_stage_down(pipe, 1);
			mdp4_overlay_iommu_pipe_free(pipe->pipe_ndx, 1);
		}
	}
}

#ifdef CONFIG_SHLCDC_BOARD /* CUST_ID_00120 */
int mdp4_dsi_cmd_overlay_get_err_flg(void)
{
	return mdp4_dsi_cmd_overlay_err_flg;
}
#endif /* CONFIG_SHLCDC_BOARD */

void mdp4_dsi_cmd_overlay(struct msm_fb_data_type *mfd)
{
	int cndx = 0;
	struct vsycn_ctrl *vctrl;
	struct mdp4_overlay_pipe *pipe;
	unsigned long flags;
	int clk_set_on = 0;
#ifdef CONFIG_SHLCDC_BOARD /* CUST_ID_00120 */
	mdp4_dsi_cmd_overlay_err_flg = 0;
#endif /* CONFIG_SHLCDC_BOARD */

	vctrl = &vsync_ctrl_db[cndx];

#ifdef CONFIG_SHLCDC_BOARD /* CUST_ID_00120 */
	if (!mfd->panel_power_on){
		mdp4_dsi_cmd_overlay_err_flg = 1;
		return;
	}
#else
	if (!mfd->panel_power_on)
		return;
#endif /* CONFIG_SHLCDC_BOARD */

	pipe = vctrl->base_pipe;
	if (pipe == NULL) {
		pr_err("%s: NO base pipe\n", __func__);
#ifdef CONFIG_SHLCDC_BOARD /* CUST_ID_00120 */
		mdp4_dsi_cmd_overlay_err_flg = 1;
#endif /* CONFIG_SHLCDC_BOARD */
		return;
	}

	mutex_lock(&vctrl->update_lock);
	if (atomic_read(&vctrl->suspend)) {
		mutex_unlock(&vctrl->update_lock);
		mutex_unlock(&mfd->dma->ov_mutex);
#ifdef CONFIG_SHLCDC_BOARD /* CUST_ID_00120 */
		mdp4_dsi_cmd_overlay_err_flg = 1;
#endif /* CONFIG_SHLCDC_BOARD */
		pr_err("%s: suspended, no more pan display\n", __func__);
		return;
	}

	spin_lock_irqsave(&vctrl->spin_lock, flags);
	vctrl->clk_control = 0;
	vctrl->pan_display++;
	if (!vctrl->clk_enabled) {
		clk_set_on = 1;
		vctrl->clk_enabled = 1;
		vctrl->expire_tick = VSYNC_EXPIRE_TICK;
	}
	spin_unlock_irqrestore(&vctrl->spin_lock, flags);

	if (clk_set_on) {
		pr_debug("%s: SET_CLK_ON\n", __func__);
		mipi_dsi_clk_cfg(1);
		mdp_clk_ctrl(1);
		vsync_irq_enable(INTR_PRIMARY_RDPTR, MDP_PRIM_RDPTR_TERM);
	}

	mutex_unlock(&vctrl->update_lock);

	if (pipe->mixer_stage == MDP4_MIXER_STAGE_BASE) {
		mdp4_mipi_vsync_enable(mfd, pipe, 0);
		mdp4_overlay_setup_pipe_addr(mfd, pipe);
		mdp4_dsi_cmd_pipe_queue(0, pipe);
	}

#ifdef CONFIG_SHLCDC_BOARD /* CUST_ID_00155 */
	mutex_lock(&mfd->dma->ov_mutex);
	mdp4_overlay_mdp_perf_upd(mfd, 1);
#else
	mdp4_overlay_mdp_perf_upd(mfd, 1);

	mutex_lock(&mfd->dma->ov_mutex);
#endif /* CUST_ID_00155 */
	mdp4_dsi_cmd_pipe_commit(cndx, 0);
	mdp4_overlay_mdp_perf_upd(mfd, 0);
	mutex_unlock(&mfd->dma->ov_mutex);
}

#ifdef CONFIG_SHLCDC_BOARD /* CUST_ID_00146 */
void mdp4_dsi_cmd_init(void)
{
	mdp_qos_wq = create_singlethread_workqueue("mdp_qos_wq");

	if(mdp_qos_wq) {
		INIT_WORK(&mdp_qos_work, mdp_qos_work_handler);
	} else {
		pr_err("%s: mdp_qos_wq, create err\n", __func__);
	}
}

void mdp4_dsi_cmd_exit(void)
{
	printk("[FB_DBG]%s :START\n", __func__);

	if(mdp_qos_wq) {
		flush_workqueue(mdp_qos_wq);
		destroy_workqueue(mdp_qos_wq);
	}
}
#endif /* CONFIG_SHLCDC_BOARD */
