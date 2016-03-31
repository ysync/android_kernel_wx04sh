/* drivers/sharp/phsmdm/phs_mdm.c
  *
  * Copyright (C) 2013 SHARP CORPORATION All rights reserved.
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

#include <linux/module.h>
#include <linux/miscdevice.h>
#include <linux/fs.h>
#include <linux/poll.h>
#include <linux/gpio.h>
#include <linux/delay.h>
#include <linux/tty.h>
#include <linux/spinlock.h>
#include <linux/sched.h>
#include <linux/slab.h>
#include <linux/platform_device.h>
#include <linux/interrupt.h>
#include <linux/pm.h>
#include <linux/ioport.h>

#include <mach/msm_iomap.h>
#include <asm/io.h>
#include <asm/termios.h>

#define	USE_DEBUG_INFO
#ifdef	USE_DEBUG_INFO
#include <linux/proc_fs.h>
#endif	/* USE_DEBUG_INFO */

#include <sharp/sh_boot_manager.h>
#include <sharp/phs_mdm.h>

#define DEBUG_LOG
#include "phsmdm_def.h"

/*#define       DEBUG */
#define	PHS_MDM_STATS_DEBUG	1

#ifdef DEBUG_LOG
int g_DEBUG_LOG = 0;
#define DBG_PRINTK(x, args...) if (g_DEBUG_LOG) printk(x, ## args)
#else
#define DBG_PRINTK(x, args...)
#endif

#ifdef	DEBUG_LOG
#define UARTDM_TXFS_ADDR 0x4c
#define UARTDM_RXFS_ADDR 0x50
#endif	/* DEBUG */
#define	GSBI5_CTS_GPIO	24
#define	GSBI5_RTS_GPIO	25

int g_mdmlogger = 0;


#include <mach/qdsp6v2/apr_tal.h>
extern struct apr_svc_ch_dev apr_svc_ch[APR_DL_MAX][APR_DEST_MAX][APR_CLIENT_MAX];

#define	DELAY_AFTER_MODEM_IS_UP		1000

#ifdef DEBUG
#define	_gpio_value_is(id, v) \
{ \
	if (gpio_get_value(id) != v) DBG_PRINTK("%s(): gpio_get_value(" #id ")=!%d, cfg/v=%04x/%04x\n", \
		__func__, v, \
		ioread32(MSM_TLMM_BASE + 0x1000 + (id * 16)), \
		ioread32(MSM_TLMM_BASE + 0x1000 + (id * 16) + 4)); \
}

#define	_gpio_set_value(id, v) \
{ \
	gpio_set_value(id, v); \
	if (gpio_get_value(id) != v) DBG_PRINTK("%s(): gpio_set_value(" #id ")!=%d, cfg/v=%04x/%04x\n", \
		__func__, v, \
		ioread32(MSM_TLMM_BASE + 0x1000 + (id * 16)), \
		ioread32(MSM_TLMM_BASE + 0x1000 + (id * 16) + 4)); \
}

#define	_gpio_request(id, name) \
{ \
	int r = gpio_request(id, name); \
	if (r) DBG_PRINTK("%s(): gpio_request(" #id ")=%d, cfg/v=%04x/%04x\n", \
		__func__, r, \
		ioread32(MSM_TLMM_BASE + 0x1000 + (id * 16)), \
		ioread32(MSM_TLMM_BASE + 0x1000 + (id * 16) + 4)); \
}
#else	/* DEBUG */
#define	_gpio_value_is(id, v)
#define	_gpio_set_value(id, v)	gpio_set_value(id, v)
#define	_gpio_request(id, name)	gpio_request(id, name)
#endif	/* DEBUG */

struct msm_phsmdm_platform_data *gpdata = NULL;

#ifdef	USE_DEBUG_INFO
static struct proc_dir_entry	*proc_entry;
#endif	/* USE_DEBUG_INFO */

#ifdef	DEBUG_LOG
static const char *disp[][2][2] = {
	{{	"disp1", "====="	}, {	"DISP1", "-----"	}},
	{{	"disp2", "====="	}, {	"DISP2", "-----"	}},
	{{	"disp3", "====="	}, {	"DISP3", "-----"	}}
};
#endif	/* DEBUG_LOG */

#ifdef	PHS_MDM_STATS_DEBUG
void phs_mdm_dsp_stat(char *info)
{
	struct msm_phsmdm_platform_data *pdata = gpdata;

	DBG_PRINTK("phs_mdm: open=%d: IWAIT=%s: DCD:%d RI:%d DISP123:%d/%d/%d [%s/%s/%s/%s/%s/%s/%s/%s] - %s\n",
			pdata ? pdata->num_open : -1,
			pdata ? (!list_empty(&pdata->phsmdm_wait.task_list) ? "YES" : "no ") : "---",
			pdata ? pdata->icount.dcd : -1,
			pdata ? pdata->icount.ri : -1,
			pdata ? pdata->icount.disp1 : -1, pdata ? pdata->icount.disp2 : -1, pdata ? pdata->icount.disp3 : -1,
			gpio_get_value(SHPHS_TIOCM_DTR) == 0 ? "DTR" : "---",
			gpio_get_value(SHPHS_TIOCM_CAR) == 0 ? "CD" : "--",
			gpio_get_value(SHPHS_TIOCM_RI) == 0 ? "RI" : "--",
			disp[0][pdata->enable_report_disp][gpio_get_value(SHPHS_DISP1)],
			disp[1][pdata->enable_report_disp][gpio_get_value(SHPHS_DISP2)],
			disp[2][pdata->enable_report_disp][gpio_get_value(SHPHS_DISP3)],
			gpio_get_value(GSBI5_CTS_GPIO) ? "---" : "CTS",
			gpio_get_value(GSBI5_RTS_GPIO) ? "---" : "RTS",
			info);
}

EXPORT_SYMBOL(phs_mdm_dsp_stat);

#endif	/* PHS_MDM_STATS_DEBUG */

static irqreturn_t phsmdm_dcd_isr(int irq, void *dev)
{
	struct msm_phsmdm_platform_data *pdata = (struct msm_phsmdm_platform_data*)dev;

	pdata->icount.dcd ++;
	DBG_PRINTK( "%s[%d]\n", __func__, pdata->icount.dcd);
	wake_up_interruptible(&pdata->phsmdm_wait);

	return IRQ_HANDLED;
}

static irqreturn_t phsmdm_disp1_isr(int irq, void *dev)
{
	struct msm_phsmdm_platform_data *pdata = (struct msm_phsmdm_platform_data*)dev;

	pdata->icount.disp1 ++;
	DBG_PRINTK( "%s[%d]\n", __func__, pdata->icount.disp1);
	wake_up_interruptible(&pdata->phsmdm_wait);

	return IRQ_HANDLED;
}

static irqreturn_t phsmdm_disp2_isr(int irq, void *dev)
{
	struct msm_phsmdm_platform_data *pdata = (struct msm_phsmdm_platform_data*)dev;
	

	pdata->icount.disp2 ++;
	DBG_PRINTK( "%s[%d]\n", __func__, pdata->icount.disp2);
	wake_up_interruptible(&pdata->phsmdm_wait);

	return IRQ_HANDLED;
}

static irqreturn_t phsmdm_disp3_isr(int irq, void *dev)
{
	struct msm_phsmdm_platform_data *pdata = (struct msm_phsmdm_platform_data*)dev;


	pdata->icount.disp3 ++;
	DBG_PRINTK( "%s[%d]\n", __func__, pdata->icount.disp3);
	wake_up_interruptible(&pdata->phsmdm_wait);

	return IRQ_HANDLED;
}

static irqreturn_t phsmdm_wakeup_isr(int irq, void *dev)
{
	struct msm_phsmdm_platform_data *pdata = (struct msm_phsmdm_platform_data*)dev;

	pdata->icount.ri ++;
	DBG_PRINTK( "%s[%d]\n", __func__, pdata->icount.ri);
	wake_up_interruptible(&pdata->phsmdm_wait);

	return IRQ_HANDLED;
}

static int phsmdm_open(struct inode *inode, struct file *filp)
{
	struct msm_phsmdm_platform_data *pdata = gpdata;
	u32 gpio_config = 0;

#ifdef DEBUG
	int gpio140;
	int gpio124;
#endif

	mutex_lock(&pdata->phsmdm_lock);

	DBG_PRINTK( "%s\n", __func__);

	if (pdata->pwrcont_num != PHSMDM_PHS_PWRCONT_PP2) {
		if (!pdata->is_not_first_open) {
			if (apr_svc_ch[APR_DL_SMD][APR_DEST_MODEM][APR_CLIENT_VOICE].dest_state == 1) {
				DBG_PRINTK("%s(): Modem Up Probe comes.\n", __func__);
				msleep_interruptible(DELAY_AFTER_MODEM_IS_UP);
				DBG_PRINTK("%s(): Modem Up complete.\n", __func__);
			} else {
				mutex_unlock(&pdata->phsmdm_lock);
				dev_err(pdata->dev, " Modem is not Up yet.\n");
				return -EPERM;
			}
		}
		pdata->is_not_first_open = 1;
	}

	pdata->num_open++;
	if (pdata->num_open == 1) {
		gpio_config = GPIO_CFG(pdata->pwrcont_num, 0, GPIO_CFG_OUTPUT, GPIO_CFG_NO_PULL, GPIO_CFG_6MA);
		gpio_tlmm_config( gpio_config, GPIO_CFG_ENABLE );
		_gpio_request(PHSMDM_PHS_PWRRESET,"PWRRESET");
		_gpio_request(SHPHS_TIOCM_DTR,"DTR");
		_gpio_request(SHPHS_TIOCM_CAR,"DCD");
		_gpio_request(SHPHS_TIOCM_RI,"RI");
		_gpio_request(SHPHS_DISP1,"DISP1");
		_gpio_request(SHPHS_DISP2,"DISP2");
		_gpio_request(SHPHS_DISP3,"DISP3");

		_gpio_set_value(pdata->pwrcont_num, 1);
		mdelay(PHSMDM_OPEN_TIME);
		_gpio_set_value(PHSMDM_PHS_DTR, 0);
		_gpio_set_value(PHSMDM_PHS_PWRRESET, 1);
/*		mdelay(PHSMDM_OPEN_TIME);	*/

#ifdef DEBUG
		gpio140 = gpio_get_value(PHSMDM_PHS_PWRCONT);
		gpio124 = gpio_get_value(PHSMDM_PHS_PWRRESET);
		DBG_PRINTK("gpio140:%d gpio124:%d\n", gpio140, gpio124);
#endif
		pdata->enable_report_disp = 1;	/* default on */

		enable_irq(pdata->phs_irq.dcd);
		enable_irq(pdata->phs_irq.disp1);
		enable_irq(pdata->phs_irq.disp2);
		enable_irq(pdata->phs_irq.disp3);
		enable_irq(pdata->phs_irq.ri);
	}

	mutex_unlock(&pdata->phsmdm_lock);

	printk("%s(): PHS Modem Is Up\n", __func__);
	return 0;
	
}

static int phsmdm_close(struct inode *inode, struct file *filp)
{
	struct msm_phsmdm_platform_data *pdata = gpdata;

#ifdef DEBUG
	int gpio140;
	int gpio124;
#endif

	mutex_lock(&pdata->phsmdm_lock);

	pdata->num_open--;
	if (pdata->num_open == 0) {
		_gpio_set_value(pdata->pwrcont_num, 0);
		_gpio_set_value(PHSMDM_PHS_PWRRESET, 0);

#ifdef DEBUG
		gpio140 = gpio_get_value(PHSMDM_PHS_PWRCONT);
		gpio124 = gpio_get_value(PHSMDM_PHS_PWRRESET);
		DBG_PRINTK("gpio140:%d gpio124:%d\n", gpio140, gpio124);
#endif
		disable_irq(pdata->phs_irq.dcd);
		if (pdata->enable_report_disp) {
			disable_irq(pdata->phs_irq.disp1);
			disable_irq(pdata->phs_irq.disp2);
			disable_irq(pdata->phs_irq.disp3);
		}
		disable_irq(pdata->phs_irq.ri);

		gpio_free(PHSMDM_PHS_PWRRESET);
		gpio_free(PHSMDM_PHS_DTR);
		gpio_free(SHPHS_TIOCM_CAR);
		gpio_free(SHPHS_TIOCM_RI);
		gpio_free(SHPHS_DISP1);
		gpio_free(SHPHS_DISP2);
		gpio_free(SHPHS_DISP3);
	}
	mutex_unlock(&pdata->phsmdm_lock);

	printk("%s(): PHS Modem Is Down\n", __func__);
	return 0;

}

static int phsmdm_get_value(struct msm_phsmdm_platform_data *pdata, unsigned long arg)
{
	int rc = 0;
	unsigned int value = 0;
	unsigned int cts_v;

	if(gpio_get_value(SHPHS_TIOCM_DTR) == 0)
	{
		value |= TIOCM_DTR;
	}
	if(gpio_get_value(SHPHS_TIOCM_CAR) == 0)
	{
		value |= TIOCM_CD;
	}
	if(gpio_get_value(SHPHS_TIOCM_RI) == 0)
	{
		value |= TIOCM_RI;
	}
	if(gpio_get_value(SHPHS_DISP1) == 0)
	{
		value |= TIOCM_DISP1;
	}
	if(gpio_get_value(SHPHS_DISP2) == 0)
	{
		value |= TIOCM_DISP2;
	}
	if(gpio_get_value(SHPHS_DISP3) == 0)
	{
		value |= TIOCM_DISP3;
	}

	cts_v = gpio_get_value(GSBI5_CTS_GPIO);
	value |=  TIOCM_DSR | (cts_v ? 0 : TIOCM_CTS);

	DBG_PRINTK("%s(): %s/%s/%s/%s/%s/%s/%s/%s\n",
			__func__,
			value & TIOCM_DTR ? "DTR" : "---",
			value & TIOCM_CD ? "CD" : "--",
			value & TIOCM_RI ? "RI" : "--",
			disp[0][pdata->enable_report_disp][(value & TIOCM_DISP1) ? 0 : 1],
			disp[1][pdata->enable_report_disp][(value & TIOCM_DISP2) ? 0 : 1],
			disp[2][pdata->enable_report_disp][(value & TIOCM_DISP3) ? 0 : 1],
			value & TIOCM_CTS ? "CTS" : "---",
			gpio_get_value(GSBI5_RTS_GPIO) ? "---" : "RTS");

#ifdef	DEBUG_LOG
	{
		unsigned int rxfs;
		unsigned int txfs;
		rxfs = ioread32(pdata->membase + UARTDM_RXFS_ADDR);
		txfs = ioread32(pdata->membase + UARTDM_TXFS_ADDR);

		if (txfs == 0x00000000) {
			DBG_PRINTK("%s(): TXFS=0x%08x\n", __func__, txfs);
		} else {
			DBG_PRINTK("%s(): TXFS=0x%08x,ASYNC=%d,BUFFER=%d,STATE=%d\n", __func__, txfs,
				(txfs >> 10) & 0x0f, (txfs >> 7) & 0x07, (txfs & 0x7f) | (txfs >> 14));
		}
		if (rxfs == 0x00000000) {
			DBG_PRINTK("%s(): RXFS=0x%08x\n", __func__, rxfs);
		} else {
			DBG_PRINTK("%s(): RXFS=0x%08x,ASYNC=%d,BUFFER=%d,STATE=%d\n", __func__, rxfs,
				(rxfs >> 10) & 0x0f, (rxfs >> 7) & 0x07, (rxfs & 0x7f) | (rxfs >> 14));
		}
	}
#endif	/* DEBUG_LOG */

	if (copy_to_user((u8*)arg, (u8*)&value, sizeof(unsigned int)) != 0) {
		rc = -EFAULT;
	}

	return rc;
}

static int phsmdm_set_value(struct msm_phsmdm_platform_data *pdata, unsigned long arg)
{
	int ret = 0;
	unsigned int args = 0;
	
	ret = copy_from_user((u8*)&args, (u8*)arg, sizeof(unsigned int));

	if(args & TIOCM_DTR)
	{
		DBG_PRINTK("%s set H\n", __func__);
		gpio_set_value(SHPHS_TIOCM_DTR, 0);
	}
	else
	{
		DBG_PRINTK("%s set L\n", __func__);
		gpio_set_value(SHPHS_TIOCM_DTR, 1);
	}

	return ret;
}

static int phsmdm_tiocmbic(struct msm_phsmdm_platform_data *pdata, unsigned long arg)
{
	int ret = 0;
	unsigned int args = 0;
	unsigned long flags;
	
	ret = copy_from_user((u8*)&args, (u8*)arg, sizeof(unsigned int));

	DBG_PRINTK("%s(0x%02x)\n", __func__, args);
	
	if(args & TIOCM_DTR)
	{
		spin_lock_irqsave(&pdata->lock, flags);
		gpio_set_value(SHPHS_TIOCM_DTR, 1);
		spin_unlock_irqrestore(&pdata->lock, flags);
	}
	
	return 0;
}

static int phsmdm_tiocmbis(struct msm_phsmdm_platform_data *pdata, unsigned long arg)
{
	int ret = 0;
	unsigned int args = 0;
	unsigned long flags;
	
	ret = copy_from_user((u8*)&args, (u8*)arg, sizeof(unsigned int));

	DBG_PRINTK("%s(0x%02x)\n", __func__, args);

	if(args & TIOCM_DTR)
	{
		spin_lock_irqsave(&pdata->lock, flags);
		gpio_set_value(SHPHS_TIOCM_DTR, 0);
		spin_unlock_irqrestore(&pdata->lock, flags);
	}
	
	return 0;
}

/* #67 */
struct phsmdm_icount g_prev = {0};

static int phsmdm_wait_modem_status(struct msm_phsmdm_platform_data *pdata, unsigned long arg)
{
	int ret = 0;
	struct phsmdm_icount now;
	unsigned int args = 0;
	DECLARE_WAITQUEUE(wait, current);

	DBG_PRINTK("%s(0x%02lx)\n", __func__, arg);

	args = (unsigned int)arg;

	spin_lock_irq(&pdata->lock);
	memcpy(&now, &pdata->icount, sizeof(pdata->icount));
	spin_unlock_irq(&pdata->lock);

	if (((args & TIOCM_CD   ) && (g_prev.dcd   != now.dcd  )) ||
		((args & TIOCM_RI   ) && (g_prev.ri    != now.ri   )) ||
		((args & TIOCM_DISP1) && (g_prev.disp1 != now.disp1)) ||
		((args & TIOCM_DISP2) && (g_prev.disp2 != now.disp2)) ||
		((args & TIOCM_DISP3) && (g_prev.disp3 != now.disp3)) ||
		((args & (TIOCM_DISP1 | TIOCM_DISP2 | TIOCM_DISP3)) && pdata->report_resume))
	{
		DBG_PRINTK("%s(0x%02lx): ret=%d, resume=%d\n", __func__, arg, ret, pdata->report_resume);

		if (args & TIOCM_CD   ) g_prev.dcd   = now.dcd;
		if (args & TIOCM_RI   ) g_prev.ri    = now.ri;
		if (args & TIOCM_DISP1) g_prev.disp1 = now.disp1;
		if (args & TIOCM_DISP2) g_prev.disp2 = now.disp2;
		if (args & TIOCM_DISP3) g_prev.disp3 = now.disp3;

		if (args & (TIOCM_DISP1 | TIOCM_DISP2 | TIOCM_DISP3)) {
			pdata->report_resume = 0;
		}

		return 0;
	}

	add_wait_queue(&pdata->phsmdm_wait, &wait);
	for(;;)
	{
		spin_lock_irq(&pdata->lock);
		memcpy(&now, &pdata->icount, sizeof(pdata->icount));
		spin_unlock_irq(&pdata->lock);
		set_current_state(TASK_INTERRUPTIBLE);
		if (((args & TIOCM_CD   ) && (g_prev.dcd   != now.dcd  )) ||
			((args & TIOCM_RI   ) && (g_prev.ri    != now.ri   )) ||
			((args & TIOCM_DISP1) && (g_prev.disp1 != now.disp1)) ||
			((args & TIOCM_DISP2) && (g_prev.disp2 != now.disp2)) ||
			((args & TIOCM_DISP3) && (g_prev.disp3 != now.disp3)))
		{
			ret = 0;
			if (args & TIOCM_CD)    g_prev.dcd   = now.dcd;
			if (args & TIOCM_RI)    g_prev.ri    = now.ri;
			if (args & TIOCM_DISP1) g_prev.disp1 = now.disp1;
			if (args & TIOCM_DISP2) g_prev.disp2 = now.disp2;
			if (args & TIOCM_DISP3) g_prev.disp3 = now.disp3;
			break;
		}

		schedule();

		if (signal_pending(current)) {
			ret = -ERESTARTSYS;
			break;
		}
	}
	current->state = TASK_RUNNING;
	remove_wait_queue(&pdata->phsmdm_wait, &wait);

	DBG_PRINTK("%s(0x%02lx): End, ret=%d\n", __func__, arg, ret);

	return ret;
}

static int phsmdm_enabledisp(struct msm_phsmdm_platform_data *pdata, unsigned long arg)
{
	int ret = 0;
	unsigned int args = 0;

	args = (unsigned int)arg;

	DBG_PRINTK("%s(): %d->%d\n", __func__, pdata->enable_report_disp, args);

	mutex_lock(&pdata->phsmdm_lock);
	if (args == 0) {	/* Disable reporting DISP */
		if (pdata->enable_report_disp == 1) {
			disable_irq(pdata->phs_irq.disp1);
			disable_irq(pdata->phs_irq.disp2);
			disable_irq(pdata->phs_irq.disp3);
		}
		pdata->enable_report_disp = 0;
	} else {			/* Enable reporting DISP */
		if (pdata->enable_report_disp == 0) {
			enable_irq(pdata->phs_irq.disp1);
			enable_irq(pdata->phs_irq.disp2);
			enable_irq(pdata->phs_irq.disp3);
		}
		pdata->enable_report_disp = 1;
	}
	mutex_unlock(&pdata->phsmdm_lock);

	return ret;
}

static int phsmdm_count(struct msm_phsmdm_platform_data *pdata, struct phsmdm_icount *data)
{
	data->dcd = pdata->icount.dcd;
	data->ri = pdata->icount.ri;
	data->disp1 = pdata->icount.disp1;
	data->disp2 = pdata->icount.disp2;
	data->disp3 = pdata->icount.disp3;

	return 0;
}

static int phsmdm_tiocgicount(struct msm_phsmdm_platform_data *pdata, void __user *arg)
{
	struct phsmdm_icount icount;

	memset(&icount, 0, sizeof(icount));

	phsmdm_count(pdata, &icount);
	DBG_PRINTK("%s DCD:%d RI:%d DISP1:%d DISP2:%d DISP3:%d\n",
				__func__,
				pdata->icount.dcd,
				pdata->icount.ri,
				pdata->icount.disp1,
				pdata->icount.disp2,
				pdata->icount.disp3);
	if (copy_to_user(arg, &icount, sizeof(icount)))
	{
		dev_err(pdata->dev, "error copy_to_user\n");
		return -EFAULT;
	}
	return 0;
}

static long phsmdm_ioctl(struct file *file, unsigned int cmd, unsigned long arg)
{
	struct msm_phsmdm_platform_data *pdata = gpdata;

	int rc = 0;
	void __user *p = (void __user *)arg;

	switch( cmd )
	{
		case SHPHS_TIOCMGET:
			rc = phsmdm_get_value(pdata, arg);
			break;

		case SHPHS_TIOCMSET:
			rc = phsmdm_set_value(pdata, arg);
			break;

		case SHPHS_TIOCMBIC:
			rc = phsmdm_tiocmbic(pdata, arg);
			break;

		case SHPHS_TIOCMBIS:
			rc = phsmdm_tiocmbis(pdata, arg);
			break;

		case SHPHS_TIOCMIWAIT:
			rc = phsmdm_wait_modem_status(pdata, arg);
			break;
		case SHPHS_TIOCGICOUNT:
			rc = phsmdm_tiocgicount(pdata, p);
			break;

		case SHPHS_ENABLEDISP:
			rc = phsmdm_enabledisp(pdata, arg);
			break;

		default:
			dev_err(pdata->dev, "%s(0x%04x) ???\n", __func__, cmd);
			rc = -EPERM;
			break;
	}

	return rc;
}


static int phsmdm_suspend(struct device *device)
{
	struct platform_device *pdev = to_platform_device(device);
	struct msm_phsmdm_platform_data *pdata = platform_get_drvdata(pdev);

	printk("%s\n", __func__);
	
	mutex_lock(&pdata->phsmdm_lock);

	if(pdata->num_open != 0)
	{
		disable_irq(pdata->phs_irq.dcd);
		if (pdata->enable_report_disp) {
			disable_irq(pdata->phs_irq.disp1);
			disable_irq(pdata->phs_irq.disp2);
			disable_irq(pdata->phs_irq.disp3);
		}
		disable_irq(pdata->phs_irq.ri);

		enable_irq_wake(pdata->phs_irq.ri);
	}
	mutex_unlock(&pdata->phsmdm_lock);

	return 0;
}

static int phsmdm_resume(struct device *device)
{
	struct platform_device *pdev = to_platform_device(device);
	struct msm_phsmdm_platform_data *pdata = platform_get_drvdata(pdev);

	printk("%s\n", __func__);

	mutex_lock(&pdata->phsmdm_lock);

	if(pdata->num_open != 0)
	{
		enable_irq(pdata->phs_irq.dcd);
		if (pdata->enable_report_disp) {
			enable_irq(pdata->phs_irq.disp1);
			enable_irq(pdata->phs_irq.disp2);
			enable_irq(pdata->phs_irq.disp3);
		}
		enable_irq(pdata->phs_irq.ri);

		disable_irq_wake(pdata->phs_irq.ri);
		pdata->report_resume = 1;
	}
	mutex_unlock(&pdata->phsmdm_lock);

	return 0;
}


static struct file_operations phsmdm_fops = {
	.owner			= THIS_MODULE,
	.unlocked_ioctl	= phsmdm_ioctl,
	.open			= phsmdm_open,
	.release		= phsmdm_close,
};

static struct miscdevice phsmdm_drv = {
	.minor = MISC_DYNAMIC_MINOR,
	.name = "phs_mdm",
	.fops = &phsmdm_fops,
};

static int __devinit phsmdm_probe(struct platform_device *pdev)
{
	int ret = 0;
	struct resource *phs_dcd, *phs_ri, *phs_disp1, *phs_disp2, *phs_disp3;
	struct msm_phsmdm_platform_data *pdata;
	uint8_t hwRevision;
	u32 gpio_config = 0;


	DBG_PRINTK("%s\n", __func__);
		
	ret = misc_register(&phsmdm_drv);

	if (ret) {
		dev_err(&pdev->dev, "fail to misc_register (phsmdm_drv)\n");
		return ret;
	}
	
		pdata = kzalloc(sizeof(*pdata), GFP_KERNEL);
		if (!pdata){
			dev_err(&pdev->dev,"err initialization\n");
			return -ENOMEM;
		}
	pdata->dev = &pdev->dev;
	pdata->num_open = 0;
	pdata->is_not_first_open = 0;
	pdata->enable_report_disp = 1;
	spin_lock_init(&pdata->lock);	
	mutex_init(&pdata->phsmdm_lock);

	hwRevision = sh_boot_get_hw_revision();
	if(hwRevision & PHSMDM_PP1)
	{
		pdata->pwrcont_num = PHSMDM_PHS_PWRCONT_PP1;
		DBG_PRINTK("%s PP1\n", __func__);	
	}
	else
	{
		pdata->pwrcont_num = PHSMDM_PHS_PWRCONT_PP2;
		DBG_PRINTK("%s PP2\n", __func__);	
	}
	gpio_config = GPIO_CFG(pdata->pwrcont_num, 0, GPIO_CFG_OUTPUT, GPIO_CFG_NO_PULL, GPIO_CFG_6MA);
	gpio_tlmm_config( gpio_config, GPIO_CFG_ENABLE );

	_gpio_set_value(pdata->pwrcont_num, 0);

	_gpio_set_value(PHSMDM_PHS_PWRRESET, 0);
	init_waitqueue_head(&pdata->phsmdm_wait);

#ifdef	DEBUG_LOG
	{
		struct resource *phs_addr;
		phs_addr = platform_get_resource_byname(pdev, IORESOURCE_MEM, "phsmdm_addr");
		pdata->membase = ioremap(phs_addr->start, resource_size(phs_addr));
	}
#endif	/* DEBUG_LOG */	
	phs_dcd = platform_get_resource_byname(pdev, IORESOURCE_IO, "phsmdm_dcd");
	if (unlikely(!phs_dcd))
	{
		dev_err(&pdev->dev,"error platform_get_resource_byname (phs_dcd)\n");
		return -ENXIO;
	}
	pdata->phs_irq.dcd = gpio_to_irq(phs_dcd->start);
	phs_ri = platform_get_resource_byname(pdev, IORESOURCE_IO, "phsmdm_ri");
	if (unlikely(!phs_ri))
	{
		dev_err(&pdev->dev,"error platform_get_resource_byname (phs_ri)\n");
		return -ENXIO;
	}
	pdata->phs_irq.ri = gpio_to_irq(phs_ri->start);
	phs_disp1 = platform_get_resource_byname(pdev, IORESOURCE_IO, "phsmdm_disp1");
	if (unlikely(!phs_disp1))
	{
		dev_err(&pdev->dev,"error platform_get_resource_byname (phs_disp1)\n");
		return -ENXIO;
	}
	pdata->phs_irq.disp1 = gpio_to_irq(phs_disp1->start);
	phs_disp2 = platform_get_resource_byname(pdev, IORESOURCE_IO, "phsmdm_disp2");
	if (unlikely(!phs_disp2))
	{
		dev_err(&pdev->dev,"error platform_get_resource_byname (phs_disp2)\n");
		return -ENXIO;
	}
	pdata->phs_irq.disp2 = gpio_to_irq(phs_disp2->start);
	phs_disp3 = platform_get_resource_byname(pdev, IORESOURCE_IO, "phsmdm_disp3");
	if (unlikely(!phs_disp3))
	{
		dev_err(&pdev->dev,"error platform_get_resource_byname (phs_disp3)\n");
		return -ENXIO;
	}
	pdata->phs_irq.disp3 = gpio_to_irq(phs_disp3->start);
	
	platform_set_drvdata(pdev, pdata);
	gpdata = pdata;
	
		ret = request_irq(pdata->phs_irq.dcd, phsmdm_dcd_isr,
						IRQF_TRIGGER_RISING | IRQF_TRIGGER_FALLING | IRQF_DISABLED,"phsmdm_isr", pdata);
		ret = request_irq(pdata->phs_irq.disp1, phsmdm_disp1_isr,
						IRQF_TRIGGER_RISING | IRQF_TRIGGER_FALLING | IRQF_DISABLED,"phsmdm_isr", pdata);
		ret = request_irq(pdata->phs_irq.disp2, phsmdm_disp2_isr,
						IRQF_TRIGGER_RISING | IRQF_TRIGGER_FALLING | IRQF_DISABLED,"phsmdm_isr", pdata);
		ret = request_irq(pdata->phs_irq.disp3, phsmdm_disp3_isr,
						IRQF_TRIGGER_RISING | IRQF_TRIGGER_FALLING | IRQF_DISABLED,"phsmdm_isr", pdata);

		ret = request_irq(pdata->phs_irq.ri, phsmdm_wakeup_isr,
						IRQF_TRIGGER_RISING | IRQF_TRIGGER_FALLING | IRQF_DISABLED,"phsmdm_wakeup_isr", pdata);
		disable_irq(pdata->phs_irq.dcd);
		disable_irq(pdata->phs_irq.disp1);
		disable_irq(pdata->phs_irq.disp2);
		disable_irq(pdata->phs_irq.disp3);
		disable_irq(pdata->phs_irq.ri);

	return ret;
}
static int __devexit phsmdm_remove(struct platform_device *pdev)
{
	int ret = 0;
	struct msm_phsmdm_platform_data *pdata = 	platform_get_drvdata(pdev);

	DBG_PRINTK("%s\n", __func__);
	
	misc_deregister(&phsmdm_drv);

	platform_set_drvdata(pdev, NULL);
	free_irq(pdata->phs_irq.dcd, pdata);
	free_irq(pdata->phs_irq.ri, pdata);
	free_irq(pdata->phs_irq.disp1, pdata);
	free_irq(pdata->phs_irq.disp2, pdata);
	free_irq(pdata->phs_irq.disp3, pdata);
#ifdef	DEBUG_LOG
	if (pdata->membase != NULL) {
		iounmap(pdata->membase);
		pdata->membase = NULL;
	}
#endif	/* DEBUG_LOG */
	gpdata = NULL;
	kfree(pdata);

	DBG_PRINTK("Unloaded.\n");

	return ret;
}
static const struct dev_pm_ops phsmdm_pm_ops = {
    .suspend =  phsmdm_suspend,
    .resume  =  phsmdm_resume,
};

static struct platform_driver phsmdm_driver = {
	.probe		= phsmdm_probe,
	.remove		= phsmdm_remove,
	.driver		= {
		.name	= "phs_mdm",
		.owner	= THIS_MODULE,
		.pm		= &phsmdm_pm_ops,
	},
};

#ifdef	USE_DEBUG_INFO

#define	BUF_ALIGNED32(x)	((x + 32 - 1) & ~(32 - 1))

ssize_t mdm_dbg_write( struct file *filp, const char __user *buff,
                        unsigned long len, void *data )
{
	static int len_buf = 0;
	static char *buf = NULL;

	if (len == 0) {
		return -EFAULT;
	}
	if (len_buf < len) {
		if (buf) {
			kfree(buf);
			buf = NULL;
		}
		len_buf = BUF_ALIGNED32(len);
		buf = (char *)kzalloc(len_buf, GFP_KERNEL);
		if (!buf) {
			len_buf = 0;
			return -ENOMEM;
		}
	}
	if (copy_from_user(buf, buff, len)) {
		return -EFAULT;
	}
	buf[len - 1] = 0x0;

	if (strcasecmp(buf, "DEBUG_LOG=1") == 0) {
		g_DEBUG_LOG = 1;
	} else if (strcasecmp(buf, "DEBUG_LOG=0") == 0) {
		g_DEBUG_LOG = 0;
#ifdef	PHS_MDM_STATS_DEBUG
	} else if (strcasecmp(buf, "STATS") == 0) {
		phs_mdm_dsp_stat("STATS");
#endif	/* PHS_MDM_STATS_DEBUG */
	} else if (strcasecmp(buf, "MDLX") == 0) {
        gpio_tlmm_config(GPIO_CFG(18, 1, GPIO_CFG_OUTPUT, GPIO_CFG_NO_PULL, GPIO_CFG_6MA), GPIO_CFG_ENABLE);
        gpio_tlmm_config(GPIO_CFG(19, 1, GPIO_CFG_INPUT, GPIO_CFG_NO_PULL, GPIO_CFG_2MA), GPIO_CFG_ENABLE);
        g_mdmlogger = 1;
	} else if (strcasecmp(buf, "MDLN") == 0) {
        gpio_tlmm_config(GPIO_CFG(18, 0, GPIO_CFG_INPUT, GPIO_CFG_PULL_DOWN, GPIO_CFG_2MA), GPIO_CFG_DISABLE);
        gpio_tlmm_config(GPIO_CFG(19, 0, GPIO_CFG_INPUT, GPIO_CFG_PULL_DOWN, GPIO_CFG_2MA), GPIO_CFG_DISABLE);
        g_mdmlogger = 0;
	} else {
		printk("phs_mdm: %s(%d) ???\n", buf, (int)len);
	}
	return len;
}

int mdm_dbg_read( char *page, char **start, off_t off,
                   int count, int *eof, void *data )
{
	int len;

	len = snprintf(page, count, "PHS modem Driver\nDEBUG_LOG=%d\n%s\n", g_DEBUG_LOG,
					g_mdmlogger ? "MDLX":"MDLN");

	*eof = 1;
	return len;
}
#endif	/* USE_DEBUG_INFO */

static int __init phsmdm_module_init(void)
{
#ifdef	USE_DEBUG_INFO
	proc_entry = create_proc_entry("driver/phs_mdm", 0644, NULL);
	if (proc_entry == NULL) {
		printk("phs_mdm: Couldn't create proc entry\n");
		return -ENOMEM;
	}

	proc_entry->read_proc = mdm_dbg_read;
	proc_entry->write_proc = mdm_dbg_write;
#endif	/* USE_DEBUG_INFO */
	return platform_driver_register(&phsmdm_driver);
}

static void __exit phsmdm_module_exit(void)
{
	platform_driver_unregister(&phsmdm_driver);
#ifdef	USE_DEBUG_INFO
	remove_proc_entry("driver/phs_mdm", NULL);
#endif	/* USE_DEBUG_INFO */
}

module_init(phsmdm_module_init);
module_exit(phsmdm_module_exit);
