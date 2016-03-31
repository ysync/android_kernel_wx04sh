/* kernel/drivers/sharp/shswic.c  (SwitchingIC Driver)
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

/* ____________________________________________________________________________________________________________________________ */
/**
 *	Include
 */

#include <linux/kernel.h>
#include <linux/miscdevice.h>
#include <linux/input.h>
#include <linux/err.h>
#include <asm/atomic.h>
#include <asm/uaccess.h>
#include <linux/platform_device.h>
#include <linux/workqueue.h>
#include <linux/jiffies.h>
#include <linux/delay.h>
#include <linux/interrupt.h>
#include <linux/i2c.h>
#include <linux/miscdevice.h>
#include <linux/regulator/consumer.h>
#include <linux/slab.h>
#include <linux/wakelock.h>
#include <linux/gpio.h>

#include <linux/module.h>

#include <sharp/shswic_kerl.h>
#ifdef CONFIG_SII8334_MHL_TX
#include <sharp/shmhl_kerl.h>
#endif /* CONFIG_SII8334_MHL_TX */
#ifdef CONFIG_HEADSET_BUTTON_SWIC
#include <sound/jack.h>
#endif /* CONFIG_HEADSET_BUTTON_SWIC */

#include "shswic_kerl_local.h"
#include "shswic_kerl_91401.h"
#include "shswic_kerl_91411.h"

/* ____________________________________________________________________________________________________________________________ */
/**
 *	Internal Define
 */

/* Driver Name */
#define SHSWIC_DRIVER_NAME 				"shswic"
#define SHSWIC_DIAG_DRIVER_NAME 		"shswic_diag"
#define SHSWIC_I2C_DRIVER_NAME 			"shswic_i2c"
#define SHSWIC_REG_DRIVER_NAME			"msm_shswic"

#ifdef CONFIG_HEADSET_BUTTON_SWIC
#define SH_SWIC_KEY_DEVICE_NAME	"shswic_key"
#endif /* CONFIG_HEADSET_BUTTON_SWIC */

#define SHSWIC_SIGNAL_PKT_COUNTS		10
#define SHSWIC_TIMER_SIGNAL_PKT_COUNTS	10

enum
{
	SHSWIC_CB_INIT_NONE = 0,
	SHSWIC_CB_INIT_INITIALIZING,
	SHSWIC_CB_INIT_COMPLETE,
};

#ifdef CONFIG_HEADSET_BUTTON_SWIC
enum
{
	SHSWIC_SEND_HEADSET_SW_ON,
	SHSWIC_SEND_HEADSET_SW_OFF,
};
#endif /* CONFIG_HEADSET_BUTTON_SWIC */

typedef struct shswic_cmd_tag
{
	struct work_struct	work;
	int					sig;
} shswic_cmd_t;

typedef struct shswic_delay_cmd_tag
{
	struct delayed_work	dwork;
	int					sig;
} shswic_delay_cmd_t;

/* ____________________________________________________________________________________________________________________________ */
/**
 *	Internal variable
 */

typedef void (*shswic_cb_detect_isr_sig_handler_t)(void);
static shswic_cb_detect_isr_sig_handler_t	shswic_cb_detect_isr_sig_handler = NULL;
typedef void (*shswic_cb_detect_init_t)(void);
static shswic_cb_detect_init_t	shswic_cb_detect_init = NULL;
typedef void (*shswic_cb_detect_retry_sig_handler_t)(void);
static shswic_cb_detect_retry_sig_handler_t	shswic_cb_detect_retry_sig_handler = NULL;
#ifdef CONFIG_SII8334_MHL_TX
typedef void (*shswic_cb_detect_mhl_proc_t)(void);
static shswic_cb_detect_mhl_proc_t	shswic_cb_detect_mhl_proc = NULL;
#endif /* CONFIG_SII8334_MHL_TX */
typedef shswic_result_t (*shswic_cb_write_vbsw_reg_t)(uint8_t vbsw_ctl);
static shswic_cb_write_vbsw_reg_t	shswic_cb_write_vbsw_reg = NULL;
typedef shswic_result_t (*shswic_cb_read_vbsw_reg_t)(uint8_t* vbsw_ctl);
static shswic_cb_read_vbsw_reg_t	shswic_cb_read_vbsw_reg = NULL;
typedef void (*shswic_cb_pre_detect_cb_call_t)(uint8_t detect);
static shswic_cb_pre_detect_cb_call_t	shswic_cb_pre_detect_cb_call = NULL;
typedef void (*shswic_cb_pre_do_exit_t)(void);
static shswic_cb_pre_do_exit_t	shswic_cb_pre_do_exit = NULL;
#ifdef CONFIG_CRADLE_SWIC
typedef int (*shswic_cb_cradle_state_t)(void);
static shswic_cb_cradle_state_t		shswic_cb_cradle_state = NULL;
#endif /* CONFIG_CRADLE_SWIC */
typedef bool (*shswic_cb_is_push_headset_button_t)(uint8_t int_status);
static shswic_cb_is_push_headset_button_t shswic_cb_is_push_headset_button = NULL;

typedef void (*shswic_cb_func_t)(uint8_t device, void* user_data);
static shswic_cb_func_t			shswic_cb_func[SHSWIC_DEVICE_NUM];
static uint32_t					shswic_cb_irq[SHSWIC_DEVICE_NUM];
static void*					shswic_cb_userdata[SHSWIC_DEVICE_NUM];
static uint8_t					shswic_cb_last_usb_port[SHSWIC_DEVICE_NUM];
#ifdef CONFIG_CRADLE_SWIC
static uint8_t					shswic_cb_last_cradle_port[SHSWIC_DEVICE_NUM];
#endif /* CONFIG_CRADLE_SWIC */
static uint32_t					shswic_last_detect = SHSWIC_ID_NONE;
static struct i2c_client 		*shswic_i2c_client;
struct workqueue_struct			*shswic_wq = NULL;
static struct mutex				shswic_task_lock;
static struct mutex				shswic_cb_lock;
static atomic_t					shswic_cb_init_state = ATOMIC_INIT(SHSWIC_CB_INIT_NONE);
static atomic_t					shswic_delay_cancel_sig = ATOMIC_INIT(0);
static uint8_t					shswic_init_flag = SHSWIC_INIT;
uint8_t							shswic_detect_state = SHSWIC_STATE_NONE;
uint8_t							shswic_detect_id = 0x0d;
uint8_t							shswic_detect_port = SHSWIC_ID_NONE;
#ifdef CONFIG_CRADLE_SWIC
static uint8_t					shswic_cradle_port = SHSWIC_ID_CRADLE_UNKNOWN;
#endif /* CONFIG_CRADLE_SWIC */
#ifdef CONFIG_SII8334_MHL_TX
shswic_read_status_t			shswic_read_data;
static struct device			*g_swic_regulator_dev = NULL;
#endif /* CONFIG_SII8334_MHL_TX */
#ifdef CONFIG_HEADSET_BUTTON_SWIC
static struct input_dev*		shswic_input_dev = NULL;
static int						shswic_last_send_sw = SHSWIC_SEND_HEADSET_SW_OFF;
#endif /* CONFIG_HEADSET_BUTTON_SWIC */
static uint16_t					shswic_signal_pkt_use_bitflg = 0;
static shswic_cmd_t*			shswic_signal_pkt = NULL;
static uint16_t					shswic_timer_signal_pkt_use_bitflg = 0;
static shswic_delay_cmd_t*		shswic_timer_signal_pkt = NULL;
#ifdef CONFIG_HEADSET_BUTTON_SWIC
bool						shswic_switch_earphone = false;
#endif /* CONFIG_HEADSET_BUTTON_SWIC */
static struct wake_lock			shswic_delayed_work_wake_lock;

static struct kobject			*shswic_kobj;
char							shswic_debug_flg[SHSWIC_DEBUG_LOG_MAX+1];


/* ____________________________________________________________________________________________________________________________ */
/**
 *	Internal prototypes
 */

static void shswic_signal_handler(struct work_struct *poWork);
static void shswic_signal_delay_handler(struct work_struct *poWork);
static void shswic_signal_handler_core(long sig);
static void shswic_send_signal(long sig);
static void shswic_detect_init(void);
static void shswic_detect_cb_init(void);
static void shswic_status_init(void);
static void shswic_detect_retry_sig_handler(void);

static void shswic_detect_cb_call(uint8_t detect);
static void shswic_detect_cb_call_func(uint8_t detect);

static int8_t shswic_state_usb_cable_proc(uint8_t detect);
#ifdef CONFIG_HOST_SWIC
static int8_t shswic_state_usb_host_cable_proc(uint8_t detect);
#endif /* CONFIG_HOST_SWIC */
static int8_t shswic_state_ac_adapter_proc(uint8_t detect);
static int8_t shswic_state_irregular_charger_proc(uint8_t detect);
static int8_t shswic_state_headset_proc(uint8_t detect);
static int8_t shswic_state_headset_sw_proc(uint8_t detect);
static int8_t shswic_state_none_proc(uint8_t detect);
#ifdef CONFIG_SII8334_MHL_TX
static int8_t shswic_state_mhl_proc(uint8_t detect);
#endif /* CONFIG_SII8334_MHL_TX */
int8_t (* shswic_state_proc[])(uint8_t detect) =
{
	shswic_state_usb_cable_proc,
#ifdef CONFIG_HOST_SWIC
	shswic_state_usb_host_cable_proc,
#endif /* CONFIG_HOST_SWIC */
	shswic_state_ac_adapter_proc,
	shswic_state_irregular_charger_proc,
	shswic_state_headset_proc,
	shswic_state_headset_sw_proc,
	shswic_state_none_proc,
#ifdef CONFIG_SII8334_MHL_TX
	shswic_state_mhl_proc,
#endif /* CONFIG_SII8334_MHL_TX */
};

static void shswic_set_shswic_cb_init_state(int state);
static int shswic_get_shswic_cb_init_state(void);
static void shswic_set_delay_cancel_sig(int sig);
static void shswic_clear_delay_cancel_sig(int sig);
static int shswic_get_delay_cancel_sig(void);

static shswic_result_t shswic_diag_get_id_data(uint8_t* id_data);
static int shswic_diag_open(struct inode *inode, struct file *file);
static int shswic_diag_release(struct inode *inode, struct file *file);
static ssize_t shswic_diag_read(struct file *file, char __user *buf, size_t count, loff_t *ppos);
static long shswic_diag_ioctl(struct file *filp, unsigned int cmd, unsigned long arg);

static int shswic_i2c_probe(struct i2c_client *client, const struct i2c_device_id *id);
static int shswic_i2c_remove(struct i2c_client *client);

static int shswic_pm_driver_probe(struct platform_device *pdev);

#ifdef CONFIG_SII8334_MHL_TX
static void shswic_detect_cb_mhl_proc(void);
#endif /* CONFIG_SII8334_MHL_TX */

#ifdef CONFIG_HEADSET_BUTTON_SWIC
static void shswic_register_input_dev(void);
static void shswic_release_input_dev(void);
#endif /* CONFIG_HEADSET_BUTTON_SWIC */

static void shswic_do_exit(void);

static int __init shswic_init(void);
static void __exit shswic_exit(void);

static int shswic_pm_suspend(struct device *dev);
static int shswic_pm_resume(struct device *dev);
static int __devexit shswic_pm_remove( struct platform_device* dev_p );

static struct file_operations shswic_diag_fops = {
	.owner			= THIS_MODULE,
	.open			= shswic_diag_open,
	.release		= shswic_diag_release,
	.read			= shswic_diag_read,
	.unlocked_ioctl	= shswic_diag_ioctl,

};
static struct miscdevice shswic_diag_device = {
	.minor			= MISC_DYNAMIC_MINOR,
	.name			= SHSWIC_DIAG_DRIVER_NAME,
	.fops			= &shswic_diag_fops,
};

static const struct i2c_device_id shswic_i2c_id[] = {
	{ SHSWIC_I2C_DRIVER_NAME, 0 },
	{ }
};
static struct i2c_driver shswic_i2c_driver = {
	.driver = {
		.owner		= THIS_MODULE,
		.name		= SHSWIC_I2C_DRIVER_NAME,
	},
	.probe			= shswic_i2c_probe,
	.id_table		= shswic_i2c_id,
	.remove			= shswic_i2c_remove,
};

static const struct dev_pm_ops shswic_pm_ops = {
    .suspend =  shswic_pm_suspend,
    .resume  =  shswic_pm_resume,
}; 

static struct platform_driver shswic_pm_driver = {
	.driver = {
		.name		= SHSWIC_REG_DRIVER_NAME,
		.owner		= THIS_MODULE,
        .pm    = &shswic_pm_ops,
	},
	.probe			= shswic_pm_driver_probe,
    .remove			= __devexit_p(shswic_pm_remove),
};


/* ____________________________________________________________________________________________________________________________ */
/**
 *	External Functions
 */
shswic_result_t shswic_detect_cb_regist(uint8_t cb_dev, uint32_t cb_irq, void* cb_func, void* user_data)
{
	shswic_result_t ret = SHSWIC_SUCCESS;

	SHSWIC_DEBUG_LOG_L("shswic_detect_cb_regist Start[cb_irq:0x%x][cb_dev:0x%x]", cb_irq, cb_dev);

	shswic_detect_cb_init();

	mutex_lock(&shswic_cb_lock);

	switch (cb_dev)
	{
		case SHSWIC_VBUS_DEVICE:
		case SHSWIC_CHG_DEVICE:
		case SHSWIC_OFFCHG_DEVICE:
		case SHSWIC_CODEC_DEVICE:
			shswic_cb_func[cb_dev]     = (shswic_cb_func_t)cb_func;
			shswic_cb_irq[cb_dev]      = cb_irq;
			shswic_cb_userdata[cb_dev] = user_data;
			if((!shswic_cb_func[cb_dev]) || (!shswic_cb_irq[cb_dev]))
			{
				SHSWIC_DEBUG_LOG_L("shswic_detect_cb_regist callback null");
			}else{
				if(SHSWIC_IMPLEMENT == shswic_init_flag)
				{
					SHSWIC_DEBUG_LOG_L("SHSWIC_CALLBACK_SIG");
					shswic_send_timer_signal(SHSWIC_CALLBACK_SIG, SHSWIC_CALLBACK_RETRY_TIME);
				}
			}
			SHSWIC_DEBUG_LOG_L("shswic_detect_cb_regist Success[cb_dev:0x%x]", cb_dev);
			break;

		default:
			SHSWIC_DEBUG_LOG_L("shswic_detect_cb_regist Param Error[cb_dev:0x%x]", cb_dev);
			ret =  SHSWIC_PARAM_ERROR;
			break;
	}

	mutex_unlock(&shswic_cb_lock);
	return ret;
}

shswic_result_t shswic_get_usb_port_status(uint8_t* device)
{
	SHSWIC_DEBUG_LOG_L("shswic_get_usb_port_status Start[State:0x%x]", shswic_detect_port);

	if (device == NULL)
	{
		SHSWIC_DEBUG_LOG_H("shswic_get_usb_port_status SHSWIC_PARAM_ERROR");
		return SHSWIC_PARAM_ERROR;
	}
	*device = shswic_detect_port;
	return SHSWIC_SUCCESS;
}

shswic_result_t shswic_get_cradle_status(uint8_t* device)
{
#ifdef CONFIG_CRADLE_SWIC
	SHSWIC_DEBUG_LOG_L("shswic_get_cradle_status Start[State:0x%x]", shswic_cradle_port);
#endif /* CONFIG_CRADLE_SWIC */

	if (device == NULL)
	{
		SHSWIC_DEBUG_LOG_H("shswic_get_cradle_status SHSWIC_PARAM_ERROR");
		return SHSWIC_PARAM_ERROR;
	}

#ifdef CONFIG_CRADLE_SWIC
	if (shswic_cradle_port == SHSWIC_ID_CRADLE)
	{
		*device = SHSWIC_ID_CRADLE;
	}
	else if(shswic_cradle_port == SHSWIC_ID_CRADLE_UNKNOWN)
	{
		*device = SHSWIC_ID_CRADLE_UNKNOWN;
	}
	else
	{
		*device = SHSWIC_ID_NONE;
	}
	return SHSWIC_SUCCESS;
#else /* CONFIG_CRADLE_SWIC */
	*device = SHSWIC_ID_NONE;
	return SHSWIC_SUCCESS;
#endif /* CONFIG_CRADLE_SWIC */
}

shswic_result_t shswic_write_vbsw_reg(uint8_t vbsw_ctl)
{
	shswic_result_t result = SHSWIC_FAILURE;

	if (vbsw_ctl > SHSWIC_VBSW_OFF)
	{
		SHSWIC_DEBUG_LOG_H("shswic_write_vbsw_reg SHSWIC_PARAM_ERROR");
		return SHSWIC_PARAM_ERROR;
	}

	if ((shswic_init_flag == SHSWIC_IMPLEMENT)
	&& (shswic_cb_write_vbsw_reg) )
	{
		SHSWIC_DEBUG_LOG_L("shswic_write_vbsw_reg vbsw_ctl = %d", vbsw_ctl);
		result = shswic_cb_write_vbsw_reg(vbsw_ctl);
	}
	return result;
}

shswic_result_t shswic_read_vbsw_reg(uint8_t* vbsw_ctl)
{
	shswic_result_t result = SHSWIC_FAILURE;

	if ((shswic_init_flag == SHSWIC_IMPLEMENT)
	&& (shswic_cb_read_vbsw_reg) )
	{
		SHSWIC_DEBUG_LOG_L("shswic_read_vbsw_reg vbsw_ctl = %d", *vbsw_ctl);
		result = shswic_cb_read_vbsw_reg(vbsw_ctl);
	}
	return result;
}

/* ____________________________________________________________________________________________________________________________ */
/**
 *	Main Functions
 */
 
static void shswic_signal_handler(struct work_struct *poWork)
{
	shswic_cmd_t* pkt_p;
	int idx = 0;

	mutex_lock(&shswic_task_lock);

	pkt_p = (shswic_cmd_t*)poWork;
	shswic_signal_handler_core(pkt_p->sig);
	idx = ((int)pkt_p - (int)shswic_signal_pkt) / sizeof(shswic_cmd_t);
	shswic_signal_pkt_use_bitflg &= ~(0x01 << idx);
	SHSWIC_DEBUG_LOG_L("shswic_signal_handler idx(%d) bit(0x%x)", idx, shswic_signal_pkt_use_bitflg);

	mutex_unlock(&shswic_task_lock);
	return;
}

static void shswic_signal_delay_handler(struct work_struct *poWork)
{
	shswic_delay_cmd_t* pkt_p;
	int idx = 0;

	mutex_lock(&shswic_task_lock);

	pkt_p = (shswic_delay_cmd_t*)poWork;

	if ((shswic_get_delay_cancel_sig() & pkt_p->sig) == 0)
	{
		shswic_signal_handler_core(pkt_p->sig);
	}
	shswic_clear_delay_cancel_sig(pkt_p->sig);

	idx = ((int)pkt_p - (int)shswic_timer_signal_pkt) / sizeof(shswic_delay_cmd_t);
	shswic_timer_signal_pkt_use_bitflg &= ~(0x01 << idx);
	SHSWIC_DEBUG_LOG_L("shswic_signal_delay_handler idx(%d) bit(0x%x)", idx, shswic_timer_signal_pkt_use_bitflg);

	if(0x00 == shswic_timer_signal_pkt_use_bitflg)
	{
		/* all delayed event completed */
		if(wake_lock_active(&shswic_delayed_work_wake_lock))
		{
			SHSWIC_DEBUG_LOG_L("delayed_wake_unlock");
			wake_unlock(&shswic_delayed_work_wake_lock);
		}
	}

	mutex_unlock(&shswic_task_lock);
	return;
}

static void shswic_signal_handler_core(long sig)
{
	switch (sig)
	{
		case SHSWIC_DETECT_ISR_SIG:
			SHSWIC_DEBUG_LOG_L("Call shswic_detect_isr_sig_handler");
			if(shswic_cb_detect_isr_sig_handler)
			{
				shswic_cb_detect_isr_sig_handler();
			}
			break;

		case SHSWIC_DETECT_INIT_SIG:
			SHSWIC_DEBUG_LOG_L("Call shswic_detect_init");
			shswic_detect_init();
			break;

		case SHSWIC_INIT_SIG:
			SHSWIC_DEBUG_LOG_L("Call shswic_status_init");
			shswic_status_init();
			break;

		case SHSWIC_DETECT_RETRY_SIG:
			SHSWIC_DEBUG_LOG_L("Call shswic_detect_retry_sig_handler");
			shswic_detect_retry_sig_handler();
			break;

#ifdef CONFIG_CRADLE_SWIC
		case SHSWIC_CRADLE_SIG:
			SHSWIC_DEBUG_LOG_L("Call shswic_state_cradle_proc");
			shswic_state_cradle_proc();
			break;
#endif /* CONFIG_CRADLE_SWIC */

#ifdef CONFIG_SII8334_MHL_TX
		case SHSWIC_DETECT_CB_MHL_SIG:
			SHSWIC_DEBUG_LOG_L("Call shswic_detect_cb_mhl_proc");
			shswic_detect_cb_mhl_proc();
			break;
#endif /* CONFIG_SII8334_MHL_TX */

		case SHSWIC_CALLBACK_SIG:
			SHSWIC_DEBUG_LOG_L("Call shswic_detect_cb_call");
			shswic_detect_cb_call(shswic_detect_port);
#ifdef CONFIG_CRADLE_SWIC
			shswic_detect_cb_call(shswic_cradle_port);
#endif /* CONFIG_CRADLE_SWIC */
			break;

		default:
			break;
	}
	return;
}

static void shswic_send_signal(long sig)
{
	shswic_cmd_t* pkt_p;
	int idx = 0;
	
	SHSWIC_DEBUG_LOG_L("Set Sig 0x%lx", sig);

	for(idx = 0; idx < SHSWIC_SIGNAL_PKT_COUNTS; ++idx)
	{
		if(!(shswic_signal_pkt_use_bitflg & (0x01 << idx)))
		{
		break;
		}
	}

	if(idx >= SHSWIC_SIGNAL_PKT_COUNTS)
	{
		SHSWIC_DEBUG_LOG_L("shswic_send_signal sig(0x%lx)", sig);
		return;
	}
	pkt_p = &(shswic_signal_pkt[idx]);
	shswic_signal_pkt_use_bitflg |= (0x01 << idx);
	SHSWIC_DEBUG_LOG_L("shswic_send_signal idx(%d) bit(0x%x)", idx, shswic_signal_pkt_use_bitflg);

	pkt_p->sig = sig;
	INIT_WORK((struct work_struct*)pkt_p, shswic_signal_handler);
	queue_work(shswic_wq, (struct work_struct*)pkt_p);
	return;
}

void shswic_send_timer_signal(long sig, int msec)
{
	shswic_delay_cmd_t* pkt_p;
	int idx = 0;

	SHSWIC_DEBUG_LOG_L("Set Delay Sig 0x%lx(%dms)", sig, msec);

	for(idx = 0; idx < SHSWIC_TIMER_SIGNAL_PKT_COUNTS; ++idx)
	{
		if(!(shswic_timer_signal_pkt_use_bitflg & (0x01 << idx)))
		{
		break;
		}
	}

	if(idx >= SHSWIC_TIMER_SIGNAL_PKT_COUNTS)
	{
		SHSWIC_DEBUG_LOG_L("shswic_send_timer_signal sig(0x%lx)", sig);
		return;
	}
	pkt_p = &(shswic_timer_signal_pkt[idx]);
	shswic_timer_signal_pkt_use_bitflg |= (0x01 << idx);
	SHSWIC_DEBUG_LOG_L("shswic_send_timer_signal idx(%d) bit(0x%x)", idx, shswic_timer_signal_pkt_use_bitflg);

	if(wake_lock_active(&shswic_delayed_work_wake_lock) == false)
	{
		SHSWIC_DEBUG_LOG_L("delayed_wake_lock");
		wake_lock(&shswic_delayed_work_wake_lock);
	}

	pkt_p->sig = sig;
	INIT_DELAYED_WORK((struct delayed_work*)pkt_p, shswic_signal_delay_handler);
	queue_delayed_work(shswic_wq, (struct delayed_work*)pkt_p, msecs_to_jiffies(msec));
	return;
}

void shswic_clear_timer_signal(long sig)
{
	SHSWIC_DEBUG_LOG_L("Cancel Delay Sig 0x%lx Start", sig);

	shswic_set_delay_cancel_sig(sig);

	return;
}

static void shswic_detect_init(void)
{
	shswic_detect_cb_init();
	
	if(shswic_cb_detect_init)
	{
		shswic_cb_detect_init();
	}
}

static void shswic_detect_cb_init(void)
{
	int i;

	if (shswic_get_shswic_cb_init_state() == SHSWIC_CB_INIT_INITIALIZING)
	{
		for (i = 0; i < 10; i++)
		{
			/* Don't call wake_lock. */
			usleep(10000);
			if (shswic_get_shswic_cb_init_state() == SHSWIC_CB_INIT_COMPLETE)
			{
				break;
			}
		}
	}

	if (shswic_get_shswic_cb_init_state() == SHSWIC_CB_INIT_NONE)
	{
		SHSWIC_DEBUG_LOG_L("shswic_detect_cb_init Start");

		shswic_set_shswic_cb_init_state(SHSWIC_CB_INIT_INITIALIZING);

		mutex_init(&shswic_cb_lock);

		for (i = 0; i < SHSWIC_DEVICE_NUM; i++)
		{
			shswic_cb_func[i]     = NULL;
			shswic_cb_irq[i]      = 0;
			shswic_cb_userdata[i] = NULL;
			shswic_cb_last_usb_port[i] = SHSWIC_ID_NONE;
#ifdef CONFIG_CRADLE_SWIC
			shswic_cb_last_cradle_port[i] = SHSWIC_ID_NONE;
#endif /* CONFIG_CRADLE_SWIC */
		}

		shswic_set_shswic_cb_init_state(SHSWIC_CB_INIT_COMPLETE);
	}
	return;
}

static void shswic_status_init(void)
{
	static uint8_t shswic_init_retry = 0;
	uint8_t id_status = 0;
	uint8_t status = 0;
	uint8_t int_status = 0;
	int8_t ret;

	SHSWIC_DEBUG_LOG_L("shswic_status_init Start");

	ret = shswic_status_read(&id_status, &status, &int_status);
	if (ret == SHSWIC_SUCCESS)
	{
		uint8_t vbsw_ctl = 0xF0;
		shswic_i2c_write(&vbsw_ctl, 1, SHSWIC_REG_VBSW_CONTROL);
		vbsw_ctl = 0xFF;
		shswic_i2c_read(&vbsw_ctl, 1, SHSWIC_REG_VBSW_CONTROL);

		switch(vbsw_ctl)
		{
			case 0xF0:
				shswic_cb_detect_isr_sig_handler = shswic_detect_isr_sig_handler_91401;
				shswic_cb_detect_init = shswic_detect_init_91401;
				shswic_cb_detect_retry_sig_handler = shswic_detect_retry_sig_handler_91401;
#ifdef CONFIG_SII8334_MHL_TX
				shswic_cb_detect_mhl_proc = shswic_detect_mhl_proc_91401;
#endif /* CONFIG_SII8334_MHL_TX */
				shswic_cb_write_vbsw_reg = shswic_write_vbsw_reg_91401;
				shswic_cb_read_vbsw_reg = shswic_read_vbsw_reg_91401;
				shswic_cb_pre_detect_cb_call = shswic_pre_detect_cb_call_91401;
				shswic_cb_pre_do_exit = shswic_pre_do_exit_91401;
#ifdef CONFIG_CRADLE_SWIC
				shswic_cb_cradle_state = shswic_cradle_state_91401;
#endif /* CONFIG_CRADLE_SWIC */
				shswic_cb_is_push_headset_button = shswic_is_push_headset_button_91401;
				shswic_init_flag = SHSWIC_IMPLEMENT;
				SHSWIC_DEBUG_LOG_L("SHSWIC_IMPLEMENT 91401");
				break;
				
			case 0x00:
				shswic_cb_detect_isr_sig_handler = shswic_detect_isr_sig_handler_91411;
				shswic_cb_detect_init = shswic_detect_init_91411;
				shswic_cb_detect_retry_sig_handler = NULL;
#ifdef CONFIG_SII8334_MHL_TX
				shswic_cb_detect_mhl_proc = shswic_detect_mhl_proc_91411;
#endif /* CONFIG_SII8334_MHL_TX */
				shswic_cb_write_vbsw_reg = shswic_write_vbsw_reg_91411;
				shswic_cb_read_vbsw_reg = shswic_read_vbsw_reg_91411;
				shswic_cb_pre_detect_cb_call = shswic_pre_detect_cb_call_91411;
				shswic_cb_pre_do_exit = shswic_pre_do_exit_91411;
#ifdef CONFIG_CRADLE_SWIC
				shswic_cb_cradle_state = shswic_cradle_state_91411;
#endif /* CONFIG_CRADLE_SWIC */
				shswic_cb_is_push_headset_button = shswic_is_push_headset_button_91411;
				shswic_init_flag = SHSWIC_IMPLEMENT;
				SHSWIC_DEBUG_LOG_L("SHSWIC_IMPLEMENT 91411");
				break;
			
			default:
				shswic_init_flag = SHSWIC_NOT_IMPLEMENT;
				SHSWIC_DEBUG_LOG_L("OVP: 0x%x", vbsw_ctl);
				SHSWIC_DEBUG_LOG_L("SHSWIC_NOT_IMPLEMENT");
				shswic_do_exit();
				break;
		}
		

		if(SHSWIC_IMPLEMENT == shswic_init_flag)
		{
			shswic_send_signal(SHSWIC_DETECT_INIT_SIG);
			SHSWIC_DEBUG_LOG_L("shswic_status_init Success");
		}
	}
	else
	{
		if (shswic_init_retry < SHSWIC_INIT_RETRY_CNT)
		{
			shswic_init_retry++;
			shswic_send_timer_signal(SHSWIC_INIT_SIG, SHSWIC_INIT_RETRY_TIME);
		}
		else
		{
			shswic_init_flag = SHSWIC_NOT_IMPLEMENT;
			SHSWIC_DEBUG_LOG_L("SHSWIC_NOT_IMPLEMENT");
			shswic_do_exit();
		}
	}
	return;
}

irqreturn_t shswic_detect_isr(int irq, void *dev_id)
{
	SHSWIC_DEBUG_LOG_L("shswic_detect_isr Start");

	shswic_send_signal(SHSWIC_DETECT_ISR_SIG);
	return IRQ_HANDLED;
}

#ifdef CONFIG_CRADLE_SWIC
irqreturn_t shswic_cradle_isr(int irq, void *dev_id)
{
	SHSWIC_DEBUG_LOG_L("shswic_cradle_isr Start");

	shswic_send_signal(SHSWIC_CRADLE_SIG);
	return IRQ_HANDLED;
}
#endif /* CONFIG_CRADLE_SWIC */

static void shswic_detect_retry_sig_handler(void)
{
	if(shswic_cb_detect_retry_sig_handler)
	{
		shswic_cb_detect_retry_sig_handler();
	}
}

#ifdef CONFIG_SII8334_MHL_TX
void shswic_detect_cb_mhl(shmhl_detect_device_t device, void* user_data)
{
	SHSWIC_DEBUG_LOG_L("shswic_shmhl_detect_cb Called [device=%d]", device);

	shswic_read_data.shswic_mhl_result = device;
	shswic_send_signal(SHSWIC_DETECT_CB_MHL_SIG);
}

static void shswic_detect_cb_mhl_proc(void)
{
	if(shswic_cb_detect_mhl_proc)
	{
		shswic_cb_detect_mhl_proc();
	}
}
#endif /* CONFIG_SII8334_MHL_TX */

int8_t shswic_status_read(uint8_t* id_status, uint8_t* status, uint8_t* int_status)
{
	int8_t ret;
	uint8_t temp_id;
	uint8_t temp_status;
	uint8_t temp_int;

	SHSWIC_DEBUG_LOG_L("shswic_status_read Start");

	*id_status = *status = *int_status = 0x00;
	temp_id = temp_status = temp_int = 0x00;

	ret = shswic_i2c_read(&temp_id, 1, SHSWIC_REG_ID_STA);
	if (ret == SHSWIC_SUCCESS)
	{
		ret = shswic_i2c_read(&temp_status, 1, SHSWIC_REG_STATUS);
		if (ret == SHSWIC_SUCCESS)
		{
			ret = shswic_i2c_read(&temp_int, 1, SHSWIC_REG_INT_STA);
			if (ret == SHSWIC_SUCCESS)
			{
				*id_status = temp_id;
				*status = temp_status;
				*int_status = temp_int;
			}
		}
	}

	SHSWIC_DEBUG_LOG_L("shswic_status_read ret %d, id = 0x%02x, status = 0x%02x, int = 0x%02x", ret, temp_id, temp_status, temp_int);

	if ((ret != SHSWIC_SUCCESS)
	&&  (shswic_init_flag == SHSWIC_IMPLEMENT) )
	{
		SHSWIC_DEBUG_LOG_H("shswic_status_read error %d", ret);
	}
	return ret;
}

static void shswic_detect_cb_call(uint8_t detect)
{
	SHSWIC_DEBUG_LOG_L("shswic_detect_cb_call detect = 0x%02x, shswic_detect_port = 0x%02x", detect, shswic_detect_port);

	if(shswic_cb_pre_detect_cb_call)
	{
		shswic_cb_pre_detect_cb_call(detect);
	}

#ifdef CONFIG_HEADSET_BUTTON_SWIC
	if((SHSWIC_ID_HEADSET != detect)
	&& (SHSWIC_ID_HEADSET_SW != detect)
	&& (SHSWIC_ID_CRADLE != detect)
	&& (SHSWIC_ID_CRADLE_NONE != detect))
	{
		if(SHSWIC_SEND_HEADSET_SW_ON == shswic_last_send_sw)
		{
		input_report_key(shswic_input_dev, KEY_MEDIA, 0);
		input_sync(shswic_input_dev);
		shswic_last_send_sw = SHSWIC_SEND_HEADSET_SW_OFF;
		}
		
		shswic_release_input_dev();
	}
#endif /* CONFIG_HEADSET_BUTTON_SWIC */

	switch (detect)
	{
		case SHSWIC_ID_USB_CABLE:
		case SHSWIC_ID_AC_ADAPTER:
		case SHSWIC_ID_IRREGULAR_CHARGER:
#ifdef CONFIG_HOST_SWIC
		case SHSWIC_ID_USB_HOST_CABLE:
#endif /* CONFIG_HOST_SWIC */
#ifdef CONFIG_SII8334_MHL_TX
		case SHSWIC_ID_MHL:
#endif /* CONFIG_SII8334_MHL_TX */
			shswic_detect_port = detect;
			shswic_detect_cb_call_func(detect);
			break;

		case SHSWIC_ID_HEADSET:
		case SHSWIC_ID_HEADSET_SW:
			shswic_detect_port = detect;
#ifdef CONFIG_HEADSET_BUTTON_SWIC
			shswic_detect_cb_call_func(detect);
			shswic_register_input_dev();
#else /* CONFIG_HEADSET_BUTTON_SWIC */
			shswic_last_detect = detect;
#endif /* CONFIG_HEADSET_BUTTON_SWIC */
			break;

		case SHSWIC_ID_NONE:
			switch (shswic_detect_port)
			{
				case SHSWIC_ID_USB_CABLE:
#ifdef CONFIG_HOST_SWIC
				case SHSWIC_ID_USB_HOST_CABLE:
#endif /* CONFIG_HOST_SWIC */
				case SHSWIC_ID_AC_ADAPTER:
				case SHSWIC_ID_IRREGULAR_CHARGER:
#ifdef CONFIG_SII8334_MHL_TX
				case SHSWIC_ID_MHL:
#endif /* CONFIG_SII8334_MHL_TX */
					shswic_detect_port = SHSWIC_ID_NONE;
					shswic_detect_cb_call_func(detect);
					break;

				case SHSWIC_ID_HEADSET:
				case SHSWIC_ID_HEADSET_SW:
					shswic_detect_port = SHSWIC_ID_NONE;
#ifdef CONFIG_HEADSET_BUTTON_SWIC
					shswic_detect_cb_call_func(detect);
#endif /* CONFIG_HEADSET_BUTTON_SWIC */
					break;

				default:
					break;
			}
			break;

#ifdef CONFIG_CRADLE_SWIC
		case SHSWIC_ID_CRADLE:
		case SHSWIC_ID_CRADLE_NONE:
			shswic_cradle_port = detect;
			shswic_detect_cb_call_func(detect);
			break;
#endif /* CONFIG_CRADLE_SWIC */

		default:
			break;
	}
	return;
}

static void shswic_detect_cb_call_func(uint8_t detect)
{
	int i;
#ifdef CONFIG_HEADSET_BUTTON_SWIC
	int switch_earphone_detect;
#endif /* CONFIG_HEADSET_BUTTON_SWIC */
	
	SHSWIC_DEBUG_LOG_L("shswic_detect_cb_call_func Start [detect = 0x%x]", detect);

	mutex_lock(&shswic_cb_lock);

	switch (detect)
	{
		case SHSWIC_ID_USB_CABLE:
#ifdef CONFIG_HOST_SWIC
		case SHSWIC_ID_USB_HOST_CABLE:
#endif /* CONFIG_HOST_SWIC */
		case SHSWIC_ID_AC_ADAPTER:
		case SHSWIC_ID_IRREGULAR_CHARGER:
#ifndef CONFIG_HEADSET_BUTTON_SWIC
		case SHSWIC_ID_HEADSET:
		case SHSWIC_ID_HEADSET_SW:
#endif /* CONFIG_HEADSET_BUTTON_SWIC */
#ifdef CONFIG_SII8334_MHL_TX
		case SHSWIC_ID_MHL:
#endif /* CONFIG_SII8334_MHL_TX */
			for (i = 0; i < SHSWIC_DEVICE_NUM; i++)
			{
				if (!shswic_cb_func[i])
				{
					continue;
				}

				if (!(shswic_cb_irq[i] & detect))
				{
					continue;
				}
				
				if(shswic_cb_last_usb_port[i] != detect)
				{
					SHSWIC_DEBUG_LOG_L("CallBack Call[detect = 0x%x]", detect);
					shswic_cb_func[i]((uint8_t)detect, shswic_cb_userdata[i]);
					shswic_cb_last_usb_port[i] = detect;
				}
			}
			shswic_last_detect = detect;
			break;

#ifdef CONFIG_HEADSET_BUTTON_SWIC
		case SHSWIC_ID_HEADSET:
		case SHSWIC_ID_HEADSET_SW:
			for (i = 0; i < SHSWIC_DEVICE_NUM; i++)
			{
				if (!shswic_cb_func[i])
				{
					continue;
				}

				if (!(shswic_cb_irq[i] & detect))
				{
					continue;
				}

				/* Can't judgment earphone implement swittch button,	*/
				/* if shswic_switch_earphone is false.					*/
				if((SHSWIC_CODEC_DEVICE != i)
				|| (!shswic_switch_earphone))
				{
					if(shswic_cb_last_usb_port[i] != detect)
					{
						SHSWIC_DEBUG_LOG_L("CallBack Call[detect = 0x%x]", detect);
						shswic_cb_func[i]((uint8_t)detect, shswic_cb_userdata[i]);
						shswic_cb_last_usb_port[i] = detect;
					}
				}
				else
				{
					/* SHSWIC_CODEC_DEVICE request customeize event type, 			*/
					/* it's SHSWIC_ID_HEADSET_SW_OFF or SHSWIC_ID_HEADSET_SW_ON.	*/
					switch_earphone_detect = SHSWIC_ID_HEADSET_SW_OFF;
					if(detect == SHSWIC_ID_HEADSET_SW)
					{
						switch_earphone_detect = SHSWIC_ID_HEADSET_SW_ON;
					}

					if(shswic_cb_last_usb_port[i] != switch_earphone_detect)
					{
						SHSWIC_DEBUG_LOG_L("CallBack Call[detect = 0x%x]", switch_earphone_detect);
						shswic_cb_func[i]((uint8_t)switch_earphone_detect, shswic_cb_userdata[i]);
						shswic_cb_last_usb_port[i] = switch_earphone_detect;
					}
				}
			}
			shswic_last_detect = detect;
			break;
#endif /* CONFIG_HEADSET_BUTTON_SWIC */

		case SHSWIC_ID_NONE:
			for (i = 0; i < SHSWIC_DEVICE_NUM; i++)
			{
				if (!shswic_cb_func[i])
				{
					continue;
				}
				
				if(shswic_cb_last_usb_port[i] != detect)
				{
					SHSWIC_DEBUG_LOG_L("CallBack Call[detect = 0x%x]", detect);
					shswic_cb_func[i]((uint8_t)detect, shswic_cb_userdata[i]);
					shswic_cb_last_usb_port[i] = detect;
				}
			}
			shswic_last_detect = detect;
			break;

#ifdef CONFIG_CRADLE_SWIC
		case SHSWIC_ID_CRADLE:
			for (i = 0; i < SHSWIC_DEVICE_NUM; i++)
			{
				if (!shswic_cb_func[i])
				{
					continue;
				}
				
				if (!(shswic_cb_irq[i] & detect))
				{
					continue;
				}
				
				if(shswic_cb_last_cradle_port[i] != detect)
				{
					SHSWIC_DEBUG_LOG_L("CallBack Call[cradle = 0x%x]", detect);
					shswic_cb_func[i]((uint8_t)detect, shswic_cb_userdata[i]);
					shswic_cb_last_cradle_port[i] = detect;
				}
			}
			break;

		case SHSWIC_ID_CRADLE_NONE:
			for (i = 0; i < SHSWIC_DEVICE_NUM; i++)
			{
				if (!shswic_cb_func[i])
				{
					continue;
				}
				
				if (!(shswic_cb_irq[i] & SHSWIC_ID_CRADLE))
				{
					continue;
				}
				
				if(shswic_cb_last_cradle_port[i] != SHSWIC_ID_NONE)
				{
					SHSWIC_DEBUG_LOG_L("CallBack Call[cradle = 0x%x]", detect);
					shswic_cb_func[i]((uint8_t)SHSWIC_ID_NONE, shswic_cb_userdata[i]);
					shswic_cb_last_cradle_port[i] = SHSWIC_ID_NONE;
				}
			}
#endif /* CONFIG_CRADLE_SWIC */

		default:
			break;
	}

	mutex_unlock(&shswic_cb_lock);
	return;
}

int8_t shswic_state_usb_cable_proc(uint8_t detect)
{
	SHSWIC_DEBUG_LOG_L("shswic_state_usb_cable_proc detect = 0x%02x", detect);

	if (detect != SHSWIC_ID_USB_CABLE)
	{
		shswic_detect_cb_call(SHSWIC_ID_NONE);

		if (detect != SHSWIC_ID_NONE)
		{
			shswic_detect_cb_call(detect);
		}
	}

	return SHSWIC_SUCCESS;
}

#ifdef CONFIG_HOST_SWIC
int8_t shswic_state_usb_host_cable_proc(uint8_t detect)
{
	SHSWIC_DEBUG_LOG_L("shswic_state_usb_host_cable_proc detect = 0x%02x", detect);

	if (detect != SHSWIC_ID_USB_HOST_CABLE)
	{
		shswic_detect_cb_call(SHSWIC_ID_NONE);

		if (detect != SHSWIC_ID_NONE)
		{
			shswic_detect_cb_call(detect);
		}
	}

	return SHSWIC_SUCCESS;
}
#endif /* CONFIG_HOST_SWIC */

int8_t shswic_state_ac_adapter_proc(uint8_t detect)
{
	SHSWIC_DEBUG_LOG_L("shswic_state_ac_adapter_proc detect = 0x%02x", detect);

	if (detect != SHSWIC_ID_AC_ADAPTER)
	{
		shswic_detect_cb_call(SHSWIC_ID_NONE);

		if (detect != SHSWIC_ID_NONE)
		{
			shswic_detect_cb_call(detect);
		}
	}

	return SHSWIC_SUCCESS;
}

int8_t shswic_state_irregular_charger_proc(uint8_t detect)
{
	SHSWIC_DEBUG_LOG_L("shswic_state_irregular_charger_proc detect = 0x%02x", detect);

	if (detect != SHSWIC_ID_IRREGULAR_CHARGER)
	{
		shswic_detect_cb_call(SHSWIC_ID_NONE);

		if (detect != SHSWIC_ID_NONE)
		{
			shswic_detect_cb_call(detect);
		}
	}

	return SHSWIC_SUCCESS;
}

int8_t shswic_state_headset_proc(uint8_t detect)
{
	SHSWIC_DEBUG_LOG_L("shswic_state_headset_proc detect = 0x%02x", detect);
	
	if (detect == SHSWIC_ID_HEADSET)
	{
		return SHSWIC_SUCCESS;
	}
	else
	{
		if (detect == SHSWIC_ID_HEADSET_SW)
		{
			shswic_detect_cb_call(SHSWIC_ID_HEADSET_SW);
		}
		else
		{
			shswic_detect_cb_call(SHSWIC_ID_NONE);

			if (detect != SHSWIC_ID_NONE)
			{
				shswic_detect_cb_call(detect);
			}
		}
	}

	return SHSWIC_SUCCESS;
}

int8_t shswic_state_headset_sw_proc(uint8_t detect)
{
	SHSWIC_DEBUG_LOG_L("shswic_state_headset_sw_proc detect = 0x%02x", detect);

	if (detect == SHSWIC_ID_HEADSET_SW)
	{
		return SHSWIC_SUCCESS;
	}
	else
	{
		if (detect == SHSWIC_ID_HEADSET)
		{
			shswic_detect_cb_call(SHSWIC_ID_HEADSET);
		}
		else
		{
			shswic_detect_cb_call(SHSWIC_ID_NONE);

			if (detect != SHSWIC_ID_NONE)
			{
				shswic_detect_cb_call(detect);
			}
		}
	}

	return SHSWIC_SUCCESS;
}

#ifdef CONFIG_CRADLE_SWIC
void shswic_state_cradle_proc(void)
{
	SHSWIC_DEBUG_LOG_L("shswic_state_cradle_proc Start");

	if(!shswic_cb_cradle_state)
	{
		return;
	}
	
	switch(shswic_cb_cradle_state())
	{
		case SHSWIC_ID_CRADLE:
			SHSWIC_DEBUG_LOG_L("shswic_state_cradle_proc Cradle-In");
			if (shswic_cradle_port != SHSWIC_ID_CRADLE)
			{
				shswic_detect_cb_call(SHSWIC_ID_CRADLE);
			}
			break;
		
		case SHSWIC_ID_CRADLE_NONE:
			SHSWIC_DEBUG_LOG_L("shswic_state_cradle_proc Cradle-None");
			if (shswic_cradle_port != SHSWIC_ID_CRADLE_NONE)
			{
				shswic_detect_cb_call(SHSWIC_ID_CRADLE_NONE);
			}
			break;
		
		case SHSWIC_ID_CRADLE_UNKNOWN:
			SHSWIC_DEBUG_LOG_L("shswic_state_cradle_proc Cradle-Unknown");
			break;
		
		default:
			break;
	}

		return;
}
#endif /* CONFIG_CRADLE_SWIC */

int8_t shswic_state_none_proc(uint8_t detect)
{
	SHSWIC_DEBUG_LOG_L("shswic_state_none_proc detect = 0x%02x", detect);

	if (detect != SHSWIC_ID_NONE)
	{
		shswic_detect_cb_call(detect);
	}
	return SHSWIC_SUCCESS;
}

#ifdef CONFIG_SII8334_MHL_TX
int8_t shswic_state_mhl_proc(uint8_t detect)
{
	SHSWIC_DEBUG_LOG_L("shswic_state_mhl_proc detect = 0x%02x", detect);

	if (detect != SHSWIC_ID_MHL)
	{
		SHSWIC_DEBUG_LOG_L("MHL_DISCONNECTED");
		shmhl_notify_id_vbus_state(true, false);
		shswic_detect_cb_call(detect);
	}
	return SHSWIC_SUCCESS;
}
#endif /* CONFIG_SII8334_MHL_TX */

static void shswic_set_shswic_cb_init_state(int state)
{
	atomic_set(&shswic_cb_init_state, state);
	return;
}

static int shswic_get_shswic_cb_init_state(void)
{
	return (atomic_read(&shswic_cb_init_state));
}

static void shswic_set_delay_cancel_sig(int sig)
{
	int l_sig = atomic_read(&shswic_delay_cancel_sig);
	l_sig |= sig;
	atomic_set(&shswic_delay_cancel_sig, l_sig);
	return;
}

static void shswic_clear_delay_cancel_sig(int sig)
{
	int l_sig = atomic_read(&shswic_delay_cancel_sig);
	l_sig &= ~sig;
	atomic_set(&shswic_delay_cancel_sig, l_sig);
	return;
}

static int shswic_get_delay_cancel_sig(void)
{
	return (atomic_read(&shswic_delay_cancel_sig));
}

static shswic_result_t shswic_diag_get_id_data(uint8_t* id_data)
{
	shswic_result_t result = SHSWIC_FAILURE;

	if (id_data == NULL)
	{
		SHSWIC_DEBUG_LOG_H("shswic_diag_get_device_id SHSWIC_PARAM_ERROR");
		return SHSWIC_PARAM_ERROR;
	}

	if (shswic_init_flag == SHSWIC_IMPLEMENT)
	{
		result = shswic_i2c_read(id_data, 1, SHSWIC_REG_DEVICE_ID);
		SHSWIC_DEBUG_LOG_L("shswic_diag_get_device_id = %d", *id_data);
	}
	return result;
}

static int shswic_diag_open(struct inode *inode, struct file *file)
{
	SHSWIC_DEBUG_LOG_L();
	return SHSWIC_SUCCESS;
}

static int shswic_diag_release(struct inode *inode, struct file *file)
{
	SHSWIC_DEBUG_LOG_L();
	return SHSWIC_SUCCESS;
}

static ssize_t shswic_diag_read(struct file *file, char __user *buf, size_t count, loff_t *ppos)
{
	uint8_t id = 0x00;

	SHSWIC_DEBUG_LOG_L();

	shswic_diag_get_id_data(&id);

	if (id != 0x00)
	{
		if (copy_to_user(buf, &id, sizeof(id)))
		{
			return 0;
		}
	}
	else
	{
		return 0;
	}
	return 1;
}

static long shswic_diag_ioctl(struct file *filp, unsigned int cmd, unsigned long arg)
{
	void __user *argp = (void __user *)arg;
	uint8_t detect_id;

	SHSWIC_DEBUG_LOG_L();

	if (argp == NULL)
	{
		SHSWIC_DEBUG_LOG_H("SHSWIC_PARAM_ERROR");
		return -EIO;
	}

	switch (cmd)
	{
		case SHSWIC_IOCTL_ID_READ:
			SHSWIC_DEBUG_LOG_L("SHSWIC_IOCTL_ID_READ");
			detect_id = shswic_detect_id;
			if (copy_to_user(argp, &detect_id, sizeof(detect_id)))
			{
				SHSWIC_DEBUG_LOG_H("copy_to_user Error");
				return -EFAULT;
			}
			break;

		default:
			break;
	}

	return SHSWIC_SUCCESS;
}

shswic_result_t shswic_i2c_write(uint8_t* write_buf, uint8_t len, uint8_t addr)
{
	int ret;
	uint8_t buf[2];
	struct i2c_msg msg;

	if (shswic_i2c_client == NULL)
	{
		SHSWIC_DEBUG_LOG_H("shswic_i2c_write errer null");
		return SHSWIC_PARAM_ERROR;
	}

	SHSWIC_DEBUG_LOG_I2C("shswic_i2c_write I2CAddr = 0x%x, RegAddr = 0x%x, Data = 0x%x", shswic_i2c_client->addr, addr, *write_buf);

	msg.addr  = shswic_i2c_client->addr,
	msg.flags = 0,
	msg.buf   = buf,
	msg.len   = 2,

	buf[0] = addr;
	buf[1] = *write_buf;

	ret = i2c_transfer(shswic_i2c_client->adapter, &msg, 1);

	if (ret < 0)
	{
		SHSWIC_DEBUG_LOG_H("shswic_i2c_write errer = %d", ret);
		return SHSWIC_FAILURE;
	}
	SHSWIC_DEBUG_LOG_I2C("shswic_i2c_write Success");

	return SHSWIC_SUCCESS;
}

shswic_result_t shswic_i2c_read(uint8_t* read_buf, uint8_t len, uint8_t addr)
{
	int ret;
	uint8_t buf[2];
	struct i2c_msg msg[2];

	if (shswic_i2c_client == NULL)
	{
		SHSWIC_DEBUG_LOG_H("shswic_i2c_read errer null");
		return SHSWIC_PARAM_ERROR;
	}

	SHSWIC_DEBUG_LOG_I2C("shswic_i2c_read I2CAddr = 0x%x, RegAddr = 0x%x", shswic_i2c_client->addr, addr);

	msg[0].addr  = shswic_i2c_client->addr,
	msg[0].flags = 0,
	msg[0].buf   = buf,
	msg[0].len   = 1,

	msg[1].addr  = shswic_i2c_client->addr,
	msg[1].flags = I2C_M_RD,
	msg[1].buf   = read_buf,
	msg[1].len   = len,

	buf[0] = addr;

	ret = i2c_transfer(shswic_i2c_client->adapter, msg, 2);
	if (ret < 0)
	{
		if(shswic_init_flag == SHSWIC_IMPLEMENT)
		{
			SHSWIC_DEBUG_LOG_H("shswic_i2c_read errer = %d", ret);
		}
		return SHSWIC_FAILURE;
	}
	SHSWIC_DEBUG_LOG_I2C("shswic_i2c_read Success Data = 0x%x", *read_buf);

	return SHSWIC_SUCCESS;
}

static int shswic_i2c_probe(struct i2c_client *client, const struct i2c_device_id *id)
{
	SHSWIC_DEBUG_LOG_L("shswic_i2c_probe Start");
	shswic_i2c_client = client;

	return SHSWIC_SUCCESS;
}

static int shswic_i2c_remove(struct i2c_client *client)
{
	SHSWIC_DEBUG_LOG_L("shswic_i2c_remove Start");
	return SHSWIC_SUCCESS;
}

static int shswic_pm_driver_probe(struct platform_device *pdev)
{
	SHSWIC_DEBUG_LOG_L("shswic_driver_probe Start");
#ifdef CONFIG_SII8334_MHL_TX
	g_swic_regulator_dev = &pdev->dev;
#endif /* CONFIG_SII8334_MHL_TX */
	return SHSWIC_SUCCESS;
}

#ifdef CONFIG_SII8334_MHL_TX
int shswic_regulator_power(bool on)
{
	static bool swic_power = false;
	static struct regulator *swic_regulator = NULL;

	SHSWIC_DEBUG_LOG_L("shswic_regulator_power Start(%d)", on);

	if (g_swic_regulator_dev == NULL)
	{
		SHSWIC_DEBUG_LOG_H("shswic_regulator_power NULL");
		return SHSWIC_REGULATOR_ERROR_PARAM;
	}

	if (swic_power == on)
	{
		return SHSWIC_REGULATOR_SUCCESS;
	}

	if (on == true)
	{
		if (swic_regulator == NULL)
		{
			swic_regulator = regulator_get(g_swic_regulator_dev, "vbus_shswic");
			if (IS_ERR(swic_regulator))
			{
				SHSWIC_DEBUG_LOG_H("shswic_regulator_power get errer");
				return SHSWIC_REGULATOR_ERROR_GET;
			}
		}

		if (regulator_enable(swic_regulator))
		{
			SHSWIC_DEBUG_LOG_H("shswic_regulator_power ON enable errer");
			regulator_put(swic_regulator);
			swic_regulator = NULL;
			return SHSWIC_REGULATOR_ERROR_ENABLE;
		}
		swic_power = true;
		SHSWIC_DEBUG_LOG_L("shswic_regulator_power ON Success");
	}
	else
	{
		if (swic_regulator == NULL)
		{
			return SHSWIC_REGULATOR_ERROR_STATE;
		}

		if (regulator_disable(swic_regulator))
		{
			SHSWIC_DEBUG_LOG_H("shswic_regulator_power OFF disable errer");
			return SHSWIC_REGULATOR_ERROR_DISABLE;
		}
		regulator_put(swic_regulator);
		swic_regulator = NULL;

		swic_power = false;
		SHSWIC_DEBUG_LOG_L("shswic_regulator_power OFF Success");
	}

	return SHSWIC_REGULATOR_SUCCESS;
}
#endif /* CONFIG_SII8334_MHL_TX */

#ifdef CONFIG_HEADSET_BUTTON_SWIC
void shswic_send_headset_sw_button(uint8_t int_status)
{
	bool is_push_headset_button = false;
	
	if(!shswic_input_dev)
	{
		SHSWIC_DEBUG_LOG_L("shswic_send_headset_sw_button shswic_input_dev[NULL]");
		return;
	}

	if(!shswic_cb_is_push_headset_button)
	{
		SHSWIC_DEBUG_LOG_H("shswic_cb_is_push_headset_button[NULL]");
		return;
	}
	
	is_push_headset_button = shswic_cb_is_push_headset_button(int_status);
	
	if((is_push_headset_button)
	&& (SHSWIC_SEND_HEADSET_SW_OFF == shswic_last_send_sw))
	{
		input_report_key(shswic_input_dev, KEY_MEDIA, 1);
		input_sync(shswic_input_dev);
		shswic_last_send_sw = SHSWIC_SEND_HEADSET_SW_ON;
	}
	else if((!is_push_headset_button)
	&& (SHSWIC_SEND_HEADSET_SW_ON == shswic_last_send_sw))
	{
		input_report_key(shswic_input_dev, KEY_MEDIA, 0);
		input_sync(shswic_input_dev);
		shswic_last_send_sw = SHSWIC_SEND_HEADSET_SW_OFF;
	}
}

static void shswic_register_input_dev(void)
{
	int ret = 0;
	
	if(!shswic_input_dev)
	{
		shswic_input_dev = input_allocate_device();
		if(shswic_input_dev)
		{
			shswic_input_dev->name = SH_SWIC_KEY_DEVICE_NAME;
			shswic_input_dev->id.vendor	= 0x0001;
			shswic_input_dev->id.product	= 1;
			shswic_input_dev->id.version	= 1;

			input_set_capability(shswic_input_dev, EV_KEY, KEY_MEDIA);
			
			ret = input_register_device(shswic_input_dev);
			if(ret)
			{
				SHSWIC_DEBUG_LOG_L("%s ret[%d]", __func__, ret);
				input_free_device(shswic_input_dev);
				shswic_input_dev = NULL;
			}
		}
		else
		{
			SHSWIC_DEBUG_LOG_H("%s FAILED input_allocate_device", __func__);
		}
	}
	else
	{
		SHSWIC_DEBUG_LOG_L("%s Allready registered", __func__);
	}
}

static void shswic_release_input_dev(void)
{
	if(!shswic_input_dev)
	{
		SHSWIC_DEBUG_LOG_L("shswic_release_input_dev shswic_input_dev[NULL]");
		return;
	}

	input_unregister_device(shswic_input_dev);
	input_free_device(shswic_input_dev);
	shswic_input_dev = NULL;
}
#endif /* CONFIG_HEADSET_BUTTON_SWIC */

int shswic_is_implement(void)
{
	return shswic_init_flag;
}


static ssize_t shswic_show_debugflg(struct kobject *kobj, struct kobj_attribute *attr, char *buf)
{
	int i;
	size_t size = -EINVAL;

	SHSWIC_DEBUG_LOG_H("shswic_show_debugflg() Start\n");

	if(NULL == buf) {
		SHSWIC_DEBUG_LOG_H("shswic_show_debugflg() Param Err!!\n");
		return -EINVAL;
	}

	size = snprintf(buf, PAGE_SIZE, "%s\n", shswic_debug_flg);

	SHSWIC_DEBUG_LOG_H("shswic_show_debugflg() size=[%d]\n", size);
	for(i=0; i < size; i++){
		SHSWIC_DEBUG_LOG_H("shswic_show_debugflg() buf[%d]=[0x%X]\n", i, buf[i]);
	}
	return size;
}

static ssize_t shswic_store_debugflg(struct kobject *kobj, struct kobj_attribute *attr, const char *buf, size_t count)
{
	if(NULL == buf) {
		SHSWIC_DEBUG_LOG_H("shswic_store_debugflg() Param Err!!\n");
		return -EINVAL;
	}

	memset(shswic_debug_flg, 0x00, sizeof(shswic_debug_flg));

	if('1' == buf[SHSWIC_DEBUG_LOG_L]) {
		SHSWIC_DEBUG_LOG_H("SHSWIC DEBUG LOG(L) ON\n");
		shswic_debug_flg[SHSWIC_DEBUG_LOG_L] = '1';
	}
	else if('0' == buf[SHSWIC_DEBUG_LOG_L]) {
		SHSWIC_DEBUG_LOG_H("SHSWIC DEBUG LOG(L) OFF\n");
		shswic_debug_flg[SHSWIC_DEBUG_LOG_L] = '0';
	}

	if('1' == buf[SHSWIC_DEBUG_LOG_I2C]) {
		SHSWIC_DEBUG_LOG_H("SHSWIC DEBUG LOG(I2C) ON\n");
		shswic_debug_flg[SHSWIC_DEBUG_LOG_I2C] = '1';
	}
	else if('0' == buf[SHSWIC_DEBUG_LOG_I2C]) {
		SHSWIC_DEBUG_LOG_H("SHSWIC DEBUG LOG(I2C) OFF\n");
		shswic_debug_flg[SHSWIC_DEBUG_LOG_I2C] = '0';
	}

	return count;
}

static struct kobj_attribute shswic_attr = __ATTR(debug_flg, 0600, shswic_show_debugflg, shswic_store_debugflg);

static struct attribute *shswic_attribs[] = {
		&shswic_attr.attr,
		NULL,
};

static struct attribute_group shswic_attr_group = {
	.name = "debug",
	.attrs = shswic_attribs,
};


static void shswic_do_exit(void)
{
	if(shswic_cb_pre_do_exit)
	{
		shswic_cb_pre_do_exit();
	}

	misc_deregister(&shswic_diag_device);

	i2c_del_driver(&shswic_i2c_driver);

	platform_driver_unregister(&shswic_pm_driver);

#ifdef CONFIG_HEADSET_BUTTON_SWIC
	shswic_release_input_dev();
#endif /* CONFIG_HEADSET_BUTTON_SWIC */

	kobject_put(shswic_kobj);

	return;
}

static int __init shswic_init(void)
{
	int err = -ENODEV;

	shswic_debug_flg[SHSWIC_DEBUG_LOG_L] = '0';
	shswic_debug_flg[SHSWIC_DEBUG_LOG_I2C] = '0';
	shswic_debug_flg[SHSWIC_DEBUG_LOG_MAX] = '0';

	SHSWIC_DEBUG_LOG_L("shswic_init");

	shswic_wq = create_singlethread_workqueue(SHSWIC_DRIVER_NAME);
	if (!shswic_wq)
	{
		SHSWIC_DEBUG_LOG_H("create_singlethread_workqueue ERROR");
		shswic_init_flag = SHSWIC_NOT_IMPLEMENT;
		SHSWIC_DEBUG_LOG_L("SHSWIC_NOT_IMPLEMENT");
		return -1;
	}

	mutex_init(&shswic_task_lock);

	wake_lock_init(&shswic_delayed_work_wake_lock, WAKE_LOCK_SUSPEND, "shswic_delayed_work_lock");

	shswic_signal_pkt = (shswic_cmd_t*)kzalloc(sizeof(shswic_cmd_t) * SHSWIC_SIGNAL_PKT_COUNTS, GFP_KERNEL);
	if(!shswic_signal_pkt)
	{
		SHSWIC_DEBUG_LOG_H("shswic_signal_pkt is NULL");
		shswic_init_flag = SHSWIC_NOT_IMPLEMENT;
		return -1;
	}
	shswic_signal_pkt_use_bitflg = 0;
	
	shswic_timer_signal_pkt = (shswic_delay_cmd_t*)kzalloc(sizeof(shswic_delay_cmd_t) * SHSWIC_TIMER_SIGNAL_PKT_COUNTS, GFP_KERNEL);
	if(!shswic_timer_signal_pkt)
	{
		SHSWIC_DEBUG_LOG_H("shswic_timer_signal_pkt is NULL");
		kfree(shswic_signal_pkt);
		shswic_signal_pkt = NULL;

		shswic_init_flag = SHSWIC_NOT_IMPLEMENT;
		return -1;
	}
	shswic_timer_signal_pkt_use_bitflg = 0;

	err = misc_register(&shswic_diag_device);
	if (err)
	{
		SHSWIC_DEBUG_LOG_H("shswic_diag_device: register failed");
		misc_deregister(&shswic_diag_device);
	}

	SHSWIC_DEBUG_LOG_L("i2c_add_driver");
	err = i2c_add_driver(&shswic_i2c_driver);
	if (err < 0)
	{
		SHSWIC_DEBUG_LOG_H("i2c_add_driver: err %d", err);
		i2c_del_driver(&shswic_i2c_driver);
	}

	err = platform_driver_register(&shswic_pm_driver);
	if (err < 0)
	{
		SHSWIC_DEBUG_LOG_H("platform_driver_register: err %d", err);
		platform_driver_unregister(&shswic_pm_driver);
	}

	shswic_kobj = kobject_create_and_add("shswic", NULL);
	if(!shswic_kobj){
		return -1;
	}

	err = sysfs_create_group(shswic_kobj, &shswic_attr_group);
	if(err){
		kobject_put(shswic_kobj);
	}

	shswic_send_signal(SHSWIC_INIT_SIG);
	return 0;
}

static void __exit shswic_exit(void)
{
	shswic_do_exit();

	if(shswic_signal_pkt)
	{
		kfree(shswic_signal_pkt);
		shswic_signal_pkt = NULL;
		shswic_signal_pkt_use_bitflg = 0;
	}

	if(shswic_timer_signal_pkt)
	{
		kfree(shswic_timer_signal_pkt);
		shswic_timer_signal_pkt = NULL;
		shswic_timer_signal_pkt_use_bitflg = 0;
	}
	
	if(shswic_wq)
	{
		destroy_workqueue(shswic_wq);
	}
}

static int shswic_pm_suspend(struct device *dev)
{
    int retval;
    SHSWIC_DEBUG_LOG_L("in suspend.");
    disable_irq(MSM_GPIO_TO_INT(SHSWIC_GPIO_INT));
    retval = enable_irq_wake(MSM_GPIO_TO_INT(SHSWIC_GPIO_INT));
    return retval;
}

static int shswic_pm_resume(struct device *dev)
{
    int retval;
    SHSWIC_DEBUG_LOG_L("in resume.");
    retval = disable_irq_wake(MSM_GPIO_TO_INT(SHSWIC_GPIO_INT));
    
    enable_irq(MSM_GPIO_TO_INT(SHSWIC_GPIO_INT));

    if(!gpio_get_value(SHSWIC_GPIO_INT)) {
	    shswic_send_signal(SHSWIC_DETECT_ISR_SIG);
	}
    return retval;
}

static int __devexit shswic_pm_remove( struct platform_device* dev_p )
{
	return 0;
}

module_init(shswic_init);
module_exit(shswic_exit);

MODULE_DESCRIPTION("SH Swic Driver");
MODULE_LICENSE("GPL v2");
MODULE_AUTHOR("SHARP CORPORATION");
MODULE_VERSION("1.0");
