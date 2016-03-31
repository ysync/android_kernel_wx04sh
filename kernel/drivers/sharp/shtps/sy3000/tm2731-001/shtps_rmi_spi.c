/* drivers/sharp/shtps/sy3000/tm2731-001/shtps_rmi_spi.c
 *
 * Copyright (c) 2012, Sharp. All rights reserved.
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
 *
 */

#include <linux/kernel.h>
#include <linux/module.h>
#include <linux/syscalls.h>
#include <linux/uaccess.h>
#include <linux/gpio.h>
#include <linux/input.h>
#include <linux/delay.h>
#include <linux/fs.h>
#include <linux/poll.h>
#include <linux/cdev.h>
#include <linux/spi/spi.h>
#include <linux/jiffies.h>
#include <linux/irq.h>
#include <linux/wakelock.h>
#include <linux/slab.h>
#include <linux/interrupt.h>
#include <linux/hrtimer.h>
#include <linux/input/mt.h>
#include <linux/pm_qos.h>
#include <mach/cpuidle.h>

#include <sharp/shtps_dev.h>
#include <sharp/sh_smem.h>

#if defined( CONFIG_SHBOOT_CUST )
	#include <sharp/sh_boot_manager.h>
#endif /* CONFIG_SHBOOT_CUST */

/* -----------------------------------------------------------------------------------
 */
//#define SHTPS_DEVELOP_MODE_ENABLE

#ifdef	SHTPS_DEVELOP_MODE_ENABLE
	//#define SHTPS_PERFORMANCE_CHECK_ENABLE
	//#define SHTPS_LOG_SEQ_ENABLE
	//#define SHTPS_LOG_SPIACCESS_SEQ_ENABLE
	#define SHTPS_LOG_DEBUG_ENABLE
	#define	SHTPS_LOG_EVENT_ENABLE
	#define	SHTPS_MODULE_PARAM_ENABLE
	#define	SHTPS_DEBUG_VARIABLE_DEFINES
#endif

#define	SHTPS_LOG_ERROR_ENABLE

#define	SHTPS_FWUPDATE_BUILTIN_ENABLE

/* -----------------------------------------------------------------------------------
 */
//#define SHTPS_FACTORY_MODE_ENABLE

#ifdef	SHTPS_FACTORY_MODE_ENABLE
	#undef	SHTPS_BOOT_FWUPDATE_ENABLE
	#undef	SHTPS_BOOT_FWUPDATE_FORCE_UPDATE
	#define	SHTPS_FMODE_GESTURE_ENABLE
#else
	#define	SHTPS_BOOT_FWUPDATE_ENABLE
	#undef	SHTPS_BOOT_FWUPDATE_FORCE_UPDATE
	#undef	SHTPS_FMODE_GESTURE_ENABLE
#endif

/* -----------------------------------------------------------------------------------
 */
#define	SHTPS_ASYNC_OPEN_ENABLE
#undef	SHTPS_VKEY_INVALID_AREA_ENABLE
#undef	SHTPS_BTLOADER_VER_ENABLE
#undef	SHTPS_SPICLOCK_CONTROL_ENABLE
#define SHTPS_SY_REGMAP_BASE3
#define SHTPS_SPI_FWBLOCKWRITE_ENABLE

#define	SHTPS_MAXFINGER_5_ENABLE
#undef	SHTPS_CHECK_HWID_ENABLE
#define	SHTPS_SMEM_BASELINE_ENABLE
#define SHTPS_FAILSAFE_ENABLE
#define SHTPS_CHARGER_ARMOR_ENABLE
#define SHTPS_PHYSICAL_KEY_ENABLE
#define SHTPS_LPWG_MODE_ENABLE

//#define SHTPS_HOVER_DETECT_ENABLE
//#define SHTPS_PEN_DETECT_ENABLE

#define SHTPS_CHECK_CRC_ERROR_ENABLE

#if defined( CONFIG_SHBOOT_CUST )
	#define SHTPS_SYSTEM_BOOT_MODE_CHECK_ENABLE
#endif /* CONFIG_SHBOOT_CUST ) */

#define SHTPS_PROXIMITY_SUPPORT_ENABLE

/* -----------------------------------------------------------------------------------
 */

#if defined( CONFIG_SPI_CUST2_SH )
	#define	SHTPS_SPI_BLOCKREAD_ENABLE
#endif /* CONFIG_SPI_CUST2_SH */

#define	SHTPS_SY3X00_SPI_CLOCK_FAST			1100000
#define	SHTPS_SY3X00_SPI_CLOCK_SLOW			600000
#ifdef	SHTPS_SPI_BLOCKREAD_ENABLE
	#define	SHTPS_SY3X00_SPI_CLOCK_READ_SPEED	SHTPS_SY3X00_SPI_CLOCK_FAST
	#define	SHTPS_SY3X00_SPI_CLOCK_READ_WAIT	40
	#define SHTPS_SY3X00_FINGER_SPIREAD_CNT		2
#else
	#define	SHTPS_SY3X00_SPI_CLOCK_READ_SPEED	SHTPS_SY3X00_SPI_CLOCK_SLOW
	#define	SHTPS_SY3X00_SPI_CLOCK_READ_WAIT	21
	#define SHTPS_SY3X00_FINGER_SPIREAD_CNT		0
#endif
#define	SHTPS_SY3X00_SPI_CLOCK_WRITE_SPEED		SHTPS_SY3X00_SPI_CLOCK_SLOW
#define	SHTPS_SY3X00_SPI_CLOCK_WRITE_WAIT		0

#define	SHTPS_SY3X00_SPIBLOCK_BUFSIZE_MAX		((2 + 3 + SHTPS_FINGER_MAX * 5) * 2)
#define	SHTPS_SY3X00_SPIBLOCK_BUFSIZE			(2 + 3 + 5 * 2)

#define SHTPS_SPI_RETRY_COUNT					200
#define SHTPS_SPI_RETRY_WAIT					5

/* -----------------------------------------------------------------------------------
 */
#ifdef SHTPS_LOG_EVENT_ENABLE
	#define SHTPS_LOG_OUTPUT_SWITCH_ENABLE
#endif /* #if defined( SHTPS_LOG_EVENT_ENABLE ) */

#if defined ( CONFIG_SHTPS_SY3000_AUTOREZERO_CONTROL )
	#define SHTPS_AUTOREZERO_SINGLE_FINGER_ENABLE
	#define SHTPS_AUTOREZERO_CONTROL_ENABLE
#else
	#undef	SHTPS_AUTOREZERO_SINGLE_FINGER_ENABLE
	#undef	SHTPS_AUTOREZERO_CONTROL_ENABLE
#endif /* #if defined ( CONFIG_SHTPS_SY3000_AUTOREZERO_CONTROL ) */

#if defined( CONFIG_SHTPS_SY3000_ALWAYS_ACTIVEMODE )
//	#define SHTPS_LOW_POWER_MODE_ENABLE
#endif /* #if defined( CONFIG_SHTPS_SY3000_ALWAYS_ACTIVEMODE ) */

#if defined(SHTPS_PERFORMANCE_CHECK_ENABLE)
	#define SHTPS_PERFORMANCE_CHECK_PIN_ENABLE
	//#define SHTPS_PERFORMANCE_TIME_LOG_ENABLE
#endif /* SHTPS_PERFORMANCE_CHECK_ENABLE */

/* -----------------------------------------------------------------------------------
 */
#if	defined( CONFIG_SHTPS_SY3000_TM2731_001 )
	#include "shtps_fw_tm2731-001.h"

#else
	#undef	SHTPS_BOOT_FWUPDATE_ENABLE
	#undef	SHTPS_BOOT_FWUPDATE_FORCE_UPDATE
	#undef	SHTPS_FWUPDATE_BUILTIN_ENABLE
#endif

/* -----------------------------------------------------------------------------------
 */
#if defined( SHTPS_LOG_SEQ_ENABLE )
	#include "shtps_seqlog.h"
	#define _log_msg_sync(id, fmt, ...) \
		_seq_log(SHTPS_SEQLOG_TAG, _SEQ_LOG_MSGKIND_SYNC, id, fmt "\n", ##__VA_ARGS__)

	#define _log_msg_send(id, fmt, ...) \
		_seq_log(SHTPS_SEQLOG_TAG, _SEQ_LOG_MSGKIND_SEND, id, fmt "\n", ##__VA_ARGS__)

	#define _log_msg_recv(id, fmt, ...) \
		_seq_log(SHTPS_SEQLOG_TAG, _SEQ_LOG_MSGKIND_RECEIVE, id, fmt "\n", ##__VA_ARGS__)
#else
	#define _log_msg_sync(id, fmt, ...)
	#define _log_msg_send(id, fmt, ...)
	#define _log_msg_recv(id, fmt, ...)
#endif /* defined( SHTPS_LOG_SEQ_ENABLE ) */

#if defined( SHTPS_LOG_ERROR_ENABLE )
	#define SHTPS_LOG_ERR_PRINT(...)	printk(KERN_ERR "[shtps] " __VA_ARGS__)
#else
	#define SHTPS_LOG_ERR_PRINT(...)
#endif /* defined( SHTPS_LOG_ERROR_ENABLE ) */

#if defined( SHTPS_LOG_DEBUG_ENABLE )
    #if defined( SHTPS_LOG_OUTPUT_SWITCH_ENABLE )
        #define	SHTPS_LOG_DBG_PRINT(...)					\
            if((gLogOutputEnable & 0x02) != 0){				\
                printk(KERN_DEBUG "[shtps] " __VA_ARGS__);	\
            }
        #define SHTPS_LOG_DEBUG(p)				\
            if((gLogOutputEnable & 0x02) != 0){	\
                p								\
            }
        #define SHTPS_LOG_FUNC_CALL()							\
            if((gLogOutputEnable & 0x02) != 0){					\
                printk(KERN_DEBUG "[shtps] %s()\n", __func__);	\
            }
        #define SHTPS_LOG_FUNC_CALL_INPARAM(param)						\
            if((gLogOutputEnable & 0x02) != 0){							\
                printk(KERN_DEBUG "[shtps]%s(%d)\n", __func__, param);	\
            }
    #else
        #define	SHTPS_LOG_DBG_PRINT(...)	printk(KERN_DEBUG "[shtps] " __VA_ARGS__)
        #define SHTPS_LOG_DEBUG(p)	p
        #define SHTPS_LOG_FUNC_CALL()	printk(KERN_DEBUG "[shtps] %s()\n", __func__)
        #define SHTPS_LOG_FUNC_CALL_INPARAM(param)	\
                                        printk(KERN_DEBUG "[shtps]%s(%d)\n", __func__, param)
    #endif /* SHTPS_LOG_OUTPUT_SWITCH_ENABLE */
#else
	#define	SHTPS_LOG_DBG_PRINT(...)
	#define SHTPS_LOG_DEBUG(p)
	#define SHTPS_LOG_FUNC_CALL()
	#define SHTPS_LOG_FUNC_CALL_INPARAM(param)
#endif /* defined( SHTPS_LOG_DEBUG_ENABLE ) */

#if defined( SHTPS_LOG_EVENT_ENABLE ) && defined( SHTPS_LOG_OUTPUT_SWITCH_ENABLE )
    #define SHTPS_LOG_EVENT(p)				\
        if((gLogOutputEnable & 0x01) != 0){	\
            p								\
        }
#elif defined( SHTPS_LOG_EVENT_ENABLE )
	#define SHTPS_LOG_EVENT(p)	p
#else
	#define SHTPS_LOG_EVENT(p)
#endif /* defined( SHTPS_LOG_EVENT_ENABLE ) */

#define SPI_ERR_CHECK(check, label) \
	if((check)) goto label


/* -----------------------------------------------------------------------------------
 */
#if defined( SHTPS_DEBUG_VARIABLE_DEFINES )
	static int SHTPS_DRAG_DIR_FIX_CNT =				3;

	#if defined( SHTPS_MODULE_PARAM_ENABLE )
		module_param(SHTPS_DRAG_DIR_FIX_CNT, int, S_IRUGO | S_IWUSR);
	#endif /* SHTPS_MODULE_PARAM_ENABLE */

	static int SHTPS_DRAG_THRESH_VAL_X_ZERO = 		4;
	static int SHTPS_DRAG_THRESH_VAL_Y_ZERO = 		4;
	static int SHTPS_DRAG_THRESH_RETURN_TIME_ZERO =	250;

	#if defined( SHTPS_MODULE_PARAM_ENABLE )
		module_param(SHTPS_DRAG_THRESH_VAL_X_ZERO, int, S_IRUGO | S_IWUSR);
		module_param(SHTPS_DRAG_THRESH_VAL_Y_ZERO, int, S_IRUGO | S_IWUSR);
		module_param(SHTPS_DRAG_THRESH_RETURN_TIME_ZERO, int, S_IRUGO | S_IWUSR);
	#endif /* SHTPS_MODULE_PARAM_ENABLE */

	static int SHTPS_DRAG_THRESH_VAL_X_1ST = 		CONFIG_SHTPS_SY3000_SINGLE_1ST_DRSTEP;
	static int SHTPS_DRAG_THRESH_VAL_X_2ND = 		CONFIG_SHTPS_SY3000_SINGLE_2ND_DRSTEP;
	static int SHTPS_DRAG_THRESH_VAL_X_1ST_MULTI =	CONFIG_SHTPS_SY3000_MULTI_1ST_DRSTEP;
	static int SHTPS_DRAG_THRESH_VAL_X_2ND_MULTI =	CONFIG_SHTPS_SY3000_MULTI_2ND_DRSTEP;
	static int SHTPS_DRAG_THRESH_VAL_Y_1ST = 		CONFIG_SHTPS_SY3000_SINGLE_1ST_DRSTEP;
	static int SHTPS_DRAG_THRESH_VAL_Y_2ND = 		CONFIG_SHTPS_SY3000_SINGLE_2ND_DRSTEP;
	static int SHTPS_DRAG_THRESH_VAL_Y_1ST_MULTI =	CONFIG_SHTPS_SY3000_MULTI_1ST_DRSTEP;
	static int SHTPS_DRAG_THRESH_VAL_Y_2ND_MULTI =	CONFIG_SHTPS_SY3000_MULTI_2ND_DRSTEP;
	static int SHTPS_DRAG_THRESH_RETURN_TIME =		250;
#else /* #if defined( SHTPS_DEBUG_VARIABLE_DEFINES ) */
	#define SHTPS_DRAG_DIR_FIX_CNT					3

	#define SHTPS_DRAG_THRESH_VAL_X_ZERO			4
	#define SHTPS_DRAG_THRESH_VAL_Y_ZERO			4
	#define SHTPS_DRAG_THRESH_RETURN_TIME_ZERO		250

	#define SHTPS_DRAG_THRESH_VAL_X_1ST 			CONFIG_SHTPS_SY3000_SINGLE_1ST_DRSTEP
	#define SHTPS_DRAG_THRESH_VAL_X_2ND 			CONFIG_SHTPS_SY3000_SINGLE_2ND_DRSTEP
	#define SHTPS_DRAG_THRESH_VAL_X_1ST_MULTI		CONFIG_SHTPS_SY3000_MULTI_1ST_DRSTEP
	#define SHTPS_DRAG_THRESH_VAL_X_2ND_MULTI		CONFIG_SHTPS_SY3000_MULTI_2ND_DRSTEP
	#define SHTPS_DRAG_THRESH_VAL_Y_1ST 			CONFIG_SHTPS_SY3000_SINGLE_1ST_DRSTEP
	#define SHTPS_DRAG_THRESH_VAL_Y_2ND 			CONFIG_SHTPS_SY3000_SINGLE_2ND_DRSTEP
	#define SHTPS_DRAG_THRESH_VAL_Y_1ST_MULTI		CONFIG_SHTPS_SY3000_MULTI_1ST_DRSTEP
	#define SHTPS_DRAG_THRESH_VAL_Y_2ND_MULTI		CONFIG_SHTPS_SY3000_MULTI_2ND_DRSTEP
	#define SHTPS_DRAG_THRESH_RETURN_TIME			250
#endif /* #if defined( SHTPS_DEBUG_VARIABLE_DEFINES ) */

#define SHTPS_FWDATA_BLOCK_SIZE_MAX				0xFFFF

#define SHTPS_BOOTLOADER_ACK_TMO				1000
#define SHTPS_FWTESTMODE_ACK_TMO				1000
#define SHTPS_DIAGPOLL_TIME						100

#define SHTPS_FINGER_WIDTH_PALMDET				15
#define SHTPS_FINGER_WIDTH_MIN					1
#define SHTPS_FINGER_WIDTH_MAX					8

#define SHTPS_LOS_SINGLE						0x05
#define SHTPS_LOS_MULTI							0x0D

#define SHTPS_STARTUP_MIN_TIME					300
#define SHTPS_POWERON_WAIT_MS					400
#define SHTPS_POWEROFF_WAIT_MS					10
#define SHTPS_HWRESET_TIME_US					1
#define SHTPS_HWRESET_AFTER_TIME_MS				1
#define SHTPS_HWRESET_WAIT_MS					290
#define SHTPS_SWRESET_WAIT_MS					300
#define SHTPS_RESET_BOOTLOADER_WAIT_MS			400
#define SHTPS_SLEEP_OUT_WAIT_US					67500
#define SHTPS_SLEEP_IN_WAIT_MS					230

#if defined( SHTPS_SY_REGMAP_BASE3 )
#define SHTPS_FLASH_ERASE_WAIT_MS				2000
#else
#define SHTPS_FLASH_ERASE_WAIT_MS				1000
#endif /* #if defined( SHTPS_SY_REGMAP_BASE3 ) */

#define SHTPS_POSTYPE_X							0
#define SHTPS_POSTYPE_Y							1

#define SHTPS_POS_SCALE_X(ts)	(((CONFIG_SHTPS_SY3000_PANEL_SIZE_X) * 10000) / ts->map.fn11.ctrl.maxXPosition)
#define SHTPS_POS_SCALE_Y(ts)	(((CONFIG_SHTPS_SY3000_PANEL_SIZE_Y) * 10000) / ts->map.fn11.ctrl.maxYPosition)

#if defined( SHTPS_CHARGER_ARMOR_ENABLE )
	#if defined( SHTPS_DEBUG_VARIABLE_DEFINES )
		static int SHTPS_CHARGER_ARMOR_STRENGTH_DEFAULT =	0x10;
		static int SHTPS_CHARGER_ARMOR_STRENGTH =			0x20;

		#if defined( SHTPS_MODULE_PARAM_ENABLE )
			module_param(SHTPS_CHARGER_ARMOR_STRENGTH_DEFAULT, int, S_IRUGO | S_IWUSR);
			module_param(SHTPS_CHARGER_ARMOR_STRENGTH, int, S_IRUGO | S_IWUSR);
		#endif /* SHTPS_MODULE_PARAM_ENABLE */

	#else
		#define SHTPS_CHARGER_ARMOR_STRENGTH_DEFAULT		0x10
		#define SHTPS_CHARGER_ARMOR_STRENGTH				0x20
	#endif /* #if defined( SHTPS_DEBUG_VARIABLE_DEFINES ) */
#endif /* #if defined( SHTPS_CHARGER_ARMOR_ENABLE ) */

/* -----------------------------------------------------------------------------------
 */
static DEFINE_MUTEX(shtps_ctrl_lock);
static DEFINE_MUTEX(shtps_loader_lock);

/* -----------------------------------------------------------------------------------
 */

struct shtps_irq_info{
	int							irq;
	u8							state;
	u8							wake;
};

struct shtps_state_info{
	int							state;
	int							mode;
	int							starterr;
	unsigned long				starttime;
};

struct shtps_loader_info{
	int							ack;
	wait_queue_head_t			wait_ack;
};

struct shtps_diag_info{
	u8							pos_mode;
	u8							tm_mode;
	u8							tm_data[SHTPS_TM_TXNUM_MAX * SHTPS_TM_RXNUM_MAX * 2];
	int							event;
#if defined( SHTPS_PHYSICAL_KEY_ENABLE )
	int							event_touchkey;
#endif /* #if defined( SHTPS_PHYSICAL_KEY_ENABLE ) */
	int							tm_ack;
	int							tm_stop;
	wait_queue_head_t			wait;
	wait_queue_head_t			tm_wait_ack;
};

struct shtps_facetouch_info{
	int							mode;
	int							state;
	int							off_detect;
	int							palm_thresh;
	int							wake_sig;
	wait_queue_head_t			wait_off;
	struct wake_lock			wake_lock;
};

struct shtps_offset_info{
	int							enabled;
	u16							base[5];
	signed short				diff[12];
};

struct shtps_polling_info{
	int							boot_rezero_flag;
	int							stop_margin;
	int							stop_count;
	int							single_fingers_count;
	int							single_fingers_max;
	u8							single_fingers_enable;
};

struct shtps_drag_hist{
	int							pre;
	u8							dir;
	u8							count;
};

#if defined(SHTPS_LPWG_MODE_ENABLE)
struct shtps_lpwg_ctrl{
    u8                          lpwg_switch;
    u8                          is_notified;
	u8							notify_enable;
    u8                          doze_wakeup_threshold;
    u8                          clip[2];
	struct wake_lock            wake_lock;
	struct pm_qos_request		pm_qos_lock_idle;
	signed long					pm_qos_idle_value;

	u8							block_touchevent;
	unsigned long				wakeup_time;
	u8							tu_rezero_req;
	unsigned long				notify_time;
	struct delayed_work			notify_interval_delayed_work;
};
#endif /* SHTPS_LPWG_MODE_ENABLE */

#if defined( SHTPS_ASYNC_OPEN_ENABLE )
struct shtps_req_msg {
	struct list_head queue;
	void	(*complete)(void *context);
	void	*context;
	int		status;
	int		event;
	void	*param_p;
};
#endif /* #if defined( SHTPS_ASYNC_OPEN_ENABLE ) */

struct shtps_rmi_spi {
	struct spi_device*			spi;
	struct input_dev*			input;
	int							rst_pin;
	struct shtps_irq_info		irq_mgr;
	struct shtps_touch_info		report_info;
	struct shtps_touch_info		center_info;
	struct shtps_state_info		state_mgr;
	struct shtps_loader_info	loader;
	struct shtps_diag_info		diag;
	struct shtps_facetouch_info	facetouch;
	struct shtps_offset_info	offset;
	struct shtps_polling_info	poll_info;
	struct rmi_map				map;
	struct shtps_touch_state	touch_state;
	wait_queue_head_t			wait_start;
	struct delayed_work 		tmo_check;
	unsigned char				finger_state[3];	/* SHTPS_FINGER_MAX/4+1 */
	struct hrtimer				rezero_delayed_timer;
	struct work_struct			rezero_delayed_work;

	#if defined( SHTPS_ASYNC_OPEN_ENABLE )
		struct workqueue_struct		*workqueue_p;
		struct work_struct			work_data;
		struct list_head			queue;
		spinlock_t					queue_lock;
		struct shtps_req_msg		*cur_msg_p;
	#endif /* #if defined( SHTPS_ASYNC_OPEN_ENABLE ) */

	#if defined( SHTPS_VKEY_INVALID_AREA_ENABLE )
		u8							invalid_area_touch;
	#endif /* #if defined( SHTPS_VKEY_INVALID_AREA_ENABLE ) */

	#if defined( SHTPS_LOW_POWER_MODE_ENABLE )
		u8							lpmode_state;
	#endif /* #if defined( SHTPS_LOW_POWER_MODE_ENABLE ) */

	u16							bt_ver;
	u8							*purposebuf_p;
	char						phys[32];

	#if defined( SHTPS_CHARGER_ARMOR_ENABLE )
		u8							charger_armor_state;
	#endif /* #if defined( SHTPS_CHARGER_ARMOR_ENABLE ) */

	#if defined( SHTPS_PHYSICAL_KEY_ENABLE )
		struct input_dev*			input_key;
		u16							keycodes[3];
		u8							key_state;
	#endif /* #if defined( SHTPS_PHYSICAL_KEY_ENABLE ) */

	#if defined(SHTPS_SYSTEM_BOOT_MODE_CHECK_ENABLE)
		u16							system_boot_mode;
	#endif /* SHTPS_SYSTEM_BOOT_MODE_CHECK_ENABLE */

    #if defined(SHTPS_LPWG_MODE_ENABLE)
        struct shtps_lpwg_ctrl		lpwg;
    #endif /* SHTPS_LPWG_MODE_ENABLE */

	struct shtps_drag_hist		drag_hist[SHTPS_FINGER_MAX][2];
};

static dev_t 					shtpsif_devid;
static struct class*			shtpsif_class;
static struct device*			shtpsif_device;
struct cdev 					shtpsif_cdev;
static struct shtps_rmi_spi*	gShtps_rmi_spi = NULL;

#if defined( SHTPS_LOG_OUTPUT_SWITCH_ENABLE )
static u8						gLogOutputEnable = 0;
#endif /* #if defined( SHTPS_LOG_OUTPUT_SWITCH_ENABLE ) */

#if defined( SHTPS_MODULE_PARAM_ENABLE )
	static int shtps_irq_wake_state = 0;
	static int shtps_spi_clk_ctrl_state = 0;
	static int shtps_rezero_state = 0;

	module_param(shtps_irq_wake_state, int, S_IRUGO);
	module_param(shtps_spi_clk_ctrl_state, int, S_IRUGO);
	module_param(shtps_rezero_state, int, S_IRUGO | S_IWUSR);
#endif /* SHTPS_MODULE_PARAM_ENABLE */

#if defined(SHTPS_LPWG_MODE_ENABLE)
	#if defined( SHTPS_DEBUG_VARIABLE_DEFINES )
		static int SHTPS_LPWG_DOZE_INTERVAL_DEF =			0x03;
		static int SHTPS_LPWG_DOZE_INTERVAL =				0x0A;
		static int SHTPS_LPWG_DOZE_WAKEUP_THRESHOLD_DEF = 	0x64;
		static int SHTPS_LPWG_DOZE_HOLDOFF_DEF = 			0x05;
		static int SHTPS_LPWG_DOZE_HOLDOFF = 				0x00;
		static int SHTPS_LPWG_DOZE_CTL = 					0x02;
		static int SHTPS_LPWG_MAX_ACTIVE_DURATION = 		0x01;
		static int SHTPS_LPWG_TX_CLIP_LO = 					0x60;
		static int SHTPS_LPWG_TX_CLIP_HI = 					0x60;
		static int SHTPS_LPWG_DOZE_0D_THRESHOLD = 			0xFF;
		static int SHTPS_LPWG_NORMAL_0D_THRESHOLD = 		0x8C;
		static int SHTPS_LPWG_QOS_LATENCY_DEF_VALUE = 		1999;
		static int SHTPS_LPWG_BLOCK_TIME_MAX_MS = 				2000;
		static int SHTPS_LPWG_MIN_NOTIFY_INTERVAL = 		1000;

		#if defined( SHTPS_MODULE_PARAM_ENABLE )
			module_param(SHTPS_LPWG_DOZE_INTERVAL_DEF, int, S_IRUGO | S_IWUSR);
			module_param(SHTPS_LPWG_DOZE_INTERVAL, int, S_IRUGO | S_IWUSR);
			module_param(SHTPS_LPWG_DOZE_WAKEUP_THRESHOLD_DEF, int, S_IRUGO | S_IWUSR);
			module_param(SHTPS_LPWG_DOZE_HOLDOFF_DEF, int, S_IRUGO | S_IWUSR);
			module_param(SHTPS_LPWG_DOZE_HOLDOFF, int, S_IRUGO | S_IWUSR);
			module_param(SHTPS_LPWG_DOZE_CTL, int, S_IRUGO | S_IWUSR);
			module_param(SHTPS_LPWG_MAX_ACTIVE_DURATION, int, S_IRUGO | S_IWUSR);
			module_param(SHTPS_LPWG_TX_CLIP_LO, int, S_IRUGO | S_IWUSR);
			module_param(SHTPS_LPWG_TX_CLIP_HI, int, S_IRUGO | S_IWUSR);
			module_param(SHTPS_LPWG_DOZE_0D_THRESHOLD, int, S_IRUGO | S_IWUSR);
			module_param(SHTPS_LPWG_NORMAL_0D_THRESHOLD, int, S_IRUGO | S_IWUSR);
			module_param(SHTPS_LPWG_QOS_LATENCY_DEF_VALUE, int, S_IRUGO | S_IWUSR);
			module_param(SHTPS_LPWG_BLOCK_TIME_MAX_MS, int, S_IRUGO | S_IWUSR);
			module_param(SHTPS_LPWG_MIN_NOTIFY_INTERVAL, int, S_IRUGO | S_IWUSR);
		#endif /* SHTPS_MODULE_PARAM_ENABLE */
	#else
		#define SHTPS_LPWG_DOZE_INTERVAL_DEF				0x03
		#define SHTPS_LPWG_DOZE_INTERVAL					0x0A
		#define SHTPS_LPWG_DOZE_WAKEUP_THRESHOLD_DEF		0x64
		#define SHTPS_LPWG_DOZE_HOLDOFF_DEF					0x05
		#define SHTPS_LPWG_DOZE_HOLDOFF 					0x00
		#define SHTPS_LPWG_DOZE_CTL 						0x02
		#define SHTPS_LPWG_MAX_ACTIVE_DURATION 				0x01
		#define SHTPS_LPWG_TX_CLIP_LO						0x60
		#define SHTPS_LPWG_TX_CLIP_HI						0x60
		#define SHTPS_LPWG_DOZE_0D_THRESHOLD 				0xFF
		#define SHTPS_LPWG_NORMAL_0D_THRESHOLD	 			0x8C
		#define SHTPS_LPWG_QOS_LATENCY_DEF_VALUE	 		1999
		#define SHTPS_LPWG_BLOCK_TIME_MAX_MS	 		2000
		#define SHTPS_LPWG_MIN_NOTIFY_INTERVAL	 			1000
	#endif /* SHTPS_DEBUG_VARIABLE_DEFINES ) */

	#if defined(SHTPS_PROXIMITY_SUPPORT_ENABLE)
		#include <sharp/proximity.h>

		#if defined( SHTPS_DEBUG_VARIABLE_DEFINES )
			static int	SHTPS_LPWG_PROXIMITY_SUPPORT_ENABLE   = 1;

			#if defined( SHTPS_MODULE_PARAM_ENABLE )
				module_param(SHTPS_LPWG_PROXIMITY_SUPPORT_ENABLE, int, S_IRUGO | S_IWUSR);
			#endif /* SHTPS_MODULE_PARAM_ENABLE */
		#else
			#define SHTPS_LPWG_PROXIMITY_SUPPORT_ENABLE		1
		#endif /* SHTPS_DEBUG_VARIABLE_DEFINES ) */
	#endif /* SHTPS_PROXIMITY_SUPPORT_ENABLE */
#endif /* SHTPS_LPWG_MODE_ENABLE */
/* -----------------------------------------------------------------------------------
 */
enum{
	SHTPS_FWTESTMODE_V01 = 0x00,
	SHTPS_FWTESTMODE_V02,
	SHTPS_FWTESTMODE_V03,
};

enum{
	SHTPS_REZERO_REQUEST_REZERO				= 0x01,
	SHTPS_REZERO_REQUEST_AUTOREZERO_DISABLE	= 0x02,
	SHTPS_REZERO_REQUEST_AUTOREZERO_ENABLE	= 0x04,
	SHTPS_REZERO_REQUEST_WAKEUP_REZERO		= 0x08,
};

enum{
	SHTPS_REZERO_HANDLE_EVENT_MTD = 0,
	SHTPS_REZERO_HANDLE_EVENT_TOUCH,
	SHTPS_REZERO_HANDLE_EVENT_TOUCHUP,
};

enum{
	SHTPS_REZERO_TRIGGER_BOOT = 0,
	SHTPS_REZERO_TRIGGER_WAKEUP,
	SHTPS_REZERO_TRIGGER_ENDCALL,
};

enum{
	SHTPS_EVENT_TU,
	SHTPS_EVENT_TD,
	SHTPS_EVENT_DRAG,
	SHTPS_EVENT_MTDU,
};

enum{
	SHTPS_TOUCH_STATE_NO_TOUCH = 0,
	SHTPS_TOUCH_STATE_FINGER,
	SHTPS_TOUCH_STATE_PEN,
	SHTPS_TOUCH_STATE_HOVER,
};

enum{
	SHTPS_STARTUP_SUCCESS,
	SHTPS_STARTUP_FAILED
};

enum{
	SHTPS_IRQ_WAKE_DISABLE,
	SHTPS_IRQ_WAKE_ENABLE,
};

enum{
	SHTPS_IRQ_STATE_DISABLE,
	SHTPS_IRQ_STATE_ENABLE,
};

enum{
	SHTPS_MODE_NORMAL,
	SHTPS_MODE_LOADER,
};

enum{
	SHTPS_EVENT_START,
	SHTPS_EVENT_STOP,
	SHTPS_EVENT_SLEEP,
	SHTPS_EVENT_WAKEUP,
	SHTPS_EVENT_STARTLOADER,
	SHTPS_EVENT_STARTTM,
	SHTPS_EVENT_STOPTM,
	SHTPS_EVENT_FACETOUCHMODE_ON,
	SHTPS_EVENT_FACETOUCHMODE_OFF,
	SHTPS_EVENT_GETSENSOR,
	SHTPS_EVENT_INTERRUPT,
	SHTPS_EVENT_TIMEOUT,
};

enum{
	SHTPS_STATE_IDLE,
	SHTPS_STATE_WAIT_WAKEUP,
	SHTPS_STATE_WAIT_READY,
	SHTPS_STATE_ACTIVE,
	SHTPS_STATE_BOOTLOADER,
	SHTPS_STATE_FACETOUCH,
	SHTPS_STATE_FWTESTMODE,
	SHTPS_STATE_SLEEP,
	SHTPS_STATE_SLEEP_FACETOUCH,
};

enum{
	SHTPS_PHYSICAL_KEY_DOWN = 0,
	SHTPS_PHYSICAL_KEY_UP,
	SHTPS_PHYSICAL_KEY_SWEEPON,
	SHTPS_PHYSICAL_KEY_NUM,
};

enum{
	SHTPS_IRQ_FLASH		= 0x01,
	SHTPS_IRQ_STATE		= 0x02,
	SHTPS_IRQ_ABS 		= 0x04,
	SHTPS_IRQ_ANALOG 	= 0x08,
    SHTPS_IRQ_BUTTON 	= 0x10,
    SHTPS_IRQ_SENSOR 	= 0x20,
	SHTPS_IRQ_ALL		= (  SHTPS_IRQ_FLASH
							| SHTPS_IRQ_STATE
							| SHTPS_IRQ_ABS
							| SHTPS_IRQ_ANALOG
							| SHTPS_IRQ_BUTTON
							| SHTPS_IRQ_SENSOR),
};

#if defined( SHTPS_LOW_POWER_MODE_ENABLE )
enum{
	SHTPS_LPMODE_OFF					= 0x00,
	SHTPS_LPMODE_NON_CONTINUOUS_REQ_ON	= 0x01,
	SHTPS_LPMODE_CONTINUOUS_REQ_ON		= 0x02,
	SHTPS_LPMODE_ON						= ( SHTPS_LPMODE_NON_CONTINUOUS_REQ_ON
											| SHTPS_LPMODE_CONTINUOUS_REQ_ON),
};
#endif /* #if defined( SHTPS_LOW_POWER_MODE_ENABLE ) */

#if defined( SHTPS_ASYNC_OPEN_ENABLE )
enum{
	SHTPS_FUNC_REQ_EVEMT_OPEN = 0,
	SHTPS_FUNC_REQ_EVEMT_CLOSE,
	SHTPS_FUNC_REQ_EVEMT_ENABLE,
	SHTPS_FUNC_REQ_EVEMT_DISABLE,
#if defined( SHTPS_CHECK_CRC_ERROR_ENABLE )
	SHTPS_FUNC_REQ_EVEMT_CHECK_CRC_ERROR,
#endif /* if defined( SHTPS_CHECK_CRC_ERROR_ENABLE ) */
};
#endif /* #if defined( SHTPS_ASYNC_OPEN_ENABLE ) */

enum{
	SHTPS_DRAG_DIR_NONE = 0,
	SHTPS_DRAG_DIR_PLUS,
	SHTPS_DRAG_DIR_MINUS,
};

#if defined(SHTPS_PROXIMITY_SUPPORT_ENABLE)
enum{
	SHTPS_PROXIMITY_ENABLE = SH_PROXIMITY_ENABLE,
	SHTPS_PROXIMITY_DISABLE = SH_PROXIMITY_DISABLE,
};

enum{
	SHTPS_PROXIMITY_NEAR = SH_PROXIMITY_NEAR,
	SHTPS_PROXIMITY_FAR = SH_PROXIMITY_FAR,
};
#endif /* SHTPS_PROXIMITY_SUPPORT_ENABLE */

typedef int (shtps_state_func)(struct shtps_rmi_spi *ts, int param);
struct shtps_state_func {
	shtps_state_func	*enter;
	shtps_state_func	*start;
	shtps_state_func	*stop;
	shtps_state_func	*sleep;
	shtps_state_func	*wakeup;
	shtps_state_func	*start_ldr;
	shtps_state_func	*start_tm;
	shtps_state_func	*stop_tm;
	shtps_state_func	*facetouch_on;
	shtps_state_func	*facetouch_off;
	shtps_state_func	*get_sensor;
	shtps_state_func	*interrupt;
	shtps_state_func	*timeout;
};

#if defined( SHTPS_VKEY_INVALID_AREA_ENABLE )
	#if defined( SHTPS_DEBUG_VARIABLE_DEFINES )
		static struct{
			int	sx;
			int sy;
			int ex;
			int ey;
		} shtps_invalid_area[] = {
			{    0, 1270,  719, 1279 },
			{ -1, -1, -1, -1 }
		};
	#else /* #if defined( SHTPS_DEBUG_VARIABLE_DEFINES ) */
		static const struct{
			int	sx;
			int sy;
			int ex;
			int ey;
		} shtps_invalid_area[] = {
			{    0, 1270,  719, 1279 },
			{ -1, -1, -1, -1 }
		};
	#endif /* #if defined( SHTPS_DEBUG_VARIABLE_DEFINES ) */
#endif /* #if defined( SHTPS_VKEY_INVALID_AREA_ENABLE ) */

/* -----------------------------------------------------------------------------------
 */
static int request_event(struct shtps_rmi_spi *ts, int event, int param);
static int state_change(struct shtps_rmi_spi *ts, int state);
static int shtps_statef_nop(struct shtps_rmi_spi *ts, int param);
static int shtps_statef_cmn_error(struct shtps_rmi_spi *ts, int param);
static int shtps_statef_cmn_stop(struct shtps_rmi_spi *ts, int param);
static int shtps_statef_idle_start(struct shtps_rmi_spi *ts, int param);
static int shtps_statef_idle_start_ldr(struct shtps_rmi_spi *ts, int param);
static int shtps_statef_idle_int(struct shtps_rmi_spi *ts, int param);
static int shtps_statef_waiwakeup_stop(struct shtps_rmi_spi *ts, int param);
static int shtps_statef_waiwakeup_int(struct shtps_rmi_spi *ts, int param);
static int shtps_statef_waiwakeup_tmo(struct shtps_rmi_spi *ts, int param);
static int shtps_state_waiready_stop(struct shtps_rmi_spi *ts, int param);
static int shtps_state_waiready_int(struct shtps_rmi_spi *ts, int param);
static int shtps_state_waiready_tmo(struct shtps_rmi_spi *ts, int param);
static int shtps_statef_active_enter(struct shtps_rmi_spi *ts, int param);
static int shtps_statef_active_stop(struct shtps_rmi_spi *ts, int param);
static int shtps_statef_active_sleep(struct shtps_rmi_spi *ts, int param);
static int shtps_statef_active_starttm(struct shtps_rmi_spi *ts, int param);
static int shtps_statef_active_int(struct shtps_rmi_spi *ts, int param);
static int shtps_statef_loader_enter(struct shtps_rmi_spi *ts, int param);
static int shtps_statef_loader_stop(struct shtps_rmi_spi *ts, int param);
static int shtps_statef_loader_int(struct shtps_rmi_spi *ts, int param);
static int shtps_statef_fwtm_enter(struct shtps_rmi_spi *ts, int param);
static int shtps_statef_fwtm_stop(struct shtps_rmi_spi *ts, int param);
static int shtps_statef_fwtm_stoptm(struct shtps_rmi_spi *ts, int param);
static int shtps_statef_fwtm_getsensor(struct shtps_rmi_spi *ts, int param);
static int shtps_statef_fwtm_int(struct shtps_rmi_spi *ts, int param);
static int shtps_statef_sleep_enter(struct shtps_rmi_spi *ts, int param);
static int shtps_statef_sleep_wakeup(struct shtps_rmi_spi *ts, int param);
static int shtps_statef_sleep_starttm(struct shtps_rmi_spi *ts, int param);
static int shtps_statef_sleep_int(struct shtps_rmi_spi *ts, int param);
static int shtps_fwupdate_enable(struct shtps_rmi_spi *ts);
static int shtps_start(struct shtps_rmi_spi *ts);
static int shtps_wait_startup(struct shtps_rmi_spi *ts);
static u16 shtps_fwver(struct shtps_rmi_spi *ts);
static int shtps_statef_cmn_facetouch_on(struct shtps_rmi_spi *ts, int param);
static int shtps_statef_cmn_facetouch_off(struct shtps_rmi_spi *ts, int param);
static int shtps_statef_active_facetouch_on(struct shtps_rmi_spi *ts, int param);
static int shtps_statef_sleep_facetouch_on(struct shtps_rmi_spi *ts, int param);
static int shtps_statef_facetouch_sleep(struct shtps_rmi_spi *ts, int param);
static int shtps_statef_facetouch_starttm(struct shtps_rmi_spi *ts, int param);
static int shtps_statef_facetouch_facetouch_off(struct shtps_rmi_spi *ts, int param);
static int shtps_statef_facetouch_int(struct shtps_rmi_spi *ts, int param);
static int shtps_statef_sleep_facetouch_enter(struct shtps_rmi_spi *ts, int param);
static int shtps_statef_sleep_facetouch_stop(struct shtps_rmi_spi *ts, int param);
static int shtps_statef_sleep_facetouch_wakeup(struct shtps_rmi_spi *ts, int param);
static int shtps_statef_sleep_facetouch_starttm(struct shtps_rmi_spi *ts, int param);
static int shtps_statef_sleep_facetouch_facetouch_off(struct shtps_rmi_spi *ts, int param);
static int shtps_statef_sleep_facetouch_int(struct shtps_rmi_spi *ts, int param);
static void shtps_event_report(struct shtps_rmi_spi *ts, struct shtps_touch_info *info, u8 event);
static void shtps_rezero(struct shtps_rmi_spi *ts);
static void shtps_delayed_rezero(struct shtps_rmi_spi *ts, unsigned long delay_us);
static void shtps_read_touchevent(struct shtps_rmi_spi *ts, int state);
#if defined(SHTPS_LPWG_MODE_ENABLE)
static void shtps_lpwg_wakelock(struct shtps_rmi_spi *ts, int on);
static void shtps_get_lpwg_def_settings(struct shtps_rmi_spi *ts);
#endif /* SHTPS_LPWG_MODE_ENABLE */

#if defined( SHTPS_BOOT_FWUPDATE_ENABLE ) || defined( SHTPS_CHECK_CRC_ERROR_ENABLE )
static int shtps_fw_update(struct shtps_rmi_spi *ts, const unsigned char *fw_data);
#endif /* #if defined( SHTPS_BOOT_FWUPDATE_ENABLE ) || defined( SHTPS_CHECK_CRC_ERROR_ENABLE ) */

#if defined( SHTPS_ASYNC_OPEN_ENABLE )
static int shtps_get_bt_ver(struct shtps_rmi_spi *ts);
#endif /* #if defined( SHTPS_ASYNC_OPEN_ENABLE ) */

/* -----------------------------------------------------------------------------------
 */
const static struct shtps_state_func state_idle = {
    .enter          = shtps_statef_nop,
    .start          = shtps_statef_idle_start,
    .stop           = shtps_statef_cmn_stop,
    .sleep          = shtps_statef_nop,
    .wakeup         = shtps_statef_nop,
    .start_ldr      = shtps_statef_idle_start_ldr,
    .start_tm       = shtps_statef_cmn_error,
    .stop_tm        = shtps_statef_cmn_error,
    .facetouch_on   = shtps_statef_cmn_facetouch_on,
    .facetouch_off  = shtps_statef_cmn_facetouch_off,
    .get_sensor     = shtps_statef_nop,
    .interrupt      = shtps_statef_idle_int,
    .timeout        = shtps_statef_nop
};

const static struct shtps_state_func state_waiwakeup = {
    .enter          = shtps_statef_nop,
    .start          = shtps_statef_nop,
    .stop           = shtps_statef_waiwakeup_stop,
    .sleep          = shtps_statef_nop,
    .wakeup         = shtps_statef_nop,
    .start_ldr      = shtps_statef_cmn_error,
    .start_tm       = shtps_statef_cmn_error,
    .stop_tm        = shtps_statef_cmn_error,
    .facetouch_on   = shtps_statef_cmn_facetouch_on,
    .facetouch_off  = shtps_statef_cmn_facetouch_off,
    .get_sensor     = shtps_statef_nop,
    .interrupt      = shtps_statef_waiwakeup_int,
    .timeout        = shtps_statef_waiwakeup_tmo
};

const static struct shtps_state_func state_waiready = {
    .enter          = shtps_statef_nop,
    .start          = shtps_statef_nop,
    .stop           = shtps_state_waiready_stop,
    .sleep          = shtps_statef_nop,
    .wakeup         = shtps_statef_nop,
    .start_ldr      = shtps_statef_cmn_error,
    .start_tm       = shtps_statef_cmn_error,
    .stop_tm        = shtps_statef_cmn_error,
    .facetouch_on   = shtps_statef_cmn_facetouch_on,
    .facetouch_off  = shtps_statef_cmn_facetouch_off,
    .get_sensor     = shtps_statef_nop,
    .interrupt      = shtps_state_waiready_int,
    .timeout        = shtps_state_waiready_tmo
};

const static struct shtps_state_func state_active = {
    .enter          = shtps_statef_active_enter,
    .start          = shtps_statef_nop,
    .stop           = shtps_statef_active_stop,
    .sleep          = shtps_statef_active_sleep,
    .wakeup         = shtps_statef_nop,
    .start_ldr      = shtps_statef_cmn_error,
    .start_tm       = shtps_statef_active_starttm,
    .stop_tm        = shtps_statef_cmn_error,
    .facetouch_on   = shtps_statef_active_facetouch_on,
    .facetouch_off  = shtps_statef_nop,
    .get_sensor     = shtps_statef_nop,
    .interrupt      = shtps_statef_active_int,
    .timeout        = shtps_statef_nop
};

const static struct shtps_state_func state_loader = {
    .enter          = shtps_statef_loader_enter,
    .start          = shtps_statef_nop,
    .stop           = shtps_statef_loader_stop,
    .sleep          = shtps_statef_nop,
    .wakeup         = shtps_statef_nop,
    .start_ldr      = shtps_statef_nop,
    .start_tm       = shtps_statef_cmn_error,
    .stop_tm        = shtps_statef_cmn_error,
    .facetouch_on   = shtps_statef_cmn_facetouch_on,
    .facetouch_off  = shtps_statef_cmn_facetouch_off,
    .get_sensor     = shtps_statef_nop,
    .interrupt      = shtps_statef_loader_int,
    .timeout        = shtps_statef_nop
};

const static struct shtps_state_func state_facetouch = {
    .enter          = shtps_statef_nop,
    .start          = shtps_statef_nop,
    .stop           = shtps_statef_active_stop,
    .sleep          = shtps_statef_facetouch_sleep,
    .wakeup         = shtps_statef_nop,
    .start_ldr      = shtps_statef_cmn_error,
    .start_tm       = shtps_statef_facetouch_starttm,
    .stop_tm        = shtps_statef_cmn_error,
    .facetouch_on   = shtps_statef_nop,
    .facetouch_off  = shtps_statef_facetouch_facetouch_off,
    .get_sensor     = shtps_statef_nop,
    .interrupt      = shtps_statef_facetouch_int,
    .timeout        = shtps_statef_nop
};

const static struct shtps_state_func state_fwtm = {
    .enter          = shtps_statef_fwtm_enter,
    .start          = shtps_statef_nop,
    .stop           = shtps_statef_fwtm_stop,
    .sleep          = shtps_statef_nop,
    .wakeup         = shtps_statef_nop,
    .start_ldr      = shtps_statef_cmn_error,
    .start_tm       = shtps_statef_nop,
    .stop_tm        = shtps_statef_fwtm_stoptm,
    .facetouch_on   = shtps_statef_cmn_facetouch_on,
    .facetouch_off  = shtps_statef_cmn_facetouch_off,
    .get_sensor     = shtps_statef_fwtm_getsensor,
    .interrupt      = shtps_statef_fwtm_int,
    .timeout        = shtps_statef_nop
};

const static struct shtps_state_func state_sleep = {
    .enter          = shtps_statef_sleep_enter,
    .start          = shtps_statef_nop,
    .stop           = shtps_statef_cmn_stop,
    .sleep          = shtps_statef_nop,
    .wakeup         = shtps_statef_sleep_wakeup,
    .start_ldr      = shtps_statef_cmn_error,
    .start_tm       = shtps_statef_sleep_starttm,
    .stop_tm        = shtps_statef_cmn_error,
    .facetouch_on   = shtps_statef_sleep_facetouch_on,
    .facetouch_off  = shtps_statef_nop,
    .get_sensor     = shtps_statef_nop,
    .interrupt      = shtps_statef_sleep_int,
    .timeout        = shtps_statef_nop
};

const static struct shtps_state_func state_sleep_facetouch = {
    .enter          = shtps_statef_sleep_facetouch_enter,
    .start          = shtps_statef_nop,
    .stop           = shtps_statef_sleep_facetouch_stop,
    .sleep          = shtps_statef_nop,
    .wakeup         = shtps_statef_sleep_facetouch_wakeup,
    .start_ldr      = shtps_statef_cmn_error,
    .start_tm       = shtps_statef_sleep_facetouch_starttm,
    .stop_tm        = shtps_statef_cmn_error,
    .facetouch_on   = shtps_statef_nop,
    .facetouch_off  = shtps_statef_sleep_facetouch_facetouch_off,
    .get_sensor     = shtps_statef_nop,
    .interrupt      = shtps_statef_sleep_facetouch_int,
    .timeout        = shtps_statef_nop
};

const static struct shtps_state_func *state_func_tbl[] = {
	&state_idle,
	&state_waiwakeup,
	&state_waiready,
	&state_active,
	&state_loader,
	&state_facetouch,
	&state_fwtm,
	&state_sleep,
	&state_sleep_facetouch,
};

/* -----------------------------------------------------------------------------------
 */
#define SHTPS_PERFORMANCE_CHECK_STATE_START	(0)
#define SHTPS_PERFORMANCE_CHECK_STATE_CONT	(1)
#define SHTPS_PERFORMANCE_CHECK_STATE_END	(2)

#if defined(SHTPS_PERFORMANCE_CHECK_PIN_ENABLE)
	#define SHTPS_PERFORMANCE_CHECK_PIN		(50)
	static int shtps_performance_check_pin_state = 0;
#endif /* SHTPS_PERFORMANCE_CHECK_PIN_ENABLE */

#if defined( SHTPS_PERFORMANCE_TIME_LOG_ENABLE )
	static int shtps_performace_log_point = 0;
	static struct timeval shtps_performance_log_tv;
#endif /* SHTPS_PERFORMANCE_TIME_LOG_ENABLE */

static inline void shtps_performance_check_init(void)
{
	#if defined( SHTPS_PERFORMANCE_CHECK_PIN_ENABLE )
		int result;
		result = gpio_request(SHTPS_PERFORMANCE_CHECK_PIN, "tps_test");
		if(result < 0){
			printk(KERN_DEBUG "[shtps]test pin gpio_request() error : %d\n", result);
		}
		result = gpio_tlmm_config(GPIO_CFG(SHTPS_PERFORMANCE_CHECK_PIN, 0, GPIO_CFG_OUTPUT, GPIO_CFG_NO_PULL, GPIO_CFG_2MA), GPIO_CFG_ENABLE);
		if(result < 0){
			printk(KERN_DEBUG "[shtps]test pin gpio_tlmm_config() error : %d\n", result);
		}
		
		shtps_performance_check_pin_state = 0;
		gpio_set_value(SHTPS_PERFORMANCE_CHECK_PIN, shtps_performance_check_pin_state);
	#endif /* SHTPS_PERFORMANCE_CHECK_PIN_ENABLE */

	#if defined( SHTPS_PERFORMANCE_TIME_LOG_ENABLE )
		shtps_performace_log_point = 0;
	#endif /* SHTPS_PERFORMANCE_TIME_LOG_ENABLE */
}

static inline void shtps_performance_check(int state)
{
	#if defined( SHTPS_PERFORMANCE_CHECK_PIN_ENABLE )
		if(state == SHTPS_PERFORMANCE_CHECK_STATE_START){
			shtps_performance_check_pin_state = 1;
		}else{
			shtps_performance_check_pin_state = 
				(shtps_performance_check_pin_state == 0)? 1 : 0;
		}
		
		gpio_set_value(SHTPS_PERFORMANCE_CHECK_PIN, shtps_performance_check_pin_state);
		
		if(state == SHTPS_PERFORMANCE_CHECK_STATE_END){
			shtps_performance_check_pin_state = 0;
			gpio_set_value(SHTPS_PERFORMANCE_CHECK_PIN, shtps_performance_check_pin_state);
		}
	#endif /* SHTPS_PERFORMANCE_CHECK_PIN_ENABLE */

	#if defined( SHTPS_PERFORMANCE_TIME_LOG_ENABLE )
		if(state == SHTPS_PERFORMANCE_CHECK_STATE_START){
			shtps_performace_log_point = 1;
		}else{
			static struct timeval tv;
			do_gettimeofday(&tv);
			
			printk("[shtps][performace] pt:%02d time:%ldus\n",
				shtps_performace_log_point++,
				(tv.tv_sec * 1000000 + tv.tv_usec) - 
					(shtps_performance_log_tv.tv_sec * 1000000 + shtps_performance_log_tv.tv_usec));
		}
		do_gettimeofday(&shtps_performance_log_tv);
	#endif /* SHTPS_PERFORMANCE_TIME_LOG_ENABLE */
}

/* -----------------------------------------------------------------------------------
 */
static int shtps_device_access_setup(struct shtps_rmi_spi *ts)
{
	#if defined( SHTPS_SPICLOCK_CONTROL_ENABLE )
		extern	void sh_spi_tps_active(struct spi_device *spi);

		sh_spi_tps_active(ts->spi);
		SHTPS_LOG_DBG_PRINT("spi active\n");

		#if defined( SHTPS_MODULE_PARAM_ENABLE )
			shtps_spi_clk_ctrl_state = 1;
		#endif /* SHTPS_MODULE_PARAM_ENABLE */
	#endif	/* SHTPS_SPICLOCK_CONTROL_ENABLE */

	return 0;
}

static int shtps_device_access_teardown(struct shtps_rmi_spi *ts)
{
	#if defined( SHTPS_SPICLOCK_CONTROL_ENABLE )
		extern	void sh_spi_tps_standby(struct spi_device *spi);

		sh_spi_tps_standby(ts->spi);
		SHTPS_LOG_DBG_PRINT("spi standby\n");

		#if defined( SHTPS_MODULE_PARAM_ENABLE )
			shtps_spi_clk_ctrl_state = 0;
		#endif /* SHTPS_MODULE_PARAM_ENABLE */
	#endif /* SHTPS_SPICLOCK_CONTROL_ENABLE */

	return 0;
}

/* -----------------------------------------------------------------------------------
 */
#if defined(SHTPS_PROXIMITY_SUPPORT_ENABLE)
static int shtps_proximity_state_check(struct shtps_rmi_spi *ts)
{
	int state = -1;
	int data = -1;

	SHTPS_LOG_DBG_PRINT("[proximity] check state start\n");
	PROX_stateread_func(&state, &data);
	SHTPS_LOG_DBG_PRINT("[proximity] check state end\n");

	if (state == SHTPS_PROXIMITY_ENABLE &&
		data == SHTPS_PROXIMITY_NEAR) {
		return 1;
	} else {
		return 0;
	}
}

static int shtps_proximity_check(struct shtps_rmi_spi *ts)
{
	int data = -1;

	SHTPS_LOG_DBG_PRINT("[proximity] check start\n");
	PROX_dataread_func(&data);
	SHTPS_LOG_DBG_PRINT("[proximity] check end\n");

	return data;
}
#endif /* SHTPS_PROXIMITY_SUPPORT_ENABLE */

#if defined( SHTPS_SPI_FWBLOCKWRITE_ENABLE )
/* -----------------------------------------------------------------------------------
 */
static int shtps_rmi_write_fw_data(struct shtps_rmi_spi *ts, u16 addr, u8 *buf)
{
	struct spi_message	message;
	struct spi_transfer	t[17];
	u8					tx_buf[2];
	u16					blockSize;
	int					status;
	int					i;

	blockSize = F34_QUERY_BLOCKSIZE(ts->map.fn34.query.data);

	memset(&t, 0, sizeof(t));

	tx_buf[0] = (addr >> 8) & 0xff;
	tx_buf[1] = addr & 0xff;

	spi_message_init(&message);

	t[0].tx_buf 		= tx_buf;
	t[0].rx_buf 		= NULL;
	t[0].len    		= 2;
	t[0].speed_hz		= SHTPS_SY3X00_SPI_CLOCK_WRITE_SPEED;
	t[0].delay_usecs	= SHTPS_SY3X00_SPI_CLOCK_WRITE_WAIT;
	// t[0].delay_usecs	= 65;
	spi_message_add_tail(&t[0], &message);

	for(i = 0; i < blockSize; i++){
		t[i+1].tx_buf 		= &buf[i];
		t[i+1].rx_buf 		= NULL;
		t[i+1].len    		= 1;
		t[i+1].speed_hz		= SHTPS_SY3X00_SPI_CLOCK_WRITE_SPEED;
		t[i+1].delay_usecs	= SHTPS_SY3X00_SPI_CLOCK_WRITE_WAIT;
		// t[i+1].delay_usecs	= 6;

		spi_message_add_tail(&t[i+1], &message);
	}

#ifdef SHTPS_LOG_SPIACCESS_SEQ_ENABLE
	_log_msg_sync(LOGMSG_ID__SPI_WRITE, "0x%04X|0x%02X", addr, data);
#endif /* SHTPS_LOG_SPIACCESS_SEQ_ENABLE */
	status = spi_sync(ts->spi, &message);
#ifdef SHTPS_LOG_SPIACCESS_SEQ_ENABLE
	_log_msg_sync(LOGMSG_ID__SPI_WRITE_COMP, "%d", status);
#endif /* SHTPS_LOG_SPIACCESS_SEQ_ENABLE */

	return status;
}
#endif /* #if defined( SHTPS_SPI_FWBLOCKWRITE_ENABLE ) */

/* -----------------------------------------------------------------------------------
 */
static void shtps_rmi_write_async_spicomplete(void *arg)
{
	complete(arg);
	udelay(65);
}
static int shtps_rmi_write_async(struct shtps_rmi_spi *ts, u16 addr, u8 data)
{
	DECLARE_COMPLETION_ONSTACK(done);
	struct spi_message	message;
	struct spi_transfer	t;
	u8					tx_buf[3];
	int					status;

	memset(&t, 0, sizeof(t));

	spi_message_init(&message);
	spi_message_add_tail(&t, &message);
	t.tx_buf 		= tx_buf;
	t.rx_buf 		= NULL;
	t.len    		= 3;
	t.speed_hz		= SHTPS_SY3X00_SPI_CLOCK_WRITE_SPEED;
	t.delay_usecs	= SHTPS_SY3X00_SPI_CLOCK_WRITE_WAIT;

	tx_buf[0] = (addr >> 8) & 0xff;
	tx_buf[1] = addr & 0xff;
	tx_buf[2] = data;

	#ifdef SHTPS_LOG_SPIACCESS_SEQ_ENABLE
		_log_msg_sync(LOGMSG_ID__SPI_WRITE, "0x%04X|0x%02X", addr, data);
	#endif /* SHTPS_LOG_SPIACCESS_SEQ_ENABLE */

	message.complete = shtps_rmi_write_async_spicomplete;
	message.context  = &done;
	status = spi_async(ts->spi, &message);
	if(status == 0){
		wait_for_completion(&done);
		status = message.status;
	}
	message.context = NULL;

	#ifdef SHTPS_LOG_SPIACCESS_SEQ_ENABLE
		_log_msg_sync(LOGMSG_ID__SPI_WRITE_COMP, "%d", status);
	#endif /* SHTPS_LOG_SPIACCESS_SEQ_ENABLE */

	return status;
}
static int shtps_rmi_write(struct shtps_rmi_spi *ts, u16 addr, u8 data)
{
	int retry = 3;
	int err;

	do{
		err = shtps_rmi_write_async(ts, addr, data);
	}while(err != 0 && retry-- > 0);

	return err;
}

#if	defined( SHTPS_SPI_BLOCKREAD_ENABLE )
static void shtps_rmi_one_cs_spicomplete(void *arg)
{
	complete(arg);
	udelay(40);
}
static int shtps_rmi_read_block(struct shtps_rmi_spi *ts, u16 addr, u8 *readbuf, u32 size, u8 *buf)
{
	DECLARE_COMPLETION_ONSTACK(done);
	struct spi_message	message;
	struct spi_transfer	t;
	u8 *txbuf_p;
	u8 *rxbuf_p;
	int status;
	int i;

	memset(&t, 0, sizeof(t));
	memset(buf, 0, (size+2*2));

	txbuf_p = buf;
	rxbuf_p = buf + sizeof(u8)*(size + 2);

	txbuf_p[0] = ((addr >> 8) & 0xff) | 0x80;
	txbuf_p[1] = addr & 0xff;

	spi_message_init(&message);
	spi_message_add_tail(&t, &message);

	t.tx_buf		= txbuf_p;
	t.rx_buf  		= rxbuf_p;
	t.len			= size + 2;
	t.bits_per_word	= 8;
	t.speed_hz		= SHTPS_SY3X00_SPI_CLOCK_READ_SPEED;
	t.delay_usecs	= SHTPS_SY3X00_SPI_CLOCK_READ_WAIT;

	message.complete = shtps_rmi_one_cs_spicomplete;
	message.context  = &done;
	status = spi_async(ts->spi, &message);
	if(status == 0){
		wait_for_completion(&done);
		status = message.status;
	}
	message.context = NULL;

	rxbuf_p += 2;
	for(i = 0;i < size;i++){
		*readbuf++ = *rxbuf_p++;
	}

	return status;
}
static int shtps_rmi_read(struct shtps_rmi_spi *ts, u16 addr, u8 *buf, u32 size)
{
	int status = 0;
	int	i;
	u32	s;
	u8 transbuf[(SHTPS_SY3X00_SPIBLOCK_BUFSIZE+2)*2];
	int retry = SHTPS_SPI_RETRY_COUNT;

	do{
		s = size;
		for(i=0;i<size;i+=SHTPS_SY3X00_SPIBLOCK_BUFSIZE){
			status = shtps_rmi_read_block(ts,
				addr+i,
				buf+i,
				(s>SHTPS_SY3X00_SPIBLOCK_BUFSIZE)?(SHTPS_SY3X00_SPIBLOCK_BUFSIZE):(s),
				transbuf);
			//
			if(status){
				struct timespec tu;
				tu.tv_sec = (time_t)0;
				tu.tv_nsec = SHTPS_SPI_RETRY_WAIT * 1000000;
				hrtimer_nanosleep(&tu, NULL, HRTIMER_MODE_REL, CLOCK_MONOTONIC);
				break;
			}
			s -= SHTPS_SY3X00_SPIBLOCK_BUFSIZE;
		}
	}while(status != 0 && retry-- > 0);

	if(status){
		SHTPS_LOG_ERR_PRINT("spi read error\n");
	}
	#if defined( SHTPS_LOG_DEBUG_ENABLE )
	else if(retry < (SHTPS_SPI_RETRY_COUNT)){
		SHTPS_LOG_DBG_PRINT("spi retry num = %d\n", retry);
	}
	#endif /* SHTPS_LOG_DEBUG_ENABLE */
	
	return status;
}
#else	/* SHTPS_SPI_BLOCKREAD_ENABLE */
/* -----------------------------------------------------------------------------------
 */
struct shtps_spi_async_ctrl_tbl{
	struct completion		work;
	struct spi_message		message;
	struct spi_transfer		t;
	int						wait_utime;
	u8						tx_buf[2];
	u8						rx_buf[2];
};

/* -----------------------------------------------------------------------------------
 */
static void shtps_rmi_readb_async_comp_1(void *arg)
{
	struct shtps_spi_async_ctrl_tbl	*ptr_p;
	int	wait_utime;

	ptr_p = (struct shtps_spi_async_ctrl_tbl *)arg;
	wait_utime = ptr_p->wait_utime;

	if(wait_utime){
		udelay( wait_utime );
	}
}

/* -----------------------------------------------------------------------------------
 */
static void shtps_rmi_readb_async_comp_2(void *arg)
{
	struct shtps_spi_async_ctrl_tbl	*ptr_p;
	int	wait_utime;

	ptr_p = (struct shtps_spi_async_ctrl_tbl *)arg;
	wait_utime = ptr_p->wait_utime;

	complete( &(ptr_p->work) );

	if(wait_utime){
		udelay( wait_utime );
	}
}
/* -----------------------------------------------------------------------------------
 */
static int shtps_rmi_readb(struct shtps_rmi_spi *ts, u16 addr, u8 *buf)
{
	struct	shtps_spi_async_ctrl_tbl	spi_work[2], *spi_p;

	int	status;

	*buf = 0;
	memset(spi_work, 0, sizeof(spi_work));

	/* Write Reg.Addr. */
	spi_p = &spi_work[0];

	spi_message_init( &(spi_p->message) );
	spi_message_add_tail( &(spi_p->t), &(spi_p->message));

	spi_p->wait_utime		= 40;
	spi_p->tx_buf[0]		= ((addr >> 8) & 0xff) | 0x80;
	spi_p->tx_buf[1]		= addr & 0xff;
	spi_p->t.tx_buf			= spi_p->tx_buf;
	spi_p->t.rx_buf			= NULL;
	spi_p->t.bits_per_word	= 8;
	spi_p->t.len			= 2;
	spi_p->t.speed_hz		= SHTPS_SY3X00_SPI_CLOCK_READ_SPEED;
	spi_p->t.delay_usecs	= SHTPS_SY3X00_SPI_CLOCK_READ_WAIT;
	spi_p->message.complete	= shtps_rmi_readb_async_comp_1;
	spi_p->message.context	= spi_p;

	#ifdef SHTPS_LOG_SPIACCESS_SEQ_ENABLE
		_log_msg_sync(LOGMSG_ID__SPI_WRITEADDR, "0x%04X", addr);
	#endif /* SHTPS_LOG_SPIACCESS_SEQ_ENABLE */
	status = spi_async(ts->spi, &(spi_p->message));
	#ifdef SHTPS_LOG_SPIACCESS_SEQ_ENABLE
		_log_msg_sync(LOGMSG_ID__SPI_WRITEADDR_COMP, "%d", status);
	#endif /* SHTPS_LOG_SPIACCESS_SEQ_ENABLE */
	if(status){
		return status;
	}

	/* Read data */
	spi_p = &spi_work[1];

	init_completion( &(spi_p->work) );

	spi_message_init( &(spi_p->message) );
	spi_message_add_tail( &(spi_p->t), &(spi_p->message));

	spi_p->wait_utime		= 40;
	spi_p->t.tx_buf			= NULL;
	spi_p->t.rx_buf			= spi_p->rx_buf;
	spi_p->t.bits_per_word	= 8;
	spi_p->t.len			= 1;
	spi_p->t.speed_hz		= SHTPS_SY3X00_SPI_CLOCK_READ_SPEED;
	spi_p->t.delay_usecs	= SHTPS_SY3X00_SPI_CLOCK_READ_WAIT;
	spi_p->message.complete	= shtps_rmi_readb_async_comp_2;
	spi_p->message.context	= spi_p;

	#ifdef SHTPS_LOG_SPIACCESS_SEQ_ENABLE
		_log_msg_sync(LOGMSG_ID__SPI_READ, "0x%04X", addr);
	#endif /* SHTPS_LOG_SPIACCESS_SEQ_ENABLE */
	status = spi_async(ts->spi, &(spi_p->message));
	#ifdef SHTPS_LOG_SPIACCESS_SEQ_ENABLE
		_log_msg_sync(LOGMSG_ID__SPI_READ_COMP, "0x%04X|0x%02X", addr, rx_buf[0]);
	#endif /* SHTPS_LOG_SPIACCESS_SEQ_ENABLE */
	if(status == 0){
		wait_for_completion( &(spi_p->work) );
		status = spi_p->message.status;

	}else{
		return status;

	}

	*buf = spi_p->rx_buf[0];

	return status;
}
/* -----------------------------------------------------------------------------------
 */
static int shtps_rmi_read(struct shtps_rmi_spi *ts, u16 addr, u8 *buf, u32 size)
{
	int i;
	int status;

	for(i = 0;i < size;i++){
		int retry;
		retry = 3;
		do{
			status = shtps_rmi_readb(ts, addr + i, buf + i);
		}while(status != 0 && retry-- > 0);

		if(status){
			goto err_exit;
		}
	}
	return 0;

err_exit:
	return status;
}
#endif /* #if defined( SHTPS_SPI_BLOCKREAD_ENABLE ) */

/* -----------------------------------------------------------------------------------
 */
#if defined( SHTPS_PERFORMANCE_CHECK_ENABLE )
	static irqreturn_t shtps_irq_handler(int irq, void *dev_id)
	{
		shtps_performance_check(SHTPS_PERFORMANCE_CHECK_STATE_START);
		return IRQ_WAKE_THREAD;
	}
#endif /* SHTPS_PERFORMANCE_CHECK_ENABLE */

static irqreturn_t shtps_irq(int irq, void *dev_id)
{
	struct shtps_rmi_spi	*ts = dev_id;

	shtps_performance_check(SHTPS_PERFORMANCE_CHECK_STATE_CONT);

	_log_msg_send( LOGMSG_ID__IRQ_NOTIFY, "");
	_log_msg_recv( LOGMSG_ID__IRQ_NOTIFY, "");
	
	#if defined(SHTPS_LPWG_MODE_ENABLE)
	if(0 != ts->lpwg.lpwg_switch && SHTPS_STATE_SLEEP == ts->state_mgr.state){
		shtps_lpwg_wakelock(ts, 1);
	}
	#endif /* defined(SHTPS_LPWG_MODE_ENABLE) */
	
	request_event(ts, SHTPS_EVENT_INTERRUPT, 1);

	#if defined(SHTPS_LPWG_MODE_ENABLE)
	shtps_lpwg_wakelock(ts, 0);
	#endif /* defined(SHTPS_LPWG_MODE_ENABLE) */

	shtps_performance_check(SHTPS_PERFORMANCE_CHECK_STATE_END);

	return IRQ_HANDLED;
}

static void shtps_work_tmof(struct work_struct *data)
{
	struct delayed_work  *dw = container_of(data, struct delayed_work, work);
	struct shtps_rmi_spi *ts = container_of(dw, struct shtps_rmi_spi, tmo_check);

	_log_msg_sync( LOGMSG_ID__TIMER_TIMEOUT, "");
	request_event(ts, SHTPS_EVENT_TIMEOUT, 0);
}

static enum hrtimer_restart shtps_delayed_rezero_timer_function(struct hrtimer *timer)
{
	struct shtps_rmi_spi *ts = container_of(timer, struct shtps_rmi_spi, rezero_delayed_timer);

	_log_msg_send( LOGMSG_ID__DELAYED_REZERO_TRIGGER, "");
	schedule_work(&ts->rezero_delayed_work);
	return HRTIMER_NORESTART;
}
static void shtps_rezero_delayed_work_function(struct work_struct *work)
{
	struct shtps_rmi_spi *ts = container_of(work, struct shtps_rmi_spi, rezero_delayed_work);

	_log_msg_recv( LOGMSG_ID__DELAYED_REZERO_TRIGGER, "");
	mutex_lock(&shtps_ctrl_lock);

	#if defined(SHTPS_LPWG_MODE_ENABLE)
		if(ts->lpwg.block_touchevent != 0){
			if(ts->touch_state.numOfFingers == 0){
				shtps_rezero(ts);
				ts->lpwg.block_touchevent = 0;
				ts->lpwg.tu_rezero_req = 0;
			}else{
				SHTPS_LOG_DBG_PRINT("LPWG delayed rezero fail by touch state\n");
				ts->lpwg.tu_rezero_req = 1;
			}
		}else{
			shtps_rezero(ts);
			ts->lpwg.block_touchevent = 0;
			ts->lpwg.tu_rezero_req = 0;
		}
	#else
		shtps_rezero(ts);
	#endif /* SHTPS_LPWG_MODE_ENABLE */

	mutex_unlock(&shtps_ctrl_lock);
}
static void shtps_set_palmthresh(struct shtps_rmi_spi *ts, int thresh)
{
#if defined( CONFIG_SHTPS_SY3000_FACETOUCH_DETECT ) || defined( CONFIG_SHTPS_SY3000_FACETOUCH_OFF_DETECT )
	if(ts->facetouch.palm_thresh != thresh){
	#if defined( SHTPS_SY_REGMAP_BASE3 )
		u8 val;

		shtps_rmi_read( ts,
						ts->map.fn11.ctrlBase + 0x29,
						&val, 1);

		shtps_rmi_write(ts,
						ts->map.fn11.ctrlBase + 0x29,
						(val & 0xC0) | (ts->facetouch.palm_thresh & 0x3F);
		_log_msg_sync( LOGMSG_ID__SET_PALM_THRESH, "0x%04X|0x%02X",
							ts->map.fn11.ctrlBase + 0x29,
							(val & 0xC0) | (ts->facetouch.palm_thresh & 0x3F));
	#else
		int offset = 0;

		offset += (F11_QUERY_HASGESTURE1(ts->map.fn11.query.data) != 0)? 6: 0;
		ts->facetouch.palm_thresh = thresh;
		shtps_rmi_write(ts,
						ts->map.fn11.ctrlBase + F11_QUERY_MAXELEC(ts->map.fn11.query.data) * 2 + 0x0C + offset,
						ts->facetouch.palm_thresh);

		_log_msg_sync( LOGMSG_ID__SET_PALM_THRESH, "0x%04X|0x%02X",
							ts->map.fn11.ctrlBase + F11_QUERY_MAXELEC(ts->map.fn11.query.data) * 2 + 0x0C + offset,
							ts->facetouch.palm_thresh);
	#endif /* #if defined( SHTPS_SY_REGMAP_BASE3 ) */
		SHTPS_LOG_DBG_PRINT("palm thresh = %d\n", ts->facetouch.palm_thresh);
	}
#endif /* #if defined( CONFIG_SHTPS_SY3000_FACETOUCH_DETECT ) || defined( CONFIG_SHTPS_SY3000_FACETOUCH_OFF_DETECT ) */
}

static void shtps_delayed_rezero(struct shtps_rmi_spi *ts, unsigned long delay_us)
{
	_log_msg_sync( LOGMSG_ID__DELAYED_REZERO_SET, "%lu", delay_us);
	hrtimer_cancel(&ts->rezero_delayed_timer);
	hrtimer_start(&ts->rezero_delayed_timer, ktime_set(0, delay_us * 1000), HRTIMER_MODE_REL);
}

static void shtps_delayed_rezero_cancel(struct shtps_rmi_spi *ts)
{
	_log_msg_sync( LOGMSG_ID__DELAYED_REZERO_CANCEL, "");
	hrtimer_try_to_cancel(&ts->rezero_delayed_timer);
}

static void shtps_rezero(struct shtps_rmi_spi *ts)
{
#if defined( SHTPS_AUTOREZERO_CONTROL_ENABLE )
	u8  val;

	_log_msg_sync( LOGMSG_ID__REZERO_EXEC, "");
	SHTPS_LOG_DBG_PRINT("fw rezero execute\n");
	shtps_rmi_read(ts, 0xF0, &val, 1);
	shtps_rmi_write(ts, 0xF0, val & ~0x01);
	shtps_rmi_write(ts, ts->map.fn11.commandBase, 0x01);
	shtps_rmi_write(ts, 0xF0, val | 0x01);
#else
	_log_msg_sync( LOGMSG_ID__REZERO_EXEC, "");
	SHTPS_LOG_DBG_PRINT("fw rezero execute\n");
	shtps_rmi_write(ts, ts->map.fn11.commandBase, 0x01);
#endif /* #if defined( SHTPS_AUTOREZERO_CONTROL_ENABLE ) */

#if defined( SHTPS_MODULE_PARAM_ENABLE )
	shtps_rezero_state = 1;
#endif /* SHTPS_MODULE_PARAM_ENABLE */
}

#if defined( SHTPS_AUTOREZERO_CONTROL_ENABLE )
static u8 shtps_is_singlefinger(struct shtps_rmi_spi *ts, u8 gs_info)
{
#if defined( SHTPS_AUTOREZERO_SINGLE_FINGER_ENABLE )
	u8  ret = 0x00;
	u8  buf;

	shtps_rmi_read(ts, 0xF0, &buf, 1);
	if(buf & 0x02){
		ret = 0x01;
	}
	_log_msg_sync( LOGMSG_ID__GET_SINGLE_FINGER_FLAG, "%d", ret);
	return ret;
#else
	return 0x01;
#endif /* #if defined( SHTPS_AUTOREZERO_SINGLE_FINGER_ENABLE ) */
}

static void shtps_autorezero_disable(struct shtps_rmi_spi *ts)
{
	u8  val;

	_log_msg_sync( LOGMSG_ID__AUTO_REZERO_DISABLE, "");
	SHTPS_LOG_DBG_PRINT("fw auto rezero disable\n");
	shtps_rmi_read(ts, 0xF0, &val, 1);
	shtps_rmi_write(ts, 0xF0, val | 0x01);
}

static void shtps_autorezero_enable(struct shtps_rmi_spi *ts)
{
	u8  val;

	_log_msg_sync( LOGMSG_ID__AUTO_REZERO_ENABLE, "");
	SHTPS_LOG_DBG_PRINT("fw auto rezero enable\n");
	shtps_rmi_read(ts, 0xF0, &val, 1);
	shtps_rmi_write(ts, 0xF0, val & ~0x01);
}
#endif /* #if defined( SHTPS_AUTOREZERO_CONTROL_ENABLE ) */

static void shtps_rezero_request(struct shtps_rmi_spi *ts, u8 request, u8 trigger)
{
#if defined( SHTPS_AUTOREZERO_CONTROL_ENABLE )
	int single_finger_count;

	_log_msg_sync( LOGMSG_ID__REZERO_REQUEST, "%d|%d", request, trigger);
	if(request & SHTPS_REZERO_REQUEST_WAKEUP_REZERO){
		shtps_delayed_rezero(ts, SHTPS_SLEEP_OUT_WAIT_US);
	}
	if(request & SHTPS_REZERO_REQUEST_REZERO){
		shtps_rezero(ts);
	}

	if(request & SHTPS_REZERO_REQUEST_AUTOREZERO_DISABLE){
		shtps_autorezero_disable(ts);
	}

	if(request & SHTPS_REZERO_REQUEST_AUTOREZERO_ENABLE){
		shtps_autorezero_enable(ts);
	}
#else
	_log_msg_sync( LOGMSG_ID__REZERO_REQUEST, "%d|%d", request, trigger);
	if(request & SHTPS_REZERO_REQUEST_WAKEUP_REZERO){
		shtps_delayed_rezero(ts, SHTPS_SLEEP_OUT_WAIT_US);
	}
	if(request & SHTPS_REZERO_REQUEST_REZERO){
		shtps_rezero(ts);
	}
#endif /* #if defined( SHTPS_AUTOREZERO_CONTROL_ENABLE ) */
}

static void shtps_rezero_handle(struct shtps_rmi_spi *ts, u8 event, u8 gs)
{
#if defined( SHTPS_AUTOREZERO_CONTROL_ENABLE )
	_log_msg_sync( LOGMSG_ID__REZERO_HANDLE, "%d|%d|%d", ts->poll_info.stop_margin, event, gs);
	if(!ts->poll_info.stop_margin){
		return;
	}

	if(event == SHTPS_REZERO_HANDLE_EVENT_MTD){
		ts->poll_info.single_fingers_enable = 0;

	}else if(event == SHTPS_REZERO_HANDLE_EVENT_TOUCH){
		if(ts->poll_info.single_fingers_enable && shtps_is_singlefinger(ts, gs)){
			ts->poll_info.single_fingers_count++;
			SHTPS_LOG_DBG_PRINT("single finger count = %d\n",
									ts->poll_info.single_fingers_count);
		}

	}else if(event == SHTPS_REZERO_HANDLE_EVENT_TOUCHUP){
		ts->poll_info.single_fingers_enable = 1;
		if((++ts->poll_info.stop_count >= ts->poll_info.stop_margin) &&
			(ts->poll_info.single_fingers_count >= ts->poll_info.single_fingers_max))
		{
			shtps_autorezero_disable(ts);

			ts->poll_info.stop_margin          = 0;
			ts->poll_info.single_fingers_count = 0;
			ts->poll_info.single_fingers_enable= 0;
			ts->poll_info.single_fingers_max   = 0;
		}
	}
#endif /* #if defined( SHTPS_AUTOREZERO_CONTROL_ENABLE ) */
}

static void shtps_reset_startuptime(struct shtps_rmi_spi *ts)
{
	ts->state_mgr.starttime = jiffies + msecs_to_jiffies(SHTPS_STARTUP_MIN_TIME);
}

static unsigned long shtps_check_startuptime(struct shtps_rmi_spi *ts)
{
	if(time_after(jiffies, ts->state_mgr.starttime)){
		return 0;
	}
	return (jiffies_to_msecs(ts->state_mgr.starttime - jiffies)) % SHTPS_STARTUP_MIN_TIME;
}

static int shtps_start(struct shtps_rmi_spi *ts)
{
	return request_event(ts, SHTPS_EVENT_START, 0);
}

static void shtps_shutdown(struct shtps_rmi_spi *ts)
{
	request_event(ts, SHTPS_EVENT_STOP, 0);
}

/* -----------------------------------------------------------------------------------
 */
#if defined ( CONFIG_SHTPS_SY3000_POSITION_OFFSET )
static int shtps_offset_area(struct shtps_rmi_spi *ts, int x, int y)
{
	if(y < ts->offset.base[2]){
		if(x < ts->offset.base[0]){
			return 0x00;
		}else if(x < ts->offset.base[1]){
			return 0x01;
		}else{
			return 0x02;
		}
	}else if(y < ts->offset.base[3]){
		if(x < ts->offset.base[0]){
			return 0x03;
		}else if(x < ts->offset.base[1]){
			return 0x04;
		}else{
			return 0x05;
		}
	}else if(y < ts->offset.base[4]){
		if(x < ts->offset.base[0]){
			return 0x06;
		}else if(x < ts->offset.base[1]){
			return 0x07;
		}else{
			return 0x08;
		}
	}else{
		if(x < ts->offset.base[0]){
			return 0x09;
		}else if(x < ts->offset.base[1]){
			return 0x0A;
		}else{
			return 0x0B;
		}
	}
	return 0x00;
}
#endif /* #if deifned( CONFIG_SHTPS_SY3000_POSITION_OFFSET ) */

static int shtps_offset_pos(struct shtps_rmi_spi *ts, int *x, int *y)
{
#if defined ( CONFIG_SHTPS_SY3000_POSITION_OFFSET )
	int area;
	int pq, rs;
	int xp, xq, xr, xs;
	int yp, yq, yr, ys;
	int base_xp, base_xq;
	int base_yp, base_yq;

	if(!ts->offset.enabled){
		return 0;
	}

	area = shtps_offset_area(ts, *x, *y);

	xp = xq = xr = xs = yp = yq = yr = ys = 0;
	if(area == 0x00){
		xq = xs = ts->offset.diff[0];
		yr = ys = ts->offset.diff[1];
		base_xp = 0;
		base_xq = ts->offset.base[0];
		base_yp = 0;
		base_yq = ts->offset.base[2];
	}else if(area == 0x01){
		xp = xr = ts->offset.diff[0];
		xq = xs = ts->offset.diff[2];
		yr = ts->offset.diff[1];
		ys = ts->offset.diff[3];
		base_xp = ts->offset.base[0];
		base_xq = ts->offset.base[1];
		base_yp = 0;
		base_yq = ts->offset.base[2];
	}else if(area == 0x02){
		xq = xr = ts->offset.diff[2];
		yr = ys = ts->offset.diff[3];
		base_xp = ts->offset.base[1];
		base_xq = CONFIG_SHTPS_SY3000_PANEL_SIZE_X;
		base_yp = 0;
		base_yq = ts->offset.base[2];
	}else if(area == 0x03){
		xq = ts->offset.diff[0];
		xs = ts->offset.diff[4];
		yp = yq = ts->offset.diff[1];
		yr = ys = ts->offset.diff[5];
		base_xp = 0;
		base_xq = ts->offset.base[0];
		base_yp = ts->offset.base[2];
		base_yq = ts->offset.base[3];
	}else if(area == 0x04){
		xp = ts->offset.diff[0];
		xq = ts->offset.diff[2];
		xr = ts->offset.diff[4];
		xs = ts->offset.diff[6];
		yp = ts->offset.diff[1];
		yq = ts->offset.diff[3];
		yr = ts->offset.diff[5];
		ys = ts->offset.diff[7];
		base_xp = ts->offset.base[0];
		base_xq = ts->offset.base[1];
		base_yp = ts->offset.base[2];
		base_yq = ts->offset.base[3];
	}else if(area == 0x05){
		xp = ts->offset.diff[2];
		xr = ts->offset.diff[6];
		yp = yq = ts->offset.diff[3];
		yr = ys = ts->offset.diff[7];
		base_xp = ts->offset.base[1];
		base_xq = CONFIG_SHTPS_SY3000_PANEL_SIZE_X;
		base_yp = ts->offset.base[2];
		base_yq = ts->offset.base[3];
	}else if(area == 0x06){
		xq = ts->offset.diff[4];
		xs = ts->offset.diff[8];
		yp = yq = ts->offset.diff[5];
		yr = ys = ts->offset.diff[9];
		base_xp = 0;
		base_xq = ts->offset.base[0];
		base_yp = ts->offset.base[3];
		base_yq = ts->offset.base[4];
	}else if(area == 0x07){
		xp = ts->offset.diff[4];
		xq = ts->offset.diff[6];
		xr = ts->offset.diff[8];
		xs = ts->offset.diff[10];
		yp = ts->offset.diff[5];
		yq = ts->offset.diff[7];
		yr = ts->offset.diff[9];
		ys = ts->offset.diff[11];
		base_xp = ts->offset.base[0];
		base_xq = ts->offset.base[1];
		base_yp = ts->offset.base[3];
		base_yq = ts->offset.base[4];
	}else if(area == 0x08){
		xp = ts->offset.diff[6];
		xr = ts->offset.diff[10];
		yp = yq = ts->offset.diff[7];
		yr = ys = ts->offset.diff[11];
		base_xp = ts->offset.base[1];
		base_xq = CONFIG_SHTPS_SY3000_PANEL_SIZE_X;
		base_yp = ts->offset.base[3];
		base_yq = ts->offset.base[4];
	}else if(area == 0x09){
		xq = xs = ts->offset.diff[8];
		yp = yq = ts->offset.diff[9];
		base_xp = 0;
		base_xq = ts->offset.base[0];
		base_yp = ts->offset.base[4];
		base_yq = CONFIG_SHTPS_SY3000_PANEL_SIZE_Y;
	}else if(area == 0x0A){
		xp = xr = ts->offset.diff[8];
		xq = xs = ts->offset.diff[10];
		yp = ts->offset.diff[9];
		yq = ts->offset.diff[11];
		base_xp = ts->offset.base[0];
		base_xq = ts->offset.base[1];
		base_yp = ts->offset.base[4];
		base_yq = CONFIG_SHTPS_SY3000_PANEL_SIZE_Y;
	}else{
		xq = xr = ts->offset.diff[10];
		yp = yq = ts->offset.diff[11];
		base_xp = ts->offset.base[1];
		base_xq = CONFIG_SHTPS_SY3000_PANEL_SIZE_X;
		base_yp = ts->offset.base[4];
		base_yq = CONFIG_SHTPS_SY3000_PANEL_SIZE_Y;
	}

	pq = (xq - xp) * (*x - base_xp) / (base_xq - base_xp) + xp;
	rs = (xs - xr) * (*x - base_xp) / (base_xq - base_xp) + xr;
	*x -= ((rs - pq) * (*y - base_yp) / (base_yq - base_yp) + pq);

	pq = (yq - yp) * (*x - base_xp) / (base_xq - base_xp) + yp;
	rs = (ys - yr) * (*x - base_xp) / (base_xq - base_xp) + yr;
	*y -= ((rs - pq) * (*y - base_yp) / (base_yq - base_yp) + pq);
#endif /* #if deifned( CONFIG_SHTPS_SY3000_POSITION_OFFSET ) */
	return 0;
}

static int shtps_map_construct(struct shtps_rmi_spi *ts)
{
	struct rmi_pdt	pdt;
	int				i;
	int				rc;
	int				page;
	u8				maxPosition[4];
	char			productID[11];

	_log_msg_sync( LOGMSG_ID__MAP_CONSTRUCT, "");
	SHTPS_LOG_FUNC_CALL();

	msleep(3);
	memset(&ts->map, 0, sizeof(ts->map));

	/* Read the PDT */
	for(page = 0;page < 3;page++){
		shtps_rmi_write(ts, 0xFF, page);
		for(i = 0xE9;i > 0x0a;i-=sizeof(pdt)){
			rc = shtps_rmi_read(ts, ((page & 0x0f) << 8) | i, (u8*)&pdt, sizeof(pdt));
			if(rc){
				goto err_exit;
			}

			if(!pdt.functionNumber){
				/** End of PDT */
				break;
			}

			switch(pdt.functionNumber){
			case 0x01:
				SHTPS_LOG_DBG_PRINT("Found: RMI Device Control\n");
				ts->map.fn01.enable		= 1;
				ts->map.fn01.queryBase  = ((page & 0x0f) << 8) | pdt.queryBaseAddr;
				ts->map.fn01.ctrlBase   = ((page & 0x0f) << 8) | pdt.controlBaseAddr;
				ts->map.fn01.dataBase   = ((page & 0x0f) << 8) | pdt.dataBaseAddr;
				ts->map.fn01.commandBase= ((page & 0x0f) << 8) | pdt.commandBaseAddr;

				rc = shtps_rmi_read(ts, ts->map.fn01.queryBase,
									ts->map.fn01.query.data, sizeof(ts->map.fn01.query.data));

#if defined( SHTPS_SY_REGMAP_BASE3 )
				memcpy(productID, &F01_QUERY_PRODUCTID(ts->map.fn01.query.data), sizeof(productID));
				productID[10] = '\0';
				SHTPS_LOG_DBG_PRINT("F01 Query Data\n");
				SHTPS_LOG_DBG_PRINT("-------------------------------------------------\n");
				SHTPS_LOG_DBG_PRINT("Manufacturer ID       : 0x%02x\n", F01_QUERY_MANUFACTURERID(ts->map.fn01.query.data));
				SHTPS_LOG_DBG_PRINT("CutomMap              : 0x%02x\n", F01_QUERY_CUSTOMMAP(ts->map.fn01.query.data));
				SHTPS_LOG_DBG_PRINT("NonCompliant          : 0x%02x\n", F01_QUERY_NONCOMPLIANT(ts->map.fn01.query.data));
				SHTPS_LOG_DBG_PRINT("HasSensorID           : 0x%02x\n", F01_QUERY_HASSENSORID(ts->map.fn01.query.data));
				SHTPS_LOG_DBG_PRINT("HasAdjustableDoze     : 0x%02x\n", F01_QUERY_HASAJUSTABLEDOZE(ts->map.fn01.query.data));
				SHTPS_LOG_DBG_PRINT("HasAdjDozeHoldoff     : 0x%02x\n", F01_QUERY_HASADJDOZEHOLDOFF(ts->map.fn01.query.data));
				SHTPS_LOG_DBG_PRINT("HasProductProperties2 : 0x%02x\n", F01_QUERY_HASPRODUCTPROPERTIES2(ts->map.fn01.query.data));
				SHTPS_LOG_DBG_PRINT("productInfo0          : 0x%02x\n", F01_QUERY_PRODUCTINFO0(ts->map.fn01.query.data));
				SHTPS_LOG_DBG_PRINT("productInfo1          : 0x%02x\n", F01_QUERY_PRODUCTINFO1(ts->map.fn01.query.data));
				SHTPS_LOG_DBG_PRINT("Year                  : 0x%02x\n", F01_QUERY_DATECODEYEAR(ts->map.fn01.query.data));
				SHTPS_LOG_DBG_PRINT("Month                 : 0x%02x\n", F01_QUERY_DATECODEMONTH(ts->map.fn01.query.data));
				SHTPS_LOG_DBG_PRINT("Day                   : 0x%02x\n", F01_QUERY_DATECODEDAY(ts->map.fn01.query.data));
				SHTPS_LOG_DBG_PRINT("CP1                   : 0x%02x\n", F01_QUERY_CP1(ts->map.fn01.query.data));
				SHTPS_LOG_DBG_PRINT("CP2                   : 0x%02x\n", F01_QUERY_CP2(ts->map.fn01.query.data));
				SHTPS_LOG_DBG_PRINT("WaferLotID0           : 0x%04x\n", F01_QUERY_WAFERLOTID0(ts->map.fn01.query.data));
				SHTPS_LOG_DBG_PRINT("WaferLotID1           : 0x%04x\n", F01_QUERY_WAFERLOTID1(ts->map.fn01.query.data));
				SHTPS_LOG_DBG_PRINT("WaferLotID2           : 0x%02x\n", F01_QUERY_WAFERLOTID2(ts->map.fn01.query.data));
				SHTPS_LOG_DBG_PRINT("ProductID             : %s\n", productID);
				SHTPS_LOG_DBG_PRINT("HasDS4Queries         : 0x%02x\n", F01_QUERY_HASDS4QUERIES(ts->map.fn01.query.data));
				SHTPS_LOG_DBG_PRINT("DS4 Length            : 0x%02x\n", F01_QUERY_DS4_LENGTH(ts->map.fn01.query.data));
				SHTPS_LOG_DBG_PRINT("DS4 HasPackageQuery   : 0x%02x\n", F01_QUERY_DS4_HASPACKAGEIDQUERY(ts->map.fn01.query.data));
				SHTPS_LOG_DBG_PRINT("DS4 HasPackratQuery   : 0x%02x\n", F01_QUERY_DS4_HASPACKRATQUERY(ts->map.fn01.query.data));
				SHTPS_LOG_DBG_PRINT("DS4 HasResetQuery     : 0x%02x\n", F01_QUERY_DS4_HASRESETQUERY(ts->map.fn01.query.data));
				SHTPS_LOG_DBG_PRINT("DS4 HasMaskRevQuery   : 0x%02x\n", F01_QUERY_DS4_HASMASKREVQUERY(ts->map.fn01.query.data));
				SHTPS_LOG_DBG_PRINT("DS4 HasI2CControl     : 0x%02x\n", F01_QUERY_DS4_HASI2CCONTROL(ts->map.fn01.query.data));
				SHTPS_LOG_DBG_PRINT("DS4 HasSPIControl     : 0x%02x\n", F01_QUERY_DS4_HASSPICONTROL(ts->map.fn01.query.data));
				SHTPS_LOG_DBG_PRINT("DS4 HasATTNControl    : 0x%02x\n", F01_QUERY_DS4_HASATTNCONTROL(ts->map.fn01.query.data));
				SHTPS_LOG_DBG_PRINT("RESET Enabled         : 0x%02x\n", F01_QUERY_RESET_ENABLED(ts->map.fn01.query.data));
				SHTPS_LOG_DBG_PRINT("RESET Polarity        : 0x%02x\n", F01_QUERY_RESET_POLARITY(ts->map.fn01.query.data));
				SHTPS_LOG_DBG_PRINT("RESET Pull-upEnabled  : 0x%02x\n", F01_QUERY_RESET_PULLUPENABLED(ts->map.fn01.query.data));
#else
				memcpy(productID, &F01_QUERY_PRODUCTID(ts->map.fn01.query.data), sizeof(productID));
				productID[10] = '\0';
				SHTPS_LOG_DBG_PRINT("F01 Query Data\n");
				SHTPS_LOG_DBG_PRINT("-------------------------------------------------\n");
				SHTPS_LOG_DBG_PRINT("Manufacturer ID : 0x%02x\n", F01_QUERY_MANUFACTURERID(ts->map.fn01.query.data));
				SHTPS_LOG_DBG_PRINT("NonCompliant    : 0x%02x\n", F01_QUERY_NONCOMPLIANT(ts->map.fn01.query.data));
				SHTPS_LOG_DBG_PRINT("cutomMap        : 0x%02x\n", F01_QUERY_CUSTOMMAP(ts->map.fn01.query.data));
				SHTPS_LOG_DBG_PRINT("productInfo0    : 0x%02x\n", F01_QUERY_PRODUCTINFO0(ts->map.fn01.query.data));
				SHTPS_LOG_DBG_PRINT("productInfo1    : 0x%02x\n", F01_QUERY_PRODUCTINFO1(ts->map.fn01.query.data));
				SHTPS_LOG_DBG_PRINT("dataCodeYear    : 0x%02x\n", F01_QUERY_DATACODEYEAR(ts->map.fn01.query.data));
				SHTPS_LOG_DBG_PRINT("dataCodeMonth   : 0x%02x\n", F01_QUERY_DATACODEMONTH(ts->map.fn01.query.data));
				SHTPS_LOG_DBG_PRINT("dataCodeDay     : 0x%02x\n", F01_QUERY_DATACODEDAY(ts->map.fn01.query.data));
				SHTPS_LOG_DBG_PRINT("testID_h        : 0x%02x\n", F01_QUERY_TESTID_HI(ts->map.fn01.query.data));
				SHTPS_LOG_DBG_PRINT("testID_l        : 0x%02x\n", F01_QUERY_TESTID_LO(ts->map.fn01.query.data));
				SHTPS_LOG_DBG_PRINT("serialNum_h     : 0x%02x\n", F01_QUERY_SERIALNUM_HI(ts->map.fn01.query.data));
				SHTPS_LOG_DBG_PRINT("serialNum_l     : 0x%02x\n", F01_QUERY_SERIALNUM_LO(ts->map.fn01.query.data));
				SHTPS_LOG_DBG_PRINT("Product ID      : %s\n"    , productID);
#endif /* #if defined( SHTPS_SY_REGMAP_BASE3 ) */

				break;

			case 0x05:
				SHTPS_LOG_DBG_PRINT("Found: Image Reporting\n");
				ts->map.fn05.enable		= 1;
				ts->map.fn05.queryBase  = ((page & 0x0f) << 8) | pdt.queryBaseAddr;
				ts->map.fn05.ctrlBase   = ((page & 0x0f) << 8) | pdt.controlBaseAddr;
				ts->map.fn05.dataBase   = ((page & 0x0f) << 8) | pdt.dataBaseAddr;
				ts->map.fn05.commandBase= ((page & 0x0f) << 8) | pdt.commandBaseAddr;

				rc = shtps_rmi_read(ts, ts->map.fn05.queryBase,
									ts->map.fn05.query.data, sizeof(ts->map.fn05.query.data));

				SHTPS_LOG_DBG_PRINT("F05 Query Data\n");
				SHTPS_LOG_DBG_PRINT("-------------------------------------------------\n");
				SHTPS_LOG_DBG_PRINT("NumberOfReceiverElectrodes    : 0x%02x\n", F05_QUERY_NUMOFRCVEL(ts->map.fn05.query.data));
				SHTPS_LOG_DBG_PRINT("NumberOfTransmitterElectrodes : 0x%02x\n", F05_QUERY_NUMOFTRANSEL(ts->map.fn05.query.data));
				SHTPS_LOG_DBG_PRINT("Has15bitDelta                 : 0x%02x\n", F05_QUERY_HAS16BIT(ts->map.fn05.query.data));
				SHTPS_LOG_DBG_PRINT("SizeOfF05ImageWindow          : 0x%02x\n", F05_QUERY_IMAGEWINDOWSIZE(ts->map.fn05.query.data));
				break;

			case 0x11:
				SHTPS_LOG_DBG_PRINT("Found: 2-D Sensors\n");
				ts->map.fn11.enable		= 1;
				ts->map.fn11.queryBase  = ((page & 0x0f) << 8) | pdt.queryBaseAddr;
				ts->map.fn11.ctrlBase   = ((page & 0x0f) << 8) | pdt.controlBaseAddr;
				ts->map.fn11.dataBase   = ((page & 0x0f) << 8) | pdt.dataBaseAddr;
				ts->map.fn11.commandBase= ((page & 0x0f) << 8) | pdt.commandBaseAddr;

				rc = shtps_rmi_read(ts, ts->map.fn11.queryBase,
									ts->map.fn11.query.data, sizeof(ts->map.fn11.query.data));
				if(rc){
					goto err_exit;
				}
				rc = shtps_rmi_read(ts, ts->map.fn11.ctrlBase + 0x06, maxPosition, 4);
				if(rc){
					goto err_exit;
				}
				ts->map.fn11.ctrl.maxXPosition = maxPosition[0] | ((maxPosition[1] & 0x0F) << 0x08);
				ts->map.fn11.ctrl.maxYPosition = maxPosition[2] | ((maxPosition[3] & 0x0F) << 0x08);
#if defined( SHTPS_SY_REGMAP_BASE3 )
				SHTPS_LOG_DBG_PRINT("F11 Query Data\n");
				SHTPS_LOG_DBG_PRINT("-------------------------------------------------\n");
				SHTPS_LOG_DBG_PRINT("NumberOf2DSensors              : 0x%02x\n", F11_QUERY_NUMOFSENSORS(ts->map.fn11.query.data));
				SHTPS_LOG_DBG_PRINT("HasQuery9                      : 0x%02x\n", F11_QUERY_HASQUERY9(ts->map.fn11.query.data));
				SHTPS_LOG_DBG_PRINT("HasQuery11                     : 0x%02x\n", F11_QUERY_HASQUERY11(ts->map.fn11.query.data));
				SHTPS_LOG_DBG_PRINT("HasQuery12                     : 0x%02x\n", F11_QUERY_HASQUERY12(ts->map.fn11.query.data));
				SHTPS_LOG_DBG_PRINT("NumberOfFingers                : 0x%02x\n", F11_QUERY_NUMOFFINGERS(ts->map.fn11.query.data));
				SHTPS_LOG_DBG_PRINT("HasRelMode                     : 0x%02x\n", F11_QUERY_HASREL(ts->map.fn11.query.data));
				SHTPS_LOG_DBG_PRINT("HasAbsMode                     : 0x%02x\n", F11_QUERY_HASABS(ts->map.fn11.query.data));
				SHTPS_LOG_DBG_PRINT("HasGestures                    : 0x%02x\n", F11_QUERY_HASGESTURES(ts->map.fn11.query.data));
				SHTPS_LOG_DBG_PRINT("HasSensitivityAdjust           : 0x%02x\n", F11_QUERY_HASSENSADJUST(ts->map.fn11.query.data));
				SHTPS_LOG_DBG_PRINT("Configurable                   : 0x%02x\n", F11_QUERY_CONFIGURABLE(ts->map.fn11.query.data));
				SHTPS_LOG_DBG_PRINT("NumberOfXElectrodes            : 0x%02x\n", F11_QUERY_NUMOFXELEC(ts->map.fn11.query.data));
				SHTPS_LOG_DBG_PRINT("NumberOfYElectrodes            : 0x%02x\n", F11_QUERY_NUMOFYELEC(ts->map.fn11.query.data));
				SHTPS_LOG_DBG_PRINT("MaximumElectrodes              : 0x%02x\n", F11_QUERY_MAXELEC(ts->map.fn11.query.data));
				SHTPS_LOG_DBG_PRINT("AbsoluteDataSize               : 0x%02x\n", F11_QUERY_ABSOLUTEDATASIZE(ts->map.fn11.query.data));
				SHTPS_LOG_DBG_PRINT("AnchoredFinger                 : 0x%02x\n", F11_QUERY_ANCHOREDFINGER(ts->map.fn11.query.data));
				SHTPS_LOG_DBG_PRINT("HasDribble                     : 0x%02x\n", F11_QUERY_HASDRIBBLE(ts->map.fn11.query.data));
				SHTPS_LOG_DBG_PRINT("HasBendingCorrection           : 0x%02x\n", F11_QUERY_HASBENDINGCORRECTION(ts->map.fn11.query.data));
				SHTPS_LOG_DBG_PRINT("HasLargeObjectSuppression      : 0x%02x\n", F11_QUERY_HASLARGEOBJECTSUPPRESSION(ts->map.fn11.query.data));
				SHTPS_LOG_DBG_PRINT("HasJitterFilter                : 0x%02x\n", F11_QUERY_HASJITTERFILTER(ts->map.fn11.query.data));
				SHTPS_LOG_DBG_PRINT("HasPen                         : 0x%02x\n", F11_QUERY_HASPEN(ts->map.fn11.query.data));
				SHTPS_LOG_DBG_PRINT("HasFingerProximity             : 0x%02x\n", F11_QUERY_HASFINGERPROXIMITY(ts->map.fn11.query.data));
				SHTPS_LOG_DBG_PRINT("HasLargeObjectSensitivity      : 0x%02x\n", F11_QUERY_HASLARGEOBJECTSENSITIVITY(ts->map.fn11.query.data));
				SHTPS_LOG_DBG_PRINT("HasTwoPenThresholds            : 0x%02x\n", F11_QUERY_HASTWOPENTHRESHOLDS(ts->map.fn11.query.data));
				SHTPS_LOG_DBG_PRINT("HasPenHoverDiscrimination      : 0x%02x\n", F11_QUERY_HASPENHOVERDISCRIMINATION(ts->map.fn11.query.data));
				SHTPS_LOG_DBG_PRINT("HasNewPenRegisters             : 0x%02x\n", F11_QUERY_HASNEWPENREGISTERS(ts->map.fn11.query.data));
				SHTPS_LOG_DBG_PRINT("HasZTuning                     : 0x%02x\n", F11_QUERY_HASZTUNING(ts->map.fn11.query.data));
				SHTPS_LOG_DBG_PRINT("HasPositionInterpolationTuning : 0x%02x\n", F11_QUERY_HASPOSITIONIPTUNING(ts->map.fn11.query.data));
				SHTPS_LOG_DBG_PRINT("HasWTuning                     : 0x%02x\n", F11_QUERY_HASWTUNING(ts->map.fn11.query.data));
				SHTPS_LOG_DBG_PRINT("HasPitchInfo                   : 0x%02x\n", F11_QUERY_HASPITCHINFO(ts->map.fn11.query.data));
				SHTPS_LOG_DBG_PRINT("HasDefaultFingerWidth          : 0x%02x\n", F11_QUERY_HASDEFAULTFINGERWIDTH(ts->map.fn11.query.data));
				SHTPS_LOG_DBG_PRINT("HasSegmentationAggressiveness  : 0x%02x\n", F11_QUERY_HASSEGAGGRESSIVENESS(ts->map.fn11.query.data));
				SHTPS_LOG_DBG_PRINT("HasTxRxClip                    : 0x%02x\n", F11_QUERY_HASTXRXCLIP(ts->map.fn11.query.data));
				SHTPS_LOG_DBG_PRINT("HasDrummingCorrection          : 0x%02x\n", F11_QUERY_HASDRUMMINGCORRECTION(ts->map.fn11.query.data));
				SHTPS_LOG_DBG_PRINT("Has8bitW                       : 0x%02x\n", F11_QUERY_HAS8BITW(ts->map.fn11.query.data));
				SHTPS_LOG_DBG_PRINT("Has2DAjustableMapping          : 0x%02x\n", F11_QUERY_HAS2DAJSTMAPPING(ts->map.fn11.query.data));
#else
				SHTPS_LOG_DBG_PRINT("F11 Query Data\n");
				SHTPS_LOG_DBG_PRINT("-------------------------------------------------\n");
				SHTPS_LOG_DBG_PRINT("numOfSensors    : 0x%02x\n", F11_QUERY_NUMOFSENSORS(ts->map.fn11.query.data));
				SHTPS_LOG_DBG_PRINT("configurable    : 0x%02x\n", F11_QUERY_CONFIGURABLE(ts->map.fn11.query.data));
				SHTPS_LOG_DBG_PRINT("hasSensAdjust   : 0x%02x\n", F11_QUERY_HASSENSADJUST(ts->map.fn11.query.data));
				SHTPS_LOG_DBG_PRINT("hasGestures     : 0x%02x\n", F11_QUERY_HASGESTURES(ts->map.fn11.query.data));
				SHTPS_LOG_DBG_PRINT("hasAbs          : 0x%02x\n", F11_QUERY_HASABS(ts->map.fn11.query.data));
				SHTPS_LOG_DBG_PRINT("hasRel          : 0x%02x\n", F11_QUERY_HASREL(ts->map.fn11.query.data));
				SHTPS_LOG_DBG_PRINT("numOfFingers    : 0x%02x\n", F11_QUERY_NUMOFFINGERS(ts->map.fn11.query.data));
				SHTPS_LOG_DBG_PRINT("numOfXElec      : 0x%02x\n", F11_QUERY_NUMOFXELEC(ts->map.fn11.query.data));
				SHTPS_LOG_DBG_PRINT("numOfYElec      : 0x%02x\n", F11_QUERY_NUMOFYELEC(ts->map.fn11.query.data));
				SHTPS_LOG_DBG_PRINT("maxElec         : 0x%02x\n", F11_QUERY_MAXELEC(ts->map.fn11.query.data));
				SHTPS_LOG_DBG_PRINT("hasAnchored     : 0x%02x\n", F11_QUERY_HASANCHORED(ts->map.fn11.query.data));
				SHTPS_LOG_DBG_PRINT("absDataSize     : 0x%02x\n", F11_QUERY_ABSDATASIZE(ts->map.fn11.query.data));
				SHTPS_LOG_DBG_PRINT("hasPinch        : 0x%02x\n", F11_QUERY_HASPINCH(ts->map.fn11.query.data));
				SHTPS_LOG_DBG_PRINT("hasPress        : 0x%02x\n", F11_QUERY_HASPRESS(ts->map.fn11.query.data));
				SHTPS_LOG_DBG_PRINT("hasFlick        : 0x%02x\n", F11_QUERY_HASFLICK(ts->map.fn11.query.data));
				SHTPS_LOG_DBG_PRINT("hasEarlyTap     : 0x%02x\n", F11_QUERY_HASEARLYTAP(ts->map.fn11.query.data));
				SHTPS_LOG_DBG_PRINT("hasDoubleTap    : 0x%02x\n", F11_QUERY_HASDOUBLETAP(ts->map.fn11.query.data));
				SHTPS_LOG_DBG_PRINT("hasTapHost      : 0x%02x\n", F11_QUERY_HASTAPHOST(ts->map.fn11.query.data));
				SHTPS_LOG_DBG_PRINT("hasSingleTap    : 0x%02x\n", F11_QUERY_HASSINGLETAP(ts->map.fn11.query.data));
				SHTPS_LOG_DBG_PRINT("hasRotate       : 0x%02x\n", F11_QUERY_HASROTATE(ts->map.fn11.query.data));
				SHTPS_LOG_DBG_PRINT("hasPalmDet      : 0x%02x\n", F11_QUERY_HASPALMDET(ts->map.fn11.query.data));
				SHTPS_LOG_DBG_PRINT("2D MAX X POS    : 0x%04x\n", ts->map.fn11.ctrl.maxXPosition);
				SHTPS_LOG_DBG_PRINT("2D MAX Y POS    : 0x%04x\n", ts->map.fn11.ctrl.maxYPosition);
#endif /* #if defined( SHTPS_SY_REGMAP_BASE3 ) */
				break;

			case 0x34:
				SHTPS_LOG_DBG_PRINT("Found: Flash memory management\n");
				ts->map.fn34.enable		= 1;
				ts->map.fn34.queryBase = ((page & 0x0f) << 8) | pdt.queryBaseAddr;
				ts->map.fn34.dataBase  = ((page & 0x0f) << 8) | pdt.dataBaseAddr;
				ts->map.fn34.ctrlBase   = ((page & 0x0f) << 8) | pdt.controlBaseAddr;

				rc = shtps_rmi_read(ts, ts->map.fn34.queryBase,
									ts->map.fn34.query.data, sizeof(ts->map.fn34.query.data));
				if(rc){
					goto err_exit;
				}
				SHTPS_LOG_DBG_PRINT("F34 Query Data\n");
				SHTPS_LOG_DBG_PRINT("-------------------------------------------------\n");
				SHTPS_LOG_DBG_PRINT("bootLoaderID0       : 0x%02x\n", F34_QUERY_BOOTLOADERID0(ts->map.fn34.query.data));
				SHTPS_LOG_DBG_PRINT("bootLoaderID1       : 0x%02x\n", F34_QUERY_BOOTLOADERID1(ts->map.fn34.query.data));
				SHTPS_LOG_DBG_PRINT("unlocked            : 0x%02x\n", F34_QUERY_UNLOCKED(ts->map.fn34.query.data));
				SHTPS_LOG_DBG_PRINT("blockSize           : 0x%04x\n", F34_QUERY_BLOCKSIZE(ts->map.fn34.query.data));
				SHTPS_LOG_DBG_PRINT("firmBlockCount      : 0x%04x\n", F34_QUERY_FIRMBLOCKCOUNT(ts->map.fn34.query.data));
				SHTPS_LOG_DBG_PRINT("configBlockCount    : 0x%04x\n", F34_QUERY_CONFIGBLOCKCOUNT(ts->map.fn34.query.data));
				break;

			case 0x54:
				SHTPS_LOG_DBG_PRINT("Found: Specification Addendum\n");
				ts->map.fn54.enable		= 1;
				ts->map.fn54.queryBase  = ((page & 0x0f) << 8) | pdt.queryBaseAddr;
				ts->map.fn54.ctrlBase   = ((page & 0x0f) << 8) | pdt.controlBaseAddr;
				ts->map.fn54.dataBase   = ((page & 0x0f) << 8) | pdt.dataBaseAddr;
				ts->map.fn54.commandBase= ((page & 0x0f) << 8) | pdt.commandBaseAddr;

				rc = shtps_rmi_read(ts, ts->map.fn54.queryBase,
									ts->map.fn54.query.data, sizeof(ts->map.fn54.query.data));
				if(rc){
					goto err_exit;
				}

#if defined( SHTPS_SY_REGMAP_BASE3 )
				SHTPS_LOG_DBG_PRINT("F54 Query Data\n");
				SHTPS_LOG_DBG_PRINT("-------------------------------------------------\n");
				SHTPS_LOG_DBG_PRINT("NumberOfReceiverElectrodes    : 0x%02x\n", F54_QUERY_NUMOFRCVEL(ts->map.fn54.query.data));
				SHTPS_LOG_DBG_PRINT("NumberOfTransmitterElectrodes : 0x%02x\n", F54_QUERY_NUMOFTRANSEL(ts->map.fn54.query.data));
				SHTPS_LOG_DBG_PRINT("HasBaseLine                   : 0x%02x\n", F54_QUERY_HASBASELINE(ts->map.fn54.query.data));
				SHTPS_LOG_DBG_PRINT("HasImage8                     : 0x%02x\n", F54_QUERY_HAS8BIT(ts->map.fn54.query.data));
				SHTPS_LOG_DBG_PRINT("HasImage16                    : 0x%02x\n", F54_QUERY_HAS16BIT(ts->map.fn54.query.data));
				SHTPS_LOG_DBG_PRINT("ClockRate                     : 0x%02x\n", F54_QUERY_CLOCKRATE(ts->map.fn54.query.data));
				SHTPS_LOG_DBG_PRINT("AnalogHardwareFamily          : 0x%02x\n", F54_QUERY_ANALOGHARDWAREFAMILY(ts->map.fn54.query.data));
				SHTPS_LOG_DBG_PRINT("HasPixelTouchThresholdTuning  : 0x%02x\n", F54_QUERY_HASPIXELTOUCHTHRESHOLDTUNING(ts->map.fn54.query.data));
				SHTPS_LOG_DBG_PRINT("HasArbitrarySensorAssignment  : 0x%02x\n", F54_QUERY_HASARBITRARYSENSORASSIGNMENT(ts->map.fn54.query.data));
				SHTPS_LOG_DBG_PRINT("HasInterferenceMetric         : 0x%02x\n", F54_QUERY_HASINTERFERENCEMETRIC(ts->map.fn54.query.data));
				SHTPS_LOG_DBG_PRINT("HasSenseFrequencyControl      : 0x%02x\n", F54_QUERY_HASSENSEFREQCONTROL(ts->map.fn54.query.data));
				SHTPS_LOG_DBG_PRINT("HasFirmwareNoiseMitigation    : 0x%02x\n", F54_QUERY_HASFWNOISEMITIGATION(ts->map.fn54.query.data));
				SHTPS_LOG_DBG_PRINT("HasLowPowerCtrl               : 0x%02x\n", F54_QUERY_HASLOWPOERCTRL(ts->map.fn54.query.data));
				SHTPS_LOG_DBG_PRINT("HasTwoByteReportRateReporting : 0x%02x\n", F54_QUERY_HASTWOBYTEREPORTRATEREPORTING(ts->map.fn54.query.data));
				SHTPS_LOG_DBG_PRINT("HasOneByteReportRateReporting : 0x%02x\n", F54_QUERY_HASONEBYTEREPORTRATEREPORTING(ts->map.fn54.query.data));
				SHTPS_LOG_DBG_PRINT("HasRelaxationCtrl             : 0x%02x\n", F54_QUERY_HASRELAXATIONCTRL(ts->map.fn54.query.data));
				SHTPS_LOG_DBG_PRINT("AxisCompensationMode          : 0x%02x\n", F54_QUERY_AXISCOMPENSATIONMODE(ts->map.fn54.query.data));
				SHTPS_LOG_DBG_PRINT("HasIIRFilter                  : 0x%02x\n", F54_QUERY_HASIIRFILTER(ts->map.fn54.query.data));
				SHTPS_LOG_DBG_PRINT("HasCMNRemoval                 : 0x%02x\n", F54_QUERY_HASCMNREMOVAL(ts->map.fn54.query.data));
				SHTPS_LOG_DBG_PRINT("HasCMNCapScaleFactor          : 0x%02x\n", F54_QUERY_HASCMNCAPSCALEFACTOR(ts->map.fn54.query.data));
				SHTPS_LOG_DBG_PRINT("HasPixelThresholdHysteresis   : 0x%02x\n", F54_QUERY_HASPIXCELTHRESHHYSTERESIS(ts->map.fn54.query.data));
				SHTPS_LOG_DBG_PRINT("HasEdgeCompensation           : 0x%02x\n", F54_QUERY_HASEDGECOMPENSATION(ts->map.fn54.query.data));
				SHTPS_LOG_DBG_PRINT("HasPerFrequencyNoiseControl   : 0x%02x\n", F54_QUERY_HASPERFREQNOISECTRL(ts->map.fn54.query.data));
				SHTPS_LOG_DBG_PRINT("HasForceFastRelaxation        : 0x%02x\n", F54_QUERY_HASFORCEFASTRELAXATION(ts->map.fn54.query.data));
				SHTPS_LOG_DBG_PRINT("HasMMStateMitigation          : 0x%02x\n", F54_QUERY_HASMMSTATEMITIGATION(ts->map.fn54.query.data));
				SHTPS_LOG_DBG_PRINT("HasCDM4                       : 0x%02x\n", F54_QUERY_HASCDM4(ts->map.fn54.query.data));
				SHTPS_LOG_DBG_PRINT("HasVarianceMetric             : 0x%02x\n", F54_QUERY_HASVARIANCEMETRIC(ts->map.fn54.query.data));
				SHTPS_LOG_DBG_PRINT("Has0DRelaxation               : 0x%02x\n", F54_QUERY_HAS0DRELAXATIONCTRL(ts->map.fn54.query.data));
				SHTPS_LOG_DBG_PRINT("Has0DAcquisitionCtrl          : 0x%02x\n", F54_QUERY_HAS0DACQUISITIONCTRL(ts->map.fn54.query.data));
				SHTPS_LOG_DBG_PRINT("MaxNumOfSensingFrequencies    : 0x%02x\n", F54_QUERY_MAXNUMOFSENSINGFREQ(ts->map.fn54.query.data));
				SHTPS_LOG_DBG_PRINT("BurstsPerCluster              : 0x%02x\n", F54_QUERY_BURSTSPERCLUSTER(ts->map.fn54.query.data));
#else
				SHTPS_LOG_DBG_PRINT("F54 Query Data\n");
				SHTPS_LOG_DBG_PRINT("-------------------------------------------------\n");
				SHTPS_LOG_DBG_PRINT("NumberOfReceiverElectrodes    : 0x%02x\n", F54_QUERY_NUMOFRCVEL(ts->map.fn54.query.data));
				SHTPS_LOG_DBG_PRINT("NumberOfTransmitterElectrodes : 0x%02x\n", F54_QUERY_NUMOFTRANSEL(ts->map.fn54.query.data));
				SHTPS_LOG_DBG_PRINT("Has16bitDelta                 : 0x%02x\n", F54_QUERY_HAS16BIT(ts->map.fn54.query.data));
				SHTPS_LOG_DBG_PRINT("Has8bitDelta                  : 0x%02x\n", F54_QUERY_HAS8BIT(ts->map.fn54.query.data));
#endif /* #if defined( SHTPS_SY_REGMAP_BASE3 ) */
				break;

			case 0x19:
				SHTPS_LOG_DBG_PRINT("Found: 0-D Capacitivve Buttons\n");
				ts->map.fn19.enable		= 1;
				ts->map.fn19.queryBase  = ((page & 0x0f) << 8) | pdt.queryBaseAddr;
				ts->map.fn19.ctrlBase   = ((page & 0x0f) << 8) | pdt.controlBaseAddr;
				ts->map.fn19.dataBase   = ((page & 0x0f) << 8) | pdt.dataBaseAddr;
				ts->map.fn19.commandBase= ((page & 0x0f) << 8) | pdt.commandBaseAddr;

				rc = shtps_rmi_read(ts, ts->map.fn19.queryBase,
									ts->map.fn19.query.data, sizeof(ts->map.fn19.query.data));
				if(rc){
					goto err_exit;
				}

				SHTPS_LOG_DBG_PRINT("F19 Query Data\n");
				SHTPS_LOG_DBG_PRINT("-------------------------------------------------\n");
				SHTPS_LOG_DBG_PRINT("HasSensitivityAdjust   : 0x%02x\n", F19_QUERY_HASSENSITIVITYADJUST(ts->map.fn19.query.data));
				SHTPS_LOG_DBG_PRINT("HasHysteresisThreshold : 0x%02x\n", F19_QUERY_HASHYSTERESISTHRESHOLD(ts->map.fn19.query.data));
				SHTPS_LOG_DBG_PRINT("ButtonCount            : 0x%02x\n", F19_QUERY_BUTTONCOUNT(ts->map.fn19.query.data));
				break;

			case 0x1A:
				SHTPS_LOG_DBG_PRINT("Found: 0-D Capacitivve Buttons\n");
				ts->map.fn1A.enable		= 1;
				ts->map.fn1A.queryBase  = ((page & 0x0f) << 8) | pdt.queryBaseAddr;
				ts->map.fn1A.ctrlBase   = ((page & 0x0f) << 8) | pdt.controlBaseAddr;
				ts->map.fn1A.dataBase   = ((page & 0x0f) << 8) | pdt.dataBaseAddr;
				ts->map.fn1A.commandBase= ((page & 0x0f) << 8) | pdt.commandBaseAddr;

				rc = shtps_rmi_read(ts, ts->map.fn1A.queryBase,
									ts->map.fn1A.query.data, sizeof(ts->map.fn1A.query.data));
				if(rc){
					goto err_exit;
				}

				SHTPS_LOG_DBG_PRINT("F1A Query Data\n");
				SHTPS_LOG_DBG_PRINT("-------------------------------------------------\n");
				SHTPS_LOG_DBG_PRINT("MaxButtonCount               : 0x%02x\n", F1A_QUERY_MAX_BUTTONCOUNT(ts->map.fn1A.query.data));
				SHTPS_LOG_DBG_PRINT("HasGeneralControle           : 0x%02x\n", F1A_QUERY_HASGENERALCONTROL(ts->map.fn1A.query.data));
				SHTPS_LOG_DBG_PRINT("HasInterruptEnable           : 0x%02x\n", F1A_QUERY_HASINTERRUPTENABLE(ts->map.fn1A.query.data));
				SHTPS_LOG_DBG_PRINT("HasMultiButtonSelect         : 0x%02x\n", F1A_QUERY_HASMULTIBUTTONSELECT(ts->map.fn1A.query.data));
				SHTPS_LOG_DBG_PRINT("HasTxRxMapping               : 0x%02x\n", F1A_QUERY_HASTXRXMAPPING(ts->map.fn1A.query.data));
				SHTPS_LOG_DBG_PRINT("HasPerButtonThreshold        : 0x%02x\n", F1A_QUERY_HASPERBUTTONTHRESHOLD(ts->map.fn1A.query.data));
				SHTPS_LOG_DBG_PRINT("HasReleaseThreshold          : 0x%02x\n", F1A_QUERY_HASRELEASETHRESHOLD(ts->map.fn1A.query.data));
				SHTPS_LOG_DBG_PRINT("HasStrongestButtonHysteresis : 0x%02x\n", F1A_QUERY_HASSTRONGESTBUTTONHYSTERESIS(ts->map.fn1A.query.data));
				SHTPS_LOG_DBG_PRINT("HasFilterStrength            : 0x%02x\n", F1A_QUERY_HASFILTERSTRENGTH(ts->map.fn1A.query.data));
				break;

			default:
				break;
			}
		}
	}
	shtps_rmi_write(ts, 0xFF, 0x00);

	if(0 == ts->map.fn01.enable){
		rc = -1;
		goto err_exit;
	}

#if defined(SHTPS_LPWG_MODE_ENABLE)
	shtps_get_lpwg_def_settings(ts);
#endif /* SHTPS_LPWG_MODE_ENABLE */

	return 0;

err_exit:
	return rc;
}

#if defined( CONFIG_SHTPS_SY3000_FACETOUCH_OFF_DETECT )
static void shtps_wake_lock(struct shtps_rmi_spi *ts)
{
	if(ts->state_mgr.state == SHTPS_STATE_SLEEP_FACETOUCH){
		SHTPS_LOG_FUNC_CALL();
		_log_msg_sync( LOGMSG_ID__FACETOUCH_WAKE_LOCK, "");
		wake_lock(&ts->facetouch.wake_lock);
	}
}

static void shtps_wake_unlock(struct shtps_rmi_spi *ts)
{
	SHTPS_LOG_FUNC_CALL();
	_log_msg_sync( LOGMSG_ID__FACETOUCH_WAKE_UNLOCK, "");
	wake_unlock(&ts->facetouch.wake_lock);
}
#endif /* #if defined( CONFIG_SHTPS_SY3000_FACETOUCH_OFF_DETECT ) */

static void shtps_reset(struct shtps_rmi_spi *ts)
{
	SHTPS_LOG_FUNC_CALL();
	_log_msg_sync( LOGMSG_ID__HW_RESET, "");

	gpio_set_value(ts->rst_pin, 0);
	udelay(SHTPS_HWRESET_TIME_US);
	gpio_set_value(ts->rst_pin, 1);

	mdelay(SHTPS_HWRESET_AFTER_TIME_MS);
}

static void shtps_sleep(struct shtps_rmi_spi *ts, int on)
{
	u8 val;

	SHTPS_LOG_FUNC_CALL_INPARAM(on);

	_log_msg_sync( LOGMSG_ID__SET_SLEEP, "%d", on);
	shtps_rmi_read(ts, ts->map.fn01.ctrlBase, &val, 1);
	if(on){
		#if defined( SHTPS_LOW_POWER_MODE_ENABLE )
			ts->lpmode_state &= ~SHTPS_LPMODE_NON_CONTINUOUS_REQ_ON;
		#endif /* #if defined( SHTPS_LOW_POWER_MODE_ENABLE ) */

		shtps_delayed_rezero_cancel(ts);
		#if defined( CONFIG_SHTPS_SY3000_ALWAYS_ACTIVEMODE )
			val &= ~0x04;
			shtps_rmi_write(ts, ts->map.fn01.ctrlBase, val);
		#endif /* #if defined( CONFIG_SHTPS_SY3000_ALWAYS_ACTIVEMODE ) */
		#if defined(SHTPS_LPWG_MODE_ENABLE)
            if (ts->lpwg.lpwg_switch)
    			shtps_rmi_write(ts, ts->map.fn01.ctrlBase, val & (~0x03));
            else
                shtps_rmi_write(ts, ts->map.fn01.ctrlBase, val | 0x01);
		#else
			shtps_rmi_write(ts, ts->map.fn01.ctrlBase, val | 0x01);
		#endif /* SHTPS_LPWG_MODE_ENABLE */
	}else{
		#if defined( CONFIG_SHTPS_SY3000_ALWAYS_ACTIVEMODE )
			#if defined( SHTPS_LOW_POWER_MODE_ENABLE )
				if((ts->lpmode_state & SHTPS_LPMODE_ON) == 0){
					val |= 0x04;
					shtps_rmi_write(ts, ts->map.fn01.ctrlBase, val);
				}
			#else
				val |= 0x04;
				shtps_rmi_write(ts, ts->map.fn01.ctrlBase, val);
			#endif /* #if defined( SHTPS_LOW_POWER_MODE_ENABLE ) */
		#endif /* #if defined( CONFIG_SHTPS_SY3000_ALWAYS_ACTIVEMODE ) */
		shtps_rmi_write(ts, ts->map.fn01.ctrlBase, val & 0xFC);
	}

	#if defined( SHTPS_LOW_POWER_MODE_ENABLE )
		SHTPS_LOG_DBG_PRINT("sleep request recieved. state = 0x%02x\n", ts->lpmode_state);
	#endif /* #if defined( SHTPS_LOW_POWER_MODE_ENABLE ) */
}

#if defined( SHTPS_LOW_POWER_MODE_ENABLE )
static void shtps_set_doze_mode(struct shtps_rmi_spi *ts, int on)
{
	u8 val;
	SHTPS_LOG_FUNC_CALL_INPARAM(on);

	shtps_rmi_read(ts, ts->map.fn01.ctrlBase, &val, 1);
	if((val & 0x03) != 0){
		return;
	}
	
	if(on){
		val &= ~0x04;
		shtps_rmi_write(ts, ts->map.fn01.ctrlBase, val);
		SHTPS_LOG_DBG_PRINT("doze mode on ([0x%02x] <- 0x%02x)\n", ts->map.fn01.ctrlBase, val);
	}else{
		val |= 0x04;
		shtps_rmi_write(ts, ts->map.fn01.ctrlBase, val);
		SHTPS_LOG_DBG_PRINT("doze mode off([0x%02x] <- 0x%02x)\n", ts->map.fn01.ctrlBase, val);
	}
}

static void shtps_set_lpmode(struct shtps_rmi_spi *ts, int type, int on)
{
	if(on){
		if((ts->lpmode_state & type) == 0){
			SHTPS_LOG_DBG_PRINT("%s(0x%02x) ON\n", 
				(type == SHTPS_LPMODE_NON_CONTINUOUS_REQ_ON)? "NON CONTINUOUS LPMODE" :
				(type == SHTPS_LPMODE_CONTINUOUS_REQ_ON)? "CONTINUOUS LPMODE" : "UNKNOWN ERROR", type);

			if((ts->lpmode_state & SHTPS_LPMODE_ON) == 0){
				shtps_set_doze_mode(ts, 1);
			}
			ts->lpmode_state |= type;
		}
	}else{
		if((ts->lpmode_state & type) != 0){
			SHTPS_LOG_DBG_PRINT("%s(0x%02x) OFF\n", 
				(type == SHTPS_LPMODE_NON_CONTINUOUS_REQ_ON)? "NON CONTINUOUS LPMODE" :
				(type == SHTPS_LPMODE_CONTINUOUS_REQ_ON)? "CONTINUOUS LPMODE" : "UNKNOWN ERROR", type);

			ts->lpmode_state &= ~type;
			if((ts->lpmode_state & SHTPS_LPMODE_ON) == 0){
				shtps_set_doze_mode(ts, 0);
			}
		}
	}
	
	SHTPS_LOG_DBG_PRINT("lpmode request recieved. state = 0x%02x\n", ts->lpmode_state);
}

#endif /* #if defined( SHTPS_LOW_POWER_MODE_ENABLE ) */

static void shtps_set_charger_armor_init(struct shtps_rmi_spi *ts)
{
	#if defined( SHTPS_CHARGER_ARMOR_ENABLE )
		if(ts->charger_armor_state == 0){
			shtps_rmi_write(ts, ts->map.fn11.ctrlBase + 0x39, SHTPS_CHARGER_ARMOR_STRENGTH_DEFAULT);
		}else{
			shtps_rmi_write(ts, ts->map.fn11.ctrlBase + 0x39, SHTPS_CHARGER_ARMOR_STRENGTH);
		}
	#endif /* #if defined( SHTPS_CHARGER_ARMOR_ENABLE ) */
}

#if defined(SHTPS_LPWG_MODE_ENABLE)
static void shtps_lpwg_notify_interval_delayed_work_function(struct work_struct *work)
{
	struct delayed_work *dw = container_of(work, struct delayed_work, work);
	struct shtps_lpwg_ctrl *lpwg_p = container_of(dw, struct shtps_lpwg_ctrl, notify_interval_delayed_work);

	SHTPS_LOG_FUNC_CALL();

	mutex_lock(&shtps_ctrl_lock);
	lpwg_p->notify_enable = 1;
	mutex_unlock(&shtps_ctrl_lock);

	SHTPS_LOG_DBG_PRINT("[LPWG] notify interval end\n");
}

static void shtps_lpwg_notify_interval_stop(struct shtps_rmi_spi *ts)
{
	SHTPS_LOG_FUNC_CALL();

	cancel_delayed_work(&ts->lpwg.notify_interval_delayed_work);
}

static void shtps_lpwg_notify_interval_start(struct shtps_rmi_spi *ts)
{
	SHTPS_LOG_FUNC_CALL();

	shtps_lpwg_notify_interval_stop(ts);
	schedule_delayed_work(&ts->lpwg.notify_interval_delayed_work, msecs_to_jiffies(SHTPS_LPWG_MIN_NOTIFY_INTERVAL));

	SHTPS_LOG_DBG_PRINT("[LPWG] notify interval start\n");
}

static void shtps_lpwg_wakelock_init(struct shtps_rmi_spi *ts)
{
	ts->lpwg.notify_enable = 1;
	ts->lpwg.lpwg_switch = 0;
    wake_lock_init(&ts->lpwg.wake_lock, WAKE_LOCK_SUSPEND, "shtps_lpwg_wake_lock");
	pm_qos_add_request(&ts->lpwg.pm_qos_lock_idle, PM_QOS_CPU_DMA_LATENCY, PM_QOS_DEFAULT_VALUE);
	ts->lpwg.pm_qos_idle_value = msm_cpuidle_get_deep_idle_latency();
	if(ts->lpwg.pm_qos_idle_value == 0){
		ts->lpwg.pm_qos_idle_value = SHTPS_LPWG_QOS_LATENCY_DEF_VALUE;
	}
	INIT_DELAYED_WORK(&ts->lpwg.notify_interval_delayed_work, shtps_lpwg_notify_interval_delayed_work_function);
}

static void shtps_lpwg_wakelock_destroy(struct shtps_rmi_spi *ts)
{
   wake_lock_destroy(&ts->lpwg.wake_lock);
   pm_qos_remove_request(&ts->lpwg.pm_qos_lock_idle);
}

static void shtps_lpwg_wakelock(struct shtps_rmi_spi *ts, int on)
{
	if(on){
	    wake_lock(&ts->lpwg.wake_lock);
	    pm_qos_update_request(&ts->lpwg.pm_qos_lock_idle, ts->lpwg.pm_qos_idle_value);
		SHTPS_LOG_DBG_PRINT("lpwg wake lock ON\n");
	}else{
		wake_unlock(&ts->lpwg.wake_lock);
	    pm_qos_update_request(&ts->lpwg.pm_qos_lock_idle, PM_QOS_DEFAULT_VALUE);
		SHTPS_LOG_DBG_PRINT("lpwg wake lock OFF\n");
	}
}

static void shtps_get_lpwg_def_settings(struct shtps_rmi_spi *ts)
{
	shtps_rmi_read(ts, ts->map.fn01.ctrlBase + 0x03, &ts->lpwg.doze_wakeup_threshold, 1);
	shtps_rmi_read(ts, ts->map.fn11.ctrlBase + 0x25, &ts->lpwg.clip[0], 2);
}

static void shtps_set_lpwg_mode_on(struct shtps_rmi_spi *ts)
{
	u8 data;

	shtps_lpwg_notify_interval_stop(ts);
	ts->lpwg.notify_enable = 1;
	ts->lpwg.is_notified = 0;

    shtps_rmi_write(ts, ts->map.fn1A.ctrlBase + 0x13, SHTPS_LPWG_DOZE_0D_THRESHOLD);
    shtps_rmi_write(ts, ts->map.fn1A.ctrlBase + 0x14, SHTPS_LPWG_DOZE_0D_THRESHOLD);
	shtps_rmi_write(ts, 0x009E, SHTPS_LPWG_DOZE_CTL);
	shtps_rmi_write(ts, 0x0402, SHTPS_LPWG_MAX_ACTIVE_DURATION);
	shtps_rmi_write(ts, ts->map.fn01.ctrlBase + 0x02, SHTPS_LPWG_DOZE_INTERVAL);
	shtps_rmi_write(ts, ts->map.fn01.ctrlBase + 0x03, ts->lpwg.doze_wakeup_threshold);
	shtps_rmi_write(ts, ts->map.fn01.ctrlBase + 0x04, SHTPS_LPWG_DOZE_HOLDOFF);
	shtps_rmi_write(ts, ts->map.fn11.ctrlBase + 0x25, SHTPS_LPWG_TX_CLIP_LO);
	shtps_rmi_write(ts, ts->map.fn11.ctrlBase + 0x26, SHTPS_LPWG_TX_CLIP_HI);

	shtps_rmi_read(ts, ts->map.fn11.ctrlBase, &data, 1);
	shtps_rmi_write(ts, ts->map.fn11.ctrlBase, (data & 0xF8) | 0x04);

	SHTPS_LOG_DBG_PRINT("LPWG mode ON\n");
}

static void shtps_set_lpwg_mode_off(struct shtps_rmi_spi *ts)
{
	u8 data;

	shtps_rmi_read(ts, ts->map.fn11.ctrlBase, &data, 1);
	shtps_rmi_write(ts, ts->map.fn11.ctrlBase, (data & 0xF8) | 0x00);

	shtps_rmi_write(ts, 0x009E, 0x00);
    shtps_rmi_write(ts, ts->map.fn1A.ctrlBase + 0x13, SHTPS_LPWG_NORMAL_0D_THRESHOLD);
    shtps_rmi_write(ts, ts->map.fn1A.ctrlBase + 0x14, SHTPS_LPWG_NORMAL_0D_THRESHOLD);
	shtps_rmi_write(ts, ts->map.fn01.ctrlBase + 0x02, SHTPS_LPWG_DOZE_INTERVAL_DEF);
	shtps_rmi_write(ts, ts->map.fn01.ctrlBase + 0x03, SHTPS_LPWG_DOZE_WAKEUP_THRESHOLD_DEF);
	shtps_rmi_write(ts, ts->map.fn01.ctrlBase + 0x04, SHTPS_LPWG_DOZE_HOLDOFF_DEF);
	shtps_rmi_write(ts, ts->map.fn11.ctrlBase + 0x25, ts->lpwg.clip[0]);
	shtps_rmi_write(ts, ts->map.fn11.ctrlBase + 0x26, ts->lpwg.clip[1]);

	shtps_read_touchevent(ts, SHTPS_STATE_IDLE);

	SHTPS_LOG_DBG_PRINT("LPWG mode OFF\n");
}
#endif /* SHTPS_LPWG_MODE_ENABLE */

static int shtps_set_charger_armor(struct shtps_rmi_spi *ts, int on)
{
	#if defined( SHTPS_CHARGER_ARMOR_ENABLE )
		if(on != 0){
			if(ts->charger_armor_state == 0){
				ts->charger_armor_state = 1;
				shtps_rmi_write(ts, ts->map.fn11.ctrlBase + 0x39, SHTPS_CHARGER_ARMOR_STRENGTH);
			}
		}else{
			if(ts->charger_armor_state != 0){
				ts->charger_armor_state = 0;
				shtps_rmi_write(ts, ts->map.fn11.ctrlBase + 0x39, SHTPS_CHARGER_ARMOR_STRENGTH_DEFAULT);
			}
		}
	#endif /* #if defined( SHTPS_CHARGER_ARMOR_ENABLE ) */

	return 0;
}

static int shtps_fwdate(struct shtps_rmi_spi *ts, u8 *year, u8 *month)
{
	u8 buf[2] = { 0x00, 0x00 };
	u8 retry = 3;

	do{
		shtps_rmi_read(ts, ts->map.fn01.queryBase + 4, buf, 2);
		if(buf[0] != 0x00 && buf[1] != 0x00){
			break;
		}
	}while(retry-- > 0);

	*year = buf[0] & 0x1F;
#if defined( SHTPS_SY_REGMAP_BASE3 )
	*month= ((buf[0] >> 5) & 0x07) | ((buf[1] << 3) & 0x08 );
#else
	*month= buf[1] & 0x0F;
#endif /* #if defined( SHTPS_SY_REGMAP_BASE3 ) */

	_log_msg_sync( LOGMSG_ID__GET_FWDATE, "%d|%d", *year, *month);
	return 0;
}

static u16 shtps_fwver(struct shtps_rmi_spi *ts)
{
	u8 ver[2] = { 0x00, 0x00 };
	u8 retry = 3;

	do{
		#if defined( SHTPS_SY_REGMAP_BASE3 )
		shtps_rmi_read(ts, ts->map.fn34.ctrlBase + 2, ver, 2);
		#else
		shtps_rmi_read(ts, ts->map.fn01.queryBase + 2, ver, 2);
		#endif /* #if defined( SHTPS_SY_REGMAP_BASE3 ) */
		if(ver[0] != 0x00 || ver[1] != 0x00){
			break;
		}
	}while(retry-- > 0);

	_log_msg_sync( LOGMSG_ID__GET_FWVER, "0x%04X", ((ver[0] << 0x08) & 0xff00) | ver[1]);
	return ((ver[0] << 0x08) & 0xff00) | ver[1];
}

static int shtps_init_param(struct shtps_rmi_spi *ts)
{
	int rc;
#if defined( SHTPS_BTLOADER_VER_ENABLE )
	u8 val[8];
#endif	/* #if defined( SHTPS_BTLOADER_VER_ENABLE ) */

	_log_msg_sync( LOGMSG_ID__FW_INIT, "");
	SHTPS_LOG_FUNC_CALL();

	rc = shtps_rmi_write(ts, ts->map.fn01.dataBase, 0x00);
	SPI_ERR_CHECK(rc, err_exit);

	if(ts->map.fn11.enable){
		u8 data;
		rc = shtps_rmi_read(ts, ts->map.fn11.ctrlBase, &data, 1);
		SPI_ERR_CHECK(rc, err_exit);
		rc = shtps_rmi_write(ts, ts->map.fn11.ctrlBase, (data & 0xF8) | 0x00);
		SPI_ERR_CHECK(rc, err_exit);
#if !defined( SHTPS_SY_REGMAP_BASE3 )
		if(F11_QUERY_HASGESTURES(ts->map.fn11.query.data)){
			rc = shtps_rmi_write(ts, ts->map.fn11.ctrlBase + 10, 0x00);
			SPI_ERR_CHECK(rc, err_exit);
			if(F11_QUERY_HASGESTURE1(ts->map.fn11.query.data)){
				rc = shtps_rmi_write(ts, ts->map.fn11.ctrlBase + 11, 0x00);
				SPI_ERR_CHECK(rc, err_exit);
			}
		}
#endif /* #if !defined( SHTPS_SY_REGMAP_BASE3 ) */
	}
	rc = shtps_rmi_write(ts, ts->map.fn01.ctrlBase + 1, SHTPS_IRQ_ALL);
	SPI_ERR_CHECK(rc, err_exit);

	shtps_set_palmthresh(ts, SHTPS_LOS_SINGLE);

	#if defined( CONFIG_SHTPS_SY3000_ALWAYS_ACTIVEMODE )
	rc = shtps_rmi_write(ts, ts->map.fn01.ctrlBase, 0x84);
	#else
	rc = shtps_rmi_write(ts, ts->map.fn01.ctrlBase, 0x80);
	#endif /* #if defined( CONFIG_SHTPS_SY3000_ALWAYS_ACTIVEMODE ) */
	SPI_ERR_CHECK(rc, err_exit);


#if defined( SHTPS_BTLOADER_VER_ENABLE )
	if( ts->bt_ver == 0x0102 ){
		shtps_rmi_write(ts, 0xFF, 0x01);
		shtps_rmi_read(ts, ts->map.fn54.ctrlBase + 0x30, val, 8);
		val[0] |= 0x08;
		val[1] |= 0x08;
		val[2] |= 0x08;
		val[4] &= ~0x08;
		val[5] &= ~0x08;
		val[6] &= ~0x08;
		shtps_rmi_write(ts, ts->map.fn54.ctrlBase + 0x30, val[0]);
		shtps_rmi_write(ts, ts->map.fn54.ctrlBase + 0x31, val[1]);
		shtps_rmi_write(ts, ts->map.fn54.ctrlBase + 0x32, val[2]);
		shtps_rmi_write(ts, ts->map.fn54.ctrlBase + 0x34, val[4]);
		shtps_rmi_write(ts, ts->map.fn54.ctrlBase + 0x35, val[5]);
		shtps_rmi_write(ts, ts->map.fn54.ctrlBase + 0x36, val[6]);
		shtps_rmi_write(ts, ts->map.fn54.commandBase, 0x04);
		msleep(25);
		shtps_rmi_write(ts, 0xFF, 0x00);
	}
#endif	/* #if defined( SHTPS_BTLOADER_VER_ENABLE ) */

#if defined( SHTPS_PHYSICAL_KEY_ENABLE )
	if(ts->map.fn1A.enable){
		//int	button_cnt = F1A_QUERY_MAX_BUTTONCOUNT(ts->map.fn1A.query.data);
		int	button_cnt = 2;

		rc = shtps_rmi_write(ts, 0xFF, ((ts->map.fn1A.ctrlBase >> 8) & 0xFF));
		SPI_ERR_CHECK(rc, err_exit);

		rc = shtps_rmi_write(ts, ts->map.fn1A.ctrlBase, 0x00);
		SPI_ERR_CHECK(rc, err_exit);

		rc = shtps_rmi_write(ts, ts->map.fn1A.ctrlBase + 1, ((1 << button_cnt) - 1) & 0xFF);
		SPI_ERR_CHECK(rc, err_exit);

		rc = shtps_rmi_write(ts, ts->map.fn1A.ctrlBase + 2, 0x00);
		SPI_ERR_CHECK(rc, err_exit);

		rc = shtps_rmi_write(ts, 0xFF, 0x00);
		SPI_ERR_CHECK(rc, err_exit);
	}
#endif /* #if defined( SHTPS_PHYSICAL_KEY_ENABLE ) */

#if defined(SHTPS_LPWG_MODE_ENABLE)
	#if defined( SHTPS_PHYSICAL_KEY_ENABLE )
	if(ts->map.fn1A.enable){
		rc = shtps_rmi_write(ts, ts->map.fn1A.ctrlBase + 0x13, SHTPS_LPWG_NORMAL_0D_THRESHOLD);
		SPI_ERR_CHECK(rc, err_exit);
		rc = shtps_rmi_write(ts, ts->map.fn1A.ctrlBase + 0x14, SHTPS_LPWG_NORMAL_0D_THRESHOLD);
		SPI_ERR_CHECK(rc, err_exit);
	}
	#endif /* #if defined( SHTPS_PHYSICAL_KEY_ENABLE ) */
	
	rc = shtps_rmi_write(ts, ts->map.fn01.ctrlBase + 0x02, SHTPS_LPWG_DOZE_INTERVAL_DEF);
	SPI_ERR_CHECK(rc, err_exit);
	rc = shtps_rmi_write(ts, ts->map.fn01.ctrlBase + 0x03, SHTPS_LPWG_DOZE_WAKEUP_THRESHOLD_DEF);
	SPI_ERR_CHECK(rc, err_exit);
	rc = shtps_rmi_write(ts, ts->map.fn01.ctrlBase + 0x04, SHTPS_LPWG_DOZE_HOLDOFF_DEF);
	SPI_ERR_CHECK(rc, err_exit);
#endif /* SHTPS_LPWG_MODE_ENABLE */

#if defined( SHTPS_AUTOREZERO_CONTROL_ENABLE )
	{
		u8  val;
		_log_msg_sync( LOGMSG_ID__AUTO_REZERO_ENABLE, "");
		shtps_rmi_read(ts, 0xF0, &val, 1);
		shtps_rmi_write(ts, 0xF0, val | 0x01);
	}
#endif /* #if defined( SHTPS_AUTOREZERO_CONTROL_ENABLE ) */

	shtps_set_charger_armor_init(ts);

	return 0;

err_exit:
	return -1;
}

static void shtps_standby_param(struct shtps_rmi_spi *ts)
{
	_log_msg_sync( LOGMSG_ID__FW_STANDBY, "");
	SHTPS_LOG_FUNC_CALL();
	shtps_rmi_write(ts, ts->map.fn01.ctrlBase, 0x01);
	shtps_set_palmthresh(ts, 0);

#if 0
	msleep(SHTPS_SLEEP_IN_WAIT_MS);
	while(retry-- > 0){
		shtps_rmi_read(ts, ts->map.fn01.dataBase, buf, 2);
		if((buf[1] & 0x0F) == 0x00){
			break;
		}
		udelay(1000);
	}
#endif
}

static void shtps_clr_startup_err(struct shtps_rmi_spi *ts)
{
	SHTPS_LOG_FUNC_CALL();
	ts->state_mgr.starterr = SHTPS_STARTUP_SUCCESS;
}

static int shtps_wait_startup(struct shtps_rmi_spi *ts)
{
	SHTPS_LOG_FUNC_CALL();
	_log_msg_sync( LOGMSG_ID__FW_STARTUP_COMP_WAIT, "");
	wait_event_interruptible(ts->wait_start,
		ts->state_mgr.state == SHTPS_STATE_ACTIVE          ||
		ts->state_mgr.state == SHTPS_STATE_BOOTLOADER      ||
		ts->state_mgr.state == SHTPS_STATE_FWTESTMODE      ||
		ts->state_mgr.state == SHTPS_STATE_SLEEP           ||
		ts->state_mgr.state == SHTPS_STATE_FACETOUCH       ||
		ts->state_mgr.state == SHTPS_STATE_SLEEP_FACETOUCH ||
		ts->state_mgr.starterr == SHTPS_STARTUP_FAILED);

	_log_msg_recv( LOGMSG_ID__FW_STARTUP_COMP, "%d|%d", ts->state_mgr.state, ts->state_mgr.starterr);

	return ts->state_mgr.starterr;
}

static void shtps_notify_startup(struct shtps_rmi_spi *ts, u8 err)
{
	SHTPS_LOG_FUNC_CALL_INPARAM(err);
	ts->state_mgr.starterr = err;
	_log_msg_send( LOGMSG_ID__FW_STARTUP_COMP, "%d|%d", ts->state_mgr.state, ts->state_mgr.starterr);
	wake_up_interruptible(&ts->wait_start);
}

static void shtps_set_startmode(struct shtps_rmi_spi *ts, u8 mode)
{
	SHTPS_LOG_FUNC_CALL_INPARAM(mode);
	_log_msg_sync( LOGMSG_ID__FW_STARTUP_MODE, "%d", mode);
	ts->state_mgr.mode = mode;
}

static int shtps_get_startmode(struct shtps_rmi_spi *ts)
{
	SHTPS_LOG_FUNC_CALL();
	return ts->state_mgr.mode;
}

static inline int shtps_get_fingermax(struct shtps_rmi_spi *ts)
{
	const int finger_tbl[8]={1, 2, 3, 4, 5, 10, 1, 1};

	return finger_tbl[F11_QUERY_NUMOFFINGERS(ts->map.fn11.query.data)];
}

static int shtps_get_facetouchmode(struct shtps_rmi_spi *ts)
{
	SHTPS_LOG_FUNC_CALL();
	return ts->facetouch.mode;
}

static void shtps_set_facetouchmode(struct shtps_rmi_spi *ts, int mode)
{
	SHTPS_LOG_FUNC_CALL_INPARAM(mode);
	_log_msg_sync( LOGMSG_ID__SET_FACETOUCH_MODE, "%d", mode);
	ts->facetouch.mode = mode;
	if(mode == 0){
		ts->facetouch.off_detect = 0;
	}
}

#if defined( CONFIG_SHTPS_SY3000_FACETOUCH_OFF_DETECT )
static void shtps_notify_facetouchoff(struct shtps_rmi_spi *ts, int force)
{
	ts->facetouch.state = 0;
	ts->facetouch.off_detect = 1;
	_log_msg_send( LOGMSG_ID__DETECT_FACETOUCH, "");
	wake_up_interruptible(&ts->facetouch.wait_off);
	SHTPS_LOG_DBG_PRINT("face touch off detect. wake_up()\n");
}

static void shtps_check_facetouch(struct shtps_rmi_spi *ts, struct shtps_touch_info *info)
{
	int i;
	u8 	fingerMax = shtps_get_fingermax(ts);

	for(i = 0;i < fingerMax;i++){
		if(info->fingers[i].state == SHTPS_TOUCH_STATE_FINGER){
			if(info->fingers[i].wx >= 15){
				ts->facetouch.state = 1;
				break;
			}
		}
	}
}
#endif /* #if defined( CONFIG_SHTPS_SY3000_FACETOUCH_OFF_DETECT ) */

/* -----------------------------------------------------------------------------------
 */
static int shtps_get_diff(unsigned short pos1, unsigned short pos2, unsigned long factor)
{
	int diff = (pos1 * factor / 10000) - (pos2 * factor / 10000);
	return (diff >= 0)? diff : -diff;
}

static void shtps_rec_notify_time(struct shtps_rmi_spi *ts, int xy, int index)
{
	ts->touch_state.drag_timeout[index][xy] = jiffies + msecs_to_jiffies(ts->touch_state.dragStepReturnTime[index][xy]);
}

static int shtps_chk_notify_time(struct shtps_rmi_spi *ts, int xy, int index)
{
	if(time_after(jiffies, ts->touch_state.drag_timeout[index][xy])){
		return -1;
	}
	return 0;
}

static int shtps_get_fingerwidth(struct shtps_rmi_spi *ts, int num, struct shtps_touch_info *info)
{
#if defined( CONFIG_SHTPS_SY3000_FACETOUCH_DETECT ) || defined( CONFIG_SHTPS_SY3000_FACETOUCH_OFF_DETECT )
	int w;

	if(info->gs2 & 0x01 || (shtps_get_facetouchmode(ts) && info->finger_num >= 2)){
		w = SHTPS_FINGER_WIDTH_PALMDET;
	}else{
		w = (info->fingers[num].wx >= info->fingers[num].wy)? info->fingers[num].wx : info->fingers[num].wy;
		if(w < SHTPS_FINGER_WIDTH_MIN){
			w = SHTPS_FINGER_WIDTH_MIN;
		}else if(w > SHTPS_FINGER_WIDTH_MAX){
			w = SHTPS_FINGER_WIDTH_MAX;
		}
	}
	return w;
#else  /* #if defined( CONFIG_SHTPS_SY3000_FACETOUCH_DETECT ) || defined( CONFIG_SHTPS_SY3000_FACETOUCH_OFF_DETECT ) */
	int w = (info->fingers[num].wx >= info->fingers[num].wy)? info->fingers[num].wx : info->fingers[num].wy;
	return (w < SHTPS_FINGER_WIDTH_MIN)? SHTPS_FINGER_WIDTH_MIN : w;
#endif /* #if defined( CONFIG_SHTPS_SY3000_FACETOUCH_DETECT ) || defined( CONFIG_SHTPS_SY3000_FACETOUCH_OFF_DETECT ) */
}

static int shtps_get_dragstep(struct shtps_rmi_spi *ts, int xy, int type, int fingers)
{
	int dragStep;

	if(type == SHTPS_DRAG_THRESHOLD_ZERO){
		if(xy == SHTPS_POSTYPE_X){
			dragStep = (fingers <= 1)? SHTPS_DRAG_THRESH_VAL_X_ZERO : SHTPS_DRAG_THRESH_VAL_X_1ST_MULTI;
		}else{
			dragStep = (fingers <= 1)? SHTPS_DRAG_THRESH_VAL_Y_ZERO : SHTPS_DRAG_THRESH_VAL_Y_1ST_MULTI;
		}
	}else if(type == SHTPS_DRAG_THRESHOLD_1ST){
		if(xy == SHTPS_POSTYPE_X){
			dragStep = (fingers <= 1)? SHTPS_DRAG_THRESH_VAL_X_1ST : SHTPS_DRAG_THRESH_VAL_X_1ST_MULTI;
		}else{
			dragStep = (fingers <= 1)? SHTPS_DRAG_THRESH_VAL_Y_1ST : SHTPS_DRAG_THRESH_VAL_Y_1ST_MULTI;
		}
	}else{
		if(xy == SHTPS_POSTYPE_X){
			dragStep = (fingers <= 1)? SHTPS_DRAG_THRESH_VAL_X_2ND : SHTPS_DRAG_THRESH_VAL_X_2ND_MULTI;
		}else{
			dragStep = (fingers <= 1)? SHTPS_DRAG_THRESH_VAL_Y_2ND : SHTPS_DRAG_THRESH_VAL_Y_2ND_MULTI;
		}
	}

	return dragStep;
}

static void shtps_set_dragstep(struct shtps_rmi_spi *ts,
		struct shtps_touch_info *info, int type, int xy, int finger)
{
	_log_msg_sync( LOGMSG_ID__SET_DRAG_STEP, "%d|%d|%d", type, xy, finger);

	if(type == SHTPS_DRAG_THRESHOLD_ZERO){
		ts->touch_state.dragStep[finger][xy] = type;
		ts->touch_state.dragStepReturnTime[finger][xy] = SHTPS_DRAG_THRESH_RETURN_TIME_ZERO;
	}else if(type == SHTPS_DRAG_THRESHOLD_1ST){
		ts->touch_state.dragStep[finger][xy] = type;
		ts->touch_state.dragStepReturnTime[finger][xy] = SHTPS_DRAG_THRESH_RETURN_TIME;
	}else{
		if(xy == SHTPS_POSTYPE_X){
			_log_msg_sync( LOGMSG_ID__SET_FINGER_CENTER, "%d|%d|%d", xy,
							info->fingers[finger].x * SHTPS_POS_SCALE_X(ts) / 10000,
							info->fingers[finger].x);
			ts->center_info.fingers[finger].x = info->fingers[finger].x;
		}else{
			_log_msg_sync( LOGMSG_ID__SET_FINGER_CENTER, "%d|%d|%d", xy,
							info->fingers[finger].y * SHTPS_POS_SCALE_Y(ts) / 10000,
							info->fingers[finger].y);
			ts->center_info.fingers[finger].y = info->fingers[finger].y;
		}
		ts->touch_state.dragStep[finger][xy] = type;
		shtps_rec_notify_time(ts, xy, finger);
	}
}

static inline void shtps_init_drag_hist(struct shtps_rmi_spi *ts, int xy, int finger, int pos)
{
	ts->drag_hist[finger][xy].pre   = pos;
	ts->drag_hist[finger][xy].count = 0;
}

static void shtps_add_drag_hist(struct shtps_rmi_spi *ts, int xy, int finger, int pos)
{
	int pre = ts->drag_hist[finger][xy].pre;
	u8 dir  = (pos > pre)? SHTPS_DRAG_DIR_PLUS :
			  (pos < pre)? SHTPS_DRAG_DIR_MINUS :
						   SHTPS_DRAG_DIR_NONE;

	SHTPS_LOG_DBG_PRINT("add drag hist[%d][%s] pre = %d, cur = %d, dir = %s, cnt = %d, remain time = %d\n",
		finger, (xy == SHTPS_POSTYPE_X)? "X" : "Y",
		pre, pos, 
		(dir == SHTPS_DRAG_DIR_PLUS)? "PLUS" : (dir == SHTPS_DRAG_DIR_MINUS)? "MINUS" : "NONE",
		ts->drag_hist[finger][xy].count,
		time_after(jiffies, ts->touch_state.drag_timeout[finger][xy]));

	if(dir != SHTPS_DRAG_DIR_NONE){
		if(ts->drag_hist[finger][xy].count == 0){
			ts->drag_hist[finger][xy].dir   = dir;
			ts->drag_hist[finger][xy].count = 1;
		}else{
			if(ts->drag_hist[finger][xy].dir == dir){
				if(ts->drag_hist[finger][xy].count < SHTPS_DRAG_DIR_FIX_CNT){
					ts->drag_hist[finger][xy].count++;
				}

				if(ts->drag_hist[finger][xy].count >= SHTPS_DRAG_DIR_FIX_CNT &&
						ts->touch_state.dragStep[finger][xy] == SHTPS_DRAG_THRESHOLD_2ND)
				{
					shtps_rec_notify_time(ts, xy, finger);
					if(xy == SHTPS_POSTYPE_X){
						ts->center_info.fingers[finger].x = pos;
					}else{
						ts->center_info.fingers[finger].y = pos;
					}
					SHTPS_LOG_DBG_PRINT("update center pos(%d, %d) time=%lu\n",
								ts->center_info.fingers[finger].x, ts->center_info.fingers[finger].y,
								ts->touch_state.drag_timeout[finger][xy]);
				}
			}else{
				ts->drag_hist[finger][xy].count = 1;
			}

			ts->drag_hist[finger][xy].dir = dir;
		}

		ts->drag_hist[finger][xy].pre = pos;
	}
}

static inline void shtps_set_eventtype(u8 *event, u8 type)
{
	*event = type;
}

static void shtps_calc_notify(struct shtps_rmi_spi *ts, u8 *buf, struct shtps_touch_info *info, u8 *event)
{
	int		i;
	u8		fingerMax = shtps_get_fingermax(ts);
	u8		numOfFingers = 0;
	u8*		fingerInfo;
	u8		base;
	int 	diff_x;
	int 	diff_y;
	int 	diff_cx;
	int 	diff_cy;
	int		dragStep1stX;
	int		dragStep1stY;
	int		dragStepCurX;
	int		dragStepCurY;

	SHTPS_LOG_FUNC_CALL();

	shtps_set_eventtype(event, 0xff);

	if(fingerMax > 8){
		base = 3;
	}else if (fingerMax > 4){
		base = 2;
	}else{
		base = 1;
	}
	for(i = 0;i < fingerMax;i++){
		info->fingers[i].state = (buf[i / 4] >> (2 * (i % 4))) & 0x03;
		if(info->fingers[i].state != SHTPS_TOUCH_STATE_NO_TOUCH){
			numOfFingers++;
		}
	}
	info->finger_num = numOfFingers;

	dragStep1stX = shtps_get_dragstep(ts, SHTPS_POSTYPE_X, SHTPS_DRAG_THRESHOLD_1ST, numOfFingers);
	dragStep1stY = shtps_get_dragstep(ts, SHTPS_POSTYPE_Y, SHTPS_DRAG_THRESHOLD_1ST, numOfFingers);
	for(i = 0;i < fingerMax;i++){
		fingerInfo = &buf[base + i * 5];
		info->fingers[i].x		= F11_DATA_XPOS(fingerInfo);
		info->fingers[i].y		= F11_DATA_YPOS(fingerInfo);
		info->fingers[i].wx		= F11_DATA_WX(fingerInfo);
		info->fingers[i].wy		= F11_DATA_WY(fingerInfo);
		info->fingers[i].z		= F11_DATA_Z(fingerInfo);

		_log_msg_sync( LOGMSG_ID__FW_EVENT, "%d|%d|%d|%d|%d|%d|%d|%d|%d", i, info->fingers[i].state,
							info->fingers[i].x * SHTPS_POS_SCALE_X(ts) / 10000,
							info->fingers[i].y * SHTPS_POS_SCALE_Y(ts) / 10000,
							info->fingers[i].x,
							info->fingers[i].y,
							info->fingers[i].wx,
							info->fingers[i].wy,
							info->fingers[i].z);

		dragStepCurX = shtps_get_dragstep(ts, SHTPS_POSTYPE_X, ts->touch_state.dragStep[i][SHTPS_POSTYPE_X], numOfFingers);
		dragStepCurY = shtps_get_dragstep(ts, SHTPS_POSTYPE_Y, ts->touch_state.dragStep[i][SHTPS_POSTYPE_Y], numOfFingers);

		if(info->fingers[i].state != SHTPS_TOUCH_STATE_NO_TOUCH){
			if(ts->report_info.fingers[i].state != SHTPS_TOUCH_STATE_NO_TOUCH){
				shtps_add_drag_hist(ts, SHTPS_POSTYPE_X, i, info->fingers[i].x);
				shtps_add_drag_hist(ts, SHTPS_POSTYPE_Y, i, info->fingers[i].y);
			}

			diff_x = shtps_get_diff(info->fingers[i].x, ts->report_info.fingers[i].x, SHTPS_POS_SCALE_X(ts));
			diff_y = shtps_get_diff(info->fingers[i].y, ts->report_info.fingers[i].y, SHTPS_POS_SCALE_Y(ts));
			diff_cx= shtps_get_diff(info->fingers[i].x, ts->center_info.fingers[i].x, SHTPS_POS_SCALE_X(ts));
			diff_cy= shtps_get_diff(info->fingers[i].y, ts->center_info.fingers[i].y, SHTPS_POS_SCALE_Y(ts));

			if(ts->report_info.fingers[i].state != SHTPS_TOUCH_STATE_NO_TOUCH){
				if(diff_cy >= dragStep1stY){
					if(ts->touch_state.dragStep[i][1] != SHTPS_DRAG_THRESHOLD_2ND){
						shtps_set_dragstep(ts, info, SHTPS_DRAG_THRESHOLD_2ND, SHTPS_POSTYPE_X, i);
						dragStepCurX = shtps_get_dragstep(ts, SHTPS_POSTYPE_X,
											ts->touch_state.dragStep[i][SHTPS_POSTYPE_X], numOfFingers);
					}
				}

				if(diff_x >= dragStepCurX){
					if(diff_cx >= dragStep1stX){
						shtps_set_eventtype(event, SHTPS_EVENT_DRAG);
						if(ts->touch_state.dragStep[i][0] != SHTPS_DRAG_THRESHOLD_2ND){
							shtps_set_dragstep(ts, info, SHTPS_DRAG_THRESHOLD_2ND, SHTPS_POSTYPE_Y, i);
							dragStepCurY = shtps_get_dragstep(ts, SHTPS_POSTYPE_Y,
												ts->touch_state.dragStep[i][SHTPS_POSTYPE_Y], numOfFingers);
						}
						shtps_set_dragstep(ts, info, SHTPS_DRAG_THRESHOLD_2ND, SHTPS_POSTYPE_X, i);

					}else if(shtps_chk_notify_time(ts, SHTPS_POSTYPE_X, i) == 0 ||
								ts->touch_state.dragStep[i][SHTPS_POSTYPE_X] != SHTPS_DRAG_THRESHOLD_2ND){
						shtps_set_eventtype(event, SHTPS_EVENT_DRAG);

					}else{
						info->fingers[i].x = ts->report_info.fingers[i].x;
						shtps_set_dragstep(ts, info, SHTPS_DRAG_THRESHOLD_1ST, SHTPS_POSTYPE_X, i);
					}
				}else{
					info->fingers[i].x = ts->report_info.fingers[i].x;
				}

				if(diff_y >= dragStepCurY){
					if(diff_cy >= dragStep1stY){
						shtps_set_eventtype(event, SHTPS_EVENT_DRAG);
						shtps_set_dragstep(ts, info, SHTPS_DRAG_THRESHOLD_2ND, SHTPS_POSTYPE_Y, i);

					}else if(shtps_chk_notify_time(ts, SHTPS_POSTYPE_Y, i) == 0 ||
								ts->touch_state.dragStep[i][1] != SHTPS_DRAG_THRESHOLD_2ND)
					{
						shtps_set_eventtype(event, SHTPS_EVENT_DRAG);
						ts->touch_state.dragStep[i][1] = SHTPS_DRAG_THRESHOLD_2ND;

					}else{
						info->fingers[i].y = ts->report_info.fingers[i].y;
						shtps_set_dragstep(ts, info, SHTPS_DRAG_THRESHOLD_1ST, SHTPS_POSTYPE_Y, i);
					}
				}else{
					info->fingers[i].y = ts->report_info.fingers[i].y;
				}
			}else{
				ts->center_info.fingers[i].x = info->fingers[i].x;
				ts->center_info.fingers[i].y = info->fingers[i].y;
			}
		}else{
			shtps_set_dragstep(ts, info, SHTPS_DRAG_THRESHOLD_ZERO, SHTPS_POSTYPE_X, i);
			shtps_set_dragstep(ts, info, SHTPS_DRAG_THRESHOLD_ZERO, SHTPS_POSTYPE_Y, i);

			shtps_init_drag_hist(ts, SHTPS_POSTYPE_X, i, info->fingers[i].x);
			shtps_init_drag_hist(ts, SHTPS_POSTYPE_Y, i, info->fingers[i].y);
		}

		if(info->fingers[i].state != ts->report_info.fingers[i].state){
			shtps_set_eventtype(event, SHTPS_EVENT_MTDU);
		}
	}

#if defined( SHTPS_SY_REGMAP_BASE3 )
	info->gs1 = (buf[base + fingerMax * 5] & 0x02)? 0x01 : 0x00;
	info->gs2 = 0;
#else
	if(F11_QUERY_HASGESTURES(ts->map.fn11.query.data)){
		if(F11_QUERY_HASGESTURE1(ts->map.fn11.query.data) == 0){
			info->gs1 		= 0;
			info->gs2 		= buf[base + fingerMax * 5];
		}else{
			info->gs1 		= buf[base + fingerMax * 5];
			info->gs2 		= buf[base + fingerMax * 5 + 1];
		}
	}else{
			info->gs1 		= 0;
			info->gs2 		= 0;
	}
#endif /* #if defined( SHTPS_SY_REGMAP_BASE3 ) */

	if(numOfFingers > 0){
		ts->poll_info.stop_count = 0;
		if(ts->touch_state.numOfFingers == 0){
			shtps_set_eventtype(event, SHTPS_EVENT_TD);
		}
		if(numOfFingers >= 2 && ts->touch_state.numOfFingers < 2){
			if(shtps_get_facetouchmode(ts)){
				shtps_set_palmthresh(ts, SHTPS_LOS_MULTI);
			}
			shtps_rezero_handle(ts, SHTPS_REZERO_HANDLE_EVENT_MTD, info->gs2);
		}
		shtps_rezero_handle(ts, SHTPS_REZERO_HANDLE_EVENT_TOUCH, info->gs2);
	}else{
		if(ts->touch_state.numOfFingers != 0){
			shtps_set_eventtype(event, SHTPS_EVENT_TU);
			shtps_set_palmthresh(ts, SHTPS_LOS_SINGLE);
		}
		shtps_rezero_handle(ts, SHTPS_REZERO_HANDLE_EVENT_TOUCHUP, info->gs2);
	}
}


inline static void shtps_report_touch_on(struct shtps_rmi_spi *ts, int finger, int x, int y, int w, int wx, int wy, int z)
{
	int lcd_x = x * SHTPS_POS_SCALE_X(ts) / 10000;
	int lcd_y = y * SHTPS_POS_SCALE_Y(ts) / 10000;

	shtps_offset_pos(ts, &lcd_x, &lcd_y);

	input_mt_slot(ts->input, finger);
	input_mt_report_slot_state(ts->input, MT_TOOL_FINGER, true);
	input_report_abs(ts->input, ABS_MT_TOUCH_MAJOR, w);
	input_report_abs(ts->input, ABS_MT_POSITION_X,  lcd_x);
	input_report_abs(ts->input, ABS_MT_POSITION_Y,  lcd_y);
	input_report_abs(ts->input, ABS_MT_PRESSURE,    z);

	SHTPS_LOG_EVENT(
		printk(KERN_DEBUG "[shtps]Notify event[%d] touch=100(%d), x=%d(%d), y=%d(%d) w=%d(%d,%d)\n",
						finger, z, lcd_x, x, lcd_y, y, w, wx, wy);
	);
	_log_msg_sync( LOGMSG_ID__EVENT_NOTIFY, "%d|100|%d|%d|%d|%d|%d|%d|%d|%d",
						finger, lcd_x, x, lcd_y, y, w, wx, wy, z);
}

inline static void shtps_report_touch_off(struct shtps_rmi_spi *ts, int finger, int x, int y, int w, int wx, int wy, int z)
{
	input_mt_slot(ts->input, finger);
	input_mt_report_slot_state(ts->input, MT_TOOL_FINGER, false);

	SHTPS_LOG_EVENT(
		printk(KERN_DEBUG "[shtps]Notify event[%d] touch=100(%d), x=%d(%d), y=%d(%d) w=%d(%d,%d)\n",
						finger, z, (x * SHTPS_POS_SCALE_X(ts) / 10000), x, (y * SHTPS_POS_SCALE_Y(ts) / 10000), y,
						w, wx, wy);
	);
	_log_msg_sync( LOGMSG_ID__EVENT_NOTIFY, "%d|100|%d|%d|%d|%d|%d|%d|%d|%d",
						finger, (x * SHTPS_POS_SCALE_X(ts) / 10000), x, (y * SHTPS_POS_SCALE_Y(ts) / 10000), y,
						w, wx, wy, z);
}

#if defined(SHTPS_PEN_DETECT_ENABLE)
inline static void shtps_report_touch_pen_on(struct shtps_rmi_spi *ts, int pen, int x, int y, int w, int wx, int wy, int z)
{
	int lcd_x = x * SHTPS_POS_SCALE_X(ts) / 10000;
	int lcd_y = y * SHTPS_POS_SCALE_Y(ts) / 10000;

	shtps_offset_pos(ts, &lcd_x, &lcd_y);

	input_mt_slot(ts->input, pen);
	input_mt_report_slot_state(ts->input, MT_TOOL_PEN, true);
	input_report_abs(ts->input, ABS_MT_TOUCH_MAJOR, w);
	input_report_abs(ts->input, ABS_MT_POSITION_X,  lcd_x);
	input_report_abs(ts->input, ABS_MT_POSITION_Y,  lcd_y);
	input_report_abs(ts->input, ABS_MT_PRESSURE,    z);

	SHTPS_LOG_EVENT(
		printk(KERN_DEBUG "[shtps][pen]Notify event[%d] touch=100(%d), x=%d(%d), y=%d(%d) w=%d(%d,%d)\n",
						pen, z, lcd_x, x, lcd_y, y, w, wx, wy);
	);
}

inline static void shtps_report_touch_pen_off(struct shtps_rmi_spi *ts, int pen, int x, int y, int w, int wx, int wy, int z)
{
	input_mt_slot(ts->input, pen);
	input_mt_report_slot_state(ts->input, MT_TOOL_PEN, false);

	SHTPS_LOG_EVENT(
		printk(KERN_DEBUG "[shtps][pen]Notify event[%d] touch=100(%d), x=%d(%d), y=%d(%d) w=%d(%d,%d)\n",
						pen, z, (x * SHTPS_POS_SCALE_X(ts) / 10000), x, (y * SHTPS_POS_SCALE_Y(ts) / 10000), y,
						w, wx, wy);
	);
}
#endif /* SHTPS_PEN_DETECT_ENABLE */

static void shtps_event_report(struct shtps_rmi_spi *ts, struct shtps_touch_info *info, u8 event)
{
	int	i;
	u8 	fingerMax = shtps_get_fingermax(ts);

	SHTPS_LOG_FUNC_CALL();

	for(i = 0;i < fingerMax;i++){
		if(info->fingers[i].state == SHTPS_TOUCH_STATE_FINGER){
			shtps_report_touch_on(ts, i,
								  info->fingers[i].x,
								  info->fingers[i].y,
								  shtps_get_fingerwidth(ts, i, info),
								  info->fingers[i].wx,
								  info->fingers[i].wy,
								  info->fingers[i].z);

		#if defined(SHTPS_HOVER_DETECT_ENABLE)
			}else if(info->fingers[i].state == SHTPS_TOUCH_STATE_HOVER){
				shtps_report_touch_on(ts, i,
									  info->fingers[i].x,
									  info->fingers[i].y,
									  shtps_get_fingerwidth(ts, i, info),
									  info->fingers[i].wx,
									  info->fingers[i].wy,
									  0);
		#endif /* SHTPS_HOVER_DETECT_ENABLE */

		#if defined(SHTPS_PEN_DETECT_ENABLE)
			}else if(info->fingers[i].state == SHTPS_TOUCH_STATE_PEN){
				shtps_report_touch_pen_on(ts, i,
									  info->fingers[i].x,
									  info->fingers[i].y,
									  shtps_get_fingerwidth(ts, i, info),
									  info->fingers[i].wx,
									  info->fingers[i].wy,
									  info->fingers[i].z);
			}else if(ts->report_info.fingers[i].state == SHTPS_TOUCH_STATE_PEN){
				shtps_report_touch_pen_off(ts, i,
									  info->fingers[i].x,
									  info->fingers[i].y,
									  shtps_get_fingerwidth(ts, i, info),
									  info->fingers[i].wx,
									  info->fingers[i].wy,
									  info->fingers[i].z);
		#endif /* SHTPS_PEN_DETECT_ENABLE */

		}else if(ts->report_info.fingers[i].state != SHTPS_TOUCH_STATE_NO_TOUCH){
			shtps_report_touch_off(ts, i,
								  info->fingers[i].x,
								  info->fingers[i].y,
								  shtps_get_fingerwidth(ts, i, info),
								  info->fingers[i].wx,
								  info->fingers[i].wy,
								  info->fingers[i].z);
		}
	}
	shtps_performance_check(SHTPS_PERFORMANCE_CHECK_STATE_CONT);
	input_sync(ts->input);
	shtps_performance_check(SHTPS_PERFORMANCE_CHECK_STATE_CONT);

	ts->touch_state.numOfFingers = info->finger_num;

	ts->diag.event = 1;
	memcpy(&ts->report_info, info, sizeof(ts->report_info));
	wake_up_interruptible(&ts->diag.wait);
}

#if defined( CONFIG_SHTPS_SY3000_FACETOUCH_OFF_DETECT ) || defined(SHTPS_LPWG_MODE_ENABLE)
static void shtps_event_update(struct shtps_rmi_spi *ts, struct shtps_touch_info *info)
{
	int	i;
	u8	numOfFingers = 0;
	u8 	fingerMax = shtps_get_fingermax(ts);

	SHTPS_LOG_FUNC_CALL();

	for(i = 0;i < fingerMax;i++){
		if(info->fingers[i].state != SHTPS_TOUCH_STATE_NO_TOUCH){
			numOfFingers++;
		}
	}
	ts->touch_state.numOfFingers = numOfFingers;
}
#endif /* CONFIG_SHTPS_SY3000_FACETOUCH_OFF_DETECT || SHTPS_LPWG_MODE_ENABLE */

static void shtps_event_force_touchup(struct shtps_rmi_spi *ts)
{
	int	i;
	int isEvent = 0;
	u8 	fingerMax = shtps_get_fingermax(ts);

	SHTPS_LOG_FUNC_CALL();

	for(i = 0;i < fingerMax;i++){
		#if defined(SHTPS_PEN_DETECT_ENABLE)
			if(ts->report_info.fingers[i].state == SHTPS_TOUCH_STATE_PEN){
				shtps_report_touch_pen_off(ts, i,
									  ts->report_info.fingers[i].x,
									  ts->report_info.fingers[i].y,
									  0,
									  0,
									  0,
									  0);
			}else
		#endif /* SHTPS_PEN_DETECT_ENABLE */

		if(ts->report_info.fingers[i].state != 0x00){
			isEvent = 1;
			shtps_report_touch_off(ts, i,
								  ts->report_info.fingers[i].x,
								  ts->report_info.fingers[i].y,
								  0,
								  0,
								  0,
								  0);
		}
	}
	if(isEvent){
		input_sync(ts->input);
		ts->touch_state.numOfFingers = 0;
		memset(&ts->report_info, 0, sizeof(ts->report_info));
	}
}

#if defined( SHTPS_PHYSICAL_KEY_ENABLE )
static void shtps_key_event_report(struct shtps_rmi_spi *ts, u8 state)
{
	int isEvent = 0;

	SHTPS_LOG_FUNC_CALL();

	if(ts->key_state != state){
		if( ((ts->key_state & (1 << SHTPS_PHYSICAL_KEY_DOWN)) ^ (state & (1 << SHTPS_PHYSICAL_KEY_DOWN))) != 0 ){
			input_event(ts->input_key, EV_MSC, MSC_SCAN, SHTPS_PHYSICAL_KEY_DOWN);
			input_report_key(ts->input_key, ts->keycodes[SHTPS_PHYSICAL_KEY_DOWN], ((state >> SHTPS_PHYSICAL_KEY_DOWN) & 0x01));
			isEvent = 1;
		}
		if( ((ts->key_state & (1 << SHTPS_PHYSICAL_KEY_UP)) ^ (state & (1 << SHTPS_PHYSICAL_KEY_UP))) != 0 ){
			input_event(ts->input_key, EV_MSC, MSC_SCAN, SHTPS_PHYSICAL_KEY_UP);
			input_report_key(ts->input_key, ts->keycodes[SHTPS_PHYSICAL_KEY_UP], ((state >> SHTPS_PHYSICAL_KEY_UP) & 0x01));
			isEvent = 1;
		}
	}

	if(isEvent){
		input_sync(ts->input_key);
		ts->key_state = state;
		ts->diag.event_touchkey = 1;
		wake_up_interruptible(&ts->diag.wait);
	}
}

static void shtps_key_event_force_touchup(struct shtps_rmi_spi *ts)
{
	int isEvent = 0;

	SHTPS_LOG_FUNC_CALL();

	if(ts->key_state != 0){
		if( (ts->key_state & (1 << SHTPS_PHYSICAL_KEY_DOWN)) != 0 ){
			input_event(ts->input_key, EV_MSC, MSC_SCAN, SHTPS_PHYSICAL_KEY_DOWN);
			input_report_key(ts->input_key, ts->keycodes[SHTPS_PHYSICAL_KEY_DOWN], 0);
			isEvent = 1;
		}
		if( (ts->key_state & (1 << SHTPS_PHYSICAL_KEY_UP)) != 0 ){
			input_event(ts->input_key, EV_MSC, MSC_SCAN, SHTPS_PHYSICAL_KEY_UP);
			input_report_key(ts->input_key, ts->keycodes[SHTPS_PHYSICAL_KEY_UP], 0);
			isEvent = 1;
		}
	}

	if(isEvent){
		input_sync(ts->input_key);
	}

	ts->key_state = 0;
}

#if defined(SHTPS_LPWG_MODE_ENABLE)
static void shtps_notify_wakeup_event(struct shtps_rmi_spi *ts)
{
	SHTPS_LOG_FUNC_CALL();
	SHTPS_LOG_DBG_PRINT("wakeup touch blocked until rezero\n");
	ts->lpwg.block_touchevent = 1;
	ts->lpwg.is_notified = 1;
	ts->lpwg.notify_time = jiffies;
	ts->lpwg.notify_enable = 0;

	input_report_key(ts->input_key, ts->keycodes[SHTPS_PHYSICAL_KEY_SWEEPON], 1);
	input_sync(ts->input_key);

	input_report_key(ts->input_key, ts->keycodes[SHTPS_PHYSICAL_KEY_SWEEPON], 0);
	input_sync(ts->input_key);

	shtps_lpwg_notify_interval_start(ts);
}
#endif /* SHTPS_LPWG_MODE_ENABLE */

#endif /* SHTPS_PHYSICAL_KEY_ENABLE */

/* -----------------------------------------------------------------------------------
 */
static int shtps_tm_irqcheck(struct shtps_rmi_spi *ts)
{
	u8 buf[2];

	SHTPS_LOG_FUNC_CALL();
	shtps_rmi_read(ts, ts->map.fn01.dataBase, buf, 2);

	if((buf[1] & SHTPS_IRQ_ANALOG) != 0x00){
		return 1;
	}
	return 0;
}

static int shtps_tm_wait_attn(struct shtps_rmi_spi *ts)
{
	int rc;

	_log_msg_sync( LOGMSG_ID__FW_TESTMODE_ATTN_WAIT, "");
	rc = wait_event_interruptible_timeout(ts->diag.tm_wait_ack,
			ts->diag.tm_ack == 1 || ts->diag.tm_stop == 1,
			msecs_to_jiffies(SHTPS_FWTESTMODE_ACK_TMO));
	_log_msg_recv( LOGMSG_ID__FW_TESTMODE_ATTN, "");

#if defined( SHTPS_LOG_SEQ_ENABLE )
	if(rc == 0){
		_log_msg_sync( LOGMSG_ID__FW_TESTMODE_ATTN_TIMEOUT, "");
	}
#endif /* #if defined( SHTPS_LOG_SEQ_ENABLE ) */

	if(ts->diag.tm_ack == 0 || ts->diag.tm_stop == 1){
		return -1;
	}

	ts->diag.tm_ack = 0;
	return 0;
}

static void shtps_tm_wakeup(struct shtps_rmi_spi *ts)
{
	SHTPS_LOG_FUNC_CALL();
	ts->diag.tm_ack = 1;
	_log_msg_send( LOGMSG_ID__FW_TESTMODE_ATTN, "");
	wake_up_interruptible(&ts->diag.tm_wait_ack);
}

static void shtps_tm_cancel(struct shtps_rmi_spi *ts)
{
	SHTPS_LOG_FUNC_CALL();
	ts->diag.tm_stop = 1;
	_log_msg_sync( LOGMSG_ID__FW_TESTMODE_ATTN_CANCEL, "");
	_log_msg_send( LOGMSG_ID__FW_TESTMODE_ATTN, "");
	wake_up_interruptible(&ts->diag.tm_wait_ack);
}

static int shtps_get_tm_rxsize(struct shtps_rmi_spi *ts)
{
	int receive_num = 0;

	if(ts->map.fn05.enable != 0){
		receive_num = F05_QUERY_NUMOFRCVEL(ts->map.fn05.query.data);
	}else{
		receive_num = F54_QUERY_NUMOFRCVEL(ts->map.fn54.query.data);
	}

	return (receive_num > SHTPS_TM_TXNUM_MAX)? SHTPS_TM_TXNUM_MAX : receive_num;
}

static int shtps_get_tm_txsize(struct shtps_rmi_spi *ts)
{
	int trans_num = 0;

	if(ts->map.fn05.enable != 0){
		trans_num   = F05_QUERY_NUMOFTRANSEL(ts->map.fn05.query.data);
	}else{
		trans_num   = F54_QUERY_NUMOFTRANSEL(ts->map.fn54.query.data);
	}

	return (trans_num > SHTPS_TM_RXNUM_MAX)? SHTPS_TM_RXNUM_MAX : trans_num;
}

static int shtps_start_tm(struct shtps_rmi_spi *ts)
{
	SHTPS_LOG_FUNC_CALL();

	_log_msg_sync( LOGMSG_ID__FW_TESTMODE_START, "%d", ts->diag.tm_mode);
	ts->diag.tm_stop = 0;
	ts->diag.tm_ack = 0;
	if(ts->diag.tm_mode == SHTPS_FWTESTMODE_V01){
		shtps_rmi_write(ts, ts->map.fn01.ctrlBase,      0x04);
		shtps_rmi_write(ts, ts->map.fn01.ctrlBase + 1,  0x00);
#if !defined( SHTPS_SY_REGMAP_BASE3 )
		if(F11_QUERY_HASGESTURES(ts->map.fn11.query.data)){
			shtps_rmi_write(ts, ts->map.fn11.ctrlBase + 10, 0x00);
			if(F11_QUERY_HASGESTURE1(ts->map.fn11.query.data)){
				shtps_rmi_write(ts, ts->map.fn11.ctrlBase + 11, 0x00);
			}
		}
#endif /* #if !defined( SHTPS_SY_REGMAP_BASE3 ) */
		shtps_rmi_write(ts, ts->map.fn01.dataBase,      0x42);
		shtps_rmi_write(ts, ts->map.fn01.dataBase,      0xe1);

		shtps_rmi_write(ts, 0xff, 0x80);
		shtps_rmi_write(ts, 0x00, 0x01);

	}else{
		shtps_rmi_write(ts, ts->map.fn01.ctrlBase,      0x04);
		shtps_rmi_write(ts, ts->map.fn01.ctrlBase + 1,  SHTPS_IRQ_ANALOG);
#if !defined( SHTPS_SY_REGMAP_BASE3 )
		if(F11_QUERY_HASGESTURES(ts->map.fn11.query.data)){
			shtps_rmi_write(ts, ts->map.fn11.ctrlBase + 10, 0x00);
			if(F11_QUERY_HASGESTURE1(ts->map.fn11.query.data)){
				shtps_rmi_write(ts, ts->map.fn11.ctrlBase + 11, 0x00);
			}
		}
#endif /* #if !defined( SHTPS_SY_REGMAP_BASE3 ) */
		shtps_rmi_write(ts, 0xff, 0x01);
	}
	return 0;
}

static void shtps_stop_tm(struct shtps_rmi_spi *ts)
{
	SHTPS_LOG_FUNC_CALL();

	_log_msg_sync( LOGMSG_ID__FW_TESTMODE_STOP, "%d", ts->diag.tm_mode);
	shtps_tm_cancel(ts);
	shtps_rmi_write(ts, 0xff, 0x00);
	shtps_init_param(ts);
}

static void shtps_read_tmdata(struct shtps_rmi_spi *ts, u8 mode)
{
	int i;
	int j;
	int receive_num = shtps_get_tm_rxsize(ts);
	int trans_num   = shtps_get_tm_txsize(ts);
	u8  tmp[SHTPS_TM_TXNUM_MAX * 2];
	u16 index = 0;

	SHTPS_LOG_FUNC_CALL();

	_log_msg_sync( LOGMSG_ID__FW_TESTMODE_GETDATA, "%d", ts->diag.tm_mode);

	if(ts->diag.tm_mode == SHTPS_FWTESTMODE_V01){
		mutex_lock(&shtps_ctrl_lock);
		if(mode == SHTPS_TMMODE_FRAMELINE){
			for(i = 0;i < trans_num;i++){
				shtps_rmi_write(ts, 0x0201, (0x80 | (i & 0x3f)));
				shtps_rmi_read(ts, 0x0202, tmp, receive_num);

				for(j = 0;j < receive_num;j++){
					ts->diag.tm_data[(j * trans_num) + i] = tmp[j];
				}
			}
		}else if(mode == SHTPS_TMMODE_BASELINE){
			for(i = 0;i < trans_num;i++){
				shtps_rmi_write(ts, 0x0201, (0x40 | (i & 0x3f)));
				shtps_rmi_read(ts, 0x0202, tmp, receive_num * 2);

				for(j = 0;j < receive_num;j++){
					ts->diag.tm_data[(j * trans_num * 2) + (i * 2)]     = tmp[j * 2];
					ts->diag.tm_data[(j * trans_num * 2) + (i * 2) + 1] = tmp[j * 2 + 1];
				}
			}
		}
		mutex_unlock(&shtps_ctrl_lock);
	}else if(ts->diag.tm_mode == SHTPS_FWTESTMODE_V02){
		if(mode == SHTPS_TMMODE_FRAMELINE){
			mutex_lock(&shtps_ctrl_lock);
			shtps_rmi_write(ts, ts->map.fn05.dataBase + 1, 0x80);
			shtps_rmi_write(ts, ts->map.fn05.commandBase, 0x04);
			mutex_unlock(&shtps_ctrl_lock);
			if(shtps_tm_wait_attn(ts) != 0) goto tm_read_cancel;

			mutex_lock(&shtps_ctrl_lock);
			for(i = 0;i < trans_num;i++){
				shtps_rmi_write(ts, ts->map.fn05.dataBase + 1, (0x80 | (i & 0x3f)));
				shtps_rmi_read(ts, ts->map.fn05.dataBase + 2, tmp, receive_num);
				for(j = 0;j < receive_num;j++){
					ts->diag.tm_data[(j * trans_num) + i] = tmp[j];
				}
			}
			mutex_unlock(&shtps_ctrl_lock);
		}else if(mode == SHTPS_TMMODE_BASELINE){
			mutex_lock(&shtps_ctrl_lock);
			shtps_rmi_write(ts, ts->map.fn05.dataBase + 1, 0x40);
			shtps_rmi_write(ts, ts->map.fn05.commandBase, 0x04);
			mutex_unlock(&shtps_ctrl_lock);
			if(shtps_tm_wait_attn(ts) != 0) goto tm_read_cancel;

			mutex_lock(&shtps_ctrl_lock);
			for(i = 0;i < trans_num;i++){
				shtps_rmi_write(ts, ts->map.fn05.dataBase + 1, (0x40 | (i & 0x3f)));
				shtps_rmi_read(ts, ts->map.fn05.dataBase + 2, tmp, receive_num * 2);
				for(j = 0;j < receive_num;j++){
					ts->diag.tm_data[(j * trans_num * 2) + (i * 2)]     = tmp[j * 2];
					ts->diag.tm_data[(j * trans_num * 2) + (i * 2) + 1] = tmp[j * 2 + 1];
				}
			}
			mutex_unlock(&shtps_ctrl_lock);
		}
	}else if(ts->diag.tm_mode == SHTPS_FWTESTMODE_V03){
		if(mode == SHTPS_TMMODE_FRAMELINE){
			mutex_lock(&shtps_ctrl_lock);
			shtps_rmi_write(ts, ts->map.fn54.dataBase, 0x02);
			shtps_rmi_write(ts, ts->map.fn54.commandBase, 0x01);
			mutex_unlock(&shtps_ctrl_lock);
			if(shtps_tm_wait_attn(ts) != 0) goto tm_read_cancel;

			mutex_lock(&shtps_ctrl_lock);
			for(i = 0;i < trans_num;i++){
				for(j = 0;j < (receive_num * 2);j++){
					shtps_rmi_write(ts, ts->map.fn54.dataBase + 1, (index & 0xFF));
					shtps_rmi_write(ts, ts->map.fn54.dataBase + 2, ((index >> 0x08) & 0xFF));

					shtps_rmi_read(ts, ts->map.fn54.dataBase + 3, &tmp[j], 1);
					index++;
				}

				for(j = 0;j < receive_num;j++){
					ts->diag.tm_data[(j * trans_num * 2) + (i * 2)]     = tmp[j * 2];
					ts->diag.tm_data[(j * trans_num * 2) + (i * 2) + 1] = tmp[(j * 2) + 1];
				}
			}
			mutex_unlock(&shtps_ctrl_lock);
		}else if(mode == SHTPS_TMMODE_BASELINE){
			mutex_lock(&shtps_ctrl_lock);
			shtps_rmi_write(ts, ts->map.fn54.dataBase, 0x03);
			shtps_rmi_write(ts, ts->map.fn54.commandBase, 0x01);
			mutex_unlock(&shtps_ctrl_lock);
			if(shtps_tm_wait_attn(ts) != 0) goto tm_read_cancel;

			mutex_lock(&shtps_ctrl_lock);
			for(i = 0;i < trans_num;i++){
				for(j = 0;j < (receive_num * 2);j++){
					shtps_rmi_write(ts, ts->map.fn54.dataBase + 1, (index & 0xFF));
					shtps_rmi_write(ts, ts->map.fn54.dataBase + 2, ((index >> 0x08) & 0xFF));

					shtps_rmi_read(ts, ts->map.fn54.dataBase + 3, &tmp[j], 1);
					index++;
				}

				for(j = 0;j < receive_num;j++){
					ts->diag.tm_data[(j * trans_num * 2) + (i * 2)]     = tmp[j * 2];
					ts->diag.tm_data[(j * trans_num * 2) + (i * 2) + 1] = tmp[(j * 2) + 1];
				}
			}
			mutex_unlock(&shtps_ctrl_lock);
		}
	}else{
		memset(ts->diag.tm_data, 0, sizeof(ts->diag.tm_data));
	}

	return;

tm_read_cancel:
	memset(ts->diag.tm_data, 0, sizeof(ts->diag.tm_data));
	return;
}

/* -----------------------------------------------------------------------------------
 */
static void shtps_irq_wake_disable(struct shtps_rmi_spi *ts)
{
	SHTPS_LOG_FUNC_CALL();
	_log_msg_sync( LOGMSG_ID__IRQ_WAKE_DISABLE, "%d", ts->irq_mgr.wake);
	if(ts->irq_mgr.wake != SHTPS_IRQ_WAKE_DISABLE){
		disable_irq_wake(ts->irq_mgr.irq);
		ts->irq_mgr.wake = SHTPS_IRQ_WAKE_DISABLE;

		#if defined( SHTPS_MODULE_PARAM_ENABLE )
			shtps_irq_wake_state = SHTPS_IRQ_WAKE_DISABLE;
		#endif /* SHTPS_MODULE_PARAM_ENABLE */
	}
}

static void shtps_irq_wake_enable(struct shtps_rmi_spi *ts)
{
	SHTPS_LOG_FUNC_CALL();
	_log_msg_sync( LOGMSG_ID__IRQ_WAKE_ENABLE, "%d", ts->irq_mgr.wake);
	if(ts->irq_mgr.wake != SHTPS_IRQ_WAKE_ENABLE){
		enable_irq_wake(ts->irq_mgr.irq);
		ts->irq_mgr.wake = SHTPS_IRQ_WAKE_ENABLE;

		#if defined( SHTPS_MODULE_PARAM_ENABLE )
			shtps_irq_wake_state = SHTPS_IRQ_WAKE_ENABLE;
		#endif /* SHTPS_MODULE_PARAM_ENABLE */
	}
}

static void shtps_irq_disable(struct shtps_rmi_spi *ts)
{
	SHTPS_LOG_FUNC_CALL();
	_log_msg_sync( LOGMSG_ID__IRQ_DISABLE, "%d", ts->irq_mgr.state);
	if(ts->irq_mgr.state != SHTPS_IRQ_STATE_DISABLE){
		disable_irq_nosync(ts->irq_mgr.irq);
		ts->irq_mgr.state = SHTPS_IRQ_STATE_DISABLE;
	}
}

static void shtps_irq_enable(struct shtps_rmi_spi *ts)
{
	SHTPS_LOG_FUNC_CALL();
	_log_msg_sync( LOGMSG_ID__IRQ_ENABLE, "%d", ts->irq_mgr.state);
	if(ts->irq_mgr.state != SHTPS_IRQ_STATE_ENABLE){
		enable_irq(ts->irq_mgr.irq);
		ts->irq_mgr.state = SHTPS_IRQ_STATE_ENABLE;
	}
}

static int shtps_irq_resuest(struct shtps_rmi_spi *ts)
{
	int rc;

	SHTPS_LOG_FUNC_CALL();

	_log_msg_sync( LOGMSG_ID__IRQ_REQUEST, "%d", ts->irq_mgr.irq);

	#if defined(SHTPS_PERFORMANCE_CHECK_ENABLE)
		rc = request_threaded_irq(ts->irq_mgr.irq,
								  shtps_irq_handler,
								  shtps_irq,
								  IRQF_TRIGGER_FALLING | IRQF_ONESHOT,
								  SH_TOUCH_DEVNAME,
								  ts);
	#else
		rc = request_threaded_irq(ts->irq_mgr.irq,
								  NULL,
								  shtps_irq,
								  IRQF_TRIGGER_FALLING | IRQF_ONESHOT,
								  SH_TOUCH_DEVNAME,
								  ts);
	#endif /* SHTPS_PERFORMANCE_CHECK_ENABLE */

	if(rc){
		_log_msg_sync( LOGMSG_ID__IRQ_REQUEST_NACK, "");
		SHTPS_LOG_ERR_PRINT("request_threaded_irq error:%d\n",rc);
		return -1;
	}

	ts->irq_mgr.state = SHTPS_IRQ_STATE_ENABLE;
	ts->irq_mgr.wake  = SHTPS_IRQ_WAKE_DISABLE;
	shtps_irq_disable(ts);
	return 0;
}

static void shtps_irqtimer_start(struct shtps_rmi_spi *ts, long time_ms)
{
	SHTPS_LOG_FUNC_CALL();
	_log_msg_sync( LOGMSG_ID__IRQ_TIMER_START, "%lu", time_ms);
	schedule_delayed_work(&ts->tmo_check, msecs_to_jiffies(time_ms));
}

static void shtps_irqtimer_stop(struct shtps_rmi_spi *ts)
{
	SHTPS_LOG_FUNC_CALL();
	_log_msg_sync( LOGMSG_ID__IRQ_TIMER_CANCEL, "");
	cancel_delayed_work(&ts->tmo_check);
}

#if defined(SHTPS_LPWG_MODE_ENABLE)
static void shtps_read_touchevent_insleep(struct shtps_rmi_spi *ts, int state)
{
	int status = 0;
	u8 buf[2];
	u8 val;

	memset(buf, 0, sizeof(buf));

	shtps_device_access_setup(ts);

	status = shtps_rmi_read(ts, ts->map.fn01.dataBase, buf, 2);
	shtps_rmi_read(ts, 0x0058, &val, 1);
	SHTPS_LOG_DBG_PRINT("LPWG detect [status = 0x%02X]\n", val);

	if((val & SHTPS_LPWG_DOZE_CTL) != 0){
		if(ts->lpwg.notify_enable != 0){
			#if defined(SHTPS_PROXIMITY_SUPPORT_ENABLE)
			{
				if(SHTPS_LPWG_PROXIMITY_SUPPORT_ENABLE != 0){
					if (shtps_proximity_state_check(ts) == 0) {
						if( shtps_proximity_check(ts) != SHTPS_PROXIMITY_NEAR ){
							shtps_notify_wakeup_event(ts);
						}else{
							SHTPS_LOG_DBG_PRINT("[LPWG] proximity near\n");
						}
					}else{
						SHTPS_LOG_DBG_PRINT("[LPWG] proximity power_on + near\n");
					}
				}
				else{
					shtps_notify_wakeup_event(ts);
				}
			}
			#else
				shtps_notify_wakeup_event(ts);
			#endif /* SHTPS_PROXIMITY_SUPPORT_ENABLE */
		}
		else{
			SHTPS_LOG_DBG_PRINT("[LPWG] notify event blocked\n");
		}
	}else{
	    SHTPS_LOG_DBG_PRINT("[LPWG] unknown interrupt source detection!! [0x%02X]\n", val);\
	}

	shtps_device_access_teardown(ts);
}
#endif /*  SHTPS_LPWG_MODE_ENABLE */

static void shtps_read_touchevent(struct shtps_rmi_spi *ts, int state)
{
	u8 buf[2 + SHTPS_FINGER_MAX * 5 + (SHTPS_FINGER_MAX / 4) + 4];
	u8 event;
	u8 finger;
	u8 fingerMax = shtps_get_fingermax(ts);
	u8 offset, readsize;
	u8 *pFingerInfoBuf;
	struct shtps_touch_info info;

	SHTPS_LOG_FUNC_CALL();
	_log_msg_sync( LOGMSG_ID__READ_EVENT, "%d", state);

	shtps_device_access_setup(ts);

#if defined( SHTPS_VKEY_INVALID_AREA_ENABLE )
	ts->invalid_area_touch = 0;
#endif /* #if defined( SHTPS_VKEY_INVALID_AREA_ENABLE ) */

	if(fingerMax > 8){
		offset = 5;
		readsize = 3 + 5 * SHTPS_SY3X00_FINGER_SPIREAD_CNT;
	}else if (fingerMax > 4){
		offset = 4;
		readsize = 2 + 5 * SHTPS_SY3X00_FINGER_SPIREAD_CNT;
	}else{
		offset = 3;
		readsize = 1 + 5 * SHTPS_SY3X00_FINGER_SPIREAD_CNT;
	}

	memset(buf, 0, sizeof(buf));

	shtps_performance_check(SHTPS_PERFORMANCE_CHECK_STATE_CONT);

	shtps_rmi_read(ts, ts->map.fn01.dataBase, buf, 2);
	shtps_rmi_read(ts, ts->map.fn11.dataBase, &buf[2], readsize);

	#if	defined(SHTPS_MAXFINGER_5_ENABLE)
		buf[3] &= 0x03;
		buf[4] = 0;
	#endif

	ts->finger_state[0] = buf[2];
	ts->finger_state[1] = buf[3];
	ts->finger_state[2] = buf[4];

	pFingerInfoBuf = &buf[offset];
	for(finger = 0;finger < fingerMax;finger++){
		if(((buf[2 + finger / 4] >> (2 * (finger % 4))) & 0x03) != SHTPS_TOUCH_STATE_NO_TOUCH ||
		   (ts->report_info.fingers[finger].state != SHTPS_TOUCH_STATE_NO_TOUCH))
		{
			#if	defined( SHTPS_SPI_BLOCKREAD_ENABLE )
				#if	(SHTPS_SY3X00_FINGER_SPIREAD_CNT != 5)
					if(finger >= SHTPS_SY3X00_FINGER_SPIREAD_CNT){
						shtps_rmi_read(ts,
							   ts->map.fn01.dataBase + offset + (finger * 5),
							   &pFingerInfoBuf[finger * 5], 5);
					}
				#endif	/* (SHTPS_SY3X00_FINGER_SPIREAD_CNT != 5) */
			#else
				shtps_rmi_read(ts,
						   ts->map.fn01.dataBase + offset + (finger * 5),
						   &pFingerInfoBuf[finger * 5], 5);
			#endif

			#if defined(SHTPS_PEN_DETECT_ENABLE)
				if(((buf[2 + finger / 4] >> (2 * (finger % 4))) & 0x03) == SHTPS_TOUCH_STATE_PEN)
				{
					shtps_rmi_read(ts,
									ts->map.fn01.dataBase + offset + (fingerMax * 5) + 3,
									&pFingerInfoBuf[(finger * 5) + 4], 1);
				}
			#endif /* SHTPS_PEN_DETECT_ENABLE */

			#if defined( SHTPS_VKEY_INVALID_AREA_ENABLE )
			{
				int x, y, idx;
				x = (((pFingerInfoBuf[(finger * 5) + 0] << 0x04) & 0x0FF0) | (pFingerInfoBuf[(finger * 5) + 2] & 0x0F));
				y = (((pFingerInfoBuf[(finger * 5) + 1] << 0x04) & 0x0FF0) | (pFingerInfoBuf[(finger * 5) + 2] >> 0x04 & 0x0F));
				x = x * SHTPS_POS_SCALE_X(ts) / 10000;
				y = y * SHTPS_POS_SCALE_Y(ts) / 10000;
				for(idx = 0;shtps_invalid_area[idx].sx != -1;idx++){
					if(shtps_invalid_area[idx].sx <= x && x <= shtps_invalid_area[idx].ex &&
					   shtps_invalid_area[idx].sy <= y && y <= shtps_invalid_area[idx].ey)
					{
						_log_msg_sync( LOGMSG_ID__INVALID_AREA, "%d|%d|%d", finger, x, y);
						buf[2 + finger / 4] &= ~(0x03 << (2 * (finger % 4)));
						ts->invalid_area_touch = 1;
					}
				}
			}
			#endif /* #if defined( SHTPS_VKEY_INVALID_AREA_ENABLE ) */
		}
	}

#if defined( SHTPS_SY_REGMAP_BASE3 )
	#if defined( SHTPS_FMODE_GESTURE_ENABLE )
		shtps_rmi_read(ts, ts->map.fn01.dataBase + offset + fingerMax * 5,
									   &pFingerInfoBuf[fingerMax * 5], 1);
	#else
		;
	#endif
#else
	if(F11_QUERY_HASGESTURES(ts->map.fn11.query.data)){
		if(F11_QUERY_HASGESTURE1(ts->map.fn11.query.data) == 0){
			shtps_rmi_read(ts, ts->map.fn01.dataBase + offset + fingerMax * 5,
						   &pFingerInfoBuf[fingerMax * 5], 1);
		}else{
			shtps_rmi_read(ts, ts->map.fn01.dataBase + offset + fingerMax * 5,
						   &pFingerInfoBuf[fingerMax * 5], 2);
		}
	}
#endif /* #if defined( SHTPS_SY_REGMAP_BASE3 ) */

	shtps_performance_check(SHTPS_PERFORMANCE_CHECK_STATE_CONT);

	switch(state){
	case SHTPS_STATE_SLEEP:
	case SHTPS_STATE_IDLE:
		shtps_device_access_teardown(ts);
		break;

	case SHTPS_STATE_SLEEP_FACETOUCH:
#if defined( CONFIG_SHTPS_SY3000_FACETOUCH_OFF_DETECT )
		shtps_calc_notify(ts, &buf[2], &info, &event);
		shtps_event_update(ts, &info);
		if(event == SHTPS_EVENT_TU){
			shtps_notify_facetouchoff(ts, 1);
		}else{
			shtps_wake_unlock(ts);
		}
		if(ts->touch_state.numOfFingers == 0){
			shtps_device_access_teardown(ts);
		}
#endif /* #if defined( CONFIG_SHTPS_SY3000_FACETOUCH_OFF_DETECT ) */
		break;

	case SHTPS_STATE_FACETOUCH:
	case SHTPS_STATE_ACTIVE:
	default:
		#if defined( SHTPS_PHYSICAL_KEY_ENABLE )
			if((buf[1] & SHTPS_IRQ_BUTTON) != 0){
				u8 present_key_state = 0;

				shtps_rmi_read(ts, ts->map.fn1A.dataBase, &present_key_state, 1);
				shtps_key_event_report(ts, present_key_state);
			}
		#endif /* SHTPS_PHYSICAL_KEY_ENABLE */

		shtps_calc_notify(ts, &buf[2], &info, &event);
		if(event != 0xff){
			#if defined(SHTPS_LPWG_MODE_ENABLE)
				if(ts->lpwg.block_touchevent == 0){
					shtps_event_report(ts, &info, event);
				}else{
					u8 rezero_req = 0;

					SHTPS_LOG_DBG_PRINT("LPWG touch event blocked\n");
					shtps_event_update(ts, &info);

					if(time_after(jiffies,
						ts->lpwg.wakeup_time + msecs_to_jiffies(SHTPS_LPWG_BLOCK_TIME_MAX_MS)))
					{
						rezero_req = 1;
						SHTPS_LOG_DBG_PRINT("LPWG force rezero\n");
					}else if( (ts->lpwg.tu_rezero_req != 0) && (event == SHTPS_EVENT_TU) ){
						rezero_req = 1;
						SHTPS_LOG_DBG_PRINT("LPWG rezero by touch up\n");
					}

					if(rezero_req != 0){
						ts->touch_state.numOfFingers = 0;
						ts->lpwg.block_touchevent = 0;
						ts->lpwg.tu_rezero_req = 0;
						shtps_rezero_request(ts,
											 SHTPS_REZERO_REQUEST_REZERO,
											 SHTPS_REZERO_TRIGGER_WAKEUP);
					}
				}

			#else
				shtps_event_report(ts, &info, event);
			#endif /* SHTPS_LPWG_MODE_ENABLE */

			#if defined( CONFIG_SHTPS_SY3000_FACETOUCH_OFF_DETECT )
			if(SHTPS_STATE_FACETOUCH == state){
				if(event == SHTPS_EVENT_TU){
					shtps_notify_facetouchoff(ts, 0);
				}else{
					shtps_check_facetouch(ts, &info);
				}
			}
			#endif /* #if defined( CONFIG_SHTPS_SY3000_FACETOUCH_OFF_DETECT ) */
		}

		if(ts->touch_state.numOfFingers == 0){
			if((ts->finger_state[0] | ts->finger_state[1] | ts->finger_state[2]) == 0){
				shtps_device_access_teardown(ts);
			}
		}

		break;
	}
}

static void shtps_loader_irqclr(struct shtps_rmi_spi *ts)
{
	u8 buf[2];

	SHTPS_LOG_FUNC_CALL();
	shtps_rmi_read(ts, ts->map.fn01.dataBase, buf, 2);
	_log_msg_sync( LOGMSG_ID__BL_IRQCLR, "0x%02X", buf[1]);
}

/* -----------------------------------------------------------------------------------
 */
#if defined( SHTPS_BOOT_FWUPDATE_ENABLE )
static int shtps_boot_fwupdate_enable_check(struct shtps_rmi_spi *ts)
{
	#if 0
	{
		u8 buf;

		shtps_reset(ts);
		msleep(SHTPS_STARTUP_MIN_TIME);

		shtps_rmi_write(ts, 0xFF, 0x00);
		shtps_rmi_read(ts, 0xEE, &buf, 1);

		_log_msg_sync( LOGMSG_ID__CHECK_TOUCHPANEL, "%d", (buf != 0x00)? 1:0);

		if(buf == 0x00){
			return 0;
		}
	}
	#endif

	#if defined( SHTPS_CHECK_HWID_ENABLE )
	{
		sharp_smem_common_type *sharp_smem;
		unsigned long rev;

		sharp_smem = sh_smem_get_common_address();
		rev = sharp_smem->sh_hw_revision;

		switch( rev ){
			case	0x00:
			case	0x05:
				return 0;
			default:
				break;
		}
	}
	#endif

	#if defined(SHTPS_SYSTEM_BOOT_MODE_CHECK_ENABLE)
		if( (ts->system_boot_mode == SH_BOOT_O_C) || (ts->system_boot_mode == SH_BOOT_U_O_C) ){
			return 0;
		}
	#endif /* SHTPS_SYSTEM_BOOT_MODE_CHECK_ENABLE */

	return 1;
}

#if !defined(SHTPS_BOOT_FWUPDATE_FORCE_UPDATE)
static int shtps_fwup_flag_check(void)
{
	sharp_smem_common_type *smemdata = NULL;

	smemdata = sh_smem_get_common_address();
	if(smemdata != NULL){
		SHTPS_LOG_DBG_PRINT("shtps_fwup_flag : %s\n", smemdata->shtps_fwup_flag == 0 ? "off" : "on");
		if(smemdata->shtps_fwup_flag == 0){
			return 0;
		}else{
			return 1;
		}
	}

	return -1;
}
#endif /* #if !defined(SHTPS_BOOT_FWUPDATE_FORCE_UPDATE) */

static void shtps_fwup_flag_clear(void)
{
	sharp_smem_common_type *smemdata = NULL;

	smemdata = sh_smem_get_common_address();
	if(smemdata != NULL){
		smemdata->shtps_fwup_flag = 0;
	}
}
#endif /* #if defined( SHTPS_BOOT_FWUPDATE_ENABLE ) */

static int shtps_fwupdate_enable(struct shtps_rmi_spi *ts)
{
#if defined( SHTPS_FWUPDATE_DISABLE )
	return 0;
#else
	return 1;
#endif /* #if defined( SHTPS_FWUPDATE_DISABLE ) */
}

static int shtps_loader_wait_attn(struct shtps_rmi_spi *ts)
{
	int rc;

	_log_msg_sync( LOGMSG_ID__BL_ATTN_WAIT, "");
	rc = wait_event_interruptible_timeout(ts->loader.wait_ack,
			ts->loader.ack == 1,
			msecs_to_jiffies(SHTPS_BOOTLOADER_ACK_TMO));

	_log_msg_recv( LOGMSG_ID__BL_ATTN, "");

	if(0 == rc && 0 == ts->loader.ack){
		_log_msg_sync( LOGMSG_ID__BL_ATTN_ERROR, "");
		return -1;
	}

	if(0 == rc){
		_log_msg_sync( LOGMSG_ID__BL_ATTN_TIMEOUT, "");
		SHTPS_LOG_ERR_PRINT("shtps_loader_wait_attn() warning rc = %d\n", rc);
	}

	ts->loader.ack = 0;
	return 0;
}

static void shtps_loader_wakeup(struct shtps_rmi_spi *ts)
{
	SHTPS_LOG_FUNC_CALL();
	ts->loader.ack = 1;
	_log_msg_send( LOGMSG_ID__BL_ATTN, "");
	wake_up_interruptible(&ts->loader.wait_ack);
}

static int shtps_loader_cmd(struct shtps_rmi_spi *ts, u8 cmd, u8 isLockdown)
{
	int rc;
	u8  buf;
	u16 blockSize;

	_log_msg_sync( LOGMSG_ID__BL_COMMAND, "0x%02X|%d", cmd, isLockdown);
	if(isLockdown){
		rc = shtps_rmi_write(ts, ts->map.fn34.dataBase + 2,
						 	 F34_QUERY_BOOTLOADERID0(ts->map.fn34.query.data));
		SPI_ERR_CHECK(rc, err_exit);

		rc = shtps_rmi_write(ts, ts->map.fn34.dataBase + 3,
				 			 F34_QUERY_BOOTLOADERID1(ts->map.fn34.query.data));
		SPI_ERR_CHECK(rc, err_exit);
	}
	blockSize = F34_QUERY_BLOCKSIZE(ts->map.fn34.query.data);

	rc = shtps_rmi_read(ts, ts->map.fn34.dataBase + 2 + blockSize, &buf, 1);
	SPI_ERR_CHECK(rc, err_exit);

	rc = shtps_rmi_write(ts, ts->map.fn34.dataBase + 2 + blockSize, (buf & 0xF0) | (cmd & 0x0F));
	SPI_ERR_CHECK(rc, err_exit);

	return 0;

err_exit:
	return rc;
}

static int shtps_enter_bootloader(struct shtps_rmi_spi *ts)
{
	int rc;
#if defined( SHTPS_BTLOADER_VER_ENABLE )
	u8 ver[2] = { 0x00, 0x00 };
#endif	/* #if defined( SHTPS_BTLOADER_VER_ENABLE ) */

	_log_msg_sync( LOGMSG_ID__BL_ENTER, "");

	mutex_lock(&shtps_loader_lock);

	ts->loader.ack = 0;

	request_event(ts, SHTPS_EVENT_STOP, 0);
	msleep(SHTPS_SLEEP_IN_WAIT_MS);
	if(request_event(ts, SHTPS_EVENT_STARTLOADER, 0) != 0){
		return -1;
	}
	shtps_wait_startup(ts);

	shtps_sleep(ts, 1);
	msleep(SHTPS_SLEEP_IN_WAIT_MS);

	shtps_loader_cmd(ts, 0x0F, 1);
	rc = shtps_loader_wait_attn(ts);
	if(rc){
		SHTPS_LOG_ERR_PRINT("shtps_enter_bootloader() mode change error\n");
		goto err_exit;
	}
	shtps_map_construct(ts);

#if defined( SHTPS_BTLOADER_VER_ENABLE )
	shtps_rmi_read(ts, 0x0066, ver, 2);
	if( (ver[0] == 0) && (ver[1] == 0) ){
		shtps_rmi_read(ts, 0x00A4, ver, 2);
	}
	ts->bt_ver = ((ver[0] << 0x08) & 0xff00) | ver[1];
#endif	/* #if defined( SHTPS_BTLOADER_VER_ENABLE ) */

	_log_msg_sync( LOGMSG_ID__BL_ENTER_DONE, "");
	SHTPS_LOG_DBG_PRINT("shtps_enter_bootloader() done\n");
	mutex_unlock(&shtps_loader_lock);
	return 0;

err_exit:
	_log_msg_sync( LOGMSG_ID__BL_ENTER_FAIL, "");
	mutex_unlock(&shtps_loader_lock);
	return -1;
}

static int shtps_exit_bootloader(struct shtps_rmi_spi *ts)
{
	u8  status;

	_log_msg_sync( LOGMSG_ID__BL_EXIT, "");
	shtps_rmi_write(ts, ts->map.fn01.commandBase, 0x01);
	msleep(SHTPS_RESET_BOOTLOADER_WAIT_MS);

	shtps_rmi_read(ts, ts->map.fn01.dataBase, &status, 1);
	request_event(ts, SHTPS_EVENT_STOP, 0);

	if((status & 0x40) != 0 || (status & 0x0F) == 4 || (status & 0x0F) == 5 || (status & 0x0F) == 6){
		SHTPS_LOG_DBG_PRINT("shtps_exit_bootloader() error status = 0x%02x\n", status);
		_log_msg_sync( LOGMSG_ID__BL_EXIT_FAIL, "0x%02X", status);
		return -1;
	}

	SHTPS_LOG_DBG_PRINT("shtps_exit_bootloader() done\n");
	_log_msg_sync( LOGMSG_ID__BL_EXIT_DONE, "");
	return 0;
}

static int shtps_lockdown_bootloader(struct shtps_rmi_spi *ts, u8* fwdata)
{
	return 0;
}

static int shtps_flash_erase(struct shtps_rmi_spi *ts)
{
	int rc;
	u8  status;
	u16 blockSize;

	if(!shtps_fwupdate_enable(ts)){
		return 0;
	}

	_log_msg_sync( LOGMSG_ID__BL_ERASE, "");

	mutex_lock(&shtps_loader_lock);

	shtps_loader_cmd(ts, 0x03, 1);
	msleep(SHTPS_FLASH_ERASE_WAIT_MS);
	rc = shtps_loader_wait_attn(ts);
	if(rc){
		SHTPS_LOG_ERR_PRINT("shtps_loader_wait_attn() err %d\n", rc);
		goto err_exit;
	}

	blockSize = F34_QUERY_BLOCKSIZE(ts->map.fn34.query.data);
	rc = shtps_rmi_read(ts, ts->map.fn34.dataBase + 2 + blockSize,
						&status, 1);
	SPI_ERR_CHECK((rc || status != 0x80), err_exit);

	rc = shtps_rmi_write(ts, ts->map.fn34.dataBase, 0);
	SPI_ERR_CHECK(rc, err_exit);
	rc = shtps_rmi_write(ts, ts->map.fn34.dataBase + 1, 0);
	SPI_ERR_CHECK(rc, err_exit);

	SHTPS_LOG_DBG_PRINT("shtps_flash_erase() done\n");
	_log_msg_sync( LOGMSG_ID__BL_ERASE_DONE, "");
	mutex_unlock(&shtps_loader_lock);
	return 0;

err_exit:
	_log_msg_sync( LOGMSG_ID__BL_ERASE_FAIL, "");
	mutex_unlock(&shtps_loader_lock);
	return -1;
}

static int shtps_flash_writeImage(struct shtps_rmi_spi *ts, u8 *fwdata)
{
	int rc;
	u16 blockNum;
	u16 blockSize;

	if(!shtps_fwupdate_enable(ts)){
		return 0;
	}

	_log_msg_sync( LOGMSG_ID__BL_WRITEIMAGE, "");

	mutex_lock(&shtps_loader_lock);

	blockNum  = F34_QUERY_FIRMBLOCKCOUNT(ts->map.fn34.query.data);
	blockSize = F34_QUERY_BLOCKSIZE(ts->map.fn34.query.data);

	#if defined( SHTPS_SPI_FWBLOCKWRITE_ENABLE )
		rc = shtps_rmi_write_fw_data(ts, ts->map.fn34.dataBase + 2, fwdata);
		SPI_ERR_CHECK(rc, err_exit);
	#else
		{
			int i;
			for(i = 0;i < blockSize;i++){
				rc = shtps_rmi_write(ts, ts->map.fn34.dataBase + 2 + i, fwdata[i]);
				SPI_ERR_CHECK(rc, err_exit);
			}
		}
	#endif /* #if defined( SHTPS_SPI_FWBLOCKWRITE_ENABLE ) */

	rc = shtps_loader_cmd(ts, 0x02, 0);
	SPI_ERR_CHECK(rc, err_exit);
	rc = shtps_loader_wait_attn(ts);
	if(rc){
		goto err_exit;
	}

	SHTPS_LOG_DBG_PRINT("shtps_flash_writeImage() done\n");
	_log_msg_sync( LOGMSG_ID__BL_WRITEIMAGE_DONE, "");
	mutex_unlock(&shtps_loader_lock);
	return 0;

err_exit:
	_log_msg_sync( LOGMSG_ID__BL_WRITEIMAGE_FAIL, "");
	mutex_unlock(&shtps_loader_lock);
	return -1;
}

static int shtps_flash_writeConfig(struct shtps_rmi_spi *ts, u8 *fwdata)
{
	int rc;
	int block;
	u16 blockNum;
	u16 blockSize;
	u8  status;

	if(!shtps_fwupdate_enable(ts)){
		return 0;
	}

	_log_msg_sync( LOGMSG_ID__BL_WRITECONFIG, "");

	mutex_lock(&shtps_loader_lock);

	blockNum  = F34_QUERY_CONFIGBLOCKCOUNT(ts->map.fn34.query.data);
	blockSize = F34_QUERY_BLOCKSIZE(ts->map.fn34.query.data);


	rc = shtps_rmi_write(ts, ts->map.fn34.dataBase, 0);
	SPI_ERR_CHECK(rc, err_exit);

	rc = shtps_rmi_write(ts, ts->map.fn34.dataBase + 1, 0);
	SPI_ERR_CHECK(rc, err_exit);

	for(block = 0;block < blockNum;block++){
		#if defined( SHTPS_SPI_FWBLOCKWRITE_ENABLE )
			rc = shtps_rmi_write_fw_data(ts, ts->map.fn34.dataBase + 2, &fwdata[block * blockSize]);
			SPI_ERR_CHECK(rc, err_exit);
		#else
			{
				int i;
				for(i = 0;i < blockSize;i++){
					rc = shtps_rmi_write(ts, ts->map.fn34.dataBase + 2 + i,
										 fwdata[(block * blockSize) + i]);
					SPI_ERR_CHECK(rc, err_exit);
				}
			}
		#endif /* #if defined( SHTPS_SPI_FWBLOCKWRITE_ENABLE ) */

		rc = shtps_loader_cmd(ts, 0x06, 0);
		SPI_ERR_CHECK(rc, err_exit);

		rc = shtps_loader_wait_attn(ts);
		SPI_ERR_CHECK(rc, err_exit);
		rc = shtps_rmi_read(ts, ts->map.fn34.dataBase + 2 + blockSize,
							&status, 1);
		SPI_ERR_CHECK((rc || status != 0x80), err_exit);
	}

	SHTPS_LOG_DBG_PRINT("shtps_flash_writeConfig() done\n");
	_log_msg_sync( LOGMSG_ID__BL_WRITECONFIG_DONE, "");
	rc = shtps_exit_bootloader(ts);
	SPI_ERR_CHECK(rc, err_exit);

	mutex_unlock(&shtps_loader_lock);
	return 0;

err_exit:
	_log_msg_sync( LOGMSG_ID__BL_WRITECONFIG_FAIL, "");
	mutex_unlock(&shtps_loader_lock);
	return -1;
}

/* -----------------------------------------------------------------------------------
 */
#if defined( SHTPS_CHECK_CRC_ERROR_ENABLE )
static void shtps_func_check_crc_error(struct shtps_rmi_spi *ts)
{
	u8 buf;
	const unsigned char* fw_data = NULL;
	u8 update = 0;

	if(shtps_start(ts) == 0){
		shtps_wait_startup(ts);
	}

	shtps_rmi_read(ts, 0x0013, &buf, 1);

	if((buf & 0x0F) == 4 || (buf & 0x0F) == 5 || (buf & 0x0F) == 6){
		SHTPS_LOG_ERR_PRINT("Touch panel CRC error detect\n");
		update = 1;
	}

	if(update != 0){
		fw_data = tps_fw_data;
		if(fw_data){
			int ret;
			int retry = 5;
			do{
				ret = shtps_fw_update(ts, fw_data);
				request_event(ts, SHTPS_EVENT_STOP, 0);
			}while(ret != 0 && (retry-- > 0));
		}
	}

	shtps_get_bt_ver(ts);
	if(shtps_start(ts) == 0){
		shtps_wait_startup(ts);
	}
}
#endif /* if defined( SHTPS_CHECK_CRC_ERROR_ENABLE ) */

#if defined( SHTPS_ASYNC_OPEN_ENABLE )
static void shtps_func_open(struct shtps_rmi_spi *ts)
{
	#if defined( SHTPS_BOOT_FWUPDATE_ENABLE )
		if( shtps_boot_fwupdate_enable_check(ts) != 0 ){
			u8 buf;
			const unsigned char* fw_data = NULL;
			int ver;
			u8 update = 0;

			if(shtps_start(ts) == 0){
				shtps_wait_startup(ts);
			}

			shtps_rmi_read(ts, 0x0013, &buf, 1);

			#if defined( SHTPS_BOOT_FWUPDATE_FORCE_UPDATE )
				ver = shtps_fwver(ts);
				printk(KERN_DEBUG "fw version = 0x%04x\n", ver);
				if(ver != SHTPS_FWVER_NEWER){
					update = 1;
				}
			#else
				if(shtps_fwup_flag_check() > 0){
					ver = shtps_fwver(ts);
					if(ver != SHTPS_FWVER_NEWER){
						update = 1;
					}
				}
			#endif /* if defined( SHTPS_BOOT_FWUPDATE_FORCE_UPDATE ) */

			if((buf & 0x0F) == 4 || (buf & 0x0F) == 5 || (buf & 0x0F) == 6){
				SHTPS_LOG_ERR_PRINT("Touch panel CRC error detect\n");
				update = 1;
			}

			if(update != 0){
				fw_data = tps_fw_data;
				if(fw_data){
					int ret;
					int retry = 5;
					do{
						ret = shtps_fw_update(ts, fw_data);
						request_event(ts, SHTPS_EVENT_STOP, 0);
					}while(ret != 0 && (retry-- > 0));
				}
			}
		}
		shtps_fwup_flag_clear();
	#endif /* #if defined( SHTPS_BOOT_FWUPDATE_ENABLE ) */

	shtps_get_bt_ver(ts);
	if(shtps_start(ts) == 0){
		shtps_wait_startup(ts);
	}
}

static void shtps_func_close(struct shtps_rmi_spi *ts)
{
	shtps_shutdown(ts);
}

static int shtps_func_enable(struct shtps_rmi_spi *ts)
{
	if(shtps_start(ts) != 0){
		return -EFAULT;
	}
	if(shtps_wait_startup(ts) != 0){
		return -EFAULT;
	}

	return 0;
}

static void shtps_func_disable(struct shtps_rmi_spi *ts)
{
	shtps_shutdown(ts);
}

static void shtps_func_request_async_complete(void *arg_p)
{
	kfree( arg_p );
}

static void shtps_func_request_async( struct shtps_rmi_spi *ts, int event)
{
	struct shtps_req_msg		*msg_p;
	unsigned long	flags;

	msg_p = (struct shtps_req_msg *)kzalloc( sizeof( struct shtps_req_msg ), GFP_KERNEL );
	if ( msg_p == NULL ){
		SHTPS_LOG_ERR_PRINT("Out of memory [event:%d]\n", event);
		return;
	}

	msg_p->complete = shtps_func_request_async_complete;
	msg_p->event = event;
	msg_p->context = msg_p;
	msg_p->status = -1;

	spin_lock_irqsave( &(ts->queue_lock), flags);
	list_add_tail( &(msg_p->queue), &(ts->queue) );
	spin_unlock_irqrestore( &(ts->queue_lock), flags);
	queue_work(ts->workqueue_p, &(ts->work_data) );
}

static void shtps_func_request_sync_complete(void *arg_p)
{
	complete( arg_p );
}

static int shtps_func_request_sync( struct shtps_rmi_spi *ts, int event)
{
	DECLARE_COMPLETION_ONSTACK(done);
	struct shtps_req_msg msg;
	unsigned long	flags;

	msg.complete = shtps_func_request_sync_complete;
	msg.event = event;
	msg.context = &done;
	msg.status = -1;

	spin_lock_irqsave( &(ts->queue_lock), flags);
	list_add_tail( &(msg.queue), &(ts->queue) );
	spin_unlock_irqrestore( &(ts->queue_lock), flags);
	queue_work(ts->workqueue_p, &(ts->work_data) );

	wait_for_completion(&done);

	return msg.status;
}

static void shtps_func_workq( struct work_struct *work_p )
{
	struct shtps_rmi_spi	*ts;
	unsigned long			flags;

	ts = container_of(work_p, struct shtps_rmi_spi, work_data);

	while( list_empty( &(ts->queue) ) == 0 ){
		spin_lock_irqsave( &(ts->queue_lock), flags );
		ts->cur_msg_p = list_entry( ts->queue.next, struct shtps_req_msg, queue);
		list_del_init( &(ts->cur_msg_p->queue) );
		spin_unlock_irqrestore( &(ts->queue_lock), flags );

		SHTPS_LOG_DBG_PRINT("FuncReq[%d] start\n", ts->cur_msg_p->event);

		switch(ts->cur_msg_p->event){
			case SHTPS_FUNC_REQ_EVEMT_OPEN:
				shtps_func_open(ts);
				ts->cur_msg_p->status = 0;
				break;

			case SHTPS_FUNC_REQ_EVEMT_CLOSE:
				shtps_func_close(ts);
				ts->cur_msg_p->status = 0;
				break;

			case SHTPS_FUNC_REQ_EVEMT_ENABLE:
				ts->cur_msg_p->status = shtps_func_enable(ts);
				break;

			case SHTPS_FUNC_REQ_EVEMT_DISABLE:
				shtps_func_disable(ts);
				ts->cur_msg_p->status = 0;
				break;

			#if defined( SHTPS_CHECK_CRC_ERROR_ENABLE )
			case SHTPS_FUNC_REQ_EVEMT_CHECK_CRC_ERROR:
				shtps_func_check_crc_error(ts);
				ts->cur_msg_p->status = 0;
				break;
			#endif /* #if defined( SHTPS_CHECK_CRC_ERROR_ENABLE ) */

			default:
				ts->cur_msg_p->status = -1;
				break;
		}

		SHTPS_LOG_DBG_PRINT("FuncReq[%d] end\n", ts->cur_msg_p->event);

		if( ts->cur_msg_p->complete ){
			ts->cur_msg_p->complete( ts->cur_msg_p->context );
		}
	}
}
#endif /* #if defined( SHTPS_ASYNC_OPEN_ENABLE ) */

/* -----------------------------------------------------------------------------------
 */
static int request_event(struct shtps_rmi_spi *ts, int event, int param)
{
	int ret;

	_log_msg_sync( LOGMSG_ID__REQUEST_EVENT, "%d|%d", event, ts->state_mgr.state);

	SHTPS_LOG_DBG_PRINT("event %d in state %d\n", event, ts->state_mgr.state);

	SHTPS_LOG_DBG_PRINT("mutex_lock()\n");
	mutex_lock(&shtps_ctrl_lock);

	switch(event){
	case SHTPS_EVENT_START:
		ret = state_func_tbl[ts->state_mgr.state]->start(ts, param);
		break;
	case SHTPS_EVENT_STOP:
		ret = state_func_tbl[ts->state_mgr.state]->stop(ts, param);
		break;
	case SHTPS_EVENT_SLEEP:
		ret = state_func_tbl[ts->state_mgr.state]->sleep(ts, param);
		break;
	case SHTPS_EVENT_WAKEUP:
		ret = state_func_tbl[ts->state_mgr.state]->wakeup(ts, param);
		break;
	case SHTPS_EVENT_STARTLOADER:
		ret = state_func_tbl[ts->state_mgr.state]->start_ldr(ts, param);
		break;
	case SHTPS_EVENT_STARTTM:
		ret = state_func_tbl[ts->state_mgr.state]->start_tm(ts, param);
		break;
	case SHTPS_EVENT_STOPTM:
		ret = state_func_tbl[ts->state_mgr.state]->stop_tm(ts, param);
		break;
	case SHTPS_EVENT_FACETOUCHMODE_ON:
		ret = state_func_tbl[ts->state_mgr.state]->facetouch_on(ts, param);
		break;
	case SHTPS_EVENT_FACETOUCHMODE_OFF:
		ret = state_func_tbl[ts->state_mgr.state]->facetouch_off(ts, param);
		break;
	case SHTPS_EVENT_GETSENSOR:
		ret = state_func_tbl[ts->state_mgr.state]->get_sensor(ts, param);
		break;
	case SHTPS_EVENT_INTERRUPT:
		ret = state_func_tbl[ts->state_mgr.state]->interrupt(ts, param);
		break;
	case SHTPS_EVENT_TIMEOUT:
		ret = state_func_tbl[ts->state_mgr.state]->timeout(ts, param);
		break;
	default:
		ret = -1;
		break;
	}

	SHTPS_LOG_DBG_PRINT("mutex_unlock()\n");
	mutex_unlock(&shtps_ctrl_lock);

	return ret;
}

static int state_change(struct shtps_rmi_spi *ts, int state)
{
	int ret = 0;
	int old_state = ts->state_mgr.state;

	_log_msg_sync( LOGMSG_ID__STATE_CHANGE, "%d|%d", ts->state_mgr.state, state);
	SHTPS_LOG_DBG_PRINT("state %d -> %d\n", ts->state_mgr.state, state);

	if(ts->state_mgr.state != state){
		ts->state_mgr.state = state;
		ret = state_func_tbl[ts->state_mgr.state]->enter(ts, old_state);
	}
	return ret;
}

/* -----------------------------------------------------------------------------------
 */
static int shtps_statef_nop(struct shtps_rmi_spi *ts, int param)
{
	_log_msg_sync( LOGMSG_ID__STATEF_NOP, "%d", ts->state_mgr.state);
	SHTPS_LOG_FUNC_CALL();
	return 0;
}

static int shtps_statef_cmn_error(struct shtps_rmi_spi *ts, int param)
{
	_log_msg_sync( LOGMSG_ID__STATEF_ERROR, "%d", ts->state_mgr.state);
	SHTPS_LOG_FUNC_CALL();
	return -1;
}

static int shtps_statef_cmn_stop(struct shtps_rmi_spi *ts, int param)
{
	_log_msg_sync( LOGMSG_ID__STATEF_STOP, "%d", ts->state_mgr.state);
	shtps_standby_param(ts);
	shtps_irq_disable(ts);
	state_change(ts, SHTPS_STATE_IDLE);
	return 0;
}

static int shtps_statef_cmn_facetouch_on(struct shtps_rmi_spi *ts, int param)
{
	shtps_set_facetouchmode(ts, 1);
	return 0;
}

static int shtps_statef_cmn_facetouch_off(struct shtps_rmi_spi *ts, int param)
{
	shtps_set_facetouchmode(ts, 0);
	return 0;
}

/* -----------------------------------------------------------------------------------
 */
static int shtps_statef_idle_start(struct shtps_rmi_spi *ts, int param)
{
	shtps_clr_startup_err(ts);
	shtps_reset(ts);
	shtps_irq_enable(ts);
	shtps_irqtimer_start(ts, 100);
	shtps_set_startmode(ts, SHTPS_MODE_NORMAL);
	state_change(ts, SHTPS_STATE_WAIT_WAKEUP);
	return 0;
}

static int shtps_statef_idle_start_ldr(struct shtps_rmi_spi *ts, int param)
{
	shtps_clr_startup_err(ts);
	shtps_reset(ts);
	shtps_irq_enable(ts);
	shtps_irqtimer_start(ts, 100);
	shtps_set_startmode(ts, SHTPS_MODE_LOADER);
	state_change(ts, SHTPS_STATE_WAIT_WAKEUP);
	return 0;
}

static int shtps_statef_idle_int(struct shtps_rmi_spi *ts, int param)
{
	shtps_read_touchevent(ts, SHTPS_STATE_IDLE);
	return 0;
}

/* -----------------------------------------------------------------------------------
 */
static int shtps_statef_waiwakeup_stop(struct shtps_rmi_spi *ts, int param)
{
	shtps_standby_param(ts);
	shtps_irq_disable(ts);
	shtps_irqtimer_stop(ts);
	state_change(ts, SHTPS_STATE_IDLE);
	return 0;
}

static int shtps_statef_waiwakeup_int(struct shtps_rmi_spi *ts, int param)
{
	shtps_reset_startuptime(ts);
	shtps_irqtimer_stop(ts);
	shtps_irqtimer_start(ts, 1000);
	state_change(ts, SHTPS_STATE_WAIT_READY);
	return 0;
}

static int shtps_statef_waiwakeup_tmo(struct shtps_rmi_spi *ts, int param)
{
	shtps_irqtimer_start(ts, 1000);
	state_change(ts, SHTPS_STATE_WAIT_READY);
	return 0;
}

/* -----------------------------------------------------------------------------------
 */
static int shtps_state_waiready_stop(struct shtps_rmi_spi *ts, int param)
{
	shtps_standby_param(ts);
	shtps_irq_disable(ts);
	shtps_irqtimer_stop(ts);
	state_change(ts, SHTPS_STATE_IDLE);
	return 0;
}

static int shtps_state_waiready_int(struct shtps_rmi_spi *ts, int param)
{
	unsigned long time;
	if((time = shtps_check_startuptime(ts)) != 0){
		SHTPS_LOG_DBG_PRINT("startup wait time : %lu\n", time);
		msleep(time);
	}

	shtps_irqtimer_stop(ts);

	if(shtps_map_construct(ts) != 0){
		SHTPS_LOG_ERR_PRINT("shtps_map_construct() error!!\n");
		shtps_statef_cmn_stop(ts, 0);
		shtps_notify_startup(ts, SHTPS_STARTUP_FAILED);
		return -1;
	}

	if(SHTPS_MODE_NORMAL == shtps_get_startmode(ts)){
		state_change(ts, SHTPS_STATE_ACTIVE);
	}else{
		state_change(ts, SHTPS_STATE_BOOTLOADER);
	}
	shtps_notify_startup(ts, SHTPS_STARTUP_SUCCESS);
	return 0;
}

static int shtps_state_waiready_tmo(struct shtps_rmi_spi *ts, int param)
{
	if(shtps_map_construct(ts) != 0){
		SHTPS_LOG_ERR_PRINT("shtps_map_construct() error!!\n");
		shtps_statef_cmn_stop(ts, 0);
		shtps_notify_startup(ts, SHTPS_STARTUP_FAILED);
		return -1;
	}

	if(SHTPS_MODE_NORMAL == shtps_get_startmode(ts)){
		state_change(ts, SHTPS_STATE_ACTIVE);
	}else{
		state_change(ts, SHTPS_STATE_BOOTLOADER);
	}
	shtps_notify_startup(ts, SHTPS_STARTUP_SUCCESS);
	return 0;
}

/* -----------------------------------------------------------------------------------
 */
static int shtps_statef_active_enter(struct shtps_rmi_spi *ts, int param)
{
	if(param == SHTPS_STATE_WAIT_READY){
		shtps_init_param(ts);
		if(ts->poll_info.boot_rezero_flag == 0){
			ts->poll_info.boot_rezero_flag = 1;
			shtps_rezero_request(ts,
								 SHTPS_REZERO_REQUEST_AUTOREZERO_ENABLE,
								 SHTPS_REZERO_TRIGGER_BOOT);
		}
		shtps_read_touchevent(ts, ts->state_mgr.state);
		shtps_irq_wake_enable(ts);
	}

	if(1 == shtps_get_facetouchmode(ts)){
		shtps_rezero_request(ts, SHTPS_REZERO_REQUEST_AUTOREZERO_DISABLE, 0);
		state_change(ts, SHTPS_STATE_FACETOUCH);
	}

	#if defined(SHTPS_SYSTEM_BOOT_MODE_CHECK_ENABLE)
		if( (ts->system_boot_mode == SH_BOOT_O_C) || (ts->system_boot_mode == SH_BOOT_U_O_C) ){
			state_change(ts, SHTPS_STATE_SLEEP);
		}
	#endif /* SHTPS_SYSTEM_BOOT_MODE_CHECK_ENABLE */

	return 0;
}

static int shtps_statef_active_stop(struct shtps_rmi_spi *ts, int param)
{
	shtps_irq_wake_disable(ts);
	return shtps_statef_cmn_stop(ts, param);
}

static int shtps_statef_active_sleep(struct shtps_rmi_spi *ts, int param)
{
	state_change(ts, SHTPS_STATE_SLEEP);
	return 0;
}

static int shtps_statef_active_starttm(struct shtps_rmi_spi *ts, int param)
{
	state_change(ts, SHTPS_STATE_FWTESTMODE);
	return 0;
}

static int shtps_statef_active_facetouch_on(struct shtps_rmi_spi *ts, int param)
{
	shtps_rezero_request(ts, SHTPS_REZERO_REQUEST_AUTOREZERO_DISABLE, 0);
	shtps_set_facetouchmode(ts, 1);
	state_change(ts, SHTPS_STATE_FACETOUCH);
	return 0;
}

static int shtps_statef_active_int(struct shtps_rmi_spi *ts, int param)
{
	shtps_read_touchevent(ts, SHTPS_STATE_ACTIVE);
	return 0;
}

/* -----------------------------------------------------------------------------------
 */
static int shtps_statef_loader_enter(struct shtps_rmi_spi *ts, int param)
{
	shtps_irq_wake_enable(ts);
	shtps_loader_irqclr(ts);
	return 0;
}

static int shtps_statef_loader_stop(struct shtps_rmi_spi *ts, int param)
{
	shtps_irq_wake_disable(ts);
	return shtps_statef_cmn_stop(ts, param);
}

static int shtps_statef_loader_int(struct shtps_rmi_spi *ts, int param)
{
	shtps_loader_irqclr(ts);
	shtps_loader_wakeup(ts);
	return 0;
}

/* -----------------------------------------------------------------------------------
 */
static int shtps_statef_facetouch_sleep(struct shtps_rmi_spi *ts, int param)
{
	state_change(ts, SHTPS_STATE_SLEEP_FACETOUCH);
	return 0;
}

static int shtps_statef_facetouch_starttm(struct shtps_rmi_spi *ts, int param)
{
	state_change(ts, SHTPS_STATE_FWTESTMODE);
	return 0;
}

static int shtps_statef_facetouch_facetouch_off(struct shtps_rmi_spi *ts, int param)
{
	shtps_rezero_request(ts,
						 SHTPS_REZERO_REQUEST_AUTOREZERO_ENABLE,
						 SHTPS_REZERO_TRIGGER_ENDCALL);
	shtps_set_facetouchmode(ts, 0);
	state_change(ts, SHTPS_STATE_ACTIVE);
	return 0;
}

static int shtps_statef_facetouch_int(struct shtps_rmi_spi *ts, int param)
{
	shtps_read_touchevent(ts, SHTPS_STATE_FACETOUCH);
	return 0;
}

/* -----------------------------------------------------------------------------------
 */
static int shtps_statef_fwtm_enter(struct shtps_rmi_spi *ts, int param)
{
	shtps_tm_irqcheck(ts);
	shtps_irq_wake_enable(ts);
	shtps_start_tm(ts);
	return 0;
}

static int shtps_statef_fwtm_stop(struct shtps_rmi_spi *ts, int param)
{
	shtps_irq_wake_disable(ts);
	return shtps_statef_cmn_stop(ts, param);
}

static int shtps_statef_fwtm_stoptm(struct shtps_rmi_spi *ts, int param)
{
	shtps_stop_tm(ts);
	state_change(ts, SHTPS_STATE_ACTIVE);
	return 0;
}

static int shtps_statef_fwtm_getsensor(struct shtps_rmi_spi *ts, int param)
{
	shtps_read_tmdata(ts, param);;
	return 0;
}

static int shtps_statef_fwtm_int(struct shtps_rmi_spi *ts, int param)
{
	if(shtps_tm_irqcheck(ts)){
		shtps_tm_wakeup(ts);
	}
	return 0;
}

/* -----------------------------------------------------------------------------------
 */
static int shtps_statef_sleep_enter(struct shtps_rmi_spi *ts, int param)
{
	#if defined( SHTPS_PHYSICAL_KEY_ENABLE )
		shtps_key_event_force_touchup(ts);
	#endif /* SHTPS_PHYSICAL_KEY_ENABLE */
	shtps_event_force_touchup(ts);
	shtps_irq_wake_disable(ts);

	#if defined( SHTPS_FAILSAFE_ENABLE )
		{
			u8 buf[2];
			shtps_rmi_read(ts, ts->map.fn01.dataBase, buf, 2);
		}
	#endif /* #if defined( SHTPS_FAILSAFE_ENABLE ) */

	shtps_sleep(ts, 1);

	#if defined(SHTPS_LPWG_MODE_ENABLE)
	if (ts->lpwg.lpwg_switch){
		shtps_irq_wake_enable(ts);
		shtps_set_lpwg_mode_on(ts);
	}
	ts->lpwg.tu_rezero_req = 0;
	ts->lpwg.block_touchevent = 0;
	#endif /* SHTPS_LPWG_MODE_ENABLE */
	return 0;
}

static int shtps_statef_sleep_wakeup(struct shtps_rmi_spi *ts, int param)
{
	shtps_irq_wake_enable(ts);
	#if defined(SHTPS_LPWG_MODE_ENABLE)
	if (ts->lpwg.lpwg_switch)
		ts->lpwg.wakeup_time = jiffies;
	shtps_set_lpwg_mode_off(ts);
	#endif /* SHTPS_LPWG_MODE_ENABLE */
	shtps_sleep(ts, 0);
	shtps_rezero_request(ts,
						 SHTPS_REZERO_REQUEST_WAKEUP_REZERO |
						 SHTPS_REZERO_REQUEST_AUTOREZERO_ENABLE,
						 SHTPS_REZERO_TRIGGER_WAKEUP);
	state_change(ts, SHTPS_STATE_ACTIVE);
	return 0;
}

static int shtps_statef_sleep_starttm(struct shtps_rmi_spi *ts, int param)
{
	shtps_sleep(ts, 0);
	msleep(25);
	state_change(ts, SHTPS_STATE_FWTESTMODE);
	return 0;
}

static int shtps_statef_sleep_facetouch_on(struct shtps_rmi_spi *ts, int param)
{
#if defined( CONFIG_SHTPS_SY3000_FACETOUCH_OFF_DETECT )
	shtps_irq_wake_enable(ts);
	shtps_sleep(ts, 0);
	shtps_rezero_request(ts, SHTPS_REZERO_REQUEST_AUTOREZERO_DISABLE, 0);
#endif /* #if defined( CONFIG_SHTPS_SY3000_FACETOUCH_OFF_DETECT ) */
	shtps_set_facetouchmode(ts, 1);
	state_change(ts, SHTPS_STATE_SLEEP_FACETOUCH);
	return 0;
}

static int shtps_statef_sleep_int(struct shtps_rmi_spi *ts, int param)
{
	#if defined(SHTPS_LPWG_MODE_ENABLE)
	if (ts->lpwg.lpwg_switch){
		shtps_read_touchevent_insleep(ts, SHTPS_STATE_SLEEP);
	}else{
		shtps_read_touchevent(ts, SHTPS_STATE_SLEEP);
	}
	#else
	shtps_read_touchevent(ts, SHTPS_STATE_SLEEP);
	#endif /* SHTPS_LPWG_MODE_ENABLE */
	return 0;
}

/* -----------------------------------------------------------------------------------
 */
static int shtps_statef_sleep_facetouch_enter(struct shtps_rmi_spi *ts, int param)
{
#if defined( CONFIG_SHTPS_SY3000_FACETOUCH_OFF_DETECT )
	return 0;
#else
	shtps_irq_wake_disable(ts);
	shtps_sleep(ts, 1);
	return 0;
#endif /* #if !defined( CONFIG_SHTPS_SY3000_FACETOUCH_OFF_DETECT ) */
}

static int shtps_statef_sleep_facetouch_stop(struct shtps_rmi_spi *ts, int param)
{
#if defined( CONFIG_SHTPS_SY3000_FACETOUCH_OFF_DETECT )
	shtps_wake_unlock(ts);
	shtps_irq_wake_disable(ts);
#endif /* #if defined( CONFIG_SHTPS_SY3000_FACETOUCH_OFF_DETECT ) */
	return shtps_statef_cmn_stop(ts, param);
}

static int shtps_statef_sleep_facetouch_wakeup(struct shtps_rmi_spi *ts, int param)
{
#if defined( CONFIG_SHTPS_SY3000_FACETOUCH_OFF_DETECT )
	shtps_wake_unlock(ts);
#else
	shtps_irq_wake_enable(ts);
	shtps_sleep(ts, 0);
#endif /* #if defined( CONFIG_SHTPS_SY3000_FACETOUCH_OFF_DETECT ) */
	shtps_rezero_request(ts, SHTPS_REZERO_REQUEST_WAKEUP_REZERO, 0);
	state_change(ts, SHTPS_STATE_FACETOUCH);
	return 0;
}

static int shtps_statef_sleep_facetouch_starttm(struct shtps_rmi_spi *ts, int param)
{
#if defined( CONFIG_SHTPS_SY3000_FACETOUCH_OFF_DETECT )
	shtps_wake_unlock(ts);
#else
	shtps_sleep(ts, 0);
#endif /* #if defined( CONFIG_SHTPS_SY3000_FACETOUCH_OFF_DETECT ) */
	state_change(ts, SHTPS_STATE_FWTESTMODE);
	return 0;
}

static int shtps_statef_sleep_facetouch_facetouch_off(struct shtps_rmi_spi *ts, int param)
{
#if defined( CONFIG_SHTPS_SY3000_FACETOUCH_OFF_DETECT )
	shtps_wake_unlock(ts);
	shtps_rezero_request(ts,
						 SHTPS_REZERO_REQUEST_AUTOREZERO_ENABLE,
						 SHTPS_REZERO_TRIGGER_ENDCALL);
	shtps_irq_wake_disable(ts);
#endif /* #if defined( CONFIG_SHTPS_SY3000_FACETOUCH_OFF_DETECT ) */
	shtps_set_facetouchmode(ts, 0);
	state_change(ts, SHTPS_STATE_SLEEP);
	return 0;
}

static int shtps_statef_sleep_facetouch_int(struct shtps_rmi_spi *ts, int param)
{
#if defined( CONFIG_SHTPS_SY3000_FACETOUCH_OFF_DETECT )
	if(param == 1){
		shtps_wake_lock(ts);
		ts->facetouch.wake_sig = 1;
		_log_msg_send( LOGMSG_ID__DETECT_FACETOUCH, "");
		wake_up_interruptible(&ts->facetouch.wait_off);
		shtps_read_touchevent(ts, SHTPS_STATE_SLEEP_FACETOUCH);
		ts->facetouch.wake_sig = 0;
	}
#else
	shtps_read_touchevent(ts, SHTPS_STATE_SLEEP);
#endif /* #if defined( CONFIG_SHTPS_SY3000_FACETOUCH_OFF_DETECT ) */

	return 0;
}

/* -----------------------------------------------------------------------------------
 */
static int shtps_ioctl_enable(struct shtps_rmi_spi *ts)
{
	int ret;

	#if defined( SHTPS_ASYNC_OPEN_ENABLE )
		ret = shtps_func_request_sync(ts, SHTPS_FUNC_REQ_EVEMT_ENABLE);
	#else
		ret = shtps_start(ts);
		shtps_wait_startup(ts);
	#endif /* #if defined( SHTPS_ASYNC_OPEN_ENABLE ) */

	return ret;
}

static int shtps_ioctl_disable(struct shtps_rmi_spi *ts)
{
	#if defined( SHTPS_ASYNC_OPEN_ENABLE )
		shtps_func_request_sync(ts, SHTPS_FUNC_REQ_EVEMT_DISABLE);
	#else
		shtps_shutdown(ts);
	#endif /* #if defined( SHTPS_ASYNC_OPEN_ENABLE ) */

	return 0;
}

static int shtps_ioctl_reset(struct shtps_rmi_spi *ts)
{
	int power = (ts->state_mgr.state == SHTPS_STATE_IDLE)? 0 : 1;

	if(power == 0){
		shtps_irq_enable(ts);
	}

	shtps_reset(ts);
	msleep(SHTPS_HWRESET_WAIT_MS);

	if(power == 0){
		shtps_irq_disable(ts);
	}

	return 0;
}

static int shtps_ioctl_softreset(struct shtps_rmi_spi *ts)
{
	int power = (ts->state_mgr.state == SHTPS_STATE_IDLE)? 0 : 1;

	if(power == 0){
		shtps_irq_enable(ts);
	}

	if(shtps_rmi_write(ts, ts->map.fn01.commandBase, 0x01) != 0){
		return -EFAULT;
	}
	msleep(SHTPS_SWRESET_WAIT_MS);

	if(power == 0){
		shtps_irq_disable(ts);
	}

	return 0;
}

static int shtps_ioctl_getver(struct shtps_rmi_spi *ts, unsigned long arg)
{
	u8  tps_stop_request = 0;
	u16 ver;

	if(0 == arg){
		SHTPS_LOG_ERR_PRINT("[%s] error - arg == 0\n", __func__);
		return -EINVAL;
	}

	if(ts->state_mgr.state == SHTPS_STATE_IDLE){
		tps_stop_request = 1;
	}
	
	if(0 != shtps_start(ts)){
		SHTPS_LOG_ERR_PRINT("[%s] error - shtps_start()\n", __func__);
		return -EFAULT;
	}
	shtps_wait_startup(ts);

	SHTPS_LOG_DBG_PRINT("mutex_lock()\n");
	mutex_lock(&shtps_ctrl_lock);

	ver = shtps_fwver(ts);

	SHTPS_LOG_DBG_PRINT("mutex_unlock()\n");
	mutex_unlock(&shtps_ctrl_lock);

	if(tps_stop_request){
		shtps_shutdown(ts);
	}

	if(copy_to_user((u16*)arg, &ver, sizeof(ver))){
		SHTPS_LOG_ERR_PRINT("[%s] error - copy_to_user()\n", __func__);
		return -EFAULT;
	}

	SHTPS_LOG_DBG_PRINT("[%s] version = 0x%04x\n", __func__, ver);
	return 0;
}

static int shtps_ioctl_enter_bootloader(struct shtps_rmi_spi *ts, unsigned long arg)
{
	int rc;
	struct shtps_bootloader_info info;

	if(0 == arg){
		return -EINVAL;
	}
	request_event(ts, SHTPS_EVENT_STOP, 0);
	rc = shtps_enter_bootloader(ts);

	if(0 == rc){
		info.block_size        = F34_QUERY_BLOCKSIZE(ts->map.fn34.query.data);
		info.program_block_num = F34_QUERY_FIRMBLOCKCOUNT(ts->map.fn34.query.data);
		info.config_block_num  = F34_QUERY_CONFIGBLOCKCOUNT(ts->map.fn34.query.data);

		if(copy_to_user((u8*)arg, (u8*)&info, sizeof(struct shtps_bootloader_info))){
			return -EFAULT;
		}
	}

	if(rc){
		return -EFAULT;
	}

	return 0;
}

static int shtps_ioctl_lockdown_bootloader(struct shtps_rmi_spi *ts, unsigned long arg)
{
	int rc;
	u8 *data;
	struct shtps_ioctl_param param;

	if(0 == arg || 0 != copy_from_user(&param, (void __user *)arg, sizeof(param))){
		return -EINVAL;
	}

	if(param.size > SHTPS_FWDATA_BLOCK_SIZE_MAX){
		return -EINVAL;
	}

#if defined( SHTPS_FWUPDATE_BUILTIN_ENABLE )
	if (param.size < 0) {
		int position = 0;
		SHTPS_LOG_DBG_PRINT("[%s] builtin !\n", __func__);
		if (0 != copy_from_user(&position, param.data, sizeof(position))) {
			return -EINVAL;
		}
		if (position < 0 || position > SHTPS_FWSIZE_NEWER) {
			return -EINVAL;
		}
		rc = shtps_lockdown_bootloader(ts, (u8*)&(tps_fw_data[position]));
	}
	else {
		SHTPS_LOG_DBG_PRINT("[%s] normal.\n", __func__);
		data = (u8*)kmalloc(param.size, GFP_KERNEL);
		if(data == NULL){
			return -EINVAL;
		}
		if(0 != copy_from_user(data, param.data, param.size)){
			return -EINVAL;
		}
		rc = shtps_lockdown_bootloader(ts, data);
		kfree(data);
	}
#else
	data = (u8*)kmalloc(param.size, GFP_KERNEL);
	if(data == NULL){
		return -EINVAL;
	}
	if(0 != copy_from_user(data, param.data, param.size)){
		return -EINVAL;
	}
	rc = shtps_lockdown_bootloader(ts, data);
	kfree(data);
#endif /* #if defined( SHTPS_FWUPDATE_BUILTIN_ENABLE ) */

	if(rc){
		return -EFAULT;
	}

	return 0;
}

static int shtps_ioctl_erase_flash(struct shtps_rmi_spi *ts, unsigned long arg)
{
	return shtps_flash_erase(ts);
}

static int shtps_ioctl_write_image(struct shtps_rmi_spi *ts, unsigned long arg)
{
	int rc;
	u8 *data;
	struct shtps_ioctl_param param;

	if(0 == arg || 0 != copy_from_user(&param, (void __user *)arg, sizeof(param))){
		return -EINVAL;
	}

	if(param.size > SHTPS_FWDATA_BLOCK_SIZE_MAX){
		return -EINVAL;
	}

#if defined( SHTPS_FWUPDATE_BUILTIN_ENABLE )
	if (param.size < 0) {
		int position = 0;
		SHTPS_LOG_DBG_PRINT("[%s] builtin !\n", __func__);
		if (0 != copy_from_user(&position, param.data, sizeof(position))) {
			return -EINVAL;
		}
		if (position < 0 || position > SHTPS_FWSIZE_NEWER) {
			return -EINVAL;
		}
		rc = shtps_flash_writeImage(ts, (u8*)&(tps_fw_data[position]));
	}
	else {
		SHTPS_LOG_DBG_PRINT("[%s] normal !\n", __func__);
		data = (u8*)kmalloc(param.size, GFP_KERNEL);
		if(data == NULL){
			return -EINVAL;
		}
		if(0 != copy_from_user(data, param.data, param.size)){
			return -EINVAL;
		}
		rc = shtps_flash_writeImage(ts, data);
		kfree(data);
	}
#else
	data = (u8*)kmalloc(param.size, GFP_KERNEL);
	if(data == NULL){
		return -EINVAL;
	}
	if(0 != copy_from_user(data, param.data, param.size)){
		return -EINVAL;
	}
	rc = shtps_flash_writeImage(ts, data);
	kfree(data);
#endif /* #if defined( SHTPS_FWUPDATE_BUILTIN_ENABLE ) */

	if(rc){
		return -EFAULT;
	}

	return 0;
}

static int shtps_ioctl_write_config(struct shtps_rmi_spi *ts, unsigned long arg)
{
	int rc;
	u8 *data;
	struct shtps_ioctl_param param;

	if(0 == arg || 0 != copy_from_user(&param, (void __user *)arg, sizeof(param))){
		return -EINVAL;
	}

	if(param.size > SHTPS_FWDATA_BLOCK_SIZE_MAX){
		return -EINVAL;
	}

#if defined( SHTPS_FWUPDATE_BUILTIN_ENABLE )
	if (param.size < 0) {
		int position = 0;
		SHTPS_LOG_DBG_PRINT("[%s] builtin !\n", __func__);
		if (0 != copy_from_user(&position, param.data, sizeof(position))) {
			return -EINVAL;
		}
		if (position < 0 || position > SHTPS_FWSIZE_NEWER) {
			return -EINVAL;
		}
		rc = shtps_flash_writeConfig(ts, (u8*)&(tps_fw_data[position]));
	}
	else {
		SHTPS_LOG_DBG_PRINT("[%s] normal !\n", __func__);
		data = (u8*)kmalloc(param.size, GFP_KERNEL);
		if(data == NULL){
			return -EINVAL;
		}
		if(0 != copy_from_user(data, param.data, param.size)){
			return -EINVAL;
		}
		rc = shtps_flash_writeConfig(ts, data);
		kfree(data);
	}
#else
	data = (u8*)kmalloc(param.size, GFP_KERNEL);
	if(data == NULL){
		return -EINVAL;
	}
	if(0 != copy_from_user(data, param.data, param.size)){
		return -EINVAL;
	}
	rc = shtps_flash_writeConfig(ts, data);
	kfree(data);
#endif /* #if defined( SHTPS_FWUPDATE_BUILTIN_ENABLE ) */

	if(rc){
		return -EFAULT;
	}

	return 0;
}

static int shtps_ioctl_get_touchinfo(struct shtps_rmi_spi *ts, unsigned long arg)
{
	int i;
	u8 	fingerMax = shtps_get_fingermax(ts);
	struct shtps_touch_info info;

	memcpy(&info, &ts->report_info, sizeof(info));
	for(i = 0;i < fingerMax;i++){
		info.fingers[i].x = info.fingers[i].x * SHTPS_POS_SCALE_X(ts) / 10000;
		info.fingers[i].y = info.fingers[i].y * SHTPS_POS_SCALE_Y(ts) / 10000;
	}

	if(copy_to_user((u8*)arg, (u8*)&info, sizeof(info))){
		return -EFAULT;
	}

	return 0;
}

static int shtps_ioctl_get_touchinfo_untrans(struct shtps_rmi_spi *ts, unsigned long arg)
{
	if(copy_to_user((u8*)arg, (u8*)&ts->report_info, sizeof(ts->report_info))){
		return -EFAULT;
	}

	return 0;
}

static int shtps_ioctl_set_touchinfo_mode(struct shtps_rmi_spi *ts, unsigned long arg)
{
	ts->diag.pos_mode = arg;
	return 0;
}

#if defined( SHTPS_PHYSICAL_KEY_ENABLE )
static int shtps_ioctl_get_touchkeyinfo(struct shtps_rmi_spi *ts, unsigned long arg)
{
	int ret;
	struct shtps_touch_key_info info;

	ret = wait_event_interruptible_timeout(ts->diag.wait, 
			ts->diag.event_touchkey == 1,
			msecs_to_jiffies(SHTPS_DIAGPOLL_TIME));

	if(ret != 0){
		info.up_key_state = ((ts->key_state >> SHTPS_PHYSICAL_KEY_UP) & 0x01);
		info.down_key_state = ((ts->key_state >> SHTPS_PHYSICAL_KEY_DOWN) & 0x01);

		ts->diag.event_touchkey = 0;

		if(copy_to_user((u8*)arg, (u8*)&info, sizeof(info))){
			return -EFAULT;
		}

		return 0;
	}

	return -EFAULT;
}
#endif /* SHTPS_PHYSICAL_KEY_ENABLE */

static int shtps_ioctl_reg_read(struct shtps_rmi_spi *ts, unsigned long arg)
{
	u8 buf;
	u8 data[2];
	struct shtps_ioctl_param param;

	if(0 == arg || 0 != copy_from_user(&param, (void __user *)arg, sizeof(param))){
		return -EINVAL;
	}

	if(param.size == 1){
		if(0 != copy_from_user(data, param.data, 1)){
			return -EINVAL;
		}
		if(shtps_rmi_read(ts, data[0], &buf, 1)){
			return -EFAULT;
		}
	}else{
		if(0 != copy_from_user(data, param.data, 2)){
			return -EINVAL;
		}
		if(shtps_rmi_read(ts, data[0] << 0x08 | data[1], &buf, 1)){
			return -EFAULT;
		}
	}
	if(copy_to_user(((struct shtps_ioctl_param*)arg)->data, (u8*)&buf, 1)){
		return -EFAULT;
	}

	return 0;
}

static int shtps_ioctl_reg_allread(struct shtps_rmi_spi *ts, unsigned long arg)
{
	u8 dbbuf[0x100];
	u8 data;
	struct shtps_ioctl_param param;

	if(0 == arg || 0 != copy_from_user(&param, (void __user *)arg, sizeof(param))){
		return -EINVAL;
	}

	if(0 != copy_from_user(&data, param.data, 1)){
		return -EINVAL;
	}
	if(shtps_rmi_read(ts, data << 0x08, dbbuf, 0x100)){
		return -EFAULT;
	}

	if(copy_to_user(((struct shtps_ioctl_param*)arg)->data, (u8*)dbbuf, 0x100)){
		return -EFAULT;
	}
	return 0;
}

static int shtps_ioctl_reg_write(struct shtps_rmi_spi *ts, unsigned long arg)
{
	u8 data[3];
	struct shtps_ioctl_param param;

	if(0 == arg || 0 != copy_from_user(&param, (void __user *)arg, sizeof(param))){
		return -EINVAL;
	}

	if(param.size == 2){
		if(0 != copy_from_user(data, param.data, 2)){
			return -EINVAL;
		}
		if(shtps_rmi_write(ts, data[0], data[1])){
			return -EFAULT;
		}
	}else{
		if(0 != copy_from_user(data, param.data, 3)){
			return -EINVAL;
		}
		if(shtps_rmi_write(ts, data[0] << 0x08 | data[1], data[2])){
			return -EFAULT;
		}
	}
	return 0;
}

static int shtps_ioctl_tm_start(struct shtps_rmi_spi *ts, unsigned long arg)
{
	ts->diag.tm_mode = SHTPS_FWTESTMODE_V01;
	if(0 != request_event(ts, SHTPS_EVENT_STARTTM, arg)){
		return -EFAULT;
	}
	return 0;
}

static int shtps_ioctl_tm_stop(struct shtps_rmi_spi *ts)
{
	if(0 != request_event(ts, SHTPS_EVENT_STOPTM, 0)){
		return -EFAULT;
	}
	return 0;
}

static int shtps_ioctl_get_baseline(struct shtps_rmi_spi *ts, unsigned long arg)
{
	shtps_read_tmdata(ts, SHTPS_TMMODE_BASELINE);
	if(copy_to_user((u8*)arg, (u8*)ts->diag.tm_data,
		shtps_get_tm_rxsize(ts) * shtps_get_tm_txsize(ts) * 2)){
		return -EFAULT;
	}
	return 0;
}

static int shtps_ioctl_get_frameline(struct shtps_rmi_spi *ts, unsigned long arg)
{
	shtps_read_tmdata(ts, SHTPS_TMMODE_FRAMELINE);
	if(copy_to_user((u8*)arg, (u8*)ts->diag.tm_data,
#if defined( SHTPS_SY_REGMAP_BASE3 )
		shtps_get_tm_rxsize(ts) * shtps_get_tm_txsize(ts) * 2)){
#else
		shtps_get_tm_rxsize(ts) * shtps_get_tm_txsize(ts))){
#endif /* #if defined( SHTPS_SY_REGMAP_BASE3 ) */
		return -EFAULT;
	}
	return 0;
}

static int shtps_ioctl_start_facetouchmode(struct shtps_rmi_spi *ts)
{
	return request_event(ts, SHTPS_EVENT_FACETOUCHMODE_ON, 0);
}

static int shtps_ioctl_stop_facetouchmode(struct shtps_rmi_spi *ts)
{
	int ret = request_event(ts, SHTPS_EVENT_FACETOUCHMODE_OFF, 0);
#if defined( CONFIG_SHTPS_SY3000_FACETOUCH_OFF_DETECT )
	_log_msg_send( LOGMSG_ID__DETECT_FACETOUCH, "");
	wake_up_interruptible(&ts->facetouch.wait_off);
	shtps_wake_unlock(ts);
#endif /* #if defined( CONFIG_SHTPS_SY3000_FACETOUCH_OFF_DETECT ) */
	return ret;
}

static int shtps_ioctl_poll_facetouchoff(struct shtps_rmi_spi *ts)
{
#if defined( CONFIG_SHTPS_SY3000_FACETOUCH_OFF_DETECT )
	int rc;

	_log_msg_sync( LOGMSG_ID__DETECT_FACETOUCH_WAIT, "");
	rc = wait_event_interruptible(ts->facetouch.wait_off,
		(ts->facetouch.off_detect == 1) || (ts->facetouch.mode == 0) ||
		(ts->facetouch.wake_sig == 1));

	_log_msg_recv( LOGMSG_ID__DETECT_FACETOUCH, "%d|%d|%d",
						ts->facetouch.off_detect, ts->facetouch.mode, ts->facetouch.wake_sig);

	ts->facetouch.wake_sig = 0;

	if(ts->facetouch.off_detect){
		SHTPS_LOG_DBG_PRINT("face touch off detect\n");
		rc = TPSDEV_FACETOUCHOFF_DETECT;
		ts->facetouch.off_detect = 0;
	}else{
		rc = TPSDEV_FACETOUCHOFF_NOCHG;
	}

	return rc;
#else
	return TPSDEV_FACETOUCHOFF_NOCHG;
#endif /* #if defined( CONFIG_SHTPS_SY3000_FACETOUCH_OFF_DETECT ) */
}

static int shtps_ioctl_get_fwstatus(struct shtps_rmi_spi *ts, unsigned long arg)
{
	unsigned char status;

	shtps_rmi_read(ts, ts->map.fn01.dataBase, &status, 1);
	status = status & 0x0F;

	if(copy_to_user((u8*)arg, (u8*)&status, sizeof(status))){
		return -EFAULT;
	}
	return 0;
}

static int shtps_ioctl_get_fwdate(struct shtps_rmi_spi *ts, unsigned long arg)
{
	u8 year;
	u8 month;
	unsigned short date;

	SHTPS_LOG_DBG_PRINT("mutex_lock()\n");
	mutex_lock(&shtps_ctrl_lock);

	shtps_fwdate(ts, &year, &month);

	SHTPS_LOG_DBG_PRINT("mutex_unlock()\n");
	mutex_unlock(&shtps_ctrl_lock);

	date = (year << 0x08) | month;

	if(copy_to_user((u16*)arg, (u16*)&date, sizeof(date))){
		return -EFAULT;
	}
	return 0;
}

static int shtps_ioctl_calibration_param(struct shtps_rmi_spi *ts, unsigned long arg)
{
	u8 *data;
	struct shtps_ioctl_param param;

	if(0 == arg || 0 != copy_from_user(&param, (void __user *)arg, sizeof(param))){
		return -EINVAL;
	}

	if(param.size > sizeof(struct shtps_offset_info)){
		return -EINVAL;
	}
	data = (u8*)kmalloc(param.size, GFP_KERNEL);
	if(data == NULL){
		return -EINVAL;
	}
	if(0 != copy_from_user(data, param.data, param.size)){
		return -EINVAL;
	}
	memcpy(ts->offset.base, data, sizeof(u16) * 5);
	ts->offset.diff[0] = (signed short)(data[11] << 0x08 | data[10]);
	ts->offset.diff[1] = (signed short)(data[13] << 0x08 | data[12]);
	ts->offset.diff[2] = (signed short)(data[15] << 0x08 | data[14]);
	ts->offset.diff[3] = (signed short)(data[17] << 0x08 | data[16]);
	ts->offset.diff[4] = (signed short)(data[19] << 0x08 | data[18]);
	ts->offset.diff[5] = (signed short)(data[21] << 0x08 | data[20]);
	ts->offset.diff[6] = (signed short)(data[23] << 0x08 | data[22]);
	ts->offset.diff[7] = (signed short)(data[25] << 0x08 | data[24]);
	ts->offset.diff[8] = (signed short)(data[27] << 0x08 | data[26]);
	ts->offset.diff[9] = (signed short)(data[29] << 0x08 | data[28]);
	ts->offset.diff[10]= (signed short)(data[31] << 0x08 | data[30]);
	ts->offset.diff[11]= (signed short)(data[33] << 0x08 | data[32]);
	kfree(data);

	if(ts->offset.base[0] == 0){
		ts->offset.enabled = 0;
	}else{
		ts->offset.enabled = 1;
	}

	return 0;
}

static int shtps_ioctl_debug_reqevent(struct shtps_rmi_spi *ts, unsigned long arg)
{
	request_event(ts, (int)arg, 0);
	return 0;
}

static int shtps_ioctl_set_dragstep_x(struct shtps_rmi_spi *ts, unsigned long arg)
{
#if defined( SHTPS_DEBUG_VARIABLE_DEFINES )
	if((int)arg != 0){
		SHTPS_DRAG_THRESH_VAL_X_1ST = (int)(arg & 0xFF);
		SHTPS_DRAG_THRESH_VAL_X_2ND = (int)((arg >> 0x08) & 0xFF);
		SHTPS_DRAG_THRESH_VAL_X_1ST_MULTI = (int)((arg >> 0x10) & 0xFF);
		SHTPS_DRAG_THRESH_VAL_X_2ND_MULTI = (int)((arg >> 0x18) & 0xFF);
	}
#endif /* #if defined( SHTPS_DEBUG_VARIABLE_DEFINES ) */
	return SHTPS_DRAG_THRESH_VAL_X_1ST |
		   ((SHTPS_DRAG_THRESH_VAL_X_2ND << 0x08) & 0x0000FF00) |
		   ((SHTPS_DRAG_THRESH_VAL_X_1ST_MULTI << 0x10) & 0x00FF0000) |
		   ((SHTPS_DRAG_THRESH_VAL_X_2ND_MULTI << 0x18) & 0xFF000000);
}
static int shtps_ioctl_set_dragstep_y(struct shtps_rmi_spi *ts, unsigned long arg)
{
#if defined( SHTPS_DEBUG_VARIABLE_DEFINES )
	if((int)arg != 0){
		SHTPS_DRAG_THRESH_VAL_Y_1ST = (int)(arg & 0xFF);
		SHTPS_DRAG_THRESH_VAL_Y_2ND = (int)((arg >> 0x08) & 0xFF);
		SHTPS_DRAG_THRESH_VAL_Y_1ST_MULTI = (int)((arg >> 0x10) & 0xFF);
		SHTPS_DRAG_THRESH_VAL_Y_2ND_MULTI = (int)((arg >> 0x18) & 0xFF);
	}
#endif /* #if defined( SHTPS_DEBUG_VARIABLE_DEFINES ) */
	return SHTPS_DRAG_THRESH_VAL_Y_1ST |
		   ((SHTPS_DRAG_THRESH_VAL_Y_2ND << 0x08) & 0x0000FF00) |
		   ((SHTPS_DRAG_THRESH_VAL_Y_1ST_MULTI << 0x10) & 0x00FF0000) |
		   ((SHTPS_DRAG_THRESH_VAL_Y_2ND_MULTI << 0x18) & 0xFF000000);
}
static int shtps_ioctl_set_dragstep(struct shtps_rmi_spi *ts, unsigned long arg)
{
	return 0;
}

static int shtps_ioctl_set_fingerfixtime(struct shtps_rmi_spi *ts, unsigned long arg)
{
#if defined( SHTPS_DEBUG_VARIABLE_DEFINES )
	if((int)arg < 0xFFFF){
		SHTPS_DRAG_THRESH_RETURN_TIME =	(int)arg;
	}
#endif /* #if defined( SHTPS_DEBUG_VARIABLE_DEFINES ) */
	return SHTPS_DRAG_THRESH_RETURN_TIME;
}

static int shtps_ioctl_rezero(struct shtps_rmi_spi *ts, unsigned long arg)
{
	SHTPS_LOG_DBG_PRINT("mutex_lock()\n");
	mutex_lock(&shtps_ctrl_lock);

	if(ts->poll_info.boot_rezero_flag == 0){
		ts->poll_info.boot_rezero_flag = 1;
			shtps_rezero_request(ts,
								 SHTPS_REZERO_REQUEST_REZERO |
								 SHTPS_REZERO_REQUEST_AUTOREZERO_ENABLE,
								 SHTPS_REZERO_TRIGGER_BOOT);
	}

	SHTPS_LOG_DBG_PRINT("mutex_unlock()\n");
	mutex_unlock(&shtps_ctrl_lock);

	return 0;
}

static int shtps_ioctl_ack_facetouchoff(struct shtps_rmi_spi *ts, unsigned long arg)
{
#if defined( CONFIG_SHTPS_SY3000_FACETOUCH_OFF_DETECT )
	mutex_lock(&shtps_ctrl_lock);
	shtps_wake_unlock(ts);
	mutex_unlock(&shtps_ctrl_lock);
#endif /* #if defined( CONFIG_SHTPS_SY3000_FACETOUCH_OFF_DETECT ) */
	return 0;
}

static int shtps_ioctl_tmf05_start(struct shtps_rmi_spi *ts, unsigned long arg)
{
	if(ts->map.fn05.enable != 0){
		ts->diag.tm_mode = SHTPS_FWTESTMODE_V02;
	}else{
		ts->diag.tm_mode = SHTPS_FWTESTMODE_V03;
	}

	if(0 != request_event(ts, SHTPS_EVENT_STARTTM, arg)){
		return -EFAULT;
	}
	return 0;
}

#if defined( SHTPS_LOG_OUTPUT_SWITCH_ENABLE )
static int shtps_ioctl_log_enable(struct shtps_rmi_spi *ts, unsigned long arg)
{
	gLogOutputEnable = (int)arg;
	return 0;
}
#endif /* #if defined( SHTPS_LOG_OUTPUT_SWITCH_ENABLE ) */

static int shtps_ioctl_set_invalid_area(struct shtps_rmi_spi *ts, unsigned long arg)
{
#if defined( SHTPS_VKEY_INVALID_AREA_ENABLE )
	#if defined( SHTPS_DEBUG_VARIABLE_DEFINES )
		if( (0 <= (int)arg) && ((int)arg <= 1279) ){
			shtps_invalid_area[0].sx = 0;
			shtps_invalid_area[0].sy = (int)arg;
		}else if( (int)arg < 0 ){
			shtps_invalid_area[0].sx = -1;
		}
	#endif /* #if defined( SHTPS_DEBUG_VARIABLE_DEFINES ) */

	return shtps_invalid_area[0].sy;

#else
	return -EFAULT;

#endif /* #if defined( SHTPS_VKEY_INVALID_AREA_ENABLE ) */
}

#if defined( SHTPS_FWUPDATE_BUILTIN_ENABLE )
static int shtps_ioctl_getver_builtin(struct shtps_rmi_spi *ts, unsigned long arg)
{
	u16 ver = (u16)SHTPS_FWVER_NEWER;

	if(0 == arg){
		SHTPS_LOG_ERR_PRINT("[%s] error - arg == 0\n", __func__);
		return -EINVAL;
	}

	if(copy_to_user((u16*)arg, &ver, sizeof(ver))){
		SHTPS_LOG_ERR_PRINT("[%s] error - copy_to_user()\n", __func__);
		return -EFAULT;
	}

	SHTPS_LOG_DBG_PRINT("[%s] built-in version = 0x%04x\n", __func__, ver);
	return 0;
}
#endif /* #if defined( SHTPS_FWUPDATE_BUILTIN_ENABLE ) */

#if defined( SHTPS_SMEM_BASELINE_ENABLE )
static int shtps_ioctl_get_smem_baseline(struct shtps_rmi_spi *ts, unsigned long arg)
{
	sharp_smem_common_type *smemdata = sh_smem_get_common_address();

	if(!smemdata){
		return -EFAULT;
	}
	
	if(copy_to_user((u8*)arg, (u8*)(smemdata->shdiag_TpsBaseLineTbl),
		shtps_get_tm_rxsize(ts) * shtps_get_tm_txsize(ts) * 2)){
		return -EFAULT;
	}
	return 0;
}
#endif /* #if defined( SHTPS_SMEM_BASELINE_ENABLE ) */

static int shtps_ioctl_set_low_power_mode(struct shtps_rmi_spi *ts, unsigned long arg)
{
#if defined( SHTPS_LOW_POWER_MODE_ENABLE )
	SHTPS_LOG_DBG_PRINT("mutex_lock()\n");
	mutex_lock(&shtps_ctrl_lock);

	shtps_set_lpmode(ts, SHTPS_LPMODE_NON_CONTINUOUS_REQ_ON, (int)arg);

	SHTPS_LOG_DBG_PRINT("mutex_unlock()\n");
	mutex_unlock(&shtps_ctrl_lock);
#endif /* #if defined( SHTPS_LOW_POWER_MODE_ENABLE ) */

	return 0;
}

static int shtps_ioctl_set_continuous_low_power_mode(struct shtps_rmi_spi *ts, unsigned long arg)
{
#if defined( SHTPS_LOW_POWER_MODE_ENABLE )
	SHTPS_LOG_DBG_PRINT("mutex_lock()\n");
	mutex_lock(&shtps_ctrl_lock);

	shtps_set_lpmode(ts, SHTPS_LPMODE_CONTINUOUS_REQ_ON, (int)arg);

	SHTPS_LOG_DBG_PRINT("mutex_unlock()\n");
	mutex_unlock(&shtps_ctrl_lock);
#endif /* #if defined( SHTPS_LOW_POWER_MODE_ENABLE ) */

	return 0;
}

static int shtps_ioctl_set_charger_armor(struct shtps_rmi_spi *ts, unsigned long arg)
{
	int ret = 0;

	SHTPS_LOG_DBG_PRINT("mutex_lock()\n");
	mutex_lock(&shtps_ctrl_lock);

	ret = shtps_set_charger_armor(ts, (int)arg);

	SHTPS_LOG_DBG_PRINT("mutex_unlock()\n");
	mutex_unlock(&shtps_ctrl_lock);

	return ret;
}

static int shtps_ioctl_set_wireless_charger_armor(struct shtps_rmi_spi *ts, unsigned long arg)
{
	int ret = 0;

	SHTPS_LOG_DBG_PRINT("mutex_lock()\n");
	mutex_lock(&shtps_ctrl_lock);

	ret = shtps_set_charger_armor(ts, (int)arg);

	SHTPS_LOG_DBG_PRINT("mutex_unlock()\n");
	mutex_unlock(&shtps_ctrl_lock);

	return ret;
}

#if defined( SHTPS_LPWG_MODE_ENABLE )
static int shtps_ioctl_lpwg_enable(struct shtps_rmi_spi *ts, unsigned long arg)
{
    mutex_lock(&shtps_loader_lock);
	ts->lpwg.lpwg_switch = arg;
	ts->lpwg.notify_enable = 1;
    SHTPS_LOG_DBG_PRINT(" [LPWG] lpwg_switch = %d\n", ts->lpwg.lpwg_switch);
    if (SHTPS_STATE_SLEEP == ts->state_mgr.state) 
    {
        shtps_statef_sleep_enter(ts, 0);
    }
    mutex_unlock(&shtps_loader_lock);
	return 0;
}
#endif /* SHTPS_LPWG_MODE_ENABLE */

#if defined( SHTPS_CHECK_CRC_ERROR_ENABLE )
static int shtps_ioctl_check_crc_error(struct shtps_rmi_spi *ts)
{
	#if defined( SHTPS_ASYNC_OPEN_ENABLE )
		shtps_func_request_sync(ts, SHTPS_FUNC_REQ_EVEMT_CHECK_CRC_ERROR);
	#else
		shtps_func_check_crc_error(ts);
	#endif /* #if defined( SHTPS_ASYNC_OPEN_ENABLE ) */

	return 0;
}
#endif /* if defined( SHTPS_CHECK_CRC_ERROR_ENABLE ) */
/* -----------------------------------------------------------------------------------
 */
static int shtpsif_open(struct inode *inode, struct file *file)
{
	_log_msg_sync( LOGMSG_ID__DEVICE_OPEN, "");
	SHTPS_LOG_DBG_PRINT("[shtpsif]Open(PID:%ld)\n", sys_getpid());
	return 0;
}

static int shtpsif_release(struct inode *inode, struct file *file)
{
	_log_msg_sync( LOGMSG_ID__DEVICE_RELEASE, "");
	SHTPS_LOG_DBG_PRINT("[shtpsif]Close(PID:%ld)\n", sys_getpid());
	return 0;
}

static ssize_t shtpsif_read(struct file *file, char *buf, size_t count, loff_t *pos)
{
	int i;
	struct shtps_rmi_spi *ts = gShtps_rmi_spi;
	u8 	fingerMax = shtps_get_fingermax(ts);
	struct shtps_touch_info info;

	_log_msg_sync( LOGMSG_ID__DEVICE_READ, "");

	if(NULL == ts){
		_log_msg_sync( LOGMSG_ID__DEVICE_READ_FAIL, "");
		return -EFAULT;
	}

	wait_event_interruptible(ts->diag.wait, ts->diag.event == 1);

	memcpy(&info, &ts->report_info, sizeof(info));
	if(ts->diag.pos_mode == TPSDEV_TOUCHINFO_MODE_LCDSIZE){
		for(i = 0;i < fingerMax;i++){
			info.fingers[i].x = info.fingers[i].x * SHTPS_POS_SCALE_X(ts) / 10000;
			info.fingers[i].y = info.fingers[i].y * SHTPS_POS_SCALE_Y(ts) / 10000;
		}
	}
	if(copy_to_user((u8*)buf, (u8*)&info, sizeof(info))){
		_log_msg_sync( LOGMSG_ID__DEVICE_READ_FAIL, "");
		return -EFAULT;
	}

	ts->diag.event = 0;
	_log_msg_sync( LOGMSG_ID__DEVICE_READ_DONE, "");
	return sizeof(ts->report_info);
}

static unsigned int shtpsif_poll(struct file *file, poll_table *wait)
{
	int ret;
	struct shtps_rmi_spi *ts = gShtps_rmi_spi;

	_log_msg_sync( LOGMSG_ID__DEVICE_POLL, "");

	if(NULL == ts){
		_log_msg_sync( LOGMSG_ID__DEVICE_POLL_FAIL, "");
		return POLLERR;
	}

	ret = wait_event_interruptible_timeout(ts->diag.wait,
			ts->diag.event == 1,
			msecs_to_jiffies(SHTPS_DIAGPOLL_TIME));

	if(0 != ret){
		_log_msg_sync( LOGMSG_ID__DEVICE_POLL_DONE, "%d", POLLIN | POLLRDNORM);
		return POLLIN | POLLRDNORM;
	}

	_log_msg_sync( LOGMSG_ID__DEVICE_POLL_DONE, "0");
	return 0;
}

static long shtpsif_ioctl(struct file *file, unsigned int cmd, unsigned long arg)
{
	int	rc = 0;
	struct shtps_rmi_spi *ts = gShtps_rmi_spi;

	_log_msg_sync( LOGMSG_ID__DEVICE_IOCTL, "%ld|0x%08X|0x%lX", sys_getpid(), cmd, arg);
	SHTPS_LOG_DBG_PRINT("[shtpsif]ioctl(PID:%ld,CMD:%d,ARG:0x%lx)\n",
											sys_getpid(), cmd, arg);

	if(ts == NULL){
		SHTPS_LOG_DBG_PRINT("[shtpsif] shtpsif_ioctl ts == NULL\n");
		_log_msg_sync( LOGMSG_ID__DEVICE_IOCTL_FAIL, "");
		return -EFAULT;
	}

	switch(cmd){
	case TPSDEV_ENABLE: 				rc = shtps_ioctl_enable(ts);					break;
	case TPSDEV_DISABLE:				rc = shtps_ioctl_disable(ts);					break;
	case TPSDEV_RESET:					rc = shtps_ioctl_reset(ts);						break;
	case TPSDEV_SOFT_RESET:				rc = shtps_ioctl_softreset(ts);					break;
	case TPSDEV_GET_FW_VERSION:			rc = shtps_ioctl_getver(ts, arg);				break;
	case TPSDEV_ENTER_BOOTLOADER:		rc = shtps_ioctl_enter_bootloader(ts, arg);		break;
	case TPSDEV_LOCKDOWN_BOOTLOADER:	rc = shtps_ioctl_lockdown_bootloader(ts, arg);	break;
	case TPSDEV_ERASE_FLASE:			rc = shtps_ioctl_erase_flash(ts, arg);			break;
	case TPSDEV_WRITE_IMAGE:			rc = shtps_ioctl_write_image(ts, arg);			break;
	case TPSDEV_WRITE_CONFIG:			rc = shtps_ioctl_write_config(ts, arg);			break;
	case TPSDEV_GET_TOUCHINFO:			rc = shtps_ioctl_get_touchinfo(ts, arg);		break;
	case TPSDEV_GET_TOUCHINFO_UNTRANS:	rc = shtps_ioctl_get_touchinfo_untrans(ts, arg);break;
	case TPSDEV_SET_TOUCHMONITOR_MODE:	rc = shtps_ioctl_set_touchinfo_mode(ts, arg);	break;
	case TPSDEV_READ_REG:				rc = shtps_ioctl_reg_read(ts, arg);				break;
	case TPSDEV_READ_ALL_REG:			rc = shtps_ioctl_reg_allread(ts, arg);			break;
	case TPSDEV_WRITE_REG:				rc = shtps_ioctl_reg_write(ts, arg);			break;
	case TPSDEV_START_TM:				rc = shtps_ioctl_tm_start(ts, arg);				break;
	case TPSDEV_STOP_TM:				rc = shtps_ioctl_tm_stop(ts);					break;
	case TPSDEV_GET_BASELINE:			rc = shtps_ioctl_get_baseline(ts, arg);			break;
	case TPSDEV_GET_FRAMELINE:			rc = shtps_ioctl_get_frameline(ts, arg);		break;
	case TPSDEV_START_FACETOUCHMODE:	rc = shtps_ioctl_start_facetouchmode(ts);		break;
	case TPSDEV_STOP_FACETOUCHMODE:		rc = shtps_ioctl_stop_facetouchmode(ts);		break;
	case TPSDEV_POLL_FACETOUCHOFF:		rc = shtps_ioctl_poll_facetouchoff(ts);			break;
	case TPSDEV_GET_FWSTATUS:			rc = shtps_ioctl_get_fwstatus(ts, arg);			break;
	case TPSDEV_GET_FWDATE:				rc = shtps_ioctl_get_fwdate(ts, arg);			break;
	case TPSDEV_CALIBRATION_PARAM:		rc = shtps_ioctl_calibration_param(ts, arg);	break;
	case TPSDEV_DEBUG_REQEVENT:			rc = shtps_ioctl_debug_reqevent(ts, arg);		break;
	case TPSDEV_SET_DRAGSTEP:			rc = shtps_ioctl_set_dragstep(ts, arg);			break;
	case TPSDEV_SET_FINGERFIXTIME:		rc = shtps_ioctl_set_fingerfixtime(ts, arg);	break;
	case TPSDEV_REZERO:					rc = shtps_ioctl_rezero(ts, arg);				break;
	case TPSDEV_ACK_FACETOUCHOFF:		rc = shtps_ioctl_ack_facetouchoff(ts, arg);		break;
	case TPSDEV_START_TM_F05:			rc = shtps_ioctl_tmf05_start(ts, arg);			break;
	case TPSDEV_SET_DRAGSTEP_X:			rc = shtps_ioctl_set_dragstep_x(ts, arg);		break;
	case TPSDEV_SET_DRAGSTEP_Y:			rc = shtps_ioctl_set_dragstep_y(ts, arg);		break;
#if defined( SHTPS_LOG_OUTPUT_SWITCH_ENABLE )
	case TPSDEV_LOGOUTPUT_ENABLE:		rc = shtps_ioctl_log_enable(ts, arg);			break;
#endif /* #if defined( SHTPS_LOG_OUTPUT_SWITCH_ENABLE ) */
#if defined( SHTPS_PHYSICAL_KEY_ENABLE )
	case TPSDEV_GET_TOUCHKEYINFO:		rc = shtps_ioctl_get_touchkeyinfo(ts, arg);		break;
#endif /* #if defined( SHTPS_PHYSICAL_KEY_ENABLE ) */
#if defined( SHTPS_FWUPDATE_BUILTIN_ENABLE )
	case TPSDEV_GET_FW_VERSION_BUILTIN:	rc = shtps_ioctl_getver_builtin(ts, arg);		break;
#endif /* #if defined( SHTPS_FWUPDATE_BUILTIN_ENABLE ) */
	case TPSDEV_SET_INVALID_AREA:		rc = shtps_ioctl_set_invalid_area(ts, arg);		break;
#if defined( SHTPS_SMEM_BASELINE_ENABLE )
	case TPSDEV_GET_SMEM_BASELINE:		rc = shtps_ioctl_get_smem_baseline(ts, arg);	break;
#endif /* #if defined( SHTPS_SMEM_BASELINE_ENABLE ) */
	case TPSDEV_SET_LOWPOWER_MODE:		rc = shtps_ioctl_set_low_power_mode(ts, arg);	break;
	case TPSDEV_SET_CONT_LOWPOWER_MODE:	rc = shtps_ioctl_set_continuous_low_power_mode(ts, arg); break;
	case TPSDEV_SET_CHARGER_ARMOR:		rc = shtps_ioctl_set_charger_armor(ts, arg); break;
	case TPSDEV_SET_WIRELESS_CHARGER_ARMOR:	rc = shtps_ioctl_set_wireless_charger_armor(ts, arg); break;
#if defined( SHTPS_LPWG_MODE_ENABLE )
	case TPSDEV_LPWG_ENABLE:	rc = shtps_ioctl_lpwg_enable(ts, arg); break;
#endif /* SHTPS_LPWG_MODE_ENABLE */
#if defined( SHTPS_CHECK_CRC_ERROR_ENABLE )
	case TPSDEV_CHECK_CRC_ERROR:	rc = shtps_ioctl_check_crc_error(ts); break;
#endif /* if defined( SHTPS_CHECK_CRC_ERROR_ENABLE ) */
	default:
		SHTPS_LOG_DBG_PRINT("[shtpsif] shtpsif_ioctl switch(cmd) default\n");
		_log_msg_sync( LOGMSG_ID__DEVICE_IOCTL_FAIL, "");
		rc = -ENOIOCTLCMD;
		break;
	}
	_log_msg_sync( LOGMSG_ID__DEVICE_IOCTL_DONE, "%d", rc);
	return rc;
}

static const struct file_operations shtpsif_fileops = {
	.owner   = THIS_MODULE,
	.open    = shtpsif_open,
	.release = shtpsif_release,
	.read    = shtpsif_read,
	.poll    = shtpsif_poll,
	.unlocked_ioctl   = shtpsif_ioctl,
};

int __init shtpsif_init(void)
{
	int rc;

	_log_msg_sync( LOGMSG_ID__DEVICE_INIT, "");
	rc = alloc_chrdev_region(&shtpsif_devid, 0, 1, SH_TOUCH_IF_DEVNAME);
	if(rc < 0){
		SHTPS_LOG_ERR_PRINT("shtpsif:alloc_chrdev_region error\n");
		return rc;
	}

	shtpsif_class = class_create(THIS_MODULE, SH_TOUCH_IF_DEVNAME);
	if (IS_ERR(shtpsif_class)) {
		rc = PTR_ERR(shtpsif_class);
		SHTPS_LOG_ERR_PRINT("shtpsif:class_create error\n");
		goto error_vid_class_create;
	}

	shtpsif_device = device_create(shtpsif_class, NULL,
								shtpsif_devid, &shtpsif_cdev,
								SH_TOUCH_IF_DEVNAME);
	if (IS_ERR(shtpsif_device)) {
		rc = PTR_ERR(shtpsif_device);
		SHTPS_LOG_ERR_PRINT("shtpsif:device_create error\n");
		goto error_vid_class_device_create;
	}

	cdev_init(&shtpsif_cdev, &shtpsif_fileops);
	shtpsif_cdev.owner = THIS_MODULE;
	rc = cdev_add(&shtpsif_cdev, shtpsif_devid, 1);
	if(rc < 0){
		SHTPS_LOG_ERR_PRINT("shtpsif:cdev_add error\n");
		goto err_via_cdev_add;
	}

	SHTPS_LOG_DBG_PRINT("[shtpsif]shtpsif_init() done\n");
	_log_msg_sync( LOGMSG_ID__DEVICE_INIT_DONE, "");

	return 0;

err_via_cdev_add:
	cdev_del(&shtpsif_cdev);
error_vid_class_device_create:
	class_destroy(shtpsif_class);
error_vid_class_create:
	unregister_chrdev_region(shtpsif_devid, 1);
	_log_msg_sync( LOGMSG_ID__DEVICE_INIT_FAIL, "");

	return rc;
}
module_init(shtpsif_init);

static void __exit shtpsif_exit(void)
{
	cdev_del(&shtpsif_cdev);
	class_destroy(shtpsif_class);
	unregister_chrdev_region(shtpsif_devid, 1);

	_log_msg_sync( LOGMSG_ID__DEVICE_EXIT_DONE, "");
	SHTPS_LOG_DBG_PRINT("[shtpsif]shtpsif_exit() done\n");
}
module_exit(shtpsif_exit);

/* -----------------------------------------------------------------------------------
 */
static int shtps_get_bt_ver(struct shtps_rmi_spi *ts)
{
#if defined( SHTPS_BTLOADER_VER_ENABLE )
	if(0 != shtps_enter_bootloader(ts)){
		return -1;
	}

	if(0 != shtps_exit_bootloader(ts)){
		return -1;
	}
#endif	/* #if defined( SHTPS_BTLOADER_VER_ENABLE ) */
	return 0;
}

static int shtps_rmi_open(struct input_dev *dev)
{
	struct shtps_rmi_spi *ts = (struct shtps_rmi_spi*)input_get_drvdata(dev);

	_log_msg_sync( LOGMSG_ID__OPEN, "%ld", sys_getpid());
	SHTPS_LOG_DBG_PRINT("[shtps]Open(PID:%ld)\n", sys_getpid());

#if defined( SHTPS_ASYNC_OPEN_ENABLE )
	shtps_func_request_async(ts, SHTPS_FUNC_REQ_EVEMT_OPEN);
#else
#if defined( SHTPS_BOOT_FWUPDATE_ENABLE )
	if( shtps_boot_fwupdate_enable_check(ts) != 0 ){
		u8 buf;
		const unsigned char* fw_data = NULL;
		int ver;
		u8 update = 0;

		if(shtps_start(ts) == 0){
			shtps_wait_startup(ts);
		}

		shtps_rmi_read(ts, 0x0013, &buf, 1);

		#if defined( SHTPS_BOOT_FWUPDATE_FORCE_UPDATE )
			ver = shtps_fwver(ts);
			printk(KERN_DEBUG "fw version = 0x%04x\n", ver);
			if(ver != SHTPS_FWVER_NEWER){
				update = 1;
			}
		#else
			if(shtps_fwup_flag_check() > 0){
				ver = shtps_fwver(ts);
				if(ver != SHTPS_FWVER_NEWER){
					update = 1;
				}
			}
		#endif /* if defined( SHTPS_BOOT_FWUPDATE_FORCE_UPDATE ) */

		if((buf & 0x0F) == 4 || (buf & 0x0F) == 5 || (buf & 0x0F) == 6){
			SHTPS_LOG_ERR_PRINT("Touch panel CRC error detect\n");
			update = 1;
		}

		if(update != 0){
			fw_data = tps_fw_data;
			if(fw_data){
				int ret;
				int retry = 5;
				do{
					ret = shtps_fw_update(ts, fw_data);
					request_event(ts, SHTPS_EVENT_STOP, 0);
				}while(ret != 0 && (retry-- > 0));
			}
		}
	}
	shtps_fwup_flag_clear();

#endif /* #if defined( SHTPS_BOOT_FWUPDATE_ENABLE ) */
	shtps_get_bt_ver(ts);
	shtps_start(ts);
#endif /* #if defined( SHTPS_ASYNC_OPEN_ENABLE ) */

	return 0;
}

static void shtps_rmi_close(struct input_dev *dev)
{
	struct shtps_rmi_spi *ts = (struct shtps_rmi_spi*)input_get_drvdata(dev);

	_log_msg_sync( LOGMSG_ID__CLOSE, "%ld", sys_getpid());
	SHTPS_LOG_DBG_PRINT("[shtps]Close(PID:%ld)\n", sys_getpid());

	#if defined( SHTPS_ASYNC_OPEN_ENABLE )
		shtps_func_request_async(ts, SHTPS_FUNC_REQ_EVEMT_CLOSE);
	#else
		shtps_shutdown(ts);
	#endif /* #if defined( SHTPS_ASYNC_OPEN_ENABLE ) */
}

static int __devinit shtps_rmi_probe(struct spi_device *spi)
{
	int i;
	int result;
	struct input_dev *input_dev;
	struct shtps_rmi_spi *ts;
	struct shtps_platform_data *pdata = spi->dev.platform_data;

	_log_msg_sync( LOGMSG_ID__PROBE, "");
	SHTPS_LOG_DBG_PRINT("[shtps]mutex_lock()\n");
	mutex_lock(&shtps_ctrl_lock);

	ts = kzalloc(sizeof(struct shtps_rmi_spi), GFP_KERNEL);
	input_dev = input_allocate_device();
	if (!input_dev || !ts) {
		result = -ENOMEM;
		mutex_unlock(&shtps_ctrl_lock);
		goto fail_alloc_mem;
	}
	spi_set_drvdata(spi, ts);

	if (pdata && pdata->setup) {
		result = pdata->setup(&spi->dev);
		if (result){
			mutex_unlock(&shtps_ctrl_lock);
			goto fail_alloc_mem;
		}
	}
	INIT_DELAYED_WORK(&ts->tmo_check, shtps_work_tmof);

	init_waitqueue_head(&ts->wait_start);
	init_waitqueue_head(&ts->loader.wait_ack);
	init_waitqueue_head(&ts->diag.wait);
	init_waitqueue_head(&ts->diag.tm_wait_ack);
#if defined( CONFIG_SHTPS_SY3000_FACETOUCH_OFF_DETECT )
	init_waitqueue_head(&ts->facetouch.wait_off);
#endif /* #if defined( CONFIG_SHTPS_SY3000_FACETOUCH_OFF_DETECT ) */

	hrtimer_init(&ts->rezero_delayed_timer, CLOCK_MONOTONIC, HRTIMER_MODE_REL);
	ts->rezero_delayed_timer.function = shtps_delayed_rezero_timer_function;
	INIT_WORK(&ts->rezero_delayed_work, shtps_rezero_delayed_work_function);

#if defined( SHTPS_ASYNC_OPEN_ENABLE )
	ts->workqueue_p = alloc_workqueue("TPS_WORK", WQ_UNBOUND | WQ_MEM_RECLAIM, 1);
	if(ts->workqueue_p == NULL){
		mutex_unlock(&shtps_ctrl_lock);
		goto fail_alloc_mem;
	}
	INIT_WORK( &(ts->work_data), shtps_func_workq );
	INIT_LIST_HEAD( &(ts->queue) );
	spin_lock_init( &(ts->queue_lock) );
#endif /* #if defined( SHTPS_ASYNC_OPEN_ENABLE ) */

	ts->map.fn01.ctrlBase = SHTPS_F01_RMI_CTRL_ADR;
	ts->state_mgr.state = SHTPS_STATE_IDLE;
	ts->loader.ack = 0;
	ts->diag.event = 0;
	ts->facetouch.mode = 0;
	ts->facetouch.state = 0;
	ts->facetouch.off_detect = 0;
	ts->facetouch.wake_sig = 0;
	ts->offset.enabled = 0;
	ts->bt_ver = 0;

#if defined( SHTPS_LOW_POWER_MODE_ENABLE )
	ts->lpmode_state = SHTPS_LPMODE_OFF;
#endif /*` #if defined( SHTPS_LOW_POWER_MODE_ENABLE ) */

#if defined( SHTPS_CHARGER_ARMOR_ENABLE )
	ts->charger_armor_state = 0;
#endif /* #if defined( SHTPS_CHARGER_ARMOR_ENABLE ) */

	memset(&ts->poll_info,   0, sizeof(ts->poll_info));
	memset(&ts->report_info, 0, sizeof(ts->report_info));
	memset(&ts->center_info, 0, sizeof(ts->center_info));
	memset(&ts->touch_state, 0, sizeof(ts->touch_state));
	for(i = 0;i < SHTPS_FINGER_MAX;i++){
		ts->touch_state.dragStep[i][0] = SHTPS_DRAG_THRESHOLD_ZERO;
		ts->touch_state.dragStep[i][1] = SHTPS_DRAG_THRESHOLD_ZERO;
		ts->touch_state.dragStepReturnTime[i][0] = SHTPS_DRAG_THRESH_RETURN_TIME_ZERO;
		ts->touch_state.dragStepReturnTime[i][1] = SHTPS_DRAG_THRESH_RETURN_TIME_ZERO;
	}
	memset(ts->drag_hist, 0, sizeof(ts->drag_hist));

#if defined( CONFIG_SHTPS_SY3000_FACETOUCH_OFF_DETECT )
	wake_lock_init(&ts->facetouch.wake_lock, WAKE_LOCK_SUSPEND, "shtps_wake_lock");
#endif /* #if defined( CONFIG_SHTPS_SY3000_FACETOUCH_OFF_DETECT ) */

#if defined(SHTPS_LPWG_MODE_ENABLE)
	shtps_lpwg_wakelock_init(ts);
	ts->lpwg.tu_rezero_req = 0;
	ts->lpwg.block_touchevent = 0;
#endif /* SHTPS_LPWG_MODE_ENABLE */ 

	/** get platform info */
	gShtps_rmi_spi	= ts;
	ts->spi			= spi;
	ts->input		= input_dev;
	ts->rst_pin		= pdata->gpio_rst;
	ts->irq_mgr.irq	= spi->irq;

	/* setup touchscreen */
	snprintf(ts->phys, sizeof(ts->phys), "%s", dev_name(&spi->dev));
	//
	input_dev->name 		= spi->modalias;
	input_dev->phys         = ts->phys;
	//input_dev->id.bustype = BUS_SPI;
	input_dev->id.vendor	= 0x0001;
	input_dev->id.product	= 0x0002;
	input_dev->id.version	= 0x0100;
	input_dev->dev.parent	= &spi->dev;
	input_dev->open			= shtps_rmi_open;
	input_dev->close		= shtps_rmi_close;

	if(shtps_irq_resuest(ts)){
		SHTPS_LOG_ERR_PRINT("shtps:request_irq error\n");
		mutex_unlock(&shtps_ctrl_lock);
		goto fail_alloc_mem;
	}

	SHTPS_LOG_DBG_PRINT("mutex_unlock()\n");
	mutex_unlock(&shtps_ctrl_lock);

	result = spi_setup(ts->spi);
	if(result < 0){
		SHTPS_LOG_DBG_PRINT("spi_setup fail\n");
	}

	__set_bit(EV_KEY, input_dev->evbit);
#if defined( CONFIG_SHTPS_SY3000_VIRTUAL_KEY )
	__set_bit(KEY_PROG1, input_dev->keybit);
#endif /* #if defined( CONFIG_SHTPS_SY3000_VIRTUAL_KEY ) */
	__set_bit(EV_ABS, input_dev->evbit);
	__set_bit(INPUT_PROP_DIRECT, input_dev->propbit);

	input_set_drvdata(input_dev, ts);

#if defined( SHTPS_MAXFINGER_5_ENABLE )
	input_mt_init_slots(input_dev, 5);
#else
	input_mt_init_slots(input_dev, SHTPS_FINGER_MAX);
#endif

	if(input_dev->mt == NULL){
		result = -ENOMEM;
		goto fail_alloc_mem;
	}

	#if defined(SHTPS_PEN_DETECT_ENABLE)
		input_set_abs_params(input_dev, ABS_MT_TOOL_TYPE, 0, MT_TOOL_MAX, 0, 0);
	#endif /* SHTPS_PEN_DETECT_ENABLE */
	input_set_abs_params(input_dev, ABS_MT_TOUCH_MAJOR, 0, SHTPS_FINGER_WIDTH_PALMDET, 0, 0);
	input_set_abs_params(input_dev, ABS_MT_POSITION_X,  0, CONFIG_SHTPS_SY3000_LCD_SIZE_X - 1, 0, 0);
	input_set_abs_params(input_dev, ABS_MT_POSITION_Y,  0, CONFIG_SHTPS_SY3000_LCD_SIZE_Y - 1, 0, 0);
	input_set_abs_params(input_dev, ABS_MT_PRESSURE,    0, 255, 0, 0);

	result = input_register_device(input_dev);
	if (result)
		goto fail_input_register_device;

	#if defined( SHTPS_PHYSICAL_KEY_ENABLE )
	{
		struct input_dev *input_dev_key;

		input_dev_key = input_allocate_device();
		if(input_dev_key == NULL){
			result = -ENOMEM;
			goto fail_alloc_mem;
		}

		ts->input_key = input_dev_key;

		input_dev_key->name 		= "shtps_key";
		input_dev_key->phys         = ts->phys;
		input_dev_key->id.vendor	= 0x0000;
		input_dev_key->id.product	= 0x0000;
		input_dev_key->id.version	= 0x0000;
		input_dev_key->dev.parent	= &spi->dev;

		__set_bit(EV_KEY, input_dev_key->evbit);

		input_set_drvdata(input_dev_key, ts);

		input_dev_key->keycode = ts->keycodes;
		input_dev_key->keycodemax = SHTPS_PHYSICAL_KEY_NUM;
		input_dev_key->keycodesize = sizeof(ts->keycodes);
		ts->keycodes[SHTPS_PHYSICAL_KEY_DOWN] = KEY_VOLUMEDOWN;
		ts->keycodes[SHTPS_PHYSICAL_KEY_UP] = KEY_VOLUMEUP;
		ts->keycodes[SHTPS_PHYSICAL_KEY_SWEEPON] = KEY_SWEEPON;

		__set_bit(KEY_VOLUMEDOWN, input_dev_key->keybit);
		__set_bit(KEY_VOLUMEUP, input_dev_key->keybit);
#if defined(SHTPS_LPWG_MODE_ENABLE)
	    __set_bit(KEY_SWEEPON, input_dev_key->keybit);
#endif /*  SHTPS_LPWG_MODE_ENABLE */
		__clear_bit(KEY_RESERVED, input_dev_key->keybit);

		input_set_capability(input_dev_key, EV_MSC, MSC_SCAN);

		result = input_register_device(input_dev_key);
		if (result)
			goto fail_input_register_device;

		ts->key_state  = 0;
		ts->diag.event_touchkey = 0;
	}
	#endif /* SHTPS_PHYSICAL_KEY_ENABLE */

	ts->purposebuf_p = (u8 *)kzalloc(SHTPS_SY3X00_SPIBLOCK_BUFSIZE_MAX, GFP_KERNEL);
	if(ts->purposebuf_p == NULL){
		result = -ENOMEM;
		goto fail_alloc_mem_touchbuf;
	}

	#if defined(SHTPS_SYSTEM_BOOT_MODE_CHECK_ENABLE)
		ts->system_boot_mode = sh_boot_get_bootmode();
	#endif /* SHTPS_SYSTEM_BOOT_MODE_CHECK_ENABLE */

	shtps_performance_check_init();

	SHTPS_LOG_DBG_PRINT("shtps_rmi_probe() done\n");
	_log_msg_sync( LOGMSG_ID__PROBE_DONE, "");

#if defined( SHTPS_PHYSICAL_KEY_ENABLE )
    shtps_ioctl_enable (gShtps_rmi_spi);
#endif /* SHTPS_PHYSICAL_KEY_ENABLE */

	return 0;

fail_input_register_device:
fail_alloc_mem_touchbuf:
fail_alloc_mem:
#if defined( CONFIG_SHTPS_SY3000_FACETOUCH_OFF_DETECT )
	wake_lock_destroy(&ts->facetouch.wake_lock);
#endif /* #if defined( CONFIG_SHTPS_SY3000_FACETOUCH_OFF_DETECT ) */

#if defined(SHTPS_LPWG_MODE_ENABLE)
	shtps_lpwg_wakelock_destroy(ts);
#endif /* SHTPS_LPWG_MODE_ENABLE */

	if(input_dev->mt){
		input_mt_destroy_slots(input_dev);
	}
	input_free_device(input_dev);
	#if defined(SHTPS_PHYSICAL_KEY_ENABLE)
		input_free_device(ts->input_key);
	#endif /* SHTPS_PHYSICAL_KEY_ENABLE */
	kfree(ts);
	_log_msg_sync( LOGMSG_ID__PROBE_FAIL, "");
	return result;
}

static int __devexit shtps_rmi_remove(struct spi_device *spi)
{
	struct shtps_rmi_spi *ts = spi_get_drvdata(spi);
	struct shtps_platform_data *pdata = spi->dev.platform_data;

	hrtimer_cancel(&ts->rezero_delayed_timer);

#if defined( SHTPS_ASYNC_OPEN_ENABLE )
	destroy_workqueue(ts->workqueue_p);
#endif /* #if defined( SHTPS_ASYNC_OPEN_ENABLE ) */

#if defined( CONFIG_SHTPS_SY3000_FACETOUCH_OFF_DETECT )
	wake_lock_destroy(&ts->facetouch.wake_lock);
#endif /* #if defined( CONFIG_SHTPS_SY3000_FACETOUCH_OFF_DETECT ) */

#if defined(SHTPS_LPWG_MODE_ENABLE)
	shtps_lpwg_wakelock_destroy(ts);
#endif /* SHTPS_LPWG_MODE_ENABLE */

	free_irq(ts->irq_mgr.irq, ts);
	if (pdata && pdata->teardown){
		pdata->teardown(&spi->dev);
	}

	kfree(ts->purposebuf_p);

	if(ts->input->mt){
		input_mt_destroy_slots(ts->input);
	}
	input_free_device(ts->input);
	#if defined(SHTPS_PHYSICAL_KEY_ENABLE)
		input_free_device(ts->input_key);
	#endif /* SHTPS_PHYSICAL_KEY_ENABLE */
	gShtps_rmi_spi = NULL;
	kfree(ts);

	_log_msg_sync( LOGMSG_ID__REMOVE_DONE, "");
	SHTPS_LOG_DBG_PRINT("shtps_rmi_remove() done\n");
	return 0;
}

static int shtps_rmi_suspend(struct spi_device *spi, pm_message_t mesg)
{
	_log_msg_sync( LOGMSG_ID__SUSPEND, "");
	return 0;
}

static int shtps_rmi_resume(struct spi_device *spi)
{
	_log_msg_sync( LOGMSG_ID__RESUME, "");

	return 0;
}

static struct spi_driver shtps_rmi_driver = {
	.probe		= shtps_rmi_probe,
	.remove		= __devexit_p(shtps_rmi_remove),
	.suspend	= shtps_rmi_suspend,
	.resume		= shtps_rmi_resume,
	.driver		= {
		.name  = SH_TOUCH_DEVNAME,
		.owner = THIS_MODULE,
	},
};

static int __init shtps_rmi_init(void)
{
	SHTPS_LOG_DBG_PRINT("shtps_rmi_init() start\n");
	_log_msg_sync( LOGMSG_ID__INIT, "");

	return spi_register_driver(&shtps_rmi_driver);
}

static void __exit shtps_rmi_exit(void)
{
	spi_unregister_driver(&shtps_rmi_driver);

	SHTPS_LOG_DBG_PRINT("shtps_rmi_exit() done\n");
	_log_msg_sync( LOGMSG_ID__EXIT, "");
}

module_init(shtps_rmi_init);
module_exit(shtps_rmi_exit);

MODULE_DESCRIPTION("SHARP TOUCHPANEL DRIVER MODULE");
MODULE_LICENSE("GPL v2");
MODULE_AUTHOR("SHARP CORPORATION");
MODULE_VERSION("1.00");

/* -----------------------------------------------------------------------------------
 */
void msm_tps_setsleep(int on)
{
	struct shtps_rmi_spi *ts = gShtps_rmi_spi;

	_log_msg_sync( LOGMSG_ID__API_SLEEP, "%d", on);
	if(ts){
		if(on){
			shtps_device_access_teardown(ts);
			request_event(ts, SHTPS_EVENT_SLEEP, 0);
		}else{
			request_event(ts, SHTPS_EVENT_WAKEUP, 0);
		}
	}
	_log_msg_sync( LOGMSG_ID__API_SLEEP_DONE, "");
}
EXPORT_SYMBOL(msm_tps_setsleep);

void shtps_setFlipInformation(int state)
{
#if defined( SHTPS_VKEY_CANCEL_ENABLE )
	struct shtps_rmi_spi *ts = gShtps_rmi_spi;
	request_event(ts, SHTPS_EVENT_FORMCHANGE, state);
#endif /* #if defined( SHTPS_VKEY_CANCEL_ENABLE ) */
}
EXPORT_SYMBOL(shtps_setFlipInformation);

#if defined( SHTPS_BOOT_FWUPDATE_ENABLE ) || defined( SHTPS_CHECK_CRC_ERROR_ENABLE )
static int shtps_fw_update(struct shtps_rmi_spi *ts, const unsigned char *fw_data)
{
	int i;
	unsigned long blockSize;
	unsigned long blockNum;

	_log_msg_sync( LOGMSG_ID__BOOT_FW_UPDATE, "");

	if(0 != shtps_enter_bootloader(ts)){
		SHTPS_LOG_ERR_PRINT("error - shtps_enter_bootloader()\n");
		_log_msg_sync( LOGMSG_ID__BOOT_FW_UPDATE_FAIL, "0");
		return -1;
	}

	if(0 != shtps_lockdown_bootloader(ts, (u8*)&fw_data[0x00d0])){
		SHTPS_LOG_ERR_PRINT("error - shtps_lockdown_bootloader()\n");
		_log_msg_sync( LOGMSG_ID__BOOT_FW_UPDATE_FAIL, "1");
		return -1;
	}

	if(0 != shtps_flash_erase(ts)){
		SHTPS_LOG_ERR_PRINT("error - shtps_flash_erase()\n");
		_log_msg_sync( LOGMSG_ID__BOOT_FW_UPDATE_FAIL, "2");
		return -1;
	}

	blockSize = F34_QUERY_BLOCKSIZE(ts->map.fn34.query.data);
	blockNum  = F34_QUERY_FIRMBLOCKCOUNT(ts->map.fn34.query.data);

	for(i = 0;i < blockNum;i++){
		if(0 != shtps_flash_writeImage(ts, (u8*)&fw_data[0x0100 + i * blockSize])){
			SHTPS_LOG_ERR_PRINT("error - shtps_flash_writeImage(%d)\n", i);
			_log_msg_sync( LOGMSG_ID__BOOT_FW_UPDATE_FAIL, "3");
			return -1;
		}
	}

	if(0 != shtps_flash_writeConfig(ts, (u8*)&fw_data[0x0100 + (blockNum * blockSize)])){
		SHTPS_LOG_ERR_PRINT("error - shtps_flash_writeConfig()\n");
		_log_msg_sync( LOGMSG_ID__BOOT_FW_UPDATE_FAIL, "4");
		return -1;
	}
	if(0 != shtps_exit_bootloader(ts)){
		SHTPS_LOG_ERR_PRINT("error - shtps_exit_bootloader()\n");
		_log_msg_sync( LOGMSG_ID__BOOT_FW_UPDATE_FAIL, "5");
		return -1;
	}
	printk(KERN_DEBUG "[shtps] fw update done\n");
	_log_msg_sync( LOGMSG_ID__BOOT_FW_UPDATE_DONE, "");

	return 0;
}
#endif /* #if defined( SHTPS_BOOT_FWUPDATE_ENABLE ) || defined( SHTPS_CHECK_CRC_ERROR_ENABLE ) */
