/* drivers/sharp/shtps/nassau/xxx/shtps_nas_spi.c
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
 */

#include <linux/kernel.h>
#include <linux/platform_device.h>
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
#include <linux/regulator/consumer.h>

#include <sharp/shtps_dev.h>
#include <sharp/sh_smem.h>
#include <sharp/sh_boot_manager.h>

#include <mach/perflock.h>

/* -----------------------------------------------------------------------------------
 */
//#define SHTPS_DEVELOP_MODE_ENABLE
//#define SHTPS_FACTORY_MODE_ENABLE

#define SHTPS_LOG_ERROR_ENABLE

#if defined( SHTPS_DEVELOP_MODE_ENABLE )
	#define SHTPS_LOG_EVENT_ENABLE
	#define SHTPS_LOG_DEBUG_ENABLE
	//#define SHTPS_LOG_SPI_DEBUG_ENABLE	
	#define SHTPS_MODULE_PARAM_ENABLE
	//#define SHTPS_DEBUG_PERFORMANCE_CHECK_ENABLE
	#define SHTPS_DEBUG_CREATE_KOBJ_ENABLE
#endif /* #if defined( SHTPS_DEVELOP_MODE_ENABLE ) */

#if defined( SHTPS_FACTORY_MODE_ENABLE )

#else
	#define SHTPS_BOOT_FWUPDATE_ENABLE
	#undef SHTPS_BOOT_FWUPDATE_FORCE_UPDATE
	#define SHTPS_BOOT_FWUPDATE_NOT_LOADER_UPDATE
	#undef SHTPS_BOOT_FWUPDATE_FORCE_LOADER_UPDATE
#endif /* #if defined( SHTPS_FACTORY_MODE_ENABLE ) */

#define	SHTPS_ASYNC_OPEN_ENABLE
#define SHTPS_SPICLOCK_CONTROL_ENABLE
#define SHTPS_POSITION_DIRECT_REPORT_ENABLE
#define SHTPS_SYSTEM_BOOT_MODE_CHECK_ENABLE
#define SHTPS_PEN_DETECT_ENABLE
#define SHTPS_PEN_HAND_DETECT_ENABLE
#define SHTPS_RESET_RECOVERY_ENABLE
#define SHTPS_REBOOT_RECOVERY_ENABLE
#define SHTPS_DRAG_STEP_ENABLE
#define SHTPS_POWER_MODE_CONTROL_ENABLE
#define SHTPS_CPU_CLOCK_CONTROL_ENABLE

#define SHTPS_PEN_DETECT_THRESHOLD_ENABLE
#define SHTPS_TESTMODE_120HZ_WAIT_ENABLE
#define SHTPS_TESTMODE_SLEEP_WAIT_ENABLE
#define SHTPS_TESTMODE_ENTER_WAIT_ENABLE
#define SHTPS_TESTMODE_NEW_METHOD_ENABLE

#if defined( SHTPS_PEN_DETECT_ENABLE ) && defined( SHTPS_PEN_DETECT_THRESHOLD_ENABLE )
	#define SHTPS_PEN_DETECT_THRESHOLD_VARIABLE
#endif	/* #if defined( SHTPS_PEN_DETECT_ENABLE ) */

#if defined( SHTPS_TESTMODE_120HZ_WAIT_ENABLE )
	#define SHTPS_TESTMODE_120HZ_WAIT_VARIABLE
#endif	/* #if defined( SHTPS_TESTMODE_120HZ_WAIT_ENABLE ) */

#if defined( SHTPS_TESTMODE_SLEEP_WAIT_ENABLE )
	#define SHTPS_TESTMODE_SLEEP_WAIT_VARIABLE
#endif	/* #if defined( SHTPS_TESTMODE_SLEEP_WAIT_ENABLE ) */

#if defined( SHTPS_TESTMODE_ENTER_WAIT_ENABLE )
	#define SHTPS_TESTMODE_ENTER_WAIT_VARIABLE
#endif	/* #if defined( SHTPS_TESTMODE_ENTER_WAIT_ENABLE ) */


/* -----------------------------------------------------------------------------------
 */
#if defined( SHTPS_LOG_ERROR_ENABLE )
	#define SHTPS_LOG_ERR_PRINT(...)	printk(KERN_ERR "[shtps] " __VA_ARGS__)
#else
	#define SHTPS_LOG_ERR_PRINT(...)
#endif /* defined( SHTPS_LOG_ERROR_ENABLE ) */

#if defined( SHTPS_LOG_DEBUG_ENABLE )
	#define SHTPS_LOG_DBG_PRINT(...)	\
		if(LogOutputEnable > 1)		printk(KERN_DEBUG "[shtps] " __VA_ARGS__)
#else
	#define	SHTPS_LOG_DBG_PRINT(...)
#endif /* defined( SHTPS_LOG_DEBUG_ENABLE ) */

#if defined( SHTPS_LOG_SPI_DEBUG_ENABLE )
	#define	SHTPS_LOG_SPI_DBG_PRINT(...)	printk(KERN_DEBUG "[shtps] " __VA_ARGS__)
#else
	#define	SHTPS_LOG_SPI_DBG_PRINT(...)
#endif /* defined( SHTPS_LOG_DEBUG_ENABLE ) */

#if defined( SHTPS_LOG_EVENT_ENABLE )
	#define SHTPS_LOG_EVT_PRINT(...)	\
		if(LogOutputEnable > 0)		printk(KERN_DEBUG "[shtps] " __VA_ARGS__)
#else
	#define SHTPS_LOG_EVT_PRINT(...)
#endif /* defined( SHTPS_LOG_EVENT_ENABLE ) */


/* -----------------------------------------------------------------------------------
 */
#if defined( CONFIG_SHTPS_NASSAU_TS2_001 )
	#include "shtps_def_ts2-001.h"
	#include "shtps_fw_ts2-001.h"

	#include "shtps_ldr_ts2-001.h"
	#include "shtps_fw_old_ts2-001.h"
	#include "shtps_ldr_old_ts2-001.h"
#else
	#undef  SHTPS_BOOT_FWUPDATE_ENABLE
	#define SHTPS_FWVER_NEWER     0x00000000
	#define SHTPS_PARAMVER_NEWER  0x00000000
#endif /* #if defined( CONFIG_SHTPS_NASSAU_TS2_001 ) */

/* -----------------------------------------------------------------------------------
 */
#if defined( SHTPS_POSITION_DIRECT_REPORT_ENABLE )
	#define SHTPS_POS_SCALE_CONVERT_X(x)	x
	#define SHTPS_POS_SCALE_CONVERT_Y(y)	y
#else
	#define SHTPS_POS_SCALE_CONVERT_X(x) \
		(x <= SHTPS_NASSAU_DEV_POS_OFFSET_X)? 0 : \
			( ((x - SHTPS_NASSAU_DEV_POS_OFFSET_X) * CONFIG_SHTPS_NASSAU_PANEL_SIZE_X) \
				/ (SHTPS_NASSAU_DEV_POS_MAX_X - SHTPS_NASSAU_DEV_POS_OFFSET_X) )

	#define SHTPS_POS_SCALE_CONVERT_Y(y) \
		(y <= SHTPS_NASSAU_DEV_POS_OFFSET_Y)? 0 : \
			( ((y - SHTPS_NASSAU_DEV_POS_OFFSET_Y) * CONFIG_SHTPS_NASSAU_PANEL_SIZE_Y) \
				/ (SHTPS_NASSAU_DEV_POS_MAX_Y - SHTPS_NASSAU_DEV_POS_OFFSET_Y) )
#endif /* #if defined( SHTPS_POSITION_DIRECT_REPORT_ENABLE ) */

/* -----------------------------------------------------------------------------------
 */
#define SHTPS_POWEROFF_WAIT_MS 500
#define SHTPS_FW_SLEEP_WAIT_MS				1000

#define SHTPS_PEN_DETECT_THRESHOLD			630

#define SHTPS_DCMAP_FIXED_MODE_WAIT_MS		100
#define SHTPS_DCMAP_FORCE_CALIBRATION_WAIT_MS			5
#define SHTPS_DCMAP_FORCE_CALIBRATION_CNT_MAX			4

#define SHTPS_TESTMODE_120HZ_WAIT_MS		20
#define SHTPS_TESTMODE_SLEEP_WAIT_MS		20
#define SHTPS_TESTMODE_ENTER_WAIT_MS		30

#define SHTPS_TESTMODE_OPT_NONE							(0x00)
#define SHTPS_TESTMODE_OPT_CALIBRATION_BLOCK_BYPASS		(0x01)
#define SHTPS_TESTMODE_OPT_120HZ						(0x02)
#define SHTPS_TESTMODE_OPT_FIXED_MODE_HEAD				(0x04)
#define SHTPS_TESTMODE_OPT_FIXED_MODE_BACK				(0x08)

#define SHTPS_BOOT_FWUPDATE_RETRY_MAX					3

#define SHTPS_NEW_BOOTLOADER_FW_VER_MINIMUM				0x0041

#define SHTPS_WAIT_STARTUP_TIMEOUT_MS		1000

#define SHTPS_PEN_HAND_COORDINATES_X		0
#define SHTPS_PEN_HAND_COORDINATES_Y		9999

#define SHTPS_INT_CHECK_MAX					8

#if defined(SHTPS_CPU_CLOCK_CONTROL_ENABLE)
	#define SHTPS_PERF_LOCK_ENABLE_TIME_MS		50
#endif /* SHTPS_CPU_CLOCK_CONTROL_ENABLE */

/* -----------------------------------------------------------------------------------
 */
static DEFINE_MUTEX(shtps_ctrl_lock);

#if defined( SHTPS_POWER_MODE_CONTROL_ENABLE )
	static DEFINE_MUTEX(shtps_power_mode_ctrl_lock);
#endif /* #if defined( SHTPS_POWER_MODE_CONTROL_ENABLE ) */

#if defined(SHTPS_CPU_CLOCK_CONTROL_ENABLE)
	static DEFINE_MUTEX(shtps_cpu_clock_ctrl_lock);
#endif /* SHTPS_CPU_CLOCK_CONTROL_ENABLE */

/* -----------------------------------------------------------------------------------
 */
struct shtps_nas_spi;

struct shtps_irq_info{
	int							irq;
	u8							state;
	u8							wake;
};

struct shtps_diag_info{
	int							ready;
	int							cancel;
	wait_queue_head_t			wait;
};

struct shtps_testmode_info{
	int							ready;
	int							cancel;
	u8							opt;
	wait_queue_head_t			wait;
	u8							clb_cnt;
};

typedef int (shtps_command_exec_func)(struct shtps_nas_spi *ts);
struct shtps_command_info{
	shtps_command_exec_func*	func;
	u8							result[SHTPS_NASSAU_CMD_RESULT_MAX_SIZE];
	u8							prevstate;
	int							ready;
	int							cancel;
	wait_queue_head_t			wait;
};

struct shtps_offset_info{
	int							enabled;
	u16							base[5];
	signed short				diff[12];
};

struct shtps_loader_info{
	int							ready;
	int							cancel;
	wait_queue_head_t			wait;
};

struct shtps_state_info{
	int							state;
	int							mode;
	int							starterr;
	unsigned long				starttime;
};

struct shtps_drag_hist{
	int							pre;
	u8							dir;
	u8							count;
};

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

struct shtps_nas_spi {
	char						phys[32];
	struct spi_device*			spi;
	struct input_dev*			input;
	int							pow_pin;
	int							rst_pin;
	struct shtps_irq_info		irq_mgr;
	
	struct shtps_state_info		state_mgr;
	u16							pending;
	
	struct shtps_touch_info		report_info;

	struct shtps_diag_info		diag;
	struct shtps_testmode_info	testmode;
	struct shtps_loader_info	loader;
	struct shtps_command_info	command;
	struct shtps_offset_info	offset;
	struct shtps_offset_info	offset_pen;
	u8							is_lcd_on;
	u8							fw_startup_state;

	int (*v17_power)(int on);

	#if defined(SHTPS_SYSTEM_BOOT_MODE_CHECK_ENABLE)
		u16							system_boot_mode;
	#endif /* SHTPS_SYSTEM_BOOT_MODE_CHECK_ENABLE */

	u32							bt_ver;
	u32							bt_paramver;
	u8							bt_select;
	u8							fw_update_continue;

	wait_queue_head_t			wait_start;
	#if defined( SHTPS_ASYNC_OPEN_ENABLE )
		struct workqueue_struct		*workqueue_p;
		struct work_struct			work_data;
		struct list_head			queue;
		spinlock_t					queue_lock;
		struct shtps_req_msg		*cur_msg_p;
	#endif /* #if defined( SHTPS_ASYNC_OPEN_ENABLE ) */

	#if defined(SHTPS_DRAG_STEP_ENABLE)
		struct shtps_touch_info		center_info;
		struct shtps_touch_state	touch_state;
		struct shtps_drag_hist		drag_hist[SHTPS_FINGER_MAX][2];
	#endif /* SHTPS_DRAG_STEP_ENABLE */

	#if defined( SHTPS_POWER_MODE_CONTROL_ENABLE )
		u8							power_mode_state;
	#endif /* SHTPS_POWER_MODE_CONTROL_ENABLE */

	#if defined(SHTPS_CPU_CLOCK_CONTROL_ENABLE)
		struct perf_lock			perf_lock_level_highest;
		struct perf_lock			perf_lock_level_high;
		struct perf_lock			perf_lock_level_low;
		struct delayed_work			perf_lock_disable_delayed_work;
		int							perf_lock_enable_time_ms;
		int							perf_lock_level;
		int							report_event;
	#endif /* SHTPS_CPU_CLOCK_CONTROL_ENABLE */
};

typedef int (shtps_state_func)(struct shtps_nas_spi *ts, int param);
struct shtps_state_func {
	shtps_state_func	*enter;
	shtps_state_func	*start;
	shtps_state_func	*stop;
	shtps_state_func	*sleep;
	shtps_state_func	*wakeup;
	shtps_state_func	*start_ldr;
	shtps_state_func	*stop_ldr;
	shtps_state_func	*start_tm;
	shtps_state_func	*stop_tm;
	shtps_state_func	*cmd_request;
	shtps_state_func	*interrupt;
	shtps_state_func	*timeout;
};

/* -----------------------------------------------------------------------------------
 */
static struct shtps_nas_spi 	*g_shtps_nas_spi = NULL;
static dev_t 					shtpsif_devid;
static struct class*			shtpsif_class;
static struct device*			shtpsif_device;
struct cdev 					shtpsif_cdev;

/* -----------------------------------------------------------------------------------
 */
#if defined( SHTPS_LOG_EVENT_ENABLE ) || defined(SHTPS_LOG_DEBUG_ENABLE)
	static int LogOutputEnable = 0;
	module_param(LogOutputEnable, int, S_IRUGO | S_IWUSR);
#endif /* SHTPS_LOG_EVENT_ENABLE || SHTPS_LOG_DEBUG_ENABLE */

#if defined( SHTPS_MODULE_PARAM_ENABLE )
	static int shtps_irq_wake_state = 0;
	static int shtps_spi_clk_ctrl_state = 0;
	static int shtps_rezero_state = 0;

	module_param(shtps_irq_wake_state, int, S_IRUGO);
	module_param(shtps_spi_clk_ctrl_state, int, S_IRUGO);
	module_param(shtps_rezero_state, int, S_IRUGO | S_IWUSR);
#endif /* SHTPS_MODULE_PARAM_ENABLE */

#if defined( SHTPS_PEN_DETECT_THRESHOLD_VARIABLE )
	static int shtps_pen_detect_pressure_threshold = SHTPS_PEN_DETECT_THRESHOLD;
	#if defined( SHTPS_MODULE_PARAM_ENABLE )
		module_param(shtps_pen_detect_pressure_threshold, int, S_IRUGO | S_IWUSR);
	#endif /* SHTPS_MODULE_PARAM_ENABLE */
#endif /* #if defined( SHTPS_PEN_DETECT_THRESHOLD_VARIABLE ) */

#if defined( SHTPS_TESTMODE_120HZ_WAIT_VARIABLE )
	static int shtps_testmode_120hz_wait_ms = SHTPS_TESTMODE_120HZ_WAIT_MS;
	#if defined( SHTPS_MODULE_PARAM_ENABLE )
		module_param(shtps_testmode_120hz_wait_ms, int, S_IRUGO | S_IWUSR);
	#endif /* SHTPS_MODULE_PARAM_ENABLE */
#endif /* #if defined( SHTPS_TESTMODE_120HZ_WAIT_VARIABLE ) */

#if defined( SHTPS_TESTMODE_SLEEP_WAIT_VARIABLE )
	static int shtps_testmode_sleep_wait_ms = SHTPS_TESTMODE_SLEEP_WAIT_MS;
	#if defined( SHTPS_MODULE_PARAM_ENABLE )
		module_param(shtps_testmode_sleep_wait_ms, int, S_IRUGO | S_IWUSR);
	#endif /* SHTPS_MODULE_PARAM_ENABLE */
#endif /* #if defined( SHTPS_TESTMODE_SLEEP_WAIT_VARIABLE ) */

#if defined( SHTPS_TESTMODE_ENTER_WAIT_VARIABLE )
	static int shtps_testmode_enter_wait_ms = SHTPS_TESTMODE_ENTER_WAIT_MS;
	#if defined( SHTPS_MODULE_PARAM_ENABLE )
		module_param(shtps_testmode_enter_wait_ms, int, S_IRUGO | S_IWUSR);
	#endif /* SHTPS_MODULE_PARAM_ENABLE */
#endif /* #if defined( SHTPS_TESTMODE_ENTER_WAIT_VARIABLE ) */

#if defined(SHTPS_DRAG_STEP_ENABLE)
	static int SHTPS_DRAG_DIR_FIX_CNT =				3;

	#if defined( SHTPS_MODULE_PARAM_ENABLE )
		module_param(SHTPS_DRAG_DIR_FIX_CNT, int, S_IRUGO | S_IWUSR);
	#endif /* SHTPS_MODULE_PARAM_ENABLE */

	static int SHTPS_DRAG_THRESH_VAL_X_ZERO = 		CONFIG_SHTPS_NASSAU_SINGLE_ZERO_DRSTEP;
	static int SHTPS_DRAG_THRESH_VAL_Y_ZERO = 		CONFIG_SHTPS_NASSAU_SINGLE_ZERO_DRSTEP;
	static int SHTPS_PEN_DRAG_THRESH_VAL_X_ZERO = 	CONFIG_SHTPS_NASSAU_PEN_SINGLE_ZERO_DRSTEP;
	static int SHTPS_PEN_DRAG_THRESH_VAL_Y_ZERO = 	CONFIG_SHTPS_NASSAU_PEN_SINGLE_ZERO_DRSTEP;
	static int SHTPS_DRAG_THRESH_RETURN_TIME_ZERO =	250;

	static int SHTPS_DRAG_THRESH_VAL_X_1ST = 		CONFIG_SHTPS_NASSAU_SINGLE_1ST_DRSTEP;
	static int SHTPS_DRAG_THRESH_VAL_X_2ND = 		CONFIG_SHTPS_NASSAU_SINGLE_2ND_DRSTEP;
	static int SHTPS_DRAG_THRESH_VAL_X_1ST_MULTI =	CONFIG_SHTPS_NASSAU_MULTI_1ST_DRSTEP;
	static int SHTPS_DRAG_THRESH_VAL_X_2ND_MULTI =	CONFIG_SHTPS_NASSAU_MULTI_2ND_DRSTEP;
	static int SHTPS_DRAG_THRESH_VAL_Y_1ST = 		CONFIG_SHTPS_NASSAU_SINGLE_1ST_DRSTEP;
	static int SHTPS_DRAG_THRESH_VAL_Y_2ND = 		CONFIG_SHTPS_NASSAU_SINGLE_2ND_DRSTEP;
	static int SHTPS_DRAG_THRESH_VAL_Y_1ST_MULTI =	CONFIG_SHTPS_NASSAU_MULTI_1ST_DRSTEP;
	static int SHTPS_DRAG_THRESH_VAL_Y_2ND_MULTI =	CONFIG_SHTPS_NASSAU_MULTI_2ND_DRSTEP;

	static int SHTPS_PEN_DRAG_THRESH_VAL_X_1ST = 		CONFIG_SHTPS_NASSAU_PEN_SINGLE_1ST_DRSTEP;
	static int SHTPS_PEN_DRAG_THRESH_VAL_X_2ND = 		CONFIG_SHTPS_NASSAU_PEN_SINGLE_2ND_DRSTEP;
	static int SHTPS_PEN_DRAG_THRESH_VAL_X_1ST_MULTI =	CONFIG_SHTPS_NASSAU_PEN_MULTI_1ST_DRSTEP;
	static int SHTPS_PEN_DRAG_THRESH_VAL_X_2ND_MULTI =	CONFIG_SHTPS_NASSAU_PEN_MULTI_2ND_DRSTEP;
	static int SHTPS_PEN_DRAG_THRESH_VAL_Y_1ST = 		CONFIG_SHTPS_NASSAU_PEN_SINGLE_1ST_DRSTEP;
	static int SHTPS_PEN_DRAG_THRESH_VAL_Y_2ND = 		CONFIG_SHTPS_NASSAU_PEN_SINGLE_2ND_DRSTEP;
	static int SHTPS_PEN_DRAG_THRESH_VAL_Y_1ST_MULTI =	CONFIG_SHTPS_NASSAU_PEN_MULTI_1ST_DRSTEP;
	static int SHTPS_PEN_DRAG_THRESH_VAL_Y_2ND_MULTI =	CONFIG_SHTPS_NASSAU_PEN_MULTI_2ND_DRSTEP;

	static int SHTPS_DRAG_THRESH_RETURN_TIME =		250;

	#if defined( SHTPS_MODULE_PARAM_ENABLE )
		module_param(SHTPS_DRAG_THRESH_VAL_X_ZERO,       int, S_IRUGO | S_IWUSR);
		module_param(SHTPS_DRAG_THRESH_VAL_Y_ZERO,       int, S_IRUGO | S_IWUSR);
		module_param(SHTPS_PEN_DRAG_THRESH_VAL_X_ZERO,   int, S_IRUGO | S_IWUSR);
		module_param(SHTPS_PEN_DRAG_THRESH_VAL_Y_ZERO,   int, S_IRUGO | S_IWUSR);
		module_param(SHTPS_DRAG_THRESH_RETURN_TIME_ZERO, int, S_IRUGO | S_IWUSR);

		module_param(SHTPS_DRAG_THRESH_VAL_X_1ST,        int, S_IRUGO | S_IWUSR);
		module_param(SHTPS_DRAG_THRESH_VAL_X_2ND,        int, S_IRUGO | S_IWUSR);
		module_param(SHTPS_DRAG_THRESH_VAL_X_1ST_MULTI,  int, S_IRUGO | S_IWUSR);
		module_param(SHTPS_DRAG_THRESH_VAL_X_2ND_MULTI,  int, S_IRUGO | S_IWUSR);
		module_param(SHTPS_DRAG_THRESH_VAL_Y_1ST,        int, S_IRUGO | S_IWUSR);
		module_param(SHTPS_DRAG_THRESH_VAL_Y_2ND,        int, S_IRUGO | S_IWUSR);
		module_param(SHTPS_DRAG_THRESH_VAL_Y_1ST_MULTI,  int, S_IRUGO | S_IWUSR);
		module_param(SHTPS_DRAG_THRESH_VAL_Y_2ND_MULTI,  int, S_IRUGO | S_IWUSR);

		module_param(SHTPS_PEN_DRAG_THRESH_VAL_X_1ST,       int, S_IRUGO | S_IWUSR);
		module_param(SHTPS_PEN_DRAG_THRESH_VAL_X_2ND,       int, S_IRUGO | S_IWUSR);
		module_param(SHTPS_PEN_DRAG_THRESH_VAL_X_1ST_MULTI, int, S_IRUGO | S_IWUSR);
		module_param(SHTPS_PEN_DRAG_THRESH_VAL_X_2ND_MULTI, int, S_IRUGO | S_IWUSR);
		module_param(SHTPS_PEN_DRAG_THRESH_VAL_Y_1ST,       int, S_IRUGO | S_IWUSR);
		module_param(SHTPS_PEN_DRAG_THRESH_VAL_Y_2ND,       int, S_IRUGO | S_IWUSR);
		module_param(SHTPS_PEN_DRAG_THRESH_VAL_Y_1ST_MULTI, int, S_IRUGO | S_IWUSR);
		module_param(SHTPS_PEN_DRAG_THRESH_VAL_Y_2ND_MULTI, int, S_IRUGO | S_IWUSR);

		module_param(SHTPS_DRAG_THRESH_RETURN_TIME,      int, S_IRUGO | S_IWUSR);
	#endif /* SHTPS_MODULE_PARAM_ENABLE */
#endif /* SHTPS_DRAG_STEP_ENABLE */

/* -----------------------------------------------------------------------------------
 */
enum{
	SHTPS_IRQ_WAKE_DISABLE,
	SHTPS_IRQ_WAKE_ENABLE,
};

enum{
	SHTPS_IRQ_STATE_DISABLE,
	SHTPS_IRQ_STATE_ENABLE,
};

enum{
	SHTPS_PENDING_FLAG_WAKEUP 			= 0x0001,
	SHTPS_PENDING_FLAG_SLEEP			= 0x0002,
	SHTPS_PENDING_FLAG_STARTLOADER  	= 0x0004,
	SHTPS_PENDING_FLAG_STARTTESTMODE  	= 0x0008,
	SHTPS_PENDING_FLAG_NO_ID_ATTACH	  	= 0x0010,
};

enum{
	SHTPS_EVENT_START,
	SHTPS_EVENT_STOP,
	SHTPS_EVENT_SLEEP,
	SHTPS_EVENT_WAKEUP,
	SHTPS_EVENT_STARTLOADER,
	SHTPS_EVENT_STOPLOADER,
	SHTPS_EVENT_STARTTM,
	SHTPS_EVENT_STOPTM,
	SHTPS_EVENT_CMDREQUEST,
	SHTPS_EVENT_INTERRUPT,
	SHTPS_EVENT_TIMEOUT,
};

enum{
	SHTPS_STATE_POWEROFF,
	SHTPS_STATE_RESETTING,
	SHTPS_STATE_ATTACH,
	SHTPS_STATE_POWERON,
	SHTPS_STATE_SLEEPING,
	SHTPS_STATE_SLEEP,
	SHTPS_STATE_ACTIVATING,
	SHTPS_STATE_ACTIVE,
	SHTPS_STATE_WAIT_TESTMODE,
	SHTPS_STATE_TESTMODE,
	SHTPS_STATE_WAIT_LOADER,
	SHTPS_STATE_LOADER,
	SHTPS_STATE_CMD_EXECUTING,
};

enum{
	SHTPS_INPUT_TYPE_NONE	= 0x0000,
	SHTPS_INPUT_TYPE_FINGER	= 0x0001,
	SHTPS_INPUT_TYPE_PEN	= 0x0002,
	SHTPS_INPUT_TYPE_HOVER	= 0x0003,
};

enum {
	SHTPS_EVENT_TYPE_NONE					= 0x00,
	SHTPS_EVENT_TYPE_TOUCHDOWN				= 0x01,
	SHTPS_EVENT_TYPE_TOUCHUP_BY_ID_INVALID	= 0x02,
	SHTPS_EVENT_TYPE_TOUCHUP_BY_STATE		= 0x03,
	SHTPS_EVENT_TYPE_DRAG					= 0x04,
};

enum {
	SHTPS_FW_STARTUP_LOADER = 0,
	SHTPS_FW_STARTUP_PROGRAM,
	SHTPS_FW_STARTUP_SLEEP,
	SHTPS_FW_STARTUP_CRC_ERROR,
	SHTPS_FW_STARTUP_UNKNOWN,
};

enum {
	SHTPS_BOOTLOADER_OLD = 0,
	SHTPS_BOOTLOADER_NEW,
};

#if defined( SHTPS_ASYNC_OPEN_ENABLE )
enum{
	SHTPS_FUNC_REQ_EVEMT_OPEN = 0,
	SHTPS_FUNC_REQ_EVEMT_CLOSE,
	SHTPS_FUNC_REQ_EVEMT_ENABLE,
	SHTPS_FUNC_REQ_EVEMT_DISABLE,
};
#endif /* #if defined( SHTPS_ASYNC_OPEN_ENABLE ) */

#if defined(SHTPS_DRAG_STEP_ENABLE)
enum shtps_drag_threshold_mode {
	SHTPS_DRAG_THRESHOLD_ZERO,
	SHTPS_DRAG_THRESHOLD_1ST,
	SHTPS_DRAG_THRESHOLD_2ND
};

enum {
	SHTPS_POSTYPE_X = 0,
	SHTPS_POSTYPE_Y
};

enum{
	SHTPS_EVENT_TU,
	SHTPS_EVENT_TD,
	SHTPS_EVENT_DRAG,
	SHTPS_EVENT_MTDU,
};

enum{
	SHTPS_DRAG_DIR_NONE = 0,
	SHTPS_DRAG_DIR_PLUS,
	SHTPS_DRAG_DIR_MINUS,
};
#endif /* SHTPS_DRAG_STEP_ENABLE */

#if defined( SHTPS_POWER_MODE_CONTROL_ENABLE )
enum{
	SHTPS_NORMAL_POWER_MODE				= 0x00,
	SHTPS_LPMODE_NON_CONTINUOUS_REQ_ON	= 0x01,
	SHTPS_LPMODE_CONTINUOUS_REQ_ON		= 0x02,
	SHTPS_HPMODE_NON_CONTINUOUS_REQ_ON	= 0x04,
	SHTPS_HPMODE_CONTINUOUS_REQ_ON		= 0x08,

	SHTPS_LPMODE_ON						= ( SHTPS_LPMODE_NON_CONTINUOUS_REQ_ON
											| SHTPS_LPMODE_CONTINUOUS_REQ_ON),
	SHTPS_HPMODE_ON						= ( SHTPS_HPMODE_NON_CONTINUOUS_REQ_ON
											| SHTPS_HPMODE_CONTINUOUS_REQ_ON),
};
#endif /* #if defined( SHTPS_POWER_MODE_CONTROL_ENABLE ) */

#if defined(SHTPS_CPU_CLOCK_CONTROL_ENABLE)
enum{
	SHTPS_PERF_LOCK_LEVEL_HIGHEST = 0,
	SHTPS_PERF_LOCK_LEVEL_HIGH,
	SHTPS_PERF_LOCK_LEVEL_LOW,
};
#endif /* SHTPS_CPU_CLOCK_CONTROL_ENABLE */

/* -----------------------------------------------------------------------------------
 */
static int request_event(struct shtps_nas_spi *ts, int event, int param);
static int state_change(struct shtps_nas_spi *ts, int state);
static int shtps_statef_nop(struct shtps_nas_spi *ts, int param);
static int shtps_statef_cmn_stop(struct shtps_nas_spi *ts, int param);
static int shtps_statef_cmn_sleep_as_stop(struct shtps_nas_spi *ts, int param);
static int shtps_statef_cmn_sleep(struct shtps_nas_spi *ts, int param);
static int shtps_statef_cmn_wakeup(struct shtps_nas_spi *ts, int param);
static int shtps_statef_cmn_startldr(struct shtps_nas_spi *ts, int param);
static int shtps_statef_cmn_stopldr(struct shtps_nas_spi *ts, int param);
static int shtps_statef_cmn_starttm(struct shtps_nas_spi *ts, int param);
static int shtps_statef_cmn_stoptm(struct shtps_nas_spi *ts, int param);
static int shtps_statef_cmn_error(struct shtps_nas_spi *ts, int param);
static int shtps_statef_powoff_start(struct shtps_nas_spi *ts, int param);
static int shtps_statef_powoff_wakeup_start(struct shtps_nas_spi *ts, int param);
static int shtps_statef_powoff_startldr(struct shtps_nas_spi *ts, int param);
static int shtps_statef_powoff_starttm(struct shtps_nas_spi *ts, int param);
static int shtps_statef_resetting_enter(struct shtps_nas_spi *ts, int param);
static int shtps_statef_resetting_int(struct shtps_nas_spi *ts, int param);
static int shtps_statef_attach_int(struct shtps_nas_spi *ts, int param);
static int shtps_statef_powon_int(struct shtps_nas_spi *ts, int param);
static int shtps_statef_sleeping_int(struct shtps_nas_spi *ts, int param);
static int shtps_statef_sleep_enter(struct shtps_nas_spi *ts, int param);
static int shtps_statef_sleep_wakeup(struct shtps_nas_spi *ts, int param);
static int shtps_statef_sleep_startldr(struct shtps_nas_spi *ts, int param);
static int shtps_statef_sleep_starttm(struct shtps_nas_spi *ts, int param);
static int shtps_statef_sleep_command(struct shtps_nas_spi *ts, int param);
static int shtps_statef_sleep_int(struct shtps_nas_spi *ts, int param);
static int shtps_statef_activating_int(struct shtps_nas_spi *ts, int param);
static int shtps_statef_active_enter(struct shtps_nas_spi *ts, int param);
static int shtps_statef_active_sleep(struct shtps_nas_spi *ts, int param);
static int shtps_statef_active_startldr(struct shtps_nas_spi *ts, int param);
static int shtps_statef_active_starttm(struct shtps_nas_spi *ts, int param);
static int shtps_statef_active_command(struct shtps_nas_spi *ts, int param);
static int shtps_statef_active_int(struct shtps_nas_spi *ts, int param);
static int shtps_statef_wait_testmode_enter(struct shtps_nas_spi *ts, int param);
static int shtps_statef_wait_testmode_int(struct shtps_nas_spi *ts, int param);
static int shtps_statef_testmode_enter(struct shtps_nas_spi *ts, int param);
static int shtps_statef_testmode_stoptm(struct shtps_nas_spi *ts, int param);
static int shtps_statef_testmode_command(struct shtps_nas_spi *ts, int param);
static int shtps_statef_testmode_int(struct shtps_nas_spi *ts, int param);
static int shtps_statef_wait_loader_int(struct shtps_nas_spi *ts, int param);
static int shtps_statef_loader_stopldr(struct shtps_nas_spi *ts, int param);
static int shtps_statef_loader_int(struct shtps_nas_spi *ts, int param);
static int shtps_statef_command_enter(struct shtps_nas_spi *ts, int param);
static int shtps_statef_command_stop(struct shtps_nas_spi *ts, int param);
static int shtps_statef_command_sleep(struct shtps_nas_spi *ts, int param);
static int shtps_statef_command_int(struct shtps_nas_spi *ts, int param);

/* -----------------------------------------------------------------------------------
 */
const static struct shtps_state_func state_poweroff = {
    .enter          = shtps_statef_nop,
    .start          = shtps_statef_powoff_start,
    .stop           = shtps_statef_nop,
    .sleep          = shtps_statef_cmn_sleep,
    .wakeup         = shtps_statef_powoff_wakeup_start,
    .start_ldr      = shtps_statef_powoff_startldr,
    .stop_ldr       = shtps_statef_cmn_stopldr,
    .start_tm       = shtps_statef_powoff_starttm,
    .stop_tm        = shtps_statef_cmn_stoptm,
    .cmd_request    = shtps_statef_powoff_start,
    .interrupt      = shtps_statef_nop,
    .timeout        = shtps_statef_nop,
};

const static struct shtps_state_func state_resetting = {
    .enter          = shtps_statef_resetting_enter,
    .start          = shtps_statef_nop,
    .stop           = shtps_statef_cmn_stop,
    .sleep          = shtps_statef_cmn_sleep_as_stop,
    .wakeup         = shtps_statef_cmn_wakeup,
    .start_ldr      = shtps_statef_cmn_startldr,
    .stop_ldr       = shtps_statef_cmn_stopldr,
    .start_tm       = shtps_statef_cmn_starttm,
    .stop_tm        = shtps_statef_cmn_stoptm,
    .cmd_request    = shtps_statef_nop,
    .interrupt      = shtps_statef_resetting_int,
    .timeout        = shtps_statef_nop,
};

const static struct shtps_state_func state_attach = {
    .enter          = shtps_statef_nop,
    .start          = shtps_statef_nop,
    .stop           = shtps_statef_cmn_stop,
    .sleep          = shtps_statef_cmn_sleep_as_stop,
    .wakeup         = shtps_statef_cmn_wakeup,
    .start_ldr      = shtps_statef_cmn_startldr,
    .stop_ldr       = shtps_statef_cmn_stopldr,
    .start_tm       = shtps_statef_cmn_starttm,
    .stop_tm        = shtps_statef_cmn_stoptm,
    .cmd_request    = shtps_statef_nop,
    .interrupt      = shtps_statef_attach_int,
    .timeout        = shtps_statef_nop,
};

const static struct shtps_state_func state_poweron = {
    .enter          = shtps_statef_nop,
    .start          = shtps_statef_nop,
    .stop           = shtps_statef_cmn_stop,
    .sleep          = shtps_statef_cmn_sleep_as_stop,
    .wakeup         = shtps_statef_cmn_wakeup,
    .start_ldr      = shtps_statef_cmn_startldr,
    .stop_ldr       = shtps_statef_cmn_stopldr,
    .start_tm       = shtps_statef_cmn_starttm,
    .stop_tm        = shtps_statef_cmn_stoptm,
    .cmd_request    = shtps_statef_nop,
    .interrupt      = shtps_statef_powon_int,
    .timeout        = shtps_statef_nop,
};

const static struct shtps_state_func state_sleeping = {
    .enter          = shtps_statef_nop,
    .start          = shtps_statef_nop,
    .stop           = shtps_statef_cmn_stop,
    .sleep          = shtps_statef_cmn_sleep_as_stop,
    .wakeup         = shtps_statef_cmn_wakeup,
    .start_ldr      = shtps_statef_cmn_startldr,
    .stop_ldr       = shtps_statef_cmn_stopldr,
    .start_tm       = shtps_statef_cmn_starttm,
    .stop_tm        = shtps_statef_cmn_stoptm,
    .cmd_request    = shtps_statef_nop,
    .interrupt      = shtps_statef_sleeping_int,
    .timeout        = shtps_statef_nop,
};

const static struct shtps_state_func state_sleep = {
    .enter          = shtps_statef_sleep_enter,
    .start          = shtps_statef_nop,
    .stop           = shtps_statef_cmn_stop,
    .sleep          = shtps_statef_cmn_sleep_as_stop,
    .wakeup         = shtps_statef_sleep_wakeup,
    .start_ldr      = shtps_statef_sleep_startldr,
    .stop_ldr       = shtps_statef_cmn_stopldr,
    .start_tm       = shtps_statef_sleep_starttm,
    .stop_tm        = shtps_statef_cmn_stoptm,
    .cmd_request    = shtps_statef_sleep_command,
    .interrupt      = shtps_statef_sleep_int,
    .timeout        = shtps_statef_nop,
};

const static struct shtps_state_func state_activating = {
    .enter          = shtps_statef_nop,
    .start          = shtps_statef_nop,
    .stop           = shtps_statef_cmn_stop,
    .sleep          = shtps_statef_cmn_sleep_as_stop,
    .wakeup         = shtps_statef_cmn_wakeup,
    .start_ldr      = shtps_statef_cmn_startldr,
    .stop_ldr       = shtps_statef_cmn_stopldr,
    .start_tm       = shtps_statef_cmn_starttm,
    .stop_tm        = shtps_statef_cmn_stoptm,
    .cmd_request    = shtps_statef_nop,
    .interrupt      = shtps_statef_activating_int,
    .timeout        = shtps_statef_nop,
};

const static struct shtps_state_func state_active = {
    .enter          = shtps_statef_active_enter,
    .start          = shtps_statef_nop,
    .stop           = shtps_statef_cmn_stop,
    .sleep          = shtps_statef_active_sleep,
    .wakeup         = shtps_statef_cmn_wakeup,
    .start_ldr      = shtps_statef_active_startldr,
    .stop_ldr       = shtps_statef_cmn_stopldr,
    .start_tm       = shtps_statef_active_starttm,
    .stop_tm        = shtps_statef_cmn_stoptm,
    .cmd_request    = shtps_statef_active_command,
    .interrupt      = shtps_statef_active_int,
    .timeout        = shtps_statef_nop,
};

const static struct shtps_state_func state_wait_testmode = {
    .enter          = shtps_statef_wait_testmode_enter,
    .start          = shtps_statef_nop,
    .stop           = shtps_statef_cmn_stop,
    .sleep          = shtps_statef_cmn_sleep,
    .wakeup         = shtps_statef_cmn_wakeup,
    .start_ldr      = shtps_statef_cmn_error,
    .stop_ldr       = shtps_statef_nop,
    .start_tm       = shtps_statef_nop,
    .stop_tm        = shtps_statef_cmn_stoptm,
    .cmd_request    = shtps_statef_nop,
    .interrupt      = shtps_statef_wait_testmode_int,
    .timeout        = shtps_statef_nop,
};

const static struct shtps_state_func state_testmode = {
    .enter          = shtps_statef_testmode_enter,
    .start          = shtps_statef_nop,
    .stop           = shtps_statef_cmn_stop,
    .sleep          = shtps_statef_cmn_sleep,
    .wakeup         = shtps_statef_cmn_wakeup,
    .start_ldr      = shtps_statef_cmn_error,
    .stop_ldr       = shtps_statef_nop,
    .start_tm       = shtps_statef_nop,
    .stop_tm        = shtps_statef_testmode_stoptm,
    .cmd_request    = shtps_statef_testmode_command,
    .interrupt      = shtps_statef_testmode_int,
    .timeout        = shtps_statef_nop,
};

const static struct shtps_state_func state_wait_loader = {
    .enter          = shtps_statef_nop,
    .start          = shtps_statef_nop,
    .stop           = shtps_statef_cmn_stop,
    .sleep          = shtps_statef_cmn_sleep,
    .wakeup         = shtps_statef_cmn_wakeup,
    .start_ldr      = shtps_statef_nop,
    .stop_ldr       = shtps_statef_loader_stopldr,
    .start_tm       = shtps_statef_cmn_error,
    .stop_tm        = shtps_statef_nop,
    .cmd_request    = shtps_statef_cmn_error,
    .interrupt      = shtps_statef_wait_loader_int,
    .timeout        = shtps_statef_nop,
};

const static struct shtps_state_func state_loader = {
    .enter          = shtps_statef_nop,
    .start          = shtps_statef_nop,
    .stop           = shtps_statef_cmn_stop,
    .sleep          = shtps_statef_cmn_sleep,
    .wakeup         = shtps_statef_cmn_wakeup,
    .start_ldr      = shtps_statef_nop,
    .stop_ldr       = shtps_statef_loader_stopldr,
    .start_tm       = shtps_statef_cmn_error,
    .stop_tm        = shtps_statef_nop,
    .cmd_request    = shtps_statef_cmn_error,
    .interrupt      = shtps_statef_loader_int,
    .timeout        = shtps_statef_nop,
};

const static struct shtps_state_func state_cmd_executing = {
    .enter          = shtps_statef_command_enter,
    .start          = shtps_statef_nop,
    .stop           = shtps_statef_command_stop,
    .sleep          = shtps_statef_command_sleep,
    .wakeup         = shtps_statef_cmn_wakeup,
    .start_ldr      = shtps_statef_cmn_startldr,
    .stop_ldr       = shtps_statef_cmn_stopldr,
    .start_tm       = shtps_statef_cmn_starttm,
    .stop_tm        = shtps_statef_cmn_stoptm,
    .cmd_request    = shtps_statef_nop,
    .interrupt      = shtps_statef_command_int,
    .timeout        = shtps_statef_nop,
};

const static struct shtps_state_func *state_func_tbl[] = {
	&state_poweroff,
	&state_resetting,
	&state_attach,
	&state_poweron,
	&state_sleeping,
	&state_sleep,
	&state_activating,
	&state_active,
	&state_wait_testmode,
	&state_testmode,
	&state_wait_loader,
	&state_loader,
	&state_cmd_executing
};

/* -----------------------------------------------------------------------------------
 */
#define SHTPS_PERFORMANCE_CHECK_POINT_START	(0)
#define SHTPS_PERFORMANCE_CHECK_POINT_CONT	(1)
#define SHTPS_PERFORMANCE_CHECK_POINT_END	(2)

#if defined( SHTPS_DEBUG_PERFORMANCE_CHECK_ENABLE )
	#define SHTPS_PERFORMANCE_CHECK_PIN			(37)
	static int shtps_performance_check_pin_state = 0;
#elif defined( SHTPS_DEBUG_PERFORMANCE_TIME_LOG_ENABLE )
	static int shtps_performace_log_point = 0;
	static struct timeval shtps_performance_log_tv;
#endif /* #if defined( SHTPS_DEBUG_PERFORMANCE_CHECK_ENABLE ) */

static inline void shtps_performance_check_init(void)
{
#if defined( SHTPS_DEBUG_PERFORMANCE_CHECK_ENABLE )
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
#elif defined( SHTPS_DEBUG_PERFORMANCE_TIME_LOG_ENABLE )
#endif
}

static inline void shtps_performance_check_point_set(int state)
{
#if defined( SHTPS_DEBUG_PERFORMANCE_CHECK_ENABLE )
	if(state == SHTPS_PERFORMANCE_CHECK_POINT_START){
		shtps_performance_check_pin_state = 1;
	}else{
		shtps_performance_check_pin_state = 
			(shtps_performance_check_pin_state == 0)? 1 : 0;
	}
	
	gpio_set_value(SHTPS_PERFORMANCE_CHECK_PIN, shtps_performance_check_pin_state);
	
	if(state == SHTPS_PERFORMANCE_CHECK_POINT_END){
		shtps_performance_check_pin_state = 0;
		gpio_set_value(SHTPS_PERFORMANCE_CHECK_PIN, shtps_performance_check_pin_state);
	}
#elif defined( SHTPS_DEBUG_PERFORMANCE_TIME_LOG_ENABLE )
	if(state == SHTPS_PERFORMANCE_CHECK_POINT_START){
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
#endif
}

/* -----------------------------------------------------------------------------------
 */
static int _shtps_reg_write(struct shtps_nas_spi *ts, u8 addr, u8 *buf, u32 size)
{
	int					status;
	struct spi_message	message;
	struct spi_transfer	t;
	u8					txbuf[SHTPS_NASSAU_SPIBLOCK_BUFSIZE + 2];

	memset(&t, 0, sizeof(t));
	spi_message_init(&message);
	spi_message_add_tail(&t, &message);

	txbuf[0] = 0x00;
	txbuf[1] = addr & 0x7F;
	memcpy(&txbuf[2], buf, size);
	
	t.tx_buf		= txbuf;
	t.rx_buf  		= NULL;
	t.len			= size + 2;
	t.bits_per_word	= 8;

	#if defined( SHTPS_LOG_SPI_DEBUG_ENABLE )
	{
		int i;
		for(i = 0;i < size;i++){
			SHTPS_LOG_SPI_DBG_PRINT("spi write reg[0x%02x] <- 0x%02x\n", txbuf[1] + i, buf[i]);
		}
	}
	#endif /* #if defined( SHTPS_LOG_SPI_DEBUG_ENABLE ) */
	
	if((status = spi_sync(ts->spi, &message)) != 0){
		return status;
	}
	return 0;
}

static int shtps_reg_write(struct shtps_nas_spi *ts, u8 addr, u8 *buf, u32 size)
{
	int status;
	int retry;
	int	i;
	u32	s;

	s = size;
	for(i = 0;i < size;i += SHTPS_NASSAU_SPIBLOCK_BUFSIZE){
		retry = 3;
		do{
			status = _shtps_reg_write(ts,
				addr + i,
				buf  + i,
				(s > SHTPS_NASSAU_SPIBLOCK_BUFSIZE)?(SHTPS_NASSAU_SPIBLOCK_BUFSIZE):(s));

			if(status != 0){
				udelay(SHTPS_NASSAU_SPIRETRY_INTERVAL_US);
				SHTPS_LOG_ERR_PRINT("%s spi error status = %d, retry = %d\n",__func__ , status, retry);
			}
		}while(status != 0 && retry-- > 0);
		
		if(status != 0)	return status;
		
		s -= SHTPS_NASSAU_SPIBLOCK_BUFSIZE;
	}
	return 0;
}

static int shtps_reg_write_single(struct shtps_nas_spi *ts, u8 addr, u8 data)
{
	int status;
	int retry = 3;

	do{
		status = _shtps_reg_write(ts, addr, &data, 1);
		if(status != 0){
			udelay(SHTPS_NASSAU_SPIRETRY_INTERVAL_US);
			SHTPS_LOG_ERR_PRINT("%s spi error status = %d\n",__func__ ,  status);
		}
	}while(status != 0 && retry-- > 0);
		
	return status;
}

static int _shtps_reg_read(struct shtps_nas_spi *ts, u8 addr, u8 *buf, u32 size)
{
	int					status;
	struct spi_message	message;
	struct spi_transfer	t;
	u8					txbuf[SHTPS_NASSAU_SPIBLOCK_BUFSIZE + 1];
	u8 					rxbuf[SHTPS_NASSAU_SPIBLOCK_BUFSIZE + 1];

	memset(&t, 0, sizeof(t));
	spi_message_init(&message);
	spi_message_add_tail(&t, &message);

	txbuf[0] = 0x00;
	txbuf[1] = addr;
	
	t.tx_buf		= txbuf;
	t.rx_buf  		= NULL;
	t.len			= 2;
	t.bits_per_word	= 8;

	if((status = spi_sync(ts->spi, &message)) != 0){
		return status;
	}
	
    spi_message_init(&message);
	spi_message_add_tail(&t, &message);

	txbuf[0] = 0x80;
	
	t.tx_buf		= txbuf;
	t.rx_buf  		= rxbuf;
	t.len			= size + 1;
	t.bits_per_word	= 8;

	if((status = spi_sync(ts->spi, &message)) != 0){
		return status;
	}

	#if defined( SHTPS_LOG_SPI_DEBUG_ENABLE )
	{
		int i;
		for(i = 0;i < size;i++){
			SHTPS_LOG_SPI_DBG_PRINT("spi read reg[0x%02x] = 0x%02x\n", addr+i, rxbuf[1+i]);
		}
	}
	#endif /* #if defined( SHTPS_LOG_SPI_DEBUG_ENABLE ) */
	
	memcpy(buf, &rxbuf[1], size);
	return 0;
	
}

static int shtps_reg_read(struct shtps_nas_spi *ts, u16 addr, u8 *buf, u32 size)
{
	int status;
	int retry;
	int	i;
	u32	s;

	s = size;
	for(i = 0;i < size;i += SHTPS_NASSAU_SPIBLOCK_BUFSIZE){
		retry = 3;
		do{
			status = _shtps_reg_read(ts,
				addr + i,
				buf  + i,
				(s > SHTPS_NASSAU_SPIBLOCK_BUFSIZE)?(SHTPS_NASSAU_SPIBLOCK_BUFSIZE):(s));

			if(status != 0){
				udelay(SHTPS_NASSAU_SPIRETRY_INTERVAL_US);
				SHTPS_LOG_ERR_PRINT("%s spi error status = %d\n",__func__, status);
			}
		}while(status != 0 && retry-- > 0);
		
		if(status != 0)	return status;
		
		s -= SHTPS_NASSAU_SPIBLOCK_BUFSIZE;
	}
	return 0;
}

/* -----------------------------------------------------------------------------------
 */
static int shtps_device_access_setup(struct shtps_nas_spi *ts)
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

static int shtps_device_access_teardown(struct shtps_nas_spi *ts)
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
static void shtps_wakeup_diag_touchevent(struct shtps_nas_spi *ts)
{
	SHTPS_LOG_DBG_PRINT("%s()\n", __func__);
	ts->diag.ready = 1;
	wake_up_interruptible(&ts->diag.wait);
}

static void shtps_cancel_diag_touchevent(struct shtps_nas_spi *ts)
{
	SHTPS_LOG_DBG_PRINT("%s()\n", __func__);
	ts->diag.cancel = 1;
	wake_up_interruptible(&ts->diag.wait);
}

static int shtps_wait_diag_touchevent(struct shtps_nas_spi *ts)
{
	int ret = 0;

	SHTPS_LOG_DBG_PRINT("%s()\n", __func__);
	wait_event_interruptible(ts->diag.wait, (ts->diag.ready || ts->diag.cancel));

	SHTPS_LOG_DBG_PRINT("%s() wakeup\n", __func__);
	if(ts->diag.cancel == 1){
		SHTPS_LOG_DBG_PRINT("%s() canceled\n", __func__);
		ts->diag.cancel = 0;
		ret = -1;
	}
	ts->diag.ready = 0;

	return ret;
}

/* -----------------------------------------------------------------------------------
 */
static void shtps_power_on(struct shtps_nas_spi *ts)
{
	SHTPS_LOG_DBG_PRINT("%s()\n", __func__);
    gpio_set_value(ts->rst_pin, 0);
	gpio_direction_output(ts->pow_pin, 1);
	if(ts->v17_power){
		ts->v17_power(1);
	}
	msleep(SHTPS_NASSAU_RESET_AFTER_WAIT_MS);
	shtps_device_access_setup(ts);
    gpio_set_value(ts->rst_pin, 1);
}

static void shtps_power_off(struct shtps_nas_spi *ts)
{
	SHTPS_LOG_DBG_PRINT("%s()\n", __func__);
    gpio_set_value(ts->rst_pin, 0);
	msleep(SHTPS_NASSAU_RESET_AFTER_WAIT_MS);
	shtps_device_access_teardown(ts);
	if(ts->v17_power){
		ts->v17_power(0);
	}
	gpio_direction_output(ts->pow_pin, 0);
}

static void shtps_reset(struct shtps_nas_spi *ts)
{
	SHTPS_LOG_DBG_PRINT("%s()\n", __func__);
    gpio_set_value(ts->rst_pin, 0);
	msleep(SHTPS_NASSAU_RESET_AFTER_WAIT_MS);
    gpio_set_value(ts->rst_pin, 1);
}

static u8 shtps_check_intsrc(struct shtps_nas_spi *ts, u8 source)
{
	u8 buf[1];
	
	if(shtps_reg_read(ts, SHTPS_NAS_REG_HOSTINT, buf, 1) != 0){
		SHTPS_LOG_ERR_PRINT("%s() spi read error\n", __func__);
		return 0;
	}
	
	SHTPS_LOG_DBG_PRINT("%s() INT(0x%02x) & 0x%02x = 0x%02x\n", __func__, buf[0], source, (buf[0] & source));
	return (buf[0] & source);
}

static void shtps_clr_int(struct shtps_nas_spi *ts, u8 source)
{
	SHTPS_LOG_DBG_PRINT("%s(0x%02x)\n", __func__, source);
	shtps_reg_write_single(ts, SHTPS_NAS_REG_HOSTINT, source);
}

static u8 shtps_get_intsrc(struct shtps_nas_spi *ts, u8 source)
{
	int i;
	u8 intsrc = SHTPS_NAS_INT_NONE;
	u8 val;

	for(i = 0; i < SHTPS_INT_CHECK_MAX; i++){
		val = SHTPS_NAS_INT_NONE;
		val = shtps_check_intsrc(ts, 0xFF);
		intsrc |= val;

		if(val == SHTPS_NAS_INT_NONE){
			break;
		}
		shtps_clr_int(ts, val);
	}

	if(i >= SHTPS_INT_CHECK_MAX){
		intsrc = SHTPS_NAS_INT_RESET;
	}

	return intsrc;
}

static void shtps_check_fw_startup(struct shtps_nas_spi *ts)
{
	u8 buf = 0x00;

	shtps_reg_write_single(ts, SHTPS_NAS_REG_BANK, SHTPS_NAS_BANK_CMDRESULT);
	shtps_reg_read(ts, SHTPS_NAS_REG_CMD, &buf, 1);

	SHTPS_LOG_DBG_PRINT("%s() fw startup val = 0x%02X\n", __func__, buf);

	if(ts->fw_update_continue == 0x00){
		if(buf == 0x00){
			ts->fw_startup_state = SHTPS_FW_STARTUP_CRC_ERROR;
		}else{
			if(ts->is_lcd_on != 0x00){
				ts->fw_startup_state = SHTPS_FW_STARTUP_PROGRAM;
			}else{
				ts->fw_startup_state = SHTPS_FW_STARTUP_SLEEP;
			}
		}
	}else{
		ts->fw_startup_state = SHTPS_FW_STARTUP_LOADER;
	}
}

static int shtps_next_touch_report_req(struct shtps_nas_spi *ts)
{
	int ret = 0;

	ret = shtps_reg_write_single(ts, SHTPS_NAS_REG_MICOM, 0x10);

	return ret;
}

/* -----------------------------------------------------------------------------------
 */
static int shtps_write_cmd(struct shtps_nas_spi *ts, u8 *cmd, int size)
{
	if(shtps_reg_write_single(ts, SHTPS_NAS_REG_BANK, SHTPS_NAS_BANK_CMD) != 0){
		SHTPS_LOG_ERR_PRINT("%s() bank change error\n", __func__);
		return -1;
	}
	if(shtps_reg_write(ts, SHTPS_NAS_REG_CMD, cmd, size) != 0){
		SHTPS_LOG_ERR_PRINT("%s() command write error\n", __func__);
		return -1;
	}
	if(shtps_reg_write_single(ts, SHTPS_NAS_REG_MICOM, 0x01) != 0){
		SHTPS_LOG_ERR_PRINT("%s() micom ind write error\n", __func__);
		return -1;
	}
	return 0;
}

static int shtps_write_init_cmds(struct shtps_nas_spi *ts)
{
	const u8 cmd[] = { 0x01, 0x00, 0x00 };
	SHTPS_LOG_DBG_PRINT("%s()\n", __func__);
	return shtps_write_cmd(ts, (u8*)cmd, sizeof(cmd));
}

static int shtps_write_attach_cmds(struct shtps_nas_spi *ts)
{
	const u8 cmd[] = { 0xD7, 0x00, 0x01, 0x00, 0x01 };
	SHTPS_LOG_DBG_PRINT("%s()\n", __func__);
	return shtps_write_cmd(ts, (u8*)cmd, sizeof(cmd));
}

static int shtps_write_active_cmds(struct shtps_nas_spi *ts)
{
	const u8 cmd[] = { 0x02, 0x00, 0x02, 0x00, 0x03, 0x00 };
	SHTPS_LOG_DBG_PRINT("%s()\n", __func__);
	return shtps_write_cmd(ts, (u8*)cmd, sizeof(cmd));
}

#if defined( SHTPS_POWER_MODE_CONTROL_ENABLE )
static int shtps_write_active_low_power_cmds(struct shtps_nas_spi *ts)
{
	const u8 cmd[] = { 0x02, 0x00, 0x02, 0x00, 0x03, 0x01 };
	SHTPS_LOG_DBG_PRINT("%s()\n", __func__);
	return shtps_write_cmd(ts, (u8*)cmd, sizeof(cmd));
}

static int shtps_write_active_high_power_cmds(struct shtps_nas_spi *ts)
{
	const u8 cmd[] = { 0x02, 0x00, 0x02, 0x00, 0x03, 0x02 };
	SHTPS_LOG_DBG_PRINT("%s()\n", __func__);
	return shtps_write_cmd(ts, (u8*)cmd, sizeof(cmd));
}

static int shtps_write_normal_power_cmds(struct shtps_nas_spi *ts)
{
	const u8 cmd[] = { 0x02, 0x00, 0x02, 0x00, 0xFF, 0x00 };
	SHTPS_LOG_DBG_PRINT("%s()\n", __func__);
	return shtps_write_cmd(ts, (u8*)cmd, sizeof(cmd));
}

static int shtps_write_low_power_cmds(struct shtps_nas_spi *ts)
{
	const u8 cmd[] = { 0x02, 0x00, 0x02, 0x00, 0xFF, 0x01 };
	SHTPS_LOG_DBG_PRINT("%s()\n", __func__);
	return shtps_write_cmd(ts, (u8*)cmd, sizeof(cmd));
}

static int shtps_write_high_power_cmds(struct shtps_nas_spi *ts)
{
	const u8 cmd[] = { 0x02, 0x00, 0x02, 0x00, 0xFF, 0x02 };
	SHTPS_LOG_DBG_PRINT("%s()\n", __func__);
	return shtps_write_cmd(ts, (u8*)cmd, sizeof(cmd));
}

#if 0
static int shtps_get_system_state_cmds(struct shtps_nas_spi *ts)
{
	const u8 cmd[] = { 0x03, 0x00, 0x03 };
	SHTPS_LOG_DBG_PRINT("%s()\n", __func__);
	return shtps_write_cmd(ts, (u8*)cmd, sizeof(cmd));
}
#endif
#endif /* #if defined( SHTPS_POWER_MODE_CONTROL_ENABLE ) */

static int shtps_write_sleep_cmds(struct shtps_nas_spi *ts)
{
	const u8 cmd[] = { 0x02, 0x00, 0x02, 0x00, 0x00, 0x00 };
	SHTPS_LOG_DBG_PRINT("%s()\n", __func__);
	return shtps_write_cmd(ts, (u8*)cmd, sizeof(cmd));
}

static int shtps_write_bootloader_cmds(struct shtps_nas_spi *ts)
{
	const u8 cmd[] = { 0xE1, 0x00, 0x04, 0x00, 0xFF, 0x00, 0xAA, 0x55 };
	SHTPS_LOG_DBG_PRINT("%s()\n", __func__);

	ts->loader.ready  = 0;
	ts->loader.cancel = 0;
	
	return shtps_write_cmd(ts, (u8*)cmd, sizeof(cmd));
}

static int shtps_write_testmode_cmds(struct shtps_nas_spi *ts)
{
	SHTPS_LOG_DBG_PRINT("%s()\n", __func__);

#if defined( SHTPS_TESTMODE_ENTER_WAIT_ENABLE )
#if defined( SHTPS_TESTMODE_ENTER_WAIT_VARIABLE )
	if (shtps_testmode_enter_wait_ms > 0)
		mdelay(shtps_testmode_enter_wait_ms);
#else
	mdelay(SHTPS_TESTMODE_ENTER_WAIT_MS);
#endif /* #if defined( SHTPS_TESTMODE_ENTER_WAIT_VARIABLE ) */
#endif 	/* #if defined( SHTPS_TESTMODE_ENTER_WAIT_ENABLE ) */

	ts->testmode.ready  = 0;
	ts->testmode.cancel = 0;

	shtps_reg_write_single(ts, SHTPS_NAS_REG_BANK, 0x10);
	shtps_reg_write_single(ts, 0x20, 0x01);
		
	shtps_reg_write_single(ts, SHTPS_NAS_REG_BANK, 0x0C);
	shtps_reg_write_single(ts, 0x0C, 0x02);	
	
	if (ts->testmode.opt & SHTPS_TESTMODE_OPT_FIXED_MODE_HEAD) {
		shtps_reg_write_single(ts, SHTPS_NAS_REG_BANK, 0x31);
		shtps_reg_write_single(ts, 0x0A, 0x00);
	}	
	if (ts->testmode.opt & SHTPS_TESTMODE_OPT_FIXED_MODE_BACK) {
		shtps_reg_write_single(ts, SHTPS_NAS_REG_BANK, 0x31);
		shtps_reg_write_single(ts, 0x0A, 0x01);
	}	
	
	shtps_reg_write_single(ts, SHTPS_NAS_REG_BANK, 0x50);
	shtps_reg_write_single(ts, 0x20, 0x01);
	
	shtps_reg_write_single(ts, SHTPS_NAS_REG_BANK, 0x30);
	shtps_reg_write_single(ts, 0x47, 0x00);
	shtps_reg_write_single(ts, 0x48, 0x00);	
	shtps_reg_write_single(ts, 0x14, 0x13);
	shtps_reg_write_single(ts, 0x16, 0x14);
	shtps_reg_write_single(ts, 0x32, 0x00);
	shtps_reg_write_single(ts, 0x50, 0x00);
	shtps_reg_write_single(ts, 0x0C, 0x03);
	shtps_reg_write_single(ts, 0x4F, 0x01);
	if (ts->testmode.opt & SHTPS_TESTMODE_OPT_CALIBRATION_BLOCK_BYPASS) {
		shtps_reg_write_single(ts, 0x35, 0x01);
	}	
	if ((ts->testmode.opt & SHTPS_TESTMODE_OPT_FIXED_MODE_HEAD) ||
		(ts->testmode.opt & SHTPS_TESTMODE_OPT_FIXED_MODE_BACK)) {
		shtps_reg_write_single(ts, 0x30, 0x00);
		shtps_reg_write_single(ts, 0x31, 0x01);
		mdelay(SHTPS_DCMAP_FIXED_MODE_WAIT_MS);
		shtps_reg_write_single(ts, 0x31, 0x00);
		shtps_reg_write_single(ts, 0x30, 0x0B);
	}
	else{
		#if defined(SHTPS_TESTMODE_NEW_METHOD_ENABLE)
			shtps_reg_write_single(ts, SHTPS_NAS_REG_BANK, 0x31);
			shtps_reg_write_single(ts, 0x0A, 0x02);
		#endif /* SHTPS_TESTMODE_NEW_METHOD_ENABLE */
	}

#if defined(SHTPS_TESTMODE_NEW_METHOD_ENABLE)
#else
	shtps_reg_write_single(ts, SHTPS_NAS_REG_BANK, 0x0C);
	shtps_reg_write_single(ts, 0x0D, 0x01);
#endif /* SHTPS_TESTMODE_NEW_METHOD_ENABLE */

	return 0;
}

static int shtps_write_exit_testmode_cmds(struct shtps_nas_spi *ts)
{
	SHTPS_LOG_DBG_PRINT("%s()\n", __func__);

	if (ts->testmode.opt & SHTPS_TESTMODE_OPT_CALIBRATION_BLOCK_BYPASS) {
		shtps_reg_write_single(ts, SHTPS_NAS_REG_BANK, 0x30);
		shtps_reg_write_single(ts, 0x35, 0x00);
	}
	
	shtps_reg_write_single(ts, SHTPS_NAS_REG_BANK, 0x0C);
	shtps_reg_write_single(ts, 0x0C, 0x00);
	
	shtps_reg_write_single(ts, SHTPS_NAS_REG_BANK, 0x10);
	shtps_reg_write_single(ts, 0x20, 0x00);

	ts->testmode.ready  = 0;
	ts->testmode.cancel = 0;
	
	return 0;
}

static int shtps_write_getfwver_cmds(struct shtps_nas_spi *ts)
{
	const u8 cmd[] = { 0xE0, 0x00, 0x0C, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00 };
	SHTPS_LOG_DBG_PRINT("%s()\n", __func__);
	return shtps_write_cmd(ts, (u8*)cmd, sizeof(cmd));
}

static int shtps_write_activate_cmds(struct shtps_nas_spi *ts)
{
	int ret = 0;

	#if defined( SHTPS_POWER_MODE_CONTROL_ENABLE )
		if( (ts->power_mode_state & SHTPS_LPMODE_ON) != 0 ){
			ret = shtps_write_active_low_power_cmds(ts);
		}else if( (ts->power_mode_state & SHTPS_HPMODE_ON) != 0 ){
			ret = shtps_write_active_high_power_cmds(ts);
		}else{
			ret = shtps_write_active_cmds(ts);
		}
	#else
		ret = shtps_write_active_cmds(ts);
	#endif

	return ret;
}

/* -----------------------------------------------------------------------------------
 */
#if defined( SHTPS_DEBUG_PERFORMANCE_CHECK_ENABLE )
	static irqreturn_t shtps_irq_handler(int irq, void *dev_id)
	{
		shtps_performance_check_point_set(SHTPS_PERFORMANCE_CHECK_POINT_START);
		return IRQ_WAKE_THREAD;
	}
#endif /* #if defined( SHTPS_DEBUG_PERFORMANCE_CHECK_ENABLE ) */

static irqreturn_t shtps_irq(int irq, void *dev_id)
{
	struct shtps_nas_spi	*ts = dev_id;

	SHTPS_LOG_DBG_PRINT("%s()\n", __func__);

	shtps_performance_check_point_set(SHTPS_PERFORMANCE_CHECK_POINT_CONT);
	request_event(ts, SHTPS_EVENT_INTERRUPT, 1);
	shtps_performance_check_point_set(SHTPS_PERFORMANCE_CHECK_POINT_END);

	return IRQ_HANDLED;
}

#if 0
static void shtps_irq_wake_disable(struct shtps_nas_spi *ts)
{
	SHTPS_LOG_DBG_PRINT("%s()\n", __func__);
	if(ts->irq_mgr.wake != SHTPS_IRQ_WAKE_DISABLE){
		disable_irq_wake(ts->irq_mgr.irq);
		ts->irq_mgr.wake = SHTPS_IRQ_WAKE_DISABLE;

		#if defined( SHTPS_MODULE_PARAM_ENABLE )
			shtps_irq_wake_state = SHTPS_IRQ_WAKE_DISABLE;
		#endif /* SHTPS_MODULE_PARAM_ENABLE */
	}
}

static void shtps_irq_wake_enable(struct shtps_nas_spi *ts)
{
	SHTPS_LOG_DBG_PRINT("%s()\n", __func__);
	if(ts->irq_mgr.wake != SHTPS_IRQ_WAKE_ENABLE){
		enable_irq_wake(ts->irq_mgr.irq);
		ts->irq_mgr.wake = SHTPS_IRQ_WAKE_ENABLE;

		#if defined( SHTPS_MODULE_PARAM_ENABLE )
			shtps_irq_wake_state = SHTPS_IRQ_WAKE_ENABLE;
		#endif /* SHTPS_MODULE_PARAM_ENABLE */
	}
}
#endif

static void shtps_irq_disable(struct shtps_nas_spi *ts)
{
	SHTPS_LOG_DBG_PRINT("%s()\n", __func__);
	if(ts->irq_mgr.state != SHTPS_IRQ_STATE_DISABLE){
		disable_irq_nosync(ts->irq_mgr.irq);
		ts->irq_mgr.state = SHTPS_IRQ_STATE_DISABLE;
	}
}

static void shtps_irq_enable(struct shtps_nas_spi *ts)
{
	SHTPS_LOG_DBG_PRINT("%s()\n", __func__);
	if(ts->irq_mgr.state != SHTPS_IRQ_STATE_ENABLE){
		enable_irq(ts->irq_mgr.irq);
		ts->irq_mgr.state = SHTPS_IRQ_STATE_ENABLE;
	}
}

static int shtps_irq_resuest(struct shtps_nas_spi *ts)
{
	int rc;

	SHTPS_LOG_DBG_PRINT("%s()\n", __func__);

#if defined( SHTPS_DEBUG_PERFORMANCE_CHECK_ENABLE )
	rc = request_threaded_irq(ts->irq_mgr.irq,
							  shtps_irq_handler,
							  shtps_irq,
							  IRQF_TRIGGER_RISING | IRQF_ONESHOT,
							  SH_TOUCH_DEVNAME,
							  ts);
#else
	rc = request_threaded_irq(ts->irq_mgr.irq,
							  NULL,
							  shtps_irq,
							  IRQF_TRIGGER_RISING | IRQF_ONESHOT,
							  SH_TOUCH_DEVNAME,
							  ts);
#endif /* #if defined( SHTPS_DEBUG_PERFORMANCE_CHECK_ENABLE ) */

	if(rc){
		SHTPS_LOG_ERR_PRINT("request_threaded_irq error:%d\n",rc);
		return -1;
	}

	ts->irq_mgr.state = SHTPS_IRQ_STATE_ENABLE;
	ts->irq_mgr.wake  = SHTPS_IRQ_WAKE_DISABLE;
	shtps_irq_disable(ts);
	return 0;
}

/* -----------------------------------------------------------------------------------
 */
#if defined(SHTPS_CPU_CLOCK_CONTROL_ENABLE)
static void shtps_perf_lock_enable(struct shtps_nas_spi *ts)
{
	mutex_lock(&shtps_cpu_clock_ctrl_lock);

	switch (ts->perf_lock_level) {
		case SHTPS_PERF_LOCK_LEVEL_HIGHEST:
			if ( is_perf_lock_active(&ts->perf_lock_level_highest) == 0 ){
				perf_lock(&ts->perf_lock_level_highest);
				SHTPS_LOG_DBG_PRINT("perf_lock_level_highest start (%d ms)\n", ts->perf_lock_enable_time_ms);
			}
			break;

		case SHTPS_PERF_LOCK_LEVEL_HIGH:
			if ( is_perf_lock_active(&ts->perf_lock_level_high) == 0 ){
				perf_lock(&ts->perf_lock_level_high);
				SHTPS_LOG_DBG_PRINT("perf_lock_level_high start (%d ms)\n", ts->perf_lock_enable_time_ms);
			}
			break;

		case SHTPS_PERF_LOCK_LEVEL_LOW:
			if ( is_perf_lock_active(&ts->perf_lock_level_low) == 0 ){
				perf_lock(&ts->perf_lock_level_low);
				SHTPS_LOG_DBG_PRINT("perf_lock_level_low start (%d ms)\n", ts->perf_lock_enable_time_ms);
			}
			break;

		default:
			SHTPS_LOG_ERR_PRINT("perf_lock select level error [%d]\n", ts->perf_lock_level);
			break;
	}

	mutex_unlock(&shtps_cpu_clock_ctrl_lock);
}

static void shtps_perf_lock_disable(struct shtps_nas_spi *ts)
{
	mutex_lock(&shtps_cpu_clock_ctrl_lock);

	if (is_perf_lock_active(&ts->perf_lock_level_highest)){
		perf_unlock(&ts->perf_lock_level_highest);
		SHTPS_LOG_DBG_PRINT("perf_lock_level_highest end\n");
	}
	if (is_perf_lock_active(&ts->perf_lock_level_high)){
		perf_unlock(&ts->perf_lock_level_high);
		SHTPS_LOG_DBG_PRINT("perf_lock_level_high end\n");
	}
	if (is_perf_lock_active(&ts->perf_lock_level_low)){
		perf_unlock(&ts->perf_lock_level_low);
		SHTPS_LOG_DBG_PRINT("perf_lock_level_low end\n");
	}

	mutex_unlock(&shtps_cpu_clock_ctrl_lock);
}

static int shtps_perf_lock_disable_timer_start(struct shtps_nas_spi *ts, unsigned long delay_ms)
{
	cancel_delayed_work(&ts->perf_lock_disable_delayed_work);
	schedule_delayed_work(&ts->perf_lock_disable_delayed_work, msecs_to_jiffies(delay_ms));

	return 0;
}

static void shtps_perf_lock_disable_delayed_work_function(struct work_struct *work)
{
	struct delayed_work *dw = container_of(work, struct delayed_work, work);
	struct shtps_nas_spi *ts = container_of(dw, struct shtps_nas_spi, perf_lock_disable_delayed_work);

	SHTPS_LOG_DBG_PRINT("%s()\n", __func__);

	shtps_perf_lock_disable(ts);
	SHTPS_LOG_DBG_PRINT("perf_lock end by Timer\n");

	return;
}
#endif /* SHTPS_CPU_CLOCK_CONTROL_ENABLE */

/* -----------------------------------------------------------------------------------
 */
inline static void shtps_clear_fingerinfo(struct shtps_touch_info *info)
{
	u8 i;

	SHTPS_LOG_DBG_PRINT("%s()\n", __func__);
	for(i = 0;i < SHTPS_FINGER_MAX;i++){
		info->fingers[i].id = SHTPS_FINGER_ID_INVALID;
	}
}

static void shtps_set_fingerinfo(struct shtps_nas_spi *ts, struct shtps_touch_info *info, u8 *buf)
{
	u8 i;
	u8 id = buf[0] & 0x0F;
	u8 index = SHTPS_FINGER_ID_INVALID;
	
	for(i = 0;i < SHTPS_FINGER_MAX;i++){
		if(ts->report_info.fingers[i].id == id){
			index = i;
			break;
		}else if(ts->report_info.fingers[i].id == SHTPS_FINGER_ID_INVALID){
			if((index == SHTPS_FINGER_ID_INVALID) && (info->fingers[i].id == SHTPS_FINGER_ID_INVALID)){
				index = i;
			}
		}
	}
	if(index == SHTPS_FINGER_ID_INVALID){
		SHTPS_LOG_DBG_PRINT("%s() no empty info. regist skipped. state = 0x%02x, id = %u\n",
		 	__func__, ((buf[0] >> 0x04) & 0x0F), (buf[0] & 0x0F));	
		 return;
	}

	
	info->fingers[index].state	= (buf[0] >> 0x04) & 0x0F;
	info->fingers[index].id 	= (buf[0] & 0x0F);
#if defined( SHTPS_POSITION_DIRECT_REPORT_ENABLE )
	info->fingers[index].x 		= (buf[2] << 0x08) | buf[1];
	info->fingers[index].y 		= (buf[4] << 0x08) | buf[3];
	info->fingers[index].w 		= buf[5];
	info->fingers[index].z 		= (buf[7] << 0x08) | buf[6];
#else
	info->fingers[index].x 		= (buf[2] * 1000 + (unsigned long)((buf[1] * 1000) / 256));
	info->fingers[index].y 		= (buf[4] * 1000 + (unsigned long)((buf[3] * 1000) / 256));
	info->fingers[index].w 		= buf[5];
	info->fingers[index].z 		= (buf[7] * 1000 + (unsigned long)((buf[6] * 1000) / 256));
#endif /* #if defined( SHTPS_POSITION_DIRECT_REPORT_ENABLE ) */

#if defined( SHTPS_PEN_DETECT_ENABLE )
	if (ts->report_info.fingers[index].id != SHTPS_FINGER_ID_INVALID) {		
		if (ts->report_info.fingers[index].tool != SHTPS_INPUT_TYPE_NONE)
			info->fingers[index].tool 	= ts->report_info.fingers[index].tool;
		else
			info->fingers[index].tool 	= SHTPS_INPUT_TYPE_FINGER;
	}
	else {
#if defined( SHTPS_PEN_DETECT_THRESHOLD_VARIABLE )
		if (info->fingers[index].z < (unsigned short)shtps_pen_detect_pressure_threshold)
#else
		if (info->fingers[index].z < SHTPS_PEN_DETECT_THRESHOLD)
#endif	/* #if defined( SHTPS_PEN_DETECT_THRESHOLD_VARIABLE ) */
			info->fingers[index].tool 	= SHTPS_INPUT_TYPE_PEN;
		else
			info->fingers[index].tool 	= SHTPS_INPUT_TYPE_FINGER;
	}
#else
	info->fingers[index].tool 	= SHTPS_INPUT_TYPE_FINGER;
#endif	/* #if defined( SHTPS_PEN_DETECT_ENABLE ) */


	SHTPS_LOG_DBG_PRINT("%s() index = %u, state = 0x%02x, tool = %u, id = %u, x = %u, y = %u, w = %u, z = %u\n",
	 	__func__, index, info->fingers[index].state, info->fingers[index].tool, info->fingers[index].id,
	 	info->fingers[index].x, info->fingers[index].y, info->fingers[index].w, info->fingers[index].z);	
}

#if defined ( CONFIG_SHTPS_NASSAU_POSITION_OFFSET )
static int shtps_offset_area(struct shtps_nas_spi *ts, int x, int y)
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

#if defined(SHTPS_PEN_DETECT_ENABLE)
static int shtps_offset_pen_area(struct shtps_nas_spi *ts, int x, int y)
{
	if(y < ts->offset_pen.base[2]){
		if(x < ts->offset_pen.base[0]){
			return 0x00;
		}else if(x < ts->offset_pen.base[1]){
			return 0x01;
		}else{
			return 0x02;
		}
	}else if(y < ts->offset_pen.base[3]){
		if(x < ts->offset_pen.base[0]){
			return 0x03;
		}else if(x < ts->offset_pen.base[1]){
			return 0x04;
		}else{
			return 0x05;
		}
	}else if(y < ts->offset_pen.base[4]){
		if(x < ts->offset_pen.base[0]){
			return 0x06;
		}else if(x < ts->offset_pen.base[1]){
			return 0x07;
		}else{
			return 0x08;
		}
	}else{
		if(x < ts->offset_pen.base[0]){
			return 0x09;
		}else if(x < ts->offset_pen.base[1]){
			return 0x0A;
		}else{
			return 0x0B;
		}
	}
	return 0x00;
}
#endif /* #if defined(SHTPS_PEN_DETECT_ENABLE) */
#endif /* #if deifned( CONFIG_SHTPS_NASSAU_POSITION_OFFSET ) */

static int shtps_offset_pos(struct shtps_nas_spi *ts, int *x, int *y)
{
#if defined ( CONFIG_SHTPS_NASSAU_POSITION_OFFSET )
	int area;
	int pq, rs;
	int xp, xq, xr, xs;
	int yp, yq, yr, ys;
	int base_xp, base_xq;
	int base_yp, base_yq;
	int x_org, y_org;

	if(!ts->offset.enabled){
		return 0;
	}

	#if defined(SHTPS_PEN_HAND_DETECT_ENABLE)
		if((*x == SHTPS_PEN_HAND_COORDINATES_X) && (*y == SHTPS_PEN_HAND_COORDINATES_Y) ){
			return 0;
		}
	#endif /* SHTPS_PEN_HAND_DETECT_ENABLE */

	x_org = *x;
	y_org = *y;

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
		base_xq = CONFIG_SHTPS_NASSAU_PANEL_SIZE_X;
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
		base_xq = CONFIG_SHTPS_NASSAU_PANEL_SIZE_X;
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
		base_xq = CONFIG_SHTPS_NASSAU_PANEL_SIZE_X;
		base_yp = ts->offset.base[3];
		base_yq = ts->offset.base[4];
	}else if(area == 0x09){
		xq = xs = ts->offset.diff[8];
		yp = yq = ts->offset.diff[9];
		base_xp = 0;
		base_xq = ts->offset.base[0];
		base_yp = ts->offset.base[4];
		base_yq = CONFIG_SHTPS_NASSAU_PANEL_SIZE_Y;
	}else if(area == 0x0A){
		xp = xr = ts->offset.diff[8];
		xq = xs = ts->offset.diff[10];
		yp = ts->offset.diff[9];
		yq = ts->offset.diff[11];
		base_xp = ts->offset.base[0];
		base_xq = ts->offset.base[1];
		base_yp = ts->offset.base[4];
		base_yq = CONFIG_SHTPS_NASSAU_PANEL_SIZE_Y;
	}else{
		xq = xr = ts->offset.diff[10];
		yp = yq = ts->offset.diff[11];
		base_xp = ts->offset.base[1];
		base_xq = CONFIG_SHTPS_NASSAU_PANEL_SIZE_X;
		base_yp = ts->offset.base[4];
		base_yq = CONFIG_SHTPS_NASSAU_PANEL_SIZE_Y;
	}

	pq = (xq - xp) * (*x - base_xp) / (base_xq - base_xp) + xp;
	rs = (xs - xr) * (*x - base_xp) / (base_xq - base_xp) + xr;
	*x -= ((rs - pq) * (*y - base_yp) / (base_yq - base_yp) + pq);

	pq = (yq - yp) * (*x - base_xp) / (base_xq - base_xp) + yp;
	rs = (ys - yr) * (*x - base_xp) / (base_xq - base_xp) + yr;
	*y -= ((rs - pq) * (*y - base_yp) / (base_yq - base_yp) + pq);

	SHTPS_LOG_DBG_PRINT("Offset pos : (%4d, %4d) -> (%4d, %4d)\n",
							x_org, y_org, *x, *y);
#endif /* #if deifned( CONFIG_SHTPS_NASSAU_POSITION_OFFSET ) */
	return 0;
}

#if defined(SHTPS_PEN_DETECT_ENABLE)
static int shtps_offset_pos_pen(struct shtps_nas_spi *ts, int *x, int *y)
{
#if defined ( CONFIG_SHTPS_NASSAU_POSITION_OFFSET )
	int area;
	int pq, rs;
	int xp, xq, xr, xs;
	int yp, yq, yr, ys;
	int base_xp, base_xq;
	int base_yp, base_yq;
	int x_org, y_org;

	if(!ts->offset_pen.enabled){
		return 0;
	}

	#if defined(SHTPS_PEN_HAND_DETECT_ENABLE)
		if((*x == SHTPS_PEN_HAND_COORDINATES_X) && (*y == SHTPS_PEN_HAND_COORDINATES_Y) ){
			return 0;
		}
	#endif /* SHTPS_PEN_HAND_DETECT_ENABLE */

	x_org = *x;
	y_org = *y;

	area = shtps_offset_pen_area(ts, *x, *y);

	xp = xq = xr = xs = yp = yq = yr = ys = 0;
	if(area == 0x00){
		xq = xs = ts->offset_pen.diff[0];
		yr = ys = ts->offset_pen.diff[1];
		base_xp = 0;
		base_xq = ts->offset_pen.base[0];
		base_yp = 0;
		base_yq = ts->offset_pen.base[2];
	}else if(area == 0x01){
		xp = xr = ts->offset_pen.diff[0];
		xq = xs = ts->offset_pen.diff[2];
		yr = ts->offset_pen.diff[1];
		ys = ts->offset_pen.diff[3];
		base_xp = ts->offset_pen.base[0];
		base_xq = ts->offset_pen.base[1];
		base_yp = 0;
		base_yq = ts->offset_pen.base[2];
	}else if(area == 0x02){
		xq = xr = ts->offset_pen.diff[2];
		yr = ys = ts->offset_pen.diff[3];
		base_xp = ts->offset_pen.base[1];
		base_xq = CONFIG_SHTPS_NASSAU_PANEL_SIZE_X;
		base_yp = 0;
		base_yq = ts->offset_pen.base[2];
	}else if(area == 0x03){
		xq = ts->offset_pen.diff[0];
		xs = ts->offset_pen.diff[4];
		yp = yq = ts->offset_pen.diff[1];
		yr = ys = ts->offset_pen.diff[5];
		base_xp = 0;
		base_xq = ts->offset_pen.base[0];
		base_yp = ts->offset_pen.base[2];
		base_yq = ts->offset_pen.base[3];
	}else if(area == 0x04){
		xp = ts->offset_pen.diff[0];
		xq = ts->offset_pen.diff[2];
		xr = ts->offset_pen.diff[4];
		xs = ts->offset_pen.diff[6];
		yp = ts->offset_pen.diff[1];
		yq = ts->offset_pen.diff[3];
		yr = ts->offset_pen.diff[5];
		ys = ts->offset_pen.diff[7];
		base_xp = ts->offset_pen.base[0];
		base_xq = ts->offset_pen.base[1];
		base_yp = ts->offset_pen.base[2];
		base_yq = ts->offset_pen.base[3];
	}else if(area == 0x05){
		xp = ts->offset_pen.diff[2];
		xr = ts->offset_pen.diff[6];
		yp = yq = ts->offset_pen.diff[3];
		yr = ys = ts->offset_pen.diff[7];
		base_xp = ts->offset_pen.base[1];
		base_xq = CONFIG_SHTPS_NASSAU_PANEL_SIZE_X;
		base_yp = ts->offset_pen.base[2];
		base_yq = ts->offset_pen.base[3];
	}else if(area == 0x06){
		xq = ts->offset_pen.diff[4];
		xs = ts->offset_pen.diff[8];
		yp = yq = ts->offset_pen.diff[5];
		yr = ys = ts->offset_pen.diff[9];
		base_xp = 0;
		base_xq = ts->offset_pen.base[0];
		base_yp = ts->offset_pen.base[3];
		base_yq = ts->offset_pen.base[4];
	}else if(area == 0x07){
		xp = ts->offset_pen.diff[4];
		xq = ts->offset_pen.diff[6];
		xr = ts->offset_pen.diff[8];
		xs = ts->offset_pen.diff[10];
		yp = ts->offset_pen.diff[5];
		yq = ts->offset_pen.diff[7];
		yr = ts->offset_pen.diff[9];
		ys = ts->offset_pen.diff[11];
		base_xp = ts->offset_pen.base[0];
		base_xq = ts->offset_pen.base[1];
		base_yp = ts->offset_pen.base[3];
		base_yq = ts->offset_pen.base[4];
	}else if(area == 0x08){
		xp = ts->offset_pen.diff[6];
		xr = ts->offset_pen.diff[10];
		yp = yq = ts->offset_pen.diff[7];
		yr = ys = ts->offset_pen.diff[11];
		base_xp = ts->offset_pen.base[1];
		base_xq = CONFIG_SHTPS_NASSAU_PANEL_SIZE_X;
		base_yp = ts->offset_pen.base[3];
		base_yq = ts->offset_pen.base[4];
	}else if(area == 0x09){
		xq = xs = ts->offset_pen.diff[8];
		yp = yq = ts->offset_pen.diff[9];
		base_xp = 0;
		base_xq = ts->offset_pen.base[0];
		base_yp = ts->offset_pen.base[4];
		base_yq = CONFIG_SHTPS_NASSAU_PANEL_SIZE_Y;
	}else if(area == 0x0A){
		xp = xr = ts->offset_pen.diff[8];
		xq = xs = ts->offset_pen.diff[10];
		yp = ts->offset_pen.diff[9];
		yq = ts->offset_pen.diff[11];
		base_xp = ts->offset_pen.base[0];
		base_xq = ts->offset_pen.base[1];
		base_yp = ts->offset_pen.base[4];
		base_yq = CONFIG_SHTPS_NASSAU_PANEL_SIZE_Y;
	}else{
		xq = xr = ts->offset_pen.diff[10];
		yp = yq = ts->offset_pen.diff[11];
		base_xp = ts->offset_pen.base[1];
		base_xq = CONFIG_SHTPS_NASSAU_PANEL_SIZE_X;
		base_yp = ts->offset_pen.base[4];
		base_yq = CONFIG_SHTPS_NASSAU_PANEL_SIZE_Y;
	}

	pq = (xq - xp) * (*x - base_xp) / (base_xq - base_xp) + xp;
	rs = (xs - xr) * (*x - base_xp) / (base_xq - base_xp) + xr;
	*x -= ((rs - pq) * (*y - base_yp) / (base_yq - base_yp) + pq);

	pq = (yq - yp) * (*x - base_xp) / (base_xq - base_xp) + yp;
	rs = (ys - yr) * (*x - base_xp) / (base_xq - base_xp) + yr;
	*y -= ((rs - pq) * (*y - base_yp) / (base_yq - base_yp) + pq);

	SHTPS_LOG_DBG_PRINT("Offset pos PEN : (%4d, %4d) -> (%4d, %4d)\n",
							x_org, y_org, *x, *y);
#endif /* #if deifned( CONFIG_SHTPS_NASSAU_POSITION_OFFSET ) */
	return 0;
}
#endif /* #if defined(SHTPS_PEN_DETECT_ENABLE) */

inline static void shtps_report_touch_on(struct shtps_nas_spi *ts, u8 id, u8 tool, u16 x, u16 y, u8 w, u16 z)
{
	int lcd_x = SHTPS_POS_SCALE_CONVERT_X(x);
	int lcd_y = SHTPS_POS_SCALE_CONVERT_Y(y);
	
	switch (tool) {
#if defined(SHTPS_PEN_DETECT_ENABLE)
	case SHTPS_INPUT_TYPE_PEN:
		shtps_offset_pos_pen(ts, &lcd_x, &lcd_y);
		break;
#endif /* #if defined(SHTPS_PEN_DETECT_ENABLE) */
	default:
		shtps_offset_pos(ts, &lcd_x, &lcd_y);
		break;
	}
	
	w = (w > SHTPS_FINGER_WIDTH_PALMDET   ? SHTPS_FINGER_WIDTH_PALMDET   : w);
	z = (z > SHTPS_NASSAU_MT_PRESSURE_MAX ? SHTPS_NASSAU_MT_PRESSURE_MAX : z);
	
	input_mt_slot(ts->input, id);
	
	switch (tool) {
#if defined(SHTPS_PEN_DETECT_ENABLE)
	case SHTPS_INPUT_TYPE_PEN:
		input_mt_report_slot_state(ts->input, MT_TOOL_PEN, true);
		input_report_abs(ts->input, ABS_MT_TOUCH_MAJOR, w);
		input_report_abs(ts->input, ABS_MT_POSITION_X,  lcd_x);
		input_report_abs(ts->input, ABS_MT_POSITION_Y,  lcd_y);
		input_report_abs(ts->input, ABS_MT_PRESSURE,    z);	
		break;
#endif /* #if defined(SHTPS_PEN_DETECT_ENABLE) */
	case SHTPS_INPUT_TYPE_FINGER:
	default:
		input_mt_report_slot_state(ts->input, MT_TOOL_FINGER, true);
		input_report_abs(ts->input, ABS_MT_TOUCH_MAJOR, w);
		input_report_abs(ts->input, ABS_MT_POSITION_X,  lcd_x);
		input_report_abs(ts->input, ABS_MT_POSITION_Y,  lcd_y);
		input_report_abs(ts->input, ABS_MT_PRESSURE,    z);
		break;
	}

#if defined( SHTPS_POSITION_DIRECT_REPORT_ENABLE )
	SHTPS_LOG_EVT_PRINT("Notify event[ID=%02d] tool=%u, touch=100(%d), x=%d, y=%d w=%d\n",
							id, tool, z, lcd_x, lcd_y, w);
#else
	SHTPS_LOG_EVT_PRINT("Notify event[ID=%02d] tool=%u, touch=100(%d.%03d), x=%d(%d.%03d), y=%d(%d.%03d) w=%d\n",
							id, tool,
							z / 1000, z % 1000, 
							lcd_x, x / 1000, x % 1000, 
							lcd_y, y / 1000, y % 1000, w);
#endif /* #if defined( SHTPS_POSITION_DIRECT_REPORT_ENABLE ) */
}

inline static void shtps_report_touch_off(struct shtps_nas_spi *ts, u8 id, u8 tool, u16 x, u16 y, u8 w, u16 z)
{
	input_mt_slot(ts->input, id);
	
	switch (tool) {
#if defined(SHTPS_PEN_DETECT_ENABLE)
	case SHTPS_INPUT_TYPE_PEN:
		input_mt_report_slot_state(ts->input, MT_TOOL_PEN, false);
		break;
#endif /* #if defined(SHTPS_PEN_DETECT_ENABLE) */
	case SHTPS_INPUT_TYPE_FINGER:
	default:
	input_mt_report_slot_state(ts->input, MT_TOOL_FINGER, false);
		break;
	}

#if defined( SHTPS_POSITION_DIRECT_REPORT_ENABLE )
	SHTPS_LOG_EVT_PRINT("Notify event[ID=%02d] tool=%u, touch=  0(%d), x=%d, y=%d w=%d\n",
							id, tool, z, x, y, w);
#else
	SHTPS_LOG_EVT_PRINT("Notify event[ID=%02d] tool=%u, touch=  0(%d.%03d), x=%d(%d.%03d), y=%d(%d.%03d) w=%d\n",
							id, tool,
							z / 1000, z % 1000, 
							SHTPS_POS_SCALE_CONVERT_X(x), x / 1000, x % 1000, 
							SHTPS_POS_SCALE_CONVERT_Y(y), y / 1000, y % 1000, w);
#endif /* #if defined( SHTPS_POSITION_DIRECT_REPORT_ENABLE ) */
}

static u8 shtps_is_touch_up(struct shtps_nas_spi *ts)
{
	u8 finger;
	for(finger = 0;finger < SHTPS_FINGER_MAX;finger++){
		if(ts->report_info.fingers[finger].id != SHTPS_FINGER_ID_INVALID){
			return 0;
		}
	}
	return 1;
}

static u8 shtps_get_event_type(struct shtps_nas_spi *ts, struct shtps_touch_info *info, int finger)
{
	if(ts->report_info.fingers[finger].id == SHTPS_FINGER_ID_INVALID &&
		info->fingers[finger].id == SHTPS_FINGER_ID_INVALID) {
		return SHTPS_EVENT_TYPE_NONE;
	}
	else if(ts->report_info.fingers[finger].id == SHTPS_FINGER_ID_INVALID &&
		info->fingers[finger].id != SHTPS_FINGER_ID_INVALID) {		
		return SHTPS_EVENT_TYPE_TOUCHDOWN;
	}
	else if(ts->report_info.fingers[finger].id != SHTPS_FINGER_ID_INVALID && 
				info->fingers[finger].id == SHTPS_FINGER_ID_INVALID) {
		return SHTPS_EVENT_TYPE_TOUCHUP_BY_ID_INVALID;		
	}
	else {
		if(info->fingers[finger].state == SHTPS_FINGER_STATE_TU) {
			return SHTPS_EVENT_TYPE_TOUCHUP_BY_STATE;			
		}
		else {
			return SHTPS_EVENT_TYPE_DRAG;
		}
	}
}

static void shtps_event_report(struct shtps_nas_spi *ts, struct shtps_touch_info *info)
{
	u8 finger;

	for (finger=0; finger<SHTPS_FINGER_MAX; finger++) {
		
		switch (shtps_get_event_type(ts, info, finger)) {
		case SHTPS_EVENT_TYPE_TOUCHDOWN:
			shtps_report_touch_on(	ts, 
									info->fingers[finger].id,
									info->fingers[finger].tool,
									info->fingers[finger].x,
									info->fingers[finger].y,
									info->fingers[finger].w,
									info->fingers[finger].z);
			break;
		case SHTPS_EVENT_TYPE_TOUCHUP_BY_ID_INVALID:
			shtps_report_touch_off(	ts, 
									ts->report_info.fingers[finger].id,
									ts->report_info.fingers[finger].tool,
									ts->report_info.fingers[finger].x,
									ts->report_info.fingers[finger].y,
									ts->report_info.fingers[finger].w,
									ts->report_info.fingers[finger].z);
			break;
		case SHTPS_EVENT_TYPE_TOUCHUP_BY_STATE:
			shtps_report_touch_off(	ts, 
									info->fingers[finger].id,
									info->fingers[finger].tool,
									info->fingers[finger].x,
									info->fingers[finger].y,
									info->fingers[finger].w,
									info->fingers[finger].z);
			info->fingers[finger].id = SHTPS_FINGER_ID_INVALID;
			break;
		case SHTPS_EVENT_TYPE_DRAG:
			shtps_report_touch_on(	ts, 
									info->fingers[finger].id,
									info->fingers[finger].tool,
									info->fingers[finger].x,
									info->fingers[finger].y,
									info->fingers[finger].w,
									info->fingers[finger].z);
			break;
		case SHTPS_EVENT_TYPE_NONE:
		default:
			continue;
		}
		
	}

	input_sync(ts->input);
	memcpy(&ts->report_info, info, sizeof(ts->report_info));
	shtps_wakeup_diag_touchevent(ts);
}

static void shtps_event_force_touchup(struct shtps_nas_spi *ts)
{
	u8 finger;

	SHTPS_LOG_DBG_PRINT("%s()\n", __func__);
	for(finger = 0;finger < SHTPS_FINGER_MAX;finger++){
		if(ts->report_info.fingers[finger].id != SHTPS_FINGER_ID_INVALID){
			shtps_report_touch_off(	ts, 
									ts->report_info.fingers[finger].id,
									ts->report_info.fingers[finger].tool,
									ts->report_info.fingers[finger].x,
									ts->report_info.fingers[finger].y,
									ts->report_info.fingers[finger].w,
									ts->report_info.fingers[finger].z);
		}
	}
	input_sync(ts->input);
	shtps_clear_fingerinfo(&ts->report_info);
}

static void shtps_pen_hand_event_report(struct shtps_nas_spi *ts, struct shtps_touch_info *info)
{
	#if defined(SHTPS_PEN_HAND_DETECT_ENABLE)
	{
		u8 finger;
		int cnt = 0;

		for (finger = 0; finger < SHTPS_FINGER_MAX; finger++) {
			if (info->fingers[finger].w == 0xFF) {
				switch (shtps_get_event_type(ts, info, finger)) {
					case SHTPS_EVENT_TYPE_TOUCHUP_BY_STATE:
						shtps_report_touch_on(	ts, 
												info->fingers[finger].id,
												info->fingers[finger].tool,
												SHTPS_PEN_HAND_COORDINATES_X,
												SHTPS_PEN_HAND_COORDINATES_Y,
												info->fingers[finger].w,
												info->fingers[finger].z);
						cnt++;
						break;

					default:
						continue;
				}
			}
		}
		if (cnt > 0){
			SHTPS_LOG_DBG_PRINT("Pen Hand Detected\n");
			input_sync(ts->input);
		}
	}
	#endif /* SHTPS_PEN_HAND_DETECT_ENABLE */
}

#if defined(SHTPS_DRAG_STEP_ENABLE)
static int shtps_get_diff(unsigned short pos1, unsigned short pos2)
{
	int diff = pos1 - pos2;

	return (diff >= 0)? diff : -diff;
}

static void shtps_rec_notify_time(struct shtps_nas_spi *ts, int xy, int index)
{
	ts->touch_state.drag_timeout[index][xy] = jiffies + msecs_to_jiffies(ts->touch_state.dragStepReturnTime[index][xy]);
}

static int shtps_chk_notify_time(struct shtps_nas_spi *ts, int xy, int index)
{
	if(time_after(jiffies, ts->touch_state.drag_timeout[index][xy])){
		return -1;
	}
	return 0;
}

static int shtps_get_dragstep(struct shtps_nas_spi *ts, int xy, int type, int fingers, u8 tool)
{
	int dragStep;

	if(type == SHTPS_DRAG_THRESHOLD_ZERO){
		if(xy == SHTPS_POSTYPE_X){
			if(tool == SHTPS_INPUT_TYPE_FINGER){
				dragStep = (fingers <= 1)? SHTPS_DRAG_THRESH_VAL_X_ZERO : SHTPS_DRAG_THRESH_VAL_X_1ST_MULTI;
			}else{
				dragStep = (fingers <= 1)? SHTPS_PEN_DRAG_THRESH_VAL_X_ZERO : SHTPS_PEN_DRAG_THRESH_VAL_X_1ST_MULTI;
			}
		}else{
			if(tool == SHTPS_INPUT_TYPE_FINGER){
				dragStep = (fingers <= 1)? SHTPS_DRAG_THRESH_VAL_Y_ZERO : SHTPS_DRAG_THRESH_VAL_Y_1ST_MULTI;
			}else{
				dragStep = (fingers <= 1)? SHTPS_PEN_DRAG_THRESH_VAL_Y_ZERO : SHTPS_PEN_DRAG_THRESH_VAL_Y_1ST_MULTI;
			}
		}
	}else if(type == SHTPS_DRAG_THRESHOLD_1ST){
		if(xy == SHTPS_POSTYPE_X){
			if(tool == SHTPS_INPUT_TYPE_FINGER){
				dragStep = (fingers <= 1)? SHTPS_DRAG_THRESH_VAL_X_1ST : SHTPS_DRAG_THRESH_VAL_X_1ST_MULTI;
			}else{
				dragStep = (fingers <= 1)? SHTPS_PEN_DRAG_THRESH_VAL_X_1ST : SHTPS_PEN_DRAG_THRESH_VAL_X_1ST_MULTI;
			}
		}else{
			if(tool == SHTPS_INPUT_TYPE_FINGER){
				dragStep = (fingers <= 1)? SHTPS_DRAG_THRESH_VAL_Y_1ST : SHTPS_DRAG_THRESH_VAL_Y_1ST_MULTI;
			}else{
				dragStep = (fingers <= 1)? SHTPS_PEN_DRAG_THRESH_VAL_Y_1ST : SHTPS_PEN_DRAG_THRESH_VAL_Y_1ST_MULTI;
			}
		}
	}else{
		if(xy == SHTPS_POSTYPE_X){
			if(tool == SHTPS_INPUT_TYPE_FINGER){
				dragStep = (fingers <= 1)? SHTPS_DRAG_THRESH_VAL_X_2ND : SHTPS_DRAG_THRESH_VAL_X_2ND_MULTI;
			}else{
				dragStep = (fingers <= 1)? SHTPS_PEN_DRAG_THRESH_VAL_X_2ND : SHTPS_PEN_DRAG_THRESH_VAL_X_2ND_MULTI;
			}
		}else{
			if(tool == SHTPS_INPUT_TYPE_FINGER){
				dragStep = (fingers <= 1)? SHTPS_DRAG_THRESH_VAL_Y_2ND : SHTPS_DRAG_THRESH_VAL_Y_2ND_MULTI;
			}else{
				dragStep = (fingers <= 1)? SHTPS_PEN_DRAG_THRESH_VAL_Y_2ND : SHTPS_PEN_DRAG_THRESH_VAL_Y_2ND_MULTI;
			}
		}
	}

	return dragStep;
}

static void shtps_set_dragstep(struct shtps_nas_spi *ts,
		struct shtps_touch_info *info, int type, int xy, int finger)
{
	if(type == SHTPS_DRAG_THRESHOLD_ZERO){
		ts->touch_state.dragStep[finger][xy] = type;
		ts->touch_state.dragStepReturnTime[finger][xy] = SHTPS_DRAG_THRESH_RETURN_TIME_ZERO;
	}else if(type == SHTPS_DRAG_THRESHOLD_1ST){
		ts->touch_state.dragStep[finger][xy] = type;
		ts->touch_state.dragStepReturnTime[finger][xy] = SHTPS_DRAG_THRESH_RETURN_TIME;
	}else{
		if(xy == SHTPS_POSTYPE_X){
			ts->center_info.fingers[finger].x = info->fingers[finger].x;
		}else{
			ts->center_info.fingers[finger].y = info->fingers[finger].y;
		}
		ts->touch_state.dragStep[finger][xy] = type;
		shtps_rec_notify_time(ts, xy, finger);
	}
}

static inline void shtps_init_drag_hist(struct shtps_nas_spi *ts, int xy, int finger, int pos)
{
	ts->drag_hist[finger][xy].pre   = pos;
	ts->drag_hist[finger][xy].count = 0;
}

static void shtps_add_drag_hist(struct shtps_nas_spi *ts, int xy, int finger, int pos)
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
#endif /* SHTPS_DRAG_STEP_ENABLE */

static void shtps_calc_notify(struct shtps_nas_spi *ts, struct shtps_touch_info *info)
{
	u8  event = 0xff;

	#if defined(SHTPS_DRAG_STEP_ENABLE)
	/** Position pre proc */
	{
		int i;
		u8  numOfFingers = 0;
		u8  numOfPen = 0;
		int diff_x;
		int diff_y;
		int diff_cx;
		int diff_cy;
		int dragStep1stX;
		int dragStep1stY;
		int FingerDragStep1stX;
		int FingerDragStep1stY;
		int PenDragStep1stX;
		int PenDragStep1stY;
		int dragStepCurX;
		int dragStepCurY;

		SHTPS_LOG_DBG_PRINT("%s()\n", __func__);

		for (i = 0; i < SHTPS_FINGER_MAX; i++)
		{
			u8 event_type = SHTPS_EVENT_TYPE_NONE;

			event_type = shtps_get_event_type(ts, info, i);
			if( (event_type == SHTPS_EVENT_TYPE_TOUCHDOWN) || (event_type == SHTPS_EVENT_TYPE_DRAG) ){
				if(info->fingers[i].tool == SHTPS_INPUT_TYPE_FINGER){
					numOfFingers++;
				}else{
					numOfPen++;
				}
			}
		}

		FingerDragStep1stX = shtps_get_dragstep(ts, SHTPS_POSTYPE_X, SHTPS_DRAG_THRESHOLD_1ST, numOfFingers, SHTPS_INPUT_TYPE_FINGER);
		FingerDragStep1stY = shtps_get_dragstep(ts, SHTPS_POSTYPE_Y, SHTPS_DRAG_THRESHOLD_1ST, numOfFingers, SHTPS_INPUT_TYPE_FINGER);

		PenDragStep1stX = shtps_get_dragstep(ts, SHTPS_POSTYPE_X, SHTPS_DRAG_THRESHOLD_1ST, numOfPen, SHTPS_INPUT_TYPE_PEN);
		PenDragStep1stY = shtps_get_dragstep(ts, SHTPS_POSTYPE_Y, SHTPS_DRAG_THRESHOLD_1ST, numOfPen, SHTPS_INPUT_TYPE_PEN);

		for(i = 0; i < SHTPS_FINGER_MAX; i++)
		{
			u8 event_type = SHTPS_EVENT_TYPE_NONE;

			if(info->fingers[i].tool == SHTPS_INPUT_TYPE_FINGER){
				dragStep1stX = FingerDragStep1stX;
				dragStep1stY = FingerDragStep1stY;
				dragStepCurX = shtps_get_dragstep(ts, SHTPS_POSTYPE_X, ts->touch_state.dragStep[i][SHTPS_POSTYPE_X], numOfFingers, SHTPS_INPUT_TYPE_FINGER);
				dragStepCurY = shtps_get_dragstep(ts, SHTPS_POSTYPE_Y, ts->touch_state.dragStep[i][SHTPS_POSTYPE_Y], numOfFingers, SHTPS_INPUT_TYPE_FINGER);
			}else{
				dragStep1stX = PenDragStep1stX;
				dragStep1stY = PenDragStep1stY;
				dragStepCurX = shtps_get_dragstep(ts, SHTPS_POSTYPE_X, ts->touch_state.dragStep[i][SHTPS_POSTYPE_X], numOfPen, SHTPS_INPUT_TYPE_PEN);
				dragStepCurY = shtps_get_dragstep(ts, SHTPS_POSTYPE_Y, ts->touch_state.dragStep[i][SHTPS_POSTYPE_Y], numOfPen, SHTPS_INPUT_TYPE_PEN);
			}

			event_type = shtps_get_event_type(ts, info, i);

			if(event_type == SHTPS_EVENT_TYPE_DRAG){
				shtps_add_drag_hist(ts, SHTPS_POSTYPE_X, i, info->fingers[i].x);
				shtps_add_drag_hist(ts, SHTPS_POSTYPE_Y, i, info->fingers[i].y);

				diff_x = shtps_get_diff(info->fingers[i].x, ts->report_info.fingers[i].x);
				diff_y = shtps_get_diff(info->fingers[i].y, ts->report_info.fingers[i].y);
				diff_cx= shtps_get_diff(info->fingers[i].x, ts->center_info.fingers[i].x);
				diff_cy= shtps_get_diff(info->fingers[i].y, ts->center_info.fingers[i].y);

				if(diff_cy >= dragStep1stY){
					if(ts->touch_state.dragStep[i][SHTPS_POSTYPE_Y] != SHTPS_DRAG_THRESHOLD_2ND){
						shtps_set_dragstep(ts, info, SHTPS_DRAG_THRESHOLD_2ND, SHTPS_POSTYPE_X, i);
						if(info->fingers[i].tool == SHTPS_INPUT_TYPE_FINGER){
							dragStepCurX = shtps_get_dragstep(ts, SHTPS_POSTYPE_X,
												ts->touch_state.dragStep[i][SHTPS_POSTYPE_X], numOfFingers, SHTPS_INPUT_TYPE_FINGER);
						}else{
							dragStepCurX = shtps_get_dragstep(ts, SHTPS_POSTYPE_X,
												ts->touch_state.dragStep[i][SHTPS_POSTYPE_X], numOfPen, SHTPS_INPUT_TYPE_PEN);
						}
					}
				}

				if(diff_x >= dragStepCurX){
					if(diff_cx >= dragStep1stX){
						event = SHTPS_EVENT_DRAG;
						if(ts->touch_state.dragStep[i][SHTPS_POSTYPE_X] != SHTPS_DRAG_THRESHOLD_2ND){
							shtps_set_dragstep(ts, info, SHTPS_DRAG_THRESHOLD_2ND, SHTPS_POSTYPE_Y, i);
							if(info->fingers[i].tool == SHTPS_INPUT_TYPE_FINGER){
								dragStepCurY = shtps_get_dragstep(ts, SHTPS_POSTYPE_Y,
													ts->touch_state.dragStep[i][SHTPS_POSTYPE_Y], numOfFingers, SHTPS_INPUT_TYPE_FINGER);
							}else{
								dragStepCurY = shtps_get_dragstep(ts, SHTPS_POSTYPE_Y,
													ts->touch_state.dragStep[i][SHTPS_POSTYPE_Y], numOfPen, SHTPS_INPUT_TYPE_PEN);
							}
						}
						shtps_set_dragstep(ts, info, SHTPS_DRAG_THRESHOLD_2ND, SHTPS_POSTYPE_X, i);

					}else if(shtps_chk_notify_time(ts, 0, i) == 0 ||
								ts->touch_state.dragStep[i][SHTPS_POSTYPE_X] != SHTPS_DRAG_THRESHOLD_2ND){
						event = SHTPS_EVENT_DRAG;

					}else{
						info->fingers[i].x = ts->report_info.fingers[i].x;
						shtps_set_dragstep(ts, info, SHTPS_DRAG_THRESHOLD_1ST, SHTPS_POSTYPE_X, i);
					}
				}else{
					info->fingers[i].x = ts->report_info.fingers[i].x;
				}

				if(diff_y >= dragStepCurY){
					if(diff_cy >= dragStep1stY){
						event = SHTPS_EVENT_DRAG;
						shtps_set_dragstep(ts, info, SHTPS_DRAG_THRESHOLD_2ND, SHTPS_POSTYPE_Y, i);

					}else if(shtps_chk_notify_time(ts, 1, i) == 0 ||
								ts->touch_state.dragStep[i][SHTPS_POSTYPE_Y] != SHTPS_DRAG_THRESHOLD_2ND)
					{
						event = SHTPS_EVENT_DRAG;
						ts->touch_state.dragStep[i][SHTPS_POSTYPE_Y] = SHTPS_DRAG_THRESHOLD_2ND;

					}else{
						info->fingers[i].y = ts->report_info.fingers[i].y;
						shtps_set_dragstep(ts, info, SHTPS_DRAG_THRESHOLD_1ST, SHTPS_POSTYPE_Y, i);
					}
				}else{
					info->fingers[i].y = ts->report_info.fingers[i].y;
				}
			}else if(event_type == SHTPS_EVENT_TYPE_TOUCHDOWN){
				ts->center_info.fingers[i].x = info->fingers[i].x;
				ts->center_info.fingers[i].y = info->fingers[i].y;
			}else{
				shtps_set_dragstep(ts, info, SHTPS_DRAG_THRESHOLD_ZERO, SHTPS_POSTYPE_X, i);
				shtps_set_dragstep(ts, info, SHTPS_DRAG_THRESHOLD_ZERO, SHTPS_POSTYPE_Y, i);

				shtps_init_drag_hist(ts, SHTPS_POSTYPE_X, i, info->fingers[i].x);
				shtps_init_drag_hist(ts, SHTPS_POSTYPE_Y, i, info->fingers[i].y);
			}

			if( (event_type == SHTPS_EVENT_TYPE_TOUCHDOWN) ||
				(event_type == SHTPS_EVENT_TYPE_TOUCHUP_BY_ID_INVALID) ||
				(event_type == SHTPS_EVENT_TYPE_TOUCHUP_BY_STATE) )
			{
				event = SHTPS_EVENT_MTDU;
			}
		}

		if( (numOfFingers + numOfPen) > 0){
			if(ts->touch_state.numOfFingers == 0){
				event = SHTPS_EVENT_TD;
			}
		}else{
			if(ts->touch_state.numOfFingers != 0){
				event = SHTPS_EVENT_TU;
			}
		}

		if(event != 0xff){
			ts->touch_state.numOfFingers = (numOfFingers + numOfPen);
		}
	}

	#else
		event = SHTPS_EVENT_DRAG;
	#endif /* SHTPS_DRAG_STEP_ENABLE */

	#if defined(SHTPS_CPU_CLOCK_CONTROL_ENABLE)
		if(event == SHTPS_EVENT_TD){
			shtps_perf_lock_enable(ts);
			shtps_perf_lock_disable_timer_start(ts, ts->perf_lock_enable_time_ms);
			SHTPS_LOG_DBG_PRINT("perf_lock start by TouchDown\n");
		}else if(event == SHTPS_EVENT_DRAG){
			if(ts->report_event == SHTPS_EVENT_TD){
				shtps_perf_lock_enable(ts);
				shtps_perf_lock_disable_timer_start(ts, ts->perf_lock_enable_time_ms);
				SHTPS_LOG_DBG_PRINT("perf_lock start by Drag\n");
			}
		}else if(event == SHTPS_EVENT_TU){
			shtps_perf_lock_disable(ts);
			SHTPS_LOG_DBG_PRINT("perf_lock end by TouchUp\n");
		}
	#endif /* SHTPS_CPU_CLOCK_CONTROL_ENABLE */

	/** Event notify */
	if(event != 0xff){
		shtps_event_report(ts, info);

		#if defined(SHTPS_CPU_CLOCK_CONTROL_ENABLE)
			ts->report_event = event;
		#endif /* SHTPS_CPU_CLOCK_CONTROL_ENABLE */
	}
}

static void shtps_read_touchevent(struct shtps_nas_spi *ts, int state)
{
	struct shtps_touch_info info;
	u8 finger;
	u8 finger_num;
	u8 buf[5 + 8 * SHTPS_FINGER_MAX];

	if(shtps_reg_read(ts, SHTPS_NAS_REG_CMD, buf, 5) != 0){
		/** SPI read error */
		goto err_exit;
	}
	
	finger_num = buf[0] & 0x0F;
	SHTPS_LOG_DBG_PRINT("%s() finger num = %d\n", __func__, finger_num);
	
	if(shtps_reg_read(ts, 0x0D, &buf[5], 8 * finger_num) != 0){
		/** SPI read error */
		goto err_exit;
	}
	
	shtps_clear_fingerinfo(&info);
	for(finger = 0;finger < finger_num;finger++){
		shtps_set_fingerinfo(ts, &info, &buf[5 + finger * 8]);
	}
	
	shtps_pen_hand_event_report(ts, &info);
	
	shtps_calc_notify(ts, &info);
	if(shtps_is_touch_up(ts)){
		;
	}
	
	shtps_next_touch_report_req(ts);

	return;
	
err_exit:
	shtps_next_touch_report_req(ts);
}

/* -----------------------------------------------------------------------------------
 */
static void shtps_testmode_clr_ready(struct shtps_nas_spi *ts)
{
	SHTPS_LOG_DBG_PRINT("%s()\n", __func__);
	ts->testmode.ready = 0;
	ts->testmode.cancel= 0;
}

static void shtps_testmode_ready(struct shtps_nas_spi *ts)
{
	SHTPS_LOG_DBG_PRINT("%s()\n", __func__);
	ts->testmode.ready = 1;
	wake_up_interruptible(&ts->testmode.wait);
}

static void shtps_testmode_cancel(struct shtps_nas_spi *ts)
{
	SHTPS_LOG_DBG_PRINT("%s()\n", __func__);
	ts->testmode.cancel = 1;
	wake_up_interruptible(&ts->testmode.wait);
}

static int shtps_testmode_getdata(struct shtps_nas_spi *ts, u8 *buf, int len)
{
	u8 *rp;
	int page;
	int size;
	int ret = 0;
	int remain = len;

	SHTPS_LOG_DBG_PRINT("%s()\n", __func__);

	ret = wait_event_interruptible_timeout(
			ts->testmode.wait,
			(ts->testmode.ready || ts->testmode.cancel),
			msecs_to_jiffies(SHTPS_NASSAU_TMENTER_TIMEOUT_MS));	

	SHTPS_LOG_DBG_PRINT("%s() wakeup\n", __func__);
	if( (ret <= 0) || (ts->testmode.cancel != 0) ){
		SHTPS_LOG_DBG_PRINT("%s() canceled [%d]\n", __func__, ret);
		ts->testmode.cancel = 0;
		return -1;
	}

	SHTPS_LOG_DBG_PRINT("mutex_lock()\n");
	mutex_lock(&shtps_ctrl_lock);

	if ((ts->testmode.opt & SHTPS_TESTMODE_OPT_120HZ) != 0) {
		ts->testmode.ready = 0;
#if defined( SHTPS_TESTMODE_120HZ_WAIT_VARIABLE )
		if (shtps_testmode_120hz_wait_ms > 0)
			mdelay(shtps_testmode_120hz_wait_ms);
#else
		mdelay(SHTPS_TESTMODE_120HZ_WAIT_MS);
#endif /*#if defined( SHTPS_TESTMODE_120HZ_WAIT_VARIABLE ) */
		shtps_reg_write_single(ts, 0x02, 0x0C);
		shtps_reg_write_single(ts, 0x0D, 0x01);
	}

	if ((ret = shtps_reg_write_single(ts, SHTPS_NAS_REG_BANK, 0x02)) != 0) {
		SHTPS_LOG_ERR_PRINT("%s() dc map num bank change error\n", __func__);
		goto error_exit;
	}
	if ((ret = shtps_reg_read(ts, SHTPS_NAS_REG_CMD + 1, buf, 2)) != 0) {
		SHTPS_LOG_ERR_PRINT("%s() dc map num read error\n", __func__);
		goto error_exit;
	}	
	remain = (int)(buf[0] & 0xFF) * (int)(buf[1] & 0xFF) * 2;
	rp = buf + 2;

	for(page = 0x80;page < 0xBF;page++){
		size = (remain > SHTPS_NASSAU_DCMAP_PAGE_SIZE)?
					SHTPS_NASSAU_DCMAP_PAGE_SIZE : remain;
			
		if((ret = shtps_reg_write_single(ts, SHTPS_NAS_REG_BANK, page)) != 0){
			SHTPS_LOG_ERR_PRINT("%s() bank change error\n", __func__);
			goto error_exit;
		}
		if((ret = shtps_reg_read(ts, SHTPS_NAS_REG_CMD, rp, size)) != 0){
			SHTPS_LOG_ERR_PRINT("%s() dc map data read error\n", __func__);
			goto error_exit;
		}
		
		rp += size;
		remain -= size;
		if(remain <= 0){
			break;
		}
	}
	
error_exit:
	if ((ts->testmode.opt & SHTPS_TESTMODE_OPT_120HZ) == 0) {
		shtps_reg_write_single(ts, 0x02, 0x0C);
		shtps_reg_write_single(ts, 0x0D, 0x01);
		ts->testmode.ready = 0;
	}

	{
		u8 val;

		shtps_reg_write_single(ts, 0x02, 0x10);
		shtps_reg_read(ts, 0x20, &val, 0x01);
		if(val != 0x01){
			SHTPS_LOG_ERR_PRINT("%s() reg verify error [0x%X]\n", __func__, val);
			ret = -1;
		}
	}

	SHTPS_LOG_DBG_PRINT("mutex_unlock()\n");
	mutex_unlock(&shtps_ctrl_lock);
	return ret;
}

/* -----------------------------------------------------------------------------------
 */
static void shtps_loader_clr_ready(struct shtps_nas_spi *ts)
{
	SHTPS_LOG_DBG_PRINT("%s()\n", __func__);
	ts->loader.ready  = 0;
	ts->loader.cancel = 0;
}

static void shtps_loader_ready(struct shtps_nas_spi *ts)
{
	SHTPS_LOG_DBG_PRINT("%s()\n", __func__);
	ts->loader.ready  = 1;
	wake_up_interruptible(&ts->loader.wait);
}

static void shtps_loader_cancel(struct shtps_nas_spi *ts)
{
	SHTPS_LOG_DBG_PRINT("%s()\n", __func__);
	ts->loader.cancel = 1;
	wake_up_interruptible(&ts->loader.wait);
}

static int shtps_wait_loader_ready(struct shtps_nas_spi *ts)
{
	int ret = 0;
	SHTPS_LOG_DBG_PRINT("%s()\n", __func__);

#if 0
	ret = wait_event_interruptible_timeout(
				ts->loader.wait, 
				(ts->loader.ready || ts->loader.cancel), 
				msecs_to_jiffies(SHTPS_NASSAU_LOADER_WAIT_ACK_TMO));
	
	SHTPS_LOG_DBG_PRINT("%s() wakeup\n", __func__);
	if(ret == 0 || ts->loader.cancel == 1){
		SHTPS_LOG_DBG_PRINT("%s() canceled\n", __func__);
		ts->loader.cancel = 0;
		return -1;
	}
#else
	ret = wait_event_interruptible(
				ts->loader.wait, 
				(ts->loader.ready || ts->loader.cancel));
	
	SHTPS_LOG_DBG_PRINT("%s() wakeup\n", __func__);
	if(ret != 0 || ts->loader.cancel == 1){
		SHTPS_LOG_DBG_PRINT("%s() canceled\n", __func__);
		ts->loader.cancel = 0;
		return -1;
	}
#endif
	
	ts->loader.ready = 0;
	return 0;
}

static int shtps_get_loader_cmd_result(struct shtps_nas_spi *ts, u8 *result)
{
	if(shtps_reg_write_single(ts, SHTPS_NAS_REG_BANK, SHTPS_NAS_BANK_CMDRESULT)){
		return -1;
	}
	
	if(shtps_reg_read(ts, SHTPS_NAS_REG_CMD + 1, result, 1)){
		return -1;
	}
	
	SHTPS_LOG_DBG_PRINT("%s() result = 0x%02x\n", __func__, *result);
	return 0;
}

static int shtps_loader_erase_sector(struct shtps_nas_spi *ts)
{
	const u8 cmd[] = { 0xE2, 0x00, 0x01, 0x00, 0x00 };
	u8  result;
	
	SHTPS_LOG_DBG_PRINT("%s()\n", __func__);
	if(shtps_write_cmd(ts, (u8*)cmd, sizeof(cmd))){
		return -1;
	}

	if(shtps_wait_loader_ready(ts)){
		return -1;
	}
	
	if(shtps_get_loader_cmd_result(ts, &result)){
		return -1;
	}
	if(result){
		SHTPS_LOG_ERR_PRINT("%s() failed erase sector (%s)\n",
			__func__,
			(result == 0x2D)? "OpCode is invalid" :
			(result == 0x2E)? "Parameter error"   :
			(result == 0x31)? "Erase error" : "Unknown error");
		return -1;
	}

	return 0;
}

static int shtps_loader_write_block(struct shtps_nas_spi *ts, u8 *data, u16 offset, u8 *datasum)
{
	const u8 head[] = { 0xE3, 0x00, SHTPS_NASSAU_LOADER_BLOCK_SIZE, 0x00 };
	
	int i;
	u8 txbuf[0x02 + 0x01 + 0x11 + SHTPS_NASSAU_LOADER_BLOCK_SIZE];
	u8 checksum = 0;
	u8 result;

	SHTPS_LOG_DBG_PRINT("%s() offset = %d\n", __func__, offset);
	if(shtps_reg_write_single(ts, SHTPS_NAS_REG_BANK, SHTPS_NAS_BANK_CMD)){
		SHTPS_LOG_ERR_PRINT("%s() bank change error\n", __func__);
		return -1;
	}
	
	if(shtps_reg_write(ts, SHTPS_NAS_REG_CMD, (u8*)head, sizeof(head))){
		SHTPS_LOG_ERR_PRINT("%s() header write error\n", __func__);
		return -1;
	}

	for(i = 0;i < SHTPS_NASSAU_LOADER_BLOCK_SIZE;i++){
		checksum += data[i];
	}
	*datasum += checksum;

	SHTPS_LOG_DBG_PRINT("%s() checksum = 0x%02x\n", __func__, (~checksum)+1);
	
	memset(txbuf, 0, sizeof(txbuf));
	txbuf[0x00] = offset & 0xFF;
	txbuf[0x01] = (offset >> 0x08) & 0xFF;
	txbuf[0x02] = (~checksum) + 1;
	memcpy(&txbuf[0x14], data, SHTPS_NASSAU_LOADER_BLOCK_SIZE);
	
	if(shtps_reg_write(ts, SHTPS_NAS_REG_CMD + sizeof(head), txbuf, sizeof(txbuf))){
		SHTPS_LOG_ERR_PRINT("%s() data write error\n", __func__);
		return -1;
	}

	if(shtps_reg_write_single(ts, SHTPS_NAS_REG_MICOM, 0x01)){
		SHTPS_LOG_ERR_PRINT("%s() micom request write error\n", __func__);
		return -1;
	}
	
	if(shtps_wait_loader_ready(ts)){
		SHTPS_LOG_ERR_PRINT("%s() write wait error\n", __func__);
		return -1;
	}
	
	if(shtps_get_loader_cmd_result(ts, &result)){
		return -1;
	}
	if(result){
		SHTPS_LOG_ERR_PRINT("%s() failed erase sector (%s)\n",
			__func__,
			(result == 0x2D)? "OpCode is invalid" :
			(result == 0x2E)? "Parameter error"   :
			(result == 0x2F)? "Checksum error" : "Unknown error");
		return -1;
	}
	
	return 0;
}

static int shtps_loader_write_verify(struct shtps_nas_spi *ts, u16 offset, u8 checksum)
{
	const u8 head[] = { 0xE4, 0x00, 0x03, 0x00 };

	u8 result;
	u8 txbuf[3];
	
	SHTPS_LOG_DBG_PRINT("%s() offset = %d\n", __func__, offset);
	if(shtps_reg_write_single(ts, SHTPS_NAS_REG_BANK, SHTPS_NAS_BANK_CMD)){
		SHTPS_LOG_ERR_PRINT("%s() bank change error\n", __func__);
		return -1;
	}

	if(shtps_reg_write(ts, SHTPS_NAS_REG_CMD, (u8*)head, sizeof(head))){
		SHTPS_LOG_ERR_PRINT("%s() verify header write error\n", __func__);
		return -1;
	}
	txbuf[0x00] = offset & 0xFF;
	txbuf[0x01] = (offset >> 0x08) & 0xFF;
	txbuf[0x02] = checksum;
	
	if( shtps_reg_write(ts, SHTPS_NAS_REG_CMD + sizeof(head), txbuf, sizeof(txbuf))){
		SHTPS_LOG_ERR_PRINT("%s() data write error\n", __func__);
		return -1;
	}

	if(shtps_reg_write_single(ts, SHTPS_NAS_REG_MICOM, 0x01)){
		SHTPS_LOG_ERR_PRINT("%s() micom request write error\n", __func__);
		return -1;
	}
	
	if(shtps_wait_loader_ready(ts)){
		SHTPS_LOG_ERR_PRINT("%s() write wait error\n", __func__);
		return -1;
	}
	
	if(shtps_get_loader_cmd_result(ts, &result)){
		return -1;
	}
	if(result){
		SHTPS_LOG_ERR_PRINT("%s() failed erase sector (%s)\n",
			__func__,
			(result == 0x2D)? "OpCode is invalid" :
			(result == 0x2E)? "Parameter error"   :
			(result == 0x2F)? "Checksum error" : 
			(result == 0x33)? "Verify error" : "Unknown error");
		return -1;
	}
	
	return 0;
}

static int shtps_loader_write_image(struct shtps_nas_spi *ts, u8 *data, u16 page_offset, u16 size)
{
	u8  page;
	u8  checksum= 0;
	u16 offset  = 0;
	
	SHTPS_LOG_DBG_PRINT("%s() offset = %d, size = %d\n", __func__, page_offset, size);
	if(data == NULL ||
		size != (SHTPS_NASSAU_LOADER_BLOCK_SIZE * SHTPS_NASSAU_LOADER_PAGE_SIZE)){
		SHTPS_LOG_ERR_PRINT("%s() parameter error, data=%p, size=%d\n", __func__, data, size);
		return -1;
	}
	
	for(page = 0;page < SHTPS_NASSAU_LOADER_PAGE_SIZE;page++){
		if(shtps_loader_write_block(ts, &data[offset], offset, &checksum)){
			SHTPS_LOG_ERR_PRINT("%s() block data write error\n", __func__);
			return -1;
		}
		offset += SHTPS_NASSAU_LOADER_BLOCK_SIZE;
	}
	
	if(shtps_loader_write_verify(ts, page_offset, (~checksum) + 1)){
		SHTPS_LOG_ERR_PRINT("%s() verify data write error\n", __func__);
		return -1;
	}
	
	return 0;
}

static int shtps_boot_erase_sector(struct shtps_nas_spi *ts)
{
	const u8 cmd[] = { 0xE8, 0x00, 0x05, 0x00, 0x44, 0x33, 0x22, 0x11, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00 };
	u8  result;
	
	SHTPS_LOG_DBG_PRINT("%s()\n", __func__);
	if(shtps_write_cmd(ts, (u8*)cmd, sizeof(cmd))){
		return -1;
	}

	if(shtps_wait_loader_ready(ts)){
		return -1;
	}
	
	if(shtps_get_loader_cmd_result(ts, &result)){
		return -1;
	}
	if(result){
		SHTPS_LOG_ERR_PRINT("%s() failed erase sector loader(%s)\n",
			__func__,
			(result == 0x2D)? "OpCode is invalid" :
			(result == 0x2E)? "Parameter error"   :
			(result == 0x31)? "Erase error" : "Unknown error");
		return -1;
	}

	return 0;
}

static int shtps_boot_write_block(struct shtps_nas_spi *ts, u8 *data, u16 offset, u8 *datasum)
{
	const u8 head[] = { 0xE9, 0x00, SHTPS_NASSAU_LOADER_BLOCK_SIZE, 0x00, 0x44, 0x33, 0x22, 0x11 };
	
	int i;
	u8 txbuf[0x02 + 0x01 + 0x0D + SHTPS_NASSAU_LOADER_BLOCK_SIZE];
	u8 checksum = 0;
	u8 result;

	SHTPS_LOG_DBG_PRINT("%s() offset = %d\n", __func__, offset);
	if(shtps_reg_write_single(ts, SHTPS_NAS_REG_BANK, SHTPS_NAS_BANK_CMD)){
		SHTPS_LOG_ERR_PRINT("%s() bank change error\n", __func__);
		return -1;
	}
	
	if(shtps_reg_write(ts, SHTPS_NAS_REG_CMD, (u8*)head, sizeof(head))){
		SHTPS_LOG_ERR_PRINT("%s() header write error\n", __func__);
		return -1;
	}

	for(i = 0;i < SHTPS_NASSAU_LOADER_BLOCK_SIZE;i++){
		checksum += data[i];
	}
	*datasum += checksum;

	SHTPS_LOG_DBG_PRINT("%s() checksum = 0x%02x\n", __func__, (~checksum)+1);
	
	memset(txbuf, 0, sizeof(txbuf));
	txbuf[0x00] = offset & 0xFF;
	txbuf[0x01] = (offset >> 0x08) & 0xFF;
	txbuf[0x02] = (~checksum) + 1;
	memcpy(&txbuf[0x10], data, SHTPS_NASSAU_LOADER_BLOCK_SIZE);
	
	if(shtps_reg_write(ts, SHTPS_NAS_REG_CMD + sizeof(head), txbuf, sizeof(txbuf))){
		SHTPS_LOG_ERR_PRINT("%s() data write error\n", __func__);
		return -1;
	}

	if(shtps_reg_write_single(ts, SHTPS_NAS_REG_MICOM, 0x01)){
		SHTPS_LOG_ERR_PRINT("%s() micom request write error\n", __func__);
		return -1;
	}
	
	if(shtps_wait_loader_ready(ts)){
		SHTPS_LOG_ERR_PRINT("%s() write wait error\n", __func__);
		return -1;
	}
	
	if(shtps_get_loader_cmd_result(ts, &result)){
		return -1;
	}
	if(result){
		SHTPS_LOG_ERR_PRINT("%s() failed send flash data loader command [%d](%s)\n",
			__func__,
			result,
			(result == 0x2D)? "OpCode is invalid" :
			(result == 0x2E)? "Parameter error"   :
			(result == 0x2F)? "Checksum error" : "Unknown error");
		return -1;
	}
	
	return 0;
}

static int shtps_boot_write_verify(struct shtps_nas_spi *ts, u16 offset, u8 checksum)
{
	const u8 head[] = { 0xEA, 0x00, 0x07, 0x00, 0x44, 0x33, 0x22, 0x11 };

	u8 result;
	u8 txbuf[4];
	
	SHTPS_LOG_DBG_PRINT("%s() offset = %d\n", __func__, offset);
	if(shtps_reg_write_single(ts, SHTPS_NAS_REG_BANK, SHTPS_NAS_BANK_CMD)){
		SHTPS_LOG_ERR_PRINT("%s() bank change error\n", __func__);
		return -1;
	}

	if(shtps_reg_write(ts, SHTPS_NAS_REG_CMD, (u8*)head, sizeof(head))){
		SHTPS_LOG_ERR_PRINT("%s() verify header write error\n", __func__);
		return -1;
	}
	txbuf[0x00] = offset & 0xFF;
	txbuf[0x01] = (offset >> 0x08) & 0xFF;
	txbuf[0x02] = checksum;
	txbuf[0x03] = 0x00;
	
	if( shtps_reg_write(ts, SHTPS_NAS_REG_CMD + sizeof(head), txbuf, sizeof(txbuf))){
		SHTPS_LOG_ERR_PRINT("%s() data write error\n", __func__);
		return -1;
	}

	if(shtps_reg_write_single(ts, SHTPS_NAS_REG_MICOM, 0x01)){
		SHTPS_LOG_ERR_PRINT("%s() micom request write error\n", __func__);
		return -1;
	}
	
	if(shtps_wait_loader_ready(ts)){
		SHTPS_LOG_ERR_PRINT("%s() write wait error\n", __func__);
		return -1;
	}
	
	if(shtps_get_loader_cmd_result(ts, &result)){
		return -1;
	}
	if(result){
		SHTPS_LOG_ERR_PRINT("%s() failed write verify data loader command [%d](%s)\n",
			__func__,
			result,
			(result == 0x2D)? "OpCode is invalid" :
			(result == 0x2E)? "Parameter error"   :
			(result == 0x2F)? "Checksum error" : 
			(result == 0x33)? "Verify error" : "Unknown error");
		return -1;
	}
	
	return 0;
}

static int shtps_boot_write_image(struct shtps_nas_spi *ts, u8 *data, u16 page_offset, u16 size)
{
	u8  page;
	u8  checksum= 0;
	u16 offset  = 0;
	
	SHTPS_LOG_DBG_PRINT("%s() offset = %d, size = %d\n", __func__, page_offset, size);
	if(data == NULL ||
		size != (SHTPS_NASSAU_LOADER_BLOCK_SIZE * SHTPS_NASSAU_LOADER_PAGE_SIZE)){
		SHTPS_LOG_ERR_PRINT("%s() parameter error, data=%p, size=%d\n", __func__, data, size);
		return -1;
	}
	
	for(page = 0;page < SHTPS_NASSAU_LOADER_PAGE_SIZE;page++){
		if(shtps_boot_write_block(ts, &data[offset], offset, &checksum)){
			SHTPS_LOG_ERR_PRINT("%s() block data write error\n", __func__);
			return -1;
		}
		offset += SHTPS_NASSAU_LOADER_BLOCK_SIZE;
	}
	
	if(shtps_boot_write_verify(ts, page_offset, (~checksum) + 1)){
		SHTPS_LOG_ERR_PRINT("%s() verify data write error\n", __func__);
		return -1;
	}
	
	return 0;
}

/* -----------------------------------------------------------------------------------
 */
static u8 shtps_command_existing(struct shtps_nas_spi *ts)
{
	SHTPS_LOG_DBG_PRINT("%s() isexsits = %d\n", __func__, (ts->command.func)? 1 : 0);
	if(ts->command.func){
		return 1;
	}
	return 0;
}

static void shtps_command_setprevstate(struct shtps_nas_spi *ts, u8 state)
{
	SHTPS_LOG_DBG_PRINT("%s() prev state <- %d\n", __func__, state);
	ts->command.prevstate = state;
}

static int shtps_command_setfunc(struct shtps_nas_spi *ts, shtps_command_exec_func *func)
{
	SHTPS_LOG_DBG_PRINT("%s()\n", __func__);

	if(ts->command.func){
		SHTPS_LOG_DBG_PRINT("%s() command function is busy\n", __func__);
		return -1;
	}
	
	ts->command.func   = func;
	ts->command.ready  = 0;
	ts->command.cancel = 0;
	return 0;
}

static void shtps_command_ready(struct shtps_nas_spi *ts)
{
	SHTPS_LOG_DBG_PRINT("%s()\n", __func__);
	ts->command.func  = NULL;
	ts->command.ready = 1;
	wake_up_interruptible(&ts->command.wait);
}

static void shtps_command_cancel(struct shtps_nas_spi *ts)
{
	SHTPS_LOG_DBG_PRINT("%s()\n", __func__);
	ts->command.func   = NULL;
	ts->command.cancel = 1;
	wake_up_interruptible(&ts->command.wait);
}

static int shtps_wait_command_result(struct shtps_nas_spi *ts)
{
	int ret = 0;
	SHTPS_LOG_DBG_PRINT("%s()\n", __func__);
	ret = wait_event_interruptible_timeout(
				ts->command.wait, 
				(ts->command.ready || ts->command.cancel), 
				msecs_to_jiffies(SHTPS_NASSAU_COMMAND_WAIT_ACK_TMO));
	
	SHTPS_LOG_DBG_PRINT("%s() wakeup\n", __func__);
	if(ret == 0 || ts->command.cancel == 1){
		SHTPS_LOG_DBG_PRINT("%s() canceled\n", __func__);
		ts->command.cancel = 0;
		return -1;
	}
	
	ts->command.ready = 0;
	return 0;
}

static int shtps_command_read_result(struct shtps_nas_spi *ts)
{
	int ret = 0;
	
	ret = shtps_reg_write_single(ts, SHTPS_NAS_REG_BANK, SHTPS_NAS_BANK_CMDRESULT);
	if(ret){
		SHTPS_LOG_DBG_PRINT("%s() bank change error\n", __func__);
		return -1;
	}
	
	ret = shtps_reg_read(ts, SHTPS_NAS_REG_CMD, ts->command.result, SHTPS_NASSAU_CMD_RESULT_MAX_SIZE);
	if(ret){
		SHTPS_LOG_DBG_PRINT("%s() result read error\n", __func__);
		return -1;
	}
	
	SHTPS_LOG_DBG_PRINT("%s() result status = 0x%02x\n", __func__, ts->command.result[1]);
	return 0;
}

/* -----------------------------------------------------------------------------------
 */
static void shtps_wait_startup(struct shtps_nas_spi *ts)
{
	long remained;

	SHTPS_LOG_DBG_PRINT("%s()\n", __func__);
	remained = wait_event_interruptible_timeout(ts->wait_start,
				ts->state_mgr.state == SHTPS_STATE_SLEEP         ||
				ts->state_mgr.state == SHTPS_STATE_ACTIVE        ||
				ts->state_mgr.state == SHTPS_STATE_TESTMODE      ||
				ts->state_mgr.state == SHTPS_STATE_LOADER,
				msecs_to_jiffies(SHTPS_WAIT_STARTUP_TIMEOUT_MS));

	if(remained > 0){
		SHTPS_LOG_DBG_PRINT("%s() wake_up state=%d\n", __func__, ts->state_mgr.state);
	}else{
		SHTPS_LOG_ERR_PRINT("%s() wake_up timeout!! state=%d\n", __func__, ts->state_mgr.state);
	}
}

static void shtps_notify_startup(struct shtps_nas_spi *ts)
{
	SHTPS_LOG_DBG_PRINT("%s() state=%d\n", __func__, ts->state_mgr.state);
	wake_up_interruptible(&ts->wait_start);
}

/* -----------------------------------------------------------------------------------
 */
#if defined( SHTPS_BOOT_FWUPDATE_ENABLE )
static int shtps_boot_fwupdate_enable_check(struct shtps_nas_spi *ts)
{
	#if defined(SHTPS_SYSTEM_BOOT_MODE_CHECK_ENABLE)
		#if 0
		if( (ts->system_boot_mode == SH_BOOT_O_C) || (ts->system_boot_mode == SH_BOOT_U_O_C) ){
			return 0;
		}
		#endif
	#endif /* SHTPS_SYSTEM_BOOT_MODE_CHECK_ENABLE */

	return 1;
}

static int shtps_fwup_flag_check(void)
{
	#if defined(SHTPS_BOOT_FWUPDATE_FORCE_UPDATE)
		return 1;

	#else
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
	#endif /* SHTPS_BOOT_FWUPDATE_FORCE_UPDATE */
}

static void shtps_fwup_flag_clear(void)
{
	sharp_smem_common_type *smemdata = NULL;

	smemdata = sh_smem_get_common_address();
	if(smemdata != NULL){
		smemdata->shtps_fwup_flag = 0;
	}
}

static void shtps_check_bootloader_form_fw(struct shtps_nas_spi *ts, u32 fwver)
{
	#if defined(SHTPS_BOOT_FWUPDATE_NOT_LOADER_UPDATE)
		ts->bt_select = SHTPS_BOOTLOADER_NEW;

	#elif defined(SHTPS_BOOT_FWUPDATE_FORCE_LOADER_UPDATE)
		ts->bt_select = SHTPS_BOOTLOADER_OLD;

	#else
		if(fwver < SHTPS_NEW_BOOTLOADER_FW_VER_MINIMUM){
			ts->bt_select = SHTPS_BOOTLOADER_OLD;
		}else{
			ts->bt_select = SHTPS_BOOTLOADER_NEW;
		}
	#endif /* SHTPS_BOOT_FWUPDATE_FORCE_LOADER_UPDATE */

	return;
}

static void shtps_work_bootfwupdatef(struct shtps_nas_spi *ts)
{
	int ret;
	int get_fwver_ret = 0;
	int fwupdate = 0;
	u32 fwver = 0;
	u32 paramver = 0;

	get_fwver_ret = shtps_command_setfunc(ts, shtps_write_getfwver_cmds);
	if(get_fwver_ret != 0){
		msleep(10);
		shtps_command_cancel(ts);
		get_fwver_ret = shtps_command_setfunc(ts, shtps_write_getfwver_cmds);
		if(get_fwver_ret != 0){
			SHTPS_LOG_ERR_PRINT("%s() fw ver get command set error\n", __func__);
		}
	}

	if(get_fwver_ret == 0){
		get_fwver_ret = request_event(ts, SHTPS_EVENT_CMDREQUEST, 0);
		if(get_fwver_ret != 0){
			SHTPS_LOG_ERR_PRINT("%s() fw ver get command req error\n", __func__);
		}
	}

	if(get_fwver_ret == 0){
		get_fwver_ret = shtps_wait_command_result(ts);
		if(get_fwver_ret != 0){
			SHTPS_LOG_ERR_PRINT("%s() fw ver get command result error\n", __func__);
		}
	}

	if(get_fwver_ret == 0){
		fwver =  (ts->command.result[ 2])         | 
				 (ts->command.result[ 3] << 0x08) | 
				 (ts->command.result[ 4] << 0x10) | 
				 (ts->command.result[ 5] << 0x18);
					
		paramver=(ts->command.result[10])         | 
				 (ts->command.result[11] << 0x08) | 
				 (ts->command.result[12] << 0x10) | 
				 (ts->command.result[13] << 0x18);
	}

	SHTPS_LOG_DBG_PRINT("%s() cur fwver=%08x%08x\n", __func__, fwver, paramver);

	if(shtps_fwup_flag_check() > 0){
		if(get_fwver_ret != 0){
			SHTPS_LOG_ERR_PRINT("%s() Get FwVer Error Detect\n", __func__);
			fwupdate = 1;
			msleep(10);
		}else{
			if(fwver != SHTPS_FWVER_NEWER || paramver != SHTPS_PARAMVER_NEWER){
				SHTPS_LOG_ERR_PRINT("%s() need fw update. cur fwver=%08x%08x / new fwver=%08x%08x\n", 
										__func__, fwver, paramver, SHTPS_FWVER_NEWER, SHTPS_PARAMVER_NEWER);
				fwupdate = 1;
			}
		}
	}else{
		fwupdate = 0;
	}

	if(ts->fw_startup_state == SHTPS_FW_STARTUP_CRC_ERROR){
		SHTPS_LOG_ERR_PRINT("%s() CRC Error Detect\n", __func__);
		fwupdate = 1;
	}

	if(fwupdate){
		int retry;

		shtps_check_bootloader_form_fw(ts, fwver);

		if(ts->bt_select == SHTPS_BOOTLOADER_NEW)
		{
			ret = request_event(ts, SHTPS_EVENT_STARTLOADER, 0);
			if(!ret){
				ret = shtps_wait_loader_ready(ts);
			}else{
				return;
			}

			for(retry = 0; retry <= SHTPS_BOOT_FWUPDATE_RETRY_MAX; retry++){
				if(retry != 0){
					SHTPS_LOG_ERR_PRINT("%s() fw update error. retry=%d\n", __func__, retry);
				}
				ret = shtps_loader_erase_sector(ts);
				if(!ret){
					u16 offset = 0;
					int size = SHTPS_NASSAU_LOADER_BLOCK_SIZE * SHTPS_NASSAU_LOADER_PAGE_SIZE;

					for(offset = 0;offset < SHTPS_FWSIZE_NEWER;offset += size){
						ret = shtps_loader_write_image(ts, (u8*)&tps_fw_data[offset], offset, size);
						if(ret){
							SHTPS_LOG_ERR_PRINT("%s() fw image write error. offset=%d\n", __func__, offset);
							break;
						}
					}
					if(!ret){
						ts->fw_update_continue = 0;
						request_event(ts, SHTPS_EVENT_STOPLOADER, 0);
						break;
					}
				}else{
					SHTPS_LOG_ERR_PRINT("%s() fw erase error\n", __func__);
				}
			}
		}
		else
		{
			ret = request_event(ts, SHTPS_EVENT_STARTLOADER, 0);
			if(!ret){
				ret = shtps_wait_loader_ready(ts);
			}else{
				return;
			}

			ts->fw_update_continue = 1;
			for(retry = 0; retry <= SHTPS_BOOT_FWUPDATE_RETRY_MAX; retry++){
				if(retry != 0){
					SHTPS_LOG_ERR_PRINT("%s() bootloader update error. retry=%d\n", __func__, retry);
				}
				ret = shtps_boot_erase_sector(ts);
				if(!ret){
					u16 offset = 0;
					int size = SHTPS_NASSAU_LOADER_BLOCK_SIZE * SHTPS_NASSAU_LOADER_PAGE_SIZE;

					for(offset = 0;offset < SHTPS_LDRSIZE_NEWER;offset += size){
						ret = shtps_boot_write_image(ts, (u8*)&tps_ldr_data[offset], offset, size);
						if(ret){
							SHTPS_LOG_ERR_PRINT("%s() bootloader image write error. offset=%d\n", __func__, offset);
							break;
						}
					}
					if(!ret){
						request_event(ts, SHTPS_EVENT_STOPLOADER, 0);
						break;
					}
				}else{
					SHTPS_LOG_ERR_PRINT("%s() bootloader erase error\n", __func__);
				}
			}

			if(ret != 0){
				return;
			}

			for(retry = 0; retry < 10; retry++){
				if(ts->state_mgr.state == SHTPS_STATE_LOADER){
					break;
				}
				msleep(100);
			}
			ret = shtps_wait_loader_ready(ts);
			if(ret != 0){
				SHTPS_LOG_ERR_PRINT("%s() [E]state is not loader [%d]\n", __func__, ts->state_mgr.state);
				return;
			}

			{
				//request_event(ts, SHTPS_EVENT_STARTLOADER, 0);
				ret = shtps_loader_erase_sector(ts);
				if(ret != 0){
					SHTPS_LOG_ERR_PRINT("%s() fw erase error\n", __func__);
					return;
				}

				ts->fw_update_continue = 1;
				request_event(ts, SHTPS_EVENT_STOPLOADER, 0);

				for(retry = 0; retry < 10; retry++){
					if(ts->state_mgr.state == SHTPS_STATE_LOADER){
						break;
					}
					msleep(100);
				}
				ret = shtps_wait_loader_ready(ts);
				if(ret != 0){
					SHTPS_LOG_ERR_PRINT("%s() [E]state is not loader [%d]\n", __func__, ts->state_mgr.state);
					return;
				}
			}

			//request_event(ts, SHTPS_EVENT_STARTLOADER, 0);
			for(retry = 0; retry <= SHTPS_BOOT_FWUPDATE_RETRY_MAX; retry++){
				if(retry != 0){
					SHTPS_LOG_ERR_PRINT("%s() fw update error. retry=%d\n", __func__, retry);
				}
				ret = shtps_loader_erase_sector(ts);
				if(!ret){
					u16 offset = 0;
					int size = SHTPS_NASSAU_LOADER_BLOCK_SIZE * SHTPS_NASSAU_LOADER_PAGE_SIZE;

					for(offset = 0;offset < SHTPS_FWSIZE_NEWER;offset += size){
						ret = shtps_loader_write_image(ts, (u8*)&tps_fw_data[offset], offset, size);
						if(ret){
							SHTPS_LOG_ERR_PRINT("%s() fw image write error. offset=%d\n", __func__, offset);
							break;
						}
					}
					if(!ret){
						ts->fw_update_continue = 0;
						request_event(ts, SHTPS_EVENT_STOPLOADER, 0);
						break;
					}
				}else{
					SHTPS_LOG_ERR_PRINT("%s() fw erase error\n", __func__);
				}
			}
		}
	}else{
		request_event(ts, SHTPS_EVENT_START, 1);
	}
	shtps_fwup_flag_clear();

	shtps_wait_startup(ts);
}
#endif /* #if defined( SHTPS_BOOT_FWUPDATE_ENABLE ) */

#if defined( SHTPS_ASYNC_OPEN_ENABLE )
static void shtps_func_open(struct shtps_nas_spi *ts)
{
	#if defined( SHTPS_BOOT_FWUPDATE_ENABLE )
		if( shtps_boot_fwupdate_enable_check(ts) != 0 ){
			shtps_work_bootfwupdatef(ts);
		}
	#endif /* #if defined( SHTPS_BOOT_FWUPDATE_ENABLE ) */

	request_event(ts, SHTPS_EVENT_START, 0);
	shtps_wait_startup(ts);
}

static void shtps_func_close(struct shtps_nas_spi *ts)
{
	request_event(ts, SHTPS_EVENT_STOP, 0);
}

static int shtps_func_enable(struct shtps_nas_spi *ts)
{
	int ret;
	ret = request_event(ts, SHTPS_EVENT_START, 0);
	shtps_wait_startup(ts);
	return ret;
}

static int shtps_func_disable(struct shtps_nas_spi *ts)
{
	return request_event(ts, SHTPS_EVENT_STOP, 0);
}

static void shtps_func_request_async_complete(void *arg_p)
{
	kfree( arg_p );
}

static void shtps_func_request_async( struct shtps_nas_spi *ts, int event)
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

static int shtps_func_request_sync( struct shtps_nas_spi *ts, int event)
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
	struct shtps_nas_spi	*ts;
	unsigned long			flags;

	ts = container_of(work_p, struct shtps_nas_spi, work_data);

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
				ts->cur_msg_p->status = shtps_func_disable(ts);;
				break;

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
static int request_event(struct shtps_nas_spi *ts, int event, int param)
{
	int ret;

	SHTPS_LOG_DBG_PRINT("mutex_lock()\n");
	mutex_lock(&shtps_ctrl_lock);
	
	SHTPS_LOG_DBG_PRINT("event %d in state %d\n", event, ts->state_mgr.state);
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
	case SHTPS_EVENT_STOPLOADER:
		ret = state_func_tbl[ts->state_mgr.state]->stop_ldr(ts, param);
		break;
	case SHTPS_EVENT_STARTTM:
		ret = state_func_tbl[ts->state_mgr.state]->start_tm(ts, param);
		break;
	case SHTPS_EVENT_STOPTM:
		ret = state_func_tbl[ts->state_mgr.state]->stop_tm(ts, param);
		break;
	case SHTPS_EVENT_CMDREQUEST:
		ret = state_func_tbl[ts->state_mgr.state]->cmd_request(ts, param);
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

static int state_change(struct shtps_nas_spi *ts, int state)
{
	int ret = 0;
	int old_state = ts->state_mgr.state;

	SHTPS_LOG_DBG_PRINT("state %d -> %d\n", ts->state_mgr.state, state);

	if(ts->state_mgr.state != state){
		ts->state_mgr.state = state;
		ret = state_func_tbl[ts->state_mgr.state]->enter(ts, old_state);
	}
	return ret;
}

/* -----------------------------------------------------------------------------------
 */
static void shtps_set_pending_flg(struct shtps_nas_spi *ts, int flag)
{
	ts->pending |= flag;
	
	if(flag & SHTPS_PENDING_FLAG_WAKEUP){
		SHTPS_LOG_DBG_PRINT("%s() pending flag(0x%04x) & 0x%04x = 0x%04x\n",
			__func__, ts->pending, ~SHTPS_PENDING_FLAG_SLEEP, ts->pending & ~SHTPS_PENDING_FLAG_SLEEP);
		ts->pending &= ~SHTPS_PENDING_FLAG_SLEEP;
	}else if(flag & SHTPS_PENDING_FLAG_SLEEP){
		SHTPS_LOG_DBG_PRINT("%s() pending flag(0x%04x) & 0x%04x = 0x%04x\n",
			__func__, ts->pending, ~SHTPS_PENDING_FLAG_WAKEUP, ts->pending & ~SHTPS_PENDING_FLAG_WAKEUP);
		ts->pending &= ~SHTPS_PENDING_FLAG_WAKEUP;
	}
	
	if(flag & SHTPS_PENDING_FLAG_STARTLOADER){
		SHTPS_LOG_DBG_PRINT("%s() pending flag(0x%04x) & 0x%04x = 0x%04x\n",
			__func__, ts->pending, ~SHTPS_PENDING_FLAG_STARTTESTMODE, ts->pending & ~SHTPS_PENDING_FLAG_STARTTESTMODE);
		shtps_loader_clr_ready(ts);
		ts->pending &= ~SHTPS_PENDING_FLAG_STARTTESTMODE;
	}else if(flag & SHTPS_PENDING_FLAG_STARTTESTMODE){
		SHTPS_LOG_DBG_PRINT("%s() pending flag(0x%04x) & 0x%04x = 0x%04x\n",
			__func__, ts->pending, ~SHTPS_PENDING_FLAG_STARTLOADER, ts->pending & ~SHTPS_PENDING_FLAG_STARTLOADER);
		shtps_testmode_clr_ready(ts);
		ts->pending &= ~SHTPS_PENDING_FLAG_STARTLOADER;
	}
	SHTPS_LOG_DBG_PRINT("%s() pending flag = 0x%04x\n", __func__, ts->pending);
}

static void shtps_clr_pending_flg(struct shtps_nas_spi *ts, int flag)
{
	SHTPS_LOG_DBG_PRINT("%s() pending flag(0x%04x) & 0x%04x = 0x%04x\n", 
		__func__, ts->pending, ~flag, ts->pending & ~flag);
	ts->pending &= ~flag;
}

/* -----------------------------------------------------------------------------------
 */
static int shtps_statef_nop(struct shtps_nas_spi *ts, int param)
{
	SHTPS_LOG_DBG_PRINT("%s()\n", __func__);
	return 0;
}

static int shtps_statef_cmn_stop(struct shtps_nas_spi *ts, int param)
{
	#if defined(SHTPS_CPU_CLOCK_CONTROL_ENABLE)
		ts->report_event = SHTPS_EVENT_TU;
		shtps_perf_lock_disable(ts);
		SHTPS_LOG_DBG_PRINT("perf_lock end by ForceTouchUp\n");
	#endif /* SHTPS_CPU_CLOCK_CONTROL_ENABLE */

	#if defined( SHTPS_POWER_MODE_CONTROL_ENABLE )
		if((ts->pending & SHTPS_PENDING_FLAG_SLEEP) != 0){
			SHTPS_LOG_DBG_PRINT("powermode_mutex_lock()\n");
			mutex_lock(&shtps_power_mode_ctrl_lock);
			ts->power_mode_state &= ~SHTPS_HPMODE_NON_CONTINUOUS_REQ_ON;
			SHTPS_LOG_DBG_PRINT("power mode state = 0x%02x\n", ts->power_mode_state);
			SHTPS_LOG_DBG_PRINT("powermode_mutex_unlock()\n");
			mutex_unlock(&shtps_power_mode_ctrl_lock);
		}
	#endif /* SHTPS_POWER_MODE_CONTROL_ENABLE */

	shtps_event_force_touchup(ts);
	shtps_clr_pending_flg(ts, 0xFFFF);
	shtps_irq_disable(ts);
	shtps_power_off(ts);
	state_change(ts, SHTPS_STATE_POWEROFF);
	return 0;
}

static int shtps_statef_cmn_sleep(struct shtps_nas_spi *ts, int param)
{
	shtps_set_pending_flg(ts, SHTPS_PENDING_FLAG_SLEEP);
	ts->is_lcd_on = 0;
	return 0;
}

static int shtps_statef_cmn_sleep_as_stop(struct shtps_nas_spi *ts, int param)
{
	shtps_set_pending_flg(ts, SHTPS_PENDING_FLAG_SLEEP);
	shtps_statef_cmn_stop(ts, param);
	ts->is_lcd_on = 0;
	return 0;
}

static int shtps_statef_cmn_wakeup(struct shtps_nas_spi *ts, int param)
{
	ts->is_lcd_on = 1;
	shtps_set_pending_flg(ts, SHTPS_PENDING_FLAG_WAKEUP);
	return 0;
}

static int shtps_statef_cmn_startldr(struct shtps_nas_spi *ts, int param)
{
	shtps_set_pending_flg(ts, SHTPS_PENDING_FLAG_STARTLOADER);
	return 0;
}

static int shtps_statef_cmn_stopldr(struct shtps_nas_spi *ts, int param)
{
	shtps_clr_pending_flg(ts, SHTPS_PENDING_FLAG_STARTLOADER);
	return 0;
}

static int shtps_statef_cmn_starttm(struct shtps_nas_spi *ts, int param)
{
	shtps_set_pending_flg(ts, SHTPS_PENDING_FLAG_STARTTESTMODE);
	return 0;
}

static int shtps_statef_cmn_stoptm(struct shtps_nas_spi *ts, int param)
{
	shtps_clr_pending_flg(ts, SHTPS_PENDING_FLAG_STARTTESTMODE);
	return 0;
}

static int shtps_statef_cmn_error(struct shtps_nas_spi *ts, int param)
{
	SHTPS_LOG_DBG_PRINT("%s()\n", __func__);
	return -1;
}

static int shtps_statef_cmn_reset_recovery(struct shtps_nas_spi *ts, int param)
{
	#if defined(SHTPS_RESET_RECOVERY_ENABLE)
		SHTPS_LOG_DBG_PRINT("%s(0x%X)\n", __func__, param);
		shtps_event_force_touchup(ts);
		shtps_command_cancel(ts);
		shtps_loader_cancel(ts);
		if( (param & SHTPS_PENDING_FLAG_STARTTESTMODE) == 0 ){
			shtps_testmode_cancel(ts);
		}
		shtps_clr_pending_flg(ts, 0xFFFF);
		shtps_set_pending_flg(ts, param);
		shtps_reset(ts);
		state_change(ts, SHTPS_STATE_RESETTING);
	#endif /* SHTPS_RESET_RECOVERY_ENABLE */

	return 0;
}

static int shtps_statef_cmn_reboot_recovery(struct shtps_nas_spi *ts, int param)
{
	#if defined(SHTPS_REBOOT_RECOVERY_ENABLE)
		SHTPS_LOG_DBG_PRINT("%s(0x%X)\n", __func__, param);
		shtps_event_force_touchup(ts);
		shtps_command_cancel(ts);
		shtps_loader_cancel(ts);
		if( (param & SHTPS_PENDING_FLAG_STARTTESTMODE) == 0 ){
			shtps_testmode_cancel(ts);
		}
		shtps_clr_pending_flg(ts, 0xFFFF);
		shtps_irq_disable(ts);
		shtps_power_off(ts);
		mdelay(10);
		shtps_set_pending_flg(ts, param);
		shtps_irq_enable(ts);
		shtps_power_on(ts);
		state_change(ts, SHTPS_STATE_RESETTING);
	#endif /* SHTPS_REBOOT_RECOVERY_ENABLE */

	return 0;
}

/* -----------------------------------------------------------------------------------
 */
static int shtps_statef_powoff_start(struct shtps_nas_spi *ts, int param)
{
	shtps_set_pending_flg(ts, SHTPS_PENDING_FLAG_WAKEUP);
	shtps_irq_enable(ts);
	shtps_power_on(ts);
	state_change(ts, SHTPS_STATE_RESETTING);
	return 0;
}

static int shtps_statef_powoff_wakeup_start(struct shtps_nas_spi *ts, int param)
{
	ts->is_lcd_on = 1;
	shtps_statef_powoff_start(ts, param);
	return 0;
}

static int shtps_statef_powoff_startldr(struct shtps_nas_spi *ts, int param)
{
	shtps_set_pending_flg(ts, SHTPS_PENDING_FLAG_STARTLOADER);
	shtps_irq_enable(ts);
	shtps_power_on(ts);
	state_change(ts, SHTPS_STATE_RESETTING);
	return 0;
}

static int shtps_statef_powoff_starttm(struct shtps_nas_spi *ts, int param)
{
	ts->testmode.ready = 0;
	shtps_set_pending_flg(ts, SHTPS_PENDING_FLAG_STARTTESTMODE | SHTPS_PENDING_FLAG_WAKEUP);
	shtps_irq_enable(ts);
	shtps_power_on(ts);
	state_change(ts, SHTPS_STATE_RESETTING);
	return 0;
}

/* -----------------------------------------------------------------------------------
 */
static int shtps_statef_resetting_enter(struct shtps_nas_spi *ts, int param)
{
	return 0;
}

static int shtps_statef_resetting_int(struct shtps_nas_spi *ts, int param)
{
	u8 intsrc = shtps_get_intsrc(ts, 0xFF);

	if(intsrc == SHTPS_NAS_INT_NONE){
		shtps_statef_cmn_reboot_recovery(ts, ts->pending);
	}else if((intsrc & SHTPS_NAS_INT_RESET) != 0){
		shtps_check_fw_startup(ts);
		if(ts->fw_startup_state == SHTPS_FW_STARTUP_PROGRAM){
			shtps_write_init_cmds(ts);
			if( (ts->pending & SHTPS_PENDING_FLAG_NO_ID_ATTACH) != 0){
				shtps_clr_pending_flg(ts, SHTPS_PENDING_FLAG_NO_ID_ATTACH);
				state_change(ts, SHTPS_STATE_ATTACH);
			}else{
				state_change(ts, SHTPS_STATE_POWERON);
			}
		}else if(ts->fw_startup_state == SHTPS_FW_STARTUP_LOADER){
			SHTPS_LOG_DBG_PRINT("fw startup loader state\n");
			shtps_set_pending_flg(ts, SHTPS_PENDING_FLAG_STARTLOADER);
			shtps_command_setprevstate(ts, SHTPS_STATE_RESETTING);
			shtps_command_setfunc(ts, shtps_write_getfwver_cmds);
			state_change(ts, SHTPS_STATE_CMD_EXECUTING);
		}else if(ts->fw_startup_state == SHTPS_FW_STARTUP_SLEEP){
			SHTPS_LOG_DBG_PRINT("fw startup sleep state\n");
			if( (ts->pending & SHTPS_PENDING_FLAG_STARTLOADER) != 0 ){
				msleep(SHTPS_FW_SLEEP_WAIT_MS);
				shtps_write_bootloader_cmds(ts);
				state_change(ts, SHTPS_STATE_WAIT_LOADER);
			}else{
				if(shtps_command_existing(ts)){
					msleep(SHTPS_FW_SLEEP_WAIT_MS);
					shtps_command_setprevstate(ts, SHTPS_STATE_POWEROFF);
					state_change(ts, SHTPS_STATE_CMD_EXECUTING);
				}else{
					shtps_statef_cmn_stop(ts, param);
				}
			}
		}else if(ts->fw_startup_state == SHTPS_FW_STARTUP_CRC_ERROR){
			SHTPS_LOG_DBG_PRINT("fw startup crc error\n");
			if( (ts->pending & SHTPS_PENDING_FLAG_STARTLOADER) != 0 ){
				shtps_write_bootloader_cmds(ts);
				state_change(ts, SHTPS_STATE_WAIT_LOADER);
			}else{
				if(shtps_command_existing(ts)){
					shtps_command_setprevstate(ts, SHTPS_STATE_POWEROFF);
					state_change(ts, SHTPS_STATE_CMD_EXECUTING);
				}else{
					shtps_statef_cmn_stop(ts, param);
				}
			}
		}else{
			SHTPS_LOG_ERR_PRINT("fw startup unknown state\n");
			shtps_statef_cmn_stop(ts, param);
		}
	}else{
		SHTPS_LOG_DBG_PRINT("%s() not support interrupt [%d]\n", __func__, intsrc);
	}

	return 0;
}

/* -----------------------------------------------------------------------------------
 */
static int shtps_statef_attach_int(struct shtps_nas_spi *ts, int param)
{
	u8 intsrc = shtps_get_intsrc(ts, 0xFF);

	if(intsrc == SHTPS_NAS_INT_NONE){
		shtps_statef_cmn_reboot_recovery(ts,
				(SHTPS_PENDING_FLAG_WAKEUP | SHTPS_PENDING_FLAG_NO_ID_ATTACH));
	}else if( (intsrc & SHTPS_NAS_INT_RESET) != 0 ){
		shtps_statef_cmn_reset_recovery(ts,
				(SHTPS_PENDING_FLAG_WAKEUP | SHTPS_PENDING_FLAG_NO_ID_ATTACH));
	}else if((intsrc & SHTPS_NAS_INT_MICOM) != 0){
		shtps_write_attach_cmds(ts);
		state_change(ts, SHTPS_STATE_POWERON);
	}else{
		SHTPS_LOG_DBG_PRINT("%s() not support interrupt [%d]\n", __func__, intsrc);
	}
	
	return 0;
}

/* -----------------------------------------------------------------------------------
 */
static int shtps_statef_powon_int(struct shtps_nas_spi *ts, int param)
{
	u8 intsrc = shtps_get_intsrc(ts, 0xFF);

	if(intsrc == SHTPS_NAS_INT_NONE){
		shtps_statef_cmn_reboot_recovery(ts, ts->pending);
	}else if( (intsrc & SHTPS_NAS_INT_RESET) != 0 ){
		shtps_statef_cmn_reset_recovery(ts, ts->pending);
	}else if((intsrc & SHTPS_NAS_INT_MICOM) != 0){
		if((ts->pending & SHTPS_PENDING_FLAG_STARTLOADER) != 0 ||
			(ts->pending & SHTPS_PENDING_FLAG_STARTTESTMODE) != 0 ||
			(ts->pending & SHTPS_PENDING_FLAG_SLEEP) != 0)
		{
			shtps_write_sleep_cmds(ts);
			state_change(ts, SHTPS_STATE_SLEEPING);
			
		}else if(ts->pending & SHTPS_PENDING_FLAG_WAKEUP){
			shtps_write_activate_cmds(ts);
			state_change(ts, SHTPS_STATE_ACTIVATING);
			
		}else{
			state_change(ts, SHTPS_STATE_SLEEP);
		}
	}else{
		SHTPS_LOG_DBG_PRINT("%s() not support interrupt [%d]\n", __func__, intsrc);
	}
	
	return 0;
}

/* -----------------------------------------------------------------------------------
 */
static int shtps_statef_sleeping_int(struct shtps_nas_spi *ts, int param)
{
	u8 intsrc = shtps_get_intsrc(ts, 0xFF);

	if(intsrc == SHTPS_NAS_INT_NONE){
		shtps_statef_cmn_reboot_recovery(ts, ts->pending);
	}else if( (intsrc & SHTPS_NAS_INT_RESET) != 0 ){
		shtps_statef_cmn_reset_recovery(ts, ts->pending);
	}else if((intsrc & SHTPS_NAS_INT_MICOM) != 0){
		if(shtps_command_read_result(ts) == 0){
			if(ts->command.result[1] == 0x28){
				SHTPS_LOG_DBG_PRINT("%s() system state E_PENDING\n", __func__);
			}else if(ts->command.result[1] != 0){
				SHTPS_LOG_ERR_PRINT("%s() fw state change error (%s)\n",
					__func__,
					(ts->command.result[1] == 0x2C)? "Invalid state" :
					(ts->command.result[1] == 0x2D)? "OpCode is invalid" :
					(ts->command.result[1] == 0x2E)? "Parameter error" : "Unknown error");
			}
		}

		state_change(ts, SHTPS_STATE_SLEEP);
	}else{
		SHTPS_LOG_DBG_PRINT("%s() not support interrupt [%d]\n", __func__, intsrc);
	}

	return 0;
}

/* -----------------------------------------------------------------------------------
 */
static int shtps_statef_sleep_enter(struct shtps_nas_spi *ts, int param)
{

	if(shtps_command_existing(ts)){
		shtps_command_setprevstate(ts, SHTPS_STATE_SLEEP);
		state_change(ts, SHTPS_STATE_CMD_EXECUTING);
		return 0;
	}
	
	if(ts->pending & SHTPS_PENDING_FLAG_STARTLOADER){
		shtps_write_bootloader_cmds(ts);
		state_change(ts, SHTPS_STATE_WAIT_LOADER);

	}else if(ts->pending & SHTPS_PENDING_FLAG_STARTTESTMODE){
		shtps_write_testmode_cmds(ts);
		state_change(ts, SHTPS_STATE_WAIT_TESTMODE);
		
	}else if(ts->pending & SHTPS_PENDING_FLAG_WAKEUP){
		shtps_write_activate_cmds(ts);
		state_change(ts, SHTPS_STATE_ACTIVATING);
		
	}else if((ts->pending & SHTPS_PENDING_FLAG_SLEEP) != 0){
		return shtps_statef_cmn_stop(ts, param);
	}
	
	return 0;
}

static int shtps_statef_sleep_wakeup(struct shtps_nas_spi *ts, int param)
{
	ts->is_lcd_on = 1;
	shtps_set_pending_flg(ts, SHTPS_PENDING_FLAG_WAKEUP);
	shtps_write_activate_cmds(ts);
	state_change(ts, SHTPS_STATE_ACTIVATING);
	return 0;
}

static int shtps_statef_sleep_startldr(struct shtps_nas_spi *ts, int param)
{
	shtps_set_pending_flg(ts, SHTPS_PENDING_FLAG_STARTLOADER);
	shtps_write_bootloader_cmds(ts);
	state_change(ts, SHTPS_STATE_WAIT_LOADER);
	return 0;
}

static int shtps_statef_sleep_starttm(struct shtps_nas_spi *ts, int param)
{
	shtps_set_pending_flg(ts, SHTPS_PENDING_FLAG_STARTTESTMODE);
	shtps_write_testmode_cmds(ts);
	state_change(ts, SHTPS_STATE_WAIT_TESTMODE);
	return 0;
}

static int shtps_statef_sleep_command(struct shtps_nas_spi *ts, int param)
{
	if(!shtps_command_existing(ts)){
		return -1;
	}
	
	shtps_command_setprevstate(ts, SHTPS_STATE_SLEEP);
	state_change(ts, SHTPS_STATE_CMD_EXECUTING);
	return 0;
}

static int shtps_statef_sleep_int(struct shtps_nas_spi *ts, int param)
{
	u8 intsrc = shtps_get_intsrc(ts, 0xFF);

	if(intsrc == SHTPS_NAS_INT_NONE){
		shtps_statef_cmn_reboot_recovery(ts, ts->pending);
	}else if( (intsrc & SHTPS_NAS_INT_RESET) != 0 ){
		shtps_statef_cmn_reset_recovery(ts, ts->pending);
	}else{
		SHTPS_LOG_DBG_PRINT("%s() not support interrupt [%d]\n", __func__, intsrc);
	}

	return 0;
}

/* -----------------------------------------------------------------------------------
 */
static int shtps_statef_activating_int(struct shtps_nas_spi *ts, int param)
{
	u8 intsrc = shtps_get_intsrc(ts, 0xFF);

	if(intsrc == SHTPS_NAS_INT_NONE){
		shtps_statef_cmn_reboot_recovery(ts, ts->pending);
	}else if( (intsrc & SHTPS_NAS_INT_RESET) != 0 ){
		shtps_statef_cmn_reset_recovery(ts, ts->pending);
	}else if((intsrc & SHTPS_NAS_INT_MICOM) != 0){
		if(shtps_command_read_result(ts) == 0){
			if(ts->command.result[1] == 0x28){
				SHTPS_LOG_DBG_PRINT("%s() system state E_PENDING\n", __func__);
			}else if(ts->command.result[1] != 0){
				SHTPS_LOG_ERR_PRINT("%s() fw state change error (%s)\n",
					__func__,
					(ts->command.result[1] == 0x2C)? "Invalid state" :
					(ts->command.result[1] == 0x2D)? "OpCode is invalid" :
					(ts->command.result[1] == 0x2E)? "Parameter error" : "Unknown error");
			}
		}
		
		if((ts->pending & SHTPS_PENDING_FLAG_SLEEP) != 0 ||
			(ts->pending & SHTPS_PENDING_FLAG_STARTLOADER) != 0 ||
			(ts->pending & SHTPS_PENDING_FLAG_STARTTESTMODE) != 0 )
		{
#if defined( SHTPS_TESTMODE_SLEEP_WAIT_ENABLE )
#if defined( SHTPS_TESTMODE_SLEEP_WAIT_VARIABLE )
			if ((ts->pending & SHTPS_PENDING_FLAG_STARTTESTMODE) != 0 && shtps_testmode_sleep_wait_ms > 0)
				mdelay(shtps_testmode_sleep_wait_ms);
#else
			if ((ts->pending & SHTPS_PENDING_FLAG_STARTTESTMODE) != 0)
				mdelay(SHTPS_TESTMODE_SLEEP_WAIT_MS);
#endif	/* #if defined( SHTPS_TESTMODE_SLEEP_WAIT_VARIABLE ) */
#endif /* #if defined( SHTPS_TESTMODE_SLEEP_WAIT_ENABLE ) */
			shtps_write_sleep_cmds(ts);
			state_change(ts, SHTPS_STATE_SLEEPING);
		}else{
			state_change(ts, SHTPS_STATE_ACTIVE);
			shtps_notify_startup(ts);
		}
	}else{
		SHTPS_LOG_DBG_PRINT("%s() not support interrupt [%d]\n", __func__, intsrc);
	}

	return 0;
}

/* -----------------------------------------------------------------------------------
 */
static int shtps_statef_active_enter(struct shtps_nas_spi *ts, int param)
{
	shtps_reg_write_single(ts, SHTPS_NAS_REG_BANK, 0x00);

	if(shtps_command_existing(ts)){
		if(ts->command.func == shtps_write_getfwver_cmds){
			shtps_write_sleep_cmds(ts);
			state_change(ts, SHTPS_STATE_SLEEPING);
		}else{
			shtps_command_setprevstate(ts, SHTPS_STATE_ACTIVE);
			state_change(ts, SHTPS_STATE_CMD_EXECUTING);
		}
	}else{
		if((ts->pending & SHTPS_PENDING_FLAG_STARTTESTMODE) != 0 ||
			(ts->pending & SHTPS_PENDING_FLAG_STARTLOADER) != 0)
		{
#if defined( SHTPS_TESTMODE_SLEEP_WAIT_ENABLE )
#if defined( SHTPS_TESTMODE_SLEEP_WAIT_VARIABLE )
			if ((ts->pending & SHTPS_PENDING_FLAG_STARTTESTMODE) != 0 && shtps_testmode_sleep_wait_ms > 0)
				mdelay(shtps_testmode_sleep_wait_ms);
#else
			if ((ts->pending & SHTPS_PENDING_FLAG_STARTTESTMODE) != 0)
				mdelay(SHTPS_TESTMODE_SLEEP_WAIT_MS);
#endif	/* #if defined( SHTPS_TESTMODE_SLEEP_WAIT_VARIABLE ) */
#endif /* #if defined( SHTPS_TESTMODE_SLEEP_WAIT_ENABLE ) */
			shtps_write_sleep_cmds(ts);
			state_change(ts, SHTPS_STATE_SLEEPING);

		}else if((ts->pending & SHTPS_PENDING_FLAG_SLEEP) != 0){
			return shtps_statef_cmn_stop(ts, param);
		}else{
			shtps_next_touch_report_req(ts);
		}
	}
	
	return 0;
}

static int shtps_statef_active_sleep(struct shtps_nas_spi *ts, int param)
{
	return shtps_statef_cmn_sleep_as_stop(ts, param);
}

static int shtps_statef_active_startldr(struct shtps_nas_spi *ts, int param)
{
	shtps_set_pending_flg(ts, SHTPS_PENDING_FLAG_STARTLOADER);
#if defined( SHTPS_TESTMODE_SLEEP_WAIT_ENABLE )
#if defined( SHTPS_TESTMODE_SLEEP_WAIT_VARIABLE )
	if ((ts->pending & SHTPS_PENDING_FLAG_STARTTESTMODE) != 0 && shtps_testmode_sleep_wait_ms > 0)
		mdelay(shtps_testmode_sleep_wait_ms);
#else
	if ((ts->pending & SHTPS_PENDING_FLAG_STARTTESTMODE) != 0)
		mdelay(SHTPS_TESTMODE_SLEEP_WAIT_MS);
#endif	/* #if defined( SHTPS_TESTMODE_SLEEP_WAIT_VARIABLE ) */
#endif /* #if defined( SHTPS_TESTMODE_SLEEP_WAIT_ENABLE ) */
	shtps_write_sleep_cmds(ts);
	state_change(ts, SHTPS_STATE_SLEEPING);
	return 0;
}

static int shtps_statef_active_starttm(struct shtps_nas_spi *ts, int param)
{
	shtps_set_pending_flg(ts, SHTPS_PENDING_FLAG_STARTTESTMODE);
	shtps_write_sleep_cmds(ts);
	state_change(ts, SHTPS_STATE_SLEEPING);
	return 0;
}

static int shtps_statef_active_command(struct shtps_nas_spi *ts, int param)
{
	if(!shtps_command_existing(ts)){
		return -1;
	}
	
	if(ts->command.func == shtps_write_getfwver_cmds){
		shtps_write_sleep_cmds(ts);
		state_change(ts, SHTPS_STATE_SLEEPING);
	}else{
		shtps_command_setprevstate(ts, SHTPS_STATE_ACTIVE);
		state_change(ts, SHTPS_STATE_CMD_EXECUTING);
	}

	return 0;
}

static int shtps_statef_active_int(struct shtps_nas_spi *ts, int param)
{
	u8 intsrc = shtps_check_intsrc(ts, 0xFF);

	if(intsrc == SHTPS_NAS_INT_NONE){
		shtps_statef_cmn_reboot_recovery(ts, SHTPS_PENDING_FLAG_WAKEUP);
	}else if( (intsrc & SHTPS_NAS_INT_RESET) != 0 ){
		shtps_clr_int(ts, intsrc);
		shtps_statef_cmn_reset_recovery(ts, SHTPS_PENDING_FLAG_WAKEUP);
	}else if((intsrc & SHTPS_NAS_INT_TOUCH) != 0){
		shtps_read_touchevent(ts, SHTPS_STATE_ACTIVE);
		intsrc |= shtps_check_intsrc(ts, 0xFF);
		shtps_clr_int(ts, intsrc);
		if( (intsrc & SHTPS_NAS_INT_RESET) != 0 ){
			shtps_statef_cmn_reset_recovery(ts, SHTPS_PENDING_FLAG_WAKEUP);
		}else if((intsrc & SHTPS_NAS_INT_TOUCH) != 0){
			shtps_next_touch_report_req(ts);
		}
	}else{
		SHTPS_LOG_DBG_PRINT("%s() not support interrupt [%d]\n", __func__, intsrc);
		if(intsrc != 0){
			shtps_clr_int(ts, intsrc);
		}
	}

	return 0;
}

/* -----------------------------------------------------------------------------------
 */
static int shtps_statef_wait_testmode_enter(struct shtps_nas_spi *ts, int param)
{
	#if defined(SHTPS_TESTMODE_NEW_METHOD_ENABLE)
		shtps_reg_write_single(ts, SHTPS_NAS_REG_BANK, 0x30);
		shtps_reg_write_single(ts, 0x30, 0x00);
		shtps_reg_write_single(ts, 0x31, 0x01);

		SHTPS_LOG_DBG_PRINT("force calibration execute\n");
		shtps_reg_write_single(ts, SHTPS_NAS_REG_BANK, 0x0C);
		shtps_reg_write_single(ts, 0x0D, 0x01);

		ts->testmode.clb_cnt = 0x01;
	#endif /* SHTPS_TESTMODE_NEW_METHOD_ENABLE */

	return 0;
}

static int shtps_statef_wait_testmode_int(struct shtps_nas_spi *ts, int param)
{
	u8 intsrc = shtps_get_intsrc(ts, 0xFF);

	if(intsrc == SHTPS_NAS_INT_NONE){
		shtps_statef_cmn_reboot_recovery(ts,
				(SHTPS_PENDING_FLAG_WAKEUP | SHTPS_PENDING_FLAG_STARTTESTMODE));
	}else if( (intsrc & SHTPS_NAS_INT_RESET) != 0 ){
		shtps_statef_cmn_reset_recovery(ts,
				(SHTPS_PENDING_FLAG_WAKEUP | SHTPS_PENDING_FLAG_STARTTESTMODE));
	}else if((intsrc & SHTPS_NAS_INT_DCMAP) != 0){
		if((ts->pending & SHTPS_PENDING_FLAG_STARTTESTMODE) != 0){
			#if defined(SHTPS_TESTMODE_NEW_METHOD_ENABLE)
				if(ts->testmode.clb_cnt < SHTPS_DCMAP_FORCE_CALIBRATION_CNT_MAX){
					mdelay(SHTPS_DCMAP_FORCE_CALIBRATION_WAIT_MS);
					SHTPS_LOG_DBG_PRINT("force calibration execute\n");
					shtps_reg_write_single(ts, 0x0D, 0x01);
					ts->testmode.clb_cnt++;
				}else{
					mdelay(SHTPS_DCMAP_FORCE_CALIBRATION_WAIT_MS);
					shtps_reg_write_single(ts, SHTPS_NAS_REG_BANK, 0x30);
					shtps_reg_write_single(ts, 0x31, 0x00);
					shtps_reg_write_single(ts, 0x30, 0x07);

					shtps_reg_write_single(ts, SHTPS_NAS_REG_BANK, 0x0C);
					shtps_reg_write_single(ts, 0x0D, 0x01);

					state_change(ts, SHTPS_STATE_TESTMODE);
				}
			#else
				shtps_testmode_ready(ts);
				state_change(ts, SHTPS_STATE_TESTMODE);
			#endif /* SHTPS_TESTMODE_NEW_METHOD_ENABLE */
		}else{
			shtps_testmode_cancel(ts);
			shtps_write_sleep_cmds(ts);
			state_change(ts, SHTPS_STATE_SLEEPING);
		}
	}else{
		SHTPS_LOG_DBG_PRINT("%s() not support interrupt [%d]\n", __func__, intsrc);
	}

	return 0;
}

/* -----------------------------------------------------------------------------------
 */
static int shtps_statef_testmode_enter(struct shtps_nas_spi *ts, int param)
{
	#if 0
		if(shtps_command_existing(ts)){
			shtps_command_setprevstate(ts, SHTPS_STATE_TESTMODE);
			state_change(ts, SHTPS_STATE_CMD_EXECUTING);
		}
	#endif

	return 0;
}

static int shtps_statef_testmode_stoptm(struct shtps_nas_spi *ts, int param)
{
	shtps_clr_pending_flg(ts, SHTPS_PENDING_FLAG_STARTTESTMODE);
	shtps_write_exit_testmode_cmds(ts);
	
	shtps_clr_int(ts, 0xFF);
	shtps_write_activate_cmds(ts);
	state_change(ts, SHTPS_STATE_ACTIVATING);
	return 0;
}

static int shtps_statef_testmode_command(struct shtps_nas_spi *ts, int param)
{
	#if 0
		if(!shtps_command_existing(ts)){
			return -1;
		}
		
		shtps_command_setprevstate(ts, SHTPS_STATE_TESTMODE);
		state_change(ts, SHTPS_STATE_CMD_EXECUTING);
	#endif

	return 0;
}

static int shtps_statef_testmode_int(struct shtps_nas_spi *ts, int param)
{
	u8 intsrc = shtps_get_intsrc(ts, 0xFF);

	if(intsrc == SHTPS_NAS_INT_NONE){
		;
	}else if( (intsrc & SHTPS_NAS_INT_RESET) != 0 ){
		;
	}else if((intsrc & SHTPS_NAS_INT_DCMAP) != 0){
		shtps_testmode_ready(ts);
	}else{
		SHTPS_LOG_DBG_PRINT("%s() not support interrupt [%d]\n", __func__, intsrc);
	}

	return 0;
}

/* -----------------------------------------------------------------------------------
 */
static int shtps_statef_wait_loader_int(struct shtps_nas_spi *ts, int param)
{
	u8 intsrc = shtps_get_intsrc(ts, 0xFF);

	if(intsrc == SHTPS_NAS_INT_NONE){
		shtps_statef_cmn_reboot_recovery(ts, SHTPS_PENDING_FLAG_WAKEUP);
	}else if( (intsrc & SHTPS_NAS_INT_RESET) != 0 ){
		shtps_statef_cmn_reset_recovery(ts, SHTPS_PENDING_FLAG_WAKEUP);
	}else if((intsrc & SHTPS_NAS_INT_MICOM) != 0){
		u8 result;
		if(shtps_get_loader_cmd_result(ts, &result) != 0){
			shtps_statef_loader_stopldr(ts, 0);
			return -1;
		}
		if(result != 0){
			SHTPS_LOG_ERR_PRINT("%s() enter bootloader error (%s)\n",
				__func__,
				(result == 0x2B)? "Magic number is invalid" :
				(result == 0x2C)? "Firmware status error (status != SLEEP)" :
				(result == 0x2D)? "OpCode is invalid" : "Unknown error");
			shtps_statef_loader_stopldr(ts, 0);
			return -1;
		}
		
		shtps_loader_ready(ts);
		state_change(ts, SHTPS_STATE_LOADER);
	}else{
		SHTPS_LOG_DBG_PRINT("%s() not support interrupt [%d]\n", __func__, intsrc);
	}

	return 0;
}

/* -----------------------------------------------------------------------------------
 */
static int shtps_statef_loader_stopldr(struct shtps_nas_spi *ts, int param)
{
	shtps_clr_pending_flg(ts, SHTPS_PENDING_FLAG_STARTLOADER);
	shtps_set_pending_flg(ts, SHTPS_PENDING_FLAG_WAKEUP);
	shtps_loader_cancel(ts);
	shtps_reset(ts);
	state_change(ts, SHTPS_STATE_RESETTING);
	return 0;
}

static int shtps_statef_loader_int(struct shtps_nas_spi *ts, int param)
{
	u8 intsrc = shtps_get_intsrc(ts, 0xFF);

	if(intsrc == SHTPS_NAS_INT_NONE){
		shtps_statef_cmn_reboot_recovery(ts, SHTPS_PENDING_FLAG_WAKEUP);
	}else if( (intsrc & SHTPS_NAS_INT_RESET) != 0 ){
		shtps_statef_cmn_reset_recovery(ts, SHTPS_PENDING_FLAG_WAKEUP);
	}else if((intsrc & SHTPS_NAS_INT_MICOM) != 0){
		shtps_loader_ready(ts);
	}else{
		SHTPS_LOG_DBG_PRINT("%s() not support interrupt [%d]\n", __func__, intsrc);
	}

	return 0;
}

/* -----------------------------------------------------------------------------------
 */
static int shtps_statef_command_enter(struct shtps_nas_spi *ts, int param)
{
	if(!ts->command.func || ts->command.func(ts) != 0){
		shtps_command_cancel(ts);
		state_change(ts, ts->command.prevstate);
		return -1;
	}
	return 0;
}

static int shtps_statef_command_stop(struct shtps_nas_spi *ts, int param)
{
	shtps_command_cancel(ts);
	return shtps_statef_cmn_stop(ts, param);
}

static int shtps_statef_command_sleep(struct shtps_nas_spi *ts, int param)
{
	shtps_command_cancel(ts);
	return shtps_statef_cmn_sleep_as_stop(ts, param);
}

static int shtps_statef_command_int(struct shtps_nas_spi *ts, int param)
{
	u8 intsrc = shtps_get_intsrc(ts, 0xFF);

	if(intsrc == SHTPS_NAS_INT_NONE){
		shtps_statef_cmn_reboot_recovery(ts, SHTPS_PENDING_FLAG_WAKEUP);
	}else if( (intsrc & SHTPS_NAS_INT_RESET) != 0 ){
		shtps_statef_cmn_reset_recovery(ts, SHTPS_PENDING_FLAG_WAKEUP);
	}else{
		if((intsrc & SHTPS_NAS_INT_MICOM) != 0){
			if(shtps_command_read_result(ts) != 0){
				shtps_command_cancel(ts);
			}else{
				shtps_command_ready(ts);
			}
			
			if((ts->pending & SHTPS_PENDING_FLAG_WAKEUP) != 0 &&
				ts->command.prevstate == SHTPS_STATE_SLEEP)
			{
				shtps_write_sleep_cmds(ts);
				state_change(ts, SHTPS_STATE_SLEEPING);
				
			}else if((ts->pending & SHTPS_PENDING_FLAG_SLEEP) != 0 &&
				ts->command.prevstate == SHTPS_STATE_ACTIVE)
			{
				shtps_write_activate_cmds(ts);
				state_change(ts, SHTPS_STATE_ACTIVATING);
				
			}else if((ts->pending & SHTPS_PENDING_FLAG_STARTLOADER) != 0 &&
				ts->command.prevstate == SHTPS_STATE_RESETTING)
			{
				ts->bt_ver = (ts->command.result[ 2])         | 
							 (ts->command.result[ 3] << 0x08) | 
							 (ts->command.result[ 4] << 0x10) | 
							 (ts->command.result[ 5] << 0x18);
				ts->bt_paramver= (ts->command.result[10])         | 
								 (ts->command.result[11] << 0x08) | 
								 (ts->command.result[12] << 0x10) | 
								 (ts->command.result[13] << 0x18);

				SHTPS_LOG_DBG_PRINT("BootLoaderVersion = %X.%X\n", ts->bt_ver, ts->bt_paramver);

				shtps_write_bootloader_cmds(ts);
				state_change(ts, SHTPS_STATE_WAIT_LOADER);

			}else if(ts->command.prevstate == SHTPS_STATE_POWEROFF){
				shtps_statef_cmn_stop(ts, param);
			}else{
				state_change(ts, ts->command.prevstate);
			}
		}
	}

	return 0;
}

/* -----------------------------------------------------------------------------------
 */
#if defined(SHTPS_DEBUG_CREATE_KOBJ_ENABLE)
static ssize_t store_spi_write(struct device *dev, struct device_attribute *attr,
             const char *buf, size_t count)
{
	struct shtps_nas_spi *ts = g_shtps_nas_spi;
    u8 data[1];
    int ret, reg, val;
    sscanf(buf,"%d %d", &reg, &val);
    
    data[0] = (u8)val;
    ret = shtps_reg_write(ts, (u8)reg, data, 1);

    printk("[shtps][test] spi write [0x%02x] <- 0x%02x. ret=%d\n", reg, data[0], ret);
    return count;
}

static DEVICE_ATTR(spi_write, S_IRUGO | S_IWUSR, NULL, store_spi_write);

static ssize_t store_spi_read(struct device *dev, struct device_attribute *attr,
             const char *buf, size_t count)
{
	struct shtps_nas_spi *ts = g_shtps_nas_spi;
    int i;
    
    u8 data[32];
    int ret, reg, size;
    sscanf(buf,"%d %d", &reg, &size);

    if(size > 32) size = 32;
    ret = shtps_reg_read(ts, (u8)reg, data, size);
    for(i = 0;i < size;i++){
        printk("[shtps][test] spi read [0x%02x] = 0x%02x. ret=%d\n", reg+i, data[i], ret);
    }
    return count;
}

static DEVICE_ATTR(spi_read, S_IRUGO | S_IWUSR, NULL, store_spi_read);

static ssize_t store_tp_reset(struct device *dev, struct device_attribute *attr,
             const char *buf, size_t count)
{
	struct shtps_nas_spi *ts = g_shtps_nas_spi;
    int val;
    sscanf(buf,"%d", &val);

    if(val){
        gpio_set_value(ts->rst_pin, 1);
    }else{
        gpio_set_value(ts->rst_pin, 0);
    }

    return count;
}

static DEVICE_ATTR(tp_reset, S_IRUGO | S_IWUSR, NULL, store_tp_reset);

static ssize_t store_tp_power(struct device *dev, struct device_attribute *attr,
             const char *buf, size_t count)
{
	struct shtps_nas_spi *ts = g_shtps_nas_spi;
    int val;
    sscanf(buf,"%d", &val);

    if(val){
        gpio_direction_output(ts->pow_pin, 1);
		if(ts->v17_power){
			ts->v17_power(1);
		}
    }else{
		if(ts->v17_power){
			ts->v17_power(0);
		}
        gpio_direction_output(ts->pow_pin, 0);
    }

    return count;
}

static DEVICE_ATTR(tp_power, S_IRUGO | S_IWUSR, NULL, store_tp_power);

#if defined(SHTPS_CPU_CLOCK_CONTROL_ENABLE)
static ssize_t show_perflock_timeout(struct device *dev, struct device_attribute *attr,
			char *buf)
{
	struct shtps_nas_spi *ts = g_shtps_nas_spi;
	
	return snprintf(buf, 0x40,
			"PerfLock Timeout: %d [ms]\n", ts->perf_lock_enable_time_ms);
}

static ssize_t store_perflock_timeout(struct device *dev, struct device_attribute *attr,
			const char *buf, size_t count)
{
	struct shtps_nas_spi *ts = g_shtps_nas_spi;
	int value = 0;

	sscanf(buf,"%d", &value);

	if(value > 0){
		ts->perf_lock_enable_time_ms = value;
		SHTPS_LOG_DBG_PRINT("PerfLock Timeout set value %d [ms]", ts->perf_lock_enable_time_ms);
	}else{
		SHTPS_LOG_ERR_PRINT("PerfLock Timeout set value error [%d]", value);
	}

    return count;
}

static DEVICE_ATTR(perflock_timeout, S_IRUSR | S_IWUSR, show_perflock_timeout, store_perflock_timeout);

static ssize_t show_perflock_level(struct device *dev, struct device_attribute *attr,
			char *buf)
{
	struct shtps_nas_spi *ts = g_shtps_nas_spi;

	return snprintf(buf, 0x40,
			"PerfLock Level: %s\n",
			ts->perf_lock_level == SHTPS_PERF_LOCK_LEVEL_HIGHEST ? "highest" :
			ts->perf_lock_level == SHTPS_PERF_LOCK_LEVEL_HIGH ? "high" : "low");
}

static ssize_t store_perflock_level(struct device *dev, struct device_attribute *attr,
			const char *buf, size_t count)
{
	struct shtps_nas_spi *ts = g_shtps_nas_spi;
	int value = 0;

	sscanf(buf,"%d", &value);

	if( (SHTPS_PERF_LOCK_LEVEL_HIGHEST <= value) && (value <= SHTPS_PERF_LOCK_LEVEL_LOW) ){
		ts->perf_lock_level = value;
		SHTPS_LOG_DBG_PRINT("PerfLock Level set value [%d]", ts->perf_lock_level);
	}else{
		SHTPS_LOG_ERR_PRINT("PerfLock Level set value error [%d]", value);
	}

    return count;
}

static DEVICE_ATTR(perflock_level, S_IRUSR | S_IWUSR, show_perflock_level, store_perflock_level);
#endif /* SHTPS_CPU_CLOCK_CONTROL_ENABLE */

static struct attribute *dev_attrs[] = {
    &dev_attr_spi_write.attr,
    &dev_attr_spi_read.attr,
    &dev_attr_tp_reset.attr,
    &dev_attr_tp_power.attr,
	#if defined(SHTPS_CPU_CLOCK_CONTROL_ENABLE)
	    &dev_attr_perflock_timeout.attr,
	    &dev_attr_perflock_level.attr,
	#endif /* SHTPS_CPU_CLOCK_CONTROL_ENABLE */
    NULL
};

static struct attribute_group dev_attr_grp = {
    .attrs = dev_attrs,
};
#endif /* SHTPS_DEBUG_CREATE_KOBJ_ENABLE */

/* -----------------------------------------------------------------------------------
 */
#if defined( SHTPS_POWER_MODE_CONTROL_ENABLE )
static void shtps_set_power_mode(struct shtps_nas_spi *ts, int type, int on)
{
	int change = 0;

	if(on){
		if((ts->power_mode_state & type) == 0){
			SHTPS_LOG_DBG_PRINT("%s(0x%02x) ON\n", 
				(type == SHTPS_LPMODE_NON_CONTINUOUS_REQ_ON)? "NON CONTINUOUS LPMODE" :
				(type == SHTPS_LPMODE_CONTINUOUS_REQ_ON)? "CONTINUOUS LPMODE" :
				(type == SHTPS_HPMODE_NON_CONTINUOUS_REQ_ON)? "NON CONTINUOUS HPMODE" :
				(type == SHTPS_HPMODE_CONTINUOUS_REQ_ON)? "CONTINUOUS HPMODE" : "UNKNOWN ERROR", type);

			ts->power_mode_state |= type;
			change = 1;
		}
	}else{
		if((ts->power_mode_state & type) != 0){
			SHTPS_LOG_DBG_PRINT("%s(0x%02x) OFF\n", 
				(type == SHTPS_LPMODE_NON_CONTINUOUS_REQ_ON)? "NON CONTINUOUS LPMODE" :
				(type == SHTPS_LPMODE_CONTINUOUS_REQ_ON)? "CONTINUOUS LPMODE" :
				(type == SHTPS_HPMODE_NON_CONTINUOUS_REQ_ON)? "NON CONTINUOUS HPMODE" :
				(type == SHTPS_HPMODE_CONTINUOUS_REQ_ON)? "CONTINUOUS HPMODE" : "UNKNOWN ERROR", type);

			ts->power_mode_state &= ~type;
			change = 1;
		}
	}

	if( (ts->state_mgr.state == SHTPS_STATE_ACTIVE) && (change != 0) ){
		#if 1
		{
			int ret = 0;

			if( (ts->power_mode_state & SHTPS_LPMODE_ON) != 0 ){
				ret = shtps_command_setfunc(ts, shtps_write_low_power_cmds);
			}else if( (ts->power_mode_state & SHTPS_HPMODE_ON) != 0 ){
				ret = shtps_command_setfunc(ts, shtps_write_high_power_cmds);
			}else{
				ret = shtps_command_setfunc(ts, shtps_write_normal_power_cmds);
			}

			if(ret == 0){
				ret = request_event(ts, SHTPS_EVENT_CMDREQUEST, 0);
				if(ret){
					SHTPS_LOG_ERR_PRINT("%s() set system state command req error\n", __func__);
				}
				ret = shtps_wait_command_result(ts);
				if(ret){
					SHTPS_LOG_ERR_PRINT("%s() set system state command result error\n", __func__);
				}else{
					if(ts->command.result[1] == 0x28){
						SHTPS_LOG_DBG_PRINT("%s() system state E_PENDING\n", __func__);
					}else if(ts->command.result[1] != 0){
						SHTPS_LOG_ERR_PRINT("%s() set system state error (%s)\n",
							__func__,
							(ts->command.result[1] == 0x2C)? "Invalid state" :
							(ts->command.result[1] == 0x2D)? "OpCode is invalid" :
							(ts->command.result[1] == 0x2E)? "Parameter error" : "Unknown error");
					}
				}
			}else{
				SHTPS_LOG_ERR_PRINT("%s() set system state command func set error\n", __func__);
			}

			#if 0
				if(ret == 0){
					mdelay(10);
					ret = shtps_command_setfunc(ts, shtps_get_system_state_cmds);
					if(ret == 0){
						ret = request_event(ts, SHTPS_EVENT_CMDREQUEST, 0);
						if(ret){
							SHTPS_LOG_ERR_PRINT("%s() get system state command req error\n", __func__);
						}
						ret = shtps_wait_command_result(ts);
						if(ret){
							SHTPS_LOG_ERR_PRINT("%s() get system state command result error\n", __func__);
						}else{
							if(ts->command.result[1] != 0){
								SHTPS_LOG_ERR_PRINT("%s() get system state error (%s)\n",
									__func__,
									(ts->command.result[1] == 0x2D)? "OpCode is invalid" :
									(ts->command.result[1] == 0x2E)? "Parameter error" : "Unknown error");
							}else{
								SHTPS_LOG_DBG_PRINT("%s() system state [state:0x%X][mode:0x%X][pend:0x%X]\n",
										__func__, ts->command.result[2], ts->command.result[3], ts->command.result[4]);
							}
						}
					}else{
						SHTPS_LOG_ERR_PRINT("%s() get system state command func set error\n", __func__);
					}
				}
			#endif
		}
		#else
			shtps_statef_cmn_reset_recovery(ts, 0);
		#endif
	}

	SHTPS_LOG_DBG_PRINT("power mode request recieved. state = 0x%02x\n", ts->power_mode_state);
}
#endif /* #if defined( SHTPS_POWER_MODE_CONTROL_ENABLE ) */

/* -----------------------------------------------------------------------------------
 */
static int shtps_ioctl_enable(struct shtps_nas_spi *ts, unsigned long arg)
{
	int ret;

	#if defined( SHTPS_ASYNC_OPEN_ENABLE )
		ret = shtps_func_request_sync(ts, SHTPS_FUNC_REQ_EVEMT_ENABLE);
	#else
		ret = request_event(ts, SHTPS_EVENT_START, 0);
		shtps_wait_startup(ts);
	#endif /* #if defined( SHTPS_ASYNC_OPEN_ENABLE ) */

	return ret;
}

static int shtps_ioctl_enable_no_id(struct shtps_nas_spi *ts, unsigned long arg)
{
	int ret;
	request_event(ts, SHTPS_EVENT_STOP, 0);
	msleep(SHTPS_POWEROFF_WAIT_MS);

	mutex_lock(&shtps_ctrl_lock);
	shtps_set_pending_flg(ts, SHTPS_PENDING_FLAG_NO_ID_ATTACH);
	mutex_unlock(&shtps_ctrl_lock);

	ret = request_event(ts, SHTPS_EVENT_START, 0);
	shtps_wait_startup(ts);
	return ret;
}

static int shtps_ioctl_disable(struct shtps_nas_spi *ts, unsigned long arg)
{
	int ret;

	#if defined( SHTPS_ASYNC_OPEN_ENABLE )
		ret = shtps_func_request_sync(ts, SHTPS_FUNC_REQ_EVEMT_DISABLE);
	#else
		ret = request_event(ts, SHTPS_EVENT_STOP, 0);
	#endif /* #if defined( SHTPS_ASYNC_OPEN_ENABLE ) */

	return ret;
}

static int shtps_ioctl_reset(struct shtps_nas_spi *ts, unsigned long arg)
{
	shtps_reset(ts);

	return 0;
}

static int shtps_ioctl_getver(struct shtps_nas_spi *ts, unsigned long arg)
{
	int ret = 0;
	u8  buf[8];
	u8	turnback = 0;
	
	if (ts->state_mgr.state == SHTPS_STATE_POWEROFF) {
		turnback = 1;
	}
	
	ret = shtps_command_setfunc(ts, shtps_write_getfwver_cmds);
	if(ret){
		return -EBUSY;
	}
	
	ret = request_event(ts, SHTPS_EVENT_CMDREQUEST, 0);
	if(ret){
		return -EFAULT;
	}
	
	ret = shtps_wait_command_result(ts);
	if(ret){
		return -EFAULT;
	}
	
	SHTPS_LOG_DBG_PRINT("%s() opcode=0x%02x, result=0x%02x, fwver=0x%08x, hwver=0x%08x, paramver=0x%08x\n",
		 __func__, ts->command.result[0], ts->command.result[1],
		 ts->command.result[ 2] | (ts->command.result[ 3] << 0x08) | (ts->command.result[ 4] << 0x10) | (ts->command.result[ 5] << 0x18),
		 ts->command.result[ 6] | (ts->command.result[ 7] << 0x08) | (ts->command.result[ 8] << 0x10) | (ts->command.result[ 9] << 0x18),
		 ts->command.result[10] | (ts->command.result[11] << 0x08) | (ts->command.result[12] << 0x10) | (ts->command.result[13] << 0x18));

	if (turnback != 0) {
		request_event(ts, SHTPS_EVENT_STOP, 0);
	}
	
	memcpy(&buf[0], (u8*)&ts->command.result[ 2], 4);
	memcpy(&buf[4], (u8*)&ts->command.result[10], 4);
	if(copy_to_user((u8*)arg, buf, 8)){
		SHTPS_LOG_ERR_PRINT("[%s] error - copy_to_user()\n", __func__);
		return -EFAULT;
	}
	
	return 0;
}

static int shtps_ioctl_bl_enter(struct shtps_nas_spi *ts, unsigned long arg)
{
	int ret = request_event(ts, SHTPS_EVENT_STARTLOADER, 0);
	if(ret == 0){
		ret = shtps_wait_loader_ready(ts);
	}
	return ret;
}

static int shtps_ioctl_bl_erase(struct shtps_nas_spi *ts, unsigned long arg)
{
	if(ts->state_mgr.state != SHTPS_STATE_LOADER){
		return -EFAULT;
	}
	return shtps_loader_erase_sector(ts);
}

static int shtps_ioctl_bl_writeimage(struct shtps_nas_spi *ts, unsigned long arg)
{
	u8 buf[SHTPS_NASSAU_LOADER_BLOCK_SIZE * SHTPS_NASSAU_LOADER_PAGE_SIZE + 2];
	struct shtps_ioctl_param param;
	int size;

	if(ts->state_mgr.state != SHTPS_STATE_LOADER){
		return -EFAULT;
	}

	if(0 == arg || 0 != copy_from_user(&param, (void __user *)arg, sizeof(param))){
		return -EINVAL;
	}

	if(param.size > sizeof(buf)){
		return -EINVAL;
	}
	
	if(0 != copy_from_user(buf, param.data, param.size)){
		return -EINVAL;
	}

	if(param.size == 2){
		u16 offset = buf[0] | buf[1] << 0x08;
		size = SHTPS_NASSAU_LOADER_BLOCK_SIZE * SHTPS_NASSAU_LOADER_PAGE_SIZE;

		if(ts->bt_select == SHTPS_BOOTLOADER_OLD){
			if(offset > sizeof(tps_fw_data_old)){
				return -EINVAL;
			}
			memcpy(&buf[2], &tps_fw_data_old[offset], size);
		}else{
			if(offset > sizeof(tps_fw_data)){
				return -EINVAL;
			}
			memcpy(&buf[2], &tps_fw_data[offset], size);
		}
	}else{
		size = param.size - 2;
	}
	
	return shtps_loader_write_image(ts, &buf[2], buf[0] | (buf[1] << 0x08), size);
}

static int shtps_ioctl_bl_erase_loader(struct shtps_nas_spi *ts, unsigned long arg)
{
	if(ts->state_mgr.state != SHTPS_STATE_LOADER){
		return -EFAULT;
	}
	return shtps_boot_erase_sector(ts);
}

static int shtps_ioctl_bl_writeimage_loader(struct shtps_nas_spi *ts, unsigned long arg)
{
	u8 buf[SHTPS_NASSAU_LOADER_BLOCK_SIZE * SHTPS_NASSAU_LOADER_PAGE_SIZE + 2];
	struct shtps_ioctl_param param;
	int size;

	if(ts->state_mgr.state != SHTPS_STATE_LOADER){
		return -EFAULT;
	}

	if(0 == arg || 0 != copy_from_user(&param, (void __user *)arg, sizeof(param))){
		return -EINVAL;
	}

	if(param.size > sizeof(buf)){
		return -EINVAL;
	}
	
	if(0 != copy_from_user(buf, param.data, param.size)){
		return -EINVAL;
	}

	if(param.size == 2){
		u16 offset = buf[0] | buf[1] << 0x08;
		size = SHTPS_NASSAU_LOADER_BLOCK_SIZE * SHTPS_NASSAU_LOADER_PAGE_SIZE;

		if(ts->bt_select == SHTPS_BOOTLOADER_OLD){
			if(offset > sizeof(tps_ldr_data_old)){
				return -EINVAL;
			}
			memcpy(&buf[2], &tps_ldr_data_old[offset], size);
		}else{
			if(offset > sizeof(tps_ldr_data)){
				return -EINVAL;
			}
			memcpy(&buf[2], &tps_ldr_data[offset], size);
		}
	}else{
		size = param.size - 2;
	}
	
	return shtps_boot_write_image(ts, &buf[2], buf[0] | (buf[1] << 0x08), size);
}

static int shtps_ioctl_bl_exit(struct shtps_nas_spi *ts, unsigned long arg)
{
	if(ts->state_mgr.state != SHTPS_STATE_LOADER){
		return -EFAULT;
	}
	
	return request_event(ts, SHTPS_EVENT_STOPLOADER, 0);
}

static int shtps_ioctl_get_touchinfo(struct shtps_nas_spi *ts, unsigned long arg)
{
	int i;
	struct shtps_touch_info info;

	memset(&info, 0, sizeof(info));
	
	mutex_lock(&shtps_ctrl_lock);
	for(i = 0;i < SHTPS_FINGER_MAX;i++){
		if(ts->report_info.fingers[i].id != SHTPS_FINGER_ID_INVALID){
			memcpy(&info.fingers[i], &ts->report_info.fingers[i], sizeof(info.fingers[i]));
			info.fingers[i].x = SHTPS_POS_SCALE_CONVERT_X(info.fingers[i].x);
			info.fingers[i].y = SHTPS_POS_SCALE_CONVERT_Y(info.fingers[i].y);
		}
	}
	mutex_unlock(&shtps_ctrl_lock);
	
	if(copy_to_user((u8*)arg, (u8*)&info, sizeof(info))){
		return -EFAULT;
	}
	
	return 0;
}

static int shtps_ioctl_get_touchinfo_untrans(struct shtps_nas_spi *ts, unsigned long arg)
{
	int i;
	struct shtps_touch_info info;

	memset(&info, 0, sizeof(info));

	mutex_lock(&shtps_ctrl_lock);
	for(i = 0;i < SHTPS_FINGER_MAX;i++){
		if(ts->report_info.fingers[i].id != SHTPS_FINGER_ID_INVALID){
			memcpy(&info.fingers[i], &ts->report_info.fingers[i], sizeof(info.fingers[i]));
		}
	}
	mutex_unlock(&shtps_ctrl_lock);
	
	if(copy_to_user((u8*)arg, (u8*)&info, sizeof(info))){
		return -EFAULT;
	}
	
	return 0;
}

static int shtps_ioctl_get_touchinfo_blk(struct shtps_nas_spi *ts, unsigned long arg)
{
	int i;
	struct shtps_touch_info info;

	memset(&info, 0, sizeof(info));

	if(shtps_wait_diag_touchevent(ts) != 0){
		return -1;
	}

	mutex_lock(&shtps_ctrl_lock);
	for(i = 0;i < SHTPS_FINGER_MAX;i++){
		if(ts->report_info.fingers[i].id != SHTPS_FINGER_ID_INVALID){
			memcpy(&info.fingers[i], &ts->report_info.fingers[i], sizeof(info.fingers[i]));
			info.fingers[i].x = SHTPS_POS_SCALE_CONVERT_X(info.fingers[i].x);
			info.fingers[i].y = SHTPS_POS_SCALE_CONVERT_Y(info.fingers[i].y);
		}
	}
	mutex_unlock(&shtps_ctrl_lock);

	if(copy_to_user((u8*)arg, (u8*)&info, sizeof(info))){
		return -EFAULT;
	}
	
	return 0;
}

static int shtps_ioctl_get_touchinfo_untrans_blk(struct shtps_nas_spi *ts, unsigned long arg)
{
	int i;
	struct shtps_touch_info info;

	memset(&info, 0, sizeof(info));

	if(shtps_wait_diag_touchevent(ts) != 0){
		return -1;
	}

	mutex_lock(&shtps_ctrl_lock);
	for(i = 0;i < SHTPS_FINGER_MAX;i++){
		if(ts->report_info.fingers[i].id != SHTPS_FINGER_ID_INVALID){
			memcpy(&info.fingers[i], &ts->report_info.fingers[i], sizeof(info.fingers[i]));
		}
	}	
	mutex_unlock(&shtps_ctrl_lock);
	
	if(copy_to_user((u8*)arg, (u8*)&info, sizeof(info))){
		return -EFAULT;
	}
	
	return 0;
}

static int shtps_ioctl_get_touchinfo_cancel(struct shtps_nas_spi *ts, unsigned long arg)
{
	shtps_cancel_diag_touchevent(ts);
	return 0;
}

static int shtps_ioctl_reg_read(struct shtps_nas_spi *ts, unsigned long arg)
{
	int ret;
	u8 buf;
	u8 addr;
	struct shtps_ioctl_param param;

	if(0 == arg || 0 != copy_from_user(&param, (void __user *)arg, sizeof(param))){
		return -EINVAL;
	}

	if(0 != copy_from_user(&addr, param.data, 1)){
		return -EINVAL;
	}
	
	mutex_lock(&shtps_ctrl_lock);
	ret = shtps_reg_read(ts, addr, &buf, 1);
	mutex_unlock(&shtps_ctrl_lock);
	if(ret){
		return -EFAULT;
	}
	if(copy_to_user(((struct shtps_ioctl_param*)arg)->data, (u8*)&buf, 1)){
		return -EFAULT;
	}
	
	return 0;
}

static int shtps_ioctl_reg_allread(struct shtps_nas_spi *ts, unsigned long arg)
{
	return 0;
}

static int shtps_ioctl_reg_write(struct shtps_nas_spi *ts, unsigned long arg)
{
	int ret;
	u8 data[2];
	struct shtps_ioctl_param param;

	if(0 == arg || 0 != copy_from_user(&param, (void __user *)arg, sizeof(param))){
		return -EINVAL;
	}

	if(0 != copy_from_user(data, param.data, 2)){
		return -EINVAL;
	}
	mutex_lock(&shtps_ctrl_lock);
	ret = shtps_reg_write_single(ts, data[0], data[1]);
	mutex_unlock(&shtps_ctrl_lock);
	if(ret){
		return -EFAULT;
	}

	return 0;
}

static int shtps_ioctl_tm_enter(struct shtps_nas_spi *ts, unsigned long arg)
{
	u8 buf[3];
	int ret;

	if(0 == arg || 0 != copy_from_user(buf, (void __user *)arg, 3)){
		return -EINVAL;
	}	
	ts->testmode.opt = buf[0];
	SHTPS_LOG_DBG_PRINT("%s() : testmode opt=0x%02x\n", __func__, ts->testmode.opt);
	
	if(request_event(ts, SHTPS_EVENT_STARTTM, 0)){
		return -EFAULT;
	}
	
	ret = wait_event_interruptible_timeout(
			ts->testmode.wait, 
			(ts->testmode.ready || ts->testmode.cancel), 
			msecs_to_jiffies(SHTPS_NASSAU_TMENTER_TIMEOUT_MS));	
	
	if(ret <= 0 || ts->testmode.cancel != 0){
		SHTPS_LOG_ERR_PRINT("[%s] enter testmode error. ret=%d, cancel=%d\n", __func__, ret, ts->testmode.cancel);
		return -EFAULT;
	}
	
	mutex_lock(&shtps_ctrl_lock);
	ret = shtps_reg_write_single(ts, SHTPS_NAS_REG_BANK, SHTPS_NAS_BANK_DC2);
	if(!ret){
		ret = shtps_reg_read(ts, SHTPS_NAS_REG_CMD, buf, 3);
	}
	mutex_unlock(&shtps_ctrl_lock);
	
	if(ret){
		return -EFAULT;
	}
	
	if(copy_to_user((u8*)arg, (u8*)buf, 3)){
		return -EFAULT;
	}
	
	return 0;
}

static int shtps_ioctl_tm_exit(struct shtps_nas_spi *ts, unsigned long arg)
{
	if(request_event(ts, SHTPS_EVENT_STOPTM, 0)){
		return -EFAULT;
	}
	shtps_testmode_cancel(ts);
	return 0;
}

static int shtps_ioctl_tm_getdata(struct shtps_nas_spi *ts, unsigned long arg)
{
	u8 *buf;
	struct shtps_ioctl_param param;

	if(0 == arg || 0 != copy_from_user(&param, (void __user *)arg, sizeof(param))){
		return -EINVAL;
	}

	buf = kzalloc(param.size, GFP_KERNEL);
	if(!buf){
		return -ENOMEM;
	}
	
	if(shtps_testmode_getdata(ts, buf, param.size) != 0){
		goto err_exit;
	}
	
	if(copy_to_user(((struct shtps_ioctl_param*)arg)->data, buf, param.size)){
		goto err_exit;
	}
	
	kfree(buf);
	return 0;

err_exit:
	kfree(buf);
	return -EFAULT;

}

static int shtps_ioctl_tm_getdata_num(struct shtps_nas_spi *ts, unsigned long arg)
{
	int ret = -EFAULT;
	u8 buf[2];
	
	mutex_lock(&shtps_ctrl_lock);

	if(!arg){
		goto err_exit;
	}

	if ((ret = shtps_reg_write_single(ts, SHTPS_NAS_REG_BANK, 0x02)) != 0) {
		goto err_exit;
	}
	if ((ret = shtps_reg_read(ts, SHTPS_NAS_REG_CMD + 1, buf, 2)) != 0) {
		goto err_exit;
	}
	
	if(copy_to_user((u8*)arg, buf, 2)){
		goto err_exit;
	}
	
	ret = 0;

err_exit:
	mutex_unlock(&shtps_ctrl_lock);
	return ret;

}

static int shtps_ioctl_calibration_param(struct shtps_nas_spi *ts, unsigned long arg)
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
		kfree(data);
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

static int shtps_ioctl_calibration_pen_param(struct shtps_nas_spi *ts, unsigned long arg)
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
		kfree(data);
		return -EINVAL;
	}
	memcpy(ts->offset_pen.base, data, sizeof(u16) * 5);
	ts->offset_pen.diff[0] = (signed short)(data[11] << 0x08 | data[10]);
	ts->offset_pen.diff[1] = (signed short)(data[13] << 0x08 | data[12]);
	ts->offset_pen.diff[2] = (signed short)(data[15] << 0x08 | data[14]);
	ts->offset_pen.diff[3] = (signed short)(data[17] << 0x08 | data[16]);
	ts->offset_pen.diff[4] = (signed short)(data[19] << 0x08 | data[18]);
	ts->offset_pen.diff[5] = (signed short)(data[21] << 0x08 | data[20]);
	ts->offset_pen.diff[6] = (signed short)(data[23] << 0x08 | data[22]);
	ts->offset_pen.diff[7] = (signed short)(data[25] << 0x08 | data[24]);
	ts->offset_pen.diff[8] = (signed short)(data[27] << 0x08 | data[26]);
	ts->offset_pen.diff[9] = (signed short)(data[29] << 0x08 | data[28]);
	ts->offset_pen.diff[10]= (signed short)(data[31] << 0x08 | data[30]);
	ts->offset_pen.diff[11]= (signed short)(data[33] << 0x08 | data[32]);
	kfree(data);

	if(ts->offset_pen.base[0] == 0){
		ts->offset_pen.enabled = 0;
	}else{
		ts->offset_pen.enabled = 1;
	}

	return 0;
}

static int shtps_ioctl_debug_reqevent(struct shtps_nas_spi *ts, unsigned long arg)
{
#if defined( SHTPS_DEVELOP_MODE_ENABLE )
	if(arg & 0x8000){
		request_event(ts, (int)((arg >> 0x08) & 0x7F), 0);
		request_event(ts, (int)(arg & 0xFF), 0);
	}else{
		request_event(ts, (int)arg, 0);
	}
#endif /* #if defined( SHTPS_DEVELOP_MODE_ENABLE ) */
	return 0;
}

static int shtps_ioctl_getver_builtin(struct shtps_nas_spi *ts, unsigned long arg)
{
	u32 fwver    = SHTPS_FWVER_NEWER;
	u32 paramver = SHTPS_PARAMVER_NEWER;
	u8  buf[8];

	if(0 == arg){
		SHTPS_LOG_ERR_PRINT("[%s] error - arg == 0\n", __func__);
		return -EINVAL;
	}

	buf[0] = fwver & 0xFF;
	buf[1] = (fwver >> 0x08) & 0xFF;
	buf[2] = (fwver >> 0x10) & 0xFF;
	buf[3] = (fwver >> 0x18) & 0xFF;
	buf[4] = paramver & 0xFF;
	buf[5] = (paramver >> 0x08) & 0xFF;
	buf[6] = (paramver >> 0x10) & 0xFF;
	buf[7] = (paramver >> 0x18) & 0xFF;

	if(copy_to_user((u8*)arg, buf, 8)){
		SHTPS_LOG_ERR_PRINT("[%s] error - copy_to_user()\n", __func__);
		return -EFAULT;
	}

	SHTPS_LOG_DBG_PRINT("[%s] built-in fw version = 0x%08x/param version = 0x%08x\n", 
		__func__, fwver, paramver);

	return 0;
}

static int shtps_ioctl_fw_update_continue_req(struct shtps_nas_spi *ts, unsigned long arg)
{
	ts->fw_update_continue = (u8)arg;

	return 0;
}

static int shtps_ioctl_bootloader_select(struct shtps_nas_spi *ts, unsigned long arg)
{
	u8 select;

	select = (u8)arg;

	if(select == 0){
		ts->bt_select = SHTPS_BOOTLOADER_OLD;
	}else{
		ts->bt_select = SHTPS_BOOTLOADER_NEW;
	}

	return 0;
}

static int shtps_ioctl_log_enable(struct shtps_nas_spi *ts, unsigned long arg)
{
	#if defined( SHTPS_LOG_EVENT_ENABLE ) || defined(SHTPS_LOG_DEBUG_ENABLE)
		LogOutputEnable = (int)arg;
		return 0;

	#else
		return -1;
	#endif /* SHTPS_LOG_EVENT_ENABLE || SHTPS_LOG_DEBUG_ENABLE */
}

static int shtps_ioctl_set_low_power_mode(struct shtps_nas_spi *ts, unsigned long arg)
{
#if defined( SHTPS_POWER_MODE_CONTROL_ENABLE )
	SHTPS_LOG_DBG_PRINT("powermode_mutex_lock()\n");
	mutex_lock(&shtps_power_mode_ctrl_lock);

	shtps_set_power_mode(ts, SHTPS_LPMODE_NON_CONTINUOUS_REQ_ON, (int)arg);

	SHTPS_LOG_DBG_PRINT("powermode_mutex_unlock()\n");
	mutex_unlock(&shtps_power_mode_ctrl_lock);
#endif /* #if defined( SHTPS_POWER_MODE_CONTROL_ENABLE ) */

	return 0;
}

static int shtps_ioctl_set_continuous_low_power_mode(struct shtps_nas_spi *ts, unsigned long arg)
{
#if defined( SHTPS_POWER_MODE_CONTROL_ENABLE )
	SHTPS_LOG_DBG_PRINT("powermode_mutex_lock()\n");
	mutex_lock(&shtps_power_mode_ctrl_lock);

	shtps_set_power_mode(ts, SHTPS_LPMODE_CONTINUOUS_REQ_ON, (int)arg);

	SHTPS_LOG_DBG_PRINT("powermode_mutex_unlock()\n");
	mutex_unlock(&shtps_power_mode_ctrl_lock);
#endif /* #if defined( SHTPS_POWER_MODE_CONTROL_ENABLE ) */

	return 0;
}

static int shtps_ioctl_set_high_power_mode(struct shtps_nas_spi *ts, unsigned long arg)
{
#if defined( SHTPS_POWER_MODE_CONTROL_ENABLE )
	SHTPS_LOG_DBG_PRINT("powermode_mutex_lock()\n");
	mutex_lock(&shtps_power_mode_ctrl_lock);

	shtps_set_power_mode(ts, SHTPS_HPMODE_NON_CONTINUOUS_REQ_ON, (int)arg);

	SHTPS_LOG_DBG_PRINT("powermode_mutex_unlock()\n");
	mutex_unlock(&shtps_power_mode_ctrl_lock);
#endif /* #if defined( SHTPS_POWER_MODE_CONTROL_ENABLE ) */

	return 0;
}

/* -----------------------------------------------------------------------------------
 */
static int shtpsif_open(struct inode *inode, struct file *file)
{
	SHTPS_LOG_DBG_PRINT("%s(PID:%ld)\n", __func__, sys_getpid());
	return 0;
}

static int shtpsif_release(struct inode *inode, struct file *file)
{
	SHTPS_LOG_DBG_PRINT("%s(PID:%ld)\n", __func__, sys_getpid());
	return 0;
}

static long shtpsif_ioctl(struct file *file, unsigned int cmd, unsigned long arg)
{
	int	rc = 0;
	struct shtps_nas_spi *ts = g_shtps_nas_spi;

	SHTPS_LOG_DBG_PRINT("ioctl(PID:%ld,CMD:%d,ARG:0x%lx)\n",
								sys_getpid(), cmd, arg);

	if(ts == NULL){
		SHTPS_LOG_DBG_PRINT("shtpsif_ioctl ts == NULL\n");
		return -EFAULT;
	}

	switch(cmd){
	case TPSDEV_ENABLE:						rc = shtps_ioctl_enable(ts, arg);					break;
	case TPSDEV_DISABLE:					rc = shtps_ioctl_disable(ts, arg);					break;
	case TPSDEV_RESET:						rc = shtps_ioctl_reset(ts, arg);					break;
	case TPSDEV_GET_FW_VERSION:				rc = shtps_ioctl_getver(ts, arg);					break;
	case TPSDEV_BL_ENTER:					rc = shtps_ioctl_bl_enter(ts, arg);					break;
	case TPSDEV_BL_ERASE_SECTOR:			rc = shtps_ioctl_bl_erase(ts, arg);					break;
	case TPSDEV_BL_WRITE_IMAGE:				rc = shtps_ioctl_bl_writeimage(ts, arg);			break;
	case TPSDEV_BL_EXIT:					rc = shtps_ioctl_bl_exit(ts, arg);					break;
	case TPSDEV_GET_TOUCHINFO:				rc = shtps_ioctl_get_touchinfo(ts, arg);			break;
	case TPSDEV_GET_TOUCHINFO_UNTRANS:		rc = shtps_ioctl_get_touchinfo_untrans(ts, arg);	break;
	case TPSDEV_GET_TOUCHINFO_BLK:			rc = shtps_ioctl_get_touchinfo_blk(ts, arg);		break;
	case TPSDEV_GET_TOUCHINFO_UNTRANS_BLK:	rc = shtps_ioctl_get_touchinfo_untrans_blk(ts, arg);break;
	case TPSDEV_GET_TOUCHINFO_CANCEL:		rc = shtps_ioctl_get_touchinfo_cancel(ts, arg);		break;
	case TPSDEV_READ_REG:					rc = shtps_ioctl_reg_read(ts, arg);					break;
	case TPSDEV_READ_ALL_REG:				rc = shtps_ioctl_reg_allread(ts, arg);				break;
	case TPSDEV_WRITE_REG:					rc = shtps_ioctl_reg_write(ts, arg);				break;
	case TPSDEV_TESTMODE_ENTER:				rc = shtps_ioctl_tm_enter(ts, arg);					break;
	case TPSDEV_TESTMODE_EXIT:				rc = shtps_ioctl_tm_exit(ts, arg);					break;
	case TPSDEV_TESTMODE_GETDATA:			rc = shtps_ioctl_tm_getdata(ts, arg);				break;
	case TPSDEV_CALIBRATION_PARAM:			rc = shtps_ioctl_calibration_param(ts, arg);		break;
	case TPSDEV_DEBUG_REQEVENT:				rc = shtps_ioctl_debug_reqevent(ts, arg);			break;
	case TPSDEV_GET_FW_VERSION_BUILTIN:		rc = shtps_ioctl_getver_builtin(ts, arg);			break;
	case TPSDEV_TESTMODE_GETDATA_NUM:		rc = shtps_ioctl_tm_getdata_num(ts, arg);			break;
	case TPSDEV_BL_ERASE_SECTOR_LOADER:		rc = shtps_ioctl_bl_erase_loader(ts, arg);			break;
	case TPSDEV_BL_WRITE_IMAGE_LOADER:		rc = shtps_ioctl_bl_writeimage_loader(ts, arg);		break;
	case TPSDEV_ENABLE_NO_ID:				rc = shtps_ioctl_enable_no_id(ts, arg);				break;
	case TPSDEV_FWUPDATE_CONTINUE:			rc = shtps_ioctl_fw_update_continue_req(ts, arg);	break;
	case TPSDEV_BOOTLOADER_SELECT:			rc = shtps_ioctl_bootloader_select(ts, arg);		break;
	case TPSDEV_LOGOUTPUT_ENABLE:			rc = shtps_ioctl_log_enable(ts, arg);				break;
	case TPSDEV_SET_LOWPOWER_MODE:			rc = shtps_ioctl_set_low_power_mode(ts, arg);		break;
	case TPSDEV_SET_CONT_LOWPOWER_MODE:		rc = shtps_ioctl_set_continuous_low_power_mode(ts, arg); break;
	case TPSDEV_SET_HIGHPOWER_MODE:			rc = shtps_ioctl_set_high_power_mode(ts, arg);		break;
	case TPSDEV_CALIBRATION_PEN_PARAM:		rc = shtps_ioctl_calibration_pen_param(ts, arg);	break;
	default:
		SHTPS_LOG_DBG_PRINT("shtpsif_ioctl switch(cmd) default\n");
		rc = -ENOIOCTLCMD;
		break;
	}
	return rc;
}

static const struct file_operations shtpsif_fileops = {
	.owner   = THIS_MODULE,
	.open    = shtpsif_open,
	.release = shtpsif_release,
	.unlocked_ioctl   = shtpsif_ioctl,
};

int __init shtpsif_init(void)
{
	int rc;

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

#if 0
	if(sizeof(dev_attrs) > sizeof(struct attribute *)){
		rc = sysfs_create_group(&(shtpsif_cdev.kobj), &dev_attr_grp);
	}
#endif
	SHTPS_LOG_DBG_PRINT("shtpsif_init() done\n");

	return 0;

err_via_cdev_add:
	cdev_del(&shtpsif_cdev);
error_vid_class_device_create:
	class_destroy(shtpsif_class);
error_vid_class_create:
	unregister_chrdev_region(shtpsif_devid, 1);
	return rc;
}
module_init(shtpsif_init);

static void __exit shtpsif_exit(void)
{
#if defined(SHTPS_DEBUG_CREATE_KOBJ_ENABLE)
	sysfs_remove_group(&(shtpsif_cdev.kobj), &dev_attr_grp);
#endif /* SHTPS_DEBUG_CREATE_KOBJ_ENABLE */
	cdev_del(&shtpsif_cdev);
	class_destroy(shtpsif_class);
	unregister_chrdev_region(shtpsif_devid, 1);

	SHTPS_LOG_DBG_PRINT("[shtpsif]shtpsif_exit() done\n");
}
module_exit(shtpsif_exit);


/* -----------------------------------------------------------------------------------
 */
static int shtps_nas_open(struct input_dev *dev)
{
	struct shtps_nas_spi *ts = (struct shtps_nas_spi*)input_get_drvdata(dev);
	int ret = 0;

	SHTPS_LOG_DBG_PRINT("%s(PID:%ld)\n",__func__, sys_getpid());

	#if defined( SHTPS_ASYNC_OPEN_ENABLE )
		shtps_func_request_async(ts, SHTPS_FUNC_REQ_EVEMT_OPEN);
		ret = 0;
	#else
		#if defined( SHTPS_BOOT_FWUPDATE_ENABLE )
			if( shtps_boot_fwupdate_enable_check(ts) != 0 ){
				shtps_work_bootfwupdatef(ts);
			}
		#endif /* #if defined( SHTPS_BOOT_FWUPDATE_ENABLE ) */

		ret = request_event(ts, SHTPS_EVENT_START, 0);
		shtps_wait_startup(ts);
	#endif /* #if defined( SHTPS_ASYNC_OPEN_ENABLE ) */

	return ret;
}

static void shtps_nas_close(struct input_dev *dev)
{
	struct shtps_nas_spi *ts = (struct shtps_nas_spi*)input_get_drvdata(dev);

	SHTPS_LOG_DBG_PRINT("%s(PID:%ld)\n",__func__, sys_getpid());

	#if defined( SHTPS_ASYNC_OPEN_ENABLE )
		shtps_func_request_async(ts, SHTPS_FUNC_REQ_EVEMT_CLOSE);
	#else
		request_event(ts, SHTPS_EVENT_STOP, 0);
	#endif /* #if defined( SHTPS_ASYNC_OPEN_ENABLE ) */
}

static int __devinit shtps_nas_probe(struct spi_device *spi)
{
	int result;
	struct input_dev *input_dev;
	struct shtps_nas_spi *ts;
	struct shtps_nassau_touch_platform_data *pdata = spi->dev.platform_data;
	
	SHTPS_LOG_DBG_PRINT("%s() called\n", __func__);

	mutex_lock(&shtps_ctrl_lock);

	SHTPS_LOG_DBG_PRINT("%s() allocate memory\n", __func__);
	ts = kzalloc(sizeof(struct shtps_nas_spi), GFP_KERNEL);
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

	ts->offset.enabled = 0;
	ts->offset_pen.enabled = 0;

	g_shtps_nas_spi	= ts;
	ts->spi			= spi;
	ts->input		= input_dev;
	ts->pow_pin		= pdata->gpio_power_pin;
	ts->rst_pin		= pdata->gpio_reset_pin;
	ts->v17_power	= pdata->v17_power;
	ts->irq_mgr.irq	= spi->irq;
	ts->pending		= 0;
	ts->bt_ver		= 0;
	ts->bt_paramver	= 0;
	ts->bt_select	= 0;
	ts->fw_update_continue = 0;
	ts->is_lcd_on	= 1;
	ts->fw_startup_state = SHTPS_FW_STARTUP_UNKNOWN;

	memset(&ts->diag, 0, sizeof(ts->diag));
	memset(&ts->testmode, 0, sizeof(ts->testmode));
	memset(&ts->loader, 0, sizeof(ts->loader));
	memset(&ts->command, 0, sizeof(ts->command));
	memset(&ts->state_mgr, 0, sizeof(ts->state_mgr));

	#if defined(SHTPS_DRAG_STEP_ENABLE)
		memset(&ts->center_info, 0, sizeof(ts->center_info));
		memset(&ts->touch_state, 0, sizeof(ts->touch_state));
		{
			int i;
			for(i = 0;i < SHTPS_FINGER_MAX;i++){
				ts->touch_state.dragStep[i][0] = SHTPS_DRAG_THRESHOLD_ZERO;
				ts->touch_state.dragStep[i][1] = SHTPS_DRAG_THRESHOLD_ZERO;
				ts->touch_state.dragStepReturnTime[i][0] = SHTPS_DRAG_THRESH_RETURN_TIME_ZERO;
				ts->touch_state.dragStepReturnTime[i][1] = SHTPS_DRAG_THRESH_RETURN_TIME_ZERO;
			}
		}
		memset(ts->drag_hist, 0, sizeof(ts->drag_hist));
	#endif /* SHTPS_DRAG_STEP_ENABLE */

	#if defined(SHTPS_CPU_CLOCK_CONTROL_ENABLE)
		ts->report_event             = SHTPS_EVENT_TU;
		ts->perf_lock_enable_time_ms = SHTPS_PERF_LOCK_ENABLE_TIME_MS;
		ts->perf_lock_level          = SHTPS_PERF_LOCK_LEVEL_HIGHEST;
		perf_lock_init(&ts->perf_lock_level_highest, PERF_LOCK_HIGHEST, "shtps_perf_lock_highest");
		perf_lock_init(&ts->perf_lock_level_high,    PERF_LOCK_1242MHz, "shtps_perf_lock_high");
		perf_lock_init(&ts->perf_lock_level_low,     PERF_LOCK_810MHz,  "shtps_perf_lock_low");
		INIT_DELAYED_WORK(&ts->perf_lock_disable_delayed_work, shtps_perf_lock_disable_delayed_work_function);
	#endif /* SHTPS_CPU_CLOCK_CONTROL_ENABLE */

	init_waitqueue_head(&ts->diag.wait);
	init_waitqueue_head(&ts->testmode.wait);
	init_waitqueue_head(&ts->loader.wait);
	init_waitqueue_head(&ts->command.wait);
	init_waitqueue_head(&ts->wait_start);

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

	shtps_clear_fingerinfo(&ts->report_info);

	snprintf(ts->phys, sizeof(ts->phys), "%s", dev_name(&spi->dev));

	input_dev->name 		= spi->modalias;
	input_dev->phys         = ts->phys;
	input_dev->id.vendor	= 0x0001;
	input_dev->id.product	= 0x0002;
	input_dev->id.version	= 0x0100;
	input_dev->dev.parent	= &spi->dev;
	input_dev->open			= shtps_nas_open;
	input_dev->close		= shtps_nas_close;

	SHTPS_LOG_DBG_PRINT("%s() irq request\n", __func__);
	if(shtps_irq_resuest(ts)){
		SHTPS_LOG_ERR_PRINT("request_irq error\n");
		mutex_unlock(&shtps_ctrl_lock);
		goto fail_alloc_mem;
	}

	mutex_unlock(&shtps_ctrl_lock);

	SHTPS_LOG_DBG_PRINT("%s() spi setup\n", __func__);
	if(spi_setup(ts->spi) < 0){
		SHTPS_LOG_DBG_PRINT("spi_setup fail\n");
	}
	
	__set_bit(EV_KEY, input_dev->evbit);
	__set_bit(EV_ABS, input_dev->evbit);
	__set_bit(INPUT_PROP_DIRECT, input_dev->propbit);
	
	input_set_drvdata(input_dev, ts);
	input_mt_init_slots(input_dev, 16);
	
	if(input_dev->mt == NULL){
		result = -ENOMEM;
		goto fail_alloc_mem;
	}

#if defined(SHTPS_PEN_DETECT_ENABLE)
	input_set_abs_params(input_dev, ABS_MT_TOOL_TYPE,   0, MT_TOOL_MAX, 0, 0);
#endif /* SHTPS_PEN_DETECT_ENABLE */
	input_set_abs_params(input_dev, ABS_MT_TOUCH_MAJOR, 0, SHTPS_FINGER_WIDTH_PALMDET, 0, 0);
	input_set_abs_params(input_dev, ABS_MT_POSITION_X,  0, CONFIG_SHTPS_NASSAU_LCD_SIZE_X, 0, 0);
	input_set_abs_params(input_dev, ABS_MT_POSITION_Y,  0, CONFIG_SHTPS_NASSAU_LCD_SIZE_Y, 0, 0);
	input_set_abs_params(input_dev, ABS_MT_PRESSURE,    0, SHTPS_NASSAU_MT_PRESSURE_MAX, 0, 0);

	SHTPS_LOG_DBG_PRINT("%s() input register\n", __func__);
	result = input_register_device(input_dev);
	if (result){
		goto fail_input_register_device;
	}

	#if defined(SHTPS_SYSTEM_BOOT_MODE_CHECK_ENABLE)
		ts->system_boot_mode = sh_boot_get_bootmode();
		if( (ts->system_boot_mode == SH_BOOT_O_C) || (ts->system_boot_mode == SH_BOOT_U_O_C) ){
			SHTPS_LOG_DBG_PRINT("%s() startup lcd off\n", __func__);
			ts->is_lcd_on = 0;
		}else{
			if( sh_smem_get_softupdate_flg() ){
				SHTPS_LOG_DBG_PRINT("%s() startup lcd off\n", __func__);
				ts->is_lcd_on = 0;
			}else{
				SHTPS_LOG_DBG_PRINT("%s() startup lcd on\n", __func__);
				ts->is_lcd_on = 1;
			}
		}
	#endif /* SHTPS_SYSTEM_BOOT_MODE_CHECK_ENABLE */

	shtps_performance_check_init();

#if defined(SHTPS_DEBUG_CREATE_KOBJ_ENABLE)
	if(sizeof(dev_attrs) > sizeof(struct attribute *)){
		result = sysfs_create_group(&(spi->dev.kobj), &dev_attr_grp);
	}
#endif /* SHTPS_DEBUG_CREATE_KOBJ_ENABLE */

	SHTPS_LOG_DBG_PRINT("%s() done\n", __func__);
	return 0;

fail_input_register_device:
fail_alloc_mem:
	if(input_dev->mt){
		input_mt_destroy_slots(input_dev);
	}
	input_free_device(input_dev);
	kfree(ts);
	return result;
}

static int __devexit shtps_nas_remove(struct spi_device *spi)
{
	struct shtps_nas_spi *ts = spi_get_drvdata(spi);
	struct shtps_nassau_touch_platform_data *pdata = spi->dev.platform_data;

	#if defined( SHTPS_ASYNC_OPEN_ENABLE )
		destroy_workqueue(ts->workqueue_p);
	#endif /* #if defined( SHTPS_ASYNC_OPEN_ENABLE ) */

	free_irq(ts->irq_mgr.irq, ts);
	if (pdata && pdata->teardown){
		pdata->teardown(&spi->dev);
	}

	if(ts->input->mt){
		input_mt_destroy_slots(ts->input);
	}
	input_free_device(ts->input);

	g_shtps_nas_spi = NULL;
	kfree(ts);

	SHTPS_LOG_DBG_PRINT("shtps_rmi_remove() done\n");
	return 0;
}

static struct spi_driver shtps_nas_driver = {
	.probe		= shtps_nas_probe,
	.remove		= __devexit_p(shtps_nas_remove),
	.driver		= {
		.name   = SPI_TOUCH_NAME,
		.owner  = THIS_MODULE,
        .bus    = &spi_bus_type,
	},
};

static int __init shtps_nas_init(void)
{
	return spi_register_driver(&shtps_nas_driver);
}
module_init(shtps_nas_init);

static void __exit shtps_nas_exit(void)
{
	spi_unregister_driver(&shtps_nas_driver);
}
module_exit(shtps_nas_exit);

/* -----------------------------------------------------------------------------------
 */
void msm_tps_setsleep(int on)
{
	struct shtps_nas_spi *ts = g_shtps_nas_spi;

	if(ts){
		if(on){
			request_event(ts, SHTPS_EVENT_SLEEP, 0);
		}else{
			request_event(ts, SHTPS_EVENT_WAKEUP, 0);
		}
	}
}
EXPORT_SYMBOL(msm_tps_setsleep);
