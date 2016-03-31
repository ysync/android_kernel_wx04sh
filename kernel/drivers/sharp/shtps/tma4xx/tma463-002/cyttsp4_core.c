/*
 * Core Source for:
 * Cypress TrueTouch(TM) Standard Product (TTSP) touchscreen drivers.
 * For use with Cypress Gen4 and Solo parts.
 * Supported parts include:
 * CY8CTMA884/616
 * CY8CTMA4XX
 *
 * Copyright (C) 2009-2012 Cypress Semiconductor, Inc.
 * Copyright (C) 2011 Motorola Mobility, Inc.
 *
 * This program is free software; you can redistribute it and/or
 * modify it under the terms of the GNU General Public License
 * version 2, and only version 2, as published by the
 * Free Software Foundation.
 *
 * This program is distributed in the hope that it will be useful,
 * but WITHOUT ANY WARRANTY; without even the implied warranty of
 * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
 * GNU General Public License for more details.
 *
 * You should have received a copy of the GNU General Public License along
 * with this program; if not, write to the Free Software Foundation, Inc.,
 * 51 Franklin Street, Fifth Floor, Boston, MA 02110-1301, USA.
 *
 * Contact Cypress Semiconductor at www.cypress.com <kev@cypress.com>
 *
 */
#include "cyttsp4_core.h"

#include <linux/delay.h>
#include <linux/gpio.h>
#include <linux/interrupt.h>
#include <linux/slab.h>
#include <sharp/touch_platform.h>
#ifdef SH_TPSIF_COMMAND
#include <sharp/shtps_dev.h>
#endif	/* SH_TPSIF_COMMAND */
#ifdef TMA443_COMPATIBILITY
#include <sharp/sh_smem.h>
#endif /* TMA443_COMPATIBILITY */
#include <linux/firmware.h>	/* This enables firmware class loader code */

#define SH_TPSIF_LOCAL_DEVICE_REGIST_ENABLE
#if defined(SH_TPSIF_LOCAL_DEVICE_REGIST_ENABLE)
	#define SH_TPSIF_IOCTL_DISABLE
#endif /* SH_TPSIF_LOCAL_DEVICE_REGIST_ENABLE */

/* platform address lookup offsets */
#define CY_TCH_ADDR_OFS		0
#define CY_LDR_ADDR_OFS		1

/* helpers */
#define GET_NUM_TOUCHES(x)          ((x) & 0x1F)
#define IS_LARGE_AREA(x)            ((x) & 0x20)
#define IS_BAD_PKT(x)               ((x) & 0x20)
#define GET_HSTMODE(reg)            ((reg & 0x70) >> 4)
#define IS_BOOTLOADERMODE(reg)      (reg & 0x01)
#ifdef SH_TPSIF_COMMAND
#define	MINMAX(min, max, val)	((min)>(val) ? (min) : ((max)<(val) ? (max) : (val)))
#define	SET_POINT(val, x1, y1)      val.x = (x1); val.y = (y1)
#define	SET_AREA(val, x1, y1, x2, y2, x3, y3, x4, y4)	\
	val.p.x=x1;val.p.y=y1;val.q.x=x2;val.q.y=y2;	\
	val.r.x=x3;val.r.y=y3;val.s.x=x4;val.s.y=y4;
#endif	/* SH_TPSIF_COMMAND */

/* maximum number of concurrent tracks */
#define CY_NUM_TCH_ID               10
/* maximum number of track IDs */
#define CY_NUM_TRK_ID               16
/* maximum number of command data bytes */
#define CY_NUM_DAT                  6
/* maximum number of config block read data */
#define CY_NUM_CONFIG_BYTES        128

#define CY_REG_BASE                 0x00
#define CY_DELAY_DFLT               20		/* ms */
#define CY_DELAY_MAX                (500/CY_DELAY_DFLT)	/* half second */
#define CY_HALF_SEC_TMO_MS          500		/* half second in msecs */
#define CY_TEN_SEC_TMO_MS           10000	/* ten seconds in msecs */
#define CY_HANDSHAKE_BIT            0x80
#define CY_WAKE_DFLT                99	/* causes wake strobe on INT line
					 * in sample board configuration
					 * platform data->hw_recov() function
					 */
/* power mode select bits */
#define CY_SOFT_RESET_MODE          0x01
#define CY_DEEP_SLEEP_MODE          0x02
#define CY_LOW_POWER_MODE           0x04
/* device mode bits */
#define CY_MODE_CHANGE              0x08 /* rd/wr hst_mode */
#define CY_OPERATE_MODE             0x00 /* rd/wr hst_mode */
#define CY_SYSINFO_MODE             0x10 /* rd/wr hst_mode */
#define CY_CONFIG_MODE              0x20 /* rd/wr hst_mode */
#define CY_BL_MODE                  0x01 /* wr hst mode == soft reset
					  * was 0x10 to rep_stat for LTS
					  */
#define CY_IGNORE_VALUE             0xFFFF
#define CY_CMD_RDY_BIT              0x40

#define CY_REG_OP_START             0
#define CY_REG_SI_START             0
#define CY_REG_OP_END               0x20
#define CY_REG_SI_END               0x20

#ifdef CY_USE_TMA400
#define CY_TCH_CRC_LOC_TMA400       5884 /* location of CRC in touch EBID */
#endif /* --CY_USE_TMA400 */

/* register field lengths */
#define CY_NUM_REVCTRL              8
#define CY_NUM_MFGID                8
#define CY_NUM_TCHREC               10
#define CY_NUM_DDATA                32
#define CY_NUM_MDATA                64
#define CY_TMA884_MAX_BYTES         255 /*
					  * max reg access for TMA884
					  * in config mode
					  */
#define CY_TMA400_MAX_BYTES         512 /*
					  * max reg access for TMA400
					  * in config mode
					  */

/* touch event id codes */
#define CY_GET_EVENTID(reg)         ((reg & 0x60) >> 5)
#define CY_GET_TRACKID(reg)         (reg & 0x1F)
#define CY_NOMOVE                   0
#define CY_TOUCHDOWN                1
#define CY_MOVE                     2
#define CY_LIFTOFF                  3

#define CY_CFG_BLK_SIZE             126
#define CY_EBID_ROW_SIZE_DFLT       128

#define CY_BL_VERS_SIZE             12
#define CY_NUM_TMA400_TT_CFG_BLK    51 /* Rev84 mapping */

#if defined(CY_USE_FORCE_LOAD) || defined(CONFIG_TOUCHSCREEN_DEBUG) || defined(SH_TPSIF_COMMAND)
#define CY_BL_FW_NAME_SIZE          NAME_MAX
#endif

#ifdef CONFIG_TOUCHSCREEN_DEBUG
#define CY_BL_TXT_FW_IMG_SIZE       128261
#define CY_BL_BIN_FW_IMG_SIZE       128261
#define CY_NUM_PKG_PKT              4
#define CY_NUM_PKT_DATA             32
#define CY_MAX_PKG_DATA             (CY_NUM_PKG_PKT * CY_NUM_PKT_DATA)
#define CY_MAX_IC_BUF               256
#endif /* --CONFIG_TOUCHSCREEN_DEBUG */

#ifdef CY_USE_REG_ACCESS
#define CY_RW_REGID_MAX             0xFFFF
#define CY_RW_REG_DATA_MAX          0xFF
#endif

/* abs settings */
/* abs value offset */
#define CY_SIGNAL_OST   0
#define CY_MIN_OST      1
#define CY_MAX_OST      2
#define CY_FUZZ_OST     3
#define CY_FLAT_OST     4
/* axis signal offset */
#define CY_NUM_ABS_SET  5 /* number of abs signals */
#define CY_ABS_X_OST    0
#define CY_ABS_Y_OST    1
#define CY_ABS_P_OST    2
#define CY_ABS_W_OST    3
#define CY_ABS_ID_OST   4

#ifdef SH_TPSIF_COMMAND
/* boot mode */
#define SH_BOOT_MODE_HW_CHK	0x41

#define	ADJUST_POINT	6	/* Number of Adjustment points */
#define	AREA_COUNT	(ADJUST_POINT * 2) /* Number of Adjustment Area */
#define DOUBLE_ACCURACY	10000

#define	POS_X0		0
#define	POS_X1		179
#define	POS_X2		539
#define	POS_X3		719
#define	POS_Y0		0
#define	POS_Y1		319
#define	POS_Y2		639
#define	POS_Y3		959
#define	POS_Y4		1279
#define	POS_LIMIT	100
#endif	/* SH_TPSIF_COMMAND */

enum cyttsp4_driver_state {
	CY_IDLE_STATE,		/* IC cannot be reached */
	CY_READY_STATE,		/* pre-operational; ready to go to ACTIVE */
	CY_ACTIVE_STATE,	/* app is running, IC is scanning */
	CY_SLEEP_STATE,		/* app is running, IC is idle */
	CY_BL_STATE,		/* bootloader is running */
	CY_SYSINFO_STATE,	/* switching to sysinfo mode */
	CY_CMD_STATE,		/* command initiation mode */
	CY_EXIT_BL_STATE,	/* sync bl heartbeat to app ready int */
	CY_TRANSFER_STATE,	/* changing states */
	CY_INVALID_STATE	/* always last in the list */
};

static const char * const cyttsp4_driver_state_string[] = {
	/* Order must match enum cyttsp4_driver_state above */
	"IDLE",
	"READY",
	"ACTIVE",
	"SLEEP",
	"BOOTLOADER",
	"SYSINFO",
	"CMD",
	"EXIT_BL",
	"TRANSFER",
	"INVALID"
};

enum cyttsp4_controller_mode {
	CY_MODE_BOOTLOADER,
	CY_MODE_SYSINFO,
	CY_MODE_OPERATIONAL,
	CY_MODE_CONFIG,
	CY_MODE_NUM
};

enum cyttsp4_ic_grpnum {
	CY_IC_GRPNUM_RESERVED = 0,
	CY_IC_GRPNUM_CMD_REGS,
	CY_IC_GRPNUM_TCH_REP,
	CY_IC_GRPNUM_DATA_REC,
	CY_IC_GRPNUM_TEST_REC,
	CY_IC_GRPNUM_PCFG_REC,
	CY_IC_GRPNUM_TCH_PARM_VAL,
	CY_IC_GRPNUM_TCH_PARM_SIZ,
	CY_IC_GRPNUM_RESERVED1,
	CY_IC_GRPNUM_RESERVED2,
	CY_IC_GRPNUM_OPCFG_REC,
	CY_IC_GRPNUM_DDATA_REC,
	CY_IC_GRPNUM_MDATA_REC,
	CY_IC_GRPNUM_TEST_DATA,
	CY_IC_GRPNUM_NUM
};

enum cyttsp4_ic_op_mode_commands {
	CY_GET_PARAM_CMD = 0x02,
	CY_SET_PARAM_CMD = 0x03,
	CY_GET_CFG_BLK_CRC = 0x05,
};

enum cyttsp4_ic_config_mode_commands {
	CY_GET_EBID_ROW_SIZE = 0x02,
	CY_READ_EBID_DATA = 0x03,
	CY_WRITE_EBID_DATA = 0x04,
	CY_VERIFY_EBID_CRC = 0x11,
};

#ifdef CY_USE_TMA400
enum cyttsp4_ic_ebid {
	CY_TCH_PARM_EBID = 0x00,
	CY_MDATA_EBID = 0x01,
	CY_DDATA_EBID = 0x02,
};
#endif /* --CY_USE_TMA400 */

#ifdef CY_USE_TMA884
enum cyttsp4_ic_ebid {
	CY_TCH_PARM_EBID = 0x00,
	CY_DDATA_EBID = 0x05,
	CY_MDATA_EBID = 0x06,
};
#endif /* --CY_USE_TMA884 */

enum cyttsp4_flags {
	CY_NO_FLAGS = 0x00,
#ifdef CY_USE_DEBUG_TOOLS
	CY_FLIP = 0x08,
	CY_INV_X = 0x10,
	CY_INV_Y = 0x20,
#endif /* --CY_USE_DEBUG_TOOLS */
};

/* GEN4/SOLO Operational interface definitions */
struct cyttsp4_touch {
	int x;	/* x position */
	int y;	/* y position */
	int p;	/* pressure */
	int t;	/* track id */
	int e;	/* event id */
	int o;	/* object type */
	int w;	/* size */
};

/* TMA400 TT_CFG interface definitions */
struct cyttsp4_tma400A_config_crc {
	u8 CONFIG_CRC[4];
};
struct cyttsp4_tma400A_sdk_controller_config {
	u8 SDK_CTRL_CFG_SIZE[4];
	u8 X_LEN_PHY[2];
	u8 Y_LEN_PHY[2];
	u8 HST_MODE0;
	u8 ACT_DIST0;
	u8 SCAN_TYP0;
	u8 ACT_INTRVL0;
	u8 ACT_LFT_INTRVL0;
	u8 Reserved_1;
	u8 TCH_TMOUT0[2];
	u8 LP_INTRVL0[2];
	u8 PWR_CFG;
	u8 INT_CFG;
	u8 INT_PULSE_DATA;
	u8 OPMODE_CFG;
	u8 HANDSHAKE_TIMEOUT[2];
	u8 TIMER_CAL_INTERVAL;
	u8 Reserved_2;
	u8 RP2P_MIN[2];
	u8 ILEAK_MAX[2];
	u8 RFB_P2P[2];
	u8 RFB_EXT[2];
	u8 IDACOPEN_LOW;
	u8 IDACOPEN_HIGH;
	u8 GIDAC_OPEN;
	u8 GAIN_OPEN;
	u8 POST_CFG;
	u8 GESTURE_CFG;
	u8 GEST_EN[32];
	u8 Reserved_align[52];
} __packed;

struct cyttsp4_tma400A_tt_cfg {
	struct cyttsp4_tma400A_config_crc config_crc;
	struct cyttsp4_tma400A_sdk_controller_config sdk_controller_config;
} __packed;

/* TTSP System Information interface definitions */
struct cyttsp4_cydata {
	u8 ttpidh;
	u8 ttpidl;
	u8 fw_ver_major;
	u8 fw_ver_minor;
	u8 revctrl[CY_NUM_REVCTRL];
	u8 blver_major;
	u8 blver_minor;
	u8 jtag_si_id3;
	u8 jtag_si_id2;
	u8 jtag_si_id1;
	u8 jtag_si_id0;
	u8 mfgid_sz;
	u8 mfg_id[CY_NUM_MFGID];
	u8 cyito_idh;
	u8 cyito_idl;
	u8 cyito_verh;
	u8 cyito_verl;
	u8 ttsp_ver_major;
	u8 ttsp_ver_minor;
	u8 device_info;
} __packed;

struct cyttsp4_test {
	u8 post_codeh;
	u8 post_codel;
} __packed;

struct cyttsp4_pcfg {
	u8 electrodes_x;
	u8 electrodes_y;
	u8 len_xh;
	u8 len_xl;
	u8 len_yh;
	u8 len_yl;
	u8 axis_xh;
	u8 axis_xl;
	u8 axis_yh;
	u8 axis_yl;
	u8 max_zh;
	u8 max_zl;
} __packed;

struct cyttsp4_opcfg {
	u8 cmd_ofs;
	u8 rep_ofs;
	u8 rep_szh;
	u8 rep_szl;
	u8 num_btns;
	u8 tt_stat_ofs;
	u8 obj_cfg0;
	u8 max_tchs;
	u8 tch_rec_siz;
	u8 tch_rec_0;	/* x position */
	u8 tch_rec_1;
	u8 tch_rec_2;	/* y position */
	u8 tch_rec_3;
	u8 tch_rec_4;	/* pressure */
	u8 tch_rec_5;
	u8 tch_rec_6;	/* track id */
	u8 tch_rec_7;
	u8 tch_rec_8;	/* event id */
	u8 tch_rec_9;
	u8 tch_rec_10;	/* object type */
	u8 tch_rec_11;
	u8 tch_rec_12;	/* size */
	u8 tch_rec_13;
} __packed;

struct cyttsp4_sysinfo_data {
	u8 hst_mode;
	u8 reserved;
	u8 map_szh;
	u8 map_szl;
	u8 cydata_ofsh;
	u8 cydata_ofsl;
	u8 test_ofsh;
	u8 test_ofsl;
	u8 pcfg_ofsh;
	u8 pcfg_ofsl;
	u8 opcfg_ofsh;
	u8 opcfg_ofsl;
	u8 ddata_ofsh;
	u8 ddata_ofsl;
	u8 mdata_ofsh;
	u8 mdata_ofsl;
} __packed;

struct cyttsp4_sysinfo_ptr {
	struct cyttsp4_cydata *cydata;
	struct cyttsp4_test *test;
	struct cyttsp4_pcfg *pcfg;
	struct cyttsp4_opcfg *opcfg;
	struct cyttsp4_ddata *ddata;
	struct cyttsp4_mdata *mdata;
} __packed;

struct cyttsp4_sysinfo_ofs {
	size_t cmd_ofs;
	size_t rep_ofs;
	size_t rep_sz;
	size_t tt_stat_ofs;
	size_t tch_rec_siz;
	size_t obj_cfg0;
	size_t max_tchs;
	size_t mode_size;
	size_t data_size;
	size_t map_sz;
	size_t cydata_ofs;
	size_t test_ofs;
	size_t pcfg_ofs;
	size_t opcfg_ofs;
	size_t ddata_ofs;
	size_t mdata_ofs;
	size_t cydata_size;
	size_t test_size;
	size_t pcfg_size;
	size_t opcfg_size;
	size_t ddata_size;
	size_t mdata_size;
	size_t tch_rec_x_ofs;	/* x position */
	size_t tch_rec_x_size;
	size_t tch_rec_x_max;
	size_t tch_rec_y_ofs;	/* y position */
	size_t tch_rec_y_size;
	size_t tch_rec_y_max;
	size_t tch_rec_p_ofs;	/* pressure */
	size_t tch_rec_p_size;
	size_t tch_rec_p_max;
	size_t tch_rec_t_ofs;	/* track id */
	size_t tch_rec_t_size;
	size_t tch_rec_t_max;
	size_t tch_rec_e_ofs;	/* event id */
	size_t tch_rec_e_size;
	size_t tch_rec_e_max;
	size_t tch_rec_o_ofs;	/* object type */
	size_t tch_rec_o_size;
	size_t tch_rec_o_max;
	size_t tch_rec_w_ofs;	/* size */
	size_t tch_rec_w_size;
	size_t tch_rec_w_max;
};

/* driver context structure definitions */
#ifdef CONFIG_TOUCHSCREEN_DEBUG
struct cyttsp4_dbg_pkg {
	bool ready;
	int cnt;
	u8 data[CY_MAX_PKG_DATA];
};
#endif /* --CONFIG_TOUCHSCREEN_DEBUG */

struct cyttsp4 {
	struct device *dev;
	int irq;
	struct input_dev *input;
	struct mutex data_lock;		/* prevent concurrent accesses */
	struct workqueue_struct		*cyttsp4_wq;
	struct work_struct		cyttsp4_resume_startup_work;
	char phys[32];
	const struct bus_type *bus_type;
	const struct touch_platform_data *platform_data;
	u8 *xy_mode;			/* operational mode and status regs */
	u8 *xy_data;			/* operational touch regs */
	u8 *xy_data_touch1;		/* includes 1-byte for tt_stat */
	struct cyttsp4_bus_ops *bus_ops;
	struct cyttsp4_sysinfo_data sysinfo_data;
	struct cyttsp4_sysinfo_ptr sysinfo_ptr;
	struct cyttsp4_sysinfo_ofs si_ofs;
	struct completion int_running;
	struct completion si_int_running;
	struct completion ready_int_running;
	enum cyttsp4_driver_state driver_state;
	enum cyttsp4_controller_mode current_mode;
	bool irq_enabled;
	bool powered; /* protect against multiple open */
	bool was_suspended;
	bool switch_flag;
	bool soft_reset_asserted;
	u16 flags;
	size_t max_config_bytes;
	size_t ebid_row_size;
#ifdef CONFIG_HAS_EARLYSUSPEND
	struct early_suspend early_suspend;
#endif
#ifdef CY_USE_WATCHDOG
	struct work_struct work;
	struct timer_list timer;
#endif
#if defined(CY_USE_FORCE_LOAD) || defined(CONFIG_TOUCHSCREEN_DEBUG) || defined(SH_TPSIF_COMMAND)
	bool waiting_for_fw;
	char *fwname;
#endif
#ifdef CONFIG_TOUCHSCREEN_DEBUG
	u8 *pr_buf;
	bool debug_upgrade;
	int ic_grpnum;
	int ic_grpoffset;
	bool ic_grptest;
#endif /* --CONFIG_TOUCHSCREEN_DEBUG */
#ifdef CY_USE_REG_ACCESS
	size_t rw_regid;
#endif
#ifdef SH_TPSIF_COMMAND
	bool adjust_enabled;
#endif	/* SH_TPSIF_COMMAND */
#ifdef TMA443_COMPATIBILITY
	bool tma443_compat;
#endif	/* TMA443_COMPATIBILITY */
#ifdef CALIBRATION_AFTER_FIRMWARE_UPDATE
	bool startup_fw_upgraded;
#endif	/* CALIBRATION_AFTER_FIRMWARE_UPDATE */
};

#ifdef SH_TPSIF_COMMAND
typedef struct {
	int x;
	int y;
} sh_tpsif_point_t;

typedef struct {
	sh_tpsif_point_t p;		/* Upper left */
	sh_tpsif_point_t q;		/* Upper right */
	sh_tpsif_point_t r;		/* Lower left */
	sh_tpsif_point_t s;		/* Lower right */
} sh_tpsif_area_t;

typedef struct {
	int value;
	int num;
} sh_tpsif_qsort_t;
#endif	/* SH_TPSIF_COMMAND */

#ifdef SH_TPSIF_COMMAND
static struct cyttsp4 *g_tpsif_ts;
static wait_queue_head_t sh_tpsif_wq;
static int sh_tpsif_event;
static struct hrtimer sh_tpsif_polling_timer;
static struct work_struct sh_tpsif_polling_work;

/* Coordinates of six criteria points for adjusting */
static const sh_tpsif_point_t sh_tpsif_base_point[ADJUST_POINT] = {
	{POS_X1, POS_Y1}, {POS_X2, POS_Y1},
	{POS_X1, POS_Y2}, {POS_X2, POS_Y2},
	{POS_X1, POS_Y3}, {POS_X2, POS_Y3},
};
/* Coordinates of the split area for adjusting */
static const sh_tpsif_area_t sh_tpsif_area_rect[AREA_COUNT] = {
	{{POS_X0, POS_Y0}, {POS_X1, POS_Y0}, {POS_X0, POS_Y1}, {POS_X1, POS_Y1}},
	{{POS_X1, POS_Y0}, {POS_X2, POS_Y0}, {POS_X1, POS_Y1}, {POS_X2, POS_Y1}},
	{{POS_X2, POS_Y0}, {POS_X3, POS_Y0}, {POS_X2, POS_Y1}, {POS_X3, POS_Y1}},
	{{POS_X0, POS_Y1}, {POS_X1, POS_Y1}, {POS_X0, POS_Y2}, {POS_X1, POS_Y2}},
	{{POS_X1, POS_Y1}, {POS_X2, POS_Y1}, {POS_X1, POS_Y2}, {POS_X2, POS_Y2}},
	{{POS_X2, POS_Y1}, {POS_X3, POS_Y1}, {POS_X2, POS_Y2}, {POS_X3, POS_Y2}},
	{{POS_X0, POS_Y2}, {POS_X1, POS_Y2}, {POS_X0, POS_Y3}, {POS_X1, POS_Y3}},
	{{POS_X1, POS_Y2}, {POS_X2, POS_Y2}, {POS_X1, POS_Y3}, {POS_X2, POS_Y3}},
	{{POS_X2, POS_Y2}, {POS_X3, POS_Y2}, {POS_X2, POS_Y3}, {POS_X3, POS_Y3}},
	{{POS_X0, POS_Y3}, {POS_X1, POS_Y3}, {POS_X0, POS_Y4}, {POS_X1, POS_Y4}},
	{{POS_X1, POS_Y3}, {POS_X2, POS_Y3}, {POS_X1, POS_Y4}, {POS_X2, POS_Y4}},
	{{POS_X2, POS_Y3}, {POS_X3, POS_Y3}, {POS_X2, POS_Y4}, {POS_X3, POS_Y4}},
};
static sh_tpsif_area_t sh_tpsif_area_diff[AREA_COUNT];
static sh_tpsif_point_t sh_tpsif_adjust_param[ADJUST_POINT];
#endif	/* SH_TPSIF_COMMAND */

#if defined(SH_TPSIF_LOCAL_DEVICE_REGIST_ENABLE)
	#include <linux/spi/spi.h>
	#include <mach/gpiomux.h>
	#include "cyttsp4_params.h"
	#include "cyttsp4_img.h"

	#define SH_TPSIF_SPI_BUS_NUM	5
	#define SH_TPSIF_SPI_MOSI_PORT 22
	#define SH_TPSIF_SPI_MISO_PORT 23
	#define SH_TPSIF_SPI_CS_PORT   24
	#define SH_TPSIF_SPI_CLK_PORT  25

	#define TOUCH_GPIO_RST_CYTTSP 16
	#define TOUCH_GPIO_IRQ_CYTTSP 106

	#define TP_PWR_INPUT_5V 0
	#define TP_PWR_INPUT_3V 1

	#define TP_PWR_EN 33
	#define CY_WAKE_DFLT 99

	#define CY_ABS_MIN_X 0
	#define CY_ABS_MIN_Y 0
	#define CY_ABS_MIN_P 0
	#define CY_ABS_MIN_W 0
	#define CY_ABS_MIN_T 0

	#define CY_ABS_MAX_X 719
	#define CY_ABS_MAX_Y 1279
	#define CY_ABS_MAX_P 255
	#define CY_ABS_MAX_W 255
	#define CY_ABS_MAX_T 9
	#define CY_IGNORE_VALUE 0xFFFF

	static struct gpiomux_setting spi_port_configs = {
		.func = GPIOMUX_FUNC_1,
		.drv = GPIOMUX_DRV_2MA,
		.pull = GPIOMUX_PULL_NONE,
	};

	static struct msm_gpiomux_config sh_tpsif_port_configs[] = {
		{
			.gpio = SH_TPSIF_SPI_MOSI_PORT,
			.settings = {
				[GPIOMUX_ACTIVE]    = &spi_port_configs,
				[GPIOMUX_SUSPENDED] = &spi_port_configs,
			},
		},
		{
			.gpio = SH_TPSIF_SPI_MISO_PORT,
			.settings = {
				[GPIOMUX_ACTIVE]    = &spi_port_configs,
				[GPIOMUX_SUSPENDED] = &spi_port_configs,
			},
		},
		{
			.gpio = SH_TPSIF_SPI_CS_PORT,
			.settings = {
				[GPIOMUX_ACTIVE]    = &spi_port_configs,
				[GPIOMUX_SUSPENDED] = &spi_port_configs,
			},
		},
		{
			.gpio = SH_TPSIF_SPI_CLK_PORT,
			.settings = {
				[GPIOMUX_ACTIVE]    = &spi_port_configs,
				[GPIOMUX_SUSPENDED] = &spi_port_configs,
			},
		},
	};

	static void sh_tpsif_device_port_set(void)
	{
		msm_gpiomux_install(sh_tpsif_port_configs, ARRAY_SIZE(sh_tpsif_port_configs));
	}

	static int cyttsp4_hw_power(int on)
	{
		return 0;
	}

	static int cyttsp4_hw_reset(void)
	{
		int ret = 0;

		gpio_set_value(TOUCH_GPIO_RST_CYTTSP, 1);
		pr_info("%s: gpio_set_value(step%d)=%d\n", __func__, 1, 1);
		msleep(20);
		gpio_set_value(TOUCH_GPIO_RST_CYTTSP, 0);
		pr_info("%s: gpio_set_value(step%d)=%d\n", __func__, 2, 0);
		msleep(40);
		gpio_set_value(TOUCH_GPIO_RST_CYTTSP, 1);
		msleep(91);
		pr_info("%s: gpio_set_value(step%d)=%d\n", __func__, 3, 1);

		return ret;
	}

	static int cyttsp4_hw_recov(int on)
	{
		int retval = 0;

		pr_info("%s: on=%d\n", __func__, on);
		switch (on) {
		case 0:
			cyttsp4_hw_reset();
			retval = 0;
			break;
		case CY_WAKE_DFLT:
			retval = gpio_request(TOUCH_GPIO_IRQ_CYTTSP, NULL);
			if (retval < 0) {
				pr_err("%s: Fail request IRQ pin r=%d\n",
					__func__, retval);
				break;
			}
			retval = gpio_direction_output(TOUCH_GPIO_IRQ_CYTTSP, 0);
			if (retval < 0) {
				pr_err("%s: Fail switch IRQ pin to output"
					" r=%d\n", __func__, retval);
			} else {
				udelay(2000);
				retval = gpio_direction_input(TOUCH_GPIO_IRQ_CYTTSP);
				if (retval < 0) {
					pr_err("%s: Fail switch IRQ pin to input"
						" r=%d\n", __func__, retval);
				}
			}
			gpio_free(TOUCH_GPIO_IRQ_CYTTSP);
			break;
		default:
			retval = -ENOSYS;
			break;
		}

		return retval;
	}

	static int cyttsp4_irq_stat(void)
	{
		return gpio_get_value(TOUCH_GPIO_IRQ_CYTTSP);
	}

	static struct touch_settings cyttsp4_sett_param_regs = {
		.data = (uint8_t *)&cyttsp4_param_regs[0],
		.size = sizeof(cyttsp4_param_regs),
		.tag = 0,
	};

	static struct touch_settings cyttsp4_sett_param_size = {
		.data = (uint8_t *)&cyttsp4_param_size[0],
		.size = sizeof(cyttsp4_param_size),
		.tag = 0,
	};

	/* Design Data Table */
	static u8 cyttsp4_ddata[] = {
		0, 1, 2, 3, 4, 5, 6, 7, 8, 9, 10, 11, 12, 13, 14, 15,
		16, 17, 18, 19, 20, 21, 22, 23, 24 /* test padding ,
		25, 26, 27, 28, 29, 30, 31 */
	};

	static struct touch_settings cyttsp4_sett_ddata = {
		.data = (uint8_t *)&cyttsp4_ddata[0],
		.size = sizeof(cyttsp4_ddata),
		.tag = 0,
	};

	/* Manufacturing Data Table */
	static u8 cyttsp4_mdata[] = {
		65, 64, /* test truncation */63, 62, 61, 60, 59, 58, 57, 56, 55,
		54, 53, 52, 51, 50, 49, 48,
		47, 46, 45, 44, 43, 42, 41, 40, 39, 38, 37, 36, 35, 34, 33, 32,
		31, 30, 29, 28, 27, 26, 25, 24, 23, 22, 21, 20, 19, 18, 17, 16,
		15, 14, 13, 12, 11, 10, 9, 8, 7, 6, 5, 4, 3, 2, 1, 0
	};

	static struct touch_settings cyttsp4_sett_mdata = {
		.data = (uint8_t *)&cyttsp4_mdata[0],
		.size = sizeof(cyttsp4_mdata),
		.tag = 0,
	};

	static struct touch_firmware cyttsp4_firmware = {
		.img = cyttsp4_img,
		.size = sizeof(cyttsp4_img),
		.ver = cyttsp4_ver,
		.vsize = sizeof(cyttsp4_ver),
	};

	static const uint16_t cyttsp4_abs[] = {
		ABS_MT_POSITION_X, CY_ABS_MIN_X, CY_ABS_MAX_X, 0, 0,
		ABS_MT_POSITION_Y, CY_ABS_MIN_Y, CY_ABS_MAX_Y, 0, 0,
		ABS_MT_PRESSURE, CY_ABS_MIN_P, CY_ABS_MAX_P, 0, 0,
		CY_IGNORE_VALUE, CY_ABS_MIN_W, CY_ABS_MAX_W, 0, 0,
		ABS_MT_TRACKING_ID, CY_ABS_MIN_T, CY_ABS_MAX_T, 0, 0,
	};

	static struct touch_framework cyttsp4_framework_local = {
		.abs = (uint16_t *)&cyttsp4_abs[0],
		.size = sizeof(cyttsp4_abs)/sizeof(uint16_t),
		.enable_vkeys = 0,
	};

	static struct touch_platform_data cyttsp4_spi_touch_platform_data_local = {
		.sett = {
			NULL,   /* Reserved */
			NULL,   /* Command Registers */
			NULL,   /* Touch Report */
			NULL,   /* Cypress Data Record */
			NULL,   /* Test Record */
			NULL,   /* Panel Configuration Record */
			&cyttsp4_sett_param_regs,
			&cyttsp4_sett_param_size,
			NULL,   /* Reserved */
			NULL,   /* Reserved */
			NULL,   /* Operational Configuration Record */
			&cyttsp4_sett_ddata,    /* Design Data Record */
			&cyttsp4_sett_mdata,    /* Manufacturing Data Record */
		},
		.fw = &cyttsp4_firmware,
		.frmwrk = &cyttsp4_framework_local,
		.addr = {0xFF, 0xFF},   /* not used for SPI */
		.flags = 0x00,
		.hw_reset = cyttsp4_hw_reset,
		.hw_recov = cyttsp4_hw_recov,
		.hw_power = cyttsp4_hw_power,
		.irq_stat = cyttsp4_irq_stat,
	};
#endif /* SH_TPSIF_LOCAL_DEVICE_REGIST_ENABLE */

#if !defined(CY_NO_AUTO_LOAD) || \
	defined(CY_USE_FORCE_LOAD) || \
	defined(CONFIG_TOUCHSCREEN_DEBUG) || \
	defined(SH_TPSIF_COMMAND)
static int _cyttsp4_load_app(struct cyttsp4 *ts, const u8 *fw, int fw_size);
#endif /* !CY_NO_AUTO_LOAD || CY_USE_FORCE_LOAD || CONFIG_TOUCHSCREEN_DEBUG || SH_TPSIF_COMMAND */
static int _cyttsp4_ldr_exit(struct cyttsp4 *ts);
static int _cyttsp4_startup(struct cyttsp4 *ts);
static int _cyttsp4_get_ic_crc(struct cyttsp4 *ts,
	enum cyttsp4_ic_ebid ebid, u8 *crc_h, u8 *crc_l);
static int cyttsp4_get_charger_armor_status(struct cyttsp4 *ts, bool *enabled);
static int cyttsp4_set_charger_armor(struct cyttsp4 *ts, bool on);
static irqreturn_t cyttsp4_irq(int irq, void *handle);
static int _cyttsp4_set_mode(struct cyttsp4 *ts, u8 new_mode);
#ifdef CY_USE_TMA884
static int _cyttsp4_calc_data_crc(struct cyttsp4 *ts,
	size_t ndata, u8 *pdata, u8 *crc_h, u8 *crc_l, const char *name);
#endif /* --CY_USE_TMA884 */
#ifdef CONFIG_TOUCHSCREEN_DEBUG
#ifdef CY_USE_TMA400
static int _cyttsp4_store_tch_param_tma400(struct cyttsp4 *ts,
	u8 *ic_buf, size_t length);
static int _cyttsp4_calc_ic_crc_tma400(struct cyttsp4 *ts,
	enum cyttsp4_ic_ebid ebid, u8 *crc_h, u8 *crc_l, bool read_back_verify);
#endif /* --CY_USE_TMA400 */
#endif /* --CONFIG_TOUCHSCREEN_DEBUG */
#ifdef SH_TPSIF_COMMAND
static void _sh_tpsif_adjust_point(struct cyttsp4 *ts, int *x, int *y);
static int _sh_tpsif_poll_start(struct cyttsp4 *ts);
static void _sh_tpsif_poll_stop(struct cyttsp4 *ts);
static void _sh_tpsif_poll_scan(struct work_struct *work);
static enum hrtimer_restart _sh_tpsif_poll_timer_handler(struct hrtimer *timer);
static int sh_tpsif_hw_reset(struct cyttsp4 *ts);
static int sh_tpsif_sw_reset(struct cyttsp4 *ts);
static int sh_tpsif_hw_reset_startup(struct cyttsp4 *ts);
static int sh_tpsif_sw_reset_startup(struct cyttsp4 *ts);
static int _sh_tpsif_calibration_idac(struct cyttsp4 *ts);
static int _sh_tpsif_firmware_version(struct cyttsp4 *ts, u32 *version);
#endif	/* SH_TPSIF_COMMAND */


static void _cyttsp4_pr_state(struct cyttsp4 *ts)
{
	dev_info(ts->dev,
			"%s: %s\n", __func__,
		ts->driver_state < CY_INVALID_STATE ?
		cyttsp4_driver_state_string[ts->driver_state] :
		"INVALID");
}

static void _cyttsp4_pr_buf(struct cyttsp4 *ts, u8 *dptr, int size,
	const char *data_name)
{
#ifdef CONFIG_TOUCHSCREEN_DEBUG
	int i = 0;
	int max = (CY_MAX_PRBUF_SIZE - 1) - sizeof(CY_PR_TRUNCATED);

	if (ts == NULL)
		dev_err(ts->dev,
			"%s: ts=%p\n", __func__, ts);
	else if (ts->pr_buf == NULL)
		dev_err(ts->dev,
			"%s: ts->pr_buf=%p\n", __func__, ts->pr_buf);
	else if (ts->bus_ops->tsdebug >= CY_DBG_LVL_2) {
		ts->pr_buf[0] = 0;
		for (i = 0; i < size && i < max; i++)
			sprintf(ts->pr_buf, "%s %02X", ts->pr_buf, dptr[i]);
		dev_info(ts->dev,
			"%s:  %s[0..%d]=%s%s\n", __func__, data_name, size-1,
			ts->pr_buf, size <= max ? "" : CY_PR_TRUNCATED);
	}
#endif /* --CONFIG_TOUCHSCREEN_DEBUG */
	return;
}

static int _cyttsp4_read_block_data(struct cyttsp4 *ts, u16 command,
	size_t length, void *buf, int i2c_addr, bool use_subaddr)
{
	int retval = 0;
	int tries = 0;

	if ((buf == NULL) || (length == 0)) {
		dev_err(ts->dev,
			"%s: pointer or length error"
			" buf=%p length=%d\n", __func__, buf, length);
		retval = -EINVAL;
	} else {
		for (tries = 0, retval = -1;
			tries < CY_NUM_RETRY && (retval < 0);
			tries++) {
			retval = ts->bus_ops->read(ts->bus_ops, command,
				length, buf, i2c_addr, use_subaddr);
			if (retval < 0) {
				msleep(CY_DELAY_DFLT);
				/*
				 * TODO: remove the extra sleep delay when
				 * the loader exit sequence is streamlined
				  */
				msleep(150);
			}
		}

		if (retval < 0) {
			dev_err(ts->dev,
			"%s: bus read block data fail (ret=%d)\n",
				__func__, retval);
		}
	}

	return retval;
}

static int _cyttsp4_write_block_data(struct cyttsp4 *ts, u16 command,
	size_t length, const void *buf, int i2c_addr, bool use_subaddr)
{
	int retval = 0;
	int tries = 0;

	if ((buf == NULL) || (length == 0)) {
		dev_err(ts->dev,
			"%s: pointer or length error"
			" buf=%p length=%d\n", __func__, buf, length);
		retval = -EINVAL;
	} else {
		for (tries = 0, retval = -1;
			tries < CY_NUM_RETRY && (retval < 0);
			tries++) {
			retval = ts->bus_ops->write(ts->bus_ops, command,
				length, buf, i2c_addr, use_subaddr);
			if (retval < 0)
				msleep(CY_DELAY_DFLT);
		}

		if (retval < 0) {
			dev_err(ts->dev,
			"%s: bus write block data fail (ret=%d)\n",
				__func__, retval);
		}
	}

	return retval;
}

#ifdef CY_USE_TMA400
static int _cyttsp4_wait_ready_int_no_init(struct cyttsp4 *ts,
	unsigned long timeout_ms)
{
	unsigned long uretval;
	int retval = 0;

	mutex_unlock(&ts->data_lock);
	uretval = wait_for_completion_interruptible_timeout(
		&ts->ready_int_running, msecs_to_jiffies(timeout_ms));
	mutex_lock(&ts->data_lock);
	if (uretval == 0) {
		dev_err(ts->dev,
			"%s: timeout waiting for interrupt\n",
			__func__);
		retval = -ETIMEDOUT;
	}

	return retval;
}
#endif /* --CY_USE_TMA400 */

static int _cyttsp4_wait_int(struct cyttsp4 *ts, unsigned long timeout_ms)
{
	unsigned long uretval;
	int retval = 0;

	INIT_COMPLETION(ts->int_running);
	mutex_unlock(&ts->data_lock);
	uretval = wait_for_completion_interruptible_timeout(
		&ts->int_running, msecs_to_jiffies(timeout_ms));
	mutex_lock(&ts->data_lock);
	if (uretval == 0) {
		dev_err(ts->dev,
			"%s: timeout waiting for interrupt\n",
			__func__);
		retval = -ETIMEDOUT;
	}

	return retval;
}

static int _cyttsp4_wait_si_int(struct cyttsp4 *ts, unsigned long timeout_ms)
{
	unsigned long uretval;
	int retval = 0;

	mutex_unlock(&ts->data_lock);
	uretval = wait_for_completion_interruptible_timeout(
		&ts->si_int_running, msecs_to_jiffies(timeout_ms));
	mutex_lock(&ts->data_lock);
	if (uretval == 0) {
		dev_err(ts->dev,
			"%s: timeout waiting for bootloader interrupt\n",
			__func__);
		retval = -ETIMEDOUT;
	}

	return retval;
}

static void _cyttsp4_queue_startup(struct cyttsp4 *ts, bool was_suspended)
{
	ts->was_suspended = was_suspended;
	queue_work(ts->cyttsp4_wq,
		&ts->cyttsp4_resume_startup_work);
	dev_info(ts->dev,
			"%s: startup queued\n", __func__);
}

static u16 _cyttsp4_calc_partial_crc(struct cyttsp4 *ts,
	u8 *pdata, size_t ndata, u16 crc)
{
	int i = 0;
	int j = 0;

	for (i = 0; i < ndata; i++) {
		crc ^= ((u16)pdata[i] << 8);

		for (j = 8; j > 0; --j) {
			if (crc & 0x8000)
				crc = (crc << 1) ^ 0x1021;
			else
				crc = crc << 1;
		}
	}

	return crc;
}

static void _cyttsp4_calc_crc(struct cyttsp4 *ts,
	u8 *pdata, size_t ndata, u8 *crc_h, u8 *crc_l)
{
	u16 crc = 0;

	if (pdata == NULL)
		dev_err(ts->dev,
			"%s: Null data ptr\n", __func__);
	else if (ndata == 0)
		dev_err(ts->dev,
			"%s: Num data is 0\n", __func__);
	else {
		/* Calculate CRC */
		crc = 0xFFFF;
		crc = _cyttsp4_calc_partial_crc(ts, pdata, ndata, crc);
		*crc_h = crc / 256;
		*crc_l = crc % 256;
	}
}

static bool _cyttsp4_chk_cmd_rdy(struct cyttsp4 *ts, u8 cmd)
{
	bool cond = !!(cmd & CY_CMD_RDY_BIT);
	dev_vdbg(ts->dev,
		"%s: cmd=%02X cond=%d\n", __func__, cmd, (int)cond);

	return cond;
}

static bool _cyttsp4_chk_mode_change(struct cyttsp4 *ts, u8 cmd)
{
	bool cond = !(cmd & CY_MODE_CHANGE);
	dev_vdbg(ts->dev,
		"%s: cmd=%02X cond=%d\n", __func__, cmd, (int)cond);

	return cond;
}

static void _cyttsp4_change_state(struct cyttsp4 *ts,
	enum cyttsp4_driver_state new_state)
{
	ts->driver_state = new_state;
	_cyttsp4_pr_state(ts);
}

static int _cyttsp4_put_cmd_wait(struct cyttsp4 *ts, u16 ofs,
	size_t cmd_len, const void *cmd_buf, unsigned long timeout_ms,
	bool (*cond)(struct cyttsp4 *, u8), u8 *retcmd,
	int i2c_addr, bool use_subaddr)
{
	enum cyttsp4_driver_state tmp_state;
	unsigned long uretval = 0;
	u8 cmd = 0;
	int tries = 0;
	int retval = 0;

	/* unlock here to allow any pending irq to complete */
	tmp_state = ts->driver_state;
	_cyttsp4_change_state(ts, CY_TRANSFER_STATE);
	mutex_unlock(&ts->data_lock);
	mutex_lock(&ts->data_lock);
	_cyttsp4_change_state(ts, CY_CMD_STATE);
	INIT_COMPLETION(ts->int_running);
	mutex_unlock(&ts->data_lock);
	retval = _cyttsp4_write_block_data(ts, ofs, cmd_len,
		cmd_buf, i2c_addr, use_subaddr);
	if (retval < 0) {
		dev_err(ts->dev,
			"%s: Fail writing cmd buf r=%d\n",
			__func__, retval);
		mutex_lock(&ts->data_lock);
		goto _cyttsp4_put_cmd_wait_exit;
	}
_cyttsp4_put_cmd_wait_retry:
	uretval = wait_for_completion_interruptible_timeout(
		&ts->int_running, msecs_to_jiffies(timeout_ms));
	mutex_lock(&ts->data_lock);

	retval = _cyttsp4_read_block_data(ts, ofs,
		sizeof(cmd), &cmd, i2c_addr, use_subaddr);
	if (retval < 0) {
		dev_err(ts->dev,
			"%s: fail read cmd status  r=%d\n",
			__func__, retval);
	}
	if ((cond != NULL) && !cond(ts, cmd)) {
		if (uretval == 0) {
			dev_err(ts->dev,
			"%s: timeout waiting for cmd ready\n",
				__func__);
			retval = -ETIMEDOUT;
		} else {
			if (tries++ < 2) {
				INIT_COMPLETION(ts->int_running);
				mutex_unlock(&ts->data_lock);
				goto _cyttsp4_put_cmd_wait_retry;
			} else {
				dev_err(ts->dev,
			"%s: cmd not ready error"
					" cmd_stat=0x%02X\n",
					__func__, cmd);
				retval = -EIO;
			}
		}
	} else {
		/* got command ready */
		if (retcmd != NULL)
			*retcmd = cmd;
		retval = 0;
		dev_vdbg(ts->dev,
			"%s: got command ready; cmd=%02X retcmd=%p tries=%d\n",
			__func__, cmd, retcmd, tries);
	}

_cyttsp4_put_cmd_wait_exit:
	_cyttsp4_change_state(ts, tmp_state);
	return retval;
}

static int _cyttsp4_handshake(struct cyttsp4 *ts, u8 hst_mode)
{
	int retval = 0;
	u8 cmd = 0;

	cmd = hst_mode & CY_HANDSHAKE_BIT ?
		hst_mode & ~CY_HANDSHAKE_BIT :
		hst_mode | CY_HANDSHAKE_BIT;

	retval = _cyttsp4_write_block_data(ts, CY_REG_BASE,
		sizeof(cmd), (u8 *)&cmd,
		ts->platform_data->addr[CY_TCH_ADDR_OFS], true);

	if (retval < 0) {
		dev_err(ts->dev,
			"%s: bus write fail on handshake (ret=%d)\n",
			__func__, retval);
	}

	return retval;
}

#ifdef CY_USE_TMA400
static void _cyttsp_read_table_crc(struct cyttsp4 *ts, const u8 *ptable,
	u8 *crc_h, u8 *crc_l)
{
	size_t crc_loc = (ptable[3] * 256) + ptable[2];

	*crc_h = ptable[crc_loc];
	*crc_l = ptable[crc_loc + 1];
}

static int _cyttsp4_cmd_handshake(struct cyttsp4 *ts)
{
	u8 host_mode = 0;
	int retval = 0;

	retval = _cyttsp4_read_block_data(ts, CY_REG_BASE,
		sizeof(host_mode), &host_mode,
		ts->platform_data->addr[CY_TCH_ADDR_OFS], true);
	if (retval < 0) {
		dev_err(ts->dev,
			"%s: Fail read host mode r=%d\n",
			__func__, retval);
	} else {
		retval = _cyttsp4_handshake(ts, host_mode);
		if (retval < 0) {
			dev_err(ts->dev,
			"%s: Fail handshake r=%d\n",
				__func__, retval);
		}
	}

	return retval;
}

/* Get EBID Row Size is a Config mode command */
static int _cyttsp4_get_ebid_row_size(struct cyttsp4 *ts)
{
	int retval = 0;
	u8 cmd = 0;
	u8 cmd_dat[CY_NUM_DAT + 1];	/* +1 for cmd byte */

	memset(cmd_dat, 0, sizeof(cmd_dat));
	cmd_dat[0] = CY_GET_EBID_ROW_SIZE;	/* get EBID row size command */

	retval = _cyttsp4_put_cmd_wait(ts, ts->si_ofs.cmd_ofs,
		sizeof(cmd_dat), cmd_dat, CY_HALF_SEC_TMO_MS,
		_cyttsp4_chk_cmd_rdy, &cmd,
		ts->platform_data->addr[CY_TCH_ADDR_OFS], true);
	if (retval < 0) {
		dev_err(ts->dev,
			"%s: Fail Get EBID row size command r=%d\n",
			__func__, retval);
	} else {
		retval = _cyttsp4_read_block_data(ts, ts->si_ofs.cmd_ofs,
			sizeof(cmd_dat), cmd_dat,
			ts->platform_data->addr[CY_TCH_ADDR_OFS], true);
		if (retval < 0) {
			dev_err(ts->dev,
			"%s: Fail get EBID row size r=%d\n",
				__func__, retval);
			ts->ebid_row_size = CY_EBID_ROW_SIZE_DFLT;
			dev_err(ts->dev,
			"%s: Use default EBID row size=%d\n",
				__func__, ts->ebid_row_size);
		} else {
			ts->ebid_row_size = (cmd_dat[1] * 256) + cmd_dat[2];
			retval = _cyttsp4_cmd_handshake(ts);
			if (retval < 0) {
				dev_err(ts->dev,
			"%s: Command handshake error r=%d\n",
					__func__, retval);
				/* continue anyway; rely on handshake tmo */
				retval = 0;
			}
		}
	}

	return retval;
}

/* Get EBID Row Data is a Config mode command */
static int _cyttsp4_get_ebid_data_tma400(struct cyttsp4 *ts,
	enum cyttsp4_ic_ebid ebid, size_t row_id, u8 *pdata)
{
	int rc = 0;
	int retval = 0;
	u8 crc_h = 0;
	u8 crc_l = 0;
	u8 cmd = 0;
	u8 status = 0;
	u8 cmd_dat[CY_NUM_DAT + 1];	/* +1 for cmd byte */

	memset(cmd_dat, 0, sizeof(cmd_dat));
	cmd_dat[0] = CY_READ_EBID_DATA;	/* get EBID data command */
	cmd_dat[1] = row_id / 256;
	cmd_dat[2] = row_id % 256;
	cmd_dat[3] = ts->ebid_row_size / 256;
	cmd_dat[4] = ts->ebid_row_size % 256;
	cmd_dat[5] = ebid;

	if (pdata == NULL) {
		dev_err(ts->dev,
			"%s: Get EBID=%d row=%d Data buffer err ptr=%p\n",
			__func__, ebid, row_id, pdata);
		goto _cyttsp4_get_ebid_data_tma400_exit;
	}

	retval = _cyttsp4_put_cmd_wait(ts, ts->si_ofs.cmd_ofs,
		sizeof(cmd_dat), cmd_dat, CY_HALF_SEC_TMO_MS,
		_cyttsp4_chk_cmd_rdy, &cmd,
		ts->platform_data->addr[CY_TCH_ADDR_OFS], true);
	if (retval < 0) {
		dev_err(ts->dev,
			"%s: Fail Get EBID=%d row=%d Data cmd r=%d\n",
			__func__, ebid, row_id, retval);
		goto _cyttsp4_get_ebid_data_tma400_exit;
	}

	retval = _cyttsp4_read_block_data(ts, ts->si_ofs.cmd_ofs + 1,
		sizeof(status), &status,
		ts->platform_data->addr[CY_TCH_ADDR_OFS], true);
	if (retval < 0) {
		dev_err(ts->dev,
			"%s: Fail get EBID=%d row=%d status r=%d\n",
			__func__, ebid, row_id, retval);
		goto _cyttsp4_get_ebid_data_tma400_exit;
	}

	if (status != 0x00) {
		dev_err(ts->dev,
			"%s: Get EBID=%d row=%d status=%d error\n",
			__func__, ebid, row_id, status);
		retval = -EIO;
		goto _cyttsp4_get_ebid_data_tma400_exit;
	}

	retval = _cyttsp4_read_block_data(ts, ts->si_ofs.cmd_ofs + 1 + 5,
		ts->ebid_row_size + 2, pdata,
		ts->platform_data->addr[CY_TCH_ADDR_OFS], true);
	if (retval < 0) {
		dev_err(ts->dev,
			"%s: fail EBID=%d row=%d data r=%d\n",
			__func__, ebid, row_id, retval);
		retval = -EIO;
	} else {
		_cyttsp4_calc_crc(ts, pdata, ts->ebid_row_size, &crc_h, &crc_l);
		if (pdata[ts->ebid_row_size] != crc_h ||
			pdata[ts->ebid_row_size + 1] != crc_l) {
			dev_err(ts->dev,
			"%s: EBID=%d row_id=%d row_data_crc=%02X%02X"
				" not equal to calc_crc=%02X%02X\n",
				__func__, ebid, row_id,
				pdata[ts->ebid_row_size],
				pdata[ts->ebid_row_size + 1],
				crc_h, crc_l);
			/* continue anyway; allow handshake */
			rc = -EIO;
		}
		retval = _cyttsp4_cmd_handshake(ts);
		if (retval < 0) {
			dev_err(ts->dev,
			"%s: Command handshake error r=%d\n",
				__func__, retval);
			/* continue anyway; rely on handshake tmo */
			retval = 0;
		}
		retval = rc;
	}

_cyttsp4_get_ebid_data_tma400_exit:
	return retval;
}

static const u8 cyttsp4_security_key[] = {
	0xA5, 0x01, 0x02, 0x03, 0xFF, 0xFE, 0xFD, 0x5A
};

/* Put EBID Row Data is a Config mode command */
static int _cyttsp4_put_ebid_data_tma400(struct cyttsp4 *ts,
	enum cyttsp4_ic_ebid ebid, size_t row_id, u8 *out_data)
{
	u8 calc_crc[2];
	u8 *pdata = NULL;
	u8 ret_cmd = 0;
	size_t psize = 0;
	u8 status = 0;
	int retval = 0;

	memset(calc_crc, 0, sizeof(calc_crc));
	psize = 1 + 5 + ts->ebid_row_size + sizeof(cyttsp4_security_key) + 2;
	pdata = kzalloc(psize, GFP_KERNEL);
	if (pdata == NULL || out_data == NULL) {
		dev_err(ts->dev,
			"%s: Buffer ptr err EBID=%d row=%d"
			" alloc_ptr=%p out_data=%p\n",
			__func__, ebid, row_id, pdata, out_data);
		retval = -EINVAL;
	} else {
		pdata[0] = CY_WRITE_EBID_DATA;	/* put ebid data command */
		pdata[1] = row_id / 256;
		pdata[2] = row_id % 256;
		pdata[3] = ts->ebid_row_size / 256;
		pdata[4] = ts->ebid_row_size % 256;
		pdata[5] = ebid;
		memcpy(&pdata[1 + 5], out_data, ts->ebid_row_size);
		memcpy(&pdata[1 + 5 + ts->ebid_row_size],
			cyttsp4_security_key, sizeof(cyttsp4_security_key));
		_cyttsp4_calc_crc(ts, &pdata[1 + 5], ts->ebid_row_size,
			&calc_crc[0], &calc_crc[1]);
		memcpy(&pdata[1 + 5 + ts->ebid_row_size +
			sizeof(cyttsp4_security_key)],
			calc_crc, sizeof(calc_crc));

		retval = _cyttsp4_put_cmd_wait(ts, ts->si_ofs.cmd_ofs,
			psize, pdata, CY_HALF_SEC_TMO_MS,
			_cyttsp4_chk_cmd_rdy, &ret_cmd,
			ts->platform_data->addr[CY_TCH_ADDR_OFS], true);
		if (retval < 0) {
			dev_err(ts->dev,
			"%s: Fail Put EBID=%d row=%d Data cmd r=%d\n",
				__func__, ebid, row_id, retval);
			goto _cyttsp4_put_ebid_data_tma400_exit;
		}

		retval = _cyttsp4_read_block_data(ts, ts->si_ofs.cmd_ofs + 1,
			sizeof(status), &status,
			ts->platform_data->addr[CY_TCH_ADDR_OFS], true);
		if (retval < 0) {
			dev_err(ts->dev,
			"%s: Fail put EBID=%d row=%d"
				" read status r=%d\n",
				__func__, ebid, row_id, retval);
			goto _cyttsp4_put_ebid_data_tma400_exit;
		}

		retval = _cyttsp4_cmd_handshake(ts);
		if (retval < 0) {
			dev_err(ts->dev,
			"%s: Fail handshake on Put EBID=%d row=%d"
				" r=%d\n", __func__, ebid, row_id, retval);
			/* continue; rely on handshake tmo */
			retval = 0;
		}

		if (status != 0x00) {
			dev_err(ts->dev,
			"%s: Put EBID=%d row=%d status=%d error\n",
				__func__, ebid, row_id, status);
			retval = -EIO;
		} else
			retval = 0;
	}
_cyttsp4_put_ebid_data_tma400_exit:
	if (pdata != NULL)
		kfree(pdata);
	return retval;
}

/* Put All Touch Params is a Config mode command */
static int _cyttsp4_put_all_params_tma400(struct cyttsp4 *ts)
{
	enum cyttsp4_ic_ebid ebid = CY_TCH_PARM_EBID;
	size_t row_id = 0;
	size_t num_rows = 0;
	size_t table_size = 0;
	size_t residue = 0;
	u8 *pdata = NULL;
	u8 *ptable = NULL;
	int retval = 0;

	pdata = kzalloc(ts->ebid_row_size, GFP_KERNEL);
	if (pdata == NULL) {
		dev_err(ts->dev,
			"%s: Alloc error ebid=%d\n",
			__func__, ebid);
		retval = -ENOMEM;
	} else if (ts->platform_data->sett[CY_IC_GRPNUM_TCH_PARM_VAL] == NULL)
		dev_err(ts->dev,
			"%s: NULL param values table\n", __func__);
	else if (ts->platform_data->sett[CY_IC_GRPNUM_TCH_PARM_VAL]
		->data == NULL)
		dev_err(ts->dev,
			"%s: NULL param values table data\n", __func__);
	else if (ts->platform_data->sett[CY_IC_GRPNUM_TCH_PARM_VAL]->size == 0)
		dev_err(ts->dev,
			"%s: param values table size is 0\n", __func__);
	else {
		ptable = (u8 *)ts->platform_data->sett
			[CY_IC_GRPNUM_TCH_PARM_VAL]->data;
		table_size = ts->platform_data->sett
			[CY_IC_GRPNUM_TCH_PARM_VAL]->size;
		num_rows = table_size / ts->ebid_row_size;
		dev_vdbg(ts->dev,
			"%s: num_rows=%d row_size=%d"
			" table_size=%d\n", __func__,
			num_rows, ts->ebid_row_size, table_size);
		for (row_id = 0; row_id < num_rows;) {
			memcpy(pdata, ptable, ts->ebid_row_size);
			dev_vdbg(ts->dev,
				"%s: row=%d pdata=%p\n",
				__func__, row_id, pdata);
			_cyttsp4_pr_buf(ts, pdata, ts->ebid_row_size,
				"ebid_data");
			retval = _cyttsp4_put_ebid_data_tma400(ts,
				ebid, row_id, pdata);
			if (retval < 0) {
				dev_err(ts->dev,
			"%s: Fail put row=%d r=%d\n",
					__func__, row_id, retval);
				break;
			} else {
				ptable += ts->ebid_row_size;
				row_id++;
			}
		}
		if (!(retval < 0)) {
			residue = table_size % ts->ebid_row_size;
			if (residue) {
				memset(pdata, 0, ts->ebid_row_size);
				memcpy(pdata, ptable, residue);
				dev_vdbg(ts->dev,
					"%s: ebid=%d row=%d data:\n",
					__func__, ebid, row_id);
				_cyttsp4_pr_buf(ts, pdata, ts->ebid_row_size,
					"ebid_data");
				retval = _cyttsp4_put_ebid_data_tma400(ts,
					ebid, row_id, pdata);
				if (retval < 0) {
					dev_err(ts->dev,
			"%s: Fail put row=%d r=%d\n",
						__func__, row_id, retval);
				}
			}
		}
	}

	return retval;
}

#if defined(STARTUP_MDDATA_CHECK) ||		   \
	(defined(CONFIG_TOUCHSCREEN_DEBUG) &&	   \
		defined(CY_USE_DEBUG_TOOLS) &&	   \
		defined(CY_USE_DEV_DEBUG_TOOLS) && \
		defined(CY_USE_REG_ACCESS))
/* Check MDDATA is a Config mode command */
static int _cyttsp4_check_mddata_tma400(struct cyttsp4 *ts, bool *updated)
{
	enum cyttsp4_ic_ebid ebid = CY_DDATA_EBID;
	bool ddata_updated = false;
	bool mdata_updated = false;
	size_t num_data = 0;
	size_t crc_ofs = 0;
	u8 crc_h = 0;
	u8 crc_l = 0;
	u8 *pdata = NULL;
	u8 *pmddata = NULL;
	int retval = 0;

	if (ts->ebid_row_size == 0) {
		dev_err(ts->dev,
			"%s: fail allocate set MDDATA buffer\n", __func__);
		retval = -EINVAL;
		goto _cyttsp4_check_mddata_tma400_exit;
	}
	pdata = kzalloc(ts->ebid_row_size, GFP_KERNEL);
	if (pdata == NULL) {
		dev_err(ts->dev,
			"%s: fail allocate set MDDATA buffer\n", __func__);
		retval = -ENOMEM;
		goto _cyttsp4_check_mddata_tma400_exit;
	}
	pmddata = kzalloc(ts->ebid_row_size, GFP_KERNEL);
	if (pmddata == NULL) {
		dev_err(ts->dev,
			"%s: fail allocate set MDDATA buffer\n", __func__);
		retval = -ENOMEM;
		goto _cyttsp4_check_mddata_tma400_exit;
	}

	/* check for platform_data DDATA */
	ebid = CY_DDATA_EBID;
	if (ts->platform_data->sett[CY_IC_GRPNUM_DDATA_REC] == NULL) {
		dev_vdbg(ts->dev,
			"%s: No platform DDATA table\n", __func__);
		goto _cyttsp4_check_mdata_block;
	}
	if (ts->platform_data->sett[CY_IC_GRPNUM_DDATA_REC]->data == NULL) {
		dev_vdbg(ts->dev,
			"%s: No platform DDATA table data\n", __func__);
		goto _cyttsp4_check_mdata_block;
	}
	if (ts->platform_data->sett[CY_IC_GRPNUM_DDATA_REC]->size == 0) {
		dev_vdbg(ts->dev,
			"%s: Platform DDATA table has size=0\n", __func__);
		goto _cyttsp4_check_mdata_block;
	}

	dev_vdbg(ts->dev,
		"%s: call get ebid data for DDATA\n", __func__);
	retval = _cyttsp4_get_ebid_data_tma400(ts, ebid, 0, pdata);
	if (retval < 0) {
		dev_err(ts->dev,
			"%s: Fail get DDATA r=%d\n", __func__, retval);
		goto _cyttsp4_check_mdata_block;
	}

	dev_vdbg(ts->dev,
		"%s: copy pdata -> pmddata\n", __func__);
	memcpy(pmddata, pdata, 4);
	num_data = ts->platform_data->sett
		[CY_IC_GRPNUM_DDATA_REC]->size < CY_NUM_DDATA ?
		ts->platform_data->sett
		[CY_IC_GRPNUM_DDATA_REC]->size : CY_NUM_DDATA;
	dev_vdbg(ts->dev,
		"%s: copy %d bytes from platform data to ddata array\n",
		__func__, num_data);
	memcpy(&pmddata[4], ts->platform_data->sett
		[CY_IC_GRPNUM_DDATA_REC]->data, num_data);
	if (num_data < CY_NUM_DDATA)
		memset(&pmddata[4 + num_data], 0, CY_NUM_DDATA - num_data);
	crc_ofs = (pmddata[3] * 256) + pmddata[2];
	if (crc_ofs == 0)
		crc_ofs = 126;
	dev_vdbg(ts->dev,
		"%s: ddata crc_ofs=%d num_data=%d\n",
		__func__, crc_ofs, num_data);

	_cyttsp4_calc_crc(ts, pmddata, crc_ofs, &crc_h, &crc_l);
	pmddata[crc_ofs] = crc_l;
	pmddata[crc_ofs+1] = crc_h;
	_cyttsp4_pr_buf(ts, pdata, ts->ebid_row_size, "pdata");
	_cyttsp4_pr_buf(ts, pmddata, ts->ebid_row_size, "pmddata");
	if (pmddata[crc_ofs] != pdata[crc_ofs] ||
		pmddata[crc_ofs+1] != pdata[crc_ofs+1]) {
		retval = _cyttsp4_put_ebid_data_tma400(ts, ebid, 0, pmddata);
		if (retval < 0)
			dev_err(ts->dev,
			"%s: Fail put DDATA r=%d\n", __func__, retval);
		else
			ddata_updated = true;
	}

_cyttsp4_check_mdata_block:
	/* check for platform_data MDATA */
	memset(pdata, 0, ts->ebid_row_size);
	memset(pmddata, 0, ts->ebid_row_size);
	ebid = CY_MDATA_EBID;
	if (ts->platform_data->sett[CY_IC_GRPNUM_MDATA_REC] == NULL) {
		dev_vdbg(ts->dev,
			"%s: No platform MDATA table\n", __func__);
		goto _cyttsp4_check_mddata_tma400_exit;
	}
	if (ts->platform_data->sett[CY_IC_GRPNUM_MDATA_REC]->data == NULL) {
		dev_vdbg(ts->dev,
			"%s: No platform MDATA table data\n", __func__);
		goto _cyttsp4_check_mddata_tma400_exit;
	}
	if (ts->platform_data->sett[CY_IC_GRPNUM_MDATA_REC]->size == 0) {
		dev_vdbg(ts->dev,
			"%s: Platform MDATA table has size=0\n", __func__);
		goto _cyttsp4_check_mddata_tma400_exit;
	}

	retval = _cyttsp4_get_ebid_data_tma400(ts, ebid, 0, pdata);
	if (retval < 0) {
		dev_err(ts->dev,
			"%s: Fail get MDATA r=%d\n", __func__, retval);
		goto _cyttsp4_check_mddata_tma400_exit;
	}

	memcpy(pmddata, pdata, 4);
	num_data = ts->platform_data->sett
		[CY_IC_GRPNUM_MDATA_REC]->size < CY_NUM_MDATA ?
		ts->platform_data->sett
		[CY_IC_GRPNUM_MDATA_REC]->size : CY_NUM_MDATA;
	dev_vdbg(ts->dev,
		"%s: copy %d bytes from platform data to mdata array\n",
		__func__, num_data);
	memcpy(&pmddata[4], ts->platform_data->sett
		[CY_IC_GRPNUM_MDATA_REC]->data, num_data);
	if (num_data < CY_NUM_MDATA)
		memset(&pmddata[4 + num_data], 0, CY_NUM_MDATA - num_data);
	crc_ofs = (pmddata[3] * 256) + pmddata[2];
	if (crc_ofs == 0)
		crc_ofs = 124;
	dev_vdbg(ts->dev,
		"%s: mdata crc_ofs=%d num_data=%d\n",
		__func__, crc_ofs, num_data);
	_cyttsp4_calc_crc(ts, pmddata, crc_ofs, &crc_h, &crc_l);
	pmddata[crc_ofs] = crc_l;
	pmddata[crc_ofs+1] = crc_h;
	_cyttsp4_pr_buf(ts, pdata, ts->ebid_row_size, "pdata");
	_cyttsp4_pr_buf(ts, pmddata, ts->ebid_row_size, "pmddata");
	if (pmddata[crc_ofs] != pdata[crc_ofs] ||
		pmddata[crc_ofs+1] != pdata[crc_ofs+1]) {
		retval = _cyttsp4_put_ebid_data_tma400(ts, ebid, 0, pmddata);
		if (retval < 0)
			dev_err(ts->dev,
			"%s: Fail put MDATA r=%d\n", __func__, retval);
		else
			mdata_updated = true;
	}

_cyttsp4_check_mddata_tma400_exit:
	if (pdata != NULL)
		kfree(pdata);
	if (pmddata != NULL)
		kfree(pmddata);
	if (updated != NULL)
		*updated = ddata_updated || mdata_updated;
	return retval;
}
#endif	/* defined(STARTUP_MDDATA_CHECK) ||		\
	   (defined(CONFIG_TOUCHSCREEN_DEBUG) &&	\
	   defined(CY_USE_DEBUG_TOOLS) &&		\
	   defined(CY_USE_DEV_DEBUG_TOOLS) &&		\
	   defined(CY_USE_REG_ACCESS)) */
#endif /* --CY_USE_TMA400 */

#ifdef CY_USE_TMA884
static int _cyttsp4_handshake_enable(struct cyttsp4 *ts)
{
	int retval = 0;
	u8 cmd_dat[CY_NUM_DAT + 1];	/* +1 for cmd byte */

	memset(cmd_dat, 0, sizeof(cmd_dat));
	cmd_dat[0] = 0x26;	/* handshake enable operational cmd */
	cmd_dat[1] = 0x03;	/* synchronous level handshake */
	retval = _cyttsp4_put_cmd_wait(ts, ts->si_ofs.cmd_ofs,
		sizeof(cmd_dat), cmd_dat, CY_HALF_SEC_TMO_MS,
		_cyttsp4_chk_cmd_rdy, NULL,
		ts->platform_data->addr[CY_TCH_ADDR_OFS], true);
	if (retval < 0) {
		dev_err(ts->dev,
			"%s: Fail Enable Handshake command r=%d\n",
			__func__, retval);
		goto _cyttsp4_set_handshake_enable_exit;
	}

	retval = _cyttsp4_read_block_data(ts, ts->si_ofs.cmd_ofs,
		sizeof(cmd_dat), cmd_dat,
		ts->platform_data->addr[CY_TCH_ADDR_OFS], true);
	if (retval < 0) {
		dev_err(ts->dev,
			"%s: Fail read Enable Hanshake command status"
			"r=%d\n", __func__, retval);
		goto _cyttsp4_set_handshake_enable_exit;
	}

	if (cmd_dat[6] != cmd_dat[1]) {
		dev_err(ts->dev,
			"%s: Fail enable handshake in device\n",
			__func__);
		/* return no error and let driver handshake anyway */
	}

	dev_vdbg(ts->dev,
		"%s: check cmd ready r=%d"
		" cmd[]=%02X %02X %02X %02X %02X %02X %02X\n",
		__func__, retval,
		cmd_dat[0], cmd_dat[1], cmd_dat[2], cmd_dat[3],
		cmd_dat[4], cmd_dat[5], cmd_dat[6]);

_cyttsp4_set_handshake_enable_exit:
	return retval;
}
#endif /* --CY_USE_TMA884 */

/*
 * change device mode - For example, change from
 * system information mode to operating mode
 */
static int _cyttsp4_set_device_mode(struct cyttsp4 *ts,
	u8 new_mode, u8 new_cur_mode, char *mode)
{
	u8 cmd = 0;
	int retval = 0;

	cmd = new_mode + CY_MODE_CHANGE;

	retval = _cyttsp4_put_cmd_wait(ts, CY_REG_BASE,
		sizeof(cmd), &cmd, CY_TEN_SEC_TMO_MS,
		_cyttsp4_chk_mode_change, &cmd,
		ts->platform_data->addr[CY_TCH_ADDR_OFS], true);
	if (retval < 0) {
		dev_err(ts->dev,
			"%s: Fail Set mode command new_mode=%02X r=%d\n",
			__func__, new_mode, retval);
		goto _cyttsp4_set_device_mode_exit;
	}

	if (cmd != new_mode) {
		dev_err(ts->dev,
			"%s: failed to switch to %s mode\n", __func__, mode);
		retval = -EIO;
	} else {
		ts->current_mode = new_cur_mode;
		retval = _cyttsp4_handshake(ts, cmd);
		if (retval < 0) {
			dev_err(ts->dev,
			"%s: Fail handshake r=%d\n", __func__, retval);
			/* continue; rely on handshake tmo */
			retval = 0;
		}
	}

	dev_dbg(ts->dev,
		"%s: check op ready ret=%d host_mode=%02X\n",
		__func__, retval, cmd);

_cyttsp4_set_device_mode_exit:
	return retval;
}

static int _cyttsp4_set_mode(struct cyttsp4 *ts, u8 new_mode)
{
	enum cyttsp4_driver_state new_state = CY_TRANSFER_STATE;
	u8 new_cur_mode = CY_MODE_OPERATIONAL;
	char *mode = NULL;
#ifdef CY_USE_TMA400
	unsigned long uretval = 0;
#endif /* --CY_USE_TMA400 */
	int retval = 0;

	switch (new_mode) {
	case CY_OPERATE_MODE:
		new_cur_mode = CY_MODE_OPERATIONAL;
		mode = "operational";
		INIT_COMPLETION(ts->ready_int_running);
		_cyttsp4_change_state(ts, CY_READY_STATE);
		new_state = CY_ACTIVE_STATE;
		break;
	case CY_SYSINFO_MODE:
		new_cur_mode = CY_MODE_SYSINFO;
		mode = "sysinfo";
		new_state = CY_SYSINFO_STATE;
		break;
	case CY_CONFIG_MODE:
		new_cur_mode = CY_MODE_OPERATIONAL;
		mode = "config";
		new_state = ts->driver_state;

		break;
	default:
		dev_err(ts->dev,
			"%s: invalid mode change request m=0x%02X\n",
			__func__, new_mode);
		retval = -EINVAL;
		goto _cyttsp_set_mode_exit;
	}

	retval = _cyttsp4_set_device_mode(ts,
		new_mode, new_cur_mode, mode);
	if (retval < 0) {
		dev_err(ts->dev,
			"%s: Fail switch to %s mode\n", __func__, mode);
		_cyttsp4_change_state(ts, CY_IDLE_STATE);
	} else {
#ifdef CY_USE_TMA400
		if (new_mode == CY_OPERATE_MODE) {
			uretval = _cyttsp4_wait_ready_int_no_init(ts,
				CY_HALF_SEC_TMO_MS * 5);
		}
#endif /* --CY_USE_TMA400 */
		_cyttsp4_change_state(ts, new_state);
	}

_cyttsp_set_mode_exit:
	return retval;
}

#ifdef CY_USE_TMA884
static int _cyttsp4_write_config_block(struct cyttsp4 *ts, u8 blockid,
	const u8 *pdata, size_t ndata, u8 crc_h, u8 crc_l, const char *name)
{
	uint8_t *buf = NULL;
	size_t buf_size = 0;
	u8 status = 0;
	int retval = 0;

	/* pre-amble (10) + data (122) + crc (2) + key (8) */
	buf_size = sizeof(uint8_t) * 142;
	buf = kzalloc(buf_size, GFP_KERNEL);
	if (buf == NULL) {
		dev_err(ts->dev,
			"%s: Failed to allocate buffer for %s\n",
			__func__, name);
		retval = -ENOMEM;
		goto _cyttsp4_write_config_block_exit;
	}

	if (pdata == NULL) {
		dev_err(ts->dev,
			"%s: bad data pointer\n", __func__);
		retval = -ENXIO;
		goto _cyttsp4_write_config_block_exit;
	}

	if (ndata > 122) {
		dev_err(ts->dev,
			"%s: %s is too large n=%d size=%d\n",
			__func__, name, ndata, 122);
		retval = -EOVERFLOW;
		goto _cyttsp4_write_config_block_exit;
	}

	/* Set command bytes */
	buf[0] = 0x04; /* cmd */
	buf[1] = 0x00; /* row offset high */
	buf[2] = 0x00; /* row offset low */
	buf[3] = 0x00; /* write block length high */
	buf[4] = 0x80; /* write block length low */
	buf[5] = blockid; /* write block id */
	buf[6] = 0x00; /* num of config bytes + 4 high */
	buf[7] = 0x7E; /* num of config bytes + 4 low */
	buf[8] = 0x00; /* max block size w/o crc high */
	buf[9] = 0x7E; /* max block size w/o crc low */

	/* Copy platform data */
	memcpy(&(buf[10]), pdata, ndata);

	/* Copy block CRC */
	buf[132] = crc_h;
	buf[133] = crc_l;

	/* Set key bytes */
	buf[134] = 0x45;
	buf[135] = 0x63;
	buf[136] = 0x36;
	buf[137] = 0x6F;
	buf[138] = 0x34;
	buf[139] = 0x38;
	buf[140] = 0x73;
	buf[141] = 0x77;

	/* Write config block */
	_cyttsp4_pr_buf(ts, buf, buf_size, name);

	retval = _cyttsp4_write_block_data(ts, ts->si_ofs.cmd_ofs + 1,
		141, &(buf[1]),
		ts->platform_data->addr[CY_TCH_ADDR_OFS], true);
	if (retval < 0) {
		dev_err(ts->dev,
			"%s: Failed to write config %s r=%d\n",
			__func__, name, retval);
		goto _cyttsp4_write_config_block_exit;
	}

	retval = _cyttsp4_put_cmd_wait(ts, ts->si_ofs.cmd_ofs,
		1, &(buf[0]), CY_TEN_SEC_TMO_MS,
		_cyttsp4_chk_cmd_rdy, NULL,
		ts->platform_data->addr[CY_TCH_ADDR_OFS], true);
	if (retval < 0) {
		dev_err(ts->dev,
			"%s: Fail write config command r=%d\n",
			__func__, retval);
		goto _cyttsp4_write_config_block_exit;
	}

	retval = _cyttsp4_read_block_data(ts, ts->si_ofs.cmd_ofs + 1,
		sizeof(status), &status,
		ts->platform_data->addr[CY_TCH_ADDR_OFS], true);
	if (retval < 0) {
		dev_err(ts->dev,
			"%s: Fail read status r=%d\n",
			__func__, retval);
		goto _cyttsp4_write_config_block_exit;
	}

	if (status != 0x00) {
		dev_err(ts->dev,
			"%s: Write config status=%d error\n",
			__func__, status);
		goto _cyttsp4_write_config_block_exit;
	}

_cyttsp4_write_config_block_exit:
	kfree(buf);
	return retval;
}
#endif /* --CY_USE_TMA884 */

#ifdef CONFIG_TOUCHSCREEN_DEBUG
#ifdef CY_USE_TMA884
static int _cyttsp4_read_config_block(struct cyttsp4 *ts, u8 blockid,
	u8 *pdata, size_t ndata, const char *name)
{
	int retval = 0;
	u8 cmd[CY_NUM_DAT+1];
	u8 status;

	/* Set command bytes */
	cmd[0] = 0x03; /* cmd */
	cmd[1] = 0x00; /* row offset high */
	cmd[2] = 0x00; /* row offset low */
	cmd[3] = ndata / 256; /* write block length high */
	cmd[4] = ndata % 256; /* write block length low */
	cmd[5] = blockid; /* read block id */
	cmd[6] = 0x00; /* blank fill */

	/* Write config block */
	_cyttsp4_pr_buf(ts, cmd, sizeof(cmd), name);

	retval = _cyttsp4_put_cmd_wait(ts, ts->si_ofs.cmd_ofs,
		sizeof(cmd), cmd, CY_TEN_SEC_TMO_MS,
		_cyttsp4_chk_cmd_rdy, NULL,
		ts->platform_data->addr[CY_TCH_ADDR_OFS], true);
	if (retval < 0) {
		dev_err(ts->dev,
			"%s: Fail write config command r=%d\n",
			__func__, retval);
		goto _cyttsp4_read_config_block_exit;
	}

	if (pdata[1] != 0x00) {
		dev_err(ts->dev,
			"%s: Read config block command failed"
			" response=%02X %02X\n",
			__func__, pdata[0], pdata[1]);
		retval = -EIO;
	}
	retval = _cyttsp4_read_block_data(ts, ts->si_ofs.cmd_ofs + 1,
		sizeof(status), &status,
		ts->platform_data->addr[CY_TCH_ADDR_OFS], true);
	if (retval < 0) {
		dev_err(ts->dev,
			"%s: Fail read status r=%d\n",
			__func__, retval);
		goto _cyttsp4_read_config_block_exit;
	}

	if (status != 0x00) {
		dev_err(ts->dev,
			"%s: Write config status=%d error\n",
			__func__, status);
		goto _cyttsp4_read_config_block_exit;
	} else {
		memset(pdata, 0, ndata);
		retval = _cyttsp4_read_block_data(ts, ts->si_ofs.cmd_ofs,
			ndata, pdata,
			ts->platform_data->addr[CY_TCH_ADDR_OFS], true);
		if (retval < 0) {
			dev_err(ts->dev,
			"%s: fail read cmd status"
				" r=%d\n", __func__, retval);
		} else {
			/* write the returned raw read config block data */
			_cyttsp4_pr_buf(ts, pdata, ndata, name);
		}
	}

_cyttsp4_read_config_block_exit:
	return retval;
}
#endif /* --CY_USE_TMA884 */
#endif /* --CONFIG_TOUCHSCREEN_DEBUG */

#ifdef CY_USE_TMA884
static int _cyttsp4_set_op_params(struct cyttsp4 *ts, u8 crc_h, u8 crc_l)
{
	int retval = 0;

	if (ts->platform_data->sett[CY_IC_GRPNUM_TCH_PARM_VAL] == NULL) {
		dev_err(ts->dev,
			"%s: Missing Platform Touch Parameter"
			" values table\n", __func__);
		retval = -ENXIO;
		goto _cyttsp4_set_op_params_exit;
	}

	if ((ts->platform_data->sett
		[CY_IC_GRPNUM_TCH_PARM_VAL]->data == NULL) ||
		(ts->platform_data->sett
		[CY_IC_GRPNUM_TCH_PARM_VAL]->size == 0)) {
		dev_err(ts->dev,
			"%s: Missing Platform Touch Parameter"
			" values table data\n", __func__);
		retval = -ENXIO;
		goto _cyttsp4_set_op_params_exit;
	}

	/* Change to Config Mode */
	retval = _cyttsp4_set_mode(ts, CY_CONFIG_MODE);
	if (retval < 0) {
		dev_err(ts->dev,
			"%s: Failed to switch to config mode"
			" for touch params\n", __func__);
		goto _cyttsp4_set_op_params_exit;
	}
	retval = _cyttsp4_write_config_block(ts, CY_TCH_PARM_EBID,
		ts->platform_data->sett[CY_IC_GRPNUM_TCH_PARM_VAL]->data,
		ts->platform_data->sett[CY_IC_GRPNUM_TCH_PARM_VAL]->size,
		crc_h, crc_l, "platform_touch_param_data");

_cyttsp4_set_op_params_exit:
	return retval;
}

static int _cyttsp4_set_data_block(struct cyttsp4 *ts, u8 blkid, u8 *pdata,
	size_t ndata, const char *name, bool force, bool *data_updated)
{
	u8 data_crc[2];
	u8 ic_crc[2];
	int retval = 0;

	memset(data_crc, 0, sizeof(data_crc));
	memset(ic_crc, 0, sizeof(ic_crc));
	*data_updated = false;

	_cyttsp4_pr_buf(ts, pdata, ndata, name);

	dev_vdbg(ts->dev,
		"%s: calc %s crc\n", __func__, name);
	retval = _cyttsp4_calc_data_crc(ts, ndata, pdata,
		&data_crc[0], &data_crc[1],
		name);
	if (retval < 0) {
		dev_err(ts->dev,
			"%s: fail calc crc for %s (0x%02X%02X) r=%d\n",
			__func__, name,
			data_crc[0], data_crc[1],
			retval);
		goto _cyttsp_set_data_block_exit;
	}

	dev_vdbg(ts->dev,
		"%s: get ic %s crc\n", __func__, name);
	retval = _cyttsp4_set_mode(ts, CY_OPERATE_MODE);
	if (retval < 0) {
		dev_err(ts->dev,
			"%s: Failed to switch to operational mode\n", __func__);
		goto _cyttsp_set_data_block_exit;
	}
	retval = _cyttsp4_get_ic_crc(ts, blkid,
		&ic_crc[0], &ic_crc[1]);
	if (retval < 0) {
		dev_err(ts->dev,
			"%s: fail get ic crc for %s (0x%02X%02X) r=%d\n",
			__func__, name,
			ic_crc[0], ic_crc[1],
			retval);
		goto _cyttsp_set_data_block_exit;
	}

	dev_vdbg(ts->dev,
		"%s: %s calc_crc=0x%02X%02X ic_crc=0x%02X%02X\n",
		__func__, name,
		data_crc[0], data_crc[1],
		ic_crc[0], ic_crc[1]);
	if ((data_crc[0] != ic_crc[0]) || (data_crc[1] != ic_crc[1]) || force) {
		/* Change to Config Mode */
		retval = _cyttsp4_set_mode(ts, CY_CONFIG_MODE);
		if (retval < 0) {
			dev_err(ts->dev,
			"%s: Failed to switch to config mode"
				" for sysinfo regs\n", __func__);
			goto _cyttsp_set_data_block_exit;
		}
		retval = _cyttsp4_write_config_block(ts, blkid, pdata,
			ndata, data_crc[0], data_crc[1], name);
		if (retval < 0) {
			dev_err(ts->dev,
			"%s: fail write %s config block r=%d\n",
				__func__, name, retval);
			goto _cyttsp_set_data_block_exit;
		}

		dev_vdbg(ts->dev,
			"%s: write %s config block ok\n", __func__, name);
		*data_updated = true;
	}

_cyttsp_set_data_block_exit:
	return retval;
}

static int _cyttsp4_set_sysinfo_regs(struct cyttsp4 *ts, bool *updated)
{
	bool ddata_updated = false;
	bool mdata_updated = false;
	size_t num_data = 0;
	u8 *pdata = NULL;
	int retval = 0;

	pdata = kzalloc(CY_NUM_MDATA, GFP_KERNEL);
	if (pdata == NULL) {
		dev_err(ts->dev,
			"%s: fail allocate set sysinfo regs buffer\n",
			__func__);
		retval = -ENOMEM;
		goto _cyttsp4_set_sysinfo_regs_err;
	}

	/* check for missing DDATA */
	if (ts->platform_data->sett[CY_IC_GRPNUM_DDATA_REC] == NULL) {
		dev_vdbg(ts->dev,
			"%s: No platform_ddata table\n", __func__);
		dev_vdbg(ts->dev,
			"%s: Use a zero filled array to compare with device\n",
			__func__);
		goto _cyttsp4_set_sysinfo_regs_set_ddata_block;
	}
	if ((ts->platform_data->sett[CY_IC_GRPNUM_DDATA_REC]->data == NULL) ||
		(ts->platform_data->sett[CY_IC_GRPNUM_DDATA_REC]->size == 0)) {
		dev_vdbg(ts->dev,
			"%s: No platform_ddata table data\n", __func__);
		dev_vdbg(ts->dev,
			"%s: Use a zero filled array to compare with device\n",
			__func__);
		goto _cyttsp4_set_sysinfo_regs_set_ddata_block;
	}

	/* copy platform data design data to the device eeprom */
	num_data = ts->platform_data->sett
		[CY_IC_GRPNUM_DDATA_REC]->size < CY_NUM_DDATA ?
		ts->platform_data->sett
		[CY_IC_GRPNUM_DDATA_REC]->size : CY_NUM_DDATA;
	dev_vdbg(ts->dev,
		"%s: copy %d bytes from platform data to ddata array\n",
		__func__, num_data);
	memcpy(pdata, ts->platform_data->sett[CY_IC_GRPNUM_DDATA_REC]->data,
		num_data);

_cyttsp4_set_sysinfo_regs_set_ddata_block:
	/* set data block will check CRC match/nomatch */
	retval = _cyttsp4_set_data_block(ts, CY_DDATA_EBID, pdata,
		CY_NUM_DDATA, "platform_ddata", false, &ddata_updated);
	if (retval < 0) {
		dev_err(ts->dev,
			"%s: Fail while writing platform_ddata"
			" block to ic r=%d\n", __func__, retval);
	}

	/* check for missing MDATA */
	if (ts->platform_data->sett[CY_IC_GRPNUM_MDATA_REC] == NULL) {
		dev_vdbg(ts->dev,
			"%s: No platform_mdata table\n", __func__);
		dev_vdbg(ts->dev,
			"%s: Use a zero filled array to compare with device\n",
			__func__);
		goto _cyttsp4_set_sysinfo_regs_set_mdata_block;
	}
	if ((ts->platform_data->sett[CY_IC_GRPNUM_MDATA_REC]->data == NULL) ||
		(ts->platform_data->sett[CY_IC_GRPNUM_MDATA_REC]->size == 0)) {
		dev_vdbg(ts->dev,
			"%s: No platform_mdata table data\n", __func__);
		dev_vdbg(ts->dev,
			"%s: Use a zero filled array to compare with device\n",
			__func__);
		goto _cyttsp4_set_sysinfo_regs_set_mdata_block;
	}

	/* copy platform manufacturing data to the device eeprom */
	num_data = ts->platform_data->sett
		[CY_IC_GRPNUM_MDATA_REC]->size < CY_NUM_MDATA ?
		ts->platform_data->sett
		[CY_IC_GRPNUM_MDATA_REC]->size : CY_NUM_MDATA;
	dev_vdbg(ts->dev,
		"%s: copy %d bytes from platform data to mdata array\n",
		__func__, num_data);
	memcpy(pdata, ts->platform_data->sett[CY_IC_GRPNUM_MDATA_REC]->data,
		num_data);

_cyttsp4_set_sysinfo_regs_set_mdata_block:
	/* set data block will check CRC match/nomatch */
	retval = _cyttsp4_set_data_block(ts, CY_MDATA_EBID, pdata,
		CY_NUM_MDATA, "platform_mdata", false, &mdata_updated);
	if (retval < 0) {
		dev_err(ts->dev,
			"%s: Fail while writing platform_mdata"
			" block to ic r=%d\n", __func__, retval);
	}

	kfree(pdata);
_cyttsp4_set_sysinfo_regs_err:
	*updated = ddata_updated || mdata_updated;
	return retval;
}
#endif /* --CY_USE_TMA884 */

static int _cyttsp4_bits_2_bytes(struct cyttsp4 *ts, int nbits, int *max)
{
	int nbytes;

	*max = 1 << nbits;

	for (nbytes = 0; nbits > 0;) {
		dev_vdbg(ts->dev,
			"%s: nbytes=%d nbits=%d\n", __func__, nbytes, nbits);
		nbytes++;
		if (nbits > 8)
			nbits -= 8;
		else
			nbits = 0;
		dev_vdbg(ts->dev,
			"%s: nbytes=%d nbits=%d\n", __func__, nbytes, nbits);
	}

	return nbytes;
}

static int _cyttsp4_get_sysinfo_regs(struct cyttsp4 *ts)
{
	int retval = 0;

	/* get the sysinfo data offsets */
	retval = _cyttsp4_read_block_data(ts, CY_REG_BASE,
		sizeof(ts->sysinfo_data), &(ts->sysinfo_data),
		ts->platform_data->addr[CY_TCH_ADDR_OFS], true);
	if (retval < 0) {
		dev_err(ts->dev,
			"%s: fail read sysinfo data offsets r=%d\n",
			__func__, retval);
		goto _cyttsp4_get_sysinfo_regs_exit_no_handshake;
	} else {
		/* Print sysinfo data offsets */
		_cyttsp4_pr_buf(ts, (u8 *)&ts->sysinfo_data,
			sizeof(ts->sysinfo_data), "sysinfo_data_offsets");

		/* convert sysinfo data offset bytes into integers */
		ts->si_ofs.map_sz = (ts->sysinfo_data.map_szh * 256) +
			ts->sysinfo_data.map_szl;
		ts->si_ofs.cydata_ofs = (ts->sysinfo_data.cydata_ofsh * 256) +
			ts->sysinfo_data.cydata_ofsl;
		ts->si_ofs.test_ofs = (ts->sysinfo_data.test_ofsh * 256) +
			ts->sysinfo_data.test_ofsl;
		ts->si_ofs.pcfg_ofs = (ts->sysinfo_data.pcfg_ofsh * 256) +
			ts->sysinfo_data.pcfg_ofsl;
		ts->si_ofs.opcfg_ofs = (ts->sysinfo_data.opcfg_ofsh * 256) +
			ts->sysinfo_data.opcfg_ofsl;
		ts->si_ofs.ddata_ofs = (ts->sysinfo_data.ddata_ofsh * 256) +
			ts->sysinfo_data.ddata_ofsl;
		ts->si_ofs.mdata_ofs = (ts->sysinfo_data.mdata_ofsh * 256) +
			ts->sysinfo_data.mdata_ofsl;
	}

	/* get the sysinfo cydata */
	ts->si_ofs.cydata_size = ts->si_ofs.test_ofs - ts->si_ofs.cydata_ofs;
	if (ts->si_ofs.cydata_size <= 0) {
		dev_err(ts->dev, "%s: invalid cydata_size %d\n", __func__, ts->si_ofs.cydata_size);
		retval = -EINVAL;
		goto _cyttsp4_get_sysinfo_regs_exit_no_handshake;
	}
	ts->sysinfo_ptr.cydata = kzalloc(ts->si_ofs.cydata_size, GFP_KERNEL);
	if (ts->sysinfo_ptr.cydata == NULL) {
		retval = -ENOMEM;
		dev_err(ts->dev,
			"%s: fail alloc cydata memory r=%d\n",
			__func__, retval);
		goto _cyttsp4_get_sysinfo_regs_exit_no_handshake;
	} else {
		memset(ts->sysinfo_ptr.cydata, 0, ts->si_ofs.cydata_size);
		retval = _cyttsp4_read_block_data(ts, ts->si_ofs.cydata_ofs,
			ts->si_ofs.cydata_size, ts->sysinfo_ptr.cydata,
			ts->platform_data->addr[CY_TCH_ADDR_OFS], true);
		if (retval < 0) {
			dev_err(ts->dev,
			"%s: fail read cydata r=%d\n",
				__func__, retval);
			goto _cyttsp4_get_sysinfo_regs_exit;
		}
		/* Print sysinfo cydata */
		_cyttsp4_pr_buf(ts, (u8 *)ts->sysinfo_ptr.cydata,
			ts->si_ofs.cydata_size, "sysinfo_cydata");
	}
	/* get the sysinfo test data */
	ts->si_ofs.test_size = ts->si_ofs.pcfg_ofs - ts->si_ofs.test_ofs;
	if (ts->si_ofs.test_size <= 0) {
		dev_err(ts->dev, "%s: invalid test_size %d\n", __func__, ts->si_ofs.test_size);
		retval = -EINVAL;
		goto _cyttsp4_get_sysinfo_regs_exit_no_handshake;
	}
	ts->sysinfo_ptr.test = kzalloc(ts->si_ofs.test_size, GFP_KERNEL);
	if (ts->sysinfo_ptr.test == NULL) {
		retval = -ENOMEM;
		dev_err(ts->dev,
			"%s: fail alloc test memory r=%d\n",
			__func__, retval);
		goto _cyttsp4_get_sysinfo_regs_exit_no_handshake;
	} else {
		memset(ts->sysinfo_ptr.test, 0, ts->si_ofs.test_size);
		retval = _cyttsp4_read_block_data(ts, ts->si_ofs.test_ofs,
			ts->si_ofs.test_size, ts->sysinfo_ptr.test,
			ts->platform_data->addr[CY_TCH_ADDR_OFS], true);
		if (retval < 0) {
			dev_err(ts->dev,
			"%s: fail read test data r=%d\n",
				__func__, retval);
			goto _cyttsp4_get_sysinfo_regs_exit;
		}
		/* Print sysinfo test data */
		_cyttsp4_pr_buf(ts, (u8 *)ts->sysinfo_ptr.test,
			ts->si_ofs.test_size, "sysinfo_test_data");
#ifdef CY_USE_TMA400
		if (ts->sysinfo_ptr.test->post_codel & 0x01) {
			dev_info(ts->dev,
			"%s: Reset was a WATCHDOG RESET codel=%02X\n",
				__func__, ts->sysinfo_ptr.test->post_codel);
		}

		if (!(ts->sysinfo_ptr.test->post_codel & 0x02)) {
			dev_info(ts->dev,
			"%s: Config Data CRC FAIL codel=%02X\n",
				__func__, ts->sysinfo_ptr.test->post_codel);
		}

		if (!(ts->sysinfo_ptr.test->post_codel & 0x04)) {
			dev_info(ts->dev,
			"%s: PANEL TEST FAIL codel=%02X\n",
				__func__, ts->sysinfo_ptr.test->post_codel);
		}

		dev_info(ts->dev,
			"%s: SCANNING is %s codel=%02X\n", __func__,
			ts->sysinfo_ptr.test->post_codel & 0x08 ? "ENABLED" :
			"DISABLED", ts->sysinfo_ptr.test->post_codel);
#endif /* --CY_USE_TMA400 */
	}
	/* get the sysinfo pcfg data */
	ts->si_ofs.pcfg_size = ts->si_ofs.opcfg_ofs - ts->si_ofs.pcfg_ofs;
	if (ts->si_ofs.pcfg_size <= 0) {
		dev_err(ts->dev, "%s: invalid pcfg_size %d\n", __func__, ts->si_ofs.pcfg_size);
		retval = -EINVAL;
		goto _cyttsp4_get_sysinfo_regs_exit_no_handshake;
	}
	ts->sysinfo_ptr.pcfg = kzalloc(ts->si_ofs.pcfg_size, GFP_KERNEL);
	if (ts->sysinfo_ptr.pcfg == NULL) {
		retval = -ENOMEM;
		dev_err(ts->dev,
			"%s: fail alloc pcfg memory r=%d\n",
			__func__, retval);
		goto _cyttsp4_get_sysinfo_regs_exit_no_handshake;
	} else {
		memset(ts->sysinfo_ptr.pcfg, 0, ts->si_ofs.pcfg_size);
		retval = _cyttsp4_read_block_data(ts, ts->si_ofs.pcfg_ofs,
			ts->si_ofs.pcfg_size, ts->sysinfo_ptr.pcfg,
			ts->platform_data->addr[CY_TCH_ADDR_OFS], true);
		if (retval < 0) {
			dev_err(ts->dev,
			"%s: fail read pcfg data r=%d\n",
				__func__, retval);
			goto _cyttsp4_get_sysinfo_regs_exit;
		}
		/* Print sysinfo pcfg data */
		_cyttsp4_pr_buf(ts, (u8 *)ts->sysinfo_ptr.pcfg,
			ts->si_ofs.pcfg_size, "sysinfo_pcfg_data");
	}
	/* get the sysinfo opcfg data */
	ts->si_ofs.cmd_ofs = 0;
	ts->si_ofs.rep_ofs = 0;
	ts->si_ofs.rep_sz = 0;
	ts->si_ofs.tt_stat_ofs = 0;
	ts->si_ofs.max_tchs = 0;
	ts->si_ofs.tch_rec_siz = 0;
	ts->si_ofs.tch_rec_x_ofs = 0;	/* x position */
	ts->si_ofs.tch_rec_x_size = 0;
	ts->si_ofs.tch_rec_y_ofs = 0;	/* y position */
	ts->si_ofs.tch_rec_y_size = 0;
	ts->si_ofs.tch_rec_p_ofs = 0;	/* pressure */
	ts->si_ofs.tch_rec_p_size = 0;
	ts->si_ofs.tch_rec_t_ofs = 0;	/* track id */
	ts->si_ofs.tch_rec_t_size = 0;
	ts->si_ofs.tch_rec_e_ofs = 0;	/* event id */
	ts->si_ofs.tch_rec_e_size = 0;
	ts->si_ofs.tch_rec_o_ofs = 0;	/* object type */
	ts->si_ofs.tch_rec_o_size = 0;
	ts->si_ofs.tch_rec_w_ofs = 0;	/* size */
	ts->si_ofs.tch_rec_w_size = 0;
	ts->si_ofs.mode_size = 0;
	ts->si_ofs.data_size = 0;
	ts->si_ofs.opcfg_size = ts->si_ofs.ddata_ofs - ts->si_ofs.opcfg_ofs;
	if (ts->si_ofs.opcfg_size <= 0) {
		dev_err(ts->dev, "%s: invalid opcfg_size %d\n", __func__, ts->si_ofs.opcfg_size);
		retval = -EINVAL;
		goto _cyttsp4_get_sysinfo_regs_exit_no_handshake;
	}
	ts->sysinfo_ptr.opcfg = kzalloc(ts->si_ofs.opcfg_size, GFP_KERNEL);
	if (ts->sysinfo_ptr.opcfg == NULL) {
		retval = -ENOMEM;
		dev_err(ts->dev,
			"%s: fail alloc opcfg memory r=%d\n",
			__func__, retval);
		goto _cyttsp4_get_sysinfo_regs_exit_no_handshake;
	} else {
		memset(ts->sysinfo_ptr.opcfg, 0, ts->si_ofs.opcfg_size);
		retval = _cyttsp4_read_block_data(ts, ts->si_ofs.opcfg_ofs,
			ts->si_ofs.opcfg_size, ts->sysinfo_ptr.opcfg,
			ts->platform_data->addr[CY_TCH_ADDR_OFS], true);
		if (retval < 0) {
			dev_err(ts->dev,
			"%s: fail read opcfg data r=%d\n",
				__func__, retval);
			goto _cyttsp4_get_sysinfo_regs_exit;
		}
		ts->si_ofs.cmd_ofs = ts->sysinfo_ptr.opcfg->cmd_ofs;
		ts->si_ofs.rep_ofs = ts->sysinfo_ptr.opcfg->rep_ofs;
		ts->si_ofs.rep_sz = (ts->sysinfo_ptr.opcfg->rep_szh * 256) +
			ts->sysinfo_ptr.opcfg->rep_szl;
		ts->si_ofs.tt_stat_ofs = ts->sysinfo_ptr.opcfg->tt_stat_ofs;
		ts->si_ofs.obj_cfg0 = ts->sysinfo_ptr.opcfg->obj_cfg0;
		ts->si_ofs.max_tchs = ts->sysinfo_ptr.opcfg->max_tchs;
		ts->si_ofs.tch_rec_siz = ts->sysinfo_ptr.opcfg->tch_rec_siz;
		ts->si_ofs.tch_rec_x_ofs = ts->sysinfo_ptr.opcfg->tch_rec_0;
		ts->si_ofs.tch_rec_x_size =
			_cyttsp4_bits_2_bytes(ts,
			ts->sysinfo_ptr.opcfg->tch_rec_1,
			 &ts->si_ofs.tch_rec_x_max);
		ts->si_ofs.tch_rec_y_ofs = ts->sysinfo_ptr.opcfg->tch_rec_2;
		ts->si_ofs.tch_rec_y_size =
			_cyttsp4_bits_2_bytes(ts,
			ts->sysinfo_ptr.opcfg->tch_rec_3,
			 &ts->si_ofs.tch_rec_y_max);
		ts->si_ofs.tch_rec_p_ofs = ts->sysinfo_ptr.opcfg->tch_rec_4;
		ts->si_ofs.tch_rec_p_size =
			_cyttsp4_bits_2_bytes(ts,
			ts->sysinfo_ptr.opcfg->tch_rec_5,
			 &ts->si_ofs.tch_rec_p_max);
		ts->si_ofs.tch_rec_t_ofs = ts->sysinfo_ptr.opcfg->tch_rec_6;
		ts->si_ofs.tch_rec_t_size =
			_cyttsp4_bits_2_bytes(ts,
			ts->sysinfo_ptr.opcfg->tch_rec_7,
			 &ts->si_ofs.tch_rec_t_max);
		ts->si_ofs.tch_rec_e_ofs = ts->sysinfo_ptr.opcfg->tch_rec_8;
		ts->si_ofs.tch_rec_e_size =
			_cyttsp4_bits_2_bytes(ts,
			ts->sysinfo_ptr.opcfg->tch_rec_9,
			 &ts->si_ofs.tch_rec_e_max);
		ts->si_ofs.tch_rec_o_ofs = ts->sysinfo_ptr.opcfg->tch_rec_10;
		ts->si_ofs.tch_rec_o_size =
			_cyttsp4_bits_2_bytes(ts,
			ts->sysinfo_ptr.opcfg->tch_rec_11,
			 &ts->si_ofs.tch_rec_o_max);
		ts->si_ofs.tch_rec_w_ofs = ts->sysinfo_ptr.opcfg->tch_rec_12;
		ts->si_ofs.tch_rec_w_size =
			_cyttsp4_bits_2_bytes(ts,
			ts->sysinfo_ptr.opcfg->tch_rec_13,
			 &ts->si_ofs.tch_rec_w_max);
		ts->si_ofs.mode_size = ts->si_ofs.tt_stat_ofs + 1;
		ts->si_ofs.data_size = ts->si_ofs.max_tchs *
			ts->sysinfo_ptr.opcfg->tch_rec_siz;
		/* Print sysinfo opcfg data */
		_cyttsp4_pr_buf(ts, (u8 *)ts->sysinfo_ptr.opcfg,
			ts->si_ofs.opcfg_size, "sysinfo_opcfg_data");
	}

	/* get the sysinfo ddata data */
	ts->si_ofs.ddata_size = ts->si_ofs.mdata_ofs - ts->si_ofs.ddata_ofs;
	if (ts->si_ofs.ddata_size <= 0) {
		dev_err(ts->dev, "%s: invalid ddata_size %d\n", __func__, ts->si_ofs.ddata_size);
		retval = -EINVAL;
		goto _cyttsp4_get_sysinfo_regs_exit_no_handshake;
	}
	ts->sysinfo_ptr.ddata = kzalloc(ts->si_ofs.ddata_size, GFP_KERNEL);
	if (ts->sysinfo_ptr.ddata == NULL) {
		dev_err(ts->dev,
			"%s: fail alloc ddata memory r=%d\n",
			__func__, retval);
		/* continue */
	} else {
		memset(ts->sysinfo_ptr.ddata, 0, ts->si_ofs.ddata_size);
		retval = _cyttsp4_read_block_data(ts, ts->si_ofs.ddata_ofs,
			ts->si_ofs.ddata_size, ts->sysinfo_ptr.ddata,
			ts->platform_data->addr[CY_TCH_ADDR_OFS], true);
		if (retval < 0) {
			dev_err(ts->dev,
			"%s: fail read ddata data r=%d\n",
				__func__, retval);
			goto _cyttsp4_get_sysinfo_regs_exit;
		}
		/* Print sysinfo ddata */
		_cyttsp4_pr_buf(ts, (u8 *)ts->sysinfo_ptr.ddata,
			ts->si_ofs.ddata_size, "sysinfo_ddata");
	}
	/* get the sysinfo mdata data */
	ts->si_ofs.mdata_size = ts->si_ofs.map_sz - ts->si_ofs.mdata_ofs;
	if (ts->si_ofs.mdata_size <= 0) {
		dev_err(ts->dev, "%s: invalid mdata_size %d\n", __func__, ts->si_ofs.mdata_size);
		retval = -EINVAL;
		goto _cyttsp4_get_sysinfo_regs_exit;
	}
	ts->sysinfo_ptr.mdata = kzalloc(ts->si_ofs.mdata_size, GFP_KERNEL);
	if (ts->sysinfo_ptr.mdata == NULL) {
		dev_err(ts->dev,
			"%s: fail alloc mdata memory r=%d\n",
			__func__, retval);
		/* continue */
	} else {
		memset(ts->sysinfo_ptr.mdata, 0, ts->si_ofs.mdata_size);
		retval = _cyttsp4_read_block_data(ts, ts->si_ofs.mdata_ofs,
			ts->si_ofs.mdata_size, ts->sysinfo_ptr.mdata,
			ts->platform_data->addr[CY_TCH_ADDR_OFS], true);
		if (retval < 0) {
			dev_err(ts->dev,
			"%s: fail read mdata data r=%d\n",
				__func__, retval);
			goto _cyttsp4_get_sysinfo_regs_exit;
		}
		/* Print sysinfo mdata */
		_cyttsp4_pr_buf(ts, (u8 *)ts->sysinfo_ptr.mdata,
			ts->si_ofs.mdata_size, "sysinfo_mdata");
	}

	dev_vdbg(ts->dev,
		"%s: cydata_ofs =%4d siz=%4d\n", __func__,
		ts->si_ofs.cydata_ofs, ts->si_ofs.cydata_size);
	dev_vdbg(ts->dev,
		"%s: test_ofs   =%4d siz=%4d\n", __func__,
		ts->si_ofs.test_ofs, ts->si_ofs.test_size);
	dev_vdbg(ts->dev,
		"%s: pcfg_ofs   =%4d siz=%4d\n", __func__,
		ts->si_ofs.pcfg_ofs, ts->si_ofs.pcfg_size);
	dev_vdbg(ts->dev,
		"%s: opcfg_ofs  =%4d siz=%4d\n", __func__,
		ts->si_ofs.opcfg_ofs, ts->si_ofs.opcfg_size);
	dev_vdbg(ts->dev,
		"%s: ddata_ofs  =%4d siz=%4d\n", __func__,
		ts->si_ofs.ddata_ofs, ts->si_ofs.ddata_size);
	dev_vdbg(ts->dev,
		"%s: mdata_ofs  =%4d siz=%4d\n", __func__,
		ts->si_ofs.mdata_ofs, ts->si_ofs.mdata_size);
	dev_vdbg(ts->dev,
		"%s: cmd_ofs    =%4d\n", __func__, ts->si_ofs.cmd_ofs);
	dev_vdbg(ts->dev,
		"%s: rep_ofs    =%4d\n", __func__, ts->si_ofs.rep_ofs);
	dev_vdbg(ts->dev,
		"%s: rep_sz     =%4d\n", __func__, ts->si_ofs.rep_sz);
	dev_vdbg(ts->dev,
		"%s: tt_stat_ofs=%4d\n", __func__, ts->si_ofs.tt_stat_ofs);
	dev_vdbg(ts->dev,
		"%s: tch_rec_siz=%4d\n", __func__, ts->si_ofs.tch_rec_siz);
	dev_vdbg(ts->dev,
		"%s: max_tchs   =%4d\n", __func__, ts->si_ofs.max_tchs);
	dev_vdbg(ts->dev,
		"%s: mode_siz   =%4d\n", __func__, ts->si_ofs.mode_size);
	dev_vdbg(ts->dev,
		"%s: data_siz   =%4d\n", __func__, ts->si_ofs.data_size);
	dev_vdbg(ts->dev,
		"%s: map_sz     =%4d\n", __func__, ts->si_ofs.map_sz);
	dev_vdbg(ts->dev,
		"%s: tch_rec_x_ofs=%2d\n", __func__, ts->si_ofs.tch_rec_x_ofs);
	dev_vdbg(ts->dev,
		"%s: tch_rec_x_siz=%2d\n", __func__, ts->si_ofs.tch_rec_x_size);
	dev_vdbg(ts->dev,
		"%s: tch_rec_x_max=%2d\n", __func__, ts->si_ofs.tch_rec_x_max);
	dev_vdbg(ts->dev,
		"%s: tch_rec_y_ofs=%2d\n", __func__, ts->si_ofs.tch_rec_y_ofs);
	dev_vdbg(ts->dev,
		"%s: tch_rec_y_siz=%2d\n", __func__, ts->si_ofs.tch_rec_y_size);
	dev_vdbg(ts->dev,
		"%s: tch_rec_y_max=%2d\n", __func__, ts->si_ofs.tch_rec_y_max);
	dev_vdbg(ts->dev,
		"%s: tch_rec_p_ofs=%2d\n", __func__, ts->si_ofs.tch_rec_p_ofs);
	dev_vdbg(ts->dev,
		"%s: tch_rec_p_siz=%2d\n", __func__, ts->si_ofs.tch_rec_p_size);
	dev_vdbg(ts->dev,
		"%s: tch_rec_p_max=%2d\n", __func__, ts->si_ofs.tch_rec_p_max);
	dev_vdbg(ts->dev,
		"%s: tch_rec_t_ofs=%2d\n", __func__, ts->si_ofs.tch_rec_t_ofs);
	dev_vdbg(ts->dev,
		"%s: tch_rec_t_siz=%2d\n", __func__, ts->si_ofs.tch_rec_t_size);
	dev_vdbg(ts->dev,
		"%s: tch_rec_t_max=%2d\n", __func__, ts->si_ofs.tch_rec_t_max);
	dev_vdbg(ts->dev,
		"%s: tch_rec_e_ofs=%2d\n", __func__, ts->si_ofs.tch_rec_e_ofs);
	dev_vdbg(ts->dev,
		"%s: tch_rec_e_siz=%2d\n", __func__, ts->si_ofs.tch_rec_e_size);
	dev_vdbg(ts->dev,
		"%s: tch_rec_e_max=%2d\n", __func__, ts->si_ofs.tch_rec_e_max);
	dev_vdbg(ts->dev,
		"%s: tch_rec_o_ofs=%2d\n", __func__, ts->si_ofs.tch_rec_o_ofs);
	dev_vdbg(ts->dev,
		"%s: tch_rec_o_siz=%2d\n", __func__, ts->si_ofs.tch_rec_o_size);
	dev_vdbg(ts->dev,
		"%s: tch_rec_o_max=%2d\n", __func__, ts->si_ofs.tch_rec_o_max);
	dev_vdbg(ts->dev,
		"%s: tch_rec_w_ofs=%2d\n", __func__, ts->si_ofs.tch_rec_w_ofs);
	dev_vdbg(ts->dev,
		"%s: tch_rec_w_siz=%2d\n", __func__, ts->si_ofs.tch_rec_w_size);
	dev_vdbg(ts->dev,
		"%s: tch_rec_w_max=%2d\n", __func__, ts->si_ofs.tch_rec_w_max);

	if (ts->si_ofs.mode_size <= 0) {
		dev_err(ts->dev, "%s: invalid mode_size %d\n", __func__, ts->si_ofs.mode_size);
		retval = -EINVAL;
		goto _cyttsp4_get_sysinfo_regs_exit_no_handshake;
	}
	if (ts->si_ofs.data_size <= 0) {
		dev_err(ts->dev, "%s: invalid data_size %d\n", __func__, ts->si_ofs.data_size);
		retval = -EINVAL;
		goto _cyttsp4_get_sysinfo_regs_exit_no_handshake;
	}
	if (ts->si_ofs.tch_rec_siz + 1 <= 0) {
		dev_err(ts->dev, "%s: invalid tch_rec_siz+1 %d\n", __func__, ts->si_ofs.tch_rec_siz + 1);
		retval = -EINVAL;
		goto _cyttsp4_get_sysinfo_regs_exit_no_handshake;
	}

	if (ts->xy_mode == NULL)
		ts->xy_mode = kzalloc(ts->si_ofs.mode_size, GFP_KERNEL);
	if (ts->xy_data == NULL)
		ts->xy_data = kzalloc(ts->si_ofs.data_size, GFP_KERNEL);
	if (ts->xy_data_touch1 == NULL) {
		ts->xy_data_touch1 = kzalloc(ts->si_ofs.tch_rec_siz + 1,
			GFP_KERNEL);
	}
	if ((ts->xy_mode == NULL) || (ts->xy_data == NULL) ||
		(ts->xy_data_touch1 == NULL)) {
		dev_err(ts->dev,
			"%s: fail memory alloc xy_mode=%p xy_data=%p"
			"xy_data_touch1=%p\n", __func__,
			ts->xy_mode, ts->xy_data, ts->xy_data_touch1);
		/* continue */
	}

	dev_vdbg(ts->dev,
		"%s: xy_mode=%p xy_data=%p xy_data_touch1=%p\n",
		__func__, ts->xy_mode, ts->xy_data, ts->xy_data_touch1);

_cyttsp4_get_sysinfo_regs_exit:
	/* provide flow control handshake */
	retval = _cyttsp4_handshake(ts, ts->sysinfo_data.hst_mode);
	if (retval < 0) {
		dev_err(ts->dev,
			"%s: handshake fail on sysinfo reg\n",
			__func__);
		/* continue; rely on handshake tmo */
	}

_cyttsp4_get_sysinfo_regs_exit_no_handshake:
	return retval;
}

static int _cyttsp4_load_status_regs(struct cyttsp4 *ts)
{
	int rep_stat_ofs;
	int retval = 0;

	rep_stat_ofs = ts->si_ofs.rep_ofs + 1;
	if (ts->xy_mode == NULL) {
		dev_err(ts->dev,
			"%s: mode ptr not yet initialized xy_mode=%p\n",
			__func__, ts->xy_mode);
		/* continue */
	} else {
		retval = _cyttsp4_read_block_data(ts, CY_REG_BASE,
			rep_stat_ofs + 1, ts->xy_mode,
			ts->platform_data->addr[CY_TCH_ADDR_OFS], true);
		if (retval < 0) {
			dev_err(ts->dev,
			"%s: fail read mode regs r=%d\n",
				__func__, retval);
			retval = -EIO;
		}
		_cyttsp4_pr_buf(ts, ts->xy_mode, rep_stat_ofs + 1, "xy_mode");
	}
	return retval;
}

static void _cyttsp4_get_touch_axis(struct cyttsp4 *ts,
	int *axis, int size, int max, u8 *xy_data)
{
	int nbyte;
	int next;

	for (nbyte = 0, *axis = 0, next = 0; nbyte < size; nbyte++) {
		dev_vdbg(ts->dev,
			"%s: *axis=%02X(%d) size=%d max=%08X xy_data=%p"
			" xy_data[%d]=%02X(%d)\n",
			__func__, *axis, *axis, size, max, xy_data, next,
			xy_data[next], xy_data[next]);
		*axis = (*axis * 256) + xy_data[next++];
	}

	*axis &= max - 1;

	dev_vdbg(ts->dev,
		"%s: *axis=%02X(%d) size=%d max=%08X xy_data=%p"
		" xy_data[%d]=%02X(%d)\n",
		__func__, *axis, *axis, size, max, xy_data, next,
		xy_data[next], xy_data[next]);
}

static void _cyttsp4_get_touch(struct cyttsp4 *ts,
	struct cyttsp4_touch *touch, u8 *xy_data)
{
#ifdef CY_USE_DEBUG_TOOLS
	int tmp;
#endif /* --CY_USE_DEBUG_TOOLS */

	_cyttsp4_get_touch_axis(ts, &touch->x,
		ts->si_ofs.tch_rec_x_size, ts->si_ofs.tch_rec_x_max,
		xy_data + ts->si_ofs.tch_rec_x_ofs);
	dev_vdbg(ts->dev,
		"%s: get x=%08X(%d) rec_x_size=%d"
		" rec_x_ofs=%d xy_data+ofs=%p\n",
		__func__, touch->x, touch->x, ts->si_ofs.tch_rec_x_size,
		ts->si_ofs.tch_rec_x_ofs, xy_data + ts->si_ofs.tch_rec_x_ofs);
	_cyttsp4_get_touch_axis(ts, &touch->y,
		ts->si_ofs.tch_rec_y_size, ts->si_ofs.tch_rec_y_max,
		xy_data + ts->si_ofs.tch_rec_y_ofs);
	dev_vdbg(ts->dev,
		"%s: get y=%08X(%d) rec_y_size=%d"
		" rec_y_ofs=%d xy_data+ofs=%p\n",
		__func__, touch->y, touch->y, ts->si_ofs.tch_rec_y_size,
		ts->si_ofs.tch_rec_y_ofs, xy_data + ts->si_ofs.tch_rec_y_ofs);
	_cyttsp4_get_touch_axis(ts, &touch->p,
		ts->si_ofs.tch_rec_p_size, ts->si_ofs.tch_rec_p_max,
		xy_data + ts->si_ofs.tch_rec_p_ofs);
	dev_vdbg(ts->dev,
		"%s: get p=%08X(%d) rec_p_size=%d"
		" rec_p_ofs=%d xy_data+ofs=%p\n",
		__func__, touch->p, touch->p, ts->si_ofs.tch_rec_p_size,
		ts->si_ofs.tch_rec_p_ofs, xy_data + ts->si_ofs.tch_rec_p_ofs);
	_cyttsp4_get_touch_axis(ts, &touch->t,
		ts->si_ofs.tch_rec_t_size, ts->si_ofs.tch_rec_t_max,
		xy_data + ts->si_ofs.tch_rec_t_ofs);
	dev_vdbg(ts->dev,
		"%s: get t=%08X(%d) rec_t_size=%d"
		" rec_t_ofs=%d rec_t_max=%d xy_data+ofs=%p\n",
		__func__, touch->t, touch->t, ts->si_ofs.tch_rec_t_size,
		ts->si_ofs.tch_rec_t_ofs, ts->si_ofs.tch_rec_t_max,
		xy_data + ts->si_ofs.tch_rec_t_ofs);
	_cyttsp4_get_touch_axis(ts, &touch->w,
		ts->si_ofs.tch_rec_w_size, ts->si_ofs.tch_rec_w_max,
		xy_data + ts->si_ofs.tch_rec_w_ofs);
	dev_vdbg(ts->dev,
		"%s: get w=%08X(%d) rec_w_size=%d"
		" rec_w_ofs=%d rec_w_max=%d xy_data+ofs=%p\n",
		__func__, touch->w, touch->w, ts->si_ofs.tch_rec_w_size,
		ts->si_ofs.tch_rec_w_ofs, ts->si_ofs.tch_rec_w_max,
		xy_data + ts->si_ofs.tch_rec_w_ofs);
	_cyttsp4_get_touch_axis(ts, &touch->e,
		ts->si_ofs.tch_rec_e_size, ts->si_ofs.tch_rec_e_max,
		xy_data + ts->si_ofs.tch_rec_e_ofs);
	dev_vdbg(ts->dev,
		"%s: get e=%08X(%d) rec_e_size=%d"
		" rec_e_ofs=%d rec_e_max=%d xy_data+ofs=%p\n",
		__func__, touch->e, touch->e, ts->si_ofs.tch_rec_e_size,
		ts->si_ofs.tch_rec_e_ofs, ts->si_ofs.tch_rec_e_max,
		xy_data + ts->si_ofs.tch_rec_e_ofs);
	_cyttsp4_get_touch_axis(ts, &touch->o,
		ts->si_ofs.tch_rec_o_size, ts->si_ofs.tch_rec_o_max,
		xy_data + ts->si_ofs.tch_rec_o_ofs);
	dev_vdbg(ts->dev,
		"%s: get o=%08X(%d) rec_0_size=%d"
		" rec_o_ofs=%d rec_o_max=%d xy_data+ofs=%p\n",
		__func__, touch->o, touch->o, ts->si_ofs.tch_rec_o_size,
		ts->si_ofs.tch_rec_e_ofs, ts->si_ofs.tch_rec_e_max,
		xy_data + ts->si_ofs.tch_rec_e_ofs);
#ifdef CY_USE_DEBUG_TOOLS
	if (ts->flags & CY_FLIP) {
		tmp = touch->x;
		touch->x = touch->y;
		touch->y = tmp;
	}
	if (ts->flags & CY_INV_X) {
		touch->x =
			ts->platform_data->frmwrk->abs
			[(CY_ABS_X_OST*CY_NUM_ABS_SET)
			+2] - touch->x;
	}
	if (ts->flags & CY_INV_Y) {
		touch->y =
			ts->platform_data->frmwrk->abs
			[(CY_ABS_Y_OST*CY_NUM_ABS_SET)
			+2] - touch->y;
	}
#endif /* --CY_USE_DEBUG_TOOLS */
}

/* read xy_data for all current touches */
static int _cyttsp4_xy_worker(struct cyttsp4 *ts)
{
	struct cyttsp4_touch touch;
	u8 num_cur_tch = 0;
	u8 hst_mode = 0;
	u8 rep_len = 0;
	u8 rep_stat = 0;
	u8 tt_stat = 0;
	int i;
	int t = 0;
	int signal;
	int retval;
#if defined(CONFIG_SHTPS_TMA4XX_TMA463_003)
	u32 version;
#endif	/* defined(CONFIG_SHTPS_TMA4XX_TMA463_003) */

	int id;
	static int num_pre_tch = 0;
	static struct cyttsp4_touch pre_touch[] = {
		/* {x, y, p, t, e, o, w} */
		{0, 0, 0, 0, 0, 0, 0},
		{0, 0, 0, 1, 0, 0, 0},
		{0, 0, 0, 2, 0, 0, 0},
		{0, 0, 0, 3, 0, 0, 0},
		{0, 0, 0, 4, 0, 0, 0},
		{0, 0, 0, 5, 0, 0, 0},
		{0, 0, 0, 6, 0, 0, 0},
		{0, 0, 0, 7, 0, 0, 0},
		{0, 0, 0, 8, 0, 0, 0},
		{0, 0, 0, 9, 0, 0, 0},
	};

	/*
	 * Get event data from CYTTSP device.
	 * The event data includes all data
	 * for all active touches.
	 */
	/*
	 * Use 2 reads: first to get mode bytes,
	 * second to get status (touch count) and touch 1 data.
	 * An optional 3rd read to get touch 2 - touch n data.
	 */
	memset(ts->xy_mode, 0, ts->si_ofs.mode_size);
	memset(ts->xy_data_touch1, 0, 1 + ts->si_ofs.tch_rec_siz);

	retval = _cyttsp4_load_status_regs(ts);
	if (retval < 0) {
		/*
		 * bus failure implies Watchdog -> bootloader running
		 * on TMA884 parts
		*/
		dev_err(ts->dev,
			"%s: 1st read fail on mode regs r=%d\n",
			__func__, retval);
		retval = -EIO;
		goto _cyttsp4_xy_worker_exit;
	}
	retval = _cyttsp4_read_block_data(ts, ts->si_ofs.tt_stat_ofs,
		1+ts->si_ofs.tch_rec_siz, ts->xy_data_touch1,
		ts->platform_data->addr[CY_TCH_ADDR_OFS], true);
	if (retval < 0) {
		/* bus failure may imply bootloader running */
		dev_err(ts->dev,
			"%s: read fail on mode regs r=%d\n",
			__func__, retval);
		retval = -EIO;
		goto _cyttsp4_xy_worker_exit;
	}

	hst_mode = ts->xy_mode[CY_REG_BASE];
	rep_len = ts->xy_mode[ts->si_ofs.rep_ofs];
	rep_stat = ts->xy_mode[ts->si_ofs.rep_ofs + 1];
	tt_stat = ts->xy_data_touch1[0];
	dev_dbg(ts->dev,
		"%s: hst_mode=%02X rep_len=%d rep_stat=%02X tt_stat=%02X\n",
		__func__, hst_mode, rep_len, rep_stat, tt_stat);

	if (rep_len == 0) {
		dev_err(ts->dev,
			"%s: report length error rep_len=%d\n",
			__func__, rep_len);
		goto _cyttsp4_xy_worker_exit;
	}

	if (GET_NUM_TOUCHES(tt_stat) > 0) {
		memcpy(ts->xy_data, ts->xy_data_touch1 + 1,
			ts->si_ofs.tch_rec_siz);
	}
	if (GET_NUM_TOUCHES(tt_stat) > 1) {
		retval = _cyttsp4_read_block_data(ts, ts->si_ofs.tt_stat_ofs+1 +
			ts->si_ofs.tch_rec_siz,
			(GET_NUM_TOUCHES(tt_stat) - 1) * ts->si_ofs.tch_rec_siz,
			ts->xy_data + ts->si_ofs.tch_rec_siz,
			ts->platform_data->addr[CY_TCH_ADDR_OFS], true);
		if (retval < 0) {
			dev_err(ts->dev,
			"%s: read fail on touch regs r=%d\n",
				__func__, retval);
			goto _cyttsp4_xy_worker_exit;
		}
	}

	/* provide flow control handshake */
	retval = _cyttsp4_handshake(ts, hst_mode);
	if (retval < 0) {
		dev_err(ts->dev,
			"%s: handshake fail on operational reg\n",
			__func__);
		/* continue; rely on handshake tmo */
		retval = 0;
	}

	/* determine number of currently active touches */
	num_cur_tch = GET_NUM_TOUCHES(tt_stat);

	/* print xy data */
	_cyttsp4_pr_buf(ts, ts->xy_data, num_cur_tch *
		ts->si_ofs.tch_rec_siz, "xy_data");

	/* check for any error conditions */
	if (ts->driver_state == CY_IDLE_STATE) {
		dev_err(ts->dev,
			"%s: IDLE STATE detected\n", __func__);
		retval = 0;
		goto _cyttsp4_xy_worker_exit;
	} else if (IS_BAD_PKT(rep_stat)) {
		dev_err(ts->dev,
			"%s: Invalid buffer detected\n", __func__);
		retval = 0;
		goto _cyttsp4_xy_worker_exit;
	} else if (IS_BOOTLOADERMODE(rep_stat)) {
		dev_info(ts->dev,
			"%s: BL mode found in ACTIVE state\n",
			__func__);
		retval = -EIO;
		goto _cyttsp4_xy_worker_exit;
	} else if (GET_HSTMODE(hst_mode) == GET_HSTMODE(CY_SYSINFO_MODE)) {
		/* if in sysinfo mode switch to op mode */
		dev_err(ts->dev,
			"%s: Sysinfo mode=0x%02X detected in ACTIVE state\n",
			__func__, hst_mode);
		retval = _cyttsp4_set_mode(ts, CY_OPERATE_MODE);
		if (retval < 0) {
			_cyttsp4_change_state(ts, CY_IDLE_STATE);
			dev_err(ts->dev,
			"%s: Fail set operational mode (r=%d)\n",
				__func__, retval);
		} else {
			_cyttsp4_change_state(ts, CY_ACTIVE_STATE);
			dev_vdbg(ts->dev,
				"%s: enable handshake\n", __func__);
#ifdef CY_USE_TMA884
			retval = _cyttsp4_handshake_enable(ts);
			if (retval < 0) {
				dev_err(ts->dev,
			"%s: fail enable handshake r=%d",
					__func__, retval);
			}
#endif /* --CY_USE_TMA884 */
		}
		goto _cyttsp4_xy_worker_exit;
	} else if (IS_LARGE_AREA(tt_stat)) {
		/* terminate all active tracks */
		num_cur_tch = 0;
		dev_dbg(ts->dev, "%s: Large area detected\n", __func__);
	} else if (num_cur_tch > ts->si_ofs.max_tchs) {
		if (num_cur_tch == 0x1F) {
			/* terminate all active tracks */
			dev_err(ts->dev,
			"%s: Num touch err detected (n=%d)\n",
				__func__, num_cur_tch);
			num_cur_tch = 0;
		} else {
			dev_err(ts->dev,
			"%s: too many tch; set to max tch (n=%d c=%d)\n",
				__func__, num_cur_tch, CY_NUM_TCH_ID);
			num_cur_tch = CY_NUM_TCH_ID;
		}
	}

	dev_vdbg(ts->dev,
		"%s: num_cur_tch=%d\n", __func__, num_cur_tch);

	/* extract xy_data for all currently reported touches */
	if (num_pre_tch || num_cur_tch) {
		for (id = 0; id < CY_NUM_TCH_ID; id++) {
			for (i = 0; i < num_cur_tch; i++) {
				_cyttsp4_get_touch(ts, &touch,
						ts->xy_data + (i * ts->si_ofs.tch_rec_siz));
				if (touch.t == id)
					break;
			}
			if (i >= num_cur_tch) {
				if (pre_touch[id].p == 0)
					continue;
				pre_touch[id].p = 0;

				input_mt_slot(ts->input, id);
				input_mt_report_slot_state(ts->input, MT_TOOL_FINGER, false);
				continue;
			}

#if defined(CONFIG_SHTPS_TMA4XX_TMA463_001)
			/* TODO: not use magic number `2' */
			touch.x = touch.x / 2;
			touch.y = touch.y / 2;
#elif defined(CONFIG_SHTPS_TMA4XX_TMA463_003)
			if (_sh_tpsif_firmware_version(ts, &version) < 0) {
				dev_err(ts->dev, "%s: failed to get firmware version\n", __func__);
			} else {
				/* if the firmware version doesn't equal to 01.00.000[0-3] */
				if (!(((u8 *)&version)[3] == 0x01 &&
						((u8 *)&version)[2] == 0x00 &&
						((u8 *)&version)[1] == 0x00 &&
						(((u8 *)&version)[0] >> 2) == 0x00)) {
					touch.x = touch.x / 2;
					touch.y = touch.y / 2;
				}
			}
#endif

#ifdef SH_TPSIF_COMMAND
			/* adjusting the coordinates */
			if (ts->adjust_enabled == true)
				_sh_tpsif_adjust_point(ts, &(touch.x), &(touch.y));
#endif	/* SH_TPSIF_COMMAND */

			if ((touch.t < ts->platform_data->frmwrk->abs
				[(CY_ABS_ID_OST*CY_NUM_ABS_SET)+CY_MIN_OST]) ||
				(touch.t > ts->platform_data->frmwrk->abs
				[(CY_ABS_ID_OST*CY_NUM_ABS_SET)+CY_MAX_OST])) {
				dev_err(ts->dev,
					"%s: touch=%d has bad track_id=%d"
					"max_id=%d\n",
					__func__, i, touch.t,
					ts->platform_data->frmwrk->abs
					[(CY_ABS_ID_OST*CY_NUM_ABS_SET)+
					CY_MAX_OST]);
			} else {
				/* use 0 based track id's */
				signal = ts->platform_data->frmwrk->abs
					[(CY_ABS_ID_OST*CY_NUM_ABS_SET)+0];
				if (signal != CY_IGNORE_VALUE) {
					t = touch.t -
						ts->platform_data->frmwrk->abs
						[(CY_ABS_ID_OST*CY_NUM_ABS_SET)+
						CY_MIN_OST];
					input_mt_slot(ts->input, t);
					input_mt_report_slot_state(ts->input, MT_TOOL_FINGER, true);
				}

				signal = ts->platform_data->frmwrk->abs
					[(CY_ABS_X_OST*CY_NUM_ABS_SET)+0];
				if (signal != CY_IGNORE_VALUE)
					input_report_abs(ts->input,
						signal, touch.x);

				signal = ts->platform_data->frmwrk->abs
					[(CY_ABS_Y_OST*CY_NUM_ABS_SET)+0];
#ifdef REVERSE_Y_AXIS
				if (signal != CY_IGNORE_VALUE) {
					input_report_abs(ts->input,
							signal,
							ts->platform_data->frmwrk->abs
							[(CY_ABS_Y_OST*CY_NUM_ABS_SET)+CY_MAX_OST] +
							ts->platform_data->frmwrk->abs
							[(CY_ABS_Y_OST*CY_NUM_ABS_SET)+CY_MIN_OST] -
							touch.y);
				}
#else
				if (signal != CY_IGNORE_VALUE)
					input_report_abs(ts->input,
						signal, touch.y);
#endif

				signal = ts->platform_data->frmwrk->abs
					[(CY_ABS_P_OST*CY_NUM_ABS_SET)+0];
				if (signal != CY_IGNORE_VALUE)
					input_report_abs(ts->input,
						signal, touch.p);

				signal = ts->platform_data->frmwrk->abs
					[(CY_ABS_W_OST*CY_NUM_ABS_SET)
					+0];
				if (signal != CY_IGNORE_VALUE)
					input_report_abs(ts->input,
						signal, touch.w);
			}
			dev_dbg(ts->dev,
				"%s: t=%d x=%04X y=%04X z=%02X"
				" w=%02X e=%02X o=%02X\n", __func__,
				t, touch.x, touch.y,
				touch.p, touch.w, touch.e, touch.o);
		}
		input_sync(ts->input);
	} else if (num_cur_tch == 0) {
		/* input_sync(ts->input); */
	}

	/* store current touch */
	for (i = 0; i < num_cur_tch; i++) {
		_cyttsp4_get_touch(ts, &touch,
				ts->xy_data + (i * ts->si_ofs.tch_rec_siz));
		if (touch.t < CY_NUM_TCH_ID)
			pre_touch[touch.t] = touch;
	}
	num_pre_tch = num_cur_tch;

	dev_dbg(ts->dev, "%s:\n", __func__);
	retval = 0;
_cyttsp4_xy_worker_exit:
#ifdef CY_USE_LEVEL_IRQ
	udelay(500);
#endif
	return retval;
}

#ifdef CY_USE_WATCHDOG
#define CY_TIMEOUT msecs_to_jiffies(1000)
static void _cyttsp4_start_wd_timer(struct cyttsp4 *ts)
{
	mod_timer(&ts->timer, jiffies + CY_TIMEOUT);

	return;
}

static void _cyttsp4_stop_wd_timer(struct cyttsp4 *ts)
{
	del_timer(&ts->timer);
	cancel_work_sync(&ts->work);

	return;
}

static void cyttsp4_timer_watchdog(struct work_struct *work)
{
	struct cyttsp4 *ts = container_of(work, struct cyttsp4, work);
	u8 rep_stat = 0;
	int retval = 0;

	if (ts == NULL) {
		dev_err(ts->dev,
			"%s: NULL context pointer\n", __func__);
		return;
	}

	mutex_lock(&ts->data_lock);
	if (ts->driver_state == CY_ACTIVE_STATE) {
		retval = _cyttsp4_load_status_regs(ts);
		if (retval < 0) {
			dev_err(ts->dev,
			"%s: failed to access device"
				" in watchdog timer r=%d\n", __func__, retval);
			_cyttsp4_queue_startup(ts, false);
			goto cyttsp4_timer_watchdog_exit_error;
		}
		rep_stat = ts->xy_mode[ts->si_ofs.rep_ofs + 1];
		if (IS_BOOTLOADERMODE(rep_stat)) {
			dev_err(ts->dev,
			"%s: device found in bootloader mode"
				" when operational mode rep_stat=0x%02X\n",
				__func__, rep_stat);
			_cyttsp4_queue_startup(ts, false);
			goto cyttsp4_timer_watchdog_exit_error;
		}
	}

	_cyttsp4_start_wd_timer(ts);
 cyttsp4_timer_watchdog_exit_error:
	mutex_unlock(&ts->data_lock);
	return;
}

static void cyttsp4_timer(unsigned long handle)
{
	struct cyttsp4 *ts = (struct cyttsp4 *)handle;

	if (!work_pending(&ts->work))
		schedule_work(&ts->work);

	return;
}
#endif

static int _cyttsp4_soft_reset(struct cyttsp4 *ts)
{
	u8 cmd = CY_SOFT_RESET_MODE;

	return _cyttsp4_write_block_data(ts, CY_REG_BASE,
		sizeof(cmd), &cmd,
		ts->platform_data->addr[CY_TCH_ADDR_OFS], true);
}

static int _cyttsp4_reset(struct cyttsp4 *ts)
{
	enum cyttsp4_driver_state tmp_state = ts->driver_state;
	int retval = 0;

	if (ts->platform_data->hw_reset) {
		retval = ts->platform_data->hw_reset();
		if (retval == -ENOSYS) {
			retval = _cyttsp4_soft_reset(ts);
			ts->soft_reset_asserted = true;
		} else
			ts->soft_reset_asserted = false;
	} else {
		retval = _cyttsp4_soft_reset(ts);
		ts->soft_reset_asserted = true;
	}

	if (retval < 0) {
		_cyttsp4_pr_state(ts);
		return retval;
	} else {
		ts->current_mode = CY_MODE_BOOTLOADER;
		ts->driver_state = CY_BL_STATE;
		if (tmp_state != CY_BL_STATE)
			_cyttsp4_pr_state(ts);
		return retval;
	}
}

static void cyttsp4_ts_work_func(struct work_struct *work)
{
	struct cyttsp4 *ts =
		container_of(work, struct cyttsp4, cyttsp4_resume_startup_work);
	int retval = 0;

	mutex_lock(&ts->data_lock);

	retval = _cyttsp4_startup(ts);
	if (retval < 0) {
		dev_err(ts->dev,
			"%s: Startup failed with error code %d\n",
			__func__, retval);
		_cyttsp4_change_state(ts, CY_IDLE_STATE);
#ifdef CY_USE_WATCHDOG
	} else {
		_cyttsp4_start_wd_timer(ts);
#endif
	}

	mutex_unlock(&ts->data_lock);

	return;
}

static int _cyttsp4_enter_sleep(struct cyttsp4 *ts)
{
	int retval = 0;
#if defined(CONFIG_PM_SLEEP) || \
	defined(CONFIG_PM) || \
	defined(CONFIG_HAS_EARLYSUSPEND)
	uint8_t sleep = CY_DEEP_SLEEP_MODE;

	dev_vdbg(ts->dev,
		"%s: Put the part back to sleep\n", __func__);

	retval = _cyttsp4_write_block_data(ts, CY_REG_BASE,
		sizeof(sleep), &sleep,
		ts->platform_data->addr[CY_TCH_ADDR_OFS], true);
	if (retval < 0) {
		dev_err(ts->dev,
			"%s: Failed to write sleep bit r=%d\n",
			__func__, retval);
	} else
		_cyttsp4_change_state(ts, CY_SLEEP_STATE);
#endif
	return retval;
}

static int _cyttsp4_wakeup(struct cyttsp4 *ts)
{
	int retval = 0;
#if defined(CONFIG_PM_SLEEP) || \
	defined(CONFIG_PM) || \
	defined(CONFIG_HAS_EARLYSUSPEND)
	unsigned long timeout = 0;
	unsigned long uretval = 0;
	u8 hst_mode = 0;
#ifdef CY_USE_TMA400
	u8 rep_stat = 0;
#endif /* --CY_USE_TMA400 */
	int wake = CY_WAKE_DFLT;

	_cyttsp4_change_state(ts, CY_CMD_STATE);
	INIT_COMPLETION(ts->int_running);
	if (ts->platform_data->hw_recov == NULL) {
		dev_vdbg(ts->dev,
			"%s: no hw_recov function\n", __func__);
		retval = -ENOSYS;
	} else {
		/* wake using strobe on host alert pin */
		retval = ts->platform_data->hw_recov(wake);
		if (retval < 0) {
			if (retval == -ENOSYS) {
				dev_vdbg(ts->dev,
					"%s: no hw_recov wake code=%d"
					" function\n", __func__, wake);
			} else {
				dev_err(ts->dev,
			"%s: fail hw_recov(wake=%d)"
					" function r=%d\n",
					__func__, wake, retval);
				retval = -ENOSYS;
			}
		}
	}

	if (retval == -ENOSYS) {
		/*
		 * Wake the chip with bus traffic
		 * The first few reads should always fail because
		 * the part is not ready to respond,
		 * but the retries should succeed.
		 */
		/*
		 * Even though this is hardware-specific, it is done
		 * here because the board config file doesn't have
		 * access to the bus read routine
		 */
		retval = _cyttsp4_read_block_data(ts, CY_REG_BASE,
			sizeof(hst_mode), &hst_mode,
			ts->platform_data->addr[CY_TCH_ADDR_OFS],
			true);
		if (retval < 0) {
			/* device may not be ready even with the
			 * bus read retries so just go ahead and
			 * wait for the cmd rdy interrupt or timeout
			 */
			retval = 0;
		} else {
			/* IC is awake but still need to check for
			 * proper mode
			 */
		}
	} else
		retval = 0;

	/* Wait for cmd rdy interrupt to signal device wake */
	timeout = msecs_to_jiffies(CY_HALF_SEC_TMO_MS);
	mutex_unlock(&ts->data_lock);
	uretval = wait_for_completion_interruptible_timeout(
		&ts->int_running, timeout);
	mutex_lock(&ts->data_lock);

	/* read registers even if wait ended with timeout */
	retval = _cyttsp4_read_block_data(ts,
		CY_REG_BASE, sizeof(hst_mode), &hst_mode,
		ts->platform_data->addr[CY_TCH_ADDR_OFS], true);

	/* TMA884 indicates bootloader mode by changing addr */
	if (retval < 0) {
		dev_err(ts->dev,
			"%s: failed to resume or in bootloader (r=%d)\n",
			__func__, retval);
	} else {
#ifdef CY_USE_TMA400
		/* read rep stat register for bootloader status */
		retval = _cyttsp4_load_status_regs(ts);
		if (retval < 0) {
			dev_err(ts->dev,
			"%s: failed to access device on resume r=%d\n",
				__func__, retval);
			goto _cyttsp4_wakeup_exit;
		}
		rep_stat = ts->xy_mode[ts->si_ofs.rep_ofs + 1];
		if (IS_BOOTLOADERMODE(rep_stat)) {
			dev_err(ts->dev,
			"%s: device in bootloader mode on wakeup"
				" rep_stat=0x%02X\n",
				__func__, rep_stat);
			retval = -EIO;
			goto _cyttsp4_wakeup_exit;
		}
#endif /* --CY_USE_TMA400 */
		retval = _cyttsp4_handshake(ts, hst_mode);
		if (retval < 0) {
			dev_err(ts->dev,
			"%s: fail resume INT handshake (r=%d)\n",
				__func__, retval);
			/* continue; rely on handshake tmo */
			retval = 0;
		}
		_cyttsp4_change_state(ts, CY_ACTIVE_STATE);
	}
#ifdef CY_USE_TMA400
_cyttsp4_wakeup_exit:
#endif /* --CY_USE_TMA400 */
#endif
	return retval;
}

#if defined(CONFIG_PM) || \
	defined(CONFIG_PM_SLEEP) || \
	defined(CONFIG_HAS_EARLYSUSPEND)

#if defined(CONFIG_HAS_EARLYSUSPEND)
int cyttsp4_suspend(void *handle)
{
	struct cyttsp4 *ts = handle;
#elif defined(CONFIG_PM_SLEEP)
static int cyttsp4_suspend(struct device *dev)
{
	struct cyttsp4 *ts = dev_get_drvdata(dev);
#else
int cyttsp4_suspend(void *handle)
{
	struct cyttsp4 *ts = handle;
#endif
	int retval = 0;

	switch (ts->driver_state) {
	case CY_ACTIVE_STATE:
#if defined(CY_USE_FORCE_LOAD) || defined(CONFIG_TOUCHSCREEN_DEBUG) || defined(SH_TPSIF_COMMAND)
		if (ts->waiting_for_fw) {
			retval = -EBUSY;
			dev_err(ts->dev,
			"%s: Suspend Blocked while waiting for"
				" fw load in %s state\n", __func__,
				cyttsp4_driver_state_string[ts->driver_state]);
			break;
		}
#endif
		dev_vdbg(ts->dev, "%s: Suspending...\n", __func__);
#ifdef CY_USE_WATCHDOG
		_cyttsp4_stop_wd_timer(ts);
#endif
		if (ts->irq_enabled)
			disable_irq(ts->irq);

		mutex_lock(&ts->data_lock);

		retval = _cyttsp4_enter_sleep(ts);
		if (retval < 0) {
			dev_err(ts->dev,
			"%s: fail enter sleep r=%d\n",
				__func__, retval);
		} else
			_cyttsp4_change_state(ts, CY_SLEEP_STATE);

		mutex_unlock(&ts->data_lock);

		break;
	case CY_SLEEP_STATE:
		dev_err(ts->dev,
			"%s: already in Sleep state\n", __func__);
		break;
	/*
	 * These states could be changing the device state
	 * Some of these states don't directly change device state
	 * but the next state could happen at any time and that
	 * state DOES modify the device state
	 * they must complete before allowing suspend.
	 */
	case CY_BL_STATE:
	case CY_CMD_STATE:
	case CY_SYSINFO_STATE:
	case CY_READY_STATE:
	case CY_TRANSFER_STATE:
		retval = -EBUSY;
		dev_err(ts->dev,
			"%s: Suspend Blocked while in %s state\n", __func__,
			cyttsp4_driver_state_string[ts->driver_state]);
		break;
	case CY_IDLE_STATE:
	case CY_INVALID_STATE:
	default:
		dev_err(ts->dev,
			"%s: Cannot enter suspend from %s state\n", __func__,
			cyttsp4_driver_state_string[ts->driver_state]);
		break;
	}

	return retval;
}
EXPORT_SYMBOL_GPL(cyttsp4_suspend);

#if defined(CONFIG_HAS_EARLYSUSPEND)
int cyttsp4_resume(void *handle)
{
	struct cyttsp4 *ts = handle;
#elif defined(CONFIG_PM_SLEEP)
static int cyttsp4_resume(struct device *dev)
{
	struct cyttsp4 *ts = dev_get_drvdata(dev);
#else
int cyttsp4_resume(void *handle)
{
	struct cyttsp4 *ts = handle;
#endif
	int retval = 0;

	dev_vdbg(ts->dev, "%s: Resuming...\n", __func__);

	mutex_lock(&ts->data_lock);

#ifdef CY_USE_LEVEL_IRQ
	/* workaround level interrupt unmasking issue */
	if (ts->irq_enabled) {
		disable_irq_nosync(ts->irq);
		udelay(5);
		enable_irq(ts->irq);
	}
#endif

	switch (ts->driver_state) {
	case CY_SLEEP_STATE:
		retval = _cyttsp4_wakeup(ts);
		if (retval < 0) {
			dev_err(ts->dev,
			"%s: wakeup fail r=%d\n",
				__func__, retval);
			_cyttsp4_pr_state(ts);
			if (ts->irq_enabled)
				enable_irq(ts->irq);
			_cyttsp4_queue_startup(ts, false);
			break;
		}
		_cyttsp4_change_state(ts, CY_ACTIVE_STATE);
		if (ts->irq_enabled)
			enable_irq(ts->irq);
#ifdef CY_USE_WATCHDOG
		_cyttsp4_start_wd_timer(ts);
#endif
		break;
	case CY_IDLE_STATE:
	case CY_READY_STATE:
	case CY_ACTIVE_STATE:
	case CY_BL_STATE:
	case CY_SYSINFO_STATE:
	case CY_CMD_STATE:
	case CY_TRANSFER_STATE:
	case CY_INVALID_STATE:
	default:
		dev_err(ts->dev,
			"%s: Already in %s state\n", __func__,
			cyttsp4_driver_state_string[ts->driver_state]);
		break;
	}

	mutex_unlock(&ts->data_lock);

	dev_vdbg(ts->dev,
		"%s: exit Resume r=%d\n", __func__, retval);
	return retval;
}
EXPORT_SYMBOL_GPL(cyttsp4_resume);
#endif
#if !defined(CONFIG_HAS_EARLYSUSPEND) && defined(CONFIG_PM_SLEEP)
const struct dev_pm_ops cyttsp4_pm_ops = {
	SET_SYSTEM_SLEEP_PM_OPS(cyttsp4_suspend, cyttsp4_resume)
};
EXPORT_SYMBOL_GPL(cyttsp4_pm_ops);
#endif


#if defined(CONFIG_HAS_EARLYSUSPEND)
void cyttsp4_early_suspend(struct early_suspend *h)
{
	struct cyttsp4 *ts = container_of(h, struct cyttsp4, early_suspend);
	int retval = 0;

	dev_vdbg(ts->dev, "%s: EARLY SUSPEND ts=%p\n",
		__func__, ts);
	retval = cyttsp4_suspend(ts);
	if (retval < 0) {
		dev_err(ts->dev,
			"%s: Early suspend failed with error code %d\n",
			__func__, retval);
	}
}
void cyttsp4_late_resume(struct early_suspend *h)
{
	struct cyttsp4 *ts = container_of(h, struct cyttsp4, early_suspend);
	int retval = 0;

	dev_vdbg(ts->dev, "%s: LATE RESUME ts=%p\n",
		__func__, ts);
	retval = cyttsp4_resume(ts);
	if (retval < 0) {
		dev_err(ts->dev,
			"%s: Late resume failed with error code %d\n",
			__func__, retval);
	}
}
#endif

#if !defined(CY_NO_AUTO_LOAD)
static int _cyttsp4_boot_loader(struct cyttsp4 *ts, bool *upgraded)
{
	int retval = 0;
	int i = 0;
	u32 fw_vers_platform = 0;
	u32 fw_vers_img = 0;
	u32 fw_revctrl_platform_h = 0;
	u32 fw_revctrl_platform_l = 0;
	u32 fw_revctrl_img_h = 0;
	u32 fw_revctrl_img_l = 0;
	bool new_fw_vers = false;
	bool new_fw_revctrl = false;
	bool new_vers = false;

	*upgraded = false;
	if (ts->driver_state == CY_SLEEP_STATE) {
		dev_err(ts->dev,
			"%s: cannot load firmware in sleep state\n",
			__func__);
		retval = 0;
	} else if ((ts->platform_data->fw->ver == NULL) ||
		(ts->platform_data->fw->img == NULL)) {
		dev_err(ts->dev,
			"%s: empty version list or no image\n",
			__func__);
		retval = 0;
	} else if (ts->platform_data->fw->vsize != CY_BL_VERS_SIZE) {
		dev_err(ts->dev,
			"%s: bad fw version list size=%d\n",
			__func__, ts->platform_data->fw->vsize);
		retval = 0;
	} else {
		/* automatically update firmware if new version detected */
		fw_vers_img = (ts->sysinfo_ptr.cydata->fw_ver_major * 256);
		fw_vers_img += ts->sysinfo_ptr.cydata->fw_ver_minor;
		fw_vers_platform = ts->platform_data->fw->ver[2] * 256;
		fw_vers_platform += ts->platform_data->fw->ver[3];
#ifdef CY_ANY_DIFF_NEW_VER
		if (fw_vers_platform != fw_vers_img)
			new_fw_vers = true;
		else
			new_fw_vers = false;
#else
		if (fw_vers_platform > fw_vers_img)
			new_fw_vers = true;
		else
			new_fw_vers = false;
#endif
		dev_vdbg(ts->dev,
			"%s: fw_vers_platform=%04X fw_vers_img=%04X\n",
			__func__, fw_vers_platform, fw_vers_img);

#if 0
		fw_revctrl_img_h = ts->sysinfo_ptr.cydata->revctrl[0];
		fw_revctrl_img_l = ts->sysinfo_ptr.cydata->revctrl[4];
#else
		fw_revctrl_img_h = ts->sysinfo_ptr.cydata->cyito_verh;
		fw_revctrl_img_l = ts->sysinfo_ptr.cydata->cyito_verl;
#endif
		fw_revctrl_platform_h = ts->platform_data->fw->ver[4];
		fw_revctrl_platform_l = ts->platform_data->fw->ver[8];
		for (i = 1; i < 4; i++) {
#if 0
			fw_revctrl_img_h = (fw_revctrl_img_h * 256) +
				ts->sysinfo_ptr.cydata->revctrl[0+i];
			fw_revctrl_img_l = (fw_revctrl_img_l * 256) +
				ts->sysinfo_ptr.cydata->revctrl[4+i];
#endif
			fw_revctrl_platform_h = (fw_revctrl_platform_h * 256) +
				ts->platform_data->fw->ver[4+i];
			fw_revctrl_platform_l = (fw_revctrl_platform_l * 256) +
				ts->platform_data->fw->ver[8+i];
		}
#ifdef CY_ANY_DIFF_NEW_VER
		if (fw_revctrl_platform_h != fw_revctrl_img_h)
			new_fw_revctrl = true;
		else if (fw_revctrl_platform_h == fw_revctrl_img_h) {
			if (fw_revctrl_platform_l != fw_revctrl_img_l)
				new_fw_revctrl = true;
			else
				new_fw_revctrl = false;
		} else
			new_fw_revctrl = false;
#else
		if (fw_revctrl_platform_h > fw_revctrl_img_h)
			new_fw_revctrl = true;
		else if (fw_revctrl_platform_h == fw_revctrl_img_h) {
			if (fw_revctrl_platform_l > fw_revctrl_img_l)
				new_fw_revctrl = true;
			else
				new_fw_revctrl = false;
		} else
			new_fw_revctrl = false;
#endif
		if (new_fw_vers || new_fw_revctrl)
			new_vers = true;

		dev_vdbg(ts->dev,
			"%s: fw_revctrl_platform_h=%08X"
			" fw_revctrl_img_h=%08X\n", __func__,
			fw_revctrl_platform_h, fw_revctrl_img_h);
		dev_vdbg(ts->dev,
			"%s: fw_revctrl_platform_l=%08X"
			" fw_revctrl_img_l=%08X\n", __func__,
			fw_revctrl_platform_l, fw_revctrl_img_l);
		dev_vdbg(ts->dev,
			"%s: new_fw_vers=%d new_fw_revctrl=%d new_vers=%d\n",
			__func__,
			(int)new_fw_vers, (int)new_fw_revctrl, (int)new_vers);

		if (new_vers) {
			dev_info(ts->dev,
			"%s: upgrading firmware...\n", __func__);
			retval = _cyttsp4_load_app(ts,
				ts->platform_data->fw->img,
				ts->platform_data->fw->size);
			if (retval < 0) {
				dev_err(ts->dev,
			"%s: communication fail"
					" on load fw r=%d\n",
					__func__, retval);
				_cyttsp4_change_state(ts, CY_IDLE_STATE);
				retval = -EIO;
			} else
				*upgraded = true;
		} else {
			dev_vdbg(ts->dev,
				"%s: No auto firmware upgrade required\n",
				__func__);
		}
	}

	return retval;
}
#endif

static ssize_t cyttsp4_ic_ver_show(struct device *dev,
		struct device_attribute *attr, char *buf)
{
	struct cyttsp4 *ts = dev_get_drvdata(dev);

	return sprintf(buf, "%s: 0x%02X 0x%02X\n%s: 0x%02X\n%s: 0x%02X\n%s: "
		"0x%02X 0x%02X 0x%02X 0x%02X 0x%02X 0x%02X 0x%02X 0x%02X\n",
		"TrueTouch Product ID",
		ts->sysinfo_ptr.cydata->ttpidh,
		ts->sysinfo_ptr.cydata->ttpidl,
		"Firmware Major Version", ts->sysinfo_ptr.cydata->fw_ver_major,
		"Firmware Minor Version", ts->sysinfo_ptr.cydata->fw_ver_minor,
		"Revision Control Number", ts->sysinfo_ptr.cydata->revctrl[0],
		ts->sysinfo_ptr.cydata->revctrl[1],
		ts->sysinfo_ptr.cydata->revctrl[2],
		ts->sysinfo_ptr.cydata->revctrl[3],
		ts->sysinfo_ptr.cydata->revctrl[4],
		ts->sysinfo_ptr.cydata->revctrl[5],
		ts->sysinfo_ptr.cydata->revctrl[6],
		ts->sysinfo_ptr.cydata->revctrl[7]);
}
static DEVICE_ATTR(ic_ver, S_IRUGO, cyttsp4_ic_ver_show, NULL);

/* Driver version */
static ssize_t cyttsp4_drv_ver_show(struct device *dev,
		struct device_attribute *attr, char *buf)
{
	struct cyttsp4 *ts = dev_get_drvdata(dev);

	return snprintf(buf, CY_MAX_PRBUF_SIZE,
		"Driver: %s\nVersion: %s\nDate: %s\n",
		ts->input->name, CY_DRIVER_VERSION, CY_DRIVER_DATE);
}
static DEVICE_ATTR(drv_ver, S_IRUGO, cyttsp4_drv_ver_show, NULL);


/* Driver status */
static ssize_t cyttsp4_drv_stat_show(struct device *dev,
		struct device_attribute *attr, char *buf)
{
	struct cyttsp4 *ts = dev_get_drvdata(dev);

	return snprintf(buf, CY_MAX_PRBUF_SIZE,
		"Driver state is %s\n",
		cyttsp4_driver_state_string[ts->driver_state]);
}
static DEVICE_ATTR(drv_stat, S_IRUGO, cyttsp4_drv_stat_show, NULL);

#ifdef CONFIG_TOUCHSCREEN_DEBUG
static ssize_t cyttsp_ic_irq_stat_show(struct device *dev,
	struct device_attribute *attr, char *buf)
{
	int retval;
	struct cyttsp4 *ts = dev_get_drvdata(dev);

	if (ts->platform_data->irq_stat) {
		retval = ts->platform_data->irq_stat();
		switch (retval) {
		case 0:
			return snprintf(buf, CY_MAX_PRBUF_SIZE,
				"Interrupt line is LOW.\n");
		case 1:
			return snprintf(buf, CY_MAX_PRBUF_SIZE,
				"Interrupt line is HIGH.\n");
		default:
			return snprintf(buf, CY_MAX_PRBUF_SIZE,
				"Function irq_stat() returned %d.\n", retval);
		}
	} else {
		return snprintf(buf, CY_MAX_PRBUF_SIZE,
			"Function irq_stat() undefined.\n");
	}
}
static DEVICE_ATTR(hw_irqstat, S_IRUSR | S_IWUSR,
	cyttsp_ic_irq_stat_show, NULL);
#endif /* --CONFIG_TOUCHSCREEN_DEBUG */

#ifdef CONFIG_TOUCHSCREEN_DEBUG
/* Disable Driver IRQ */
static ssize_t cyttsp4_drv_irq_show(struct device *dev,
		struct device_attribute *attr, char *buf)
{
	static const char *fmt_disabled = "Driver interrupt is DISABLED\n";
	static const char *fmt_enabled = "Driver interrupt is ENABLED\n";
	struct cyttsp4 *ts = dev_get_drvdata(dev);

	if (ts->irq_enabled == false)
		return snprintf(buf, strlen(fmt_disabled)+1, fmt_disabled);
	else
		return snprintf(buf, strlen(fmt_enabled)+1, fmt_enabled);
}
static ssize_t cyttsp4_drv_irq_store(struct device *dev,
		struct device_attribute *attr, const char *buf, size_t size)
{
	int retval = 0;
	struct cyttsp4 *ts = dev_get_drvdata(dev);
	unsigned long value;

	mutex_lock(&(ts->data_lock));

	if (size > 2) {
		dev_err(ts->dev,
			"%s: Err, data too large\n", __func__);
		retval = -EOVERFLOW;
		goto cyttsp4_drv_irq_store_error_exit;
	}

	retval = strict_strtoul(buf, 10, &value);
	if (retval < 0) {
		dev_err(ts->dev,
			"%s: Failed to convert value\n", __func__);
		goto cyttsp4_drv_irq_store_error_exit;
	}

	if (ts->irq_enabled == false) {
		if (value == 1) {
			/* Enable IRQ */
			enable_irq(ts->irq);
			dev_info(ts->dev,
			"%s: Driver IRQ now enabled\n", __func__);
			ts->irq_enabled = true;
		} else {
			dev_info(ts->dev,
			"%s: Driver IRQ already disabled\n", __func__);
		}
	} else {
		if (value == 0) {
			/* Disable IRQ */
			disable_irq_nosync(ts->irq);
			dev_info(ts->dev,
			"%s: Driver IRQ now disabled\n", __func__);
			ts->irq_enabled = false;
		} else {
			dev_info(ts->dev,
			"%s: Driver IRQ already enabled\n", __func__);
		}
	}

	retval = size;

cyttsp4_drv_irq_store_error_exit:
	mutex_unlock(&(ts->data_lock));
	return retval;
}
static DEVICE_ATTR(drv_irq, S_IRUSR | S_IWUSR,
	cyttsp4_drv_irq_show, cyttsp4_drv_irq_store);
#endif /* --CONFIG_TOUCHSCREEN_DEBUG */

#ifdef CONFIG_TOUCHSCREEN_DEBUG
/* Driver debugging */
static ssize_t cyttsp4_drv_debug_show(struct device *dev,
		struct device_attribute *attr, char *buf)
{
	struct cyttsp4 *ts = dev_get_drvdata(dev);

#ifdef CY_USE_DEBUG_TOOLS
	return snprintf(buf, CY_MAX_PRBUF_SIZE,
		"Debug Setting: %u flip=%d inv-x=%d inv-y=%d\n",
		ts->bus_ops->tsdebug,
		(int)(!!(ts->flags & CY_FLIP)),
		(int)(!!(ts->flags & CY_INV_X)),
		(int)(!!(ts->flags & CY_INV_Y)));
#else
	return snprintf(buf, CY_MAX_PRBUF_SIZE,
		"Debug Setting: %u\n", ts->bus_ops->tsdebug);
#endif /* --CY_USE_DEBUG_TOOLS */
}
static ssize_t cyttsp4_drv_debug_store(struct device *dev,
		struct device_attribute *attr, const char *buf, size_t size)
{
	struct cyttsp4 *ts = dev_get_drvdata(dev);
	int retval = 0;
	unsigned long value = 0;

	retval = strict_strtoul(buf, 10, &value);
	if (retval < 0) {
		dev_err(ts->dev,
			"%s: Failed to convert value\n", __func__);
		goto cyttsp4_drv_debug_store_exit;
	}

	switch (value) {
	case CY_DBG_LVL_0:
	case CY_DBG_LVL_1:
	case CY_DBG_LVL_2:
	case CY_DBG_LVL_3:
		dev_info(ts->dev,
			"%s: Debug setting=%d\n", __func__, (int)value);
		ts->bus_ops->tsdebug = value;
		break;
#ifdef CY_USE_DEBUG_TOOLS
	case CY_DBG_SUSPEND:
		dev_info(ts->dev,
			"%s: SUSPEND (ts=%p)\n", __func__, ts);
#if defined(CONFIG_HAS_EARLYSUSPEND)
		cyttsp4_early_suspend(&ts->early_suspend);
#elif defined(CONFIG_PM_SLEEP)
		cyttsp4_suspend(ts->dev);
#elif defined(CONFIG_PM) || defined(CONFIG_HAS_EARLYSUSPEND)
		cyttsp4_suspend(ts);
#endif
		break;
	case CY_DBG_RESUME:
		dev_info(ts->dev,
			"%s: RESUME (ts=%p)\n", __func__, ts);
#if defined(CONFIG_HAS_EARLYSUSPEND)
		cyttsp4_late_resume(&ts->early_suspend);
#elif defined(CONFIG_PM_SLEEP)
		cyttsp4_resume(ts->dev);
#elif defined(CONFIG_PM)
		cyttsp4_resume(ts);
#endif
		break;
	case CY_DBG_SOFT_RESET:
		dev_info(ts->dev,
			"%s: SOFT RESET (ts=%p)\n",
			__func__, ts);
		retval = _cyttsp4_soft_reset(ts);
		dev_info(ts->dev,
			"%s: return from _cyttsp4_soft_reset r=%d\n",
			__func__, retval);
		break;
	case CY_DBG_RESET:
		dev_info(ts->dev,
			"%s: RESET (ts=%p)\n",
			__func__, ts);
		mutex_lock(&ts->data_lock);
		retval = _cyttsp4_startup(ts);
		mutex_unlock(&ts->data_lock);
		dev_info(ts->dev,
			"%s: return from _cyttsp4_startup test r=%d\n",
			__func__, retval);
		break;
#ifdef CY_USE_DEV_DEBUG_TOOLS
	case CY_DBG_PUT_ALL_PARAMS:
#ifdef CY_USE_TMA400
	{
		enum cyttsp4_ic_ebid ebid = CY_TCH_PARM_EBID;
		u8 ic_crc[2];

		dev_info(ts->dev,
			"%s: PUT_ALL_PARAMS (ts=%p)\n",
			__func__, ts);
		mutex_lock(&ts->data_lock);
		memset(ic_crc, 0, sizeof(ic_crc));
		retval = _cyttsp4_set_mode(ts, CY_CONFIG_MODE);
		if (retval < 0) {
			dev_err(ts->dev,
			"%s: Fail switch to config mode r=%d\n",
				__func__, retval);
		} else {
			retval = _cyttsp4_put_all_params_tma400(ts);
			if (retval < 0) {
				dev_err(ts->dev,
			"%s: fail put all params r=%d\n",
					__func__, retval);
			} else {
				retval = _cyttsp4_calc_ic_crc_tma400(ts,
					ebid, &ic_crc[0], &ic_crc[1], true);
				if (retval < 0) {
					dev_err(ts->dev,
			"%s: fail verify params r=%d\n",
						__func__, retval);
				}
				_cyttsp4_pr_buf(ts, ic_crc, sizeof(ic_crc),
					"verify_params_ic_crc");
			}
			retval = _cyttsp4_set_mode(ts, CY_OPERATE_MODE);
			if (retval < 0) {
				dev_err(ts->dev,
			"%s: Fail switch op mode r=%d\n",
					__func__, retval);
			}
		}
		mutex_unlock(&ts->data_lock);
		break;
	}
#endif /* --CY_USE_TMA400 */
#ifdef CY_USE_TMA884
		dev_err(ts->dev,
			"%s: INVALID debug setting=%d\n", __func__, (int)value);
		break;
#endif /* --CY_USE_TMA884 */
	case CY_DBG_CHECK_MDDATA:
#ifdef CY_USE_TMA400
#ifdef CY_USE_REG_ACCESS
	{
		bool updated = false;

		dev_info(ts->dev,
			"%s: CHECK MDDATA ts=%p\n", __func__, ts);
		mutex_lock(&ts->data_lock);
		retval = _cyttsp4_set_mode(ts, CY_CONFIG_MODE);
		if (retval < 0) {
			dev_err(ts->dev,
			"%s: Fail switch config mode r=%d\n",
				__func__, retval);
		} else {
			dev_vdbg(ts->dev,
				"%s: call check_mddata ts=%p\n", __func__, ts);
			retval = _cyttsp4_check_mddata_tma400(ts, &updated);
			if (retval < 0) {
				dev_err(ts->dev,
			"%s: Fail Check mddata r=%d\n",
					__func__, retval);
			}
			dev_vdbg(ts->dev,
				"%s: mddata updated=%s\n", __func__,
				updated ? "true" : "false");
			retval = _cyttsp4_set_mode(ts, CY_OPERATE_MODE);
			if (retval < 0) {
				dev_err(ts->dev,
			"%s: Fail switch operate mode r=%d\n",
					__func__, retval);
			}
		}
		mutex_unlock(&ts->data_lock);
		break;
	}
#endif /* --CY_USE_REG_ACCESS */
#endif /* --CY_USE_TMA400 */
#ifdef CY_USE_TMA884
		dev_err(ts->dev,
			"%s: INVALID debug setting=%d\n", __func__, (int)value);
		break;
#endif /* --CY_USE_TMA884 */
	case CY_DBG_GET_MDDATA:
#ifdef CY_USE_TMA400
#ifdef CY_USE_REG_ACCESS
	{
		/* to use this command first use the set rw_regid
		 * to the ebid of interest 1:MDATA 2:DDATA
		 */
		enum cyttsp4_ic_ebid ebid = ts->rw_regid;
		u8 *pdata = NULL;

		dev_info(ts->dev,
			"%s: GET IC MDDATA=%d (ts=%p)\n",
			__func__, ebid, ts);
		mutex_lock(&ts->data_lock);
		retval = _cyttsp4_set_mode(ts, CY_CONFIG_MODE);
		if (retval < 0) {
			dev_err(ts->dev,
			"%s: Fail switch config mode r=%d\n",
				__func__, retval);
		} else {
			pdata = kzalloc(ts->ebid_row_size, GFP_KERNEL);
			if (pdata == NULL) {
				dev_err(ts->dev,
			"%s: Fail allocate block buffer\n",
					__func__);
				retval = -ENOMEM;
			} else {
				retval = _cyttsp4_get_ebid_data_tma400(ts,
					ebid, 0, pdata);
				if (retval < 0) {
					dev_err(ts->dev,
			"%s: Fail get touch ebid=%d"
						" data at row=0 r=%d\n",
						__func__, ebid, retval);
				}
				dev_vdbg(ts->dev,
					"%s: ebid=%d row=0 data:\n",
					__func__, ebid);
				_cyttsp4_pr_buf(ts, pdata, ts->ebid_row_size,
					"ebid_data");
				retval = _cyttsp4_set_mode(ts, CY_OPERATE_MODE);
				if (retval < 0) {
					dev_err(ts->dev,
			"%s: Fail switch operate mode"
						" r=%d\n", __func__, retval);
				}
			}
		}
		if (pdata != NULL)
			kfree(pdata);
		retval = _cyttsp4_set_mode(ts, CY_OPERATE_MODE);
		mutex_unlock(&ts->data_lock);
		break;
	}
#endif /* --CY_USE_REG_ACCESS */
#endif /* --CY_USE_TMA400 */
#ifdef CY_USE_TMA884
		dev_err(ts->dev,
			"%s: INVALID debug setting=%d\n", __func__, (int)value);
		break;
#endif /* --CY_USE_TMA884 */
	case CY_DBG_GET_IC_TCH_CRC:
#ifdef CY_USE_TMA400
	{
		u8 ic_crc[2];

		memset(ic_crc, 0, sizeof(ic_crc));
		dev_info(ts->dev,
			"%s: GET TOUCH CRC (ts=%p)\n",
			__func__, ts);
		mutex_lock(&ts->data_lock);
		retval = _cyttsp4_get_ic_crc(ts, CY_TCH_PARM_EBID,
			&ic_crc[0], &ic_crc[1]);
		if (retval < 0) {
			dev_err(ts->dev,
			"%s: Fail read ic crc r=%d\n",
				__func__, retval);
		}
		_cyttsp4_pr_buf(ts, ic_crc, sizeof(ic_crc), "read_ic_crc");
		mutex_unlock(&ts->data_lock);
		break;
	}
#endif /* --CY_USE_TMA400 */
#ifdef CY_USE_TMA884
		dev_err(ts->dev,
			"%s: INVALID debug setting=%d\n", __func__, (int)value);
		break;
#endif /* --CY_USE_TMA884 */
	case CY_DBG_GET_IC_TCH_ROW:
#ifdef CY_USE_TMA400
#ifdef CY_USE_REG_ACCESS
	{
		/* to use this command first use the set rw_regid
		 * to the row of interest
		 */
		u8 *pdata = NULL;

		dev_info(ts->dev,
			"%s: GET TOUCH BLOCK ROW=%d (ts=%p)\n",
			__func__, ts->rw_regid, ts);
		mutex_lock(&ts->data_lock);
		retval = _cyttsp4_set_mode(ts, CY_CONFIG_MODE);
		if (retval < 0) {
			dev_err(ts->dev,
			"%s: Fail switch config mode r=%d\n",
				__func__, retval);
			goto CY_DBG_GET_IC_TCH_ROW_exit;
		}
		pdata = kzalloc(ts->ebid_row_size, GFP_KERNEL);
		if (pdata == NULL) {
			dev_err(ts->dev,
			"%s: Fail allocate block buffer\n", __func__);
			retval = -ENOMEM;
			goto CY_DBG_GET_IC_TCH_ROW_exit;
		}
		retval = _cyttsp4_get_ebid_data_tma400(ts, CY_TCH_PARM_EBID,
			ts->rw_regid, pdata);
		if (retval < 0) {
			dev_err(ts->dev,
			"%s: Fail get touch ebid data at row=%d r=%d\n",
				__func__, ts->rw_regid, retval);
		}
		dev_vdbg(ts->dev,
			"%s: tch ebid row=%d data:\n", __func__, ts->rw_regid);
		_cyttsp4_pr_buf(ts, pdata, ts->ebid_row_size, "ebid_data");
CY_DBG_GET_IC_TCH_ROW_exit:
		if (pdata != NULL)
			kfree(pdata);
		retval = _cyttsp4_set_mode(ts, CY_OPERATE_MODE);
		mutex_unlock(&ts->data_lock);
		break;
	}
#endif /* --CY_USE_REG_ACCESS */
#endif /* --CY_USE_TMA400 */
#ifdef CY_USE_TMA884
		dev_err(ts->dev,
			"%s: INVALID debug setting=%d\n", __func__, (int)value);
		break;
#endif /* --CY_USE_TMA884 */
	case CY_DBG_READ_IC_TCH_CRC:
#ifdef CY_USE_TMA400
#ifdef CY_USE_REG_ACCESS
	{
		/* to use this command first use the set rw_regid
		 * to the row of interest
		 */
		u8 *pdata = NULL;
		size_t location = 0;
		size_t ofs = 0;
		size_t row = 0;

		dev_info(ts->dev,
			"%s: READ TOUCH BLOCK CRC (ts=%p)\n",
			__func__, ts);
		mutex_lock(&ts->data_lock);
		_cyttsp4_change_state(ts, CY_READY_STATE);
		retval = _cyttsp4_set_mode(ts, CY_CONFIG_MODE);
		if (retval < 0) {
			dev_err(ts->dev,
			"%s: Fail switch config mode r=%d\n",
				__func__, retval);
			goto CY_DBG_READ_IC_TCH_CRC_exit;
		}
		pdata = kzalloc(ts->ebid_row_size, GFP_KERNEL);
		if (pdata == NULL) {
			dev_err(ts->dev,
			"%s: Fail allocate block buffer\n", __func__);
			retval = -ENOMEM;
			goto CY_DBG_READ_IC_TCH_CRC_exit;
		}

		retval = _cyttsp4_get_ebid_data_tma400(ts, CY_TCH_PARM_EBID,
			row, pdata);
		if (retval < 0) {
			dev_err(ts->dev,
			"%s: Fail get touch ebid data at row=%d r=%d\n",
				__func__, row, retval);
		}
		dev_vdbg(ts->dev,
			"%s: tch ebid row=%d data:\n", __func__, 0);
		_cyttsp4_pr_buf(ts, pdata, ts->ebid_row_size, "ebid_data");
		location = (pdata[3] * 256) + pdata[2];
		row = location / ts->ebid_row_size;
		ofs = location % ts->ebid_row_size;
		memset(pdata, 0, ts->ebid_row_size);
		dev_vdbg(ts->dev,
			"%s: tch ebid crc_loc=%08X row=%d ofs=%d:\n",
			__func__, location, row, ofs);
		retval = _cyttsp4_get_ebid_data_tma400(ts, CY_TCH_PARM_EBID,
			row, pdata);
		if (retval < 0) {
			dev_err(ts->dev,
			"%s: Fail get touch ebid data at row=%d r=%d\n",
				__func__, row, retval);
		}
		dev_vdbg(ts->dev,
			"%s: tch ebid row=%d data:\n", __func__, row);
		_cyttsp4_pr_buf(ts, pdata, ts->ebid_row_size, "ebid_data");
		_cyttsp4_pr_buf(ts, &pdata[ofs], 4, "crc_data");
		dev_vdbg(ts->dev,
			"%s: tch ebid crc=%02X %02X\n",
			__func__, pdata[ofs], pdata[ofs+1]);

CY_DBG_READ_IC_TCH_CRC_exit:
		if (pdata != NULL)
			kfree(pdata);
		retval = _cyttsp4_set_mode(ts, CY_OPERATE_MODE);
		if (retval < 0) {
			dev_err(ts->dev,
			"%s: Fail switch operate mode r=%d\n",
				__func__, retval);
		}
		mutex_unlock(&ts->data_lock);
		break;
	}
#endif /* --CY_USE_REG_ACCESS */
#endif /* --CY_USE_TMA400 */
#ifdef CY_USE_TMA884
		dev_err(ts->dev,
			"%s: INVALID debug setting=%d\n", __func__, (int)value);
		break;
#endif /* --CY_USE_TMA884 */
	case CY_DBG_CALC_IC_TCH_CRC:
#ifdef CY_USE_TMA400
	{
		u8 ic_crc[2];

		memset(ic_crc, 0, sizeof(ic_crc));
		dev_info(ts->dev,
			"%s: CALC IC TOUCH CRC (ts=%p)\n",
			__func__, ts);
		mutex_lock(&ts->data_lock);
		retval = _cyttsp4_set_mode(ts, CY_CONFIG_MODE);
		if (retval < 0) {
			dev_err(ts->dev,
			"%s: Fail switch to config mode r=%d\n",
				__func__, retval);
		} else {
			retval = _cyttsp4_calc_ic_crc_tma400(ts,
				CY_TCH_PARM_EBID, &ic_crc[0], &ic_crc[1], true);
			if (retval < 0) {
				dev_err(ts->dev,
			"%s: Fail read ic crc r=%d\n",
					__func__, retval);
			}
			_cyttsp4_pr_buf(ts, ic_crc, sizeof(ic_crc),
				"calc_ic_crc_tma400");
			retval = _cyttsp4_set_mode(ts, CY_OPERATE_MODE);
			if (retval < 0) {
				dev_err(ts->dev,
			"%s: Fail switch to operational mode"
					" r=%d\n", __func__, retval);
			}
		}
		mutex_unlock(&ts->data_lock);
		break;
	}
#endif /* --CY_USE_TMA400 */
#ifdef CY_USE_TMA884
		dev_err(ts->dev,
			"%s: INVALID debug setting=%d\n", __func__, (int)value);
		break;
#endif /* --CY_USE_TMA884 */
	case CY_DBG_READ_TABLE_TCH_CRC:
#ifdef CY_USE_TMA400
	{
		u8 *ptable = NULL;
		u8 ic_crc[2];

		memset(ic_crc, 0, sizeof(ic_crc));
		dev_info(ts->dev,
			"%s: GET TABLE TOUCH CRC (ts=%p)\n",
			__func__, ts);
		mutex_lock(&ts->data_lock);
		if (ts->platform_data->sett
			[CY_IC_GRPNUM_TCH_PARM_VAL] == NULL)
			dev_err(ts->dev,
			"%s: NULL param values table\n",
				__func__);
		else if (ts->platform_data->sett
			[CY_IC_GRPNUM_TCH_PARM_VAL]->data == NULL)
			dev_err(ts->dev,
			"%s: NULL param values table data\n",
				__func__);
		else if (ts->platform_data->sett
			[CY_IC_GRPNUM_TCH_PARM_VAL]->size == 0)
			dev_err(ts->dev,
			"%s: param values table size is 0\n",
				__func__);
		else {
			ptable = (u8 *)ts->platform_data->sett
				[CY_IC_GRPNUM_TCH_PARM_VAL]->data;
			_cyttsp_read_table_crc(ts, ptable,
				&ic_crc[0], &ic_crc[1]);
			_cyttsp4_pr_buf(ts, ic_crc, sizeof(ic_crc),
				"read_table_crc_400");
		}
		mutex_unlock(&ts->data_lock);
		break;
	}
#endif /* --CY_USE_TMA400 */
#ifdef CY_USE_TMA884
		dev_err(ts->dev,
			"%s: INVALID debug setting=%d\n", __func__, (int)value);
		break;
#endif /* --CY_USE_TMA884 */
	case CY_DBG_CALC_TABLE_TCH_CRC:
#ifdef CY_USE_TMA400
	{
		u8 ic_crc[2];
		u8 *pdata = NULL;
		size_t table_size = 0;

		memset(ic_crc, 0, sizeof(ic_crc));
		dev_info(ts->dev,
			"%s: CALC TABLE TOUCH CRC (ts=%p)\n",
			__func__, ts);
		mutex_lock(&ts->data_lock);
		retval = _cyttsp4_set_mode(ts, CY_CONFIG_MODE);
		if (retval < 0) {
			dev_err(ts->dev,
			"%s: Fail switch to config mode r=%d\n",
				__func__, retval);
		} else {
			if (ts->platform_data->sett
				[CY_IC_GRPNUM_TCH_PARM_VAL] == NULL)
				dev_err(ts->dev,
			"%s: NULL param values table\n",
					__func__);
			else if (ts->platform_data->sett
				[CY_IC_GRPNUM_TCH_PARM_VAL]->data == NULL)
				dev_err(ts->dev,
			"%s: NULL param values table data\n",
					__func__);
			else if (ts->platform_data->sett
				[CY_IC_GRPNUM_TCH_PARM_VAL]->size == 0)
				dev_err(ts->dev,
			"%s: param values table size is 0\n",
					__func__);
			else {
				pdata = (u8 *)ts->platform_data->sett
					[CY_IC_GRPNUM_TCH_PARM_VAL]->data;
				table_size = ts->platform_data->sett
					[CY_IC_GRPNUM_TCH_PARM_VAL]->size;
				table_size -= 2;
				dev_vdbg(ts->dev,
					"%s: calc table size=%d\n",
					__func__, table_size);
				_cyttsp4_calc_crc(ts, pdata, table_size,
					&ic_crc[0], &ic_crc[1]);
				_cyttsp4_pr_buf(ts, ic_crc, sizeof(ic_crc),
					"calc_table_crc_400");
				dev_vdbg(ts->dev,
					"%s: calc table size=%d\n",
					__func__, table_size);
				retval = _cyttsp4_set_mode(ts, CY_OPERATE_MODE);
				if (retval < 0) {
					dev_err(ts->dev,
			"%s: Fail switch to operational"
						" mode r=%d\n",
						__func__, retval);
				}
			}
		}
		mutex_unlock(&ts->data_lock);
		break;
	}
#endif /* --CY_USE_TMA400 */
#ifdef CY_USE_TMA884
		dev_err(ts->dev,
			"%s: INVALID debug setting=%d\n", __func__, (int)value);
		break;
#endif /* --CY_USE_TMA884 */
#ifdef CY_USE_TMA400
	case CY_DBG_PUT_IC_TCH_ROW:
	{
		enum cyttsp4_ic_ebid ebid = CY_TCH_PARM_EBID;
		size_t row_id = 0;
		size_t num_rows = 0;
		size_t table_size = 0;
		size_t residue = 0;
		size_t ndata = 0;
		int i = 0;
		bool match = false;
		u8 *pdata = NULL;
		u8 *ptable = NULL;

		mutex_lock(&ts->data_lock);
		retval = _cyttsp4_set_mode(ts, CY_CONFIG_MODE);
		if (retval < 0) {
			dev_err(ts->dev,
			"%s: Fail switch to config mode r=%d\n",
				__func__, retval);
			goto CY_DBG_PUT_IC_TCH_ROW_exit;
		}
		pdata = kzalloc(ts->ebid_row_size, GFP_KERNEL);
		if (pdata == NULL) {
			dev_err(ts->dev,
			"%s: Alloc error ebid=%d\n",
				__func__, ebid);
			retval = -ENOMEM;
		} else if (ts->platform_data->sett[CY_IC_GRPNUM_TCH_PARM_VAL]
			== NULL)
			dev_err(ts->dev,
			"%s: NULL param values table\n", __func__);
		else if (ts->platform_data->sett[CY_IC_GRPNUM_TCH_PARM_VAL]
			->data == NULL)
			dev_err(ts->dev,
			"%s: NULL param values table data\n", __func__);
		else if (ts->platform_data->sett[CY_IC_GRPNUM_TCH_PARM_VAL]
			->size == 0)
			dev_err(ts->dev,
			"%s: param values table size is 0\n", __func__);
		else {
			ptable = (u8 *)ts->platform_data->sett
				[CY_IC_GRPNUM_TCH_PARM_VAL]->data;
			table_size = ts->platform_data->sett
				[CY_IC_GRPNUM_TCH_PARM_VAL]->size;
			row_id = ts->rw_regid;
			num_rows = table_size / ts->ebid_row_size;
				residue = table_size % ts->ebid_row_size;
			if (residue)
				num_rows++;
			dev_vdbg(ts->dev,
				"%s: num_rows=%d row_size=%d"
				" table_size=%d residue=%d\n", __func__,
				num_rows, ts->ebid_row_size, table_size,
				residue);
			if (row_id < num_rows) {
				ptable += row_id * ts->ebid_row_size;
				if (row_id < num_rows - 1)
					ndata = ts->ebid_row_size;
				else
					ndata = residue;
				memcpy(pdata, ptable, ndata);
				dev_vdbg(ts->dev,
					"%s: row=%d pdata=%p ndata=%d\n",
					__func__, ts->rw_regid, pdata, ndata);
				_cyttsp4_pr_buf(ts, pdata, ndata, "ebid_data");
				retval = _cyttsp4_put_ebid_data_tma400(ts,
					ebid, row_id, pdata);
				if (retval < 0) {
					dev_err(ts->dev,
			"%s: Fail put row=%d r=%d\n",
						__func__, row_id, retval);
					break;
				}
				/* read back and compare to table */
				dev_vdbg(ts->dev,
					"%s: read back and compare to table\n",
					__func__);
				retval = _cyttsp4_get_ebid_data_tma400(ts,
					ebid, ts->rw_regid, pdata);
				if (retval < 0) {
					dev_err(ts->dev,
			"%s: Fail get row to cmp r=%d\n",
						__func__, retval);
					break;
				}
				_cyttsp4_pr_buf(ts, pdata, ndata, "read_back");
				for (i = 0, match = true; i < ndata && match;
					i++) {
					if (*ptable != *pdata) {
						dev_vdbg(ts->dev,
							"%s: read back err "
							" table[%d]=%02X "
							" pdata[%d]=%02X\n",
							__func__, i, ptable[i],
							i, pdata[i]);
						match = false;
					}
					ptable++;
					pdata++;
				}
				if (match) {
					dev_vdbg(ts->dev,
						"%s: row=%d matches after"
						" put ebid=%d row\n",
						__func__, row_id, ebid);
				} else {
					dev_err(ts->dev,
			"%s: row=%d fails match"
						" after put ebid=%d row\n",
						__func__, row_id, ebid);
				}
			} else {
				dev_err(ts->dev,
			"%s: row_id=ts->rw_regid=%d >"
					" num_rows=%d\n", __func__,
					row_id, num_rows);
			}
		}
		retval = _cyttsp4_set_mode(ts, CY_OPERATE_MODE);
		if (retval < 0) {
			dev_err(ts->dev,
			"%s: Fail switch to operational"
				" mode r=%d\n",
				__func__, retval);
		}
CY_DBG_PUT_IC_TCH_ROW_exit:
		mutex_unlock(&ts->data_lock);
	}
#endif /* --CY_USE_TMA400 */
#ifdef CY_USE_TMA884
		dev_err(ts->dev,
			"%s: INVALID debug setting=%d\n", __func__, (int)value);
		break;
#endif /* --CY_USE_TMA884 */
#endif /* --CY_USE_DEV_DEBUG_TOOLS */
#endif /* --CY_USE_DEBUG_TOOLS */
	default:
		dev_err(ts->dev,
			"%s: INVALID debug setting=%d\n",
			__func__, (int)value);
		break;
	}

	retval = size;
cyttsp4_drv_debug_store_exit:
	return retval;
}
static DEVICE_ATTR(drv_debug, S_IWUSR | S_IRUGO,
	cyttsp4_drv_debug_show, cyttsp4_drv_debug_store);
#endif /* --CONFIG_TOUCHSCREEN_DEBUG */

#ifdef CY_USE_REG_ACCESS
static ssize_t cyttsp_drv_rw_regid_show(struct device *dev,
	struct device_attribute *attr, char *buf)
{
	struct cyttsp4 *ts = dev_get_drvdata(dev);

	return snprintf(buf, CY_MAX_PRBUF_SIZE,
		"Current Read/Write Regid=%02X(%d)\n",
		ts->rw_regid, ts->rw_regid);
}
static ssize_t cyttsp_drv_rw_regid_store(struct device *dev,
	struct device_attribute *attr, const char *buf, size_t size)
{
	struct cyttsp4 *ts = dev_get_drvdata(dev);
	int retval = 0;
	unsigned long value;

	mutex_lock(&ts->data_lock);
	retval = strict_strtoul(buf, 10, &value);
	if (retval < 0) {
		retval = strict_strtoul(buf, 16, &value);
		if (retval < 0) {
			dev_err(ts->dev,
			"%s: Failed to convert value\n",
				__func__);
			goto cyttsp_drv_rw_regid_store_exit;
		}
	}

	if (value > CY_RW_REGID_MAX) {
		ts->rw_regid = CY_RW_REGID_MAX;
		dev_err(ts->dev,
			"%s: Invalid Read/Write Regid; set to max=%d\n",
			__func__, ts->rw_regid);
	} else
		ts->rw_regid = value;

	retval = size;

cyttsp_drv_rw_regid_store_exit:
	mutex_unlock(&ts->data_lock);
	return retval;
}
static DEVICE_ATTR(drv_rw_regid, S_IWUSR | S_IRUGO,
	cyttsp_drv_rw_regid_show, cyttsp_drv_rw_regid_store);


static ssize_t cyttsp_drv_rw_reg_data_show(struct device *dev,
	struct device_attribute *attr, char *buf)
{
	struct cyttsp4 *ts = dev_get_drvdata(dev);
	int retval;
	u8 reg_data;

	retval = _cyttsp4_read_block_data(ts, ts->rw_regid,
			sizeof(reg_data), &reg_data,
			ts->platform_data->addr[CY_TCH_ADDR_OFS], true);

	if (retval < 0)
		return snprintf(buf, CY_MAX_PRBUF_SIZE,
			"Read/Write Regid(%02X(%d) Failed\n",
			ts->rw_regid, ts->rw_regid);
	else
		return snprintf(buf, CY_MAX_PRBUF_SIZE,
			"Read/Write Regid=%02X(%d) Data=%02X(%d)\n",
			ts->rw_regid, ts->rw_regid, reg_data, reg_data);
}
static ssize_t cyttsp_drv_rw_reg_data_store(struct device *dev,
	struct device_attribute *attr, const char *buf, size_t size)
{
	struct cyttsp4 *ts = dev_get_drvdata(dev);
	int retval = 0;
	unsigned long value;
	u8 reg_data = 0;

	retval = strict_strtoul(buf, 10, &value);
	if (retval < 0) {
		retval = strict_strtoul(buf, 16, &value);
		if (retval < 0) {
			dev_err(ts->dev,
			"%s: Failed to convert value\n",
				__func__);
			goto cyttsp_drv_rw_reg_data_store_exit;
		}
	}

	if (value > CY_RW_REG_DATA_MAX) {
		dev_err(ts->dev,
			"%s: Invalid Register Data Range; no write\n",
			__func__);
	} else {
		reg_data = (u8)value;
		retval = _cyttsp4_write_block_data(ts, ts->rw_regid,
			sizeof(reg_data), &reg_data,
			ts->platform_data->addr[CY_TCH_ADDR_OFS], true);
		if (retval < 0) {
			dev_err(ts->dev,
			"%s: Failed write to Regid=%02X(%d)\n",
				__func__, ts->rw_regid, ts->rw_regid);
		}
	}

	retval = size;

cyttsp_drv_rw_reg_data_store_exit:
	return retval;
}
static DEVICE_ATTR(drv_rw_reg_data, S_IWUSR | S_IRUGO,
	cyttsp_drv_rw_reg_data_show, cyttsp_drv_rw_reg_data_store);
#endif

#ifdef CONFIG_TOUCHSCREEN_DEBUG
/* Group Number */
static ssize_t cyttsp4_ic_grpnum_show(struct device *dev,
		struct device_attribute *attr, char *buf)
{
	struct cyttsp4 *ts = dev_get_drvdata(dev);

	return snprintf(buf, CY_MAX_PRBUF_SIZE,
		"Current Group: %d\n", ts->ic_grpnum);
}
static ssize_t cyttsp4_ic_grpnum_store(struct device *dev,
		struct device_attribute *attr, const char *buf, size_t size)
{
	struct cyttsp4 *ts = dev_get_drvdata(dev);
	unsigned long value = 0;
	int retval = 0;

	mutex_lock(&(ts->data_lock));
	retval = strict_strtoul(buf, 10, &value);
	if (retval < 0) {
		dev_err(ts->dev,
			"%s: Failed to convert value\n", __func__);
		goto cyttsp4_ic_grpnum_store_error_exit;
	}

	if (value > 0xFF) {
		value = 0xFF;
		dev_err(ts->dev,
			"%s: value is greater than max;"
			" set to %d\n", __func__, (int)value);
	}
	ts->ic_grpnum = value;

	dev_vdbg(ts->dev,
		"%s: grpnum=%d\n", __func__, ts->ic_grpnum);

cyttsp4_ic_grpnum_store_error_exit:
	retval = size;
	mutex_unlock(&(ts->data_lock));
	return retval;
}
static DEVICE_ATTR(ic_grpnum, S_IRUSR | S_IWUSR,
	cyttsp4_ic_grpnum_show, cyttsp4_ic_grpnum_store);

/* Group Offset */
static ssize_t cyttsp4_ic_grpoffset_show(struct device *dev,
		struct device_attribute *attr, char *buf)
{
	struct cyttsp4 *ts = dev_get_drvdata(dev);

	return snprintf(buf, CY_MAX_PRBUF_SIZE,
		"Current Offset: %u\n", ts->ic_grpoffset);
}
static ssize_t cyttsp4_ic_grpoffset_store(struct device *dev,
	struct device_attribute *attr, const char *buf, size_t size)
{
	struct cyttsp4 *ts = dev_get_drvdata(dev);
	unsigned long value;
	int retval = 0;

	mutex_lock(&(ts->data_lock));
	retval = strict_strtoul(buf, 10, &value);
	if (retval < 0) {
		dev_err(ts->dev,
			"%s: Failed to convert value\n", __func__);
		goto cyttsp4_ic_grpoffset_store_error_exit;
	}

#ifdef CY_USE_TMA400
	if (value > 0xFFFF) {
		value = 0xFFFF;
		dev_err(ts->dev,
			"%s: value is greater than max;"
			" set to %d\n", __func__, (int)value);
	}
#endif /* --CY_USE_TMA400 */
#ifdef CY_USE_TMA884
	if (value > 0xFF) {
		value = 0xFF;
		dev_err(ts->dev,
			"%s: value is greater than max;"
			" set to %d\n", __func__, (int)value);
	}
#endif /* --CY_USE_TMA884 */
	ts->ic_grpoffset = value;

	dev_vdbg(ts->dev,
		"%s: grpoffset=%d\n", __func__, ts->ic_grpoffset);

cyttsp4_ic_grpoffset_store_error_exit:
	retval = size;
	mutex_unlock(&(ts->data_lock));
	return retval;
}
static DEVICE_ATTR(ic_grpoffset, S_IRUSR | S_IWUSR,
	cyttsp4_ic_grpoffset_show, cyttsp4_ic_grpoffset_store);

/* Group Data */
#ifdef CY_USE_TMA400
static int _cyttsp4_show_tch_param_tma400(struct cyttsp4 *ts,
	u8 *ic_buf, size_t *num_data)
{
	/*
	 * get data from ts->ic_grpoffset to
	 * end of block containing ts->ic_grpoffset
	 */
	enum cyttsp4_ic_ebid ebid = CY_TCH_PARM_EBID;
	int start_addr;
	int row_id;
	u8 *pdata;
	int retval = 0;

	retval = _cyttsp4_set_mode(ts, CY_CONFIG_MODE);
	if (retval < 0) {
		dev_err(ts->dev,
			"%s: Fail switch to config mode r=%d\n",
			__func__, retval);
		goto _cyttsp4_show_tch_param_tma400_err;
	}

	dev_vdbg(ts->dev,
		"%s: read block_size=%d pdata=%p\r",
		__func__, ts->ebid_row_size, pdata);

	pdata = kzalloc(ts->ebid_row_size, GFP_KERNEL);
	if (pdata == NULL) {
		dev_err(ts->dev,
			"%s: Fail allocate block buffer\n", __func__);
		retval = -ENOMEM;
		goto _cyttsp4_show_tch_param_tma400_exit;
	}

	start_addr = ts->ic_grpoffset;
	row_id = start_addr / ts->ebid_row_size;
	start_addr %= ts->ebid_row_size;

	dev_vdbg(ts->dev,
		"%s: read row=%d size=%d pdata=%p\r",
		__func__, row_id, ts->ebid_row_size, pdata);

	retval = _cyttsp4_get_ebid_data_tma400(ts, ebid, row_id, pdata);
	if (retval < 0) {
		dev_err(ts->dev,
			"%s: Fail get row=%d r=%d\n",
			__func__, row_id, retval);
		goto _cyttsp4_show_tch_param_tma400_exit;
	}

	*num_data = ts->ebid_row_size - start_addr;
	memcpy(&ic_buf[0], &pdata[start_addr], *num_data);

_cyttsp4_show_tch_param_tma400_exit:
	retval = _cyttsp4_set_mode(ts, CY_OPERATE_MODE);
	if (retval < 0) {
		dev_err(ts->dev,
			"%s: Fail switch to operational mode r=%d\n",
			__func__, retval);
	}
	if (pdata != NULL)
		kfree(pdata);
_cyttsp4_show_tch_param_tma400_err:
	return retval;
}
#endif /* --CY_USE_TMA400 */

static ssize_t _cyttsp4_ic_grpdata_show(struct device *dev,
		struct device_attribute *attr, char *buf)
{
	struct cyttsp4 *ts = dev_get_drvdata(dev);
	int i;
	int retval = 0;
	size_t num_read = 0;
	u8 *ic_buf;
	u8 *pdata;
#ifdef CY_USE_TMA884
	size_t ndata = 0;
	u8 blockid = 0;
#endif /* --CY_USE_TMA884 */

	ic_buf = kzalloc(CY_MAX_PRBUF_SIZE, GFP_KERNEL);
	if (ic_buf == NULL) {
		dev_err(ts->dev,
			"%s: Failed to allocate buffer for %s\n",
			__func__, "ic_grpdata_show");
		return snprintf(buf, CY_MAX_PRBUF_SIZE,
			"Group %d buffer allocation error.\n",
			ts->ic_grpnum);
	}
	dev_vdbg(ts->dev,
		"%s: grpnum=%d grpoffset=%u\n",
		__func__, ts->ic_grpnum, ts->ic_grpoffset);

	if (ts->ic_grpnum >= CY_IC_GRPNUM_NUM) {
		dev_err(ts->dev,
			"%s: Group %d does not exist.\n",
			__func__, ts->ic_grpnum);
		kfree(ic_buf);
		return snprintf(buf, CY_MAX_PRBUF_SIZE,
			"Group %d does not exist.\n",
			ts->ic_grpnum);
	}

	switch (ts->ic_grpnum) {
	case CY_IC_GRPNUM_RESERVED:
		goto cyttsp4_ic_grpdata_show_grperr;
		break;
	case CY_IC_GRPNUM_CMD_REGS:
		num_read = ts->si_ofs.rep_ofs - ts->si_ofs.cmd_ofs;
		dev_vdbg(ts->dev,
			"%s: GRP=CMD_REGS: num_read=%d at ofs=%d + grpofs=%d\n",
			__func__, num_read,
			ts->si_ofs.cmd_ofs, ts->ic_grpoffset);
		if (ts->ic_grpoffset >= num_read)
			goto cyttsp4_ic_grpdata_show_ofserr;
		else {
			num_read -= ts->ic_grpoffset;
			retval = _cyttsp4_read_block_data(ts, ts->ic_grpoffset +
				ts->si_ofs.cmd_ofs, num_read, ic_buf,
				ts->platform_data->addr[CY_TCH_ADDR_OFS], true);
			if (retval < 0)
				goto cyttsp4_ic_grpdata_show_prerr;
		}
		break;
	case CY_IC_GRPNUM_TCH_REP:
		num_read = ts->si_ofs.rep_sz;
		dev_vdbg(ts->dev,
			"%s: GRP=TCH_REP: num_read=%d at ofs=%d + grpofs=%d\n",
			__func__, num_read,
			ts->si_ofs.rep_ofs, ts->ic_grpoffset);
		if (ts->ic_grpoffset >= num_read)
			goto cyttsp4_ic_grpdata_show_ofserr;
		else {
			num_read -= ts->ic_grpoffset;
			retval = _cyttsp4_read_block_data(ts, ts->ic_grpoffset +
				ts->si_ofs.rep_ofs, num_read, ic_buf,
				ts->platform_data->addr[CY_TCH_ADDR_OFS], true);
			if (retval < 0)
				goto cyttsp4_ic_grpdata_show_prerr;
		}
		break;
	case CY_IC_GRPNUM_DATA_REC:
		num_read = ts->si_ofs.cydata_size;
		dev_vdbg(ts->dev,
			"%s: GRP=DATA_REC: num_read=%d at ofs=%d + grpofs=%d\n",
			__func__, num_read,
			ts->si_ofs.cydata_ofs, ts->ic_grpoffset);
		if (ts->ic_grpoffset >= num_read)
			goto cyttsp4_ic_grpdata_show_ofserr;
		else {
			num_read -= ts->ic_grpoffset;
			retval = _cyttsp4_set_mode(ts, CY_SYSINFO_MODE);
			if (retval < 0) {
				dev_err(ts->dev,
			"%s: Fail enter Sysinfo mode r=%d\n",
					__func__, retval);
				goto cyttsp4_ic_grpdata_show_data_rderr;
			}
			retval = _cyttsp4_read_block_data(ts, ts->ic_grpoffset +
				ts->si_ofs.cydata_ofs, num_read, ic_buf,
				ts->platform_data->addr[CY_TCH_ADDR_OFS], true);
			if (retval < 0) {
				dev_err(ts->dev,
			"%s: Fail read Sysinfo ddata r=%d\n",
					__func__, retval);
				goto cyttsp4_ic_grpdata_show_data_rderr;
			}
			retval = _cyttsp4_set_mode(ts, CY_OPERATE_MODE);
			if (retval < 0) {
				dev_err(ts->dev,
			"%s: Fail enter Operational mode r=%d\n",
					__func__, retval);
			}
		}
		break;
cyttsp4_ic_grpdata_show_data_rderr:
		dev_err(ts->dev,
			"%s: Fail read cydata record\n", __func__);
		goto cyttsp4_ic_grpdata_show_prerr;
		break;
	case CY_IC_GRPNUM_TEST_REC:
		num_read =  ts->si_ofs.test_size;
		dev_vdbg(ts->dev,
			"%s: GRP=TEST_REC: num_read=%d at ofs=%d + grpofs=%d\n",
			__func__, num_read,
			ts->si_ofs.test_ofs, ts->ic_grpoffset);
		if (ts->ic_grpoffset >= num_read)
			goto cyttsp4_ic_grpdata_show_ofserr;
		else {
			num_read -= ts->ic_grpoffset;
			retval = _cyttsp4_set_mode(ts, CY_SYSINFO_MODE);
			if (retval < 0) {
				dev_err(ts->dev,
			"%s: Fail enter Sysinfo mode r=%d\n",
					__func__, retval);
				goto cyttsp4_ic_grpdata_show_test_rderr;
			}
			retval = _cyttsp4_read_block_data(ts, ts->ic_grpoffset +
				ts->si_ofs.test_ofs, num_read, ic_buf,
				ts->platform_data->addr[CY_TCH_ADDR_OFS], true);
			if (retval < 0) {
				dev_err(ts->dev,
			"%s: Fail read Sysinfo ddata r=%d\n",
					__func__, retval);
				goto cyttsp4_ic_grpdata_show_test_rderr;
			}
			retval = _cyttsp4_set_mode(ts, CY_OPERATE_MODE);
			if (retval < 0) {
				dev_err(ts->dev,
			"%s: Fail enter Operational mode r=%d\n",
					__func__, retval);
			}
		}
		break;
cyttsp4_ic_grpdata_show_test_rderr:
		dev_err(ts->dev,
			"%s: Fail read test record\n", __func__);
		goto cyttsp4_ic_grpdata_show_prerr;
		break;
	case CY_IC_GRPNUM_PCFG_REC:
		num_read = ts->si_ofs.pcfg_size;
		dev_vdbg(ts->dev,
			"%s: GRP=PCFG_REC: num_read=%d at ofs=%d + grpofs=%d\n",
			__func__, num_read,
			ts->si_ofs.pcfg_ofs, ts->ic_grpoffset);
		if (ts->ic_grpoffset >= num_read)
			goto cyttsp4_ic_grpdata_show_ofserr;
		else {
			num_read -= ts->ic_grpoffset;
			retval = _cyttsp4_set_mode(ts, CY_SYSINFO_MODE);
			if (retval < 0) {
				dev_err(ts->dev,
			"%s: Fail enter Sysinfo mode r=%d\n",
					__func__, retval);
				goto cyttsp4_ic_grpdata_show_pcfg_rderr;
			}
			retval = _cyttsp4_read_block_data(ts, ts->ic_grpoffset +
				ts->si_ofs.pcfg_ofs, num_read, ic_buf,
				ts->platform_data->addr[CY_TCH_ADDR_OFS], true);
			if (retval < 0) {
				dev_err(ts->dev,
			"%s: Fail read Sysinfo ddata r=%d\n",
					__func__, retval);
				goto cyttsp4_ic_grpdata_show_pcfg_rderr;
			}
			retval = _cyttsp4_set_mode(ts, CY_OPERATE_MODE);
			if (retval < 0) {
				dev_err(ts->dev,
			"%s: Fail enter Operational mode r=%d\n",
					__func__, retval);
			}
		}
		break;
cyttsp4_ic_grpdata_show_pcfg_rderr:
		dev_err(ts->dev,
			"%s: Fail read pcfg record\n", __func__);
		goto cyttsp4_ic_grpdata_show_prerr;
		break;
	case CY_IC_GRPNUM_OPCFG_REC:
		num_read = ts->si_ofs.opcfg_size;
		dev_vdbg(ts->dev,
			"%s: GRP=OPCFG_REC:"
			" num_read=%d at ofs=%d + grpofs=%d\n",
			__func__, num_read,
			ts->si_ofs.opcfg_ofs, ts->ic_grpoffset);
		if (ts->ic_grpoffset >= num_read)
			goto cyttsp4_ic_grpdata_show_ofserr;
		else {
			num_read -= ts->ic_grpoffset;
			retval = _cyttsp4_set_mode(ts, CY_SYSINFO_MODE);
			if (retval < 0) {
				dev_err(ts->dev,
			"%s: Fail enter Sysinfo mode r=%d\n",
					__func__, retval);
				goto cyttsp4_ic_grpdata_show_opcfg_rderr;
			}
			retval = _cyttsp4_read_block_data(ts, ts->ic_grpoffset +
				ts->si_ofs.opcfg_ofs, num_read, ic_buf,
				ts->platform_data->addr[CY_TCH_ADDR_OFS], true);
			if (retval < 0) {
				dev_err(ts->dev,
			"%s: Fail read Sysinfo ddata r=%d\n",
					__func__, retval);
				goto cyttsp4_ic_grpdata_show_opcfg_rderr;
			}
			retval = _cyttsp4_set_mode(ts, CY_OPERATE_MODE);
			if (retval < 0) {
				dev_err(ts->dev,
			"%s: Fail enter Operational mode r=%d\n",
					__func__, retval);
			}
		}
		break;
cyttsp4_ic_grpdata_show_opcfg_rderr:
		dev_err(ts->dev,
			"%s: Fail read opcfg record\n", __func__);
		goto cyttsp4_ic_grpdata_show_prerr;
		break;
	case CY_IC_GRPNUM_TCH_PARM_VAL:
#ifdef CY_USE_TMA400
		retval = _cyttsp4_show_tch_param_tma400(ts, ic_buf, &num_read);
		if (retval < 0) {
			dev_err(ts->dev,
			"%s: Fail show Touch Parameters"
				" for TMA400 r=%d\n", __func__, retval);
			goto cyttsp4_ic_grpdata_show_tch_rderr;
		}
		break;
#endif /* --CY_USE_TMA400 */
#ifdef CY_USE_TMA884
		ndata = CY_NUM_CONFIG_BYTES;
		/* do not show cmd, block size and end of block bytes */
		num_read = ndata - (6+4+6);
		dev_vdbg(ts->dev,
			"%s: GRP=PARM_VAL: num_read=%d at ofs=%d + grpofs=%d\n",
			__func__, num_read,
			0, ts->ic_grpoffset);
		if (ts->ic_grpoffset >= num_read)
			goto cyttsp4_ic_grpdata_show_ofserr;
		else {
			blockid = CY_TCH_PARM_EBID;
			pdata = kzalloc(ndata, GFP_KERNEL);
			if (pdata == NULL) {
				dev_err(ts->dev,
			"%s: Failed to allocate read buffer"
					" for %s\n",
					__func__, "platform_touch_param_data");
				retval = -ENOMEM;
				goto cyttsp4_ic_grpdata_show_tch_rderr;
			}
			dev_vdbg(ts->dev,
				"%s: read config block=0x%02X\n",
				__func__, blockid);
			retval = _cyttsp4_set_mode(ts, CY_CONFIG_MODE);
			if (retval < 0) {
				dev_err(ts->dev,
			"%s: Failed to switch to config mode\n",
					__func__);
				goto cyttsp4_ic_grpdata_show_tch_rderr;
			}
			retval = _cyttsp4_read_config_block(ts,
				blockid, pdata, ndata,
				"platform_touch_param_data");
			if (retval < 0) {
				dev_err(ts->dev,
			"%s: Failed read config block %s r=%d\n",
					__func__, "platform_touch_param_data",
					retval);
				goto cyttsp4_ic_grpdata_show_tch_rderr;
			}
			retval = _cyttsp4_set_mode(ts, CY_OPERATE_MODE);
			if (retval < 0) {
				_cyttsp4_change_state(ts, CY_IDLE_STATE);
				dev_err(ts->dev,
			"%s: Fail set operational mode (r=%d)\n",
					__func__, retval);
				goto cyttsp4_ic_grpdata_show_tch_rderr;
			}
			dev_vdbg(ts->dev,
				"%s: memcpy config block=0x%02X\n",
				__func__, blockid);
			num_read -= ts->ic_grpoffset;
			/*
			 * cmd+rdy_bit, status, ebid, lenh, lenl, reserved,
			 * data[0] .. data[ndata-6]
			 * skip data[0] .. data[3] - block size bytes
			 */
			memcpy(ic_buf,
				&pdata[6+4] + ts->ic_grpoffset, num_read);
			kfree(pdata);
		}
		break;
#endif /* --CY_USE_TMA884 */
cyttsp4_ic_grpdata_show_tch_rderr:
		if (pdata != NULL)
			kfree(pdata);
		goto cyttsp4_ic_grpdata_show_prerr;
	case CY_IC_GRPNUM_TCH_PARM_SIZ:
		if (ts->platform_data->sett
			[CY_IC_GRPNUM_TCH_PARM_SIZ] == NULL) {
			dev_err(ts->dev,
			"%s: Missing platform data"
				" Touch Parameters Sizes table\n", __func__);
			goto cyttsp4_ic_grpdata_show_prerr;
		}
		if (ts->platform_data->sett
			[CY_IC_GRPNUM_TCH_PARM_SIZ]->data == NULL) {
			dev_err(ts->dev,
			"%s: Missing platform data"
				" Touch Parameters Sizes table data\n",
				__func__);
			goto cyttsp4_ic_grpdata_show_prerr;
		}
		num_read = ts->platform_data->sett
			[CY_IC_GRPNUM_TCH_PARM_SIZ]->size;
		dev_vdbg(ts->dev,
			"%s: GRP=PARM_SIZ:"
			" num_read=%d at ofs=%d + grpofs=%d\n",
			__func__, num_read,
			0, ts->ic_grpoffset);
		if (ts->ic_grpoffset >= num_read)
			goto cyttsp4_ic_grpdata_show_ofserr;
		else {
			num_read -= ts->ic_grpoffset;
			memcpy(ic_buf, (u8 *)ts->platform_data->sett
				[CY_IC_GRPNUM_TCH_PARM_SIZ]->data +
				ts->ic_grpoffset, num_read);
		}
		break;
	case CY_IC_GRPNUM_DDATA_REC:
		num_read = ts->si_ofs.ddata_size;
		dev_vdbg(ts->dev,
			"%s: GRP=DDATA_REC:"
			" num_read=%d at ofs=%d + grpofs=%d\n",
			__func__, num_read,
			ts->si_ofs.ddata_ofs, ts->ic_grpoffset);
		if (ts->ic_grpoffset >= num_read)
			goto cyttsp4_ic_grpdata_show_ofserr;
		else {
			num_read -= ts->ic_grpoffset;
			retval = _cyttsp4_set_mode(ts, CY_SYSINFO_MODE);
			if (retval < 0) {
				dev_err(ts->dev,
			"%s: Fail enter Sysinfo mode r=%d\n",
					__func__, retval);
				goto cyttsp4_ic_grpdata_show_ddata_rderr;
			}
			retval = _cyttsp4_read_block_data(ts, ts->ic_grpoffset +
				ts->si_ofs.ddata_ofs, num_read, ic_buf,
				ts->platform_data->addr[CY_TCH_ADDR_OFS], true);
			if (retval < 0) {
				dev_err(ts->dev,
			"%s: Fail read Sysinfo ddata r=%d\n",
					__func__, retval);
				goto cyttsp4_ic_grpdata_show_ddata_rderr;
			}
			retval = _cyttsp4_set_mode(ts, CY_OPERATE_MODE);
			if (retval < 0) {
				dev_err(ts->dev,
			"%s: Fail enter Operational mode r=%d\n",
					__func__, retval);
			}
		}
		break;
cyttsp4_ic_grpdata_show_ddata_rderr:
		dev_err(ts->dev,
			"%s: Fail read ddata\n", __func__);
		goto cyttsp4_ic_grpdata_show_prerr;
		break;
	case CY_IC_GRPNUM_MDATA_REC:
		num_read = ts->si_ofs.mdata_size;
		dev_vdbg(ts->dev,
			"%s: GRP=MDATA_REC:"
			" num_read=%d at ofs=%d + grpofs=%d\n",
			__func__, num_read,
			ts->si_ofs.mdata_ofs, ts->ic_grpoffset);
		if (ts->ic_grpoffset >= num_read)
			goto cyttsp4_ic_grpdata_show_ofserr;
		else {
			num_read -= ts->ic_grpoffset;
			retval = _cyttsp4_set_mode(ts, CY_SYSINFO_MODE);
			if (retval < 0) {
				dev_err(ts->dev,
			"%s: Fail enter Sysinfo mode r=%d\n",
					__func__, retval);
				goto cyttsp4_ic_grpdata_show_mdata_rderr;
			}
			retval = _cyttsp4_read_block_data(ts, ts->ic_grpoffset +
				ts->si_ofs.mdata_ofs, num_read, ic_buf,
				ts->platform_data->addr[CY_TCH_ADDR_OFS], true);
			if (retval < 0) {
				dev_err(ts->dev,
			"%s: Fail read Sysinfo regs r=%d\n",
					__func__, retval);
				goto cyttsp4_ic_grpdata_show_mdata_rderr;
			}
			retval = _cyttsp4_set_mode(ts, CY_OPERATE_MODE);
			if (retval < 0) {
				dev_err(ts->dev,
			"%s: Fail enter Operational mode r=%d\n",
					__func__, retval);
			}
		}
		break;
cyttsp4_ic_grpdata_show_mdata_rderr:
		dev_err(ts->dev,
			"%s: Fail read mdata\n", __func__);
		goto cyttsp4_ic_grpdata_show_prerr;
		break;
	default:
		goto cyttsp4_ic_grpdata_show_grperr;
		break;
	}

	snprintf(buf, CY_MAX_PRBUF_SIZE,
		"Group %d, Offset %u:\n", ts->ic_grpnum, ts->ic_grpoffset);
	for (i = 0; i < num_read; i++) {
		snprintf(buf, CY_MAX_PRBUF_SIZE,
			"%s0x%02X\n", buf, ic_buf[i]);
	}
	kfree(ic_buf);
	return snprintf(buf, CY_MAX_PRBUF_SIZE,
		"%s(%d bytes)\n", buf, num_read);

cyttsp4_ic_grpdata_show_ofserr:
	dev_err(ts->dev,
			"%s: Group Offset=%d exceeds Group Read Length=%d\n",
		__func__, ts->ic_grpoffset, num_read);
	kfree(ic_buf);
	snprintf(buf, CY_MAX_PRBUF_SIZE,
		"Cannot read Group %d Data.\n",
		ts->ic_grpnum);
	return snprintf(buf, CY_MAX_PRBUF_SIZE,
		"%sGroup Offset=%d exceeds Group Read Length=%d\n",
		buf, ts->ic_grpoffset, num_read);
cyttsp4_ic_grpdata_show_prerr:
	dev_err(ts->dev,
			"%s: Cannot read Group %d Data.\n",
		__func__, ts->ic_grpnum);
	kfree(ic_buf);
	return snprintf(buf, CY_MAX_PRBUF_SIZE,
		"Cannot read Group %d Data.\n",
		ts->ic_grpnum);
cyttsp4_ic_grpdata_show_grperr:
	dev_err(ts->dev,
			"%s: Group %d does not exist.\n",
		__func__, ts->ic_grpnum);
	kfree(ic_buf);
	return snprintf(buf, CY_MAX_PRBUF_SIZE,
		"Group %d does not exist.\n",
		ts->ic_grpnum);
}
static ssize_t cyttsp4_ic_grpdata_show(struct device *dev,
		struct device_attribute *attr, char *buf)
{
	struct cyttsp4 *ts = dev_get_drvdata(dev);
	ssize_t retval = 0;

	mutex_lock(&ts->data_lock);
	if (ts->driver_state == CY_SLEEP_STATE) {
		dev_err(ts->dev,
			"%s: Group Show Test blocked: IC suspended\n",
			__func__);
		retval = snprintf(buf, CY_MAX_PRBUF_SIZE,
			"Group %d Show Test blocked: IC suspended\n",
			ts->ic_grpnum);
	} else
		retval = _cyttsp4_ic_grpdata_show(dev, attr, buf);
	mutex_unlock(&ts->data_lock);

	return retval;
}

#ifdef CY_USE_TMA400
static int _cyttsp4_store_tch_param_tma400(struct cyttsp4 *ts,
	u8 *ic_buf, size_t length)
{
	int retval = 0;
	int next_data = 0;
	int num_data = 0;
	int start_addr = 0;
	int end_addr = 0;
	int start_row = 0;
	int end_row = 0;
	int row_id = 0;
	int row_ofs = 0;
	int num_rows = 0;
	int crc_loc = 0;
	enum cyttsp4_ic_ebid ebid = CY_TCH_PARM_EBID;
	u8 calc_ic_crc[2];
	u8 *pdata = NULL;

	memset(calc_ic_crc, 0, sizeof(calc_ic_crc));
	retval = _cyttsp4_set_mode(ts, CY_CONFIG_MODE);
	if (retval < 0) {
		dev_err(ts->dev,
			"%s: Fail switch to config mode r=%d\n",
			__func__, retval);
		goto _cyttsp4_store_tch_param_tma400_err;
	}

	start_addr = ts->ic_grpoffset;
	next_data = 0;
	end_addr = start_addr + length;
	start_row = start_addr / ts->ebid_row_size;
	start_addr %= ts->ebid_row_size;
	end_row = end_addr / ts->ebid_row_size;
	end_addr %= ts->ebid_row_size;
	num_rows = end_row - start_row + 1;

	dev_vdbg(ts->dev,
		"%s: start_addr=0x%04X(%d) size=%d start_row=%d end_row=%d"
		" end_addr=%04X(%d) num_rows=%d\n",
		__func__,
		start_addr, start_addr, ts->ebid_row_size, start_row,
		end_row, end_addr, end_addr, num_rows);

	pdata = kzalloc(ts->ebid_row_size, GFP_KERNEL);
	if (pdata == NULL) {
		dev_err(ts->dev,
			"%s: Fail allocate block buffer\n", __func__);
		retval = -ENOMEM;
		goto _cyttsp4_store_tch_param_tma400_exit;
	}

	for (row_id = start_row;
		row_id < start_row + num_rows; row_id++) {
		dev_vdbg(ts->dev,
			"%s: get EBID row=%d\n", __func__, row_id);
		retval = _cyttsp4_get_ebid_data_tma400(ts, ebid, row_id, pdata);
		if (retval < 0) {
			dev_err(ts->dev,
			"%s: Fail get EBID row=%d r=%d\n",
				__func__, row_id, retval);
			goto _cyttsp4_store_tch_param_tma400_exit;
		}
		num_data = ts->ebid_row_size - start_addr;
		if (row_id == end_row)
			num_data -= ts->ebid_row_size - end_addr;
		memcpy(&pdata[start_addr], &ic_buf[next_data], num_data);
		next_data += num_data;
		dev_vdbg(ts->dev,
			"%s: put_row=%d size=%d pdata=%p start_addr=%04X"
			" &pdata[start_addr]=%p num_data=%d\n", __func__,
			row_id, ts->ebid_row_size, pdata, start_addr,
			&pdata[start_addr], num_data);
		_cyttsp4_pr_buf(ts, &pdata[start_addr], num_data, "put_block");
		_cyttsp4_pr_buf(ts, pdata, ts->ebid_row_size, "print_block");
		retval = _cyttsp4_put_ebid_data_tma400(ts,
			ebid, row_id, pdata);
		if (retval < 0) {
			dev_err(ts->dev,
			"%s: Fail put EBID row=%d r=%d\n",
				__func__, row_id, retval);
			goto _cyttsp4_store_tch_param_tma400_exit;
		}

		start_addr = 0;
		ts->ic_grptest = true;
	}

	/* Update CRC bytes to force restore on reboot */
	if (ts->ic_grptest) {
		memset(calc_ic_crc, 0, sizeof(calc_ic_crc));
		dev_vdbg(ts->dev,
			"%s: Calc IC CRC values\n", __func__);
		retval = _cyttsp4_calc_ic_crc_tma400(ts, ebid,
			&calc_ic_crc[1], &calc_ic_crc[0], false);
		if (retval < 0) {
			dev_err(ts->dev,
			"%s: Fail calc ic crc r=%d\n",
				__func__, retval);
		}
		retval = _cyttsp4_get_ebid_data_tma400(ts, ebid, 0, pdata);
		if (retval < 0) {
			dev_err(ts->dev,
			"%s: Fail get EBID row=%d r=%d\n",
				__func__, row_id, retval);
			goto _cyttsp4_store_tch_param_tma400_exit;
		}
		crc_loc = (pdata[3] * 256) + pdata[2];
		row_ofs = crc_loc % ts->ebid_row_size;
		row_id = crc_loc / ts->ebid_row_size;
		dev_vdbg(ts->dev,
		"%s: tch ebid=%d crc_loc=%08X crc_row=%d crc_ofs=%d data:\n",
		__func__, ebid, crc_loc, row_id, row_ofs);
		retval = _cyttsp4_get_ebid_data_tma400(ts, ebid, row_id, pdata);
		if (retval < 0) {
			dev_err(ts->dev,
			"%s: Fail get EBID row=%d r=%d\n",
				__func__, row_id, retval);
			goto _cyttsp4_store_tch_param_tma400_exit;
		}
		memcpy(&pdata[row_ofs], calc_ic_crc, sizeof(calc_ic_crc));
		retval = _cyttsp4_put_ebid_data_tma400(ts,
			ebid, row_id, pdata);
		if (retval < 0) {
			dev_err(ts->dev,
			"%s: Fail put EBID row=%d r=%d\n",
				__func__, row_id, retval);
		}
	}

_cyttsp4_store_tch_param_tma400_exit:
	retval = _cyttsp4_set_mode(ts, CY_OPERATE_MODE);
	if (retval < 0) {
		dev_err(ts->dev,
			"%s: Fail switch to operational mode r=%d\n",
			__func__, retval);
	}
	if (pdata != NULL)
		kfree(pdata);
_cyttsp4_store_tch_param_tma400_err:
	return retval;
}
#endif /* --CY_USE_TMA400 */

#ifdef CY_USE_TMA884
static int _cyttsp4_write_mddata(struct cyttsp4 *ts, size_t write_length,
	size_t mddata_length, u8 blkid, size_t mddata_ofs,
	u8 *ic_buf, const char *mddata_name)
{
	bool mddata_updated = false;
	u8 *pdata;
	int retval = 0;

	pdata = kzalloc(CY_MAX_PRBUF_SIZE, GFP_KERNEL);
	if (pdata == NULL) {
		dev_err(ts->dev,
			"%s: Fail allocate data buffer\n", __func__);
		retval = -ENOMEM;
		goto cyttsp4_write_mddata_exit;
	}
	if (ts->current_mode != CY_MODE_OPERATIONAL) {
		dev_err(ts->dev,
			"%s: Must be in operational mode to start write of"
			" %s (current mode=%d)\n",
			__func__, mddata_name, ts->current_mode);
		retval = -EPERM;
		goto cyttsp4_write_mddata_exit;
	}
	if ((write_length + ts->ic_grpoffset) > mddata_length) {
		dev_err(ts->dev,
			"%s: Requested length(%d) is greater than"
			" %s size(%d)\n", __func__,
			write_length, mddata_name, mddata_length);
		retval = -EINVAL;
		goto cyttsp4_write_mddata_exit;
	}
	retval = _cyttsp4_set_mode(ts, CY_SYSINFO_MODE);
	if (retval < 0) {
		dev_err(ts->dev,
			"%s: Fail to enter Sysinfo mode r=%d\n",
			__func__, retval);
		goto cyttsp4_write_mddata_exit;
	}
	dev_vdbg(ts->dev,
		"%s: blkid=%02X mddata_ofs=%d mddata_length=%d"
		" mddata_name=%s write_length=%d grpofs=%d\n",
		__func__, blkid, mddata_ofs, mddata_length, mddata_name,
		write_length, ts->ic_grpoffset);
	_cyttsp4_read_block_data(ts, mddata_ofs, mddata_length, pdata,
		ts->platform_data->addr[CY_TCH_ADDR_OFS], true);
	if (retval < 0) {
		dev_err(ts->dev,
			"%s: Fail to read %s regs r=%d\n",
			__func__, mddata_name, retval);
		goto cyttsp4_write_mddata_exit;
	}
	memcpy(pdata + ts->ic_grpoffset, ic_buf, write_length);
	_cyttsp4_set_data_block(ts, blkid, pdata,
		mddata_length, mddata_name, true, &mddata_updated);
	if ((retval < 0) || !mddata_updated) {
		dev_err(ts->dev,
			"%s: Fail while writing %s block r=%d updated=%d\n",
			__func__, mddata_name, retval, (int)mddata_updated);
	}
	retval = _cyttsp4_set_mode(ts, CY_OPERATE_MODE);
	if (retval < 0) {
		dev_err(ts->dev,
			"%s: Fail to enter Operational mode r=%d\n",
			__func__, retval);
	}

cyttsp4_write_mddata_exit:
	kfree(pdata);
	return retval;
}
#endif /* --CY_USE_TMA884 */

static ssize_t _cyttsp4_ic_grpdata_store(struct device *dev,
		struct device_attribute *attr, const char *buf, size_t size)
{
	struct cyttsp4 *ts = dev_get_drvdata(dev);
	unsigned long value;
	int retval = 0;
	const char *pbuf = buf;
	int i, j;
	char *scan_buf = NULL;
	u8 *ic_buf = NULL;
	size_t length;
#ifdef CY_USE_TMA884
	u8 *pdata = NULL;
	size_t mddata_length, ndata;
	u8 blockid = 0;
	bool mddata_updated = false;
	const char *mddata_name = "invalid name";
#endif /* --CY_USE_TMA884 */

	scan_buf = kzalloc(CY_MAX_PRBUF_SIZE, GFP_KERNEL);
	if (scan_buf == NULL) {
		dev_err(ts->dev,
			"%s: Failed to allocate scan buffer for"
			" Group Data store\n", __func__);
		goto cyttsp4_ic_grpdata_store_exit;
	}
	ic_buf = kzalloc(CY_MAX_PRBUF_SIZE, GFP_KERNEL);
	if (ic_buf == NULL) {
		dev_err(ts->dev,
			"%s: Failed to allocate ic buffer for"
			" Group Data store\n", __func__);
		goto cyttsp4_ic_grpdata_store_exit;
	}
	dev_vdbg(ts->dev,
		"%s: grpnum=%d grpoffset=%u\n",
		__func__, ts->ic_grpnum, ts->ic_grpoffset);

	if (ts->ic_grpnum >= CY_IC_GRPNUM_NUM) {
		dev_err(ts->dev,
			"%s: Group %d does not exist.\n",
			__func__, ts->ic_grpnum);
		retval = size;
		goto cyttsp4_ic_grpdata_store_exit;
	}
	dev_vdbg(ts->dev,
		"%s: pbuf=%p buf=%p size=%d sizeof(scan_buf)=%d buf=%s\n",
		__func__, pbuf, buf, size, sizeof(scan_buf), buf);

	i = 0;
	while (pbuf <= (buf + size)) {
		while (((*pbuf == ' ') || (*pbuf == ',')) &&
			(pbuf < (buf + size)))
			pbuf++;
		if (pbuf < (buf + size)) {
			memset(scan_buf, 0, CY_MAX_PRBUF_SIZE);
			for (j = 0; j < sizeof("0xHH") &&
				*pbuf != ' ' && *pbuf != ','; j++)
				scan_buf[j] = *pbuf++;
			retval = strict_strtoul(scan_buf, 16, &value);
			if (retval < 0) {
				dev_err(ts->dev,
			"%s: Invalid data format. "
					"Use \"0xHH,...,0xHH\" instead.\n",
					__func__);
				retval = size;
				goto cyttsp4_ic_grpdata_store_exit;
			} else {
				if (i >= ts->max_config_bytes) {
					dev_err(ts->dev,
			"%s: Max command size exceeded"
					" (size=%d max=%d)\n", __func__,
					i, ts->max_config_bytes);
					goto cyttsp4_ic_grpdata_store_exit;
				}
				ic_buf[i] = value;
				dev_vdbg(ts->dev,
					"%s: ic_buf[%d] = 0x%02X\n",
					__func__, i, ic_buf[i]);
				i++;
			}
		} else
			break;
	}
	length = i;

	/* write ic_buf to log */
	_cyttsp4_pr_buf(ts, ic_buf, length, "ic_buf");

	switch (ts->ic_grpnum) {
	case CY_IC_GRPNUM_CMD_REGS:
		if ((length + ts->ic_grpoffset + ts->si_ofs.cmd_ofs) >
			ts->si_ofs.rep_ofs) {
			dev_err(ts->dev,
			"%s: Length(%d) + offset(%d) + cmd_offset(%d)"
				" is beyond cmd reg space[%d..%d]\n", __func__,
				length, ts->ic_grpoffset, ts->si_ofs.cmd_ofs,
				ts->si_ofs.cmd_ofs, ts->si_ofs.rep_ofs - 1);
			goto cyttsp4_ic_grpdata_store_exit;
		}
		retval = _cyttsp4_write_block_data(ts, ts->ic_grpoffset +
			ts->si_ofs.cmd_ofs, length, ic_buf,
			ts->platform_data->addr[CY_TCH_ADDR_OFS], true);
		if (retval < 0) {
			dev_err(ts->dev,
			"%s: Fail write command regs r=%d\n",
				__func__, retval);
		}
		if (!ts->ic_grptest) {
			dev_info(ts->dev,
			"%s: Disabled settings checksum verifications"
				" until next boot.\n", __func__);
			ts->ic_grptest = true;
		}
		break;
	case CY_IC_GRPNUM_TCH_PARM_VAL:
#ifdef CY_USE_TMA400
		retval = _cyttsp4_store_tch_param_tma400(ts, ic_buf, length);
		if (retval < 0) {
			dev_err(ts->dev,
			"%s: Fail store Touch Parameters"
				" for TMA400 r=%d\n", __func__, retval);
		}
		break;
#endif /* --CY_USE_TMA400 */
#ifdef CY_USE_TMA884
		mddata_name = "Touch Parameters";
		ndata = CY_NUM_CONFIG_BYTES;
		blockid = CY_TCH_PARM_EBID;
		/* do not show cmd, block size and end of block bytes */
		mddata_length = ndata - (6+4+6);
		dev_vdbg(ts->dev,
			"%s: GRP=PARM_VAL: write length=%d at ofs=%d +"
			" grpofs=%d\n", __func__, length,
			0, ts->ic_grpoffset);
		if ((length + ts->ic_grpoffset) > mddata_length) {
			dev_err(ts->dev,
			"%s: Requested length(%d) is greater than"
				" %s size(%d)\n", __func__,
				length, mddata_name, mddata_length);
			retval = -EINVAL;
			goto cyttsp4_ic_grpdata_store_tch_wrerr;
		}
		pdata = kzalloc(ndata, GFP_KERNEL);
		if (pdata == NULL) {
			dev_err(ts->dev,
			"%s: Failed to allocate read/write buffer"
				" for %s\n",
				__func__, "platform_touch_param_data");
			retval = -ENOMEM;
			goto cyttsp4_ic_grpdata_store_tch_wrerr;
		}
		dev_vdbg(ts->dev,
			"%s: read config block=0x%02X\n",
			__func__, blockid);
		retval = _cyttsp4_set_mode(ts, CY_CONFIG_MODE);
		if (retval < 0) {
			dev_err(ts->dev,
			"%s: Failed to switch to config mode\n",
				__func__);
			goto cyttsp4_ic_grpdata_store_tch_wrerr;
		}
		retval = _cyttsp4_read_config_block(ts,
			blockid, pdata, ndata,
			"platform_touch_param_data");
		if (retval < 0) {
			dev_err(ts->dev,
			"%s: Failed read config block %s r=%d\n",
				__func__, "platform_touch_param_data",
				retval);
			goto cyttsp4_ic_grpdata_store_tch_wrerr;
		}
		/*
		 * cmd+rdy_bit, status, ebid, lenh, lenl, reserved,
		 * data[0] .. data[ndata-6]
		 * skip data[0] .. data[3] - block size bytes
		 */
		memcpy(&pdata[6+4+ts->ic_grpoffset], ic_buf, length);
		_cyttsp4_set_data_block(ts, blockid, &pdata[6+4],
			mddata_length, mddata_name, true, &mddata_updated);
		if ((retval < 0) || !mddata_updated) {
			dev_err(ts->dev,
			"%s: Fail while writing %s block r=%d"
				" updated=%d\n", __func__,
				mddata_name, retval, (int)mddata_updated);
		}
		if (!ts->ic_grptest) {
			dev_info(ts->dev,
			"%s: Disabled settings checksum verifications"
				" until next boot.\n", __func__);
			ts->ic_grptest = true;
		}
		retval = _cyttsp4_startup(ts);
		if (retval < 0) {
			dev_err(ts->dev,
			"%s: Fail restart after writing params r=%d\n",
				__func__, retval);
		}
cyttsp4_ic_grpdata_store_tch_wrerr:
		kfree(pdata);
		break;
#endif /* --CY_USE_TMA884 */
	case CY_IC_GRPNUM_DDATA_REC:
#ifdef CY_USE_TMA400
		dev_info(ts->dev,
			"%s: store Design Data not supported"
			" for TMA400 (use store Touch Parameters)\n",
			__func__);
		break;
#endif /* --CY_USE_TMA400 */
#ifdef CY_USE_TMA884
		mddata_length = ts->si_ofs.ddata_size;
		dev_vdbg(ts->dev,
			"%s: DDATA_REC length=%d mddata_length=%d blkid=%02X"
			" ddata_ofs=%d name=%s\n", __func__, length,
			mddata_length, CY_DDATA_EBID, ts->si_ofs.ddata_ofs,
			"Design Data");
		_cyttsp4_pr_buf(ts, ic_buf, length, "Design Data");
		retval = _cyttsp4_write_mddata(ts, length, mddata_length,
			CY_DDATA_EBID, ts->si_ofs.ddata_ofs, ic_buf,
			"Design Data");
		if (retval < 0) {
			dev_err(ts->dev,
			"%s: Fail writing Design Data\n",
				__func__);
		} else if (!ts->ic_grptest) {
			dev_info(ts->dev,
			"%s: Disabled settings checksum verifications"
				" until next boot.\n", __func__);
			ts->ic_grptest = true;
		}
		break;
#endif /* --CY_USE_TMA884 */
	case CY_IC_GRPNUM_MDATA_REC:
#ifdef CY_USE_TMA400
		dev_info(ts->dev,
			"%s: store Manufacturing Data not supported"
			" for TMA400\n", __func__);
		break;
#endif /* --CY_USE_TMA400 */
#ifdef CY_USE_TMA884
		mddata_length = ts->si_ofs.mdata_size;
		dev_vdbg(ts->dev,
			"%s: MDATA_REC length=%d mddata_length=%d blkid=%02X"
			" ddata_ofs=%d name=%s\n", __func__, length,
			mddata_length, CY_MDATA_EBID, ts->si_ofs.mdata_ofs,
			"Manufacturing Data");
		_cyttsp4_pr_buf(ts, ic_buf, length, "Manufacturing Data");
		retval = _cyttsp4_write_mddata(ts, length, mddata_length,
			CY_MDATA_EBID, ts->si_ofs.mdata_ofs, ic_buf,
			"Manufacturing Data");
		if (retval < 0) {
			dev_err(ts->dev,
			"%s: Fail writing Manufacturing Data\n",
				__func__);
		} else if (!ts->ic_grptest) {
			dev_info(ts->dev,
			"%s: Disabled settings checksum verifications"
				" until next boot.\n", __func__);
			ts->ic_grptest = true;
		}
		break;
#endif /* --CY_USE_TMA884 */
	default:
		dev_err(ts->dev,
			"%s: Group=%d is read only\n",
			__func__, ts->ic_grpnum);
		break;
	}

cyttsp4_ic_grpdata_store_exit:
	if (scan_buf != NULL)
		kfree(scan_buf);
	if (ic_buf != NULL)
		kfree(ic_buf);
	return size;
}
static ssize_t cyttsp4_ic_grpdata_store(struct device *dev,
		struct device_attribute *attr, const char *buf, size_t size)
{
	struct cyttsp4 *ts = dev_get_drvdata(dev);
	ssize_t retval = 0;

	mutex_lock(&ts->data_lock);
	if (ts->driver_state == CY_SLEEP_STATE) {
		dev_err(ts->dev,
			"%s: Group Store Test blocked: IC suspended\n",
			__func__);
		retval = size;
	} else
		retval = _cyttsp4_ic_grpdata_store(dev, attr, buf, size);
	mutex_unlock(&ts->data_lock);

	return retval;
}
static DEVICE_ATTR(ic_grpdata, S_IRUSR | S_IWUSR,
	cyttsp4_ic_grpdata_show, cyttsp4_ic_grpdata_store);

static ssize_t cyttsp4_drv_flags_show(struct device *dev,
		struct device_attribute *attr, char *buf)
{
	struct cyttsp4 *ts = dev_get_drvdata(dev);

	return snprintf(buf, CY_MAX_PRBUF_SIZE,
		"Current Driver Flags: 0x%04X\n", ts->flags);
}
static ssize_t cyttsp4_drv_flags_store(struct device *dev,
		struct device_attribute *attr, const char *buf, size_t size)
{
	struct cyttsp4 *ts = dev_get_drvdata(dev);
	unsigned long value = 0;
	ssize_t retval = 0;

	mutex_lock(&(ts->data_lock));
	retval = strict_strtoul(buf, 16, &value);
	if (retval < 0) {
		dev_err(ts->dev,
			"%s: Failed to convert value\n", __func__);
		goto cyttsp4_drv_flags_store_error_exit;
	}

	if (value > 0xFFFF) {
		dev_err(ts->dev,
			"%s: value=%lu is greater than max;"
			" drv_flags=0x%04X\n", __func__, value, ts->flags);
	} else {
		ts->flags = value;
	}

	dev_vdbg(ts->dev,
		"%s: drv_flags=0x%04X\n", __func__, ts->flags);

cyttsp4_drv_flags_store_error_exit:
	retval = size;
	mutex_unlock(&(ts->data_lock));
	return retval;
}
static DEVICE_ATTR(drv_flags, S_IRUSR | S_IWUSR,
	cyttsp4_drv_flags_show, cyttsp4_drv_flags_store);

static ssize_t cyttsp4_hw_power_show(struct device *dev,
				struct device_attribute *attr, char *buf)
{
	struct cyttsp4 *ts = dev_get_drvdata(dev);
	int is_on;
	char* stat = "Unknown";

	if (ts->platform_data->hw_power) {
		is_on = ts->platform_data->hw_power(99);
		stat = (is_on) ? "On" : "Off";
	}

	return snprintf(buf, CY_MAX_PRBUF_SIZE, "Power: %s\n", stat);
}
static ssize_t cyttsp4_hw_power_store(struct device *dev,
		struct device_attribute *attr, const char *buf, size_t size)
{
	struct cyttsp4 *ts = dev_get_drvdata(dev);
	unsigned long value = 0;
	ssize_t retval = 0;

	mutex_lock(&(ts->data_lock));
	retval = strict_strtoul(buf, 10, &value);
	if (retval < 0) {
		dev_err(ts->dev, "%s: Failed to convert value\n", __func__);
		goto cyttsp4_hw_power_error_exit;
	}

	if (value < 0 || value > 1) {
		dev_err(ts->dev, "%s: Invalid value %lu\n", __func__, value);
	} else {
		if (ts->platform_data->hw_power)
			retval = ts->platform_data->hw_power((int)value);
	}

	if (retval < 0) {
		dev_err(ts->dev, "%s: fail hw_power r=%d\n", __func__, retval);
	}

cyttsp4_hw_power_error_exit:
	retval = size;
	mutex_unlock(&(ts->data_lock));
	return retval;
}
static DEVICE_ATTR(hw_power, S_IRUSR | S_IWUSR,
		cyttsp4_hw_power_show, cyttsp4_hw_power_store);

static ssize_t cyttsp4_hw_reset_store(struct device *dev,
		struct device_attribute *attr, const char *buf, size_t size)
{
	struct cyttsp4 *ts = dev_get_drvdata(dev);
	ssize_t retval = 0;

	mutex_lock(&(ts->data_lock));
	retval = _cyttsp4_startup(ts);
	mutex_unlock(&(ts->data_lock));
	if (retval < 0) {
		dev_err(ts->dev,
			"%s: fail hw_reset device restart r=%d\n",
			__func__, retval);
	}

	retval = size;
	return retval;
}
static DEVICE_ATTR(hw_reset, S_IWUSR, NULL, cyttsp4_hw_reset_store);

static ssize_t cyttsp4_hw_recov_store(struct device *dev,
		struct device_attribute *attr, const char *buf, size_t size)
{
	struct cyttsp4 *ts = dev_get_drvdata(dev);
	unsigned long value = 0;
	ssize_t retval = 0;

	mutex_lock(&(ts->data_lock));
	retval = strict_strtoul(buf, 10, &value);
	if (retval < 0) {
		dev_err(ts->dev,
			"%s: Failed to convert value\n", __func__);
		goto cyttsp4_hw_recov_store_error_exit;
	}

	if (ts->platform_data->hw_recov == NULL) {
		dev_err(ts->dev,
			"%s: no hw_recov function\n", __func__);
		goto cyttsp4_hw_recov_store_error_exit;
	}

	retval = ts->platform_data->hw_recov((int)value);
	if (retval < 0) {
		dev_err(ts->dev,
			"%s: fail hw_recov(value=%d) function r=%d\n",
			__func__, (int)value, retval);
	}

cyttsp4_hw_recov_store_error_exit:
	retval = size;
	mutex_unlock(&(ts->data_lock));
	return retval;
}
static DEVICE_ATTR(hw_recov, S_IWUSR, NULL, cyttsp4_hw_recov_store);

static ssize_t cyttsp4_ic_charmor_show(struct device *dev,
		struct device_attribute *attr, char *buf)
{
	struct cyttsp4 *ts = dev_get_drvdata(dev);
	int retval = 0;
	bool enabled = false;
	
	retval = cyttsp4_get_charger_armor_status(ts, &enabled);
	if (retval < 0) {
		return snprintf(buf, CY_MAX_PRBUF_SIZE,
				"Charger Armor: Unknown\n");
	}
	
	return snprintf(buf, CY_MAX_PRBUF_SIZE,
			"Charger Armor: %s\n", enabled ? "ON" : "OFF");
}
static ssize_t cyttsp4_ic_charmor_store(struct device *dev,
		struct device_attribute *attr, const char *buf, size_t size)
{
	struct cyttsp4 *ts = dev_get_drvdata(dev);
	unsigned long value = 0;
	int retval = 0;
	bool enabled;

	retval = strict_strtoul(buf, 10, &value);
	if (retval < 0) {
		dev_err(ts->dev,
			"%s: Failed to convert value\n", __func__);
		goto cyttsp4_ic_charmor_store_exit;
	}

	enabled = value ? true : false;
	
	retval = cyttsp4_set_charger_armor(ts, enabled);
	if (retval < 0) {
		dev_err(ts->dev,
			"%s: Failed to set charger armor\n", __func__);
		goto cyttsp4_ic_charmor_store_exit;
	}

cyttsp4_ic_charmor_store_exit:
	retval = size;
	return retval;
}
static DEVICE_ATTR(ic_charmor, S_IRUSR | S_IWUSR,
		cyttsp4_ic_charmor_show, cyttsp4_ic_charmor_store);

#endif /* --CONFIG_TOUCHSCREEN_DEBUG */

#define CY_CMD_I2C_ADDR					0
#define CY_STATUS_SIZE_BYTE				1
#define CY_STATUS_TYP_DELAY				2
#define CY_CMD_TAIL_LEN					3
#define CY_CMD_BYTE					1
#define CY_STATUS_BYTE					1
#define CY_MAX_STATUS_SIZE				32
#define CY_MIN_STATUS_SIZE				5
#define CY_START_OF_PACKET				0x01
#define CY_END_OF_PACKET				0x17
#define CY_DATA_ROW_SIZE				288
#define CY_DATA_ROW_SIZE_TMA400				128
#define CY_PACKET_DATA_LEN				96
#define CY_MAX_PACKET_LEN				512
#define CY_COMM_BUSY					0xFF
#define CY_CMD_BUSY					0xFE
#define CY_SEPARATOR_OFFSET				0
#define CY_ARRAY_ID_OFFSET				0
#define CY_ROW_NUM_OFFSET				1
#define CY_ROW_SIZE_OFFSET				3
#define CY_ROW_DATA_OFFSET				5
#define CY_FILE_SILICON_ID_OFFSET			0
#define CY_FILE_REV_ID_OFFSET				4
#define CY_CMD_LDR_HOST_SYNC				0xFF /* tma400 */
#define CY_CMD_LDR_EXIT					0x3B
#define CY_CMD_LDR_EXIT_CMD_SIZE			7
#define CY_CMD_LDR_EXIT_STAT_SIZE			7

enum ldr_status {
	ERROR_SUCCESS = 0,
	ERROR_COMMAND = 1,
	ERROR_FLASH_ARRAY = 2,
	ERROR_PACKET_DATA = 3,
	ERROR_PACKET_LEN = 4,
	ERROR_PACKET_CHECKSUM = 5,
	ERROR_FLASH_PROTECTION = 6,
	ERROR_FLASH_CHECKSUM = 7,
	ERROR_VERIFY_IMAGE = 8,
	ERROR_UKNOWN1 = 9,
	ERROR_UKNOWN2 = 10,
	ERROR_UKNOWN3 = 11,
	ERROR_UKNOWN4 = 12,
	ERROR_UKNOWN5 = 13,
	ERROR_UKNOWN6 = 14,
	ERROR_INVALID_COMMAND = 15,
	ERROR_INVALID
};

static u16 _cyttsp4_compute_crc(struct cyttsp4 *ts, u8 *buf, int size)
{
	u16 crc = 0xffff;
	u16 tmp;
	int i;

	/* RUN CRC */

	if (size == 0)
		crc = ~crc;
	else {

		do {
			for (i = 0, tmp = 0x00ff & *buf++; i < 8;
				i++, tmp >>= 1) {
				if ((crc & 0x0001) ^ (tmp & 0x0001))
					crc = (crc >> 1) ^ 0x8408;
				else
					crc >>= 1;
			}
		} while (--size);

		crc = ~crc;
		tmp = crc;
		crc = (crc << 8) | (tmp >> 8 & 0xFF);
	}

	return crc;
}

static int _cyttsp4_get_status(struct cyttsp4 *ts,
	u8 *buf, int size, unsigned long timeout_ms)
{
	unsigned long uretval = 0;
	int tries = 0;
	int retval = 0;

	if (timeout_ms != 0) {
		/* wait until status ready interrupt or timeout occurs */
		uretval = wait_for_completion_interruptible_timeout(
			&ts->int_running, msecs_to_jiffies(timeout_ms));

		/* read the status packet */
		if (buf == NULL) {
			dev_err(ts->dev,
			"%s: Status buf ptr is NULL\n", __func__);
			retval = -EINVAL;
			goto _cyttsp4_get_status_exit;
		}
		for (tries = 0; tries < 2; tries++) {
			retval = _cyttsp4_read_block_data(ts, CY_REG_BASE, size,
				buf, ts->platform_data->addr[CY_LDR_ADDR_OFS],
#ifdef CY_USE_TMA400
		true);
#endif /* --CY_USE_TMA400 */
#ifdef CY_USE_TMA884
		false);
#endif /* --CY_USE_TMA884 */
			/*
			 * retry if bus read error or
			 * status byte shows not ready
			 */
			if ((buf[1] == CY_COMM_BUSY) || (buf[1] == CY_CMD_BUSY))
				msleep(CY_DELAY_DFLT);
			else
				break;
		}
		dev_vdbg(ts->dev,
			"%s: tries=%d ret=%d status=%02X\n",
			__func__, tries, retval, buf[1]);
	}

_cyttsp4_get_status_exit:
	mutex_lock(&ts->data_lock);
	return retval;
}

/*
 * Send a bootloader command to the device;
 * Wait for the ISR to execute indicating command
 * was received and status is ready;
 * Releases data_lock mutex to allow ISR to run,
 * then locks it again.
 */
static int _cyttsp4_send_cmd(struct cyttsp4 *ts, const u8 *cmd_buf,
	int cmd_size, u8 *stat_ret, size_t num_stat_byte,
	size_t status_size, unsigned long timeout_ms)
{
	u8 *status_buf = NULL;
	int retval = 0;

	if (timeout_ms > 0) {
		status_buf = kzalloc(CY_MAX_STATUS_SIZE, GFP_KERNEL);
		if (status_buf == NULL) {
			dev_err(ts->dev,
			"%s: Fail alloc status buffer=%p\n",
				__func__, status_buf);
			goto _cyttsp4_send_cmd_exit;
		}
	}

	if (cmd_buf == NULL) {
		dev_err(ts->dev,
			"%s: bad cmd_buf=%p\n", __func__, cmd_buf);
		goto _cyttsp4_send_cmd_exit;
	}

	if (cmd_size == 0) {
		dev_err(ts->dev,
			"%s: bad cmd_size=%d\n", __func__, cmd_size);
		goto _cyttsp4_send_cmd_exit;
	}

	_cyttsp4_pr_buf(ts, (u8 *)cmd_buf, cmd_size, "send_cmd");

	mutex_unlock(&ts->data_lock);
	if (timeout_ms > 0)
		INIT_COMPLETION(ts->int_running);
	retval = _cyttsp4_write_block_data(ts, CY_REG_BASE, cmd_size, cmd_buf,
		ts->platform_data->addr[CY_LDR_ADDR_OFS],
#ifdef CY_USE_TMA400
		true);
#endif /* --CY_USE_TMA400 */
#ifdef CY_USE_TMA884
		false);
#endif /* --CY_USE_TMA884 */
	if (retval < 0) {
		dev_err(ts->dev,
			"%s: Fail writing command=%02X\n",
			__func__, cmd_buf[CY_CMD_BYTE]);
		mutex_lock(&ts->data_lock);
		goto _cyttsp4_send_cmd_exit;
	}

	/* get the status and lock the mutex */
	if (timeout_ms > 0) {
		retval = _cyttsp4_get_status(ts, status_buf,
			status_size, timeout_ms);
		if ((retval < 0) || (status_buf[0] != CY_START_OF_PACKET)) {
			dev_err(ts->dev,
			"%s: Error getting status r=%d"
				" status_buf[0]=%02X\n",
				__func__, retval, status_buf[0]);
			if (!(retval < 0))
				retval = -EIO;
			goto _cyttsp4_send_cmd_exit;
		} else {
			if (status_buf[CY_STATUS_BYTE] != ERROR_SUCCESS) {
				dev_err(ts->dev,
			"%s: Status=0x%02X error\n",
					__func__, status_buf[CY_STATUS_BYTE]);
				retval = -EIO;
			} else if (stat_ret != NULL) {
				if (num_stat_byte < status_size)
					*stat_ret = status_buf[num_stat_byte];
				else
					*stat_ret = 0;
			}
		}
	} else {
		if (stat_ret != NULL)
			*stat_ret = ERROR_SUCCESS;
		mutex_lock(&ts->data_lock);
	}

_cyttsp4_send_cmd_exit:
	if (status_buf != NULL)
		kfree(status_buf);
	return retval;
}

struct cyttsp4_dev_id {
	u32 silicon_id;
	u8 rev_id;
	u32 bl_ver;
};

#if !defined(CY_NO_AUTO_LOAD) || \
	defined(CY_USE_FORCE_LOAD) || \
	defined(CONFIG_TOUCHSCREEN_DEBUG) || \
	defined(SH_TPSIF_COMMAND)
#define CY_CMD_LDR_ENTER				0x38
#define CY_CMD_LDR_ENTER_CMD_SIZE			7
#define CY_CMD_LDR_ENTER_STAT_SIZE			15
#define CY_CMD_LDR_INIT					0x48
#define CY_CMD_LDR_INIT_CMD_SIZE			15
#define CY_CMD_LDR_INIT_STAT_SIZE			7
#define CY_CMD_LDR_ERASE_ROW				0x34
#define CY_CMD_LDR_ERASE_ROW_CMD_SIZE			10
#define CY_CMD_LDR_ERASE_ROW_STAT_SIZE			7
#define CY_CMD_LDR_SEND_DATA				0x37
#define CY_CMD_LDR_SEND_DATA_CMD_SIZE			4 /* hdr bytes only */
#define CY_CMD_LDR_SEND_DATA_STAT_SIZE			8
#define CY_CMD_LDR_PROG_ROW				0x39
#define CY_CMD_LDR_PROG_ROW_CMD_SIZE			7 /* hdr bytes only */
#define CY_CMD_LDR_PROG_ROW_STAT_SIZE			7
#define CY_CMD_LDR_VERIFY_ROW				0x3A
#define CY_CMD_LDR_VERIFY_ROW_STAT_SIZE			8
#define CY_CMD_LDR_VERIFY_ROW_CMD_SIZE			10
#define CY_CMD_LDR_VERIFY_CHKSUM			0x31
#define CY_CMD_LDR_VERIFY_CHKSUM_CMD_SIZE		7
#define CY_CMD_LDR_VERIFY_CHKSUM_STAT_SIZE		8

static u16 _cyttsp4_get_short(u8 *buf)
{
	return ((u16)(*buf) << 8) + *(buf+1);
}

static u8 *_cyttsp4_get_row(struct cyttsp4 *ts,
	u8 *row_buf, u8 *image_buf, int size)
{
	int i;
	for (i = 0; i < size; i++) {
		/* copy a row from the image */
		row_buf[i] = image_buf[i];
	}

	image_buf = image_buf + size;
	return image_buf;
}

static int _cyttsp4_ldr_enter(struct cyttsp4 *ts, struct cyttsp4_dev_id *dev_id)
{
	u16 crc;
	int i = 0;
	size_t cmd_size;
	u8 status_buf[CY_MAX_STATUS_SIZE];
	u8 status = 0;
	int retval = 0;
	/* +1 for TMA400 host sync byte */
	u8 ldr_enter_cmd[CY_CMD_LDR_ENTER_CMD_SIZE+1];

	memset(status_buf, 0, sizeof(status_buf));
	dev_id->bl_ver = 0;
	dev_id->rev_id = 0;
	dev_id->silicon_id = 0;

#ifdef CY_USE_TMA400
	ldr_enter_cmd[i++] = CY_CMD_LDR_HOST_SYNC;
#endif /* --CY_USE_TMA400 */
	ldr_enter_cmd[i++] = CY_START_OF_PACKET;
	ldr_enter_cmd[i++] = CY_CMD_LDR_ENTER;
	ldr_enter_cmd[i++] = 0x00;	/* data len lsb */
	ldr_enter_cmd[i++] = 0x00;	/* data len msb */
#ifdef CY_USE_TMA400
	crc = _cyttsp4_compute_crc(ts, &ldr_enter_cmd[1], i - 1);
	cmd_size = sizeof(ldr_enter_cmd);
#endif /* --CY_USE_TMA400 */
#ifdef CY_USE_TMA884
	crc = _cyttsp4_compute_crc(ts, ldr_enter_cmd, i);
	cmd_size = sizeof(ldr_enter_cmd) - 1;
#endif /* --CY_USE_TMA884 */
	ldr_enter_cmd[i++] = (u8)crc;
	ldr_enter_cmd[i++] = (u8)(crc >> 8);
	ldr_enter_cmd[i++] = CY_END_OF_PACKET;

	mutex_unlock(&ts->data_lock);
	INIT_COMPLETION(ts->int_running);
	retval = _cyttsp4_write_block_data(ts, CY_REG_BASE, cmd_size,
		ldr_enter_cmd, ts->platform_data->addr[CY_LDR_ADDR_OFS],
#ifdef CY_USE_TMA400
		true);
#endif /* --CY_USE_TMA400 */
#ifdef CY_USE_TMA884
		false);
#endif /* --CY_USE_TMA884 */
	if (retval < 0) {
		dev_err(ts->dev,
			"%s: write block failed %d\n", __func__, retval);
		goto _cyttsp4_ldr_enter_exit;
	}

	/* Wait for ISR, get status and lock mutex */
	retval = _cyttsp4_get_status(ts, status_buf,
		CY_CMD_LDR_ENTER_STAT_SIZE, CY_HALF_SEC_TMO_MS);

	if (retval < 0) {
		dev_err(ts->dev,
			"%s: Fail get status to Enter Loader command r=%d\n",
			__func__, retval);
	} else {
		status = status_buf[CY_STATUS_BYTE];
		if (status == ERROR_SUCCESS) {
			dev_id->bl_ver =
				status_buf[11] << 16 |
				status_buf[10] <<  8 |
				status_buf[9] <<  0;
			dev_id->rev_id =
				status_buf[8] <<  0;
			dev_id->silicon_id =
				status_buf[7] << 24 |
				status_buf[6] << 16 |
				status_buf[5] <<  8 |
				status_buf[4] <<  0;
			retval = 0;
		} else
			retval = -EIO;

		dev_vdbg(ts->dev,
			"%s: status=%d "
			"bl_ver=%08X rev_id=%02X silicon_id=%08X\n",
			__func__, status,
			dev_id->bl_ver, dev_id->rev_id, dev_id->silicon_id);
	}

_cyttsp4_ldr_enter_exit:
	return retval;
}

#ifdef CY_USE_TMA400
static int _cyttsp4_ldr_init(struct cyttsp4 *ts)
{
	u16 crc;
	int i = 0;
	int retval = 0;
	/* +1 for TMA400 host sync byte */
	u8 ldr_init_cmd[CY_CMD_LDR_INIT_CMD_SIZE+1];

	ldr_init_cmd[i++] = CY_CMD_LDR_HOST_SYNC;
	ldr_init_cmd[i++] = CY_START_OF_PACKET;
	ldr_init_cmd[i++] = CY_CMD_LDR_INIT;
	ldr_init_cmd[i++] = 0x08;	/* data len lsb */
	ldr_init_cmd[i++] = 0x00;	/* data len msb */
	memcpy(&ldr_init_cmd[i], cyttsp4_security_key,
		sizeof(cyttsp4_security_key));
	i += sizeof(cyttsp4_security_key);
	crc = _cyttsp4_compute_crc(ts, &ldr_init_cmd[1], i - 1);
	ldr_init_cmd[i++] = (u8)crc;
	ldr_init_cmd[i++] = (u8)(crc >> 8);
	ldr_init_cmd[i++] = CY_END_OF_PACKET;

	retval = _cyttsp4_send_cmd(ts, ldr_init_cmd, i, NULL, 0,
		CY_CMD_LDR_INIT_STAT_SIZE, CY_TEN_SEC_TMO_MS);
	if (retval < 0) {
		dev_err(ts->dev,
			"%s: Fail ldr init r=%d\n",
			__func__, retval);
	}

	return retval;
}
#endif /* --CY_USE_TMA400 */

struct cyttsp4_hex_image {
	u8 array_id;
	u16 row_num;
	u16 row_size;
	u8 row_data[CY_DATA_ROW_SIZE];
} __packed;

#ifdef CY_USE_TMA884
static int _cyttsp4_ldr_erase_row(struct cyttsp4 *ts,
	struct cyttsp4_hex_image *row_image)
{
	u16 crc;
	int i = 0;
	int retval = 0;
	/* +1 for TMA400 host sync byte */
	u8 ldr_erase_row_cmd[CY_CMD_LDR_ERASE_ROW_CMD_SIZE+1];

#ifdef CY_USE_TMA400
	ldr_erase_row_cmd[i++] = CY_CMD_LDR_HOST_SYNC;
#endif /* --CY_USE_TMA400 */
	ldr_erase_row_cmd[i++] = CY_START_OF_PACKET;
	ldr_erase_row_cmd[i++] = CY_CMD_LDR_ERASE_ROW;
	ldr_erase_row_cmd[i++] = 0x03;	/* data len lsb */
	ldr_erase_row_cmd[i++] = 0x00;	/* data len msb */
	ldr_erase_row_cmd[i++] = row_image->array_id;
	ldr_erase_row_cmd[i++] = (u8)row_image->row_num;
	ldr_erase_row_cmd[i++] = (u8)(row_image->row_num >> 8);
#ifdef CY_USE_TMA400
	crc = _cyttsp4_compute_crc(ts, &ldr_erase_row_cmd[1], i - 1);
#endif /* --CY_USE_TMA400 */
#ifdef CY_USE_TMA884
	crc = _cyttsp4_compute_crc(ts, ldr_erase_row_cmd, i);
#endif /* --CY_USE_TMA884 */
	ldr_erase_row_cmd[i++] = (u8)crc;
	ldr_erase_row_cmd[i++] = (u8)(crc >> 8);
	ldr_erase_row_cmd[i++] = CY_END_OF_PACKET;

	retval = _cyttsp4_send_cmd(ts, ldr_erase_row_cmd, i, NULL, 0,
		CY_CMD_LDR_ERASE_ROW_STAT_SIZE, CY_HALF_SEC_TMO_MS);

	if (retval < 0) {
		dev_err(ts->dev,
			"%s: Fail erase row=%d r=%d\n",
			__func__, row_image->row_num, retval);
	}
	return retval;
}
#endif

static int _cyttsp4_ldr_parse_row(struct cyttsp4 *ts, u8 *row_buf,
	struct cyttsp4_hex_image *row_image)
{
	u16 i, j;
	int retval = 0;

	if (!row_buf) {
		dev_err(ts->dev,
			"%s parse row error - buf is null\n", __func__);
		retval = -EINVAL;
		goto cyttsp4_ldr_parse_row_exit;
	}

	row_image->array_id = row_buf[CY_ARRAY_ID_OFFSET];
	row_image->row_num = _cyttsp4_get_short(&row_buf[CY_ROW_NUM_OFFSET]);
	row_image->row_size = _cyttsp4_get_short(&row_buf[CY_ROW_SIZE_OFFSET]);

	if (row_image->row_size > ARRAY_SIZE(row_image->row_data)) {
		dev_err(ts->dev,
			"%s: row data buffer overflow\n", __func__);
		retval = -EOVERFLOW;
		goto cyttsp4_ldr_parse_row_exit;
	}

	for (i = 0, j = CY_ROW_DATA_OFFSET;
		i < row_image->row_size; i++)
		row_image->row_data[i] = row_buf[j++];

	retval = 0;

cyttsp4_ldr_parse_row_exit:
	return retval;
}

static int _cyttsp4_ldr_prog_row(struct cyttsp4 *ts,
	struct cyttsp4_hex_image *row_image)
{
	u16 crc;
	int next;
	int data;
	int row_data;
	u16 row_sum = 0;
	size_t data_len;
#ifdef CY_USE_TMA884
	int segment;
#endif /* --CY_USE_TMA884 */
	int retval = 0;

	u8 *cmd = kzalloc(CY_MAX_PACKET_LEN, GFP_KERNEL);

	if (cmd != NULL) {
		row_data = 0;
		row_sum = 0;

#ifdef CY_USE_TMA884
		for (segment = 0; segment <
			(CY_DATA_ROW_SIZE/CY_PACKET_DATA_LEN)-1;
			segment++) {
			next = 0;
			cmd[next++] = CY_START_OF_PACKET;
			cmd[next++] = CY_CMD_LDR_SEND_DATA;
			cmd[next++] = (u8)CY_PACKET_DATA_LEN;
			cmd[next++] = (u8)(CY_PACKET_DATA_LEN >> 8);

			for (data = 0;
				data < CY_PACKET_DATA_LEN; data++) {
				cmd[next] = row_image->row_data
					[row_data++];
				row_sum += cmd[next];
				next++;
			}

			crc = _cyttsp4_compute_crc(ts, cmd, next);
			cmd[next++] = (u8)crc;
			cmd[next++] = (u8)(crc >> 8);
			cmd[next++] = CY_END_OF_PACKET;

			retval = _cyttsp4_send_cmd(ts, cmd, next, NULL,
				0, CY_CMD_LDR_SEND_DATA_STAT_SIZE,
				CY_HALF_SEC_TMO_MS);

			if (retval < 0) {
				dev_err(ts->dev,
			"%s: send row=%d segment=%d"
					" fail r=%d\n",
					__func__, row_image->row_num,
					segment, retval);
				goto cyttsp4_ldr_prog_row_exit;
			}
		}
#endif /* --CY_USE_TMA884 */

		next = 0;
#ifdef CY_USE_TMA400
		cmd[next++] = CY_CMD_LDR_HOST_SYNC;
#endif /* --CY_USE_TMA400 */
		cmd[next++] = CY_START_OF_PACKET;
		cmd[next++] = CY_CMD_LDR_PROG_ROW;
		/*
		 * include array id size and row id size in CY_PACKET_DATA_LEN
		 */
#ifdef CY_USE_TMA400
		data_len = CY_DATA_ROW_SIZE_TMA400;
#endif /* --CY_USE_TMA400 */
#ifdef CY_USE_TMA884
		data_len = CY_PACKET_DATA_LEN;
#endif /* --CY_USE_TMA884 */
		cmd[next++] = (u8)(data_len+3);
		cmd[next++] = (u8)((data_len+3) >> 8);
		cmd[next++] = row_image->array_id;
		cmd[next++] = (u8)row_image->row_num;
		cmd[next++] = (u8)(row_image->row_num >> 8);

		for (data = 0;
			data < data_len; data++) {
			cmd[next] = row_image->row_data[row_data++];
			row_sum += cmd[next];
			next++;
		}

#ifdef CY_USE_TMA400
		crc = _cyttsp4_compute_crc(ts, &cmd[1], next - 1);
#endif /* --CY_USE_TMA400 */
#ifdef CY_USE_TMA884
		crc = _cyttsp4_compute_crc(ts, cmd, next);
#endif /* --CY_USE_TMA884 */
		cmd[next++] = (u8)crc;
		cmd[next++] = (u8)(crc >> 8);
		cmd[next++] = CY_END_OF_PACKET;

		retval = _cyttsp4_send_cmd(ts, cmd, next, NULL, 0,
			CY_CMD_LDR_PROG_ROW_STAT_SIZE, CY_HALF_SEC_TMO_MS);

		if (retval < 0) {
			dev_err(ts->dev,
			"%s: prog row=%d fail r=%d\n",
				__func__, row_image->row_num, retval);
			goto cyttsp4_ldr_prog_row_exit;
		}

	} else {
		dev_err(ts->dev,
			"%s prog row error - cmd buf is NULL\n", __func__);
		retval = -EIO;
	}

cyttsp4_ldr_prog_row_exit:
	if (cmd != NULL)
		kfree(cmd);
	return retval;
}

static int _cyttsp4_ldr_verify_row(struct cyttsp4 *ts,
	struct cyttsp4_hex_image *row_image)
{
	u16 crc;
	int i = 0;
	u8 verify_checksum;
	int retval = 0;
	/* +1 for TMA400 host sync byte */
	u8 ldr_verify_row_cmd[CY_CMD_LDR_VERIFY_ROW_CMD_SIZE+1];

#ifdef CY_USE_TMA400
	ldr_verify_row_cmd[i++] = CY_CMD_LDR_HOST_SYNC;
#endif /* --CY_USE_TMA400 */
	ldr_verify_row_cmd[i++] = CY_START_OF_PACKET;
	ldr_verify_row_cmd[i++] = CY_CMD_LDR_VERIFY_ROW;
	ldr_verify_row_cmd[i++] = 0x03;	/* data len lsb */
	ldr_verify_row_cmd[i++] = 0x00;	/* data len msb */
	ldr_verify_row_cmd[i++] = row_image->array_id;
	ldr_verify_row_cmd[i++] = (u8)row_image->row_num;
	ldr_verify_row_cmd[i++] = (u8)(row_image->row_num >> 8);
#ifdef CY_USE_TMA400
	crc = _cyttsp4_compute_crc(ts, &ldr_verify_row_cmd[1], i - 1);
#endif /* --CY_USE_TMA400 */
#ifdef CY_USE_TMA884
	crc = _cyttsp4_compute_crc(ts, ldr_verify_row_cmd, i);
#endif /* --CY_USE_TMA884 */
	ldr_verify_row_cmd[i++] = (u8)crc;
	ldr_verify_row_cmd[i++] = (u8)(crc >> 8);
	ldr_verify_row_cmd[i++] = CY_END_OF_PACKET;

	retval = _cyttsp4_send_cmd(ts, ldr_verify_row_cmd, i,
		&verify_checksum, 4,
		CY_CMD_LDR_VERIFY_ROW_STAT_SIZE, CY_HALF_SEC_TMO_MS);

	if (retval < 0) {
		dev_err(ts->dev,
			"%s: verify row=%d fail r=%d\n",
			__func__, row_image->row_num, retval);
	}

	return retval;
}

static int _cyttsp4_ldr_verify_chksum(struct cyttsp4 *ts, u8 *app_chksum)
{
	u16 crc;
	int i = 0;
	int retval = 0;
	/* +1 for TMA400 host sync byte */
	u8 ldr_verify_chksum_cmd[CY_CMD_LDR_VERIFY_CHKSUM_CMD_SIZE+1];

#ifdef CY_USE_TMA400
	ldr_verify_chksum_cmd[i++] = CY_CMD_LDR_HOST_SYNC;
#endif /* --CY_USE_TMA400 */
	ldr_verify_chksum_cmd[i++] = CY_START_OF_PACKET;
	ldr_verify_chksum_cmd[i++] = CY_CMD_LDR_VERIFY_CHKSUM;
	ldr_verify_chksum_cmd[i++] = 0x00;	/* data len lsb */
	ldr_verify_chksum_cmd[i++] = 0x00;	/* data len msb */
#ifdef CY_USE_TMA400
	crc = _cyttsp4_compute_crc(ts, &ldr_verify_chksum_cmd[1], i - 1);
#endif /* --CY_USE_TMA400 */
#ifdef CY_USE_TMA884
	crc = _cyttsp4_compute_crc(ts, ldr_verify_chksum_cmd, i);
#endif /* --CY_USE_TMA884 */
	ldr_verify_chksum_cmd[i++] = (u8)crc;
	ldr_verify_chksum_cmd[i++] = (u8)(crc >> 8);
	ldr_verify_chksum_cmd[i++] = CY_END_OF_PACKET;

	retval = _cyttsp4_send_cmd(ts, ldr_verify_chksum_cmd, i,
		app_chksum, 4,
		CY_CMD_LDR_VERIFY_CHKSUM_STAT_SIZE, CY_HALF_SEC_TMO_MS);

	if (retval < 0) {
		dev_err(ts->dev,
			"%s: verify checksum fail r=%d\n",
			__func__, retval);
	}

	return retval;
}

static int _cyttsp4_load_app(struct cyttsp4 *ts, const u8 *fw, int fw_size)
{
	u8 *p;
#ifdef CY_USE_TMA884
	u8 tries;
#endif
	int ret;
	int retval;	/* need separate return value at exit stage */
	struct cyttsp4_dev_id *file_id = NULL;
	struct cyttsp4_dev_id *dev_id = NULL;
	struct cyttsp4_hex_image *row_image = NULL;
	u8 app_chksum;

	u8 *row_buf = NULL;
	size_t image_rec_size;
	size_t row_buf_size = 1024 > CY_MAX_PRBUF_SIZE ?
		1024 : CY_MAX_PRBUF_SIZE;
	int row_count = 0;

#ifdef CY_USE_TMA400
	image_rec_size = CY_DATA_ROW_SIZE_TMA400 +
		(sizeof(struct cyttsp4_hex_image) - CY_DATA_ROW_SIZE);
#endif /* --CY_USE_TMA400 */
#ifdef CY_USE_TMA884
	image_rec_size = sizeof(struct cyttsp4_hex_image);
#endif /* --CY_USE_TMA884 */

	if (!fw_size || (fw_size % image_rec_size != 0)) {
		dev_err(ts->dev,
			"%s: Firmware image is misaligned\n", __func__);
		retval = -EINVAL;
		goto _cyttsp4_load_app_exit;
	}

#ifdef CY_USE_WATCHDOG
	_cyttsp4_stop_wd_timer(ts);
#endif

	dev_info(ts->dev,
			"%s: start load app\n", __func__);

	row_buf = kzalloc(row_buf_size, GFP_KERNEL);
	row_image = kzalloc(sizeof(struct cyttsp4_hex_image), GFP_KERNEL);
	file_id = kzalloc(sizeof(struct cyttsp4_dev_id), GFP_KERNEL);
	dev_id = kzalloc(sizeof(struct cyttsp4_dev_id), GFP_KERNEL);
	if ((row_buf == NULL) || (row_image == NULL) ||
		(file_id == NULL) || (dev_id == NULL)) {
		dev_err(ts->dev,
			"%s: Unable to alloc row buffers(%p %p %p %p)\n",
			__func__, row_buf, row_image, file_id, dev_id);
		retval = -ENOMEM;
		goto _cyttsp4_load_app_error_exit;
	}

	p = (u8 *)fw;
	/* Enter Loader and return Silicon ID and Rev */

	retval = _cyttsp4_reset(ts);
	if (retval < 0) {
		dev_err(ts->dev,
			"%s: Fail reset device r=%d\n", __func__, retval);
		goto _cyttsp4_load_app_exit;
	}
	retval = _cyttsp4_wait_int(ts, CY_TEN_SEC_TMO_MS);
	if (retval < 0) {
		dev_err(ts->dev,
			"%s: Fail waiting for bootloader interrupt\n",
			__func__);
		goto _cyttsp4_load_app_exit;
	}

	_cyttsp4_change_state(ts, CY_BL_STATE);
	dev_info(ts->dev,
			"%s: Send BL Loader Enter\n", __func__);
	retval = _cyttsp4_ldr_enter(ts, dev_id);
	if (retval < 0) {
		dev_err(ts->dev,
			"%s: Error cannot start Loader (ret=%d)\n",
			__func__, retval);
		goto _cyttsp4_load_app_error_exit;
	}

	dev_vdbg(ts->dev,
		"%s: dev: silicon id=%08X rev=%02X bl=%08X\n",
		__func__, dev_id->silicon_id,
		dev_id->rev_id, dev_id->bl_ver);

#ifdef CY_USE_TMA400
	udelay(1000);
	retval = _cyttsp4_ldr_init(ts);
	if (retval < 0) {
		dev_err(ts->dev,
			"%s: Error cannot init Loader (ret=%d)\n",
			__func__, retval);
		goto _cyttsp4_load_app_error_exit;
	}
#endif /* --CY_USE_TMA400 */

	dev_info(ts->dev,
			"%s: Send BL Loader Blocks\n", __func__);
	while (p < (fw + fw_size)) {
		/* Get row */
		dev_dbg(ts->dev,
			"%s: read row=%d\n", __func__, ++row_count);
		memset(row_buf, 0, row_buf_size);
		p = _cyttsp4_get_row(ts, row_buf, p, image_rec_size);

		/* Parse row */
		dev_vdbg(ts->dev,
			"%s: p=%p buf=%p buf[0]=%02X\n", __func__,
			p, row_buf, row_buf[0]);
		retval = _cyttsp4_ldr_parse_row(ts, row_buf, row_image);
		dev_vdbg(ts->dev,
			"%s: array_id=%02X row_num=%04X(%d)"
				" row_size=%04X(%d)\n", __func__,
			row_image->array_id,
			row_image->row_num, row_image->row_num,
			row_image->row_size, row_image->row_size);
		if (retval < 0) {
			dev_err(ts->dev,
			"%s: Parse Row Error "
				"(a=%d r=%d ret=%d\n",
				__func__, row_image->array_id,
				row_image->row_num,
				retval);
			goto bl_exit;
		} else {
			dev_vdbg(ts->dev,
				"%s: Parse Row "
				"(a=%d r=%d ret=%d\n",
				__func__, row_image->array_id,
				row_image->row_num, retval);
		}

#ifdef CY_USE_TMA884
		/* erase row */
		tries = 0;
		do {
			retval = _cyttsp4_ldr_erase_row(ts, row_image);
			if (retval < 0) {
				dev_err(ts->dev,
			"%s: Erase Row Error "
					"(array=%d row=%d ret=%d try=%d)\n",
					__func__, row_image->array_id,
					row_image->row_num, retval, tries);
			}
		} while (retval && tries++ < 5);

		if (retval < 0)
			goto _cyttsp4_load_app_error_exit;
#endif

		/* program row */
		retval = _cyttsp4_ldr_prog_row(ts, row_image);
		if (retval < 0) {
			dev_err(ts->dev,
			"%s: Program Row Error "
				"(array=%d row=%d ret=%d)\n",
				__func__, row_image->array_id,
				row_image->row_num, retval);
			goto _cyttsp4_load_app_error_exit;
		}

		/* verify row */
		retval = _cyttsp4_ldr_verify_row(ts, row_image);
		if (retval < 0) {
			dev_err(ts->dev,
			"%s: Verify Row Error "
				"(array=%d row=%d ret=%d)\n",
				__func__, row_image->array_id,
				row_image->row_num, retval);
			goto _cyttsp4_load_app_error_exit;
		}

		dev_vdbg(ts->dev,
			"%s: array=%d row_cnt=%d row_num=%04X\n",
			__func__, row_image->array_id, row_count,
			row_image->row_num);
	}

	/* verify app checksum */
	retval = _cyttsp4_ldr_verify_chksum(ts, &app_chksum);
	dev_dbg(ts->dev,
		"%s: Application Checksum = %02X r=%d\n",
		__func__, app_chksum, retval);
	if (retval < 0) {
		dev_err(ts->dev,
			"%s: ldr_verify_chksum fail r=%d\n", __func__, retval);
		retval = 0;
	}

	/* exit loader */
bl_exit:
	dev_info(ts->dev,
			"%s: Send BL Loader Terminate\n", __func__);
	ret = _cyttsp4_ldr_exit(ts);
	if (ret) {
		dev_err(ts->dev,
			"%s: Error on exit Loader (ret=%d)\n",
			__func__, ret);
		retval = ret;
		goto _cyttsp4_load_app_error_exit;
	}

	/*
	 * this is a temporary parking state;
	 * the driver will always run startup
	 * after the loader has completed
	 */
	_cyttsp4_change_state(ts, CY_TRANSFER_STATE);
	goto _cyttsp4_load_app_exit;

_cyttsp4_load_app_error_exit:
	_cyttsp4_change_state(ts, CY_BL_STATE);
_cyttsp4_load_app_exit:
	kfree(row_buf);
	kfree(row_image);
	kfree(file_id);
	kfree(dev_id);
	return retval;
}
#endif /* !CY_NO_AUTO_LOAD || CY_USE_FORCE_LOAD || CONFIG_TOUCHSCREEN_DEBUG || SH_TPSIF_COMMAND */

/* Constructs loader exit command and sends via _cyttsp4_send_cmd() */
static int _cyttsp4_ldr_exit(struct cyttsp4 *ts)
{
	u16 crc;
	int i = 0;
	int retval = 0;
	/* +1 for TMA400 host sync byte */
	u8 ldr_exit_cmd[CY_CMD_LDR_EXIT_CMD_SIZE+1];

#ifdef CY_USE_TMA400
	ldr_exit_cmd[i++] = CY_CMD_LDR_HOST_SYNC;
#endif /* --CY_USE_TMA400 */
	ldr_exit_cmd[i++] = CY_START_OF_PACKET;
	ldr_exit_cmd[i++] = CY_CMD_LDR_EXIT;
	ldr_exit_cmd[i++] = 0x00;	/* data len lsb */
	ldr_exit_cmd[i++] = 0x00;	/* data len msb */
#ifdef CY_USE_TMA400
	crc = _cyttsp4_compute_crc(ts, &ldr_exit_cmd[1], i - 1);
#endif /* --CY_USE_TMA400 */
#ifdef CY_USE_TMA884
	crc = _cyttsp4_compute_crc(ts, ldr_exit_cmd, i);
#endif /* --CY_USE_TMA884 */
	ldr_exit_cmd[i++] = (u8)crc;
	ldr_exit_cmd[i++] = (u8)(crc >> 8);
	ldr_exit_cmd[i++] = CY_END_OF_PACKET;

	retval = _cyttsp4_send_cmd(ts, ldr_exit_cmd, i, NULL, 0,
		CY_CMD_LDR_EXIT_STAT_SIZE, 0);

	if (retval < 0) {
		dev_err(ts->dev,
			"%s: BL Loader exit fail r=%d\n",
			__func__, retval);
	}

	dev_vdbg(ts->dev,
		"%s: Exit BL Loader r=%d\n", __func__, retval);

	return retval;
}

#if defined(CY_USE_FORCE_LOAD) || defined(CONFIG_TOUCHSCREEN_DEBUG) || defined(SH_TPSIF_COMMAND)
/* Force firmware upgrade */
static void cyttsp4_firmware_cont(const struct firmware *fw, void *context)
{
	int retval = 0;
	struct device *dev = context;
	struct cyttsp4 *ts = dev_get_drvdata(dev);
	u8 header_size = 0;

	mutex_lock(&ts->data_lock);

	if (fw == NULL) {
		dev_err(ts->dev,
			"%s: Firmware not found\n", __func__);
		goto cyttsp4_firmware_cont_exit;
	}

	if ((fw->data == NULL) || (fw->size == 0)) {
		dev_err(ts->dev,
			"%s: No firmware received\n", __func__);
		goto cyttsp4_firmware_cont_release_exit;
	}

	header_size = fw->data[0];
	if (header_size >= (fw->size + 1)) {
		dev_err(ts->dev,
			"%s: Firmware format is invalid\n", __func__);
		goto cyttsp4_firmware_cont_release_exit;
	}
	retval = _cyttsp4_load_app(ts, &(fw->data[header_size + 1]),
		fw->size - (header_size + 1));
	if (retval < 0) {
		dev_err(ts->dev,
			"%s: Firmware update failed with error code %d\n",
			__func__, retval);
		_cyttsp4_change_state(ts, CY_IDLE_STATE);
		retval = -EIO;
		goto cyttsp4_firmware_cont_release_exit;
	}

#ifdef CONFIG_TOUCHSCREEN_DEBUG
	ts->debug_upgrade = true;
#endif /* --CONFIG_TOUCHSCREEN_DEBUG */

	retval = _cyttsp4_startup(ts);
	if (retval < 0) {
		dev_err(ts->dev,
			"%s: Failed to restart IC with error code %d\n",
			__func__, retval);
		_cyttsp4_change_state(ts, CY_IDLE_STATE);
	}

#ifdef SH_TPSIF_COMMAND
#ifdef CALIBRATION_AFTER_FIRMWARE_UPDATE
	dev_info(ts->dev, "%s: calibration after firmware update\n", __func__);
	retval = _sh_tpsif_calibration_idac(ts);
	if (retval < 0)
		dev_err(ts->dev, "%s: fail calibration\n", __func__);
#ifdef RESET_AFTER_CALIBRATION
	_cyttsp4_startup(ts);
#endif	/* RESET_AFTER_CALIBRATION */
#endif	/* CALIBRATION_AFTER_FIRMWARE_UPDATE */
#endif	/* SH_TPSIF_COMMAND */

cyttsp4_firmware_cont_release_exit:
	release_firmware(fw);

cyttsp4_firmware_cont_exit:
	ts->waiting_for_fw = false;
	mutex_unlock(&ts->data_lock);
	return;
}
static ssize_t cyttsp4_ic_reflash_show(struct device *dev,
		struct device_attribute *attr, char *buf)
{
	static const char *wait_fw_ld = "Driver is waiting for firmware load\n";
	static const char *no_fw_ld = "No firmware loading in progress\n";
	struct cyttsp4 *ts = dev_get_drvdata(dev);

	if (ts->waiting_for_fw)
		return snprintf(buf, strlen(wait_fw_ld)+1, wait_fw_ld);
	else
		return snprintf(buf, strlen(no_fw_ld)+1, no_fw_ld);
}
static ssize_t cyttsp4_ic_reflash_store(struct device *dev,
		struct device_attribute *attr, const char *buf, size_t size)
{
	int i;
	int retval = 0;
	struct cyttsp4 *ts = dev_get_drvdata(dev);

	if (ts->waiting_for_fw) {
		dev_err(ts->dev,
			"%s: Driver is already waiting for firmware\n",
			__func__);
		retval = -EALREADY;
		goto cyttsp4_ic_reflash_store_exit;
	}

	/*
	 * must configure FW_LOADER in .config file
	 * CONFIG_HOTPLUG=y
	 * CONFIG_FW_LOADER=y
	 * CONFIG_FIRMWARE_IN_KERNEL=y
	 * CONFIG_EXTRA_FIRMWARE=""
	 * CONFIG_EXTRA_FIRMWARE_DIR=""
	 */

	if (size > CY_BL_FW_NAME_SIZE) {
		dev_err(ts->dev,
			"%s: Filename too long\n", __func__);
		retval = -ENAMETOOLONG;
		goto cyttsp4_ic_reflash_store_exit;
	} else {
		/*
		 * name string must be in alloc() memory
		 * or is lost on context switch
		 * strip off any line feed character(s)
		 * at the end of the buf string
		 */
		for (i = 0; buf[i]; i++) {
			if (buf[i] < ' ')
				ts->fwname[i] = 0;
			else
				ts->fwname[i] = buf[i];
		}
	}

	dev_vdbg(ts->dev,
		"%s: Enabling firmware class loader\n", __func__);

	retval = request_firmware_nowait(THIS_MODULE,
		FW_ACTION_NOHOTPLUG, (const char *)ts->fwname, ts->dev,
		GFP_KERNEL, ts->dev, cyttsp4_firmware_cont);
	if (retval < 0) {
		dev_err(ts->dev,
			"%s: Fail request firmware class file load\n",
			__func__);
		ts->waiting_for_fw = false;
		goto cyttsp4_ic_reflash_store_exit;
	} else {
		ts->waiting_for_fw = true;
		retval = size;
	}

cyttsp4_ic_reflash_store_exit:
	return retval;
}
static DEVICE_ATTR(ic_reflash, S_IRUSR | S_IWUSR,
	cyttsp4_ic_reflash_show, cyttsp4_ic_reflash_store);
#endif /* CY_USE_FORCE_LOAD || CONFIG_TOUCHSCREEN_DEBUG || SH_TPSIF_COMMAND */

#ifdef CY_USE_TMA884
static int _cyttsp4_calc_data_crc(struct cyttsp4 *ts, size_t ndata, u8 *pdata,
	u8 *crc_h, u8 *crc_l, const char *name)
{
	int retval = 0;
	u8 *buf = NULL;

	*crc_h = 0;
	*crc_l = 0;

	buf = kzalloc(sizeof(uint8_t) * 126, GFP_KERNEL);
	if (buf == NULL) {
		dev_err(ts->dev,
			"%s: Failed to allocate buf\n", __func__);
		retval = -ENOMEM;
		goto _cyttsp4_calc_data_crc_exit;
	}

	if (pdata == NULL) {
		dev_err(ts->dev,
			"%s: bad data pointer\n", __func__);
		retval = -ENXIO;
		goto _cyttsp4_calc_data_crc_exit;
	}

	if (ndata > 122) {
		dev_err(ts->dev,
			"%s: %s is too large n=%d size=%d\n",
			__func__, name, ndata, 126);
		retval = -EOVERFLOW;
		goto _cyttsp4_calc_data_crc_exit;
	}

	buf[0] = 0x00; /* num of config bytes + 4 high */
	buf[1] = 0x7E; /* num of config bytes + 4 low */
	buf[2] = 0x00; /* max block size w/o crc high */
	buf[3] = 0x7E; /* max block size w/o crc low */

	/* Copy platform data */
	memcpy(&(buf[4]), pdata, ndata);

	/* Calculate CRC */
	_cyttsp4_calc_crc(ts, buf, 126, crc_h, crc_l);

	dev_vdbg(ts->dev,
		"%s: crc=%02X%02X\n", __func__, *crc_h, *crc_l);

_cyttsp4_calc_data_crc_exit:
	kfree(buf);
	return retval;
}
#endif /* --CY_USE_TMA884 */

#ifdef CONFIG_TOUCHSCREEN_DEBUG
#ifdef CY_USE_TMA400
static int _cyttsp4_calc_ic_crc_tma400(struct cyttsp4 *ts,
	enum cyttsp4_ic_ebid ebid, u8 *crc_h, u8 *crc_l, bool read_back_verify)
{
	u16 crc = 0x0000;
	size_t crc_loc = 0;
	size_t crc_row = 0;
	size_t crc_ofs = 0;
	size_t ndata = 0;
	int row_id = 0;
	u8 *pdata = NULL;
	size_t ntable = 0;
	size_t tsize = 0;
	u8 *ptable = NULL;
	bool match = true;
	int i = 0;
	int retval = 0;

	pdata = kzalloc(ts->ebid_row_size, GFP_KERNEL);
	if (pdata == NULL) {
		dev_err(ts->dev,
			"%s: Fail allocate block buffer\n", __func__);
		retval = -ENOMEM;
		goto _cyttsp4_calc_ic_tch_crc_tma400_exit;
	}

	if (read_back_verify) {
		if (ts->platform_data->sett
			[CY_IC_GRPNUM_TCH_PARM_VAL] == NULL) {
			dev_err(ts->dev,
			"%s: NULL param values table\n",
				__func__);
			goto _cyttsp4_calc_ic_tch_crc_tma400_exit;
		} else if (ts->platform_data->sett
			[CY_IC_GRPNUM_TCH_PARM_VAL]->data == NULL) {
			dev_err(ts->dev,
			"%s: NULL param values table data\n",
				__func__);
			goto _cyttsp4_calc_ic_tch_crc_tma400_exit;
		} else if (ts->platform_data->sett
			[CY_IC_GRPNUM_TCH_PARM_VAL]->size == 0) {
			dev_err(ts->dev,
			"%s: param values table size is 0\n",
				__func__);
			goto _cyttsp4_calc_ic_tch_crc_tma400_exit;
		} else {
			ptable = (u8 *)ts->platform_data->sett
				[CY_IC_GRPNUM_TCH_PARM_VAL]->data;
			tsize = ts->platform_data->sett
				[CY_IC_GRPNUM_TCH_PARM_VAL]->size;
		}
	}

	crc = 0xFFFF;
	row_id = 0;
	dev_vdbg(ts->dev,
		"%s: tch ebid=%d row=%d data:\n", __func__, ebid, row_id);
	retval = _cyttsp4_get_ebid_data_tma400(ts, ebid, row_id, pdata);
	if (retval < 0) {
		dev_err(ts->dev,
			"%s: Fail get ebid=%d row=%d data r=%d\n",
			__func__, ebid, row_id, retval);
		retval = -EIO;
		goto _cyttsp4_calc_ic_tch_crc_tma400_exit;
	}
	_cyttsp4_pr_buf(ts, pdata, ts->ebid_row_size, "ebid_data");
	/* determine CRC location */
	crc_loc = (pdata[3] * 256) + pdata[2];
	crc_ofs = crc_loc % ts->ebid_row_size;
	crc_row = crc_loc / ts->ebid_row_size;
	dev_vdbg(ts->dev,
		"%s: tch ebid=%d crc_loc=%08X crc_row=%d crc_ofs=%d data:\n",
		__func__, ebid, crc_loc, crc_row, crc_ofs);

	ndata = 0;
	/* if CRC is in row 0, then the loop is skipped */
	for (row_id = 0; row_id < crc_row; row_id++) {
		dev_vdbg(ts->dev,
			"%s: Get CRC bytes for ebid=%d row=%d crc_row=%d\n",
			__func__, ebid, row_id, crc_row);
		retval = _cyttsp4_get_ebid_data_tma400(ts, ebid, row_id, pdata);
		if (retval < 0) {
			dev_err(ts->dev,
			"%s: Fail get row=%d data r=%d\n",
				__func__, row_id, retval);
			retval = -EIO;
			goto _cyttsp4_calc_ic_tch_crc_tma400_exit;
		}
		_cyttsp4_pr_buf(ts, pdata, ts->ebid_row_size, "ebid_data");
		crc = _cyttsp4_calc_partial_crc(ts,
			pdata, ts->ebid_row_size, crc);
		if (read_back_verify  && (ntable < tsize)) {
			for (i = 0; match && i < ts->ebid_row_size; i++) {
				if (ptable[ntable] != pdata[i]) {
					dev_vdbg(ts->dev,
						"%s: read back err row=%d"
						" table[%d]=%02X"
						" pdata[%d]=%02X\n",
						__func__, row_id,
						ntable, ptable[ntable],
						i, pdata[i]);
					match = false;
				}
				ntable++;
				if (ntable >= tsize) {
					dev_err(ts->dev,
			"%s: row=%d ntbl=%d tsz=%d\n",
						__func__, row_id,
						ntable, tsize);
					break;
				}
			}
		}
		ndata += ts->ebid_row_size;
	}
	/* last row is partial and contains the CRC */
	dev_vdbg(ts->dev,
		"%s: Get CRC bytes for row=%d crc_row=%d\n",
		__func__, crc_row, crc_row);
	retval = _cyttsp4_get_ebid_data_tma400(ts, ebid, crc_row, pdata);
	if (retval < 0) {
		dev_err(ts->dev,
			"%s: Fail get row=%d data r=%d\n",
			__func__, crc_row, retval);
		retval = -EIO;
		goto _cyttsp4_calc_ic_tch_crc_tma400_exit;
	}
	_cyttsp4_pr_buf(ts, pdata, ts->ebid_row_size, "ebid_data");
	crc = _cyttsp4_calc_partial_crc(ts, pdata, crc_ofs, crc);
	ndata += crc_ofs;
	dev_vdbg(ts->dev,
		"%s: ndata=%d\n", __func__, ndata);
	if (read_back_verify  && (ntable < tsize)) {
		dev_vdbg(ts->dev,
			"%s: crc_row=%d ntbl=%d tsz=%d crc_ofs=%d\n",
			__func__, crc_row, ntable, tsize, crc_ofs);
		for (i = 0; match && i < crc_ofs; i++) {
			if (ptable[ntable] != pdata[i]) {
				dev_vdbg(ts->dev,
					"%s: read back err crc_row=%d"
					" table[%d]=%02X"
					" pdata[%d]=%02X\n",
					__func__, crc_row,
					ntable, ptable[ntable],
					i, pdata[i]);
				match = false;
			}
			ntable++;
			if (ntable > tsize) {
				dev_err(ts->dev,
			"%s: crc_row=%d ntbl=%d tsz=%d\n",
					__func__, crc_row, ntable, tsize);
				break;
			}
		}
	}
_cyttsp4_calc_ic_tch_crc_tma400_exit:
	*crc_h = crc / 256;
	*crc_l = crc % 256;

	if (pdata != NULL)
		kfree(pdata);
	if (read_back_verify) {
		if (!match)
			retval = -EIO;
	}
	return retval;
}
#endif /* --CY_USE_TMA400 */
#endif /* --CONFIG_TOUCHSCREEN_DEBUG */

#ifdef CY_USE_TMA884
static int _cyttsp4_calc_settings_crc(struct cyttsp4 *ts, u8 *crc_h, u8 *crc_l)
{
	int retval = 0;
	u8 *buf = NULL;
	u8 size = 0;

	buf = kzalloc(sizeof(uint8_t) * 126, GFP_KERNEL);
	if (buf == NULL) {
		dev_err(ts->dev,
			"%s: Failed to allocate buf\n", __func__);
		retval = -ENOMEM;
		goto _cyttsp4_calc_settings_crc_exit;
	}

	if (ts->platform_data->sett[CY_IC_GRPNUM_TCH_PARM_VAL] == NULL) {
		dev_err(ts->dev,
			"%s: Missing Platform Touch Parameter"
			" values table\n",  __func__);
		retval = -ENXIO;
		goto _cyttsp4_calc_settings_crc_exit;
	}
	if ((ts->platform_data->sett
		[CY_IC_GRPNUM_TCH_PARM_VAL]->data == NULL) ||
		(ts->platform_data->sett
		[CY_IC_GRPNUM_TCH_PARM_VAL]->size == 0)) {
		dev_err(ts->dev,
			"%s: Missing Platform Touch Parameter"
			" values table data\n", __func__);
		retval = -ENXIO;
		goto _cyttsp4_calc_settings_crc_exit;
	}

	size = ts->platform_data->sett[CY_IC_GRPNUM_TCH_PARM_VAL]->size;

	if (size > 122) {
		dev_err(ts->dev,
			"%s: Platform data is too large\n", __func__);
		retval = -EOVERFLOW;
		goto _cyttsp4_calc_settings_crc_exit;
	}

	buf[0] = 0x00; /* num of config bytes + 4 high */
	buf[1] = 0x7E; /* num of config bytes + 4 low */
	buf[2] = 0x00; /* max block size w/o crc high */
	buf[3] = 0x7E; /* max block size w/o crc low */

	/* Copy platform data */
	memcpy(&(buf[4]),
		ts->platform_data->sett[CY_IC_GRPNUM_TCH_PARM_VAL]->data,
		size);

	/* Calculate CRC */
	_cyttsp4_calc_crc(ts, buf, 126, crc_h, crc_l);

_cyttsp4_calc_settings_crc_exit:
	kfree(buf);
	return retval;
}
#endif /* --CY_USE_TMA884 */

/* Get IC CRC is operational mode command */
static int _cyttsp4_get_ic_crc(struct cyttsp4 *ts,
	enum cyttsp4_ic_ebid ebid, u8 *crc_h, u8 *crc_l)
{
	int retval = 0;
	u8 cmd_dat[CY_NUM_DAT + 1];	/* +1 for cmd byte */

	memset(cmd_dat, 0, sizeof(cmd_dat));
	cmd_dat[0] = CY_GET_CFG_BLK_CRC;/* pack cmd */
	cmd_dat[1] = ebid;		/* pack EBID id */

	retval = _cyttsp4_put_cmd_wait(ts, ts->si_ofs.cmd_ofs,
		sizeof(cmd_dat), cmd_dat, CY_HALF_SEC_TMO_MS,
		_cyttsp4_chk_cmd_rdy, NULL,
		ts->platform_data->addr[CY_TCH_ADDR_OFS], true);
	if (retval < 0) {
		dev_err(ts->dev,
			"%s: Fail Get CRC command r=%d\n",
			__func__, retval);
		goto _cyttsp4_get_ic_crc_exit;
	}

	memset(cmd_dat, 0, sizeof(cmd_dat));
	retval = _cyttsp4_read_block_data(ts, ts->si_ofs.cmd_ofs,
		sizeof(cmd_dat), cmd_dat,
		ts->platform_data->addr[CY_TCH_ADDR_OFS], true);
	if (retval < 0) {
		dev_err(ts->dev,
			"%s: Fail Get CRC status r=%d\n",
			__func__, retval);
		goto _cyttsp4_get_ic_crc_exit;
	}

	/* Check CRC status and assign values */
	if (cmd_dat[1] != 0) {
		dev_err(ts->dev,
			"%s: Get CRC status=%d error\n",
			__func__, cmd_dat[1]);
		retval = -EIO;
		goto _cyttsp4_get_ic_crc_exit;
	}

	*crc_h = cmd_dat[2];
	*crc_l = cmd_dat[3];

#ifdef CY_USE_TMA400
	retval = _cyttsp4_cmd_handshake(ts);
	if (retval < 0) {
		dev_err(ts->dev,
			"%s: Command handshake error r=%d\n",
			__func__, retval);
		/* continue anyway; rely on handshake tmo */
		retval = 0;
	}
#endif /* --CY_USE_TMA400 */

_cyttsp4_get_ic_crc_exit:
	return retval;
}

/* Get Parameter is operational mode command */
static int _cyttsp4_get_parameter(struct cyttsp4 *ts,
	u8 id, u8 *data, u8 *size)
{
	int retval = 0;
	int i;
	u8 cmd_dat[CY_NUM_DAT + 1]; /* +1 for cmd byte */

	memset(cmd_dat, 0, sizeof(cmd_dat));
	cmd_dat[0] = CY_GET_PARAM_CMD;	 /* pack cmd */
	cmd_dat[1] = id;		 /* pack parameter id */

	retval = _cyttsp4_put_cmd_wait(ts, ts->si_ofs.cmd_ofs,
		sizeof(cmd_dat), cmd_dat, CY_HALF_SEC_TMO_MS,
		_cyttsp4_chk_cmd_rdy, NULL,
		ts->platform_data->addr[CY_TCH_ADDR_OFS], true);
	if (retval < 0) {
		dev_err(ts->dev,
			"%s: Fail Get Parameter command r=%d\n",
			__func__, retval);
		goto _cyttsp4_get_parameter_exit;
	}

	memset(cmd_dat, 0, sizeof(cmd_dat));
	retval = _cyttsp4_read_block_data(ts, ts->si_ofs.cmd_ofs,
		sizeof(cmd_dat), cmd_dat,
		ts->platform_data->addr[CY_TCH_ADDR_OFS], true);
	if (retval < 0) {
		dev_err(ts->dev,
			"%s: Fail Get Parameter status r=%d\n",
			__func__, retval);
		goto _cyttsp4_get_parameter_exit;
	}

	/* Check returned size */
	*size = cmd_dat[2];
	if (*size == 0) {
		dev_err(ts->dev,
			"%s: Get Parameter status=%d error\n",
			__func__, *size);
		retval = -EIO;
		goto _cyttsp4_get_parameter_exit;
	}

	for (i = 0; i < *size && i < 4; i++)
		data[i] = cmd_dat[3 + i];

#ifdef CY_USE_TMA400
	retval = _cyttsp4_cmd_handshake(ts);
	if (retval < 0) {
		dev_err(ts->dev,
			"%s: Command handshake error r=%d\n",
			__func__, retval);
		/* continue anyway; rely on handshake tmo */
		retval = 0;
	}
#endif /* --CY_USE_TMA400 */

_cyttsp4_get_parameter_exit:
	return retval;
}

/* Set Parameter is operational mode command */
static int _cyttsp4_set_parameter(struct cyttsp4 *ts,
	u8 id, u8 *data, u8 size)
{
	int retval = 0;
	u8 cmd_dat[CY_NUM_DAT + 1]; /* +1 for cmd byte */

	if (size > 4) {
		dev_err(ts->dev,
			"%s: Fail Set Parameter data size=%d\n",
			__func__, size);
		retval = -EINVAL;
		goto _cyttsp4_set_parameter_exit;
	}
	
	memset(cmd_dat, 0, sizeof(cmd_dat));
	cmd_dat[0] = CY_SET_PARAM_CMD;	 /* pack cmd */
	cmd_dat[1] = id;		 /* pack parameter id */
	cmd_dat[2] = size;		 /* pack size */
	memcpy(&cmd_dat[3], data, size); /* pack data */

	retval = _cyttsp4_put_cmd_wait(ts, ts->si_ofs.cmd_ofs,
		sizeof(cmd_dat), cmd_dat, CY_HALF_SEC_TMO_MS,
		_cyttsp4_chk_cmd_rdy, NULL,
		ts->platform_data->addr[CY_TCH_ADDR_OFS], true);
	if (retval < 0) {
		dev_err(ts->dev,
			"%s: Fail Set Parameter command r=%d\n",
			__func__, retval);
		goto _cyttsp4_set_parameter_exit;
	}

	memset(cmd_dat, 0, sizeof(cmd_dat));
	retval = _cyttsp4_read_block_data(ts, ts->si_ofs.cmd_ofs,
		sizeof(cmd_dat), cmd_dat,
		ts->platform_data->addr[CY_TCH_ADDR_OFS], true);
	if (retval < 0) {
		dev_err(ts->dev,
			"%s: Fail Set Parameter status r=%d\n",
			__func__, retval);
		goto _cyttsp4_set_parameter_exit;
	}

	/* Check returned size */
	if (cmd_dat[2] != size) {
		dev_err(ts->dev,
			"%s: Set Parameter status=%d error\n",
			__func__, cmd_dat[2]);
		retval = -EIO;
		goto _cyttsp4_set_parameter_exit;
	}

#ifdef CY_USE_TMA400
	retval = _cyttsp4_cmd_handshake(ts);
	if (retval < 0) {
		dev_err(ts->dev,
			"%s: Command handshake error r=%d\n",
			__func__, retval);
		/* continue anyway; rely on handshake tmo */
		retval = 0;
	}
#endif /* --CY_USE_TMA400 */

_cyttsp4_set_parameter_exit:
	return retval;
}

#define CY_CHARGER_STATUS 0x51
static int cyttsp4_get_charger_armor_status(struct cyttsp4 *ts, bool *enabled)
{
	int retval = 0;
	u8 size;
	u8 data[4];

	mutex_lock(&ts->data_lock);
	retval = _cyttsp4_get_parameter(ts, CY_CHARGER_STATUS, data, &size);
	if (retval < 0 || size != 1) {
		dev_err(ts->dev, "%s: Failed to get parameter\n", __func__);
		goto cyttsp4_get_charger_armor_status_exit;
	}

	*enabled = (data[0] & 0x01) ? true : false;

cyttsp4_get_charger_armor_status_exit:
	mutex_unlock(&ts->data_lock);
	return retval;
}

static int cyttsp4_set_charger_armor(struct cyttsp4 *ts, bool on)
{
	int retval = 0;
	const u8 size = 1;
	u8 data = on ? 0x01 : 0x00;

	mutex_lock(&ts->data_lock);
	retval = _cyttsp4_set_parameter(ts, CY_CHARGER_STATUS, &data, size);
	if (retval < 0) {
		dev_err(ts->dev, "%s: Failed to set parameter\n", __func__);
		goto cyttsp4_set_charger_armor_exit;
	}

cyttsp4_set_charger_armor_exit:
	mutex_unlock(&ts->data_lock);
	return retval;
}

#ifdef CY_USE_TMA400
static int _cyttsp4_startup(struct cyttsp4 *ts)
{
	int tries;
	int retval = 0;
	u8 ic_crc[2];
	u8 table_crc[2];
	bool put_all_params_done = false;
	bool upgraded = false;
	bool mddata_updated = false;
#ifdef SH_TPSIF_COMMAND
	u32 version;
#endif	/* SH_TPSIF_COMMAND */

	put_all_params_done = true;
	/* 
	 * FIXME
	 *
	 * Default value of "put_all_params_done" is false.  Now
	 * changed to true in order to check firmware update function.
	 * 
	 * Don't forget to return to false in software for production.
	 *
	 */

#ifdef CALIBRATION_AFTER_FIRMWARE_UPDATE
	ts->startup_fw_upgraded = false;
#endif	/* CALIBRATION_AFTER_FIRMWARE_UPDATE */

	tries = 0;
#ifdef CY_USE_WATCHDOG
	_cyttsp4_stop_wd_timer(ts);
#endif
_cyttsp4_startup_tma400_restart:

	dev_vdbg(ts->dev,
		"%s: enter driver_state=%d\n", __func__, ts->driver_state);
	ts->current_mode = CY_MODE_BOOTLOADER;
	retval = _cyttsp4_reset(ts);
	if (retval < 0) {
		dev_err(ts->dev,
			"%s: Fail reset device r=%d\n", __func__, retval);
		/* continue anyway in case device was already in bootloader */
	}
	/*
	 * Wait for heartbeat interrupt. If we didn't get the CPU quickly, this
	 * may not be the first interupt.
	 */
	dev_vdbg(ts->dev,
		"%s: wait for first bootloader interrupt\n", __func__);
	retval = _cyttsp4_wait_int(ts, CY_TEN_SEC_TMO_MS);
	if (retval < 0) {
		dev_err(ts->dev,
			"%s: Fail waiting for bootloader interrupt\n",
			__func__);
		goto _cyttsp4_startup_tma400_exit;
	}

	/*
	 * exit BL mode and eliminate race between heartbeat and
	 * command / response interrupts
	 */
	_cyttsp4_change_state(ts, CY_EXIT_BL_STATE);
	ts->switch_flag = true;
	retval = _cyttsp4_wait_si_int(ts, CY_TEN_SEC_TMO_MS);
	if (retval < 0) {
		dev_err(ts->dev,
			"%s: Fail wait switch to Sysinfo r=%d\n",
			__func__, retval);
		/* continue anyway in case sync missed */
	}
	if (ts->driver_state != CY_SYSINFO_STATE) {
		dev_err(ts->dev,
			"%s: Fail set sysinfo mode; switch to sysinfo anyway\r",
			__func__);
		_cyttsp4_change_state(ts, CY_SYSINFO_STATE);
	} else {
		dev_vdbg(ts->dev,
			"%s: Exit BL ok; now in sysinfo mode\n", __func__);
		_cyttsp4_pr_state(ts);
	}

#ifdef TMA443_COMPATIBILITY
	if (ts->tma443_compat) {
		dev_vdbg(ts->dev, "%s: msleep(5000)\n", __func__);
		msleep(5000);
	}
#endif

	dev_vdbg(ts->dev,
		"%s: Read Sysinfo regs and get rev numbers try=%d\n",
		__func__, tries);
	retval = _cyttsp4_get_sysinfo_regs(ts);
	if (retval < 0) {
		dev_err(ts->dev,
			"%s: Read Block fail -get sys regs (r=%d)\n",
			__func__, retval);
		dev_err(ts->dev,
			"%s: Fail to switch from Bootloader "
			"to Application r=%d\n",
			__func__, retval);

		_cyttsp4_change_state(ts, CY_BL_STATE);

		if (upgraded) {
			dev_err(ts->dev,
				"%s: app failed to launch after"
				" platform firmware upgrade\n", __func__);
			retval = -EIO;
			goto _cyttsp4_startup_tma400_exit;
		}

#ifndef CY_NO_AUTO_LOAD
#ifdef TMA443_COMPATIBILITY
		if (!ts->tma443_compat) {
#endif	/* TMA443_COMPATIBILITY */
			dev_info(ts->dev,
				"%s: attempting to reflash IC...\n", __func__);
			if (ts->platform_data->fw->img == NULL ||
				ts->platform_data->fw->size == 0) {
				dev_err(ts->dev,
					"%s: no platform firmware available"
					" for reflashing\n", __func__);
				_cyttsp4_change_state(ts, CY_INVALID_STATE);
				retval = -ENODATA;
				goto _cyttsp4_startup_tma400_exit;
			}
			retval = _cyttsp4_load_app(ts,
						ts->platform_data->fw->img,
						ts->platform_data->fw->size);
			if (retval) {
				dev_err(ts->dev,
					"%s: failed to reflash IC (r=%d)\n",
					__func__, retval);
				_cyttsp4_change_state(ts, CY_INVALID_STATE);
				retval = -EIO;
				goto _cyttsp4_startup_tma400_exit;
			}
#ifdef CALIBRATION_AFTER_FIRMWARE_UPDATE
			ts->startup_fw_upgraded = true;
#endif	/* CALIBRATION_AFTER_FIRMWARE_UPDATE */
			upgraded = true;
			dev_info(ts->dev,
				"%s: resetting IC after reflashing\n", __func__);
			goto _cyttsp4_startup_tma400_restart; /* Reset the part */
#ifdef TMA443_COMPATIBILITY
		}
#endif	/* TMA443_COMPATIBILITY */
#endif /* --CY_NO_AUTO_LOAD */
	}

#ifdef SH_TPSIF_COMMAND
	if (_sh_tpsif_firmware_version(ts, &version) < 0) {
		dev_err(ts->dev, "%s: Touch Panel Firmware Version = Unknown\n", __func__);
	} else {
		dev_info(ts->dev, "%s: Touch Panel Firmware Version = %02X.%02X.%02X%02X\n",
			__func__, ((u8 *)&version)[3], ((u8 *)&version)[2],
			((u8 *)&version)[1], ((u8 *)&version)[0]);
		if (((u8 *)&version)[3] == 0x00 && ((u8 *)&version)[2] == 0x24) {
			dev_info(ts->dev, "%s: Firmware for ES10.2\n", __func__);
			put_all_params_done = true;
			mddata_updated = true;
			if (((u8 *)&version)[1] == 0x13 && ((u8 *)&version)[0] == 0xEC) {
				dev_info(ts->dev, "%s: Inverted Y-axis\n", __func__);
				ts->flags |= CY_INV_Y;
			}
		} else {
			dev_info(ts->dev, "%s: Firmware for ES100\n", __func__);
		}
	}
#endif	/* SH_TPSIF_COMMAND */

#if 1
#ifndef CY_NO_AUTO_LOAD
#ifdef CONFIG_TOUCHSCREEN_DEBUG
	if (!ts->ic_grptest && !(ts->debug_upgrade)) {
#endif /* --CONFIG_TOUCHSCREEN_DEBUG */
		retval = _cyttsp4_boot_loader(ts, &upgraded);
		if (retval < 0) {
			dev_err(ts->dev,
				"%s: fail boot loader r=%d)\n",
				__func__, retval);
			_cyttsp4_change_state(ts, CY_IDLE_STATE);
			goto _cyttsp4_startup_tma400_exit;
		}
#ifdef CALIBRATION_AFTER_FIRMWARE_UPDATE
		if (upgraded)
			ts->startup_fw_upgraded = true;
#endif	/* CALIBRATION_AFTER_FIRMWARE_UPDATE */
		if (upgraded)
			goto _cyttsp4_startup_tma400_restart;
#ifdef CONFIG_TOUCHSCREEN_DEBUG
	}
#endif /* --CONFIG_TOUCHSCREEN_DEBUG */
#endif /* --CY_NO_AUTO_LOAD */
#endif

	retval = _cyttsp4_set_mode(ts, CY_CONFIG_MODE);
	if (retval < 0) {
		dev_err(ts->dev,
			"%s: Fail set config mode 1 r=%d\n", __func__, retval);
		goto _cyttsp4_startup_tma400_bypass_crc_check;
	}

	retval = _cyttsp4_get_ebid_row_size(ts);
	if (retval < 0) {
		dev_err(ts->dev,
			"%s: Fail get EBID row size; using default r=%d\n",
			__func__, retval);
	}
	dev_vdbg(ts->dev,
		"%s: get EBID row size=%d\n", __func__, ts->ebid_row_size);

#ifdef CONFIG_TOUCHSCREEN_DEBUG
	if (ts->ic_grptest)
		goto _cyttsp4_startup_tma400_bypass_crc_check;
#endif /* --CONFIG_TOUCHSCREEN_DEBUG */
	memset(ic_crc, 0, sizeof(ic_crc));
	dev_vdbg(ts->dev,
		"%s: Read IC CRC values\n", __func__);
	/* Get settings CRC from touch IC */
	retval = _cyttsp4_set_mode(ts, CY_OPERATE_MODE);
	if (retval < 0) {
		dev_err(ts->dev,
			"%s: Fail set operational mode 1 (r=%d)\n",
			__func__, retval);
		goto _cyttsp4_startup_tma400_exit;
	}
	retval = _cyttsp4_get_ic_crc(ts, CY_TCH_PARM_EBID,
				&ic_crc[0], &ic_crc[1]);
	if (retval < 0) {
		dev_err(ts->dev,
			"%s: Fail read ic crc r=%d\n",
			__func__, retval);
	}

	_cyttsp4_pr_buf(ts, ic_crc, sizeof(ic_crc), "read_ic_crc");

	retval = _cyttsp4_set_mode(ts, CY_CONFIG_MODE);
	if (retval < 0) {
		dev_err(ts->dev,
			"%s: Fail set config mode 2 r=%d\n", __func__, retval);
		goto _cyttsp4_startup_tma400_exit;
	}
	if (!put_all_params_done) {
		if (ts->platform_data->sett
			[CY_IC_GRPNUM_TCH_PARM_VAL]->data == NULL) {
			dev_err(ts->dev,
				"%s: missing param table\n", __func__);
			goto _cyttsp4_startup_tma400_bypass_crc_check;
		} else if (ts->platform_data->sett[CY_IC_GRPNUM_TCH_PARM_VAL]
			->data == NULL) {
			dev_err(ts->dev,
				"%s: missing param values table data\n",
				__func__);
			goto _cyttsp4_startup_tma400_bypass_crc_check;
		} else if (ts->platform_data->sett[CY_IC_GRPNUM_TCH_PARM_VAL]
			->size == 0) {
			dev_err(ts->dev,
				"%s: param values table size is 0\n", __func__);
			goto _cyttsp4_startup_tma400_bypass_crc_check;
		}
		_cyttsp_read_table_crc(ts, ts->platform_data->sett
				[CY_IC_GRPNUM_TCH_PARM_VAL]->data,
				&table_crc[0], &table_crc[1]);
		_cyttsp4_pr_buf(ts, table_crc, sizeof(table_crc),
				"read_table_crc");
		if ((ic_crc[0] != table_crc[0]) ||
			(ic_crc[1] != table_crc[1])) {
			retval = _cyttsp4_put_all_params_tma400(ts);
			if (retval < 0) {
				dev_err(ts->dev,
					"%s: Fail put all params r=%d\n",
					__func__, retval);
				goto _cyttsp4_startup_tma400_bypass_crc_check;
			}
			put_all_params_done = true;
			goto _cyttsp4_startup_tma400_restart;
		}
	}

_cyttsp4_startup_tma400_bypass_crc_check:
#ifdef STARTUP_MDDATA_CHECK
	if (!mddata_updated) {
		retval = _cyttsp4_check_mddata_tma400(ts, &mddata_updated);
		if (retval < 0) {
			dev_err(ts->dev,
				"%s: Fail update MDDATA r=%d\n",
				__func__, retval);
		} else if (mddata_updated)
			goto _cyttsp4_startup_tma400_restart;
	}
#endif	/* STARTUP_MDDATA_CHECK */

	dev_vdbg(ts->dev,
		"%s: enter operational mode\n", __func__);
	/* mode=operational mode, state = active_state */
	retval = _cyttsp4_set_mode(ts, CY_OPERATE_MODE);
	if (retval < 0) {
		dev_err(ts->dev,
			"%s: Fail set operational mode 2 (r=%d)\n",
			__func__, retval);
		goto _cyttsp4_startup_tma400_exit;
	}

	if (ts->was_suspended) {
		ts->was_suspended = false;
		retval = _cyttsp4_enter_sleep(ts);
		if (retval < 0) {
			dev_err(ts->dev,
				"%s: fail resume sleep r=%d\n",
				__func__, retval);
		}
	} else {
#ifdef CY_USE_WATCHDOG
		_cyttsp4_start_wd_timer(ts);
#endif
	}
_cyttsp4_startup_tma400_exit:
	return retval;
}
#endif /* --CY_USE_TMA400 */

#ifdef CY_USE_TMA884
#define CY_IRQ_DEASSERT	1
#define CY_IRQ_ASSERT	0
static int _cyttsp4_startup(struct cyttsp4 *ts)
{
	int retval = 0;
	int i = 0;
	u8 pdata_crc[2];
	u8 ic_crc[2];
	bool upgraded = false;
	bool mddata_updated = false;
	bool wrote_sysinfo_regs = false;
	bool wrote_settings = false;

#ifdef CY_USE_WATCHDOG
	_cyttsp4_stop_wd_timer(ts);
#endif
_cyttsp4_startup_start:
	memset(pdata_crc, 0, sizeof(pdata_crc));
	memset(ic_crc, 0, sizeof(ic_crc));
	dev_vdbg(ts->dev,
		"%s: enter driver_state=%d\n", __func__, ts->driver_state);
	_cyttsp4_change_state(ts, CY_BL_STATE);

	retval = _cyttsp4_reset(ts);
	if (retval < 0) {
		dev_err(ts->dev,
			"%s: Fail reset device r=%d\n", __func__, retval);
		/* continue anyway in case device was already in bootloader */
	}

	/* wait for interrupt to set ready completion */
	retval = _cyttsp4_wait_int(ts, CY_TEN_SEC_TMO_MS);
	if (retval < 0) {
		dev_err(ts->dev,
			"%s: Fail waiting for bootloader interrupt\n",
			__func__);
		goto _cyttsp4_startup_exit;
	}

	INIT_COMPLETION(ts->si_int_running);
	_cyttsp4_change_state(ts, CY_EXIT_BL_STATE);
	ts->switch_flag = true;
	retval = _cyttsp4_wait_si_int(ts, CY_TEN_SEC_TMO_MS);
	if (retval < 0) {
		dev_err(ts->dev,
			"%s: Fail wait switch to Sysinfo r=%d\n",
			__func__, retval);
		/* continue anyway in case sync missed */
	}
	if (ts->driver_state != CY_SYSINFO_STATE)
		_cyttsp4_change_state(ts, CY_SYSINFO_STATE);
	else
		_cyttsp4_pr_state(ts);

	/*
	 * TODO: remove this wait for toggle high when
	 * startup from ES10 firmware is no longer required
	 */
	/* Wait for IRQ to toggle high */
	dev_vdbg(ts->dev,
		"%s: wait for irq toggle high\n", __func__);
	retval = -ETIMEDOUT;
	for (i = 0; i < CY_DELAY_MAX * 10 * 5; i++) {
		if (ts->platform_data->irq_stat() == CY_IRQ_DEASSERT) {
			retval = 0;
			break;
		}
		mdelay(CY_DELAY_DFLT);
	}
	if (retval < 0) {
		dev_err(ts->dev,
			"%s: timeout waiting for irq to de-assert\n",
			__func__);
		goto _cyttsp4_startup_exit;
	}

	dev_vdbg(ts->dev,
		"%s: read sysinfo 1\n", __func__);
	memset(&ts->sysinfo_data, 0,
		sizeof(struct cyttsp4_sysinfo_data));
	retval = _cyttsp4_read_block_data(ts, CY_REG_BASE,
		sizeof(struct cyttsp4_sysinfo_data), &ts->sysinfo_data,
		ts->platform_data->addr[CY_TCH_ADDR_OFS], true);
	if (retval < 0) {
		dev_err(ts->dev,
			"%s: Fail to switch from Bootloader "
			"to Application r=%d\n",
			__func__, retval);

		_cyttsp4_change_state(ts, CY_BL_STATE);

		if (upgraded) {
			dev_err(ts->dev,
			"%s: app failed to launch after"
				" platform firmware upgrade\n", __func__);
			retval = -EIO;
			goto _cyttsp4_startup_exit;
		}

		dev_info(ts->dev,
			"%s: attempting to reflash IC...\n", __func__);
		if (ts->platform_data->fw->img == NULL ||
			ts->platform_data->fw->size == 0) {
			dev_err(ts->dev,
			"%s: no platform firmware available"
				" for reflashing\n", __func__);
			_cyttsp4_change_state(ts, CY_INVALID_STATE);
			retval = -ENODATA;
			goto _cyttsp4_startup_exit;
		}
		retval = _cyttsp4_load_app(ts,
			ts->platform_data->fw->img,
			ts->platform_data->fw->size);
		if (retval) {
			dev_err(ts->dev,
			"%s: failed to reflash IC (r=%d)\n",
				__func__, retval);
			_cyttsp4_change_state(ts, CY_INVALID_STATE);
			retval = -EIO;
			goto _cyttsp4_startup_exit;
		}
		upgraded = true;
		dev_info(ts->dev,
			"%s: resetting IC after reflashing\n", __func__);
		goto _cyttsp4_startup_start; /* Reset the part */
	}

	/*
	 * read system information registers
	 * get version numbers and fill sysinfo regs
	 */
	dev_vdbg(ts->dev,
		"%s: Read Sysinfo regs and get version numbers\n", __func__);
	retval = _cyttsp4_get_sysinfo_regs(ts);
	if (retval < 0) {
		dev_err(ts->dev,
			"%s: Read Block fail -get sys regs (r=%d)\n",
			__func__, retval);
		_cyttsp4_change_state(ts, CY_IDLE_STATE);
		goto _cyttsp4_startup_exit;
	}

#ifndef CY_NO_AUTO_LOAD
#ifdef CONFIG_TOUCHSCREEN_DEBUG
	if (!ts->ic_grptest && !(ts->debug_upgrade)) {
#endif /* --CONFIG_TOUCHSCREEN_DEBUG */
		retval = _cyttsp4_boot_loader(ts, &upgraded);
		if (retval < 0) {
			dev_err(ts->dev,
			"%s: fail boot loader r=%d)\n",
				__func__, retval);
			_cyttsp4_change_state(ts, CY_IDLE_STATE);
			goto _cyttsp4_startup_exit;
		}
		if (upgraded)
			goto _cyttsp4_startup_start;
#ifdef CONFIG_TOUCHSCREEN_DEBUG
	}
#endif /* --CONFIG_TOUCHSCREEN_DEBUG */
#endif /* --CY_NO_AUTO_LOAD */

	if (!wrote_sysinfo_regs) {
#ifdef CONFIG_TOUCHSCREEN_DEBUG
		if (ts->ic_grptest)
			goto _cyttsp4_startup_set_sysinfo_done;
#endif /* --CONFIG_TOUCHSCREEN_DEBUG */
		dev_vdbg(ts->dev,
			"%s: Set Sysinfo regs\n", __func__);
		retval = _cyttsp4_set_mode(ts, CY_SYSINFO_MODE);
		if (retval < 0) {
			dev_err(ts->dev,
			"%s: Set SysInfo Mode fail r=%d\n",
				__func__, retval);
			_cyttsp4_change_state(ts, CY_IDLE_STATE);
			goto _cyttsp4_startup_exit;
		}
		retval = _cyttsp4_set_sysinfo_regs(ts, &mddata_updated);
		if (retval < 0) {
			dev_err(ts->dev,
			"%s: Set SysInfo Regs fail r=%d\n",
				__func__, retval);
			_cyttsp4_change_state(ts, CY_IDLE_STATE);
			goto _cyttsp4_startup_exit;
		} else
			wrote_sysinfo_regs = true;
	}

#ifdef CONFIG_TOUCHSCREEN_DEBUG
_cyttsp4_startup_set_sysinfo_done:
#endif /* --CONFIG_TOUCHSCREEN_DEBUG */
	dev_vdbg(ts->dev,
		"%s: enter operational mode\n", __func__);
	retval = _cyttsp4_set_mode(ts, CY_OPERATE_MODE);
	if (retval < 0) {
		_cyttsp4_change_state(ts, CY_IDLE_STATE);
		dev_err(ts->dev,
			"%s: Fail set operational mode (r=%d)\n",
			__func__, retval);
		goto _cyttsp4_startup_exit;
	} else {
#ifdef CONFIG_TOUCHSCREEN_DEBUG
		if (ts->ic_grptest)
			goto _cyttsp4_startup_settings_valid;
#endif /* --CONFIG_TOUCHSCREEN_DEBUG */
		/* Calculate settings CRC from platform settings */
		dev_vdbg(ts->dev,
			"%s: Calculate settings CRC and get IC CRC\n",
			__func__);
		retval = _cyttsp4_calc_settings_crc(ts,
			&pdata_crc[0], &pdata_crc[1]);
		if (retval < 0) {
			dev_err(ts->dev,
			"%s: Unable to calculate settings CRC\n",
				__func__);
			goto _cyttsp4_startup_exit;
		}

		/* Get settings CRC from touch IC */
		retval = _cyttsp4_get_ic_crc(ts, CY_TCH_PARM_EBID,
			&ic_crc[0], &ic_crc[1]);
		if (retval < 0) {
			dev_err(ts->dev,
			"%s: Unable to get settings CRC\n", __func__);
			goto _cyttsp4_startup_exit;
		}

		/* Compare CRC values */
		dev_vdbg(ts->dev,
			"%s: PDATA CRC = 0x%02X%02X, IC CRC = 0x%02X%02X\n",
			__func__, pdata_crc[0], pdata_crc[1],
			ic_crc[0], ic_crc[1]);

		if ((pdata_crc[0] == ic_crc[0]) &&
			(pdata_crc[1] == ic_crc[1]))
			goto _cyttsp4_startup_settings_valid;

		/* Update settings */
		dev_info(ts->dev,
			"%s: Updating IC settings...\n", __func__);

		if (wrote_settings) {
			dev_err(ts->dev,
			"%s: Already updated IC settings\n",
				__func__);
			goto _cyttsp4_startup_settings_valid;
		}

		retval = _cyttsp4_set_op_params(ts, pdata_crc[0], pdata_crc[1]);
		if (retval < 0) {
			dev_err(ts->dev,
			"%s: Set Operational Params fail r=%d\n",
				__func__, retval);
			goto _cyttsp4_startup_exit;
		}

		wrote_settings = true;
	}

_cyttsp4_startup_settings_valid:
	if (mddata_updated || wrote_settings) {
		dev_info(ts->dev,
			"%s: Resetting IC after writing settings\n",
			__func__);
		mddata_updated = false;
		wrote_settings = false;
		goto _cyttsp4_startup_start; /* Reset the part */
	}
	dev_vdbg(ts->dev,
		"%s: enable handshake\n", __func__);
	retval = _cyttsp4_handshake_enable(ts);
	if (retval < 0)
		dev_err(ts->dev,
			"%s: fail enable handshake r=%d", __func__, retval);

	_cyttsp4_change_state(ts, CY_ACTIVE_STATE);

	if (ts->was_suspended) {
		ts->was_suspended = false;
		retval = _cyttsp4_enter_sleep(ts);
		if (retval < 0) {
			dev_err(ts->dev,
			"%s: fail resume sleep r=%d\n",
				__func__, retval);
		}
	} else {
#ifdef CY_USE_WATCHDOG
		_cyttsp4_start_wd_timer(ts);
#endif
	}

_cyttsp4_startup_exit:
	return retval;
}
#endif /* --CY_USE_TMA884 */

static irqreturn_t cyttsp4_irq(int irq, void *handle)
{
	struct cyttsp4 *ts = handle;
	u8 rep_stat = 0;
	int retval = 0;

	dev_vdbg(ts->dev,
		"%s: GOT IRQ ps=%d\n", __func__, ts->driver_state);
	mutex_lock(&ts->data_lock);

	dev_vdbg(ts->dev,
		"%s: DO IRQ ps=%d\n", __func__, ts->driver_state);

	switch (ts->driver_state) {
	case CY_BL_STATE:
	case CY_CMD_STATE:
		complete(&ts->int_running);
#ifdef CY_USE_LEVEL_IRQ
		udelay(1000);
#endif
		break;
	case CY_SYSINFO_STATE:
		complete(&ts->si_int_running);
#ifdef CY_USE_LEVEL_IRQ
		udelay(500);
#endif
		break;
	case CY_EXIT_BL_STATE:
#ifdef CY_USE_LEVEL_IRQ
		udelay(1000);
#endif
		if (ts->switch_flag == true) {
			ts->switch_flag = false;
			retval = _cyttsp4_ldr_exit(ts);
			if (retval < 0) {
				dev_err(ts->dev,
			"%s: Fail bl exit r=%d\n",
					__func__, retval);
			} else
				ts->driver_state = CY_SYSINFO_STATE;
		}
		break;
	case CY_SLEEP_STATE:
		dev_vdbg(ts->dev,
			"%s: Attempt to process touch after enter sleep or"
			" unexpected wake event\n", __func__);
		retval = _cyttsp4_wakeup(ts); /* in case its really asleep */
		if (retval < 0) {
			dev_err(ts->dev,
			"%s: wakeup fail r=%d\n",
				__func__, retval);
			_cyttsp4_pr_state(ts);
			_cyttsp4_queue_startup(ts, true);
			break;
		}
		/* Put the part back to sleep */
		retval = _cyttsp4_enter_sleep(ts);
		if (retval < 0) {
			dev_err(ts->dev,
			"%s: fail resume sleep r=%d\n",
				__func__, retval);
			_cyttsp4_pr_state(ts);
			_cyttsp4_queue_startup(ts, true);
		}
		break;
	case CY_IDLE_STATE:
		if (ts->xy_mode == NULL) {
			/* initialization is not complete; invalid pointers */
			break;
		}

		/* device now available; signal initialization */
		dev_info(ts->dev,
			"%s: Received IRQ in IDLE state\n",
			__func__);
		/* Try to determine the IC's current state */
		retval = _cyttsp4_load_status_regs(ts);
		if (retval < 0) {
			dev_err(ts->dev,
			"%s: Still unable to access IC after IRQ r=%d\n",
				__func__, retval);
			break;
		}
		rep_stat = ts->xy_mode[ts->si_ofs.rep_ofs + 1];
		if (IS_BOOTLOADERMODE(rep_stat)) {
			dev_info(ts->dev,
			"%s: BL mode found in IDLE state\n",
				__func__);
			_cyttsp4_queue_startup(ts, false);
			break;
		}
		dev_err(ts->dev,
			"%s: interrupt received in IDLE state -"
			" try processing touch\n",
			__func__);
		_cyttsp4_change_state(ts, CY_ACTIVE_STATE);
#ifdef CY_USE_WATCHDOG
		_cyttsp4_start_wd_timer(ts);
#endif
		retval = _cyttsp4_xy_worker(ts);
		if (retval < 0) {
			dev_err(ts->dev,
			"%s: xy_worker IDLE fail r=%d\n",
				__func__, retval);
			_cyttsp4_queue_startup(ts, false);
			break;
		}

#ifdef CY_USE_LEVEL_IRQ
		udelay(500);
#endif
		break;
	case CY_READY_STATE:
		complete(&ts->ready_int_running);
		/* do not break; do worker */
	case CY_ACTIVE_STATE:
		/* process the touches */
		retval = _cyttsp4_xy_worker(ts);
		if (retval < 0) {
			dev_err(ts->dev,
			"%s: XY Worker fail r=%d\n",
				__func__, retval);
			_cyttsp4_queue_startup(ts, false);
		}
		break;
	default:
		break;
	}

	mutex_unlock(&ts->data_lock);
	dev_vdbg(ts->dev,
		"%s: DONE IRQ ps=%d\n", __func__, ts->driver_state);

	return IRQ_HANDLED;
}

static void _cyttsp4_file_init(struct cyttsp4 *ts)
{
#ifdef CONFIG_TOUCHSCREEN_DEBUG
	if (device_create_file(ts->dev, &dev_attr_drv_debug))
		dev_err(ts->dev,
			"%s: Error, could not create drv_debug\n", __func__);

	if (device_create_file(ts->dev, &dev_attr_drv_flags))
		dev_err(ts->dev,
			"%s: Error, could not create drv_flags\n", __func__);

	if (device_create_file(ts->dev, &dev_attr_drv_irq))
		dev_err(ts->dev,
			"%s: Error, could not create drv_irq\n", __func__);
#endif /* --CONFIG_TOUCHSCREEN_DEBUG */

	if (device_create_file(ts->dev, &dev_attr_drv_stat))
		dev_err(ts->dev,
			"%s: Error, could not create drv_stat\n", __func__);

	if (device_create_file(ts->dev, &dev_attr_drv_ver))
		dev_err(ts->dev,
			"%s: Error, could not create drv_ver\n", __func__);

#if defined(CY_USE_FORCE_LOAD) || defined(CONFIG_TOUCHSCREEN_DEBUG) || defined(SH_TPSIF_COMMAND)
	if (device_create_file(ts->dev, &dev_attr_ic_reflash))
		dev_err(ts->dev,
			"%s: Error, could not create ic_reflash\n", __func__);
#endif

#ifdef CONFIG_TOUCHSCREEN_DEBUG
	if (device_create_file(ts->dev, &dev_attr_hw_irqstat))
		dev_err(ts->dev,
			"%s: Error, could not create hw_irqstat\n", __func__);

	if (device_create_file(ts->dev, &dev_attr_hw_reset))
		dev_err(ts->dev,
			"%s: Error, could not create hw_reset\n", __func__);

	if (device_create_file(ts->dev, &dev_attr_hw_recov))
		dev_err(ts->dev,
			"%s: Error, could not create hw_recov\n", __func__);

	if (device_create_file(ts->dev, &dev_attr_hw_power))
		dev_err(ts->dev,
			"%s: Error, could not create hw_power\n", __func__);

	if (device_create_file(ts->dev, &dev_attr_ic_grpdata))
		dev_err(ts->dev,
			"%s: Error, could not create ic_grpdata\n", __func__);

	if (device_create_file(ts->dev, &dev_attr_ic_grpnum))
		dev_err(ts->dev,
			"%s: Error, could not create ic_grpnum\n", __func__);

	if (device_create_file(ts->dev, &dev_attr_ic_grpoffset))
		dev_err(ts->dev,
			"%s: Error, could not create ic_grpoffset\n", __func__);

	if (device_create_file(ts->dev, &dev_attr_ic_charmor))
		dev_err(ts->dev,
			"%s: Error, could not create ic_charmor\n", __func__);
#endif /* --CONFIG_TOUCHSCREEN_DEBUG */

	if (device_create_file(ts->dev, &dev_attr_ic_ver))
		dev_err(ts->dev,
			"%s: Cannot create ic_ver\n", __func__);

#ifdef CY_USE_REG_ACCESS
	if (device_create_file(ts->dev, &dev_attr_drv_rw_regid))
		dev_err(ts->dev,
			"%s: Cannot create drv_rw_regid\n", __func__);

	if (device_create_file(ts->dev, &dev_attr_drv_rw_reg_data))
		dev_err(ts->dev,
			"%s: Cannot create drv_rw_reg_data\n", __func__);
#endif

	return;
}

static void _cyttsp4_file_free(struct cyttsp4 *ts)
{
	device_remove_file(ts->dev, &dev_attr_drv_ver);
	device_remove_file(ts->dev, &dev_attr_drv_stat);
	device_remove_file(ts->dev, &dev_attr_ic_ver);
#if defined(CY_USE_FORCE_LOAD) || defined(CONFIG_TOUCHSCREEN_DEBUG) || defined(SH_TPSIF_COMMAND)
	device_remove_file(ts->dev, &dev_attr_ic_reflash);
#endif
#ifdef CONFIG_TOUCHSCREEN_DEBUG
	device_remove_file(ts->dev, &dev_attr_ic_grpnum);
	device_remove_file(ts->dev, &dev_attr_ic_grpoffset);
	device_remove_file(ts->dev, &dev_attr_ic_grpdata);
	device_remove_file(ts->dev, &dev_attr_hw_irqstat);
	device_remove_file(ts->dev, &dev_attr_drv_irq);
	device_remove_file(ts->dev, &dev_attr_drv_debug);
	device_remove_file(ts->dev, &dev_attr_drv_flags);
	device_remove_file(ts->dev, &dev_attr_hw_reset);
	device_remove_file(ts->dev, &dev_attr_hw_recov);
	device_remove_file(ts->dev, &dev_attr_hw_power);
	device_remove_file(ts->dev, &dev_attr_ic_charmor);
#endif /* --CONFIG_TOUCHSCREEN_DEBUG */
#ifdef CY_USE_REG_ACCESS
	device_remove_file(ts->dev, &dev_attr_drv_rw_regid);
	device_remove_file(ts->dev, &dev_attr_drv_rw_reg_data);
#endif
}

static int cyttsp4_open(struct input_dev *dev)
{
	int retval = 0;

	struct cyttsp4 *ts = input_get_drvdata(dev);
	dev_dbg(ts->dev, "%s: Open call ts=%p\n", __func__, ts);
	mutex_lock(&ts->data_lock);
	if (!ts->powered) {

		/* Power on */
		if (ts->platform_data->hw_power == NULL) {
			pr_err("%s: no hw_power function\n", __func__);
		} else {
			retval = ts->platform_data->hw_power(1); // power on
			if(retval < 0) {
				pr_err("%s: failed power on\n", __func__);
			}
		}

		/*
		 * execute complete startup procedure.  After this
		 * call the device is in active state and the worker
		 * is running
		 */
		retval = _cyttsp4_startup(ts);

		/* powered if no hard failure */
		if (retval < 0) {
			ts->powered = false;
			_cyttsp4_change_state(ts, CY_IDLE_STATE);
			dev_err(ts->dev,
				"%s: startup fail at power on r=%d\n",
				__func__, retval);
		} else
			ts->powered = true;

		dev_info(ts->dev,
			"%s: Powered ON(%d) r=%d\n",
			__func__, (int)ts->powered, retval);

#ifdef SH_TPSIF_COMMAND
#ifdef STARTUP_CALIBRATION
		if (retval == 0) {
			if (_sh_tpsif_calibration_idac(ts) < 0)
				dev_err(ts->dev, "%s: fail calibration\n", __func__);
#ifdef RESET_AFTER_CALIBRATION
			_cyttsp4_startup(ts);
#endif	/* RESET_AFTER_CALIBRATION */
		}
#else  /* STARTUP_CALIBRATION */
#ifdef CALIBRATION_AFTER_FIRMWARE_UPDATE
		if (retval == 0 && ts->startup_fw_upgraded) {
			dev_info(ts->dev, "%s: calibration after firmware update\n", __func__);
			if (_sh_tpsif_calibration_idac(ts) < 0)
				dev_err(ts->dev, "%s: fail calibration\n", __func__);
#ifdef RESET_AFTER_CALIBRATION
			_cyttsp4_startup(ts);
#endif	/* RESET_AFTER_CALIBRATION */
		}
#endif	/* CALIBRATION_AFTER_FIRMWARE_UPDATE */
#endif	/* STARTUP_CALIBRATION */
#endif	/* SH_TPSIF_COMMAND */
	}
	mutex_unlock(&ts->data_lock);
	return 0;
}

static void cyttsp4_close(struct input_dev *dev)
{
	/*
	 * close() normally powers down the device
	 * this call simply returns unless power
	 * to the device can be controlled by the driver
	 */
	return;
}

void cyttsp4_core_release(void *handle)
{
	struct cyttsp4 *ts = handle;

	dev_dbg(ts->dev, "%s: Release call ts=%p\n",
		__func__, ts);
	if (ts == NULL) {
		dev_err(ts->dev,
			"%s: Null context pointer on driver release\n",
			__func__);
		goto cyttsp4_core_release_exit;
	}

#ifdef CONFIG_HAS_EARLYSUSPEND
	unregister_early_suspend(&ts->early_suspend);
#endif
	_cyttsp4_file_free(ts);
	if (mutex_is_locked(&ts->data_lock))
		mutex_unlock(&ts->data_lock);
	mutex_destroy(&ts->data_lock);
	free_irq(ts->irq, ts);
	if (ts->input->mt)
		input_mt_destroy_slots(ts->input);
	input_unregister_device(ts->input);
	if (ts->cyttsp4_wq) {
		destroy_workqueue(ts->cyttsp4_wq);
		ts->cyttsp4_wq = NULL;
	}

	if (ts->sysinfo_ptr.cydata != NULL)
		kfree(ts->sysinfo_ptr.cydata);
	if (ts->sysinfo_ptr.test != NULL)
		kfree(ts->sysinfo_ptr.test);
	if (ts->sysinfo_ptr.pcfg != NULL)
		kfree(ts->sysinfo_ptr.pcfg);
	if (ts->sysinfo_ptr.opcfg != NULL)
		kfree(ts->sysinfo_ptr.opcfg);
	if (ts->sysinfo_ptr.ddata != NULL)
		kfree(ts->sysinfo_ptr.ddata);
	if (ts->sysinfo_ptr.mdata != NULL)
		kfree(ts->sysinfo_ptr.mdata);
	if (ts->xy_mode != NULL)
		kfree(ts->xy_mode);
	if (ts->xy_data != NULL)
		kfree(ts->xy_data);
	if (ts->xy_data_touch1 != NULL)
		kfree(ts->xy_data_touch1);

	/* Power off */
	if (ts->platform_data->hw_power == NULL) {
		pr_err("%s: no hw_power function\n", __func__);
	} else {
		if (ts->platform_data->hw_power(0) < 0) // power off
			pr_err("%s: failed power off\n", __func__);
	}

	kfree(ts);
cyttsp4_core_release_exit:
	return;
}
EXPORT_SYMBOL_GPL(cyttsp4_core_release);

void *cyttsp4_core_init(struct cyttsp4_bus_ops *bus_ops,
	struct device *dev, int irq, char *name)
{
	unsigned long irq_flags = 0;
	int i = 0;
	int min = 0;
	int max = 0;
	u16 signal = 0;
	int retval = 0;
	struct input_dev *input_device = NULL;
	struct cyttsp4 *ts = NULL;
#ifdef TMA443_COMPATIBILITY
	sharp_smem_common_type *sharp_smem;
	unsigned long rev;
#endif	/* TMA443_COMPATIBILITY */

	if (dev == NULL) {
		pr_err("%s: Error, dev pointer is Null\n", __func__);
		goto error_alloc_data;
	}

	if (bus_ops == NULL) {
		pr_err("%s: Error, bus_ops Pointer is Null\n", __func__);
		goto error_alloc_data;
	}
	ts = kzalloc(sizeof(*ts), GFP_KERNEL);
	if (ts == NULL) {
		pr_err("%s: Error, kzalloc context memory\n", __func__);
		goto error_alloc_data;
	}

#ifdef SH_TPSIF_COMMAND
	g_tpsif_ts = ts;
	init_waitqueue_head(&sh_tpsif_wq);
	sh_tpsif_event = 0;
	hrtimer_init(&sh_tpsif_polling_timer, CLOCK_MONOTONIC, HRTIMER_MODE_REL);
	sh_tpsif_polling_timer.function = _sh_tpsif_poll_timer_handler;
	INIT_WORK(&sh_tpsif_polling_work, _sh_tpsif_poll_scan);
#endif	/* SH_TPSIF_COMMAND */

#if defined(CY_USE_FORCE_LOAD) || defined(CONFIG_TOUCHSCREEN_DEBUG) || defined(SH_TPSIF_COMMAND)
	ts->fwname = kzalloc(CY_BL_FW_NAME_SIZE, GFP_KERNEL);
	if (ts->fwname == NULL) {
		pr_err("%s: Error, kzalloc fwname\n", __func__);
		goto error_alloc_failed;
	}
#endif
#ifdef CONFIG_TOUCHSCREEN_DEBUG
	ts->pr_buf = kzalloc(CY_MAX_PRBUF_SIZE, GFP_KERNEL);
	if (ts->pr_buf == NULL) {
		pr_err("%s: Error, kzalloc pr_buf\n", __func__);
		goto error_alloc_failed;
	}
#endif /* --CONFIG_TOUCHSCREEN_DEBUG */

#ifdef TMA443_COMPATIBILITY
	sharp_smem = sh_smem_get_common_address();
	rev = sharp_smem->sh_hw_revision;
	if (rev == 0x00000000 /*  SH_HW_REV_ES0*/ ||
		rev == 0x00000005 /* SH_HW_REV_ES1 */) {
		ts->tma443_compat = true;
	} else {
		ts->tma443_compat = false;
	}
#endif	/* TMA443_COMPATIBILITY */

	ts->cyttsp4_wq =
		create_singlethread_workqueue("cyttsp4_resume_startup_wq");
	if (ts->cyttsp4_wq == NULL) {
		pr_err("%s: No memory for cyttsp4_resume_startup_wq\n",
			__func__);
		goto error_alloc_failed;
	}

	ts->driver_state = CY_INVALID_STATE;
	ts->current_mode = CY_MODE_BOOTLOADER;
	ts->powered = false;
	ts->was_suspended = false;
	ts->switch_flag = false;
	ts->soft_reset_asserted = false;

	ts->xy_data = NULL;
	ts->xy_mode = NULL;
	ts->xy_data_touch1 = NULL;

#ifdef SH_TPSIF_COMMAND
	ts->adjust_enabled = false;
#endif	/* SH_TPSIF_COMMAND */
#ifdef CALIBRATION_AFTER_FIRMWARE_UPDATE
	ts->startup_fw_upgraded = false;
#endif	/* CALIBRATION_AFTER_FIRMWARE_UPDATE */

	ts->dev = dev;
	ts->bus_ops = bus_ops;
	ts->platform_data = dev->platform_data;
	if (ts->platform_data == NULL) {
		dev_err(ts->dev,
			"%s: Error, platform data is Null\n", __func__);
		goto error_alloc_failed;
	}

	if (ts->platform_data->frmwrk == NULL) {
		dev_err(ts->dev,
			"%s: Error, platform data framework is Null\n",
			__func__);
		goto error_alloc_failed;
	}

	if (ts->platform_data->frmwrk->abs == NULL) {
		dev_err(ts->dev,
			"%s: Error, platform data framework array is Null\n",
			__func__);
		goto error_alloc_failed;
	}

	mutex_init(&ts->data_lock);
	init_completion(&ts->int_running);
	init_completion(&ts->si_int_running);
	init_completion(&ts->ready_int_running);
	ts->flags = ts->platform_data->flags;
#if defined(CY_USE_FORCE_LOAD) || defined(CONFIG_TOUCHSCREEN_DEBUG) || defined(SH_TPSIF_COMMAND)
	ts->waiting_for_fw = false;
#endif
#ifdef CONFIG_TOUCHSCREEN_DEBUG
	ts->debug_upgrade = false;
	ts->ic_grpnum = CY_IC_GRPNUM_RESERVED;
	ts->ic_grpoffset = 0;
	ts->ic_grptest = false;
	ts->bus_ops->tsdebug = CY_DBG_LVL_0;
#endif /* --CONFIG_TOUCHSCREEN_DEBUG */

#ifdef CY_USE_TMA400
	ts->max_config_bytes = CY_TMA400_MAX_BYTES;
#endif /* --CY_USE_TMA400 */
#ifdef CY_USE_TMA884
	ts->max_config_bytes = CY_TMA884_MAX_BYTES;
#endif /* --CY_USE_TMA884 */

	ts->irq = irq;
	if (ts->irq <= 0) {
		dev_vdbg(ts->dev,
			"%s: Error, failed to allocate irq\n", __func__);
			goto error_init;
	}

	/* Create the input device and register it. */
	dev_vdbg(ts->dev,
		"%s: Create the input device and register it\n", __func__);
	input_device = input_allocate_device();
	if (input_device == NULL) {
		dev_err(ts->dev,
			"%s: Error, failed to allocate input device\n",
			__func__);
		goto error_init;
	}

	ts->input = input_device;
	input_device->name = name;
	snprintf(ts->phys, sizeof(ts->phys)-1, "%s", dev_name(dev));
	input_device->phys = ts->phys;
	input_device->dev.parent = ts->dev;
	ts->bus_type = bus_ops->dev->bus;
#ifdef CY_USE_WATCHDOG
	INIT_WORK(&ts->work, cyttsp4_timer_watchdog);
	setup_timer(&ts->timer, cyttsp4_timer, (unsigned long)ts);
#endif

	input_device->open = cyttsp4_open;
	input_device->close = cyttsp4_close;
	input_set_drvdata(input_device, ts);
	dev_set_drvdata(dev, ts);

	dev_vdbg(ts->dev,
		"%s: Initialize event signals\n", __func__);
	__set_bit(EV_ABS, input_device->evbit);

	input_mt_init_slots(input_device, CY_NUM_TCH_ID);
	if (input_device->mt == NULL) {
		pr_err("%s: Error, failed to init input slots\n",
			__func__);
		goto error_input_register_device;
	}

	for (i = 0; i < (ts->platform_data->frmwrk->size/CY_NUM_ABS_SET); i++) {
		signal = ts->platform_data->frmwrk->abs[
			(i*CY_NUM_ABS_SET)+CY_SIGNAL_OST];
		if (signal != CY_IGNORE_VALUE
			&& signal != ABS_MT_TRACKING_ID
			) {
			min = ts->platform_data->frmwrk->abs
				[(i*CY_NUM_ABS_SET)+CY_MIN_OST];
			max = ts->platform_data->frmwrk->abs
				[(i*CY_NUM_ABS_SET)+CY_MAX_OST];
			if (i == CY_ABS_ID_OST) {
				/* shift track ids down to start at 0 */
				max = max - min;
				min = min - min;
			}
			input_set_abs_params(input_device,
				signal,
				min,
				max,
				ts->platform_data->frmwrk->abs[
					(i*CY_NUM_ABS_SET)+CY_FUZZ_OST],
				ts->platform_data->frmwrk->abs[
					(i*CY_NUM_ABS_SET)+CY_FLAT_OST]);
		}
	}

	input_set_events_per_packet(input_device, 6 * CY_NUM_TCH_ID);

	dev_vdbg(ts->dev,
		"%s: Initialize irq\n", __func__);
#ifdef CY_USE_LEVEL_IRQ
	irq_flags = IRQF_TRIGGER_LOW | IRQF_ONESHOT;
#else
	irq_flags = IRQF_TRIGGER_FALLING | IRQF_ONESHOT;
#endif
	retval = request_threaded_irq(ts->irq, NULL, cyttsp4_irq,
		irq_flags, ts->input->name, ts);
	if (retval < 0) {
		dev_err(ts->dev,
			"%s: failed to init irq r=%d name=%s\n",
			__func__, retval, ts->input->name);
		ts->irq_enabled = false;
		goto error_init;
	} else {
		ts->irq_enabled = true;
	}

	retval = input_register_device(input_device);
	if (retval < 0) {
		dev_err(ts->dev,
			"%s: Error, failed to register input device r=%d\n",
			__func__, retval);
		goto error_init;
	}

	/* add /sys files */
	_cyttsp4_file_init(ts);

#ifdef CONFIG_HAS_EARLYSUSPEND
	ts->early_suspend.level = EARLY_SUSPEND_LEVEL_BLANK_SCREEN + 1;
	ts->early_suspend.suspend = cyttsp4_early_suspend;
	ts->early_suspend.resume = cyttsp4_late_resume;
	register_early_suspend(&ts->early_suspend);
#endif

	INIT_WORK(&ts->cyttsp4_resume_startup_work, cyttsp4_ts_work_func);

	#if defined(SH_TPSIF_LOCAL_DEVICE_REGIST_ENABLE)
		sh_tpsif_device_port_set();
	#endif /* SH_TPSIF_LOCAL_DEVICE_REGIST_ENABLE */

	goto no_error;

error_input_register_device:
	if (input_device->mt)
		input_mt_destroy_slots(input_device);
	input_free_device(input_device);

error_init:
	mutex_destroy(&ts->data_lock);
	if (ts->cyttsp4_wq) {
		destroy_workqueue(ts->cyttsp4_wq);
		ts->cyttsp4_wq = NULL;
	}
error_alloc_failed:
#ifdef CONFIG_TOUCHSCREEN_DEBUG
	if (ts->fwname != NULL) {
		kfree(ts->fwname);
		ts->fwname = NULL;
	}
	if (ts->pr_buf != NULL) {
		kfree(ts->pr_buf);
		ts->pr_buf = NULL;
	}
#endif /* --CONFIG_TOUCHSCREEN_DEBUG */
	if (ts != NULL) {
		kfree(ts);
		ts = NULL;
	}
error_alloc_data:
	dev_err(ts->dev,
			"%s: Failed Initialization\n", __func__);
no_error:
	return ts;
}
EXPORT_SYMBOL_GPL(cyttsp4_core_init);

MODULE_LICENSE("GPL");
MODULE_DESCRIPTION("Cypress TrueTouch(R) Standard touchscreen driver core");
MODULE_AUTHOR("Cypress");


#ifdef SH_TPSIF_COMMAND
#include <linux/cdev.h>		/* cdev */
#include <asm/uaccess.h>	/* copy_from_user, copy_to_user */
#include <linux/poll.h>
#include <linux/sched.h>
#include <linux/wait.h>

/* power */
#define SH_TPSIF_HW_POWER_ON	1
#define SH_TPSIF_HW_POWER_OFF	0

#define SH_TPSIF_POLL_INTERVAL 100 /* [ms] */

static dev_t sh_tpsif_dev;
static struct cdev sh_tpsif_cdev;
static struct class *sh_tpsif_class;

static struct cyttsp4_scan_data sh_tpsif_scan_data[2];
static struct cyttsp4_idac_data sh_tpsif_idac_data;
static int sh_tpsif_scan_type = TPSIF_SCAN_TYPE_MUTUAL_RAW;
static int sh_tpsif_polling_loop = 0;

static void _sh_tpsif_adjust_point(struct cyttsp4 *ts, int *x, int *y)
{
	int i;
	long l_xpq;
	long l_xrs;
	long l_x;
	long l_ypr;
	long l_yqs;
	long l_y;

	dev_vdbg(ts->dev, "%s: before (%d, %d)\n", __func__, *x, *y);

	/* divide the area */
	for (i = 0; i < AREA_COUNT; i++) {
		if (sh_tpsif_area_rect[i].p.x <= *x && sh_tpsif_area_rect[i].s.x > *x &&
			sh_tpsif_area_rect[i].p.y <= *y && sh_tpsif_area_rect[i].s.y > *y)
			break;
	}
	/* if not belong to any area, do not adjust */
	if (i != AREA_COUNT) {
		/* do an adjustment of coordinate */

		l_xpq = (((sh_tpsif_area_diff[i].q.x * DOUBLE_ACCURACY) -
				(sh_tpsif_area_diff[i].p.x * DOUBLE_ACCURACY)) /
			(sh_tpsif_area_rect[i].q.x - sh_tpsif_area_rect[i].p.x)) *
			(*x - sh_tpsif_area_rect[i].p.x) +
			(sh_tpsif_area_diff[i].p.x * DOUBLE_ACCURACY);
		l_xrs = (((sh_tpsif_area_diff[i].s.x * DOUBLE_ACCURACY) -
				(sh_tpsif_area_diff[i].r.x * DOUBLE_ACCURACY)) /
			(sh_tpsif_area_rect[i].s.x - sh_tpsif_area_rect[i].r.x)) *
			(*x - sh_tpsif_area_rect[i].r.x) +
			(sh_tpsif_area_diff[i].r.x * DOUBLE_ACCURACY);
		l_x   = ((l_xrs - l_xpq) / (sh_tpsif_area_rect[i].r.y - sh_tpsif_area_rect[i].p.y)) *
			(*y - sh_tpsif_area_rect[i].p.y) + l_xpq;
		l_ypr = (((sh_tpsif_area_diff[i].r.y * DOUBLE_ACCURACY) -
				(sh_tpsif_area_diff[i].p.y * DOUBLE_ACCURACY)) /
			(sh_tpsif_area_rect[i].r.y - sh_tpsif_area_rect[i].p.y)) *
			(*y - sh_tpsif_area_rect[i].p.y) +
			(sh_tpsif_area_diff[i].p.y * DOUBLE_ACCURACY);
		l_yqs = (((sh_tpsif_area_diff[i].s.y * DOUBLE_ACCURACY) -
				(sh_tpsif_area_diff[i].q.y * DOUBLE_ACCURACY)) /
			(sh_tpsif_area_rect[i].s.y - sh_tpsif_area_rect[i].q.y)) *
			(*y - sh_tpsif_area_rect[i].q.y) +
			(sh_tpsif_area_diff[i].q.y * DOUBLE_ACCURACY);
		l_y   = ((l_yqs - l_ypr) / (sh_tpsif_area_rect[i].q.x - sh_tpsif_area_rect[i].p.x)) *
			(*x - sh_tpsif_area_rect[i].p.x) + l_ypr;
		*x = *x - (int)(l_x / DOUBLE_ACCURACY);
		*y = *y - (int)(l_y / DOUBLE_ACCURACY);
	}

	/* to be adjusted inside the range */
	*x = MINMAX(0, ts->platform_data->frmwrk->abs[(CY_ABS_X_OST*CY_NUM_ABS_SET)+CY_MAX_OST], *x);
	*y = MINMAX(0, ts->platform_data->frmwrk->abs[(CY_ABS_Y_OST*CY_NUM_ABS_SET)+CY_MAX_OST], *y);

	dev_vdbg(ts->dev, "%s: after (%d, %d)\n", __func__, *x, *y);
}

static void _sh_tpsif_qsort(sh_tpsif_qsort_t *table, int top, int end)
{
	int i, j;
	int center;
	sh_tpsif_qsort_t swap;

	i = top;
	j = end;

	center = table[(top + end) / 2].value;

	while (1) {
		while (table[i].value < center)
			i++;
		while (center < table[j].value)
			j--;
		if (i >= j)
			break;
		memcpy(&swap, &table[i], sizeof(sh_tpsif_qsort_t));
		memcpy(&table[i], &table[j], sizeof(sh_tpsif_qsort_t));
		memcpy(&table[j], &swap, sizeof(sh_tpsif_qsort_t));
		i++;
		j--;
	}
	if (top < i - 1)
		_sh_tpsif_qsort(table, top, i - 1);
	if (j + 1 < end)
		_sh_tpsif_qsort(table, j + 1, end);
}

static void _sh_tpsif_round_value(int *value)
{
	sh_tpsif_qsort_t table[6];
	int i;

	for (i = 0; i < 6; i++) {
		table[i].num = i;
		table[i].value = value[i];
	}
	_sh_tpsif_qsort(table, 0, 5);
	value[table[0].num] = value[table[1].num];
	value[table[5].num] = value[table[4].num];
}

static int sh_tpsif_set_adjust_param(struct cyttsp4 *ts, u16 *param)
{
	int i;
	sh_tpsif_point_t sd[ADJUST_POINT];
	int diff[2][6];
	int retval = 0;

	/* adjustment of coordinate is invalid */
	if (param == NULL) {
		ts->adjust_enabled = false;
		dev_vdbg(ts->dev, "%s: param is NULL\n", __func__);
		return retval;
	}

	dev_vdbg(ts->dev, "%s: (%4d, %4d) (%4d, %4d)\n", __func__, param[0], param[1], param[2], param[3]);
	dev_vdbg(ts->dev, "%s: (%4d, %4d) (%4d, %4d)\n", __func__, param[4], param[5], param[6], param[7]);
	dev_vdbg(ts->dev, "%s: (%4d, %4d) (%4d, %4d)\n", __func__, param[8], param[9], param[10], param[11]);

	/* parameter check */
	for (i = 0; i < ADJUST_POINT; i++) {
		if (param[i * 2 + 0] > sh_tpsif_base_point[i].x + POS_LIMIT ||
			param[i * 2 + 0] < sh_tpsif_base_point[i].x - POS_LIMIT)
			return -EINVAL;
		if (param[i * 2 + 1] > sh_tpsif_base_point[i].y + POS_LIMIT ||
			param[i * 2 + 1] < sh_tpsif_base_point[i].y - POS_LIMIT)
			return -EINVAL;
	}

	/* save parameters */
	SET_POINT(sh_tpsif_adjust_param[0], param[ 0], param[ 1]);
	SET_POINT(sh_tpsif_adjust_param[1], param[ 2], param[ 3]);
	SET_POINT(sh_tpsif_adjust_param[2], param[ 4], param[ 5]);
	SET_POINT(sh_tpsif_adjust_param[3], param[ 6], param[ 7]);
	SET_POINT(sh_tpsif_adjust_param[4], param[ 8], param[ 9]);
	SET_POINT(sh_tpsif_adjust_param[5], param[10], param[11]);

#if 0	/* changing the calculation method of diff */
	/* calculate diff value */
	for (i = 0; i < ADJUST_POINT; i++) {
		sd[i].x = (sh_tpsif_adjust_param[i].x - sh_tpsif_base_point[i].x) * 3 / 4;
		sd[i].y = (sh_tpsif_adjust_param[i].y - sh_tpsif_base_point[i].y) * 3 / 4;
	}
#else
	for (i = 0; i < ADJUST_POINT; i++) {
		diff[0][i] = (sh_tpsif_adjust_param[i].x - sh_tpsif_base_point[i].x);
		diff[1][i] = (sh_tpsif_adjust_param[i].y - sh_tpsif_base_point[i].y);
	}

	/* truncate the maximum and minimum */
	_sh_tpsif_round_value(diff[0]); /* X */
	_sh_tpsif_round_value(diff[1]); /* Y */

	for (i = 0; i < ADJUST_POINT; i++) {
		sd[i].x = diff[0][i] * 75 / 100;
		sd[i].y = diff[1][i] * 75 / 100;
	}
#endif	/* changing the calculation method of diff */

	/* store the blurring value of each four area corners */
	/*                     |-------p-------| |-------q-------| |-------r-------| |-------s-------|*/
	SET_AREA(sh_tpsif_area_diff[ 0], 0      , 0      , sd[0].x, 0      , 0      , sd[0].y, sd[0].x, sd[0].y);
	SET_AREA(sh_tpsif_area_diff[ 1], sd[0].x, 0      , sd[1].x, 0      , sd[0].x, sd[0].y, sd[1].x, sd[1].y);
	SET_AREA(sh_tpsif_area_diff[ 2], sd[1].x, 0      , 0      , 0      , sd[1].x, sd[1].y, 0      , sd[1].y);
	SET_AREA(sh_tpsif_area_diff[ 3], 0      , sd[0].y, sd[0].x, sd[0].y, 0      , sd[2].y, sd[2].x, sd[2].y);
	SET_AREA(sh_tpsif_area_diff[ 4], sd[0].x, sd[0].y, sd[1].x, sd[1].y, sd[2].x, sd[2].y, sd[3].x, sd[3].y);
	SET_AREA(sh_tpsif_area_diff[ 5], sd[1].x, sd[1].y, 0      , sd[1].y, sd[3].x, sd[3].y, 0      , sd[3].y);
	SET_AREA(sh_tpsif_area_diff[ 6], 0      , sd[2].y, sd[2].x, sd[2].y, 0      , sd[4].y, sd[4].x, sd[4].y);
	SET_AREA(sh_tpsif_area_diff[ 7], sd[2].x, sd[2].y, sd[3].x, sd[3].y, sd[4].x, sd[4].y, sd[5].x, sd[5].y);
	SET_AREA(sh_tpsif_area_diff[ 8], sd[3].x, sd[3].y, 0      , sd[3].y, sd[5].x, sd[5].y, 0      , sd[5].y);
	SET_AREA(sh_tpsif_area_diff[ 9], 0      , sd[4].y, sd[4].x, sd[4].y, 0      , 0      , sd[4].x, 0      );
	SET_AREA(sh_tpsif_area_diff[10], sd[4].x, sd[4].y, sd[5].x, sd[5].y, sd[4].x, 0      , sd[5].x, 0      );
	SET_AREA(sh_tpsif_area_diff[11], sd[5].x, sd[5].y, 0      , sd[5].y, sd[5].x, 0      , 0      , 0      );

	/* to valid an adjustment of coordinate */
	ts->adjust_enabled = true;

	return retval;
}

static int sh_tpsif_hw_power(struct cyttsp4 *ts, int on)
{
	int retval = 0;

	mutex_lock(&ts->data_lock);

	if (ts->platform_data->hw_power)
		retval = ts->platform_data->hw_power(on);
	else
		retval = -ENOSYS;

	mutex_unlock(&ts->data_lock);

	return retval;
}

static int sh_tpsif_hw_reset(struct cyttsp4 *ts)
{
	int retval = 0;

	mutex_lock(&ts->data_lock);

	if (ts->platform_data->hw_reset)
		retval = ts->platform_data->hw_reset();
	else
		retval = -ENOSYS;

	if (retval == 0) {
		ts->current_mode = CY_MODE_BOOTLOADER;
		ts->driver_state = CY_BL_STATE;
	}

	mutex_unlock(&ts->data_lock);

	return retval;
}

static int _sh_tpsif_sw_reset(struct cyttsp4 *ts)
{
	int retval = 0;

	retval = _cyttsp4_soft_reset(ts);
	if (retval == 0) {
		ts->current_mode = CY_MODE_BOOTLOADER;
		ts->driver_state = CY_BL_STATE;
	}

	return retval;
}

static int sh_tpsif_sw_reset(struct cyttsp4 *ts)
{
	int retval = 0;

	mutex_lock(&ts->data_lock);
	retval = _sh_tpsif_sw_reset(ts);
	mutex_unlock(&ts->data_lock);

	return retval;
}

/* copied from _cyttsp4_startup_tma400() */
#ifdef CY_USE_TMA400
static int _sh_tpsif_startup_sw_reset_tma400(struct cyttsp4 *ts)
{
	int tries;
	int retval = 0;
	u8 ic_crc[2];
	u8 table_crc[2];
	bool put_all_params_done = false;
	bool upgraded = false;
	bool mddata_updated = false;
#ifdef SH_TPSIF_COMMAND
	u32 version;
#endif	/* SH_TPSIF_COMMAND */

	put_all_params_done = true;
	/* 
	 * FIXME
	 *
	 * Default value of "put_all_params_done" is false.  Now
	 * changed to true in order to check firmware update function.
	 * 
	 * Don't forget to return to false in software for production.
	 *
	 */

	tries = 0;
#ifdef CY_USE_WATCHDOG
	_cyttsp4_stop_wd_timer(ts);
#endif
_sh_tpsif_startup_sw_reset_tma400_restart:

	dev_vdbg(ts->dev,
		"%s: enter driver_state=%d\n", __func__, ts->driver_state);
	ts->current_mode = CY_MODE_BOOTLOADER;
	retval = _sh_tpsif_sw_reset(ts);	/* Soft Reset */
	if (retval < 0) {
		dev_err(ts->dev,
			"%s: Fail reset device r=%d\n", __func__, retval);
		/* continue anyway in case device was already in bootloader */
	}
	/*
	 * Wait for heartbeat interrupt. If we didn't get the CPU quickly, this
	 * may not be the first interupt.
	 */
	dev_vdbg(ts->dev,
		"%s: wait for first bootloader interrupt\n", __func__);
	retval = _cyttsp4_wait_int(ts, CY_TEN_SEC_TMO_MS);
	if (retval < 0) {
		dev_err(ts->dev,
			"%s: Fail waiting for bootloader interrupt\n",
			__func__);
		goto _sh_tpsif_startup_sw_reset_tma400_exit;
	}

	/*
	 * exit BL mode and eliminate race between heartbeat and
	 * command / response interrupts
	 */
	_cyttsp4_change_state(ts, CY_EXIT_BL_STATE);
	ts->switch_flag = true;
	retval = _cyttsp4_wait_si_int(ts, CY_TEN_SEC_TMO_MS);
	if (retval < 0) {
		dev_err(ts->dev,
			"%s: Fail wait switch to Sysinfo r=%d\n",
			__func__, retval);
		/* continue anyway in case sync missed */
	}
	if (ts->driver_state != CY_SYSINFO_STATE) {
		dev_err(ts->dev,
			"%s: Fail set sysinfo mode; switch to sysinfo anyway\r",
			__func__);
		_cyttsp4_change_state(ts, CY_SYSINFO_STATE);
	} else {
		dev_vdbg(ts->dev,
			"%s: Exit BL ok; now in sysinfo mode\n", __func__);
		_cyttsp4_pr_state(ts);
	}

#ifdef TMA443_COMPATIBILITY
	if (ts->tma443_compat) {
		dev_vdbg(ts->dev, "%s: msleep(5000)\n", __func__);
		msleep(5000);
	}
#endif

	dev_vdbg(ts->dev,
		"%s: Read Sysinfo regs and get rev numbers try=%d\n",
		__func__, tries);
	retval = _cyttsp4_get_sysinfo_regs(ts);
	if (retval < 0) {
		dev_err(ts->dev,
			"%s: Read Block fail -get sys regs (r=%d)\n",
			__func__, retval);
		dev_err(ts->dev,
			"%s: Fail to switch from Bootloader "
			"to Application r=%d\n",
			__func__, retval);

		_cyttsp4_change_state(ts, CY_BL_STATE);

		if (upgraded) {
			dev_err(ts->dev,
			"%s: app failed to launch after"
				" platform firmware upgrade\n", __func__);
			retval = -EIO;
			goto _sh_tpsif_startup_sw_reset_tma400_exit;
		}

#ifndef CY_NO_AUTO_LOAD
		dev_info(ts->dev,
			"%s: attempting to reflash IC...\n", __func__);
		if (ts->platform_data->fw->img == NULL ||
			ts->platform_data->fw->size == 0) {
			dev_err(ts->dev,
			"%s: no platform firmware available"
				" for reflashing\n", __func__);
			_cyttsp4_change_state(ts, CY_INVALID_STATE);
			retval = -ENODATA;
			goto _sh_tpsif_startup_sw_reset_tma400_exit;
		}
		retval = _cyttsp4_load_app(ts,
			ts->platform_data->fw->img,
			ts->platform_data->fw->size);
		if (retval) {
			dev_err(ts->dev,
			"%s: failed to reflash IC (r=%d)\n",
				__func__, retval);
			_cyttsp4_change_state(ts, CY_INVALID_STATE);
			retval = -EIO;
			goto _sh_tpsif_startup_sw_reset_tma400_exit;
		}
		upgraded = true;
		dev_info(ts->dev,
			"%s: resetting IC after reflashing\n", __func__);
		goto _sh_tpsif_startup_sw_reset_tma400_restart; /* Reset the part */
#endif /* --CY_NO_AUTO_LOAD */
	}

#ifdef SH_TPSIF_COMMAND
	if (_sh_tpsif_firmware_version(ts, &version) < 0) {
		dev_err(ts->dev, "%s: Touch Panel Firmware Version = Unknown\n", __func__);
	} else {
		dev_info(ts->dev, "%s: Touch Panel Firmware Version = %02X.%02X.%02X%02X\n",
			__func__, ((u8 *)&version)[3], ((u8 *)&version)[2],
			((u8 *)&version)[1], ((u8 *)&version)[0]);
		if (((u8 *)&version)[3] == 0x00 && ((u8 *)&version)[2] == 0x24) {
			dev_info(ts->dev, "%s: Firmware for ES10.2\n", __func__);
			put_all_params_done = true;
			mddata_updated = true;
			if (((u8 *)&version)[1] == 0x13 && ((u8 *)&version)[0] == 0xEC) {
				dev_info(ts->dev, "%s: Inverted Y-axis\n", __func__);
				ts->flags |= CY_INV_Y;
			}
		} else {
			dev_info(ts->dev, "%s: Firmware for ES100\n", __func__);
		}
	}
#endif	/* SH_TPSIF_COMMAND */

#ifndef CY_NO_AUTO_LOAD
#ifdef CONFIG_TOUCHSCREEN_DEBUG
	if (!ts->ic_grptest && !(ts->debug_upgrade)) {
#endif /* --CONFIG_TOUCHSCREEN_DEBUG */
		retval = _cyttsp4_boot_loader(ts, &upgraded);
		if (retval < 0) {
			dev_err(ts->dev,
			"%s: fail boot loader r=%d)\n",
				__func__, retval);
			_cyttsp4_change_state(ts, CY_IDLE_STATE);
			goto _sh_tpsif_startup_sw_reset_tma400_exit;
		}
		if (upgraded)
			goto _sh_tpsif_startup_sw_reset_tma400_restart;
#ifdef CONFIG_TOUCHSCREEN_DEBUG
	}
#endif /* --CONFIG_TOUCHSCREEN_DEBUG */
#endif /* --CY_NO_AUTO_LOAD */

	retval = _cyttsp4_set_mode(ts, CY_CONFIG_MODE);
	if (retval < 0) {
		dev_err(ts->dev,
			"%s: Fail set config mode 1 r=%d\n", __func__, retval);
		goto _sh_tpsif_startup_sw_reset_tma400_bypass_crc_check;
	}

	retval = _cyttsp4_get_ebid_row_size(ts);
	if (retval < 0) {
		dev_err(ts->dev,
			"%s: Fail get EBID row size; using default r=%d\n",
			__func__, retval);
	}
	dev_vdbg(ts->dev,
		"%s: get EBID row size=%d\n", __func__, ts->ebid_row_size);

#ifdef CONFIG_TOUCHSCREEN_DEBUG
	if (ts->ic_grptest)
		goto _sh_tpsif_startup_sw_reset_tma400_bypass_crc_check;
#endif /* --CONFIG_TOUCHSCREEN_DEBUG */
	memset(ic_crc, 0, sizeof(ic_crc));
	dev_vdbg(ts->dev,
		"%s: Read IC CRC values\n", __func__);
	/* Get settings CRC from touch IC */
	retval = _cyttsp4_set_mode(ts, CY_OPERATE_MODE);
	if (retval < 0) {
		dev_err(ts->dev,
			"%s: Fail set operational mode 1 (r=%d)\n",
			__func__, retval);
		goto _sh_tpsif_startup_sw_reset_tma400_exit;
	}
	retval = _cyttsp4_get_ic_crc(ts, CY_TCH_PARM_EBID,
		&ic_crc[0], &ic_crc[1]);
	if (retval < 0) {
		dev_err(ts->dev,
			"%s: Fail read ic crc r=%d\n",
			__func__, retval);
	}

	_cyttsp4_pr_buf(ts, ic_crc, sizeof(ic_crc), "read_ic_crc");

	retval = _cyttsp4_set_mode(ts, CY_CONFIG_MODE);
	if (retval < 0) {
		dev_err(ts->dev,
			"%s: Fail set config mode 2 r=%d\n", __func__, retval);
		goto _sh_tpsif_startup_sw_reset_tma400_exit;
	}
	if (!put_all_params_done) {
		if (ts->platform_data->sett
			[CY_IC_GRPNUM_TCH_PARM_VAL]->data == NULL) {
			dev_err(ts->dev,
			"%s: missing param table\n", __func__);
			goto _sh_tpsif_startup_sw_reset_tma400_bypass_crc_check;
		} else if (ts->platform_data->sett[CY_IC_GRPNUM_TCH_PARM_VAL]
			->data == NULL) {
			dev_err(ts->dev,
			"%s: missing param values table data\n",
				__func__);
			goto _sh_tpsif_startup_sw_reset_tma400_bypass_crc_check;
		} else if (ts->platform_data->sett[CY_IC_GRPNUM_TCH_PARM_VAL]
			->size == 0) {
			dev_err(ts->dev,
			"%s: param values table size is 0\n", __func__);
			goto _sh_tpsif_startup_sw_reset_tma400_bypass_crc_check;
		}
		_cyttsp_read_table_crc(ts, ts->platform_data->sett
			[CY_IC_GRPNUM_TCH_PARM_VAL]->data,
			&table_crc[0], &table_crc[1]);
		_cyttsp4_pr_buf(ts, table_crc, sizeof(table_crc),
			"read_table_crc");
		if ((ic_crc[0] != table_crc[0]) ||
			(ic_crc[1] != table_crc[1])) {
			retval = _cyttsp4_put_all_params_tma400(ts);
			if (retval < 0) {
				dev_err(ts->dev,
			"%s: Fail put all params r=%d\n",
					__func__, retval);
				goto _sh_tpsif_startup_sw_reset_tma400_bypass_crc_check;
			}
			put_all_params_done = true;
			goto _sh_tpsif_startup_sw_reset_tma400_restart;
		}
	}

_sh_tpsif_startup_sw_reset_tma400_bypass_crc_check:
#ifdef STARTUP_MDDATA_CHECK
	if (!mddata_updated) {
		retval = _cyttsp4_check_mddata_tma400(ts, &mddata_updated);
		if (retval < 0) {
			dev_err(ts->dev,
			"%s: Fail update MDDATA r=%d\n",
				__func__, retval);
		} else if (mddata_updated)
			goto _sh_tpsif_startup_sw_reset_tma400_restart;
	}
#endif	/* STARTUP_MDDATA_CHECK */

	dev_vdbg(ts->dev,
		"%s: enter operational mode\n", __func__);
	/* mode=operational mode, state = active_state */
	retval = _cyttsp4_set_mode(ts, CY_OPERATE_MODE);
	if (retval < 0) {
		dev_err(ts->dev,
			"%s: Fail set operational mode 2 (r=%d)\n",
			__func__, retval);
		goto _sh_tpsif_startup_sw_reset_tma400_exit;
	}

	if (ts->was_suspended) {
		ts->was_suspended = false;
		retval = _cyttsp4_enter_sleep(ts);
		if (retval < 0) {
			dev_err(ts->dev,
			"%s: fail resume sleep r=%d\n",
				__func__, retval);
		}
	} else {
#ifdef CY_USE_WATCHDOG
		_cyttsp4_start_wd_timer(ts);
#endif
	}
_sh_tpsif_startup_sw_reset_tma400_exit:
	return retval;
}
#endif /* --CY_USE_TMA400 */

#ifdef CY_USE_TMA884
static int _sh_tpsif_startup_sw_reset_tma400(struct cyttsp4 *ts)
{
	return 0;
}
#endif /* --CY_USE_TMA884 */

static int _sh_tpsif_startup_sw_reset(struct cyttsp4 *ts)
{
	int retval = 0;

	retval = _sh_tpsif_startup_sw_reset_tma400(ts);
	if (retval < 0) {
		dev_err(ts->dev, "%s: Fail startup TMA400 r=%d\n",
			__func__, retval);
	}

	return retval;
}

static int sh_tpsif_hw_reset_startup(struct cyttsp4 *ts)
{
	int retval = 0;
	enum cyttsp4_driver_state tmp_state;

	tmp_state = ts->driver_state;

	if (tmp_state == CY_SLEEP_STATE) {
		retval = cyttsp4_resume(ts);
		if (retval < 0) {
			dev_err(ts->dev, "%s: Fail resume\n", __func__);
			return retval;
		}
	}

	mutex_lock(&ts->data_lock);
	retval = _cyttsp4_startup(ts);
	mutex_unlock(&ts->data_lock);

	if (tmp_state == CY_SLEEP_STATE) {
		cyttsp4_suspend(ts);
	}

	return retval;
}

static int sh_tpsif_sw_reset_startup(struct cyttsp4 *ts)
{
	int retval = 0;
	enum cyttsp4_driver_state tmp_state;

	tmp_state = ts->driver_state;

	if (tmp_state == CY_SLEEP_STATE) {
		retval = cyttsp4_resume(ts);
		if (retval < 0) {
			dev_err(ts->dev, "%s: Fail resume\n", __func__);
			return retval;
		}
	}

	mutex_lock(&ts->data_lock);
	retval = _sh_tpsif_startup_sw_reset(ts);
	mutex_unlock(&ts->data_lock);

	if (tmp_state == CY_SLEEP_STATE) {
		cyttsp4_suspend(ts);
	}

	return retval;
}

static int sh_tpsif_change_mode(struct cyttsp4 *ts, int mode)
{
	int retval = -EINVAL;

	mutex_lock(&ts->data_lock);

	if (ts->driver_state != CY_ACTIVE_STATE) {
		dev_err(ts->dev, "%s: Cannot change mode in sleep state\n", __func__);
		goto sh_tpsif_change_mode_exit;
	}

	switch (mode) {
	case TPSIF_MODE_OPERATE:
		retval = _cyttsp4_set_mode(ts, CY_OPERATE_MODE);
		break;
	case TPSIF_MODE_SYSINFO:
		retval = _cyttsp4_set_mode(ts, CY_SYSINFO_MODE);
		break;
	case TPSIF_MODE_CONFIG:
		retval = _cyttsp4_set_mode(ts, CY_CONFIG_MODE);
		break;
	}

	if (retval < 0) {
		dev_err(ts->dev, "%s: Fail change mode (r=%d)\n",
			__func__, retval);
		_cyttsp4_change_state(ts, CY_IDLE_STATE);
	}

sh_tpsif_change_mode_exit:

	mutex_unlock(&ts->data_lock);
	return retval;
}

static int _sh_tpsif_read_scan_result(struct cyttsp4 *ts, int type, struct cyttsp4_scan_data *data)
{
	int retval = 0;
	uint8_t response[6] = {0x00, 0x00, 0x00, 0x00, 0x00, 0x00};

	retval = _cyttsp4_read_block_data(ts, ts->si_ofs.cmd_ofs,
					sizeof(response), &(response[0]),
					ts->platform_data->addr[CY_TCH_ADDR_OFS],
					true);
	if (retval < 0) {
		dev_err(ts->dev, "%s: fail read cmd status"
			" r=%d\n", __func__, retval);
		goto _sh_tpsif_read_scan_result_error;
	}

	dev_vdbg(ts->dev, "%s: Status         = 0x%02X\n", __func__, response[1]);
	dev_vdbg(ts->dev, "%s: Data ID        = 0x%02X\n", __func__, response[2]);
	dev_vdbg(ts->dev, "%s: Num Elements 1 = 0x%02X\n", __func__, response[3]);
	dev_vdbg(ts->dev, "%s: Num Elements 2 = 0x%02X\n", __func__, response[4]);
	dev_vdbg(ts->dev, "%s: Data format    = 0x%02X\n", __func__, response[5]);

	data->data_num = ((u16)response[3] << 8) | response[4];
	data->data_size = response[5] & 0x07;
	data->matrix_mapping = (response[5] & 0x08) >> 3;
	data->endian = (response[5] & 0x10) >> 4;
	data->sign_type = (response[5] & 0x20) >> 5;
	data->scan_type = type;

	data->buf_size = sizeof(u8) * data->data_size * data->data_num;

	retval = _cyttsp4_read_block_data(ts, ts->si_ofs.cmd_ofs + 6,
					data->buf_size, data->buf,
					ts->platform_data->addr[CY_TCH_ADDR_OFS],
					true);
	if (retval < 0) {
		dev_err(ts->dev, "%s: fail read scan result"
			" r=%d\n", __func__, retval);
		goto _sh_tpsif_read_scan_result_error;
	}

	goto _sh_tpsif_read_scan_result_exit;

_sh_tpsif_read_scan_result_error:

_sh_tpsif_read_scan_result_exit:

	return retval;
}

static int _sh_tpsif_retrieve_panel_scan(struct cyttsp4 *ts, int type)
{
	int retval = 0;
	uint8_t *buf = NULL;
	size_t buf_size;
	u8 status;

	buf_size = sizeof(uint8_t) * 6;
	buf = kzalloc(buf_size, GFP_KERNEL);
	if (buf == NULL) {
		dev_err(ts->dev, "%s: Failed to allocate buffer\n", __func__);
		retval = -ENOMEM;
		goto _sh_tpsif_retrieve_panel_scan_exit;
	}

	if (type < 0 || type >= TPSIF_SCAN_TYPE_NUM) {
		dev_err(ts->dev, "%s: Ivalid scan type %d\n", __func__, type);
		retval = -EINVAL;
		goto _sh_tpsif_retrieve_panel_scan_exit;
	}

	/* Set command bytes */
	buf[0] = 0x0C; /* Command */
	buf[1] = 0x00; /* Offset */
	buf[2] = 0x00; /* Offset */
	buf[3] = (uint8_t) ((TPSIF_TX_NUM * TPSIF_RX_NUM) >> 8); /* MSB of (TX * RX) */
	buf[4] = (uint8_t) ((TPSIF_TX_NUM * TPSIF_RX_NUM) & 0xff); /* LSB of (TX * RX) */
	buf[5] = type;

	retval = _cyttsp4_write_block_data(ts, ts->si_ofs.cmd_ofs + 1,
					5, &(buf[1]),
					ts->platform_data->addr[CY_TCH_ADDR_OFS], true);
	if (retval < 0) {
		dev_err(ts->dev, "%s: Failed to write parameters for retrieve command r=%d\n",
			__func__, retval);
		goto _sh_tpsif_retrieve_panel_scan_exit;
	}

	/* Write command */
	retval = _cyttsp4_put_cmd_wait(ts, ts->si_ofs.cmd_ofs,
				1, &(buf[0]), CY_HALF_SEC_TMO_MS,
				_cyttsp4_chk_cmd_rdy, NULL,
				ts->platform_data->addr[CY_TCH_ADDR_OFS], true);
	if (retval < 0) {
		dev_err(ts->dev, "%s: Failed to write retrieve command r=%d\n",
			__func__, retval);
		goto _sh_tpsif_retrieve_panel_scan_exit;
	}

	retval = _cyttsp4_read_block_data(ts, ts->si_ofs.cmd_ofs + 1,
					sizeof(status), &status,
					ts->platform_data->addr[CY_TCH_ADDR_OFS],
					true);
	if (retval < 0) {
		dev_err(ts->dev, "%s: fail read cmd status r=%d\n", __func__, retval);
		goto _sh_tpsif_retrieve_panel_scan_exit;
	}

	if (status != 0x00) {
		dev_err(ts->dev, "%s: Retrieve panel scan command failed"
			" status=%d\n", __func__, status);
		retval = -EIO;
	}

_sh_tpsif_retrieve_panel_scan_exit:
	kfree(buf);
	return retval;
}

static int _sh_tpsif_execute_panel_scan(struct cyttsp4 *ts)
{
	int retval = 0;
	uint8_t *buf = NULL;
	size_t buf_size;
	u8 status;

	buf_size = sizeof(uint8_t) * 1;
	buf = kzalloc(buf_size, GFP_KERNEL);
	if (buf == NULL) {
		dev_err(ts->dev, "%s: Failed to allocate buffer\n", __func__);
		retval = -ENOMEM;
		goto _sh_tpsif_execute_panel_scan_exit;
	}

	/* Set command bytes */
	buf[0] = 0x0B; /* Command */

	/* Write command */
	retval = _cyttsp4_put_cmd_wait(ts, ts->si_ofs.cmd_ofs,
				sizeof(buf), &(buf[0]), CY_HALF_SEC_TMO_MS,
				_cyttsp4_chk_cmd_rdy, NULL,
				ts->platform_data->addr[CY_TCH_ADDR_OFS], true);
	if (retval < 0) {
		dev_err(ts->dev, "%s: Failed to write execute panel scan command r=%d\n",
			__func__, retval);
		goto _sh_tpsif_execute_panel_scan_exit;
	}

	retval = _cyttsp4_read_block_data(ts, ts->si_ofs.cmd_ofs + 1,
					sizeof(status), &status,
					ts->platform_data->addr[CY_TCH_ADDR_OFS],
					true);
	if (retval < 0) {
		dev_err(ts->dev, "%s: fail read cmd status r=%d\n", __func__, retval);
		goto _sh_tpsif_execute_panel_scan_exit;
	}

	if (status != 0x00) {
		dev_err(ts->dev, "%s: Execute panel scan command failed"
			" status=%d\n", __func__, status);
		retval = -EIO;
	}

_sh_tpsif_execute_panel_scan_exit:
	kfree(buf);
	return retval;
}

static int sh_tpsif_scan_rawdata(struct cyttsp4 *ts, int type, struct cyttsp4_scan_data *data)
{
	int retval = 0;

	mutex_lock(&ts->data_lock);

	/* Switch to config mode */
	if (ts->current_mode != CY_MODE_CONFIG) {
		retval = _cyttsp4_set_mode(ts, CY_CONFIG_MODE);
		if (retval < 0) {
			dev_err(ts->dev, "%s: Failed to switch to config mode"
				" for sysinfo regs\n", __func__);
			goto sh_tpsif_scan_rawdata_exit;
		}
	}

	/* Execute panel scan */
	retval = _sh_tpsif_execute_panel_scan(ts);
	if (retval < 0) {
		dev_err(ts->dev, "%s: Fail execute panel scan r=%d\n", __func__, retval);
		sh_tpsif_polling_loop = 0;
		goto sh_tpsif_scan_rawdata_op_mode;
	}

	/* Retrieve panel scan */
	retval = _sh_tpsif_retrieve_panel_scan(ts, type);
	if (retval < 0) {
		dev_err(ts->dev, "%s: Fail retrieve panel scan r=%d\n", __func__, retval);
		sh_tpsif_polling_loop = 0;
		goto sh_tpsif_scan_rawdata_op_mode;
	}

	/* Read scan result */
	retval = _sh_tpsif_read_scan_result(ts, type, data);
	if (retval < 0) {
		dev_err(ts->dev, "%s: Fail read scan result r=%d\n", __func__, retval);
		sh_tpsif_polling_loop = 0;
		goto sh_tpsif_scan_rawdata_op_mode;
	}

sh_tpsif_scan_rawdata_op_mode:

sh_tpsif_scan_rawdata_exit:

	mutex_unlock(&ts->data_lock);

	return retval;
}

static int _sh_tpsif_retrieve_data_structure(struct cyttsp4 *ts, int type)
{
	int retval = 0;
	uint8_t *buf = NULL;
	size_t buf_size;
	u8 status;

	buf_size = sizeof(uint8_t) * 6;
	buf = kzalloc(buf_size, GFP_KERNEL);
	if (buf == NULL) {
		dev_err(ts->dev, "%s: Failed to allocate buffer\n", __func__);
		retval = -ENOMEM;
		goto _sh_tpsif_retrieve_data_structure_exit;
	}

	if (type < 0 || type >= TPSIF_SCAN_TYPE_NUM) {
		dev_err(ts->dev, "%s: Ivalid scan type %d\n", __func__, type);
		retval = -EINVAL;
		goto _sh_tpsif_retrieve_data_structure_exit;
	}

	/* Set command bytes */
	buf[0] = 0x10; /* Command */
	buf[1] = 0x00; /* Offset */
	buf[2] = 0x00; /* Offset */
	if (type == 0x00) {	/* Mutual IDAC */
		buf[3] = (uint8_t) ((TPSIF_TX_NUM * TPSIF_RX_NUM + 1) >> 8); /* MSB of (TX * RX + 1) */
		buf[4] = (uint8_t) ((TPSIF_TX_NUM * TPSIF_RX_NUM + 1) & 0xff); /* LSB of (TX * RX + 1) */
	} else if (type == 0x01) { /* Self IDAC */
		buf[3] = (uint8_t) ((TPSIF_TX_NUM + TPSIF_RX_NUM + 2) >> 8); /* MSB of (TX + RX + 2) */
		buf[4] = (uint8_t) ((TPSIF_TX_NUM + TPSIF_RX_NUM + 2) & 0xff); /* LSB of (TX + RX + 2) */
	}
	buf[5] = type;

	retval = _cyttsp4_write_block_data(ts, ts->si_ofs.cmd_ofs + 1,
					5, &(buf[1]),
					ts->platform_data->addr[CY_TCH_ADDR_OFS], true);
	if (retval < 0) {
		dev_err(ts->dev, "%s: Failed to write parameters for retrieve command r=%d\n",
			__func__, retval);
		goto _sh_tpsif_retrieve_data_structure_exit;
	}

	/* Write command */
	retval = _cyttsp4_put_cmd_wait(ts, ts->si_ofs.cmd_ofs,
				1, &(buf[0]), CY_HALF_SEC_TMO_MS,
				_cyttsp4_chk_cmd_rdy, NULL,
				ts->platform_data->addr[CY_TCH_ADDR_OFS], true);
	if (retval < 0) {
		dev_err(ts->dev, "%s: Failed to write retrieve command r=%d\n",
			__func__, retval);
		goto _sh_tpsif_retrieve_data_structure_exit;
	}

	retval = _cyttsp4_read_block_data(ts, ts->si_ofs.cmd_ofs + 1,
					sizeof(status), &status,
					ts->platform_data->addr[CY_TCH_ADDR_OFS],
					true);
	if (retval < 0) {
		dev_err(ts->dev, "%s: fail read cmd status r=%d\n", __func__, retval);
		goto _sh_tpsif_retrieve_data_structure_exit;
	}

	if (status != 0x00) {
		dev_err(ts->dev, "%s: Retrieve panel scan command failed"
			" status=%d\n", __func__, status);
		retval = -EIO;
	}

_sh_tpsif_retrieve_data_structure_exit:
	kfree(buf);
	return retval;
}

static int _sh_tpsif_read_idac_data(struct cyttsp4 *ts, int type, struct cyttsp4_idac_data *data)
{
	int retval = 0;
	uint8_t response[6] = {0x00, 0x00, 0x00, 0x00, 0x00, 0x00};

	retval = _cyttsp4_read_block_data(ts, ts->si_ofs.cmd_ofs,
					sizeof(response), &(response[0]),
					ts->platform_data->addr[CY_TCH_ADDR_OFS],
					true);
	if (retval < 0) {
		dev_err(ts->dev, "%s: fail read cmd status"
			" r=%d\n", __func__, retval);
		goto _sh_tpsif_read_idac_data_exit;
	}

	if (type == 0x00) {	/* Mutual IDAC */
		data->data_num = TPSIF_TX_NUM * TPSIF_RX_NUM + 1;
		data->data_size = 1;
		data->scan_type = TPSIF_SCAN_TYPE_IDAC_MUTUAL;
	} else if (type == 0x01) { /* Self IDAC */
		data->data_num = TPSIF_TX_NUM + TPSIF_RX_NUM + 2;
		data->data_size = 1;
		data->scan_type = TPSIF_SCAN_TYPE_IDAC_SELF;
	}

	data->buf_size = sizeof(u8) * data->data_size * data->data_num;

	retval = _cyttsp4_read_block_data(ts, ts->si_ofs.cmd_ofs + 6,
					data->buf_size, data->buf,
					ts->platform_data->addr[CY_TCH_ADDR_OFS],
					true);
	if (retval < 0) {
		dev_err(ts->dev, "%s: fail read scan result"
			" r=%d\n", __func__, retval);
		goto _sh_tpsif_read_idac_data_exit;
	}

	goto _sh_tpsif_read_idac_data_exit;

_sh_tpsif_read_idac_data_exit:

	return retval;
}

static int sh_tpsif_scan_idac(struct cyttsp4 *ts, int type, struct cyttsp4_idac_data *data)
{
	int retval = 0;

	mutex_lock(&ts->data_lock);

	/* Switch to config mode */
	if (ts->current_mode != CY_MODE_CONFIG) {
		retval = _cyttsp4_set_mode(ts, CY_CONFIG_MODE);
		if (retval < 0) {
			dev_err(ts->dev, "%s: Failed to switch to config mode"
				" for sysinfo regs\n", __func__);
			goto sh_tpsif_scan_idac_exit;
		}
	}

	/* Retrieve data structure */
	retval = _sh_tpsif_retrieve_data_structure(ts, type);
	if (retval < 0) {
		dev_err(ts->dev, "%s: Fail retrieve data structure r=%d\n", __func__, retval);
		sh_tpsif_polling_loop = 0;
		goto sh_tpsif_scan_idac_exit;
	}

	/* Read IDAC data */
	retval = _sh_tpsif_read_idac_data(ts, type, data);
	if (retval < 0) {
		dev_err(ts->dev, "%s: Fail read idac data r=%d\n", __func__, retval);
		sh_tpsif_polling_loop = 0;
		goto sh_tpsif_scan_idac_exit;
	}

sh_tpsif_scan_idac_exit:

	mutex_unlock(&ts->data_lock);

	return retval;
}

static int _sh_tpsif_calibration(struct cyttsp4 *ts, u8 mode)
{
	int retval = 0;
	uint8_t *buf = NULL;
	size_t buf_size;
	u8 status;

	buf_size = sizeof(uint8_t) * 4;
	buf = kzalloc(buf_size, GFP_KERNEL);
	if (buf == NULL) {
		dev_err(ts->dev, "%s: Failed to allocate buffer\n", __func__);
		retval = -ENOMEM;
		goto _sh_tpsif_calibration_exit;
	}

	/* Set command bytes */
	buf[0] = 0x09; /* Command */
	buf[1] = mode; /* Mode */
	buf[2] = 0x00; /* Force / Gain */
	buf[3] = 0x00; /* Global IDAC */

	retval = _cyttsp4_write_block_data(ts, ts->si_ofs.cmd_ofs + 1,
					3, &(buf[1]),
					ts->platform_data->addr[CY_TCH_ADDR_OFS], true);
	if (retval < 0) {
		dev_err(ts->dev, "%s: Failed to write calibration parameters r=%d\n",
			__func__, retval);
		goto _sh_tpsif_calibration_exit;
	}

	/* Write command */
	retval = _cyttsp4_put_cmd_wait(ts, ts->si_ofs.cmd_ofs,
				1, &(buf[0]), CY_TEN_SEC_TMO_MS,
				_cyttsp4_chk_cmd_rdy, NULL,
				ts->platform_data->addr[CY_TCH_ADDR_OFS], true);
	if (retval < 0) {
		dev_err(ts->dev, "%s: Failed to write calibration command r=%d\n",
			__func__, retval);
		goto _sh_tpsif_calibration_exit;
	}

	retval = _cyttsp4_read_block_data(ts, ts->si_ofs.cmd_ofs + 1,
					sizeof(status), &status,
					ts->platform_data->addr[CY_TCH_ADDR_OFS],
					true);
	if (retval < 0) {
		dev_err(ts->dev, "%s: fail read cmd status r=%d\n", __func__, retval);
		goto _sh_tpsif_calibration_exit;
	}

	if (status != 0x00) {
		dev_err(ts->dev, "%s: Calibration command failed"
			" status=%d\n", __func__, status);
		retval = -EIO;
	}

_sh_tpsif_calibration_exit:
	kfree(buf);
	return retval;
}

static int _sh_tpsif_calibration_idac(struct cyttsp4 *ts)
{
	int retval = 0;

	if (ts->driver_state != CY_ACTIVE_STATE) {
		retval = -EINVAL;
		dev_err(ts->dev, "%s: Cannot execute calibration"
			" in current state\n", __func__);
		goto _sh_tpsif_calibration_idac_exit;
	}

	retval = _cyttsp4_set_mode(ts, CY_CONFIG_MODE);
	if (retval < 0) {
		dev_err(ts->dev, "%s: Failed to switch to config mode"
			" for sysinfo regs\n", __func__);
		goto _sh_tpsif_calibration_idac_exit;
	}

	retval = _sh_tpsif_calibration(ts, TPSIF_CALIB_MODE_MUTUAL_FINE);
	if (retval < 0) {
		dev_err(ts->dev, "%s: Fail mutal calibration r=%d\n", __func__, retval);
		goto _sh_tpsif_calibration_idac_op_mode;
	}

	retval = _sh_tpsif_calibration(ts, TPSIF_CALIB_MODE_SELF);
	if (retval < 0) {
		dev_err(ts->dev, "%s: Fail self calibration r=%d\n", __func__, retval);
		goto _sh_tpsif_calibration_idac_op_mode;
	}

_sh_tpsif_calibration_idac_op_mode:

	retval = _cyttsp4_set_mode(ts, CY_OPERATE_MODE);
	if (retval < 0) {
		_cyttsp4_change_state(ts, CY_IDLE_STATE);
		dev_err(ts->dev, "%s: Fail set operational mode (r=%d)\n", __func__, retval);
	}

_sh_tpsif_calibration_idac_exit:

	return retval;
}

static int _sh_tpsif_firmware_version(struct cyttsp4 *ts, u32 *version)
{
	int retval = 0;
	u8 ver_major, ver_minor;
	u8 config_verh, config_verl;

	if (ts->sysinfo_ptr.cydata == NULL || ts->sysinfo_ptr.pcfg == NULL) {
		retval = -EINVAL;
		goto _sh_tpsif_firmware_version_exit;
	}

	ver_major = ts->sysinfo_ptr.cydata->fw_ver_major;
	ver_minor = ts->sysinfo_ptr.cydata->fw_ver_minor;

	if (ver_major == 0x00 && ver_minor == 0x24) {
		/* ES10.2 */
		config_verh = ts->sysinfo_ptr.pcfg->len_xh;
		config_verl = ts->sysinfo_ptr.pcfg->len_xl;
	} else {
		/* ES100 */
		config_verh = ts->sysinfo_ptr.cydata->cyito_verh;
		config_verl = ts->sysinfo_ptr.cydata->cyito_verl;
	}

	*version = (ver_major << 24) | (ver_minor << 16)
		| (config_verh << 8) | (config_verl << 0);

_sh_tpsif_firmware_version_exit:

	return retval;
}

static int _sh_tpsif_firmware_update(struct cyttsp4 *ts, u8 *data, size_t size)
{
	int retval = 0;
	u8 header_size = 0;

	mutex_lock(&ts->data_lock);

	if ((data == NULL) || (size == 0)) {
		dev_err(ts->dev, "%s: No firmware received\n", __func__);
		goto _sh_tpsif_firmware_update_exit;
	}

	header_size = data[0];
	if (header_size >= (size + 1)) {
		dev_err(ts->dev, "%s: Firmware format is invalid\n", __func__);
		goto _sh_tpsif_firmware_update_exit;
	}

	retval = _cyttsp4_load_app(ts, &(data[header_size + 1]),
		size - (header_size + 1));
	if (retval < 0) {
		dev_err(ts->dev,
			"%s: Firmware update failed with error code %d\n",
			__func__, retval);
		_cyttsp4_change_state(ts, CY_IDLE_STATE);
		retval = -EIO;

		/* restart */
		_cyttsp4_startup(ts);

		goto _sh_tpsif_firmware_update_exit;
	}

#ifdef CONFIG_TOUCHSCREEN_DEBUG
	ts->debug_upgrade = true;
#endif /* --CONFIG_TOUCHSCREEN_DEBUG */

	retval = _cyttsp4_startup(ts);
	if (retval < 0) {
		dev_err(ts->dev,
			"%s: Failed to restart IC with error code %d\n",
			__func__, retval);
		_cyttsp4_change_state(ts, CY_IDLE_STATE);
	}

#ifdef CALIBRATION_AFTER_FIRMWARE_UPDATE
	dev_info(ts->dev, "%s: calibration after firmware update\n", __func__);
	retval = _sh_tpsif_calibration_idac(ts);
	if (retval < 0)
		dev_err(ts->dev, "%s: fail calibration\n", __func__);
#ifdef RESET_AFTER_CALIBRATION
	_cyttsp4_startup(ts);
#endif	/* RESET_AFTER_CALIBRATION */
#endif	/* CALIBRATION_AFTER_FIRMWARE_UPDATE */

_sh_tpsif_firmware_update_exit:
	mutex_unlock(&ts->data_lock);
	return retval;
}

static int sh_tpsif_start_firmware_update(struct cyttsp4 *ts, u8 *data, size_t size)
{
	int retval = -EFAULT;
	enum cyttsp4_driver_state tmp_state;

	if (ts->waiting_for_fw) {
		dev_err(ts->dev, "%s: Driver is already waiting for firmware\n",
			__func__);
		retval = -EALREADY;
		goto sh_tpsif_start_firmware_update_exit;
	}

	if (ts->driver_state != CY_IDLE_STATE &&
		ts->driver_state != CY_ACTIVE_STATE &&
		ts->driver_state != CY_SLEEP_STATE &&
		ts->driver_state != CY_BL_STATE) {
		dev_err(ts->dev, "%s: Cannot update firmware"
			" in current state\n", __func__);
		goto sh_tpsif_start_firmware_update_exit;
	}

	tmp_state = ts->driver_state;

	if (tmp_state == CY_SLEEP_STATE) {
#if defined(CONFIG_HAS_EARLYSUSPEND)
		cyttsp4_late_resume(&ts->early_suspend);
		retval = 0;
#elif defined(CONFIG_PM_SLEEP)
		retval = cyttsp4_resume(ts->dev);
#elif defined(CONFIG_PM)
		retval = cyttsp4_resume(ts);
#endif
		if (retval < 0) {
			dev_err(ts->dev, "%s: Fail resume\n", __func__);
			goto sh_tpsif_start_firmware_update_exit;
		}
	}

	dev_dbg(ts->dev,
		"%s: Enabling firmware class loader\n", __func__);

	ts->waiting_for_fw = true;
	retval = _sh_tpsif_firmware_update(ts, data, size);
	ts->waiting_for_fw = false;
	if (retval < 0) {
		dev_err(ts->dev, "%s: Fail request firmware class file load\n",
			__func__);
	} else {
		retval = size;
	}

	if (tmp_state == CY_SLEEP_STATE) {
#if defined(CONFIG_HAS_EARLYSUSPEND)
		cyttsp4_early_suspend(&ts->early_suspend);
#elif defined(CONFIG_PM_SLEEP)
		cyttsp4_suspend(ts->dev);
#elif defined(CONFIG_PM) || defined(CONFIG_HAS_EARLYSUSPEND)
		cyttsp4_suspend(ts);
#endif
	}

sh_tpsif_start_firmware_update_exit:
	return retval;
}

static int _sh_tpsif_poll_start(struct cyttsp4 *ts)
{
	if (ts->driver_state != CY_ACTIVE_STATE)
		return -EINVAL;

	hrtimer_cancel(&sh_tpsif_polling_timer);
	hrtimer_start(&sh_tpsif_polling_timer, ktime_set(0, SH_TPSIF_POLL_INTERVAL * 1000 * 1000), HRTIMER_MODE_REL);

	sh_tpsif_polling_loop = 1;

	return 0;
}

static void _sh_tpsif_poll_stop(struct cyttsp4 *ts)
{
	sh_tpsif_polling_loop = 0;

	hrtimer_try_to_cancel(&sh_tpsif_polling_timer);
}

static void _sh_tpsif_poll_scan(struct work_struct *work)
{
	//struct cyttsp4 *ts = container_of(work, struct cyttsp4, sh_tpsif_polling_work);
	struct cyttsp4 *ts = g_tpsif_ts;

	if (sh_tpsif_scan_type == TPSIF_SCAN_TYPE_MUTUAL_RAW_AND_BASE) {
		sh_tpsif_scan_rawdata(ts, TPSIF_SCAN_TYPE_MUTUAL_RAW, &sh_tpsif_scan_data[0]);
		sh_tpsif_scan_rawdata(ts, TPSIF_SCAN_TYPE_MUTUAL_BASE, &sh_tpsif_scan_data[1]);
	} else if (sh_tpsif_scan_type == TPSIF_SCAN_TYPE_SELF_RAW_AND_BASE) {
		sh_tpsif_scan_rawdata(ts, TPSIF_SCAN_TYPE_SELF_RAW, &sh_tpsif_scan_data[0]);
		sh_tpsif_scan_rawdata(ts, TPSIF_SCAN_TYPE_SELF_BASE, &sh_tpsif_scan_data[1]);
	} else if (sh_tpsif_scan_type == TPSIF_SCAN_TYPE_IDAC_MUTUAL) {
		sh_tpsif_scan_idac(ts, 0x00, &sh_tpsif_idac_data);
	} else if (sh_tpsif_scan_type == TPSIF_SCAN_TYPE_IDAC_SELF) {
		sh_tpsif_scan_idac(ts, 0x01, &sh_tpsif_idac_data);
	} else
		sh_tpsif_scan_rawdata(ts, sh_tpsif_scan_type, &sh_tpsif_scan_data[0]);

	sh_tpsif_event = 1;
	wake_up_interruptible(&sh_tpsif_wq);

	if (sh_tpsif_polling_loop)
		_sh_tpsif_poll_start(ts);
}

static enum hrtimer_restart _sh_tpsif_poll_timer_handler(struct hrtimer *timer)
{
	schedule_work(&sh_tpsif_polling_work);

	return HRTIMER_NORESTART;
}

#if defined(SH_TPSIF_IOCTL_DISABLE)
int sh_tpsif_command(struct cyttsp4 *ts, unsigned int cmd, unsigned long arg)
#else
static int sh_tpsif_command(struct cyttsp4 *ts, unsigned int cmd, unsigned long arg)
#endif /* SH_TPSIF_IOCTL_DISABLE */
{
	int retval = 0;
	void __user *argp = (void __user *)arg;
	u8 *fw;
	static u16 addr = 0x00;
	u8 data;
	u32 version;
	int mode;
	int scan_type;
	u16 param[AREA_COUNT];

	switch (cmd) {
	case TPSIF_POWER_ON:
		retval = sh_tpsif_hw_power(ts, SH_TPSIF_HW_POWER_ON);
		break;
	case TPSIF_POWER_OFF:
		retval = sh_tpsif_hw_power(ts, SH_TPSIF_HW_POWER_OFF);
		break;
	case TPSIF_HWRESET:
		retval = sh_tpsif_hw_reset(ts);
		break;
	case TPSIF_SWRESET:
		retval = sh_tpsif_sw_reset(ts);
		break;
	case TPSIF_SLEEP_ON:
#if defined(CONFIG_HAS_EARLYSUSPEND)
		cyttsp4_early_suspend(&ts->early_suspend);
		retval = 0;
#elif defined(CONFIG_PM_SLEEP)
		retval = cyttsp4_suspend(ts->dev);
#elif defined(CONFIG_PM) || defined(CONFIG_HAS_EARLYSUSPEND)
		retval = cyttsp4_suspend(ts);
#endif
		break;
	case TPSIF_SLEEP_OFF:
#if defined(CONFIG_HAS_EARLYSUSPEND)
		cyttsp4_late_resume(&ts->early_suspend);
		retval = 0;
#elif defined(CONFIG_PM_SLEEP)
		retval = cyttsp4_resume(ts->dev);
#elif defined(CONFIG_PM)
		retval = cyttsp4_resume(ts);
#endif
		break;
	case TPSIF_FW_VERSION:
		retval = _sh_tpsif_firmware_version(ts, &version);
		if (retval < 0)
			dev_err(ts->dev, "%s: fail get version\n", __func__);
		if (copy_to_user((u32 __user *)argp, &version, sizeof(version))) {
			return -EFAULT;
		}
		break;
	case TPSIF_FW_UPDATE:
		fw = kzalloc(TPSIF_FW_IMG_SIZE, GFP_KERNEL);
		if (fw  == NULL) {
			dev_err(ts->dev, "%s: fail allocate memory for firmware\n", __func__);
			return -ENOMEM;
		}
		if (copy_from_user(fw, (u8 __user *)argp, TPSIF_FW_IMG_SIZE)) {
			kfree(fw);
			return -EFAULT;
		}
		retval = sh_tpsif_start_firmware_update(ts, fw, TPSIF_FW_IMG_SIZE);
		kfree(fw);
		break;
	case TPSIF_DRV_STAT:
		/* TODO */
		break;
	case TPSIF_DRV_VER:
		/* TODO */
		break;
	case TPSIF_IRQ_STAT:
		/* TODO */
		break;
	case TPSIF_CALIBRATION_IDAC:
		mutex_lock(&ts->data_lock);
		retval = _sh_tpsif_calibration_idac(ts);
#ifdef RESET_AFTER_CALIBRATION
		_cyttsp4_startup(ts);
#endif	/* RESET_AFTER_CALIBRATION */
		mutex_unlock(&ts->data_lock);
		if (retval < 0)
			dev_err(ts->dev, "%s: fail calibration\n", __func__);
		break;
	case TPSIF_SET_REG_ADDR:
		if (copy_from_user(&addr, (u16 __user *)argp, sizeof(addr))) {
			return -EFAULT;
		}
		break;
	case TPSIF_READ_REG:
		retval = _cyttsp4_read_block_data(ts, addr, sizeof(data), &data,
						ts->platform_data->addr[CY_TCH_ADDR_OFS], false);
		if (retval < 0)
			dev_err(ts->dev, "%s: fail read register %04x\n", __func__, addr);
		if (copy_to_user((u8 __user *)argp, &data, sizeof(data))) {
			return -EFAULT;
		}
		break;
	case TPSIF_WRITE_REG:
		if (copy_from_user(&data, (u8 __user *)argp, sizeof(data))) {
			return -EFAULT;
		}
		retval = _cyttsp4_write_block_data(ts, addr, sizeof(data), &data,
						ts->platform_data->addr[CY_TCH_ADDR_OFS], false);
		if (retval < 0)
			dev_err(ts->dev, "%s: fail write register %04x\n", __func__, addr);
		break;
	case TPSIF_SWITCH_MODE:
		if (copy_from_user(&mode, (u8 __user *)argp, sizeof(mode))) {
			return -EFAULT;
		}
		_sh_tpsif_poll_stop(ts);
		retval = sh_tpsif_change_mode(ts, mode);
		break;
	case TPSIF_START_SCAN:
		if (copy_from_user(&scan_type, (int __user *)argp, sizeof(scan_type)))
			return -EFAULT;
		if (scan_type < 0 || scan_type >= TPSIF_SCAN_TYPE_NUM)
			return -EFAULT;
		_sh_tpsif_poll_stop(ts);
		sh_tpsif_scan_type = scan_type;
		retval = _sh_tpsif_poll_start(ts);
		break;
	case TPSIF_HWRESET_STARTUP:
		retval = sh_tpsif_hw_reset_startup(ts);
		break;
	case TPSIF_SWRESET_STARTUP:
		retval = sh_tpsif_sw_reset_startup(ts);
		break;
	case TPSIF_CALIBRATION_PARAM:
		if (argp == NULL) {
			retval = sh_tpsif_set_adjust_param(ts, NULL);
		} else {
			if (copy_from_user(param, (u16 __user *)argp, sizeof(param))) {
				return -EFAULT;
			}
			retval = sh_tpsif_set_adjust_param(ts, param);
		}
		break;
	default:
		return -ENOTTY;
	}
	return retval;
}

static int sh_tpsif_open(struct inode *inode, struct file *filp)
{
	return 0;
}

static int sh_tpsif_release(struct inode *inode, struct file *filp)
{
	struct cyttsp4 *ts = g_tpsif_ts;

	sh_tpsif_event = 0;
	_sh_tpsif_poll_stop(ts);

	return 0;
}

static long sh_tpsif_ioctl(struct file *filp, unsigned int cmd, unsigned long arg)
{
	#if defined(SH_TPSIF_IOCTL_DISABLE)
		return 0;
	#else
		struct cyttsp4 *ts = g_tpsif_ts;

		return (long) sh_tpsif_command(ts, cmd, arg);
	#endif /* SH_TPSIF_IOCTL_DISABLE */
}

static ssize_t sh_tpsif_read(struct file *filp, char *buf, size_t count, loff_t *pos)
{
	struct cyttsp4_scan_data scan_data[2];
	struct cyttsp4_idac_data idac_data;
	ssize_t size;

	wait_event_interruptible(sh_tpsif_wq, sh_tpsif_event == 1);

	if (sh_tpsif_scan_type == TPSIF_SCAN_TYPE_IDAC_MUTUAL ||
		sh_tpsif_scan_type == TPSIF_SCAN_TYPE_IDAC_SELF) {
		memcpy(&idac_data, &sh_tpsif_idac_data, sizeof(idac_data));

		if (copy_to_user((u8 __user *)buf, &idac_data, sizeof(idac_data)))
			return -EFAULT;

		size = sizeof(idac_data);
	} else if (sh_tpsif_scan_type == TPSIF_SCAN_TYPE_MUTUAL_RAW_AND_BASE ||
		sh_tpsif_scan_type == TPSIF_SCAN_TYPE_SELF_RAW_AND_BASE) {
		memcpy(&scan_data, &sh_tpsif_scan_data, sizeof(scan_data));

		if (copy_to_user((u8 __user *)buf, &scan_data, sizeof(scan_data)))
			return -EFAULT;

		size = sizeof(scan_data);
	} else {
		memcpy(&scan_data[0], &sh_tpsif_scan_data[0], sizeof(scan_data[0]));

		if (copy_to_user((u8 __user *)buf, &scan_data[0], sizeof(scan_data[0])))
			return -EFAULT;

		size = sizeof(scan_data[0]);
	}

	sh_tpsif_event = 0;

	return size;
}

static unsigned int sh_tpsif_poll(struct file *filp, poll_table *wait)
{
	int retval;

	retval = wait_event_interruptible_timeout(sh_tpsif_wq,
						sh_tpsif_event == 1,
						msecs_to_jiffies(SH_TPSIF_POLL_INTERVAL));

	if(retval)
		return POLLIN | POLLRDNORM;

	return 0;
}

static const struct file_operations sh_tpsif_fops =
{
	.owner		= THIS_MODULE,
	.open		= sh_tpsif_open,
	.release	= sh_tpsif_release,
	.read		= sh_tpsif_read,
	.unlocked_ioctl = sh_tpsif_ioctl,
	.poll		= sh_tpsif_poll,
};

#if defined(SH_TPSIF_LOCAL_DEVICE_REGIST_ENABLE)
static void sh_tpsif_device_attach(void)
{
	struct spi_master *master_p = NULL;
	struct spi_device *device_p = NULL;
	struct spi_board_info spi_info;

	master_p = spi_busnum_to_master(SH_TPSIF_SPI_BUS_NUM);

	if(master_p != NULL){
		strcpy(spi_info.modalias, CY_SPI_NAME);
		spi_info.irq = MSM_GPIO_TO_INT(TOUCH_GPIO_IRQ_CYTTSP);
		spi_info.max_speed_hz = 1100000;
		spi_info.bus_num = SH_TPSIF_SPI_BUS_NUM;
		spi_info.chip_select = 0;
		spi_info.mode = SPI_MODE_0;
		spi_info.platform_data = &cyttsp4_spi_touch_platform_data_local;

		device_p = spi_new_device(master_p, &spi_info);
		if(device_p == NULL){
			printk(KERN_DEBUG "[shtps][%s]device regist error\n", __func__);
		}
	}else{
		printk(KERN_DEBUG "[shtps][%s]master search error\n", __func__);
	}
}

#if 0
static void sh_tpsif_device_detach(void)
{
	spi_unregister_device();
}
#endif

#endif /* SH_TPSIF_LOCAL_DEVICE_REGIST_ENABLE */

int __init sh_tpsif_init(void)
{
	int ret;
	dev_t major = 0;
	dev_t minor = 0;

	ret = alloc_chrdev_region(&sh_tpsif_dev, 0, 1, TPSIF_DEV_NAME);
	if (!ret) {
		major = MAJOR(sh_tpsif_dev);
		minor = MINOR(sh_tpsif_dev);
	} else
		goto sh_tpsif_err_exit;

	cdev_init(&sh_tpsif_cdev, &sh_tpsif_fops);

	sh_tpsif_cdev.owner = THIS_MODULE;
	sh_tpsif_cdev.ops = &sh_tpsif_fops;

	ret = cdev_add(&sh_tpsif_cdev, sh_tpsif_dev, 1);
	if (ret)
		goto sh_tpsif_err_add;

	sh_tpsif_class = class_create(THIS_MODULE, TPSIF_DEV_NAME);
	if (IS_ERR(sh_tpsif_class))
		goto sh_tpsif_err_add;

	device_create(sh_tpsif_class, NULL,
		sh_tpsif_dev, &sh_tpsif_cdev, TPSIF_DEV_NAME);

	#if defined(SH_TPSIF_LOCAL_DEVICE_REGIST_ENABLE)
		sh_tpsif_device_attach();
	#endif /* SH_TPSIF_LOCAL_DEVICE_REGIST_ENABLE */

	return 0;

sh_tpsif_err_add:
	cdev_del(&sh_tpsif_cdev);

sh_tpsif_err_exit:
	return -1;
}
module_init(sh_tpsif_init);

void __exit sh_tpsif_exit(void)
{
	device_destroy(sh_tpsif_class, sh_tpsif_dev);
	class_destroy(sh_tpsif_class);
	cdev_del(&sh_tpsif_cdev);

	return;
}
module_exit(sh_tpsif_exit);

#endif	/* SH_TPSIF_COMMAND */

void msm_tps_setsleep(int on)
{
	return;
}
EXPORT_SYMBOL(msm_tps_setsleep);
