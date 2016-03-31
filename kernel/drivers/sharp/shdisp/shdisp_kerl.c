/* driver/sharp/shdisp/shdisp_kerl.c  (Display Driver)
 *
 * Copyright (C) 2011-2012 SHARP CORPORATION
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

/* ------------------------------------------------------------------------- */
/* SHARP DISPLAY DRIVER FOR KERNEL MODE                                      */
/* ------------------------------------------------------------------------- */

/* ------------------------------------------------------------------------- */
/* INCLUDE FILES                                                             */
/* ------------------------------------------------------------------------- */

#include <linux/compiler.h>
#include <linux/types.h>
#include <linux/kernel.h>
#include <linux/module.h>
#include <linux/fs.h>
#include <linux/platform_device.h>
#include <linux/device.h>
#include <linux/cdev.h>
#include <linux/interrupt.h>
#include <linux/init.h>
#include <linux/poll.h>
#include <linux/mm.h>
#include <linux/idr.h>
#include <linux/kobject.h>
#include <linux/string.h>
#include <linux/ioctl.h>
#include <linux/gpio.h>
#include <linux/hrtimer.h>
#include <linux/irq.h>
#include <linux/i2c.h>
#include <linux/input.h>
#include <linux/slab.h>
#include <linux/delay.h>
#include <linux/proc_fs.h>
#include <linux/fb.h>
#ifdef CONFIG_TOUCHSCREEN_SHTPS
#include <sharp/shtps_dev.h>
#endif /* CONFIG_TOUCHSCREEN_SHTPS */
#include <linux/sched.h>
#include <linux/timer.h>
#include <linux/wakelock.h>

#ifdef CONFIG_SHTERM
#include <sharp/shterm_k.h>
#endif /* CONFIG_SHTERM */
#include <sharp/sh_smem.h>
#include <sharp/shdisp_kerl.h>
#include "shdisp_system.h"
#include "shdisp_type.h"
#include "shdisp_pwm.h"

#include "shdisp_bl69y6.h"

#if defined(CONFIG_SHDISP_PANEL_PHARAOH)
#include "shdisp_pharaoh.h"
#endif /* CONFIG_SHDISP_PANEL_PHARAOH */

#if defined(CONFIG_SHDISP_PANEL_NICOLE)
#include "shdisp_nicole.h"
#endif /* CONFIG_SHDISP_PANEL_NICOLE */

#if defined(CONFIG_SHDISP_PANEL_TAKT)
#include "shdisp_takt.h"
#endif /* CONFIG_SHDISP_PANEL_TAKT */

#if defined(CONFIG_SHDISP_PANEL_KITE)
#include "shdisp_kite.h"
#endif /* CONFIG_SHDISP_PANEL_KITE */

#if defined(CONFIG_SHDISP_PANEL_RYOMA)
#include "shdisp_ryoma.h"
#endif /* CONFIG_SHDISP_PANEL_RYOMA */

#if defined(CONFIG_SHDISP_PANEL_FLUTE)
#include "shdisp_flute.h"
#endif /* CONFIG_SHDISP_PANEL_FLUTE */

#ifdef CONFIG_SHLCDC_LED_BD2802GU
#include "shdisp_bd2802gu.h"
#endif

#include <asm/io.h>
#include <asm/pgtable.h>
#include <asm/delay.h>
#include <asm/param.h>

#include "../../video/msm/mipi_sharp.h"

#include "../../video/msm/msm_fb.h"

/* ------------------------------------------------------------------------- */
/* MACRAOS                                                                   */
/* ------------------------------------------------------------------------- */

#define SHDISP_NAME "shdisp"

#define SHDISP_BOOT_MODE_NORMAL             0xFFFF
#define SHDISP_BOOT_MODE_OFF_CHARGE         0x0020
#define SHDISP_BOOT_MODE_USB_OFFCHARGE      0x0021

#define SHDISP_ALS_IRQ_REQ_BK_CTL           0x01
#define SHDISP_ALS_IRQ_REQ_DBC              0x02

/* ------------------------------------------------------------------------- */
/* VARIABLES                                                                 */
/* ------------------------------------------------------------------------- */

static dev_t shdisp_dev;
static dev_t shdisp_major = 0;
static dev_t shdisp_minor = 0;
static int shdisp_driver_is_initialized = 0;
static struct cdev shdisp_cdev;
static struct class *shdisp_class;
static struct shdisp_kernel_context shdisp_kerl_ctx;
static struct shdisp_boot_context shdisp_boot_ctx;
static struct semaphore shdisp_sem;
static struct semaphore shdisp_sem_callback;
static struct semaphore shdisp_sem_irq_fac;
static struct semaphore shdisp_sem_timer;
static struct shdisp_panel_operations *shdisp_panel_fops = NULL;
static struct timer_list shdisp_timer;
static int shdisp_timer_stop = 1;
static struct semaphore shdisp_lux_change_sem;
static int    lux_change_wait_flg = SHDISP_LUX_CHANGE_STATE_INIT;
static struct completion lux_change_notify;
static sharp_smem_common_type *sh_smem_common = NULL;

static struct shdisp_panel_operations shdisp_def_fops = {
    .init_io          = NULL,
    .exit_io          = NULL,
    .init_isr         = NULL,
    .set_param        = NULL,
    .power_on         = NULL,
    .power_off        = NULL,
    .init_1st         = NULL,
    .init_2nd         = NULL,
    .disp_on          = NULL,
    .sleep            = NULL,
    .deep_standby     = NULL,
    .check_upper_unit = NULL,
    .check_flicker    = NULL,
    .write_reg        = NULL,
    .read_reg         = NULL,
    .set_flicker      = NULL,
    .get_flicker      = NULL,
    .check_recovery   = NULL,
    .recovery_type    = NULL,
    .set_abl_lut      = NULL,
    .disp_update      = NULL,
    .clear_screen     = NULL,
    .get_flicker_low  = NULL,
};

static int shdisp_subscribe_type_table[NUM_SHDISP_IRQ_TYPE] = {
    SHDISP_SUBSCRIBE_TYPE_INT,
    SHDISP_SUBSCRIBE_TYPE_INT,
    SHDISP_SUBSCRIBE_TYPE_INT
};

static void (*shdisp_callback_table[NUM_SHDISP_IRQ_TYPE])(void) = {
    NULL,
    NULL,
    NULL
};

static spinlock_t                 shdisp_q_lock;
static struct shdisp_queue_data_t shdisp_queue_data;

static struct workqueue_struct    *shdisp_wq_gpio;
static struct work_struct         shdisp_wq_gpio_wk;

static struct workqueue_struct    *shdisp_wq_gpio_task;
static struct work_struct         shdisp_wq_gpio_task_wk;

static struct workqueue_struct    *shdisp_wq_timer_task;
static struct work_struct         shdisp_wq_timer_task_wk;

static int shdisp_smem_read_flag = 0;

static unsigned char shdisp_als_irq_req_state = 0;
static unsigned char shdisp_als_irq_subscribe_type = 0;


static struct       wake_lock shdisp_wake_lock_wq;
static int          shdisp_wake_lock_wq_refcnt;

static spinlock_t   shdisp_wake_spinlock;

static struct workqueue_struct    *shdisp_wq_sensor_start;
static struct delayed_work        shdisp_sensor_start_wk;

static int  shdisp_wait_sensor_start_time = SHDISP_OPT_CHANGE_WAIT_TIME;
static int  shdisp_new_opt_mode = SHDISP_LUX_MODE_LOW;

static int  shdisp_photo_sensor_api_first = 0;

extern int fps_low_mode;
extern int base_fps_low_mode;

/* ------------------------------------------------------------------------- */
/* PROTOTYPES                                                                */
/* ------------------------------------------------------------------------- */

static void shdisp_init_context(void);
static void shdisp_get_boot_context(void);
static void shdisp_context_initialize(int mode);

static int shdisp_check_initialized(void);
static int shdisp_check_upper_unit(void);
static int shdisp_set_upper_unit(int mode);
static int shdisp_check_bdic_exist(void);
static int shdisp_check_panel_info(void);
static int shdisp_check_clock_info(void);
static int shdisp_check_clock_sw(void);
static int shdisp_get_boot_disp_status(void);
static unsigned short shdisp_get_hw_revision(void);
static unsigned short shdisp_get_alpha(void);
static unsigned short shdisp_get_alpha_low(void);
static struct shdisp_lcddr_phy_gamma_reg* shdisp_get_lcddr_phy_gamma(void);
static int shdisp_bdic_subscribe_check(struct shdisp_subscribe *subscribe);
static int shdisp_bdic_unsubscribe_check(int irq_type);

static int shdisp_open(struct inode *inode, struct file *filp);
static int shdisp_write(struct file *filp, const char __user *buf,size_t count, loff_t *ppos);
static int shdisp_read(struct file *filp, char __user *buf,size_t count, loff_t *ppos);
static int shdisp_mmap(struct file *filp, struct vm_area_struct *vma);
static long shdisp_ioctl(struct file *filp, unsigned int cmd, unsigned long arg);
static int shdisp_release(struct inode *inode, struct file *filp);

static int shdisp_ioctl_get_context(void __user *argp);
static int shdisp_ioctl_set_host_gpio(void __user *argp);
static int shdisp_ioctl_tri_led_set_color(void __user *argp);
static int shdisp_ioctl_bdic_write_reg(void __user *argp);
static int shdisp_ioctl_bdic_read_reg(void __user *argp);
static int shdisp_ioctl_bdic_multi_read_reg(void __user *argp);
static int shdisp_ioctl_get_lux(void __user *argp);
static int shdisp_ioctl_pwm_config(void __user *argp);
static int shdisp_ioctl_pwm_stop(void);
static int shdisp_ioctl_photo_sensor_pow_ctl(void __user *argp);
static int shdisp_ioctl_lcddr_write_reg(void __user *argp);
static int shdisp_ioctl_lcddr_read_reg(void __user *argp);
static int shdisp_ioctl_set_flicker_param(void __user *argp);
static int shdisp_ioctl_get_flicker_param(void __user *argp);
static int shdisp_ioctl_get_flicker_low_param(void __user *argp);
static int shdisp_ioctl_bkl_set_auto_mode(void __user *argp);
static int shdisp_ioctl_bkl_set_dtv_mode(void __user *argp);
static int shdisp_ioctl_bkl_set_emg_mode(void __user *argp);
static int shdisp_ioctl_ledc_power_on(void);
static int shdisp_ioctl_ledc_power_off(void);
static int shdisp_ioctl_ledc_set_rgb(void __user *argp);
static int shdisp_ioctl_ledc_set_color(void __user *argp);
static int shdisp_ioctl_ledc_write_reg(void __user *argp);
static int shdisp_ioctl_ledc_read_reg(void __user *argp);
static int shdisp_ioctl_lux_change_ind(void __user *argp);
static int shdisp_ioctl_bkl_set_eco_mode(void __user *argp);
static int shdisp_ioctl_set_cabc(void __user *argp);
static int shdisp_ioctl_set_cabc_update_param(void __user *argp);
static int shdisp_ioctl_bkl_set_chg_mode(void __user *argp);

static int shdisp_SQE_main_lcd_power_on(void);
static int shdisp_SQE_main_lcd_reset(void);
static int shdisp_SQE_main_lcd_power_off(void);
static int shdisp_SQE_main_disp_init_1st(void);
static int shdisp_SQE_main_disp_init_2nd(void);
static int shdisp_SQE_main_disp_on(void);
static int shdisp_SQE_main_disp_off(void);
static int shdisp_SQE_main_bkl_ctl(int type, struct shdisp_main_bkl_ctl *bkl);
static int shdisp_SQE_main_bkl_set_dtv_mode(int dtv_mode);
static int shdisp_SQE_main_bkl_set_emg_mode(int emg_mode);
static int shdisp_SQE_main_bkl_set_eco_mode(int eco_mode);
static int shdisp_SQE_main_bkl_set_chg_mode(int chg_mode);
static int shdisp_SQE_set_host_gpio(struct shdisp_host_gpio *host_gpio);
static int shdisp_SQE_tri_led_set_color(struct shdisp_tri_led *tri_led);
static int shdisp_SQE_bdic_write_reg(unsigned char reg, unsigned char val);
static int shdisp_SQE_bdic_read_reg(unsigned char reg, unsigned char *val);
static int shdisp_SQE_bdic_multi_read_reg(unsigned char reg, unsigned char *val, int size);
static int shdisp_SQE_get_lux(struct shdisp_photo_sensor_val *value);
static int shdisp_SQE_change_level_lut(void);
static int shdisp_SQE_pwm_config(int duty);
static int shdisp_SQE_pwm_stop(void);
static int shdisp_SQE_write_bdic_i2c(struct shdisp_bdic_i2c_msg *i2c_msg);
static int shdisp_SQE_read_bdic_i2c(struct shdisp_bdic_i2c_msg *i2c_msg);
static int shdisp_SQE_photo_sensor_pow_ctl(struct shdisp_photo_sensor_power_ctl *ctl);
static int shdisp_SQE_panel_write_reg(struct shdisp_lcddr_reg *panel_reg);
static int shdisp_SQE_panel_read_reg(struct shdisp_lcddr_reg *panel_reg);
static int shdisp_SQE_prox_sensor_pow_ctl(int power_mode);
static int shdisp_SQE_set_flicker_param(unsigned short alpha);
static int shdisp_SQE_get_flicker_param(unsigned short *alpha);
static int shdisp_SQE_get_flicker_low_param(unsigned short *alpha);
static int shdisp_SQE_ledc_power_on(void);
static int shdisp_SQE_ledc_power_off(void);
static int shdisp_SQE_ledc_set_rgb(struct shdisp_ledc_rgb *ledc_rgb);
static int shdisp_SQE_ledc_set_color(struct shdisp_ledc_req *ledc_req);
static int shdisp_SQE_ledc_write_reg(unsigned char reg, unsigned char val);
static int shdisp_SQE_ledc_read_reg(unsigned char reg, unsigned char *val);
static int shdisp_SQE_check_recovery(void);
static int shdisp_SQE_do_recovery(void);
static int shdisp_SQE_event_subscribe(struct shdisp_subscribe *subscribe);
static int shdisp_SQE_event_unsubscribe(int irq_type);
static int shdisp_SQE_bdic_i2c_error_handling(void);
static int shdisp_SQE_lux_change_ind(struct shdisp_photo_sensor_val *value);
static int shdisp_SQE_set_cabc(struct shdisp_main_dbc *value);
static int shdisp_SQE_set_cabc_update_param(struct shdisp_main_dbc *value);
static int shdisp_SQE_cabc_ctrl(void);
static int shdisp_SQE_pwm_enable(void);
static int shdisp_SQE_pwm_disable(void);
static int shdisp_SQE_sensor_start_task(void);

static void shdisp_semaphore_start(void);
static void shdisp_semaphore_end(const char *func);

static irqreturn_t shdisp_gpio_int_isr( int irq_num, void *data );
static void shdisp_workqueue_handler_gpio(struct work_struct *work);
static void shdisp_workqueue_gpio_task(struct work_struct *work);
static void shdisp_wake_lock_init(void);
static void shdisp_wake_lock(void);
static void shdisp_wake_unlock(void);
static void shdisp_timer_int_isr(unsigned long data);
static void shdisp_timer_int_register(void);
static void shdisp_timer_int_delete(void);
static void shdisp_timer_int_mod(void);
static void shdisp_workqueue_timer_task(struct work_struct *work);
static void shdisp_photo_lux_irq(void);
static int shdisp_als_irq_subscribe_bkl_ctrl(int mode);
static void shdisp_sensor_start_handler(struct work_struct *work);

#if defined (CONFIG_ANDROID_ENGINEERING)
static int shdisp_proc_write(struct file *file, const char *buffer, unsigned long count, void *data);
static void shdisp_dbg_info_output(int mode);
static void shdisp_dbg_que(int kind);
static void shdisp_debug_subscribe(void);
static void callback_ps(void);
#endif /* CONFIG_ANDROID_ENGINEERING */
static void shdisp_fb_open(void);
static void shdisp_fb_close(void);

static void shdisp_panel_API_create(void);
static int shdisp_panel_API_init_io(void);
static int shdisp_panel_API_exit_io(void);
static int shdisp_panel_API_init_isr(void);
static void shdisp_panel_API_set_param(struct shdisp_panel_param_str *param_str);
static int shdisp_panel_API_power_on(void);
static int shdisp_panel_API_power_off(void);
static int shdisp_panel_API_disp_init_1st(void);
static int shdisp_panel_API_disp_init_2nd(void);
static int shdisp_panel_API_disp_on(void);
static int shdisp_panel_API_sleep(void);
static int shdisp_panel_API_deep_standby(void);
static int shdisp_panel_API_diag_write_reg(unsigned char addr, unsigned char *write_data, unsigned char size);
static int shdisp_panel_API_diag_read_reg(unsigned char addr, unsigned char *read_data, unsigned char size);
static int shdisp_panel_API_diag_set_flicker_param(unsigned short alpha);
static int shdisp_panel_API_diag_get_flicker_param(unsigned short *alpha);
static int shdisp_panel_API_diag_get_flicker_low_param(unsigned short *alpha);
static int shdisp_panel_API_check_recovery(void);
static int shdisp_panel_API_get_recovery_type(int *type);


static struct file_operations shdisp_fops = {
    .owner          = THIS_MODULE,
    .open           = shdisp_open,
    .write          = shdisp_write,
    .read           = shdisp_read,
    .mmap           = shdisp_mmap,
    .unlocked_ioctl = shdisp_ioctl,
    .release        = shdisp_release,
};


/* ------------------------------------------------------------------------- */
/* KERNEL LOG DEBUG MACRAOS(module_param)                                    */
/* ------------------------------------------------------------------------- */

static int debug_cabc_mode = 0;

static int change_cabc_mode = 0;

static int change_cabc_level_lut = 0;

static int cabc_mode_set = 0;

static int change_mode = 0;

static int change_level_lut = 0;

static int shdisp_cabc_current_mode = 0;

static int shdisp_cabc_current_lut = 0;


/* ------------------------------------------------------------------------- */
/* DEBUG MACRAOS                                                             */
/* ------------------------------------------------------------------------- */
//#define SHDISP_SW_DISABLE_SMEM_INFO

#define SHDISP_TRACE(fmt, args...) \
        if((shdisp_kerl_ctx.dbgTraceF & 0x01) == 0x01){ \
            printk("[SHDISP_TRACE] " fmt, ## args); \
        }

#define SHDISP_FILE "shdisp_kerl.c"
#define SHDISP_ERR(fmt, args...) \
        printk("[SHDISP_ERROR][%s][%s] " fmt, SHDISP_FILE, __func__, ## args);

/* ------------------------------------------------------------------------- */
/* FUNCTIONS                                                                 */
/* ------------------------------------------------------------------------- */
/* ------------------------------------------------------------------------- */
/* API                                                                       */
/* ------------------------------------------------------------------------- */
/* ------------------------------------------------------------------------- */
/* shdisp_api_get_fbcon_status                                               */
/* ------------------------------------------------------------------------- */

int shdisp_api_get_fbcon_status(void)
{
    return SHDISP_RESULT_SUCCESS;
}


/* ------------------------------------------------------------------------- */
/* shdisp_api_main_lcd_power_on                                              */
/* ------------------------------------------------------------------------- */

int shdisp_api_main_lcd_power_on(void)
{
    int ret;
    
    if (shdisp_check_initialized() != SHDISP_RESULT_SUCCESS) {
        return SHDISP_RESULT_SUCCESS;
    }
    
    if (shdisp_check_bdic_exist() != SHDISP_RESULT_SUCCESS) {
        return SHDISP_RESULT_SUCCESS;
    }
    
    if (shdisp_check_upper_unit() != SHDISP_RESULT_SUCCESS) {
        return SHDISP_RESULT_SUCCESS;
    }
    
    shdisp_semaphore_start();

    ret = shdisp_SQE_main_lcd_power_on();
    
    shdisp_semaphore_end(__func__);

    if (ret != SHDISP_RESULT_SUCCESS) {
        SHDISP_ERR("<RESULT_FAILURE> shdisp_SQE_main_lcd_power_on.\n");
        return SHDISP_RESULT_FAILURE;
    }
    
    return SHDISP_RESULT_SUCCESS;
}


/* ------------------------------------------------------------------------- */
/* shdisp_api_main_lcd_reset                                                 */
/* ------------------------------------------------------------------------- */

int shdisp_api_main_lcd_reset(void)
{
    int ret;
    
    if (shdisp_check_initialized() != SHDISP_RESULT_SUCCESS) {
        return SHDISP_RESULT_SUCCESS;
    }
    
    if (shdisp_check_bdic_exist() != SHDISP_RESULT_SUCCESS) {
        return SHDISP_RESULT_SUCCESS;
    }
    
    if (shdisp_check_upper_unit() != SHDISP_RESULT_SUCCESS) {
        return SHDISP_RESULT_SUCCESS;
    }
    
    shdisp_semaphore_start();

    ret = shdisp_SQE_main_lcd_reset();
    
    shdisp_semaphore_end(__func__);

    if (ret != SHDISP_RESULT_SUCCESS) {
        SHDISP_ERR("<RESULT_FAILURE> shdisp_SQE_main_lcd_reset.\n");
        return SHDISP_RESULT_FAILURE;
    }
    
    return SHDISP_RESULT_SUCCESS;
}


/* ------------------------------------------------------------------------- */
/* shdisp_api_main_lcd_power_off                                             */
/* ------------------------------------------------------------------------- */

int shdisp_api_main_lcd_power_off(void)
{
    int ret;
    
    if (shdisp_check_initialized() != SHDISP_RESULT_SUCCESS) {
        return SHDISP_RESULT_SUCCESS;
    }
    
    if (shdisp_check_bdic_exist() != SHDISP_RESULT_SUCCESS) {
        return SHDISP_RESULT_SUCCESS;
    }
    
    if (shdisp_check_upper_unit() != SHDISP_RESULT_SUCCESS) {
        return SHDISP_RESULT_SUCCESS;
    }
    
    shdisp_semaphore_start();

    ret = shdisp_SQE_main_lcd_power_off();
    
    shdisp_semaphore_end(__func__);

    if (ret != SHDISP_RESULT_SUCCESS) {
        SHDISP_ERR("<RESULT_FAILURE> shdisp_SQE_main_lcd_power_off.\n");
        return SHDISP_RESULT_FAILURE;
    }
    
    return SHDISP_RESULT_SUCCESS;
}


/* ------------------------------------------------------------------------- */
/* shdisp_api_main_disp_init_1st                                             */
/* ------------------------------------------------------------------------- */

int shdisp_api_main_disp_init_1st(void)
{
    int ret;
    
    if (shdisp_check_initialized() != SHDISP_RESULT_SUCCESS) {
        return SHDISP_RESULT_SUCCESS;
    }
    
    if (shdisp_check_upper_unit() != SHDISP_RESULT_SUCCESS) {
        return SHDISP_RESULT_SUCCESS;
    }
    
    shdisp_semaphore_start();

    ret = shdisp_SQE_main_disp_init_1st();
    
    shdisp_semaphore_end(__func__);

    if (ret != SHDISP_RESULT_SUCCESS) {
        return SHDISP_RESULT_FAILURE;
    }
    
    return SHDISP_RESULT_SUCCESS;
}


/* ------------------------------------------------------------------------- */
/* shdisp_api_main_disp_init_2nd                                             */
/* ------------------------------------------------------------------------- */

int shdisp_api_main_disp_init_2nd(void)
{
    int ret;
    
    if (shdisp_check_initialized() != SHDISP_RESULT_SUCCESS) {
        return SHDISP_RESULT_SUCCESS;
    }
    
    if (shdisp_check_upper_unit() != SHDISP_RESULT_SUCCESS) {
        return SHDISP_RESULT_SUCCESS;
    }
    
    shdisp_semaphore_start();

    ret = shdisp_SQE_main_disp_init_2nd();
    
    shdisp_semaphore_end(__func__);
    
    if (ret != SHDISP_RESULT_SUCCESS) {
        return SHDISP_RESULT_FAILURE;
    }
    
    return SHDISP_RESULT_SUCCESS;
}


/* ------------------------------------------------------------------------- */
/* shdisp_api_main_disp_on                                                   */
/* ------------------------------------------------------------------------- */

int shdisp_api_main_disp_on(void)
{
    int ret;
    
    if (shdisp_check_initialized() != SHDISP_RESULT_SUCCESS) {
        return SHDISP_RESULT_SUCCESS;
    }
    
    if (shdisp_check_upper_unit() != SHDISP_RESULT_SUCCESS) {
        return SHDISP_RESULT_SUCCESS;
    }
    
    SHDISP_TRACE("shdisp_api_main_disp_on\n");
    
    shdisp_semaphore_start();
    
    ret = shdisp_SQE_main_disp_on();
    
    shdisp_semaphore_end(__func__);
    
    if (ret != SHDISP_RESULT_SUCCESS) {
        SHDISP_ERR("<RESULT_FAILURE> shdisp_SQE_main_disp_on.\n");
        return SHDISP_RESULT_FAILURE;
    }
    
#ifdef CONFIG_TOUCHSCREEN_SHTPS
    msm_tps_setsleep(0);
#endif /* CONFIG_TOUCHSCREEN_SHTPS */
    
    return SHDISP_RESULT_SUCCESS;
}


/* ------------------------------------------------------------------------- */
/* shdisp_api_main_disp_off                                                  */
/* ------------------------------------------------------------------------- */

int shdisp_api_main_disp_off(void)
{
    int ret;
    
    if (shdisp_check_initialized() != SHDISP_RESULT_SUCCESS) {
        return SHDISP_RESULT_SUCCESS;
    }
    
    if (shdisp_check_upper_unit() != SHDISP_RESULT_SUCCESS) {
        return SHDISP_RESULT_SUCCESS;
    }
    
    SHDISP_TRACE("shdisp_api_main_disp_off\n");
    
#ifdef CONFIG_TOUCHSCREEN_SHTPS
    msm_tps_setsleep(1);
#endif /* CONFIG_TOUCHSCREEN_SHTPS */
    
    shdisp_semaphore_start();
    
    ret = shdisp_SQE_main_disp_off();
    
    shdisp_semaphore_end(__func__);
    
    if (ret != SHDISP_RESULT_SUCCESS) {
        SHDISP_ERR("<RESULT_FAILURE> shdisp_SQE_main_disp_off.\n");
        return SHDISP_RESULT_FAILURE;
    }
    
    return SHDISP_RESULT_SUCCESS;
}


/* ------------------------------------------------------------------------- */
/* shdisp_api_main_bkl_on                                                    */
/* ------------------------------------------------------------------------- */

int shdisp_api_main_bkl_on(struct shdisp_main_bkl_ctl *bkl)
{
    int ret;
    
    if (bkl == NULL) {
        SHDISP_ERR("<NULL_POINTER> bkl.\n");
        return SHDISP_RESULT_FAILURE;
    }
    
    if (shdisp_check_initialized() != SHDISP_RESULT_SUCCESS) {
        return SHDISP_RESULT_SUCCESS;
    }
    
    if (shdisp_check_bdic_exist() != SHDISP_RESULT_SUCCESS) {
        return SHDISP_RESULT_SUCCESS;
    }
    
    if (shdisp_check_upper_unit() != SHDISP_RESULT_SUCCESS) {
        return SHDISP_RESULT_SUCCESS;
    }
    
    if ((bkl->mode <= SHDISP_MAIN_BKL_MODE_OFF) ||
        (bkl->mode >= SHDISP_MAIN_BKL_MODE_DTV_OFF)) {
            SHDISP_ERR("<INVALID_VALUE> bkl->mode.\n");
        return SHDISP_RESULT_FAILURE;
    }
    
    if (bkl->mode != SHDISP_MAIN_BKL_MODE_FIX) {
        return SHDISP_RESULT_SUCCESS;
    }
    
    if (bkl->param <= SHDISP_MAIN_BKL_PARAM_0) {
        SHDISP_ERR("<INVALID_VALUE> bkl->param.\n");
        return SHDISP_RESULT_FAILURE;
    }
    
    if (bkl->param >= NUM_SHDISP_MAIN_BKL_PARAM) {
        bkl->param = SHDISP_MAIN_BKL_PARAM_22;
    }
    
    shdisp_semaphore_start();

    ret = shdisp_SQE_main_bkl_ctl(SHDISP_MAIN_BKL_DEV_TYPE_APP, bkl);
    
    shdisp_als_irq_subscribe_bkl_ctrl(SHDISP_BKL_MODE_ON);

    shdisp_semaphore_end(__func__);

    if (ret != SHDISP_RESULT_SUCCESS) {
        SHDISP_ERR("<RESULT_FAILURE> shdisp_SQE_main_bkl_ctl.\n");
        return SHDISP_RESULT_FAILURE;
    }
    
    return SHDISP_RESULT_SUCCESS;
}


/* ------------------------------------------------------------------------- */
/* shdisp_api_main_bkl_off                                                   */
/* ------------------------------------------------------------------------- */

int shdisp_api_main_bkl_off(void)
{
    int ret;
    struct shdisp_main_bkl_ctl bkl_ctl;
    
    if (shdisp_check_initialized() != SHDISP_RESULT_SUCCESS) {
        return SHDISP_RESULT_SUCCESS;
    }
    
    if (shdisp_check_bdic_exist() != SHDISP_RESULT_SUCCESS) {
        return SHDISP_RESULT_SUCCESS;
    }
    
    if (shdisp_check_upper_unit() != SHDISP_RESULT_SUCCESS) {
        return SHDISP_RESULT_SUCCESS;
    }
    
    bkl_ctl.mode  = SHDISP_MAIN_BKL_MODE_OFF;
    bkl_ctl.param = SHDISP_MAIN_BKL_PARAM_0;
    
    shdisp_semaphore_start();

    ret = shdisp_SQE_main_bkl_ctl(SHDISP_MAIN_BKL_DEV_TYPE_APP, &(bkl_ctl));
    
    shdisp_als_irq_subscribe_bkl_ctrl(SHDISP_BKL_MODE_OFF);

    shdisp_SQE_pwm_stop();

    shdisp_semaphore_end(__func__);

    if (ret != SHDISP_RESULT_SUCCESS) {
        SHDISP_ERR("<RESULT_FAILURE> shdisp_SQE_main_bkl_ctl.\n");
        return SHDISP_RESULT_FAILURE;
    }
    
    return SHDISP_RESULT_SUCCESS;
}


/* ------------------------------------------------------------------------- */
/* shdisp_api_shutdown                                                       */
/* ------------------------------------------------------------------------- */

int shdisp_api_shutdown(void)
{
    return SHDISP_RESULT_SUCCESS;
}


/* ------------------------------------------------------------------------- */
/* shdisp_api_write_bdic_i2c                                                 */
/* ------------------------------------------------------------------------- */

int shdisp_api_write_bdic_i2c(struct shdisp_bdic_i2c_msg *i2c_msg)
{
    int ret;
    
    if (i2c_msg == NULL) {
        SHDISP_ERR("<NULL_POINTER> i2c_msg.\n");
        return SHDISP_RESULT_FAILURE;
    }
    
    if (i2c_msg->wbuf == NULL) {
        SHDISP_ERR("<NULL_POINTER> i2c_msg->wbuf.\n");
        return SHDISP_RESULT_FAILURE;
    }
    
    if (shdisp_check_initialized() != SHDISP_RESULT_SUCCESS) {
        return SHDISP_RESULT_SUCCESS;
    }
    
    if (shdisp_check_bdic_exist() != SHDISP_RESULT_SUCCESS) {
        return SHDISP_RESULT_SUCCESS;
    }
    
    if (i2c_msg->mode != SHDISP_BDIC_I2C_M_W) {
        i2c_msg->mode = SHDISP_BDIC_I2C_M_W;
    }
    
    shdisp_semaphore_start();

    ret = shdisp_SQE_write_bdic_i2c(i2c_msg);
    
    shdisp_semaphore_end(__func__);

    if (ret == SHDISP_RESULT_FAILURE_I2C_TMO) {
        SHDISP_ERR("<TIME_OUT> shdisp_SQE_write_bdic_i2c.\n");
        return SHDISP_RESULT_FAILURE_I2C_TMO;
    }
    else if (ret == SHDISP_RESULT_FAILURE) {
        SHDISP_ERR("<RESULT_FAILURE> shdisp_SQE_write_bdic_i2c.\n");
        return SHDISP_RESULT_FAILURE;
    }
    
    return SHDISP_RESULT_SUCCESS;
}


/* ------------------------------------------------------------------------- */
/* shdisp_api_read_bdic_i2c                                                  */
/* ------------------------------------------------------------------------- */

int shdisp_api_read_bdic_i2c(struct shdisp_bdic_i2c_msg *i2c_msg)
{
    int ret;
    
    if (i2c_msg == NULL) {
        SHDISP_ERR("<NULL_POINTER> i2c_msg.\n");
        return SHDISP_RESULT_FAILURE;
    }
    
    if (i2c_msg->wbuf == NULL) {
        SHDISP_ERR("<NULL_POINTER> i2c_msg->wbuf.\n");
        return SHDISP_RESULT_FAILURE;
    }
    
    if (i2c_msg->rbuf == NULL) {
        SHDISP_ERR("<NULL_POINTER> i2c_msg->rbuf.\n");
        return SHDISP_RESULT_FAILURE;
    }
    
    if (shdisp_check_initialized() != SHDISP_RESULT_SUCCESS) {
        return SHDISP_RESULT_SUCCESS;
    }
    
    if (shdisp_check_bdic_exist() != SHDISP_RESULT_SUCCESS) {
        return SHDISP_RESULT_SUCCESS;
    }
    
    if (i2c_msg->mode != SHDISP_BDIC_I2C_M_R) {
        i2c_msg->mode = SHDISP_BDIC_I2C_M_R;
    }
    
    shdisp_semaphore_start();

    ret = shdisp_SQE_read_bdic_i2c(i2c_msg);
    
    shdisp_semaphore_end(__func__);

    if (ret == SHDISP_RESULT_FAILURE_I2C_TMO) {
        SHDISP_ERR("<TIME_OUT> shdisp_SQE_read_bdic_i2c.\n");
        return SHDISP_RESULT_FAILURE_I2C_TMO;
    }
    else if (ret == SHDISP_RESULT_FAILURE) {
        SHDISP_ERR("<RESULT_FAILURE> shdisp_SQE_read_bdic_i2c.\n");
        return SHDISP_RESULT_FAILURE;
    }
    
    return SHDISP_RESULT_SUCCESS;
}


/* ------------------------------------------------------------------------- */
/* shdisp_api_prox_sensor_pow_ctl                                            */
/* ------------------------------------------------------------------------- */

int shdisp_api_prox_sensor_pow_ctl(int power_mode, struct shdisp_prox_params *prox_params)
{
    int ret;
    
    if (power_mode >= NUM_SHDISP_PROX_SENSOR_POWER) {
        SHDISP_ERR("<INVALID_VALUE> power_mode(%d).\n", power_mode);
        return SHDISP_RESULT_FAILURE;
    }

    SHDISP_TRACE("[SHDISP]shdisp_api_prox_sensor_pow_ctl:power_mode=%d\n", power_mode );

    if( power_mode == SHDISP_PROX_SENSOR_POWER_ON ){
        if( prox_params == NULL )
            return SHDISP_RESULT_FAILURE;
        shdisp_bdic_API_set_prox_sensor_param(prox_params);
    }
    
    shdisp_semaphore_start();

    ret = shdisp_SQE_prox_sensor_pow_ctl(power_mode);
    
    shdisp_semaphore_end(__func__);

    if (ret != SHDISP_RESULT_SUCCESS) {
        SHDISP_ERR("<RESULT_FAILURE> shdisp_SQE_prox_sensor_pow_ctl.\n");
        return SHDISP_RESULT_FAILURE;
    }
    
    return SHDISP_RESULT_SUCCESS;
}


/* ------------------------------------------------------------------------- */
/* shdisp_api_get_boot_context                                               */
/* ------------------------------------------------------------------------- */

void shdisp_api_get_boot_context(void)
{
    shdisp_init_context();
    
    return;
}


/* ------------------------------------------------------------------------- */
/* shdisp_api_get_panel_info                                                 */
/* ------------------------------------------------------------------------- */

int shdisp_api_get_panel_info(void)
{
    int ret;
    
    ret = shdisp_check_panel_info();
    
    return ret;
}


/* ------------------------------------------------------------------------- */
/* shdisp_api_get_clock_info                                                 */
/* ------------------------------------------------------------------------- */

int shdisp_api_get_clock_info(void)
{
    int ret;
    
    ret = shdisp_check_clock_info();
    
    return ret;
}


/* ------------------------------------------------------------------------- */
/* shdisp_api_get_clock_sw                                                   */
/* ------------------------------------------------------------------------- */

int shdisp_api_get_clock_sw(void)
{
    int ret;
    
    ret = shdisp_check_clock_sw();
    
    return ret;
}


/* ------------------------------------------------------------------------- */
/* shdisp_api_get_boot_disp_status                                           */
/* ------------------------------------------------------------------------- */

int shdisp_api_get_boot_disp_status(void)
{
    return shdisp_get_boot_disp_status();
}


/* ------------------------------------------------------------------------- */
/* shdisp_api_get_upper_unit                                                 */
/* ------------------------------------------------------------------------- */

int shdisp_api_get_upper_unit(void)
{
    int ret;
    
    ret = shdisp_check_upper_unit();
    
    return ret;
}

/* ------------------------------------------------------------------------- */
/* shdisp_api_set_upper_unit                                                 */
/* ------------------------------------------------------------------------- */

int shdisp_api_set_upper_unit(int mode)
{
    int ret;
    
    ret = shdisp_set_upper_unit(mode);
    
    return ret;
}


/* ------------------------------------------------------------------------- */
/* shdisp_api_get_hw_revision                                                */
/* ------------------------------------------------------------------------- */

unsigned short shdisp_api_get_hw_revision(void)
{
    return shdisp_get_hw_revision();
}


/* ------------------------------------------------------------------------- */
/* shdisp_api_get_alpha                                                      */
/* ------------------------------------------------------------------------- */

unsigned short shdisp_api_get_alpha(void)
{
    return shdisp_get_alpha();
}


/* ------------------------------------------------------------------------- */
/* shdisp_api_get_alpha_low                                                  */
/* ------------------------------------------------------------------------- */

unsigned short shdisp_api_get_alpha_low(void)
{
    return shdisp_get_alpha_low();
}


/* ------------------------------------------------------------------------- */
/* shdisp_api_get_lcddr_phy_gamma                                            */
/* ------------------------------------------------------------------------- */

struct shdisp_lcddr_phy_gamma_reg* shdisp_api_get_lcddr_phy_gamma(void)
{
    return shdisp_get_lcddr_phy_gamma();
}


/* ------------------------------------------------------------------------- */
/* shdisp_api_check_recovery                                                 */
/* ------------------------------------------------------------------------- */
int shdisp_api_check_recovery (void)
{
    int ret;
    
    if (shdisp_check_initialized() != SHDISP_RESULT_SUCCESS) {
        return SHDISP_RESULT_SUCCESS;
    }
    
    if (shdisp_check_bdic_exist() != SHDISP_RESULT_SUCCESS) {
        return SHDISP_RESULT_SUCCESS;
    }
    
    if (shdisp_check_upper_unit() != SHDISP_RESULT_SUCCESS) {
        return SHDISP_RESULT_SUCCESS;
    }

    shdisp_semaphore_start();

    ret = shdisp_SQE_check_recovery();

    shdisp_semaphore_end(__func__);

    if (ret != SHDISP_RESULT_SUCCESS) {
        SHDISP_ERR("<RESULT_FAILURE> shdisp_api_check_recovery.\n");
        return SHDISP_RESULT_FAILURE;
    }
    
    return SHDISP_RESULT_SUCCESS;
}


/* ------------------------------------------------------------------------- */
/* shdisp_api_do_recovery                                                    */
/* ------------------------------------------------------------------------- */
int shdisp_api_do_recovery (void)
{
    int ret;
    
    if (shdisp_check_initialized() != SHDISP_RESULT_SUCCESS) {
        return SHDISP_RESULT_SUCCESS;
    }
    
    if (shdisp_check_bdic_exist() != SHDISP_RESULT_SUCCESS) {
        return SHDISP_RESULT_SUCCESS;
    }
    
    if (shdisp_check_upper_unit() != SHDISP_RESULT_SUCCESS) {
        return SHDISP_RESULT_SUCCESS;
    }

    shdisp_semaphore_start();

    ret = shdisp_SQE_do_recovery();

    shdisp_semaphore_end(__func__);

    if (ret != SHDISP_RESULT_SUCCESS) {
        SHDISP_ERR("<RESULT_FAILURE> shdisp_api_do_recovery.\n");
        return SHDISP_RESULT_FAILURE;
    }

    return SHDISP_RESULT_SUCCESS;
}


/* ------------------------------------------------------------------------- */
/* shdisp_api_event_subscribe                                                */
/* ------------------------------------------------------------------------- */

int shdisp_api_event_subscribe(struct shdisp_subscribe *subscribe)
{
    int ret;
    
    SHDISP_TRACE("shdisp_api_event_subscribe:Start(irq_type=%d)\n", subscribe->irq_type);
    
    
    
    
    

    if( shdisp_bdic_subscribe_check(subscribe) != SHDISP_RESULT_SUCCESS )
        return SHDISP_RESULT_FAILURE;
    
    if ( subscribe->irq_type == SHDISP_IRQ_TYPE_DET ) {
        if (shdisp_check_upper_unit() != SHDISP_RESULT_SUCCESS) {
            return SHDISP_RESULT_SUCCESS;
        }
    }


    if(( subscribe->irq_type == SHDISP_IRQ_TYPE_ALS ) && (shdisp_subscribe_type_table[subscribe->irq_type] == SHDISP_SUBSCRIBE_TYPE_INT)){
        return SHDISP_RESULT_FAILURE;
    }
    
    
    shdisp_semaphore_start();
    
    ret = shdisp_SQE_event_subscribe(subscribe);
    
    shdisp_semaphore_end(__func__);

    if( subscribe->irq_type == SHDISP_IRQ_TYPE_ALS ){
        shdisp_photo_lux_irq();
    }
    
    if (ret != SHDISP_RESULT_SUCCESS) {
        SHDISP_ERR("<RESULT_FAILURE> shdisp_SQE_event_subscribe.\n");
        return SHDISP_RESULT_FAILURE;
    }
    
    return SHDISP_RESULT_SUCCESS;
}


/* ------------------------------------------------------------------------- */
/* shdisp_api_event_unsubscribe                                              */
/* ------------------------------------------------------------------------- */

int shdisp_api_event_unsubscribe(int irq_type)
{
    int ret;
    
    
    

    if( shdisp_bdic_unsubscribe_check(irq_type) != SHDISP_RESULT_SUCCESS )
        return SHDISP_RESULT_FAILURE;

    if(( irq_type == SHDISP_IRQ_TYPE_ALS ) && (shdisp_subscribe_type_table[irq_type] == SHDISP_SUBSCRIBE_TYPE_INT)){
        return SHDISP_RESULT_FAILURE;
    }

    shdisp_semaphore_start();

    
    ret = shdisp_SQE_event_unsubscribe(irq_type);
    
    shdisp_semaphore_end(__func__);
    
    if (ret != SHDISP_RESULT_SUCCESS) {
        SHDISP_ERR("<RESULT_FAILURE> shdisp_SQE_event_unsubscribe.\n");
        return SHDISP_RESULT_FAILURE;
    }
    
    return SHDISP_RESULT_SUCCESS;
}


/* ------------------------------------------------------------------------- */
/* shdisp_api_check_cabc                                                     */
/* ------------------------------------------------------------------------- */

int shdisp_api_check_cabc(struct shdisp_check_cabc_val *value)
{
    int ret;

    ret = SHDISP_RESULT_SUCCESS;

    if( 0 < cabc_mode_set  && cabc_mode_set < NUM_SHDISP_CABC_MODE)
    {
        change_mode = cabc_mode_set;
    }
    else
    {
        change_mode = 0;
    }

    value->old_mode = shdisp_cabc_current_mode;
    value->old_lut = shdisp_cabc_current_lut;


    if(change_mode == SHDISP_MAIN_DISP_CABC_MODE_OFF)
    {
        value->mode = SHDISP_MAIN_DISP_CABC_MODE_OFF;
        value->lut  = SHDISP_MAIN_DISP_CABC_LUT0;
    }
    else
    {

        value->mode = change_mode;

        shdisp_semaphore_start();
        

        change_level_lut = shdisp_SQE_change_level_lut();
        shdisp_semaphore_end(__func__);

        value->lut  = change_level_lut;



        if(debug_cabc_mode == 1)
        {
            change_mode = change_cabc_mode;

            if( 0 < change_cabc_level_lut  && change_cabc_level_lut < NUM_SHDISP_CABC_LUT)
            {
                change_level_lut = change_cabc_level_lut;
            }
            else
            {
                change_level_lut = 0;
            }
        }
        value->mode = change_mode;
        value->lut  = change_level_lut;
    }

    
    value->change = 0;
    if(shdisp_cabc_current_mode != value->mode)
    {
        value->change = 1;
    }
    else
    {
        if(shdisp_cabc_current_lut != value->lut)
        {
            value->change = 1;
        }
    }
    shdisp_cabc_current_mode = value->mode;
    shdisp_cabc_current_lut = value->lut;

    SHDISP_TRACE("shdisp_api_check_cabc : shdisp_cabc_current_mode = %d.\n", shdisp_cabc_current_mode);
    SHDISP_TRACE("shdisp_api_check_cabc : shdisp_cabc_current_lut = %d.\n", shdisp_cabc_current_lut);
    SHDISP_TRACE("shdisp_api_check_cabc : value->change = %d.\n", value->change);
    return ret;
}


/* ------------------------------------------------------------------------- */
/* shdisp_api_pwm_enable                                                     */
/* ------------------------------------------------------------------------- */

int shdisp_api_pwm_enable(void)
{
    int ret;
    
    if (shdisp_check_initialized() != SHDISP_RESULT_SUCCESS) {
        return SHDISP_RESULT_SUCCESS;
    }
    
    if (shdisp_check_upper_unit() != SHDISP_RESULT_SUCCESS) {
        return SHDISP_RESULT_SUCCESS;
    }
    
    SHDISP_TRACE("shdisp_api_pwm_enable\n");
    
    shdisp_semaphore_start();
    
    ret = shdisp_SQE_pwm_enable();
    
    shdisp_semaphore_end(__func__);
    
    if (ret != SHDISP_RESULT_SUCCESS) {
        SHDISP_ERR("<RESULT_FAILURE> shdisp_SQE_pwm_enable.\n");
        return SHDISP_RESULT_FAILURE;
    }
    
    return SHDISP_RESULT_SUCCESS;
}


/* ------------------------------------------------------------------------- */
/* shdisp_api_pwm_disable                                                   */
/* ------------------------------------------------------------------------- */

int shdisp_api_pwm_disable(void)
{
    int ret;
    
    if (shdisp_check_initialized() != SHDISP_RESULT_SUCCESS) {
        return SHDISP_RESULT_SUCCESS;
    }
    
    if (shdisp_check_upper_unit() != SHDISP_RESULT_SUCCESS) {
        return SHDISP_RESULT_SUCCESS;
    }
    
    SHDISP_TRACE("shdisp_api_pwm_disable\n");
    
    shdisp_semaphore_start();
    
    ret = shdisp_SQE_pwm_disable();
    
    shdisp_semaphore_end(__func__);
    
    if (ret != SHDISP_RESULT_SUCCESS) {
        SHDISP_ERR("<RESULT_FAILURE> shdisp_SQE_pwm_disable.\n");
        return SHDISP_RESULT_FAILURE;
    }
    
    return SHDISP_RESULT_SUCCESS;
}


/* ------------------------------------------------------------------------- */
/* shdisp_api_photo_sensor_pow_ctl                                           */
/* ------------------------------------------------------------------------- */

int shdisp_api_photo_sensor_pow_ctl(void)
{
    int ret;

    int bFirst;
    
    struct shdisp_photo_sensor_power_ctl ctrl;

    if(cabc_mode_set != SHDISP_MAIN_DISP_CABC_MODE_OFF)
    {
        ctrl.power = SHDISP_PHOTO_SENSOR_ENABLE;
        ctrl.type  = SHDISP_PHOTO_SENSOR_TYPE_LUX;

        shdisp_semaphore_start();
        
        bFirst = 0;
        if(( shdisp_als_irq_req_state & SHDISP_ALS_IRQ_REQ_DBC ) == 0 ){
            bFirst = 1;
        }
        
        ret = shdisp_SQE_photo_sensor_pow_ctl(&ctrl);
        
        shdisp_semaphore_end(__func__);

        if (ret != SHDISP_RESULT_SUCCESS) {
            SHDISP_ERR("<RESULT_FAILURE> shdisp_api_photo_sensor_pow_ctl.\n");
            return SHDISP_RESULT_FAILURE;
        }

        if( bFirst ){
            shdisp_photo_sensor_api_first = 1;
            shdisp_photo_lux_irq();
            shdisp_photo_sensor_api_first = 0;
        }
    }
    
    return SHDISP_RESULT_SUCCESS;
}

/* ------------------------------------------------------------------------- */
/* shdisp_bdic_subscribe_check                                               */
/* ------------------------------------------------------------------------- */

static int shdisp_bdic_subscribe_check(struct shdisp_subscribe *subscribe)
{

    if (shdisp_check_initialized() != SHDISP_RESULT_SUCCESS) {
        return SHDISP_RESULT_FAILURE;
    }
    
    if (shdisp_check_bdic_exist() != SHDISP_RESULT_SUCCESS) {
        return SHDISP_RESULT_FAILURE;
    }
    
    if (subscribe == NULL) {
        SHDISP_ERR("<NULL POINTER> INT_SUBSCRIBE subscribe\n");
        return SHDISP_RESULT_FAILURE;
    }
    
    if (( subscribe->irq_type < SHDISP_IRQ_TYPE_ALS ) || ( subscribe->irq_type >= NUM_SHDISP_IRQ_TYPE )) {
        SHDISP_ERR("<INVALID_VALUE> subscribe->irq_type(%d)\n", subscribe->irq_type);
        return SHDISP_RESULT_FAILURE;
    }
    
    if( subscribe->callback == NULL ){
        SHDISP_ERR("<NULL_POINTER> subscribe->callback\n");
        return SHDISP_RESULT_FAILURE;
    }
    
    return SHDISP_RESULT_SUCCESS;
}


/* ------------------------------------------------------------------------- */
/* shdisp_bdic_unsubscribe_check                                               */
/* ------------------------------------------------------------------------- */

static int shdisp_bdic_unsubscribe_check(int irq_type)
{
    if (shdisp_check_initialized() != SHDISP_RESULT_SUCCESS) {
        return SHDISP_RESULT_FAILURE;
    }
    
    if (shdisp_check_bdic_exist() != SHDISP_RESULT_SUCCESS) {
        return SHDISP_RESULT_FAILURE;
    }
    
    if(( irq_type < SHDISP_IRQ_TYPE_ALS ) || ( irq_type >= NUM_SHDISP_IRQ_TYPE )) {
        SHDISP_ERR("<INVALID_VALUE> irq_type(%d)\n", irq_type);
        return SHDISP_RESULT_FAILURE;
    }
    return SHDISP_RESULT_SUCCESS;
}


/* ------------------------------------------------------------------------- */
/* shdisp_api_do_mipi_dsi_det_recovery                                       */
/* ------------------------------------------------------------------------- */

void shdisp_api_do_mipi_dsi_det_recovery(void)
{
    int ret;
    unsigned long flags = 0;

    shdisp_SYS_set_irq(SHDISP_IRQ_DISABLE);

    spin_lock_irqsave( &shdisp_q_lock, flags);

    SHDISP_TRACE("shdisp_api_do_mipi_dsi_det_recovery:Start\n");

    if (shdisp_wq_gpio) {
        ret = queue_work(shdisp_wq_gpio, &shdisp_wq_gpio_wk);
        if( ret == 0 ) {
            SHDISP_ERR("<QUEUE_WORK_FAILURE> shdisp_api_do_mipi_dsi_det_recovery.\n");
        }
        else {
            shdisp_bdic_API_IRQ_set_det_flag();
        }
    }
    spin_unlock_irqrestore( &shdisp_q_lock, flags);
}


/* ------------------------------------------------------------------------- */
/* shdisp_api_clr_mipi_dsi_det_recovery_flag                                 */
/* ------------------------------------------------------------------------- */

void shdisp_api_clr_mipi_dsi_det_recovery_flag(void)
{
    shdisp_bdic_API_IRQ_clr_det_flag();
}


/* ------------------------------------------------------------------------- */
/* shdisp_api_get_mipi_dsi_det_recovery_flag                                 */
/* ------------------------------------------------------------------------- */

int shdisp_api_get_mipi_dsi_det_recovery_flag(void)
{
    return shdisp_bdic_API_IRQ_get_det_flag();
}


/* ------------------------------------------------------------------------- */
/* shdisp_api_get_shutdown_mode                                              */
/* ------------------------------------------------------------------------- */

int shdisp_api_get_shutdown_mode(void)
{
    return msm_fb_get_shutdown_mode();
}

/* ------------------------------------------------------------------------- */
/* shdisp_api_get_recovery_mode                                              */
/* ------------------------------------------------------------------------- */

int shdisp_api_get_recovery_mode(void)
{
    return mipi_sharp_is_recovery();
}

/* ------------------------------------------------------------------------- */
/* shdisp_api_init_cabc_state                                                */
/* ------------------------------------------------------------------------- */

void shdisp_api_init_cabc_state(int mode, int lut)
{
    shdisp_cabc_current_mode = mode;
    shdisp_cabc_current_lut  = lut;
}

/* ------------------------------------------------------------------------- */
/* INITIALIZE                                                                */
/* ------------------------------------------------------------------------- */

static void shdisp_init_context(void)
{
    if (shdisp_smem_read_flag != 0) {
        return;
    }
    
    shdisp_get_boot_context();
    
    shdisp_kerl_ctx.driver_is_initialized       = SHDISP_DRIVER_IS_NOT_INITIALIZED;
    shdisp_kerl_ctx.hw_revision                 = shdisp_boot_ctx.hw_revision;
    shdisp_kerl_ctx.handset_color               = shdisp_boot_ctx.handset_color;
    shdisp_kerl_ctx.upper_unit_is_connected     = shdisp_boot_ctx.upper_unit_is_connected;
    shdisp_kerl_ctx.bdic_is_exist               = shdisp_boot_ctx.bdic_is_exist;
    shdisp_kerl_ctx.main_disp_status            = shdisp_boot_ctx.main_disp_status;
    shdisp_kerl_ctx.main_bkl.mode               = shdisp_boot_ctx.main_bkl.mode;
    shdisp_kerl_ctx.main_bkl.param              = shdisp_boot_ctx.main_bkl.param;
    shdisp_kerl_ctx.tri_led.red                 = shdisp_boot_ctx.tri_led.red;
    shdisp_kerl_ctx.tri_led.green               = shdisp_boot_ctx.tri_led.green;
    shdisp_kerl_ctx.tri_led.blue                = shdisp_boot_ctx.tri_led.blue;
    shdisp_kerl_ctx.tri_led.ext_mode            = shdisp_boot_ctx.tri_led.ext_mode;
    shdisp_kerl_ctx.tri_led.led_mode            = shdisp_boot_ctx.tri_led.led_mode;
    shdisp_kerl_ctx.tri_led.ontime              = shdisp_boot_ctx.tri_led.ontime;
    shdisp_kerl_ctx.tri_led.interval            = shdisp_boot_ctx.tri_led.interval;
    shdisp_kerl_ctx.alpha                       = shdisp_boot_ctx.alpha;
    shdisp_kerl_ctx.alpha_low                   = shdisp_boot_ctx.alpha_low;
    shdisp_kerl_ctx.dma_lut_status              = shdisp_boot_ctx.dma_lut_status;
    memcpy(&(shdisp_kerl_ctx.photo_sensor_adj), &(shdisp_boot_ctx.photo_sensor_adj), sizeof(struct shdisp_photo_sensor_adj));
    shdisp_kerl_ctx.dtv_status                  = SHDISP_DTV_OFF;
    shdisp_kerl_ctx.thermal_status              = SHDISP_MAIN_BKL_EMG_OFF;
    shdisp_kerl_ctx.eco_bkl_status              = SHDISP_MAIN_BKL_ECO_OFF;
    shdisp_kerl_ctx.usb_chg_status              = SHDISP_MAIN_BKL_CHG_OFF;
    shdisp_kerl_ctx.ledc_status.ledc_is_exist     = shdisp_boot_ctx.ledc_status.ledc_is_exist;
    shdisp_kerl_ctx.ledc_status.power_status      = shdisp_boot_ctx.ledc_status.power_status;
    shdisp_kerl_ctx.ledc_status.ledc_req.red      = shdisp_boot_ctx.ledc_status.ledc_req.red;
    shdisp_kerl_ctx.ledc_status.ledc_req.green    = shdisp_boot_ctx.ledc_status.ledc_req.green;
    shdisp_kerl_ctx.ledc_status.ledc_req.blue     = shdisp_boot_ctx.ledc_status.ledc_req.blue;
    shdisp_kerl_ctx.ledc_status.ledc_req.led_mode = shdisp_boot_ctx.ledc_status.ledc_req.led_mode;
    shdisp_kerl_ctx.ledc_status.ledc_req.on_count = shdisp_boot_ctx.ledc_status.ledc_req.on_count;
    shdisp_kerl_ctx.shdisp_lcd                  = shdisp_boot_ctx.shdisp_lcd;
    shdisp_kerl_ctx.dbgTraceF                   = 0x00;
    memcpy(&(shdisp_kerl_ctx.lcddr_phy_gamma), &(shdisp_boot_ctx.lcddr_phy_gamma), sizeof(struct shdisp_lcddr_phy_gamma_reg));
    memcpy(&(shdisp_kerl_ctx.lcddr_rom_gamma), &(shdisp_boot_ctx.lcddr_rom_gamma), sizeof(struct shdisp_lcddr_phy_gamma_reg));
    
#ifdef CONFIG_SHLCDC_LED_BD2802GU
    shdisp_ledc_API_set_status_addr(&shdisp_kerl_ctx.ledc_status);
#endif
    
    shdisp_smem_read_flag = 1;
    
    return;
}

/* ------------------------------------------------------------------------- */
/* shdisp_get_boot_context                                                   */
/* ------------------------------------------------------------------------- */

static void shdisp_get_boot_context(void)
{
#ifdef SHDISP_SW_DISABLE_SMEM_INFO
    struct shdisp_boot_context *local_boot_context = NULL;
#endif
    
    sh_smem_common = (sharp_smem_common_type *)sh_smem_get_common_address();
    if (sh_smem_common == NULL) {
        shdisp_context_initialize(0);
        shdisp_context_initialize(1);
        shdisp_bdic_API_check_sensor_param(&(shdisp_boot_ctx.photo_sensor_adj),&(shdisp_boot_ctx.photo_sensor_adj));
        shdisp_boot_ctx.upper_unit_is_connected = SHDISP_UPPER_UNIT_IS_NOT_CONNECTED;
    }
    else {
#ifdef SHDISP_SW_DISABLE_SMEM_INFO
        local_boot_context = (struct shdisp_boot_context *)(sh_smem_common->shdisp_data_buf);
        shdisp_context_initialize(0);
        shdisp_boot_ctx.alpha = local_boot_context->alpha;
        shdisp_boot_ctx.alpha_low = local_boot_context->alpha_low;
        shdisp_boot_ctx.upper_unit_is_connected = local_boot_context->upper_unit_is_connected;
        memcpy(&(shdisp_boot_ctx.photo_sensor_adj), &(local_boot_context->photo_sensor_adj), sizeof(struct shdisp_photo_sensor_adj));
        shdisp_boot_ctx.dma_lut_status = local_boot_context->dma_lut_status;
#else
        memcpy(&shdisp_boot_ctx, &sh_smem_common->shdisp_data_buf, sizeof(struct shdisp_boot_context));
#endif
    }
    
    return;
}

/* ------------------------------------------------------------------------- */
/* shdisp_context_initialize                                                 */
/* ------------------------------------------------------------------------- */

static void shdisp_context_initialize(int mode)
{
    if (mode == 0) {
        shdisp_boot_ctx.driver_is_initialized       = SHDISP_DRIVER_IS_INITIALIZED;
        shdisp_boot_ctx.hw_revision                 = SHDISP_HW_REV_PP2;
        shdisp_boot_ctx.handset_color               = 0;
        shdisp_boot_ctx.upper_unit_is_connected     = SHDISP_UPPER_UNIT_IS_CONNECTED;
        shdisp_boot_ctx.bdic_is_exist               = SHDISP_BDIC_IS_EXIST;
        shdisp_boot_ctx.main_disp_status            = SHDISP_MAIN_DISP_OFF;
        shdisp_boot_ctx.main_bkl.mode               = SHDISP_MAIN_BKL_MODE_OFF;
        shdisp_boot_ctx.main_bkl.param              = SHDISP_MAIN_BKL_PARAM_0;
        shdisp_boot_ctx.tri_led.red                 = 0;
        shdisp_boot_ctx.tri_led.green               = 0;
        shdisp_boot_ctx.tri_led.blue                = 0;
        shdisp_boot_ctx.tri_led.ext_mode            = SHDISP_TRI_LED_EXT_MODE_DISABLE;
        shdisp_boot_ctx.tri_led.led_mode            = SHDISP_TRI_LED_MODE_NORMAL;
        shdisp_boot_ctx.tri_led.ontime              = 0;
        shdisp_boot_ctx.tri_led.interval            = 0;
        shdisp_boot_ctx.alpha                       = 0;
        shdisp_boot_ctx.alpha_low                   = 0;
        shdisp_boot_ctx.dma_lut_status              = 0;
        shdisp_boot_ctx.ledc_status.ledc_is_exist     = SHDISP_LEDC_IS_EXIST;
        shdisp_boot_ctx.ledc_status.power_status      = SHDISP_LEDC_PWR_STATUS_OFF;
        shdisp_boot_ctx.ledc_status.ledc_req.red      = 0;
        shdisp_boot_ctx.ledc_status.ledc_req.green    = 0;
        shdisp_boot_ctx.ledc_status.ledc_req.blue     = 0;
        shdisp_boot_ctx.ledc_status.ledc_req.led_mode = SHDISP_LEDC_RGB_MODE_NORMAL;
        shdisp_boot_ctx.ledc_status.ledc_req.on_count = SHDISP_LEDC_ONCOUNT_REPEAT;
        shdisp_boot_ctx.shdisp_lcd                  = 0;
    }
    else {
        memset(&(shdisp_boot_ctx.photo_sensor_adj), 0, sizeof(struct shdisp_photo_sensor_adj));
        memset(&(shdisp_boot_ctx.lcddr_phy_gamma), 0, sizeof(struct shdisp_lcddr_phy_gamma_reg));
        memset(&(shdisp_boot_ctx.lcddr_rom_gamma), 0, sizeof(struct shdisp_lcddr_phy_gamma_reg));
    }
    return;
}


/* ------------------------------------------------------------------------- */
/* CHECKER                                                                   */
/* ------------------------------------------------------------------------- */
/* ------------------------------------------------------------------------- */
/* shdisp_check_initialized                                                  */
/* ------------------------------------------------------------------------- */

static int shdisp_check_initialized(void)
{
    if (shdisp_kerl_ctx.driver_is_initialized == SHDISP_DRIVER_IS_INITIALIZED) {
        return SHDISP_RESULT_SUCCESS;
    }
    else {
        printk("[SHDISP] shdisp_check_initialized error : driver is not initialized.\n");
        return SHDISP_RESULT_FAILURE;
    }
}


/* ------------------------------------------------------------------------- */
/* shdisp_check_upper_unit                                                   */
/* ------------------------------------------------------------------------- */

static int shdisp_check_upper_unit(void)
{
    if (shdisp_kerl_ctx.upper_unit_is_connected == SHDISP_UPPER_UNIT_IS_CONNECTED) {
        return SHDISP_RESULT_SUCCESS;
    }
    else {
        printk("[SHDISP] shdisp_check_upper_unit error : upper unit is not connected.\n");
        return SHDISP_RESULT_FAILURE;
    }
}


/* ------------------------------------------------------------------------- */
/* shdisp_set_upper_unit                                                     */
/* ------------------------------------------------------------------------- */

static int shdisp_set_upper_unit(int mode)
{
    shdisp_kerl_ctx.upper_unit_is_connected = mode;
    
    return SHDISP_RESULT_SUCCESS;
}


/* ------------------------------------------------------------------------- */
/* shdisp_check_bdic_exist                                                   */
/* ------------------------------------------------------------------------- */

static int shdisp_check_bdic_exist(void)
{
    if (shdisp_kerl_ctx.bdic_is_exist == SHDISP_BDIC_IS_EXIST) {
        return SHDISP_RESULT_SUCCESS;
    }
    else {
        printk("[SHDISP] shdisp_check_bdic_exist error : bdic is not exist.\n");
        return SHDISP_RESULT_FAILURE;
    }
}


/* ------------------------------------------------------------------------- */
/* shdisp_check_panel_info                                                   */
/* ------------------------------------------------------------------------- */

static int shdisp_check_panel_info(void)
{
    int revision = 0;
    
#if defined(CONFIG_SHDISP_PANEL_SWITCH)
#if defined(CONFIG_MACH_LYNX_DL10) || defined(CONFIG_MACH_LYNX_DL12)
    switch (shdisp_kerl_ctx.hw_revision) {
    case SHDISP_HW_REV_ES0:
        revision = 1;
        break;
    case SHDISP_HW_REV_ES1:
    case SHDISP_HW_REV_ES15:
    case SHDISP_HW_REV_PP1:
    case SHDISP_HW_REV_PP15:
    case SHDISP_HW_REV_PP2:
    case SHDISP_HW_REV_PP2_B:
    default:
        revision = 0;
        break;
    }
#elif defined(CONFIG_MACH_BLT)
    switch (shdisp_kerl_ctx.hw_revision) {
    case SHDISP_HW_REV_ES0:
    case SHDISP_HW_REV_ES1:
        revision = 1;
        break;
    case SHDISP_HW_REV_ES15:
    case SHDISP_HW_REV_PP1:
    case SHDISP_HW_REV_PP15:
    case SHDISP_HW_REV_PP2:
    case SHDISP_HW_REV_PP2_B:
    default:
        revision = 0;
        break;
    }
#endif
#else
#if defined(CONFIG_MACH_MNB)
    switch (shdisp_kerl_ctx.hw_revision) {
    case SHDISP_HW_REV_ES0:
    case SHDISP_HW_REV_ES05:
    case SHDISP_HW_REV_ES1:
        revision = 1;
        break;
    case SHDISP_HW_REV_ES15:
    case SHDISP_HW_REV_PP1:
    case SHDISP_HW_REV_PP15:
    case SHDISP_HW_REV_PP2:
    case SHDISP_HW_REV_PP2_B:
    default:
        revision = 0;
        break;
    }
#elif defined(CONFIG_MACH_ARS)
    switch (shdisp_kerl_ctx.hw_revision) {
    case SHDISP_HW_REV_ES0:
    case SHDISP_HW_REV_ES05:
        revision = 1;
        break;
    case SHDISP_HW_REV_ES1:
    case SHDISP_HW_REV_ES15:
    case SHDISP_HW_REV_PP1:
    case SHDISP_HW_REV_PP15:
    case SHDISP_HW_REV_PP2:
    case SHDISP_HW_REV_PP2_B:
    default:
        revision = 0;
        break;
    }
#endif
#endif
    
    return revision;
}


/* ------------------------------------------------------------------------- */
/* shdisp_check_clock_info                                                   */
/* ------------------------------------------------------------------------- */

static int shdisp_check_clock_info(void)
{
    return 0;
}


/* ------------------------------------------------------------------------- */
/* shdisp_check_clock_sw                                                     */
/* ------------------------------------------------------------------------- */

static int shdisp_check_clock_sw(void)
{
    return 0;
}


/* ------------------------------------------------------------------------- */
/* shdisp_get_boot_disp_status                                               */
/* ------------------------------------------------------------------------- */

static int shdisp_get_boot_disp_status(void)
{
    return shdisp_boot_ctx.main_disp_status;
}


/* ------------------------------------------------------------------------- */
/* shdisp_get_hw_revision                                                    */
/* ------------------------------------------------------------------------- */

static unsigned short shdisp_get_hw_revision(void)
{
    return shdisp_kerl_ctx.hw_revision;
}


/* ------------------------------------------------------------------------- */
/* shdisp_get_alpha                                                          */
/* ------------------------------------------------------------------------- */

static unsigned short shdisp_get_alpha(void)
{
    return shdisp_kerl_ctx.alpha;
}


/* ------------------------------------------------------------------------- */
/* shdisp_get_alpha_low                                                      */
/* ------------------------------------------------------------------------- */

static unsigned short shdisp_get_alpha_low(void)
{
    return shdisp_kerl_ctx.alpha_low;
}


/* ------------------------------------------------------------------------- */
/* shdisp_get_lcddr_phy_gamma                                                */
/* ------------------------------------------------------------------------- */

static struct shdisp_lcddr_phy_gamma_reg* shdisp_get_lcddr_phy_gamma(void)
{
    return &shdisp_kerl_ctx.lcddr_phy_gamma;
}


/* ------------------------------------------------------------------------- */
/* FOPS                                                                      */
/* ------------------------------------------------------------------------- */
/* ------------------------------------------------------------------------- */
/* shdisp_open                                                               */
/* ------------------------------------------------------------------------- */

static int shdisp_open(struct inode *inode, struct file *filp)
{
    if (shdisp_driver_is_initialized == 0) {
        printk("[SHDISP] new open shdisp device driver.\n");
    }

    shdisp_driver_is_initialized++;

    return 0;
}


/* ------------------------------------------------------------------------- */
/* shdisp_write                                                              */
/* ------------------------------------------------------------------------- */

static int shdisp_write(struct file *filp, const char __user *buf,
                         size_t count, loff_t *ppos)
{
    return 0;
}


/* ------------------------------------------------------------------------- */
/* shdisp_read                                                               */
/* ------------------------------------------------------------------------- */

static int shdisp_read(struct file *filp, char __user *buf,
                        size_t count, loff_t *ppos)
{
    return 0;
}


/* ------------------------------------------------------------------------- */
/* shdisp_mmap                                                               */
/* ------------------------------------------------------------------------- */

static int shdisp_mmap(struct file *filp, struct vm_area_struct *vma)
{
    return 0;
}


/* ------------------------------------------------------------------------- */
/* shdisp_ioctl                                                              */
/* ------------------------------------------------------------------------- */

static long shdisp_ioctl(struct file *filp, unsigned int cmd, unsigned long arg)
{
    int ret;
    void __user *argp = (void __user*)arg;

    switch (cmd) {
    case SHDISP_IOCTL_GET_CONTEXT:
        ret = shdisp_ioctl_get_context(argp);
        break;
    case SHDISP_IOCTL_SET_HOST_GPIO:
        ret = shdisp_ioctl_set_host_gpio(argp);
        break;
    case SHDISP_IOCTL_TRI_LED_SET_COLOR:
        ret = shdisp_ioctl_tri_led_set_color(argp);
        break;
    case SHDISP_IOCTL_BDIC_WRITE_REG:
        ret = shdisp_ioctl_bdic_write_reg(argp);
        break;
    case SHDISP_IOCTL_BDIC_READ_REG:
        ret = shdisp_ioctl_bdic_read_reg(argp);
        break;
    case SHDISP_IOCTL_BDIC_MULTI_READ_REG:
        ret = shdisp_ioctl_bdic_multi_read_reg(argp);
        break;
    case SHDISP_IOCTL_GET_LUX:
        ret = shdisp_ioctl_get_lux(argp);
        break;
    case SHDISP_IOCTL_PWM_CONFIG:
        ret = shdisp_ioctl_pwm_config(argp);
        break;
    case SHDISP_IOCTL_PWM_STOP:
        ret = shdisp_ioctl_pwm_stop();
        break;
    case SHDISP_IOCTL_PHOTO_SENSOR_POW_CTL:
        ret = shdisp_ioctl_photo_sensor_pow_ctl(argp);
        break;
    case SHDISP_IOCTL_LCDDR_WRITE_REG:
        ret = shdisp_ioctl_lcddr_write_reg(argp);
        break;
    case SHDISP_IOCTL_LCDDR_READ_REG:
        ret = shdisp_ioctl_lcddr_read_reg(argp);
        break;
    case SHDISP_IOCTL_SET_FLICKER_PARAM:
        ret = shdisp_ioctl_set_flicker_param(argp);
        break;
    case SHDISP_IOCTL_GET_FLICKER_PARAM:
        ret = shdisp_ioctl_get_flicker_param(argp);
        break;
    case SHDISP_IOCTL_BKL_SET_AUTO_MODE:
        ret = shdisp_ioctl_bkl_set_auto_mode(argp);
        break;
    case SHDISP_IOCTL_BKL_SET_DTV_MODE:
        ret = shdisp_ioctl_bkl_set_dtv_mode(argp);
        break;
    case SHDISP_IOCTL_BKL_SET_EMG_MODE:
        ret = shdisp_ioctl_bkl_set_emg_mode(argp);
        break;
    case SHDISP_IOCTL_LEDC_POWER_ON:
        ret = shdisp_ioctl_ledc_power_on();
        break;
    case SHDISP_IOCTL_LEDC_POWER_OFF:
        ret = shdisp_ioctl_ledc_power_off();
        break;
    case SHDISP_IOCTL_LEDC_SET_RGB:
        ret = shdisp_ioctl_ledc_set_rgb(argp);
        break;
    case SHDISP_IOCTL_LEDC_SET_COLOR:
        ret = shdisp_ioctl_ledc_set_color(argp);
        break;
    case SHDISP_IOCTL_LEDC_WRITE_REG:
        ret = shdisp_ioctl_ledc_write_reg(argp);
        break;
    case SHDISP_IOCTL_LEDC_READ_REG:
        ret = shdisp_ioctl_ledc_read_reg(argp);
        break;
    case SHDISP_IOCTL_LUX_CHANGE_IND:
        ret = shdisp_ioctl_lux_change_ind(argp);
        break;
    case SHDISP_IOCTL_BKL_SET_ECO_MODE:
        ret = shdisp_ioctl_bkl_set_eco_mode(argp);
        break;
    case SHDISP_IOCTL_SET_CABC:
        ret = shdisp_ioctl_set_cabc(argp);
        break;
    case SHDISP_IOCTL_BKL_SET_CHG_MODE:
        ret = shdisp_ioctl_bkl_set_chg_mode(argp);
        break;
    case SHDISP_IOCTL_GET_FLICKER_LOW_PARAM:
        ret = shdisp_ioctl_get_flicker_low_param(argp);
        break;
    case SHDISP_IOCTL_SET_CABC_UPDATE_PARAM:
        ret = shdisp_ioctl_set_cabc_update_param(argp);
        break;
    default:
        SHDISP_ERR("<INVALID_VALUE> cmd(0x%08x).\n", cmd);
        ret = -EFAULT;
        break;
    }

    return ret;
}


/* ------------------------------------------------------------------------- */
/* shdisp_release                                                            */
/* ------------------------------------------------------------------------- */

static int shdisp_release(struct inode *inode, struct file *filp)
{
    if (shdisp_driver_is_initialized > 0) {
        shdisp_driver_is_initialized--;

        if (shdisp_driver_is_initialized == 0) {
            printk("[SHDISP] all close shdisp device driver.\n");
        }
    }

    return 0;
}


/* ------------------------------------------------------------------------- */
/* IOCTL                                                                     */
/* ------------------------------------------------------------------------- */
/* ------------------------------------------------------------------------- */
/* shdisp_ioctl_get_context                                                  */
/* ------------------------------------------------------------------------- */

static int shdisp_ioctl_get_context(void __user *argp)
{
    int ret;

    shdisp_semaphore_start();

    ret = copy_to_user(argp, &shdisp_kerl_ctx, sizeof(struct shdisp_kernel_context));

    shdisp_semaphore_end(__func__);

    if (ret != 0) {
        SHDISP_ERR("<RESULT_FAILURE> copy_to_user.\n");
        return ret;
    }
    
    return 0;
}


/* ------------------------------------------------------------------------- */
/* shdisp_ioctl_set_host_gpio                                                */
/* ------------------------------------------------------------------------- */

static int shdisp_ioctl_set_host_gpio(void __user *argp)
{
    int ret;
    struct shdisp_host_gpio host_gpio;
    
    host_gpio.num   = 0;
    host_gpio.value = 0;
    
    shdisp_semaphore_start();

    ret = shdisp_SQE_set_host_gpio(&host_gpio);
    
    shdisp_semaphore_end(__func__);

    if (ret != SHDISP_RESULT_SUCCESS) {
        SHDISP_ERR("<RESULT_FAILURE> shdisp_SQE_set_host_gpio.\n");
        return -1;
    }
    
    return 0;
}


/* ------------------------------------------------------------------------- */
/* shdisp_ioctl_tri_led_set_color                                            */
/* ------------------------------------------------------------------------- */

static int shdisp_ioctl_tri_led_set_color(void __user *argp)
{

    int ret;
    struct shdisp_tri_led tri_led;
    
    shdisp_semaphore_start();

    ret = copy_from_user(&tri_led, argp, sizeof(struct shdisp_tri_led));
    
    if (ret != 0) {
        SHDISP_ERR("<RESULT_FAILURE> copy_from_user.\n");
        shdisp_semaphore_end(__func__);
        return ret;
    }
    
    ret = shdisp_SQE_tri_led_set_color(&tri_led);
    
    shdisp_semaphore_end(__func__);

    if (ret != SHDISP_RESULT_SUCCESS) {
        SHDISP_ERR("<RESULT_FAILURE> shdisp_SQE_tri_led_set_color.\n");
        return -1;
    }
    
    return 0;
}


/* ------------------------------------------------------------------------- */
/* shdisp_ioctl_bdic_write_reg                                               */
/* ------------------------------------------------------------------------- */

static int shdisp_ioctl_bdic_write_reg(void __user *argp)
{
    int ret;
    struct shdisp_diag_bdic_reg bdic_reg;
    
    shdisp_semaphore_start();

    ret = copy_from_user(&bdic_reg, argp, sizeof(struct shdisp_diag_bdic_reg));
    
    if (ret != 0) {
        SHDISP_ERR("<RESULT_FAILURE> copy_from_user.\n");
        shdisp_semaphore_end(__func__);
        return ret;
    }
    
    ret = shdisp_SQE_bdic_write_reg(bdic_reg.reg, bdic_reg.val);
    
    shdisp_semaphore_end(__func__);

    if (ret != SHDISP_RESULT_SUCCESS) {
        SHDISP_ERR("<RESULT_FAILURE> shdisp_SQE_bdic_write_reg.\n");
        return -1;
    }
    
    return 0;
}


/* ------------------------------------------------------------------------- */
/* shdisp_ioctl_bdic_read_reg                                                */
/* ------------------------------------------------------------------------- */

static int shdisp_ioctl_bdic_read_reg(void __user *argp)
{
    int ret;
    struct shdisp_diag_bdic_reg bdic_reg;
    
    shdisp_semaphore_start();

    ret = copy_from_user(&bdic_reg, argp, sizeof(struct shdisp_diag_bdic_reg));
    
    if (ret != 0) {
        SHDISP_ERR("<RESULT_FAILURE> copy_from_user.\n");
        shdisp_semaphore_end(__func__);
        return ret;
    }
    
    ret = shdisp_SQE_bdic_read_reg(bdic_reg.reg, &(bdic_reg.val));
    
    if (ret != SHDISP_RESULT_SUCCESS) {
        SHDISP_ERR("<RESULT_FAILURE> shdisp_SQE_bdic_read_reg.\n");
        shdisp_semaphore_end(__func__);
        return -1;
    }
    
    ret = copy_to_user(argp, &bdic_reg, sizeof(struct shdisp_diag_bdic_reg));
    
    shdisp_semaphore_end(__func__);

    if (ret != 0) {
        SHDISP_ERR("<RESULT_FAILURE> copy_to_user.\n");
        return ret;
    }
    
    return 0;
}


/* ------------------------------------------------------------------------- */
/* shdisp_ioctl_bdic_multi_read_reg                                          */
/* ------------------------------------------------------------------------- */

static int shdisp_ioctl_bdic_multi_read_reg(void __user *argp)
{
    int ret;
    struct shdisp_diag_bdic_reg_multi bdic_reg;
    
    shdisp_semaphore_start();

    ret = copy_from_user(&bdic_reg, argp, sizeof(struct shdisp_diag_bdic_reg_multi));
    
    if (ret != 0) {
        SHDISP_ERR("<RESULT_FAILURE> copy_from_user.\n");
        shdisp_semaphore_end(__func__);
        return ret;
    }
    
    ret = shdisp_SQE_bdic_multi_read_reg(bdic_reg.reg, bdic_reg.val, (int)bdic_reg.size);
    
    if (ret != SHDISP_RESULT_SUCCESS) {
        SHDISP_ERR("<RESULT_FAILURE> shdisp_SQE_bdic_multi_read_reg.\n");
        shdisp_semaphore_end(__func__);
        return -1;
    }
    
    ret = copy_to_user(argp, &bdic_reg, sizeof(struct shdisp_diag_bdic_reg_multi));
    
    shdisp_semaphore_end(__func__);

    if (ret != 0) {
        SHDISP_ERR("<RESULT_FAILURE> copy_to_user.\n");
        return ret;
    }
    
    return 0;
}


/* ------------------------------------------------------------------------- */
/* shdisp_ioctl_get_lux                                                      */
/* ------------------------------------------------------------------------- */

static int shdisp_ioctl_get_lux(void __user *argp)
{
    int ret;
    struct shdisp_photo_sensor_val val;
    
    shdisp_semaphore_start();

    ret = copy_from_user(&val, argp, sizeof(struct shdisp_photo_sensor_val));
    
    if (ret != 0) {
        SHDISP_ERR("<RESULT_FAILURE> copy_from_user.\n");
        shdisp_semaphore_end(__func__);
        return ret;
    }
    
    ret = shdisp_bdic_API_PHOTO_SENSOR_lux_change_ind(&(val.mode));
    if (ret != SHDISP_RESULT_SUCCESS) {
        SHDISP_ERR("<RESULT_FAILURE> shdisp_bdic_API_PHOTO_SENSOR_lux_change_ind.\n");
        val.result = SHDISP_RESULT_FAILURE;
    }
    else {
        val.result = SHDISP_RESULT_SUCCESS;
    }

    ret = shdisp_SQE_get_lux(&(val));
    
    SHDISP_TRACE("shdisp_ioctl_get_lux value=0x%04X, lux=%lu, mode=%d\n", val.value, val.lux, val.mode );

    if (ret != SHDISP_RESULT_SUCCESS) {
        SHDISP_ERR("<RESULT_FAILURE> shdisp_SQE_get_lux.\n");
    }
    
    ret = copy_to_user(argp, &val, sizeof(struct shdisp_photo_sensor_val));
    
    shdisp_semaphore_end(__func__);

    if (ret != 0) {
        SHDISP_ERR("<RESULT_FAILURE> copy_to_user.\n");
        return ret;
    }
    
    return 0;
}


/* ------------------------------------------------------------------------- */
/* shdisp_ioctl_pwm_config                                                   */
/* ------------------------------------------------------------------------- */

static int shdisp_ioctl_pwm_config(void __user *argp)
{
    int ret;
    int duty;
    
    shdisp_semaphore_start();

    ret = copy_from_user(&duty, argp, sizeof(int));
    
    if (ret != 0) {
        SHDISP_ERR("<RESULT_FAILURE> copy_from_user.\n");
        shdisp_semaphore_end(__func__);
        return ret;
    }
    
    if ((duty <= 0) || (duty > 100)) {
        SHDISP_ERR("<INVALID_VALUE> duty(%d).\n", duty);
        shdisp_semaphore_end(__func__);
        return -1;
    }
    
    ret = shdisp_SQE_pwm_config(duty);
    
    shdisp_semaphore_end(__func__);

    if (ret != SHDISP_RESULT_SUCCESS) {
        SHDISP_ERR("<RESULT_FAILURE> shdisp_SQE_pwm_config.\n");
        return -1;
    }
    
    return 0;
}


/* ------------------------------------------------------------------------- */
/* shdisp_ioctl_pwm_stop                                                     */
/* ------------------------------------------------------------------------- */

static int shdisp_ioctl_pwm_stop(void)
{
    int ret;
    
    shdisp_semaphore_start();

    ret = shdisp_SQE_pwm_stop();
    
    shdisp_semaphore_end(__func__);

    if (ret != SHDISP_RESULT_SUCCESS) {
        SHDISP_ERR("<RESULT_FAILURE> shdisp_SQE_pwm_stop.\n");
        return -1;
    }
    
    return 0;
}


/* ------------------------------------------------------------------------- */
/* shdisp_ioctl_photo_sensor_pow_ctl                                         */
/* ------------------------------------------------------------------------- */

static int shdisp_ioctl_photo_sensor_pow_ctl(void __user *argp)
{
    int ret;
    struct shdisp_photo_sensor_power_ctl power_ctl;
    int bFirst;

    shdisp_semaphore_start();

    ret = copy_from_user(&power_ctl, argp, sizeof(struct shdisp_photo_sensor_power_ctl));
    
    if (ret != 0) {
        SHDISP_ERR("<RESULT_FAILURE> copy_from_user.\n");
        shdisp_semaphore_end(__func__);
        return ret;
    }
    
    bFirst = 0;
    if(( power_ctl.type == SHDISP_PHOTO_SENSOR_TYPE_LUX ) &&
       ( power_ctl.power == SHDISP_PHOTO_SENSOR_ENABLE ) &&
       (( shdisp_als_irq_req_state & SHDISP_ALS_IRQ_REQ_DBC ) == 0 )){
        bFirst = 1;
    }
    
    ret = shdisp_SQE_photo_sensor_pow_ctl(&power_ctl);
    
    shdisp_semaphore_end(__func__);

    if (ret != SHDISP_RESULT_SUCCESS) {
        SHDISP_ERR("<RESULT_FAILURE> shdisp_SQE_photo_sensor_pow_ctl.\n");
        return -1;
    }

    if( bFirst ){
        shdisp_photo_lux_irq();
    }
    
    return 0;
}


/* ------------------------------------------------------------------------- */
/* shdisp_ioctl_lcddr_write_reg                                              */
/* ------------------------------------------------------------------------- */

static int shdisp_ioctl_lcddr_write_reg(void __user *argp)
{
    int ret;
    struct shdisp_lcddr_reg panel_reg;
    
    shdisp_semaphore_start();

    ret = copy_from_user(&panel_reg, argp, sizeof(struct shdisp_lcddr_reg));
    
    if (ret != 0) {
        SHDISP_ERR("<RESULT_FAILURE> copy_from_user.\n");
        shdisp_semaphore_end(__func__);
        return ret;
    }
    
    ret = shdisp_SQE_panel_write_reg(&panel_reg);
    
    shdisp_semaphore_end(__func__);

    if (ret != SHDISP_RESULT_SUCCESS) {
        SHDISP_ERR("<RESULT_FAILURE> shdisp_SQE_panel_write_reg.\n");
        return -1;
    }
    
    return 0;
}


/* ------------------------------------------------------------------------- */
/* shdisp_ioctl_lcddr_read_reg                                               */
/* ------------------------------------------------------------------------- */

static int shdisp_ioctl_lcddr_read_reg(void __user *argp)
{
    int ret;
    struct shdisp_lcddr_reg panel_reg;
    
    shdisp_semaphore_start();

    ret = copy_from_user(&panel_reg, argp, sizeof(struct shdisp_lcddr_reg));
    
    if (ret != 0) {
        SHDISP_ERR("<RESULT_FAILURE> copy_from_user.\n");
        shdisp_semaphore_end(__func__);
        return ret;
    }
    
    ret = shdisp_SQE_panel_read_reg(&panel_reg);
    
    if (ret != SHDISP_RESULT_SUCCESS) {
        SHDISP_ERR("<RESULT_FAILURE> shdisp_SQE_panel_read_reg.\n");
        shdisp_semaphore_end(__func__);
        return -1;
    }
    
    ret = copy_to_user(argp, &panel_reg, sizeof(struct shdisp_lcddr_reg));
    
    shdisp_semaphore_end(__func__);

    if (ret != 0) {
        SHDISP_ERR("<RESULT_FAILURE> copy_to_user.\n");
        return ret;
    }
    
    return 0;
}


/* ------------------------------------------------------------------------- */
/* shdisp_ioctl_set_flicker_param                                            */
/* ------------------------------------------------------------------------- */

static int shdisp_ioctl_set_flicker_param(void __user *argp)
{
    int ret;
    unsigned short alpha;
    
    shdisp_semaphore_start();

    ret = copy_from_user(&alpha, argp, sizeof(unsigned short));
    
    if (ret != 0) {
        SHDISP_ERR("<RESULT_FAILURE> copy_from_user.\n");
        shdisp_semaphore_end(__func__);
        return ret;
    }
    
    ret = shdisp_SQE_set_flicker_param(alpha);
    
    shdisp_semaphore_end(__func__);

    if (ret != SHDISP_RESULT_SUCCESS) {
        SHDISP_ERR("<RESULT_FAILURE> shdisp_SQE_set_flicker_param.\n");
        return -1;
    }
    
    return 0;
}


/* ------------------------------------------------------------------------- */
/* shdisp_ioctl_get_flicker_param                                            */
/* ------------------------------------------------------------------------- */

static int shdisp_ioctl_get_flicker_param(void __user *argp)
{
    int ret;
    unsigned short alpha;
    
    shdisp_semaphore_start();

    alpha = 0;
    
    ret = shdisp_SQE_get_flicker_param(&alpha);
    
    if (ret != SHDISP_RESULT_SUCCESS) {
        SHDISP_ERR("<RESULT_FAILURE> shdisp_SQE_get_flicker_param.\n");
        shdisp_semaphore_end(__func__);
        return -1;
    }
    
    ret = copy_to_user(argp, &alpha, sizeof(unsigned short));
    
    shdisp_semaphore_end(__func__);

    if (ret != 0) {
        SHDISP_ERR("<RESULT_FAILURE> copy_to_user.\n");
        return ret;
    }
    
    return 0;
}


/* ------------------------------------------------------------------------- */
/* shdisp_ioctl_get_flicker_low_param                                        */
/* ------------------------------------------------------------------------- */

static int shdisp_ioctl_get_flicker_low_param(void __user *argp)
{
    int ret;
    unsigned short alpha;
    
    shdisp_semaphore_start();

    alpha = 0;
    
    ret = shdisp_SQE_get_flicker_low_param(&alpha);
    
    if (ret != SHDISP_RESULT_SUCCESS) {
        SHDISP_ERR("<RESULT_FAILURE> shdisp_SQE_get_flicker_low_param.\n");
        shdisp_semaphore_end(__func__);
        return -1;
    }
    
    ret = copy_to_user(argp, &alpha, sizeof(unsigned short));
    
    shdisp_semaphore_end(__func__);

    if (ret != 0) {
        SHDISP_ERR("<RESULT_FAILURE> copy_to_user.\n");
        return ret;
    }
    
    return 0;
}


/* ------------------------------------------------------------------------- */
/* shdisp_ioctl_bkl_set_auto_mode                                            */
/* ------------------------------------------------------------------------- */

static int shdisp_ioctl_bkl_set_auto_mode(void __user *argp)
{
    int ret;
    int mode;
    struct shdisp_main_bkl_ctl bkl_ctl;
    
    shdisp_semaphore_start();

    ret = copy_from_user(&mode, argp, sizeof(int));
    
    if (ret != 0) {
        SHDISP_ERR("<RESULT_FAILURE> copy_from_user.\n");
        shdisp_semaphore_end(__func__);
        return ret;
    }
    
    if (mode == SHDISP_MAIN_BKL_AUTO_OFF) {
        SHDISP_TRACE("BKL_AUTO_MODE : AUTO_OFF\n");
        bkl_ctl.mode  = SHDISP_MAIN_BKL_MODE_AUTO;
        bkl_ctl.param = SHDISP_MAIN_BKL_PARAM_0;
    }
    else if (mode == SHDISP_MAIN_BKL_AUTO_ON) {
        SHDISP_TRACE("BKL_AUTO_MODE : AUTO_ON\n");
        bkl_ctl.mode  = SHDISP_MAIN_BKL_MODE_AUTO;
        bkl_ctl.param = SHDISP_MAIN_BKL_PARAM_1;
    }
    else {
        SHDISP_ERR("<INVALID_VALUE> mode(%d).\n", mode);
        shdisp_semaphore_end(__func__);
        return -1;
    }
    
    ret = shdisp_SQE_main_bkl_ctl(SHDISP_MAIN_BKL_DEV_TYPE_APP_AUTO, &(bkl_ctl));
    
    shdisp_als_irq_subscribe_bkl_ctrl(SHDISP_BKL_MODE_AUTO);

    shdisp_semaphore_end(__func__);

    if (ret != SHDISP_RESULT_SUCCESS) {
        SHDISP_ERR("<RESULT_FAILURE> shdisp_SQE_main_bkl_ctl.\n");
        return -1;
    }
    
    return 0;
}


/* ------------------------------------------------------------------------- */
/* shdisp_ioctl_bkl_set_dtv_mode                                             */
/* ------------------------------------------------------------------------- */

static int shdisp_ioctl_bkl_set_dtv_mode(void __user *argp)
{
    int ret;
    int dtv_mode;
    
    ret = copy_from_user(&dtv_mode, argp, sizeof(int));
    
    if (ret != 0) {
        SHDISP_ERR("<RESULT_FAILURE> copy_from_user.\n");
        return ret;
    }

    shdisp_semaphore_start();
    
    ret = shdisp_SQE_main_bkl_set_dtv_mode(dtv_mode);
    
    shdisp_semaphore_end(__func__);

    if (ret != SHDISP_RESULT_SUCCESS) {
        SHDISP_ERR("<RESULT_FAILURE> shdisp_SQE_main_bkl_set_dtv_mode.\n");
        return -1;
    }
    
    return 0;
}


/* ------------------------------------------------------------------------- */
/* shdisp_ioctl_bkl_set_emg_mode                                             */
/* ------------------------------------------------------------------------- */

static int shdisp_ioctl_bkl_set_emg_mode(void __user *argp)
{
    int ret;
    int emg_mode;
    
    ret = copy_from_user(&emg_mode, argp, sizeof(int));
    
    if (ret != 0) {
        SHDISP_ERR("<RESULT_FAILURE> copy_from_user.\n");
        return ret;
    }
    
    shdisp_semaphore_start();

    ret = shdisp_SQE_main_bkl_set_emg_mode(emg_mode);

    shdisp_semaphore_end(__func__);
    
    if (ret != SHDISP_RESULT_SUCCESS) {
        SHDISP_ERR("<RESULT_FAILURE> shdisp_SQE_main_bkl_set_emg_mode.\n");
        return -1;
    }
    
    return 0;
}


/* ------------------------------------------------------------------------- */
/* shdisp_ioctl_ledc_power_on                                                */
/* ------------------------------------------------------------------------- */

static int shdisp_ioctl_ledc_power_on(void)
{
    int ret;

    shdisp_semaphore_start();
    
    ret = shdisp_SQE_ledc_power_on();

    shdisp_semaphore_end(__func__);
    
    if (ret != SHDISP_RESULT_SUCCESS) {
        return -1;
    }
    
    return 0;
}


/* ------------------------------------------------------------------------- */
/* shdisp_ioctl_ledc_power_off                                               */
/* ------------------------------------------------------------------------- */

static int shdisp_ioctl_ledc_power_off(void)
{
    int ret;
    
    shdisp_semaphore_start();

    ret = shdisp_SQE_ledc_power_off();

    shdisp_semaphore_end(__func__);
    
    if (ret != SHDISP_RESULT_SUCCESS) {
        return -1;
    }
    
    return 0;
}


/* ------------------------------------------------------------------------- */
/* shdisp_ioctl_ledc_set_rgb                                                 */
/* ------------------------------------------------------------------------- */

static int shdisp_ioctl_ledc_set_rgb(void __user *argp)
{
    int ret;
    struct shdisp_ledc_rgb ledc_rgb;
    
    ret = copy_from_user(&ledc_rgb, argp, sizeof(struct shdisp_ledc_rgb));
    
    if (ret != 0) {
        return ret;
    }
    
    shdisp_semaphore_start();

    ret = shdisp_SQE_ledc_set_rgb(&ledc_rgb);

    shdisp_semaphore_end(__func__);
    
    if (ret != SHDISP_RESULT_SUCCESS) {
        return -1;
    }
    
    return 0;
}


/* ------------------------------------------------------------------------- */
/* shdisp_ioctl_ledc_set_color                                               */
/* ------------------------------------------------------------------------- */

static int shdisp_ioctl_ledc_set_color(void __user *argp)
{
    int ret;
    struct shdisp_ledc_req ledc_req;
    
    ret = copy_from_user(&ledc_req, argp, sizeof(struct shdisp_ledc_req));
    
    if (ret != 0) {
        return ret;
    }
    
    shdisp_semaphore_start();

    ret = shdisp_SQE_ledc_set_color(&ledc_req);

    shdisp_semaphore_end(__func__);
    
    if (ret != SHDISP_RESULT_SUCCESS) {
        return -1;
    }
    
    return 0;
}


/* ------------------------------------------------------------------------- */
/* shdisp_ioctl_ledc_write_reg                                               */
/* ------------------------------------------------------------------------- */

static int shdisp_ioctl_ledc_write_reg(void __user *argp)
{
    int ret;
    struct shdisp_diag_ledc_reg ledc_reg;
    
    ret = copy_from_user(&ledc_reg, argp, sizeof(struct shdisp_diag_ledc_reg));
    
    if (ret != 0) {
        return ret;
    }
    
    shdisp_semaphore_start();

    ret = shdisp_SQE_ledc_write_reg(ledc_reg.reg, ledc_reg.val);
    
    shdisp_semaphore_end(__func__);

    if (ret != SHDISP_RESULT_SUCCESS) {
        return -1;
    }
    
    return 0;
}


/* ------------------------------------------------------------------------- */
/* shdisp_ioctl_ledc_read_reg                                                */
/* ------------------------------------------------------------------------- */

static int shdisp_ioctl_ledc_read_reg(void __user *argp)
{
    int ret;
    struct shdisp_diag_ledc_reg ledc_reg;
    
    ret = copy_from_user(&ledc_reg, argp, sizeof(struct shdisp_diag_ledc_reg));
    
    if (ret != 0) {
        return ret;
    }
    
    shdisp_semaphore_start();

    ret = shdisp_SQE_ledc_read_reg(ledc_reg.reg, &(ledc_reg.val));

    shdisp_semaphore_end(__func__);
    
    if (ret != SHDISP_RESULT_SUCCESS) {
        return -1;
    }
    
    ret = copy_to_user(argp, &ledc_reg, sizeof(struct shdisp_diag_ledc_reg));
    
    if (ret != 0) {
        return ret;
    }
    
    return 0;
}


/* ------------------------------------------------------------------------- */
/* shdisp_ioctl_lux_change_ind                                               */
/* ------------------------------------------------------------------------- */

static int shdisp_ioctl_lux_change_ind(void __user *argp)
{
    int ret;
    struct shdisp_photo_sensor_val val;
   
   
    down(&shdisp_lux_change_sem);
    
    ret = copy_from_user(&val, argp, sizeof(struct shdisp_photo_sensor_val));
    
    if (ret != 0) {
        SHDISP_ERR("<RESULT_FAILURE> copy_from_user.\n");
        up(&shdisp_lux_change_sem);
        return ret;
    }
    
    ret = shdisp_SQE_lux_change_ind(&(val));
    
    if (ret != SHDISP_RESULT_SUCCESS) {
        SHDISP_ERR("<RESULT_FAILURE> shdisp_SQE_lux_change_ind.\n");
    }
    
    ret = copy_to_user(argp, &val, sizeof(struct shdisp_photo_sensor_val));

    up(&shdisp_lux_change_sem);

    
    if (ret != 0) {
        SHDISP_ERR("<RESULT_FAILURE> copy_to_user.\n");
        return ret;
    }
    
    return 0;
}


/* ------------------------------------------------------------------------- */
/* shdisp_ioctl_bkl_set_eco_mode                                             */
/* ------------------------------------------------------------------------- */

static int shdisp_ioctl_bkl_set_eco_mode(void __user *argp)
{
    int ret;
    int eco_mode;
    
    ret = copy_from_user(&eco_mode, argp, sizeof(int));
    
    if (ret != 0) {
        SHDISP_ERR("<RESULT_FAILURE> copy_from_user.\n");
        return ret;
    }
    
    shdisp_semaphore_start();

    ret = shdisp_SQE_main_bkl_set_eco_mode(eco_mode);

    shdisp_semaphore_end(__func__);
    
    if (ret != SHDISP_RESULT_SUCCESS) {
        SHDISP_ERR("<RESULT_FAILURE> shdisp_SQE_main_bkl_set_eco_mode.\n");
        return -1;
    }
    
    return 0;
}


/* ------------------------------------------------------------------------- */
/* shdisp_ioctl_set_cabc                                                     */
/* ------------------------------------------------------------------------- */
static int shdisp_ioctl_set_cabc(void __user *argp)
{
    int ret;
    struct shdisp_main_dbc val;

    ret = copy_from_user(&val, argp, sizeof(struct shdisp_main_dbc));
    
    if (ret != 0) {
        SHDISP_ERR("<RESULT_FAILURE> copy_from_user.\n");
        return ret;
    }
    
    SHDISP_TRACE("shdisp_ioctl_set_cabc : shdisp_SQE_set_cabc S\n");
    SHDISP_TRACE("shdisp_ioctl_set_cabc : val->mode = %d \n",val.mode);
    SHDISP_TRACE("shdisp_ioctl_set_cabc : val->auto_mode = %d \n",val.auto_mode);
    ret = shdisp_SQE_set_cabc(&val);
    SHDISP_TRACE("shdisp_ioctl_set_cabc : val->mode = %d \n",val.mode);
    SHDISP_TRACE("shdisp_ioctl_set_cabc : val->auto_mode = %d \n",val.auto_mode);
    SHDISP_TRACE("shdisp_ioctl_set_cabc : shdisp_SQE_set_cabc E\n"); 
    
    if (ret != SHDISP_RESULT_SUCCESS) {
        SHDISP_ERR("<RESULT_FAILURE> shdisp_SQE_set_cabc.\n");
    }
    
    ret = copy_to_user(argp, &val, sizeof(struct shdisp_main_dbc));
    
    if (ret != 0) {
        SHDISP_ERR("<RESULT_FAILURE> copy_to_user.\n");
        return ret;
    }
    
    return 0;
}


/* ------------------------------------------------------------------------- */
/* shdisp_ioctl_set_cabc_update_param                                          */
/* ------------------------------------------------------------------------- */
static int shdisp_ioctl_set_cabc_update_param(void __user *argp)
{
    int ret;
    struct shdisp_main_dbc val;

    ret = copy_from_user(&val, argp, sizeof(struct shdisp_main_dbc));
    
    if (ret != 0) {
        SHDISP_ERR("<RESULT_FAILURE> copy_from_user.\n");
        return ret;
    }
    
    SHDISP_TRACE("shdisp_ioctl_set_cabc_update_param : shdisp_SQE_set_cabc_update_param S\n");
    SHDISP_TRACE("shdisp_ioctl_set_cabc_update_param : val->mode = %d \n",val.mode);
    SHDISP_TRACE("shdisp_ioctl_set_cabc_update_param : val->auto_mode = %d \n",val.auto_mode);
    ret = shdisp_SQE_set_cabc_update_param(&val);
    SHDISP_TRACE("shdisp_ioctl_set_cabc_update_param : val->mode = %d \n",val.mode);
    SHDISP_TRACE("shdisp_ioctl_set_cabc_update_param : val->auto_mode = %d \n",val.auto_mode);
    SHDISP_TRACE("shdisp_ioctl_set_cabc_update_param : shdisp_SQE_set_cabc_update_param E\n"); 
    
    if (ret != SHDISP_RESULT_SUCCESS) {
        SHDISP_ERR("<RESULT_FAILURE> shdisp_SQE_set_cabc_update_param.\n");
    }
    
    ret = copy_to_user(argp, &val, sizeof(struct shdisp_main_dbc));
    
    if (ret != 0) {
        SHDISP_ERR("<RESULT_FAILURE> copy_to_user.\n");
        return ret;
    }
    
    return 0;
}


/* ------------------------------------------------------------------------- */
/* shdisp_ioctl_bkl_set_chg_mode                                             */
/* ------------------------------------------------------------------------- */

static int shdisp_ioctl_bkl_set_chg_mode(void __user *argp)
{
    int ret;
    int chg_mode;
    
    ret = copy_from_user(&chg_mode, argp, sizeof(int));
    
    if (ret != 0) {
        SHDISP_ERR("<RESULT_FAILURE> copy_from_user.\n");
        return ret;
    }
    
    shdisp_semaphore_start();

    ret = shdisp_SQE_main_bkl_set_chg_mode(chg_mode);

    shdisp_semaphore_end(__func__);
    
    if (ret != SHDISP_RESULT_SUCCESS) {
        SHDISP_ERR("<RESULT_FAILURE> shdisp_SQE_main_bkl_set_chg_mode.\n");
        return -1;
    }
    
    return 0;
}


/* ------------------------------------------------------------------------- */
/* SEQUENCE                                                                  */
/* ------------------------------------------------------------------------- */
/* ------------------------------------------------------------------------- */
/* shdisp_SQE_main_lcd_power_on                                              */
/* ------------------------------------------------------------------------- */

static int shdisp_SQE_main_lcd_power_on(void)
{
    
    if (shdisp_kerl_ctx.main_disp_status == SHDISP_MAIN_DISP_ON) {
        return SHDISP_RESULT_SUCCESS;
    }
    
    shdisp_bdic_API_set_power_mode(SHDISP_BDIC_DEV_TYPE_LCD_PWR, SHDISP_BDIC_DEV_PWR_ON);
    
    shdisp_panel_API_power_on();
    
    
    return SHDISP_RESULT_SUCCESS;
}


/* ------------------------------------------------------------------------- */
/* shdisp_SQE_main_lcd_reset                                                 */
/* ------------------------------------------------------------------------- */

static int shdisp_SQE_main_lcd_reset(void)
{
    
    if (shdisp_kerl_ctx.main_disp_status == SHDISP_MAIN_DISP_ON) {
        return SHDISP_RESULT_SUCCESS;
    }
    
    shdisp_bdic_API_LCD_set_hw_reset();
    shdisp_SYS_delay_us(1000);
    shdisp_bdic_API_LCD_release_hw_reset();
    shdisp_SYS_delay_us(12000);
    
    return SHDISP_RESULT_SUCCESS;
}


/* ------------------------------------------------------------------------- */
/* shdisp_SQE_main_lcd_power_off                                             */
/* ------------------------------------------------------------------------- */

static int shdisp_SQE_main_lcd_power_off(void)
{
    
    if (shdisp_kerl_ctx.main_disp_status == SHDISP_MAIN_DISP_ON) {
        return SHDISP_RESULT_SUCCESS;
    }
    
    shdisp_panel_API_power_off();
    
    shdisp_bdic_API_set_power_mode(SHDISP_BDIC_DEV_TYPE_LCD_PWR, SHDISP_BDIC_DEV_PWR_OFF);
    
    
    return SHDISP_RESULT_SUCCESS;
}


/* ------------------------------------------------------------------------- */
/* shdisp_SQE_main_disp_init_1st                                             */
/* ------------------------------------------------------------------------- */

static int shdisp_SQE_main_disp_init_1st(void)
{
    
    if (shdisp_kerl_ctx.main_disp_status == SHDISP_MAIN_DISP_ON) {
        return SHDISP_RESULT_SUCCESS;
    }
    
    shdisp_panel_API_disp_init_1st();
    
    
    return SHDISP_RESULT_SUCCESS;
}


/* ------------------------------------------------------------------------- */
/* shdisp_SQE_main_disp__init_2nd                                            */
/* ------------------------------------------------------------------------- */

static int shdisp_SQE_main_disp_init_2nd(void)
{
    
    if (shdisp_kerl_ctx.main_disp_status == SHDISP_MAIN_DISP_ON) {
        return SHDISP_RESULT_SUCCESS;
    }
    
    shdisp_panel_API_disp_init_2nd();
    
    
    return SHDISP_RESULT_SUCCESS;
}


/* ------------------------------------------------------------------------- */
/* shdisp_SQE_main_disp_on                                                   */
/* ------------------------------------------------------------------------- */

static int shdisp_SQE_main_disp_on(void)
{
    
    if (shdisp_kerl_ctx.main_disp_status == SHDISP_MAIN_DISP_ON) {
        return SHDISP_RESULT_SUCCESS;
    }
    
    shdisp_panel_API_disp_on();
    
    shdisp_kerl_ctx.main_disp_status = SHDISP_MAIN_DISP_ON;
    
#ifdef CONFIG_SHTERM
    shterm_k_set_info(SHTERM_INFO_LCDPOW, 1);
#endif /* CONFIG_SHTERM */
    
    return SHDISP_RESULT_SUCCESS;
}


/* ------------------------------------------------------------------------- */
/* shdisp_SQE_main_disp_off                                                  */
/* ------------------------------------------------------------------------- */

static int shdisp_SQE_main_disp_off(void)
{
    
    if (shdisp_kerl_ctx.main_disp_status == SHDISP_MAIN_DISP_OFF) {
        return SHDISP_RESULT_SUCCESS;
    }
    

    shdisp_panel_API_sleep();
    shdisp_panel_API_deep_standby();
    
    shdisp_kerl_ctx.main_disp_status = SHDISP_MAIN_DISP_OFF;
    
#ifdef CONFIG_SHTERM
    shterm_k_set_info(SHTERM_INFO_LCDPOW, 0);
#endif /* CONFIG_SHTERM */
    
    return SHDISP_RESULT_SUCCESS;
}


/* ------------------------------------------------------------------------- */
/* shdisp_SQE_main_bkl_ctl                                                   */
/* ------------------------------------------------------------------------- */

static int shdisp_SQE_main_bkl_ctl(int type, struct shdisp_main_bkl_ctl *bkl)
{
    struct shdisp_main_bkl_ctl temp, request;
    unsigned long int notify_value = 0, notify_brightness = 0;
    
    
    if (type >= NUM_SHDISP_MAIN_BKL_DEV_TYPE) {
        SHDISP_ERR("<INVALID_VALUE> type(%d).\n", type);
        return SHDISP_RESULT_FAILURE;
    }
    
    temp.mode  = bkl->mode;
    temp.param = bkl->param;
    shdisp_bdic_API_LCD_BKL_get_request(type, &temp, &request);
    
    if ((request.mode == shdisp_kerl_ctx.main_bkl.mode) && (request.param == shdisp_kerl_ctx.main_bkl.param)) {
        return SHDISP_RESULT_SUCCESS;
    }
    
    if (request.mode != SHDISP_MAIN_BKL_MODE_OFF) {
        shdisp_bdic_API_set_power_mode(SHDISP_BDIC_DEV_TYPE_LCD_BKL, SHDISP_BDIC_DEV_PWR_ON);
    }
    
    switch (request.mode) {
    case SHDISP_MAIN_BKL_MODE_OFF:
        shdisp_bdic_API_LCD_BKL_off();
        notify_value = 0;
        notify_brightness = 0;
        break;
    case SHDISP_MAIN_BKL_MODE_FIX:
        shdisp_bdic_API_LCD_BKL_fix_on(request.param);
        notify_value = 1;
        shdisp_bdic_API_LCD_BKL_get_param( &notify_brightness );
        break;
    case SHDISP_MAIN_BKL_MODE_AUTO:
        shdisp_bdic_API_LCD_BKL_auto_on(request.param);
        notify_value = 1;
        shdisp_bdic_API_LCD_BKL_get_param( &notify_brightness );
        break;
    default:
        break;
    }
    
    if (request.mode == SHDISP_MAIN_BKL_MODE_OFF) {
        shdisp_bdic_API_set_power_mode(SHDISP_BDIC_DEV_TYPE_LCD_BKL, SHDISP_BDIC_DEV_PWR_OFF);
        shdisp_bdic_API_set_power_mode(SHDISP_BDIC_DEV_TYPE_PHOTO_SENSOR_BKL, SHDISP_BDIC_DEV_PWR_OFF);
#if defined(CONFIG_SHDISP_USE_CABC)
        shdisp_bdic_API_set_power_mode(SHDISP_BDIC_DEV_TYPE_PHOTO_SENSOR_LUX, SHDISP_BDIC_DEV_PWR_OFF);
#endif
    }
    else {
        shdisp_bdic_API_set_power_mode(SHDISP_BDIC_DEV_TYPE_PHOTO_SENSOR_BKL, SHDISP_BDIC_DEV_PWR_ON);
    }
    
    shdisp_kerl_ctx.main_bkl.mode  = request.mode;
    shdisp_kerl_ctx.main_bkl.param = request.param;
    
#ifdef CONFIG_SHTERM 
    shterm_k_set_info(SHTERM_INFO_BACKLIGHT, notify_value);
    shterm_k_set_info(SHTERM_INFO_BACKLIGHT_LEV, notify_brightness);
#endif /* CONFIG_SHTERM */
    
    return SHDISP_RESULT_SUCCESS;
}


/* ------------------------------------------------------------------------- */
/* shdisp_SQE_main_bkl_set_dtv_mode                                          */
/* ------------------------------------------------------------------------- */

static int shdisp_SQE_main_bkl_set_dtv_mode(int dtv_mode)
{
    unsigned long int notify_brightness = 0;
    
    
    if (dtv_mode == SHDISP_MAIN_BKL_DTV_OFF) {
        SHDISP_TRACE("BKL_DTV_MODE : DTV_OFF\n");
        shdisp_bdic_API_LCD_BKL_dtv_off();
    }
    else if(dtv_mode == SHDISP_MAIN_BKL_DTV_ON) {
        SHDISP_TRACE("BKL_DTV_MODE : DTV_ON\n");
        shdisp_bdic_API_LCD_BKL_dtv_on();
    }
    else {
        SHDISP_ERR("<INVALID_VALUE> dtv_mode(%d).\n", dtv_mode);
        return SHDISP_RESULT_FAILURE;
    }
    
    shdisp_kerl_ctx.dtv_status = dtv_mode;
    
    
    shdisp_bdic_API_LCD_BKL_get_param( &notify_brightness );
#ifdef CONFIG_SHTERM
    shterm_k_set_info(SHTERM_INFO_BACKLIGHT_LEV, notify_brightness);
#endif /* CONFIG_SHTERM */
    
    return SHDISP_RESULT_SUCCESS;
}


/* ------------------------------------------------------------------------- */
/* shdisp_SQE_main_bkl_set_emg_mode                                          */
/* ------------------------------------------------------------------------- */

static int shdisp_SQE_main_bkl_set_emg_mode(int emg_mode)
{
    unsigned long int notify_brightness = 0;
    
    
    if (emg_mode == SHDISP_MAIN_BKL_EMG_OFF) {
        SHDISP_TRACE("BKL_EMG_MODE : NORMAL\n");
        shdisp_bdic_API_LCD_BKL_emg_off();
    }
    else if(emg_mode == SHDISP_MAIN_BKL_EMG_ON) {
        SHDISP_TRACE("BKL_DTV_MODE : EMERGENCY\n");
        shdisp_bdic_API_LCD_BKL_emg_on();
    }
    else {
        SHDISP_ERR("<INVALID_VALUE> emg_mode(%d).\n", emg_mode);
        return SHDISP_RESULT_FAILURE;
    }
    
    shdisp_kerl_ctx.thermal_status = emg_mode;
    
    
    shdisp_bdic_API_LCD_BKL_get_param( &notify_brightness );
#ifdef CONFIG_SHTERM
    shterm_k_set_info(SHTERM_INFO_BACKLIGHT_LEV, notify_brightness);
#endif /* CONFIG_SHTERM */
    
    return SHDISP_RESULT_SUCCESS;
}


/* ------------------------------------------------------------------------- */
/* shdisp_SQE_main_bkl_set_eco_mode                                          */
/* ------------------------------------------------------------------------- */

static int shdisp_SQE_main_bkl_set_eco_mode(int eco_mode)
{
    unsigned long int notify_brightness = 0;
    
    if (eco_mode == SHDISP_MAIN_BKL_ECO_OFF) {
        SHDISP_TRACE("BKL_ECO_MODE : OFF\n");
        shdisp_bdic_API_LCD_BKL_eco_off();
    }
    else if(eco_mode == SHDISP_MAIN_BKL_ECO_ON) {
        SHDISP_TRACE("BKL_ECO_MODE : ON\n");
        shdisp_bdic_API_LCD_BKL_eco_on();
    }
    else {
        SHDISP_ERR("<INVALID_VALUE> eco_mode(%d).\n", eco_mode);
        return SHDISP_RESULT_FAILURE;
    }
    
    shdisp_kerl_ctx.eco_bkl_status = eco_mode;
    
    shdisp_bdic_API_LCD_BKL_get_param( &notify_brightness );
#ifdef CONFIG_SHTERM
    shterm_k_set_info(SHTERM_INFO_BACKLIGHT_LEV, notify_brightness);
#endif /* CONFIG_SHTERM */
    
    return SHDISP_RESULT_SUCCESS;
}


/* ------------------------------------------------------------------------- */
/* shdisp_SQE_main_bkl_set_chg_mode                                          */
/* ------------------------------------------------------------------------- */

static int shdisp_SQE_main_bkl_set_chg_mode(int chg_mode)
{
    unsigned long int notify_brightness = 0;
    
    if (chg_mode == SHDISP_MAIN_BKL_CHG_OFF) {
        SHDISP_TRACE("BKL_CHG_MODE : OFF\n");
        shdisp_bdic_API_LCD_BKL_chg_off();
    }
    else if(chg_mode == SHDISP_MAIN_BKL_CHG_ON) {
        SHDISP_TRACE("BKL_CHG_MODE : ON\n");
        shdisp_bdic_API_LCD_BKL_chg_on();
    }
    else {
        SHDISP_ERR("<INVALID_VALUE> chg_mode(%d).\n", chg_mode);
        return SHDISP_RESULT_FAILURE;
    }
    
    shdisp_kerl_ctx.usb_chg_status = chg_mode;
    
    shdisp_bdic_API_LCD_BKL_get_param( &notify_brightness );
#ifdef CONFIG_SHTERM
    shterm_k_set_info(SHTERM_INFO_BACKLIGHT_LEV, notify_brightness);
#endif /* CONFIG_SHTERM */
    
    return SHDISP_RESULT_SUCCESS;
}


/* ------------------------------------------------------------------------- */
/* shdisp_SQE_set_host_gpio                                                  */
/* ------------------------------------------------------------------------- */

static int shdisp_SQE_set_host_gpio(struct shdisp_host_gpio *host_gpio)
{
    int ret;
    
    
    ret = shdisp_SYS_set_Host_gpio((host_gpio->num), (host_gpio->value));
    
    if (ret != SHDISP_RESULT_SUCCESS) {
        SHDISP_ERR("<RESULT_FAILURE> shdisp_SYS_set_Host_gpio.\n");
        return SHDISP_RESULT_FAILURE;
    }
    
    
    return SHDISP_RESULT_SUCCESS;
}


/* ------------------------------------------------------------------------- */
/* shdisp_SQE_tri_led_set_color                                              */
/* ------------------------------------------------------------------------- */

static int shdisp_SQE_tri_led_set_color(struct shdisp_tri_led *tri_led)
{
#ifndef CONFIG_MACH_LYNX_DL10
    unsigned char color, xstb_ch012;
    
    
    if (tri_led->led_mode == SHDISP_TRI_LED_MODE_NORMAL) {
        if ((shdisp_kerl_ctx.tri_led.red      == tri_led->red) &&
            (shdisp_kerl_ctx.tri_led.green    == tri_led->green) &&
            (shdisp_kerl_ctx.tri_led.blue     == tri_led->blue) &&
            (shdisp_kerl_ctx.tri_led.ext_mode == tri_led->ext_mode) &&
            (shdisp_kerl_ctx.tri_led.led_mode == tri_led->led_mode)) {
            return SHDISP_RESULT_SUCCESS;
        }
    }
    else {
        if ((shdisp_kerl_ctx.tri_led.red      == tri_led->red) &&
            (shdisp_kerl_ctx.tri_led.green    == tri_led->green) &&
            (shdisp_kerl_ctx.tri_led.blue     == tri_led->blue) &&
            (shdisp_kerl_ctx.tri_led.ext_mode == tri_led->ext_mode) &&
            (shdisp_kerl_ctx.tri_led.led_mode == tri_led->led_mode) &&
            (shdisp_kerl_ctx.tri_led.ontime   == tri_led->ontime) &&
            (shdisp_kerl_ctx.tri_led.interval == tri_led->interval)) {
            return SHDISP_RESULT_SUCCESS;
        }
    }
    
    color = (tri_led->blue << 2) | (tri_led->green << 1) | tri_led->red;
    xstb_ch012 = ((tri_led->red == 0) && (tri_led->green == 0) && (tri_led->blue == 0)) ? 0 : 1;
    
    if (xstb_ch012 != 0) {
        shdisp_bdic_API_set_power_mode(SHDISP_BDIC_DEV_TYPE_TRI_LED, SHDISP_BDIC_DEV_PWR_ON);
        
        switch (tri_led->led_mode){
        case SHDISP_TRI_LED_MODE_NORMAL:
            shdisp_bdic_API_TRI_LED_normal_on( color );
            break;
        case SHDISP_TRI_LED_MODE_BLINK:
            shdisp_bdic_API_TRI_LED_blink_on( color, tri_led->ontime, tri_led->interval );
            break;
        case SHDISP_TRI_LED_MODE_FIREFLY:
            shdisp_bdic_API_TRI_LED_firefly_on( color, tri_led->ontime, tri_led->interval );
            break;
        default:
            break;
        }
        
        if (tri_led->led_mode == SHDISP_TRI_LED_MODE_NORMAL) {
            shdisp_bdic_API_set_power_mode(SHDISP_BDIC_DEV_TYPE_TRI_LED_ANIME, SHDISP_BDIC_DEV_PWR_OFF);
        }
        else {
            shdisp_bdic_API_set_power_mode(SHDISP_BDIC_DEV_TYPE_TRI_LED_ANIME, SHDISP_BDIC_DEV_PWR_ON);
        }
    }
    else {
        shdisp_bdic_API_TRI_LED_off();
    }
    
    if ((xstb_ch012 == 0) ||
        ((tri_led->led_mode != SHDISP_TRI_LED_MODE_NORMAL) && (tri_led->interval != SHDISP_TRI_LED_INTERVAL_NONE))) {
        shdisp_bdic_API_set_power_mode(SHDISP_BDIC_DEV_TYPE_TRI_LED, SHDISP_BDIC_DEV_PWR_OFF);
    }
    
    shdisp_kerl_ctx.tri_led.red      = tri_led->red;
    shdisp_kerl_ctx.tri_led.green    = tri_led->green;
    shdisp_kerl_ctx.tri_led.blue     = tri_led->blue;
    shdisp_kerl_ctx.tri_led.led_mode = tri_led->led_mode;
    if ((tri_led->led_mode == SHDISP_TRI_LED_MODE_BLINK) ||
        (tri_led->led_mode == SHDISP_TRI_LED_MODE_FIREFLY)) {
        shdisp_kerl_ctx.tri_led.ontime   = tri_led->ontime;
        shdisp_kerl_ctx.tri_led.interval = tri_led->interval;
    }
    
    
#endif
    return SHDISP_RESULT_SUCCESS;
}


/* ------------------------------------------------------------------------- */
/* shdisp_SQE_bdic_write_reg                                                 */
/* ------------------------------------------------------------------------- */

static int shdisp_SQE_bdic_write_reg(unsigned char reg, unsigned char val)
{
    
    shdisp_bdic_API_DIAG_write_reg(reg, val);
    
    
    return SHDISP_RESULT_SUCCESS;
}


/* ------------------------------------------------------------------------- */
/* shdisp_SQE_bdic_read_reg                                                  */
/* ------------------------------------------------------------------------- */

static int shdisp_SQE_bdic_read_reg(unsigned char reg, unsigned char *val)
{
    
    shdisp_bdic_API_DIAG_read_reg(reg, val);
    
    
    return SHDISP_RESULT_SUCCESS;
}


/* ------------------------------------------------------------------------- */
/* shdisp_SQE_bdic_multi_read_reg                                            */
/* ------------------------------------------------------------------------- */

static int shdisp_SQE_bdic_multi_read_reg(unsigned char reg, unsigned char *val, int size)
{
    int ret = 0;
    
    ret = shdisp_bdic_API_DIAG_multi_read_reg(reg, val, size);
    
    return ret;
}


/* ------------------------------------------------------------------------- */
/* shdisp_SQE_get_lux                                                        */
/* ------------------------------------------------------------------------- */

static int shdisp_SQE_get_lux(struct shdisp_photo_sensor_val *value)
{
    int ret;
    
    ret = shdisp_bdic_API_PHOTO_SENSOR_get_lux(&(value->value), &(value->lux));
    
    if (ret != SHDISP_RESULT_SUCCESS) {
        SHDISP_ERR("<RESULT_FAILURE> shdisp_bdic_API_PHOTO_SENSOR_get_lux.\n");
        value->result = SHDISP_RESULT_FAILURE;
    }
    else {
        value->result = SHDISP_RESULT_SUCCESS;
    }
    
    return ret;
}


/* ------------------------------------------------------------------------- */
/* shdisp_SQE_change_level_lut                                          */
/* ------------------------------------------------------------------------- */

static int shdisp_SQE_change_level_lut(void)
{
    int level_lut = SHDISP_MAIN_DISP_CABC_LUT0;

    level_lut = shdisp_bdic_API_change_level_lut();

    return level_lut;
}


/* ------------------------------------------------------------------------- */
/* shdisp_SQE_pwm_config                                                     */
/* ------------------------------------------------------------------------- */

static int shdisp_SQE_pwm_config(int duty)
{
    int ret;
    
    ret = shdisp_api_get_panel_info();
    if (ret == 1) {
        SHDISP_TRACE("[SHDISP]shdisp_SQE_pwm_config : not set duty=%d \n", duty);
        return SHDISP_RESULT_SUCCESS;
    }
    if (shdisp_kerl_ctx.main_bkl.mode == SHDISP_MAIN_BKL_MODE_OFF) {
        return SHDISP_RESULT_SUCCESS;
    }
    SHDISP_TRACE("[SHDISP]shdisp_SQE_pwm_config : set duty=%d \n", duty);
    
    ret = shdisp_PWM_request();
    
    if (ret != SHDISP_RESULT_SUCCESS) {
        SHDISP_ERR("<RESULT_FAILURE> shdisp_SQE_pwm_config.\n");
        return SHDISP_RESULT_FAILURE;
    }
    
    ret = shdisp_PWM_config(duty);
    
    if (ret != SHDISP_RESULT_SUCCESS) {
        SHDISP_ERR("<RESULT_FAILURE> shdisp_PWM_config.\n");
        return SHDISP_RESULT_FAILURE;
    }
    
    ret = shdisp_PWM_enable();
    
    if (ret != SHDISP_RESULT_SUCCESS) {
        SHDISP_ERR("<RESULT_FAILURE> shdisp_PWM_enable.\n");
        return SHDISP_RESULT_FAILURE;
    }
    
    shdisp_bdic_API_LCD_BKL_pwm_enable();
    
    return SHDISP_RESULT_SUCCESS;
}


/* ------------------------------------------------------------------------- */
/* shdisp_SQE_pwm_stop                                                       */
/* ------------------------------------------------------------------------- */

static int shdisp_SQE_pwm_stop(void)
{
    int ret;

    ret = shdisp_api_get_panel_info();
    if (ret == 1) {
        SHDISP_TRACE("[SHDISP]shdisp_SQE_pwm_stop : not set \n");
        return SHDISP_RESULT_SUCCESS;
    }
    SHDISP_TRACE("[SHDISP]shdisp_SQE_pwm_stop : set \n");

    shdisp_bdic_API_LCD_BKL_pwm_disable();
    
    shdisp_PWM_disable();
    shdisp_PWM_release();
    
    return SHDISP_RESULT_SUCCESS;
}


/* ------------------------------------------------------------------------- */
/* shdisp_SQE_write_bdic_i2c                                                 */
/* ------------------------------------------------------------------------- */

static int shdisp_SQE_write_bdic_i2c(struct shdisp_bdic_i2c_msg *i2c_msg)
{
    int ret;
    
    ret = shdisp_bdic_API_i2c_transfer(i2c_msg);
    
    if (ret == SHDISP_RESULT_FAILURE_I2C_TMO) {
        SHDISP_ERR("<TIME_OUT> shdisp_bdic_API_i2c_transfer.\n");
        return SHDISP_RESULT_FAILURE_I2C_TMO;
    }
    else if (ret == SHDISP_RESULT_FAILURE) {
        SHDISP_ERR("<RESULT_FAILURE> shdisp_bdic_API_i2c_transfer.\n");
        return SHDISP_RESULT_FAILURE;
    }
    
    return SHDISP_RESULT_SUCCESS;
}


/* ------------------------------------------------------------------------- */
/* shdisp_SQE_read_bdic_i2c                                                  */
/* ------------------------------------------------------------------------- */

static int shdisp_SQE_read_bdic_i2c(struct shdisp_bdic_i2c_msg *i2c_msg)
{
    int ret;
    
    ret = shdisp_bdic_API_i2c_transfer(i2c_msg);
    
    if (ret == SHDISP_RESULT_FAILURE_I2C_TMO) {
        SHDISP_ERR("<TIME_OUT> shdisp_bdic_API_i2c_transfer.\n");
        return SHDISP_RESULT_FAILURE_I2C_TMO;
    }
    else if (ret == SHDISP_RESULT_FAILURE) {
        SHDISP_ERR("<RESULT_FAILURE> shdisp_bdic_API_i2c_transfer.\n");
        return SHDISP_RESULT_FAILURE;
    }
    
    return SHDISP_RESULT_SUCCESS;
}


/* ------------------------------------------------------------------------- */
/* shdisp_SQE_bdic_i2c_error_handling                                        */
/* ------------------------------------------------------------------------- */

static int shdisp_SQE_bdic_i2c_error_handling(void)
{
    void (*temp_callback)(void);

    SHDISP_TRACE("shdisp_SQE_bdic_i2c_error_handling : start\n");
    shdisp_semaphore_start();
    shdisp_bdic_API_IRQ_backup_irq_reg();
    shdisp_bdic_API_IRQ_irq_reg_i2c_mask();
    shdisp_bdic_API_i2c_error_power_off();
    shdisp_semaphore_end(__func__);

    shdisp_SYS_delay_us(10000);

    shdisp_semaphore_start();
    shdisp_bdic_API_IRQ_i2c_error_Clear();
    shdisp_bdic_API_i2c_error_power_recovery();
    shdisp_semaphore_end(__func__);

    down(&shdisp_sem_callback);
    temp_callback = shdisp_callback_table[SHDISP_IRQ_TYPE_PS];
    up(&shdisp_sem_callback);
    
    if ( temp_callback != NULL ) {
        (*temp_callback)();
    }

    shdisp_semaphore_start();
    shdisp_bdic_API_IRQ_return_irq_reg();
    shdisp_semaphore_end(__func__);

    down(&shdisp_sem_callback);
    temp_callback = shdisp_callback_table[SHDISP_IRQ_TYPE_ALS];
    up(&shdisp_sem_callback);
    
    if ( temp_callback != NULL ) {
        (*temp_callback)();
    }

    SHDISP_TRACE("shdisp_SQE_bdic_i2c_error_handling : Finish\n");
    return SHDISP_RESULT_SUCCESS;
}


/* ------------------------------------------------------------------------- */
/* shdisp_SQE_photo_sensor_pow_ctl                                           */
/* ------------------------------------------------------------------------- */

static int shdisp_SQE_photo_sensor_pow_ctl(struct shdisp_photo_sensor_power_ctl *ctl)
{
    int ret;
    int type, power;
    int param_chk = 0;

    struct shdisp_subscribe strctsubs;

    
    switch (ctl->type) {
    case SHDISP_PHOTO_SENSOR_TYPE_APP:
        type = SHDISP_BDIC_DEV_TYPE_PHOTO_SENSOR_APP;
        break;
    case SHDISP_PHOTO_SENSOR_TYPE_LUX:
        type = SHDISP_BDIC_DEV_TYPE_PHOTO_SENSOR_LUX;
        break;
    default:
        type = 0;
        param_chk = 1;
        SHDISP_ERR("<INVALID_VALUE> ctl->type(%d).\n", ctl->type);
        break;
    }
    
    switch (ctl->power) {
    case SHDISP_PHOTO_SENSOR_DISABLE:
        power = SHDISP_BDIC_DEV_PWR_OFF;
        break;
    case SHDISP_PHOTO_SENSOR_ENABLE:
        power = SHDISP_BDIC_DEV_PWR_ON;
        break;
    default:
        power = 0;
        param_chk = 1;
        SHDISP_ERR("<INVALID_VALUE> ctl->power(%d).\n", ctl->power);
        break;
    }
    
    if (param_chk == 1) {
        return SHDISP_RESULT_FAILURE;
    }
    
    ret = shdisp_bdic_API_set_power_mode(type, power);
    
    if (ret != SHDISP_RESULT_SUCCESS) {
        SHDISP_ERR("<RESULT_FAILURE> shdisp_bdic_API_set_power_mode.\n");
        return SHDISP_RESULT_FAILURE;
    }
    
    
    if( ctl->type == SHDISP_PHOTO_SENSOR_TYPE_LUX ){
        while( 1 ){
            SHDISP_TRACE("PHOTO_SENSOR Lux Power = %d\n", ctl->power );
            shdisp_als_irq_subscribe_type = SHDISP_ALS_IRQ_REQ_DBC;
            if( ctl->power == SHDISP_PHOTO_SENSOR_ENABLE ){
                strctsubs.irq_type = SHDISP_IRQ_TYPE_ALS;
                strctsubs.callback = shdisp_photo_lux_irq;
                
                if(shdisp_subscribe_type_table[SHDISP_IRQ_TYPE_ALS] == SHDISP_SUBSCRIBE_TYPE_INT){
                    if( shdisp_als_irq_req_state != 0 ){
                        shdisp_als_irq_req_state |= shdisp_als_irq_subscribe_type;
                        break;
                    };
                }
                shdisp_SQE_event_subscribe(&strctsubs);
            }
            else if( ctl->power == SHDISP_PHOTO_SENSOR_DISABLE ){
                if(shdisp_subscribe_type_table[SHDISP_IRQ_TYPE_ALS] == SHDISP_SUBSCRIBE_TYPE_INT){
                    shdisp_als_irq_req_state &= ~shdisp_als_irq_subscribe_type;
                    if( shdisp_als_irq_req_state != 0 ){
                        break;
                    }
                }
                if (delayed_work_pending(&shdisp_sensor_start_wk)) {
                    cancel_delayed_work(&shdisp_sensor_start_wk);
                    flush_workqueue(shdisp_wq_sensor_start);
                    SHDISP_TRACE("shdisp_api_event_unsubscribe:cancel_delayed_work\n");
                }
                shdisp_SQE_event_unsubscribe(SHDISP_IRQ_TYPE_ALS);
            }
            break;
        }
    }

    return SHDISP_RESULT_SUCCESS;
}


/* ------------------------------------------------------------------------- */
/* shdisp_SQE_panel_write_reg                                                */
/* ------------------------------------------------------------------------- */

static int shdisp_SQE_panel_write_reg(struct shdisp_lcddr_reg *panel_reg)
{
    int ret;
    
    
    ret = shdisp_panel_API_diag_write_reg(panel_reg->address, panel_reg->buf, panel_reg->size);
    
    if (ret != SHDISP_RESULT_SUCCESS) {
        SHDISP_ERR("<RESULT_FAILURE> shdisp_panel_API_diag_write_reg.\n");
        return SHDISP_RESULT_FAILURE;
    }
    
    
    return SHDISP_RESULT_SUCCESS;
}


/* ------------------------------------------------------------------------- */
/* shdisp_SQE_panel_read_reg                                                 */
/* ------------------------------------------------------------------------- */

static int shdisp_SQE_panel_read_reg(struct shdisp_lcddr_reg *panel_reg)
{
    int ret;
    
    
    ret = shdisp_panel_API_diag_read_reg(panel_reg->address, panel_reg->buf, panel_reg->size);
    
    if (ret != SHDISP_RESULT_SUCCESS) {
        SHDISP_ERR("<RESULT_FAILURE> shdisp_panel_API_diag_read_reg.\n");
        return SHDISP_RESULT_FAILURE;
    }
    
    
    return SHDISP_RESULT_SUCCESS;
}


/* ------------------------------------------------------------------------- */
/* shdisp_SQE_prox_sensor_pow_ctl                                            */
/* ------------------------------------------------------------------------- */

static int shdisp_SQE_prox_sensor_pow_ctl(int power_mode)
{
    int ret;
    int power = SHDISP_BDIC_DEV_PWR_OFF;
    
    
    switch (power_mode) {
    case SHDISP_PROX_SENSOR_POWER_OFF:
        power = SHDISP_BDIC_DEV_PWR_OFF;
        break;
    case SHDISP_PROX_SENSOR_POWER_ON:
        power = SHDISP_BDIC_DEV_PWR_ON;
        break;
    default:
        break;
    }
    
    ret = shdisp_bdic_API_set_power_mode(SHDISP_BDIC_DEV_TYPE_PROX_SENSOR, power);
    
    if (ret != SHDISP_RESULT_SUCCESS) {
        SHDISP_ERR("<RESULT_FAILURE> shdisp_bdic_API_set_power_mode.\n");
        return SHDISP_RESULT_FAILURE;
    }
    
    return SHDISP_RESULT_SUCCESS;
}


/* ------------------------------------------------------------------------- */
/* shdisp_SQE_set_flicker_param                                              */
/* ------------------------------------------------------------------------- */

static int shdisp_SQE_set_flicker_param(unsigned short alpha)
{
    int ret;
    
    
    if (shdisp_kerl_ctx.main_disp_status != SHDISP_MAIN_DISP_ON) {
        return SHDISP_RESULT_SUCCESS;
    }
    
    ret = shdisp_panel_API_diag_set_flicker_param(alpha);
    
    if (ret != SHDISP_RESULT_SUCCESS) {
        SHDISP_ERR("<RESULT_FAILURE> shdisp_panel_API_diag_set_flicker_param.\n");
        return SHDISP_RESULT_FAILURE;
    }
    else {
        shdisp_kerl_ctx.alpha = alpha;
        shdisp_kerl_ctx.alpha_low = alpha;
    }
    
    
    return SHDISP_RESULT_SUCCESS;
}


/* ------------------------------------------------------------------------- */
/* shdisp_SQE_get_flicker_param                                              */
/* ------------------------------------------------------------------------- */

static int shdisp_SQE_get_flicker_param(unsigned short *alpha)
{
    int ret;
    
    
    if (shdisp_kerl_ctx.main_disp_status != SHDISP_MAIN_DISP_ON) {
        return SHDISP_RESULT_SUCCESS;
    }
    
    ret = shdisp_panel_API_diag_get_flicker_param(alpha);
    
    if (ret != SHDISP_RESULT_SUCCESS) {
        SHDISP_ERR("<RESULT_FAILURE> shdisp_panel_API_diag_get_flicker_param.\n");
        return SHDISP_RESULT_FAILURE;
    }
    
    
    return SHDISP_RESULT_SUCCESS;
}


/* ------------------------------------------------------------------------- */
/* shdisp_SQE_get_flicker_low_param                                          */
/* ------------------------------------------------------------------------- */

static int shdisp_SQE_get_flicker_low_param(unsigned short *alpha)
{
    int ret;
    
    
    if (shdisp_kerl_ctx.main_disp_status != SHDISP_MAIN_DISP_ON) {
        return SHDISP_RESULT_SUCCESS;
    }
    
    ret = shdisp_panel_API_diag_get_flicker_low_param(alpha);
    
    if (ret != SHDISP_RESULT_SUCCESS) {
        SHDISP_ERR("<RESULT_FAILURE> shdisp_panel_API_diag_get_flicker_low_param.\n");
        return SHDISP_RESULT_FAILURE;
    }
    
    
    return SHDISP_RESULT_SUCCESS;
}


/* ------------------------------------------------------------------------- */
/* shdisp_SQE_ledc_power_on                                                  */
/* ------------------------------------------------------------------------- */

static int shdisp_SQE_ledc_power_on(void)
{
#ifdef CONFIG_SHLCDC_LED_BD2802GU
    
    shdisp_ledc_API_power_on();
    
    
#endif
    return SHDISP_RESULT_SUCCESS;
}


/* ------------------------------------------------------------------------- */
/* shdisp_SQE_ledc_power_off                                                 */
/* ------------------------------------------------------------------------- */

static int shdisp_SQE_ledc_power_off(void)
{
#ifdef CONFIG_SHLCDC_LED_BD2802GU
    
    shdisp_ledc_API_power_off();
    
    
#endif
    return SHDISP_RESULT_SUCCESS;
}


/* ------------------------------------------------------------------------- */
/* shdisp_SQE_ledc_set_rgb                                                   */
/* ------------------------------------------------------------------------- */

static int shdisp_SQE_ledc_set_rgb(struct shdisp_ledc_rgb *ledc_rgb)
{
#ifdef CONFIG_SHLCDC_LED_BD2802GU
    
    shdisp_ledc_API_set_rgb(ledc_rgb);
    
    
#endif
    return SHDISP_RESULT_SUCCESS;
}


/* ------------------------------------------------------------------------- */
/* shdisp_SQE_ledc_set_color                                                 */
/* ------------------------------------------------------------------------- */

static int shdisp_SQE_ledc_set_color(struct shdisp_ledc_req *ledc_req)
{
#ifdef CONFIG_SHLCDC_LED_BD2802GU
    
    shdisp_ledc_API_set_color(ledc_req);
    
    
#endif
    return SHDISP_RESULT_SUCCESS;
}


/* ------------------------------------------------------------------------- */
/* shdisp_SQE_ledc_write_reg                                                 */
/* ------------------------------------------------------------------------- */

static int shdisp_SQE_ledc_write_reg(unsigned char reg, unsigned char val)
{
#ifdef CONFIG_SHLCDC_LED_BD2802GU
    
    shdisp_ledc_API_DIAG_write_reg(reg, val);
    
    
#endif
    return SHDISP_RESULT_SUCCESS;
}


/* ------------------------------------------------------------------------- */
/* shdisp_SQE_ledc_read_reg                                                  */
/* ------------------------------------------------------------------------- */

static int shdisp_SQE_ledc_read_reg(unsigned char reg, unsigned char *val)
{
#ifdef CONFIG_SHLCDC_LED_BD2802GU
    
    shdisp_ledc_API_DIAG_read_reg(reg, val);
    
    
#endif
    return SHDISP_RESULT_SUCCESS;
}


/* ------------------------------------------------------------------------- */
/* shdisp_SQE_check_recovery                                                 */
/* ------------------------------------------------------------------------- */
static int shdisp_SQE_check_recovery(void)
{
    int ret;
    
    ret = shdisp_panel_API_check_recovery();
    
    if (ret != SHDISP_RESULT_SUCCESS) {
        SHDISP_ERR("<RESULT_FAILURE> shdisp_panel_API_check_recovery.\n");
        return SHDISP_RESULT_FAILURE;
    }
    
    return SHDISP_RESULT_SUCCESS;
}


/* ------------------------------------------------------------------------- */
/* shdisp_SQE_do_recovery                                                    */
/* ------------------------------------------------------------------------- */
static int shdisp_SQE_do_recovery(void)
{
    int ret;

    ret = shdisp_bdic_API_RECOVERY_check_bdic_practical();
    
    if (ret != SHDISP_RESULT_SUCCESS) {
        SHDISP_TRACE("[SHDISP] shdisp_SQE_do_recovery bdic reset disable\n");
        return SHDISP_RESULT_SUCCESS;
    }
    

#if 1
    SHDISP_TRACE("[SHDISP] shdisp_SQE_do_recovery bdic shutdown start\n");

    shdisp_bdic_API_bdic_shutdown();

    shdisp_bdic_API_RECOVERY_resume_bdic_status();
    
#endif

    return SHDISP_RESULT_SUCCESS;
}


/* ------------------------------------------------------------------------- */
/* shdisp_SQE_event_subscribe                                                */
/* ------------------------------------------------------------------------- */

static int shdisp_SQE_event_subscribe(struct shdisp_subscribe *subscribe)
{
    int ret;
    int i;
    int bAllNull = 0;
    
    if (shdisp_subscribe_type_table[subscribe->irq_type] == SHDISP_SUBSCRIBE_TYPE_INT) {
        ret = shdisp_bdic_API_IRQ_check_type( subscribe->irq_type );
        if( ret != SHDISP_RESULT_SUCCESS ){
            SHDISP_ERR("<RESULT_FAILURE> shdisp_bdic_API_IRQ_check_type.\n");
            return SHDISP_RESULT_FAILURE;
        }
    }
    
    down(&shdisp_sem_callback);
    
    bAllNull = 1;
    for (i=0; i< NUM_SHDISP_IRQ_TYPE ; i++) {
        if ((shdisp_subscribe_type_table[i] == SHDISP_SUBSCRIBE_TYPE_INT) &&
            (shdisp_callback_table[i] != NULL)) {
                bAllNull = 0;
            }
    }
    
    if ( shdisp_callback_table[subscribe->irq_type] != NULL ) {
        SHDISP_TRACE("INT_SUBSCRIBE CHANGE(irq_type=%d)\n", subscribe->irq_type);
    }
    else {
        SHDISP_TRACE("INT_SUBSCRIBE NEW ENTRY(irq_type=%d)\n", subscribe->irq_type);
    }
    
    shdisp_callback_table[subscribe->irq_type] = subscribe->callback;

    if (shdisp_subscribe_type_table[subscribe->irq_type] == SHDISP_SUBSCRIBE_TYPE_INT) {
        if( subscribe->irq_type == SHDISP_IRQ_TYPE_ALS )
            shdisp_als_irq_req_state |= shdisp_als_irq_subscribe_type;
        shdisp_bdic_API_IRQ_set_reg( subscribe->irq_type, SHDISP_IRQ_NO_MASK );
    }
    else {
        shdisp_timer_int_register();
    }
    
    up(&shdisp_sem_callback);
    
    if (shdisp_subscribe_type_table[subscribe->irq_type] == SHDISP_SUBSCRIBE_TYPE_INT) {
        if ( bAllNull ) {
            SHDISP_TRACE("INT_SUBSCRIBE enable_irq\n");
            shdisp_SYS_set_irq(SHDISP_IRQ_ENABLE);
        }
    }
    
    return SHDISP_RESULT_SUCCESS;
}


/* ------------------------------------------------------------------------- */
/* shdisp_SQE_event_unsubscribe                                              */
/* ------------------------------------------------------------------------- */

static int shdisp_SQE_event_unsubscribe(int irq_type)
{
    int ret;
    int i;
    int bAllNull = 0;
    
    if (shdisp_subscribe_type_table[irq_type] == SHDISP_SUBSCRIBE_TYPE_INT) {
        ret = shdisp_bdic_API_IRQ_check_type( irq_type );
        if ( ret !=  SHDISP_RESULT_SUCCESS ) {
            SHDISP_ERR("<RESULT_FAILURE> shdisp_bdic_API_IRQ_check_type.\n");
            return SHDISP_RESULT_FAILURE;
        }
    }

    down(&shdisp_sem_callback);

    if ( shdisp_callback_table[irq_type] == NULL ) {
        SHDISP_TRACE("INT_UNSUBSCRIBE DONE(irq_type=%d)\n", irq_type);
    }
    else {
        if (shdisp_subscribe_type_table[irq_type] == SHDISP_SUBSCRIBE_TYPE_INT) {
            shdisp_bdic_API_IRQ_set_reg( irq_type, SHDISP_IRQ_MASK );
        }
        else {
            shdisp_timer_int_delete();
        }
        
        shdisp_callback_table[irq_type] = NULL;
        
        if (shdisp_subscribe_type_table[irq_type] == SHDISP_SUBSCRIBE_TYPE_INT) {
            bAllNull = 1;
            for ( i=0; i< NUM_SHDISP_IRQ_TYPE ; i++) {
                if ((shdisp_subscribe_type_table[i] == SHDISP_SUBSCRIBE_TYPE_INT) &&
                    (shdisp_callback_table[i] != NULL)) {
                        bAllNull = 0;
                    }
            }
            if ( bAllNull ) {
                shdisp_SYS_set_irq(SHDISP_IRQ_DISABLE);
                SHDISP_TRACE("INT_UNSUBSCRIBE disable_irq\n");
            }
        }
        
        SHDISP_TRACE("INT_UNSUBSCRIBE SUCCESS(irq_type=%d)\n", irq_type);
    }

    up(&shdisp_sem_callback);
    
    return SHDISP_RESULT_SUCCESS;
}


/* ------------------------------------------------------------------------- */
/* shdisp_SQE_lux_change_ind                                                 */
/* ------------------------------------------------------------------------- */

static int shdisp_SQE_lux_change_ind(struct shdisp_photo_sensor_val *value)
{
    int ret;

    SHDISP_TRACE("shdisp_SQE_lux_change_ind : wait complete\n");

    lux_change_wait_flg = SHDISP_LUX_CHANGE_STATE_WAIT;
    INIT_COMPLETION(lux_change_notify);
    ret = wait_for_completion_interruptible(&lux_change_notify);
    if(ret != 0){
        value->result = SHDISP_RESULT_FAILURE;
        return SHDISP_RESULT_FAILURE;
    }

    if(lux_change_wait_flg == SHDISP_LUX_CHANGE_STATE_EXIT){
        value->result = SHDISP_RESULT_FAILURE;
        return SHDISP_RESULT_FAILURE;
    }

    SHDISP_TRACE("shdisp_SQE_lux_change_ind : wake up by complete\n");
    
    shdisp_semaphore_start();
    
    ret = shdisp_bdic_API_PHOTO_SENSOR_lux_change_ind(&(value->mode));

    if (ret != SHDISP_RESULT_SUCCESS) {
        SHDISP_ERR("<RESULT_FAILURE> shdisp_bdic_API_PHOTO_SENSOR_lux_change_ind.\n");
        value->result = SHDISP_RESULT_FAILURE;
    }
    else {
        value->result = SHDISP_RESULT_SUCCESS;
    }
    
    ret = shdisp_SQE_get_lux(value);
    
    shdisp_semaphore_end(__func__);
    
    if (ret != SHDISP_RESULT_SUCCESS) {
        SHDISP_ERR("<RESULT_FAILURE> shdisp_SQE_get_lux.\n");
    }
    
    return ret;
}


/* ------------------------------------------------------------------------- */
/* shdisp_SQE_sensor_start_task                                              */
/* ------------------------------------------------------------------------- */

static int shdisp_SQE_sensor_start_task(void)
{
    int ret;
    
    ret = shdisp_bdic_API_SENSOR_start( shdisp_new_opt_mode );
    return ret;
}


/* ------------------------------------------------------------------------- */
/* shdisp_SQE_set_cabc                                                       */
/* ------------------------------------------------------------------------- */

static int shdisp_SQE_set_cabc(struct shdisp_main_dbc *value)
{
    int ret;

    SHDISP_TRACE("shdisp_SQE_set_cabc : value->mode = %d \n", value->mode);
    SHDISP_TRACE("shdisp_SQE_set_cabc : value->auto_mode = %d \n", value->auto_mode);

    if(!(value->mode == 0 || value->mode == 1))
    {
        SHDISP_ERR("<RESULT_FAILURE> shdisp_SQE_set_cabc : value->mode .\n");
        return SHDISP_RESULT_FAILURE;
    }
    if(!(value->auto_mode == 0 || value->auto_mode == 1))
    {
        SHDISP_ERR("<RESULT_FAILURE> shdisp_SQE_set_cabc : value->auto_mode .\n");
        return SHDISP_RESULT_FAILURE;
    }
    
    if(value->mode == 0 && value->auto_mode == 0 )
    {
        SHDISP_TRACE("shdisp_SQE_set_cabc : SHDISP_MAIN_DISP_CABC_MODE_OFF \n");
        cabc_mode_set = SHDISP_MAIN_DISP_CABC_MODE_OFF;
    }
    else if(value->mode == 1 && value->auto_mode == 0 )
    {
        SHDISP_TRACE("shdisp_SQE_set_cabc : SHDISP_MAIN_DISP_CABC_MODE_DBC \n");
        cabc_mode_set = SHDISP_MAIN_DISP_CABC_MODE_DBC;
    }
    else if(value->mode == 0 && value->auto_mode == 1 )
    {
        SHDISP_TRACE("shdisp_SQE_set_cabc : SHDISP_MAIN_DISP_CABC_MODE_ACC \n");
        cabc_mode_set = SHDISP_MAIN_DISP_CABC_MODE_ACC;
    }
    else if(value->mode == 1 && value->auto_mode == 1 )
    {
        SHDISP_TRACE("shdisp_SQE_set_cabc : SHDISP_MAIN_DISP_CABC_MODE_DBC_ACC \n");
        cabc_mode_set = SHDISP_MAIN_DISP_CABC_MODE_DBC_ACC;
    }

    ret = shdisp_SQE_cabc_ctrl();
    
    return ret;
}

/* ------------------------------------------------------------------------- */
/* shdisp_SQE_set_cabc_update_param                                          */
/* ------------------------------------------------------------------------- */

static int shdisp_SQE_set_cabc_update_param(struct shdisp_main_dbc *value)
{
    int ret = SHDISP_RESULT_SUCCESS;

    SHDISP_TRACE("shdisp_SQE_set_cabc_update_param : value->mode = %d \n", value->mode);
    SHDISP_TRACE("shdisp_SQE_set_cabc_update_param : value->auto_mode = %d \n", value->auto_mode);

    if(!(value->mode == 0 || value->mode == 1))
    {
        SHDISP_ERR("<RESULT_FAILURE> shdisp_SQE_set_cabc_update_param : value->mode .\n");
        return SHDISP_RESULT_FAILURE;
    }
    if(!(value->auto_mode == 0 || value->auto_mode == 1))
    {
        SHDISP_ERR("<RESULT_FAILURE> shdisp_SQE_set_cabc_update_param : value->auto_mode .\n");
        return SHDISP_RESULT_FAILURE;
    }
    if(value->mode == 0 && value->auto_mode == 0 )
    {
        SHDISP_TRACE("shdisp_SQE_set_cabc_update_param : SHDISP_MAIN_DISP_CABC_MODE_OFF \n");
        cabc_mode_set = SHDISP_MAIN_DISP_CABC_MODE_OFF;
    }
    else if(value->mode == 1 && value->auto_mode == 0 )
    {
        SHDISP_TRACE("shdisp_SQE_set_cabc_update_param : SHDISP_MAIN_DISP_CABC_MODE_DBC \n");
        cabc_mode_set = SHDISP_MAIN_DISP_CABC_MODE_DBC;
    }
    else if(value->mode == 0 && value->auto_mode == 1 )
    {
        SHDISP_TRACE("shdisp_SQE_set_cabc_update_param : SHDISP_MAIN_DISP_CABC_MODE_ACC \n");
        cabc_mode_set = SHDISP_MAIN_DISP_CABC_MODE_ACC;
    }
    else if(value->mode == 1 && value->auto_mode == 1 )
    {
        SHDISP_TRACE("shdisp_SQE_set_cabc_update_param : SHDISP_MAIN_DISP_CABC_MODE_DBC_ACC \n");
        cabc_mode_set = SHDISP_MAIN_DISP_CABC_MODE_DBC_ACC;
    }

    return ret;
}


/* ------------------------------------------------------------------------- */
/* shdisp_SQE_cabc_ctrl                                                      */
/* ------------------------------------------------------------------------- */

static int shdisp_SQE_cabc_ctrl(void)
{
    int ret;
    struct shdisp_check_cabc_val cabc_value;

    if(shdisp_photo_sensor_api_first == 1)
    {
        return SHDISP_RESULT_SUCCESS;
    }

    ret = shdisp_api_check_cabc(&cabc_value);

    if(cabc_value.change == 0)
    {
        SHDISP_TRACE("shdisp_SQE_set_cabc : no change.\n");
        return SHDISP_RESULT_SUCCESS;
    }

    if(cabc_value.old_mode == SHDISP_MAIN_DISP_CABC_MODE_OFF && cabc_value.mode == SHDISP_MAIN_DISP_CABC_MODE_DBC )
    {
        SHDISP_TRACE("shdisp_SQE_set_cabc : OFF->DBC .\n");
        mipi_sharp_api_cabc_init();

        mipi_sharp_api_cabc_indoor_on();
    }
    else if(cabc_value.old_mode == SHDISP_MAIN_DISP_CABC_MODE_OFF && cabc_value.mode == SHDISP_MAIN_DISP_CABC_MODE_ACC )
    {
        if(cabc_value.lut == SHDISP_MAIN_DISP_CABC_LUT0)
        {
            SHDISP_TRACE("shdisp_SQE_set_cabc : OFF->ACC  LUT0.\n");
        }
        else
        {
            SHDISP_TRACE("shdisp_SQE_set_cabc : OFF->ACC  LUT1-5.\n");
            mipi_sharp_api_cabc_init();


            mipi_sharp_api_cabc_outdoor_on(cabc_value.lut);
        }
    }
    else if(cabc_value.old_mode == SHDISP_MAIN_DISP_CABC_MODE_OFF && cabc_value.mode == SHDISP_MAIN_DISP_CABC_MODE_DBC_ACC )
    {
        if(cabc_value.lut == SHDISP_MAIN_DISP_CABC_LUT0)
        {
            SHDISP_TRACE("shdisp_SQE_set_cabc : OFF->DBC_ACC  LUT0.\n");

            mipi_sharp_api_cabc_init();

            mipi_sharp_api_cabc_indoor_on();
        }
        else
        {
            SHDISP_TRACE("shdisp_SQE_set_cabc : OFF->DBC_ACC  LUT1-5.\n");

            mipi_sharp_api_cabc_init();


            mipi_sharp_api_cabc_outdoor_on(cabc_value.lut);
        }
    }

    else if(cabc_value.mode == SHDISP_MAIN_DISP_CABC_MODE_OFF )
    {
        SHDISP_TRACE("shdisp_SQE_set_cabc : OFF .\n");
        mipi_sharp_api_cabc_off(0,1);
    }

    else if(cabc_value.old_mode == SHDISP_MAIN_DISP_CABC_MODE_DBC && cabc_value.mode == SHDISP_MAIN_DISP_CABC_MODE_ACC )
    {
        if(cabc_value.lut == SHDISP_MAIN_DISP_CABC_LUT0)
        {
            SHDISP_TRACE("shdisp_SQE_set_cabc : DBC->ACC  LUT0.\n");

            mipi_sharp_api_cabc_off(0,1);
        }
        else
        {
            SHDISP_TRACE("shdisp_SQE_set_cabc : DBC->ACC  LUT1-5.\n");

            mipi_sharp_api_cabc_off(1,0);


            mipi_sharp_api_cabc_outdoor_on(cabc_value.lut);
        }
    }
    else if(cabc_value.old_mode == SHDISP_MAIN_DISP_CABC_MODE_DBC && cabc_value.mode == SHDISP_MAIN_DISP_CABC_MODE_DBC_ACC )
    {
        if(cabc_value.lut == SHDISP_MAIN_DISP_CABC_LUT0)
        {
            SHDISP_TRACE("shdisp_SQE_set_cabc : DBC->DBC_ACC  LUT0.\n");
        }
        else
        {
            SHDISP_TRACE("shdisp_SQE_set_cabc : DBC->DBC_ACC  LUT1-5.\n");

            mipi_sharp_api_cabc_off(1,0);


            mipi_sharp_api_cabc_outdoor_on(cabc_value.lut);
        }
    }

    else if(cabc_value.old_mode == SHDISP_MAIN_DISP_CABC_MODE_ACC && cabc_value.mode == SHDISP_MAIN_DISP_CABC_MODE_DBC )
    {
        if(cabc_value.old_lut == SHDISP_MAIN_DISP_CABC_LUT0)
        {
            SHDISP_TRACE("shdisp_SQE_set_cabc : ACC->DBC  old_LUT0.\n");

            mipi_sharp_api_cabc_init();

            mipi_sharp_api_cabc_indoor_on();
        }
        else
        {
            SHDISP_TRACE("shdisp_SQE_set_cabc : ACC->DBC  none old_LUT0 .\n");

            mipi_sharp_api_cabc_off(1,0);

            mipi_sharp_api_cabc_indoor_on();
        }
    }
    else if(cabc_value.old_mode == SHDISP_MAIN_DISP_CABC_MODE_ACC && cabc_value.mode == SHDISP_MAIN_DISP_CABC_MODE_ACC)
    {
        if(cabc_value.lut == SHDISP_MAIN_DISP_CABC_LUT0)
        {
            SHDISP_TRACE("shdisp_SQE_set_cabc : ACC->ACC  LUT1-5 -> LUT0.\n");

            mipi_sharp_api_cabc_off(0,1);
        }
        else
        {
            if(cabc_value.old_lut == SHDISP_MAIN_DISP_CABC_LUT0)
            {
                SHDISP_TRACE("shdisp_SQE_set_cabc : ACC->ACC  LUT0 -> LUT1-5.\n");

                mipi_sharp_api_cabc_init();


                mipi_sharp_api_cabc_outdoor_on(cabc_value.lut);
            }
            else
            {
                SHDISP_TRACE("shdisp_SQE_set_cabc : ACC->ACC  LUT1-5 -> LUT1-5.\n");

                mipi_sharp_api_cabc_outdoor_move(cabc_value.lut);
            }
        }
    }
    else if(cabc_value.old_mode == SHDISP_MAIN_DISP_CABC_MODE_ACC && cabc_value.mode == SHDISP_MAIN_DISP_CABC_MODE_DBC_ACC)
    {
        if(cabc_value.old_lut == SHDISP_MAIN_DISP_CABC_LUT0)
        {
            if(cabc_value.lut == SHDISP_MAIN_DISP_CABC_LUT0)
            {
                SHDISP_TRACE("shdisp_SQE_set_cabc : ACC->DBC_ACC  LUT0 -> LUT0.\n");

                mipi_sharp_api_cabc_init();

                mipi_sharp_api_cabc_indoor_on();
            }
            else
            {
                SHDISP_TRACE("shdisp_SQE_set_cabc : ACC->DBC_ACC  LUT0 -> LUT1-5.\n");

                mipi_sharp_api_cabc_init();


                mipi_sharp_api_cabc_outdoor_on(cabc_value.lut);
            }
        }
        else
        {
            if(cabc_value.lut == SHDISP_MAIN_DISP_CABC_LUT0)
            {
                SHDISP_TRACE("shdisp_SQE_set_cabc : ACC->DBC_ACC  LUT1-5 -> LUT0.\n");

                mipi_sharp_api_cabc_off(1,0);

                mipi_sharp_api_cabc_indoor_on();
            }
            else
            {
                SHDISP_TRACE("shdisp_SQE_set_cabc : ACC->DBC_ACC  LUT1-5 -> LUT1-5.\n");

                mipi_sharp_api_cabc_outdoor_move(cabc_value.lut);
            }
        }
    }
    else if(cabc_value.old_mode == SHDISP_MAIN_DISP_CABC_MODE_DBC_ACC && cabc_value.mode == SHDISP_MAIN_DISP_CABC_MODE_DBC )
    {
        if(cabc_value.old_lut == SHDISP_MAIN_DISP_CABC_LUT0)
        {
            SHDISP_TRACE("shdisp_SQE_set_cabc : DBC_ACC->DBC  LUT1-5 -> LUT0.\n");
        }
        else
        {
            SHDISP_TRACE("shdisp_SQE_set_cabc : DBC_ACC->DBC  LUT1-5 -> LUT1-5.\n");

            mipi_sharp_api_cabc_off(1,0);

            mipi_sharp_api_cabc_indoor_on();
        }
    }
    else if(cabc_value.old_mode == SHDISP_MAIN_DISP_CABC_MODE_DBC_ACC && cabc_value.mode == SHDISP_MAIN_DISP_CABC_MODE_ACC )
    {
        if(cabc_value.lut == SHDISP_MAIN_DISP_CABC_LUT0)
        {
            SHDISP_TRACE("shdisp_SQE_set_cabc : DBC_ACC->ACC  LUT1-5 -> LUT0.\n");

            mipi_sharp_api_cabc_off(0,1);
        }
        else
        {
            if(cabc_value.old_lut == SHDISP_MAIN_DISP_CABC_LUT0)
            {
                SHDISP_TRACE("shdisp_SQE_set_cabc : DBC_ACC->ACC  LUT0 -> LUT1-5.\n");

                mipi_sharp_api_cabc_off(1,0);


                mipi_sharp_api_cabc_outdoor_on(cabc_value.lut);
            }
            else
            {
                SHDISP_TRACE("shdisp_SQE_set_cabc : DBC_ACC->ACC  LUT1-5 -> LUT1-5.\n");

                mipi_sharp_api_cabc_outdoor_move(cabc_value.lut);
            }
        }
    }
    else if(cabc_value.old_mode == SHDISP_MAIN_DISP_CABC_MODE_DBC_ACC && cabc_value.mode == SHDISP_MAIN_DISP_CABC_MODE_DBC_ACC )
    {
        if(cabc_value.old_lut == SHDISP_MAIN_DISP_CABC_LUT0)
        {
            SHDISP_TRACE("shdisp_SQE_set_cabc : DBC_ACC->DBC_ACC  LUT0 -> LUT1-5.\n");

            mipi_sharp_api_cabc_off(1,0);


            mipi_sharp_api_cabc_outdoor_on(cabc_value.lut);
        }
        else
        {
            if(cabc_value.lut == SHDISP_MAIN_DISP_CABC_LUT0)
            {
                SHDISP_TRACE("shdisp_SQE_set_cabc : DBC_ACC->DBC_ACC  LUT1-5 -> LUT0.\n");

                mipi_sharp_api_cabc_off(1,0);

                mipi_sharp_api_cabc_indoor_on();
            }
            else
            {
                SHDISP_TRACE("shdisp_SQE_set_cabc : DBC_ACC->DBC_ACC  LUT1-5 -> LUT1-5.\n");

                mipi_sharp_api_cabc_outdoor_move(cabc_value.lut);
            }
        }
    }

    return ret;
}



/* ------------------------------------------------------------------------- */
/* shdisp_SQE_pwm_enable                                                     */
/* ------------------------------------------------------------------------- */

static int shdisp_SQE_pwm_enable(void)
{
    shdisp_bdic_API_LCD_BKL_pwm_enable();
    
    return SHDISP_RESULT_SUCCESS;
}


/* ------------------------------------------------------------------------- */
/* shdisp_SQE_pwm_disable                                                    */
/* ------------------------------------------------------------------------- */

static int shdisp_SQE_pwm_disable(void)
{
    shdisp_bdic_API_LCD_BKL_pwm_disable();
    
    return SHDISP_RESULT_SUCCESS;
}


/* ------------------------------------------------------------------------- */
/* shdisp_semaphore_start                                                    */
/* ------------------------------------------------------------------------- */

static void shdisp_semaphore_start(void)
{
    down(&shdisp_sem);
    
#ifdef SHDISP_SYS_SW_TIME_API
    shdisp_sys_dbg_hw_check_start();
#endif
}


/* ------------------------------------------------------------------------- */
/* shdisp_semaphore_end                                                      */
/* ------------------------------------------------------------------------- */

static void shdisp_semaphore_end(const char *func)
{
#ifdef SHDISP_SYS_SW_TIME_API
    shdisp_sys_dbg_hw_check_end(func);
#endif
    
    up(&shdisp_sem);
}


/* ------------------------------------------------------------------------- */
/* INTERRUPT                                                                 */
/* ------------------------------------------------------------------------- */
/* ------------------------------------------------------------------------- */
/* shdisp_gpio_int_isr                                                       */
/* ------------------------------------------------------------------------- */

static irqreturn_t shdisp_gpio_int_isr( int irq_num, void *data )
{
    irqreturn_t rc = IRQ_HANDLED;
    int ret;
    unsigned long flags = 0;

    shdisp_SYS_set_irq(SHDISP_IRQ_DISABLE);

    spin_lock_irqsave( &shdisp_q_lock, flags);

    SHDISP_TRACE("shdisp_gpio_int_isr:Start\n");

    if (shdisp_wq_gpio) {
        shdisp_wake_lock();
        ret = queue_work(shdisp_wq_gpio, &shdisp_wq_gpio_wk);
        if( ret == 0 ) {
            shdisp_wake_unlock();
            SHDISP_ERR("<QUEUE_WORK_FAILURE> shdisp_gpio_int_isr.\n");
        }
    }
    spin_unlock_irqrestore( &shdisp_q_lock, flags);
    
    return rc;
}


/* ------------------------------------------------------------------------- */
/* shdisp_workqueue_handler_gpio                                             */
/* ------------------------------------------------------------------------- */

static void shdisp_workqueue_handler_gpio(struct work_struct *work)
{
    struct shdisp_queue_data_t *qdata = NULL;
    int    i;
    int    bFirstQue = 0;
    int    ret;
    int    nBDIC_QueFac = 0;


    SHDISP_TRACE("shdisp_workqueue_handler_gpio : Start\n");
    
#ifdef DBG_INT
    gpio_tlmm_config(GPIO_CFG(65, 0, GPIO_CFG_OUTPUT, GPIO_CFG_NO_PULL, GPIO_CFG_2MA), GPIO_CFG_ENABLE);
    gpio_set_value(65, 1);
#endif

    shdisp_semaphore_start();
    shdisp_bdic_API_IRQ_save_fac();
    shdisp_semaphore_end(__func__);

    do{
        shdisp_semaphore_start();
        ret = shdisp_bdic_API_IRQ_check_fac();
        shdisp_semaphore_end(__func__);
        if( ret != SHDISP_RESULT_SUCCESS ){
            SHDISP_TRACE("shdisp_workqueue_handler_gpio : no factory\n");
            break;
        }
        
        down(&shdisp_sem_irq_fac);
        for(i=0; i<SHDISP_IRQ_MAX_KIND; i++){
            shdisp_semaphore_start();
            nBDIC_QueFac = shdisp_bdic_API_IRQ_get_fac(i);
            shdisp_semaphore_end(__func__);
            if( nBDIC_QueFac == SHDISP_BDIC_IRQ_TYPE_NONE )
                break;
            
            if (shdisp_wq_gpio_task) {
                qdata = kmalloc( sizeof(shdisp_queue_data), GFP_KERNEL );
                if( qdata != NULL ){
                    qdata->irq_GFAC = nBDIC_QueFac;
                    list_add_tail(&qdata->list, &shdisp_queue_data.list);
                    if( bFirstQue == 0 ){
                        bFirstQue = 1;
                        shdisp_wake_lock();
                        ret = queue_work(shdisp_wq_gpio_task, &shdisp_wq_gpio_task_wk);
                        if( ret == 0 ){
                            shdisp_wake_unlock();
                            SHDISP_TRACE("<QUEUE_WORK_FAILURE> shdisp_workqueue_handler_gpio.\n");
                        }
                    }
                }
                else{
                   SHDISP_ERR("<QUEUE_WORK_FAILURE> shdisp_workqueue_handler_gpio:kmalloc failed (BDIC_QueFac=%d)\n", nBDIC_QueFac);
                }
            }
        }
        up(&shdisp_sem_irq_fac);

    }while(0);

    shdisp_semaphore_start();
    shdisp_bdic_API_IRQ_Clear();
    shdisp_semaphore_end(__func__);

    if( shdisp_bdic_API_IRQ_check_DET() != SHDISP_BDIC_IRQ_TYPE_DET ){
        SHDISP_TRACE("shdisp_workqueue_handler_gpio:enable_irq for No DET\n");
        shdisp_SYS_set_irq(SHDISP_IRQ_ENABLE);
    }
    SHDISP_TRACE("shdisp_workqueue_handler_gpio : Finish\n");
#ifdef DBG_INT
    gpio_tlmm_config(GPIO_CFG(65, 0, GPIO_CFG_OUTPUT, GPIO_CFG_NO_PULL, GPIO_CFG_2MA), GPIO_CFG_ENABLE);
    gpio_set_value(65, 0);
#endif
    shdisp_wake_unlock();
    return;
}


/* ------------------------------------------------------------------------- */
/* shdisp_workqueue_gpio_task                                                */
/* ------------------------------------------------------------------------- */

static void shdisp_workqueue_gpio_task(struct work_struct *work)
{
    struct list_head *listptr;
    struct shdisp_queue_data_t  *entry;
    struct shdisp_queue_data_t  *entryFirst = NULL;
    int     nFirstBDIC_GFAC=0;
    int     nFirst_GFAC=-1;
    int     bFirst=0;
    int     bThrough=0;
    void (*temp_callback)(void);

    SHDISP_TRACE("shdisp_workqueue_gpio_task : Start\n");

    do{
        down(&shdisp_sem_irq_fac);
        bThrough = 0;
        entryFirst = NULL;
        bFirst = 0;
        nFirstBDIC_GFAC = 0;
        list_for_each(listptr, &shdisp_queue_data.list) {
            entry = list_entry( listptr, struct shdisp_queue_data_t, list);
            if( bFirst == 0 ){
                entryFirst = entry;
                nFirstBDIC_GFAC = entry->irq_GFAC;
                bFirst = 1;
            }
            else{
                if( entry->irq_GFAC == nFirstBDIC_GFAC ){
                    bThrough =1;
                }
            }
        }
        
        if( entryFirst != NULL ){
            list_del( &entryFirst->list );
            kfree( entryFirst );
        }
        else{
            SHDISP_TRACE("shdisp_workqueue_gpio_task : no entry\n");
            up(&shdisp_sem_irq_fac);
            break;
        }
        up(&shdisp_sem_irq_fac);
        
        
        if( bThrough == 0 ){
            if( nFirstBDIC_GFAC == SHDISP_BDIC_IRQ_TYPE_NONE ){
                SHDISP_TRACE("shdisp_workqueue_gpio_task failed (no BDIC_GFAC=%d)\n", nFirstBDIC_GFAC);
            }
            else{
                nFirst_GFAC = -1;
                switch ( nFirstBDIC_GFAC ) {
                case SHDISP_BDIC_IRQ_TYPE_ALS:
                        nFirst_GFAC = SHDISP_IRQ_TYPE_ALS;
                        break;
                case SHDISP_BDIC_IRQ_TYPE_PS:
                        nFirst_GFAC = SHDISP_IRQ_TYPE_PS;
                        break;
                case SHDISP_BDIC_IRQ_TYPE_DET:
                        nFirst_GFAC = SHDISP_IRQ_TYPE_DET;
                        break;
                default:
                        break;
                }
                
                SHDISP_TRACE("shdisp_workqueue_gpio_task : Callback[%d] Start\n", nFirstBDIC_GFAC);
                if( nFirst_GFAC >= 0 ){
                    down(&shdisp_sem_callback);
                    temp_callback = shdisp_callback_table[nFirst_GFAC];
                    up(&shdisp_sem_callback);

                    if( temp_callback != NULL ){
                        if( nFirst_GFAC == SHDISP_IRQ_TYPE_DET ) {
                            shdisp_semaphore_start();
                            shdisp_bdic_API_IRQ_backup_irq_reg();
                            shdisp_bdic_API_IRQ_det_irq_mask();
                            shdisp_semaphore_end(__func__);
                        }
                            
                        (*temp_callback)();

                        if( nFirst_GFAC == SHDISP_IRQ_TYPE_DET ) {
                            shdisp_semaphore_start();
                            shdisp_bdic_API_IRQ_det_fac_Clear();
                            if(!mipi_sharp_get_suspended_recovery_info()){
                                shdisp_bdic_API_IRQ_return_irq_reg();
                            }else{
                                shdisp_bdic_API_IRQ_return_irq_reg_no_det();
                            }
                            shdisp_semaphore_end(__func__);
                        }
                    }
                    else{
                        SHDISP_TRACE("shdisp_workqueue_gpio_task Callback is Nulle pointer(irq_type=%d)\n", nFirst_GFAC);
                    }
                    
                    if( nFirst_GFAC == SHDISP_IRQ_TYPE_DET ){
                        SHDISP_TRACE("shdisp_workqueue_gpio_task:enable_irq for DET\n");
                        shdisp_SYS_set_irq(SHDISP_IRQ_ENABLE);
                    }
                }
                else if( nFirstBDIC_GFAC == SHDISP_BDIC_IRQ_TYPE_I2C_ERR ){
                    shdisp_SQE_bdic_i2c_error_handling();
                }
            }
        }
        else{
            SHDISP_TRACE("shdisp_workqueue_gpio_task : Skip (BDIC_GFAC=%d)\n", nFirstBDIC_GFAC);
        }
    }while(1);
    

    SHDISP_TRACE("shdisp_workqueue_gpio_task : Finish\n");
    shdisp_wake_unlock();
    
    return;
}


static void shdisp_wake_lock_init(void)
{
    unsigned long flags = 0;

    spin_lock_irqsave( &shdisp_wake_spinlock, flags);
    shdisp_wake_lock_wq_refcnt = 0;
    wake_lock_init(&shdisp_wake_lock_wq, WAKE_LOCK_SUSPEND, "shdisp_wake_lock_wq");
    spin_unlock_irqrestore( &shdisp_wake_spinlock, flags);
}


static void shdisp_wake_lock(void)
{
    unsigned long flags = 0;

    spin_lock_irqsave( &shdisp_wake_spinlock, flags);
    if (shdisp_wake_lock_wq_refcnt++ == 0) {
        wake_lock(&shdisp_wake_lock_wq);
    }
    spin_unlock_irqrestore( &shdisp_wake_spinlock, flags);
}
            
static void shdisp_wake_unlock(void)
{
    unsigned long flags = 0;

    spin_lock_irqsave( &shdisp_wake_spinlock, flags);
    if (--shdisp_wake_lock_wq_refcnt <= 0) {
        wake_unlock(&shdisp_wake_lock_wq);
        shdisp_wake_lock_wq_refcnt = 0;
    }
    spin_unlock_irqrestore( &shdisp_wake_spinlock, flags);
}

/* ------------------------------------------------------------------------- */
/* shdisp_timer_int_isr                                                      */
/* ------------------------------------------------------------------------- */

static void shdisp_timer_int_isr(unsigned long data)
{
    int ret;
    
    SHDISP_TRACE("Timeout ( registered %ld, now %ld ).\n", data, jiffies);
    SHDISP_TRACE("shdisp_timer_int_isr:Start\n");
    
    if (shdisp_timer_stop) {
        SHDISP_TRACE("Timer is not to be restarted.\n");
        return;
    }
    if (shdisp_wq_timer_task) {
        ret = queue_work(shdisp_wq_timer_task, &shdisp_wq_timer_task_wk);
        if( ret == 0 )
            SHDISP_ERR("<QUEUE_WORK_FAILURE> shdisp_timer_int_isr.\n");
    }
    
    return;
}


/* ------------------------------------------------------------------------- */
/* shdisp_timer_int_register                                                 */
/* ------------------------------------------------------------------------- */

static void shdisp_timer_int_register(void)
{
    down(&shdisp_sem_timer);
    
    shdisp_timer.expires  = jiffies + (10 * HZ);
    shdisp_timer.data     = (unsigned long)jiffies;
    shdisp_timer.function = shdisp_timer_int_isr;
    
    if (!shdisp_timer_stop) {
        up(&shdisp_sem_timer);
        return;
    }
    
    add_timer(&shdisp_timer);
    shdisp_timer_stop = 0;
    
    up(&shdisp_sem_timer);
    
    return;
}


/* ------------------------------------------------------------------------- */
/* shdisp_timer_int_delete                                                   */
/* ------------------------------------------------------------------------- */

static void shdisp_timer_int_delete(void)
{
    down(&shdisp_sem_timer);
    
    del_timer_sync(&shdisp_timer);
    shdisp_timer_stop = 1;
    
    up(&shdisp_sem_timer);
    
    return;
}


/* ------------------------------------------------------------------------- */
/* shdisp_timer_int_mod                                                      */
/* ------------------------------------------------------------------------- */

static void shdisp_timer_int_mod(void)
{
    down(&shdisp_sem_timer);
    
    if (shdisp_timer_stop) {
        up(&shdisp_sem_timer);
        return;
    }
    
    mod_timer(&shdisp_timer, (unsigned long)(jiffies + (10 * HZ)));
    shdisp_timer_stop = 0;
    
    up(&shdisp_sem_timer);
    
    return;
}


/* ------------------------------------------------------------------------- */
/* shdisp_workqueue_timer_task                                               */
/* ------------------------------------------------------------------------- */

static void shdisp_workqueue_timer_task(struct work_struct *work)
{
    int     ret=0;
    int     nFirst_GFAC=-1;
    void    (*temp_callback)(void);
    
    nFirst_GFAC = SHDISP_IRQ_TYPE_DET;
    
    if (shdisp_kerl_ctx.dbgTraceF != 0xFF) {
        shdisp_semaphore_start();
        ret = shdisp_panel_API_check_recovery();
        shdisp_semaphore_end(__func__);
        if (ret == SHDISP_RESULT_SUCCESS) {
            shdisp_timer_int_mod();
            return;
        }
        SHDISP_TRACE("A recovery processing is required.\n");
    }
    
    down(&shdisp_sem_callback);
    temp_callback = shdisp_callback_table[nFirst_GFAC];
    up(&shdisp_sem_callback);
    
    if ( temp_callback != NULL ) {
        if ( nFirst_GFAC == SHDISP_IRQ_TYPE_DET ) {
            shdisp_semaphore_start();
            shdisp_bdic_API_IRQ_backup_irq_reg();
            shdisp_semaphore_end(__func__);
        }
            
        (*temp_callback)();

        if ( nFirst_GFAC == SHDISP_IRQ_TYPE_DET ) {
            shdisp_semaphore_start();
            shdisp_bdic_API_IRQ_return_irq_reg();
            shdisp_semaphore_end(__func__);
        }
    }
    else {
        SHDISP_TRACE("shdisp_workqueue_timer_task Callback is Nulle pointer(irq_type=%d)\n", nFirst_GFAC);
    }
    
    shdisp_timer_int_mod();
    
    return;
}


/* ------------------------------------------------------------------------- */
/* shdisp_photo_lux_irq                                                      */
/* ------------------------------------------------------------------------- */

static void shdisp_photo_lux_irq(void)
{
    long    wait_jiffies;
    int     ret;

    SHDISP_TRACE("shdisp_photo_lux_irq : Start\n");

    shdisp_semaphore_start();

    shdisp_bdic_API_PHOTO_lux_timer( SHDISP_BDIC_PHOTO_LUX_TIMER_OFF );
    ret = shdisp_bdic_API_PHOTO_SENSOR_lux_judge();

    shdisp_bdic_API_PHOTO_lux_timer( SHDISP_BDIC_PHOTO_LUX_TIMER_ON );
    shdisp_semaphore_end(__func__);

    if(( ret == SHDISP_BDIC_LUX_JUDGE_OUT ) ||
       ( ret == SHDISP_BDIC_LUX_JUDGE_OUT_CONTI )){
        shdisp_new_opt_mode = SHDISP_LUX_MODE_HIGH;
    }
    else
        shdisp_new_opt_mode = SHDISP_LUX_MODE_LOW;

    if( ret == SHDISP_BDIC_LUX_JUDGE_OUT ){
        if (shdisp_wq_sensor_start) {
            SHDISP_TRACE("shdisp_photo_lux_irq : queue_delayed_work(time=%d)\n",shdisp_wait_sensor_start_time);
            wait_jiffies = msecs_to_jiffies(shdisp_wait_sensor_start_time);
            queue_delayed_work(shdisp_wq_sensor_start, &shdisp_sensor_start_wk, wait_jiffies);
        }
    }
    else{
        shdisp_semaphore_start();
        shdisp_SQE_sensor_start_task();
        shdisp_semaphore_end(__func__);

#if defined(CONFIG_SHDISP_USE_CABC)
        shdisp_SQE_cabc_ctrl();
#endif
        
        if((shdisp_als_irq_req_state & SHDISP_ALS_IRQ_REQ_DBC) == 0 )
            return;
            
        if(lux_change_wait_flg != SHDISP_LUX_CHANGE_STATE_WAIT){
            return;
        }

        SHDISP_TRACE("shdisp_photo_lux_irq : complete notify\n");

        lux_change_wait_flg = SHDISP_LUX_CHANGE_STATE_WAKEUP;
        complete(&lux_change_notify);
    }
    return;
}


/* ------------------------------------------------------------------------- */
/* shdisp_sensor_start_handler                                               */
/* ------------------------------------------------------------------------- */

static void shdisp_sensor_start_handler(struct work_struct *work)
{
    SHDISP_TRACE("shdisp_sensor_start_handler Start\n");

    shdisp_semaphore_start();
    shdisp_SQE_sensor_start_task();
    shdisp_semaphore_end(__func__);

    shdisp_photo_lux_irq();

    SHDISP_TRACE("shdisp_sensor_start_handler Finish\n");
}


/* ------------------------------------------------------------------------- */
/* shdisp_als_irq_subscribe_bkl_ctrl                                         */
/* ------------------------------------------------------------------------- */

static int shdisp_als_irq_subscribe_bkl_ctrl(int mode)
{
    struct shdisp_subscribe strctsubs;

    shdisp_als_irq_subscribe_type = SHDISP_ALS_IRQ_REQ_BK_CTL;
    
    if( mode == SHDISP_BKL_MODE_OFF ){
#if defined(CONFIG_SHDISP_USE_CABC)
        if( shdisp_als_irq_req_state != 0 ){
            shdisp_als_irq_req_state = 0;
            shdisp_SQE_event_unsubscribe(SHDISP_IRQ_TYPE_ALS);
        }
#else
        shdisp_als_irq_req_state &= ~shdisp_als_irq_subscribe_type;
        if( shdisp_als_irq_req_state != 0 ){
            return SHDISP_RESULT_SUCCESS;
        }
        if (delayed_work_pending(&shdisp_sensor_start_wk)) {
            cancel_delayed_work(&shdisp_sensor_start_wk);
            flush_workqueue(shdisp_wq_sensor_start);
            SHDISP_TRACE("shdisp_api_event_unsubscribe:cancel_delayed_work\n");
        }
        shdisp_SQE_event_unsubscribe(SHDISP_IRQ_TYPE_ALS);
#endif
    }
    else if( mode == SHDISP_BKL_MODE_ON ){
        if (shdisp_kerl_ctx.main_bkl.mode == SHDISP_MAIN_BKL_MODE_AUTO) {
            if( shdisp_als_irq_req_state != 0 ){
                shdisp_als_irq_req_state |= shdisp_als_irq_subscribe_type;
                return SHDISP_RESULT_SUCCESS;
            }
            shdisp_als_irq_req_state |= shdisp_als_irq_subscribe_type;
            strctsubs.irq_type = SHDISP_IRQ_TYPE_ALS;
            strctsubs.callback = shdisp_photo_lux_irq;
            shdisp_SQE_event_subscribe(&strctsubs);
        }
    }
    else if( mode == SHDISP_BKL_MODE_AUTO ){
        if (shdisp_kerl_ctx.main_bkl.mode != SHDISP_MAIN_BKL_MODE_AUTO) {
            shdisp_als_irq_req_state &= ~shdisp_als_irq_subscribe_type;
            if( shdisp_als_irq_req_state != 0 ){
                return SHDISP_RESULT_SUCCESS;
            }
            if (delayed_work_pending(&shdisp_sensor_start_wk)) {
                cancel_delayed_work(&shdisp_sensor_start_wk);
                flush_workqueue(shdisp_wq_sensor_start);
                SHDISP_TRACE("shdisp_api_event_unsubscribe:cancel_delayed_work\n");
            }
            shdisp_SQE_event_unsubscribe(SHDISP_IRQ_TYPE_ALS);
        }
        else{
            if( shdisp_als_irq_req_state != 0 ){
                shdisp_als_irq_req_state |= shdisp_als_irq_subscribe_type;
                return SHDISP_RESULT_SUCCESS;
            }
            shdisp_als_irq_req_state |= shdisp_als_irq_subscribe_type;
            strctsubs.irq_type = SHDISP_IRQ_TYPE_ALS;
            strctsubs.callback = shdisp_photo_lux_irq;
            shdisp_SQE_event_subscribe(&strctsubs);
        }
    }
    
    return SHDISP_RESULT_SUCCESS;
}


/* ------------------------------------------------------------------------- */
/* OTHER                                                                     */
/* ------------------------------------------------------------------------- */
/* ------------------------------------------------------------------------- */
/* shdisp_proc_write                                                         */
/* ------------------------------------------------------------------------- */

#if defined (CONFIG_ANDROID_ENGINEERING)
static int shdisp_proc_write(struct file *file, const char *buffer, unsigned long count, void *data)
{
#define SHDISP_LEN_ID    (2)
#define SHDISP_LEN_PARAM (4)
#define SHDISP_PARAM_MAX (4)

    unsigned long len = count;
    struct shdisp_procfs shdisp_pfs;
    char buf[SHDISP_LEN_PARAM + 1];
    char kbuf[SHDISP_LEN_ID + SHDISP_PARAM_MAX * SHDISP_LEN_PARAM];
    int i;
    int ret = 0;
    struct shdisp_bdic_i2c_msg i2c_msg;
    unsigned char i2c_wbuf[6];
    unsigned char i2c_rbuf[6];
    struct shdisp_prox_params prox_params;
    struct shdisp_main_bkl_ctl bkl;
    unsigned char   val;
    
    len--;
    /* Check length */
    if (len < SHDISP_LEN_ID){
        return count;
    }
    if (len > (SHDISP_LEN_ID + SHDISP_PARAM_MAX * SHDISP_LEN_PARAM)){
       len = SHDISP_LEN_ID + SHDISP_PARAM_MAX * SHDISP_LEN_PARAM;
    }

    if (copy_from_user(kbuf, buffer, len)){
        return -EFAULT;
    }
    /* Get FunctionID */
    memcpy(buf, kbuf, SHDISP_LEN_ID);
    buf[SHDISP_LEN_ID] = '\0';
    shdisp_pfs.id = simple_strtol(buf, NULL, 10);
    shdisp_pfs.par[0] = 0;
    shdisp_pfs.par[1] = 0;
    shdisp_pfs.par[2] = 0;
    shdisp_pfs.par[3] = 0;
    
    /* Get Parameters */
    for(i = 0; (i + 1) * SHDISP_LEN_PARAM <= (len - SHDISP_LEN_ID); i++){
        memcpy(buf, &(kbuf[SHDISP_LEN_ID + i * SHDISP_LEN_PARAM]), SHDISP_LEN_PARAM);
        buf[SHDISP_LEN_PARAM] = '\0';
        shdisp_pfs.par[i] = simple_strtol(buf, NULL, 16);
    }
    
    printk("[SHDISP] shdisp_proc_write(%d, 0x%04x, 0x%04x, 0x%04x, 0x%04x)\n", shdisp_pfs.id, shdisp_pfs.par[0], shdisp_pfs.par[1], shdisp_pfs.par[2], shdisp_pfs.par[3]);
    
    switch (shdisp_pfs.id){
    case SHDISP_DEBUG_PROCESS_STATE_OUTPUT:
        shdisp_dbg_info_output((int)shdisp_pfs.par[0]);
        break;
    
    case SHDISP_DEBUG_TRACE_LOG_SWITCH:
        if (shdisp_pfs.par[0] == 0) {
            printk("[SHDISP] Trace log OFF\n");
            shdisp_kerl_ctx.dbgTraceF = (unsigned char)shdisp_pfs.par[0];
        }
        else {
            printk("[SHDISP] Trace log ON(%d)\n", shdisp_pfs.par[0]);
            shdisp_kerl_ctx.dbgTraceF = (unsigned char)shdisp_pfs.par[0];
        }
        SHDISP_TRACE("TraceLog enable check!!\n");
        break;
    
    case SHDISP_DEBUG_BDIC_I2C_WRITE:
        printk("[SHDISP] BDIC-I2C WRITE (addr : %d, data : %d)\n", shdisp_pfs.par[0], shdisp_pfs.par[1]);
        i2c_wbuf[0] = shdisp_pfs.par[0];
        i2c_wbuf[1] = shdisp_pfs.par[1];
        
        i2c_msg.addr = 0x39;
        i2c_msg.mode = SHDISP_BDIC_I2C_M_W;
        i2c_msg.wlen = 2;
        i2c_msg.rlen = 0;
        i2c_msg.wbuf = &i2c_wbuf[0];
        i2c_msg.rbuf = NULL;
        ret = shdisp_api_write_bdic_i2c(&i2c_msg);
        break;
        
    case SHDISP_DEBUG_BDIC_I2C_READ:
        printk("[SHDISP] BDIC-I2C READ (addr : %d)\n", shdisp_pfs.par[0]);
        i2c_wbuf[0] = shdisp_pfs.par[0];
        i2c_rbuf[0] = 0x00;
        
        i2c_msg.addr = 0x39;
        i2c_msg.mode = SHDISP_BDIC_I2C_M_R;
        i2c_msg.wlen = 1;
        i2c_msg.rlen = 1;
        i2c_msg.wbuf = &i2c_wbuf[0];
        i2c_msg.rbuf = &i2c_rbuf[0];
        ret = shdisp_api_read_bdic_i2c(&i2c_msg);
        printk("[SHDISP] Read data(0x%02x)\n", i2c_rbuf[0]);
        break;
        
    case SHDISP_DEBUG_PROX_SENSOR_CTL:
        printk("[SHDISP] PROX_SENSOR_CTL (power_mode : %d)\n", shdisp_pfs.par[0]);
        if (shdisp_pfs.par[0] == 0) {
            ret = shdisp_api_prox_sensor_pow_ctl(SHDISP_PROX_SENSOR_POWER_OFF, NULL);
        }
        else {
            printk("[SHDISP] POWER_ON_PARAM (LOW : %d, HHIGH : %d)\n", shdisp_pfs.par[1], shdisp_pfs.par[2]);
            prox_params.threshold_low  = (unsigned int)shdisp_pfs.par[1];
            prox_params.threshold_high = (unsigned int)shdisp_pfs.par[2];
            ret = shdisp_api_prox_sensor_pow_ctl(SHDISP_PROX_SENSOR_POWER_ON, &prox_params);
        }
        break;
        
    case SHDISP_DEBUG_BKL_CTL:
        if (shdisp_pfs.par[0] == 0) {
            printk("[SHDISP] BKL_OFF\n");
            ret = shdisp_api_main_bkl_off();
        }
        else {
            printk("[SHDISP] BKL_ON (mode : %d, param : %d)\n", shdisp_pfs.par[1], shdisp_pfs.par[2]);
            bkl.mode  = shdisp_pfs.par[1];
            bkl.param = shdisp_pfs.par[2];
            ret = shdisp_api_main_bkl_on(&bkl);
        }
        break;
        
    case 6:
        i = shdisp_pfs.par[0];
        printk("[SHDISP] shdisp_proc_write(%d):Test Pattern=%d\n", shdisp_pfs.id, i);
        shdisp_dbg_que(i);
        break;

    case 7:
        printk("[SHDISP] shdisp_proc_write(%d):Interrupt Clear All\n", shdisp_pfs.id);
        shdisp_bdic_API_IRQ_dbg_Clear_All();
        break;

    case 8:
        printk("[SHDISP] shdisp_proc_write(%d):Interrupt Clear\n", shdisp_pfs.id);
        shdisp_bdic_API_IRQ_Clear();
        break;

    case 9:
        printk("[SHDISP] shdisp_proc_write(%d):dummy subscribe\n", shdisp_pfs.id);
        shdisp_debug_subscribe();
        break;

    case 10:
        printk("[SHDISP] shdisp_proc_write(%d):dummy unsubscribe\n", shdisp_pfs.id);
        shdisp_api_event_unsubscribe(SHDISP_IRQ_TYPE_PS);
        break;

    case 11:
        printk("[SHDISP] shdisp_proc_write(%d):dummy unsubscribe\n", shdisp_pfs.id);
        shdisp_api_event_unsubscribe(SHDISP_IRQ_TYPE_ALS);
        break;

    case 12:
        printk("[SHDISP] shdisp_proc_write(%d): LUX REGISTER CHANGE\n", shdisp_pfs.id);
        shdisp_bdic_API_IRQ_dbg_photo_param(shdisp_pfs.par[0], shdisp_pfs.par[1]);
        break;

    case 13:
        printk("[SHDISP] shdisp_proc_write(%d): XEN_SENSOR clear wait time=%d[msec]\n", shdisp_pfs.id, shdisp_pfs.par[0]);
        shdisp_wait_sensor_start_time = shdisp_pfs.par[0];
        break;

    case 14:
        printk("[SHDISP] shdisp_proc_write(%d): XEN_SENSOR bit clear\n", shdisp_pfs.id);
        shdisp_semaphore_start();
        shdisp_SQE_bdic_read_reg(BDIC_REG_SENSOR, &val);
        val &= 0xFE;
        shdisp_SQE_bdic_write_reg(BDIC_REG_SENSOR, val);
        shdisp_semaphore_end(__func__);
        break;

    case 15:
        printk("[SHDISP] FPS LOW=%d BASE FPS LOW=%d\n", fps_low_mode,base_fps_low_mode);
        break;

    default:
        break;
    }
    
    printk("[SHDISP] result : %d.\n", ret);
    
    return count;
}
#endif /* CONFIG_ANDROID_ENGINEERING */


#if defined (CONFIG_ANDROID_ENGINEERING)
/* ------------------------------------------------------------------------- */
/* shdisp_dbg_info_output                                                    */
/* ------------------------------------------------------------------------- */

static void shdisp_dbg_info_output(int mode)
{
    int i;

    switch (mode) {
    case SHDISP_DEBUG_INFO_TYPE_BOOT:
        printk("[SHDISP] BOOT INFO ->>\n");
        printk("[SHDISP] boot_ctx.driver_is_initialized         = %d.\n", shdisp_boot_ctx.driver_is_initialized);
        printk("[SHDISP] boot_ctx.hw_revision                   = %d.\n", (int)shdisp_boot_ctx.hw_revision);
        printk("[SHDISP] boot_ctx.handset_color                 = %d.\n", shdisp_boot_ctx.handset_color);
        printk("[SHDISP] boot_ctx.upper_unit_is_connected       = %d.\n", shdisp_boot_ctx.upper_unit_is_connected);
        printk("[SHDISP] boot_ctx.bdic_is_exist                 = %d.\n", shdisp_boot_ctx.bdic_is_exist);
        printk("[SHDISP] boot_ctx.main_disp_status              = %d.\n", shdisp_boot_ctx.main_disp_status);
        printk("[SHDISP] boot_ctx.main_bkl.mode                 = %d.\n", shdisp_boot_ctx.main_bkl.mode);
        printk("[SHDISP] boot_ctx.main_bkl.param                = %d.\n", shdisp_boot_ctx.main_bkl.param);
        printk("[SHDISP] boot_ctx.tri_led.red                   = %d.\n", (int)shdisp_boot_ctx.tri_led.red);
        printk("[SHDISP] boot_ctx.tri_led.green                 = %d.\n", (int)shdisp_boot_ctx.tri_led.green);
        printk("[SHDISP] boot_ctx.tri_led.blue                  = %d.\n", (int)shdisp_boot_ctx.tri_led.blue);
        printk("[SHDISP] boot_ctx.tri_led.ext_mode              = %d.\n", shdisp_boot_ctx.tri_led.ext_mode);
        printk("[SHDISP] boot_ctx.tri_led.led_mode              = %d.\n", shdisp_boot_ctx.tri_led.led_mode);
        printk("[SHDISP] boot_ctx.tri_led.ontime                = %d.\n", shdisp_boot_ctx.tri_led.ontime);
        printk("[SHDISP] boot_ctx.tri_led.interval              = %d.\n", shdisp_boot_ctx.tri_led.interval);
        printk("[SHDISP] boot_ctx.alpha                         = 0x%04X.\n", (unsigned int)shdisp_boot_ctx.alpha);
        printk("[SHDISP] boot_ctx.alpha_low                     = 0x%04X.\n", (unsigned int)shdisp_boot_ctx.alpha_low);
        printk("[SHDISP] boot_ctx.dma_lut_status                = 0x%04X.\n", (unsigned short)shdisp_boot_ctx.dma_lut_status);
        printk("[SHDISP] boot_ctx.photo_sensor_adj.status       = 0x%02X.\n", (unsigned int)shdisp_boot_ctx.photo_sensor_adj.status);
        printk("[SHDISP] boot_ctx.photo_sensor_adj.als_adj0     = 0x%04X.\n", (unsigned int)shdisp_boot_ctx.photo_sensor_adj.als_adj0);
        printk("[SHDISP] boot_ctx.photo_sensor_adj.als_adj1     = 0x%04X.\n", (unsigned int)shdisp_boot_ctx.photo_sensor_adj.als_adj1);
        printk("[SHDISP] boot_ctx.photo_sensor_adj.als_shift    = 0x%02X.\n", (unsigned int)shdisp_boot_ctx.photo_sensor_adj.als_shift);
        printk("[SHDISP] boot_ctx.photo_sensor_adj.clear_offset = 0x%02X.\n", (unsigned int)shdisp_boot_ctx.photo_sensor_adj.clear_offset);
        printk("[SHDISP] boot_ctx.photo_sensor_adj.ir_offset    = 0x%02X.\n", (unsigned int)shdisp_boot_ctx.photo_sensor_adj.ir_offset);
        printk("[SHDISP] boot_ctx.photo_sensor_adj.adc_lclip    = 0x%02X.\n", (unsigned int)shdisp_boot_ctx.photo_sensor_adj.adc_lclip);
        printk("[SHDISP] boot_ctx.photo_sensor_adj.key_backlight    = 0x%02X.\n", (unsigned int)shdisp_boot_ctx.photo_sensor_adj.key_backlight);
        printk("[SHDISP] boot_ctx.photo_sensor_adj.chksum       = 0x%06X.\n", (unsigned int)shdisp_boot_ctx.photo_sensor_adj.chksum);
        printk("[SHDISP] boot_ctx.ledc_status.ledc_is_exist     = %d.\n", shdisp_boot_ctx.ledc_status.ledc_is_exist);
        printk("[SHDISP] boot_ctx.ledc_status.power_status      = %d.\n", shdisp_boot_ctx.ledc_status.power_status);
        printk("[SHDISP] boot_ctx.ledc_status.ledc_req.red      = %d.\n", (int)shdisp_boot_ctx.ledc_status.ledc_req.red);
        printk("[SHDISP] boot_ctx.ledc_status.ledc_req.green    = %d.\n", (int)shdisp_boot_ctx.ledc_status.ledc_req.green);
        printk("[SHDISP] boot_ctx.ledc_status.ledc_req.blue     = %d.\n", (int)shdisp_boot_ctx.ledc_status.ledc_req.blue);
        printk("[SHDISP] boot_ctx.ledc_status.ledc_req.led_mode = %d.\n", shdisp_boot_ctx.ledc_status.ledc_req.led_mode);
        printk("[SHDISP] boot_ctx.ledc_status.ledc_req.on_count = %d.\n", shdisp_boot_ctx.ledc_status.ledc_req.on_count);
        printk("[SHDISP] boot_ctx.shdisp_lcd                    = 0x%04X.\n", (unsigned int)shdisp_boot_ctx.shdisp_lcd);
        printk("[SHDISP] boot_ctx.lcddr_phy_gamma.status        = 0x%02X.\n", shdisp_boot_ctx.lcddr_phy_gamma.status);
        printk("[SHDISP] boot_ctx.lcddr_phy_gamma.buf           = ");
        for(i = 0; i < SHDISP_LCDDR_PHY_GAMMA_BUF_MAX; i++) {
            printk("%02X,", shdisp_boot_ctx.lcddr_phy_gamma.buf[i]);
        }
        printk("\n");
        printk("[SHDISP] boot_ctx.lcddr_phy_gamma.chksum        = 0x%04X.\n", shdisp_boot_ctx.lcddr_phy_gamma.chksum);
        printk("[SHDISP] boot_ctx.lcddr_rom_gamma.buf           = ");
        for(i = 0; i < SHDISP_LCDDR_PHY_GAMMA_BUF_MAX; i++) {
            printk("%02X,", shdisp_boot_ctx.lcddr_rom_gamma.buf[i]);
        }
        printk("\n");
        printk("[SHDISP] BOOT INFO <<-\n");
        break;
    case SHDISP_DEBUG_INFO_TYPE_KERNEL:
        printk("[SHDISP] KERNEL INFO ->>\n");
        printk("[SHDISP] kerl_ctx.driver_is_initialized         = %d.\n", shdisp_kerl_ctx.driver_is_initialized);
        printk("[SHDISP] kerl_ctx.hw_revision                   = %d.\n", (int)shdisp_kerl_ctx.hw_revision);
        printk("[SHDISP] kerl_ctx.handset_color                 = %d.\n", shdisp_kerl_ctx.handset_color);
        printk("[SHDISP] kerl_ctx.upper_unit_is_connected       = %d.\n", shdisp_kerl_ctx.upper_unit_is_connected);
        printk("[SHDISP] kerl_ctx.bdic_is_exist                 = %d.\n", shdisp_kerl_ctx.bdic_is_exist);
        printk("[SHDISP] kerl_ctx.main_disp_status              = %d.\n", shdisp_kerl_ctx.main_disp_status);
        printk("[SHDISP] kerl_ctx.main_bkl.mode                 = %d.\n", shdisp_kerl_ctx.main_bkl.mode);
        printk("[SHDISP] kerl_ctx.main_bkl.param                = %d.\n", shdisp_kerl_ctx.main_bkl.param);
        printk("[SHDISP] kerl_ctx.tri_led.red                   = %d.\n", (int)shdisp_kerl_ctx.tri_led.red);
        printk("[SHDISP] kerl_ctx.tri_led.green                 = %d.\n", (int)shdisp_kerl_ctx.tri_led.green);
        printk("[SHDISP] kerl_ctx.tri_led.blue                  = %d.\n", (int)shdisp_kerl_ctx.tri_led.blue);
        printk("[SHDISP] kerl_ctx.tri_led.ext_mode              = %d.\n", shdisp_kerl_ctx.tri_led.ext_mode);
        printk("[SHDISP] kerl_ctx.tri_led.led_mode              = %d.\n", shdisp_kerl_ctx.tri_led.led_mode);
        printk("[SHDISP] kerl_ctx.tri_led.ontime                = %d.\n", shdisp_kerl_ctx.tri_led.ontime);
        printk("[SHDISP] kerl_ctx.tri_led.interval              = %d.\n", shdisp_kerl_ctx.tri_led.interval);
        printk("[SHDISP] kerl_ctx.alpha                         = 0x%04X.\n", (unsigned int)shdisp_kerl_ctx.alpha);
        printk("[SHDISP] kerl_ctx.alpha_low                     = 0x%04X.\n", (unsigned int)shdisp_kerl_ctx.alpha_low);
        printk("[SHDISP] kerl_ctx.dma_lut_status                = 0x%04X.\n", (unsigned short)shdisp_kerl_ctx.dma_lut_status);
        printk("[SHDISP] kerl_ctx.photo_sensor_adj.status       = 0x%02X.\n", (unsigned int)shdisp_kerl_ctx.photo_sensor_adj.status);
        printk("[SHDISP] kerl_ctx.photo_sensor_adj.als_adj0     = 0x%04X.\n", (unsigned int)shdisp_kerl_ctx.photo_sensor_adj.als_adj0);
        printk("[SHDISP] kerl_ctx.photo_sensor_adj.als_adj1     = 0x%04X.\n", (unsigned int)shdisp_kerl_ctx.photo_sensor_adj.als_adj1);
        printk("[SHDISP] kerl_ctx.photo_sensor_adj.als_shift    = 0x%02X.\n", (unsigned int)shdisp_kerl_ctx.photo_sensor_adj.als_shift);
        printk("[SHDISP] kerl_ctx.photo_sensor_adj.clear_offset = 0x%02X.\n", (unsigned int)shdisp_kerl_ctx.photo_sensor_adj.clear_offset);
        printk("[SHDISP] kerl_ctx.photo_sensor_adj.ir_offset    = 0x%02X.\n", (unsigned int)shdisp_kerl_ctx.photo_sensor_adj.ir_offset);
        printk("[SHDISP] kerl_ctx.photo_sensor_adj.adc_lclip    = 0x%02X.\n", (unsigned int)shdisp_kerl_ctx.photo_sensor_adj.adc_lclip);
        printk("[SHDISP] kerl_ctx.photo_sensor_adj.key_backlight    = 0x%02X.\n", (unsigned int)shdisp_kerl_ctx.photo_sensor_adj.key_backlight);
        printk("[SHDISP] kerl_ctx.photo_sensor_adj.chksum       = 0x%06X.\n", (unsigned int)shdisp_kerl_ctx.photo_sensor_adj.chksum);
        printk("[SHDISP] kerl_ctx.dtv_status                    = %d.\n", shdisp_kerl_ctx.dtv_status);
        printk("[SHDISP] kerl_ctx.thermal_status                = %d.\n", shdisp_kerl_ctx.thermal_status);
        printk("[SHDISP] kerl_ctx.eco_bkl_status                = %d.\n", shdisp_kerl_ctx.eco_bkl_status);
        printk("[SHDISP] kerl_ctx.usb_chg_status                = %d.\n", shdisp_kerl_ctx.usb_chg_status);
        printk("[SHDISP] kerl_ctx.ledc_status.ledc_is_exist     = %d.\n", shdisp_kerl_ctx.ledc_status.ledc_is_exist);
        printk("[SHDISP] kerl_ctx.ledc_status.power_status      = %d.\n", shdisp_kerl_ctx.ledc_status.power_status);
        printk("[SHDISP] kerl_ctx.ledc_status.ledc_req.red      = %d.\n", (int)shdisp_kerl_ctx.ledc_status.ledc_req.red);
        printk("[SHDISP] kerl_ctx.ledc_status.ledc_req.green    = %d.\n", (int)shdisp_kerl_ctx.ledc_status.ledc_req.green);
        printk("[SHDISP] kerl_ctx.ledc_status.ledc_req.blue     = %d.\n", (int)shdisp_kerl_ctx.ledc_status.ledc_req.blue);
        printk("[SHDISP] kerl_ctx.ledc_status.ledc_req.led_mode = %d.\n", shdisp_kerl_ctx.ledc_status.ledc_req.led_mode);
        printk("[SHDISP] kerl_ctx.ledc_status.ledc_req.on_count = %d.\n", shdisp_kerl_ctx.ledc_status.ledc_req.on_count);
        printk("[SHDISP] kerl_ctx.shdisp_lcd                    = 0x%04X.\n", (unsigned int)shdisp_kerl_ctx.shdisp_lcd);
        printk("[SHDISP] kerl_ctx.lcddr_phy_gamma.status        = 0x%02X.\n", shdisp_kerl_ctx.lcddr_phy_gamma.status);
        printk("[SHDISP] kerl_ctx.lcddr_phy_gamma.buf           = ");
        for(i = 0; i < SHDISP_LCDDR_PHY_GAMMA_BUF_MAX; i++) {
            printk("%02X,", shdisp_kerl_ctx.lcddr_phy_gamma.buf[i]);
        }
        printk("\n");
        printk("[SHDISP] kerl_ctx.lcddr_phy_gamma.chksum        = 0x%04X.\n", shdisp_kerl_ctx.lcddr_phy_gamma.chksum);
        printk("[SHDISP] kerl_ctx.lcddr_rom_gamma.buf           = ");
        for(i = 0; i < SHDISP_LCDDR_PHY_GAMMA_BUF_MAX; i++) {
            printk("%02X,", shdisp_kerl_ctx.lcddr_rom_gamma.buf[i]);
        }
        printk("\n");
        printk("[SHDISP] shdisp_als_irq_req_state               = 0x%02X.\n", shdisp_als_irq_req_state );
        for( i=0; i<NUM_SHDISP_IRQ_TYPE ; i++){
            if( shdisp_callback_table[i] != NULL )
                printk("[SHDISP] shdisp_callback_table[%d]              = subscribed.\n",i);
            else
                printk("[SHDISP] shdisp_callback_table[%d]              = no subscribed.\n",i);
        }
        printk("[SHDISP] KERNEL INFO <<-\n");
        break;
    case SHDISP_DEBUG_INFO_TYPE_BDIC:
        shdisp_bdic_API_DBG_INFO_output();
        break;
    }
    
    return;
}
#endif /* CONFIG_ANDROID_ENGINEERING */


#if defined (CONFIG_ANDROID_ENGINEERING)
/* ------------------------------------------------------------------------- */
/* shdisp_dbg_que                                                            */
/* ------------------------------------------------------------------------- */

static void shdisp_dbg_que(int kind)
{
    unsigned int nRcvGFAC=0;
    struct shdisp_queue_data_t *qdata = NULL;
    int    i;
    int    bFirstQue = 0;
    int    ret;
    int    nBDIC_QueFac = 0;


    SHDISP_TRACE("shdisp_dbg_que : Start\n");

    //#define SHDISP_INT_ENABLE_GFAC          0x002C0108 -> 0x002C0308
    switch (kind) {
    case 1: nRcvGFAC = 0x00000000;        break;
    case 2: nRcvGFAC = 0x00200000;        break;
    case 3: nRcvGFAC = 0x00000100;        break;
    case 4: nRcvGFAC = 0x00200100;        break;
    case 5: nRcvGFAC = 0x00000008;        break;
    case 6: nRcvGFAC = 0x00200008;        break;
    case 7: nRcvGFAC = 0x00000108;        break;
    case 8: nRcvGFAC = 0x00200108;        break;
    case 9: nRcvGFAC = 0x00080000;        break;
    case 10:nRcvGFAC = 0x00280000;        break;
    case 11:nRcvGFAC = 0x00080100;        break;
    case 12:nRcvGFAC = 0x00280100;        break;
    case 13:nRcvGFAC = 0x00080008;        break;
    case 14:nRcvGFAC = 0x00280008;        break;
    case 15:nRcvGFAC = 0x00080108;        break;
    case 16:nRcvGFAC = 0x00280108;        break;
    case 17:nRcvGFAC = 0x00040000;        break;
    case 18:nRcvGFAC = 0x00240000;        break;
    case 19:nRcvGFAC = 0x00040100;        break;
    case 20:nRcvGFAC = 0x00240100;        break;
    case 21:nRcvGFAC = 0x00040008;        break;
    case 22:nRcvGFAC = 0x00240008;        break;
    case 23:nRcvGFAC = 0x00040108;        break;
    case 24:nRcvGFAC = 0x00240108;        break;
    case 25:nRcvGFAC = 0x000C0000;        break;
    case 26:nRcvGFAC = 0x002C0000;        break;
    case 27:nRcvGFAC = 0x000C0100;        break;
    case 28:nRcvGFAC = 0x002C0100;        break;
    case 29:nRcvGFAC = 0x000C0008;        break;
    case 30:nRcvGFAC = 0x002C0008;        break;
    case 31:nRcvGFAC = 0x000C0108;        break;
    case 32:nRcvGFAC = 0x002C0108;        break;
    case 33:nRcvGFAC = 0x00000200;        break;
    case 34:nRcvGFAC = 0x00080200;        break;
    case 35:nRcvGFAC = 0x00200200;        break;
    case 36:nRcvGFAC = 0x00280200;        break;
    case 37:nRcvGFAC = 0x00000300;        break;
    case 38:nRcvGFAC = 0x00080300;        break;
    case 39:nRcvGFAC = 0x00200300;        break;
    case 40:nRcvGFAC = 0x00280300;        break;
    case 41:nRcvGFAC = 0x00000208;        break;
    case 42:nRcvGFAC = 0x00080208;        break;
    case 43:nRcvGFAC = 0x00200208;        break;
    case 44:nRcvGFAC = 0x00280208;        break;
    case 45:nRcvGFAC = 0x00000308;        break;
    case 46:nRcvGFAC = 0x00080308;        break;
    case 47:nRcvGFAC = 0x00200308;        break;
    case 48:nRcvGFAC = 0x00280308;        break;
    case 49:nRcvGFAC = 0x00040200;        break;
    case 50:nRcvGFAC = 0x000C0200;        break;
    case 51:nRcvGFAC = 0x00240200;        break;
    case 52:nRcvGFAC = 0x002C0200;        break;
    case 53:nRcvGFAC = 0x00040300;        break;
    case 54:nRcvGFAC = 0x000C0300;        break;
    case 55:nRcvGFAC = 0x00240300;        break;
    case 56:nRcvGFAC = 0x002C0300;        break;
    case 57:nRcvGFAC = 0x00040208;        break;
    case 58:nRcvGFAC = 0x000C0208;        break;
    case 59:nRcvGFAC = 0x00240208;        break;
    case 60:nRcvGFAC = 0x002C0208;        break;
    case 61:nRcvGFAC = 0x00040308;        break;
    case 62:nRcvGFAC = 0x000C0308;        break;
    case 63:nRcvGFAC = 0x00240308;        break;
    case 64:nRcvGFAC = 0x002C0308;        break;

    default: nRcvGFAC = 0;                break;
    }

    shdisp_SYS_set_irq(SHDISP_IRQ_DISABLE);
    shdisp_wake_lock();

    shdisp_bdic_API_IRQ_dbg_set_fac(nRcvGFAC);

    do{
        shdisp_semaphore_start();
        ret = shdisp_bdic_API_IRQ_check_fac();
        shdisp_semaphore_end(__func__);
        if( ret != SHDISP_RESULT_SUCCESS ){
            SHDISP_TRACE("shdisp_workqueue_handler_gpio : no factory\n");
            break;
        }
        
        down(&shdisp_sem_irq_fac);
        for(i=0; i<SHDISP_IRQ_MAX_KIND; i++){
            shdisp_semaphore_start();
            nBDIC_QueFac = shdisp_bdic_API_IRQ_get_fac(i);
            shdisp_semaphore_end(__func__);
            if( nBDIC_QueFac == SHDISP_BDIC_IRQ_TYPE_NONE )
                break;
            
            if (shdisp_wq_gpio_task) {
                qdata = kmalloc( sizeof(shdisp_queue_data), GFP_KERNEL );
                if( qdata != NULL ){
                    qdata->irq_GFAC = nBDIC_QueFac;
                    list_add_tail(&qdata->list, &shdisp_queue_data.list);
                    if( bFirstQue == 0 ){
                        bFirstQue = 1;
                        shdisp_wake_lock();
                        ret = queue_work(shdisp_wq_gpio_task, &shdisp_wq_gpio_task_wk);
                        if( ret == 0 ){
                            shdisp_wake_unlock();
                            SHDISP_ERR("<QUEUE_WORK_FAILURE> shdisp_workqueue_handler_gpio.\n");
                        }
                    }
                }
                else{
                   SHDISP_ERR("<QUEUE_WORK_FAILURE> shdisp_workqueue_handler_gpio:kmalloc failed (BDIC_QueFac=%d)\n", nBDIC_QueFac);
                }
            }
        }
        up(&shdisp_sem_irq_fac);

    }while(0);

    shdisp_semaphore_start();
    shdisp_bdic_API_IRQ_Clear();
    shdisp_semaphore_end(__func__);

    if( shdisp_bdic_API_IRQ_check_DET() != SHDISP_BDIC_IRQ_TYPE_DET )
        shdisp_SYS_set_irq(SHDISP_IRQ_ENABLE);

    SHDISP_TRACE("shdisp_dbg_que : Finish\n");
    shdisp_wake_unlock();
}


/* ------------------------------------------------------------------------- */
/* shdisp_debug_subscribe                                                    */
/* ------------------------------------------------------------------------- */

static void shdisp_debug_subscribe(void)
{
    struct shdisp_subscribe dbg_subs;

    dbg_subs.irq_type = SHDISP_IRQ_TYPE_PS;
    dbg_subs.callback = callback_ps;
    shdisp_api_event_subscribe(&dbg_subs);
    
}


/* ------------------------------------------------------------------------- */
/* callback_ps                                                               */
/* ------------------------------------------------------------------------- */

static void callback_ps(void)
{
    printk("[SHDISP] callback_ps Start\n");
    msleep(1000);
    printk("[SHDISP] callback_ps Finish\n");
}


#endif /* CONFIG_ANDROID_ENGINEERING */


/* ------------------------------------------------------------------------- */
/* shdisp_fb_open                                                            */
/* ------------------------------------------------------------------------- */

static void shdisp_fb_open(void)
{
    struct fb_info *info = NULL;
    
    if (sh_smem_common != NULL) {
        if ((sh_smem_common->sh_boot_mode == SHDISP_BOOT_MODE_OFF_CHARGE) ||
            (sh_smem_common->sh_boot_mode == SHDISP_BOOT_MODE_USB_OFFCHARGE)) {
            return;
        }
    }
    
    if (!num_registered_fb){
        return;
    }
    info = registered_fb[0];
    if (!info){
        return;
    }
    if (!try_module_get(info->fbops->owner)){
        return;
    }
    if (info->fbops->fb_open && info->fbops->fb_open(info, 0)) {
        module_put(info->fbops->owner);
        return;
    }
    return;
}


/* ------------------------------------------------------------------------- */
/* shdisp_fb_close                                                           */
/* ------------------------------------------------------------------------- */

static void shdisp_fb_close(void)
{
    struct fb_info *info = NULL;
    
    info = registered_fb[0];
    if (!info){
        return;
    }
    if (info->fbops->fb_release){
        info->fbops->fb_release(info, 0);
    }
    module_put(info->fbops->owner);
    return;
}


/* ------------------------------------------------------------------------- */
/* shdisp_panel_API_create                                                   */
/* ------------------------------------------------------------------------- */

static void shdisp_panel_API_create(void)
{
    shdisp_panel_fops = &shdisp_def_fops;
    
#if defined(CONFIG_SHDISP_PANEL_SWITCH)
    if(shdisp_api_get_panel_info() == 0) {
#if defined(CONFIG_SHDISP_PANEL_KITE)
        shdisp_panel_fops = shdisp_kite_API_create();
#endif
    }
    else {
#if defined(CONFIG_SHDISP_PANEL_PHARAOH)
        shdisp_panel_fops = shdisp_pharaoh_API_create();
#endif
    }
#else
#if defined(CONFIG_SHDISP_PANEL_PHARAOH)
    shdisp_panel_fops = shdisp_pharaoh_API_create();
#elif defined(CONFIG_SHDISP_PANEL_NICOLE)
    shdisp_panel_fops = shdisp_nicole_API_create();
#elif defined(CONFIG_SHDISP_PANEL_TAKT)
    shdisp_panel_fops = shdisp_takt_API_create();
#elif defined(CONFIG_SHDISP_PANEL_KITE)
    shdisp_panel_fops = shdisp_kite_API_create();
#elif defined(CONFIG_SHDISP_PANEL_RYOMA)
    shdisp_panel_fops = shdisp_ryoma_API_create();
#elif defined(CONFIG_SHDISP_PANEL_FLUTE)
    shdisp_panel_fops = shdisp_flute_API_create();
#endif /* CONFIG_SHDISP_PANEL */
#endif /* CONFIG_SHDISP_PANEL_SWITCH */
}


/* ------------------------------------------------------------------------- */
/* shdisp_panel_API_init_io                                                  */
/* ------------------------------------------------------------------------- */

static int shdisp_panel_API_init_io(void)
{
    if (shdisp_panel_fops->init_io) {
        return shdisp_panel_fops->init_io();
    }
    return SHDISP_RESULT_SUCCESS;
}


/* ------------------------------------------------------------------------- */
/* shdisp_panel_API_exit_io                                                  */
/* ------------------------------------------------------------------------- */

static int shdisp_panel_API_exit_io(void)
{
    if (shdisp_panel_fops->exit_io) {
        return shdisp_panel_fops->exit_io();
    }
    return SHDISP_RESULT_SUCCESS;
}


/* ------------------------------------------------------------------------- */
/* shdisp_panel_API_init_isr                                                 */
/* ------------------------------------------------------------------------- */

static int shdisp_panel_API_init_isr(void)
{
    if (shdisp_panel_fops->init_isr) {
        return shdisp_panel_fops->init_isr();
    }
    return SHDISP_RESULT_SUCCESS;
}

/* ------------------------------------------------------------------------- */
/* shdisp_panel_API_set_param                                                */
/* ------------------------------------------------------------------------- */

static void shdisp_panel_API_set_param(struct shdisp_panel_param_str *param_str)
{
    if (shdisp_panel_fops->set_param) {
        shdisp_panel_fops->set_param(param_str);
    }
}


/* ------------------------------------------------------------------------- */
/* shdisp_panel_API_power_on                                                 */
/* ------------------------------------------------------------------------- */

static int shdisp_panel_API_power_on(void)
{
    if (shdisp_panel_fops->power_on) {
        return shdisp_panel_fops->power_on();
    }
    return SHDISP_RESULT_SUCCESS;
}


/* ------------------------------------------------------------------------- */
/* shdisp_panel_API_power_off                                                */
/* ------------------------------------------------------------------------- */

static int shdisp_panel_API_power_off(void)
{
    if (shdisp_panel_fops->power_off) {
        return shdisp_panel_fops->power_off();
    }
    return SHDISP_RESULT_SUCCESS;
}


/* ------------------------------------------------------------------------- */
/* shdisp_panel_API_disp_init_1st                                            */
/* ------------------------------------------------------------------------- */

static int shdisp_panel_API_disp_init_1st(void)
{
    if (shdisp_panel_fops->init_1st) {
        return shdisp_panel_fops->init_1st();
    }
    return SHDISP_RESULT_SUCCESS;
}


/* ------------------------------------------------------------------------- */
/* shdisp_panel_API_disp_init_2nd                                            */
/* ------------------------------------------------------------------------- */

static int shdisp_panel_API_disp_init_2nd(void)
{
    if (shdisp_panel_fops->init_2nd) {
        return shdisp_panel_fops->init_2nd();
    }
    return SHDISP_RESULT_SUCCESS;
}


/* ------------------------------------------------------------------------- */
/* shdisp_panel_API_disp_on                                                  */
/* ------------------------------------------------------------------------- */

static int shdisp_panel_API_disp_on(void)
{
    if (shdisp_panel_fops->disp_on) {
        return shdisp_panel_fops->disp_on();
    }
    return SHDISP_RESULT_SUCCESS;
}


/* ------------------------------------------------------------------------- */
/* shdisp_panel_API_sleep                                                    */
/* ------------------------------------------------------------------------- */

static int shdisp_panel_API_sleep(void)
{
    if (shdisp_panel_fops->sleep) {
        return shdisp_panel_fops->sleep();
    }
    return SHDISP_RESULT_SUCCESS;
}


/* ------------------------------------------------------------------------- */
/* shdisp_panel_API_deep_standby                                             */
/* ------------------------------------------------------------------------- */

static int shdisp_panel_API_deep_standby(void)
{
    if (shdisp_panel_fops->deep_standby) {
        return shdisp_panel_fops->deep_standby();
    }
    return SHDISP_RESULT_SUCCESS;
}


/* ------------------------------------------------------------------------- */
/* shdisp_panel_API_diag_write_reg                                           */
/* ------------------------------------------------------------------------- */

static int shdisp_panel_API_diag_write_reg(unsigned char addr, unsigned char *write_data, unsigned char size)
{
    if (shdisp_panel_fops->write_reg) {
        return shdisp_panel_fops->write_reg(addr, write_data, size);
    }
    return SHDISP_RESULT_SUCCESS;
}


/* ------------------------------------------------------------------------- */
/* shdisp_panel_API_diag_read_reg                                            */
/* ------------------------------------------------------------------------- */

static int shdisp_panel_API_diag_read_reg(unsigned char addr, unsigned char *read_data, unsigned char size)
{
    if (shdisp_panel_fops->read_reg) {
        return shdisp_panel_fops->read_reg(addr, read_data, size);
    }
    return SHDISP_RESULT_SUCCESS;
}


/* ------------------------------------------------------------------------- */
/* shdisp_panel_API_diag_set_flicker_param                                   */
/* ------------------------------------------------------------------------- */

static int shdisp_panel_API_diag_set_flicker_param(unsigned short alpha)
{
    if (shdisp_panel_fops->set_flicker) {
        return shdisp_panel_fops->set_flicker(alpha);
    }
    return SHDISP_RESULT_SUCCESS;
}


/* ------------------------------------------------------------------------- */
/* shdisp_panel_API_diag_get_flicker_param                                   */
/* ------------------------------------------------------------------------- */

static int shdisp_panel_API_diag_get_flicker_param(unsigned short *alpha)
{
    if (shdisp_panel_fops->get_flicker) {
        return shdisp_panel_fops->get_flicker(alpha);
    }
    return SHDISP_RESULT_SUCCESS;
}


/* ------------------------------------------------------------------------- */
/* shdisp_panel_API_diag_get_flicker_low_param                               */
/* ------------------------------------------------------------------------- */

static int shdisp_panel_API_diag_get_flicker_low_param(unsigned short *alpha)
{
    if (shdisp_panel_fops->get_flicker_low) {
        return shdisp_panel_fops->get_flicker_low(alpha);
    }
    return SHDISP_RESULT_SUCCESS;
}


/* ------------------------------------------------------------------------- */
/* shdisp_panel_API_check_recovery                                           */
/* ------------------------------------------------------------------------- */

static int shdisp_panel_API_check_recovery(void)
{
    if (shdisp_panel_fops->check_recovery) {
        return shdisp_panel_fops->check_recovery();
    }
    return SHDISP_RESULT_SUCCESS;
}


/* ------------------------------------------------------------------------- */
/* shdisp_panel_API_get_recovery_type                                        */
/* ------------------------------------------------------------------------- */

static int shdisp_panel_API_get_recovery_type(int *type)
{
    if (shdisp_panel_fops->recovery_type) {
        return shdisp_panel_fops->recovery_type(type);
    }
    return SHDISP_RESULT_SUCCESS;
}


/* ------------------------------------------------------------------------- */
/* shdisp_init                                                               */
/* ------------------------------------------------------------------------- */

static int __init shdisp_init(void)
{
    int ret, idx;
    struct shdisp_bdic_state_str    state_str;
    struct shdisp_panel_param_str   param_str;
    int shdisp_subscribe_type;
    int i;
    unsigned long int notify_value = 0, notify_brightness = 0;
    struct shdisp_main_bkl_ctl bkl_ctl;
    struct shdisp_tri_led tri_led;
    
#if defined (CONFIG_ANDROID_ENGINEERING)
    struct proc_dir_entry *entry;
#endif /* CONFIG_ANDROID_ENGINEERING */
    
    shdisp_init_context();

    shdisp_panel_API_create();

    shdisp_SYS_Host_gpio_init();

    ret = alloc_chrdev_region(&shdisp_dev, 0, 1, SHDISP_NAME);

    if (!ret) {
        shdisp_major = MAJOR(shdisp_dev);
        shdisp_minor = MINOR(shdisp_dev);
    }
    else {
        goto shdisp_err_1;
    }
    
    cdev_init(&shdisp_cdev, &shdisp_fops);

    shdisp_cdev.owner = THIS_MODULE;
    shdisp_cdev.ops   = &shdisp_fops;

    ret = cdev_add(&shdisp_cdev, MKDEV(shdisp_major,0), 1);

    if (ret) {
        goto shdisp_err_2;
    }

    shdisp_class = class_create(THIS_MODULE, SHDISP_NAME);

    if (IS_ERR(shdisp_class)) {
        goto shdisp_err_3;
    }

    device_create(shdisp_class, NULL,
                  shdisp_dev, &shdisp_cdev, SHDISP_NAME);

    ret = shdisp_SYS_bdic_i2c_init();

    if (ret) {
        goto shdisp_err_4;
    }

#ifdef CONFIG_SHLCDC_LED_BD2802GU
    ret = shdisp_ledc_API_init();

    if (ret) {
        goto shdisp_err_5;
    }
#endif
    ret = shdisp_SYS_Host_control(SHDISP_HOST_CTL_CMD_LCD_CLK_INIT, 0);
    if (ret) {
        goto shdisp_err_6;
    }

    ret = shdisp_panel_API_init_io();

    if (ret) {
        goto shdisp_err_61;
    }

    ret = shdisp_panel_API_init_isr();

    if (ret) {
        goto shdisp_err_7;
    }

#if defined (CONFIG_ANDROID_ENGINEERING)
    entry = create_proc_entry("driver/SHDISP", 0666, NULL); 
    
    if (entry == NULL) {
        goto shdisp_err_7;
    }
    
    entry->write_proc = shdisp_proc_write;
#endif /* CONFIG_ANDROID_ENGINEERING */
    
    sema_init(&shdisp_sem, 1);
    
    sema_init(&shdisp_sem_callback, 1);
    sema_init(&shdisp_sem_irq_fac, 1 );
    sema_init(&shdisp_lux_change_sem, 1);
    sema_init(&shdisp_sem_timer, 1);
    
    spin_lock_init( &shdisp_q_lock );
    spin_lock_init( &shdisp_wake_spinlock );
    
	
	

    shdisp_wake_lock_init();

    memset(&shdisp_queue_data, 0, sizeof(shdisp_queue_data));
    INIT_LIST_HEAD( &shdisp_queue_data.list);
    
    shdisp_wq_gpio = create_workqueue("shdisp_gpio_queue");
    
    if (shdisp_wq_gpio) {
        INIT_WORK(&shdisp_wq_gpio_wk,
                  shdisp_workqueue_handler_gpio);
    }
    else{
        goto shdisp_err_8;
    }
    
    shdisp_wq_gpio_task = create_workqueue("shdisp_gpio_queue_task");

    if (shdisp_wq_gpio_task) {
        INIT_WORK(&shdisp_wq_gpio_task_wk,
                  shdisp_workqueue_gpio_task);
    }
    else{
        goto shdisp_err_9;
    }

    down(&shdisp_sem_callback);
    for(i=0; i<NUM_SHDISP_IRQ_TYPE ; i++)
        shdisp_callback_table[i] = NULL;
    up(&shdisp_sem_callback);
    
    init_timer(&shdisp_timer);

    shdisp_wq_timer_task = create_workqueue("shdisp_timer_queue_task");
    if (shdisp_wq_timer_task) {
        INIT_WORK(&shdisp_wq_timer_task_wk, shdisp_workqueue_timer_task);
    }
    else {
        goto shdisp_err_10;
    }
    
    for (i=0; i<NUM_SHDISP_IRQ_TYPE ; i++) {
        shdisp_subscribe_type = SHDISP_SUBSCRIBE_TYPE_INT;
        
        if (i == SHDISP_IRQ_TYPE_DET) {
            shdisp_panel_API_get_recovery_type(&shdisp_subscribe_type);
        }
        
        shdisp_subscribe_type_table[i] = shdisp_subscribe_type;
    }
    
    shdisp_wq_sensor_start = create_workqueue("shdisp_sensor_start_queue");
    if (shdisp_wq_sensor_start) {
    	INIT_DELAYED_WORK(&shdisp_sensor_start_wk,
    			  shdisp_sensor_start_handler);
    }
    else{
        goto shdisp_err_11;
    }

    shdisp_kerl_ctx.driver_is_initialized = SHDISP_DRIVER_IS_INITIALIZED;
    
#ifdef SHDISP_SW_DISABLE_SMEM_INFO
    shdisp_boot_ctx.bdic_is_exist = shdisp_bdic_API_boot_init();
    shdisp_kerl_ctx.bdic_is_exist = shdisp_boot_ctx.bdic_is_exist;
#endif /* SHDISP_SW_DISABLE_SMEM_INFO */
    
    param_str.vcom_alpha = shdisp_kerl_ctx.alpha;
    param_str.vcom_alpha_low = shdisp_kerl_ctx.alpha_low;
    param_str.shdisp_lcd = shdisp_kerl_ctx.shdisp_lcd;
    shdisp_panel_API_set_param(&param_str);
    
    state_str.bdic_power_status = SHDISP_BDIC_PWR_STATUS_OFF;
    for (idx = 0; idx < NUM_SHDISP_BDIC_DEV_TYPE; idx++)
    {
        state_str.bdic_power_status_tbl[idx] = SHDISP_BDIC_DEV_PWR_OFF;
    }
    
#ifndef SHDISP_SW_DISABLE_SMEM_INFO
    if (shdisp_kerl_ctx.upper_unit_is_connected == SHDISP_UPPER_UNIT_IS_CONNECTED) {
        if (shdisp_kerl_ctx.main_disp_status == SHDISP_MAIN_DISP_ON) {
            state_str.bdic_power_status = SHDISP_BDIC_PWR_STATUS_ACTIVE;
            state_str.bdic_power_status_tbl[SHDISP_BDIC_DEV_TYPE_LCD_PWR] = SHDISP_BDIC_DEV_PWR_ON;
        }
        if (shdisp_kerl_ctx.main_bkl.mode != SHDISP_MAIN_BKL_MODE_OFF) {
            state_str.bdic_power_status = SHDISP_BDIC_PWR_STATUS_ACTIVE;
            state_str.bdic_power_status_tbl[SHDISP_BDIC_DEV_TYPE_LCD_BKL] = SHDISP_BDIC_DEV_PWR_ON;
        }
        if ((shdisp_kerl_ctx.tri_led.red != 0) || (shdisp_kerl_ctx.tri_led.green != 0) || (shdisp_kerl_ctx.tri_led.blue != 0)) {
            state_str.bdic_power_status = SHDISP_BDIC_PWR_STATUS_ACTIVE;
            state_str.bdic_power_status_tbl[SHDISP_BDIC_DEV_TYPE_TRI_LED] = SHDISP_BDIC_DEV_PWR_ON;
        }
    }
#endif /* SHDISP_SW_DISABLE_SMEM_INFO */
    
    state_str.bdic_is_exist = shdisp_kerl_ctx.bdic_is_exist;
    state_str.als_adj0_low  = (unsigned char)(shdisp_kerl_ctx.photo_sensor_adj.als_adj0);
    state_str.als_adj0_high = (unsigned char)(shdisp_kerl_ctx.photo_sensor_adj.als_adj0 >> 8);
    state_str.als_adj1_low  = (unsigned char)(shdisp_kerl_ctx.photo_sensor_adj.als_adj1);
    state_str.als_adj1_high = (unsigned char)(shdisp_kerl_ctx.photo_sensor_adj.als_adj1 >> 8);
    state_str.als_shift     = shdisp_kerl_ctx.photo_sensor_adj.als_shift;
    state_str.clear_offset  = shdisp_kerl_ctx.photo_sensor_adj.clear_offset;
    state_str.ir_offset     = shdisp_kerl_ctx.photo_sensor_adj.ir_offset;
    shdisp_bdic_API_check_bkl_adj_param(shdisp_kerl_ctx.dma_lut_status, &(state_str.bkl_adjust_val));
    shdisp_bdic_API_initialize(&state_str);
    
    bkl_ctl.mode  = shdisp_kerl_ctx.main_bkl.mode;
    bkl_ctl.param = shdisp_kerl_ctx.main_bkl.param;
    shdisp_bdic_API_LCD_BKL_set_request(SHDISP_MAIN_BKL_DEV_TYPE_APP, &bkl_ctl);
    
    tri_led.red      = shdisp_kerl_ctx.tri_led.red;
    tri_led.green    = shdisp_kerl_ctx.tri_led.green;
    tri_led.blue     = shdisp_kerl_ctx.tri_led.blue;
    tri_led.ext_mode = shdisp_kerl_ctx.tri_led.ext_mode;
    tri_led.led_mode = shdisp_kerl_ctx.tri_led.led_mode;
    tri_led.ontime   = shdisp_kerl_ctx.tri_led.ontime;
    tri_led.interval = shdisp_kerl_ctx.tri_led.interval;
    shdisp_bdic_API_TRI_LED_set_request(&tri_led);
    
    init_completion(&lux_change_notify);
    
    ret = shdisp_SYS_request_irq( shdisp_gpio_int_isr );
    if (ret) {
        goto shdisp_err_12;
    }
    
    shdisp_fb_open();
    
#ifdef CONFIG_SHTERM
    shterm_k_set_info(SHTERM_INFO_LCDPOW, shdisp_kerl_ctx.main_disp_status);
#endif /* CONFIG_SHTERM */
    
    if (shdisp_kerl_ctx.main_bkl.mode != SHDISP_MAIN_BKL_MODE_OFF) {
        notify_value = 1;
        shdisp_bdic_API_LCD_BKL_get_param( &notify_brightness );
    }
    
#ifdef CONFIG_SHTERM
    shterm_k_set_info(SHTERM_INFO_BACKLIGHT, notify_value);
    shterm_k_set_info(SHTERM_INFO_BACKLIGHT_LEV, notify_brightness);
#endif /* CONFIG_SHTERM */
    
    
    return 0;

shdisp_err_12:
    flush_workqueue(shdisp_wq_sensor_start);
    destroy_workqueue(shdisp_wq_sensor_start);
    shdisp_wq_sensor_start = NULL;

shdisp_err_11:
    flush_workqueue(shdisp_wq_timer_task);
    destroy_workqueue(shdisp_wq_timer_task);
    shdisp_wq_timer_task = NULL;

shdisp_err_10:
    flush_workqueue(shdisp_wq_gpio_task);
    destroy_workqueue(shdisp_wq_gpio_task);
    shdisp_wq_gpio_task = NULL;

shdisp_err_9:
    flush_workqueue(shdisp_wq_gpio);
    destroy_workqueue(shdisp_wq_gpio);
    shdisp_wq_gpio = NULL;

shdisp_err_8:
shdisp_err_7:
    shdisp_panel_API_exit_io();
    
shdisp_err_61:
    shdisp_SYS_Host_control(SHDISP_HOST_CTL_CMD_LCD_CLK_EXIT, 0);
    
shdisp_err_6:
#ifdef CONFIG_SHLCDC_LED_BD2802GU
    shdisp_ledc_API_exit();
    
shdisp_err_5:
#endif
    shdisp_SYS_bdic_i2c_exit();
    
shdisp_err_4:
    device_destroy(shdisp_class, MKDEV(shdisp_major,0));
    
shdisp_err_3:
    class_destroy(shdisp_class);
    
shdisp_err_2:
    cdev_del(&shdisp_cdev);

shdisp_err_1:
    shdisp_SYS_Host_gpio_exit();
    unregister_chrdev_region(MKDEV(shdisp_major,0), 1);
    return -1;
}
module_init(shdisp_init);


/* ------------------------------------------------------------------------- */
/* shdisp_exit                                                               */
/* ------------------------------------------------------------------------- */

static void shdisp_exit(void)
{
    shdisp_fb_close();

    lux_change_wait_flg = SHDISP_LUX_CHANGE_STATE_EXIT;
    complete(&lux_change_notify);


    wake_lock_destroy(&shdisp_wake_lock_wq);

    if (shdisp_wq_sensor_start) {
        cancel_delayed_work(&shdisp_sensor_start_wk);
        flush_workqueue(shdisp_wq_sensor_start);
        destroy_workqueue(shdisp_wq_sensor_start);
    }

    shdisp_SYS_Host_control(SHDISP_HOST_CTL_CMD_LCD_CLK_EXIT, 0);
    shdisp_SYS_free_irq();
    if (shdisp_wq_gpio) {
        flush_workqueue(shdisp_wq_gpio);
        destroy_workqueue(shdisp_wq_gpio);
        shdisp_wq_gpio = NULL;
    }

    if (shdisp_wq_gpio_task) {
        flush_workqueue(shdisp_wq_gpio_task);
        destroy_workqueue(shdisp_wq_gpio_task);
        shdisp_wq_gpio_task = NULL;
    }
    
    shdisp_panel_API_exit_io();
#ifdef CONFIG_SHLCDC_LED_BD2802GU
    shdisp_ledc_API_exit();
#endif
    shdisp_SYS_bdic_i2c_exit();
    shdisp_SYS_Host_gpio_exit();
    device_destroy(shdisp_class, MKDEV(shdisp_major,0));
    class_destroy(shdisp_class);
    cdev_del(&shdisp_cdev);
    unregister_chrdev_region(MKDEV(shdisp_major,0), 1);
    return;
}
module_exit(shdisp_exit);


MODULE_DESCRIPTION("SHARP DISPLAY DRIVER MODULE");
MODULE_LICENSE("GPL v2");
MODULE_AUTHOR("SHARP CORPORATION");
MODULE_VERSION("1.00");

/* ------------------------------------------------------------------------- */
/* END OF FILE                                                               */
/* ------------------------------------------------------------------------- */
