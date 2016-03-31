/* drivers/sharp/shdisp/shdisp_system.c  (Display Driver)
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
#include <linux/spi/spi.h>
#include <mach/gpio.h>
#include <mach/msm_iomap.h>
#include <asm/io.h>
#include <linux/clk.h>
#include <sharp/sh_smem.h>
#include "shdisp_system.h"
#include "../../../arch/arm/mach-msm/include/mach/msm_xo.h"


/* ------------------------------------------------------------------------- */
/* MACROS                                                                    */
/* ------------------------------------------------------------------------- */
#if defined(CONFIG_MACH_DECKARD_GP4)
#define CONFIG_SHDISP_EXCLK_PMIC_D1
#elif defined(CONFIG_MACH_DECKARD_AS70) || defined(CONFIG_MACH_TOR)
#define CONFIG_SHDISP_EXCLK_MSM
#endif

#define SHDISP_INT_IRQ MSM_GPIO_TO_INT(52)
//#define SHDISP_INT_IRQ MSM_GPIO_TO_INT(69)
//#define SHDISP_INT_FLAGS (IRQF_TRIGGER_FALLING | IRQF_DISABLED)
#define SHDISP_INT_FLAGS (IRQF_TRIGGER_LOW | IRQF_DISABLED)

/* ------------------------------------------------------------------------- */
/* PROTOTYPES                                                                */
/* ------------------------------------------------------------------------- */

static int shdisp_host_gpio_request(int num);
static int shdisp_host_gpio_free(int num);
static void shdisp_host_ctl_lcd_clk_start(unsigned long rate);
static void shdisp_host_ctl_lcd_clk_stop(unsigned long rate);
static int shdisp_host_ctl_lcd_clk_init(void);
static int shdisp_host_ctl_lcd_clk_exit(void);
static int  shdisp_bdic_i2c_probe(struct i2c_client *client, const struct i2c_device_id * devid);
static int  shdisp_bdic_i2c_remove(struct i2c_client *client);
static int __devinit shdisp_pharaoh_sio_probe(struct spi_device *spi);
static int __devinit shdisp_pharaoh_sio_remove(struct spi_device *spi);
static int  shdisp_nicole_i2c_probe(struct i2c_client *client, const struct i2c_device_id * devid);
static int  shdisp_nicole_i2c_remove(struct i2c_client *client);
static int  shdisp_takt_i2c_probe(struct i2c_client *client, const struct i2c_device_id * devid);
static int  shdisp_takt_i2c_remove(struct i2c_client *client);
static int shdisp_ledc_i2c_probe(struct i2c_client *client, const struct i2c_device_id * devid);
static int shdisp_ledc_i2c_remove(struct i2c_client *client);


/* ------------------------------------------------------------------------- */
/* TYPES                                                                     */
/* ------------------------------------------------------------------------- */

typedef struct bdic_data_tag
{
    struct i2c_client *this_client;
} bdic_i2c_data_t;

static const struct i2c_device_id shdisp_bdic_id[] = {
    { SHDISP_BDIC_I2C_DEVNAME, 0 },
    { }
};

static struct i2c_driver bdic_driver =
{
    .driver = {
        .owner   = THIS_MODULE,
        .name    = SHDISP_BDIC_I2C_DEVNAME,
    },
    .class    = I2C_CLASS_HWMON,
    .probe    = shdisp_bdic_i2c_probe,
    .id_table = shdisp_bdic_id,
    .remove   = shdisp_bdic_i2c_remove,
};

static struct spi_driver mipi_spi_driver = {
    .driver = {
        .name = "mipi_spi",
        .owner = THIS_MODULE,
    },
    .probe = shdisp_pharaoh_sio_probe,
    .remove = __devexit_p(shdisp_pharaoh_sio_remove),
};

typedef struct nicole_data_tag
{
    struct i2c_client *this_client;
} nicole_i2c_data_t;

static const struct i2c_device_id shdisp_nicole_id[] = {
    { SHDISP_NICOLE_I2C_DEVNAME, 0 },
    { }
};

static struct i2c_driver nicole_driver =
{
    .driver = {
        .owner   = THIS_MODULE,
        .name    = SHDISP_NICOLE_I2C_DEVNAME,
    },
    .class    = I2C_CLASS_HWMON,
    .probe    = shdisp_nicole_i2c_probe,
    .id_table = shdisp_nicole_id,
    .remove   = shdisp_nicole_i2c_remove,
};

typedef struct takt_data_tag
{
    struct i2c_client *this_client;
} takt_i2c_data_t;

static const struct i2c_device_id shdisp_takt_id[] = {
    { SHDISP_TAKT_I2C_DEVNAME, 0 },
    { }
};

static struct i2c_driver takt_driver =
{
    .driver = {
        .owner   = THIS_MODULE,
        .name    = SHDISP_TAKT_I2C_DEVNAME,
    },
    .class    = I2C_CLASS_HWMON,
    .probe    = shdisp_takt_i2c_probe,
    .id_table = shdisp_takt_id,
    .remove   = shdisp_takt_i2c_remove,
};

typedef struct ledc_data_tag
{
    struct i2c_client *this_client;
} ledc_i2c_data_t;

static const struct i2c_device_id shdisp_ledc_id[] = {
    { SHDISP_LEDC_I2C_DEVNAME, 0 },
    { }
};

static struct i2c_driver ledc_driver =
{
    .driver = {
        .owner   = THIS_MODULE,
        .name    = SHDISP_LEDC_I2C_DEVNAME,
    },
    .class    = I2C_CLASS_HWMON,
    .probe    = shdisp_ledc_i2c_probe,
    .id_table = shdisp_ledc_id,
    .remove   = shdisp_ledc_i2c_remove,
};

/* ------------------------------------------------------------------------- */
/* VARIABLES                                                                 */
/* ------------------------------------------------------------------------- */

static bdic_i2c_data_t *bdic_i2c_p = NULL;
static nicole_i2c_data_t *nicole_i2c_p = NULL;
static takt_i2c_data_t *takt_i2c_p = NULL;
static ledc_i2c_data_t *ledc_i2c_p = NULL;
static struct spi_device *spid = NULL;
static unsigned short sendaddr[1];
static unsigned short senddata[17];
static unsigned char  readdata[32];
#if defined(CONFIG_SHDISP_EXCLK_PMIC_D1)
static struct msm_xo_voter *xo_handle;
#elif defined(CONFIG_SHDISP_EXCLK_MSM)
static struct clk *gp0_clk;
#endif

#if defined(CONFIG_SHDISP_EXCLK_MSM)
static int shdisp_lcd_clk_count = 0;
#endif

/* ------------------------------------------------------------------------- */
/* DEBUG MACRAOS                                                             */
/* ------------------------------------------------------------------------- */
#define SHDISP_SYSTEM_FILE "shdisp_system.c"
#define SHDISP_ERR(fmt, args...) \
        printk("[SHDISP_ERROR][%s][%s] " fmt, SHDISP_SYSTEM_FILE, __func__, ## args);

#ifdef SHDISP_SYS_SW_TIME_API
static void shdisp_dbg_api_wait_start(void);
static void shdisp_dbg_api_wait_end(unsigned long usec);
#define SHDISP_SYS_DBG_API_WAIT_START           shdisp_dbg_api_wait_start();
#define SHDISP_SYS_DBG_API_WAIT_END(usec)       shdisp_dbg_api_wait_end(usec);
struct shdisp_sys_dbg_api_info {
    int flag;
    struct timespec t_api_start;
    struct timespec t_wait_start;
    struct timespec t_wait_req;
    struct timespec t_wait_sum;
};
static struct shdisp_sys_dbg_api_info shdisp_sys_dbg_api;
#ifdef SHDISP_SYS_SW_TIME_BDIC
static void shdisp_dbg_bdic_init(void);
static void shdisp_dbg_bdic_logout(void);
static void shdisp_dbg_bdic_singl_write_start(void);
static void shdisp_dbg_bdic_singl_write_retry(void);
static void shdisp_dbg_bdic_singl_write_end(int ret);
static void shdisp_dbg_bdic_singl_read_start(void);
static void shdisp_dbg_bdic_singl_read_retry(void);
static void shdisp_dbg_bdic_singl_read_end(int ret);
static void shdisp_dbg_bdic_multi_read_start(void);
static void shdisp_dbg_bdic_multi_read_retry(void);
static void shdisp_dbg_bdic_multi_read_end(int ret);
#define SHDISP_SYS_DBG_DBIC_INIT                shdisp_dbg_bdic_init();
#define SHDISP_SYS_DBG_DBIC_LOGOUT              shdisp_dbg_bdic_logout();
#define SHDISP_SYS_DBG_DBIC_SINGL_W_START       shdisp_dbg_bdic_singl_write_start();
#define SHDISP_SYS_DBG_DBIC_SINGL_W_RETRY       shdisp_dbg_bdic_singl_write_retry();
#define SHDISP_SYS_DBG_DBIC_SINGL_W_END(ret)    shdisp_dbg_bdic_singl_write_end(ret);
#define SHDISP_SYS_DBG_DBIC_SINGL_R_START       shdisp_dbg_bdic_singl_read_start();
#define SHDISP_SYS_DBG_DBIC_SINGL_R_RETRY       shdisp_dbg_bdic_singl_read_retry();
#define SHDISP_SYS_DBG_DBIC_SINGL_R_END(ret)    shdisp_dbg_bdic_singl_read_end(ret);
#define SHDISP_SYS_DBG_DBIC_MULTI_R_START       shdisp_dbg_bdic_multi_read_start();
#define SHDISP_SYS_DBG_DBIC_MULTI_R_RETRY       shdisp_dbg_bdic_multi_read_retry();
#define SHDISP_SYS_DBG_DBIC_MULTI_R_END(ret)    shdisp_dbg_bdic_multi_read_end(ret);
struct shdisp_sys_dbg_i2c_rw_info {
    unsigned long   w_singl_ok_count;
    unsigned long   w_singl_ng_count;
    unsigned long   w_singl_retry;
    struct timespec w_singl_t_start;
    struct timespec w_singl_t_sum;
    unsigned long   r_singl_ok_count;
    unsigned long   r_singl_ng_count;
    unsigned long   r_singl_retry;
    struct timespec r_singl_t_start;
    struct timespec r_singl_t_sum;
    unsigned long   r_multi_ok_count;
    unsigned long   r_multi_ng_count;
    unsigned long   r_multi_retry;
    struct timespec r_multi_t_start;
    struct timespec r_multi_t_sum;
};
static struct shdisp_sys_dbg_i2c_rw_info shdisp_sys_dbg_bdic;
#else  /* SHDISP_SYS_SW_TIME_BDIC */
#define SHDISP_SYS_DBG_DBIC_INIT
#define SHDISP_SYS_DBG_DBIC_LOGOUT
#define SHDISP_SYS_DBG_DBIC_SINGL_W_START
#define SHDISP_SYS_DBG_DBIC_SINGL_W_RETRY
#define SHDISP_SYS_DBG_DBIC_SINGL_W_END(ret)
#define SHDISP_SYS_DBG_DBIC_SINGL_R_START
#define SHDISP_SYS_DBG_DBIC_SINGL_R_RETRY
#define SHDISP_SYS_DBG_DBIC_SINGL_R_END(ret)
#define SHDISP_SYS_DBG_DBIC_MULTI_R_START
#define SHDISP_SYS_DBG_DBIC_MULTI_R_RETRY
#define SHDISP_SYS_DBG_DBIC_MULTI_R_END(ret)
#endif /* SHDISP_SYS_SW_TIME_BDIC */
#else  /* SHDISP_SYS_SW_TIME_API */
#define SHDISP_SYS_DBG_API_WAIT_START
#define SHDISP_SYS_DBG_API_WAIT_END(usec)
#define SHDISP_SYS_DBG_DBIC_INIT
#define SHDISP_SYS_DBG_DBIC_SINGL_W_START
#define SHDISP_SYS_DBG_DBIC_SINGL_W_RETRY
#define SHDISP_SYS_DBG_DBIC_SINGL_W_END(ret)
#define SHDISP_SYS_DBG_DBIC_SINGL_R_START
#define SHDISP_SYS_DBG_DBIC_SINGL_R_RETRY
#define SHDISP_SYS_DBG_DBIC_SINGL_R_END(ret)
#define SHDISP_SYS_DBG_DBIC_MULTI_R_START
#define SHDISP_SYS_DBG_DBIC_MULTI_R_RETRY
#define SHDISP_SYS_DBG_DBIC_MULTI_R_END(ret)
#endif /* SHDISP_SYS_SW_TIME_API */

/* ------------------------------------------------------------------------- */
/* FUNCTIONS                                                                 */
/* ------------------------------------------------------------------------- */
/* ------------------------------------------------------------------------- */
/* API                                                                       */
/* ------------------------------------------------------------------------- */
/* ------------------------------------------------------------------------- */
/* shdisp_SYS_Host_control                                                   */
/* ------------------------------------------------------------------------- */

int  shdisp_SYS_Host_control(int cmd, unsigned long rate)
{
    int ret = SHDISP_RESULT_SUCCESS;
    switch (cmd) {
    case SHDISP_HOST_CTL_CMD_LCD_CLK_START:
        shdisp_host_ctl_lcd_clk_start(rate);
        break;
    case SHDISP_HOST_CTL_CMD_LCD_CLK_STOP:
        shdisp_host_ctl_lcd_clk_stop(rate);
        break;
    case SHDISP_HOST_CTL_CMD_LCD_CLK_INIT:
        ret = shdisp_host_ctl_lcd_clk_init();
        break;
    case SHDISP_HOST_CTL_CMD_LCD_CLK_EXIT:
        ret = shdisp_host_ctl_lcd_clk_exit();
        break;
    default:
        SHDISP_ERR("<INVALID_VALUE> cmd(%d), rate(%d).\n", cmd, (int)rate);
        return SHDISP_RESULT_FAILURE;
    }
    
    return ret;
}


/* ------------------------------------------------------------------------- */
/* shdisp_SYS_delay_us                                                       */
/* ------------------------------------------------------------------------- */

void shdisp_SYS_delay_us(unsigned long usec)
{
    struct timespec tu;
    
    if (usec >= 1000*1000) {
        tu.tv_sec  = usec / 1000000;
        tu.tv_nsec = (usec % 1000000) * 1000;
    }
    else
    {
        tu.tv_sec  = 0;
        tu.tv_nsec = usec * 1000;
    }
    
    SHDISP_SYS_DBG_API_WAIT_START;
    
    hrtimer_nanosleep(&tu, NULL, HRTIMER_MODE_REL, CLOCK_MONOTONIC);
    
    SHDISP_SYS_DBG_API_WAIT_END(usec);
    
    return;
}


/* ------------------------------------------------------------------------- */
/* shdisp_SYS_Host_gpio_init                                                 */
/* ------------------------------------------------------------------------- */

void shdisp_SYS_Host_gpio_init(void)
{
    
    shdisp_host_gpio_request(SHDISP_GPIO_NUM_BL_RST_N);
    
#ifdef CONFIG_SHLCDC_LED_BD2802GU
    shdisp_host_gpio_request(SHDISP_GPIO_NUM_LEDC_RST_N);
#endif
    
    
    return;
}


/* ------------------------------------------------------------------------- */
/* shdisp_SYS_Host_gpio_exit                                                 */
/* ------------------------------------------------------------------------- */

void shdisp_SYS_Host_gpio_exit(void)
{
    shdisp_host_gpio_free(SHDISP_GPIO_NUM_BL_RST_N);
    
#ifdef CONFIG_SHLCDC_LED_BD2802GU
    shdisp_host_gpio_free(SHDISP_GPIO_NUM_LEDC_RST_N);
#endif
}


/* ------------------------------------------------------------------------- */
/* shdisp_SYS_Host_gpio_request                                              */
/* ------------------------------------------------------------------------- */

int  shdisp_SYS_Host_gpio_request(int num)
{
    return shdisp_host_gpio_request(num);
}


/* ------------------------------------------------------------------------- */
/* shdisp_SYS_Host_gpio_free                                                 */
/* ------------------------------------------------------------------------- */

int  shdisp_SYS_Host_gpio_free(int num)
{
    return shdisp_host_gpio_free(num);
}


/* ------------------------------------------------------------------------- */
/* shdisp_SYS_set_Host_gpio                                                  */
/* ------------------------------------------------------------------------- */

int  shdisp_SYS_set_Host_gpio(int num, int value)
{
    if (value != SHDISP_GPIO_CTL_LOW &&
        value != SHDISP_GPIO_CTL_HIGH ) {
        SHDISP_ERR("<INVALID_VALUE> value(%d).\n", value);
        return SHDISP_RESULT_FAILURE;
    }
    
    switch (num) {
    case SHDISP_GPIO_NUM_BL_RST_N:
    case SHDISP_GPIO_NUM_SPI_CS:
    case SHDISP_GPIO_NUM_LCD_SCS_N:
#ifdef CONFIG_SHLCDC_LED_BD2802GU
    case SHDISP_GPIO_NUM_LEDC_RST_N:
#endif
    case SHDISP_GPIO_NUM_CLK_SEL:
        gpio_set_value(num, value);
        return SHDISP_RESULT_SUCCESS;
        
    default:
        SHDISP_ERR("<INVALID_VALUE> num(%d).\n", num);
        return SHDISP_RESULT_FAILURE;
    }
}


/* ------------------------------------------------------------------------- */
/* shdisp_SYS_request_irq                                                    */
/* ------------------------------------------------------------------------- */

int  shdisp_SYS_request_irq(irqreturn_t (*irq_handler)( int , void * ) )
{
    int ret;

    if( irq_handler == NULL )
        return SHDISP_RESULT_FAILURE;

    printk("[SHDISP]shdisp_SYS_request_irq: Start\n");
        
    ret = request_irq(SHDISP_INT_IRQ, *irq_handler,
                      SHDISP_INT_FLAGS, "shdisp", NULL);

    if( ret == 0 ){
        printk("[SHDISP]shdisp_SYS_request_irq: disable_irq\n");
        disable_irq(SHDISP_INT_IRQ);
    }
    
    return ret;
}


/* ------------------------------------------------------------------------- */
/* shdisp_SYS_free_irq                                                     */
/* ------------------------------------------------------------------------- */

void  shdisp_SYS_free_irq(void)
{
    free_irq(SHDISP_INT_IRQ, NULL);
}


/* ------------------------------------------------------------------------- */
/* shdisp_SYS_set_irq                                                        */
/* ------------------------------------------------------------------------- */

int  shdisp_SYS_set_irq( int enable )
{
    int ret = SHDISP_RESULT_SUCCESS;
    
    if( enable == SHDISP_IRQ_ENABLE ){
        enable_irq_wake(SHDISP_INT_IRQ);
        enable_irq(SHDISP_INT_IRQ);
    }
    else if( enable == SHDISP_IRQ_DISABLE ){
        disable_irq_nosync(SHDISP_INT_IRQ);
        disable_irq_wake(SHDISP_INT_IRQ);
    }
    else{
        SHDISP_ERR("<INVALID_VALUE> shdisp_SYS_set_irq:enable=%d.\n", enable);
        ret = SHDISP_RESULT_FAILURE;
    }
    return ret;
}


/* ------------------------------------------------------------------------- */
/* shdisp_SYS_bdic_i2c_init                                                  */
/* ------------------------------------------------------------------------- */

int  shdisp_SYS_bdic_i2c_init(void)
{
    int ret;
    
    ret = i2c_add_driver(&bdic_driver);
    if ( ret < 0 ) {
        SHDISP_ERR("<RESULT_FAILURE> i2c_add_driver.\n");
        return SHDISP_RESULT_FAILURE;
    }
    return SHDISP_RESULT_SUCCESS;
}


/* ------------------------------------------------------------------------- */
/* shdisp_SYS_bdic_i2c_exit                                                  */
/* ------------------------------------------------------------------------- */

int  shdisp_SYS_bdic_i2c_exit(void)
{
    i2c_del_driver(&bdic_driver);
    
    return SHDISP_RESULT_SUCCESS;
}


/* ------------------------------------------------------------------------- */
/* shdisp_SYS_bdic_i2c_write                                                 */
/* ------------------------------------------------------------------------- */

int  shdisp_SYS_bdic_i2c_write(unsigned char addr, unsigned char data)
{
    struct i2c_msg msg;
    unsigned char write_buf[2];
    int i2c_ret;
    int result = 1;
    int retry;
    
    SHDISP_SYS_DBG_DBIC_SINGL_W_START;
    
    msg.addr     = bdic_i2c_p->this_client->addr;
    msg.flags    = 0;
    msg.len      = 2;
    msg.buf      = write_buf;
    write_buf[0] = addr;
    write_buf[1] = data;
    
    for (retry = 0; retry <= 10; retry++) {
        i2c_ret = i2c_transfer(bdic_i2c_p->this_client->adapter, &msg, 1);
        if ( i2c_ret > 0 ) {
            result = 0;
            break;
        }
        else {
            SHDISP_SYS_DBG_DBIC_SINGL_W_RETRY;
        }
    }
    
    SHDISP_SYS_DBG_DBIC_SINGL_W_END(result);
    
    if (result == 1) {
        SHDISP_ERR("<OTHER> i2c_transfer time out(i2c_ret = %d).\n", i2c_ret);
        return SHDISP_RESULT_FAILURE_I2C_TMO;
    }
    
    return SHDISP_RESULT_SUCCESS;
}


/* ------------------------------------------------------------------------- */
/* shdisp_SYS_bdic_i2c_read                                                  */
/* ------------------------------------------------------------------------- */

int  shdisp_SYS_bdic_i2c_read(unsigned char addr, unsigned char *data)
{
    struct i2c_msg msg;
    unsigned char write_buf[2];
    unsigned char read_buf[2];
    int i2c_ret;
    int result = 1;
    int retry;
    
    if (data == NULL) {
        SHDISP_ERR("<NULL_POINTER> data.\n");
        return SHDISP_RESULT_FAILURE;
    }
    
    SHDISP_SYS_DBG_DBIC_SINGL_R_START;
    
    for(retry = 0; retry <= 10; retry++){
        msg.addr     = bdic_i2c_p->this_client->addr;
        msg.flags    = 0;
        msg.len      = 1;
        msg.buf      = write_buf;
        write_buf[0] = addr;
        
        i2c_ret = i2c_transfer(bdic_i2c_p->this_client->adapter, &msg, 1);
        
        if( i2c_ret > 0 ){
            
            msg.addr  = bdic_i2c_p->this_client->addr;
            msg.flags = I2C_M_RD;
            msg.len   = 1;
            msg.buf   = read_buf;
            
            i2c_ret = i2c_transfer(bdic_i2c_p->this_client->adapter, &msg, 1);
            if( i2c_ret > 0 ){
                *data = read_buf[0];
                result = 0;
                break;
            }
            else {
                SHDISP_SYS_DBG_DBIC_SINGL_R_RETRY;
            }
        }
        else {
            SHDISP_SYS_DBG_DBIC_SINGL_R_RETRY;
        }
    }
    
    SHDISP_SYS_DBG_DBIC_SINGL_R_END(result);
    
    if(result == 1)
    {
        SHDISP_ERR("<OTHER> i2c_transfer time out(i2c_ret = %d).\n", i2c_ret);
        return SHDISP_RESULT_FAILURE_I2C_TMO;
    }
    
    return SHDISP_RESULT_SUCCESS;
}


/* ------------------------------------------------------------------------- */
/* shdisp_SYS_bdic_i2c_multi_read                                            */
/* ------------------------------------------------------------------------- */

int  shdisp_SYS_bdic_i2c_multi_read(unsigned char addr, unsigned char *data, int size)
{
    struct i2c_msg msg;
    unsigned char write_buf[2];
    unsigned char read_buf[1+8];
    int i2c_ret;
    int result = 1;
    int retry;
    
    if ((size < 1) || (size > 8)) {
        SHDISP_ERR("<INVALID_VALUE> size(%d).\n", size);
        return SHDISP_RESULT_FAILURE;
    }
    
    if (data == NULL) {
        SHDISP_ERR("<NULL_POINTER> data.\n");
        return SHDISP_RESULT_FAILURE;
    }
    
    SHDISP_SYS_DBG_DBIC_MULTI_R_START;
    
    msg.addr     = bdic_i2c_p->this_client->addr;
    msg.flags    = 0;
    msg.len      = 1;
    msg.buf      = write_buf;
    write_buf[0] = addr;
    
    for(retry = 0; retry <= 10; retry++){
        i2c_ret = i2c_transfer(bdic_i2c_p->this_client->adapter, &msg, 1);
        if( i2c_ret > 0 ){
            result = 0;
            break;
        }
        else {
            SHDISP_SYS_DBG_DBIC_MULTI_R_RETRY;
        }
    }
    
    if( result == 0 ){
        msg.addr  = bdic_i2c_p->this_client->addr;
        msg.flags = I2C_M_RD;
        msg.len   = size;
        msg.buf   = read_buf;
        for(retry = 0; retry <= 10; retry++){
            i2c_ret = i2c_transfer(bdic_i2c_p->this_client->adapter, &msg, 1);
            if( i2c_ret > 0 ){
                memcpy(data, &read_buf[0], size);
                result = 0;
                break;
            }
            else {
                SHDISP_SYS_DBG_DBIC_MULTI_R_RETRY;
            }
        }
    }
    
    SHDISP_SYS_DBG_DBIC_MULTI_R_END(result);
    
    if(result == 1)
    {
        SHDISP_ERR("<OTHER> i2c_transfer time out(i2c_ret = %d).\n", i2c_ret);
        return SHDISP_RESULT_FAILURE_I2C_TMO;
    }
    
    return SHDISP_RESULT_SUCCESS;
}




/* ------------------------------------------------------------------------- */
/* shdisp_SYS_pharaoh_sio_init                                               */
/* ------------------------------------------------------------------------- */

int  shdisp_SYS_pharaoh_sio_init(void)
{
    int ret;
    
    ret = spi_register_driver(&mipi_spi_driver);
    if ( ret < 0 ) {
        SHDISP_ERR("<RESULT_FAILURE> spi_register_driver.\n");
        return SHDISP_RESULT_FAILURE;
    }
    
    return SHDISP_RESULT_SUCCESS;
}


/* ------------------------------------------------------------------------- */
/* shdisp_SYS_pharaoh_sio_exit                                               */
/* ------------------------------------------------------------------------- */

int  shdisp_SYS_pharaoh_sio_exit(void)
{
    spi_unregister_driver(&mipi_spi_driver);
    
    return SHDISP_RESULT_SUCCESS;
}


/* ------------------------------------------------------------------------- */
/* shdisp_SYS_pharaoh_sio_transfer                                           */
/* ------------------------------------------------------------------------- */

void shdisp_SYS_pharaoh_sio_transfer(unsigned short reg, unsigned char *wbuf, int wlen, unsigned char *rbuf, int rlen)
{
    struct spi_message  m;
    struct spi_transfer *x, xfer[4];
    unsigned short wdata;
    const unsigned char *data;
    int i, ret = 0;
    
    spi_message_init(&m);
    
    memset(xfer, 0, sizeof(xfer));
    x = &xfer[0];
    
    wdata = (unsigned short)reg;
    
    if(wdata & 0x0001){
        wdata = ( 0x8000 | 0x0000 | (wdata >> 1) );
    }else{
        wdata = ( 0x0000 | 0x0000 | (wdata >> 1) );
    }
    
    sendaddr[0] = wdata;
    
    x->tx_buf           = sendaddr;
    x->bits_per_word    = 9;
    x->len              = 2;
    x->speed_hz         = 1100000;
    spi_message_add_tail(x, &m);
    
    if (rlen) {
        /* SPI_CS LOW */
        gpio_tlmm_config(GPIO_CFG(SHDISP_GPIO_NUM_SPI_CS, 0, GPIO_CFG_OUTPUT, GPIO_CFG_NO_PULL, GPIO_CFG_2MA), GPIO_CFG_ENABLE);
        shdisp_SYS_set_Host_gpio(SHDISP_GPIO_NUM_SPI_CS, SHDISP_GPIO_CTL_LOW);
        
        ret = spi_sync(spid, &m);
        if (ret < 0) {
            printk("[SHDISP] : spi_sync ret=%d \n", ret);
        }
        
        spi_message_init(&m);
        
        x = &xfer[1];
        x->rx_buf           = readdata;
        x->len              = rlen;
        x->bits_per_word    = 8;
        x->speed_hz         = 1100000;
        spi_message_add_tail(x, &m);
        
        ret = spi_sync(spid, &m);
        if (ret < 0) {
            printk("[SHDISP] : spi_sync ret=%d \n", ret);
        }
        
        /* SPI_CS HIGH */
        shdisp_SYS_set_Host_gpio(SHDISP_GPIO_NUM_SPI_CS, SHDISP_GPIO_CTL_HIGH);
        gpio_tlmm_config(GPIO_CFG(SHDISP_GPIO_NUM_SPI_CS, 1, GPIO_CFG_OUTPUT, GPIO_CFG_NO_PULL, GPIO_CFG_2MA), GPIO_CFG_ENABLE);
        
        for (i=0; i<rlen; i++) {
            *rbuf = readdata[i];
            rbuf++;
        }
        
    } else if (wlen > 0) {
        x++;
        
        data = wbuf;
        for (i = 0; i < wlen; i++) {
            wdata = ( 0x0100 | (unsigned short)(*data) );

            if (wdata & 0x0001) {
                wdata = ( 0x8000 | 0x0080 | (wdata >> 1) );
            }
            else
            {
                wdata = ( 0x0000 | 0x0080 | (wdata >> 1) );
            }
            
            senddata[i] = wdata;
            data++;
        }
        
        x->tx_buf           = senddata;
        x->len              = wlen * 2;
        x->bits_per_word    = 9;
        x->speed_hz         = 1100000;
        spi_message_add_tail(x, &m);

        ret = spi_sync(spid, &m);
        if (ret < 0) {
            printk("[SHDISP] : spi_sync ret=%d \n", ret);
        }


    } else {
        ret = spi_sync(spid, &m);
        if (ret < 0) {
            printk("[SHDISP] : spi_sync ret=%d \n", ret);
        }
    }
    
    if (ret < 0) {
        printk("[SHDISP] : spi_sync ret=%d \n", ret);
    }
    
    return;
}






/* ------------------------------------------------------------------------- */
/* shdisp_SYS_nicole_i2c_init                                                */
/* ------------------------------------------------------------------------- */
int  shdisp_SYS_nicole_i2c_init(void)
{
    int ret;
    
    ret = i2c_add_driver(&nicole_driver);
    if ( ret < 0 ) {
        SHDISP_ERR("<RESULT_FAILURE> i2c_add_driver.\n");
        return SHDISP_RESULT_FAILURE;
    }
    
    return SHDISP_RESULT_SUCCESS;
}


/* ------------------------------------------------------------------------- */
/* shdisp_SYS_nicole_i2c_exit                                                */
/* ------------------------------------------------------------------------- */
int  shdisp_SYS_nicole_i2c_exit(void)
{
    i2c_del_driver(&nicole_driver);
    
    return SHDISP_RESULT_SUCCESS;
}


/* ------------------------------------------------------------------------- */
/* shdisp_SYS_nicole_i2c_write                                               */
/* ------------------------------------------------------------------------- */
int  shdisp_SYS_nicole_i2c_write(unsigned char addr, unsigned char *data, int size)
{
    unsigned char write_buf[1+8];
    int i2c_ret;
    int i = 1;
    int result = 1;
    int retry;
    
    if(size <= 0 || size >= 9){
        SHDISP_ERR("<OTHER> i2c_transfer size error.\n");
        return SHDISP_RESULT_FAILURE;
    }
    
    memset(write_buf, 0, sizeof(write_buf));
    
    write_buf[0] = addr;
    
    for (i=1; i<=size; i++) {
       write_buf[i] = data[i-1];
    }
    
    for (retry = 0; retry <= 10; retry++) {
        i2c_ret = i2c_master_send(nicole_i2c_p->this_client, write_buf, size+1);
        if (i2c_ret > 0) {
            result = 0;
            break;
        }
    }
    
    if (result == 1) {
        SHDISP_ERR("<OTHER> i2c_transfer time out(i2c_ret = %d).\n", i2c_ret);
        return SHDISP_RESULT_FAILURE_I2C_TMO;
    }
    
    return SHDISP_RESULT_SUCCESS;
}


/* ------------------------------------------------------------------------- */
/* shdisp_SYS_nicole_i2c_read                                                */
/* ------------------------------------------------------------------------- */
int  shdisp_SYS_nicole_i2c_read(unsigned char addr, unsigned char *data, int size)
{
    unsigned char read_buf[1+8];
    int i2c_ret;
    int result = 1;
    int retry;

    for(retry = 0; retry <= 10; retry++){
        i2c_ret = i2c_master_recv(nicole_i2c_p->this_client, read_buf, size+1);
        if (i2c_ret > 0) {
            memcpy(data, &read_buf[1], size);
            result = 0;
            break;
        }
        else {
            result = 1;
        }
    }
    
    if (result == 1) {
        SHDISP_ERR("<OTHER> i2c_transfer time out(i2c_ret = %d).\n", i2c_ret);
        return SHDISP_RESULT_FAILURE_I2C_TMO;
    }

    return SHDISP_RESULT_SUCCESS;
}


/* ------------------------------------------------------------------------- */
/* shdisp_SYS_takt_i2c_init                                                  */
/* ------------------------------------------------------------------------- */

int  shdisp_SYS_takt_i2c_init(void)
{
    int ret;

    ret = i2c_add_driver(&takt_driver);
    if ( ret < 0 ) {
        SHDISP_ERR("<RESULT_FAILURE> i2c_add_driver.\n");
        return SHDISP_RESULT_FAILURE;
    }

    return SHDISP_RESULT_SUCCESS;
}


/* ------------------------------------------------------------------------- */
/* shdisp_SYS_takt_i2c_exit                                                  */
/* ------------------------------------------------------------------------- */

int  shdisp_SYS_takt_i2c_exit(void)
{
    i2c_del_driver(&takt_driver);
    
    return SHDISP_RESULT_SUCCESS;
}


/* ------------------------------------------------------------------------- */
/* shdisp_SYS_takt_i2c_write                                                 */
/* ------------------------------------------------------------------------- */

int  shdisp_SYS_takt_i2c_write(unsigned char addr, unsigned char *data, int size)
{
    unsigned char write_buf[2];
    int i2c_ret;
    int result = 1;
    int retry;

    memset(write_buf, 0, sizeof(write_buf));

    write_buf[0] = addr;
    write_buf[1] = data[0];
    for (retry = 0; retry <= 10; retry++) {
        i2c_ret = i2c_master_send(takt_i2c_p->this_client, write_buf, 2);
        if ( i2c_ret > 0 ) {
            result = 0;
            break;
        }else{
            result = 1;
        }
    }


    if (result == 1) {
        SHDISP_ERR("<OTHER> i2c_transfer time out(i2c_ret = %d).\n", i2c_ret);
        return SHDISP_RESULT_FAILURE_I2C_TMO;
    }

    return SHDISP_RESULT_SUCCESS;
}


/* ------------------------------------------------------------------------- */
/* shdisp_SYS_takt_i2c_read                                                  */
/* ------------------------------------------------------------------------- */

int  shdisp_SYS_takt_i2c_read(unsigned char addr, unsigned char *data, int size)
{

    unsigned char write_buf[2];
    unsigned char read_buf[2];
    int i2c_ret;
    int result = 1;
    int retry;

    write_buf[0] = data[0];
    write_buf[1] = addr;
    
    for(retry = 0; retry <= 10; retry++){
        i2c_ret = i2c_master_send(takt_i2c_p->this_client, write_buf, 2);
        if( i2c_ret > 0 ){
            result = 0;
            break;
        }
    }

    if( result == 0 ){
        for(retry = 0; retry <= 10; retry++){
            i2c_ret = i2c_master_recv(takt_i2c_p->this_client, read_buf, size+1);
            if( i2c_ret > 0 ){
                int i = 1;
                for(i=1; i<=size; i++){
                }
                memcpy(data, &read_buf[1], size);
                result = 0;
                break;
            }else{
                result = 1;
            }
        }
    }
    
    if(result == 1)
    {
        SHDISP_ERR("<OTHER> i2c_transfer time out(i2c_ret = %d).\n", i2c_ret);
        return SHDISP_RESULT_FAILURE_I2C_TMO;
    }

    return SHDISP_RESULT_SUCCESS;
}


/* ------------------------------------------------------------------------- */
/* shdisp_SYS_ledc_i2c_init                                                  */
/* ------------------------------------------------------------------------- */

int shdisp_SYS_ledc_i2c_init(void)
{
    int ret;
    
    ret = i2c_add_driver(&ledc_driver);
    if ( ret < 0 ) {
        SHDISP_ERR("<RESULT_FAILURE> ledc i2c_add_driver.\n");
        return SHDISP_RESULT_FAILURE;
    }
    
    return SHDISP_RESULT_SUCCESS;
}


/* ------------------------------------------------------------------------- */
/* shdisp_SYS_ledc_i2c_exit                                                  */
/* ------------------------------------------------------------------------- */

int shdisp_SYS_ledc_i2c_exit(void)
{
    i2c_del_driver(&ledc_driver);
    
    return SHDISP_RESULT_SUCCESS;
}


/* ------------------------------------------------------------------------- */
/* shdisp_SYS_ledc_i2c_write                                                 */
/* ------------------------------------------------------------------------- */

int shdisp_SYS_ledc_i2c_write(unsigned char addr, unsigned char data)
{
    struct i2c_msg msg;
    unsigned char write_buf[2];
    int i2c_ret;
    int result = 1;
    int retry;
    
    msg.addr     = ledc_i2c_p->this_client->addr;
    msg.flags    = 0;
    msg.len      = 2;
    msg.buf      = write_buf;
    write_buf[0] = addr;
    write_buf[1] = data;
    
    for (retry = 0; retry <= 10; retry++) {
        i2c_ret = i2c_transfer(ledc_i2c_p->this_client->adapter, &msg, 1);
        if ( i2c_ret > 0 ) {
            result = 0;
            break;
        }
    }
    
    if (result == 1) {
        SHDISP_ERR("<OTHER> ledc w i2c_transfer time out(i2c_ret = %d).\n", i2c_ret);
        return SHDISP_RESULT_FAILURE_I2C_TMO;
    }
    
    return SHDISP_RESULT_SUCCESS;
}


/* ------------------------------------------------------------------------- */
/* shdisp_SYS_ledc_i2c_read                                                  */
/* ------------------------------------------------------------------------- */

int  shdisp_SYS_ledc_i2c_read(unsigned char addr, unsigned char *data)
{
    return SHDISP_RESULT_SUCCESS;
}


/* ------------------------------------------------------------------------- */
/* SUB ROUTINE                                                               */
/* ------------------------------------------------------------------------- */
/* ------------------------------------------------------------------------- */
/* shdisp_host_gpio_request                                                  */
/* ------------------------------------------------------------------------- */

static int shdisp_host_gpio_request(int num)
{
    int ret = SHDISP_RESULT_SUCCESS;
    
    switch (num) {
    case SHDISP_GPIO_NUM_BL_RST_N:
        gpio_request(SHDISP_GPIO_NUM_BL_RST_N, "BL_RST_N");
        break;
#ifdef CONFIG_SHLCDC_LED_BD2802GU
    case SHDISP_GPIO_NUM_LEDC_RST_N:
        gpio_request(SHDISP_GPIO_NUM_LEDC_RST_N, "LEDC_RST_N");
        break;
#endif
    case SHDISP_GPIO_NUM_CLK_SEL:
        gpio_request(SHDISP_GPIO_NUM_CLK_SEL, "LCD_CLK_SEL");
        break;
    default:
        SHDISP_ERR("<INVALID_VALUE> num(%d).\n", num);
        ret = SHDISP_RESULT_FAILURE;
        break;
    }
    
    return ret;
}


/* ------------------------------------------------------------------------- */
/* shdisp_host_gpio_free                                                     */
/* ------------------------------------------------------------------------- */

static int shdisp_host_gpio_free(int num)
{
    int ret = SHDISP_RESULT_SUCCESS;
    
    switch (num) {
    case SHDISP_GPIO_NUM_BL_RST_N:
#ifdef CONFIG_SHLCDC_LED_BD2802GU
    case SHDISP_GPIO_NUM_LEDC_RST_N:
#endif
    case SHDISP_GPIO_NUM_CLK_SEL:
        gpio_free(num);
        break;
    default:
        SHDISP_ERR("<INVALID_VALUE> num(%d).\n", num);
        ret = SHDISP_RESULT_FAILURE;
        break;
    }
    
    return ret;
}


/* ------------------------------------------------------------------------- */
/* shdisp_host_ctl_lcd_clk_start                                             */
/* ------------------------------------------------------------------------- */

static void shdisp_host_ctl_lcd_clk_start(unsigned long rate)
{
#if defined(CONFIG_SHDISP_EXCLK_PMIC_D1)
    int ret = 0;
    ret = msm_xo_mode_vote(xo_handle , MSM_XO_MODE_ON);
    if (ret) {
        SHDISP_ERR("%s failed to vote for TCXOD1. ret=%d\n", __func__, ret);
    }
#elif defined(CONFIG_SHDISP_EXCLK_MSM)

    if(shdisp_lcd_clk_count != 0) {
        return;
    }
    
    gp0_clk = clk_get(NULL, "gp0_clk");
    clk_set_rate(gp0_clk, rate);
    clk_enable(gp0_clk);
    
    shdisp_lcd_clk_count = 1;

#endif  /* CONFIG_SHDISP_EXCLK_*** */
    
    return;
}


/* ------------------------------------------------------------------------- */
/* shdisp_host_ctl_lcd_clk_stop                                              */
/* ------------------------------------------------------------------------- */

static void shdisp_host_ctl_lcd_clk_stop(unsigned long rate)
{
#if defined(CONFIG_SHDISP_EXCLK_PMIC_D1)
    int ret = 0;
    ret = msm_xo_mode_vote(xo_handle , MSM_XO_MODE_OFF);
    if (ret) {
        SHDISP_ERR("%s failed to vote for TCXOD1. ret=%d\n", __func__, ret);
    }
#elif defined(CONFIG_SHDISP_EXCLK_MSM)

    if(shdisp_lcd_clk_count == 0) {
        return;
    }
    
    clk_disable(gp0_clk);
    
    shdisp_lcd_clk_count = 0;
    
#endif  /* CONFIG_SHDISP_EXCLK_*** */
    
    return;
}


/* ------------------------------------------------------------------------- */
/* shdisp_host_ctl_lcd_clk_init                                              */
/* ------------------------------------------------------------------------- */

static int shdisp_host_ctl_lcd_clk_init()
{
    int ret = SHDISP_RESULT_SUCCESS;
#if defined(CONFIG_SHDISP_EXCLK_PMIC_D1)
    xo_handle = msm_xo_get(MSM_XO_TCXO_D1, "anonymous");
    if (IS_ERR(xo_handle)) {
        ret = PTR_ERR(xo_handle);
        SHDISP_ERR("%s not able to get the handle to vote for TCXO D1 buffer. ret=%d\n", __func__, ret);
        ret = SHDISP_RESULT_FAILURE;
    }

#endif  /* CONFIG_SHDISP_EXCLK_*** */
    
    return ret;
}


/* ------------------------------------------------------------------------- */
/* shdisp_host_ctl_lcd_clk_exit                                              */
/* ------------------------------------------------------------------------- */

static int shdisp_host_ctl_lcd_clk_exit()
{
    int ret = SHDISP_RESULT_SUCCESS;
#if defined(CONFIG_SHDISP_EXCLK_PMIC_D1)
    msm_xo_put(xo_handle);

#endif  /* CONFIG_SHDISP_EXCLK_*** */
    
    return ret;
}


/* ------------------------------------------------------------------------- */
/* shdisp_bdic_i2c_probe                                                     */
/* ------------------------------------------------------------------------- */

static int shdisp_bdic_i2c_probe(struct i2c_client *client, const struct i2c_device_id * devid)
{
    bdic_i2c_data_t* i2c_p;
    
    if(bdic_i2c_p != NULL){
        return -EPERM;
    }
    
    i2c_p = (bdic_i2c_data_t*)kzalloc(sizeof(bdic_i2c_data_t),GFP_KERNEL);
    if(i2c_p == NULL){
        return -ENOMEM;
    }
    
    bdic_i2c_p = i2c_p;
    
    i2c_set_clientdata(client,i2c_p);
    i2c_p->this_client = client;
    
    return 0;
}


/* ------------------------------------------------------------------------- */
/* shdisp_bdic_i2c_remove                                                    */
/* ------------------------------------------------------------------------- */

static int shdisp_bdic_i2c_remove(struct i2c_client *client)
{
    bdic_i2c_data_t* i2c_p;
    
    i2c_p = i2c_get_clientdata(client);
    
    kfree(i2c_p);
    
    return 0;
}


/* ------------------------------------------------------------------------- */
/* shdisp_pharaoh_sio_probe                                                  */
/* ------------------------------------------------------------------------- */

static int __devinit shdisp_pharaoh_sio_probe(struct spi_device *spi)
{
    spid=spi;
    
    return 0;
}


/* ------------------------------------------------------------------------- */
/* shdisp_pharaoh_sio_remove                                                 */
/* ------------------------------------------------------------------------- */

static int __devinit shdisp_pharaoh_sio_remove(struct spi_device *spi)
{
    return 0;
}


/* ------------------------------------------------------------------------- */
/* shdisp_nicole_i2c_probe                                                   */
/* ------------------------------------------------------------------------- */

static int  shdisp_nicole_i2c_probe(struct i2c_client *client, const struct i2c_device_id * devid)
{
    nicole_i2c_data_t* i2c_p;
    
    if(nicole_i2c_p != NULL){
        return -EPERM;
    }
    
    i2c_p = (nicole_i2c_data_t*)kzalloc(sizeof(nicole_i2c_data_t),GFP_KERNEL);
    if(i2c_p == NULL){
        return -ENOMEM;
    }
    
    nicole_i2c_p = i2c_p;
    
    i2c_set_clientdata(client,i2c_p);
    i2c_p->this_client = client;
    
    return 0;
}


/* ------------------------------------------------------------------------- */
/* shdisp_nicole_i2c_remove                                                  */
/* ------------------------------------------------------------------------- */

static int  shdisp_nicole_i2c_remove(struct i2c_client *client)
{
    nicole_i2c_data_t* i2c_p;
    
    i2c_p = i2c_get_clientdata(client);
    
    kfree(i2c_p);
    
    return 0;
}


/* ------------------------------------------------------------------------- */
/* shdisp_takt_i2c_probe                                                     */
/* ------------------------------------------------------------------------- */

static int shdisp_takt_i2c_probe(struct i2c_client *client, const struct i2c_device_id * devid)
{
    takt_i2c_data_t* i2c_p;
    
    if(takt_i2c_p != NULL){
        return -EPERM;
    }
    
    i2c_p = (takt_i2c_data_t*)kzalloc(sizeof(takt_i2c_data_t),GFP_KERNEL);
    if(i2c_p == NULL){
        return -ENOMEM;
    }
    
    takt_i2c_p = i2c_p;
    
    i2c_set_clientdata(client,i2c_p);
    i2c_p->this_client = client;
    
    return 0;
}


/* ------------------------------------------------------------------------- */
/* shdisp_takt_i2c_remove                                                    */
/* ------------------------------------------------------------------------- */

static int shdisp_takt_i2c_remove(struct i2c_client *client)
{
    takt_i2c_data_t* i2c_p;
    
    i2c_p = i2c_get_clientdata(client);
    
    kfree(i2c_p);
    
    return 0;
}




/* ------------------------------------------------------------------------- */
/* shdisp_ledc_i2c_probe                                                     */
/* ------------------------------------------------------------------------- */

static int shdisp_ledc_i2c_probe(struct i2c_client *client, const struct i2c_device_id * devid)
{
    ledc_i2c_data_t* i2c_p;
    
    if(ledc_i2c_p != NULL){
        return -EPERM;
    }
    
    i2c_p = (ledc_i2c_data_t*)kzalloc(sizeof(ledc_i2c_data_t),GFP_KERNEL);
    if(i2c_p == NULL){
        return -ENOMEM;
    }
    
    ledc_i2c_p = i2c_p;
    
    i2c_set_clientdata(client,i2c_p);
    i2c_p->this_client = client;
    
    return 0;
}


/* ------------------------------------------------------------------------- */
/* shdisp_ledc_i2c_remove                                                    */
/* ------------------------------------------------------------------------- */

static int shdisp_ledc_i2c_remove(struct i2c_client *client)
{
    ledc_i2c_data_t* i2c_p;
    
    i2c_p = i2c_get_clientdata(client);
    
    kfree(i2c_p);
    
    return 0;
}










#ifdef SHDISP_SYS_SW_TIME_API
/* ------------------------------------------------------------------------- */
/* shdisp_sys_dbg_hw_check_start                                             */
/* ------------------------------------------------------------------------- */

void shdisp_sys_dbg_hw_check_start(void)
{
    memset(&shdisp_sys_dbg_api, 0, sizeof(shdisp_sys_dbg_api));
    
    SHDISP_SYS_DBG_DBIC_INIT;
    
    shdisp_sys_dbg_api.flag = 1;
    
    getnstimeofday(&shdisp_sys_dbg_api.t_api_start);
}

/* ------------------------------------------------------------------------- */
/* shdisp_sys_dbg_hw_check_end                                               */
/* ------------------------------------------------------------------------- */

void shdisp_sys_dbg_hw_check_end(const char *func)
{
    struct timespec stop, df;
    u64 msec_api, msec_req, msec_sum;
    u64 usec_api, usec_req, usec_sum;
    
    getnstimeofday(&stop);
    
    df = timespec_sub(stop, shdisp_sys_dbg_api.t_api_start);
    
    msec_api = timespec_to_ns(&df);
    do_div(msec_api, NSEC_PER_USEC);
    usec_api = do_div(msec_api, USEC_PER_MSEC);
    
    msec_req = timespec_to_ns(&shdisp_sys_dbg_api.t_wait_req);
    do_div(msec_req, NSEC_PER_USEC);
    usec_req = do_div(msec_req, USEC_PER_MSEC);
    
    msec_sum = timespec_to_ns(&shdisp_sys_dbg_api.t_wait_sum);
    do_div(msec_sum, NSEC_PER_USEC);
    usec_sum = do_div(msec_sum, USEC_PER_MSEC);
    
    printk(KERN_ERR "[API]%s() total=%lu.%03lums, wait=%lu.%03lums( %lu.%03lums )\n", func, 
    (unsigned long)msec_api, (unsigned long)usec_api,
    (unsigned long)msec_sum, (unsigned long)usec_sum,
    (unsigned long)msec_req, (unsigned long)usec_req );
    
    SHDISP_SYS_DBG_DBIC_LOGOUT;
    
    shdisp_sys_dbg_api.flag = 0;
}

/* ------------------------------------------------------------------------- */
/* shdisp_dbg_api_wait_start                                                 */
/* ------------------------------------------------------------------------- */

static void shdisp_dbg_api_wait_start(void)
{
    if(shdisp_sys_dbg_api.flag == 0) {
        return;
    }
    
    getnstimeofday(&shdisp_sys_dbg_api.t_wait_start);
}

/* ------------------------------------------------------------------------- */
/* shdisp_dbg_api_wait_end                                                   */
/* ------------------------------------------------------------------------- */

static void shdisp_dbg_api_wait_end(unsigned long usec)
{
    struct timespec stop, df;
    
    if(shdisp_sys_dbg_api.flag == 0) {
        return;
    }
    
    getnstimeofday(&stop);
    
    df = timespec_sub(stop, shdisp_sys_dbg_api.t_wait_start);
    
    shdisp_sys_dbg_api.t_wait_sum = timespec_add(shdisp_sys_dbg_api.t_wait_sum, df);
    
    timespec_add_ns(&shdisp_sys_dbg_api.t_wait_req, (usec * 1000));
}

#ifdef SHDISP_SYS_SW_TIME_BDIC
/* ------------------------------------------------------------------------- */
/* shdisp_dbg_bdic_init                                                      */
/* ------------------------------------------------------------------------- */

static void shdisp_dbg_bdic_init(void)
{
    memset(&shdisp_sys_dbg_bdic, 0, sizeof(shdisp_sys_dbg_bdic));
}

/* ------------------------------------------------------------------------- */
/* shdisp_dbg_bdic_logout                                                    */
/* ------------------------------------------------------------------------- */

static void shdisp_dbg_bdic_logout(void)
{
    u64 nsec_wk;
    unsigned long usec_wk1, usec_avl;
    
    nsec_wk = timespec_to_ns(&shdisp_sys_dbg_bdic.w_singl_t_sum);
    if(nsec_wk != 0) {
        do_div(nsec_wk, NSEC_PER_USEC);
        usec_wk1 = nsec_wk;
        usec_avl = usec_wk1 / shdisp_sys_dbg_bdic.w_singl_ok_count;
        printk(KERN_ERR "[---] -- bdic w_s %lu,%lu,%lu, total=%lu.%03lums, avl=%lu.%03lums\n",
        shdisp_sys_dbg_bdic.w_singl_ok_count, shdisp_sys_dbg_bdic.w_singl_ng_count, shdisp_sys_dbg_bdic.w_singl_retry,
        usec_wk1/USEC_PER_MSEC, usec_wk1%USEC_PER_MSEC, usec_avl/USEC_PER_MSEC, usec_avl%USEC_PER_MSEC);
    }
    
    nsec_wk = timespec_to_ns(&shdisp_sys_dbg_bdic.r_singl_t_sum);
    if(nsec_wk != 0) {
        do_div(nsec_wk, NSEC_PER_USEC);
        usec_wk1 = nsec_wk;
        usec_avl = usec_wk1 / shdisp_sys_dbg_bdic.r_singl_ok_count;
        printk(KERN_ERR "[---] -- bdic r_s %lu,%lu,%lu, total=%lu.%03lums, avl=%lu.%03lums\n",
        shdisp_sys_dbg_bdic.r_singl_ok_count, shdisp_sys_dbg_bdic.r_singl_ng_count, shdisp_sys_dbg_bdic.r_singl_retry,
        usec_wk1/USEC_PER_MSEC, usec_wk1%USEC_PER_MSEC, usec_avl/USEC_PER_MSEC, usec_avl%USEC_PER_MSEC);
    }
    
    nsec_wk = timespec_to_ns(&shdisp_sys_dbg_bdic.r_multi_t_sum);
    if(nsec_wk != 0) {
        do_div(nsec_wk, NSEC_PER_USEC);
        usec_wk1 = nsec_wk;
        usec_avl = usec_wk1 / shdisp_sys_dbg_bdic.r_multi_ok_count;
        printk(KERN_ERR "[---] -- bdic r_m %lu,%lu,%lu, total=%lu.%03lums, avl=%lu.%03lums\n",
        shdisp_sys_dbg_bdic.r_multi_ok_count, shdisp_sys_dbg_bdic.r_multi_ng_count, shdisp_sys_dbg_bdic.r_multi_retry,
        usec_wk1/USEC_PER_MSEC, usec_wk1%USEC_PER_MSEC, usec_avl/USEC_PER_MSEC, usec_avl%USEC_PER_MSEC);
    }
}

/* ------------------------------------------------------------------------- */
/* shdisp_dbg_bdic_singl_write_start                                         */
/* ------------------------------------------------------------------------- */

static void shdisp_dbg_bdic_singl_write_start(void)
{
    if(shdisp_sys_dbg_api.flag == 0) {
        return;
    }
    
    getnstimeofday(&shdisp_sys_dbg_bdic.w_singl_t_start);
}

/* ------------------------------------------------------------------------- */
/* shdisp_dbg_bdic_singl_write_retry                                         */
/* ------------------------------------------------------------------------- */

static void shdisp_dbg_bdic_singl_write_retry(void)
{
    if(shdisp_sys_dbg_api.flag == 0) {
        return;
    }
    
    shdisp_sys_dbg_bdic.w_singl_retry++;
}

/* ------------------------------------------------------------------------- */
/* shdisp_dbg_bdic_singl_write_end                                           */
/* ------------------------------------------------------------------------- */

static void shdisp_dbg_bdic_singl_write_end(int ret)
{
    struct timespec stop, df;
    
    if(shdisp_sys_dbg_api.flag == 0) {
        return;
    }
    
    getnstimeofday(&stop);
    
    df = timespec_sub(stop, shdisp_sys_dbg_bdic.w_singl_t_start);
    
    shdisp_sys_dbg_bdic.w_singl_t_sum = timespec_add(shdisp_sys_dbg_bdic.w_singl_t_sum, df);
    
    if ( ret == 0 ) {
        shdisp_sys_dbg_bdic.w_singl_ok_count++;
    }
    else {
        shdisp_sys_dbg_bdic.w_singl_ng_count++;
    }
}

/* ------------------------------------------------------------------------- */
/* shdisp_dbg_bdic_singl_read_start                                          */
/* ------------------------------------------------------------------------- */

static void shdisp_dbg_bdic_singl_read_start(void)
{
    if(shdisp_sys_dbg_api.flag == 0) {
        return;
    }
    
    getnstimeofday(&shdisp_sys_dbg_bdic.r_singl_t_start);
}

/* ------------------------------------------------------------------------- */
/* shdisp_dbg_bdic_singl_read_retry                                          */
/* ------------------------------------------------------------------------- */

static void shdisp_dbg_bdic_singl_read_retry(void)
{
    if(shdisp_sys_dbg_api.flag == 0) {
        return;
    }
    
    shdisp_sys_dbg_bdic.r_singl_retry++;
}

/* ------------------------------------------------------------------------- */
/* shdisp_dbg_bdic_singl_read_end                                            */
/* ------------------------------------------------------------------------- */

static void shdisp_dbg_bdic_singl_read_end(int ret)
{
    struct timespec stop, df;
    
    if(shdisp_sys_dbg_api.flag == 0) {
        return;
    }
    
    getnstimeofday(&stop);
    
    df = timespec_sub(stop, shdisp_sys_dbg_bdic.r_singl_t_start);
    
    shdisp_sys_dbg_bdic.r_singl_t_sum = timespec_add(shdisp_sys_dbg_bdic.r_singl_t_sum, df);
    
    if ( ret == 0 ) {
        shdisp_sys_dbg_bdic.r_singl_ok_count++;
    }
    else {
        shdisp_sys_dbg_bdic.r_singl_ng_count++;
    }
}

/* ------------------------------------------------------------------------- */
/* shdisp_dbg_bdic_multi_read_start                                          */
/* ------------------------------------------------------------------------- */

static void shdisp_dbg_bdic_multi_read_start(void)
{
    if(shdisp_sys_dbg_api.flag == 0) {
        return;
    }
    
    getnstimeofday(&shdisp_sys_dbg_bdic.r_multi_t_start);
}

/* ------------------------------------------------------------------------- */
/* shdisp_dbg_bdic_multi_read_retry                                          */
/* ------------------------------------------------------------------------- */

static void shdisp_dbg_bdic_multi_read_retry(void)
{
    if(shdisp_sys_dbg_api.flag == 0) {
        return;
    }
    
    shdisp_sys_dbg_bdic.r_multi_retry++;
}

/* ------------------------------------------------------------------------- */
/* shdisp_dbg_bdic_multi_read_end                                            */
/* ------------------------------------------------------------------------- */

static void shdisp_dbg_bdic_multi_read_end(int ret)
{
    struct timespec stop, df;
    
    if(shdisp_sys_dbg_api.flag == 0) {
        return;
    }
    
    getnstimeofday(&stop);
    
    df = timespec_sub(stop, shdisp_sys_dbg_bdic.r_multi_t_start);
    
    shdisp_sys_dbg_bdic.r_multi_t_sum = timespec_add(shdisp_sys_dbg_bdic.r_multi_t_sum, df);
    
    if ( ret == 0 ) {
        shdisp_sys_dbg_bdic.r_multi_ok_count++;
    }
    else {
        shdisp_sys_dbg_bdic.r_multi_ng_count++;
    }
}
#endif /* SHDISP_SYS_SW_TIME_BDIC */
#endif /* SHDISP_SYS_SW_TIME_API */


MODULE_DESCRIPTION("SHARP DISPLAY DRIVER MODULE");
MODULE_LICENSE("GPL v2");
MODULE_AUTHOR("SHARP CORPORATION");
MODULE_VERSION("1.00");

/* ------------------------------------------------------------------------- */
/* END OF FILE                                                               */
/* ------------------------------------------------------------------------- */
