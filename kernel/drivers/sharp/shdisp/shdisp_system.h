/* drivers/sharp/shdisp/shdisp_system.h  (Display Driver)
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

#ifndef SHDISP_SYSTEM_H
#define SHDISP_SYSTEM_H

/* ------------------------------------------------------------------------- */
/* INCLUDE FILES                                                             */
/* ------------------------------------------------------------------------- */

#include <sharp/shdisp_kerl.h>

/* ------------------------------------------------------------------------- */
/* DEBUG MACRAOS                                                             */
/* ------------------------------------------------------------------------- */
//#define SHDISP_SYS_SW_TIME_API
//#define SHDISP_SYS_SW_TIME_BDIC

/* ------------------------------------------------------------------------- */
/* MACROS                                                                    */
/* ------------------------------------------------------------------------- */

#define SHDISP_GPIO_CTL_LOW         0
#define SHDISP_GPIO_CTL_HIGH        1


#define SHDISP_GPIO_NUM_BL_RST_N    6
#define SHDISP_GPIO_NUM_LCD_CLK     54
#define SHDISP_GPIO_NUM_SPI_MOSI    22
#define SHDISP_GPIO_NUM_SPI_MISO    23
#define SHDISP_GPIO_NUM_SPI_CS      24
#define SHDISP_GPIO_NUM_SPI_SCLK    25
#define SHDISP_GPIO_NUM_BL_I2C_SDA  73
#define SHDISP_GPIO_NUM_BL_I2C_SCL  74
#define SHDISP_GPIO_NUM_LCD_SCS_N   32
#define SHDISP_GPIO_NUM_LEDC_RST_N  36
#define SHDISP_GPIO_NUM_CLK_SEL     107


#define SHDISP_BDIC_I2C_DEVNAME     ("bdic_i2c")
#define SHDISP_NICOLE_I2C_DEVNAME   ("nicole_i2c")
#define SHDISP_TAKT_I2C_DEVNAME     ("takt_i2c")
#define SHDISP_LEDC_I2C_DEVNAME     ("ledc_i2c")

/* ------------------------------------------------------------------------- */
/* TYPES                                                                     */
/* ------------------------------------------------------------------------- */
#define SHDISP_IRQ_MAX_KIND         4


enum {
    SHDISP_HOST_CTL_CMD_LCD_CLK_START,
    SHDISP_HOST_CTL_CMD_LCD_CLK_STOP,
    SHDISP_HOST_CTL_CMD_LCD_CLK_INIT,
    SHDISP_HOST_CTL_CMD_LCD_CLK_EXIT,
    NUM_SHDISP_HOST_CTL_CMD
};

enum {
    SHDISP_IRQ_DISABLE,
    SHDISP_IRQ_ENABLE,
    NUM_SHDISP_IRQ_CMD
};

enum{
    SHDISP_HW_REV_ES0,
    SHDISP_HW_REV_ES05,
    SHDISP_HW_REV_ES1,
    SHDISP_HW_REV_ES15,
    SHDISP_HW_REV_PP1,
    SHDISP_HW_REV_PP15,
    SHDISP_HW_REV_PP2,
    SHDISP_HW_REV_PP2_B,
};

/* ------------------------------------------------------------------------- */
/* PROTOTYPES                                                                */
/* ------------------------------------------------------------------------- */

int  shdisp_SYS_Host_control(int cmd, unsigned long rate);
void shdisp_SYS_delay_us(unsigned long usec);
void shdisp_SYS_Host_gpio_init(void);
void shdisp_SYS_Host_gpio_exit(void);
int  shdisp_SYS_Host_gpio_request(int num);
int  shdisp_SYS_Host_gpio_free(int num);
int  shdisp_SYS_set_Host_gpio(int num, int value);

int  shdisp_SYS_request_irq(irqreturn_t (*irq_handler)( int , void * ) );
void shdisp_SYS_free_irq(void);
int  shdisp_SYS_set_irq( int enable );
void shdisp_sys_dbg_hw_check_start(void);
void shdisp_sys_dbg_hw_check_end(const char *func);

int  shdisp_SYS_bdic_i2c_init(void);
int  shdisp_SYS_bdic_i2c_exit(void);
int  shdisp_SYS_bdic_i2c_write(unsigned char addr, unsigned char data);
int  shdisp_SYS_bdic_i2c_read(unsigned char addr, unsigned char *data);
int  shdisp_SYS_bdic_i2c_multi_read(unsigned char addr, unsigned char *data, int size);
int  shdisp_SYS_pharaoh_sio_init(void);
int  shdisp_SYS_pharaoh_sio_exit(void);
void shdisp_SYS_pharaoh_sio_transfer(unsigned short reg, unsigned char *wbuf, int wlen, unsigned char *rbuf, int rlen);
int  shdisp_SYS_nicole_i2c_init(void);
int  shdisp_SYS_nicole_i2c_exit(void);
int  shdisp_SYS_nicole_i2c_write(unsigned char addr, unsigned char *data, int size);
int  shdisp_SYS_nicole_i2c_read(unsigned char addr, unsigned char *data, int size);
int  shdisp_SYS_takt_i2c_init(void);
int  shdisp_SYS_takt_i2c_exit(void);
int  shdisp_SYS_takt_i2c_write(unsigned char addr, unsigned char *data, int size);
int  shdisp_SYS_takt_i2c_read(unsigned char addr, unsigned char *data, int size);
int  shdisp_SYS_ledc_i2c_init(void);
int  shdisp_SYS_ledc_i2c_exit(void);
int  shdisp_SYS_ledc_i2c_write(unsigned char addr, unsigned char data);
int  shdisp_SYS_ledc_i2c_read(unsigned char addr, unsigned char *data);


#endif /* SHDISP_SYSTEM_H */

/* ------------------------------------------------------------------------- */
/* END OF FILE                                                               */
/* ------------------------------------------------------------------------- */
