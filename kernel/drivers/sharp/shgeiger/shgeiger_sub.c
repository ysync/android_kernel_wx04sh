/* driver/sharp/shgeiger/shgeiger_sub.c  (shgeiger sub Driver)
  *
  * Copyright (C) 2012 SHARP CORPORATION All rights reserved.
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
/* SHARP GEIGER SENSOR SUB DRIVER FOR KERNEL MODE                             */
/* ------------------------------------------------------------------------- */

/* ------------------------------------------------------------------------- */
/* INCLUDE FILES                                                             */
/* ------------------------------------------------------------------------- */

#include <linux/kernel.h>
#include <linux/i2c.h>
#include <linux/device.h>
#include <linux/delay.h>
#include <linux/module.h>

#include <sharp/shgeiger_sub.h>

/* ------------------------------------------------------------------------- */
/* MACRAOS                                                                   */
/* ------------------------------------------------------------------------- */

#define	I2C_RETRY			3

/* adb debug_log */
static int    shgeigersub_dbg_func_log = 0;      /* log : Init = OFF */
static int    shgeigersub_dbg_func_fin_log = 0;  /* log : Init = OFF */
static int    shgeigersub_dbg_error_log = 1;     /* log : Init = ON  */

#if defined (CONFIG_ANDROID_ENGINEERING)
    module_param(shgeigersub_dbg_func_log, int, 0600);
    module_param(shgeigersub_dbg_func_fin_log, int, 0600);
    module_param(shgeigersub_dbg_error_log, int, 0600);
#endif  /* CONFIG_ANDROID_ENGINEERING */

#define FUNC_LOG() \
    if(shgeigersub_dbg_func_log == 1){ \
       printk(KERN_DEBUG "[SHGEIGER_SUB] %s is called\n", __func__); \
    }

#define FUNC_FIN_LOG() \
    if(shgeigersub_dbg_func_fin_log == 1){ \
       printk(KERN_DEBUG "[SHGEIGER_SUB] %s is finished\n", __func__); \
    }

#define DEBUG_ERROR_LOG(format, ...) \
    if(shgeigersub_dbg_error_log == 1){ \
       printk(KERN_DEBUG "[SHGEIGER_SUB][%s] " format "\n", __func__, ## __VA_ARGS__); \
    }

/* ------------------------------------------------------------------------- */
/* VARIABLES                                                                 */
/* ------------------------------------------------------------------------- */
typedef struct i2c_client       I2cClt;
typedef struct i2c_device_id    I2cDevID;


typedef struct
{
	uint8_t	mbRegAdr;					/* Register */
	uint8_t mbData;						/* Data */
} I2cWriteData;

static I2cClt		*this_client;

/* ------------------------------------------------------------------------- */
/* PROTOTYPES                                                                */
/* ------------------------------------------------------------------------- */

int SHGEIGER_EEPROM_I2cRead(char *rxData, int length);
static int SHGEIGERSUB_Remove(I2cClt *client);
static int SHGEIGERSUB_Probe(I2cClt *client, const I2cDevID *poDevId);
static int __init SHGEIGERSUB_Init(void);
static void __exit SHGEIGERSUB_Exit(void);


int SHGEIGER_EEPROM_I2cRead(char *rxData, int length)
{
	uint8_t loop_i;
	struct i2c_msg msgs[] = {
		{
			.addr = this_client->addr,
			.flags = 0,
			.len = 1,
			.buf = rxData,
		},
		{
			.addr = this_client->addr,
			.flags = I2C_M_RD,
			.len = length,
			.buf = rxData,
		},
	};

	FUNC_LOG();

	for (loop_i = 0; loop_i < I2C_RETRY; loop_i++) {
		if (i2c_transfer(this_client->adapter, msgs, 2) > 0) {
			break;
		}
		mdelay(10);
	}

	if (loop_i >= I2C_RETRY) {
		DEBUG_ERROR_LOG("I2cRead: error\n");
		return SH_GEIGERSUB_RESULT_FAILURE;
	}

	FUNC_FIN_LOG();

	return SH_GEIGERSUB_RESULT_SUCCESS;
}

static int SHGEIGERSUB_Remove(I2cClt *client)
{

	FUNC_LOG();


	FUNC_FIN_LOG();

	return 0;	
}

static const struct i2c_device_id gI2cDevIdTableAcc[] =
{
   { SH_GEIGERSUB_I2C_DEVNAME, 0 },
   { }
};

MODULE_DEVICE_TABLE(i2c, gI2cDevIdTableAcc);

static struct i2c_driver goI2cAccDriver =
{
	.driver =
	{
		.owner = THIS_MODULE,
		.name  = SH_GEIGERSUB_I2C_DEVNAME,
	},
	.probe	  = SHGEIGERSUB_Probe,
	.remove	  = SHGEIGERSUB_Remove,
	.id_table = gI2cDevIdTableAcc,
};

static int SHGEIGERSUB_Probe(I2cClt *client, const I2cDevID *poDevId)
{
	int nResult;

	FUNC_LOG();

	/* CLIENT CHECKING */
	if (!i2c_check_functionality(client->adapter, I2C_FUNC_I2C)) {
		DEBUG_ERROR_LOG("check_functionality failed.");
		nResult = -ENODEV;
		goto exit;
	}

	/* Copy to global variable */
	this_client = client;

	FUNC_FIN_LOG();
	return SH_GEIGERSUB_RESULT_SUCCESS;
	
exit:
	return nResult;

}

static int __init SHGEIGERSUB_Init(void)
{
	FUNC_LOG();

	/* I2C driver Use beginning */
	return i2c_add_driver(&goI2cAccDriver);

	FUNC_FIN_LOG();
	return SH_GEIGERSUB_RESULT_SUCCESS;
}

static void __exit SHGEIGERSUB_Exit(void)
{
	FUNC_LOG();

	/* I2C driver Use end */
	i2c_del_driver(&goI2cAccDriver);
	
	FUNC_FIN_LOG();
}


module_init(SHGEIGERSUB_Init);
module_exit(SHGEIGERSUB_Exit);


MODULE_DESCRIPTION("shgeiger sensor sub driver");
MODULE_LICENSE("GPL v2");
MODULE_AUTHOR("SHARP CORPORATION");
MODULE_VERSION("1.0");



