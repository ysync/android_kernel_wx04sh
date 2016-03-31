/*
  *Copyright (C) 2010 SHARP CORPORATION All rights reserved.
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

/*
 * SHARP PROXIMITY DRIVER(Y2659)
*/

#include <linux/kernel.h>
#include <linux/interrupt.h>
#include <linux/miscdevice.h>
#include <linux/input.h>
#include <linux/delay.h>
#include <linux/poll.h>
#include <mach/gpio.h>
#include <linux/i2c.h>
#include <linux/irq.h>
#include <linux/device.h>
#include <sharp/proximity.h>
#include <linux/hrtimer.h>
#include <linux/slab.h>
#include <linux/time.h>
#include <linux/regulator/consumer.h>
#include <linux/wakelock.h>
#include <linux/module.h>
#include <mach/vreg.h>
#include <sharp/sh_smem.h>

#include<sharp/shdisp_kerl.h>

#define DEBUG 0 
#define PROX_DEBUG_MSG				0
#define PROX_DEBUG_FUNC				0
#define PROX_DEBUG_FUNC_FIN			0
#define PROX_DEBUG_SENSOR			0

#define HW_ES0		0x00
#define HW_ES1		0x05
#define HW_ES15		0x01
#define HW_PP1		0x06
#define HW_PP15		0x03	//HW_PP16
#define HW_PP2		0x07
#define HW_PP2_B	0x02	//type_BO

#define MODEL_TYPE_A1		0x01
#define MODEL_TYPE_S1		0x02
#define MODEL_TYPE_D1		0x03
#define MODEL_TYPE_D2		0x04
#define MODEL_TYPE_D3		0x05

/* adb debug_log */
static int	proximity_dbg_func_log = 0;		/* log : Init = OFF */
static int	proximity_dbg_func_fin_log = 0;	/* log : Init = OFF */
static int	proximity_dbg_enable_log = 0;	/* log : Init = OFF */
static int	proximity_dbg_sensor_log = 0;	/* log : Init = OFF */
static int	proximity_dbg_error_log = 1;	/* log : Init = ON  */

#if defined (CONFIG_ANDROID_ENGINEERING)
	module_param(proximity_dbg_func_log, int, 0600);
	module_param(proximity_dbg_func_fin_log, int, 0600);
	module_param(proximity_dbg_enable_log, int, 0600);
	module_param(proximity_dbg_sensor_log, int, 0600);
	module_param(proximity_dbg_error_log, int, 0600);
#endif  /* CONFIG_ANDROID_ENGINEERING */

#define FUNC_LOG() \
	if(proximity_dbg_func_log == 1){ \
		printk(KERN_DEBUG "[PROXIMITY] %s is called\n", __func__); \
	}

#define FUNC_FIN_LOG() \
	if(proximity_dbg_func_fin_log == 1){ \
		printk(KERN_DEBUG "[PROXIMITY] %s is finished\n", __func__); \
	}

#define DEBUG_LOG(format, ...) \
	if(proximity_dbg_enable_log == 1){ \
		printk(KERN_DEBUG "[PROXIMITY][%s] " format "\n", __func__, ## __VA_ARGS__); \
	}

#define DEBUG_SENSOR_LOG(format, ...) \
	if(proximity_dbg_sensor_log == 1){ \
		printk(KERN_DEBUG "[PROXIMITY][%s] " format "\n", __func__, ## __VA_ARGS__); \
	}

#define DEBUG_ERROR_LOG(format, ...) \
	if(proximity_dbg_error_log == 1){ \
		printk(KERN_DEBUG "[PROXIMITY][%s] " format "\n", __func__, ## __VA_ARGS__); \
	}

/*+-------------------------------------------------------------------------+*/
/*|																			|*/
/*+-------------------------------------------------------------------------+*/
typedef struct drv_data_tag     drv_data;
typedef struct work_struct      WorkStruct;
typedef struct input_dev        InputDev;
typedef struct device           Device;

static int Threshold_Low;                            
static int Threshold_High;                            

struct drv_data_tag
{
	int			irq_gpio;
	InputDev	*input_dev;
	WorkStruct	IrqWork;
	int 		irq;
};

static drv_data 	*poProximityRec;
//static char			MVO = 0;
static atomic_t		open_flag = ATOMIC_INIT(0);
static atomic_t		sensor_data = ATOMIC_INIT(7);	/* Init = Far */
static atomic_t		enable_mode = ATOMIC_INIT(0);	/* 0=Disable,1=Enable */
static atomic_t     dataread_func_flag = ATOMIC_INIT(0);  /* 0=Disable,1=Enable */

//static struct regulator *vreg_vl8;

static int PROX_Probe( void );
static void PROX_Irq_workfunc( void );

static struct wake_lock prox_timeout_wake_lock;
static struct wake_lock prox_wake_lock;

static struct mutex prox_enable_lock;

static uint16_t sh_get_hw_revision(void)
{
	sharp_smem_common_type *p_sharp_smem_common_type;

	p_sharp_smem_common_type = sh_smem_get_common_address();
	if( p_sharp_smem_common_type != 0 )
	{
		return p_sharp_smem_common_type->sh_hw_revision;
	}else{
		return 0xFF;
	}
}

static void prox_subscribe( void )
{
	int nResult;
	struct shdisp_subscribe prox_subscribe;

	FUNC_LOG();

	prox_subscribe.irq_type = SHDISP_IRQ_TYPE_PS;
	prox_subscribe.callback = PROX_Irq_workfunc;
	
	nResult = shdisp_api_event_subscribe(&prox_subscribe);
	if(nResult != SHDISP_RESULT_SUCCESS){
		DEBUG_LOG("Proximity_subscribe Error");
	}
}

static void prox_unsubscribe( void )
{
	int nResult;

	FUNC_LOG();

	nResult = shdisp_api_event_unsubscribe(SHDISP_IRQ_TYPE_PS);
	if(nResult != SHDISP_RESULT_SUCCESS){
		DEBUG_LOG("Proximity_unsubscribe Error");
	}
}
/*		
static int PROX_I2cWrite(unsigned char RegAdr, unsigned char wData)
{
	int nResult = 0;
	struct shdisp_bdic_i2c_msg i2c_msg;
	unsigned char i2c_wbuf[2];
	
	i2c_wbuf[0] = RegAdr;
	i2c_wbuf[1] = wData;
	i2c_msg.addr = 0x39;
	i2c_msg.mode = SHDISP_BDIC_I2C_M_W;
	i2c_msg.wlen = 2;
	i2c_msg.rlen = 0;
	i2c_msg.wbuf = &i2c_wbuf[0];
	i2c_msg.rbuf = NULL;
	
	nResult = shdisp_api_write_bdic_i2c(&i2c_msg);
	if(nResult != SHDISP_RESULT_SUCCESS){
		DEBUG_LOG("I2C_WriteError id = %d",nResult);
	}

	DEBUG_LOG("I2cWrite(reg:%02X,Data:%02X)", i2c_wbuf[0], i2c_wbuf[1]);

	return nResult;

}
*/
static int PROX_I2cRead(unsigned char RegAdr,unsigned char *rData)
{
	int nResult = 0;
	int nRetry = 0;
	struct shdisp_bdic_i2c_msg i2c_msg;
	unsigned char i2c_wbuf[1];
	unsigned char i2c_rbuf[1];

	i2c_wbuf[0] = RegAdr;
	i2c_rbuf[0] = 0x00;
	i2c_msg.addr = 0x39;
	i2c_msg.mode = SHDISP_BDIC_I2C_M_R;
	i2c_msg.wlen = 1;
	i2c_msg.rlen = 1;
	i2c_msg.wbuf = &i2c_wbuf[0];
	i2c_msg.rbuf = &i2c_rbuf[0];
	
	for(nRetry=0;nRetry<5;nRetry++) {
			nResult = shdisp_api_read_bdic_i2c(&i2c_msg);
			if(nResult == SHDISP_RESULT_SUCCESS) {
				DEBUG_LOG("I2cRead : nResult = %d", nResult);
				break;
        	}
        	DEBUG_LOG("I2cReadError retry : nResult = %d", nResult);
        	if (nRetry < 4) msleep(20);
	}

   	if(nResult != SHDISP_RESULT_SUCCESS) {
			DEBUG_LOG("I2C_ReadError id = %d  retry = %d", nResult, nRetry);
			return nResult;
    }

	DEBUG_LOG("I2cRead(reg:%02X,Data:%02X)", i2c_wbuf[0], i2c_rbuf[0]);

	*rData = i2c_rbuf[0];

	return nResult;
	
}

#if CONFIG_PROXIMITY_MODEL_TYPE == MODEL_TYPE_A1 
static int IOECS_Enable(void)
{
	int nResult = -1;
	uint16_t rev;
	unsigned short proxadj[2];
	sharp_smem_common_type *p_sh_smem_common_type = NULL;

	struct shdisp_prox_params prox_params;

    mutex_lock(&prox_enable_lock);

	FUNC_LOG();

	if(atomic_read(&enable_mode) == 0)
	{
	  if(atomic_read(&dataread_func_flag) != 1){
		rev = sh_get_hw_revision();
		rev = rev & 0x07;
		DEBUG_LOG("hw_revision = %02X",rev);

		prox_subscribe();

		if(rev == HW_PP2){
			memset((void*)proxadj, 0x00, sizeof(proxadj));
			p_sh_smem_common_type = sh_smem_get_common_address();
			memcpy(proxadj, p_sh_smem_common_type->shdiag_proxadj, sizeof(proxadj));
			prox_params.threshold_low = proxadj[0];
			prox_params.threshold_high = proxadj[1];
		}
		else{
			prox_params.threshold_low = Threshold_Low;          
			prox_params.threshold_high = Threshold_High;            
		}
		
		nResult = shdisp_api_prox_sensor_pow_ctl(SHDISP_PROX_SENSOR_POWER_ON, &prox_params);
		if(nResult != SHDISP_RESULT_SUCCESS){
			DEBUG_LOG("I2C PowerON Error");
			nResult = -1;
			mutex_unlock(&prox_enable_lock);
			return nResult;
		}
	  atomic_set(&enable_mode, 1);
	  msleep(150);
	  }
	}
    mutex_unlock(&prox_enable_lock);
	return nResult;
}

#elif CONFIG_PROXIMITY_MODEL_TYPE == MODEL_TYPE_S1
static int IOECS_Enable(void)
{
	int nResult = -1;
	uint16_t rev;

	struct shdisp_prox_params prox_params;

    mutex_lock(&prox_enable_lock);

	FUNC_LOG();

	if(atomic_read(&enable_mode) == 0)
	{
	  if(atomic_read(&dataread_func_flag) != 1){
		rev = sh_get_hw_revision();
		rev = rev & 0x07;
		DEBUG_LOG("hw_revision = %02X",rev);

		prox_subscribe();

        prox_params.threshold_low = Threshold_Low;          
        prox_params.threshold_high = Threshold_High;         
		nResult = shdisp_api_prox_sensor_pow_ctl(SHDISP_PROX_SENSOR_POWER_ON, &prox_params);
		if(nResult != SHDISP_RESULT_SUCCESS){
			DEBUG_LOG("I2C PowerON Error");
			nResult = -1;
			mutex_unlock(&prox_enable_lock);
			return nResult;
		}
	  }
	  atomic_set(&enable_mode, 1);
	  msleep(150);
	}
    mutex_unlock(&prox_enable_lock);
	return nResult;
}

#elif CONFIG_PROXIMITY_MODEL_TYPE == MODEL_TYPE_D1
static int IOECS_Enable(void)
{
	int nResult = -1;
	uint16_t rev;
	unsigned short proxadj[2];
	sharp_smem_common_type *p_sh_smem_common_type = NULL;

	struct shdisp_prox_params prox_params;

    mutex_lock(&prox_enable_lock);

	FUNC_LOG();

	if(atomic_read(&enable_mode) == 0)
	{
	  if(atomic_read(&dataread_func_flag) != 1){
		rev = sh_get_hw_revision();
		rev = rev & 0x07;
		DEBUG_LOG("hw_revision = %02X",rev);

		prox_subscribe();

		if(rev == HW_PP2){
			memset((void*)proxadj, 0x00, sizeof(proxadj));
			p_sh_smem_common_type = sh_smem_get_common_address();
			memcpy(proxadj, p_sh_smem_common_type->shdiag_proxadj, sizeof(proxadj));
			prox_params.threshold_low = proxadj[0];
			prox_params.threshold_high = proxadj[1];
		}
		else{
			prox_params.threshold_low = Threshold_Low;          
			prox_params.threshold_high = Threshold_High;            
		}
		
		nResult = shdisp_api_prox_sensor_pow_ctl(SHDISP_PROX_SENSOR_POWER_ON, &prox_params);
		if(nResult != SHDISP_RESULT_SUCCESS){
			DEBUG_LOG("I2C PowerON Error");
			nResult = -1;
			mutex_unlock(&prox_enable_lock);
			return nResult;
		}
	  atomic_set(&enable_mode, 1);
	  msleep(150);
	  }
	}
    mutex_unlock(&prox_enable_lock);
	return nResult;
}

#elif CONFIG_PROXIMITY_MODEL_TYPE == MODEL_TYPE_D2
static int IOECS_Enable(void)
{
	int nResult = -1;
	uint16_t rev;
	unsigned short proxadj[2];
	sharp_smem_common_type *p_sh_smem_common_type = NULL;

	struct shdisp_prox_params prox_params;

    mutex_lock(&prox_enable_lock);

	FUNC_LOG();

	if(atomic_read(&enable_mode) == 0)
	{
	  if(atomic_read(&dataread_func_flag) != 1){
		rev = sh_get_hw_revision();
		rev = rev & 0x07;
		DEBUG_LOG("hw_revision = %02X",rev);

		prox_subscribe();

		if(rev != HW_ES0){
			memset((void*)proxadj, 0x00, sizeof(proxadj));
			p_sh_smem_common_type = sh_smem_get_common_address();
			memcpy(proxadj, p_sh_smem_common_type->shdiag_proxadj, sizeof(proxadj));
			prox_params.threshold_low = proxadj[0];
			prox_params.threshold_high = proxadj[1];
		}
		else{
			prox_params.threshold_low = Threshold_Low;          
			prox_params.threshold_high = Threshold_High;            
		}
		
		nResult = shdisp_api_prox_sensor_pow_ctl(SHDISP_PROX_SENSOR_POWER_ON, &prox_params);
		if(nResult != SHDISP_RESULT_SUCCESS){
			DEBUG_LOG("I2C PowerON Error");
			nResult = -1;
			mutex_unlock(&prox_enable_lock);
			return nResult;
		}
	  atomic_set(&enable_mode, 1);
	  msleep(150);
	  }
	}
	mutex_unlock(&prox_enable_lock);
	return nResult;
}

#elif CONFIG_PROXIMITY_MODEL_TYPE == MODEL_TYPE_D3
static int IOECS_Enable(void)
{
	int nResult = -1;
	uint16_t rev;
	unsigned short proxadj[2];
	sharp_smem_common_type *p_sh_smem_common_type = NULL;

	struct shdisp_prox_params prox_params;

    mutex_lock(&prox_enable_lock);

	FUNC_LOG();

	if(atomic_read(&enable_mode) == 0)
	{
	  if(atomic_read(&dataread_func_flag) != 1){
		rev = sh_get_hw_revision();
		rev = rev & 0x07;
		DEBUG_LOG("hw_revision = %02X",rev);

		prox_subscribe();

		if(rev == HW_PP2){
			memset((void*)proxadj, 0x00, sizeof(proxadj));
			p_sh_smem_common_type = sh_smem_get_common_address();
			memcpy(proxadj, p_sh_smem_common_type->shdiag_proxadj, sizeof(proxadj));
			prox_params.threshold_low = proxadj[0];
			prox_params.threshold_high = proxadj[1];
		}
		else{
			prox_params.threshold_low = Threshold_Low;          
			prox_params.threshold_high = Threshold_High;            
		}
		
		nResult = shdisp_api_prox_sensor_pow_ctl(SHDISP_PROX_SENSOR_POWER_ON, &prox_params);
		if(nResult != SHDISP_RESULT_SUCCESS){
			DEBUG_LOG("I2C PowerON Error");
			nResult = -1;
			mutex_unlock(&prox_enable_lock);
			return nResult;
		}
	  atomic_set(&enable_mode, 1);
	  msleep(150);
	  }
	}
    mutex_unlock(&prox_enable_lock);
	return nResult;
}

#else
static int IOECS_Enable(void)
{
	int nResult = -1;
	uint16_t rev;
        unsigned char rData = 0x00;
	unsigned short proxadj[2];
	sharp_smem_common_type *p_sh_smem_common_type = NULL;

	struct shdisp_prox_params prox_params;

    mutex_lock(&prox_enable_lock);

	FUNC_LOG();

	if(atomic_read(&enable_mode) == 0)
	{
	  if(atomic_read(&dataread_func_flag) != 1){
		rev = sh_get_hw_revision();
		rev = rev & 0x07;
		DEBUG_LOG("hw_revision = %02X",rev);

		prox_subscribe();

		memset((void*)proxadj, 0x00, sizeof(proxadj));
		p_sh_smem_common_type = sh_smem_get_common_address();
		memcpy(proxadj, p_sh_smem_common_type->shdiag_proxadj, sizeof(proxadj));
		prox_params.threshold_low = proxadj[0];
		prox_params.threshold_high = proxadj[1];
		
		nResult = shdisp_api_prox_sensor_pow_ctl(SHDISP_PROX_SENSOR_POWER_ON, &prox_params);
		if(nResult != SHDISP_RESULT_SUCCESS){
			DEBUG_LOG("I2C PowerON Error");
			nResult = -1;
			mutex_unlock(&prox_enable_lock);
			return nResult;
		}
	  }
	  nResult = PROX_I2cRead(GP2AP030_REG_COMMAND1, &rData);
	  if(nResult != 0){
		DEBUG_LOG("I2CRead_Error");
	  }
	  rData = (rData & 0x08);

	  if(0x08 == rData){
		atomic_set(&sensor_data, 0);
	  }else {
		atomic_set(&sensor_data, 7);
	  }
	  atomic_set(&enable_mode, 1);
	}
	mutex_unlock(&prox_enable_lock);
	return nResult;
}
#endif

static int IOECS_Disable(void)
{
	int nResult = -1;
	uint16_t rev;
	
    mutex_lock(&prox_enable_lock);

	FUNC_LOG();

	if(atomic_read(&enable_mode) == 1)
	{
		rev = sh_get_hw_revision();
		rev = rev & 0x07;

	//	switch(rev){
	//		case HW_ES0:
	//		case HW_ES1:
	//		case HW_ES15:
	//			regulator_disable(vreg_vl8);
	//			DEBUG_LOG("vl8 off");
	//			break;
	//		default:
	//			break;
	//	}

		prox_unsubscribe();

        if(atomic_read(&dataread_func_flag) != 1){
			nResult = shdisp_api_prox_sensor_pow_ctl(SHDISP_PROX_SENSOR_POWER_OFF, NULL);
			if(nResult != SHDISP_RESULT_SUCCESS){
				DEBUG_LOG("I2C PowerOFF Error");
				nResult = -1;
				mutex_unlock(&prox_enable_lock);
				return nResult;
			}
		}
		atomic_set(&enable_mode, 0);
	}
    mutex_unlock(&prox_enable_lock);
	return nResult;
}

static int IOECS_SetCycle(short cycle_data)
{
	FUNC_LOG();


	return 0;
}

static int IOECS_GetVO_DATA(uint8_t *cycle_data)
{
//	unsigned char rData = 0x00;

	FUNC_LOG();

//	PROX_I2cRead(GP2AP030_REG_D2_LSB,&rData);
//	PROX_I2cRead(GP2AP030_REG_D2_MSB,&rData);

	return 0;
}


static int IOECS_GetD2_DATA(unsigned short *psresult)
{
	int nResult = 0;
	unsigned char lData = 0x00;
	unsigned short LData = 0x00;
	unsigned char mData = 0x00;
	unsigned short MData = 0x00;
	FUNC_LOG();
//	DEBUG_LOG("GetVO_DATA_MUX_Down_start");
//	PROX_MutexDown();
//	DEBUG_LOG("GetVO_DATA_MUX_Down_end");

	if(atomic_read(&enable_mode) == 1)
	{
		nResult = PROX_I2cRead(GP2AP030_REG_D2_LSB,&lData);
		if(nResult != 0){
			DEBUG_LOG("I2CRead_Error");
		}
		LData = ( lData & 0x00FF );
		nResult = PROX_I2cRead(GP2AP030_REG_D2_MSB,&mData);
		if(nResult != 0){
			DEBUG_LOG("I2CRead_Error");
		}
		MData = ( mData & 0x00FF );
		
		*psresult = (( MData<<8 ) | LData );
		DEBUG_LOG("psresult = 0x%04x",*psresult);
		
	}

//	DEBUG_LOG("GetVO_DATA_MUX_UP_start");
//	PROX_MutexUP();
//	DEBUG_LOG("GetVO_DATA_MUX_UP_end");
	
	return 0;
}

static int IOECS_LT_Thresshold_Write(unsigned short lt_threshold)
{
	FUNC_LOG();
	
	Threshold_Low = lt_threshold;
	
	return 0;
}

static int IOECS_HT_Thresshold_Write(unsigned short ht_threshold)
{
	FUNC_LOG();
	
	Threshold_High = ht_threshold;
	
	return 0;
}

static int PROX_open(struct inode *inode, struct file *filp)
{
	int nResult = -1;

	FUNC_LOG();

	if (atomic_cmpxchg(&open_flag, 0, 1) == 0)
	{
		/* Init = Far */
		atomic_set(&sensor_data, 7);
		nResult = 0;
	}

	return nResult;
}

static int PROX_release(struct inode *inode, struct file *filp)
{
	FUNC_LOG();

    if(atomic_read(&dataread_func_flag) != 1){
		IOECS_Disable();
	}

	atomic_set(&open_flag, 0);

	wake_unlock(&prox_timeout_wake_lock);

	return 0;
}

static int PROX_read(struct file *filp, char __user *buf,
			   size_t count, loff_t *ppos)

{
	char tmp;

	tmp = (char)atomic_read(&sensor_data);

#if PROX_DEBUG_SENSOR == 1	
	DEBUG_LOG("PROX_read_SENSOR_DATA = %x\n",tmp);
#endif	

	if (copy_to_user(buf, &tmp, sizeof(tmp))) {
		return -EFAULT;
	}

	return 0;
}

static long PROX_ioctl(struct file *filp,
			unsigned int cmd, unsigned long arg)
{
	void __user *argp = (void __user *)arg;
	uint8_t bData;
	short cycle;
	unsigned short lt_threshold;
	unsigned short ht_threshold;
	unsigned short psresult;

	FUNC_LOG();

	switch (cmd) {
		case ECS_IOCTL_SET_CYCLE:
			DEBUG_LOG("ECS_IOCTL_SET_CYCLE");

			if (copy_from_user(&cycle, argp, sizeof(cycle))) {
				DEBUG_LOG("ECS_IOCTL_SET_CYCLE ERR");
				return -EFAULT;
			}
			break;
		case ECS_IOCTL_LT_THRESHOLD_WRITE:
			DEBUG_LOG("ECS_IOCTL_LT_THRESHOLD_WRITE");

			if (copy_from_user(&lt_threshold, argp, sizeof(lt_threshold))) {
				DEBUG_LOG("ECS_IOCTL_LT_THRESHOLD_WRITE ERR");
				return -EFAULT;
			}
			break;
		case ECS_IOCTL_HT_THRESHOLD_WRITE:
			DEBUG_LOG("ECS_IOCTL_HT_THRESHOLD_WRITE");

			if (copy_from_user(&ht_threshold, argp, sizeof(ht_threshold))) {
				DEBUG_LOG("ECS_IOCTL_HT_THRESHOLD_WRITE ERR");
				return -EFAULT;
			}
			break;
		case ECS_IOCTL_GET_D2_DATA:
			DEBUG_LOG("ECS_IOCTL_GET_D2_DATA");

			if (copy_from_user(&psresult, argp, sizeof(psresult))) {
				DEBUG_LOG("ECS_IOCTL_GET_D2_DATA ERR");
				return -EFAULT;
			}
			break;

		default:
			break;
	}

	switch (cmd) {
		case ECS_IOCTL_ENABLE:
			DEBUG_LOG("ECS_IOCTL_ENABLE");
			if(IOECS_Enable() < 0)
			{
				return -EIO;
			}
			break;
		case ECS_IOCTL_DISABLE:
			DEBUG_LOG("ECS_IOCTL_DISABLE");
			if(IOECS_Disable() < 0)
			{
				return -EIO;
			}
			break;
		case ECS_IOCTL_SET_CYCLE:
			DEBUG_LOG("ECS_IOCTL_SET_CYCLE");
			if(IOECS_SetCycle(cycle) < 0)
			{
				return -EIO;
			}
			break;
		case ECS_IOCTL_GET_VO_DATA:
			DEBUG_LOG("ECS_IOCTL_GET_VO_DATA");
			if(IOECS_GetVO_DATA(&bData) < 0)
			{
				return -EIO;
			}
			break;
		case ECS_IOCTL_GET_D2_DATA:
			DEBUG_LOG("ECS_IOCTL_GET_D2_DATA");
			if(IOECS_GetD2_DATA(&psresult) < 0)
			{
				return -EIO;
			}
			break;
		case ECS_IOCTL_LT_THRESHOLD_WRITE:
			DEBUG_LOG("ECS_IOCTL_LT_THRESHOLD_WRITE");
			if(IOECS_LT_Thresshold_Write(lt_threshold) < 0)
			{
				return -EIO;
			}
			break;
		case ECS_IOCTL_HT_THRESHOLD_WRITE:
			DEBUG_LOG("ECS_IOCTL_HT_THRESHOLD_WRITE");
			if(IOECS_HT_Thresshold_Write(ht_threshold) < 0)
			{
				return -EIO;
			}
			break;

		default:
			break;
	}

	switch (cmd) {
		case ECS_IOCTL_GET_VO_DATA:
			if (copy_to_user(argp, &bData, sizeof(bData))) {
				return -EFAULT;
			}
			break;
		case ECS_IOCTL_GET_D2_DATA:
			if (copy_to_user(argp, &psresult, sizeof(psresult))) {
				return -EFAULT;
			}
			break;
		default:
			break;
	}

	return 0;
}

static struct file_operations PROX_ctl_fops = {
	.owner		= THIS_MODULE,
	.open		= PROX_open,
	.release	= PROX_release,
	.unlocked_ioctl		= PROX_ioctl,
	.read		= PROX_read,
};



static struct miscdevice PROX_device = {
 .minor = MISC_DYNAMIC_MINOR,
	.name = "proximity_dev",
	.fops = &PROX_ctl_fops,
};

static void PROX_Irq_workfunc( void )
{
	int nResult = 0;
	unsigned char rData = 0x00;
	
	wake_lock(&prox_wake_lock);
	FUNC_LOG();

	nResult = PROX_I2cRead(GP2AP030_REG_COMMAND1, &rData);
	if(nResult != 0){
		DEBUG_LOG("I2CRead_Error");
	}
	rData = (rData & 0x08);

	if(0x08 == rData){
		atomic_set(&sensor_data, 0);
	}else {
		atomic_set(&sensor_data, 7);
	}

#if PROX_DEBUG_SENSOR == 1	
	DEBUG_LOG("PROX_Irq_workfunc_SENSOR_DATA = %x\n",atomic_read(&sensor_data));
#endif

	if(atomic_read(&sensor_data) == 0x07){
		wake_lock_timeout(&prox_timeout_wake_lock, 1 * HZ);
	}
	
	input_report_abs(poProximityRec->input_dev, ABS_DISTANCE, atomic_read(&sensor_data));
	input_sync(poProximityRec->input_dev);

#if PROX_DEBUG_MSG == 1
	/*Debug*/
	PROX_I2cRead(GP2AP030_REG_D2_LSB,&rData);
	PROX_I2cRead(GP2AP030_REG_D2_MSB,&rData);
	PROX_I2cRead(GP2AP030_REG_LT_LSB,&rData);
	PROX_I2cRead(GP2AP030_REG_LT_MSB,&rData);
	PROX_I2cRead(GP2AP030_REG_HT_LSB,&rData);
	PROX_I2cRead(GP2AP030_REG_HT_MSB,&rData);
	/*Debug End*/
#endif
	FUNC_FIN_LOG();
	wake_unlock(&prox_wake_lock);
}

/* ------------------------------------------------------------------------- */
/* PROX_dataread_func                                                        */
/* ------------------------------------------------------------------------- */
int PROX_dataread_func(int *read_data)
{
    int nResult = SH_PROXIMITY_RESULT_FAILURE;
    struct shdisp_prox_params prox_params;
    unsigned short proxadj[2];
    sharp_smem_common_type *p_sh_smem_common_type = NULL;
    unsigned char rData = 0x00;

    FUNC_LOG();

    mutex_lock(&prox_enable_lock);

	if(atomic_read(&enable_mode) != 1){

        memset((void*)proxadj, 0x00, sizeof(proxadj));
        p_sh_smem_common_type = sh_smem_get_common_address();

    #ifdef PROX_USE_SMEM
        if (p_sh_smem_common_type != NULL) {
            memcpy(proxadj, p_sh_smem_common_type->shdiag_proxadj, sizeof(proxadj));
            prox_params.threshold_low = proxadj[0];
            prox_params.threshold_high = proxadj[1];
            DEBUG_LOG("[%s][smem] LT:0x%04x, HT:0x%04x.\n", __func__, prox_params.threshold_low, prox_params.threshold_high);
        } else {
            prox_params.threshold_low = Threshold_Low;
            prox_params.threshold_high = Threshold_High;
            DEBUG_LOG("[%s][local] LT:0x%04x, HT:0x%04x.\n", __func__, prox_params.threshold_low, prox_params.threshold_high);
        }
    #else   /* PROX_USE_SMEM */
            prox_params.threshold_low = Threshold_Low;
            prox_params.threshold_high = Threshold_High;
            DEBUG_LOG("[%s][local] LT:0x%04x, HT:0x%04x.\n", __func__, prox_params.threshold_low, prox_params.threshold_high);
    #endif  /* PROX_USE_SMEM */

        nResult = shdisp_api_prox_sensor_pow_ctl(SHDISP_PROX_SENSOR_POWER_ON, &prox_params);
        if(nResult != SHDISP_RESULT_SUCCESS) {
            DEBUG_ERROR_LOG("PowerON Error");
            nResult = SH_PROXIMITY_RESULT_FAILURE;
            *read_data = SH_PROXIMITY_FAR;
            mutex_unlock(&prox_enable_lock);
            return nResult;
        }

        atomic_set(&dataread_func_flag, 1);
	}

	nResult = PROX_I2cRead(GP2AP030_REG_COMMAND1, &rData);
	if(nResult != 0){
		*read_data = SH_PROXIMITY_FAR;
		DEBUG_LOG("I2CRead_Error");
	}else{
		rData = (rData & 0x08);
		if(0x08 == rData){
			*read_data = SH_PROXIMITY_NEAR;
		}else {
			*read_data = SH_PROXIMITY_FAR;
		}
	}

    DEBUG_SENSOR_LOG("PROX_dataread_func = %d",*read_data);

    mutex_unlock(&prox_enable_lock);

    mutex_lock(&prox_enable_lock);

    if(atomic_read(&enable_mode) != 1){

        nResult = shdisp_api_prox_sensor_pow_ctl(SHDISP_PROX_SENSOR_POWER_OFF, NULL);
        if(nResult != SHDISP_RESULT_SUCCESS) {
            DEBUG_ERROR_LOG("PowerOFF Error");
            nResult = SH_PROXIMITY_RESULT_FAILURE;
            atomic_set(&dataread_func_flag, 0);
            mutex_unlock(&prox_enable_lock);
            return nResult;
        }
    }
    atomic_set(&dataread_func_flag, 0);
    mutex_unlock(&prox_enable_lock);

    FUNC_FIN_LOG();
    
    return SH_PROXIMITY_RESULT_SUCCESS;
}

int PROX_dataread_disable_func(void)
{
	FUNC_LOG();

	FUNC_FIN_LOG();
	return SH_PROXIMITY_RESULT_SUCCESS;
}

/* ------------------------------------------------------------------------- */
/* PROX_stateread_func                                                       */
/* ------------------------------------------------------------------------- */
int PROX_stateread_func(int *state_data, int *read_data)
{
	int nResult = SH_PROXIMITY_RESULT_FAILURE;
    	unsigned char rData = 0x00;

	mutex_lock(&prox_enable_lock);

	FUNC_LOG();

	if (atomic_read(&enable_mode) == 0) {
		DEBUG_LOG("Proximity Sensor Disable");
		*state_data = SH_PROXIMITY_DISABLE;
		*read_data = -1;
		mutex_unlock(&prox_enable_lock);
		return SH_PROXIMITY_RESULT_SUCCESS;
	}

	DEBUG_LOG("Proximity Sensor Enable");
	*state_data = SH_PROXIMITY_ENABLE;

        nResult = PROX_I2cRead(GP2AP030_REG_COMMAND1, &rData);
	if(nResult != 0){
		*read_data = SH_PROXIMITY_FAR;
		DEBUG_LOG("I2CRead_Error");
	}else{
		rData = (rData & 0x08);
		if(0x08 == rData){
			*read_data = SH_PROXIMITY_NEAR;
		}else {
			*read_data = SH_PROXIMITY_FAR;
		}
	}

	DEBUG_SENSOR_LOG("PROX_stateread_func = %d",*read_data);

	FUNC_FIN_LOG();

	mutex_unlock(&prox_enable_lock);
	return SH_PROXIMITY_RESULT_SUCCESS;
}

//static int PROX_ConfigGPIO(void)
//{
//
//	FUNC_LOG();
//
//	return 0;
//}

static int PROX_Initialize(void)
{
//	int nResult = 0;
//	unsigned char wData;
	uint16_t rev;
	
	FUNC_LOG();

	rev = sh_get_hw_revision();
	rev = rev & 0x07;

#if CONFIG_PROXIMITY_MODEL_TYPE == MODEL_TYPE_A1
	switch(rev){
		case HW_ES0:
			Threshold_Low = 0x012c;
			Threshold_High = 0x0190;
			break;		
		case HW_ES1:
			Threshold_Low = 0x012c;
			Threshold_High = 0x0190;
			break;		
		case HW_ES15:
			Threshold_Low = 0x012c;
			Threshold_High = 0x0190;
//			vreg_vl8 = regulator_get(NULL, "8921_l8");
//			regulator_set_voltage(vreg_vl8,3000000,3000000);
			break;
		case HW_PP1:
			Threshold_Low = 0x015e;
			Threshold_High = 0x0190;
			break;		
		case HW_PP15:	//HW_PP16
			Threshold_Low = 0x015e;
			Threshold_High = 0x0190;
			break;		
		case HW_PP2:	
			Threshold_Low = 0x00dc;
			Threshold_High = 0x00f0;
			break;		
		default:
			Threshold_Low = 0x012c;
			Threshold_High = 0x0190;
			break;
	}
#elif CONFIG_PROXIMITY_MODEL_TYPE == MODEL_TYPE_S1
	switch(rev){
		case HW_ES0:
			Threshold_Low = 0x0050;
			Threshold_High = 0x0080;
			break;		
		case HW_ES1:	//ES0.5
			Threshold_Low = 0x0050;
			Threshold_High = 0x0080;
			break;		
		case HW_ES15:	//ES1
			Threshold_Low = 0x0120;
			Threshold_High = 0x0150;
//			vreg_vl8 = regulator_get(NULL, "8921_l8");
//			regulator_set_voltage(vreg_vl8,3000000,3000000);
			break;
		case HW_PP1:
			Threshold_Low = 0x0050;
			Threshold_High = 0x0080;
			break;		
		case HW_PP2_B:	//PP2_PP2.5
			Threshold_Low = 0x0060;
			Threshold_High = 0x006a;
			break;		
		case HW_PP2:	//PMP
			Threshold_Low = 0x0060;
			Threshold_High = 0x006a;
			break;		
		default:
			Threshold_Low = 0x0050;
			Threshold_High = 0x0080;
			break;
	}
#elif CONFIG_PROXIMITY_MODEL_TYPE == MODEL_TYPE_D1
	switch(rev){
		case HW_ES0:
			Threshold_Low = 0x012c;
			Threshold_High = 0x0190;
			break;		
		case HW_ES1:
			Threshold_Low = 0x0fa0;
			Threshold_High = 0x0f50;
			break;		
		case HW_ES15:
			Threshold_Low = 0x012c;
			Threshold_High = 0x0190;
//			vreg_vl8 = regulator_get(NULL, "8921_l8");
//			regulator_set_voltage(vreg_vl8,3000000,3000000);
			break;
		case HW_PP1:
			Threshold_Low = 0x00A0;
			Threshold_High = 0x00C0;
			break;		
		case HW_PP2:
			Threshold_Low = 0x012c;
			Threshold_High = 0x0190;
			break;		
		default:
			Threshold_Low = 0x012c;
			Threshold_High = 0x0190;
			break;
	}
#elif CONFIG_PROXIMITY_MODEL_TYPE == MODEL_TYPE_D2
	switch(rev){
		case HW_ES0:
			Threshold_Low = 0x012c;
			Threshold_High = 0x0190;
			break;		
		case HW_ES1:
			Threshold_Low = 0x012c;
			Threshold_High = 0x0190;
			break;		
		case HW_ES15:
			Threshold_Low = 0x012c;
			Threshold_High = 0x0190;
//			vreg_vl8 = regulator_get(NULL, "8921_l8");
//			regulator_set_voltage(vreg_vl8,3000000,3000000);
			break;
		case HW_PP1:
			Threshold_Low = 0x01f0;
			Threshold_High = 0x0230;
			break;		
		case HW_PP2:
			Threshold_Low = 0x01f0;
			Threshold_High = 0x0230;
			break;		
		default:
			Threshold_Low = 0x012c;
			Threshold_High = 0x0190;
			break;
	}
#elif CONFIG_PROXIMITY_MODEL_TYPE == MODEL_TYPE_D3
	switch(rev){
		case HW_ES0:
			Threshold_Low = 0x012c;
			Threshold_High = 0x0190;
			break;		
		case HW_ES1:
			Threshold_Low = 0x0600;
			Threshold_High = 0x0700;
			break;		
		case HW_ES15:
			Threshold_Low = 0x012c;
			Threshold_High = 0x0190;
//			vreg_vl8 = regulator_get(NULL, "8921_l8");
//			regulator_set_voltage(vreg_vl8,3000000,3000000);
			break;
		case HW_PP1:
			Threshold_Low = 0x0470;
			Threshold_High = 0x0550;
			break;		
		case HW_PP2:
			Threshold_Low = 0x0470;
			Threshold_High = 0x0550;
			break;		
		default:
			Threshold_Low = 0x012c;
			Threshold_High = 0x0190;
			break;
	}
#else
	switch(rev){
		case HW_ES0:
			Threshold_Low = 0x012c;
			Threshold_High = 0x0190;
			break;		
		case HW_ES1:
			Threshold_Low = 0x012c;
			Threshold_High = 0x0190;
			break;		
		case HW_ES15:
			Threshold_Low = 0x012c;
			Threshold_High = 0x0190;
//			vreg_vl8 = regulator_get(NULL, "8921_l8");
//			regulator_set_voltage(vreg_vl8,3000000,3000000);
			break;
		case HW_PP1:
			Threshold_Low = 0x012c;
			Threshold_High = 0x0190;
			break;		
		case HW_PP15:	//HW_PP16
			Threshold_Low = 0x012c;
			Threshold_High = 0x0190;
			break;		
		case HW_PP2:
			Threshold_Low = 0x012c;
			Threshold_High = 0x0190;
			break;		
		default:
			Threshold_Low = 0x012c;
			Threshold_High = 0x0190;
			break;
	}
#endif
	return 0;

}

static int PROX_Remove(void)
{
//	int nResult = 0;

	FUNC_LOG();

	input_free_device(poProximityRec->input_dev);
	kfree(poProximityRec);
	
	return 0;
}

static int PROX_Probe(void)
{
	int nResult = 0;


	FUNC_LOG();

	/* Allocate memory for driver data */
	poProximityRec = kzalloc(sizeof(drv_data), GFP_KERNEL);
	if (!poProximityRec) {
		DEBUG_LOG("memory allocation failed.");
		nResult = -ENOMEM;
		goto memory_error;
	}

	nResult = PROX_Initialize();
	if (nResult < 0) {
		DEBUG_LOG("initialize failed.");
		goto initialize_error;
	}

	nResult = misc_register(&PROX_device);
	if (nResult)
	{
		DEBUG_LOG("misc_register failed.");
		goto misc_register_error;
	}



	/* Declare input device */
	poProximityRec->input_dev = input_allocate_device();
	if (!poProximityRec->input_dev) {
		nResult = -ENOMEM;
		DEBUG_LOG("Failed to allocate input device.");
		goto input_dev_error;
	}


	/* Setup input device */
	set_bit(EV_ABS, poProximityRec->input_dev->evbit);

	/* proximity value near=7, far=0 */
	input_set_abs_params(poProximityRec->input_dev, ABS_DISTANCE, 0, 7, 0, 0);

	/* Set name */
	poProximityRec->input_dev->name = "proximity";

	/* Register */
	nResult = input_register_device(poProximityRec->input_dev);
	if (nResult) {
		DEBUG_LOG("Unable to register input device.");
		goto input_register_error;
	}

	return 0;

input_register_error:
	input_free_device(poProximityRec->input_dev);
input_dev_error:
	misc_deregister(&PROX_device);
misc_register_error:
initialize_error:
	kfree(poProximityRec);
memory_error:
	return nResult;

}

static int __init PROX_Init(void)
{
	FUNC_LOG();

	wake_lock_init(&prox_timeout_wake_lock, WAKE_LOCK_SUSPEND, "prox_timeout_wake_lock");
	wake_lock_init(&prox_wake_lock, WAKE_LOCK_SUSPEND, "proximity_wake_lock");
    mutex_init(&prox_enable_lock);

	return PROX_Probe();
}

static void __exit PROX_Exit(void)
{
	FUNC_LOG();

	wake_lock_destroy(&prox_wake_lock);
	wake_unlock(&prox_timeout_wake_lock);
	wake_lock_destroy(&prox_timeout_wake_lock);

	PROX_Remove();
}


module_init(PROX_Init);
module_exit(PROX_Exit);


MODULE_DESCRIPTION("proximity sensor driver");
MODULE_LICENSE("GPL v2");
MODULE_AUTHOR("SHARP CORPORATION");
MODULE_VERSION("1.0");
