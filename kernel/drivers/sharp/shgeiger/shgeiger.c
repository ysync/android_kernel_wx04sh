/* driver/sharp/shgeiger/shgeiger.c  (shgeiger Driver)
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
/* SHARP GEIGER SENSOR DRIVER FOR KERNEL MODE                             */
/* ------------------------------------------------------------------------- */

/* ------------------------------------------------------------------------- */
/* INCLUDE FILES                                                             */
/* ------------------------------------------------------------------------- */

#include <linux/kernel.h>
#include <linux/interrupt.h>
#include <linux/miscdevice.h>
#include <linux/module.h>
#include <linux/input.h>
#include <linux/delay.h>
#include <linux/poll.h>
#include <mach/gpio.h>
#include <linux/gpio.h>
#include <mach/vreg.h>
#include <../../../arch/arm/mach-msm/timer.h>
#include <linux/i2c.h>
#include <linux/irq.h>
#include <linux/device.h>
#include <sharp/sh_boot_manager.h>
#include <linux/hrtimer.h>
#include <linux/time.h>
#include <linux/slab.h>
#include <linux/wakelock.h>
#include <linux/regulator/consumer.h>
#include <linux/wait.h>
#include <linux/freezer.h>
#include <linux/pm_qos.h>

#include <sharp/shgeiger.h>
#include <sharp/shgeiger_sub.h>
//#include <sharp/shterm_k.h>

/* ------------------------------------------------------------------------- */
/* MACRAOS                                                                   */
/* ------------------------------------------------------------------------- */

#define	I2C_RETRY								3
#define REGISTER_MAX 							21
#define SHGEIGER_QOS_LATENCY_IDLE_DISABLE		1999

/* adb debug_log */
static int    shgeiger_dbg_func_log = 0;       /* log : Init = OFF */
static int    shgeiger_dbg_func_fin_log = 0;   /* log : Init = OFF */
static int    shgeiger_dbg_enable_log = 0;     /* log : Init = OFF */
static int    shgeiger_dbg_sensordata_log = 0; /* log : Init = OFF */
static int    shgeiger_dbg_error_log = 1;      /* log : Init = ON  */

#if defined (CONFIG_ANDROID_ENGINEERING)
    module_param(shgeiger_dbg_func_log, int, 0600);
    module_param(shgeiger_dbg_func_fin_log, int, 0600);
    module_param(shgeiger_dbg_enable_log, int, 0600);
    module_param(shgeiger_dbg_sensordata_log, int, 0600);
    module_param(shgeiger_dbg_error_log, int, 0600);
#endif  /* CONFIG_ANDROID_ENGINEERING */

#define FUNC_LOG() \
    if(shgeiger_dbg_func_log == 1){ \
       printk(KERN_DEBUG "[SHGEIGER] %s is called\n", __func__); \
    }

#define FUNC_FIN_LOG() \
    if(shgeiger_dbg_func_fin_log == 1){ \
       printk(KERN_DEBUG "[SHGEIGER] %s is finished\n", __func__); \
    }

#define DEBUG_LOG(format, ...) \
    if(shgeiger_dbg_enable_log == 1){ \
       printk(KERN_DEBUG "[SHGEIGER][%s] " format "\n", __func__, ## __VA_ARGS__); \
    }

#define DEBUG_SENSORDATA_LOG(format, ...) \
    if(shgeiger_dbg_sensordata_log == 1){ \
       printk(KERN_DEBUG "[SHGEIGER][%s] " format "\n", __func__, ## __VA_ARGS__); \
    }

#define DEBUG_ERROR_LOG(format, ...) \
    if(shgeiger_dbg_error_log == 1){ \
       printk(KERN_DEBUG "[SHGEIGER][%s] " format "\n", __func__, ## __VA_ARGS__); \
    }


/* ------------------------------------------------------------------------- */
/* VARIABLES                                                                 */
/* ------------------------------------------------------------------------- */

typedef struct drv_data_tag     drv_data;
typedef struct i2c_client       I2cClt;
typedef struct i2c_device_id    I2cDevID;
typedef struct work_struct      WorkStruct;
typedef struct device           Device;

struct count_data {
	uint8_t time_slot_msb;
	uint8_t time_slot_lsb;
	uint8_t num_atom_msb;
	uint8_t num_atom_lsb;
	uint8_t num_dc_msb;
	uint8_t num_dc_lsb;
};

struct drv_data_tag
{
	int			irq_gpio;
	WorkStruct	IrqWork;
};

typedef struct
{
	uint8_t	mbRegAdr;					/* Register */
	uint8_t mbData;						/* Data */
} I2cWriteData;


static I2cClt		*this_client;

static atomic_t		open_flag = ATOMIC_INIT(0);
struct semaphore geiger_mutex;
static atomic_t		active_mode = ATOMIC_INIT(0);
static uint8_t		measure_state = 0x80;
static uint8_t		state_mask = 0x9F;
static atomic_t		first_start = ATOMIC_INIT(0);
static atomic_t		thermalstart_mode = ATOMIC_INIT(0);
static atomic_t		chipver_enable = ATOMIC_INIT(1);
static atomic_t		first_hithtmp = ATOMIC_INIT(0);
static uint8_t		chipver = 0xFF;
static int 			alarm_wait = ALARM_NOT_LOCKED;
static int 			thermal_wait = THERMAL_NOT_LOCKED;
static wait_queue_head_t alarmwait_t;
static wait_queue_head_t thermalwait_t;
static uint8_t		atom_threshold_MSB = 0xFF;
static uint8_t		atom_threshold_LSB = 0xFF;
static uint16_t		atom_threshold = 0xFFFF;
static uint8_t 	thermal_threshold_HIGH = 55;
static uint8_t 	thermal_threshold_LOW = 50;
static uint8_t Threshold_high;
static uint8_t Threshold_low;
static uint8_t dac_data = SH_LOW_SENS;
WorkStruct	vib_pause;
WorkStruct	vib_restart;
WorkStruct	spk_pause;
WorkStruct	spk_restart;
WorkStruct	dtv_shutdown;
WorkStruct	dtv_active;
WorkStruct	ir_pause;
WorkStruct	ir_restart;

static struct wake_lock irq_wakelock;
static struct pm_qos_request shgeiger_qos_cpu_dma_latency;

static unsigned long long int total_count_time = 0;
static unsigned long long int total_stop_time = 0;
static int64_t count_start_time = 0;
static int64_t count_end_time = 0;
static int64_t stop_start_time = 0;
static int64_t stop_end_time = 0;
static int64_t period = 0;

static struct shgeiger_count_data driver_count_data;
static struct shgeiger_count_data clear_count_data;
static struct shgeiger_calibration_data driver_calibration_data;

static char ragdata_es1[REGISTER_MAX] =        {0x00,0x00,0x00,0xFF,0xFF,0xFF,0x0F,0x4B,0x50,0xB7,0x65,0x0A,0x58,0x6A,0x10,0x9C,0x0F,0x1C,0x10,0x11,0x00};
static char ragdata_pp1[REGISTER_MAX] =        {0x00,0x00,0x00,0xFF,0xFF,0xFF,0x0F,0x4B,0x50,0xB7,0x65,0x0A,0x62,0x6A,0x10,0x9C,0x0F,0x1C,0x10,0x11,0x00};
static char ragdata_pp2[REGISTER_MAX] =        {0x00,0x00,0x00,0xFF,0xFF,0xFF,0x0F,0x4B,0x50,0xB7,0x65,0x0A,0x62,0x6A,0x10,0x9C,0x0F,0x1C,0x10,0x11,0x00};
static char ragdata_other[REGISTER_MAX] =      {0x00,0x00,0x00,0xFF,0xFF,0xFF,0x0F,0x4B,0x50,0xB7,0x65,0x0A,0x62,0x6A,0x10,0x9C,0x0F,0x1C,0x10,0x11,0x00};

/* ------------------------------------------------------------------------- */
/* PROTOTYPES                                                                */
/* ------------------------------------------------------------------------- */

static int SHGEIGER_Probe(I2cClt *client, const I2cDevID *poDevId);
static int SHGEIGER_Remove(I2cClt *client);
static void SHGEIGER_MutexDown(void);
static void SHGEIGER_MutexUP(void);
static void SHGEIGER_DBUGPRINT(struct shgeiger_count_data *data, char *str);
static int64_t SHGEIGER_Gettime(int64_t start_time,int64_t end_time);
static void shsys_hrtimer_msleep(unsigned int msec);
static int SHGEIGER_I2cRead(I2cClt *client, char *rxData, int length);
static int SHGEIGER_I2cWrite(I2cClt *client, uint8_t bRegAdr, uint8_t bData);
static int SHGEIGER_PowerON(void);
static int SHGEIGER_PowerOFF(void);
static int SHGEIGER_Change(uint8_t MSB, uint8_t LSB);
static int SHGEIGER_GetCountData(struct count_data *get_data);
static int SHGEIGER_BitWrite1(uint8_t addr, uint8_t mask);
static int SHGEIGER_BitWrite0(uint8_t addr, uint8_t mask);
static int SHGEIGER_CountManager( int flag );
static int SHGEIGER_ThermalRead( unsigned int *thermal );
static void SHGEIGER_IrqEnable( void );
static void SHGIEGER_IrqDisable( void );
void SHGEIGERShutdown_Dtv( WorkStruct *work );
void SHGEIGERActive_Dtv( WorkStruct *work );
void SHGEIGERPause_Vib( WorkStruct *work );
void SHGEIGERReStart_Vib( WorkStruct *work );
void SHGEIGERPause_Spk( WorkStruct *work );
void SHGEIGERReStart_Spk( WorkStruct *work );
void SHGEIGERPause_Ir( WorkStruct *work );
void SHGEIGERReStart_Ir( WorkStruct *work );
void ShGeigerShutdown_Dtv( void );
void ShGeigerActive_Dtv( void );
void ShGeigerPause_Vib( void );
void ShGeigerReStart_Vib( void );
void ShGeigerPause_Spk( void );
void ShGeigerReStart_Spk( void );
void ShGeigerPause_Ir( void );
void ShGeigerReStart_Ir( void );
static int IOECS_Clear( void );
static int IOECS_Start( void );
static int IOECS_Stop( void );
static int IOECS_Active( void );
static int IOECS_Shutdown( void );
static int IOECS_CalibrationRead( void __user *argp );
static int IOECS_TimeSlot( void __user *argp );
static int IOECS_ReadCountData( void __user *argp );
static int IOECS_Thermal_Start( void );
static int IOECS_Thermal_Stop( void );
static int IOECS_ThermalRead( void __user *argp );
static int IOECS_CountThreshold( void __user *argp );
static int IOECS_ThermalThreshold( void __user *argp );
static int IOECS_AlarmWait( void __user *argp );
static int IOECS_AlarmWaitUnlock( void __user *argp );
static int IOECS_ThermalWait( void __user *argp );
static int IOECS_ThermalWaitUnlock( void __user *argp );
static int IOECS_StateCheck( void __user *argp );
static int IOECS_GMBIST( void __user *argp );
static int IOECS_Idle_Wake_Lock( void );
static int IOECS_Idle_Wake_UnLock( void );
static int IOECS_Sensitivity_Ctl( void __user *argp );
static int SHGEIGER_open(struct inode *inode, struct file *filp);
static int SHGEIGER_release(struct inode *inode, struct file *filp);
static long SHGEIGER_ioctl(struct file *filp,unsigned int cmd, unsigned long arg);
static irqreturn_t SHGEIGER_interrupt(int irq, void *dev_id);
static void SHGEIGER_Irq_workfunc(WorkStruct *work);
static int SHGEIGER_ReleaseGPIO(drv_data *poShGeigerRec);
static int SHGEIGER_ConfigGPIO(drv_data *poShGeigerRec);
static int SHGEIGER_RegisterInitialize( void );
static int SHGEIGER_CalibrationRead( void );
static int SHGEIGER_ThermalThreshold( void );
static int SHGEIGER_Remove(I2cClt *client);
static int __init SHGEIGER_Init(void);
static void __exit SHGEIGER_Exit(void);

static const I2cDevID gI2cDevIdTableAcc[] =
{
   { SH_GEIGER_I2C_DEVNAME, 0 },
   { }
};

static struct i2c_driver goI2cAccDriver =
{
	.driver =
	{
		.owner = THIS_MODULE,
		.name  = SH_GEIGER_I2C_DEVNAME,
	},
	.probe	  = SHGEIGER_Probe,
	.remove	  = SHGEIGER_Remove,
	.id_table = gI2cDevIdTableAcc,
};

MODULE_DEVICE_TABLE(i2c, gI2cDevIdTableAcc);

static struct file_operations SHGEIGER_ctl_fops = {
	.owner		= THIS_MODULE,
	.open		= SHGEIGER_open,
	.release	= SHGEIGER_release,
	.unlocked_ioctl		= SHGEIGER_ioctl,
};

static struct miscdevice SHGEIGER_device = {
 .minor = MISC_DYNAMIC_MINOR,
	.name = "shgeiger_dev",
	.fops = &SHGEIGER_ctl_fops,
};


static void SHGEIGER_MutexDown(void)
{
	FUNC_LOG();
	down(&geiger_mutex);
	FUNC_FIN_LOG()
}

static void SHGEIGER_MutexUP(void)
{
	FUNC_LOG();
	up(&geiger_mutex);
	FUNC_FIN_LOG();
}

static void SHGEIGER_DBUGPRINT(struct shgeiger_count_data *data, char *str)
{
	DEBUG_SENSORDATA_LOG("time_slot[%10d] atom[%5d] dc[%5d] thermal[%3d] total_time[%15llu] total_stop_time[%15llu] (%s)",data->num_time_slot,data->num_atom,data->num_dc,data->thermal_data,data->total_time_nsec,data->total_stoptime_nsec,str);
}

static int64_t SHGEIGER_Gettime(int64_t start_time,int64_t end_time)
{
	int64_t time;

	FUNC_LOG();

	if (end_time != 0) {
		time = end_time - start_time;
		if(time < 0){
			time += period;
		}
	} else {
		time = 0;	
	}

	FUNC_FIN_LOG()

	return time;
}


static void shsys_hrtimer_msleep(unsigned int msec)
{
    struct timespec tu;

    if(msec >= 1000){
        tu.tv_sec  = msec / 1000;
        tu.tv_nsec = (msec % 1000) * 1000000;
    }else{
    	tu.tv_sec = 0;
        tu.tv_nsec = msec * 1000000;
    }
    hrtimer_nanosleep(&tu, NULL, HRTIMER_MODE_REL, CLOCK_MONOTONIC);
}


static int SHGEIGER_I2cRead(I2cClt *client, char *rxData, int length)
{
	uint8_t loop_i;
	struct i2c_msg msgs[] = {
		{
			.addr = client->addr,
			.flags = 0,
			.len = 1,
			.buf = rxData,
		},
		{
			.addr = client->addr,
			.flags = I2C_M_RD,
			.len = length,
			.buf = rxData,
		},
	};

	for (loop_i = 0; loop_i < I2C_RETRY; loop_i++) {
		if (i2c_transfer(client->adapter, msgs, 2) > 0) {
			break;
		}
		mdelay(10);
	}

	if (loop_i >= I2C_RETRY) {
		DEBUG_ERROR_LOG("I2cRead: error\n");
		return SH_GEIGER_RESULT_FAILURE;
	}

	return 0;
}

static int SHGEIGER_I2cWrite(I2cClt *client, uint8_t bRegAdr, uint8_t bData)
{
	int nResult;
	int nI;
	uint8_t bBuff[2];
	struct i2c_msg oMsgs[] =
		{
			[0] =
				{
					.addr	= client->addr,
					.flags	= 0,
					.buf	= (void *)bBuff,
					.len	= 2
				}
		};

	bBuff[0] = bRegAdr;
	bBuff[1] = bData;
	for(nI = 0; nI < I2C_RETRY; nI++)
	{
		nResult = i2c_transfer(client->adapter, oMsgs, 1);
		if(nResult == 1)
		{
            DEBUG_LOG("I2cWrite success(%02X,reg:%02X,Data:%02X)=%d", client->addr, bRegAdr, bData, nResult);
			return SH_GEIGER_RESULT_SUCCESS;
		}
	}

	DEBUG_ERROR_LOG("I2cWrite: error\n");
	return SH_GEIGER_RESULT_FAILURE;
}

static int SHGEIGER_PowerON(void)
{

	FUNC_LOG();

	/*EN High*/
	gpio_set_value(SH_GEIGER_EN,1);
	shsys_hrtimer_msleep(100);

//	shterm_k_set_info( SHTERM_INFO_GEIGER, 1 );

	return SH_GEIGER_RESULT_SUCCESS;
}

static int SHGEIGER_PowerOFF(void)
{
	int nResult = SH_GEIGER_RESULT_SUCCESS;

	FUNC_LOG();

	nResult = SHGEIGER_BitWrite0(EN_CP,0x80);
	if (nResult != 0){
		nResult = SH_GEIGER_RESULT_FAILURE;
	}
	/*EN Low*/
	gpio_set_value(SH_GEIGER_EN,0);
	
//	shterm_k_set_info( SHTERM_INFO_GEIGER, 0 );

	FUNC_FIN_LOG();
	return nResult;
}

static int SHGEIGER_Change(uint8_t MSB, uint8_t LSB)
{
	int result;

	FUNC_LOG();
	
	result = MSB;
	result = ((result << 8) & 0xFF00);
	result = result + LSB;

	FUNC_FIN_LOG();
	return result;
}

static int SHGEIGER_GetCountData(struct count_data *get_data)
{
	int nResult = SH_GEIGER_RESULT_SUCCESS;
	uint8_t rbuffer[5];

	FUNC_LOG();

	rbuffer[0] = NUM_TIME_SLOT_LSB;
	nResult = SHGEIGER_I2cRead(this_client,rbuffer,1);
	if(nResult != 0){
		DEBUG_ERROR_LOG("I2cRead Error");
	}
	get_data->time_slot_lsb  = rbuffer[0];

	rbuffer[0] = NUM_TIME_SLOT_MSB;
	nResult = SHGEIGER_I2cRead(this_client,rbuffer,1);
	if(nResult != 0){
		DEBUG_ERROR_LOG("I2cRead Error");
	}
	get_data->time_slot_msb  = rbuffer[0];

	rbuffer[0] = NUM_ATOM_LSB;
	nResult = SHGEIGER_I2cRead(this_client,rbuffer,1);
	if(nResult != 0){
		DEBUG_ERROR_LOG("I2cRead Error");
	}
	get_data->num_atom_lsb  = rbuffer[0];

	rbuffer[0] = NUM_ATOM_MSB;
	nResult = SHGEIGER_I2cRead(this_client,rbuffer,1);
	if(nResult != 0){
		DEBUG_ERROR_LOG("I2cRead Error");
	}
	get_data->num_atom_msb  = rbuffer[0];

	rbuffer[0] = NUM_DOC_LSB;
	nResult = SHGEIGER_I2cRead(this_client,rbuffer,1);
	if(nResult != 0){
		DEBUG_ERROR_LOG("I2cRead Error");
	}
	get_data->num_dc_lsb  = rbuffer[0];

	rbuffer[0] = NUM_DOC_MSB;
	nResult = SHGEIGER_I2cRead(this_client,rbuffer,1);
	if(nResult != 0){
		DEBUG_ERROR_LOG("I2cRead Error");
	}
	get_data->num_dc_msb  = rbuffer[0];


	FUNC_FIN_LOG();
	return nResult;

}

static int SHGEIGER_BitWrite1(uint8_t addr, uint8_t mask)
{
	int nResult = SH_GEIGER_RESULT_SUCCESS;
	uint8_t rbuffer[5];
	uint8_t rData;
	uint8_t wData;

	FUNC_LOG();

	rbuffer[0] = addr;
	nResult = SHGEIGER_I2cRead(this_client,rbuffer,1);
	if(nResult != 0){
		nResult = SH_GEIGER_RESULT_FAILURE;
		goto i2c_error;
	}
	rData = rbuffer[0];
	DEBUG_LOG("Read Data:%02Xh",rData);

	wData = rData | mask;
	DEBUG_LOG("Write Data:%02Xh",wData);

	nResult = SHGEIGER_I2cWrite(this_client,addr,wData);
	if (nResult != 0){
		nResult = SH_GEIGER_RESULT_FAILURE;
		goto i2c_error;
	}

	FUNC_FIN_LOG();
	return nResult;

i2c_error:
	return nResult;
}

static int SHGEIGER_BitWrite0(uint8_t addr, uint8_t mask)
{
	int nResult = SH_GEIGER_RESULT_SUCCESS;
	uint8_t rbuffer[5];
	uint8_t rData;
	uint8_t wData;
	uint8_t wmask;

	FUNC_LOG();

	rbuffer[0] = addr;
	nResult = SHGEIGER_I2cRead(this_client,rbuffer,1);
	if(nResult != 0){
		nResult = SH_GEIGER_RESULT_FAILURE;
		goto i2c_error;
	}
	rData = rbuffer[0];
	DEBUG_LOG("Read Data:%02Xh",rData);

	wmask = (~mask);

	wData = rData & wmask;
	DEBUG_LOG("Write Data:%02Xh",wData);

	nResult = SHGEIGER_I2cWrite(this_client,addr,wData);
	if (nResult != 0){
		nResult = SH_GEIGER_RESULT_FAILURE;
		goto i2c_error;
	}

	FUNC_FIN_LOG();
	return nResult;

i2c_error:
	return nResult;
}

static int SHGEIGER_CountManager( int flag )
{
	int64_t time = 0;
	int nResult = SH_GEIGER_RESULT_SUCCESS;
	
	FUNC_LOG();

	if(atomic_read(&active_mode) == 1){
		/*Start*/
		if(flag == 0){
			if((measure_state & state_mask) == 0x00){
				count_start_time = msm_timer_get_sclk_time(&period);
				if(stop_start_time != 0){
					stop_end_time = msm_timer_get_sclk_time(NULL);
					time = SHGEIGER_Gettime(stop_start_time,stop_end_time);
					total_stop_time += time;
				}
				nResult = SHGEIGER_BitWrite1(COMMAND1,0x10);
				if (nResult != 0){
					DEBUG_ERROR_LOG("SHGEIGER Start Error");
					nResult = SH_GEIGER_RESULT_FAILURE;
				}
			}
		}else{	/*Stop*/
			if((measure_state & state_mask) == 0x00){
				stop_start_time = msm_timer_get_sclk_time(&period);
				if(count_start_time != 0){
					count_end_time = msm_timer_get_sclk_time(NULL);
					time = SHGEIGER_Gettime(count_start_time,count_end_time);
					total_count_time += time;
				}
				nResult = SHGEIGER_BitWrite0(COMMAND1,0x10);
				if (nResult != 0){
					DEBUG_ERROR_LOG("SHGEIGER Stop Error");
					nResult = SH_GEIGER_RESULT_FAILURE;
				}
			}
		}
	}

	FUNC_FIN_LOG();
	return nResult;
}



static int SHGEIGER_ThermalRead( unsigned int *thermal )
{
	int nResult = SH_GEIGER_RESULT_SUCCESS;
	uint8_t rbuffer[5];
	unsigned int thermaldata = 0;
	
	FUNC_LOG();

	if(atomic_read(&thermalstart_mode) == 1){
		rbuffer[0] = THERMAL_VALUE;
		nResult = SHGEIGER_I2cRead(this_client,rbuffer,1);
		if(nResult != 0){
			return SH_GEIGER_RESULT_FAILURE;
		}
		thermaldata = rbuffer[0];
		DEBUG_LOG("thermaldata = %d",thermaldata);
	}else{
		DEBUG_ERROR_LOG("Not ThermalSensor start");
		return SH_GEIGER_RESULT_FAILURE;
	}

	*thermal = thermaldata;

	FUNC_FIN_LOG();
	return nResult;
}

static void SHGEIGER_IrqEnable( void )
{
	FUNC_LOG();

	enable_irq_wake(this_client->irq);
	enable_irq(this_client->irq);

	FUNC_FIN_LOG();
}

static void SHGIEGER_IrqDisable( void )
{
	FUNC_LOG();

	disable_irq_wake(this_client->irq);
	disable_irq(this_client->irq);

	FUNC_FIN_LOG();
}

void SHGEIGERShutdown_Dtv( WorkStruct *work )
{
	struct count_data copy_data;
	struct shgeiger_count_data geiger_copy_data;
	unsigned int thermal_data;

	FUNC_LOG();

	SHGEIGER_MutexDown();

	if((measure_state & 0x01) == 0x00){
		SHGEIGER_CountManager( 1 );
		/*measure_state = xxxx xxx1*/
		measure_state = (measure_state | 0x01);
		
		if(atomic_read(&active_mode) == 1){
			/*Chip Data Read*/
			SHGEIGER_GetCountData(&copy_data);
			/*Thermal Read*/
			SHGEIGER_ThermalRead( &thermal_data );
			/*Power OFF*/
			SHGEIGER_PowerOFF();
			/*Chip data is hold*/
			geiger_copy_data.num_time_slot = SHGEIGER_Change(copy_data.time_slot_msb,copy_data.time_slot_lsb);
			geiger_copy_data.num_atom = SHGEIGER_Change(copy_data.num_atom_msb,copy_data.num_atom_lsb);
			geiger_copy_data.num_dc = SHGEIGER_Change(copy_data.num_dc_msb,copy_data.num_dc_lsb);
			
			driver_count_data.num_time_slot += geiger_copy_data.num_time_slot;
			driver_count_data.num_atom += geiger_copy_data.num_atom;
			driver_count_data.num_dc += geiger_copy_data.num_dc;
			driver_count_data.thermal_data = thermal_data;
		}
		/*Thermal State Clear*/
		measure_state = (measure_state & 0xFD);
		atomic_set(&first_hithtmp,0);
	}

	SHGEIGER_MutexUP();
	FUNC_FIN_LOG();
}

void SHGEIGERActive_Dtv( WorkStruct *work )
{
	int nResult = SH_GEIGER_RESULT_SUCCESS;
	FUNC_LOG();

	SHGEIGER_MutexDown();

	if((measure_state & 0x01) == 0x01){
		if(atomic_read(&active_mode) == 1){
			SHGEIGER_PowerON();
			nResult = SHGEIGER_RegisterInitialize();
			if(nResult < 0){
				DEBUG_ERROR_LOG("RegisterInitialize Error");
			}
			nResult = SHGEIGER_BitWrite1(EN_CP,0x80);
			if (nResult != 0){
				DEBUG_ERROR_LOG("CP_EN Error");
			}
			
			shsys_hrtimer_msleep(2000);
			
			/*ThermalSensor ON*/
			nResult = SHGEIGER_BitWrite1(COMMAND1,0x40);
			if (nResult != 0){
				DEBUG_ERROR_LOG("SHGEIGER Thermal Start Error");
			}
			/*Count Threshold Setting*/
			nResult = SHGEIGER_I2cWrite(this_client,ATOM_THRESHOLD_LSB,atom_threshold_LSB);
			if (nResult != 0){
				DEBUG_ERROR_LOG("I2cWrite Error");
			}
			nResult = SHGEIGER_I2cWrite(this_client,ATOM_THRESHOLD_MSB,atom_threshold_MSB);
			if (nResult != 0){
				DEBUG_ERROR_LOG("I2cWrite Error");
			}
			DEBUG_LOG("CountThreshold MSB = %d,LSB = %d",atom_threshold_MSB,atom_threshold_LSB);
		}
		/*measure_state = xxxx xxx0*/
		measure_state = (measure_state & 0xFE);
		SHGEIGER_CountManager( 0 );
	}

	SHGEIGER_MutexUP();

}


void SHGEIGERPause_Vib( WorkStruct *work )
{
	FUNC_LOG();

	SHGEIGER_MutexDown();

	if((measure_state & 0x08) == 0x00){
		SHGEIGER_CountManager( 1 );
		/*measure_state = xxxx 1xxx*/
		measure_state = (measure_state | 0x08);
	}

	SHGEIGER_MutexUP();
	FUNC_FIN_LOG();
}

void SHGEIGERReStart_Vib( WorkStruct *work )
{
	FUNC_LOG();

	SHGEIGER_MutexDown();

	if((measure_state & 0x08) == 0x08){
		/*measure_state = xxxx 0xxx*/
		measure_state = (measure_state & 0xF7);
		SHGEIGER_CountManager( 0 );
	}

	SHGEIGER_MutexUP();
	FUNC_FIN_LOG();
}

void SHGEIGERPause_Spk( WorkStruct *work )
{
	FUNC_LOG();

	SHGEIGER_MutexDown();

	if((measure_state & 0x04) == 0x00){
		SHGEIGER_CountManager( 1 );
		/*measure_state = xxxx x1xx*/
		measure_state = (measure_state | 0x04);
	}

	SHGEIGER_MutexUP();
	FUNC_FIN_LOG();
}

void SHGEIGERReStart_Spk( WorkStruct *work )
{
	FUNC_LOG();

	SHGEIGER_MutexDown();

	if((measure_state & 0x04) == 0x04){
		/*measure_state = xxxx x0xx*/
		measure_state = (measure_state & 0xFB);
		SHGEIGER_CountManager( 0 );
	}

	SHGEIGER_MutexUP();
	FUNC_FIN_LOG();
}

void SHGEIGERPause_Ir( WorkStruct *work )
{
	FUNC_LOG();

	SHGEIGER_MutexDown();

	if((measure_state & 0x10) == 0x00){
		SHGEIGER_CountManager( 1 );
		/*measure_state = xxx1 xxxx*/
		measure_state = (measure_state | 0x10);
	}

	SHGEIGER_MutexUP();
	FUNC_FIN_LOG();
}

void SHGEIGERReStart_Ir( WorkStruct *work )
{
	FUNC_LOG();

	SHGEIGER_MutexDown();

	if((measure_state & 0x10) == 0x10){
		/*measure_state = xxx0 xxxx*/
		measure_state = (measure_state & 0xEF);
		SHGEIGER_CountManager( 0 );
	}

	SHGEIGER_MutexUP();
	FUNC_FIN_LOG();
}

void ShGeigerShutdown_Dtv( void )
{
	FUNC_LOG();

	schedule_work(&dtv_shutdown);

}

void ShGeigerActive_Dtv( void )
{
	FUNC_LOG();

	schedule_work(&dtv_active);

}

void ShGeigerPause_Vib( void )
{
	FUNC_LOG();

	schedule_work(&vib_pause);
}

void ShGeigerReStart_Vib( void )
{
	FUNC_LOG();

	schedule_work(&vib_restart);

}

void ShGeigerPause_Spk( void )
{
	FUNC_LOG();

	schedule_work(&spk_pause);

}

void ShGeigerReStart_Spk( void )
{
	FUNC_LOG();

	schedule_work(&spk_restart);

}

void ShGeigerPause_Ir( void )
{
	FUNC_LOG();

	schedule_work(&ir_pause);
}

void ShGeigerReStart_Ir( void )
{
	FUNC_LOG();

	schedule_work(&ir_restart);
}

static int IOECS_Clear( void )
{
	struct shgeiger_count_data count_data;
	struct count_data copy_data;

	FUNC_LOG();
	SHGEIGER_MutexDown();

	if((measure_state & 0x01) == 0x00){
		SHGEIGER_GetCountData(&copy_data);	
		count_data.num_time_slot = SHGEIGER_Change(copy_data.time_slot_msb,copy_data.time_slot_lsb);
		count_data.num_atom = SHGEIGER_Change(copy_data.num_atom_msb,copy_data.num_atom_lsb);
		count_data.num_dc = SHGEIGER_Change(copy_data.num_dc_msb,copy_data.num_dc_lsb);
		clear_count_data.num_time_slot = count_data.num_time_slot + driver_count_data.num_time_slot;
		clear_count_data.num_atom = count_data.num_atom + driver_count_data.num_atom;
		clear_count_data.num_dc = count_data.num_dc + driver_count_data.num_dc;
	}else{
		clear_count_data.num_time_slot = driver_count_data.num_time_slot;
		clear_count_data.num_atom = driver_count_data.num_atom;
		clear_count_data.num_dc = driver_count_data.num_dc;
	}

	/*Timer clear*/
	total_count_time = 0;
	total_stop_time = 0;

	if((measure_state & state_mask) == 0x00){
		count_start_time = msm_timer_get_sclk_time(&period);
		count_end_time = 0;
		stop_start_time = 0;
		stop_end_time = 0;
	} else {
		stop_start_time = msm_timer_get_sclk_time(&period);
		stop_end_time = 0;
		count_start_time = 0;
		count_end_time = 0;
	}

	SHGEIGER_MutexUP();
	FUNC_FIN_LOG();

	return SH_GEIGER_RESULT_SUCCESS;

}

static int IOECS_Start( void )
{
	FUNC_LOG();
	SHGEIGER_MutexDown();

	if((measure_state & 0x80) == 0x80)
	{
		/*measure_state = 0xxx xxxx*/
		measure_state = (measure_state & 0x7F);
		SHGEIGER_CountManager( 0 );
		
		if(atomic_read(&first_start) == 0){
			if((measure_state & state_mask) != 0x00){
				stop_start_time = msm_timer_get_sclk_time(&period);
			}
			atomic_set(&first_start,1);
		}
		if(atomic_read(&thermalstart_mode) == 0){
			atomic_set(&thermalstart_mode,1);
		}
		SHGEIGER_IrqEnable();
	}

	SHGEIGER_MutexUP();
	FUNC_FIN_LOG();	
	return SH_GEIGER_RESULT_SUCCESS;
}

static int IOECS_Stop( void )
{
	FUNC_LOG();
	SHGEIGER_MutexDown();

	if((measure_state & 0x80) == 0x00)
	{
		SHGEIGER_CountManager( 1 );
		/*measure_state = 1xxx xxxx*/
		measure_state = measure_state | 0x80;
		
		if(atomic_read(&thermalstart_mode) == 1){
			atomic_set(&thermalstart_mode,0);
		}
		SHGIEGER_IrqDisable();
	}

	SHGEIGER_MutexUP();
	FUNC_FIN_LOG();	
	return 0;
}

static int IOECS_Active( void )
{
	int nResult = SH_GEIGER_RESULT_SUCCESS;
	FUNC_LOG();

	SHGEIGER_MutexDown();

	if(atomic_read(&active_mode) == 0){
		if((measure_state & 0x01) == 0x00){
			SHGEIGER_PowerON();
			nResult = SHGEIGER_RegisterInitialize();
			if(nResult < 0){
				DEBUG_ERROR_LOG("RegisterInitialize Error");
				nResult = SH_GEIGER_RESULT_FAILURE;
				goto error;
			}
			nResult = SHGEIGER_BitWrite1(EN_CP,0x80);
			if (nResult != 0){
				DEBUG_ERROR_LOG("CP_EN Error");
				nResult = SH_GEIGER_RESULT_FAILURE;
				goto error;
			}
			
			shsys_hrtimer_msleep(2000);
			
			/*ThermalSensor ON*/
			nResult = SHGEIGER_BitWrite1(COMMAND1,0x40);
			if (nResult != 0){
				DEBUG_ERROR_LOG("SHGEIGER Thermal Start Error");
				nResult = SH_GEIGER_RESULT_FAILURE;
				goto error;
			}
		}

		driver_count_data.num_time_slot = 0;
		driver_count_data.num_atom = 0;
		driver_count_data.num_dc = 0;
		driver_count_data.thermal_data = 0;

		clear_count_data.num_time_slot = 0;
		clear_count_data.num_atom = 0;
		clear_count_data.num_dc = 0;
		clear_count_data.thermal_data = 0;	

		/*Timer Clear*/
		total_count_time = 0;
		total_stop_time = 0;
		count_start_time = 0;
		count_end_time = 0;
		stop_start_time = 0;
		stop_end_time = 0;
		period = 0;
		
		atomic_set(&first_start, 0);
		atomic_set(&active_mode, 1);
	}

	SHGEIGER_MutexUP();

	FUNC_FIN_LOG();	
	return nResult;
	
error:
	SHGEIGER_PowerOFF();
	SHGEIGER_MutexUP();
	return nResult;
}

static int IOECS_Shutdown( void )
{
	int nResult = 0;
	FUNC_LOG();

	SHGEIGER_MutexDown();

	if(atomic_read(&active_mode) == 1)
	{
		/*Counter Stop*/
		if((measure_state & 0x80) == 0x00)
		{
			SHGEIGER_CountManager( 1 );
			/*measure_state = 1xxx xxxx*/
			measure_state = measure_state | 0x80;
			
			if(atomic_read(&thermalstart_mode) == 1){
				atomic_set(&thermalstart_mode,0);
			}
			SHGIEGER_IrqDisable();
		}

		if(alarm_wait == ALARM_ALREADY_LOCKED){
			wake_up_interruptible( &alarmwait_t );
			alarm_wait = ALARM_NOT_LOCKED;
		}
		if(thermal_wait == THERMAL_ALREADY_LOCKED){
			wake_up_interruptible( &thermalwait_t );
			thermal_wait = THERMAL_NOT_LOCKED;
		}		
		irq_set_irq_type(this_client->irq,IRQ_TYPE_LEVEL_LOW);

		if((measure_state & 0x01) == 0x00){
			/*ThermalSensor OFF*/
			nResult = SHGEIGER_BitWrite0(COMMAND1,0x40);
			if (nResult != 0){
				DEBUG_ERROR_LOG("SHGEIGER Thermal Stop Error");
			}
			SHGEIGER_PowerOFF();
		}
		dac_data = SH_LOW_SENS;
		/*Thermal State Clear*/
		measure_state = (measure_state & 0xFD);
		atomic_set(&first_hithtmp,0);
		atomic_set(&active_mode, 0);
	}

	SHGEIGER_MutexUP();

	FUNC_FIN_LOG();	
	return SH_GEIGER_RESULT_SUCCESS;
}

static int IOECS_CalibrationRead( void __user *argp )
{
	int nResult = SH_GEIGER_RESULT_SUCCESS;

	FUNC_LOG();

	nResult = copy_to_user(argp,&driver_calibration_data,sizeof(struct shgeiger_calibration_data));
	if( nResult != 0 ){
		return SH_GEIGER_RESULT_FAILURE;
	}

	FUNC_FIN_LOG();	
	return SH_GEIGER_RESULT_SUCCESS;
}

static int IOECS_TimeSlot( void __user *argp )
{
	FUNC_LOG();


	FUNC_FIN_LOG();	
	return SH_GEIGER_RESULT_SUCCESS;
}

static int IOECS_ReadCountData( void __user *argp )
{
	struct shgeiger_count_data count_data;
	struct count_data copy_data;
	unsigned int thermal_data;
	int nResult = SH_GEIGER_RESULT_SUCCESS;
	int64_t count_time = 0;
	int64_t stop_time = 0;

	FUNC_LOG();
	SHGEIGER_MutexDown();

	if((measure_state & 0x01) == 0x00){
		SHGEIGER_GetCountData(&copy_data);
	}
	
	/*Count time*/
	if((measure_state & state_mask) == 0x00){
		count_end_time = msm_timer_get_sclk_time(NULL);
		count_time = SHGEIGER_Gettime(count_start_time,count_end_time);
	}else{
		stop_end_time = msm_timer_get_sclk_time(NULL);
		stop_time = SHGEIGER_Gettime(stop_start_time,stop_end_time);
	}
	count_data.total_time_nsec = total_count_time + count_time;
	count_data.total_stoptime_nsec = total_stop_time + stop_time;

	if((measure_state & 0x01) == 0x00){
		count_data.num_time_slot = SHGEIGER_Change(copy_data.time_slot_msb,copy_data.time_slot_lsb);
		count_data.num_atom = SHGEIGER_Change(copy_data.num_atom_msb,copy_data.num_atom_lsb);
		count_data.num_dc = SHGEIGER_Change(copy_data.num_dc_msb,copy_data.num_dc_lsb);
		SHGEIGER_ThermalRead( &thermal_data );
		count_data.thermal_data = thermal_data;

		SHGEIGER_DBUGPRINT(&count_data, "[0]ChipReadData");
		SHGEIGER_DBUGPRINT(&driver_count_data, "[1]DriverData");

		count_data.num_time_slot += driver_count_data.num_time_slot;
		count_data.num_atom += driver_count_data.num_atom;
		count_data.num_dc += driver_count_data.num_dc;

		SHGEIGER_DBUGPRINT(&count_data, "[2]ChipReadData + DriverData");
	}else{
		count_data.num_time_slot = driver_count_data.num_time_slot;
		count_data.num_atom = driver_count_data.num_atom;
		count_data.num_dc = driver_count_data.num_dc;
		count_data.thermal_data = driver_count_data.thermal_data;

		SHGEIGER_DBUGPRINT(&count_data, "[3]DriverData");
	}

	SHGEIGER_DBUGPRINT(&clear_count_data, "[4]ClearData");

	count_data.num_time_slot -= clear_count_data.num_time_slot;
	count_data.num_atom -= clear_count_data.num_atom;
	count_data.num_dc -= clear_count_data.num_dc;	
	
	SHGEIGER_DBUGPRINT(&count_data, "[5]ChipReadData + DriverData - ClearData");

	nResult = copy_to_user(argp,&count_data,sizeof(struct shgeiger_count_data));
	if(nResult != 0){
		SHGEIGER_MutexUP();
		return SH_GEIGER_RESULT_FAILURE;
	}

	SHGEIGER_MutexUP();

	FUNC_FIN_LOG();	
	return SH_GEIGER_RESULT_SUCCESS;
}

static int IOECS_Thermal_Start( void )
{
	FUNC_LOG();

//	if(atomic_read(&thermalstart_mode) == 0){
//		nResult = SHGEIGER_BitWrite1(COMMAND1,0x40);
//		if (nResult != 0){
//			DEBUG_LOG("SHGEIGER ThermalStart Error");
//			nResult = -1;
//			goto error;
//		}
//		atomic_set(&thermalstart_mode, 1);
//	}

	FUNC_FIN_LOG();	
	return SH_GEIGER_RESULT_SUCCESS;

//error:
//	return nResult;
}

static int IOECS_Thermal_Stop( void )
{
	FUNC_LOG();

//	if(atomic_read(&thermalstart_mode) == 1){
//		nResult = SHGEIGER_BitWrite0(COMMAND1,0x40);
//		if (nResult != 0){
//			DEBUG_LOG("SHGEIGER ThermalStop Error");
//			nResult = -1;
//			goto error;
//		}
//		atomic_set(&thermalstart_mode, 0);
//	}

	FUNC_FIN_LOG();	
	return SH_GEIGER_RESULT_SUCCESS;

//error:
//	return nResult;
}

static int IOECS_ThermalRead( void __user *argp )
{
	unsigned int thermal;
	int nResult = SH_GEIGER_RESULT_SUCCESS;
	
	FUNC_LOG();

	SHGEIGER_MutexDown();

	if((measure_state & 0x01) == 0x00){
		SHGEIGER_ThermalRead( &thermal );
	}else{
		thermal = driver_count_data.thermal_data;
	}
	
	nResult = copy_to_user(argp, &thermal, sizeof(thermal));
	if(nResult != 0){
		SHGEIGER_MutexUP();
		return SH_GEIGER_RESULT_FAILURE;
	}

	SHGEIGER_MutexUP();

	return SH_GEIGER_RESULT_SUCCESS;

}

static int IOECS_CountThreshold( void __user *argp )
{
	int nResult = SH_GEIGER_RESULT_SUCCESS;
	unsigned int user_threshold;
	struct count_data copy_data;
	struct shgeiger_count_data geiger_copy_data;

	FUNC_LOG();

	SHGEIGER_MutexDown();

	nResult = copy_from_user(&user_threshold,argp,sizeof(user_threshold));
	if(nResult != 0){
		SHGEIGER_MutexUP();
		return SH_GEIGER_RESULT_FAILURE;
	}
	if(user_threshold > 65535 || user_threshold == 0 ){
		user_threshold = 65535;
	}

	if((measure_state & 0x01) == 0x00){
		/* Chip data is hold and Chip data clear  */
		SHGEIGER_GetCountData(&copy_data);
		geiger_copy_data.num_time_slot = SHGEIGER_Change(copy_data.time_slot_msb,copy_data.time_slot_lsb);
		geiger_copy_data.num_atom = SHGEIGER_Change(copy_data.num_atom_msb,copy_data.num_atom_lsb);
		geiger_copy_data.num_dc = SHGEIGER_Change(copy_data.num_dc_msb,copy_data.num_dc_lsb);

		SHGEIGER_DBUGPRINT(&geiger_copy_data, "[6]ChipData");

		driver_count_data.num_time_slot += geiger_copy_data.num_time_slot;
		driver_count_data.num_atom += geiger_copy_data.num_atom;
		driver_count_data.num_dc += geiger_copy_data.num_dc;

		SHGEIGER_DBUGPRINT(&driver_count_data, "[7]DriverData");

		nResult = SHGEIGER_BitWrite1(COUNT_CLEAR,0x01);
		if (nResult != 0){
			DEBUG_ERROR_LOG("IOECS_CountThreshold Error");
		}
		nResult = SHGEIGER_BitWrite0(COUNT_CLEAR,0x01);
		if (nResult != 0){
			DEBUG_ERROR_LOG("IOECS_CountThreshold Error");
		}
		/**************************************/
		
		/*Count Threshold Setting*/
		DEBUG_LOG("CountThreshold = %d",user_threshold);
		atom_threshold = user_threshold;
		atom_threshold_LSB = (user_threshold & 0x00FF);
		atom_threshold_MSB = ((user_threshold >> 8) & 0x00FF);
		DEBUG_LOG("CountThreshold MSB = %d,LSB = %d",atom_threshold_MSB,atom_threshold_LSB);

		nResult = SHGEIGER_I2cWrite(this_client,ATOM_THRESHOLD_LSB,atom_threshold_LSB);
		if (nResult != 0){
			DEBUG_ERROR_LOG("I2cWrite Error");
		}
		nResult = SHGEIGER_I2cWrite(this_client,ATOM_THRESHOLD_MSB,atom_threshold_MSB);
		if (nResult != 0){
			DEBUG_ERROR_LOG("I2cWrite Error");
		}
		/*******************************************/
	}else{
		DEBUG_LOG("CountThreshold = %d",user_threshold);
		atom_threshold = user_threshold;
		atom_threshold_LSB = (user_threshold & 0x00FF);
		atom_threshold_MSB = ((user_threshold >> 8) & 0x00FF);
		DEBUG_LOG("CountThreshold MSB = %d,LSB = %d",atom_threshold_MSB,atom_threshold_LSB);
	}

	SHGEIGER_MutexUP();
	FUNC_FIN_LOG();
	return SH_GEIGER_RESULT_SUCCESS;

}

static int IOECS_ThermalThreshold( void __user *argp )
{
	FUNC_LOG();

	FUNC_FIN_LOG();
	return SH_GEIGER_RESULT_SUCCESS;

}

static int IOECS_ChipVerRead( void __user *argp )
{
	int nResult = SH_GEIGER_RESULT_SUCCESS;

	FUNC_LOG();

	if(atomic_read(&chipver_enable) == 1){
		nResult = copy_to_user(argp, &chipver, sizeof(chipver));
		if(nResult != 0){
			nResult = SH_GEIGER_RESULT_FAILURE;
		}
	}else{
		nResult = SH_GEIGER_RESULT_FAILURE;
	}

	FUNC_FIN_LOG();
	return nResult;

}

static int IOECS_AlarmWait( void __user *argp )
{
	int nResult = SH_GEIGER_RESULT_SUCCESS;

	FUNC_LOG();

	if(alarm_wait == ALARM_ALREADY_LOCKED){
		DEBUG_LOG("Already locked");
		nResult = copy_to_user(argp, &alarm_wait, sizeof(alarm_wait));
		return SH_GEIGER_RESULT_SUCCESS;
	}

	alarm_wait = ALARM_ALREADY_LOCKED;

	DEBUG_LOG("Alarm Lock Start");
	nResult = wait_event_freezable(alarmwait_t,(alarm_wait != ALARM_ALREADY_LOCKED));
	DEBUG_LOG("Alarm Lock End");

	if( nResult != 0){
		DEBUG_LOG("System_Busy");
		alarm_wait = ALARM_UNLOCK_FAILED;
	}

	nResult = copy_to_user(argp, &alarm_wait, sizeof(alarm_wait));
	if(nResult != 0){
		DEBUG_ERROR_LOG("copy_to_user Error");
		alarm_wait = ALARM_NOT_LOCKED;
		return SH_GEIGER_RESULT_FAILURE;
	}
	alarm_wait = ALARM_NOT_LOCKED;
	FUNC_FIN_LOG();
	return SH_GEIGER_RESULT_SUCCESS;

}

static int IOECS_AlarmWaitUnlock( void __user *argp )
{
	int nResult = SH_GEIGER_RESULT_SUCCESS;
	FUNC_LOG();
	
	if(alarm_wait == ALARM_NOT_LOCKED){
		DEBUG_LOG("Alarm not locked");
		nResult = copy_to_user(argp, &alarm_wait, sizeof(alarm_wait));
		return SH_GEIGER_RESULT_FAILURE;
	}

	alarm_wait = ALARM_UNLOCK_SUCCESS;
	wake_up_interruptible( &alarmwait_t );
	nResult = copy_to_user(argp, &alarm_wait, sizeof(alarm_wait));
	if(nResult != 0){
		DEBUG_ERROR_LOG("copy_to_user Error");
		return SH_GEIGER_RESULT_FAILURE;
	}
	FUNC_FIN_LOG();
	return SH_GEIGER_RESULT_SUCCESS;
}

static int IOECS_ThermalWait( void __user *argp )
{
	int nResult = SH_GEIGER_RESULT_SUCCESS;
	FUNC_LOG();

	if(thermal_wait == THERMAL_ALREADY_LOCKED){
		DEBUG_LOG("Already locked");
		nResult = copy_to_user(argp, &thermal_wait, sizeof(thermal_wait));
		return SH_GEIGER_RESULT_SUCCESS;
	}

	if(((measure_state & 0x02) == 0x02) && (atomic_read(&first_hithtmp) == 0)){
		thermal_wait = THERMAL_HIGH_UNLOCK;
		nResult = copy_to_user(argp, &thermal_wait, sizeof(thermal_wait));
		thermal_wait = THERMAL_NOT_LOCKED;
		atomic_set(&first_hithtmp,1);
		return SH_GEIGER_RESULT_SUCCESS;
	}
	atomic_set(&first_hithtmp,1);

	thermal_wait =  THERMAL_ALREADY_LOCKED;

	DEBUG_LOG("Thermal Lock Start");
	nResult = wait_event_freezable(thermalwait_t,(thermal_wait != THERMAL_ALREADY_LOCKED));
	DEBUG_LOG("Thermal Lock End");

	if( nResult != 0 ){
		DEBUG_LOG("System_Busy");
		thermal_wait = THERMAL_UNLOCK_FAILED;
	}

	nResult = copy_to_user(argp, &thermal_wait, sizeof(thermal_wait));
	if(nResult != 0){
		DEBUG_ERROR_LOG("copy_to_user Error");
		thermal_wait = THERMAL_NOT_LOCKED;
		return SH_GEIGER_RESULT_FAILURE;
	}
	thermal_wait = THERMAL_NOT_LOCKED;

	FUNC_FIN_LOG();
	return SH_GEIGER_RESULT_SUCCESS;

}

static int IOECS_ThermalWaitUnlock( void __user *argp )
{
	int nResult = SH_GEIGER_RESULT_SUCCESS;
	FUNC_LOG();
	
	if(thermal_wait == THERMAL_NOT_LOCKED){
		nResult = copy_to_user(argp, &thermal_wait, sizeof(thermal_wait));
		return SH_GEIGER_RESULT_FAILURE;
	}

	thermal_wait = THERMAL_UNLOCK_SUCCESS;
	wake_up_interruptible( &thermalwait_t );
	nResult = copy_to_user(argp, &thermal_wait, sizeof(thermal_wait));
	if(nResult != 0){
		DEBUG_ERROR_LOG("copy_to_user Error");
		return SH_GEIGER_RESULT_FAILURE;
	}

	FUNC_FIN_LOG();
	return SH_GEIGER_RESULT_SUCCESS;
}

static int IOECS_StateCheck( void __user *argp )
{
	int state = UNNOWN;
	int nResult = SH_GEIGER_RESULT_SUCCESS;
	FUNC_LOG();

	if(atomic_read(&open_flag) == 0){/*Close*/
		state = CLOSE;
	}else if(atomic_read(&active_mode) == 0){/*Open*/
		state = SHUTDOWN;
	}else{/*Active*/
		if(atomic_read(&first_start) == 0){
			state = ACTIVE;
		}else if((measure_state & 0x80) == 0x80){
			state = STOP;
		}else{
			state = ((measure_state & state_mask) & 0x7F);
			state = (state & 0x00FF);
		}
	}
	
	nResult = copy_to_user(argp, &state, sizeof(state));
	if(nResult != 0){
		DEBUG_ERROR_LOG("copy_to_user Error");
		return SH_GEIGER_RESULT_FAILURE;
	}
	FUNC_FIN_LOG();
	return SH_GEIGER_RESULT_SUCCESS;
}

static int IOECS_GMBIST( void __user *argp )
{
	int nResult = SH_GEIGER_RESULT_SUCCESS;
	int nResult_flag = SH_GEIGER_RESULT_SUCCESS;
	unsigned int atom = 0;
	struct count_data copy_data;
	struct shgeiger_bistdata bist_data; 

	/*initialize*/
	bist_data.vip_atom = 0;
	bist_data.vim_atom = 0;

	FUNC_LOG();

	/*TimeSlot*/
	nResult = SHGEIGER_I2cWrite(this_client,SLOT_TIME,0x01);
	if (nResult != 0){
		DEBUG_ERROR_LOG("I2cWrite Error");
		nResult_flag = SH_GEIGER_RESULT_FAILURE;
	}

	/*BYPASS Mode*/
	nResult = SHGEIGER_I2cWrite(this_client,COMMAND1,0x77);
	if (nResult != 0){
		DEBUG_ERROR_LOG("I2cWrite Error");
		nResult_flag = SH_GEIGER_RESULT_FAILURE;
	}

	/*Chip data clear*/
	nResult = SHGEIGER_BitWrite1(COUNT_CLEAR,0x01);
	if (nResult != 0){
		DEBUG_ERROR_LOG("I2cWrite Error");
		nResult_flag = SH_GEIGER_RESULT_FAILURE;
	}
	nResult = SHGEIGER_BitWrite0(COUNT_CLEAR,0x01);
	if (nResult != 0){
		DEBUG_ERROR_LOG("I2cWrite Error");
		nResult_flag = SH_GEIGER_RESULT_FAILURE;
	}

	/*Comparison threshold*/
	nResult = SHGEIGER_I2cWrite(this_client,0x0D,0x3F);
	if (nResult != 0){
		DEBUG_ERROR_LOG("I2cWrite Error");
		nResult_flag = SH_GEIGER_RESULT_FAILURE;
	}

	/*Nomal Mode*/
	nResult = SHGEIGER_I2cWrite(this_client,0x03,0x00);
	if (nResult != 0){
		DEBUG_ERROR_LOG("I2cWrite Error");
		nResult_flag = SH_GEIGER_RESULT_FAILURE;
	}

	/*Wait(2s)*/
	shsys_hrtimer_msleep(2000);

	/*Test Start(BIST_POLARITY=VIP)*/
	nResult = SHGEIGER_I2cWrite(this_client,0x03,0x80);
	if (nResult != 0){
		DEBUG_ERROR_LOG("I2cWrite Error");
		nResult_flag = SH_GEIGER_RESULT_FAILURE;
	}

	/*Wait(2s)*/
	shsys_hrtimer_msleep(2000);

	/*Atom Read*/
	nResult = SHGEIGER_GetCountData(&copy_data);
	atom = SHGEIGER_Change(copy_data.num_atom_msb,copy_data.num_atom_lsb);
	bist_data.vip_atom = atom;
	if (nResult != 0){
		DEBUG_ERROR_LOG("Atom Read Failed");
		bist_data.vip_atom = 0;
		nResult_flag = SH_GEIGER_RESULT_FAILURE;
	}
	atom = 0;

	/*Chip data clear*/
	nResult = SHGEIGER_BitWrite1(COUNT_CLEAR,0x01);
	if (nResult != 0){
		DEBUG_ERROR_LOG("I2cWrite Error");
		nResult_flag = SH_GEIGER_RESULT_FAILURE;	
	}
	nResult = SHGEIGER_BitWrite0(COUNT_CLEAR,0x01);
	if (nResult != 0){
		DEBUG_ERROR_LOG("I2cWrite Error");
		nResult_flag = SH_GEIGER_RESULT_FAILURE;
	}

	/*Nomal Mode*/
	nResult = SHGEIGER_I2cWrite(this_client,0x03,0x00);
	if (nResult != 0){
		DEBUG_ERROR_LOG("I2cWrite Error");
		nResult_flag = SH_GEIGER_RESULT_FAILURE;
	}

	/*Wait*/
	shsys_hrtimer_msleep(2000);

	/*BIST_POLARITY=VIM*/
	nResult = SHGEIGER_I2cWrite(this_client,0x03,0x08);
	if (nResult != 0){
		DEBUG_ERROR_LOG("I2cWrite Error");
		nResult_flag = SH_GEIGER_RESULT_FAILURE;
	}	

	/*Test Start(BIST_POLARITY=VIM)*/
	nResult = SHGEIGER_I2cWrite(this_client,0x03,0x88);
	if (nResult != 0){
		DEBUG_ERROR_LOG("I2cWrite Error");
		nResult_flag = SH_GEIGER_RESULT_FAILURE;
	}	

	/*Wait*/
	shsys_hrtimer_msleep(2000);

	/*Atom Read*/
	nResult = SHGEIGER_GetCountData(&copy_data);
	atom = SHGEIGER_Change(copy_data.num_atom_msb,copy_data.num_atom_lsb);
	bist_data.vim_atom = atom;
	if (nResult != 0){
		DEBUG_ERROR_LOG("Atom Read Failed");
		bist_data.vim_atom = 0;
		nResult_flag = SH_GEIGER_RESULT_FAILURE;
	}
	

	nResult = SHGEIGER_RegisterInitialize();
	if(nResult < 0){
		DEBUG_ERROR_LOG("RegisterInitialize Error");
		nResult_flag = SH_GEIGER_RESULT_FAILURE;
	}

	nResult = copy_to_user(argp,&bist_data,sizeof(struct shgeiger_bistdata));
	if(nResult != 0){
		DEBUG_ERROR_LOG("copy_to_user Error");
		nResult_flag = SH_GEIGER_RESULT_FAILURE;
	}

	FUNC_FIN_LOG();	
	
	return nResult_flag;

}

static int IOECS_Idle_Wake_Lock( void )
{
	DEBUG_LOG("wake_lock_idle on\n");
	pm_qos_update_request(&shgeiger_qos_cpu_dma_latency, SHGEIGER_QOS_LATENCY_IDLE_DISABLE);

	return SH_GEIGER_RESULT_SUCCESS;
}


static int IOECS_Idle_Wake_UnLock( void )
{
	DEBUG_LOG("wake_lock_idle off\n");
	pm_qos_update_request(&shgeiger_qos_cpu_dma_latency, PM_QOS_DEFAULT_VALUE);

	return SH_GEIGER_RESULT_SUCCESS;
}

static int IOECS_Sensitivity_Ctl( void __user *argp )
{
	int nResult = SH_GEIGER_RESULT_SUCCESS;
	int nResult_flag = SH_GEIGER_RESULT_SUCCESS;
	unsigned char user_mode;
	int sensor_off_flg = 0;

	FUNC_LOG();
	SHGEIGER_MutexDown();

	nResult = copy_from_user(&user_mode,argp,sizeof(user_mode));
	if(nResult != 0){
		SHGEIGER_MutexUP();
		return SH_GEIGER_RESULT_FAILURE;
	}

	if(user_mode >= NUM_SENSITIVITY_MODE){
		DEBUG_ERROR_LOG("not support mode");
		SHGEIGER_MutexUP();
		return SH_GEIGER_RESULT_FAILURE;
	}

	if(atomic_read(&active_mode) != 1){
		DEBUG_ERROR_LOG("sensor not active");
		SHGEIGER_MutexUP();
		return SH_GEIGER_RESULT_FAILURE;
	}

	if((measure_state & 0x80) == 0x00)
	{
		DEBUG_LOG("sensor stop\n");
		SHGEIGER_CountManager( 1 );
		/*measure_state = 1xxx xxxx*/
		measure_state = measure_state | 0x80;
		
		if(atomic_read(&thermalstart_mode) == 1){
			atomic_set(&thermalstart_mode,0);
		}
		SHGIEGER_IrqDisable();
		sensor_off_flg = 1;
	}

	switch( user_mode ){
		case 0:
			dac_data = SH_LOW_SENS;
			DEBUG_LOG("Sensitivity LowMode\n");
			break;
		case 1:
			dac_data = SH_HIGH_SENS;
			DEBUG_LOG("Sensitivity HighMode\n");
			break;
		default:
			break;
	}

	if((measure_state & 0x01) == 0x00){
		nResult = SHGEIGER_I2cWrite(this_client,0x0D,dac_data);
		if ( nResult != 0 ){
			DEBUG_ERROR_LOG("I2cWrite Error");
			nResult_flag = SH_GEIGER_RESULT_FAILURE;
		}

		nResult = SHGEIGER_BitWrite1(COUNT_CLEAR,0x01);
		if (nResult != 0){
			DEBUG_ERROR_LOG("I2cWrite Error");
			nResult_flag = SH_GEIGER_RESULT_FAILURE;
		}

		nResult = SHGEIGER_BitWrite0(COUNT_CLEAR,0x01);
		if (nResult != 0){
			DEBUG_ERROR_LOG("I2cWrite Error");
			nResult_flag = SH_GEIGER_RESULT_FAILURE;
		}
	}

	driver_count_data.num_time_slot = 0;
	driver_count_data.num_atom = 0;
	driver_count_data.num_dc = 0;
	driver_count_data.thermal_data = 0;
	clear_count_data.num_time_slot = 0;
	clear_count_data.num_atom = 0;
	clear_count_data.num_dc = 0;
	clear_count_data.thermal_data = 0;
	total_count_time = 0;
	total_stop_time = 0;
	count_start_time = 0;
	count_end_time = 0;
	stop_start_time = 0;
	stop_end_time = 0;
	period = 0;
	atomic_set(&first_start, 0);

	if(((measure_state & 0x80) == 0x80) && (sensor_off_flg == 1))
	{
		DEBUG_LOG("sensor start\n");
		/*measure_state = 0xxx xxxx*/
		measure_state = (measure_state & 0x7F);
		SHGEIGER_CountManager( 0 );
		
		if(atomic_read(&first_start) == 0){
			if((measure_state & state_mask) != 0x00){
				stop_start_time = msm_timer_get_sclk_time(&period);
			}
			atomic_set(&first_start,1);
		}
		if(atomic_read(&thermalstart_mode) == 0){
			atomic_set(&thermalstart_mode,1);
		}
		SHGEIGER_IrqEnable();
	}

	SHGEIGER_MutexUP();
	FUNC_FIN_LOG();
	return nResult_flag;
}

static int SHGEIGER_open(struct inode *inode, struct file *filp)
{
	FUNC_LOG();

	atomic_set(&open_flag, 1);

	FUNC_FIN_LOG();
	return SH_GEIGER_RESULT_SUCCESS;
}

static int SHGEIGER_release(struct inode *inode, struct file *filp)
{
	FUNC_LOG();

	atomic_set(&open_flag, 0);
	DEBUG_LOG("wake_lock_idle off\n");
	pm_qos_update_request(&shgeiger_qos_cpu_dma_latency, PM_QOS_DEFAULT_VALUE);

	FUNC_FIN_LOG();
	return SH_GEIGER_RESULT_SUCCESS;
}

static long SHGEIGER_ioctl(struct file *filp,
			unsigned int cmd, unsigned long arg)
{
    void __user *argp = (void __user*)arg;

	FUNC_LOG();

	switch (cmd) {
		case SHGEIGER_IOCTL_ACTIVE:
			DEBUG_LOG("SHGEIGER_IOCTL_ACTIVE");
			if(IOECS_Active() < 0)
			{
				return -EIO;
			}
			break;
		case SHGEIGER_IOCTL_SHUTDOWN:
			DEBUG_LOG("SHGEIGER_IOCTL_SHUTDOWN");
			if(IOECS_Shutdown() < 0)
			{
				return -EIO;
			}
			break;
		case SHGEIGER_IOCTL_START:
			DEBUG_LOG("SHGEIGER_IOCTL_START");
			if(IOECS_Start() < 0)
			{
				return -EIO;
			}
			break;
		case SHGEIGER_IOCTL_STOP:
			DEBUG_LOG("SHGEIGER_IOCTL_STOP");
			if(IOECS_Stop() < 0)
			{
				return -EIO;
			}
			break;
		case SHGEIGER_IOCTL_CALIBRATIONREAD:
			DEBUG_LOG("SHGEIGER_IOCTL_CALIBRATIONREAD");
			if(IOECS_CalibrationRead( argp ) < 0)
			{
				return -EIO;
			}
			break;
		case SHGEIGER_IOCTL_TIMESLOT:
			DEBUG_LOG("SHGEIGER_IOCTL_TIMESLOT");
			if(IOECS_TimeSlot( argp ) < 0)
			{
				return -EIO;
			}
			break;
		case SHGEIGER_IOCTL_READCOUNTDATA:
			DEBUG_LOG("SHGEIGER_IOCTL_READCOUNTDATA");
			if(IOECS_ReadCountData( argp ) < 0)
			{
				return -EIO;
			}
			break;
		case SHGEIGER_IOCTL_CLEAR:
			DEBUG_LOG("SHGEIGER_IOCTL_CLEAR");
			if(IOECS_Clear() < 0)
			{
				return -EIO;
			}
			break;
		case SHGEIGER_IOCTL_THERMAL_START:
			DEBUG_LOG("SHGEIGER_IOCTL_THERMAL_START");
			if(IOECS_Thermal_Start( ) < 0)
			{
				return -EIO;
			}
			break;
		case SHGEIGER_IOCTL_THERMAL_STOP:
			DEBUG_LOG("SHGEIGER_IOCTL_THERMAL_STOP");
			if(IOECS_Thermal_Stop( ) < 0)
			{
				return -EIO;
			}
			break;
		case SHGEIGER_IOCTL_THERMALREAD:
			DEBUG_LOG("SHGEIGER_IOCTL_THERMALREAD");
			if(IOECS_ThermalRead( argp ) < 0)
			{
				return -EIO;
			}
			break;		
		case SHGEIGER_IOCTL_COUNTTHRESHOLD:
			DEBUG_LOG("SHGEIGER_IOCTL_COUNTTHRESHOLD");
			if(IOECS_CountThreshold( argp ) < 0)
			{
				return -EIO;
			}
			break;
		case SHGEIGER_IOCTL_THERMALTHRESHOLD:
			DEBUG_LOG("SHGEIGER_IOCTL_THERMALTHRESHOLD");
			if(IOECS_ThermalThreshold( argp ) < 0)
			{
				return -EIO;
			}
			break;
		case SHGEIGER_IOCTL_CHIPVERREAD:
			DEBUG_LOG("SHGEIGER_IOCTL_CHIPVERREAD");
			if(IOECS_ChipVerRead( argp ) < 0)
			{
				return -EIO;
			}
			break;
		case SHGEIGER_IOCTL_ALARMWAIT:
			DEBUG_LOG("SHGEIGER_IOCTL_ALARMWAIT");
			if(IOECS_AlarmWait( argp ) < 0)
			{
				return -EIO;
			}
			break;
		case SHGEIGER_IOCTL_ALARMWAIT_UNLOCK:
			DEBUG_LOG("SHGEIGER_IOCTL_ALARMWAIT_UNLOCK");
			if(IOECS_AlarmWaitUnlock( argp ) < 0)
			{
				return -EIO;
			}
			break;
		case SHGEIGER_IOCTL_THERMALWAIT:
			DEBUG_LOG("SHGEIGER_IOCTL_THERMALWAIT");
			if(IOECS_ThermalWait( argp ) < 0)
			{
				return -EIO;
			}
			break;
		case SHGEIGER_IOCTL_THERMALWAIT_UNLOCK:
			DEBUG_LOG("SHGEIGER_IOCTL_THERMALWAIT_UNLOCK");
			if(IOECS_ThermalWaitUnlock( argp ) < 0)
			{
				return -EIO;
			}
			break;
		case SHGEIGER_IOCTL_STATECHECK:
			DEBUG_LOG("SHGEIGER_IOCTL_STATECHECK");
			if(IOECS_StateCheck( argp ) < 0)
			{
				return -EIO;
			}
			break;			
		case SHGEIGER_IOCTL_GMBIST:
			DEBUG_LOG("SHGEIGER_IOCTL_GMBIST");
			if(IOECS_GMBIST( argp ) < 0)
			{
				return -EIO;
			}
			break;
		case SHGEIGER_IOCTL_IDLE_WAKE_LOCK:
			DEBUG_LOG("SHGEIGER_IOCTL_IDLE_WAKE_LOCK");
			if(IOECS_Idle_Wake_Lock() < 0)
			{
				return -EIO;
			}
			break;
		case SHGEIGER_IOCTL_IDLE_WAKE_UNLOCK:
			DEBUG_LOG("SHGEIGER_IOCTL_IDLE_WAKE_UNLOCK");
			if(IOECS_Idle_Wake_UnLock() < 0)
			{
				return -EIO;
			}
			break;
		case SHGEIGER_IOCTL_SENSITIVITY_CTL:
			DEBUG_LOG("SHGEIGER_IOCTL_SENSITIVITY_CTL");
			if(IOECS_Sensitivity_Ctl( argp ) < 0)
			{
				return -EIO;
			}
			break;
		default:
			break;
	}
	FUNC_FIN_LOG();
	return SH_GEIGER_RESULT_SUCCESS;
}

static irqreturn_t SHGEIGER_interrupt(int irq, void *dev_id)
{
	drv_data *poShGeigerRec = dev_id;

	FUNC_LOG();

	wake_lock(&irq_wakelock);

	disable_irq_wake(this_client->irq);
	disable_irq_nosync(this_client->irq);
	schedule_work(&poShGeigerRec->IrqWork);

	FUNC_FIN_LOG();
	return IRQ_HANDLED;

}

static void SHGEIGER_Irq_workfunc(WorkStruct *work)
{
	int nResult = 0;
	struct count_data copy_data;
	struct shgeiger_count_data geiger_copy_data;
	unsigned int thermal_data;

	FUNC_LOG();

	SHGEIGER_MutexDown();

	/* Chip data is hold.  */
	SHGEIGER_GetCountData(&copy_data);
	geiger_copy_data.num_time_slot = SHGEIGER_Change(copy_data.time_slot_msb,copy_data.time_slot_lsb);
	geiger_copy_data.num_atom = SHGEIGER_Change(copy_data.num_atom_msb,copy_data.num_atom_lsb);
	geiger_copy_data.num_dc = SHGEIGER_Change(copy_data.num_dc_msb,copy_data.num_dc_lsb);
	SHGEIGER_ThermalRead( &thermal_data );
	geiger_copy_data.thermal_data = thermal_data;

	/*********************/

	/*AlarmWait*/
	if(geiger_copy_data.num_atom >= atom_threshold){
		DEBUG_LOG("Count Over");
		driver_count_data.num_time_slot += geiger_copy_data.num_time_slot;
		driver_count_data.num_atom += geiger_copy_data.num_atom;
		driver_count_data.num_dc += geiger_copy_data.num_dc;

		/*Chip data clear*/
		nResult = SHGEIGER_BitWrite1(COUNT_CLEAR,0x01);
		if (nResult != 0){
			DEBUG_ERROR_LOG("SHGEIGER_Irq_workfunc Error");
		}
		SHGEIGER_IrqEnable();
		nResult = SHGEIGER_BitWrite0(COUNT_CLEAR,0x01);
		if (nResult != 0){
			DEBUG_ERROR_LOG("SHGEIGER_Irq_workfunc Error");
		}
		/*Chip data clear End*/
		
		if(alarm_wait == ALARM_ALREADY_LOCKED){
			alarm_wait = ALARM_UNLOCK;
			wake_up_interruptible( &alarmwait_t );
		}
		
		goto exit;
	}
	
	/*TimeSlot Max*/
	if(geiger_copy_data.num_time_slot >= 65535){
		DEBUG_LOG("TimeSlot Max");
		
		driver_count_data.num_time_slot += geiger_copy_data.num_time_slot;
		driver_count_data.num_atom += geiger_copy_data.num_atom;
		driver_count_data.num_dc += geiger_copy_data.num_dc;
		
		/*Chip data clear*/
		nResult = SHGEIGER_BitWrite1(COUNT_CLEAR,0x01);
		if (nResult != 0){
			DEBUG_ERROR_LOG("SHGEIGER_Irq_workfunc Error");
		}
		SHGEIGER_IrqEnable();
		nResult = SHGEIGER_BitWrite0(COUNT_CLEAR,0x01);
		if (nResult != 0){
			DEBUG_ERROR_LOG("SHGEIGER_Irq_workfunc Error");
		}
		/*Chip data clear End*/
		goto exit;
	}

	/*thermal wait*/
	if(geiger_copy_data.thermal_data >= Threshold_high){
		DEBUG_LOG("Thermal HIGH");
		irq_set_irq_type(this_client->irq,IRQ_TYPE_LEVEL_HIGH);
		/**Count Pause**/
		if((measure_state & 0x02) == 0x00){
			SHGEIGER_CountManager( 1 );
			/*measure_state = xxxx xx1x*/
			measure_state = (measure_state | 0x02);
		}
		/*********************/
		if(thermal_wait == THERMAL_ALREADY_LOCKED){
			thermal_wait = THERMAL_HIGH_UNLOCK;
			wake_up_interruptible( &thermalwait_t );
		}
		SHGEIGER_IrqEnable();
		goto exit;
	}
	if(geiger_copy_data.thermal_data <= Threshold_low){
		DEBUG_LOG("Thermal Normal");
		irq_set_irq_type(this_client->irq,IRQ_TYPE_LEVEL_LOW);
		/**Count ReStart**/
		if((measure_state & 0x02) == 0x02){
			/*measure_state = xxxx xx0x*/
			measure_state = (measure_state & 0xFD);
			SHGEIGER_CountManager( 0 );
		}
		/*****************/
		if(thermal_wait == THERMAL_ALREADY_LOCKED){
			thermal_wait = THERMAL_NOLMAL_UNLOCK;
			wake_up_interruptible( &thermalwait_t );
		}
		SHGEIGER_IrqEnable();
		goto exit;
	}
	
exit:
	SHGEIGER_MutexUP();
	wake_unlock(&irq_wakelock);

	FUNC_FIN_LOG();
	return;
}


static int SHGEIGER_ReleaseGPIO(drv_data *poShGeigerRec)
{
	FUNC_LOG();

	gpio_free(poShGeigerRec->irq_gpio);

	FUNC_FIN_LOG();
	return SH_GEIGER_RESULT_SUCCESS;
}

static int SHGEIGER_ConfigGPIO(drv_data *poShGeigerRec)
{
	FUNC_LOG();

	poShGeigerRec->irq_gpio = SH_GEIGER_IRQ;
	this_client->irq = MSM_GPIO_TO_INT(poShGeigerRec->irq_gpio);

	FUNC_FIN_LOG();
	return SH_GEIGER_RESULT_SUCCESS;
}

static int SHGEIGER_RegisterInitialize( void )
{
	int nResult = SH_GEIGER_RESULT_SUCCESS;
	uint8_t adder = 0x01;
	int i = 0;
	unsigned short boot_mode;

	FUNC_LOG();

	if( chipver == 0x10 ){	/*ES1*/
		ragdata_es1[5] = Threshold_high;
		ragdata_es1[6] = Threshold_low;
		for(i=0;i < REGISTER_MAX;i++){
			nResult = SHGEIGER_I2cWrite(this_client,adder,ragdata_es1[i]);
			if (nResult != 0){
				nResult = SH_GEIGER_RESULT_FAILURE;
				goto i2c_error;
			}
			adder++;
		}
	}else if( chipver == 0x11 ){	/*PP1*/
		ragdata_pp1[5] = Threshold_high;
		ragdata_pp1[6] = Threshold_low;
		for(i=0;i < REGISTER_MAX;i++){
			nResult = SHGEIGER_I2cWrite(this_client,adder,ragdata_pp1[i]);
			if (nResult != 0){
				nResult = SH_GEIGER_RESULT_FAILURE;
				goto i2c_error;
			}
			adder++;
		}
	}else if( chipver == 0x12 ){	/*PP2*/
		ragdata_pp2[5] = Threshold_high;
		ragdata_pp2[6] = Threshold_low;		
		for(i=0;i < REGISTER_MAX;i++){
			nResult = SHGEIGER_I2cWrite(this_client,adder,ragdata_pp2[i]);
			if (nResult != 0){
				nResult = SH_GEIGER_RESULT_FAILURE;
				goto i2c_error;
			}
			adder++;
		}
	}else{
		for(i=0;i < REGISTER_MAX;i++){
			ragdata_other[5] = Threshold_high;
			ragdata_other[6] = Threshold_low;				
			nResult = SHGEIGER_I2cWrite(this_client,adder,ragdata_other[i]);
			if (nResult != 0){
				nResult = SH_GEIGER_RESULT_FAILURE;
				goto i2c_error;
			}
			adder++;
		}
	}

	/*Sensitivity Setting*/
	boot_mode = sh_boot_get_bootmode();
	if( (boot_mode == SH_BOOT_F_T) || (boot_mode == SH_BOOT_H_C) ) {
		nResult = SHGEIGER_I2cWrite(this_client,0x0D,SH_HIGH_SENS);
		if ( nResult != 0 ){
			DEBUG_ERROR_LOG("I2cWrite Error");
		}
	}else{
		nResult = SHGEIGER_I2cWrite(this_client,0x0D,dac_data);
		if ( nResult != 0 ){
			DEBUG_ERROR_LOG("I2cWrite Error");
		}
	}

i2c_error:
	FUNC_FIN_LOG();
	return nResult;
}

static int SHGEIGER_CalibrationRead( void )
{
	int nResult = SH_GEIGER_RESULT_SUCCESS;
	uint8_t rbuffer[5];
	uint8_t multi_lsb = 0x00;
	uint8_t multi_msb = 0x00;

	FUNC_LOG();

	SHGEIGER_PowerON();

	rbuffer[0] = BG_LSB;
	nResult = SHGEIGER_EEPROM_I2cRead(rbuffer,1);
	if(nResult != 0){
		nResult = SH_GEIGER_RESULT_FAILURE;
		goto i2c_error;
	}
	DEBUG_LOG("BG_LSB = 0x%02X",rbuffer[0]);
	driver_calibration_data.bg_lsb = rbuffer[0];

	rbuffer[0] = BG_MSB;
	nResult = SHGEIGER_EEPROM_I2cRead(rbuffer,1);
	if(nResult != 0){
		nResult = SH_GEIGER_RESULT_FAILURE;
		goto i2c_error;
	}
	DEBUG_LOG("BG_MSB = 0x%02X",rbuffer[0]);
	driver_calibration_data.bg_msb = rbuffer[0];

	rbuffer[0] = MULTI_LSB;
	nResult = SHGEIGER_EEPROM_I2cRead(rbuffer,1);
	if(nResult != 0){
		nResult = SH_GEIGER_RESULT_FAILURE;
		goto i2c_error;
	}
	DEBUG_LOG("MULTI_LSB = 0x%02X",rbuffer[0]);
	multi_lsb = rbuffer[0];

	rbuffer[0] = MULTI_MSB;
	nResult = SHGEIGER_EEPROM_I2cRead(rbuffer,1);
	if(nResult != 0){
		nResult = SH_GEIGER_RESULT_FAILURE;
		goto i2c_error;
	}
	DEBUG_LOG("MULTI_MSB = 0x%02X",rbuffer[0]);
	multi_msb = rbuffer[0];

	rbuffer[0] = MDULE_VER;
	nResult = SHGEIGER_EEPROM_I2cRead(rbuffer,1);
	if(nResult != 0){
		atomic_set(&chipver_enable,0);
		nResult = SH_GEIGER_RESULT_FAILURE;
		goto i2c_error;
	}
	DEBUG_LOG("MDULE_VER = 0x%02X",rbuffer[0]);
	chipver = rbuffer[0];

	rbuffer[0] = THERMAL_A;
	nResult = SHGEIGER_EEPROM_I2cRead(rbuffer,1);
	if(nResult != 0){
		nResult = SH_GEIGER_RESULT_FAILURE;
		goto i2c_error;
	}
	DEBUG_LOG("THERMAL_A = 0x%02X",rbuffer[0]);
	driver_calibration_data.thermal_a = rbuffer[0];

	rbuffer[0] = THERMAL_B;
	nResult = SHGEIGER_EEPROM_I2cRead(rbuffer,1);
	if(nResult != 0){
		nResult = SH_GEIGER_RESULT_FAILURE;
		goto i2c_error;
	}
	DEBUG_LOG("THERMAL_B = 0x%02X",rbuffer[0]);
	driver_calibration_data.thermal_b = rbuffer[0];

i2c_error:
	/*Calibration_default_data*/
	driver_calibration_data.bg = 0.0;
	driver_calibration_data.multi = 0.0;
	driver_calibration_data.multi_data = SHGEIGER_Change(multi_msb,multi_lsb);
	SHGEIGER_PowerOFF();
	FUNC_FIN_LOG();
	return nResult;
}

static int SHGEIGER_ThermalThreshold( void )
{
	int nResult = SH_GEIGER_RESULT_SUCCESS;

	FUNC_LOG();

	Threshold_high = ((thermal_threshold_HIGH+driver_calibration_data.thermal_b)*128)/driver_calibration_data.thermal_a;

	Threshold_low = ((thermal_threshold_LOW+driver_calibration_data.thermal_b)*128)/driver_calibration_data.thermal_a;

	DEBUG_LOG("THERMAL_HIGH : %d,0x%02X", Threshold_high,Threshold_high);
	DEBUG_LOG("THERMAL_LOW : %d,0x%02X", Threshold_low,Threshold_low);

	FUNC_FIN_LOG();
	return nResult;
}


static int SHGEIGER_Remove(I2cClt *client)
{
	drv_data *poShGeigerRec = i2c_get_clientdata(client);

	FUNC_LOG();

	pm_qos_remove_request(&shgeiger_qos_cpu_dma_latency);
	dev_info(&client->dev, "removing driver\n");
	device_init_wakeup(&client->dev, 0);
	SHGEIGER_ReleaseGPIO(poShGeigerRec);
	kfree(poShGeigerRec);

	FUNC_FIN_LOG();
	return SH_GEIGER_RESULT_SUCCESS;	
}

static int SHGEIGER_Probe(I2cClt *client, const I2cDevID *poDevId)
{
	drv_data *poShGeigerRec = NULL;
	int nResult = SH_GEIGER_RESULT_SUCCESS;

	FUNC_LOG();

	sema_init(&geiger_mutex,1);

	/* CLIENT CHECKING */
	if (!i2c_check_functionality(client->adapter, I2C_FUNC_I2C)) {
		DEBUG_ERROR_LOG("check_functionality failed.");
		nResult = -ENODEV;
		goto exit0;
	}

	/* Allocate memory for driver data */
	poShGeigerRec = kzalloc(sizeof(drv_data), GFP_KERNEL);
	if (!poShGeigerRec){
		DEBUG_ERROR_LOG("memory allocation failed.");
		nResult = -ENOMEM;
		goto exit1;
	}

	INIT_WORK(&poShGeigerRec->IrqWork, SHGEIGER_Irq_workfunc);
	i2c_set_clientdata(client, poShGeigerRec);

	INIT_WORK(&vib_pause,SHGEIGERPause_Vib);
	INIT_WORK(&vib_restart,SHGEIGERReStart_Vib);
	INIT_WORK(&spk_pause,SHGEIGERPause_Spk);
	INIT_WORK(&spk_restart,SHGEIGERReStart_Spk);
	INIT_WORK(&dtv_shutdown,SHGEIGERShutdown_Dtv);
	INIT_WORK(&dtv_active,SHGEIGERActive_Dtv);
	INIT_WORK(&ir_pause,SHGEIGERPause_Ir);
	INIT_WORK(&ir_restart,SHGEIGERReStart_Ir);

	/* Copy to global variable */
	this_client = client;

	/* GPIO setting */
	nResult = SHGEIGER_ConfigGPIO(poShGeigerRec);
	if(nResult < 0)
	{
		DEBUG_ERROR_LOG("ConfigGPIO is Err.");
		nResult = SH_GEIGER_RESULT_FAILURE;
		goto exit2;
	}

	/*CalibrationData Read*/
	nResult = SHGEIGER_CalibrationRead();
	if(nResult < 0){
		DEBUG_ERROR_LOG("SHGEIGER_CalibrationRead Error");
		nResult = SH_GEIGER_RESULT_FAILURE;
		goto exit2;
	}

	/*ThermalThreshold Setting*/
	nResult = SHGEIGER_ThermalThreshold();
	if(nResult < 0){
		DEBUG_ERROR_LOG("SHGEIGER_ThermalThreshold Error");
		nResult = SH_GEIGER_RESULT_FAILURE;
		goto exit2;
	}

	nResult = misc_register(&SHGEIGER_device);
	if (nResult)
	{
		DEBUG_ERROR_LOG("misc_register failed.");
		nResult = SH_GEIGER_RESULT_FAILURE;
		goto exit4;
	}

	/* irq will not be enabled on request irq */
	irq_set_status_flags(client->irq, IRQ_NOAUTOEN);

	nResult = request_irq(client->irq, SHGEIGER_interrupt, IRQF_TRIGGER_LOW | IRQF_DISABLED,
					  "GEIGER_INT", poShGeigerRec);
	if (nResult < 0) {
		DEBUG_ERROR_LOG("request irq failed.");
		nResult = SH_GEIGER_RESULT_FAILURE;
		goto exit5;
	}

	init_waitqueue_head( &alarmwait_t );
	init_waitqueue_head( &thermalwait_t );
	pm_qos_add_request(&shgeiger_qos_cpu_dma_latency, PM_QOS_CPU_DMA_LATENCY, PM_QOS_DEFAULT_VALUE);

	FUNC_FIN_LOG();
	return SH_GEIGER_RESULT_SUCCESS;

exit5:
	misc_deregister(&SHGEIGER_device);
exit4:
exit2:
	kfree(poShGeigerRec);
exit1:
exit0:
	return nResult;

}

static int __init SHGEIGER_Init(void)
{
	FUNC_LOG();

	wake_lock_init(&irq_wakelock, WAKE_LOCK_SUSPEND, "shgeiger_wake_lock");

	/* I2C driver Use beginning */
	return i2c_add_driver(&goI2cAccDriver);
}

static void __exit SHGEIGER_Exit(void)
{
	FUNC_LOG();

	wake_lock_destroy(&irq_wakelock);

	/* I2C driver Use end */
	i2c_del_driver(&goI2cAccDriver);
}


module_init(SHGEIGER_Init);
module_exit(SHGEIGER_Exit);


MODULE_DESCRIPTION("shgeiger sensor driver");
MODULE_LICENSE("GPL v2");
MODULE_AUTHOR("SHARP CORPORATION");
MODULE_VERSION("1.0");
