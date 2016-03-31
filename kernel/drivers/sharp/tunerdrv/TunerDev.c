/**************************************************************************************************/
/** 
	@file		TunerDev.c
	@brief		Tuner Device Control
*/
/**************************************************************************************************/

#include <linux/module.h>
#include <linux/miscdevice.h>
#include <linux/fs.h>
#include <linux/poll.h>
//#include <linux/smp_lock.h>
#include <mach/vreg.h>
#include <mach/gpio.h>
#include <linux/clk.h>
#include <asm/uaccess.h> /* for access_ok() */
#include <linux/regulator/consumer.h> /* for regulator_xx() */
#include <linux/pm_qos.h>

#include <asm/io.h>

#include <mach/dma.h>
#include <mach/msm_tsif.h>

#include "gpio_def.h"

#if defined(CONFIG_MACH_ARS)
#include <mach/perflock.h>
#include <linux/mutex.h>
#endif
//#include <sharp/sh_smem.h>

static int gpio_init(void);
static int gpio_get(unsigned int id, int *val);
static int gpio_set(unsigned int id, int val);
static int tuner_vreg_enable(void);
static int tuner_vreg_disable(void);
static int tuner_clk_enable(void);
static int tuner_clk_disable(void);
static int hw_revision_get(unsigned int *val);
#if defined(CONFIG_MACH_ARS)
static int tuner_msm_clock_set(unsigned char freqFlag);
#endif
static struct clk *gp_clk;

#if defined(CONFIG_MACH_ARS)
static struct mutex dtv_lock;
static struct perf_lock dtv_perf_lock_384;
static struct perf_lock dtv_perf_lock_486;
static uint8_t flg_384 = 0;
static uint8_t flg_486 = 0;
#endif

#if defined( CONFIG_MACH_MNB ) || defined(CONFIG_MACH_ARS)
static struct pm_qos_request	pq_req;
#define DTV_PM_QOS_LATENCY_VALUE	1700
#endif

stGPIO_DEF use_gpiono[] = {
	/* GPIO No (ID)			, GPIO PORT No			, Direction		, Init Value	, Initialized	, diag-label	*/
#if VALID_GPIO_DTVEN
	{GPIO_DTVEN_PORTID		, GPIO_DTVEN_PORTNO		, DirctionOut	, 0				, 0				, "gpio_dtv_en"		},
#endif
#if VALID_GPIO_DTVRST
	{GPIO_DTVRST_PORTID 	, GPIO_DTVRST_PORTNO	, DirctionOut	, 0				, 0				, "gpio_dtv_reset"	},
#endif
#if VALID_GPIO_DTVLNAEN
	{GPIO_DTVLNAEN_PORTID	, GPIO_DTVLNAEN_PORTNO	, DirctionOut	, 0				, 0				, "gpio_dtv_lnaen"	},
#endif
#if VALID_GPIO_DTVANTSW
	{GPIO_DTVANTSW_PORTID	, GPIO_DTVANTSW_PORTNO	, DirctionOut	, 0				, 0				, "gpio_dtv_antsw"	},
#endif
#if VALID_GPIO_DTVMANTSL
	{GPIO_DTVMANTSL_PORTID	, GPIO_DTVMANTSL_PORTNO	, DirctionOut	, 0				, 0				, "gpio_dtv_mantsl"	},
#endif
#if VALID_GPIO_DTVUANTSL
	{GPIO_DTVUANTSL_PORTID	, GPIO_DTVUANTSL_PORTNO	, DirctionOut	, 0				, 0				, "gpio_dtv_uantsl"	},
#endif
#if VALID_GPIO_DTVCANTSL
	{GPIO_DTVCANTSL_PORTID	, GPIO_DTVCANTSL_PORTNO	, DirctionOut	, 0				, 0				, "gpio_dtv_cantsl"	},
#endif
#if VALID_GPIO_DTVHWREV
	{GPIO_DTVHWREV_PORTID	, GPIO_DTVHWREV_PORTNO	, DirctionIn	, 0				, 0				, "gpio_dtv_hwrev"	},
#endif
};

/**************************************************************************************************/
/**
	@brief	tuner_open
	@param	struct inode	*inode		[I]
	@param	struct file		*file		[I]
	@retval	0	Success
	@retval	-1	Failed
*/
/**************************************************************************************************/
static int tuner_open(struct inode *inode, struct file *file)
{
	int ret;

#if defined( CONFIG_MACH_MNB ) || defined(CONFIG_MACH_ARS)
	pm_qos_add_request( &pq_req, PM_QOS_CPU_DMA_LATENCY, PM_QOS_DEFAULT_VALUE );
	printk("%s:%d !!!tuner_open  pm_qos_add_request() \n", __FILE__, __LINE__);

	pm_qos_update_request( &pq_req, DTV_PM_QOS_LATENCY_VALUE );
	printk("%s:%d !!!tuner_open  pm_qos_update_request() [DTV_PM_QOS_LATENCY_VALUE] \n", __FILE__, __LINE__);
#endif

	ret  = gpio_init();
	if (ret == 1) {
		/* Failed */
		printk("%s:%d !!!tuner_open gpio_init() error \n", __FILE__, __LINE__);
		return (-1);
	}

	return (0);
}

/**************************************************************************************************/
/**
	@brief	tuner_release
	@param	struct inode	*inode		[I]
	@param	struct file		*file		[I]
	@retval	0	Success
	@retval	-1	Failed
*/
/**************************************************************************************************/
static int tuner_release(struct inode *inode, struct file *file)
{
#if defined( CONFIG_MACH_MNB ) || defined(CONFIG_MACH_ARS)
	pm_qos_update_request( &pq_req, PM_QOS_DEFAULT_VALUE );
	printk("%s:%d !!!tuner_release  pm_qos_update_request() [PM_QOS_DEFAULT_VALUE] \n", __FILE__, __LINE__);

	pm_qos_remove_request( &pq_req );
	printk("%s:%d !!!tuner_release  pm_qos_remove_request() \n", __FILE__, __LINE__);
#endif
	
#if defined(CONFIG_MACH_ARS)
	mutex_lock(&dtv_lock);
	if (flg_384 == 1) {
		flg_384 = 0;
		perf_unlock(&dtv_perf_lock_384);
		printk(KERN_DEBUG "[perflock] DTV perflock_384 disabled.\n");
	}
	if (flg_486 == 1) {
		flg_486 = 0;
		perf_unlock(&dtv_perf_lock_486);
		printk(KERN_DEBUG "[perflock] DTV perflock_486 disabled.\n");
	}
	mutex_unlock(&dtv_lock);
#endif

	return (0);
}

/**************************************************************************************************/
/**
	@brief	tuner_release
	@param	struct file		*file	[I]
	@param	unsigned int	cmd		[I]
	@param	unsigned long	arg		[I]
	@retval	0		Suncess
	@retval	!=0		Failed
*/
/**************************************************************************************************/
static long tuner_ioctl(struct file *file, unsigned int cmd, unsigned long arg)
{
	int ret;
	ioctl_cmd io_cmd_work;
	ioctl_cmd *io_cmd = &io_cmd_work;

	if (copy_from_user(io_cmd, (ioctl_cmd*)arg, sizeof(ioctl_cmd))) {
		printk("%s:%d  !!!tuner_ioctl copy_from_user error %08x\n", __FILE__, __LINE__, (unsigned int)arg);
		return -EFAULT;
	}

	switch ( cmd ) {
	case IOC_GPIO_VAL_SET:
		ret = gpio_set(io_cmd->id, io_cmd->val);
		if (ret != 0) {
			printk("%s:%d  !!!tuner_ioctl set error[cmd %x val %d][ret:%d]\n", __FILE__, __LINE__, io_cmd->id, io_cmd->val, ret);
			return -EINVAL;
		}
		break;
	case IOC_GPIO_VAL_GET:
		ret = gpio_get(io_cmd->id, &(io_cmd->val));
		if (ret != 0) {
			printk("%s:%d  !!!tuner_ioctl get error[cmd %x val %d][ret:%d]\n", __FILE__, __LINE__, io_cmd->id, io_cmd->val, ret);
			return -EINVAL;
		}
		if (copy_to_user((ioctl_cmd*)arg, io_cmd, sizeof(ioctl_cmd))) {
			printk("%s:%d  !!!tuner_ioctl copy_to_user error %08x\n", __FILE__, __LINE__, (unsigned int)arg);
			return -EFAULT;
		}
		break;
	case IOC_VREG_ENABLE:
		ret = tuner_vreg_enable();
		if (ret != 0) {
			printk("%s:%d  !!!tuner_ioctl set error[ret:%d]\n", __FILE__, __LINE__, ret);
			return -EINVAL;
		}
		break;
	case IOC_VREG_DISABLE:
		ret = tuner_vreg_disable();
		if (ret != 0) {
			printk("%s:%d  !!!tuner_ioctl set error[ret:%d]\n", __FILE__, __LINE__, ret);
			return -EINVAL;
		}
		break;
	case IOC_CLK_ENABLE:
		ret = tuner_clk_enable();
		if (ret != 0) {
			printk("%s:%d  !!!tuner_ioctl set error[ret:%d]\n", __FILE__, __LINE__, ret);
			return -EINVAL;
		}
		break;
	case IOC_CLK_DISABLE:
		ret = tuner_clk_disable();
		if (ret != 0) {
			printk("%s:%d  !!!tuner_ioctl set error[ret:%d]\n", __FILE__, __LINE__, ret);
			return -EINVAL;
		}
		break;
	case IOC_HW_REV_GET:
		ret = hw_revision_get(&(io_cmd->val));
		if (ret != 0) {
			printk("%s:%d  !!!tuner_ioctl get error[ret:%d]\n", __FILE__, __LINE__, ret);
			return -EINVAL;
		}
		if (copy_to_user((ioctl_cmd*)arg, io_cmd, sizeof(ioctl_cmd))) {
			printk("%s:%d  !!!tuner_ioctl copy_to_user error %08x\n", __FILE__, __LINE__, (unsigned int)arg);
			return -EFAULT;
		}
		break;
#if defined(CONFIG_MACH_ARS)
	case IOC_MSM_FREQ_SET:
		ret = tuner_msm_clock_set((unsigned char)io_cmd->id);
		if (ret != 0) {
			printk("%s:%d  !!!tuner_ioctl get error[ret:%d]\n", __FILE__, __LINE__, ret);
			return -EINVAL;
		}
		break;
#endif
	default:
		/* NOTE:  returning a fault code here could cause trouble
		 * in buggy userspace code.  Some old kernel bugs returned
		 * zero in this case, and userspace code might accidentally
		 * have depended on that bug.
		 */
		return -ENOTTY;
	}
	return 0;
}

static struct file_operations tuner_fops = {
	.owner				= THIS_MODULE,
	.unlocked_ioctl		= tuner_ioctl,
	.open				= tuner_open,
	.release			= tuner_release,
};

static struct miscdevice tuner_dev = {
	.minor = MISC_DYNAMIC_MINOR,
	.name = "tunctrl",
	.fops = &tuner_fops,
};

/**************************************************************************************************/
/**
	@brief	tuner_init
	@param	none
	@retval	0		Success
	@retval	!=0		Failed
*/
/**************************************************************************************************/
static int __init tuner_init(void)
{
	int ret, i ;
	int loop = sizeof(use_gpiono)/sizeof(stGPIO_DEF);

	ret = misc_register(&tuner_dev);
	if (ret) {
		printk("%s.%s.%d !!! fail to misc_register (MISC_DYNAMIC_MINOR)\n", __FILE__, __func__, __LINE__);
		return ret;
	}

	for(i=0; i<loop; i++){
		ret = gpio_request(use_gpiono[i].no , use_gpiono[i].label);
		if(ret < 0){
			printk(KERN_DEBUG "%s gpio_request() error : %d\n", use_gpiono[i].label, ret);
		}
	}

#if defined(CONFIG_MACH_ARS)
	mutex_init(&dtv_lock);
	perf_lock_init(&dtv_perf_lock_384, PERF_LOCK_384MHz, "DTV_384");
	perf_lock_init(&dtv_perf_lock_486, PERF_LOCK_486MHz, "DTV_486");
#endif

	return 0;
}

/**************************************************************************************************/
/**
	@brief	tuner_cleanup
	@param	none
	@retval	none
*/
/**************************************************************************************************/
static void __exit tuner_cleanup(void)
{
	int i ;
	int loop = sizeof(use_gpiono)/sizeof(stGPIO_DEF);

	for(i=0; i<loop; i++){
		gpio_free(use_gpiono[i].no);
	}

	misc_deregister(&tuner_dev);
}

/**************************************************************************************************/
/**
	@brief	gpio_init
	@param	none
	@retval	0	Success
	@retval	1	Failed
*/
/**************************************************************************************************/
static int gpio_init(void)
{
	int loop = sizeof(use_gpiono)/sizeof(stGPIO_DEF);
	int errcnt = 0;
	stGPIO_DEF *p = &use_gpiono[0];
	int i;
	
	for (i=0; i<loop; i++, p++) {
		if (p->direction == DirctionIn) {
			/* GPIO Input */
			if (gpio_direction_input(p->no) < 0) {
				/* Failed */
				errcnt ++;
				printk( "%s:%d gpio_direction_input error NO.%d \n", __FILE__,__LINE__, p->no);
				continue;
			}
			p->init_done = 1;
			continue;
		}
		if (p->direction == DirctionOut) {
			/* GPIO Output */
			if (gpio_direction_output(p->no, p->out_val) < 0) {
				/* Failed */
				errcnt ++;
				printk("%s: gpio_direction_output error NO.%d \n", __FILE__, p->no);
				continue;
			}
			p->init_done = 1;
			continue;
		}
	}

	if (errcnt != 0) {
		printk("%s: gpio_init error count %d\n", __FILE__, errcnt);
		return 1;
	}
	return 0;
}

static int tuner_clk_enable(void)
{
	unsigned 	gpio_cfg;
	int			ret;

	gpio_cfg = GPIO_CFG( CLOCK_CONTROL_PORTNO, 5, GPIO_CFG_OUTPUT, GPIO_CFG_NO_PULL, GPIO_CFG_6MA );	/* gp_clk_2b */

	ret = gpio_tlmm_config( gpio_cfg, GPIO_CFG_ENABLE );
	if ( ret < 0 ) {
		printk("%s: gpio_cfg error %d\n", __FILE__, ret);
		return 1;
	}

	gp_clk = clk_get( NULL, "gp2_clk" );
	clk_set_rate( gp_clk, 27000000 );
	clk_prepare( gp_clk );
	clk_enable( gp_clk );

//	writel(0xA00, MSM_CLK_CTL_BASE + 0x2D24 + 32*( 2 ));		/* gp_clk_2b */

	return 0;
}

static int tuner_clk_disable(void)
{
	clk_disable( gp_clk );
	clk_unprepare( gp_clk );
//	writel(0x0000, MSM_CLK_CTL_BASE + 0x2D24 + 32*( 2 ));		/* gp_clk_2b */

	return 0;
}
	

/**************************************************************************************************/
/**
	@brief	gpio_set
	@param	unsigned int	id			[I]	GPIO No
	@param	int				value		[I]	GPIO Value
	@retval	0		Success
	@retval	!=0		Failed
*/
/**************************************************************************************************/
static int gpio_set(unsigned int id, int value)
{
	int loop = sizeof(use_gpiono)/sizeof(stGPIO_DEF);
	stGPIO_DEF *p = use_gpiono;
	int flag = 0;
	int i;
	int portno ;

	for(i=0; i<loop; i++, p++){
		if (p->id == id){
			flag = 1;
			portno = p->no ;
			break;
		}
	}
	if (flag == 0) {
		printk("%s: !!! gpio_set() error No.%d value %d \n", __FILE__, id, value);
		return EINVAL;
	}
//	gpio_set_value(portno, value);
	gpio_set_value_cansleep(portno, value);
	return 0;
}

/**************************************************************************************************/
/**
	@brief	gpio_get
	@param	unsigned int	id			[I]	GPIO No
	@param	int				*val		[I]	GPIO Value
	@retval	0		Success
	@retval	!=0		Failed
*/
/**************************************************************************************************/
static int gpio_get(unsigned int id, int *val)
{
	int loop = sizeof(use_gpiono)/sizeof(stGPIO_DEF);
	stGPIO_DEF *p = &use_gpiono[0];
	int flag = 0;
	int i;
	int portno ;

	*val = 0;

	for(i=0; i<loop; i++, p++){
		if (p->id == id){
			flag = 1;
			portno = p->no ;
			break;
		}
	}
	if (flag == 0) {
		printk("%s: !!! gpio_get() No.%d error \n", __FILE__, id);
		return EINVAL;
	}
//	*val = gpio_get_value(portno);
	*val = gpio_get_value_cansleep(portno);

	return 0;
}

/**************************************************************************************************/
/**
	@brief	tuner_vreg_enable
	@param	none
	@retval	0		Success
	@retval	!=0		Failed
*/
/**************************************************************************************************/
static int tuner_vreg_enable(void)
{
	struct regulator *reg;
	struct device *dev = NULL;
	const char *id = "8921_l18";
	int min_uV = 1200000, max_uV = 1200000;

	reg= regulator_get(dev, id);
	if (IS_ERR(reg)) {
		printk("Unable to get %s regulator\n", id);
		return -1;
	}

    regulator_set_voltage(reg, min_uV, max_uV);

	if (!regulator_is_enabled(reg)) {
		regulator_enable(reg);
	}

	regulator_put(reg);

	return 0;
}

/**************************************************************************************************/
/**
	@brief	tuner_vreg_disable
	@param	none
	@retval	0		Success
	@retval	!=0		Failed
*/
/**************************************************************************************************/
static int tuner_vreg_disable(void)
{
	struct regulator *reg;
	struct device *dev = NULL;
	const char *id = "8921_l18";

	reg= regulator_get(dev, id);
	if (IS_ERR(reg)) {
		printk("Unable to get %s regulator\n", id);
		return -1;
	}

	if (regulator_is_enabled(reg)) {
		regulator_disable(reg);
	}

	regulator_put(reg);

	return 0;
}

/**************************************************************************************************/
/**
	@brief	hw_revision_get
	@param	int		   *val		[I]	sh_hw_revision value
	@retval	0		Success
	@retval	!=0		Failed
*/
/**************************************************************************************************/
static int hw_revision_get(unsigned int *val)
{
#if 0
	sharp_smem_common_type *sh_smem_common = NULL;

	sh_smem_common = (sharp_smem_common_type *)sh_smem_get_common_address();
	if (sh_smem_common != NULL) {
		*val = (unsigned int)sh_smem_common->sh_hw_revision;
	}
	else{
		return -1;
	}
#endif
	return 0;
}

#if defined(CONFIG_MACH_ARS)
/**************************************************************************************************/
/**
	@brief	hw_revision_get
	@param	int		  freq		[I]	clock freq flag
	@retval	0		Success
	@retval	!=0		Failed
*/
/**************************************************************************************************/
static int tuner_msm_clock_set(unsigned char freqFlag)
{
	if(freqFlag == 0){	/* set to 486MHz */
		mutex_lock(&dtv_lock);

		if(flg_384 == 1){
			flg_384 = 0;
			perf_unlock(&dtv_perf_lock_384);
			printk(KERN_DEBUG "[perflock] DTV perflock_384 disabled.\n");
		}

		if (flg_486 == 0) {
			perf_lock(&dtv_perf_lock_486);
			printk(KERN_DEBUG "[perflock] DTV perflock_486 enabled.\n");
			flg_486 = 1;
		}

		mutex_unlock(&dtv_lock);
	}
	else{	/* set to 384MHz */
		mutex_lock(&dtv_lock);

		if(flg_486 == 1){
			flg_486 = 0;
			perf_unlock(&dtv_perf_lock_486);
			printk(KERN_DEBUG "[perflock] DTV perflock_486 disabled.\n");
		}

		if (flg_384 == 0) {
			perf_lock(&dtv_perf_lock_384);
			printk(KERN_DEBUG "[perflock] DTV perflock enabled.\n");
			flg_384 = 1;
		}

		mutex_unlock(&dtv_lock);
	}
	
	return 0;
}
#endif

MODULE_LICENSE("GPL");
module_init(tuner_init);
module_exit(tuner_cleanup);
