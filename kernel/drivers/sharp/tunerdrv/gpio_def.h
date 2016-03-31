/**************************************************************************************************/
/** 
	@file		gpio_def.h
	@brief		GPIO Definition Header
*/
/**************************************************************************************************/

#ifndef GPIO_DEF
	#define GPIO_DEF

/* LINUX/android/kernel/arch/arm/mach-msm/board-msm8960.h */
#include <mach/gpio.h>
#define PM8921_GPIO_BASE		NR_GPIO_IRQS
#define PM8921_GPIO_PM_TO_SYS(pm_gpio)  (pm_gpio - 1 + PM8921_GPIO_BASE)

#include "gpio_def_ext.h"

typedef struct GPIO_DEF {
	unsigned int id;		/* GPIO Number (ID) */
	unsigned int no;		/* GPIO PORT Number */
	int direction;			/* I/O Direction */
	int out_val;			/* Initialized Value */
	int init_done;			/* GPIO Initialized ? 1:Complete (Don't Care) */
	char *label;			/* labels may be useful for diagnostics */
} stGPIO_DEF;

#define DirctionIn (0)
#define DirctionOut (1)

/* #define GPIO_DTVEN_PORTNO		(xx) */
#define GPIO_DTVRST_PORTNO			(PM8921_GPIO_PM_TO_SYS(36))
/* #define GPIO_DTVLNAEN_PORTNO		(xx) */
/* #define GPIO_DTVANTSW_PORTNO		(PM8921_GPIO_PM_TO_SYS(15)) */
/* #define GPIO_DTVMANTSL_PORTNO		(PM8921_GPIO_PM_TO_SYS(16)) */
/* #define GPIO_DTVUANTSL_PORTNO		(PM8921_GPIO_PM_TO_SYS(17)) */
/* #define GPIO_DTVCANTSL_PORTNO		(PM8921_GPIO_PM_TO_SYS(18)) */
#define GPIO_DTVHWREV_PORTNO		(31)

//#define CLOCK_CONTROL_PORTNO	(3)
#define CLOCK_CONTROL_PORTNO	(37)

#endif	//GPIO_DEF
