/* linux/sharp/hkdkc210_touch_mt.h */

#ifndef __SHTPS_NASSAU_TS2_001_H__
#define __SHTPS_NASSAU_TS2_001_H__

#define	SPI_TOUCH_NAME				"SH_touchpanel"
#define SH_TOUCH_DEVNAME			SPI_TOUCH_NAME
#define SH_TOUCH_IF_DEVNAME 		"shtpsif"
#define SH_TOUCH_IF_DEVPATH 		"/dev/shtpsif"

#define SHTPS_NASSAU_GPIO_RST		16
#define SHTPS_NASSAU_GPIO_IRQ		106
#define SHTPS_NASSAU_GPIO_PWR		33
#define SHTPS_NASSAU_GPIO_OSC		32

////#define SHTPS_NASSAU_SPI_CLOCK	2000000
//#define SHTPS_NASSAU_SPI_CLOCK	600000
#define SHTPS_NASSAU_SPI_CLOCK		4000000

struct shtps_nassau_touch_platform_data {
	int (*setup)(struct device *);
	void (*teardown)(struct device *);
	int (*v17_power)(int on);

	int		gpio_irq_pin;
	int		gpio_reset_pin;	
	int		gpio_power_pin;
	int		gpio_oscstart_pin;

	u16		x_min, x_max;
	u16		y_min, y_max;
};

/* -----------------------------------------------------------------------------------
 */
extern void msm_tps_setsleep(int on);

#endif /* __SHTPS_NASSAU_TS2_001_H__ */
