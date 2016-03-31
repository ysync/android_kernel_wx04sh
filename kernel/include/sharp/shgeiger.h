/*
 * Copyright (C) 2010 SHARP CORPORATION All rights reserved.
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
 * Definitions for Shgeiger chip.
 */
#ifndef SHGEIGER_H
#define SHGEIGER_H

#include <linux/ioctl.h>

#define SH_GEIGER_RESULT_FAILURE    -1
#define SH_GEIGER_RESULT_SUCCESS    0

#define SH_GEIGER_I2C_DEVNAME	"SH_GEIGER"
#define SH_GEIGER_ASIC_I2C_SLAVE		0x48
#define SH_GEIGER_IRQ		35
#define SH_GEIGER_EN		34
#define SH_HIGH_SENS		0x62
#define SH_LOW_SENS			0x70

/* Shgeiger register address */
#define CHIPID					0x00
#define COUNT_CLEAR				0x01
#define COMMAND1				0x02
#define GMBIST					0x03
#define ATOM_THRESHOLD_LSB		0x04
#define ATOM_THRESHOLD_MSB		0x05
#define THERMAL_THRESHOLD_HITH	0x06
#define THERMAL_THRESHOLD_LOW	0x07
#define SLOT_TIME				0x08
#define COMMAND2				0x09
#define EN_CP					0x0B

#define NUM_TIME_SLOT_LSB		0x16
#define NUM_TIME_SLOT_MSB		0x17
#define NUM_ATOM_LSB			0x18
#define NUM_ATOM_MSB			0x19
#define NUM_DOC_LSB				0x1A
#define NUM_DOC_MSB				0x1B
#define THERMAL_VALUE			0x1C

enum {
	ALARM_ALREADY_LOCKED = 0,
	ALARM_NOT_LOCKED = 1,
	ALARM_UNLOCK = 2,
	ALARM_UNLOCK_SUCCESS = 3,
	ALARM_UNLOCK_FAILED = 4,
	ALARM_ERROR = -1,
	ALARM_UNLOCK_ERROR = -2,
};

enum {
	THERMAL_ALREADY_LOCKED = 0,
	THERMAL_NOT_LOCKED = 1,
	THERMAL_HIGH_UNLOCK = 2,
	THERMAL_NOLMAL_UNLOCK = 3,
	THERMAL_UNLOCK_SUCCESS = 4,
	THERMAL_UNLOCK_FAILED = 5,
	THERMAL_ERROR = -1,
	THERMAL_UNLOCK_ERROR= -2,
};

enum {
	CLOSE = 	0xFF + 1,
	ACTIVE =	0xFF + 2,
	SHUTDOWN = 	0xFF + 3,
	STOP = 		0xFF + 4,
	UNNOWN = 	-1,
};


struct shgeiger_calibration_data {
	double bg;
	double multi;
	unsigned int multi_data;
	char bg_msb;
	char bg_lsb;
	unsigned int thermal_a;
	unsigned int thermal_b;
};

struct shgeiger_count_data {
	unsigned int num_time_slot;
	unsigned int num_atom;
	unsigned int num_dc;
	int thermal_data;
	unsigned long int time_msec;
	unsigned long int stoptime_msec;
	unsigned long long int total_time_nsec;
	unsigned long long int total_stoptime_nsec;
};

struct shgeiger_thermal_threshold {
	int assert_threshold;
	int negate_threshold;
};

struct shgeiger_bistdata {
	unsigned int vip_atom;
	unsigned int vim_atom;
};

enum {
	LOW_SENSITIVITY_MODE = 0,
	HIGH_SENSITIVITY_MODE,
	NUM_SENSITIVITY_MODE,
};

#define SHGEIGERIO						'g'
#define SHGEIGER_IOCTL_ACTIVE				_IO(SHGEIGERIO, 0x01)
#define SHGEIGER_IOCTL_SHUTDOWN				_IO(SHGEIGERIO, 0x02)
#define SHGEIGER_IOCTL_START				_IO(SHGEIGERIO, 0x03)
#define SHGEIGER_IOCTL_STOP					_IO(SHGEIGERIO, 0x04)
#define SHGEIGER_IOCTL_CALIBRATIONREAD		_IOR(SHGEIGERIO, 0x05, struct shgeiger_calibration_data)
#define SHGEIGER_IOCTL_TIMESLOT				_IOW(SHGEIGERIO, 0x06, unsigned int)
#define SHGEIGER_IOCTL_READCOUNTDATA		_IOR(SHGEIGERIO, 0x07, struct shgeiger_count_data)
#define SHGEIGER_IOCTL_CLEAR				_IO(SHGEIGERIO, 0x08)
#define SHGEIGER_IOCTL_THERMAL_START		_IO(SHGEIGERIO, 0x09)
#define SHGEIGER_IOCTL_THERMAL_STOP			_IO(SHGEIGERIO, 0x0A)
#define SHGEIGER_IOCTL_THERMALREAD			_IOR(SHGEIGERIO, 0x0B, int)
#define SHGEIGER_IOCTL_COUNTTHRESHOLD		_IOW(SHGEIGERIO, 0x0C, unsigned int)
#define SHGEIGER_IOCTL_THERMALTHRESHOLD		_IOW(SHGEIGERIO, 0x0D, struct shgeiger_thermal_threshold )
#define SHGEIGER_IOCTL_CHIPVERREAD			_IOR(SHGEIGERIO, 0x0E, char)
#define SHGEIGER_IOCTL_ALARMWAIT			_IOR(SHGEIGERIO, 0x0F, int)
#define SHGEIGER_IOCTL_ALARMWAIT_UNLOCK		_IOR(SHGEIGERIO, 0x10, int)
#define SHGEIGER_IOCTL_THERMALWAIT			_IOR(SHGEIGERIO, 0x11, int)
#define SHGEIGER_IOCTL_THERMALWAIT_UNLOCK	_IOR(SHGEIGERIO, 0x12, int)
#define SHGEIGER_IOCTL_GMBIST				_IOR(SHGEIGERIO, 0x13, struct shgeiger_bistdata)
#define SHGEIGER_IOCTL_IDLE_WAKE_LOCK		_IO(SHGEIGERIO, 0x14)
#define SHGEIGER_IOCTL_IDLE_WAKE_UNLOCK		_IO(SHGEIGERIO, 0x15)
#define SHGEIGER_IOCTL_SENSITIVITY_CTL		_IOW(SHGEIGERIO, 0x16, unsigned char)
#define SHGEIGER_IOCTL_STATECHECK			_IOR(SHGEIGERIO, 0x17, int)


void ShGeigerPause_Vib( void );
void ShGeigerReStart_Vib( void );
void ShGeigerPause_Spk( void );
void ShGeigerReStart_Spk( void );
void ShGeigerShutdown_Dtv( void );
void ShGeigerActive_Dtv( void );
void ShGeigerPause_Ir( void );
void ShGeigerReStart_Ir( void );


#endif /* SHGEIGER_H */
