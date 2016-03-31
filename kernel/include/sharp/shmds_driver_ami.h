/* include/sharp/shmds_driver_ami.h  (MotionSensor Driver)
 *
 * Copyright (C) 2012 SHARP CORPORATION
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

#ifndef SHMDS_AMI_H
#define SHMDS_AMI_H

enum{
    MS_POSITION_OPEN = 0,
    MS_POSITION_CLOSE,
    MS_POSITION_SWIVEL,
    MS_POSITION_CYCLOID,
};

extern void SHMDS_Pedometer_ReStart(void);
extern void SHMDS_Pedometer_Pause(void);
extern void SHMDS_SetFlipInformation(unsigned char position);

#endif
