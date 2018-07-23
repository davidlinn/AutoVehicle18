/* SIDE LIDAR
 * LidarPWM.h - These functions use a hardware timer to measure the length of pulses recieved
 *  from the popular LiDAR Lite v3 and convert them to cm via a calibration curve.
 *  Code can be reused to read the length of any high pulse to sub-ms precision.
 *  Created on: Jun 20, 2018
 *      Author: NetBurner- Paul Breed, David Linn
 */

#ifndef LIDARPWM_H_
#define LIDARPWM_H_

#include <basictypes.h>
#include <constants.h>

//initiates constant lidar data gathering via PWM, timer2=left lidar, timer3=right lidar
//calls GlobalTimerInit for both timers
void LidarPWMInit();
void LidarPWMInitOld();
double getLeftLidar(); //returns distance in cm
double getRightLidar(); //returns distance in cm
int getGlobalTimerOverflow();
void GlobalTimerInit(int timerNum);

#endif /* LIDARPWM_H_ */
