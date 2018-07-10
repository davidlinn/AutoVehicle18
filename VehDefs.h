/*
 * Vehicle Definitions- priorities and other global defs
 *
 *  Created on: Jun 12, 2018
 *      Author: NetBurner
 */

#ifndef VEHDEFS_H_
#define VEHDEFS_H_

//Priorities
#define LCD_PRIO MAIN_PRIO+4
#define IMU_PRIO MAIN_PRIO+3
#define DRIVE_PRIO MAIN_PRIO+1
#define FTP_PRIO MAIN_PRIO-1
#define LOG_PRIO MAIN_PRIO-2
#define SPINNING_LIDAR_PRIO MAIN_PRIO-3

//HiResTimers (DMA Timers)
//if more DMA timers needed, I would try to rewrite servodrive.cpp using the PWM library
#define STEERING_TIMER 0 //Don't change or use by other funcs, tcr changes every clk
#define THROTTLE_TIMER 1 //Don't change or use by other funcs, tcr changes every clk
#define LEFTLIDAR_TIMER 2 //Timer Counter Register not reset: can be used by other funcs for measurement
#define RIGHTLIDAR_TIMER 3 //Timer Counter Register not reset: can be used by other funcs for measurement
#define DSM_TIMER 3 //Read current time only
#define IMU_TIMER 3 //Read current time only
#define GLOBAL_TIMER 3 //Read current time only

//UART Ports
#define LCD_PORT 5
#define RC_PORT 6
#define LIDAR_PORT 2 //Spinning LiDAR

#endif /* VEHDEFS_H_ */
