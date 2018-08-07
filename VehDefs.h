/*
 * Vehicle Definitions- priorities and other global defs
 *
 *  Created on: Jun 12, 2018
 *      Author: NetBurner
 */

#ifndef VEHDEFS_H_
#define VEHDEFS_H_

//Priorities: There may be up to 9 user tasks, prios 46-55.
#define LCD_PRIO 55
#define MAP_PRIO 54
#define IMU_PRIO 53
#define LOG_PRIO 52
#define SPINNING_LIDAR_PRIO 51
//MAIN_PRIO 50
#define DRIVE_PRIO 49
#define FTP_PRIO 48

//HiResTimers (DMA Timers)
//if more DMA timers needed, I would try to rewrite servodrive.cpp using the PWM library
#define STEERING_TIMER 0 //Don't change or use by other funcs, tcr changes every clk
#define THROTTLE_TIMER 1 //Don't change or use by other funcs, tcr changes every clk
#define LEFTLIDAR_TIMER 2 //Timer Counter Register not reset: can be used by other funcs for measurement
#define RIGHTLIDAR_TIMER 3 //Timer Counter Register not reset: can be used by other funcs for measurement
#define DSM_TIMER 3 //Read current time only
#define IMU_TIMER 3 //Read current time only
#define GLOBAL_TIMER 3 //Read current time only

//PIT Timers
#define IMU_PIT_TIMER 1
#define SP_LIDAR_PIT_TIMER 2

//UART Ports
#define LCD_PORT 5
#define RC_PORT 6
#define LIDAR_PORT 2 //Spinning LiDAR

#endif /* VEHDEFS_H_ */
