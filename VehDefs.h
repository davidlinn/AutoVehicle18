/*
 * Vehicle Definitions- priorities and other global defs
 *
 *  Created on: Jun 12, 2018
 *      Author: NetBurner
 */

#ifndef VEHDEFS_H_
#define VEHDEFS_H_

//Priorities
#define IMU_PRIO MAIN_PRIO+1
#define LOG_PRIO MAIN_PRIO-2
#define FTP_PRIO MAIN_PRIO-1

//HiResTimers (DMA Timers)
//if more DMA timers needed, I would try to rewrite servodrive.cpp using the PWM library
#define STEERING_TIMER 0
#define THROTTLE_TIMER 1
#define IMU_TIMER 3
#define UPDATE_TIMER 2

#endif /* VEHDEFS_H_ */
