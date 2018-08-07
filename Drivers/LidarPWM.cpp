/*
 * LidarPWM.cpp: SIDE LIDAR
 *
 *  Created on: Jun 20, 2018
 *      Author: NetBurner
 */

#include "LidarPWM.h"
#include <ucos.h>
#include <cfinter.h>
#include <sim5441X.h>           /*on-chip register definitions*/
#include <predef.h>
#include <stdio.h>
#include <pins.h>
#include <pin_irq.h>
#include <intcdefs.h>
#include <HiResTimer.h>
#include "VehDefs.h"
#include "Profiler.h"
#include <constants.h>

volatile static uint8_t bLIDAR_MODE_LEFT;
volatile static uint32_t LIDAR_LAST_START_LEFT;
volatile uint32_t LIDAR_VALUE_LEFT;
volatile static int bLIDAR_MODE_RIGHT;
volatile static uint32_t LIDAR_LAST_START_RIGHT;
volatile uint32_t LIDAR_VALUE_RIGHT FAST_USER_VAR;

static int globalTimerOverflow = 0;
int getGlobalTimerOverflow() {return globalTimerOverflow;}

//Left Interrupt
/*INTERRUPT(LIDAR_ISR_LEFT,0x2700)
{
	sim2.timer[2].ter=3;
	if(bLIDAR_MODE_LEFT==1) //Just hit rising edge
		{
		sim2.timer[2].tmr=0x0083;   // 0000 0000 10 0 0 0 0 1 1 :Wait for Falling edge
		LIDAR_LAST_START_LEFT=sim2.timer[2].tcr;
		bLIDAR_MODE_LEFT=2;
		}
	else
		if(bLIDAR_MODE_LEFT==2) //Just hit falling edge
		{

			sim2.timer[2].tmr=0x0043;   // 0000 0000 01 0 0 0 0 1 1 :Wait for Rising edge
			bLIDAR_MODE_LEFT=1;
			uint32_t v=sim2.timer[2].tcr;
#ifndef SIM
			LIDAR_VALUE_LEFT=(v-LIDAR_LAST_START_LEFT);
#endif
		}
}*/

//Right Interrupt
INTERRUPT(LIDAR_ISR_RIGHT,0x2700) {
	sim2.timer[3].ter = 3;
	if (sim2.timer[3].tcn < LIDAR_LAST_START_RIGHT) {
		++globalTimerOverflow;
		LIDAR_LAST_START_RIGHT = 0;
		bLIDAR_MODE_RIGHT = 1;
		sim2.timer[3].tmr = 0x0053; //Rising edge
		return;
	}
	if (bLIDAR_MODE_RIGHT == 1) { //Just hit rising edge
		sim2.timer[3].tmr = 0x0093; // 0000 0000 10 0 0 0 0 1 1 //Falling edge
		LIDAR_LAST_START_RIGHT = sim2.timer[3].tcr;
		bLIDAR_MODE_RIGHT = 2;
	}
	else if (bLIDAR_MODE_RIGHT == 2) { //Just hit falling edge
		sim2.timer[3].tmr = 0x0053; // 0000 0000 01 0 0 0 0 1 1 //Rising edge
		bLIDAR_MODE_RIGHT = 1;
		uint32_t v = sim2.timer[3].tcr;
#ifndef SIM
		LIDAR_VALUE_RIGHT = (v - LIDAR_LAST_START_RIGHT); //Correctly wraps during overflows because unsigned int
#endif
	}
}

void LidarPWMInit() {
	//left
	//GlobalTimerInit(2);
	//SETUP_DMATIMER2_ISR(&LIDAR_ISR_LEFT,2);
	//bLIDAR_MODE_LEFT=1;
	//right
	SETUP_DMATIMER3_ISR(&LIDAR_ISR_RIGHT,2);
	GlobalTimerInit(3);
	bLIDAR_MODE_RIGHT=1;
}

void GlobalTimerInit(int timerNum) {
	printf("Initialized global timer %i\n", timerNum);
	sim2.timer[timerNum].txmr=0;
	sim2.timer[timerNum].ter=3;
	sim2.timer[timerNum].trr=0xFFFFFFFF;
	sim2.timer[timerNum].tcr=0;
	sim2.timer[timerNum].tcn=0;
	sim2.timer[timerNum].ter=3;        //           00 0 1 1 00 1 init_ticks tmr
	sim2.timer[timerNum].tmr=0x0053;   // 0000 0000 01 0 0 0 01 1 //Rising edge
}

int getLeftLidar() {
	//This calibration curve models the car as a point and assumes the LiDAR lens
	//is 4 in away from the point. Add 1 for each mm the lens is mounted further
	//from the center of the car. Likewise, subtract 1 for each mm the lens is mounted
	//closer to the center of the car.
	//Uncertainty is about 25 mm
	return (int)(LIDAR_VALUE_LEFT*.0079668)-40;
}

int getRightLidar() {
	return (int)(LIDAR_VALUE_RIGHT*.0079668)-40;
}
//3.97261
