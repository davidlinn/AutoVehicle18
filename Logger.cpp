/*
 * Logger.cpp
 *
 *  Created on: Jun 11, 2018
 *      Author: NetBurner
 */

#include "Logger.h"
#include "Drivers/IMUSetupAndSample.h"
#include "Drivers/Odometer.h"
#include "Drivers/servodrive.h"
#include "Nav.h"
#include "Drivers/introspec.h"
#include "Drivers/dsm2.h"
#include <ucosmcfc.h>
#include <ucos.h>
#include <constants.h>
#include "VehDefs.h"
#include "Drivers/SpinningLidar.h"
#include "Drivers/LidarPWM.h"
#include <c++/5.2.0/string>

extern Odometer odo;
extern Nav nav;

//Comment out values not to log- here and in logLoop()
START_INTRO_OBJ(MainLogObject,"Log")
double_element steering{"steering"};
double_element throttle{"throttle"};
float_element heading{"heading"};
float_element x{"x"};
float_element y{"y"};
float_element v{"v"};
int_element finished{"finished"};
float_element rightLidar{"rightLidar"};
SPINNING_LIDAR_LOG_DEF
END_INTRO_OBJ;

MainLogObject mainLog;

void Logger::logLoop(void*) {
	//Comment out values not to log- here and in MainLogObject
	while (1) {
		if (rc_ch[4]>1000) { //if Log is switched on
			USER_ENTER_CRITICAL()
			mainLog.steering=getServoPos(0);
			mainLog.throttle=getServoPos(1);
			mainLog.heading=getHeading();
			mainLog.x=nav.getX();
			mainLog.y=nav.getY();
			mainLog.v=nav.getV();
			mainLog.finished=(int)nav.isFinished();
			mainLog.rightLidar=getRightLidar(); //distance in feet
			SPINNING_LIDAR_LOG_UPDATE
			USER_EXIT_CRITICAL()
			mainLog.Log();
		}
		OSTimeDly(TICKS_PER_SECOND/5); //Log at 5 Hz
	}
}

void Logger::logBegin() {
    LogFileVersions();
	OSSimpleTaskCreatewName(logLoop,LOG_PRIO,"Log");
}

void Logger::logWrite(char* buf) {
	LogMessage(buf);
}
