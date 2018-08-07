/*
 * Logger.cpp: Uses introspec.h logging system to maintain a record of robot state for debugging purposes.
 * After log is produced, it is sent over FTP to a computer, where the "read" command-line tool decodes the
 * "Log.bin" binary into ASCII and organizes it into a .csv file. The .csv file can be read into Matlab as
 * column vectors, and a Matlab script forms "scenes," or robot states, and forms an animation of the robot's
 * navigation by stringing these scenes together.
 *  Created on: Jun 11, 2018
 *      Author: David Linn
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
#include "Map.h"

extern Odometer odo;
extern Nav nav;
extern Map map;
extern double getGlobalTime();

//Comment out values not to log- here and in logLoop()

//Object logged once per log cycle
START_INTRO_OBJ(MainLogObject,"Log")
double_element globalTime{"globalTime"};
float_element heading{"heading"};
int_element x{"x"};
int_element y{"y"};
float_element v{"v"};
//Nav
int_element finished{"finished"};
uint8_element navmode{"navmode"};
//LIDAR
int_element rightLidar{"rightLidar"};
END_INTRO_OBJ;

//Object logged (360/lidarResolution) times per log cycle
START_INTRO_OBJ(SpLidarLogObj,"SpLidar")
int_element splidar{"splidar"};
uint8_element splidararraystart{"splidararraystart"};
END_INTRO_OBJ;

//Object logged (map.numCircles) times per log cycle
START_INTRO_OBJ(CircleLogObj,"Circles")
float_element circlex{"circlex"};
float_element circley{"circley"};
float_element circleradius{"circleradius"};
uint8_element circlearraystart{"circlearraystart"};
END_INTRO_OBJ;

//Object logged (map.numSegments) times per log cycle
START_INTRO_OBJ(SegLogObj,"Segs")
float_element segb{"segb"};
float_element segm{"segm"};
float_element segx1{"segx1"};
float_element segx2{"segx2"};
uint8_element segarraystart{"segarraystart"};
END_INTRO_OBJ;

MainLogObject mainLog;
SpLidarLogObj spLidarLog;
CircleLogObj circleLog;
SegLogObj segLog;

void Logger::logLoop(void*) {
	//Comment out values not to log- here and in MainLogObject
	int lidarResolution = 10;
	while (1) {
		if (rc_ch[4]>1000) { //if Log is switched on
			//mainLog
			mainLog.globalTime=getGlobalTime();
			mainLog.heading=getHeading();
			mainLog.x=nav.getX();
			mainLog.y=nav.getY();
			mainLog.v=nav.getV();
			mainLog.finished=(int)nav.isFinished();
			mainLog.navmode = (int)nav.navMethod;
			mainLog.rightLidar=getRightLidar(); //distance in feet
			mainLog.Log();
			//spLidarLog
			spLidarLog.splidararraystart = 1;
			spLidarLog.splidar=SpinningLidar::dist[0];
			spLidarLog.Log();
			spLidarLog.splidararraystart = 0;
			for (int i = lidarResolution; i < 360; i += lidarResolution) {
				spLidarLog.splidar=SpinningLidar::dist[i];
				spLidarLog.Log();
			}
			//circleLog
			circleLog.circlearraystart = 1;
			if (map.numCircles>0) {
				circleLog.circlex=map.circleList[0].x;
				circleLog.circley=map.circleList[0].y;
				circleLog.circleradius=map.circleList[0].radius;
			}
			circleLog.Log();
			circleLog.circlearraystart = 0;
			for (int i = 1; i < map.numCircles; ++i) {
				circleLog.circlex=map.circleList[i].x;
				circleLog.circley=map.circleList[i].y;
				circleLog.circleradius=map.circleList[i].radius;
				circleLog.Log();
			}
			//segLog
			segLog.segarraystart = 1;
			if (map.numSegments>0) {
				segLog.segm=map.segmentList[0].m;
				segLog.segb=map.segmentList[0].b;
				segLog.segx1=map.segmentList[0].x1;
				segLog.segx2=map.segmentList[0].x2;
			}
			segLog.Log();
			segLog.segarraystart = 0;
			for (int i = 1; i < map.numSegments; ++i) {
				segLog.segm=map.segmentList[i].m;
				segLog.segb=map.segmentList[i].b;
				segLog.segx1=map.segmentList[i].x1;
				segLog.segx2=map.segmentList[i].x2;
				segLog.Log();
			}
		}
		OSTimeDly(TICKS_PER_SECOND/3); //Log at 3 Hz
	}
}

void Logger::logBegin() {
    LogFileVersions();
	OSSimpleTaskCreatewName(logLoop,LOG_PRIO,"Log");
}

void Logger::logWrite(char* buf) {
	LogMessage(buf);
}
