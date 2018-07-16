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

//UART Ports
#define LCD_PORT 5
#define RC_PORT 6
#define LIDAR_PORT 2 //Spinning LiDAR

#define SPINNING_LIDAR_LOG_DEF float_element spinningLidar0{"spinningLidar0"};\
float_element spinningLidar10{"spinningLidar10"};\
float_element spinningLidar20{"spinningLidar20"};\
float_element spinningLidar30{"spinningLidar30"};\
float_element spinningLidar40{"spinningLidar40"};\
float_element spinningLidar50{"spinningLidar50"};\
float_element spinningLidar60{"spinningLidar60"};\
float_element spinningLidar70{"spinningLidar70"};\
float_element spinningLidar80{"spinningLidar80"};\
float_element spinningLidar90{"spinningLidar90"};\
float_element spinningLidar100{"spinningLidar100"};\
float_element spinningLidar110{"spinningLidar110"};\
float_element spinningLidar120{"spinningLidar120"};\
float_element spinningLidar130{"spinningLidar130"};\
float_element spinningLidar140{"spinningLidar140"};\
float_element spinningLidar150{"spinningLidar150"};\
float_element spinningLidar160{"spinningLidar160"};\
float_element spinningLidar170{"spinningLidar170"};\
float_element spinningLidar180{"spinningLidar180"};\
float_element spinningLidar190{"spinningLidar190"};\
float_element spinningLidar200{"spinningLidar200"};\
float_element spinningLidar210{"spinningLidar210"};\
float_element spinningLidar220{"spinningLidar220"};\
float_element spinningLidar230{"spinningLidar230"};\
float_element spinningLidar240{"spinningLidar240"};\
float_element spinningLidar250{"spinningLidar250"};\
float_element spinningLidar260{"spinningLidar260"};\
float_element spinningLidar270{"spinningLidar270"};\
float_element spinningLidar280{"spinningLidar280"};\
float_element spinningLidar290{"spinningLidar290"};\
float_element spinningLidar300{"spinningLidar300"};\
float_element spinningLidar310{"spinningLidar310"};\
float_element spinningLidar320{"spinningLidar320"};\
float_element spinningLidar330{"spinningLidar330"};\
float_element spinningLidar340{"spinningLidar340"};\
float_element spinningLidar350{"spinningLidar350"};

#define SPINNING_LIDAR_LOG_UPDATE mainLog.spinningLidar0=SpinningLidar::dist[0];\
			mainLog.spinningLidar10=SpinningLidar::dist[10];\
			mainLog.spinningLidar20=SpinningLidar::dist[20];\
			mainLog.spinningLidar30=SpinningLidar::dist[30];\
			mainLog.spinningLidar40=SpinningLidar::dist[40];\
			mainLog.spinningLidar50=SpinningLidar::dist[50];\
			mainLog.spinningLidar60=SpinningLidar::dist[60];\
			mainLog.spinningLidar70=SpinningLidar::dist[70];\
			mainLog.spinningLidar80=SpinningLidar::dist[80];\
			mainLog.spinningLidar90=SpinningLidar::dist[90];\
			mainLog.spinningLidar100=SpinningLidar::dist[100];\
			mainLog.spinningLidar110=SpinningLidar::dist[110];\
			mainLog.spinningLidar120=SpinningLidar::dist[120];\
			mainLog.spinningLidar130=SpinningLidar::dist[130];\
			mainLog.spinningLidar140=SpinningLidar::dist[140];\
			mainLog.spinningLidar150=SpinningLidar::dist[150];\
			mainLog.spinningLidar160=SpinningLidar::dist[160];\
			mainLog.spinningLidar170=SpinningLidar::dist[170];\
			mainLog.spinningLidar180=SpinningLidar::dist[180];\
			mainLog.spinningLidar190=SpinningLidar::dist[190];\
			mainLog.spinningLidar200=SpinningLidar::dist[200];\
			mainLog.spinningLidar210=SpinningLidar::dist[210];\
			mainLog.spinningLidar220=SpinningLidar::dist[220];\
			mainLog.spinningLidar230=SpinningLidar::dist[230];\
			mainLog.spinningLidar240=SpinningLidar::dist[240];\
			mainLog.spinningLidar250=SpinningLidar::dist[250];\
			mainLog.spinningLidar260=SpinningLidar::dist[260];\
			mainLog.spinningLidar270=SpinningLidar::dist[270];\
			mainLog.spinningLidar280=SpinningLidar::dist[280];\
			mainLog.spinningLidar290=SpinningLidar::dist[290];\
			mainLog.spinningLidar300=SpinningLidar::dist[300];\
			mainLog.spinningLidar310=SpinningLidar::dist[310];\
			mainLog.spinningLidar320=SpinningLidar::dist[320];\
			mainLog.spinningLidar330=SpinningLidar::dist[330];\
			mainLog.spinningLidar340=SpinningLidar::dist[340];\
			mainLog.spinningLidar350=SpinningLidar::dist[350];

#endif /* VEHDEFS_H_ */
