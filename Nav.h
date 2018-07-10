/*
 * Nav.h
 *
 *  Created on: Jun 11, 2018
 *      Author: NetBurner
 */

#ifndef NAV_H_
#define NAV_H_

#include <basictypes.h>
#include <HiResTimer.h>
#include <VehDefs.h>

class Nav { //defines the robot's position in the map and plans motion
public:
	Nav();
	virtual ~Nav();
	//Getter Methods
	float getX();
	float getY();
	float getV();
	float getHeadDes();
	bool isFinished();
	//Navigation Calculations
	void navUpdate();
	float getSteer();
	float getThrottle();

private:
	//Position
	float x; //estimated X,Y from odometer+IMU readings in FEET
	float y;
	uint32_t lastOdo;
	float lastx;
	float lasty;
	//Velocity
	float v;
	float lastv;
	//Time
	HiResTimer* navtimer = HiResTimer::getHiResTimer(GLOBAL_TIMER);
	double lastnavupdate;
	double deltat;
	//Orientation
	float lastHeading;
	//Waypoints, Desired Quantities
	float x_des = 20;
	float y_des = 0;
	float heading_des;
	//State
	bool finished = false;
	int forward = 1;
};

#endif /* NAV_H_ */
