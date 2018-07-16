/*
 * Nav.cpp
 *
 *  Created on: Jun 11, 2018
 *      Author: NetBurner
 */

#include "Nav.h"
#include "Drivers/Odometer.h"
#include "Drivers/IMUSetupAndSample.h"
#include "Utility.h"
#include <math.h>
#include "Drivers/servodrive.h"
#include <stdio.h>
#include "VehDefs.h"
#include <HiResTimer.h>
#include <predef.h>
#include <ucos.h>
#include <constants.h>
#include <SimpleAD.h>
#include "Drivers/SpinningLidar.h"
#include "Drivers/LidarPWM.h"

#define K_P_STEER .01
#define K_P_WALLFOLLOW (1./40.)
#define BRAKE_PWR -.5 //more neg equals more pwr

extern Odometer odo;

Nav::Nav() {}
Nav::~Nav() {}
float Nav::getX() {return x;}
float Nav::getY() {return y;}
float Nav::getHeadDes() {return heading_des;}
float Nav::getV() {return v;}
bool Nav::isFinished() {return finished;}
float Nav::getRightWallEst() {return rightWallEst;}
float Nav::getHeadError() {return headError;}

//MAIN NAVIGATION METHODS

void Nav::navUpdate() {
	if (Utility::switchVal(GetADResult(1))==-1) { //if first switch is in the left position
		x = 0; y = 0; //reset coordinates
		rightWallEst = getRightLidar();
		prevRightWallEst = rightWallEst; //reset walls
		finished = false; //not finished
		return;
	}
	//Record time since last update
	deltat = navtimer->readTime()-lastnavupdate;
	lastnavupdate = navtimer->readTime();
	//Save previous coordinates
	lastx = x;
	lasty = y;
	//Update coordinates using odometer difference and heading
	double headingAverage = (getHeading()+lastHeading)/2;
	double feetTraveled = Utility::odoToFeet(odo.getCount()-lastOdo);
	if (getServoPos(1)>-.005) { //moving forward
		x += feetTraveled*cos(headingAverage*M_PI/180.);
		y += feetTraveled*sin(headingAverage*M_PI/180.);
	}
	else { //moving backward
		x -= feetTraveled*cos(headingAverage*M_PI/180.);
		y -= feetTraveled*sin(headingAverage*M_PI/180.);
	}
	//Calculate velocity
	lastv = v;
	v = feetTraveled/deltat; //feet per second
	//Save current heading and odometer
	lastHeading = getHeading();
	lastOdo = odo.getCount();
	//Calculate desired heading
	heading_des = (180./M_PI)*atan2(y_des-y,x_des-x);

	//FOR WALL FOLLOWING
	//Update walls and error in heading if we moved
	if (feetTraveled > 0) {
		diff = wallUpdate(); //difference in estimates in feet
		headError = (180./M_PI)*asin(diff/feetTraveled); //in degrees
	}

}

float Nav::getSteer() { //1 left, -1 right. getSteer() must also provide a value for forward
	switch (navMethod) {
	case simpleWaypoint: return waypointSteer();
	case followRightWall: return followRightWallSteer();
	case followPath: return followPathSteer();
	}
	return 0;
}

float Nav::getThrottle() {
	switch (navMethod) {
	case simpleWaypoint: return moveUntilFinished(.145,-.5);
	case followRightWall: return moveUntilFinished(.145,-.5);
	case followPath: return moveUntilFinished(.145,-.5);
	}
	return 0;
}

//STEERING-RELATED METHODS

float Nav::waypointSteer() {
	float error = Utility::degreeWrap(heading_des-lastHeading);
	if (error>90 || error<-90) {
		forward = -1;
		error = Utility::degreeWrap(error-180); //calculate error as if car was flipped
	}
	else forward = 1;
	return forward*K_P_STEER*error;
}

float Nav::followRightWallSteer() {
	forward = 1;
	if (rightWallEst>1 && fabs(diff) < 1 && !finished) //if estimate is not wildly different from previous
		return K_P_WALLFOLLOW*headError;
	else {
		finished = 1;
		return 0;
	}
}

float Nav::wallUpdate() { //updates wall estimates and returns difference between current and previous estimate in feet
	float d1 = getRightLidar();
	float d2 = SpinningLidar::dist[90];
	int d2Quality = SpinningLidar::sampleQuality[90];
	prevRightWallEst = rightWallEst;
	if (d1 > 0) { //if good reading from solid state lidar
		if (d2 > 0 && d2Quality > 0) rightWallEst = (d1+d2)/2;
		else rightWallEst = d1;
	}
	else rightWallEst = 0;
	return Utility::cmToFt(prevRightWallEst-rightWallEst);
}

float Nav::followPathSteer() {
	return 0;
}

//THROTTLE-RELATED METHODS

float Nav::moveUntilFinished(float throttle, float brake) {
	if (!finished) {
		if (getServoPos(1)>0 && (forward==-1)) //switch directions from forward to backward
			brakeAndZero(brake);
		if (waypointFinishCheck(brake)) return 0;
		else return forward*throttle-.035; //backwards needs more power than forwards
	}
	return 0;
}

int Nav::waypointFinishCheck(float brake) {
	float errorsq = (x-x_des)*(x-x_des) + (y-y_des)*(y-y_des); //won't be negative
	if (errorsq < 1) {
		finished = true;
		if (getServoPos(1)>0 && (forward==1)) {
			SetServoPos(1,brake); //brake (only works if moving forward)
			OSTimeDly(TICKS_PER_SECOND/2);
		}
		SetServoPos(1,0); //Reset throttle to zero
		return 1;
	}
	else return 0;
}

void Nav::brakeAndZero(float brake) { //Only call if moving forward
	SetServoPos(1,brake); //brake
	OSTimeDly(TICKS_PER_SECOND/2);
	SetServoPos(1,0); //Electronic speed controller needs to record a zero
	OSTimeDly(TICKS_PER_SECOND/2);
}
