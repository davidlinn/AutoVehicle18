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
#include <math.h>
#include "VehDefs.h"
#include <HiResTimer.h>
#include <predef.h>
#include <ucos.h>
#include <constants.h>

//#include "Drivers/SpinningLidar.h"

#define K_P_STEER .01
#define V_DES .11

extern Odometer odo;

Nav::Nav() {}
Nav::~Nav() {}
float Nav::getX() {return x;}
float Nav::getY() {return y;}
float Nav::getHeadDes() {return heading_des;}
float Nav::getV() {return v;}
bool Nav::isFinished() {return finished;}

void Nav::navUpdate() {
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
	v = sqrt(pow(x-lastx,2)+pow(y-lasty,2));
	//Save current heading and odometer
	lastHeading = getHeading();
	lastOdo = odo.getCount();
	//Calculate desired heading
	heading_des = (180./M_PI)*atan2(y_des-y,x_des-x);
}

float Nav::getSteer() { //1 left, -1 right
	float error = Utility::degreeWrap(heading_des-lastHeading);
	if (error>90 || error<-90) {
		forward = -1;
		error = Utility::degreeWrap(error-180); //calculate error as if car was flipped
	}
	else forward = 1;
	return forward*K_P_STEER*error;
}

float Nav::getThrottle() {
	if (!finished) {
		if (getServoPos(1)>0 && !forward) { //switch directions from forward to backward
			SetServoPos(1,-.8); //brake
			OSTimeDly(TICKS_PER_SECOND/2);
			SetServoPos(1,0); //Electronic speed controller needs to record a zero
			OSTimeDly(TICKS_PER_SECOND/2);
		}

		float errorsq = (x-x_des)*(x-x_des) + (y-y_des)*(y-y_des); //won't be negative
		if (errorsq < 1) {
			finished = true;
			if (forward) {
				SetServoPos(1,-.8); //brake (only works if moving forward)
				OSTimeDly(TICKS_PER_SECOND/2);
			}
			SetServoPos(1,0); //Reset throttle to zero
			return 0;
		}
		else return forward*.145-.035; //backwards needs more power than forwards
	}
	return 0;
}
