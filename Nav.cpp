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
//#include "Drivers/SpinningLidar.h"

#define K_P .01

extern Odometer odo;

Nav::Nav() {

}

Nav::~Nav() {

}

float Nav::getX() {
	return x;
}

float Nav::getY() {
	return y;
}

void Nav::navUpdate() { //called every odometer tick
	double headingAverage = (getHeading()+lastHeading)/2;
	//double inchesTraveled = .21622; //inches traveled in a odo tick
	double feetTraveled = Utility::odoToFeet(odo.getCount()-lastOdo);
	if (getServoPos(1)>-.1) { //moving forward
		x += feetTraveled*cos(headingAverage*M_PI/180.);
		y += feetTraveled*sin(headingAverage*M_PI/180.);
	}
	else { //reverse
		x -= feetTraveled*cos(headingAverage*M_PI/180.);
		y -= feetTraveled*sin(headingAverage*M_PI/180.);
	}
	lastHeading = getHeading();
	lastOdo = odo.getCount();
	heading_des = (180./M_PI)*atan2(y_des-y,x_des-x);
}

float Nav::getSteer() { //1 left, -1 right
	float error = Utility::degreeWrap(heading_des-lastHeading);
	return K_P*error;
}

float Nav::getThrottle() {
	return 0;
}

float Nav::getHeadDes() {
	return heading_des;
}
