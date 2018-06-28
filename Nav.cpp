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
//#include "Drivers/SpinningLidar.h"

extern Odometer odo;

Nav::Nav() {

}

Nav::~Nav() {

}

double Nav::getX() {
	return x;
}

double Nav::getY() {
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
}

double Nav::getSteer() {
	return 0;
}

double Nav::getThrottle() {
	return 0;
}
