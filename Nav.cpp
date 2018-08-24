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
#include "utils.h"
#include "Profiler.h"
#include "FastMath.h"
#include <stdlib.h>

#define K_P_WALLFOLLOW .02
#define BRAKE_PWR -.5 //more neg equals more pwr
#define WAYPOINT_SUCCESS_RADIUS_SQ 250000 //50 cm

extern Odometer odo;
extern Map map;
extern double getGlobalTime();

Nav::Nav() {}
Nav::~Nav() {}
int Nav::getX() {return x;}
int Nav::getY() {return y;}
float Nav::getHeadDes() {return heading_des;}
float Nav::getV() {return v;}
bool Nav::isFinished() {return finished;}
int Nav::getRightWallEst() {return rightWallEst;}
float Nav::getHeadError() {return headError;}

//MAIN NAVIGATION METHODS

void Nav::navUpdate() {
	//The following lines assume that the ADC is updated every so often by calling StartAD(): we do it in LCDUpdate()
	if (Utility::switchVal(GetADResult(1))==-1) { //if first switch is in the left position
		x = 0; y = 0; //reset coordinates
		rightWallEst = getRightLidar();
		prevRightWallEst = rightWallEst; //reset walls
		finished = false; //not finished
		return;
	}
	else if (Utility::switchVal(GetADResult(1))==1) //if first switch is in the right position
		zeroHeading(); //zero out the heading (globally)
	//Record time since last update
	currenttime = getGlobalTime();
	deltat = currenttime-lastnavupdate;
	lastnavupdate = currenttime;
	//Save previous coordinates
	lastx = x;
	lasty = y;
	//Update coordinates using odometer difference and heading
	double headingAverage = (getHeading()+lastHeading)/2;
	int mmTraveled = Utility::odoToMM(odo.getCount()-lastOdo);
	if (getServoPos(1)>-.005) { //moving forward
		x += mmTraveled*fastcos(headingAverage);
		y += mmTraveled*fastsin(headingAverage);
	}
	else { //moving backward
		x -= mmTraveled*fastcos(headingAverage);
		y -= mmTraveled*fastsin(headingAverage);
	}
	//Calculate velocity
	lastv = v;
	v = mmTraveled/(1000*deltat); //m/s
	//Save current heading and odometer
	lastHeading = getHeading();
	lastOdo = odo.getCount();
	//Calculate desired heading simply before adding heading adjustment algorithms
	if (navMethod==simpleWaypoint)
		heading_des = (180./M_PI)*atan2(y_des-y,x_des-x);
	//FOR WALL FOLLOWING
	//Update walls and error in heading if we moved
	if (mmTraveled > 0 && navMethod==followRightWall) {
		wallUpdate();
		heading_des = lastHeading+(180./M_PI)*asin(((float)diff)/mmTraveled); //in degrees
	}
	//Greedy Heading Algorithm
	cocHDesAdjust();
}

float Nav::getSteer() { //1 left, -1 right. getSteer() must also provide a value for forward
	switch (navMethod) {
	case simpleWaypoint: return headingSteer();
	case followRightWall: return headingSteer();
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

float Nav::headingSteer() {
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
	if (rightWallEst>150 && fabs(diff) < 300 && !finished) //if estimate is not wildly different from previous
		return K_P_WALLFOLLOW*headError;
	else {
		finished = 1;
		return 0;
	}
}

//updates estimates of distance to physical or virtual wall, attempting to filter erroneous wall estimates from other obstacles
void Nav::wallUpdate() {
	prevRightWallEst = rightWallEst;
	rightWallEst = SpinningLidar::dist[90];
	diff = prevRightWallEst-rightWallEst; //positive if getting closer to right wall
}

float Nav::followPathSteer() {
	return 0;
}

float Nav::artificialPotential() {
	int SOURCEMAG = 20;
	int x0,y0;
	int mag;
	int dir;
	for (int i = 0; i < 360; i+=2) {
		if (SpinningLidar::dist[i] > 1) {
			dir = Utility::degreeWrap(-i-90+lastHeading); //direction of "force"
			mag = SpinningLidar::dist[i]; // 1/r^2
			mag = SpinningLidar::sampleQuality[i]/(mag*mag);
			x0 += mag*fastcos(dir);
			y0 += mag*fastsin(dir);
		}
	}
	x0 += SOURCEMAG*fastcos(heading_des);
	y0 += SOURCEMAG*fastsin(heading_des);
	return (180./M_PI)*atan2(y0,x0); //in degrees
}

void Nav::greedyHeadDesAdjust() {
	if (objectInPath(heading_des,2,3000)) { //if there's something at that heading, adjust heading greedily
		int closestPath = closestFreeSector(heading_des,2,3000);
		if (closestPath == -1000) {
			closestPath = closestFreeSector(heading_des,2,2000);
			if (closestPath == -1000) {
				closestPath = closestFreeSector(heading_des,3,700);
				if (closestPath == -1000) {
					closestPath = Utility::Zto360Wrap(heading_des+180); //give up and finding a path and try to reverse
				}
			}
		}
		heading_des = closestPath;
		K_P_STEER = .03;
	}
	else K_P_STEER = .02;
}

void Nav::cocHDesAdjust() {
	//Populate circle of concern
	int head = heading_des; //convert heading to integer
	circleOfConcern[0] = SpinningLidar::dist[Utility::Zto360Wrap(head)];
	for (int i = 1; i < CIR_CONCERN_SIZE-1; i+=2) {
		circleOfConcern[i] = SpinningLidar::dist[Utility::Zto360Wrap(head+i)]; //left
		circleOfConcern[i+1] = SpinningLidar::dist[Utility::Zto360Wrap(head-i)]; //right
	}
	//Update concern level member variable
	updateConcernLvl();
	//Now that we know which level in our circle of concern we need to work in, let's set heading_des to the closest
	//free arc. If there's no free arc found within 90 degrees, give up and keep going toward heading_des until concernLvl 0
	switch (concernLvl) {
		case 4: K_P_STEER = .02; return; //end heading adjustment if there's no concern
		case 3: K_P_STEER = .025; heading_des = closestFreeSector(head, 2, 4000); break; //otherwise, adjust heading_des based on concernLvl
		case 2: K_P_STEER = .03; heading_des = closestFreeSector(head, 4, 2290); break;
		case 1: K_P_STEER = .035; heading_des = closestFreeSector(head, 6, 1520); break;
		default: K_P_STEER = .04; heading_des = Utility::degreeWrap(head+180); break; //case 0: just reverse
	}
}

//THROTTLE-RELATED METHODS

float Nav::moveUntilFinished(float throttle, float brake) {
	//When assigning power, note that backwards needs more power than forwards
	if (!finished) {
		if (getServoPos(1)>.01 && forward==-1) //switch directions from forward to backward
			brakeAndZero(brake, .3, .3); //brake for .3s, zero for .3s to allow speed controller to accept a reverse signal
		if (waypointFinishCheck(brake)) return 0;
		else {
			if (throttleStackSize > 0) { //always return from the stack if there's anything on there
				return forward*throttleStack[--throttleStackSize]-.035;
			}
			else if (concernLvl == 0) { //if an object is in our immediate circle of concern, two brakes, a zero, and a go OR one zero and a go
				throttleStack[throttleStackSize++] = .145;
				if (getServoPos(1)>.01) { //if currently moving forward
					throttleStack[throttleStackSize++] = .035; //zero
					throttleStack[throttleStackSize++] = brake;
					return brake;
				}
				else return 0;
			}
			else if (concernLvl == 1) { //if an object is in the primary circle of concern, one zero and a go
				throttleStack[throttleStackSize++] = .145;
				return 0;
			}
			else if (concernLvl == 2) { //secondary circle of concern: one zero and a go
				throttleStack[throttleStackSize++] = .145;
				return 0;
			}
			else if (concernLvl == 3) { //tertiary circle of concern
				return forward*.145-.035;
			}
			else return forward*.165-.035; //no concern
		}
	}
	return 0;
}

int Nav::waypointFinishCheck(float brake) {
	int errorsq = (x-x_des)*(x-x_des) + (y-y_des)*(y-y_des); //won't be negative
	if (errorsq < WAYPOINT_SUCCESS_RADIUS_SQ) {
		if (numWaypts == 0) { //if no more waypoints on the stack
			finished = true;
			if (getServoPos(1)>.01 && (forward==1)) {
				SetServoPos(1,brake); //brake for .33 secs (only works if moving forward)
				OSTimeDly(TICKS_PER_SECOND/3);
			}
			SetServoPos(1,0); //Reset throttle to zero
			return 1;
		}
		else { //else pop a waypoint off the stack
			--numWaypts;
			x_des = x_des_stack[numWaypts];
			y_des = y_des_stack[numWaypts];
		}
	}
	return 0;
}

void Nav::brakeAndZero(float brake, float brakeSecs, float zeroSecs) {
	if (getServoPos(1)>.01) { //if speed controller is currently being sent a forward value
		SetServoPos(1,brake); //brake for .33 secs
		OSTimeDly(TICKS_PER_SECOND*brakeSecs);
	}
	SetServoPos(1,0); //Electronic speed controller should record a zero
	OSTimeDly(TICKS_PER_SECOND*zeroSecs);
}

bool Nav::objectInPath(int heading, int degreeSpread, int distance) {
	int head = getHeading();
	for (int i = head-heading-degreeSpread; i < head-heading+degreeSpread; ++i) {
		int dir = Utility::Zto360Wrap(i);
		int mag = SpinningLidar::dist[dir];
		if (SpinningLidar::sampleQuality[dir]>1 && mag>1 && mag<distance)
			return true;
	}
	return false;
}

int Nav::closestFreeSector(int startingDegree, int halfArc, int r) { //assumes startingDegree path is bad
	int arc = 2*halfArc; //total arc in degrees
	int sum, mag, dir;
	int head = getHeading();
	int leftIndex = head-startingDegree; //set left and right indices corresponding to a lidar value
	int rightIndex = leftIndex;
	while ((leftIndex < head-startingDegree+90) || (rightIndex > head-startingDegree-90)) {
		//check left path
		dir = Utility::Zto360Wrap(++leftIndex);
		mag = SpinningLidar::dist[dir];
		sum = 0;
		while (mag > r || mag < 1) { //good if lidar sees no obstructions
			if (++sum > arc) return Utility::degreeWrap(head-leftIndex-halfArc);
			dir = Utility::Zto360Wrap(++leftIndex);
			mag = SpinningLidar::dist[dir];
		}
		//check right path
		dir = Utility::Zto360Wrap(--rightIndex);
		mag = SpinningLidar::dist[dir];
		sum = 0;
		while (mag > r || mag < 1) { //good if lidar sees no obstructions
			if (++sum > arc) return Utility::degreeWrap(head-rightIndex+halfArc);
			dir = Utility::Zto360Wrap(--rightIndex);
			mag = SpinningLidar::dist[dir];
		}
	}
	return startingDegree; //if there's no free path within 90 degrees, give up and just return startingDegree
}

void Nav::updateConcernLvl() {
	//Lvl 0
	for (int i = 0; i < CIR_CONCERN_SIZE; ++i) { //23 degree arc
		if (circleOfConcern[i] > 1 && circleOfConcern[i] < 800) { //80 cm
			concernLvl = 0;
			return;
		}
	}
	//Lvl 1
	for (int i = 0; i < 13; ++i) { //13 degree arc
		if (circleOfConcern[i] > 1 && circleOfConcern[i] < 1520) { //152 cm
			concernLvl = 1;
			return;
		}
	}
	//Lvl 2
	for (int i = 0; i < 9; ++i) { //9 degree arc
		if (circleOfConcern[i] > 1 && circleOfConcern[i] < 2290) { //229 cm
			concernLvl = 2;
			return;
		}
	}
	//Lvl 3
	for (int i = 0; i < 5; ++i) { //5 degree arc
		if (circleOfConcern[i] > 1 && circleOfConcern[i] < 4000) { //400 cm
			concernLvl = 3;
			return;
		}
	}
	concernLvl = 4;
	return;
}
