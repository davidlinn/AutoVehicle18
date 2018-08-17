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
#include "VehDefs.h"
#include "Path.h"
#include "Map.h"

#define CIR_CONCERN_SIZE 23

class Nav { //defines the robot's position in the map and plans motion
public:
	Nav();
	virtual ~Nav();
//Getter Methods
	int getX(); //in mm
	int getY();
	float getV();
	float getHeadDes();
	bool isFinished();
	int getRightWallEst(); //in mm
	float getHeadError();
//Main Navigation Calculations
	void navUpdate();
	float getSteer();
	float getThrottle();
//Steer-related
	float headingSteer(); //returns steer val
	float followRightWallSteer(); //returns steer val
	float wallUpdate(); //updates wall estimates and returns difference between ests in ft
	float followPathSteer(); //returns steer val
	float aStarSteer(); //returns steer val
	void greedyHeadDesAdjust(); //if necessary, adds an intermediate waypoint to steer around an obstacle, updates waypoints and heading_des
//Throttle-related
	float moveUntilFinished(float throttle, float brake); //returns steer val
	void brakeAndZero(float brake, float brakeSecs, float zeroSecs); //applies brake for .5s and then zeros throttle, only call if car is moving forward!
	int waypointFinishCheck(float brake); //returns 1 if finished with waypoint navigation, 0 otherwise
//Algorithms
	Path shortestPath(int x, int y); //returns the shortest path to a waypoint using A*
		//returns the heading that follows path of least potential
	float artificialPotentialWithMap(); //using map
	float artificialPotential(); //using one lidar scan
	bool objectInPath(int heading, int degreeSpread, int distance); //returns true if lidar at heading_des+-degreeSpread sees an object closer than distance

	//Adjusts heading_des based on circles of concern- smart greedy algorithm
	void cocHDesAdjust();
	//Updates concernLvl member var, assumes circleOfConcern already populated
	void updateConcernLvl();
	//Returns the heading that corresponds to the free sector of arc size 2*halfArc degrees and radius r closest to startingDegree.
	//If no sector found within 90 degrees, returns startingDegree flipped 180 degrees
	int closestFreeSector(int startingDegree, int halfArc, int r);

//State
	enum NavMethod {
		simpleWaypoint, //navigates to a waypoint by minimizing error between heading and heading_des
		followRightWall, //follows wall by minimizing the change in distance from it, stops when wall ends
		//followLeftWall,
		//followWalls,
		//createMap, //manual navigation, builds map
		//rememberPath, //manual navigation, creates a repeatable path by recording coordinates, heading, and throttle/steer vals
		followPath, //repeats path by minimizing deviance from recorded path while recognizing obstacles
	};
	NavMethod navMethod;

private:
//Position
	int x; //estimated X,Y from odometer+IMU readings in MILLIMETERS
	int y;
	uint32_t lastOdo;
	int lastx;
	int lasty;
//Velocity
	float v;
	float lastv;
	int distTraveled;
//Time
	double currenttime;
	double lastnavupdate;
	double deltat;
//Orientation
	float lastHeading;
	float headError; //for wall following
//Waypoints, Desired Quantities
	int x_des = 8000; //current waypoint
	int y_des = 0;
	int x_des_stack[32]; //stack holding future waypoints
	int y_des_stack[32];
	unsigned int numWaypts = 0; //number of waypoints on stack, 0 if no waypoints on stack
	float heading_des;
	Path path;
//State
	bool finished = false;
	int forward = 1; //1 is forward, -1 is backward
	bool obstacleInPath = false;
	int rightWallEst; //in mm
	int prevRightWallEst;
	int diff;
	float K_P_STEER = .02;
	int concernLvl = 4;
	int circleOfConcern[CIR_CONCERN_SIZE]; //holds lidar readings 0,359,1,358,2,etc. Size is biggest circle of concern
	float throttleStack[5]; //holds future throttle values; if throttleStackIndex != -1, throttleStack[throttleStackIndex] gets returned
	int throttleStackSize = 0;
};

#endif /* NAV_H_ */
