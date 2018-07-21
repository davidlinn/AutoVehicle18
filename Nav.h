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
	float getRightWallEst();
	float getHeadError();
//Main Navigation Calculations
	void navUpdate();
	float getSteer();
	float getThrottle();
//Steer-related
	float headingSteer();
	float followRightWallSteer();
	float wallUpdate(); //updates wall estimates and returns difference between ests in ft
	float followPathSteer();
//Throttle-related
	float moveUntilFinished(float throttle, float brake);
	void brakeAndZero(float brake);
	int waypointFinishCheck(float brake);
//Algorithms
	Path shortestPath(int x, int y); //returns the shortest path to a waypoint using A*
		//returns the heading that follows path of least potential
	float artificialPotentialWithMap(); //using map
	float artificialPotential(); //using one lidar scan
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
	Map map;

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
	float distTraveled;
//Time
	HiResTimer* navtimer = HiResTimer::getHiResTimer(GLOBAL_TIMER);
	double lastnavupdate;
	double deltat;
//Orientation
	float lastHeading;
	float headError; //for wall following
//Waypoints, Desired Quantities
	float x_des = 20;
	float y_des = 0;
	float heading_des;
	Path path;
//State
	bool finished = false;
	int forward = 1; //1 is forward, -1 is backward
	bool obstacleInPath = false;
	float rightWallEst; //in cm
	float prevRightWallEst;
	float diff;
};

#endif /* NAV_H_ */
