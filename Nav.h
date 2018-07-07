/*
 * Nav.h
 *
 *  Created on: Jun 11, 2018
 *      Author: NetBurner
 */

#ifndef NAV_H_
#define NAV_H_

#include <basictypes.h>

class Nav { //defines the robot's position in the map and plans motion
public:
	Nav();
	virtual ~Nav();
	float getX();
	float getY();
	void navUpdate();
	float getSteer();
	float getThrottle();
	float getHeadDes();

private:
	float x; //estimated X,Y from odometer+IMU readings in FEET
	float y;
	float lastHeading;
	uint32_t lastOdo;
	float x_des = 30;
	float y_des = 0;
	float heading_des;
};

#endif /* NAV_H_ */
