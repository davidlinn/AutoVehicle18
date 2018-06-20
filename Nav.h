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
	double getX();
	double getY();
	void navUpdate();

private:
	double x; //estimated X,Y from odometer+IMU readings
	double y;
	double lastHeading;
	uint32_t lastOdo;
};

#endif /* NAV_H_ */
