/*
 * Nav.h
 *
 *  Created on: Jun 11, 2018
 *      Author: NetBurner
 */

#ifndef NAV_H_
#define NAV_H_

class Nav { //defines the robot's position in the map and plans motion
public:
	Nav();
	virtual ~Nav();
	double getX();
	double getY();

private:
	double x; //estimated X,Y from odometer+IMU readings
	double y;
};

#endif /* NAV_H_ */
