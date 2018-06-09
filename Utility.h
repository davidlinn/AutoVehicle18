/*
 * Utility.h
 *
 *  Created on: Jun 1, 2018
 *      Author: NetBurner
 */

#ifndef UTILITY_H_
#define UTILITY_H_

namespace Utility {
	int mode();
	void countdown(int secs, int x, int y);
	int inchesToOdo(int inches);
	int I2CScan(bool *discovered);
	float degreeWrap(float deg); //returns a degree val between -180 and 180
}


#endif
