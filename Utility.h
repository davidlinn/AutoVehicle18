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
	double feetToOdo(double feet);
	double odoToFeet(uint32_t odo);
	int I2CScan(bool *discovered);
	float degreeWrap(float deg); //returns a degree val between -180 and 180

	//Switches: From viewpoint of car, most forward switch is ch3, green button in back is ch0
	//Left position produces ADC count of 0, middle in 16000s, right in 32000s
	//Depressed green button produces ADC count of 0, undepressed in 32000s
	//Returns -1 if left or depressed, 0 if middle, 1 if right or undepressed
	int switchVal(int ADCCount);

	float cmToFt(float cm);
}


#endif
