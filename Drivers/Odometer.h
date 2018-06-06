/*
 * Odometer.h
 *
 *  Created on: Jun 1, 2018
 *      Author: NetBurner
 */

#ifndef ODOMETER_H_
#define ODOMETER_H_

#include <basictypes.h>

class Odometer {
public:
	Odometer();
	Odometer(int pin);
	virtual ~Odometer();
	uint32_t getCount();
	void resetCount();

private:
	static void OdoIrq();
};

#endif /* ODOMETER_H_ */
