/*
 * Odometer.cpp
 *
 *  Created on: Jun 1, 2018
 *      Author: NetBurner
 */

#include "Odometer.h"
#include <pin_irq.h>
#include <stdio.h>
#include <basictypes.h>
#include <sim5441X.h>
#include "Nav.h"

static volatile uint32_t OdoCount;

extern Nav nav;

Odometer::Odometer()
{
}

Odometer::Odometer(int pin)
{
	// Initialize all ints to 0 automatically
	bool initSuccess = SetPinIrq(pin,-1,OdoIrq); //Set up pin as a hardware interrupt
									//Runs OdoIrq on negative edge
	if (initSuccess) printf("Initialized odometer\n");
	else printf("Failed to initialize odometer\n");
}

Odometer::~Odometer() {
	// TODO Auto-generated destructor stub
}

uint32_t Odometer::getCount() {
	return OdoCount;
}

void Odometer::resetCount() {
	OdoCount = 0;
}

void Odometer::OdoIrq() {
	OdoCount++;
}
