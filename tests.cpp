

#include <predef.h>
#include <stdio.h>
#include <ctype.h>
#include <startnet.h>
#include <autoupdate.h>
#include <dhcpclient.h>
#include <smarttrap.h>
#include <taskmon.h>
#include <NetworkDebug.h>
#include <pins.h>
#include <pin_irq.h>
#include <serial.h>
#include <sim5441X.h>           /*on-chip register definitions*/
#include "Drivers/dsm2.h"
#include "Drivers/servodrive.h"
#include "Drivers/lcd.h"
#include "Utility.h"
#include <Drivers/Odometer.h>
#include "tests.h"

extern LCD lcd;

namespace tests {
	void odoCalibration() {
		Odometer testOdo(49); //Create an odometer for this test
		int trialNum = 1;
		while (trialNum <= 5 && Utility::mode()==3) { //repeat for 5 trials as long as we stay in autonomous mode
			while (testOdo.getCount() < (uint32_t)10*trialNum ) {
				//Run Motor
				SetServoPos(0,0); //Steer straight
				SetServoPos(1,.11); //Move forward slowly
				OSTimeDly(1);
			}
			//Stop motor and wait 30 secs to measure distance and reposition vehicle
			SetServoPos(1,0);
			OSTimeDly(3*TICKS_PER_SECOND);
			//Print odometer to LCD
			char buf[9];
			sprintf(buf,"Odo: %lu",testOdo.getCount());
			lcd.print(buf);
			//Delay
			OSTimeDly(12*TICKS_PER_SECOND); //measure during this time
			lcd.clear();
			Utility::countdown(15,0,0); //reposition vehicle during this time
			lcd.clear();
			testOdo.resetCount();
			++trialNum;
		}
	}

	void moveForwardXInches(int x) {
		Odometer odo(49);
		int stopOdoCount = Utility::inchesToOdo(x);
		while ((int)odo.getCount() <= stopOdoCount-10)
			SetServoPos(1,.11);
		SetServoPos(1,0);
	}

}
