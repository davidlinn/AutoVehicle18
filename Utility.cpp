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
#include <multichanneli2c.h>

extern LCD lcd;

namespace Utility {

	int mode() {
		//(1=139-539,2=824-1224,3=1508-1908)
		//1 = manual, 2=semi-auto, 3=full auto
		if (rc_ch[5] < 700) return 1;
		else if (rc_ch[5] > 1400)	return 3;
		else return 2;
	}

	void countdown(int secs, int x, int y) { //display countdown info, secs is an integer between 1 and 99
		while (secs>0) {
			//Setup to print one or two digits
			bool lessThanTen = (secs < 10);
			lcd.moveCursor(x,y);
			char buf[2-lessThanTen];
			if (lessThanTen) sprintf(buf, "0%i", secs);
			else sprintf(buf, "%i", secs);
			lcd.print(buf);
			--secs;
			OSTimeDly(TICKS_PER_SECOND);
		}
	}

	double feetToOdo(double feet) {
		return feet*12./4.6249;
	}

	double odoToFeet(uint32_t odo) {
		//y = 4.6249x
		return 4.6249*((double)odo)/12.;
	}

	int odoToMM(uint32_t odo) {
		return 117*odo;
	}

	/*
	 * Scans the I2C bus for connected devices. The variable 'slaveAddress' is used
	 * to revert this device's address to after the scan is complete. The function
	 * returns the quantity of devices discovered on the bus.
	 */
	int I2CScan(bool *discovered)
	{
	    int count = 0;

	    /* The I2C bus speed on the 5270 processor is set by a divider of the internal
	     * clock frequency of 147.5MHz / 2 = 73.75MHz. The maximum I2C bus speed is
	     * 100KHz. 73.75MHz/100KHz = 737.5. Referring to the I2C freq divider table
	     * in the Freescale manual the closest divider is 768 (register value = 0x39).
	     * 73.75MHz/768 = 95,703 bps.
	     */

	    // Run the I2c initialization routine in case it hasn't already been run
	    I2CInit();              // init to the default address of 0x08


	    for (int x = 1; x < 0x80; x++)
	    {
	        int result = I2CStart(x, I2C_START_WRITE, 1);
	        if (result < I2C_TIMEOUT)
	        {
	            discovered[x] = true;
	            I2CStop();
	            count++;
	            iprintf("Found device at address 0x%02x \r\n", x);
	        }
	        else
	        {
	            I2CStop();
	        	I2CResetPeripheral();
	        }
	    }

	    iprintf("Found %d devices on the bus! \r\n", count);

	    return count;
	}

	float degreeWrap(float deg) {
		while (deg>180)
			deg-=360;
		while (deg<-180)
			deg+=360;
		return deg;
	}

	int switchVal(int ADCCount) {
		if (ADCCount<8000)
			return -1;
		else if (ADCCount<24000)
			return 0;
		else
			return 1;
	}

	float cmToFt(float cm) {
		return .0328084*cm;
	}

	float Zto360Wrap(float deg) {
		while (deg < 0) deg += 360;
		while (deg >= 360) deg -= 360;
		return deg;
	}

	int Zto360Wrap(int deg) {
		while (deg < 0) deg += 360;
		while (deg >= 360) deg -= 360;
		return deg;
	}
}
