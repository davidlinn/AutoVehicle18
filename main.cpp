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
#include "tests.h"
#include "Utility.h"
#include "Drivers/Odometer.h"
#include "Drivers/IMU.h"
#include <multichanneli2c.h>

extern "C" {
void UserMain(void * pd);
}

//GLOBAL VARIABLES
//Classes initialized with default constructor, set to do nothing.
//The parameterized constructors are later called in init().

OS_SEM MainTaskSem; //Semaphore: the function that has this gets to run
LCD lcd; //LCD object, reinstantiated in init()
Odometer odo; //Global odometer object, reinstantiated in init()
IMU imu;

// ---Code for Odometer---

//---------------------------

void init() {
	//RC
	Pins[14].function(PIN_14_UART6_RXD);	//RC RX
	InitDSM2Rx(6); //Initialize RC on Port 6
	iprintf("Initialized RC\n");

	//Drive
	Pins[19].function(PIN_19_T0OUT);	   //Steer servo
	Pins[21].function(PIN_21_T1OUT);	   //Throttle Servo
	ServoDriveInit();
	iprintf("Initialized steering\n");

	//Odometer
	odo = Odometer(49); //creates Odometer object on pin 49

	//I2C (communication used for IMU)
	MultiChannel_I2CInit();

	//IMU
	Pins[27].function(PIN_27_I2C0_SCL    );//I2C for IMU
	Pins[29].function(PIN_29_I2C0_SDA    );//I2C For IMU
	Pins[50].function(PIN_50_IRQ2  ); //IRQ for IMU
	imu = IMU(0x68, MAIN_PRIO-4); //the MPU-9250 IMU has I2C address 0x68

	//LIDAR
	Pins[13].function(PIN_13_UART2_RXD);	//LIDAR RX
	Pins[16].function(PIN_16_UART2_TXD);	//LIDAR TX
	Pins[25].function(PIN_25_T3IN); //LIDAR pulse

	//Display
	Pins[38].function(PIN_38_UART5_TXD); //TX to Serial LCD
	lcd = LCD(5,9600); //Reinstantiates global lcd on serial port 5 w/ baud rate 9600
	lcd.clear();
}

const char * AppName="AutoVeh18";

void UserMain(void * pd) {
    EnableOSStackProtector(); // TODO: Uncomment #define UCOS_STACKOVERFLOW in predef.h, rebuild libraries
    InitializeStack();
    GetDHCPAddressIfNecessary();
    OSChangePrio(MAIN_PRIO);
    EnableAutoUpdate();
    StartHTTP();
    EnableTaskMonitor();

    #ifndef _DEBUG
    EnableSmartTraps();
    #endif

    #ifdef _DEBUG
    InitializeNetworkGDB_and_Wait();
    #endif

    iprintf("Application started\n");
    init();

    SetServoPos(1,0);    //Ensure that Electronic Speed Controller
	OSTimeDly(TICKS_PER_SECOND*5);  //reads a stall for five seconds
	iprintf("Initialized throttle\n");

    while (1) {
    	imu.printRaw();
    	OSTimeDly(TICKS_PER_SECOND);
    }
}
