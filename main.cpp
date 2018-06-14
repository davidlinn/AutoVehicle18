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
#include "Drivers/IMUSetupAndSample.h"
#include <multichanneli2c.h>
#include "Drivers/MPU9250.h"
#include "Logger.h"
#include "Nav.h"
#include "Drivers/introspec.h"
#include "VehDefs.h"
#include <HiResTimer.h>
LOGFILEINFO;

extern "C" {
void UserMain(void * pd);
}

//GLOBAL VARIABLES
//Classes initialized with default constructor, set to do nothing.
//The parameterized constructors are later called in init().

OS_SEM MainTaskSem; //Semaphore: the function that has this gets to run
LCD lcd; //LCD object, reinstantiated in init()
Odometer odo; //Global odometer object, reinstantiated in init()
MPU9250 imu; //Global imu object, reinstantiated in init()
Nav nav;

void motorValUpdate() {
	//printf("\nRC Ch1: %lu, RC Ch2: %lu", rc_ch[1], rc_ch[2]);
	SetServoPos(0,HiCon(rc_ch[1])); //Steer
    //SetServoRaw(0,rc_ch[1]);
	SetServoRaw(1,rc_ch[2]); //Throttle
}

void init() {
	//Display
	Pins[38].function(PIN_38_UART5_TXD); //TX to Serial LCD
	lcd = LCD(5,9600); //Reinstantiates global lcd on serial port 5 w/ baud rate 9600
	lcd.clear();

	//RC
	Pins[14].function(PIN_14_UART6_RXD);	//RC RX
	InitDSM2Rx(6); //Initialize RC on Port 6
	iprintf("Initialized RC\n");

	//Odometer
	odo = Odometer(49); //creates Odometer object on pin 49

	//I2C (communication used for IMU)
	MultiChannel_I2CInit();

	//IMU
	Pins[27].function(PIN_27_I2C0_SCL    );//I2C for IMU
	Pins[29].function(PIN_29_I2C0_SDA    );//I2C For IMU
	Pins[50].hiz();
	Pins[50].function(PIN_50_IRQ2  ); //IRQ for IMU
	imu = MPU9250(50); //reconstruct MPU9250 object on interrupt pin 50 (interrupt pin goes high when new data available)
	IMUSetup();

	//LIDAR
	Pins[13].function(PIN_13_UART2_RXD);	//LIDAR RX
	Pins[16].function(PIN_16_UART2_TXD);	//LIDAR TX
	Pins[25].function(PIN_25_T3IN); //LIDAR pulse

	//FTP Server for Log Transmission
	InitLogFtp(FTP_PRIO);

	//Drive
	Pins[19].function(PIN_19_T0OUT);	   //Steer servo
	Pins[21].function(PIN_21_T1OUT);	   //Throttle Servo
	ServoDriveInit();
	iprintf("Initializing steering and throttle\n");
	SetServoPos(0,0);
	SetServoPos(1,0);    //Ensure that Electronic Speed Controller reads a stall for five seconds
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
    IMURun();

    HiResTimer* motorValUpdateTimer = HiResTimer::getHiResTimer(UPDATE_TIMER);
    motorValUpdateTimer->init(.02);
    motorValUpdateTimer->setInterruptFunction(motorValUpdate);
    motorValUpdateTimer->start();

    while (1) {
    	OSTimeDly(TICKS_PER_SECOND);
    }
}
