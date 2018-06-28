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
#include "Drivers/LidarPWM.h"
#include "Drivers/SpinningLidar.h"
#include "Drivers/pwm.h"
LOGFILEINFO;

extern "C" {
void UserMain(void * pd);
}
extern volatile bool bLog;

//GLOBAL VARIABLES
//Classes initialized with default constructor, set to do nothing.
//The parameterized constructors are later called in init().

OS_SEM MainTaskSem; //Semaphore: the function that has this gets to run
LCD lcd; //LCD object, reinstantiated in init()
Odometer odo; //Global odometer object, reinstantiated in init()
MPU9250 imu; //Global imu object, reinstantiated in init()
Nav nav; //Constructor does nothing

void init() {
	//Display
	Pins[38].function(PIN_38_UART5_TXD); //TX to Serial LCD
	lcd = LCD(LCD_PORT,9600); //Reinstantiates global lcd on serial port defined in VehDefs.h w/ baud rate 9600
	lcd.clear();

	//Side LiDARs: Do immediately after display because this also initializes DMA Timers
	// 2 & 3, which RC and IMU rely on
	Pins[23].function(PIN_23_T2IN); //LIDAR pulse left / Timer 2 in
	Pins[25].function(PIN_25_T3IN); //LIDAR pulse right / Timer 3 in
	LidarPWMInit(); //initializes both side LIDARS and global timers

	//Top LiDAR
	Pins[13].function(PIN_13_UART2_RXD);	//LIDAR RX
	Pins[16].function(PIN_16_UART2_TXD);	//LIDAR TX
	PinPWM(39, 25000, .58); //PWM at 25000 Hz with a 58% duty cycle
	//motor val provides about 180 samples per 360 deg scan- that means a sample every 2 degrees or so
	//OSTimeDly(TICKS_PER_SECOND);

	//RC
	Pins[14].function(PIN_14_UART6_RXD);	//RC RX
	InitDSM2Rx(RC_PORT); //Initialize RC on UART Port 6
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
	OSTimeDly(3); //delay 3 ticks to give IMU time to set up its registers
	IMUSetup();

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

void LCDUpdate(void*) {
	while (1) {
		lcd.clear();
		char buf[16];
		sprintf(buf,"Lidar:%10f",SpinningLidar::dist[0]);
		for (int i = 0; i<360; i+=15)
			printf("Degree %i: %10f, Quality: %i\n",i,SpinningLidar::dist[i],SpinningLidar::sampleQuality[i]);
			//printf("Degree %i: %10i, Quality: %i\n",i,SpinningLidar::prevDist[i],SpinningLidar::prevSampleQuality[i]);
		lcd.print(buf,16);
		OSTimeDly(TICKS_PER_SECOND/2); //delay .5s
	}
}

void Drive(void*) {
	while (1) {
		if (Utility::mode()==1) { //Fully manual
			SetServoPos(0,HiCon(rc_ch[1])); //Steer
			SetServoRaw(1,rc_ch[2]); //Throttle
		}
		else if (Utility::mode()==2) { //Autonomous steering, manual throttle
			SetServoPos(0,nav.getSteer()); //Steer
			SetServoRaw(1,rc_ch[2]); //Throttle
		}
		else { //Fully autonomous
			SetServoPos(0,nav.getSteer()); //Steer
			SetServoPos(1,nav.getThrottle()); //Throttle
		}
		nav.navUpdate();
		OSTimeDly(TICKS_PER_SECOND/10);
	}
}

const char * AppName="AutoVeh18";

void UserMain(void * pd) {
    EnableOSStackProtector(); // done TODO: Uncomment #define UCOS_STACKOVERFLOW in predef.h, rebuild libraries
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
    bLog = true;
    Logger::logBegin();

    //LCD Update Task
    OSSimpleTaskCreatewName(LCDUpdate,LCD_PRIO,"LCD Update");

    //Servo Vals Update Task
    OSSimpleTaskCreatewName(Drive,DRIVE_PRIO,"Drive");

    //Top LIDAR Task
    SpinningLidar::SpinningLidarInit(); //start top LIDAR serial read and processing task

    while (1) {
    	OSTimeDly(TICKS_PER_SECOND);
    }
}
