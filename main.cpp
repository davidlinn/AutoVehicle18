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
#include <SimpleAD.h>
#include "Boot_Settings.h"
#include <math.h>
#include "Profiler.h"
#include <constants.h>
#include "FastMath.h"
#include <sim.h>
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
Path mainPath;
Map map;
NV_SettingsStruct NV_Settings;

//Global Time
double getGlobalTime() {
	return ((double) TIME_PER_CLK * ( // How long is a clock cycle?
			(double) sim2.timer[3].tcn + (  // The count of the current DMA count register
	        (double) getGlobalTimerOverflow() * ( // How many times have we reset?
	        (double) sim2.timer[3].trr + 1.0 )))); // What is the overflow value?
}

void init() {
	//Math
	setupFastMath();
	//testFastMath2();

	//Display
	Pins[38].function(PIN_38_UART5_TXD); //TX to Serial LCD
	lcd = LCD(LCD_PORT,9600); //Reinstantiates global lcd on serial port defined in VehDefs.h w/ baud rate 9600
	lcd.clear();

	//Switches: From viewpoint of car, most forward switch is ch3, green button in back is ch0
	//Left position produces ADC count of 0, middle in 16000s, right in 32000s
	//Depressed green button produces ADC count of 0, undepressed in 32000s
	InitSingleEndAD(); //initializes Analog to Digital Converter on processor

	//I2C (communication used for IMU)
	MultiChannel_I2CInit();
	//IMU: Do not initialize global timer before this
	Pins[27].function(PIN_27_I2C0_SCL    );//I2C for IMU
	Pins[29].function(PIN_29_I2C0_SDA    );//I2C For IMU
	//Pins[50].hiz();
	//Pins[50].function(PIN_50_IRQ2  ); //IRQ for IMU
	imu = MPU9250(50); //reconstruct MPU9250 object on interrupt pin 50 (interrupt pin goes high when new data available)
	OSTimeDly(3); //delay 3 ticks to give IMU time to set up its registers
	IMUSetup();

	//Side LiDARs
	Pins[23].function(PIN_23_T2IN); //LIDAR pulse left / Timer 2 in
	Pins[25].function(PIN_25_T3IN); //LIDAR pulse right / Timer 3 in
	LidarPWMInit(); //initializes both side LIDARS and global timers
	OSTimeDly(TICKS_PER_SECOND/10);

	//RC
	Pins[14].function(PIN_14_UART6_RXD);	//RC RX
	InitDSM2Rx(RC_PORT); //Initialize RC on UART Port 6

	//Top LiDAR
	Pins[13].function(PIN_13_UART2_RXD);	//LIDAR RX
	Pins[16].function(PIN_16_UART2_TXD);	//LIDAR TX
	PinPWM(39, 25000, .58); //PWM at 25000 Hz with a 58% duty cycle
	//motor val provides about 180 samples per 360 deg scan- that means a sample every 2 degrees or so
	//OSTimeDly(TICKS_PER_SECOND);

	//Odometer
	odo = Odometer(49); //creates Odometer object on pin 49

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
	//bool initGrid = false;
	while (1) {
		lcd.clear();
		char buf[32];
		//Nav
		if (!nav.isFinished())
			sprintf(buf,"x:%3.1f,y:%3.1f,h:%3.1f,h_des:%3.1f",nav.getX()/1000.,nav.getY()/1000.,getHeading(),nav.getHeadDes());
		else sprintf(buf,":) x:%3.1f,y:%3.1f,h:%3.1f",nav.getX()/1000.,nav.getY()/1000.,getHeading());
		lcd.print(buf,32);
		StartAD(); //Updates analog to digital converter so other functions can read switches
		//printf("\nSpLidar[0]:%i,Heading:%f,Right Lidar Val:%i,GlobalTimerTime:%f\n",SpinningLidar::dist[0],getHeading(),getRightLidar(),getGlobalTime());
		//printf("\nLeftLidar:%i\n",getLeftLidar());
		//printf("Log is %ipct full",GetLogPercent());
		//Mapping
		//if (getGlobalTime()>20)
		//	map.featureUpdate();
		/*{
			if (!initGrid) {
				map.initLocalGrid();
				initGrid = true;
			}
			else map.updateOGrid();
			for (int i = 0; i < map.numLidarSegs; ++i)
				printf("LidarSeg %i: startDeg=%i,endDeg=%i\n",i,map.lidarseglist[i].startingDegree,map.lidarseglist[i].endingDegree);
			for (int i = 0; i < map.numSegments; ++i)
				printf("Segment %i: m=%f,b=%f,x1=%f,x2=%f\n",i,map.segmentList[i].m,map.segmentList[i].b,map.segmentList[i].x1,map.segmentList[i].x2);
			for (int i = 0; i < map.numCircles; ++i)
				printf("Circle %i: x=%f,y=%f\n",i,map.circleList[i].x,map.circleList[i].y);
		}*/
		OSTimeDly(TICKS_PER_SECOND/2); //run at 2 Hz
	}
}

void Drive(void*) {
	while (1) {
		Profiler::tic(1);
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
		int switch3 = Utility::switchVal(GetADResult(3));
		if (switch3 == -1) nav.navMethod = Nav::NavMethod::simpleWaypoint;
		else if (switch3 == 0) nav.navMethod = Nav::NavMethod::followRightWall;
		else nav.navMethod = Nav::NavMethod::followPath;
		Profiler::tic(2);
		nav.navUpdate();
		Profiler::toc(2);
		Profiler::toc(1);
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
    CheckNVSettings();
    init();
    IMURun();
    bLog = true;
    Logger::logBegin();

    //LCD Update Task
    OSSimpleTaskCreatewName(LCDUpdate,LCD_PRIO,"LCD Update");

    //Servo Vals Update Task
    OSSimpleTaskCreatewNameSRAM(Drive,DRIVE_PRIO,"Drive");

    //Top LIDAR Task
    SpinningLidar::SpinningLidarInit(); //start top LIDAR serial read and processing task

    //Profiling
    //OSTimeDly(TICKS_PER_SECOND*10);
    //Profiler::start(getGlobalTime);
	//OSTimeDly(15*TICKS_PER_SECOND);
	//Profiler::stop();

    while (1) {
    	OSTimeDly(TICKS_PER_SECOND);
    }
}
