#include <predef.h>
#include <init.h>
#include <stdio.h>
#include <basictypes.h>
#include <ucos.h> //ucos.h in 2.8, nbrtos.h in 3.0
#include <cfinter.h>
#include <sim5441X.h>           /*on-chip register definitions*/
#include <intcdefs.h>
#include <pins.h>
#include <pinconstant.h>
//#include <nbrtos.h>
#include <CFinter.h>
#include "servodrive.h"
//#include "introspec.h"


//LOGFILEINFO;





volatile uint32_t ServoTimes[NUM_SERVO_DRIVE];
volatile int ServoMode[NUM_SERVO_DRIVE];
volatile uint32_t ServoElapsed[NUM_SERVO_DRIVE];
volatile uint32_t ServoIsr[NUM_SERVO_DRIVE];
volatile double ServoPos[NUM_SERVO_DRIVE];


//Clock is 125Mhz
//


#define SERVO_MIN  (56250*2)	//0.9msec
#define SERVO_ZERO (93750*2)  //1.5 msec
#define SERVO_MAX (131200*2)	//2.1 msec
#define SERVO_INTERVAL (2500000) //20 msec
#define START_DELAY    (250000000)
#define START_DELTA    (SERVO_MAX*2)
#define SERVO_DELTA (((double)SERVO_MAX-(double)SERVO_MIN)/2.0)

#define MODE_SERVO_PULSE (0)
#define MODE_SERVO_WAIT (1)
#define MODE_SERVO_START (2)


OS_SEM * pNotifyNextFrameSem;
volatile uint32_t ServoFrameCnt;


extern OS_SEM MainTaskSem;

volatile uint32_t IsrCorrections;

void CoreRcTimer(int ch)
{
	sim2.timer[ch].ter=3; //clear event
	ServoIsr[ch]++;
	uint32_t ntrr=sim2.timer[ch].tcn+10000; //default time to delay is 80us
	
	switch (ServoMode[ch]) {
		case MODE_SERVO_START: //Servo just went to 1...
		case MODE_SERVO_PULSE:
			ntrr=(SERVO_INTERVAL-ServoElapsed[ch]); //Down for 20ms-ServoElapsed
			ServoMode[ch]=MODE_SERVO_WAIT;
			if(ch==0) {
				if(pNotifyNextFrameSem) pNotifyNextFrameSem->Post();
				ServoFrameCnt++; //go to next frame if steering channel
			}
			break;
		case MODE_SERVO_WAIT:
			ServoElapsed[ch]=ServoTimes[ch]; //Set ServoElapsed to input (.9-2.1ms)
			ntrr=ServoTimes[ch]; //Up for input (.9-2.1ms)
			ServoMode[ch]=MODE_SERVO_PULSE;
			break;
	}
	sim2.timer[ch].trr=ntrr;
}


//Interrupt Service Routine Ch0_ISR blocks all other interrupts
//i.e. this is the most important thing we have to do
//CoreRcTimer is called every time the timer goes off
INTERRUPT(Ch0_ISR,0x2700)
{
  CoreRcTimer(0);
}
	
INTERRUPT(Ch1_ISR,0x2700)
{
  CoreRcTimer(1);
}

/*INTERRUPT(Ch2_ISR,0x2700)
{
  CoreRcTimer(2);
}*/


void ServoDriveInit()
{
	Pins[19].function(PIN_19_T0OUT );
	Pins[21].function(PIN_21_T1OUT );
	//Pins[23].function(PIN_23_T2OUT );

	for(int i=0; i<NUM_SERVO_DRIVE; i++) {
		ServoTimes[i]=SERVO_ZERO;
		ServoMode[i]=MODE_SERVO_START;
		ServoElapsed[i]=0;
		sim2.timer[i].tmr=0;
    }
	//prescaler=1,disable capture event out/reference mode,toggle output
	// 0000 0000      00                                       1
	//disable DMA request/interrupt for reference reached, restart timer count when reach ref
	//            0                                          1
	//input clk: stop count, reset timer
	// 00                    0
	sim2.timer[0].tmr=0x0028;
	sim2.timer[0].txmr=0; //disable unused features, increment timer by 1
	sim2.timer[0].ter=3;
	sim2.timer[0].trr=START_DELAY;
	sim2.timer[0].tcr=0;
	sim2.timer[0].tcn=1;

	sim2.timer[1].tmr=0x0028;   // 0000 0000 00 1 0 1 0 0 0
	sim2.timer[1].txmr=0;
	sim2.timer[1].ter=3;
	sim2.timer[1].trr=START_DELAY+START_DELTA;
	sim2.timer[1].tcr=0;
	sim2.timer[1].tcn=1;

	sim2.timer[0].ter=3;
	sim2.timer[1].ter=3;
	
	asm(" nop");
	
   // ShowChData("a)",1);

	SETUP_DMATIMER0_ISR(&Ch0_ISR,2); //set interrupt controller at lvl 2
	SETUP_DMATIMER1_ISR(&Ch1_ISR,2);

	sim2.timer[0].ter=3;
	sim2.timer[1].ter=3;
    OSLock();
    //enable timer, set clk to internal bus clk/1,
    // enable DMA request/interrupt upon reaching ref
	sim2.timer[0].tmr=0x003B; // 0000 0000 00 1 1 1 0 1 1
	sim2.timer[1].tmr=0x003B; // 0000 0000 00 1 1 1 0 1 1
	OSUnlock();
}






void SetServoPos(int ch, double v)
{
	ServoPos[ch] = v;
	//DRIVE STRAIGHT OFFSET
	if (ch == 0)
		v += .045;
	//--------------------
	uint32_t dw;
	if ((ch<0) || (ch>=NUM_SERVO_DRIVE)) return;
	if(v<=-1.0) ServoTimes[ch]=SERVO_MIN;
	else if (v>=1.0) ServoTimes[ch]=SERVO_MAX;
	else if (v==0.0) ServoTimes[ch]=SERVO_ZERO;
	else
	{
		v*=SERVO_DELTA;
		v+=SERVO_ZERO;
		dw=(uint32_t)v;
		//printf("Setting ServoTimes[%i] to %lu\n",ch,dw);
		ServoTimes[ch]=dw;
		//Servo set to SERVO_DELTA*v+SERVO_ZERO
    }
}

void SetServoRaw(int ch, int v)
{
	if ((ch<0) || (ch>=NUM_SERVO_DRIVE)) return; //return if ch out of bounds

	//calculations make ServoPos[ch] comparable for SetServoRaw and SetServoPos
	ServoPos[ch] = ((double)(v-1024)*60)/SERVO_DELTA;

    int vc=v;
    vc-=1024;
    vc*=60;
    vc+=SERVO_ZERO;

	if (vc<=SERVO_MIN) ServoTimes[ch]=SERVO_MIN;
	else if(vc>=SERVO_MAX) ServoTimes[ch]=SERVO_MAX;
	else ServoTimes[ch]=vc;
	//printf("Setting ServoTimes[%i] to %lu\n",ch,ServoTimes[ch]);
	//Servo set to 60(v-1024)+SERVO_ZERO
}


void SetServoHighRaw(int ch, int v)
{
	if ((ch<0) || (ch>=NUM_SERVO_DRIVE)) return; //return if ch out of bounds

    float f=v;
	f-=1024;
	f/=500;
	//ServoPos(ch,(v-1024)/500)
	//Servo set to (SERVO_DELTA*(v-1024)/500) + SERVO_ZERO
	SetServoPos(ch,f);
}



int GetServoCount(int servo)
{
	return ServoTimes[servo];
}


double getServoPos(int servo) {
	return ServoPos[servo];
}




