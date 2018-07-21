#include <predef.h>
#include <init.h>
#include <stdio.h>
#include <ctype.h>
#include <basictypes.h>
#include <fdprintf.h>
#include <pins.h>
#include <serial.h>
#include <serinternal.h>
#include <sim.h>
#include <pin_irq.h>
//#include <nbrtos.h>
#include "dsm2.h"
//#include "lidar.h"
//#include "servodrive.h"
#include <HiResTimer.h>
//#include "introspec.h"


//LOGFILEINFO;


volatile uint32_t  rc_ch[16];
volatile uint32_t RCFrameCnt;


void ( * RCCallBack)(uint32_t ch, uint32_t v);

extern HiResTimer* globalTimer;



void RC_ProcessChars( int num, uint8_t c ) 
{
//iprintf("RCProcessChars\n");
static uint32_t lt;
static uint8_t lc;
static int state;
//uint32_t t=sim2.timer[3].tcn;
/*uint32_t dt;
if(t<lt)
{//Roll over every 34 sec or so...
 dt=(t-0x80000000)-(lt-0x80000000);
}
else
{
dt=(t-lt);
}*/
double t= globalTimer->readTime();
double dt = globalTimer->readTime()-lt;

//4 msec
//16 msec
//125000000 clk/sec
//125000 clk/msec
//4 msec 500000
//16 msec = 2000000

//if ((dt>500000) && (dt<2000000))
if (dt>.004 && dt<.016) {
	RCFrameCnt++;
	state=1;
}
else
{
 state++;
 if(((state & 1)==0) && (state>2))
 {
	uint16_t w=lc;
	w=(w<<8)|c;
	uint32_t ch=(w & 0x7800)>>11;//  0111 1000 0000 0000
	if (ch<16)
	{
	   uint32_t v=(w & 0x7FF);
	   rc_ch[ch]=v;
	   //iprintf("Ch %lu: %lu\n",ch,v); //Print RC recieved data
	   if(RCCallBack)
		    RCCallBack(ch,v);
    }
 }
}



lc=c;
lt=t; //Last time
}

void InitDSM2Rx(int serial_port)
{
	iprintf("InitDSM2Rx\n");
	SimpleOpenSerial(serial_port,115200); //RC
	UartData[serial_port].m_pPutCharFunc =RC_ProcessChars;

}



