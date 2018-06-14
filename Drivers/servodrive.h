#ifndef SERVO_DRIVE_H
#define SERVO_DRIVE_H

#include <ucos.h>

void ServoDriveInit();
//Make sure you delay long enough after init and setting to 0.
//The Electronic Speed Control has a safety check that requires
//it to see a stalled motor before it arms

void SetServoPos(int servo, double v);
//Servo 0 is steering, Servo 1 is throttle
//-1<=v<=1
//Steer: 1=left,-1=right
//Throttle: 1=full forward, -1=full backward

void SetServoRaw(int servo, int v);
//Throttle: 1024-1029=Stall, 348=Full back, 1722= Full Forward

int GetServoCount(int servo);

double getServoPos(int servo);

//void SetServoHighRaw(int servo, int v);

inline float HiCon(int v) {return (float)((float)(v-1024)/500.0);};
extern OS_SEM * pNotifyNextFrameSem;
extern volatile uint32_t ServoFrameCnt;

#define NUM_SERVO_DRIVE (2)

#endif
