//Scale is 1024=center range 0 to 2048

//RC Channels:
//1 Steering (left is high)
//2 Throttle
//3 - 							(left joystick)
//4 Log (Off=340,On=1708)		(gear)
//5 Mode (1=347,2=1032,3=1716)
//6 File						(aux2)
//7-9 unused

extern volatile uint32_t  rc_ch[16];
extern volatile uint32_t RCFrameCnt;
extern void ( * RCCallBack)(uint32_t ch, uint32_t v);
void InitDSM2Rx(int serial_port);




