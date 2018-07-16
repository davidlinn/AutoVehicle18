//Scale is 1024=center range 0 to 2048

//RC Channels:
//rc_ch[0] Left Joysick Up/Down 569-1708
//rc_ch[1] Steering (left is high)
//2 Throttle
//3 Left Joystick L/R
//4 Log (Off=340,On=1708)		(gear)
//5 Mode (1=347,2=1032,3=1716)
//6 File						(aux2) 340-1024-1708
//7 Spinning Disc on Right side (353-1705)
//8 ?

extern volatile uint32_t  rc_ch[16];
extern volatile uint32_t RCFrameCnt;
extern void ( * RCCallBack)(uint32_t ch, uint32_t v);
void InitDSM2Rx(int serial_port);




