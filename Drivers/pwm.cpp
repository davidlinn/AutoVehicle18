#include "pwm.h"
#include <predef.h>
#include <stdio.h>
#include <ctype.h>
#include <startnet.h>
#include <autoupdate.h>
#include <dhcpclient.h>
#include <smarttrap.h>
#include <taskmon.h>
#include <sim.h>
#include <pins.h>
#include <NetworkDebug.h>


int startCnt = 0xF000;
//int reset = 0x1000;
int pwmOn = 0xFF00;
//int pwmOff = 0x0100;

void PinPWM(int pin, int desiredfrq, double dutyCycle) { //0<dutyCycle<1
	desiredfrq *= 12.8866;
	int clkfrq = 125000000;
	int reset = clkfrq/desiredfrq;
	int pwmOff = dutyCycle*reset;
	EdgeAlignedPWM(pin,reset,pwmOff);
}

//Call EdgeAlignedPWM(15, reset , pwmOff);             //240Hz


/*
 * PinFunctionPWM - Declares the input pin is to be used for PWM
 *
 * pin - input from a PWM initialization function
 */
void PinFunctionPWM(int pin)
{

//Switch statements
#ifdef NANO54415
    //NANO54415
    switch(pin){
    case 13 : Pins[pin].function(PIN_13_PWM_A3); break;
    case 15 : Pins[pin].function(PIN_15_PWM_A1); break;
    case 16 : Pins[pin].function(PIN_16_PWM_B3); break;
    case 31 : Pins[pin].function(PIN_31_PWM_A0); break;
    case 33 : Pins[pin].function(PIN_33_PWM_B0); break;
    case 35 : Pins[pin].function(PIN_35_PWM_B2); break;
    case 37 : Pins[pin].function(PIN_37_PWM_A2); break;
    case 39 : Pins[pin].function(PIN_39_PWM_B1); break;
    }
#elif defined MOD5441X

    //MOD54415
    switch(pin){
    case 19 : J2[pin].function(PINJ2_19_PWM_B3); break;
    case 25 : J2[pin].function(PINJ2_25_PWM_A0); break;
    case 27 : J2[pin].function(PINJ2_27_PWM_B0); break;
    case 28 : J2[pin].function(PINJ2_28_PWM_B2); break;
    case 30 : J2[pin].function(PINJ2_30_PWM_A1); break;
    case 31 : J2[pin].function(PINJ2_31_PWM_A3); break;
    case 35 : J2[pin].function(PINJ2_35_PWM_A2); break;
    case 40 : J2[pin].function(PINJ2_40_PWM_B1); break;
    }
#else
#error PLATFORM NOT SUPPORTED
#endif



}

/*
 * CheckLoadOkayClear - Checks whether the PWM is currently in
 * the process of resetting it's buffers.  If it is, it aborts
 * and resets LoadOkay so that other registers can be overwritten
 */
void CheckLoadOkayClear(void)
{
    if((sim1.mcpwm.mcr & 0x000F) == 0x000F)
    {
        iprintf("LDOK set, resetting. \r\n");
        sim1.mcpwm.mcr |= 0x00F0;
    }

    //Once cleared, clear the reset bit to enable LDOK to run later.
    sim1.mcpwm.mcr &= 0xFFEF;
}

/*
 * RegistDefinitions - This function calls all the other register
 * settings commonly used to enable PWM.  This function ensures that
 * all flags are clear, faults are disabled, and sets any other conditions
 * commonly used when running PWM.
 */
void RegisterDefinitions(int submod, BOOL AnotB)
{
    //Clear Comparison Flags
    sim1.mcpwm.sm[submod].sr |= 0x001F;     //Setting 1 to the last 5 positions clears flags.

    //Disable all Faults
    sim1.mcpwm.sm[submod].dismap &= 0xF000; //The last 3 hex digits are fault enables.

    //Outputs not inverted
    sim1.mcpwm.sm[submod].ocr &= 0xE0C0;    //The second hex digit controls bit inversion.

    //Set output mask to 0
    sim1.mcpwm.mask = 0;                    //This mask register would set the mask value of the
                                            //PWM signal before the Output Control Register(OCR) is set

    //Enable the pwm output
    if (AnotB)  sim1.mcpwm.outen    |= (0x0001<<(8+submod)); //Second hex bit enables A signals.
    else        sim1.mcpwm.outen    |= (0x0001<<(4+submod)); //Third hex bit enables B signals.
}

/*
 * InitializeModule - Calling this function last will turn on the PWM for
 * any submodule. LoadOkay is set, sending the values written to the registers in
 * RegisterDefinitions into the buffers of the PWM module.  Then, the Run bits
 * are activated enabling the modules to begin modulating.
 */
void InitializeModule(void)
{
    //Set PWM_MCR[LDOK]
    sim1.mcpwm.mcr |= 0x000F; //Each of the last 4 bits correspond to the each of the 4 PWM module's LDOK

    //Set PWM_MCR[RUN]
    sim1.mcpwm.mcr |= 0x0F00;//Each bit of the second hex digit correspond to the corresponding 4 PWM modules RUN

}

/*
 * CenterAlignedPWM creates a PWM signal on pin 35 that is centered about an origin value
 * in the middle of the pwm count range.
 * Calculation: 125Mhz/128 ~= 1MHz/8192 counts per cycle ~= 120Hz/cycle ~= 8.33ms/cycle
 *              Duty = 512/8192*100% = 6.25%
 */
void CenterAlignedPWM(int pin, int startCnt, int reset, int pwmOn, int pwmOff )
{
    //The submod and AnotB variables maintain information about what submodule and output PWM the user is
    //working with. For instance on the MOD54415, pin J2[19] is connected to PWM submodule 3B. AnotB being
    //simply a boolean describing whether the pin belongs to A if 1 or B if 0.
    int submod;
    bool AnotB;

#ifdef MOD5441X
    switch(pin){
    case 19 : submod = 3;   AnotB = 0; break;
    case 25 : submod = 0;   AnotB = 1; break;
    case 27 : submod = 0;   AnotB = 0; break;
    case 28 : submod = 2;   AnotB = 0; break;
    case 30 : submod = 1;   AnotB = 1; break;
    case 31 : submod = 3;   AnotB = 1; break;
    case 35 : submod = 2;   AnotB = 1; break;
    case 40 : submod = 1;   AnotB = 0; break;
    default : submod = 1;   AnotB = 0; break;
    }
#elif defined NANO54415
    switch(pin){
    case 13 : submod = 3;   AnotB = 1; break;
    case 15 : submod = 1;   AnotB = 1; break;
    case 16 : submod = 3;   AnotB = 0; break;
    case 31 : submod = 0;   AnotB = 1; break;
    case 33 : submod = 0;   AnotB = 0; break;
    case 35 : submod = 2;   AnotB = 0; break;
    case 37 : submod = 2;   AnotB = 1; break;
    case 39 : submod = 1;   AnotB = 0; break;
    default : submod = 1;   AnotB = 0; break;
    }
#else
#error PLATFORM NOT SUPPORTED
#endif

    PinFunctionPWM(pin);

    CheckLoadOkayClear();

    /*
     * The following values set the register compare values for the PWM moudules.
     * Init is the base count value that the module counts up from
     * Val[1] is the cap count value that the module counter resets at
     * Val[2] and Val[3] set the on and off values of the PWM_A signal respectively
     * Val[4] and Val[5] set the on and off values of the PWM_B signal respectively
     *
     * With the numbering scheme given below, the value of Val[0], a median value is placed at 0x0000
     * and performing center-aligned PWM is done simply by placing the on and off values at equal
     * positive and negative values.
     */
    sim1.mcpwm.sm[submod].init = startCnt;              //Inital Value = -4096
    if (AnotB) sim1.mcpwm.sm[submod].val[2] = pwmOn;    //PWM_A On = -256
    else sim1.mcpwm.sm[submod].val[4] = pwmOn;          //PWM_B On = -256
    sim1.mcpwm.sm[submod].val[0] = (reset-startCnt)/2;  //Median 0
    if (!AnotB) sim1.mcpwm.sm[submod].val[5] = pwmOff;  //PWM_B Off = 256
    else sim1.mcpwm.sm[submod].val[3] = pwmOff;         //PWM_A Off = 256
    sim1.mcpwm.sm[submod].val[1] = reset;               //Final Value = 4096

    //The following control registers are instantiated with the following conditions
    //Clock pre-scaler = 128
    sim1.mcpwm.sm[submod].cr1 = 0x0470;                 //0000_0100_0111_0000


    //Set PWMA and PWMB to be dependent on modules
    //Set all signals to local signals with force_en low
    //Debug and wait are disabled
    sim1.mcpwm.sm[submod].cr2 = 0x2000;             //0000_0000_0000_0000

    RegisterDefinitions(submod, AnotB);

    InitializeModule();
}


/*
 * EdgeAlignedPWM creates a PWM signal on Pin 35 with a period of approximately 5ms and duty
 * cycle of 6.25%.
 *
 * Calculation: 125Mhz/128 ~= 1MHz/4096 counts per cycle ~= 240Hz/cycle ~= 4.16ms/cycle
 *              Duty = 256/4096*100% = 6.25%
 */
void EdgeAlignedPWM(int pin, int reset, int pwmOff )
{

    int submod;
    bool AnotB;

#ifdef MOD5441X
    switch(pin){
    case 19 : submod = 3;   AnotB = 0;  break;
    case 25 : submod = 0;   AnotB = 1;  break;
    case 27 : submod = 0;   AnotB = 0;  break;
    case 28 : submod = 2;   AnotB = 0;  break;
    case 30 : submod = 1;   AnotB = 1;  break;
    case 31 : submod = 3;   AnotB = 1;  break;
    case 35 : submod = 2;   AnotB = 1;  break;
    case 40 : submod = 1;   AnotB = 0;  break;
    default : submod = 1;   AnotB = 0; break;
    }
#elif defined NANO54415
    switch(pin){
    case 13 : submod = 3;   AnotB = 1;  break;
    case 15 : submod = 1;   AnotB = 1;  break;
    case 16 : submod = 3;   AnotB = 0;  break;
    case 31 : submod = 0;   AnotB = 1;  break;
    case 33 : submod = 0;   AnotB = 0;  break;
    case 35 : submod = 2;   AnotB = 0;  break;
    case 37 : submod = 2;   AnotB = 1;  break;
    case 39 : submod = 1;   AnotB = 0;  break;
    default : submod = 1;   AnotB = 0; break;
    }
#else
#error PLATFORM NOT SUPPORTED
#endif

    PinFunctionPWM(pin);

    CheckLoadOkayClear();

    /*
     * For edge aligned outputs simply set the val[2], val[3], val[4], or
     * val[5] registers to match the init or val[1] register.
     */
    sim1.mcpwm.sm[submod].init = 0x0000;    //Inital Value = 0
    if (AnotB) sim1.mcpwm.sm[submod].val[2] = 0x0000;   //PWM_A On = 0
    else sim1.mcpwm.sm[submod].val[4] = 0x0000; //PWM_B On = 0

    sim1.mcpwm.sm[submod].val[0] = reset/2; //Median 0

    if (!AnotB) sim1.mcpwm.sm[submod].val[5] = pwmOff;  //PWM_B Off = 256
    else sim1.mcpwm.sm[submod].val[3] = pwmOff; //PWM_A Off = 256

    // Set the initial deadtime to 0 cycles
    if (AnotB)  { sim1.mcpwm.sm[submod].dtcnt0 = 0x0000; }
    else        { sim1.mcpwm.sm[submod].dtcnt1 = 0x0000; }

    sim1.mcpwm.sm[submod].val[1] = reset;   //Final Value = 4096

    //Enable Control Registers
    sim1.mcpwm.sm[submod].cr1 = 0x0470;                 //0000_0100_0111_0000
    sim1.mcpwm.sm[submod].cr2 = 0x2000;             //0010_0000_0000_0000

    RegisterDefinitions(submod, AnotB);

    InitializeModule();

}
