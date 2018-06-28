/*
 * pwm.h
 *
 *  Created on: Jun 26, 2018
 *      Author: NetBurner
 */

#ifndef PWM_H_
#define PWM_H_

#include <basictypes.h>

void PinPWM(int pin, int desiredfrq, double dutyCycle);

void EdgeAlignedPWM(int pin, int reset, int pwmOff );
void PinFunctionPWM(int pin);
void CheckLoadOkayClear(void);
void RegisterDefinitions(int submod, BOOL AnotB);
void InitializeModule(void);
void CenterAlignedPWM(int pin, int startCnt, int reset, int pwmOn, int pwmOff );



#endif /* PWM_H_ */
