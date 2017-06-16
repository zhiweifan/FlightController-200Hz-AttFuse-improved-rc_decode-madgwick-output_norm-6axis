
#ifndef PWM_H
#define PWM_H

#include "typedef.h"
void PWM_Config(void);

//PWM1-4
#define TIM3_PERIOD						60000
//PWM5-8
#define TIM1_PERIOD						60000
//PWM9-12
#define TIM4_PERIOD						60000
//PWM13-14
#define TIM5_PERIOD						60000
//PWM15-18
#define TIM8_PERIOD						60000


float SetPWM1(float duty);
float SetPWM2(float duty);
float SetPWM3(float duty);
float SetPWM4(float duty);
float SetPWM5(float duty);
float SetPWM6(float duty);
float SetPWM7(float duty);
float SetPWM8(float duty);
float SetPWM9(float duty);
float SetPWM10(float duty);
float SetPWM11(float duty);
float SetPWM12(float duty);
float SetPWM13(float duty);
float SetPWM14(float duty);
float SetPWM15(float duty);


extern CtrlSys_TypeDef CtrlSys;


#endif


