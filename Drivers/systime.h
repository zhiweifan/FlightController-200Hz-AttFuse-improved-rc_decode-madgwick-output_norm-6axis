

#ifndef SYSTIME_H
#define SYSTIME_H
#include "main.h"

void SysTime_Config(void);

void TIM6_DAC_IRQHandler(void);
void TIM7_IRQHandler(void);

extern void SysProcess(void);
extern CtrlSys_TypeDef CtrlSys;

extern int systicks;
#endif


