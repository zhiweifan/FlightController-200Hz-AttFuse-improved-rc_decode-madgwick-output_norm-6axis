#ifndef LED_H
#define LED_H

#include "sys_m4.h"

#define	LED_1										(1)
#define	LED_2										(1<<1)
#define	LED_3										(1<<2)
#define	LED_4										(1<<3)
#define LED_ALL									(15)

#define LED1										PEOut(2)
#define LED2										PEOut(3)
#define LED3										PEOut(4)
#define LED4										PEOut(5)

#define LED1_IN									PEIn(2)
#define LED2_IN									PEIn(3)
#define LED3_IN									PEIn(4)
#define LED4_IN									PEIn(5)

#define LED1_ON()								LED1=0
#define LED2_ON()								LED2=0
#define LED3_ON()								LED3=0
#define LED4_ON()								LED4=0

#define LED1_OFF()							LED1=1
#define LED2_OFF()							LED2=1
#define LED3_OFF()							LED3=1
#define LED4_OFF()							LED4=1

#define LED1_TOGGLE()						LED1=!LED1
#define LED2_TOGGLE()						LED2=!LED2
#define LED3_TOGGLE()						LED3=!LED3
#define LED4_TOGGLE()						LED4=!LED4

#define LED_STATUS_IDLE					0
#define LED_STATUS_CALIB        1

void LED_Config(void);

void LED_SetStatus(u8 LED,u8 status);
void LED_ResetStatus(u8 LED);

#endif


