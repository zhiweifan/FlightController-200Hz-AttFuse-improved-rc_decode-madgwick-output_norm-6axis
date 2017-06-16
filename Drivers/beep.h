#ifndef BEEP_H
#define BEEP_H

#include "stm32f4xx.h"

#define BEEP												PDOut(4)

#define BEEP_STATUS_IDLE						0
#define BEEP_STATUS_START						1
#define BEEP_STATUS_STOP						2
#define BEEP_STATUS_LOW_VOLT				3
#define BEEP_STATUS_ESC							4
#define BEEP_STATUS_ACK							5


void BEEP_Config(void);
char BEEP_SetStatus(u8 status);
char BEEP_Routing(void);


#endif



