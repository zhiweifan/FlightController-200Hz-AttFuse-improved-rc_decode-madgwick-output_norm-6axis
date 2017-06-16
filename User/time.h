
#ifndef __TIME_H__
#define __TIME_H__

#include "stm32f4xx.h"

enum
{
	TIME_LED1,
	TIME_LED2,
	TIME_LED3,
	TIME_LED4,
	TIME_BEEP,
	TIME_DELAY,
	TIME_START_STOP,
	TIME_RC_OK,
	TIME_ATT,
	TIME_ALT,
	TIME_ALT_SPEED,
	TIME_GPS,
	TIME_ALL
};

uint8_t TIME_CheckTimeUs(uint8_t Number, int32_t Time);
float TIME_GetTimeUs(uint8_t Number);
void TIME_Config(void);

void TIME_Reset(uint8_t Number);

#endif 




