

#include "beep.h"
#include "sys_m4.h"
#include "time.h"

#define STOP									0xff
u8 status_cnt=0,curr_status=BEEP_STATUS_IDLE;

void BEEP_Config(void)
{
	GPIO_InitTypeDef gpio;
	RCC_GPIOD(ENABLE);
	
	gpio.GPIO_Pin=GPIO_Pin_4;
	gpio.GPIO_Mode=GPIO_Mode_OUT;
	gpio.GPIO_OType=GPIO_OType_PP;
	gpio.GPIO_PuPd=GPIO_PuPd_DOWN;
	gpio.GPIO_Speed=GPIO_Speed_100MHz;
	GPIO_Init(GPIOD,&gpio);
	BEEP=0;
	BEEP_SetStatus(BEEP_STATUS_IDLE);
}
char BEEP_Idle(void)
{
	BEEP=0;
	return 0;
}
char BEEP_Flash(u32 time_on,u32 time_off)
{
	if(BEEP==1)
	{
		if(TIME_CheckTimeUs(TIME_BEEP,time_on))
		{
			BEEP=0;
		}
	}
	else
	{
		if(TIME_CheckTimeUs(TIME_BEEP,time_off))
		{
			//BEEP=1;
			return 1;
		}
	}
	return 0;
}
char BEEP_Flash_LOOP(u32 time_on,u32 time_off)
{
	if(BEEP==1)
	{
		if(TIME_CheckTimeUs(TIME_BEEP,time_on))
		{
			BEEP=0;
		}
	}
	else
	{
		if(TIME_CheckTimeUs(TIME_BEEP,time_off))
		{
			BEEP=1;
			return 1;
		}
	}
	return 0;
}
char BEEP_Stop(void)
{
	if(status_cnt==1)
	{
		TIME_Reset(TIME_BEEP);
		BEEP=1;
		status_cnt=2;
		return 0;
	}
	else if(status_cnt==2)
	{
		if(BEEP_Flash(100000,100000))
		{
			BEEP=1;
			status_cnt=3;
			return 0;
		}
	}
	else if(status_cnt==3)
	{
		if(BEEP_Flash(500000,100000))
		{
			//BEEP=1;
			status_cnt=0;
			return 1;
		}
	}
	else
	{
		return 1;
		
	}
	return 0;
}
char BEEP_Start(void)
{
	if(status_cnt==1)
	{
		TIME_Reset(TIME_BEEP);
		BEEP=1;
		status_cnt=2;
		return 0;
	}
	else if(status_cnt==2)
	{
		if(BEEP_Flash(150000,50000))
		{
			BEEP=1;
			status_cnt=3;
			return 0;
		}
	}
	else if(status_cnt==3)
	{
		if(BEEP_Flash(150000,50000))
		{
			//BEEP=1;
			status_cnt=0;
			return 1;
		}
	}
	else
	{
		return 1;
		
	}
	return 0;
}
char BEEP_ESC(void)
{
	if(status_cnt==1)
	{
		TIME_Reset(TIME_BEEP);
		BEEP=1;
		status_cnt=2;
		return 0;
	}
	else if(status_cnt==2)
	{
		if(BEEP_Flash(100000,50000))
		{
			BEEP=1;
			status_cnt=3;
			return 0;
		}
	}
	else if(status_cnt==3)
	{
		if(BEEP_Flash(100000,50000))
		{
			BEEP=1;
			status_cnt=4;
			return 0;
		}
	}
	else if(status_cnt==4)
	{
		if(BEEP_Flash(100000,50000))
		{
			BEEP=1;
			status_cnt=5;
			return 0;
		}
	}
	else if(status_cnt==5)
	{
		if(BEEP_Flash(100000,50000))
		{
			BEEP=1;
			status_cnt=6;
			return 0;
		}
	}
	else if(status_cnt==6)
	{
		if(BEEP_Flash(100000,50000))
		{
			//BEEP=1;
			status_cnt=0;
			return 1;
		}
	}
	else
	{
		return 1;
		
	}
	return 0;
}
char BEEP_LowVolt(void)
{
	if(status_cnt==1)
	{
		TIME_Reset(TIME_BEEP);
		BEEP=1;
		status_cnt=2;
		return 0;
	}
	else if(status_cnt==2)
	{
		if(BEEP_Flash(200000,100000))
		{
			//BEEP=1;
			status_cnt=0;
			return 1;
		}
	}
	return 1;
}
char BEEP_Ack(void)
{
	if(status_cnt==1)
	{
		TIME_Reset(TIME_BEEP);
		BEEP=1;
		status_cnt=2;
		return 0;
	}
	else if(status_cnt==2)
	{
		if(BEEP_Flash(100000,10000))
		{
			//BEEP=1;
			status_cnt=0;
			return 1;
		}
	}
	return 1;
}
char BEEP_Routing(void)
{
	char res=0;
	if(status_cnt==0)
	{
		curr_status=BEEP_STATUS_IDLE;
		BEEP_Idle();
	}
	else
	{
		switch(curr_status)
		{
			case BEEP_STATUS_START:
				res=BEEP_Start();
			break;
			case BEEP_STATUS_STOP:
				res=BEEP_Stop();
			break;
			case BEEP_STATUS_LOW_VOLT:
				res=BEEP_LowVolt();
			break;
			case BEEP_STATUS_ESC:
				res=BEEP_ESC();
			break;
			case BEEP_STATUS_ACK:
				res=BEEP_Ack();
			break;
		}
	}
	return res;
}

char BEEP_SetStatus(u8 status)
{
	if(status_cnt==0)
	{
		if(status!=BEEP_STATUS_IDLE)
		{
			status_cnt=1;
		}
		curr_status=status;
	}
	else
	{
		if(curr_status!=status&&status!=BEEP_STATUS_IDLE)
		{
			status_cnt=1;
			curr_status=status;
		}
	}
	
	return BEEP_Routing();
}










