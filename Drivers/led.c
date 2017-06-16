
#include "led.h"
#include "time.h"

u8 led_status=0;

void LED_Config(void)
{
	GPIO_InitTypeDef gpio;
	RCC_GPIOE(ENABLE);
	
	gpio.GPIO_Pin=GPIO_Pin_2|GPIO_Pin_3|GPIO_Pin_4|GPIO_Pin_5;
	gpio.GPIO_Mode=GPIO_Mode_OUT;
	gpio.GPIO_OType=GPIO_OType_PP;
	gpio.GPIO_PuPd=GPIO_PuPd_UP;
	gpio.GPIO_Speed=GPIO_Speed_100MHz;
	GPIO_Init(GPIOE,&gpio);

}
char LED1_Flash(u32 time_on,u32 time_off)
{
	if(LED1==0)
	{
		if(TIME_CheckTimeUs(TIME_LED1,time_on))
		{
			LED1=1;
		}
	}
	else
	{
		if(TIME_CheckTimeUs(TIME_LED1,time_off))
		{
			LED1=0;
			return 1;
		}
	}
	return 0;
}
char LED2_Flash(u32 time_on,u32 time_off)
{
	if(LED2==0)
	{
		if(TIME_CheckTimeUs(TIME_LED2,time_on))
		{
			LED2=1;
		}
	}
	else
	{
		if(TIME_CheckTimeUs(TIME_LED2,time_off))
		{
			LED2=0;
			return 1;
		}
	}
	return 0;
}
char LED3_Flash(u32 time_on,u32 time_off)
{
	if(LED3==0)
	{
		if(TIME_CheckTimeUs(TIME_LED3,time_on))
		{
			LED3=1;
		}
	}
	else
	{
		if(TIME_CheckTimeUs(TIME_LED3,time_off))
		{
			LED3=0;
			return 1;
		}
	}
	return 0;
}
char LED4_Flash(u32 time_on,u32 time_off)
{
	if(LED4==0)
	{
		if(TIME_CheckTimeUs(TIME_LED4,time_on))
		{
			LED4=1;
		}
	}
	else
	{
		if(TIME_CheckTimeUs(TIME_LED4,time_off))
		{
			LED4=0;
			return 1;
		}
	}
	return 0;
}
void LED_Flash(u8 LED,u32 time_on,u32 time_off)
{
	if(LED&LED_1)
	{
		LED1_Flash(time_on,time_off);
	}
	if(LED&LED_2)
	{
		LED2_Flash(time_on,time_off);
	}
	if(LED&LED_3)
	{
		LED3_Flash(time_on,time_off);
	}
	if(LED&LED_4)
	{
		LED4_Flash(time_on,time_off);
	}

}

void LED1_Idle(void)
{
	LED1_Flash(100000,1600000);
}
void LED2_Idle(void)
{
	LED2_Flash(100000,1600000);
}
void LED3_Idle(void)
{
	LED3_Flash(100000,1600000);
}
void LED4_Idle(void)
{
	LED4_Flash(100000,1600000);
}
void LED_Idle(u8 LED)
{
	if(LED&LED_1)
	{
		LED1_Idle();
	}
	if(LED&LED_2)
	{
		LED2_Idle();
	}
	if(LED&LED_3)
	{
		LED3_Idle();
	}
	if(LED&LED_4)
	{
		LED4_Idle();
	}

}
void LED_ResetStatus(u8 LED)
{
	led_status=LED_STATUS_IDLE;
}
void LED_SetStatus(u8 LED,u8 status)
{
	if(led_status<status)
	{
		led_status=status;
	}
	switch(status)
	{
		case LED_STATUS_CALIB:
			LED_Flash(LED,100000,100000);
		break;
		case LED_STATUS_IDLE:
			LED_Idle(LED);
		break;
		
	}

	
}
