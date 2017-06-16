

#include "systime.h"
#include "stm32f4xx_tim.h"

#define CTRL_PERIOD  10000  //单位us


//APB1 Prescaler=4,APB2 Prescaler=2,
//so CLK_INT for TIM2,3,4,5,6,7,12,13,14 is 84MHz
//CLK_INT for TIM1,8,9,10,11 is 168MHz
void SysTime_Config(void)
{
	TIM_TimeBaseInitTypeDef  tim;
	NVIC_InitTypeDef         nvic;

	RCC_APB1PeriphClockCmd(RCC_APB1Periph_TIM6,ENABLE);
	
	tim.TIM_Prescaler =84-1;
	tim.TIM_CounterMode = TIM_CounterMode_Up;
	tim.TIM_ClockDivision = TIM_CKD_DIV1;
	tim.TIM_Period = CTRL_PERIOD-1;
	TIM_TimeBaseInit(TIM6,&tim);
	
	nvic.NVIC_IRQChannel = TIM6_DAC_IRQn;
	nvic.NVIC_IRQChannelPreemptionPriority =0;
	nvic.NVIC_IRQChannelSubPriority =0;
	nvic.NVIC_IRQChannelCmd = ENABLE;
	NVIC_Init(&nvic);
	
	TIM_Cmd(TIM6, ENABLE);	 
	TIM_ClearFlag(TIM6, TIM_FLAG_Update);
	TIM_ITConfig(TIM6, TIM_IT_Update,ENABLE);
	TIM_ClearFlag(TIM6, TIM_FLAG_Update);
}
int systicks;
void TIM6_DAC_IRQHandler(void)
{
	if (TIM_GetITStatus(TIM6,TIM_IT_Update)!= RESET) 
	{
		TIM_ClearITPendingBit(TIM6,TIM_IT_Update);
		TIM_ClearFlag(TIM6, TIM_FLAG_Update);	
		
		systicks++;
		SysProcess();
		
	}
}


