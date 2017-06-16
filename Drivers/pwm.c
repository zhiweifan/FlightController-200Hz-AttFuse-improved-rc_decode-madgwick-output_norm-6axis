

#include "pwm.h"
#include "stm32f4xx_tim.h"
#include "sys_m4.h"

//#define TIM3_PERIOD						56000

//PWM map
//board silk:		1		2		3		4		5		6		7		8		9		10	11	12	13	14	15		
//       net:		5		6		7		8		1		2		3		4		9		10	11	12	13	14	15


//APB1 Prescaler=4,APB2 Prescaler=2,
//so CLK_INT for TIM2,3,4,5,6,7,12,13,14 is 84MHz
//CLK_INT for TIM1,8,9,10,11 is 168MHz
void PWM_Config(void)
{
	GPIO_InitTypeDef          gpio;
	TIM_TimeBaseInitTypeDef   tim;
	TIM_OCInitTypeDef         oc;
	
	RCC_GPIOA(ENABLE);
	RCC_GPIOB(ENABLE);
	RCC_GPIOC(ENABLE);
	RCC_GPIOD(ENABLE);
	RCC_GPIOE(ENABLE);
	RCC_TIM3(ENABLE);
	RCC_TIM1(ENABLE);
	RCC_TIM4(ENABLE);
	RCC_TIM5(ENABLE);
	RCC_TIM8(ENABLE);

	gpio.GPIO_OType=GPIO_OType_OD;
//	gpio.GPIO_OType=GPIO_OType_PP;
	gpio.GPIO_Pin = GPIO_Pin_6 | GPIO_Pin_7;
	gpio.GPIO_Mode = GPIO_Mode_AF;
	gpio.GPIO_Speed = GPIO_Speed_100MHz;
	GPIO_Init(GPIOA,&gpio);
	
	gpio.GPIO_Pin = GPIO_Pin_0 | GPIO_Pin_1;
	gpio.GPIO_Mode = GPIO_Mode_AF;
	gpio.GPIO_Speed = GPIO_Speed_100MHz;
	GPIO_Init(GPIOB,&gpio);
	
	GPIO_PinAFConfig(GPIOA,GPIO_PinSource6, GPIO_AF_TIM3);
	GPIO_PinAFConfig(GPIOA,GPIO_PinSource7, GPIO_AF_TIM3);    
	GPIO_PinAFConfig(GPIOB,GPIO_PinSource0, GPIO_AF_TIM3);
	GPIO_PinAFConfig(GPIOB,GPIO_PinSource1, GPIO_AF_TIM3); 
	
//	tim.TIM_Prescaler =3-1;   //56000对应2ms
	tim.TIM_Prescaler =7-1;   //60000对应5ms
	tim.TIM_CounterMode = TIM_CounterMode_Up;
	tim.TIM_Period =TIM3_PERIOD;   //2ms
	tim.TIM_ClockDivision = TIM_CKD_DIV1;
	TIM_TimeBaseInit(TIM3,&tim);

//不管向上还是向下计数，PWM1:TIM_Pulse代表有效电平时间(0-(TIM_Pulse-1))；PWM2：TIM_Pulse代表无效电平时间(0-(TIM_Pulse-1))
//TIM_OCPolarity_Low代表低电平为有效电平；TIM_OCPolarity_High代表高电平为有效电平
	oc.TIM_OCMode = TIM_OCMode_PWM1;  
	oc.TIM_OutputState = TIM_OutputState_Enable;
	oc.TIM_OutputNState = TIM_OutputState_Disable;
	oc.TIM_Pulse = 0;
	oc.TIM_OCPolarity = TIM_OCPolarity_High;
	oc.TIM_OCNPolarity = TIM_OCPolarity_Low;
	oc.TIM_OCIdleState = TIM_OCIdleState_Reset;
	oc.TIM_OCNIdleState = TIM_OCNIdleState_Set;
	TIM_OC1Init(TIM3,&oc);
	TIM_OC2Init(TIM3,&oc);
	TIM_OC3Init(TIM3,&oc);
	TIM_OC4Init(TIM3,&oc);
	
	TIM_CCPreloadControl(TIM3,ENABLE);
	TIM_OC1PreloadConfig(TIM3,TIM_OCPreload_Enable);
	TIM_OC2PreloadConfig(TIM3,TIM_OCPreload_Enable);
	TIM_OC3PreloadConfig(TIM3,TIM_OCPreload_Enable);
	TIM_OC4PreloadConfig(TIM3,TIM_OCPreload_Enable);
	TIM_Cmd(TIM3,ENABLE);
	//TIM_ARRPreloadConfig(TIM1,ENABLE);
	
	//PWM5-PWM8
	gpio.GPIO_Pin = GPIO_Pin_9 | GPIO_Pin_11|GPIO_Pin_13 | GPIO_Pin_14;
	gpio.GPIO_Mode = GPIO_Mode_AF;
	gpio.GPIO_Speed = GPIO_Speed_100MHz;
	GPIO_Init(GPIOE,&gpio);
	GPIO_PinAFConfig(GPIOE,GPIO_PinSource9, GPIO_AF_TIM1);
	GPIO_PinAFConfig(GPIOE,GPIO_PinSource11, GPIO_AF_TIM1);    
	GPIO_PinAFConfig(GPIOE,GPIO_PinSource13, GPIO_AF_TIM1);
	GPIO_PinAFConfig(GPIOE,GPIO_PinSource14, GPIO_AF_TIM1); 
	tim.TIM_Prescaler =14-1;   //60000对应5ms
	tim.TIM_Period =TIM1_PERIOD;   //2ms
	TIM_TimeBaseInit(TIM1,&tim);
	oc.TIM_OCMode = TIM_OCMode_PWM2;  
	oc.TIM_OutputState = TIM_OutputState_Enable;
	oc.TIM_OutputNState = TIM_OutputState_Disable;
	oc.TIM_Pulse = 0;
	oc.TIM_OCPolarity =TIM_OCPolarity_High;// TIM_OCPolarity_Low;//TIM_OCPolarity_High;
	oc.TIM_OCNPolarity =TIM_OCPolarity_Low;
	oc.TIM_OCIdleState = TIM_OCIdleState_Reset;
	oc.TIM_OCNIdleState = TIM_OCNIdleState_Set;
	//oc.TIM_OCMode = TIM_OCMode_PWM2;
	TIM_OC1Init(TIM1,&oc);
//	oc.TIM_OCMode = TIM_OCMode_PWM2;
	oc.TIM_OCPolarity =TIM_OCPolarity_High;
	TIM_OC2Init(TIM1,&oc);
//	oc.TIM_OCMode = TIM_OCMode_PWM2;
	oc.TIM_OCPolarity =TIM_OCPolarity_High;
	TIM_OC3Init(TIM1,&oc);
	oc.TIM_OCMode = TIM_OCMode_PWM1;
	oc.TIM_OCPolarity =TIM_OCPolarity_High;
	TIM_OC4Init(TIM1,&oc);
	TIM_CCPreloadControl(TIM1,ENABLE);
	TIM_OC1PreloadConfig(TIM1,TIM_OCPreload_Enable);
	TIM_OC2PreloadConfig(TIM1,TIM_OCPreload_Enable);
	TIM_OC3PreloadConfig(TIM1,TIM_OCPreload_Enable);
	TIM_OC4PreloadConfig(TIM1,TIM_OCPreload_Enable);
	TIM_Cmd(TIM1,ENABLE);
	TIM_CtrlPWMOutputs(TIM1,ENABLE);//TIMx: where x can be 1 or 8 to select the TIMx peripheral
	
	//PWM9-PWM12
	gpio.GPIO_Pin = GPIO_Pin_12 | GPIO_Pin_13|GPIO_Pin_14 | GPIO_Pin_15;
	gpio.GPIO_Mode = GPIO_Mode_AF;
	gpio.GPIO_Speed = GPIO_Speed_100MHz;
	GPIO_Init(GPIOD,&gpio);
	GPIO_PinAFConfig(GPIOD,GPIO_PinSource12, GPIO_AF_TIM4);
	GPIO_PinAFConfig(GPIOD,GPIO_PinSource13, GPIO_AF_TIM4);    
	GPIO_PinAFConfig(GPIOD,GPIO_PinSource14, GPIO_AF_TIM4);
	GPIO_PinAFConfig(GPIOD,GPIO_PinSource15, GPIO_AF_TIM4); 
	tim.TIM_Prescaler =7-1;   //60000对应5ms //timer-84MHz
	tim.TIM_Period =TIM4_PERIOD;   //2ms
	TIM_TimeBaseInit(TIM4,&tim);
	TIM_OC1Init(TIM4,&oc);
	TIM_OC2Init(TIM4,&oc);
	TIM_OC3Init(TIM4,&oc);
	TIM_OC4Init(TIM4,&oc);
	TIM_CCPreloadControl(TIM4,ENABLE);
	TIM_OC1PreloadConfig(TIM4,TIM_OCPreload_Enable);
	TIM_OC2PreloadConfig(TIM4,TIM_OCPreload_Enable);
	TIM_OC3PreloadConfig(TIM4,TIM_OCPreload_Enable);
	TIM_OC4PreloadConfig(TIM4,TIM_OCPreload_Enable);
	TIM_Cmd(TIM4,ENABLE);
	
	//PWM13-PWM14
	gpio.GPIO_Pin = GPIO_Pin_0 | GPIO_Pin_1;
	gpio.GPIO_Mode = GPIO_Mode_AF;
	gpio.GPIO_Speed = GPIO_Speed_100MHz;
	GPIO_Init(GPIOA,&gpio);
	GPIO_PinAFConfig(GPIOA,GPIO_PinSource0, GPIO_AF_TIM5);
	GPIO_PinAFConfig(GPIOA,GPIO_PinSource1, GPIO_AF_TIM5);    
	tim.TIM_Prescaler =7-1;   //60000对应5ms //timer-84MHz
	tim.TIM_Period =TIM5_PERIOD;   //2ms
	TIM_TimeBaseInit(TIM5,&tim);
	TIM_OC1Init(TIM5,&oc);
	TIM_OC2Init(TIM5,&oc);
	TIM_CCPreloadControl(TIM5,ENABLE);
	TIM_OC1PreloadConfig(TIM5,TIM_OCPreload_Enable);
	TIM_OC2PreloadConfig(TIM5,TIM_OCPreload_Enable);
	TIM_Cmd(TIM5,ENABLE);
	
	//PWM15-PWM18
	gpio.GPIO_Pin = GPIO_Pin_6|GPIO_Pin_7|GPIO_Pin_8|GPIO_Pin_9;
	gpio.GPIO_Mode = GPIO_Mode_AF;
	gpio.GPIO_Speed = GPIO_Speed_100MHz;
	GPIO_Init(GPIOC,&gpio);
	GPIO_PinAFConfig(GPIOC,GPIO_PinSource6, GPIO_AF_TIM8);
	GPIO_PinAFConfig(GPIOC,GPIO_PinSource7, GPIO_AF_TIM8);    
	GPIO_PinAFConfig(GPIOC,GPIO_PinSource8, GPIO_AF_TIM8);
	GPIO_PinAFConfig(GPIOC,GPIO_PinSource9, GPIO_AF_TIM8); 
	tim.TIM_Prescaler =14-1;   //60000对应5ms
	tim.TIM_Period =TIM8_PERIOD;   //2ms
	TIM_TimeBaseInit(TIM8,&tim);
	oc.TIM_OCMode = TIM_OCMode_PWM2;
//	oc.TIM_OCPolarity =TIM_OCPolarity_High;
	TIM_OC1Init(TIM8,&oc);
	TIM_OC2Init(TIM8,&oc);
	TIM_OC3Init(TIM8,&oc);
	TIM_OC4Init(TIM8,&oc);
	TIM_CCPreloadControl(TIM8,ENABLE);
	TIM_OC1PreloadConfig(TIM8,TIM_OCPreload_Enable);
	TIM_OC2PreloadConfig(TIM8,TIM_OCPreload_Enable);
	TIM_OC3PreloadConfig(TIM8,TIM_OCPreload_Enable);
	TIM_OC4PreloadConfig(TIM8,TIM_OCPreload_Enable);
	TIM_Cmd(TIM8,ENABLE);
	TIM_CtrlPWMOutputs(TIM8,ENABLE);//TIMx: where x can be 1 or 8 to select the TIMx peripheral
	
	
}

float SetPWM1(float duty)
{
	u16 tmp;
	if(duty<0)
	{
		duty=0;
	}
	else if(duty>1.f)
	{
		duty=1.f;
	}
	tmp=TIM3_PERIOD*duty;
	TIM_SetCompare1(TIM3,tmp);
	return duty;
}
float SetPWM2(float duty)
{
	u16 tmp;
	if(duty<0)
	{
		duty=0;
	}
	else if(duty>1.f)
	{
		duty=1.f;
	}
	tmp=TIM3_PERIOD*duty;
	TIM_SetCompare2(TIM3,tmp);
	return duty;
}
float SetPWM3(float duty)
{
	u16 tmp;
	if(duty<0)
	{
		duty=0;
	}
	else if(duty>1.f)
	{
		duty=1.f;
	}
	tmp=TIM3_PERIOD*duty;
	TIM_SetCompare3(TIM3,tmp);
	return duty;
}
float SetPWM4(float duty)
{
	u16 tmp;
	if(duty<0)
	{
		duty=0;
	}
	else if(duty>1.f)
	{
		duty=1.f;
	}
	tmp=TIM3_PERIOD*duty;
	TIM_SetCompare4(TIM3,tmp);
	return duty;
}


float SetPWM5(float duty)
{
	u16 tmp;
	if(duty<0)
	{
		duty=0;
	}
	else if(duty>1.f)
	{
		duty=1.f;
	}
	tmp=TIM1_PERIOD*duty;
	TIM_SetCompare1(TIM1,tmp);
	return duty;
}
float SetPWM6(float duty)
{
	u16 tmp;
	if(duty<0)
	{
		duty=0;
	}
	else if(duty>1.f)
	{
		duty=1.f;
	}
	tmp=TIM1_PERIOD*duty;
	TIM_SetCompare2(TIM1,tmp);
	return duty;
}
float SetPWM7(float duty)
{
	u16 tmp;
	if(duty<0)
	{
		duty=0;
	}
	else if(duty>1.f)
	{
		duty=1.f;
	}
	tmp=TIM1_PERIOD*duty;
	TIM_SetCompare3(TIM1,tmp);
	return duty;
}
float SetPWM8(float duty)
{
	u16 tmp;
	if(duty<0)
	{
		duty=0;
	}
	else if(duty>1.f)
	{
		duty=1.f;
	}
	tmp=TIM1_PERIOD*duty;
	TIM_SetCompare4(TIM1,tmp);
	return duty;
}


float SetPWM9(float duty)
{
	u16 tmp;
	if(duty<0)
	{
		duty=0;
	}
	else if(duty>1.f)
	{
		duty=1.f;
	}
	tmp=TIM4_PERIOD*duty;
	TIM_SetCompare1(TIM4,tmp);
	return duty;
}
float SetPWM10(float duty)
{
	u16 tmp;
	if(duty<0)
	{
		duty=0;
	}
	else if(duty>1.f)
	{
		duty=1.f;
	}
	tmp=TIM4_PERIOD*duty;
	TIM_SetCompare2(TIM4,tmp);
	return duty;
}
float SetPWM11(float duty)
{
	u16 tmp;
	if(duty<0)
	{
		duty=0;
	}
	else if(duty>1.f)
	{
		duty=1.f;
	}
	tmp=TIM4_PERIOD*duty;
	TIM_SetCompare3(TIM4,tmp);
	return duty;
}
float SetPWM12(float duty)
{
	u16 tmp;
	if(duty<0)
	{
		duty=0;
	}
	else if(duty>1.f)
	{
		duty=1.f;
	}
	tmp=TIM4_PERIOD*duty;
	TIM_SetCompare4(TIM4,tmp);
	return duty;
}


float SetPWM13(float duty)
{
	u16 tmp;
	if(duty<0)
	{
		duty=0;
	}
	else if(duty>1.f)
	{
		duty=1.f;
	}
	tmp=TIM5_PERIOD*duty;
	TIM_SetCompare1(TIM5,tmp);
	return duty;
}
float SetPWM14(float duty)
{
	u16 tmp;
	if(duty<0)
	{
		duty=0;
	}
	else if(duty>1.f)
	{
		duty=1.f;
	}
	tmp=TIM5_PERIOD*duty;
	TIM_SetCompare2(TIM5,tmp);
	return duty;
}


float SetPWM15(float duty)
{
	u16 tmp;
	if(duty<0)
	{
		duty=0;
	}
	else if(duty>1.f)
	{
		duty=1.f;
	}
	tmp=TIM8_PERIOD*duty;
	TIM_SetCompare1(TIM8,tmp);
	return duty;
}
float SetPWM16(float duty)
{
	u16 tmp;
	if(duty<0)
	{
		duty=0;
	}
	else if(duty>1.f)
	{
		duty=1.f;
	}
	tmp=TIM8_PERIOD*duty;
	TIM_SetCompare2(TIM8,tmp);
	return duty;
}
float SetPWM17(float duty)
{
	u16 tmp;
	if(duty<0)
	{
		duty=0;
	}
	else if(duty>1.f)
	{
		duty=1.f;
	}
	tmp=TIM8_PERIOD*duty;
	TIM_SetCompare3(TIM8,tmp);
	return duty;
}
float SetPWM18(float duty)
{
	u16 tmp;
	if(duty<0)
	{
		duty=0;
	}
	else if(duty>1.f)
	{
		duty=1.f;
	}
	tmp=TIM8_PERIOD*duty;
	TIM_SetCompare4(TIM8,tmp);
	return duty;
}






