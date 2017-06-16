
///适用于stm32f4xx

#include "time.h"
#include "misc.h"


#define SYS_FREQ    168  //MHz
//#define SYS_FREQ    72  //MHz



u32 time_ov[TIME_ALL];

#define DRIVE_GetTiick() (SysTick->VAL)
#define DRIVE_TICK_PER   0x1000000


__IO int32_t TimeArray[TIME_ALL],TimeArrayTmp[TIME_ALL];

/******************************************************************************
* @函数名称：void TIME_Init(void)
* @函数描述：时钟初始化
* @输入参数：None
* @输出参数：None
*******************************************************************************/
void TIME_Config(void)
{
//	while(SysTick_Config(SystemCoreClock / 1000000));
	SysTick->LOAD = 0xffffff;
	SysTick->VAL = 0; 
	SysTick->CTRL  = SysTick_CTRL_CLKSOURCE_Msk | SysTick_CTRL_ENABLE_Msk; 
	
}
/******************************************************************************
* @函数名称：uint8_t TIME_CheckTimeUs(uint8_t Number, int32_t Time)  
* @函数描述：校验时间
* @输入参数：时间序号,time最大99.864ms(168MHz)/233ms(72MHz)  //改进后可达12.78秒(168MHz)/29秒(72MHz)
* @输出参数：1：时间完成
*******************************************************************************/
uint8_t TIME_CheckTimeUs(uint8_t Number, int32_t Time) 
{
		int32_t tmp=DRIVE_GetTiick();
		int64_t NewTime = TimeArray[Number] -tmp ;
    int32_t ov = TimeArrayTmp[Number] -tmp ;

#if SYS_FREQ==168	
	  if(Time>90000)
#elif SYS_FREQ==72
		if(Time>233000)
#endif
		{
			if(ov< 0)
			{
					time_ov[Number]++;
			}
			NewTime = (int64_t)time_ov[Number]*DRIVE_TICK_PER + NewTime;
			if(NewTime >= (Time *SYS_FREQ))
			{
				time_ov[Number]=0;
				TimeArrayTmp[Number]=tmp;
				TimeArray[Number] = tmp;
				return 1;
			}
			else
			{
				TimeArrayTmp[Number]=tmp;
				return 0;
			}

		}
		else
		{
			if(NewTime < 0)
			{
					NewTime = DRIVE_TICK_PER + NewTime;
					
			}
			time_ov[Number]=0;
			TimeArrayTmp[Number]=tmp;
			if(NewTime >= (Time * SYS_FREQ))
			{
					TimeArray[Number] = tmp;
					return 1;
			}
		}
    return 0;
}
/******************************************************************************
* @函数名称：int32_t TIME_GetTime(uint8_t Number) 
* @函数描述：获取定时时间,单位1/168us
* @输入参数：时间序号
* @输出参数：时间值
*******************************************************************************/
float TIME_GetTimeUs(uint8_t Number) 
{
		int32_t tmp=DRIVE_GetTiick();
    int32_t NewTime = TimeArray[Number] - tmp;
    if(NewTime < 0)
    {
        NewTime = DRIVE_TICK_PER + NewTime;
    }
		
		TimeArray[Number] = tmp;
    return NewTime/(float)SYS_FREQ;
		
}
/******************************************************************************
* @函数名称：void TIME_Reset(uint8_t Number)
* @函数描述：复位时间，所有的，单个的
* @输入参数：时间序号
* @输出参数：None
*******************************************************************************/
void TIME_Reset(uint8_t Number)
{
	uint8_t i;
	if(Number == TIME_ALL)
	{
		for(i=0; i<TIME_ALL; i++)
		{
			TimeArray[i] = DRIVE_GetTiick();
			time_ov[i]=0;
			TimeArrayTmp[i]=TimeArray[i];
		}
	}
	else
	{
		TimeArray[Number] = DRIVE_GetTiick();
		time_ov[Number]=0;
		TimeArrayTmp[Number]=TimeArray[Number];
	}
	
}
/* 文件结束 --------------------------------------------------------------------*/

