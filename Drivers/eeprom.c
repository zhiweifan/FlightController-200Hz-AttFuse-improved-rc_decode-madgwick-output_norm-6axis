
#include "eeprom.h"
#include "stm32f4xx_flash.h"
#include "main.h"

//#define PARAMS_HEAD         0x5a

//不能写成const uint32_t table[PARAMS_SIZE+2] __attribute__((at(PAGE_Config))) = {0x55} ;//因为擦除的最小单位是一页
//const uint32_t PARAMS_REGION[1024*128/4] __attribute__((at(DATAMAP_ADDR))) = {0x55} ;//防止误擦除
const uint32_t PARAMS_REGION[REGION_SIZE/4] __attribute__((at(DATAMAP_ADDR)));

volatile PARAMS_TypeDef SysConfig;	

void load_params(u32* src);
//void save_params(PARAMS_TypeDef* des);

void MemoryCopy(u32* src,u32* des,u32 length)
{
	u32 i;
	for(i=0;i<length;i++)
	{
		des[i]=src[i];
	}
}
void MemoryCopy2(u32* des,u32* src,u32 length)
{
	u32 i;
	for(i=0;i<length;i++)
	{
		des[i]=src[i];
	}
}
char LoadConfig(void)
{
	u32 i=0,jy=0;
	struct data_map *temp_addr;
//	temp_addr= (struct data_map *)PARAMS_REGION;
	temp_addr= (struct data_map *)DATAMAP_ADDR;
	if(temp_addr->cnt==PARAMS_SIZE)
	{
		for(i=0;i<temp_addr->cnt;i++)
		{
			jy+=*(((u32*)&temp_addr->params)+i);
		}
	
		if(jy==temp_addr->jy)
		{
			load_params((u32*)&temp_addr->params);
			return 1;
		}
	}
	else
	{
		//while(1);
	}
	return 0;
}

//将当前配置写入flash
char WriteConfig(void)
{
	u32 i=0,jy=0;
	struct data_map *temp_addr;
	uint32_t* pdata;
//	temp_addr= (struct data_map *)PARAMS_REGION;
	temp_addr= (struct data_map *)DATAMAP_ADDR;
	pdata=(u32*)&temp_addr->params;
//	save_params(&temp_addr->params);

//	jy=PARAMS_HEAD;
	for(i=0;i<PARAMS_SIZE;i++)
	{
		jy+=*(((u32*)&SysConfig)+i);
	}
	
	FLASH_Unlock();
	FLASH_ClearFlag(FLASH_FLAG_EOP |FLASH_FLAG_OPERR|FLASH_FLAG_WRPERR|FLASH_FLAG_PGAERR|FLASH_FLAG_PGPERR|FLASH_FLAG_PGSERR);//20150809增加
	FLASH_EraseSector(FLASH_SECTOR,VoltageRange_3); 
	FLASH_ProgramWord((u32)&temp_addr->cnt,PARAMS_SIZE);
	FLASH_ProgramWord((u32)&temp_addr->jy,jy);
	for(i=0;i<PARAMS_SIZE;i++)
	{
		FLASH_ProgramWord((u32)(pdata+i),*(((u32*)&SysConfig)+i));
	}
	FLASH_Lock();
	return 1;
}

void load_params(u32* src)
{
	MemoryCopy(src,(u32*)&SysConfig,PARAMS_SIZE);
}

char GetConfig(void)
{
	char res;
	res=LoadConfig();
	if(!res)
	{
		return 0;
	}
	
	SysConfig.duty_max=LIMIT_MIN_MAX(SysConfig.duty_max,DUTY_UP_MIN,DUTY_UP_MAX);
	SysConfig.duty_min=LIMIT_MIN_MAX(SysConfig.duty_min,DUTY_LOW_MIN,DUTY_UP_MAX);
	
	SysConfig.bias_pitch=LIMIT_MIN_MAX(SysConfig.bias_pitch,-10.f,10.f);
	SysConfig.bias_roll=LIMIT_MIN_MAX(SysConfig.bias_roll,-10.f,10.f);
	
	CtrlSys.duty_max=SysConfig.duty_max;
	CtrlSys.duty_min=SysConfig.duty_min;
	CtrlSys.bias_pitch=SysConfig.bias_pitch;
	CtrlSys.bias_roll=SysConfig.bias_roll;
	CtrlSys.test_mode=SysConfig.test_mode;
	
	CtrlSys.acc_line_z_bias=SysConfig.acc_line_z_bias;
	CtrlSys.yaw_bias=SysConfig.yaw_bias;
	CtrlSys.idling_throttle=SysConfig.idling_throttle;
	CtrlSys.accel_output_bias=SysConfig.accel_output_bias;
	
	*(PID_PARAMS_TypeDef*)&PitchPID=SysConfig.PitchPID;
	*(PID_PARAMS_TypeDef*)&PitchRatePID=SysConfig.PitchRatePID;
	*(PID_PARAMS_TypeDef*)&RollPID=SysConfig.RollPID;
	*(PID_PARAMS_TypeDef*)&RollRatePID=SysConfig.RollRatePID;
	*(PID_PARAMS_TypeDef*)&YawPID=SysConfig.YawPID;
	*(PID_PARAMS_TypeDef*)&YawRatePID=SysConfig.YawRatePID;
	*(PID_PARAMS_TypeDef*)&AccPID=SysConfig.AccPID;
	*(PID_PARAMS_TypeDef*)&HeightPID=SysConfig.HeightPID;
	*(PID_PARAMS_TypeDef*)&HeightRatePID=SysConfig.HeightRatePID;
	imu_calib_params=SysConfig.imu_calib_params;
	alt_filter_qr=SysConfig.alt_filter_qr;
	return 1;
}
char SetConfig(void)
{

	SysConfig.duty_max=CtrlSys.duty_max;
	SysConfig.duty_min=CtrlSys.duty_min;
	SysConfig.bias_pitch=CtrlSys.bias_pitch;
	SysConfig.bias_roll=CtrlSys.bias_roll;
	if(CtrlSys.test_mode==2||CtrlSys.test_mode==3)
	{
		SysConfig.test_mode=CtrlSys.test_mode;
	}
	else
	{
		SysConfig.test_mode=0;
	}
	SysConfig.acc_line_z_bias=CtrlSys.acc_line_z_bias;
	SysConfig.yaw_bias=CtrlSys.yaw_bias;
	SysConfig.idling_throttle=CtrlSys.idling_throttle;
	SysConfig.accel_output_bias=CtrlSys.accel_output_bias;
	
	SysConfig.PitchPID=*(PID_PARAMS_TypeDef*)&PitchPID;
	SysConfig.PitchRatePID=*(PID_PARAMS_TypeDef*)&PitchRatePID;
	SysConfig.RollPID=*(PID_PARAMS_TypeDef*)&RollPID;
	SysConfig.RollRatePID=*(PID_PARAMS_TypeDef*)&RollRatePID;
	SysConfig.YawPID=*(PID_PARAMS_TypeDef*)&YawPID;
	SysConfig.YawRatePID=*(PID_PARAMS_TypeDef*)&YawRatePID;
	SysConfig.AccPID=*(PID_PARAMS_TypeDef*)&AccPID;
	SysConfig.HeightPID=*(PID_PARAMS_TypeDef*)&HeightPID;
	SysConfig.HeightRatePID=*(PID_PARAMS_TypeDef*)&HeightRatePID;
	
	SysConfig.imu_calib_params=imu_calib_params;
	SysConfig.alt_filter_qr=alt_filter_qr;
	WriteConfig();
	return 1;
}

//------------------End of File----------------------------
