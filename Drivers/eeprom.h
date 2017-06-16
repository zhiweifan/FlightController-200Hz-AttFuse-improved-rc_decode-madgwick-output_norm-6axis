#ifndef __EEPROM_H
#define __EEPROM_H

#include "stm32f4xx.h"
#include "pid.h"
#include "imu.h"
#include "att_kalman.h"
#include "alt_filter.h"

//STM32F10xPQ
//P=T(36),C(48),R(64),V(100),Z(144)
//Q=4(16K),6(32K),8(64K),B(128K),C(256K),D(384K),E(512K),G(1M)
//low-density devices(32KB,1KB/page)
//medium-density devices(128KB,1KB/page)
//high-density devices(512KB,2KB/page)
//connectivity line devices(256KB,2KB/page)
//XL-density(devices(1M,2KB/page)

//stm32f4xx分区
// ((uint32_t)0x08000000) /* Base @ of Sector 0, 16 Kbytes */  
// ((uint32_t)0x08004000) /* Base @ of Sector 1, 16 Kbytes */ 
// ((uint32_t)0x08008000) /* Base @ of Sector 2, 16 Kbytes */ 
// ((uint32_t)0x0800C000) /* Base @ of Sector 3, 16 Kbytes */ 
// ((uint32_t)0x08010000) /* Base @ of Sector 4, 64 Kbytes */ 
// ((uint32_t)0x08020000) /* Base @ of Sector 5, 128 Kbytes */ 
// ((uint32_t)0x08040000) /* Base @ of Sector 6, 128 Kbytes */ 
// ((uint32_t)0x08060000) /* Base @ of Sector 7, 128 Kbytes */  
// ((uint32_t)0x08080000) /* Base @ of Sector 8, 128 Kbytes */   
// ((uint32_t)0x080A0000) /* Base @ of Sector 9, 128 Kbytes */   
// ((uint32_t)0x080C0000) /* Base @ of Sector 10, 128 Kbytes */ 
// ((uint32_t)0x080E0000) /* Base @ of Sector 11, 128 Kbytes */

//#define  PAGE_SIZE			2048
#define REGION_SIZE				(16*1024)
#define FLASH_SECTOR			FLASH_Sector_1
#define DATAMAP_ADDR			((uint32_t)0x08004000)//将配置信息存放在第1区(不要放置在0区，否则无法运行)


#define PARAMS_SIZE  				(sizeof(PARAMS_TypeDef)/4) //4字节为单位


typedef __packed struct
{
	PID_PARAMS_TypeDef PitchPID;
	PID_PARAMS_TypeDef PitchRatePID;
	PID_PARAMS_TypeDef RollPID;
	PID_PARAMS_TypeDef RollRatePID;
	PID_PARAMS_TypeDef YawPID;
	PID_PARAMS_TypeDef YawRatePID;
	PID_PARAMS_TypeDef AccPID;
	PID_PARAMS_TypeDef HeightRatePID;
	PID_PARAMS_TypeDef HeightPID;
	u32 test_mode;
	float duty_min;
	float duty_max;
	float bias_pitch;
	float bias_roll;
	float accel_output_bias;
	float acc_line_z_bias;
	float yaw_bias;
	float idling_throttle;
	IMU_CALIB_PARAMS_TypeDef imu_calib_params;
//	ATT_KALMAN_QR_TypeDef att_kalman_qr;
	ALT_FILTER_QR_TypeDef alt_filter_qr;
}PARAMS_TypeDef;

struct data_map{
	u32 cnt;   //PARAMS_TypeDef的大小，4字节为单位
	u32 jy;
	PARAMS_TypeDef params;
};

//char WriteConfig(void);  //写入配置
//char LoadConfig(void);	 //读取配置
//void MemoryCopy(u32* src,u32* des,u32 length);

char GetConfig(void);
char SetConfig(void);
extern volatile PARAMS_TypeDef SysConfig;


#endif /* __EEPROM_H */

//------------------End of File----------------------------
