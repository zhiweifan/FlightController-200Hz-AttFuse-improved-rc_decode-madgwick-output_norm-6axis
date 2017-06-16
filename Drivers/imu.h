
#ifndef IMU_H
#define IMU_H
#include "stm32f4xx.h"

typedef struct
{
	char cmd;
	float param;
}IMU_CMD_TypeDef;

typedef __packed struct
{
volatile float acc_bias[3];
volatile float acc_Ae[9];
volatile float acc_coef[9];
volatile float mag_bias[3];
volatile float mag_Ae[9];
volatile float mag_coef[9];
volatile float gyro_bias[3];
}IMU_CALIB_PARAMS_TypeDef;

typedef struct
{
volatile	float yaw;
volatile	float pitch;
volatile	float roll;
volatile	int16_t ax_raw;
volatile	int16_t ay_raw;
volatile	int16_t az_raw;
volatile	int16_t gx_raw;   
volatile	int16_t gy_raw;
volatile	int16_t gz_raw;
volatile	int16_t mx_raw;
volatile	int16_t my_raw;
volatile	int16_t mz_raw;
volatile	float ax;
volatile	float ay;
volatile	float az;
volatile	float gx;  
volatile	float gy;
volatile	float gz;
volatile	float mx;
volatile	float my;
volatile	float mz;
volatile	float ax_line;
volatile	float ay_line;
volatile	float az_line;
volatile	float acc_line_tc;
volatile	float altitude;
}AHRS_TypeDef;

typedef struct
{
volatile	float longitude;//经度
volatile	float latitude;//纬度
volatile	float altitude;//海拔
volatile	float course;//航向
volatile	float speed;//速度
volatile	unsigned char satellite_num;//卫星数量
}GPS_TypeDef;

void IMU_Config(void);
void IMU_Update(void);

extern AHRS_TypeDef ahrs;
extern GPS_TypeDef gps;
extern IMU_CMD_TypeDef imu_cmd;
extern IMU_CALIB_PARAMS_TypeDef imu_calib_params;

#endif


