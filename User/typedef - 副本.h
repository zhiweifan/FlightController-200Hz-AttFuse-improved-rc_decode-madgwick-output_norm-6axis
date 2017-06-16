
#ifndef TYPEDEF_H
#define TYPEDEF_H




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
volatile	float altitude;
}AHRS_TypeDef;

typedef struct
{
volatile char load_success;//加载参数成功
volatile char test_mode;//0：正常模式1：电调测试2：设定电调行程
volatile char stop;//测试模式无效
volatile char hovering_mode;//悬停模式
}CtrlSys_TypeDef;

typedef struct
{
volatile	float longitude;//经度
volatile	float latitude;//纬度
volatile	float altitude;//海拔
volatile	float course;//航向
volatile	float speed;//速度
volatile	unsigned char satellite_num;//卫星数量
}GPS_TypeDef;








#endif



