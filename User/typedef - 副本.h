
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
volatile char load_success;//���ز����ɹ�
volatile char test_mode;//0������ģʽ1���������2���趨����г�
volatile char stop;//����ģʽ��Ч
volatile char hovering_mode;//��ͣģʽ
}CtrlSys_TypeDef;

typedef struct
{
volatile	float longitude;//����
volatile	float latitude;//γ��
volatile	float altitude;//����
volatile	float course;//����
volatile	float speed;//�ٶ�
volatile	unsigned char satellite_num;//��������
}GPS_TypeDef;








#endif



