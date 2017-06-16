
#ifndef MADGWICKAHRS_H
#define MADGWICKAHRS_H

//typedef struct
//{
//volatile	float q0;
//volatile	float q1;
//volatile	float q2;
//volatile	float q3;
//volatile	float yaw_rad;
//volatile	float pitch_rad;
//volatile	float roll_rad;
//}MadgwickAhrs_TypeDef;

void AHRS_Update6(float gx, float gy, float gz, float ax, float ay, float az,float beta,float deltat);
void AHRS_Update9(float gx, float gy, float gz, float ax, float ay, float az, float mx, float my, float mz,float beta,float deltat);
void AHRS_GetAngle(float *yaw_rad,float *pitch_rad,float *roll_rad);
float AHRS_UpdateYaw(float angle_measure,float beta,float deltat) ;
void AHRS_SetQFromAngle(float yaw_rad,float pitch_rad,float roll_rad);
float AHRS_D2R(float degree);
float AHRS_R2D(float rad);
//extern MadgwickAhrs_TypeDef madgwick_ahrs;



#endif


