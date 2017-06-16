
#include "att_kalman.h"
#include "kalman_lib.h"
#include "time.h"
#include "math.h"
#include "matrix.h"
#define M_PI  ((float)3.1415926535)
#define M_2PI  ((float)6.283185307)
#define R2D    ((float)57.29577951472)
#define D2R    ((float)0.017453292519444)
	
//t8_sram:20kb
ATT_KALMAN_TypeDef att_kalman;

void ATT_InputsTranslate(KALMAN_PARAMS_TypeDef *pkalman);
//x:gravity_body[3],accel_ground[3],w_body[3],ww_body[3],w_bias[3]
//z:accel_sensor[3],gyro_sensor[3],GRAVITY[3]
void ATT_KALMAN_Config(void)
{
	//gravity_body
  att_kalman.Q[0][0]=0.01f;
	att_kalman.Q[1][1]=0.01f;
	att_kalman.Q[2][2]=0.01f;
	//accel_ground
	att_kalman.Q[3][3]=0.01f;
	att_kalman.Q[4][4]=0.01f;
	att_kalman.Q[5][5]=0.01f;
	//w_body
	att_kalman.Q[6][6]=0.1f;
	att_kalman.Q[7][7]=0.1f;
	att_kalman.Q[8][8]=0.1f;
	//ww_body
	att_kalman.Q[9][9]=0.5f;
	att_kalman.Q[10][10]=0.5f;
	att_kalman.Q[11][11]=0.5f;
	//w_bias
	att_kalman.Q[12][12]=0.01f;
	att_kalman.Q[13][13]=0.01f;
	att_kalman.Q[14][14]=0.01f;
	
	//z:accel_sensor
	att_kalman.R[0][0]=0.1f;
	att_kalman.R[1][1]=0.1f;
	att_kalman.R[2][2]=0.1f;
	//z:gyro_sensor
	att_kalman.R[3][3]=1.01f;
	att_kalman.R[4][4]=1.01f;
	att_kalman.R[5][5]=1.01f;
	//z:GRAVITY
	att_kalman.R[6][6]=0.001f;
	att_kalman.R[7][7]=0.001f;
	att_kalman.R[8][8]=0.001f;
	
	att_kalman.P[0][0]=0.01f;
	att_kalman.P[1][1]=0.01f;
	att_kalman.P[2][2]=0.01f;
	att_kalman.P[3][3]=0.01f;
	att_kalman.P[4][4]=0.01f;
	att_kalman.P[5][5]=0.01f;
	att_kalman.P[6][6]=0.01f;
	att_kalman.P[7][7]=0.01f;
	att_kalman.P[8][8]=0.01f;
	att_kalman.P[9][9]=0.01f;
	att_kalman.P[10][10]=0.01f;
	att_kalman.P[11][11]=0.01f;
	att_kalman.P[12][12]=0.01f;
	att_kalman.P[13][13]=0.01f;
	att_kalman.P[14][14]=0.01f;
	
	//gravity_body
	att_kalman.x[0]=0;
	att_kalman.x[1]=0;
	att_kalman.x[2]=0;
	//accel_ground
	att_kalman.x[3]=0;
	att_kalman.x[4]=0;
	att_kalman.x[5]=0;
	//w_body
	att_kalman.x[6]=0;
	att_kalman.x[7]=0;
	att_kalman.x[8]=0;
	//ww_body
	att_kalman.x[9]=0;
	att_kalman.x[10]=0;
	att_kalman.x[11]=0;
	//w_bias
	att_kalman.x[12]=0;
	att_kalman.x[13]=0;
	att_kalman.x[14]=0;

	att_kalman.RotM[0][0]=1;
	att_kalman.RotM[1][1]=1;
	att_kalman.RotM[2][2]=1;
}
float ATT_LimitAngle(float a)
{
	if(a>M_PI)
	{
		a=a-M_PI*2;
	}
	else if(a<-M_PI)
	{
		a=a+M_PI*2;
	}
	return a;
}
//a_x_y_z unit :GRAVITY
//g_x_y_z unit:degree/s
void ATT_Update(float ax,float ay,float az,float gx,float gy,float gz)
{
	float gx_r,gy_r,gz_r;
	float yaw_r,pitch_r,roll_r;
	float rotm_pred[3][3];
	float rotm_yaw[3][3];
	float rotm_tmp[3][3];
	float c1,c2,c3,s1,s2,s3;
	float c2c3,s1s2c3,c1s3,s1s3,c1s2c3,c2s3,c1c3,s1s2s3,c1s2s3,s1c3,s1c2,c1c2;
	KALMAN_PARAMS_TypeDef kalman;
	att_kalman.deltat=(float)TIME_GetTimeUs(TIME_ATT)/1000000.0f;
	
	gx_r=gx*D2R;
	gy_r=gy*D2R;
	gz_r=gz*D2R;
	//last angle
	yaw_r=att_kalman.yaw_r;
	pitch_r=att_kalman.pitch_r;
	roll_r=att_kalman.roll_r;
	
	c1=cos(roll_r);
	c2=cos(pitch_r);
	c3=cos(yaw_r);
	s1=sin(roll_r);
	s2=sin(pitch_r);
	s3=sin(yaw_r);
	
//	c1=1.f;
//	c2=0;
//	c3=0;
//	s1=0;
//	s2=0;
//	s3=0;
	
	c2c3=c2*c3;
	s1c3=s1*c3;
	s1c2=s1*c2;
	c1c2=c1*c2;
	c1s3=c1*s3;
	s1s3=s1*s3;
	c2s3=c2*s3;
	c1c3=c1*c3;
	s1s2c3=s1c3*s2;
	c1s2c3=c1*s2*c3;
	s1s2s3=s1s3*s2;
	c1s2s3=c1s3*s2;
	
	att_kalman.RotM[0][0]=c2c3;
	att_kalman.RotM[0][1]=s1s2c3-c1s3;
	att_kalman.RotM[0][2]=s1s3+c1s2c3;
	att_kalman.RotM[1][0]=c2s3;
	att_kalman.RotM[1][1]=c1c3+s1s2s3;
	att_kalman.RotM[1][2]=c1s2s3-s1c3;
	att_kalman.RotM[2][0]=-s2;
	att_kalman.RotM[2][1]=s1c2;
	att_kalman.RotM[2][2]=c1c2;
	
	rotm_yaw[0][0]=c3;
	rotm_yaw[0][1]=-s3;
	rotm_yaw[0][2]=0;
	rotm_yaw[1][0]=s3;
	rotm_yaw[1][1]=c3;
	rotm_yaw[1][2]=0;
	rotm_yaw[2][0]=0;
	rotm_yaw[2][1]=0;
	rotm_yaw[2][2]=1.f;
	
	//angle predict
	yaw_r=ATT_LimitAngle(att_kalman.yaw_r+gz_r*att_kalman.deltat);
	pitch_r=ATT_LimitAngle(att_kalman.pitch_r+gy_r*att_kalman.deltat);
	roll_r=ATT_LimitAngle(att_kalman.roll_r+gx_r*att_kalman.deltat);
	
	c1=cos(roll_r);
	c2=cos(pitch_r);
	c3=cos(yaw_r);
	s1=sin(roll_r);
	s2=sin(pitch_r);
	s3=sin(yaw_r);
	
//	c1=1.f;
//	c2=0;
//	c3=0;
//	s1=0;
//	s2=0;
//	s3=0;
	
	c2c3=c2*c3;
	s1c3=s1*c3;
	s1c2=s1*c2;
	c1c2=c1*c2;
	c1s3=c1*s3;
	s1s3=s1*s3;
	c2s3=c2*s3;
	c1c3=c1*c3;
	s1s2c3=s1c3*s2;
	c1s2c3=c1*s2*c3;
	s1s2s3=s1s3*s2;
	c1s2s3=c1s3*s2;
	
	rotm_pred[0][0]=c2c3;
	rotm_pred[0][1]=s1s2c3-c1s3;
	rotm_pred[0][2]=s1s3+c1s2c3;
	rotm_pred[1][0]=c2s3;
	rotm_pred[1][1]=c1c3+s1s2s3;
	rotm_pred[1][2]=c1s2s3-s1c3;
	rotm_pred[2][0]=-s2;
	rotm_pred[2][1]=s1c2;
	rotm_pred[2][2]=c1c2;
	
	MATRIX_AT_B((float*)rotm_tmp,(float*)rotm_pred,(float*)att_kalman.RotM,3,3,3);
	//gravity_body=rotm_pred'*att_kalman.RotM*gravity_body
	att_kalman.A[0][0]=rotm_tmp[0][0];
	att_kalman.A[0][1]=rotm_tmp[0][1];
	att_kalman.A[0][2]=rotm_tmp[0][2];
	att_kalman.A[1][0]=rotm_tmp[1][0];
	att_kalman.A[1][1]=rotm_tmp[1][1];
	att_kalman.A[1][2]=rotm_tmp[1][2];
	att_kalman.A[2][0]=rotm_tmp[2][0];
	att_kalman.A[2][1]=rotm_tmp[2][1];
	att_kalman.A[2][2]=rotm_tmp[2][2];
	//accel_ground=accel_ground
	att_kalman.A[3][3]=1.f;
	att_kalman.A[4][4]=1.f;
	att_kalman.A[5][5]=1.f;
	//w_body=w_body+deltat*ww_body
	att_kalman.A[6][6]=1.f;
	att_kalman.A[7][7]=1.f;
	att_kalman.A[8][8]=1.f;
	
	att_kalman.A[6][9]=att_kalman.deltat;
	att_kalman.A[7][10]=att_kalman.deltat;
	att_kalman.A[8][11]=att_kalman.deltat;
	//ww_body=ww_body
	att_kalman.A[9][9]=1.f;
	att_kalman.A[10][10]=1.f;
	att_kalman.A[11][11]=1.f;
	//w_bias=w_bias
	att_kalman.A[12][12]=1.f;
	att_kalman.A[13][13]=1.f;
	att_kalman.A[14][14]=1.f;
	
	MATRIX_AT_B((float*)rotm_tmp,(float*)rotm_pred,(float*)rotm_yaw,3,3,3);
	//accel_sensor=gravity_body+rotm_pred'*rotm_yaw*accel_ground
	att_kalman.C[0][0]=1.f;
	att_kalman.C[1][1]=1.f;
	att_kalman.C[2][2]=1.f;
	
	att_kalman.C[0][3]=rotm_tmp[0][0];
	att_kalman.C[0][4]=rotm_tmp[0][1];
	att_kalman.C[0][5]=rotm_tmp[0][2];
	att_kalman.C[1][3]=rotm_tmp[1][0];
	att_kalman.C[1][4]=rotm_tmp[1][1];
	att_kalman.C[1][5]=rotm_tmp[1][2];
	att_kalman.C[2][3]=rotm_tmp[2][0];
	att_kalman.C[2][4]=rotm_tmp[2][1];
	att_kalman.C[2][5]=rotm_tmp[2][2];
	
	//gyro_sensor=w_body+w_bias
	att_kalman.C[3][6]=1.f;
	att_kalman.C[4][7]=1.f;
	att_kalman.C[5][8]=1.f;
	
	att_kalman.C[3][12]=1.f;
	att_kalman.C[4][13]=1.f;
	att_kalman.C[5][14]=1.f;
	
	//GRAVITY=rotm_pred*gravity_body
	att_kalman.C[6][0]=rotm_pred[0][0];
	att_kalman.C[6][1]=rotm_pred[0][0];
	att_kalman.C[6][2]=rotm_pred[0][0];
	att_kalman.C[7][0]=rotm_pred[0][0];
	att_kalman.C[7][1]=rotm_pred[0][0];
	att_kalman.C[7][2]=rotm_pred[0][0];
	att_kalman.C[8][0]=rotm_pred[0][0];
	att_kalman.C[8][1]=rotm_pred[0][0];
	att_kalman.C[8][2]=rotm_pred[0][0];
	
	att_kalman.z[0]=ax;
	att_kalman.z[1]=ay;
	att_kalman.z[2]=az;
	
	att_kalman.z[3]=gx_r;
	att_kalman.z[4]=gy_r;
	att_kalman.z[5]=gz_r;
	
	att_kalman.z[6]=0;
	att_kalman.z[7]=0;
	att_kalman.z[8]=1.f;
	
	ATT_InputsTranslate(&kalman);
	if(!KALMAN_Update(&kalman))
	{
		while(1);
	}
	roll_r=ATT_LimitAngle(att_kalman.yaw_r+att_kalman.x[6]*att_kalman.deltat);
	pitch_r=ATT_LimitAngle(att_kalman.pitch_r+att_kalman.x[7]*att_kalman.deltat);
	yaw_r=ATT_LimitAngle(att_kalman.roll_r+att_kalman.x[8]*att_kalman.deltat);
	att_kalman.yaw_r=yaw_r;
	att_kalman.pitch_r=pitch_r;
	att_kalman.roll_r=roll_r;
	att_kalman.yaw=yaw_r*R2D;
	att_kalman.pitch=pitch_r*R2D;
	att_kalman.roll=roll_r*R2D;
}


void ATT_InputsTranslate(KALMAN_PARAMS_TypeDef *pkalman)
{
	pkalman->A=(float*)att_kalman.A;
	pkalman->C=(float*)att_kalman.C;
	pkalman->P=(float*)att_kalman.P;
	pkalman->Q=(float*)att_kalman.Q;
	pkalman->R=(float*)att_kalman.R;
	pkalman->x=(float*)att_kalman.x;
	pkalman->z=(float*)att_kalman.z;
	pkalman->x_size=ATT_X_SIZE;
	pkalman->z_size=ATT_Z_SIZE;
}
