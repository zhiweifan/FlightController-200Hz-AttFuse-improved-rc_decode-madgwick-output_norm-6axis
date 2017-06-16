
#include "att_kalman.h"
#include "kalman_lib.h"
#include "math.h"
#include "matrix.h"
//#include "math_lib.h"
#include "madgwickahrs.h"
#define M_PI  		((float)3.1415926535)
#define M_2PI  		((float)6.283185307)
#define R2D    		((float)57.29577951472)
#define D2R    		((float)0.017453292519444)
//#define GRAVITY  	((float)9.794f)
	

//#define Q_ACC														(0.5f)//(0.003f)//(0.009f)
//#define Q_ACC_Z													Q_ACC
//#define Q_ROT_SPEED											(1e-4f)//(1e-4f)
//#define Q_ROT_SPEED_Z										Q_ROT_SPEED
//#define Q_ROT_ACC												(0.08f)
//#define Q_ROT_ACC_Z											Q_ROT_ACC
//#define Q_MAG														(0.005f/12)//(0.005f)
//#define Q_ROT_SPEED_BIAS								(1e-5f)

//#define R_ACC														(10000.0f)//(3300.0f)//(10000.0f)
//#define R_ACC_Z													R_ACC
//#define R_GYRO													(0.0008f)
//#define R_GYRO_Z												R_GYRO
//#define R_MAG														(100.0f)

//#define Q_ACC														(0.01f)//(0.003f)//(0.009f)
//#define Q_ACC_Z													Q_ACC
//#define Q_ROT_SPEED											(0.01f)//(1e-4f)
//#define Q_ROT_SPEED_Z										Q_ROT_SPEED
//#define Q_ROT_ACC												(2.2f)
//#define Q_ROT_ACC_Z											Q_ROT_ACC
//#define Q_MAG														(0.001f)//(0.005f)
//#define Q_ROT_SPEED_BIAS								(1e-5f)

//#define R_ACC														(3.0f)//(3300.0f)//(10000.0f)
//#define R_ACC_Z													R_ACC
//#define R_GYRO													(0.0008f)
//#define R_GYRO_Z												R_GYRO
//#define R_MAG														(1000.0f)


#define Q_ACC														(0.001f)//(0.003f)//(0.009f)
#define Q_ACC_Z													Q_ACC
#define Q_ROT_SPEED											(0.001f)//(1e-4f)
#define Q_ROT_SPEED_Z										0.1f
#define Q_ROT_ACC												(30.2f)
#define Q_ROT_ACC_Z											1.2f
#define Q_MAG														(0.001f)//(0.005f)
#define Q_ROT_SPEED_BIAS								(1e-5f)

#define R_ACC														(200.0f)//(3300.0f)//(10000.0f)
#define R_ACC_Z													R_ACC
#define R_GYRO													(0.1f)
#define R_GYRO_Z												R_GYRO
#define R_MAG														(2000.0f)



//t8_sram:20kb

YAW_KALMAN_TypeDef yaw_kalman;
ATT_KALMAN_TypeDef att_kalman;
ATT_KALMAN_QR_TypeDef att_kalman_qr;

void YAW_InputsTranslate(KALMAN_PARAMS_TypeDef *pkalman);
void YAW_CrossMulTrans(float* res,float vector,char transpose,float rate);
void DATA_Norm(float *data,int size);

float att_abs(float x)
{
	return x>0?x:-x;
}
void YAW_KALMAN_Config(void)
{
	
	yaw_kalman.P[0][0]=1.01f;
	yaw_kalman.P[1][1]=1.01f;
	yaw_kalman.P[2][2]=1.01f;
	yaw_kalman.P[3][3]=1.01f;
	yaw_kalman.P[4][4]=1.01f;
	
	yaw_kalman.x[0]=0.1f;
	yaw_kalman.x[1]=0.1f;
	yaw_kalman.x[2]=0.0f;
	yaw_kalman.x[3]=0.0f;
	yaw_kalman.x[4]=0.0f;
}



//x:gravity_body[3],accel_ground[3],w_body[3],ww_body[3],w_bias[3]
//z:accel_sensor[3],gyro_sensor[3],GRAVITY[3]
void ATT_KALMAN_Config(void)
{
	
	ATT_KALMAN_SetQR();
	YAW_KALMAN_Config();
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
void ATT_Update(float ax,float ay,float az,float gx,float gy,float gz,float mx,float my,float mz,char cal_yaw,float deltat)
{
	float pitch_r,roll_r;
	float yaw_r,tmp2[2][2];
	KALMAN_PARAMS_TypeDef kalman;
	AHRS_Update6(gx,gy,gz,ax,ay,az,0.05f,deltat);
	AHRS_GetAngle(&yaw_r,&pitch_r,&roll_r);

	att_kalman.pitch_r=pitch_r;
	att_kalman.roll_r=roll_r;
//	att_kalman.yaw=yaw_r*R2D;
	att_kalman.pitch=pitch_r*R2D;
	att_kalman.roll=roll_r*R2D;
	
	att_kalman.gx=gx;
	att_kalman.gy=gy;
	att_kalman.gz=gz;
	
	att_kalman.cos_roll = cos(roll_r);
	att_kalman.sin_roll = sin(roll_r);
	att_kalman.cos_pitch = cos(pitch_r);
	att_kalman.sin_pitch = sin(pitch_r);
	att_kalman.ax_line=(att_kalman.cos_pitch*ax+(att_kalman.sin_roll*att_kalman.sin_pitch)*ay+2*(att_kalman.cos_roll*att_kalman.sin_pitch)*az)*GRAVITY;
	att_kalman.ay_line=((att_kalman.cos_roll)*ay+(-att_kalman.sin_roll)*az)*GRAVITY;
	att_kalman.az_line=(-att_kalman.sin_pitch*ax+(att_kalman.sin_roll*att_kalman.cos_pitch)*ay+(att_kalman.cos_roll*att_kalman.cos_pitch)*az-1.f)*GRAVITY;///0.9775
	
	
	if(cal_yaw)
	{
		yaw_kalman.deltat=att_kalman.deltat;
		YAW_CrossMulTrans((float*)tmp2,yaw_kalman.x[2],1,yaw_kalman.deltat*0.5f);
		tmp2[0][0]=1;
		tmp2[1][1]=1;
		yaw_kalman.A[0][0]=tmp2[0][0];
		yaw_kalman.A[0][1]=tmp2[0][1];
		yaw_kalman.A[1][0]=tmp2[1][0];
		yaw_kalman.A[1][1]=tmp2[1][1];
		
		yaw_kalman.A[0][2]=yaw_kalman.x[1]*yaw_kalman.deltat*0.5f;
		yaw_kalman.A[1][2]=-yaw_kalman.x[0]*yaw_kalman.deltat*0.5f;
		
		yaw_kalman.A[2][2]=1;
		yaw_kalman.A[2][3]=yaw_kalman.deltat;
		yaw_kalman.A[3][3]=1;
		yaw_kalman.A[4][4]=1;
		
		yaw_kalman.C[0][0]=1;
		yaw_kalman.C[1][1]=1;
		yaw_kalman.C[2][2]=1;
		yaw_kalman.C[2][4]=1;
		
		yaw_kalman.z[0]=mx * att_kalman.cos_pitch + my * att_kalman.sin_roll * att_kalman.sin_pitch + mz * att_kalman.cos_roll * att_kalman.sin_pitch;
		yaw_kalman.z[1]=my* att_kalman.cos_roll - mz * att_kalman.sin_roll;
		yaw_kalman.z[2]=gz*D2R;
		YAW_InputsTranslate(&kalman);
		if(!KALMAN_Update(&kalman))
		{
			while(1);
		}
		//DATA_Norm((float*)yaw_kalman.x,2);
		yaw_r=atan2(-yaw_kalman.x[1],yaw_kalman.x[0]);
		yaw_kalman.yaw_r=yaw_r;
		yaw_kalman.yaw=yaw_r*R2D;
		yaw_kalman.gz=yaw_kalman.x[2]*R2D;
		
	}
}


void YAW_InputsTranslate(KALMAN_PARAMS_TypeDef *pkalman)
{
	pkalman->A=(float*)yaw_kalman.A;
	pkalman->C=(float*)yaw_kalman.C;
	pkalman->P=(float*)yaw_kalman.P;
	pkalman->Q=(float*)yaw_kalman.Q;
	pkalman->R=(float*)yaw_kalman.R;
	pkalman->x=(float*)yaw_kalman.x;
	pkalman->z=(float*)yaw_kalman.z;
	pkalman->x_size=YAW_X_SIZE;
	pkalman->z_size=YAW_Z_SIZE;
}

//res'=-res
void ATT_CrossMulTrans(float* res,float* vector,char transpose,float rate)
{
	if(transpose)
	{
		res[0]=0;
		res[1]=vector[2]*rate;
		res[2]=-vector[1]*rate;
		res[3]=-vector[2]*rate;
		res[4]=0;
		res[5]=vector[0]*rate;
		res[6]=vector[1]*rate;
		res[7]=-vector[0]*rate;
		res[8]=0;
	}
	else
	{
		res[0]=0;
		res[1]=-vector[2]*rate;
		res[2]=vector[1]*rate;
		res[3]=vector[2]*rate;
		res[4]=0;
		res[5]=-vector[0]*rate;
		res[6]=-vector[1]*rate;
		res[7]=vector[0]*rate;
		res[8]=0;
	}
	
}
//res'=-res
void YAW_CrossMulTrans(float* res,float vector,char transpose,float rate)
{
	if(transpose)
	{
		res[0]=0;
		res[1]=vector*rate;
		
		res[2]=-vector*rate;
		res[3]=0;
		
	}
	else
	{
		res[0]=0;
		res[1]=-vector*rate;
		
		res[2]=vector*rate;
		res[3]=0;
	
	}
	
}
void DATA_Norm(float *data,int size)
{
	float sum=0;
	int i;
	for(i=0;i<size;i++)
	{
		sum+=data[i]*data[i];
	}
	sum=sqrt(sum);
	for(i=0;i<size;i++)
	{
		data[i]=data[i]/sum;
	}
}
void ATT_KALMAN_SetQR(void)
{
	
	//x:yaw_x,yaw_y,w,ww,w_bias
	//z:yaw_x,yaw_y,gyro_z
	yaw_kalman.Q[0][0]=att_kalman_qr.q_mag;
	yaw_kalman.Q[1][1]=att_kalman_qr.q_mag;
	yaw_kalman.Q[2][2]=att_kalman_qr.q_rot_speed_z;
	yaw_kalman.Q[3][3]=att_kalman_qr.q_rot_acc_z;
	yaw_kalman.Q[4][4]=att_kalman_qr.q_rot_speed_bias;
	
	//z:yaw_x,yaw_y
	yaw_kalman.R[0][0]=att_kalman_qr.r_mag;
	yaw_kalman.R[1][1]=att_kalman_qr.r_mag;
	//z:gyro_z
	yaw_kalman.R[2][2]=att_kalman_qr.r_gyro_z;
}


void ATT_KALMAN_SetDefaultQR(void)
{
	att_kalman_qr.q_acc=Q_ACC;
	att_kalman_qr.q_acc_z=Q_ACC_Z;
	att_kalman_qr.q_rot_speed=Q_ROT_SPEED;
	att_kalman_qr.q_rot_speed_z=Q_ROT_SPEED_Z;
	att_kalman_qr.q_rot_acc=Q_ROT_ACC;
	att_kalman_qr.q_rot_acc_z=Q_ROT_ACC_Z;
	att_kalman_qr.q_rot_speed_bias=Q_ROT_SPEED_BIAS;
	att_kalman_qr.q_mag=Q_MAG;
	att_kalman_qr.r_acc=R_ACC;
	att_kalman_qr.r_acc_z=R_ACC_Z;
	att_kalman_qr.r_gyro=R_GYRO;
	att_kalman_qr.r_gyro_z=R_GYRO_Z;
	att_kalman_qr.r_mag=R_MAG;
}









