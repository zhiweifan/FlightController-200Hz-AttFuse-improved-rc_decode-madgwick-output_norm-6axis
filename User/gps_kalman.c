

#include "gps_kalman.h"


#include "kalman_lib.h"
#include "imu.h"
#include "math.h"
#include "time.h"
//地球半径
#define EARTH_RADIUS      6371000.f


//no speed reduction
//#define ALT_FILTER_Q_H										0.1f//(1.5f)
//#define ALT_FILTER_Q_V										0.01f//(0.01f)
//#define ALT_FILTER_Q_A										1.1f//(0.1f)
//#define ALT_FILTER_Q_A_BIAS								0.00001f//(0.01f)
//#define ALT_FILTER_R_H										50000000.0f//(0.3f)
//#define ALT_FILTER_R_A										0.00001f//(1.1f)


//with speed reduction

#define ALT_FILTER_Q_H										0.001f//(1.5f)
#define ALT_FILTER_Q_V										0.01f//(0.01f)
#define ALT_FILTER_Q_A										0.1f//(0.1f)
#define ALT_FILTER_Q_A_BIAS								0.002f//(0.01f)
#define ALT_FILTER_R_H										50000000000.0f//(0.3f)
#define ALT_FILTER_R_A										0.00001f//(1.1f)

GPS_KALMAN_TypeDef gps_kalman;
GPS_KALMAN_QR_TypeDef gps_kalman_qr;

void GPS_KALMAN_InputsTranslate(KALMAN_PARAMS_TypeDef *pkalman);

void GPS_KALMAN_Config(void)
{
	
  GPS_KALMAN_SetQR();
	
	
	gps_kalman.P[0][0]=6530.6543f;
	gps_kalman.P[0][1]=889.28363f;
	gps_kalman.P[0][2]=22.3026142f;
	gps_kalman.P[0][3]=-22.3026142f;
	gps_kalman.P[1][0]=889.28363f;
	gps_kalman.P[1][1]=252.091156f;
	gps_kalman.P[1][2]=6.88223553f;
	gps_kalman.P[1][3]=-6.88223553f;
	gps_kalman.P[2][0]=22.3026142f;
	gps_kalman.P[2][1]=6.88223553f;
	gps_kalman.P[2][2]=0.937909245f;
	gps_kalman.P[2][3]=-0.937899232f;
	gps_kalman.P[3][0]=-22.3026142f;
	gps_kalman.P[3][1]=-6.88223553f;
	gps_kalman.P[3][2]=-0.937899232f;
	gps_kalman.P[3][3]=0.937899172f;
	
	gps_kalman.C[0][0]=1.f;
	gps_kalman.C[1][2]=1.f;
	gps_kalman.C[1][3]=1.f;
	
	gps_kalman.x[0]=0;
	gps_kalman.x[1]=0;
	gps_kalman.x[2]=0;
	gps_kalman.x[3]=CtrlSys.acc_line_z_bias;
	gps_kalman.acc_line_z_tc=0.0f;
}

void GPS_KALMAN_Update(void)
{
	float t1,t2;
	float acc_line_z_tc;
	float tc;
	static float acc_mean;
	static float acc_z_mean=0,acc_z_ok_time=0;
	static int acc_z_is_init=-1;
	KALMAN_PARAMS_TypeDef kalman;
	
	gps_kalman.deltat=(float)TIME_GetTimeUs(TIME_GPS)/1000000.0f;
	if(gps_kalman.deltat<1e-8f)
	{
		gps_kalman.deltat=1e-8f;
	}
	
	t1=gps_kalman.deltat;
	t2=t1*gps_kalman.deltat;

	if(acc_z_is_init<=0)
	{
		if(acc_z_is_init==-1)
		{
			if(ahrs.az_line==0)
			{
				return ;
			}
			else
			{
				acc_z_is_init=0;
				return;
			}
		}
		tc=4.f/gps_kalman.deltat;
		acc_z_mean=(tc*acc_z_mean+ahrs.az_line)/(tc+1.0f);
		if(fabs(acc_z_mean-ahrs.az_line)<0.03f&&fabs(acc_z_mean)<0.6f)
		{
			acc_z_ok_time+=gps_kalman.deltat;
		}
		else
		{
			acc_z_ok_time=0;
		}
		if(acc_z_ok_time>20.f)
		{
			gps_kalman.x[3]=acc_z_mean;
			CtrlSys.acc_line_z_bias=acc_z_mean;
			acc_z_is_init=1;
			CtrlSys.acc_z_is_init=1;
		}
		else
		{
			tc=2.0f/gps_kalman.deltat;
			gps_kalman.x[0]=(gps_kalman.x[0]*tc+ahrs.altitude)/(tc+1.f);
		}
		return ;
	}

	if(CtrlSys.acc_z_is_init!=1)
	{
		return;
	}
	
	//x:3,z:2
	gps_kalman.A[0][0]=1.f;
	gps_kalman.A[0][1]=t1;
	gps_kalman.A[0][2]=t2/2.f;
//	gps_kalman.A[0][3]=t3/6.f;
	gps_kalman.A[1][1]=1.f;
	gps_kalman.A[1][2]=t1;
//	gps_kalman.A[1][3]=t2/2.f;
	gps_kalman.A[2][2]=1.f;
//	gps_kalman.A[2][3]=t1;
	gps_kalman.A[3][3]=1.f;
	
	gps_kalman.z[0]=ahrs.altitude;
	acc_line_z_tc=gps_kalman.acc_line_z_tc/gps_kalman.deltat;
	gps_kalman.z[1]=(acc_line_z_tc*gps_kalman.z[1]+(ahrs.az_line))/(acc_line_z_tc+1.00f);
	GPS_KALMAN_InputsTranslate(&kalman);
	if(!KALMAN_Update(&kalman))
	{
		while(1);
	}
	tc=0.05f/gps_kalman.deltat;
	acc_mean=(tc*acc_mean+gps_kalman.z[1])/(tc+1.0f);
	
	//if(fabs(alt_filter.x[1])<0.1f)
	//if(fabs(alt_now-alt_last)<0.003f*0.1f)
//	if(fabs(acc_mean)<0.2f&&fabs(alt_filter.z[1])<0.1f)
//	{
//		
//		alt_filter.x[1]=0;
//	}
//	else if(fabs(acc_mean)<0.2f)
	
	
	if(fabs(acc_mean)<0.3f)
	{
		tc=2.f/gps_kalman.deltat;
		gps_kalman.x[1]=tc*gps_kalman.x[1]/(tc+1.0f);
	}
	tc=5.f/gps_kalman.deltat;
	gps_kalman.x[2]=tc*gps_kalman.x[2]/(tc+1.0f);

}

void GPS_KALMAN_InputsTranslate(KALMAN_PARAMS_TypeDef *pkalman)
{
	pkalman->A=(float*)gps_kalman.A;
	pkalman->C=(float*)gps_kalman.C;
	pkalman->P=(float*)gps_kalman.P;
	pkalman->Q=(float*)gps_kalman.Q;
	pkalman->R=(float*)gps_kalman.R;
	pkalman->x=(float*)gps_kalman.x;
	pkalman->z=(float*)gps_kalman.z;
	pkalman->x_size=X_SIZE;
	pkalman->z_size=Z_SIZE;
}

void GPS_KALMAN_SetQR(void)
{
	gps_kalman.Q[0][0]=gps_kalman_qr.q_h;
	gps_kalman.Q[1][1]=gps_kalman_qr.q_v;
	gps_kalman.Q[2][2]=gps_kalman_qr.q_a;
	gps_kalman.Q[3][3]=gps_kalman_qr.q_a_bias;
	gps_kalman.R[0][0]=gps_kalman_qr.r_h;
	gps_kalman.R[1][1]=gps_kalman_qr.r_a;
}
void GPS_KALMAN_SetDefaultQR(void)
{
	gps_kalman_qr.q_h=ALT_FILTER_Q_H;
	gps_kalman_qr.q_v=ALT_FILTER_Q_V;
	gps_kalman_qr.q_a=ALT_FILTER_Q_A;
	gps_kalman_qr.q_a_bias=ALT_FILTER_Q_A_BIAS;
	gps_kalman_qr.r_h=ALT_FILTER_R_H;
	gps_kalman_qr.r_a=ALT_FILTER_R_A;
}








