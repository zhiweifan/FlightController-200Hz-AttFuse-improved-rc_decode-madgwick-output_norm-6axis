
#include "alt_filter.h"
#include "kalman_lib.h"
#include "main.h"
#include "math.h"
//no speed reduction
//#define ALT_FILTER_Q_H										0.1f//(1.5f)
//#define ALT_FILTER_Q_V										0.01f//(0.01f)
//#define ALT_FILTER_Q_A										1.1f//(0.1f)
//#define ALT_FILTER_Q_A_BIAS								0.00001f//(0.01f)
//#define ALT_FILTER_R_H										50000000.0f//(0.3f)
//#define ALT_FILTER_R_A										0.00001f//(1.1f)


//with speed reduction
//#define ALT_FILTER_Q_H										0.001f//(1.5f)
//#define ALT_FILTER_Q_V										0.01f//(0.01f)
//#define ALT_FILTER_Q_A										0.1f//(0.1f)
//#define ALT_FILTER_Q_A_BIAS								0.001f//(0.01f)
//#define ALT_FILTER_R_H										50000000.0f//(0.3f)
//#define ALT_FILTER_R_A										0.00001f//(1.1f)

#define ALT_FILTER_Q_H										0.001f//(1.5f)
#define ALT_FILTER_Q_V										0.01f//(0.01f)
#define ALT_FILTER_Q_A										0.1f//(0.1f)
#define ALT_FILTER_Q_A_BIAS								0.002f//(0.01f)
#define ALT_FILTER_R_H										50000000000.0f//(0.3f)
#define ALT_FILTER_R_A										0.00001f//(1.1f)

ALT_FILTER_TypeDef alt_filter;
ALT_FILTER_QR_TypeDef alt_filter_qr;

void InputsTranslate(KALMAN_PARAMS_TypeDef *pkalman);

void ALT_FILTER_Config(void)
{
	
  ALT_FILTER_SetQR();
	
	
//	alt_filter.P[0][0]=1.f;
//	alt_filter.P[1][1]=1.f;
//	alt_filter.P[2][2]=1.f;
//	alt_filter.P[3][3]=1.f;
	
//	alt_filter.P[0][0]=100000000.f;
//	alt_filter.P[1][1]=10000000.f;
//	alt_filter.P[2][2]=1.f;
//	alt_filter.P[3][3]=1.f;
	
	
	alt_filter.P[0][0]=6530.6543f;
	alt_filter.P[0][1]=889.28363f;
	alt_filter.P[0][2]=22.3026142f;
	alt_filter.P[0][3]=-22.3026142f;
	alt_filter.P[1][0]=889.28363f;
	alt_filter.P[1][1]=252.091156f;
	alt_filter.P[1][2]=6.88223553f;
	alt_filter.P[1][3]=-6.88223553f;
	alt_filter.P[2][0]=22.3026142f;
	alt_filter.P[2][1]=6.88223553f;
	alt_filter.P[2][2]=0.937909245f;
	alt_filter.P[2][3]=-0.937899232f;
	alt_filter.P[3][0]=-22.3026142f;
	alt_filter.P[3][1]=-6.88223553f;
	alt_filter.P[3][2]=-0.937899232f;
	alt_filter.P[3][3]=0.937899172f;
	
//	alt_filter.P[0][0]=6611.70752f;
//	alt_filter.P[0][1]=892.027466f;
//	alt_filter.P[0][2]=22.3754044f;
//	alt_filter.P[0][3]=-22.3754044f;
//	alt_filter.P[1][0]=892.027466f;
//	alt_filter.P[1][1]=249.424164f;
//	alt_filter.P[1][2]=6.81329536f;
//	alt_filter.P[1][3]=-6.81329584f;
//	alt_filter.P[2][0]=22.3754044f;
//	alt_filter.P[2][1]=6.81329536f;
//	alt_filter.P[2][2]=0.918785572f;
//	alt_filter.P[2][3]=-0.918775558f;
//	alt_filter.P[3][0]=-22.3754044f;
//	alt_filter.P[3][1]=-6.81329584f;
//	alt_filter.P[3][2]=-0.918775558f;
//	alt_filter.P[3][3]=0.918775499f;

	alt_filter.C[0][0]=1.f;
	alt_filter.C[1][2]=1.f;
	alt_filter.C[1][3]=1.f;
	
	alt_filter.x[0]=0;
	alt_filter.x[1]=0;
	alt_filter.x[2]=0;
	alt_filter.x[3]=CtrlSys.acc_line_z_bias;
	alt_filter.acc_line_z_tc=0.0f;
}

void ALT_Update(void)
{
	float t1,t2;
	float acc_line_z_tc;
	float tc;
	static float acc_mean;
	static float acc_z_mean=0,acc_z_ok_time=0;
	static int acc_z_is_init=-1;
//	static float q_time=0;
	KALMAN_PARAMS_TypeDef kalman;
	
	alt_filter.deltat=(float)TIME_GetTimeUs(TIME_ALT)/1000000.0f;
	if(alt_filter.deltat<1e-8f)
	{
		alt_filter.deltat=1e-8f;
	}
	t1=alt_filter.deltat;
	t2=t1*alt_filter.deltat;

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
		tc=4.f/alt_filter.deltat;
		acc_z_mean=(tc*acc_z_mean+ahrs.az_line)/(tc+1.0f);
		if(fabs(acc_z_mean-ahrs.az_line)<0.03f&&fabs(acc_z_mean)<0.6f)
		{
			acc_z_ok_time+=alt_filter.deltat;
		}
		else
		{
			acc_z_ok_time=0;
		}
		if(acc_z_ok_time>20.f)
		{
			alt_filter.x[3]=acc_z_mean;
			CtrlSys.acc_line_z_bias=acc_z_mean;
			acc_z_is_init=1;
			CtrlSys.acc_z_is_init=1;
		}
		else
		{
			tc=2.0f/alt_filter.deltat;
			alt_filter.x[0]=(alt_filter.x[0]*tc+ahrs.altitude)/(tc+1.f);
		}
		return ;
	}
//	if(fabs(ahrs.az_line-CtrlSys.acc_line_z_bias)<1.f&&fabs()<0.5f)
//	if((fabs(ahrs.altitude-alt_filter.x[0])>0.4f||fabs(ahrs.az_line-CtrlSys.acc_line_z_bias)<0.5f)&&q_time<=0)
////	if(fabs(ahrs.altitude-alt_filter.x[0])>0.4f)
//	{
//		alt_filter.Q[3][3]=alt_filter_qr.q_a_bias;
//	}
//	else
//	{
//		q_time+=alt_filter.deltat;
//		if(q_time>10)
//		{
//			q_time=0;
//		}
//		alt_filter.Q[3][3]=alt_filter_qr.q_a_bias/10.f;
//		
//	}
	//x:3,z:2
	alt_filter.A[0][0]=1.f;
	alt_filter.A[0][1]=t1;
	alt_filter.A[0][2]=t2/2.f;
//	alt_filter.A[0][3]=t3/6.f;
	alt_filter.A[1][1]=1.f;
	alt_filter.A[1][2]=t1;
//	alt_filter.A[1][3]=t2/2.f;
	alt_filter.A[2][2]=1.f;
//	alt_filter.A[2][3]=t1;
	alt_filter.A[3][3]=1.f;
	
	alt_filter.z[0]=ahrs.altitude;
	acc_line_z_tc=alt_filter.acc_line_z_tc/alt_filter.deltat;
	alt_filter.z[1]=(acc_line_z_tc*alt_filter.z[1]+(ahrs.az_line))/(acc_line_z_tc+1.00f);
	InputsTranslate(&kalman);
	if(!KALMAN_Update(&kalman))
	{
		while(1);
	}
	tc=0.05f/alt_filter.deltat;
	acc_mean=(tc*acc_mean+alt_filter.z[1])/(tc+1.0f);
	
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
		tc=2.f/alt_filter.deltat;
		alt_filter.x[1]=tc*alt_filter.x[1]/(tc+1.0f);
	}
	tc=5.f/alt_filter.deltat;
	alt_filter.x[2]=tc*alt_filter.x[2]/(tc+1.0f);
	
//	if(TIME_CheckTimeUs(TIME_ALT_SPEED,100000))
//	{
//		TIME_Reset(TIME_ALT_SPEED);
//		alt_last=alt_now;
//		alt_now=alt_mean;
//	}
}

//void ALT_Update(void)
//{
//	float t1,t2;
//	float acc_line_z_tc;
//	static int cnt=0;
//	static float last_h;
//	float tc1,tc2;
//	alt_filter.deltat=(float)TIME_GetTimeUs(TIME_ALT)/1000000.0f;
//	t1=alt_filter.deltat;
//	t2=t1*alt_filter.deltat;

//	alt_filter.z[0]=ahrs.altitude;
//	acc_line_z_tc=alt_filter.acc_line_z_tc/alt_filter.deltat;
//	alt_filter.z[1]=(acc_line_z_tc*alt_filter.z[1]+ahrs.az_line)/(acc_line_z_tc+1.00f);

//	alt_filter.x[2]=alt_filter.z[1];
//	tc1=alt_filter_qr.q_h/alt_filter.deltat;
//	tc2=alt_filter_qr.q_v/alt_filter.deltat;
//	
//	alt_filter.x[0]=(tc1*(alt_filter.x[0]+alt_filter.x[2]*t2/2.f)+alt_filter.z[0])/(tc1+1.f);
//	alt_filter.x[1]=(tc2*(alt_filter.x[1]+alt_filter.x[2]*t1)+(alt_filter.x[0]-last_h)/t1)/(tc2+1.f);
//	last_h=alt_filter.x[0];
//}
void InputsTranslate(KALMAN_PARAMS_TypeDef *pkalman)
{
	pkalman->A=(float*)alt_filter.A;
	pkalman->C=(float*)alt_filter.C;
	pkalman->P=(float*)alt_filter.P;
	pkalman->Q=(float*)alt_filter.Q;
	pkalman->R=(float*)alt_filter.R;
	pkalman->x=(float*)alt_filter.x;
	pkalman->z=(float*)alt_filter.z;
	pkalman->x_size=X_SIZE;
	pkalman->z_size=Z_SIZE;
}

void ALT_FILTER_SetQR(void)
{
	alt_filter.Q[0][0]=alt_filter_qr.q_h;
	alt_filter.Q[1][1]=alt_filter_qr.q_v;
	alt_filter.Q[2][2]=alt_filter_qr.q_a;
	alt_filter.Q[3][3]=alt_filter_qr.q_a_bias;
	alt_filter.R[0][0]=alt_filter_qr.r_h;
	alt_filter.R[1][1]=alt_filter_qr.r_a;
}
void ALT_FILTER_SetDefaultQR(void)
{
	alt_filter_qr.q_h=ALT_FILTER_Q_H;
	alt_filter_qr.q_v=ALT_FILTER_Q_V;
	alt_filter_qr.q_a=ALT_FILTER_Q_A;
	alt_filter_qr.q_a_bias=ALT_FILTER_Q_A_BIAS;
	alt_filter_qr.r_h=ALT_FILTER_R_H;
	alt_filter_qr.r_a=ALT_FILTER_R_A;
}




