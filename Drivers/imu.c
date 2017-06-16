
#include "imu.h"
#include "ellipsoid.h"
#include "matrix.h"
#include "time.h"
#include "beep.h"
#include "eeprom.h"
#include "math.h"
#include "madgwickahrs.h"
#include "main.h"
#define M_PI  		((float)3.1415926535)
#define M_2PI  		((float)6.283185307)
#define R2D    		((float)57.29577951472)
#define D2R    		((float)0.017453292519444)

AHRS_TypeDef ahrs=
{0,0,0,0,0,0,0,0,0,0,
	0,0,0,0,0,0,0,0,0,0,
	0,0,0,0,0,0};
GPS_TypeDef gps=
{0,0,0,0,0,0};;
IMU_CMD_TypeDef imu_cmd={0,0};


#if FLIGHT_BOARD==1
//NO.1 FlightBoard
IMU_CALIB_PARAMS_TypeDef imu_calib_params={
{-0.0156029f,-0.00590315f,-0.0664579f},//acc_bias[3]
{0,0,0,0,0,0,0,0,0},//acc_Ae[9]
{1.00325f,0,0,0,0.990761f,0,0,0,1.00026f},//acc_coef[9]
{63.1414f,51.0636f,32.6183f},//mag_bias[3]
{0,0,0,0,0,0,0,0,0},//mag_Ae[9]
{0.00795209f,0,0,
0,0.00803077f,0,
0,0,0.008508f},//mag_coef[9]
{-1.02596f,-0.713195f,2.48364f}//gyro_bias[3]
};
#elif FLIGHT_BOARD==2
//NO.2 FlightBoard-20151116
IMU_CALIB_PARAMS_TypeDef imu_calib_params={
{-0.0199297f,-0.00551172f,-0.0127118f},//acc_bias[3]
{0,0,0,0,0,0,0,0,0},//acc_Ae[9]
{1.00453f,0,0,0,0.996481f,0,0,0,1.0051f},//acc_coef[9]
{-34.204f,-41.3738f,116.592f},//mag_bias[3]
{0,0,0,0,0,0,0,0,0},//mag_Ae[9]
{0.00824478f,0,0,
0,0.00809547f,0,
0,0,0.00853927f},//mag_coef[9]
{0.30501f,-0.702089f,1.71959f}//gyro_bias[3]
};
#elif FLIGHT_BOARD==3
//NO.3 FlightBoard
IMU_CALIB_PARAMS_TypeDef imu_calib_params={
{-0.00674702f,-0.0173129f,-0.0248858f},//acc_bias[3]
{0,0,0,0,0,0,0,0,0},//acc_Ae[9]
{1.00476f,0,0,0,1.00286f,0,0,0,1.0021f},//acc_coef[9]
{-34.204f,-41.3738f,116.592f},//mag_bias[3]
{0,0,0,0,0,0,0,0,0},//mag_Ae[9]
{0.00824478f,0,0,
0,0.00809547f,0,
0,0,0.00853927f},//mag_coef[9]
{0.527687f,-1.44278f,0.558613f}//gyro_bias[3]
};
#endif

#define IMU_CMD_CALIB_GYRO								1
#define IMU_CMD_CALIB_ACCEL								2
#define IMU_CMD_CALIB_MAG									3

char calib_accel_status=0;//0-start,1-z_up,2-z_down,3-x_up,4-x_down,5-y_up,6-y_down,7-finish
char calib_mag_status=0;//0-start,1-1st axis rot,2-2nd axis rot,3-3rd axis rot
char calib_gyro_status=0;//0-start,1-calib,2-finish

char IMU_Calib(float ax,float ay,float az,float gx,float gy,float gz,float mx,float my,float mz);
char IMU_CalibAcc(float ax,float ay,float az,char reset);
char IMU_CalibGyro(float gx,float gy,float gz,char reset);
char IMU_CalibMag(float mx,float my,float mz,char reset);
float IMU_AngleLowPass(float a1,float a2,float k);  //|k|<<1,res=(1-k)*a1+k*a2
float IMU_LimitAngle(float a);

void IMU_Config(void)
{
	ahrs.acc_line_tc=0.05f;
	//ATT_KALMAN_Config();
}
float acc_norm,mag_norm;
float init_time=0;
float YAW_TC=1.0f;
void IMU_Update(void)
{
	float acc[3],gyro[3],mag[3];
	float a[3],g[3],m[3];
	float deltat;
	static float deltat_s=0;
	static float g_s[3],a_s[3]={0,0,1},a_s2[3]={0,0,1};
	float tc;
	static float yaw_r,pitch_r,roll_r;
	float cos_roll,sin_roll,cos_pitch,sin_pitch;
	float ax_line,ay_line,az_line;
	float mag_x,mag_y,yaw_r_measure;
	#define INIT_SIZE  20
	#define STATIC_THR   0.2f
	static int gyro_init=-1,i,init_index=0;
	static float gyro_data[3][INIT_SIZE],gyro_deltat=0;
	float gyro_mean[3];
	char gyro_is_static=1;
	#define GYRO_TC    (0.005f)
	#define ACC_TC     (0.005f)
	acc[0]=ahrs.ax_raw*ACC_SCOPE/32768.f;
	acc[1]=ahrs.ay_raw*ACC_SCOPE/32768.f;
	acc[2]=ahrs.az_raw*ACC_SCOPE/32768.f;

	gyro[0]=(ahrs.gx_raw)*GYRO_SCOPE/32768.f;
	gyro[1]=(ahrs.gy_raw)*GYRO_SCOPE/32768.f;
	gyro[2]=(ahrs.gz_raw)*GYRO_SCOPE/32768.f;
	
	mag[0]=ahrs.mx_raw;
	mag[1]=ahrs.my_raw;
	mag[2]=ahrs.mz_raw;
	
	deltat=(float)TIME_GetTimeUs(TIME_ATT)/1000000.0f;
	if(deltat<1e-8f)
	{
		deltat=1e-8f;
	}
	
	if(gyro_init<1)
	{
		if(gyro_init==-1)
		{
			for(i=0;i<INIT_SIZE;i++)
			{
				gyro_data[0][i]=i;
			}
			for(i=0;i<INIT_SIZE;i++)
			{
				gyro_data[1][i]=i;
			}
			for(i=0;i<INIT_SIZE;i++)
			{
				gyro_data[2][i]=i;
			}
			gyro_init=0;
		}
		else
		{
			gyro_deltat+=deltat;
			if(gyro_deltat>0.1f)
			{
				gyro_deltat=0;
				gyro_is_static=1;
				init_index++;
				if(init_index>=INIT_SIZE)
				{
					init_index=0;
				}
				gyro_data[0][init_index]=gyro[0];
				gyro_data[1][init_index]=gyro[1];
				gyro_data[2][init_index]=gyro[2];
				if(fabs(gyro[0])>5||fabs(gyro[1])>5||fabs(gyro[2])>5)
				{
					gyro_is_static=0;
				}
				if(gyro_is_static)
				{
					for(i=0;i<INIT_SIZE;i++)
					{
						gyro_mean[0]+=gyro_data[0][i];
					}
					gyro_mean[0]/=(float)INIT_SIZE;
					for(i=0;i<INIT_SIZE;i++)
					{
						if(gyro_is_static)
						{
							if(fabs(gyro_data[0][i]-gyro_mean[0])>STATIC_THR)
							{
								gyro_is_static=0;
							}
						}
						else
						{
							break;
						}
					}
				}
				if(gyro_is_static)
				{
					for(i=0;i<INIT_SIZE;i++)
					{
						gyro_mean[1]+=gyro_data[1][i];
					}
					gyro_mean[1]/=(float)INIT_SIZE;
					for(i=0;i<INIT_SIZE;i++)
					{
						if(gyro_is_static)
						{
							if(fabs(gyro_data[1][i]-gyro_mean[1])>STATIC_THR)
							{
								gyro_is_static=0;
							}
						}
						else
						{
							break;
						}
					}
				}
				if(gyro_is_static)
				{
					for(i=0;i<INIT_SIZE;i++)
					{
						gyro_mean[2]+=gyro_data[2][i];
					}
					gyro_mean[2]/=(float)INIT_SIZE;
					for(i=0;i<INIT_SIZE;i++)
					{
						if(gyro_is_static)
						{
							if(fabs(gyro_data[2][i]-gyro_mean[2])>STATIC_THR)
							{
								gyro_is_static=0;
							}
						}
						else
						{
							break;
						}
					}
				}
				if(gyro_is_static)
				{
					gyro_init=1;
					imu_calib_params.gyro_bias[0]=gyro_mean[0];
					imu_calib_params.gyro_bias[1]=gyro_mean[1];
					imu_calib_params.gyro_bias[2]=gyro_mean[2];
					BEEP_SetStatus(BEEP_STATUS_ESC);
				}
			}
			
			
		}
	}
	if(imu_cmd.cmd!=0||(calib_accel_status!=0)||(calib_gyro_status!=0)||(calib_mag_status!=0))
	{
		if(calib_gyro_status!=0)
		{
			gyro_init=1;
		}
		deltat_s+=deltat;
		if(deltat_s>0.00999f)
		{
			deltat_s=0;
			IMU_Calib(acc[0],acc[1],acc[2],gyro[0],gyro[1],gyro[2],mag[0],mag[1],mag[2]);
		}
	}
	
	acc[0]=acc[0]-imu_calib_params.acc_bias[0];
	acc[1]=acc[1]-imu_calib_params.acc_bias[1];
	acc[2]=acc[2]-imu_calib_params.acc_bias[2];
	
	g[0]=gyro[0]-imu_calib_params.gyro_bias[0];
	g[1]=gyro[1]-imu_calib_params.gyro_bias[1];
	g[2]=gyro[2]-imu_calib_params.gyro_bias[2];
	
	mag[0]=mag[0]-imu_calib_params.mag_bias[0];
	mag[1]=mag[1]-imu_calib_params.mag_bias[1];
	mag[2]=mag[2]-imu_calib_params.mag_bias[2];
	
	MATRIX_Mul(a,(float*)imu_calib_params.acc_coef,acc,3,3,1);
	MATRIX_Mul(m,(float*)imu_calib_params.mag_coef,mag,3,3,1);
	
	tc=GYRO_TC/deltat;
	g_s[0]=(tc*g_s[0]+g[0])/(tc+1.0f);
	g_s[1]=(tc*g_s[1]+g[1])/(tc+1.0f);
	g_s[2]=(tc*g_s[2]+g[2])/(tc+1.0f);
	tc=ACC_TC/deltat;
	a_s[0]=(tc*a_s[0]+a[0])/(tc+1.0f);
	a_s[1]=(tc*a_s[1]+a[1])/(tc+1.0f);
	a_s[2]=(tc*a_s[2]+a[2])/(tc+1.0f);

	if(init_time<1)
	{
		tc=0.3f/deltat;
		a_s2[0]=(tc*a_s2[0]+a_s[0])/(tc+1.0f);
		a_s2[1]=(tc*a_s2[1]+a_s[1])/(tc+1.0f);
		a_s2[2]=(tc*a_s2[2]+a_s[2])/(tc+1.0f);
		
		roll_r=atan2(a_s2[1],a_s2[2]); //=y/z
		pitch_r=atan2(-a_s2[0],sqrt(a_s2[1]*a_s2[1]+a_s2[2]*a_s2[2])); 
		cos_roll = cos(roll_r);
		sin_roll = sin(roll_r);
		cos_pitch = cos(pitch_r);
		sin_pitch = sin(pitch_r);
		mag_x=m[0]*cos_pitch+m[1]*sin_roll*sin_pitch+m[2]*cos_roll*sin_pitch;
		mag_y=m[1]*cos_roll-m[2]*sin_roll;
		yaw_r_measure=atan2(-mag_y,mag_x);
		yaw_r=IMU_AngleLowPass(yaw_r+g_s[2]*D2R*deltat,yaw_r_measure,(deltat)/(0.2f+deltat));
		AHRS_SetQFromAngle(yaw_r,pitch_r,roll_r);
		ahrs.roll=AHRS_R2D(roll_r);
		ahrs.pitch=AHRS_R2D(pitch_r);
		ahrs.yaw=yaw_r*R2D;
		
		ahrs.ax_line=0;
		ahrs.ay_line=0;
		ahrs.az_line=0;
		
		init_time+=deltat;
	}
	else
	{
//		AHRS_Update6(g_s[0],g_s[1],g_s[2],a_s[0],a_s[1],a_s[2],0.028f,deltat);
		
		AHRS_Update6(g_s[0],g_s[1],g_s[2],a_s[0],a_s[1],a_s[2],0.2f,deltat);
		
//		AHRS_Update6(g_s[0],g_s[1],g_s[2],a_s[0],a_s[1],a_s[2],0.02f,deltat);
//		AHRS_Update6(g_s[0],g_s[1],g_s[2],a_s[0],a_s[1],a_s[2],0.008f,deltat);
		AHRS_GetAngle(&yaw_r_measure,&pitch_r,&roll_r);
		ahrs.roll=AHRS_R2D(roll_r);
		ahrs.pitch=AHRS_R2D(pitch_r);
		cos_roll = cos(roll_r);
		sin_roll = sin(roll_r);
		cos_pitch = cos(pitch_r);
		sin_pitch = sin(pitch_r);
		
		mag_x=m[0]*cos_pitch+m[1]*sin_roll*sin_pitch+m[2]*cos_roll*sin_pitch;
		mag_y=m[1]*cos_roll-m[2]*sin_roll;
		yaw_r_measure=atan2(-mag_y,mag_x);
		//yaw_r=IMU_AngleLowPass(yaw_r+g_s[2]*D2R*deltat,yaw_r_measure,(deltat)/(YAW_TC+deltat));
		
//		yaw_r=AHRS_UpdateYaw(yaw_r_measure,0.02f,deltat);
		yaw_r=AHRS_UpdateYaw(yaw_r_measure,0.5f,deltat);
		ahrs.yaw=yaw_r*R2D;
		
		ax_line=(cos_pitch*a_s[0]+(sin_roll*sin_pitch)*a_s[1]+(cos_roll*sin_pitch)*a_s[2])*GRAVITY;
		ay_line=((cos_roll)*a_s[1]+(-sin_roll)*a_s[2])*GRAVITY;
		az_line=(-sin_pitch*a_s[0]+(sin_roll*cos_pitch)*a_s[1]+(cos_roll*cos_pitch)*a_s[2]-1.f)*GRAVITY;///0.9775
		tc=ahrs.acc_line_tc/deltat;
		ahrs.ax_line=(tc*ahrs.ax_line+ax_line)/(tc+1.0f);
		ahrs.ay_line=(tc*ahrs.ay_line+ay_line)/(tc+1.0f);
		ahrs.az_line=(tc*ahrs.az_line+az_line)/(tc+1.0f);
	}
	
	ahrs.gx=g_s[0];
	ahrs.gy=g_s[1];
	ahrs.gz=g_s[2];
	
	
	
}

char IMU_Calib(float ax,float ay,float az,float gx,float gy,float gz,float mx,float my,float mz)
{
	
	if((calib_accel_status==0)&&(calib_gyro_status==0)&&(calib_mag_status==0))
	{
		switch(imu_cmd.cmd)
		{
			case IMU_CMD_CALIB_GYRO:
				calib_gyro_status=1;
				IMU_CalibGyro(gx,gy,gz,1);
			break;
			case IMU_CMD_CALIB_ACCEL:
				ELLIPSOID_Reset(1);
				calib_accel_status=1;
				IMU_CalibAcc(ax,ay,az,1);
			break;
			case IMU_CMD_CALIB_MAG:
//				ELLIPSOID_Reset(0);
				ELLIPSOID_Reset(1);
				calib_mag_status=1;
				IMU_CalibMag(mx,my,mz,1);
			break;
		}
		imu_cmd.cmd=0;
		return 1;
	}
	if((calib_accel_status!=0))
	{
		IMU_CalibAcc(ax,ay,az,0);
	}
	else if((calib_mag_status!=0))
	{
		IMU_CalibMag(mx,my,mz,0);
	}
	else
	{
		IMU_CalibGyro(gx,gy,gz,0);
	}
	
	return 1;
}
char IMU_CalibGyro(float gx,float gy,float gz,char reset)
{
	static int cnt;
	static float gyro[3];
	if(reset)
	{
		cnt=0;
		gyro[0]=gx;
		gyro[1]=gy;
		gyro[2]=gz;
		return 0;
	}
	if(cnt<500)
	{
		gyro[0]=(100.f*gyro[0]+gx)/(100.f+1);
		gyro[1]=(100.f*gyro[1]+gy)/(100.f+1);
		gyro[2]=(100.f*gyro[2]+gz)/(100.f+1);
		//BEEP_SetStatus(BEEP_STATUS_ACK);
		LED_SetStatus(LED_ALL,LED_STATUS_CALIB);
		cnt++;
	}
	else
	{
		imu_calib_params.gyro_bias[0]=gyro[0];
		imu_calib_params.gyro_bias[1]=gyro[1];
		imu_calib_params.gyro_bias[2]=gyro[2];
		calib_gyro_status=0;
		SetConfig();
		LED_ResetStatus(LED_ALL);
		BEEP_SetStatus(BEEP_STATUS_START);
	}
	return 1;
}

char IMU_CalibAcc(float ax,float ay,float az,char reset)
{
	static int cnt,point_cnt;
	static float ax_s,ay_s,az_s;
	if(reset)
	{
		cnt=0;
		point_cnt=0;
		ax_s=ax;
		ay_s=ay;
		az_s=az;
		return 0;
	}
	if(point_cnt>=300)
	{
		if(!ELLIPSOID_Fitting((float*)imu_calib_params.acc_Ae,(float*)imu_calib_params.acc_bias,(float*)imu_calib_params.acc_coef))
		{
			BEEP_SetStatus(BEEP_STATUS_STOP);
			LED_ResetStatus(LED_ALL);
			return 0;
		}
		LED_ResetStatus(LED_ALL);
		calib_accel_status=0;
		SetConfig();
		BEEP_SetStatus(BEEP_STATUS_START);
	}
	else if(cnt>=4)
	{
		cnt=0;
		if(point_cnt!=50&&point_cnt!=100&&point_cnt!=150&&point_cnt!=200&&point_cnt!=250)
		{
			ax_s=(100*ax_s+ax)/101.f;
			ay_s=(100*ay_s+ay)/101.f;
			az_s=(100*az_s+az)/101.f;
			ELLIPSOID_AddPoint(ax_s,ay_s,az_s,point_cnt);
			point_cnt++;
			//BEEP_SetStatus(BEEP_STATUS_ACK);
			LED_SetStatus(LED_ALL,LED_STATUS_CALIB);
		}
		else
		{
			if(imu_cmd.cmd==IMU_CMD_CALIB_ACCEL)
			{
				imu_cmd.cmd=0;
				ELLIPSOID_AddPoint(ax,ay,az,point_cnt);
				point_cnt++;
				ax_s=ax;
				ay_s=ay;
				az_s=az;
			}
			LED_ResetStatus(LED_ALL);
		}
		
	}
	else
	{
		cnt++;
	}
	return 1;
}

char IMU_CalibMag(float mx,float my,float mz,char reset)
{
	static int cnt,point_cnt;
	if(reset)
	{
		cnt=0;
		point_cnt=0;
		return 0;
	}
	if(point_cnt>=300)
	{
		if(!ELLIPSOID_Fitting((float*)imu_calib_params.mag_Ae,(float*)imu_calib_params.mag_bias,(float*)imu_calib_params.mag_coef))
		{
			BEEP_SetStatus(BEEP_STATUS_STOP);
			LED_ResetStatus(LED_ALL);
			return 0;
		}
		LED_ResetStatus(LED_ALL);
		calib_mag_status=0;
		SetConfig();
		BEEP_SetStatus(BEEP_STATUS_START);
	}
	else if(cnt>=20)
	{
		cnt=0;
		if(point_cnt!=100&&point_cnt!=200)
		{
			ELLIPSOID_AddPoint(mx,my,mz,point_cnt);
			point_cnt++;
			BEEP_SetStatus(BEEP_STATUS_ACK);
			LED_SetStatus(LED_ALL,LED_STATUS_CALIB);
		}
		else
		{
			if(imu_cmd.cmd==IMU_CMD_CALIB_MAG)
			{
				imu_cmd.cmd=0;
				ELLIPSOID_AddPoint(mx,my,mz,point_cnt);
				point_cnt++;
			}
			LED_ResetStatus(LED_ALL);
		}
		
	}
	else
	{
		cnt++;
	}
	
	return 1;
}



float IMU_LimitAngle(float a)
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
float IMU_AngleLowPass(float a1,float a2,float k)  //|k|<<1,res=(1-k)*a1+k*a2
{
	float tmp=0;
	tmp=a1-a2;
	tmp=tmp>0?tmp:-tmp;
	if(tmp<M_PI)
	{
			tmp=(1-k)*a1+k*a2;
	}
	else
	{
		if(a2>0)
		{
			a2=a2-M_PI*2;
		}
		else
		{
			a2=a2+M_PI*2;
		}
		tmp=(1-k)*(a1)+k*a2;
	}
	
	return IMU_LimitAngle(tmp);
}




