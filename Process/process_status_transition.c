
#include "process_status_transition.h"
#include "main.h"

void StartStop(void);
void SensorOk(void);
void IsTakeoff(void);
void StatusTransition(void)
{
	SensorOk();
	StartStop();
	IsTakeoff();
}
void StartStop(void)
{
	#define STOP_CNT    300
	static unsigned int start_cnt=0,stop_last=1;
	static char fly_flag=0;
	if(!CtrlSys.rc_ok)
	{
		return ;
	}
	if(stop_last!=CtrlSys.stop)
	{
		stop_last=CtrlSys.stop;
		if(CtrlSys.stop==1)
		{
			start_cnt=0;
			TIME_Reset(TIME_START_STOP);
		}
	}
	if(CtrlSys.stop)
	{
		if(start_cnt==200)
		{
			start_cnt++;
			BEEP_SetStatus(BEEP_STATUS_START);
			TIME_Reset(TIME_START_STOP);
		}
		else if(start_cnt>200)
		{
//			if(TIME_CheckTimeUs(TIME_START_STOP,2000000))
//			{
//				CtrlSys.stop=0;
//				start_cnt=0;
//				fly_flag=0;
//				TIME_Reset(TIME_START_STOP);
//				YawPID.Inputs.Ref=YawPID.Inputs.Measure;
//			}
			if(CtrlSys.stop&&TIME_CheckTimeUs(TIME_START_STOP,1000000))
			{
				CtrlSys.stop=0;
				//start_cnt=0;
				fly_flag=0;
				TIME_Reset(TIME_START_STOP);
				YawPID.Inputs.Ref=YawPID.Inputs.Measure;
				YawRatePID.Inputs.Ref=0;
			}
			else if(CtrlSys.stop==0)
			{
				if(TIME_CheckTimeUs(TIME_START_STOP,2000000))
				{
					start_cnt=0;
					TIME_Reset(TIME_START_STOP);
				}
				else
				{
					YawPID.Inputs.Ref=YawPID.Inputs.Measure;
					YawRatePID.Inputs.Ref=0;
				}
			}
			
		}
		else
		{
			if(CtrlSys.height_close_loop_mode)
			{
				if(CtrlSys.sensor_ok==1&&CtrlSys.rc_ok&&CtrlSys.ref_roll<-ANGLE_MAX*0.9f&&CtrlSys.ref_pitch<-ANGLE_MAX*0.9f&&
				((CtrlSys.ref_gimbal_yaw_rate<-GIMBAL_ROT_MAX_YAW*0.9f&&CtrlSys.ref_gimbal_pitch_rate<-GIMBAL_ROT_MAX*0.9f)))
				{
					start_cnt++;
				}
				else
				{
					start_cnt=0;
				}
			}
			else
			{
				if(CtrlSys.sensor_ok==1&&CtrlSys.rc_ok&&CtrlSys.ref_roll<-ANGLE_MAX*0.9f&&CtrlSys.ref_pitch<-ANGLE_MAX*0.9f&&
				((CtrlSys.ref_yaw_rate<-ROT_MAX_YAW*0.9f&&CtrlSys.throttle_raw<10)))
				{
					start_cnt++;
				}
				else
				{
					start_cnt=0;
				}
			}
			
		}
	}
	else
	{
		if((CtrlSys.test_mode==0||CtrlSys.test_mode==4)&&((ahrs.pitch)>50||ABS(ahrs.roll)>50))
		{
			CtrlSys.stop=1;
			start_cnt=0;
			fly_flag=0;
			return ;
		}
		if(fly_flag==0&&TIME_CheckTimeUs(TIME_START_STOP,1000000))
		{
			fly_flag=1;
		}
		if(start_cnt==STOP_CNT)
		{
			start_cnt++;
			if(CtrlSys.height_close_loop_mode==0)
			{
				BEEP_SetStatus(BEEP_STATUS_STOP);
			}
		}
		else if(start_cnt>STOP_CNT)
		{
			if(CtrlSys.height_close_loop_mode!=0)
			{
				if(!CtrlSys.is_takeoff)
				{
					if(start_cnt<10000)
					{
						start_cnt++;
					}
					if(start_cnt>(STOP_CNT+50))
					{
						BEEP_SetStatus(BEEP_STATUS_STOP);
						start_cnt=0;
						fly_flag=0;
						CtrlSys.stop=1;
						TIME_Reset(TIME_START_STOP);
					}
				}
				else
				{
					if(start_cnt>0)
					{
						start_cnt--;
					}
				}
			}
			else
			{
				CtrlSys.stop=1;
				start_cnt=0;
				fly_flag=0;
				TIME_Reset(TIME_START_STOP);
			}
			
		}
		else
		{
			if(CtrlSys.height_close_loop_mode)
			{
//				if(fly_flag&&(CtrlSys.throttle_raw<10||CtrlSys.ref_gimbal_pitch_rate<-GIMBAL_ROT_MAX*0.9f))
				if(fly_flag&&(CtrlSys.ref_gimbal_pitch_rate<-GIMBAL_ROT_MAX*0.9f))
				{
					start_cnt++;
				}
				else
				{
					start_cnt=0;
				}
			}
			else
			{
				if(fly_flag&&(CtrlSys.throttle_raw<10))
				{
					start_cnt++;
				}
				else
				{
					start_cnt=0;
				}
			}
			
		}
	}
}
void SensorOk(void)
{
	static float gyro[3]={3.f,3.f,3.f};
	static int gyro_ok_cnt=0;
	if(CtrlSys.sensor_ok==0)
	{
		if(CtrlSys.sensor_alt_ok==0)
		{
			if((ahrs.altitude!=0))
			{
				CtrlSys.sensor_alt_ok=1;
				
			}
		}
		if(CtrlSys.sensor_alt_ok==1&&CtrlSys.sensor_gyro_ok==0)
		{
			gyro[0]=gyro[0]*0.95f+ahrs.gx*0.05f;
			gyro[1]=gyro[1]*0.95f+ahrs.gy*0.05f;
			gyro[2]=gyro[2]*0.95f+ahrs.gz*0.05f;
//			if(ABS(gyro[0])<0.1f&&ABS(gyro[1])<0.1f&&ABS(gyro[2])<0.05f)
			if(ABS(gyro[0])<0.1f&&ABS(gyro[1])<0.1f&&ABS(gyro[2])<0.1f)
			{
				gyro_ok_cnt++;
				if(gyro_ok_cnt>100)
				{
					CtrlSys.sensor_gyro_ok=1;
				}
			}
			else
			{
				gyro_ok_cnt=0;
			}
		}
		if(CtrlSys.sensor_alt_ok&&CtrlSys.sensor_gyro_ok)
		{
//			if(CtrlSys.height_close_loop_mode==0)
//			{
//				CtrlSys.sensor_ok=1;
//				BEEP_SetStatus(BEEP_STATUS_STOP);
//			}
//			else
			{
				if(CtrlSys.acc_z_is_init==1)
				{
					CtrlSys.sensor_ok=1;
					BEEP_SetStatus(BEEP_STATUS_STOP);
				}
			}
		}
	}
}

void IsTakeoff(void)
{
	static int takeoff_cnt=0,no_takeoff_cnt=0;
	if(CtrlSys.throttle_mean>TAKEOFF_THR)
	{
		if(takeoff_cnt<100)
		{
			takeoff_cnt++;
		}
	}
	else
	{
		if(takeoff_cnt>0)
		{
			takeoff_cnt--;
		}
	}
	if(takeoff_cnt>70)
	{
		if(CtrlSys.is_takeoff==0)
		{
			BEEP_SetStatus(BEEP_STATUS_ACK);
		}
		CtrlSys.is_takeoff=1;
	}
	if(CtrlSys.throttle_mean<NO_TAKEOFF_THR)
	{
		if(no_takeoff_cnt<200)
		{
			no_takeoff_cnt++;
		}
	}
	else
	{
		if(no_takeoff_cnt>0)
		{
			no_takeoff_cnt--;
		}
	}
	if(no_takeoff_cnt>170)
	{
		if(CtrlSys.is_takeoff==1)
		{
			BEEP_SetStatus(BEEP_STATUS_ACK);
		}
		CtrlSys.is_takeoff=0;
	}
	if(CtrlSys.stop)
	{
		CtrlSys.is_takeoff=0;
		no_takeoff_cnt=0;
		takeoff_cnt=0;
	}
	
}

