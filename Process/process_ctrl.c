

#include "process_ctrl.h"
#include "main.h"
#include "math.h"
float ref_pitch,ref_roll,ref_yaw_rate;
float ref_pitch_rate,ref_roll_rate;
float ref_height_rate,ref_accel;

void DataProcess(void);
void IntegralReset(void);
void HeightCtrl(void);
void YawCtrl(void);
void PitchCtrl(void);
void RollCtrl(void);
//CtrlSys.test_mode:
//0-normal,1-motor test
//2-esc_setting,3-esc_no_setting
//4-double loop,5-single loop
void CtrlProcess(void)
{
	DataProcess();
	IntegralReset();
	HeightCtrl();
	YawCtrl();
	PitchCtrl();
	RollCtrl();
}

void DataProcess(void)
{
	ref_pitch=AddDead(CtrlSys.ref_pitch,-ANGLE_MAX,ANGLE_MAX,-ANGLE_MAX,ANGLE_MAX,0.5f,1);
	ref_pitch_rate=AddDead(CtrlSys.ref_pitch_rate,-ROT_MAX,ROT_MAX,-ROT_MAX,ROT_MAX,2.f,1);
	ref_roll=AddDead(CtrlSys.ref_roll,-ANGLE_MAX,ANGLE_MAX,-ANGLE_MAX,ANGLE_MAX,0.5f,1);
	ref_roll_rate=AddDead(CtrlSys.ref_roll_rate,-ROT_MAX,ROT_MAX,-ROT_MAX,ROT_MAX,2.f,1);
	ref_yaw_rate=AddDead(CtrlSys.ref_yaw_rate,-ROT_MAX_YAW,ROT_MAX_YAW,-ROT_MAX_YAW,ROT_MAX_YAW,10.0f,1);
	ref_height_rate=AddDead(CtrlSys.ref_height_rate,-HEIGHT_RATE_MAX,HEIGHT_RATE_MAX,-HEIGHT_RATE_MAX,HEIGHT_RATE_MAX,HEIGHT_RATE_MAX/50.f,1);
	ref_accel=AddDead(CtrlSys.ref_accel,-ACCEL_MAX,ACCEL_MAX,-ACCEL_MAX,ACCEL_MAX,ACCEL_MAX/50.f,1);
	
	if(CtrlSys.rc_ok==0)
	{
		ref_height_rate=0;
	}
	
	if(CtrlSys.stop)
	{
	//	if(CtrlSys.height_close_loop_mode==3)
		{
			CtrlSys.height_bias=alt_filter.x[0];
		}

		CtrlSys.throttle_mean=0;
	}
	else
	{
//		if(CtrlSys.height_close_loop_mode)
//		{
//			if(!CtrlSys.first_rise&&CtrlSys.throttle_mean<800&&HeightPID.Inputs.Ref<0.5f)
//			{
//				if(CtrlSys.height_close_loop_mode==3)
//				{
//					CtrlSys.height_bias=alt_filter.x[0];
//				}
//			}
//		}
	}
}
void IntegralReset(void)
{
	static float az_line_mean=0,height_speed=0;
	static int reset_cnt=0,stop_cnt=0;
//	if((!CtrlSys.is_takeoff)||(!CtrlSys.throttle_enough)||CtrlSys.stop)
	if((!CtrlSys.is_takeoff)||CtrlSys.stop)
//	if(CtrlSys.stop)
	{
		YawRatePID.Outputs.I=0;
		PitchPID.Outputs.I=0;	
		RollPID.Outputs.I=0;
		PitchRatePID.Outputs.I=0;
		RollRatePID.Outputs.I=0;
		
		
	}
	if(!CtrlSys.stop&&CtrlSys.height_close_loop_mode)
	{
		az_line_mean=(100.f*az_line_mean+(ahrs.az_line-CtrlSys.acc_line_z_bias))/(100.0f+1.f);
		height_speed=(100.f*height_speed+(alt_filter.x[1]))/(100.0f+1.f);
		if(fabs(az_line_mean)<0.3f&&fabs(height_speed)<0.1f)
		{
			if(CtrlSys.height_close_loop_mode==3)
			{
				if(HeightPID.Status.Err[0]<-0.2f&&HeightRatePID.Status.Err[0]<-0.1f&&AccPID.Status.Err[0]<-1.f)
				{
					if(reset_cnt<100)
					{
						reset_cnt++;
					}
				}
				else
				{
					if(reset_cnt>0)
					{
						reset_cnt--;
					}
				}
				if(reset_cnt>80)
				{
					YawRatePID.Outputs.I=0;
					PitchPID.Outputs.I=0;	
					RollPID.Outputs.I=0;
					PitchRatePID.Outputs.I=0;
					RollRatePID.Outputs.I=0;
					BEEP_SetStatus(BEEP_STATUS_ACK);
				}
				if(HeightPID.Status.Err[0]<-1.f&&HeightRatePID.Status.Err[0]<-0.5f&&AccPID.Status.Err[0]<-4.f)
				{
					if(stop_cnt<100)
					{
						stop_cnt++;
					}
				}
				else
				{
					if(stop_cnt>0)
					{
						stop_cnt--;
					}
				}
				if(CtrlSys.stop==0&&stop_cnt>70)
				{
					stop_cnt=0;
					CtrlSys.stop=1;
					BEEP_SetStatus(BEEP_STATUS_STOP);
				}
			}
			else if(CtrlSys.height_close_loop_mode==1)
			{
				
			}
		}
		else
		{
//			reset_cnt=0;
//			stop_cnt=0;
			if(reset_cnt>0)
			{
				reset_cnt--;
			}
			if(stop_cnt>0)
			{
				stop_cnt--;
			}
		}
	}
	else
	{
		if(CtrlSys.stop)
		{
			stop_cnt=0;
			reset_cnt=0;
		}
	}
//	else
//	{
//		if(CtrlSys.height_close_loop_mode)
//		{

//			if(!CtrlSys.first_rise)
//			{
//				YawRatePID.Outputs.I=0;
//				PitchPID.Outputs.I=0;	
//				RollPID.Outputs.I=0;
//				PitchRatePID.Outputs.I=0;
//				RollRatePID.Outputs.I=0;
//				
//				
//			}
//		}
//	}
}

//CtrlSys.height_close_loop_mode:0-open_loop,1-accel_close_loop,2-height_rate_close_loop,3-height_close_loop
void HeightCtrl(void)
{
	float height,speed,acc;
	float throttle;
//	static float accel_output_bias;
	float acc_i_output_min=0;

	height=alt_filter.x[0]-CtrlSys.height_bias;
	speed=alt_filter.x[1];
	acc=ahrs.az_line-CtrlSys.acc_line_z_bias;
	if(CtrlSys.stop)
	{
		//accel_output_bias=0;
	}
	else
	{
		//accel_output_bias=(100.f*accel_output_bias+CtrlSys.accel_output_bias)/(100.f+1.f);
		acc_i_output_min=CtrlSys.accel_output_bias;
		if(CtrlSys.height_close_loop_mode==3&&AccPID.Outputs.I<acc_i_output_min)
		{
			if(ABS(AccPID.Status.MeasureFilter)<0.2f)
			{
				if((HeightPID.Status.Err[0])>-0.1f)
				{
					AccPID.Outputs.I=(20.f*AccPID.Outputs.I+acc_i_output_min)/(20.f+1.f);
				}
				else if((HeightPID.Status.Err[0])>-0.2f&&AccPID.Outputs.I<acc_i_output_min*0.8f)
				{
					AccPID.Outputs.I=(20.f*AccPID.Outputs.I+acc_i_output_min*0.8f)/(20.f+1.f);
				}
				else if((HeightPID.Status.Err[0])>-0.3f&&AccPID.Outputs.I<acc_i_output_min*0.6f)
				{
					AccPID.Outputs.I=(30.f*AccPID.Outputs.I+acc_i_output_min*0.6f)/(30.f+1.f);
				}
				
			}
		}
	}
	if(CtrlSys.stop)
	{
		HeightPID.Inputs.Ref=-0.5f;
	}
	else
	{
		if(ABS(HeightPID.Status.Err[0])<1.0f)
		{
			HeightPID.Inputs.Ref+=ref_height_rate*0.008f;//0.004f;
		}
		else
		{
			if(sign(HeightPID.Status.Err[0])!=sign(ref_height_rate))
			{
				HeightPID.Inputs.Ref+=ref_height_rate*0.008f;//0.004f;
			}
		}
		
	}
	if(CtrlSys.test_mode!=0||CtrlSys.height_close_loop_mode==0)
	{
		HeightPID.Inputs.Ref=0;
		HeightPID.Inputs.Measure=height;
		PID_Reset(&HeightPID);
		
		HeightRatePID.Inputs.Ref=0;
		HeightRatePID.Inputs.Measure=speed;
		HeightRatePID.Inputs.MeasureSpeed=acc;
		PID_Calculate(&HeightRatePID);
		PID_Reset2(&HeightRatePID);
		
		AccPID.Inputs.Ref=0;
		AccPID.Inputs.Measure=acc;
		PID_Calculate(&AccPID);
		PID_Reset2(&AccPID);
		
		throttle=CtrlSys.throttle;
	}
	else if(CtrlSys.height_close_loop_mode==1)//1-accel_close_loop
	{
		HeightPID.Inputs.Ref=0;
		HeightPID.Inputs.Measure=height;
		HeightPID.Inputs.MeasureSpeed=speed;
		PID_Reset(&HeightPID);
		
		HeightRatePID.Inputs.Ref=0;
		HeightRatePID.Inputs.Measure=speed;
		HeightRatePID.Inputs.MeasureSpeed=acc;
		PID_Calculate(&HeightRatePID);
		PID_Reset2(&HeightRatePID);
		
		AccPID.Inputs.Ref=ref_accel;
		AccPID.Inputs.Measure=acc;
		PID_Calculate(&AccPID);
		
		throttle=AccPID.Outputs.Output;
	}
	else if(CtrlSys.height_close_loop_mode==2)//2-height_rate_close_loop
	{
		HeightPID.Inputs.Ref=0;
		HeightPID.Inputs.Measure=height;
		HeightPID.Inputs.MeasureSpeed=speed;
		PID_Reset(&HeightPID);
		
		HeightRatePID.Inputs.Ref=ref_height_rate;
		HeightRatePID.Inputs.Measure=speed;
		HeightRatePID.Inputs.MeasureSpeed=acc;
		PID_Calculate(&HeightRatePID);
		
		AccPID.Inputs.Ref=HeightRatePID.Outputs.Output;
		AccPID.Inputs.Measure=acc;
		PID_Calculate(&AccPID);
		
		throttle=AccPID.Outputs.Output;
	}
	else if(CtrlSys.height_close_loop_mode==3)//3-height_close_loop
	{
		HeightPID.Inputs.Measure=height;
		HeightPID.Inputs.MeasureSpeed=speed;
		PID_Calculate(&HeightPID);
		
		HeightRatePID.Inputs.Ref=HeightPID.Outputs.Output;
		HeightRatePID.Inputs.Measure=speed;
		HeightRatePID.Inputs.MeasureSpeed=acc;
		PID_Calculate(&HeightRatePID);
		
		AccPID.Inputs.Ref=HeightRatePID.Outputs.Output;
		AccPID.Inputs.Measure=acc;
		PID_Calculate(&AccPID);
		
		throttle=AccPID.Outputs.Output;
	}
	
	CtrlSys.res_throttle=LIMIT_MIN_MAX(throttle,0,PID_MAX);
	
}
void YawCtrl(void)
{
	if(CtrlSys.test_mode!=0)
	{
		YawPID.Inputs.Ref=0;
		YawPID.Inputs.Measure=GetDeltaAngle(ahrs.yaw-CtrlSys.yaw_bias);
		PID_Reset(&YawPID);
		
		YawRatePID.Inputs.Ref=0;
		YawRatePID.Inputs.Measure=ahrs.gz;
		PID_Reset(&YawRatePID);
	}
	else
	{
		if(CtrlSys.yaw_close_loop)
		{
			if(fabs(YawPID.Status.Err[0])>90&&sign(YawPID.Status.Err[0])==sign(ref_yaw_rate))
			{
				ref_yaw_rate=0;
			}
			if(CtrlSys.stop||!CtrlSys.is_takeoff||fabs(YawPID.Params.Kp)<1e-20f)
//			if(CtrlSys.stop||fabs(YawPID.Params.Kp)<1e-20f)
			{
				YawPID.Inputs.Ref=YawPID.Inputs.Measure;
			}
			else
			{
				if(CtrlSys.height_close_loop_mode!=0)
				{
					if(fabs(ref_height_rate)<HEIGHT_RATE_MAX*0.2f)
					{
						YawPID.Inputs.Ref=GetDeltaAngle(YawPID.Inputs.Ref+ref_yaw_rate*0.01f);
					}
					else if(fabs(ref_height_rate)<HEIGHT_RATE_MAX*0.3f)
					{
						YawPID.Inputs.Ref=GetDeltaAngle(YawPID.Inputs.Ref+ref_yaw_rate*0.005f);
					}
					else if(fabs(ref_height_rate)<HEIGHT_RATE_MAX*0.5f)
					{
						YawPID.Inputs.Ref=GetDeltaAngle(YawPID.Inputs.Ref+ref_yaw_rate*0.003f);
					}
					else
					{
						YawPID.Inputs.Ref=GetDeltaAngle(YawPID.Inputs.Ref+ref_yaw_rate*0.001f);
					}
				}
				else
				{
					YawPID.Inputs.Ref=GetDeltaAngle(YawPID.Inputs.Ref+ref_yaw_rate*0.01f);
					
				}
			}
			YawPID.Inputs.Measure=GetDeltaAngle(ahrs.yaw-CtrlSys.yaw_bias);
			
			YawPID.Inputs.MeasureSpeed=ahrs.gz;
			YawPID.Inputs.Error=GetDeltaAngle(YawPID.Inputs.Ref-YawPID.Inputs.Measure);
			PID_Calculate(&YawPID);
			YawPID.Outputs.I=0;//20151027
			
			YawRatePID.Inputs.Ref=YawPID.Outputs.Output;
			YawRatePID.Inputs.Measure=ahrs.gz;
			//YawRatePID.Outputs.I=0;//20151027
			PID_Calculate(&YawRatePID);
		}
		else
		{
			YawPID.Inputs.Ref=YawPID.Inputs.Measure;
			YawPID.Inputs.Measure=ahrs.yaw-CtrlSys.yaw_bias;
			PID_Reset(&YawPID);
			
			if(CtrlSys.stop||!CtrlSys.is_takeoff)
			{
				YawRatePID.Inputs.Ref=0;
			}
			else
			{
				if(CtrlSys.height_close_loop_mode!=0)
				{
					if(fabs(ref_height_rate)<HEIGHT_RATE_MAX*0.2f)
					{
						YawRatePID.Inputs.Ref=ref_yaw_rate;
					}
					else if(fabs(ref_height_rate)<HEIGHT_RATE_MAX*0.3f)
					{
						YawRatePID.Inputs.Ref=ref_yaw_rate*0.7f;
					}
					else if(fabs(ref_height_rate)<HEIGHT_RATE_MAX*0.5f)
					{
						YawRatePID.Inputs.Ref=ref_yaw_rate*0.3f;
					}
					else
					{
						YawRatePID.Inputs.Ref=ref_yaw_rate*0.1f;
					}
				}
				else
				{
//					if(CtrlSys.throttle_raw>2048*0.8f)
//					{
//						YawRatePID.Inputs.Ref=ref_yaw_rate;
//					}
//					else if(CtrlSys.throttle_raw>2048*0.4f)
//					{
//						YawRatePID.Inputs.Ref=ref_yaw_rate*0.5f;
//					}
//					else
//					{
//						YawRatePID.Inputs.Ref=ref_yaw_rate*0.1f;
//					}
					YawRatePID.Inputs.Ref=ref_yaw_rate;
				}
			}
			
			YawRatePID.Inputs.Measure=ahrs.gz;
			PID_Calculate(&YawRatePID);
		}
	}
	CtrlSys.res_yaw=LIMIT_MIN_MAX(YawRatePID.Outputs.Output,-PID_MAX,PID_MAX);
}

//CtrlSys.test_mode:
//0-normal,1-motor test
//2-esc_setting,3-esc_no_setting
//4-double loop,5-single loop
void PitchCtrl(void)
{
	float pitch;
	pitch=ahrs.pitch-CtrlSys.bias_pitch;
	if(CtrlSys.test_mode==0)//0-normal
	{
		if(CtrlSys.rc_ok==1)
		{
			PitchPID.Inputs.Ref=ref_pitch;
		}
		else
		{
			PitchPID.Inputs.Ref=0;
		}
		PitchPID.Inputs.Measure=pitch;
		PitchPID.Inputs.MeasureSpeed=ahrs.gy;
		PID_Calculate(&PitchPID);
		
		PitchRatePID.Inputs.Ref=PitchPID.Outputs.Output;
		PitchRatePID.Inputs.Measure=ahrs.gy;
		PID_Calculate(&PitchRatePID);
	}
	else if(CtrlSys.test_mode==4)//4-double loop
	{
		if(CtrlSys.pitch_test)
		{
			PitchPID.Inputs.Ref=ref_pitch;
			PitchPID.Inputs.Measure=pitch;
			PitchPID.Inputs.MeasureSpeed=ahrs.gy;
			PID_Calculate(&PitchPID);
			
			PitchRatePID.Inputs.Ref=PitchPID.Outputs.Output;
			PitchRatePID.Inputs.Measure=ahrs.gy;
			PID_Calculate(&PitchRatePID);
		}
		else
		{
			PitchPID.Inputs.Ref=0;
			PitchPID.Inputs.Measure=pitch;
			PID_Reset(&PitchPID);
			
			PitchRatePID.Inputs.Ref=0;
			PitchRatePID.Inputs.Measure=ahrs.gy;
			PID_Reset(&PitchRatePID);
		}
	}
	else if(CtrlSys.test_mode==5)//5-single loop
	{
		if(CtrlSys.pitch_test)
		{
			PitchPID.Inputs.Ref=0;
			PitchPID.Inputs.Measure=pitch;
			PID_Reset(&PitchPID);
			
			PitchRatePID.Inputs.Ref=ref_pitch_rate;
			PitchRatePID.Inputs.Measure=ahrs.gy;
			PID_Calculate(&PitchRatePID);
		}
		else
		{
			PitchPID.Inputs.Ref=0;
			PitchPID.Inputs.Measure=pitch;
			PID_Reset(&PitchPID);
			
			PitchRatePID.Inputs.Ref=0;
			PitchRatePID.Inputs.Measure=ahrs.gy;
			PID_Reset(&PitchRatePID);
		}
	}
	else
	{
		PitchPID.Inputs.Ref=0;
		PitchPID.Inputs.Measure=pitch;
		PID_Reset(&PitchPID);
		
		PitchRatePID.Inputs.Ref=0;
		PitchRatePID.Inputs.Measure=ahrs.gy;
		PID_Reset(&PitchRatePID);
	}
	CtrlSys.res_pitch=LIMIT_MIN_MAX(PitchRatePID.Outputs.Output,-PID_MAX,PID_MAX);
}
void RollCtrl(void)
{
	float roll;
	roll=ahrs.roll-CtrlSys.bias_roll;
	if(CtrlSys.test_mode==0)//0-normal
	{
		if(CtrlSys.rc_ok==1)
		{
			RollPID.Inputs.Ref=ref_roll;
		}
		else
		{
			RollPID.Inputs.Ref=0;
		}
		
		RollPID.Inputs.Measure=roll;
		RollPID.Inputs.MeasureSpeed=ahrs.gx;
		PID_Calculate(&RollPID);
		
		RollRatePID.Inputs.Ref=RollPID.Outputs.Output;
		RollRatePID.Inputs.Measure=ahrs.gx;
		PID_Calculate(&RollRatePID);
	}
	else if(CtrlSys.test_mode==4)//4-double loop
	{
		if(CtrlSys.pitch_test==0)
		{
			RollPID.Inputs.Ref=ref_roll;
			RollPID.Inputs.Measure=roll;
			RollPID.Inputs.MeasureSpeed=ahrs.gx;
			PID_Calculate(&RollPID);
			
			RollRatePID.Inputs.Ref=RollPID.Outputs.Output;
			RollRatePID.Inputs.Measure=ahrs.gx;
			PID_Calculate(&RollRatePID);
		}
		else
		{
			RollPID.Inputs.Ref=0;
			RollPID.Inputs.Measure=roll;
			PID_Reset(&RollPID);
			
			RollRatePID.Inputs.Ref=0;
			RollRatePID.Inputs.Measure=ahrs.gx;
			PID_Reset(&RollRatePID);
		}
	}
	else if(CtrlSys.test_mode==5)//5-single loop
	{
		if(CtrlSys.pitch_test==0)
		{
			RollPID.Inputs.Ref=0;
			RollPID.Inputs.Measure=roll;
			PID_Reset(&RollPID);
			
			RollRatePID.Inputs.Ref=ref_roll_rate;
			RollRatePID.Inputs.Measure=ahrs.gx;
			PID_Calculate(&RollRatePID);
		}
		else
		{
			RollPID.Inputs.Ref=0;
			RollPID.Inputs.Measure=roll;
			PID_Reset(&RollPID);
			
			RollRatePID.Inputs.Ref=0;
			RollRatePID.Inputs.Measure=ahrs.gx;
			PID_Reset(&RollRatePID);
		}
	}
	else
	{
		RollPID.Inputs.Ref=0;
		RollPID.Inputs.Measure=roll;
		PID_Reset(&RollPID);
		
		RollRatePID.Inputs.Ref=0;
		RollRatePID.Inputs.Measure=ahrs.gx;
		PID_Reset(&RollRatePID);
	}
	CtrlSys.res_roll=LIMIT_MIN_MAX(RollRatePID.Outputs.Output,-PID_MAX,PID_MAX);
}

