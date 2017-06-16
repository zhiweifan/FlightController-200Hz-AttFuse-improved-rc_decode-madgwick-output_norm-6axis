

#include "main.h"


CtrlSys_TypeDef CtrlSys;
PID_TypeDef PitchPID,PitchRatePID;
PID_TypeDef RollPID,RollRatePID;
PID_TypeDef YawPID,YawRatePID;
PID_TypeDef AccPID,HeightPID,HeightRatePID;
//x轴指向前，y轴指向左
//左前电机逆时针，右前电机顺时针
//左前，右前，左后，右后电机分别是1,2，3,4号电机

void CTRL_Config(void);

int freq_cnt,freq;
int main(void)
{
	PID_StructInit(&PitchPID);
	PID_StructInit(&PitchRatePID);
	PID_StructInit(&RollPID);
	PID_StructInit(&RollRatePID);
	PID_StructInit(&YawPID);
	PID_StructInit(&YawRatePID);
	PID_StructInit(&AccPID);
	PID_StructInit(&HeightRatePID);
	PID_StructInit(&HeightPID);
	CtrlSys.load_success=GetConfig();
	CTRL_Config();
	TIME_Config();
	LED_Config();
	BEEP_Config();
	USART1_Config();
	USART2_Config();
	PWM_Config();
	DataFrame_Init();
	
	SysTime_Config();
	TIME_Reset(TIME_ALL);
	if(CtrlSys.test_mode==2)
	{
		BEEP_SetStatus(BEEP_STATUS_ACK);
		SetMotorThrottle(SQRT_PID_MAX);
		while(CtrlSys.throttle_raw<2048)
		{
			DataFrame_GetData(&rxdf1);
			BEEP_Routing();
			if(!TIME_CheckTimeUs(TIME_DELAY,3000000))
			{
				CtrlSys.test_mode=0;
				CtrlSys.stop=1;
				CtrlSys.esc_setting_finished=1;
				//SetConfig();
				break;
			}
		}
		if(CtrlSys.test_mode==2)
		{
			TIME_Reset(TIME_DELAY);
			while(!TIME_CheckTimeUs(TIME_DELAY,10000000)&&CtrlSys.throttle_raw>2048)//最长10s
			{
				DataFrame_GetData(&rxdf1);
				BEEP_Routing();
			}
			SetMotorStop();
			BEEP_SetStatus(BEEP_STATUS_ACK);
			TIME_Reset(TIME_DELAY);
			while(!TIME_CheckTimeUs(TIME_DELAY,5000000))
			{
				BEEP_Routing();
			}
			CtrlSys.esc_setting_finished=1;
			BEEP_SetStatus(BEEP_STATUS_ESC);
		}
	}
	else if(CtrlSys.test_mode==3)
	{
		CtrlSys.esc_setting_finished=1;
		BEEP_SetStatus(BEEP_STATUS_ESC);
	}
	else
	{
		//BEEP_SetStatus(BEEP_STATUS_ACK);
	}
	CtrlSys.esc_setting_finished=1;
	TIME_Reset(TIME_ALL);
	IMU_Config();
	ALT_FILTER_Config();
	while(1)
	{
		IMU_Update();//3400Hz
		ALT_Update();//3400Hz->2365Hz
		freq_cnt++;
		if(systicks>=100)
		{
			systicks=0;
			freq=freq_cnt;
			freq_cnt=0;
			
		}
		BEEP_Routing();
		LED_SetStatus(LED_ALL,LED_STATUS_IDLE);
		DataFrame_UpdateReadIndex(&txdf1);
		DataFrame_UpdateReadIndex(&txdf2);
		DataFrame_GetData(&rxdf1);
		DataFrame_GetData(&rxdf2);
		if(TIME_CheckTimeUs(TIME_DELAY,10000))
		{
			
			
			if(CtrlSys.curve_en)
			{
				DataFrame_SendData(&txdf1);
			}
			
		}
		if(TIME_CheckTimeUs(TIME_RC_OK,50000))
		{
			if(CtrlSys.rc_ok_cnt>=1)
			{
				CtrlSys.rc_ok=1;
				CtrlSys.rc_ok_cnt=0;
			}
			else
			{
				CtrlSys.rc_ok=0;
			}
		}
	}

}

void CTRL_Config(void)
{
	if(!CtrlSys.load_success)
	{
		CtrlSys.duty_min=DUTY_MIN;
		CtrlSys.duty_max=DUTY_MAX;
	}
	CtrlSys.duty_scope=(CtrlSys.duty_max-CtrlSys.duty_min);
	if(!CtrlSys.load_success)
	{
		ALT_FILTER_SetDefaultQR();
		CtrlSys.accel_output_bias=ACC_OUTPUT_BIAS;
		CtrlSys.bias_pitch=0.f;
		CtrlSys.bias_roll=0;
		CtrlSys.test_mode=0;
		CtrlSys.yaw_bias=0;
		CtrlSys.idling_throttle=NO_TAKEOFF_THR*0.5f;
		#if FLIGHT_BOARD==1
		CtrlSys.acc_line_z_bias=0;//-0.22f;//-0.095f;//0.845f;
		#elif FLIGHT_BOARD==2
		CtrlSys.acc_line_z_bias=0;//-0.045f;
		#else
		CtrlSys.acc_line_z_bias=0;
		#endif
		#if AXIS_NUM==4
		PitchPID.Params.Kp=15.f;
		PitchPID.Params.Ki=0;
		PitchPID.Params.Kd=0.0f;
		PitchPID.Params.ErrDevTc=0.f;
		PitchPID.Params.RefTc=0.f;
		PitchRatePID.Params.MeasureTc=0.f;
		PitchRatePID.Params.RefTc=0.0f;
		PitchRatePID.Params.Kp=3.9f;
		PitchRatePID.Params.Ki=0.0117f;
		PitchRatePID.Params.Kd=46.9f;
		PitchRatePID.Params.ErrDevTc=0.0f;
		
		RollPID.Params.Kp=15.f;
		RollPID.Params.Ki=0.0f;
		RollPID.Params.Kd=0.0f;
		RollPID.Params.ErrDevTc=0.f;
		RollPID.Params.RefTc=0.f;
		RollRatePID.Params.MeasureTc=0.f;
		RollRatePID.Params.RefTc=0.0f;
		RollRatePID.Params.Kp=3.9f;
		RollRatePID.Params.Ki=0.0117f;
		RollRatePID.Params.Kd=46.9f;
		RollRatePID.Params.ErrDevTc=0.0f;
		
		YawPID.Params.Kp=5.0f;
		
		YawRatePID.Params.Kp=19.0f;
		YawRatePID.Params.Ki=0.39f;
		YawRatePID.Params.MeasureTc=0.f;
		
		AccPID.Params.Kp=390.6f;
		AccPID.Params.Ki=19.5f;
		AccPID.Params.MeasureTc=0.f;
		
		HeightRatePID.Params.Kp=3.1f;
		
		HeightPID.Params.Kp=1.6f;//2.5f;
		HeightPID.Params.Ki=0.00000f;
		HeightPID.Params.Kd=0.3f;//0.5f;
		HeightPID.Params.ErrDevTc=0.f;
		
		#elif AXIS_NUM==6
		
		PitchPID.Params.Kp=10.f;
		PitchPID.Params.Ki=0;
		PitchPID.Params.Kd=0.0f;
		PitchPID.Params.ErrDevTc=0.f;
		PitchPID.Params.RefTc=0.f;
		PitchRatePID.Params.MeasureTc=0.f;
		PitchRatePID.Params.RefTc=0.0f;
		PitchRatePID.Params.Kp=10.f;
		PitchRatePID.Params.Ki=0.012f;
		PitchRatePID.Params.Kd=45.f;
		PitchRatePID.Params.ErrDevTc=1.0f;
		
		RollPID.Params.Kp=10.f;
		RollPID.Params.Ki=0.0f;
		RollPID.Params.Kd=0.0f;
		RollPID.Params.ErrDevTc=0.f;
		RollPID.Params.RefTc=0.f;
		RollRatePID.Params.MeasureTc=0.f;
		RollRatePID.Params.RefTc=0.0f;
		RollRatePID.Params.Kp=10.f;
		RollRatePID.Params.Ki=0.012f;
		RollRatePID.Params.Kd=45.f;
		RollRatePID.Params.ErrDevTc=1.0f;
		
		YawPID.Params.Kp=10.0f;
		YawPID.Params.Kd=3.f;
		
		YawRatePID.Params.Kp=23.0f;
		YawRatePID.Params.Ki=0.1f;
		YawRatePID.Params.Kd=80.f;
		YawRatePID.Params.MeasureTc=0.f;
		YawRatePID.Params.ErrDevTc=2.f;
		
		AccPID.Params.Kp=390.6f;
		AccPID.Params.Ki=19.5f;
		AccPID.Params.MeasureTc=0.f;
		
		HeightRatePID.Params.Kp=3.1f;
		
		HeightPID.Params.Kp=1.6f;//2.5f;
		HeightPID.Params.Ki=0.00000f;
		HeightPID.Params.Kd=0.3f;//0.5f;
		HeightPID.Params.ErrDevTc=0.f;
		
		#endif
		
	}
	 //flash里没有存储的参数在这里初始化
	
	/////////////////////PID/////////////////////////////
	PitchPID.Limits.OutputLimit=ROT_MAX;
	PitchPID.Limits.IntLimit=ROT_MAX*0.5f;
	PitchPID.Limits.PropLimit=ROT_MAX;
	PitchPID.Limits.DevLimit=ROT_MAX*0.5f;
	PitchPID.Limits.RefMax=ANGLE_MAX;
	PitchPID.Limits.MeasureMax=ANGLE_MAX*1.3f;
	PitchPID.Limits.ErrMax=ANGLE_MAX*2.0f;
	PitchPID.Limits.ErrDevMax=ROT_MAX;
	PitchPID.Settings.UseMeasureSpeed=1;
	
	PitchRatePID.Limits.OutputLimit=PID_MAX;
	PitchRatePID.Limits.IntLimit=PID_MAX*0.2f;
	PitchRatePID.Limits.PropLimit=PID_MAX;
	PitchRatePID.Limits.DevLimit=PID_MAX*0.3f;
	PitchRatePID.Limits.RefMax=ROT_MAX;
	PitchRatePID.Limits.MeasureMax=ROT_MAX*1.3f;
	PitchRatePID.Limits.ErrMax=ROT_MAX*2.0f;
	PitchRatePID.Limits.ErrDevMax=ROT_MAX/10.f;
	
	RollPID.Limits.OutputLimit=ROT_MAX;
	RollPID.Limits.IntLimit=ROT_MAX*0.5f;
	RollPID.Limits.PropLimit=ROT_MAX;
	RollPID.Limits.DevLimit=ROT_MAX*0.5f;
	RollPID.Limits.RefMax=ANGLE_MAX;
	RollPID.Limits.MeasureMax=ANGLE_MAX*1.3f;
	RollPID.Limits.ErrMax=ANGLE_MAX*2.0f;
	RollPID.Limits.ErrDevMax=ROT_MAX;
	RollPID.Settings.UseMeasureSpeed=1;
	
	RollRatePID.Limits.OutputLimit=PID_MAX;
	RollRatePID.Limits.IntLimit=PID_MAX*0.2f;
	RollRatePID.Limits.PropLimit=PID_MAX;
	RollRatePID.Limits.DevLimit=PID_MAX*0.3f;
	RollRatePID.Limits.RefMax=ROT_MAX;
	RollRatePID.Limits.MeasureMax=ROT_MAX*1.3f;
	RollRatePID.Limits.ErrMax=ROT_MAX*2.0f;
	RollRatePID.Limits.ErrDevMax=ROT_MAX/10.f;
	
	YawPID.Limits.OutputLimit=ROT_MAX_YAW;
	YawPID.Limits.IntLimit=ROT_MAX_YAW*0.5f;
	YawPID.Limits.PropLimit=ROT_MAX_YAW*2;
	YawPID.Limits.DevLimit=ROT_MAX_YAW;
	YawPID.Limits.ErrDevMax=ROT_MAX_YAW*1.3f;
	YawPID.Settings.UseMeasureSpeed=1;
	YawPID.Settings.UseError=1;
	
	YawRatePID.Limits.OutputLimit=PID_MAX;
	YawRatePID.Limits.IntLimit=PID_MAX*0.2f;
	YawRatePID.Limits.PropLimit=PID_MAX;
	YawRatePID.Limits.DevLimit=PID_MAX*0.3f;
	YawRatePID.Limits.RefMax=ROT_MAX_YAW;
	YawRatePID.Limits.MeasureMax=ROT_MAX_YAW*1.3f;
	YawRatePID.Limits.ErrMax=ROT_MAX_YAW*2.0f;
	YawRatePID.Limits.ErrDevMax=ROT_MAX_YAW/2.f;
	
	AccPID.Limits.OutputLimit=PID_MAX;
	AccPID.Limits.IntLimit=PID_MAX*0.7f;
	AccPID.Limits.PropLimit=PID_MAX;
	AccPID.Limits.DevLimit=PID_MAX*0.5f;
	AccPID.Limits.RefMax=ACCEL_MAX;
	AccPID.Limits.MeasureMax=ACCEL_MAX*1.5f;
	AccPID.Limits.ErrMax=ACCEL_MAX*2.0f;
	AccPID.Limits.ErrDevMax=ACCEL_MAX;

	HeightRatePID.Limits.OutputLimit=ACCEL_MAX;
	HeightRatePID.Limits.IntLimit=ACCEL_MAX*0.5f;
	HeightRatePID.Limits.PropLimit=ACCEL_MAX;
	HeightRatePID.Limits.DevLimit=ACCEL_MAX*0.5f;
	//HeightRatePID.Limits.RefMax=HEIGHT_RATE_MAX;
	//HeightRatePID.Limits.MeasureMax=HEIGHT_RATE_MAX*1.3f;
	//HeightRatePID.Limits.ErrMax=HEIGHT_RATE_MAX*2.0f;
	HeightRatePID.Limits.ErrDevMax=ACCEL_MAX;
	HeightRatePID.Settings.UseMeasureSpeed=1;
	
	//HeightPID.Limits.OutputLimit=HEIGHT_RATE_MAX;
	//HeightPID.Limits.IntLimit=HEIGHT_RATE_MAX*0.5f;
	//HeightPID.Limits.PropLimit=HEIGHT_RATE_MAX;
	//HeightPID.Limits.DevLimit=HEIGHT_RATE_MAX*0.5f;
	//HeightPID.Limits.ErrMax=1.f;
	//HeightPID.Limits.ErrDevMax=HEIGHT_RATE_MAX*2.f;
	HeightPID.Settings.UseMeasureSpeed=1;
	
	CtrlSys.pitch_test=1;
	CtrlSys.is_takeoff=0;
	CtrlSys.height_close_loop_mode=0;
	CtrlSys.yaw_close_loop=1;
	CtrlSys.rc_ok=0;
	CtrlSys.rc_ok_cnt=0;
	CtrlSys.sensor_ok=0;
	CtrlSys.sensor_alt_ok=0;
	CtrlSys.sensor_gyro_ok=0;
	CtrlSys.stop=1;
	CtrlSys.curve_en=0;
	CtrlSys.esc_setting_finished=0;
	CtrlSys.ref_yaw_raw=0;
	CtrlSys.ref_pitch_raw=0;
	CtrlSys.ref_roll_raw=0;
	CtrlSys.throttle_raw=0;
	CtrlSys.ref_gimbal_yaw_raw=0;
	CtrlSys.ref_gimbal_pitch_raw=0;
	CtrlSys.ref_yaw_rate=0;
	CtrlSys.ref_yaw=0;
	CtrlSys.ref_pitch=0;
	CtrlSys.ref_roll=0;
	CtrlSys.throttle=0;
	CtrlSys.throttle_esc_setting=0;

	CtrlSys.throttle_mean=0;
	CtrlSys.ref_gimbal_yaw=0;
	CtrlSys.ref_gimbal_pitch=0;
	CtrlSys.res_yaw=0;
	CtrlSys.res_pitch=0;
	CtrlSys.res_roll=0;
	CtrlSys.res_throttle=0;
	CtrlSys.acc_z_is_init=0;
}

void SysProcess(void)
{
	SensorProcess();
	CmdProcess();
	StatusTransition();
	CtrlProcess();
	OutputProcess();
}
void ControllerReset(void)
{
	PID_Reset(&PitchPID);
	PID_Reset(&PitchRatePID);
	PID_Reset(&RollPID);
	PID_Reset(&RollRatePID);
	PID_Reset(&YawPID);
	PID_Reset(&YawRatePID);
	PID_Reset2(&AccPID);
	PID_Reset2(&HeightRatePID);
	PID_Reset(&HeightPID);
	HeightPID.Inputs.Ref=-0.5f;
	
	CtrlSys.res_yaw=0;
	CtrlSys.res_pitch=0;
	CtrlSys.res_roll=0;
	CtrlSys.res_throttle=0;
	CtrlSys.throttle_mean=0;

}


