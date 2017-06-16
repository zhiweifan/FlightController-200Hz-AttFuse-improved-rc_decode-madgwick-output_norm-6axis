
#include "process_cmd.h"
#include "main.h"


void CmdProcess(void)
{
	CtrlSys.ref_pitch=(float)(CtrlSys.ref_pitch_raw-2048)*ANGLE_MAX/2048.f;
	CtrlSys.ref_pitch_rate=(float)(CtrlSys.ref_pitch_raw-2048)*ROT_MAX/2048.f;
	CtrlSys.ref_roll=(float)(CtrlSys.ref_roll_raw-2048)*ANGLE_MAX/2048.f;
	CtrlSys.ref_roll_rate=(float)(CtrlSys.ref_roll_raw-2048)*ROT_MAX/2048.f;
	CtrlSys.ref_height_rate=(float)(CtrlSys.ref_gimbal_pitch_raw-2048)*HEIGHT_RATE_MAX/2048.f;
	CtrlSys.ref_accel=(float)(CtrlSys.ref_gimbal_pitch_raw-2048)*ACCEL_MAX/2048.f;
	CtrlSys.throttle=CtrlSys.duty_min+CtrlSys.throttle_raw*PID_MAX/4095.f;
	
	//throttle_raw's scope is about 3~4090
	if(CtrlSys.throttle_raw<40)
	{
		CtrlSys.throttle_esc_setting=0;
	}
	else if(CtrlSys.throttle_raw>4055)
	{
		CtrlSys.throttle_esc_setting=SQRT_PID_MAX;
	}
	else
	{
		CtrlSys.throttle_esc_setting=CtrlSys.throttle_raw*SQRT_PID_MAX/4095.f;
	}
	
	CtrlSys.ref_gimbal_yaw_rate=(CtrlSys.ref_gimbal_yaw_raw-2048)*GIMBAL_ROT_MAX_YAW/2048.f;
	CtrlSys.ref_gimbal_pitch_rate=(CtrlSys.ref_gimbal_pitch_raw-2048)*GIMBAL_ROT_MAX/2048.f;
	
	if(CtrlSys.height_close_loop_mode!=0)
	{
		CtrlSys.ref_yaw_rate=(CtrlSys.ref_gimbal_yaw_raw-2048)*ROT_MAX_YAW/2048.f;

	}
	else
	{
		CtrlSys.ref_yaw_rate=(CtrlSys.ref_yaw_raw-2048)*ROT_MAX_YAW/2048.f;
	}
	
	
}

