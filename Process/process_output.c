
#include "process_output.h"
#include "main.h"
#include "math.h"
#include "pwm.h"


void SetMotor(void);

void OutputProcess(void)
{
	if(CtrlSys.stop)
	{
		if(CtrlSys.test_mode==3||CtrlSys.test_mode==2)//2-esc_setting,3-esc_no_setting
		{
			if(CtrlSys.esc_setting_finished)
			{
				SetMotorThrottle(CtrlSys.throttle_esc_setting);
			}
		}
		else
		{
			SetMotorStop();
			ControllerReset();
		}
	}
	else
	{
		if(CtrlSys.test_mode==3||CtrlSys.test_mode==2)//2-esc_setting,3-esc_no_setting
		{
			if(CtrlSys.esc_setting_finished)
			{
				SetMotorThrottle(CtrlSys.throttle_esc_setting);
			}
		}
		else
		{
			SetMotor();
		}
	}
}

void SetMotor(void)
{
	float motor_lf,motor_rf,motor_lb,motor_rb;
	float motor_l,motor_r;
	float throttle_mean,idling_throttle;
	
	idling_throttle=CtrlSys.idling_throttle;
	
	
	#if AXIS_NUM==4
	//左右电机外旋
//	motor_lf=LimitMinMax_float(-CtrlSys.res_pitch+CtrlSys.res_roll-CtrlSys.res_yaw+CtrlSys.res_throttle,idling_throttle,PID_MAX);
//	motor_rf=LimitMinMax_float(-CtrlSys.res_pitch-CtrlSys.res_roll+CtrlSys.res_yaw+CtrlSys.res_throttle,idling_throttle,PID_MAX);
//	motor_lb=LimitMinMax_float(CtrlSys.res_pitch+CtrlSys.res_roll+CtrlSys.res_yaw+CtrlSys.res_throttle,idling_throttle,PID_MAX);
//	motor_rb=LimitMinMax_float(CtrlSys.res_pitch-CtrlSys.res_roll-CtrlSys.res_yaw+CtrlSys.res_throttle,idling_throttle,PID_MAX);
	
	//左右电机内旋
	motor_lf=LimitMinMax_float(-CtrlSys.res_pitch+CtrlSys.res_roll+CtrlSys.res_yaw+CtrlSys.res_throttle,idling_throttle,PID_MAX);//NO.1
	motor_rf=LimitMinMax_float(-CtrlSys.res_pitch-CtrlSys.res_roll-CtrlSys.res_yaw+CtrlSys.res_throttle,idling_throttle,PID_MAX);//NO.2
	motor_lb=LimitMinMax_float(CtrlSys.res_pitch+CtrlSys.res_roll-CtrlSys.res_yaw+CtrlSys.res_throttle,idling_throttle,PID_MAX);//NO.3
	motor_rb=LimitMinMax_float(CtrlSys.res_pitch-CtrlSys.res_roll+CtrlSys.res_yaw+CtrlSys.res_throttle,idling_throttle,PID_MAX);//NO.4
	
	#elif AXIS_NUM==6
	motor_lf=LimitMinMax_float(-CtrlSys.res_pitch+0.5f*CtrlSys.res_roll-      CtrlSys.res_yaw+CtrlSys.res_throttle,idling_throttle,PID_MAX);//NO.1
	motor_rf=LimitMinMax_float(-CtrlSys.res_pitch-0.5f*CtrlSys.res_roll+      CtrlSys.res_yaw+CtrlSys.res_throttle,idling_throttle,PID_MAX);//NO.2
	motor_lb=LimitMinMax_float( CtrlSys.res_pitch+0.5f*CtrlSys.res_roll-      CtrlSys.res_yaw+CtrlSys.res_throttle,idling_throttle,PID_MAX);//NO.3
	motor_rb=LimitMinMax_float( CtrlSys.res_pitch-0.5f*CtrlSys.res_roll+      CtrlSys.res_yaw+CtrlSys.res_throttle,idling_throttle,PID_MAX);//NO.4
	motor_l =LimitMinMax_float(                        CtrlSys.res_roll+      CtrlSys.res_yaw+CtrlSys.res_throttle,idling_throttle,PID_MAX);//NO.5
	motor_r =LimitMinMax_float(                   -    CtrlSys.res_roll-      CtrlSys.res_yaw+CtrlSys.res_throttle,idling_throttle,PID_MAX);//NO.6
	#endif
	if(CtrlSys.stop)
	{
		CtrlSys.throttle_mean=0;
	}
	else
	{
		#if AXIS_NUM==4
		throttle_mean=(motor_lf+motor_rf+motor_lb+motor_rb)/4.f;
		#elif  AXIS_NUM==6
		throttle_mean=(motor_lf+motor_rf+motor_lb+motor_rb+motor_l+motor_r)/6.f;
		#endif
		throttle_mean=LIMIT_MIN_MAX(throttle_mean,0,PID_MAX);
		CtrlSys.throttle_mean=(CtrlSys.throttle_mean*50.f+throttle_mean)/(50.f+1.0f);
	}
	
	motor_lf=sqrt(motor_lf);
	motor_rf=sqrt(motor_rf);
	motor_lb=sqrt(motor_lb);
	motor_rb=sqrt(motor_rb);
	motor_l =sqrt(motor_l );
	motor_r =sqrt(motor_r );
	
	SetMotor1(motor_lf,0);
	SetMotor2(motor_rf,0);
	SetMotor3(motor_lb,0);
	SetMotor4(motor_rb,0);
	SetMotor5(motor_l ,0);
	SetMotor6(motor_r ,0);
}

float SetMotor1(float output,char stop)
{
	float duty;
	if(stop)
	{
		duty=DUTY_STOP;
	}
	else
	{
		if(output<0)
		{
			output=0;
		}
		else if(output>SQRT_PID_MAX)
		{
			output=SQRT_PID_MAX;
		}
		duty=CtrlSys.duty_min+output*CtrlSys.duty_scope/SQRT_PID_MAX;
	}
	return SetPWM1(duty);
}
float SetMotor2(float output,char stop)
{
	float duty;
	if(stop)
	{
		duty=DUTY_STOP;
	}
	else
	{
		if(output<0)
		{
			output=0;
		}
		else if(output>SQRT_PID_MAX)
		{
			output=SQRT_PID_MAX;
		}
		duty=CtrlSys.duty_min+output*CtrlSys.duty_scope/SQRT_PID_MAX;
	}
	return SetPWM2(duty);
}
float SetMotor3(float output,char stop)
{
	float duty;
	if(stop)
	{
		duty=DUTY_STOP;
	}
	else
	{
		if(output<0)
		{
			output=0;
		}
		else if(output>SQRT_PID_MAX)
		{
			output=SQRT_PID_MAX;
		}
		duty=CtrlSys.duty_min+output*CtrlSys.duty_scope/SQRT_PID_MAX;
	}
	return SetPWM3(duty);
}
float SetMotor4(float output,char stop)
{
	float duty;
	if(stop)
	{
		duty=DUTY_STOP;
	}
	else
	{
		if(output<0)
		{
			output=0;
		}
		else if(output>SQRT_PID_MAX)
		{
			output=SQRT_PID_MAX;
		}
		duty=CtrlSys.duty_min+output*CtrlSys.duty_scope/SQRT_PID_MAX;
	}
	return SetPWM4(duty);
}
float SetMotor5(float output,char stop)
{
	float duty;
	if(stop)
	{
		duty=DUTY_STOP;
	}
	else
	{
		if(output<0)
		{
			output=0;
		}
		else if(output>SQRT_PID_MAX)
		{
			output=SQRT_PID_MAX;
		}
		duty=CtrlSys.duty_min+output*CtrlSys.duty_scope/SQRT_PID_MAX;
	}
	return SetPWM5(duty);
}
float SetMotor6(float output,char stop)
{
	float duty;
	if(stop)
	{
		duty=DUTY_STOP;
	}
	else
	{
		if(output<0)
		{
			output=0;
		}
		else if(output>SQRT_PID_MAX)
		{
			output=SQRT_PID_MAX;
		}
		duty=CtrlSys.duty_min+output*CtrlSys.duty_scope/SQRT_PID_MAX;
	}
	return SetPWM6(duty);
}
float SetMotor7(float output,char stop)
{
	float duty;
	if(stop)
	{
		duty=DUTY_STOP7;
	}
	else
	{
		if(output<0)
		{
			output=0;
		}
		else if(output>SQRT_PID_MAX)
		{
			output=SQRT_PID_MAX;
		}
		duty=DUTY_MIN7+output*(DUTY_MAX7-DUTY_MIN7)/SQRT_PID_MAX;
	}
	return SetPWM7(duty);
}
float SetMotor8(float output,char stop)
{
	float duty;
	if(stop)
	{
		duty=DUTY_STOP8;
	}
	else
	{
		if(output<0)
		{
			output=0;
		}
		else if(output>SQRT_PID_MAX)
		{
			output=SQRT_PID_MAX;
		}
		duty=DUTY_MIN8+output*(DUTY_MAX8-DUTY_MIN8)/SQRT_PID_MAX;
	}
	return SetPWM8(duty);
}
float SetMotor9(float output,char stop)
{
	float duty;
	if(stop)
	{
		duty=DUTY_STOP9;
	}
	else
	{
		if(output<0)
		{
			output=0;
		}
		else if(output>SQRT_PID_MAX)
		{
			output=SQRT_PID_MAX;
		}
		duty=DUTY_MIN9+output*(DUTY_MAX9-DUTY_MIN9)/SQRT_PID_MAX;
	}
	return SetPWM9(duty);
}
float SetMotor10(float output,char stop)
{
	float duty;
	if(stop)
	{
		duty=DUTY_STOP10;
	}
	else
	{
		if(output<0)
		{
			output=0;
		}
		else if(output>SQRT_PID_MAX)
		{
			output=SQRT_PID_MAX;
		}
		duty=DUTY_MIN10+output*(DUTY_MAX10-DUTY_MIN10)/SQRT_PID_MAX;
	}
	return SetPWM10(duty);
}
float SetMotor11(float output,char stop)
{
	float duty;
	if(stop)
	{
		duty=DUTY_STOP11;
	}
	else
	{
		if(output<0)
		{
			output=0;
		}
		else if(output>SQRT_PID_MAX)
		{
			output=SQRT_PID_MAX;
		}
		duty=DUTY_MIN11+output*(DUTY_MAX11-DUTY_MIN11)/SQRT_PID_MAX;
	}
	return SetPWM11(duty);
}
float SetMotor12(float output,char stop)
{
	float duty;
	if(stop)
	{
		duty=DUTY_STOP12;
	}
	else
	{
		if(output<0)
		{
			output=0;
		}
		else if(output>SQRT_PID_MAX)
		{
			output=SQRT_PID_MAX;
		}
		duty=DUTY_MIN12+output*(DUTY_MAX12-DUTY_MIN12)/SQRT_PID_MAX;
	}
	return SetPWM12(duty);
}
float SetMotor13(float output,char stop)
{
	float duty;
	if(stop)
	{
		duty=DUTY_STOP13;
	}
	else
	{
		if(output<0)
		{
			output=0;
		}
		else if(output>SQRT_PID_MAX)
		{
			output=SQRT_PID_MAX;
		}
		duty=DUTY_MIN13+output*(DUTY_MAX13-DUTY_MIN13)/SQRT_PID_MAX;
	}
	return SetPWM13(duty);
}
float SetMotor14(float output,char stop)
{
	float duty;
	if(stop)
	{
		duty=DUTY_STOP14;
	}
	else
	{
		if(output<0)
		{
			output=0;
		}
		else if(output>SQRT_PID_MAX)
		{
			output=SQRT_PID_MAX;
		}
		duty=DUTY_MIN14+output*(DUTY_MAX14-DUTY_MIN14)/SQRT_PID_MAX;
	}
	return SetPWM14(duty);
}
float SetMotor15(float output,char stop)
{
	float duty;
	if(stop)
	{
		duty=DUTY_STOP15;
	}
	else
	{
		if(output<0)
		{
			output=0;
		}
		else if(output>SQRT_PID_MAX)
		{
			output=SQRT_PID_MAX;
		}
		duty=DUTY_MIN15+output*(DUTY_MAX15-DUTY_MIN15)/SQRT_PID_MAX;
	}
	return SetPWM15(duty);
}
void SetMotorThrottle(float output)
{
	SetMotor1(output,0);
	SetMotor2(output,0);
	SetMotor3(output,0);
	SetMotor4(output,0);
	SetMotor5(output,0);
	SetMotor6(output,0);
	
	SetMotor7(output,0);
	SetMotor8(output,0);
	SetMotor9(output,0);
	SetMotor10(output,0);
	SetMotor11(output,0);
	SetMotor12(output,0);
	SetMotor13(output,0);
	SetMotor14(output,0);
	SetMotor15(output,0);
	
}
void SetMotorStop(void)
{
	SetMotor1(0,1);
	SetMotor2(0,1);
	SetMotor3(0,1);
	SetMotor4(0,1);
	SetMotor5(0,1);
	SetMotor6(0,1);
	
	SetMotor7(0,1);
	SetMotor8(0,1);
	SetMotor9(0,1);
	SetMotor10(0,1);
	SetMotor11(0,1);
	SetMotor12(0,1);
	SetMotor13(0,1);
	SetMotor14(0,1);
	SetMotor15(0,1);
}
float my_sqrt(float x)
{
	if(x>0)
	{
		return sqrt(x);
	}
	else if(x<0)
	{
		return sqrt(-x);
	}
	else
	{
		return 0;
	}
}







