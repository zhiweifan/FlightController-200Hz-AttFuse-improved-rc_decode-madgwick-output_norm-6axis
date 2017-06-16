

#ifndef MAIN_H
#define MAIN_H



#include "stm32f4xx.h"
#include "typedef.h"
#include "sys_m4.h"
#include "math_lib.h"
#include "time.h"
#include "led.h"
#include "beep.h"
#include "usart1.h"
#include "usart2.h"
//#include "pwm.h"
#include "dataframe.h"
#include "pid.h"
#include "eeprom.h"
#include "process_sensor.h"
#include "process_cmd.h"
#include "process_status_transition.h"
#include "process_ctrl.h"
#include "process_output.h"
#include "process_test.h"
#include "systime.h"
#include "alt_filter.h"
#include "imu.h"
#include "gps_kalman.h"

#define FLIGHT_BOARD        3
#define AXIS_NUM						6

#if AXIS_NUM==4
#define ACC_OUTPUT_BIAS     2783
#define TAKEOFF_THR					1343
#define NO_TAKEOFF_THR			1221
#elif AXIS_NUM==6
#define ACC_OUTPUT_BIAS     2560
#define TAKEOFF_THR					1000
#define NO_TAKEOFF_THR			600
#endif

#define ANGLE_MAX														30.f
#define ROT_MAX															200.f
#define ROT_MAX_YAW													(ROT_MAX/3.f)
#define GIMBAL_ROT_MAX											(ROT_MAX/3.f)
#define GIMBAL_ROT_MAX_YAW									(ROT_MAX/3.f)
#define ACCEL_MAX														5.f
#define HEIGHT_RATE_MAX											1.5f//0.018f//0.01f


//一般遥控器行程0.8ms，最低位置1.0ms-1.2ms,最高位置1.2ms-2.0ms
#define DUTY_UP_MAX									(1.95f/5.f)
#define DUTY_UP_MIN									(1.75f/5.f)
#define DUTY_LOW_MAX								(1.25f/5.f)
#define DUTY_LOW_MIN								(1.0f/5.f)
#define DUTY_MIN										(1.1f/5.f)//0.22f
#define DUTY_MAX										(1.9f/5.f)//0.38f
#define DUTY_STOP										(1.1f/5.f)//0.22f
#define PID_MAX											(10000.0f)
#define SQRT_PID_MAX								(100.0f)

#define DUTY_MIN7										(0.f/5.f)//0.22f
#define DUTY_MAX7										(5.f/5.f)//0.38f
#define DUTY_STOP7									(0.f/5.f)//0.22f
#define DUTY_MIN8										(0.f/5.f)//0.22f
#define DUTY_MAX8										(5.f/5.f)//0.38f
#define DUTY_STOP8									(0.f/5.f)//0.22f
#define DUTY_MIN9										(0.f/5.f)//0.22f
#define DUTY_MAX9										(5.f/5.f)//0.38f
#define DUTY_STOP9									(0.f/5.f)//0.22f
#define DUTY_MIN10									(0.f/5.f)//0.22f
#define DUTY_MAX10									(5.f/5.f)//0.38f
#define DUTY_STOP10									(0.f/5.f)//0.22f
#define DUTY_MIN11									(0.f/5.f)//0.22f
#define DUTY_MAX11									(5.f/5.f)//0.38f
#define DUTY_STOP11									(0.f/5.f)//0.22f
#define DUTY_MIN12									(0.f/5.f)//0.22f
#define DUTY_MAX12									(5.f/5.f)//0.38f
#define DUTY_STOP12									(0.f/5.f)//0.22f
#define DUTY_MIN13									(0.f/5.f)//0.22f
#define DUTY_MAX13									(5.f/5.f)//0.38f
#define DUTY_STOP13									(0.f/5.f)//0.22f
#define DUTY_MIN14									(0.f/5.f)//0.22f
#define DUTY_MAX14									(5.f/5.f)//0.38f
#define DUTY_STOP14									(0.f/5.f)//0.22f
#define DUTY_MIN15									(0.f/5.f)//0.22f
#define DUTY_MAX15									(5.f/5.f)//0.38f
#define DUTY_STOP15									(0.f/5.f)//0.22f


extern CtrlSys_TypeDef CtrlSys;
extern PID_TypeDef PitchPID,PitchRatePID;
extern PID_TypeDef RollPID,RollRatePID;
extern PID_TypeDef YawPID,YawRatePID;
extern PID_TypeDef AccPID,HeightPID,HeightRatePID;
extern void SysProcess(void);
extern void ControllerReset(void);

#endif


