
#ifndef TYPEDEF_H
#define TYPEDEF_H

#include "stm32f4xx.h"




typedef struct
{
volatile char esc_setting_finished;
volatile char rc_ok;
volatile char sensor_alt_ok;
volatile char sensor_gyro_ok;
volatile char sensor_ok;
//volatile char is_static;
volatile char is_takeoff;	
volatile int  rc_ok_cnt;
volatile char height_close_loop_mode;//0-open_loop,1-accel_close_loop,2-height_rate_close_loop,3-height_close_loop
volatile char yaw_close_loop;
volatile char load_success;//加载参数成功
volatile char test_mode;//0：正常模式；1：系统测试；2：设定电调行程；3:电调测试；4:双环模式；5：内环模式
volatile char pitch_test;//1:测试pitch轴，0-测试roll轴
volatile char stop;//测试模式无效
volatile char curve_en;
volatile float duty_min;
volatile float duty_max;
volatile float bias_pitch;
volatile float bias_roll;
volatile uint16_t ref_pitch_raw;
volatile uint16_t ref_roll_raw;
volatile uint16_t ref_yaw_raw;
volatile uint16_t throttle_raw;
volatile uint16_t ref_gimbal_pitch_raw;
volatile uint16_t ref_gimbal_yaw_raw;
volatile float ref_pitch;
volatile float ref_pitch_rate;
volatile float ref_roll;
volatile float ref_roll_rate;
volatile float ref_yaw_rate;
volatile float ref_yaw;
volatile float ref_gimbal_pitch;
volatile float ref_gimbal_yaw;
volatile float ref_gimbal_pitch_rate;
volatile float ref_gimbal_yaw_rate;
volatile float ref_height_rate;
volatile float height_bias;
volatile float yaw_bias;
volatile float ref_accel;
volatile float accel_output_bias;
volatile float acc_line_z_bias;
volatile float throttle;
volatile float throttle_esc_setting;
volatile float throttle_mean;
volatile float idling_throttle;
volatile float res_yaw;
volatile float res_pitch;
volatile float res_roll;
volatile float res_throttle;
volatile float duty_scope;
volatile int acc_z_is_init;
}CtrlSys_TypeDef;










#endif



