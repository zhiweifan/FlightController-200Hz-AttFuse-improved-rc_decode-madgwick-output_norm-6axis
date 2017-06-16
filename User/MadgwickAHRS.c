//=====================================================================================================
// MadgwickAHRS.c
//=====================================================================================================
//
// Implementation of Madgwick's IMU and AHRS algorithms.
// See: http://www.x-io.co.uk/node/8#open_source_ahrs_and_imu_algorithms
//
// DateAuthor          Notes
// 29/09/2011SOH Madgwick    Initial release
// 02/10/2011SOH MadgwickOptimised for reduced CPU load
//
//=====================================================================================================

//---------------------------------------------------------------------------------------------------
// Header files
 
#include "MadgwickAHRS.h"
#include <math.h>
//float pitch_rad_last;
float _gz_rad,_yaw_rad;
//---------------------------------------------------------------------------------------------------
// Definitions
 
//#define sampleFreq 512.0f// sample frequency in Hz
//#define betaDef 0.1f// 2 * proportional gain
 
//---------------------------------------------------------------------------------------------------
// Variable definitions
 
//volatile float beta = betaDef;// 2 * proportional gain (Kp)
volatile float q0 = 1.0f, q1 = 0.0f, q2 = 0.0f, q3 = 0.0f;// quaternion of sensor frame relative to auxiliary frame
 
//MadgwickAhrs_TypeDef madgwick_ahrs;
 
//---------------------------------------------------------------------------------------------------
// Function declarations
 
float invSqrt(float x);
 
//====================================================================================================
// Functions
 
//---------------------------------------------------------------------------------------------------
// AHRS algorithm update
//attention:mag will effect pitch and roll  
void AHRS_Update9(float gx, float gy, float gz, float ax, float ay, float az, float mx, float my, float mz,float beta,float deltat) 
{
	float recipNorm;
	float s0, s1, s2, s3;
	float qDot1, qDot2, qDot3, qDot4;
	float hx, hy;
	float _2q0mx, _2q0my, _2q0mz, _2q1mx, _2bx, _2bz, _4bx, _4bz, 
		_2q0, _2q1, _2q2, _2q3, _2q0q2, _2q2q3, q0q0, q0q1, q0q2, q0q3, q1q1, q1q2, q1q3, q2q2, q2q3, q3q3;
	
	// Use IMU algorithm if magnetometer measurement invalid (avoids NaN in magnetometer normalisation)
	if((mx == 0.0f) && (my == 0.0f) && (mz == 0.0f)) 
	{
		AHRS_Update6(gx, gy, gz, ax, ay, az,beta,deltat);
		return;
	}
	gx=AHRS_D2R(gx);
	gy=AHRS_D2R(gy);
	gz=AHRS_D2R(gz);
	_gz_rad=gz;
	// Rate of change of quaternion from gyroscope
	qDot1 = 0.5f * (-q1 * gx - q2 * gy - q3 * gz);
	qDot2 = 0.5f * (q0 * gx + q2 * gz - q3 * gy);
	qDot3 = 0.5f * (q0 * gy - q1 * gz + q3 * gx);
	qDot4 = 0.5f * (q0 * gz + q1 * gy - q2 * gx);
	 
	// Compute feedback only if accelerometer measurement valid (avoids NaN in accelerometer normalisation)
	if(!((ax == 0.0f) && (ay == 0.0f) && (az == 0.0f))) 
	{
		// Normalise accelerometer measurement
		recipNorm = invSqrt(ax * ax + ay * ay + az * az);
		ax *= recipNorm;
		ay *= recipNorm;
		az *= recipNorm;  
		 
		// Auxiliary variables to avoid repeated arithmetic
		_2q0mx = 2.0f * q0 * mx;
		_2q0my = 2.0f * q0 * my;
		_2q0mz = 2.0f * q0 * mz;
		_2q1mx = 2.0f * q1 * mx;
		_2q0 = 2.0f * q0;
		_2q1 = 2.0f * q1;
		_2q2 = 2.0f * q2;
		_2q3 = 2.0f * q3;
		_2q0q2 = 2.0f * q0 * q2;
		_2q2q3 = 2.0f * q2 * q3;
		q0q0 = q0 * q0;
		q0q1 = q0 * q1;
		q0q2 = q0 * q2;
		q0q3 = q0 * q3;
		q1q1 = q1 * q1;
		q1q2 = q1 * q2;
		q1q3 = q1 * q3;
		q2q2 = q2 * q2;
		q2q3 = q2 * q3;
		q3q3 = q3 * q3;
		 
		// Reference direction of Earth's magnetic field
		hx = mx * q0q0 - _2q0my * q3 + _2q0mz * q2 + mx * q1q1 + _2q1 * my * q2 + _2q1 * mz * q3 - mx * q2q2 - mx * q3q3;
		hy = _2q0mx * q3 + my * q0q0 - _2q0mz * q1 + _2q1mx * q2 - my * q1q1 + my * q2q2 + _2q2 * mz * q3 - my * q3q3;
		_2bx = sqrt(hx * hx + hy * hy);
		_2bz = -_2q0mx * q2 + _2q0my * q1 + mz * q0q0 + _2q1mx * q3 - mz * q1q1 + _2q2 * my * q3 - mz * q2q2 + mz * q3q3;
		_4bx = 2.0f * _2bx;
		_4bz = 2.0f * _2bz;
		 
		// Gradient decent algorithm corrective step
		s0 = -_2q2 * (2.0f * q1q3 - _2q0q2 - ax) + _2q1 * (2.0f * q0q1 + _2q2q3 - ay) - _2bz * q2 * (_2bx * (0.5f - q2q2 - q3q3) + _2bz * (q1q3 - q0q2) - mx) + (-_2bx * q3 + _2bz * q1) * (_2bx * (q1q2 - q0q3) + _2bz * (q0q1 + q2q3) - my) + _2bx * q2 * (_2bx * (q0q2 + q1q3) + _2bz * (0.5f - q1q1 - q2q2) - mz);
		s1 = _2q3 * (2.0f * q1q3 - _2q0q2 - ax) + _2q0 * (2.0f * q0q1 + _2q2q3 - ay) - 4.0f * q1 * (1 - 2.0f * q1q1 - 2.0f * q2q2 - az) + _2bz * q3 * (_2bx * (0.5f - q2q2 - q3q3) + _2bz * (q1q3 - q0q2) - mx) + (_2bx * q2 + _2bz * q0) * (_2bx * (q1q2 - q0q3) + _2bz * (q0q1 + q2q3) - my) + (_2bx * q3 - _4bz * q1) * (_2bx * (q0q2 + q1q3) + _2bz * (0.5f - q1q1 - q2q2) - mz);
		s2 = -_2q0 * (2.0f * q1q3 - _2q0q2 - ax) + _2q3 * (2.0f * q0q1 + _2q2q3 - ay) - 4.0f * q2 * (1 - 2.0f * q1q1 - 2.0f * q2q2 - az) + (-_4bx * q2 - _2bz * q0) * (_2bx * (0.5f - q2q2 - q3q3) + _2bz * (q1q3 - q0q2) - mx) + (_2bx * q1 + _2bz * q3) * (_2bx * (q1q2 - q0q3) + _2bz * (q0q1 + q2q3) - my) + (_2bx * q0 - _4bz * q2) * (_2bx * (q0q2 + q1q3) + _2bz * (0.5f - q1q1 - q2q2) - mz);
		s3 = _2q1 * (2.0f * q1q3 - _2q0q2 - ax) + _2q2 * (2.0f * q0q1 + _2q2q3 - ay) + (-_4bx * q3 + _2bz * q1) * (_2bx * (0.5f - q2q2 - q3q3) + _2bz * (q1q3 - q0q2) - mx) + (-_2bx * q0 + _2bz * q2) * (_2bx * (q1q2 - q0q3) + _2bz * (q0q1 + q2q3) - my) + _2bx * q1 * (_2bx * (q0q2 + q1q3) + _2bz * (0.5f - q1q1 - q2q2) - mz);
		recipNorm = invSqrt(s0 * s0 + s1 * s1 + s2 * s2 + s3 * s3); // normalise step magnitude
		s0 *= recipNorm;
		s1 *= recipNorm;
		s2 *= recipNorm;
		s3 *= recipNorm;
		 
		// Apply feedback step
		qDot1 -= beta * s0;
		qDot2 -= beta * s1;
		qDot3 -= beta * s2;
		qDot4 -= beta * s3;
	 
	}
	 
	// Integrate rate of change of quaternion to yield quaternion
//	q0 += qDot1 * (1.0f / sampleFreq);
//	q1 += qDot2 * (1.0f / sampleFreq);
//	q2 += qDot3 * (1.0f / sampleFreq);
//	q3 += qDot4 * (1.0f / sampleFreq);
	q0 += qDot1 * (deltat);
	q1 += qDot2 * (deltat);
	q2 += qDot3 * (deltat);
	q3 += qDot4 * (deltat);
	
	// Normalise quaternion
	recipNorm = invSqrt(q0 * q0 + q1 * q1 + q2 * q2 + q3 * q3);
	q0 *= recipNorm;
	q1 *= recipNorm;
	q2 *= recipNorm;
	q3 *= recipNorm;
 
//	madgwick_ahrs.pitch_rad = asin(-2 * q1 * q3 + 2 * q0* q2);	// pitch
//	madgwick_ahrs.roll_rad= atan2(2 * q2 * q3 + 2 * q0 * q1, -2 * q1 * q1 - 2 * q2* q2 + 1);
//	madgwick_ahrs.yaw_rad= atan2(2*(q1*q2 + q0*q3),q0*q0+q1*q1-q2*q2-q3*q3);
}

//---------------------------------------------------------------------------------------------------
// IMU algorithm update
void AHRS_Update6(float gx, float gy, float gz, float ax, float ay, float az,float beta,float deltat) 
{
 
	float recipNorm;
	float s0, s1, s2, s3;
	float qDot1, qDot2, qDot3, qDot4;
	float _2q0, _2q1, _2q2, _2q3, _4q0, _4q1, _4q2 ,_8q1, _8q2, q0q0, q1q1, q2q2, q3q3;
	gx=AHRS_D2R(gx);
	gy=AHRS_D2R(gy);
	gz=AHRS_D2R(gz);
	_gz_rad=gz;
	// Rate of change of quaternion from gyroscope
	qDot1 = 0.5f * (-q1 * gx - q2 * gy - q3 * gz);
	qDot2 = 0.5f * (q0 * gx + q2 * gz - q3 * gy);
	qDot3 = 0.5f * (q0 * gy - q1 * gz + q3 * gx);
	qDot4 = 0.5f * (q0 * gz + q1 * gy - q2 * gx);
	 
	// Compute feedback only if accelerometer measurement valid (avoids NaN in accelerometer normalisation)
	if(!((ax == 0.0f) && (ay == 0.0f) && (az == 0.0f))) 
	{
		// Normalise accelerometer measurement
		recipNorm = invSqrt(ax * ax + ay * ay + az * az);
		ax *= recipNorm;
		ay *= recipNorm;
		az *= recipNorm;  
		 
		// Auxiliary variables to avoid repeated arithmetic
		_2q0 = 2.0f * q0;
		_2q1 = 2.0f * q1;
		_2q2 = 2.0f * q2;
		_2q3 = 2.0f * q3;
		_4q0 = 4.0f * q0;
		_4q1 = 4.0f * q1;
		_4q2 = 4.0f * q2;
		_8q1 = 8.0f * q1;
		_8q2 = 8.0f * q2;
		q0q0 = q0 * q0;
		q1q1 = q1 * q1;
		q2q2 = q2 * q2;
		q3q3 = q3 * q3;
		 
		// Gradient decent algorithm corrective step
		s0 = _4q0 * q2q2 + _2q2 * ax + _4q0 * q1q1 - _2q1 * ay;
		s1 = _4q1 * q3q3 - _2q3 * ax + 4.0f * q0q0 * q1 - _2q0 * ay - _4q1 + _8q1 * q1q1 + _8q1 * q2q2 + _4q1 * az;
		s2 = 4.0f * q0q0 * q2 + _2q0 * ax + _4q2 * q3q3 - _2q3 * ay - _4q2 + _8q2 * q1q1 + _8q2 * q2q2 + _4q2 * az;
		s3 = 4.0f * q1q1 * q3 - _2q1 * ax + 4.0f * q2q2 * q3 - _2q2 * ay;
		
		//the fact is non-normalise-step is better
//		recipNorm = invSqrt(s0 * s0 + s1 * s1 + s2 * s2 + s3 * s3); // normalise step magnitude
//		s0 *= recipNorm;
//		s1 *= recipNorm;
//		s2 *= recipNorm;
//		s3 *= recipNorm;
		 
		// Apply feedback step
		qDot1 -= beta * s0;
		qDot2 -= beta * s1;
		qDot3 -= beta * s2;
		qDot4 -= beta * s3;
	 
	}
	 
	// Integrate rate of change of quaternion to yield quaternion
//	q0 += qDot1 * (1.0f / sampleFreq);
//	q1 += qDot2 * (1.0f / sampleFreq);
//	q2 += qDot3 * (1.0f / sampleFreq);
//	q3 += qDot4 * (1.0f / sampleFreq);
	q0 += qDot1 * (deltat);
	q1 += qDot2 * (deltat);
	q2 += qDot3 * (deltat);
	q3 += qDot4 * (deltat);
	
	// Normalise quaternion
	recipNorm = invSqrt(q0 * q0 + q1 * q1 + q2 * q2 + q3 * q3);
	q0 *= recipNorm;
	q1 *= recipNorm;
	q2 *= recipNorm;
	q3 *= recipNorm;
	
//	madgwick_ahrs.pitch_rad = asin(-2 * q1 * q3 + 2 * q0* q2);	// pitch
//	madgwick_ahrs.roll_rad= atan2(2 * q2 * q3 + 2 * q0 * q1, -2 * q1 * q1 - 2 * q2* q2 + 1);
//	madgwick_ahrs.yaw_rad= atan2(2*(q1*q2 + q0*q3),q0*q0+q1*q1-q2*q2-q3*q3);
}
float AHRS_LimitAngle(float a)
{
	#define M_PI 3.1415926535898f
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
// IMU algorithm update
float AHRS_UpdateYaw(float angle_measure,float beta,float deltat) 
{
	float recipNorm;
	float gz_est;
	
	{
		
//		recipNorm=sin(AHRS_LimitAngle(_yaw_rad-angle_measure));
		recipNorm=AHRS_LimitAngle(_yaw_rad-angle_measure);
		
//		if(recipNorm>0)
//		{
//			recipNorm=1;
//		}
//		else
//		{
//			recipNorm=-1;
//		}
	}
	gz_est=_gz_rad-beta*recipNorm;
	_yaw_rad=AHRS_LimitAngle(_yaw_rad+gz_est*deltat);
	return _yaw_rad;
}	 
void AHRS_GetAngle(float *yaw_rad,float *pitch_rad,float *roll_rad)
{
//	float pitch1_rad,pitch2_rad;
//	#define AHRS_PI    ((float)3.1415926535898f)
//	
//	pitch1_rad= asin(-2 * q1 * q3 + 2 * q0* q2);	// pitch
//	
//	if(fabs(*pitch_rad-pitch_rad_last)>
	
	*pitch_rad= asin(-2 * q1 * q3 + 2 * q0* q2);	// pitch
	*roll_rad = atan2(2 * q2 * q3 + 2 * q0 * q1, -2 * q1 * q1 - 2 * q2* q2 + 1);//roll
	*yaw_rad  = atan2(2*(q1*q2 + q0*q3),q0*q0+q1*q1-q2*q2-q3*q3);//yaw
	
}
void AHRS_SetQFromAngle(float yaw_rad,float pitch_rad,float roll_rad)
{
	float c1,s1,c2,s2,c3,s3;
	c1=cos(roll_rad/2.f);
	s1=sin(roll_rad/2.f);
	c2=cos(pitch_rad/2.f);
	s2=sin(pitch_rad/2.f);
	c3=cos(yaw_rad/2.f);
	s3=sin(yaw_rad/2.f);
//	q0=c1*c2*c3-s1*s2*s3;
//	q1=c1*s2*s3+s1*c2*c3;
//	q2=c1*s2*c3-s1*c2*s3;
//	q3=c1*c2*s3+s1*s2*c3;
	q0=c1*c2*c3+s1*s2*s3;
	q1=s1*c2*c3-c1*s2*s3;
	q2=c1*s2*c3+s1*c2*s3;
	q3=c1*c2*s3-s1*s2*c3;
	
	_yaw_rad=yaw_rad;
}
float AHRS_D2R(float degree)
{
	return degree*0.017453292519943f;
}
float AHRS_R2D(float rad)
{
	return rad*57.295779513082f;
}
//---------------------------------------------------------------------------------------------------
// Fast inverse square-root
// See: http://en.wikipedia.org/wiki/Fast_inverse_square_root
float invSqrt(float x) 
{
	float halfx = 0.5f * x;
	float y = x;
	long i = *(long*)&y;
	i = 0x5f3759df - (i>>1);
	y = *(float*)&i;
	y = y * (1.5f - (halfx * y * y));
	return y;
}
 
//====================================================================================================
// END OF CODE
//====================================================================================================












