
#include "math_lib.h"
#define TRANSP(n,type)									((type *)(&(n)))
#define TRANS(n,type)										(*((type *)(&(n))))
#define ABS(x)													(((x)>0)?(x):(-(x)))
#define SIGN(x)													(((x)>0)?(1):(-1))
#define LIMIT(x,y)											(((x)>(y))?(y):(((x)<(-(y)))?(-(y)):(x)))
#define LIMIT_UP(x,y)										(((x)>(y))?(y):(x))
#define LIMIT_LOW(x,y)									(((x)<(y))?(y):(x))
#define LIMIT_MIN_MAX(x,min,max)				(((x)<(min))?(min):(((x)>(max))?(max):(x)))

float AddDead(float x,float x_min,float x_max,float y_min,float y_max,float dead,float sign)
{
	float y,k,x_mean,y_mean;
	x_mean=(x_min+x_max)/2.0f;
	y_mean=(y_min+y_max)/2.0f;
	if((x-x_mean)<dead&&(x-x_mean)>-dead)
	{
		return y_mean;
	}
	else
	{
		k=(y_max-y_min)/(x_max-x_min-dead-dead);
		if(x>x_max)
		{
			x=x_max;
		}
		else if(x<x_min)
		{
			x=x_min;
		}
		if((x-x_mean)>dead)
		{
			x=x-dead;
		}
		else
		{
			x=x+dead;
		}
		y=y_mean+k*(x-x_mean)*sign;
		return y;
	}
}
float GetDeltaAngle(float delta)
{
	if(ABS(delta)>180.0f)
	{
		if(delta>0)
		{
			delta=delta-360;
		}
		else
		{
			delta=delta+360;
		}
	}
	
	if(ABS(delta)>180.0f)
	{
		if(delta>0)
		{
			delta=delta-360;
		}
		else
		{
			delta=delta+360;
		}
	}
	
	return delta;
}
float LimitAngle(float a)
{
	if(a>180)
	{
		a=a-360;
	}
	else if(a<-180)
	{
		a=a+360;
	}
	return a;
}
int sign(float x)
{
	return x>0?1:-1;
}
int LimitMinMax_int(int x,int min,int max)
{
	return LIMIT_MIN_MAX(x,min,max);
}
float LimitMinMax_float(float x,float min,float max)
{
	return LIMIT_MIN_MAX(x,min,max);
}
float ABS_float(float x)
{
	return ABS(x);
}









