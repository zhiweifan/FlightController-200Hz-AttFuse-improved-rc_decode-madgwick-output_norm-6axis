


#include "pid.h"

#define abs(x) ((x)>0? (x):(-(x)))
//#define INF_SMALL   (1e-20f)


void PID_DeInit(PID_TypeDef *pid,float out_abs_max,float proportion_limit_coef,float integration_limit_coef,float dev_limit_coef,float ddev_limit_coef)
{
	pid->Ref=0;
	pid->RefTc=0;
	pid->RefFilter=0;
	pid->Measure=0;
	pid->MeasureFilter=0;
	pid->MeasureTc=0.01;
	pid->Err[0]=0;
	pid->Err[1]=0;
	pid->Err[2]=0;
	pid->ErrDev[0]=0;
	pid->ErrDev[1]=0;
	pid->ErrDevTc=0.1;
	pid->ErrDDev=0;
	pid->ErrDDevTc=0.1;
	pid->Kp=0;
	pid->Ki=0;
	pid->Kd=0;
	pid->Kdd=0;
	pid->P=0;
	pid->I=0;
	pid->D=0;
	pid->DD=0;
	pid->OutputMax=out_abs_max;
	pid->PropLimit=0;
	pid->IntLimit=0;
	pid->DevLimit=0;
	pid->DDevLimit=0;
	if(proportion_limit_coef<0||proportion_limit_coef>2)
	{
		pid->PropLimit=out_abs_max*DEFAULT_PROP_LIMIT_COEF;
	}
	else
	{
		pid->PropLimit=out_abs_max*proportion_limit_coef;
	}
	if(integration_limit_coef<0||integration_limit_coef>1)
	{
		pid->IntLimit=out_abs_max*DEFAULT_INT_LIMIT_COEF;
	}
	else
	{
		pid->IntLimit=out_abs_max*integration_limit_coef;
	}
	if(dev_limit_coef<0||dev_limit_coef>2)
	{
		pid->DevLimit=out_abs_max*DEFAULT_DEV_LIMIT_COEF;
	}
	else
	{
		pid->DevLimit=out_abs_max*dev_limit_coef;
	}
	if(ddev_limit_coef<0||ddev_limit_coef>2)
	{
		pid->DDevLimit=out_abs_max*DEFAULT_DDEV_LIMIT_COEF;
	}
	else
	{
		pid->DDevLimit=out_abs_max*ddev_limit_coef;
	}
	pid->Output=0;
}
//int sign(float x)
//{
//	return x>0?1:-1;
//}
float PID_Limit(float x,float max)
{
	#define LIMIT_MIN_MAX(x,min,max)				(((x)<(min))?(min):(((x)>(max))?(max):(x)))
	return LIMIT_MIN_MAX(x,-max,max);
}
void PID_Calculate(PID_TypeDef *pid,char use_speed,float speed,char use_error,float error,float err_max,float err_dev_max)
{
	
	pid->RefFilter=(pid->RefTc*pid->RefFilter+pid->Ref)/(1.0f+pid->RefTc);
	
	pid->Err[2]=pid->Err[1];
	pid->Err[1]=pid->Err[0];
	if(use_error)
	{
		if(err_max>0)
		{
			error=PID_Limit(error,err_max);
		}
		
		pid->MeasureFilter=(pid->MeasureTc*pid->MeasureFilter+error)/(1.0f+pid->MeasureTc);
		
	}
	else
	{
		
		pid->MeasureFilter=(pid->MeasureTc*pid->MeasureFilter+pid->Measure)/(1.0f+pid->MeasureTc);
		if(err_max>0)
		{
			pid->Err[0]=PID_Limit(pid->RefFilter-pid->MeasureFilter,err_max);
		}
		else
		{
			pid->Err[0]=pid->RefFilter-pid->MeasureFilter;
		}
	}
	
	pid->ErrDev[1]=pid->ErrDev[0];
	if(use_speed)
	{
		if(err_dev_max>0)
		{
			pid->ErrDev[0]=(pid->ErrDevTc*pid->ErrDev[0]+PID_Limit((-speed),err_dev_max))/(1.0f+pid->ErrDevTc);
		}
		else
		{
			pid->ErrDev[0]=(pid->ErrDevTc*pid->ErrDev[0]+(-speed))/(1.0f+pid->ErrDevTc);
		}
		
	}
	else
	{
		pid->ErrDev[0]=(pid->ErrDevTc*pid->ErrDev[0]+PID_Limit((pid->Err[0]-pid->Err[1]),err_dev_max))/(1.0f+pid->ErrDevTc);
	}
	pid->ErrDDev=(pid->ErrDDevTc*pid->ErrDDev+(pid->ErrDev[0]-pid->ErrDev[1]))/(1.0f+pid->ErrDDevTc);
	
	
	pid->P=pid->Kp*pid->Err[0];
	if(pid->PropLimit>0&&abs(pid->P)>pid->PropLimit)
	{
		if(pid->P>0)
		{
			pid->P=pid->PropLimit;
		}
		else
		{
			pid->P=-pid->PropLimit;
		}
	}
	if(pid->Ki!=0)
	{
		pid->I=pid->I+pid->Ki*pid->Err[0];
		if(pid->IntLimit>0&&abs(pid->I)>pid->IntLimit)
		{
			if(pid->I>0)
			{
				pid->I=pid->IntLimit;
			}
			else
			{
				pid->I=-pid->IntLimit;
			}
		}
	}
	else
	{
		pid->I=0;
	}
	
	
	if(pid->Kd!=0)
	{
		pid->D=pid->Kd*pid->ErrDev[0];
		if(pid->DevLimit>0&&abs(pid->D)>pid->DevLimit)
		{
			if(pid->D>0)
			{
				pid->D=pid->DevLimit;
			}
			else
			{
				pid->D=-pid->DevLimit;
			}
		}
	}
	else
	{
		pid->D=0;
	}
	if(pid->Kdd!=0)
	{
		pid->DD=pid->Kdd*pid->ErrDDev;
		if(pid->DDevLimit>0&&abs(pid->DD)>pid->DDevLimit)
		{
			if(pid->DD>0)
			{
				pid->DD=pid->DDevLimit;
			}
			else
			{
				pid->DD=-pid->DDevLimit;
			}
		}
	}
	else
	{
		pid->DD=0;
	}
	
	pid->Output=pid->P+pid->I+pid->D+pid->DD;
}

float DataFilter(float measure,float* pFIFO,int n)
{
	float sum=0;
	int i=0;
	for(i=0;i<n;i++)
	{
		sum+=pFIFO[i];
	}
	sum=(sum+measure)/(n+1);
	for(i=0;i<n-1;i++)
	{
		pFIFO[i]=pFIFO[i+1];
	}
	pFIFO[n-1]=measure;
	return sum;
}







