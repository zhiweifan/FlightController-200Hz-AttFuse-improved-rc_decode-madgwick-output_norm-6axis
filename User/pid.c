


#include "pid.h"

//#define abs(x) ((x)>0? (x):(-(x)))
//#define INF_SMALL   (1e-20f)
float PID_abs(float x)
{
	return ((x)>0? (x):(-(x)));
}

void PID_Reset(PID_TypeDef *pid)
{
	pid->Inputs.Error=0;
	pid->Inputs.MeasureSpeed=0;
	pid->Inputs.MeasureAcc=0;
//	pid->Inputs.Ref=0;
//	pid->Inputs.Measure=0;
	
	pid->Outputs.P=0;
	pid->Outputs.I=0;
	pid->Outputs.D=0;
	pid->Outputs.DD=0;
	pid->Outputs.Output=0;
	
	pid->Status.Err[0]=0;
	pid->Status.Err[1]=0;
	pid->Status.Err[2]=0;
	pid->Status.ErrDev[0]=0;
	pid->Status.ErrDev[1]=0;
	pid->Status.ErrDDev=0;
	pid->Status.LastMeasure=0;
	pid->Status.MeasureFilter=0;
	pid->Status.RefFilter=0;
	
}
void PID_Reset2(PID_TypeDef *pid)
{
	pid->Inputs.Error=0;
	pid->Inputs.MeasureSpeed=0;
	pid->Inputs.MeasureAcc=0;
//	pid->Inputs.Ref=0;
//	pid->Inputs.Measure=0;
	
	pid->Outputs.P=0;
	pid->Outputs.I=0;
	pid->Outputs.D=0;
	pid->Outputs.DD=0;
	pid->Outputs.Output=0;
	
	pid->Status.Err[0]=0;
	pid->Status.Err[1]=0;
	pid->Status.Err[2]=0;
	pid->Status.ErrDev[0]=0;
	pid->Status.ErrDev[1]=0;
	pid->Status.ErrDDev=0;
	pid->Status.LastMeasure=0;
	//pid->Status.MeasureFilter=0;
	pid->Status.RefFilter=0;
	
}
void PID_StructInit(PID_TypeDef *pid)
{
	pid->Inputs.Error=0;
	pid->Inputs.MeasureSpeed=0;
	pid->Inputs.MeasureAcc=0;
	pid->Inputs.Ref=0;
	pid->Inputs.Measure=0;
	
	pid->Outputs.P=0;
	pid->Outputs.I=0;
	pid->Outputs.D=0;
	pid->Outputs.DD=0;
	pid->Outputs.Output=0;
	
	pid->Status.Err[0]=0;
	pid->Status.Err[1]=0;
	pid->Status.Err[2]=0;
	pid->Status.ErrDev[0]=0;
	pid->Status.ErrDev[1]=0;
	pid->Status.ErrDDev=0;
	pid->Status.LastMeasure=0;
	pid->Status.MeasureFilter=0;
	pid->Status.RefFilter=0;
	
	pid->Params.Kp=0;
	pid->Params.Ki=0;
	pid->Params.Kd=0;
	pid->Params.Kdd=0;
	pid->Params.MeasureTc=0;
	pid->Params.RefTc=0;
	pid->Params.ErrDevTc=0;
	pid->Params.ErrDDevTc=0;
	
	pid->Limits.PropLimit=0;
	pid->Limits.IntLimit=0;
	pid->Limits.DevLimit=0;
	pid->Limits.DDevLimit=0;
	pid->Limits.ErrMax=0;
	pid->Limits.ErrDevMax=0;
	pid->Limits.ErrDDevMax=0;
	pid->Limits.OutputLimit=0;
	pid->Limits.MeasureMax=0;
	pid->Limits.RefMax=0;
	
	pid->Settings.UseError=0;
	pid->Settings.UseMeasureAcc=0;
	pid->Settings.UseMeasureSpeed=0;
	pid->Settings.CalculateMeasureSpeed=0;
}
float PID_Limit(float x,float max)
{
	#define LIMIT_MIN_MAX(x,min,max)				(((x)<(min))?(min):(((x)>(max))?(max):(x)))
	return LIMIT_MIN_MAX(x,-max,max);
}
void PID_Calculate(PID_TypeDef *pid)
{
	float tmp;
	pid->Status.Err[2]=pid->Status.Err[1];
	pid->Status.Err[1]=pid->Status.Err[0];
	if(pid->Settings.UseError)
	{
		if(pid->Limits.ErrMax>0)
		{
			tmp=PID_Limit(pid->Inputs.Error,pid->Limits.ErrMax);
		}
		else
		{
			tmp=pid->Inputs.Error;
		}
		pid->Status.Err[0]=(pid->Params.MeasureTc*pid->Status.Err[0]+tmp)/(pid->Params.MeasureTc+1.0f);
	}
	else
	{
		if(pid->Limits.RefMax>0)
		{
			tmp=PID_Limit(pid->Inputs.Ref,pid->Limits.RefMax);
		}
		else
		{
			tmp=pid->Inputs.Ref;
		}
		pid->Status.RefFilter=(pid->Params.RefTc*pid->Status.RefFilter+tmp)/(pid->Params.RefTc+1.0f);
		
		if(pid->Limits.MeasureMax>0)
		{
			tmp=PID_Limit(pid->Inputs.Measure,pid->Limits.MeasureMax);
		}
		else
		{
			tmp=pid->Inputs.Measure;
		}
		pid->Status.MeasureFilter=(pid->Params.MeasureTc*pid->Status.MeasureFilter+tmp)/(pid->Params.MeasureTc+1.0f);
		
		tmp=pid->Status.RefFilter-pid->Status.MeasureFilter;
		if(pid->Limits.ErrMax>0)
		{
			tmp=PID_Limit(tmp,pid->Limits.ErrMax);
		}
		pid->Status.Err[0]=tmp;
		
	}
	
	pid->Status.ErrDev[1]=pid->Status.ErrDev[0];
	if(pid->Settings.UseMeasureSpeed)
	{
		if(pid->Limits.ErrDevMax>0)
		{
			tmp=PID_Limit(-pid->Inputs.MeasureSpeed,pid->Limits.ErrDevMax);
		}
		else
		{
			tmp=-pid->Inputs.MeasureSpeed;
		}
		pid->Status.ErrDev[0]=(pid->Params.ErrDevTc*pid->Status.ErrDev[0]+tmp)/(pid->Params.ErrDevTc+1.0f);
	}
	else
	{
		if(pid->Limits.ErrDevMax>0)
		{
			if(pid->Settings.CalculateMeasureSpeed)
			{
				tmp=PID_Limit(pid->Status.LastMeasure-pid->Inputs.Measure,pid->Limits.ErrDevMax);
			}
			else
			{
				tmp=PID_Limit(pid->Status.Err[0]-pid->Status.Err[1],pid->Limits.ErrDevMax);
			}
			
		}
		else
		{
			if(pid->Settings.CalculateMeasureSpeed)
			{
				tmp=pid->Status.LastMeasure-pid->Inputs.Measure;
			}
			else
			{
				tmp=pid->Status.Err[0]-pid->Status.Err[1];
			}
		}
		pid->Status.ErrDev[0]=(pid->Params.ErrDevTc*pid->Status.ErrDev[0]+tmp)/(pid->Params.ErrDevTc+1.0f);
	}
	
	if(pid->Params.Kdd!=0)
	{
		if(pid->Settings.UseMeasureAcc)
		{
			if(pid->Limits.ErrDDevMax>0)
			{
				tmp=PID_Limit(-pid->Inputs.MeasureAcc,pid->Limits.ErrDDevMax);
			}
			else
			{
				tmp=-pid->Inputs.MeasureAcc;
			}
			pid->Status.ErrDDev=(pid->Params.ErrDDevTc*pid->Status.ErrDDev+tmp)/(pid->Params.ErrDDevTc+1.0f);
		}
		else
		{
			if(pid->Limits.ErrDDevMax>0)
			{
				tmp=PID_Limit(pid->Status.ErrDev[0]-pid->Status.ErrDev[1],pid->Limits.ErrDDevMax);
			}
			else
			{
				tmp=pid->Status.ErrDev[0]-pid->Status.ErrDev[1];
			}
			pid->Status.ErrDDev=(pid->Params.ErrDDevTc*pid->Status.ErrDDev+tmp)/(pid->Params.ErrDDevTc+1.0f);
		}
		
		pid->Outputs.DD=pid->Params.Kdd*pid->Status.ErrDDev;
		if(pid->Limits.DDevLimit>0&&PID_abs(pid->Outputs.DD)>pid->Limits.DDevLimit)
		{
			if(pid->Outputs.DD>0)
			{
				pid->Outputs.DD=pid->Limits.DDevLimit;
			}
			else
			{
				pid->Outputs.DD=-pid->Limits.DDevLimit;
			}
		}
	}
	else
	{
		pid->Outputs.DD=0;
	}
	
	if(pid->Params.Kp!=0)
	{
		pid->Outputs.P=pid->Params.Kp*pid->Status.Err[0];
		if(pid->Limits.PropLimit>0&&PID_abs(pid->Outputs.P)>pid->Limits.PropLimit)
		{
			if(pid->Outputs.P>0)
			{
				pid->Outputs.P=pid->Limits.PropLimit;
			}
			else
			{
				pid->Outputs.P=-pid->Limits.PropLimit;
			}
		}
	}
	else
	{
		pid->Outputs.P=0;
	}
	
	if(pid->Params.Ki!=0)
	{
		pid->Outputs.I=pid->Outputs.I+pid->Params.Ki*pid->Status.Err[0];
		if(pid->Limits.IntLimit>0&&PID_abs(pid->Outputs.I)>pid->Limits.IntLimit)
		{
			if(pid->Outputs.I>0)
			{
				pid->Outputs.I=pid->Limits.IntLimit;
			}
			else
			{
				pid->Outputs.I=-pid->Limits.IntLimit;
			}
		}
	}
	else
	{
		pid->Outputs.I=0;
	}
	
	if(pid->Params.Kd!=0)
	{
		pid->Outputs.D=pid->Params.Kd*pid->Status.ErrDev[0];
		if(pid->Limits.DevLimit>0&&PID_abs(pid->Outputs.D)>pid->Limits.DevLimit)
		{
			if(pid->Outputs.D>0)
			{
				pid->Outputs.D=pid->Limits.DevLimit;
			}
			else
			{
				pid->Outputs.D=-pid->Limits.DevLimit;
			}
		}
	}
	else
	{
		pid->Outputs.D=0;
	}
	
	
	pid->Outputs.Output=pid->Outputs.P+pid->Outputs.I+pid->Outputs.D+pid->Outputs.DD;
	if(pid->Limits.OutputLimit>0&&PID_abs(pid->Outputs.Output)>pid->Limits.OutputLimit)
	{
		if(pid->Outputs.Output>0)
		{
			pid->Outputs.Output=pid->Limits.OutputLimit;
		}
		else
		{
			pid->Outputs.Output=-pid->Limits.OutputLimit;
		}
	}
		
	pid->Status.LastMeasure=pid->Inputs.Measure;
	
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







