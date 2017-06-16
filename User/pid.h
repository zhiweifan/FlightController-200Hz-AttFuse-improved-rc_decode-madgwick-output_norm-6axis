

#ifndef PID_H
#define PID_H

#define DEFAULT_PROP_LIMIT_COEF				2.0f
#define DEFAULT_INT_LIMIT_COEF				0.6f
#define DEFAULT_DEV_LIMIT_COEF				2.0f
#define DEFAULT_DDEV_LIMIT_COEF				1.f

typedef __packed struct
{
////////////////////////parameters///////////////////////////
volatile float RefTc;								//the time constant of reference input
volatile float MeasureTc;
volatile float ErrDevTc;						//the time constant of measure derivative
volatile float ErrDDevTc;
volatile float Kp;
volatile float Ki;
volatile float Kd;
volatile float Kdd;
}PID_PARAMS_TypeDef;
typedef struct
{
////////////////////////Limits//////////////////////////////
volatile float PropLimit;//proportion
volatile float IntLimit;//integral
volatile float DevLimit;//derivative
volatile float DDevLimit;
volatile float OutputLimit;
	
volatile float ErrMax;
volatile float ErrDevMax;
volatile float ErrDDevMax;
volatile float MeasureMax;
volatile float RefMax;
}PID_LIMITS_TypeDef;
typedef struct
{
////////////////////////Inputs////////////////////////////////
volatile float Ref;									//reference input
volatile float Measure;							
volatile float Error;
volatile float MeasureSpeed;
volatile float MeasureAcc;	
}PID_INPUTS_TypeDef;
typedef struct
{
///////////////////////Status/////////////////////////////////
volatile float RefFilter;	
volatile float MeasureFilter;
volatile float LastMeasure;						
volatile float Err[3];							//Err[0]is the current error
volatile float ErrDev[2];						//measure derivative
volatile float ErrDDev;	
}PID_STATUS_TypeDef;
typedef struct
{
/////////////////////Outputs/////////////////////////////
volatile float P;
volatile float I;
volatile float D;
volatile float DD;
volatile float Output;	
}PID_OUTPUTS_TypeDef;
typedef struct
{
/////////////////////Settings/////////////////////////////
volatile char  UseError;
volatile char  UseMeasureSpeed;
volatile char  UseMeasureAcc;
volatile char  CalculateMeasureSpeed;
//volatile char  IsMeasurePeriodic;	
}PID_SETTINGS_TypeDef;

typedef struct
{
PID_PARAMS_TypeDef   Params;
PID_LIMITS_TypeDef   Limits;
PID_INPUTS_TypeDef   Inputs;
PID_STATUS_TypeDef   Status;
PID_OUTPUTS_TypeDef  Outputs;
PID_SETTINGS_TypeDef Settings;
}PID_TypeDef;

//void PID_DeInit(PID_TypeDef *pid,float out_abs_max,float proportion_limit_coef,float integration_limit_coef,float dev_limit_coef,float ddev_limit_coef);

void PID_StructInit(PID_TypeDef *pid);
void PID_Reset(PID_TypeDef *pid);
void PID_Reset2(PID_TypeDef *pid);
void PID_Calculate(PID_TypeDef *pid);
float DataFilter(float measure,float* pFIFO,int n);






#endif





