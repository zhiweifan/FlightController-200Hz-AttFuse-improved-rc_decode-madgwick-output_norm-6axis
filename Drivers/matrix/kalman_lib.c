

#include "kalman_lib.h"
#include "matrix.h"
//#include "stdlib.h"



char StatusPrediction(KALMAN_PARAMS_TypeDef* pkalman,float *x);
char VariancePrediction(KALMAN_PARAMS_TypeDef* pkalman,float *p);
char CalculateGain(KALMAN_PARAMS_TypeDef* pkalman,float *p,float *gain);
char StatusEstimation(KALMAN_PARAMS_TypeDef* pkalman,float *x,float *gain);
char VarianceUpdate(KALMAN_PARAMS_TypeDef* pkalman,float *p,float *gain);

//minimum heap size needed is (3*x_size*x_size+x_size*(1+z_size))*4,x_size>=2
char KALMAN_Update(KALMAN_PARAMS_TypeDef* pkalman)
{
	float *x_tmp,*p_tmp,*gain;
	x_tmp=(float *)malloc(pkalman->x_size*sizeof(float));
	p_tmp=(float *)malloc(pkalman->x_size*pkalman->x_size*sizeof(float));
	gain=(float *)malloc(pkalman->x_size*pkalman->z_size*sizeof(float));
	
	if(x_tmp==0||p_tmp==0||gain==0)
	{
		free(gain);
		free(p_tmp);
		free(x_tmp);
		return 0;
	}
	if(!StatusPrediction(pkalman,x_tmp))
	{
		free(gain);
		free(p_tmp);
		free(x_tmp);
		return 0;
	}
	if(!VariancePrediction(pkalman,p_tmp))
	{
		free(gain);
		free(p_tmp);
		free(x_tmp);
		return 0;
	}
	if(!CalculateGain(pkalman,p_tmp,gain))
	{
		free(gain);
		free(p_tmp);
		free(x_tmp);
		return 0;
	}
	if(!StatusEstimation(pkalman,x_tmp,gain))
	{
		free(gain);
		free(p_tmp);
		free(x_tmp);
		return 0;
	}
	if(!VarianceUpdate(pkalman,p_tmp,gain))
	{
		free(gain);
		free(p_tmp);
		free(x_tmp);
		return 0;
	}
	
	free(gain);
	free(p_tmp);
	free(x_tmp);

	return 1;
}

char StatusPrediction(KALMAN_PARAMS_TypeDef* pkalman,float *x)
{
	MATRIX_Mul(x,pkalman->A,pkalman->x,pkalman->x_size,pkalman->x_size,1);
	return 1;
}
char VariancePrediction(KALMAN_PARAMS_TypeDef* pkalman,float *p)
{
	MATRIX_A_P_AT(p,pkalman->A,pkalman->P,pkalman->x_size,pkalman->x_size);
	MATRIX_Add(p,pkalman->Q,pkalman->x_size,pkalman->x_size);
	return 1;
}
char CalculateGain(KALMAN_PARAMS_TypeDef* pkalman,float *p,float *gain)
{
	#define ABS(x)  ((x)>0?(x):-(x))
	float *tmp,*tmp2;
	float res;
	tmp=(float*)malloc(pkalman->z_size*pkalman->z_size*sizeof(float));
	tmp2=(float*)malloc(pkalman->x_size*pkalman->z_size*sizeof(float));

	if(tmp==0||tmp2==0)
	{
		free(tmp2);
		free(tmp);
		return 0;
	}
	MATRIX_A_P_AT(tmp,pkalman->C,p,pkalman->z_size,pkalman->x_size);
	MATRIX_Add(tmp,pkalman->R,pkalman->z_size,pkalman->z_size);
	res=MATRIX_Inv(tmp,pkalman->z_size);
	if(ABS(res)<(float)1e-35)
	{
		free(tmp2);
		free(tmp);
		return 0;
	}
	MATRIX_A_BT(tmp2,p,pkalman->C,pkalman->x_size,pkalman->x_size,pkalman->z_size);
	MATRIX_Mul(gain,tmp2,tmp,pkalman->x_size,pkalman->z_size,pkalman->z_size);
	free(tmp2);
	free(tmp);
	return 1;
}
char StatusEstimation(KALMAN_PARAMS_TypeDef* pkalman,float *x,float *gain)
{
	float *tmp,*tmp2,*tmp3;
	tmp=(float*)malloc(pkalman->x_size*pkalman->x_size*sizeof(float));
	tmp2=(float*)malloc(pkalman->x_size*sizeof(float));
	tmp3=(float*)malloc(pkalman->x_size*sizeof(float));
	if(tmp==0||tmp2==0||tmp3==0)
	{
		free(tmp3);
		free(tmp2);
		free(tmp);
		return 0;
	}
	MATRIX_Mul(tmp2,pkalman->C,x,pkalman->z_size,pkalman->x_size,1);
	MATRIX_Copy(tmp3,pkalman->z,pkalman->z_size,1);
	MATRIX_Sub(tmp3,tmp2,pkalman->z_size,1);
	MATRIX_Mul(tmp2,gain,tmp3,pkalman->x_size,pkalman->z_size,1);
	MATRIX_Add(tmp2,x,pkalman->x_size,1);
	MATRIX_Copy(pkalman->x,tmp2,pkalman->x_size,1);
	free(tmp3);
	free(tmp2);
	free(tmp);
	return 1;
}
char VarianceUpdate(KALMAN_PARAMS_TypeDef* pkalman,float *p,float *gain)
{
	float *tmp,*tmp2;
	int i,j;

	tmp=(float*)malloc(pkalman->x_size*pkalman->x_size*sizeof(float));
	tmp2=(float*)malloc(pkalman->x_size*pkalman->x_size*sizeof(float));
	if(tmp==0||tmp2==0)
	{
		free(tmp2);
		free(tmp);
		return 0;
	}
	MATRIX_Mul(tmp,gain,pkalman->C,pkalman->x_size,pkalman->z_size,pkalman->x_size);
	for(i=0;i<pkalman->x_size;i++)
	{
		for(j=0;j<pkalman->x_size;j++)
		{
			if(i==j)
			{
				tmp[i*pkalman->x_size+j]=1.0f-tmp[i*pkalman->x_size+j];
			}
			else
			{
				tmp[i*pkalman->x_size+j]=-tmp[i*pkalman->x_size+j];
			}
		}
	}
	MATRIX_A_P_AT(tmp2,tmp,p,pkalman->x_size,pkalman->x_size);
	MATRIX_A_P_AT(tmp,gain,pkalman->R,pkalman->x_size,pkalman->z_size);
	MATRIX_Add(tmp,tmp2,pkalman->x_size,pkalman->x_size);
	MATRIX_Copy(pkalman->P,tmp,pkalman->x_size,pkalman->x_size);
	
	free(tmp2);
	free(tmp);
	return 1;
}















