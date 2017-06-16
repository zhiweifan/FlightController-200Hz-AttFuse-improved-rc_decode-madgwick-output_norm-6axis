

#include "ellipsoid.h"
#include "matrix.h"
#include "math.h"
#define POINTS_MAX    300
//columns:x^2,y^2,z^2,2*x*y,2*x*z,2*y*z,2*x,2*y,2*z
float points[POINTS_MAX][9];
//all is -1
float res[POINTS_MAX];
int points_recv;
char NoRot;
float sx,sy,sz,mx,my,mz;
void ELLIPSOID_Norm(void);//data normalization
char ELLIPSOID_AntiNorm(float V[10],float* Ae,float* x0,float* coef);//ellipsoid anti-normalization
float ABS(float a)
{
	return a>0?a:-a;
}
void ELLIPSOID_Reset(char no_rot)
{
	int i,j;
	for(i=0;i<POINTS_MAX;i++)
	{
		for(j=0;j<9;j++)
		{
			points[i][j]=0;
		}
	}
	for(i=0;i<POINTS_MAX;i++)
	{
		res[i]=-1;
	}
	points_recv=0;
	NoRot=no_rot;
}
void ELLIPSOID_AddPoint(float x,float y,float z,int index)
{
	float* pPoints;
	pPoints=(float*)&points[0][0];
	pPoints=pPoints+POINTS_MAX*6;
	if(index>=POINTS_MAX||index<0)
	{
		return ;
	}
	if(NoRot)
	{
		pPoints[index*3+0]=x;
		pPoints[index*3+1]=y;
		pPoints[index*3+2]=z;
	}
	else
	{
		points[index][6]=x;
		points[index][7]=y;
		points[index][8]=z;
	}
	if(index+1>points_recv)
	{
		points_recv=index+1;
	}
}
void ELLIPSOID_Norm(void)//data normalization
{
	int m,POINTS;
	float* pPoints,*pPoints2;
	float mean[3]={0,0,0},max[3]={-1e30,-1e30,-1e30},min[3]={1e30,1e30,1e30};
	POINTS=points_recv;
	
	
	if(NoRot)
	{
		pPoints=(float*)&points[0][0];
		pPoints2=pPoints+POINTS_MAX*6;
		for(m=0;m<POINTS;m++)
		{
			mean[0]+=pPoints2[m*3+0];
			mean[1]+=pPoints2[m*3+1];
			mean[2]+=pPoints2[m*3+2];
			if(pPoints2[m*3+0]>max[0])
			{
				max[0]=pPoints2[m*3+0];
			}
			if(pPoints2[m*3+0]<min[0])
			{
				min[0]=pPoints2[m*3+0];
			}
			
			if(pPoints2[m*3+1]>max[1])
			{
				max[1]=pPoints2[m*3+1];
			}
			if(pPoints2[m*3+1]<min[1])
			{
				min[1]=pPoints2[m*3+1];
			}
			
			if(pPoints2[m*3+2]>max[2])
			{
				max[2]=pPoints2[m*3+2];
			}
			if(pPoints2[m*3+2]<min[2])
			{
				min[2]=pPoints2[m*3+2];
			}
			
			
		}
		mean[0]=mean[0]/(float)POINTS;
		mean[1]=mean[1]/(float)POINTS;
		mean[2]=mean[2]/(float)POINTS;
		mx=mean[0];
		my=mean[1];
		mz=mean[2];
		sx=(max[0]-min[0])/2.f;
		sy=(max[1]-min[1])/2.f;
		sz=(max[2]-min[2])/2.f;
		for(m=0;m<POINTS;m++)
		{
			pPoints2[m*3+0]=(pPoints2[m*3+0]-mx)/sx;
			pPoints2[m*3+1]=(pPoints2[m*3+1]-my)/sy;
			pPoints2[m*3+2]=(pPoints2[m*3+2]-mz)/sz;
		}
		
	}
	else
	{
		for(m=0;m<POINTS;m++)
		{
			mean[0]+=points[m][6];
			mean[1]+=points[m][7];
			mean[2]+=points[m][8];
			if(points[m][6]>max[0])
			{
				max[0]=points[m][6];
			}
			if(points[m][6]<min[0])
			{
				min[0]=points[m][6];
			}
			
			if(points[m][7]>max[1])
			{
				max[1]=points[m][7];
			}
			if(points[m][7]<min[1])
			{
				min[1]=points[m][7];
			}
			
			if(points[m][8]>max[2])
			{
				max[2]=points[m][8];
			}
			if(points[m][8]<min[2])
			{
				min[2]=points[m][8];
			}
			
			
		}
		mean[0]=mean[0]/(float)POINTS;
		mean[1]=mean[1]/(float)POINTS;
		mean[2]=mean[2]/(float)POINTS;
		mx=mean[0];
		my=mean[1];
		mz=mean[2];
		sx=(max[0]-min[0])/2.f;
		sy=(max[1]-min[1])/2.f;
		sz=(max[2]-min[2])/2.f;
		for(m=0;m<POINTS;m++)
		{
			points[m][6]=(points[m][6]-mx)/sx;
			points[m][7]=(points[m][7]-my)/sy;
			points[m][8]=(points[m][8]-mz)/sz;
		}
	}
}
char ELLIPSOID_Fitting_no_rot(float* Ae,float* x0,float* coef)
{
	int m,POINTS;
	float tmp[10];
	float* pPoints,*pPoints2;
	pPoints=(float*)&points[0][0];
	pPoints2=pPoints+POINTS_MAX*6;
	
	POINTS=points_recv;
	if(POINTS<6)
	{
		return 0;
	}
	for(m=0;m<POINTS;m++)
	{
		pPoints[m*6+0]=pPoints2[m*3+0]*pPoints2[m*3+0];
		pPoints[m*6+1]=pPoints2[m*3+1]*pPoints2[m*3+1];
		pPoints[m*6+2]=pPoints2[m*3+2]*pPoints2[m*3+2];
		
		pPoints[m*6+3]=pPoints2[m*3+0]*2.f;
		pPoints[m*6+4]=pPoints2[m*3+1]*2.f;
		pPoints[m*6+5]=pPoints2[m*3+2]*2.f;
	}
	if(!MATRIX_LeastSquareSolution((float*)pPoints,(float*)res,(float*)tmp,POINTS,6))
	{
		return 0;
	}
	tmp[6]=tmp[3];
	tmp[7]=tmp[4];
	tmp[8]=tmp[5];
	tmp[3]=0;
	tmp[4]=0;
	tmp[5]=0;
	tmp[9]=1;
	return ELLIPSOID_AntiNorm(tmp,Ae,x0,coef);
}
char ELLIPSOID_Fitting_rot(float* Ae,float* x0,float* coef)
{
	int m,POINTS;
	float tmp[10];
	POINTS=points_recv;
	if(POINTS<9)
	{
		return 0;
	}
	for(m=0;m<POINTS;m++)
	{
		points[m][0]=points[m][6]*points[m][6];
		points[m][1]=points[m][7]*points[m][7];
		points[m][2]=points[m][8]*points[m][8];
		
		points[m][3]=points[m][6]*points[m][7]*2.f;
		points[m][4]=points[m][6]*points[m][8]*2.f;
		points[m][5]=points[m][7]*points[m][8]*2.f;
		
		points[m][6]=points[m][6]*2.f;
		points[m][7]=points[m][7]*2.f;
		points[m][8]=points[m][8]*2.f;
	}
	if(!MATRIX_LeastSquareSolution((float*)points,(float*)res,(float*)tmp,POINTS,9))
	{
		return 0;
	}
	tmp[9]=1;
	return ELLIPSOID_AntiNorm(tmp,Ae,x0,coef);
}
char ELLIPSOID_Fitting(float* Ae,float* x0,float* coef)//return 1 if succeed
{
	if(points_recv<6)
	{
		return 0;
	}
	ELLIPSOID_Norm();
	if(NoRot)
	{
		return ELLIPSOID_Fitting_no_rot(Ae,x0,coef);
	}
	else
	{
		return ELLIPSOID_Fitting_rot(Ae,x0,coef);
	}
}

char ELLIPSOID_AntiNorm(float V[10],float* Ae,float* x0,float* coef)
{
	float a,b,c,d,e,f,p,q,r,g;
	float I1,I2,I3,I4;
	float mat[16];
	float VV[10];
	float A[9],B[3],tmp;
	float P[3][3],D[3][3];
	float X0[3];
	int i;
	a=V[0];
	b=V[1];
	c=V[2];
	d=V[3];
	e=V[4];
	f=V[5];
	p=V[6];
	q=V[7];
	r=V[8];
	g=V[9];
	I1 = a + b + c;
	I2 = a*b + b*c + a*c -d*d - e*e - f*f;
	mat[0]=a;
	mat[1]=d;
	mat[2]=e;
	mat[3]=d;
	mat[4]=b;
	mat[5]=f;
	mat[6]=e;
	mat[7]=f;
	mat[8]=c;
	//	I3 = det([a d e; d b f; e f c]);
	I3 =MATRIX_Det(mat,3);

	mat[0]=a;
	mat[1]=d;
	mat[2]=e;
	mat[3]=p;
	mat[4]=d;
	mat[5]=b;
	mat[6]=f;
	mat[7]=q;
	mat[8]=e;
	mat[9]=f;
	mat[10]=c;
	mat[11]=r;
	mat[12]=p;
	mat[13]=q;
	mat[14]=r;
	mat[15]=g;
	//	I4 = det([a d e p; d b f q; e f c r; p q r g]);
	I4 =MATRIX_Det(mat,4);
	if((I1 == 0)||(I2 <=0 )||(I1*I3<=0)||(I4>=0))
	{
		return 0;
	}
	V[3]=V[3]*2.f;
	V[4]=V[4]*2.f;
	V[5]=V[5]*2.f;
	V[6]=V[6]*2.f;
	V[7]=V[7]*2.f;
	V[8]=V[8]*2.f;
	VV[0]=V[0]*sy*sy*sz*sz;
	VV[1]=V[1]*sx*sx*sz*sz;
	VV[2]=V[2]*sx*sx*sy*sy;
	VV[3]=V[3]*sx*sy*sz*sz;
	VV[4]=V[4]*sx*sy*sy*sz;
	VV[5]=V[5]*sx*sx*sy*sz;
	VV[6]=-2*V[0]*sy*sy*sz*sz*mx - V[3]*sx*sy*sz*sz*my - V[4]*sx*sy*sy*sz*mz + V[6]*sx*sy*sy*sz*sz;
	VV[7]=-2*V[1]*sx*sx*sz*sz*my - V[3]*sx*sy*sz*sz*mx - V[5]*sx*sx*sy*sz*mz + V[7]*sx*sx*sy*sz*sz;
	VV[8]=-2*V[2]*sx*sx*sy*sy*mz - V[4]*sx*sy*sy*sz*mx - V[5]*sx*sx*sy*sz*my + V[8]*sx*sx*sy*sy*sz;
	VV[9]=V[0]*sy*sy*sz*sz*mx*mx + V[1]*sx*sx*sz*sz*my*my + V[2]*sx*sx*sy*sy*mz*mz
          	+ V[3]*sx*sy*sz*sz*mx*my + V[4]*sx*sy*sy*sz*mx*mz + V[5]*sx*sx*sy*sz*my*mz
            - V[6]*sx*sy*sy*sz*sz*mx - V[7]*sx*sx*sy*sz*sz*my - V[8]*sx*sx*sy*sy*sz*mz
            + V[9]*sx*sx*sy*sy*sz*sz;
	for(i=0;i<10;i++)
	{
		V[i]=VV[i];
	}
	V[3]=V[3]/2.f;
	V[4]=V[4]/2.f;
	V[5]=V[5]/2.f;
	V[6]=V[6]/2.f;
	V[7]=V[7]/2.f;
	V[8]=V[8]/2.f;
	
	A[0]=V[0];
	A[1]=V[3];
	A[2]=V[4];
	A[3]=V[3];
	A[4]=V[1];
	A[5]=V[5];
	A[6]=V[4];
	A[7]=V[5];
	A[8]=V[2];
	for(i=0;i<9;i++)
	{
		mat[i]=A[i];
	}
	B[0]=V[6];
	B[1]=V[7];
	B[2]=V[8];
	if(ABS(MATRIX_Inv(mat,3))<(float)1e-35)
	{
		return 0;
	}
	for(i=0;i<9;i++)
	{
		mat[i]=-mat[i];
	}
	MATRIX_Mul(X0,mat,B,3,3,1);
	MATRIX_X_P_XT(&tmp,X0,A,X0,3);
	tmp-=V[9];
	for(i=0;i<9;i++)
	{
		Ae[i]=A[i]/tmp;
	}
	MATRIX_CongruentTransformation(Ae,(float*)P,(float*)D,3);
	for(i=0;i<3;i++)
	{
		if(D[i][i]<=0)
		{
			return 0;
		}
		P[0][i]/=sqrt(D[i][i]);
		P[1][i]/=sqrt(D[i][i]);
		P[2][i]/=sqrt(D[i][i]);
	}
	if(ABS(MATRIX_Inv((float*)P,3))<(float)1e-35)
	{
		return 0;
	}
	coef[0]=P[0][0];
	coef[1]=P[0][1];
	coef[2]=P[0][2];
	coef[3]=P[1][0];
	coef[4]=P[1][1];
	coef[5]=P[1][2];
	coef[6]=P[2][0];
	coef[7]=P[2][1];
	coef[8]=P[2][2];
	x0[0]=X0[0];
	x0[1]=X0[1];
	x0[2]=X0[2];
	return 1;
}











