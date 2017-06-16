#include "matrix.h"
//#include "stdlib.h"
#include "stm32f4xx.h"

/////////////////////////malloc-free///////////////////
#define CACHE_BYTES   (20*1024)//must be even
char cache_pool[CACHE_BYTES];
int cache_index=0;
void *malloc(int size)
{
	void *p;
	if(cache_index+size>CACHE_BYTES)
	{
		return 0;
	}
	p=&cache_pool[cache_index];
	cache_index+=size;
	return p;
}
void free(void * addr)
{
	if((unsigned long)addr<=(unsigned long)(&cache_pool[0]))
	{
		cache_index=0;
		return;
	}
	cache_index=(unsigned long)addr-(unsigned long)(&cache_pool[0]);
}
////////////////////////end///////////////////////////

float MATRIX_Det(float *m,int size)//the source matrix is changed
{
	float result=1,stemp;
	float temp;
	int switchtime=0;
	int row,nextrow,col;
	for(row=0;row<size-1;row++)
	{
		nextrow=row+1;
		if(m[row*size+row]==0)
		{
			while(m[nextrow*size+nextrow]==0)
			{
				nextrow++;
			}
			if((nextrow==size-1)&&(m[nextrow*size+nextrow]==0))
			{
				return 0;
			}
			else
			{
				switchtime++;
				for(col=0;col<size;col++)
				{
					stemp=m[row*size+col];
					m[row*size+col]=m[nextrow*size+col];
					m[nextrow*size+col]=stemp;
				}
			}
		}
		for(nextrow=row+1;nextrow<size;nextrow++)
		{
			temp=(float)m[nextrow*size+row]/m[row*size+row];
			for(col=0;col<size;col++)
			{
				m[nextrow*size+col]+=-temp*m[row*size+col];
			}
		}
	}
	
	for(row=0;row<size;row++)
	{
		result*=m[row*size+row];
	}
	if(switchtime%2)
	{
		return -result;
	}
	else
	{
		return result;
	}
}
char MATRIX_Add(float *ma,float *mb,int rows_num,int columns_num)//ma=ma+mb
{
	int i,j;
	
	if(rows_num<1||columns_num<1)
	{
		return 0;
	}
	for(i=0;i<rows_num;i++)
	{
		for(j=0;j<columns_num;j++)
		{
			ma[i*columns_num+j]=ma[i*columns_num+j]+mb[i*columns_num+j];
		}
	}
	return 1;
}
char MATRIX_Sub(float *ma,float *mb,int rows_num,int columns_num)//ma=ma-mb
{
	int i,j;
	if(rows_num<1||columns_num<1)
	{
		return 0;
	}
	for(i=0;i<rows_num;i++)
	{
		for(j=0;j<columns_num;j++)
		{
			ma[i*columns_num+j]=ma[i*columns_num+j]-mb[i*columns_num+j];
		}
	}
	return 1;
}
//char MATRIX_Mul(float *ma,float *mb,float *mc,int rows_num_mb,int columns_num_mb,int columns_num_mc)//ma=mb*mc
//{
//	int i,j,k;
//	float tmp;
//	u16 *ma_i,*mb_i,*mc_i;
//	int i_i;
//	
//	if(rows_num_mb<1||columns_num_mb<1||columns_num_mc<1)
//	{
//		return 0;
//	}
//	ma_i=(u16*)malloc(rows_num_mb<<1);
//	mb_i=(u16*)malloc(rows_num_mb<<1);
//	mc_i=(u16*)malloc(columns_num_mb<<1);
//	if(ma_i==0||mb_i==0||mc_i==0)
//	{
//		free(mc_i);
//		free(mb_i);
//		free(ma_i);
//		return 0;
//	}
//	ma_i[0]=0;
//	mb_i[0]=0;
//	mc_i[0]=0;
//	if(rows_num_mb>1)
//	{
//		for(i_i=1;i_i<rows_num_mb;i_i++)
//		{
//			ma_i[i_i]=ma_i[i_i-1]+columns_num_mc;
//			mb_i[i_i]=mb_i[i_i-1]+columns_num_mb;
//		}
//	}
//	
//	if(columns_num_mb>1)
//	{
//		for(i_i=1;i_i<columns_num_mb;i_i++)
//		{
//			mc_i[i_i]=mc_i[i_i-1]+columns_num_mc;
//		}
//	}
//	for(i=0;i<rows_num_mb;i++)
//	{
//		for(j=0;j<columns_num_mc;j++)
//		{
//			tmp=0;
//			for(k=0;k<columns_num_mb;k++)
//			{
//				tmp+=mb[mb_i[i]+k]*mc[mc_i[k]+j];
//			}
//			ma[ma_i[i]+j]=tmp;
//		}
//	}
//	free(mc_i);
//	free(mb_i);
//	free(ma_i);
//	return 1;
//}
char MATRIX_Mul(float *ma,float *mb,float *mc,int rows_num_mb,int columns_num_mb,int columns_num_mc)//ma=mb*mc
{
	int i,j,k;
	float tmp;
	if(rows_num_mb<1||columns_num_mb<1||columns_num_mc<1)
	{
		return 0;
	}
	for(i=0;i<rows_num_mb;i++)
	{
		for(j=0;j<columns_num_mc;j++)
		{
			tmp=0;
			for(k=0;k<columns_num_mb;k++)
			{
				tmp+=mb[i*columns_num_mb+k]*mc[k*columns_num_mc+j];
			}
			ma[i*columns_num_mc+j]=tmp;
		}
	}
	return 1;
}
char MATRIX_Copy(float *ma,float *mb,int rows_num,int columns_num)//ma=mb
{
	int i,j;
	if(rows_num<1||columns_num<1)
	{
		return 0;
	}
	for(i=0;i<rows_num;i++)
	{
		for(j=0;j<columns_num;j++)
		{
			ma[i*columns_num+j]=mb[i*columns_num+j];
		}
	}
	return 1;
}
char MATRIX_Transpose(float *des,float *src,int rows_num_src,int columns_num_src)//des=src'
{
	int i,j;
	if(rows_num_src<1||columns_num_src<1)
	{
		return 0;
	}
	for(i=0;i<columns_num_src;i++)
	{
		for(j=0;j<rows_num_src;j++)
		{
			des[i*rows_num_src+j]=src[j*columns_num_src+i];
		}
	}
	return 1;
}
char MATRIX_X_P_XT(float *y,float *x1,float *P,float *x2,int p_size)//y=x'*P*x
{
	int i,j;
	float tmp;
	if(p_size<1)
	{
		return 0;
	}
	tmp=0;
	for(i=0;i<p_size;i++)
	{
		for(j=0;j<p_size;j++)
		{
			tmp+=P[i*p_size+j]*x1[i]*x2[j];
		}
	}
	*y=tmp;
	return 1;
}
char MATRIX_A_P_AT(float *y,float *A,float *P,int rows_num_A,int p_size)//y=A*P*A'
{
	int i,j;
	if(p_size<1||rows_num_A<1)
	{
		return 0;
	}
	for(i=0;i<rows_num_A;i++)
	{
		for(j=i;j<rows_num_A;j++)
		{
			MATRIX_X_P_XT(&y[i*rows_num_A+j],&A[i*p_size],P,&A[j*p_size],p_size);
		}
	}
	if(rows_num_A>1)
	{
		for(i=1;i<rows_num_A;i++)
		{
			for(j=0;j<i;j++)
			{
				y[i*rows_num_A+j]=y[j*rows_num_A+i];
			}
		}
	}
	return 1;
}
char MATRIX_A_BT(float *y,float *A,float *B,int rows_num_A,int columns_num_A,int rows_num_B)//y=A*B'
{
	float tmp;
	int i,j,k;
	if(rows_num_A<1||columns_num_A<1||rows_num_B<1)
	{
		return 0;
	}
	for(i=0;i<rows_num_A;i++)
	{
		for(j=0;j<rows_num_B;j++)
		{
			tmp=0;
			for(k=0;k<columns_num_A;k++)
			{
				tmp+=A[i*columns_num_A+k]*B[j*columns_num_A+k];
			}
			y[i*rows_num_B+j]=tmp;
		}
	}
	return 1;
}
char MATRIX_AT_B(float *y,float *A,float *B,int rows_num_A,int columns_num_A,int columns_num_B)//y=A'*B
{
	float tmp;
	int i,j,k;
	if(rows_num_A<1||columns_num_A<1||columns_num_B<1)
	{
		return 0;
	}
	for(i=0;i<columns_num_A;i++)
	{
		for(j=0;j<columns_num_B;j++)
		{
			tmp=0;
			for(k=0;k<rows_num_A;k++)
			{
				tmp+=A[k*columns_num_A+i]*B[k*columns_num_B+j];
			}
			y[i*columns_num_B+j]=tmp;
		}
	}
	return 1;
}
	
//need:(rows_num*columns_num+columns_num*columns_num+columns_num)*4 BYTES
char MATRIX_LeastSquareSolution(float *m,float *y,float * solution,int rows_num,int columns_num)
{
	#define MAX(a,b)    ((a)>(b)?(a):(b))
	#define ABS(x)      ((x)>0?(x):-(x))
	float *tmp1,*tmp2,*tmp3;
	float res;
	if(rows_num<1||columns_num<1||rows_num<columns_num)
	{
		return 0;
	}
	tmp1=(float *)malloc(rows_num*columns_num*sizeof(float));
	tmp2=(float *)malloc(columns_num*columns_num*sizeof(float));
	tmp3=(float *)malloc(columns_num*sizeof(float));
	
	if(tmp1==0||tmp2==0||tmp3==0)
	{
		free(tmp3);
		free(tmp2);
		free(tmp1);
		return 0;
	}
	MATRIX_Transpose(tmp1,m,rows_num,columns_num);
	MATRIX_Mul(tmp2,tmp1,m,columns_num,rows_num,columns_num);
	res=MATRIX_Inv(tmp2,columns_num);
	if(ABS(res)<(float)1e-35)
	{
		free(tmp3);
		free(tmp2);
		free(tmp1);
		return 0;
	}
	MATRIX_Mul(tmp3,tmp1,y,columns_num,rows_num,1);
	MATRIX_Mul(solution,tmp2,tmp3,columns_num,columns_num,1);

	free(tmp3);
	free(tmp2);
	free(tmp1);
	return 1;
}

char MATRIX_CongruentTransformation(float *m,float *P,float *d,int size)//x=P*xx,P'*m*P=d
{
	float *a;
	float l;
	int i,j,k,r;
	if(size<2)
	{
		return 0;
	}
	a=(float*)malloc(size*size*2*sizeof(float));
	for(i=0;i<size*2;i++)
	{
		for(j=0;j<size;j++)
		{
			if(i<size)
			{
				a[i*size+j]=m[j*size+i];
			}
			else
			{
				if((i-size)==j)
				{
					a[i*size+j]=1;
				}
				else
				{
					a[i*size+j]=0;
				}
			}
		}
	}
	for(k=0;k<size;k++)
	{
		if(a[k*size+k]==0)
		{
			for(r=k+1;r<size;r++)
			{
				if(a[k*size+r]!=0)
				{
					for(i=k;i<size;i++)
					{
						a[k*size+i]+=a[r*size+i];
					}
					for(i=k;i<2*size;i++)
					{
						a[i*size+k]+=a[i*size+r];
					}
				}
			}
		}
		for(i=k+1;i<size;i++)
		{
			l=a[i*size+k]/a[k*size+k];
			for(j=k;j<size;j++)
			{
				a[i*size+j]-=l*a[k*size+j];
			}
			for(j=k;j<size*2;j++)
			{
				a[j*size+i]-=l*a[j*size+k];
			}
		}
	}
	for(i=0;i<size;i++)
	{
		for(j=0;j<size;j++)
		{
			P[i*size+j]=a[(i+size)*size+j];
			d[i*size+j]=a[i*size+j];
		}
	}

	free(a);
	return 1;	
}


