
#include "matrix.h"

/*************************算法介绍 ***************************************/
//
//矩阵求逆在3D程序中很常见，主要应用于求Billboard矩阵。按照定义的计
//算方法乘法运算，严重影响了性能。在需要大量Billboard矩阵运算时，矩
//阵求逆的优化能极大提高性能。
//这里要介绍的矩阵求逆算法称为全选主元高斯-约旦法。 
//
//高斯-约旦法（全选主元）求逆的步骤如下： 
//首先，对于 k 从 0 到 n - 1 的每一个值作如下几步： 
//(1)从第 k 行、第 k 列开始的右下角子阵中选取绝对值最大的元素，并记住次
//   元素所在的行号和列号，在通过行交换和列交换将它交换到主元素位置上。
//   这一步称为全选主元。 
//(2) m(k, k) = 1 / m(k, k) 
//(3) m(k, j) = m(k, j) * m(k, k)，j = 0, 1, ..., n-1；j != k 
//(4) m(i, j) = m(i, j) - m(i, k) * m(k, j)，i, j = 0, 1, ..., n-1；i, j != k 
//(5) m(i, k) = -m(i, k) * m(k, k)，i = 0, 1, ..., n-1；i != k //注意负号
//然后，根据在全选主元过程中所记录的行、列交换的信息进行恢复，恢复的原
//则如下：k从n-1到0逆向取值，在全选主元过程中记录了相应的行号和列号，依原来的
//顺序进行行列变换，只不过把记录的列号当行号，把记录的行号当列号分别与第
//k行第k列进行交换
/**************************************************************************/
#define  DIMENSION_MAX  10       //被求矩阵的最大维数
#define  INF_N          (1e-30)   //定义无穷小

void swap(float *a,float *b);
float Abs(float a);

/***************************实现N阶矩阵**********************************/ 
/*************************返回原矩阵的行列值*****************************/
float MATRIX_Inv(float *m, int size)
{ 
	unsigned char is[DIMENSION_MAX]; 
	unsigned char js[DIMENSION_MAX]; 
	float fDet = 1.0f; 
	float fMax = 0.0f; 
	float f=0;
	int k=0;
	int tmp=0;
	int i=0;
	int j=0;
	char fh=1;  //行列值符号
	if(size>DIMENSION_MAX)
	return 0;

	for (k = 0; k < size; k ++) 
	{ 
		// 第一步，全选主元 
		fMax=0;
		for (i = k; i < size; i ++) 
		{ 
			for (j = k; j < size; j ++) 
			{ 
				f = Abs(m[i*size+j]); 
				if (f > fMax) 
				{ 
					fMax = f; 
					is[k] = i; 
					js[k] = j; 
				} 
			} 
		} 
		if (fMax <(float)INF_N ) 
		return 0; 
		
		if (is[k] != k) 
		{ 
			fh=-fh;
			for(tmp=0;tmp<size;tmp++)
			{
				swap(&m[k*size+tmp],&m[is[k]*size+tmp]);
			}
		} 
		if (js[k] != k) 
		{ 
			fh=-fh;
			for(tmp=0;tmp<size;tmp++)
			{
				swap(&m[tmp*size+k],&m[tmp*size+is[k]]);
			}
		} 
		
		// 计算行列值 
		fDet *= m[k*size+k]; 
		
		// 计算逆矩阵 
		
		// 第二步 
		m[k*size+k]= 1.0f / m[k*size+k]; 
		// 第三步 
		for (j = 0; j < size; j ++) 
		{ 
			if (j != k) 
			m[k*size+j] *= m[k*size+k]; 
		} 
		// 第四步 
		for (i = 0; i < size; i ++) 
		{ 
			if (i != k) 
			{ 
				for (j = 0; j < size; j ++) 
				{ 
					if (j != k) 
					m[i*size+j] = m[i*size+j] - m[i*size+k] * m[k*size+j]; 
				} 
			} 
		} 
		// 第五步 
		for (i = 0; i < size; i ++) 
		{ 
			if (i != k) 
			m[i*size+k] *= -m[k*size+k]; 
		} 
	} 
	
	for (k = size-1; k >= 0; k --) 
	{ 
		if (js[k] != k) 
		{ 
			for(tmp=0;tmp<size;tmp++)
			{
				swap(&m[k*size+tmp],&m[js[k]*size+tmp]);
			}
		} 
		if (is[k] != k) 
		{ 
			for(tmp=0;tmp<size;tmp++)
			{
				swap(&m[tmp*size+k],&m[tmp*size+is[k]]);
			}
		} 
	} 

	return fDet * fh; 
} 

void swap(float *a,float *b)
{
	float temp=0;
	temp=*a;
	*a=*b;
	*b=temp;
}
float Abs(float a)
{
	if(a<0)
	return -a;
	else
	return a;
}

