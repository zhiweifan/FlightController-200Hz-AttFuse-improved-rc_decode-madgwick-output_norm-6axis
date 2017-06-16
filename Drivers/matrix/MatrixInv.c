
#include "matrix.h"

/*************************�㷨���� ***************************************/
//
//����������3D�����кܳ�������ҪӦ������Billboard���󡣰��ն���ļ�
//�㷽���˷����㣬����Ӱ�������ܡ�����Ҫ����Billboard��������ʱ����
//��������Ż��ܼ���������ܡ�
//����Ҫ���ܵľ��������㷨��Ϊȫѡ��Ԫ��˹-Լ������ 
//
//��˹-Լ������ȫѡ��Ԫ������Ĳ������£� 
//���ȣ����� k �� 0 �� n - 1 ��ÿһ��ֵ�����¼����� 
//(1)�ӵ� k �С��� k �п�ʼ�����½�������ѡȡ����ֵ����Ԫ�أ�����ס��
//   Ԫ�����ڵ��кź��кţ���ͨ���н������н���������������Ԫ��λ���ϡ�
//   ��һ����Ϊȫѡ��Ԫ�� 
//(2) m(k, k) = 1 / m(k, k) 
//(3) m(k, j) = m(k, j) * m(k, k)��j = 0, 1, ..., n-1��j != k 
//(4) m(i, j) = m(i, j) - m(i, k) * m(k, j)��i, j = 0, 1, ..., n-1��i, j != k 
//(5) m(i, k) = -m(i, k) * m(k, k)��i = 0, 1, ..., n-1��i != k //ע�⸺��
//Ȼ�󣬸�����ȫѡ��Ԫ����������¼���С��н�������Ϣ���лָ����ָ���ԭ
//�����£�k��n-1��0����ȡֵ����ȫѡ��Ԫ�����м�¼����Ӧ���кź��кţ���ԭ����
//˳��������б任��ֻ�����Ѽ�¼���кŵ��кţ��Ѽ�¼���кŵ��кŷֱ����
//k�е�k�н��н���
/**************************************************************************/
#define  DIMENSION_MAX  10       //�����������ά��
#define  INF_N          (1e-30)   //��������С

void swap(float *a,float *b);
float Abs(float a);

/***************************ʵ��N�׾���**********************************/ 
/*************************����ԭ���������ֵ*****************************/
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
	char fh=1;  //����ֵ����
	if(size>DIMENSION_MAX)
	return 0;

	for (k = 0; k < size; k ++) 
	{ 
		// ��һ����ȫѡ��Ԫ 
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
		
		// ��������ֵ 
		fDet *= m[k*size+k]; 
		
		// ��������� 
		
		// �ڶ��� 
		m[k*size+k]= 1.0f / m[k*size+k]; 
		// ������ 
		for (j = 0; j < size; j ++) 
		{ 
			if (j != k) 
			m[k*size+j] *= m[k*size+k]; 
		} 
		// ���Ĳ� 
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
		// ���岽 
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

