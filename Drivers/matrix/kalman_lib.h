
#ifndef KALMAN_LIB_H
#define KALMAN_LIB_H

//#define X_SIZE   4
//#define Z_SIZE   2

////x(k+1)=A*x(k)+xi
////z(k)=C*x(k)+eta
////D(xi)=Q,D(eta)=R
//typedef struct
//{
//	float x[X_SIZE];
//	float z[Z_SIZE];
//	float A[X_SIZE][X_SIZE];
//	float C[Z_SIZE][X_SIZE];
//	float Q[X_SIZE][X_SIZE];
//	float R[Z_SIZE][Z_SIZE];
//	float P[X_SIZE][X_SIZE];
//}KALMAN_TypeDef;
typedef struct
{
	int x_size;
	int z_size;
	float *x;
	float *z;
	float *A;
	float *C;
	float *Q;
	float *R;
	float *P;
}KALMAN_PARAMS_TypeDef;


//minimum heap size needed is x_size*x_size+x_size*max(1+z_size,x_size)
char KALMAN_Update(KALMAN_PARAMS_TypeDef* pkalman);

#endif



