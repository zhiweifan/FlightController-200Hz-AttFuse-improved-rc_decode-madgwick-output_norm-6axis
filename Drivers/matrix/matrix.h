#ifndef __MATRIX_H__
#define __MATRIX_H__

float MATRIX_Inv(float *m, int size); //返回原矩阵行列式
float MATRIX_Det(float *m,int size);//the source matrix is changed
int MATRIX_PInv(float a[],int m,int n,float a_inv[],float eps,float u[],float v[]);

//以下函数成功返回1
char MATRIX_Add(float *ma,float *mb,int rows_num,int columns_num);//ma=ma+mb
char MATRIX_Sub(float *ma,float *mb,int rows_num,int columns_num);//ma=ma-mb
char MATRIX_Mul(float *ma,float *mb,float *mc,int rows_num_mb,int columns_num_mb,int columns_num_mc);//ma=mb*mc
//char MATRIX_MulSquare(float **ma,float **mb,int size);//ma=ma*mb
char MATRIX_Copy(float *ma,float *mb,int rows_num,int columns_num);//ma=mb
char MATRIX_Transpose(float *des,float *src,int rows_num_src,int columns_num_src);//des=src'
char MATRIX_A_P_AT(float *y,float *A,float *P,int rows_num_A,int p_size);//y=A*P*A'
char MATRIX_X_P_XT(float *y,float *x1,float *P,float *x2,int p_size);//y=x'*P*x
char MATRIX_A_BT(float *y,float *A,float *B,int rows_num_A,int columns_num_A,int rows_num_B);//y=A*B'
char MATRIX_AT_B(float *y,float *A,float *B,int rows_num_A,int columns_num_A,int columns_num_B);//y=A'*B
//char MATRIX_A_B_C(float *y,float *A,float *B,float *C,int rows_num_A,int columns_num_A,int columns_num_B,int columns_num_C);//y=A*B*C
char MATRIX_LeastSquareSolution(float *m,float *y,float * solution,int rows_num,int columns_num);//solution=m\y
//congruent transformation
char MATRIX_CongruentTransformation(float *m,float *P,float *d,int size);//x=P*xx,P'*m*P=d

void *malloc(int size);
void free(void * addr);

#endif

