
#ifndef ALT_FILTER_H
#define ALT_FILTER_H

#define X_SIZE   4
#define Z_SIZE   2

//x(k+1)=A*x(k)+xi
//z(k)=C*x(k)+eta
//D(xi)=Q,D(eta)=R
typedef __packed struct
{
	float x[X_SIZE];
	float z[Z_SIZE];
	float A[X_SIZE][X_SIZE];
	float C[Z_SIZE][X_SIZE];
	float Q[X_SIZE][X_SIZE];
	float R[Z_SIZE][Z_SIZE];
	float P[X_SIZE][X_SIZE];
	float deltat;
	float acc_line_z_tc;
}ALT_FILTER_TypeDef;

typedef __packed struct
{
	float q_h;
	float q_v;
	float q_a;
	float q_a_bias;
	float r_h;
	float r_a;
}ALT_FILTER_QR_TypeDef;

extern ALT_FILTER_TypeDef alt_filter;
extern ALT_FILTER_QR_TypeDef alt_filter_qr;

void ALT_FILTER_Config(void);
void ALT_Update(void);

void ALT_FILTER_SetQR(void);
void ALT_FILTER_SetDefaultQR(void);



#endif



