
#ifndef GPS_KALMAN
#define GPS_KALMAN

#include "typedef.h"

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
}GPS_KALMAN_TypeDef;

typedef __packed struct
{
	float q_h;
	float q_v;
	float q_a;
	float q_a_bias;
	float r_h;
	float r_a;
}GPS_KALMAN_QR_TypeDef;

extern GPS_KALMAN_TypeDef gps_kalman;
extern GPS_KALMAN_QR_TypeDef gps_kalman_qr;

void GPS_KALMAN_Config(void);
void GPS_KALMAN_Update(void);

void GPS_KALMAN_SetQR(void);
void GPS_KALMAN_SetDefaultQR(void);

extern CtrlSys_TypeDef CtrlSys;

#endif



