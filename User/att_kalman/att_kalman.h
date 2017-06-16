
#ifndef ATT_KALMAN_H
#define ATT_KALMAN_H

#define ATT_X_SIZE   11//12
#define ATT_Z_SIZE   6

#define YAW_X_SIZE   5
#define YAW_Z_SIZE   3

#define ACC_SCOPE							4.f//+-2G
#define GYRO_SCOPE						2000.f//+-2000°/s
#define GRAVITY  							((float)9.794f)
	
//x(k+1)=A*x(k)+xi
//z(k)=C*x(k)+eta
//D(xi)=Q,D(eta)=R
typedef struct
{

volatile	float deltat;
//volatile	float yaw_r;
volatile	float pitch_r;
volatile	float roll_r;
volatile	float cos_roll;
volatile	float cos_pitch;
volatile	float sin_roll;
volatile	float sin_pitch;
volatile	float gx;
volatile	float gy;
volatile	float gz;
volatile	float yaw;
volatile	float pitch;
volatile	float roll;
volatile	float ax_line;
volatile	float ay_line;
volatile	float az_line;
volatile	float acc_norm;
volatile float yaw_test;
}ATT_KALMAN_TypeDef;

typedef struct
{
volatile	float x[YAW_X_SIZE];
volatile	float z[YAW_Z_SIZE];
volatile	float A[YAW_X_SIZE][YAW_X_SIZE];
volatile	float C[YAW_Z_SIZE][YAW_X_SIZE];
volatile	float Q[YAW_X_SIZE][YAW_X_SIZE];
volatile	float R[YAW_Z_SIZE][YAW_Z_SIZE];
volatile	float P[YAW_X_SIZE][YAW_X_SIZE];
volatile	float deltat;
volatile	float yaw_r;
volatile	float gz;
volatile	float yaw;
	
}YAW_KALMAN_TypeDef;

typedef __packed struct
{
volatile	float q_acc;
volatile	float q_acc_z;
volatile	float q_rot_speed;
volatile	float q_rot_speed_z;
volatile	float q_rot_acc;
volatile	float q_rot_acc_z;
volatile	float q_rot_speed_bias;
volatile	float q_mag;
volatile	float r_acc;
volatile	float r_acc_z;
volatile	float r_gyro;
volatile	float r_gyro_z;
volatile	float r_mag;
	
}ATT_KALMAN_QR_TypeDef;


extern YAW_KALMAN_TypeDef yaw_kalman;
extern ATT_KALMAN_QR_TypeDef att_kalman_qr;
extern ATT_KALMAN_TypeDef att_kalman;
void ATT_KALMAN_Config(void);
void ATT_KALMAN_SetQR(void);
void ATT_KALMAN_SetDefaultQR(void);
void ATT_Update(float ax,float ay,float az,float gx,float gy,float gz,float mx,float my,float mz,char cal_yaw,float deltat);




#endif



