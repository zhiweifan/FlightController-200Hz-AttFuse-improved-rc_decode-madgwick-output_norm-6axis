



#include "dataframe.h"
#include "main.h"
//#include "stm32f4xx.h"

RxDataFrame_TypeDef rxdf1,rxdf2;
TxDataFrame_TypeDef txdf1,txdf2;
volatile u32 rc_sw_value;

void DataFrame_Init(void)
{
	txdf1.DataBytes=0;
	txdf1.FIFO=USART1_DMA_SendBuff;
	txdf1.FifoSize=USART1_TX_DMA_BUFFSIZE;
	txdf1.FrameCnt=0;
	txdf1.FrameType=0;
	txdf1.pData=0;
	txdf1.ReadIndex=0;
	txdf1.WriteIndex=0;
	txdf1.DMA_Stream=USART1_TX_DMA_STREAM;
	txdf1.DMA_ReadIndex=0;
	txdf1.DMA_TransNum=0;
	
	rxdf1.DataBytesMax=500;
	rxdf1.FIFO=USART1_DMA_RecvBuff;
	rxdf1.FifoSize=USART1_RX_DMA_BUFFSIZE;
	rxdf1.FrameCnt=0;
	rxdf1.FrameType=0;
	rxdf1.ReadIndex=0;
	rxdf1.WriteIndex=0;
	rxdf1.DMA_Stream=USART1_RX_DMA_STREAM;
	
	
	txdf2.DataBytes=0;
	txdf2.FIFO=USART2_DMA_SendBuff;
	txdf2.FifoSize=USART2_TX_DMA_BUFFSIZE;
	txdf2.FrameCnt=0;
	txdf2.FrameType=0;
	txdf2.pData=0;
	txdf2.ReadIndex=0;
	txdf2.WriteIndex=0;
	txdf2.DMA_Stream=USART2_TX_DMA_STREAM;
	txdf2.DMA_ReadIndex=0;
	txdf2.DMA_TransNum=0;
	
	rxdf2.DataBytesMax=24;
	rxdf2.FIFO=USART2_DMA_RecvBuff;
	rxdf2.FifoSize=USART2_RX_DMA_BUFFSIZE;
	rxdf2.FrameCnt=0;
	rxdf2.FrameType=0;
	rxdf2.ReadIndex=0;
	rxdf2.WriteIndex=0;
	rxdf2.DMA_Stream=USART2_RX_DMA_STREAM;
	
}
#define  MAX_SENT_PARAMS      80
char DataFrame_SendParams(TxDataFrame_TypeDef* pTxDataFrame,float* tosend,u8* index,u8 num)
{
	u8 data[MAX_SENT_PARAMS*5+6];
	u16 i=0;
	if((num)>MAX_SENT_PARAMS)
	{
		return 0;
	}
	for(i=0;i<num;i++)
	{
		data[i*5]=index[i];
		data[i*5+1]=*(TRANSP(tosend[i],u8));
		data[i*5+2]=*(TRANSP(tosend[i],u8)+1);
		data[i*5+3]=*(TRANSP(tosend[i],u8)+2);
		data[i*5+4]=*(TRANSP(tosend[i],u8)+3);
	}

	pTxDataFrame->pData=data;
	pTxDataFrame->DataBytes=num*5;
	pTxDataFrame->FrameCnt=num;
	pTxDataFrame->FrameType=0x80;

	return DataFrame_WriteFifo(pTxDataFrame);
}
void SetPIDParams(u8 *data,u8 params_cnt);
void GetPIDParams(u8 *data,u8 params_cnt);
void SwitchDecode(u32 data);
void KeyDecode(u8 data);
void SendIMUCalibParams(char sel);
char DataFrame_GetData(RxDataFrame_TypeDef* pRxDataFrame)
{
	u8 data[500]={0};
	u16 pw;
	//TODO
	u32 tmp=0;
	
	/////////////////////////////////////////////////////////////////////////////
	pw=pRxDataFrame->FifoSize-DMA_GetCurrDataCounter(pRxDataFrame->DMA_Stream);
	if(pw>=pRxDataFrame->FifoSize)
	{
		pw-=pRxDataFrame->FifoSize;
	}
	pRxDataFrame->WriteIndex=pw;
	pRxDataFrame->pData=data;
	if(!DataFrame_CheckFifo(pRxDataFrame))
	{
		return 0;
	}
	//TODO
	
	if(pRxDataFrame==(&rxdf1))
	{
		if((pRxDataFrame->FrameType&0x80)==0)
		{
			if(pRxDataFrame->FrameType==100&&pRxDataFrame->FrameCnt==12)
			{
				CtrlSys.rc_ok_cnt++;
				CtrlSys.ref_yaw_raw=data[0]+(data[1]<<8);
				CtrlSys.ref_pitch_raw=data[2]+(data[3]<<8);
				CtrlSys.ref_roll_raw=data[4]+(data[5]<<8);
				CtrlSys.throttle_raw=data[6]+(data[7]<<8);
				CtrlSys.ref_gimbal_yaw_raw=data[8]+(data[9]<<8);
				CtrlSys.ref_gimbal_pitch_raw=data[10]+(data[11]<<8);
				
			}
			else if(pRxDataFrame->FrameType==101&&pRxDataFrame->FrameCnt==4)
			{
				SwitchDecode(data[0]+(data[1]<<8)+(data[2]<<16)+(data[3]<<24));
			}
			else if(pRxDataFrame->FrameType==102&&pRxDataFrame->FrameCnt==1)
			{
				KeyDecode(data[0]);
				
			}
		}
		else//遥控器指令
		{
			BEEP_SetStatus(BEEP_STATUS_ACK);
			
			if(pRxDataFrame->FrameType==255&&pRxDataFrame->FrameCnt==1&&data[0]==0)
			{
				CtrlSys.stop=data[1];
				if(CtrlSys.stop)
				{
					ControllerReset();
					SetMotorStop();
				}
			}
			else if(pRxDataFrame->FrameType==(100+128))//转发给传感器主控
			{
				txdf2.pData=data;
				txdf2.DataBytes=pRxDataFrame->FrameCnt*5;
				txdf2.FrameCnt=pRxDataFrame->FrameCnt;
				txdf2.FrameType=128;
				DataFrame_WriteFifo(&txdf2);
				imu_cmd.cmd=data[0];
				imu_cmd.param=FloatWord2Float(data[1]+(data[2]<<8)+(data[3]<<16)+(data[4]<<24));
			}
			else if(pRxDataFrame->FrameType==(101+128))//Get IMU Calibrate Params
			{
				SendIMUCalibParams(data[0]);
			}
			else if(pRxDataFrame->FrameType==128)//PID控制器参数设置
			{
				SetPIDParams(data,pRxDataFrame->FrameCnt);
				
			}
			else if(pRxDataFrame->FrameType==129)//转发PID控制器参数到上位机
			{
				GetPIDParams(data,pRxDataFrame->FrameCnt);
			}
			
		
		}
		
	}
	else if(pRxDataFrame==(&rxdf2))//传感器数据
	{
		if(pRxDataFrame->FrameCnt==24&&pRxDataFrame->FrameType==0)
		{
			tmp=data[0]+(data[1]<<8);
			ahrs.ax_raw=tmp;
			tmp=data[2]+(data[3]<<8);
			ahrs.ay_raw=tmp;
			tmp=data[4]+(data[5]<<8);
			ahrs.az_raw=tmp;
			tmp=data[6]+(data[7]<<8);
			ahrs.gx_raw=tmp;
			tmp=data[8]+(data[9]<<8);
			ahrs.gy_raw=tmp;
			tmp=data[10]+(data[11]<<8);
			ahrs.gz_raw=tmp;
			tmp=data[12]+(data[13]<<8)+(data[14]<<16)+(data[15]<<24);
			ahrs.altitude=FloatWord2Float(tmp);
			tmp=data[18]+(data[19]<<8);
			ahrs.mx_raw=tmp;
			tmp=data[20]+(data[21]<<8);
			ahrs.my_raw=tmp;
			tmp=data[22]+(data[23]<<8);
			ahrs.mz_raw=tmp;
		}
		else if(pRxDataFrame->FrameCnt==21&&pRxDataFrame->FrameType==1)
		{
			tmp=data[0]+(data[1]<<8)+(data[2]<<16)+(data[3]<<24);
			gps.longitude=FloatWord2Float(tmp);
			tmp=data[4]+(data[5]<<8)+(data[6]<<16)+(data[7]<<24);
			gps.latitude=FloatWord2Float(tmp);
			tmp=data[8]+(data[9]<<8)+(data[10]<<16)+(data[11]<<24);
			gps.altitude=FloatWord2Float(tmp);
			tmp=data[12]+(data[13]<<8)+(data[14]<<16)+(data[15]<<24);
			gps.course=FloatWord2Float(tmp);
			tmp=data[16]+(data[17]<<8)+(data[18]<<16)+(data[19]<<24);
			gps.speed=FloatWord2Float(tmp);
			tmp=data[20];
			gps.satellite_num=tmp;
		}
	}
	return 1;
}



char DataFrame_SendData(TxDataFrame_TypeDef* pTxDataFrame)
{
	u8 data[80]={0};
	u32 tmp=0;
	
	if(pTxDataFrame==(&txdf1))
	{
		tmp=Float2Halfword(ahrs.yaw,180,1);//1
		data[0]=tmp;
		data[1]=tmp>>8;
		tmp=Float2Halfword(ahrs.pitch-CtrlSys.bias_pitch,180,1);//2
		data[2]=tmp;
		data[3]=tmp>>8;
		tmp=Float2Halfword(ahrs.roll-CtrlSys.bias_roll,180,1);//3
		data[4]=tmp;
		data[5]=tmp>>8;
		tmp=Float2Halfword(ahrs.gx,2000.f,1);//4
		data[6]=tmp;
		data[7]=tmp>>8;
		tmp=Float2Halfword(ahrs.gy,2000.f,1);//5
		data[8]=tmp;
		data[9]=tmp>>8;
		tmp=Float2Halfword(ahrs.gz,2000.f,1);//6
		data[10]=tmp;
		data[11]=tmp>>8;
		tmp=Float2Halfword(YawPID.Inputs.Ref,180,1);//7
		data[12]=tmp;
		data[13]=tmp>>8;
		tmp=Float2Halfword(PitchPID.Inputs.Ref,180,1);//8
		data[14]=tmp;
		data[15]=tmp>>8;
		tmp=Float2Halfword(RollPID.Inputs.Ref,180,1);//9
		data[16]=tmp;
		data[17]=tmp>>8;
		tmp=Float2Halfword(RollRatePID.Inputs.Ref,2000.f,1);//10
		data[18]=tmp;
		data[19]=tmp>>8;
		tmp=Float2Halfword(PitchRatePID.Inputs.Ref,2000.f,1);//11
		data[20]=tmp;
		data[21]=tmp>>8;
		tmp=Float2Halfword(YawRatePID.Inputs.Ref,2000.f,1);//12
		data[22]=tmp;
		data[23]=tmp>>8;
		if(CtrlSys.stop)
		{
			if(CtrlSys.height_close_loop_mode==3)
			{
				tmp=Float2FloatWord(alt_filter.x[0]-CtrlSys.height_bias);//13
			}
			else
			{
				tmp=Float2FloatWord(alt_filter.x[0]);//13
			}
		}
		else
		{
			tmp=Float2FloatWord(alt_filter.x[0]-CtrlSys.height_bias);//13
		}
		
		data[24]=tmp;
		data[25]=tmp>>8;
		data[26]=tmp>>16;
		data[27]=tmp>>24;
		tmp=Float2Halfword(ahrs.ax_line,4.f*GRAVITY,1);//14
		data[28]=tmp;
		data[29]=tmp>>8;
		tmp=Float2Halfword(ahrs.ay_line,4.f*GRAVITY,1);//15
		data[30]=tmp;
		data[31]=tmp>>8;
		if(CtrlSys.stop)
		{
			tmp=Float2Halfword(ahrs.az_line,4.f*GRAVITY,1);//16
		}
		else
		{
			tmp=Float2Halfword(AccPID.Status.MeasureFilter,4.f*GRAVITY,1);//16
		}
		
		data[32]=tmp;
		data[33]=tmp>>8;
		tmp=CtrlSys.throttle_raw;//17
		data[34]=tmp;
		data[35]=tmp>>8;
		tmp=Float2Halfword(CtrlSys.throttle_mean,PID_MAX,0);//18
		data[36]=tmp;
		data[37]=tmp>>8;
		tmp=Float2FloatWord(HeightPID.Inputs.Ref);//19
		data[38]=tmp;
		data[39]=tmp>>8;
		data[40]=tmp>>16;
		data[41]=tmp>>24;
		tmp=Float2Halfword(HeightRatePID.Status.MeasureFilter,2.0f,1);//20
		data[42]=tmp;
		data[43]=tmp>>8;
		tmp=Float2Halfword(HeightRatePID.Inputs.Ref,2.0f,1);//21
		data[44]=tmp;
		data[45]=tmp>>8;
		tmp=Float2Halfword(AccPID.Inputs.Ref,10.0f,1);//22
		data[46]=tmp;
		data[47]=tmp>>8;
		if(CtrlSys.stop)
		{
			tmp=Float2FloatWord(ahrs.altitude);//23
		}
		else
		{
			tmp=Float2FloatWord(ahrs.altitude-CtrlSys.height_bias);//23
		}
		data[48]=tmp;
		data[49]=tmp>>8;
		data[50]=tmp>>16;
		data[51]=tmp>>24;
		
		pTxDataFrame->pData=data;
		pTxDataFrame->DataBytes=52;
		pTxDataFrame->FrameCnt=52;
		pTxDataFrame->FrameType=0;
	}

	return DataFrame_WriteFifo(pTxDataFrame);
}


void SetPIDParams(u8 *data,u8 params_cnt)
{
	u32 tmp=0,tmp2=0;
	float tmpf=0;
	char imu_flag=0;
	for(tmp=0;tmp<params_cnt;tmp++)
	{
		tmp2=tmp*5;
		tmpf=FloatWord2Float(data[tmp2+1]+(data[tmp2+2]<<8)+(data[tmp2+3]<<16)+(data[tmp2+4]<<24));
		switch(data[tmp2])
		{
			case 1:
				if(CtrlSys.test_mode==1)
				{
					SetMotor1(tmpf,0);
				}
			break;
			case 2:
				if(CtrlSys.test_mode==1)
				{
					SetMotor2(tmpf,0);
				}
			break;
			case 3:
				if(CtrlSys.test_mode==1)
				{
					SetMotor3(tmpf,0);
				}
			break;
			case 4:
				if(CtrlSys.test_mode==1)
				{
					SetMotor4(tmpf,0);
				}
			break;
			case 5:
				if(CtrlSys.test_mode==1)
				{
					SetMotor5(tmpf,0);
				}
			break;
			case 6:
				if(CtrlSys.test_mode==1)
				{
					SetMotor6(tmpf,0);
				}
			break;
				case 7:
				if(CtrlSys.test_mode==1)
				{
					SetMotor7(tmpf,0);
				}
			break;
			case 8:
				if(CtrlSys.test_mode==1)
				{
					SetMotor8(tmpf,0);
				}
			break;
			case 9:
				if(CtrlSys.test_mode==1)
				{
					SetMotor9(tmpf,0);
				}
			break;
			case 10:
				if(CtrlSys.test_mode==1)
				{
					SetMotor10(tmpf,0);
				}
			break;
			
			case 11:
				PitchPID.Params.Kp=tmpf;
			break;
			case 12:
				PitchPID.Params.Ki=tmpf;
			break;
			case 13:
				PitchPID.Params.Kd=tmpf;
			break;
			case 14:
				PitchPID.Params.MeasureTc=tmpf;
			break;
			case 15:
				PitchPID.Params.ErrDevTc=tmpf;
			break;
			case 16:
				PitchPID.Params.RefTc=tmpf;
			break;
			case 21:
				PitchRatePID.Params.Kp=tmpf;
			break;
			case 22:
				PitchRatePID.Params.Ki=tmpf;
			break;
			case 23:
				PitchRatePID.Params.Kd=tmpf;
			break;
			case 24:
				PitchRatePID.Params.MeasureTc=tmpf;
			break;
			case 25:
				PitchRatePID.Params.ErrDevTc=tmpf;
			break;
			case 26:
				PitchRatePID.Params.Kdd=tmpf;
			break;
			case 27:
				PitchRatePID.Params.ErrDDevTc=tmpf;
			break;
			case 28:
				PitchRatePID.Params.RefTc=tmpf;
			break;
			case 31:
				RollPID.Params.Kp=tmpf;
			break;
			case 32:
				RollPID.Params.Ki=tmpf;
			break;
			case 33:
				RollPID.Params.Kd=tmpf;
			break;
			case 34:
				RollPID.Params.MeasureTc=tmpf;
			break;
			case 35:
				RollPID.Params.ErrDevTc=tmpf;
			break;
			case 36:
				RollPID.Params.RefTc=tmpf;
			break;
			case 41:
				RollRatePID.Params.Kp=tmpf;
			break;
			case 42:
				RollRatePID.Params.Ki=tmpf;
			break;
			case 43:
				RollRatePID.Params.Kd=tmpf;
			break;
			case 44:
				RollRatePID.Params.MeasureTc=tmpf;
			break;
			case 45:
				RollRatePID.Params.ErrDevTc=tmpf;
			break;
			case 46:
				RollRatePID.Params.Kdd=tmpf;
			break;
			case 47:
				RollRatePID.Params.ErrDDevTc=tmpf;
			break;
			case 48:
				RollRatePID.Params.RefTc=tmpf;
			break;
			case 51:
				YawPID.Params.Kp=tmpf;
			break;
			case 52:
				YawPID.Params.Ki=tmpf;
			break;
			case 53:
				YawPID.Params.Kd=tmpf;
			break;
			case 54:
				YawPID.Params.MeasureTc=tmpf;
			break;
			case 55:
				YawPID.Params.ErrDevTc=tmpf;
			break;
			case 56:
				YawPID.Params.RefTc=tmpf;
			break;
			case 61:
				YawRatePID.Params.Kp=tmpf;
			break;
			case 62:
				YawRatePID.Params.Ki=tmpf;
			break;
			case 63:
				YawRatePID.Params.Kd=tmpf;
			break;
			case 64:
				YawRatePID.Params.MeasureTc=tmpf;
			break;
			case 65:
				YawRatePID.Params.ErrDevTc=tmpf;
			break;
			case 66:
				YawRatePID.Params.Kdd=tmpf;
			break;
			case 67:
				YawRatePID.Params.ErrDDevTc=tmpf;
			break;
			case 68:
				YawRatePID.Params.RefTc=tmpf;
			break;
			
			case 71:
				AccPID.Params.Kp=tmpf;
			break;
			case 72:
				AccPID.Params.Ki=tmpf;
			break;
			case 73:
				AccPID.Params.Kd=tmpf;
			break;
			case 74:
				AccPID.Params.MeasureTc=tmpf;
			break;
			case 75:
				AccPID.Params.ErrDevTc=tmpf;
			break;
			case 76:
				AccPID.Params.RefTc=tmpf;
			break;
			case 77:
				CtrlSys.accel_output_bias=LIMIT_MIN_MAX(tmpf,0,4095.f);
			break;
			case 81:
				HeightPID.Params.Kp=tmpf;
			break;
			case 82:
				HeightPID.Params.Ki=tmpf;
			break;
			case 83:
				HeightPID.Params.Kd=tmpf;
			break;
			case 84:
				HeightPID.Params.MeasureTc=tmpf;
			break;
			case 85:
				HeightPID.Params.ErrDevTc=tmpf;
			break;
			case 86:
				HeightPID.Params.RefTc=tmpf;
			break;
			case 91:
				HeightRatePID.Params.Kp=tmpf;
			break;
			case 92:
				HeightRatePID.Params.Ki=tmpf;
			break;
			case 93:
				HeightRatePID.Params.Kd=tmpf;
			break;
			case 94:
				HeightRatePID.Params.MeasureTc=tmpf;
			break;
			case 95:
				HeightRatePID.Params.ErrDevTc=tmpf;
			break;
			case 96:
				HeightRatePID.Params.RefTc=tmpf;
			break;
			case 200:
				CtrlSys.bias_pitch=tmpf;
			break;
			case 201:
				CtrlSys.bias_roll=tmpf;
			break;
			case 203:
				CtrlSys.duty_min=LimitMinMax_float(tmpf,DUTY_LOW_MIN,DUTY_LOW_MAX);
			break;
			case 204:
				CtrlSys.duty_max=LimitMinMax_float(tmpf,DUTY_UP_MIN,DUTY_UP_MAX);
			break;
		
			case 206:
				CtrlSys.acc_line_z_bias=tmpf;
			break;
			case 207:
				CtrlSys.yaw_bias=tmpf;
			break;
			case 208:
				CtrlSys.idling_throttle=LIMIT_MIN_MAX(tmpf,0,600);
			break;
//			case 210:
//				att_kalman_qr.q_acc=tmpf;
//				imu_flag=1;
//			break;
//			case 211:
//				att_kalman_qr.q_acc_z=tmpf;
//				imu_flag=1;
//			break;
//			case 212:
//				att_kalman_qr.q_rot_speed=tmpf;
//				imu_flag=1;
//			break;
//			case 213:
//				att_kalman_qr.q_rot_speed_z=tmpf;
//				imu_flag=1;
//			break;
//			case 214:
//				att_kalman_qr.q_rot_acc=tmpf;
//				imu_flag=1;
//			break;
//			case 215:
//				att_kalman_qr.q_rot_acc_z=tmpf;
//				imu_flag=1;
//			break;
//			case 216:
//				att_kalman_qr.q_rot_speed_bias=tmpf;
//				imu_flag=1;
//			break;
//			case 217:
//				att_kalman_qr.q_mag=tmpf;
//				imu_flag=1;
//			break;
//			case 218:
//				att_kalman_qr.r_acc=tmpf;
//				imu_flag=1;
//			break;
//			case 219:
//				att_kalman_qr.r_acc_z=tmpf;
//				imu_flag=1;
//			break;
//			case 220:
//				att_kalman_qr.r_gyro=tmpf;
//				imu_flag=1;
//			break;
//			case 221:
//				att_kalman_qr.r_gyro_z=tmpf;
//				imu_flag=1;
//			break;
//			case 222:
//				att_kalman_qr.r_mag=tmpf;
//				imu_flag=1;
//			break;
			case 223:
				alt_filter_qr.q_h=tmpf;
				imu_flag=1;
			break;
			case 224:
				alt_filter_qr.q_v=tmpf;
				imu_flag=1;
			break;
			case 225:
				alt_filter_qr.q_a=tmpf;
				imu_flag=1;
			break;
			case 226:
				alt_filter_qr.q_a_bias=tmpf;
				imu_flag=1;
			break;
			case 227:
				alt_filter_qr.r_h=tmpf;
				imu_flag=1;
			break;
			case 228:
				alt_filter_qr.r_a=tmpf;
				imu_flag=1;
			break;
			case 229:
				alt_filter.acc_line_z_tc=tmpf;
			break;
			case 230:
				if(data[tmp2+1])
				{
					CtrlSys.yaw_close_loop=1;
				}
				else
				{
					CtrlSys.yaw_close_loop=0;
				}
			break;
		}
	}
	if(imu_flag==1)
	{
//		ATT_KALMAN_SetQR();
		ALT_FILTER_SetQR();
	}
}




void GetPIDParams(u8 *data,u8 params_cnt)
{
	u32 tmp=0,tmp2=0;
	float tmpf[MAX_SENT_PARAMS];
	u8 index[MAX_SENT_PARAMS];
	if(params_cnt>MAX_SENT_PARAMS)
	{
		return;
	}
	for(tmp=0;tmp<params_cnt;tmp++)
	{
		tmp2=tmp*5;
		index[tmp]=data[tmp2];
		switch(index[tmp])
		{
			case 1:
				
			break;
			case 2:
				
			break;
			case 3:
				
			break;
			case 4:
				
			break;
			case 11:
				tmpf[tmp]=PitchPID.Params.Kp;
			break;
			case 12:
				tmpf[tmp]=PitchPID.Params.Ki;
			break;
			case 13:
				tmpf[tmp]=PitchPID.Params.Kd;
			break;
			case 14:
				tmpf[tmp]=PitchPID.Params.MeasureTc;
			break;
			case 15:
				tmpf[tmp]=PitchPID.Params.ErrDevTc;
			break;
			case 16:
				tmpf[tmp]=PitchPID.Params.RefTc;
			break;
			case 21:
				tmpf[tmp]=PitchRatePID.Params.Kp;
			break;
			case 22:
				tmpf[tmp]=PitchRatePID.Params.Ki;
			break;
			case 23:
				tmpf[tmp]=PitchRatePID.Params.Kd;
			break;
			case 24:
				tmpf[tmp]=PitchRatePID.Params.MeasureTc;
			break;
			case 25:
				tmpf[tmp]=PitchRatePID.Params.ErrDevTc;
			break;
			case 26:
				tmpf[tmp]=PitchRatePID.Params.Kdd;
			break;
			case 27:
				tmpf[tmp]=PitchRatePID.Params.ErrDDevTc;
			break;
			case 28:
				tmpf[tmp]=PitchRatePID.Params.RefTc;
			break;
			case 31:
				tmpf[tmp]=RollPID.Params.Kp;
			break;
			case 32:
				tmpf[tmp]=RollPID.Params.Ki;
			break;
			case 33:
				tmpf[tmp]=RollPID.Params.Kd;
			break;
			case 34:
				tmpf[tmp]=RollPID.Params.MeasureTc;
			break;
			case 35:
				tmpf[tmp]=RollPID.Params.ErrDevTc;
			break;
			case 36:
				tmpf[tmp]=RollPID.Params.RefTc;
			break;
			case 41:
				tmpf[tmp]=RollRatePID.Params.Kp;
			break;
			case 42:
				tmpf[tmp]=RollRatePID.Params.Ki;
			break;
			case 43:
				tmpf[tmp]=RollRatePID.Params.Kd;
			break;
			case 44:
				tmpf[tmp]=RollRatePID.Params.MeasureTc;
			break;
			case 45:
				tmpf[tmp]=RollRatePID.Params.ErrDevTc;
			break;
			case 46:
				tmpf[tmp]=RollRatePID.Params.Kdd;
			break;
			case 47:
				tmpf[tmp]=RollRatePID.Params.ErrDDevTc;
			break;
			case 48:
				tmpf[tmp]=RollRatePID.Params.RefTc;
			break;
			case 51:
				tmpf[tmp]=YawPID.Params.Kp;
			break;
			case 52:
				tmpf[tmp]=YawPID.Params.Ki;
			break;
			case 53:
				tmpf[tmp]=YawPID.Params.Kd;
			break;
			case 54:
				tmpf[tmp]=YawPID.Params.MeasureTc;
			break;
			case 55:
				tmpf[tmp]=YawPID.Params.ErrDevTc;
			break;
			case 56:
				tmpf[tmp]=YawPID.Params.RefTc;
			break;
			case 61:
				tmpf[tmp]=YawRatePID.Params.Kp;
			break;
			case 62:
				tmpf[tmp]=YawRatePID.Params.Ki;
			break;
			case 63:
				tmpf[tmp]=YawRatePID.Params.Kd;
			break;
			case 64:
				tmpf[tmp]=YawRatePID.Params.MeasureTc;
			break;
			case 65:
				tmpf[tmp]=YawRatePID.Params.ErrDevTc;
			break;
			case 66:
				tmpf[tmp]=YawRatePID.Params.Kdd;
			break;
			case 67:
				tmpf[tmp]=YawRatePID.Params.ErrDDevTc;
			break;
			case 68:
				tmpf[tmp]=YawRatePID.Params.RefTc;
			break;
			
			case 71:
				tmpf[tmp]=AccPID.Params.Kp;
			break;
			case 72:
				tmpf[tmp]=AccPID.Params.Ki;
			break;
			case 73:
				tmpf[tmp]=AccPID.Params.Kd;
			break;
			case 74:
				tmpf[tmp]=AccPID.Params.MeasureTc;
			break;
			case 75:
				tmpf[tmp]=AccPID.Params.ErrDevTc;
			break;
			case 76:
				tmpf[tmp]=AccPID.Params.RefTc;
			break;
			case 77:
				tmpf[tmp]=CtrlSys.accel_output_bias;
			break;
			case 81:
				tmpf[tmp]=HeightPID.Params.Kp;
			break;
			case 82:
				tmpf[tmp]=HeightPID.Params.Ki;
			break;
			case 83:
				tmpf[tmp]=HeightPID.Params.Kd;
			break;
			case 84:
				tmpf[tmp]=HeightPID.Params.MeasureTc;
			break;
			case 85:
				tmpf[tmp]=HeightPID.Params.ErrDevTc;
			break;
			case 86:
				tmpf[tmp]=HeightPID.Params.RefTc;
			break;
		
			case 91:
				tmpf[tmp]=HeightRatePID.Params.Kp;
			break;
			case 92:
				tmpf[tmp]=HeightRatePID.Params.Ki;
			break;
			case 93:
				tmpf[tmp]=HeightRatePID.Params.Kd;
			break;
			case 94:
				tmpf[tmp]=HeightRatePID.Params.MeasureTc;
			break;
			case 95:
				tmpf[tmp]=HeightRatePID.Params.ErrDevTc;
			break;
			case 96:
				tmpf[tmp]=HeightRatePID.Params.RefTc;
			break;
			
			case 200:
				tmpf[tmp]=CtrlSys.bias_pitch;
			break;
			case 201:
				tmpf[tmp]=CtrlSys.bias_roll;
			break;
			case 202:
//				tmpf[tmp]=CtrlSys.bias_throttle;
			break;
			case 203:
				tmpf[tmp]=CtrlSys.duty_min;
			break;
			case 204:
				tmpf[tmp]=CtrlSys.duty_max;
			break;
			
			case 206:
				tmpf[tmp]=CtrlSys.acc_line_z_bias;
			break;
			case 207:
				tmpf[tmp]=CtrlSys.yaw_bias;
			break;
			case 208:
				tmpf[tmp]=CtrlSys.idling_throttle;
			break;
//			case 210:
//				tmpf[tmp]=att_kalman_qr.q_acc;
//			break;
//			case 211:
//				tmpf[tmp]=att_kalman_qr.q_acc_z;
//			break;
//			case 212:
//				tmpf[tmp]=att_kalman_qr.q_rot_speed;
//			break;
//			case 213:
//				tmpf[tmp]=att_kalman_qr.q_rot_speed_z;
//			break;
//			case 214:
//				tmpf[tmp]=att_kalman_qr.q_rot_acc;
//			break;
//			case 215:
//				tmpf[tmp]=att_kalman_qr.q_rot_acc_z;
//			break;
//			case 216:
//				tmpf[tmp]=att_kalman_qr.q_rot_speed_bias;
//			break;
//			case 217:
//				tmpf[tmp]=att_kalman_qr.q_mag;
//			break;
//			case 218:
//				tmpf[tmp]=att_kalman_qr.r_acc;
//			break;
//			case 219:
//				tmpf[tmp]=att_kalman_qr.r_acc_z;
//			break;
//			case 220:
//				tmpf[tmp]=att_kalman_qr.r_gyro;
//			break;
//			case 221:
//				tmpf[tmp]=att_kalman_qr.r_gyro_z;
//			break;
//			case 222:
//				tmpf[tmp]=att_kalman_qr.r_mag;
//			break;
			case 223:
				tmpf[tmp]=alt_filter_qr.q_h;
			break;
			case 224:
				tmpf[tmp]=alt_filter_qr.q_v;
			break;
			case 225:
				tmpf[tmp]=alt_filter_qr.q_a;
			break;
			case 226:
				tmpf[tmp]=alt_filter_qr.q_a_bias;
			break;
			case 227:
				tmpf[tmp]=alt_filter_qr.r_h;
			break;
			case 228:
				tmpf[tmp]=alt_filter_qr.r_a;
			break;
			case 229:
				tmpf[tmp]=alt_filter.acc_line_z_tc;
			break;
		}
		
	}
	DataFrame_SendParams(&txdf1,tmpf,index,params_cnt);
}

//from left to right:sw1-sw8
//				     up-1										down-0
//SW1(b7)			lock										unlock
//SW2(b6)			enable curve						disable curve
//SW3-4(b5-4)			height_close_loop_mode
//SW5(b3)			pitch test							roll test
//SW6-8(b2-0)			        test_mode
void SwitchDecode(u32 data)
{
	static char setup=0;
	int i;
	char v1,v_testmode,beep_flag=0;//养成初始化的好习惯，不要用默认的初始化，串口下载时变量默认值并不为0
//	beep_flag=0;
	v1=(data>>7)&(1);
	if(v1)
	{
		return ;
	}
	v_testmode=(data)&(7);
	if(CtrlSys.test_mode!=v_testmode&&CtrlSys.stop==1)
	{
		CtrlSys.test_mode=v_testmode;
		ControllerReset();
		CtrlSys.stop=1;
		SetMotorStop();
		beep_flag=1;
	}
	
	for(i=3;i<32;i++)
	{
		v1=(data>>i)&(1);
		switch(i)
		{
		//CtrlSys.test_mode:
		//0-normal,1-motor test
		//2-esc_setting,3-esc_no_setting
		//4-double loop,5-single loop
			case 3:
				if(v1!=CtrlSys.pitch_test)
				{
					CtrlSys.pitch_test=v1;
					beep_flag=1;
					ControllerReset();
				}
			break;
			case 4:
				v1=(data>>i)&(3);
				if(CtrlSys.height_close_loop_mode!=v1)
				{//0-open_loop,1-accel_close_loop,2-height_rate_close_loop,3-height_close_loop
					CtrlSys.height_close_loop_mode=v1;
					ControllerReset();
					beep_flag=1;
					
				}
				i++;
			break;
			case 6:
				if(CtrlSys.curve_en!=v1)
				{
					CtrlSys.curve_en=v1;
					beep_flag=1;
				}
			break;
			case 8:
				v1=!v1;
				if(CtrlSys.yaw_close_loop!=v1)
				{
					CtrlSys.yaw_close_loop=v1;
					beep_flag=1;
				}
			break;	
			case 9:
				
			break;	
			case 10:
				
			break;	
			case 11:
				
			break;	
			case 12:
				
			break;	
			case 13:
				
			break;	
			case 14:
				
			break;	
			case 15:
				
			break;	
		}
	}
	if(setup&&beep_flag)
	{
		BEEP_SetStatus(BEEP_STATUS_ACK);
	}
	else if(setup==0)
	{
		setup=1;
	}
	rc_sw_value=data;
}

void KeyDecode(u8 data)
{
	char rate,v1;
//	rate=(rc_sw_value>>3)&1;
	rate=1;
	v1=(rc_sw_value>>7)&(1);
	if(v1)
	{
		return ;
	}
	BEEP_SetStatus(BEEP_STATUS_ACK);
	switch(data)
	{
		case 0:  //stop
			CtrlSys.stop=1;
			ControllerReset();
			SetMotorStop();
		break;
		case 1://key_up 
			if(rate==0)
			{
				CtrlSys.bias_pitch=LimitMinMax_float(CtrlSys.bias_pitch+0.01f,-10,10);
			}
			else
			{
				CtrlSys.bias_pitch=LimitMinMax_float(CtrlSys.bias_pitch+0.1f,-10,10);
			}
		break;
		case 2://key_down 
			if(rate==0)
			{
				CtrlSys.bias_pitch=LimitMinMax_float(CtrlSys.bias_pitch-0.01f,-10,10);
			}
			else
			{
				CtrlSys.bias_pitch=LimitMinMax_float(CtrlSys.bias_pitch-0.1f,-10,10);
			}
		break;
		case 3://key_right 
			if(rate==0)
			{
				CtrlSys.bias_roll=LimitMinMax_float(CtrlSys.bias_roll+0.01f,-10,10);
			}
			else
			{
				CtrlSys.bias_roll=LimitMinMax_float(CtrlSys.bias_roll+0.1f,-10,10);
			}
		break;
		case 4://key_left 
			if(rate==0)
			{
				CtrlSys.bias_roll=LimitMinMax_float(CtrlSys.bias_roll-0.01f,-10,10);
			}
			else
			{
				CtrlSys.bias_roll=LimitMinMax_float(CtrlSys.bias_roll-0.1f,-10,10);
			}
		break;
		case 5: //save
			SetConfig();
		break;
		case 6:  //clear
			CtrlSys.bias_pitch=0;
			CtrlSys.bias_roll=0;
		break;
		case 7:  //
			
		break;
		case 8:  //
			
		break;
		case 9:  //
			
		break;
		case 10:  //no gps
			
		break;
	}
}



void SendIMUCalibParams(char sel)
{
	u32 tmp=0;//,tmp2=0;
	float tmpf[21];
	u8 index[21];
	u8 count;
	if(sel==2)//accel
	{
		count=21;
		for(tmp=0;tmp<21;tmp++)
		{
			if(tmp<3)//0
			{
				index[tmp]=tmp;
				tmpf[tmp]=imu_calib_params.acc_bias[tmp];
			}
			else if(tmp<12)//10
			{
				index[tmp]=tmp+7;
				tmpf[tmp]=imu_calib_params.acc_Ae[tmp-3];
			}
			else if(tmp<21)//20
			{
				index[tmp]=tmp+8;
				tmpf[tmp]=imu_calib_params.acc_coef[tmp-12];
			}
		}
	}
	else if(sel==3)//mag
	{
		count=21;
		for(tmp=0;tmp<21;tmp++)
		{
			if(tmp<3)//0
			{
				index[tmp]=tmp;
				tmpf[tmp]=imu_calib_params.mag_bias[tmp];
			}
			else if(tmp<12)//10
			{
				index[tmp]=tmp+7;
				tmpf[tmp]=imu_calib_params.mag_Ae[tmp-3];
			}
			else if(tmp<21)//20
			{
				index[tmp]=tmp+8;
				tmpf[tmp]=imu_calib_params.mag_coef[tmp-12];
			}
		}
	}
	else if(sel==1)//gyro
	{
		count=3;
		for(tmp=0;tmp<3;tmp++)
		{
			if(tmp<3)//0
			{
				index[tmp]=tmp;
				tmpf[tmp]=imu_calib_params.gyro_bias[tmp];
			}
		}
	}
//	for(tmp=0;tmp<45;tmp++)
//	{
//	
//		if(tmp<3)//0
//		{
//			index[tmp]=tmp;
//			tmpf[tmp]=imu_calib_params.acc_bias[tmp];
//		}
//		else if(tmp<12)//10
//		{
//			index[tmp]=tmp+7;
//			tmpf[tmp]=imu_calib_params.acc_Ae[tmp-3];
//		}
//		else if(tmp<21)//20
//		{
//			index[tmp]=tmp+8;
//			tmpf[tmp]=imu_calib_params.acc_coef[tmp-12];
//		}
//		else if(tmp<24)//30
//		{
//			index[tmp]=tmp+9;
//			tmpf[tmp]=imu_calib_params.mag_bias[tmp-21];
//		}
//		else if(tmp<33)//40
//		{
//			index[tmp]=tmp+16;
//			tmpf[tmp]=imu_calib_params.mag_Ae[tmp-24];
//		}
//		else if(tmp<42)//50
//		{
//			index[tmp]=tmp+17;
//			tmpf[tmp]=imu_calib_params.mag_coef[tmp-33];
//		}
//		else//60
//		{
//			index[tmp]=tmp+18;
//			tmpf[tmp]=imu_calib_params.gyro_bias[tmp-42];
//		}
//		
//	}
	DataFrame_SendParams(&txdf1,tmpf,index,count);
}



