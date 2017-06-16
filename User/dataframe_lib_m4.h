

#ifndef DATAFRAME_LIB_M4_H
#define DATAFRAME_LIB_M4_H

#include "stm32f4xx.h"


typedef struct
{
volatile u8 FrameType;			//最高bit为索引位，索引位为1则每5字节为一个单元，否则每4个字节为一个单元
volatile u8 FrameCnt;				//索引位为1则等于单元数，否则等于字节数
volatile u16 DataBytesMax;				//最大字节数
volatile u8* pData;					//参数存储基地址
volatile u8* FIFO;					//接收缓冲区基地址
volatile u16 FifoSize;			//接收缓冲区大小
volatile u16 WriteIndex;		
volatile u16 ReadIndex;
DMA_Stream_TypeDef * DMA_Stream;
} RxDataFrame_TypeDef;

typedef struct
{
volatile u8 FrameType;
volatile u8 FrameCnt;				//要发送帧的数据段的字节数或单元数
volatile u16 DataBytes;			//发送数据的字节数
volatile u8* pData;
volatile u8* FIFO;
volatile u16 FifoSize;
volatile u16 WriteIndex;		
volatile u16 ReadIndex;
DMA_Stream_TypeDef * DMA_Stream;
volatile u16 DMA_ReadIndex;
volatile u16 DMA_TransNum;
} TxDataFrame_TypeDef;


void DataFrame_Init(void);

u32 Float2FloatWord(float a);
u16 Float2Halfword(float a,float abs_max,char is_signed);
u8 Float2Byte(float a,float abs_max,char is_signed);
float Halfword2Float(u16 a,float abs_max,char is_signed);
float Byte2Float(u8 a,float abs_max,char is_signed);
float Word2Float(u32 a,float abs_max,char is_signed);
double Word2Double(u32 a,double abs_max,char is_signed);
float FloatWord2Float(u32 a);

void DataFrame_UpdateReadIndex(TxDataFrame_TypeDef* pTxDataFrame);
char DataFrame_CheckFifo(RxDataFrame_TypeDef* rxdf);
char DataFrame_WriteFifo(TxDataFrame_TypeDef* txdf);


#endif 



