


#include "dataframe_lib_m4.h"
#include "stm32f4xx_dma.h"
#include "stm32f4xx.h"

u16 GetIndex(u16 a,u16 max)
{
	if(a<max)
	{
	 	return a;
	}
	else
	{
	 	a=a-max;
		return GetIndex(a,max);
	}
}
u16 Float2Halfword(float a,float abs_max,char is_signed)
{
	int32_t tmp=0;
	u16 res=0;
	if(abs_max<=0)
	{
		return 0;
	}
	if(is_signed)
	{
		tmp=a*32768/abs_max;
		if(tmp>32767)
		{
			tmp=32767;
		}
		else if(tmp<-32768)
		{
			tmp=-32768;
		}
		res=(s16)tmp;
	}
	else
	{
		tmp=a*65535/abs_max;
		if(tmp>65535)
		{
			tmp=65535;
		}
		else if(tmp<0)
		{
			tmp=0;
		}
		res=(u32)tmp;
	}
	
	return res;
}

u8 Float2Byte(float a,float abs_max,char is_signed)
{
	int32_t tmp=0;
	u8 res=0;
	if(abs_max<=0)
	{
		return 0;
	}
	if(is_signed)
	{
		tmp=a*127/abs_max;
		if(tmp>127)
		{
			tmp=127;
		}
		else if(tmp<-128)
		{
			tmp=-128;
		}
		res=(s8)tmp;
	}
	else
	{
		tmp=a*255/abs_max;
		if(tmp>255)
		{
			tmp=255;
		}
		else if(tmp<0)
		{
			tmp=0;
		}
		res=(u32)tmp;
	}
	
	return res;
}
float Halfword2Float(u16 a,float abs_max,char is_signed)
{
	if(is_signed)
	{
		return ((s16)a)*abs_max/32768;
	}
	else
	{
		return a*abs_max/65536;
	}
}
float Byte2Float(u8 a,float abs_max,char is_signed)
{
	if(is_signed)
	{
		return ((s8)a)*abs_max/128;
	}
	else
	{
		return a*abs_max/256;
	}
}
float Word2Float(u32 a,float abs_max,char is_signed)
{
	if(is_signed)
	{
		return ((s32)a)*abs_max/0x7fffffff;
	}
	else
	{
		return a*abs_max/0xffffffff;
	}
}
double Word2Double(u32 a,double abs_max,char is_signed)
{
	if(is_signed)
	{
		return ((s32)a)*abs_max/0x7fffffff;
	}
	else
	{
		return a*abs_max/0xffffffff;
	}
}
float FloatWord2Float(u32 a)
{
	float tmp;
	tmp=*((float*)(&a));
	return tmp;
}
u32 Float2FloatWord(float a)
{
	
	return *((u32*)(&a));
}
//成功返回1
char DataFrame_CheckFifo(RxDataFrame_TypeDef* rxdf)
{
	u16 pw=0,pr=0,i=0;
	u16 tmp=0;
	u8 tmp2=0,jy=0;
  
	pw=rxdf->WriteIndex;
	pr=rxdf->ReadIndex;
	if(pr>pw)
	{
	 	pw+=rxdf->FifoSize;
	}
	if(pw<pr+7)
	{
		return 0;
	}

	while(pw>=(pr+7))
	{
	 	if(rxdf->FIFO[GetIndex(pr,rxdf->FifoSize)]==0x5a)    //检测到帧头
		{
			tmp=rxdf->FIFO[GetIndex(pr+2,rxdf->FifoSize)];
			tmp2=rxdf->FIFO[GetIndex(pr+1,rxdf->FifoSize)];
			if(tmp2&(1<<7))
			{
				tmp=tmp*5;
			}
			if(tmp>(rxdf->DataBytesMax))
			{
				rxdf->ReadIndex=GetIndex(pr+1,rxdf->FifoSize);	
				pr++;
				continue;
			}
			if(pr+tmp+5>=pw||rxdf->FIFO[GetIndex(pr+tmp+4,rxdf->FifoSize)]!='\r'||rxdf->FIFO[GetIndex(pr+tmp+5,rxdf->FifoSize)]!='\n')
			{
				if(pr+tmp+5>=pw)
				{
						return 0;
				}
				else
				{
					rxdf->ReadIndex=GetIndex(pr+1,rxdf->FifoSize);	
					pr++;
					continue;
	
				}
				
			}
			else    
			{					
				
				for(i=0;i<tmp+3;i++)
				{
					jy+=rxdf->FIFO[GetIndex(pr+i,rxdf->FifoSize)];
				}
				if(jy!=rxdf->FIFO[GetIndex(pr+tmp+3,rxdf->FifoSize)])
				{
					rxdf->ReadIndex=GetIndex(pr+tmp+6,rxdf->FifoSize);	
					return 0;//2; 
				}
			
				for(i=0;i<tmp;i++)
				{
					rxdf->pData[i]=rxdf->FIFO[GetIndex(pr+i+3,rxdf->FifoSize)];
				}
				rxdf->FrameType=tmp2;
				rxdf->FrameCnt=rxdf->FIFO[GetIndex(pr+2,rxdf->FifoSize)];
				rxdf->ReadIndex=GetIndex(pr+tmp+6,rxdf->FifoSize);
				return 1;
			}
		}	
		else          //如果不是帧头
		{
			rxdf->ReadIndex=GetIndex(pr+1,rxdf->FifoSize);
			pr++;
			continue;
		}
	}
	return 0;
	
}


char DataFrame_WriteFifo(TxDataFrame_TypeDef* txdf)
{
	u16 tmp=0,i=0,freelength=0,pr=0,pw=0;
	u8 jy=0;
	if(txdf->FrameType&(1<<7))
	{
		tmp=txdf->FrameCnt*5;
		if(tmp!=txdf->DataBytes||(tmp+6)>txdf->FifoSize )
		{
			return 0;
		}
		
	}
	else
	{
		tmp=txdf->FrameCnt;
		if(tmp!=txdf->DataBytes||(tmp+6)>txdf->FifoSize)
		{
			return 0;
		}
		
	}
	pr=txdf->ReadIndex;
	pw=txdf->WriteIndex;
	
//	if(pr==pw)
//	{
//		return 0;
//	}
//	else if(pr<pw)
	if(pr<=pw)
	{
		freelength=pr+(txdf->FifoSize-pw);
	}
	else
	{
		freelength=pr-pw;
	}
	if(freelength<(txdf->DataBytes+6))
	{
		return 0;
	}
	
	txdf->FIFO[GetIndex(pw,txdf->FifoSize)]=0x5a;
	jy=0x5a;
	txdf->FIFO[GetIndex(pw+1,txdf->FifoSize)]=txdf->FrameType;
	jy+=txdf->FrameType;
	txdf->FIFO[GetIndex(pw+2,txdf->FifoSize)]=txdf->FrameCnt;
	jy+=txdf->FrameCnt;
	for(i=0;i<tmp;i++)
	{
		txdf->FIFO[GetIndex(pw+i+3,txdf->FifoSize)]=txdf->pData[i];
		jy+=txdf->pData[i];
	}
	txdf->FIFO[GetIndex(pw+tmp+3,txdf->FifoSize)]=jy;
	txdf->FIFO[GetIndex(pw+tmp+4,txdf->FifoSize)]='\r';
	txdf->FIFO[GetIndex(pw+tmp+5,txdf->FifoSize)]='\n';
	txdf->WriteIndex=GetIndex(pw+txdf->DataBytes+6,txdf->FifoSize);
	return 1;
	
}

#define DMA_Stream0_IT_MASK     (uint32_t)(DMA_LISR_FEIF0 | DMA_LISR_DMEIF0 | \
                                           DMA_LISR_TEIF0 | DMA_LISR_HTIF0 | \
                                           DMA_LISR_TCIF0)
#define DMA_Stream1_IT_MASK     (uint32_t)(DMA_Stream0_IT_MASK << 6)
#define DMA_Stream2_IT_MASK     (uint32_t)(DMA_Stream0_IT_MASK << 16)
#define DMA_Stream3_IT_MASK     (uint32_t)(DMA_Stream0_IT_MASK << 22)
#define DMA_Stream4_IT_MASK     (uint32_t)(DMA_Stream0_IT_MASK | (uint32_t)0x20000000)
#define DMA_Stream5_IT_MASK     (uint32_t)(DMA_Stream1_IT_MASK | (uint32_t)0x20000000)
#define DMA_Stream6_IT_MASK     (uint32_t)(DMA_Stream2_IT_MASK | (uint32_t)0x20000000)
#define DMA_Stream7_IT_MASK     (uint32_t)(DMA_Stream3_IT_MASK | (uint32_t)0x20000000)
void DataFrame_UpdateReadIndex(TxDataFrame_TypeDef* pTxDataFrame)
{
//	DMA_InitTypeDef   dma;
//	u32 cr=0;
//	static u16 readindex=0,num=0;
	u32 pr=0;
	if(pTxDataFrame->ReadIndex==pTxDataFrame->WriteIndex)
	{
		pTxDataFrame->DMA_TransNum=0;
		return;
	}
	pr=DMA_GetCurrDataCounter(pTxDataFrame->DMA_Stream);
	if(pr)
	{
		pr=pTxDataFrame->DMA_ReadIndex+pTxDataFrame->DMA_TransNum-pr;
		if(pr>=pTxDataFrame->FifoSize)
		{
			pr-=pTxDataFrame->FifoSize;
		}
		pTxDataFrame->ReadIndex=pr;
		return;
	}
	else
	{
		pr=pTxDataFrame->DMA_ReadIndex+pTxDataFrame->DMA_TransNum;
		if(pr>=pTxDataFrame->FifoSize)
		{
			pr-=pTxDataFrame->FifoSize;
		}
//		pTxDataFrame->ReadIndex=pr;
		pTxDataFrame->DMA_ReadIndex=pr;
		pTxDataFrame->DMA_TransNum=0;
	}
	if(pTxDataFrame->WriteIndex<pTxDataFrame->DMA_ReadIndex)
	{
		pTxDataFrame->DMA_TransNum=pTxDataFrame->FifoSize-pTxDataFrame->DMA_ReadIndex;
	}
	else
	{
		pTxDataFrame->DMA_TransNum=pTxDataFrame->WriteIndex-pTxDataFrame->DMA_ReadIndex;
	}
	
	
	DMA_Cmd(pTxDataFrame->DMA_Stream, DISABLE );  //关闭USART1 TX DMA1 所指示的通道
	while (DMA_GetCmdStatus(pTxDataFrame->DMA_Stream) != DISABLE){}	//确保DMA可以被设置
	if (pTxDataFrame->DMA_Stream == DMA1_Stream0)
  {
    /* Reset interrupt pending bits for DMA1 Stream0 */
    DMA1->LIFCR = DMA_Stream0_IT_MASK;
  }
  else if (pTxDataFrame->DMA_Stream == DMA1_Stream1)
  {
    /* Reset interrupt pending bits for DMA1 Stream1 */
    DMA1->LIFCR = DMA_Stream1_IT_MASK;
  }
  else if (pTxDataFrame->DMA_Stream == DMA1_Stream2)
  {
    /* Reset interrupt pending bits for DMA1 Stream2 */
    DMA1->LIFCR = DMA_Stream2_IT_MASK;
  }
  else if (pTxDataFrame->DMA_Stream == DMA1_Stream3)
  {
    /* Reset interrupt pending bits for DMA1 Stream3 */
    DMA1->LIFCR = DMA_Stream3_IT_MASK;
  }
  else if (pTxDataFrame->DMA_Stream == DMA1_Stream4)
  {
    /* Reset interrupt pending bits for DMA1 Stream4 */
    DMA1->HIFCR = DMA_Stream4_IT_MASK;
  }
  else if (pTxDataFrame->DMA_Stream == DMA1_Stream5)
  {
    /* Reset interrupt pending bits for DMA1 Stream5 */
    DMA1->HIFCR = DMA_Stream5_IT_MASK;
  }
  else if (pTxDataFrame->DMA_Stream == DMA1_Stream6)
  {
    /* Reset interrupt pending bits for DMA1 Stream6 */
    DMA1->HIFCR = (uint32_t)DMA_Stream6_IT_MASK;
  }
  else if (pTxDataFrame->DMA_Stream== DMA1_Stream7)
  {
    /* Reset interrupt pending bits for DMA1 Stream7 */
    DMA1->HIFCR = DMA_Stream7_IT_MASK;
  }
  else if (pTxDataFrame->DMA_Stream == DMA2_Stream0)
  {
    /* Reset interrupt pending bits for DMA2 Stream0 */
    DMA2->LIFCR = DMA_Stream0_IT_MASK;
  }
  else if (pTxDataFrame->DMA_Stream == DMA2_Stream1)
  {
    /* Reset interrupt pending bits for DMA2 Stream1 */
    DMA2->LIFCR = DMA_Stream1_IT_MASK;
  }
  else if (pTxDataFrame->DMA_Stream== DMA2_Stream2)
  {
    /* Reset interrupt pending bits for DMA2 Stream2 */
    DMA2->LIFCR = DMA_Stream2_IT_MASK;
  }
  else if (pTxDataFrame->DMA_Stream== DMA2_Stream3)
  {
    /* Reset interrupt pending bits for DMA2 Stream3 */
    DMA2->LIFCR = DMA_Stream3_IT_MASK;
  }
  else if (pTxDataFrame->DMA_Stream == DMA2_Stream4)
  {
    /* Reset interrupt pending bits for DMA2 Stream4 */
    DMA2->HIFCR = DMA_Stream4_IT_MASK;
  }
  else if (pTxDataFrame->DMA_Stream== DMA2_Stream5)
  {
    /* Reset interrupt pending bits for DMA2 Stream5 */
    DMA2->HIFCR = DMA_Stream5_IT_MASK;
  }
  else if (pTxDataFrame->DMA_Stream== DMA2_Stream6)
  {
    /* Reset interrupt pending bits for DMA2 Stream6 */
    DMA2->HIFCR = DMA_Stream6_IT_MASK;
  }
  else 
  {
    if (pTxDataFrame->DMA_Stream == DMA2_Stream7)
    {
      /* Reset interrupt pending bits for DMA2 Stream7 */
      DMA2->HIFCR = DMA_Stream7_IT_MASK;
    }
  }
	
 	DMA_SetCurrDataCounter(pTxDataFrame->DMA_Stream,pTxDataFrame->DMA_TransNum);//DMA通道的DMA缓存的大小;使用此函数必须先关闭相应的DMA通道
	pTxDataFrame->DMA_Stream->M0AR=(u32)(&pTxDataFrame->FIFO[pTxDataFrame->DMA_ReadIndex]);
 	DMA_Cmd(pTxDataFrame->DMA_Stream, ENABLE);  //使能USART1 TX DMA1 所指示的通道
	

	
	pTxDataFrame->ReadIndex=pr;
}


















