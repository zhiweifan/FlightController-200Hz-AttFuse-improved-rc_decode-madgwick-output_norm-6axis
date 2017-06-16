#include "usart2.h"
#include "stdio.h"
#include "sys_m4.h"

volatile u8 USART2_DMA_SendBuff[USART2_TX_DMA_BUFFSIZE]={0};
volatile u8 USART2_DMA_RecvBuff[USART2_RX_DMA_BUFFSIZE]={0};


void USART2_Config(void)
{
	USART_InitTypeDef usart;
	GPIO_InitTypeDef gpio;
	DMA_InitTypeDef   dma;

	RCC_GPIOA(ENABLE);
	RCC_USART2(ENABLE);
	RCC_DMA1(ENABLE);

	GPIO_PinAFConfig(GPIOA,GPIO_PinSource2 ,GPIO_AF_USART2);
	GPIO_PinAFConfig(GPIOA,GPIO_PinSource3 ,GPIO_AF_USART2);
	
	gpio.GPIO_Pin = GPIO_Pin_2|GPIO_Pin_3;
	gpio.GPIO_Mode = GPIO_Mode_AF;
	gpio.GPIO_OType = GPIO_OType_PP;
	gpio.GPIO_Speed = GPIO_Speed_50MHz;
	gpio.GPIO_PuPd = GPIO_PuPd_UP;
	GPIO_Init(GPIOA,&gpio);


	USART_DeInit(USART2);
	usart.USART_BaudRate = USART2_BAUDRATE;
	usart.USART_WordLength = USART_WordLength_9b;
	usart.USART_StopBits = USART_StopBits_1;
	usart.USART_Parity = USART_Parity_Even;
	usart.USART_Mode = USART_Mode_Rx | USART_Mode_Tx;
	usart.USART_HardwareFlowControl = USART_HardwareFlowControl_None;   
	USART_Init(USART2,&usart);
//		USART_ITConfig(USART2, USART_IT_RXNE, ENABLE);
	USART_Cmd(USART2,ENABLE);
	USART_ClearFlag(USART2, USART_FLAG_TC);
	
	
	
	USART_DMACmd(USART2,USART_DMAReq_Rx,ENABLE);
	USART_DMACmd(USART2,USART_DMAReq_Tx,ENABLE);
	
	DMA_Cmd(USART2_TX_DMA_STREAM, DISABLE ); 
	DMA_Cmd(USART2_RX_DMA_STREAM, DISABLE ); 
	//USART2_TX_DMA
	dma.DMA_Channel=DMA_Channel_4;
	dma.DMA_PeripheralBaseAddr = (uint32_t)&(USART2->DR);
	dma.DMA_Memory0BaseAddr = (uint32_t)USART2_DMA_SendBuff;   
	dma.DMA_DIR = DMA_DIR_MemoryToPeripheral;
	dma.DMA_BufferSize = 0;
	dma.DMA_PeripheralInc = DMA_PeripheralInc_Disable;
	dma.DMA_MemoryInc = DMA_MemoryInc_Enable;
	dma.DMA_PeripheralDataSize = DMA_PeripheralDataSize_Byte;
	dma.DMA_MemoryDataSize = DMA_MemoryDataSize_Byte;
	dma.DMA_Mode = DMA_Mode_Normal;
	dma.DMA_Priority = DMA_Priority_Medium;
	dma.DMA_FIFOMode = DMA_FIFOMode_Disable;
	dma.DMA_FIFOThreshold = DMA_FIFOThreshold_Full;
	dma.DMA_MemoryBurst = DMA_MemoryBurst_Single;
	dma.DMA_PeripheralBurst = DMA_PeripheralBurst_Single;
	DMA_Init(USART2_TX_DMA_STREAM,&dma);
	DMA_SetCurrDataCounter(USART2_TX_DMA_STREAM,0);
//    DMA_ITConfig(DMA1_Channel6, DMA_IT_TC, ENABLE);		
	
	//USART2_RX_DMA
	dma.DMA_Channel=DMA_Channel_4;
	dma.DMA_PeripheralBaseAddr = (uint32_t)&(USART2->DR);
	dma.DMA_Memory0BaseAddr = (uint32_t)USART2_DMA_RecvBuff;   
	dma.DMA_DIR = DMA_DIR_PeripheralToMemory;
	dma.DMA_BufferSize =USART2_RX_DMA_BUFFSIZE;
	dma.DMA_PeripheralInc = DMA_PeripheralInc_Disable;
	dma.DMA_MemoryInc = DMA_MemoryInc_Enable;
	dma.DMA_PeripheralDataSize = DMA_PeripheralDataSize_Byte;
	dma.DMA_MemoryDataSize = DMA_MemoryDataSize_Byte;
	dma.DMA_Mode = DMA_Mode_Circular;
	dma.DMA_Priority = DMA_Priority_Medium;
	dma.DMA_FIFOMode = DMA_FIFOMode_Disable;
	dma.DMA_FIFOThreshold = DMA_FIFOThreshold_Full;
	dma.DMA_MemoryBurst = DMA_MemoryBurst_Single;
	dma.DMA_PeripheralBurst = DMA_PeripheralBurst_Single;
	DMA_Init(USART2_RX_DMA_STREAM,&dma);
	DMA_Cmd(USART2_RX_DMA_STREAM, ENABLE ); 
   
}


//int fputc(int ch, FILE *f)
//{
//    while (USART_GetFlagStatus(USART2,USART_FLAG_TC) == RESET);
//    USART_SendData(USART2, (uint8_t)ch);    
//    return ch;
//}


