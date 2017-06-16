#include "usart1.h"
#include "stdio.h"
#include "sys_m4.h"

volatile u8 USART1_DMA_SendBuff[USART1_TX_DMA_BUFFSIZE]={0};
volatile u8 USART1_DMA_RecvBuff[USART1_RX_DMA_BUFFSIZE]={0};


void USART1_Config(void)
{
	USART_InitTypeDef usart;
	GPIO_InitTypeDef gpio;
	DMA_InitTypeDef   dma;

	RCC_GPIOA(ENABLE);
	RCC_USART1(ENABLE);
	RCC_DMA2(ENABLE);

	GPIO_PinAFConfig(GPIOA,GPIO_PinSource9 ,GPIO_AF_USART1);
	GPIO_PinAFConfig(GPIOA,GPIO_PinSource10 ,GPIO_AF_USART1);
	
	gpio.GPIO_Pin = GPIO_Pin_9|GPIO_Pin_10;
	gpio.GPIO_Mode = GPIO_Mode_AF;
	gpio.GPIO_OType = GPIO_OType_PP;
	gpio.GPIO_Speed = GPIO_Speed_50MHz;
	gpio.GPIO_PuPd = GPIO_PuPd_UP;
	GPIO_Init(GPIOA,&gpio);


	USART_DeInit(USART1);
	usart.USART_BaudRate = USART1_BAUDRATE;
	usart.USART_WordLength = USART_WordLength_9b;
	usart.USART_StopBits = USART_StopBits_1;
	usart.USART_Parity = USART_Parity_Even;
	usart.USART_Mode = USART_Mode_Rx | USART_Mode_Tx;
	usart.USART_HardwareFlowControl = USART_HardwareFlowControl_None;   
	USART_Init(USART1,&usart);
//		USART_ITConfig(USART1, USART_IT_RXNE, ENABLE);
	USART_Cmd(USART1,ENABLE);
	USART_ClearFlag(USART1, USART_FLAG_TC);
	
	
	
	USART_DMACmd(USART1,USART_DMAReq_Rx,ENABLE);
	USART_DMACmd(USART1,USART_DMAReq_Tx,ENABLE);
	
	DMA_Cmd(USART1_TX_DMA_STREAM, DISABLE ); 
	DMA_Cmd(USART1_RX_DMA_STREAM, DISABLE ); 
	//USART1_TX_DMA
	dma.DMA_Channel=DMA_Channel_4;
	dma.DMA_PeripheralBaseAddr = (uint32_t)&(USART1->DR);
	dma.DMA_Memory0BaseAddr = (uint32_t)USART1_DMA_SendBuff;   
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
	DMA_Init(USART1_TX_DMA_STREAM,&dma);
	DMA_SetCurrDataCounter(USART1_TX_DMA_STREAM,0);
	
	
	//USART1_RX_DMA
	dma.DMA_Channel=DMA_Channel_4;
	dma.DMA_PeripheralBaseAddr = (uint32_t)&(USART1->DR);
	dma.DMA_Memory0BaseAddr = (uint32_t)USART1_DMA_RecvBuff;   
	dma.DMA_DIR = DMA_DIR_PeripheralToMemory;
	dma.DMA_BufferSize =USART1_RX_DMA_BUFFSIZE;
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
	DMA_Init(USART1_RX_DMA_STREAM,&dma);
	DMA_Cmd(USART1_RX_DMA_STREAM, ENABLE ); 
   
}


int fputc(int ch, FILE *f)
{
    while (USART_GetFlagStatus(USART1,USART_FLAG_TC) == RESET);
    USART_SendData(USART1, (uint8_t)ch);    
    return ch;
}


