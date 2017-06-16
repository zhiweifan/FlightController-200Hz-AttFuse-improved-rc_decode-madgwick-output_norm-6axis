#ifndef __USART1_H__
#define __USART1_H__


#include "stm32f4xx.h"
#include "stm32f4xx_usart.h"

#define   USART1_TX_DMA_BUFFSIZE				2000
#define   USART1_RX_DMA_BUFFSIZE				2000


#define   USART1_TX_DMA_STREAM     DMA2_Stream7

#define   USART1_RX_DMA_STREAM     DMA2_Stream5


#define		USART1_BAUDRATE						115200




void USART1_Config(void);


extern volatile u8 USART1_DMA_SendBuff[USART1_TX_DMA_BUFFSIZE];
extern volatile u8 USART1_DMA_RecvBuff[USART1_RX_DMA_BUFFSIZE];

#endif




