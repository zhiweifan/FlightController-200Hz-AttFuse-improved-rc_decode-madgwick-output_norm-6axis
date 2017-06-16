#ifndef __USART2_H__
#define __USART2_H__


#include "stm32f4xx.h"
#include "stm32f4xx_usart.h"

#define   USART2_TX_DMA_BUFFSIZE				300
#define   USART2_RX_DMA_BUFFSIZE				500


#define   USART2_TX_DMA_STREAM     DMA1_Stream6

#define   USART2_RX_DMA_STREAM     DMA1_Stream5


#define		USART2_BAUDRATE						921600//115200




void USART2_Config(void);


extern volatile u8 USART2_DMA_SendBuff[USART2_TX_DMA_BUFFSIZE];
extern volatile u8 USART2_DMA_RecvBuff[USART2_RX_DMA_BUFFSIZE];

#endif




