

#ifndef DMA_LIB_H
#define DMA_LIB_H
#include "stm32f4xx.h"
#include "stm32f4xx_dma.h"

void DMA_StartSend(DMA_Stream_TypeDef* DMAy_Streamx,u32 addr,unsigned int send_length);



#endif



