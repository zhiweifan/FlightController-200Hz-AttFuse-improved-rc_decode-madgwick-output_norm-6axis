

#ifndef DATAFRAME_H
#define DATAFRAME_H

#include "stm32f4xx.h"
#include "dataframe_lib_m4.h"

void DataFrame_Init(void);
char DataFrame_SendParams(TxDataFrame_TypeDef* pTxDataFrame,float* tosend,u8* index,u8 num);
char DataFrame_GetData(RxDataFrame_TypeDef* pRxDataFrame);
char DataFrame_SendData(TxDataFrame_TypeDef* pTxDataFrame);

extern RxDataFrame_TypeDef rxdf1,rxdf2;
extern TxDataFrame_TypeDef txdf1,txdf2;

#endif 



