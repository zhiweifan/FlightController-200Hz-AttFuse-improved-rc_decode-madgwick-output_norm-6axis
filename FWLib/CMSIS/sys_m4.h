
//by 艾永恒 (Ai Yongheng 20141101)

#ifndef SYS_M4_H
#define SYS_M4_H     

#include "stm32f4xx.h"
#include "stm32f4xx_rcc.h"

#define TRANSP(n,type)									((type *)(&(n)))
#define TRANS(n,type)										(*((type *)(&(n))))
#define ABS(x)													(((x)>0)?(x):(-(x)))
#define SIGN(x)													(((x)>0)?(1):(-1))
#define LIMIT(x,y)											(((x)>(y))?(y):(((x)<(-(y)))?(-(y)):(x)))
#define LIMIT_UP(x,y)										(((x)>(y))?(y):(x))
#define LIMIT_LOW(x,y)									(((x)<(y))?(y):(x))
#define LIMIT_MIN_MAX(x,min,max)				(((x)<(min))?(min):(((x)>(max))?(max):(x)))


#define BITBAND(addr, bitnum)						((addr & 0xF0000000)+0x2000000+((addr &0xFFFFF)<<5)+(bitnum<<2)) 
#define MEM_ADDR(addr)									(*((volatile unsigned long  *)(addr))) 
#define BIT_ADDR(addr, bitnum)					MEM_ADDR(BITBAND(addr, bitnum)) 

#define GPIOA_ODR_Addr    (GPIOA_BASE+0x14)  
#define GPIOB_ODR_Addr    (GPIOB_BASE+0x14)
#define GPIOC_ODR_Addr    (GPIOC_BASE+0x14)
#define GPIOD_ODR_Addr    (GPIOD_BASE+0x14) 
#define GPIOE_ODR_Addr    (GPIOE_BASE+0x14)
#define GPIOF_ODR_Addr    (GPIOF_BASE+0x14)   
#define GPIOG_ODR_Addr    (GPIOG_BASE+0x14)  
#define GPIOH_ODR_Addr    (GPIOH_BASE+0x14)
#define GPIOI_ODR_Addr    (GPIOI_BASE+0x14)
#define GPIOJ_ODR_Addr    (GPIOJ_BASE+0x14)
#define GPIOK_ODR_Addr    (GPIOK_BASE+0x14)

#define GPIOA_IDR_Addr    (GPIOA_BASE+0x10)
#define GPIOB_IDR_Addr    (GPIOB_BASE+0x10)
#define GPIOC_IDR_Addr    (GPIOC_BASE+0x10)
#define GPIOD_IDR_Addr    (GPIOD_BASE+0x10)
#define GPIOE_IDR_Addr    (GPIOE_BASE+0x10)
#define GPIOF_IDR_Addr    (GPIOF_BASE+0x10)
#define GPIOG_IDR_Addr    (GPIOG_BASE+0x10)
#define GPIOH_IDR_Addr    (GPIOH_BASE+0x10)
#define GPIOI_IDR_Addr    (GPIOI_BASE+0x10)
#define GPIOJ_IDR_Addr    (GPIOJ_BASE+0x10)
#define GPIOK_IDR_Addr    (GPIOK_BASE+0x10)


#define PAOut(n)   BIT_ADDR(GPIOA_ODR_Addr,n)  
#define PAIn(n)    BIT_ADDR(GPIOA_IDR_Addr,n)  

#define PBOut(n)   BIT_ADDR(GPIOB_ODR_Addr,n)  
#define PBIn(n)    BIT_ADDR(GPIOB_IDR_Addr,n)  

#define PCOut(n)   BIT_ADDR(GPIOC_ODR_Addr,n)  
#define PCIn(n)    BIT_ADDR(GPIOC_IDR_Addr,n)  

#define PDOut(n)   BIT_ADDR(GPIOD_ODR_Addr,n)  
#define PDIn(n)    BIT_ADDR(GPIOD_IDR_Addr,n)  

#define PEOut(n)   BIT_ADDR(GPIOE_ODR_Addr,n)  
#define PEIn(n)    BIT_ADDR(GPIOE_IDR_Addr,n)  

#define PFOut(n)   BIT_ADDR(GPIOF_ODR_Addr,n)  
#define PFIn(n)    BIT_ADDR(GPIOF_IDR_Addr,n)  

#define PGOut(n)   BIT_ADDR(GPIOG_ODR_Addr,n)  
#define PGIn(n)    BIT_ADDR(GPIOG_IDR_Addr,n)  

#define PHOut(n)   BIT_ADDR(GPIOH_ODR_Addr,n)  
#define PHIn(n)    BIT_ADDR(GPIOH_IDR_Addr,n)  

#define PIOut(n)   BIT_ADDR(GPIOI_ODR_Addr,n)  
#define PIIn(n)    BIT_ADDR(GPIOI_IDR_Addr,n)  

#define PJOut(n)   BIT_ADDR(GPIOJ_ODR_Addr,n)  
#define PJIn(n)    BIT_ADDR(GPIOJ_IDR_Addr,n)  

#define PKOut(n)   BIT_ADDR(GPIOK_ODR_Addr,n)  
#define PKIn(n)    BIT_ADDR(GPIOK_IDR_Addr,n)  


///RCC
//AHB1
#define RCC_GPIOA(enable)															RCC_AHB1PeriphClockCmd(RCC_AHB1Periph_GPIOA,enable)
#define RCC_GPIOB(enable)															RCC_AHB1PeriphClockCmd(RCC_AHB1Periph_GPIOB,enable)
#define RCC_GPIOC(enable)															RCC_AHB1PeriphClockCmd(RCC_AHB1Periph_GPIOC,enable)
#define RCC_GPIOD(enable)															RCC_AHB1PeriphClockCmd(RCC_AHB1Periph_GPIOD,enable)
#define RCC_GPIOE(enable)															RCC_AHB1PeriphClockCmd(RCC_AHB1Periph_GPIOE,enable)
#define RCC_GPIOF(enable)															RCC_AHB1PeriphClockCmd(RCC_AHB1Periph_GPIOF,enable)
#define RCC_GPIOG(enable)															RCC_AHB1PeriphClockCmd(RCC_AHB1Periph_GPIOG,enable)
#define RCC_GPIOH(enable)															RCC_AHB1PeriphClockCmd(RCC_AHB1Periph_GPIOH,enable)
#define RCC_GPIOI(enable)															RCC_AHB1PeriphClockCmd(RCC_AHB1Periph_GPIOI,enable)
#define RCC_CRC(enable)																RCC_AHB1PeriphClockCmd(RCC_AHB1Periph_CRC,enable)
#define RCC_FLITF(enable)															RCC_AHB1PeriphClockCmd(RCC_AHB1Periph_FLITF,enable)
#define RCC_SRAM1(enable)															RCC_AHB1PeriphClockCmd(RCC_AHB1Periph_SRAM1,enable)
#define RCC_SRAM2(enable)															RCC_AHB1PeriphClockCmd(RCC_AHB1Periph_SRAM2,enable)
#define RCC_BKPSRAM(enable)														RCC_AHB1PeriphClockCmd(RCC_AHB1Periph_BKPSRAM,enable)
#define RCC_CCMDATARAMEN(enable)											RCC_AHB1PeriphClockCmd(RCC_AHB1Periph_CCMDATARAMEN,enable)
#define RCC_DMA1(enable)															RCC_AHB1PeriphClockCmd(RCC_AHB1Periph_DMA1,enable)
#define RCC_DMA2(enable)															RCC_AHB1PeriphClockCmd(RCC_AHB1Periph_DMA2,enable)
#define RCC_ETH_MAC(enable)														RCC_AHB1PeriphClockCmd(RCC_AHB1Periph_ETH_MAC,enable)
#define RCC_ETH_MAC_Tx(enable)												RCC_AHB1PeriphClockCmd(RCC_AHB1Periph_ETH_MAC_Tx,enable)
#define RCC_ETH_MAC_Rx(enable)												RCC_AHB1PeriphClockCmd(RCC_AHB1Periph_ETH_MAC_Rx,enable)
#define RCC_ETH_MAC_PTP(enable)												RCC_AHB1PeriphClockCmd(RCC_AHB1Periph_ETH_MAC_PTP,enable)
#define RCC_OTG_HS(enable)														RCC_AHB1PeriphClockCmd(RCC_AHB1Periph_OTG_HS,enable)
#define RCC_OTG_HS_ULPI(enable)												RCC_AHB1PeriphClockCmd(RCC_AHB1Periph_OTG_HS_ULPI,enable)


//AHB2
#define RCC_DCMI(enable)															RCC_AHB2PeriphClockCmd(RCC_AHB2Periph_DCMI,enable)
#define RCC_CRYP(enable)															RCC_AHB2PeriphClockCmd(RCC_AHB2Periph_CRYP,enable)
#define RCC_HASH(enable)															RCC_AHB2PeriphClockCmd(RCC_AHB2Periph_HASH,enable)
#define RCC_RNG(enable)																RCC_AHB2PeriphClockCmd(RCC_AHB2Periph_RNG,enable)
#define RCC_OTG_FS(enable)														RCC_AHB2PeriphClockCmd(RCC_AHB2Periph_OTG_FS,enable)


//AHB3
#define RCC_FSMC(enable)															RCC_AHB3PeriphClockCmd(RCC_AHB3Periph_FSMC,enable)


//APB1
#define RCC_TIM2(enable)															RCC_APB1PeriphClockCmd(RCC_APB1Periph_TIM2,enable)
#define RCC_TIM3(enable)															RCC_APB1PeriphClockCmd(RCC_APB1Periph_TIM3,enable)
#define RCC_TIM4(enable)															RCC_APB1PeriphClockCmd(RCC_APB1Periph_TIM4,enable)
#define RCC_TIM5(enable)															RCC_APB1PeriphClockCmd(RCC_APB1Periph_TIM5,enable)
#define RCC_TIM6(enable)															RCC_APB1PeriphClockCmd(RCC_APB1Periph_TIM6,enable)
#define RCC_TIM7(enable)															RCC_APB1PeriphClockCmd(RCC_APB1Periph_TIM7,enable)
#define RCC_TIM12(enable)															RCC_APB1PeriphClockCmd(RCC_APB1Periph_TIM12,enable)
#define RCC_TIM13(enable)															RCC_APB1PeriphClockCmd(RCC_APB1Periph_TIM13,enable)
#define RCC_TIM14(enable)															RCC_APB1PeriphClockCmd(RCC_APB1Periph_TIM14,enable)
#define RCC_WWDG(enable)															RCC_APB1PeriphClockCmd(RCC_APB1Periph_WWDG,enable)
#define RCC_SPI2(enable)															RCC_APB1PeriphClockCmd(RCC_APB1Periph_SPI2,enable)
#define RCC_SPI3(enable)															RCC_APB1PeriphClockCmd(RCC_APB1Periph_SPI3,enable)
#define RCC_USART2(enable)														RCC_APB1PeriphClockCmd(RCC_APB1Periph_USART2,enable)
#define RCC_USART3(enable)														RCC_APB1PeriphClockCmd(RCC_APB1Periph_USART3,enable)
#define RCC_UART4(enable)															RCC_APB1PeriphClockCmd(RCC_APB1Periph_UART4,enable)
#define RCC_UART5(enable)															RCC_APB1PeriphClockCmd(RCC_APB1Periph_UART5,enable)
#define RCC_I2C1(enable)															RCC_APB1PeriphClockCmd(RCC_APB1Periph_I2C1,enable)
#define RCC_I2C2(enable)															RCC_APB1PeriphClockCmd(RCC_APB1Periph_I2C2,enable)
#define RCC_I2C3(enable)															RCC_APB1PeriphClockCmd(RCC_APB1Periph_I2C3,enable)
#define RCC_CAN1(enable)															RCC_APB1PeriphClockCmd(RCC_APB1Periph_CAN1,enable)
#define RCC_CAN2(enable)															RCC_APB1PeriphClockCmd(RCC_APB1Periph_CAN2,enable)
#define RCC_PWR(enable)																RCC_APB1PeriphClockCmd(RCC_APB1Periph_PWR,enable)
#define RCC_DAC(enable)																RCC_APB1PeriphClockCmd(RCC_APB1Periph_DAC,enable)


//APB2
#define RCC_TIM1(enable)															RCC_APB2PeriphClockCmd(RCC_APB2Periph_TIM1,enable)
#define RCC_TIM8(enable)															RCC_APB2PeriphClockCmd(RCC_APB2Periph_TIM8,enable)
#define RCC_USART1(enable)														RCC_APB2PeriphClockCmd(RCC_APB2Periph_USART1,enable)
#define RCC_USART6(enable)														RCC_APB2PeriphClockCmd(RCC_APB2Periph_USART6,enable)
#define RCC_ADC(enable)																RCC_APB2PeriphClockCmd(RCC_APB2Periph_ADC,enable)
#define RCC_ADC1(enable)															RCC_APB2PeriphClockCmd(RCC_APB2Periph_ADC1,enable)
#define RCC_ADC2(enable)															RCC_APB2PeriphClockCmd(RCC_APB2Periph_ADC2,enable)
#define RCC_ADC3(enable)															RCC_APB2PeriphClockCmd(RCC_APB2Periph_ADC3,enable)
#define RCC_SDIO(enable)															RCC_APB2PeriphClockCmd(RCC_APB2Periph_SDIO,enable)
#define RCC_SPI1(enable)															RCC_APB2PeriphClockCmd(RCC_APB2Periph_SPI1,enable)
#define RCC_SYSCFG(enable)														RCC_APB2PeriphClockCmd(RCC_APB2Periph_SYSCFG,enable)
#define RCC_TIM9(enable)															RCC_APB2PeriphClockCmd(RCC_APB2Periph_TIM9,enable)
#define RCC_TIM10(enable)															RCC_APB2PeriphClockCmd(RCC_APB2Periph_TIM10,enable)
#define RCC_TIM11(enable)															RCC_APB2PeriphClockCmd(RCC_APB2Periph_TIM11,enable)






#endif


