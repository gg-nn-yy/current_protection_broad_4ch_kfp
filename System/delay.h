#ifndef __DELAY_H__
#define __DELAY_H__

/* ======================== 头文件 ========================================== */
#include "stm32f0xx.h"



/* ======================== 宏定义 ========================================== */
//寄存器基地址
#define    DWT_CR        (*(volatile uint32_t*)(0xE0001000))
#define    DWT_CYCCNT    (*(volatile uint32_t*)(0xE0001004))
#define    DEM_CR        (*(volatile uint32_t*)(0xE000EDFC))
 
//定义需使能位
#define    DEM_CR_TRCENA       (1 << 24)
#define    DWT_CR_CYCCNTENA    (1 << 0)



/* ======================== 自定义类型 ====================================== */



/* ======================== 接口声明区 ====================================== */
#if 0
void vDelayInit(void);
void vDelayNus(uint32_t nUs);
#endif
void vDelayNms(uint32_t nMs);


#endif /* __DELAY_H__ */
