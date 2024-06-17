#ifndef __DELAY_H__
#define __DELAY_H__

/* ======================== ͷ�ļ� ========================================== */
#include "stm32f0xx.h"



/* ======================== �궨�� ========================================== */
//�Ĵ�������ַ
#define    DWT_CR        (*(volatile uint32_t*)(0xE0001000))
#define    DWT_CYCCNT    (*(volatile uint32_t*)(0xE0001004))
#define    DEM_CR        (*(volatile uint32_t*)(0xE000EDFC))
 
//������ʹ��λ
#define    DEM_CR_TRCENA       (1 << 24)
#define    DWT_CR_CYCCNTENA    (1 << 0)



/* ======================== �Զ������� ====================================== */



/* ======================== �ӿ������� ====================================== */
#if 0
void vDelayInit(void);
void vDelayNus(uint32_t nUs);
#endif
void vDelayNms(uint32_t nMs);


#endif /* __DELAY_H__ */
