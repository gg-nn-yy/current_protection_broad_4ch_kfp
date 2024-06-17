#ifndef __LED_DRIVER_H__
#define __LED_DRIVER_H__

/* ======================== ͷ�ļ� ========================================== */
#include <stdint.h>
#include "stm32f0xx.h"



/* ======================== �궨�� ========================================== */



/* ======================== �Զ������� ====================================== */
/* ledӲ����Ϣ���Ͷ��� */
typedef struct {
	uint32_t		rcc;
	GPIO_TypeDef*	gpio;
	uint16_t		pin;
} led_hw_info_t;


/* led���ö�� */
enum {
	LED1 = 0,
	LED2 = 1,
	LED3 = 2,
	LED4 = 3,
	LED5 = 4,
	LED6 = 5,
	LED7 = 6,
	LED8 = 7,
};


/* ======================== �ӿ������� ====================================== */
void vLed_DriverInit(void);       /* led������ʼ�� */

void vLed_TurnOn(uint8_t LedNo);  /* led��         */

void vLed_TurnOff(uint8_t LedNo); /* led��         */

void vLed_Toggle(uint8_t LedNo);  /* led��ת       */

void vLed_TestTask1(void);
void vLed_TestTask2(void);
void vLed_TestTask3(void);
void vLed_TestTask4(void);
void vLed_TestTask5(void);
void vLed_TestTask6(void);
void vLed_TestTask7(void);
void vLed_TestTask8(void);

#endif /* __LED_DRIVER_H__ */
