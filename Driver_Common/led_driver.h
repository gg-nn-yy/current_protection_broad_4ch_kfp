#ifndef __LED_DRIVER_H__
#define __LED_DRIVER_H__

/* ======================== 头文件 ========================================== */
#include <stdint.h>
#include "stm32f0xx.h"



/* ======================== 宏定义 ========================================== */



/* ======================== 自定义类型 ====================================== */
/* led硬件信息类型定义 */
typedef struct {
	uint32_t		rcc;
	GPIO_TypeDef*	gpio;
	uint16_t		pin;
} led_hw_info_t;


/* led编号枚举 */
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


/* ======================== 接口声明区 ====================================== */
void vLed_DriverInit(void);       /* led驱动初始化 */

void vLed_TurnOn(uint8_t LedNo);  /* led开         */

void vLed_TurnOff(uint8_t LedNo); /* led关         */

void vLed_Toggle(uint8_t LedNo);  /* led翻转       */

void vLed_TestTask1(void);
void vLed_TestTask2(void);
void vLed_TestTask3(void);
void vLed_TestTask4(void);
void vLed_TestTask5(void);
void vLed_TestTask6(void);
void vLed_TestTask7(void);
void vLed_TestTask8(void);

#endif /* __LED_DRIVER_H__ */
