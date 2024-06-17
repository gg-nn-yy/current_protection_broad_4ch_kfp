#ifndef __RS485_DRIVER_H__
#define __RS485_DRIVER_H__

/* ======================== 头文件 ========================================== */
#include <stdbool.h>

#include "stm32f0xx.h"



/* ======================== 宏定义 ========================================== */



/* ======================== 自定义类型 ====================================== */
/* rs485模块硬件信息类型定义 */
typedef struct {
	uint32_t				rcc;
	GPIO_TypeDef*			gpio;
			
	USART_TypeDef*			uart;
	uint32_t				uart_rcc;
	uint16_t				tx_pin;
	uint16_t				rx_pin;
	uint8_t					irq;
	
} rs485_hw_info_t;



/* ======================== 接口声明区 ====================================== */
void vRs485_DriverInit(void);
void vRs485_TestTask(void);


#endif /* __RS485_DRIVER_H__ */
