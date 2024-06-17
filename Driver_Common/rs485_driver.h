#ifndef __RS485_DRIVER_H__
#define __RS485_DRIVER_H__

/* ======================== ͷ�ļ� ========================================== */
#include <stdbool.h>

#include "stm32f0xx.h"



/* ======================== �궨�� ========================================== */



/* ======================== �Զ������� ====================================== */
/* rs485ģ��Ӳ����Ϣ���Ͷ��� */
typedef struct {
	uint32_t				rcc;
	GPIO_TypeDef*			gpio;
			
	USART_TypeDef*			uart;
	uint32_t				uart_rcc;
	uint16_t				tx_pin;
	uint16_t				rx_pin;
	uint8_t					irq;
	
} rs485_hw_info_t;



/* ======================== �ӿ������� ====================================== */
void vRs485_DriverInit(void);
void vRs485_TestTask(void);


#endif /* __RS485_DRIVER_H__ */
