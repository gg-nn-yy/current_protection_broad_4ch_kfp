/* ======================== ͷ�ļ� ========================================== */
#include <stdio.h>

#include "rs485_driver.h"



/* ======================== ȫ�ֱ��������� ================================== */
/* ����ģ�����������Ϣ */
static rs485_hw_info_t g_rs485_hw_info = {
	.rcc         = RCC_AHBPeriph_GPIOA,
	.gpio        = GPIOA,
                      
	.uart        = USART1,
	.uart_rcc    = RCC_APB2Periph_USART1,
	.tx_pin      = GPIO_Pin_9,
	.rx_pin      = GPIO_Pin_10,
	.irq         = USART1_IRQn,
};


/* ======================== �꺯�������� ==================================== */



/* ======================== �ֲ��ӿڶ����� ================================== */
/*******************************************************************************
 * @brief  GPIO��ʼ��
 *
 * @param 
 *
 * @return 
* * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * */
static void prvvGpio_Init(void)
{
	/* ����ʱ��                                                               */
	RCC_AHBPeriphClockCmd(g_rs485_hw_info.rcc, ENABLE);
	
	/* ��������                                                               */
	GPIO_PinAFConfig(g_rs485_hw_info.gpio, GPIO_PinSource9,  GPIO_AF_1);
	GPIO_PinAFConfig(g_rs485_hw_info.gpio, GPIO_PinSource10, GPIO_AF_1);
	
	/* ��������                                                               */
	GPIO_InitTypeDef  gpio_init_para;
	gpio_init_para.GPIO_Mode  = GPIO_Mode_AF;  	  		// �������
	gpio_init_para.GPIO_OType = GPIO_OType_PP;       	// ����
	gpio_init_para.GPIO_Pin   = g_rs485_hw_info.tx_pin;
	gpio_init_para.GPIO_PuPd  = GPIO_PuPd_UP; 			// ��������
	gpio_init_para.GPIO_Speed = GPIO_Speed_50MHz;	 	// IO���ٶ�Ϊ50MHz
	GPIO_Init(g_rs485_hw_info.gpio, &gpio_init_para);
	
	gpio_init_para.GPIO_Mode  = GPIO_Mode_IN;  	  		// ����
	gpio_init_para.GPIO_OType = GPIO_OType_PP;       	// ����
	gpio_init_para.GPIO_Pin   = g_rs485_hw_info.rx_pin;
	gpio_init_para.GPIO_PuPd  = GPIO_PuPd_UP; 			// ����
	gpio_init_para.GPIO_Speed = GPIO_Speed_50MHz;	 	// IO���ٶ�Ϊ50MHz
	GPIO_Init(g_rs485_hw_info.gpio, &gpio_init_para);
}


/*******************************************************************************
 * @brief  UART��ʼ��
 *
 * @param  
 *
 * @return 
* * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * */
static void prvvUart_Init(uint32_t BuadRate)
{
	/* ����ʱ��                                                               */
	RCC_APB2PeriphClockCmd(g_rs485_hw_info.uart_rcc, ENABLE);
	
	/* ��������                                                               */
	USART_InitTypeDef uart_init_para = {
		.USART_BaudRate 		   = BuadRate,
		.USART_Mode 			   = USART_Mode_Tx | USART_Mode_Rx,
		.USART_Parity 			   = USART_Parity_No,
		.USART_StopBits 		   = USART_StopBits_1,
		.USART_WordLength 		   = USART_WordLength_8b,
		.USART_HardwareFlowControl = USART_HardwareFlowControl_None,
	};
	USART_Init(g_rs485_hw_info.uart , &uart_init_para);
	
	#if 0
	/* �ж�����                                                               */
	USART_ITConfig(g_rs485_hw_info.uart, USART_IT_IDLE, ENABLE);
	NVIC_PriorityGroupConfig(NVIC_PriorityGroup_2);
	NVIC_InitTypeDef NVIC_init_para = {
		.NVIC_IRQChannel 					= g_rs485_hw_info.irq,
		.NVIC_IRQChannelCmd 				= ENABLE,
		.NVIC_IRQChannelPreemptionPriority 	= 1,
		.NVIC_IRQChannelSubPriority 		= 1
	};
	NVIC_Init(&NVIC_init_para);
	#endif
	
	/* ʹ�ܴ���                                                               */
	USART_Cmd(g_rs485_hw_info.uart, ENABLE);
}


/*******************************************************************************
 * @brief 	��ӳ��fputc
 *
 * @param 
 *
 * @return 	��
* * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * */
int fputc(int ch , FILE *f)
{
	USART_SendData(g_rs485_hw_info.uart, ch);
	while ( RESET == USART_GetFlagStatus( g_rs485_hw_info.uart, 
		                                   USART_FLAG_TXE          ));

	return ch;
}



/* ======================== ����ӿڶ����� ================================== */
/*******************************************************************************
 * @brief rs485ģ��������ʼ��
 *
 * @param void
 *
 * @return void
* * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * */
void vRs485_DriverInit(void)
{
	prvvGpio_Init();
	prvvUart_Init(115200);
}


/*******************************************************************************
 * @brief  rs485��������
 *
 * @param 
 *
 * @return 
* * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * */
void vRs485_TestTask(void)
{
	printf("Hello World!\r\n");
}
