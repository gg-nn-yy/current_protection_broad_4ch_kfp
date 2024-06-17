/* ======================== 头文件 ========================================== */
#include <stdio.h>

#include "rs485_driver.h"



/* ======================== 全局变量定义区 ================================== */
/* 蓝牙模块驱动相关信息 */
static rs485_hw_info_t g_rs485_hw_info = {
	.rcc         = RCC_AHBPeriph_GPIOA,
	.gpio        = GPIOA,
                      
	.uart        = USART1,
	.uart_rcc    = RCC_APB2Periph_USART1,
	.tx_pin      = GPIO_Pin_9,
	.rx_pin      = GPIO_Pin_10,
	.irq         = USART1_IRQn,
};


/* ======================== 宏函数定义区 ==================================== */



/* ======================== 局部接口定义区 ================================== */
/*******************************************************************************
 * @brief  GPIO初始化
 *
 * @param 
 *
 * @return 
* * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * */
static void prvvGpio_Init(void)
{
	/* 开启时钟                                                               */
	RCC_AHBPeriphClockCmd(g_rs485_hw_info.rcc, ENABLE);
	
	/* 复用配置                                                               */
	GPIO_PinAFConfig(g_rs485_hw_info.gpio, GPIO_PinSource9,  GPIO_AF_1);
	GPIO_PinAFConfig(g_rs485_hw_info.gpio, GPIO_PinSource10, GPIO_AF_1);
	
	/* 引脚配置                                                               */
	GPIO_InitTypeDef  gpio_init_para;
	gpio_init_para.GPIO_Mode  = GPIO_Mode_AF;  	  		// 复用输出
	gpio_init_para.GPIO_OType = GPIO_OType_PP;       	// 推挽
	gpio_init_para.GPIO_Pin   = g_rs485_hw_info.tx_pin;
	gpio_init_para.GPIO_PuPd  = GPIO_PuPd_UP; 			// 无上下拉
	gpio_init_para.GPIO_Speed = GPIO_Speed_50MHz;	 	// IO口速度为50MHz
	GPIO_Init(g_rs485_hw_info.gpio, &gpio_init_para);
	
	gpio_init_para.GPIO_Mode  = GPIO_Mode_IN;  	  		// 输入
	gpio_init_para.GPIO_OType = GPIO_OType_PP;       	// 推挽
	gpio_init_para.GPIO_Pin   = g_rs485_hw_info.rx_pin;
	gpio_init_para.GPIO_PuPd  = GPIO_PuPd_UP; 			// 上拉
	gpio_init_para.GPIO_Speed = GPIO_Speed_50MHz;	 	// IO口速度为50MHz
	GPIO_Init(g_rs485_hw_info.gpio, &gpio_init_para);
}


/*******************************************************************************
 * @brief  UART初始化
 *
 * @param  
 *
 * @return 
* * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * */
static void prvvUart_Init(uint32_t BuadRate)
{
	/* 开启时钟                                                               */
	RCC_APB2PeriphClockCmd(g_rs485_hw_info.uart_rcc, ENABLE);
	
	/* 串口配置                                                               */
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
	/* 中断配置                                                               */
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
	
	/* 使能串口                                                               */
	USART_Cmd(g_rs485_hw_info.uart, ENABLE);
}


/*******************************************************************************
 * @brief 	重映射fputc
 *
 * @param 
 *
 * @return 	无
* * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * */
int fputc(int ch , FILE *f)
{
	USART_SendData(g_rs485_hw_info.uart, ch);
	while ( RESET == USART_GetFlagStatus( g_rs485_hw_info.uart, 
		                                   USART_FLAG_TXE          ));

	return ch;
}



/* ======================== 对外接口定义区 ================================== */
/*******************************************************************************
 * @brief rs485模块驱动初始化
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
 * @brief  rs485测试任务
 *
 * @param 
 *
 * @return 
* * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * */
void vRs485_TestTask(void)
{
	printf("Hello World!\r\n");
}
