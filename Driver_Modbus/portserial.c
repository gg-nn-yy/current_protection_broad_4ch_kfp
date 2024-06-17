/*
 * FreeModbus Libary: BARE Port
 * Copyright (C) 2006 Christian Walter <wolti@sil.at>
 *
 * This library is free software; you can redistribute it and/or
 * modify it under the terms of the GNU Lesser General Public
 * License as published by the Free Software Foundation; either
 * version 2.1 of the License, or (at your option) any later version.
 *
 * This library is distributed in the hope that it will be useful,
 * but WITHOUT ANY WARRANTY; without even the implied warranty of
 * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the GNU
 * Lesser General Public License for more details.
 *
 * You should have received a copy of the GNU Lesser General Public
 * License along with this library; if not, write to the Free Software
 * Foundation, Inc., 51 Franklin St, Fifth Floor, Boston, MA  02110-1301  USA
 *
 * File: $Id$
 */


/* ----------------------- Modbus includes ----------------------------------*/
#include "mb.h"
#include "mbport.h"
#include "port.h"
#include "gd32f30x.h"



/* ------------------------ 全局变量定义区 ---------------------------------- */
typedef struct {
	uint32_t 		uartNo;
	rcu_periph_enum rcuUart;
	rcu_periph_enum	rcuGpio;
	uint32_t		gpio;
	uint32_t		txPin;
	uint32_t		rxPin;
	uint8_t 		irq;
} UartHwInfo_t;

static UartHwInfo_t g_UartHwInfo = {
	.uartNo  = USART1,
	.rcuUart = RCU_USART1,
	.rcuGpio = RCU_GPIOA,
	.gpio    = GPIOA,
	.txPin   = GPIO_PIN_2,
	.rxPin   = GPIO_PIN_3,
	.irq     = USART1_IRQn
};




/* ------------------------ 局部接口定义区 ---------------------------------- */
/*******************************************************************************
 * @brief gpio初始化
 *
 * @param void
 *
 * @return void
* * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * */
static void prvvGpioInit(void)
{
	rcu_periph_clock_enable(RCU_GPIOA);
	
	gpio_init(  g_UartHwInfo.gpio,   GPIO_MODE_AF_PP, 
	            GPIO_OSPEED_10MHZ,   g_UartHwInfo.txPin  );
	
	gpio_init(  g_UartHwInfo.gpio,   GPIO_MODE_IPU,   
	            GPIO_OSPEED_10MHZ,   g_UartHwInfo.rxPin  );
}



/*******************************************************************************
 * @brief  UART初始化
 *
 * @param  
 *
 * @return 
* * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * */
static void prvvUartInit(uint32_t baudRate)
{
	/* 使能串口时钟 */
	rcu_periph_clock_enable(g_UartHwInfo.rcuUart);
	/* 复位串口 */
	usart_deinit(g_UartHwInfo.uartNo);
	/* 设置波特率 */
	usart_baudrate_set(g_UartHwInfo.uartNo, baudRate);
	/* 使能发送 */
	usart_transmit_config(g_UartHwInfo.uartNo, USART_TRANSMIT_ENABLE);
	/* 使能接收 */
	usart_receive_config(g_UartHwInfo.uartNo, USART_RECEIVE_ENABLE);
	/* 使能NVIC中断和优先级 */
	nvic_irq_enable(g_UartHwInfo.irq, 0, 0);
	/* 使能串口 */
	usart_enable(g_UartHwInfo.uartNo);
}

static void prvvSwitchInit(void)
{
	rcu_periph_clock_enable(RCU_GPIOC);
	gpio_init(GPIOC, GPIO_MODE_OUT_PP, GPIO_OSPEED_50MHZ, GPIO_PIN_5);
}
/* ----------------------- static functions ---------------------------------*/
static void prvvUARTTxReadyISR( void );
static void prvvUARTRxISR( void );

/* ----------------------- Start implementation -----------------------------*/
void
vMBPortSerialEnable( BOOL xRxEnable, BOOL xTxEnable )
{
    /* If xRXEnable enable serial receive interrupts.
	 * If xTxENable enable transmitter empty interrupts.
     */
	if (TRUE == xRxEnable) {
		  // 使能接收中断
		usart_interrupt_enable(g_UartHwInfo.uartNo, USART_INT_RBNE);
		gpio_bit_reset(GPIOC, GPIO_PIN_5);
		
	} else {
		 // 失能接收中断
		usart_interrupt_disable(g_UartHwInfo.uartNo, USART_INT_RBNE);
		gpio_bit_set(GPIOC, GPIO_PIN_5);
	}
	
	if (TRUE == xTxEnable) {
		   // 使能发送中断
		usart_interrupt_enable(g_UartHwInfo.uartNo, USART_INT_TC);
		gpio_bit_set(GPIOC, GPIO_PIN_5);
		
	} else {
		  // 失能发送中断
		usart_interrupt_disable(g_UartHwInfo.uartNo, USART_INT_TC);
		gpio_bit_reset(GPIOC, GPIO_PIN_5);
	}
}

BOOL
xMBPortSerialInit( UCHAR ucPORT, ULONG ulBaudRate, UCHAR ucDataBits, eMBParity eParity )
{
	(void)ucPORT;
	(void)ucDataBits;
	(void)eParity;
	prvvSwitchInit();
	prvvGpioInit();
	prvvUartInit(ulBaudRate);
    return TRUE;
}

BOOL
xMBPortSerialPutByte( CHAR ucByte )
{
    /* Put a byte in the UARTs transmit buffer. This function is called
     * by the protocol stack if pxMBFrameCBTransmitterEmpty( ) has been
     * called. */
	usart_data_transmit(g_UartHwInfo.uartNo, ucByte);
    return TRUE;
}

BOOL
xMBPortSerialGetByte( CHAR * pucByte )
{
    /* Return the byte in the UARTs receive buffer. This function is called
     * by the protocol stack after pxMBFrameCBByteReceived( ) has been called.
     */
	*pucByte = usart_data_receive(g_UartHwInfo.uartNo);
    return TRUE;
}

/* Create an interrupt handler for the transmit buffer empty interrupt (or an 
 * equivalent) for your target processor.
 * 为目标处理器的传输缓冲区空中断(或等效中断)创建一个中断处理程序。
 * This function should then call pxMBFrameCBTransmitterEmpty( ) which tells the
 * protocol stack that a new character can be sent.
 * 这个函数应该调用pxMBFrameCBTransmitterEmpty()，它告诉协议栈可以发送一个新字符。
 * The protocol stack will then call xMBPortSerialPutByte( ) to send the character.
 * 然后，协议栈将调用xMBPortSerialPutByte()来发送字符。
 */
static void prvvUARTTxReadyISR( void )
{
    pxMBFrameCBTransmitterEmpty(  );
}

/* Create an interrupt handler for the receive interrupt for your target processor.
 * 为目标处理器的接收中断创建一个中断处理程序。
 * This function should then call pxMBFrameCBByteReceived( ).
 * 然后这个函数应该调用pxmbframecbbyterreceived()。
 * The protocol stack will then call xMBPortSerialGetByte( ) to retrieve the 
 * character.
 * 然后，协议栈将调用xMBPortSerialGetByte()来检索字符。 
 */
static void prvvUARTRxISR( void )
{
    pxMBFrameCBByteReceived(  );
}

void USART1_IRQHandler(void)
{
	/* 接收完成中断 */
	if ( SET == usart_interrupt_flag_get( g_UartHwInfo.uartNo, 
		                                   USART_INT_FLAG_RBNE   )) {
		prvvUARTRxISR();
		usart_interrupt_flag_clear(g_UartHwInfo.uartNo, USART_INT_FLAG_RBNE);
	}
	
	/* 发送完成中断 */
	if ( SET == usart_interrupt_flag_get( g_UartHwInfo.uartNo,
		                                   USART_INT_FLAG_TC    )) {
		prvvUARTTxReadyISR();
		usart_interrupt_flag_clear(g_UartHwInfo.uartNo, USART_INT_FLAG_TC);
	}
}

