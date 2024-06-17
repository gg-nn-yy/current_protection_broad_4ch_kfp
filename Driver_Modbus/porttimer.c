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

/* ----------------------- Platform includes --------------------------------*/
#include "port.h"
#include "gd32f30x.h"



/* ----------------------- Modbus includes ----------------------------------*/
#include "mb.h"
#include "mbport.h"



/* ----------------------- static functions ---------------------------------*/
static void prvvTIMERExpiredISR( void );



/* ----------------------- Start implementation -----------------------------*/
BOOL
xMBPortTimersInit( USHORT usTim1Timerout50us )
{
	/* ʹ�ܶ�ʱ��ʱ�� */
	rcu_periph_clock_enable(RCU_TIMER3);
	/* ��λ��ʱ�� */
	timer_deinit(TIMER3);
	/* ��ʱ����ʼ������ */
	timer_parameter_struct timerInitPara;
	timer_struct_para_init(&timerInitPara);
	timerInitPara.prescaler = 5999;                   // ����Ԥ��Ƶ��ֵ, Ƶ��120MHz / 6000 = 20kHz,��Ӧ����50us
	timerInitPara.period    = usTim1Timerout50us - 1; // �����Զ���װ��ֵ
	timer_init(TIMER3, &timerInitPara);
	
	/* ʹ�ܶ�ʱ�����������ж� */
	timer_interrupt_enable(TIMER3, TIMER_INT_UP);
	/* ʹ�ܶ�ʱ���жϺ����ȼ� */
	nvic_irq_enable(TIMER3_IRQn, 0, 0);
	/* ʹ�ܶ�ʱ�� */
	//timer_enable(TIMER3);
	
    return TRUE;
}


inline void
vMBPortTimersEnable(  )
{
    /* Enable the timer with the timeout passed to xMBPortTimersInit( ) */
	timer_counter_value_config(TIMER3, 0);                 // ���㶨ʱ��
	timer_interrupt_flag_clear(TIMER3, TIMER_INT_FLAG_UP); // �����ʱ�������ж�
	timer_interrupt_enable(TIMER3, TIMER_INT_UP);          // ʹ�ܶ�ʱ�������ж�
	timer_enable(TIMER3);                                  // ʹ�ܶ�ʱ��
}

inline void
vMBPortTimersDisable(  )
{
    /* Disable any pending timers. */
	timer_counter_value_config(TIMER3, 0);                 // ���㶨ʱ��
	timer_interrupt_flag_clear(TIMER3, TIMER_INT_FLAG_UP); // �����ʱ�������ж�
	timer_interrupt_disable(TIMER3, TIMER_INT_UP);         // ���ö�ʱ�������ж�
	timer_disable(TIMER3);                                 // ʧ�ܶ�ʱ��
}

/* Create an ISR which is called whenever the timer has expired. 
 * ����һ��ISR������ʱ������ʱ��������
 * This function must then call pxMBPortCBTimerExpired( ) to notify the protocol
 * stack that the timer has expired.
 * ��������������pxMBPortCBTimerExpired()��֪ͨЭ��ջ��ʱ���Ѿ����ڡ�
 */
static void prvvTIMERExpiredISR( void )
{
    ( void )pxMBPortCBTimerExpired(  );
}

void TIMER3_IRQHandler(void)
{
	if ( SET == timer_interrupt_flag_get(TIMER3, TIMER_INT_FLAG_UP )) {
		timer_interrupt_flag_clear(TIMER3, TIMER_INT_FLAG_UP);
		timer_counter_value_config(TIMER3, 0);
		prvvTIMERExpiredISR();
	}
}

