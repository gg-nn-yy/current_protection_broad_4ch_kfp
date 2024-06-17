/**
  ******************************************************************************
  * @file    Project/STM32F0xx_StdPeriph_Templates/main.c 
  * @author  MCD Application Team
  * @version V1.6.0
  * @date    13-October-2021
  * @brief   Main program body
  ******************************************************************************
  * @attention
  *
  * Copyright (c) 2014 STMicroelectronics.
  * All rights reserved.
  *
  * This software is licensed under terms that can be found in the LICENSE file
  * in the root directory of this software component.
  * If no LICENSE file comes with this software, it is provided AS-IS.
  *
  ******************************************************************************
  */

/* ======================== ͷ�ļ� ========================================== */
#include "main.h"
#include "stm32f0xx.h"

#include "systick.h"
#include "led_driver.h"
#include "rs485_driver.h"
#include "ads1118_driver.h"

#include "filter_middleware.h"

#include "sensor_app.h"


/* ======================== ȫ�ֱ��������� ================================== */
/* ����������� */
static task_comps_t g_task_comps[] = {

	{TASK_SUSPEND, 100, 100, vLed_TestTask1},
	{TASK_SUSPEND, 200, 200, vLed_TestTask2},
	{TASK_SUSPEND, 300, 300, vLed_TestTask3},
	{TASK_SUSPEND, 400, 400, vLed_TestTask4},
	{TASK_SUSPEND, 500, 500, vLed_TestTask5},
	{TASK_SUSPEND, 600, 600, vLed_TestTask6},
	{TASK_SUSPEND, 700, 700, vLed_TestTask7},
	{TASK_SUSPEND, 800, 800, vLed_TestTask8},
	
	{TASK_SUSPEND, 5,  5,    vAds1118_Task},
	{TASK_SUSPEND, 200, 200, vSensor_Task},

};

#define TASK_NUM_MAX  (sizeof(g_task_comps) / sizeof(g_task_comps[0]))
	


/* ======================== �꺯�������� ==================================== */



/* ======================== �ӿڶ����� ====================================== */
/*******************************************************************************
 * @brief �����¼�����������������loop�������Ե��ã�����⵽��������ĵ��ȱ�
 *        ־Ϊ����̬ʱ�������񣬲���������������½����ȱ�־��Ϊ����̬
 *
 * @param void
 *
 * @return void
* * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * */
static void vTaskHandler(void)
{
	for (uint8_t i = 0; i < TASK_NUM_MAX; i++) {
		
		if (TASK_RUN == g_task_comps[i].runFlag) {
			
			g_task_comps[i].pTaskFuncCb();
			g_task_comps[i].runFlag = TASK_SUSPEND;
		}
	}
}


/*******************************************************************************
 * @brief ����ʱ���ص���������SysTick_Handler��1ms����һ���жϣ��е��ã�ÿ��
 *        ���ý����������ʱ��Ƭ����ֵ��һ�������������ʱ��Ƭ����ֵ��Ϊ0ʱ����
 *        ����ĵ��ȱ�־��Ϊ����̬��������ʱ��Ƭ����ֵ
 *
 * @param void
 *
 * @return void
* * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * */
static void vTaskScheduleCb(void)
{
	for (uint8_t i = 0; i < TASK_NUM_MAX; i++) {
		
		g_task_comps[i].timeCount--;
		
		if ( 0 == g_task_comps[i].timeCount ) {
			
			g_task_comps[i].runFlag = TASK_RUN;
			g_task_comps[i].timeCount = g_task_comps[i].timeReload;
		}
	}
}


/*******************************************************************************
 * @brief ������ʼ��
 *
 * @param void
 *
 * @return void
* * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * */
static void Driver_Init(void)
{
	vLed_DriverInit();
	vRs485_DriverInit();
	vAds1118_DriverInit();
	vSystickInit();
}


/*******************************************************************************
 * @brief Ӧ�ó�ʼ��
 *
 * @param void
 *
 * @return void
* * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * */
static void App_Init(void)
{
	vFilter_Init();
	vTaskScheduleCbReg(vTaskScheduleCb);
}


/*******************************************************************************
 * @brief ������
 *
 * @param void
 *
 * @return void
* * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * */
int main(void)
{
	Driver_Init();
	App_Init();
	
	while (1) {
		vTaskHandler();
	}
}



