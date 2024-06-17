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

/* ======================== 头文件 ========================================== */
#include "main.h"
#include "stm32f0xx.h"

#include "systick.h"
#include "led_driver.h"
#include "rs485_driver.h"
#include "ads1118_driver.h"

#include "filter_middleware.h"

#include "sensor_app.h"


/* ======================== 全局变量定义区 ================================== */
/* 任务组件数组 */
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
	


/* ======================== 宏函数定义区 ==================================== */



/* ======================== 接口定义区 ====================================== */
/*******************************************************************************
 * @brief 任务事件处理函数，在主函数loop中周期性调用，当检测到任务组件的调度标
 *        志为运行态时调度任务，并在任务结束后重新将调度标志置为挂起态
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
 * @brief 任务时间表回调函数，在SysTick_Handler（1ms产生一次中断）中调用，每次
 *        调用将任务组件的时间片计数值减一，当任务组件的时间片计数值减为0时，将
 *        任务的调度标志置为运行态，并重置时间片计数值
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
 * @brief 驱动初始化
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
 * @brief 应用初始化
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
 * @brief 主函数
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



