/**
  ******************************************************************************
  * @file    Project/STM32F0xx_StdPeriph_Templates/main.h 
  * @author  MCD Application Team
  * @version V1.6.0
  * @date    13-October-2021
  * @brief   Header for main.c module
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
  
/* Define to prevent recursive inclusion -------------------------------------*/
#ifndef __MAIN_H__ 
#define __MAIN_H__

/* ======================== 头文件 ========================================== */
#include <stdbool.h>
#include "stm32f0xx.h"



/* ======================== 宏定义 ========================================== */
#define TASK_SUSPEND false  /* 挂起态                                         */
#define TASK_RUN     true   /* 运行态                                         */
	


/* ======================== 自定义类型 ====================================== */
/* 任务组件 类型定义                                                          */
typedef struct {
	bool     runFlag;                 /* 调度标志                             */
	uint16_t timeCount;               /* 时间片计数值                         */
	uint16_t timeReload;              /* 计数重载值                           */
	void    (*pTaskFuncCb)(void);    /* 任务函数指针                         */
} task_comps_t;



/* ======================== 接口声明区 ====================================== */


#endif /* __MAIN_H__ */

