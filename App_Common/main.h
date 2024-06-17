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

/* ======================== ͷ�ļ� ========================================== */
#include <stdbool.h>
#include "stm32f0xx.h"



/* ======================== �궨�� ========================================== */
#define TASK_SUSPEND false  /* ����̬                                         */
#define TASK_RUN     true   /* ����̬                                         */
	


/* ======================== �Զ������� ====================================== */
/* ������� ���Ͷ���                                                          */
typedef struct {
	bool     runFlag;                 /* ���ȱ�־                             */
	uint16_t timeCount;               /* ʱ��Ƭ����ֵ                         */
	uint16_t timeReload;              /* ��������ֵ                           */
	void    (*pTaskFuncCb)(void);    /* ������ָ��                         */
} task_comps_t;



/* ======================== �ӿ������� ====================================== */


#endif /* __MAIN_H__ */

