/* USER CODE BEGIN Header */
/**
  ******************************************************************************
  * @file    stm32h7xx_it.h
  * @brief   This file contains the headers of the interrupt handlers.
  ******************************************************************************
  * @attention
  *
  * Copyright (c) 2023 STMicroelectronics.
  * All rights reserved.
  *
  * This software is licensed under terms that can be found in the LICENSE file
  * in the root directory of this software component.
  * If no LICENSE file comes with this software, it is provided AS-IS.
  *
 ******************************************************************************
  */
/* USER CODE END Header */

/* Define to prevent recursive inclusion -------------------------------------*/
#ifndef __STM32H7xx_IT_H
#define __STM32H7xx_IT_H

#ifdef __cplusplus
 extern "C" {
#endif

/* Private includes ----------------------------------------------------------*/
/* USER CODE BEGIN Includes */
#include "stdio.h"
#include <stdbool.h>
#include "stdint.h"
/* USER CODE END Includes */

/* Exported types ------------------------------------------------------------*/
/* USER CODE BEGIN ET */
void key_process(void);
unsigned int scan_irda(void);
/* USER CODE END ET */

/* Exported constants --------------------------------------------------------*/
/* USER CODE BEGIN EC */
 extern bool nec_ValueChanged;  // 标记变量是否改变
 extern  uint32_t OverflowCount;
 
 
extern uint32_t IC_TIMES;  // 捕获次数，单位1ms
extern uint8_t IC_START_FLAG;  // 捕获开始标志，1：已捕获到高电平；0：还没有捕获到高电平
extern uint8_t IC_DONE_FLAG;  // 捕获完成标志，1：已完成一次高电平捕获
extern uint16_t IC_VALUE;
extern uint8_t irda_code_flag;

extern bool GPIO_TS_IIC_INT_Pin_state;
/* USER CODE END EC */

/* Exported macro ------------------------------------------------------------*/
/* USER CODE BEGIN EM */

/* USER CODE END EM */

/* Exported functions prototypes ---------------------------------------------*/
void NMI_Handler(void);
void HardFault_Handler(void);
void MemManage_Handler(void);
void BusFault_Handler(void);
void UsageFault_Handler(void);
void SVC_Handler(void);
void DebugMon_Handler(void);
void PendSV_Handler(void);
void SysTick_Handler(void);
void EXTI3_IRQHandler(void);
void EXTI4_IRQHandler(void);
void USART1_IRQHandler(void);
void EXTI15_10_IRQHandler(void);
void TIM5_IRQHandler(void);
void UART4_IRQHandler(void);
void TIM7_IRQHandler(void);
void LTDC_IRQHandler(void);
void OTG_FS_IRQHandler(void);
void BDMA_Channel0_IRQHandler(void);
/* USER CODE BEGIN EFP */

/* USER CODE END EFP */

#ifdef __cplusplus
}
#endif

#endif /* __STM32H7xx_IT_H */
