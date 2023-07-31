/* USER CODE BEGIN Header */
/**
  ******************************************************************************
  * @file           : main.h
  * @brief          : Header for main.c file.
  *                   This file contains the common defines of the application.
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
#ifndef __MAIN_H
#define __MAIN_H

#ifdef __cplusplus
extern "C" {
#endif

/* Includes ------------------------------------------------------------------*/
#include "stm32h7xx_hal.h"

/* Private includes ----------------------------------------------------------*/
/* USER CODE BEGIN Includes */

/* USER CODE END Includes */

/* Exported types ------------------------------------------------------------*/
/* USER CODE BEGIN ET */

/* USER CODE END ET */

/* Exported constants --------------------------------------------------------*/
/* USER CODE BEGIN EC */

/* USER CODE END EC */

/* Exported macro ------------------------------------------------------------*/
/* USER CODE BEGIN EM */

/* USER CODE END EM */

/* Exported functions prototypes ---------------------------------------------*/
void Error_Handler(void);

/* USER CODE BEGIN EFP */

/* USER CODE END EFP */

/* Private defines -----------------------------------------------------------*/
#define GPIO_LCD_BL_Pin GPIO_PIN_4
#define GPIO_LCD_BL_GPIO_Port GPIOD
#define LED_R_Pin GPIO_PIN_15
#define LED_R_GPIO_Port GPIOC
#define WL_HOST_WAKE_Pin GPIO_PIN_3
#define WL_HOST_WAKE_GPIO_Port GPIOE
#define WL_HOST_WAKE_EXTI_IRQn EXTI3_IRQn
#define UART4_RX_Pin GPIO_PIN_9
#define UART4_RX_GPIO_Port GPIOI
#define WIFI_REG_ON_Pin GPIO_PIN_13
#define WIFI_REG_ON_GPIO_Port GPIOC
#define LED_B_Pin GPIO_PIN_8
#define LED_B_GPIO_Port GPIOI
#define QSPI_CS_W25Q64_Pin GPIO_PIN_6
#define QSPI_CS_W25Q64_GPIO_Port GPIOG
#define UART4_TX_Pin GPIO_PIN_0
#define UART4_TX_GPIO_Port GPIOA
#define USER_KEY_Pin GPIO_PIN_4
#define USER_KEY_GPIO_Port GPIOH
#define USER_KEY_EXTI_IRQn EXTI4_IRQn
#define SCL_PAJ7620_Pin GPIO_PIN_11
#define SCL_PAJ7620_GPIO_Port GPIOH
#define PB2_Pin GPIO_PIN_2
#define PB2_GPIO_Port GPIOB
#define SDA_PAJ7620_Pin GPIO_PIN_12
#define SDA_PAJ7620_GPIO_Port GPIOH
#define DHT11_Pin GPIO_PIN_1
#define DHT11_GPIO_Port GPIOB
#define SD_CAP_Pin GPIO_PIN_12
#define SD_CAP_GPIO_Port GPIOB
#define SPI1_FLASH_NSS_Pin GPIO_PIN_4
#define SPI1_FLASH_NSS_GPIO_Port GPIOA
#define BUZZER_Pin GPIO_PIN_0
#define BUZZER_GPIO_Port GPIOB

/* USER CODE BEGIN Private defines */
#define BUZZER_ON    HAL_GPIO_WritePin(GPIOB, PB2_Pin|BUZZER_Pin, GPIO_PIN_SET)
#define BUZZER_OFF   HAL_GPIO_WritePin(GPIOB, PB2_Pin|BUZZER_Pin, GPIO_PIN_RESET)




/* USER CODE END Private defines */

#ifdef __cplusplus
}
#endif

#endif /* __MAIN_H */
