#ifndef __BSP_H
#define __BSP_H

#include "stm32h7xx_hal.h"

/*******************************LED***************************************************/
#define   LED_R_ON     HAL_GPIO_WritePin(LED_R_GPIO_Port, LED_R_Pin, GPIO_PIN_RESET)
#define   LED_R_OFF    HAL_GPIO_WritePin(LED_R_GPIO_Port, LED_R_Pin, GPIO_PIN_SET)

#define   LED_B_ON 		 HAL_GPIO_WritePin(LED_B_GPIO_Port, LED_B_Pin, GPIO_PIN_RESET)
#define   LED_B_OFF    HAL_GPIO_WritePin(LED_B_GPIO_Port, LED_B_Pin, GPIO_PIN_SET)
/**************************************************************************************/


/*********************************key********************************************/
// 定义按键引脚和GPIO端口
#define BUTTON_PIN GPIO_PIN_4
#define BUTTON_GPIO_PORT GPIOH
/********************************************************************************/







#endif /* __BSP_H */
