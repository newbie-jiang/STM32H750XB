/**
  ******************************************************************************
  * @file    main.c
  * @author  fire
  * @version V1.0
  * @date    2013-xx-xx
  * @brief   手势识别模块实验
  ******************************************************************************
  * @attention
  *
  * 实验平台:野火 F103-霸道 STM32 开发板 
  * 论坛    :http://www.firebbs.cn
  * 淘宝    :https://fire-stm32.taobao.com
  *
  ******************************************************************************
  */ 
 
#include "stm32f10x.h"
#include "./usart/bsp_usart.h"
#include "./SysTick/bsp_SysTick.h"
#include "./paj7620u2/paj7620u2.h"
#include "bsp_exti.h"
/**
  * @brief  主函数
  * @param  无
  * @retval 无
  */
int main(void)
{		
  /*初始化USART 配置模式为 115200 8-N-1，中断接收*/
  USART_Config();
  EXTI_Key_Config(); 
  
  printf(" 欢迎使用野火开发板\r\n");
  printf(" 手势识别传感器实验\r\n");
  
  while(!paj7620u2_init())  //PAJ7620U2传感器初始化
	{
	  printf("PAJ7620U2 初始化失败!!!\r\n");
	  delay_ms(600);	
	}
    printf("PAJ7620U2 初始化成功\r\n");
  while(1)
	{
		paj7620u2_sensor_test();//PAJ7620U2传感器测试
	}	
 }
  
  
/*********************************************END OF FILE**********************/
