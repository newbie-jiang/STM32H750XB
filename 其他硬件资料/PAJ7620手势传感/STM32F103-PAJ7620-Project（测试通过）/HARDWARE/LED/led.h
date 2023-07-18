#ifndef __LED_H
#define __LED_H	 
#include "sys.h"
//////////////////////////////////////////////////////////////////////////////////	 
// 本程序只供学习使用，未经作者许可，不得用于其它任何用途
// STM32F103开发板 LED驱动代码	   
// STM32学习有问题，请加入q群交流: 643807576

//寄存器操作（简约版）
#define LED0 PBout(5)// PB5
#define LED1 PEout(5)// PE5	

/* 定义LED控制GPIO的宏 */
#define LED0_GPIO_PORT    	GPIOB			              
#define LED0_GPIO_CLK 	    RCC_APB2Periph_GPIOB		
#define LED0_GPIO_PIN		  	GPIO_Pin_5  
 
#define LED1_GPIO_PORT    	GPIOE		              
#define LED1_GPIO_CLK 	    RCC_APB2Periph_GPIOE		
#define LED1_GPIO_PIN		  	GPIO_Pin_5  

void LED_Init(void);//初始化
	    
#endif
