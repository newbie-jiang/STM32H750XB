#include "led.h"

//STM32学习有问题，请加入q群交流: 643807576
//初始化PB5和PE5为输出口.并使能这两个口的时钟		    
//LED GPIO初始化
void LED_Init(void)
{
 
 GPIO_InitTypeDef  GPIO_InitStructure;
 	
 RCC_APB2PeriphClockCmd(LED0_GPIO_CLK|LED1_GPIO_CLK, ENABLE);	 //使能LED0、LED1端口的时钟
	
 GPIO_InitStructure.GPIO_Pin = LED0_GPIO_PIN;				 //LED0端口配置
 GPIO_InitStructure.GPIO_Mode = GPIO_Mode_Out_PP; 		 //推挽输出
 GPIO_InitStructure.GPIO_Speed = GPIO_Speed_50MHz;		 //IO口速度为50MHz
 GPIO_Init(GPIOB, &GPIO_InitStructure);					 				//根据设定参数初始化GPIO
 GPIO_SetBits(LED0_GPIO_PORT,LED0_GPIO_PIN);						 //LED0输出高

 GPIO_InitStructure.GPIO_Pin = LED1_GPIO_PIN;	    		 //LED1端口配置, 推挽输出
 GPIO_Init(GPIOE, &GPIO_InitStructure);	  				 			//推挽输出 ，IO口速度为50MHz
 GPIO_SetBits(LED1_GPIO_PORT,LED1_GPIO_PIN); 						 //LED1输出高
}
 
