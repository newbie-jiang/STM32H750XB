#ifndef __SYSTICK_H
#define __SYSTICK_H

#include "stm32f10x.h"
#define Delay_ms(x) Delay_us(100*x)	 /*µ¥Î»ms*/

#define  delay_ms(x)  SysTick_Delay_Ms(x)
#define  delay_us(x)  SysTick_Delay_Us(x)

void SysTick_Init(void);
void Delay_us(__IO u32 nTime);


void SysTick_Delay_Us( __IO uint32_t us);
void SysTick_Delay_Ms( __IO uint32_t ms);


#endif /* __SYSTICK_H */
