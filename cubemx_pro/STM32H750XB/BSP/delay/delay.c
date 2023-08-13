#include "delay.h" 
#include "tim.h" 


//void delay_us(uint32_t us)
//{
// tim_delay_us(&htim6,us);
//}

void delay_us(uint32_t us)
{
 static unsigned int i=0;
 for ( i=0;i<us*300;++i);
}



void delay_ms(uint32_t ms)
{
 HAL_Delay(ms);
}