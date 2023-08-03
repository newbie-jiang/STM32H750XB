#include "delay.h" 



void delay_us(uint16_t us)
{
 static unsigned int i=0;
 for ( i=0;i<us*300;++i);
}