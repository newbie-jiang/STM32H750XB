
#ifndef __APP_H
#define __APP_H


#include "stm32h7xx_hal.h"

 extern uint32_t rawValue;

void ctp_test(void);
void lcd_init(void);


void bsp_init(void);
void application(void);
#endif /* __APP_H */
