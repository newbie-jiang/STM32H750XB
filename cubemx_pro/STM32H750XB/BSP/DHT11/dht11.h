#ifndef __DHT11_H
#define __DHT11_H

#include "stdint.h"


void dht11_OUT(void);
void dht11_IN(void);
uint8_t READ_DHT11(void);




#define DHT11_IO_IN()    dht11_IN()
#define DHT11_IO_OUT()   dht11_OUT()
 

#define DHT11_DQ_OUT(n)    ((n)>0 ? HAL_GPIO_WritePin(DHT11_GPIO_Port, DHT11_Pin, GPIO_PIN_SET)\
                                  : HAL_GPIO_WritePin(DHT11_GPIO_Port, DHT11_Pin, GPIO_PIN_RESET))//DHT11_OUT 




   	
uint8_t DHT11_Init(void);//初始化DHT11
uint8_t DHT11_Read_Data(uint8_t *temp,uint8_t *humi);//读取温湿度
uint8_t DHT11_Read_Byte(void);//读出一个字节
uint8_t DHT11_Read_Bit(void);//读出一个位
uint8_t DHT11_Check(void);//检测是否存在DHT11
void DHT11_Rst(void);//复位DHT11  
#endif
