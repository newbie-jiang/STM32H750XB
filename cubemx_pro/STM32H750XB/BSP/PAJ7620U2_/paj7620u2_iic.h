#ifndef __PAJ7620U2_IIC_H
#define __PAJ7620U2_IIC_H

#include "stdint.h"
#include "gpio.h"


extern uint16_t  tick_us_count;

void SDA_IN(void);
void SDA_OUT(void);
uint8_t READ_SDA(void);
void delay_us(uint16_t us);


///**
//PH11  SCL
//PH12  SDA
//*/


//GPIO引脚配置（本例程 PF7接模块的SCL脚、PF8接模块的SDA脚）	 
#define GS_IIC_SCL(n)    ((n)>0 ? HAL_GPIO_WritePin(GPIOH, SCL_PAJ7620_Pin, GPIO_PIN_SET)\
                             : HAL_GPIO_WritePin(GPIOH, SCL_PAJ7620_Pin, GPIO_PIN_RESET))		//SCL（SCL_OUT） 


#define GS_IIC_SDA(n)    ((n)>0 ? HAL_GPIO_WritePin(GPIOH, SDA_PAJ7620_Pin, GPIO_PIN_SET)\
                                : HAL_GPIO_WritePin(GPIOH, SDA_PAJ7620_Pin, GPIO_PIN_RESET))		//SCL（SCL_OUT）  


//#define GS_IIC_SDA    GS_SDA_OUT() 		//SDA_OUT，用于发送SDA数据给IIC传感器模块
//#define GS_READ_SDA    READ_SDA(); 		//SDA_IN，用于读取IIC传感器模块的SDA数据

//I/O方向配置(寄存器操作).
#define GS_SDA_IN()  SDA_IN();	  //PH12  浮空输入
#define GS_SDA_OUT() SDA_OUT();	//PH12  推挽输出（通用）

//PS:  I/O方向不会配置? :傻瓜式操作 ---> https://xinso.blog.csdn.net/article/details/115862486

uint8_t GS_Write_Byte(uint8_t REG_Address,uint8_t REG_data);
uint8_t GS_Read_Byte(uint8_t REG_Address);
uint8_t GS_Read_nByte(uint8_t REG_Address,uint16_t len,uint8_t *buf);
void GS_i2c_init(void);
void GS_WakeUp(void);

#endif


