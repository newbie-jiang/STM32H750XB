#ifndef __PAJ7620U2_IIC_H
#define __PAJ7620U2_IIC_H

#include "stm32f10x.h"


/* 定义读写SCL和SDA的宏，已增加代码的可移植性和可阅读性 */
#if 0	/* 条件编译： 1 选择GPIO的库函数实现IO读写 */
	#define GS_I2C_SCL_1()  GPIO_SetBits(GS_I2C_GPIO_PORT,  GS_I2C_SCL_PIN)		/* SCL = 1 */
	#define GS_I2C_SCL_0()  GPIO_ResetBits(GS_I2C_GPIO_PORT,  GS_I2C_SCL_PIN)		/* SCL = 0 */
	
	#define GS_I2C_SDA_1()  GPIO_SetBits(GS_I2C_GPIO_PORT, GS_I2C_SDA_PIN)		/* SDA = 1 */
	#define GS_I2C_SDA_0()  GPIO_ResetBits(GS_I2C_GPIO_PORT, GS_I2C_SDA_PIN)		/* SDA = 0 */
	
	#define GS_I2C_SDA_READ()  GPIO_ReadInputDataBit(GS_I2C_GPIO_PORT, GS_I2C_SDA_PIN)	/* 读SDA口线状态 */
#else	/* 这个分支选择直接寄存器操作实现IO读写 */
    /*　注意：如下写法，在IAR最高级别优化时，会被编译器错误优化 */
	#define GS_I2C_SCL_1()  GS_I2C_GPIO_PORT->BSRR =  GS_I2C_SCL_PIN				/* SCL = 1 */
	#define GS_I2C_SCL_0()  GS_I2C_GPIO_PORT->BRR =  GS_I2C_SCL_PIN				/* SCL = 0 */
	
	#define GS_I2C_SDA_1()  GS_I2C_GPIO_PORT->BSRR = GS_I2C_SDA_PIN				/* SDA = 1 */
	#define GS_I2C_SDA_0()  GS_I2C_GPIO_PORT->BRR = GS_I2C_SDA_PIN				/* SDA = 0 */
	
	#define GS_I2C_SDA_READ()  ((GS_I2C_GPIO_PORT->IDR & GS_I2C_SDA_PIN) != 0)	/* 读SDA口线状态 */
#endif

  #define GS_SDA_IN()  {GPIOB->CRH&=0XFFFF0FFF;GPIOB->CRH|=8<<12;}
  #define GS_SDA_OUT() {GPIOB->CRH&=0XFFFF0FFF;GPIOB->CRH|=3<<12;}

/* 定义I2C总线连接的GPIO端口, 用户只需要修改下面4行代码即可任意改变SCL和SDA的引脚 */
#define GS_I2C_GPIO_PORT				GPIOB			/* GPIO端口 */
#define GS_I2C_GPIO_CLK			  	RCC_APB2Periph_GPIOB		/* GPIO端口时钟 */
#define GS_I2C_SCL_PIN					GPIO_Pin_10			/* 连接到SCL时钟线的GPIO */
#define GS_I2C_SDA_PIN					GPIO_Pin_11/* 连接到SDA数据线的GPIO */

uint8_t GS_Write_Byte(uint8_t REG_Address,uint8_t REG_data);
uint8_t GS_Read_Byte(uint8_t REG_Address);
uint8_t GS_Read_nByte(uint8_t REG_Address,uint16_t len,uint8_t *buf);
void GS_i2c_init(void);
void GS_WakeUp(void);

#endif


