#ifndef __PAJ7620U2_IIC_H
#define __PAJ7620U2_IIC_H
//////////////////////////////////////////////////////////////////////////////////	 
//本程序只供学习使用，未经作者许可，不得用于其它任何用途
//STM32F10x开发板  PAJ7620传感器模块IIC驱动	（模块I2C引脚配置头文件）
//STM32学习有问题，请加入q群交流: 643807576

#include "sys.h"

//GPIO引脚配置（本例程 PF7接模块的SCL脚、PF8接模块的SDA脚）	 
#define GS_IIC_SCL    PFout(7) 		//SCL（SCL_OUT）
#define GS_IIC_SDA    PFout(8) 		//SDA_OUT，用于发送SDA数据给IIC传感器模块
#define GS_READ_SDA   PFin(8) 		//SDA_IN，用于读取IIC传感器模块的SDA数据

//I/O方向配置(寄存器操作).
#define GS_SDA_IN()  {GPIOF->CRH&=0xFFFFFFF0;GPIOF->CRH|=4<<0;}	//PF8  浮空输入
#define GS_SDA_OUT() {GPIOF->CRH&=0xFFFFFFF0;GPIOF->CRH|=3<<0;}	//PF8  推挽输出（通用）

//PS:  I/O方向不会配置? :傻瓜式操作 ---> https://xinso.blog.csdn.net/article/details/115862486

u8 GS_Write_Byte(u8 REG_Address,u8 REG_data);
u8 GS_Read_Byte(u8 REG_Address);
u8 GS_Read_nByte(u8 REG_Address,u16 len,u8 *buf);
void GS_i2c_init(void);
void GS_WakeUp(void);

#endif


