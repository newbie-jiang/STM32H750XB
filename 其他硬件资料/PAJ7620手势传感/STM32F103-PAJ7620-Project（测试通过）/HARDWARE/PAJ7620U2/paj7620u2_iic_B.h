#ifndef __PAJ7620U2_IIC_H
#define __PAJ7620U2_IIC_H

#include "sys.h"

#define GS_B_SDA_IN()  {GPIOF->CRH&=0xFFFFFFF0;GPIOF->CRH|=4<<0;}	//PF8  浮空输入
#define GS_B_SDA_OUT() {GPIOF->CRH&=0xFFFFFFF0;GPIOF->CRH|=3<<0;}	//PF8  推挽输出（通用）

//IO操作函数	 
#define GS_B_IIC_SCL    PFout(7) 		//SCL
#define GS_B_IIC_SDA    PFout(8) 		//SDA	 
#define GS_B_READ_SDA   PFin(8) 		//输入SDA 

u8 GS_Write_Byte(u8 REG_Address,u8 REG_data);
u8 GS_Read_Byte(u8 REG_Address);
u8 GS_Read_nByte(u8 REG_Address,u16 len,u8 *buf);
void GS_i2c_init(void);
void GS_WakeUp(void);
#endif


