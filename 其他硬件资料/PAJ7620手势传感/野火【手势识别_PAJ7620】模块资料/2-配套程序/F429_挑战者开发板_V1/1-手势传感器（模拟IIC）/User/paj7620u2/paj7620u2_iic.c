#include "paj7620u2_iic.h"
#include "paj7620u2.h"
#include "./SysTick/bsp_SysTick.h"

/**
  * @brief  PAJ7620U2初始化
  * @param  无
  * @retval 无
  */
void GS_i2c_init(void)
{
	GPIO_InitTypeDef GPIO_InitStructure;

	RCC_AHB1PeriphClockCmd(GS_I2C_GPIO_CLK, ENABLE);	/* 打开GPIO时钟 */

	GPIO_InitStructure.GPIO_Pin =GS_I2C_SCL_PIN| GS_I2C_SDA_PIN;
	GPIO_InitStructure.GPIO_Speed = GPIO_Speed_50MHz;
	GPIO_InitStructure.GPIO_Mode = GPIO_Mode_OUT;  	/* 开漏输出 */
  GPIO_InitStructure.GPIO_OType = GPIO_OType_PP;  //推挽输出
	GPIO_Init(GS_I2C_GPIO_PORT, &GPIO_InitStructure);
	
  GPIO_SetBits(GS_I2C_GPIO_PORT,GS_I2C_SCL_PIN| GS_I2C_SDA_PIN);//PB10,PB11 输出高	
}

/**
  * @brief  产生IIC起始信号
  * @param  无
  * @retval 无
  */
static void GS_IIC_Start(void)
{
	GS_SDA_OUT();   //sda线输出
	GS_I2C_SDA_1()  ;	  	  
	GS_I2C_SCL_1();
	delay_us(4);
 	GS_I2C_SDA_0()  ;//START:when CLK is high,DATA change form high to low 
	delay_us(4);
	GS_I2C_SCL_0();  //钳住I2C总线，准备发送或接收数据 
}

/**
  * @brief  产生IIC停止信号
  * @param  无
  * @retval 无
  */
static void GS_IIC_Stop(void)
{
	GS_SDA_OUT();    //sda线输出
	GS_I2C_SCL_0();
	GS_I2C_SDA_0()  ;//STOP:when CLK is high DATA change form low to high
 	delay_us(4);
	GS_I2C_SCL_1(); 
	GS_I2C_SDA_1()  ;//发送I2C总线结束信号
	delay_us(4);							   	
}
      
/**
  * @brief  等待应答信号到来
  * @param  无
  * @retval 1:接收应答失败，0:接收应答成功
  */
static uint8_t GS_IIC_Wait_Ack(void)
{
	uint8_t ucErrTime=0;
	GS_SDA_IN();  //SDA设置为输入  
	GS_I2C_SDA_1()  ;
  delay_us(3);	   
	GS_I2C_SCL_1()  ;
  delay_us(3);	 
	while(GS_I2C_SDA_READ())
	{
		ucErrTime++;
		if(ucErrTime>250)
		{
			GS_IIC_Stop();
			return 1;
		}
	}
	GS_I2C_SCL_0();//时钟输出0 	   
	return 0;  
}

/**
  * @brief  产生ACK应答
  * @param  无
  * @retval 无
  */
static void GS_IIC_Ack(void)
{
	GS_I2C_SCL_0();
	GS_SDA_OUT();
	GS_I2C_SDA_0()  ;
	delay_us(3);
	GS_I2C_SCL_1();
	delay_us(3);
	GS_I2C_SCL_0();
}

/**
  * @brief  不产生ACK应答
  * @param  无
  * @retval 无
  */
static void GS_IIC_NAck(void)
{
	GS_I2C_SCL_0();
	GS_SDA_OUT();
	GS_I2C_SDA_1()  ;
	delay_us(2);
	GS_I2C_SCL_1();
	delay_us(2);
	GS_I2C_SCL_0();
}

/**
  * @brief  IIC发送一个字节
  * @param  传输的字节
  * @retval 1:有应答,0:无应答
  */
static void GS_IIC_Send_Byte(uint8_t txd)
{                        
    uint8_t t;   
	  GS_SDA_OUT(); 	    
    GS_I2C_SCL_0();//拉低时钟开始数据传输
    for(t=0;t<8;t++)
    {              
		 if((txd&0x80)>>7)
			 GS_I2C_SDA_1()  ;
		 else
			 GS_I2C_SDA_0()  ;
      txd<<=1; 	  
      delay_us(5);  
      GS_I2C_SCL_1();
      delay_us(5); 
      GS_I2C_SCL_0();	
      delay_us(5);
    }	 
} 

/**
  * @brief  读1个字节
  * @param  ack=1时，发送ACK，ack=0，发送nACK
  * @retval 接收到的数据
  */
static uint8_t GS_IIC_Read_Byte(uint8_t ack)
{
	uint8_t i,receive=0;
	GS_SDA_IN();//SDA设置为输入
	for(i=0;i<8;i++ )
	{
		GS_I2C_SCL_0(); 
		delay_us(4);
	  GS_I2C_SCL_1();
		receive<<=1;
		if(GS_I2C_SDA_READ()) receive++;   
	  delay_us(4); 
	}					 
	if (!ack)
		GS_IIC_NAck();//发送nACK
	else
		GS_IIC_Ack(); //发送ACK   
	return receive;
}

/**
  * @brief  PAJ7620U2写一个字节数据
  * @param  寄存器地址，寄存器数据
  * @retval 1:成功，0:失败
  */
uint8_t GS_Write_Byte(uint8_t REG_Address,uint8_t REG_data)
{
	GS_IIC_Start();
	GS_IIC_Send_Byte(PAJ7620_ID);
	if(GS_IIC_Wait_Ack())
	{
		GS_IIC_Stop();//释放总线
		return 1;//没应答则退出
	}
	GS_IIC_Send_Byte(REG_Address);
	GS_IIC_Wait_Ack();	
	GS_IIC_Send_Byte(REG_data);
	GS_IIC_Wait_Ack();	
	GS_IIC_Stop();

	return 0;
}

/**
  * @brief  PAJ7620U2读一个字节数据
  * @param  寄存器地址
  * @retval 接收到数据
  */
uint8_t GS_Read_Byte(uint8_t REG_Address)
{
	uint8_t REG_data;
	
	GS_IIC_Start();
	GS_IIC_Send_Byte(PAJ7620_ID);//发写命令
	if(GS_IIC_Wait_Ack())
	{
		GS_IIC_Stop();//释放总线
		return 0;     //没应答则退出
	}		
	GS_IIC_Send_Byte(REG_Address);
	GS_IIC_Wait_Ack();
	GS_IIC_Start(); 
	GS_IIC_Send_Byte(PAJ7620_ID|0x01);//发读命令
	GS_IIC_Wait_Ack();
	REG_data = GS_IIC_Read_Byte(0);
	GS_IIC_Stop();

	return REG_data;
}

/**
  * @brief  PAJ7620U2读n个字节数据
  * @param  寄存器地址，数据字节数，数据指针
  * @retval 接收到数据
  */
uint8_t GS_Read_nByte(uint8_t REG_Address,uint16_t len,uint8_t *buf)
{
	GS_IIC_Start();
	GS_IIC_Send_Byte(PAJ7620_ID);//发写命令
	if(GS_IIC_Wait_Ack()) 
	{
		GS_IIC_Stop();//释放总线
		return 1;//没应答则退出
	}
	GS_IIC_Send_Byte(REG_Address);
	GS_IIC_Wait_Ack();

	GS_IIC_Start();
	GS_IIC_Send_Byte(PAJ7620_ID|0x01);//发读命令
	GS_IIC_Wait_Ack();
	while(len)
	{
		if(len==1)
		{
			*buf = GS_IIC_Read_Byte(0);
		}
		else
		{
			*buf = GS_IIC_Read_Byte(1);
		}
		buf++;
		len--;
	}
	GS_IIC_Stop();               //释放总线

	return 0;	
}

/**
  * @brief  PAJ7620U2唤醒
  * @param  无
  * @retval 无
  */
void GS_WakeUp(void)
{
	GS_IIC_Start();
	GS_IIC_Send_Byte(PAJ7620_ID);//发写命令
	GS_IIC_Stop();               //释放总线
}
