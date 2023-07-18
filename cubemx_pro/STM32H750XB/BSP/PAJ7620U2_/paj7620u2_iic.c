#include "paj7620u2_iic.h"
#include "paj7620u2.h"
#include "stdio.h"

void delay_us(uint16_t us)
{
 static unsigned int i=0;
 for ( i=0;i<us*14;++i);
}


/**
PH11  SCL
PH12  SDA
*/

void SDA_IN(void)
{
  GPIO_InitTypeDef GPIO_InitStruct = {0};
	__HAL_RCC_GPIOH_CLK_ENABLE();
	//HAL_GPIO_WritePin(GPIOH, SCL_PAJ7620_Pin|SDA_PAJ7620_Pin, GPIO_PIN_SET);
	/*Configure GPIO pins : PHPin PHPin */
  GPIO_InitStruct.Pin = SDA_PAJ7620_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_INPUT;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_HIGH;
  HAL_GPIO_Init(GPIOH, &GPIO_InitStruct);
}

void SDA_OUT(void)
{
  GPIO_InitTypeDef GPIO_InitStruct = {0};
	__HAL_RCC_GPIOH_CLK_ENABLE();
	//HAL_GPIO_WritePin(GPIOH, SCL_PAJ7620_Pin|SDA_PAJ7620_Pin, GPIO_PIN_SET);
	/*Configure GPIO pins : PHPin PHPin */
  GPIO_InitStruct.Pin = SDA_PAJ7620_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_PULLUP;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_HIGH;
  HAL_GPIO_Init(GPIOH, &GPIO_InitStruct);
}


uint8_t READ_SDA(void)
{
 return HAL_GPIO_ReadPin(GPIOH,SDA_PAJ7620_Pin);
}



//产生IIC起始信号
static void GS_IIC_Start()
{
	GS_SDA_OUT();//sda线输出
	GS_IIC_SDA(1);	  	  
	GS_IIC_SCL(1);
	delay_us(4);
	GS_IIC_SDA(0);//START:when CLK is high,DATA change form high to low 
	delay_us(4);
	GS_IIC_SCL(0);//钳住I2C总线，准备发送或接收数据 
}

//产生IIC停止信号
static void GS_IIC_Stop()
{
	GS_SDA_OUT();//sda线输出
	GS_IIC_SCL(0);
	GS_IIC_SDA(0);//STOP:when CLK is high DATA change form low to high
	delay_us(4);
	GS_IIC_SCL(1); 
	GS_IIC_SDA(1);//发送I2C总线结束信号
	delay_us(4);				   	
}

//等待应答信号到来
//返回值：1，接收应答失败
//        0，接收应答成功
static uint8_t GS_IIC_Wait_Ack()
{
	
	uint8_t ucErrTime=0;
	GS_SDA_IN();  //SDA设置为输入  
	GS_IIC_SDA(1);delay_us(3);	   
	GS_IIC_SCL(1);delay_us(3);	 
	while(READ_SDA())
	{
		ucErrTime++;
		if(ucErrTime>250)
		{
			GS_IIC_Stop();
			return 1;
		}
	}
	GS_IIC_SCL(0);//时钟输出0 	
	return 0;  
}

//产生ACK应答
static void GS_IIC_Ack()
{
	GS_IIC_SCL(0);
	GS_SDA_OUT();
	GS_IIC_SDA(0);
	delay_us(3);
	GS_IIC_SCL(1);
	delay_us(3);
	GS_IIC_SCL(0);
}

//不产生ACK应答		    
static void GS_IIC_NAck()
{
	GS_IIC_SCL(0);
	GS_SDA_OUT();
	GS_IIC_SDA(1);
	delay_us(2);
	GS_IIC_SCL(1);
	delay_us(2);
	GS_IIC_SCL(0);
}

//IIC发送一个字节
//返回从机有无应答
//1，有应答
//0，无应答			  
static void GS_IIC_Send_Byte(uint8_t txd)
{                        
	uint8_t t;   
	GS_SDA_OUT(); 	    
	GS_IIC_SCL(0);//拉低时钟开始数据传输
	for(t=0;t<8;t++)
	{              
		if((txd&0x80)>>7)
			GS_IIC_SDA(1);
		else
			GS_IIC_SDA(0);
		txd<<=1; 	
    
		delay_us(5); 
   	
		GS_IIC_SCL(1);
		delay_us(5); 
		GS_IIC_SCL(0);	
		delay_us(5);
	}	 
} 

//读1个字节，ack=1时，发送ACK，ack=0，发送nACK   
static uint8_t GS_IIC_Read_Byte(uint8_t ack)
{
	
	uint8_t i,receive=0;

	GS_SDA_IN();//SDA设置为输入
	for(i=0;i<8;i++ )
	{
		GS_IIC_SCL(0); 
		delay_us(4);
		GS_IIC_SCL(1);
		receive<<=1;
		if(READ_SDA())receive++;   
		delay_us(4); 
	}					 
	if (!ack)
		GS_IIC_NAck();//发送nACK
	else
		GS_IIC_Ack(); //发送ACK   
	
	return receive;
}

//PAJ7620U2写一个字节数据
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

//PAJ7620U2读一个字节数据
uint8_t GS_Read_Byte(uint8_t REG_Address)
{
	uint8_t REG_data;
	
	GS_IIC_Start();
	GS_IIC_Send_Byte(PAJ7620_ID);//发写命令
	if(GS_IIC_Wait_Ack())
	{
		 GS_IIC_Stop();//释放总线
		 return 0;//没应答则退出
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
//PAJ7620U2读n个字节数据
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
	GS_IIC_Stop();//释放总线

	return 0;
	
}
//PAJ7620唤醒
void GS_WakeUp()
{
	//printf("test3\r\n");
	GS_IIC_Start();
	GS_IIC_Send_Byte(PAJ7620_ID);//发写命令
	//printf("test4\r\n");
	GS_IIC_Stop();//释放总线
}

//PAJ2670 I2C初始化
void GS_i2c_init()
{
	GPIO_InitTypeDef GPIO_InitStruct = {0};
	__HAL_RCC_GPIOH_CLK_ENABLE();
	HAL_GPIO_WritePin(GPIOH, SCL_PAJ7620_Pin|SDA_PAJ7620_Pin, GPIO_PIN_SET);
	/*Configure GPIO pins : PHPin PHPin */
  GPIO_InitStruct.Pin = SCL_PAJ7620_Pin|SDA_PAJ7620_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_PULLUP;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_HIGH;
  HAL_GPIO_Init(GPIOH, &GPIO_InitStruct);
}
