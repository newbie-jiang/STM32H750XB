#include "paj7620u2_B.h"
#include "paj7620u2_cfg.h"
#include "delay.h"
#include "usart.h"
#include "led.h"

//选择PAJ7620U2 BANK区域
void paj7620u2_selectBank(bank_e bank)
{
	switch(bank)
	{
		case BANK0: GS_Write_Byte(PAJ_REGITER_BANK_SEL,PAJ_BANK0);break;//BANK0寄存器区域
		case BANK1: GS_Write_Byte(PAJ_REGITER_BANK_SEL,PAJ_BANK1);break;//BANK1寄存器区域
	}	
}

//PAJ7620U2唤醒
u8 paj7620u2_wakeup(void)
{ 
	u8 data=0x0a;
	GS_WakeUp();//唤醒PAJ7620U2
	delay_ms(5);//唤醒时间>400us
	GS_WakeUp();//唤醒PAJ7620U2
	delay_ms(5);//唤醒时间>400us
	paj7620u2_selectBank(BANK0);//进入BANK0寄存器区域
	data = GS_Read_Byte(0x00);//读取状态
	if(data!=0x20) return 0; //唤醒失败
	
	return 1;
}

//PAJ7620U2初始化
//返回值：0:失败 1:成功
u8 paj7620u2_init(void)
{
	u8 i;
	u8 status;
	
	GS_i2c_init();//IIC初始化
    status = paj7620u2_wakeup();//唤醒PAJ7620U2
	if(!status) return 0;
	paj7620u2_selectBank(BANK0);//进入BANK0寄存器区域
	for(i=0;i<INIT_SIZE;i++)
	{
		GS_Write_Byte(init_Array[i][0],init_Array[i][1]);//初始化PAJ7620U2
	}
    paj7620u2_selectBank(BANK0);//切换回BANK0寄存器区域
	
	return 1;
}

//手势识别测试
void Gesture_test(void)
{
	u8 i;
    u8 status;
	u8 key;
	u8 data[2]={0x00};
	u16 gesture_data;
	u8 ledflash=0;
	
	paj7620u2_selectBank(BANK0);//进入BANK0寄存器区域
	for(i=0;i<GESTURE_SIZE;i++)
	{
		GS_Write_Byte(gesture_arry[i][0],gesture_arry[i][1]);//手势识别模式初始化
	}
	paj7620u2_selectBank(BANK0);//切换回BANK0寄存器区域
	i=0;
	
	while(1)
	{
    key = KEY_Scan(0);
		if(key==WKUP_PRES)
		{
			GS_Write_Byte(PAJ_SET_INT_FLAG1,0X00);//关闭手势识别中断输出
			GS_Write_Byte(PAJ_SET_INT_FLAG2,0X00);
			break;
		}			
        status = GS_Read_nByte(PAJ_GET_INT_FLAG1,2,&data[0]);//读取手势状态			
		if(!status)
		{   
			gesture_data =(u16)data[1]<<8 | data[0];
			if(gesture_data) 
			{
				switch(gesture_data)
				{
					case GES_UP:               LCD_ShowString(110,250,200,16,24,"UP          ");
					                           printf("Up\r\n");            ledflash=1;      break; //向上
					case GES_DOWM:             LCD_ShowString(100,250,200,16,24,"Dowm        ");      
               						           printf("Down\r\n");          ledflash=1;      break; //向下
					case GES_LEFT:             LCD_ShowString(100,250,200,16,24,"Left        ");           
  						                       printf("Left\r\n");          ledflash=1;      break; //向左
					case GES_RIGHT:            LCD_ShowString(100,250,200,16,24,"Right       ");       
                						       printf("Right\r\n");         ledflash=1;      break; //向右
					case GES_FORWARD:          LCD_ShowString(80,250,200,16,24,"Forward     ");        
						                       printf("Forward\r\n");       ledflash=1;      break; //向前
					case GES_BACKWARD:         LCD_ShowString(80,250,200,16,24,"Backward    ");    
            						           printf("Backward\r\n");      ledflash=1;      break; //向后
					case GES_CLOCKWISE:        LCD_ShowString(70,250,200,16,24,"Clockwise   ");     
                						       printf("Clockwise\r\n");     ledflash=1;      break; //顺时针
					case GES_COUNT_CLOCKWISE:  LCD_ShowString(50,250,200,16,24,"AntiClockwise");  
                   						       printf("AntiClockwise\r\n"); ledflash=1;      break; //逆时针
					case GES_WAVE:             LCD_ShowString(100,250,200,16,24,"Wave         ");    
						                       printf("Wave\r\n");          ledflash=1;      break; //挥动
					default:  ledflash=0; break;
					
				}	
                if(ledflash)//DS1闪烁
				{   
					LED1=0;delay_ms(80);LED1=1;delay_ms(80);
					LED1=0;delay_ms(80);LED1=1;delay_ms(80);
					LCD_ShowString(40,250,200,16,24,"                        ");
					ledflash=0;
				}						
			}
			
		}
		delay_ms(50);
		i++;
		if(i==5)
		{
			LED0=!LED0;//提示系统正在运行	
			i=0;
		}		   
	}
}

//接近检测测试
void Ps_test(void)
{
	u8 i;
	u8 key;
	u8 data[2]={0x00};
	u8 obj_brightness=0;
	u16 obj_size=0;
	
	paj7620u2_selectBank(BANK0);//进入BANK0寄存器区域
	for(i=0;i<PROXIM_SIZE;i++)
	{
		GS_Write_Byte(proximity_arry[i][0],proximity_arry[i][1]);//接近检测模式初始化
	}
	paj7620u2_selectBank(BANK0);//切换回BANK0寄存器区域
	i=0;
	POINT_COLOR=BLUE;//设置字体为蓝色
	LCD_Fill(30,170,300,300,WHITE);
	LCD_ShowString(30,180,200,16,16,"KEY_UP: Exit the test");
	LCD_ShowString(30,210,200,16,16,"Ps test");
	LCD_ShowString(30,240,200,16,16,"Brightness");
	LCD_ShowString(160,240,200,16,16,"Size");
	POINT_COLOR=RED;//设置字体为蓝色	
	
	while(1)
	{	
		key = KEY_Scan(0);
		if(key==WKUP_PRES) break;
		
		obj_brightness = GS_Read_Byte(PAJ_GET_OBJECT_BRIGHTNESS);//读取物体亮度
		data[0] = GS_Read_Byte(PAJ_GET_OBJECT_SIZE_1);//读取物体大小
		data[1] = GS_Read_Byte(PAJ_GET_OBJECT_SIZE_2);
		obj_size = ((u16)data[1] & 0x0f)<<8 | data[0];
		LCD_ShowxNum(50,270,obj_brightness,3,24,0);
		LCD_ShowxNum(152,270,obj_size,3,24,0);
		printf("obj_brightness = %d\r\n",obj_brightness);
        printf("obj_size:%d\r\n",obj_size);
		
		delay_ms(100);
		i++;
		if(i==5)
		{
		    LED0=!LED0;//提示系统正在运行	
			i=0;
		}
	}
}

//PAJ7620U2传感器测试
void paj7620u2_sensor_test(void)
{   
	 u8 i=0;
	 while(1)
	 {
			Gesture_test();//手势检测模式  
			delay_ms(50);
			i++;
			if(i==5)
			{
				LED0=!LED0;//提示系统正在运行	
				i=0;
			}
			break;
	 }
}
