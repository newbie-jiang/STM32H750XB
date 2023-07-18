#include "paj7620u2.h"
#include "paj7620u2_cfg.h"
#include "./usart/bsp_usart.h"
#include "./SysTick/bsp_SysTick.h"
#include "paj7620u2_iic.h"

uint8_t  flag=0;

/**
  * @brief  选择PAJ7620U2 BANK区域
  * @param  无
  * @retval 无
  */
void paj7620u2_selectBank(bank_e bank)
{
	switch(bank)
	{
		case BANK0: GS_Write_Byte(PAJ_REGITER_BANK_SEL,PAJ_BANK0);break;//BANK0寄存器区域
		case BANK1: GS_Write_Byte(PAJ_REGITER_BANK_SEL,PAJ_BANK1);break;//BANK1寄存器区域
	}			
}

/**
  * @brief  PAJ7620U2唤醒
  * @param  无
  * @retval 0:失败 1:成功
  */
uint8_t paj7620u2_wakeup(void)
{ 
	uint8_t data=0x0a;
	GS_WakeUp();//唤醒PAJ7620U2
	delay_ms(5);//唤醒时间>700us
	GS_WakeUp();//唤醒PAJ7620U2
	delay_ms(5);//唤醒时间>700us
	paj7620u2_selectBank(BANK0);//进入BANK0寄存器区域
	data = GS_Read_Byte(0x00);//读取状态
	if(data!=0x20) return 0; //唤醒失败
	
	return 1;
}


/**
  * @brief  PAJ7620U2初始化
  * @param  无
  * @retval 0:失败 1:成功
  */
uint8_t paj7620u2_init(void)
{
	uint8_t i;
	uint8_t status;
	
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

/**
  * @brief  手势识别测试
  * @param  无
  * @retval 无
  */
void Gesrure_test(void)
{
  uint8_t status,i;
	uint8_t data[2]={0x00};
	uint16_t gesture_data;

	paj7620u2_selectBank(BANK0);//进入BANK0
	for(i=0;i<GESTURE_SIZE;i++)
	{
		GS_Write_Byte(gesture_arry[i][0],gesture_arry[i][1]);//手势识别模式初始化
	}
	paj7620u2_selectBank(BANK0);//切换回BANK0
  printf("按KEY1键退出当前模式\r\n");
	
	while(1)
	{       
		if(flag)
		{
      flag=0;
			GS_Write_Byte(PAJ_SET_INT_FLAG1,0X00);//关闭手势识别中断输出
			GS_Write_Byte(PAJ_SET_INT_FLAG2,0X00);
			break;
		}			
    status = GS_Read_nByte(PAJ_GET_INT_FLAG1,2,&data[0]);//读取手势状态			
		if(!status)
		{   
			gesture_data =(uint16_t)data[1]<<8 | data[0];
			if(gesture_data) 
			{
				switch(gesture_data)
				{
					case GES_UP:              
					                           printf("Up\r\n");                break; //向上
					case GES_DOWM:                 
               						           printf("Dowm\r\n");              break; //向下
					case GES_LEFT:                     
  						                       printf("Left\r\n");              break; //向左
					case GES_RIGHT:                  
                						         printf("Right\r\n");             break; //向右
					case GES_FORWARD:                 
						                         printf("Forward\r\n");           break; //向前
					case GES_BACKWARD:           
            						             printf("Backward\r\n");          break; //向后
					case GES_CLOCKWISE:         
                						         printf("Clockwise\r\n");         break; //顺时针
					case GES_COUNT_CLOCKWISE:  
                   						       printf("AntiClockwise\r\n");     break; //逆时针
					case GES_WAVE:               
						                         printf("Wave\r\n");              break; //挥动
					default:  break;	
				}	          
			}	
		}		   
	}
}

/**
  * @brief  接近检测测试
  * @param  无
  * @retval 无
  */
void Ps_test(void)
{
  
  uint8_t i;
	uint8_t data[2]={0x00};
	uint8_t obj_brightness=0;
	uint16_t obj_size=0;
	
	paj7620u2_selectBank(BANK0);//进入BANK0
	for(i=0;i<PROXIM_SIZE;i++)
	{
		GS_Write_Byte(proximity_arry[i][0],proximity_arry[i][1]);//接近检测模式初始化
	}
	paj7620u2_selectBank(BANK0);//切换回BANK0
	printf("按KEY1键退出当前模式\r\n");
	
	while(1)
	{	
		if(flag) 
    {
      flag=0;
      break;
    }
		obj_brightness = GS_Read_Byte(PAJ_GET_OBJECT_BRIGHTNESS);//读取物体亮度
		data[0] = GS_Read_Byte(PAJ_GET_OBJECT_SIZE_1);//读取物体大小
		data[1] = GS_Read_Byte(PAJ_GET_OBJECT_SIZE_2);
		obj_size = ((uint16_t)data[1] & 0x0f)<<8 | data[0];
		printf("obj_brightness: %d\r\n",obj_brightness);
    printf("obj_size: %d\r\n",obj_size);
	
		delay_ms(500);	
	}	
}

/**
  * @brief  指令信息
  * @param  无
  * @retval 无
  */
void Show_Mode_Message(void)
{ 
  printf("   指令   ------      功能 \r\n");
  printf("    1     ------    手势检测模式 \r\n"); 
  printf("    2     ------    接近检测模式 \r\n");
}

/**
  * @brief  PAJ7620U2传感器测试
  * @param  无
  * @retval 无
  */
void paj7620u2_sensor_test(void)
{   
	uint32_t   ch;
  
	Show_Mode_Message();//显示普通测量模式UI
	while(1)
	{
		scanf("%d",&ch);
    printf("接收到字符：%d\r\n",ch);

		switch(ch)
		{
			case 1:  Gesrure_test();                 break;//手势检测模式
			case 2:  Ps_test();                      break;//接近检测模式 
      default: printf("请输入合法指令！\r\n");  break;
		}
		Show_Mode_Message();
	} 
}

