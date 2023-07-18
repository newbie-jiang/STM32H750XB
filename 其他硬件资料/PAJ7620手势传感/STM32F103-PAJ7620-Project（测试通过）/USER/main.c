#include "sys.h"
#include "delay.h"
#include "led.h"				//LED0、LED1
#include "usart.h"			//串口1调试信息打印

#include "PAJ7620U2.h"	//PAJ7620
#include "timer.h"			//定时器中断

//////////////////////////////////////////////////////////////////////////////////	 
//本程序只供学习使用，未经作者许可，不得用于其它任何用途
//STM32F10x开发板  PAJ7620传感器模块驱动	
//STM32学习有问题，请加入q群交流: 643807576

void PAJ7620_Init(void);

int main(void)
{		
	delay_init();	    	 //延时函数初始化	  
	NVIC_PriorityGroupConfig(NVIC_PriorityGroup_2); //设置NVIC中断分组2:2位抢占优先级，2位响应优先级
	
	uart_init(115200);	    //串口初始化为115200
 	LED_Init();			    		//LED端口初始化
	PAJ7620_Init();					//LED1亮起、串口输出“PAJ7620U2 OK”--->说明PAJ7620模块初始化正常
	 
	//TIM2_Int_Init(1299,7199); //130ms定时--手势传感数据采集，130ms是手势类型采样时间，采样时间越小，识别反应越灵敏，可以自己调一下采样时间到合适。
	
	while(1)
	{
		delay_ms(200);
	

		//手势识别
		gesture->detect = GS_Read_nByte(PAJ_GET_INT_FLAG1,2,&gesture->data[0]);//读取手势状态			
		if(!gesture->detect)
		{   
			gesture->type =(u16)gesture->data[1]<<8 | gesture->data[0];
			if(gesture->type) 
			{
				switch(gesture->type)
				{
					case GES_UP:               printf("Up\r\n");            gesture->valid=1;      break; //向上
					case GES_DOWN:             printf("Down\r\n");          gesture->valid=1;     break; //向下
					case GES_LEFT:            printf("Left\r\n");          gesture->valid=1;    break; //向左
					case GES_RIGHT:          	printf("Right\r\n");         gesture->valid=1;    break; //向右
					case GES_FORWARD:       	printf("Forward\r\n");       gesture->valid=1;   break; //向前
					case GES_BACKWARD:      		printf("Backward\r\n");      gesture->valid=1;   break; //向后
					case GES_CLOCKWISE:     		printf("Clockwise\r\n");     gesture->valid=1;   break; //顺时针
					case GES_ANTI_CLOCKWISE:  printf("AntiClockwise\r\n"); gesture->valid=1;    break; //逆时针
					case GES_WAVE:           printf("Wave\r\n");     gesture->valid=1;    break; //挥动
					default:  																				gesture->valid=0;   break;	
				}
			}
		}
	
		
		
		;
	}
}

void  PAJ7620_Init(void)
{
	while(!paj7620u2_init())//PAJ7620U2传感器初始化
	{
		printf("PAJ7620U2_B Error!!!\r\n");
		delay_ms(500);
		LED0=!LED0;//DS0闪烁
	}
	LED1 =~LED1;	
	delay_ms(1000);
	Gesture_Init();
  printf("PAJ7620U2 OK\r\n");
}
