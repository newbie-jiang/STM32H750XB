#include "timer.h"
#include "USART.h"

#include "PAJ7620U2.h"

//通用定时器2中断初始化    //这里时钟选择为APB1的2倍，而APB1为36M
//arr：自动重装值。
//psc：时钟预分频数
void TIM2_Int_Init(u16 arr,u16 psc)
{
  TIM_TimeBaseInitTypeDef  TIM_TimeBaseStructure;
	NVIC_InitTypeDef NVIC_InitStructure;

	RCC_APB1PeriphClockCmd(RCC_APB1Periph_TIM2, ENABLE); //时钟使能
	
	//定时器TIM2初始化
	TIM_TimeBaseStructure.TIM_Period = arr; //设置在下一个更新事件装入活动的自动重装载寄存器周期的值	
	TIM_TimeBaseStructure.TIM_Prescaler =psc; //设置用来作为TIMx时钟频率除数的预分频值
	TIM_TimeBaseStructure.TIM_ClockDivision = TIM_CKD_DIV1; //设置时钟分割:TDTS = Tck_tim
	TIM_TimeBaseStructure.TIM_CounterMode = TIM_CounterMode_Up;  //TIM向上计数模式
	TIM_TimeBaseInit(TIM2, &TIM_TimeBaseStructure); //根据指定的参数初始化TIMx的时间基数单位
 
	TIM_ITConfig(TIM2,TIM_IT_Update,ENABLE ); //使能指定的TIM2中断,允许更新中断

	//中断优先级NVIC设置
	NVIC_InitStructure.NVIC_IRQChannel = TIM2_IRQn;  //TIM2中断
	NVIC_InitStructure.NVIC_IRQChannelPreemptionPriority = 0;  //先占优先级0级
	NVIC_InitStructure.NVIC_IRQChannelSubPriority = 0;  //从优先级1级
	NVIC_InitStructure.NVIC_IRQChannelCmd = ENABLE; //IRQ通道被使能
	NVIC_Init(&NVIC_InitStructure);  //初始化NVIC寄存器


	TIM_Cmd(TIM2, ENABLE);  //使能TIMx					 
}

//定时器2中断服务程序			/* 手势类型识别 */
void TIM2_IRQHandler(void)   //TIM2中断
{
	if (TIM_GetITStatus(TIM2, TIM_IT_Update) != RESET)  //检查TIM2更新中断发生与否
	{
		TIM_ClearITPendingBit(TIM2, TIM_IT_Update  );  //清除TIMx更新中断标志 

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
	}
}
