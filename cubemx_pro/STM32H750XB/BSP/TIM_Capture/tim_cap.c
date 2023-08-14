#include "tim_cap.h"
#include "main.h"
#include "stdio.h"

/*此种方式未做溢出处理，适合特定频率*/

uint32_t InputCaptureValue1 = 0;  //输入捕获值1
uint32_t InputCaptureValue2 = 0;  //输入捕获值2
uint8_t  CaptureNumber = 0;       //捕获次数
double_t TIM2CH1_Frequency = 0;   //频率值
double_t TIM2CH1_DutyCycle = 0;   //占空比
uint32_t TIM2_OverflowCount = 0;  //溢出次数

/*输入捕获回调函数*/
void HAL_TIM_IC_CaptureCallback(TIM_HandleTypeDef *htim)
{
	uint16_t value1=0,value2=0;
	static uint32_t count=0;
	double fre,duty;
	count++;
	 if (htim->Instance == TIM2) 
  {
	    if(htim->Channel==HAL_TIM_ACTIVE_CHANNEL_1)
		   {
		   	value1 = HAL_TIM_ReadCapturedValue(htim,TIM_CHANNEL_1);	//直接	
		   	value2 = HAL_TIM_ReadCapturedValue(htim,TIM_CHANNEL_2);	//间接
				HAL_TIM_IC_Stop(htim,TIM_CHANNEL_1);	//重新开启定时器通道
		   	HAL_TIM_IC_Stop(htim,TIM_CHANNEL_2);	//重新开启定时器通道
		   	__HAL_TIM_SetCounter(htim,0);			//计数值清零
		   	fre = (240000000.0/240)/value1;		//计算频率
		   	duty = (float)value2/value1 * 100;  //计算占空比
		   	HAL_TIM_IC_Start(htim,TIM_CHANNEL_1);	//重新开启定时器通道
		   	HAL_TIM_IC_Start(htim,TIM_CHANNEL_2);	//重新开启定时器通道
		   }

  }
	
	
   	if(count>=1000&&fre&&duty)
   	{
   	  printf("TIM2CH1_Freq=%lfHZ\n,TIM2CH1_Duty=%lf%%\r\n",fre,duty);
   	  count=0;
   	}
   		
	
		
}
