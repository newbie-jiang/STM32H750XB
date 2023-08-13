/* USER CODE BEGIN Header */
/**
  ******************************************************************************
  * @file    stm32h7xx_it.c
  * @brief   Interrupt Service Routines.
  ******************************************************************************
  * @attention
  *
  * Copyright (c) 2023 STMicroelectronics.
  * All rights reserved.
  *
  * This software is licensed under terms that can be found in the LICENSE file
  * in the root directory of this software component.
  * If no LICENSE file comes with this software, it is provided AS-IS.
  *
  ******************************************************************************
  */
/* USER CODE END Header */

/* Includes ------------------------------------------------------------------*/
#include "main.h"
#include "stm32h7xx_it.h"
/* Private includes ----------------------------------------------------------*/
/* USER CODE BEGIN Includes */
#include <string.h>
#include "bsp.h"
#include "app.h"
#include "stdio.h"
#include "usart.h"
#include "irda_nec.h"
#include "lcd.h"
/* USER CODE END Includes */

/* Private typedef -----------------------------------------------------------*/
/* USER CODE BEGIN TD */
 volatile uint32_t buttonPressTime = 0;
 volatile uint8_t buttonPressCount = 0;
 int buzzer_flag;
 
 uint16_t irda_count=0; /*0-65535*/
 uint16_t tick_us_count;
 
 bool isValueChanged = false;  // 标记变量是否改变
 bool nec_ValueChanged = false;  // 标记变量是否改变
 
bool GPIO_TS_IIC_INT_Pin_state = false;
uint8_t irda_code_flag;

uint32_t IC_TIMES;  // 捕获次数，单位1ms
uint8_t IC_START_FLAG;  // 捕获开始标志，1：已捕获到高电平；0：还没有捕获到高电平
uint8_t IC_DONE_FLAG;  // 捕获完成标志，1：已完成一次高电平捕获
uint16_t IC_VALUE;  // 输入捕获的捕获值
/* USER CODE END TD */

/* Private define ------------------------------------------------------------*/
/* USER CODE BEGIN PD */

/* USER CODE END PD */

/* Private macro -------------------------------------------------------------*/
/* USER CODE BEGIN PM */

/* USER CODE END PM */

/* Private variables ---------------------------------------------------------*/
/* USER CODE BEGIN PV */

/* USER CODE END PV */

/* Private function prototypes -----------------------------------------------*/
/* USER CODE BEGIN PFP */

/* USER CODE END PFP */

/* Private user code ---------------------------------------------------------*/
/* USER CODE BEGIN 0 */

/* USER CODE END 0 */

/* External variables --------------------------------------------------------*/
extern PCD_HandleTypeDef hpcd_USB_OTG_FS;
extern DMA_HandleTypeDef hdma_adc3;
extern LTDC_HandleTypeDef hltdc;
extern TIM_HandleTypeDef htim5;
extern DMA_HandleTypeDef hdma_uart4_tx;
extern TIM_HandleTypeDef htim7;

/* USER CODE BEGIN EV */

/* USER CODE END EV */

/******************************************************************************/
/*           Cortex Processor Interruption and Exception Handlers          */
/******************************************************************************/
/**
  * @brief This function handles Non maskable interrupt.
  */
void NMI_Handler(void)
{
  /* USER CODE BEGIN NonMaskableInt_IRQn 0 */

  /* USER CODE END NonMaskableInt_IRQn 0 */
  /* USER CODE BEGIN NonMaskableInt_IRQn 1 */
  while (1)
  {
  }
  /* USER CODE END NonMaskableInt_IRQn 1 */
}

/**
  * @brief This function handles Hard fault interrupt.
  */
void HardFault_Handler(void)
{
  /* USER CODE BEGIN HardFault_IRQn 0 */

  /* USER CODE END HardFault_IRQn 0 */
  while (1)
  {
    /* USER CODE BEGIN W1_HardFault_IRQn 0 */
    /* USER CODE END W1_HardFault_IRQn 0 */
  }
}

/**
  * @brief This function handles Memory management fault.
  */
void MemManage_Handler(void)
{
  /* USER CODE BEGIN MemoryManagement_IRQn 0 */

  /* USER CODE END MemoryManagement_IRQn 0 */
  while (1)
  {
    /* USER CODE BEGIN W1_MemoryManagement_IRQn 0 */
    /* USER CODE END W1_MemoryManagement_IRQn 0 */
  }
}

/**
  * @brief This function handles Pre-fetch fault, memory access fault.
  */
void BusFault_Handler(void)
{
  /* USER CODE BEGIN BusFault_IRQn 0 */

  /* USER CODE END BusFault_IRQn 0 */
  while (1)
  {
    /* USER CODE BEGIN W1_BusFault_IRQn 0 */
    /* USER CODE END W1_BusFault_IRQn 0 */
  }
}

/**
  * @brief This function handles Undefined instruction or illegal state.
  */
void UsageFault_Handler(void)
{
  /* USER CODE BEGIN UsageFault_IRQn 0 */

  /* USER CODE END UsageFault_IRQn 0 */
  while (1)
  {
    /* USER CODE BEGIN W1_UsageFault_IRQn 0 */
    /* USER CODE END W1_UsageFault_IRQn 0 */
  }
}

/**
  * @brief This function handles System service call via SWI instruction.
  */
void SVC_Handler(void)
{
  /* USER CODE BEGIN SVCall_IRQn 0 */

  /* USER CODE END SVCall_IRQn 0 */
  /* USER CODE BEGIN SVCall_IRQn 1 */

  /* USER CODE END SVCall_IRQn 1 */
}

/**
  * @brief This function handles Debug monitor.
  */
void DebugMon_Handler(void)
{
  /* USER CODE BEGIN DebugMonitor_IRQn 0 */

  /* USER CODE END DebugMonitor_IRQn 0 */
  /* USER CODE BEGIN DebugMonitor_IRQn 1 */

  /* USER CODE END DebugMonitor_IRQn 1 */
}

/**
  * @brief This function handles Pendable request for system service.
  */
void PendSV_Handler(void)
{
  /* USER CODE BEGIN PendSV_IRQn 0 */

  /* USER CODE END PendSV_IRQn 0 */
  /* USER CODE BEGIN PendSV_IRQn 1 */

  /* USER CODE END PendSV_IRQn 1 */
}

/**
  * @brief This function handles System tick timer.
  */
void SysTick_Handler(void)
{
  /* USER CODE BEGIN SysTick_IRQn 0 */

  /* USER CODE END SysTick_IRQn 0 */

  /* USER CODE BEGIN SysTick_IRQn 1 */

  /* USER CODE END SysTick_IRQn 1 */
}

/******************************************************************************/
/* STM32H7xx Peripheral Interrupt Handlers                                    */
/* Add here the Interrupt Handlers for the used peripherals.                  */
/* For the available peripheral interrupt handler names,                      */
/* please refer to the startup file (startup_stm32h7xx.s).                    */
/******************************************************************************/

/**
  * @brief This function handles EXTI line3 interrupt.
  */
void EXTI3_IRQHandler(void)
{
  /* USER CODE BEGIN EXTI3_IRQn 0 */

  /* USER CODE END EXTI3_IRQn 0 */
  HAL_GPIO_EXTI_IRQHandler(WL_HOST_WAKE_Pin);
  /* USER CODE BEGIN EXTI3_IRQn 1 */

  /* USER CODE END EXTI3_IRQn 1 */
}

/**
  * @brief This function handles EXTI line4 interrupt.
  */
void EXTI4_IRQHandler(void)
{
  /* USER CODE BEGIN EXTI4_IRQn 0 */
	
  /* USER CODE END EXTI4_IRQn 0 */
  HAL_GPIO_EXTI_IRQHandler(USER_KEY_Pin);
  /* USER CODE BEGIN EXTI4_IRQn 1 */

  /* USER CODE END EXTI4_IRQn 1 */
}

/**
  * @brief This function handles DMA1 stream0 global interrupt.
  */
void DMA1_Stream0_IRQHandler(void)
{
  /* USER CODE BEGIN DMA1_Stream0_IRQn 0 */

  /* USER CODE END DMA1_Stream0_IRQn 0 */
  HAL_DMA_IRQHandler(&hdma_uart4_tx);
  /* USER CODE BEGIN DMA1_Stream0_IRQn 1 */

  /* USER CODE END DMA1_Stream0_IRQn 1 */
}

/**
  * @brief This function handles EXTI line[15:10] interrupts.
  */
void EXTI15_10_IRQHandler(void)
{
  /* USER CODE BEGIN EXTI15_10_IRQn 0 */

  /* USER CODE END EXTI15_10_IRQn 0 */
  HAL_GPIO_EXTI_IRQHandler(GPIO_TS_IIC_INT_Pin);
  /* USER CODE BEGIN EXTI15_10_IRQn 1 */

  /* USER CODE END EXTI15_10_IRQn 1 */
}

/**
  * @brief This function handles TIM5 global interrupt.
  */
void TIM5_IRQHandler(void)
{
  /* USER CODE BEGIN TIM5_IRQn 0 */

  /* USER CODE END TIM5_IRQn 0 */
  HAL_TIM_IRQHandler(&htim5);
  /* USER CODE BEGIN TIM5_IRQn 1 */
	
  
	nec_ValueChanged=true;
	
	
	uint32_t code=IC_IRDA_NEC();
	if(code!=0x00)
	{
	  printf("irda_code:%08x\r\n",code);
		if(code==0x00ff22dd)
		{
		  lcd_clear(WHITE);
		}else if(code==0x00ff30cf)
		{
			irda_code_flag=1;
		}
	  else if(code==0x00ff18e7){
			
		  irda_code_flag=2;
		
	}
		
		
		
		
		
	}
  /* USER CODE END TIM5_IRQn 1 */
}

/**
  * @brief This function handles TIM7 global interrupt.
  */
void TIM7_IRQHandler(void)
{
  /* USER CODE BEGIN TIM7_IRQn 0 */
//  static int num;
	static uint16_t buzzer_count;
  

	
// int BUZZER_ON_TIME_MS =100;
// int BUZZER_OFF_TIME_MS= 2000;
  // 调用按键处理函数
 //Button_Process();
  /* USER CODE END TIM7_IRQn 0 */
  HAL_TIM_IRQHandler(&htim7);
  /* USER CODE BEGIN TIM7_IRQn 1 */
 
	/************一个中断翻转一次***********************/
//	 num++;
//	if(num)
//	HAL_GPIO_TogglePin(PB2_GPIO_Port, PB2_Pin);
	/***************************************************/
 
	/*********************BUZZER*************************/
	if(buzzer_flag==1)
	{
	     buzzer_count++;
    if (buzzer_count == 1) {
        BUZZER_ON; 
	    }
		else if (buzzer_count >= 100) {
        BUZZER_OFF;
			
    if (buzzer_count >= 2100) {
        buzzer_count = 0;
       }
    }
   	buzzer_flag=0;	
	}
  /***************************************************/
  /* USER CODE END TIM7_IRQn 1 */
}

/**
  * @brief This function handles LTDC global interrupt.
  */
void LTDC_IRQHandler(void)
{
  /* USER CODE BEGIN LTDC_IRQn 0 */

  /* USER CODE END LTDC_IRQn 0 */
  HAL_LTDC_IRQHandler(&hltdc);
  /* USER CODE BEGIN LTDC_IRQn 1 */

  /* USER CODE END LTDC_IRQn 1 */
}

/**
  * @brief This function handles USB On The Go FS global interrupt.
  */
void OTG_FS_IRQHandler(void)
{
  /* USER CODE BEGIN OTG_FS_IRQn 0 */

  /* USER CODE END OTG_FS_IRQn 0 */
  HAL_PCD_IRQHandler(&hpcd_USB_OTG_FS);
  /* USER CODE BEGIN OTG_FS_IRQn 1 */

  /* USER CODE END OTG_FS_IRQn 1 */
}

/**
  * @brief This function handles DMAMUX2 overrun interrupt.
  */
void DMAMUX2_OVR_IRQHandler(void)
{
  /* USER CODE BEGIN DMAMUX2_OVR_IRQn 0 */

  /* USER CODE END DMAMUX2_OVR_IRQn 0 */

  /* USER CODE BEGIN DMAMUX2_OVR_IRQn 1 */

  /* USER CODE END DMAMUX2_OVR_IRQn 1 */
}

/**
  * @brief This function handles BDMA channel0 global interrupt.
  */
void BDMA_Channel0_IRQHandler(void)
{
  /* USER CODE BEGIN BDMA_Channel0_IRQn 0 */

  /* USER CODE END BDMA_Channel0_IRQn 0 */
  HAL_DMA_IRQHandler(&hdma_adc3);
  /* USER CODE BEGIN BDMA_Channel0_IRQn 1 */

  /* USER CODE END BDMA_Channel0_IRQn 1 */
}

/* USER CODE BEGIN 1 */

void key_process(void)
{
 if(HAL_GPIO_ReadPin(USER_KEY_GPIO_Port,USER_KEY_Pin)==RESET)
   {
      // 按键释放时的处理
      uint32_t currentTime = HAL_GetTick();
      uint32_t pressDuration = currentTime - buttonPressTime;

       if (pressDuration<800&&pressDuration>20) {
        // 短按键处理
				 LED_R_ON;
				

      } else {
        // 长按键处理
			   buttonPressCount = 0;
				// 执行长按键事件处理
				 LED_R_OFF;

        
      }
    }

}




uint16_t get_irda_tim(void)
{
  return irda_count;
}




void HAL_GPIO_EXTI_Callback(uint16_t GPIO_Pin)
{
	static uint16_t get_new_tim; /*获取最新时间*/
	static uint16_t get_old_tim; /*获取上次时间*/
	static uint16_t think_tim;/*间隔时间*/
	
  if(GPIO_Pin==GPIO_TS_IIC_INT_Pin)
	 {
		 get_new_tim=HAL_GetTick();
		 /**************向上溢出，考虑溢出情况**************************/	
		if(get_new_tim > get_old_tim) 
			think_tim = get_new_tim - get_old_tim;          /*无溢出*/
		else   
			think_tim = get_new_tim + 65535 - get_old_tim;    /*溢出*/
		
		get_old_tim = get_new_tim;
    /***************************************************************/
      /*检测中断时间为10ms GPIO_TS_IIC_INT_Pin_state=true*/
		
		if(think_tim>=7&&think_tim<=13)
		{
			ctp_test();
		 //GPIO_TS_IIC_INT_Pin_state=true;
		 //HAL_GPIO_TogglePin(GPIOB, PB2_Pin);
		}
		
	}
	 
//	if(GPIO_Pin==USER_KEY_Pin)
//	{
//	  key_process();
//	}


}


/******************************NEC协议******************************************************
引导码:  低电平9ms      高电平4.5ms      13.5ms
0:       低电平0.56ms   高电平0.56ms     1.12ms
1:       低电平0.56ms   高电平1.68ms     2.24ms
********************************************************************************************/
/**********************根据定时器计数可调（可调参数）***************************************/
#define   TIM_INTERRUPT_TIME        1                     /* 计一次数时间(us)*/
#define   OVERFLOW                  65535                 /* 溢出值 (变量类型相关这里无符号16位) */
/******************NEC协议解码(适用于NEC协议，非必要勿改（可动态增加范围）)***********************/
#define   START_MIN               13000/TIM_INTERRUPT_TIME    /* 引导码最小计数次数 */
#define   START_MAX               14000/TIM_INTERRUPT_TIME    /* 引导码最大计数次数 */
  
#define   SURE_0_MIN              1000/TIM_INTERRUPT_TIME     /* 0最小计数次数 */           
#define   SURE_0_MAX              1200/TIM_INTERRUPT_TIME     /* 0最大计数次数 */            
 
#define   SURE_1_MIN              2100/TIM_INTERRUPT_TIME      /* 1最小计数次数 */ 
#define   SURE_1_MAX              2350/TIM_INTERRUPT_TIME      /* 1最大计数次数 */            
/*******************************************************************************************/  

uint32_t scan_irda(void)
{
	static uint16_t get_new_tim; /*获取最新时间*/
	static uint16_t get_old_tim; /*获取上次时间*/
	static uint16_t think_tim;/*间隔时间*/
	static char irda_state;/*解码状态*/
	static uint32_t irda_decoder,get_ir_code;/*解码值*/
	static uint8_t code_flag; /*解码成功标志位*/
  static uint8_t irda_data_bit_count=32;/*红外的32个bit*/

	if(isValueChanged)/*产生下降沿中断*/
	{
	
		/*获取计数值*/
		get_new_tim=get_irda_tim();
		/**************向上溢出，考虑溢出情况**************************/	
		if(get_new_tim > get_old_tim) 
			think_tim = get_new_tim - get_old_tim;          /*无溢出*/
		else   
			think_tim = get_new_tim + OVERFLOW - get_old_tim;    /*溢出*/
		
		get_old_tim = get_new_tim;
    /***************************************************************/
		
		
	switch (irda_state)
	{
		case 0: 
			/*引导码*/
			if((think_tim>=START_MIN&&think_tim<=START_MAX))
					irda_state=1; /*引导码正确进入下一阶段*/
			else 
				//HAL_TIM_Base_Stop(&htim6);
			 break; /*非引导码直接跳出*/
	  
		case 1:
			/*判断0和1*/
	   if((think_tim>=SURE_0_MIN&&think_tim<=SURE_0_MAX))   /*判断为0   1.12ms*/
			{
				irda_decoder|=0<<(irda_data_bit_count-1); 
			  irda_data_bit_count-=1; /*红外32位数据*/
				irda_state=2;
				
			}
			else if(think_tim>=SURE_1_MIN&&think_tim<=SURE_1_MAX) /*判断为1   2.24ms*/
			{
			  irda_decoder|=1<<(irda_data_bit_count-1);
				irda_data_bit_count-=1;
				irda_state=2;
			}
			
			else /*非0和1*/
			{
			 irda_state=0; /*重新判断引导码*/
			
			}
			
			
		/*判断是否接受满32位数据，接受满则完成解码，未解码完成重新判断0和1*/	
		case 2:
			
		if(irda_data_bit_count==0)   /*是否满足32位数据*/
		{
		  code_flag=1; /*解码完成解码标志位置1*/
		  irda_data_bit_count=32; /*重新设置32bit数据*/
		  irda_state=0; /*重新判断引导码*/
	  }
	 else
			irda_state=1; /*继续判断0和1*/
		  	
	
	}
	
	 
	 isValueChanged=false;/*恢复中断标志位*/
	
	}
	
	if(code_flag==1)
	{
	  
	  get_ir_code=irda_decoder;
    code_flag=0;	
		irda_decoder=0;	
		return get_ir_code;
		
	}
	
	 return 0x00;
		
			
}





/* USER CODE END 1 */
