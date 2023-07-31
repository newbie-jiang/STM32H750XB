/* USER CODE BEGIN Header */
/**
  ******************************************************************************
  * @file           : main.c
  * @brief          : Main program body
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
#include "adc.h"
#include "fatfs.h"
#include "i2c.h"
#include "quadspi.h"
#include "sdmmc.h"
#include "spi.h"
#include "tim.h"
#include "usart.h"
#include "usb_device.h"
#include "gpio.h"
#include "fmc.h"

/* Private includes ----------------------------------------------------------*/
/* USER CODE BEGIN Includes */
#include "bsp.h"
#include <stdio.h>
#include "stm32h7xx_it.h"
#include "PAJ7620U2.h"	//PAJ7620
#include "PAJ7620U2_iic.h"	//PAJ7620
#include "stm32_u8g2.h"
#include "oled_test.h"
#include "dht11.h"
#include "sd.h"
#include "sdram.h"
#include "w25qxx.h"
#include "qspi_w25q64.h"
#include "usbd_storage_if.h"
#include "ap6212_wifi.h"
#include "irda_nec.h"
#include "tim_cap.h"
/* USER CODE END Includes */

/* Private typedef -----------------------------------------------------------*/
/* USER CODE BEGIN PTD */
void  PAJ7620_Init(void);
/* USER CODE END PTD */

/* Private define ------------------------------------------------------------*/
/* USER CODE BEGIN PD */
/*******************************fatfs*************************************/
//FATFS fs;                 // Work area (file system object) for logical drive
//FIL fil;                  // file objects
//uint32_t byteswritten;                /* File write counts */
//uint32_t bytesread;                   /* File read counts */
//uint8_t wtext[] = "This is STM32H750XB working with FatFs"; /* File write buffer */
//uint8_t rtext[100];                     /* File read buffers */
//char filename[] = "0:/mytest_2023_07_27.txt";

/**************************************************************************/

/* USER CODE END PD */

/* Private macro -------------------------------------------------------------*/
/* USER CODE BEGIN PM */
//uint16_t testsram[250000] __attribute__((at(0XC0000000)));//测试用数组
/* USER CODE END PM */

/* Private variables ---------------------------------------------------------*/

/* USER CODE BEGIN PV */

/* USER CODE END PV */

/* Private function prototypes -----------------------------------------------*/
void SystemClock_Config(void);
void PeriphCommonClock_Config(void);
/* USER CODE BEGIN PFP */
 int numerical_value;
extern int buzzer_flag;
uint8_t ID[2];


uint8_t dat[11] = "mculover666";
uint8_t read_buf[11] = {0};
uint8_t UserTxBuffer[] = "usb cdc test!\r\n";

 uint16_t  ccr2=200;
 uint16_t  ccr3=100;
/* USER CODE END PFP */

/* Private user code ---------------------------------------------------------*/
/* USER CODE BEGIN 0 */
float readInternalVoltage(void)
{
    uint16_t rawValue;
   
    // 启动ADC转换
    HAL_ADC_Start(&hadc3);


    // 等待转换完成
     HAL_ADC_PollForConversion(&hadc3, HAL_MAX_DELAY);
	 

    // 读取ADC转换结果
    rawValue = HAL_ADC_GetValue(&hadc3);

    // 将原始值转换为电压
    //voltage = ((float)rawValue / 65535) * 3.3;  // 假设参考电压为3.3V

    // 停止ADC转换
    HAL_ADC_Stop(&hadc3);
	

    return rawValue;
}
/* USER CODE END 0 */

/**
  * @brief  The application entry point.
  * @retval int
  */
int main(void)
{
  /* USER CODE BEGIN 1 */
  uint8_t t;
  uint8_t temperature;  	    
	uint8_t humidity; 
	uint32_t time = 0;
  /* USER CODE END 1 */

  /* Enable I-Cache---------------------------------------------------------*/
  SCB_EnableICache();

  /* Enable D-Cache---------------------------------------------------------*/
  SCB_EnableDCache();

  /* MCU Configuration--------------------------------------------------------*/

  /* Reset of all peripherals, Initializes the Flash interface and the Systick. */
  HAL_Init();

  /* USER CODE BEGIN Init */

  /* USER CODE END Init */

  /* Configure the system clock */
  SystemClock_Config();

/* Configure the peripherals common clocks */
  PeriphCommonClock_Config();

  /* USER CODE BEGIN SysInit */

  /* USER CODE END SysInit */

  /* Initialize all configured peripherals */
  MX_GPIO_Init();
  MX_UART4_Init();
  MX_ADC3_Init();
  MX_TIM6_Init();
  MX_I2C1_Init();
  MX_TIM1_Init();
  MX_SDMMC1_SD_Init();
  MX_FATFS_Init();
  MX_FMC_Init();
  MX_SPI1_Init();
  MX_QUADSPI_Init();
  MX_USB_DEVICE_Init();
  MX_SDMMC2_SD_Init();
  MX_TIM5_Init();
  MX_TIM2_Init();
  MX_TIM3_Init();
  /* USER CODE BEGIN 2 */
  //点亮
//	HAL_ADCEx_Calibration_Start(&hadc3,ADC_CALIB_OFFSET,ADC_SINGLE_ENDED);//校准ADC
//	HAL_ADCEx_Calibration_Start(&hadc1,ADC_CALIB_OFFSET,ADC_SINGLE_ENDED);//校准ADC
  
//	 LED_R_ON;
//	 LED_B_ON;

    //SDRAM_InitSequence();
		
	    HAL_TIM_Base_Start_IT(&htim6);
		  
		
	
		//HAL_TIM_IC_Start_IT(&htim5,TIM_CHANNEL_3);
		
//		HAL_TIM_PWM_Init(&htim3);
//		 HAL_TIM_Base_Start(&htim3);/*启动定时器*/	
		 HAL_TIM_PWM_Start(&htim3,TIM_CHANNEL_1);
	  
		
		
		 HAL_TIM_Base_Start_IT(&htim2);/*开启更新中断*/	
		 HAL_TIM_IC_Start_IT(&htim2,TIM_CHANNEL_1);
		 HAL_TIM_IC_Start_IT(&htim2,TIM_CHANNEL_2);
		
	   printf("STM32H750XB !!!\r\n");	
	    //PAJ7620_Init();
			
			//DHT11_Init();
			
			//u8g2_t u8g2;
      //u8g2Init(&u8g2);
			
//      get_sd_informatization();
//      HAL_Delay(200);
//      SD_EraseTest();
//      SD_Write_Read_Test();
//	     HAL_SD_InitCard(&hsd2);
//	     HAL_SD_Init(&hsd2);
	     //fatfs_test();
			//	HAL_GPIO_WritePin(GPIOC, LED_R_Pin|WIFI_REG_ON_Pin, GPIO_PIN_RESET);
//			HAL_Delay(10);
//		  HAL_GPIO_WritePin(GPIOC, LED_R_Pin|WIFI_REG_ON_Pin, GPIO_PIN_SET); 
			   //mount_sd();
			
      //fsmc_sdram_test();
			
			
//	   BSP_W25Qx_Init();
//		 /*读取设备制造商id   一般不同大小的设备id不同，可根据返回值判断是哪一个设备*/
//     BSP_W25Qx_Read_ID(ID);
//     printf(" W25Qxxx ID is : 0x%02X 0x%02X \r\n\r\n",ID[0],ID[1]);
//	   w25q128_test();


//printf("Test W25QXX...\r\n");
//uint16_t device_id = W25QXX_ReadID();
//printf("device_id = 0x%04X\r\n\r\n", device_id);

///* 为了验证，首先读取要写入地址处的数据 */
//printf("-------- read data before write -----------\r\n");
//W25QXX_Read(read_buf, 5, 11);
//printf("read date is %s\r\n", (char*)read_buf);

///* 擦除该扇区 */
//printf("-------- erase sector 0 -----------\r\n");
//W25QXX_Erase_Sector(0);

///* 写数据 */
//printf("-------- write data -----------\r\n");
//W25QXX_Page_Program(dat, 5, 11);

///* 再次读数据 */
//printf("-------- read data after write -----------\r\n");
//W25QXX_Read(read_buf, 5, 11);
//printf("read date is %s\r\n", (char*)read_buf);


    //get_ap6212_wifi_informatization();
//	 HAL_TIM_PWM_Start(&htim5,TIM_CHANNEL_2);//初始化通道2
//   HAL_TIM_PWM_Start(&htim5,TIM_CHANNEL_3);//初始化通道3
//	 TIM5->CCR2 = 200 ; 
//	 TIM5->CCR3 = 800 ; 
  /* USER CODE END 2 */

  /* Infinite loop */
  /* USER CODE BEGIN WHILE */
  while (1)
  {
    /* USER CODE END WHILE */

    /* USER CODE BEGIN 3 */
		
		
		// printf("PWM_Frequency = %d \r\n",PWM_RisingCount);
		
	
		
		
		//tim_pwm_pluse_change(&ccr2);
// static uint16_t ccr2=200;
// static uint16_t Pulse_flag=1;			

//		
//		if(Pulse_flag==1)
//		{
//		   ccr2++;
//			   if(ccr2>=800)
//				  Pulse_flag=2;
//			HAL_Delay(1);
//		}
//		else if(Pulse_flag==2)
//		{
//		   ccr2--;
//			if(ccr2<=200)
//			
//			Pulse_flag=1;
//			
//			HAL_Delay(1);
//		}
//			
//			 TIM5->CCR2 = ccr2; 
		
	
		
	
		
		
		
		
		
	   //key_process();
		/*PC3接电位器测试*/
//		HAL_Delay(500);
//    numerical_value=(int)readInternalVoltage();
//		printf("%d\r\n",numerical_value);
//  buzzer_flag=1;

//    BUZZER_ON;
//		HAL_Delay(100);
//    BUZZER_OFF;		
//		HAL_Delay(2000);
		
		
//		uint32_t code=scan_irda();
//		if(code!=0x00)
//		{
//		
//		 printf("%08x\r\n",code);
//			
//		}
  // HAL_Delay(200);

	//手势识别
//		gesture->detect = GS_Read_nByte(PAJ_GET_INT_FLAG1,2,&gesture->data[0]);//读取手势状态			
//		if(!gesture->detect)
//		{   
//			gesture->type =(uint16_t)gesture->data[1]<<8 | gesture->data[0];
//			if(gesture->type) 
//			{
//				switch(gesture->type)
//				{
//					case GES_UP:               printf("Up\r\n");            gesture->valid=1;      break; //向上
//					case GES_DOWN:             printf("Down\r\n");          gesture->valid=1;      break; //向下
//					case GES_LEFT:             printf("Left\r\n");          gesture->valid=1;      break; //向左
//					case GES_RIGHT:          	 printf("Right\r\n");         gesture->valid=1;      break; //向右
//					case GES_FORWARD:       	 printf("Forward\r\n");       gesture->valid=1;      break; //向前
//					case GES_BACKWARD:      	 printf("Backward\r\n");      gesture->valid=1;      break; //向后
//					case GES_CLOCKWISE:     	 printf("Clockwise\r\n");     gesture->valid=1;      break; //顺时针
//					case GES_ANTI_CLOCKWISE:   printf("AntiClockwise\r\n"); gesture->valid=1;      break; //逆时针
//					case GES_WAVE:             printf("Wave\r\n");          gesture->valid=1;      break; //挥动
//					default:  																				      gesture->valid=0;      break;	
//				}
//				
//			}
//		}
  
	
//    delay_us(5);
//   
//   HAL_GPIO_WritePin(PB2_GPIO_Port, PB2_Pin,1);
//	  delay_us(5);
//   
//   HAL_GPIO_WritePin(PB2_GPIO_Port, PB2_Pin,0);


//      u8g2_FirstPage(&u8g2);
//       do
//       {
//				 draw(&u8g2);

//				 u8g2DrawTest(&u8g2);
//       } while (u8g2_NextPage(&u8g2));


  



//	if(t%20==0)
//		{          
//			DHT11_Read_Data(&temperature,&humidity);	
//      printf("Tem:%d\r\n",temperature);
//			printf("Hum:%d\r\n",humidity);	
//			printf("\r\n\n");
//		
//		}				   
//	 	HAL_Delay(100);
//		
//		t++;  


//    USBD_CDC_SetTxBuffer(&hUsbDeviceFS, (uint8_t*)&UserTxBuffer, sizeof(UserTxBuffer));

//    USBD_CDC_TransmitPacket(&hUsbDeviceFS);

//    HAL_Delay(1000);
//    HAL_GPIO_WritePin(GPIOB, PB2_Pin, GPIO_PIN_RESET);
//    tim_delay_us(&htim6,1);/*1us*/
//	tim_delay_us(&htim6,10);/*1us*/
//		
//    HAL_GPIO_WritePin(GPIOB, PB2_Pin, GPIO_PIN_SET);
    



	}
  
  /* USER CODE END 3 */
}

/**
  * @brief System Clock Configuration
  * @retval None
  */
void SystemClock_Config(void)
{
  RCC_OscInitTypeDef RCC_OscInitStruct = {0};
  RCC_ClkInitTypeDef RCC_ClkInitStruct = {0};

  /** Supply configuration update enable
  */
  HAL_PWREx_ConfigSupply(PWR_LDO_SUPPLY);

  /** Configure the main internal regulator output voltage
  */
  __HAL_PWR_VOLTAGESCALING_CONFIG(PWR_REGULATOR_VOLTAGE_SCALE1);

  while(!__HAL_PWR_GET_FLAG(PWR_FLAG_VOSRDY)) {}

  __HAL_RCC_SYSCFG_CLK_ENABLE();
  __HAL_PWR_VOLTAGESCALING_CONFIG(PWR_REGULATOR_VOLTAGE_SCALE0);

  while(!__HAL_PWR_GET_FLAG(PWR_FLAG_VOSRDY)) {}

  /** Initializes the RCC Oscillators according to the specified parameters
  * in the RCC_OscInitTypeDef structure.
  */
  RCC_OscInitStruct.OscillatorType = RCC_OSCILLATORTYPE_HSE;
  RCC_OscInitStruct.HSEState = RCC_HSE_ON;
  RCC_OscInitStruct.PLL.PLLState = RCC_PLL_ON;
  RCC_OscInitStruct.PLL.PLLSource = RCC_PLLSOURCE_HSE;
  RCC_OscInitStruct.PLL.PLLM = 5;
  RCC_OscInitStruct.PLL.PLLN = 192;
  RCC_OscInitStruct.PLL.PLLP = 2;
  RCC_OscInitStruct.PLL.PLLQ = 2;
  RCC_OscInitStruct.PLL.PLLR = 2;
  RCC_OscInitStruct.PLL.PLLRGE = RCC_PLL1VCIRANGE_2;
  RCC_OscInitStruct.PLL.PLLVCOSEL = RCC_PLL1VCOWIDE;
  RCC_OscInitStruct.PLL.PLLFRACN = 0;
  if (HAL_RCC_OscConfig(&RCC_OscInitStruct) != HAL_OK)
  {
    Error_Handler();
  }

  /** Initializes the CPU, AHB and APB buses clocks
  */
  RCC_ClkInitStruct.ClockType = RCC_CLOCKTYPE_HCLK|RCC_CLOCKTYPE_SYSCLK
                              |RCC_CLOCKTYPE_PCLK1|RCC_CLOCKTYPE_PCLK2
                              |RCC_CLOCKTYPE_D3PCLK1|RCC_CLOCKTYPE_D1PCLK1;
  RCC_ClkInitStruct.SYSCLKSource = RCC_SYSCLKSOURCE_PLLCLK;
  RCC_ClkInitStruct.SYSCLKDivider = RCC_SYSCLK_DIV1;
  RCC_ClkInitStruct.AHBCLKDivider = RCC_HCLK_DIV2;
  RCC_ClkInitStruct.APB3CLKDivider = RCC_APB3_DIV2;
  RCC_ClkInitStruct.APB1CLKDivider = RCC_APB1_DIV2;
  RCC_ClkInitStruct.APB2CLKDivider = RCC_APB2_DIV2;
  RCC_ClkInitStruct.APB4CLKDivider = RCC_APB4_DIV2;

  if (HAL_RCC_ClockConfig(&RCC_ClkInitStruct, FLASH_LATENCY_4) != HAL_OK)
  {
    Error_Handler();
  }
}

/**
  * @brief Peripherals Common Clock Configuration
  * @retval None
  */
void PeriphCommonClock_Config(void)
{
  RCC_PeriphCLKInitTypeDef PeriphClkInitStruct = {0};

  /** Initializes the peripherals clock
  */
  PeriphClkInitStruct.PeriphClockSelection = RCC_PERIPHCLK_ADC|RCC_PERIPHCLK_SDMMC
                              |RCC_PERIPHCLK_SPI1;
  PeriphClkInitStruct.PLL2.PLL2M = 2;
  PeriphClkInitStruct.PLL2.PLL2N = 16;
  PeriphClkInitStruct.PLL2.PLL2P = 4;
  PeriphClkInitStruct.PLL2.PLL2Q = 1;
  PeriphClkInitStruct.PLL2.PLL2R = 1;
  PeriphClkInitStruct.PLL2.PLL2RGE = RCC_PLL2VCIRANGE_3;
  PeriphClkInitStruct.PLL2.PLL2VCOSEL = RCC_PLL2VCOWIDE;
  PeriphClkInitStruct.PLL2.PLL2FRACN = 0;
  PeriphClkInitStruct.SdmmcClockSelection = RCC_SDMMCCLKSOURCE_PLL2;
  PeriphClkInitStruct.Spi123ClockSelection = RCC_SPI123CLKSOURCE_PLL2;
  PeriphClkInitStruct.AdcClockSelection = RCC_ADCCLKSOURCE_PLL2;
  if (HAL_RCCEx_PeriphCLKConfig(&PeriphClkInitStruct) != HAL_OK)
  {
    Error_Handler();
  }
}

/* USER CODE BEGIN 4 */


void  PAJ7620_Init(void)
{
	while(!paj7620u2_init())//PAJ7620U2传感器初始化
	{
		printf("PAJ7620U2_B Error!!!\r\n");
		
		HAL_Delay(500);
	}
	
	HAL_Delay(1000);
	Gesture_Init();
  printf("PAJ7620U2 OK\r\n");
}


/* USER CODE END 4 */

/**
  * @brief  Period elapsed callback in non blocking mode
  * @note   This function is called  when TIM7 interrupt took place, inside
  * HAL_TIM_IRQHandler(). It makes a direct call to HAL_IncTick() to increment
  * a global variable "uwTick" used as application time base.
  * @param  htim : TIM handle
  * @retval None
  */
void HAL_TIM_PeriodElapsedCallback(TIM_HandleTypeDef *htim)
{
  /* USER CODE BEGIN Callback 0 */

  /* USER CODE END Callback 0 */
  if (htim->Instance == TIM7) {
    HAL_IncTick();
  }
  /* USER CODE BEGIN Callback 1 */
  if (htim->Instance == TIM2)
    {
      TIM2_OverflowCount++;
	}
			//printf("OverflowCount=%d\r\n",OverflowCount);
			
    
  /* USER CODE END Callback 1 */
}

/**
  * @brief  This function is executed in case of error occurrence.
  * @retval None
  */
void Error_Handler(void)
{
  /* USER CODE BEGIN Error_Handler_Debug */
  /* User can add his own implementation to report the HAL error return state */
  __disable_irq();
  while (1)
  {
  }
  /* USER CODE END Error_Handler_Debug */
}

#ifdef  USE_FULL_ASSERT
/**
  * @brief  Reports the name of the source file and the source line number
  *         where the assert_param error has occurred.
  * @param  file: pointer to the source file name
  * @param  line: assert_param error line source number
  * @retval None
  */
void assert_failed(uint8_t *file, uint32_t line)
{
  /* USER CODE BEGIN 6 */
  /* User can add his own implementation to report the file name and line number,
     ex: printf("Wrong parameters value: file %s on line %d\r\n", file, line) */
  /* USER CODE END 6 */
}
#endif /* USE_FULL_ASSERT */
