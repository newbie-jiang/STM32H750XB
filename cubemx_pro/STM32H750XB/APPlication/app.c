#include "app.h" 
#include "bsp.h" 
#include "lcd.h" 
#include "touch.h"
#include "tim.h"
#include "stdio.h"
#include "sdram.h"
#include "stm32h7xx.h"
#include "paj7620u2.h"
#include "oled_test.h"
#include "stm32_u8g2.h"
#include "dht11.h"
#include "delay.h" 
#include "adc.h" 
#include "ltdc_.h" 
#include "stm32h7xx_it.h" 
#include "sd.h" 
#include "w25qxx.h"
#include "w25q64jv.h"
#include "display.h"



 uint32_t rawValue;
//  n  
/* 10个触控点的颜色(电容触摸屏用) */
const uint16_t POINT_COLOR_TBL[10] = {RED, GREEN, BLUE, BROWN, YELLOW, MAGENTA, CYAN, LIGHTBLUE, BRRED, GRAY};

void sys_qspi_enable_memmapmode(uint8_t ftype)
{
    uint32_t tempreg = 0; 
//    GPIO_InitTypeDef qspi_gpio;

//    __HAL_RCC_GPIOB_CLK_ENABLE();                            /* 使能PORTB时钟 */
//    __HAL_RCC_GPIOD_CLK_ENABLE();                            /* 使能PORTD时钟 */
//    __HAL_RCC_GPIOE_CLK_ENABLE();                            /* 使能PORTE时钟 */
//    __HAL_RCC_QSPI_CLK_ENABLE();                             /* QSPI时钟使能 */

//    qspi_gpio.Pin = GPIO_PIN_6;                              /* PB6 AF10 */
//    qspi_gpio.Mode = GPIO_MODE_AF_PP;
//    qspi_gpio.Speed = GPIO_SPEED_FREQ_VERY_HIGH;
//    qspi_gpio.Pull = GPIO_PULLUP;
//    qspi_gpio.Alternate = GPIO_AF10_QUADSPI;
//    HAL_GPIO_Init(GPIOB, &qspi_gpio);

//    qspi_gpio.Pin = GPIO_PIN_2;                              /* PB2 AF9 */
//    qspi_gpio.Alternate = GPIO_AF9_QUADSPI;
//    HAL_GPIO_Init(GPIOB, &qspi_gpio);

//    qspi_gpio.Pin = GPIO_PIN_11 | GPIO_PIN_12 | GPIO_PIN_13; /* PD11,12,13 AF9 */
//    qspi_gpio.Alternate = GPIO_AF9_QUADSPI;
//    HAL_GPIO_Init(GPIOD, &qspi_gpio);

//    qspi_gpio.Pin = GPIO_PIN_2;                              /* PE2 AF9 */
//    qspi_gpio.Alternate = GPIO_AF9_QUADSPI;
//    HAL_GPIO_Init(GPIOE, &qspi_gpio);

    /* QSPI设置，参考QSPI实验的QSPI_Init函数 */
    RCC->AHB3RSTR |= 1 << 14;       /* 复位QSPI */
    RCC->AHB3RSTR &= ~(1 << 14);    /* 停止复位QSPI */

    while (QUADSPI->SR & (1 << 5)); /* 等待BUSY位清零 */

    /* QSPI时钟源已经在sys_stm32_clock_init()函数中设置 */
    QUADSPI->CR = 0X01000310;       /* 设置CR寄存器, 这些值怎么来的，请参考QSPI实验/看H750参考手册寄存器描述分析 */
    QUADSPI->DCR = 0X00180201;      /* 设置DCR寄存器(FLASH容量32M(最大容量设置为32M, 默认用16M的), tSHSL = 3个时钟) */
    QUADSPI->CR |= 1 << 0;          /* 使能QSPI */
    //QUADSPI->CR |= 0 << 0;          /* 失能QSPI */
    /*
     *  注意:QSPI QE位的使能，在QSPI烧写算法里面，就已经设置了
     *  所以, 这里可以不用设置QE位，否则需要加入对QE位置1的代码
     *  不过, 代码必须通过仿真器下载, 直接烧录到外部QSPI FLASH, 是不可用的
     *  如果想直接烧录到外部QSPI FLASH也可以用, 则需要在这里添加QE位置1的代码
     *
     *  另外, 对与W25Q256,还需要使能4字节地址模式,或者设置S3的ADP位为1.
     *  我们在QSPI烧写算法里面已经设置了ADP=1(上电即32位地址模式),因此这里也
     *  不需要发送进入4字节地址模式指令/设置ADP=1了, 否则还需要设置ADP=1
     */

    /* BY/W25QXX 写使能（0X06指令）*/
    while (QUADSPI->SR & (1 << 5)); /* 等待BUSY位清零 */

    QUADSPI->CCR = 0X00000106;      /* 发送0X06指令，BY/W25QXX写使能 */

    while ((QUADSPI->SR & (1 << 1)) == 0);/* 等待指令发送完成 */

    QUADSPI->FCR |= 1 << 1;         /* 清除发送完成标志位 */

    /* MemroyMap 模式设置 */
    while (QUADSPI->SR & (1 << 5)); /* 等待BUSY位清零 */

    QUADSPI->ABR = 0;               /* 交替字节设置为0，实际上就是25QXX 0XEB指令的, M0~M7 = 0 */
    tempreg = 0XEB;                 /* INSTRUCTION[7:0] = 0XEB, 发送0XEB指令（Fast Read QUAD I/O） */
    tempreg |= 1 << 8;              /* IMODE[1:0] = 1, 单线传输指令 */
    tempreg |= 3 << 10;             /* ADDRESS[1:0] = 3, 四线传输地址 */
    tempreg |= (2 + ftype) << 12;   /* ADSIZE[1:0] = 2/3, 24位(ftype = 0) / 32位(ftype = 1)地址长度 */
    tempreg |= 3 << 14;             /* ABMODE[1:0] = 3, 四线传输交替字节 */
    tempreg |= 0 << 16;             /* ABSIZE[1:0] = 0, 8位交替字节(M0~M7) */
    tempreg |= 4 << 18;             /* DCYC[4:0] = 4, 4个dummy周期 */
    tempreg |= 3 << 24;             /* DMODE[1:0] = 3, 四线传输数据 */
    tempreg |= 3 << 26;             /* FMODE[1:0] = 3, 内存映射模式 */
    QUADSPI->CCR = tempreg;         /* 设置CCR寄存器 */

    /* 设置QSPI FLASH空间的MPU保护 */
    SCB->SHCSR &= ~(1 << 16);       /* 禁止MemManage */
    MPU->CTRL &= ~(1 << 0);         /* 禁止MPU */
    MPU->RNR = 0;                   /* 设置保护区域编号为0(1~7可以给其他内存用) */
    MPU->RBAR = 0X90000000;         /* 基地址为0X9000 000, 即QSPI的起始地址 */
    MPU->RASR = 0X0303002D;         /* 设置相关保护参数(禁止共用, 允许cache, 允许缓冲), 详见MPU实验的解析 */
    MPU->CTRL = (1 << 2) | (1 << 0);/* 使能PRIVDEFENA, 使能MPU */
    SCB->SHCSR |= 1 << 16;          /* 使能MemManage */
}

/**
 * @brief       清空屏幕并在右上角显示"RST"
 * @param       无
 * @retval      无
 */
void load_draw_dialog(void)
{
    lcd_clear(WHITE);                                                /* 清屏 */
    lcd_show_string(lcddev.width - 24, 0, 200, 16, 16, "RST", BLUE); /* 显示清屏区域 */
}


/**
 * @brief       画粗线
 * @param       x1,y1: 起点坐标
 * @param       x2,y2: 终点坐标
 * @param       size : 线条粗细程度
 * @param       color: 线的颜色
 * @retval      无
 */
void lcd_draw_bline(uint16_t x1, uint16_t y1, uint16_t x2, uint16_t y2, uint8_t size, uint16_t color)
{
    uint16_t t;
    int xerr = 0, yerr = 0, delta_x, delta_y, distance;
    int incx, incy, row, col;

    if (x1 < size || x2 < size || y1 < size || y2 < size)
        return;

    delta_x = x2 - x1; /* 计算坐标增量 */
    delta_y = y2 - y1;
    row = x1;
    col = y1;

    if (delta_x > 0)
    {
        incx = 1; /* 设置单步方向 */
    }
    else if (delta_x == 0)
    {
        incx = 0; /* 垂直线 */
    }
    else
    {
        incx = -1;
        delta_x = -delta_x;
    }

    if (delta_y > 0)
    {
        incy = 1;
    }
    else if (delta_y == 0)
    {
        incy = 0; /* 水平线 */
    }
    else
    {
        incy = -1;
        delta_y = -delta_y;
    }

    if (delta_x > delta_y)
        distance = delta_x; /* 选取基本增量坐标轴 */
    else
        distance = delta_y;

    for (t = 0; t <= distance + 1; t++) /* 画线输出 */
    {
        lcd_fill_circle(row, col, size, color); /* 画点 */
        xerr += delta_x;
        yerr += delta_y;

        if (xerr > distance)
        {
            xerr -= distance;
            row += incx;
        }

        if (yerr > distance)
        {
            yerr -= distance;
            col += incy;
        }
    }
}




void ctp_test(void)
{
//    uint8_t t = 0;
//  
//    uint16_t lastpos[10][2];        /* 最后一次的数据 */
//    uint8_t maxp = 5;

//    while (1)
//    {
//        tp_dev.scan(0);

//        for (t = 0; t < maxp; t++)
//        {
//            if ((tp_dev.sta) & (1 << t))
//            {
//                if (tp_dev.x[t] < lcddev.width && tp_dev.y[t] < lcddev.height)  /* 坐标在屏幕范围内 */
//                {
//                    if (lastpos[t][0] == 0XFFFF)
//                    {
//                        lastpos[t][0] = tp_dev.x[t];
//                        lastpos[t][1] = tp_dev.y[t];
//                    }

//                    lcd_draw_bline(lastpos[t][0], lastpos[t][1], tp_dev.x[t], tp_dev.y[t], 2, POINT_COLOR_TBL[t]); /* 画线 */
//                    lastpos[t][0] = tp_dev.x[t];
//                    lastpos[t][1] = tp_dev.y[t];

//                    if (tp_dev.x[t] > (lcddev.width - 24) && tp_dev.y[t] < 20)
//                    {
//                        load_draw_dialog();/* 清除 */
//                    }
//                }
//            }
//            else 
//            {
//                lastpos[t][0] = 0XFFFF;
//            }
//        }

//        delay_ms(5);
//   
//    }



  static  uint8_t t;
  static  uint8_t maxp = 5;
  static  uint16_t lastpos[10][2];        /* 最后一次的数据 */
    tp_dev.scan(0); // 假设这个扫描操作是非阻塞的

    for (t = 0; t < maxp; t++)
    {
        if ((tp_dev.sta) & (1 << t))
        {
            if (tp_dev.x[t] < lcddev.width && tp_dev.y[t] < lcddev.height)
            {
                if (lastpos[t][0] == 0XFFFF)
                {
                    lastpos[t][0] = tp_dev.x[t];
                    lastpos[t][1] = tp_dev.y[t];
                }

                lcd_draw_bline(lastpos[t][0], lastpos[t][1], tp_dev.x[t], tp_dev.y[t], 2, POINT_COLOR_TBL[t]);
                lastpos[t][0] = tp_dev.x[t];
                lastpos[t][1] = tp_dev.y[t];

                if (tp_dev.x[t] > (lcddev.width - 24) && tp_dev.y[t] < 20)
                {
                    load_draw_dialog();
                }
            }
        }
        else 
        {
            lastpos[t][0] = 0XFFFF;
        }
    }
}


static void  PAJ7620_Init(void)
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


static void paj7620_scan(void)
{
		gesture->detect = GS_Read_nByte(PAJ_GET_INT_FLAG1,2,&gesture->data[0]);//读取手势状态			
		if(!gesture->detect)
		{   
			gesture->type =(uint16_t)gesture->data[1]<<8 | gesture->data[0];
			if(gesture->type) 
			{
				switch(gesture->type)
				{
					case GES_UP:               printf("%d:Up\r\n",gesture->type);            gesture->valid=1;      break; //向上
					case GES_DOWN:             printf("%d:Down\r\n",gesture->type);          gesture->valid=1;      break; //向下
					case GES_LEFT:             printf("%d:Left\r\n",gesture->type);          gesture->valid=1;      break; //向左
					case GES_RIGHT:          	 printf("%d:Right\r\n",gesture->type);         gesture->valid=1;      break; //向右
					case GES_FORWARD:       	 printf("%d:Forward\r\n",gesture->type);       gesture->valid=1;      break; //向前
					case GES_BACKWARD:      	 printf("%d:Backward\r\n",gesture->type);      gesture->valid=1;      break; //向后
					case GES_CLOCKWISE:     	 printf("%d:Clockwise\r\n",gesture->type);     gesture->valid=1;      break; //顺时针
					case GES_ANTI_CLOCKWISE:   printf("%d:AntiClockwise\r\n",gesture->type); gesture->valid=1;      break; //逆时针
					case GES_WAVE:             printf("%d:Wave\r\n",gesture->type);          gesture->valid=1;      break; //挥动
					default:  																				      gesture->valid=0;      break;	
				}
				
//              printf("%d:\r\n",gesture->type);
			}
		}
}

void __lcd_init(void)
{
    
		 lcd_init();
	   g_point_color=RED; /*画笔颜色*/ 
	   lcd_display_dir(1);		//屏幕显示方向：0竖屏，1横屏
	   lcd_clear(WHITE);
		
//    lcd_clear(BLACK);
//		lcd_clear(BLUE);
//		lcd_clear(RED);
//		lcd_clear(MAGENTA);
//		lcd_clear(GREEN);
//		lcd_clear(CYAN);
//		lcd_clear(YELLOW);
//		lcd_clear(BRRED);
//		lcd_clear(GRAY);
//		lcd_clear(LGRAY);
//		lcd_clear(BROWN);
	    lcd_fill(20 , 80,120 ,400,0xFFE0);
			lcd_fill(140, 80,240 ,400,0x8888);
			lcd_fill(260, 80,360 ,400,0x2222);
			lcd_fill(380, 80,480 ,400,0xbbbb); 
			lcd_fill(20,400,480,475,0x1111);
			
		 
			lcd_show_string(100,10,800,80,32,"Newbie:STM32H750XB Display LCD RGB888",RED); 
		

	    lcd_draw_rectangle(500,120,780,400,RED);
	   for(int i=137;i>=1;i-=3)
			{
	      lcd_draw_circle(640,260,i,RED);
			}
			
			

			
}


void delay_us_init()
{
  HAL_TIM_Base_Start_IT(&htim6);
}

void irda_tick_init()
{
  HAL_TIM_Base_Start_IT(&htim6);
}




void readInternalVoltage(void)
{
  static float Current_Temp;
  static	float voltage;
	uint16_t TS_CAL1;
	uint16_t TS_CAL2;
	uint16_t VREFINT_CAL;
	
  uint16_t value[3];
  TS_CAL1 = *(__IO uint16_t *)(0x1FF1E820); 
	TS_CAL2 = *(__IO uint16_t *)(0x1FF1E840); 
	VREFINT_CAL = *(__IO uint16_t *)(0x1FF1E860UL); 
 
   for(int i=0;i<=2;i++)  
	{
		  HAL_ADC_Start_DMA(&hadc3,(uint32_t*)value,3);
      value[i] = HAL_ADC_GetValue(&hadc3);
	    printf("ADC value[%d]=%d\r\n",i,value[i]);
		
		if(i==0)/*电位器电压*/
		{
		  printf("ADC external voltage=%f\r\n",value[i]*3.3/65535);
			printf("\r\n");
		}
		
		if(i==1)/*内部参考电压*/
		{
			printf("ADC inner voltage=%f\r\n",3.3*VREFINT_CAL/value[i]);
			printf("\r\n");
		}
		
		if(i==2)/*内部温度*/
		{
		
			Current_Temp=(110.0 - 30.0)*(value[2] - TS_CAL1)/ (TS_CAL2 - TS_CAL1) + 30;
			printf("ADC temp:%f\r\n",Current_Temp);
			printf("\r\n");
		}
	  
	
	}



}









void qspi_test(void)
{
	static uint8_t test_buf[W25Q64JV_SECTOR_SIZE];
	static uint8_t read_buf[W25Q64JV_SECTOR_SIZE];

  for (int i = 0; i < W25Q64JV_SECTOR_SIZE; i++) {
    test_buf[i] = i;
		printf("test_buf[%d]=%d\r\n",i,test_buf[i]); /*数值类型为无符号八位 255之后又从0开始写*/
	}
  QSPI_W25Q64JV_Write(test_buf, 0x0, W25Q64JV_SECTOR_SIZE);
	
	
	
  QSPI_W25Q64JV_Read(read_buf, 0x0, W25Q64JV_SECTOR_SIZE);
	for(int i=0;i<W25Q64JV_SECTOR_SIZE;i++)
	 {
	  printf("read_buf[%d]=%d\r\n",i,test_buf[i]);
	
	 }
}

void adc_Calibration(void)
{
 HAL_ADCEx_Calibration_Start(&hadc3,ADC_CALIB_OFFSET,ADC_SINGLE_ENDED);//校准ADC
}


void HAL_ADC_ConvCpltCallback(ADC_HandleTypeDef* hadc)
{
	
	
  printf("05%d",rawValue);


}






void bsp_init(void)
{

	
	uint8_t ID[2];
//	u8g2_t u8g2;
//  u8g2Init(&u8g2);
 //  sys_qspi_enable_memmapmode(0);  
	 delay_us_init();
	 irda_tick_init();

	 SDRAM_InitSequence(); /*初始化SDRAM*/
	// HAL_GPIO_WritePin(GPIO_LCD_BL_GPIO_Port, GPIO_LCD_BL_Pin, GPIO_PIN_RESET);/*关闭屏幕背光*/
	 __lcd_init();
   PAJ7620_Init();
 	 DHT11_Init();
 	 HAL_Delay(1000);
	
	 adc_Calibration();
   HAL_ADC_Start(&hadc3);
   //BSP_W25Qx_Init();
	 lcd_clear(WHITE);
//   mount_sd();/*挂载SD卡*/
	 QSPI_W25Q64JV_Reset();
   uint8_t id[2];
   if (QSPI_OK != QSPI_W25Q64JV_DeviceID(id)) {
    while (1);
  }
	 printf("QSPI_W25Q64JV id=%x%X\r\n",id[0],id[1]);
    HAL_Delay(1000);

  for(int i=0;i<=800;i+=2)
	{
		for(int j=0;j<=480;j+=2)  
     lcd_draw_point(i,j,RED);
	}


	
 draw_grid(800, 480, 80, 80, 4,RED);

	//testDrawXBM(&u8g2);
  //u8g2DrawTest(&u8g2);
	
	

	
	
	  tp_dev.init();
	  printf("run here \r\n");   
	

		HAL_Delay(100);
	

}





void application(void)
{
	
 static uint8_t temperature=10;  	    
 static uint8_t humidity=10; 
 static uint8_t write_qspi_displaynum; 
 static uint8_t read_qspi_displaynum; 
	
	static uint32_t  tick=0; 
	tick=HAL_GetTick(); 
//	if(tick%200==0)
//	{
//	 paj7620_scan();
//	}
//	
//	if(tick%1000==0)
//	{
//	    DHT11_Read_Data(&temperature,&humidity);	
//		
//      printf("Tem:%d\r\n",temperature);
//		  printf("Hum:%d\r\n",humidity);	
//		  printf("\r\n");
//		
//	}
	
//	if(tick%2000==0)
//	{
//	   
//		write_qspi_displaynum++;

//		lcd_show_num(600,200,write_qspi_displaynum,3,32,RED);
//	}
	
	
	
	
// if(irda_code_flag==1)
// {
//    		QSPI_W25Q64JV_Write(&write_qspi_displaynum, 0x0, 1);
//	      lcd_show_string(300,300,100,100,32,"qspi write",RED); 
//	      lcd_show_num(300,400,read_qspi_displaynum,3,32,RED);
//        printf("write qspi flash ok");
//	 
//     
//	           irda_code_flag=0;
// }
//  else if(irda_code_flag==2)
//	{	
//  	    QSPI_W25Q64JV_Read(&read_qspi_displaynum, 0x0, 1);
//     	  irda_code_flag=0;   
//		    lcd_show_string(300,300,100,100,32,"save value for qspi ok",RED); 
//		    lcd_show_num(300,500,read_qspi_displaynum,3,32,RED);
//		   
//	}
//	
	
	
	if(tick%800==0)
	{
	   readInternalVoltage();
	}

	
}

