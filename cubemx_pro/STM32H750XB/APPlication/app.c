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
#include "fatfs.h"
#include "ff.h"
#include "config.h"

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








/************************雷达数据解析*******************************/

void HAL_UART_TxCpltCallback(UART_HandleTypeDef *huart) {
    if (huart->Instance == UART4) {  
        // 当数据发送完毕后，此部分代码会执行
        // 您可以在此处添加代码，例如灭掉一个LED或设置一个标志
			
    }
		
		if (huart->Instance == USART1) {  
        // 当数据发送完毕后，此部分代码会执行
        // 您可以在此处添加代码，例如灭掉一个LED或设置一个标志
			
    }
}


/************************雷达发送数据*******************************/
static void initUartPacket(UartPacket* packet) {
    packet->header[0] = 0x55; /*数据头*/
    packet->header[1] = 0x5A;
    packet->tail = 0xFE;/*数据尾*/
}

static void setPacketContent(UartPacket* packet, uint8_t command, uint16_t data) {
    packet->command = command; /*命令*/
	  packet->data_high = (data >> 8) & 0xFF; /*发送数据高位*/
    packet->data_low = data & 0xFF;/*发送数据低位*/
    packet->checksum = command ^ packet->data_high ^ packet->data_low; // 计算校验和
}

static void sendUartPacket(UART_HandleTypeDef* huart, UartPacket* packet) {
    HAL_UART_Transmit(huart, (uint8_t*)packet, sizeof(UartPacket), HAL_MAX_DELAY);
}



/*支持命令如下
0x09   打开关闭雷达       0x0001打开  0x0000关闭
0x89   读取雷达开关状态
0x01   写入距离等级       0x0000  ---  0x000F(15) 0距离最大
0x81   读取距离等级 
0x02   写入延迟时间       0x0000  ---  0xffff
0x82   读取延迟时间       
0x03   打开关闭光感       0x0001打开   0x0000关闭  
0x83   读取光感开关
0x06   设置封锁时间       0x0000  ---  0xffff    

封锁时间，也称保护时间，当雷达 OUT 引脚由高变低之后，即感应输出
结束后，接下来会有一段时间停止检测，这段停止检测的时间被称为封锁时间，
默认 1S(数值 0x03e8=1000ms),一般不作修改，如特殊需要修改，应设置不小于
500ms
*/
UartPacket packet;
void usart_radar_process(void)
{
  static uint8_t commandValue = 0x09;
  static uint16_t dataValue = 0x0000;
	
  initUartPacket(&packet);
  setPacketContent(&packet, commandValue, dataValue);
  sendUartPacket(&huart1, &packet);
}






/*********************debug串口数据解析***********************************/
uint8_t rx_data;
uint8_t rx_buffer[128];
uint32_t rx_index = 0;

//UartPacket txPacket;  // 用于发送的数据包
UartPacket rxPacket;  // 用于接收的数据包


//{"LED":"ON"} or {"LED":"OFF"}
void HAL_UART_RxCpltCallback(UART_HandleTypeDef *huart)
{
    if (huart->Instance == UART4) 
    {
        // Add received byte to the buffer
        rx_buffer[rx_index++] = rx_data;

        // Check if received a complete JSON message e.g., {"LED":"ON"} or {"LED":"OFF"}
        if (rx_data == '}' && rx_index < sizeof(rx_buffer)) 
        {
            rx_buffer[rx_index] = '\0';  // Null terminate the string
            if (strstr((char *)rx_buffer, "{\"LED\":\"ON\"}")) 
            {
               HAL_GPIO_WritePin(GPIOC, LED_R_Pin, GPIO_PIN_RESET);
            } 
            else if (strstr((char *)rx_buffer, "{\"LED\":\"OFF\"}")) 
            {
               HAL_GPIO_WritePin(GPIOC, LED_R_Pin, GPIO_PIN_SET);
            }
						HAL_UART_Transmit_IT(&huart4,(uint8_t*)rx_buffer,sizeof(rx_buffer));
						
            rx_index = 0; // Reset the buffer index
        }
				
				

        // Restart interrupt for next byte
        HAL_UART_Receive_IT(&huart4, &rx_data, 1);
     }
		
//					if (huart->Instance == USART1) 
//				{
//		      // 如果接收缓冲区内容与发送的数据包相匹配
//            if(memcmp(&packet, &rxPacket, sizeof(UartPacket)) == 0) 
//            {
//							/*数据匹配，写命令成功*/
//                printf("radar write command ok !!!\r\n");
//            }
//            else 
//            {
//                // 数据不匹配，判断是否为查询操作
//							   printf("radar write command ERR !!!\r\n");
//            }
//						
//						HAL_UART_Receive_IT(&huart1, (uint8_t*)&rxPacket, sizeof(UartPacket));
//		    }
		
}
/*******************************************************************************************************************/
/*******************************************************************************************************************/
/**************************************************SD卡操作函数*****************************************************/
/*******************************************************************************************************************/
/*******************************************************************************************************************/
FIL file;      // 文件对象
FRESULT fres;  // FatFs函数的返回类型
FILINFO fno;
FATFS fs;       // 文件系统对象



/*************************挂载SD***************************************/
void mount_sd(void)
{
	retSD = f_mount(&fs, SDPath, 1);
	if(retSD)
	{
		   #ifdef  FATFS_DEBUG
			   printf(" mount error : %d \r\n",retSD);
			 #endif
			
			Error_Handler();
	}
	else
			
	     #ifdef  FATFS_DEBUG
			   printf(" mount sucess!!! \r\n");
			 #endif
	
}
/*************************格式化SD卡***************************************/
/*在调用f_mkfs之前，确保没有对目标驱动器进行挂载
格式化之后，通常会卸载驱动器以确保所有的系统资源得到正确的释放*/

//驱动器的最大扇区大小
#define  FF_MAX_SS  512

FRESULT mkfs_sdcard(void) {

    BYTE work[FF_MAX_SS];  // Working buffer for f_mkfs

    fres = f_mount(&fs, "", 0);  // 挂载设备
    if (fres != FR_OK) {
			#ifdef  FATFS_DEBUG
			printf("ERR SD Mounting failed:%d\r\n",fres);
			#endif
			
        return fres;  // 挂载错误
    }

    fres = f_mkfs("", FM_FAT32, 0, work, sizeof(work));  // Format as FAT32
    if (fres != FR_OK) {
			#ifdef  FATFS_DEBUG
			printf("ERR SD Formatting failed:%d\r\n",fres);
			#endif
        return fres;  //格式化失败
    }

    f_mount(0, "", 0);  // 格式化后默认卸载驱动
		#ifdef  FATFS_DEBUG
		printf("SD Formatting ok\r\n");
		#endif
    return FR_OK;
}



/**********************检查目录*****************************/
static uint8_t Check_whether_the_directory_exists(const char* path)
{
    
    fres = f_stat(path, &fno);
    if (fres == FR_OK && (fno.fattrib & AM_DIR)) {
        return 1;  // 目录存在
    }
    return 0;  // 目录不存在
}

/************************创建目录***********************************/
static uint8_t createDirectory(const char* path) {
    if (!Check_whether_the_directory_exists(path)) {
        fres = f_mkdir(path);
        if(fres != FR_OK) {
            //处理错误，显示错误消息
					#ifdef  FATFS_DEBUG
					printf("ERR create Directory :%d\r\n",fres);
					#endif
					    return 1;/*返回错误信息*/
        } else {
            // 目录成功创建
					#ifdef  FATFS_DEBUG
					    printf("create Directory ok\r\n");
					#endif
						  return 0;
        }
    } else {
        // 目录已经存在
			#ifdef  FATFS_DEBUG
		     printf("Directory already exists\r\n");
			#endif
		     return 0;
    }
}

/*****************************创建目录********************************/
uint8_t use_createDirectory(void)
{
  createDirectory("/system");
}





/************************检查文件是否存在******************************/
static uint8_t createFileIfNotExist(const char* folderPath, const char* filename) {
    char fullPath[256];  // 假设文件路径长度不超过256。可以根据实际需要调整大小

    // 判断是否为根目录
    if (folderPath && *folderPath) {
        sprintf(fullPath, "%s/%s", folderPath, filename);
    } else {
        sprintf(fullPath, "%s", filename); // 直接使用文件名，因为它在根目录
    }

    // 检查文件是否存在
    fres = f_stat(fullPath, &fno);
    if (fres == FR_OK) {
        // 文件存在
        return 1;
    } else if (fres == FR_NO_FILE) {
        // 文件不存在，尝试创建
        fres = f_open(&file, fullPath, FA_WRITE | FA_CREATE_NEW);
        if (fres == FR_OK) {
            f_close(&file);
            return 0;  // 文件已成功创建
        } else {
            return -1;  // 创建文件时发生错误
        }
    } else {
        // 其他错误
        return -1;
    }
}

/*************************创建文件*************************/
static uint8_t create_a_file(const char* folderPath, const char* filename) {
    uint8_t result = createFileIfNotExist(folderPath, filename);

    if (result == 1) {
        // 文件已存在
        #ifdef FATFS_DEBUG
            printf("File already exists: %d\r\n", fres);
        #endif
    } else if (result == 0) {
        // 文件已成功创建
        #ifdef FATFS_DEBUG
            printf("File created successfully: %d\r\n", fres);
        #endif
    } else {
        // 发生错误
        #ifdef FATFS_DEBUG
            printf("Error creating file: %d\r\n", fres);
        #endif
    }

    return 0;
}

/**************************创建文件************************************/
uint8_t use_create_a_file(void)
{
  // 在根目录创建
  // create_a_file("", "myfile.txt");
  // 或在子文件夹创建
  // create_a_file("system", "myfile.txt");
    create_a_file("system", "file1.txt");
	  create_a_file("system", "file2.txt");
}

/*************************写文件函数*************************************/

uint8_t writeToFile(const char *filePath, const void *buffer, uint32_t length) {
  
    UINT bytesWritten;

    // 打开文件
    fres = f_open(&file, filePath, FA_WRITE | FA_OPEN_ALWAYS);
    if (fres != FR_OK) {
        // 文件打开失败
        return -1;
    }

    // 将写指针移到文件末尾，以便在文件末尾追加数据
    f_lseek(&file, f_size(&file));

    // 写入数据
    fres = f_write(&file, buffer, length, &bytesWritten);

    // 关闭文件
    f_close(&file);

    // 检查是否所有数据都已写入
    if (fres == FR_OK && bytesWritten == length) {
        #ifdef FATFS_DEBUG
            printf("File write length ok: %d\r\n", fres);
        #endif
			 return 0; // 成功
    } else {
			 #ifdef FATFS_DEBUG
            printf("File write length err: %d\r\n", fres);
        #endif
			return -1; // 写入错误
    }
}


/**************************写入文件************************************/
uint8_t use_writeToFile(void) {
   
	 uint8_t buffer[256];
	 for(int i=0;i<256;i++)
	{
	  buffer[i]=i;
	}

   writeToFile("system/file1.txt", buffer, sizeof(buffer));

    return 0;  
}



/**
 * 读取文件的内容。
 * @param filePath 要读取的文件的路径。
 * @param buffer 存放读取数据的缓冲区。
 * @param bufferSize 缓冲区的大小。
 * @param offset 多少字符开始读取。
 * @return 实际读取的字节数，或-1表示错误。
 */
uint32_t readFileContent(const char* filePath, void* buffer, int bufferSize, DWORD offset) {

    UINT bytesRead;

    fres = f_open(&file, filePath, FA_READ);
    if (fres != FR_OK) {
        // 文件打开失败
        return -1;
    }

    // 设置读取的开始位置
    if (f_lseek(&file, offset) != FR_OK) {
        // 错误处理
        f_close(&file);
        return -1;
    }

    fres = f_read(&file, buffer, bufferSize - 1, &bytesRead);
    if (fres != FR_OK) {
        // 读取文件失败
        f_close(&file);
        return -1;
    }

//    buffer[bytesRead] = '\0';  // 添加字符串终止符

    f_close(&file);  // 记得关闭文件
		#ifdef FATFS_DEBUG
	  printf("bytesRead num=%d\r\n",bytesRead);/*实际读取字节数*/
    #endif
    return bytesRead;  // 返回实际读取的字节数
}


/*使用读文件函数*/
uint8_t use_readFileContent(void)
{
  uint8_t buffer[512];
	memset(buffer, 0, sizeof(buffer));/*清空缓冲区*/
	
  uint32_t bytesRead = readFileContent("system/file1.txt", buffer, sizeof(buffer), 0); // 从第0个字节开始读取
	HAL_Delay(10);
	
	
	for(int i=0;i<512;i++)
	printf("read sd buffer[%d]:%x\r\n",i,buffer[i]);

}


/*sd卡应用*/
void sd_application(void)
{
//	 mkfs_sdcard();//格式化sd卡
	 mount_sd();/*挂载sd卡*/
   use_createDirectory();/*创建文件夹*/
   use_create_a_file();/*创建文件*/ 
   use_writeToFile();/*写文件*/
   use_readFileContent();/*读文件*/
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
//	 printf("QSPI_W25Q64JV id=%x%X\r\n",id[0],id[1]);
    HAL_Delay(1000);

  for(int i=0;i<=800;i+=2)
	{
		for(int j=0;j<=480;j+=2)  
     lcd_draw_point(i,j,RED);
	}


	
   draw_grid(800, 480, 80, 80, 4,RED);

	//testDrawXBM(&u8g2);
  //u8g2DrawTest(&u8g2);
	
	HAL_UART_Receive_IT(&huart4, &rx_data, 1);
//  HAL_UART_Receive_IT(&huart1, (uint8_t*)&rxPacket, sizeof(UartPacket));
//   usart_radar_process();

	
	
	  tp_dev.init();
//	  printf("run here \r\n");   
	
   
		HAL_Delay(100);
    sd_application();
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
	
	
//	if(tick%800==0)
//	{
//	   readInternalVoltage();
//	}

	
}

