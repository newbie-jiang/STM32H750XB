#include "sd.h" 

#include "stdio.h" 
#include "sdmmc.h" 
#include "string.h" 
#include "fatfs.h"

///*******************************fatfs*************************************/
FATFS fs;                 // Work area (file system object) for logical drive
FIL fil;                  // file objects
uint32_t byteswritten;                /* File write counts */
uint32_t bytesread;                   /* File read counts */
uint8_t wtext[] = "This is STM32H750XB working with FatFs"; /* File write buffer */
uint8_t rtext[100];                     /* File read buffers */
char filename[] = "0:/mytest_2023_07_21.txt";

///**************************************************************************/




typedef enum {FAILED = 0, PASSED = !FAILED} TestStatus;

/* 私有宏定义 ----------------------------------------------------------------*/
#define BLOCK_SIZE            512         // SD卡块大小     
#define NUMBER_OF_BLOCKS      8           // 测试块数量(小于15)
#define WRITE_READ_ADDRESS    0x00002000  // 测试读写地址

/* 私有变量 ------------------------------------------------------------------*/
uint32_t Buffer_Block_Tx[BLOCK_SIZE*NUMBER_OF_BLOCKS]; // 写数据缓存
uint32_t Buffer_Block_Rx[BLOCK_SIZE*NUMBER_OF_BLOCKS] = {1,2}; // 读数据缓存
HAL_StatusTypeDef sd_status;    // HAL库函数操作SD卡函数返回值：操作结果
TestStatus test_status;           // 数据测试结果

/* 扩展变量 ------------------------------------------------------------------*/
/* 私有函数原形 --------------------------------------------------------------*/
void Fill_Buffer(uint32_t *pBuffer, uint32_t BufferLength, uint32_t Offset);
TestStatus Buffercmp(uint32_t* pBuffer1, uint32_t* pBuffer2, uint32_t BufferLength);
TestStatus eBuffercmp(uint32_t* pBuffer, uint32_t BufferLength);

void SD_EraseTest(void);
void SD_Write_Read_Test(void);




void get_sd_informatization(void)
{
	 
   printf("Micro SD Card Test...\r\n");
  /* 检测SD卡是否正常（处于数据传输模式的传输状态） */
  if(HAL_SD_GetCardState(&hsd1) == HAL_SD_CARD_TRANSFER)
  {      
    printf("Initialize SD card successfully!\r\n");
    // 打印SD卡基本信息
    printf(" SD card information! \r\n");
		/*实际存储容量 = LogBlockNbr * LogBlockSize*/
		printf(" LogBlockNbr   : %d \r\n", hsd1.SdCard.LogBlockNbr);	// 逻辑块数量
	  printf(" LogBlockSize  : %d \r\n", hsd1.SdCard.LogBlockSize); // 逻辑块大小(字节)
    printf(" Card Log Capacity  : %llu byte\r\n", (unsigned long long)hsd1.SdCard.BlockSize * hsd1.SdCard.BlockNbr);// 显示容量(字节)
		printf(" Card Log Capacity  : %llu M\r\n", ((unsigned long long)hsd1.SdCard.BlockSize * hsd1.SdCard.BlockNbr)/(1024*1024));// 显示容量(M)
		printf(" CardBlockNbr  : %d \r\n", hsd1.SdCard.BlockNbr);   // 物理块数量
    printf(" CardBlockSize : %d \r\n", hsd1.SdCard.BlockSize);   // 物理块大小
		printf(" Card physical Capacity  : %llu M\r\n", ((unsigned long long)hsd1.SdCard.BlockSize * hsd1.SdCard.BlockNbr)/(1024*1024));// 显示容量(字节)

    printf(" RCA           : %d \r\n", hsd1.SdCard.RelCardAdd);  // 卡相对地址
    printf(" CardType      : %d \r\n", hsd1.SdCard.CardType);    // 卡类型
    // 读取并打印SD卡的CID信息
    HAL_SD_CardCIDTypeDef sdcard_cid;
    HAL_SD_GetCardCID(&hsd1,&sdcard_cid);
    printf(" ManufacturerID: %d \r\n",sdcard_cid.ManufacturerID); /*获取sd卡制造商*/
  }
  else
  {
    printf("SD card init fail!\r\n" );
  }



}




void SD_EraseTest(void)
{
	/* 第1个参数为SD卡句柄，第2个参数为擦除起始地址，第3个参数为擦除结束地址 */
   sd_status=HAL_SD_Erase(&hsd1,WRITE_READ_ADDRESS,WRITE_READ_ADDRESS+NUMBER_OF_BLOCKS*4);
   printf("erase status:%d\r\n",sd_status);
	 HAL_Delay(100);
  if (sd_status == HAL_OK)
  {	
    /* 读取刚刚擦除的区域 */
    sd_status = HAL_SD_ReadBlocks(&hsd1,(uint8_t *)Buffer_Block_Rx,WRITE_READ_ADDRESS,NUMBER_OF_BLOCKS,0xffff);
    printf("erase read status:%d\r\n",sd_status);
    /* 把擦除区域读出来对比 */
    test_status = eBuffercmp(Buffer_Block_Rx,BLOCK_SIZE*NUMBER_OF_BLOCKS);

    if(test_status == PASSED)
      printf("》擦除测试成功！\r\n" ); 
    else	  
      printf("》擦除不成功，数据出错！\r\n" );      
  }
  else
  {
    printf("》擦除测试失败！部分SD不支持擦除，只要读写测试通过即可\r\n" );
  }
}

/**
  * 函数功能: SD卡读写测试
  * 输入参数: 无
  * 返 回 值: 无
  * 说    明: 无
  */
void SD_Write_Read_Test(void)
{  
	int i,j = 0;
  /* 填充数据到写缓存 */
  Fill_Buffer(Buffer_Block_Tx,BLOCK_SIZE*NUMBER_OF_BLOCKS, 0x32F1);
  
  /* 往SD卡写入数据 */
  sd_status = HAL_SD_WriteBlocks(&hsd1,(uint8_t *)Buffer_Block_Tx,WRITE_READ_ADDRESS,NUMBER_OF_BLOCKS,0xffff);
  printf("write status:%d\r\n",sd_status);
  HAL_Delay(200);
 // HAL_Delay(500);
  /* 从SD卡读取数据 */
  sd_status = HAL_SD_ReadBlocks(&hsd1,(uint8_t *)Buffer_Block_Rx,WRITE_READ_ADDRESS,NUMBER_OF_BLOCKS,0xffff);
  printf("read status:%d\r\n",sd_status);
  
  /* 比较数据 */
  test_status = Buffercmp(Buffer_Block_Tx, Buffer_Block_Rx, BLOCK_SIZE*NUMBER_OF_BLOCKS/4);	//比较
  if(test_status == PASSED)
	{
    printf("》读写测试成功！\r\n" );
		
		for(i=0;i<BLOCK_SIZE*NUMBER_OF_BLOCKS/4;i++)
		{
			if(j==8)
			{
				printf("\r\n");
				j=0;
			}
			
			printf("%08x   ",Buffer_Block_Rx[i]);
			j++;
		}
		printf("\r\n");
	}
  else  
  	printf("》读写测试失败！\r\n " );  
}


/**
  * 函数功能: 在缓冲区中填写数据
  * 输入参数: pBuffer：要填充的缓冲区
  *           BufferLength：要填充的大小
  *           Offset：填在缓冲区的第一个值 
  * 返 回 值: 无
  * 说    明: 无
  */
void Fill_Buffer(uint32_t *pBuffer, uint32_t BufferLength, uint32_t Offset)
{
  uint32_t index = 0;
  /* 填充数据 */
  for (index = 0; index < BufferLength; index++ )
  {
    pBuffer[index] = index + Offset;
  }
}

/**
  * 函数功能: 比较两个缓冲区中的数据是否相等
  * 输入参数: pBuffer1：要比较的缓冲区1的指针
  *           pBuffer2：要比较的缓冲区2的指针
  *           BufferLength：缓冲区长度
  * 返 回 值: PASSED：相等
  *           FAILED：不等
  * 说    明: 无
  */
TestStatus Buffercmp(uint32_t* pBuffer1, uint32_t* pBuffer2, uint32_t BufferLength)
{
  while (BufferLength--)
  {
    if(BufferLength%50==0)
    {
      printf("buf:0x%08X - 0x%08X\r\n",*pBuffer1,*pBuffer2);
    }
    if (*pBuffer1 != *pBuffer2)
    {
      return FAILED;
    }
    pBuffer1++;
    pBuffer2++;
  }
  return PASSED;
}

/**
  * 函数功能: 检查缓冲区的数据是否为0xff或0
  * 输入参数: pBuffer：要比较的缓冲区的指针
  *           BufferLength：缓冲区长度
  * 返 回 值: PASSED：缓冲区的数据全为0xff或0
  *           FAILED：缓冲区的数据至少有一个不为0xff或0 
  * 说    明: 无
  */
TestStatus eBuffercmp(uint32_t* pBuffer, uint32_t BufferLength)
{
  while (BufferLength--)
  {
    /* SD卡擦除后的可能值为0xff或0 */
    if ((*pBuffer != 0xFFFFFFFF) && (*pBuffer != 0))
    {
      return FAILED;
    }
    pBuffer++;
  }
  return PASSED;
}





void mount_sd(void)
{
	retSD = f_mount(&fs, SDPath, 1);
	if(retSD)
	{
			printf(" mount error : %d \r\n",retSD);
			Error_Handler();
	}
	else
			printf(" mount sucess!!! \r\n");

}





void fatfs_test(void)
{

printf("\r\n ****** FatFs Example ******\r\n\r\n");
    /*##-1- Register the file system object to the FatFs module ##############*/
	//f_mkfs("1:",FM_ANY,0,fatbuf,FF_MAX_SS);
	retSD = f_mount(&fs, SDPath, 1);
	if(retSD)
	{
			printf(" mount error : %d \r\n",retSD);
			Error_Handler();
	}
	else
			printf(" mount sucess!!! \r\n");
	/*##-2- Create and Open new text file objects with write access ######*/
	retSD = f_open(&fil, filename, FA_CREATE_NEW | FA_WRITE);
	if(retSD)
			printf(" open file error : %d\r\n",retSD);
	else
			printf(" open file sucess!!! \r\n");
	/*##-3- Write data to the text files ###############################*/
	retSD = f_write(&fil, wtext, sizeof(wtext), (void *)&byteswritten);
	if(retSD)
			printf(" write file error : %d\r\n",retSD);
	else
	{
			printf(" write file sucess!!! \r\n");
			printf(" write Data : %s\r\n",wtext);
	}
	/*##-4- Close the open text files ################################*/
	retSD = f_close(&fil);
	if(retSD)
			printf(" close error : %d\r\n",retSD);
	else
			printf(" close sucess!!! \r\n");
	/*##-5- Open the text files object with read access ##############*/
	retSD = f_open(&fil, filename, FA_READ);
	if(retSD)
			printf(" open file error : %d\r\n",retSD);
	else
			printf(" open file sucess!!! \r\n");
	/*##-6- Read data from the text files ##########################*/
	retSD = f_read(&fil, rtext, sizeof(rtext), (UINT*)&bytesread);
	if(retSD)
			printf(" read error!!! %d\r\n",retSD);
	else
	{
			printf(" read sucess!!! \r\n");
			printf(" read Data : %s\r\n",rtext);
	}
	/*##-7- Close the open text files ############################*/
	retSD = f_close(&fil);
	if(retSD) 
			printf(" close error!!! %d\r\n",retSD);
	else
			printf(" close sucess!!! \r\n");

}




