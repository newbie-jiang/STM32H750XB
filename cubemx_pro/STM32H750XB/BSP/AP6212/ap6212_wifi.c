#include "ap6212_wifi.h"

#include "stdio.h" 
#include "sdmmc.h" 
#include "string.h" 
#include "fatfs.h"













void get_ap6212_wifi_informatization(void)
{
	  HAL_SD_CardCIDTypeDef sdcard_cid;
    HAL_SD_GetCardCID(&hsd2,&sdcard_cid);
    printf("wifi weak Test...\r\n");
	  printf(" ManufacturerID: %d \r\n",sdcard_cid.ManufacturerID); /*获取sd卡制造商*/
	 uint32_t state =HAL_SD_GetCardState(&hsd2);
	
	printf("%04d\r\n",state);
	
  /* 检测SD卡是否正常（处于数据传输模式的传输状态） */
  if(HAL_SD_GetCardState(&hsd2) == HAL_SD_CARD_READY)
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
  
   
  }
  else
  {
    printf("wifi init fail!\r\n" );
  }



}



