/*********************************************************************************************************
*
* File                : ws_W25Qx.c
* Hardware Environment: 
* Build Environment   : RealView MDK-ARM  Version: 4.20
* Version             : V1.0
* By                  : 
*
*                                  (c) Copyright 2005-2011, WaveShare
*                                       http://www.waveshare.net
*                                          All Rights Reserved
*
*********************************************************************************************************/
#include <string.h>
#include "w25qxx.h"
#include "stdio.h"

/**
  * @brief  Initializes the W25Q128FV interface.
  * @retval None
  */
uint8_t BSP_W25Qx_Init(void)
{ 
	/* Reset W25Qxxx */
	BSP_W25Qx_Reset();
	
	return BSP_W25Qx_GetStatus();
}

/**
  * @brief  This function reset the W25Qx.
  * @retval None
  */
static void	BSP_W25Qx_Reset(void)
{
	uint8_t cmd[2] = {RESET_ENABLE_CMD,RESET_MEMORY_CMD};
	
	W25Qx_Enable();
	/* Send the reset command */
	HAL_SPI_Transmit(&hspi1, cmd, 2, W25Qx_TIMEOUT_VALUE);	
	W25Qx_Disable();

}

/**
  * @brief  Reads current status of the W25Q128FV.
  * @retval W25Q128FV memory status
  */
static uint8_t BSP_W25Qx_GetStatus(void)
{
	uint8_t cmd[] = {READ_STATUS_REG1_CMD};
	uint8_t status;
	
	W25Qx_Enable();
	/* Send the read status command */
	HAL_SPI_Transmit(&hspi1, cmd, 1, W25Qx_TIMEOUT_VALUE);	
	/* Reception of the data */
	HAL_SPI_Receive(&hspi1,&status, 1, W25Qx_TIMEOUT_VALUE);
	W25Qx_Disable();
	
	/* Check the value of the register */
  if((status & W25Q128FV_FSR_BUSY) != 0)
  {
    return W25Qx_BUSY;
  }
	else
	{
		return W25Qx_OK;
	}		
}

/**
  * @brief  This function send a Write Enable and wait it is effective.
  * @retval None
  */
uint8_t BSP_W25Qx_WriteEnable(void)
{
	uint8_t cmd[] = {WRITE_ENABLE_CMD};
	uint32_t tickstart = HAL_GetTick();

	/*Select the FLASH: Chip Select low */
	W25Qx_Enable();
	/* Send the read ID command */
	HAL_SPI_Transmit(&hspi1, cmd, 1, W25Qx_TIMEOUT_VALUE);	
	/*Deselect the FLASH: Chip Select high */
	W25Qx_Disable();
	
	/* Wait the end of Flash writing */
	while(BSP_W25Qx_GetStatus() == W25Qx_BUSY);
	{
		/* Check for the Timeout */
    if((HAL_GetTick() - tickstart) > W25Qx_TIMEOUT_VALUE)
    {        
			return W25Qx_TIMEOUT;
    }
	}
	
	return W25Qx_OK;
}

/**
  * @brief  Read Manufacture/Device ID.
	* @param  return value address
  * @retval None
  */
void BSP_W25Qx_Read_ID(uint8_t *ID)
{
	uint8_t cmd[4] = {READ_ID_CMD,0x00,0x00,0x00};
	
	W25Qx_Enable();
	/* Send the read ID command */
	HAL_SPI_Transmit(&hspi1, cmd, 4, W25Qx_TIMEOUT_VALUE);	
	/* Reception of the data */
	HAL_SPI_Receive(&hspi1,ID, 2, W25Qx_TIMEOUT_VALUE);
	W25Qx_Disable();
		
}

/**
  * @brief  Reads an amount of data from the QSPI memory.
  * @param  pData: Pointer to data to be read
  * @param  ReadAddr: Read start address
  * @param  Size: Size of data to read    
  * @retval QSPI memory status
  */
uint8_t BSP_W25Qx_Read(uint8_t* pData, uint32_t ReadAddr, uint32_t Size)
{
	uint8_t cmd[4];

	/* Configure the command */
	cmd[0] = READ_CMD;
	cmd[1] = (uint8_t)(ReadAddr >> 16);
	cmd[2] = (uint8_t)(ReadAddr >> 8);
	cmd[3] = (uint8_t)(ReadAddr);
	
	W25Qx_Enable();
	/* Send the read ID command */
	HAL_SPI_Transmit(&hspi1, cmd, 4, W25Qx_TIMEOUT_VALUE);	
	/* Reception of the data */
	if (HAL_SPI_Receive(&hspi1, pData,Size,W25Qx_TIMEOUT_VALUE) != HAL_OK)
  {
    return W25Qx_ERROR;
  }
	W25Qx_Disable();
	return W25Qx_OK;
}

/**
  * @brief  Writes an amount of data to the QSPI memory.
  * @param  pData: Pointer to data to be written
  * @param  WriteAddr: Write start address
  * @param  Size: Size of data to write,No more than 256byte.    
  * @retval QSPI memory status
  */
uint8_t BSP_W25Qx_Write(uint8_t* pData, uint32_t WriteAddr, uint32_t Size)
{
	uint8_t cmd[4];
	uint32_t end_addr, current_size, current_addr;
	uint32_t tickstart = HAL_GetTick();
	
	/* Calculation of the size between the write address and the end of the page */
  current_addr = 0;

  while (current_addr <= WriteAddr)
  {
    current_addr += W25Q128FV_PAGE_SIZE;
  }
  current_size = current_addr - WriteAddr;

  /* Check if the size of the data is less than the remaining place in the page */
  if (current_size > Size)
  {
    current_size = Size;
  }

  /* Initialize the adress variables */
  current_addr = WriteAddr;
  end_addr = WriteAddr + Size;
	
  /* Perform the write page by page */
  do
  {
		/* Configure the command */
		cmd[0] = PAGE_PROG_CMD;
		cmd[1] = (uint8_t)(current_addr >> 16);
		cmd[2] = (uint8_t)(current_addr >> 8);
		cmd[3] = (uint8_t)(current_addr);

		/* Enable write operations */
		BSP_W25Qx_WriteEnable();
	
		W25Qx_Enable();
    /* Send the command */
    if (HAL_SPI_Transmit(&hspi1,cmd, 4, W25Qx_TIMEOUT_VALUE) != HAL_OK)
    {
      return W25Qx_ERROR;
    }
    
    /* Transmission of the data */
    if (HAL_SPI_Transmit(&hspi1, pData,current_size, W25Qx_TIMEOUT_VALUE) != HAL_OK)
    {
      return W25Qx_ERROR;
    }
			W25Qx_Disable();
    	/* Wait the end of Flash writing */
		while(BSP_W25Qx_GetStatus() == W25Qx_BUSY);
		{
			/* Check for the Timeout */
			if((HAL_GetTick() - tickstart) > W25Qx_TIMEOUT_VALUE)
			{        
				return W25Qx_TIMEOUT;
			}
		}
    
    /* Update the address and size variables for next page programming */
    current_addr += current_size;
    pData += current_size;
    current_size = ((current_addr + W25Q128FV_PAGE_SIZE) > end_addr) ? (end_addr - current_addr) : W25Q128FV_PAGE_SIZE;
  } while (current_addr < end_addr);

	
	return W25Qx_OK;
}

/**
  * @brief  Erases the specified block of the QSPI memory. 
  * @param  BlockAddress: Block address to erase  
  * @retval QSPI memory status
  */
uint8_t BSP_W25Qx_Erase_Block(uint32_t Address)
{
	uint8_t cmd[4];
	uint32_t tickstart = HAL_GetTick();
	cmd[0] = SECTOR_ERASE_CMD;
	cmd[1] = (uint8_t)(Address >> 16);
	cmd[2] = (uint8_t)(Address >> 8);
	cmd[3] = (uint8_t)(Address);
	
	/* Enable write operations */
	BSP_W25Qx_WriteEnable();
	
	/*Select the FLASH: Chip Select low */
	W25Qx_Enable();
	/* Send the read ID command */
	HAL_SPI_Transmit(&hspi1, cmd, 4, W25Qx_TIMEOUT_VALUE);	
	/*Deselect the FLASH: Chip Select high */
	W25Qx_Disable();
	
	/* Wait the end of Flash writing */
	while(BSP_W25Qx_GetStatus() == W25Qx_BUSY);
	{
		/* Check for the Timeout */
    if((HAL_GetTick() - tickstart) > W25Q128FV_SECTOR_ERASE_MAX_TIME)
    {        
			return W25Qx_TIMEOUT;
    }
	}
	return W25Qx_OK;
}

/**
  * @brief  Erases the entire QSPI memory.This function will take a very long time.
  * @retval QSPI memory status
  */
uint8_t BSP_W25Qx_Erase_Chip(void)
{
	uint8_t cmd[4];
	uint32_t tickstart = HAL_GetTick();
	cmd[0] = SECTOR_ERASE_CMD;
	
	/* Enable write operations */
	BSP_W25Qx_WriteEnable();
	
	/*Select the FLASH: Chip Select low */
	W25Qx_Enable();
	/* Send the read ID command */
	HAL_SPI_Transmit(&hspi1, cmd, 1, W25Qx_TIMEOUT_VALUE);	
	/*Deselect the FLASH: Chip Select high */
	W25Qx_Disable();
	
	/* Wait the end of Flash writing */
	while(BSP_W25Qx_GetStatus() != W25Qx_BUSY);
	{
		/* Check for the Timeout */
    if((HAL_GetTick() - tickstart) > W25Q128FV_BULK_ERASE_MAX_TIME)
    {        
			return W25Qx_TIMEOUT;
    }
	}
	return W25Qx_OK;
}










void w25q128_test(void)
{
	static uint8_t wData[0x100];

  static uint8_t rData[0x100];
	
	  /*##-2- Erase Block ##################################*/

  if(BSP_W25Qx_Erase_Block(0) == W25Qx_OK)

      printf(" SPI Erase Block ok\r\n");

  else
	{
	 printf(" SPI Erase Block err\r\n");
	 Error_Handler();

	}

     
    

  /*##-3- Written to the flash ########################*/

  /* fill buffer */

  for(uint32_t i =0;i<0x100;i ++)

  {

          wData[i] = i;

        rData[i] = 0;

  }

    

  if(BSP_W25Qx_Write(wData,0x00,0x100)== W25Qx_OK)

      printf(" SPI Write ok\r\n");

  else
	{
		 printf(" SPI Write err\r\n");
	   Error_Handler();
	}

     

 

  /*##-4- Read the flash     ########################*/

  if(BSP_W25Qx_Read(rData,0x00,0x100)== W25Qx_OK)

      printf(" SPI Read ok\r\n\r\n");

  else
	{
		printf(" SPI Read err\r\n\r\n");
	  Error_Handler();
	}

     

    

  printf("SPI Read Data : \r\n");

  for(uint32_t i =0;i<0x100;i++)

      printf("0x%02X  ",rData[i]);

  printf("\r\n\r\n");

    

  /*##-5- check date          ########################*/   
	/**
	  比较数据是否相等
	  int memcmp(const void* ptr1, const void* ptr2, size_t num);
    参数说明：

     ptr1：指向要比较的第一个内存块的指针。
     ptr2：指向要比较的第二个内存块的指针。
     num：要比较的字节数。
     返回值：

     如果两个内存块的前 num 字节相等，则返回 0。
     如果第一个内存块的前 num 字节小于第二个内存块，则返回一个负整数。
     如果第一个内存块的前 num 字节大于第二个内存块，则返回一个正整数。
     因此，if(memcmp(wData, rData, 0x100) == 0) 这句代码的意思是，它会比较两个内存块 wData 和 rData 的前 0x100 字节是否完全相等。如果相等，条件表达式 memcmp(wData, rData, 0x100) == 0 返回 true，执行 if 语句块内的代码；如果不相等，则条件表达式返回 false，不执行 if 语句块内的代码。

     这种比较通常用于检查两个内存块是否相等，比如检查数据是否正确读取或写入，或者检查两个缓冲区的内容是否一致。
	
	
	*/

  if(memcmp(wData,rData,0x100) == 0 ) 

      printf(" W25Q128FV SPI Test OK\r\n");

  else

      printf(" W25Q128FV SPI Test False\r\n");





}


