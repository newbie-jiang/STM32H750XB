
#ifndef __W25Q64_H
#define __W25Q64_H

#ifdef __cplusplus
 extern "C" {
#endif 


#include "stm32h7xx.h"
#include "spi.h"
#include "main.h" 





#define ManufactDeviceID_CMD	0x90
#define READ_STATU_REGISTER_1   0x05
#define READ_STATU_REGISTER_2   0x35
#define READ_DATA_CMD	        0x03
#define WRITE_ENABLE_CMD	    0x06
#define WRITE_DISABLE_CMD	    0x04
#define SECTOR_ERASE_CMD	    0x20
#define CHIP_ERASE_CMD	        0xc7
#define PAGE_PROGRAM_CMD        0x02


HAL_StatusTypeDef QSPI_Transmit(uint8_t* send_buf, uint32_t size);
HAL_StatusTypeDef QSPI_Receive(uint8_t* recv_buf, uint32_t size);
uint16_t W25QXX_ReadID(void);
void W25QXX_Read(uint8_t* dat_buffer, uint32_t start_read_addr, uint16_t byte_to_read);
uint8_t W25QXX_ReadSR(uint8_t reg);
void W25QXX_Wait_Busy(void);
void W25QXX_Write_Enable(void);
void W25QXX_Write_Disable(void);
void W25QXX_Erase_Sector(uint32_t sector_addr);
void W25QXX_Page_Program(uint8_t* dat, uint32_t WriteAddr, uint16_t byte_to_write);









  
#ifdef __cplusplus
}
#endif

#endif /* __W25Q64H */

