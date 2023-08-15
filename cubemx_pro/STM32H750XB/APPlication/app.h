
#ifndef __APP_H
#define __APP_H


#include "stm32h7xx_hal.h"
#include "ff.h"


typedef struct {
    uint8_t header[2];    // 头部固定字节
    uint8_t command;      // 命令字节
    uint8_t data_high;    // 数据高字节
    uint8_t data_low;     // 数据低字节
    uint8_t checksum;     // 校验字节
    uint8_t tail;         // 尾部固定字节
} UartPacket;




 extern uint32_t rawValue;

void ctp_test(void);
void lcd_init(void);


void bsp_init(void);
void application(void);


/*****************SD卡操作函数*****************************/
void mount_sd(void);/*挂载sd卡*/
FRESULT mkfs_sdcard(void);/*格式化sd卡  格式化前不能挂载sd卡*/

uint8_t use_createDirectory(void);/*创建文件夹*/
uint8_t use_create_a_file(void);/*创建文件*/ 
uint8_t use_writeToFile(void);/*写文件*/
uint8_t use_readFileContent(void);/*读文件*/



/**********************************************************/

#endif /* __APP_H */
