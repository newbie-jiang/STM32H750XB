/**
 ****************************************************************************************************
 * @file        demo.c
 * @author      正点原子团队(ALIENTEK)
 * @version     V1.0
 * @date        2022-06-21
 * @brief       ATK-MS7620模块接近检测测试实验
 * @license     Copyright (c) 2020-2032, 广州市星翼电子科技有限公司
 ****************************************************************************************************
 * @attention
 *
 * 实验平台:正点原子 STM32F103开发板
 * 在线视频:www.yuanzige.com
 * 技术论坛:www.openedv.com
 * 公司网址:www.alientek.com
 * 购买地址:openedv.taobao.com
 *
 ****************************************************************************************************
 */

#include "demo.h"
#include "./BSP/ATK_MS7620/atk_ms7620.h"
#include "./BSP/LED/led.h"
#include "./SYSTEM/usart/usart.h"
#include "./SYSTEM/delay/delay.h"

/**
 * @brief       例程演示入口函数
 * @param       无
 * @retval      无
 */
void demo_run(void)
{
    uint8_t ret;
    uint8_t brightness;
    uint16_t size;
    
    /* 初始化ATK-MS7620模块 */
    ret = atk_ms7620_init();
    if (ret != 0)
    {
        printf("ATK-MS7620 init failed!\r\n");
        while (1)
        {
            LED0_TOGGLE();
            delay_ms(200);
        }
    }
    
    /* 配置ATK-MS7620模块为接近检测模式 */
    ret = atk_ms7620_mode_config(ATK_MS7620_MODE_PS);
    if (ret != 0)
    {
        printf("ATK_MS7620 config failed!\r\n");
        while (1)
        {
            LED0_TOGGLE();
            delay_ms(200);
        }
    }
    
    printf("ATK-MS7620 config succedded!\r\n");
    
    while (1)
    {
        /* 获取物体亮度和大小 */
        ret  = atk_ms7620_get_obj_brightness(&brightness);
        ret += atk_ms7620_get_obj_size(&size);
        if (ret == ATK_MS7620_EOK)
        {
            printf("Object brightness: %d, size: %d\r\n", brightness, size);
        }
    }
}
