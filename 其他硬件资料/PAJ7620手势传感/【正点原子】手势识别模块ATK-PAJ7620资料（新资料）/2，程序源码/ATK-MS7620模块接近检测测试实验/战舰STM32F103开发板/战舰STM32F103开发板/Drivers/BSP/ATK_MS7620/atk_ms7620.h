/**
 ****************************************************************************************************
 * @file        atk_ms7620.h
 * @author      正点原子团队(ALIENTEK)
 * @version     V1.0
 * @date        2022-06-21
 * @brief       ATK-MS7620模块驱动代码
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

#ifndef __ATK_MS7620_H
#define __ATK_MS7620_H

#include "./SYSTEM/sys/sys.h"

/* ATK-MS7620模块模式枚举 */
typedef enum
{
    ATK_MS7620_MODE_PS = 0x00,  /* 接近检测模式 */
    ATK_MS7620_MODE_GESTURE,    /* 手势检测模式 */
} atk_ms7620_mode_t;

/* ATK-MS7620模块手势枚举 */
typedef enum
{
    ATK_MS7620_GESTURE_UP = 0x00,
    ATK_MS7620_GESTURE_DOWN,
    ATK_MS7620_GESTURE_LEFT,
    ATK_MS7620_GESTURE_RIGHT,
    ATK_MS7620_GESTURE_FORWARD,
    ATK_MS7620_GESTURE_BACKWARD,
    ATK_MS7620_GESTURE_CLOCKWISE,
    ATK_MS7620_GESTURE_ANTICLOCKWISE,
    ATK_MS7620_GESTURE_WAVE,
} atk_ms7620_gesture_t;

/* 函数错误代码 */
#define ATK_MS7620_EOK      0   /* 没有错误 */
#define ATK_MS7620_ERROR    1   /* 通用错误 */
#define ATK_MS7620_EINVAL   2   /* 参数错误 */
#define ATK_MS7620_EACK     3   /* IIC通讯ACK错误 */

/* 操作函数 */
uint8_t atk_ms7620_init(void);                                  /* ATK-MS7620模块初始化 */
uint8_t atk_ms7620_mode_config(atk_ms7620_mode_t mode);         /* 配置ATK-MS7620模块的模式 */
uint8_t atk_ms7620_get_obj_brightness(uint8_t *brightness);     /* ATK-MS7620模块获取物体亮度 */
uint8_t atk_ms7620_get_obj_size(uint16_t *size);                /* ATK-MS7620模块获取物体大小 */
uint8_t atk_ms7620_get_gesture(atk_ms7620_gesture_t *gesture);  /* ATK-MS7620模块获取手势 */

#endif
