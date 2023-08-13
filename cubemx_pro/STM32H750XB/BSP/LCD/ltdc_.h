/**
 ****************************************************************************************************
 * @file        ltdc.h
 * @author      正点原子团队(ALIENTEK)
 * @version     V1.0
 * @date        2022-4-20
 * @brief       LTDC 驱动代码
 * @license     Copyright (c) 2020-2032, 广州市星翼电子科技有限公司
 ****************************************************************************************************
 * @attention
 *
 * 实验平台:正点原子 阿波罗 F429开发板
 * 在线视频:www.yuanzige.com
 * 技术论坛:www.openedv.com
 * 公司网址:www.alientek.com
 * 购买地址:openedv.taobao.com
 *
 * 修改说明
 * V1.0 20220420
 * 第一次发布
 *
 ****************************************************************************************************
 */


#ifndef _LCD_H
#define _LCD_H

#include "sys.h"

/* LCD LTDC重要参数集 */
typedef struct  
{
    uint32_t pwidth;      /* LCD面板的宽度,固定参数,不随显示方向改变,如果为0,说明没有任何RGB屏接入 */
    uint32_t pheight;     /* LCD面板的高度,固定参数,不随显示方向改变 */
    uint16_t hsw;         /* 水平同步宽度 */
    uint16_t vsw;         /* 垂直同步宽度 */
    uint16_t hbp;         /* 水平后廊 */
    uint16_t vbp;         /* 垂直后廊 */
    uint16_t hfp;         /* 水平前廊 */
    uint16_t vfp;         /* 垂直前廊  */
    uint8_t activelayer;  /* 当前层编号:0/1 */
    uint8_t dir;          /* 0,竖屏;1,横屏; */
    uint16_t width;       /* LCD宽度 */
    uint16_t height;      /* LCD高度 */
    uint32_t pixsize;     /* 每个像素所占字节数 */
}_ltdc_dev; 

extern _ltdc_dev lcdltdc;                   /* 管理LCD LTDC参数 */
extern LTDC_HandleTypeDef g_ltdc_handle;    /* LTDC句柄 */
extern DMA2D_HandleTypeDef g_dma2d_handle;  /* DMA2D句柄 */

#define LCD_PIXEL_FORMAT_ARGB8888       0X00
#define LCD_PIXEL_FORMAT_RGB888         0X01
#define LCD_PIXEL_FORMAT_RGB565         0X02
#define LCD_PIXEL_FORMAT_ARGB1555       0X03
#define LCD_PIXEL_FORMAT_ARGB4444       0X04
#define LCD_PIXEL_FORMAT_L8             0X05
#define LCD_PIXEL_FORMAT_AL44           0X06
#define LCD_PIXEL_FORMAT_AL88           0X07

/******************************************************************************************/
/*用户修改配置部分:

 * 定义颜色像素格式,一般用RGB565 */
#define LCD_PIXFORMAT       LCD_PIXEL_FORMAT_RGB565
/* 定义默认背景层颜色 */
#define LTDC_BACKLAYERCOLOR           0X00000000
/* LCD帧缓冲区首地址,这里定义在SDRAM里面. */
#define LCD_FRAME_BUF_ADDR            0XC0000000  

void ltdc_switch(uint8_t sw);                                                                                                                                        /* LTDC开关 */
void ltdc_layer_switch(uint8_t layerx, uint8_t sw);                                                                                                                  /* 层开关 */
void ltdc_select_layer(uint8_t layerx);                                                                                                                              /* 层选择 */
void ltdc_display_dir(uint8_t dir);                                                                                                                                  /* 显示方向控制 */
void ltdc_draw_point(uint16_t x, uint16_t y, uint32_t color);                                                                                                        /* 画点函数 */
uint32_t ltdc_read_point(uint16_t x, uint16_t y);                                                                                                                    /* 读点函数 */
void ltdc_fill(uint16_t sx, uint16_t sy, uint16_t ex, uint16_t ey, uint32_t color);                                                                                  /* 矩形单色填充函数 */
void ltdc_color_fill(uint16_t sx, uint16_t sy, uint16_t ex, uint16_t ey, uint16_t *color);                                                                           /* 矩形彩色填充函数 */
void ltdc_clear(uint32_t color);                                                                                                                                     /* 清屏函数 */
uint8_t ltdc_clk_set(uint32_t pllsain, uint32_t pllsair, uint32_t pllsaidivr);                                                                                       /* LTDC时钟配置 */
void ltdc_layer_window_config(uint8_t layerx, uint16_t sx, uint16_t sy, uint16_t width, uint16_t height);                                                            /* LTDC层窗口设置 */
void ltdc_layer_parameter_config(uint8_t layerx, uint32_t bufaddr, uint8_t pixformat, uint8_t alpha, uint8_t alpha0, uint8_t bfac1, uint8_t bfac2, uint32_t bkcolor);/* LTDC基本参数设置 */
uint16_t ltdc_panelid_read(void);                                                                                                                                    /* LCD ID读取函数 */
void ltdc_init(void);                                                                                                                                                /* LTDC初始化函数 */


#endif 
