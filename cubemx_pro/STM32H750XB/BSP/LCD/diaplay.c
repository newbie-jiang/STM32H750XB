#include "display.h"
#include "lcd.h"
#include "touch.h"
/*********************************************************************************************************************/
/**
  * @brief  draw_grid() : 绘制网格
  * @param  screen_width ：全屏宽     
  * @param  screen_height: 全屏高
  * @param  cell_width:  每个网格宽
  * @param  cell_height: 每个网格高
	* @param  grid_line_width: 绘制线宽
	* @param  COLOR: 颜色
  * @retval None
  */
	
//draw_grid(screen_width, screen_height, cell_width, cell_height, grid_line_width,COLOR);
/*********************************************************************************************************************/
void draw_grid(uint16_t screen_width, 
	             uint16_t screen_height, 
               uint16_t cell_width,    
               uint16_t cell_height,   
               uint8_t grid_line_width,
               uint32_t COLOR)
{
    
   // 画垂直线
    for (uint16_t x = 0; x <= screen_width; x += cell_width) {
        for (uint16_t y = 0; y < screen_height; y++) {
            for (uint16_t offset = 0; offset < grid_line_width; offset++) {
                lcd_draw_point(x + offset, y, COLOR);
            }
        }
    }

    // 画水平线
    for (uint16_t y = 0; y <= screen_height; y += cell_height) {
        for (uint16_t x = 0; x < screen_width; x++) {
            for (uint16_t offset = 0; offset < grid_line_width; offset++) {
                lcd_draw_point(x, y + offset, COLOR);
            }
        }
    }
}
/*----------------------------------------------------------------------------------------------------------------------*/

void touch_key(void)
{
  /*支持单击，双击，长按*/

	
	

}
