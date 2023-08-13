#ifndef __DISPLAY_H
#define __DISPLAY_H

#include "stdint.h"
void draw_grid(uint16_t screen_width, 
	             uint16_t screen_height, 
               uint16_t cell_width,    
               uint16_t cell_height,   
               uint8_t grid_line_width,
               uint32_t COLOR);















#endif   /*__DISPLAY_H*/