#ifndef __CONFIG_H
#define __CONFIG_H

#include "LCD_1in28.h"

#define DISPLAY_WIDTH   LCD_1IN28_WIDTH
#define DISPLAY_HEIGHT  LCD_1IN28_HEIGHT

// helper function for defining colors in RGB565 mode with MSB-first byte-order
#define RGB565(r,g,b) __bswap16(((r & 0xF8) << 8) | ((g & 0xFC) << 3) | ((b & 0xF8) >> 3))

#endif
