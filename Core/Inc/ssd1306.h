/*
 * ssd1306.h
 *
 *  Created on: Jul 16, 2025
 *      Author: berke
 */

#ifndef INC_SSD1306_H_
#define INC_SSD1306_H_



#include "stm32f3xx_hal.h"

#define SSD1306_WIDTH 128
#define SSD1306_HEIGHT 64

typedef enum {
    Black = 0x00,
    White = 0x01
} SSD1306_COLOR;

void ssd1306_Init(void);
void ssd1306_Fill(SSD1306_COLOR color);
void ssd1306_UpdateScreen(void);
void ssd1306_DrawPixel(uint8_t x, uint8_t y, SSD1306_COLOR color);



#endif /* INC_SSD1306_H_ */
