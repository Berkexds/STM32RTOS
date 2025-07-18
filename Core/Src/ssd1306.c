/*
 * ssd1306.c
 *
 *  Created on: Jul 16, 2025
 *      Author: berke
 */




#include "ssd1306.h"
#include "stdlib.h"
#include <string.h>

extern I2C_HandleTypeDef hi2c1;

static uint8_t SSD1306_Buffer[SSD1306_WIDTH * SSD1306_HEIGHT / 8];

#define SSD1306_I2C_ADDR 0x78

static void ssd1306_WriteCommand(uint8_t byte) {
    uint8_t data[2] = {0x00, byte};
    HAL_I2C_Master_Transmit(&hi2c1, SSD1306_I2C_ADDR, data, 2, HAL_MAX_DELAY);
}

void ssd1306_Init(void) {
    HAL_Delay(100);

    ssd1306_WriteCommand(0xAE);
    ssd1306_WriteCommand(0x20);
    ssd1306_WriteCommand(0x10);
    ssd1306_WriteCommand(0xB0);
    ssd1306_WriteCommand(0xC8);
    ssd1306_WriteCommand(0x00);
    ssd1306_WriteCommand(0x10);
    ssd1306_WriteCommand(0x40);
    ssd1306_WriteCommand(0x81);
    ssd1306_WriteCommand(0xFF);
    ssd1306_WriteCommand(0xA1);
    ssd1306_WriteCommand(0xA6);
    ssd1306_WriteCommand(0xA8);
    ssd1306_WriteCommand(0x3F);
    ssd1306_WriteCommand(0xA4);
    ssd1306_WriteCommand(0xD3);
    ssd1306_WriteCommand(0x00);
    ssd1306_WriteCommand(0xD5);
    ssd1306_WriteCommand(0xF0);
    ssd1306_WriteCommand(0xD9);
    ssd1306_WriteCommand(0x22);
    ssd1306_WriteCommand(0xDA);
    ssd1306_WriteCommand(0x12);
    ssd1306_WriteCommand(0xDB);
    ssd1306_WriteCommand(0x20);
    ssd1306_WriteCommand(0x8D);
    ssd1306_WriteCommand(0x14);
    ssd1306_WriteCommand(0xAF);

    ssd1306_Fill(Black);
    ssd1306_UpdateScreen();
}

void ssd1306_UpdateScreen(void) {
    for (uint8_t i = 0; i < 8; i++) {
        ssd1306_WriteCommand(0xB0 + i);
        ssd1306_WriteCommand(0x00);
        ssd1306_WriteCommand(0x10);

        uint8_t data[129];
        data[0] = 0x40;
        memcpy(&data[1], &SSD1306_Buffer[SSD1306_WIDTH * i], SSD1306_WIDTH);

        HAL_I2C_Master_Transmit(&hi2c1, SSD1306_I2C_ADDR, data, 129, HAL_MAX_DELAY);
    }
}

void ssd1306_Fill(SSD1306_COLOR color) {
    memset(SSD1306_Buffer, (color == Black) ? 0x00 : 0xFF, sizeof(SSD1306_Buffer));
}

void ssd1306_DrawPixel(uint8_t x, uint8_t y, SSD1306_COLOR color) {
    if (x >= SSD1306_WIDTH || y >= SSD1306_HEIGHT) return;

    if (color == White)
        SSD1306_Buffer[x + (y / 8) * SSD1306_WIDTH] |= 1 << (y % 8);
    else
        SSD1306_Buffer[x + (y / 8) * SSD1306_WIDTH] &= ~(1 << (y % 8));
}
