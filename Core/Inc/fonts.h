#ifndef FONTS_H_
#define FONTS_H_

#include "stdint.h"

// Font yapısı
typedef struct {
    const uint8_t width;     // Karakter genişliği
    const uint8_t height;    // Karakter yüksekliği
    const uint16_t *data;    // Karakter verileri
} FontDef;

// 7x10 Font
extern FontDef Font_7x10;

// 11x18 Font
extern FontDef Font_11x18;

// 16x26 Font
extern FontDef Font_16x26;

#endif /* FONTS_H_ */
