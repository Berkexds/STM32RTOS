#include "fonts.h"

// 7x10 font tablosu (ASCII 32-126)
static const uint16_t Font7x10 [] = {
    // ASCII ' ' (32) - Boşluk
    0x0000, 0x0000, 0x0000, 0x0000, 0x0000,
    // ASCII '!' (33)
    0x0000, 0x0000, 0xBE00, 0x0000, 0x0000,
    // ASCII '"' (34)
    0x0000, 0x0E00, 0x0000, 0x0E00, 0x0000,
    // ... buraya tüm karakter tablosu eklenir
};

// Font tanımları
FontDef Font_7x10 = {7, 10, Font7x10};

// Örnek başka fontlar (eğer ihtiyacın yoksa silebilirsin)
static const uint16_t Font11x18 [] = {
    // ASCII ' ' (32)
    0x0000, 0x0000, 0x0000, 0x0000, 0x0000,
    // Diğer karakterler
};

FontDef Font_11x18 = {11, 18, Font11x18};

static const uint16_t Font16x26 [] = {
    // ASCII ' ' (32)
    0x0000, 0x0000, 0x0000, 0x0000, 0x0000,
    // Diğer karakterler
};

FontDef Font_16x26 = {16, 26, Font16x26};
