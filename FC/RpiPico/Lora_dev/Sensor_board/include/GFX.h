#ifndef GFX_H
#define GFX_H

#include "SSD1309.h"
#include "pico/stdlib.h"
#include <string.h>

#define _swap_int(a, b) \
    {                   \
        int t = a;      \
        a = b;          \
        b = t;          \
    }

#define abs(x) ((x) < 0 ? -(x) : (x))

// Define a struct to simulate the class functionality
typedef struct GFX
{
    SSD1309 base; // Include base class equivalent struct
    const uint8_t *font;
    uint8_t size;
} GFX;

// Function prototypes
void GFX_init(GFX *self, uint16_t DevAddr, uint8_t width, uint8_t height, i2c_inst_t *i2c, uint8_t resetPin);
void GFX_drawChar(GFX *self, int x, int y, char chr, colors color);
void GFX_drawString(GFX *self, int x, int y, const char *str, colors color);
void GFX_drawCharArray(GFX *self, int x, int y, char *str, colors color);
void GFX_drawProgressBar(GFX *self, int x, int y, uint16_t w, uint16_t h, uint8_t progress, colors color);
void GFX_drawFillRectangle(GFX *self, int x, int y, uint16_t w, uint16_t h, colors color);
void GFX_drawRectangle(GFX *self, int x, int y, uint16_t w, uint16_t h, colors color);
void GFX_drawFastHLine(GFX *self, int x_start, int y_start, int w, colors color);
void GFX_drawFastVLine(GFX *self, int x_start, int y_start, int w, colors color);
void GFX_writeLine(GFX *self, int x_start, int y_start, int x_end, int y_end, colors color);

#endif // GFX_H
