// ssd1309_wrapper.cpp
#include "GFX.hpp"

// Global instances of SSD1309 and GFX
static GFX *gfx_display = nullptr;

extern "C" void initGFX(spi_inst_t *spi)
{
    printf("initGFX\n");
    sleep_ms(1000);
    static GFX tmp(128, 64, spi, 24, 17, 23);
    gfx_display = &tmp;
}

extern "C" void display()
{
    gfx_display->display();
}

extern "C" void drawChar(int x, int y, char chr, colors color)
{
    gfx_display->drawChar(x, y, chr, color);
}
extern "C" void drawString(int x, int y, const char *str, colors color)
{
    gfx_display->drawString(x, y, str, color);
}
extern "C" void drawCharArray(int x, int y, char *str, colors color)
{
    gfx_display->drawCharArray(x, y, str, color);
}

extern "C" void drawProgressBar(int x, int y, uint16_t w, uint16_t h, uint8_t progress, colors color)
{
    gfx_display->drawProgressBar(x, y, w, h, progress, color);
}
extern "C" void drawFillRectangle(int x, int y, uint16_t w, uint16_t h, colors color)
{
    gfx_display->drawFillRectangle(x, y, w, h, color);
}
extern "C" void drawRectangle(int x, int y, uint16_t w, uint16_t h, colors color)
{
    gfx_display->drawRectangle(x, y, w, h, color);
}
extern "C" void drawFastHLine(int x_start, int y_start, int w, colors color)
{
    gfx_display->drawFastHLine(x_start, y_start, w, color);
}
extern "C" void drawFastVLine(int x_start, int y_start, int w, colors color)
{
    gfx_display->drawFastVLine(x_start, y_start, w, color);
}
extern "C" void writeLine(int x_start, int y_start, int x_end, int y_end, colors color)
{
    gfx_display->writeLine(x_start, y_start, x_end, y_end, color);
}
extern "C" void clear()
{
    gfx_display->clear();
}
