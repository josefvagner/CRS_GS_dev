// ssd1309_wrapper.h
#ifndef SSD1309_WRAPPER_H
#define SSD1309_WRAPPER_H

#include <stdint.h>

typedef enum
{
    BLACK = 0,
    WHITE = 1,
    INVERSE = 2
} colors;

#ifdef __cplusplus
extern "C"
{
#endif
    void display();
    void initGFX(spi_inst_t *spi);
    void drawChar(int x, int y, char chr, colors color);
    void drawString(int x, int y, const char *str, colors color);
    void drawCharArray(int x, int y, char *str, colors color);

    void drawProgressBar(int x, int y, uint16_t w, uint16_t h, uint8_t progress, colors color);
    void drawFillRectangle(int x, int y, uint16_t w, uint16_t h, colors color);
    void drawRectangle(int x, int y, uint16_t w, uint16_t h, colors color);
    void drawFastHLine(int x_start, int y_start, int w, colors color);
    void drawFastVLine(int x_start, int y_start, int w, colors color);
    void writeLine(int x_start, int y_start, int x_end, int y_end, colors color);

    void clear();

#ifdef __cplusplus
}
#endif

#endif // SSD1309_WRAPPER_H
