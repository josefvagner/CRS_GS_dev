#ifndef _WS2812_H
#define _WS2812_H

#include "pico/stdlib.h"
#include <stdlib.h>

struct rgb_s
{
	uint8_t     r;
	uint8_t     g;
	uint8_t     b;
};

typedef struct ws2812_s
{
    uint8_t         pin;    // GPIO pin WS2812 is connected to
    uint8_t         num;    // number of LED's
    struct rgb_s*   rgb;    // Flexible array member
} ws2812_t;

bool ws2812_setup(ws2812_t *ws2812, uint8_t pin, uint8_t num);
void ws2812_set(ws2812_t *ws2812, uint8_t led, uint8_t r, uint8_t g, uint8_t b);
void ws2812_set_all(ws2812_t *ws2812, uint8_t r, uint8_t g, uint8_t b);
void ws2812_show(ws2812_t *ws2812);

#endif // _WS2812_H
