#ifndef SSD1309_H
#define SSD1309_H

#include "hardware/i2c.h"
#include "pico/stdlib.h"
#include <stdio.h>
// #include "pico/malloc.h"
// #include "pico/mem_ops.h"
#include <stdlib.h>
#include "string.h"
#include "stdint.h"

#define SSD1309_setContrastControl 0x81
#define SSD1309_followRAMcontent 0xA4
#define SSD1309_allPixelsOn 0xA5
#define SSD1309_inversionOff 0xA6
#define SSD1309_inversionOn 0xA7
#define SSD1309_pwrOff 0xAE
#define SSD1309_pwrOn 0xAF
#define SSD1309_nop 0xE3
#define SSD1309_setCommandLock 0xFD
#define SSD1309_contHScrollSetupRight 0x26
#define SSD1309_contHScrollSetupLeft 0x27
#define SSD1309_contVHScrollSetupRight 0x29
#define SSD1309_contVHScrollSetupLeft 0x2A
#define SSD1309_deactivateScroll 0x2E
#define SSD1309_activateScroll 0x2F
#define SSD1309_setVScrollArea 0xA3
#define SSD1309_contentScrollSetupRight 0x2C
#define SSD1309_contentScrollSetupLeft 0x2D
#define SSD1309_setLowCSAinPAM 0x00
#define SSD1309_setHighCSAinPAM 0x10
#define SSD1309_setMemoryAddressingMode 0x20
#define SSD1309_setColumnAddress 0x21
#define SSD1309_setPageAddress 0x22
#define SSD1309_setPSAinPAM 0xB0
#define SSD1309_setDisplayStartLine 0x40
#define SSD1309_setSegmentMapReset 0xA0
#define SSD1309_setSegmentMapFlipped 0xA1
#define SSD1309_setMultiplexRatio 0xA8
#define SSD1309_setCOMoutputNormal 0xC0
#define SSD1309_setCOMoutputFlipped 0xC8
#define SSD1309_setDisplayOffset 0xD3
#define SSD1309_setCOMpinsHWconfig 0xDA
#define SSD1309_setGPIO = 0xDC
#define SSD1309_setDisplayClockDivideRatio 0xD5
#define SSD1309_setPreChargePeriod 0xD9
#define SSD1309_setVCOMHdeselectLevel 0xDB

typedef enum
{
	BLACK,
	WHITE,
	INVERSE
} colors;

typedef enum
{
	FRAMES_5,
	FRAMES_64,
	FRAMES_128,
	FRAMES_256,
	FRAMES_2,
	FRAMES_3,
	FRAMES_4,
	FRAMES_1
} scrollInterval;

typedef struct
{
	uint16_t DevAddr;
	i2c_inst_t *i2c;
	uint8_t width;
	uint8_t height;
	uint8_t resetPin;

	unsigned char *buffer;
} SSD1309;

void SSD1309_sendData(SSD1309 *ssd1309, uint8_t *buffer, size_t buff_size);
void SSD1309_sendCommand(SSD1309 *ssd1309, uint8_t command);

void SSD1309_create(SSD1309 *ssd1309, uint16_t DevAddr, uint8_t width, uint8_t height, i2c_inst_t *i2c, uint8_t resetPin);
void SSD1309_destroy(SSD1309 *ssd1309);

void SSD1309_reset(SSD1309 *ssd1309);
void SSD1309_displayON(SSD1309 *ssd1309, uint8_t On);
void SSD1309_invertColors(SSD1309 *ssd1309, uint8_t Invert);
void SSD1309_rotateDisplay(SSD1309 *ssd1309, uint8_t Rotate);
void SSD1309_setContrast(SSD1309 *ssd1309, uint8_t Contrast);
void SSD1309_scrollVerticalyLeft(SSD1309 *ssd1309, uint8_t startColumn, uint8_t endColumn, uint8_t startRow, uint8_t endRow, scrollInterval interval, uint8_t horizontal, uint8_t scrollingOffset);
void SSD1309_scrollHorizontalLeft(SSD1309 *ssd1309, uint8_t startColumn, uint8_t endColumn, uint8_t startRow, uint8_t endRow, scrollInterval interval);
void SSD1309_scrollVerticalyRight(SSD1309 *ssd1309, uint8_t startColumn, uint8_t endColumn, uint8_t startRow, uint8_t endRow, scrollInterval interval, uint8_t horizontal, uint8_t scrollingOffset);
void SSD1309_scrollHorizontalRight(SSD1309 *ssd1309, uint8_t startColumn, uint8_t endColumn, uint8_t startRow, uint8_t endRow, scrollInterval interval);

void SSD1309_drawPixel(SSD1309 *ssd1309, int16_t x, int16_t y, colors Color);
void SSD1309_clear(SSD1309 *ssd1309, colors Color);
void SSD1309_display(SSD1309 *ssd1309, unsigned char *data);
uint8_t SSD1309_getHeight(SSD1309 *ssd1309);
uint8_t SSD1309_getWidth(SSD1309 *ssd1309);

#endif // SSD1309_H
