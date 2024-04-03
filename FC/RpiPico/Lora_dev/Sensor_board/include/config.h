#ifndef __CONFIG_H__
#define __CONFIG_H__

#include <string.h>
#include "hardware/i2c.h"

// #define BREADBOARD // reroute pins for use on my breadboard

/* PINS */

#ifdef BREADBOARD
// 1
// #define BNO_INT_PIN 2
#define BMP_INT_PIN 7
#define BNO_RST_PIN 6
// 7..24
#define BNO_SCK_PIN 5
#define BMP_SDA_PIN 2
#define BMP_SCK_PIN 3
#define BNO_SDA_PIN 4

#define BNO_I2C i2c0
#define BMP_I2C i2c1
#else
// 1
#define BNO_INT_PIN 2
#define BMP_INT_PIN 3
#define BNO_RST_PIN 6
// 7..24
#define BNO_SCK_PIN 27
#define BMP_SDA_PIN 26
#define BMP_SCK_PIN 27
#define BNO_SDA_PIN 26

#define BNO_I2C i2c1
#define BMP_I2C i2c1
#endif

#endif // config.h