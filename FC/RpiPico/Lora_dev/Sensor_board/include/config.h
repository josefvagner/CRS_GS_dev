#ifndef __CONFIG_H__
#define __CONFIG_H__

#include <string.h>
#include "hardware/i2c.h"

#define SSD_RST_PIN 16

#define BNO_SCK_PIN 27
#define BMP_SDA_PIN 26
#define BMP_SCK_PIN 27
#define BNO_SDA_PIN 26
#define SSD_SDA_PIN 26
#define SSD_SCK_PIN 27

#define BNO_I2C i2c1
#define BMP_I2C i2c1
#define SSD_I2C i2c1

#endif // config.h