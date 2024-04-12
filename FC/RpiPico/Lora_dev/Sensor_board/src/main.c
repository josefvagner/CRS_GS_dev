/* pico SDK libraries*/
#include "pico/stdlib.h"
#include <stdio.h>
#include "hardware/gpio.h"
#include "hardware/i2c.h"
// #include "RP2040.h"
/* custom libraries */
#include "bmp3.h"
#include "bno055.h"
#include "common.h"
#include "GFX.h"
#include "SSD1309.h"
/* configuration */
#include "config.h"

struct bmp3_dev bmp388;
struct bmp3_data bmpData = {0};
struct bmp3_settings bmpSettings = {0};
struct bmp3_status bmpStatus = {{0}};

struct bno055_t bno055;
struct bno055_all_float_t
{
    struct bno055_accel_float_t accel;            /* Accelerometer data */
    struct bno055_mag_float_t mag;                /* Magnetometer data */
    struct bno055_gyro_float_t gyro;              /* Gyroscope data */
    struct bno055_euler_float_t euler;            /* Euler angle data */
    struct bno055_linear_accel_float_t lin_accel; /* Linear acceleration data */
    struct bno055_gravity_float_t grav_accel;     /* Gravity acceleration data */
};

struct GFX gfx;

s8 BNO055_I2C_bus_read(u8 dev_addr, u8 reg_addr, u8 *reg_data, u8 cnt)
{
    int ret = i2c_write_blocking(BNO_I2C, dev_addr, &reg_addr, 1, true);
    ret += i2c_read_blocking(BNO_I2C, dev_addr, reg_data, cnt, false);

    if (ret > 0)
        ret = 0; // compatibility with BNO driver
    return ret;
}

s8 BNO055_I2C_bus_write(u8 dev_addr, u8 reg_addr, u8 *reg_data, u8 cnt)
{
    uint8_t buf[cnt + 1];
    buf[0] = reg_addr; // first written byte is register address to write to (start writing from)
    memcpy(buf + 1, reg_data, cnt);
    int ret = i2c_write_blocking(BNO_I2C, dev_addr, buf, cnt + 1, false);
    if (ret > 0)
        ret = 0; // compatibility with BNO driver
    return ret;
}

void BNO055_delay_msek(u32 msek)
{
    sleep_ms(msek); // Delay using millisecond sleep
}

void fail();

bool reserved_addr(uint8_t addr)
{
    return (addr & 0x78) == 0 || (addr & 0x78) == 0x78;
}

int main()
{
    // Initialize the Pico SDK
    stdio_init_all();

    sleep_us(500);

    /* I2C setup*/

    i2c_init(BNO_I2C, 100000);
    gpio_set_function(BNO_SCK_PIN, GPIO_FUNC_I2C);
    gpio_set_function(BNO_SDA_PIN, GPIO_FUNC_I2C);
    gpio_pull_up(BNO_SCK_PIN);
    gpio_pull_up(BNO_SDA_PIN);

    /* Sensor setup */
    bno055.bus_write = BNO055_I2C_bus_write;
    bno055.bus_read = BNO055_I2C_bus_read;
    bno055.delay_msec = BNO055_delay_msek;
    bno055.dev_addr = BNO055_I2C_ADDR1;

    sleep_ms(1000);

    /* SCAN for BNO */
    // TODO remove, only for testing
    printf("expecting BNO at I2C pins SCL: %d, SDA: %d\n", BNO_SCK_PIN, BNO_SDA_PIN);
    for (int addr = 0; addr < (1 << 7); ++addr)
    {
        int ret0;
        uint8_t rxdata;
        if (reserved_addr(addr))
        {
            ret0 = PICO_ERROR_GENERIC;
        }
        else
        {
            ret0 = i2c_read_blocking(BNO_I2C, addr, &rxdata, 1, false);
        }
        if (ret0 >= 0)
        {
            printf("I2C (%d&%d): found address: 0x%02x\n", BNO_SCK_PIN, BNO_SDA_PIN, addr);
        }
    }

    printf("123\n");

    int ret;
    // Initialize BNO055 sensor
    ret = bno055_init(&bno055);
    if (ret != BNO055_SUCCESS)
    {
        printf("BNO init failed\n");
        fail();
    }

    // Set power mode to normal
    ret = bno055_set_power_mode(BNO055_POWER_MODE_NORMAL);
    if (ret != BNO055_SUCCESS)
    {
        fail();
    }
    // Set operation mode to config
    ret = bno055_set_operation_mode(BNO055_OPERATION_MODE_CONFIG);
    if (ret != BNO055_SUCCESS)
    {
        fail();
    }

    // Set accelerometer power mode to normal
    ret = bno055_set_accel_power_mode(BNO055_ACCEL_NORMAL);
    if (ret != BNO055_SUCCESS)
    {
        fail();
    }

    // Set gyro power mode to normal
    ret = bno055_set_gyro_power_mode(BNO055_GYRO_POWER_MODE_NORMAL);
    if (ret != BNO055_SUCCESS)
    {
        fail();
    }

    // Set magnetometer power mode to normal
    ret = bno055_set_mag_power_mode(BNO055_MAG_POWER_MODE_NORMAL);
    if (ret != BNO055_SUCCESS)
    {
        fail();
    }

    // Set operation mode to NDOF
    ret = bno055_set_operation_mode(BNO055_OPERATION_MODE_NDOF);
    if (ret != BNO055_SUCCESS)
    {
        fail();
    }

    int8_t rslt;
    uint8_t loop = 0;
    uint16_t settings_sel;

    rslt = bmp3_interface_init(&bmp388, BMP3_I2C_INTF);
    bmp3_check_rslt("bmp3_interface_init", rslt);

    rslt = bmp3_init(&bmp388);
    bmp3_check_rslt("bmp3_init", rslt);

    bmpSettings.int_settings.drdy_en = BMP3_ENABLE;
    bmpSettings.press_en = BMP3_ENABLE;
    bmpSettings.temp_en = BMP3_ENABLE;

    bmpSettings.odr_filter.press_os = BMP3_OVERSAMPLING_2X;
    bmpSettings.odr_filter.temp_os = BMP3_OVERSAMPLING_2X;
    bmpSettings.odr_filter.odr = BMP3_ODR_100_HZ;

    settings_sel = BMP3_SEL_PRESS_EN | BMP3_SEL_TEMP_EN | BMP3_SEL_PRESS_OS | BMP3_SEL_TEMP_OS | BMP3_SEL_ODR |
                   BMP3_SEL_DRDY_EN;

    rslt = bmp3_set_sensor_settings(settings_sel, &bmpSettings, &bmp388);
    bmp3_check_rslt("bmp3_set_sensor_settings", rslt);

    bmpSettings.op_mode = BMP3_MODE_NORMAL;
    rslt = bmp3_set_op_mode(&bmpSettings, &bmp388);
    bmp3_check_rslt("bmp3_set_op_mode", rslt);

    sleep_ms(1000);

    GFX_init(&gfx, 0x3C, 128, 64, SSD_I2C, SSD_RST_PIN);
    printf("GFX initialized\n");
    // SSD1309_displayON(&gfx.base, 1);
    // printf("Display ON\n");
    SSD1309_clear(&gfx.base, BLACK);
    printf("Display cleared\n");
    GFX_drawString(&gfx, 20, 20, "Hello World!", WHITE);
    printf("String drawn\n");
    SSD1309_display(&gfx.base, NULL);

    printf("Starting loop\n");

    for (;;)
    {
        rslt = bmp3_get_status(&bmpStatus, &bmp388);
        bmp3_check_rslt("bmp3_get_status", rslt);

        if ((rslt == BMP3_OK) && (bmpStatus.intr.drdy == BMP3_ENABLE))
        {
            rslt = bmp3_get_sensor_data(BMP3_PRESS_TEMP, &bmpData, &bmp388);
            bmp3_check_rslt("bmp3_get_sensor_data", rslt);
            rslt = bmp3_get_status(&bmpStatus, &bmp388);
            bmp3_check_rslt("bmp3_get_status", rslt);
        }

        struct bno055_all_float_t data;
        if (bno055_convert_float_accel_xyz_msq(&data.accel) == BNO055_ERROR)
            fail();
        if (bno055_convert_float_gyro_xyz_dps(&data.gyro) == BNO055_ERROR)
            fail();

        printf("Accel X: %05.3f, Y: %05.3f, Z: %05.3f\t(m/s2)\t|\t", data.accel.x, data.accel.y, data.accel.z);
        printf("Gyro  X: %05.1f, Y: %05.1f, Z: %05.1f\t(deg/s)\t|\t", data.gyro.x, data.gyro.y, data.gyro.z);
        printf("BMP T: %.2f deg C, P: %.2f Pa\n", (bmpData.temperature), (bmpData.pressure));
        sleep_ms(500);
    }
}

void fail()
{
    while (1)
    {
        sleep_ms(1000);
        printf("Fail\n");
    }
}