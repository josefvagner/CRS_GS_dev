#include <stdio.h>
#include "pico/stdlib.h"
#include "hardware/i2c.h"
#include "bno055.h"

#define BNO055_I2C_BUS_WRITE_ARRAY_INDEX ((u8)1)
#define I2C_BUFFER_LEN 8
#define I2C_SDA 4
#define I2C_SCL 5

s8 BNO055_I2C_bus_read(u8 dev_addr, u8 reg_addr, u8 *reg_data, u8 cnt);
s8 BNO055_I2C_bus_write(u8 dev_addr, u8 reg_addr, u8 *reg_data, u8 cnt);
s8 I2C_routine(void);
void BNO055_delay_msek(u32 msek);

i2c_inst_t *i2c = i2c0;

struct bno055_t bno055;
struct bno055_accel_t accel_xyz;
// struct bno055_mag_t mag_xyz;
// struct bno055_gyro_t gyro_xyz;
// struct bno055_euler_t euler_hrp;
// struct bno055_quaternion_t quaternion_wxyz;
// struct bno055_linear_accel_t linear_acce_xyz;
// struct bno055_gravity_t gravity_xyz;

int main()
{
    stdio_init_all();
    sleep_ms(1000);
    printf("ahoj\n");
    s32 comres = BNO055_ERROR;

    gpio_set_function(I2C_SDA, GPIO_FUNC_I2C);
    gpio_set_function(I2C_SCL, GPIO_FUNC_I2C);
    gpio_pull_up(I2C_SDA);
    gpio_pull_up(I2C_SCL);

    i2c_init(i2c, 400 * 1000);

    I2C_routine();
    comres = bno055_init(&bno055);
    printf("comres: %d\n", comres);
    if (comres != BNO055_SUCCESS)
    {
        while (true)
        {
            printf("error BNO\n");
            sleep_ms(1000);
        }
    }
    comres += bno055_set_power_mode(BNO055_POWER_MODE_NORMAL);
    comres += bno055_set_operation_mode(BNO055_OPERATION_MODE_AMG);
    // Loop forever
    while (true)
    {
        comres += bno055_read_accel_xyz(&accel_xyz);
        printf("comres: %d | X: %.2f | Y: %.2f | Z: %.2f\r\n", accel_xyz.x, accel_xyz.y, accel_xyz.z);

        sleep_ms(100);
    }
}

s8 I2C_routine(void)
{
    bno055.bus_write = BNO055_I2C_bus_write;
    bno055.bus_read = BNO055_I2C_bus_read;
    bno055.delay_msec = BNO055_delay_msek;
    bno055.dev_addr = BNO055_I2C_ADDR2;

    return BNO055_INIT_VALUE;
}

s8 BNO055_I2C_bus_write(u8 dev_addr, u8 reg_addr, u8 *reg_data, u8 cnt)
{
    u8 array[cnt + 1];
    u8 stringpos = BNO055_INIT_VALUE;

    array[BNO055_INIT_VALUE] = reg_addr;
    for (stringpos = BNO055_INIT_VALUE; stringpos < cnt; stringpos++)
    {
        array[stringpos + BNO055_I2C_BUS_WRITE_ARRAY_INDEX] = *(reg_data + stringpos);
    }

    int ret = i2c_write_blocking(i2c, dev_addr, array, (size_t)(cnt + 1), false);

    printf("spi write ret: %d | exp: %d\n", ret, cnt);

    if ((u8)ret != cnt + 1)
    {
        printf("error write\n");
        return (s8)BNO055_ERROR;
    }

    return (s8)BNO055_SUCCESS;
}

s8 BNO055_I2C_bus_read(u8 dev_addr, u8 reg_addr, u8 *reg_data, u8 cnt)
{
    i2c_write_blocking(i2c, dev_addr, &reg_addr, 1, true);
    int ret = i2c_read_blocking(i2c, dev_addr, reg_data, (size_t)cnt, false);
    printf("spi read ret: %d | exp: %d\n", ret, cnt);
    if ((u8)ret != cnt)
    {
        printf("error read\n");
        return (s8)BNO055_ERROR;
    }

    return (s8)BNO055_SUCCESS;
}

void BNO055_delay_msek(u32 msek)
{
    sleep_ms(msek);
}
