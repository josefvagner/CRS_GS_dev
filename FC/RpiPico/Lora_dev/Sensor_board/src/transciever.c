/* pico SDK libraries*/
#include "pico/stdlib.h"
#include <stdio.h>
#include "hardware/gpio.h"
#include "hardware/i2c.h"
#include "hardware/pio.h"
// #include "RP2040.h"
/* custom libraries */
#include "bmp3.h"
#include "bno055.h"
#include "common.h"
#include "sx1280_spi.h"
#include "quadrature.pio.h"
#include "GFX_wrapper.h"
/* configuration */
#include "config.h"

#define FREQ_HZ 20
#define PING_TRESHOLD_MS 500

#define QUADRATURE_A_PIN 22
#define QUADRATURE_B_PIN 21

#define ENC_BTN 20

uint32_t tLastEncBtnPress = 0;
bool encBtnPressed = false;

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
struct bno055_all_float_t data;

typedef enum
{
    SET_NONE,
    SET_SF,
    SET_BW,
    SET_CR,
} setting_t;

typedef enum
{
    SF_5 = 5,
    SF_6,
    SF_7,
    SF_8,
    SF_9,
    SF_10,
    SF_11,
    SF_12,
} lora_sf_t;

typedef enum
{
    BW_200,
    BW_400,
    BW_800,
    BW_1600,
} lora_bw_t;

typedef enum
{
    CR_4_5,
    CR_4_6,
    CR_4_7,
    CR_4_8,
    CR_LI_4_5,
    CR_LI_4_6,
    CR_LI_4_8,
} lora_cr_t;

typedef enum
{
    NONE,
    LEFT,
    RIGHT
} direction_t;

s8 BNO055_I2C_bus_read(u8 dev_addr, u8 reg_addr, u8 *reg_data, u8 cnt);
s8 BNO055_I2C_bus_write(u8 dev_addr, u8 reg_addr, u8 *reg_data, u8 cnt);
void BNO055_delay_msek(u32 msek);
bool reserved_addr(uint8_t addr);
void fail();
void initSensors();
void meassure();

void encoder_irq_handler(uint gpio, uint32_t events)
{
    if ((millis() - tLastEncBtnPress) < 200)
    {
        return;
    }
    tLastEncBtnPress = millis();
    encBtnPressed = true;
}

int main()
{
    // Initialize the Pico SDK
    stdio_init_all();
    sleep_ms(1000);

    /* I2C setup and sensor init*/
    i2c_init(BNO_I2C, 100000);
    gpio_set_function(BNO_SCK_PIN, GPIO_FUNC_I2C);
    gpio_set_function(BNO_SDA_PIN, GPIO_FUNC_I2C);
    gpio_pull_up(BNO_SCK_PIN);
    gpio_pull_up(BNO_SDA_PIN);
    initSensors();

    /* SPI setup */
    spi_init(spi0, (uint)5e6);
    gpio_set_function(2, GPIO_FUNC_SPI);
    gpio_set_function(3, GPIO_FUNC_SPI);
    gpio_set_function(4, GPIO_FUNC_SPI);

    /* CSs GPIO setup */
    gpio_init(17);
    gpio_set_dir(17, 1);
    gpio_put(17, 1);

    gpio_init(5);
    gpio_set_dir(5, 1);
    gpio_put(5, 1);

    /* Lora init */
    sx1280_spi_t dev = {
        .spi = spi0,
        .misoPin = 4,
        .mosiPin = 3,
        .sckPin = 2,
        .csPin = 5,
        .busyPin = 6,
        .resetPin = 1,
        .dio1Pin = 7,
        .dio2Pin = 8,
        .dio3Pin = 9,
        .state = STANDBY_RC};
    Sx1280SPIInit(&dev);
    WaitForSetup(&dev);

    /* Encoder setup */
    PIO encoderPIO = pio0;
    uint offsetA = pio_add_program(encoderPIO, &quadratureA_program);
    uint smA = pio_claim_unused_sm(encoderPIO, true);
    uint offsetB = pio_add_program(encoderPIO, &quadratureB_program);
    uint smB = pio_claim_unused_sm(encoderPIO, true);
    quadratureA_program_init(encoderPIO, smA, offsetA, QUADRATURE_A_PIN, QUADRATURE_B_PIN);
    quadratureB_program_init(encoderPIO, smB, offsetB, QUADRATURE_A_PIN, QUADRATURE_B_PIN);
    gpio_init(ENC_BTN);
    gpio_set_dir(ENC_BTN, GPIO_IN);
    gpio_set_irq_enabled_with_callback(ENC_BTN, GPIO_IRQ_EDGE_RISE, true, &encoder_irq_handler);

    /* OLED init */
    initGFX(spi0);

    /* variable definition */
    msg_t msg = {0};
    uint16_t irq = 0;
    uint32_t tRefresh = 0;
    uint32_t tLastSend = 0;
    uint8_t rxBuffStatus[2];
    int32_t enc;

    setting_t target = SET_NONE;
    lora_sf_t sf = SF_7;
    lora_bw_t bw = BW_1600;
    lora_cr_t cr = CR_4_5;

    bool redraw = true;
    int freq = 0;
    direction_t dir = NONE;

    char str[64];

    for (;;)
    {
        pio_sm_exec_wait_blocking(encoderPIO, smA, pio_encode_in(pio_x, 32));
        pio_sm_exec_wait_blocking(encoderPIO, smB, pio_encode_in(pio_x, 32));
        enc = pio_sm_get_blocking(encoderPIO, smA) + pio_sm_get_blocking(encoderPIO, smB);

        if (enc >= 4)
        {
            dir = LEFT;
            pio_sm_exec(encoderPIO, smA, pio_encode_set(pio_x, 0));
            pio_sm_exec(encoderPIO, smB, pio_encode_set(pio_x, 0));
        }
        else if (enc <= -4)
        {
            dir = RIGHT;
            pio_sm_exec(encoderPIO, smA, pio_encode_set(pio_x, 0));
            pio_sm_exec(encoderPIO, smB, pio_encode_set(pio_x, 0));
        }

        enc = 0;

        switch (dir)
        {
        case NONE:
            break;
        case LEFT:
            switch (target)
            {
            case SET_NONE:
                break;
            case SET_SF:
                sf = sf > SF_5 ? sf - 1 : SF_5;
                redraw = true;
                break;
            case SET_BW:
                bw = bw > BW_200 ? bw - 1 : BW_200;
                redraw = true;
                break;
            case SET_CR:
                cr = cr > CR_4_5 ? cr - 1 : CR_4_5;
                redraw = true;
                break;
            default:
                break;
            }
            break;
        case RIGHT:
            switch (target)
            {
            case SET_NONE:
                break;
            case SET_SF:
                sf = sf < SF_12 ? sf + 1 : SF_12;
                redraw = true;
                break;
            case SET_BW:
                bw = bw < BW_1600 ? bw + 1 : BW_1600;
                redraw = true;
                break;
            case SET_CR:
                cr = cr < CR_LI_4_8 ? cr + 1 : CR_LI_4_8;
                redraw = true;
                break;
            default:
                break;
            }
            break;
        default:
            break;
        }
        dir = NONE;

        if (encBtnPressed)
        {
            target = target < SET_CR ? target + 1 : SET_NONE;
            if (target == SET_NONE)
            {
                switch (sf)
                {
                case SF_5:
                    modParams[0] = 0x50;
                    break;
                case SF_6:
                    modParams[0] = 0x60;
                    break;
                case SF_7:
                    modParams[0] = 0x70;
                    break;
                case SF_8:
                    modParams[0] = 0x80;
                    break;
                case SF_9:
                    modParams[0] = 0x90;
                    break;
                case SF_10:
                    modParams[0] = 0xA0;
                    break;
                case SF_11:
                    modParams[0] = 0xB0;
                    break;
                case SF_12:
                    modParams[0] = 0xC0;
                    break;
                default:
                    break;
                }

                switch (bw)
                {
                case BW_200:
                    modParams[1] = 0x34;
                    break;
                case BW_400:
                    modParams[1] = 0x26;
                    break;
                case BW_800:
                    modParams[1] = 0x18;
                    break;
                case BW_1600:
                    modParams[1] = 0x0A;
                    break;
                default:
                    break;
                }

                switch (cr)
                {
                case CR_4_5:
                    modParams[2] = 0x01;
                    break;
                case CR_4_6:
                    modParams[2] = 0x02;
                    break;
                case CR_4_7:
                    modParams[2] = 0x03;
                    break;
                case CR_4_8:
                    modParams[2] = 0x04;
                    break;
                case CR_LI_4_5:
                    modParams[2] = 0x05;
                    break;
                case CR_LI_4_6:
                    modParams[2] = 0x06;
                    break;
                case CR_LI_4_8:
                    modParams[2] = 0x07;
                    break;
                default:
                    break;
                }
                WaitForSetup(&dev);
                // printf("Modulation params set to: SF: %d, BW: %d, CR: %d\n", sf, bw, cr);
            }
            encBtnPressed = false;
            redraw = true;
        }

        if (redraw || (millis() - tRefresh) > 1000 / FREQ_HZ)
        {
            clear();
            sprintf(str, "acc: %5.1f, %5.1f, %5.1f", data.accel.x, data.accel.y, data.accel.z);
            drawCharArray(0, 5, str, WHITE);
            sprintf(str, "gyr: %5.1f, %5.1f, %5.1f", data.gyro.x, data.gyro.y, data.gyro.z);
            drawCharArray(0, 15, str, WHITE);
            sprintf(str, "bmp: %5.1f, %5.1f", bmpData.temperature, bmpData.pressure);
            drawCharArray(0, 25, str, WHITE);

            switch (target)
            {
            case SET_NONE:
                break;
            case SET_SF:
                drawFillRectangle(0, 34, 26, 7, WHITE);
                break;
            case SET_BW:
                drawFillRectangle(34, 34, 38, 7, WHITE);
                break;
            case SET_CR:
                drawFillRectangle(79, 34, 42, 7, WHITE);
                break;
            default:
                break;
            }

            sprintf(str, "SF: %2d", sf);
            drawCharArray(1, 35, str, INVERSE);

            switch (bw)
            {
            case BW_200:
                drawCharArray(35, 35, "BW: 200 ", INVERSE);
                break;
            case BW_400:
                drawCharArray(35, 35, "BW: 400 ", INVERSE);
                break;
            case BW_800:
                drawCharArray(35, 35, "BW: 800 ", INVERSE);
                break;
            case BW_1600:
                drawCharArray(35, 35, "BW: 1600", INVERSE);
                break;
            default:
                break;
            }

            switch (cr)
            {
            case CR_4_5:
                drawCharArray(80, 35, " CR: 4/5 ", INVERSE);
                break;
            case CR_4_6:
                drawCharArray(80, 35, " CR: 4/6 ", INVERSE);
                break;
            case CR_4_7:
                drawCharArray(80, 35, " CR: 4/7 ", INVERSE);
                break;
            case CR_4_8:
                drawCharArray(80, 35, " CR: 4/8 ", INVERSE);
                break;
            case CR_LI_4_5:
                drawCharArray(80, 35, "CR: 4/5 x", INVERSE);
                break;
            case CR_LI_4_6:
                drawCharArray(80, 35, "CR: 4/6 x", INVERSE);
                break;
            case CR_LI_4_8:
                drawCharArray(80, 35, "CR: 4/8 x", INVERSE);
                break;
            default:
                break;
            }

            sprintf(str, "f %2d Hz", freq);
            drawCharArray(0, 45, str, WHITE);

            display();
            tRefresh = millis();
            redraw = false;
        }

        /* handle lora irq */
        irq = 0;
        if (GetIrqStatus(&dev, &irq) == -1)
        {
            WaitForSetup(&dev);
        }

        /* handle lora dio pins */
        if (irq & 0b0000000000000001) // TxDone
        {
            if (irq & 0b0100000001100000) // Error
            {
                perror("TX error");
            }
            else
            {
                // printf("Tx done\n");
                if (ClrIrqStatus(&dev, 0xFFFF) == -1)
                {
                    WaitForSetup(&dev);
                }
                irq = 0;
                if (SetRx(&dev, 0x02, 0xFFFF) == -1)
                {
                    WaitForSetup(&dev);
                }
                dev.state = RX;
                freq = (int)(1000 / (millis() - tLastSend));
                tLastSend = millis();
            }
        }

        /* Reset dio pins */
        if (irq)
        {
            if (ClrIrqStatus(&dev, 0xFFFF) == -1)
            {
                WaitForSetup(&dev);
            }
        }

        /* Data periodic handle */
        if (dev.state == RX || dev.state == STANDBY_RC)
        {
            meassure();
            memcpy(&msg.accel, &data.accel, sizeof(data.accel));
            memcpy(&msg.gyro, &data.gyro, sizeof(data.gyro));
            memcpy(&msg.bmp, &bmpData, sizeof(bmpData));

            if (WriteBuffer(&dev, (uint8_t *)&msg, sizeof(msg)) == -1)
            {
                WaitForSetup(&dev);
            }

            if (ClrIrqStatus(&dev, 0xFFFF) == -1)
            {
                WaitForSetup(&dev);
            }

            if (SetTx(&dev, 0x02, 0) == -1)
            {
                WaitForSetup(&dev);
            }
            dev.state = TX;
        }

        // sleep_ms(50);
    }
}

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

void fail()
{
    while (1)
    {
        sleep_ms(1000);
        printf("Fail\n");
    }
}

bool reserved_addr(uint8_t addr)
{
    return (addr & 0x78) == 0 || (addr & 0x78) == 0x78;
}

void scanIIC()
{
    printf("Scanning I2C on pins SCL: %d, SDA: %d\n", BNO_SCK_PIN, BNO_SDA_PIN);
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
}

void initSensors()
{
    /* init BNO055 */
    bno055.bus_write = BNO055_I2C_bus_write;
    bno055.bus_read = BNO055_I2C_bus_read;
    bno055.delay_msec = BNO055_delay_msek;
    bno055.dev_addr = BNO055_I2C_ADDR1;

    int ret;
    ret = bno055_init(&bno055);
    if (ret != BNO055_SUCCESS)
    {
        printf("BNO init failed\n");
        fail();
    }
    ret = bno055_set_power_mode(BNO055_POWER_MODE_NORMAL);
    if (ret != BNO055_SUCCESS)
        fail();
    // Set operation mode to config
    ret = bno055_set_operation_mode(BNO055_OPERATION_MODE_CONFIG);
    if (ret != BNO055_SUCCESS)
        fail();
    // Set accelerometer power mode to normal
    ret = bno055_set_accel_power_mode(BNO055_ACCEL_NORMAL);
    if (ret != BNO055_SUCCESS)
        fail();
    // Set gyro power mode to normal
    ret = bno055_set_gyro_power_mode(BNO055_GYRO_POWER_MODE_NORMAL);
    if (ret != BNO055_SUCCESS)
        fail();
    // Set magnetometer power mode to normal
    ret = bno055_set_mag_power_mode(BNO055_MAG_POWER_MODE_NORMAL);
    if (ret != BNO055_SUCCESS)
        fail();
    // Set operation mode to NDOF
    ret = bno055_set_operation_mode(BNO055_OPERATION_MODE_NDOF);
    if (ret != BNO055_SUCCESS)
        fail();

    /* init BMP */
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
}

void meassure()
{
    int8_t rslt = bmp3_get_status(&bmpStatus, &bmp388);
    bmp3_check_rslt("bmp3_get_status", rslt);

    if ((rslt == BMP3_OK) && (bmpStatus.intr.drdy == BMP3_ENABLE))
    {
        rslt = bmp3_get_sensor_data(BMP3_PRESS_TEMP, &bmpData, &bmp388);
        bmp3_check_rslt("bmp3_get_sensor_data", rslt);
        rslt = bmp3_get_status(&bmpStatus, &bmp388);
        bmp3_check_rslt("bmp3_get_status", rslt);
    }

    if (bno055_convert_float_accel_xyz_msq(&data.accel) == BNO055_ERROR)
        fail();
    if (bno055_convert_float_gyro_xyz_dps(&data.gyro) == BNO055_ERROR)
        fail();
}