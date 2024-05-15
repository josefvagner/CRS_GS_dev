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

struct bmp3_data bmpData = {0};
struct bno055_all_float_t
{
    struct bno055_accel_float_t accel; /* Accelerometer data */
    struct bno055_gyro_float_t gyro;   /* Gyroscope data */
};
struct bno055_all_float_t bnoData = {0};

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
    sleep_ms(500);

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
        .state = RX};
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
    spi_init(spi1, (uint)5e6);
    gpio_set_function(26, GPIO_FUNC_SPI);
    gpio_set_function(27, GPIO_FUNC_SPI);

    gpio_init(17);
    gpio_set_dir(17, 1);
    gpio_put(17, 1);

    initGFX(spi1);

    /* variable definition */
    msg_t msg = {0};
    uint16_t irq = 0;
    uint32_t tRefresh = 0;
    uint32_t tLastRecieved = 0;
    uint32_t tPacketLoss = 0;
    uint8_t rxBuffStatus[2];
    int32_t enc;
    uint8_t packetStatus[5] = {0};
    float rssi = 0;
    float snr = 0;

    setting_t target = SET_NONE;
    lora_sf_t sf = SF_7;
    lora_bw_t bw = BW_1600;
    lora_cr_t cr = CR_4_5;

    bool redraw = true;
    int freq = 0;
    direction_t dir = NONE;

    int posMsgs = 0;
    int negMsgs = 0;
    float packetLoss = 0.0;

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
            printf("%f %f %f %f %f %d\n", bnoData.accel.x, bnoData.accel.y, bnoData.accel.z, rssi, snr, freq);
            clear();
            sprintf(str, "acc: %5.1f, %5.1f, %5.1f", bnoData.accel.x, bnoData.accel.y, bnoData.accel.z);
            drawCharArray(0, 5, str, WHITE);
            sprintf(str, "gyr: %5.1f, %5.1f, %5.1f", bnoData.gyro.x, bnoData.gyro.y, bnoData.gyro.z);
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

            sprintf(str, "f %2d Hz, PL %6.2f", freq, packetLoss);
            drawCharArray(0, 45, str, WHITE);
            sprintf(str, "RSSI %6.1f dBm, SNR %5.1f dB", rssi, snr);
            drawCharArray(0, 55, str, WHITE);

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
        if (irq & 0b0000000000000010) // RxDone
        {
            if (irq & 0b0100000001100000) // Error
            {
                negMsgs = negMsgs + 1;
            }
            else
            {
                if (GetRxBufferStatus(&dev, rxBuffStatus) == -1)
                    WaitForSetup(&dev);
                if (ReadBuffer(&dev, (uint8_t *)&msg, (size_t)rxBuffStatus[0], rxBuffStatus[1]) == -1)
                    WaitForSetup(&dev);
                if (GetPacketStatus(&dev, packetStatus) == -1)
                    WaitForSetup(&dev);
                posMsgs = posMsgs + 1;
                rssi = -1 * packetStatus[0] / 2.0;
                snr = packetStatus[1] / 4.0;
                memcpy(&bnoData.accel, &msg.accel, sizeof(bnoData.accel));
                memcpy(&bnoData.gyro, &msg.gyro, sizeof(bnoData.gyro));
                memcpy(&bmpData, &msg.bmp, sizeof(bmpData));
                freq = (int)(1000 / (millis() - tLastRecieved));
                tLastRecieved = millis();
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

        if ((millis() - tPacketLoss) > 1000)
        {
            tPacketLoss = millis();
            packetLoss = 100.0 * negMsgs / (posMsgs + negMsgs);
            negMsgs = 0;
            posMsgs = 0;
        }

        // sleep_ms(50);
    }
}
