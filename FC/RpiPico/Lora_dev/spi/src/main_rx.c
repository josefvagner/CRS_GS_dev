#include <stdio.h>
#include "pico/stdlib.h"
#include "hardware/uart.h"
#include "sx1280_spi.h"

typedef struct
{
    int gps_sat;
    float gps_lon;
    float gps_lat;
    float gps_alt;
    float velocity;
    float rel_alti;
    float in_timestamp;
    float out_timestamp;
    float bmp_pres;
    float ina_curr;
    float ina_volt;
    float mpu_mag_x;
    float mpu_mag_y;
    float mpu_mag_z;
    float mpu_accel_x;
    float mpu_accel_y;
    float mpu_accel_z;
    float mpu_gyro_x;
    float mpu_gyro_y;
    float mpu_gyro_z;
    uint8_t fsw_state;
} GsMsg_t;

int main()
{
    sleep_ms(2000);
    stdio_init_all();
    sleep_ms(1000);
    printf("starting....RX\n");
    sx1280_spi_t dev = {spi0, 19, 16, 18, 17, 2, 1};
    sx1280SPIInit(&dev);
    waitBusyPin(&dev);
    SetStandby(&dev, 0x00);
    SetPacketType(&dev, 0x01);
    uint8_t rfFreq[3] = {0xB8, 0x9D, 0x89};
    SetRfFrequency(&dev, rfFreq);
    SetBufferBaseAddress(&dev, TX_BASE_ADDR, RX_BASE_ADDR);
    uint8_t modParams[3] = {0x50, 0x0A, 0x01};
    SetModulationParams(&dev, modParams);
    uint8_t packetParams[7] = {
        DF_PREAMBLE_LENGTH,
        DF_HEADER_TYPE,
        DF_PACKET_LEN,
        DF_CYCLICAL_REDUNDANCY_CHECK,
        DF_CHIRP_INVERT,
        0x00,
        0x00};
    SetPacketParams(&dev, packetParams);

    SetDioIrqParams(&dev, 0b0100000001100010, 0, 0, 0);
    SetRx(&dev, 0x02, 0xFFFF);

    uint8_t rxBuffStatus[2];

    uint8_t pt = GetPacketType(&dev);
    printf("packet type %u\n", pt);

    while (true)
    {
        uint8_t irq = GetIrqStatus(&dev);

        if (irq & 0x02)
        {
            if (irq & 0b1000000)
            {
                printf("Crc error...\n");
            }
            else if (irq & 0b100000)
            {
                printf("Header error...\n");
            }
            else if (irq & 0b100000000000000)
            {
                printf("Timeout...\n");
            }
            else
            {
                GetRxBufferStatus(&dev, rxBuffStatus);
                uint8_t msg[rxBuffStatus[0]];
                ReadBuffer(&dev, msg, (size_t)rxBuffStatus[0], rxBuffStatus[1]);
                GsMsg_t *data = (GsMsg_t *)msg;
                printf("New msg [%d]: fsw_state = %d\n", (int)rxBuffStatus[0], (int)data->fsw_state);
            }
            ClrIrqStatus(&dev, 0xFFFF);
        }
    }
}
