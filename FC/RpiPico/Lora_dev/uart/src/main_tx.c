#include <stdio.h>
#include "pico/stdlib.h"
#include "hardware/uart.h"
#include "sx1280_uart.h"

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
    GsMsg_t msg = {
        200, 100.2, 100.3, 100.3, 100.5, 234.4, 1364.545044, 1364.564087, 98121.734375, 175.699997, 7.784000, -11.128282, 23.315508, 19.417095, -0.205971, -9.958491, 0.253872, -2.288818, -0.427246, 0.732421, 0};

    stdio_init_all();
    sleep_ms(1000);
    sx1280UartInit();
    SetStandby(0x00);
    SetPacketType(0x01);
    uint8_t rfFreq[3] = {0xB8, 0x9D, 0x89};
    SetRfFrequency(rfFreq);
    SetBufferBaseAddress(TX_BASE_ADDR, RX_BASE_ADDR);
    uint8_t modParams[3] = {0x50, 0x0A, 0x01};
    SetModulationParams(modParams);
    uint8_t packetParams[7] = {
        DF_PREAMBLE_LENGTH,
        DF_HEADER_TYPE,
        (uint8_t)sizeof(msg),
        DF_CYCLICAL_REDUNDANCY_CHECK,
        DF_CHIRP_INVERT,
        0x00,
        0x00};
    SetPacketParams(packetParams);
    SetDioIrqParams(0b0100000000000001, 0b0000000000000001, 0b0100000000000000, 0);
    SetTxParams(0x1F, 0x00);

    uint16_t irq = 0;

    uint8_t pt = GetPacketType();
    printf("packet type %u\n", pt);
    printf("msg size: %d\n", sizeof(msg));

    uint8_t buff[sizeof(msg)] = {0};
    uint64_t t0 = time_us_64();
    uint64_t tnow = time_us_64();

    while (true)
    {
        msg.fsw_state = msg.fsw_state >= 9 ? 0 : msg.fsw_state + 1;
        myMemcpy(buff, &msg, sizeof(msg));
        WriteBuffer(buff, sizeof(buff));
        SetTx(0x02, 0);
        while (true)
        {
            irq = GetIrqStatus();
            if (irq & 0x01)
            {
                tnow = time_us_64();
                printf("Tx done: fsw_state = %d, time = %d\n", msg.fsw_state, tnow - t0);
                t0 = tnow;
                break;
            }
            if (irq & 0b0100000000000000)
            {
                printf("Tx timeout...\n");
                break;
            }
        }
        ClrIrqStatus(0xFFFF);
    }
    printf("end\n");
}
