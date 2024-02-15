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

#define nn 10

int main()
{
    /*
    GsMsg_t msg = {
        200, 100.2, 100.3, 100.3, 100.5, 234.4, 1364.545044, 1364.564087, 98121.734375, 175.699997, 7.784000, -11.128282, 23.315508, 19.417095, -0.205971, -9.958491, 0.253872, -2.288818, -0.427246, 0.732421, 0};
    */

    GsMsg_t msg = {0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0};

    sleep_ms(2000);
    stdio_init_all();
    sleep_ms(1000);
    printf("starting....TX\n");
    sx1280UartInit();
    printf("1\n");
    waitBusyPin();
    printf("2\n");
    SetStandby(0x00);
    printf("3\n");
    SetPacketType(0x01);
    printf("4\n");
    uint8_t rfFreq[3] = {0xB8, 0x9D, 0x89};
    SetRfFrequency(rfFreq);
    printf("5\n");
    SetBufferBaseAddress(TX_BASE_ADDR, RX_BASE_ADDR);
    printf("6\n");
    uint8_t modParams[3] = {0x50, 0x0A, 0x01};
    SetModulationParams(modParams);
    printf("7\n");
    uint8_t packetParams[7] = {
        DF_PREAMBLE_LENGTH,
        DF_HEADER_TYPE,
        (uint8_t)sizeof(msg),
        DF_CYCLICAL_REDUNDANCY_CHECK,
        DF_CHIRP_INVERT,
        0x00,
        0x00};
    SetPacketParams(packetParams);

    printf("8\n");

    SetDioIrqParams(0b0100000000000001, 0, 0, 0);
    // SetDioIrqParams(0xFFFF, 0, 0, 0);
    printf("9\n");
    SetTxParams(0x1F, 0xE0);
    printf("10\n");

    uint16_t irq = 0;

    uint8_t pt = GetPacketType();
    printf("packet type %u\n", pt);
    printf("msg size: %d\n", sizeof(msg));

    uint8_t buff[sizeof(msg)] = {0};

    while (true)
    {
        myMemcpy(buff, &msg, sizeof(msg));
        WriteBuffer(buff, sizeof(buff));
        /*
        uint8_t all[256];
        ReadBuffer(all, 256, 0x00);
        printf("Whole buffer: ");
        printBuffDec(all, 256);
        */
        SetTx(0x02, 0);
        while (true)
        {
            irq = GetIrqStatus();
            if (irq & 0x01)
            {
                printf("Tx done: fsw_state = %d\n", msg.fsw_state);
                break;
            }
            if (irq & 0b0100000000000000)
            {
                printf("Tx timeout...\n");
                break;
            }
        }
        ClrIrqStatus(0xFFFF);
        msg.fsw_state = msg.fsw_state >= 9 ? 0 : msg.fsw_state + 1;
    }
    printf("end\n");
}
