#include <stdio.h>
#include <stdlib.h>
#include "sx1280_uart.h"
#include "cJSON.h"
#include <string.h>

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

char *convert_to_json(GsMsg_t *data)
{
    cJSON *root = cJSON_CreateObject();
    cJSON_AddNumberToObject(root, "gps_sat", data->gps_sat);
    cJSON_AddNumberToObject(root, "gps_lon", data->gps_lon);
    cJSON_AddNumberToObject(root, "gps_lat", data->gps_lat);
    cJSON_AddNumberToObject(root, "gps_alt", data->gps_alt);
    cJSON_AddNumberToObject(root, "velocity", data->velocity);
    cJSON_AddNumberToObject(root, "rel_alti", data->rel_alti);
    cJSON_AddNumberToObject(root, "in_timestamp", data->in_timestamp);
    cJSON_AddNumberToObject(root, "out_timestamp", data->out_timestamp);
    cJSON_AddNumberToObject(root, "bmp_pres", data->bmp_pres);
    cJSON_AddNumberToObject(root, "ina_Curr", data->ina_curr);
    cJSON_AddNumberToObject(root, "ina_Voltage", data->ina_volt);
    cJSON_AddNumberToObject(root, "mpu_mag_x", data->mpu_mag_x);
    cJSON_AddNumberToObject(root, "mpu_mag_y", data->mpu_mag_y);
    cJSON_AddNumberToObject(root, "mpu_mag_z", data->mpu_mag_z);
    cJSON_AddNumberToObject(root, "mpu_accel_x", data->mpu_accel_x);
    cJSON_AddNumberToObject(root, "mpu_accel_y", data->mpu_accel_y);
    cJSON_AddNumberToObject(root, "mpu_accel_z", data->mpu_accel_z);
    cJSON_AddNumberToObject(root, "mpu_gyro_x", data->mpu_gyro_x);
    cJSON_AddNumberToObject(root, "mpu_gyro_y", data->mpu_gyro_y);
    cJSON_AddNumberToObject(root, "mpu_gyro_z", data->mpu_gyro_z);
    cJSON_AddNumberToObject(root, "fsw_state", data->fsw_state);

    char *json_string = cJSON_Print(root); // Convert to string
    cJSON_Delete(root);                    // Free memory

    return json_string;
}

int main()
{
    stdio_init_all();
    sleep_ms(1000);
    printf("starting....RX\n");
    sx1280UartInit();
    waitBusyPin();
    SetStandby(0x00);
    SetPacketType(0x01);
    uint8_t rfFreq[3] = {RF_FREQ_1, RF_FREQ_2, RF_FREQ_3};
    SetRfFrequency(rfFreq);
    SetBufferBaseAddress(TX_BASE_ADDR, RX_BASE_ADDR);
    uint8_t modParams[3] = {MOD_PARAM_1, MOD_PARAM_2, MOD_PARAM_3};
    SetModulationParams(modParams);
    uint8_t packetParams[7] = {
        DF_PREAMBLE_LENGTH,
        DF_HEADER_TYPE,
        (uint8_t)sizeof(GsMsg_t),
        DF_CYCLICAL_REDUNDANCY_CHECK,
        DF_CHIRP_INVERT,
        0x00,
        0x00};
    SetPacketParams(packetParams);
    SetRxDutyCycle(0x03, 0, 0x00FA);
    SetDioIrqParams(0b0100000001100010, 0, 0, 0);
    SetRx(0x02, 0xFFFF);

    uint8_t pt = GetPacketType();
    printf("packet type %u\n", pt);
    ClrIrqStatus(0xFFFF);

    uint8_t rxBuffStatus[2];
    uint8_t msgRaw[256];
    GsMsg_t msg;

    int lastFswState = -1;
    long missedPackets = 0;
    long allPackets = 0;
    uint64_t tStart = time_us_64();

    while ((time_us_64() - tStart) / 1e6 <= 20)
    {
        uint16_t irq = GetIrqStatus();

        if (irq & 0x02)
        {
            allPackets += 1;
            if (irq & 0b1000000)
            {
                missedPackets += 1;
                printf("Crc error...\n");
            }
            else if (irq & 0b100000)
            {
                missedPackets += 1;
                printf("Header error...\n");
            }
            else if (irq & 0b100000000000000)
            {
                missedPackets += 1;
                printf("Timeout...\n");
            }
            else
            {
                GetRxBufferStatus(rxBuffStatus);
                ReadBuffer(msgRaw, (size_t)rxBuffStatus[0], rxBuffStatus[1]);
                myMemcpy(&msg, msgRaw, (size_t)rxBuffStatus[0]);
                printf("New msg [%d] [%d]: fsw_state = %d\n", (int)rxBuffStatus[0], (int)rxBuffStatus[1], msg.fsw_state);
                // char *json_string = convert_to_json(&msg);
                // free(json_string);
            }
            ClrIrqStatus(0xFFFF);
        }
    }
    printf("All recieved packets = %d, Missed packets = %d, Packet lost = %.2f %, Freq = %.2f Hz\n", allPackets, missedPackets, (float)(missedPackets / allPackets), (float)(allPackets / 20));

    while (true)
    {
        /* code */
    }

    return 0;
}