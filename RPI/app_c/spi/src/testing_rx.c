#include <stdio.h>
#include <stdlib.h>
#include "sx1280_spi.h"
#include "cJSON.h"
#include <string.h>
#include <time.h>

char *convert_to_json(GsMsg_t *data)
{
    cJSON *root = cJSON_CreateObject();
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
    cJSON_AddNumberToObject(root, "payload_released", data->payload_released);
    cJSON_AddNumberToObject(root, "drogue_released", data->drogue_released);
    cJSON_AddNumberToObject(root, "parachute_released", data->parachute_released);

    char *json_string = cJSON_Print(root); // Convert to string
    cJSON_Delete(root);                    // Free memory

    return json_string;
}

int main()
{
    printf("starting....RX\n");
    sx1280_spi_t dev = {-1, 0, 9, 10, 11, 22, 17, 27};
    sx1280SPIInit(&dev);
    printf("init correct spi: %d\n", dev.spi);
    waitForSetup(&dev);
    printf("setup correct\n");
    SetRx(&dev, 0x02, 0xFFFF);
    printf("set rx correct\n");

    uint8_t rxBuffStatus[2];
    uint8_t msgRaw[256];
    GsMsg_t msg;

    int lastFswState = -1;
    long missedPackets = 0;
    long allPackets = 0;
    time_t tStart = time(NULL);
    uint16_t irq = 0;

    while ((time(NULL) - tStart) <= 10)
    {
        if (GetIrqStatus(&dev, &irq) == -1)
        {
            waitForSetup(&dev);
        }

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
                if (GetRxBufferStatus(&dev, rxBuffStatus) == -1)
                {
                    waitForSetup(&dev);
                }
                if (ReadBuffer(&dev, msgRaw, (size_t)rxBuffStatus[0], rxBuffStatus[1]) == -1)
                {
                    waitForSetup(&dev);
                }
                myMemcpy(&msg, msgRaw, (size_t)rxBuffStatus[0]);
                printf("New msg [%d] [%d]: fsw_state = %d\n", (int)rxBuffStatus[0], (int)rxBuffStatus[1], msg.fsw_state);
                // char *json_string = convert_to_json(&msg);
                // free(json_string);
            }
            if (ClrIrqStatus(&dev, 0xFFFF) == -1)
            {
                waitForSetup(&dev);
            }
        }
    }
    printf("All recieved packets = %d, Missed packets = %d, Packet lost = %.2f %, Freq = %.2f Hz\n", allPackets, missedPackets, ((float)missedPackets / (float)allPackets), (float)(allPackets / 10.0));

    return 0;
}