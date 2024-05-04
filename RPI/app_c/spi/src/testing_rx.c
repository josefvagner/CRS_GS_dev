#include <stdio.h>
#include <stdlib.h>
#include "sx1280_spi.h"
#include "cJSON.h"
#include <string.h>
#include <time.h>

#define PING_FREQ_HZ 10

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
    sx1280_spi_t dev = {-1, -1, 0, 9, 10, 11, 22, 17, 27, 23, 24, 25, STANDBY_RC};

    Sx1280SPIInit(&dev);
    printf("init correct spi: %d\n", dev.spi);
    WaitForSetup(&dev);
    printf("setup correct\n");
    // SetRx(&dev, 0x02, 0xFFFF);
    // printf("set rx correct\n");

    uint8_t rxBuffStatus[2];
    GsMsg_t msg;
    GsPingMsg_t ping = {0};

    long long tLastPing = 0;
    long long tStart = millis();
    long long tLastMsg = millis();

    int lastFswState = -1;
    long missedPackets = 0;
    long allPackets = 0;

    int dio1 = 0;
    int dio2 = 0;
    int dio3 = 0;

    while ((millis() - tStart) <= 10 * 1e3)
    {
        dio1 = gpio_read(dev.pi, dev.dio1Pin);
        dio2 = gpio_read(dev.pi, dev.dio2Pin);
        dio3 = gpio_read(dev.pi, dev.dio3Pin);

        if (dio1) // TxDone
        {
            if (dio3) // Error
            {
                perror("TX error");
            }
            else
            {
                printf("Ping done: idx = %d\n", ping.idx);

                if (ClrIrqStatus(&dev, 0xFFFF) == -1)
                {
                    WaitForSetup(&dev);
                }

                if (SetRx(&dev, 0x02, 0xFFFF) == -1)
                {
                    WaitForSetup(&dev);
                }
                dev.state = RX;
            }
        }
        if (dio2) // RxDone
        {
            allPackets += 1;
            if (dio3) // Error
            {
                missedPackets += 1;
                perror("RX error");
            }
            else
            {
                if (GetRxBufferStatus(&dev, rxBuffStatus) == -1)
                {
                    WaitForSetup(&dev);
                }
                if (ReadBuffer(&dev, (uint8_t *)&msg, (size_t)rxBuffStatus[0], rxBuffStatus[1]) == -1)
                {
                    WaitForSetup(&dev);
                }
                printf("New msg [%d] [%d]: fsw_state = %d , time = %d\n", (int)rxBuffStatus[0], (int)rxBuffStatus[1], msg.fsw_state, millis() - tLastMsg);
                tLastMsg = millis();
                // char *json_string = convert_to_json(&msg);
                // sendToApi(json_string);
                // free(json_string);
            }
        }

        if ((millis() - tLastPing) >= (long long)(1000 / PING_FREQ_HZ))
        {
            ping.idx = ping.idx > 100 ? 0 : ping.idx + 1;
            if (WriteBuffer(&dev, (uint8_t *)&ping, sizeof(ping)) == -1)
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
            tLastPing = millis();
        }

        if (dio1 || dio2 || dio3)
        {
            if (ClrIrqStatus(&dev, 0xFFFF) == -1)
            {
                WaitForSetup(&dev);
            }
        }
    }
    printf("All recieved packets = %d, Missed packets = %d, Packet lost = %.2f %, Freq = %.2f Hz\n", allPackets, missedPackets, ((float)missedPackets / (float)allPackets), (float)(allPackets / 10.0));

    return 0;
}