#include <stdio.h>
#include <stdlib.h>
#include "sx1280_spi.h"
#include "cJSON.h"
#include <string.h>
#include <sys/socket.h>
#include <netinet/in.h>
#include <arpa/inet.h>

#define SERVER_IP "0.0.0.0"
#define SERVER_PORT 8000
#define ENDPOINT "/json_update"

char *convert_to_json(GsMsg_t *data)
{
    cJSON *root = cJSON_CreateObject();
    cJSON_AddNumberToObject(root, "velocity", data->velocity);
    cJSON_AddNumberToObject(root, "rel_alti", data->rel_alti);
    cJSON_AddNumberToObject(root, "in_timestamp", data->in_timestamp);
    cJSON_AddNumberToObject(root, "out_timestamp", data->out_timestamp);
    cJSON_AddNumberToObject(root, "bmp_pres", data->bmp_pres);
    cJSON_AddNumberToObject(root, "bmp_temp", data->bmp_temp);
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

void sendToApi(const char *json)
{
    // printf("Connecting to FastApi\n");
    int sockfd = socket(AF_INET, SOCK_STREAM, 0);

    if (sockfd < 0)
    {
        perror("socket creation failed");
    }
    struct sockaddr_in server_addr;
    server_addr.sin_family = AF_INET;
    server_addr.sin_port = htons(8000);                 // Adjust port number as needed
    server_addr.sin_addr.s_addr = inet_addr("0.0.0.0"); // Replace with Python server's IP
    if (connect(sockfd, (struct sockaddr *)&server_addr, sizeof(server_addr)) < 0)
    {
        perror("connection failed");
        close(sockfd);
    }

    char request[strlen(json) + 150];
    sprintf(request, "POST /json_update HTTP/1.1\r\n"
                     "Host: 0.0.0.0:8000\r\n"
                     "Content-Type: application/json\r\n"
                     "Content-Length: %ld\r\n\r\n"
                     "%s",
            strlen(json), json);
    int sent_bytes = send(sockfd, request, strlen(request), 0);
    if (sent_bytes < 0)
    {
        perror("send failed");
        close(sockfd);
    }
    close(sockfd);
}

int main()
{
    printf("starting....RX\n");
    sx1280_spi_t dev = {-1, 0, 9, 10, 11, 22, 17, 27};
    sx1280SPIInit(&dev);
    waitForSetup(&dev);

    uint8_t rxBuffStatus[2];
    uint8_t msgRaw[256];
    GsMsg_t msg;
    uint16_t irq = 0;

    while (true)
    {
        irq = 0;
        if (GetIrqStatus(&dev, &irq) == -1)
        {
            waitForSetup(&dev);
        }

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
                char *json_string = convert_to_json(&msg);
                sendToApi(json_string);
                free(json_string);
            }
            if (ClrIrqStatus(&dev, 0xFFFF) == -1)
            {
                waitForSetup(&dev);
            }
        }
    }
    return 0;
}
