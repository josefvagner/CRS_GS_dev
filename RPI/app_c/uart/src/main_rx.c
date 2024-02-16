#include <stdio.h>
#include <stdlib.h>
#include "sx1280_uart.h"
#include "cJSON.h"
#include <string.h>
#include <sys/socket.h>
#include <netinet/in.h>
#include <arpa/inet.h>

#define SERVER_IP "gspi.local"
#define SERVER_PORT 8000
#define ENDPOINT "/json_update"

/*
bool rxDone = false;
bool rxError = false;

void dio1Callback()
{
    printf("rxDone\n");
    rxDone = true;
}

void dio2Callback()
{
    printf("rxError\n");
    rxError = true;
}
*/

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
    bool sendLocal = true;
    int sockfd = socket(AF_INET, SOCK_STREAM, 0);

    if (sockfd < 0)
    {
        sendLocal = false;
        perror("socket creation failed");
    }

    // Connect to Python server
    struct sockaddr_in server_addr;
    server_addr.sin_family = AF_INET;
    server_addr.sin_port = htons(8000);                 // Adjust port number as needed
    server_addr.sin_addr.s_addr = inet_addr("0.0.0.0"); // Replace with Python server's IP
    if (connect(sockfd, (struct sockaddr *)&server_addr, sizeof(server_addr)) < 0)
    {
        sendLocal = false;
        perror("connection failed");
    }

    printf("starting....RX\n");
    sx1280UartInit();
    printf("1\n");
    waitBusyPin();
    SetStandby(0x00);
    printf("2\n");
    SetPacketType(0x01);
    uint8_t rfFreq[3] = {0xB8, 0x9D, 0x89};
    SetRfFrequency(rfFreq);
    printf("3\n");
    SetBufferBaseAddress(TX_BASE_ADDR, RX_BASE_ADDR);
    printf("4\n");
    uint8_t modParams[3] = {0x50, 0x0A, 0x01};
    SetModulationParams(modParams);
    printf("5\n");
    uint8_t packetParams[7] = {
        DF_PREAMBLE_LENGTH,
        DF_HEADER_TYPE,
        (uint8_t)sizeof(GsMsg_t),
        DF_CYCLICAL_REDUNDANCY_CHECK,
        DF_CHIRP_INVERT,
        0x00,
        0x00};
    SetPacketParams(packetParams);
    printf("6\n");
    SetRxDutyCycle(0x03, 0, 0x00FA);
    SetDioIrqParams(0b0100000001100010, 0, 0, 0);
    SetRx(0x02, 0xFFFF);

    uint8_t pt = GetPacketType();
    printf("packet type %u\n", pt);
    ClrIrqStatus(0xFFFF);

    uint8_t rxBuffStatus[2];
    uint8_t msgRaw[256];
    GsMsg_t msg;

    while (true)
    {
        uint16_t irq = GetIrqStatus();

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
                GetRxBufferStatus(rxBuffStatus);
                ReadBuffer(msgRaw, (size_t)rxBuffStatus[0], rxBuffStatus[1]);
                myMemcpy(&msg, msgRaw, (size_t)rxBuffStatus[0]);
                printf("New msg [%d] [%d]: fsw_state = %d\n", (int)rxBuffStatus[0], (int)rxBuffStatus[1], msg.fsw_state);
                char *json_string = convert_to_json(&msg);
                if (sendLocal)
                {
                    char request[strlen(json_string) + 150];
                    sprintf(request, "POST /json_update HTTP/1.1\r\n"
                                     "Host: 0.0.0.0:8000\r\n"
                                     "Content-Type: application/json\r\n"
                                     "Content-Length: %ld\r\n\r\n"
                                     "%s",
                            strlen(json_string), json_string);
                    int sent_bytes = send(sockfd, request, strlen(request), 0);
                    if (sent_bytes < 0)
                    {
                        perror("send failed");
                    }
                }
                free(json_string);
            }
            ClrIrqStatus(0xFFFF);
        }
    }
    return 0;
}
