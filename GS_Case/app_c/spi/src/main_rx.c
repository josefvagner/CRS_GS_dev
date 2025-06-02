#include <stdio.h>
#include <stdlib.h>
#include <unistd.h>
#include <string.h>
#include <fcntl.h>
#include <errno.h>

#include "sx1280_spi.h"
#include "cJSON.h"

#include <netinet/in.h>
#include <arpa/inet.h>

#include <net/if.h>
#include <sys/ioctl.h>
#include <sys/socket.h>

#include <linux/can.h>
#include <linux/can/raw.h>

#define SERVER_IP "0.0.0.0"
#define SERVER_PORT 8000
#define ENDPOINT "/json_update"

#define PING_FREQ_HZ 10

char *convert_to_json(sd_csv_data_t *data)
{
    cJSON *root = cJSON_CreateObject();
    cJSON *fsw_state = cJSON_CreateObject();
    cJSON *board_status = cJSON_CreateObject();
    cJSON *eject_output = cJSON_CreateObject();
    cJSON *bmp_data = cJSON_CreateObject();
    cJSON *pos_data = cJSON_CreateObject();
    cJSON *bno_data = cJSON_CreateObject();
    cJSON *ina_data = cJSON_CreateObject();

    cJSON_AddNumberToObject(root, "timestamp", data->timestamp);

    cJSON_AddNumberToObject(fsw_state, "state", data->fsw_state.state);

    cJSON_AddNumberToObject(board_status, "pwb_status", data->board_status.pwb_status);
    cJSON_AddNumberToObject(board_status, "cnb_status", data->board_status.cnb_status);
    cJSON_AddNumberToObject(board_status, "snb_status", data->board_status.snb_status);
    cJSON_AddNumberToObject(board_status, "cmb_status", data->board_status.cmb_status);

    cJSON_AddNumberToObject(eject_output, "doors", data->eject_output.doors);
    cJSON_AddNumberToObject(eject_output, "payload", data->eject_output.payload);
    cJSON_AddNumberToObject(eject_output, "drogue", data->eject_output.drogue);
    cJSON_AddNumberToObject(eject_output, "parachute", data->eject_output.parachute);

    cJSON_AddNumberToObject(bmp_data, "pressure", data->bmp_data.pressure);
    cJSON_AddNumberToObject(bmp_data, "temperature", data->bmp_data.temperature);

    cJSON_AddNumberToObject(pos_data, "abs_altitude", data->pos_data.abs_altitude);
    cJSON_AddNumberToObject(pos_data, "rel_altitude", data->pos_data.rel_altitude);
    cJSON_AddNumberToObject(pos_data, "est_velocity", data->pos_data.est_velocity);
    cJSON_AddNumberToObject(pos_data, "est_acceleration", data->pos_data.est_acceleration);

    cJSON_AddNumberToObject(bno_data, "acceleration", data->bno_data.acceleration);
    cJSON_AddNumberToObject(bno_data, "rotation", data->bno_data.rotation);
    cJSON_AddNumberToObject(bno_data, "euler_h", data->bno_data.euler_h);
    cJSON_AddNumberToObject(bno_data, "euler_p", data->bno_data.euler_p);
    cJSON_AddNumberToObject(bno_data, "euler_r", data->bno_data.euler_r);

    cJSON_AddNumberToObject(ina_data, "voltage", data->ina_data.voltage);
    cJSON_AddNumberToObject(ina_data, "current", data->ina_data.current);

    cJSON_AddItemToObject(root, "fsw_state", fsw_state);
    cJSON_AddItemToObject(root, "board_status", board_status);
    cJSON_AddItemToObject(root, "eject_output", eject_output);
    cJSON_AddItemToObject(root, "bmp_data", bmp_data);
    cJSON_AddItemToObject(root, "pos_data", pos_data);
    cJSON_AddItemToObject(root, "bno_data", bno_data);
    cJSON_AddItemToObject(root, "ina_data", ina_data);

    char *json_string = cJSON_Print(root);
    cJSON_Delete(root);

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
    server_addr.sin_port = htons(SERVER_PORT);          // Adjust port number as needed
    server_addr.sin_addr.s_addr = inet_addr(SERVER_IP); // Replace with Python server's IP
    if (connect(sockfd, (struct sockaddr *)&server_addr, sizeof(server_addr)) < 0)
    {
        perror("connection failed");
        close(sockfd);
    }

    char request[strlen(json) + 150];
    sprintf(request, "POST /json_update HTTP/1.1\r\n"
                     "Host: %s:%d\r\n"
                     "Content-Type: application/json\r\n"
                     "Content-Length: %ld\r\n\r\n"
                     "%s",
            SERVER_IP, SERVER_PORT, strlen(json), json);
    int sent_bytes = send(sockfd, request, strlen(request), 0);
    if (sent_bytes < 0)
    {
        perror("send failed");
        close(sockfd);
    }
    close(sockfd);
}

bool sendMsg(sx1280_spi_t *dev, GsPingMsg_t *msg)
{
    int dio1, dio3;
    bool ret = false;

    if (WriteBuffer(dev, (uint8_t *)msg, sizeof(*msg)) == -1)
    {
        WaitForSetup(dev);
    }

    if (ClrIrqStatus(dev, 0xFFFF) == -1)
    {
        WaitForSetup(dev);
    }

    if (SetTx(dev, 0x02, 0) == -1)
    {
        WaitForSetup(dev);
    }
    dev->state = TX;

    while (true)
    {
        dio1 = gpio_read(dev->pi, dev->dio1Pin);
        dio3 = gpio_read(dev->pi, dev->dio3Pin);
        if (dio3) // Error
        {
            uint16_t irq = 0;
            if (GetIrqStatus(dev, &irq) == -1)
            {
                WaitForSetup(dev);
            }
            printf("Error in tx mode irq = %b\n", irq);
            ret = false;
            break;
        }
        if (dio1) // TxDone
        {
            ret = true;
            break;
        }
        // usleep(100);
    }

    if (ClrIrqStatus(dev, 0xFFFF) == -1)
    {
        WaitForSetup(dev);
    }
    if (SetRx(dev, 0x02, 0xFFFF) == -1)
    {
        WaitForSetup(dev);
    }
    dev->state = RX;
    return ret;
}

int main()
{
    printf("starting....RX\n");

    sx1280_spi_t dev = {
        .pi = -1,
        .spi = -1,
        .spiChen = 1,
        .misoPin = 19,
        .mosiPin = 20,
        .sckPin = 21,
        .csPin = 18,
        .busyPin = 12,
        .resetPin = 6,
        .dio1Pin = 13,
        .dio2Pin = 16,
        .dio3Pin = 14,
        .state = STANDBY_RC,
    };
    Sx1280SPIInit(&dev);
    WaitForSetup(&dev);

    uint8_t rxBuffStatus[2];
    sd_csv_data_t msg;
    GsPingMsg_t ping = {0};

    long long tLastPing = 0;
    long long tLastMsg = millis();
    long long tNow = millis();
    long long msgPeriod = (long long)(1000 / PING_FREQ_HZ);

    int dio1 = 0;
    int dio2 = 0;
    int dio3 = 0;

    uint16_t irq = 0;

    printf("starting\n");

    while (true)
    {
        tNow = millis();
        dio1 = gpio_read(dev.pi, dev.dio1Pin);
        dio2 = gpio_read(dev.pi, dev.dio2Pin);
        dio3 = gpio_read(dev.pi, dev.dio3Pin);

        if (dev.state == RX && (dio2 || dio3))
        {
            if (dio3)
            {
                irq = 0;
                if (GetIrqStatus(&dev, &irq) == -1)
                {
                    WaitForSetup(&dev);
                }
                printf("Error: irq = %b\n", irq);
            }
            else if (dio2) // RxDone
            {
                if (GetRxBufferStatus(&dev, rxBuffStatus) == -1)
                {
                    WaitForSetup(&dev);
                }
                if (ReadBuffer(&dev, (uint8_t *)&msg, (size_t)rxBuffStatus[0], rxBuffStatus[1]) == -1)
                {
                    WaitForSetup(&dev);
                }
                printf("New msg: fsw_state = %d \t freq = %d Hz\n", msg.fsw_state.state, (int)(1000 / (tNow - tLastMsg)));
                if (msg.fsw_state.state < 10)
                {
                    char *json_string = convert_to_json(&msg);
                    sendToApi(json_string);
                    free(json_string);
                }
                tLastMsg = tNow;
            }
            if (ClrIrqStatus(&dev, 0xFFFF) == -1)
            {
                WaitForSetup(&dev);
            }
        }

        if ((tNow - tLastPing) >= msgPeriod)
        {
            ping.idx = ping.idx > 100 ? 0 : ping.idx + 1;
            if (sendMsg(&dev, &ping))
            {
                // printf("Ping sended idx = %d, freq = %d\n", ping.idx, (int)(1000 / (millis() - tLastPing)));
                tLastPing = tNow;
            }
        }

        if (dio1 || dio2 || dio3)
        {
            if (ClrIrqStatus(&dev, 0xFFFF) == -1)
            {
                WaitForSetup(&dev);
            }
        }
        usleep(100);
    }
    return 0;
}
