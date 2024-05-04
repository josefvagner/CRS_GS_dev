#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include <unistd.h>
#include <pthread.h>
#include <signal.h>
#include <net/if.h>
#include <sys/types.h>
#include <sys/socket.h>
#include <sys/ioctl.h>
#include <fcntl.h>
#include <linux/can.h>
#include <linux/can/raw.h>
#include <netinet/in.h>
#include <arpa/inet.h>

#include "sx1280_spi.h"
#include "cJSON.h"
#include "can_message.h"

#define SERVER_IP "192.168.88.251"
#define SERVER_PORT 8000
#define ENDPOINT "/json_update"

#define PING_FREQ_HZ 10

typedef struct
{
    struct can_frame frame;
    pthread_mutex_t lock;
    int updated; // Flag to indicate new data
} can_shared_data;

typedef struct
{
    GsMsg_t msg;
    pthread_mutex_t lock;
    int updated; // Flag to indicate new data
} lora_shared_data;

can_shared_data recv_data;
can_shared_data send_data;
lora_shared_data lora_data;
volatile sig_atomic_t run = 1; // Global flag for running

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

// Signal handler for SIGINT
void sigint_handler(int sig)
{
    run = 0;
}

// Function for CAN reading
void *can_reader(void *arg)
{
    printf("[CAN READER] starting\n");
    int s = *((int *)arg);

    while (run)
    {
        struct can_frame frame;
        if (read(s, &frame, sizeof(frame)) > 0)
        {
            pthread_mutex_lock(&recv_data.lock);
            recv_data.frame = frame;
            recv_data.updated = 1;
            pthread_mutex_unlock(&recv_data.lock);
        }
        usleep(1000); // Reduce CPU usage
    }
    printf("[CAN READER] ending\n");
    return NULL;
}

// Function for CAN sending
void *can_sender(void *arg)
{
    printf("[CAN SENDER] starting\n");
    int s = *((int *)arg);
    struct can_frame frame = {
        .can_id = 0x7E0,
        .can_dlc = 1,
        .data = {0x01}};

    while (run)
    {
        pthread_mutex_lock(&send_data.lock);
        if (send_data.updated)
        {
            if (write(s, &send_data.frame, sizeof(send_data.frame)) != sizeof(send_data.frame))
            {
                perror("Write");
            }
            send_data.updated = 0;
        }
        pthread_mutex_unlock(&send_data.lock);

        usleep(1000); // Send every 500 ms
    }
    printf("[CAN SENDER] ending\n");
    return NULL;
}

void *api_sender(void *arg)
{
    printf("[API SENDER] starting\n");

    while (run)
    {
        pthread_mutex_lock(&lora_data.lock);
        if (lora_data.updated)
        {
            char *json = convert_to_json(&lora_data.msg);
            sendToApi(json);
            lora_data.updated = 0;
        }
        pthread_mutex_unlock(&lora_data.lock);
        usleep(1000); // Send every 500 ms
    }
    printf("[API SENDER] ending\n");
    return NULL;
}

int main()
{
    struct sockaddr_can addr;
    struct ifreq ifr;
    pthread_t can_read_thread_id, can_send_thread_id, api_send_thread_id;
    int s;

    signal(SIGINT, sigint_handler); // Register the signal handler

    // Create a socket
    if ((s = socket(PF_CAN, SOCK_RAW, CAN_RAW)) < 0)
    {
        perror("Error while opening socket");
        return -1;
    }

    fcntl(s, F_SETFL, O_NONBLOCK); // Set the socket to non-blocking

    strcpy(ifr.ifr_name, "can0");
    ioctl(s, SIOCGIFINDEX, &ifr);

    memset(&addr, 0, sizeof(addr));
    addr.can_family = AF_CAN;
    addr.can_ifindex = ifr.ifr_ifindex;
    if (bind(s, (struct sockaddr *)&addr, sizeof(addr)) < 0)
    {
        perror("Error in socket bind");
        return -2;
    }

    pthread_mutex_init(&recv_data.lock, NULL);
    pthread_mutex_init(&send_data.lock, NULL);
    pthread_mutex_init(&lora_data.lock, NULL);
    recv_data.updated = 0;
    send_data.updated = 0;
    lora_data.updated = 0;

    pthread_create(&can_read_thread_id, NULL, can_reader, &s);
    pthread_create(&can_send_thread_id, NULL, can_sender, &s);
    pthread_create(&api_send_thread_id, NULL, api_sender, NULL);

    sx1280_spi_t dev = {-1, -1, 1, 19, 20, 21, 18, 12, 6, 13, 16, 19, STANDBY_RC};
    Sx1280SPIInit(&dev);
    WaitForSetup(&dev);

    uint8_t rxBuffStatus[2];
    GsPingMsg_t ping = {0};

    long long tLastPing = 0;
    long long tLastMsg = millis();

    int dio1 = 0;
    int dio2 = 0;
    int dio3 = 0;

    while (run)
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
                // printf("Ping done: idx = %d\n", ping.idx);
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
            if (dio3) // Error
            {
                perror("RX error");
            }
            else
            {
                if (GetRxBufferStatus(&dev, rxBuffStatus) == -1)
                {
                    WaitForSetup(&dev);
                }
                pthread_mutex_lock(&lora_data.lock);
                if (ReadBuffer(&dev, (uint8_t *)&lora_data.msg, (size_t)rxBuffStatus[0], rxBuffStatus[1]) == -1)
                {
                    WaitForSetup(&dev);
                }
                printf("New msg: fsw_state = %d \t freq = %d Hz\n", lora_data.msg.fsw_state, (int)(1000 / (millis() - tLastMsg)));
                lora_data.updated = 1;
                pthread_mutex_unlock(&lora_data.lock);
                tLastMsg = millis();
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

        pthread_mutex_lock(&recv_data.lock);
        if (recv_data.updated)
        {
            printf("\nReceived a frame with ID: 0x%03X\n", recv_data.frame.can_id);
            if (recv_data.frame.can_dlc > 0)
            {
                printf("Data: ");
                for (int i = 0; i < recv_data.frame.can_dlc; i++)
                    printf("%02X ", recv_data.frame.data[i]);
                printf("\n");
            }
            recv_data.updated = 0;
        }
        pthread_mutex_unlock(&recv_data.lock);
        printf(".");
        fflush(stdout);
        usleep(100000);
    }

    printf("[MAIN] stoping - wainting for threads to end\n");
    pthread_join(can_read_thread_id, NULL);
    pthread_join(can_send_thread_id, NULL);
    pthread_join(api_send_thread_id, NULL);
    close(s);
    printf("[MAIN] ending\n");
    return 0;
}
