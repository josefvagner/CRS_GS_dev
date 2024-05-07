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
#include "cansat_panel_config.h"

typedef struct
{
    struct can_frame frame;
    pthread_mutex_t lock;
    bool updated; // Flag to indicate new data
} can_shared_data;

typedef struct
{
    sd_csv_data_t msg;
    pthread_mutex_t lock;
    bool updated; // Flag to indicate new data
} lora_recv_shared_data;

typedef struct
{
    GsLoraMsg_t msg;
    pthread_mutex_t lock;
    bool updated; // Flag to indicate new data
} lora_send_shared_data;

can_shared_data recv_data;
can_shared_data send_data;
lora_send_shared_data lora_send_data;
lora_recv_shared_data lora_recv_data;
lora_recv_shared_data lora_recv_data_api;
volatile sig_atomic_t run = 1; // Global flag for running

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
void *can_thread(void *arg)
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

        if (send_data.updated)
        {
            pthread_mutex_lock(&send_data.lock);
            if (write(s, &send_data.frame, sizeof(send_data.frame)) != sizeof(send_data.frame))
            {
                perror("Write");
            }
            send_data.updated = 0;
            pthread_mutex_unlock(&send_data.lock);
        }

        usleep(100); // Reduce CPU usage
    }
    printf("[CAN READER] ending\n");
    return NULL;
}

void *api_sender(void *arg)
{
    printf("[API SENDER] starting\n");

    while (run)
    {
        if (lora_recv_data_api.updated)
        {
            pthread_mutex_lock(&lora_recv_data_api.lock);
            char *json = convert_to_json(&lora_recv_data_api.msg);
            lora_recv_data_api.updated = false;
            pthread_mutex_unlock(&lora_recv_data_api.lock);
            sendToApi(json);
        }
        usleep(1000); // Send every 500 ms
    }
    printf("[API SENDER] ending\n");
    return NULL;
}

bool sendLoraMsg(sx1280_spi_t *dev, GsLoraMsg_t *msg)
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

void *lora_thread(void *arg)
{
    printf("[LORA THREAD] starting\n");
    sx1280_spi_t *dev = (sx1280_spi_t *)arg;

    uint8_t rxBuffStatus[2];
    sd_csv_data_t msg_fsw = {0};
    GsLoraMsg_t msg_gs = {0};

    pthread_mutex_lock(&lora_recv_data.lock);
    memcpy(&lora_recv_data.msg, &msg_fsw, sizeof(msg_fsw));
    memcpy(&lora_recv_data_api.msg, &msg_fsw, sizeof(msg_fsw));
    pthread_mutex_unlock(&lora_recv_data.lock);

    pthread_mutex_lock(&lora_send_data.lock);
    memcpy(&msg_gs, &lora_send_data.msg, sizeof(lora_send_data.msg));
    pthread_mutex_unlock(&lora_send_data.lock);

    long long tLastPing = 0;
    long long tLastMsg = millis();
    long long tNow = millis();

    // long long tStart = millis();
    // long long diff = 0;

    int dio1 = 0;
    int dio2 = 0;
    int dio3 = 0;

    uint16_t irq = 0;

    while (run)
    {
        tNow = millis();
        /* simulation
        diff = tNow - tStart;

        if (diff >= 5000)
        {
            msg_fsw.fsw_state.state = SM_STATE_READY;
        }
        if (diff >= 10000)
        {
            msg_fsw.fsw_state.state = SM_STATE_ARM;
        }
        if (diff >= 15000)
        {
            msg_fsw.fsw_state.state = SM_STATE_ASCENT;
            msg_fsw.eject_output.doors = 1;
        }
        if (diff >= 20000)
        {
            msg_fsw.fsw_state.state = SM_STATE_APOGEE;
            msg_fsw.eject_output.payload = 1;
        }
        if (diff >= 25000)
        {
            msg_fsw.fsw_state.state = SM_STATE_DESCENT;
            msg_fsw.eject_output.drogue = 1;
        }
        if (diff >= 30000)
        {
            msg_fsw.fsw_state.state = SM_STATE_LANDING;
            msg_fsw.eject_output.parachute = 1;
        }
        if (diff >= 35000)
        {
            msg_fsw.fsw_state.state = SM_STATE_LANDED;
        }

        if ((tNow - tLastPing) >= LORA_SEND_T_MS)
        {
            pthread_mutex_lock(&lora_send_data.lock);
            memcpy(&msg_gs, &lora_send_data.msg, sizeof(msg_gs));
            lora_send_data.updated = false;
            pthread_mutex_unlock(&lora_send_data.lock);

            printf("Ping sended idx = %d\t%d %d %d %d %d \t", msg_gs.idx, msg_gs.rbf, msg_gs.main, msg_gs.drogue, msg_gs.cansat, msg_gs.door);

            pthread_mutex_lock(&lora_recv_data.lock);
            memcpy(&lora_recv_data.msg, &msg_fsw, sizeof(msg_fsw));
            lora_recv_data.updated = true;
            pthread_mutex_unlock(&lora_recv_data.lock);
            tLastPing = tNow;
        }
        */

        dio1 = gpio_read(dev->pi, dev->dio1Pin);
        dio2 = gpio_read(dev->pi, dev->dio2Pin);
        dio3 = gpio_read(dev->pi, dev->dio3Pin);

        if (dev->state == RX && (dio2 || dio3))
        {
            if (dio3)
            {
                irq = 0;
                if (GetIrqStatus(dev, &irq) == -1)
                {
                    WaitForSetup(dev);
                }
                printf("Error: irq = %b\n", irq);
            }
            else if (dio2) // RxDone
            {
                if (GetRxBufferStatus(dev, rxBuffStatus) == -1)
                {
                    WaitForSetup(dev);
                }
                if (ReadBuffer(dev, (uint8_t *)&msg_fsw, (size_t)rxBuffStatus[0], rxBuffStatus[1]) == -1)
                {
                    WaitForSetup(dev);
                }
                printf("New msg_fsw: fsw_state = %d \t freq = %d Hz\n", msg_fsw.fsw_state.state, (int)(1000 / (tNow - tLastMsg)));
                if (msg_fsw.fsw_state.state < 10)
                {
                    pthread_mutex_lock(&lora_recv_data.lock);
                    pthread_mutex_lock(&lora_recv_data_api.lock);
                    memcpy(&lora_recv_data.msg, &msg_fsw, sizeof(msg_fsw));
                    memcpy(&lora_recv_data_api.msg, &msg_fsw, sizeof(msg_fsw));
                    lora_recv_data.updated = true;
                    lora_recv_data_api.updated = true;
                    pthread_mutex_unlock(&lora_recv_data.lock);
                    pthread_mutex_unlock(&lora_recv_data_api.lock);
                }
                tLastMsg = tNow;
            }
            if (ClrIrqStatus(dev, 0xFFFF) == -1)
            {
                WaitForSetup(dev);
            }
        }

        if (((tNow - tLastPing) >= LORA_SEND_T_MS) || lora_send_data.updated)
        {
            pthread_mutex_lock(&lora_send_data.lock);
            memcpy(&msg_gs, &lora_send_data.msg, sizeof(lora_send_data.msg));
            lora_send_data.updated = false;
            pthread_mutex_unlock(&lora_send_data.lock);
            if (sendLoraMsg(dev, &msg_gs))
            {
                // printf("Ping sended idx = %d, freq = %d\n", msg_gs.idx, (int)(1000 / (millis() - tLastPing)));
                tLastPing = tNow;
            }
        }

        if (dio1 || dio2 || dio3)
        {
            if (ClrIrqStatus(dev, 0xFFFF) == -1)
            {
                WaitForSetup(dev);
            }
        }
        usleep(100); // Send every 500 ms
    }

    printf("[LORA THREAD] ending\n");
    return NULL;
}

int main()
{
    cansat_panel_t panel = {0};
    struct sockaddr_can addr;
    struct ifreq ifr;
    pthread_t can_thread_id, lora_thread_id, api_send_thread_id;
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

    emergency_timer_data_t timers = {
        .main = 1.0,
        .drogue = 1.0,
        .cansat = 1.0,
        .door = 1.0,
    };

    GsLoraMsg_t msg_gs = {
        .idx = 0,
        .rbf = 0,
        .main = 0,
        .drogue = 0,
        .cansat = 0,
        .door = 0,
        .emergency_timer = timers,
    };

    sd_csv_data_t msg_fsw = {0};

    lora_send_data.msg = msg_gs;

    Sx1280SPIInit(&dev);
    WaitForSetup(&dev);

    pthread_mutex_init(&recv_data.lock, NULL);
    pthread_mutex_init(&send_data.lock, NULL);
    pthread_mutex_init(&lora_send_data.lock, NULL);
    pthread_mutex_init(&lora_recv_data.lock, NULL);
    pthread_mutex_init(&lora_recv_data_api.lock, NULL);
    recv_data.updated = false;
    send_data.updated = false;
    lora_send_data.updated = false;
    lora_recv_data.updated = false;
    lora_recv_data_api.updated = false;

    pthread_create(&can_thread_id, NULL, can_thread, &s);
    pthread_create(&lora_thread_id, NULL, lora_thread, &dev);
    pthread_create(&api_send_thread_id, NULL, api_sender, NULL);

    long long last_lora_send_time = millis();
    long long last_can_send_time = millis();
    long long testT = millis();
    long long tNow = millis();
    long long last_lora_msg_recieved_time = 0;

    int i = 0;

    while (run)
    {
        tNow = millis();
        if (recv_data.updated)
        {
            // printf("\nReceived a frame with ID: 0x%03X\n", recv_data.frame.can_id);
            if (recv_data.frame.can_dlc > 0)
            {
                uint64_t tmp = 0;
                pthread_mutex_lock(&recv_data.lock);
                memcpy(&tmp, recv_data.frame.data, sizeof(uint64_t));
                pthread_mutex_unlock(&recv_data.lock);
                panel.rbf_btn = (tmp >> RBF_BTN_SHIFT) & 0x01;
                panel.rbf_key_btn = (tmp >> RBF_KEY_BTN_SHIFT) & 0x01;
                panel.main_btn = (tmp >> MAIN_BTN_SHIFT) & 0x01;
                panel.drogue_btn = (tmp >> DROGUE_BTN_SHIFT) & 0x01;
                panel.cansat_btn = (tmp >> CANSAT_BTN_SHIFT) & 0x01;
                panel.door_btn = (tmp >> DOOR_BTN_SHIFT) & 0x01;
                panel.res_1_btn = (tmp >> RES_1_BTN_SHIFT) & 0x01;
                panel.res_2_btn = (tmp >> RES_2_BTN_SHIFT) & 0x01;
                panel.res_3_btn = (tmp >> RES_3_BTN_SHIFT) & 0x01;
                panel.res_4_btn = (tmp >> RES_4_BTN_SHIFT) & 0x01;
            }
            pthread_mutex_lock(&recv_data.lock);
            recv_data.updated = false;
            pthread_mutex_unlock(&recv_data.lock);
        }

        if (msg_fsw.eject_output.doors)
            panel.door_1_neo = panel.door_2_neo = panel.doors_open_neo = GREEN;
        if (msg_fsw.eject_output.payload)
            panel.cansat_1_2_neo = panel.cansat_3_4_neo = GREEN;
        if (msg_fsw.eject_output.drogue)
            panel.drogue_neo = GREEN;
        if (msg_fsw.eject_output.parachute)
            panel.main_neo = GREEN;
        if (msg_fsw.fsw_state.state == SM_STATE_READY)
            panel.ready_to_launch_neo = GREEN;
        else if (msg_fsw.fsw_state.state == SM_STATE_ARM)
            panel.ready_to_launch_neo = RED;
        else
            panel.ready_to_launch_neo = OFF;

        if (msg_fsw.fsw_state.state > SM_STATE_ARM)
        {
            if (panel.main_btn == 0)
                panel.main_led = true;
            if (panel.drogue_btn == 0)
                panel.drogue_led = true;
            if (panel.cansat_btn == 0)
                panel.cansat_led = true;
            if (panel.door_btn == 0)
                panel.door_led = true;
        }
        else
            panel.main_led = panel.drogue_led = panel.cansat_led = panel.door_led = false;

        pthread_mutex_lock(&lora_send_data.lock);
        lora_send_data.msg.main = panel.main_btn;
        lora_send_data.msg.drogue = panel.drogue_btn;
        lora_send_data.msg.cansat = panel.cansat_btn;
        lora_send_data.msg.door = panel.door_btn;
        pthread_mutex_unlock(&lora_send_data.lock);

        if ((tNow - last_lora_msg_recieved_time) >= 1000)
            panel.rockets_comms_neo = panel.radio_record_neo = RED;
        else
            panel.rockets_comms_neo = panel.radio_record_neo = GREEN;

        if (panel.rbf_key_btn == 0)
        {
            panel.rbf_neo = RED;
            pthread_mutex_lock(&lora_send_data.lock);
            lora_send_data.msg.rbf = 0;
            pthread_mutex_unlock(&lora_send_data.lock);
        }
        else if (panel.rbf_btn == 0)
        {
            panel.rbf_neo = ORANGE;
            pthread_mutex_lock(&lora_send_data.lock);
            lora_send_data.msg.rbf = 0;
            pthread_mutex_unlock(&lora_send_data.lock);
        }
        else
        {
            panel.rbf_neo = GREEN;
            pthread_mutex_lock(&lora_send_data.lock);
            lora_send_data.msg.rbf = 1;
            pthread_mutex_unlock(&lora_send_data.lock);
        }

        if (panel.main_led && panel.main_btn)
        {
            pthread_mutex_lock(&lora_send_data.lock);
            lora_send_data.msg.main = 1;
            lora_send_data.updated = true;
            pthread_mutex_unlock(&lora_send_data.lock);
            panel.main_led = false;
        }

        if (panel.drogue_led && panel.drogue_btn)
        {
            pthread_mutex_lock(&lora_send_data.lock);
            lora_send_data.msg.drogue = 1;
            lora_send_data.updated = true;
            pthread_mutex_unlock(&lora_send_data.lock);
            panel.drogue_led = false;
        }

        if (panel.cansat_led && panel.cansat_btn)
        {
            pthread_mutex_lock(&lora_send_data.lock);
            lora_send_data.msg.cansat = 1;
            lora_send_data.updated = true;
            pthread_mutex_unlock(&lora_send_data.lock);
            panel.cansat_led = false;
        }

        if (panel.door_led && panel.door_btn)
        {
            pthread_mutex_lock(&lora_send_data.lock);
            lora_send_data.msg.door = 1;
            lora_send_data.updated = true;
            pthread_mutex_unlock(&lora_send_data.lock);
            panel.door_led = false;
        }

        if (lora_recv_data.updated)
        {
            pthread_mutex_lock(&lora_recv_data.lock);
            memcpy(&msg_fsw, &lora_recv_data.msg, sizeof(sd_csv_data_t));
            lora_recv_data.updated = false;
            pthread_mutex_unlock(&lora_recv_data.lock);
            printf("msg_fsw: %d\n", lora_recv_data.msg.fsw_state.state);
            last_lora_msg_recieved_time = tNow;
        }

        if ((tNow - last_lora_send_time) >= LORA_SEND_T_MS)
        {
            pthread_mutex_lock(&lora_send_data.lock);
            lora_send_data.msg.idx = lora_send_data.msg.idx > 100 ? 0 : lora_send_data.msg.idx + 1;
            pthread_mutex_unlock(&lora_send_data.lock);
            last_lora_send_time = millis();
        }

        /*
        if ((tNow - testT) >= 1000)
        {
            i = i > 5 ? 0 : i + 1;
            panel.rbf_neo = (uint8_t)i;
            panel.door_2_neo = (uint8_t)i;
            panel.res_4_neo = (uint8_t)i;
            panel.main_led = i % 2;
            printf("main_led = %d, rbf_neo = %d     main_btn = %d, drogue_btn = %d\n", panel.main_led, panel.rbf_neo, panel.main_btn, panel.drogue_btn);
            testT = millis();
        }
        */

        if ((tNow - last_can_send_time) >= CAN_SEND_T_MS)
        {
            uint64_t states = 0;
            states |= (uint64_t)panel.rbf_btn << RBF_BTN_SHIFT;
            states |= (uint64_t)panel.rbf_key_btn << RBF_KEY_BTN_SHIFT;
            states |= (uint64_t)panel.main_btn << MAIN_BTN_SHIFT;
            states |= (uint64_t)panel.drogue_btn << DROGUE_BTN_SHIFT;
            states |= (uint64_t)panel.cansat_btn << CANSAT_BTN_SHIFT;
            states |= (uint64_t)panel.door_btn << DOOR_BTN_SHIFT;
            states |= (uint64_t)panel.res_1_btn << RES_1_BTN_SHIFT;
            states |= (uint64_t)panel.res_2_btn << RES_2_BTN_SHIFT;
            states |= (uint64_t)panel.res_3_btn << RES_3_BTN_SHIFT;
            states |= (uint64_t)panel.res_4_btn << RES_4_BTN_SHIFT;

            states |= (uint64_t)panel.main_led << MAIN_LED_SHIFT;
            states |= (uint64_t)panel.drogue_led << DROGUE_LED_SHIFT;
            states |= (uint64_t)panel.cansat_led << CANSAT_LED_SHIFT;
            states |= (uint64_t)panel.door_led << DOOR_LED_SHIFT;
            states |= (uint64_t)panel.res_1_led << RES_1_LED_SHIFT;
            states |= (uint64_t)panel.res_2_led << RES_2_LED_SHIFT;
            states |= (uint64_t)panel.res_3_led << RES_3_LED_SHIFT;
            states |= (uint64_t)panel.res_4_led << RES_4_LED_SHIFT;

            states |= (uint64_t)panel.rbf_neo << RBF_NEO_SHIFT;
            states |= (uint64_t)panel.main_neo << MAIN_NEO_SHIFT;
            states |= (uint64_t)panel.drogue_neo << DROGUE_NEO_SHIFT;
            states |= (uint64_t)panel.ready_to_launch_neo << READY_TO_LAUNCH_NEO_SHIFT;
            states |= (uint64_t)panel.doors_open_neo << DOORS_OPEN_NEO_SHIFT;
            states |= (uint64_t)panel.radio_record_neo << RADIO_RECORD_NEO_SHIFT;
            states |= (uint64_t)panel.rockets_comms_neo << ROCKETS_COMMS_NEO_SHIFT;

            states |= (uint64_t)panel.door_1_neo << DOOR_1_NEO_SHIFT;
            states |= (uint64_t)panel.door_2_neo << DOOR_2_NEO_SHIFT;
            states |= (uint64_t)panel.cansat_1_2_neo << CANSAT_1_2_NEO_SHIFT;
            states |= (uint64_t)panel.cansat_3_4_neo << CANSAT_3_4_NEO_SHIFT;

            states |= (uint64_t)panel.res_1_neo << RES_1_NEO_SHIFT;
            states |= (uint64_t)panel.res_2_neo << RES_2_NEO_SHIFT;
            states |= (uint64_t)panel.res_3_neo << RES_3_NEO_SHIFT;
            states |= (uint64_t)panel.res_4_neo << RES_4_NEO_SHIFT;

            pthread_mutex_lock(&send_data.lock);
            send_data.frame.can_id = CAN_MESSAGE_PANEL;
            send_data.frame.can_dlc = 8;
            memcpy(send_data.frame.data, &states, sizeof(uint64_t));
            send_data.updated = true;
            pthread_mutex_unlock(&send_data.lock);

            last_can_send_time = millis();
        }

        fflush(stdout);
        usleep(100);
    }

    printf("[MAIN] stoping - wainting for threads to end\n");
    pthread_join(can_thread_id, NULL);
    pthread_join(lora_thread_id, NULL);
    pthread_join(api_send_thread_id, NULL);
    spi_close(dev.pi, dev.spi);
    pigpio_stop(dev.pi);
    close(s);
    printf("[MAIN] ending\n");
    return 0;
}
