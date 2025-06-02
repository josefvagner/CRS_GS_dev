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
#include <math.h>
#include <lgpio.h>

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
    int freq;
    bool updated; // Flag to indicate new data
} lora_recv_shared_data;

typedef struct
{
    GsLoraMsg_t msg;
    pthread_mutex_t lock;
    bool updated; // Flag to indicate new data
} lora_send_shared_data;

can_shared_data recv_data = {
    .updated = false,
};
can_shared_data send_data = {
    .updated = false,
};
lora_send_shared_data lora_send_data = {
    .msg = {0},
    .updated = false,
};
lora_recv_shared_data lora_recv_data = {
    .msg = {0},
    .updated = false,
    .freq = 0,
};
lora_recv_shared_data lora_recv_data_api = {
    .msg = {0},
    .updated = false,
    .freq = 0,
};

cansat_panel_t panel = {0};

int pi = -1;

volatile sig_atomic_t run = 1; // Global flag for running

char *convert_to_json(sd_csv_data_t *data, cansat_panel_t *panel);
void sendToApi(const char *json);
void sigint_handler(int sig);
void *can_thread(void *arg);
void *api_sender(void *arg);
bool sendLoraMsg(sx1280_spi_t *dev, GsLoraMsg_t *msg);
void *lora_thread(void *arg);

int main()
{
    /* REGISTER CTRL+C KILL SIGNAL */
    signal(SIGINT, sigint_handler);

    /* CAN SOCKET INITIALIZATION */
    struct sockaddr_can addr;
    struct ifreq ifr;
    int s;
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

    pi = lgGpiochipOpen(0);
    if (pi < 0)
    {
        perror("Can't open gpiochip");
        exit(100);
    }

    if (lgGpioClaimInput(pi, LG_SET_PULL_UP, LOCK_BTN_PIN) < 0)
    {
        perror("Can't claim BUSY pin");
        exit(100);
    }

    /* LORA INITIALIZATION */
    sx1280_spi_t dev = {
        .pi = pi,
        .spi = -1,
        .spiDev = 1,
        .spiChen = 0,
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

    /* SETUP THREADS */
    pthread_t can_thread_id, lora_thread_id, api_send_thread_id;
    pthread_mutex_init(&recv_data.lock, NULL);
    pthread_mutex_init(&send_data.lock, NULL);
    pthread_mutex_init(&lora_send_data.lock, NULL);
    pthread_mutex_init(&lora_recv_data.lock, NULL);
    pthread_mutex_init(&lora_recv_data_api.lock, NULL);

    pthread_create(&can_thread_id, NULL, can_thread, &s);
    pthread_create(&lora_thread_id, NULL, lora_thread, &dev);
    pthread_create(&api_send_thread_id, NULL, api_sender, NULL);

    /* SETUP MAIN LOOP VARIABLES */
    long long now = millis();
    long long last_lora_msg_recieved_time = 0;
    long long last_lora_send_time = millis();
    long long last_can_send_time = millis();
    long long last_print_time = millis();
    int i = 0;

    sleep(1);
    printf("\n");
    fputs("\e[?25l", stdout); // Hide cursor

    /* MAIN LOOP */
    while (run)
    {
        now = millis();

        if ((now - last_print_time) >= 100)
        {
            printf("FSW timestamp: %5.2f s   |   ", (float)lora_recv_data.msg.timestamp / 1000);
            printf("LORA freq: %3d Hz   |   ", lora_recv_data.freq);
            printf("LORA recv/send state: %8s / %-8s\n", SmStateVerbose[lora_recv_data.msg.fsw_state.state_pwb], SmStateVerbose[lora_send_data.msg.fsw_state]);
            last_print_time = now;
        }

        /* NEW MESSAGE FROM CAN */
        if (recv_data.updated)
        {
            if (recv_data.frame.can_dlc > 0 && lgGpioRead(pi, LOCK_BTN_PIN) == 0)
            {
                uint64_t tmp = 0;
                pthread_mutex_lock(&recv_data.lock);
                memcpy(&tmp, &recv_data.frame.data, sizeof(uint64_t));
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

        /* NEW MESSAGE FROM LORA */
        if (lora_recv_data.updated)
        {
            if (lora_recv_data.msg.fsw_state.state_pwb > lora_send_data.msg.fsw_state)
            {
                pthread_mutex_lock(&lora_send_data.lock);
                lora_send_data.msg.fsw_state = lora_recv_data.msg.fsw_state.state_pwb;
                lora_send_data.updated = true;
                pthread_mutex_unlock(&lora_send_data.lock);
            }
            pthread_mutex_lock(&lora_recv_data.lock);
            lora_recv_data.updated = false;
            pthread_mutex_unlock(&lora_recv_data.lock);
            last_lora_msg_recieved_time = now;
        }

        if ((now - last_lora_msg_recieved_time) >= 500)
            panel.rockets_comms_neo = panel.radio_record_neo = RED;
        else
            panel.rockets_comms_neo = panel.radio_record_neo = GREEN;

        switch (lora_recv_data.msg.eject_output.doors)
        {
        case WAITING:
            panel.door_1_neo = panel.door_2_neo = panel.doors_open_neo = OFF;
            break;
        case READY:
            panel.door_1_neo = panel.door_2_neo = panel.doors_open_neo = RED;
            break;
        case EJECTING:
            panel.door_1_neo = panel.door_2_neo = panel.doors_open_neo = ORANGE;
            break;
        case EJECTED:
            panel.door_1_neo = panel.door_2_neo = panel.doors_open_neo = GREEN;
            break;
        default:
            break;
        }

        switch (lora_recv_data.msg.eject_output.payload)
        {
        case WAITING:
            panel.cansat_1_2_neo = panel.cansat_3_4_neo = OFF;
            break;
        case READY:
            panel.cansat_1_2_neo = panel.cansat_3_4_neo = RED;
            break;
        case EJECTING:
            panel.cansat_1_2_neo = panel.cansat_3_4_neo = ORANGE;
            break;
        case EJECTED:
            panel.cansat_1_2_neo = panel.cansat_3_4_neo = GREEN;
            break;
        default:
            break;
        }

        switch (lora_recv_data.msg.eject_output.drogue)
        {
        case WAITING:
            panel.drogue_neo = OFF;
            break;
        case READY:
            panel.drogue_neo = RED;
            break;
        case EJECTING:
            panel.drogue_neo = ORANGE;
            break;
        case EJECTED:
            panel.drogue_neo = GREEN;
            break;
        default:
            break;
        }

        switch (lora_recv_data.msg.eject_output.parachute)
        {
        case WAITING:
            panel.main_neo = OFF;
            break;
        case READY:
            panel.main_neo = RED;
            break;
        case EJECTING:
            panel.main_neo = ORANGE;
            break;
        case EJECTED:
            panel.main_neo = GREEN;
            break;
        default:
            break;
        }

        panel.main_led = lora_recv_data.msg.eject_output.parachute >= READY;
        panel.drogue_led = lora_recv_data.msg.eject_output.drogue >= READY;
        panel.door_led = lora_recv_data.msg.eject_output.doors >= READY;
        panel.cansat_led = lora_recv_data.msg.eject_output.payload >= READY;

        panel.res_4_led = panel.cansat_btn || panel.door_btn || panel.main_btn || panel.drogue_btn;

        if (panel.res_4_btn && !panel.rbf_btn && !panel.rbf_key_btn && lora_send_data.msg.fsw_state != SM_STATE_INIT)
        {
            printf("[MAIN] Resetting FSW state\n");
            pthread_mutex_lock(&lora_send_data.lock);
            lora_send_data.msg.fsw_state = SM_STATE_INIT;
            pthread_mutex_unlock(&lora_send_data.lock);
            pthread_mutex_lock(&lora_recv_data.lock);
            sd_csv_data_t tmp = {0};
            lora_recv_data.msg = tmp;
            pthread_mutex_unlock(&lora_recv_data.lock);
        }

        switch (lora_recv_data.msg.fsw_state.state_pwb)
        {
        case SM_STATE_INIT:
            panel.ready_to_launch_neo = OFF;
            break;
        case SM_STATE_READY:
        case SM_STATE_WAIT_GS:
            panel.ready_to_launch_neo = GREEN;
            break;
        case SM_STATE_ARM:
            panel.ready_to_launch_neo = RED;
            if ((panel.rbf_btn == 0 || panel.rbf_key_btn == 0) && lora_send_data.msg.fsw_state < SM_STATE_FAIL)
            {
                pthread_mutex_lock(&lora_send_data.lock);
                lora_send_data.msg.fsw_state = SM_STATE_FAIL;
                lora_send_data.updated = true;
                pthread_mutex_unlock(&lora_send_data.lock);
            }
            break;
        case SM_STATE_ASCENT:
            panel.ready_to_launch_neo = OFF;
            break;
        case SM_STATE_APOGEE:
        case SM_STATE_DESCENT:
        case SM_STATE_LANDING:
        case SM_STATE_LANDED:
            panel.ready_to_launch_neo = OFF;
            break;
        case SM_STATE_FAIL:
            panel.ready_to_launch_neo = OFF;
            break;

        default:
            break;
        }

        if (panel.rbf_key_btn == 0 || panel.rbf_btn == 0)
        {
            panel.rbf_neo = lora_recv_data.msg.fsw_state.state_pwb == SM_STATE_WAIT_GS ? BLUE : RED;
        }
        else
        {
            panel.rbf_neo = GREEN;
        }

        if (lora_recv_data.msg.fsw_state.state_pwb <= SM_STATE_READY && panel.rbf_btn && panel.rbf_key_btn)
        {

            if (lora_send_data.msg.fsw_state < SM_STATE_FAIL)
            {
                pthread_mutex_lock(&lora_send_data.lock);
                lora_send_data.msg.fsw_state = SM_STATE_FAIL;
                lora_send_data.updated = true;
                pthread_mutex_unlock(&lora_send_data.lock);
            }
        }

        if (lora_recv_data.msg.fsw_state.state_pwb == SM_STATE_WAIT_GS && panel.rbf_btn && panel.rbf_key_btn)
        {

            if (lora_send_data.msg.fsw_state < SM_STATE_ARM)
            {
                pthread_mutex_lock(&lora_send_data.lock);
                lora_send_data.msg.fsw_state = SM_STATE_ARM;
                lora_send_data.updated = true;
                pthread_mutex_unlock(&lora_send_data.lock);
            }
        }

        if (panel.main_btn && lora_send_data.msg.fsw_state < SM_STATE_LANDING)
        {
            pthread_mutex_lock(&lora_send_data.lock);
            lora_send_data.msg.fsw_state = SM_STATE_LANDING;
            lora_send_data.updated = true;
            pthread_mutex_unlock(&lora_send_data.lock);
        }

        if (panel.drogue_btn && lora_send_data.msg.fsw_state < SM_STATE_DESCENT)
        {
            pthread_mutex_lock(&lora_send_data.lock);
            lora_send_data.msg.fsw_state = SM_STATE_DESCENT;
            lora_send_data.updated = true;
            pthread_mutex_unlock(&lora_send_data.lock);
        }

        if (panel.cansat_btn && lora_send_data.msg.fsw_state < SM_STATE_APOGEE)
        {
            pthread_mutex_lock(&lora_send_data.lock);
            lora_send_data.msg.fsw_state = SM_STATE_APOGEE;
            lora_send_data.updated = true;
            pthread_mutex_unlock(&lora_send_data.lock);
        }

        if (panel.door_btn && lora_send_data.msg.fsw_state < SM_STATE_APOGEE)
        {
            pthread_mutex_lock(&lora_send_data.lock);
            lora_send_data.msg.fsw_state = SM_STATE_APOGEE;
            lora_send_data.updated = true;
            pthread_mutex_unlock(&lora_send_data.lock);
        }

        /* UPDATE INDEX AND SEND NEW LORA MESSAGE */
        if ((now - last_lora_send_time) >= LORA_SEND_T_MS)
        {
            pthread_mutex_lock(&lora_send_data.lock);
            lora_send_data.msg.idx = lora_send_data.msg.idx > 100 ? 0 : lora_send_data.msg.idx + 1;
            lora_send_data.updated = true;
            pthread_mutex_unlock(&lora_send_data.lock);
            last_lora_send_time = millis();
        }

        /* SEND NEW CAN MESSAGE */
        if ((now - last_can_send_time) >= CAN_SEND_T_MS)
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

        usleep(100);
    }

    fputs("\e[?25h", stdout); // Show cursor

    printf("\n\n");

    printf("[MAIN] stoping - wainting for threads to end\n");
    pthread_join(can_thread_id, NULL);
    pthread_join(lora_thread_id, NULL);
    pthread_join(api_send_thread_id, NULL);
    lgSpiClose(dev.spi);
    lgGpiochipClose(pi);
    close(s);
    printf("[MAIN] ending\n");
    return 0;
}

char *convert_to_json(sd_csv_data_t *data, cansat_panel_t *panel)
{
    cJSON *root = cJSON_CreateObject();
    cJSON *fsw_state = cJSON_CreateObject();
    cJSON *board_status = cJSON_CreateObject();
    cJSON *eject_output = cJSON_CreateObject();
    cJSON *bmp_data = cJSON_CreateObject();
    cJSON *pos_data = cJSON_CreateObject();
    cJSON *bno_data = cJSON_CreateObject();
    cJSON *ina_data_pwb = cJSON_CreateObject();
    cJSON *ina_data_dob = cJSON_CreateObject();
    cJSON *gps_data = cJSON_CreateObject();

    cJSON_AddNumberToObject(root, "timestamp", data->timestamp);
    cJSON_AddBoolToObject(root, "lock_btn", lgGpioRead(pi, LOCK_BTN_PIN) == 1);
    cJSON_AddBoolToObject(root, "comms", panel->rockets_comms_neo == GREEN);
    cJSON_AddNumberToObject(root, "state_gs", lora_send_data.msg.fsw_state);

    cJSON_AddNumberToObject(fsw_state, "state_cmb", data->fsw_state.state_cmb);
    cJSON_AddNumberToObject(fsw_state, "state_cnb", data->fsw_state.state_cnb);
    cJSON_AddNumberToObject(fsw_state, "state_pwb", data->fsw_state.state_pwb);

    cJSON_AddNumberToObject(board_status, "pwb_status", data->board_status.pwb_status);
    cJSON_AddNumberToObject(board_status, "dob_status", data->board_status.dob_status);
    cJSON_AddNumberToObject(board_status, "cnb_status", data->board_status.cnb_status);
    cJSON_AddNumberToObject(board_status, "snb_status", data->board_status.snb_status);
    cJSON_AddNumberToObject(board_status, "cmb_status", data->board_status.cmb_status);

    cJSON_AddNumberToObject(eject_output, "doors", data->eject_output.doors);
    cJSON_AddNumberToObject(eject_output, "doors_dob", data->eject_output.doors_dob);
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

    cJSON_AddNumberToObject(ina_data_pwb, "voltage", data->ina_data_pwb.voltage);
    cJSON_AddNumberToObject(ina_data_pwb, "current", data->ina_data_pwb.current);

    cJSON_AddNumberToObject(ina_data_dob, "voltage", data->ina_data_dob.voltage);
    cJSON_AddNumberToObject(ina_data_dob, "current", data->ina_data_dob.current);

    cJSON_AddNumberToObject(gps_data, "is_connected", data->gps_data.is_connected);
    cJSON_AddNumberToObject(gps_data, "num_fixed_satellites", data->gps_data.num_fixed_satellites);
    cJSON_AddNumberToObject(gps_data, "latitude", data->gps_data.latitude);
    cJSON_AddNumberToObject(gps_data, "longitude", data->gps_data.longitude);

    cJSON_AddItemToObject(root, "fsw_state", fsw_state);
    cJSON_AddItemToObject(root, "board_status", board_status);
    cJSON_AddItemToObject(root, "eject_output", eject_output);
    cJSON_AddItemToObject(root, "bmp_data", bmp_data);
    cJSON_AddItemToObject(root, "pos_data", pos_data);
    cJSON_AddItemToObject(root, "bno_data", bno_data);
    cJSON_AddItemToObject(root, "ina_data_pwb", ina_data_pwb);
    cJSON_AddItemToObject(root, "ina_data_dob", ina_data_dob);
    cJSON_AddItemToObject(root, "gps_data", gps_data);

    char *json_string = cJSON_Print(root);
    cJSON_Delete(root);

    return json_string;
}

void sendToApi(const char *json)
{
    int sockfd = socket(AF_INET, SOCK_STREAM, 0);
    if (sockfd < 0)
        perror("socket creation failed");
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
                // fprintf(stderr, "[CAN THREAD] Error while writing to CAN");
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

    long long tLastApiSend = millis();

    while (run)
    {
        if (lora_recv_data_api.updated || (millis() - tLastApiSend) >= 50)
        {
            pthread_mutex_lock(&lora_recv_data_api.lock);
            char *json = convert_to_json(&lora_recv_data_api.msg, &panel);
            lora_recv_data_api.updated = false;
            pthread_mutex_unlock(&lora_recv_data_api.lock);
            sendToApi(json);
            tLastApiSend = millis();
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
        dio1 = lgGpioRead(dev->pi, dev->dio1Pin);
        dio3 = lgGpioRead(dev->pi, dev->dio3Pin);
        if (dio3) // Error
        {
            uint16_t irq = 0;
            if (GetIrqStatus(dev, &irq) == -1)
            {
                WaitForSetup(dev);
            }
            // fprintf(stderr, "[LORA THREAD] Error in TX mode with irq: %b", irq);
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

        dio1 = lgGpioRead(dev->pi, dev->dio1Pin);
        dio2 = lgGpioRead(dev->pi, dev->dio2Pin);
        dio3 = lgGpioRead(dev->pi, dev->dio3Pin);

        if (dev->state == RX && (dio2 || dio3))
        {
            if (dio3)
            {
                irq = 0;
                if (GetIrqStatus(dev, &irq) == -1)
                {
                    WaitForSetup(dev);
                }
                // fprintf(stderr, "[LORA THREAD] Error in RX mode with irq: %b", irq);
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
                // printf("New msg_fsw: fsw_state = %d \t freq = %d Hz\n", msg_fsw.fsw_state.state, (int)(1000 / (tNow - tLastMsg)));

                if (msg_fsw.fsw_state.state_pwb <= SM_STATE_FAIL)
                {
                    int freq = (int)(1000 / (tNow - tLastMsg));
                    pthread_mutex_lock(&lora_recv_data.lock);
                    pthread_mutex_lock(&lora_recv_data_api.lock);
                    memcpy(&lora_recv_data.msg, &msg_fsw, sizeof(msg_fsw));
                    memcpy(&lora_recv_data_api.msg, &msg_fsw, sizeof(msg_fsw));
                    lora_recv_data.updated = true;
                    lora_recv_data_api.updated = true;
                    lora_recv_data.freq = freq;
                    lora_recv_data_api.freq = freq;
                    pthread_mutex_unlock(&lora_recv_data.lock);
                    pthread_mutex_unlock(&lora_recv_data_api.lock);
                    tLastMsg = tNow;
                }
            }
            if (ClrIrqStatus(dev, 0xFFFF) == -1)
            {
                WaitForSetup(dev);
            }
        }

        if (lora_send_data.updated)
        {
            pthread_mutex_lock(&lora_send_data.lock);
            memcpy(&msg_gs, &lora_send_data.msg, sizeof(lora_send_data.msg));
            lora_send_data.updated = false;
            pthread_mutex_unlock(&lora_send_data.lock);
            if (sendLoraMsg(dev, &msg_gs))
            {
                // printf("Ping sended idx = %d \t %d\n", msg_gs.idx, msg_gs.fsw_state);
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
