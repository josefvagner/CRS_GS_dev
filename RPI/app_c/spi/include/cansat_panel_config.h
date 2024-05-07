#ifndef __CONFIG_H__
#define __CONFIG_H__

#include <stdint.h>

#define CAN_MSG_FREQ 30
#define LORA_MSG_FREQ 10

#define CAN_SEND_T_MS (long long)(1000 / CAN_MSG_FREQ)
#define LORA_SEND_T_MS (long long)(1000 / LORA_MSG_FREQ)

#define SERVER_IP "192.168.88.251"
#define SERVER_PORT 8000
#define ENDPOINT "/json_update"

#define RBF_BTN_SHIFT 0
#define RBF_KEY_BTN_SHIFT 1
#define MAIN_BTN_SHIFT 2
#define DROGUE_BTN_SHIFT 3
#define CANSAT_BTN_SHIFT 4
#define DOOR_BTN_SHIFT 5
#define RES_1_BTN_SHIFT 6
#define RES_2_BTN_SHIFT 7
#define RES_3_BTN_SHIFT 8
#define RES_4_BTN_SHIFT 9

#define MAIN_LED_SHIFT 10
#define DROGUE_LED_SHIFT 11
#define CANSAT_LED_SHIFT 12
#define DOOR_LED_SHIFT 13
#define RES_1_LED_SHIFT 14
#define RES_2_LED_SHIFT 15
#define RES_3_LED_SHIFT 16
#define RES_4_LED_SHIFT 17

#define RBF_NEO_SHIFT 18
#define MAIN_NEO_SHIFT 21
#define DROGUE_NEO_SHIFT 24
#define READY_TO_LAUNCH_NEO_SHIFT 27
#define DOORS_OPEN_NEO_SHIFT 30
#define RADIO_RECORD_NEO_SHIFT 33
#define ROCKETS_COMMS_NEO_SHIFT 36

#define DOOR_2_NEO_SHIFT 39
#define DOOR_1_NEO_SHIFT 42
#define CANSAT_3_4_NEO_SHIFT 45
#define CANSAT_1_2_NEO_SHIFT 48

#define RES_4_NEO_SHIFT 51
#define RES_3_NEO_SHIFT 54
#define RES_2_NEO_SHIFT 57
#define RES_1_NEO_SHIFT 60

enum
{
    OFF,
    RED,
    GREEN,
    BLUE,
    ORANGE,
};

// state number encoding
typedef enum
{
    SM_STATE_INIT,
    SM_STATE_READY,
    SM_STATE_ARM,
    SM_STATE_ASCENT,
    SM_STATE_APOGEE,  // payload
    SM_STATE_DESCENT, // drogue
    SM_STATE_LANDING, // parachute
    SM_STATE_LANDED,
    SM_STATE_FAIL
} SmState_e;

typedef enum
{
    CAN_MESSAGE_PANEL,
    CAN_MESSAGE_PANEL_W_BTNS,
} can_message_type;

typedef struct
{
    uint8_t rbf_neo;
    uint8_t main_neo;
    uint8_t drogue_neo;
    uint8_t ready_to_launch_neo;
    uint8_t doors_open_neo;
    uint8_t radio_record_neo;
    uint8_t rockets_comms_neo;

    uint8_t door_2_neo;
    uint8_t door_1_neo;
    uint8_t cansat_3_4_neo;
    uint8_t cansat_1_2_neo;

    uint8_t res_4_neo;
    uint8_t res_3_neo;
    uint8_t res_2_neo;
    uint8_t res_1_neo;

    bool rbf_btn;
    bool rbf_key_btn;
    bool main_btn;
    bool drogue_btn;
    bool cansat_btn;
    bool door_btn;
    bool res_1_btn;
    bool res_2_btn;
    bool res_3_btn;
    bool res_4_btn;

    bool main_led;
    bool drogue_led;
    bool cansat_led;
    bool door_led;
    bool res_1_led;
    bool res_2_led;
    bool res_3_led;
    bool res_4_led;
} cansat_panel_t;

#endif // __CONFIG_H__