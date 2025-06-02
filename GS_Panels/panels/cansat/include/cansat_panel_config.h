#ifndef __CONFIG_H__
#define __CONFIG_H__

/* CAN defines */
#define CAN_PIO_NUM 0
#define CAN_PIO_IRQ PIO0_IRQ_0_IRQn
#define CAN_SYS_CLOCK 125000000

/* PINS */
#define CAN_RX_PIN 11
#define CAN_TX_PIN 12
#define BUZZER_PIN 16

//--- Neopixel 1 ---//
/*
    RBF
    MAIN
    DROGUE
    ROCKET COMMS
    DOORS OPEN
    RADIO RECORD
    READY TO LAUNCH
    RESERVE 4
    RESERVE 3
    RESERVE 2
    RESERVE 1
    DOOR 2 OPENED
    DOOR 1 OPENED
    CANSAT 3-4 FIRED
    CANSAT 1-2 FIRED
    LOGO
    LOGO
    LOGO
    LOGO
    LOGO
    LOGO
*/
#define NEO_PIN 6
#define NEO_COUNT 63

#define RBF_BTN_PIN 19     // 13
#define RBF_KEY_BTN_PIN 13 // 19
#define MAIN_BTN_PIN 14
#define DROGUE_BTN_PIN 7
#define CANSAT_BTN_PIN 1 // Payload 1, 2 in schematic
#define DOOR_BTN_PIN 28  // Payload 3, 4 in schematic
#define RES_1_BTN_PIN 15
#define RES_2_BTN_PIN 8
#define RES_3_BTN_PIN 2
#define RES_4_BTN_PIN 27

// Button LED is a button backlight
#define MAIN_LED_PIN 17
#define DROGUE_LED_PIN 10
#define CANSAT_LED_PIN 4 // Payload 1, 2 in schematic
#define DOOR_LED_PIN 21  // Payload 3, 4 in schematic
#define RES_1_LED_PIN 18
#define RES_2_LED_PIN 9
#define RES_3_LED_PIN 3
#define RES_4_LED_PIN 22

#define CAN_MSG_FREQ 30
#define DEBOUNCE_TIME_US 1e5

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
#define ROCKETS_COMMS_NEO_SHIFT 27
#define DOORS_OPEN_NEO_SHIFT 30
#define RADIO_RECORD_NEO_SHIFT 33
#define READY_TO_LAUNCH_NEO_SHIFT 36
#define RES_4_NEO_SHIFT 39
#define RES_3_NEO_SHIFT 42
#define RES_2_NEO_SHIFT 45
#define RES_1_NEO_SHIFT 48
#define DOOR_2_NEO_SHIFT 51
#define DOOR_1_NEO_SHIFT 54
#define CANSAT_3_4_NEO_SHIFT 57
#define CANSAT_1_2_NEO_SHIFT 60

enum
{
    OFF,
    RED,
    GREEN,
    BLUE,
    ORANGE,
};

#endif // __CONFIG_H__