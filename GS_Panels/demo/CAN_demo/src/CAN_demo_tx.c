#include "pico/stdlib.h"
#include "hardware/gpio.h"
#include "hardware/irq.h"
#include "hardware/pio.h"
#include "RP2040.h"
#include "can.h"
#include <ws2812.h>
#include <stdio.h>

#define BUZZER_PIN 16

//--- Neopixel 1 ---//
/*
    RBF
    MAIN
    DROGUE
    READY TO LAUNCH
    DOORS OPEN
    RADIO RECORD
    ROCKET COMMS
*/
#define NEO_1_PIN 6
#define NEO_1_COUNT 21

//--- Neopixel 2 ---//
/*
    DOOR 2 OPENED
    DOOR 1 OPENED
    CANSAT 3-4 FIRED
    CANSAT 1-2 FIRED
*/
#define NEO_2_PIN 0
#define NEO_2_COUNT 12

//--- Neopixel 3 ---//
/*
    RESERVE 4
    RESERVE 3
    RESERVE 2
    RESERVE 1
*/
#define NEO_3_PIN 26
#define NEO_3_COUNT 12

#define RBF_SWITCH 13
#define RBF_KEY 19
#define MAIN_BUTTON 14
#define DROGUE_BUTTON 7
#define CANSAT_BUTTON 1 // Payload 1, 2 in schematic
#define DOOR_BUTTON 28  // Payload 3, 4 in schematic
#define RESERVER_1_BUTTON 15
#define RESERVER_2_BUTTON 8
#define RESERVER_3_BUTTON 2
#define RESERVER_4_BUTTON 27

// Button LED is a button backlight
#define MAIN_BUTTON_LED 17
#define DROGUE_BUTTON_LED 10
#define CANSAT_BUTTON_LED 4 // Payload 1, 2 in schematic
#define DOOR_BUTTON_LED 21  // Payload 3, 4 in schematic
#define RESERVE_1_BUTTON_LED 18
#define RESERVE_2_BUTTON_LED 9
#define RESERVE_3_BUTTON_LED 3
#define RESERVE_4_BUTTON_LED 22

static struct ws2812_t ws2812_1;
static struct ws2812_t ws2812_2;
static struct ws2812_t ws2812_3;

/* CAN defines */
#define PIO_NUM 0
#define PIO_IRQ PIO0_IRQ_0_IRQn
#define SYS_CLOCK 125000000
#define CAN_TX_PIN 12
#define CAN_RX_PIN 11

can_message_t msg;

static void can_rx_cb(struct can2040 *cd, uint32_t notify, struct can2040_msg *msg)
{
    // Add message processing code here...
}

int main()
{
    // Initialize the Pico SDK
    stdio_init_all();

    ws2812_setup(&ws2812_1, NEO_1_PIN, NEO_1_COUNT);
    ws2812_setup(&ws2812_2, NEO_2_PIN, NEO_2_COUNT);
    ws2812_setup(&ws2812_3, NEO_3_PIN, NEO_3_COUNT);

    ws2812_set_all(&ws2812_1, 255, 0, 0);
    ws2812_show(&ws2812_1);

    ws2812_set_all(&ws2812_2, 0, 255, 0);
    ws2812_show(&ws2812_2);

    ws2812_set_all(&ws2812_3, 0, 0, 255);
    ws2812_show(&ws2812_3);

    can_setup(PIO_NUM, PIO_IRQ, SYS_CLOCK, CAN_RX_PIN, CAN_TX_PIN, can_rx_cb);

    sleep_ms(1000);

    int brightness = 0;

    while (true)
    {
        brightness = (brightness + 1) % 255;

        ws2812_set_all(&ws2812_2, 0, brightness, 0);

        for (int i = 0; i < 5; i++)
        {
            ws2812_show(&ws2812_2);
            sleep_us(200);
        }

        msg.type = CAN_MESSAGE_NEOPIXEL;
        msg.data.neopixel.id = 1;
        msg.data.neopixel.pos = 4;
        msg.data.neopixel.r = 0;
        msg.data.neopixel.g = brightness;
        msg.data.neopixel.b = 0;
        can_transmit(&msg);
        sleep_ms(1000);
    }

    return 0;
}