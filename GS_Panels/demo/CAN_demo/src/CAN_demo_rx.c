#include "pico/stdlib.h"
#include "hardware/gpio.h"
#include "can.h"
#include "hardware/irq.h"
#include "hardware/pio.h"
#include "RP2040.h"
#include <ws2812.h>
#include <stdio.h>
#include <string.h>
#include <stdbool.h>

/* NeoPixel defines */
#define WS2812_PIN 22 // Stack pin is 29
#define WS2812_COUNT 1

/* CAN defines */
#define PIO_NUM 0
#define PIO_IRQ PIO0_IRQ_0_IRQn
#define SYS_CLOCK 125000000
#define CAN_TX_PIN 4
#define CAN_RX_PIN 5

struct ws2812_t ws2812;

/* structs of messages we are interested in receiving */
static struct can_message_neopixel_s neopixel;
static bool updateNeopixel;

/* called on CAN message receive */
static void can_rx_cb(struct can2040 *cd, uint32_t notify, struct can2040_msg *msg)
{
    switch (msg->id)
    {
    /* the type of message also stores its id*/
    case (CAN_MESSAGE_NEOPIXEL):
    {
        /* copy correct size of data to be processed later, do not process here ! */
        memcpy(&neopixel, msg->data, sizeof(struct can_message_neopixel_s));
        updateNeopixel = true;
    }
    }
}

int main()
{
    // Initialize the Pico SDK
    stdio_init_all();

    ws2812_setup(&ws2812, WS2812_PIN, WS2812_COUNT);
    ws2812_set(&ws2812, 0, 15, 10, 0);
    ws2812_show(&ws2812);
    sleep_us(200);
    ws2812_show(&ws2812); // send data twice to force the color (problems with 3V3 systems ?)
    sleep_us(200);

    can_setup(PIO_NUM, PIO_IRQ, SYS_CLOCK, CAN_RX_PIN, CAN_TX_PIN, can_rx_cb);

    // ws2812_set(&ws2812, 0, 0, 10, 2);
    // ws2812_show(&ws2812);
    // sleep_us(200);

    while (true)
    {
        /* update neopixel from most recent received data */
        if(updateNeopixel) {
            ws2812_set(&ws2812, 0, neopixel.r, neopixel.g, neopixel.b);
            ws2812_show(&ws2812);
            sleep_ms(1);
            updateNeopixel = false;
        }
    }

    return 0;
}