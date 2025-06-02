#ifndef _CAN_H
#define _CAN_H

#include "pico/stdlib.h"
#include "hardware/gpio.h"
#include "can2040.h"
#include "hardware/irq.h"
#include "hardware/pio.h"
#include "RP2040.h"
#include <stdio.h>
#include "can_message.h"

#define BITRATE 500000

// set up the CAN bus and specify callback called when message is received
void can_setup(uint32_t pio_num, uint pio_irq, uint32_t sys_clock, uint32_t gpio_rx, uint32_t gpio_tx, can2040_rx_cb can2040_cb);

// transmit message
int can_transmit(can_message_t *msg);

#endif // _CAN_H
