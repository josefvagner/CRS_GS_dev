#include "can.h"
#include <string.h>

static struct can2040 cbus;

static void
PIOx_IRQHandler(void)
{
    can2040_pio_irq_handler(&cbus);
}

void can_setup(uint32_t pio_num, uint pio_irq, uint32_t sys_clock, uint32_t gpio_rx, uint32_t gpio_tx, can2040_rx_cb can2040_cb)
{
    // Setup canbus
    can2040_setup(&cbus, pio_num);
    can2040_callback_config(&cbus, can2040_cb);

    // Enable irqs
    irq_set_exclusive_handler(pio_irq, PIOx_IRQHandler);
    NVIC_SetPriority(pio_irq, 1);
    NVIC_EnableIRQ(pio_irq);

    // Start canbus
    can2040_start(&cbus, sys_clock, BITRATE, gpio_rx, gpio_tx);
}

int can_transmit(can_message_t *msg)
{
    struct can2040_msg c2040_msg;

    c2040_msg.id = msg->type;

    c2040_msg.dlc = message_size[msg->type];
    memcpy(c2040_msg.data, msg->data.data, message_size[msg->type]);
    return can2040_transmit(&cbus, &c2040_msg);
}