/* pico SDK libraries*/
#include "pico/stdlib.h"
#include "hardware/gpio.h"
#include "RP2040.h"
#include "pico/time.h"
/* custom libraries */
#include "can.h"
#include "ws2812.h"
#include <string.h>
/* configuration */
#include "cansat_panel_config.h"

can_message_t msg;
uint64_t states = 0;

static ws2812_t ws2812;

bool btn_pressed = false;
uint8_t pressed_btn_gpio = 0;

bool debounce_active = false;
absolute_time_t last_interrupt_time;

absolute_time_t last_msg_send_time;
absolute_time_t last_neo_update_time;

bool new_msg = false;

void button_interrupt_handler(uint gpio, uint32_t events)
{
    if (!debounce_active)
    {
        if (!btn_pressed)
        {
            debounce_active = true;
            last_interrupt_time = get_absolute_time();
            pressed_btn_gpio = (uint8_t)gpio;
            btn_pressed = true;
        }
    }
    else
    {
        absolute_time_t current_time = get_absolute_time();
        if (absolute_time_diff_us(last_interrupt_time, current_time) > DEBOUNCE_TIME_US)
        {
            debounce_active = false;
        }
    }
    gpio_acknowledge_irq(gpio, events);
}

void set_neo_range(ws2812_t *neo, int start, int end, int state)
{
    for (int i = start; i <= end; i++)
    {
        switch (state)
        {
        case OFF:
            ws2812_set(neo, i, 0, 0, 0);
            break;
        case RED:
            ws2812_set(neo, i, 255, 0, 0);
            break;
        case GREEN:
            ws2812_set(neo, i, 0, 255, 0);
            break;
        case BLUE:
            ws2812_set(neo, i, 0, 0, 255);
            break;
        case ORANGE:
            ws2812_set(neo, i, 255, 60, 0);
            break;
        default:
            break;
        }
    }
}

static void can_rx_cb(struct can2040 *cd, uint32_t notify, struct can2040_msg *recv)
{
    if (!((recv->id == CAN_MESSAGE_PANEL) || (recv->id == CAN_MESSAGE_PANEL_W_BTNS)))
    {
        return;
    }

    if (recv->dlc != sizeof(struct can_message_panel_s))
    {
        return;
    }

    uint64_t tmp = 0;
    memcpy(&tmp, recv->data, sizeof(struct can_message_panel_s));

    if (recv->id == CAN_MESSAGE_PANEL_W_BTNS)
    {
        states = tmp;
    }
    else
    {
        states = (states & (uint64_t)0x3FF) | (tmp & ~((uint64_t)0x3FF));
    }

    new_msg = true;
}

void print_uint64_in_binary(uint64_t number)
{
    for (int i = 63; i >= 0; i--)
    {
        uint64_t mask = 1ULL << i;            // Create a mask for the ith bit
        putchar((number & mask) ? '1' : '0'); // Check the ith bit and print '1' or '0'

        // Optional: add a space every 8 bits for better readability
        if (i == 10 || i == 18)
        {
            putchar(' ');
        }

        if (i > 18 && i % 3 == 0)
        {
            putchar(' ');
        }
    }
}

int main()
{
    // Initialize the Pico SDK
    stdio_init_all();

    /* NeoPixel setup */
    ws2812_setup(&ws2812, NEO_PIN, NEO_COUNT);
    ws2812_set_all(&ws2812, 0, 0, 0);
    set_neo_range(&ws2812, 45, 62, BLUE);

    ws2812_show(&ws2812);

    /* CAN setup */
    can_setup(CAN_PIO_NUM, CAN_PIO_IRQ, CAN_SYS_CLOCK, CAN_RX_PIN, CAN_TX_PIN, can_rx_cb);

    /* Button LED setup */
    gpio_init(MAIN_LED_PIN);
    gpio_init(DROGUE_LED_PIN);
    gpio_init(CANSAT_LED_PIN);
    gpio_init(DOOR_LED_PIN);
    gpio_init(RES_1_LED_PIN);
    gpio_init(RES_2_LED_PIN);
    gpio_init(RES_3_LED_PIN);
    gpio_init(RES_4_LED_PIN);

    gpio_set_dir(MAIN_LED_PIN, GPIO_OUT);
    gpio_set_dir(DROGUE_LED_PIN, GPIO_OUT);
    gpio_set_dir(CANSAT_LED_PIN, GPIO_OUT);
    gpio_set_dir(DOOR_LED_PIN, GPIO_OUT);
    gpio_set_dir(RES_1_LED_PIN, GPIO_OUT);
    gpio_set_dir(RES_2_LED_PIN, GPIO_OUT);
    gpio_set_dir(RES_3_LED_PIN, GPIO_OUT);
    gpio_set_dir(RES_4_LED_PIN, GPIO_OUT);

    /* Button setup */
    gpio_init(RBF_BTN_PIN);
    gpio_init(RBF_KEY_BTN_PIN);
    gpio_init(MAIN_BTN_PIN);
    gpio_init(DROGUE_BTN_PIN);
    gpio_init(CANSAT_BTN_PIN);
    gpio_init(DOOR_BTN_PIN);
    gpio_init(RES_1_BTN_PIN);
    gpio_init(RES_2_BTN_PIN);
    gpio_init(RES_3_BTN_PIN);
    gpio_init(RES_4_BTN_PIN);

    gpio_set_dir(RBF_BTN_PIN, GPIO_IN);
    gpio_set_dir(RBF_KEY_BTN_PIN, GPIO_IN);
    gpio_set_dir(MAIN_BTN_PIN, GPIO_IN);
    gpio_set_dir(DROGUE_BTN_PIN, GPIO_IN);
    gpio_set_dir(CANSAT_BTN_PIN, GPIO_IN);
    gpio_set_dir(DOOR_BTN_PIN, GPIO_IN);
    gpio_set_dir(RES_1_BTN_PIN, GPIO_IN);
    gpio_set_dir(RES_2_BTN_PIN, GPIO_IN);
    gpio_set_dir(RES_3_BTN_PIN, GPIO_IN);
    gpio_set_dir(RES_4_BTN_PIN, GPIO_IN);

    gpio_pull_up(RBF_BTN_PIN);
    gpio_pull_up(RBF_KEY_BTN_PIN);
    gpio_pull_up(MAIN_BTN_PIN);
    gpio_pull_up(DROGUE_BTN_PIN);
    gpio_pull_up(CANSAT_BTN_PIN);
    gpio_pull_up(DOOR_BTN_PIN);
    gpio_pull_up(RES_1_BTN_PIN);
    gpio_pull_up(RES_2_BTN_PIN);
    gpio_pull_up(RES_3_BTN_PIN);
    gpio_pull_up(RES_4_BTN_PIN);

    // Set up interrupts for all button pins
    // gpio_set_irq_enabled_with_callback(RBF_BTN_PIN, GPIO_IRQ_EDGE_FALL, true, &button_interrupt_handler);
    // gpio_set_irq_enabled_with_callback(RBF_KEY_BTN_PIN, GPIO_IRQ_EDGE_FALL, true, &button_interrupt_handler);
    // gpio_set_irq_enabled_with_callback(MAIN_BTN_PIN, GPIO_IRQ_EDGE_FALL, true, &button_interrupt_handler);
    // gpio_set_irq_enabled_with_callback(DROGUE_BTN_PIN, GPIO_IRQ_EDGE_FALL, true, &button_interrupt_handler);
    // gpio_set_irq_enabled_with_callback(CANSAT_BTN_PIN, GPIO_IRQ_EDGE_FALL, true, &button_interrupt_handler);
    // gpio_set_irq_enabled_with_callback(DOOR_BTN_PIN, GPIO_IRQ_EDGE_FALL, true, &button_interrupt_handler);
    // gpio_set_irq_enabled_with_callback(RES_1_BTN_PIN, GPIO_IRQ_EDGE_FALL, true, &button_interrupt_handler);
    // gpio_set_irq_enabled_with_callback(RES_2_BTN_PIN, GPIO_IRQ_EDGE_FALL, true, &button_interrupt_handler);
    // gpio_set_irq_enabled_with_callback(RES_3_BTN_PIN, GPIO_IRQ_EDGE_FALL, true, &button_interrupt_handler);
    // gpio_set_irq_enabled_with_callback(RES_4_BTN_PIN, GPIO_IRQ_EDGE_FALL, true, &button_interrupt_handler);

    msg.type = CAN_MESSAGE_PANEL;
    states = 0;

    absolute_time_t current_time = get_absolute_time();
    absolute_time_t last_msg_send_time = get_absolute_time();

    while (true)
    {
        current_time = get_absolute_time();

        if (gpio_get(RBF_BTN_PIN) == 0)
            states |= (1 << RBF_BTN_SHIFT);
        else
            states &= ~(1 << RBF_BTN_SHIFT);

        if (gpio_get(RBF_KEY_BTN_PIN) == 0)
            states |= (1 << RBF_KEY_BTN_SHIFT);
        else
            states &= ~(1 << RBF_KEY_BTN_SHIFT);

        if (gpio_get(MAIN_BTN_PIN) == 0 && gpio_get(MAIN_LED_PIN))
            states |= (1 << MAIN_BTN_SHIFT);

        if (gpio_get(DROGUE_BTN_PIN) == 0 && gpio_get(DROGUE_LED_PIN))
            states |= (1 << DROGUE_BTN_SHIFT);

        if (gpio_get(CANSAT_BTN_PIN) == 0 && gpio_get(CANSAT_LED_PIN))
            states |= (1 << CANSAT_BTN_SHIFT);

        if (gpio_get(DOOR_BTN_PIN) == 0 && gpio_get(DOOR_LED_PIN))
            states |= (1 << DOOR_BTN_SHIFT);

        if (gpio_get(RES_1_BTN_PIN) == 0)
            states |= (1 << RES_1_BTN_SHIFT);

        if (gpio_get(RES_2_BTN_PIN) == 0)
            states |= (1 << RES_2_BTN_SHIFT);

        if (gpio_get(RES_3_BTN_PIN) == 0)
            states |= (1 << RES_3_BTN_SHIFT);

        if (gpio_get(RES_4_BTN_PIN) == 0)
        {
            states &= ~(((1 << 8) - 1) << 2);
            states |= (1 << RES_4_BTN_SHIFT);
        }
        else
            states &= ~(1 << RES_4_BTN_SHIFT);

        /*
        if (btn_pressed)
        {
            printf("Button pressed: %d\n", pressed_btn_gpio);
            switch (pressed_btn_gpio)
            {
            case RBF_BTN_PIN:
                states |= (1 << RBF_BTN_SHIFT);
                break;
            case RBF_KEY_BTN_PIN:
                states |= (1 << RBF_KEY_BTN_SHIFT);
                break;
            case MAIN_BTN_PIN:
                states |= (1 << MAIN_BTN_SHIFT);
                break;
            case DROGUE_BTN_PIN:
                states |= (1 << DROGUE_BTN_SHIFT);
                break;
            case CANSAT_BTN_PIN:
                states |= (1 << CANSAT_BTN_SHIFT);
                break;
            case DOOR_BTN_PIN:
                states |= (1 << DOOR_BTN_SHIFT);
                break;
            case RES_1_BTN_PIN:
                states |= (1 << RES_1_BTN_SHIFT);
                break;
            case RES_2_BTN_PIN:
                states |= (1 << RES_2_BTN_SHIFT);
                break;
            case RES_3_BTN_PIN:
                states |= (1 << RES_3_BTN_SHIFT);
                break;
            case RES_4_BTN_PIN:
                states |= (1 << RES_4_BTN_SHIFT);
                break;
            default:
                perror("Unknown button pressed");
            }
            btn_pressed = false;
        }
        */

        if (new_msg && (absolute_time_diff_us(last_neo_update_time, current_time) >= (1e6 / 20)))
        {
            printf("states = ");
            print_uint64_in_binary(states);
            printf("\n");
            gpio_put(MAIN_LED_PIN, (states >> MAIN_LED_SHIFT) & 0x01);
            gpio_put(DROGUE_LED_PIN, (states >> DROGUE_LED_SHIFT) & 0x01);
            gpio_put(CANSAT_LED_PIN, (states >> CANSAT_LED_SHIFT) & 0x01);
            gpio_put(DOOR_LED_PIN, (states >> DOOR_LED_SHIFT) & 0x01);
            gpio_put(RES_1_LED_PIN, (states >> RES_1_LED_SHIFT) & 0x01);
            gpio_put(RES_2_LED_PIN, (states >> RES_2_LED_SHIFT) & 0x01);
            gpio_put(RES_3_LED_PIN, (states >> RES_3_LED_SHIFT) & 0x01);
            gpio_put(RES_4_LED_PIN, (states >> RES_4_LED_SHIFT) & 0x01);

            set_neo_range(&ws2812, 0, 2, (states >> RBF_NEO_SHIFT) & 0x07);
            set_neo_range(&ws2812, 3, 5, (states >> MAIN_NEO_SHIFT) & 0x07);
            set_neo_range(&ws2812, 6, 8, (states >> DROGUE_NEO_SHIFT) & 0x07);
            set_neo_range(&ws2812, 9, 11, (states >> ROCKETS_COMMS_NEO_SHIFT) & 0x07);
            set_neo_range(&ws2812, 12, 14, (states >> DOORS_OPEN_NEO_SHIFT) & 0x07);
            set_neo_range(&ws2812, 15, 17, (states >> RADIO_RECORD_NEO_SHIFT) & 0x07);
            set_neo_range(&ws2812, 18, 20, (states >> READY_TO_LAUNCH_NEO_SHIFT) & 0x07);
            set_neo_range(&ws2812, 21, 23, (states >> RES_4_NEO_SHIFT) & 0x07);
            set_neo_range(&ws2812, 24, 26, (states >> RES_3_NEO_SHIFT) & 0x07);
            set_neo_range(&ws2812, 27, 29, (states >> RES_2_NEO_SHIFT) & 0x07);
            set_neo_range(&ws2812, 30, 32, (states >> RES_1_NEO_SHIFT) & 0x07);
            set_neo_range(&ws2812, 33, 35, (states >> DOOR_2_NEO_SHIFT) & 0x07);
            set_neo_range(&ws2812, 36, 38, (states >> DOOR_1_NEO_SHIFT) & 0x07);
            set_neo_range(&ws2812, 39, 41, (states >> CANSAT_3_4_NEO_SHIFT) & 0x07);
            set_neo_range(&ws2812, 42, 44, (states >> CANSAT_1_2_NEO_SHIFT) & 0x07);
            ws2812_show(&ws2812);
            new_msg = false;
            last_neo_update_time = current_time;
        }

        if (absolute_time_diff_us(last_msg_send_time, current_time) > (1e6 / CAN_MSG_FREQ))
        {
            memcpy(msg.data.panel.states, &states, sizeof(struct can_message_panel_s));
            can_transmit(&msg);
            last_msg_send_time = current_time;
        }

        sleep_us(100);
    }
}
