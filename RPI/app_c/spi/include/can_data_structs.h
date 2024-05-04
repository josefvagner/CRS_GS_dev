#ifndef _CAN_DATA_STRUCTS_H
#define _CAN_DATA_STRUCTS_H

struct can_message_neopixel_s
{
    uint8_t id;
    uint8_t pos;
    uint8_t r;
    uint8_t g;
    uint8_t b;
};

struct can_message_button_led_s
{
    uint8_t id;
    uint8_t state;
};

struct can_message_button_s
{
    uint8_t id;
    uint8_t state;
};

#endif // _CAN_DATA_STRUCTS_H