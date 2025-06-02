#ifndef _CAN_DEFS_H
#define _CAN_DEFS_H

#include "can_data_structs.h"

/*
name of the message, set in can_message_t type when transmitting
order determines message ID
*/
typedef enum
{
    CAN_MESSAGE_PANEL,
    CAN_MESSAGE_PANEL_W_BTNS,
    /* number of messages */
    CAN_MESSAGE_COUNT,
} can_message_type;

/*
size of the data structures
must be in same order as can_message_type !!
*/
static const uint8_t message_size[] = {
    sizeof(struct can_message_panel_s),
    sizeof(struct can_message_panel_s),
};

/*
message struct/union
union must contain all data structs (order does not matter here)
*/
typedef struct
{
    can_message_type type;
    union
    {
        struct can_message_panel_s panel;
        struct can_message_panel_s panel_w_btns;
        /* universal pointer + max size*/
        uint8_t data[8];
    } data;
} can_message_t;

/*
checks if struct is larger than it should be
CAN limit is 8 bytes (struct also includes 32-bit type so 12 bytes max total)
*/
static_assert(sizeof(can_message_t) <= 12, "message in can_message_t is larger than maximum allowed size of 8 bytes");
/*
checks if sall message names defined in 'can_message_type' have their size set in 'message_size[]'
keep in mind that order of these 2 lists can't be checked and must be verified manually
*/
static_assert(CAN_MESSAGE_COUNT == sizeof(message_size), "not all messages have size defined in 'message_size[]' or message type name is missing in 'can_message_type'");

#endif // can_defs.h