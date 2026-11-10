#ifndef MESH_MESSAGES_H
#define MESH_MESSAGES_H

#include <stdint.h>

typedef enum
{
    MSG_HEARTBEAT = 0,
    MSG_ALARM_ON,
    MSG_ALARM_OFF,
    MSG_ACK
} message_type_t;

typedef struct
{
    uint8_t node_id;
    message_type_t msg_type;
     int64_t timestamp;
} mesh_message_t;

#endif