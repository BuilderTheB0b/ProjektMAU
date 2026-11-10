#ifndef MESH_MESSAGES_H
#define MESH_MESSAGES_H

#include <stdint.h>
#include <stdbool.h>

typedef enum
{
    MSG_HEARTBEAT = 0,
    MSG_ALARM_ON,
    MSG_ALARM_OFF,
    MSG_ACK
} message_type_t;

typedef struct {
    uint32_t node_id;          // Avsändarens nod-ID
    message_type_t msg_type;   // Meddelandetyp
    int64_t timestamp;         // Tidsstämpel för latensmätning
    bool alarm_active;         // True om larm är aktivt
    uint16_t person_count;     // Antal personer i zonen
} mesh_message_t;

#endif