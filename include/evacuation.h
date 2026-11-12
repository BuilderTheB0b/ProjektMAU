#ifndef EVACUATION_H
#define EVACUATION_H

#include <stdbool.h>
#include <stdint.h>

#define EVAC_NUM_NODES 18
#define EVAC_BUFFER_PENALTY 15.0f
#define EVAC_PEOPLE_FACTOR 0.5f

typedef enum {
    EVAC_NODE_NORMAL = 0,
    EVAC_NODE_FIRE,
    EVAC_NODE_OFFLINE
} evac_node_status_t;

typedef struct {
    evac_node_status_t status;
    uint16_t people_count;
} evac_node_state_t;

typedef struct {
    bool route_found;
    int next_node;      /* -1 if unavailable; start node if already at exit */
    int target_exit;    /* -1 if unavailable */
    float total_cost;   /* weighted cost, NOT physical metres */
} evacuation_result_t;

/* Uses the fixed 18-node building topology from simulering.py.
   Caller supplies a complete, consistent snapshot of all node states.
   No network I/O or hardware side effects. */
evacuation_result_t evacuation_calculate_route(
    int start_node,
    const evac_node_state_t states[EVAC_NUM_NODES]);

#endif
