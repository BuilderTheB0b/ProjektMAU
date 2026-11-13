#include "evacuation.h"

#include <float.h>
#include <stddef.h>

#define INF_COST (FLT_MAX / 4.0f)
#define NO_NODE (-1)
#define EPSILON (0.0001f)

typedef struct {
    uint8_t from;
    uint8_t to;
    uint8_t metres;
} edge_t;


static const edge_t edges[] = {
    /*
    {0,3,10}, {0,4,8}, {0,5,12}, {1,4,4}, {2,5,5},
    {3,4,8}, {4,5,8}, {6,7,8}, {7,8,8},
    {3,6,8}, {4,7,6}, {5,8,8},
    {9,6,5}, {10,7,5}, {11,8,5},
    {6,12,6}, {7,13,6}, {8,14,6},
    {12,15,4}, {13,16,5}, {14,17,4},
    */
    {0, 1, 10}, {0, 2, 8}
};




//Utgångar hårdkodade i testningssyfte
static const int exits[] = {1, 2};

//static const int exits[] = {15,16,17};

static float base[EVAC_NUM_NODES][EVAC_NUM_NODES];
static bool topology_initialized = false;


static void init_topology(void)
{
    if (topology_initialized) return;
    for (int i = 0; i < EVAC_NUM_NODES; ++i) {
        for (int j = 0; j < EVAC_NUM_NODES; ++j) {
            base[i][j] = (i == j) ? 0.0f : INF_COST;
        }
    }
    // Går igenom static const edge_t edges och skapar förbindelse
    for (size_t i = 0; i < sizeof(edges)/sizeof(edges[0]); ++i) {
        base[edges[i].from][edges[i].to] = (float)edges[i].metres;
        base[edges[i].to][edges[i].from] = (float)edges[i].metres;
    }
    topology_initialized = true;
}

// Kollar om en nod får användsa 
static bool is_blocked(const evac_node_state_t *states, int node)
{
    return states[node].status == EVAC_NODE_FIRE ||
           states[node].status == EVAC_NODE_OFFLINE;
}


evacuation_result_t evacuation_calculate_route(
    int start_node, 
    const evac_node_state_t states[EVAC_NUM_NODES])
{
    // Behåller alltid dessa värden om ingen väg hittas
    evacuation_result_t result = {
        .route_found = false, 
        .next_node = NO_NODE,
        .target_exit = NO_NODE, 
        .total_cost = INF_COST
    };


    if (states == NULL || start_node < 0 || start_node >= EVAC_NUM_NODES) {
        return result;
    }


    init_topology();
    if (is_blocked(states, start_node)) 
    return result;


    bool buffer[EVAC_NUM_NODES] = {false};


    // Går genom alla noder och kollar efter brand
    for (int fire = 0; fire < EVAC_NUM_NODES; ++fire) {
        // om ingen brand kör nästa 
        if (states[fire].status != EVAC_NODE_FIRE) continue;
        // Om noden brinner körs detta 
        for (int node = 0; node < EVAC_NUM_NODES; ++node) {
            if (node != fire && base[fire][node] < INF_COST) {
                buffer[node] = true;
            }
        }
    }

    //Lagrar den lägsta hittade kostnaden
    // från startnoden till varje annan nod
    float dist[EVAC_NUM_NODES];

    // Sammanlagda personantalet längs 
    // den aktuella bästa vägen
    uint32_t occupancy[EVAC_NUM_NODES];


    int previous[EVAC_NUM_NODES];
    // Noder som har behandlats färdigt av Dijkstra
    bool visited[EVAC_NUM_NODES] = {false};
    // Återsäller allt innan varje beräkning
    for (int i = 0; i < EVAC_NUM_NODES; ++i) {
        dist[i] = INF_COST;
        occupancy[i] = UINT32_MAX;
        previous[i] = NO_NODE;
    }

    dist[start_node] = 0.0f;
    occupancy[start_node] = states[start_node].people_count;


    for (int iteration = 0; iteration < EVAC_NUM_NODES; ++iteration) {
        int u = NO_NODE;
        for (int v = 0; v < EVAC_NUM_NODES; ++v) {
            if (visited[v] || dist[v] >= INF_COST) continue;
            if (u == NO_NODE || dist[v] < dist[u] - EPSILON ||
                ((dist[v] - dist[u] < EPSILON) &&
                 (occupancy[v] < occupancy[u] ||
                  (occupancy[v] == occupancy[u] && v < u)))) {
                u = v;
            }
        }

        if (u == NO_NODE) break;
        visited[u] = true;

        // Undersöker grannnoder
        for (int v = 0; v < EVAC_NUM_NODES; ++v) {
            if (visited[v] || v == u || is_blocked(states, v) ||
                base[u][v] >= INF_COST) continue;

            float cost = base[u][v] +
                (buffer[v] ? EVAC_BUFFER_PENALTY : 0.0f) +
                EVAC_PEOPLE_FACTOR * (float)states[v].people_count;
            float candidate = dist[u] + cost;
            uint32_t candidate_occupancy =
                occupancy[u] + states[v].people_count;
            if (candidate < dist[v] - EPSILON ||
                ((candidate - dist[v] < EPSILON) &&
                 candidate_occupancy < occupancy[v])) {
                dist[v] = candidate;
                occupancy[v] = candidate_occupancy;
                previous[v] = u;
            }
        }
    }

    int best = NO_NODE;
    for (size_t i = 0; i < sizeof(exits)/sizeof(exits[0]); ++i) {
        int e = exits[i];
        if (dist[e] >= INF_COST) continue;
        if (best == NO_NODE || dist[e] < dist[best] - EPSILON ||
            ((dist[e] - dist[best] < EPSILON) &&
             (occupancy[e] < occupancy[best] ||
              (occupancy[e] == occupancy[best] && e < best)))) {
            best = e;
        }
    }
    if (best == NO_NODE) return result;

    int next = best;
    while (next != start_node && previous[next] != start_node) {
        if (previous[next] == NO_NODE) return result;
        next = previous[next];
    }
    result.route_found = true;
    result.next_node = next;
    result.target_exit = best;
    result.total_cost = dist[best];
    return result;
}
