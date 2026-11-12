# Dijkstra evacuation module (prototype)

Standalone ESP-IDF/C module ported from `simulering.py`. The existing mesh code in `main.c` and `mesh_tasks.c` is unchanged.

## API
```c
#include "evacuation.h"

evac_node_state_t nodes[EVAC_NUM_NODES] = {0};
nodes[1].status = EVAC_NODE_FIRE;
nodes[6].people_count = 20;

evacuation_result_t r = evacuation_calculate_route(0, nodes);
if (r.route_found) {
    // r.next_node: next adjacent waypoint
    // r.target_exit: selected exit (15, 16 or 17)
    // r.total_cost: weighted routing cost, not metres
} else {
    // no traversable route
}
```

## Model
- Same 18-node bidirectional graph and exits 15/16/17 as `simulering.py`.
- Fire and offline nodes are not traversable.
- Entering any immediate neighbour of a fire node adds **15 cost units** once, even if multiple fire nodes are adjacent.
- Entering a node adds **0.5 × its people count**.
- Minimum cost wins; ties prefer fewer people along the route, then the lower exit ID.
- No route: `route_found=false`, `next_node=-1`, `target_exit=-1`.
- Starting at an exit returns that exit as the next node, with cost zero.

## Suggested validation cases
| Case | Start | Fire | Offline | People | Expected |
|---|---:|---|---|---|---|
| Normal | 0 | none | none | none | Exit 16, next 4, cost 25 |
| Chemical lab fire | 0 | 1 | none | none | Exit 15, next 3, cost 28 |
| West/Middle unavailable | 0 | 1 | 3,4 | none | Exit 17, next 5, cost 30 |
| All starting neighbours burning | 0 | 3,4,5 | none | none | No route |

**Integration note:** This module does not yet subscribe to mesh messages. The caller must maintain a synchronized snapshot of the 18 node states and call `evacuation_calculate_route` on updates. `src/CMakeLists.txt` already includes all `src/*.*` files via glob.

**Safety:** Research prototype only. A soft buffer penalty may still route people next to fire when other paths are unavailable. Not suitable for actual life-safety guidance without extensive validation, sensors, reliability measures and regulatory assessment.
