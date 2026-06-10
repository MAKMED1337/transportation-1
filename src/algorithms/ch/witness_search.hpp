#pragma once

#include "algorithms/ch/work_graph.hpp"
#include "algorithms/stamped_vector.hpp"

#include <cstddef>
#include <cstdint>
#include <limits>

namespace transport::ch {

using transport::VertexId;
using transport::Distance;

// Sentinel returned by WitnessSearch::run() when no witness path exists within the search bounds.
constexpr Distance kInf = std::numeric_limits<Distance>::max();

// Maximum number of edges a witness path may use. A fixed hop limit keeps every local search cheap;
// missing a longer witness only causes an extra (still correct) shortcut to be added.
constexpr uint32_t kWitnessHopLimit = 5;

// Bounded local Dijkstra used to decide whether contracting a node requires a shortcut: it looks for
// an alternative path (the "witness") that does not pass through the node being contracted.
class WitnessSearch {
public:
    explicit WitnessSearch(size_t vertices);

    // Shortest distance source->target that avoids `forbidden` and already-contracted nodes, exploring
    // only paths of weight <= max_distance and at most `hop_limit` edges. Returns kInf when no such
    // witness exists within the bounds.
    [[nodiscard]] Distance run(const WorkGraph &graph, VertexId source, VertexId target, VertexId forbidden,
                               Distance max_distance, uint32_t hop_limit);

private:
    // Per-vertex search state. Distance and hop count are always written together, so they live in one
    // cell and one StampedVector covers both with a single stamp array.
    struct Cell {
        Distance dist;
        uint32_t hops;
    };

    // Stamped scratch reused across runs so each run resets in O(1) instead of clearing the whole vector.
    StampedVector<Cell> table_;
};

} // namespace transport::ch
