#pragma once

#include "graph/types.hpp"

namespace transport {

// Min-heap entry for Dijkstra-family priority queues: ordered by ascending key
// (use with std::greater<> so the smallest key is popped first).
struct HeapNode {
    Distance key = 0;
    VertexId v = 0;
    bool operator>(const HeapNode &other) const { return key > other.key; }
};

} // namespace transport
