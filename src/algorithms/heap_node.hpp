#pragma once

#include "graph/types.hpp"

#include <cstdint>

namespace transport {

// Min-heap entry for Dijkstra-family priority queues: ordered by ascending key
// (use with std::greater<> so the smallest key is popped first).
struct HeapNode {
    Distance key = 0;
    VertexId v = 0;
    bool operator>(const HeapNode &other) const { return key > other.key; }
};

// Signed key variant for bidirectional A* (consistent potentials can produce negative keys).
struct SignedHeapNode {
    int64_t key = 0;
    VertexId v = 0;
    bool operator>(const SignedHeapNode &other) const { return key > other.key; }
};

} // namespace transport
