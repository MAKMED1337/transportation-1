#pragma once

#include <cstdint>

namespace transport {

// Min-heap entry for Dijkstra-family priority queues: ordered by ascending key
// (use with std::greater<> so the smallest key is popped first).
struct HeapNode {
    uint64_t key = 0;
    uint32_t v = 0;
    bool operator>(const HeapNode &other) const { return key > other.key; }
};

} // namespace transport
