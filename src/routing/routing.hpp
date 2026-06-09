#pragma once

#include "graph/graph.hpp"

#include <cstdint>
#include <limits>

namespace transport {

constexpr uint64_t kUnreachable = std::numeric_limits<uint64_t>::max();

struct PathResult {
    uint64_t distance_units = kUnreachable;
    uint32_t settled = 0;
};

} // namespace transport
