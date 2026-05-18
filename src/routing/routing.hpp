#pragma once

#include "graph/graph.hpp"

#include <cstdint>
namespace transport {

struct PathResult {
    uint64_t distance_units = UINT64_MAX;
    uint32_t settled = 0;
};

} // namespace transport
