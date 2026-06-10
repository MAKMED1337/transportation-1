#pragma once

#include "graph/types.hpp"

#include <limits>

namespace transport {

constexpr Distance kUnreachable = std::numeric_limits<Distance>::max();

struct PathResult {
    Distance distance_units = kUnreachable;
    uint32_t settled = 0;
};

} // namespace transport
