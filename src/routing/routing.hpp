#pragma once

#include "graph/types.hpp"

#include <limits>

namespace transport {

constexpr Distance kUnreachable = std::numeric_limits<Distance>::max();

struct QueryStats {
    uint32_t settled = 0;
    uint32_t settled_forward = 0; // bidirectional algorithms; 0 for unidirectional
    uint32_t settled_backward = 0;
    uint64_t relaxed_arcs = 0; // arcs whose relaxation was attempted
    uint64_t heap_pushes = 0;
    uint64_t heuristic_evals = 0; // A*/ALT potential evaluations
    uint64_t pruned_by_flag = 0;  // arc-flags/CHASE: arcs skipped by flag test
    uint64_t table_lookups = 0;   // TNR DT lookups / HL intersection probes
    bool used_fallback = false;   // TNR/HL: fell back to CH
};

struct PathResult {
    Distance distance_units = kUnreachable;
    QueryStats stats;
};

} // namespace transport
