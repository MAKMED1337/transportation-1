#pragma once

#include "graph/graph.hpp"

#include <cstdint>
#include <string>
#include <vector>

namespace transport {

// Assigns a region id in [0, regions) to each vertex using the requested method.
//   "grid"     — divide lat/lon bounding box into a sqrt(regions) × sqrt(regions) uniform grid
//   "inertial" — binary kd-split alternating lon/lat, log2(regions) levels (regions must be a power of 2)
// Returns a vector of length vertex_count.
[[nodiscard]] std::vector<uint16_t> build_partition(const Graph &graph, uint32_t regions, const std::string &method);

} // namespace transport
