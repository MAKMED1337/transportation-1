#include "algorithms/partition.hpp"

#include <algorithm>
#include <cmath>
#include <cstdint>
#include <stdexcept>
#include <string>
#include <vector>

namespace transport {
namespace {

std::vector<uint16_t> partition_grid(const Graph &graph, uint32_t regions) {
    const uint32_t V = graph.vertex_count();
    // Approximate square grid: cols × rows where cols*rows >= regions, cols≈rows.
    const uint32_t side = static_cast<uint32_t>(std::ceil(std::sqrt(static_cast<double>(regions))));
    const uint32_t cols = side;
    const uint32_t rows = (regions + cols - 1) / cols;

    double lat_min = 1e18, lat_max = -1e18, lon_min = 1e18, lon_max = -1e18;
    for (const NodeCoord &c : graph.coords) {
        if (c.lat < lat_min) {
            lat_min = c.lat;
        }
        if (c.lat > lat_max) {
            lat_max = c.lat;
        }
        if (c.lon < lon_min) {
            lon_min = c.lon;
        }
        if (c.lon > lon_max) {
            lon_max = c.lon;
        }
    }
    const double lat_range = std::max(lat_max - lat_min, 1e-9);
    const double lon_range = std::max(lon_max - lon_min, 1e-9);

    std::vector<uint16_t> result(V);
    for (VertexId v = 0; v < V; ++v) {
        const auto r = static_cast<uint32_t>(std::min(
            (graph.coords[v].lat - lat_min) / lat_range * static_cast<double>(rows), static_cast<double>(rows - 1)));
        const auto c = static_cast<uint32_t>(std::min(
            (graph.coords[v].lon - lon_min) / lon_range * static_cast<double>(cols), static_cast<double>(cols - 1)));
        const uint32_t region = r * cols + c;
        result[v] = static_cast<uint16_t>(std::min(region, regions - 1));
    }
    return result;
}

// Recursively assign region ids to vertices by kd-splitting [lo, hi) on the given dimension.
// `indices` is the working set; `region_ids` stores the result; `depth` is the current recursion level.
void kd_split(const Graph &graph, std::vector<VertexId> &indices, std::vector<uint16_t> &region_ids, uint32_t base,
              uint32_t count, uint32_t depth) {
    if (count == 0) {
        return;
    }
    if (count == 1) {
        for (const VertexId v : indices) {
            region_ids[v] = static_cast<uint16_t>(base);
        }
        return;
    }

    // Alternate split dimension: even depth = longitude, odd depth = latitude.
    const bool split_lon = (depth % 2 == 0);
    std::sort(indices.begin(), indices.end(), [&](VertexId a, VertexId b) {
        return split_lon ? graph.coords[a].lon < graph.coords[b].lon : graph.coords[a].lat < graph.coords[b].lat;
    });

    const uint32_t half_left = count / 2;
    const uint32_t half_right = count - half_left;
    const size_t n = indices.size();
    const size_t split = n / 2;

    std::vector<VertexId> left(indices.begin(), indices.begin() + static_cast<ptrdiff_t>(split));
    std::vector<VertexId> right(indices.begin() + static_cast<ptrdiff_t>(split), indices.end());

    kd_split(graph, left, region_ids, base, half_left, depth + 1);
    kd_split(graph, right, region_ids, base + half_left, half_right, depth + 1);
}

std::vector<uint16_t> partition_inertial(const Graph &graph, uint32_t regions) {
    // Check regions is a power of 2.
    if (regions == 0 || (regions & (regions - 1)) != 0) {
        throw std::invalid_argument("inertial partition requires regions to be a power of 2");
    }
    const uint32_t V = graph.vertex_count();
    std::vector<uint16_t> result(V, 0);
    std::vector<VertexId> all(V);
    for (VertexId v = 0; v < V; ++v) {
        all[v] = v;
    }
    kd_split(graph, all, result, 0, regions, 0);
    return result;
}

} // namespace

std::vector<uint16_t> build_partition(const Graph &graph, uint32_t regions, const std::string &method) {
    if (regions == 0 || regions > 65535) {
        throw std::invalid_argument("regions must be in [1, 65535]");
    }
    if (method == "grid") {
        return partition_grid(graph, regions);
    }
    if (method == "inertial") {
        return partition_inertial(graph, regions);
    }
    throw std::invalid_argument("unknown partition method: " + method);
}

} // namespace transport
