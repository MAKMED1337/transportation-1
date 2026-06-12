#pragma once

#include "graph/graph.hpp"
#include "graph/reverse_graph.hpp"
#include "graph/types.hpp"

#include <cstdint>
#include <string>
#include <vector>

namespace transport {
namespace alt {

// All per-vertex distances are uint32_t (Poland max road distance < 2^32 scaled units).
constexpr uint32_t kLandmarkInf = std::numeric_limits<uint32_t>::max();

enum class LandmarkStrategy { Random, Farthest, Planar, Avoid };

struct LandmarkSet {
    std::vector<VertexId> landmarks; // k selected landmark vertices
    std::vector<uint32_t> dist_from; // [l * V + v] = d(landmark[l], v)
    std::vector<uint32_t> dist_to;   // [l * V + v] = d(v, landmark[l])  (via reverse graph)
    uint32_t vertex_count = 0;
    std::string strategy_name;
};

// Run a one-to-all Dijkstra from `source` on `graph`, storing uint32 distances into `out`
// (laid out as `out[v]`). `out` must have size `graph.vertex_count()`.
void one_to_all(const Graph &graph, VertexId source, std::vector<uint32_t> &out);

// Run a one-to-all Dijkstra from `source` on `reverse`, storing uint32 distances into `out`.
void one_to_all_reverse(const ReverseAdjacency &reverse, VertexId source, std::vector<uint32_t> &out);

// Select k landmarks using the given strategy and compute all distance tables.
// `seed` is used for random/farthest strategies to choose the initial vertex.
LandmarkSet build_landmarks(const Graph &graph, const ReverseAdjacency &reverse, uint32_t k, LandmarkStrategy strategy,
                            uint32_t seed);

} // namespace alt
} // namespace transport
