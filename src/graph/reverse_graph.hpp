#pragma once

#include "graph/graph.hpp"

#include <cstdint>
#include <vector>

namespace transport {

// CSR of the transposed graph (every original arc (u→v,w) becomes (v→u,w) here).
// `edge_id[i]` = original edge index in graph.edges for reverse_edges[i], allowing flag arrays indexed
// by original edge index to be tested during backward search.
struct ReverseAdjacency {
    std::vector<uint64_t> offsets;    // V+1
    std::vector<Edge> edges;          // .to = original tail, .weight = same
    std::vector<uint64_t> edge_id;    // parallel to edges: index into original graph.edges
};

[[nodiscard]] ReverseAdjacency build_reverse_adjacency(const Graph &graph);

} // namespace transport
