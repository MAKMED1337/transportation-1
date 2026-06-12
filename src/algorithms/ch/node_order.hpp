#pragma once

#include "algorithms/ch/witness_search.hpp"
#include "algorithms/ch/work_graph.hpp"

#include <cstdint>
#include <vector>

namespace transport::ch {

using transport::VertexId;
using transport::Weight;

// A shortcut arc that would replace a path through the contracted node.
struct Shortcut {
    VertexId from;
    VertexId to;
    Weight weight;
    uint32_t originals = 2; // base-graph edges covered by this shortcut
};

// One entry in the staged-hop-limit schedule: when remaining_arcs/remaining_vertices >= threshold,
// switch the witness search to `hop_limit`.
struct HopStage {
    double mean_degree_threshold;
    uint32_t hop_limit;
};

// Configurable contraction ordering parameters. All six terms contribute to the priority function
// priority(v) = w_E*E + w_D*D + w_Q*Q + w_O*O + w_S*S; lower priority is contracted earlier.
//   E = edge difference (shortcuts added minus edges removed)
//   D = number of already-contracted neighbors
//   Q = depth in the contraction tree (max depth of contracted neighbors + 1)
//   O = original-edge difference (sum of originals of shortcuts minus originals of removed arcs)
//   S = witness search settled count during simulated contraction
//   V = Voronoi region size term (not implemented; weight should stay 0)
struct OrderParams {
    int32_t w_edge_diff = 4;
    int32_t w_deleted_neighbors = 2;
    int32_t w_depth = 1;
    int32_t w_original_edges = 1;
    int32_t w_search_space = 0;
    int32_t w_voronoi = 0;
    std::vector<HopStage> hop_stages{{0.0, 2}, {3.3, 3}, {10.0, 5}};
};

// Shortcuts that contracting `vertex` would introduce. Pure — does not modify the graph, doubles as
// the "simulated contraction" used for ordering.
[[nodiscard]] std::vector<Shortcut> find_shortcuts(const WorkGraph &graph, VertexId vertex, WitnessSearch &witness,
                                                   uint32_t hop_limit);

// Full multi-term ordering priority. Calls find_shortcuts internally via witness; afterwards,
// witness.sim_settled() holds the S term for this vertex.
[[nodiscard]] int64_t compute_priority(const WorkGraph &graph, VertexId vertex, WitnessSearch &witness,
                                       uint32_t hop_limit, const OrderParams &params,
                                       const std::vector<uint32_t> &depth,
                                       const std::vector<uint32_t> &deleted_neighbors);

} // namespace transport::ch
