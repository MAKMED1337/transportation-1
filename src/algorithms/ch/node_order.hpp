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
};

// The shortcuts that contracting `vertex` would introduce: for every uncontracted incoming/outgoing
// pair whose only path of weight <= sum goes through `vertex` (no witness found). Pure — does not
// modify the graph, so it doubles as the "simulated contraction" used for ordering.
[[nodiscard]] std::vector<Shortcut> find_shortcuts(const WorkGraph &graph, VertexId vertex, WitnessSearch &witness,
                                                   uint32_t hop_limit);

// Edge difference priority: (shortcuts that would be added) - (uncontracted edges that would be
// removed) by contracting `vertex`. Lower is contracted earlier; keeps the graph sparse.
[[nodiscard]] int64_t edge_difference(const WorkGraph &graph, VertexId vertex, WitnessSearch &witness,
                                      uint32_t hop_limit);

} // namespace transport::ch
