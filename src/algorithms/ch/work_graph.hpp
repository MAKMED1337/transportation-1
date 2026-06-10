#pragma once

#include "graph/graph.hpp"

#include <cstddef>
#include <cstdint>
#include <vector>

namespace transport::ch {

using transport::VertexId;
using transport::Weight;

// A directed arc in the mutable graph used during contraction. Unlike the immutable CSR Edge, this
// adjacency keeps both out- and in-arcs so shortcuts can be inserted as nodes are contracted.
struct WorkArc {
    VertexId to = 0;
    Weight weight = 0;
};

struct WorkGraph {
    explicit WorkGraph(size_t vertices) : out(vertices), in(vertices), contracted(vertices, 0) {}
    explicit WorkGraph(const Graph &graph);

    std::vector<std::vector<WorkArc>> out;
    std::vector<std::vector<WorkArc>> in;
    std::vector<uint8_t> contracted;

    [[nodiscard]] VertexId vertex_count() const { return static_cast<VertexId>(out.size()); }

    // Inserts arc from->to, or relaxes the existing one to the smaller weight. Keeps out[] and in[]
    // in sync.
    void add_or_relax(VertexId from, VertexId to, Weight weight);

    // Arcs whose endpoint has not yet been contracted (the only ones relevant to future shortcuts).
    [[nodiscard]] std::vector<WorkArc> uncontracted_in(VertexId v) const;
    [[nodiscard]] std::vector<WorkArc> uncontracted_out(VertexId v) const;
};

} // namespace transport::ch
