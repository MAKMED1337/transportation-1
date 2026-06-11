#include "graph/reverse_graph.hpp"

#include <cstddef>
#include <cstdint>
#include <vector>

namespace transport {

ReverseAdjacency build_reverse_adjacency(const Graph &graph) {
    const VertexId V = graph.vertex_count();
    const uint64_t E = graph.edge_count();

    // Count in-degree of each vertex (= out-degree in reverse graph)
    std::vector<uint64_t> in_deg(static_cast<size_t>(V) + 1, 0);
    for (const Edge &e : graph.edges) {
        ++in_deg[static_cast<size_t>(e.to) + 1];
    }

    // Prefix-sum to get offsets
    for (size_t i = 1; i <= static_cast<size_t>(V); ++i) {
        in_deg[i] += in_deg[i - 1];
    }

    ReverseAdjacency rev;
    rev.offsets = in_deg; // copy; in_deg now used as fill cursor
    rev.edges.resize(static_cast<size_t>(E));
    rev.edge_id.resize(static_cast<size_t>(E));

    // Scatter: for each original arc (u→v, w) at global index idx, write into rev's adjacency of v
    for (VertexId u = 0; u < V; ++u) {
        const uint64_t begin = graph.offsets[u];
        const uint64_t end = graph.offsets[u + 1];
        for (uint64_t i = begin; i < end; ++i) {
            const Edge &e = graph.edges[static_cast<size_t>(i)];
            const size_t slot = static_cast<size_t>(in_deg[static_cast<size_t>(e.to)]++);
            rev.edges[slot] = Edge{u, e.weight};
            rev.edge_id[slot] = i;
        }
    }

    return rev;
}

} // namespace transport
