#include "algorithms/bidirectional_dijkstra.hpp"

#include "algorithms/heap_node.hpp"
#include "graph/reverse_graph.hpp"

#include <cstddef>
#include <cstdint>
#include <functional>
#include <limits>
#include <queue>
#include <string_view>

namespace transport {

BidirectionalDijkstraAlgorithm::BidirectionalDijkstraAlgorithm(const Graph &graph)
    : graph_(graph), dist_f_(graph.vertex_count(), kUnreachable), dist_b_(graph.vertex_count(), kUnreachable) {}

std::string_view BidirectionalDijkstraAlgorithm::name() const { return "bidijkstra"; }

std::string BidirectionalDijkstraAlgorithm::variant() const {
    return "alternating by smaller top, stop when top_f + top_b >= mu";
}

void BidirectionalDijkstraAlgorithm::preprocess() { reverse_ = build_reverse_adjacency(graph_); }

PathResult BidirectionalDijkstraAlgorithm::query(VertexId source, VertexId target) const {
    dist_f_.reset();
    dist_b_.reset();
    std::priority_queue<HeapNode, std::vector<HeapNode>, std::greater<>> pq_f, pq_b;

    if (source == target) {
        return PathResult{0, QueryStats{}};
    }

    dist_f_.set(source, 0);
    dist_b_.set(target, 0);
    pq_f.push({0, source});
    pq_b.push({0, target});

    Distance mu = kUnreachable;
    QueryStats stats;

    // Saturating add: avoids overflow when either operand is kUnreachable
    auto sat_add = [](Distance a, Distance b) -> Distance {
        if (a == kUnreachable || b == kUnreachable) {
            return kUnreachable;
        }
        const Distance r = a + b;
        return (r < a) ? kUnreachable : r; // wrapped
    };

    while (!pq_f.empty() || !pq_b.empty()) {
        const Distance top_f = pq_f.empty() ? kUnreachable : pq_f.top().key;
        const Distance top_b = pq_b.empty() ? kUnreachable : pq_b.top().key;

        if (sat_add(top_f, top_b) >= mu) {
            break;
        }

        // Alternate: pick the direction with the smaller top key
        const bool go_forward = top_f <= top_b;

        if (go_forward) {
            const HeapNode top = pq_f.top();
            pq_f.pop();
            if (top.key != dist_f_.get(top.v)) {
                continue;
            }
            ++stats.settled;
            ++stats.settled_forward;

            const uint64_t begin = graph_.offsets[top.v];
            const uint64_t end = graph_.offsets[top.v + 1];
            for (uint64_t i = begin; i < end; ++i) {
                const Edge &e = graph_.edges[static_cast<size_t>(i)];
                ++stats.relaxed_arcs;
                const Distance nd = top.key + e.weight;
                if (nd < dist_f_.get(e.to)) {
                    dist_f_.set(e.to, nd);
                    pq_f.push({nd, e.to});
                    ++stats.heap_pushes;
                    // Update mu if backward side has seen this vertex
                    const Distance db = dist_b_.get(e.to);
                    if (db < kUnreachable) {
                        const Distance candidate = nd + db;
                        if (candidate < mu) {
                            mu = candidate;
                        }
                    }
                }
            }
        } else {
            const HeapNode top = pq_b.top();
            pq_b.pop();
            if (top.key != dist_b_.get(top.v)) {
                continue;
            }
            ++stats.settled;
            ++stats.settled_backward;

            const uint64_t begin = reverse_.offsets[top.v];
            const uint64_t end = reverse_.offsets[top.v + 1];
            for (uint64_t i = begin; i < end; ++i) {
                const Edge &e = reverse_.edges[static_cast<size_t>(i)];
                ++stats.relaxed_arcs;
                const Distance nd = top.key + e.weight;
                if (nd < dist_b_.get(e.to)) {
                    dist_b_.set(e.to, nd);
                    pq_b.push({nd, e.to});
                    ++stats.heap_pushes;
                    // Update mu if forward side has seen this vertex
                    const Distance df = dist_f_.get(e.to);
                    if (df < kUnreachable) {
                        const Distance candidate = df + nd;
                        if (candidate < mu) {
                            mu = candidate;
                        }
                    }
                }
            }
        }
    }

    return PathResult{mu, stats};
}

} // namespace transport
