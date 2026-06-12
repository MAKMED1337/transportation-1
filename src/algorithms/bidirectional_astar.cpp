#include "algorithms/bidirectional_astar.hpp"

#include "algorithms/heap_node.hpp"

#include <cmath>
#include <cstddef>
#include <cstdint>
#include <functional>
#include <limits>
#include <queue>
#include <string>
#include <string_view>

namespace transport {

BidirectionalAStarAlgorithm::BidirectionalAStarAlgorithm(const Graph &graph, Variant variant)
    : graph_(graph), variant_(variant), dist_f_(graph.vertex_count(), kUnreachable),
      dist_b_(graph.vertex_count(), kUnreachable) {}

std::string_view BidirectionalAStarAlgorithm::name() const { return "bidi_astar"; }

std::string BidirectionalAStarAlgorithm::variant() const {
    return variant_ == Variant::Conservative
               ? "bidirectional A* conservative (independent haversine potentials, stop top_f>=mu OR top_b>=mu)"
               : "bidirectional A* consistent (averaged haversine potentials, stop top_f+top_b>=mu)";
}

void BidirectionalAStarAlgorithm::preprocess() { reverse_ = build_reverse_adjacency(graph_); }

PathResult BidirectionalAStarAlgorithm::query(VertexId source, VertexId target) const {
    if (source == target) {
        return PathResult{0, QueryStats{}};
    }

    dist_f_.reset();
    dist_b_.reset();

    const NodeCoord &cs = graph_.coords[source];
    const NodeCoord &ct = graph_.coords[target];

    // pi_f(v) = lower bound on d(v, target)
    auto pi_f = [this, &ct](VertexId v) -> int64_t {
        return static_cast<int64_t>(
            std::floor(haversine_meters(graph_.coords[v], ct) * static_cast<double>(kDistanceScale)));
    };
    // pi_b(v) = lower bound on d(source, v)
    auto pi_b = [this, &cs](VertexId v) -> int64_t {
        return static_cast<int64_t>(
            std::floor(haversine_meters(cs, graph_.coords[v]) * static_cast<double>(kDistanceScale)));
    };

    // Consistent: p_f(v) = (pi_f(v) - pi_b(v)) / 2; p_b = -p_f
    // Conservative: each direction uses its own pi independently
    auto key_f = [&](VertexId v, Distance gv) -> int64_t {
        if (variant_ == Variant::Conservative) {
            return static_cast<int64_t>(gv) + pi_f(v);
        }
        return static_cast<int64_t>(gv) + (pi_f(v) - pi_b(v)) / 2;
    };
    auto key_b = [&](VertexId v, Distance gv) -> int64_t {
        if (variant_ == Variant::Conservative) {
            return static_cast<int64_t>(gv) + pi_b(v);
        }
        return static_cast<int64_t>(gv) - (pi_f(v) - pi_b(v)) / 2;
    };

    std::priority_queue<SignedHeapNode, std::vector<SignedHeapNode>, std::greater<>> pq_f, pq_b;

    dist_f_.set(source, 0);
    dist_b_.set(target, 0);
    pq_f.push({key_f(source, 0), source});
    pq_b.push({key_b(target, 0), target});

    // Use large sentinel for "empty queue top"
    constexpr int64_t kSentinel = std::numeric_limits<int64_t>::max() / 4;

    Distance mu = kUnreachable;
    QueryStats stats;

    while (!pq_f.empty() || !pq_b.empty()) {
        const int64_t top_f = pq_f.empty() ? kSentinel : pq_f.top().key;
        const int64_t top_b = pq_b.empty() ? kSentinel : pq_b.top().key;

        if (mu != kUnreachable) {
            if (variant_ == Variant::Conservative) {
                if (top_f >= static_cast<int64_t>(mu) || top_b >= static_cast<int64_t>(mu)) {
                    break;
                }
            } else {
                // consistent: stop when top_f + top_b >= mu (correction term = 0)
                if (top_f != kSentinel && top_b != kSentinel && top_f + top_b >= static_cast<int64_t>(mu)) {
                    break;
                }
                if (top_f != kSentinel && top_f >= static_cast<int64_t>(mu)) {
                    break;
                }
                if (top_b != kSentinel && top_b >= static_cast<int64_t>(mu)) {
                    break;
                }
            }
        }

        // Pick direction with smaller key
        const bool go_forward = (top_f <= top_b);

        if (go_forward) {
            const SignedHeapNode top = pq_f.top();
            pq_f.pop();
            const Distance gv = dist_f_.get(top.v);
            ++stats.heuristic_evals;
            if (top.key != key_f(top.v, gv)) {
                continue; // stale
            }

            ++stats.settled;
            ++stats.settled_forward;

            const uint64_t begin = graph_.offsets[top.v];
            const uint64_t end = graph_.offsets[top.v + 1];
            for (uint64_t i = begin; i < end; ++i) {
                const Edge &e = graph_.edges[static_cast<size_t>(i)];
                ++stats.relaxed_arcs;
                const Distance ng = gv + e.weight;
                if (ng < dist_f_.get(e.to)) {
                    dist_f_.set(e.to, ng);
                    ++stats.heuristic_evals;
                    pq_f.push({key_f(e.to, ng), e.to});
                    ++stats.heap_pushes;
                    // Update mu
                    const Distance db = dist_b_.get(e.to);
                    if (db < kUnreachable) {
                        const Distance candidate = ng + db;
                        if (candidate < mu) {
                            mu = candidate;
                        }
                    }
                }
            }
        } else {
            const SignedHeapNode top = pq_b.top();
            pq_b.pop();
            const Distance gv = dist_b_.get(top.v);
            ++stats.heuristic_evals;
            if (top.key != key_b(top.v, gv)) {
                continue; // stale
            }

            ++stats.settled;
            ++stats.settled_backward;

            const uint64_t begin = reverse_.offsets[top.v];
            const uint64_t end = reverse_.offsets[top.v + 1];
            for (uint64_t i = begin; i < end; ++i) {
                const Edge &e = reverse_.edges[static_cast<size_t>(i)];
                ++stats.relaxed_arcs;
                const Distance ng = gv + e.weight;
                if (ng < dist_b_.get(e.to)) {
                    dist_b_.set(e.to, ng);
                    ++stats.heuristic_evals;
                    pq_b.push({key_b(e.to, ng), e.to});
                    ++stats.heap_pushes;
                    // Update mu
                    const Distance df = dist_f_.get(e.to);
                    if (df < kUnreachable) {
                        const Distance candidate = df + ng;
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
