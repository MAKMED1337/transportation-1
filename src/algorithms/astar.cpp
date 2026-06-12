#include "algorithms/astar.hpp"

#include <cmath>
#include <cstddef>
#include <cstdint>
#include <functional>
#include <queue>
#include <string_view>
#include <vector>

namespace transport {

AStarAlgorithm::AStarAlgorithm(const Graph &graph) : graph_(graph), g_(graph.vertex_count(), kUnreachable) {}

std::string_view AStarAlgorithm::name() const { return "astar"; }

PathResult AStarAlgorithm::query(VertexId source, VertexId target) const {
    g_.reset();

    struct AStarNode {
        Distance f;
        Distance g;
        VertexId v;
        bool operator>(const AStarNode &other) const { return f > other.f; }
    };
    std::priority_queue<AStarNode, std::vector<AStarNode>, std::greater<>> pq;

    const NodeCoord &t = graph_.coords[target];
    auto heuristic = [this, &t](VertexId v) -> Distance {
        const NodeCoord &c = graph_.coords[v];
        return static_cast<uint64_t>(std::floor(haversine_meters(c, t) * static_cast<double>(kDistanceScale)));
    };

    QueryStats stats;
    g_.set(source, 0);
    ++stats.heuristic_evals;
    pq.push({heuristic(source), 0, source});

    while (!pq.empty()) {
        const AStarNode top = pq.top();
        pq.pop();

        if (top.g != g_.get(top.v)) {
            continue;
        }

        ++stats.settled;
        if (top.v == target) {
            break;
        }

        const uint64_t begin = graph_.offsets[top.v];
        const uint64_t end = graph_.offsets[top.v + 1];
        for (uint64_t i = begin; i < end; ++i) {
            const Edge &e = graph_.edges[static_cast<size_t>(i)];
            ++stats.relaxed_arcs;
            const Distance ng = top.g + e.weight;
            if (ng < g_.get(e.to)) {
                g_.set(e.to, ng);
                ++stats.heuristic_evals;
                pq.push({ng + heuristic(e.to), ng, e.to});
                ++stats.heap_pushes;
            }
        }
    }

    return PathResult{g_.get(target), stats};
}

} // namespace transport
