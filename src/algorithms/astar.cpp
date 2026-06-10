#include "algorithms/astar.hpp"

#include "algorithms/heap_node.hpp"

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
    std::priority_queue<HeapNode, std::vector<HeapNode>, std::greater<>> pq;

    const NodeCoord &t = graph_.coords[target];
    auto heuristic = [this, &t](VertexId v) -> Distance {
        const NodeCoord &c = graph_.coords[v];
        return static_cast<uint64_t>(std::floor(haversine_meters(c, t) * static_cast<double>(kDistanceScale)));
    };

    g_.set(source, 0);
    pq.push({heuristic(source), source});

    uint32_t settled = 0;
    while (!pq.empty()) {
        const HeapNode top = pq.top();
        pq.pop();

        const Distance current_f = g_.get(top.v) + heuristic(top.v);
        if (top.key != current_f) {
            continue;
        }

        ++settled;
        if (top.v == target) {
            break;
        }

        const uint64_t begin = graph_.offsets[top.v];
        const uint64_t end = graph_.offsets[top.v + 1];
        for (uint64_t i = begin; i < end; ++i) {
            const Edge &e = graph_.edges[static_cast<size_t>(i)];
            const Distance ng = g_.get(top.v) + e.weight;
            if (ng < g_.get(e.to)) {
                g_.set(e.to, ng);
                pq.push({ng + heuristic(e.to), e.to});
            }
        }
    }

    return PathResult{g_.get(target), settled};
}

} // namespace transport
