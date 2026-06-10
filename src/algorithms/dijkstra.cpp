#include "algorithms/dijkstra.hpp"

#include "algorithms/heap_node.hpp"

#include <cstddef>
#include <cstdint>
#include <functional>
#include <queue>
#include <string_view>
#include <vector>

namespace transport {

DijkstraAlgorithm::DijkstraAlgorithm(const Graph &graph)
    : graph_(graph), dist_(graph.vertex_count(), kUnreachable) {}

std::string_view DijkstraAlgorithm::name() const { return "dijkstra"; }

PathResult DijkstraAlgorithm::query(VertexId source, VertexId target) const {
    dist_.reset();
    std::priority_queue<HeapNode, std::vector<HeapNode>, std::greater<>> pq;
    dist_.set(source, 0);
    pq.push({0, source});

    uint32_t settled = 0;
    while (!pq.empty()) {
        const HeapNode top = pq.top();
        pq.pop();
        if (top.key != dist_.get(top.v)) {
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
            const Distance nd = top.key + e.weight;
            if (nd < dist_.get(e.to)) {
                dist_.set(e.to, nd);
                pq.push({nd, e.to});
            }
        }
    }

    return PathResult{dist_.get(target), settled};
}

} // namespace transport
