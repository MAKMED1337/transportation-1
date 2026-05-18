#include "algorithms/astar.hpp"

#include <cmath>
#include <cstdint>
#include <functional>
#include <limits>
#include <queue>
#include <string_view>
#include <vector>

namespace transport {
namespace {

struct HeapNode {
    uint64_t key = 0;
    uint32_t v = 0;
    bool operator>(const HeapNode &other) const { return key > other.key; }
};

constexpr uint64_t kInf = std::numeric_limits<uint64_t>::max();

} // namespace

AStarAlgorithm::AStarAlgorithm(const Graph &graph) : graph_(graph) {}

std::string_view AStarAlgorithm::name() const { return "astar"; }

PathResult AStarAlgorithm::query(uint32_t source, uint32_t target) const {
    const uint32_t n = graph_.vertex_count();
    std::vector<uint64_t> g(n, kInf);
    std::priority_queue<HeapNode, std::vector<HeapNode>, std::greater<>> pq;

    const NodeCoord &t = graph_.coords[target];
    auto heuristic = [this, &t](uint32_t v) -> uint64_t {
        const NodeCoord &c = graph_.coords[v];
        return static_cast<uint64_t>(std::floor(haversine_meters(c, t) * static_cast<double>(kDistanceScale)));
    };

    g[source] = 0;
    pq.push({heuristic(source), source});

    uint32_t settled = 0;
    while (!pq.empty()) {
        const HeapNode top = pq.top();
        pq.pop();

        const uint64_t current_f = g[top.v] + heuristic(top.v);
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
            const uint64_t ng = g[top.v] + e.weight_units;
            if (ng < g[e.to]) {
                g[e.to] = ng;
                pq.push({ng + heuristic(e.to), e.to});
            }
        }
    }

    return PathResult{g[target], settled};
}

} // namespace transport
