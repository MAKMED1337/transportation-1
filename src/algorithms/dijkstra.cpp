#include "algorithms/dijkstra.hpp"

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

DijkstraAlgorithm::DijkstraAlgorithm(const Graph &graph) : graph_(graph) {}

std::string_view DijkstraAlgorithm::name() const { return "dijkstra"; }

PathResult DijkstraAlgorithm::query(uint32_t source, uint32_t target) const {
    const uint32_t n = graph_.vertex_count();
    std::vector<uint64_t> dist(n, kInf);
    std::priority_queue<HeapNode, std::vector<HeapNode>, std::greater<>> pq;
    dist[source] = 0;
    pq.push({0, source});

    uint32_t settled = 0;
    while (!pq.empty()) {
        const HeapNode top = pq.top();
        pq.pop();
        if (top.key != dist[top.v]) {
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
            const uint64_t nd = top.key + e.weight_units;
            if (nd < dist[e.to]) {
                dist[e.to] = nd;
                pq.push({nd, e.to});
            }
        }
    }

    return PathResult{dist[target], settled};
}

} // namespace transport
