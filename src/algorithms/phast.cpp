#include "algorithms/phast.hpp"

#include "algorithms/heap_node.hpp"

#include <algorithm>
#include <cstdint>
#include <functional>
#include <limits>
#include <queue>
#include <vector>

namespace transport {

namespace {

constexpr uint32_t kInf32 = std::numeric_limits<uint32_t>::max();

// Dijkstra on an arbitrary adjacency (offsets[], edges[]) from `source` into dist[].
// dist must be pre-filled with kInf32 except dist[source] = 0.
void bfs_up(const std::vector<uint64_t> &offsets, const std::vector<Edge> &edges, VertexId source,
            std::vector<uint32_t> &dist) {
    std::priority_queue<HeapNode, std::vector<HeapNode>, std::greater<>> pq;
    pq.push({0, source});
    while (!pq.empty()) {
        const HeapNode top = pq.top();
        pq.pop();
        if (top.key != dist[top.v]) {
            continue;
        }
        for (uint64_t i = offsets[top.v], end = offsets[top.v + 1]; i < end; ++i) {
            const Edge &e = edges[static_cast<size_t>(i)];
            const uint64_t nd64 = static_cast<uint64_t>(top.key) + e.weight;
            const uint32_t nd = nd64 >= kInf32 ? kInf32 : static_cast<uint32_t>(nd64);
            if (nd < dist[e.to]) {
                dist[e.to] = nd;
                pq.push({nd, e.to});
            }
        }
    }
}

} // namespace

std::vector<VertexId> build_inv_rank(const ContractionHierarchy &ch) {
    const uint32_t V = ch.vertex_count();
    std::vector<VertexId> inv(V);
    for (VertexId v = 0; v < V; ++v) {
        inv[ch.rank[v]] = v;
    }
    return inv;
}

void phast_all_to_one(const ContractionHierarchy &ch, const std::vector<VertexId> &inv_rank, VertexId target,
                      std::vector<uint32_t> &dist) {
    const uint32_t V = ch.vertex_count();
    std::fill(dist.begin(), dist.end(), kInf32);
    dist[target] = 0;

    // Phase 1: backward upward search from target gives d(v, target) for high-rank v.
    bfs_up(ch.backward_offsets, ch.backward_edges, target, dist);

    // Phase 2: downward sweep using forward arcs.
    // For v (descending rank): arc v→u in forward_offsets (rank[u] > rank[v]):
    //   d(v, target) ≤ w(v,u) + d(u, target)
    for (uint32_t r = V; r-- > 0;) {
        const VertexId v = inv_rank[r];
        for (uint64_t i = ch.forward_offsets[v], end = ch.forward_offsets[v + 1]; i < end; ++i) {
            const Edge &e = ch.forward_edges[static_cast<size_t>(i)];
            if (dist[e.to] == kInf32) {
                continue;
            }
            const uint64_t nd64 = static_cast<uint64_t>(e.weight) + dist[e.to];
            const uint32_t nd = nd64 >= kInf32 ? kInf32 : static_cast<uint32_t>(nd64);
            if (nd < dist[v]) {
                dist[v] = nd;
            }
        }
    }
}

void phast_one_to_all(const ContractionHierarchy &ch, const std::vector<VertexId> &inv_rank, VertexId source,
                      std::vector<uint32_t> &dist) {
    const uint32_t V = ch.vertex_count();
    std::fill(dist.begin(), dist.end(), kInf32);
    dist[source] = 0;

    // Phase 1: forward upward search from source gives d(source, u) for high-rank u.
    bfs_up(ch.forward_offsets, ch.forward_edges, source, dist);

    // Phase 2: downward sweep using backward arcs.
    // backward_offsets[v] stores arc v→u where rank[u] > rank[v], representing original arc u→v.
    // For v (descending rank): arc v→u in backward_offsets (original arc u→v, rank[u] > rank[v]):
    //   d(source, v) ≤ d(source, u) + w(u,v)
    // Since rank[u] > rank[v], u is processed before v → dist[u] is already settled. ✓
    for (uint32_t r = V; r-- > 0;) {
        const VertexId v = inv_rank[r];
        for (uint64_t i = ch.backward_offsets[v], end = ch.backward_offsets[v + 1]; i < end; ++i) {
            const Edge &e = ch.backward_edges[static_cast<size_t>(i)];
            if (dist[e.to] == kInf32) {
                continue;
            }
            const uint64_t nd64 = static_cast<uint64_t>(e.weight) + dist[e.to];
            const uint32_t nd = nd64 >= kInf32 ? kInf32 : static_cast<uint32_t>(nd64);
            if (nd < dist[v]) {
                dist[v] = nd;
            }
        }
    }
}

void phast_all_to_one_batch(const ContractionHierarchy &ch, const std::vector<VertexId> &inv_rank,
                            const std::vector<VertexId> &targets, std::vector<uint32_t> &dist_pack) {
    const uint32_t V = ch.vertex_count();
    const uint32_t P = kPhastPack;
    std::fill(dist_pack.begin(), dist_pack.end(), kInf32);

    // Phase 1: run P independent backward upward Dijkstras.
    for (uint32_t i = 0; i < P && i < static_cast<uint32_t>(targets.size()); ++i) {
        const VertexId t = targets[i];
        dist_pack[static_cast<size_t>(t) * P + i] = 0;
        std::priority_queue<HeapNode, std::vector<HeapNode>, std::greater<>> pq;
        pq.push({0, t});
        while (!pq.empty()) {
            const HeapNode top = pq.top();
            pq.pop();
            if (top.key != dist_pack[static_cast<size_t>(top.v) * P + i]) {
                continue;
            }
            for (uint64_t j = ch.backward_offsets[top.v], end = ch.backward_offsets[top.v + 1]; j < end; ++j) {
                const Edge &e = ch.backward_edges[static_cast<size_t>(j)];
                const uint64_t nd64 = static_cast<uint64_t>(top.key) + e.weight;
                const uint32_t nd = nd64 >= kInf32 ? kInf32 : static_cast<uint32_t>(nd64);
                uint32_t &slot = dist_pack[static_cast<size_t>(e.to) * P + i];
                if (nd < slot) {
                    slot = nd;
                    pq.push({nd, e.to});
                }
            }
        }
    }

    // Phase 2: one merged downward sweep — processes all P targets simultaneously.
    for (uint32_t r = V; r-- > 0;) {
        const VertexId v = inv_rank[r];
        uint32_t *dv = &dist_pack[static_cast<size_t>(v) * P];
        for (uint64_t i = ch.forward_offsets[v], end = ch.forward_offsets[v + 1]; i < end; ++i) {
            const Edge &e = ch.forward_edges[static_cast<size_t>(i)];
            const uint32_t *du = &dist_pack[static_cast<size_t>(e.to) * P];
            for (uint32_t pi = 0; pi < P; ++pi) {
                if (du[pi] == kInf32) {
                    continue;
                }
                const uint64_t nd64 = static_cast<uint64_t>(e.weight) + du[pi];
                const uint32_t nd = nd64 >= kInf32 ? kInf32 : static_cast<uint32_t>(nd64);
                if (nd < dv[pi]) {
                    dv[pi] = nd;
                }
            }
        }
    }
}

void phast_one_to_all_batch(const ContractionHierarchy &ch, const std::vector<VertexId> &inv_rank,
                            const std::vector<VertexId> &sources, std::vector<uint32_t> &dist_pack) {
    const uint32_t V = ch.vertex_count();
    const uint32_t P = kPhastPack;
    std::fill(dist_pack.begin(), dist_pack.end(), kInf32);

    // Phase 1: run P independent forward upward Dijkstras.
    for (uint32_t i = 0; i < P && i < static_cast<uint32_t>(sources.size()); ++i) {
        const VertexId s = sources[i];
        dist_pack[static_cast<size_t>(s) * P + i] = 0;
        std::priority_queue<HeapNode, std::vector<HeapNode>, std::greater<>> pq;
        pq.push({0, s});
        while (!pq.empty()) {
            const HeapNode top = pq.top();
            pq.pop();
            if (top.key != dist_pack[static_cast<size_t>(top.v) * P + i]) {
                continue;
            }
            for (uint64_t j = ch.forward_offsets[top.v], end = ch.forward_offsets[top.v + 1]; j < end; ++j) {
                const Edge &e = ch.forward_edges[static_cast<size_t>(j)];
                const uint64_t nd64 = static_cast<uint64_t>(top.key) + e.weight;
                const uint32_t nd = nd64 >= kInf32 ? kInf32 : static_cast<uint32_t>(nd64);
                uint32_t &slot = dist_pack[static_cast<size_t>(e.to) * P + i];
                if (nd < slot) {
                    slot = nd;
                    pq.push({nd, e.to});
                }
            }
        }
    }

    // Phase 2: one merged downward sweep over backward arcs.
    for (uint32_t r = V; r-- > 0;) {
        const VertexId v = inv_rank[r];
        uint32_t *dv = &dist_pack[static_cast<size_t>(v) * P];
        for (uint64_t i = ch.backward_offsets[v], end = ch.backward_offsets[v + 1]; i < end; ++i) {
            const Edge &e = ch.backward_edges[static_cast<size_t>(i)];
            const uint32_t *du = &dist_pack[static_cast<size_t>(e.to) * P];
            for (uint32_t pi = 0; pi < P; ++pi) {
                if (du[pi] == kInf32) {
                    continue;
                }
                const uint64_t nd64 = static_cast<uint64_t>(e.weight) + du[pi];
                const uint32_t nd = nd64 >= kInf32 ? kInf32 : static_cast<uint32_t>(nd64);
                if (nd < dv[pi]) {
                    dv[pi] = nd;
                }
            }
        }
    }
}

} // namespace transport
