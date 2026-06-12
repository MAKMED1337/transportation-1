#include "algorithms/alt/landmarks.hpp"

#include "algorithms/heap_node.hpp"

#include <algorithm>
#include <cmath>
#include <cstddef>
#include <cstdint>
#include <functional>
#include <limits>
#include <numbers>
#include <queue>
#include <random>
#include <stdexcept>
#include <string>
#include <vector>

namespace transport {
namespace alt {

// --- one-to-all helpers ---

void one_to_all(const Graph &graph, VertexId source, std::vector<uint32_t> &out) {
    const uint32_t V = graph.vertex_count();
    std::fill(out.begin(), out.end(), kLandmarkInf);
    std::priority_queue<HeapNode, std::vector<HeapNode>, std::greater<>> pq;
    out[source] = 0;
    pq.push({0, source});
    while (!pq.empty()) {
        const HeapNode top = pq.top();
        pq.pop();
        if (top.key != out[top.v]) {
            continue;
        }
        const uint64_t begin = graph.offsets[top.v];
        const uint64_t end = graph.offsets[top.v + 1];
        for (uint64_t i = begin; i < end; ++i) {
            const Edge &e = graph.edges[static_cast<size_t>(i)];
            if (e.to >= V) {
                continue;
            }
            const uint64_t nd = static_cast<uint64_t>(top.key) + e.weight;
            const uint32_t nd32 = (nd >= kLandmarkInf) ? kLandmarkInf : static_cast<uint32_t>(nd);
            if (nd32 < out[e.to]) {
                out[e.to] = nd32;
                pq.push({nd32, e.to});
            }
        }
    }
}

void one_to_all_reverse(const ReverseAdjacency &reverse, VertexId source, std::vector<uint32_t> &out) {
    const auto V = static_cast<VertexId>(reverse.offsets.size() - 1);
    std::fill(out.begin(), out.end(), kLandmarkInf);
    std::priority_queue<HeapNode, std::vector<HeapNode>, std::greater<>> pq;
    out[source] = 0;
    pq.push({0, source});
    while (!pq.empty()) {
        const HeapNode top = pq.top();
        pq.pop();
        if (top.key != out[top.v]) {
            continue;
        }
        const uint64_t begin = reverse.offsets[top.v];
        const uint64_t end = reverse.offsets[top.v + 1];
        for (uint64_t i = begin; i < end; ++i) {
            const Edge &e = reverse.edges[static_cast<size_t>(i)];
            if (e.to >= V) {
                continue;
            }
            const uint64_t nd = static_cast<uint64_t>(top.key) + e.weight;
            const uint32_t nd32 = (nd >= kLandmarkInf) ? kLandmarkInf : static_cast<uint32_t>(nd);
            if (nd32 < out[e.to]) {
                out[e.to] = nd32;
                pq.push({nd32, e.to});
            }
        }
    }
}

// --- selection strategies ---

// Random: pick k distinct vertices that have at least one outgoing edge
static std::vector<VertexId> select_random(const Graph &graph, uint32_t k, uint32_t seed) {
    const uint32_t V = graph.vertex_count();
    std::mt19937 rng(seed);
    std::vector<VertexId> candidates;
    candidates.reserve(V);
    for (VertexId v = 0; v < V; ++v) {
        if (graph.offsets[v + 1] > graph.offsets[v]) {
            candidates.push_back(v);
        }
    }
    std::shuffle(candidates.begin(), candidates.end(), rng);
    if (static_cast<uint32_t>(candidates.size()) < k) {
        k = static_cast<uint32_t>(candidates.size());
    }
    return std::vector<VertexId>(candidates.begin(), candidates.begin() + k);
}

// Farthest: iteratively pick the vertex with max min-distance to already-chosen landmarks
static std::vector<VertexId> select_farthest(const Graph &graph, uint32_t k, uint32_t seed) {
    const uint32_t V = graph.vertex_count();
    std::mt19937 rng(seed);
    std::uniform_int_distribution<uint32_t> pick(0, V - 1);

    std::vector<VertexId> chosen;
    chosen.reserve(k);
    std::vector<uint32_t> min_dist(V, kLandmarkInf);
    std::vector<uint32_t> tmp(V);

    // Seed vertex
    const VertexId seed_v = pick(rng);
    one_to_all(graph, seed_v, tmp);

    // First landmark: vertex farthest from seed
    uint32_t best_d = 0;
    VertexId best_v = seed_v;
    for (VertexId v = 0; v < V; ++v) {
        if (tmp[v] > best_d) {
            best_d = tmp[v];
            best_v = v;
        }
    }
    chosen.push_back(best_v);
    one_to_all(graph, best_v, tmp);
    for (VertexId v = 0; v < V; ++v) {
        if (tmp[v] < min_dist[v]) {
            min_dist[v] = tmp[v];
        }
    }

    for (uint32_t i = 1; i < k; ++i) {
        best_d = 0;
        best_v = chosen[0];
        for (VertexId v = 0; v < V; ++v) {
            if (min_dist[v] > best_d) {
                best_d = min_dist[v];
                best_v = v;
            }
        }
        chosen.push_back(best_v);
        one_to_all(graph, best_v, tmp);
        for (VertexId v = 0; v < V; ++v) {
            if (tmp[v] < min_dist[v]) {
                min_dist[v] = tmp[v];
            }
        }
    }
    return chosen;
}

// Planar (sector-based): divide space into k angular sectors around the centroid, pick farthest vertex per sector
static std::vector<VertexId> select_planar(const Graph &graph, uint32_t k, uint32_t /*seed*/) {
    const uint32_t V = graph.vertex_count();
    if (V == 0 || k == 0) {
        return {};
    }

    // Compute centroid
    double sum_lat = 0.0, sum_lon = 0.0;
    uint32_t count = 0;
    for (const NodeCoord &c : graph.coords) {
        if (c.lat != 0.0 || c.lon != 0.0) {
            sum_lat += c.lat;
            sum_lon += c.lon;
            ++count;
        }
    }
    if (count == 0) {
        // Fall back to random if no coordinates
        return select_random(graph, k, 42);
    }
    const double clat = sum_lat / static_cast<double>(count);
    const double clon = sum_lon / static_cast<double>(count);

    // Find vertex nearest to centroid
    double best_dist = std::numeric_limits<double>::max();
    VertexId center_v = 0;
    for (VertexId v = 0; v < V; ++v) {
        const double dlat = graph.coords[v].lat - clat;
        const double dlon = graph.coords[v].lon - clon;
        const double d = dlat * dlat + dlon * dlon;
        if (d < best_dist) {
            best_dist = d;
            center_v = v;
        }
    }

    // Divide into k sectors by angle; pick vertex with max haversine from center per sector
    const NodeCoord &cc = graph.coords[center_v];
    const double two_pi = 2.0 * std::numbers::pi;
    std::vector<double> sector_best_dist(k, -1.0);
    std::vector<VertexId> sector_best_v(k, 0);

    for (VertexId v = 0; v < V; ++v) {
        const NodeCoord &c = graph.coords[v];
        const double angle = std::atan2(c.lat - cc.lat, c.lon - cc.lon); // [-pi, pi]
        const double normalized = (angle + std::numbers::pi) / two_pi;   // [0, 1)
        const uint32_t sector = static_cast<uint32_t>(normalized * static_cast<double>(k)) % k;
        const double d = haversine_meters(cc, c);
        if (d > sector_best_dist[sector]) {
            sector_best_dist[sector] = d;
            sector_best_v[sector] = v;
        }
    }

    // Collect non-empty sectors; fill empty ones from the largest sector
    std::vector<VertexId> chosen;
    chosen.reserve(k);
    for (uint32_t s = 0; s < k; ++s) {
        if (sector_best_dist[s] >= 0.0) {
            chosen.push_back(sector_best_v[s]);
        }
    }
    // If some sectors were empty, fill by splitting largest until we have k
    while (static_cast<uint32_t>(chosen.size()) < k) {
        // Just add the farthest vertex not already chosen
        double fbest = -1.0;
        VertexId fv = 0;
        for (VertexId v = 0; v < V; ++v) {
            const double d = haversine_meters(cc, graph.coords[v]);
            if (d > fbest) {
                bool already = false;
                for (VertexId lv : chosen) {
                    if (lv == v) {
                        already = true;
                        break;
                    }
                }
                if (!already) {
                    fbest = d;
                    fv = v;
                }
            }
        }
        chosen.push_back(fv);
    }

    return std::vector<VertexId>(chosen.begin(), chosen.begin() + k);
}

// --- main entry ---

LandmarkSet build_landmarks(const Graph &graph, const ReverseAdjacency &reverse, uint32_t k, LandmarkStrategy strategy,
                            uint32_t seed) {
    const uint32_t V = graph.vertex_count();
    k = std::min(k, V);

    std::vector<VertexId> chosen;
    std::string sname;
    switch (strategy) {
    case LandmarkStrategy::Random:
        chosen = select_random(graph, k, seed);
        sname = "random";
        break;
    case LandmarkStrategy::Farthest:
        chosen = select_farthest(graph, k, seed);
        sname = "farthest";
        break;
    case LandmarkStrategy::Planar:
        chosen = select_planar(graph, k, seed);
        sname = "planar";
        break;
    case LandmarkStrategy::Avoid:
        throw std::invalid_argument("avoid landmark strategy is not implemented");
        break;
    }

    const size_t kk = chosen.size();
    LandmarkSet ls;
    ls.landmarks = chosen;
    ls.vertex_count = V;
    ls.strategy_name = sname;
    ls.dist_from.resize(kk * static_cast<size_t>(V));
    ls.dist_to.resize(kk * static_cast<size_t>(V));

    std::vector<uint32_t> tmp(V);
    for (size_t i = 0; i < kk; ++i) {
        // dist_from[i*V + v] = d(landmark[i], v) — forward Dijkstra from landmark
        one_to_all(graph, chosen[i], tmp);
        std::copy(tmp.begin(), tmp.end(), ls.dist_from.begin() + static_cast<ptrdiff_t>(i * V));

        // dist_to[i*V + v] = d(v, landmark[i]) — reverse Dijkstra (= forward on reverse graph from landmark)
        one_to_all_reverse(reverse, chosen[i], tmp);
        std::copy(tmp.begin(), tmp.end(), ls.dist_to.begin() + static_cast<ptrdiff_t>(i * V));
    }

    return ls;
}

} // namespace alt
} // namespace transport
