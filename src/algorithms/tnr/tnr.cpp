#include "algorithms/tnr/tnr.hpp"

#include "algorithms/heap_node.hpp"
#include "apps/bench_utils.hpp"

#include <algorithm>
#include <cassert>
#include <cstdint>
#include <functional>
#include <limits>
#include <numeric>
#include <queue>
#include <sstream>
#include <stdexcept>
#include <string_view>
#include <thread>
#include <vector>

namespace transport {

namespace {

constexpr uint32_t kInf32 = std::numeric_limits<uint32_t>::max();

// Local heap node using uint32_t keys (all TNR distances are uint32_t).
struct U32HeapNode {
    uint32_t key;
    VertexId v;
    bool operator>(const U32HeapNode &o) const { return key > o.key; }
};
using U32Pq = std::priority_queue<U32HeapNode, std::vector<U32HeapNode>, std::greater<U32HeapNode>>;

// Upward CH Dijkstra on forward or backward arcs.
// Stops relaxing when settling a transit node (transit_of[v] != kInf32).
// Calls on_transit(transit_idx, dist) when one is settled.
// Calls on_vertex(v, dist) for every settled non-transit vertex.
template <typename OnTransit, typename OnVertex>
void upward_search(const std::vector<uint64_t> &offsets, const std::vector<Edge> &edges,
                   const std::vector<uint32_t> &transit_of, VertexId source, std::vector<uint32_t> &dist,
                   OnTransit &&on_transit, OnVertex &&on_vertex) {
    U32Pq pq;
    dist[source] = 0;
    pq.push({0, source});
    while (!pq.empty()) {
        const U32HeapNode top = pq.top();
        pq.pop();
        if (top.key != dist[top.v]) {
            continue;
        }
        const uint32_t ti = transit_of[top.v];
        if (ti != kInf32) {
            on_transit(ti, top.key);
            // Do not relax beyond transit nodes — they act as terminals.
            continue;
        }
        on_vertex(top.v, top.key);
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

// Unconditional upward Dijkstra (no transit pruning). Used for DT build.
void upward_search_full(const std::vector<uint64_t> &offsets, const std::vector<Edge> &edges, VertexId source,
                        std::vector<uint32_t> &dist) {
    U32Pq pq;
    dist[source] = 0;
    pq.push({0, source});
    while (!pq.empty()) {
        const U32HeapNode top = pq.top();
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

TnrAlgorithm::TnrAlgorithm(const Graph &graph, uint32_t transit, uint32_t threads)
    : graph_(graph), transit_count_(transit), threads_(threads) {}

std::string_view TnrAlgorithm::name() const { return "tnr"; }

std::string TnrAlgorithm::variant() const {
    std::ostringstream oss;
    oss << "transit=" << transit_count_ << " threads=" << threads_;
    return oss.str();
}

void TnrAlgorithm::inject_ch(ContractionHierarchy ch) {
    ch_ = std::move(ch);
    ch_provided_ = true;
}

void TnrAlgorithm::preprocess() {
    if (preprocessed_) {
        return;
    }
    bench::Stopwatch total_sw;

    if (!ch_provided_) {
        ContractionHierarchyAlgorithm ch_algo(graph_);
        ch_algo.preprocess();
        ch_ = ch_algo.get_ch();
        stats_.ch_was_injected = false;
    } else {
        stats_.ch_was_injected = true;
    }

    const uint32_t V = ch_.vertex_count();
    const uint32_t T = std::min(transit_count_, V);
    transit_count_ = T;

    ch_fwd_dist_ = StampedVector<Distance>(V, kUnreachable);
    ch_bwd_dist_ = StampedVector<Distance>(V, kUnreachable);

    // Identify transit nodes: top T CH ranks.
    transit_rank_threshold_ = (V >= T) ? (V - T) : 0;
    transit_of_.assign(V, kInf32);
    transit_verts_.resize(T);
    // transit index i corresponds to rank = transit_rank_threshold_ + i
    // (rank ascending → lower-ranked transit node has index 0)
    for (VertexId v = 0; v < V; ++v) {
        if (ch_.rank[v] >= transit_rank_threshold_) {
            const uint32_t idx = ch_.rank[v] - transit_rank_threshold_;
            transit_of_[v] = idx;
            transit_verts_[idx] = v;
        }
    }
    stats_.transit_nodes = T;

    build_dt_table();
    build_access_nodes();
    build_locality_filter();

    stats_.total_wall_s = bench::to_seconds(total_sw.wall_elapsed());
    stats_.total_cpu_s = bench::to_seconds(total_sw.cpu_elapsed());
    preprocessed_ = true;
}

void TnrAlgorithm::build_dt_table() {
    bench::Stopwatch sw;
    const uint32_t T = transit_count_;
    const uint32_t V = ch_.vertex_count();

    // DT table indexed [i * T + j] = d(transit_i, transit_j).
    dt_.assign(static_cast<size_t>(T) * T, kInf32);
    // Self-distances.
    for (uint32_t i = 0; i < T; ++i) {
        dt_[static_cast<size_t>(i) * T + i] = 0;
    }

    // Phase 1: for each transit t_j, backward upward search; record bucket[k] = d for
    // every other transit T_k above t_j.
    // buckets[k][j] = d(transit_k, transit_j) candidate — but only over backward arcs,
    // meaning backward_edges from t_j going upward to ancestors.
    // We store: bucket_bwd[j] = sparse list of (transit_idx, dist) pairs.
    // Since transit nodes have the highest ranks, backward searches only visit other transit nodes.
    std::vector<std::vector<std::pair<uint32_t, uint32_t>>> bucket_bwd(T);

    {
        std::vector<uint32_t> dist(V, kInf32);
        for (uint32_t j = 0; j < T; ++j) {
            const VertexId tj = transit_verts_[j];
            dist[tj] = 0;
            upward_search_full(ch_.backward_offsets, ch_.backward_edges, tj, dist);
            // Collect results for transit vertices only.
            // Bucket indexed by the settled intermediate node k (not by j), so that phase 2
            // can scan bucket_bwd[k] to find all targets j reachable from k with d(tk→tj).
            for (uint32_t k = 0; k < T; ++k) {
                const VertexId tk = transit_verts_[k];
                if (dist[tk] < kInf32) {
                    bucket_bwd[k].push_back({j, dist[tk]});
                }
                dist[tk] = kInf32; // reset selectively
            }
            dist[tj] = kInf32; // reset source
        }
    }

    // Phase 2: for each transit s_i, forward upward search; scan buckets to fill DT.
    {
        std::vector<uint32_t> dist(V, kInf32);
        for (uint32_t i = 0; i < T; ++i) {
            const VertexId si = transit_verts_[i];
            dist[si] = 0;
            upward_search_full(ch_.forward_offsets, ch_.forward_edges, si, dist);
            // For each transit k that si's forward search settled:
            for (uint32_t k = 0; k < T; ++k) {
                const VertexId tk = transit_verts_[k];
                const uint32_t d_ik = dist[tk];
                if (d_ik == kInf32) {
                    continue;
                }
                // Transit k is on the forward path from si. Scan bucket_bwd to find targets j
                // where d(tk → tj) is known from the backward search.
                for (const auto &[j, d_kj] : bucket_bwd[k]) {
                    const uint64_t candidate = static_cast<uint64_t>(d_ik) + d_kj;
                    const uint32_t cand32 = candidate >= kInf32 ? kInf32 : static_cast<uint32_t>(candidate);
                    const size_t idx = static_cast<size_t>(i) * T + j;
                    if (cand32 < dt_[idx]) {
                        dt_[idx] = cand32;
                    }
                }
                dist[tk] = kInf32;
            }
            dist[si] = kInf32;
        }
    }

    stats_.dt_build_wall_s = bench::to_seconds(sw.wall_elapsed());
    stats_.dt_build_cpu_s = bench::to_seconds(sw.cpu_elapsed());
    stats_.dt_table_mb = static_cast<double>(static_cast<size_t>(T) * T * sizeof(uint32_t)) / (1024.0 * 1024.0);
}

void TnrAlgorithm::build_access_nodes() {
    bench::Stopwatch sw;
    const uint32_t V = ch_.vertex_count();
    const uint32_t T = transit_count_;

    // Per-vertex temporary candidate lists before dominance reduction.
    struct Cand {
        uint32_t transit_idx;
        uint32_t dist;
    };

    std::vector<std::vector<Cand>> fwd_cands(V);
    std::vector<std::vector<Cand>> bwd_cands(V);

    // Parallel: split vertices across threads.
    const uint32_t nthreads = std::min(threads_, V);
    auto compute_range = [&](uint32_t thread_id) {
        const uint32_t chunk = (V + nthreads - 1) / nthreads;
        const uint32_t start = thread_id * chunk;
        const uint32_t end = std::min(start + chunk, V);

        std::vector<uint32_t> dist(V, kInf32);
        std::vector<VertexId> touched; // vertices we dirtied in dist[]

        for (uint32_t v = start; v < end; ++v) {
            // Forward access nodes.
            touched.clear();
            upward_search(
                ch_.forward_offsets, ch_.forward_edges, transit_of_, v, dist,
                [&](uint32_t ti, uint32_t d) {
                    fwd_cands[v].push_back({ti, d});
                    touched.push_back(transit_verts_[ti]); // reset dist after search
                },
                [&](VertexId u, uint32_t /*d*/) { touched.push_back(u); });
            dist[v] = kInf32;
            for (const VertexId u : touched) {
                dist[u] = kInf32;
            }

            // Backward access nodes.
            touched.clear();
            upward_search(
                ch_.backward_offsets, ch_.backward_edges, transit_of_, v, dist,
                [&](uint32_t ti, uint32_t d) {
                    bwd_cands[v].push_back({ti, d});
                    touched.push_back(transit_verts_[ti]); // reset dist after search
                },
                [&](VertexId u, uint32_t /*d*/) { touched.push_back(u); });
            dist[v] = kInf32;
            for (const VertexId u : touched) {
                dist[u] = kInf32;
            }
        }
    };

    if (nthreads <= 1) {
        compute_range(0);
    } else {
        std::vector<std::thread> threads;
        threads.reserve(nthreads);
        for (uint32_t t = 0; t < nthreads; ++t) {
            threads.emplace_back(compute_range, t);
        }
        for (std::thread &th : threads) {
            th.join();
        }
    }

    // Dominance reduction: drop (a, d_a) if ∃ (a', d_a') with d_a' + DT[a'][a] ≤ d_a.
    auto reduce = [&](std::vector<Cand> &cands) {
        if (cands.size() <= 1) {
            return;
        }
        std::vector<bool> dominated(cands.size(), false);
        for (size_t i = 0; i < cands.size(); ++i) {
            for (size_t j = 0; j < cands.size(); ++j) {
                if (i == j || dominated[j]) {
                    continue;
                }
                // Check if j dominates i: d_j + DT[j][i] ≤ d_i.
                const size_t dt_idx = static_cast<size_t>(cands[j].transit_idx) * T + cands[i].transit_idx;
                const uint64_t via = static_cast<uint64_t>(cands[j].dist) + dt_[dt_idx];
                if (via <= static_cast<uint64_t>(cands[i].dist)) {
                    dominated[i] = true;
                    break;
                }
            }
        }
        size_t out = 0;
        for (size_t i = 0; i < cands.size(); ++i) {
            if (!dominated[i]) {
                cands[out++] = cands[i];
            }
        }
        cands.resize(out);
    };

    for (uint32_t v = 0; v < V; ++v) {
        reduce(fwd_cands[v]);
        reduce(bwd_cands[v]);
    }

    // Build CSRs.
    fwd_access_offsets_.resize(static_cast<size_t>(V) + 1);
    bwd_access_offsets_.resize(static_cast<size_t>(V) + 1);
    fwd_access_offsets_[0] = 0;
    bwd_access_offsets_[0] = 0;
    for (uint32_t v = 0; v < V; ++v) {
        fwd_access_offsets_[v + 1] = fwd_access_offsets_[v] + static_cast<uint64_t>(fwd_cands[v].size());
        bwd_access_offsets_[v + 1] = bwd_access_offsets_[v] + static_cast<uint64_t>(bwd_cands[v].size());
    }
    fwd_access_nodes_.reserve(fwd_access_offsets_[V]);
    bwd_access_nodes_.reserve(bwd_access_offsets_[V]);
    for (uint32_t v = 0; v < V; ++v) {
        for (const Cand &c : fwd_cands[v]) {
            fwd_access_nodes_.push_back({c.transit_idx, c.dist});
        }
        for (const Cand &c : bwd_cands[v]) {
            bwd_access_nodes_.push_back({c.transit_idx, c.dist});
        }
    }

    // Stats.
    uint64_t total_fwd = 0, total_bwd = 0;
    uint32_t max_fwd = 0, max_bwd = 0;
    for (uint32_t v = 0; v < V; ++v) {
        const auto nf = static_cast<uint32_t>(fwd_cands[v].size());
        const auto nb = static_cast<uint32_t>(bwd_cands[v].size());
        total_fwd += nf;
        total_bwd += nb;
        max_fwd = std::max(max_fwd, nf);
        max_bwd = std::max(max_bwd, nb);
    }
    stats_.avg_access_fwd = V > 0 ? static_cast<double>(total_fwd) / V : 0.0;
    stats_.avg_access_bwd = V > 0 ? static_cast<double>(total_bwd) / V : 0.0;
    stats_.max_access_fwd = max_fwd;
    stats_.max_access_bwd = max_bwd;
    stats_.access_nodes_wall_s = bench::to_seconds(sw.wall_elapsed());
    stats_.access_nodes_cpu_s = bench::to_seconds(sw.cpu_elapsed());

    const uint64_t access_bytes = (fwd_access_nodes_.size() + bwd_access_nodes_.size()) * sizeof(AccessNode) +
                                  (fwd_access_offsets_.size() + bwd_access_offsets_.size()) * sizeof(uint64_t);
    stats_.access_storage_mb = static_cast<double>(access_bytes) / (1024.0 * 1024.0);
}

void TnrAlgorithm::build_locality_filter() {
    bench::Stopwatch sw;
    const uint32_t V = ch_.vertex_count();
    const uint32_t T = transit_count_;

    // Voronoi: multi-source forward Dijkstra from all transit nodes on the original graph.
    voronoi_region_.assign(V, kNoRegion);
    {
        // Use transit index as uint16 (safe since T ≤ 16384 < 65535).
        std::priority_queue<HeapNode, std::vector<HeapNode>, std::greater<>> pq;
        std::vector<uint32_t> dist(V, kInf32);
        for (uint32_t i = 0; i < T; ++i) {
            const VertexId tv = transit_verts_[i];
            dist[tv] = 0;
            voronoi_region_[tv] = static_cast<uint16_t>(i);
            pq.push({0, tv});
        }
        while (!pq.empty()) {
            const HeapNode top = pq.top();
            pq.pop();
            if (top.key != dist[top.v]) {
                continue;
            }
            for (uint64_t i = graph_.offsets[top.v], end = graph_.offsets[top.v + 1]; i < end; ++i) {
                const Edge &e = graph_.edges[static_cast<size_t>(i)];
                const uint64_t nd64 = static_cast<uint64_t>(top.key) + e.weight;
                const uint32_t nd = nd64 >= kInf32 ? kInf32 : static_cast<uint32_t>(nd64);
                if (nd < dist[e.to]) {
                    dist[e.to] = nd;
                    voronoi_region_[e.to] = voronoi_region_[top.v];
                    pq.push({nd, e.to});
                }
            }
        }
    }

    stats_.voronoi_wall_s = bench::to_seconds(sw.wall_elapsed());
    stats_.voronoi_cpu_s = bench::to_seconds(sw.cpu_elapsed());

    // For each vertex v, build its locality list from the regions visited during its access
    // node search. We re-run the access searches but only collect regions.
    const uint32_t nthreads = std::min(threads_, V);

    std::vector<std::vector<uint16_t>> fwd_lists(V);
    std::vector<std::vector<uint16_t>> bwd_lists(V);

    auto compute_lists = [&](uint32_t thread_id) {
        const uint32_t chunk = (V + nthreads - 1) / nthreads;
        const uint32_t start = thread_id * chunk;
        const uint32_t end = std::min(start + chunk, V);

        std::vector<uint32_t> dist(V, kInf32);
        std::vector<VertexId> touched;

        for (uint32_t v = start; v < end; ++v) {
            // Forward list.
            touched.clear();
            std::vector<uint16_t> fwd_regions;
            const uint16_t own = voronoi_region_[v];
            if (own != kNoRegion) {
                fwd_regions.push_back(own);
            }
            upward_search(
                ch_.forward_offsets, ch_.forward_edges, transit_of_, v, dist, [&](uint32_t /*ti*/, uint32_t /*d*/) {},
                [&](VertexId u, uint32_t /*d*/) {
                    touched.push_back(u);
                    const uint16_t r = voronoi_region_[u];
                    if (r != kNoRegion) {
                        fwd_regions.push_back(r);
                    }
                });
            dist[v] = kInf32;
            for (const VertexId u : touched) {
                dist[u] = kInf32;
            }
            std::sort(fwd_regions.begin(), fwd_regions.end());
            fwd_regions.erase(std::unique(fwd_regions.begin(), fwd_regions.end()), fwd_regions.end());
            fwd_lists[v] = std::move(fwd_regions);

            // Backward list.
            touched.clear();
            std::vector<uint16_t> bwd_regions;
            if (own != kNoRegion) {
                bwd_regions.push_back(own);
            }
            upward_search(
                ch_.backward_offsets, ch_.backward_edges, transit_of_, v, dist, [&](uint32_t /*ti*/, uint32_t /*d*/) {},
                [&](VertexId u, uint32_t /*d*/) {
                    touched.push_back(u);
                    const uint16_t r = voronoi_region_[u];
                    if (r != kNoRegion) {
                        bwd_regions.push_back(r);
                    }
                });
            dist[v] = kInf32;
            for (const VertexId u : touched) {
                dist[u] = kInf32;
            }
            std::sort(bwd_regions.begin(), bwd_regions.end());
            bwd_regions.erase(std::unique(bwd_regions.begin(), bwd_regions.end()), bwd_regions.end());
            bwd_lists[v] = std::move(bwd_regions);
        }
    };

    if (nthreads <= 1) {
        compute_lists(0);
    } else {
        std::vector<std::thread> threads;
        threads.reserve(nthreads);
        for (uint32_t t = 0; t < nthreads; ++t) {
            threads.emplace_back(compute_lists, t);
        }
        for (std::thread &th : threads) {
            th.join();
        }
    }

    // Build CSRs.
    fwd_local_offsets_.resize(static_cast<size_t>(V) + 1);
    bwd_local_offsets_.resize(static_cast<size_t>(V) + 1);
    fwd_local_offsets_[0] = 0;
    bwd_local_offsets_[0] = 0;
    for (uint32_t v = 0; v < V; ++v) {
        fwd_local_offsets_[v + 1] = fwd_local_offsets_[v] + static_cast<uint64_t>(fwd_lists[v].size());
        bwd_local_offsets_[v + 1] = bwd_local_offsets_[v] + static_cast<uint64_t>(bwd_lists[v].size());
    }
    fwd_local_regions_.reserve(fwd_local_offsets_[V]);
    bwd_local_regions_.reserve(bwd_local_offsets_[V]);
    for (uint32_t v = 0; v < V; ++v) {
        fwd_local_regions_.insert(fwd_local_regions_.end(), fwd_lists[v].begin(), fwd_lists[v].end());
        bwd_local_regions_.insert(bwd_local_regions_.end(), bwd_lists[v].begin(), bwd_lists[v].end());
    }

    const uint64_t local_bytes = (fwd_local_regions_.size() + bwd_local_regions_.size()) * sizeof(uint16_t) +
                                 (fwd_local_offsets_.size() + bwd_local_offsets_.size()) * sizeof(uint64_t);
    stats_.locality_filter_mb = static_cast<double>(local_bytes) / (1024.0 * 1024.0);
}

// Sorted-merge intersection check: returns true if the two sorted lists share any element.
static bool lists_intersect(const uint16_t *a, size_t na, const uint16_t *b, size_t nb) {
    size_t ia = 0, ib = 0;
    while (ia < na && ib < nb) {
        if (a[ia] == b[ib]) {
            return true;
        }
        if (a[ia] < b[ib]) {
            ++ia;
        } else {
            ++ib;
        }
    }
    return false;
}

Distance TnrAlgorithm::ch_query(VertexId source, VertexId target, QueryStats &stats) const {
    ch_fwd_dist_.reset();
    ch_bwd_dist_.reset();
    auto &fwd_dist = ch_fwd_dist_;
    auto &bwd_dist = ch_bwd_dist_;

    std::priority_queue<HeapNode, std::vector<HeapNode>, std::greater<>> fpq, bpq;
    fwd_dist.set(source, 0);
    bwd_dist.set(target, 0);
    fpq.push({0, source});
    bpq.push({0, target});

    Distance best = kUnreachable;
    while (!fpq.empty() || !bpq.empty()) {
        const Distance ft = fpq.empty() ? kUnreachable : fpq.top().key;
        const Distance bt = bpq.empty() ? kUnreachable : bpq.top().key;
        if (ft >= best && bt >= best) {
            break;
        }
        const bool do_fwd = (!fpq.empty() && (bpq.empty() || ft <= bt));
        auto &pq = do_fwd ? fpq : bpq;
        const auto &my_offsets = do_fwd ? ch_.forward_offsets : ch_.backward_offsets;
        const auto &my_edges = do_fwd ? ch_.forward_edges : ch_.backward_edges;
        auto &my_dist = do_fwd ? fwd_dist : bwd_dist;
        auto &opp_dist = do_fwd ? bwd_dist : fwd_dist;

        const HeapNode top = pq.top();
        pq.pop();
        if (top.key != my_dist.get(top.v)) {
            continue;
        }
        const Distance opp = opp_dist.get(top.v);
        if (opp < kUnreachable) {
            best = std::min(best, top.key + opp);
        }
        ++stats.settled;
        if (do_fwd) {
            ++stats.settled_forward;
        } else {
            ++stats.settled_backward;
        }
        for (uint64_t i = my_offsets[top.v], end = my_offsets[top.v + 1]; i < end; ++i) {
            const Edge &e = my_edges[static_cast<size_t>(i)];
            ++stats.relaxed_arcs;
            const Distance nd = top.key + e.weight;
            if (nd < my_dist.get(e.to) && nd < best) {
                my_dist.set(e.to, nd);
                pq.push({nd, e.to});
                ++stats.heap_pushes;
            }
        }
    }
    return best;
}

PathResult TnrAlgorithm::query(VertexId source, VertexId target) const {
    if (!preprocessed_) {
        throw std::logic_error("TnrAlgorithm::preprocess() must be called before query()");
    }

    QueryStats stats;

    if (source == target) {
        return PathResult{0, stats};
    }

    const size_t fwd_begin = static_cast<size_t>(fwd_local_offsets_[source]);
    const size_t fwd_end = static_cast<size_t>(fwd_local_offsets_[source + 1]);
    const size_t bwd_begin = static_cast<size_t>(bwd_local_offsets_[target]);
    const size_t bwd_end = static_cast<size_t>(bwd_local_offsets_[target + 1]);

    const bool local = lists_intersect(fwd_local_regions_.data() + fwd_begin, fwd_end - fwd_begin,
                                       bwd_local_regions_.data() + bwd_begin, bwd_end - bwd_begin);

    if (local) {
        stats.used_fallback = true;
        const Distance dist = ch_query(source, target, stats);
        return PathResult{dist, stats};
    }

    // Non-local: DT query.
    const size_t af_begin = static_cast<size_t>(fwd_access_offsets_[source]);
    const size_t af_end = static_cast<size_t>(fwd_access_offsets_[source + 1]);
    const size_t ab_begin = static_cast<size_t>(bwd_access_offsets_[target]);
    const size_t ab_end = static_cast<size_t>(bwd_access_offsets_[target + 1]);

    const uint32_t T = transit_count_;
    Distance best = kUnreachable;
    for (size_t ai = af_begin; ai < af_end; ++ai) {
        const AccessNode &af = fwd_access_nodes_[ai];
        for (size_t bi = ab_begin; bi < ab_end; ++bi) {
            const AccessNode &ab = bwd_access_nodes_[bi];
            ++stats.table_lookups;
            const uint32_t dt_val = dt_[static_cast<size_t>(af.transit_idx) * T + ab.transit_idx];
            if (dt_val == kInf32) {
                continue;
            }
            const uint64_t candidate = static_cast<uint64_t>(af.dist) + dt_val + ab.dist;
            if (candidate < static_cast<uint64_t>(best)) {
                best = static_cast<Distance>(candidate);
            }
        }
    }

    return PathResult{best, stats};
}

} // namespace transport
