#include "algorithms/chase/chase.hpp"

#include "algorithms/ch/contraction_hierarchy.hpp"
#include "algorithms/heap_node.hpp"
#include "algorithms/partition.hpp"
#include "graph/graph.hpp"
#include "graph/types.hpp"
#include "routing/routing.hpp"

#include <algorithm>
#include <chrono>
#include <cstdint>
#include <ctime>
#include <limits>
#include <numeric>
#include <queue>
#include <sstream>
#include <stdexcept>
#include <string>
#include <string_view>
#include <vector>

namespace transport {

ChaseAlgorithm::ChaseAlgorithm(const Graph &graph, float core_fraction, uint32_t regions, std::string partition_method,
                               uint32_t threads)
    : graph_(graph), core_fraction_(core_fraction), regions_(regions), partition_method_(std::move(partition_method)),
      threads_(threads == 0 ? 1 : threads), core_threshold_(0), fwd_dist_(graph.vertex_count(), kUnreachable),
      bwd_dist_(graph.vertex_count(), kUnreachable) {
    if (regions == 0 || regions > 64) {
        throw std::invalid_argument("chase: regions must be in [1, 64]");
    }
    if (core_fraction <= 0.0f || core_fraction > 1.0f) {
        throw std::invalid_argument("chase: core_fraction must be in (0, 1]");
    }
}

std::string_view ChaseAlgorithm::name() const { return "chase"; }

std::string ChaseAlgorithm::variant() const {
    std::ostringstream ss;
    ss << "core=" << core_fraction_ << " regions=" << regions_ << " partition=" << partition_method_;
    if (threads_ > 1) {
        ss << " threads=" << threads_;
    }
    return ss.str();
}

void ChaseAlgorithm::inject_ch(ContractionHierarchy ch) {
    ch_ = std::move(ch);
    ch_provided_ = true;
}

const ContractionHierarchy &ChaseAlgorithm::get_ch() const { return ch_; }

const ChaseAlgorithm::ChaseStats &ChaseAlgorithm::chase_stats() const { return stats_; }

void ChaseAlgorithm::preprocess() {
    if (preprocessed_) {
        return;
    }

    const auto wall_start = std::chrono::steady_clock::now();
    const clock_t cpu_start = std::clock();

    stats_.ch_was_injected = ch_provided_;

    if (!ch_provided_) {
        ContractionHierarchyAlgorithm ch_algo(graph_);
        ch_algo.preprocess();
        ch_ = ch_algo.get_ch();
    }

    const uint32_t V = ch_.vertex_count();
    core_threshold_ = static_cast<uint32_t>(static_cast<float>(V) * (1.0f - core_fraction_) + 0.5f);

    region_of_ = build_partition(graph_, regions_, partition_method_);

    build_core_subgraph();

    const auto flags_start = std::chrono::steady_clock::now();
    const clock_t flags_cpu_start = std::clock();
    compute_core_flags();
    stats_.flags_wall_s = std::chrono::duration<double>(std::chrono::steady_clock::now() - flags_start).count();
    stats_.flags_cpu_s = static_cast<double>(std::clock() - flags_cpu_start) / static_cast<double>(CLOCKS_PER_SEC);

    uint32_t core_v = 0;
    for (uint32_t v = 0; v < V; ++v) {
        if (ch_.rank[v] >= core_threshold_) {
            ++core_v;
        }
    }
    stats_.core_fraction = core_fraction_;
    stats_.core_vertices = core_v;
    stats_.core_arcs_fwd = static_cast<uint64_t>(cf_edges_.size());
    stats_.core_arcs_bwd = static_cast<uint64_t>(cb_edges_.size());
    stats_.region_count = regions_;
    stats_.flags_mb =
        static_cast<double>((cf_flags_.size() + cb_flags_.size() + cf_reach_.size() + cb_reach_.size()) * 8) /
        (1024.0 * 1024.0);
    stats_.total_wall_s = std::chrono::duration<double>(std::chrono::steady_clock::now() - wall_start).count();
    stats_.total_cpu_s = static_cast<double>(std::clock() - cpu_start) / static_cast<double>(CLOCKS_PER_SEC);

    preprocessed_ = true;
}

void ChaseAlgorithm::build_core_subgraph() {
    const uint32_t V = ch_.vertex_count();
    cf_offsets_.assign(V + 1, 0);
    cb_offsets_.assign(V + 1, 0);

    for (uint32_t u = 0; u < V; ++u) {
        if (ch_.rank[u] < core_threshold_) {
            continue;
        }
        for (uint64_t i = ch_.forward_offsets[u], end = ch_.forward_offsets[u + 1]; i < end; ++i) {
            if (ch_.rank[ch_.forward_edges[static_cast<size_t>(i)].to] >= core_threshold_) {
                ++cf_offsets_[u + 1];
            }
        }
        for (uint64_t i = ch_.backward_offsets[u], end = ch_.backward_offsets[u + 1]; i < end; ++i) {
            if (ch_.rank[ch_.backward_edges[static_cast<size_t>(i)].to] >= core_threshold_) {
                ++cb_offsets_[u + 1];
            }
        }
    }

    for (uint32_t u = 0; u < V; ++u) {
        cf_offsets_[u + 1] += cf_offsets_[u];
        cb_offsets_[u + 1] += cb_offsets_[u];
    }

    cf_edges_.resize(cf_offsets_[V]);
    cb_edges_.resize(cb_offsets_[V]);

    std::vector<uint64_t> cf_fill(cf_offsets_.begin(), cf_offsets_.end());
    std::vector<uint64_t> cb_fill(cb_offsets_.begin(), cb_offsets_.end());

    for (uint32_t u = 0; u < V; ++u) {
        if (ch_.rank[u] < core_threshold_) {
            continue;
        }
        for (uint64_t i = ch_.forward_offsets[u], end = ch_.forward_offsets[u + 1]; i < end; ++i) {
            const Edge &e = ch_.forward_edges[static_cast<size_t>(i)];
            if (ch_.rank[e.to] >= core_threshold_) {
                cf_edges_[cf_fill[u]++] = e;
            }
        }
        for (uint64_t i = ch_.backward_offsets[u], end = ch_.backward_offsets[u + 1]; i < end; ++i) {
            const Edge &e = ch_.backward_edges[static_cast<size_t>(i)];
            if (ch_.rank[e.to] >= core_threshold_) {
                cb_edges_[cb_fill[u]++] = e;
            }
        }
    }
}

void ChaseAlgorithm::compute_core_flags() {
    const uint32_t V = ch_.vertex_count();

    // Count boundary vertices for stats (heads of cross-region core arcs).
    {
        std::vector<bool> cf_mark(V, false);
        std::vector<bool> cb_mark(V, false);
        for (uint32_t u = 0; u < V; ++u) {
            if (ch_.rank[u] < core_threshold_) {
                continue;
            }
            for (uint64_t i = cf_offsets_[u], end = cf_offsets_[u + 1]; i < end; ++i) {
                const VertexId v = cf_edges_[static_cast<size_t>(i)].to;
                if (region_of_[u] != region_of_[v]) {
                    cf_mark[v] = true;
                }
            }
            for (uint64_t i = cb_offsets_[u], end = cb_offsets_[u + 1]; i < end; ++i) {
                const VertexId v = cb_edges_[static_cast<size_t>(i)].to;
                if (region_of_[u] != region_of_[v]) {
                    cb_mark[v] = true;
                }
            }
        }
        uint32_t total_cf_bnd = 0;
        uint32_t total_cb_bnd = 0;
        for (uint32_t v = 0; v < V; ++v) {
            if (cf_mark[v]) {
                ++total_cf_bnd;
            }
            if (cb_mark[v]) {
                ++total_cb_bnd;
            }
        }
        stats_.core_boundary_fwd = total_cf_bnd;
        stats_.core_boundary_bwd = total_cb_bnd;
    }

    // Compute transitive upward reachability masks in descending rank order.
    // Since cf/cb arcs only go to higher-ranked vertices (already processed), OR-ing
    // neighbours' masks gives the transitive closure of upward reachability.
    cf_reach_.assign(V, 0);
    cb_reach_.assign(V, 0);

    std::vector<VertexId> by_rank(V);
    std::iota(by_rank.begin(), by_rank.end(), VertexId{0});
    std::sort(by_rank.begin(), by_rank.end(), [&](VertexId a, VertexId b) { return ch_.rank[a] > ch_.rank[b]; });

    for (const VertexId v : by_rank) {
        if (ch_.rank[v] < core_threshold_) {
            continue;
        }
        uint64_t cf_mask = 1ULL << region_of_[v];
        uint64_t cb_mask = 1ULL << region_of_[v];
        for (uint64_t k = cf_offsets_[v], end = cf_offsets_[v + 1]; k < end; ++k) {
            cf_mask |= cf_reach_[cf_edges_[static_cast<size_t>(k)].to];
        }
        for (uint64_t k = cb_offsets_[v], end = cb_offsets_[v + 1]; k < end; ++k) {
            cb_mask |= cb_reach_[cb_edges_[static_cast<size_t>(k)].to];
        }
        cf_reach_[v] = cf_mask;
        cb_reach_[v] = cb_mask;
    }

    // Store per-arc flags as the head vertex's reachability mask.
    const size_t cf_count = cf_edges_.size();
    const size_t cb_count = cb_edges_.size();
    cf_flags_.assign(cf_count, 0);
    cb_flags_.assign(cb_count, 0);
    for (uint32_t v = 0; v < V; ++v) {
        for (uint64_t k = cf_offsets_[v], end = cf_offsets_[v + 1]; k < end; ++k) {
            cf_flags_[static_cast<size_t>(k)] = cf_reach_[cf_edges_[static_cast<size_t>(k)].to];
        }
        for (uint64_t k = cb_offsets_[v], end = cb_offsets_[v + 1]; k < end; ++k) {
            cb_flags_[static_cast<size_t>(k)] = cb_reach_[cb_edges_[static_cast<size_t>(k)].to];
        }
    }
}

PathResult ChaseAlgorithm::query(VertexId source, VertexId target) const {
    if (!preprocessed_) {
        throw std::runtime_error("chase: preprocess() must be called before query()");
    }

    fwd_dist_.reset();
    bwd_dist_.reset();

    fwd_dist_.set(source, 0);
    bwd_dist_.set(target, 0);

    std::priority_queue<HeapNode, std::vector<HeapNode>, std::greater<>> fwd_pq;
    std::priority_queue<HeapNode, std::vector<HeapNode>, std::greater<>> bwd_pq;
    fwd_pq.push({0, source});
    bwd_pq.push({0, target});

    // Phase 1 entry sets: core vertices settled in the below-core CH search.
    std::vector<std::pair<VertexId, Distance>> fwd_entries;
    std::vector<std::pair<VertexId, Distance>> bwd_entries;

    Distance mu = kUnreachable;
    QueryStats stats;

    // Phase 1: bidirectional CH search. Core vertices add to entry sets without relaxing.
    auto p1_settle = [&](bool is_fwd) {
        auto &my_pq = is_fwd ? fwd_pq : bwd_pq;
        const auto &my_off = is_fwd ? ch_.forward_offsets : ch_.backward_offsets;
        const auto &my_edges = is_fwd ? ch_.forward_edges : ch_.backward_edges;
        auto &my_dist = is_fwd ? fwd_dist_ : bwd_dist_;
        auto &opp_dist = is_fwd ? bwd_dist_ : fwd_dist_;
        auto &entries = is_fwd ? fwd_entries : bwd_entries;

        const HeapNode top = my_pq.top();
        my_pq.pop();
        if (top.key != my_dist.get(top.v)) {
            return;
        }

        ++stats.settled;
        if (is_fwd) {
            ++stats.settled_forward;
        } else {
            ++stats.settled_backward;
        }

        // mu update
        const Distance opp = opp_dist.get(top.v);
        if (opp != kUnreachable) {
            const Distance cand = top.key + opp;
            if (cand < mu) {
                mu = cand;
            }
        }

        if (ch_.rank[top.v] >= core_threshold_) {
            entries.push_back({top.v, top.key});
            return; // do not relax arcs of core vertices in phase 1
        }

        for (uint64_t i = my_off[top.v], end = my_off[top.v + 1]; i < end; ++i) {
            const Edge &e = my_edges[static_cast<size_t>(i)];
            ++stats.relaxed_arcs;
            const Distance nd = top.key + static_cast<Distance>(e.weight);
            if (nd < mu && nd < my_dist.get(e.to)) {
                my_dist.set(e.to, nd);
                my_pq.push({nd, e.to});
                ++stats.heap_pushes;
            }
        }
    };

    // Phase 1 runs to exhaustion — mu-based early stop would miss core entry points
    // when mu is set by a non-core path before all first-core-contact vertices are settled.
    while (!fwd_pq.empty() || !bwd_pq.empty()) {
        const Distance ftop = fwd_pq.empty() ? kUnreachable : fwd_pq.top().key;
        const Distance btop = bwd_pq.empty() ? kUnreachable : bwd_pq.top().key;
        if (!fwd_pq.empty() && (bwd_pq.empty() || ftop <= btop)) {
            p1_settle(true);
        } else {
            p1_settle(false);
        }
    }

    // Phase 2: bidirectional search on core subgraph with reachability-filter pruning.
    if (!fwd_entries.empty() && !bwd_entries.empty()) {
        // target_mask = union of cb_reach_ masks of all backward entries.
        // source_mask = union of cf_reach_ masks of all forward entries.
        // An arc is pruned when its reachability mask does not intersect the opposing mask.
        uint64_t target_mask = 0;
        uint64_t source_mask = 0;
        for (const auto &[v, d] : bwd_entries) {
            target_mask |= cb_reach_[v];
        }
        for (const auto &[v, d] : fwd_entries) {
            source_mask |= cf_reach_[v];
        }

        std::priority_queue<HeapNode, std::vector<HeapNode>, std::greater<>> cf_pq;
        std::priority_queue<HeapNode, std::vector<HeapNode>, std::greater<>> cb_pq;

        for (const auto &[v, d] : fwd_entries) {
            if (d < fwd_dist_.get(v)) {
                fwd_dist_.set(v, d);
            }
            if (d < mu) {
                cf_pq.push({d, v});
            }
        }
        for (const auto &[v, d] : bwd_entries) {
            if (d < bwd_dist_.get(v)) {
                bwd_dist_.set(v, d);
            }
            if (d < mu) {
                cb_pq.push({d, v});
            }
        }

        auto p2_settle = [&](bool is_fwd) {
            auto &my_pq = is_fwd ? cf_pq : cb_pq;
            const auto &my_off = is_fwd ? cf_offsets_ : cb_offsets_;
            const auto &my_edges = is_fwd ? cf_edges_ : cb_edges_;
            const auto &my_flags = is_fwd ? cf_flags_ : cb_flags_;
            auto &my_dist = is_fwd ? fwd_dist_ : bwd_dist_;
            auto &opp_dist = is_fwd ? bwd_dist_ : fwd_dist_;
            const uint64_t prune_mask = is_fwd ? target_mask : source_mask;

            const HeapNode top = my_pq.top();
            my_pq.pop();
            if (top.key != my_dist.get(top.v)) {
                return;
            }

            ++stats.settled;
            if (is_fwd) {
                ++stats.settled_forward;
            } else {
                ++stats.settled_backward;
            }

            const Distance opp = opp_dist.get(top.v);
            if (opp != kUnreachable) {
                const Distance cand = top.key + opp;
                if (cand < mu) {
                    mu = cand;
                }
            }

            for (uint64_t i = my_off[top.v], end = my_off[top.v + 1]; i < end; ++i) {
                ++stats.relaxed_arcs;
                if (!(my_flags[static_cast<size_t>(i)] & prune_mask)) {
                    ++stats.pruned_by_flag;
                    continue;
                }
                const Edge &e = my_edges[static_cast<size_t>(i)];
                const Distance nd = top.key + static_cast<Distance>(e.weight);
                if (nd < mu && nd < my_dist.get(e.to)) {
                    my_dist.set(e.to, nd);
                    my_pq.push({nd, e.to});
                    ++stats.heap_pushes;
                }
            }
        };

        while (!cf_pq.empty() || !cb_pq.empty()) {
            const Distance ftop = cf_pq.empty() ? kUnreachable : cf_pq.top().key;
            const Distance btop = cb_pq.empty() ? kUnreachable : cb_pq.top().key;
            if (ftop >= mu && btop >= mu) {
                break;
            }
            if (!cf_pq.empty() && (cb_pq.empty() || ftop <= btop)) {
                p2_settle(true);
            } else {
                p2_settle(false);
            }
        }
    }

    return PathResult{mu, stats};
}

} // namespace transport
