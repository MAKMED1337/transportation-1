#include "algorithms/arcflags/arc_flags.hpp"

#include "algorithms/ch/contraction_hierarchy.hpp"
#include "algorithms/heap_node.hpp"
#include "algorithms/partition.hpp"
#include "algorithms/phast.hpp"
#include "graph/graph.hpp"
#include "graph/types.hpp"

#include <atomic>
#include <chrono>
#include <cstdint>
#include <ctime>
#include <functional>
#include <limits>
#include <memory>
#include <queue>
#include <sstream>
#include <stdexcept>
#include <string>
#include <string_view>
#include <thread>
#include <vector>

namespace transport {

namespace {
constexpr uint32_t kInf32 = std::numeric_limits<uint32_t>::max();
} // namespace

ArcFlagsAlgorithm::ArcFlagsAlgorithm(const Graph &graph, uint32_t regions, std::string partition_method,
                                     uint32_t threads)
    : graph_(graph), regions_(regions), partition_method_(std::move(partition_method)), threads_(threads),
      dist_(graph.vertex_count(), kUnreachable) {
    if (regions == 0 || regions > 64) {
        throw std::invalid_argument("arcflags: regions must be in [1, 64]");
    }
    if (threads == 0) {
        threads_ = 1;
    }
}

std::string_view ArcFlagsAlgorithm::name() const { return "arcflags"; }

std::string ArcFlagsAlgorithm::variant() const {
    std::ostringstream ss;
    ss << "regions=" << regions_ << " partition=" << partition_method_;
    if (threads_ > 1) {
        ss << " threads=" << threads_;
    }
    return ss.str();
}

void ArcFlagsAlgorithm::inject_ch(ContractionHierarchy ch) {
    ch_ = std::move(ch);
    ch_provided_ = true;
}

const ContractionHierarchy &ArcFlagsAlgorithm::get_ch() const { return ch_; }

const ArcFlagsAlgorithm::ArcFlagsStats &ArcFlagsAlgorithm::arcflags_stats() const { return stats_; }

void ArcFlagsAlgorithm::preprocess() {
    if (preprocessed_) {
        return;
    }

    const auto wall_start = std::chrono::steady_clock::now();
    const clock_t cpu_start = std::clock();

    const uint32_t V = graph_.vertex_count();
    const uint64_t E = static_cast<uint64_t>(graph_.edges.size());

    stats_.ch_was_injected = ch_provided_;

    // Build CH if not injected.
    if (!ch_provided_) {
        ContractionHierarchyAlgorithm ch_algo(graph_);
        ch_algo.preprocess();
        ch_ = ch_algo.get_ch(); // copy; expensive but one-time
    }

    const auto inv_rank = build_inv_rank(ch_);

    // Partition vertices into regions.
    const auto part_start = std::chrono::steady_clock::now();
    region_of_ = build_partition(graph_, regions_, partition_method_);
    stats_.partition_wall_s = std::chrono::duration<double>(std::chrono::steady_clock::now() - part_start).count();

    // Allocate flag array (all zeroes = no flags set yet).
    forward_flags_.assign(static_cast<size_t>(E), 0);

    // PHAST-based flag computation.
    const auto phast_wall_start = std::chrono::steady_clock::now();
    const clock_t phast_cpu_start = std::clock();
    compute_flags(inv_rank);
    stats_.phast_wall_s = std::chrono::duration<double>(std::chrono::steady_clock::now() - phast_wall_start).count();
    stats_.phast_cpu_s = static_cast<double>(std::clock() - phast_cpu_start) / static_cast<double>(CLOCKS_PER_SEC);

    // Own-region rule: every arc u→v gets bit region_of_[v] set unconditionally.
    for (uint32_t u = 0; u < V; ++u) {
        for (uint64_t i = graph_.offsets[u], end = graph_.offsets[u + 1]; i < end; ++i) {
            forward_flags_[static_cast<size_t>(i)] |= 1ULL << region_of_[graph_.edges[static_cast<size_t>(i)].to];
        }
    }

    // Compute stats.
    stats_.flags_mb = static_cast<double>(E * 8) / (1024.0 * 1024.0);
    if (E > 0) {
        uint64_t set_bits = 0;
        for (size_t k = 0; k < static_cast<size_t>(E); ++k) {
            set_bits += static_cast<uint64_t>(__builtin_popcountll(forward_flags_[k]));
        }
        stats_.flag_density = static_cast<double>(set_bits) / (static_cast<double>(E) * static_cast<double>(regions_));
    }

    stats_.total_wall_s = std::chrono::duration<double>(std::chrono::steady_clock::now() - wall_start).count();
    stats_.total_cpu_s = static_cast<double>(std::clock() - cpu_start) / static_cast<double>(CLOCKS_PER_SEC);

    preprocessed_ = true;
}

void ArcFlagsAlgorithm::compute_flags(const std::vector<VertexId> &inv_rank) {
    const uint32_t V = graph_.vertex_count();
    const uint32_t P = kPhastPack;

    // Identify boundary vertices per region.
    // b is a boundary vertex of region R = region_of_[b] iff there exists arc (u, b) with region_of_[u] != R.
    std::vector<bool> is_boundary(V, false);
    for (uint32_t u = 0; u < V; ++u) {
        for (uint64_t i = graph_.offsets[u], end = graph_.offsets[u + 1]; i < end; ++i) {
            const VertexId v = graph_.edges[static_cast<size_t>(i)].to;
            if (region_of_[u] != region_of_[v]) {
                is_boundary[v] = true;
            }
        }
    }

    std::vector<std::vector<VertexId>> boundary_by_region(regions_);
    for (VertexId v = 0; v < V; ++v) {
        if (is_boundary[v]) {
            boundary_by_region[region_of_[v]].push_back(v);
        }
    }

    uint32_t total_bv = 0;
    for (const auto &bvs : boundary_by_region) {
        total_bv += static_cast<uint32_t>(bvs.size());
    }
    stats_.boundary_vertices = total_bv;
    stats_.region_count = regions_;
    stats_.trees_computed = total_bv;

    // Build flat work list: one entry per batch of P boundary vertices.
    struct WorkItem {
        uint32_t region;
        uint32_t batch_start;
    };
    std::vector<WorkItem> work;
    work.reserve((total_bv + P - 1) / P);
    for (uint32_t R = 0; R < regions_; ++R) {
        const auto &bvs = boundary_by_region[R];
        for (uint32_t bs = 0; bs < static_cast<uint32_t>(bvs.size()); bs += P) {
            work.push_back({R, bs});
        }
    }

    if (work.empty()) {
        return;
    }

    const uint32_t num_threads = threads_;
    std::atomic<uint32_t> work_idx{0};
    uint64_t *flags_data = forward_flags_.data();

    // Each thread processes work items until exhausted.
    // Flag writes use atomic_ref so concurrent writes to different bits of the same word are safe.
    auto thread_fn = [&]() {
        std::vector<uint32_t> dist_pack(static_cast<size_t>(V) * static_cast<size_t>(P), kInf32);
        std::vector<VertexId> batch(P);

        while (true) {
            const uint32_t wi = work_idx.fetch_add(1, std::memory_order_relaxed);
            if (wi >= static_cast<uint32_t>(work.size())) {
                break;
            }

            const uint32_t R = work[wi].region;
            const uint32_t bs = work[wi].batch_start;
            const std::vector<VertexId> &bvs = boundary_by_region[R];
            const uint32_t bv_count = static_cast<uint32_t>(bvs.size());
            const uint32_t batch_size = std::min(P, bv_count - bs);
            const uint64_t mask_bit = 1ULL << R;

            // Fill batch; pad with first vertex of batch to keep PHAST dimensions uniform.
            for (uint32_t pi = 0; pi < P; ++pi) {
                batch[pi] = bvs[bs + (pi < batch_size ? pi : 0)];
            }

            phast_all_to_one_batch(ch_, inv_rank, batch, dist_pack);

            // For each original arc (u→v, weight w) at graph edge index k:
            // Equality rule: dist[u] == w + dist[v] means arc is on a shortest path to boundary vertex b.
            // (dist here is d(*,b), so the condition is d(u,b) = w + d(v,b).)
            for (uint32_t u = 0; u < V; ++u) {
                const uint32_t *du = &dist_pack[static_cast<size_t>(u) * P];
                for (uint64_t k = graph_.offsets[u], end = graph_.offsets[u + 1]; k < end; ++k) {
                    const Edge &e = graph_.edges[static_cast<size_t>(k)];
                    const uint32_t *dv = &dist_pack[static_cast<size_t>(e.to) * P];
                    for (uint32_t pi = 0; pi < batch_size; ++pi) {
                        if (du[pi] == kInf32 || dv[pi] == kInf32) {
                            continue;
                        }
                        // Avoid overflow: e.weight + dv[pi] could exceed uint32_t if dv is large.
                        // Since max distance fits in uint32_t for road networks, and both are < kInf32,
                        // their sum may still overflow. Use 64-bit arithmetic.
                        const uint64_t nd = static_cast<uint64_t>(e.weight) + dv[pi];
                        if (nd == du[pi]) {
                            std::atomic_ref<uint64_t>(flags_data[static_cast<size_t>(k)])
                                .fetch_or(mask_bit, std::memory_order_relaxed);
                            break; // bit R is now set for this arc; no need to check more targets in batch
                        }
                    }
                }
            }
        }
    };

    if (num_threads <= 1) {
        thread_fn();
    } else {
        std::vector<std::thread> workers;
        workers.reserve(num_threads);
        for (uint32_t t = 0; t < num_threads; ++t) {
            workers.emplace_back(thread_fn);
        }
        for (auto &w : workers) {
            w.join();
        }
    }
}

PathResult ArcFlagsAlgorithm::query(VertexId source, VertexId target) const {
    if (!preprocessed_) {
        throw std::runtime_error("arcflags: preprocess() must be called before query()");
    }

    const uint64_t target_mask = 1ULL << region_of_[target];

    dist_.reset();
    dist_.set(source, 0);

    std::priority_queue<HeapNode, std::vector<HeapNode>, std::greater<>> pq;
    pq.push({0, source});

    PathResult result;
    while (!pq.empty()) {
        const HeapNode top = pq.top();
        pq.pop();
        if (top.key != dist_.get(top.v)) {
            continue;
        }
        ++result.stats.settled;
        if (top.v == target) {
            break;
        }
        for (uint64_t k = graph_.offsets[top.v], end = graph_.offsets[top.v + 1]; k < end; ++k) {
            ++result.stats.relaxed_arcs;
            if (!(forward_flags_[static_cast<size_t>(k)] & target_mask)) {
                ++result.stats.pruned_by_flag;
                continue;
            }
            const Edge &e = graph_.edges[static_cast<size_t>(k)];
            const Distance nd = top.key + static_cast<Distance>(e.weight);
            if (nd < dist_.get(e.to)) {
                dist_.set(e.to, nd);
                pq.push({nd, e.to});
                ++result.stats.heap_pushes;
            }
        }
    }

    result.distance_units = dist_.get(target);
    return result;
}

} // namespace transport
