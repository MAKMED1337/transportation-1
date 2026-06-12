#include "algorithms/tnr/tnr_af.hpp"

#include "apps/bench_utils.hpp"

#include <algorithm>
#include <cstdint>
#include <limits>
#include <sstream>
#include <string_view>
#include <vector>

namespace transport {

namespace {
constexpr uint32_t kInf32 = std::numeric_limits<uint32_t>::max();
} // namespace

TnrAfAlgorithm::TnrAfAlgorithm(const Graph &graph, uint32_t transit, uint32_t threads)
    : TnrAlgorithm(graph, transit, threads) {}

std::string_view TnrAfAlgorithm::name() const { return "tnr_af"; }

std::string TnrAfAlgorithm::variant() const {
    std::ostringstream oss;
    oss << "transit=" << transit_count_ << " threads=" << threads_;
    return oss.str();
}

void TnrAfAlgorithm::preprocess() {
    if (af_preprocessed_) {
        return;
    }
    TnrAlgorithm::preprocess();
    build_af_flags();
    af_preprocessed_ = true;
}

void TnrAfAlgorithm::build_af_flags() {
    bench::Stopwatch sw;
    const uint32_t T = transit_count_;
    words_per_t_ = (T + 63) / 64;
    fwd_reach_flags_.assign(static_cast<size_t>(T) * words_per_t_, 0);
    target_mask_.resize(words_per_t_, 0);

    for (uint32_t i = 0; i < T; ++i) {
        uint64_t *row = fwd_reach_flags_.data() + static_cast<size_t>(i) * words_per_t_;
        for (uint32_t j = 0; j < T; ++j) {
            if (dt_[static_cast<size_t>(i) * T + j] < kInf32) {
                row[j / 64] |= 1ULL << (j % 64);
            }
        }
    }

    af_stats_.af_build_wall_s = bench::to_seconds(sw.wall_elapsed());
    af_stats_.af_build_cpu_s = bench::to_seconds(sw.cpu_elapsed());
    af_stats_.af_flags_mb = static_cast<double>(fwd_reach_flags_.size() * sizeof(uint64_t)) / (1024.0 * 1024.0);
}

PathResult TnrAfAlgorithm::query(VertexId source, VertexId target) const {
    QueryStats stats;

    if (source == target) {
        return PathResult{0, stats};
    }

    // Locality check (same as TnrAlgorithm).
    const size_t fl_begin = static_cast<size_t>(fwd_local_offsets_[source]);
    const size_t fl_end = static_cast<size_t>(fwd_local_offsets_[source + 1]);
    const size_t bl_begin = static_cast<size_t>(bwd_local_offsets_[target]);
    const size_t bl_end = static_cast<size_t>(bwd_local_offsets_[target + 1]);

    if (TnrAlgorithm::lists_intersect(fwd_local_regions_.data() + fl_begin, fl_end - fl_begin,
                                      bwd_local_regions_.data() + bl_begin, bl_end - bl_begin)) {
        stats.used_fallback = true;
        const Distance dist = TnrAlgorithm::ch_query(source, target, stats);
        return PathResult{dist, stats};
    }

    const size_t af_begin = static_cast<size_t>(fwd_access_offsets_[source]);
    const size_t af_end = static_cast<size_t>(fwd_access_offsets_[source + 1]);
    const size_t ab_begin = static_cast<size_t>(bwd_access_offsets_[target]);
    const size_t ab_end = static_cast<size_t>(bwd_access_offsets_[target + 1]);

    const uint32_t T = transit_count_;
    const uint32_t W = words_per_t_;

    // Build target transit bitset from B(t).
    std::fill(target_mask_.begin(), target_mask_.end(), 0);
    for (size_t bi = ab_begin; bi < ab_end; ++bi) {
        const uint32_t j = bwd_access_nodes_[bi].transit_idx;
        target_mask_[j / 64] |= 1ULL << (j % 64);
    }

    Distance best = kUnreachable;
    for (size_t ai = af_begin; ai < af_end; ++ai) {
        const AccessNode &af = fwd_access_nodes_[ai];
        ++total_af_candidates_;

        // Skip this forward access node if none of t's backward access transits are reachable.
        const uint64_t *row = fwd_reach_flags_.data() + static_cast<size_t>(af.transit_idx) * W;
        bool reachable = false;
        for (uint32_t w = 0; w < W; ++w) {
            if (row[w] & target_mask_[w]) {
                reachable = true;
                break;
            }
        }
        if (!reachable) {
            ++total_af_pruned_;
            continue;
        }

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

    if (total_af_candidates_ > 0) {
        af_stats_.af_pruned_fraction =
            static_cast<double>(total_af_pruned_) / static_cast<double>(total_af_candidates_);
    }

    return PathResult{best, stats};
}

} // namespace transport
