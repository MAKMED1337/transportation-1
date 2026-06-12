#pragma once

#include "algorithms/tnr/tnr.hpp"
#include "graph/types.hpp"

#include <cstdint>
#include <string>
#include <vector>

namespace transport {

// TNR+AF: Transit Node Routing with Arc Flags on the transit overlay.
//
// For each transit node i, fwd_reach_flags_ stores a bitset of words_per_t_ uint64_t words
// where bit j is set iff DT[i][j] < ∞ (transit i can reach transit j).
//
// At query time, a target_mask_ is assembled from the transit indices of t's backward access
// nodes.  A forward access node a_i is skipped if (fwd_reach_flags_[a_i] & target_mask) == 0,
// i.e., no backward access node of t is reachable from a_i.  This is exact (not approximate):
// DT[a_i][b_j] = ∞ for all b_j ∈ B(t)  ⟺  (flags_row_i & target_mask) == 0.
class TnrAfAlgorithm final : public TnrAlgorithm {
public:
    explicit TnrAfAlgorithm(const Graph &graph, uint32_t transit = 16384, uint32_t threads = 1);

    std::string_view name() const override;
    std::string variant() const override;
    void preprocess() override;
    [[nodiscard]] PathResult query(VertexId source, VertexId target) const override;

    struct TnrAfStats {
        double af_build_wall_s = 0.0;
        double af_build_cpu_s = 0.0;
        double af_flags_mb = 0.0;
        double af_pruned_fraction = 0.0; // fraction of fwd access-node candidates pruned at query time
    };

    [[nodiscard]] const TnrAfStats &tnr_af_stats() const { return af_stats_; }

private:
    bool af_preprocessed_ = false;
    uint32_t words_per_t_ = 0;              // ceil(transit_count_ / 64)
    std::vector<uint64_t> fwd_reach_flags_; // transit_count_ × words_per_t_

    mutable std::vector<uint64_t> target_mask_; // per-query scratch, size words_per_t_
    mutable uint64_t total_af_pruned_ = 0;
    mutable uint64_t total_af_candidates_ = 0;
    mutable TnrAfStats af_stats_;

    void build_af_flags();
};

} // namespace transport
