#pragma once

#include "algorithms/ch/contraction_hierarchy.hpp"
#include "algorithms/routing_algorithm.hpp"
#include "algorithms/stamped_vector.hpp"
#include "graph/graph.hpp"
#include "graph/types.hpp"

#include <cstdint>
#include <span>
#include <string>
#include <vector>

namespace transport {

// Hub Labels on top of a Contraction Hierarchy (tiered variant).
//
// Only the top `label_fraction` of CH ranks receive labels.  Every shortest
// path whose apex is in the labeled tier is answered by intersecting the two
// sorted hub-label arrays.  Paths entirely below the tier are captured by a
// bidirectional CH fallback (mu_low) that runs when both endpoints are
// unlabeled.  When exactly one endpoint is unlabeled, the apex is guaranteed
// to be in the tier, so only the labeled-label intersection is needed.
//
// Label construction (descending rank order):
//   L_f(v) = {(v,0)} ∪ min-merge of {(h, d+w) : (h,d)∈L_f(u)} over upward arcs.
//   Prune (h, d) if intersect(current L_f(v), L_b(h)) < d  (keep-equality rule).
//   Symmetric for L_b.
class HubLabelsAlgorithm final : public RoutingAlgorithm {
public:
    explicit HubLabelsAlgorithm(const Graph &graph, float label_fraction = 0.25f, uint64_t memory_budget_gb = 18,
                                uint32_t threads = 1);

    std::string_view name() const override;
    std::string variant() const override;
    void preprocess() override;
    [[nodiscard]] PathResult query(VertexId source, VertexId target) const override;

    void inject_ch(ContractionHierarchy ch);
    [[nodiscard]] const ContractionHierarchy &get_ch() const { return ch_; }

    struct HlStats {
        float label_fraction = 0.0f;
        uint32_t labeled_vertices = 0;
        double avg_label_size_fwd = 0.0;
        double avg_label_size_bwd = 0.0;
        uint32_t max_label_size_fwd = 0;
        uint32_t max_label_size_bwd = 0;
        double label_memory_mb = 0.0;
        double label_build_wall_s = 0.0;
        double label_build_cpu_s = 0.0;
        uint64_t prune_drops = 0;
        bool ch_was_injected = false;
    };

    [[nodiscard]] const HlStats &hl_stats() const { return stats_; }

private:
    const Graph &graph_;
    float label_fraction_;
    uint64_t memory_budget_gb_;
    uint32_t threads_;

    ContractionHierarchy ch_;
    bool ch_provided_ = false;
    bool preprocessed_ = false;

    uint32_t label_threshold_ = 0; // rank >= this → labeled
    uint32_t labeled_count_ = 0;

    // Label entry: sorted by hub within each vertex's label.
    struct HlEntry {
        VertexId hub;
        uint32_t dist;
        bool operator<(const HlEntry &o) const { return hub < o.hub; }
    };

    // CSR labels indexed by vertex_id; unlabeled vertices have empty ranges.
    std::vector<uint64_t> fwd_offsets_; // size V+1
    std::vector<HlEntry> fwd_labels_;
    std::vector<uint64_t> bwd_offsets_; // size V+1
    std::vector<HlEntry> bwd_labels_;

    HlStats stats_;

    // Scratch for query-time upward searches.
    mutable StampedVector<uint32_t> fwd_scratch_{1, ~0u};
    mutable StampedVector<uint32_t> bwd_scratch_{1, ~0u};
    // Reusable collect buffers (per-query; single-threaded queries).
    mutable std::vector<HlEntry> collect_buf_fwd_;
    mutable std::vector<HlEntry> collect_buf_bwd_;

    [[nodiscard]] bool is_labeled(VertexId v) const { return ch_.rank[v] >= label_threshold_; }

    [[nodiscard]] std::span<const HlEntry> fwd_label(VertexId v) const {
        return {fwd_labels_.data() + fwd_offsets_[v], fwd_labels_.data() + fwd_offsets_[v + 1]};
    }
    [[nodiscard]] std::span<const HlEntry> bwd_label(VertexId v) const {
        return {bwd_labels_.data() + bwd_offsets_[v], bwd_labels_.data() + bwd_offsets_[v + 1]};
    }

    // Minimum d_f + d_b over all matching hubs; returns kUnreachable if no match.
    [[nodiscard]] static Distance intersect_labels(std::span<const HlEntry> f, std::span<const HlEntry> b);

    // uint32_t variant for pruning during build.
    [[nodiscard]] static uint32_t intersect_u32(const std::vector<HlEntry> &a, const std::vector<HlEntry> &b);

    void build_labels();

    // Collect forward hub-label from unlabeled source via upward CH search.
    // Stops relaxing at labeled vertices; collects their labels.
    // Settled (source-reachable) vertices written into fwd_scratch_ (caller resets).
    // Settled unlabeled vertices appended to `unlabeled_settled`.
    void collect_fwd(VertexId source, std::vector<HlEntry> &out, std::vector<VertexId> &unlabeled_settled) const;
    void collect_bwd(VertexId target, std::vector<HlEntry> &out, std::vector<VertexId> &unlabeled_settled) const;

    // Bidirectional CH query restricted to below-threshold vertices (for mu_low).
    [[nodiscard]] Distance ch_query_unlabeled(VertexId s, VertexId t) const;
};

} // namespace transport
