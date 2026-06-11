#pragma once

#include "algorithms/ch/contraction_hierarchy.hpp"
#include "algorithms/routing_algorithm.hpp"
#include "algorithms/stamped_vector.hpp"
#include "graph/graph.hpp"
#include "graph/types.hpp"

#include <cstdint>
#include <string>
#include <vector>

namespace transport {

// CHASE: CH + Arc-flags on the core subgraph.
//
// Preprocessing:
//   1. Build CH (or use injected). Define core = top core_fraction of CH ranks.
//   2. Extract core-forward and core-backward subgraph CSRs from CH.
//   3. Partition core vertices; compute arc flags on core arcs from core boundary vertices.
//
// Query (two phases):
//   Phase 1: bidirectional upward CH search. Core vertices are "collected" into entry sets
//            S (forward) and T (backward) when first settled, without relaxing their arcs.
//   Phase 2: bidirectional search on core subgraph seeded by S and T.
//            Forward search prunes arcs unless core_fwd_flags & target_mask.
//            Backward search prunes arcs unless core_bwd_flags & source_mask.
//            Stopping criterion: CH-style (top_f + top_b >= mu from both sides).
//   Answer: mu (updated during both phases).
class ChaseAlgorithm final : public RoutingAlgorithm {
public:
    explicit ChaseAlgorithm(const Graph &graph, float core_fraction = 0.05f, uint32_t regions = 64,
                            std::string partition_method = "inertial", uint32_t threads = 1);

    std::string_view name() const override;
    std::string variant() const override;
    void preprocess() override;
    [[nodiscard]] PathResult query(VertexId source, VertexId target) const override;

    void inject_ch(ContractionHierarchy ch);
    [[nodiscard]] const ContractionHierarchy &get_ch() const;

    struct ChaseStats {
        float core_fraction = 0.0f;
        uint32_t core_vertices = 0;
        uint64_t core_arcs_fwd = 0;
        uint64_t core_arcs_bwd = 0;
        uint32_t region_count = 0;
        uint32_t core_boundary_fwd = 0;
        uint32_t core_boundary_bwd = 0;
        double flags_mb = 0.0;
        double flags_wall_s = 0.0;
        double flags_cpu_s = 0.0;
        double total_wall_s = 0.0;
        double total_cpu_s = 0.0;
        bool ch_was_injected = false;
    };

    [[nodiscard]] const ChaseStats &chase_stats() const;

private:
    const Graph &graph_;
    float core_fraction_;
    uint32_t regions_;
    std::string partition_method_;
    uint32_t threads_;

    ContractionHierarchy ch_;
    bool ch_provided_ = false;
    bool preprocessed_ = false;

    uint32_t core_threshold_; // rank >= core_threshold_ => vertex is in core

    // Core subgraph CSRs (indexed by full vertex id; non-core vertices have empty ranges)
    std::vector<uint64_t> cf_offsets_; // core-forward: upward arcs within core
    std::vector<Edge> cf_edges_;
    std::vector<uint64_t> cb_offsets_; // core-backward: upward-in-reverse arcs within core
    std::vector<Edge> cb_edges_;

    // Reachability masks: cf_reach_[v] = bitmask of regions reachable from v going upward via cf.
    // cb_reach_[v] = bitmask of regions reachable from v going upward via cb.
    // Computed in descending rank order so each vertex accumulates its neighbours' masks.
    std::vector<uint64_t> cf_reach_;
    std::vector<uint64_t> cb_reach_;

    // Per-arc flags (cf_flags_[k] = cf_reach_[cf_edges_[k].to], similarly for cb).
    // Stored per-arc for fast flag-check during query without an extra indirection.
    std::vector<uint64_t> cf_flags_;
    std::vector<uint64_t> cb_flags_;

    std::vector<uint16_t> region_of_; // region assignment for every vertex

    ChaseStats stats_;
    mutable StampedVector<Distance> fwd_dist_;
    mutable StampedVector<Distance> bwd_dist_;

    void build_core_subgraph();
    void compute_core_flags();
};

} // namespace transport
