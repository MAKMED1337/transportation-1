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

// CH-TNR: Transit Node Routing on top of a Contraction Hierarchy.
//
// Preprocessing:
//   T = top `transit` CH-rank vertices.
//   DT[i][j] = d(T_i, T_j): many-to-many via bucket CH (backward upward search from each T_j
//              appends (j, dist) to bucket[T_k] for every settled T_k; forward upward from each
//              T_i scans those buckets to fill the table).
//   Access nodes A_f(v): upward CH search from v, stopping at transit; dominance-pruned.
//   Access nodes A_b(v): backward upward CH search from v, stopping at transit; dominance-pruned.
//   Voronoi region[v] = index of nearest transit node (forward multi-source Dijkstra from T).
//   Locality lists L_f(v) / L_b(v): sorted unique Voronoi region ids of A_f(v) / A_b(v) search
//              spaces (∪ region[v] itself).
//
// Query:
//   If sorted-merge intersection of L_f(s) and L_b(t) is non-empty → local → CH fallback.
//   Else: min over (a∈A_f(s), b∈A_b(t)) of d_a + DT[a][b] + d_b.
class TnrAlgorithm final : public RoutingAlgorithm {
public:
    explicit TnrAlgorithm(const Graph &graph, uint32_t transit = 16384, uint32_t threads = 1);

    std::string_view name() const override;
    std::string variant() const override;
    void preprocess() override;
    [[nodiscard]] PathResult query(VertexId source, VertexId target) const override;

    void inject_ch(ContractionHierarchy ch);
    [[nodiscard]] const ContractionHierarchy &get_ch() const { return ch_; }

    struct TnrStats {
        uint32_t transit_nodes = 0;
        double dt_table_mb = 0.0;
        double dt_build_wall_s = 0.0;
        double dt_build_cpu_s = 0.0;
        double access_nodes_wall_s = 0.0;
        double access_nodes_cpu_s = 0.0;
        double avg_access_fwd = 0.0;
        double avg_access_bwd = 0.0;
        uint32_t max_access_fwd = 0;
        uint32_t max_access_bwd = 0;
        double access_storage_mb = 0.0;
        double voronoi_wall_s = 0.0;
        double voronoi_cpu_s = 0.0;
        double locality_filter_mb = 0.0;
        double total_wall_s = 0.0;
        double total_cpu_s = 0.0;
        bool ch_was_injected = false;
    };

    [[nodiscard]] const TnrStats &tnr_stats() const { return stats_; }

private:
    const Graph &graph_;
    uint32_t transit_count_;
    uint32_t threads_;

    ContractionHierarchy ch_;
    bool ch_provided_ = false;
    bool preprocessed_ = false;

    // Transit node index mapping
    uint32_t transit_rank_threshold_;     // rank >= this → transit
    std::vector<uint32_t> transit_of_;    // transit_of_[v] = transit index if v∈T, else UINT32_MAX
    std::vector<VertexId> transit_verts_; // transit_verts_[i] = vertex id of i-th transit node

    // DT table: dt_[i * transit_count_ + j] = d(transit_verts_[i], transit_verts_[j])
    std::vector<uint32_t> dt_;

    // Access nodes: CSR indexed by vertex id
    struct AccessNode {
        uint32_t transit_idx;
        uint32_t dist;
    };
    std::vector<uint64_t> fwd_access_offsets_; // size V+1
    std::vector<AccessNode> fwd_access_nodes_;
    std::vector<uint64_t> bwd_access_offsets_; // size V+1
    std::vector<AccessNode> bwd_access_nodes_;

    // Voronoi regions for locality filter
    std::vector<uint16_t> voronoi_region_; // region[v] = nearest transit idx (as uint16_t)
    static constexpr uint16_t kNoRegion = 0xFFFF;

    // Locality lists: CSR of sorted uint16_t region ids
    std::vector<uint64_t> fwd_local_offsets_; // size V+1
    std::vector<uint16_t> fwd_local_regions_;
    std::vector<uint64_t> bwd_local_offsets_; // size V+1
    std::vector<uint16_t> bwd_local_regions_;

    TnrStats stats_;
    mutable StampedVector<Distance> ch_fwd_dist_{1, kUnreachable};
    mutable StampedVector<Distance> ch_bwd_dist_{1, kUnreachable};

    void build_dt_table();
    void build_access_nodes();
    void build_locality_filter();

    // Bidirectional CH query for local pairs (fallback)
    [[nodiscard]] Distance ch_query(VertexId source, VertexId target, QueryStats &stats) const;
};

} // namespace transport
