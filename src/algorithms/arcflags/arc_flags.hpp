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

// Forward arc-flags: each directed edge stores a uint64_t bitmask, one bit per region.
// Bit R is set iff the edge lies on at least one shortest path into region R (equality rule)
// or the edge's head vertex is in region R (own-region rule).
// Query prunes edges whose bitmask does not include the target's region bit.
// Supports up to 64 regions (one uint64_t per edge).
// Requires a precomputed ContractionHierarchy for PHAST-based flag computation.
class ArcFlagsAlgorithm final : public RoutingAlgorithm {
public:
    explicit ArcFlagsAlgorithm(const Graph &graph, uint32_t regions = 32, std::string partition_method = "inertial",
                               uint32_t threads = 1);

    std::string_view name() const override;
    std::string variant() const override;
    void preprocess() override;
    [[nodiscard]] PathResult query(VertexId source, VertexId target) const override;

    // Inject a prebuilt CH (e.g. loaded from --ch-file). Must be called before preprocess().
    void inject_ch(ContractionHierarchy ch);
    [[nodiscard]] const ContractionHierarchy &get_ch() const;

    struct ArcFlagsStats {
        uint32_t region_count = 0;
        uint32_t boundary_vertices = 0;
        uint32_t trees_computed = 0;
        double flags_mb = 0.0;
        double flag_density = 0.0;
        double partition_wall_s = 0.0;
        double phast_wall_s = 0.0;
        double phast_cpu_s = 0.0;
        double total_wall_s = 0.0;
        double total_cpu_s = 0.0;
        std::string preprocess_note;
        bool ch_was_injected = false;
    };

    [[nodiscard]] const ArcFlagsStats &arcflags_stats() const;

private:
    const Graph &graph_;
    uint32_t regions_;
    std::string partition_method_;
    uint32_t threads_;

    ContractionHierarchy ch_;
    bool ch_provided_ = false;
    bool preprocessed_ = false;

    std::vector<uint16_t> region_of_;
    std::vector<uint64_t> forward_flags_;

    ArcFlagsStats stats_;
    mutable StampedVector<Distance> dist_;

    void compute_flags(const std::vector<VertexId> &inv_rank);
};

} // namespace transport
