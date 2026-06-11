#pragma once

#include "algorithms/ch/node_order.hpp"
#include "algorithms/routing_algorithm.hpp"
#include "algorithms/stamped_vector.hpp"
#include "graph/graph.hpp"

#include <chrono>
#include <cstdint>
#include <string>
#include <string_view>
#include <utility>
#include <vector>

namespace transport {

struct PreprocessStats {
    std::chrono::nanoseconds ordering_init_ns{0};
    uint64_t witness_calls = 0;
    uint64_t shortcuts_added = 0;
    // (contracted_count_at_switch, new_hop_limit) pairs in contraction order
    std::vector<std::pair<uint32_t, uint32_t>> hop_stage_switches;
};

class ContractionHierarchy {
public:
    std::vector<uint32_t> rank;
    std::vector<uint64_t> forward_offsets;
    std::vector<Edge> forward_edges;
    std::vector<uint64_t> backward_offsets;
    std::vector<Edge> backward_edges;

    [[nodiscard]] VertexId vertex_count() const;
};

class ContractionHierarchyAlgorithm final : public RoutingAlgorithm {
public:
    explicit ContractionHierarchyAlgorithm(const Graph &graph, ch::OrderParams params = {});

    [[nodiscard]] std::string_view name() const override;
    [[nodiscard]] std::string variant() const override;
    void preprocess() override;
    [[nodiscard]] PathResult query(VertexId source, VertexId target) const override;
    [[nodiscard]] uint64_t auxiliary_edge_count() const;
    [[nodiscard]] PreprocessStats preprocess_stats() const { return last_stats_; }
    [[nodiscard]] const ch::OrderParams &order_params() const { return order_params_; }

    // Load a pre-built CH (skips preprocessing). Sets preprocessed_ = true.
    void inject_ch(ContractionHierarchy ch);
    [[nodiscard]] const ContractionHierarchy &get_ch() const { return ch_; }

private:
    const Graph &graph_;
    ch::OrderParams order_params_;
    ContractionHierarchy ch_;
    bool preprocessed_ = false;
    PreprocessStats last_stats_;

    mutable StampedVector<Distance> forward_dist_;
    mutable StampedVector<Distance> backward_dist_;
};

} // namespace transport
