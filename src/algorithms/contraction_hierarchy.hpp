#pragma once

#include "algorithms/routing_algorithm.hpp"
#include "graph/graph.hpp"

#include <cstdint>
#include <string_view>
#include <vector>

namespace transport {

class ContractionHierarchy {
public:
    std::vector<uint32_t> rank;
    std::vector<uint64_t> forward_offsets;
    std::vector<Edge> forward_edges;
    std::vector<uint64_t> backward_offsets;
    std::vector<Edge> backward_edges;

    [[nodiscard]] uint32_t vertex_count() const;
};

class ContractionHierarchyAlgorithm final : public RoutingAlgorithm {
public:
    explicit ContractionHierarchyAlgorithm(const Graph &graph);

    [[nodiscard]] std::string_view name() const override;
    void preprocess() override;
    [[nodiscard]] PathResult query(uint32_t source, uint32_t target) const override;
    [[nodiscard]] uint64_t auxiliary_edge_count() const;

private:
    const Graph &graph_;
    ContractionHierarchy ch_;
    bool preprocessed_ = false;
};

} // namespace transport
