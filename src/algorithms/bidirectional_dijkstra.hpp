#pragma once

#include "algorithms/routing_algorithm.hpp"
#include "algorithms/stamped_vector.hpp"
#include "graph/graph.hpp"
#include "graph/reverse_graph.hpp"

#include <string_view>

namespace transport {

class BidirectionalDijkstraAlgorithm final : public RoutingAlgorithm {
public:
    explicit BidirectionalDijkstraAlgorithm(const Graph &graph);

    [[nodiscard]] std::string_view name() const override;
    [[nodiscard]] std::string variant() const override;
    void preprocess() override;
    [[nodiscard]] PathResult query(VertexId source, VertexId target) const override;

private:
    const Graph &graph_;
    ReverseAdjacency reverse_;
    mutable StampedVector<Distance> dist_f_;
    mutable StampedVector<Distance> dist_b_;
};

} // namespace transport
