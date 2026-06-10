#pragma once

#include "algorithms/routing_algorithm.hpp"
#include "algorithms/stamped_vector.hpp"
#include "graph/graph.hpp"

#include <string_view>

namespace transport {

class DijkstraAlgorithm final : public RoutingAlgorithm {
public:
    explicit DijkstraAlgorithm(const Graph &graph);

    [[nodiscard]] std::string_view name() const override;
    [[nodiscard]] PathResult query(VertexId source, VertexId target) const override;

private:
    const Graph &graph_;
    mutable StampedVector<Distance> dist_; // reused query scratch; mutated by the const query()
};

} // namespace transport
