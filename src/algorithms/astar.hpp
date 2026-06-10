#pragma once

#include "algorithms/routing_algorithm.hpp"
#include "algorithms/stamped_vector.hpp"
#include "graph/graph.hpp"

#include <string_view>

namespace transport {

class AStarAlgorithm final : public RoutingAlgorithm {
public:
    explicit AStarAlgorithm(const Graph &graph);

    [[nodiscard]] std::string_view name() const override;
    [[nodiscard]] PathResult query(VertexId source, VertexId target) const override;

private:
    const Graph &graph_;
    mutable StampedVector<Distance> g_; // reused query scratch; mutated by the const query()
};

} // namespace transport
