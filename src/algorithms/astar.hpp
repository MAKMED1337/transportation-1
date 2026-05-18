#pragma once

#include "algorithms/routing_algorithm.hpp"
#include "graph/graph.hpp"

#include <cstdint>
#include <string_view>

namespace transport {

class AStarAlgorithm final : public RoutingAlgorithm {
public:
    explicit AStarAlgorithm(const Graph &graph);

    [[nodiscard]] std::string_view name() const override;
    [[nodiscard]] PathResult query(uint32_t source, uint32_t target) const override;

private:
    const Graph &graph_;
};

} // namespace transport
