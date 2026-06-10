#pragma once

#include "graph/graph.hpp"
#include "routing/routing.hpp"

#include <cstdint>
#include <string_view>

namespace transport {

class RoutingAlgorithm {
public:
    virtual ~RoutingAlgorithm() = default;

    [[nodiscard]] virtual std::string_view name() const = 0;
    virtual void preprocess() {}
    [[nodiscard]] virtual PathResult query(VertexId source, VertexId target) const = 0;
};

} // namespace transport
