#pragma once

#include "algorithms/routing_algorithm.hpp"
#include "graph/graph.hpp"

#include <memory>
#include <string>

namespace transport {

std::unique_ptr<RoutingAlgorithm> make_routing_algorithm(const std::string &name, const Graph &graph);

} // namespace transport
