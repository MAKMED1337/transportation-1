#include "algorithms/routing_algorithm_factory.hpp"

#include "algorithms/astar.hpp"
#include "algorithms/contraction_hierarchy.hpp"
#include "algorithms/dijkstra.hpp"

#include <memory>
#include <stdexcept>
#include <string>

namespace transport {

std::unique_ptr<RoutingAlgorithm> make_routing_algorithm(const std::string &name, const Graph &graph) {
    if (name == "dijkstra") {
        return std::make_unique<DijkstraAlgorithm>(graph);
    }
    if (name == "astar") {
        return std::make_unique<AStarAlgorithm>(graph);
    }
    if (name == "ch") {
        return std::make_unique<ContractionHierarchyAlgorithm>(graph);
    }
    throw std::invalid_argument("unsupported algorithm: " + name);
}

} // namespace transport
