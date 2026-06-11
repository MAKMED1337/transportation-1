#include "algorithms/routing_algorithm_factory.hpp"

#include "algorithms/astar.hpp"
#include "algorithms/ch/contraction_hierarchy.hpp"
#include "algorithms/dijkstra.hpp"

#include <memory>
#include <stdexcept>
#include <string>

namespace transport {

uint32_t param_u32(const AlgorithmParams &p, const std::string &key, uint32_t def) {
    const auto it = p.find(key);
    if (it == p.end()) {
        return def;
    }
    return static_cast<uint32_t>(std::stoul(it->second));
}

double param_double(const AlgorithmParams &p, const std::string &key, double def) {
    const auto it = p.find(key);
    if (it == p.end()) {
        return def;
    }
    return std::stod(it->second);
}

std::string param_str(const AlgorithmParams &p, const std::string &key, std::string def) {
    const auto it = p.find(key);
    if (it == p.end()) {
        return def;
    }
    return it->second;
}

std::unique_ptr<RoutingAlgorithm> make_routing_algorithm(const std::string &name, const Graph &graph,
                                                          const AlgorithmParams &params) {
    if (name == "dijkstra") {
        if (!params.empty()) {
            throw std::invalid_argument("dijkstra takes no parameters");
        }
        return std::make_unique<DijkstraAlgorithm>(graph);
    }
    if (name == "astar") {
        if (!params.empty()) {
            throw std::invalid_argument("astar takes no parameters");
        }
        return std::make_unique<AStarAlgorithm>(graph);
    }
    if (name == "ch") {
        if (!params.empty()) {
            throw std::invalid_argument("ch takes no parameters (use ch_measure for variants)");
        }
        return std::make_unique<ContractionHierarchyAlgorithm>(graph);
    }
    throw std::invalid_argument("unsupported algorithm: " + name);
}

} // namespace transport
