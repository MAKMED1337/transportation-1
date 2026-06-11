#include "algorithms/routing_algorithm_factory.hpp"

#include "algorithms/alt/alt.hpp"
#include "algorithms/alt/landmarks.hpp"
#include "algorithms/astar.hpp"
#include "algorithms/bidirectional_astar.hpp"
#include "algorithms/bidirectional_dijkstra.hpp"
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
    if (name == "bidijkstra") {
        if (!params.empty()) {
            throw std::invalid_argument("bidijkstra takes no parameters");
        }
        return std::make_unique<BidirectionalDijkstraAlgorithm>(graph);
    }
    if (name == "bidi_astar") {
        const std::string var = param_str(params, "variant", "consistent");
        BidirectionalAStarAlgorithm::Variant v;
        if (var == "conservative") {
            v = BidirectionalAStarAlgorithm::Variant::Conservative;
        } else if (var == "consistent") {
            v = BidirectionalAStarAlgorithm::Variant::Consistent;
        } else {
            throw std::invalid_argument("bidi_astar variant must be conservative or consistent");
        }
        return std::make_unique<BidirectionalAStarAlgorithm>(graph, v);
    }
    if (name == "alt" || name == "alt_bidi") {
        const uint32_t k = param_u32(params, "landmarks", 16);
        const uint32_t active = param_u32(params, "active", 4);
        const uint32_t seed = param_u32(params, "seed", 42);
        const std::string strat_str = param_str(params, "strategy", "farthest");
        alt::LandmarkStrategy strategy;
        if (strat_str == "random") {
            strategy = alt::LandmarkStrategy::Random;
        } else if (strat_str == "farthest") {
            strategy = alt::LandmarkStrategy::Farthest;
        } else if (strat_str == "planar") {
            strategy = alt::LandmarkStrategy::Planar;
        } else if (strat_str == "avoid") {
            strategy = alt::LandmarkStrategy::Avoid;
        } else {
            throw std::invalid_argument("unknown landmark strategy: " + strat_str);
        }
        if (name == "alt") {
            return std::make_unique<AltAlgorithm>(graph, k, strategy, active, seed);
        }
        return std::make_unique<BidirectionalAltAlgorithm>(graph, k, strategy, active, seed);
    }
    throw std::invalid_argument("unsupported algorithm: " + name);
}

} // namespace transport
