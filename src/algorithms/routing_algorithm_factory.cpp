#include "algorithms/routing_algorithm_factory.hpp"

#include "algorithms/alt/alt.hpp"
#include "algorithms/alt/landmarks.hpp"
#include "algorithms/arcflags/arc_flags.hpp"
#include "algorithms/astar.hpp"
#include "algorithms/bidirectional_astar.hpp"
#include "algorithms/bidirectional_dijkstra.hpp"
#include "algorithms/ch/contraction_hierarchy.hpp"
#include "algorithms/dijkstra.hpp"

#include <memory>
#include <sstream>
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
        ch::OrderParams op;
        const auto get_w = [&](const std::string &k, int32_t def) -> int32_t {
            const auto it = params.find(k);
            return it == params.end() ? def : static_cast<int32_t>(std::stoi(it->second));
        };
        op.w_edge_diff = get_w("w_edge_diff", 4);
        op.w_deleted_neighbors = get_w("w_deleted_neighbors", 2);
        op.w_depth = get_w("w_depth", 1);
        op.w_original_edges = get_w("w_original_edges", 1);
        op.w_search_space = get_w("w_search_space", 0);
        op.w_voronoi = get_w("w_voronoi", 0);
        // hop_stages format: "2@0.0,3@3.3,5@10.0" (hop_limit@threshold, comma-separated)
        const std::string hop_str = param_str(params, "hop_stages", "");
        if (!hop_str.empty()) {
            op.hop_stages.clear();
            std::istringstream iss(hop_str);
            std::string token;
            while (std::getline(iss, token, ',')) {
                const size_t at = token.find('@');
                if (at == std::string::npos) {
                    op.hop_stages.push_back({0.0, static_cast<uint32_t>(std::stoul(token))});
                } else {
                    const uint32_t hl = static_cast<uint32_t>(std::stoul(token.substr(0, at)));
                    const double thr = std::stod(token.substr(at + 1));
                    op.hop_stages.push_back({thr, hl});
                }
            }
        }
        return std::make_unique<ContractionHierarchyAlgorithm>(graph, std::move(op));
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
    if (name == "arcflags") {
        const uint32_t regions = param_u32(params, "regions", 32);
        const std::string part = param_str(params, "partition", "inertial");
        const uint32_t threads = param_u32(params, "threads", 1);
        return std::make_unique<ArcFlagsAlgorithm>(graph, regions, part, threads);
    }
    throw std::invalid_argument("unsupported algorithm: " + name);
}

} // namespace transport
