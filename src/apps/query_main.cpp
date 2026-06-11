#include "algorithms/routing_algorithm.hpp"
#include "algorithms/routing_algorithm_factory.hpp"
#include "graph/graph.hpp"
#include "routing/routing.hpp"

#include <cstdint>
#include <iostream>
#include <map>
#include <memory>
#include <stdexcept>
#include <string>

int main(int argc, char **argv) {
    if (argc == 2 && std::string(argv[1]) == "--help") {
        std::cout << "usage: transport_query --graph <graph.bin> --source <id> --target <id> "
                     "--algorithm <name> [--param key=value ...]\n";
        return 0;
    }

    std::string graph_path;
    uint32_t source = 0;
    uint32_t target = 0;
    std::string algo = "dijkstra";
    std::map<std::string, std::string> params;

    for (int i = 1; i < argc; ++i) {
        const std::string key = argv[i];
        if (i + 1 < argc) {
            const std::string value = argv[i + 1];
            if (key == "--graph") {
                graph_path = value;
                ++i;
            } else if (key == "--source") {
                source = static_cast<uint32_t>(std::stoul(value));
                ++i;
            } else if (key == "--target") {
                target = static_cast<uint32_t>(std::stoul(value));
                ++i;
            } else if (key == "--algorithm") {
                algo = value;
                ++i;
            } else if (key == "--param") {
                const size_t eq = value.find('=');
                if (eq != std::string::npos) {
                    params[value.substr(0, eq)] = value.substr(eq + 1);
                }
                ++i;
            }
        }
    }

    if (graph_path.empty()) {
        std::cerr << "missing --graph\n";
        return 1;
    }

    const transport::Graph graph = transport::load_graph_binary(graph_path);
    if (source >= graph.vertex_count() || target >= graph.vertex_count()) {
        std::cerr << "source/target out of range\n";
        return 1;
    }

    std::unique_ptr<transport::RoutingAlgorithm> algorithm;
    try {
        algorithm = transport::make_routing_algorithm(algo, graph, params);
        algorithm->preprocess();
    } catch (const std::invalid_argument &err) {
        std::cerr << err.what() << "\n";
        return 1;
    }

    const transport::PathResult result = algorithm->query(source, target);
    std::cout << "algorithm=" << algorithm->name() << "\n";
    std::cout << "distance_units=" << result.distance_units << "\n";
    std::cout << "distance_scale=" << transport::kDistanceScale << "\n";
    std::cout << "distance_m="
              << static_cast<double>(result.distance_units) / static_cast<double>(transport::kDistanceScale) << "\n";
    std::cout << "settled=" << result.stats.settled << "\n";
    std::cout << "relaxed_arcs=" << result.stats.relaxed_arcs << "\n";
    return 0;
}
