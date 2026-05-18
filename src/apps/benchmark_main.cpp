#include "algorithms/contraction_hierarchy.hpp"
#include "algorithms/routing_algorithm.hpp"
#include "algorithms/routing_algorithm_factory.hpp"
#include "graph/graph.hpp"
#include "routing/routing.hpp"

#include <array>
#include <chrono>
#include <cstdint>
#include <filesystem>
#include <fstream>
#include <iostream>
#include <memory>
#include <random>
#include <stdexcept>
#include <string>
#include <string_view>

namespace fs = std::filesystem;

namespace {

struct TimedResult {
    transport::PathResult path;
    uint64_t query_us = 0;
};

struct TimedAlgorithmResult {
    std::string_view name;
    const TimedResult &timed;
};

uint64_t preprocess_timed(transport::RoutingAlgorithm &algorithm) {
    const auto t0 = std::chrono::steady_clock::now();
    algorithm.preprocess();
    const auto t1 = std::chrono::steady_clock::now();
    return static_cast<uint64_t>(std::chrono::duration_cast<std::chrono::microseconds>(t1 - t0).count());
}

TimedResult query_timed(const transport::RoutingAlgorithm &algorithm, uint32_t source, uint32_t target) {
    const auto t0 = std::chrono::steady_clock::now();
    const transport::PathResult result = algorithm.query(source, target);
    const auto t1 = std::chrono::steady_clock::now();
    return TimedResult{result,
                       static_cast<uint64_t>(std::chrono::duration_cast<std::chrono::microseconds>(t1 - t0).count())};
}

bool same_distance(uint64_t a, uint64_t b) { return a == b; }

void print_preprocessing_metrics(std::string_view prefix, const transport::RoutingAlgorithm &algorithm,
                                 uint64_t preprocess_us) {
    std::cout << prefix << "_preprocess_us=" << preprocess_us << "\n";
    if (const auto *ch = dynamic_cast<const transport::ContractionHierarchyAlgorithm *>(&algorithm)) {
        std::cout << prefix << "_auxiliary_edges=" << ch->auxiliary_edge_count() << "\n";
    }
}

void write_benchmark_row(std::ofstream &out, uint32_t query, uint32_t source, uint32_t target,
                         const std::array<TimedAlgorithmResult, 2> &results) {
    out << query << "," << source << "," << target;
    for (const TimedAlgorithmResult &result : results) {
        out << "," << result.name;
    }
    out << "," << transport::kDistanceScale;
    for (const TimedAlgorithmResult &result : results) {
        out << "," << result.timed.path.distance_units;
    }
    for (const TimedAlgorithmResult &result : results) {
        out << "," << result.timed.path.settled;
    }
    for (const TimedAlgorithmResult &result : results) {
        out << "," << result.timed.query_us;
    }
    out << "\n";
}

} // namespace

int main(int argc, char **argv) {
    std::string graph_path;
    std::string out_path = "reports/benchmarks/results.csv";
    std::string algorithm_a = "dijkstra";
    std::string algorithm_b = "ch";
    uint32_t queries = 10'000;
    uint32_t min_settled = 100'000;
    uint32_t max_settled = 1'000'000;
    uint32_t seed = 1;

    for (int i = 1; i + 1 < argc; ++i) {
        const std::string key = argv[i];
        const std::string value = argv[i + 1];
        if (key == "--graph") {
            graph_path = value;
            ++i;
        } else if (key == "--out") {
            out_path = value;
            ++i;
        } else if (key == "--queries") {
            queries = static_cast<uint32_t>(std::stoul(value));
            ++i;
        } else if (key == "--min-settled") {
            min_settled = static_cast<uint32_t>(std::stoul(value));
            ++i;
        } else if (key == "--max-settled") {
            max_settled = static_cast<uint32_t>(std::stoul(value));
            ++i;
        } else if (key == "--seed") {
            seed = static_cast<uint32_t>(std::stoul(value));
            ++i;
        } else if (key == "--algorithm-a") {
            algorithm_a = value;
            ++i;
        } else if (key == "--algorithm-b") {
            algorithm_b = value;
            ++i;
        }
    }

    if (graph_path.empty()) {
        std::cerr << "usage: transport_benchmark --graph <graph.bin> [--algorithm-a dijkstra|astar|ch] [--algorithm-b "
                     "dijkstra|astar|ch] [--queries N] [--min-settled A] [--max-settled B] [--seed S] [--out file]\n";
        return 1;
    }

    const transport::Graph graph = transport::load_graph_binary(graph_path);
    std::unique_ptr<transport::RoutingAlgorithm> runner_a;
    std::unique_ptr<transport::RoutingAlgorithm> runner_b;
    try {
        runner_a = transport::make_routing_algorithm(algorithm_a, graph);
        runner_b = transport::make_routing_algorithm(algorithm_b, graph);
    } catch (const std::invalid_argument &err) {
        std::cerr << err.what() << "\n";
        return 1;
    }

    const uint64_t algorithm_a_preprocess_us = preprocess_timed(*runner_a);
    const uint64_t algorithm_b_preprocess_us = preprocess_timed(*runner_b);
    print_preprocessing_metrics(runner_a->name(), *runner_a, algorithm_a_preprocess_us);
    print_preprocessing_metrics(runner_b->name(), *runner_b, algorithm_b_preprocess_us);

    std::mt19937 rng(seed);
    std::uniform_int_distribution<uint32_t> pick(0, graph.vertex_count() - 1);

    fs::create_directories(fs::path(out_path).parent_path());
    std::ofstream out(out_path);
    if (!out) {
        std::cerr << "failed to open output file\n";
        return 1;
    }
    out << "query,source,target,algorithm_a,algorithm_b,distance_scale,"
        << "algorithm_a_units,algorithm_b_units,algorithm_a_settled,algorithm_b_settled,algorithm_a_us,algorithm_b_"
           "us\n";

    uint32_t accepted = 0;
    uint32_t attempts = 0;
    while (accepted < queries && attempts < queries * 100) {
        ++attempts;
        const uint32_t source = pick(rng);
        const uint32_t target = pick(rng);
        if (source == target) {
            continue;
        }

        const TimedResult a = query_timed(*runner_a, source, target);
        const TimedResult b = query_timed(*runner_b, source, target);

        if (a.path.distance_units == UINT64_MAX || a.path.settled < min_settled || a.path.settled > max_settled) {
            continue;
        }
        if (!same_distance(a.path.distance_units, b.path.distance_units)) {
            std::cerr << "distance mismatch for query source=" << source << " target=" << target
                      << " algorithm_a=" << runner_a->name() << " distance=" << a.path.distance_units
                      << " algorithm_b=" << runner_b->name() << " distance=" << b.path.distance_units << "\n";
            return 2;
        }

        write_benchmark_row(out, accepted, source, target,
                            {TimedAlgorithmResult{runner_a->name(), a}, TimedAlgorithmResult{runner_b->name(), b}});
        ++accepted;
    }

    std::cout << "algorithm_a=" << runner_a->name() << "\n";
    std::cout << "algorithm_b=" << runner_b->name() << "\n";
    std::cout << "accepted_queries=" << accepted << "\n";
    std::cout << "attempted_queries=" << attempts << "\n";
    std::cout << "output_csv=" << out_path << "\n";
    return accepted == queries ? 0 : 3;
}
