#include "algorithms/ch/contraction_hierarchy.hpp"
#include "algorithms/routing_algorithm.hpp"
#include "algorithms/routing_algorithm_factory.hpp"
#include "apps/bench_utils.hpp"
#include "graph/graph.hpp"

#include <cstdint>
#include <iostream>
#include <map>
#include <memory>
#include <stdexcept>
#include <string>

static void usage() {
    std::cerr << "usage: transport_measure --graph <path> [--source <pbf>] [--algorithm <name>]\n"
              << "       [--param key=value ...] [--queries N] [--seed S] [--validate N]\n"
              << "       [--graph-slug <name>]\n";
}

int main(int argc, char **argv) {
    bench::BenchmarkArgs args;
    args.graph_slug = "graph";
    std::string algo_name = "ch";
    std::map<std::string, std::string> params;

    for (int i = 1; i < argc; ++i) {
        const std::string key = argv[i];
        if (i + 1 < argc) {
            const std::string value = argv[i + 1];
            if (key == "--graph") {
                args.graph_path = value;
                ++i;
            } else if (key == "--source") {
                args.source_path = value;
                ++i;
            } else if (key == "--algorithm") {
                algo_name = value;
                ++i;
            } else if (key == "--queries") {
                args.query_count = static_cast<uint32_t>(std::stoul(value));
                ++i;
            } else if (key == "--seed") {
                args.seed = static_cast<uint32_t>(std::stoul(value));
                ++i;
            } else if (key == "--validate") {
                args.validate_count = static_cast<uint32_t>(std::stoul(value));
                ++i;
            } else if (key == "--graph-slug") {
                args.graph_slug = value;
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

    if (args.graph_path.empty()) {
        usage();
        return 1;
    }

    const bench::LoadedGraph loaded = bench::load_graph(args);
    std::unique_ptr<transport::RoutingAlgorithm> algo;
    try {
        algo = transport::make_routing_algorithm(algo_name, loaded.graph, params);
    } catch (const std::invalid_argument &err) {
        std::cerr << err.what() << "\n";
        return 1;
    }

    // Algorithm-specific extra fields assembled via dynamic_cast
    auto extra_fields = [&algo]() -> bench::Json {
        bench::Json j;
        if (const auto *ch = dynamic_cast<const transport::ContractionHierarchyAlgorithm *>(algo.get())) {
            const transport::PreprocessStats stats = ch->preprocess_stats();
            j["ordering_init_wall_s"] = bench::to_seconds(stats.ordering_init_ns);
            j["ordering_note"] =
                "initial PQ scoring pass only; lazy re-scores and find_shortcuts are interleaved throughout the full "
                "contraction loop";
            j["witness_calls"] = stats.witness_calls;
            j["witness_hop_limit"] = 5;
            j["witness_note"] =
                "counts all WitnessSearch::run() calls from both edge_difference (ordering) and find_shortcuts "
                "(contraction); no per-call timing to avoid measurement overhead";
            j["auxiliary_edges"] = ch->auxiliary_edge_count();
        }
        return j;
    };

    const std::string variant = algo->variant();
    const std::string out_path =
        bench::run_benchmark_to_file(args, loaded, variant, GIT_COMMIT_HASH, *algo, extra_fields);
    if (out_path.empty()) {
        return 1;
    }

    std::cout << "algorithm=" << algo->name() << "\n";
    std::cout << "variant=" << variant << "\n";
    std::cout << "output=" << out_path << "\n";
    return 0;
}
