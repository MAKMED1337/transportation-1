#include "algorithms/ch/ch_io.hpp"
#include "algorithms/ch/contraction_hierarchy.hpp"
#include "algorithms/routing_algorithm.hpp"
#include "algorithms/routing_algorithm_factory.hpp"
#include "apps/bench_utils.hpp"
#include "graph/graph.hpp"

#include <cstdint>
#include <filesystem>
#include <iostream>
#include <map>
#include <memory>
#include <stdexcept>
#include <string>

static void usage() {
    std::cerr << "usage: transport_measure --graph <path> [--source <pbf>] [--algorithm <name>]\n"
              << "       [--param key=value ...] [--queries N] [--seed S] [--validate N]\n"
              << "       [--graph-slug <name>] [--ch-file <path>]\n";
}

int main(int argc, char **argv) {
    bench::BenchmarkArgs args;
    args.graph_slug = "graph";
    std::string algo_name = "ch";
    std::map<std::string, std::string> params;
    std::string ch_file;

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
            } else if (key == "--ch-file") {
                ch_file = value;
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

    bool ch_file_existed = false;
    auto *ch_algo = dynamic_cast<transport::ContractionHierarchyAlgorithm *>(algo.get());
    if (ch_algo && !ch_file.empty()) {
        if (std::filesystem::exists(ch_file)) {
            try {
                ch_algo->inject_ch(transport::ch::load_ch(ch_file));
                ch_file_existed = true;
                std::cout << "CH loaded from " << ch_file << "\n";
            } catch (const std::exception &e) {
                std::cerr << "warning: failed to load CH from " << ch_file << ": " << e.what() << "\n";
            }
        }
    }

    const bool ch_fields = (ch_algo != nullptr);
    const bool ch_loaded = ch_file_existed;
    const std::string ch_path_copy = ch_file;

    auto extra_fields = [&algo, ch_fields, ch_loaded, &ch_path_copy]() -> bench::Json {
        bench::Json j;
        if (!ch_fields) {
            return j;
        }
        const auto *ch = dynamic_cast<const transport::ContractionHierarchyAlgorithm *>(algo.get());
        if (!ch) {
            return j;
        }

        const transport::PreprocessStats stats = ch->preprocess_stats();
        const transport::ch::OrderParams &op = ch->order_params();

        if (ch_loaded) {
            j["ch_loaded_from_file"] = ch_path_copy;
            j["ch_loaded_note"] = "preprocess_wall_s reflects no-op load path; contraction timing unavailable";
        }

        bench::Json order_weights;
        order_weights["w_edge_diff"] = op.w_edge_diff;
        order_weights["w_deleted_neighbors"] = op.w_deleted_neighbors;
        order_weights["w_depth"] = op.w_depth;
        order_weights["w_original_edges"] = op.w_original_edges;
        order_weights["w_search_space"] = op.w_search_space;
        order_weights["w_voronoi"] = op.w_voronoi;
        j["order_weights"] = std::move(order_weights);

        bench::Json hop_stages = bench::Json::array();
        for (const auto &hs : op.hop_stages) {
            bench::Json hs_obj;
            hs_obj["threshold"] = hs.mean_degree_threshold;
            hs_obj["hop_limit"] = hs.hop_limit;
            hop_stages.push_back(std::move(hs_obj));
        }
        j["hop_stages"] = std::move(hop_stages);

        if (!stats.hop_stage_switches.empty()) {
            bench::Json switches = bench::Json::array();
            for (const auto &[cnt, hl] : stats.hop_stage_switches) {
                bench::Json sw;
                sw["contracted_at"] = cnt;
                sw["new_hop_limit"] = hl;
                switches.push_back(std::move(sw));
            }
            j["hop_stage_switches"] = std::move(switches);
        }

        if (!ch_loaded) {
            j["ordering_init_wall_s"] = bench::to_seconds(stats.ordering_init_ns);
            j["ordering_note"] = "initial PQ scoring pass only; lazy re-scores interleaved throughout contraction";
            j["shortcuts_added"] = stats.shortcuts_added;
            j["witness_calls"] = stats.witness_calls;
            j["witness_note"] =
                "counts all WitnessSearch::run() calls from both compute_priority (ordering) and find_shortcuts "
                "(contraction); no per-call timing to avoid measurement overhead";
        }
        j["auxiliary_edges"] = ch->auxiliary_edge_count();
        return j;
    };

    const std::string variant_str = algo->variant();
    const std::string out_path =
        bench::run_benchmark_to_file(args, loaded, variant_str, GIT_COMMIT_HASH, *algo, extra_fields);
    if (out_path.empty()) {
        return 1;
    }

    // Save CH after benchmarking if file was requested but didn't exist yet.
    if (ch_algo && !ch_file.empty() && !ch_file_existed) {
        if (!transport::ch::save_ch(ch_algo->get_ch(), ch_file)) {
            std::cerr << "warning: failed to save CH to " << ch_file << "\n";
        } else {
            std::cout << "CH saved to " << ch_file << "\n";
        }
    }

    std::cout << "algorithm=" << algo->name() << "\n";
    std::cout << "variant=" << variant_str << "\n";
    std::cout << "output=" << out_path << "\n";
    return 0;
}
