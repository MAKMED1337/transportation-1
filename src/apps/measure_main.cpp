#include "algorithms/arcflags/arc_flags.hpp"
#include "algorithms/ch/ch_io.hpp"
#include "algorithms/ch/contraction_hierarchy.hpp"
#include "algorithms/chase/chase.hpp"
#include "algorithms/hl/hub_labels.hpp"
#include "algorithms/routing_algorithm.hpp"
#include "algorithms/routing_algorithm_factory.hpp"
#include "algorithms/tnr/tnr.hpp"
#include "algorithms/tnr/tnr_af.hpp"
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
    auto *af_algo = dynamic_cast<transport::ArcFlagsAlgorithm *>(algo.get());
    auto *chase_algo = dynamic_cast<transport::ChaseAlgorithm *>(algo.get());
    auto *tnr_algo = dynamic_cast<transport::TnrAlgorithm *>(algo.get());
    auto *tnr_af_algo = dynamic_cast<transport::TnrAfAlgorithm *>(algo.get());
    auto *hl_algo = dynamic_cast<transport::HubLabelsAlgorithm *>(algo.get());
    if (!ch_file.empty() && std::filesystem::exists(ch_file)) {
        try {
            auto loaded_ch = transport::ch::load_ch(ch_file);
            if (ch_algo) {
                ch_algo->inject_ch(std::move(loaded_ch));
                ch_file_existed = true;
                std::cout << "CH loaded from " << ch_file << "\n";
            } else if (af_algo) {
                af_algo->inject_ch(std::move(loaded_ch));
                ch_file_existed = true;
                std::cout << "CH loaded from " << ch_file << "\n";
            } else if (chase_algo) {
                chase_algo->inject_ch(std::move(loaded_ch));
                ch_file_existed = true;
                std::cout << "CH loaded from " << ch_file << "\n";
            } else if (tnr_algo) {
                tnr_algo->inject_ch(std::move(loaded_ch));
                ch_file_existed = true;
                std::cout << "CH loaded from " << ch_file << "\n";
            } else if (hl_algo) {
                hl_algo->inject_ch(std::move(loaded_ch));
                ch_file_existed = true;
                std::cout << "CH loaded from " << ch_file << "\n";
            }
        } catch (const std::exception &e) {
            std::cerr << "warning: failed to load CH from " << ch_file << ": " << e.what() << "\n";
        }
    }

    const bool ch_fields = (ch_algo != nullptr);
    const bool af_fields = (af_algo != nullptr);
    const bool chase_fields = (chase_algo != nullptr);
    const bool tnr_fields = (tnr_algo != nullptr);
    const bool tnr_af_fields = (tnr_af_algo != nullptr);
    const bool hl_fields = (hl_algo != nullptr);
    const bool ch_loaded = ch_file_existed;
    const std::string ch_path_copy = ch_file;

    auto extra_fields = [&algo, ch_fields, af_fields, chase_fields, tnr_fields, tnr_af_fields, hl_fields, ch_loaded,
                         &ch_path_copy]() -> bench::Json {
        bench::Json j;

        if (chase_fields) {
            const auto *chase = dynamic_cast<const transport::ChaseAlgorithm *>(algo.get());
            if (chase) {
                const auto &s = chase->chase_stats();
                j["core_fraction"] = s.core_fraction;
                j["core_vertices"] = s.core_vertices;
                j["core_arcs_fwd"] = s.core_arcs_fwd;
                j["core_arcs_bwd"] = s.core_arcs_bwd;
                j["region_count"] = s.region_count;
                j["core_boundary_fwd"] = s.core_boundary_fwd;
                j["core_boundary_bwd"] = s.core_boundary_bwd;
                j["flags_mb"] = s.flags_mb;
                j["flags_wall_s"] = s.flags_wall_s;
                j["flags_cpu_s"] = s.flags_cpu_s;
                j["total_preprocess_wall_s"] = s.total_wall_s;
                j["total_preprocess_cpu_s"] = s.total_cpu_s;
                j["ch_was_injected"] = s.ch_was_injected;
                if (s.ch_was_injected) {
                    j["ch_loaded_from_file"] = ch_path_copy;
                }
            }
            return j;
        }

        if (tnr_fields) {
            const auto *tnr = dynamic_cast<const transport::TnrAlgorithm *>(algo.get());
            if (tnr) {
                const auto &s = tnr->tnr_stats();
                j["transit_nodes"] = s.transit_nodes;
                j["dt_table_mb"] = s.dt_table_mb;
                j["dt_build_wall_s"] = s.dt_build_wall_s;
                j["dt_build_cpu_s"] = s.dt_build_cpu_s;
                j["access_nodes_wall_s"] = s.access_nodes_wall_s;
                j["access_nodes_cpu_s"] = s.access_nodes_cpu_s;
                j["avg_access_fwd"] = s.avg_access_fwd;
                j["avg_access_bwd"] = s.avg_access_bwd;
                j["max_access_fwd"] = s.max_access_fwd;
                j["max_access_bwd"] = s.max_access_bwd;
                j["access_storage_mb"] = s.access_storage_mb;
                j["voronoi_wall_s"] = s.voronoi_wall_s;
                j["voronoi_cpu_s"] = s.voronoi_cpu_s;
                j["locality_filter_mb"] = s.locality_filter_mb;
                j["total_preprocess_wall_s"] = s.total_wall_s;
                j["total_preprocess_cpu_s"] = s.total_cpu_s;
                j["ch_was_injected"] = s.ch_was_injected;
                if (s.ch_was_injected) {
                    j["ch_loaded_from_file"] = ch_path_copy;
                }
            }
            if (tnr_af_fields) {
                const auto *tnr_af = dynamic_cast<const transport::TnrAfAlgorithm *>(algo.get());
                if (tnr_af) {
                    const auto &af = tnr_af->tnr_af_stats();
                    j["af_build_wall_s"] = af.af_build_wall_s;
                    j["af_build_cpu_s"] = af.af_build_cpu_s;
                    j["af_flags_mb"] = af.af_flags_mb;
                    j["af_pruned_fraction"] = af.af_pruned_fraction;
                }
            }
            return j;
        }

        if (hl_fields) {
            const auto *hl = dynamic_cast<const transport::HubLabelsAlgorithm *>(algo.get());
            if (hl) {
                const auto &s = hl->hl_stats();
                j["label_fraction"] = s.label_fraction;
                j["labeled_vertices"] = s.labeled_vertices;
                j["avg_label_size_fwd"] = s.avg_label_size_fwd;
                j["avg_label_size_bwd"] = s.avg_label_size_bwd;
                j["max_label_size_fwd"] = s.max_label_size_fwd;
                j["max_label_size_bwd"] = s.max_label_size_bwd;
                j["label_memory_mb"] = s.label_memory_mb;
                j["label_build_wall_s"] = s.label_build_wall_s;
                j["label_build_cpu_s"] = s.label_build_cpu_s;
                j["prune_drops"] = s.prune_drops;
                j["ch_was_injected"] = s.ch_was_injected;
                if (s.ch_was_injected) {
                    j["ch_loaded_from_file"] = ch_path_copy;
                }
            }
            return j;
        }

        if (af_fields) {
            const auto *af = dynamic_cast<const transport::ArcFlagsAlgorithm *>(algo.get());
            if (af) {
                const auto &s = af->arcflags_stats();
                j["region_count"] = s.region_count;
                j["boundary_vertices"] = s.boundary_vertices;
                j["trees_computed"] = s.trees_computed;
                j["flags_mb"] = s.flags_mb;
                j["flag_density"] = s.flag_density;
                j["partition_wall_s"] = s.partition_wall_s;
                j["phast_wall_s"] = s.phast_wall_s;
                j["phast_cpu_s"] = s.phast_cpu_s;
                j["total_preprocess_wall_s"] = s.total_wall_s;
                j["total_preprocess_cpu_s"] = s.total_cpu_s;
                j["ch_was_injected"] = s.ch_was_injected;
                if (!s.preprocess_note.empty()) {
                    j["preprocess_note"] = s.preprocess_note;
                }
                if (s.ch_was_injected) {
                    j["ch_loaded_from_file"] = ch_path_copy;
                }
            }
            return j;
        }

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
    const transport::ContractionHierarchy *ch_to_save = nullptr;
    if (ch_algo) {
        ch_to_save = &ch_algo->get_ch();
    } else if (af_algo) {
        ch_to_save = &af_algo->get_ch();
    } else if (chase_algo) {
        ch_to_save = &chase_algo->get_ch();
    } else if (tnr_algo) {
        ch_to_save = &tnr_algo->get_ch();
    } else if (hl_algo) {
        ch_to_save = &hl_algo->get_ch();
    }
    if (ch_to_save && !ch_file.empty() && !ch_file_existed) {
        if (!transport::ch::save_ch(*ch_to_save, ch_file)) {
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
