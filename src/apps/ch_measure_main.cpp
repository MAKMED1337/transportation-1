#include "algorithms/ch/contraction_hierarchy.hpp"
#include "apps/bench_utils.hpp"

#include <cstdint>
#include <iostream>
#include <ostream>
#include <string>

int main(int argc, char **argv) {
    if (argc < 2) {
        std::cerr << "usage: ch_measure <graph.graph> [queries=10000] [seed=1] [source_file]\n";
        return 1;
    }

    bench::BenchmarkArgs args;
    args.graph_path = argv[1];
    if (argc >= 3) {
        args.query_count = static_cast<uint32_t>(std::stoul(argv[2]));
    }
    if (argc >= 4) {
        args.seed = static_cast<uint32_t>(std::stoul(argv[3]));
    }
    if (argc >= 5) {
        args.source_path = argv[4];
    }

    auto loaded = bench::load_graph(args);
    transport::ContractionHierarchyAlgorithm ch(loaded.graph);

    bench::run_benchmark(
        args, loaded, "ch", "lazy edge-difference ordering, witness hop_limit=5", GIT_COMMIT_HASH, ch,
        [&ch](std::ostream &out) {
            const transport::PreprocessStats stats = ch.preprocess_stats();
            out << "  \"ordering_init_wall_s\": " << bench::to_seconds(stats.ordering_init_ns) << ",\n";
            out << "  \"ordering_note\": \"initial PQ scoring pass only; lazy re-scores and find_shortcuts are"
                   " interleaved throughout the full contraction loop\",\n";
            out << "  \"witness_calls\": " << stats.witness_calls << ",\n";
            out << "  \"witness_hop_limit\": 5,\n";
            out << "  \"witness_note\": \"counts all WitnessSearch::run() calls from both edge_difference"
                   " (ordering) and find_shortcuts (contraction); no per-call timing to avoid measurement"
                   " overhead\",\n";
            out << "  \"auxiliary_edges\": " << ch.auxiliary_edge_count() << ",\n";
        });

    return 0;
}
