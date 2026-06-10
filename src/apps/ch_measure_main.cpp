#include "algorithms/ch/contraction_hierarchy.hpp"
#include "graph/graph.hpp"

#include <algorithm>
#include <cassert>
#include <chrono>
#include <cstdint>
#include <ctime>
#include <iostream>
#include <numeric>
#include <random>
#include <sys/resource.h>
#include <vector>

namespace {

std::chrono::nanoseconds process_cpu_now() {
    using Ticks = std::chrono::duration<std::clock_t, std::ratio<1, CLOCKS_PER_SEC>>;
    return std::chrono::duration_cast<std::chrono::nanoseconds>(Ticks(std::clock()));
}

uint64_t peak_rss_mb() {
    struct rusage ru{};
    getrusage(RUSAGE_SELF, &ru);
    return static_cast<uint64_t>(ru.ru_maxrss) / 1024;
}

struct Stopwatch {
    std::chrono::steady_clock::time_point wall = std::chrono::steady_clock::now();
    std::chrono::nanoseconds cpu = process_cpu_now();

    [[nodiscard]] std::chrono::nanoseconds wall_elapsed() const { return std::chrono::steady_clock::now() - wall; }
    [[nodiscard]] std::chrono::nanoseconds cpu_elapsed() const { return process_cpu_now() - cpu; }
};

void print_phase(const char *label, const Stopwatch &sw) {
    std::cout << label << "_wall_s=" << static_cast<double>(sw.wall_elapsed().count()) / 1e9 << "\n";
    std::cout << label << "_cpu_s=" << static_cast<double>(sw.cpu_elapsed().count()) / 1e9 << "\n";
}

uint64_t percentile(const std::vector<uint64_t> &v, double pct) {
    assert(!v.empty());
    assert(pct >= 0.0 && pct <= 100.0);
    const size_t idx = std::min(static_cast<size_t>(static_cast<double>(v.size()) * pct / 100.0), v.size() - 1);
    return v[idx];
}

} // namespace

int main(int argc, char **argv) {
    if (argc < 2) {
        std::cerr << "usage: ch_measure <graph.graph> [queries=10000] [seed=1]\n";
        return 1;
    }

    const std::string graph_path = argv[1];
    const uint32_t query_count = argc >= 3 ? static_cast<uint32_t>(std::stoul(argv[2])) : 10'000;
    const uint32_t seed = argc >= 4 ? static_cast<uint32_t>(std::stoul(argv[3])) : 1;

    // --- load ---
    const Stopwatch load_sw;
    const transport::Graph graph = transport::load_graph_binary(graph_path);
    print_phase("load", load_sw);
    std::cout << "vertices=" << graph.vertex_count() << "\n";
    std::cout << "edges=" << graph.edge_count() << "\n";
    std::cout << "after_load_peak_rss_mb=" << peak_rss_mb() << "\n";

    // --- preprocess ---
    transport::ContractionHierarchyAlgorithm ch(graph);
    const Stopwatch pp_sw;
    ch.preprocess();
    print_phase("preprocess", pp_sw);
    std::cout << "auxiliary_edges=" << ch.auxiliary_edge_count() << "\n";
    std::cout << "after_preprocess_peak_rss_mb=" << peak_rss_mb() << "\n";

    // --- queries ---
    std::mt19937 rng(seed);
    std::uniform_int_distribution<uint32_t> pick(0, graph.vertex_count() - 1);

    std::vector<uint64_t> times;
    times.reserve(query_count);

    for (uint32_t i = 0; i < query_count; ++i) {
        const uint32_t src = pick(rng);
        const uint32_t dst = pick(rng);
        const Stopwatch t;
        (void)ch.query(src, dst);
        times.push_back(static_cast<uint64_t>(t.wall_elapsed().count()));
    }

    std::sort(times.begin(), times.end());
    const double mean_ns = static_cast<double>(std::accumulate(times.begin(), times.end(), uint64_t{0})) /
                           static_cast<double>(times.size());

    auto ns_to_us = [](uint64_t ns) { return static_cast<double>(ns) / 1000.0; };
    std::cout << "queries=" << query_count << "\n";
    std::cout << "query_mean_us=" << mean_ns / 1000.0 << "\n";
    std::cout << "query_p50_us=" << ns_to_us(percentile(times, 50)) << "\n";
    std::cout << "query_p95_us=" << ns_to_us(percentile(times, 95)) << "\n";
    std::cout << "query_p99_us=" << ns_to_us(percentile(times, 99)) << "\n";
    std::cout << "query_max_us=" << ns_to_us(times.back()) << "\n";

    return 0;
}
