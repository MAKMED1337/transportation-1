#pragma once

#include "algorithms/routing_algorithm.hpp"
#include "graph/graph.hpp"

#include <algorithm>
#include <cassert>
#include <chrono>
#include <cstdint>
#include <ctime>
#include <functional>
#include <iomanip>
#include <iostream>
#include <nlohmann/json.hpp>
#include <numeric>
#include <ostream>
#include <random>
#include <sstream>
#include <string>
#include <sys/resource.h>
#include <vector>

namespace bench {

using Json = nlohmann::ordered_json;

struct BenchmarkArgs {
    std::string graph_path;
    std::string source_path; // original data source (e.g. OSM PBF) — written to "graph.source" in JSON
    uint32_t query_count = 10'000;
    uint32_t seed = 1;
};

// Graph together with its load timing and post-load RSS, ready to hand to a RoutingAlgorithm constructor.
struct LoadedGraph {
    transport::Graph graph;
    std::chrono::nanoseconds wall_ns{};
    std::chrono::nanoseconds cpu_ns{};
    uint64_t peak_rss_mb = 0;
};

// --- helpers ---

inline std::chrono::nanoseconds process_cpu_now() {
    using Ticks = std::chrono::duration<std::clock_t, std::ratio<1, CLOCKS_PER_SEC>>;
    return std::chrono::duration_cast<std::chrono::nanoseconds>(Ticks(std::clock()));
}

struct Stopwatch {
    std::chrono::steady_clock::time_point wall_start = std::chrono::steady_clock::now();
    std::chrono::nanoseconds cpu_start = process_cpu_now();

    [[nodiscard]] std::chrono::nanoseconds wall_elapsed() const {
        return std::chrono::steady_clock::now() - wall_start;
    }
    [[nodiscard]] std::chrono::nanoseconds cpu_elapsed() const { return process_cpu_now() - cpu_start; }
};

inline double to_seconds(std::chrono::nanoseconds d) { return std::chrono::duration<double>(d).count(); }

inline double to_microseconds(std::chrono::nanoseconds d) {
    return std::chrono::duration<double, std::micro>(d).count();
}

inline uint64_t peak_rss_mb() {
    struct rusage ru{};
    getrusage(RUSAGE_SELF, &ru);
    return static_cast<uint64_t>(ru.ru_maxrss) / 1024;
}

inline std::string current_datetime_iso() {
    const std::time_t t = std::time(nullptr);
    std::tm tm{};
    localtime_r(&t, &tm);
    std::ostringstream oss;
    oss << std::put_time(&tm, "%Y-%m-%dT%H:%M:%S");
    return oss.str();
}

inline uint64_t percentile_ns(const std::vector<uint64_t> &sorted_ns, double pct) {
    assert(!sorted_ns.empty());
    const size_t idx =
        std::min(static_cast<size_t>(static_cast<double>(sorted_ns.size()) * pct / 100.0), sorted_ns.size() - 1);
    return sorted_ns[idx];
}

// --- main entry points ---

inline LoadedGraph load_graph(const BenchmarkArgs &args) {
    const Stopwatch sw;
    transport::Graph graph = transport::load_graph_binary(args.graph_path);
    const std::chrono::nanoseconds wall_ns = sw.wall_elapsed();
    const std::chrono::nanoseconds cpu_ns = sw.cpu_elapsed();
    return LoadedGraph{std::move(graph), wall_ns, cpu_ns, peak_rss_mb()};
}

// Preprocesses `algo`, runs the query loop, and writes a complete JSON object to `out`.
// `extra_fields` is called after preprocessing and must return a Json object with no keys
// overlapping the standard fields; its entries are merged before the "queries" block. Use it
// for algorithm-specific metadata (e.g. witness calls, auxiliary edge counts) that are only
// available post-preprocess. Pass {} to omit algorithm-specific fields.
inline void run_benchmark(const BenchmarkArgs &args, const LoadedGraph &loaded, std::string_view variant,
                          std::string_view commit, transport::RoutingAlgorithm &algo,
                          std::function<Json()> extra_fields = {}, std::ostream &out = std::cout) {
    const Stopwatch pp_sw;
    algo.preprocess();
    const std::chrono::nanoseconds pp_wall_ns = pp_sw.wall_elapsed();
    const std::chrono::nanoseconds pp_cpu_ns = pp_sw.cpu_elapsed();
    const uint64_t after_preprocess_rss = peak_rss_mb();

    std::mt19937 rng(args.seed);
    std::uniform_int_distribution<uint32_t> pick(0, loaded.graph.vertex_count() - 1);

    std::vector<uint64_t> times;
    times.reserve(args.query_count);
    for (uint32_t i = 0; i < args.query_count; ++i) {
        const uint32_t src = pick(rng);
        const uint32_t dst = pick(rng);
        const Stopwatch t;
        (void)algo.query(src, dst);
        times.push_back(static_cast<uint64_t>(t.wall_elapsed().count()));
    }
    std::sort(times.begin(), times.end());
    const double mean_us = static_cast<double>(std::accumulate(times.begin(), times.end(), uint64_t{0})) /
                           static_cast<double>(times.size()) / 1000.0;

    Json graph_obj;
    graph_obj["path"] = args.graph_path;
    if (!args.source_path.empty()) {
        graph_obj["source"] = args.source_path;
    }
    graph_obj["vertices"] = loaded.graph.vertex_count();
    graph_obj["directed_edges"] = loaded.graph.edge_count();

    Json j;
    j["algorithm"] = algo.name();
    j["variant"] = variant;
    j["commit"] = commit;
    j["date"] = current_datetime_iso();
    j["graph"] = std::move(graph_obj);
    j["load_wall_s"] = to_seconds(loaded.wall_ns);
    j["load_cpu_s"] = to_seconds(loaded.cpu_ns);
    j["after_load_peak_rss_mb"] = loaded.peak_rss_mb;
    j["preprocess_wall_s"] = to_seconds(pp_wall_ns);
    j["preprocess_cpu_s"] = to_seconds(pp_cpu_ns);
    j["after_preprocess_peak_rss_mb"] = after_preprocess_rss;
    if (extra_fields) {
        const Json extra = extra_fields();
        for (const auto &[key, val] : extra.items()) {
            assert(!j.contains(key) && "extra_fields key conflicts with a standard benchmark field");
            assert(key != "queries" && "extra_fields must not use the reserved key \"queries\"");
            (void)val;
        }
        j.update(extra);
    }
    j["queries"] = Json{
        {"count", args.query_count},
        {"seed", args.seed},
        {"mean_us", mean_us},
        {"p50_us", to_microseconds(std::chrono::nanoseconds(percentile_ns(times, 50)))},
        {"p95_us", to_microseconds(std::chrono::nanoseconds(percentile_ns(times, 95)))},
        {"p99_us", to_microseconds(std::chrono::nanoseconds(percentile_ns(times, 99)))},
        {"max_us", to_microseconds(std::chrono::nanoseconds(times.back()))},
    };

    out << j.dump(2) << "\n";
}

} // namespace bench
