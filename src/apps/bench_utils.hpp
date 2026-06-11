#pragma once

#include "algorithms/dijkstra.hpp"
#include "algorithms/routing_algorithm.hpp"
#include "graph/graph.hpp"
#include "routing/routing.hpp"

#include <algorithm>
#include <cassert>
#include <chrono>
#include <cstdint>
#include <ctime>
#include <filesystem>
#include <fstream>
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

namespace fs = std::filesystem;
using Json = nlohmann::ordered_json;

struct BenchmarkArgs {
    std::string graph_path;
    std::string source_path; // original data source (e.g. OSM PBF) — written to "graph.source" in JSON
    std::string graph_slug;  // short name used in the output filename, e.g. "poland"
    uint32_t query_count = 10'000;
    uint32_t seed = 1;
    uint32_t validate_count = 50; // pairs to cross-check against Dijkstra; 0 = skip
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

inline std::string current_datetime_slug() {
    const std::time_t t = std::time(nullptr);
    std::tm tm{};
    localtime_r(&t, &tm);
    std::ostringstream oss;
    oss << std::put_time(&tm, "%Y%m%dT%H%M%S");
    return oss.str();
}

// Turn a human-readable string into a filename slug (lowercase, non-alnum → '-').
inline std::string slugify(std::string_view s) {
    std::string out;
    out.reserve(s.size());
    for (char c : s) {
        if (std::isalnum(static_cast<unsigned char>(c))) {
            out += static_cast<char>(std::tolower(static_cast<unsigned char>(c)));
        } else if (!out.empty() && out.back() != '-') {
            out += '-';
        }
    }
    while (!out.empty() && out.back() == '-') {
        out.pop_back();
    }
    return out;
}

inline uint64_t percentile_ns(const std::vector<uint64_t> &sorted_ns, double pct) {
    assert(!sorted_ns.empty());
    const size_t idx =
        std::min(static_cast<size_t>(static_cast<double>(sorted_ns.size()) * pct / 100.0), sorted_ns.size() - 1);
    return sorted_ns[idx];
}

template <typename T> inline double vec_mean(const std::vector<T> &v) {
    if (v.empty()) {
        return 0.0;
    }
    return static_cast<double>(std::accumulate(v.begin(), v.end(), T{})) / static_cast<double>(v.size());
}

template <typename T> inline double vec_p50(std::vector<T> v) {
    if (v.empty()) {
        return 0.0;
    }
    std::sort(v.begin(), v.end());
    return static_cast<double>(v[v.size() / 2]);
}

template <typename T> inline double vec_p95(std::vector<T> v) {
    if (v.empty()) {
        return 0.0;
    }
    std::sort(v.begin(), v.end());
    const size_t idx = std::min(static_cast<size_t>(v.size() * 95 / 100), v.size() - 1);
    return static_cast<double>(v[idx]);
}

// --- main entry points ---

inline LoadedGraph load_graph(const BenchmarkArgs &args) {
    const Stopwatch sw;
    transport::Graph graph = transport::load_graph_binary(args.graph_path);
    const std::chrono::nanoseconds wall_ns = sw.wall_elapsed();
    const std::chrono::nanoseconds cpu_ns = sw.cpu_elapsed();
    return LoadedGraph{std::move(graph), wall_ns, cpu_ns, peak_rss_mb()};
}

// Build the output path: results/<algorithm>_<variant-slug>_<graph_slug>_<timestamp>.json
// Appends _2, _3 etc. if a collision would occur (never overwrites).
inline std::string make_result_path(const std::string &algo_name, const std::string &variant_str,
                                    const std::string &graph_slug) {
    const std::string base =
        slugify(algo_name) + "_" + slugify(variant_str) + "_" + slugify(graph_slug) + "_" + current_datetime_slug();
    fs::create_directories("results");
    std::string path = "results/" + base + ".json";
    if (!fs::exists(path)) {
        return path;
    }
    for (int n = 2; n < 10000; ++n) {
        path = "results/" + base + "_" + std::to_string(n) + ".json";
        if (!fs::exists(path)) {
            return path;
        }
    }
    return path; // fallback; extremely unlikely
}

// Preprocesses `algo`, runs the query loop, and writes a complete JSON object to `out`.
// `extra_fields` is called after preprocessing and must return a Json object with no keys
// overlapping the standard fields; its entries are merged before the "queries" block. Use it
// for algorithm-specific metadata (e.g. witness calls, auxiliary edge counts) that are only
// available post-preprocess. Pass {} to omit algorithm-specific fields.
inline bool run_benchmark(const BenchmarkArgs &args, const LoadedGraph &loaded, std::string_view variant,
                          std::string_view commit, transport::RoutingAlgorithm &algo,
                          std::function<Json()> extra_fields = {}, std::ostream &out = std::cout) {
    const Stopwatch pp_sw;
    algo.preprocess();
    const std::chrono::nanoseconds pp_wall_ns = pp_sw.wall_elapsed();
    const std::chrono::nanoseconds pp_cpu_ns = pp_sw.cpu_elapsed();
    const uint64_t after_preprocess_rss = peak_rss_mb();

    std::mt19937 rng(args.seed);
    const uint32_t V = loaded.graph.vertex_count();
    std::uniform_int_distribution<uint32_t> pick(0, V - 1);

    // Per-query recorded data
    struct QueryRecord {
        uint64_t wall_ns;
        transport::QueryStats stats;
        double haversine_km;
    };
    std::vector<QueryRecord> records;
    records.reserve(args.query_count);

    for (uint32_t i = 0; i < args.query_count; ++i) {
        const uint32_t src = pick(rng);
        const uint32_t dst = pick(rng);
        const double hkm = transport::haversine_meters(loaded.graph.coords[src], loaded.graph.coords[dst]) / 1000.0;
        const Stopwatch t;
        const transport::PathResult r = algo.query(src, dst);
        records.push_back(QueryRecord{static_cast<uint64_t>(t.wall_elapsed().count()), r.stats, hkm});
    }

    // Timing stats
    std::vector<uint64_t> times;
    times.reserve(records.size());
    for (const auto &rec : records) {
        times.push_back(rec.wall_ns);
    }
    std::sort(times.begin(), times.end());
    const double mean_us = vec_mean(times) / 1000.0;

    // Per-query stats aggregation
    std::vector<uint32_t> settled_vec, settled_f_vec, settled_b_vec;
    std::vector<uint64_t> relaxed_vec, pushes_vec, heval_vec, pruned_vec, tlookup_vec;
    uint64_t fallback_count = 0;
    settled_vec.reserve(records.size());
    relaxed_vec.reserve(records.size());
    pushes_vec.reserve(records.size());
    for (const auto &rec : records) {
        settled_vec.push_back(rec.stats.settled);
        relaxed_vec.push_back(rec.stats.relaxed_arcs);
        pushes_vec.push_back(rec.stats.heap_pushes);
        if (rec.stats.settled_forward > 0 || rec.stats.settled_backward > 0) {
            settled_f_vec.push_back(rec.stats.settled_forward);
            settled_b_vec.push_back(rec.stats.settled_backward);
        }
        if (rec.stats.heuristic_evals > 0) {
            heval_vec.push_back(rec.stats.heuristic_evals);
        }
        if (rec.stats.pruned_by_flag > 0) {
            pruned_vec.push_back(rec.stats.pruned_by_flag);
        }
        if (rec.stats.table_lookups > 0) {
            tlookup_vec.push_back(rec.stats.table_lookups);
        }
        if (rec.stats.used_fallback) {
            ++fallback_count;
        }
    }

    Json qstats;
    qstats["settled_mean"] = vec_mean(settled_vec);
    qstats["settled_p50"] = vec_p50(settled_vec);
    qstats["settled_p95"] = vec_p95(settled_vec);
    qstats["relaxed_arcs_mean"] = vec_mean(relaxed_vec);
    qstats["heap_pushes_mean"] = vec_mean(pushes_vec);
    if (!settled_f_vec.empty()) {
        qstats["settled_forward_mean"] = vec_mean(settled_f_vec);
        qstats["settled_backward_mean"] = vec_mean(settled_b_vec);
    }
    if (!heval_vec.empty()) {
        qstats["heuristic_evals_mean"] = vec_mean(heval_vec);
    }
    if (!pruned_vec.empty()) {
        qstats["pruned_by_flag_mean"] = vec_mean(pruned_vec);
    }
    if (!tlookup_vec.empty()) {
        qstats["table_lookups_mean"] = vec_mean(tlookup_vec);
    }
    if (fallback_count > 0) {
        qstats["fallback_fraction"] = static_cast<double>(fallback_count) / static_cast<double>(records.size());
    }

    // Distance bucket breakdown
    struct DistBucket {
        double lo, hi;
        std::string label;
        std::vector<uint64_t> t_ns;
    };
    std::vector<DistBucket> buckets = {{0, 10, "[0,10)km", {}},
                                       {10, 50, "[10,50)km", {}},
                                       {50, 200, "[50,200)km", {}},
                                       {200, 1e18, "[200,inf)km", {}}};
    for (const auto &rec : records) {
        for (auto &b : buckets) {
            if (rec.haversine_km >= b.lo && rec.haversine_km < b.hi) {
                b.t_ns.push_back(rec.wall_ns);
                break;
            }
        }
    }
    Json dist_buckets = Json::array();
    for (const auto &b : buckets) {
        if (b.t_ns.empty()) {
            continue;
        }
        std::vector<uint64_t> sorted_b = b.t_ns;
        std::sort(sorted_b.begin(), sorted_b.end());
        const double bmean = vec_mean(sorted_b) / 1000.0;
        const double bp50 = static_cast<double>(sorted_b[sorted_b.size() / 2]) / 1000.0;
        Json bobj;
        bobj["bucket"] = b.label;
        bobj["count"] = b.t_ns.size();
        bobj["mean_us"] = bmean;
        bobj["p50_us"] = bp50;
        dist_buckets.push_back(std::move(bobj));
    }

    // Validation against Dijkstra reference
    uint32_t validation_mismatches = 0;
    std::string validation_note;
    if (args.validate_count > 0 && algo.name() != "dijkstra") {
        const transport::DijkstraAlgorithm ref(loaded.graph);
        std::mt19937 val_rng(args.seed);
        std::uniform_int_distribution<uint32_t> val_pick(0, V - 1);
        const uint32_t to_check = std::min(args.validate_count, args.query_count);
        for (uint32_t i = 0; i < to_check; ++i) {
            const uint32_t src = val_pick(val_rng);
            const uint32_t dst = val_pick(val_rng);
            const transport::Distance ref_dist = ref.query(src, dst).distance_units;
            const transport::Distance got_dist = algo.query(src, dst).distance_units;
            if (ref_dist != got_dist) {
                ++validation_mismatches;
                std::cerr << "VALIDATION MISMATCH src=" << src << " dst=" << dst << " ref=" << ref_dist
                          << " got=" << got_dist << "\n";
            }
        }
        if (validation_mismatches > 0) {
            validation_note = "mismatches detected — results may be incorrect";
        }
    }

    // Assemble JSON
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
    j["query_stats"] = std::move(qstats);
    j["queries_by_distance"] = std::move(dist_buckets);
    j["queries_by_distance_note"] = "buckets by straight-line haversine(s,t) as a cheap Dijkstra-rank proxy";

    Json val_obj;
    val_obj["reference"] = "dijkstra";
    val_obj["count"] = (args.validate_count > 0 && algo.name() != "dijkstra")
                           ? static_cast<uint32_t>(std::min(args.validate_count, args.query_count))
                           : 0u;
    val_obj["mismatches"] = validation_mismatches;
    if (!validation_note.empty()) {
        val_obj["validation_note"] = validation_note;
    }
    j["validation"] = std::move(val_obj);

    out << j.dump(2) << "\n";
    return validation_mismatches == 0;
}

// Convenience overload that writes to a file in results/ and returns the path.
inline std::string run_benchmark_to_file(const BenchmarkArgs &args, const LoadedGraph &loaded, std::string_view variant,
                                         std::string_view commit, transport::RoutingAlgorithm &algo,
                                         std::function<Json()> extra_fields = {}) {
    const std::string path = make_result_path(std::string(algo.name()), std::string(variant), args.graph_slug);
    std::ofstream file(path);
    if (!file) {
        std::cerr << "failed to open results file: " << path << "\n";
        return "";
    }
    const bool ok = run_benchmark(args, loaded, variant, commit, algo, extra_fields, file);
    file.close();
    if (!ok) {
        std::cerr << "validation failed — results written to " << path << "\n";
    } else {
        std::cout << "results written to " << path << "\n";
    }
    return path;
}

} // namespace bench
