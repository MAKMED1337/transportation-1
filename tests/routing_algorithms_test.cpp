#include "algorithms/astar.hpp"
#include "algorithms/ch/ch_io.hpp"
#include "algorithms/ch/contraction_hierarchy.hpp"
#include "algorithms/dijkstra.hpp"
#include "algorithms/phast.hpp"
#include "algorithms/routing_algorithm_factory.hpp"
#include "graph/graph.hpp"

#include <cmath>
#include <cstdint>
#include <filesystem>
#include <iostream>
#include <limits>
#include <map>
#include <stdexcept>
#include <string>
#include <utility>
#include <vector>

namespace {

using Config = std::pair<std::string, transport::AlgorithmParams>;

transport::Graph make_graph(uint32_t vertices, const std::vector<std::vector<transport::Edge>> &rows) {
    transport::Graph graph;
    graph.coords.resize(vertices);
    graph.offsets.assign(static_cast<size_t>(vertices) + 1, 0);
    for (uint32_t v = 0; v < vertices; ++v) {
        graph.offsets[v + 1] = graph.offsets[v] + static_cast<uint64_t>(rows[v].size());
    }
    for (const std::vector<transport::Edge> &row : rows) {
        graph.edges.insert(graph.edges.end(), row.begin(), row.end());
    }
    return graph;
}

// 6x6 grid with approximate lat/lon coords and haversine-based edge weights
transport::Graph make_geo_grid(uint32_t rows, uint32_t cols) {
    const uint32_t V = rows * cols;
    transport::Graph g;
    g.coords.resize(V);
    g.offsets.assign(static_cast<size_t>(V) + 1, 0);

    const double lat0 = 50.0;
    const double lon0 = 20.0;
    const double dlat = 0.01;
    const double dlon = 0.01;

    for (uint32_t r = 0; r < rows; ++r) {
        for (uint32_t c = 0; c < cols; ++c) {
            const uint32_t v = r * cols + c;
            g.coords[v] =
                transport::NodeCoord{lat0 + static_cast<double>(r) * dlat, lon0 + static_cast<double>(c) * dlon};
        }
    }

    // Build adjacency: 4-directional grid (all directed both ways)
    std::vector<std::vector<transport::Edge>> adj(V);
    auto add_edge = [&](uint32_t from, uint32_t to) {
        const double dist_m = transport::haversine_meters(g.coords[from], g.coords[to]);
        const transport::Weight w =
            static_cast<transport::Weight>(::ceil(dist_m * static_cast<double>(transport::kDistanceScale)));
        adj[from].push_back({to, w});
    };
    for (uint32_t r = 0; r < rows; ++r) {
        for (uint32_t c = 0; c < cols; ++c) {
            const uint32_t v = r * cols + c;
            if (r + 1 < rows) {
                add_edge(v, v + cols);
                add_edge(v + cols, v);
            }
            if (c + 1 < cols) {
                add_edge(v, v + 1);
                add_edge(v + 1, v);
            }
        }
    }

    uint64_t off = 0;
    for (uint32_t v = 0; v < V; ++v) {
        g.offsets[v] = off;
        off += static_cast<uint64_t>(adj[v].size());
        g.edges.insert(g.edges.end(), adj[v].begin(), adj[v].end());
    }
    g.offsets[V] = off;
    return g;
}

bool check_all_pairs(const transport::Graph &graph, const std::vector<Config> &configs) {
    const transport::DijkstraAlgorithm dijkstra(graph);
    std::vector<std::unique_ptr<transport::RoutingAlgorithm>> algos;
    for (const auto &[name, params] : configs) {
        auto algo = transport::make_routing_algorithm(name, graph, params);
        algo->preprocess();
        algos.push_back(std::move(algo));
    }

    bool ok = true;
    for (uint32_t s = 0; s < graph.vertex_count(); ++s) {
        for (uint32_t t = 0; t < graph.vertex_count(); ++t) {
            const transport::Distance ref = dijkstra.query(s, t).distance_units;
            for (size_t i = 0; i < algos.size(); ++i) {
                const transport::Distance got = algos[i]->query(s, t).distance_units;
                if (ref != got) {
                    std::cerr << "mismatch s=" << s << " t=" << t << " dijkstra=" << ref << " " << algos[i]->name()
                              << "=" << got << "\n";
                    ok = false;
                }
            }
        }
    }
    return ok;
}

transport::Graph make_invalid_offsets_graph() {
    transport::Graph graph;
    graph.coords.resize(2);
    graph.offsets = {0, 2, 1};
    graph.edges.push_back(transport::Edge{.to = 1, .weight = 100});
    return graph;
}

transport::Graph make_invalid_edge_destination_graph() {
    transport::Graph graph;
    graph.coords.resize(2);
    graph.offsets = {0, 1, 1};
    graph.edges.push_back(transport::Edge{.to = 5, .weight = 100});
    return graph;
}

bool expect_load_failure(const std::filesystem::path &path) {
    try {
        (void)transport::load_graph_binary(path.string());
    } catch (const std::runtime_error &) {
        return true;
    }
    std::cerr << "expected malformed graph load failure for " << path << "\n";
    return false;
}

bool check_ch_io_roundtrip(const transport::Graph &graph) {
    const std::filesystem::path tmp = std::filesystem::temp_directory_path() / "transport_ch_test.ch";
    transport::ContractionHierarchyAlgorithm algo(graph);
    algo.preprocess();

    if (!transport::ch::save_ch(algo.get_ch(), tmp.string())) {
        std::cerr << "ch_io: save failed\n";
        return false;
    }
    transport::ContractionHierarchyAlgorithm loaded(graph);
    try {
        loaded.inject_ch(transport::ch::load_ch(tmp.string()));
    } catch (const std::exception &e) {
        std::cerr << "ch_io: load failed: " << e.what() << "\n";
        std::filesystem::remove(tmp);
        return false;
    }
    std::filesystem::remove(tmp);

    const transport::DijkstraAlgorithm ref(graph);
    const uint32_t V = graph.vertex_count();
    for (uint32_t s = 0; s < V; ++s) {
        for (uint32_t t = 0; t < V; ++t) {
            const transport::Distance expected = ref.query(s, t).distance_units;
            const transport::Distance got = loaded.query(s, t).distance_units;
            if (expected != got) {
                std::cerr << "ch_io roundtrip mismatch s=" << s << " t=" << t << " expected=" << expected
                          << " got=" << got << "\n";
                return false;
            }
        }
    }
    return true;
}

// Verify phast_all_to_one(t)[v] == dijkstra(v,t) and phast_one_to_all(s)[v] == dijkstra(s,v)
bool check_phast_correctness(const transport::Graph &graph) {
    transport::ContractionHierarchyAlgorithm ch_algo(graph);
    ch_algo.preprocess();
    const transport::ContractionHierarchy &ch = ch_algo.get_ch();
    const auto inv_rank = transport::build_inv_rank(ch);

    const transport::DijkstraAlgorithm ref(graph);
    const uint32_t V = graph.vertex_count();
    std::vector<uint32_t> dist(V);
    bool ok = true;

    for (uint32_t t = 0; t < V; ++t) {
        transport::phast_all_to_one(ch, inv_rank, t, dist);
        for (uint32_t v = 0; v < V; ++v) {
            const transport::Distance expected = ref.query(v, t).distance_units;
            const transport::Distance got = (dist[v] == std::numeric_limits<uint32_t>::max())
                                                ? transport::kUnreachable
                                                : static_cast<transport::Distance>(dist[v]);
            if (expected != got) {
                std::cerr << "phast_all_to_one mismatch v=" << v << " t=" << t << " expected=" << expected
                          << " got=" << got << "\n";
                ok = false;
            }
        }
    }

    for (uint32_t s = 0; s < V; ++s) {
        transport::phast_one_to_all(ch, inv_rank, s, dist);
        for (uint32_t v = 0; v < V; ++v) {
            const transport::Distance expected = ref.query(s, v).distance_units;
            const transport::Distance got = (dist[v] == std::numeric_limits<uint32_t>::max())
                                                ? transport::kUnreachable
                                                : static_cast<transport::Distance>(dist[v]);
            if (expected != got) {
                std::cerr << "phast_one_to_all mismatch s=" << s << " v=" << v << " expected=" << expected
                          << " got=" << got << "\n";
                ok = false;
            }
        }
    }

    return ok;
}

bool check_malformed_graph_files_fail_fast() {
    const std::filesystem::path dir = std::filesystem::temp_directory_path();
    const std::filesystem::path invalid_offsets = dir / "transport_invalid_offsets.graph";
    const std::filesystem::path invalid_edge = dir / "transport_invalid_edge.graph";

    if (!transport::save_graph_binary(make_invalid_offsets_graph(), invalid_offsets.string()) ||
        !transport::save_graph_binary(make_invalid_edge_destination_graph(), invalid_edge.string())) {
        std::cerr << "failed to write malformed graph fixtures\n";
        return false;
    }

    const bool ok = expect_load_failure(invalid_offsets) && expect_load_failure(invalid_edge);
    std::filesystem::remove(invalid_offsets);
    std::filesystem::remove(invalid_edge);
    return ok;
}

} // namespace

int main() {
    // Configs to test on every graph fixture
    const std::vector<Config> configs = {
        {"astar", {}},
        {"bidijkstra", {}},
        {"bidi_astar", {{"variant", "conservative"}}},
        {"bidi_astar", {{"variant", "consistent"}}},
        {"alt", {{"landmarks", "4"}, {"strategy", "random"}, {"active", "2"}, {"seed", "1"}}},
        {"alt", {{"landmarks", "4"}, {"strategy", "farthest"}, {"active", "2"}, {"seed", "1"}}},
        {"alt_bidi", {{"landmarks", "4"}, {"strategy", "farthest"}, {"active", "2"}, {"seed", "1"}}},
        {"ch", {}},
        // CH with E-only ordering (baseline) and aggressive hop limit
        {"ch",
         {{"w_edge_diff", "1"},
          {"w_deleted_neighbors", "0"},
          {"w_depth", "0"},
          {"w_original_edges", "0"},
          {"hop_stages", "5@0.0"}}},
        // Arc-flags with 4 regions, grid partition (small graphs only)
        {"arcflags", {{"regions", "4"}, {"partition", "grid"}, {"threads", "1"}}},
        // CHASE with large core_fraction so even tiny graphs have a meaningful core
        {"chase", {{"core_fraction", "0.5"}, {"regions", "4"}, {"partition", "grid"}, {"threads", "1"}}},
    };

    // Line graph: 0→1→2→3
    const transport::Graph line = make_graph(4, {{{1, 1}}, {{2, 1}}, {{3, 1}}, {}});
    if (!check_all_pairs(line, configs)) {
        return 1;
    }

    // Directed with shortcut witness
    const transport::Graph directed_with_witness = make_graph(5, {
                                                                     {{1, 2}, {2, 10}},
                                                                     {{2, 2}, {3, 20}},
                                                                     {{3, 2}},
                                                                     {{4, 2}},
                                                                     {{1, 1}},
                                                                 });
    if (!check_all_pairs(directed_with_witness, configs)) {
        return 1;
    }

    // Disconnected
    const transport::Graph disconnected = make_graph(5, {
                                                            {{1, 5}},
                                                            {{2, 5}},
                                                            {},
                                                            {{4, 1}},
                                                            {},
                                                        });
    if (!check_all_pairs(disconnected, configs)) {
        return 1;
    }

    // One-way 5-cycle: 0→1→2→3→4→0
    const transport::Graph cycle5 = make_graph(5, {{{1, 3}}, {{2, 3}}, {{3, 3}}, {{4, 3}}, {{0, 3}}});
    if (!check_all_pairs(cycle5, configs)) {
        return 1;
    }

    // Asymmetric: 0→1 costs 5, 1→0 costs 100
    const transport::Graph asymmetric = make_graph(3, {{{1, 5}, {2, 20}}, {{0, 100}, {2, 3}}, {}});
    if (!check_all_pairs(asymmetric, configs)) {
        return 1;
    }

    // 6x6 geo grid with real coordinates and haversine-based weights
    const transport::Graph geo_grid = make_geo_grid(6, 6);
    if (!check_all_pairs(geo_grid, configs)) {
        return 1;
    }

    if (!check_malformed_graph_files_fail_fast()) {
        return 1;
    }

    // CH save/load round-trip (small graph only)
    if (!check_ch_io_roundtrip(directed_with_witness)) {
        return 1;
    }

    // PHAST correctness: all-to-one and one-to-all must match Dijkstra for every (s,t)
    if (!check_phast_correctness(directed_with_witness)) {
        std::cerr << "PHAST correctness check failed on directed_with_witness\n";
        return 1;
    }
    if (!check_phast_correctness(geo_grid)) {
        std::cerr << "PHAST correctness check failed on geo_grid\n";
        return 1;
    }

    return 0;
}
