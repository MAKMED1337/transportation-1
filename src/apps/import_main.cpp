#include "graph/graph.hpp"

#include <algorithm>
#include <chrono>
#include <cstdint>
#include <filesystem>
#include <fstream>
#include <iostream>
#include <limits>
#include <optional>
#include <stdexcept>
#include <string>
#include <string_view>
#include <unordered_map>
#include <utility>
#include <vector>

#include <osmium/handler.hpp>
#include <osmium/handler/node_locations_for_ways.hpp>
#include <osmium/index/map/sparse_mem_array.hpp>
#include <osmium/io/any_input.hpp>
#include <osmium/io/reader.hpp>
#include <osmium/osm/location.hpp>
#include <osmium/tags/filter.hpp>
#include <osmium/visitor.hpp>

namespace fs = std::filesystem;
using transport::Edge;
using transport::Graph;
using transport::NodeCoord;

namespace {

struct TempEdge {
    uint32_t from = 0;
    uint32_t to = 0;
    uint32_t w = 0;
};

template <size_t N> std::optional<std::string_view> tag_value(const osmium::TagList &tags, const char (&key)[N]) {
    const auto value = tags.get_value_by_key(key);
    if (value == nullptr) {
        return std::nullopt;
    }
    return std::string_view{value};
}

bool is_driving_highway(std::string_view value) {
    static constexpr std::string_view allowed[] = {"motorway",      "trunk",        "primary",        "secondary",
                                                   "tertiary",      "unclassified", "residential",    "motorway_link",
                                                   "trunk_link",    "primary_link", "secondary_link", "tertiary_link",
                                                   "living_street", "service"};
    return std::find(std::begin(allowed), std::end(allowed), value) != std::end(allowed);
}

bool is_driving_highway(std::optional<std::string_view> value) {
    if (!value.has_value()) {
        return false;
    }
    return is_driving_highway(*value);
}

bool is_blocked_for_car(const osmium::TagList &tags) {
    const std::optional<std::string_view> access = tag_value(tags, "access");
    if (access.has_value() && (*access == "no" || *access == "private")) {
        return true;
    }
    const std::optional<std::string_view> motor_vehicle = tag_value(tags, "motor_vehicle");
    if (motor_vehicle.has_value() && (*motor_vehicle == "no" || *motor_vehicle == "private")) {
        return true;
    }
    return false;
}

bool is_oneway_forward(const osmium::TagList &tags) {
    const std::optional<std::string_view> oneway = tag_value(tags, "oneway");
    if (!oneway.has_value()) {
        const std::optional<std::string_view> junction = tag_value(tags, "junction");
        return junction.has_value() && *junction == "roundabout";
    }
    return *oneway == "yes" || *oneway == "1" || *oneway == "true";
}

bool is_oneway_reverse(const osmium::TagList &tags) {
    const std::optional<std::string_view> oneway = tag_value(tags, "oneway");
    if (!oneway.has_value()) {
        return false;
    }
    return *oneway == "-1";
}

struct WayCollector : public osmium::handler::Handler {
    std::unordered_map<osmium::object_id_type, uint32_t> id_to_index;
    std::vector<NodeCoord> coords;
    std::vector<TempEdge> edges;

    uint32_t get_or_create(const osmium::NodeRef &node) {
        const auto id = node.ref();
        const auto it = id_to_index.find(id);
        if (it != id_to_index.end()) {
            return it->second;
        }
        const uint32_t idx = static_cast<uint32_t>(coords.size());
        id_to_index.emplace(id, idx);
        coords.push_back(NodeCoord{
            .lat = node.location().lat_without_check(),
            .lon = node.location().lon_without_check(),
        });
        return idx;
    }

    void way(const osmium::Way &way) {
        const std::optional<std::string_view> highway = tag_value(way.tags(), "highway");
        if (!is_driving_highway(highway) || is_blocked_for_car(way.tags())) {
            return;
        }

        const bool forward = !is_oneway_reverse(way.tags());
        const bool backward = !is_oneway_forward(way.tags());
        if (!forward && !backward) {
            return;
        }

        const auto &nodes = way.nodes();
        if (nodes.size() < 2) {
            return;
        }

        for (size_t i = 1; i < nodes.size(); ++i) {
            const osmium::NodeRef a = nodes[i - 1];
            const osmium::NodeRef b = nodes[i];
            if (!a.location() || !b.location()) {
                continue;
            }

            const uint32_t ia = get_or_create(a);
            const uint32_t ib = get_or_create(b);
            const transport::NodeCoord coord_a{
                .lat = a.location().lat_without_check(),
                .lon = a.location().lon_without_check(),
            };
            const transport::NodeCoord coord_b{
                .lat = b.location().lat_without_check(),
                .lon = b.location().lon_without_check(),
            };
            const double d = transport::haversine_meters(coord_a, coord_b);
            const double scaled = d * static_cast<double>(transport::kDistanceScale);
            if (scaled < 0.0 || scaled > static_cast<double>(std::numeric_limits<uint32_t>::max())) {
                continue;
            }
            const uint32_t w = static_cast<uint32_t>(std::ceil(scaled));

            if (forward) {
                edges.push_back({ia, ib, w});
            }
            if (backward) {
                edges.push_back({ib, ia, w});
            }
        }
    }
};

void write_stats_json(const std::string &path, const std::string &input_file, uintmax_t input_size, const Graph &graph,
                      double import_seconds) {
    std::ofstream out(path);
    if (!out) {
        throw std::runtime_error("failed to write stats file");
    }
    out << "{\n";
    out << "  \"input_file\": \"" << input_file << "\",\n";
    out << "  \"input_size_bytes\": " << input_size << ",\n";
    out << "  \"vertices\": " << graph.vertex_count() << ",\n";
    out << "  \"directed_edges\": " << graph.edge_count() << ",\n";
    out << "  \"import_seconds\": " << import_seconds << "\n";
    out << "}\n";
}

} // namespace

int main(int argc, char **argv) {
    std::string input;
    std::string output;
    std::string stats;
    for (int i = 1; i + 1 < argc; ++i) {
        const std::string key = argv[i];
        const std::string value = argv[i + 1];
        if (key == "--input") {
            input = value;
            ++i;
        } else if (key == "--output") {
            output = value;
            ++i;
        } else if (key == "--stats") {
            stats = value;
            ++i;
        }
    }

    if (input.empty() || output.empty() || stats.empty()) {
        std::cerr << "usage: transport_import_osm --input <europe.osm.pbf> --output <graph.bin> --stats <stats.json>\n";
        return 1;
    }

    const auto t0 = std::chrono::steady_clock::now();
    WayCollector collector;
    osmium::io::File infile(input);
    osmium::io::Reader reader(infile);
    osmium::index::map::SparseMemArray<osmium::unsigned_object_id_type, osmium::Location> node_locations;
    osmium::handler::NodeLocationsForWays<
        osmium::index::map::SparseMemArray<osmium::unsigned_object_id_type, osmium::Location>>
        location_handler(node_locations);
    osmium::apply(reader, location_handler, collector);
    reader.close();

    Graph graph;
    graph.coords = std::move(collector.coords);
    graph.offsets.assign(static_cast<size_t>(graph.vertex_count()) + 1, 0);

    for (const TempEdge &e : collector.edges) {
        graph.offsets[e.from + 1] += 1;
    }
    for (size_t i = 1; i < graph.offsets.size(); ++i) {
        graph.offsets[i] += graph.offsets[i - 1];
    }

    graph.edges.resize(collector.edges.size());
    std::vector<uint64_t> cursor = graph.offsets;
    for (const TempEdge &e : collector.edges) {
        const uint64_t pos = cursor[e.from]++;
        graph.edges[static_cast<size_t>(pos)] = Edge{e.to, e.w};
    }

    fs::create_directories(fs::path(output).parent_path());
    if (!transport::save_graph_binary(graph, output)) {
        std::cerr << "failed to save graph\n";
        return 1;
    }

    const uintmax_t input_size = fs::file_size(input);
    const auto t1 = std::chrono::steady_clock::now();
    const double seconds = std::chrono::duration<double>(t1 - t0).count();
    write_stats_json(stats, input, input_size, graph, seconds);

    std::cout << "map_size_bytes=" << input_size << "\n";
    std::cout << "vertices=" << graph.vertex_count() << "\n";
    std::cout << "directed_edges=" << graph.edge_count() << "\n";
}
