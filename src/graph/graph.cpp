#include "graph/graph.hpp"

#include <array>
#include <bit>
#include <cmath>
#include <cstdint>
#include <fstream>
#include <limits>
#include <stdexcept>
#include <string>
#include <type_traits>

namespace transport {

namespace {

constexpr uint32_t kMagicIntWeights = 0x54524732U; // TRG2

template <typename T> void write_one(std::ofstream &out, const T &value) {
    static_assert(std::is_trivially_copyable_v<T>);
    const auto bytes = std::bit_cast<std::array<char, sizeof(T)>>(value);
    out.write(bytes.data(), static_cast<std::streamsize>(bytes.size()));
}

template <typename T> void read_one(std::ifstream &in, T &value) {
    static_assert(std::is_trivially_copyable_v<T>);
    std::array<char, sizeof(T)> bytes{};
    in.read(bytes.data(), static_cast<std::streamsize>(bytes.size()));
    value = std::bit_cast<T>(bytes);
}

void validate_graph(const Graph &graph, uint64_t expected_edges) {
    if (graph.offsets.size() != static_cast<size_t>(graph.vertex_count()) + 1) {
        throw std::runtime_error("corrupted graph file: invalid offset count");
    }
    if (graph.edge_count() != expected_edges) {
        throw std::runtime_error("corrupted graph file: invalid edge count");
    }
    if (!graph.offsets.empty() && graph.offsets.front() != 0) {
        throw std::runtime_error("corrupted graph file: first offset must be zero");
    }

    uint64_t previous = 0;
    for (const uint64_t offset : graph.offsets) {
        if (offset < previous) {
            throw std::runtime_error("corrupted graph file: offsets must be monotonic");
        }
        if (offset > expected_edges) {
            throw std::runtime_error("corrupted graph file: offset exceeds edge count");
        }
        previous = offset;
    }
    if (!graph.offsets.empty() && graph.offsets.back() != expected_edges) {
        throw std::runtime_error("corrupted graph file: final offset must match edge count");
    }

    const uint32_t vertices = graph.vertex_count();
    for (const Edge &edge : graph.edges) {
        if (edge.to >= vertices) {
            throw std::runtime_error("corrupted graph file: edge destination out of range");
        }
    }
}

} // namespace

uint32_t Graph::vertex_count() const { return static_cast<uint32_t>(coords.size()); }

uint64_t Graph::edge_count() const { return static_cast<uint64_t>(edges.size()); }

bool save_graph_binary(const Graph &graph, const std::string &path) {
    std::ofstream out(path, std::ios::binary);
    if (!out) {
        return false;
    }

    write_one(out, kMagicIntWeights);
    const uint32_t vertices = graph.vertex_count();
    const uint64_t edges = graph.edge_count();
    write_one(out, vertices);
    write_one(out, edges);

    for (const NodeCoord &node : graph.coords) {
        write_one(out, node.lat);
        write_one(out, node.lon);
    }
    out.write(reinterpret_cast<const char *>(graph.offsets.data()),
              static_cast<std::streamsize>(graph.offsets.size() * sizeof(uint64_t)));
    out.write(reinterpret_cast<const char *>(graph.edges.data()),
              static_cast<std::streamsize>(graph.edges.size() * sizeof(Edge)));
    return static_cast<bool>(out);
}

Graph load_graph_binary(const std::string &path) {
    std::ifstream in(path, std::ios::binary);
    if (!in) {
        throw std::runtime_error("failed to open graph file: " + path);
    }

    uint32_t magic = 0;
    read_one(in, magic);
    if (magic != kMagicIntWeights) {
        throw std::runtime_error("invalid graph file magic");
    }

    uint32_t vertices = 0;
    uint64_t edges = 0;
    read_one(in, vertices);
    read_one(in, edges);

    Graph graph;
    graph.coords.resize(vertices);
    for (NodeCoord &node : graph.coords) {
        read_one(in, node.lat);
        read_one(in, node.lon);
    }

    graph.offsets.resize(static_cast<size_t>(vertices) + 1);
    graph.edges.resize(static_cast<size_t>(edges));

    in.read(reinterpret_cast<char *>(graph.offsets.data()),
            static_cast<std::streamsize>(graph.offsets.size() * sizeof(uint64_t)));
    if (!in) {
        throw std::runtime_error("corrupted graph file");
    }

    in.read(reinterpret_cast<char *>(graph.edges.data()),
            static_cast<std::streamsize>(graph.edges.size() * sizeof(Edge)));
    if (!in) {
        throw std::runtime_error("corrupted graph file");
    }

    validate_graph(graph, edges);
    return graph;
}

double haversine_meters(const NodeCoord &a, const NodeCoord &b) {
    constexpr double kEarthRadiusM = 6371008.8;
    constexpr double kDegToRad = 3.14159265358979323846 / 180.0;

    const double p1 = a.lat * kDegToRad;
    const double p2 = b.lat * kDegToRad;
    const double dlat = (b.lat - a.lat) * kDegToRad;
    const double dlon = (b.lon - a.lon) * kDegToRad;

    const double haversine = std::sin(dlat / 2.0) * std::sin(dlat / 2.0) +
                             std::cos(p1) * std::cos(p2) * std::sin(dlon / 2.0) * std::sin(dlon / 2.0);
    const double c = 2.0 * std::atan2(std::sqrt(haversine), std::sqrt(1.0 - haversine));
    return kEarthRadiusM * c;
}

} // namespace transport
