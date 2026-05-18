#pragma once

#include <cstdint>
#include <string>
#include <vector>

namespace transport {

constexpr uint32_t kDistanceScale = 1000;

struct NodeCoord {
    double lat = 0.0;
    double lon = 0.0;
};

struct Edge {
    uint32_t to = 0;
    uint32_t weight_units = 0;
};

class Graph {
public:
    std::vector<NodeCoord> coords;
    std::vector<uint64_t> offsets;
    std::vector<Edge> edges;

    [[nodiscard]] uint32_t vertex_count() const;
    [[nodiscard]] uint64_t edge_count() const;
};

bool save_graph_binary(const Graph &graph, const std::string &path);
Graph load_graph_binary(const std::string &path);

double haversine_meters(const NodeCoord &a, const NodeCoord &b);

} // namespace transport
