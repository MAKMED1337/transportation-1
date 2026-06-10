#pragma once

#include "graph/types.hpp"

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
    VertexId to = 0;
    Weight weight = 0;
};

class Graph {
public:
    std::vector<NodeCoord> coords;
    std::vector<uint64_t> offsets;
    std::vector<Edge> edges;

    [[nodiscard]] VertexId vertex_count() const;
    [[nodiscard]] uint64_t edge_count() const;
};

[[nodiscard]] bool save_graph_binary(const Graph &graph, const std::string &path);
[[nodiscard]] Graph load_graph_binary(const std::string &path);

double haversine_meters(const NodeCoord &a, const NodeCoord &b);

} // namespace transport
