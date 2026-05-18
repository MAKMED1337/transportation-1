#include "graph/graph.hpp"

#include <cmath>
#include <cstdint>
#include <iostream>
#include <string>
#include <vector>

namespace {

void add_edge(std::vector<std::vector<transport::Edge>> &rows, const std::vector<transport::NodeCoord> &coords,
              uint32_t from, uint32_t to) {
    const transport::NodeCoord &a = coords[from];
    const transport::NodeCoord &b = coords[to];
    const uint32_t weight = static_cast<uint32_t>(
        std::ceil(transport::haversine_meters(a, b) * static_cast<double>(transport::kDistanceScale)));
    rows[from].push_back(transport::Edge{to, weight});
}

transport::Graph make_grid_graph(uint32_t width, uint32_t height) {
    const uint32_t vertices = width * height;
    std::vector<transport::NodeCoord> coords(vertices);
    auto id = [width](uint32_t x, uint32_t y) { return y * width + x; };
    for (uint32_t y = 0; y < height; ++y) {
        for (uint32_t x = 0; x < width; ++x) {
            coords[id(x, y)] =
                transport::NodeCoord{52.0 + static_cast<double>(y) * 0.01, 21.0 + static_cast<double>(x) * 0.01};
        }
    }

    std::vector<std::vector<transport::Edge>> rows(vertices);
    for (uint32_t y = 0; y < height; ++y) {
        for (uint32_t x = 0; x < width; ++x) {
            const uint32_t v = id(x, y);
            if (x + 1 < width) {
                add_edge(rows, coords, v, id(x + 1, y));
                add_edge(rows, coords, id(x + 1, y), v);
            }
            if (y + 1 < height) {
                add_edge(rows, coords, v, id(x, y + 1));
                add_edge(rows, coords, id(x, y + 1), v);
            }
            if (x + 1 < width && y + 1 < height) {
                add_edge(rows, coords, v, id(x + 1, y + 1));
            }
        }
    }

    transport::Graph graph;
    graph.coords = std::move(coords);
    graph.offsets.assign(static_cast<size_t>(vertices) + 1, 0);
    for (uint32_t v = 0; v < vertices; ++v) {
        graph.offsets[v + 1] = graph.offsets[v] + static_cast<uint64_t>(rows[v].size());
    }
    for (const std::vector<transport::Edge> &row : rows) {
        graph.edges.insert(graph.edges.end(), row.begin(), row.end());
    }
    return graph;
}

} // namespace

int main(int argc, char **argv) {
    if (argc != 2) {
        std::cerr << "usage: write_benchmark_graph <output.graph>\n";
        return 1;
    }

    const transport::Graph graph = make_grid_graph(30, 30);
    if (!transport::save_graph_binary(graph, argv[1])) {
        std::cerr << "failed to save graph\n";
        return 1;
    }

    std::cout << "vertices=" << graph.vertex_count() << "\n";
    std::cout << "directed_edges=" << graph.edge_count() << "\n";
    return 0;
}
