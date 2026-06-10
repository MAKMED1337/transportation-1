#include "algorithms/ch/work_graph.hpp"

#include <cstddef>

namespace transport::ch {
namespace {

size_t offset_index(uint64_t offset) { return static_cast<size_t>(offset); }

} // namespace

void WorkGraph::add_or_relax(VertexId from, VertexId to, Weight weight) {
    for (WorkArc &arc : out[from]) {
        if (arc.to != to) {
            continue;
        }
        if (weight >= arc.weight) {
            return;
        }

        arc.weight = weight;
        for (WorkArc &reverse : in[to]) {
            if (reverse.to == from) {
                reverse.weight = weight;
                return;
            }
        }
        return;
    }

    out[from].push_back(WorkArc{to, weight});
    in[to].push_back(WorkArc{from, weight});
}

std::vector<WorkArc> WorkGraph::uncontracted_in(VertexId v) const {
    std::vector<WorkArc> result;
    result.reserve(in[v].size());
    for (const WorkArc &arc : in[v]) {
        if (!contracted[arc.to]) {
            result.push_back(arc);
        }
    }
    return result;
}

std::vector<WorkArc> WorkGraph::uncontracted_out(VertexId v) const {
    std::vector<WorkArc> result;
    result.reserve(out[v].size());
    for (const WorkArc &arc : out[v]) {
        if (!contracted[arc.to]) {
            result.push_back(arc);
        }
    }
    return result;
}

WorkGraph::WorkGraph(const Graph &graph) : WorkGraph(graph.vertex_count()) {
    const VertexId vertices = graph.vertex_count();
    for (VertexId from = 0; from < vertices; ++from) {
        const size_t begin = offset_index(graph.offsets[from]);
        const size_t end = offset_index(graph.offsets[from + 1]);
        out[from].reserve(end - begin);

        for (size_t edge_index = begin; edge_index < end; ++edge_index) {
            const Edge &edge = graph.edges[edge_index];
            add_or_relax(from, edge.to, edge.weight);
        }
    }
}

} // namespace transport::ch
