#include "algorithms/ch/contraction_hierarchy.hpp"

#include "algorithms/ch/node_order.hpp"
#include "algorithms/ch/witness_search.hpp"
#include "algorithms/ch/work_graph.hpp"
#include "algorithms/heap_node.hpp"
#include "routing/routing.hpp"

#include <algorithm>
#include <cstddef>
#include <cstdint>
#include <functional>
#include <queue>
#include <stdexcept>
#include <string_view>
#include <vector>

namespace transport {
namespace {

using namespace ch;

size_t offset_index(uint64_t offset) { return static_cast<size_t>(offset); }

// Contracts every vertex in lazy edge-difference order, inserting shortcuts into `work`, and returns
// the contraction rank of each vertex (the order in which it was contracted).
std::vector<uint32_t> contract_graph(WorkGraph &work, WitnessSearch &witness) {
    // Lexicographic: priority first, then v for determinism. std::greater<> makes a min-heap.
    struct OrderEntry {
        int64_t priority;
        VertexId v;
        auto operator<=>(const OrderEntry &) const = default;
    };

    const VertexId vertices = work.vertex_count();
    std::vector<uint32_t> rank(vertices, 0);

    std::priority_queue<OrderEntry, std::vector<OrderEntry>, std::greater<>> pq;
    for (VertexId v = 0; v < vertices; ++v) {
        pq.push(OrderEntry{edge_difference(work, v, witness, kWitnessHopLimit), v});
    }

    uint32_t next_rank = 0;
    while (!pq.empty()) {
        const OrderEntry top = pq.top();
        pq.pop();
        if (work.contracted[top.v] != 0) {
            continue; // stale duplicate left behind by an earlier reinsert
        }

        // Lazy update: the priority may be out of date because neighbours were contracted after it was
        // queued. Recompute it; if it is no longer the minimum, reinsert instead of contracting now.
        const OrderEntry refreshed{edge_difference(work, top.v, witness, kWitnessHopLimit), top.v};
        if (!pq.empty() && refreshed > pq.top()) {
            pq.push(refreshed);
            continue;
        }

        for (const Shortcut &shortcut : find_shortcuts(work, top.v, witness, kWitnessHopLimit)) {
            work.add_or_relax(shortcut.from, shortcut.to, shortcut.weight);
        }
        work.contracted[top.v] = 1;
        rank[top.v] = next_rank++;
    }

    return rank;
}

// --- assembling the upward search graph from the contracted work graph ---

void append_or_relax_edge(std::vector<std::vector<Edge>> &arcs, VertexId from, VertexId to, Weight weight) {
    for (Edge &edge : arcs[from]) {
        if (edge.to == to) {
            edge.weight = std::min(edge.weight, weight);
            return;
        }
    }
    arcs[from].push_back(Edge{to, weight});
}

void flatten_adjacency(const std::vector<std::vector<Edge>> &arcs, std::vector<uint64_t> &offsets,
                       std::vector<Edge> &edges) {
    offsets.assign(arcs.size() + 1, 0);
    for (size_t i = 0; i < arcs.size(); ++i) {
        offsets[i + 1] = offsets[i] + static_cast<uint64_t>(arcs[i].size());
    }

    edges.clear();
    edges.reserve(offset_index(offsets.back()));
    for (const std::vector<Edge> &row : arcs) {
        edges.insert(edges.end(), row.begin(), row.end());
    }
}

void build_upward_graph(const WorkGraph &work, const std::vector<uint32_t> &rank, ContractionHierarchy &ch) {
    std::vector<std::vector<Edge>> forward(rank.size());
    std::vector<std::vector<Edge>> backward(rank.size());

    for (VertexId from = 0; from < work.vertex_count(); ++from) {
        for (const WorkArc &arc : work.out[from]) {
            if (rank[from] < rank[arc.to]) {
                append_or_relax_edge(forward, from, arc.to, arc.weight);
            } else if (rank[arc.to] < rank[from]) {
                append_or_relax_edge(backward, arc.to, from, arc.weight);
            }
        }
    }

    flatten_adjacency(forward, ch.forward_offsets, ch.forward_edges);
    flatten_adjacency(backward, ch.backward_offsets, ch.backward_edges);
}

ContractionHierarchy build_contraction_hierarchy(const Graph &graph) {
    WorkGraph work(graph);
    WitnessSearch witness(graph.vertex_count());

    ContractionHierarchy ch;
    ch.rank = contract_graph(work, witness);
    build_upward_graph(work, ch.rank, ch);
    return ch;
}

// --- bidirectional upward query ---

// Pops and relaxes one node from `pq` in its direction. `dist` is this direction's stamped distances,
// `opposite_dist` the other direction's; when both meet at a node, `best` is tightened. Returns false
// when the popped entry was a stale duplicate (no node was settled).
bool settle_next(std::priority_queue<HeapNode, std::vector<HeapNode>, std::greater<>> &pq,
                 const std::vector<uint64_t> &offsets, const std::vector<Edge> &edges, StampedVector<Distance> &dist,
                 const StampedVector<Distance> &opposite_dist, Distance &best) {
    const HeapNode top = pq.top();
    pq.pop();
    if (top.key != dist.get(top.v)) {
        return false;
    }

    const Distance opposite = opposite_dist.get(top.v);
    if (opposite < kUnreachable) {
        best = std::min(best, top.key + opposite);
    }

    const size_t begin = offset_index(offsets[top.v]);
    const size_t end = offset_index(offsets[top.v + 1]);
    for (size_t edge_index = begin; edge_index < end; ++edge_index) {
        const Edge &edge = edges[edge_index];
        const Distance next = top.key + edge.weight;
        if (next < dist.get(edge.to) && next < best) {
            dist.set(edge.to, next);
            pq.push(HeapNode{next, edge.to});
        }
    }

    return true;
}

} // namespace

VertexId ContractionHierarchy::vertex_count() const { return static_cast<VertexId>(rank.size()); }

ContractionHierarchyAlgorithm::ContractionHierarchyAlgorithm(const Graph &graph)
    : graph_(graph), forward_dist_(graph.vertex_count(), kUnreachable),
      backward_dist_(graph.vertex_count(), kUnreachable) {}

std::string_view ContractionHierarchyAlgorithm::name() const { return "ch"; }

void ContractionHierarchyAlgorithm::preprocess() {
    if (preprocessed_) {
        return;
    }
    ch_ = build_contraction_hierarchy(graph_);
    preprocessed_ = true;
}

PathResult ContractionHierarchyAlgorithm::query(VertexId source, VertexId target) const {
    if (!preprocessed_) {
        throw std::logic_error("ContractionHierarchyAlgorithm::preprocess() must be called before query()");
    }

    forward_dist_.reset();
    backward_dist_.reset();
    std::priority_queue<HeapNode, std::vector<HeapNode>, std::greater<>> forward_pq;
    std::priority_queue<HeapNode, std::vector<HeapNode>, std::greater<>> backward_pq;

    forward_dist_.set(source, 0);
    backward_dist_.set(target, 0);
    forward_pq.push(HeapNode{0, source});
    backward_pq.push(HeapNode{0, target});

    Distance best = kUnreachable;
    uint32_t settled = 0;
    while (!forward_pq.empty() || !backward_pq.empty()) {
        const Distance forward_min = forward_pq.empty() ? kUnreachable : forward_pq.top().key;
        const Distance backward_min = backward_pq.empty() ? kUnreachable : backward_pq.top().key;
        if (forward_min >= best && backward_min >= best) {
            break;
        }

        if (forward_min <= backward_min) {
            if (settle_next(forward_pq, ch_.forward_offsets, ch_.forward_edges, forward_dist_, backward_dist_, best)) {
                ++settled;
            }
        } else if (settle_next(backward_pq, ch_.backward_offsets, ch_.backward_edges, backward_dist_, forward_dist_,
                               best)) {
            ++settled;
        }
    }

    return PathResult{best, settled};
}

uint64_t ContractionHierarchyAlgorithm::auxiliary_edge_count() const {
    return static_cast<uint64_t>(ch_.forward_edges.size() + ch_.backward_edges.size());
}

} // namespace transport
