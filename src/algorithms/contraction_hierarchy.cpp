#include "algorithms/contraction_hierarchy.hpp"

#include "algorithms/heap_node.hpp"

#include <algorithm>
#include <cstdint>
#include <functional>
#include <limits>
#include <numeric>
#include <queue>
#include <stdexcept>
#include <vector>

namespace transport {
namespace {

struct WorkArc {
    uint32_t to = 0;
    uint32_t weight_units = 0;
};

struct WorkGraph {
    explicit WorkGraph(size_t vertices) : out(vertices), in(vertices) {}

    std::vector<std::vector<WorkArc>> out;
    std::vector<std::vector<WorkArc>> in;
};

constexpr uint64_t kInf = std::numeric_limits<uint64_t>::max();

[[nodiscard]] size_t offset_index(uint64_t offset) { return static_cast<size_t>(offset); }

void add_or_relax_arc(WorkGraph &graph, uint32_t from, uint32_t to, uint32_t weight_units) {
    for (WorkArc &arc : graph.out[from]) {
        if (arc.to != to) {
            continue;
        }
        if (weight_units >= arc.weight_units) {
            return;
        }

        arc.weight_units = weight_units;
        for (WorkArc &reverse : graph.in[to]) {
            if (reverse.to == from) {
                reverse.weight_units = weight_units;
                return;
            }
        }
        return;
    }

    graph.out[from].push_back(WorkArc{to, weight_units});
    graph.in[to].push_back(WorkArc{from, weight_units});
}

class WitnessSearch {
public:
    explicit WitnessSearch(size_t vertices) : dist_(vertices, kInf), seen_(vertices, 0) {}

    uint64_t run(const WorkGraph &graph, const std::vector<uint8_t> &contracted, uint32_t source, uint32_t target,
                 uint32_t forbidden, uint64_t max_distance) {
        reset_search();

        std::priority_queue<HeapNode, std::vector<HeapNode>, std::greater<>> pq;
        set_dist(source, 0);
        pq.push(HeapNode{0, source});

        while (!pq.empty()) {
            const HeapNode top = pq.top();
            pq.pop();
            if (top.key != get_dist(top.v)) {
                continue;
            }
            if (top.key > max_distance) {
                break;
            }
            if (top.v == target) {
                return top.key;
            }

            for (const WorkArc &arc : graph.out[top.v]) {
                if (arc.to == forbidden || contracted[arc.to] != 0) {
                    continue;
                }
                const uint64_t next = top.key + arc.weight_units;
                if (next < get_dist(arc.to) && next <= max_distance) {
                    set_dist(arc.to, next);
                    pq.push(HeapNode{next, arc.to});
                }
            }
        }

        return kInf;
    }

private:
    void reset_search() {
        ++stamp_;
        if (stamp_ != 0) {
            return;
        }

        std::fill(seen_.begin(), seen_.end(), 0);
        stamp_ = 1;
    }

    [[nodiscard]] uint64_t get_dist(uint32_t v) const { return seen_[v] == stamp_ ? dist_[v] : kInf; }

    void set_dist(uint32_t v, uint64_t value) {
        seen_[v] = stamp_;
        dist_[v] = value;
    }

    std::vector<uint64_t> dist_;
    std::vector<uint32_t> seen_;
    uint32_t stamp_ = 0;
};

[[nodiscard]] WorkGraph build_work_graph(const Graph &graph) {
    const uint32_t vertices = graph.vertex_count();
    WorkGraph work(vertices);

    for (uint32_t from = 0; from < vertices; ++from) {
        const size_t begin = offset_index(graph.offsets[from]);
        const size_t end = offset_index(graph.offsets[from + 1]);
        work.out[from].reserve(end - begin);

        for (size_t edge_index = begin; edge_index < end; ++edge_index) {
            const Edge &edge = graph.edges[edge_index];
            add_or_relax_arc(work, from, edge.to, edge.weight_units);
        }
    }

    return work;
}

[[nodiscard]] std::vector<uint32_t> contraction_order(const WorkGraph &graph) {
    std::vector<uint32_t> order(graph.out.size());
    std::iota(order.begin(), order.end(), 0);

    std::sort(order.begin(), order.end(), [&graph](uint32_t a, uint32_t b) {
        const size_t degree_a = graph.out[a].size() + graph.in[a].size();
        const size_t degree_b = graph.out[b].size() + graph.in[b].size();
        if (degree_a != degree_b) {
            return degree_a < degree_b;
        }
        return a < b;
    });
    return order;
}

[[nodiscard]] std::vector<uint32_t> compute_rank(const std::vector<uint32_t> &order) {
    std::vector<uint32_t> rank(order.size(), 0);
    for (uint32_t i = 0; i < order.size(); ++i) {
        rank[order[i]] = i;
    }
    return rank;
}

[[nodiscard]] std::vector<WorkArc> uncontracted_arcs(const std::vector<WorkArc> &arcs,
                                                     const std::vector<uint8_t> &contracted) {
    std::vector<WorkArc> result;
    result.reserve(arcs.size());
    for (const WorkArc &arc : arcs) {
        if (contracted[arc.to] == 0) {
            result.push_back(arc);
        }
    }
    return result;
}

void add_shortcuts_through_vertex(WorkGraph &graph, const std::vector<uint8_t> &contracted, uint32_t vertex,
                                  WitnessSearch &witness_search) {
    const std::vector<WorkArc> incoming = uncontracted_arcs(graph.in[vertex], contracted);
    const std::vector<WorkArc> outgoing = uncontracted_arcs(graph.out[vertex], contracted);

    for (const WorkArc &left : incoming) {
        for (const WorkArc &right : outgoing) {
            if (left.to == right.to) {
                continue;
            }

            const uint64_t shortcut_weight = static_cast<uint64_t>(left.weight_units) + right.weight_units;
            const uint64_t witness = witness_search.run(graph, contracted, left.to, right.to, vertex, shortcut_weight);
            // Shortcut sums are widened for overflow-safe arithmetic, but stored arc weights are uint32_t.
            if (witness > shortcut_weight && shortcut_weight <= std::numeric_limits<uint32_t>::max()) {
                add_or_relax_arc(graph, left.to, right.to, static_cast<uint32_t>(shortcut_weight));
            }
        }
    }
}

void append_or_relax_edge(std::vector<std::vector<Edge>> &arcs, uint32_t from, uint32_t to, uint32_t weight_units) {
    for (Edge &edge : arcs[from]) {
        if (edge.to == to) {
            edge.weight_units = std::min(edge.weight_units, weight_units);
            return;
        }
    }
    arcs[from].push_back(Edge{to, weight_units});
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

    for (uint32_t from = 0; from < rank.size(); ++from) {
        for (const WorkArc &arc : work.out[from]) {
            if (rank[from] < rank[arc.to]) {
                append_or_relax_edge(forward, from, arc.to, arc.weight_units);
            } else if (rank[arc.to] < rank[from]) {
                append_or_relax_edge(backward, arc.to, from, arc.weight_units);
            }
        }
    }

    flatten_adjacency(forward, ch.forward_offsets, ch.forward_edges);
    flatten_adjacency(backward, ch.backward_offsets, ch.backward_edges);
}

[[nodiscard]] ContractionHierarchy build_contraction_hierarchy(const Graph &graph) {
    WorkGraph work = build_work_graph(graph);
    const std::vector<uint32_t> order = contraction_order(work);

    std::vector<uint8_t> contracted(graph.vertex_count(), 0);
    WitnessSearch witness_search(graph.vertex_count());
    for (const uint32_t vertex : order) {
        add_shortcuts_through_vertex(work, contracted, vertex, witness_search);
        contracted[vertex] = 1;
    }

    ContractionHierarchy ch;
    ch.rank = compute_rank(order);
    build_upward_graph(work, ch.rank, ch);
    return ch;
}

bool settle_next(std::priority_queue<HeapNode, std::vector<HeapNode>, std::greater<>> &pq,
                 const std::vector<uint64_t> &offsets, const std::vector<Edge> &edges, std::vector<uint64_t> &dist,
                 const std::vector<uint64_t> &opposite_dist, uint64_t &best) {
    const HeapNode top = pq.top();
    pq.pop();
    if (top.key != dist[top.v]) {
        return false;
    }

    if (opposite_dist[top.v] < kInf) {
        best = std::min(best, top.key + opposite_dist[top.v]);
    }

    const size_t begin = offset_index(offsets[top.v]);
    const size_t end = offset_index(offsets[top.v + 1]);
    for (size_t edge_index = begin; edge_index < end; ++edge_index) {
        const Edge &edge = edges[edge_index];
        const uint64_t next = top.key + edge.weight_units;
        if (next < dist[edge.to] && next < best) {
            dist[edge.to] = next;
            pq.push(HeapNode{next, edge.to});
        }
    }

    return true;
}

[[nodiscard]] PathResult query_contraction_hierarchy(const ContractionHierarchy &ch, uint32_t source, uint32_t target) {
    const uint32_t vertices = ch.vertex_count();
    std::vector<uint64_t> forward_dist(vertices, kInf);
    std::vector<uint64_t> backward_dist(vertices, kInf);
    std::priority_queue<HeapNode, std::vector<HeapNode>, std::greater<>> forward_pq;
    std::priority_queue<HeapNode, std::vector<HeapNode>, std::greater<>> backward_pq;

    forward_dist[source] = 0;
    backward_dist[target] = 0;
    forward_pq.push(HeapNode{0, source});
    backward_pq.push(HeapNode{0, target});

    uint64_t best = kInf;
    uint32_t settled = 0;
    while (!forward_pq.empty() || !backward_pq.empty()) {
        const uint64_t forward_min = forward_pq.empty() ? kInf : forward_pq.top().key;
        const uint64_t backward_min = backward_pq.empty() ? kInf : backward_pq.top().key;
        if (forward_min >= best && backward_min >= best) {
            break;
        }

        if (forward_min <= backward_min) {
            if (settle_next(forward_pq, ch.forward_offsets, ch.forward_edges, forward_dist, backward_dist, best)) {
                ++settled;
            }
        } else if (settle_next(backward_pq, ch.backward_offsets, ch.backward_edges, backward_dist, forward_dist,
                               best)) {
            ++settled;
        }
    }

    return PathResult{best, settled};
}

} // namespace

uint32_t ContractionHierarchy::vertex_count() const { return static_cast<uint32_t>(rank.size()); }

ContractionHierarchyAlgorithm::ContractionHierarchyAlgorithm(const Graph &graph) : graph_(graph) {}

std::string_view ContractionHierarchyAlgorithm::name() const { return "ch"; }

void ContractionHierarchyAlgorithm::preprocess() {
    if (preprocessed_) {
        return;
    }
    ch_ = build_contraction_hierarchy(graph_);
    preprocessed_ = true;
}

PathResult ContractionHierarchyAlgorithm::query(uint32_t source, uint32_t target) const {
    if (!preprocessed_) {
        throw std::logic_error("ContractionHierarchyAlgorithm::preprocess() must be called before query()");
    }
    return query_contraction_hierarchy(ch_, source, target);
}

uint64_t ContractionHierarchyAlgorithm::auxiliary_edge_count() const {
    return static_cast<uint64_t>(ch_.forward_edges.size() + ch_.backward_edges.size());
}

} // namespace transport
