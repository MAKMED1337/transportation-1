#include "algorithms/ch/contraction_hierarchy.hpp"

#include "algorithms/ch/node_order.hpp"
#include "algorithms/ch/witness_search.hpp"
#include "algorithms/ch/work_graph.hpp"
#include "algorithms/heap_node.hpp"
#include "routing/routing.hpp"

#include <algorithm>
#include <chrono>
#include <cstddef>
#include <cstdint>
#include <functional>
#include <iomanip>
#include <queue>
#include <sstream>
#include <stdexcept>
#include <string>
#include <string_view>
#include <utility>
#include <vector>

namespace transport {
namespace {

using namespace ch;

size_t offset_index(uint64_t offset) { return static_cast<size_t>(offset); }

// Returns the hop limit that should be used at `mean_degree` according to `stages`.
// Stages must be sorted by threshold ascending (as the default OrderParams guarantees).
uint32_t hop_limit_for_degree(const std::vector<HopStage> &stages, double mean_degree) {
    uint32_t hl = stages.empty() ? kWitnessHopLimit : stages[0].hop_limit;
    for (const HopStage &s : stages) {
        if (mean_degree >= s.mean_degree_threshold) {
            hl = s.hop_limit;
        } else {
            break;
        }
    }
    return hl;
}

// Build a human-readable variant string for the given OrderParams.
std::string format_variant(const OrderParams &p) {
    std::ostringstream oss;
    oss << "E" << p.w_edge_diff << " D" << p.w_deleted_neighbors << " Q" << p.w_depth << " O" << p.w_original_edges
        << " S" << p.w_search_space << " V" << p.w_voronoi << ", hops";
    for (size_t i = 0; i < p.hop_stages.size(); ++i) {
        if (i > 0) {
            oss << "/";
        }
        oss << p.hop_stages[i].hop_limit;
        if (p.hop_stages[i].mean_degree_threshold > 0.0) {
            std::ostringstream tss;
            tss << std::setprecision(4) << p.hop_stages[i].mean_degree_threshold;
            oss << "@" << tss.str();
        }
    }
    return oss.str();
}

std::pair<std::vector<uint32_t>, PreprocessStats> contract_graph(WorkGraph &work, WitnessSearch &witness,
                                                                 const OrderParams &params) {
    struct OrderEntry {
        int64_t priority;
        VertexId v;
        auto operator<=>(const OrderEntry &) const = default;
    };

    const VertexId vertices = work.vertex_count();
    std::vector<uint32_t> rank(vertices, 0);
    std::vector<uint32_t> depth(vertices, 0);
    std::vector<uint32_t> deleted_neighbors(vertices, 0);

    // Track total arcs for mean-degree estimation (initial + shortcuts added).
    uint64_t arc_count = 0;
    for (VertexId v = 0; v < vertices; ++v) {
        arc_count += work.out[v].size();
    }
    uint64_t shortcuts_total = 0;
    uint32_t contracted_count = 0;

    // Initial hop limit from stages.
    const double init_mean = vertices > 0 ? static_cast<double>(arc_count) / static_cast<double>(vertices) : 0.0;
    uint32_t current_hop = hop_limit_for_degree(params.hop_stages, init_mean);
    size_t next_stage_idx = 0;
    while (next_stage_idx < params.hop_stages.size() &&
           init_mean >= params.hop_stages[next_stage_idx].mean_degree_threshold) {
        ++next_stage_idx;
    }

    std::priority_queue<OrderEntry, std::vector<OrderEntry>, std::greater<>> pq;

    auto rebuild_pq = [&]() {
        while (!pq.empty()) {
            pq.pop();
        }
        for (VertexId v = 0; v < vertices; ++v) {
            if (!work.contracted[v]) {
                pq.push(
                    OrderEntry{compute_priority(work, v, witness, current_hop, params, depth, deleted_neighbors), v});
            }
        }
    };

    const auto ordering_init_start = std::chrono::steady_clock::now();
    rebuild_pq();
    const std::chrono::nanoseconds ordering_init_ns = std::chrono::steady_clock::now() - ordering_init_start;

    PreprocessStats stats;
    stats.ordering_init_ns = ordering_init_ns;

    uint32_t next_rank = 0;
    while (!pq.empty()) {
        // Check whether we should advance to the next hop stage.
        const uint32_t remaining = vertices - contracted_count;
        if (remaining > 0 && next_stage_idx < params.hop_stages.size()) {
            const double mean_degree =
                static_cast<double>(arc_count + shortcuts_total) / static_cast<double>(remaining);
            if (mean_degree >= params.hop_stages[next_stage_idx].mean_degree_threshold) {
                current_hop = params.hop_stages[next_stage_idx].hop_limit;
                stats.hop_stage_switches.emplace_back(contracted_count, current_hop);
                ++next_stage_idx;
                rebuild_pq();
                continue;
            }
        }

        const OrderEntry top = pq.top();
        pq.pop();
        if (work.contracted[top.v] != 0) {
            continue;
        }

        // Lazy update: recompute priority; if no longer the minimum, reinsert.
        const OrderEntry refreshed{
            compute_priority(work, top.v, witness, current_hop, params, depth, deleted_neighbors), top.v};
        if (!pq.empty() && refreshed > pq.top()) {
            pq.push(refreshed);
            continue;
        }

        const std::vector<Shortcut> shortcuts = find_shortcuts(work, top.v, witness, current_hop);
        for (const Shortcut &sc : shortcuts) {
            work.add_or_relax(sc.from, sc.to, sc.weight, sc.originals);
        }
        shortcuts_total += static_cast<uint64_t>(shortcuts.size());

        // Collect unique uncontracted neighbors (both predecessors and successors).
        std::vector<VertexId> nbrs;
        for (const WorkArc &a : work.uncontracted_in(top.v)) {
            nbrs.push_back(a.to);
        }
        for (const WorkArc &a : work.uncontracted_out(top.v)) {
            nbrs.push_back(a.to);
        }
        std::sort(nbrs.begin(), nbrs.end());
        nbrs.erase(std::unique(nbrs.begin(), nbrs.end()), nbrs.end());
        for (const VertexId nb : nbrs) {
            ++deleted_neighbors[nb];
            depth[nb] = std::max(depth[nb], depth[top.v] + 1);
        }

        work.contracted[top.v] = 1;
        rank[top.v] = next_rank++;
        ++contracted_count;
    }

    stats.witness_calls = witness.calls();
    stats.shortcuts_added = shortcuts_total;
    return {rank, stats};
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

std::pair<ContractionHierarchy, PreprocessStats> build_contraction_hierarchy(const Graph &graph,
                                                                             const OrderParams &params) {
    WorkGraph work(graph);
    WitnessSearch witness(graph.vertex_count());

    ContractionHierarchy ch;
    auto [rank, stats] = contract_graph(work, witness, params);
    ch.rank = std::move(rank);
    build_upward_graph(work, ch.rank, ch);
    return {std::move(ch), stats};
}

// --- bidirectional upward query ---

bool settle_next(std::priority_queue<HeapNode, std::vector<HeapNode>, std::greater<>> &pq,
                 const std::vector<uint64_t> &offsets, const std::vector<Edge> &edges, StampedVector<Distance> &dist,
                 const StampedVector<Distance> &opposite_dist, Distance &best, QueryStats &stats, bool is_forward) {
    const HeapNode top = pq.top();
    pq.pop();
    if (top.key != dist.get(top.v)) {
        return false;
    }

    const Distance opposite = opposite_dist.get(top.v);
    if (opposite < kUnreachable) {
        best = std::min(best, top.key + opposite);
    }

    ++stats.settled;
    if (is_forward) {
        ++stats.settled_forward;
    } else {
        ++stats.settled_backward;
    }

    const size_t begin = offset_index(offsets[top.v]);
    const size_t end = offset_index(offsets[top.v + 1]);
    for (size_t edge_index = begin; edge_index < end; ++edge_index) {
        const Edge &edge = edges[edge_index];
        ++stats.relaxed_arcs;
        const Distance next = top.key + edge.weight;
        if (next < dist.get(edge.to) && next < best) {
            dist.set(edge.to, next);
            pq.push(HeapNode{next, edge.to});
            ++stats.heap_pushes;
        }
    }

    return true;
}

} // namespace

VertexId ContractionHierarchy::vertex_count() const { return static_cast<VertexId>(rank.size()); }

ContractionHierarchyAlgorithm::ContractionHierarchyAlgorithm(const Graph &graph, ch::OrderParams params)
    : graph_(graph), order_params_(std::move(params)), forward_dist_(graph.vertex_count(), kUnreachable),
      backward_dist_(graph.vertex_count(), kUnreachable) {}

std::string_view ContractionHierarchyAlgorithm::name() const { return "ch"; }

std::string ContractionHierarchyAlgorithm::variant() const { return format_variant(order_params_); }

void ContractionHierarchyAlgorithm::inject_ch(ContractionHierarchy ch) {
    ch_ = std::move(ch);
    preprocessed_ = true;
}

void ContractionHierarchyAlgorithm::preprocess() {
    if (preprocessed_) {
        return;
    }
    auto [ch, stats] = build_contraction_hierarchy(graph_, order_params_);
    ch_ = std::move(ch);
    last_stats_ = stats;
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
    QueryStats stats;
    while (!forward_pq.empty() || !backward_pq.empty()) {
        const Distance forward_min = forward_pq.empty() ? kUnreachable : forward_pq.top().key;
        const Distance backward_min = backward_pq.empty() ? kUnreachable : backward_pq.top().key;
        if (forward_min >= best && backward_min >= best) {
            break;
        }

        if (forward_min <= backward_min) {
            settle_next(forward_pq, ch_.forward_offsets, ch_.forward_edges, forward_dist_, backward_dist_, best, stats,
                        true);
        } else {
            settle_next(backward_pq, ch_.backward_offsets, ch_.backward_edges, backward_dist_, forward_dist_, best,
                        stats, false);
        }
    }

    return PathResult{best, stats};
}

uint64_t ContractionHierarchyAlgorithm::auxiliary_edge_count() const {
    return static_cast<uint64_t>(ch_.forward_edges.size() + ch_.backward_edges.size());
}

} // namespace transport
