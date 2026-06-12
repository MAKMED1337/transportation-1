#include "algorithms/ch/node_order.hpp"

#include <limits>
#include <vector>

namespace transport::ch {

std::vector<Shortcut> find_shortcuts(const WorkGraph &graph, VertexId vertex, WitnessSearch &witness,
                                     uint32_t hop_limit) {
    const std::vector<WorkArc> incoming = graph.uncontracted_in(vertex);
    const std::vector<WorkArc> outgoing = graph.uncontracted_out(vertex);

    std::vector<Shortcut> shortcuts;
    for (const WorkArc &left : incoming) {
        for (const WorkArc &right : outgoing) {
            if (left.to == right.to) {
                continue;
            }

            const Distance shortcut_weight = static_cast<Distance>(left.weight) + right.weight;
            const Distance witness_distance = witness.run(graph, left.to, right.to, vertex, shortcut_weight, hop_limit);
            if (witness_distance > shortcut_weight && shortcut_weight <= std::numeric_limits<Weight>::max()) {
                shortcuts.push_back(Shortcut{left.to, right.to, static_cast<Weight>(shortcut_weight),
                                             left.originals + right.originals});
            }
        }
    }
    return shortcuts;
}

int64_t compute_priority(const WorkGraph &graph, VertexId vertex, WitnessSearch &witness, uint32_t hop_limit,
                         const OrderParams &params, const std::vector<uint32_t> &depth,
                         const std::vector<uint32_t> &deleted_neighbors) {
    witness.reset_sim_settled();
    const std::vector<Shortcut> sc = find_shortcuts(graph, vertex, witness, hop_limit);
    const std::vector<WorkArc> in_arcs = graph.uncontracted_in(vertex);
    const std::vector<WorkArc> out_arcs = graph.uncontracted_out(vertex);

    const int64_t E = static_cast<int64_t>(sc.size()) - static_cast<int64_t>(in_arcs.size() + out_arcs.size());
    const int64_t D = static_cast<int64_t>(deleted_neighbors[vertex]);
    const int64_t Q = static_cast<int64_t>(depth[vertex]);

    int64_t orig_added = 0;
    for (const Shortcut &s : sc) {
        orig_added += static_cast<int64_t>(s.originals);
    }
    int64_t orig_removed = 0;
    for (const WorkArc &a : in_arcs) {
        orig_removed += static_cast<int64_t>(a.originals);
    }
    for (const WorkArc &a : out_arcs) {
        orig_removed += static_cast<int64_t>(a.originals);
    }
    const int64_t O = orig_added - orig_removed;
    const int64_t S = static_cast<int64_t>(witness.sim_settled());

    return params.w_edge_diff * E + params.w_deleted_neighbors * D + params.w_depth * Q + params.w_original_edges * O +
           params.w_search_space * S;
}

} // namespace transport::ch
