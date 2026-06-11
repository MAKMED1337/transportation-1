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
            // No witness within the bounds, and the sum fits in Weight, so a shortcut is needed.
            // Sums are widened to Distance for overflow-safe arithmetic but stored arcs are Weight.
            if (witness_distance > shortcut_weight && shortcut_weight <= std::numeric_limits<Weight>::max()) {
                shortcuts.push_back(Shortcut{left.to, right.to, static_cast<Weight>(shortcut_weight)});
            }
        }
    }
    return shortcuts;
}

int64_t edge_difference(const WorkGraph &graph, VertexId vertex, WitnessSearch &witness, uint32_t hop_limit) {
    const size_t added = find_shortcuts(graph, vertex, witness, hop_limit).size();
    const size_t removed = graph.uncontracted_in(vertex).size() + graph.uncontracted_out(vertex).size();
    return static_cast<int64_t>(added) - static_cast<int64_t>(removed);
}

} // namespace transport::ch
