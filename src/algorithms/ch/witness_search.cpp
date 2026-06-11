#include "algorithms/ch/witness_search.hpp"

#include "algorithms/heap_node.hpp"

#include <functional>
#include <queue>

namespace transport::ch {

WitnessSearch::WitnessSearch(size_t vertices) : table_(vertices, Cell{kInf, 0}) {}

Distance WitnessSearch::run(const WorkGraph &graph, VertexId source, VertexId target, VertexId forbidden,
                            Distance max_distance, uint32_t hop_limit) {
    ++calls_;
    table_.reset();

    std::priority_queue<HeapNode, std::vector<HeapNode>, std::greater<>> pq;
    table_.set(source, Cell{0, 0});
    pq.push(HeapNode{0, source});

    while (!pq.empty()) {
        const HeapNode top = pq.top();
        pq.pop();
        const Cell current = table_.get(top.v);
        if (top.key != current.dist) {
            continue;
        }
        if (top.key > max_distance) {
            break;
        }
        if (top.v == target) {
            return top.key;
        }
        if (current.hops >= hop_limit) {
            continue; // hop limit reached: do not expand this node further
        }

        for (const WorkArc &arc : graph.out[top.v]) {
            if (arc.to == forbidden || graph.contracted[arc.to] != 0) {
                continue;
            }
            const Distance next = top.key + arc.weight;
            if (next < table_.get(arc.to).dist && next <= max_distance) {
                table_.set(arc.to, Cell{next, current.hops + 1});
                pq.push(HeapNode{next, arc.to});
            }
        }
    }

    return kInf;
}

} // namespace transport::ch
