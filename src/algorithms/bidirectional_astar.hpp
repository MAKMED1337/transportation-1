#pragma once

#include "algorithms/routing_algorithm.hpp"
#include "algorithms/stamped_vector.hpp"
#include "graph/graph.hpp"
#include "graph/reverse_graph.hpp"

#include <string>
#include <string_view>

namespace transport {

// Two lecture variants of bidirectional A* using haversine potentials.
//
// conservative: forward keys dist_f+pi_f, backward keys dist_b+pi_b (mutually inconsistent);
//   stop when top_f >= mu OR top_b >= mu.
//
// consistent: averaged potentials p_f=(pi_f-pi_b)/2, p_b=-p_f (feasible for both directions,
//   nonnegative reduced costs); stop when top_f + top_b >= mu.
class BidirectionalAStarAlgorithm final : public RoutingAlgorithm {
public:
    enum class Variant { Conservative, Consistent };

    explicit BidirectionalAStarAlgorithm(const Graph &graph, Variant variant);

    [[nodiscard]] std::string_view name() const override;
    [[nodiscard]] std::string variant() const override;
    void preprocess() override;
    [[nodiscard]] PathResult query(VertexId source, VertexId target) const override;

private:
    const Graph &graph_;
    Variant variant_;
    ReverseAdjacency reverse_;
    mutable StampedVector<Distance> dist_f_;
    mutable StampedVector<Distance> dist_b_;
};

} // namespace transport
