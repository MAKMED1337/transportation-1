#pragma once

#include "algorithms/alt/landmarks.hpp"
#include "algorithms/routing_algorithm.hpp"
#include "algorithms/stamped_vector.hpp"
#include "graph/graph.hpp"
#include "graph/reverse_graph.hpp"

#include <string>
#include <string_view>
#include <vector>

namespace transport {

// Unidirectional A* with ALT potentials (landmarks + triangle inequality).
class AltAlgorithm final : public RoutingAlgorithm {
public:
    explicit AltAlgorithm(const Graph &graph, uint32_t landmark_count, alt::LandmarkStrategy strategy,
                          uint32_t active_landmarks, uint32_t seed);

    [[nodiscard]] std::string_view name() const override;
    [[nodiscard]] std::string variant() const override;
    void preprocess() override;
    [[nodiscard]] PathResult query(VertexId source, VertexId target) const override;

    [[nodiscard]] uint64_t landmark_table_bytes() const;

private:
    const Graph &graph_;
    uint32_t landmark_count_;
    alt::LandmarkStrategy strategy_;
    uint32_t active_landmarks_;
    uint32_t seed_;
    ReverseAdjacency reverse_;
    alt::LandmarkSet ls_;
    mutable StampedVector<Distance> dist_;
    mutable std::vector<uint32_t> active_idx_; // scratch: active landmark indices for current query

    // Returns the ALT lower-bound potential pi_f(v) for a query to `target`.
    [[nodiscard]] int64_t potential(VertexId v, VertexId target) const;

    // Select `active_landmarks_` best landmark indices for query (s,t) based on score.
    void select_active(VertexId source, VertexId target) const;
};

// Bidirectional ALT using the consistent averaged-potential scheme (same as bidi_astar consistent
// but with ALT potentials instead of haversine).
class BidirectionalAltAlgorithm final : public RoutingAlgorithm {
public:
    explicit BidirectionalAltAlgorithm(const Graph &graph, uint32_t landmark_count, alt::LandmarkStrategy strategy,
                                       uint32_t active_landmarks, uint32_t seed);

    [[nodiscard]] std::string_view name() const override;
    [[nodiscard]] std::string variant() const override;
    void preprocess() override;
    [[nodiscard]] PathResult query(VertexId source, VertexId target) const override;

    [[nodiscard]] uint64_t landmark_table_bytes() const;

private:
    const Graph &graph_;
    uint32_t landmark_count_;
    alt::LandmarkStrategy strategy_;
    uint32_t active_landmarks_;
    uint32_t seed_;
    ReverseAdjacency reverse_;
    alt::LandmarkSet ls_;
    mutable StampedVector<Distance> dist_f_;
    mutable StampedVector<Distance> dist_b_;
    mutable std::vector<uint32_t> active_idx_;

    void select_active(VertexId source, VertexId target) const;
    [[nodiscard]] int64_t pot_f(VertexId v, VertexId target) const;
    [[nodiscard]] int64_t pot_b(VertexId v, VertexId source) const;
};

} // namespace transport
