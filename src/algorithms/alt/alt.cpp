#include "algorithms/alt/alt.hpp"

#include "algorithms/heap_node.hpp"

#include <algorithm>
#include <cstddef>
#include <cstdint>
#include <functional>
#include <limits>
#include <queue>
#include <sstream>
#include <string>
#include <string_view>
#include <vector>

namespace transport {

// --- helpers ---

namespace {

// Forward potential for ALT unidirectional query to `target`:
// pi_f(v) = max over active landmarks of max(d(l,t)-d(l,v), d(v,l)-d(t,l))
// where both terms use the triangle inequality on the directed graph.
int64_t compute_potential(const alt::LandmarkSet &ls, const std::vector<uint32_t> &active_idx, VertexId v,
                          VertexId target) {
    const uint32_t V = ls.vertex_count;
    int64_t best = 0;
    for (const uint32_t li : active_idx) {
        const uint32_t dlt = ls.dist_from[static_cast<size_t>(li) * V + target]; // d(l, target)
        const uint32_t dlv = ls.dist_from[static_cast<size_t>(li) * V + v];      // d(l, v)
        const uint32_t dvl = ls.dist_to[static_cast<size_t>(li) * V + v];        // d(v, l)
        const uint32_t dtl = ls.dist_to[static_cast<size_t>(li) * V + target];   // d(target, l)

        if (dlt != alt::kLandmarkInf && dlv != alt::kLandmarkInf) {
            const int64_t term = static_cast<int64_t>(dlt) - static_cast<int64_t>(dlv);
            if (term > best) {
                best = term;
            }
        }
        if (dvl != alt::kLandmarkInf && dtl != alt::kLandmarkInf) {
            const int64_t term = static_cast<int64_t>(dvl) - static_cast<int64_t>(dtl);
            if (term > best) {
                best = term;
            }
        }
    }
    return best; // already >= 0
}

// Backward potential for ALT bidi query to `source`:
// pi_b(v) = max over active landmarks of max(d(l,v)-d(l,s), d(s,l)-d(v,l))
int64_t compute_potential_b(const alt::LandmarkSet &ls, const std::vector<uint32_t> &active_idx, VertexId v,
                            VertexId source) {
    const uint32_t V = ls.vertex_count;
    int64_t best = 0;
    for (const uint32_t li : active_idx) {
        const uint32_t dls = ls.dist_from[static_cast<size_t>(li) * V + source]; // d(l, source)
        const uint32_t dlv = ls.dist_from[static_cast<size_t>(li) * V + v];      // d(l, v)
        const uint32_t dvl = ls.dist_to[static_cast<size_t>(li) * V + v];        // d(v, l)
        const uint32_t dsl = ls.dist_to[static_cast<size_t>(li) * V + source];   // d(source, l)

        if (dlv != alt::kLandmarkInf && dls != alt::kLandmarkInf) {
            const int64_t term = static_cast<int64_t>(dlv) - static_cast<int64_t>(dls);
            if (term > best) {
                best = term;
            }
        }
        if (dsl != alt::kLandmarkInf && dvl != alt::kLandmarkInf) {
            const int64_t term = static_cast<int64_t>(dsl) - static_cast<int64_t>(dvl);
            if (term > best) {
                best = term;
            }
        }
    }
    return best;
}

// Score a landmark for query (s,t): score = max(d(l,t)-d(l,s), d(s,l)-d(t,l)) — larger = better guidance
int64_t landmark_score(const alt::LandmarkSet &ls, uint32_t li, VertexId s, VertexId t) {
    const uint32_t V = ls.vertex_count;
    const uint32_t dlt = ls.dist_from[static_cast<size_t>(li) * V + t];
    const uint32_t dls = ls.dist_from[static_cast<size_t>(li) * V + s];
    const uint32_t dsl = ls.dist_to[static_cast<size_t>(li) * V + s];
    const uint32_t dtl = ls.dist_to[static_cast<size_t>(li) * V + t];

    int64_t score = 0;
    if (dlt != alt::kLandmarkInf && dls != alt::kLandmarkInf) {
        score = std::max(score, static_cast<int64_t>(dlt) - static_cast<int64_t>(dls));
    }
    if (dsl != alt::kLandmarkInf && dtl != alt::kLandmarkInf) {
        score = std::max(score, static_cast<int64_t>(dsl) - static_cast<int64_t>(dtl));
    }
    return score;
}

void do_select_active(const alt::LandmarkSet &ls, VertexId source, VertexId target, uint32_t active_count,
                      std::vector<uint32_t> &active_idx) {
    const auto k = static_cast<uint32_t>(ls.landmarks.size());
    active_count = std::min(active_count, k);
    active_idx.resize(active_count);

    // Score all landmarks and keep top `active_count`
    std::vector<std::pair<int64_t, uint32_t>> scores;
    scores.reserve(k);
    for (uint32_t i = 0; i < k; ++i) {
        scores.emplace_back(landmark_score(ls, i, source, target), i);
    }
    std::partial_sort(scores.begin(), scores.begin() + active_count, scores.end(),
                      [](const auto &a, const auto &b) { return a.first > b.first; });
    for (uint32_t i = 0; i < active_count; ++i) {
        active_idx[i] = scores[i].second;
    }
}

std::string make_variant(uint32_t k, const std::string &strategy, uint32_t active, bool bidi) {
    std::ostringstream oss;
    oss << (bidi ? "bidi ALT" : "ALT") << " k=" << k << " strategy=" << strategy << " active=" << active;
    return oss.str();
}

} // namespace

// ---- AltAlgorithm ----

AltAlgorithm::AltAlgorithm(const Graph &graph, uint32_t landmark_count, alt::LandmarkStrategy strategy,
                           uint32_t active_landmarks, uint32_t seed)
    : graph_(graph), landmark_count_(landmark_count), strategy_(strategy), active_landmarks_(active_landmarks),
      seed_(seed), dist_(graph.vertex_count(), kUnreachable) {}

std::string_view AltAlgorithm::name() const { return "alt"; }

std::string AltAlgorithm::variant() const {
    return make_variant(landmark_count_, ls_.strategy_name.empty() ? "?" : ls_.strategy_name, active_landmarks_, false);
}

void AltAlgorithm::preprocess() {
    reverse_ = build_reverse_adjacency(graph_);
    ls_ = alt::build_landmarks(graph_, reverse_, landmark_count_, strategy_, seed_);
}

uint64_t AltAlgorithm::landmark_table_bytes() const {
    return static_cast<uint64_t>((ls_.dist_from.size() + ls_.dist_to.size()) * sizeof(uint32_t));
}

int64_t AltAlgorithm::potential(VertexId v, VertexId target) const {
    return compute_potential(ls_, active_idx_, v, target);
}

void AltAlgorithm::select_active(VertexId source, VertexId target) const {
    do_select_active(ls_, source, target, active_landmarks_, active_idx_);
}

PathResult AltAlgorithm::query(VertexId source, VertexId target) const {
    if (source == target) {
        return PathResult{0, QueryStats{}};
    }

    select_active(source, target);
    dist_.reset();

    std::priority_queue<SignedHeapNode, std::vector<SignedHeapNode>, std::greater<>> pq;
    dist_.set(source, 0);
    const int64_t h_source = potential(source, target);
    pq.push({h_source, source});

    QueryStats stats;
    ++stats.heuristic_evals;

    while (!pq.empty()) {
        const SignedHeapNode top = pq.top();
        pq.pop();

        const Distance gv = dist_.get(top.v);
        ++stats.heuristic_evals;
        if (top.key != static_cast<int64_t>(gv) + potential(top.v, target)) {
            continue; // stale
        }

        ++stats.settled;
        if (top.v == target) {
            break;
        }

        const uint64_t begin = graph_.offsets[top.v];
        const uint64_t end = graph_.offsets[top.v + 1];
        for (uint64_t i = begin; i < end; ++i) {
            const Edge &e = graph_.edges[static_cast<size_t>(i)];
            ++stats.relaxed_arcs;
            const Distance ng = gv + e.weight;
            if (ng < dist_.get(e.to)) {
                dist_.set(e.to, ng);
                ++stats.heuristic_evals;
                pq.push({static_cast<int64_t>(ng) + potential(e.to, target), e.to});
                ++stats.heap_pushes;
            }
        }
    }

    return PathResult{dist_.get(target), stats};
}

// ---- BidirectionalAltAlgorithm ----

BidirectionalAltAlgorithm::BidirectionalAltAlgorithm(const Graph &graph, uint32_t landmark_count,
                                                     alt::LandmarkStrategy strategy, uint32_t active_landmarks,
                                                     uint32_t seed)
    : graph_(graph), landmark_count_(landmark_count), strategy_(strategy), active_landmarks_(active_landmarks),
      seed_(seed), dist_f_(graph.vertex_count(), kUnreachable), dist_b_(graph.vertex_count(), kUnreachable) {}

std::string_view BidirectionalAltAlgorithm::name() const { return "alt_bidi"; }

std::string BidirectionalAltAlgorithm::variant() const {
    return make_variant(landmark_count_, ls_.strategy_name.empty() ? "?" : ls_.strategy_name, active_landmarks_, true);
}

void BidirectionalAltAlgorithm::preprocess() {
    reverse_ = build_reverse_adjacency(graph_);
    ls_ = alt::build_landmarks(graph_, reverse_, landmark_count_, strategy_, seed_);
}

uint64_t BidirectionalAltAlgorithm::landmark_table_bytes() const {
    return static_cast<uint64_t>((ls_.dist_from.size() + ls_.dist_to.size()) * sizeof(uint32_t));
}

void BidirectionalAltAlgorithm::select_active(VertexId source, VertexId target) const {
    do_select_active(ls_, source, target, active_landmarks_, active_idx_);
}

int64_t BidirectionalAltAlgorithm::pot_f(VertexId v, VertexId target) const {
    return compute_potential(ls_, active_idx_, v, target);
}

int64_t BidirectionalAltAlgorithm::pot_b(VertexId v, VertexId source) const {
    return compute_potential_b(ls_, active_idx_, v, source);
}

PathResult BidirectionalAltAlgorithm::query(VertexId source, VertexId target) const {
    if (source == target) {
        return PathResult{0, QueryStats{}};
    }

    select_active(source, target);
    dist_f_.reset();
    dist_b_.reset();

    // Consistent averaged potentials: p_f = (pi_f - pi_b)/2, p_b = -p_f
    auto key_f = [&](VertexId v, Distance gv) -> int64_t {
        return static_cast<int64_t>(gv) + (pot_f(v, target) - pot_b(v, source)) / 2;
    };
    auto key_b = [&](VertexId v, Distance gv) -> int64_t {
        return static_cast<int64_t>(gv) - (pot_f(v, target) - pot_b(v, source)) / 2;
    };

    std::priority_queue<SignedHeapNode, std::vector<SignedHeapNode>, std::greater<>> pq_f, pq_b;
    dist_f_.set(source, 0);
    dist_b_.set(target, 0);
    pq_f.push({key_f(source, 0), source});
    pq_b.push({key_b(target, 0), target});

    constexpr int64_t kSentinel = std::numeric_limits<int64_t>::max() / 4;
    Distance mu = kUnreachable;
    QueryStats stats;

    while (!pq_f.empty() || !pq_b.empty()) {
        const int64_t top_f = pq_f.empty() ? kSentinel : pq_f.top().key;
        const int64_t top_b = pq_b.empty() ? kSentinel : pq_b.top().key;

        if (mu != kUnreachable) {
            if (top_f != kSentinel && top_b != kSentinel && top_f + top_b >= static_cast<int64_t>(mu)) {
                break;
            }
            if (top_f != kSentinel && top_f >= static_cast<int64_t>(mu)) {
                break;
            }
            if (top_b != kSentinel && top_b >= static_cast<int64_t>(mu)) {
                break;
            }
        }

        const bool go_forward = (top_f <= top_b);

        if (go_forward) {
            const SignedHeapNode top = pq_f.top();
            pq_f.pop();
            const Distance gv = dist_f_.get(top.v);
            ++stats.heuristic_evals;
            if (top.key != key_f(top.v, gv)) {
                continue;
            }
            ++stats.settled;
            ++stats.settled_forward;

            const uint64_t begin = graph_.offsets[top.v];
            const uint64_t end = graph_.offsets[top.v + 1];
            for (uint64_t i = begin; i < end; ++i) {
                const Edge &e = graph_.edges[static_cast<size_t>(i)];
                ++stats.relaxed_arcs;
                const Distance ng = gv + e.weight;
                if (ng < dist_f_.get(e.to)) {
                    dist_f_.set(e.to, ng);
                    ++stats.heuristic_evals;
                    pq_f.push({key_f(e.to, ng), e.to});
                    ++stats.heap_pushes;
                    const Distance db = dist_b_.get(e.to);
                    if (db < kUnreachable && ng + db < mu) {
                        mu = ng + db;
                    }
                }
            }
        } else {
            const SignedHeapNode top = pq_b.top();
            pq_b.pop();
            const Distance gv = dist_b_.get(top.v);
            ++stats.heuristic_evals;
            if (top.key != key_b(top.v, gv)) {
                continue;
            }
            ++stats.settled;
            ++stats.settled_backward;

            const uint64_t begin = reverse_.offsets[top.v];
            const uint64_t end = reverse_.offsets[top.v + 1];
            for (uint64_t i = begin; i < end; ++i) {
                const Edge &e = reverse_.edges[static_cast<size_t>(i)];
                ++stats.relaxed_arcs;
                const Distance ng = gv + e.weight;
                if (ng < dist_b_.get(e.to)) {
                    dist_b_.set(e.to, ng);
                    ++stats.heuristic_evals;
                    pq_b.push({key_b(e.to, ng), e.to});
                    ++stats.heap_pushes;
                    const Distance df = dist_f_.get(e.to);
                    if (df < kUnreachable && df + ng < mu) {
                        mu = df + ng;
                    }
                }
            }
        }
    }

    return PathResult{mu, stats};
}

} // namespace transport
