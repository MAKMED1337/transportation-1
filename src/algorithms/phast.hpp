#pragma once

#include "algorithms/ch/contraction_hierarchy.hpp"
#include "graph/types.hpp"

#include <cstdint>
#include <vector>

namespace transport {

// Pack size for batched PHAST. 16 simultaneous targets fit in one cache line per vertex (uint32_t).
constexpr uint32_t kPhastPack = 16;

// All-to-one: compute d(v, target) for every vertex v. dist must have size vertex_count.
// Uses backward upward Dijkstra from target + forward downward sweep.
void phast_all_to_one(const ContractionHierarchy &ch, const std::vector<VertexId> &inv_rank, VertexId target,
                      std::vector<uint32_t> &dist);

// One-to-all: compute d(source, v) for every vertex v. dist must have size vertex_count.
// Uses forward upward Dijkstra from source + backward downward sweep.
void phast_one_to_all(const ContractionHierarchy &ch, const std::vector<VertexId> &inv_rank, VertexId source,
                      std::vector<uint32_t> &dist);

// Batched all-to-one for P=kPhastPack targets simultaneously.
// targets must have exactly kPhastPack entries (pad with duplicate or sentinel as needed).
// dist_pack must have size vertex_count * kPhastPack (layout: dist_pack[v*P + i]).
void phast_all_to_one_batch(const ContractionHierarchy &ch, const std::vector<VertexId> &inv_rank,
                            const std::vector<VertexId> &targets, std::vector<uint32_t> &dist_pack);

// Batched one-to-all for kPhastPack sources simultaneously.
void phast_one_to_all_batch(const ContractionHierarchy &ch, const std::vector<VertexId> &inv_rank,
                            const std::vector<VertexId> &sources, std::vector<uint32_t> &dist_pack);

// Precompute the inverse rank array (inv_rank[r] = vertex with rank r) needed by PHAST.
[[nodiscard]] std::vector<VertexId> build_inv_rank(const ContractionHierarchy &ch);

} // namespace transport
