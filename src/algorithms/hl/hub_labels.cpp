#include "algorithms/hl/hub_labels.hpp"

#include "apps/bench_utils.hpp"

#include <algorithm>
#include <cstdint>
#include <functional>
#include <limits>
#include <queue>
#include <sstream>
#include <stdexcept>
#include <string_view>
#include <unordered_map>
#include <vector>

namespace transport {

namespace {

constexpr uint32_t kInf32 = std::numeric_limits<uint32_t>::max();

struct U32HeapNode {
    uint32_t key;
    VertexId v;
    bool operator>(const U32HeapNode &o) const { return key > o.key; }
};
using U32Pq = std::priority_queue<U32HeapNode, std::vector<U32HeapNode>, std::greater<U32HeapNode>>;

} // namespace

HubLabelsAlgorithm::HubLabelsAlgorithm(const Graph &graph, float label_fraction, uint64_t memory_budget_gb,
                                       uint32_t threads)
    : graph_(graph), label_fraction_(label_fraction), memory_budget_gb_(memory_budget_gb), threads_(threads) {}

std::string_view HubLabelsAlgorithm::name() const { return "hl"; }

std::string HubLabelsAlgorithm::variant() const {
    std::ostringstream oss;
    oss << "label_fraction=" << label_fraction_;
    return oss.str();
}

void HubLabelsAlgorithm::inject_ch(ContractionHierarchy ch) {
    ch_ = std::move(ch);
    ch_provided_ = true;
}

// ----- label intersection helpers -----

Distance HubLabelsAlgorithm::intersect_labels(std::span<const HlEntry> f, std::span<const HlEntry> b) {
    Distance best = kUnreachable;
    size_t i = 0;
    size_t j = 0;
    while (i < f.size() && j < b.size()) {
        if (f[i].hub == b[j].hub) {
            const Distance d = static_cast<Distance>(f[i].dist) + b[j].dist;
            if (d < best) {
                best = d;
            }
            ++i;
            ++j;
        } else if (f[i].hub < b[j].hub) {
            ++i;
        } else {
            ++j;
        }
    }
    return best;
}

uint32_t HubLabelsAlgorithm::intersect_u32(const std::vector<HlEntry> &a, const std::vector<HlEntry> &b) {
    uint64_t best = kInf32;
    size_t i = 0;
    size_t j = 0;
    while (i < a.size() && j < b.size()) {
        if (a[i].hub == b[j].hub) {
            const uint64_t d = static_cast<uint64_t>(a[i].dist) + b[j].dist;
            if (d < best) {
                best = d;
            }
            ++i;
            ++j;
        } else if (a[i].hub < b[j].hub) {
            ++i;
        } else {
            ++j;
        }
    }
    return best >= kInf32 ? kInf32 : static_cast<uint32_t>(best);
}

// ----- preprocess -----

void HubLabelsAlgorithm::preprocess() {
    if (preprocessed_) {
        return;
    }
    bench::Stopwatch total_sw;

    if (!ch_provided_) {
        ContractionHierarchyAlgorithm ch_algo(graph_);
        ch_algo.preprocess();
        ch_ = ch_algo.get_ch();
        stats_.ch_was_injected = false;
    } else {
        stats_.ch_was_injected = true;
    }

    const uint32_t V = ch_.vertex_count();
    labeled_count_ = static_cast<uint32_t>(static_cast<double>(V) * label_fraction_);
    if (labeled_count_ == 0) {
        labeled_count_ = 1; // at least the apex
    }
    if (labeled_count_ > V) {
        labeled_count_ = V;
    }
    label_threshold_ = V - labeled_count_;

    fwd_scratch_ = StampedVector<uint32_t>(V, kInf32);
    bwd_scratch_ = StampedVector<uint32_t>(V, kInf32);

    build_labels();

    stats_.label_fraction = label_fraction_;
    stats_.labeled_vertices = labeled_count_;
    stats_.label_build_wall_s = bench::to_seconds(total_sw.wall_elapsed());
    stats_.label_build_cpu_s = bench::to_seconds(total_sw.cpu_elapsed());
    preprocessed_ = true;
}

void HubLabelsAlgorithm::build_labels() {
    bench::Stopwatch sw;
    const uint32_t V = ch_.vertex_count();
    const uint64_t budget_bytes = memory_budget_gb_ * 1024ULL * 1024 * 1024;

    // inv_rank[r] = vertex with rank r.
    std::vector<VertexId> inv_rank(V);
    for (VertexId v = 0; v < V; ++v) {
        inv_rank[ch_.rank[v]] = v;
    }

    // Temporary per-vertex label storage indexed by vertex_id.
    // Unlabeled vertices keep empty vectors throughout.
    std::vector<std::vector<HlEntry>> temp_fwd(V);
    std::vector<std::vector<HlEntry>> temp_bwd(V);

    uint64_t prune_drops = 0;
    uint64_t total_fwd_entries = 0;
    uint64_t total_bwd_entries = 0;
    uint32_t max_fwd = 0;
    uint32_t max_bwd = 0;

    // Scratch for candidate gathering.
    std::unordered_map<VertexId, uint32_t> cand_map;
    cand_map.reserve(512);
    std::vector<HlEntry> sorted_cands;
    sorted_cands.reserve(512);

    // Process labeled vertices in descending rank order.
    for (uint32_t rank = V; rank-- > label_threshold_;) {
        const VertexId v = inv_rank[rank];

        // --- Build L_f(v) ---
        cand_map.clear();
        cand_map[v] = 0;
        for (uint64_t i = ch_.forward_offsets[v], end = ch_.forward_offsets[v + 1]; i < end; ++i) {
            const Edge &e = ch_.forward_edges[static_cast<size_t>(i)];
            // u has higher rank → labeled → temp_fwd[u] is already built.
            for (const HlEntry &he : temp_fwd[e.to]) {
                const uint64_t d64 = static_cast<uint64_t>(he.dist) + e.weight;
                const uint32_t d = d64 >= kInf32 ? kInf32 : static_cast<uint32_t>(d64);
                auto [it, inserted] = cand_map.emplace(he.hub, d);
                if (!inserted && d < it->second) {
                    it->second = d;
                }
            }
        }
        sorted_cands.clear();
        for (const auto &[h, d] : cand_map) {
            sorted_cands.push_back({h, d});
        }
        std::sort(sorted_cands.begin(), sorted_cands.end());

        std::vector<HlEntry> &lf = temp_fwd[v];
        lf.clear();
        for (const HlEntry &cand : sorted_cands) {
            const uint32_t check = intersect_u32(lf, temp_bwd[cand.hub]);
            if (check < cand.dist) {
                ++prune_drops;
            } else {
                lf.push_back(cand);
            }
        }

        // --- Build L_b(v) ---
        cand_map.clear();
        cand_map[v] = 0;
        for (uint64_t i = ch_.backward_offsets[v], end = ch_.backward_offsets[v + 1]; i < end; ++i) {
            const Edge &e = ch_.backward_edges[static_cast<size_t>(i)];
            for (const HlEntry &he : temp_bwd[e.to]) {
                const uint64_t d64 = static_cast<uint64_t>(he.dist) + e.weight;
                const uint32_t d = d64 >= kInf32 ? kInf32 : static_cast<uint32_t>(d64);
                auto [it, inserted] = cand_map.emplace(he.hub, d);
                if (!inserted && d < it->second) {
                    it->second = d;
                }
            }
        }
        sorted_cands.clear();
        for (const auto &[h, d] : cand_map) {
            sorted_cands.push_back({h, d});
        }
        std::sort(sorted_cands.begin(), sorted_cands.end());

        std::vector<HlEntry> &lb = temp_bwd[v];
        lb.clear();
        for (const HlEntry &cand : sorted_cands) {
            const uint32_t check = intersect_u32(temp_fwd[cand.hub], lb);
            if (check < cand.dist) {
                ++prune_drops;
            } else {
                lb.push_back(cand);
            }
        }

        total_fwd_entries += lf.size();
        total_bwd_entries += lb.size();
        if (static_cast<uint32_t>(lf.size()) > max_fwd) {
            max_fwd = static_cast<uint32_t>(lf.size());
        }
        if (static_cast<uint32_t>(lb.size()) > max_bwd) {
            max_bwd = static_cast<uint32_t>(lb.size());
        }

        // Incremental memory budget check every 64k vertices.
        // Peak memory = 2 × label_data (temp vectors + CSR copy coexist during assembly)
        //             + fixed overhead (temp vector headers, inv_rank, offsets, CH, graph ≈ 3 GB).
        const uint32_t processed = V - rank;
        if ((processed & 0xFFFFu) == 0 && processed > 0) {
            const uint64_t bytes_so_far = (total_fwd_entries + total_bwd_entries) * sizeof(HlEntry);
            const uint64_t projected_label_bytes = bytes_so_far * labeled_count_ / processed;
            // fixed: temp vector headers (V×48) + inv_rank (V×4) + scratch (V×8) + offsets (V×16) + CH+graph (~2 GB)
            const uint64_t fixed_bytes = static_cast<uint64_t>(V) * 76 + 2ULL * 1024 * 1024 * 1024;
            const uint64_t projected_peak = 2 * projected_label_bytes + fixed_bytes;
            if (projected_peak > budget_bytes) {
                throw std::runtime_error("hl: projected peak memory " + std::to_string(projected_peak / (1024 * 1024)) +
                                         " MB exceeds budget " + std::to_string(memory_budget_gb_) +
                                         " GB after processing " + std::to_string(processed) + " of " +
                                         std::to_string(labeled_count_) + " labeled vertices");
            }
        }
    }

    // Assemble CSR from temp vectors.
    fwd_offsets_.resize(static_cast<size_t>(V) + 1);
    bwd_offsets_.resize(static_cast<size_t>(V) + 1);
    fwd_labels_.reserve(total_fwd_entries);
    bwd_labels_.reserve(total_bwd_entries);

    fwd_offsets_[0] = 0;
    bwd_offsets_[0] = 0;
    for (VertexId v = 0; v < V; ++v) {
        fwd_offsets_[v + 1] = fwd_offsets_[v] + temp_fwd[v].size();
        bwd_offsets_[v + 1] = bwd_offsets_[v] + temp_bwd[v].size();
        fwd_labels_.insert(fwd_labels_.end(), temp_fwd[v].begin(), temp_fwd[v].end());
        bwd_labels_.insert(bwd_labels_.end(), temp_bwd[v].begin(), temp_bwd[v].end());
        // Release memory as we go.
        temp_fwd[v] = std::vector<HlEntry>{};
        temp_bwd[v] = std::vector<HlEntry>{};
    }

    const double label_bytes = static_cast<double>((total_fwd_entries + total_bwd_entries) * sizeof(HlEntry));

    stats_.label_build_wall_s = bench::to_seconds(sw.wall_elapsed());
    stats_.label_build_cpu_s = bench::to_seconds(sw.cpu_elapsed());
    stats_.avg_label_size_fwd = labeled_count_ > 0 ? static_cast<double>(total_fwd_entries) / labeled_count_ : 0.0;
    stats_.avg_label_size_bwd = labeled_count_ > 0 ? static_cast<double>(total_bwd_entries) / labeled_count_ : 0.0;
    stats_.max_label_size_fwd = max_fwd;
    stats_.max_label_size_bwd = max_bwd;
    stats_.label_memory_mb = label_bytes / (1024.0 * 1024.0);
    stats_.prune_drops = prune_drops;
}

// ----- collect helpers -----

void HubLabelsAlgorithm::collect_fwd(VertexId source, std::vector<HlEntry> &out,
                                     std::vector<VertexId> &unlabeled_settled) const {
    // Upward search from source on CH forward arcs.
    // Stops relaxing at labeled vertices; collects their forward labels.
    std::unordered_map<VertexId, uint32_t> hub_best;
    hub_best.reserve(256);

    U32Pq pq;
    fwd_scratch_.set(source, 0);
    pq.push({0, source});

    while (!pq.empty()) {
        const U32HeapNode top = pq.top();
        pq.pop();
        if (top.key != fwd_scratch_.get(top.v)) {
            continue;
        }
        if (is_labeled(top.v)) {
            // Collect labels, don't relax.
            for (const HlEntry &e : fwd_label(top.v)) {
                const uint64_t d64 = static_cast<uint64_t>(top.key) + e.dist;
                if (d64 >= kInf32) {
                    continue;
                }
                const uint32_t d = static_cast<uint32_t>(d64);
                auto [it, inserted] = hub_best.emplace(e.hub, d);
                if (!inserted && d < it->second) {
                    it->second = d;
                }
            }
            continue;
        }
        unlabeled_settled.push_back(top.v);
        for (uint64_t i = ch_.forward_offsets[top.v], end = ch_.forward_offsets[top.v + 1]; i < end; ++i) {
            const Edge &e = ch_.forward_edges[static_cast<size_t>(i)];
            const uint64_t nd64 = static_cast<uint64_t>(top.key) + e.weight;
            if (nd64 >= kInf32) {
                continue;
            }
            const uint32_t nd = static_cast<uint32_t>(nd64);
            if (nd < fwd_scratch_.get(e.to)) {
                fwd_scratch_.set(e.to, nd);
                pq.push({nd, e.to});
            }
        }
    }

    out.clear();
    out.reserve(hub_best.size());
    for (const auto &[h, d] : hub_best) {
        out.push_back({h, d});
    }
    std::sort(out.begin(), out.end());
}

void HubLabelsAlgorithm::collect_bwd(VertexId target, std::vector<HlEntry> &out,
                                     std::vector<VertexId> &unlabeled_settled) const {
    std::unordered_map<VertexId, uint32_t> hub_best;
    hub_best.reserve(256);

    U32Pq pq;
    bwd_scratch_.set(target, 0);
    pq.push({0, target});

    while (!pq.empty()) {
        const U32HeapNode top = pq.top();
        pq.pop();
        if (top.key != bwd_scratch_.get(top.v)) {
            continue;
        }
        if (is_labeled(top.v)) {
            for (const HlEntry &e : bwd_label(top.v)) {
                const uint64_t d64 = static_cast<uint64_t>(top.key) + e.dist;
                if (d64 >= kInf32) {
                    continue;
                }
                const uint32_t d = static_cast<uint32_t>(d64);
                auto [it, inserted] = hub_best.emplace(e.hub, d);
                if (!inserted && d < it->second) {
                    it->second = d;
                }
            }
            continue;
        }
        unlabeled_settled.push_back(top.v);
        for (uint64_t i = ch_.backward_offsets[top.v], end = ch_.backward_offsets[top.v + 1]; i < end; ++i) {
            const Edge &e = ch_.backward_edges[static_cast<size_t>(i)];
            const uint64_t nd64 = static_cast<uint64_t>(top.key) + e.weight;
            if (nd64 >= kInf32) {
                continue;
            }
            const uint32_t nd = static_cast<uint32_t>(nd64);
            if (nd < bwd_scratch_.get(e.to)) {
                bwd_scratch_.set(e.to, nd);
                pq.push({nd, e.to});
            }
        }
    }

    out.clear();
    out.reserve(hub_best.size());
    for (const auto &[h, d] : hub_best) {
        out.push_back({h, d});
    }
    std::sort(out.begin(), out.end());
}

// ----- query -----

PathResult HubLabelsAlgorithm::query(VertexId source, VertexId target) const {
    QueryStats stats;
    const bool s_lab = is_labeled(source);
    const bool t_lab = is_labeled(target);
    Distance dist = kUnreachable;

    if (s_lab && t_lab) {
        dist = intersect_labels(fwd_label(source), bwd_label(target));
    } else if (s_lab) {
        // source labeled, target unlabeled
        bwd_scratch_.reset();
        std::vector<VertexId> dummy;
        collect_bwd(target, collect_buf_bwd_, dummy);
        bwd_scratch_.reset();
        dist = intersect_labels(fwd_label(source), collect_buf_bwd_);
    } else if (t_lab) {
        // source unlabeled, target labeled
        fwd_scratch_.reset();
        std::vector<VertexId> dummy;
        collect_fwd(source, collect_buf_fwd_, dummy);
        fwd_scratch_.reset();
        dist = intersect_labels(collect_buf_fwd_, bwd_label(target));
    } else {
        // Both unlabeled: collect + mu_low from bidi CH below threshold.
        fwd_scratch_.reset();
        bwd_scratch_.reset();

        std::vector<VertexId> fwd_unlabeled;
        std::vector<VertexId> bwd_unlabeled;
        collect_fwd(source, collect_buf_fwd_, fwd_unlabeled);
        collect_bwd(target, collect_buf_bwd_, bwd_unlabeled);

        // mu_low: min dist_fwd[v] + dist_bwd[v] over vertices settled in both searches.
        Distance mu_low = kUnreachable;
        for (const VertexId v : fwd_unlabeled) {
            const uint32_t df = fwd_scratch_.get(v);
            const uint32_t db = bwd_scratch_.get(v);
            if (df != kInf32 && db != kInf32) {
                const Distance candidate = static_cast<Distance>(df) + db;
                if (candidate < mu_low) {
                    mu_low = candidate;
                }
            }
        }

        fwd_scratch_.reset();
        bwd_scratch_.reset();

        dist = std::min(mu_low, intersect_labels(collect_buf_fwd_, collect_buf_bwd_));
        if (mu_low < kUnreachable) {
            stats.used_fallback = true;
        }
    }

    return {dist, stats};
}

} // namespace transport
