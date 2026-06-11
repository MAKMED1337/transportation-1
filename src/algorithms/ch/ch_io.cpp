#include "algorithms/ch/ch_io.hpp"

#include "graph/graph.hpp"

#include <cstdint>
#include <fstream>
#include <stdexcept>
#include <string>

namespace transport::ch {

bool save_ch(const ContractionHierarchy &ch, const std::string &path) {
    std::ofstream f(path, std::ios::binary);
    if (!f) {
        return false;
    }

    const uint32_t V = ch.vertex_count();
    const uint64_t fwd_edges = static_cast<uint64_t>(ch.forward_edges.size());
    const uint64_t bwd_edges = static_cast<uint64_t>(ch.backward_edges.size());

    auto write = [&](const void *data, size_t n) {
        f.write(static_cast<const char *>(data), static_cast<std::streamsize>(n));
    };

    write(&kChMagic, sizeof(kChMagic));
    write(&kChVersion, sizeof(kChVersion));
    write(&V, sizeof(V));
    write(&fwd_edges, sizeof(fwd_edges));
    write(&bwd_edges, sizeof(bwd_edges));
    write(ch.rank.data(), static_cast<size_t>(V) * sizeof(uint32_t));
    write(ch.forward_offsets.data(), (static_cast<size_t>(V) + 1) * sizeof(uint64_t));
    write(ch.forward_edges.data(), static_cast<size_t>(fwd_edges) * sizeof(Edge));
    write(ch.backward_offsets.data(), (static_cast<size_t>(V) + 1) * sizeof(uint64_t));
    write(ch.backward_edges.data(), static_cast<size_t>(bwd_edges) * sizeof(Edge));

    return f.good();
}

ContractionHierarchy load_ch(const std::string &path) {
    std::ifstream f(path, std::ios::binary);
    if (!f) {
        throw std::runtime_error("ch_io: cannot open " + path);
    }

    auto read_val = [&](auto &val) { f.read(reinterpret_cast<char *>(&val), sizeof(val)); };

    uint32_t magic = 0;
    uint32_t version = 0;
    uint32_t V = 0;
    uint64_t fwd_edges = 0;
    uint64_t bwd_edges = 0;
    read_val(magic);
    read_val(version);
    read_val(V);
    read_val(fwd_edges);
    read_val(bwd_edges);

    if (magic != kChMagic) {
        throw std::runtime_error("ch_io: bad magic in " + path);
    }
    if (version != kChVersion) {
        throw std::runtime_error("ch_io: unsupported version " + std::to_string(version) + " in " + path);
    }

    ContractionHierarchy ch;
    ch.rank.resize(V);
    ch.forward_offsets.resize(static_cast<size_t>(V) + 1);
    ch.forward_edges.resize(static_cast<size_t>(fwd_edges));
    ch.backward_offsets.resize(static_cast<size_t>(V) + 1);
    ch.backward_edges.resize(static_cast<size_t>(bwd_edges));

    f.read(reinterpret_cast<char *>(ch.rank.data()), static_cast<size_t>(V) * sizeof(uint32_t));
    f.read(reinterpret_cast<char *>(ch.forward_offsets.data()), (static_cast<size_t>(V) + 1) * sizeof(uint64_t));
    f.read(reinterpret_cast<char *>(ch.forward_edges.data()), static_cast<size_t>(fwd_edges) * sizeof(Edge));
    f.read(reinterpret_cast<char *>(ch.backward_offsets.data()), (static_cast<size_t>(V) + 1) * sizeof(uint64_t));
    f.read(reinterpret_cast<char *>(ch.backward_edges.data()), static_cast<size_t>(bwd_edges) * sizeof(Edge));

    if (!f) {
        throw std::runtime_error("ch_io: truncated data in " + path);
    }
    return ch;
}

} // namespace transport::ch
