#include "graph/graph.hpp"

#include <algorithm>
#include <chrono>
#include <cmath>
#include <cstdint>
#include <filesystem>
#include <fstream>
#include <iostream>
#include <limits>
#include <optional>
#include <queue>
#include <stdexcept>
#include <string>
#include <string_view>
#include <vector>

#include <osmium/handler.hpp>
#include <osmium/io/any_input.hpp>
#include <osmium/io/reader.hpp>
#include <osmium/tags/filter.hpp>
#include <osmium/visitor.hpp>

namespace fs = std::filesystem;
using transport::Edge;
using transport::Graph;
using transport::NodeCoord;

namespace {

struct RawSegment {
    uint64_t from_osm_id = 0;
    uint64_t to_osm_id = 0;
};

struct MappingRecord {
    uint64_t osm_id = 0;
    uint32_t graph_id = 0;
};

template <size_t N> std::optional<std::string_view> tag_value(const osmium::TagList &tags, const char (&key)[N]) {
    const auto value = tags.get_value_by_key(key);
    if (value == nullptr) {
        return std::nullopt;
    }
    return std::string_view{value};
}

bool is_driving_highway(std::string_view value) {
    static constexpr std::string_view allowed[] = {"motorway",      "trunk",        "primary",        "secondary",
                                                   "tertiary",      "unclassified", "residential",    "motorway_link",
                                                   "trunk_link",    "primary_link", "secondary_link", "tertiary_link",
                                                   "living_street", "service"};
    return std::find(std::begin(allowed), std::end(allowed), value) != std::end(allowed);
}

bool is_driving_highway(std::optional<std::string_view> value) {
    if (!value.has_value()) {
        return false;
    }
    return is_driving_highway(*value);
}

bool is_blocked_for_car(const osmium::TagList &tags) {
    const std::optional<std::string_view> access = tag_value(tags, "access");
    if (access.has_value() && (*access == "no" || *access == "private")) {
        return true;
    }
    const std::optional<std::string_view> motor_vehicle = tag_value(tags, "motor_vehicle");
    if (motor_vehicle.has_value() && (*motor_vehicle == "no" || *motor_vehicle == "private")) {
        return true;
    }
    return false;
}

bool is_oneway_forward(const osmium::TagList &tags) {
    const std::optional<std::string_view> oneway = tag_value(tags, "oneway");
    if (!oneway.has_value()) {
        const std::optional<std::string_view> junction = tag_value(tags, "junction");
        return junction.has_value() && *junction == "roundabout";
    }
    return *oneway == "yes" || *oneway == "1" || *oneway == "true";
}

bool is_oneway_reverse(const osmium::TagList &tags) {
    const std::optional<std::string_view> oneway = tag_value(tags, "oneway");
    if (!oneway.has_value()) {
        return false;
    }
    return *oneway == "-1";
}

uint64_t positive_osm_id(osmium::object_id_type id) {
    if (id <= 0) {
        throw std::runtime_error("unsupported non-positive OSM node id");
    }
    return static_cast<uint64_t>(id);
}

template <typename T> void write_record(std::ofstream &out, const T &record) {
    out.write(reinterpret_cast<const char *>(&record), static_cast<std::streamsize>(sizeof(T)));
    if (!out) {
        throw std::runtime_error("failed to write temp file");
    }
}

template <typename T> bool read_record(std::ifstream &in, T &record) {
    in.read(reinterpret_cast<char *>(&record), static_cast<std::streamsize>(sizeof(T)));
    if (in.gcount() == 0 && in.eof()) {
        return false;
    }
    if (in.gcount() != static_cast<std::streamsize>(sizeof(T))) {
        throw std::runtime_error("corrupted temp file");
    }
    return true;
}

class TempFileWriter {
public:
    explicit TempFileWriter(const fs::path &path) : out_(path, std::ios::binary) {
        if (!out_) {
            throw std::runtime_error("failed to open temp file for writing: " + path.string());
        }
    }

    template <typename T> void write(const T &record) { write_record(out_, record); }

    void close() { out_.close(); }

private:
    std::ofstream out_;
};

struct WayCollector : public osmium::handler::Handler {
    TempFileWriter node_writer;
    TempFileWriter segment_writer;

    WayCollector(const fs::path &nodes_path, const fs::path &segments_path)
        : node_writer(nodes_path), segment_writer(segments_path) {}

    void write_node(uint64_t osm_id) { node_writer.write(osm_id); }

    void write_segment(uint64_t from, uint64_t to) {
        segment_writer.write(RawSegment{.from_osm_id = from, .to_osm_id = to});
    }

    void close() {
        node_writer.close();
        segment_writer.close();
    }

    void way(const osmium::Way &way) {
        const std::optional<std::string_view> highway = tag_value(way.tags(), "highway");
        if (!is_driving_highway(highway) || is_blocked_for_car(way.tags())) {
            return;
        }

        const bool forward = !is_oneway_reverse(way.tags());
        const bool backward = !is_oneway_forward(way.tags());
        if (!forward && !backward) {
            return;
        }

        const auto &nodes = way.nodes();
        if (nodes.size() < 2) {
            return;
        }

        for (size_t i = 1; i < nodes.size(); ++i) {
            const uint64_t aid = positive_osm_id(nodes[i - 1].ref());
            const uint64_t bid = positive_osm_id(nodes[i].ref());
            write_node(aid);
            write_node(bid);
            if (forward) {
                write_segment(aid, bid);
            }
            if (backward) {
                write_segment(bid, aid);
            }
        }
    }
};

void sort_and_unique_nodes(std::vector<uint64_t> &node_ids) {
    std::sort(node_ids.begin(), node_ids.end());
    node_ids.erase(std::unique(node_ids.begin(), node_ids.end()), node_ids.end());
}

std::vector<fs::path> sort_node_chunks(const fs::path &raw_nodes_path, const fs::path &temp_dir) {
    constexpr size_t kNodesPerChunk = 4'000'000;
    std::ifstream in(raw_nodes_path, std::ios::binary);
    if (!in) {
        throw std::runtime_error("failed to open node temp file");
    }

    std::vector<fs::path> chunks;
    std::vector<uint64_t> node_ids;
    node_ids.reserve(kNodesPerChunk);
    uint64_t chunk_index = 0;
    while (true) {
        node_ids.clear();
        uint64_t node_id = 0;
        while (node_ids.size() < kNodesPerChunk && read_record(in, node_id)) {
            node_ids.push_back(node_id);
        }
        if (node_ids.empty()) {
            break;
        }

        sort_and_unique_nodes(node_ids);
        fs::path chunk_path = temp_dir / ("nodes.sorted." + std::to_string(chunk_index++) + ".bin");
        std::ofstream out(chunk_path, std::ios::binary);
        if (!out) {
            throw std::runtime_error("failed to write sorted node chunk");
        }
        out.write(reinterpret_cast<const char *>(node_ids.data()),
                  static_cast<std::streamsize>(node_ids.size() * sizeof(uint64_t)));
        chunks.push_back(chunk_path);
    }
    return chunks;
}

struct NodeHeapEntry {
    uint64_t osm_id = 0;
    size_t stream = 0;
};

struct NodeHeapCompare {
    bool operator()(const NodeHeapEntry &a, const NodeHeapEntry &b) const { return a.osm_id > b.osm_id; }
};

std::vector<MappingRecord> merge_nodes_write_mapping(const std::vector<fs::path> &chunks, const fs::path &mapping_path,
                                                     Graph &graph) {
    std::vector<std::ifstream> inputs;
    inputs.reserve(chunks.size());
    for (const fs::path &chunk : chunks) {
        inputs.emplace_back(chunk, std::ios::binary);
        if (!inputs.back()) {
            throw std::runtime_error("failed to read sorted node chunk");
        }
    }

    std::priority_queue<NodeHeapEntry, std::vector<NodeHeapEntry>, NodeHeapCompare> heap;
    for (size_t i = 0; i < inputs.size(); ++i) {
        uint64_t osm_id = 0;
        if (read_record(inputs[i], osm_id)) {
            heap.push(NodeHeapEntry{.osm_id = osm_id, .stream = i});
        }
    }

    std::ofstream mapping_out(mapping_path, std::ios::binary);
    if (!mapping_out) {
        throw std::runtime_error("failed to write mapping file");
    }

    std::vector<MappingRecord> mapping;
    while (!heap.empty()) {
        const NodeHeapEntry entry = heap.top();
        heap.pop();

        if (mapping.empty() || mapping.back().osm_id != entry.osm_id) {
            const auto graph_id = static_cast<uint32_t>(graph.coords.size());
            graph.coords.push_back(NodeCoord{});
            const MappingRecord mapping_record{.osm_id = entry.osm_id, .graph_id = graph_id};
            mapping.push_back(mapping_record);
            write_record(mapping_out, mapping_record);
        }

        uint64_t next = 0;
        if (read_record(inputs[entry.stream], next)) {
            heap.push(NodeHeapEntry{.osm_id = next, .stream = entry.stream});
        }
    }
    return mapping;
}

uint32_t lookup_graph_id(const std::vector<MappingRecord> &mapping, uint64_t osm_id) {
    const auto it = std::lower_bound(mapping.begin(), mapping.end(), osm_id,
                                     [](const MappingRecord &record, uint64_t id) { return record.osm_id < id; });
    if (it == mapping.end() || it->osm_id != osm_id) {
        throw std::runtime_error("internal error: missing node mapping");
    }
    return it->graph_id;
}

class CoordinateCollector : public osmium::handler::Handler {
public:
    CoordinateCollector(const std::vector<MappingRecord> &mapping, Graph &graph, std::vector<uint8_t> &has_coord)
        : mapping_(mapping), graph_(graph), has_coord_(has_coord) {}

    void node(const osmium::Node &node) {
        const uint64_t osm_id = positive_osm_id(node.id());
        const MappingRecord *mapping = find_mapping(osm_id);
        if (mapping == nullptr) {
            return;
        }

        graph_.coords[mapping->graph_id] =
            NodeCoord{.lat = node.location().lat_without_check(), .lon = node.location().lon_without_check()};
        has_coord_[mapping->graph_id] = 1;
    }

private:
    // Assumes nodes are delivered in ascending osm_id order (true for standard OSM PBF
    // files), so a single forward cursor over the sorted mapping suffices.
    const MappingRecord *find_mapping(uint64_t osm_id) {
        while (next_index_ < mapping_.size() && mapping_[next_index_].osm_id < osm_id) {
            ++next_index_;
        }
        if (next_index_ < mapping_.size() && mapping_[next_index_].osm_id == osm_id) {
            return &mapping_[next_index_++];
        }
        return nullptr;
    }

    const std::vector<MappingRecord> &mapping_;
    Graph &graph_;
    std::vector<uint8_t> &has_coord_;
    size_t next_index_ = 0;
};

std::optional<Edge> make_edge(uint32_t to, const NodeCoord &from_coord, const NodeCoord &to_coord) {
    const double meters = transport::haversine_meters(from_coord, to_coord);
    const double scaled = meters * static_cast<double>(transport::kDistanceScale);
    if (scaled > static_cast<double>(std::numeric_limits<uint32_t>::max())) {
        return std::nullopt;
    }
    return Edge{.to = to, .weight_units = static_cast<uint32_t>(std::ceil(scaled))};
}

void build_csr_edges(const fs::path &raw_segments_path, const std::vector<MappingRecord> &mapping,
                     const std::vector<uint8_t> &has_coord, Graph &graph) {
    graph.offsets.assign(static_cast<size_t>(graph.vertex_count()) + 1, 0);
    uint64_t directed_edges = 0;

    {
        std::ifstream in(raw_segments_path, std::ios::binary);
        if (!in) {
            throw std::runtime_error("failed to read raw segment temp file");
        }
        RawSegment segment;
        while (read_record(in, segment)) {
            const uint32_t from = lookup_graph_id(mapping, segment.from_osm_id);
            const uint32_t to = lookup_graph_id(mapping, segment.to_osm_id);
            if (has_coord[from] == 0 || has_coord[to] == 0) {
                continue;
            }
            if (!make_edge(to, graph.coords[from], graph.coords[to]).has_value()) {
                continue;
            }
            graph.offsets[static_cast<size_t>(from) + 1] += 1;
            ++directed_edges;
        }
    }

    for (size_t i = 1; i < graph.offsets.size(); ++i) {
        graph.offsets[i] += graph.offsets[i - 1];
    }

    graph.edges.resize(static_cast<size_t>(directed_edges));
    std::vector<uint64_t> cursor = graph.offsets;

    {
        std::ifstream in(raw_segments_path, std::ios::binary);
        if (!in) {
            throw std::runtime_error("failed to read raw segment temp file");
        }
        RawSegment segment;
        while (read_record(in, segment)) {
            const uint32_t from = lookup_graph_id(mapping, segment.from_osm_id);
            const uint32_t to = lookup_graph_id(mapping, segment.to_osm_id);
            if (has_coord[from] == 0 || has_coord[to] == 0) {
                continue;
            }
            const std::optional<Edge> edge = make_edge(to, graph.coords[from], graph.coords[to]);
            if (!edge.has_value()) {
                continue;
            }
            const uint64_t pos = cursor[from]++;
            graph.edges[static_cast<size_t>(pos)] = *edge;
        }
    }
}

void write_stats_json(const std::string &path, const std::string &input_file, uintmax_t input_size, const Graph &graph,
                      double import_seconds) {
    std::ofstream out(path);
    if (!out) {
        throw std::runtime_error("failed to write stats file");
    }
    out << "{\n";
    out << "  \"input_file\": \"" << input_file << "\",\n";
    out << "  \"input_size_bytes\": " << input_size << ",\n";
    out << "  \"vertices\": " << graph.vertex_count() << ",\n";
    out << "  \"directed_edges\": " << graph.edge_count() << ",\n";
    out << "  \"import_seconds\": " << import_seconds << "\n";
    out << "}\n";
}

} // namespace

int main(int argc, char **argv) {
    std::string input;
    std::string output;
    std::string stats;
    for (int i = 1; i + 1 < argc; ++i) {
        const std::string key = argv[i];
        const std::string value = argv[i + 1];
        if (key == "--input") {
            input = value;
            ++i;
        } else if (key == "--output") {
            output = value;
            ++i;
        } else if (key == "--stats") {
            stats = value;
            ++i;
        }
    }

    if (input.empty() || output.empty() || stats.empty()) {
        std::cerr << "usage: transport_import_osm --input <europe.osm.pbf> --output <graph.bin> --stats <stats.json>\n";
        return 1;
    }

    const auto t0 = std::chrono::steady_clock::now();
    const fs::path output_path(output);
    fs::create_directories(output_path.parent_path());

    const auto stamp = std::chrono::steady_clock::now().time_since_epoch().count();
    const fs::path temp_dir =
        output_path.parent_path() / (output_path.filename().string() + ".tmp." + std::to_string(stamp));
    fs::create_directories(temp_dir);
    const fs::path raw_nodes_path = temp_dir / "nodes.raw.bin";
    const fs::path raw_segments_path = temp_dir / "segments.raw.bin";
    const fs::path mapping_path = temp_dir / "mapping.bin";

    WayCollector collector(raw_nodes_path, raw_segments_path);
    {
        osmium::io::File infile(input);
        osmium::io::Reader reader(infile);
        osmium::apply(reader, collector);
        reader.close();
    }
    collector.close();

    Graph graph;
    std::vector<fs::path> chunks = sort_node_chunks(raw_nodes_path, temp_dir);
    std::vector<MappingRecord> mapping = merge_nodes_write_mapping(chunks, mapping_path, graph);
    std::vector<uint8_t> has_coord(graph.vertex_count(), 0);
    {
        CoordinateCollector coordinate_collector(mapping, graph, has_coord);
        osmium::io::File infile(input);
        osmium::io::Reader reader(infile);
        osmium::apply(reader, coordinate_collector);
        reader.close();
    }
    build_csr_edges(raw_segments_path, mapping, has_coord, graph);

    if (!transport::save_graph_binary(graph, output)) {
        std::cerr << "failed to save graph\n";
        return 1;
    }

    const uintmax_t input_size = fs::file_size(input);
    const auto t1 = std::chrono::steady_clock::now();
    const double seconds = std::chrono::duration<double>(t1 - t0).count();
    write_stats_json(stats, input, input_size, graph, seconds);
    fs::remove_all(temp_dir);

    std::cout << "map_size_bytes=" << input_size << "\n";
    std::cout << "vertices=" << graph.vertex_count() << "\n";
    std::cout << "directed_edges=" << graph.edge_count() << "\n";
}
