# Routing: Dijkstra, A*, CH

This project builds a directed road graph from OpenStreetMap data and compares:
- Dijkstra,
- A* (Haversine heuristic),
- Contraction Hierarchies (CH).

## Build

```bash
cmake -S . -B build -DCMAKE_BUILD_TYPE=Release
cmake --build build -j
```

## Download Europe map

```bash
./scripts/download_europe_map.sh
```

Default target:
- `data/raw/europe-260513.osm.pbf`

## Import road graph

```bash
./build/transport_import_osm \
  --input data/raw/europe-260513.osm.pbf \
  --output data/graph/europe-driving.graph \
  --stats data/graph/europe-driving.stats.json
```

Importer prints:
- map file size in bytes,
- number of vertices,
- number of directed edges.

## Single query

```bash
./build/transport_query \
  --graph data/graph/europe-driving.graph \
  --source 100 \
  --target 500000 \
  --algorithm dijkstra
```

`--algorithm` supports: `dijkstra`, `astar`, `ch`.

Output fields:
- `distance_units` (fixed-point integer units),
- `distance_scale` (units per meter),
- `distance_m` (derived human-readable meters),
- `settled`.

## Benchmark

```bash
./build/transport_benchmark \
  --graph data/graph/europe-driving.graph \
  --algorithm-a dijkstra \
  --algorithm-b astar \
  --queries 100 \
  --min-settled 100000 \
  --max-settled 1000000 \
  --seed 1 \
  --out reports/benchmarks/europe_local.csv
```

The benchmark:
- picks random `(source, target)` pairs,
- compares any pair from `dijkstra`, `astar`, and `ch`,
- keeps only pairs where algorithm A settled nodes are in `[min_settled, max_settled]`,
- verifies both selected algorithms return the same distance,
- measures preprocessing and query time externally.

CSV distance columns are integer fixed-point units:
- `distance_scale`,
- `algorithm_a_units`,
- `algorithm_b_units`.

## Source layout

- `src/algorithms/` — Dijkstra, A*, CH, algorithm interface/factory
- `src/graph/` — graph format, load/save, distance geometry
- `src/routing/` — shared routing types/helpers
- `src/apps/` — CLI entrypoints (`import`, `query`, `benchmark`)

## Notes

- Graph is directed and respects OSM one-way tags.
- Edge weights use scaled integer distance units (`kDistanceScale`).
- Supported roads focus on driving highways.
