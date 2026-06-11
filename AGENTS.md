# AGENTS.md

Project-level instructions for LLM/code agents working in this repository.

This is the only canonical agent policy file for this repo. Do not create
`CLAUDE.md`, `.cursorrules`, `.github/copilot-instructions.md`, or similar
policy files unless the user explicitly asks for them.

Project engineering notes/backlog items belong in `NOTES.md`, not in this
behavior/policy file.

## First principles

- Read existing code before deciding on an implementation.
- Keep changes tightly scoped to the user request.
- Prefer explicit, local code over broad wrappers.
- Do not add compatibility layers unless explicitly requested.
- If user feedback reflects a durable project rule, update this file.

## Project structure

- C++23 code lives under `src/`.
- Algorithms live under `src/algorithms/`.
- Graph/storage code lives under `src/graph/`.
- Shared routing types/helpers live under `src/routing/`.
- CLI applications live under `src/apps/`.
- Build system is CMake and targets Linux first.
- Raw datasets live under `data/raw/`.
- Generated graph artifacts live under `data/graph/`.
- Benchmark results live under `results/`.
- Keep large generated assets out of git.

## C++ style

- Use C++23 with strict warnings.
- Favor simple POD-like structs. Use semantic type aliases (`VertexId`, `Weight`, `Distance` from `src/graph/types.hpp`) in all interfaces; raw `uint32_t`/`uint64_t` are reserved for counts, offsets, and other values that do not carry graph semantics.
- Keep functions short and focused.
- Avoid unnecessary inheritance and virtual dispatch.
- Prefer iterative algorithms and contiguous containers in hot paths.
- Reserve memory where sizes are predictable.
- Keep includes minimal and deterministic.
- Use `snake_case` for functions/variables and `PascalCase` for types.
- Always use braces `{}` for `if`, `for`, `while`, and `do` bodies, even single-statement ones.
- Prefer `std::string_view` over `const char *` for read-only string values.
- Prefer early-exit style: `if (cond) return;` / `if (cond) continue;` over nested if-else blocks.
- Run `clang-format` on modified C++ files before finalizing changes:
  - `rg --files src tests | rg '\\.(cpp|hpp|h)$' | xargs clang-format -i`

## Algorithm and performance rules

- Baseline shortest path implementation is classic Dijkstra.
- Optimization path is A* with an admissible geographic heuristic.
- Graph representation should be compact and cache-friendly (CSR-style).
- Validate correctness by comparing algorithm result distances.
- Benchmarks must use fixed seeds for reproducibility.
- Do not claim speedups without reporting exact benchmark settings.

## Data handling

- Prefer reproducible dated map extracts over mutable `latest` links.
- Keep metadata with source file size, vertex count, edge count, and import time.
- Do not mutate raw map input files in place.
- Fail fast on malformed graph or unsupported data.

## Validation

- Do not run final validation while implementation is incomplete.
- For completed changes, run:
  - `cmake -S . -B build -DCMAKE_BUILD_TYPE=Release`
  - `cmake --build build -j`
  - `ctest --test-dir build --output-on-failure`
- For performance checks, run benchmark binaries with explicit CLI flags.

## Benchmark results

Every benchmark run writes a JSON file to `results/`. Rules:

- **Never overwrite** an existing file. Encode enough context in the name to be unique:
  `<algorithm>_<variant-slug>_<graph>_<YYYYMMDDTHHMMSS>.json` (date + time to the second,
  e.g. `ch_lazy_europe_20260611T143052.json`).
  If a collision still occurs (two runs within the same second), append `_2`, `_3`, …
- **Required top-level fields** (present regardless of algorithm):
  - `"algorithm"` — short name: `"ch"`, `"dijkstra"`, `"astar"`, …
  - `"variant"` — human-readable description of the configuration/optimisation,
    e.g. `"lazy edge-difference ordering, witness hop_limit=5"`.
  - `"commit"` — git SHA of the code that produced the result.
  - `"date"` — ISO-8601 date of the run.
  - `"graph"` — object with `"path"` (binary graph file passed to the benchmark), `"source"` (original input file the graph was built from, e.g. the OSM PBF path), `"vertices"`, `"directed_edges"`.
- **Timing:** use `_wall_s` / `_cpu_s` pairs for every timed phase so wall vs CPU is always visible.
- **Memory:** include `"peak_rss_mb"`.
- **Query stats:** include `"mean_us"`, `"p50_us"`, `"p95_us"`, `"p99_us"`, `"max_us"`, `"count"`, `"seed"` under a `"queries"` object.
- Algorithm-specific fields (`"auxiliary_edges"`, `"witness_calls"`, `"ordering_init_wall_s"`, …) are welcome — add as needed.
- Use a `_note` suffix field to document caveats inline, e.g. `"query_alloc_note": "allocates O(V) vectors per call"`.
- Keep field names and structure human-readable; a reader skimming the raw JSON should understand each value without consulting code.
- **Before running a benchmark, check that working set fits in physical RAM** (`free -h`). A benchmark bottlenecked by RAM↔swap is not representative and should not be reported. Exception: algorithms whose design explicitly trades RAM for speed (e.g. arc flags with large label tables) may document this trade-off, but should still note the swap pressure in a `_note` field.

## Agent behavior

- Before editing files, state briefly what will be edited and why.
- If the user asks to only plan, do not perform implementation edits.
- If asked to implement, carry through code, build checks, and a concise result summary.
- If environment constraints block a requested step, report the exact blocker.
