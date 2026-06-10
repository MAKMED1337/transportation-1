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
- Benchmark reports live under `reports/benchmarks/`.
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
- Prefer `std::string_view` over `const char *` for read-only string values.
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

## Agent behavior

- Before editing files, state briefly what will be edited and why.
- If the user asks to only plan, do not perform implementation edits.
- If asked to implement, carry through code, build checks, and a concise result summary.
- If environment constraints block a requested step, report the exact blocker.
