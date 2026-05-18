# NOTES

Engineering notes and backlog items that are not agent behavior/policy.

## Performance backlog

- Dijkstra/A* query initialization:
  - Current implementation initializes full distance arrays per query.
  - For large repeated-query workloads (for example Europe-scale), switch to
    reusable distance buffers with timestamp/epoch marking to avoid `O(V)`
    memory writes on every query.
