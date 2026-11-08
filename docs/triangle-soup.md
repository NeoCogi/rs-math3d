# Static triangle-soup AABB tree

[`examples/collision/aabb_tree.rs`](../examples/collision/aabb_tree.rs) adds an
example-only static hierarchy over owned `Tri3<T>` data. It supports `f32` and
fixed `RealScalar` formats without adding an allocation dependency or a new
public API to the library. The hosted builder allocates; queries use only bounded
recursion and allocate no heap scratch memory.

`AabbTree::new(triangles, leaf_size)` consumes input geometry, validates it and
builds immutable nodes. Input triangle IDs remain stable, including duplicates.
Leaves reference ranges in an ID permutation rather than moving the triangles.
Longest-extent median splits, with original-ID tie-breaking, produce deterministic
balanced topology even for coincident centers. `Kind::Leaf` and `Kind::Branch`
own the mutually exclusive leaf/child state. Children exist before a temporary
leaf commits to a branch, and the completed tree exposes no mutable geometry
or topology that could invalidate cached bounds.

Construction returns `BuildError` for zero leaf size, nonfinite coordinates,
degenerate represented triangles, unrepresentable edge/cross arithmetic or a
full coordinate span that cannot fit the scalar. Empty input is valid. Allocation
failure follows standard `Vec` behavior. Bounds and partition centers use the
represented coordinates, with no hidden expansion or float-only builder path.

Two query families are available:

- `raycast(&ray)` returns the nearest two-sided forward-ray hit, its parameter,
  represented point and input triangle ID. Exact parameter ties choose the
  smallest ID. The original stored direction is retained, including non-unit
  directions. Coplanar/represented-zero-determinant rays return no triangle hit.
- `intersects(&other, offset)` returns any surface-intersecting input ID pair.
  The other soup is translated into this soup's coordinate frame. Rotations
  require building from transformed triangles; there is no refit/update path.
  Enclosed disjoint surfaces are not considered intersecting solids.

`QueryResult` includes bounding and narrow-phase work counts. Timings include
these counters. `info()` reports triangle/node counts, depth and reserved heap
bytes, excluding allocator overhead. `triangle(id)` retrieves immutable geometry
for inspecting a witness.

Ray traversal tests all overlapping nodes without nearest-distance pruning:
rounded slab entry parameters cannot eliminate a closer rounded triangle hit.
The leaf solve uses [Möller–Trumbore's determinant formulation](https://my.eng.utah.edu/~cs6965/papers/MT97.pdf),
rejecting barycentric signs/ranges before its one parameter division. It uses no
normalization, square roots, trigonometry or angular epsilon. Mesh-pair traversal
splits the larger non-leaf subtree and terminates at the first witness. Its leaf
solve reuses the existing triangle/triangle separating-axis kernel, including
coplanar separating directions.

Queries use ordinary unchecked arithmetic: callers provide finite nonzero ray
directions, valid offsets and representable intermediate arithmetic. Fixed
rounding can erase small determinants or separating axes and can perturb hit
parameters. No exact-predicate or uniform error bound is claimed. The mandatory
leaf AABB filter is part of both accelerated and flat query predicates; it also
rejects disjoint bounds when a tiny fixed separating axis rounds to zero.
Internal bounds contain these same represented leaf bounds. Traversal preserves
the flat predicate's decisions rather than silently introducing a new tolerance.

`brute_raycast` and `brute_intersects` independently traverse the input-order
entries using those same leaf predicates and the same whole-soup AABB prefilter.
They do not traverse internal nodes or the leaf permutation. They check internal
hierarchy traversal, not exact geometric arithmetic; structural tests separately
check global/parent bounds and leaf coverage. Analytic fixtures check known
touches, misses, forward/backface and coplanar rays, direction scaling, nearest
ordering, stable duplicate IDs, and surface-only containment behavior.

## Offline model fixtures

The checked-in assets contain the Utah Teapot (6,320 triangles) and the low
resolution Stanford Bunny reconstruction (3,851 triangles). Each is centered,
uniformly scaled to a longest extent of 32 and snapped to a 1/256 grid. This is
exactly representable in f32, S15.16 and S31.32, so backend comparisons do not
start from different coordinate quantization. Triangle order/winding is retained;
no triangles collapsed during preparation. These are modified, normalized
fixtures, not original-unit or full-resolution model benchmarks.

Source attribution and data-use terms are in
[`assets/NOTICE.md`](../examples/collision/assets/NOTICE.md); source/fixture
hashes, counts and transformations are in
[`assets/manifest.json`](../examples/collision/assets/manifest.json).
`tools/prepare_collision_meshes.py --download` reproduces the fixtures from
pinned sources; local source paths can also be supplied. Its integer cross-product
filter removes collapsed faces once for every backend. Rust's checked decoder
validates the packet version, exact byte counts and vertex indices before use.
Ordinary tests/benchmarks need no network access, Python or external mesh files.

Run the tests with `cargo test --test collision_examples aabb_tree`; repeat with
`--no-default-features` for both fixed backends. The real models exercise nearest
ray/tree versus flat traversal, translated teapot/bunny overlap, input ID witnesses
and separated bounds. Tests also check balanced topology and parent containment,
empty input, duplicate centers, invalid leaf sizes, malformed packets, nonfinite
geometry and fixed overflow at the construction boundary.
