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

Four query families are available:

- `raycast(&ray)` returns the nearest two-sided forward-ray hit, its parameter,
  represented point and input triangle ID. Exact parameter ties choose the
  smallest ID. The original stored direction is retained, including non-unit
  directions. Coplanar/represented-zero-determinant rays return no triangle hit.
- `intersects(&other, offset)` returns any surface-intersecting input ID pair.
  The other soup is translated into this soup's coordinate frame. Rotations
  require building from transformed triangles; there is no refit/update path.
  Enclosed disjoint surfaces are not considered intersecting solids.
- `intersects_capsule(&capsule)` returns any triangle touching the capsule volume.
  A zero-length centerline is a sphere; zero radius tests a segment. Endpoints
  are in the soup's frame and radius must be nonnegative.
- `intersects_obb(&obb)` returns any triangle intersecting the oriented-box
  volume. Supply finite approximately orthonormal axes and nonnegative half
  extents in the soup's frame. Zero extents are supported within quantization
  limits. A box can contain an entire triangle and still report contact.

Capsule and OBB methods share one statically specialized shape traversal. Their
envelope is prepared once per query, then closed AABB overlap rejects nodes and
leaf triangles before the existing squared-distance or SAT narrow phase. They
return `QueryResult<usize>` containing any original triangle ID. Their
`brute_intersects_capsule` / `brute_intersects_obb` references use the same soup
and leaf filters in input order; witness IDs can differ. No query allocates.
Shapes fully inside a closed soup without touching a triangle return no hit.

Capsule bounds expand endpoint minima/maxima by radius. OBB bounds evaluate both
signed ends of each half axis and accumulate their minima/maxima in represented
corner-construction order. Native floor products are not odd, so a negative
corner cannot be recovered by mirroring a rounded positive half axis. This also
avoids excluding a rounded corner when floating addition of the center and a
pre-summed extent associates differently. Capsule closest-point arithmetic uses
a max-component-scaled face normal, an interior plane projection or closest
edge, and a shared prepared normal for endpoint/piercing tests. Its nonzero face
denominator stays at least one: valid small Q8 faces no longer divide by a
quartic Gram denominator that rounds to zero in S15.16. A represented collapsed
normal falls back to edges. These paths still use no square root or CORDIC.

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
`cargo run --release --manifest-path tools/Cargo.toml --bin prepare-collision-meshes -- --download` reproduces the fixtures from
pinned sources; local source paths can also be supplied. Its integer cross-product
filter removes collapsed faces once for every backend. Rust's checked decoder
validates the packet version, exact byte counts and vertex indices before use.
Ordinary tests/benchmarks need no network access or external mesh files.

Run the tests with `cargo test --test collision_examples aabb_tree`; repeat with
`--no-default-features` for both fixed backends. The real models exercise nearest
ray/tree versus flat traversal, translated teapot/bunny overlap, input ID witnesses
and separated bounds. Tests also check balanced topology and parent containment,
empty input, duplicate centers, invalid leaf sizes, malformed packets, nonfinite
geometry and fixed overflow at the construction boundary.

Capsule/OBB coverage additionally includes face/vertex touches, centerline face
piercing, degenerate centerlines, tilted boxes, zero-sized boxes, envelope-only
false positives, contained triangles, and shapes enclosed without mesh contact.
Regressions pin the small-triangle capsule division failure and the rounded
OBB corner. Both real models use 264 poses per shape across f32 and both fixed
formats, checking hierarchy classification and returned triangle witnesses.

## Benchmark method

```sh
cargo run --release --example mesh_bench -- 200 7
```

Arguments are positive tree batch/sample counts (defaults: 20/5). The standalone
hosted harness measures `f32`, `Fixed<i32,16>` (S15.16) and `Fixed<i64,32>`
(S31.32). Setup and mesh decoding are excluded. Build measurements include
validation, partitioning and allocation; input cloning and completed-tree
destruction occur outside the build timer. Every tree uses four-triangle leaves.

Each model receives 448 exact dyadic rays: six 8x8 orthographic grids plus an
8x8 oblique grid. Origins, directions and stored ray parameterizations are the
same in every backend. The teapot has 71 hits and the bunny 128 on this corpus.
Twelve relative translations test teapot/bunny surface overlap: eight hit, three
miss the whole-soup bounds, and one has overlapping soup bounds but no surface
intersection. This is a static translation workload, without rotation/refits.

Every query is compared against flat traversal before timing, warming both
paths. Rays require identical hit ID and represented parameter within a backend;
mesh pairs require the same hit/miss answer and can return different witnesses.
The report separately counts hit/ID differences and parameter differences from
f32. Both traversals have the same whole-soup prefilter and leaf predicates.
Query timing includes work counters, iteration, result copying and `black_box`
overhead; it measures warm batch throughput, not isolated cold-query latency.

Accelerated queries use the requested batches/samples and report median/min/max
ns/query. Flat rays use one batch per sample. Flat mesh/mesh uses one batch and
one sample, because a miss with overlapping soup bounds can enumerate all
24,338,320 triangle pairs. Its printed min/median/max are therefore identical
and provide a reference cost, not a statistical estimate. Fixed-point and f32
classification comparisons are also sample observations, not exact predicates
or a global precision guarantee.

## Initial ray/mesh results, 2026-11-08

Intel Core Ultra 7 155H, x86_64 Linux, pinned logical CPU 0; rustc 1.98.1
(`48a229cea`, LLVM 22.1.8), default release profile and `std` feature. No custom
build flags, LTO or CPU selection were supplied. Command:
`taskset -c 0 cargo run --release --offline --example mesh_bench -- 200 7`.
The [raw output](benchmarks/triangle-soup-x86_64.txt) retains ranges, construction
times, storage and traversal counters. ARMv5/RISC-V timings were not measured.

Accelerated median query times:

| Query | f32 | S15.16 / i32 | S31.32 / i64 |
| --- | ---: | ---: | ---: |
| Teapot nearest ray | 99.07 ns | 190.93 ns | 233.16 ns |
| Bunny nearest ray | 204.72 ns | 336.31 ns | 407.79 ns |
| Teapot/bunny surface pair | 1.810 us | 2.777 us | 2.787 us |

The tree improves f32 ray throughput by approximately 42–46x versus the flat
reference; S15.16 improves by 31–38x. Flat ray medians are 4.55/8.53 us for f32
teapot/bunny, and 7.35/10.59 us for S15.16. Tree work averages 17.4/28.5 bounds
tests per ray versus 1,496.4/1,806.2 for flat traversal after the common soup
prefilter. Narrow-phase work is the same, averaging 1.19/1.72 triangle tests.

For mesh/mesh, average bounds work falls from approximately 4.43 million tests
per pose to 380.5. The tree averages 27.5 triangle tests versus 10.2 for flat
traversal, because the traversal orders find different witnesses. Flat costs
are approximately 10.38 ms/pose for f32 and 15.86 ms for S15.16 in the single
reference sample. Those large differences depend on the mixed hit/miss corpus,
input ordering and early exits; they are not a general mesh-collision speedup.

Tree construction medians are 1.379/0.630 ms for f32 teapot/bunny and 1.024/0.533
ms for S15.16. Their tree layouts match: 4,095/2,047 nodes, depths 12/11 and
659,136/376,556 reserved bytes. S31.32 uses 1,136,640/656,768 bytes. These byte
counts exclude the benchmark's separate input/reference buffers and allocator
overhead; they describe the tree's reserved owned buffers on this host.

All three backends have zero acceleration mismatches. Their 896 ray hit/miss
answers and hit IDs match f32, and all twelve mesh-pair classifications agree.
S15.16's maximum parameter difference from f32 is 0.001680374 for the teapot and
0.000310898 for the bunny. S31.32 differences are approximately 4.36e-6/3.14e-6.
These are comparisons against f32, which also rounds; they are not absolute
errors against an exact reference. Unscaled determinants can amplify fixed
quantization beyond one scalar quantum, especially for small/near-parallel faces.

## Capsule and OBB benchmark extension

The same `mesh_bench` command now also measures shape/soup any-hit queries.
[`shape_cases.rs`](../examples/collision/shape_cases.rs) prepares a shared corpus:
64 evenly sampled input faces, four normal offsets per face (-1.5, -0.25, 0.25,
1.5), and eight remote misses, giving **264 poses per shape per model**. Capsule
centerlines follow tilted face tangents and alternate radii 0.375/0.5. OBBs rotate
around two axes, with half extents (1, 0.5, 0.25). Centers/endpoints use Q8 and
OBB axes Q16, exactly shared by f32, S15.16 and S31.32. Quantized axes approximate
orthonormal frames; no backend silently receives higher-resolution query inputs.

Setup normals, trigonometry, quantization and allocation occur outside timers.
The measured query includes preparation of its envelope, node/leaf filtering,
narrow phase, counters and result handling. Flat shape queries use one batch per
sample and the same soup/leaf bounds as tree traversal. Both traversals' returned
IDs are checked against their narrow-phase predicate. Witness IDs need not match;
the acceleration oracle compares any-hit classification. f32 classifications
are also compared separately across scalar backends.

The pinned host/compiler/profile and `200 7` command are the same as above.
Each accelerated sample contains 52,800 queries per shape/model; seven samples
produce the following median ns/query. The extension's [full output](benchmarks/triangle-soup-shapes-x86_64.txt)
retains ranges, flat timings, counters and the existing ray/mesh results.

| Query | f32 | S15.16 / i32 | S31.32 / i64 |
| --- | ---: | ---: | ---: |
| Capsule / teapot | 527.81 | 2458.49 | 2648.67 |
| Capsule / bunny | 365.86 | 1762.80 | 1826.52 |
| OBB / teapot | 248.86 | 1367.95 | 825.52 |
| OBB / bunny | 164.34 | 745.64 | 447.61 |

The tree improves f32 capsule throughput by approximately 5.6–13.3x over flat
traversal and OBB throughput by 11.3–33.4x. S15.16 improvements are 1.8–3.5x and
2.4–7.1x respectively. These mixed, mostly near-surface workloads exercise more
narrow phase than the earlier ray corpus; ratios do not generalize to arbitrary
scene distributions or other targets. Wider fixed storage is not consistently
slower on this host, as the S31.32 OBB rows demonstrate.

All 3,168 shape/model/backend queries in the untimed verification passes agree
with flat traversal, and every sampled hit classification matches f32. Capsule
hits are 164/264 on teapot and 138/264 on bunny; OBB hits are 166/264 and 141/264.
Average bound comparisons fall from 3,782.1/2,284.6 to 33.7/33.1 for capsules and
from 3,739.0/2,307.7 to 39.6/35.2 for OBBs. Tree narrow-phase counts average
3.6/2.4 triangles for capsules and 5.9/3.1 for OBBs. Different traversal orders
can find different witnesses and can increase narrow-phase work while reducing
the total bounds work substantially.

This extension uses the corrected scaled-plane capsule distance kernel and the
rounded-corner OBB envelope described above. The sample agreement does not prove
exact geometric predicates or a global distance bound, and ARMv5/RISC-V timings
remain unmeasured. These are discrete volume/triangle tests without penetration
depths, contact manifolds or continuous collision detection.
