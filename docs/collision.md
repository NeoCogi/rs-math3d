# Collision experiment

[`examples/collision/mod.rs`](../examples/collision/mod.rs) is an example-only
discrete narrow phase, generic over `RealScalar`. It reuses `Sphere3` and `Tri3`
and adds local `Obb` and `Capsule` types. It is a starting point for measuring a
small physics engine; it does not add a supported collision API to the crate.

The [static triangle-soup AABB tree](triangle-soup.md) adds accelerated nearest-ray,
capsule/OBB and mesh/mesh intersection queries using offline teapot/bunny fixtures.

All ten unordered shape pairings return a boolean. Boundary contact counts as
overlap in exact arithmetic. The [contact submodule](../examples/collision/contact.rs)
now adds witnesses, normals, penetration depths and bounded manifolds for the
[example rigid-body engine](physics.md). The original boolean entry points retain
their contracts. There is no continuous collision detection. Swapping arguments
to a mixed boolean pair means calling the same documented shape order.

| Pair | Algorithm |
| --- | --- |
| Sphere / sphere | Squared center distance |
| Sphere / capsule | Squared point-segment distance |
| Sphere / box | Clamp the box-local center to the box |
| Sphere / triangle | Scaled plane projection or closest edge |
| Capsule / capsule | Closest points on two closed segments |
| Capsule / box | Minimize piecewise quadratic segment-box distance |
| Capsule / triangle | Face piercing, endpoint-face and edge-edge distances |
| Box / box | 15 separating axes: face normals and edge crosses |
| Box / triangle | 13 separating axes: face normals and edge crosses |
| Triangle / triangle | Face normals, edge crosses and in-plane edge normals |

Queries perform no allocation, square root, trigonometry, normalization or
matrix inversion. Separating axes do not need unit length. Box orientation is
supplied by the caller. Capsule-box distance uses the rounded edges and corners
of the swept shape; simply expanding box slabs by a radius would introduce
false positives. Triangle-triangle includes the in-plane axes needed to reject
disjoint coplanar triangles.

All arithmetic uses the library's ordinary, unchecked operators and vector
kernels. This is safe Rust and introduces no `unsafe` or unchecked memory
access. The caller supplies:

- Finite coordinates, nonnegative radii and half extents.
- Orthonormal box axes, represented to the scalar's available precision.
- Nondegenerate triangles with resolvable region-test denominators.
- Representable intermediates, including squared distances and products of dot
  products, and enough fractional precision for projections and divisions.

Coincident capsule endpoints are supported as spheres. Exact parallel segment
directions are handled without dividing by a zero Gram determinant. Small
directions, almost parallel axes, or tiny triangles can quantize to degeneracy
in fixed point; these routines do not promise robust predicates for arbitrary
scales. Rounding can move a contact decision either way. No tolerance or hidden
collision margin is added. A bounded coordinate domain and explicit contact
margin would be necessary before turning this into a production physics API.
Choosing a format only by the range of the input coordinates is insufficient:
segment/segment distance still uses products of squared lengths. Larger worlds need local
coordinates, rescaling or wider storage.

Point/triangle distance now scales the face normal before plane projection and
uses closest edges outside the face. Capsule endpoint and piercing tests reuse
that normal. This supersedes the Gram-product Voronoi solve whose denominator
could round to zero on a valid one-cell Q8 triangle in S15.16. Geometry smaller
than the selected format can still collapse numerically; no global exactness
or one-quantum distance guarantee is implied.

Run the regressions with `cargo test --test collision_examples`, and repeat with
`--no-default-features` for the fixed-only path. They cover all ten pairings,
touches and misses, all point-triangle Voronoi regions, collapsed and parallel
capsule centerlines, triangle face piercing, box corner contacts, coplanar
triangles, reversed winding and rigid transforms. Independent f64 references
check segment-box distance by convex minimization and oblique box overlap by
edge clipping, including misses requiring edge-cross axes.

## Reproducing the benchmark

```sh
cargo run --release --example collision_bench -- 1000 7
```

Arguments are positive batch and sample counts (defaults: 200 and 5). Each
batch contains 128 deterministic pairs: 64 overlaps and 64 separated pairs.
Centers, box rotations and separation directions vary; capsule directions are
nonparallel and triangle pairs intersect in different planes. Hits have ample
overlap, and misses have ample separation. Boundary behavior belongs to the
regressions, rather than determining which backend does more work in the timer.
Every query checks its expected classification for all fixtures before timing.

The harness monomorphizes the same algorithms for `f32`, `f64`, `Fixed<i32, 8>`,
`Fixed<i32, 16>` and `Fixed<i64, 16>`. Input generation, conversion, box rotations,
validation, allocation and reporting are outside timing. Each operation gets a
warmup batch. Input references and outputs use `std::hint::black_box`; the measured
loop contains no trait objects or shape dispatch. Results report median, minimum
and maximum sample ns/query, including iteration and black-box overhead. These
are batch throughput measurements, not dependent instruction latencies. With an
even sample count the reported median is the upper middle sample. Fixture
storage grows with scalar width, and query timing includes fixture loads.

The scalar section separately measures add, multiply, divide, dot3, square root
and the common trait's sine-plus-cosine expression. A final section measures the
fused fixed `sin_cos` API. Compiler transformations, including possible fusion of
floating sine/cosine, remain enabled. Timings use bounded, positive scalar inputs;
they do not characterize arbitrary input magnitudes or branch distributions.

The example requires `std` for the timer and floating baselines. The collision
module itself has no `std` dependency; its fixed-only integration tests run
without a floating analytic backend. For bare-metal ARMv5/RISC-V, reuse the module
and replace the hosted timer and input setup with the target's counter/harness.
No ARM or RISC-V Rust target runtime was installed for this measurement.

## Host baseline, 2026-11-08

This initial primitive baseline predates the scaled-plane distance update.
The current capsule/OBB tree measurements are in the
[triangle-soup guide](triangle-soup.md); this historical table is retained with
its original raw output.

Intel Core Ultra 7 155H, x86_64 Linux, pinned to logical CPU 0; rustc 1.98.1
(`48a229cea`, LLVM 22.1.8), default release profile and default `std` feature.
Command: `taskset -c 0 cargo run --release --offline --example collision_bench -- 1000 7`.
No custom `RUSTFLAGS`, LTO or target CPU selection was used. Each sample measures
128,000 queries per row. The full ranges and scalar results are retained in the
[raw output](benchmarks/collision-x86_64.txt).

Median nanoseconds per overlap query:

| Pair | f32 | f64 | S23.8 / i32 | S15.16 / i32 | S47.16 / i64 |
| --- | ---: | ---: | ---: | ---: | ---: |
| Sphere / sphere | 1.26 | 1.17 | 2.40 | 2.40 | 3.75 |
| Sphere / capsule | 3.43 | 3.29 | 15.32 | 16.14 | 18.67 |
| Sphere / box | 3.11 | 2.93 | 9.26 | 9.30 | 12.26 |
| Sphere / triangle | 8.13 | 7.81 | 29.24 | 30.19 | 29.83 |
| Capsule / capsule | 8.69 | 8.33 | 34.35 | 36.04 | 37.37 |
| Capsule / box | 18.62 | 18.30 | 48.49 | 49.49 | 56.64 |
| Capsule / triangle | 38.09 | 36.49 | 119.48 | 122.95 | 123.81 |
| Box / box | 35.43 | 36.52 | 193.59 | 199.13 | 203.01 |
| Box / triangle | 38.15 | 36.93 | 154.76 | 156.19 | 147.42 |
| Triangle / triangle | 63.44 | 59.66 | 203.71 | 193.48 | 184.46 |

S15.16 is approximately 1.9–5.6 times the f32 query cost on this workload.
Fixed addition is similar to float addition in the isolated throughput test;
fixed multiplication, division, rounding and widened accumulation add cost.
The experiment does not isolate the contribution of each mechanism, so the
collision ratios cannot be attributed solely to `i128`.

Median fused fixed `sin_cos` is 254–255 ns across these formats. The common
sine-plus-cosine expression is approximately 503–509 ns for fixed versus 1.77 ns
for f32 and 5.10 ns for f64 on this bounded input distribution. The current
shared 97-step Q96 implementation explains why lower fractional precision does
not buy fewer CORDIC iterations. See [working width and embedded performance](fixed-point.md#working-width-and-embedded-performance).

This is a repeatable example workload, not an engine frame-time estimate or a
cross-target ranking. It excludes body updates, collision candidate generation,
contact generation, constraints and memory traffic from a large scene. CPU
frequency, scheduling, compiler, collision distribution and scene scale matter.
ARMv5 and RV32/RV64 may use different instruction sequences or software routines;
floating hardware and integer multiply/divide support must be evaluated on the
actual core before choosing a representation.
