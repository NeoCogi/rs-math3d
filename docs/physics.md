# Example rigid-body physics

[`examples/physics`](../examples/physics/mod.rs) is a small Newton-inspired
rigid-body example built on the [collision module](collision.md). The simulation
is independent of windowing/rendering and runs in f32, native S15.16/i32 or
native S31.32/i64. All simulation arithmetic uses the selected scalar; scene
setup and rendering conversions are separate.

Run the shared teapot/bunny scene without a display:

```sh
cargo run --release --example physics_headless -- f32 1200
cargo run --release --example physics_headless -- fixed32 1200
cargo run --release --example physics_headless -- fixed64 1200
```

The scene contains 24 dynamic spheres, oriented boxes and capsules, the static
teapot/bunny triangle soups, and a tiled floor 32 units wide. The model fixtures are
uniformly scaled to a longest extent of eight and placed upright on the floor.
The floor uses four-unit tiles so the existing segment/triangle kernels' quartic
intermediates remain bounded for S15.16. Mesh construction/setup allocates once;
tick scratch vectors retain their capacities.

## SDL2 + glow GLES3 demo

```sh
cargo run --release --features physics-demo --example physics_demo -- --scalar fixed32
```

Choose `f32`, `fixed32` (S15.16) or `fixed64` (S31.32) at launch. The optional
`physics-demo` feature enables glow 0.18 and SDL2 0.38, using the system SDL2
development package through pkg-config. A GLES3-capable driver is required.
SDL explicitly requests the OpenGL ES profile, version 3.0; the renderer checks
the actual context and compiles GLSL ES 3.00 shaders. Headless physics and normal
library/no_std builds do not require SDL2 or GL.

![Fixed-point scene with spheres, oriented boxes and capsules above the teapot and bunny](images/physics-demo.png)

The renderer derives primitive dimensions and transforms directly from each
body and uploads the same immutable triangles used by its collision trees.
Static meshes are shaded, the floor has a procedural grid, and a built-in bitmap
HUD shows the scalar backend, motion counts, contacts and broad-phase pairs.
GPU resources have explicit ownership and are destroyed before SDL's context.

| Input | Action |
| --- | --- |
| Left drag / right drag | Orbit / pan camera |
| Mouse wheel | Zoom |
| Space / N | Pause/resume / advance one tick |
| R | Reset the scene |
| 1 / 2 / 3 | Launch a sphere / box / capsule toward the scene |
| B / C | Toggle body AABBs / contact normals |
| Escape | Exit |

Interactive rendering accumulates elapsed time for fixed 120 Hz simulation,
with at most eight catch-up ticks per frame. A finite run advances exactly two
ticks per frame and disables vsync, allowing reproducible driver smoke tests:

```sh
cargo build --release --features physics-demo --example physics_demo
SDL_VIDEODRIVER=offscreen target/release/examples/physics_demo \
  --scalar fixed32 --frames 120 --hidden --capture /tmp/physics.ppm
```

The offscreen driver depends on the local SDL/EGL installation. Capture reads
the final back buffer before swapping, using GLES' supported RGBA byte format,
checks GL errors, and writes a top-down RGB PPM file. `--capture` applies when
the `--frames` limit is reached. The committed screenshot is from this real
fixed-point render path, converted from PPM to PNG with FFmpeg.

## Tick and ownership contracts

`World::step()` advances one represented 1/120-second tick:

1. Semi-implicit Euler integrates gravity and bounded velocities, then updates
   and normalizes orientation quaternions.
2. Broad phase insertion-sorts the previous X sweep order, rejects disjoint
   Y/Z intervals, and visits **all** overlapping static-tree leaf candidates.
   It produces typed body pairs or stable mesh/triangle IDs, without contacts.
3. Narrow phase reuses existing overlap kernels and generates contact points,
   B-to-A unit normals and penetration depths. Analytic rounded-shape witnesses,
   SAT, clipped polygons and spread-point reduction cover every convex pair and
   sphere/capsule/box versus triangle. Four-point manifolds support box faces.
4. Twelve accumulated sequential-impulse passes solve restitution and a two-axis
   Coulomb friction disk. Previous local anchors and normal agreement warm-start
   impulses; tangent impulses are transformed into the new contact basis.
5. Four split position passes remove penetration without adding correction
   velocity. Seam points merge, and each body/object pair retains at most eight
   deepest constraints. Damping and static-supported quiet ticks drive sleeping.

Tree splits break equal center keys by original triangle ID, and each leaf
visits IDs in ascending order. Before contact reduction, candidates sort by
variant and stable body/mesh/triangle IDs. Reduced constraints sort by owner and
their full represented geometric/solver values before sequential impulses.
These total orders remove dependence on standard-library partition layout,
broad-phase visitation order and tree leaf size. Triangle input order remains
part of the scene's identity. This changes trajectories from the initial demo,
whose solver inherited traversal order; it preserves the collision contracts.

Bodies own immutable dimensions, mass/inertia and controlled pose/velocity state.
Their world-local IDs are append-only and remain stable until reset. There is
no per-body removal/reuse and the demo limits capacity to 128. Static trees own
immutable world-space triangles, shared by collision and rendering consumers.
No global state, runtime numeric-policy switch or untyped collider payload exists.

Construction validates dimensions, material, mass, finite pose and normalized
orientation. Mass reciprocals and inverse-inertia divisions are checked and use
the same native quantization as ordinary arithmetic. Shape-inertia intermediates
and stepping still require the caller to keep every intermediate representable.

The current example constructor enforces dimensions 0.1–4 units, capsule
centerline half-length 0–4, and mass 0.1–10; the example world bounds centers to
±24. These are heuristic scene limits chosen for the initial S15.16 workload,
not fundamental physics limits or a derived overflow guarantee. They also apply
to f32/f64 even though those backends have different working ranges.

Small masses/radii produce large inverse inertia: a sphere has
`I^-1 = 5 / (2 * mass * radius^2)`. Large dimensions/coordinates increase collision
intermediates, and tiny features can be poorly resolved by S15.16's 1/65,536
quantum. An accepted input can still overflow a later product; finite values and
these thresholds do not prove safety for every contact configuration.

For a reusable general engine, body construction should validate positive finite
physical inputs and use checked, representable mass/inertia preparation. Scene
size limits and escape boundaries belong in demo configuration. That separation
is the intended generalization; the current example still enforces the limits
above. Other geometry/scales require their own range analysis.

`Awake` accumulates quiet ticks only with immutable mesh support whose normal
opposes gravity (any static contact at zero gravity) and **no body/body contact**;
90 transitions to `Sleeping`. Dynamically touching bodies stay active, so a
support sliding away cannot leave an asleep body suspended. Any body contact
wakes both participants before preparing contact masses, including slow or
angular-only contact. External velocity/impulse commands also wake the body.

An effective gravity change wakes all enabled bodies before their next
integration and resets quiet counters. Identical gravity preserves them;
nonfinite gravity is rejected without changing force or motion state. A center
outside ±24 becomes `Disabled`, is omitted from collision/rendering and rejects
motion commands. Resetting the whole world starts a fresh ID domain. The floor
is finite: bodies can roll off its edge and subsequently leave the world.

This conservative rule replaces the original per-body dynamic-support sleep
optimization. Sleeping stacks would require a continuous support/island lifecycle;
one-time waking on external mutation is insufficient because a dependent can
resleep before slow support motion separates it. The bounded example keeps
stacks active instead of introducing that graph. This trades additional stack
work for a smaller, explicit lifecycle while retaining isolated floor/mesh sleep.

`Stats` reports candidate pairs, tree bounds/triangle work, reduced contacts and
motion states for the completed tick. The headless runner also checks every
body's represented pose and velocity for nonfinite values. Deterministic ticks
repeat within one backend; different scalar quantization produces different
trajectories and contact decisions.

The [raw replay and determinism contract](physics-determinism.md) fixes starting
inputs and tick-indexed commands and checks full state fingerprints, including
sleep counters and both impulse buffers. Both fixed formats match all 1,441
reference states on native x86-64, emulated ARMv5TE and emulated RISC-V64.
Interactive camera/frame timing is outside that contract; applications must
quantize and schedule external inputs identically to obtain matching ticks.

This example uses discrete contacts and two-sided triangle surfaces. It does
not provide CCD, joints, deformables, dynamic concave meshes, exact predicates,
solid containment tests, or production-grade arbitrary-scale stability.

## Regression coverage

`cargo test --test physics_examples` covers all shape pairs, misses, contact
normal direction, coincident capsule centerlines, four-point box faces,
all-candidate traversal, sweep versus
an exhaustive pair oracle, floor settling on all three backends, restitution
and momentum, friction, angular impulses, stacked boxes, sleeping/waking,
terminal disabled bodies, construction/capacity and repeated fixed ticks.
Slow-support regressions run 1,200 ticks after a 0.14-unit/s support velocity
change, exceeding the quiet window. Gravity reversal, unchanged/rejected gravity,
side-wall contacts, zero-speed dynamic contacts and disabled-body force changes
exercise the sleep eligibility and mutation boundaries.
Partition-permutation tests cover coincident split keys and five leaf sizes;
a dense six-box test reverses/rotates candidate visitation for 180 ticks and
compares raw poses, velocities and warm impulses against the normal pipeline.

Implementation validation also ran offline all-target tests with `physics-demo`,
fixed-only tests, libm all-target checks, formatting and documentation with
warnings denied. Each backend completed 1,200 headless ticks with finite pose
and velocity checks. On SDL2 2.32.10 / Mesa 26.0.8, each backend rendered 120
offscreen frames in an actual OpenGL ES 3.2 context without a GL error; fixed32
also exercised screenshot readback. These runs validate this bounded scene,
not arbitrary scales or identical trajectories across scalar backends.
