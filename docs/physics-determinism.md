# Fixed-point physics replay

The example's fixed32 and fixed64 paths produce matching per-tick fingerprints
on native x86-64, emulated ARMv5TE and emulated RISC-V64 for the checked-in raw
reference scenes. Each format checks 1,441 states: constructor-prepared tick zero
and 1,440 complete physics ticks. This is executed cross-architecture evidence
for the versioned scenario and solver, rather than a guarantee for arbitrary
inputs, compiler bugs or future algorithm changes. Physical stability and
deterministic arithmetic remain separate properties.

## Ordering contract

Tree splits compare represented centers and break ties by original triangle ID.
Each leaf visits ascending IDs, independent of the unspecified internal order of
median selection. The physics consumer sorts candidates by variant and stable
body/mesh/triangle IDs **before** seam merging, contact reduction and warm-cache
matching. Reduced constraints sort by owner and every represented geometric and
solver value before accumulated sequential impulses. Equal total keys describe
interchangeable constraints. No randomized hash iteration or parallel reductions
enter the simulation.

Triangle and body input order are part of scene identity. Different tree leaf
sizes and broad-phase visitation orders produce the same simulated trajectory;
tree-work counters may differ. The canonical order intentionally changes the
initial demo's trajectories, which inherited partition/traversal order.

## Raw inputs and playback

[`examples/physics/replay.rs`](../examples/physics/replay.rs) owns the decoded
world and typed, tick-indexed commands. The checked-in seeds contain raw fixed
constructor inputs, quantized solver configuration, static world-space triangles
and the complete input timeline. Playback does not run the hosted float scene
generator or use rendering/camera conversions. Constructor normalization and
inertia preparation use the selected fixed scalar. Seeds start awake with empty
warm caches; they are not mid-simulation save files.

Commands replace velocity, apply a world-space impulse or replace gravity.
Their zero-based tick index applies **before** integration, and commands sharing
a tick execute in file order. Playback advances only through `step()`.
Completion and runtime input failure are terminal; a failed input cannot be
retried against a partially applied same-tick group. Decoding either returns a
complete owned replay or an error without changing an existing world.

The reference scene adds two diagnostic spheres to the ordinary 24 bodies:
one settles/sleeps and receives an impulse at tick 320; one immediately exits the
bounded world. Tick 640 applies ordered velocity/impulse commands, and ticks
900/1,100 reverse/restore gravity. A regression checks the actual tick-900
sleepers wake at tick 901 in both formats. Tests require nonzero quiet counters, sleeping,
disabled bodies, and nonzero current/previous cached impulses in both formats.

Wire/solver version `M3RPLY02` uses explicit little-endian values:

| Field | Encoding |
| --- | --- |
| Header | Eight magic bytes, then u32 storage width, fraction width, tick count, body count, mesh count, input count |
| Scalar | Sign-extended raw i64, eight bytes for either format |
| Configuration | 19 scalars in `config_values()` order |
| Body | Shape tag/dimensions, position XYZ, quaternion XYZW, mass, friction/restitution, linear XYZ, angular XYZ |
| Mesh | Friction/restitution, u32 triangle count, input-order triangles with three XYZ vertices |
| Input | u32 tick/tag; velocity/impulse include u32 body ID and two XYZ vectors; gravity includes one XYZ vector |

Shape tags are sphere=0, capsule=1, box=2. Input tags are velocity=0, impulse=1,
gravity=2. Configuration order is timestep, gravity XYZ, slop, position rate,
maximum correction, anchor-match distance, seam-merge distance, linear damping,
angular damping, speed limit, spin limit, domain, linear sleep
threshold, angular sleep threshold, warm fraction, merge-normal threshold and
match-normal threshold. Dimensions follow the corresponding `Shape` fields.

Version 02 adopts static-support-only sleeping: body/body contacts always wake
their participants and prevent sleep; changed gravity wakes all enabled bodies.
It removes the obsolete wake-speed threshold from the raw configuration and
reverses the reference gravity at tick 900. Version-01 packets are rejected,
and both seeds and complete traces are deliberately regenerated together. The
portable byte order/hash contract is unchanged; trajectories and hash identities
change with the solver contract. Old v1 evidence remains in commit `a2e5857`.

Decoding rejects wrong formats/versions, short/trailing data, invalid tags,
out-of-storage scalars, invalid constructor geometry/material/poses, invalid
command IDs/order, and resource-limit violations before playback. Limits are
128 bodies, 16 meshes, 32,768 total triangles, 4,096 inputs and 100,000 ticks.
Positions/impulse points are bounded to ±24, velocity/spin/impulses to ±15 and
gravity to ±24 per component. These checks do not prove every intermediate fits:
the [physics numeric-range preconditions](physics.md) still apply to custom seeds.

## State fingerprints

Each trace line contains a decimal completed-tick count and a 16-digit hex
FNV-1a-64 fingerprint. Integer overflow in the hash is explicitly wrapping.
Every tag, ID, length and counter hashes as a little-endian u64; every scalar
hashes its sign-extended raw i64. No native memory layout, pointer, `usize`
byte width, `DefaultHasher` or floating conversion defines a hash.

The fingerprint includes seed identity and tick, the current configuration,
all body shape/pose/velocity/material values, cached orientation axes, prepared
inverse mass/inertia, motion tag and quiet counter, and every represented field
of **both** contact buffers: owners, anchors, normal/tangents, depth, effective
masses, restitution/friction and all accumulated impulses. Seed identity covers
immutable triangles and the complete ordered command timeline once.

Acceleration nodes, sweep scratch, capacities and work counters are excluded.
They cannot change the next simulated state under canonical candidate ordering.
Fingerprint tests mutate hidden sleep/cache/configuration state and require a
different hash; another regression replays different tree leaf sizes and requires
identical hashes despite different traversal counters. FNV is a diagnostic check
with possible collisions, not a cryptographic proof of state equality or an
independent oracle for physical correctness.

## Reproduce the checks

```sh
cargo test --test physics_examples
cargo run --release --example physics_replay -- check \
  tests/fixtures/physics-fixed32.bin tests/fixtures/physics-fixed32.hashes
cargo run --release --example physics_replay -- check \
  tests/fixtures/physics-fixed64.bin tests/fixtures/physics-fixed64.hashes
```

`check` compares **every** state and stops at the first mismatch; missing/extra
trace states fail. `trace seed.bin output.hashes` produces a fresh actual trace.
Neither operation updates the checked-in expectation automatically.

For cross-target execution, install QEMU user runners and the Rust targets
`armv5te-unknown-linux-musleabi` and `riscv64gc-unknown-linux-musl`. Put the host
toolchain's `lib/rustlib/<host>/bin` directory (containing `rust-lld`) on `PATH`.
The measured x86-64 host used these static builds:

```sh
CARGO_TARGET_ARMV5TE_UNKNOWN_LINUX_MUSLEABI_LINKER=rust-lld \
  cargo build --release --target armv5te-unknown-linux-musleabi --example physics_replay
CARGO_TARGET_RISCV64GC_UNKNOWN_LINUX_MUSL_LINKER=rust-lld \
CARGO_TARGET_RISCV64GC_UNKNOWN_LINUX_MUSL_RUSTFLAGS='-C target-feature=+crt-static -C link-self-contained=yes' \
  cargo build --release --target riscv64gc-unknown-linux-musl --example physics_replay
qemu-arm -cpu arm926 target/armv5te-unknown-linux-musleabi/release/examples/physics_replay \
  check tests/fixtures/physics-fixed32.bin tests/fixtures/physics-fixed32.hashes
qemu-riscv64 target/riscv64gc-unknown-linux-musl/release/examples/physics_replay \
  check tests/fixtures/physics-fixed32.bin tests/fixtures/physics-fixed32.hashes
```

Repeat the last two commands with `fixed64` filenames. Use the explicit
`armv5te` target: the generic `arm-unknown-linux-musleabi` standard library used
here requires ARMv6, as its ELF attributes and an ARM926 instruction fault
confirmed. QEMU checks instruction semantics and ABI portability; these runs do
not measure physical ARM/RISC-V hardware performance. RISC-V32, big-endian
targets and physical ARM/RISC-V boards were not executed.

Intentional reference regeneration uses the explicit `record` command:

```sh
cargo run --release --example physics_replay -- record fixed32 /tmp/new-seed.bin /tmp/new-trace.hashes
```

Record each format separately, review the solver/seed change, and rerun all
architectures before replacing reference files. Keep the seed and trace paired.
Changes to schema or solver semantics require a deliberate version/fixture
decision. [Execution evidence](verification/physics-replay.txt) records the
toolchain and matching format-specific endpoints.
