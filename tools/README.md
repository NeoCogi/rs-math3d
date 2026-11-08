# Rust developer tools

Run these from any directory by supplying the path to `tools/Cargo.toml`.
The tools are a separate hosted package; the math library keeps its existing
dependencies and `no_std` support.

| Tool | Purpose |
| --- | --- |
| `check-unrolled-codegen` | Compiles public consumers and audits LLVM CFGs and CORDIC operand widths. |
| `fixed-precision` | Reports native scalar errors, geometric classifications, contacts and integration bias; `profile s15 capsule 10000` isolates a query loop. |
| `prepare-collision-meshes` | Verifies pinned source digests, encodes Q8 triangle soups and writes their manifest. |
| `generate-fixed-constants` | Independently generates constants and integer-plus-residual analytic references. |

```sh
cargo test --manifest-path tools/Cargo.toml
cargo run --release --manifest-path tools/Cargo.toml --bin check-unrolled-codegen
cargo run --release --manifest-path tools/Cargo.toml --bin fixed-precision
cargo run --release --manifest-path tools/Cargo.toml --bin prepare-collision-meshes -- --download
cargo run --release --manifest-path tools/Cargo.toml --bin generate-fixed-constants -- --check
cargo run --release --manifest-path tools/Cargo.toml --bin generate-fixed-constants -- --bits 768 --check
```

The mesh tool uses `curl`, `tar` and `sha256sum`. For offline sources, use
`--bunny-archive PATH --teapot-obj PATH`. `--output DIR` writes elsewhere;
`--check` verifies reproduction without writing. Both source digests are checked
before publishing any output, and archive extraction reads only the pinned PLY
member to stdout. Current binary fixtures reproduce byte for byte.

The constant/reference generator links the development host's GMP library
(`libgmp`, normally supplied by its development package). It uses owned `mpf`
values at 512-bit precision, Machin/Taylor series and half-angle identities.
It does not use the crate's fixed or CORDIC implementation as its oracle.
The 768-bit check verifies that stored constants and 36-digit residual text are
stable with another guard width. GMP is required only to build/run this generator
and its tests; it is absent from the math library and other tool binaries.

The previous arithmetic-policy variant runner is retired now that native is
the default. Historical nearest/native measurements remain in
[the selection report](../docs/fixed-rounding.md); current performance is measured
with the ordinary `mesh_bench` and `collision_bench` examples.

To share the root build cache, set `CARGO_TARGET_DIR` to the root `target` directory
and run Cargo commands sequentially. Otherwise tools use `tools/target`; clean
both targets when finished. The LLVM audit owns and removes its temporary probes.
