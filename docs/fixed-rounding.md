# Fixed arithmetic policy experiment

`python3 tools/bench_fixed_rounding.py --output /tmp/rounding-comparison --perf`
compares three isolated builds of the unchecked example collision workload:

| Build | Multiply and fused product accumulation | Division |
| --- | --- | --- |
| `current` | Existing nearest-even implementation | Nearest-even |
| `nearest` | Branchless signed nearest-even | Nearest-even |
| `native` | Arithmetic right shift, rounding toward negative infinity | Truncation toward zero |

The optimized `nearest` build separates the cost of the arithmetic policy from
the avoidable branches in the existing implementation. `native` describes the
usual integer multiply-and-shift technique, with explicitly defined signed
behavior. It is not a claim that C specifies a universal fixed-point format or
rounding policy. Both experimental builds retain widened products, fused
dot/cross accumulation, and wrapping destination narrowing. They do not reduce
input precision or quantize each dot-product term separately.

The tool owns source copies under its output directory and refuses to patch a
missing or duplicated arithmetic site. It changes ordinary fixed multiply,
divide and product accumulation. The native snapshot also prepares OBB bounds
from both signed ends of each half axis: floor rounding is not odd, so negating
one rounded end can exclude a represented corner. Numeric construction, checked
arithmetic code and analytic kernels are not patched. This isolates an unchecked
geometry experiment; the native snapshot is not a supported library mode with a complete
policy contract across every checked, conversion and analytic operation.

All Cargo operations share one sequential build directory. Logs, numerical
outputs, sources and standalone probe/benchmark binaries remain outside that
directory. The build directory is cleaned on successful completion unless
`--keep-build` is supplied. Use a new output directory for each run. Arguments
`--batches`, `--samples` and `--cpu` default to `200`, `7` and `0`.

The host requires Rust, Python 3.9 or later, and Linux `taskset`. `--perf` also
requires `perf` and permission to count user-space hardware events. No nightly
compiler, new crate dependency, runtime arithmetic-policy branch or library API
is introduced. The tool rejects custom `RUSTFLAGS` to keep its comparison
controlled and adds release debug information for attribution.

## What is measured

The existing `mesh_bench` runs unchanged against every build. It measures ray,
capsule, OBB and translated mesh-pair queries on the same Q8 teapot/bunny inputs,
including flat-reference verification. Shape centers/endpoints use Q8 and OBB
axes use Q16 in all scalar backends. Setup, conversion and quantization are
outside the query timers. Envelope construction, traversal, counters and result
handling are inside. See [the triangle-soup guide](triangle-soup.md) for corpus
size, query semantics and traversal contracts.

The additional [numerical probe](../tools/fixed_rounding_probe.rs) records:

- Exact rational errors for 100,000 bounded full-resolution operand sets per
  fixed format. Multiply, divide, fused dot and fused cross report signed bias,
  mean absolute error, RMS error and maximum error in output quanta. Integer
  quotient subtraction precedes floating error conversion, preserving small
  residuals at S31.32 precision. Divisors have magnitude at least 0.5; outputs
  fit the scalar and overflow does not contaminate these statistics.
- Cross-product antisymmetry residuals: exact arithmetic requires
  `cross(a,b) + cross(b,a) = 0`. Directed floor quantization can change that
  identity by a quantum even when both results fit.
- Capsule/triangle and OBB/triangle contacts placed at eight quanta inside and
  outside three analytically known contact planes, with both coordinate signs.
  Expected classification comes from the constructed distance, not f32 output.
- Individual outcomes for the benchmark's 448 rays and 264 poses per shape per
  mesh, on f64, f32, S15.16 and S31.32. Comparisons distinguish hit changes,
  witness changes, and maximum/RMS ray-parameter differences on matched hits.
  f64 is a rounded floating reference, not an exact geometric oracle.
- A 10,000-step constant-velocity integration illustration at represented
  `velocity = 1/1000` and `dt = 1/120`, for positive and negative motion. The
  reference uses the same represented inputs before multiplication quantization.
  This illustrates accumulated bias; it is not a rigid-body stability benchmark.

Each build runs the example geometry regressions plus policy-aware tests that
exhaust all i8 multiply/divide operand pairs at every fractional width, check
fused dot/cross against independent integer quotient/remainder references, and
check the ordinary zero-divisor panic. Nearest builds also run the existing
library unit tests. A failure remains visible in the policy test log and JSON;
existing nearest-even-specific unit expectations are not applied to the native
snapshot as if its altered arithmetic were unchanged.

With `--perf`, the probe runs one scalar and one teapot shape workload per
process, pinned to the selected CPU. Each counter run contains 2,640,000 queries.
On Intel hybrid hosts it selects the pinned CPU's PMU explicitly.
Process counters include small amortized setup and flat-verification costs;
printed query elapsed time excludes that setup. `.stat` files retain cycles,
instructions, branches and branch misses. No concurrent benchmark processes
compete for CPU time.

`comparison.json` retains the numerical summaries, timing medians/ranges,
cross-policy comparisons, test status, compiler version and source revision.
Raw CSV and benchmark/test logs preserve individual outcomes and diagnostics.
