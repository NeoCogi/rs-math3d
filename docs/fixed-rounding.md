# Fixed arithmetic policy selection

Native arithmetic is now the `Fixed<R, F>` default in both ordinary and checked
operations: widened multiplication and fused products floor-shift, division
truncates toward zero, and prepared reciprocals match division. Ordinary results
wrap; checked operations reject out-of-range quantized outputs. Conversions,
parsing, explicit `round()` and analytic outputs retain nearest-even rules.

The experiment below informed this choice. Its timing and numerical data are
historical observations from the recorded revisions, including an optimized
nearest-even control. The temporary variant-rewriting runner has been retired;
the library now has one native arithmetic contract, with no runtime mode switch.

Current Rust-only reproduction commands are:

```sh
cargo run --release --example mesh_bench -- 200 7
cargo run --release --manifest-path tools/Cargo.toml --bin fixed-precision
cargo run --release --manifest-path tools/Cargo.toml --bin fixed-precision -- profile s15 capsule 10000
```

The numerical probe retains exact bounded integer references, directed-rounding
and cross-product residuals, analytically constructed near-contact cases,
individual f64/f32/fixed ray and shape outcomes, and signed integration examples.
Its scalar error conversion subtracts the exact integer quotient before using
floating residuals. This avoids erasing low output bits at S31.32 precision.
The root unit tests cover native checked/ordinary parity, wrapping, all i8 scales,
negative fused remainders and prepared-reciprocal agreement.

The OBB envelope evaluates both signed half-axis endpoints. The originally
reproduced bounds failures and the adapted benchmark candidate remain recorded
below. Those adaptation costs are included in the historical native measurements.

The [Rust developer tools guide](../tools/README.md) covers reproduction of
constants, independent references, pinned mesh data and the LLVM codegen audit.

## Native default baseline, 2026-11-08

The supported native implementation (`f4dddfa`) was rerun on the same pinned
x86_64 host with the default release/std configuration, 200 batches and seven
samples. Median ns/query:

| Query | f32 | S15.16 / i32 | S31.32 / i64 |
| --- | ---: | ---: | ---: |
| capsule / teapot | 485.83 | 660.97 | 912.19 |
| capsule / bunny | 330.11 | 481.14 | 644.89 |
| OBB / teapot | 260.62 | 287.38 | 351.41 |
| OBB / bunny | 167.82 | 185.86 | 232.26 |

The [native raw baseline](benchmarks/native-fixed-x86_64.txt) retains ranges,
flat comparisons, work counters, exact-rational errors, signed integration,
near-contact classifications and per-backend f64 comparisons. Its complete
precision CSV matches the selected native experiment byte for byte. All 7,808
geometric records were checked, including the f64 reference itself; the other
backends have zero sampled hit or witness changes relative to it. Capsule
near-contact false positives remain 8/102 per fixed format, with no false
negatives in that constructed set. ARMv5/RISC-V timing remains unmeasured.

The completed default passes 156 library unit, 42 integration and 36 example
tests; the fixed-only suite and 24 doctests also pass. The Rust tools pass 27
tests. The codegen audit verifies 65 consumers, 14 shared wide chunks and 28
narrow chunks. Mesh reproduction retains both binary SHA256 values, and the
independent generator reproduces its output at both 512 and 768 working bits.

## Historical policy comparison, 2026-11-08

These measurements use an Intel Core Ultra 7 155H, pinned logical CPU 0,
`rustc 1.98.1 (48a229cea 2026-09-01)` with LLVM 22.1.8 on x86_64 Linux.
Release optimization and the default std backend are used, with debug info 2
for attribution. No custom RUSTFLAGS, LTO or target-cpu option was supplied.
The geometry/library revision is `0b450a9`; the harness is committed as `3d96a0d`.
Both experimental arithmetic implementations are isolated copies.

The [raw timings, counters and test output](benchmarks/fixed-rounding-x86_64.txt)
and [machine-readable aggregates](benchmarks/fixed-rounding-x86_64.json) retain
all three builds, scalar errors, ray comparisons and analytical boundary counts.
PMU counts were collected with explicit `cpu_core` events. This archived run used
the temporary variant runner at the recorded harness revision; use the current
Rust commands above to reproduce the selected native arithmetic.

### Query performance

Median ns/query, with seven samples and 200 tree batches per sample. Each
shape/model sample contains 52,800 queries. The f32 reference column comes from
the optimized nearest build; raw logs also retain f32 controls in the other builds.
Native OBB measurements include the signed-interval envelope adaptation below.

| Format | Query | f32 reference | Nearest-even | Native | Native speedup |
| --- | --- | ---: | ---: | ---: | ---: |
| S15.16 | Capsule / teapot | 511.81 | 1003.23 | 658.37 | 1.52x |
| S15.16 | Capsule / bunny | 338.97 | 726.59 | 472.02 | 1.54x |
| S15.16 | OBB / teapot | 255.51 | 439.92 | 284.43 | 1.55x |
| S15.16 | OBB / bunny | 169.85 | 267.81 | 187.29 | 1.43x |
| S31.32 | Capsule / teapot | 511.81 | 1579.39 | 918.53 | 1.72x |
| S31.32 | Capsule / bunny | 338.97 | 1011.83 | 644.00 | 1.57x |
| S31.32 | OBB / teapot | 255.51 | 498.41 | 352.13 | 1.42x |
| S31.32 | OBB / bunny | 169.85 | 293.60 | 224.43 | 1.31x |

Native improves these shape queries by **1.31–1.72x** over optimized nearest-even
(S15.16: 1.43–1.55x). f32 remains faster in this host corpus. The existing branchy
nearest build is a separate control: S15.16 capsule medians are 2482/1788 ns and
OBB medians 1383/787 ns on teapot/bunny, versus 1003/727 and 440/268 ns after
branchless nearest rounding. The policy comparison therefore does not attribute
that earlier implementation improvement to abandoning nearest-even.

Other query families show a smaller or reversed difference:

| Format | Query | Nearest-even ns | Native ns |
| --- | --- | ---: | ---: |
| S15.16 | Teapot nearest ray | 169.96 | 129.28 |
| S15.16 | Bunny nearest ray | 302.64 | 268.46 |
| S15.16 | Teapot/bunny surface pair | 2326.98 | 2149.24 |
| S31.32 | Teapot nearest ray | 222.33 | 246.70 |
| S31.32 | Bunny nearest ray | 395.50 | 431.25 |
| S31.32 | Teapot/bunny surface pair | 2384.21 | 2269.60 |

S31.32 native rays are 9–11% slower than optimized nearest-even on this run.
Removing rounding does not guarantee better generated code for every operation,
format and workload. These observations are host/corpus-specific; ARMv5 and
RISC-V were not measured.

`perf stat` on isolated teapot shape queries reports the following amortized
counts per query. All arithmetic snapshots retain the same input corpus; no
other benchmark runs concurrently.

| Format/query | Nearest instructions | Native instructions | Nearest branch misses | Native branch misses |
| --- | ---: | ---: | ---: | ---: |
| S15.16 capsule | 13346 | 8360 | 13.76 | 19.24 |
| S15.16 obb | 8118 | 5721 | 0.26 | 0.09 |
| S31.32 capsule | 17539 | 15464 | 49.95 | 12.94 |
| S31.32 obb | 9218 | 6777 | 0.36 | 0.13 |

Once nearest rounding is branchless, the native speedup comes substantially
from fewer instructions. Branch misses do not decrease in every kernel. Raw
files also retain cycle counts, f32 controls and the original branchy build.

### Numerical results

All three builds have zero tree/flat classification mismatches in the benchmark
verification passes. Optimized nearest reproduces the current build's entire
numerical probe output exactly: scalar errors, boundary decisions, integration
results and every stored geometric witness/parameter agree.

Both policies' 896 rays and 1,056 shape poses per backend retain the same hit
classifications and triangle witnesses as the probe's f64 reference. All twelve
mesh-pair classifications also match f32 in each backend. This corpus agreement
does not imply identical numerical results or exact geometric predicates.

Against exact bounded integer rationals, S15.16 mean absolute multiply error is
0.2504 quanta for nearest versus 0.4976 for native; RMS error is 0.2889 versus
0.5751. The largest observed error is 0.5 versus 0.99997 quanta. Dot/cross have
similar distributions. Native floor multiply/dot/cross introduce approximately
**-0.5 quantum of signed bias per result**, whereas sampled nearest biases are
close to zero. Native division truncates toward zero: its mean absolute error
is 0.5003 versus 0.2495 quanta, while positive/negative biases largely cancel in
the balanced sample. S31.32 shows the same distributions in its smaller quanta.
One quantum is approximately 1.526e-5 for S15.16 and 2.328e-10 for S31.32.

Nearest preserves `cross(a,b) + cross(b,a) == 0` in all 300,000 sampled bounded
components. Native has a -1 quantum residual in 299,984/300,000 S15.16 components
and all 300,000 S31.32 components. This is directed quantization of nonintegral
components, not storage overflow; `cross(a,a)` is still exactly zero.

Ray parameters differ even though the hit/witness classifications agree. Maximum
absolute differences from the f64 reference are:

| Format | Model | Nearest max parameter delta | Native max parameter delta |
| --- | --- | ---: | ---: |
| S15.16 | teapot | 1.68020023e-03 | 2.44004042e-03 |
| S15.16 | bunny | 3.12052987e-04 | 4.67143021e-04 |
| S31.32 | teapot | 1.14646070e-10 | 2.30080843e-10 |
| S31.32 | bunny | 1.16049392e-10 | 2.29043451e-10 |

These are stored ray-parameter differences, not guaranteed world-distance error
bounds. Determinant quantization can amplify S15.16 scalar rounding. RMS values
and comparisons directly between policies are retained in the JSON.

For analytically constructed capsule/triangle contacts, each format has **2
false positives out of 102** near-boundary cases under nearest and **8 out of
102** under native. Neither has false negatives in this constructed set. The
OBB plane-contact cases have zero false positives/negatives under both policies.
These cases deliberately concentrate within eight quanta of contact; their
counts are not a general collision failure rate.

At a small integration step, the floor policy can suppress positive motion
while allowing negative motion. After 10,000 updates with the same represented
`velocity=1/1000`, `dt=1/120`:

| Format | Positive reference | Nearest positive / negative | Native positive / negative |
| --- | ---: | ---: | ---: |
| S15.16 | 0.08390285075 | +0.15258789063 / -0.15258789063 | 0 / -0.15258789063 |
| S31.32 | 0.08333332728 | +0.08333241567 / -0.08333241567 | +0.08333241567 / -0.08333474398 |

Nearest also accumulates quantization error here, but retains sign symmetry.
The reference accounts for dt/velocity input quantization; this is an arithmetic
illustration rather than a full physics simulation or solver stability result.

### Required OBB adaptation and validation

An initial direct native substitution failed both fixed formats' existing
represented-corner envelope tests. The original bound mirrors a rounded positive
half axis. Under floor rounding, `floor(-x)` can be one quantum below `-floor(x)`;
its lower bound can therefore exclude an actual represented corner. For example,
the tilted-box regression with axes (0,0.6,0.8)/(0,-0.8,0.6) and half extents 0.25
fails even before any large-model query is run.

The native snapshot computes both `axis*half` and `axis*(-half)`, takes their
componentwise minima/maxima and adds each interval in represented corner order.
This encloses all eight corners without depending on odd quantization. Its cost
is included in the native OBB timings. The raw log retains the two original
failures as evidence and the passing adapted run; this is not an unfixed failure
in the measured native candidate.

All three final builds pass **21** probe/geometry tests each: 18 existing example
regressions and three policy-aware arithmetic tests. Both nearest builds also
pass all **154** existing release library unit tests. The exhaustive i8 checks
cover every operand pair at all eight fractional widths, including wrapping
outputs; independent fused references and zero division are covered separately.
At the time of this comparison no default policy was changed. Native arithmetic
has since been selected as the default, with checked arithmetic and prepared
reciprocals aligned to that policy.

The comparison exposed explicit bias and boundary tradeoffs. The current default
adopts native products/division consistently in both arithmetic tiers and retains
separate nearest-even conversion and analytic contracts, as described above.
