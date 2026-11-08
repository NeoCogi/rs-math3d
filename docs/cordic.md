# Const-selected CORDIC experiment

The fixed analytic API accepts an opt-in CORDIC working width through stable
const generics. The choice is checked by inline const assertions and folded
during monomorphization. No nightly generic-const-expression feature or runtime
mode argument is needed:

```rust
use rs_math3d::Fixed;
type F = Fixed<i32, 16>;
let angle = F::try_from_ratio(1, 4).unwrap();
let compact = angle.sin_cos_with::<64>();
let baseline = angle.sin_cos_with::<128>();
assert_eq!(compact, baseline);
```

`sin_cos_with`, `tan_with` and `acos_with` have checked counterparts named
`checked_sin_cos_with`, `checked_tan_with` and `checked_acos_with`. All take the
working integer width as the const argument, rather than the storage width or
fractional precision. The only supported values are 64 and 128. Width 64 also
requires destination `F <= 48`; unsupported combinations fail at compilation.
Every existing ordinary/checked method and `RealScalar` implementation still
selects 128. This experiment does not automatically switch any format to i64.

The existing 128-bit path uses Q96 and 97 steps. The i64 candidate selects its
working fractional precision with the const expression `clamp(F + 16, 32, 60)`
and executes one more step than that precision. Examples:

| Destination fractional bits | i64 working bits | Rotation steps |
| ---: | ---: | ---: |
| 2–16 | 32 | 33 |
| 29 | 45 | 46 |
| 32 | 48 | 49 |
| 44–48 | 60 | 61 |

The Q60 cap leaves three integer magnitude bits in signed i64. A reduced angle
fits pi; unit-circle vectoring grows by less than two. At least twelve working
guard bits remain beyond the supported destination precision. Constants come
from the existing 120-digit Decimal generator, then their selected Q60-derived
tables are rescaled in const evaluation. Rotation and vectoring expand into
up to four chunks of sixteen literal steps. Const-false steps and unused chunks
disappear; vectoring can still exit on an exact zero residual.

Both widths use one generic engine in `src/fixed/cordic.rs`. Its sealed
`Backend` trait supplies the primitive `Word`, scale, pi, gain and angle table;
the engine owns one typed state, one rotation step body, one vectoring step body
and their expanded schedules. `Wide` selects i128/Q96 independently of the
destination format, keeping one shared Q96 specialization. `Narrow<F>` supplies
the format-derived i64 constants. Primitive operators monomorphize directly in
the backend word, without conversions or dynamic dispatch inside the steps.
The earlier separate i64 module and analytic-module Q96 engine were removed.

Only the CORDIC state and step arithmetic become i64. Full-range range reduction
retains Q64/i128; acos retains its Q62 coordinates and u128 integer root before
narrowing the vectoring inputs. Output quantization and tangent division also
retain widened arithmetic. This is not a wholly i128-free embedded backend.
Preserving those boundaries isolates the working-state/iteration experiment
from a separate angle-reduction or root-precision redesign.

Sine, cosine and acos retain the experimental target of an absolute error within
one destination quantum. Guard precision is not a correct-rounding proof for
all inputs. Tangent can amplify small component errors dramatically near poles,
even when the result fits the destination; checked arithmetic detects domain or
range errors, not loss of numerical accuracy. Its i64 variant remains an
explicit experimental choice.

`tools/generate_fixed_constants.py` regenerates both CORDIC tables and independent
Decimal references. The new fixture includes exact raw integer inputs across
full storage ranges, endpoints, small values, quarter-turn neighborhoods and
deterministic random samples. Reference outputs split into an integer and a
residual, preserving sub-quantum error measurements even above f64's consecutive
integer range. Taylor sine/cosine and half-angle-reduced atan/sqrt acos supply
independent references; no CORDIC result is used as the accuracy oracle.

Run `cargo test --test cordic_precision` and repeat with
`--no-default-features`. Tests exhaust all 786,432 i16 inputs across the twelve
analytic scales, check one-quantum sine/cosine/acos error, validate domains and
checked/ordinary agreement, and cover full-range Decimal references through
F=48. `python3 tools/check_unrolled_codegen.py` additionally inspects specialized
i64 chunks for control-flow cycles and accidental i128 step arithmetic. It
traces concrete width-selected consumer calls to the generic chunk bodies and
requires Q96 consumers at F=16,48,61 to share exactly fourteen chunks. Internal
regressions compare every guard bit against the original typed iterative
schedules: Q96 on a dense 16-bit angle grid, and i64 at all 47 supported
fractional scales. This preserves step order, partial final chunks and exact-zero
vectoring exits independently of final destination rounding.

## Benchmark method

```sh
cargo run --release --example cordic_bench -- 1000 7
```

Arguments are positive batch and sample counts, defaulting to 1,000 and 7. Each
timing row measures 256 inputs per batch. Input creation, conversions, allocation,
reference parsing and error analysis occur outside timing. The harness warms
each operation, black-boxes inputs/results, and reports median/min/max sample
throughput including loop overhead. An even sample count uses the upper median.
Both widths use the same ordinary API paths and inputs. Bounded sine/cosine and
tangent inputs span [-1,1); full-range sine/cosine uses deterministic raw storage
values. Acos spans [-1,1). Tangent timing avoids poles; its precision samples
include pole neighborhoods. Timings include retained reduction/root/division
costs and are not just measurements of isolated CORDIC steps.

The precision section evaluates 9,091 independently generated Decimal references
over seven storage/scale combinations: 2,653 each for sine, cosine and tangent,
plus 1,132 for acos. It reports absolute maximum and RMS error in *destination
quanta*, counts beyond one quantum, rejected representable references, accepted
reference overflows, differences from Q96, and worst-case raw inputs. The Q96
path is measured against the same oracle, rather than treated as exact truth.
Results outside destination range are counted separately from numerical error.

## Host results, 2026-11-08

Intel Core Ultra 7 155H, x86_64 Linux, pinned logical CPU 0, rustc 1.98.1
(`48a229cea`, LLVM 22.1.8), default release profile and `std` feature, without
custom build flags, LTO or CPU selection. Command:
`taskset -c 0 cargo run --release --offline --example cordic_bench -- 1000 7`.
Each sample contains 256,000 calls. See the [full output and error ranges](benchmarks/cordic-x86_64.txt).

Median ns/call; sine/cosine columns use the bounded workload:

| Storage / F | i64 working precision | Q96 sin_cos | i64 sin_cos | Speedup | Q96 acos | i64 acos |
| --- | --- | ---: | ---: | ---: | ---: | ---: |
| i32 / 8 | Q32, 33 steps | 254.73 | 57.71 | 4.41x | 375.81 | 91.62 |
| i32 / 16 | Q32, 33 steps | 253.06 | 57.87 | 4.37x | 377.34 | 91.62 |
| i32 / 29 | Q45, 46 steps | 239.73 | 68.59 | 3.50x | 378.40 | 128.18 |
| i64 / 16 | Q32, 33 steps | 253.88 | 57.38 | 4.42x | 376.42 | 94.06 |
| i64 / 32 | Q48, 49 steps | 252.73 | 84.63 | 2.99x | 378.02 | 135.69 |
| i64 / 44 | Q60, 61 steps | 252.76 | 100.96 | 2.50x | 376.95 | 165.52 |
| i64 / 48 | Q60, 61 steps | 252.89 | 100.37 | 2.52x | 376.51 | 165.56 |

Full-range i64-core sine/cosine takes 56.88–101.04 ns across these formats,
versus 241.45–255.39 ns for Q96. The improvement therefore survives the shared
full-range reduction boundary on this machine. Acos improves by approximately
2.3–4.1x while retaining the same guarded root. These changes do not accelerate
the collision overlap queries, which contain no CORDIC calls.

Maximum absolute Decimal-reference error for the i64 candidate, in destination
quanta (1 quantum = `2^-F`):

| Storage / F | max(sine error, cosine error) | Acos error | Differences from Q96, these three operations |
| --- | ---: | ---: | ---: |
| i32 / 8 | 0.499818 | 0.499460 | 0 |
| i32 / 16 | 0.499198 | 0.497961 | 0 |
| i32 / 29 | 0.499853 | 0.489527 | 0 |
| i64 / 16 | 0.501011 | 0.497391 | 0 |
| i64 / 32 | 0.506359 | 0.493262 | 0 |
| i64 / 44 | 0.502881 | 0.496250 | 0 |
| i64 / 48 | 0.510074 | 0.495542 | 0 |

All 6,438 sine/cosine/acos Decimal samples round identically to Q96, with no
one-quantum violations. Their RMS errors range from approximately 0.25 to 0.29
quanta. The exhaustive i16 suite separately passes 1,572,864 sine/cosine component
checks and 32,772 acos checks across all twelve scales. These measurements
support the candidate's precision target; they do not prove every i32/i64 input
or guarantee bit-identical results beyond the sampled cases.

Tangent needs a separate decision. S15.16 Q96's maximum sampled error is 0.498
quanta; the i64/Q32 candidate reaches 46,649 quanta, about 0.712 in scalar units,
with 15 of 367 representable samples exceeding one quantum. The worst raw input
is 102946, or 1.570831298828125 radians, close to pi/2. Even though sine and cosine
quantize identically in the destination, their *guarded* components differ, and
the quotient amplifies that difference. Wider-storage full-range tangent also
exposes substantial existing Q96 range-reduction sensitivity near poles; full
statistics for both paths are retained rather than attributing it all to i64.
No checked success/overflow classification differences occurred in this fixture.

The evidence supports incorporating the i64 sine/cosine/acos path as an explicit
option for F<=48. Keep tangent and current automatic format selection on Q96.
Changing defaults warrants target measurements and stronger analysis across
the remaining wide formats. ARMv5/RISC-V timings were not measured, and this
implementation still requires wide arithmetic outside the CORDIC step core.
