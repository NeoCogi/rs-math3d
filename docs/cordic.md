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
i64 chunks for control-flow cycles and accidental i128 step arithmetic.
