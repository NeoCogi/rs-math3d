<!--
Copyright 2020-Present (c) Raja Lehtihet & Wael El Oraiby

Redistribution and use in source and binary forms, with or without
modification, are permitted provided that the following conditions are met:

1. Redistributions of source code must retain the above copyright notice,
this list of conditions and the following disclaimer.

2. Redistributions in binary form must reproduce the above copyright notice,
this list of conditions and the following disclaimer in the documentation
and/or other materials provided with the distribution.

3. Neither the name of the copyright holder nor the names of its contributors
may be used to endorse or promote products derived from this software without
specific prior written permission.

THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS "AS IS"
AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE
IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE
ARE DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT HOLDER OR CONTRIBUTORS BE
LIABLE FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR
CONSEQUENTIAL DAMAGES (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF
SUBSTITUTE GOODS OR SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS
INTERRUPTION) HOWEVER CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN
CONTRACT, STRICT LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE)
ARISING IN ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE
POSSIBILITY OF SUCH DAMAGE.
-->

# Signed fixed-point math

`Fixed<Repr, FRAC>` stores a signed integer divided by `2^FRAC`. The storage
type selects the width; precision is part of the Rust type and has no runtime
storage cost. The implementation belongs to rs-math3d and uses integer
arithmetic for fixed-point calculations, including analytic functions.

## Formats and range

SINT.FRAC notation counts the sign bit separately. `Fixed<i32, 8>` is S23.8:
one sign bit, 23 integer magnitude bits and eight fractional bits.

| Type | Notation | Resolution | Range |
| --- | --- | --- | --- |
| `Fixed<i32, 8>` / `S23_8` | S23.8 | `1/256` | `[-8388608, 8388608 - 1/256]` |
| `Fixed<i32, 16>` / `S15_16` | S15.16 | `1/65536` | `[-32768, 32768 - 1/65536]` |
| `Fixed<i64, 16>` / `S47_16` | S47.16 | `1/65536` | `[-2^47, 2^47 - 1/65536]` |

Storage is sealed to `i8`, `i16`, `i32`, and `i64`. Valid formats satisfy
`FRAC < Repr::BITS`; unsupported storage and widths fail during compilation.
Unsigned integers do not denote signed storage. The integer magnitude bit
count is `Repr::BITS - 1 - FRAC`.

Every valid format supports raw construction and arithmetic. Numeric trait
capabilities require representable constants:

| Capability | Additional requirement |
| --- | --- |
| `Zero`, basic operators, parsing, float conversions | Any valid format |
| `One`, `Num` | At least one integer magnitude bit |
| `Scalar`, vectors and basic matrix arithmetic | At least two integer magnitude bits, to represent two |
| `RealScalar`, `RealVector`, analytic geometry | Scalar requirements and at least two fractional bits |

Formats with very few fractional bits have correspondingly coarse geometry.
The default fixed epsilon is four quanta, so callers using coarse formats may
need an explicit tolerance appropriate to their data.

## Construction and conversions

`from_bits` interprets a scaled integer. `try_from_int` and `try_from_ratio`
interpret numerical values:

```rust
use rs_math3d::{Fixed, MathError};
type F = Fixed<i32, 8>;
# fn main() -> Result<(), MathError> {
let one = F::from_bits(256);
let half = F::try_from_ratio(1, 2)?;
let three = F::try_from_int(3)?;
assert_eq!(one.to_f64(), 1.0);
assert_eq!(half.to_bits(), 128);
assert_eq!((three * half).to_f64(), 1.5);
# Ok(())
# }
```

`From` is implemented in both directions for `f32` and `f64`, providing `Into`
automatically:

```rust
use rs_math3d::Fixed;
type F = Fixed<i32, 8>;
let a: F = 1.5f32.into();
let b: F = (-2.25f64).into();
let single: f32 = b.into();
let double: f64 = a.into();
assert_eq!(single, -2.25);
assert_eq!(double, 1.5);
```

Float-to-fixed conversions round to nearest, ties to even. `From` panics for
NaN, infinity, or a rounded value outside the destination range. Use
`try_from_f32` / `try_from_f64` to receive a `MathError`. Implementing `From`
also supplies Rust's blanket `TryFrom`; that blanket conversion has the same
panicking behavior, so it is **not** a replacement for the inherent `try_*`
constructors.

Fixed-to-float conversions round when the float lacks enough significand bits.
They do not promise a bit-exact round trip for wide storage. Use `try_rescale`
for integer-only conversion between fixed formats:

```rust
use rs_math3d::{Fixed, MathError, Vector3};
# fn main() -> Result<(), MathError> {
let value = Fixed::<i64, 8>::from_bits((1_i64 << 60) + 1);
let same: Fixed<i64, 8> = value.try_rescale()?;
assert_eq!(same.to_bits(), value.to_bits());
let vector = Vector3::new(value, Fixed::ZERO, Fixed::ONE);
let cast = vector.try_cast::<Fixed<i64, 8>>().unwrap();
assert_eq!(cast.x.to_bits(), value.to_bits());
# Ok(())
# }
```

Vector/matrix `try_cast` uses `TryCastFrom`: fixed-to-fixed conversion rescales
integers directly, preserving bits beyond a float's significand. `num-traits`
`NumCast` remains available for interoperability but its generic fixed
destination path goes through `f64`; use `try_rescale` for precision-critical
fixed conversions. Arithmetic between different fixed types requires an
explicit conversion first.

## Arithmetic and text

Addition and subtraction are exact if the result fits. Multiplication,
division, rescaling and numeric construction round nearest with ties to even.
Negative values use the same symmetric rule. Ordinary operators wrap the rounded
result modulo the signed storage width in both debug and release builds.
Division and remainder by zero can panic. Explicit `checked_add`, `checked_sub`,
`checked_mul`, `checked_div`, `checked_rem`, `checked_neg`, and `checked_abs` return errors instead.
The positive counterpart of a signed minimum does not fit its representation.

`Repr` composes `num_traits::CheckedAdd` and `CheckedSub` for native raw-integer
overflow detection. These capabilities return `Option` and have no fixed-point
scale. `Fixed` owns the shared scale, constructs the result, and translates raw
overflow into `MathError::Overflow`. This preserves native arithmetic without
custom storage forwarding methods. Generic `R: Format<F>` arithmetic inherits
these capabilities even when the format cannot represent positive one. Raw
generic callers use the `num_traits` capabilities directly; the previous
`Repr::checked_add_storage` and `Repr::checked_sub_storage` hooks are removed.
Multiplication and division retain their separate scaling and rounding logic.

`trunc` rounds toward zero and `floor` toward negative infinity. `ceil`, `round`,
and `abs` use ordinary wrapping arithmetic; `checked_ceil`, `checked_round`, and
`checked_abs` reject an unrepresentable result. Remainder follows truncating
division and has the dividend's sign.

Dot/cross products, matrix products and Hamilton quaternion products accumulate
widened integer products and quantize once per output component. Scalar
expression chains still round at each operator; algebraically equivalent
expressions can therefore give different rounded results.

`Display` emits an exact terminating decimal, without allocation. Precision
formatting flags do not truncate that representation. Decimal `FromStr` and
`from_str_radix` (radices 2–36) accept an optional sign and radix point, rounding
once at the destination precision. Whitespace, exponents and digit separators
are rejected. Fractional input length is not limited to an integer denominator
width, so exact decimal output round-trips even at 63 fractional bits.

## Geometry and matrix inverse modes

Fixed values work with vectors, matrices, quaternions, transforms, bases, and
the geometric primitives and queries. `RealScalar` / `RealVector` replace the
old float-only trait names; no compatibility aliases are retained.

```rust
use rs_math3d::{Fixed, InverseMode, Matrix4, MathError, RealVector, Vector3};
use rs_math3d::transforms;
# fn main() -> Result<(), MathError> {
type F = Fixed<i32, 8>;
let point = Vector3::new(F::ONE, F::ONE, F::ONE);
let scale = transforms::scale(Vector3::new(
    F::try_from_int(2)?, F::try_from_int(4)?, F::try_from_int(8)?,
));
let unit = point.checked_normalize(F::DELTA)?;
let fast = scale.inverse(); // Unchecked reciprocal is the default.
let checked = scale.checked_inverse()?;
let rows = scale.checked_inverse_with(InverseMode::Rows)?;
let restored = fast * (scale * point);
assert_eq!(restored.x, point.x);
assert_eq!(rows.col[0].x, fast.col[0].x);
# let _ = (unit, Matrix4::<F>::identity());
# Ok(())
# }
```

Every matrix size supports two inversion algorithms:

| Mode | Behavior and tradeoffs |
| --- | --- |
| `InverseMode::Reciprocal` (default) | Computes cofactors/adjugate and prepares one determinant reciprocal for reuse. Intended as the fast path. Cofactor and determinant intermediates must fit the scalar representation. |
| `InverseMode::Rows` | Uses partial-pivot Gauss–Jordan row elimination. Has different intermediate range, conditioning and rounding behavior; useful when adjugate intermediates overflow. |

Select with `checked_inverse_with` / `inverse_with`. The named checked methods
`checked_inverse_reciprocal` and `checked_inverse_rows` are also available. Affine
matrices expose `checked_inverse_affine_with` / `inverse_affine_with` as well as
the default reciprocal variants. Ordinary `inverse`, `inverse_with`,
`inverse_reciprocal`, `inverse_rows`, and affine variants perform no validation
and never delegate to checked inversions. They require nonsingular finite inputs
and representable intermediates; invalid inputs can produce IEEE NaNs/infinities
or wrapped fixed values, and a fixed zero divisor can panic. Checked variants
validate the inputs and each arithmetic boundary and return `MathError`.

The 3x3 adjugate computes its nine cofactors once and reuses the first row for
the determinant: three products and two additions after the cofactors, rather
than the expanded six triple products. The 4x4 schedule caches twelve 2x2
minors. Determinant-only methods evaluate only the first cofactor row.

The fixed reciprocal retains 127 internal reciprocal fraction bits. Preparation
performs one division; applying it uses widened multiplication and remainder
correction. It matches checked fixed division without first quantizing `1/det`
to the scalar format. A representable inverse therefore does not become a zero
matrix merely because the scalar reciprocal determinant would round to zero.
This does not remove cofactor/determinant range limits; row mode addresses a
different set of intermediate-range cases. Benchmark both on the intended
target with `cargo run --release --example inverse_modes`.

### Explicit expansion of fixed iteration counts

The 2×2, 3×3 and 4×4 matrix kernels expand their index ranges into statements,
including row-inverse setup, pivot search, elimination and products. Vector
component reductions and scalar slices of lengths zero through four likewise
have expanded paths. Three-axis/edge queries and eight-octant subdivision use
explicit operations. These paths do not rely on compiler loop-unrolling
heuristics, including in debug builds. Arithmetic/error evaluation order and
fixed-point rounding remain unchanged; data-dependent checks still branch.
Checked two-product fixed accumulation uses a shared, explicitly expanded
wide kernel. Unchecked accumulation uses a wrapping `i128` sum and rounds once:
discarded high bits cannot affect any supported final storage result.

CORDIC's 97 steps are expanded into seven explicit chunk calls: six chunks of
16 steps and one final step. The Q96 working state and chunk functions are shared
across fixed formats. Chunk functions deliberately remain separate to bound
optimizer work and avoid duplicating the large schedule per format. Vectoring
still exits immediately when its residual becomes exactly zero; later chunks
are skipped. The expansion is checked against iterative internal Q96 references
before destination rounding, as well as the existing analytic references.

Public scalar slices longer than four keep a runtime fallback. Text processing,
searches and integer-root convergence retain their input-dependent iteration.
Tests and constant-generation tools retain iterative reference algorithms.

Run `python3 tools/check_unrolled_codegen.py` to inspect optimized host LLVM
control flow for `f32`, `f64`, `Fixed<i32, 8>` and `Fixed<i64, 16>` matrix/dot
consumers and the shared CORDIC functions. The check detects control-flow cycles
rather than assuming every backward jump is a loop. It validates the current
host compiler's output; instruction selection and performance on other targets
can differ. Explicit expansion trades code size for reduced iteration overhead,
so use the timing harness on the deployment target.

Checked normalization uses max-component scaling and compares fixed norms
against tolerances before quantizing squared values. It handles tiny nonzero
vectors and directions whose unscaled length would overflow. Fixed
normalization and rotations approximate unit length within quantization.

## Errors and tolerances

Checked scalar/matrix/transform methods return `MathError`. Checked queries
preserve the distinction between geometric absence and a failed calculation:

```rust
use rs_math3d::{Fixed, Intersection, MathError, Ray, Tri3, Vector3};
# fn main() -> Result<(), MathError> {
type F = Fixed<i32, 8>;
let zero = F::ZERO;
let one = F::ONE;
let triangle = Tri3::new([
    Vector3::new(zero, zero, zero),
    Vector3::new(one, zero, zero),
    Vector3::new(zero, one, zero),
]);
let ray = Ray::checked_new(
    &Vector3::new(zero, zero, one),
    &Vector3::new(zero, zero, -one),
    F::DELTA,
)?;
match ray.checked_intersection(&triangle)? {
    Some((parameter, point)) => assert_eq!(point.z, zero),
    None => panic!("the fixture intersects"),
}
# Ok(())
# }
```

`checked_intersection` returns `Ok(None)` for a miss and `Err` for invalid or
unrepresentable arithmetic. `checked_intersect` returns `Result<bool, MathError>`;
`checked_distance` returns `Result<T, MathError>`. Ordinary `distance` returns
`T`, `intersect` returns `bool`, and `intersection` returns `Option` for geometric
absence, all with unchecked arithmetic. Checked primitive constructors return
`Result`; checked primitive intersection methods return `Result<Option<_>, MathError>`.
Ordinary constructors and geometric methods never call their checked siblings.
See [the arithmetic contract](arithmetic.md) for the complete API policy.

Linear tolerances use world units. Angular tolerances are dimensionless finite
values in `0..=1`: parallel tests compare `|sin(theta)|`, and perpendicular
tests compare `|cos(theta)|`. Fixed-point angular norms compare widened squared
integers, so squaring a small epsilon cannot erase it. No fixed infinity or
NaN sentinel exists; unbounded slab endpoints are represented explicitly.

Scale invariance applies within representation and quantization limits.
Rescaling/rounding input can change its represented direction. Returned line
parameters retain `point + direction * t`; checked queries reject parameters that round nonzero
travel to zero. Ordinary queries assume their parameterization is representable. Finite fixed inputs can still exceed intermediate
or output ranges, which checked arithmetic reports rather than wrapping.

## Analytic implementation and accuracy

Square root uses an integer root and is correctly rounded. Default sine/cosine use
Q96 CORDIC with guarded range reduction; arc cosine uses guarded integer square
root and vectoring. Sine, cosine and arc cosine target an absolute error within
one destination quantum. Regression coverage includes exhaustive 16-bit
inputs and independent 120-digit Decimal references at full 64-bit angle
magnitudes across multiple fractional widths.

Tangent divides guarded components before final quantization; its error grows
near poles, where a uniform absolute error bound is not useful. A result
outside the destination range is an error in checked arithmetic. `checked_sqrt`
rejects negative inputs and `checked_acos` rejects inputs outside `[-1, 1]`.
Ordinary `sqrt`, `sin`, `cos`, `sin_cos`, `tan`, `acos` and the generic `t*`
methods perform no domain/range validation. Callers must satisfy the analytic
domain and representability requirements; their results are otherwise unspecified
numerically, and zero divisors can panic. Both tiers use integer-only algorithms.

Constants and independent references are reproducible with
`python3 tools/generate_fixed_constants.py`. Python Decimal is development
tooling only; fixed runtime calculations do not depend on floats or `libm`.

### Working width and embedded performance

`Fixed<R, F>` is generic over storage and scale; its default analytic implementation is
currently shared Q96 CORDIC in `i128`, with 97 rotation steps. That width preserves
guard bits for the highest-precision `i64` formats and the documented accuracy
checks. It is not a claim that the processor has a native 128-bit integer ALU.
LLVM can split unsupported integer operations into smaller operations; see its
[type legalization documentation](https://llvm.org/docs/CodeGenerator.html#selectiondag-legalizetypes-phase).
Depending on the operation and target, software support can dominate the cost.

The [const-selected CORDIC experiment](cordic.md) now adds opt-in
`sin_cos_with::<64>()`, `tan_with::<64>()`, `acos_with::<64>()` and checked
counterparts. Inline const assertions accept working widths 64/128 and require
F<=48 for i64. A format-derived const expression selects Q32 through Q60 and
33–61 steps together. Width 128 and all existing methods keep the Q96 baseline.
The i64 tangent variant can lose substantial accuracy near poles; this opt-in
experiment does not change the default analytic accuracy contract.

Full-range angle reduction is separate from rotation: both paths' reduction
shifts the input to Q64 in `i128` before taking the remainder. Simply substituting
`i64` would overflow. Tangent and arc cosine also have their own guarded division
and square-root requirements. Beyond analytics, ordinary scalar division and
dot/cross accumulation use widened `i128`/`u128` arithmetic too. A single `i32`
product fits `i64`, but three worst-case signed `i32` products can sum to `3*2^62`,
which exceeds positive `i64`. A narrower accumulator therefore needs either a
bounded-input contract or an algorithm preserving the existing rounding/range
semantics; it cannot silently wrap early.

Use the [collision benchmark](collision.md) to separate scalar arithmetic,
square root and trigonometry from narrow-phase queries. It includes fused fixed
`sin_cos` measurements as well as the common trait's `sin` plus `cos` expression.
None of its overlap kernels calls CORDIC. On ARMv5 or RISC-V, measure on the
actual core with its compiler, multiplication/division support and floating-point
extensions; the host results do not predict those timings.

## Backend-free builds

Fixed analytic math is allocation-free and works with `core` alone:

```toml
[dependencies]
rs-math3d = { version = "0.20.1", default-features = false }
```

The crate's `f32`/`f64` analytic implementations require a floating backend:
default `std`, portable `libm`, or Unix-only `system-libm`. Feature precedence
remains `std`, then `libm`, then `system-libm`. The previous rejection of all
featureless builds is superseded by the fixed-only configuration. Tests and
examples explicitly cover this configuration; no float analytic backend is
selected merely because the test harness can link `std`.
