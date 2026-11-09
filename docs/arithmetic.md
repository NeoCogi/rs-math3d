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

# Arithmetic contracts

Ordinary methods and operators use unchecked arithmetic by default. Methods
prefixed with `checked_` explicitly validate arithmetic and return `MathError`.
The choice is local to each call; there is no global mode, feature flag, runtime
policy branch, or implicit checking based on the scalar type.

Unchecked here means no library domain, finiteness, tolerance, degeneracy or
range validation. These APIs are safe Rust. They require valid mathematical
inputs and representable intermediates. Violating those numerical preconditions
can produce NaN/infinity for floats, wrapping results for fixed-point values,
or a native arithmetic panic, including fixed division by zero. Native integer
operators retain Rust's build-profile overflow behavior. No memory safety
obligation is imposed on callers.

Fixed multiplication uses widened products and an arithmetic right shift,
rounding toward negative infinity. Fixed division truncates toward zero.
Operators wrap the final scaled integer to the storage width; checked operations
use the same quantization and reject overflow before narrowing. Wide
dot/cross/matrix/Hamilton products accumulate first and floor-shift once per
output component in both tiers. Prepared reciprocals match truncating division.
Conversions, parsing, explicit `round()` and analytic outputs retain their
documented nearest-even rules. Floating-point checked arithmetic
rejects nonfinite operands, zero divisors and nonfinite results; finite underflow
may round to zero. Checked geometric parameters additionally reject a nonzero
physical travel distance whose stored parameter rounds to zero.

## API map

| Object / operation | Ordinary, unchecked | Explicitly checked |
| --- | --- | --- |
| Scalar arithmetic | `+ - * / %`, unary `-`, `squared`, `tabs` | `checked_add/sub/mul/div/rem/neg/abs/squared` |
| Product accumulation | `Scalar::sum_products` | `Scalar::checked_sum_products` |
| Fixed integral rounding | `ceil`, `round`, `abs` | `checked_ceil`, `checked_round`, `checked_abs` |
| Analytic math | `tsqrt/tsin/tcos/ttan/tacos`; fixed `sqrt/sin/cos/sin_cos/tan/acos` | `checked_sqrt/sin/cos/tan/acos`; fixed `checked_sin_cos` |
| Prepared reciprocals | `reciprocal`, `mul_reciprocal` | `checked_reciprocal`, `checked_mul_reciprocal` |
| Vectors | operators, `dot`, `cross`, `length`, `length_squared`, `distance`, `normalize` | `checked_*` counterparts, including component/scalar arithmetic |
| Matrix arithmetic | operators, `mul_matrix_matrix/vector`, `mul_vector_matrix`, `determinant` | `checked_*` counterparts; checked addition/subtraction |
| Matrix inversions | `inverse`, `inverse_with`, `inverse_reciprocal`, `inverse_rows`, affine variants | `checked_inverse`, `checked_inverse_with`, `checked_inverse_reciprocal`, `checked_inverse_rows`, affine variants |
| Quaternions | arithmetic, inverse, normalize, matrix/axis-angle conversions | `checked_*` counterparts |
| Transforms | assembly, projection, unprojection, camera/rotation, affine decomposition | `checked_*` counterparts |
| Plane/line/ray/segment geometry | constructors, closest-point, distance, intersections | `checked_*` constructors/calculations; segment tolerance uses `checked_distance_with_epsilon` |
| Other primitives | rectangle bounds/overlap/containment; box center/extents/subdivision; sphere radius; triangle coordinates | `checked_*` arithmetic counterparts |
| Queries | `distance -> T`, `intersect -> bool`, `intersection -> Option<_>` | `checked_distance -> Result<T, MathError>`, `checked_intersect -> Result<bool, MathError>`, `checked_intersection -> Result<Option<_>, MathError>` |
| Normal/basis construction | `tri_normal`, `quad_normal`, `basis_from_unit_rh/lh` | `checked_tri_normal`, `checked_quad_normal`, `checked_basis_from_unit_rh/lh` |

Storage-only construction, component access, transpose, min/max selection,
basis/matrix field assembly and casts do not acquire an unnecessary arithmetic
policy. Fallible numeric conversions retain Rust's `try_from_*`, `try_rescale`
and `try_cast` conventions. Float `From`/`Into` conversions keep their documented
range-checking behavior. `normalize_or_zero` is an explicitly selected fallback
policy; ordinary `normalize` does not use it.

## Matrix performance and rounding

`InverseMode::Reciprocal` remains the default in both tiers. It computes the
adjugate, reuses cofactors/minors in the determinant, and prepares one reciprocal
for all output cells. Fixed reciprocals retain 127 guard fraction bits, so a
scalar `1/det` that rounds to zero does not erase a representable inverse.
`InverseMode::Rows` uses partial-pivot Gauss-Jordan elimination instead. Both
modes have directly implemented unchecked and checked paths. They can differ
in intermediate range and rounding; unchecked does not mean exact arithmetic.

`is_affine(epsilon)` and `checked_is_affine(epsilon)` are approximate property
queries when epsilon is positive. Zero epsilon tests the represented last row
exactly, without subtracting or taking absolute values. Checked affine inverse
and decomposition require exactly `[0,0,0,1]` and report `Domain` for projective
structure. Checked unprojection selects the affine fast path only for that exact
row; otherwise it uses the complete inverse, preserving every projective term.
A coefficient smaller than epsilon can still matter at the queried world scale.

All fixed-size matrix/vector kernels remain explicitly expanded. The checked
paths contain validation branches; the ordinary paths do not add them.
Data-dependent pivot choices and geometric miss decisions remain necessary.
Long public slices and convergence/search/parsing algorithms retain runtime
iteration. Both CORDIC widths use one generic engine with sealed precision
backends. Q96 retains seven statically selected chunks of expanded steps; the
opt-in i64 backend selects three or four through const expressions.

## Choosing a boundary

Validate external data with checked operations where a failure can be handled,
then use ordinary operations when the caller can establish their preconditions.
A successful constructor does not guarantee that every future calculation fits:
public coordinates can change, and multiplication can exceed the scalar range.

```rust
use rs_math3d::{Fixed, InverseMode, MathError, Matrix3, RealVector, Vector3};
# fn main() -> Result<(), MathError> {
type F = Fixed<i32, 8>;
let matrix = Matrix3::<F>::identity();
let direction = Vector3::new(F::ONE, F::ONE, F::ZERO);
let checked_unit = direction.checked_normalize(F::DELTA)?;
let checked_inverse = matrix.checked_inverse_with(InverseMode::Rows)?;
// The caller has established validity and range for these particular inputs.
let fast_inverse = matrix.inverse();
let fast_unit = direction.normalize();
assert_eq!(fast_inverse.col[0].x, checked_inverse.col[0].x);
assert_eq!(fast_unit.x, checked_unit.x);
# Ok(())
# }
```

Checked normalization uses component scaling and handles inputs whose unscaled
squared length overflows/underflows. Ordinary floating norms use direct
arithmetic and require their squared intermediates to fit. Fixed norms use
unquantized integer squares in both tiers. Ordinary and checked algorithms can
therefore have different floating/fixed rounding without either being incorrect.
Checked matrix-to-quaternion conversion validates arithmetic and finite selected
coefficients; it assumes the supplied matrix describes a rotation, rather than
performing an orthogonality audit. Checked quad-plane construction returns a
representative plane and does not assert coplanarity.

`MathError` distinguishes overflow, division by zero, domain failure,
nonfinite input, invalid tolerance and degenerate geometry. Checked intersection
queries reserve `Ok(None)` for misses and orientations within their documented
angular rejection band. Checked normal constructors report a collapsed span as
`Degenerate`. Linear epsilons are finite and nonnegative; angular epsilons are
finite in `0..=1`. Unchecked APIs assume a valid epsilon when one is supplied.
An exact zero quaternion has no unit direction: ordinary normalization follows
native arithmetic, and checked normalization reports `Degenerate`.

Checked closest-point line/segment tuples enforce the same stored-parameter rule
as checked intersections: nonzero physical travel that becomes a zero parameter
is `Overflow`. Exact zero travel and clamped segment endpoints remain valid.
Scalar checked division still permits finite underflow; this extra rule belongs
to geometric result construction. Checked plane queries validate the plane
constant before an angular miss, so nonfinite input cannot become `Ok(None)`
merely because a ray or line happens to be parallel. Unused solve arithmetic
does not need to execute for a valid finite miss.

## Migration

Replace fallible arithmetic `try_*` calls with `checked_*`; conversion names
stay unchanged. Replace `try_normalize(epsilon)` with
`checked_normalize(epsilon)` and handle a `Result`. Checked line/ray constructors
accept an epsilon; ordinary `Line::new(point, direction)` and
`Ray::new(point, direction)` return the object directly. Ordinary axis-angle
construction accepts `(axis, angle)`; its checked counterpart also accepts epsilon.
Checked primitive constructors now return `Result` rather than `Option`.
Distances return a scalar directly in the ordinary tier. Checked decomposition
returns `Result<AffineParts<_>, MathError>`; the shear-free checked tuple query
returns `Result<Option<_>, MathError>`.

`cargo run --release --manifest-path tools/Cargo.toml --bin check-unrolled-codegen` verifies optimized control-flow cycles
for both matrix/vector arithmetic tiers and shared CORDIC helpers. Run
`cargo run --release --example inverse_modes -- 100000` for target-local checked
and unchecked timing comparisons; no cross-target performance ratio is promised.
