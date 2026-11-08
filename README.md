# rs-math3d

rs-math3d is a no_std-friendly 2D/3D math library focused on computer graphics and geometry.
It provides vectors, matrices, quaternions, transforms, and common geometric primitives with
utility traits for intersection and distance queries.

## Features

- Vector math (2D/3D/4D), dot/cross products, swizzles, and real-scalar normalization helpers
- Matrix math (2x2/3x3/4x4), determinants, and real-scalar inverses and affine fast-paths
- Quaternions and transforms for floating-point and fixed-point rotations and projections
- Geometric primitives: rays, planes, triangles, boxes, spheres, line segments
- Query traits for intersection and distance computations
- Standard-library math by default, with an explicit std-free `no_std` configuration

Integer vectors, boxes, rectangles, and matrix arithmetic support discrete geometry and storage.
`RealScalar` and `RealVector` provide analytic operations for `f32`, `f64`, and signed
`Fixed<Repr, FRAC>` formats. For example, `Fixed<i32, 8>` is S23.8. Fixed analytic math
uses integer arithmetic, including square root and trigonometry.

```rust
use rs_math3d::{Fixed, RealVector, Vector3};
type F = Fixed<i32, 8>;
let scalar: F = 1.5f32.into();
let value: f64 = scalar.into();
assert_eq!(value, 1.5);
let direction = Vector3::new(scalar, F::ZERO, F::ONE);
let unit = direction.checked_normalize(F::DELTA).unwrap();
```

`From`/`Into` work in both directions for `f32` and `f64`. Float-to-fixed conversions
round nearest with ties to even and panic on non-finite or out-of-range inputs;
`try_from_f32` and `try_from_f64` return `MathError` instead. Fixed-to-float conversions
round when the float has insufficient significand precision. Raw bit construction,
ratios, rescaling, parsing, and fixed arithmetic do not use floats.

See the [fixed-point guide](docs/fixed-point.md) for format constraints, rounding,
checked geometry, conversions, and both matrix inversion modes. Reciprocal/adjugate
inversion is the default; select `InverseMode::Rows` for partial-pivot elimination.
The fixed reciprocal retains guard precision even if scalar `1/det` rounds to zero.
Run `cargo run --release --example inverse_modes` to compare both on your target,
or `cargo run --no-default-features --example fixed_scene` for a fixed-only scene.

Run `cargo run --release --example collision_bench -- 1000 7` to compare all ten
pairings of oriented boxes, spheres, capsules and triangles using `f32`, `f64`,
S23.8, S15.16 and S47.16. The [collision experiment](docs/collision.md) documents
the unchecked kernels, numerical limits, timing methodology and host results.

Run `cargo run --release --example mesh_bench -- 200 7` for an AABB tree over
Utah Teapot and Stanford Bunny triangle soups, comparing `f32`, S15.16 and S31.32
nearest-ray and mesh/mesh queries. The [triangle-soup guide](docs/triangle-soup.md)
describes construction, offline fixtures, correctness checks and measured results.
The same harness also measures tilted capsule and rotated OBB queries against
both triangle trees, including flat-reference and cross-scalar comparisons.
The [arithmetic-policy selection](docs/fixed-rounding.md) records the native
default's timings, precision and contact behavior alongside the earlier comparison.

The [const-selected CORDIC experiment](docs/cordic.md) compares
`angle.sin_cos_with::<64>()` against the default Q96/i128 implementation, with
compile-time guard precision, speed measurements and independent high-precision errors.
Both widths use one generic kernel with sealed precision backends.
Run `cargo run --release --example cordic_bench -- 1000 7` to reproduce it.

Fixed-count numeric kernels use explicit expansion for matrix dimensions, vector
reductions, geometry queries and CORDIC steps. Run
`cargo run --release --manifest-path tools/Cargo.toml --bin check-unrolled-codegen` to check optimized host control flow;
the fixed-point guide describes the remaining input-dependent loops and tradeoffs.

## Usage

Hosted applications use the default `std` backend. This includes crates marked
`#![no_std]` that run on a target with Rust's standard library and choose to
import it explicitly (for example, Linux or Windows applications):

```toml
[dependencies]
rs-math3d = "0.20.1"
```

For example, a hosted crate may keep its own `#![no_std]` attribute while
opting back into an available standard library:

```rust
#![no_std]

extern crate std;

use rs_math3d::RealScalar;
use std::f32;

pub fn quarter_turn_sine() -> f32 {
    (f32::consts::FRAC_PI_2).tsin()
}
```

The `#![no_std]` attribute alone therefore does not require `libm`; what matters
is whether the final program can link `std`. Std-free applications that need
floating-point analytics disable default features and explicitly select the
pure-Rust `libm` backend:

```toml
[dependencies]
rs-math3d = { version = "0.20.1", default-features = false, features = ["libm"] }
```

Fixed-only applications disable default features without selecting a backend:

```toml
[dependencies]
rs-math3d = { version = "0.20.1", default-features = false }
```

Unix applications that intentionally want the target's C math library may
select `system-libm` instead:

```toml
[dependencies]
rs-math3d = { version = "0.20.1", default-features = false, features = ["system-libm"] }
```

`system-libm` is supported only on Unix targets and links the native `m`
library for downstream binaries. It is not a portable freestanding backend;
use `libm` for embedded, WebAssembly, or other targets without Rust's `std`.

With default features disabled and no backend selected, fixed-point analytic math
remains available without a floating-point runtime. Floating-point analytic operations
require `std`, `libm`, or `system-libm`. If dependency feature unification enables more than one backend,
precedence is `std`, then `libm`, then `system-libm`. Consequently,
`system-libm`'s Unix restriction applies only when neither higher-precedence
backend is active.

## Arithmetic behavior

Ordinary methods and operators are unchecked by default. Use `checked_*` to
receive `MathError` for invalid inputs or unrepresentable arithmetic. Fixed
products floor-shift, division truncates toward zero, and operators wrap overflow
consistently in debug and release. Checked fixed arithmetic uses the same
quantization before range validation. Fixed division by zero can panic; unchecked float arithmetic follows
IEEE behavior. Ordinary normalization no longer substitutes identity for a zero
quaternion. Conversions retain their documented `try_*` / `From` contracts.

Matrices support `InverseMode::Reciprocal` (default) and `InverseMode::Rows`, with
independent checked and unchecked implementations. Small kernels are explicitly
expanded, and the adjugate path reuses cofactors/minors in its determinant.

Checked geometric queries distinguish arithmetic failure from a geometric miss:
`checked_intersection` returns `Result<Option<_>, MathError>`. Ordinary queries
perform no validation. Angular tolerance is dimensionless in `0..=1`; linear
tolerance uses world units. Right/left-handed basis construction is explicit.

Read the [arithmetic API contract and migration guide](docs/arithmetic.md) and
[signed fixed-point guide](docs/fixed-point.md) for format/range requirements,
rounding, inverse modes and backend-free use.

## Example

```rust
use rs_math3d::vector::Vector3;
use rs_math3d::transforms;
use rs_math3d::EPS_F32;
use core::f32::consts::PI;

fn main() {
    let axis = Vector3::new(0.0f32, 1.0, 0.0);
    let rot = transforms::rotation_from_axis_angle(&axis, PI / 4.0, EPS_F32)
        .expect("axis length too small");
    let trans = transforms::translate(Vector3::new(1.0f32, 2.0, 3.0));

    let m = trans * rot;
    let p = Vector3::new(1.0f32, 0.0, 0.0);
    let out = m * p;
    let _ = out;
}
```

## Modules

- vector: vector types and operations
- matrix: matrix types and operations
- quaternion: quaternion math for rotations
- transforms: common 3D transforms
- primitives: geometric shapes and intersection helpers
- queries: query traits and implementations
- basis: coordinate system basis helpers
- scalar: scalar traits and constants
