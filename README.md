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

Fixed-count numeric kernels use explicit expansion for matrix dimensions, vector
reductions, geometry queries and CORDIC steps. Run
`python3 tools/check_unrolled_codegen.py` to check optimized host control flow;
the fixed-point guide describes the remaining input-dependent loops and tradeoffs.

## Usage

Hosted applications use the default `std` backend. This includes crates marked
`#![no_std]` that run on a target with Rust's standard library and choose to
import it explicitly (for example, Linux or Windows applications):

```toml
[dependencies]
rs-math3d = "0.20.0"
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
rs-math3d = { version = "0.20.0", default-features = false, features = ["libm"] }
```

Fixed-only applications disable default features without selecting a backend:

```toml
[dependencies]
rs-math3d = { version = "0.20.0", default-features = false }
```

Unix applications that intentionally want the target's C math library may
select `system-libm` instead:

```toml
[dependencies]
rs-math3d = { version = "0.20.0", default-features = false, features = ["system-libm"] }
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

## Behavior Notes

- Orientation tolerances in 3D intersection and plane-normal routines are dimensionless and must be
  finite values in `0..=1`. Parallel tests compare `epsilon` with `|sin(theta)|`, while
  line/ray-to-plane tests compare it with `|cos(theta)|`. Temporary max-component normalization
  keeps these decisions stable when valid directions, triangle offsets, or plane-spanning vectors
  are rescaled, including magnitudes whose raw squared lengths would overflow or underflow. Zero,
  non-finite operands, and invalid tolerances are rejected. Internal angular classifiers keep
  within-tolerance, outside-tolerance, and invalid states distinct. A shared checked-query gate
  proceeds only for the outside-tolerance state; within-tolerance and invalid states both map to
  `None` without being conflated internally.
- Each query reuses its temporary normalized representations for classification and result
  calculation. Angular classifiers compare scalar-specific stable norms; fixed norms compare widened
  squared integers so subquantum squares do not erase a nonzero tolerance.
- Rescaling an already represented `Line`'s stored direction does not change its line/plane or
  line/triangle intersection point, but the returned `t` remains tied to the parameterization
  `p + d * t`; multiplying `d` by a nonzero scale therefore divides every representable `t` by
  the same scale. `Ray` intersection points likewise do not depend on a positive rescaling of the
  stored direction.
  Plane intersections solve physical travel with a temporary unit direction, reconstruct the
  point from that bounded representation, and convert travel back to the stored line's `t` only
  when the API returns a line parameter. A negative direction scale reverses a ray rather than
  merely reparameterizing it. If a line's original-direction parameter would overflow, or if a
  mathematically nonzero parameter would underflow to zero, the query returns `None` instead of
  returning a parameter that cannot reconstruct the reported point.
- `Ray`/`Tri3` intersection is a true ray query: hits behind the ray origin are rejected.
  Use the corresponding `Line`/`Tri3` intersection when you want the infinite-line result.
- `transforms::decompose_affine` represents nonsingular affine matrices as translation,
  proper rotation, `(xy, xz, yz)` upper-triangular shear, and scale in `T * R * H * S`
  order. The older `transforms::decompose` tuple is a shear-free wrapper and returns `None`
  when any shear coefficient exceeds the scalar epsilon.
- `Quat::default()` is the identity rotation. `Quat::normalize` also maps an exact zero
  quaternion to identity; use `Quat::try_normalize` when invalid or near-zero input must be
  detected.
- Orthonormal basis construction makes handedness explicit:
  `basis_from_unit_rh`/`checked_basis_from_unit_rh` return `[u, v, w]` with `u × v = w`, while
  the `_lh` variants return `[u, v, w]` with `u × v = -w`. These replace the old ambiguous
  `basis_from_unit` and `checked_basis_from_unit` names.
- Point-to-plane distance uses the general `abs(n · p + d) / |n|` equation and is therefore
  invariant when all plane coefficients are scaled by the same nonzero value.
- `ParametricPlane::project` solves the 2x2 Gram system of the plane axes, so projection
  works for non-orthogonal axes as well as orthogonal ones.
- `Sphere3::new` canonicalizes the radius with `abs(radius)`.
- `Quat::mat3` and `Quat::mat4` normalize the quaternion before converting it to a matrix.
- `Plane::from_quad` and `Plane::checked_from_quad` use a diagonal-derived representative plane.
  They do not validate that the four vertices are coplanar.
- `transforms::lookat` assumes `eye != dest` and an `up` vector that is not parallel to the
  view direction. The convenience function panics on invalid input; `checked_lookat` reports a numerical error.
- `Tri3::barycentric_coordinates` assumes a non-degenerate triangle. Degenerate triangles
  panic in the convenience method; `checked_barycentric_coordinates` returns `MathError::Degenerate`.

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
