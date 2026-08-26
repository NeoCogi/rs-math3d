# rs-math3d

rs-math3d is a no_std-friendly 2D/3D math library focused on computer graphics and geometry.
It provides vectors, matrices, quaternions, transforms, and common geometric primitives with
utility traits for intersection and distance queries.

## Features

- Vector math (2D/3D/4D), dot/cross products, swizzles, and float-only normalization helpers
- Matrix math (2x2/3x3/4x4), determinants, and float-only inverses and affine fast-paths
- Quaternions and transforms for floating-point rotations and projections
- Geometric primitives: rays, planes, triangles, boxes, spheres, line segments
- Query traits for intersection and distance computations
- Standard-library math by default, with an explicit std-free `no_std` configuration

Integer vectors, boxes, rectangles, and matrix arithmetic are supported for discrete geometry
and storage. Operations that require fractional results, such as normalization, inversion,
quaternions, transforms, rays, planes, and geometric queries, are restricted to `f32`/`f64`.

## Usage

Hosted applications use the default `std` backend. This includes crates marked
`#![no_std]` that run on a target with Rust's standard library and choose to
import it explicitly (for example, Linux or Windows applications):

```toml
[dependencies]
rs-math3d = "0.14.0"
```

For example, a hosted crate may keep its own `#![no_std]` attribute while
opting back into an available standard library:

```rust
#![no_std]

extern crate std;

use rs_math3d::FloatScalar;
use std::f32;

pub fn quarter_turn_sine() -> f32 {
    (f32::consts::FRAC_PI_2).tsin()
}
```

The `#![no_std]` attribute alone therefore does not require `libm`; what matters
is whether the final program can link `std`. Std-free or freestanding `no_std`
applications must disable default features and explicitly select the pure-Rust
`libm` backend:

```toml
[dependencies]
rs-math3d = { version = "0.14.0", default-features = false, features = ["libm"] }
```

Unix applications that intentionally want the target's C math library may
select `system-libm` instead:

```toml
[dependencies]
rs-math3d = { version = "0.14.0", default-features = false, features = ["system-libm"] }
```

`system-libm` is supported only on Unix targets and links the native `m`
library for downstream binaries. It is not a portable freestanding backend;
use `libm` for embedded, WebAssembly, or other targets without Rust's `std`.

A build with default features disabled and no backend selected is rejected at
compile time. If dependency feature unification enables more than one backend,
precedence is `std`, then `libm`, then `system-libm`. Consequently,
`system-libm`'s Unix restriction applies only when neither higher-precedence
backend is active.

## Behavior Notes

- Orientation tolerances in 3D intersection and plane-normal routines are dimensionless and must be
  finite values in `0..=1`. Parallel tests compare `epsilon` with `|sin(theta)|`, while
  line/ray-to-plane tests compare it with `|cos(theta)|`. Temporary max-component normalization
  keeps these decisions stable when valid directions, triangle offsets, or plane-spanning vectors
  are rescaled, including magnitudes whose raw squared lengths would overflow or underflow. Zero,
  non-finite operands, and invalid tolerances are rejected.
- Each query reuses its temporary normalized representations for classification and result
  calculation. Ordinary parallel tests compare bounded squared magnitudes without another square
  root; an uncommon scaled fallback preserves the same behavior when those squares underflow.
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
  `basis_from_unit_rh`/`try_basis_from_unit_rh` return `[u, v, w]` with `u × v = w`, while
  the `_lh` variants return `[u, v, w]` with `u × v = -w`. These replace the old ambiguous
  `basis_from_unit` and `try_basis_from_unit` names.
- Point-to-plane distance uses the general `abs(n · p + d) / |n|` equation and is therefore
  invariant when all plane coefficients are scaled by the same nonzero value.
- `ParametricPlane::project` solves the 2x2 Gram system of the plane axes, so projection
  works for non-orthogonal axes as well as orthogonal ones.
- `Sphere3::new` canonicalizes the radius with `abs(radius)`.
- `Quat::mat3` and `Quat::mat4` normalize the quaternion before converting it to a matrix.
- `Plane::from_quad` and `Plane::try_from_quad` use a diagonal-derived representative plane.
  They do not validate that the four vertices are coplanar.
- `transforms::lookat` assumes `eye != dest` and an `up` vector that is not parallel to the
  view direction. Violating those preconditions yields non-finite output.
- `Tri3::barycentric_coordinates` assumes a non-degenerate triangle. Degenerate triangles
  produce non-finite coordinates.

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
