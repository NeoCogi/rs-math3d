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
- `no_std` support with an optional `std` feature

Integer vectors, boxes, rectangles, and matrix arithmetic are supported for discrete geometry
and storage. Operations that require fractional results, such as normalization, inversion,
quaternions, transforms, rays, planes, and geometric queries, are restricted to `f32`/`f64`.

## Usage

Add to Cargo.toml:

```toml
[dependencies]
rs-math3d = { version = "0.12.0", default-features = false }
```

Select one math backend:

```toml
rs-math3d = { version = "0.12.0", default-features = false, features = ["std"] }
rs-math3d = { version = "0.12.0", default-features = false, features = ["libm"] }
rs-math3d = { version = "0.12.0", default-features = false, features = ["system-libm"] }
```

When no math backend feature is selected, normal library builds fall back to `system-libm`.
Test builds without an explicit backend use `std`.
If more than one backend feature is enabled, precedence is `std`, then `libm`, then
`system-libm`.

## Behavior Notes

- `Ray`/`Tri3` intersection is a true ray query: hits behind the ray origin are rejected.
  Use the corresponding `Line`/`Tri3` intersection when you want the infinite-line result.
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
