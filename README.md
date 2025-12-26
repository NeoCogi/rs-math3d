# rs-math3d

rs-math3d is a no_std-friendly 2D/3D math library focused on computer graphics and geometry.
It provides vectors, matrices, quaternions, transforms, and common geometric primitives with
utility traits for intersection and distance queries.

## Features

- Vector math (2D/3D/4D), dot/cross products, swizzles, safe normalization helpers
- Matrix math (2x2/3x3/4x4), determinants, inverses, affine fast-paths
- Quaternions for rotations and matrix conversions
- Transform helpers: translate, scale, rotate, project/unproject, lookat
- Geometric primitives: rays, planes, triangles, boxes, spheres, line segments
- Query traits for intersection and distance computations
- no_std support with an optional std feature

## Usage

Add to Cargo.toml:

```toml
[dependencies]
rs-math3d = { version = "0.10", default-features = false }
```

To enable std-backed math functions:

```toml
rs-math3d = { version = "0.10", features = ["std"] }
```

## Example

```rust
use rs_math3d::vector::Vector3;
use rs_math3d::transforms;
use rs_math3d::EPS_F32;

fn main() {
    let axis = Vector3::new(0.0f32, 1.0, 0.0);
    let rot = transforms::rotation_from_axis_angle(&axis, 1.0, EPS_F32)
        .expect("axis length too small");
    let trans = transforms::translate(Vector3::new(1.0f32, 2.0, 3.0));

    let m = trans * rot;
    let p = Vector3::new(1.0f32, 0.0, 0.0);
    let out = m * p;

    println!("{out:?}");
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
