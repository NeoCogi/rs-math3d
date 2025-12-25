// Copyright 2020-Present (c) Raja Lehtihet & Wael El Oraiby
//
// Redistribution and use in source and binary forms, with or without
// modification, are permitted provided that the following conditions are met:
//
// 1. Redistributions of source code must retain the above copyright notice,
// this list of conditions and the following disclaimer.
//
// 2. Redistributions in binary form must reproduce the above copyright notice,
// this list of conditions and the following disclaimer in the documentation
// and/or other materials provided with the distribution.
//
// 3. Neither the name of the copyright holder nor the names of its contributors
// may be used to endorse or promote products derived from this software without
// specific prior written permission.
//
// THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS "AS IS"
// AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE
// IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE
// ARE DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT HOLDER OR CONTRIBUTORS BE
// LIABLE FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR
// CONSEQUENTIAL DAMAGES (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF
// SUBSTITUTE GOODS OR SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS
// INTERRUPTION) HOWEVER CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN
// CONTRACT, STRICT LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE)
// ARISING IN ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE
// POSSIBILITY OF SUCH DAMAGE.

//! # rs-math3d
//!
//! A `no_std` 3D mathematics library for Rust, providing vectors, matrices,
//! quaternions, and geometric primitives for computer graphics applications.
//!
//! ## Features
//!
//! - **No standard library required**: Works in embedded and WASM environments
//! - **Generic types**: All types are generic over scalar types (f32, f64, i32, i64)
//! - **Column-major matrices**: Compatible with OpenGL and similar APIs
//! - **Comprehensive operations**: Full suite of vector, matrix, and quaternion operations
//! - **Geometric primitives**: Rays, planes, triangles, spheres, and more
//!
//! ## Quick Start
//!
//! ```
//! use rs_math3d::{Vector, Vector3, Matrix4, Quat, EPS_F32};
//! use rs_math3d::transforms;
//!
//! // Create vectors
//! let v1 = Vector3::new(1.0, 2.0, 3.0);
//! let v2 = Vector3::new(4.0, 5.0, 6.0);
//! let dot_product = Vector3::dot(&v1, &v2);
//!
//! // Create transformation matrices
//! let translation = transforms::translate(Vector3::new(10.0, 0.0, 0.0));
//! let rotation = transforms::rotation_from_axis_angle(
//!     &Vector3::new(0.0, 1.0, 0.0),
//!     std::f32::consts::PI / 4.0,
//!     EPS_F32,
//! ).unwrap();
//!
//! // Use quaternions for rotations
//! let q = Quat::of_axis_angle(
//!     &Vector3::new(0.0, 0.0, 1.0),
//!     std::f32::consts::PI / 2.0,
//!     EPS_F32,
//! ).unwrap();
//! let rotation_matrix = q.mat4();
//! ```
//!
//! ## Modules
//!
//! - [`vector`]: 2D, 3D, and 4D vectors with arithmetic operations
//! - [`matrix`]: 2x2, 3x3, and 4x4 matrices with linear algebra operations
//! - [`quaternion`]: Quaternions for 3D rotations
//! - [`transforms`]: Common transformation matrices (translate, rotate, scale, project)
//! - [`primitives`]: Geometric primitives (rays, planes, triangles, boxes, spheres)
//! - [`queries`]: Intersection and distance queries between primitives
//! - [`basis`]: Coordinate system basis representation
//! - [`scalar`]: Traits for generic numeric operations

#![no_std]
#![allow(unused_imports)]
#[cfg(any(test, feature = "std"))]
extern crate std;
pub mod basis;
pub mod matrix;
pub mod primitives;
/// Quaternion operations and conversions.
pub mod quaternion;
/// Intersection and distance query traits and helpers.
pub mod queries;
pub mod scalar;
pub mod transforms;
pub mod vector;

pub use basis::*;
pub use matrix::{Matrix2, Matrix3, Matrix4};
pub use primitives::*;
pub use quaternion::Quat;
pub use queries::*;
pub use scalar::{FloatScalar, Scalar, EPS_F32, EPS_F64};
pub use transforms::*;
pub use vector::{
    CrossProduct, FloatVector, Swizzle2, Swizzle3, Vector, Vector2, Vector3, Vector4,
};

/// 4-component byte color (RGBA).
pub type Color4b = Vector4<u8>;

/// 2D integer vector.
pub type Vec2i = Vector2<i32>;
/// 3D integer vector.
pub type Vec3i = Vector3<i32>;
/// 4D integer vector.
pub type Vec4i = Vector4<i32>;

/// 2D single-precision float vector.
pub type Vec2f = Vector2<f32>;
/// 3D single-precision float vector.
pub type Vec3f = Vector3<f32>;
/// 4D single-precision float vector.
pub type Vec4f = Vector4<f32>;

/// 2D double-precision float vector.
pub type Vec2d = Vector2<f64>;
/// 3D double-precision float vector.
pub type Vec3d = Vector3<f64>;
/// 4D double-precision float vector.
pub type Vec4d = Vector4<f64>;

/// Single-precision quaternion.
pub type Quatf = Quat<f32>;
/// Double-precision quaternion.
pub type Quatd = Quat<f64>;

/// 2x2 single-precision matrix.
pub type Mat2f = Matrix2<f32>;
/// 3x3 single-precision matrix.
pub type Mat3f = Matrix3<f32>;
/// 4x4 single-precision matrix.
pub type Mat4f = Matrix4<f32>;

/// 2x2 double-precision matrix.
pub type Mat2d = Matrix2<f64>;
/// 3x3 double-precision matrix.
pub type Mat3d = Matrix3<f64>;
/// 4x4 double-precision matrix.
pub type Mat4d = Matrix4<f64>;

/// Integer rectangle.
pub type Recti = Rect<i32>;
/// Single-precision rectangle.
pub type Rectf = Rect<f32>;
/// Double-precision rectangle.
pub type Rectd = Rect<f64>;

/// Integer dimensions.
pub type Dimensioni = Dimension<i32>;
/// Single-precision dimensions.
pub type Dimensionf = Dimension<f32>;

/// 2D single-precision line.
pub type Line2f = Line<f32, Vec2f>;
/// 2D double-precision line.
pub type Line2d = Line<f64, Vec2d>;
/// 3D single-precision line.
pub type Line3f = Line<f32, Vec3f>;
/// 3D double-precision line.
pub type Line3d = Line<f64, Vec3d>;

/// 2D single-precision line segment.
pub type Segment2f = Segment<f32, Vec2f>;
/// 2D double-precision line segment.
pub type Segment2d = Segment<f64, Vec2d>;
/// 3D single-precision line segment.
pub type Segment3f = Segment<f32, Vec3f>;
/// 3D double-precision line segment.
pub type Segment3d = Segment<f64, Vec3d>;

/// Single-precision plane.
pub type Planef = Plane<f32>;
/// Double-precision plane.
pub type Planed = Plane<f64>;

/// 3D single-precision ray.
pub type Ray3f = Ray<f32, Vec3f>;
/// 3D double-precision ray.
pub type Ray3d = Ray<f64, Vec3d>;

/// 3D single-precision axis-aligned bounding box.
pub type Box3f = Box3<f32>;
/// 3D double-precision axis-aligned bounding box.
pub type Box3d = Box3<f64>;

/// Single-precision basis.
pub type Basisf = Basis<f32>;
/// Double-precision basis.
pub type Basisd = Basis<f64>;

/// Single-precision parametric plane.
pub type ParametricPlanef = ParametricPlane<f32>;
/// Double-precision parametric plane.
pub type ParametricPlaned = ParametricPlane<f64>;

/// Creates an RGBA color from byte components.
pub fn color4b(r: u8, g: u8, b: u8, a: u8) -> Color4b {
    Color4b {
        x: r,
        y: g,
        z: b,
        w: a,
    }
}
