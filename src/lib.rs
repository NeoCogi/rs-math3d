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

#![no_std]
#![allow(unused_imports)]
pub mod basis;
pub mod matrix;
pub mod primitives;
pub mod quaternion;
pub mod queries;
pub mod scalar;
pub mod transforms;
pub mod vector;

pub use basis::*;
pub use matrix::{Matrix2, Matrix3, Matrix4};
pub use primitives::*;
pub use quaternion::Quat;
pub use queries::*;
pub use scalar::{FloatScalar, Scalar};
pub use transforms::*;
pub use vector::{
    CrossProduct, FloatVector, Swizzle2, Swizzle3, Vector, Vector2, Vector3, Vector4,
};

pub type Color4b = Vector4<u8>;

pub type Vec2i = Vector2<i32>;
pub type Vec3i = Vector3<i32>;
pub type Vec4i = Vector4<i32>;

pub type Vec2f = Vector2<f32>;
pub type Vec3f = Vector3<f32>;
pub type Vec4f = Vector4<f32>;

pub type Vec2d = Vector2<f64>;
pub type Vec3d = Vector3<f64>;
pub type Vec4d = Vector4<f64>;

pub type Quatf = Quat<f32>;
pub type Quatd = Quat<f64>;

pub type Mat2f = Matrix2<f32>;
pub type Mat3f = Matrix3<f32>;
pub type Mat4f = Matrix4<f32>;

pub type Mat2d = Matrix2<f64>;
pub type Mat3d = Matrix3<f64>;
pub type Mat4d = Matrix4<f64>;

pub type Recti = Rect<i32>;
pub type Rectf = Rect<f32>;
pub type Rectd = Rect<f64>;

pub type Dimensioni = Dimension<i32>;
pub type Dimensionf = Dimension<f32>;

pub type Line2f = Line<f32, Vec2f>;
pub type Line2d = Line<f64, Vec2d>;
pub type Line3f = Line<f32, Vec3f>;
pub type Line3d = Line<f64, Vec3d>;

pub type Segment2f = Segment<f32, Vec2f>;
pub type Segment2d = Segment<f64, Vec2d>;
pub type Segment3f = Segment<f32, Vec3f>;
pub type Segment3d = Segment<f64, Vec3d>;

pub type Planef = Plane<f32>;
pub type Planed = Plane<f64>;

pub type Ray3f = Ray<f32, Vec3f>;
pub type Ray3d = Ray<f64, Vec3d>;

pub type Box3f = Box3<f32>;
pub type Box3d = Box3<f64>;

pub type Basisf = Basis<f32>;
pub type Basisd = Basis<f64>;

pub type ParametricPlanef = ParametricPlane<f32>;
pub type ParametricPlaned = ParametricPlane<f64>;

pub fn color4b(r: u8, g: u8, b: u8, a: u8) -> Color4b {
    Color4b {
        x: r,
        y: g,
        z: b,
        w: a,
    }
}

