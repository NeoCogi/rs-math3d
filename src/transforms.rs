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
//! 3D transformation functions for computer graphics.
//!
//! This module provides functions to create and manipulate transformation
//! matrices commonly used in 3D graphics, including translation, rotation,
//! scaling, and projection matrices.
//!
//! # Examples
//!
//! ```
//! use rs_math3d::transforms;
//! use rs_math3d::vector::Vector3;
//! 
//! // Create a translation matrix
//! let translation = transforms::translate(Vector3::new(10.0, 5.0, 0.0));
//! 
//! // Create a perspective projection matrix
//! let projection = transforms::perspective(
//!     45.0f32.to_radians(),  // Field of view
//!     16.0 / 9.0,            // Aspect ratio
//!     0.1,                   // Near plane
//!     100.0                  // Far plane
//! );
//! ```

use crate::matrix::*;
use crate::quaternion::*;
use crate::scalar::*;
use crate::vector::*;
use num_traits::{Zero, One};

/// Creates a 4x4 translation matrix.
///
/// The translation matrix has the form:
/// ```text
/// [1  0  0  tx]
/// [0  1  0  ty]
/// [0  0  1  tz]
/// [0  0  0  1 ]
/// ```
///
/// # Parameters
/// - `trans`: Translation vector (tx, ty, tz)
pub fn translate<T: Scalar>(trans: Vector3<T>) -> Matrix4<T> {
    Matrix4::new(
        <T as One>::one(),
        <T as Zero>::zero(),
        <T as Zero>::zero(),
        <T as Zero>::zero(),
        <T as Zero>::zero(),
        <T as One>::one(),
        <T as Zero>::zero(),
        <T as Zero>::zero(),
        <T as Zero>::zero(),
        <T as Zero>::zero(),
        <T as One>::one(),
        <T as Zero>::zero(),
        trans.x,
        trans.y,
        trans.z,
        <T as One>::one(),
    )
}

/// Creates a 4x4 scaling matrix.
///
/// The scaling matrix has the form:
/// ```text
/// [sx 0  0  0]
/// [0  sy 0  0]
/// [0  0  sz 0]
/// [0  0  0  1]
/// ```
///
/// # Parameters
/// - `scale`: Scale factors for each axis
pub fn scale<T: Scalar>(scale: Vector3<T>) -> Matrix4<T> {
    Matrix4::new(
        scale.x,
        <T as Zero>::zero(),
        <T as Zero>::zero(),
        <T as Zero>::zero(),
        <T as Zero>::zero(),
        scale.y,
        <T as Zero>::zero(),
        <T as Zero>::zero(),
        <T as Zero>::zero(),
        <T as Zero>::zero(),
        scale.z,
        <T as Zero>::zero(),
        <T as Zero>::zero(),
        <T as Zero>::zero(),
        <T as Zero>::zero(),
        <T as One>::one(),
    )
}

/// Creates a 4x4 rotation matrix from a quaternion.
///
/// Converts a unit quaternion to its equivalent rotation matrix.
pub fn rotation_from_quat<T: FloatScalar>(q: &Quat<T>) -> Matrix4<T> {
    Quat::mat4(q)
}

/// Creates a 4x4 rotation matrix from an axis and angle.
///
/// # Parameters
/// - `axis`: The rotation axis (will be normalized)
/// - `angle`: The rotation angle in radians
/// - `epsilon`: Minimum axis length to treat as valid
///
/// # Returns
/// - `Some(matrix)` for a valid axis
/// - `None` if the axis length is too small
pub fn rotation_from_axis_angle<T: FloatScalar>(
    axis: &Vector3<T>,
    angle: T,
    epsilon: T,
) -> Option<Matrix4<T>> {
    Quat::of_axis_angle(axis, angle, epsilon).map(|q| q.mat4())
}

/// Transforms a 3D vector by a 4x4 matrix with perspective division.
///
/// The vector is treated as a point (w=1) and the result is divided by w.
pub fn transform_vec3<T: Scalar>(m: &Matrix4<T>, v: &Vector3<T>) -> Vector3<T> {
    let v4 = Vector4::new(v.x, v.y, v.z, <T as One>::one());
    let vout = *m * v4;
    Vector3::new(vout.x / vout.w, vout.y / vout.w, vout.z / vout.w)
}

/// Projects a 3D point to screen coordinates.
///
/// # Parameters
/// - `world`: World transformation matrix
/// - `persp`: Perspective projection matrix
/// - `lb`: Screen left-bottom corner
/// - `rt`: Screen right-top corner
/// - `pt`: Point to project
///
/// # Returns
/// Screen coordinates with z in \[0,1\] (0=near, 1=far)
pub fn project3<T: Scalar>(
    world: &Matrix4<T>,
    persp: &Matrix4<T>,
    lb: &Vector2<T>,
    rt: &Vector2<T>,
    pt: &Vector3<T>,
) -> Vector3<T> {
    let inp = Vector4::new(pt.x, pt.y, pt.z, <T as One>::one());
    let pw = *persp * *world;
    let mut out = pw * inp;

    out.x /= out.w;
    out.y /= out.w;
    out.z /= out.w;

    let out_x = lb.x + ((rt.x - lb.x) * (out.x + <T as One>::one()) * T::half());
    let out_y = lb.y + ((rt.y - lb.y) * (out.y + <T as One>::one()) * T::half());
    let out_z = (out.z + <T as One>::one()) * T::half();
    Vector3::new(out_x, out_y, out_z)
}

/// Unprojects screen coordinates back to 3D world space.
///
/// # Parameters
/// - `world`: World transformation matrix
/// - `persp`: Perspective projection matrix
/// - `lb`: Screen left-bottom corner
/// - `rt`: Screen right-top corner
/// - `pt`: Screen point with z-depth
///
/// # Returns
/// The corresponding 3D world point
pub fn unproject3<T: Scalar>(
    world: &Matrix4<T>,
    persp: &Matrix4<T>,
    lb: &Vector2<T>,
    rt: &Vector2<T>,
    pt: &Vector3<T>,
) -> Vector3<T> {
    let pw = *persp * *world;
    let inv = pw.inverse();
    let in_x = (T::two() * (pt.x - lb.x) / (rt.x - lb.x)) - <T as One>::one();
    let in_y = (T::two() * (pt.y - lb.y) / (rt.y - lb.y)) - <T as One>::one();
    let in_z = (T::two() * pt.z) - <T as One>::one();
    let in_w = <T as One>::one();
    let inp = Vector4::new(in_x, in_y, in_z, in_w);
    let out = inv * inp;
    let out4 = out / out.w;
    Vector3::new(out4.x, out4.y, out4.z)
}

/// Creates a perspective projection matrix from frustum bounds.
///
/// # Parameters
/// - `lbn`: Left-bottom-near corner (x, y, z)
/// - `rtf`: Right-top-far corner (x, y, z)
///
/// The frustum is defined by the near and far clipping planes.
pub fn frustum<T: Scalar>(lbn: &Vector3<T>, rtf: &Vector3<T>) -> Matrix4<T> {
    let width = rtf.x - lbn.x;
    let height = rtf.y - lbn.y;
    let depth = rtf.z - lbn.z;
    let a = (rtf.x + lbn.x) / width;
    let b = (rtf.y + lbn.y) / height;
    let c = -(rtf.z + lbn.z) / depth;
    let d = -(T::two() * rtf.z * lbn.z) / depth;

    Matrix4::new(
        T::two() * lbn.z / width,
        <T as Zero>::zero(),
        <T as Zero>::zero(),
        <T as Zero>::zero(),
        <T as Zero>::zero(),
        T::two() * lbn.z / height,
        <T as Zero>::zero(),
        <T as Zero>::zero(),
        a,
        b,
        c,
        -<T as One>::one(),
        <T as Zero>::zero(),
        <T as Zero>::zero(),
        d,
        <T as Zero>::zero(),
    )
}

/// Creates an orthographic projection matrix.
///
/// # Parameters
/// - `left`, `right`: X-axis bounds
/// - `bottom`, `top`: Y-axis bounds
/// - `near`, `far`: Z-axis bounds (depth)
///
/// Objects maintain their size regardless of depth in orthographic projection.
pub fn ortho4<T: Scalar>(left: T, right: T, bottom: T, top: T, near: T, far: T) -> Matrix4<T> {
    let width = right - left;
    let height = top - bottom;
    let depth = far - near;
    let r00 = T::two() / width;
    let r11 = T::two() / height;
    let r22 = -T::two() / depth;
    let r03 = -(right + left) / width;
    let r13 = -(top + bottom) / height;
    let r23 = -(far + near) / depth;
    Matrix4::new(
        r00,
        <T as Zero>::zero(),
        <T as Zero>::zero(),
        <T as Zero>::zero(),
        <T as Zero>::zero(),
        r11,
        <T as Zero>::zero(),
        <T as Zero>::zero(),
        <T as Zero>::zero(),
        <T as Zero>::zero(),
        r22,
        <T as Zero>::zero(),
        r03,
        r13,
        r23,
        <T as One>::one(),
    )
}

/// Creates a perspective projection matrix.
///
/// # Parameters
/// - `fovy`: Field of view angle in radians (vertical)
/// - `aspect`: Aspect ratio (width / height)
/// - `near`: Near clipping plane distance
/// - `far`: Far clipping plane distance
///
/// Uses the standard OpenGL perspective projection formula.
pub fn perspective<T: FloatScalar>(fovy: T, aspect: T, near: T, far: T) -> Matrix4<T> {
    let f = <T as One>::one() / T::ttan(fovy * T::half());
    let denom = near - far;
    let a = (far + near) / denom;
    let b = (T::two() * far * near) / denom;

    Matrix4::new(
        f / aspect,
        <T as Zero>::zero(),
        <T as Zero>::zero(),
        <T as Zero>::zero(),
        <T as Zero>::zero(),
        f,
        <T as Zero>::zero(),
        <T as Zero>::zero(),
        <T as Zero>::zero(),
        <T as Zero>::zero(),
        a,
        -<T as One>::one(),
        <T as Zero>::zero(),
        <T as Zero>::zero(),
        b,
        <T as Zero>::zero(),
    )
}

/// Creates a view matrix looking from eye position to target.
///
/// # Parameters
/// - `eye`: Camera position
/// - `dest`: Target position to look at
/// - `up`: Up vector (typically (0, 1, 0))
///
/// The resulting matrix transforms from world space to view space.
pub fn lookat<T: FloatScalar>(eye: &Vector3<T>, dest: &Vector3<T>, up: &Vector3<T>) -> Matrix4<T> {
    let f = Vector3::normalize(&(*dest - *eye));
    let s = Vector3::normalize(&Vector3::cross(&f, up));
    let u = Vector3::normalize(&Vector3::cross(&s, &f));

    let trans = translate(-*eye);

    let m = Matrix4::new(
        s.x,
        u.x,
        -f.x,
        <T as Zero>::zero(),
        s.y,
        u.y,
        -f.y,
        <T as Zero>::zero(),
        s.z,
        u.z,
        -f.z,
        <T as Zero>::zero(),
        <T as Zero>::zero(),
        <T as Zero>::zero(),
        <T as Zero>::zero(),
        <T as One>::one(),
    );
    m * trans
}

/// Decomposes a transformation matrix into scale, rotation, and translation.
///
/// # Returns
/// - `Some((scale, rotation, translation))` if successful
/// - `None` if the matrix is singular or has zero scale
///
/// # Note
/// This assumes the matrix represents a valid affine transformation.
pub fn decompose<T: FloatScalar>(m: &Matrix4<T>) -> Option<(Vector3<T>, Quat<T>, Vector3<T>)> {
    let mut col0 = Vector3::new(m.col[0].x, m.col[0].y, m.col[0].z);
    let mut col1 = Vector3::new(m.col[1].x, m.col[1].y, m.col[1].z);
    let mut col2 = Vector3::new(m.col[2].x, m.col[2].y, m.col[2].z);
    let det = m.determinant();

    // the scale needs to be tested
    let mut scale = Vector3::new(
        Vector3::length(&col0),
        Vector3::length(&col1),
        Vector3::length(&col2),
    );
    let trans = Vector3::new(m.col[3].x, m.col[3].y, m.col[3].z);

    if det < <T as Zero>::zero() {
        scale = -scale;
    }

    if scale.x != <T as Zero>::zero() {
        col0 = col0 / scale.x;
    } else {
        return Option::None;
    }

    if scale.y != <T as Zero>::zero() {
        col1 = col1 / scale.y;
    } else {
        return Option::None;
    }

    if scale.z != <T as Zero>::zero() {
        col2 = col2 / scale.z;
    } else {
        return Option::None;
    }

    let rot_matrix = Matrix3::new(
        col0.x, col0.y, col0.z, col1.x, col1.y, col1.z, col2.x, col2.y, col2.z,
    );

    let rot = Quat::of_matrix3(&rot_matrix);

    Some((scale, rot, trans))
}

#[cfg(test)]
mod tests {
    use super::*;
    #[test]
    pub fn test_decompose() {
        let ms = scale(Vector3::<f32>::new(4.0, 5.0, 6.0));
        let mt = translate(Vector3::<f32>::new(1.0, 2.0, 3.0));
        let q = Quat::<f32>::of_axis_angle(&Vector3::new(1.0, 1.0, 1.0), 1.0, EPS_F32)
            .expect("axis length too small");
        let mr = rotation_from_quat(&q);

        let m = mt * mr * ms;

        let v = decompose(&m);
        match v {
            None => assert_eq!(1, 2),
            Some((s, r, t)) => {
                assert_eq!((s.x - 4.0) < f32::epsilon(), true);
                assert_eq!((s.y - 5.0) < f32::epsilon(), true);
                assert_eq!((s.z - 6.0) < f32::epsilon(), true);

                assert_eq!((q.x - r.x) < f32::epsilon(), true);
                assert_eq!((q.y - r.y) < f32::epsilon(), true);
                assert_eq!((q.z - r.z) < f32::epsilon(), true);
                assert_eq!((q.w - r.w) < f32::epsilon(), true);

                assert_eq!((t.x - 1.0) < f32::epsilon(), true);
                assert_eq!((t.y - 2.0) < f32::epsilon(), true);
                assert_eq!((t.z - 3.0) < f32::epsilon(), true);
            }
        }
    }

    #[test]
    fn test_rotation_from_axis_angle_zero_axis() {
        let axis = Vector3::<f32>::new(0.0, 0.0, 0.0);
        assert!(rotation_from_axis_angle(&axis, 1.0, EPS_F32).is_none());
    }
}
