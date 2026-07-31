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
use num_traits::{One, Zero};

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
/// Converts a quaternion to its equivalent rotation matrix.
///
/// The quaternion is normalized before conversion so the result is always a
/// pure rotation matrix.
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
pub fn transform_vec3<T: FloatScalar>(m: &Matrix4<T>, v: &Vector3<T>) -> Vector3<T> {
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
pub fn project3<T: FloatScalar>(
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
pub fn unproject3<T: FloatScalar>(
    world: &Matrix4<T>,
    persp: &Matrix4<T>,
    lb: &Vector2<T>,
    rt: &Vector2<T>,
    pt: &Vector3<T>,
) -> Vector3<T> {
    let pw = *persp * *world;
    let inv = if pw.is_affine(T::epsilon()) {
        pw.inverse_affine()
    } else {
        pw.inverse()
    };
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
pub fn frustum<T: FloatScalar>(lbn: &Vector3<T>, rtf: &Vector3<T>) -> Matrix4<T> {
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
pub fn ortho4<T: FloatScalar>(left: T, right: T, bottom: T, top: T, near: T, far: T) -> Matrix4<T> {
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
///
/// # Preconditions
/// - `eye` and `dest` must not be the same point
/// - `up` must not be parallel, or nearly parallel, to the viewing direction
///
/// This routine performs unchecked normalization. Violating the preconditions
/// yields non-finite matrix components.
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

/// Components of a nonsingular affine transformation.
///
/// With column vectors, the represented matrix is reconstructed in the order
/// `T * R * H * S`, where `shear` stores `(h_xy, h_xz, h_yz)` and:
///
/// ```text
///     [1  h_xy  h_xz  0]
/// H = [0   1    h_yz  0]
///     [0   0     1    0]
///     [0   0     0    1]
/// ```
#[repr(C)]
#[derive(Clone, Copy, Debug)]
pub struct AffineParts<T: FloatScalar> {
    /// Translation applied after the linear transform.
    pub translation: Vector3<T>,
    /// Proper rotation represented by a unit quaternion.
    pub rotation: Quat<T>,
    /// Scale applied before shear and rotation.
    pub scale: Vector3<T>,
    /// Upper-triangular shear coefficients `(h_xy, h_xz, h_yz)`.
    pub shear: Vector3<T>,
}

/// Decomposes a nonsingular affine matrix into translation, rotation, shear,
/// and scale.
///
/// The returned parts reconstruct the input as `T * R * H * S`; see
/// [`AffineParts`] for the shear convention. A reflected transform is
/// canonicalized to three negative scale components and a proper rotation.
///
/// Returns `None` for projective matrices or when any QR pivot is at or below
/// [`FloatScalar::epsilon`].
pub fn decompose_affine<T: FloatScalar>(m: &Matrix4<T>) -> Option<AffineParts<T>> {
    let epsilon = T::epsilon();
    if !m.is_affine(epsilon) {
        return None;
    }

    let a0 = Vector3::new(m.col[0].x, m.col[0].y, m.col[0].z);
    let a1 = Vector3::new(m.col[1].x, m.col[1].y, m.col[1].z);
    let a2 = Vector3::new(m.col[2].x, m.col[2].y, m.col[2].z);

    // Modified Gram-Schmidt gives A = R * U. Dividing the off-diagonal
    // entries of U by its diagonal separates U into H * S.
    let scale_x = a0.length();
    if scale_x.partial_cmp(&epsilon) != Some(core::cmp::Ordering::Greater) {
        return None;
    }
    let mut r0 = a0 / scale_x;

    let projection_xy = Vector3::dot(&r0, &a1);
    let a1_orthogonal = a1 - r0 * projection_xy;
    let scale_y = a1_orthogonal.length();
    if scale_y.partial_cmp(&epsilon) != Some(core::cmp::Ordering::Greater) {
        return None;
    }
    let mut r1 = a1_orthogonal / scale_y;
    let shear_xy = projection_xy / scale_y;

    let projection_xz = Vector3::dot(&r0, &a2);
    let a2_without_x = a2 - r0 * projection_xz;
    let projection_yz = Vector3::dot(&r1, &a2_without_x);
    let a2_orthogonal = a2_without_x - r1 * projection_yz;
    let scale_z = a2_orthogonal.length();
    if scale_z.partial_cmp(&epsilon) != Some(core::cmp::Ordering::Greater) {
        return None;
    }
    let mut r2 = a2_orthogonal / scale_z;
    let shear_xz = projection_xz / scale_z;
    let shear_yz = projection_yz / scale_z;

    let mut scale = Vector3::new(scale_x, scale_y, scale_z);
    let rotation_determinant = Vector3::dot(&r0, &Vector3::cross(&r1, &r2));
    if rotation_determinant.tabs().partial_cmp(&epsilon) != Some(core::cmp::Ordering::Greater) {
        return None;
    }

    if rotation_determinant < <T as Zero>::zero() {
        // Negating both R and S leaves R * H * S unchanged. Flipping all
        // three axes preserves the crate's established reflection convention
        // while making R proper for quaternion conversion.
        r0 = -r0;
        r1 = -r1;
        r2 = -r2;
        scale = -scale;
    }

    let rotation_matrix = Matrix3::new(r0.x, r0.y, r0.z, r1.x, r1.y, r1.z, r2.x, r2.y, r2.z);

    Some(AffineParts {
        translation: Vector3::new(m.col[3].x, m.col[3].y, m.col[3].z),
        rotation: Quat::of_matrix3(&rotation_matrix),
        scale,
        shear: Vector3::new(shear_xy, shear_xz, shear_yz),
    })
}

/// Decomposes an affine, shear-free transformation into scale, rotation, and
/// translation.
///
/// This compatibility wrapper uses [`decompose_affine`] and returns `None`
/// when any absolute shear coefficient is greater than
/// [`FloatScalar::epsilon`]. Coefficients inside that inclusive tolerance band
/// are treated as numerical zero.
pub fn decompose<T: FloatScalar>(m: &Matrix4<T>) -> Option<(Vector3<T>, Quat<T>, Vector3<T>)> {
    let parts = decompose_affine(m)?;
    let epsilon = T::epsilon();
    if parts.shear.x.tabs() > epsilon
        || parts.shear.y.tabs() > epsilon
        || parts.shear.z.tabs() > epsilon
    {
        return None;
    }

    Some((parts.scale, parts.rotation, parts.translation))
}

#[cfg(test)]
mod tests {
    use super::*;

    fn assert_matrix4_close(actual: &Matrix4<f32>, expected: &Matrix4<f32>, tolerance: f32) {
        for column in 0..4 {
            assert!(
                (actual.col[column].x - expected.col[column].x).abs() <= tolerance,
                "column {} x differs",
                column
            );
            assert!(
                (actual.col[column].y - expected.col[column].y).abs() <= tolerance,
                "column {} y differs",
                column
            );
            assert!(
                (actual.col[column].z - expected.col[column].z).abs() <= tolerance,
                "column {} z differs",
                column
            );
            assert!(
                (actual.col[column].w - expected.col[column].w).abs() <= tolerance,
                "column {} w differs",
                column
            );
        }
    }

    fn shear_matrix(shear: Vector3<f32>) -> Matrix4<f32> {
        Matrix4::new(
            1.0, 0.0, 0.0, 0.0, shear.x, 1.0, 0.0, 0.0, shear.y, shear.z, 1.0, 0.0, 0.0, 0.0, 0.0,
            1.0,
        )
    }

    fn recompose_affine(parts: &AffineParts<f32>) -> Matrix4<f32> {
        translate(parts.translation)
            * rotation_from_quat(&parts.rotation)
            * shear_matrix(parts.shear)
            * scale(parts.scale)
    }

    fn assert_affine_roundtrip(matrix: &Matrix4<f32>) -> AffineParts<f32> {
        let parts = decompose_affine(matrix).expect("matrix should decompose");
        assert_matrix4_close(&recompose_affine(&parts), matrix, 0.0001);
        parts
    }

    #[test]
    pub fn test_decompose() {
        let ms = scale(Vector3::<f32>::new(4.0, 5.0, 6.0));
        let mt = translate(Vector3::<f32>::new(1.0, 2.0, 3.0));
        let q = Quat::<f32>::of_axis_angle(&Vector3::new(1.0, 1.0, 1.0), 1.0, EPS_F32)
            .expect("axis length too small");
        let mr = rotation_from_quat(&q);

        let m = mt * mr * ms;
        assert_affine_roundtrip(&m);

        let (s, r, t) = decompose(&m).expect("TRS matrix should decompose");
        let reconstructed = translate(t) * rotation_from_quat(&r) * scale(s);
        assert_matrix4_close(&reconstructed, &m, 0.0001);
    }

    #[test]
    fn test_decompose_affine_shear_roundtrips() {
        let pure_shears = [
            Vector3::new(0.25, 0.0, 0.0),
            Vector3::new(0.0, -0.5, 0.0),
            Vector3::new(0.0, 0.0, 0.75),
            Vector3::new(0.25, -0.5, 0.75),
        ];
        for shear in pure_shears {
            assert_affine_roundtrip(&shear_matrix(shear));
        }

        let translation = translate(Vector3::new(3.0, -2.0, 5.0));
        let rotation = rotation_from_axis_angle(&Vector3::new(1.0, 2.0, 3.0), 0.7, EPS_F32)
            .expect("axis should be valid");
        let matrix = translation
            * rotation
            * shear_matrix(Vector3::new(0.2, -0.3, 0.4))
            * scale(Vector3::new(2.0, 3.0, 4.0));
        assert_affine_roundtrip(&matrix);
        assert!(decompose(&matrix).is_none());
    }

    #[test]
    fn test_decompose_affine_reflection_convention() {
        let rotation = rotation_from_axis_angle(&Vector3::new(1.0, -2.0, 0.5), 1.1, EPS_F32)
            .expect("axis should be valid");
        let shear = shear_matrix(Vector3::new(0.2, -0.15, 0.35));
        let scales = [
            Vector3::new(-2.0, 3.0, 4.0),
            Vector3::new(-2.0, -3.0, 4.0),
            Vector3::new(-2.0, -3.0, -4.0),
        ];

        for input_scale in scales {
            let matrix = rotation * shear * scale(input_scale);
            let parts = assert_affine_roundtrip(&matrix);
            let determinant = parts.rotation.mat3().determinant();
            assert!((determinant - 1.0).abs() < 0.0001);

            if input_scale.x * input_scale.y * input_scale.z < 0.0 {
                assert!(parts.scale.x < 0.0);
                assert!(parts.scale.y < 0.0);
                assert!(parts.scale.z < 0.0);
            } else {
                assert!(parts.scale.x > 0.0);
                assert!(parts.scale.y > 0.0);
                assert!(parts.scale.z > 0.0);
            }
        }
    }

    #[test]
    fn test_decompose_wrapper_shear_boundary() {
        for shear in [
            Vector3::new(EPS_F32, 0.0, 0.0),
            Vector3::new(0.0, -EPS_F32, 0.0),
            Vector3::new(0.0, 0.0, EPS_F32),
        ] {
            let matrix = shear_matrix(shear);
            let (s, r, t) = decompose(&matrix).expect("epsilon-band shear should be accepted");
            let reconstructed = translate(t) * rotation_from_quat(&r) * scale(s);
            assert_matrix4_close(&reconstructed, &matrix, EPS_F32);
        }

        for shear in [
            Vector3::new(EPS_F32 * 2.0, 0.0, 0.0),
            Vector3::new(0.0, -EPS_F32 * 2.0, 0.0),
            Vector3::new(0.0, 0.0, EPS_F32 * 2.0),
        ] {
            assert!(decompose(&shear_matrix(shear)).is_none());
            assert!(decompose_affine(&shear_matrix(shear)).is_some());
        }
    }

    #[test]
    fn test_decompose_rejects_singular_and_projective_matrices() {
        let dependent_columns = Matrix4::new(
            1.0f32, 0.0, 0.0, 0.0, 2.0, 0.0, 0.0, 0.0, 0.0, 0.0, 1.0, 0.0, 0.0, 0.0, 0.0, 1.0,
        );
        assert!(decompose_affine(&dependent_columns).is_none());
        assert!(decompose(&dependent_columns).is_none());

        let near_zero_scale = scale(Vector3::new(EPS_F32, 1.0, 1.0));
        assert!(decompose_affine(&near_zero_scale).is_none());
        assert!(decompose(&near_zero_scale).is_none());

        let mut projective = Matrix4::<f32>::identity();
        projective.col[0].w = 1.0;
        assert!(decompose_affine(&projective).is_none());
        assert!(decompose(&projective).is_none());
    }

    #[test]
    fn test_rotation_from_axis_angle_zero_axis() {
        let axis = Vector3::<f32>::new(0.0, 0.0, 0.0);
        assert!(rotation_from_axis_angle(&axis, 1.0, EPS_F32).is_none());
    }

    #[test]
    fn test_project_unproject_roundtrip() {
        let world = Matrix4::<f32>::identity();
        let proj = ortho4(-1.0, 1.0, -1.0, 1.0, 0.1, 10.0);
        let lb = Vector2::new(-1.0, -1.0);
        let rt = Vector2::new(1.0, 1.0);
        let pt = Vector3::new(0.2, -0.4, 1.0);

        let screen = project3(&world, &proj, &lb, &rt, &pt);
        let out = unproject3(&world, &proj, &lb, &rt, &screen);

        assert!((out.x - pt.x).abs() < 0.001);
        assert!((out.y - pt.y).abs() < 0.001);
        assert!((out.z - pt.z).abs() < 0.001);
    }

    #[test]
    fn test_lookat_identity() {
        let eye = Vector3::new(0.0f32, 0.0, 0.0);
        let dest = Vector3::new(0.0f32, 0.0, -1.0);
        let up = Vector3::new(0.0f32, 1.0, 0.0);
        let view = lookat(&eye, &dest, &up);
        let v = Vector4::new(1.0f32, 2.0, 3.0, 1.0);
        let out = view * v;

        assert!((out.x - v.x).abs() < 0.001);
        assert!((out.y - v.y).abs() < 0.001);
        assert!((out.z - v.z).abs() < 0.001);
        assert!((out.w - v.w).abs() < 0.001);
    }
}
