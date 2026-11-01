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
//! # fn main() {
//! # #[cfg(any(feature = "std", feature = "libm", feature = "system-libm"))]
//! # {
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
//! # }
//! # }
//! ```

use crate::matrix::*;
use crate::quaternion::*;
use crate::scalar::*;
use crate::vector::*;
use crate::MathError;
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
pub fn rotation_from_quat<T: RealScalar>(q: &Quat<T>) -> Matrix4<T> {
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
pub fn rotation_from_axis_angle<T: RealScalar>(
    axis: &Vector3<T>,
    angle: T,
    epsilon: T,
) -> Option<Matrix4<T>> {
    Quat::of_axis_angle(axis, angle, epsilon).map(|q| q.mat4())
}

/// Checked transform_vec3 construction/calculation for finite represented scalars.
/// Returns an error for invalid domains, zero denominators, degenerate bases,
/// or unrepresentable intermediate/output values; performs no float conversion.
pub fn checked_transform_vec3<T: RealScalar>(
    m: &Matrix4<T>,
    v: &Vector3<T>,
) -> Result<Vector3<T>, MathError> {
    let out =
        Matrix4::checked_mul_matrix_vector(m, &Vector4::new(v.x, v.y, v.z, <T as One>::one()))?;
    Ok(Vector3::new(
        out.x.checked_div(out.w)?,
        out.y.checked_div(out.w)?,
        out.z.checked_div(out.w)?,
    ))
}

/// Transforms a 3D vector by a 4x4 matrix with perspective division.
///
/// The vector is treated as a point (w=1) and the result is divided by w.
pub fn transform_vec3<T: RealScalar>(m: &Matrix4<T>, v: &Vector3<T>) -> Vector3<T> {
    checked_transform_vec3(m, v).expect("transform_vec3 calculation failed")
}

/// Checked project3 construction/calculation for finite represented scalars.
/// Returns an error for invalid domains, zero denominators, degenerate bases,
/// or unrepresentable intermediate/output values; performs no float conversion.
pub fn checked_project3<T: RealScalar>(
    world: &Matrix4<T>,
    persp: &Matrix4<T>,
    lb: &Vector2<T>,
    rt: &Vector2<T>,
    pt: &Vector3<T>,
) -> Result<Vector3<T>, MathError> {
    let combined = Matrix4::checked_mul_matrix_matrix(persp, world)?;
    let point = checked_transform_vec3(&combined, pt)?;
    let one = <T as One>::one();
    let x =
        rt.x.checked_sub(lb.x)?
            .checked_mul(point.x.checked_add(one)?.checked_mul(T::half())?)?
            .checked_add(lb.x)?;
    let y =
        rt.y.checked_sub(lb.y)?
            .checked_mul(point.y.checked_add(one)?.checked_mul(T::half())?)?
            .checked_add(lb.y)?;
    Ok(Vector3::new(
        x,
        y,
        point.z.checked_add(one)?.checked_mul(T::half())?,
    ))
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
pub fn project3<T: RealScalar>(
    world: &Matrix4<T>,
    persp: &Matrix4<T>,
    lb: &Vector2<T>,
    rt: &Vector2<T>,
    pt: &Vector3<T>,
) -> Vector3<T> {
    checked_project3(world, persp, lb, rt, pt).expect("project3 calculation failed")
}

/// Checked unproject3 construction/calculation for finite represented scalars.
/// Returns an error for invalid domains, zero denominators, degenerate bases,
/// or unrepresentable intermediate/output values; performs no float conversion.
pub fn checked_unproject3<T: RealScalar>(
    world: &Matrix4<T>,
    persp: &Matrix4<T>,
    lb: &Vector2<T>,
    rt: &Vector2<T>,
    pt: &Vector3<T>,
) -> Result<Vector3<T>, MathError> {
    let combined = Matrix4::checked_mul_matrix_matrix(persp, world)?;
    let inverse = if combined.is_affine(T::epsilon()) {
        combined.checked_inverse_affine()?
    } else {
        combined.checked_inverse()?
    };
    let one = <T as One>::one();
    let x =
        pt.x.checked_sub(lb.x)?
            .checked_div(rt.x.checked_sub(lb.x)?)?
            .checked_mul(T::two())?
            .checked_sub(one)?;
    let y =
        pt.y.checked_sub(lb.y)?
            .checked_div(rt.y.checked_sub(lb.y)?)?
            .checked_mul(T::two())?
            .checked_sub(one)?;
    let z = pt.z.checked_mul(T::two())?.checked_sub(one)?;
    checked_transform_vec3(&inverse, &Vector3::new(x, y, z))
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
pub fn unproject3<T: RealScalar>(
    world: &Matrix4<T>,
    persp: &Matrix4<T>,
    lb: &Vector2<T>,
    rt: &Vector2<T>,
    pt: &Vector3<T>,
) -> Vector3<T> {
    checked_unproject3(world, persp, lb, rt, pt).expect("unproject3 calculation failed")
}

/// Checked frustum construction/calculation for finite represented scalars.
/// Returns an error for invalid domains, zero denominators, degenerate bases,
/// or unrepresentable intermediate/output values; performs no float conversion.
pub fn checked_frustum<T: RealScalar>(
    lbn: &Vector3<T>,
    rtf: &Vector3<T>,
) -> Result<Matrix4<T>, MathError> {
    let zero = <T as Zero>::zero();
    let one = <T as One>::one();
    let width = rtf.x.checked_sub(lbn.x)?;
    let height = rtf.y.checked_sub(lbn.y)?;
    let depth = rtf.z.checked_sub(lbn.z)?;
    let a = one.checked_add(T::two().checked_mul(lbn.x.checked_div(width)?)?)?;
    let b = one.checked_add(T::two().checked_mul(lbn.y.checked_div(height)?)?)?;
    let c = one
        .checked_add(T::two().checked_mul(lbn.z.checked_div(depth)?)?)?
        .checked_neg()?;
    let d = lbn
        .z
        .checked_mul(rtf.z.checked_div(depth)?)?
        .checked_mul(T::two())?
        .checked_neg()?;
    Ok(Matrix4::new(
        lbn.z.checked_div(width)?.checked_mul(T::two())?,
        zero,
        zero,
        zero,
        zero,
        lbn.z.checked_div(height)?.checked_mul(T::two())?,
        zero,
        zero,
        a,
        b,
        c,
        one.checked_neg()?,
        zero,
        zero,
        d,
        zero,
    ))
}

/// Creates a perspective projection matrix from frustum bounds.
///
/// # Parameters
/// - `lbn`: Left-bottom-near corner (x, y, z)
/// - `rtf`: Right-top-far corner (x, y, z)
///
/// The frustum is defined by the near and far clipping planes.
pub fn frustum<T: RealScalar>(lbn: &Vector3<T>, rtf: &Vector3<T>) -> Matrix4<T> {
    checked_frustum(lbn, rtf).expect("frustum calculation failed")
}

/// Checked ortho4 construction/calculation for finite represented scalars.
/// Returns an error for invalid domains, zero denominators, degenerate bases,
/// or unrepresentable intermediate/output values; performs no float conversion.
pub fn checked_ortho4<T: RealScalar>(
    left: T,
    right: T,
    bottom: T,
    top: T,
    near: T,
    far: T,
) -> Result<Matrix4<T>, MathError> {
    let zero = <T as Zero>::zero();
    let one = <T as One>::one();
    let width = right.checked_sub(left)?;
    let height = top.checked_sub(bottom)?;
    let depth = far.checked_sub(near)?;
    let x = one
        .checked_add(T::two().checked_mul(left.checked_div(width)?)?)?
        .checked_neg()?;
    let y = one
        .checked_add(T::two().checked_mul(bottom.checked_div(height)?)?)?
        .checked_neg()?;
    let z = one
        .checked_add(T::two().checked_mul(near.checked_div(depth)?)?)?
        .checked_neg()?;
    Ok(Matrix4::new(
        T::two().checked_div(width)?,
        zero,
        zero,
        zero,
        zero,
        T::two().checked_div(height)?,
        zero,
        zero,
        zero,
        zero,
        T::two().checked_neg()?.checked_div(depth)?,
        zero,
        x,
        y,
        z,
        one,
    ))
}

/// Creates an orthographic projection matrix.
///
/// # Parameters
/// - `left`, `right`: X-axis bounds
/// - `bottom`, `top`: Y-axis bounds
/// - `near`, `far`: Z-axis bounds (depth)
///
/// Objects maintain their size regardless of depth in orthographic projection.
pub fn ortho4<T: RealScalar>(left: T, right: T, bottom: T, top: T, near: T, far: T) -> Matrix4<T> {
    checked_ortho4(left, right, bottom, top, near, far).expect("ortho4 calculation failed")
}

/// Checked perspective construction/calculation for finite represented scalars.
/// Returns an error for invalid domains, zero denominators, degenerate bases,
/// or unrepresentable intermediate/output values; performs no float conversion.
pub fn checked_perspective<T: RealScalar>(
    fovy: T,
    aspect: T,
    near: T,
    far: T,
) -> Result<Matrix4<T>, MathError> {
    let one = <T as One>::one();
    let zero = <T as Zero>::zero();
    if !fovy.is_finite() || !aspect.is_finite() || !near.is_finite() || !far.is_finite() {
        return Err(MathError::NonFinite);
    }
    if fovy <= zero || aspect <= zero || near <= zero || far <= near {
        return Err(MathError::Domain);
    }
    let tangent = fovy.checked_mul(T::half())?.checked_tan()?;
    if tangent <= zero {
        return Err(MathError::Domain);
    }
    let f = one.checked_div(tangent)?;
    let depth = far.checked_sub(near)?;
    let a = one
        .checked_add(T::two().checked_mul(near.checked_div(depth)?)?)?
        .checked_neg()?;
    // Divide comparable quantities before multiplying physical distances.
    let b = near
        .checked_mul(far.checked_div(depth)?)?
        .checked_mul(T::two())?
        .checked_neg()?;
    Ok(Matrix4::new(
        f.checked_div(aspect)?,
        zero,
        zero,
        zero,
        zero,
        f,
        zero,
        zero,
        zero,
        zero,
        a,
        one.checked_neg()?,
        zero,
        zero,
        b,
        zero,
    ))
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
pub fn perspective<T: RealScalar>(fovy: T, aspect: T, near: T, far: T) -> Matrix4<T> {
    checked_perspective(fovy, aspect, near, far).expect("perspective calculation failed")
}

/// Checked lookat construction/calculation for finite represented scalars.
/// Returns an error for invalid domains, zero denominators, degenerate bases,
/// or unrepresentable intermediate/output values; performs no float conversion.
pub fn checked_lookat<T: RealScalar>(
    eye: &Vector3<T>,
    dest: &Vector3<T>,
    up: &Vector3<T>,
) -> Result<Matrix4<T>, MathError> {
    let zero = <T as Zero>::zero();
    let one = <T as One>::one();
    let forward = Vector3::checked_sub_vv(dest, eye)?.checked_normalize(T::epsilon())?;
    let up = up.checked_normalize(T::epsilon())?;
    let side = Vector3::checked_cross(&forward, &up)?.checked_normalize(T::epsilon())?;
    let up = Vector3::checked_cross(&side, &forward)?.checked_normalize(T::epsilon())?;
    Ok(Matrix4::new(
        side.x,
        up.x,
        forward.x.checked_neg()?,
        zero,
        side.y,
        up.y,
        forward.y.checked_neg()?,
        zero,
        side.z,
        up.z,
        forward.z.checked_neg()?,
        zero,
        Vector3::checked_dot(&side, eye)?.checked_neg()?,
        Vector3::checked_dot(&up, eye)?.checked_neg()?,
        Vector3::checked_dot(&forward, eye)?,
        one,
    ))
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
/// Panics when these preconditions or checked range requirements fail.
/// Use [`checked_lookat`] to receive the numerical error.
pub fn lookat<T: RealScalar>(eye: &Vector3<T>, dest: &Vector3<T>, up: &Vector3<T>) -> Matrix4<T> {
    checked_lookat(eye, dest, up).expect("lookat calculation failed")
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
pub struct AffineParts<T: RealScalar> {
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
/// [`RealScalar::epsilon`].
pub fn decompose_affine<T: RealScalar>(m: &Matrix4<T>) -> Option<AffineParts<T>> {
    let epsilon = T::epsilon();
    if !m.is_affine(epsilon) {
        return None;
    }

    let a0 = Vector3::new(m.col[0].x, m.col[0].y, m.col[0].z);
    let a1 = Vector3::new(m.col[1].x, m.col[1].y, m.col[1].z);
    let a2 = Vector3::new(m.col[2].x, m.col[2].y, m.col[2].z);

    // Modified Gram-Schmidt gives A = R * U. Dividing the off-diagonal
    // entries of U by its diagonal separates U into H * S.
    let scale_x = a0.checked_length().ok()?;
    if scale_x.partial_cmp(&epsilon) != Some(core::cmp::Ordering::Greater) {
        return None;
    }
    let mut r0 = Vector3::checked_div_vs(&a0, scale_x).ok()?;

    let projection_xy = Vector3::checked_dot(&r0, &a1).ok()?;
    let a1_orthogonal =
        Vector3::checked_sub_vv(&a1, &Vector3::checked_mul_vs(&r0, projection_xy).ok()?).ok()?;
    let scale_y = a1_orthogonal.checked_length().ok()?;
    if scale_y.partial_cmp(&epsilon) != Some(core::cmp::Ordering::Greater) {
        return None;
    }
    let mut r1 = Vector3::checked_div_vs(&a1_orthogonal, scale_y).ok()?;
    let shear_xy = projection_xy.checked_div(scale_y).ok()?;

    let projection_xz = Vector3::checked_dot(&r0, &a2).ok()?;
    let a2_without_x =
        Vector3::checked_sub_vv(&a2, &Vector3::checked_mul_vs(&r0, projection_xz).ok()?).ok()?;
    let projection_yz = Vector3::checked_dot(&r1, &a2_without_x).ok()?;
    let a2_orthogonal = Vector3::checked_sub_vv(
        &a2_without_x,
        &Vector3::checked_mul_vs(&r1, projection_yz).ok()?,
    )
    .ok()?;
    let scale_z = a2_orthogonal.checked_length().ok()?;
    if scale_z.partial_cmp(&epsilon) != Some(core::cmp::Ordering::Greater) {
        return None;
    }
    let mut r2 = Vector3::checked_div_vs(&a2_orthogonal, scale_z).ok()?;
    let shear_xz = projection_xz.checked_div(scale_z).ok()?;
    let shear_yz = projection_yz.checked_div(scale_z).ok()?;

    let mut scale = Vector3::new(scale_x, scale_y, scale_z);
    let rotation_determinant =
        Vector3::checked_dot(&r0, &Vector3::checked_cross(&r1, &r2).ok()?).ok()?;
    if rotation_determinant
        .checked_abs()
        .ok()?
        .partial_cmp(&epsilon)
        != Some(core::cmp::Ordering::Greater)
    {
        return None;
    }

    if rotation_determinant < <T as Zero>::zero() {
        // Negating both R and S leaves R * H * S unchanged. Flipping all
        // three axes preserves the crate's established reflection convention
        // while making R proper for quaternion conversion.
        r0 = Vector3::checked_mul_vs(&r0, -<T as One>::one()).ok()?;
        r1 = Vector3::checked_mul_vs(&r1, -<T as One>::one()).ok()?;
        r2 = Vector3::checked_mul_vs(&r2, -<T as One>::one()).ok()?;
        scale = Vector3::checked_mul_vs(&scale, -<T as One>::one()).ok()?;
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
/// [`RealScalar::epsilon`]. Coefficients inside that inclusive tolerance band
/// are treated as numerical zero.
pub fn decompose<T: RealScalar>(m: &Matrix4<T>) -> Option<(Vector3<T>, Quat<T>, Vector3<T>)> {
    let parts = decompose_affine(m)?;
    let epsilon = T::epsilon();
    if parts.shear.x.checked_abs().ok()? > epsilon
        || parts.shear.y.checked_abs().ok()? > epsilon
        || parts.shear.z.checked_abs().ok()? > epsilon
    {
        return None;
    }

    Some((parts.scale, parts.rotation, parts.translation))
}

#[cfg(all(test, any(feature = "std", feature = "libm", feature = "system-libm")))]
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
