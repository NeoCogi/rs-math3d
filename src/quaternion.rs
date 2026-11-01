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
//! Quaternion math for rotations and orientation.

use crate::matrix::{Matrix3, Matrix4};
use crate::scalar::*;
use crate::vector::{RealVector, Vector, Vector3};
use crate::MathError;
use core::ops::*;
use num_traits::{One, Zero};

#[repr(C)]
#[derive(Copy, Clone, Debug)]
/// A quaternion representing rotation or orientation.
///
/// The default value is the identity rotation `(0, 0, 0, 1)`.
pub struct Quat<T: Scalar> {
    /// X component (imaginary).
    pub x: T,
    /// Y component (imaginary).
    pub y: T,
    /// Z component (imaginary).
    pub z: T,
    /// W component (real).
    pub w: T,
}

impl<T: Scalar> Default for Quat<T> {
    fn default() -> Self {
        Self {
            x: <T as Zero>::zero(),
            y: <T as Zero>::zero(),
            z: <T as Zero>::zero(),
            w: <T as One>::one(),
        }
    }
}

impl<T: RealScalar> Quat<T> {
    #[allow(clippy::too_many_arguments)]
    fn from_rotation_matrix(
        m00: T,
        m01: T,
        m02: T,
        m10: T,
        m11: T,
        m12: T,
        m20: T,
        m21: T,
        m22: T,
    ) -> Self {
        let q = T::quarter();
        let one = <T as One>::one();
        let t = T::sum_products(&[
            (one, q, false),
            (m00, q, false),
            (m11, q, false),
            (m22, q, false),
        ]);
        if t > T::epsilon() {
            let w = t.tsqrt();
            Self::new(
                (m21 - m12) * q / w,
                (m02 - m20) * q / w,
                (m10 - m01) * q / w,
                w,
            )
        } else if m00 > m11 && m00 > m22 {
            let x = T::sum_products(&[
                (one, q, false),
                (m00, q, false),
                (m11, q, true),
                (m22, q, true),
            ])
            .tsqrt();
            Self::new(
                x,
                (m10 + m01) * q / x,
                (m20 + m02) * q / x,
                (m21 - m12) * q / x,
            )
        } else if m11 > m22 {
            let y = T::sum_products(&[
                (one, q, false),
                (m11, q, false),
                (m00, q, true),
                (m22, q, true),
            ])
            .tsqrt();
            Self::new(
                (m01 + m10) * q / y,
                y,
                (m21 + m12) * q / y,
                (m02 - m20) * q / y,
            )
        } else {
            let z = T::sum_products(&[
                (one, q, false),
                (m22, q, false),
                (m00, q, true),
                (m11, q, true),
            ])
            .tsqrt();
            Self::new(
                (m02 + m20) * q / z,
                (m12 + m21) * q / z,
                z,
                (m10 - m01) * q / z,
            )
        }
    }

    /// Returns the identity quaternion (no rotation).
    pub fn identity() -> Self {
        Self {
            x: <T as Zero>::zero(),
            y: <T as Zero>::zero(),
            z: <T as Zero>::zero(),
            w: <T as One>::one(),
        }
    }
    /// Creates a quaternion from components.
    pub fn new(x: T, y: T, z: T, w: T) -> Self {
        Self { x, y, z, w }
    }

    /// Computes the dot product of two quaternions.
    ///
    /// ```text
    /// q₁ · q₂ = x₁x₂ + y₁y₂ + z₁z₂ + w₁w₂
    /// ```
    pub fn dot(l: &Self, r: &Self) -> T {
        T::sum_products(&[
            (l.x, r.x, false),
            (l.y, r.y, false),
            (l.z, r.z, false),
            (l.w, r.w, false),
        ])
    }

    /// Computes the ordinary scalar norm without validating its domain or range.
    pub fn length(&self) -> T {
        T::norm(&[self.x, self.y, self.z, self.w])
    }

    /// Returns the conjugate of the quaternion.
    ///
    /// The conjugate inverts the rotation:
    /// ```text
    /// q* = -xi - yj - zk + w
    /// ```
    pub fn conjugate(q: &Self) -> Self {
        Self::new(-q.x, -q.y, -q.z, q.w)
    }

    /// Normalizes with ordinary arithmetic. Requires a nonzero quaternion and
    /// representable norm/division intermediates; zero does not become identity.
    pub fn normalize(q: &Self) -> Self {
        Self::divf(q, q.length())
    }

    /// Negates all components of the quaternion.
    ///
    /// Note: -q and q represent the same rotation (double cover property).
    pub fn neg(q: &Self) -> Self {
        Self::new(-q.x, -q.y, -q.z, -q.w)
    }
    /// Adds two quaternions component-wise.
    pub fn add(l: &Self, r: &Self) -> Self {
        Self::new(l.x + r.x, l.y + r.y, l.z + r.z, l.w + r.w)
    }
    /// Subtracts two quaternions component-wise.
    pub fn sub(l: &Self, r: &Self) -> Self {
        Self::new(l.x - r.x, l.y - r.y, l.z - r.z, l.w - r.w)
    }
    /// Multiplies two quaternions (Hamilton product).
    pub fn mul(l: &Self, r: &Self) -> Self {
        let x = T::sum_products(&[
            (l.w, r.x, false),
            (l.x, r.w, false),
            (l.y, r.z, false),
            (l.z, r.y, true),
        ]);
        let y = T::sum_products(&[
            (l.w, r.y, false),
            (l.x, r.z, true),
            (l.y, r.w, false),
            (l.z, r.x, false),
        ]);
        let z = T::sum_products(&[
            (l.w, r.z, false),
            (l.x, r.y, false),
            (l.y, r.x, true),
            (l.z, r.w, false),
        ]);
        let w = T::sum_products(&[
            (l.w, r.w, false),
            (l.x, r.x, true),
            (l.y, r.y, true),
            (l.z, r.z, true),
        ]);
        Self::new(x, y, z, w)
    }

    /// Multiplies a quaternion by a scalar.
    pub fn mulf(l: &Self, r: T) -> Self {
        Self::new(l.x * r, l.y * r, l.z * r, l.w * r)
    }
    /// Multiplies a scalar by a quaternion.
    pub fn fmul(l: T, r: &Self) -> Self {
        Self::new(l * r.x, l * r.y, l * r.z, l * r.w)
    }
    /// Divides a quaternion by a scalar.
    pub fn divf(l: &Self, r: T) -> Self {
        Self::new(l.x / r, l.y / r, l.z / r, l.w / r)
    }
    /// Divides a scalar by a quaternion component-wise.
    pub fn fdiv(l: T, r: &Self) -> Self {
        Self::new(l / r.x, l / r.y, l / r.z, l / r.w)
    }

    /// Returns `conjugate(q) / dot(q,q)` with unchecked arithmetic.
    /// Requires a nonzero represented squared norm and representable intermediates.
    pub fn inverse(q: &Self) -> Self {
        Self::divf(&Self::conjugate(q), Self::dot(q, q))
    }

    /// Converts the quaternion to a 3x3 rotation matrix.
    ///
    /// The quaternion is normalized before conversion so the output remains a
    /// pure rotation even when the input is not already unit length.
    ///
    /// The conversion formula:
    /// ```text
    /// R = I + 2s(q×) + 2(q×)²
    /// ```
    /// where q× is the cross-product matrix of the vector part.
    pub fn mat3(&self) -> Matrix3<T> {
        let q = Self::normalize(self);
        let xx = q.x * q.x;
        let xy = q.x * q.y;
        let xz = q.x * q.z;
        let xw = q.x * q.w;
        let yy = q.y * q.y;
        let yz = q.y * q.z;
        let yw = q.y * q.w;
        let zz = q.z * q.z;
        let zw = q.z * q.w;

        let m00 = <T as One>::one() - T::two() * (yy + zz);
        let m10 = T::two() * (xy + zw);
        let m20 = T::two() * (xz - yw);
        let m01 = T::two() * (xy - zw);
        let m11 = <T as One>::one() - T::two() * (xx + zz);
        let m21 = T::two() * (yz + xw);
        let m02 = T::two() * (xz + yw);
        let m12 = T::two() * (yz - xw);
        let m22 = <T as One>::one() - T::two() * (xx + yy);

        Matrix3::new(m00, m10, m20, m01, m11, m21, m02, m12, m22)
    }

    /// Converts the quaternion to a 4x4 transformation matrix.
    ///
    /// The resulting matrix represents a pure rotation with no translation.
    /// The bottom-right element is 1 for homogeneous coordinates.
    ///
    /// The quaternion is normalized before conversion so the output remains a
    /// pure rotation even when the input is not already unit length.
    pub fn mat4(&self) -> Matrix4<T> {
        let q = Self::normalize(self);
        let xx = q.x * q.x;
        let xy = q.x * q.y;
        let xz = q.x * q.z;
        let xw = q.x * q.w;
        let yy = q.y * q.y;
        let yz = q.y * q.z;
        let yw = q.y * q.w;
        let zz = q.z * q.z;
        let zw = q.z * q.w;

        let m00 = <T as One>::one() - T::two() * (yy + zz);
        let m10 = T::two() * (xy + zw);
        let m20 = T::two() * (xz - yw);
        let m01 = T::two() * (xy - zw);
        let m11 = <T as One>::one() - T::two() * (xx + zz);
        let m21 = T::two() * (yz + xw);
        let m02 = T::two() * (xz + yw);
        let m12 = T::two() * (yz - xw);
        let m22 = <T as One>::one() - T::two() * (xx + yy);

        Matrix4::new(
            m00,
            m10,
            m20,
            <T as Zero>::zero(),
            m01,
            m11,
            m21,
            <T as Zero>::zero(),
            m02,
            m12,
            m22,
            <T as Zero>::zero(),
            <T as Zero>::zero(),
            <T as Zero>::zero(),
            <T as Zero>::zero(),
            <T as One>::one(),
        )
    }

    /// Converts the quaternion to axis-angle representation.
    ///
    /// Returns:
    /// - `axis`: The normalized rotation axis
    /// - `angle`: The rotation angle in radians
    ///
    /// For a quaternion q = cos(θ/2) + sin(θ/2)(xi + yj + zk):
    /// - axis = normalize(x, y, z)
    /// - angle = 2 * acos(w)
    ///
    /// If the rotation is near-zero, returns a default axis of (1, 0, 0).
    pub fn to_axis_angle(&self) -> (Vector3<T>, T) {
        let nq = Self::normalize(self);
        let one = <T as One>::one();
        let mut cos_a = nq.w;
        cos_a = T::min(one, T::max(-one, cos_a));
        let angle = T::tacos(cos_a) * T::two();
        let axis_vec = Vector3::new(nq.x, nq.y, nq.z);
        let axis_len = axis_vec.length();
        if axis_len <= T::epsilon() {
            (
                Vector3::new(one, <T as Zero>::zero(), <T as Zero>::zero()),
                angle,
            )
        } else {
            (axis_vec / axis_len, angle)
        }
    }

    /// Creates a quaternion from a 3x3 rotation matrix.
    pub fn of_matrix3(m: &Matrix3<T>) -> Self {
        Self::from_rotation_matrix(
            m.col[0].x, m.col[1].x, m.col[2].x, m.col[0].y, m.col[1].y, m.col[2].y, m.col[0].z,
            m.col[1].z, m.col[2].z,
        )
    }

    /// Creates a quaternion from a 4x4 rotation matrix.
    pub fn of_matrix4(m: &Matrix4<T>) -> Self {
        Self::from_rotation_matrix(
            m.col[0].x, m.col[1].x, m.col[2].x, m.col[0].y, m.col[1].y, m.col[2].y, m.col[0].z,
            m.col[1].z, m.col[2].z,
        )
    }

    /// Quaternion length using stable checked scalar norm arithmetic.
    pub fn checked_length(&self) -> Result<T, MathError> {
        T::checked_norm(&[self.x, self.y, self.z, self.w])
    }
    /// Checked four-component dot product.
    pub fn checked_dot(l: &Self, r: &Self) -> Result<T, MathError> {
        T::checked_sum_products(&[
            (l.x, r.x, false),
            (l.y, r.y, false),
            (l.z, r.z, false),
            (l.w, r.w, false),
        ])
    }
    /// Checked conjugate, including signed storage minima.
    pub fn checked_conjugate(q: &Self) -> Result<Self, MathError> {
        Ok(Self::new(
            q.x.checked_neg()?,
            q.y.checked_neg()?,
            q.z.checked_neg()?,
            q.w.checked_add(<T as Zero>::zero())?,
        ))
    }
    /// Checked inverse. A zero norm reports degeneracy.
    pub fn checked_inverse(q: &Self) -> Result<Self, MathError> {
        let norm = Self::checked_dot(q, q)?;
        if norm == <T as Zero>::zero() {
            return Err(MathError::Degenerate);
        }
        Self::checked_divf(&Self::checked_conjugate(q)?, norm)
    }
    /// Componentwise checked addition.
    pub fn checked_add(l: &Self, r: &Self) -> Result<Self, MathError> {
        Ok(Self::new(
            l.x.checked_add(r.x)?,
            l.y.checked_add(r.y)?,
            l.z.checked_add(r.z)?,
            l.w.checked_add(r.w)?,
        ))
    }

    /// Subtracts quaternion components with scalar input and range checks.
    pub fn checked_sub(l: &Self, r: &Self) -> Result<Self, MathError> {
        Ok(Self::new(
            l.x.checked_sub(r.x)?,
            l.y.checked_sub(r.y)?,
            l.z.checked_sub(r.z)?,
            l.w.checked_sub(r.w)?,
        ))
    }
    /// Componentwise checked negation.
    pub fn checked_neg(q: &Self) -> Result<Self, MathError> {
        Ok(Self::new(
            q.x.checked_neg()?,
            q.y.checked_neg()?,
            q.z.checked_neg()?,
            q.w.checked_neg()?,
        ))
    }
    /// Checked quaternion/scalar mul operation.
    pub fn checked_mulf(l: &Self, r: T) -> Result<Self, MathError> {
        Ok(Self::new(
            l.x.checked_mul(r)?,
            l.y.checked_mul(r)?,
            l.z.checked_mul(r)?,
            l.w.checked_mul(r)?,
        ))
    }
    /// Checked quaternion/scalar div operation.
    pub fn checked_divf(l: &Self, r: T) -> Result<Self, MathError> {
        Ok(Self::new(
            l.x.checked_div(r)?,
            l.y.checked_div(r)?,
            l.z.checked_div(r)?,
            l.w.checked_div(r)?,
        ))
    }
    /// Checked scalar/quaternion multiplication.
    pub fn checked_fmul(l: T, r: &Self) -> Result<Self, MathError> {
        Self::checked_mulf(r, l)
    }
    /// Checked componentwise scalar/quaternion division.
    pub fn checked_fdiv(l: T, r: &Self) -> Result<Self, MathError> {
        Ok(Self::new(
            l.checked_div(r.x)?,
            l.checked_div(r.y)?,
            l.checked_div(r.z)?,
            l.checked_div(r.w)?,
        ))
    }
    /// Checked Hamilton product with one quantization per output component.
    pub fn checked_mul(l: &Self, r: &Self) -> Result<Self, MathError> {
        let x = T::checked_sum_products(&[
            (l.w, r.x, false),
            (l.x, r.w, false),
            (l.y, r.z, false),
            (l.z, r.y, true),
        ])?;
        let y = T::checked_sum_products(&[
            (l.w, r.y, false),
            (l.x, r.z, true),
            (l.y, r.w, false),
            (l.z, r.x, false),
        ])?;
        let z = T::checked_sum_products(&[
            (l.w, r.z, false),
            (l.x, r.y, false),
            (l.y, r.x, true),
            (l.z, r.w, false),
        ])?;
        let w = T::checked_sum_products(&[
            (l.w, r.w, false),
            (l.x, r.x, true),
            (l.y, r.y, true),
            (l.z, r.z, true),
        ])?;
        Ok(Self::new(x, y, z, w))
    }
    /// Rotation matrix with checked normalization and arithmetic.
    pub fn checked_mat3(&self) -> Result<Matrix3<T>, MathError> {
        let q = Self::checked_normalize(self, <T as Zero>::zero())?;
        let one = <T as One>::one();
        let two = T::two();
        let xx = q.x.checked_mul(q.x)?;
        let xy = q.x.checked_mul(q.y)?;
        let xz = q.x.checked_mul(q.z)?;
        let xw = q.x.checked_mul(q.w)?;
        let yy = q.y.checked_mul(q.y)?;
        let yz = q.y.checked_mul(q.z)?;
        let yw = q.y.checked_mul(q.w)?;
        let zz = q.z.checked_mul(q.z)?;
        let zw = q.z.checked_mul(q.w)?;
        Ok(Matrix3::new(
            one.checked_sub(two.checked_mul(yy.checked_add(zz)?)?)?,
            two.checked_mul(xy.checked_add(zw)?)?,
            two.checked_mul(xz.checked_sub(yw)?)?,
            two.checked_mul(xy.checked_sub(zw)?)?,
            one.checked_sub(two.checked_mul(xx.checked_add(zz)?)?)?,
            two.checked_mul(yz.checked_add(xw)?)?,
            two.checked_mul(xz.checked_add(yw)?)?,
            two.checked_mul(yz.checked_sub(xw)?)?,
            one.checked_sub(two.checked_mul(xx.checked_add(yy)?)?)?,
        ))
    }
    /// Homogeneous checked rotation matrix.
    pub fn checked_mat4(&self) -> Result<Matrix4<T>, MathError> {
        let m = self.checked_mat3()?;
        let z = <T as Zero>::zero();
        let o = <T as One>::one();
        Ok(Matrix4::new(
            m.col[0].x, m.col[0].y, m.col[0].z, z, m.col[1].x, m.col[1].y, m.col[1].z, z,
            m.col[2].x, m.col[2].y, m.col[2].z, z, z, z, z, o,
        ))
    }
    /// Checked axis-angle extraction; an identity rotation uses the X axis.
    pub fn checked_to_axis_angle(&self) -> Result<(Vector3<T>, T), MathError> {
        let q = Self::checked_normalize(self, <T as Zero>::zero())?;
        let one = <T as One>::one();
        let angle = T::min(one, T::max(-one, q.w))
            .checked_acos()?
            .checked_mul(T::two())?;
        let axis = Vector3::new(q.x, q.y, q.z);
        let len = axis.checked_length()?;
        Ok((
            if len <= T::epsilon() {
                Vector3::new(one, <T as Zero>::zero(), <T as Zero>::zero())
            } else {
                Vector3::checked_div_vs(&axis, len)?
            },
            angle,
        ))
    }
    /// Creates a quaternion from a valid rotation matrix with checked arithmetic.
    /// Validates finite coefficients and arithmetic, not orthogonality.
    pub fn checked_of_matrix3(m: &Matrix3<T>) -> Result<Self, MathError> {
        let q = T::quarter();
        let one = <T as One>::one();
        let m00 = m.col[0].x;
        if !m00.is_finite() {
            return Err(MathError::NonFinite);
        }
        let m01 = m.col[1].x;
        if !m01.is_finite() {
            return Err(MathError::NonFinite);
        }
        let m02 = m.col[2].x;
        if !m02.is_finite() {
            return Err(MathError::NonFinite);
        }
        let m10 = m.col[0].y;
        if !m10.is_finite() {
            return Err(MathError::NonFinite);
        }
        let m11 = m.col[1].y;
        if !m11.is_finite() {
            return Err(MathError::NonFinite);
        }
        let m12 = m.col[2].y;
        if !m12.is_finite() {
            return Err(MathError::NonFinite);
        }
        let m20 = m.col[0].z;
        if !m20.is_finite() {
            return Err(MathError::NonFinite);
        }
        let m21 = m.col[1].z;
        if !m21.is_finite() {
            return Err(MathError::NonFinite);
        }
        let m22 = m.col[2].z;
        if !m22.is_finite() {
            return Err(MathError::NonFinite);
        }
        let t = T::checked_sum_products(&[
            (one, q, false),
            (m00, q, false),
            (m11, q, false),
            (m22, q, false),
        ])?;
        if t > T::epsilon() {
            let w = t.checked_sqrt()?;
            Ok(Self::new(
                m21.checked_sub(m12)?.checked_mul(q)?.checked_div(w)?,
                m02.checked_sub(m20)?.checked_mul(q)?.checked_div(w)?,
                m10.checked_sub(m01)?.checked_mul(q)?.checked_div(w)?,
                w,
            ))
        } else if m00 > m11 && m00 > m22 {
            let v = T::checked_sum_products(&[
                (one, q, false),
                (m00, q, false),
                (m11, q, true),
                (m22, q, true),
            ])?
            .checked_sqrt()?;
            Ok(Self::new(
                v,
                m10.checked_add(m01)?.checked_mul(q)?.checked_div(v)?,
                m20.checked_add(m02)?.checked_mul(q)?.checked_div(v)?,
                m21.checked_sub(m12)?.checked_mul(q)?.checked_div(v)?,
            ))
        } else if m11 > m22 {
            let v = T::checked_sum_products(&[
                (one, q, false),
                (m00, q, true),
                (m11, q, false),
                (m22, q, true),
            ])?
            .checked_sqrt()?;
            Ok(Self::new(
                m01.checked_add(m10)?.checked_mul(q)?.checked_div(v)?,
                v,
                m21.checked_add(m12)?.checked_mul(q)?.checked_div(v)?,
                m02.checked_sub(m20)?.checked_mul(q)?.checked_div(v)?,
            ))
        } else {
            let v = T::checked_sum_products(&[
                (one, q, false),
                (m00, q, true),
                (m11, q, true),
                (m22, q, false),
            ])?
            .checked_sqrt()?;
            Ok(Self::new(
                m02.checked_add(m20)?.checked_mul(q)?.checked_div(v)?,
                m12.checked_add(m21)?.checked_mul(q)?.checked_div(v)?,
                v,
                m10.checked_sub(m01)?.checked_mul(q)?.checked_div(v)?,
            ))
        }
    }
    /// Checked rotation extraction from the upper-left 3x3 coefficients.
    pub fn checked_of_matrix4(m: &Matrix4<T>) -> Result<Self, MathError> {
        Self::checked_of_matrix3(&Matrix3::new(
            m.col[0].x, m.col[0].y, m.col[0].z, m.col[1].x, m.col[1].y, m.col[1].z, m.col[2].x,
            m.col[2].y, m.col[2].z,
        ))
    }

    /// Constructs an axis-angle quaternion without validation.
    /// Requires a nonzero axis, finite radian angle and representable intermediates.
    pub fn of_axis_angle(axis: &Vector3<T>, angle: T) -> Self {
        let unit = axis.normalize();
        let h = angle * T::half();
        let v = unit * h.tsin();
        Self::new(v.x, v.y, v.z, h.tcos())
    }
    /// Normalizes without conflating degenerate input with arithmetic failure.
    pub fn checked_normalize(q: &Self, epsilon: T) -> Result<Self, MathError> {
        let v = crate::Vector4::new(q.x, q.y, q.z, q.w).checked_normalize(epsilon)?;
        Ok(Self::new(v.x, v.y, v.z, v.w))
    }

    /// Constructs an axis-angle quaternion with checked arithmetic.
    /// Validates a finite nonnegative axis-length epsilon, nonzero axis and finite angle.
    /// Returns normalization, analytic domain or range errors explicitly.
    pub fn checked_of_axis_angle(
        axis: &Vector3<T>,
        angle: T,
        epsilon: T,
    ) -> Result<Self, MathError> {
        let unit = axis.checked_normalize(epsilon)?;
        let half_angle = angle.checked_mul(T::half())?;
        let sin = half_angle.checked_sin()?;
        let cos = half_angle.checked_cos()?;
        let vector = Vector3::checked_mul_vs(&unit, sin)?;
        Ok(Self::new(vector.x, vector.y, vector.z, cos))
    }
}

impl<T: RealScalar> Add for Quat<T> {
    type Output = Quat<T>;
    fn add(self, rhs: Self) -> Self {
        Self::add(&self, &rhs)
    }
}

impl<T: RealScalar> Sub for Quat<T> {
    type Output = Quat<T>;
    fn sub(self, rhs: Self) -> Self {
        Self::sub(&self, &rhs)
    }
}

impl<T: RealScalar> Mul for Quat<T> {
    type Output = Quat<T>;
    fn mul(self, rhs: Self) -> Self {
        Self::mul(&self, &rhs)
    }
}

impl<T: RealScalar> Div<T> for Quat<T> {
    type Output = Quat<T>;
    fn div(self, t: T) -> Self {
        Self::divf(&self, t)
    }
}

impl<T: RealScalar> Mul<T> for Quat<T> {
    type Output = Quat<T>;
    fn mul(self, t: T) -> Self {
        Self::mulf(&self, t)
    }
}

#[cfg(any(feature = "std", feature = "libm", feature = "system-libm"))]
macro_rules! impl_scalar {
    ($Op:ident, $op:ident, $Opop:ident, $t:ident) => {
        impl $Op<Quat<$t>> for $t {
            type Output = Quat<$t>;
            fn $op(self, q: Quat<$t>) -> Quat<$t> {
                Quat::$Opop(self, &q)
            }
        }
    };
}

#[cfg(any(feature = "std", feature = "libm", feature = "system-libm"))]
impl_scalar!(Div, div, fdiv, f32);
#[cfg(any(feature = "std", feature = "libm", feature = "system-libm"))]
impl_scalar!(Mul, mul, fmul, f32);
#[cfg(any(feature = "std", feature = "libm", feature = "system-libm"))]
impl_scalar!(Div, div, fdiv, f64);
#[cfg(any(feature = "std", feature = "libm", feature = "system-libm"))]
impl_scalar!(Mul, mul, fmul, f64);

#[cfg(all(test, any(feature = "std", feature = "libm", feature = "system-libm")))]
mod tests {
    use crate::MathError;
    use core::f32::consts::{FRAC_PI_2, FRAC_PI_4, PI};

    use crate::matrix::*;
    use crate::scalar::*;
    use crate::vector::*;
    use crate::Quat;

    fn quat_axis_angle_f32(axis: &Vector3<f32>, angle: f32) -> Quat<f32> {
        Quat::checked_of_axis_angle(axis, angle, EPS_F32)
            .ok()
            .expect("axis length too small")
    }

    fn quat_axis_angle_f64(axis: &Vector3<f64>, angle: f64) -> Quat<f64> {
        Quat::checked_of_axis_angle(axis, angle, EPS_F64)
            .ok()
            .expect("axis length too small")
    }

    fn mat3_axis_angle_f32(axis: &Vector3<f32>, angle: f32) -> Matrix3<f32> {
        Matrix3::checked_of_axis_angle(axis, angle, EPS_F32)
            .ok()
            .expect("axis length too small")
    }

    fn assert_vec3_close_f32(actual: Vector3<f32>, expected: Vector3<f32>) {
        assert!((actual - expected).length() < f32::epsilon());
    }

    fn assert_vec3_close_f64(actual: Vector3<f64>, expected: Vector3<f64>) {
        assert!((actual - expected).length() < f64::epsilon());
    }

    fn assert_scalar_close_f32(actual: f32, expected: f32) {
        assert!((actual - expected).abs() < f32::epsilon());
    }

    fn assert_scalar_close_f64(actual: f64, expected: f64) {
        assert!((actual - expected).abs() < f64::epsilon());
    }

    #[test]
    fn test_identity() {
        let q = Quat::<f32>::identity();
        let m = q.mat3();

        let x_axis = m.col[0];
        let y_axis = m.col[1];
        let z_axis = m.col[2];

        assert_vec3_close_f32(x_axis, Vector3::new(1.0, 0.0, 0.0));
        assert_vec3_close_f32(y_axis, Vector3::new(0.0, 1.0, 0.0));
        assert_vec3_close_f32(z_axis, Vector3::new(0.0, 0.0, 1.0));
    }

    #[test]
    fn test_default_is_identity() {
        let default_f32 = Quat::<f32>::default();
        assert_eq!(default_f32.x, 0.0);
        assert_eq!(default_f32.y, 0.0);
        assert_eq!(default_f32.z, 0.0);
        assert_eq!(default_f32.w, 1.0);

        let default_f64 = Quat::<f64>::default();
        assert_eq!(default_f64.x, 0.0);
        assert_eq!(default_f64.y, 0.0);
        assert_eq!(default_f64.z, 0.0);
        assert_eq!(default_f64.w, 1.0);
    }

    #[test]
    fn test_identity_conjugate() {
        let q = Quat::conjugate(&Quat::<f32>::identity());
        let m = q.mat3();

        let x_axis = m.col[0];
        let y_axis = m.col[1];
        let z_axis = m.col[2];

        assert_vec3_close_f32(x_axis, Vector3::new(1.0, 0.0, 0.0));
        assert_vec3_close_f32(y_axis, Vector3::new(0.0, 1.0, 0.0));
        assert_vec3_close_f32(z_axis, Vector3::new(0.0, 0.0, 1.0));
    }

    #[test]
    fn test_rot_180() {
        let v = Vector3::<f32>::new(1.0, 0.0, 0.0);
        let a = PI;

        let q = quat_axis_angle_f32(&v, a);
        let m = q.mat3();

        let x_axis = m.col[0];
        let y_axis = m.col[1];
        let z_axis = m.col[2];

        assert_vec3_close_f32(x_axis, Vector3::new(1.0, 0.0, 0.0));
        assert_vec3_close_f32(y_axis, Vector3::new(0.0, -1.0, 0.0));
        assert_vec3_close_f32(z_axis, Vector3::new(0.0, 0.0, -1.0));

        let m2 = mat3_axis_angle_f32(&v, a);

        let x2_axis = m2.col[0];
        let y2_axis = m2.col[1];
        let z2_axis = m2.col[2];

        assert_vec3_close_f32(x_axis, x2_axis);
        assert_vec3_close_f32(y_axis, y2_axis);
        assert_vec3_close_f32(z_axis, z2_axis);
    }

    #[test]
    fn test_rot_90() {
        let v = Vector3::<f32>::new(1.0, 0.0, 0.0);
        let a = FRAC_PI_2;

        let q = quat_axis_angle_f32(&v, a);
        let m = q.mat3();

        let x_axis = m.col[0];
        let y_axis = m.col[1];
        let z_axis = m.col[2];

        assert_vec3_close_f32(x_axis, Vector3::new(1.0, 0.0, 0.0));
        assert_vec3_close_f32(y_axis, Vector3::new(0.0, 0.0, 1.0));
        assert_vec3_close_f32(z_axis, Vector3::new(0.0, -1.0, 0.0));

        let m2 = mat3_axis_angle_f32(&v, a);

        let x2_axis = m2.col[0];
        let y2_axis = m2.col[1];
        let z2_axis = m2.col[2];

        assert_vec3_close_f32(x_axis, x2_axis);
        assert_vec3_close_f32(y_axis, y2_axis);
        assert_vec3_close_f32(z_axis, z2_axis);
    }

    #[test]
    fn test_rot_45() {
        let v = Vector3::<f32>::new(1.0, 0.0, 0.0);
        let a = FRAC_PI_4;

        let q = quat_axis_angle_f32(&v, a);
        let m = q.mat3();

        let x_axis = m.col[0];
        let y_axis = m.col[1];
        let z_axis = m.col[2];

        assert_vec3_close_f32(x_axis, Vector3::new(1.0, 0.0, 0.0));
        assert_vec3_close_f32(
            y_axis,
            Vector3::new(0.0, f32::sqrt(2.0) / 2.0, f32::sqrt(2.0) / 2.0),
        );
        assert_vec3_close_f32(
            z_axis,
            Vector3::new(0.0, -f32::sqrt(2.0) / 2.0, f32::sqrt(2.0) / 2.0),
        );

        let m2 = mat3_axis_angle_f32(&v, a);

        let x2_axis = m2.col[0];
        let y2_axis = m2.col[1];
        let z2_axis = m2.col[2];

        assert_vec3_close_f32(x_axis, x2_axis);
        assert_vec3_close_f32(y_axis, y2_axis);
        assert_vec3_close_f32(z_axis, z2_axis);
    }

    #[test]
    fn test_rot_composed_45() {
        let v = Vector3::<f32>::new(1.0, 0.0, 0.0);
        let a_u = FRAC_PI_4;
        let a_f = PI;

        let q_u = quat_axis_angle_f32(&v, a_u);
        let q_f = quat_axis_angle_f32(&v, a_f);

        let m = q_f.mat3();

        let x_axis = m.col[0];
        let y_axis = m.col[1];
        let z_axis = m.col[2];

        let q_t = q_u * q_u * q_u * q_u;
        let m2 = q_t.mat3();

        let x2_axis = m2.col[0];
        let y2_axis = m2.col[1];
        let z2_axis = m2.col[2];

        assert_vec3_close_f32(x_axis, x2_axis);
        assert_vec3_close_f32(y_axis, y2_axis);
        assert_vec3_close_f32(z_axis, z2_axis);
    }

    #[test]
    fn test_rot_composed_120() {
        let v = Vector3::<f32>::new(1.0, 0.0, 0.0);
        let a_u1 = FRAC_PI_2;
        let a_u2 = PI / 6.0;
        let a_f = FRAC_PI_2 + PI / 6.0;

        let q_u1 = quat_axis_angle_f32(&v, a_u1);
        let q_u2 = quat_axis_angle_f32(&v, a_u2);
        let q_f = quat_axis_angle_f32(&v, a_f);

        let m = q_f.mat3();

        let x_axis = m.col[0];
        let y_axis = m.col[1];
        let z_axis = m.col[2];

        let q_t = q_u2 * q_u1;
        let m2 = q_t.mat3();

        let x2_axis = m2.col[0];
        let y2_axis = m2.col[1];
        let z2_axis = m2.col[2];

        assert_vec3_close_f32(x_axis, x2_axis);
        assert_vec3_close_f32(y_axis, y2_axis);
        assert_vec3_close_f32(z_axis, z2_axis);
    }

    #[test]
    fn test_rot_x_rot_y() {
        let vx = Vector3::<f32>::new(1.0, 0.0, 0.0);
        let vy = Vector3::<f32>::new(0.0, 1.0, 0.0);
        let vz = Vector3::<f32>::new(0.0, 0.0, 1.0);

        let v4x = Vector4::<f32>::new(1.0, 0.0, 0.0, 1.0);
        let v4y = Vector4::<f32>::new(0.0, 1.0, 0.0, 1.0);
        let v4z = Vector4::<f32>::new(0.0, 0.0, 1.0, 1.0);

        let a = FRAC_PI_2;

        let q_ux = quat_axis_angle_f32(&vx, a);
        let q_uy = quat_axis_angle_f32(&vy, a);
        let q_uz = quat_axis_angle_f32(&vz, a);

        let mx = q_ux.mat3();
        let m4x = q_ux.mat4();

        let px = (m4x * v4x).xyz();
        let py = (m4x * v4y).xyz();
        let pz = (m4x * v4z).xyz();

        let x_axis = mx.col[0];
        let y_axis = mx.col[1];
        let z_axis = mx.col[2];

        assert_vec3_close_f32(x_axis, vx);
        assert_vec3_close_f32(y_axis, vz);
        assert_vec3_close_f32(z_axis, -vy);

        assert_vec3_close_f32(px, vx);
        assert_vec3_close_f32(py, vz);
        assert_vec3_close_f32(pz, -vy);

        // rotate x, then y
        let q_yx = q_uy * q_ux;
        let myx = q_yx.mat3();
        let m4yx = q_yx.mat4();

        let px = (m4yx * v4x).xyz();
        let py = (m4yx * v4y).xyz();
        let pz = (m4yx * v4z).xyz();

        let x_axis = myx.col[0];
        let y_axis = myx.col[1];
        let z_axis = myx.col[2];

        assert_vec3_close_f32(x_axis, -vz);
        assert_vec3_close_f32(y_axis, vx);
        assert_vec3_close_f32(z_axis, -vy);

        assert_vec3_close_f32(px, -vz);
        assert_vec3_close_f32(py, vx);
        assert_vec3_close_f32(pz, -vy);

        // rotate x, then y, then z
        let q_zyx = q_uz * q_uy * q_ux;
        let mzyx = q_zyx.mat3();

        let m4zyx = q_zyx.mat4();

        let px = (m4zyx * v4x).xyz();
        let py = (m4zyx * v4y).xyz();
        let pz = (m4zyx * v4z).xyz();

        let x_axis = mzyx.col[0];
        let y_axis = mzyx.col[1];
        let z_axis = mzyx.col[2];

        assert_vec3_close_f32(x_axis, -vz);
        assert_vec3_close_f32(y_axis, vy);
        assert_vec3_close_f32(z_axis, vx);

        assert_vec3_close_f32(px, -vz);
        assert_vec3_close_f32(py, vy);
        assert_vec3_close_f32(pz, vx);
    }

    #[test]
    fn test_axis_angle_f32() {
        let v = Vector3::normalize(&Vector3::<f32>::new(1.0, 2.0, 3.0));
        let q0 = quat_axis_angle_f32(&v, 3.0);
        let (v2, a) = q0.to_axis_angle();
        assert_vec3_close_f32(v, v2);
        assert_scalar_close_f32(a, 3.0);
    }

    #[test]
    fn test_axis_angle_f64() {
        let v = Vector3::normalize(&Vector3::<f64>::new(1.0, 2.0, 3.0));
        let q0 = quat_axis_angle_f64(&v, 3.0);
        let (v2, a) = q0.to_axis_angle();
        assert_vec3_close_f64(v, v2);
        assert_scalar_close_f64(a, 3.0);
    }

    #[test]
    fn test_axis_angle_zero_axis_none() {
        let axis = Vector3::<f32>::new(0.0, 0.0, 0.0);
        assert!(Quat::checked_of_axis_angle(&axis, 1.0, EPS_F32)
            .ok()
            .is_none());
    }

    #[test]
    fn test_quaternion_normalization() {
        // Test normalizing a non-unit quaternion
        let q = Quat::<f32>::new(1.0, 2.0, 3.0, 4.0);
        let nq = Quat::normalize(&q);
        let len = nq.length();
        assert!((len - 1.0).abs() < f32::epsilon());
        let checked = Quat::checked_normalize(&q, EPS_F32).expect("quaternion should normalize");
        assert!((checked.length() - 1.0).abs() < f32::epsilon());

        // Ordinary normalization follows IEEE arithmetic; validation is explicit.
        let q_zero = Quat::<f32>::new(0.0, 0.0, 0.0, 0.0);
        let nq_zero = Quat::normalize(&q_zero);
        assert!(nq_zero.x.is_nan());
        assert!(nq_zero.y.is_nan());
        assert!(nq_zero.z.is_nan());
        assert!(nq_zero.w.is_nan());
        assert!(Quat::checked_normalize(&q_zero, EPS_F32).ok().is_none());

        // Test already normalized quaternion
        let q_unit = Quat::<f32>::identity();
        let nq_unit = Quat::normalize(&q_unit);
        assert!((nq_unit.length() - 1.0).abs() < f32::epsilon());
    }

    #[test]
    fn test_zero_quaternion_checked_conversions_error() {
        let zero = Quat::<f32>::new(0.0, 0.0, 0.0, 0.0);
        assert!(matches!(zero.checked_mat3(), Err(MathError::Degenerate)));
        assert!(matches!(zero.checked_mat4(), Err(MathError::Degenerate)));
        assert!(matches!(
            zero.checked_to_axis_angle(),
            Err(MathError::Degenerate)
        ));
        assert!(zero.mat3().col[0].x.is_nan());
    }

    #[test]
    fn test_quaternion_inverse() {
        let q = quat_axis_angle_f32(&Vector3::new(0.0, 1.0, 0.0), FRAC_PI_2);
        let q_inv = Quat::inverse(&q);
        let product = q * q_inv;

        // Quaternion times its inverse should be identity
        assert!((product.x).abs() < 0.001);
        assert!((product.y).abs() < 0.001);
        assert!((product.z).abs() < 0.001);
        assert!((product.w - 1.0).abs() < 0.001);
    }

    #[test]
    fn test_quaternion_inverse_non_unit() {
        let q = Quat::<f32>::new(1.0, 2.0, 3.0, 4.0);
        let q_inv = Quat::inverse(&q);
        let product = q * q_inv;

        assert!(product.x.abs() < 0.001);
        assert!(product.y.abs() < 0.001);
        assert!(product.z.abs() < 0.001);
        assert!((product.w - 1.0).abs() < 0.001);
    }

    #[test]
    fn test_quaternion_multiplication() {
        // Test multiplication properties
        let q1 = quat_axis_angle_f32(&Vector3::new(1.0, 0.0, 0.0), 0.5);
        let q2 = quat_axis_angle_f32(&Vector3::new(0.0, 1.0, 0.0), 0.5);
        let q3 = quat_axis_angle_f32(&Vector3::new(0.0, 0.0, 1.0), 0.5);

        // Test associativity: (q1 * q2) * q3 == q1 * (q2 * q3)
        let left = (q1 * q2) * q3;
        let right = q1 * (q2 * q3);
        assert!((left.x - right.x).abs() < f32::epsilon());
        assert!((left.y - right.y).abs() < f32::epsilon());
        assert!((left.z - right.z).abs() < f32::epsilon());
        assert!((left.w - right.w).abs() < f32::epsilon());

        // Test identity multiplication
        let identity = Quat::<f32>::identity();
        let result = q1 * identity;
        assert!((result.x - q1.x).abs() < f32::epsilon());
        assert!((result.y - q1.y).abs() < f32::epsilon());
        assert!((result.z - q1.z).abs() < f32::epsilon());
        assert!((result.w - q1.w).abs() < f32::epsilon());
    }

    #[test]
    fn test_quaternion_addition_subtraction() {
        let q1 = Quat::<f32>::new(1.0, 2.0, 3.0, 4.0);
        let q2 = Quat::<f32>::new(5.0, 6.0, 7.0, 8.0);

        let sum = q1 + q2;
        assert_eq!(sum.x, 6.0);
        assert_eq!(sum.y, 8.0);
        assert_eq!(sum.z, 10.0);
        assert_eq!(sum.w, 12.0);

        let diff = q2 - q1;
        assert_eq!(diff.x, 4.0);
        assert_eq!(diff.y, 4.0);
        assert_eq!(diff.z, 4.0);
        assert_eq!(diff.w, 4.0);
    }

    #[test]
    fn test_quaternion_scalar_multiplication() {
        let q = Quat::<f32>::new(1.0, 2.0, 3.0, 4.0);
        let scalar = 2.0;

        let result1 = q * scalar;
        assert_eq!(result1.x, 2.0);
        assert_eq!(result1.y, 4.0);
        assert_eq!(result1.z, 6.0);
        assert_eq!(result1.w, 8.0);

        let result2 = scalar * q;
        assert_eq!(result2.x, 2.0);
        assert_eq!(result2.y, 4.0);
        assert_eq!(result2.z, 6.0);
        assert_eq!(result2.w, 8.0);

        let result3 = q / scalar;
        assert_eq!(result3.x, 0.5);
        assert_eq!(result3.y, 1.0);
        assert_eq!(result3.z, 1.5);
        assert_eq!(result3.w, 2.0);
    }

    #[test]
    fn test_to_axis_angle_edge_cases() {
        // Test near-zero rotation (identity quaternion)
        let q_identity = Quat::<f32>::identity();
        let (axis, angle) = q_identity.to_axis_angle();
        assert!((angle).abs() < 0.001);
        assert!((axis.x - 1.0).abs() < 0.001);
        assert!(axis.y.abs() < 0.001);
        assert!(axis.z.abs() < 0.001);

        // Test clamped w slightly above 1 does not produce NaN
        let q_clamped = Quat::<f32>::new(0.0, 0.0, 0.0, 1.00001);
        let (axis_clamped, angle_clamped) = q_clamped.to_axis_angle();
        assert!(!angle_clamped.is_nan());
        assert!((axis_clamped.length() - 1.0).abs() < 0.01);
        assert!(Vector3::dot(&axis_clamped, &Vector3::new(1.0, 0.0, 0.0)) > 0.99);

        // Test 180-degree rotation
        let q_180 = quat_axis_angle_f32(&Vector3::new(0.0, 1.0, 0.0), PI);
        let (axis_180, angle_180) = q_180.to_axis_angle();
        assert!((angle_180 - PI).abs() < 0.001);
        assert!((axis_180.y - 1.0).abs() < 0.001);

        // Test very small rotation
        let small_angle = 0.001;
        let q_small = quat_axis_angle_f32(&Vector3::new(1.0, 0.0, 0.0), small_angle);
        let (axis_small, angle_small) = q_small.to_axis_angle();
        assert!((angle_small - small_angle).abs() < 0.0001);
        assert!((axis_small.length() - 1.0).abs() < 0.01);
        assert!(Vector3::dot(&axis_small, &Vector3::new(1.0, 0.0, 0.0)) > 0.99);
    }

    #[test]
    fn test_matrix_quaternion_conversion() {
        // Test round-trip conversion: quaternion -> matrix -> quaternion
        let angles = [0.0, 0.5, 1.0, FRAC_PI_2, PI];
        let axes = [
            Vector3::<f32>::new(1.0, 0.0, 0.0),
            Vector3::<f32>::new(0.0, 1.0, 0.0),
            Vector3::<f32>::new(0.0, 0.0, 1.0),
            Vector3::<f32>::normalize(&Vector3::new(1.0, 1.0, 1.0)),
        ];

        for angle in &angles {
            for axis in &axes {
                let q1 = quat_axis_angle_f32(axis, *angle);
                let mat = q1.mat3();
                let q2 = Quat::of_matrix3(&mat);

                // Account for quaternion double cover (q and -q represent same rotation)
                let dot_product = Quat::dot(&q1, &q2);
                let q2_adjusted = if dot_product < 0.0 {
                    Quat::neg(&q2)
                } else {
                    q2
                };

                assert!((q1.x - q2_adjusted.x).abs() < 0.001);
                assert!((q1.y - q2_adjusted.y).abs() < 0.001);
                assert!((q1.z - q2_adjusted.z).abs() < 0.001);
                assert!((q1.w - q2_adjusted.w).abs() < 0.001);
            }
        }
    }

    #[test]
    fn test_quaternion_matrix_roundtrip() {
        let q = quat_axis_angle_f32(&Vector3::new(0.0, 1.0, 0.0), 1.234);
        let mat = q.mat3();
        let q2 = Quat::of_matrix3(&mat);
        let dot = Quat::dot(&q, &q2);
        let q2 = if dot < 0.0 { Quat::neg(&q2) } else { q2 };

        assert!((q.x - q2.x).abs() < 0.001);
        assert!((q.y - q2.y).abs() < 0.001);
        assert!((q.z - q2.z).abs() < 0.001);
        assert!((q.w - q2.w).abs() < 0.001);
    }

    #[test]
    fn test_matrix4_quaternion_roundtrip() {
        let q = quat_axis_angle_f32(&Vector3::new(0.0, 0.0, 1.0), 0.75);
        let mat4 = q.mat4();
        let q2 = Quat::of_matrix4(&mat4);
        let dot = Quat::dot(&q, &q2);
        let q2 = if dot < 0.0 { Quat::neg(&q2) } else { q2 };

        assert!((q.x - q2.x).abs() < 0.001);
        assert!((q.y - q2.y).abs() < 0.001);
        assert!((q.z - q2.z).abs() < 0.001);
        assert!((q.w - q2.w).abs() < 0.001);
    }

    #[test]
    fn test_quaternion_conjugate() {
        let q = Quat::<f32>::new(1.0, 2.0, 3.0, 4.0);
        let conj = Quat::conjugate(&q);

        assert_eq!(conj.x, -1.0);
        assert_eq!(conj.y, -2.0);
        assert_eq!(conj.z, -3.0);
        assert_eq!(conj.w, 4.0);

        // Test that conjugate of conjugate is original
        let conj_conj = Quat::conjugate(&conj);
        assert_eq!(conj_conj.x, q.x);
        assert_eq!(conj_conj.y, q.y);
        assert_eq!(conj_conj.z, q.z);
        assert_eq!(conj_conj.w, q.w);
    }

    #[test]
    fn test_quaternion_dot_product() {
        let q1 = Quat::<f32>::new(1.0, 2.0, 3.0, 4.0);
        let q2 = Quat::<f32>::new(5.0, 6.0, 7.0, 8.0);

        let dot = Quat::dot(&q1, &q2);
        let expected = 1.0 * 5.0 + 2.0 * 6.0 + 3.0 * 7.0 + 4.0 * 8.0;
        assert_eq!(dot, expected);

        // Test dot product with itself equals length squared
        let self_dot = Quat::dot(&q1, &q1);
        let length_squared = q1.length() * q1.length();
        assert!((self_dot - length_squared).abs() < 0.0001);
    }

    #[test]
    fn test_quaternion_matrix_conversion_normalizes_input() {
        let q_unit = quat_axis_angle_f32(&Vector3::new(1.0, 0.0, 0.0), FRAC_PI_2);
        let q_scaled = q_unit * 5.0;

        let mat3_unit = q_unit.mat3();
        let mat3_scaled = q_scaled.mat3();
        let mat4_unit = q_unit.mat4();
        let mat4_scaled = q_scaled.mat4();

        for i in 0..3 {
            assert!((mat3_unit.col[i].x - mat3_scaled.col[i].x).abs() < 0.001);
            assert!((mat3_unit.col[i].y - mat3_scaled.col[i].y).abs() < 0.001);
            assert!((mat3_unit.col[i].z - mat3_scaled.col[i].z).abs() < 0.001);
        }

        for i in 0..4 {
            assert!((mat4_unit.col[i].x - mat4_scaled.col[i].x).abs() < 0.001);
            assert!((mat4_unit.col[i].y - mat4_scaled.col[i].y).abs() < 0.001);
            assert!((mat4_unit.col[i].z - mat4_scaled.col[i].z).abs() < 0.001);
            assert!((mat4_unit.col[i].w - mat4_scaled.col[i].w).abs() < 0.001);
        }
    }
}

impl<R: crate::fixed::RealFormat<F>, const F: u32> Mul<Quat<crate::Fixed<R, F>>>
    for crate::Fixed<R, F>
{
    type Output = Quat<Self>;
    fn mul(self, rhs: Self::Output) -> Self::Output {
        Quat::fmul(self, &rhs)
    }
}
impl<R: crate::fixed::RealFormat<F>, const F: u32> Div<Quat<crate::Fixed<R, F>>>
    for crate::Fixed<R, F>
{
    type Output = Quat<Self>;
    fn div(self, rhs: Self::Output) -> Self::Output {
        Quat::fdiv(self, &rhs)
    }
}
