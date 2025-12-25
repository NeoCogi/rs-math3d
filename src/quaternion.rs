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
use crate::matrix::{Matrix3, Matrix4};
use crate::scalar::*;
use crate::vector::{FloatVector, Vector, Vector3};
use core::ops::*;
use num_traits::{Zero, One};

#[repr(C)]
#[derive(Copy, Clone, Debug, Default)]
pub struct Quat<T: Scalar> {
    pub x: T,
    pub y: T,
    pub z: T,
    pub w: T,
}

impl<T: FloatScalar> Quat<T> {
    pub fn identity() -> Self {
        Self {
            x: <T as Zero>::zero(),
            y: <T as Zero>::zero(),
            z: <T as Zero>::zero(),
            w: <T as One>::one(),
        }
    }
    pub fn new(x: T, y: T, z: T, w: T) -> Self {
        Self {
            x: x,
            y: y,
            z: z,
            w: w,
        }
    }

    /// Computes the dot product of two quaternions.
    ///
    /// ```text
    /// q₁ · q₂ = x₁x₂ + y₁y₂ + z₁z₂ + w₁w₂
    /// ```
    pub fn dot(l: &Self, r: &Self) -> T {
        l.x * r.x + l.y * r.y + l.z * r.z + l.w * r.w
    }

    /// Computes the length (magnitude) of the quaternion.
    ///
    /// ```text
    /// ||q|| = √(x² + y² + z² + w²)
    /// ```
    pub fn length(&self) -> T {
        T::tsqrt(Self::dot(self, self))
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
    
    /// Normalizes the quaternion to unit length.
    ///
    /// Unit quaternions represent valid rotations:
    /// ```text
    /// q̂ = q / ||q||
    /// ```
    pub fn normalize(q: &Self) -> Self {
        let ql = q.length();
        if ql > <T as Zero>::zero() {
            *q / ql
        } else {
            *q
        }
    }
    
    /// Negates all components of the quaternion.
    ///
    /// Note: -q and q represent the same rotation (double cover property).
    pub fn neg(q: &Self) -> Self {
        Self::new(-q.x, -q.y, -q.z, -q.w)
    }
    pub fn add(l: &Self, r: &Self) -> Self {
        Self::new(l.x + r.x, l.y + r.y, l.z + r.z, l.w + r.w)
    }
    pub fn sub(l: &Self, r: &Self) -> Self {
        Self::new(l.x - r.x, l.y - r.y, l.z - r.z, l.w - r.w)
    }
    pub fn mul(l: &Self, r: &Self) -> Self {
        let x = l.w * r.x + l.x * r.w + l.y * r.z - l.z * r.y;
        let y = l.w * r.y + l.y * r.w + l.z * r.x - l.x * r.z;
        let z = l.w * r.z + l.z * r.w + l.x * r.y - l.y * r.x;
        let w = l.w * r.w - l.x * r.x - l.y * r.y - l.z * r.z;
        Self::new(x, y, z, w)
    }

    pub fn mulf(l: &Self, r: T) -> Self {
        Self::new(l.x * r, l.y * r, l.z * r, l.w * r)
    }
    pub fn fmul(l: T, r: &Self) -> Self {
        Self::new(l * r.x, l * r.y, l * r.z, l * r.w)
    }
    pub fn divf(l: &Self, r: T) -> Self {
        Self::new(l.x / r, l.y / r, l.z / r, l.w / r)
    }
    pub fn fdiv(l: T, r: &Self) -> Self {
        Self::new(l / r.x, l / r.y, l / r.z, l / r.w)
    }
    /// Returns the multiplicative inverse of the quaternion.
    ///
    /// For non-zero quaternions: q⁻¹ = conjugate(q) / |q|².
    /// Returns the input unchanged if the norm is too small.
    pub fn inverse(q: &Self) -> Self {
        let denom = Self::dot(q, q);
        if denom > T::epsilon() {
            Self::conjugate(q) / denom
        } else {
            *q
        }
    }

    /// Converts the quaternion to a 3x3 rotation matrix.
    ///
    /// The conversion formula:
    /// ```text
    /// R = I + 2s(q×) + 2(q×)²
    /// ```
    /// where q× is the cross-product matrix of the vector part.
    pub fn mat3(&self) -> Matrix3<T> {
        let xx = self.x * self.x;
        let xy = self.x * self.y;
        let xz = self.x * self.z;
        let xw = self.x * self.w;
        let yy = self.y * self.y;
        let yz = self.y * self.z;
        let yw = self.y * self.w;
        let zz = self.z * self.z;
        let zw = self.z * self.w;

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
    pub fn mat4(&self) -> Matrix4<T> {
        let xx = self.x * self.x;
        let xy = self.x * self.y;
        let xz = self.x * self.z;
        let xw = self.x * self.w;
        let yy = self.y * self.y;
        let yz = self.y * self.z;
        let yw = self.y * self.w;
        let zz = self.z * self.z;
        let zw = self.z * self.w;

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
    pub fn to_axis_angle(&self) -> (Vector3<T>, T) {
        let nq = Self::normalize(self);
        let cos_a = nq.w;
        let sin_a = {
            let sin_a = T::tsqrt(<T as One>::one() - cos_a * cos_a);
            // Use epsilon for numerical stability check
            if T::tabs(sin_a) < T::epsilon() {
                <T as One>::one()
            } else {
                sin_a
            }
        };

        let angle = T::tacos(cos_a) * T::two();
        let axis = Vector3::new(nq.x, nq.y, nq.z) / sin_a;
        (axis, angle)
    }

    pub fn of_matrix3(m: &Matrix3<T>) -> Self {
        let mat0 = m.col[0].x;
        let mat1 = m.col[1].x;
        let mat2 = m.col[2].x;

        let mat4 = m.col[0].y;
        let mat5 = m.col[1].y;
        let mat6 = m.col[2].y;

        let mat8 = m.col[0].z;
        let mat9 = m.col[1].z;
        let mat10 = m.col[2].z;

        let t = <T as One>::one() + mat0 + mat5 + mat10;

        if t > <T as Zero>::zero() {
            let s = T::tsqrt(t) * T::two();

            let x = (mat9 - mat6) / s;
            let y = (mat2 - mat8) / s;
            let z = (mat4 - mat1) / s;
            let w = T::quarter() * s;
            Self::new(x, y, z, w)
        } else {
            if mat0 > mat5 && mat0 > mat10 {
                // Column 0:
                let s = T::tsqrt(<T as One>::one() + mat0 - mat5 - mat10) * T::two();
                let x = T::quarter() * s;
                let y = (mat4 + mat1) / s;
                let z = (mat2 + mat8) / s;
                let w = (mat9 - mat6) / s;
                Self::new(x, y, z, w)
            } else if mat5 > mat10 {
                // Column 1:
                let s = T::tsqrt(<T as One>::one() + mat5 - mat0 - mat10) * T::two();
                let x = (mat4 + mat1) / s;
                let y = T::quarter() * s;
                let z = (mat9 + mat6) / s;
                let w = (mat2 - mat8) / s;
                Self::new(x, y, z, w)
            } else {
                // Column 2:
                let s = T::tsqrt(<T as One>::one() + mat10 - mat0 - mat5) * T::two();
                let x = (mat2 + mat8) / s;
                let y = (mat9 + mat6) / s;
                let z = T::quarter() * s;
                let w = (mat4 - mat1) / s;
                Self::new(x, y, z, w)
            }
        }
    }

    pub fn of_matrix4(m: &Matrix4<T>) -> Self {
        let mat0 = m.col[0].x;
        let mat1 = m.col[1].x;
        let mat2 = m.col[2].x;

        let mat4 = m.col[0].y;
        let mat5 = m.col[1].y;
        let mat6 = m.col[2].y;

        let mat8 = m.col[0].z;
        let mat9 = m.col[1].z;
        let mat10 = m.col[2].z;

        let t = <T as One>::one() + mat0 + mat5 + mat10;

        if t > <T as Zero>::zero() {
            let s = T::tsqrt(t) * T::two();

            let x = (mat9 - mat6) / s;
            let y = (mat2 - mat8) / s;
            let z = (mat4 - mat1) / s;
            let w = T::quarter() * s;
            Self::new(x, y, z, w)
        } else {
            if mat0 > mat5 && mat0 > mat10 {
                // Column 0:
                let s = T::tsqrt(<T as One>::one() + mat0 - mat5 - mat10) * T::two();
                let x = T::quarter() * s;
                let y = (mat4 + mat1) / s;
                let z = (mat2 + mat8) / s;
                let w = (mat9 - mat6) / s;
                Self::new(x, y, z, w)
            } else if mat5 > mat10 {
                // Column 1:
                let s = T::tsqrt(<T as One>::one() + mat5 - mat0 - mat10) * T::two();
                let x = (mat4 + mat1) / s;
                let y = T::quarter() * s;
                let z = (mat9 + mat6) / s;
                let w = (mat2 - mat8) / s;
                Self::new(x, y, z, w)
            } else {
                // Column 2:
                let s = T::tsqrt(<T as One>::one() + mat10 - mat0 - mat5) * T::two();
                let x = (mat2 + mat8) / s;
                let y = (mat9 + mat6) / s;
                let z = T::quarter() * s;
                let w = (mat4 - mat1) / s;
                Self::new(x, y, z, w)
            }
        }
    }

    /// Creates a quaternion from an axis and angle.
    ///
    /// # Parameters
    /// - `axis`: The rotation axis (will be normalized)
    /// - `angle`: The rotation angle in radians
    /// - `epsilon`: Minimum axis length to treat as valid
    ///
    /// The quaternion is constructed as:
    /// ```text
    /// q = cos(θ/2) + sin(θ/2) * normalize(axis)
    /// ```
    ///
    /// # Returns
    /// - `Some(quaternion)` for a valid axis
    /// - `None` if the axis length is too small
    pub fn of_axis_angle(axis: &Vector3<T>, angle: T, epsilon: T) -> Option<Self> {
        let len_sq = Vector3::dot(axis, axis);
        if len_sq <= epsilon * epsilon {
            return None;
        }
        let half_angle = angle * T::half();
        let sin_a = T::tsin(half_angle);
        let cos_a = T::tcos(half_angle);
        let inv_len = <T as One>::one() / len_sq.tsqrt();
        let n = *axis * inv_len * sin_a;

        let x = n.x;
        let y = n.y;
        let z = n.z;
        let w = cos_a;
        Some(Self::new(x, y, z, w))
    }
}

impl<T: FloatScalar> Add for Quat<T> {
    type Output = Quat<T>;
    fn add(self, rhs: Self) -> Self {
        Self::add(&self, &rhs)
    }
}

impl<T: FloatScalar> Sub for Quat<T> {
    type Output = Quat<T>;
    fn sub(self, rhs: Self) -> Self {
        Self::sub(&self, &rhs)
    }
}

impl<T: FloatScalar> Mul for Quat<T> {
    type Output = Quat<T>;
    fn mul(self, rhs: Self) -> Self {
        Self::mul(&self, &rhs)
    }
}

impl<T: FloatScalar> Div<T> for Quat<T> {
    type Output = Quat<T>;
    fn div(self, t: T) -> Self {
        Self::divf(&self, t)
    }
}

impl<T: FloatScalar> Mul<T> for Quat<T> {
    type Output = Quat<T>;
    fn mul(self, t: T) -> Self {
        Self::mulf(&self, t)
    }
}

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

impl_scalar!(Div, div, fdiv, f32);
impl_scalar!(Mul, mul, fmul, f32);
impl_scalar!(Div, div, fdiv, f64);
impl_scalar!(Mul, mul, fmul, f64);

#[cfg(test)]
mod tests {
    use crate::matrix::*;
    use crate::scalar::*;
    use crate::vector::*;
    use crate::Quat;
    use crate::*;

    fn quat_axis_angle_f32(axis: &Vector3<f32>, angle: f32) -> Quat<f32> {
        Quat::of_axis_angle(axis, angle, EPS_F32).expect("axis length too small")
    }

    fn quat_axis_angle_f64(axis: &Vector3<f64>, angle: f64) -> Quat<f64> {
        Quat::of_axis_angle(axis, angle, EPS_F64).expect("axis length too small")
    }

    fn mat3_axis_angle_f32(axis: &Vector3<f32>, angle: f32) -> Matrix3<f32> {
        Matrix3::of_axis_angle(axis, angle, EPS_F32).expect("axis length too small")
    }

    #[test]
    fn test_identity() {
        let q = Quat::<f32>::identity();
        let m = q.mat3();

        let x_axis = m.col[0];
        let y_axis = m.col[1];
        let z_axis = m.col[2];

        assert_eq!(
            (x_axis - Vector3::new(1.0, 0.0, 0.0)).length() < f32::epsilon(),
            true
        );
        assert_eq!(
            (y_axis - Vector3::new(0.0, 1.0, 0.0)).length() < f32::epsilon(),
            true
        );
        assert_eq!(
            (z_axis - Vector3::new(0.0, 0.0, 1.0)).length() < f32::epsilon(),
            true
        );
    }

    #[test]
    fn test_identity_conjugate() {
        let q = Quat::conjugate(&Quat::<f32>::identity());
        let m = q.mat3();

        let x_axis = m.col[0];
        let y_axis = m.col[1];
        let z_axis = m.col[2];

        assert_eq!(
            (x_axis - Vector3::new(1.0, 0.0, 0.0)).length() < f32::epsilon(),
            true
        );
        assert_eq!(
            (y_axis - Vector3::new(0.0, 1.0, 0.0)).length() < f32::epsilon(),
            true
        );
        assert_eq!(
            (z_axis - Vector3::new(0.0, 0.0, 1.0)).length() < f32::epsilon(),
            true
        );
    }

    #[test]
    fn test_rot_180() {
        let v = Vector3::<f32>::new(1.0, 0.0, 0.0);
        let a = 3.14159265;

        let q = quat_axis_angle_f32(&v, a);
        let m = q.mat3();

        let x_axis = m.col[0];
        let y_axis = m.col[1];
        let z_axis = m.col[2];

        assert_eq!(
            (x_axis - Vector3::new(1.0, 0.0, 0.0)).length() < f32::epsilon(),
            true
        );
        assert_eq!(
            (y_axis - Vector3::new(0.0, -1.0, 0.0)).length() < f32::epsilon(),
            true
        );
        assert_eq!(
            (z_axis - Vector3::new(0.0, 0.0, -1.0)).length() < f32::epsilon(),
            true
        );

        let m2 = mat3_axis_angle_f32(&v, a);

        let x2_axis = m2.col[0];
        let y2_axis = m2.col[1];
        let z2_axis = m2.col[2];

        assert_eq!((x_axis - x2_axis).length() < f32::epsilon(), true);
        assert_eq!((y_axis - y2_axis).length() < f32::epsilon(), true);
        assert_eq!((z_axis - z2_axis).length() < f32::epsilon(), true);
    }

    #[test]
    fn test_rot_90() {
        let v = Vector3::<f32>::new(1.0, 0.0, 0.0);
        let a = 3.14159265 / 2.0;

        let q = quat_axis_angle_f32(&v, a);
        let m = q.mat3();

        let x_axis = m.col[0];
        let y_axis = m.col[1];
        let z_axis = m.col[2];

        assert_eq!(
            (x_axis - Vector3::new(1.0, 0.0, 0.0)).length() < f32::epsilon(),
            true
        );
        assert_eq!(
            (y_axis - Vector3::new(0.0, 0.0, 1.0)).length() < f32::epsilon(),
            true
        );
        assert_eq!(
            (z_axis - Vector3::new(0.0, -1.0, 0.0)).length() < f32::epsilon(),
            true
        );

        let m2 = mat3_axis_angle_f32(&v, a);

        let x2_axis = m2.col[0];
        let y2_axis = m2.col[1];
        let z2_axis = m2.col[2];

        assert_eq!((x_axis - x2_axis).length() < f32::epsilon(), true);
        assert_eq!((y_axis - y2_axis).length() < f32::epsilon(), true);
        assert_eq!((z_axis - z2_axis).length() < f32::epsilon(), true);
    }

    #[test]
    fn test_rot_45() {
        let v = Vector3::<f32>::new(1.0, 0.0, 0.0);
        let a = 3.14159265 / 4.0;

        let q = quat_axis_angle_f32(&v, a);
        let m = q.mat3();

        let x_axis = m.col[0];
        let y_axis = m.col[1];
        let z_axis = m.col[2];

        assert_eq!(
            (x_axis - Vector3::new(1.0, 0.0, 0.0)).length() < f32::epsilon(),
            true
        );
        assert_eq!(
            (y_axis - Vector3::new(0.0, f32::sqrt(2.0) / 2.0, f32::sqrt(2.0) / 2.0)).length()
                < f32::epsilon(),
            true
        );
        assert_eq!(
            (z_axis - Vector3::new(0.0, -f32::sqrt(2.0) / 2.0, f32::sqrt(2.0) / 2.0)).length()
                < f32::epsilon(),
            true
        );

        let m2 = mat3_axis_angle_f32(&v, a);

        let x2_axis = m2.col[0];
        let y2_axis = m2.col[1];
        let z2_axis = m2.col[2];

        assert_eq!((x_axis - x2_axis).length() < f32::epsilon(), true);
        assert_eq!((y_axis - y2_axis).length() < f32::epsilon(), true);
        assert_eq!((z_axis - z2_axis).length() < f32::epsilon(), true);
    }

    #[test]
    fn test_rot_composed_45() {
        let v = Vector3::<f32>::new(1.0, 0.0, 0.0);
        let a_u = 3.14159265 / 4.0;
        let a_f = 3.14159265;

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

        assert_eq!((x_axis - x2_axis).length() < f32::epsilon(), true);
        assert_eq!((y_axis - y2_axis).length() < f32::epsilon(), true);
        assert_eq!((z_axis - z2_axis).length() < f32::epsilon(), true);
    }

    #[test]
    fn test_rot_composed_120() {
        let v = Vector3::<f32>::new(1.0, 0.0, 0.0);
        let a_u1 = 3.14159265 / 2.0;
        let a_u2 = 3.14159265 / 6.0;
        let a_f = 3.14159265 / 2.0 + 3.14159265 / 6.0;

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

        assert_eq!((x_axis - x2_axis).length() < f32::epsilon(), true);
        assert_eq!((y_axis - y2_axis).length() < f32::epsilon(), true);
        assert_eq!((z_axis - z2_axis).length() < f32::epsilon(), true);
    }

    #[test]
    fn test_rot_x_rot_y() {
        let vx = Vector3::<f32>::new(1.0, 0.0, 0.0);
        let vy = Vector3::<f32>::new(0.0, 1.0, 0.0);
        let vz = Vector3::<f32>::new(0.0, 0.0, 1.0);

        let v4x = Vector4::<f32>::new(1.0, 0.0, 0.0, 1.0);
        let v4y = Vector4::<f32>::new(0.0, 1.0, 0.0, 1.0);
        let v4z = Vector4::<f32>::new(0.0, 0.0, 1.0, 1.0);

        let a = 3.14159265 / 2.0;

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

        assert_eq!((x_axis - vx).length() < f32::epsilon(), true);
        assert_eq!((y_axis - vz).length() < f32::epsilon(), true);
        assert_eq!((z_axis - -vy).length() < f32::epsilon(), true);

        assert_eq!((px - vx).length() < f32::epsilon(), true);
        assert_eq!((py - vz).length() < f32::epsilon(), true);
        assert_eq!((pz - -vy).length() < f32::epsilon(), true);

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

        assert_eq!((x_axis - -vz).length() < f32::epsilon(), true);
        assert_eq!((y_axis - vx).length() < f32::epsilon(), true);
        assert_eq!((z_axis - -vy).length() < f32::epsilon(), true);

        assert_eq!((px - -vz).length() < f32::epsilon(), true);
        assert_eq!((py - vx).length() < f32::epsilon(), true);
        assert_eq!((pz - -vy).length() < f32::epsilon(), true);

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

        assert_eq!((x_axis - -vz).length() < f32::epsilon(), true);
        assert_eq!((y_axis - vy).length() < f32::epsilon(), true);
        assert_eq!((z_axis - vx).length() < f32::epsilon(), true);

        assert_eq!((px - -vz).length() < f32::epsilon(), true);
        assert_eq!((py - vy).length() < f32::epsilon(), true);
        assert_eq!((pz - vx).length() < f32::epsilon(), true);
    }

    #[test]
    fn test_axis_angle_f32() {
        let v = Vector3::normalize(&Vector3::<f32>::new(1.0, 2.0, 3.0));
        let q0 = quat_axis_angle_f32(&v, 3.0);
        let (v2, a) = q0.to_axis_angle();
        assert_eq!((v.x - v2.x).abs() < f32::epsilon(), true);
        assert_eq!((v.y - v2.y).abs() < f32::epsilon(), true);
        assert_eq!((v.z - v2.z).abs() < f32::epsilon(), true);
        assert_eq!((a - 3.0).abs() < f32::epsilon(), true);
    }

    #[test]
    fn test_axis_angle_f64() {
        let v = Vector3::normalize(&Vector3::<f64>::new(1.0, 2.0, 3.0));
        let q0 = quat_axis_angle_f64(&v, 3.0);
        let (v2, a) = q0.to_axis_angle();
        assert_eq!((v.x - v2.x).abs() < f64::epsilon(), true);
        assert_eq!((v.y - v2.y).abs() < f64::epsilon(), true);
        assert_eq!((v.z - v2.z).abs() < f64::epsilon(), true);
        assert_eq!((a - 3.0).abs() < f64::epsilon(), true);
    }

    #[test]
    fn test_axis_angle_zero_axis_none() {
        let axis = Vector3::<f32>::new(0.0, 0.0, 0.0);
        assert!(Quat::of_axis_angle(&axis, 1.0, EPS_F32).is_none());
    }

    #[test]
    fn test_quaternion_normalization() {
        // Test normalizing a non-unit quaternion
        let q = Quat::<f32>::new(1.0, 2.0, 3.0, 4.0);
        let nq = Quat::normalize(&q);
        let len = nq.length();
        assert!((len - 1.0).abs() < f32::epsilon());
        
        // Test normalizing zero quaternion (edge case)
        let q_zero = Quat::<f32>::new(0.0, 0.0, 0.0, 0.0);
        let nq_zero = Quat::normalize(&q_zero);
        assert_eq!(nq_zero.x, 0.0);
        assert_eq!(nq_zero.y, 0.0);
        assert_eq!(nq_zero.z, 0.0);
        assert_eq!(nq_zero.w, 0.0);
        
        // Test already normalized quaternion
        let q_unit = Quat::<f32>::identity();
        let nq_unit = Quat::normalize(&q_unit);
        assert!((nq_unit.length() - 1.0).abs() < f32::epsilon());
    }

    #[test]
    fn test_quaternion_inverse() {
        let q = quat_axis_angle_f32(&Vector3::new(0.0, 1.0, 0.0), 1.57079632);
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
        
        // Test 180-degree rotation
        let q_180 = quat_axis_angle_f32(&Vector3::new(0.0, 1.0, 0.0), 3.14159265);
        let (axis_180, angle_180) = q_180.to_axis_angle();
        assert!((angle_180 - 3.14159265).abs() < 0.001);
        assert!((axis_180.y - 1.0).abs() < 0.001);
        
        // Test very small rotation
        let small_angle = 0.001;
        let q_small = quat_axis_angle_f32(&Vector3::new(1.0, 0.0, 0.0), small_angle);
        let (axis_small, angle_small) = q_small.to_axis_angle();
        assert!((angle_small - small_angle).abs() < 0.0001);
    }

    #[test]
    fn test_matrix_quaternion_conversion() {
        // Test round-trip conversion: quaternion -> matrix -> quaternion
        let angles = [0.0, 0.5, 1.0, 1.57, 3.14159];
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
        let expected = 1.0*5.0 + 2.0*6.0 + 3.0*7.0 + 4.0*8.0;
        assert_eq!(dot, expected);
        
        // Test dot product with itself equals length squared
        let self_dot = Quat::dot(&q1, &q1);
        let length_squared = q1.length() * q1.length();
        assert!((self_dot - length_squared).abs() < 0.0001);
    }
}
