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
use core::ops::*;
use crate::scalar::*;
use crate::vector::{Vector, Vector3};
use crate::matrix::{Matrix3, Matrix4};

#[repr(C)]
#[derive(Copy, Clone, Debug)]
pub struct Quat<T: Scalar> {
    pub x: T,
    pub y: T,
    pub z: T,
    pub w: T
}

impl<T: Scalar> Quat<T> {
    pub fn identity() -> Self { Self { x: T::zero(), y: T::zero(), z: T::zero(), w: T::one() } }
    pub fn new(x: T, y: T, z: T, w: T) -> Self { Self { x: x, y: y, z: z, w: w } }

    pub fn dot(l: &Self, r: &Self) -> T {
        l.x * r.x +
        l.y * r.y +
        l.z * r.z +
        l.w * r.w
    }

    pub fn length(&self) -> T { T::tsqrt(Self::dot(self, self)) }
    pub fn conjugate(q: &Self) -> Self { Self::new(-q.x, -q.y, -q.z, q.w) }
    pub fn normalize(q: &Self) -> Self { let ql = q.length(); if ql > T::zero() { *q / ql } else { *q } }
    pub fn neg(q: &Self) -> Self { Self::new(-q.x, -q.y, -q.z, -q.w) }
    pub fn add(l: &Self, r: &Self) -> Self { Self::new(l.x + r.x, l.y + r.y, l.z + r.z, l.w + r.w) }
    pub fn sub(l: &Self, r: &Self) -> Self { Self::new(l.x - r.x, l.y - r.y, l.z - r.z, l.w - r.w) }
    pub fn mul(l: &Self, r: &Self) -> Self {
        let	x = l.w * r.x + l.x * r.w + l.y * r.z - l.z * r.y;
        let	y = l.w * r.y + l.y * r.w + l.z * r.x - l.x * r.z;
        let	z = l.w * r.z + l.z * r.w + l.x * r.y - l.y * r.x;
        let	w = l.w * r.w - l.x * r.x - l.y * r.y - l.z * r.z;
        Self::new(x, y, z, w)
    }

    pub fn mulf(l: &Self, r: T) -> Self { Self::new(l.x * r, l.y * r, l.z * r, l.w * r) }
    pub fn fmul(l: T, r: &Self) -> Self { Self::new(l * r.x, l * r.y, l * r.z, l * r.w) }
    pub fn divf(l: &Self, r: T) -> Self { Self::new(l.x / r, l.y / r, l.z / r, l.w / r) }
    pub fn fdiv(l: T, r: &Self) -> Self { Self::new(l / r.x, l / r.y, l / r.z, l / r.w) }
    pub fn inverse(q: &Self) -> Self { Self::normalize(&Self::conjugate(q)) }

    pub fn mat3(&self) -> Matrix3<T> {
        let	xx = self.x * self.x;
        let	xy = self.x * self.y;
        let	xz = self.x * self.z;
        let	xw = self.x * self.w;
        let	yy = self.y * self.y;
        let	yz = self.y * self.z;
        let	yw = self.y * self.w;
        let	zz = self.z * self.z;
        let	zw = self.z * self.w;

        let	m00 = T::one() - T::two() * (yy + zz);
        let	m10 = T::two() * (xy + zw);
        let	m20 = T::two() * (xz - yw);
        let	m01 = T::two() * (xy - zw);
        let	m11 = T::one() - T::two() * (xx + zz);
        let	m21 = T::two() * (yz + xw);
        let	m02 = T::two() * (xz + yw);
        let	m12 = T::two() * (yz - xw);
        let	m22 = T::one() - T::two() * (xx + yy);

        Matrix3::new(m00, m10, m20, m01, m11, m21, m02, m12, m22)
    }

    pub fn mat4(&self) -> Matrix4<T> {
        let	xx = self.x * self.x;
        let	xy = self.x * self.y;
        let	xz = self.x * self.z;
        let	xw = self.x * self.w;
        let	yy = self.y * self.y;
        let	yz = self.y * self.z;
        let	yw = self.y * self.w;
        let	zz = self.z * self.z;
        let	zw = self.z * self.w;

        let	m00 = T::one() - T::two() * (yy + zz);
        let	m10 = T::two() * (xy + zw);
        let	m20 = T::two() * (xz - yw);
        let	m01 = T::two() * (xy - zw);
        let	m11 = T::one() - T::two() * (xx + zz);
        let	m21 = T::two() * (yz + xw);
        let	m02 = T::two() * (xz + yw);
        let	m12 = T::two() * (yz - xw);
        let	m22 = T::one() - T::two() * (xx + yy);

        Matrix4::new(m00, m10, m20, T::zero(),
                     m01, m11, m21, T::zero(),
                     m02, m12, m22, T::zero(),
                     T::zero(), T::zero(), T::zero(), T::one())
    }

    pub fn toAxisAngle(&self) -> (Vector3<T>, T) {
        let nq = Self::normalize(self);
        let	cos_a = nq.w;
        let	sin_a = {
            let sin_a = T::tsqrt(T::one() - cos_a * cos_a);
            if T::tabs(sin_a) < (T::one() / (T::l8192()))  {
                T::one()
            } else {
                sin_a
            }
        };

        let angle = T::tacos(cos_a) * T::two();
        let axis = Vector3::new(nq.x, nq.y, nq.z) / sin_a;
        (axis, angle)
    }

    pub fn ofMatrix3(m: &Matrix3<T>) -> Self {
        let	mat0 = m.col[0].x;
        let	mat1 = m.col[1].x;
        let	mat2 = m.col[2].x;

        let	mat4 = m.col[0].y;
        let	mat5 = m.col[1].y;
        let	mat6 = m.col[2].y;

        let	mat8 = m.col[0].z;
        let	mat9 = m.col[1].z;
        let	mat10 = m.col[2].z;

        let	t = T::one() + mat0 + mat5 + mat10;

        if t > T::zero() {
            let s = T::tsqrt(t) * T::two();

            let x = (mat9 - mat6) / s;
            let y = (mat2 - mat8) / s;
            let z = (mat4 - mat1) / s;
            let w = T::quarter() * s;
            Self::new(x, y, z, w)
        } else {
            if mat0 > mat5 && mat0 > mat10 {
                // Column 0:
                let s = T::tsqrt(T::one() + mat0 - mat5 - mat10) * T::two();
                let x = T::quarter() * s;
                let y = (mat4 + mat1) / s;
                let z = (mat2 + mat8) / s;
                let w = (mat9 - mat6) / s;
                Self::new(x, y, z, w)
            } else if mat5 > mat10 {
                // Column 1:
                let s = T::tsqrt(T::one() + mat5 - mat0 - mat10) * T::two();
                let x = (mat4 + mat1) / s;
                let y = T::quarter() * s;
                let z = (mat9 + mat6) / s;
                let w = (mat2 - mat8) / s;
                Self::new(x, y, z, w)
            } else {
                // Column 2:
                let s = T::tsqrt(T::one() + mat10 - mat0 - mat5) * T::two();
                let x = (mat2 + mat8) / s;
                let y = (mat9 + mat6) / s;
                let z = T::quarter() * s;
                let w = (mat4 - mat1) / s;
                Self::new(x, y, z, w)
            }
        }
    }

    pub fn ofMatrix4(m: &Matrix4<T>) -> Self {
        let	mat0 = m.col[0].x;
        let	mat1 = m.col[1].x;
        let	mat2 = m.col[2].x;

        let	mat4 = m.col[0].y;
        let	mat5 = m.col[1].y;
        let	mat6 = m.col[2].y;

        let	mat8 = m.col[0].z;
        let	mat9 = m.col[1].z;
        let	mat10 = m.col[2].z;

        let	t = T::one() + mat0 + mat5 + mat10;

        if t > T::zero() {
            let s = T::tsqrt(t) * T::two();

            let x = (mat9 - mat6) / s;
            let y = (mat2 - mat8) / s;
            let z = (mat4 - mat1) / s;
            let w = T::quarter() * s;
            Self::new(x, y, z, w)
        } else {
            if mat0 > mat5 && mat0 > mat10 {
                // Column 0:
                let s = T::tsqrt(T::one() + mat0 - mat5 - mat10) * T::two();
                let x = T::quarter() * s;
                let y = (mat4 + mat1) / s;
                let z = (mat2 + mat8) / s;
                let w = (mat9 - mat6) / s;
                Self::new(x, y, z, w)
            } else if mat5 > mat10 {
                // Column 1:
                let s = T::tsqrt(T::one() + mat5 - mat0 - mat10) * T::two();
                let x = (mat4 + mat1) / s;
                let y = T::quarter() * s;
                let z = (mat9 + mat6) / s;
                let w = (mat2 - mat8) / s;
                Self::new(x, y, z, w)
            } else {
                // Column 2:
                let s = T::tsqrt(T::one() + mat10 - mat0 - mat5) * T::two();
                let x = (mat2 + mat8) / s;
                let y = (mat9 + mat6) / s;
                let z = T::quarter() * s;
                let w = (mat4 - mat1) / s;
                Self::new(x, y, z, w)
            }
        }
    }

    pub fn ofAxisAngle(axis: &Vector3<T>, angle: T) -> Self {
        let	half_angle = angle * T::half();
        let	sin_a = T::tsin(half_angle);
        let	cos_a = T::tcos(half_angle);
        let	n : Vector3<T> = (if axis.length() > T::zero() { Vector3::normalize(axis) } else { *axis }) * sin_a;

        let x = n.x;
        let y = n.y;
        let z = n.z;
        let w = cos_a;
        Self::new(x, y, z, w)
    }
}

impl<T : Scalar> Add for Quat<T> {
    type Output = Quat<T>;
    fn add(self, rhs: Self) -> Self { Self::add(&self, &rhs) }
}

impl<T : Scalar> Sub for Quat<T> {
    type Output = Quat<T>;
    fn sub(self, rhs: Self) -> Self { Self::sub(&self, &rhs) }
}

impl<T : Scalar> Mul for Quat<T> {
    type Output = Quat<T>;
    fn mul(self, rhs: Self) -> Self { Self::mul(&self, &rhs) }
}

impl<T : Scalar> Div<T> for Quat<T> {
    type Output = Quat<T>;
    fn div(self, t: T) -> Self { Self::divf(&self, t) }
}

impl<T : Scalar> Mul<T> for Quat<T> {
    type Output = Quat<T>;
    fn mul(self, t: T) -> Self { Self::mulf(&self, t) }
}

macro_rules! implScalar {
    ($Op:ident, $op:ident, $Opop:ident, $t:ident) => {
        impl $Op<Quat<$t>> for $t {
            type Output = Quat<$t>;
            fn $op(self, q: Quat<$t>) -> Quat<$t> { Quat::$Opop(self, &q) }
        }
    };
}

implScalar!(Div, div, fdiv, f32);
implScalar!(Mul, mul, fmul, f32);
implScalar!(Div, div, fdiv, f64);
implScalar!(Mul, mul, fmul, f64);

#[cfg(test)]
mod tests {
    use crate::*;
    use crate::scalar::*;
    use crate::vector::*;
    use crate::matrix::*;
    use crate::{Quat};

    #[test]
    fn testIdentity() {
        let q = Quat::<f32>::identity();
        let m = q.mat3();

        let x_axis = m.col[0];
        let y_axis = m.col[1];
        let z_axis = m.col[2];

        assert_eq!((x_axis - Vector3::new(1.0, 0.0, 0.0)).length() < f32::epsilon(), true);
        assert_eq!((y_axis - Vector3::new(0.0, 1.0, 0.0)).length() < f32::epsilon(), true);
        assert_eq!((z_axis - Vector3::new(0.0, 0.0, 1.0)).length() < f32::epsilon(), true);
    }

    #[test]
    fn testIdentityConjugate() {
        let q = Quat::conjugate(&Quat::<f32>::identity());
        let m = q.mat3();

        let x_axis = m.col[0];
        let y_axis = m.col[1];
        let z_axis = m.col[2];

        assert_eq!((x_axis - Vector3::new(1.0, 0.0, 0.0)).length() < f32::epsilon(), true);
        assert_eq!((y_axis - Vector3::new(0.0, 1.0, 0.0)).length() < f32::epsilon(), true);
        assert_eq!((z_axis - Vector3::new(0.0, 0.0, 1.0)).length() < f32::epsilon(), true);
    }

    #[test]
    fn testRotX180() {
        let v = Vector3::<f32>::new(1.0, 0.0, 0.0);
        let a = 3.14159265;

        let q = Quat::ofAxisAngle(&v, a);
        let m = q.mat3();

        let x_axis = m.col[0];
        let y_axis = m.col[1];
        let z_axis = m.col[2];

        assert_eq!((x_axis - Vector3::new(1.0, 0.0, 0.0)).length() < f32::epsilon(), true);
        assert_eq!((y_axis - Vector3::new(0.0, -1.0, 0.0)).length() < f32::epsilon(), true);
        assert_eq!((z_axis - Vector3::new(0.0, 0.0, -1.0)).length() < f32::epsilon(), true);

        let m2 = Matrix3::ofAxisAngle(&v, a);

        let x2_axis = m2.col[0];
        let y2_axis = m2.col[1];
        let z2_axis = m2.col[2];

        assert_eq!((x_axis - x2_axis).length() < f32::epsilon(), true);
        assert_eq!((y_axis - y2_axis).length() < f32::epsilon(), true);
        assert_eq!((z_axis - z2_axis).length() < f32::epsilon(), true);
    }

    #[test]
    fn testRotX90() {
        let v = Vector3::<f32>::new(1.0, 0.0, 0.0);
        let a = 3.14159265 / 2.0;

        let q = Quat::ofAxisAngle(&v, a);
        let m = q.mat3();

        let x_axis = m.col[0];
        let y_axis = m.col[1];
        let z_axis = m.col[2];

        assert_eq!((x_axis - Vector3::new(1.0, 0.0, 0.0)).length() < f32::epsilon(), true);
        assert_eq!((y_axis - Vector3::new(0.0, 0.0, 1.0)).length() < f32::epsilon(), true);
        assert_eq!((z_axis - Vector3::new(0.0, -1.0, 0.0)).length() < f32::epsilon(), true);

        let m2 = Matrix3::ofAxisAngle(&v, a);

        let x2_axis = m2.col[0];
        let y2_axis = m2.col[1];
        let z2_axis = m2.col[2];

        assert_eq!((x_axis - x2_axis).length() < f32::epsilon(), true);
        assert_eq!((y_axis - y2_axis).length() < f32::epsilon(), true);
        assert_eq!((z_axis - z2_axis).length() < f32::epsilon(), true);
    }

    #[test]
    fn testRotX45() {
        let v = Vector3::<f32>::new(1.0, 0.0, 0.0);
        let a = 3.14159265 / 4.0;

        let q = Quat::ofAxisAngle(&v, a);
        let m = q.mat3();

        let x_axis = m.col[0];
        let y_axis = m.col[1];
        let z_axis = m.col[2];

        assert_eq!((x_axis - Vector3::new(1.0, 0.0, 0.0)).length() < f32::epsilon(), true);
        assert_eq!((y_axis - Vector3::new(0.0, f32::sqrt(2.0) / 2.0, f32::sqrt(2.0) / 2.0)).length() < f32::epsilon(), true);
        assert_eq!((z_axis - Vector3::new(0.0, -f32::sqrt(2.0) / 2.0, f32::sqrt(2.0) / 2.0)).length() < f32::epsilon(), true);

        let m2 = Matrix3::ofAxisAngle(&v, a);

        let x2_axis = m2.col[0];
        let y2_axis = m2.col[1];
        let z2_axis = m2.col[2];

        assert_eq!((x_axis - x2_axis).length() < f32::epsilon(), true);
        assert_eq!((y_axis - y2_axis).length() < f32::epsilon(), true);
        assert_eq!((z_axis - z2_axis).length() < f32::epsilon(), true);
    }

    #[test]
    fn testRotXComposed45() {
        let v = Vector3::<f32>::new(1.0, 0.0, 0.0);
        let a_u = 3.14159265 / 4.0;
        let a_f = 3.14159265;

        let q_u = Quat::ofAxisAngle(&v, a_u);
        let q_f = Quat::ofAxisAngle(&v, a_f);

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
    fn testRotXComposed120() {
        let v = Vector3::<f32>::new(1.0, 0.0, 0.0);
        let a_u1 = 3.14159265 / 2.0;
        let a_u2 = 3.14159265 / 6.0;
        let a_f = 3.14159265 / 2.0 + 3.14159265 / 6.0;

        let q_u1 = Quat::ofAxisAngle(&v, a_u1);
        let q_u2 = Quat::ofAxisAngle(&v, a_u2);
        let q_f = Quat::ofAxisAngle(&v, a_f);

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
    fn testRotXRotY() {
        let vx = Vector3::<f32>::new(1.0, 0.0, 0.0);
        let vy = Vector3::<f32>::new(0.0, 1.0, 0.0);
        let vz = Vector3::<f32>::new(0.0, 0.0, 1.0);

        let v4x = Vector4::<f32>::new(1.0, 0.0, 0.0, 1.0);
        let v4y = Vector4::<f32>::new(0.0, 1.0, 0.0, 1.0);
        let v4z = Vector4::<f32>::new(0.0, 0.0, 1.0, 1.0);

        let a = 3.14159265 / 2.0;

        let q_ux = Quat::ofAxisAngle(&vx, a);
        let q_uy = Quat::ofAxisAngle(&vy, a);
        let q_uz = Quat::ofAxisAngle(&vz, a);

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
    fn testAxisAngleF32() {
        let v = Vector3::normalize(&Vector3::<f32>::new(1.0, 2.0, 3.0));
        let q0 = Quat::ofAxisAngle(&v, 3.0);
        let (v2, a) = q0.toAxisAngle();
        assert_eq!((v.x - v2.x).abs() < f32::epsilon(), true);
        assert_eq!((v.y - v2.y).abs() < f32::epsilon(), true);
        assert_eq!((v.z - v2.z).abs() < f32::epsilon(), true);
        assert_eq!((a - 3.0).abs()    < f32::epsilon(), true);
    }

    #[test]
    fn testAxisAngleF64() {
        let v = Vector3::normalize(&Vector3::<f64>::new(1.0, 2.0, 3.0));
        let q0 = Quat::ofAxisAngle(&v, 3.0);
        let (v2, a) = q0.toAxisAngle();
        assert_eq!((v.x - v2.x).abs() < f64::epsilon(), true);
        assert_eq!((v.y - v2.y).abs() < f64::epsilon(), true);
        assert_eq!((v.z - v2.z).abs() < f64::epsilon(), true);
        assert_eq!((a - 3.0).abs()    < f64::epsilon(), true);
    }
}
