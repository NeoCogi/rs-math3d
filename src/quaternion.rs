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
use crate::vector::{Vector3};
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
        let	m01 = T::two() * (xy - zw);
        let	m02 = T::two() * (xz + yw);
        let	m10 = T::two() * (xy + zw);
        let	m11 = T::one() - T::two() * (xx + zz);
        let	m12 = T::two() * (yz - xw);
        let	m20 = T::two() * (xz - yw);
        let	m21 = T::two() * (yz + xw);
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
        let	m01 = T::two() * (xy - zw);
        let	m02 = T::two() * (xz + yw);
        let	m10 = T::two() * (xy + zw);
        let	m11 = T::one() - T::two() * (xx + zz);
        let	m12 = T::two() * (yz - xw);
        let	m20 = T::two() * (xz - yw);
        let	m21 = T::two() * (yz + xw);
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
