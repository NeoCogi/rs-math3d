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
use crate::vector::*;

#[repr(C)]
#[derive(Clone, Copy, Debug)]
pub struct Matrix2<T : Scalar> {
    pub col: [Vector2<T>; 2]
}

#[repr(C)]
#[derive(Clone, Copy, Debug)]
pub struct Matrix3<T : Scalar> {
    pub col: [Vector3<T>; 3]
}

#[repr(C)]
#[derive(Clone, Copy, Debug)]
pub struct Matrix4<T : Scalar> {
    pub col: [Vector4<T>; 4]
}

/******************************************************************************
 * Matrix2
 *
 * i j ------------------->
 * | [m0 = c0_x | m2 = c1_x]
 * V [m1 = c0_y | m3 = c1_y]
 *
 *  aij => i = row, j = col (yx form)
 *
 *****************************************************************************/
impl<T: Scalar> Matrix2<T> {
    pub fn new(m0: T, m1: T, m2: T, m3: T) -> Self {
        Matrix2 { col: [
            Vector2::new(m0, m1),
            Vector2::new(m2, m3)
            ]
        }
    }

    pub fn identity() -> Self {
        Self::new(T::one(), T::zero(), T::zero(), T::one())
    }

    pub fn determinant(&self) -> T {
        let	m00 = self.col[0].x;
        let	m10 = self.col[0].y;

        let	m01 = self.col[1].x;
        let	m11 = self.col[1].y;

        m00 * m11 - m01 * m10
    }

    pub fn transpose(&self) -> Self {
        let m00 = self.col[0].x;
        let m10 = self.col[0].y;

        let m01 = self.col[1].x;
        let m11 = self.col[1].y;

        Self::new(m00, m01, m10, m11)
    }

    pub fn inverse(&self) -> Self {
        let	m00 = self.col[0].x;
        let	m10 = self.col[0].y;

        let	m01 = self.col[1].x;
        let	m11 = self.col[1].y;

        let	inv_det = T::one() / (m00 * m11 - m01 * m10);

        let	r00 = m11 * inv_det;
        let	r01 = -m01 * inv_det;
        let	r10 = -m10 * inv_det;
        let	r11 = m00 * inv_det;

        Self::new(r00, r10, r01, r11)
    }

    //
    //             [b00 b01]
    //             [b10 b11]
    //           *   |   |
    // [a00 a01] - [c00 c01]
    // [a10 a11] - [c10 c11]
    //
    pub fn mul_matrix_matrix(l: &Self, r: &Self) -> Self {
        let	a00 = l.col[0].x;
        let	a10 = l.col[0].y;
        let	a01 = l.col[1].x;
        let	a11 = l.col[1].y;

        let	b00 = r.col[0].x;
        let	b10 = r.col[0].y;
        let	b01 = r.col[1].x;
        let	b11 = r.col[1].y;

        let	c00 = a00 * b00 + a01 * b10;
        let	c01 = a00 * b01 + a01 * b11;
        let	c10 = a10 * b00 + a11 * b10;
        let	c11 = a10 * b01 + a11 * b11;

        Self::new(c00, c10, c01, c11)
    }

    pub fn mul_matrix_vector(l: &Self, r: &Vector2<T>) -> Vector2<T> {
        Self::mul_vector_matrix(r, &l.transpose())
    }

    pub fn mul_vector_matrix(l: &Vector2<T>, r: &Self) -> Vector2<T> {
        Vector2::new(Vector2::dot(l, &r.col[0]), Vector2::dot(l, &r.col[1]))
    }

    pub fn add_matrix_matrix(l: &Self, r: &Self) -> Self {
        Matrix2 { col : [
            l.col[0] + r.col[0],
            l.col[1] + r.col[1],
        ]}
    }

    pub fn sub_matrix_matrix(l: &Self, r: &Self) -> Self {
        Matrix2 { col : [
            l.col[0] - r.col[0],
            l.col[1] - r.col[1],
        ]}
    }
}

/******************************************************************************
 * Matrix3
 *
 * i j -------------------------------->
 * | [m0 = c0_x | m3 = c1_x | m6 = c2_x]
 * | [m1 = c0_y | m4 = c1_y | m7 = c2_y]
 * V [m2 = c0_z | m5 = c1_z | m8 = c2_z]
 *
 *  aij => i = row, j = col (yx form)
 *
 *****************************************************************************/
impl<T: Scalar> Matrix3<T> {
    pub fn new(m0: T, m1: T, m2: T,
        m3: T, m4: T, m5: T,
        m6: T, m7: T, m8: T) -> Self {
        Matrix3 { col: [
            Vector3::new(m0, m1, m2),
            Vector3::new(m3, m4, m5),
            Vector3::new(m6, m7, m8)
            ]
        }
    }

    pub fn identity() -> Self {
        Self::new(
            T::one(), T::zero(), T::zero(),
            T::zero(), T::one(), T::zero(),
            T::zero(), T::zero(), T::one())
    }

    pub fn determinant(&self) -> T {
        let	m00 = self.col[0].x;
        let	m10 = self.col[0].y;
        let	m20 = self.col[0].z;

        let	m01 = self.col[1].x;
        let	m11 = self.col[1].y;
        let	m21 = self.col[1].z;

        let	m02 = self.col[2].x;
        let	m12 = self.col[2].y;
        let	m22 = self.col[2].z;

        m00 * m11 * m22 +
        m01 * m12 * m20 +
        m02 * m10 * m21 -
        m00 * m12 * m21 -
        m01 * m10 * m22 -
        m02 * m11 * m20
    }

    pub fn transpose(&self) -> Self {
        let	m00 = self.col[0].x;
        let	m10 = self.col[0].y;
        let	m20 = self.col[0].z;

        let	m01 = self.col[1].x;
        let	m11 = self.col[1].y;
        let	m21 = self.col[1].z;

        let	m02 = self.col[2].x;
        let	m12 = self.col[2].y;
        let	m22 = self.col[2].z;

        Self::new(m00, m01, m02,
                m10, m11, m12,
                m20, m21, m22)

    }

    pub fn inverse(&self) -> Self {
        let	m00 = self.col[0].x;
        let	m10 = self.col[0].y;
        let	m20 = self.col[0].z;

        let	m01 = self.col[1].x;
        let	m11 = self.col[1].y;
        let	m21 = self.col[1].z;

        let	m02 = self.col[2].x;
        let	m12 = self.col[2].y;
        let	m22 = self.col[2].z;

        let	inv_det = T::one() / (m00 * m11 * m22 +
                      m01 * m12 * m20 +
                      m02 * m10 * m21 -
                      m00 * m12 * m21 -
                      m01 * m10 * m22 -
                      m02 * m11 * m20);

        let	r00 = (m11 * m22 - m12 * m21) * inv_det;
        let	r01 = (m02 * m21 - m01 * m22) * inv_det;
        let	r02 = (m01 * m12 - m02 * m11) * inv_det;
        let	r10 = (m12 * m20 - m10 * m22) * inv_det;
        let	r11 = (m00 * m22 - m02 * m20) * inv_det;
        let	r12 = (m02 * m10 - m00 * m12) * inv_det;
        let	r20 = (m10 * m21 - m11 * m20) * inv_det;
        let	r21 = (m01 * m20 - m00 * m21) * inv_det;
        let	r22 = (m00 * m11 - m01 * m10) * inv_det;

        Self::new(r00, r10, r20,
            r01, r11, r21,
            r02, r12, r22)
    }

    //
    //                 [b00 b01 b02]
    //                 [b10 b11 b12]
    //                 [b20 b21 b22]
    //               *   |   |   |
    // [a00 a01 a02] - [c00 c01 c02]
    // [a10 a11 a12] - [c10 c11 c12]
    // [a20 a21 a22] - [c10 c11 c22]
    //
    pub fn mul_matrix_matrix(l: &Self, r: &Self) -> Self {
        let	a00 = l.col[0].x;
        let	a10 = l.col[0].y;
        let	a20 = l.col[0].z;

        let	a01 = l.col[1].x;
        let	a11 = l.col[1].y;
        let	a21 = l.col[1].z;

        let	a02 = l.col[2].x;
        let	a12 = l.col[2].y;
        let	a22 = l.col[2].z;

        let	b00 = r.col[0].x;
        let	b10 = r.col[0].y;
        let	b20 = r.col[0].z;

        let	b01 = r.col[1].x;
        let	b11 = r.col[1].y;
        let	b21 = r.col[1].z;

        let	b02 = r.col[2].x;
        let	b12 = r.col[2].y;
        let	b22 = r.col[2].z;

        let	c00 = a00 * b00 + a01 * b10 + a02 * b20;
        let	c01 = a00 * b01 + a01 * b11 + a02 * b21;
        let	c02 = a00 * b02 + a01 * b12 + a02 * b22;

        let	c10 = a10 * b00 + a11 * b10 + a12 * b20;
        let	c11 = a10 * b01 + a11 * b11 + a12 * b21;
        let	c12 = a10 * b02 + a11 * b12 + a12 * b22;

        let	c20 = a20 * b00 + a21 * b10 + a22 * b20;
        let	c21 = a20 * b01 + a21 * b11 + a22 * b21;
        let	c22 = a20 * b02 + a21 * b12 + a22 * b22;

        Self::new(c00, c10, c20,
                c01, c11, c21,
                c02, c12, c22)
    }

    pub fn mul_matrix_vector(l: &Self, r: &Vector3<T>) -> Vector3<T> {
        Self::mul_vector_matrix(r, &l.transpose())
    }

    pub fn mul_vector_matrix(l: &Vector3<T>, r: &Self) -> Vector3<T> {
        Vector3::new(Vector3::dot(l, &r.col[0]), Vector3::dot(l, &r.col[1]), Vector3::dot(l, &r.col[2]))
    }

    pub fn add_matrix_matrix(l: &Self, r: &Self) -> Self {
        Matrix3 { col : [
            l.col[0] + r.col[0],
            l.col[1] + r.col[1],
            l.col[2] + r.col[2],
        ]}
    }

    pub fn sub_matrix_matrix(l: &Self, r: &Self) -> Self {
        Matrix3 { col : [
            l.col[0] - r.col[0],
            l.col[1] - r.col[1],
            l.col[2] - r.col[2],
        ]}
    }
}

impl<T: FloatScalar> Matrix3<T> {
    pub fn of_axis_angle(axis: &Vector3<T>, angle: T) -> Self {
        let c = T::tcos(angle);
        let s = T::tsin(angle);
        let n = Vector3::normalize(axis);
        let ux  = n.x;
        let uy  = n.y;
        let uz  = n.z;
        let uxx = ux * ux;
        let uyy = uy * uy;
        let uzz = uz * uz;

        let oc = T::one() - c;

        let m0 = c + uxx * oc;
        let m1 = uy * ux * oc + uz * s;
        let m2 = uz * ux * oc - uy * s;

        let m3 = ux * uy * oc - uz * s;
        let m4 = c + uyy * oc;
        let m5 = uz * uy * oc + ux * s;

        let m6 = ux * uz * oc + uy * s;
        let m7 = uy * uz * oc - ux * s;
        let m8 = c + uzz * oc;

        Self::new(m0, m1, m2, m3, m4, m5, m6, m7, m8)
    }
}

/******************************************************************************
 * Matrix4
 *
 * i j -------------------------------------------->
 * | [m0 = c0_x | m4 = c1_x | m8 = c2_x | m12= c3_x]
 * | [m1 = c0_y | m5 = c1_y | m9 = c2_y | m13= c3_y]
 * | [m2 = c0_z | m6 = c1_z | m10= c2_z | m14= c3_z]
 * V [m3 = c0_w | m7 = c1_w | m11= c2_w | m15= c3_w]
 *
 *  aij => i = row, j = col (yx form)
 *
 *****************************************************************************/
 impl<T: Scalar> Matrix4<T> {
    pub fn new(m0: T, m1: T, m2: T, m3: T,
        m4: T, m5: T, m6: T, m7: T,
        m8: T, m9: T, m10: T, m11: T,
        m12: T, m13: T, m14: T, m15: T) -> Self {
        Matrix4 { col: [
            Vector4::new(m0, m1, m2, m3),
            Vector4::new(m4, m5, m6, m7),
            Vector4::new(m8, m9, m10, m11),
            Vector4::new(m12, m13, m14, m15),
            ]
        }
    }

    pub fn identity() -> Self {
        Self::new(
            T::one(), T::zero(), T::zero(), T::zero(),
            T::zero(), T::one(), T::zero(), T::zero(),
            T::zero(), T::zero(), T::one(), T::zero(),
            T::zero(), T::zero(), T::zero(), T::one())
    }

    pub fn determinant(&self) -> T {
        let	m00 = self.col[0].x;
        let	m10 = self.col[0].y;
        let	m20 = self.col[0].z;
        let	m30 = self.col[0].w;

        let	m01 = self.col[1].x;
        let	m11 = self.col[1].y;
        let	m21 = self.col[1].z;
        let	m31 = self.col[1].w;

        let	m02 = self.col[2].x;
        let	m12 = self.col[2].y;
        let	m22 = self.col[2].z;
        let	m32 = self.col[2].w;

        let	m03 = self.col[3].x;
        let	m13 = self.col[3].y;
        let	m23 = self.col[3].z;
        let	m33 = self.col[3].w;

        m03 * m12 * m21 * m30 - m02 * m13 * m21 * m30 -
        m03 * m11 * m22 * m30 + m01 * m13 * m22 * m30 +
        m02 * m11 * m23 * m30 - m01 * m12 * m23 * m30 -
        m03 * m12 * m20 * m31 + m02 * m13 * m20 * m31 +
        m03 * m10 * m22 * m31 - m00 * m13 * m22 * m31 -
        m02 * m10 * m23 * m31 + m00 * m12 * m23 * m31 +
        m03 * m11 * m20 * m32 - m01 * m13 * m20 * m32 -
        m03 * m10 * m21 * m32 + m00 * m13 * m21 * m32 +
        m01 * m10 * m23 * m32 - m00 * m11 * m23 * m32 -
        m02 * m11 * m20 * m33 + m01 * m12 * m20 * m33 +
        m02 * m10 * m21 * m33 - m00 * m12 * m21 * m33 -
        m01 * m10 * m22 * m33 + m00 * m11 * m22 * m33
    }

    pub fn transpose(&self) -> Self {
        let	m00 = self.col[0].x;
        let	m10 = self.col[0].y;
        let	m20 = self.col[0].z;
        let	m30 = self.col[0].w;

        let	m01 = self.col[1].x;
        let	m11 = self.col[1].y;
        let	m21 = self.col[1].z;
        let	m31 = self.col[1].w;

        let	m02 = self.col[2].x;
        let	m12 = self.col[2].y;
        let	m22 = self.col[2].z;
        let	m32 = self.col[2].w;

        let	m03 = self.col[3].x;
        let	m13 = self.col[3].y;
        let	m23 = self.col[3].z;
        let	m33 = self.col[3].w;

        Self::new(m00, m01, m02, m03,
                m10, m11, m12, m13,
                m20, m21, m22, m23,
                m30, m31, m32, m33)
    }

    pub fn inverse(&self) -> Self {
        let	m00 = self.col[0].x;
        let	m10 = self.col[0].y;
        let	m20 = self.col[0].z;
        let	m30 = self.col[0].w;

        let	m01 = self.col[1].x;
        let	m11 = self.col[1].y;
        let	m21 = self.col[1].z;
        let	m31 = self.col[1].w;

        let	m02 = self.col[2].x;
        let	m12 = self.col[2].y;
        let	m22 = self.col[2].z;
        let	m32 = self.col[2].w;

        let	m03 = self.col[3].x;
        let	m13 = self.col[3].y;
        let	m23 = self.col[3].z;
        let	m33 = self.col[3].w;

        let	denom	= m03 * m12 * m21 * m30 - m02 * m13 * m21 * m30 -
                   m03 * m11 * m22 * m30 + m01 * m13 * m22 * m30 +
                   m02 * m11 * m23 * m30 - m01 * m12 * m23 * m30 -
                   m03 * m12 * m20 * m31 + m02 * m13 * m20 * m31 +
                   m03 * m10 * m22 * m31 - m00 * m13 * m22 * m31 -
                   m02 * m10 * m23 * m31 + m00 * m12 * m23 * m31 +
                   m03 * m11 * m20 * m32 - m01 * m13 * m20 * m32 -
                   m03 * m10 * m21 * m32 + m00 * m13 * m21 * m32 +
                   m01 * m10 * m23 * m32 - m00 * m11 * m23 * m32 -
                   m02 * m11 * m20 * m33 + m01 * m12 * m20 * m33 +
                   m02 * m10 * m21 * m33 - m00 * m12 * m21 * m33 -
                   m01 * m10 * m22 * m33 + m00 * m11 * m22 * m33;
        let	inv_det = T::one() / denom;

        let	r00 = (m12 * m23 * m31 - m13 * m22 * m31 +
                   m13 * m21 * m32 - m11 * m23 * m32 -
                   m12 * m21 * m33 + m11 * m22 * m33) * inv_det;

        let	r01 = (m03 * m22 * m31 - m02 * m23 * m31 -
                   m03 * m21 * m32 + m01 * m23 * m32 +
                   m02 * m21 * m33 - m01 * m22 * m33) * inv_det;

        let	r02 = (m02 * m13 * m31 - m03 * m12 * m31 +
                   m03 * m11 * m32 - m01 * m13 * m32 -
                   m02 * m11 * m33 + m01 * m12 * m33) * inv_det;

        let	r03 = (m03 * m12 * m21 - m02 * m13 * m21 -
                   m03 * m11 * m22 + m01 * m13 * m22 +
                   m02 * m11 * m23 - m01 * m12 * m23) * inv_det;

        let	r10 = (m13 * m22 * m30 - m12 * m23 * m30 -
                   m13 * m20 * m32 + m10 * m23 * m32 +
                   m12 * m20 * m33 - m10 * m22 * m33) * inv_det;

        let	r11 = (m02 * m23 * m30 - m03 * m22 * m30 +
                   m03 * m20 * m32 - m00 * m23 * m32 -
                   m02 * m20 * m33 + m00 * m22 * m33) * inv_det;

        let	r12 = (m03 * m12 * m30 - m02 * m13 * m30 -
                   m03 * m10 * m32 + m00 * m13 * m32 +
                   m02 * m10 * m33 - m00 * m12 * m33) * inv_det;

        let	r13 = (m02 * m13 * m20 - m03 * m12 * m20 +
                   m03 * m10 * m22 - m00 * m13 * m22 -
                   m02 * m10 * m23 + m00 * m12 * m23) * inv_det;

        let	r20 = (m11 * m23 * m30 - m13 * m21 * m30 +
                   m13 * m20 * m31 - m10 * m23 * m31 -
                   m11 * m20 * m33 + m10 * m21 * m33) * inv_det;

        let	r21 = (m03 * m21 * m30 - m01 * m23 * m30 -
                   m03 * m20 * m31 + m00 * m23 * m31 +
                   m01 * m20 * m33 - m00 * m21 * m33) * inv_det;

        let	r22 = (m01 * m13 * m30 - m03 * m11 * m30 +
                   m03 * m10 * m31 - m00 * m13 * m31 -
                   m01 * m10 * m33 + m00 * m11 * m33) * inv_det;

        let	r23 = (m03 * m11 * m20 - m01 * m13 * m20 -
                   m03 * m10 * m21 + m00 * m13 * m21 +
                   m01 * m10 * m23 - m00 * m11 * m23) * inv_det;

        let	r30 = (m12 * m21 * m30 - m11 * m22 * m30 -
                   m12 * m20 * m31 + m10 * m22 * m31 +
                   m11 * m20 * m32 - m10 * m21 * m32) * inv_det;

        let	r31 = (m01 * m22 * m30 - m02 * m21 * m30 +
                   m02 * m20 * m31 - m00 * m22 * m31 -
                   m01 * m20 * m32 + m00 * m21 * m32) * inv_det;

        let	r32 = (m02 * m11 * m30 - m01 * m12 * m30 -
                   m02 * m10 * m31 + m00 * m12 * m31 +
                   m01 * m10 * m32 - m00 * m11 * m32) * inv_det;

        let	r33 = (m01 * m12 * m20 - m02 * m11 * m20 +
                   m02 * m10 * m21 - m00 * m12 * m21 -
                   m01 * m10 * m22 + m00 * m11 * m22) * inv_det;

        Self::new(r00, r10, r20, r30,
                r01, r11, r21, r31,
                r02, r12, r22, r32,
                r03, r13, r23, r33)
    }

    //
    //                     [b00 b01 b02 b03]
    //                     [b10 b11 b12 b13]
    //                     [b20 b21 b22 b23]
    //                     [b20 b21 b22 b33]
    //                   *   |   |   |   |
    // [a00 a01 a02 a03] - [c00 c01 c02 c03]
    // [a10 a11 a12 a13] - [c10 c11 c12 c13]
    // [a20 a21 a22 a23] - [c10 c11 c22 c23]
    // [a20 a21 a22 a33] - [c10 c11 c22 c33]
    //
    pub fn mul_matrix_matrix(l: &Self, r: &Self) -> Self {
        let	a00 = l.col[0].x;
        let	a10 = l.col[0].y;
        let	a20 = l.col[0].z;
        let	a30 = l.col[0].w;

        let	a01 = l.col[1].x;
        let	a11 = l.col[1].y;
        let	a21 = l.col[1].z;
        let	a31 = l.col[1].w;

        let	a02 = l.col[2].x;
        let	a12 = l.col[2].y;
        let	a22 = l.col[2].z;
        let	a32 = l.col[2].w;

        let	a03 = l.col[3].x;
        let	a13 = l.col[3].y;
        let	a23 = l.col[3].z;
        let	a33 = l.col[3].w;

        let	b00 = r.col[0].x;
        let	b10 = r.col[0].y;
        let	b20 = r.col[0].z;
        let	b30 = r.col[0].w;

        let	b01 = r.col[1].x;
        let	b11 = r.col[1].y;
        let	b21 = r.col[1].z;
        let	b31 = r.col[1].w;

        let	b02 = r.col[2].x;
        let	b12 = r.col[2].y;
        let	b22 = r.col[2].z;
        let	b32 = r.col[2].w;

        let	b03 = r.col[3].x;
        let	b13 = r.col[3].y;
        let	b23 = r.col[3].z;
        let	b33 = r.col[3].w;

        let	c00 = a00 * b00 + a01 * b10 + a02 * b20 + a03 * b30;
        let	c01 = a00 * b01 + a01 * b11 + a02 * b21 + a03 * b31;
        let	c02 = a00 * b02 + a01 * b12 + a02 * b22 + a03 * b32;
        let	c03 = a00 * b03 + a01 * b13 + a02 * b23 + a03 * b33;

        let	c10 = a10 * b00 + a11 * b10 + a12 * b20 + a13 * b30;
        let	c11 = a10 * b01 + a11 * b11 + a12 * b21 + a13 * b31;
        let	c12 = a10 * b02 + a11 * b12 + a12 * b22 + a13 * b32;
        let	c13 = a10 * b03 + a11 * b13 + a12 * b23 + a13 * b33;

        let	c20 = a20 * b00 + a21 * b10 + a22 * b20 + a23 * b30;
        let	c21 = a20 * b01 + a21 * b11 + a22 * b21 + a23 * b31;
        let	c22 = a20 * b02 + a21 * b12 + a22 * b22 + a23 * b32;
        let	c23 = a20 * b03 + a21 * b13 + a22 * b23 + a23 * b33;

        let	c30 = a30 * b00 + a31 * b10 + a32 * b20 + a33 * b30;
        let	c31 = a30 * b01 + a31 * b11 + a32 * b21 + a33 * b31;
        let	c32 = a30 * b02 + a31 * b12 + a32 * b22 + a33 * b32;
        let	c33 = a30 * b03 + a31 * b13 + a32 * b23 + a33 * b33;

        Self::new(c00, c10, c20, c30,
                c01, c11, c21, c31,
                c02, c12, c22, c32,
                c03, c13, c23, c33)
    }

    pub fn mul_matrix_vector(l: &Self, r: &Vector4<T>) -> Vector4<T> {
        Self::mul_vector_matrix(r, &l.transpose())
    }

    //
    //                     [m0 = c0_x | m4 = c1_x | m8 = c2_x | m12= c3_x]
    // [v_x v_y v_z v_w] * [m1 = c0_y | m5 = c1_y | m9 = c2_y | m13= c3_y] = [dot(v, c0) dot(v, c1) dot(v, c2) dot(v, c3)]
    //                     [m2 = c0_z | m6 = c1_z | m10= c2_z | m14= c3_z]
    //                     [m3 = c0_w | m7 = c1_w | m11= c2_w | m15= c3_w]
    //
    pub fn mul_vector_matrix(l: &Vector4<T>, r: &Self) -> Vector4<T> {
        Vector4::new(
            Vector4::dot(l, &r.col[0]),
            Vector4::dot(l, &r.col[1]),
            Vector4::dot(l, &r.col[2]),
            Vector4::dot(l, &r.col[3]))
    }

    pub fn add_matrix_matrix(l: &Self, r: &Self) -> Self {
        Matrix4 { col : [
            l.col[0] + r.col[0],
            l.col[1] + r.col[1],
            l.col[2] + r.col[2],
            l.col[3] + r.col[3],
        ]}
    }

    pub fn sub_matrix_matrix(l: &Self, r: &Self) -> Self {
        Matrix4 { col : [
            l.col[0] - r.col[0],
            l.col[1] - r.col[1],
            l.col[2] - r.col[2],
            l.col[3] - r.col[3],
        ]}
    }
}

/******************************************************************************
 * Operator overloading
 *****************************************************************************/
macro_rules! implMatrixOps {
    ($mat:ident, $vec: ident) => {
        impl<T: Scalar> Mul<$mat<T>> for $vec<T> {
            type Output = $vec<T>;
            fn mul(self, rhs: $mat<T>) -> $vec<T> {
                $mat::mul_vector_matrix(&self, &rhs)
            }
        }

        impl<T: Scalar> Mul<$vec<T>> for $mat<T> {
            type Output = $vec<T>;
            fn mul(self, rhs: $vec<T>) -> $vec<T> {
                $mat::mul_matrix_vector(&self, &rhs)
            }
        }

        impl<T: Scalar> Mul<$mat<T>> for $mat<T> {
            type Output = $mat<T>;
            fn mul(self, rhs: $mat<T>) -> $mat<T> {
                $mat::mul_matrix_matrix(&self, &rhs)
            }
        }

        impl<T: Scalar> Add<$mat<T>> for $mat<T> {
            type Output = $mat<T>;
            fn add(self, rhs: $mat<T>) -> $mat<T> {
                $mat::add_matrix_matrix(&self, &rhs)
            }
        }

        impl<T: Scalar> Sub<$mat<T>> for $mat<T> {
            type Output = $mat<T>;
            fn sub(self, rhs: $mat<T>) -> $mat<T> {
                $mat::sub_matrix_matrix(&self, &rhs)
            }
        }
    };
}

implMatrixOps!(Matrix2, Vector2);
implMatrixOps!(Matrix3, Vector3);
implMatrixOps!(Matrix4, Vector4);

impl<T: Scalar> Mul<Matrix4<T>> for Vector3<T> {
    type Output = Vector3<T>;
    fn mul(self, rhs: Matrix4<T>) -> Vector3<T> {
        Matrix4::mul_vector_matrix(&Vector4::new(self.x, self.y, self.z, T::one()), &rhs).xyz()
    }
}

impl<T: Scalar> Mul<Vector3<T>> for Matrix4<T> {
    type Output = Vector3<T>;
    fn mul(self, rhs: Vector3<T>) -> Vector3<T> {
        Matrix4::mul_matrix_vector(&self, &Vector4::new(rhs.x, rhs.y, rhs.z, T::one())).xyz()
    }
}

pub trait Matrix4Extension<T : Scalar> {
    fn mat3(&self) -> Matrix3<T>;
}

impl<T: Scalar> Matrix4Extension<T> for Matrix4<T> {
    fn mat3(&self) -> Matrix3<T> {
        Matrix3::new(self.col[0].x, self.col[0].y, self.col[0].z,
                     self.col[1].x, self.col[1].y, self.col[1].z,
                     self.col[2].x, self.col[2].y, self.col[2].z)
    }
}
