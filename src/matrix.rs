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
//! Matrix mathematics module providing 2x2, 3x3, and 4x4 matrices.
//!
//! This module provides square matrix types commonly used in computer graphics
//! and linear algebra. Matrices are stored in column-major order for compatibility
//! with graphics APIs like OpenGL.
//!
//! # Examples
//!
//! ```
//! use rs_math3d::matrix::Matrix4;
//! use rs_math3d::vector::Vector4;
//! 
//! let m = Matrix4::<f32>::identity();
//! let v = Vector4::new(1.0, 2.0, 3.0, 1.0);
//! let result = m * v; // Transform vector
//! ```

use crate::scalar::*;
use crate::vector::*;
use num_traits::{Zero, One};
use core::ops::*;

/// A 2x2 matrix stored in column-major order.
///
/// # Layout
/// ```text
/// [m₀₀ m₀₁]
/// [m₁₀ m₁₁]
/// ```
/// where `col[j][i]` represents element at row i, column j.
#[repr(C)]
#[derive(Clone, Copy, Debug)]
pub struct Matrix2<T: Scalar> {
    /// Column vectors of the matrix
    pub col: [Vector2<T>; 2],
}

/// A 3x3 matrix stored in column-major order.
///
/// Commonly used for 2D transformations (with homogeneous coordinates)
/// and 3D rotations.
///
/// # Layout
/// ```text
/// [m₀₀ m₀₁ m₀₂]
/// [m₁₀ m₁₁ m₁₂]
/// [m₂₀ m₂₁ m₂₂]
/// ```
#[repr(C)]
#[derive(Clone, Copy, Debug)]
pub struct Matrix3<T: Scalar> {
    /// Column vectors of the matrix
    pub col: [Vector3<T>; 3],
}

/// A 4x4 matrix stored in column-major order.
///
/// The standard matrix for 3D transformations using homogeneous coordinates.
///
/// # Layout
/// ```text
/// [m₀₀ m₀₁ m₀₂ m₀₃]
/// [m₁₀ m₁₁ m₁₂ m₁₃]
/// [m₂₀ m₂₁ m₂₂ m₂₃]
/// [m₃₀ m₃₁ m₃₂ m₃₃]
/// ```
#[repr(C)]
#[derive(Clone, Copy, Debug)]
pub struct Matrix4<T: Scalar> {
    /// Column vectors of the matrix
    pub col: [Vector4<T>; 4],
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
    /// Creates a new 2x2 matrix from individual elements.
    ///
    /// Elements are provided in column-major order:
    /// ```text
    /// [m0 m2]
    /// [m1 m3]
    /// ```
    pub fn new(m0: T, m1: T, m2: T, m3: T) -> Self {
        Matrix2 {
            col: [Vector2::new(m0, m1), Vector2::new(m2, m3)],
        }
    }

    /// Returns the 2x2 identity matrix.
    ///
    /// ```text
    /// [1 0]
    /// [0 1]
    /// ```
    pub fn identity() -> Self {
        Self::new(<T as One>::one(), <T as Zero>::zero(), <T as Zero>::zero(), <T as One>::one())
    }

    /// Computes the determinant of the matrix.
    ///
    /// For a 2x2 matrix:
    /// ```text
    /// det(M) = m₀₀m₁₁ - m₀₁m₁₀
    /// ```
    pub fn determinant(&self) -> T {
        let m00 = self.col[0].x;
        let m10 = self.col[0].y;

        let m01 = self.col[1].x;
        let m11 = self.col[1].y;

        m00 * m11 - m01 * m10
    }

    /// Returns the transpose of the matrix.
    ///
    /// ```text
    /// Mᵀ[i,j] = M[j,i]
    /// ```
    pub fn transpose(&self) -> Self {
        let m00 = self.col[0].x;
        let m10 = self.col[0].y;

        let m01 = self.col[1].x;
        let m11 = self.col[1].y;

        Self::new(m00, m01, m10, m11)
    }

    /// Computes the inverse of the matrix.
    ///
    /// For a 2x2 matrix:
    /// ```text
    /// M⁻¹ = (1/det(M)) * [m₁₁  -m₀₁]
    ///                    [-m₁₀  m₀₀]
    /// ```
    ///
    /// # Note
    /// Returns NaN or Inf if the matrix is singular (determinant = 0).
    pub fn inverse(&self) -> Self {
        let m00 = self.col[0].x;
        let m10 = self.col[0].y;

        let m01 = self.col[1].x;
        let m11 = self.col[1].y;

        let inv_det = <T as One>::one() / (m00 * m11 - m01 * m10);

        let r00 = m11 * inv_det;
        let r01 = -m01 * inv_det;
        let r10 = -m10 * inv_det;
        let r11 = m00 * inv_det;

        Self::new(r00, r10, r01, r11)
    }

    /// Multiplies two 2x2 matrices.
    ///
    /// Matrix multiplication follows the rule:
    /// ```text
    /// C[i,j] = Σₖ A[i,k] * B[k,j]
    /// ```
    pub fn mul_matrix_matrix(l: &Self, r: &Self) -> Self {
        let a00 = l.col[0].x;
        let a10 = l.col[0].y;
        let a01 = l.col[1].x;
        let a11 = l.col[1].y;

        let b00 = r.col[0].x;
        let b10 = r.col[0].y;
        let b01 = r.col[1].x;
        let b11 = r.col[1].y;

        let c00 = a00 * b00 + a01 * b10;
        let c01 = a00 * b01 + a01 * b11;
        let c10 = a10 * b00 + a11 * b10;
        let c11 = a10 * b01 + a11 * b11;

        Self::new(c00, c10, c01, c11)
    }

    /// Multiplies a 2x2 matrix by a 2D vector.
    ///
    /// Transforms the vector by the matrix:
    /// ```text
    /// v' = M * v
    /// ```
    pub fn mul_matrix_vector(l: &Self, r: &Vector2<T>) -> Vector2<T> {
        Self::mul_vector_matrix(r, &l.transpose())
    }

    /// Multiplies a 2D vector by a 2x2 matrix (row vector).
    ///
    /// ```text
    /// v' = vᵀ * M
    /// ```
    pub fn mul_vector_matrix(l: &Vector2<T>, r: &Self) -> Vector2<T> {
        Vector2::new(Vector2::dot(l, &r.col[0]), Vector2::dot(l, &r.col[1]))
    }

    /// Adds two matrices element-wise.
    ///
    /// ```text
    /// C[i,j] = A[i,j] + B[i,j]
    /// ```
    pub fn add_matrix_matrix(l: &Self, r: &Self) -> Self {
        Matrix2 {
            col: [l.col[0] + r.col[0], l.col[1] + r.col[1]],
        }
    }

    /// Subtracts two matrices element-wise.
    ///
    /// ```text
    /// C[i,j] = A[i,j] - B[i,j]
    /// ```
    pub fn sub_matrix_matrix(l: &Self, r: &Self) -> Self {
        Matrix2 {
            col: [l.col[0] - r.col[0], l.col[1] - r.col[1]],
        }
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
    pub fn new(m0: T, m1: T, m2: T, m3: T, m4: T, m5: T, m6: T, m7: T, m8: T) -> Self {
        Matrix3 {
            col: [
                Vector3::new(m0, m1, m2),
                Vector3::new(m3, m4, m5),
                Vector3::new(m6, m7, m8),
            ],
        }
    }

    pub fn identity() -> Self {
        Self::new(
            <T as One>::one(),
            <T as Zero>::zero(),
            <T as Zero>::zero(),
            <T as Zero>::zero(),
            <T as One>::one(),
            <T as Zero>::zero(),
            <T as Zero>::zero(),
            <T as Zero>::zero(),
            <T as One>::one(),
        )
    }

    pub fn determinant(&self) -> T {
        let m00 = self.col[0].x;
        let m10 = self.col[0].y;
        let m20 = self.col[0].z;

        let m01 = self.col[1].x;
        let m11 = self.col[1].y;
        let m21 = self.col[1].z;

        let m02 = self.col[2].x;
        let m12 = self.col[2].y;
        let m22 = self.col[2].z;

        m00 * m11 * m22 + m01 * m12 * m20 + m02 * m10 * m21
            - m00 * m12 * m21
            - m01 * m10 * m22
            - m02 * m11 * m20
    }

    /// Returns the transpose of the matrix.
    ///
    /// ```text
    /// Mᵀ[i,j] = M[j,i]
    /// ```
    pub fn transpose(&self) -> Self {
        let m00 = self.col[0].x;
        let m10 = self.col[0].y;
        let m20 = self.col[0].z;

        let m01 = self.col[1].x;
        let m11 = self.col[1].y;
        let m21 = self.col[1].z;

        let m02 = self.col[2].x;
        let m12 = self.col[2].y;
        let m22 = self.col[2].z;

        Self::new(m00, m01, m02, m10, m11, m12, m20, m21, m22)
    }

    /// Computes the inverse of the matrix.
    ///
    /// Uses the adjugate matrix method:
    /// ```text
    /// M⁻¹ = (1/det(M)) * adj(M)
    /// ```
    ///
    /// # Note
    /// Returns NaN or Inf if the matrix is singular (determinant = 0).
    pub fn inverse(&self) -> Self {
        let m00 = self.col[0].x;
        let m10 = self.col[0].y;
        let m20 = self.col[0].z;

        let m01 = self.col[1].x;
        let m11 = self.col[1].y;
        let m21 = self.col[1].z;

        let m02 = self.col[2].x;
        let m12 = self.col[2].y;
        let m22 = self.col[2].z;

        let inv_det = <T as One>::one()
            / (m00 * m11 * m22 + m01 * m12 * m20 + m02 * m10 * m21
                - m00 * m12 * m21
                - m01 * m10 * m22
                - m02 * m11 * m20);

        let r00 = (m11 * m22 - m12 * m21) * inv_det;
        let r01 = (m02 * m21 - m01 * m22) * inv_det;
        let r02 = (m01 * m12 - m02 * m11) * inv_det;
        let r10 = (m12 * m20 - m10 * m22) * inv_det;
        let r11 = (m00 * m22 - m02 * m20) * inv_det;
        let r12 = (m02 * m10 - m00 * m12) * inv_det;
        let r20 = (m10 * m21 - m11 * m20) * inv_det;
        let r21 = (m01 * m20 - m00 * m21) * inv_det;
        let r22 = (m00 * m11 - m01 * m10) * inv_det;

        Self::new(r00, r10, r20, r01, r11, r21, r02, r12, r22)
    }

    /// Multiplies two 3x3 matrices.
    ///
    /// Matrix multiplication follows the rule:
    /// ```text
    /// C[i,j] = Σₖ A[i,k] * B[k,j]
    /// ```
    pub fn mul_matrix_matrix(l: &Self, r: &Self) -> Self {
        let a00 = l.col[0].x;
        let a10 = l.col[0].y;
        let a20 = l.col[0].z;

        let a01 = l.col[1].x;
        let a11 = l.col[1].y;
        let a21 = l.col[1].z;

        let a02 = l.col[2].x;
        let a12 = l.col[2].y;
        let a22 = l.col[2].z;

        let b00 = r.col[0].x;
        let b10 = r.col[0].y;
        let b20 = r.col[0].z;

        let b01 = r.col[1].x;
        let b11 = r.col[1].y;
        let b21 = r.col[1].z;

        let b02 = r.col[2].x;
        let b12 = r.col[2].y;
        let b22 = r.col[2].z;

        let c00 = a00 * b00 + a01 * b10 + a02 * b20;
        let c01 = a00 * b01 + a01 * b11 + a02 * b21;
        let c02 = a00 * b02 + a01 * b12 + a02 * b22;

        let c10 = a10 * b00 + a11 * b10 + a12 * b20;
        let c11 = a10 * b01 + a11 * b11 + a12 * b21;
        let c12 = a10 * b02 + a11 * b12 + a12 * b22;

        let c20 = a20 * b00 + a21 * b10 + a22 * b20;
        let c21 = a20 * b01 + a21 * b11 + a22 * b21;
        let c22 = a20 * b02 + a21 * b12 + a22 * b22;

        Self::new(c00, c10, c20, c01, c11, c21, c02, c12, c22)
    }

    /// Multiplies a 3x3 matrix by a 3D vector.
    ///
    /// Transforms the vector by the matrix:
    /// ```text
    /// v' = M * v
    /// ```
    pub fn mul_matrix_vector(l: &Self, r: &Vector3<T>) -> Vector3<T> {
        Self::mul_vector_matrix(r, &l.transpose())
    }

    /// Multiplies a 3D vector by a 3x3 matrix (row vector).
    ///
    /// ```text
    /// v' = vᵀ * M
    /// ```
    pub fn mul_vector_matrix(l: &Vector3<T>, r: &Self) -> Vector3<T> {
        Vector3::new(
            Vector3::dot(l, &r.col[0]),
            Vector3::dot(l, &r.col[1]),
            Vector3::dot(l, &r.col[2]),
        )
    }

    /// Adds two matrices element-wise.
    ///
    /// ```text
    /// C[i,j] = A[i,j] + B[i,j]
    /// ```
    pub fn add_matrix_matrix(l: &Self, r: &Self) -> Self {
        Matrix3 {
            col: [
                l.col[0] + r.col[0],
                l.col[1] + r.col[1],
                l.col[2] + r.col[2],
            ],
        }
    }

    /// Subtracts two matrices element-wise.
    ///
    /// ```text
    /// C[i,j] = A[i,j] - B[i,j]
    /// ```
    pub fn sub_matrix_matrix(l: &Self, r: &Self) -> Self {
        Matrix3 {
            col: [
                l.col[0] - r.col[0],
                l.col[1] - r.col[1],
                l.col[2] - r.col[2],
            ],
        }
    }
}

impl<T: FloatScalar> Matrix3<T> {
    /// Creates a 3x3 rotation matrix from an axis and angle.
    ///
    /// Uses Rodrigues' rotation formula:
    /// ```text
    /// R = I + sin(θ)K + (1 - cos(θ))K²
    /// ```
    /// where K is the cross-product matrix of the normalized axis.
    ///
    /// # Parameters
    /// - `axis`: The rotation axis (will be normalized)
    /// - `angle`: The rotation angle in radians
    /// - `epsilon`: Minimum axis length to treat as valid
    ///
    /// # Returns
    /// - `Some(matrix)` for a valid axis
    /// - `None` if the axis length is too small
    pub fn of_axis_angle(axis: &Vector3<T>, angle: T, epsilon: T) -> Option<Self> {
        let len_sq = Vector3::dot(axis, axis);
        if len_sq <= epsilon * epsilon {
            return None;
        }
        let inv_len = <T as One>::one() / len_sq.tsqrt();
        let n = *axis * inv_len;
        let c = T::tcos(angle);
        let s = T::tsin(angle);
        let ux = n.x;
        let uy = n.y;
        let uz = n.z;
        let uxx = ux * ux;
        let uyy = uy * uy;
        let uzz = uz * uz;

        let oc = <T as One>::one() - c;

        let m0 = c + uxx * oc;
        let m1 = uy * ux * oc + uz * s;
        let m2 = uz * ux * oc - uy * s;

        let m3 = ux * uy * oc - uz * s;
        let m4 = c + uyy * oc;
        let m5 = uz * uy * oc + ux * s;

        let m6 = ux * uz * oc + uy * s;
        let m7 = uy * uz * oc - ux * s;
        let m8 = c + uzz * oc;

        Some(Self::new(m0, m1, m2, m3, m4, m5, m6, m7, m8))
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
    pub fn new(
        m0: T,
        m1: T,
        m2: T,
        m3: T,
        m4: T,
        m5: T,
        m6: T,
        m7: T,
        m8: T,
        m9: T,
        m10: T,
        m11: T,
        m12: T,
        m13: T,
        m14: T,
        m15: T,
    ) -> Self {
        Matrix4 {
            col: [
                Vector4::new(m0, m1, m2, m3),
                Vector4::new(m4, m5, m6, m7),
                Vector4::new(m8, m9, m10, m11),
                Vector4::new(m12, m13, m14, m15),
            ],
        }
    }

    pub fn identity() -> Self {
        Self::new(
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
            <T as Zero>::zero(),
            <T as Zero>::zero(),
            <T as Zero>::zero(),
            <T as One>::one(),
        )
    }

    /// Computes the determinant of the matrix.
    ///
    /// Uses Laplace expansion along the first column.
    /// A non-zero determinant indicates the matrix is invertible.
    pub fn determinant(&self) -> T {
        let m00 = self.col[0].x;
        let m10 = self.col[0].y;
        let m20 = self.col[0].z;
        let m30 = self.col[0].w;

        let m01 = self.col[1].x;
        let m11 = self.col[1].y;
        let m21 = self.col[1].z;
        let m31 = self.col[1].w;

        let m02 = self.col[2].x;
        let m12 = self.col[2].y;
        let m22 = self.col[2].z;
        let m32 = self.col[2].w;

        let m03 = self.col[3].x;
        let m13 = self.col[3].y;
        let m23 = self.col[3].z;
        let m33 = self.col[3].w;

        m03 * m12 * m21 * m30 - m02 * m13 * m21 * m30 - m03 * m11 * m22 * m30
            + m01 * m13 * m22 * m30
            + m02 * m11 * m23 * m30
            - m01 * m12 * m23 * m30
            - m03 * m12 * m20 * m31
            + m02 * m13 * m20 * m31
            + m03 * m10 * m22 * m31
            - m00 * m13 * m22 * m31
            - m02 * m10 * m23 * m31
            + m00 * m12 * m23 * m31
            + m03 * m11 * m20 * m32
            - m01 * m13 * m20 * m32
            - m03 * m10 * m21 * m32
            + m00 * m13 * m21 * m32
            + m01 * m10 * m23 * m32
            - m00 * m11 * m23 * m32
            - m02 * m11 * m20 * m33
            + m01 * m12 * m20 * m33
            + m02 * m10 * m21 * m33
            - m00 * m12 * m21 * m33
            - m01 * m10 * m22 * m33
            + m00 * m11 * m22 * m33
    }

    /// Returns the transpose of the matrix.
    ///
    /// ```text
    /// Mᵀ[i,j] = M[j,i]
    /// ```
    pub fn transpose(&self) -> Self {
        let m00 = self.col[0].x;
        let m10 = self.col[0].y;
        let m20 = self.col[0].z;
        let m30 = self.col[0].w;

        let m01 = self.col[1].x;
        let m11 = self.col[1].y;
        let m21 = self.col[1].z;
        let m31 = self.col[1].w;

        let m02 = self.col[2].x;
        let m12 = self.col[2].y;
        let m22 = self.col[2].z;
        let m32 = self.col[2].w;

        let m03 = self.col[3].x;
        let m13 = self.col[3].y;
        let m23 = self.col[3].z;
        let m33 = self.col[3].w;

        Self::new(
            m00, m01, m02, m03, m10, m11, m12, m13, m20, m21, m22, m23, m30, m31, m32, m33,
        )
    }

    /// Computes the inverse of the matrix.
    ///
    /// Uses the adjugate matrix method:
    /// ```text
    /// M⁻¹ = (1/det(M)) * adj(M)
    /// ```
    ///
    /// # Note
    /// Returns NaN or Inf if the matrix is singular (determinant = 0).
    pub fn inverse(&self) -> Self {
        let m00 = self.col[0].x;
        let m10 = self.col[0].y;
        let m20 = self.col[0].z;
        let m30 = self.col[0].w;

        let m01 = self.col[1].x;
        let m11 = self.col[1].y;
        let m21 = self.col[1].z;
        let m31 = self.col[1].w;

        let m02 = self.col[2].x;
        let m12 = self.col[2].y;
        let m22 = self.col[2].z;
        let m32 = self.col[2].w;

        let m03 = self.col[3].x;
        let m13 = self.col[3].y;
        let m23 = self.col[3].z;
        let m33 = self.col[3].w;

        let denom = m03 * m12 * m21 * m30 - m02 * m13 * m21 * m30 - m03 * m11 * m22 * m30
            + m01 * m13 * m22 * m30
            + m02 * m11 * m23 * m30
            - m01 * m12 * m23 * m30
            - m03 * m12 * m20 * m31
            + m02 * m13 * m20 * m31
            + m03 * m10 * m22 * m31
            - m00 * m13 * m22 * m31
            - m02 * m10 * m23 * m31
            + m00 * m12 * m23 * m31
            + m03 * m11 * m20 * m32
            - m01 * m13 * m20 * m32
            - m03 * m10 * m21 * m32
            + m00 * m13 * m21 * m32
            + m01 * m10 * m23 * m32
            - m00 * m11 * m23 * m32
            - m02 * m11 * m20 * m33
            + m01 * m12 * m20 * m33
            + m02 * m10 * m21 * m33
            - m00 * m12 * m21 * m33
            - m01 * m10 * m22 * m33
            + m00 * m11 * m22 * m33;
        let inv_det = <T as One>::one() / denom;

        let r00 = (m12 * m23 * m31 - m13 * m22 * m31 + m13 * m21 * m32
            - m11 * m23 * m32
            - m12 * m21 * m33
            + m11 * m22 * m33)
            * inv_det;

        let r01 = (m03 * m22 * m31 - m02 * m23 * m31 - m03 * m21 * m32
            + m01 * m23 * m32
            + m02 * m21 * m33
            - m01 * m22 * m33)
            * inv_det;

        let r02 = (m02 * m13 * m31 - m03 * m12 * m31 + m03 * m11 * m32
            - m01 * m13 * m32
            - m02 * m11 * m33
            + m01 * m12 * m33)
            * inv_det;

        let r03 = (m03 * m12 * m21 - m02 * m13 * m21 - m03 * m11 * m22
            + m01 * m13 * m22
            + m02 * m11 * m23
            - m01 * m12 * m23)
            * inv_det;

        let r10 = (m13 * m22 * m30 - m12 * m23 * m30 - m13 * m20 * m32
            + m10 * m23 * m32
            + m12 * m20 * m33
            - m10 * m22 * m33)
            * inv_det;

        let r11 = (m02 * m23 * m30 - m03 * m22 * m30 + m03 * m20 * m32
            - m00 * m23 * m32
            - m02 * m20 * m33
            + m00 * m22 * m33)
            * inv_det;

        let r12 = (m03 * m12 * m30 - m02 * m13 * m30 - m03 * m10 * m32
            + m00 * m13 * m32
            + m02 * m10 * m33
            - m00 * m12 * m33)
            * inv_det;

        let r13 = (m02 * m13 * m20 - m03 * m12 * m20 + m03 * m10 * m22
            - m00 * m13 * m22
            - m02 * m10 * m23
            + m00 * m12 * m23)
            * inv_det;

        let r20 = (m11 * m23 * m30 - m13 * m21 * m30 + m13 * m20 * m31
            - m10 * m23 * m31
            - m11 * m20 * m33
            + m10 * m21 * m33)
            * inv_det;

        let r21 = (m03 * m21 * m30 - m01 * m23 * m30 - m03 * m20 * m31
            + m00 * m23 * m31
            + m01 * m20 * m33
            - m00 * m21 * m33)
            * inv_det;

        let r22 = (m01 * m13 * m30 - m03 * m11 * m30 + m03 * m10 * m31
            - m00 * m13 * m31
            - m01 * m10 * m33
            + m00 * m11 * m33)
            * inv_det;

        let r23 = (m03 * m11 * m20 - m01 * m13 * m20 - m03 * m10 * m21
            + m00 * m13 * m21
            + m01 * m10 * m23
            - m00 * m11 * m23)
            * inv_det;

        let r30 = (m12 * m21 * m30 - m11 * m22 * m30 - m12 * m20 * m31
            + m10 * m22 * m31
            + m11 * m20 * m32
            - m10 * m21 * m32)
            * inv_det;

        let r31 = (m01 * m22 * m30 - m02 * m21 * m30 + m02 * m20 * m31
            - m00 * m22 * m31
            - m01 * m20 * m32
            + m00 * m21 * m32)
            * inv_det;

        let r32 = (m02 * m11 * m30 - m01 * m12 * m30 - m02 * m10 * m31
            + m00 * m12 * m31
            + m01 * m10 * m32
            - m00 * m11 * m32)
            * inv_det;

        let r33 = (m01 * m12 * m20 - m02 * m11 * m20 + m02 * m10 * m21
            - m00 * m12 * m21
            - m01 * m10 * m22
            + m00 * m11 * m22)
            * inv_det;

        Self::new(
            r00, r10, r20, r30, r01, r11, r21, r31, r02, r12, r22, r32, r03, r13, r23, r33,
        )
    }

    /// Multiplies two 4x4 matrices.
    ///
    /// Matrix multiplication follows the rule:
    /// ```text
    /// C[i,j] = Σₖ A[i,k] * B[k,j]
    /// ```
    pub fn mul_matrix_matrix(l: &Self, r: &Self) -> Self {
        let a00 = l.col[0].x;
        let a10 = l.col[0].y;
        let a20 = l.col[0].z;
        let a30 = l.col[0].w;

        let a01 = l.col[1].x;
        let a11 = l.col[1].y;
        let a21 = l.col[1].z;
        let a31 = l.col[1].w;

        let a02 = l.col[2].x;
        let a12 = l.col[2].y;
        let a22 = l.col[2].z;
        let a32 = l.col[2].w;

        let a03 = l.col[3].x;
        let a13 = l.col[3].y;
        let a23 = l.col[3].z;
        let a33 = l.col[3].w;

        let b00 = r.col[0].x;
        let b10 = r.col[0].y;
        let b20 = r.col[0].z;
        let b30 = r.col[0].w;

        let b01 = r.col[1].x;
        let b11 = r.col[1].y;
        let b21 = r.col[1].z;
        let b31 = r.col[1].w;

        let b02 = r.col[2].x;
        let b12 = r.col[2].y;
        let b22 = r.col[2].z;
        let b32 = r.col[2].w;

        let b03 = r.col[3].x;
        let b13 = r.col[3].y;
        let b23 = r.col[3].z;
        let b33 = r.col[3].w;

        let c00 = a00 * b00 + a01 * b10 + a02 * b20 + a03 * b30;
        let c01 = a00 * b01 + a01 * b11 + a02 * b21 + a03 * b31;
        let c02 = a00 * b02 + a01 * b12 + a02 * b22 + a03 * b32;
        let c03 = a00 * b03 + a01 * b13 + a02 * b23 + a03 * b33;

        let c10 = a10 * b00 + a11 * b10 + a12 * b20 + a13 * b30;
        let c11 = a10 * b01 + a11 * b11 + a12 * b21 + a13 * b31;
        let c12 = a10 * b02 + a11 * b12 + a12 * b22 + a13 * b32;
        let c13 = a10 * b03 + a11 * b13 + a12 * b23 + a13 * b33;

        let c20 = a20 * b00 + a21 * b10 + a22 * b20 + a23 * b30;
        let c21 = a20 * b01 + a21 * b11 + a22 * b21 + a23 * b31;
        let c22 = a20 * b02 + a21 * b12 + a22 * b22 + a23 * b32;
        let c23 = a20 * b03 + a21 * b13 + a22 * b23 + a23 * b33;

        let c30 = a30 * b00 + a31 * b10 + a32 * b20 + a33 * b30;
        let c31 = a30 * b01 + a31 * b11 + a32 * b21 + a33 * b31;
        let c32 = a30 * b02 + a31 * b12 + a32 * b22 + a33 * b32;
        let c33 = a30 * b03 + a31 * b13 + a32 * b23 + a33 * b33;

        Self::new(
            c00, c10, c20, c30, c01, c11, c21, c31, c02, c12, c22, c32, c03, c13, c23, c33,
        )
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
            Vector4::dot(l, &r.col[3]),
        )
    }

    /// Adds two matrices element-wise.
    ///
    /// ```text
    /// C[i,j] = A[i,j] + B[i,j]
    /// ```
    pub fn add_matrix_matrix(l: &Self, r: &Self) -> Self {
        Matrix4 {
            col: [
                l.col[0] + r.col[0],
                l.col[1] + r.col[1],
                l.col[2] + r.col[2],
                l.col[3] + r.col[3],
            ],
        }
    }

    /// Subtracts two matrices element-wise.
    ///
    /// ```text
    /// C[i,j] = A[i,j] - B[i,j]
    /// ```
    pub fn sub_matrix_matrix(l: &Self, r: &Self) -> Self {
        Matrix4 {
            col: [
                l.col[0] - r.col[0],
                l.col[1] - r.col[1],
                l.col[2] - r.col[2],
                l.col[3] - r.col[3],
            ],
        }
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
        Matrix4::mul_vector_matrix(&Vector4::new(self.x, self.y, self.z, <T as One>::one()), &rhs).xyz()
    }
}

impl<T: Scalar> Mul<Vector3<T>> for Matrix4<T> {
    type Output = Vector3<T>;
    fn mul(self, rhs: Vector3<T>) -> Vector3<T> {
        Matrix4::mul_matrix_vector(&self, &Vector4::new(rhs.x, rhs.y, rhs.z, <T as One>::one())).xyz()
    }
}

pub trait Matrix4Extension<T: Scalar> {
    fn mat3(&self) -> Matrix3<T>;
}

impl<T: Scalar> Matrix4Extension<T> for Matrix4<T> {
    fn mat3(&self) -> Matrix3<T> {
        Matrix3::new(
            self.col[0].x,
            self.col[0].y,
            self.col[0].z,
            self.col[1].x,
            self.col[1].y,
            self.col[1].z,
            self.col[2].x,
            self.col[2].y,
            self.col[2].z,
        )
    }
}

#[cfg(test)]
mod tests {
    use super::*;
    use crate::vector::*;

    #[test]
    fn test_matrix2_identity() {
        let m = Matrix2::<f32>::identity();
        assert_eq!(m.col[0].x, 1.0);
        assert_eq!(m.col[0].y, 0.0);
        assert_eq!(m.col[1].x, 0.0);
        assert_eq!(m.col[1].y, 1.0);
    }

    #[test]
    fn test_matrix2_determinant() {
        let m = Matrix2::<f32>::new(1.0, 2.0, 3.0, 4.0);
        let det = m.determinant();
        assert_eq!(det, -2.0); // 1*4 - 3*2 = -2
        
        // Test singular matrix
        let m_singular = Matrix2::<f32>::new(1.0, 2.0, 2.0, 4.0);
        let det_singular = m_singular.determinant();
        assert_eq!(det_singular, 0.0);
    }

    #[test]
    fn test_matrix2_inverse() {
        let m = Matrix2::<f32>::new(1.0, 2.0, 3.0, 4.0);
        let m_inv = m.inverse();
        let product = Matrix2::mul_matrix_matrix(&m, &m_inv);
        
        // Check if product is identity
        assert!((product.col[0].x - 1.0).abs() < 0.001);
        assert!((product.col[0].y).abs() < 0.001);
        assert!((product.col[1].x).abs() < 0.001);
        assert!((product.col[1].y - 1.0).abs() < 0.001);
    }

    #[test]
    fn test_matrix2_transpose() {
        let m = Matrix2::<f32>::new(1.0, 2.0, 3.0, 4.0);
        let mt = m.transpose();
        assert_eq!(mt.col[0].x, 1.0);
        assert_eq!(mt.col[0].y, 3.0);
        assert_eq!(mt.col[1].x, 2.0);
        assert_eq!(mt.col[1].y, 4.0);
        
        // Transpose of transpose should be original
        let mtt = mt.transpose();
        assert_eq!(mtt.col[0].x, m.col[0].x);
        assert_eq!(mtt.col[0].y, m.col[0].y);
        assert_eq!(mtt.col[1].x, m.col[1].x);
        assert_eq!(mtt.col[1].y, m.col[1].y);
    }

    #[test]
    fn test_matrix3_identity() {
        let m = Matrix3::<f32>::identity();
        assert_eq!(m.col[0].x, 1.0);
        assert_eq!(m.col[1].y, 1.0);
        assert_eq!(m.col[2].z, 1.0);
        assert_eq!(m.col[0].y, 0.0);
        assert_eq!(m.col[0].z, 0.0);
    }

    #[test]
    fn test_matrix3_determinant() {
        let m = Matrix3::<f32>::new(
            1.0, 0.0, 0.0,
            0.0, 1.0, 0.0,
            0.0, 0.0, 1.0
        );
        assert_eq!(m.determinant(), 1.0);
        
        let m2 = Matrix3::<f32>::new(
            2.0, 3.0, 1.0,
            1.0, 0.0, 2.0,
            1.0, 2.0, 1.0
        );
        let det = m2.determinant();
        assert!((det - -3.0).abs() < 0.001);
    }

    #[test]
    fn test_matrix3_inverse() {
        let m = Matrix3::<f32>::new(
            2.0, 3.0, 1.0,
            1.0, 0.0, 2.0,
            1.0, 2.0, 1.0
        );
        let m_inv = m.inverse();
        let product = Matrix3::mul_matrix_matrix(&m, &m_inv);
        
        // Check if product is close to identity
        for i in 0..3 {
            for j in 0..3 {
                let val = match (i, j) {
                    (0, 0) => product.col[0].x,
                    (1, 0) => product.col[0].y,
                    (2, 0) => product.col[0].z,
                    (0, 1) => product.col[1].x,
                    (1, 1) => product.col[1].y,
                    (2, 1) => product.col[1].z,
                    (0, 2) => product.col[2].x,
                    (1, 2) => product.col[2].y,
                    (2, 2) => product.col[2].z,
                    _ => 0.0,
                };
                let expected = if i == j { 1.0 } else { 0.0 };
                assert!((val - expected).abs() < 0.001);
            }
        }
    }

    #[test]
    fn test_matrix3_axis_angle_zero_axis() {
        let axis = Vector3::<f32>::new(0.0, 0.0, 0.0);
        assert!(Matrix3::of_axis_angle(&axis, 1.0, EPS_F32).is_none());
    }

    #[test]
    fn test_matrix4_identity() {
        let m = Matrix4::<f32>::identity();
        for i in 0..4 {
            for j in 0..4 {
                let val = match j {
                    0 => match i {
                        0 => m.col[0].x,
                        1 => m.col[0].y,
                        2 => m.col[0].z,
                        3 => m.col[0].w,
                        _ => 0.0,
                    },
                    1 => match i {
                        0 => m.col[1].x,
                        1 => m.col[1].y,
                        2 => m.col[1].z,
                        3 => m.col[1].w,
                        _ => 0.0,
                    },
                    2 => match i {
                        0 => m.col[2].x,
                        1 => m.col[2].y,
                        2 => m.col[2].z,
                        3 => m.col[2].w,
                        _ => 0.0,
                    },
                    3 => match i {
                        0 => m.col[3].x,
                        1 => m.col[3].y,
                        2 => m.col[3].z,
                        3 => m.col[3].w,
                        _ => 0.0,
                    },
                    _ => 0.0,
                };
                let expected = if i == j { 1.0 } else { 0.0 };
                assert_eq!(val, expected);
            }
        }
    }

    #[test]
    fn test_matrix_vector_multiplication() {
        // Test Matrix2 * Vector2
        let m2 = Matrix2::<f32>::new(1.0, 2.0, 3.0, 4.0);
        let v2 = Vector2::<f32>::new(5.0, 6.0);
        let result2 = m2 * v2;
        assert_eq!(result2.x, 23.0); // 1*5 + 3*6 = 23
        assert_eq!(result2.y, 34.0); // 2*5 + 4*6 = 34
        
        // Test Matrix3 * Vector3
        let m3 = Matrix3::<f32>::identity();
        let v3 = Vector3::<f32>::new(1.0, 2.0, 3.0);
        let result3 = m3 * v3;
        assert_eq!(result3.x, 1.0);
        assert_eq!(result3.y, 2.0);
        assert_eq!(result3.z, 3.0);
        
        // Test Matrix4 * Vector4
        let m4 = Matrix4::<f32>::identity();
        let v4 = Vector4::<f32>::new(1.0, 2.0, 3.0, 4.0);
        let result4 = m4 * v4;
        assert_eq!(result4.x, 1.0);
        assert_eq!(result4.y, 2.0);
        assert_eq!(result4.z, 3.0);
        assert_eq!(result4.w, 4.0);
    }

    #[test]
    fn test_matrix_multiplication() {
        // Test associativity: (A * B) * C == A * (B * C)
        let a = Matrix2::<f32>::new(1.0, 2.0, 3.0, 4.0);
        let b = Matrix2::<f32>::new(5.0, 6.0, 7.0, 8.0);
        let c = Matrix2::<f32>::new(9.0, 10.0, 11.0, 12.0);
        
        let left = (a * b) * c;
        let right = a * (b * c);
        
        assert!((left.col[0].x - right.col[0].x).abs() < 0.001);
        assert!((left.col[0].y - right.col[0].y).abs() < 0.001);
        assert!((left.col[1].x - right.col[1].x).abs() < 0.001);
        assert!((left.col[1].y - right.col[1].y).abs() < 0.001);
    }

    #[test]
    fn test_matrix_addition_subtraction() {
        let m1 = Matrix2::<f32>::new(1.0, 2.0, 3.0, 4.0);
        let m2 = Matrix2::<f32>::new(5.0, 6.0, 7.0, 8.0);
        
        let sum = m1 + m2;
        assert_eq!(sum.col[0].x, 6.0);
        assert_eq!(sum.col[0].y, 8.0);
        assert_eq!(sum.col[1].x, 10.0);
        assert_eq!(sum.col[1].y, 12.0);
        
        let diff = m2 - m1;
        assert_eq!(diff.col[0].x, 4.0);
        assert_eq!(diff.col[0].y, 4.0);
        assert_eq!(diff.col[1].x, 4.0);
        assert_eq!(diff.col[1].y, 4.0);
    }

    #[test]
    fn test_matrix4_inverse() {
        // Test with a known invertible matrix
        let m = Matrix4::<f32>::new(
            2.0, 0.0, 0.0, 0.0,
            0.0, 1.0, 0.0, 0.0,
            0.0, 0.0, 0.5, 0.0,
            1.0, 2.0, 3.0, 1.0
        );
        
        let m_inv = m.inverse();
        let product = m * m_inv;
        
        // Check if product is close to identity
        for i in 0..4 {
            for j in 0..4 {
                let expected = if i == j { 1.0 } else { 0.0 };
                let val = match j {
                    0 => match i {
                        0 => product.col[0].x,
                        1 => product.col[0].y,
                        2 => product.col[0].z,
                        3 => product.col[0].w,
                        _ => 0.0,
                    },
                    1 => match i {
                        0 => product.col[1].x,
                        1 => product.col[1].y,
                        2 => product.col[1].z,
                        3 => product.col[1].w,
                        _ => 0.0,
                    },
                    2 => match i {
                        0 => product.col[2].x,
                        1 => product.col[2].y,
                        2 => product.col[2].z,
                        3 => product.col[2].w,
                        _ => 0.0,
                    },
                    3 => match i {
                        0 => product.col[3].x,
                        1 => product.col[3].y,
                        2 => product.col[3].z,
                        3 => product.col[3].w,
                        _ => 0.0,
                    },
                    _ => 0.0,
                };
                assert!((val - expected).abs() < 0.001, 
                    "Matrix inverse failed at [{}, {}]: expected {}, got {}", i, j, expected, val);
            }
        }
    }
}
