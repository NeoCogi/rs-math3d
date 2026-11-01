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
//! Integer matrices are supported for storage, addition, subtraction, and
//! multiplication. Operations that require fractional results, such as inversion
//! and axis-angle construction, are available through `RealScalar` for floating
//! and suitable fixed formats.
//!
//! # Examples
//!
//! ```
//! # fn main() {
//! # #[cfg(any(feature = "std", feature = "libm", feature = "system-libm"))]
//! # {
//! use rs_math3d::matrix::Matrix4;
//! use rs_math3d::vector::Vector4;
//!
//! let m = Matrix4::<f32>::identity();
//! let v = Vector4::new(1.0, 2.0, 3.0, 1.0);
//! let result = m * v; // Transform vector
//! # }
//! # }
//! ```

use crate::scalar::*;
use crate::vector::*;
use crate::MathError;
use core::ops::*;
use num_traits::{One, Zero};

/// Matrix inversion algorithm. The reciprocal/adjugate path is the default
/// for low arithmetic cost; row elimination offers different range/conditioning
/// tradeoffs and is selected explicitly per operation.
#[derive(Clone, Copy, Debug, Default, Eq, PartialEq)]
pub enum InverseMode {
    /// Compute an adjugate and apply one reusable determinant reciprocal.
    #[default]
    Reciprocal,
    /// Use checked partial-pivot Gauss-Jordan row elimination.
    Rows,
}

/// Inverts row-major stack arrays with partial pivoting. The caller owns both
/// arrays; no heap storage, global scratch memory or hidden float conversion is
/// used. Row updates accumulate differences of products before quantization.
fn invert_rows<T: RealScalar, const N: usize>(
    values: &mut [[T; N]; N],
    inverse: &mut [[T; N]; N],
) -> Result<(), MathError> {
    let zero = <T as Zero>::zero();
    let one = <T as One>::one();
    for row in values.iter() {
        for &value in row {
            if !value.is_finite() {
                return Err(MathError::NonFinite);
            }
        }
    }
    for column in 0..N {
        let mut pivot = column;
        for row in column + 1..N {
            if values[row][column].try_abs()? > values[pivot][column].try_abs()? {
                pivot = row;
            }
        }
        let divisor = values[pivot][column];
        if divisor == zero {
            return Err(MathError::Degenerate);
        }
        values.swap(column, pivot);
        inverse.swap(column, pivot);
        for entry in 0..N {
            values[column][entry] = values[column][entry].try_div(divisor)?;
            inverse[column][entry] = inverse[column][entry].try_div(divisor)?;
        }
        values[column][column] = one;
        for row in 0..N {
            if row == column {
                continue;
            }
            let factor = values[row][column];
            for entry in 0..N {
                values[row][entry] = T::try_sum_products(&[
                    (values[row][entry], one, false),
                    (factor, values[column][entry], true),
                ])?;
                inverse[row][entry] = T::try_sum_products(&[
                    (inverse[row][entry], one, false),
                    (factor, inverse[column][entry], true),
                ])?;
            }
            values[row][column] = zero;
        }
    }
    Ok(())
}

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
        Self::new(
            <T as One>::one(),
            <T as Zero>::zero(),
            <T as Zero>::zero(),
            <T as One>::one(),
        )
    }

    /// Attempts to cast each element into another numeric type.
    pub fn try_cast<U>(&self) -> Option<Matrix2<U>>
    where
        T: Copy,
        U: Scalar + crate::TryCastFrom<T>,
    {
        Some(Matrix2::new(
            U::try_cast_from(self.col[0].x).ok()?,
            U::try_cast_from(self.col[0].y).ok()?,
            U::try_cast_from(self.col[1].x).ok()?,
            U::try_cast_from(self.col[1].y).ok()?,
        ))
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

    /// Inverts using the default reciprocal/adjugate path.
    pub fn try_inverse(&self) -> Result<Self, MathError>
    where
        T: RealScalar,
    {
        self.try_inverse_with(InverseMode::default())
    }
    /// Selects the inversion algorithm explicitly; both modes check every
    /// arithmetic boundary and can differ in fixed-point rounding and range.
    pub fn try_inverse_with(&self, mode: InverseMode) -> Result<Self, MathError>
    where
        T: RealScalar,
    {
        match mode {
            InverseMode::Reciprocal => self.try_inverse_reciprocal(),
            InverseMode::Rows => self.try_inverse_rows(),
        }
    }
    /// Selects an inversion mode, panicking on invalid/unrepresentable input.
    pub fn inverse_with(&self, mode: InverseMode) -> Self
    where
        T: RealScalar,
    {
        self.try_inverse_with(mode).expect("matrix inverse failed")
    }
    /// Computes the adjugate and one guarded determinant reciprocal. Fixed
    /// reciprocals retain internal precision even if scalar 1/det rounds to zero.
    /// Cofactor/determinant intermediates must fit the scalar format; use Rows
    /// when its different intermediate range is preferable.
    pub fn try_inverse_reciprocal(&self) -> Result<Self, MathError>
    where
        T: RealScalar,
    {
        let m00 = self.col[0].x;
        let m10 = self.col[0].y;
        let m01 = self.col[1].x;
        let m11 = self.col[1].y;
        let denominator = ((m00).try_mul(m11)?).try_sub((m01).try_mul(m10)?)?;
        if denominator == <T as Zero>::zero() {
            return Err(MathError::Degenerate);
        }
        let reciprocal = denominator.try_reciprocal()?;
        let r00 = (m11).try_mul_reciprocal(reciprocal)?;
        let r01 = ((m01).try_neg()?).try_mul_reciprocal(reciprocal)?;
        let r10 = ((m10).try_neg()?).try_mul_reciprocal(reciprocal)?;
        let r11 = (m00).try_mul_reciprocal(reciprocal)?;
        Ok(Self::new(r00, r10, r01, r11))
    }

    /// Inverts with partial-pivot Gauss-Jordan elimination and checked
    /// arithmetic. Singular pivots, non-finite inputs and unrepresentable
    /// intermediate/output values are reported instead of producing NaNs.
    /// Fixed-point elimination rounds at each represented row operation.
    pub fn try_inverse_rows(&self) -> Result<Self, MathError>
    where
        T: RealScalar,
    {
        let rows = self.transpose();
        let mut values = [[<T as Zero>::zero(); 2]; 2];
        let mut inverse = [[<T as Zero>::zero(); 2]; 2];
        for row in 0..2 {
            inverse[row][row] = <T as One>::one();
            let v = rows.col[row];
            values[row][0] = v.x;
            values[row][1] = v.y;
        }
        invert_rows(&mut values, &mut inverse)?;
        let mut result = Self::identity();
        result.col[0].x = inverse[0][0];
        result.col[0].y = inverse[1][0];
        result.col[1].x = inverse[0][1];
        result.col[1].y = inverse[1][1];
        Ok(result)
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
    /// Panics for singular, non-finite or unrepresentable matrices. Use `try_inverse`
    /// to inspect the numerical error.
    pub fn inverse(&self) -> Self
    where
        T: RealScalar,
    {
        self.try_inverse()
            .expect("matrix inverse is not representable")
    }

    /// Multiplies two 2x2 matrices.
    ///
    /// Matrix multiplication follows the rule:
    /// ```text
    /// C[i,j] = Σₖ A[i,k] * B[k,j]
    /// ```
    pub fn mul_matrix_matrix(l: &Self, r: &Self) -> Self {
        let rows = l.transpose();
        let mut result = Self::identity();
        for column in 0..2 {
            result.col[column] = Self::mul_vector_matrix(&r.col[column], &rows);
        }
        result
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
    /// Creates a new 3x3 matrix from column-major elements.
    #[allow(clippy::too_many_arguments)]
    pub fn new(m0: T, m1: T, m2: T, m3: T, m4: T, m5: T, m6: T, m7: T, m8: T) -> Self {
        Matrix3 {
            col: [
                Vector3::new(m0, m1, m2),
                Vector3::new(m3, m4, m5),
                Vector3::new(m6, m7, m8),
            ],
        }
    }

    /// Returns the 3x3 identity matrix.
    ///
    /// ```text
    /// [1 0 0]
    /// [0 1 0]
    /// [0 0 1]
    /// ```
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

    /// Attempts to cast each element into another numeric type.
    pub fn try_cast<U>(&self) -> Option<Matrix3<U>>
    where
        T: Copy,
        U: Scalar + crate::TryCastFrom<T>,
    {
        Some(Matrix3::new(
            U::try_cast_from(self.col[0].x).ok()?,
            U::try_cast_from(self.col[0].y).ok()?,
            U::try_cast_from(self.col[0].z).ok()?,
            U::try_cast_from(self.col[1].x).ok()?,
            U::try_cast_from(self.col[1].y).ok()?,
            U::try_cast_from(self.col[1].z).ok()?,
            U::try_cast_from(self.col[2].x).ok()?,
            U::try_cast_from(self.col[2].y).ok()?,
            U::try_cast_from(self.col[2].z).ok()?,
        ))
    }

    /// Computes the determinant of the matrix.
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

    /// Inverts using the default reciprocal/adjugate path.
    pub fn try_inverse(&self) -> Result<Self, MathError>
    where
        T: RealScalar,
    {
        self.try_inverse_with(InverseMode::default())
    }
    /// Selects the inversion algorithm explicitly; both modes check every
    /// arithmetic boundary and can differ in fixed-point rounding and range.
    pub fn try_inverse_with(&self, mode: InverseMode) -> Result<Self, MathError>
    where
        T: RealScalar,
    {
        match mode {
            InverseMode::Reciprocal => self.try_inverse_reciprocal(),
            InverseMode::Rows => self.try_inverse_rows(),
        }
    }
    /// Selects an inversion mode, panicking on invalid/unrepresentable input.
    pub fn inverse_with(&self, mode: InverseMode) -> Self
    where
        T: RealScalar,
    {
        self.try_inverse_with(mode).expect("matrix inverse failed")
    }
    /// Computes the adjugate and one guarded determinant reciprocal. Fixed
    /// reciprocals retain internal precision even if scalar 1/det rounds to zero.
    /// Cofactor/determinant intermediates must fit the scalar format; use Rows
    /// when its different intermediate range is preferable.
    pub fn try_inverse_reciprocal(&self) -> Result<Self, MathError>
    where
        T: RealScalar,
    {
        let m00 = self.col[0].x;
        let m10 = self.col[0].y;
        let m20 = self.col[0].z;
        let m01 = self.col[1].x;
        let m11 = self.col[1].y;
        let m21 = self.col[1].z;
        let m02 = self.col[2].x;
        let m12 = self.col[2].y;
        let m22 = self.col[2].z;
        let denominator = (((((((m00).try_mul(m11)?).try_mul(m22)?)
            .try_add(((m01).try_mul(m12)?).try_mul(m20)?)?)
        .try_add(((m02).try_mul(m10)?).try_mul(m21)?)?)
        .try_sub(((m00).try_mul(m12)?).try_mul(m21)?)?)
        .try_sub(((m01).try_mul(m10)?).try_mul(m22)?)?)
        .try_sub(((m02).try_mul(m11)?).try_mul(m20)?)?;
        if denominator == <T as Zero>::zero() {
            return Err(MathError::Degenerate);
        }
        let reciprocal = denominator.try_reciprocal()?;
        let r00 =
            (((m11).try_mul(m22)?).try_sub((m12).try_mul(m21)?)?).try_mul_reciprocal(reciprocal)?;
        let r01 =
            (((m02).try_mul(m21)?).try_sub((m01).try_mul(m22)?)?).try_mul_reciprocal(reciprocal)?;
        let r02 =
            (((m01).try_mul(m12)?).try_sub((m02).try_mul(m11)?)?).try_mul_reciprocal(reciprocal)?;
        let r10 =
            (((m12).try_mul(m20)?).try_sub((m10).try_mul(m22)?)?).try_mul_reciprocal(reciprocal)?;
        let r11 =
            (((m00).try_mul(m22)?).try_sub((m02).try_mul(m20)?)?).try_mul_reciprocal(reciprocal)?;
        let r12 =
            (((m02).try_mul(m10)?).try_sub((m00).try_mul(m12)?)?).try_mul_reciprocal(reciprocal)?;
        let r20 =
            (((m10).try_mul(m21)?).try_sub((m11).try_mul(m20)?)?).try_mul_reciprocal(reciprocal)?;
        let r21 =
            (((m01).try_mul(m20)?).try_sub((m00).try_mul(m21)?)?).try_mul_reciprocal(reciprocal)?;
        let r22 =
            (((m00).try_mul(m11)?).try_sub((m01).try_mul(m10)?)?).try_mul_reciprocal(reciprocal)?;
        Ok(Self::new(r00, r10, r20, r01, r11, r21, r02, r12, r22))
    }

    /// Inverts with partial-pivot Gauss-Jordan elimination and checked
    /// arithmetic. Singular pivots, non-finite inputs and unrepresentable
    /// intermediate/output values are reported instead of producing NaNs.
    /// Fixed-point elimination rounds at each represented row operation.
    pub fn try_inverse_rows(&self) -> Result<Self, MathError>
    where
        T: RealScalar,
    {
        let rows = self.transpose();
        let mut values = [[<T as Zero>::zero(); 3]; 3];
        let mut inverse = [[<T as Zero>::zero(); 3]; 3];
        for row in 0..3 {
            inverse[row][row] = <T as One>::one();
            let v = rows.col[row];
            values[row][0] = v.x;
            values[row][1] = v.y;
            values[row][2] = v.z;
        }
        invert_rows(&mut values, &mut inverse)?;
        let mut result = Self::identity();
        result.col[0].x = inverse[0][0];
        result.col[0].y = inverse[1][0];
        result.col[0].z = inverse[2][0];
        result.col[1].x = inverse[0][1];
        result.col[1].y = inverse[1][1];
        result.col[1].z = inverse[2][1];
        result.col[2].x = inverse[0][2];
        result.col[2].y = inverse[1][2];
        result.col[2].z = inverse[2][2];
        Ok(result)
    }

    /// Computes the inverse of the matrix.
    ///
    /// Uses the reciprocal/adjugate path by default; `inverse_with` selects rows:
    /// ```text
    /// M⁻¹ = (1/det(M)) * adj(M)
    /// ```
    ///
    /// # Note
    /// Panics for singular, non-finite or unrepresentable matrices. Use `try_inverse`
    /// to inspect the numerical error.
    pub fn inverse(&self) -> Self
    where
        T: RealScalar,
    {
        self.try_inverse()
            .expect("matrix inverse is not representable")
    }

    /// Multiplies two 3x3 matrices.
    ///
    /// Matrix multiplication follows the rule:
    /// ```text
    /// C[i,j] = Σₖ A[i,k] * B[k,j]
    /// ```
    pub fn mul_matrix_matrix(l: &Self, r: &Self) -> Self {
        let rows = l.transpose();
        let mut result = Self::identity();
        for column in 0..3 {
            result.col[column] = Self::mul_vector_matrix(&r.col[column], &rows);
        }
        result
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

impl<T: RealScalar> Matrix3<T> {
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
        let n = axis.checked_normalize(epsilon).ok()?;
        let c = angle.try_cos().ok()?;
        let s = angle.try_sin().ok()?;
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
    /// Creates a new 4x4 matrix from column-major elements.
    #[allow(clippy::too_many_arguments)]
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

    /// Returns the 4x4 identity matrix.
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

    /// Attempts to cast each element into another numeric type.
    pub fn try_cast<U>(&self) -> Option<Matrix4<U>>
    where
        T: Copy,
        U: Scalar + crate::TryCastFrom<T>,
    {
        Some(Matrix4::new(
            U::try_cast_from(self.col[0].x).ok()?,
            U::try_cast_from(self.col[0].y).ok()?,
            U::try_cast_from(self.col[0].z).ok()?,
            U::try_cast_from(self.col[0].w).ok()?,
            U::try_cast_from(self.col[1].x).ok()?,
            U::try_cast_from(self.col[1].y).ok()?,
            U::try_cast_from(self.col[1].z).ok()?,
            U::try_cast_from(self.col[1].w).ok()?,
            U::try_cast_from(self.col[2].x).ok()?,
            U::try_cast_from(self.col[2].y).ok()?,
            U::try_cast_from(self.col[2].z).ok()?,
            U::try_cast_from(self.col[2].w).ok()?,
            U::try_cast_from(self.col[3].x).ok()?,
            U::try_cast_from(self.col[3].y).ok()?,
            U::try_cast_from(self.col[3].z).ok()?,
            U::try_cast_from(self.col[3].w).ok()?,
        ))
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

    /// Returns true if the matrix is affine (last row equals [0, 0, 0, 1]).
    pub fn is_affine(&self, epsilon: T) -> bool {
        let zero = <T as Zero>::zero();
        if !(epsilon >= zero) {
            return false;
        }
        let absolute = |value: T| {
            if value < zero {
                value.try_neg().ok()
            } else {
                Some(value)
            }
        };
        let inside = |value: T| absolute(value).map_or(false, |v| v <= epsilon);
        inside(self.col[0].w)
            && inside(self.col[1].w)
            && inside(self.col[2].w)
            && self.col[3]
                .w
                .try_sub(<T as One>::one())
                .map_or(false, inside)
    }

    /// Inverts using the default reciprocal/adjugate path.
    pub fn try_inverse(&self) -> Result<Self, MathError>
    where
        T: RealScalar,
    {
        self.try_inverse_with(InverseMode::default())
    }
    /// Selects the inversion algorithm explicitly; both modes check every
    /// arithmetic boundary and can differ in fixed-point rounding and range.
    pub fn try_inverse_with(&self, mode: InverseMode) -> Result<Self, MathError>
    where
        T: RealScalar,
    {
        match mode {
            InverseMode::Reciprocal => self.try_inverse_reciprocal(),
            InverseMode::Rows => self.try_inverse_rows(),
        }
    }
    /// Selects an inversion mode, panicking on invalid/unrepresentable input.
    pub fn inverse_with(&self, mode: InverseMode) -> Self
    where
        T: RealScalar,
    {
        self.try_inverse_with(mode).expect("matrix inverse failed")
    }
    /// Computes the adjugate and one guarded determinant reciprocal. Fixed
    /// reciprocals retain internal precision even if scalar 1/det rounds to zero.
    /// Cofactor/determinant intermediates must fit the scalar format; use Rows
    /// when its different intermediate range is preferable.
    pub fn try_inverse_reciprocal(&self) -> Result<Self, MathError>
    where
        T: RealScalar,
    {
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
        let denom = ((((((((((((((((((((((((((m03).try_mul(m12)?)
            .try_mul(m21)?)
        .try_mul(m30)?)
        .try_sub((((m02).try_mul(m13)?).try_mul(m21)?).try_mul(m30)?)?)
        .try_sub((((m03).try_mul(m11)?).try_mul(m22)?).try_mul(m30)?)?)
        .try_add((((m01).try_mul(m13)?).try_mul(m22)?).try_mul(m30)?)?)
        .try_add((((m02).try_mul(m11)?).try_mul(m23)?).try_mul(m30)?)?)
        .try_sub((((m01).try_mul(m12)?).try_mul(m23)?).try_mul(m30)?)?)
        .try_sub((((m03).try_mul(m12)?).try_mul(m20)?).try_mul(m31)?)?)
        .try_add((((m02).try_mul(m13)?).try_mul(m20)?).try_mul(m31)?)?)
        .try_add((((m03).try_mul(m10)?).try_mul(m22)?).try_mul(m31)?)?)
        .try_sub((((m00).try_mul(m13)?).try_mul(m22)?).try_mul(m31)?)?)
        .try_sub((((m02).try_mul(m10)?).try_mul(m23)?).try_mul(m31)?)?)
        .try_add((((m00).try_mul(m12)?).try_mul(m23)?).try_mul(m31)?)?)
        .try_add((((m03).try_mul(m11)?).try_mul(m20)?).try_mul(m32)?)?)
        .try_sub((((m01).try_mul(m13)?).try_mul(m20)?).try_mul(m32)?)?)
        .try_sub((((m03).try_mul(m10)?).try_mul(m21)?).try_mul(m32)?)?)
        .try_add((((m00).try_mul(m13)?).try_mul(m21)?).try_mul(m32)?)?)
        .try_add((((m01).try_mul(m10)?).try_mul(m23)?).try_mul(m32)?)?)
        .try_sub((((m00).try_mul(m11)?).try_mul(m23)?).try_mul(m32)?)?)
        .try_sub((((m02).try_mul(m11)?).try_mul(m20)?).try_mul(m33)?)?)
        .try_add((((m01).try_mul(m12)?).try_mul(m20)?).try_mul(m33)?)?)
        .try_add((((m02).try_mul(m10)?).try_mul(m21)?).try_mul(m33)?)?)
        .try_sub((((m00).try_mul(m12)?).try_mul(m21)?).try_mul(m33)?)?)
        .try_sub((((m01).try_mul(m10)?).try_mul(m22)?).try_mul(m33)?)?)
        .try_add((((m00).try_mul(m11)?).try_mul(m22)?).try_mul(m33)?)?;
        let denominator = denom;
        if denominator == <T as Zero>::zero() {
            return Err(MathError::Degenerate);
        }
        let reciprocal = denominator.try_reciprocal()?;
        let r00 = ((((((((m12).try_mul(m23)?).try_mul(m31)?)
            .try_sub(((m13).try_mul(m22)?).try_mul(m31)?)?)
        .try_add(((m13).try_mul(m21)?).try_mul(m32)?)?)
        .try_sub(((m11).try_mul(m23)?).try_mul(m32)?)?)
        .try_sub(((m12).try_mul(m21)?).try_mul(m33)?)?)
        .try_add(((m11).try_mul(m22)?).try_mul(m33)?)?)
        .try_mul_reciprocal(reciprocal)?;
        let r01 = ((((((((m03).try_mul(m22)?).try_mul(m31)?)
            .try_sub(((m02).try_mul(m23)?).try_mul(m31)?)?)
        .try_sub(((m03).try_mul(m21)?).try_mul(m32)?)?)
        .try_add(((m01).try_mul(m23)?).try_mul(m32)?)?)
        .try_add(((m02).try_mul(m21)?).try_mul(m33)?)?)
        .try_sub(((m01).try_mul(m22)?).try_mul(m33)?)?)
        .try_mul_reciprocal(reciprocal)?;
        let r02 = ((((((((m02).try_mul(m13)?).try_mul(m31)?)
            .try_sub(((m03).try_mul(m12)?).try_mul(m31)?)?)
        .try_add(((m03).try_mul(m11)?).try_mul(m32)?)?)
        .try_sub(((m01).try_mul(m13)?).try_mul(m32)?)?)
        .try_sub(((m02).try_mul(m11)?).try_mul(m33)?)?)
        .try_add(((m01).try_mul(m12)?).try_mul(m33)?)?)
        .try_mul_reciprocal(reciprocal)?;
        let r03 = ((((((((m03).try_mul(m12)?).try_mul(m21)?)
            .try_sub(((m02).try_mul(m13)?).try_mul(m21)?)?)
        .try_sub(((m03).try_mul(m11)?).try_mul(m22)?)?)
        .try_add(((m01).try_mul(m13)?).try_mul(m22)?)?)
        .try_add(((m02).try_mul(m11)?).try_mul(m23)?)?)
        .try_sub(((m01).try_mul(m12)?).try_mul(m23)?)?)
        .try_mul_reciprocal(reciprocal)?;
        let r10 = ((((((((m13).try_mul(m22)?).try_mul(m30)?)
            .try_sub(((m12).try_mul(m23)?).try_mul(m30)?)?)
        .try_sub(((m13).try_mul(m20)?).try_mul(m32)?)?)
        .try_add(((m10).try_mul(m23)?).try_mul(m32)?)?)
        .try_add(((m12).try_mul(m20)?).try_mul(m33)?)?)
        .try_sub(((m10).try_mul(m22)?).try_mul(m33)?)?)
        .try_mul_reciprocal(reciprocal)?;
        let r11 = ((((((((m02).try_mul(m23)?).try_mul(m30)?)
            .try_sub(((m03).try_mul(m22)?).try_mul(m30)?)?)
        .try_add(((m03).try_mul(m20)?).try_mul(m32)?)?)
        .try_sub(((m00).try_mul(m23)?).try_mul(m32)?)?)
        .try_sub(((m02).try_mul(m20)?).try_mul(m33)?)?)
        .try_add(((m00).try_mul(m22)?).try_mul(m33)?)?)
        .try_mul_reciprocal(reciprocal)?;
        let r12 = ((((((((m03).try_mul(m12)?).try_mul(m30)?)
            .try_sub(((m02).try_mul(m13)?).try_mul(m30)?)?)
        .try_sub(((m03).try_mul(m10)?).try_mul(m32)?)?)
        .try_add(((m00).try_mul(m13)?).try_mul(m32)?)?)
        .try_add(((m02).try_mul(m10)?).try_mul(m33)?)?)
        .try_sub(((m00).try_mul(m12)?).try_mul(m33)?)?)
        .try_mul_reciprocal(reciprocal)?;
        let r13 = ((((((((m02).try_mul(m13)?).try_mul(m20)?)
            .try_sub(((m03).try_mul(m12)?).try_mul(m20)?)?)
        .try_add(((m03).try_mul(m10)?).try_mul(m22)?)?)
        .try_sub(((m00).try_mul(m13)?).try_mul(m22)?)?)
        .try_sub(((m02).try_mul(m10)?).try_mul(m23)?)?)
        .try_add(((m00).try_mul(m12)?).try_mul(m23)?)?)
        .try_mul_reciprocal(reciprocal)?;
        let r20 = ((((((((m11).try_mul(m23)?).try_mul(m30)?)
            .try_sub(((m13).try_mul(m21)?).try_mul(m30)?)?)
        .try_add(((m13).try_mul(m20)?).try_mul(m31)?)?)
        .try_sub(((m10).try_mul(m23)?).try_mul(m31)?)?)
        .try_sub(((m11).try_mul(m20)?).try_mul(m33)?)?)
        .try_add(((m10).try_mul(m21)?).try_mul(m33)?)?)
        .try_mul_reciprocal(reciprocal)?;
        let r21 = ((((((((m03).try_mul(m21)?).try_mul(m30)?)
            .try_sub(((m01).try_mul(m23)?).try_mul(m30)?)?)
        .try_sub(((m03).try_mul(m20)?).try_mul(m31)?)?)
        .try_add(((m00).try_mul(m23)?).try_mul(m31)?)?)
        .try_add(((m01).try_mul(m20)?).try_mul(m33)?)?)
        .try_sub(((m00).try_mul(m21)?).try_mul(m33)?)?)
        .try_mul_reciprocal(reciprocal)?;
        let r22 = ((((((((m01).try_mul(m13)?).try_mul(m30)?)
            .try_sub(((m03).try_mul(m11)?).try_mul(m30)?)?)
        .try_add(((m03).try_mul(m10)?).try_mul(m31)?)?)
        .try_sub(((m00).try_mul(m13)?).try_mul(m31)?)?)
        .try_sub(((m01).try_mul(m10)?).try_mul(m33)?)?)
        .try_add(((m00).try_mul(m11)?).try_mul(m33)?)?)
        .try_mul_reciprocal(reciprocal)?;
        let r23 = ((((((((m03).try_mul(m11)?).try_mul(m20)?)
            .try_sub(((m01).try_mul(m13)?).try_mul(m20)?)?)
        .try_sub(((m03).try_mul(m10)?).try_mul(m21)?)?)
        .try_add(((m00).try_mul(m13)?).try_mul(m21)?)?)
        .try_add(((m01).try_mul(m10)?).try_mul(m23)?)?)
        .try_sub(((m00).try_mul(m11)?).try_mul(m23)?)?)
        .try_mul_reciprocal(reciprocal)?;
        let r30 = ((((((((m12).try_mul(m21)?).try_mul(m30)?)
            .try_sub(((m11).try_mul(m22)?).try_mul(m30)?)?)
        .try_sub(((m12).try_mul(m20)?).try_mul(m31)?)?)
        .try_add(((m10).try_mul(m22)?).try_mul(m31)?)?)
        .try_add(((m11).try_mul(m20)?).try_mul(m32)?)?)
        .try_sub(((m10).try_mul(m21)?).try_mul(m32)?)?)
        .try_mul_reciprocal(reciprocal)?;
        let r31 = ((((((((m01).try_mul(m22)?).try_mul(m30)?)
            .try_sub(((m02).try_mul(m21)?).try_mul(m30)?)?)
        .try_add(((m02).try_mul(m20)?).try_mul(m31)?)?)
        .try_sub(((m00).try_mul(m22)?).try_mul(m31)?)?)
        .try_sub(((m01).try_mul(m20)?).try_mul(m32)?)?)
        .try_add(((m00).try_mul(m21)?).try_mul(m32)?)?)
        .try_mul_reciprocal(reciprocal)?;
        let r32 = ((((((((m02).try_mul(m11)?).try_mul(m30)?)
            .try_sub(((m01).try_mul(m12)?).try_mul(m30)?)?)
        .try_sub(((m02).try_mul(m10)?).try_mul(m31)?)?)
        .try_add(((m00).try_mul(m12)?).try_mul(m31)?)?)
        .try_add(((m01).try_mul(m10)?).try_mul(m32)?)?)
        .try_sub(((m00).try_mul(m11)?).try_mul(m32)?)?)
        .try_mul_reciprocal(reciprocal)?;
        let r33 = ((((((((m01).try_mul(m12)?).try_mul(m20)?)
            .try_sub(((m02).try_mul(m11)?).try_mul(m20)?)?)
        .try_add(((m02).try_mul(m10)?).try_mul(m21)?)?)
        .try_sub(((m00).try_mul(m12)?).try_mul(m21)?)?)
        .try_sub(((m01).try_mul(m10)?).try_mul(m22)?)?)
        .try_add(((m00).try_mul(m11)?).try_mul(m22)?)?)
        .try_mul_reciprocal(reciprocal)?;
        Ok(Self::new(
            r00, r10, r20, r30, r01, r11, r21, r31, r02, r12, r22, r32, r03, r13, r23, r33,
        ))
    }

    /// Inverts with partial-pivot Gauss-Jordan elimination and checked
    /// arithmetic. Singular pivots, non-finite inputs and unrepresentable
    /// intermediate/output values are reported instead of producing NaNs.
    /// Fixed-point elimination rounds at each represented row operation.
    pub fn try_inverse_rows(&self) -> Result<Self, MathError>
    where
        T: RealScalar,
    {
        let rows = self.transpose();
        let mut values = [[<T as Zero>::zero(); 4]; 4];
        let mut inverse = [[<T as Zero>::zero(); 4]; 4];
        for row in 0..4 {
            inverse[row][row] = <T as One>::one();
            let v = rows.col[row];
            values[row][0] = v.x;
            values[row][1] = v.y;
            values[row][2] = v.z;
            values[row][3] = v.w;
        }
        invert_rows(&mut values, &mut inverse)?;
        let mut result = Self::identity();
        result.col[0].x = inverse[0][0];
        result.col[0].y = inverse[1][0];
        result.col[0].z = inverse[2][0];
        result.col[0].w = inverse[3][0];
        result.col[1].x = inverse[0][1];
        result.col[1].y = inverse[1][1];
        result.col[1].z = inverse[2][1];
        result.col[1].w = inverse[3][1];
        result.col[2].x = inverse[0][2];
        result.col[2].y = inverse[1][2];
        result.col[2].z = inverse[2][2];
        result.col[2].w = inverse[3][2];
        result.col[3].x = inverse[0][3];
        result.col[3].y = inverse[1][3];
        result.col[3].z = inverse[2][3];
        result.col[3].w = inverse[3][3];
        Ok(result)
    }

    /// Computes the inverse of the matrix.
    ///
    /// Uses the reciprocal/adjugate path by default; `inverse_with` selects rows:
    /// ```text
    /// M⁻¹ = (1/det(M)) * adj(M)
    /// ```
    ///
    /// # Note
    /// Panics for singular, non-finite or unrepresentable matrices. Use `try_inverse`
    /// to inspect the numerical error.
    pub fn inverse(&self) -> Self
    where
        T: RealScalar,
    {
        self.try_inverse()
            .expect("matrix inverse is not representable")
    }

    /// Checks the affine last row and inverts its linear part before computing
    /// the inverse translation. All products and negations are checked.
    pub fn try_inverse_affine(&self) -> Result<Self, MathError>
    where
        T: RealScalar,
    {
        self.try_inverse_affine_with(InverseMode::default())
    }

    /// Inverts an affine matrix with an explicit linear inversion algorithm.
    pub fn try_inverse_affine_with(&self, mode: InverseMode) -> Result<Self, MathError>
    where
        T: RealScalar,
    {
        if !self.is_affine(T::epsilon()) {
            return Err(MathError::Domain);
        }
        let rot = Matrix3::new(
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
        .try_inverse_with(mode)?;
        let translation = Vector3::new(self.col[3].x, self.col[3].y, self.col[3].z);
        let inverse_translation = Matrix3::try_mul_matrix_vector(&rot, &translation)?;
        Ok(Self::new(
            rot.col[0].x,
            rot.col[0].y,
            rot.col[0].z,
            <T as Zero>::zero(),
            rot.col[1].x,
            rot.col[1].y,
            rot.col[1].z,
            <T as Zero>::zero(),
            rot.col[2].x,
            rot.col[2].y,
            rot.col[2].z,
            <T as Zero>::zero(),
            inverse_translation.x.try_neg()?,
            inverse_translation.y.try_neg()?,
            inverse_translation.z.try_neg()?,
            <T as One>::one(),
        ))
    }

    /// Selects the affine inverse algorithm, panicking on numerical failure.
    pub fn inverse_affine_with(&self, mode: InverseMode) -> Self
    where
        T: RealScalar,
    {
        self.try_inverse_affine_with(mode)
            .expect("affine inverse failed")
    }

    /// Computes the inverse of an affine matrix (rotation/scale + translation).
    ///
    /// Assumes the last row is `[0, 0, 0, 1]`.
    pub fn inverse_affine(&self) -> Self
    where
        T: RealScalar,
    {
        self.try_inverse_affine()
            .expect("affine inverse is not representable")
    }

    /// Multiplies two 4x4 matrices.
    ///
    /// Matrix multiplication follows the rule:
    /// ```text
    /// C[i,j] = Σₖ A[i,k] * B[k,j]
    /// ```
    pub fn mul_matrix_matrix(l: &Self, r: &Self) -> Self {
        let rows = l.transpose();
        let mut result = Self::identity();
        for column in 0..4 {
            result.col[column] = Self::mul_vector_matrix(&r.col[column], &rows);
        }
        result
    }

    /// Multiplies a 4x4 matrix by a 4D vector.
    ///
    /// ```text
    /// v' = M * v
    /// ```
    pub fn mul_matrix_vector(l: &Self, r: &Vector4<T>) -> Vector4<T> {
        Self::mul_vector_matrix(r, &l.transpose())
    }

    //
    //                     [m0 = c0_x | m4 = c1_x | m8 = c2_x | m12= c3_x]
    // [v_x v_y v_z v_w] * [m1 = c0_y | m5 = c1_y | m9 = c2_y | m13= c3_y] = [dot(v, c0) dot(v, c1) dot(v, c2) dot(v, c3)]
    //                     [m2 = c0_z | m6 = c1_z | m10= c2_z | m14= c3_z]
    //                     [m3 = c0_w | m7 = c1_w | m11= c2_w | m15= c3_w]
    //
    /// Multiplies a 4D vector by a 4x4 matrix (row vector).
    ///
    /// ```text
    /// v' = vᵀ * M
    /// ```
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
    ($mat:ident, $vec:ident, $($field:ident),+) => {
        impl<T: Scalar> $mat<T> {
            /// Multiplies a matrix and column vector using checked dot products.
            pub fn try_mul_matrix_vector(l: &Self, r: &$vec<T>) -> Result<$vec<T>, MathError> {
                let rows = l.transpose();
                let mut rows = rows.col.iter();
                let mut result = <$vec<T> as Vector<T>>::zero();
                $(result.$field = $vec::try_dot(rows.next().expect("matrix row count"),r)?;)*
                Ok(result)
            }
            /// Multiplies matrices with one fixed quantization per output cell.
            pub fn try_mul_matrix_matrix(l: &Self, r: &Self) -> Result<Self, MathError> {
                let mut result = Self::identity();
                for i in 0..result.col.len() {
                    result.col[i] = Self::try_mul_matrix_vector(l,&r.col[i])?;
                }
                Ok(result)
            }
        }
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

implMatrixOps!(Matrix2, Vector2, x, y);
implMatrixOps!(Matrix3, Vector3, x, y, z);
implMatrixOps!(Matrix4, Vector4, x, y, z, w);

impl<T: Scalar> Mul<Matrix4<T>> for Vector3<T> {
    type Output = Vector3<T>;
    fn mul(self, rhs: Matrix4<T>) -> Vector3<T> {
        Matrix4::mul_vector_matrix(
            &Vector4::new(self.x, self.y, self.z, <T as One>::one()),
            &rhs,
        )
        .xyz()
    }
}

impl<T: Scalar> Mul<Vector3<T>> for Matrix4<T> {
    type Output = Vector3<T>;
    fn mul(self, rhs: Vector3<T>) -> Vector3<T> {
        Matrix4::mul_matrix_vector(&self, &Vector4::new(rhs.x, rhs.y, rhs.z, <T as One>::one()))
            .xyz()
    }
}

/// Convenience extension for extracting a 3x3 submatrix.
pub trait Matrix4Extension<T: Scalar> {
    /// Returns the upper-left 3x3 rotation/scale submatrix.
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

#[cfg(all(test, any(feature = "std", feature = "libm", feature = "system-libm")))]
mod tests {
    use super::*;

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
        let m = Matrix3::<f32>::new(1.0, 0.0, 0.0, 0.0, 1.0, 0.0, 0.0, 0.0, 1.0);
        assert_eq!(m.determinant(), 1.0);

        let m2 = Matrix3::<f32>::new(2.0, 3.0, 1.0, 1.0, 0.0, 2.0, 1.0, 2.0, 1.0);
        let det = m2.determinant();
        assert!((det - -3.0).abs() < 0.001);
    }

    #[test]
    fn test_matrix3_inverse() {
        let m = Matrix3::<f32>::new(2.0, 3.0, 1.0, 1.0, 0.0, 2.0, 1.0, 2.0, 1.0);
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
            2.0, 0.0, 0.0, 0.0, 0.0, 1.0, 0.0, 0.0, 0.0, 0.0, 0.5, 0.0, 1.0, 2.0, 3.0, 1.0,
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
                assert!(
                    (val - expected).abs() < 0.001,
                    "Matrix inverse failed at [{}, {}]: expected {}, got {}",
                    i,
                    j,
                    expected,
                    val
                );
            }
        }
    }

    #[test]
    fn test_matrix4_inverse_affine() {
        let m = Matrix4::<f32>::new(
            0.0, 1.0, 0.0, 0.0, -1.0, 0.0, 0.0, 0.0, 0.0, 0.0, 2.0, 0.0, 3.0, 4.0, 5.0, 1.0,
        );
        assert!(m.is_affine(EPS_F32));

        let inv_affine = m.inverse_affine();
        let inv_full = m.inverse();

        for i in 0..4 {
            let a = inv_affine.col[i];
            let b = inv_full.col[i];
            assert!((a.x - b.x).abs() < 0.001);
            assert!((a.y - b.y).abs() < 0.001);
            assert!((a.z - b.z).abs() < 0.001);
            assert!((a.w - b.w).abs() < 0.001);
        }

        let non_affine = Matrix4::<f32>::new(
            1.0, 0.0, 0.0, 0.0, 0.0, 1.0, 0.0, 0.0, 0.0, 0.0, 1.0, -1.0, 0.0, 0.0, 0.0, 0.0,
        );
        assert!(!non_affine.is_affine(EPS_F32));
    }

    #[test]
    fn test_matrix_try_cast() {
        let mi = Matrix2::<i32>::new(1, 2, 3, 4);
        let mf = mi
            .try_cast::<f32>()
            .expect("integer matrix should cast to f32");
        assert_eq!(mf.col[0].x, 1.0);
        assert_eq!(mf.col[0].y, 2.0);
        assert_eq!(mf.col[1].x, 3.0);
        assert_eq!(mf.col[1].y, 4.0);
    }
}
