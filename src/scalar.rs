//! Scalar trait definitions for generic numeric operations.
//!
//! This module defines traits that extend `num-traits` for use in
//! mathematical operations throughout the library. It provides a
//! unified interface for integer, floating-point, and signed fixed-point types.
//!
//! The module leverages the `num-traits` crate for basic numeric
//! operations while adding specialized methods needed for 3D math.

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

#[cfg(any(feature = "std", feature = "libm", feature = "system-libm"))]
use crate::math::Math;
use crate::MathError;
use core::cmp::PartialOrd;
use core::ops::Neg;
use num_traits::{Num, NumAssignOps};

// Re-export for convenience
pub use num_traits::{One, Zero};

/// Default epsilon for f32 comparisons.
pub const EPS_F32: f32 = 1.0 / (1024.0 * 1024.0);
/// Default epsilon for f64 comparisons.
pub const EPS_F64: f64 = 1.0 / (1024.0 * 1024.0 * 1024.0 * 1024.0);

/// Core scalar trait for numeric types used in the library.
///
/// This trait is intentionally limited to operations that make sense for both
/// integer and floating-point types. Analytic and fractional operations live on
/// [`RealScalar`].
///
/// It is implemented for `i32`, `i64`, `f32`, `f64`, and fixed formats that can represent two.
///
/// # Required Methods
///
/// Types implementing this trait must provide:
/// - Basic arithmetic operations (via `Num` and `NumAssignOps`)
/// - Comparison operations (via `PartialOrd`)
/// - Additional constants and utility methods
pub trait Scalar:
    Num + NumAssignOps + Neg<Output = Self> + PartialOrd + Clone + Copy + Sized
{
    /// Adds with a range check. Implementations must not wrap before checking.
    fn try_add(self, rhs: Self) -> Result<Self, MathError>;
    /// Subtracts with a range check.
    fn try_sub(self, rhs: Self) -> Result<Self, MathError>;
    /// Multiplies with a range check and implementation-defined quantization.
    fn try_mul(self, rhs: Self) -> Result<Self, MathError>;
    /// Divides with zero and range checks.
    fn try_div(self, rhs: Self) -> Result<Self, MathError>;
    /// Negates with a range check.
    fn try_neg(self) -> Result<Self, MathError>;
    /// Accumulates signed products. Fixed implementations round only once.
    fn try_sum_products(terms: &[(Self, Self, bool)]) -> Result<Self, MathError> {
        let mut sum = <Self as Zero>::zero();
        for &(a, b, subtract) in terms {
            let product = a.try_mul(b)?;
            sum = if subtract {
                sum.try_sub(product)?
            } else {
                sum.try_add(product)?
            };
        }
        Ok(sum)
    }
    /// Computes a signed sum of products, panicking on arithmetic failure.
    fn sum_products(terms: &[(Self, Self, bool)]) -> Self {
        Self::try_sum_products(terms).expect("scalar product accumulation failed")
    }
    // Additional methods not provided by num-traits
    /// Returns the constant 2 for the scalar type.
    fn two() -> Self;
    /// Returns the minimum of two values.
    fn min(l: Self, r: Self) -> Self;
    /// Returns the maximum of two values.
    fn max(l: Self, r: Self) -> Self;
    /// Returns the squared value (self * self).
    fn squared(self) -> Self {
        self * self
    }
    /// Returns the absolute value.
    fn tabs(self) -> Self;
}

/// Trait for fractional scalars with checked analytic operations.
///
/// Extends `Scalar` with trigonometric functions, square roots and stable norms.
/// Fixed implementations use integer arithmetic. Floating implementations
/// require one of the explicitly selected floating-point backends.
///
/// # Backend selection
///
/// Floating-point functions are routed through a compile-time math backend. The default
/// `std` backend supports hosted crates even when their source uses
/// `#![no_std]`, provided they can explicitly link Rust's standard library.
/// Programs that cannot link `std` must disable default features and enable
/// `libm`; Unix users may instead opt into `system-libm`. Featureless builds
/// support fixed-point analytic math. Unified floating features use the
/// precedence `std`, then `libm`, then `system-libm`.
pub trait RealScalar: Scalar {
    /// Reusable reciprocal representation. Fixed implementations retain guard
    /// precision rather than quantizing `1 / denominator` to the scalar format.
    type Reciprocal: Copy;
    /// Prepares one reciprocal for repeated checked multiplication.
    fn try_reciprocal(self) -> Result<Self::Reciprocal, MathError>;
    /// Applies a prepared reciprocal, rounding only the final scalar result.
    fn try_mul_reciprocal(self, reciprocal: Self::Reciprocal) -> Result<Self, MathError>;
    /// Returns a reasonable epsilon for comparisons.
    fn epsilon() -> Self;
    /// Returns 1/2 for the scalar type.
    fn half() -> Self;
    /// Returns 1/4 for the scalar type.
    fn quarter() -> Self;
    /// Returns whether this value is finite. Fixed-point values always are.
    fn is_finite(self) -> bool;
    /// Computes a checked absolute value, including signed storage minima.
    fn try_abs(self) -> Result<Self, MathError>;
    /// Computes a norm without first quantizing small component squares.
    fn try_norm(values: &[Self]) -> Result<Self, MathError> {
        let mut scale = <Self as Zero>::zero();
        for &v in values {
            if !v.is_finite() {
                return Err(MathError::NonFinite);
            }
            scale = Self::max(scale, v.try_abs()?);
        }
        if scale == <Self as Zero>::zero() {
            return Ok(scale);
        }
        let mut sum = <Self as Zero>::zero();
        for &v in values {
            let q = v.try_div(scale)?;
            sum = sum.try_add(q.try_mul(q)?)?;
        }
        sum.try_sqrt()?.try_mul(scale)
    }
    /// Tests the exact represented length against a nonnegative tolerance.
    /// Fixed implementations compare unquantized squared integer quantities.
    fn norm_exceeds(values: &[Self], epsilon: Self) -> Result<bool, MathError> {
        let zero = <Self as Zero>::zero();
        if !epsilon.is_finite() || epsilon < zero {
            return Err(MathError::InvalidTolerance);
        }
        let mut scale = zero;
        for &value in values {
            scale = Self::max(scale, value.try_abs()?);
        }
        if scale == zero {
            return Ok(false);
        }
        if scale > epsilon {
            return Ok(true);
        }
        let allowed = match epsilon.try_div(scale) {
            Ok(value) => value,
            // A finite tolerance divided by a tiny component can overflow only
            // far above every possible bounded scaled norm for a real slice.
            Err(MathError::Overflow) => return Ok(false),
            Err(error) => return Err(error),
        };
        let mut sum = zero;
        for &value in values {
            let scaled = value.try_div(scale)?;
            sum = sum.try_add(scaled.try_mul(scaled)?)?;
        }
        Ok(sum.try_sqrt()? > allowed)
    }
    /// Checked square root; negative inputs are outside its domain.
    fn try_sqrt(self) -> Result<Self, MathError> {
        if !self.is_finite() {
            return Err(MathError::NonFinite);
        }
        if self < <Self as Zero>::zero() {
            return Err(MathError::Domain);
        }
        let result = self.tsqrt();
        if result.is_finite() {
            Ok(result)
        } else {
            Err(MathError::Overflow)
        }
    }
    /// Checked sine for a finite radian angle.
    fn try_sin(self) -> Result<Self, MathError> {
        if !self.is_finite() {
            return Err(MathError::NonFinite);
        }
        let result = self.tsin();
        if result.is_finite() {
            Ok(result)
        } else {
            Err(MathError::Overflow)
        }
    }
    /// Checked cosine for a finite radian angle.
    fn try_cos(self) -> Result<Self, MathError> {
        if !self.is_finite() {
            return Err(MathError::NonFinite);
        }
        let result = self.tcos();
        if result.is_finite() {
            Ok(result)
        } else {
            Err(MathError::Overflow)
        }
    }
    /// Checked tangent; overflow near a pole is reported explicitly.
    fn try_tan(self) -> Result<Self, MathError> {
        if !self.is_finite() {
            return Err(MathError::NonFinite);
        }
        let result = self.ttan();
        if result.is_finite() {
            Ok(result)
        } else {
            Err(MathError::Overflow)
        }
    }
    /// Checked arc cosine on the closed domain [-1, 1].
    fn try_acos(self) -> Result<Self, MathError> {
        if !self.is_finite() {
            return Err(MathError::NonFinite);
        }
        if self < -<Self as One>::one() || self > <Self as One>::one() {
            return Err(MathError::Domain);
        }
        Ok(self.tacos())
    }
    /// Returns the square root.
    fn tsqrt(self) -> Self;
    /// Returns the sine (radians).
    fn tsin(self) -> Self;
    /// Returns the cosine (radians).
    fn tcos(self) -> Self;
    /// Returns the tangent (radians).
    fn ttan(self) -> Self;
    /// Returns the arc cosine (radians).
    fn tacos(self) -> Self;
}

// Implementation for i32
impl Scalar for i32 {
    fn try_add(self, rhs: Self) -> Result<Self, MathError> {
        self.checked_add(rhs).ok_or(MathError::Overflow)
    }
    fn try_sub(self, rhs: Self) -> Result<Self, MathError> {
        self.checked_sub(rhs).ok_or(MathError::Overflow)
    }
    fn try_mul(self, rhs: Self) -> Result<Self, MathError> {
        self.checked_mul(rhs).ok_or(MathError::Overflow)
    }
    fn try_div(self, rhs: Self) -> Result<Self, MathError> {
        if rhs == 0 {
            Err(MathError::DivisionByZero)
        } else {
            self.checked_div(rhs).ok_or(MathError::Overflow)
        }
    }
    fn try_neg(self) -> Result<Self, MathError> {
        self.checked_neg().ok_or(MathError::Overflow)
    }

    fn two() -> Self {
        2
    }
    fn min(l: Self, r: Self) -> Self {
        if l < r {
            l
        } else {
            r
        }
    }
    fn max(l: Self, r: Self) -> Self {
        if l > r {
            l
        } else {
            r
        }
    }
    fn tabs(self) -> Self {
        self.abs()
    }
}

// Implementation for i64
impl Scalar for i64 {
    fn try_add(self, rhs: Self) -> Result<Self, MathError> {
        self.checked_add(rhs).ok_or(MathError::Overflow)
    }
    fn try_sub(self, rhs: Self) -> Result<Self, MathError> {
        self.checked_sub(rhs).ok_or(MathError::Overflow)
    }
    fn try_mul(self, rhs: Self) -> Result<Self, MathError> {
        self.checked_mul(rhs).ok_or(MathError::Overflow)
    }
    fn try_div(self, rhs: Self) -> Result<Self, MathError> {
        if rhs == 0 {
            Err(MathError::DivisionByZero)
        } else {
            self.checked_div(rhs).ok_or(MathError::Overflow)
        }
    }
    fn try_neg(self) -> Result<Self, MathError> {
        self.checked_neg().ok_or(MathError::Overflow)
    }

    fn two() -> Self {
        2
    }
    fn min(l: Self, r: Self) -> Self {
        if l < r {
            l
        } else {
            r
        }
    }
    fn max(l: Self, r: Self) -> Self {
        if l > r {
            l
        } else {
            r
        }
    }
    fn tabs(self) -> Self {
        self.abs()
    }
}

// Implementation for f32
impl Scalar for f32 {
    fn try_add(self, rhs: Self) -> Result<Self, MathError> {
        if !self.is_finite() || !rhs.is_finite() {
            return Err(MathError::NonFinite);
        }
        let result = self + rhs;
        if result.is_finite() {
            Ok(result)
        } else {
            Err(MathError::Overflow)
        }
    }
    fn try_sub(self, rhs: Self) -> Result<Self, MathError> {
        if !self.is_finite() || !rhs.is_finite() {
            return Err(MathError::NonFinite);
        }
        let result = self - rhs;
        if result.is_finite() {
            Ok(result)
        } else {
            Err(MathError::Overflow)
        }
    }
    fn try_mul(self, rhs: Self) -> Result<Self, MathError> {
        if !self.is_finite() || !rhs.is_finite() {
            return Err(MathError::NonFinite);
        }
        let result = self * rhs;
        if result.is_finite() {
            Ok(result)
        } else {
            Err(MathError::Overflow)
        }
    }
    fn try_div(self, rhs: Self) -> Result<Self, MathError> {
        if !self.is_finite() || !rhs.is_finite() {
            return Err(MathError::NonFinite);
        }
        if rhs == 0.0 {
            return Err(MathError::DivisionByZero);
        }
        let result = self / rhs;
        if result.is_finite() {
            Ok(result)
        } else {
            Err(MathError::Overflow)
        }
    }
    fn try_neg(self) -> Result<Self, MathError> {
        if self.is_finite() {
            Ok(-self)
        } else {
            Err(MathError::NonFinite)
        }
    }
    fn sum_products(terms: &[(Self, Self, bool)]) -> Self {
        terms.iter().fold(
            0.0,
            |s, &(a, b, subtract)| if subtract { s - a * b } else { s + a * b },
        )
    }

    fn two() -> Self {
        2.0
    }
    fn min(l: Self, r: Self) -> Self {
        if l < r {
            l
        } else {
            r
        }
    }
    fn max(l: Self, r: Self) -> Self {
        if l > r {
            l
        } else {
            r
        }
    }
    fn tabs(self) -> Self {
        self.abs()
    }
}

// Implementation for f64
impl Scalar for f64 {
    fn try_add(self, rhs: Self) -> Result<Self, MathError> {
        if !self.is_finite() || !rhs.is_finite() {
            return Err(MathError::NonFinite);
        }
        let result = self + rhs;
        if result.is_finite() {
            Ok(result)
        } else {
            Err(MathError::Overflow)
        }
    }
    fn try_sub(self, rhs: Self) -> Result<Self, MathError> {
        if !self.is_finite() || !rhs.is_finite() {
            return Err(MathError::NonFinite);
        }
        let result = self - rhs;
        if result.is_finite() {
            Ok(result)
        } else {
            Err(MathError::Overflow)
        }
    }
    fn try_mul(self, rhs: Self) -> Result<Self, MathError> {
        if !self.is_finite() || !rhs.is_finite() {
            return Err(MathError::NonFinite);
        }
        let result = self * rhs;
        if result.is_finite() {
            Ok(result)
        } else {
            Err(MathError::Overflow)
        }
    }
    fn try_div(self, rhs: Self) -> Result<Self, MathError> {
        if !self.is_finite() || !rhs.is_finite() {
            return Err(MathError::NonFinite);
        }
        if rhs == 0.0 {
            return Err(MathError::DivisionByZero);
        }
        let result = self / rhs;
        if result.is_finite() {
            Ok(result)
        } else {
            Err(MathError::Overflow)
        }
    }
    fn try_neg(self) -> Result<Self, MathError> {
        if self.is_finite() {
            Ok(-self)
        } else {
            Err(MathError::NonFinite)
        }
    }
    fn sum_products(terms: &[(Self, Self, bool)]) -> Self {
        terms.iter().fold(
            0.0,
            |s, &(a, b, subtract)| if subtract { s - a * b } else { s + a * b },
        )
    }

    fn two() -> Self {
        2.0
    }
    fn min(l: Self, r: Self) -> Self {
        if l < r {
            l
        } else {
            r
        }
    }
    fn max(l: Self, r: Self) -> Self {
        if l > r {
            l
        } else {
            r
        }
    }
    fn tabs(self) -> Self {
        self.abs()
    }
}

#[cfg(any(feature = "std", feature = "libm", feature = "system-libm"))]
impl RealScalar for f32 {
    type Reciprocal = Self;
    fn try_reciprocal(self) -> Result<Self, MathError> {
        <Self as Scalar>::try_div(1.0, self)
    }
    fn try_mul_reciprocal(self, reciprocal: Self) -> Result<Self, MathError> {
        <Self as Scalar>::try_mul(self, reciprocal)
    }
    fn epsilon() -> Self {
        EPS_F32
    }
    fn half() -> Self {
        0.5
    }
    fn quarter() -> Self {
        0.25
    }
    fn is_finite(self) -> bool {
        self.is_finite()
    }
    fn try_abs(self) -> Result<Self, MathError> {
        if self.is_finite() {
            Ok(self.abs())
        } else {
            Err(MathError::NonFinite)
        }
    }
    fn tsqrt(self) -> Self {
        Math::sqrt_f32(self)
    }
    fn tsin(self) -> Self {
        Math::sin_f32(self)
    }
    fn tcos(self) -> Self {
        Math::cos_f32(self)
    }
    fn ttan(self) -> Self {
        Math::tan_f32(self)
    }
    fn tacos(self) -> Self {
        Math::acos_f32(self)
    }
}

#[cfg(any(feature = "std", feature = "libm", feature = "system-libm"))]
impl RealScalar for f64 {
    type Reciprocal = Self;
    fn try_reciprocal(self) -> Result<Self, MathError> {
        <Self as Scalar>::try_div(1.0, self)
    }
    fn try_mul_reciprocal(self, reciprocal: Self) -> Result<Self, MathError> {
        <Self as Scalar>::try_mul(self, reciprocal)
    }
    fn epsilon() -> Self {
        EPS_F64
    }
    fn half() -> Self {
        0.5
    }
    fn quarter() -> Self {
        0.25
    }
    fn is_finite(self) -> bool {
        self.is_finite()
    }
    fn try_abs(self) -> Result<Self, MathError> {
        if self.is_finite() {
            Ok(self.abs())
        } else {
            Err(MathError::NonFinite)
        }
    }
    fn tsqrt(self) -> Self {
        Math::sqrt_f64(self)
    }
    fn tsin(self) -> Self {
        Math::sin_f64(self)
    }
    fn tcos(self) -> Self {
        Math::cos_f64(self)
    }
    fn ttan(self) -> Self {
        Math::tan_f64(self)
    }
    fn tacos(self) -> Self {
        Math::acos_f64(self)
    }
}

#[cfg(all(test, any(feature = "std", feature = "libm", feature = "system-libm")))]
mod tests {
    use super::*;
    #[test]
    pub fn test() {
        let out = -1.0;
        let f = out.tabs();
        assert_eq!(f, 1.0);
    }

    #[test]
    fn test_float_scalar_ops_f32() {
        let v = 4.0f32;
        assert!((v.tsqrt() - 2.0).abs() < 0.0001);
        assert!(0.0f32.tsin().abs() < 0.0001);
        assert!((0.0f32.tcos() - 1.0).abs() < 0.0001);
        assert!(0.0f32.ttan().abs() < 0.0001);
        assert!(1.0f32.tacos().abs() < 0.0001);
    }

    #[test]
    fn test_float_scalar_ops_f64() {
        let v = 4.0f64;
        assert!((v.tsqrt() - 2.0).abs() < 0.0000001);
        assert!(0.0f64.tsin().abs() < 0.0000001);
        assert!((0.0f64.tcos() - 1.0).abs() < 0.0000001);
        assert!(0.0f64.ttan().abs() < 0.0000001);
        assert!(1.0f64.tacos().abs() < 0.0000001);
    }
}
