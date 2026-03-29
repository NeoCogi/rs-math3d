//! Scalar trait definitions for generic numeric operations.
//!
//! This module defines traits that extend `num-traits` for use in
//! mathematical operations throughout the library. It provides a
//! unified interface for both integer and floating-point types.
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

use crate::math::Math;
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
/// [`FloatScalar`].
///
/// It is implemented for `i32`, `i64`, `f32`, and `f64`.
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

/// Trait for floating-point scalars with transcendental functions.
///
/// Extends the base `Scalar` trait with operations specific to
/// floating-point numbers, including trigonometric functions and
/// square root.
///
/// # Implementation Note
///
/// These functions are routed through the crate's selected math backend:
/// `std`, `libm`, or `system-libm`.
pub trait FloatScalar: Scalar {
    /// Returns a reasonable epsilon for comparisons.
    fn epsilon() -> Self;
    /// Returns 1/2 for the scalar type.
    fn half() -> Self;
    /// Returns 1/4 for the scalar type.
    fn quarter() -> Self;
    /// Returns positive infinity.
    fn infinity() -> Self;
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

impl FloatScalar for f32 {
    fn epsilon() -> Self {
        EPS_F32
    }
    fn half() -> Self {
        0.5
    }
    fn quarter() -> Self {
        0.25
    }
    fn infinity() -> Self {
        core::f32::INFINITY
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

impl FloatScalar for f64 {
    fn epsilon() -> Self {
        EPS_F64
    }
    fn half() -> Self {
        0.5
    }
    fn quarter() -> Self {
        0.25
    }
    fn infinity() -> Self {
        core::f64::INFINITY
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

#[cfg(test)]
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
