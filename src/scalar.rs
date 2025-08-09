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

use num_traits::{Num, NumAssignOps};
use core::cmp::PartialOrd;
use core::ops::Neg;

// Re-export for convenience
pub use num_traits::{Zero, One};

/// Core scalar trait for numeric types used in the library.
///
/// This trait extends `num-traits` types with additional methods
/// commonly needed in 3D mathematics. It is implemented for
/// `i32`, `i64`, `f32`, and `f64`.
///
/// # Required Methods
///
/// Types implementing this trait must provide:
/// - Basic arithmetic operations (via `Num` and `NumAssignOps`)
/// - Comparison operations (via `PartialOrd`)
/// - Additional constants and utility methods
pub trait Scalar:
    Num
    + NumAssignOps
    + Neg<Output = Self>
    + PartialOrd
    + Clone
    + Copy
    + Sized
{
    // Additional methods not provided by num-traits
    fn two() -> Self;
    fn half() -> Self;
    fn quarter() -> Self;
    fn l8192() -> Self;
    fn epsilon() -> Self;
    fn min(l: Self, r: Self) -> Self;
    fn max(l: Self, r: Self) -> Self;
    fn squared(self) -> Self {
        self * self
    }
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
/// These functions link to external C math libraries (libm on Unix,
/// MSVCRT on Windows) since they're not available in no_std Rust.
pub trait FloatScalar: Scalar {
    fn infinity() -> Self;
    fn tsqrt(self) -> Self;
    fn tsin(self) -> Self;
    fn tcos(self) -> Self;
    fn ttan(self) -> Self;
    fn tacos(self) -> Self;
}

// Implementation for i32
impl Scalar for i32 {
    fn two() -> Self { 2 }
    fn half() -> Self { 0 }  // Integer division
    fn quarter() -> Self { 0 }  // Integer division
    fn l8192() -> Self { 8192 }
    fn epsilon() -> Self { 0 }
    fn min(l: Self, r: Self) -> Self {
        if l < r { l } else { r }
    }
    fn max(l: Self, r: Self) -> Self {
        if l > r { l } else { r }
    }
    fn tabs(self) -> Self {
        self.abs()
    }
}

// Implementation for i64
impl Scalar for i64 {
    fn two() -> Self { 2 }
    fn half() -> Self { 0 }  // Integer division
    fn quarter() -> Self { 0 }  // Integer division
    fn l8192() -> Self { 8192 }
    fn epsilon() -> Self { 0 }
    fn min(l: Self, r: Self) -> Self {
        if l < r { l } else { r }
    }
    fn max(l: Self, r: Self) -> Self {
        if l > r { l } else { r }
    }
    fn tabs(self) -> Self {
        self.abs()
    }
}

// Implementation for f32
impl Scalar for f32 {
    fn two() -> Self { 2.0 }
    fn half() -> Self { 0.5 }
    fn quarter() -> Self { 0.25 }
    fn l8192() -> Self { 8192.0 }
    fn epsilon() -> Self { 
        1.0 / (1024.0 * 1024.0)
    }
    fn min(l: Self, r: Self) -> Self {
        if l < r { l } else { r }
    }
    fn max(l: Self, r: Self) -> Self {
        if l > r { l } else { r }
    }
    fn tabs(self) -> Self {
        self.abs()
    }
}

// Implementation for f64
impl Scalar for f64 {
    fn two() -> Self { 2.0 }
    fn half() -> Self { 0.5 }
    fn quarter() -> Self { 0.25 }
    fn l8192() -> Self { 8192.0 }
    fn epsilon() -> Self {
        1.0 / (1024.0 * 1024.0 * 1024.0 * 1024.0)
    }
    fn min(l: Self, r: Self) -> Self {
        if l < r { l } else { r }
    }
    fn max(l: Self, r: Self) -> Self {
        if l > r { l } else { r }
    }
    fn tabs(self) -> Self {
        self.abs()
    }
}

// FloatScalar implementation for f32
// Note: Without std or libm, we need to provide our own implementations
// or link to external math libraries
impl FloatScalar for f32 {
    fn infinity() -> Self {
        core::f32::INFINITY
    }
    fn tsqrt(self) -> Self {
        // Use external C math function
        extern "C" {
            fn sqrtf(x: f32) -> f32;
        }
        unsafe { sqrtf(self) }
    }
    fn tsin(self) -> Self {
        extern "C" {
            fn sinf(x: f32) -> f32;
        }
        unsafe { sinf(self) }
    }
    fn tcos(self) -> Self {
        extern "C" {
            fn cosf(x: f32) -> f32;
        }
        unsafe { cosf(self) }
    }
    fn ttan(self) -> Self {
        extern "C" {
            fn tanf(x: f32) -> f32;
        }
        unsafe { tanf(self) }
    }
    fn tacos(self) -> Self {
        extern "C" {
            fn acosf(x: f32) -> f32;
        }
        unsafe { acosf(self) }
    }
}

// FloatScalar implementation for f64
// Note: Without std or libm, we need to provide our own implementations
// or link to external math libraries
impl FloatScalar for f64 {
    fn infinity() -> Self {
        core::f64::INFINITY
    }
    fn tsqrt(self) -> Self {
        extern "C" {
            fn sqrt(x: f64) -> f64;
        }
        unsafe { sqrt(self) }
    }
    fn tsin(self) -> Self {
        extern "C" {
            fn sin(x: f64) -> f64;
        }
        unsafe { sin(self) }
    }
    fn tcos(self) -> Self {
        extern "C" {
            fn cos(x: f64) -> f64;
        }
        unsafe { cos(self) }
    }
    fn ttan(self) -> Self {
        extern "C" {
            fn tan(x: f64) -> f64;
        }
        unsafe { tan(self) }
    }
    fn tacos(self) -> Self {
        extern "C" {
            fn acos(x: f64) -> f64;
        }
        unsafe { acos(self) }
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
}