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

//! Signed binary fixed-point numbers with explicit storage and precision.
//!
//! [`Fixed<i32, 8>`] is S23.8: one sign bit, 23 integer value bits and eight
//! fractional bits. Arithmetic rounds to nearest, ties to even. Operators and
//! `From<f32/f64>` panic on invalid or unrepresentable results; `try_*` methods
//! return [`MathError`]. Floating conversions are explicit interoperability,
//! never the implementation of fixed arithmetic or analytic functions.
//!
//! ```
//! use rs_math3d::Fixed;
//! type F = Fixed<i32, 8>;
//! let a = F::try_from_ratio(3, 2).unwrap();
//! let b: F = 2.0f32.into();
//! let result: f64 = (a * b).into();
//! assert_eq!(result, 3.0);
//! assert_eq!(a.to_bits(), 384);
//! ```
//!
//! Invalid formats fail at the type boundary:
//! ```compile_fail
//! use rs_math3d::Fixed;
//! let _ = Fixed::<i32, 32>::from_bits(0);
//! ```
//! Unsigned storage does not denote a signed fixed-point number:
//! ```compile_fail
//! use rs_math3d::Fixed;
//! let _ = Fixed::<u32, 8>::from_bits(0);
//! ```
//!
//! Storage-valid formats must also represent the constants required by their
//! geometry traits. S1.30 cannot represent `Scalar::two()`:
//! ```compile_fail
//! use rs_math3d::{Fixed, Scalar};
//! fn needs_scalar<T: Scalar>() {}
//! needs_scalar::<Fixed<i32, 30>>();
//! ```
//! Analytic formats must represent an exact quarter:
//! ```compile_fail
//! use rs_math3d::{Fixed, RealScalar};
//! fn needs_real<T: RealScalar>() {}
//! needs_real::<Fixed<i32, 1>>();
//! ```

mod analytic;
mod arithmetic;
mod constants;
mod parse;
mod reciprocal;
mod storage;

use crate::MathError;
use arithmetic::{round_div, round_shift, signed, Accumulator};
use core::{fmt, ops::*, str::FromStr};
use num_traits::{FromPrimitive, Num, NumCast, One, ToPrimitive, Zero};
pub use reciprocal::FixedReciprocal;
pub use storage::{Format, RealFormat, Repr, ScalarFormat, UnitFormat};

/// A signed binary rational `bits / 2^FRAC` with no hidden runtime state.
///
/// Valid storage is `i8`, `i16`, `i32`, or `i64`, with fewer fractional bits
/// than storage bits. `Num` requires room for positive one, `Scalar` requires
/// room for two, and `RealScalar` additionally requires at least two fractional
/// bits. Formats without those capabilities still support raw arithmetic.
#[repr(transparent)]
#[derive(Clone, Copy, Eq, PartialEq, Ord, PartialOrd, Hash)]
pub struct Fixed<R: Format<F>, const F: u32> {
    /// The authoritative signed scaled integer; all bit patterns are valid.
    bits: R,
}

/// S23.8 stored in a signed 32-bit integer.
pub type S23_8 = Fixed<i32, 8>;
/// S15.16 stored in a signed 32-bit integer.
pub type S15_16 = Fixed<i32, 16>;
/// S47.16 stored in a signed 64-bit integer.
pub type S47_16 = Fixed<i64, 16>;

impl<R: Format<F>, const F: u32> Fixed<R, F> {
    /// Exact numeric zero.
    pub const ZERO: Self = Self { bits: R::ZERO };
    /// One quantum, equal to `2^-FRAC`.
    pub const DELTA: Self = Self { bits: R::UNIT };
    /// Number of integer magnitude bits, excluding the sign bit.
    pub const INT_BITS: u32 = R::BITS - 1 - F;
    /// Number of fractional bits.
    pub const FRAC_BITS: u32 = F;

    /// Constructs the exact scaled integer without numeric conversion.
    pub const fn from_bits(bits: R) -> Self {
        Self { bits }
    }
    /// Returns the exact stored scaled integer.
    pub const fn to_bits(self) -> R {
        self.bits
    }
    /// Returns the smallest representable number.
    pub fn min_value() -> Self {
        Self::raw(R::MIN).expect("storage minimum")
    }
    /// Returns the largest representable number.
    pub fn max_value() -> Self {
        Self::raw(R::MAX).expect("storage maximum")
    }

    /// Narrows a widened raw value only after checking its storage range.
    pub(super) fn raw(bits: i128) -> Result<Self, MathError> {
        R::narrow(bits)
            .map(Self::from_bits)
            .ok_or(MathError::Overflow)
    }
    /// Exact integer scale. Valid formats always fit this in positive `i128`.
    pub(super) fn scale() -> i128 {
        1i128 << F
    }

    /// Constructs an integer, rejecting overflow before narrowing.
    pub fn try_from_int(value: i64) -> Result<Self, MathError> {
        Self::raw((value as i128) * Self::scale())
    }
    /// Constructs a rational, rounded once to nearest with ties to even.
    pub fn try_from_ratio(numerator: i64, denominator: i64) -> Result<Self, MathError> {
        Self::raw(round_div(
            (numerator as i128) * Self::scale(),
            denominator as i128,
        )?)
    }
    /// Converts an IEEE single-precision value exactly before quantization.
    pub fn try_from_f32(value: f32) -> Result<Self, MathError> {
        Self::try_from_f64(value as f64)
    }
    /// Decodes IEEE bits before quantization, preserving range checks even
    /// where converting a signed integer maximum to `f64` rounds upward.
    pub fn try_from_f64(value: f64) -> Result<Self, MathError> {
        let bits = value.to_bits();
        let negative = bits >> 63 != 0;
        let exponent = ((bits >> 52) & 0x7ff) as i32;
        if exponent == 0x7ff {
            return Err(MathError::NonFinite);
        }
        let fraction = bits & ((1u64 << 52) - 1);
        let (mantissa, exponent) = if exponent == 0 {
            (fraction as u128, -1074)
        } else {
            (((1u64 << 52) | fraction) as u128, exponent - 1075)
        };
        let shift = exponent + F as i32;
        let magnitude = if shift >= 0 {
            let factor = 1u128.checked_shl(shift as u32).ok_or(MathError::Overflow)?;
            mantissa.checked_mul(factor).ok_or(MathError::Overflow)?
        } else {
            round_shift(mantissa, (-shift) as u32)
        };
        Self::raw(signed(magnitude, negative)?)
    }
    /// Converts to `f32`; values beyond its significand precision are rounded.
    pub fn to_f32(self) -> f32 {
        self.bits.widen() as f32 / Self::scale() as f32
    }
    /// Converts to `f64`; values beyond its significand precision are rounded.
    pub fn to_f64(self) -> f64 {
        self.bits.widen() as f64 / Self::scale() as f64
    }

    /// Converts storage/precision using integer arithmetic and one rounding.
    pub fn try_rescale<S: Format<G>, const G: u32>(self) -> Result<Fixed<S, G>, MathError> {
        let bits = self.bits.widen();
        let result = if G >= F {
            bits * (1i128 << (G - F))
        } else {
            round_div(bits, 1i128 << (F - G))?
        };
        Fixed::<S, G>::raw(result)
    }
    /// Adds without wrapping, returning an error if the result is out of range.
    pub fn try_add(self, rhs: Self) -> Result<Self, MathError> {
        Self::raw(self.bits.widen() + rhs.bits.widen())
    }
    /// Subtracts without wrapping.
    pub fn try_sub(self, rhs: Self) -> Result<Self, MathError> {
        Self::raw(self.bits.widen() - rhs.bits.widen())
    }
    /// Multiplies widened integers and rounds exactly once.
    #[inline]
    pub fn try_mul(self, rhs: Self) -> Result<Self, MathError> {
        let product = self.bits.widen() * rhs.bits.widen();
        Self::raw(signed(round_shift(product.unsigned_abs(), F), product < 0)?)
    }
    /// Divides widened integers, detecting a zero divisor and result overflow.
    pub fn try_div(self, rhs: Self) -> Result<Self, MathError> {
        Self::raw(round_div(
            self.bits.widen() * Self::scale(),
            rhs.bits.widen(),
        )?)
    }
    /// Returns the truncating remainder; its sign follows the dividend.
    pub fn try_rem(self, rhs: Self) -> Result<Self, MathError> {
        if rhs.bits.widen() == 0 {
            return Err(MathError::DivisionByZero);
        }
        Self::raw(self.bits.widen() % rhs.bits.widen())
    }
    /// Negates, rejecting the asymmetric minimum's positive counterpart.
    pub fn try_neg(self) -> Result<Self, MathError> {
        Self::raw(-self.bits.widen())
    }
    /// Returns a checked absolute value, including the signed minimum boundary.
    pub fn try_abs(self) -> Result<Self, MathError> {
        Self::raw(self.bits.widen().abs())
    }
    /// Truncates toward zero. The resulting integral value remains representable.
    pub fn trunc(self) -> Self {
        Self::raw(self.bits.widen() / Self::scale() * Self::scale()).expect("truncation range")
    }
    /// Rounds toward negative infinity.
    pub fn floor(self) -> Self {
        Self::raw(self.bits.widen().div_euclid(Self::scale()) * Self::scale()).expect("floor range")
    }
    /// Rounds toward positive infinity, returning an error at an upper boundary.
    pub fn try_ceil(self) -> Result<Self, MathError> {
        let n = self.bits.widen();
        Self::raw((-(-n).div_euclid(Self::scale())) * Self::scale())
    }
    /// Rounds to an integral value, ties to even, checking the final range.
    pub fn try_round(self) -> Result<Self, MathError> {
        Self::raw(round_div(self.bits.widen(), Self::scale())? * Self::scale())
    }
    /// Computes a signed sum of products exactly before a single quantization.
    /// Each tuple contains `(left, right, subtract_product)`.
    pub(crate) fn sum_products(terms: &[(Self, Self, bool)]) -> Result<Self, MathError> {
        let mut accumulator = Accumulator::default();
        for &(a, b, subtract) in terms {
            accumulator.add(a.bits.widen() * b.bits.widen(), subtract)?;
        }
        Self::raw(accumulator.finish(F)?)
    }
    /// Parses signed integer/fractional digits in radices 2 through 36.
    /// Whitespace, exponents and digit separators are deliberately rejected.
    pub fn from_str_radix(text: &str, radix: u32) -> Result<Self, MathError> {
        parse::parse(text, radix)
    }
}

impl<R: UnitFormat<F>, const F: u32> Fixed<R, F> {
    /// Exact numeric one; available only when positive one is representable.
    pub const ONE: Self = Self { bits: R::ONE_BITS };
}

impl<R: Format<F>, const F: u32> Default for Fixed<R, F> {
    fn default() -> Self {
        Self::ZERO
    }
}
impl<R: Format<F>, const F: u32> fmt::Debug for Fixed<R, F> {
    fn fmt(&self, f: &mut fmt::Formatter<'_>) -> fmt::Result {
        write!(f, "S{}.{}({})", Self::INT_BITS, F, self)
    }
}
impl<R: Format<F>, const F: u32> fmt::Display for Fixed<R, F> {
    fn fmt(&self, f: &mut fmt::Formatter<'_>) -> fmt::Result {
        parse::display(*self, f)
    }
}
impl<R: Format<F>, const F: u32> FromStr for Fixed<R, F> {
    type Err = MathError;
    fn from_str(text: &str) -> Result<Self, Self::Err> {
        Self::from_str_radix(text, 10)
    }
}

/// Generates operators from checked arithmetic so release and debug agree.
macro_rules! operators {
    ($($op:ident, $method:ident, $assign:ident, $assign_method:ident, $try:ident);* $(;)?) => { $(
        impl<R: Format<F>, const F: u32> $op for Fixed<R, F> {
            type Output = Self;
            fn $method(self, rhs: Self) -> Self { self.$try(rhs).expect("fixed arithmetic failed") }
        }
        impl<R: Format<F>, const F: u32> $assign for Fixed<R, F> {
            fn $assign_method(&mut self, rhs: Self) { *self = self.$try(rhs).expect("fixed arithmetic failed"); }
        }
    )* };
}
operators!(Add, add, AddAssign, add_assign, try_add;
    Sub, sub, SubAssign, sub_assign, try_sub;
    Mul, mul, MulAssign, mul_assign, try_mul;
    Div, div, DivAssign, div_assign, try_div;
    Rem, rem, RemAssign, rem_assign, try_rem;);

impl<R: Format<F>, const F: u32> Neg for Fixed<R, F> {
    type Output = Self;
    fn neg(self) -> Self {
        self.try_neg().expect("fixed negation overflow")
    }
}
impl<R: Format<F>, const F: u32> Zero for Fixed<R, F> {
    fn zero() -> Self {
        Self::ZERO
    }
    fn is_zero(&self) -> bool {
        self.bits.widen() == 0
    }
}
impl<R: UnitFormat<F>, const F: u32> One for Fixed<R, F> {
    fn one() -> Self {
        Self::ONE
    }
}
impl<R: UnitFormat<F>, const F: u32> Num for Fixed<R, F> {
    type FromStrRadixErr = MathError;
    fn from_str_radix(s: &str, radix: u32) -> Result<Self, MathError> {
        Self::from_str_radix(s, radix)
    }
}

/// Defines both conversion directions. `Into` is supplied by Rust's blanket
/// implementation; implementing it separately would conflict with that impl.
macro_rules! float_conversions {
    ($($float:ty, $from:ident, $to:ident);*) => { $(
        impl<R: Format<F>, const F: u32> From<$float> for Fixed<R, F> {
            /// Rounds to nearest, ties to even.
            /// # Panics
            /// Panics for NaN, infinity, or an out-of-range rounded result.
            fn from(value: $float) -> Self { Self::$from(value).expect("invalid float-to-fixed conversion") }
        }
        impl<R: Format<F>, const F: u32> From<Fixed<R, F>> for $float {
            /// Converts numerically, rounding if the float lacks significand bits.
            fn from(value: Fixed<R, F>) -> Self { value.$to() }
        }
    )* };
}
float_conversions!(f32, try_from_f32, to_f32; f64, try_from_f64, to_f64);

#[cfg(test)]
mod tests;

impl<R: Format<F>, const F: u32> ToPrimitive for Fixed<R, F> {
    fn to_i64(&self) -> Option<i64> {
        Some((self.bits.widen() / Self::scale()) as i64)
    }
    fn to_u64(&self) -> Option<u64> {
        let value = self.bits.widen() / Self::scale();
        if value < 0 {
            None
        } else {
            Some(value as u64)
        }
    }
    fn to_f32(&self) -> Option<f32> {
        Some((*self).to_f32())
    }
    fn to_f64(&self) -> Option<f64> {
        Some((*self).to_f64())
    }
}
impl<R: Format<F>, const F: u32> FromPrimitive for Fixed<R, F> {
    fn from_i64(value: i64) -> Option<Self> {
        Self::try_from_int(value).ok()
    }
    fn from_u64(value: u64) -> Option<Self> {
        Self::raw(value as i128 * Self::scale()).ok()
    }
    fn from_f32(value: f32) -> Option<Self> {
        Self::try_from_f32(value).ok()
    }
    fn from_f64(value: f64) -> Option<Self> {
        Self::try_from_f64(value).ok()
    }
}
impl<R: Format<F>, const F: u32> NumCast for Fixed<R, F> {
    /// Convenience cast through `f64`; use `try_rescale` for exact fixed casts.
    fn from<T: ToPrimitive>(value: T) -> Option<Self> {
        Self::try_from_f64(value.to_f64()?).ok()
    }
}
