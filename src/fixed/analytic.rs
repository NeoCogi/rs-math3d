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

//! Integer-only analytics with default Q96/i128 and opt-in const-selected i64
//! CORDIC. Both use Q64/i128 full-range reduction and guarded u128 acos roots.

use super::{
    arithmetic::{integer_sqrt, round_div},
    constants,
    cordic::{self, Backend, Narrow, Wide},
    Fixed, RealFormat, ScalarFormat,
};
use crate::{MathError, RealScalar, Scalar};

/// Rounds a nonnegative integer square root to the nearest integer. Midpoint
/// ties cannot occur because `(n + 1/2)^2` is never an integer.
fn rounded_sqrt(value: u128) -> u128 {
    let root = integer_sqrt(value);
    root + (value - root * root > root) as u128
}

use super::arithmetic::round_div_wrapping as div;

impl<R: RealFormat<F>, const F: u32> Fixed<R, F> {
    /// Integer square root without domain validation. Supply a nonnegative input.
    pub fn sqrt(self) -> Self {
        let radicand = (self.bits.widen() as u128).wrapping_mul(Self::scale() as u128);
        Self::wrapping_raw(rounded_sqrt(radicand) as i128)
    }
    /// Computes sine and cosine without destination range validation.
    pub fn sin_cos(self) -> (Self, Self) {
        self.sin_cos_with::<128>()
    }
    /// Selects CORDIC's working integer width at compile time: 128 retains the
    /// default Q96 path; experimental 64 uses clamp(F+16,32,60) working fraction
    /// bits and requires F<=48. Both round once to the destination format.
    /// Full-range angle reduction still uses i128. This is an opt-in experiment;
    /// ordinary operators and RealScalar methods continue using the default.
    ///
    /// ```
    /// use rs_math3d::Fixed;
    /// let angle = Fixed::<i32, 16>::try_from_ratio(1, 4).unwrap();
    /// let (sine, cosine) = angle.sin_cos_with::<64>();
    /// assert!(sine > Fixed::ZERO && cosine > sine);
    /// ```
    /// Unsupported widths and insufficient guard precision fail at compilation:
    /// ```compile_fail
    /// use rs_math3d::Fixed;
    /// let _ = Fixed::<i32, 16>::ZERO.sin_cos_with::<32>();
    /// ```
    /// ```compile_fail
    /// use rs_math3d::Fixed;
    /// let _ = Fixed::<i64, 61>::ZERO.sin_cos_with::<64>();
    /// ```
    pub fn sin_cos_with<const WORK_BITS: u32>(self) -> (Self, Self) {
        let (sin, cos) = self.sin_cos_wide_with::<WORK_BITS>();
        (
            Self::wrapping_raw(div(sin, 1i128 << (96 - F))),
            Self::wrapping_raw(div(cos, 1i128 << (96 - F))),
        )
    }
    /// Integer-only sine with ordinary wrapping output quantization.
    pub fn sin(self) -> Self {
        self.sin_cos().0
    }
    /// Integer-only cosine with ordinary wrapping output quantization.
    pub fn cos(self) -> Self {
        self.sin_cos().1
    }
    /// Integer-only tangent without range or pole validation.
    pub fn tan(self) -> Self {
        self.tan_with::<128>()
    }
    /// Const-selected tangent; width restrictions match [`Self::sin_cos_with`].
    /// The experimental i64 guard precision can substantially magnify absolute
    /// error near poles. Division still uses i128, and a zero divisor can panic.
    pub fn tan_with<const WORK_BITS: u32>(self) -> Self {
        let (sin, cos) = self.sin_cos_wide_with::<WORK_BITS>();
        Self::wrapping_raw(div(
            div(sin, 1i128 << 32) * Self::scale(),
            div(cos, 1i128 << 32),
        ))
    }
    /// Arc cosine without domain validation. Supply a value in [-1, 1].
    pub fn acos(self) -> Self {
        self.acos_with::<128>()
    }
    /// Const-selected arc cosine; width restrictions match [`Self::sin_cos_with`].
    /// Supply [-1,1]. The guarded unit-circle root still uses u128 before i64
    /// vectoring; narrowing a squared input in the destination format is avoided.
    pub fn acos_with<const WORK_BITS: u32>(self) -> Self {
        Self::wrapping_raw(div(self.acos_wide_with::<WORK_BITS>(), 1i128 << (96 - F)))
    }
    /// Computes a correctly rounded fixed square root without a float backend.
    pub fn checked_sqrt(self) -> Result<Self, MathError> {
        if self.bits.widen() < 0 {
            return Err(MathError::Domain);
        }
        let radicand = self.bits.widen() as u128 * Self::scale() as u128;
        Self::raw(rounded_sqrt(radicand) as i128)
    }

    /// Computes sine/cosine together, retaining Q96 precision until narrowing.
    /// Range reduction's error is bounded relative to this format's quantum.
    pub fn checked_sin_cos(self) -> Result<(Self, Self), MathError> {
        self.checked_sin_cos_with::<128>()
    }

    /// Const-selected sine/cosine with destination range validation. The i64
    /// restrictions and experimental accuracy policy match [`Self::sin_cos_with`].
    pub fn checked_sin_cos_with<const WORK_BITS: u32>(self) -> Result<(Self, Self), MathError> {
        let (sin, cos) = self.sin_cos_wide_with::<WORK_BITS>();
        Ok((Self::from_q96(sin)?, Self::from_q96(cos)?))
    }

    /// Converts a guarded Q96 analytic result with one ties-to-even rounding.
    fn from_q96(value: i128) -> Result<Self, MathError> {
        Self::raw(round_div(value, 1i128 << (96 - F))?)
    }

    /// Rejects unsupported const policies during monomorphization. Inline const
    /// expressions avoid a runtime policy check and unstable generic const bounds.
    const fn validate_cordic<const WORK_BITS: u32>() {
        assert!(
            WORK_BITS == 64 || WORK_BITS == 128,
            "CORDIC working width must be 64 or 128"
        );
        assert!(
            WORK_BITS != 64 || F <= 48,
            "i64 CORDIC requires at most 48 fractional bits"
        );
    }

    /// Rotates a reduced angle and returns Q96 components for shared output
    /// quantization. The i64 core widens only after its final rotation step.
    fn sin_cos_wide_with<const WORK_BITS: u32>(self) -> (i128, i128) {
        const { Self::validate_cordic::<WORK_BITS>() };
        if self.bits.widen() == 0 {
            return (0, 1i128 << 96);
        }
        // Q64 leaves enough integer headroom for the widest input, even at F=2.
        // Remainder is taken before increasing internal fractional precision.
        let q64 = self.bits.widen() << (64 - F);
        let reduced = q64.rem_euclid(constants::TAU_Q64);
        let reduced = if reduced > constants::TAU_Q64 / 2 {
            reduced - constants::TAU_Q64
        } else {
            reduced
        };
        if WORK_BITS == 64 {
            let frac = Narrow::<F>::FRAC;
            let pi = Narrow::<F>::PI;
            let mut angle = div(reduced, 1i128 << (64 - frac)) as i64;
            let mut cosine_sign = 1;
            if angle > pi / 2 {
                angle = pi - angle;
                cosine_sign = -1;
            } else if angle < -pi / 2 {
                angle = -pi - angle;
                cosine_sign = -1;
            }
            let (sin, cos) = cordic::rotate::<Narrow<F>>(angle);
            return (
                (sin as i128) << (96 - frac),
                ((cos * cosine_sign) as i128) << (96 - frac),
            );
        }
        let mut angle = reduced << 32;
        let mut cosine_sign = 1;
        if angle > Wide::PI / 2 {
            angle = Wide::PI - angle;
            cosine_sign = -1;
        } else if angle < -Wide::PI / 2 {
            angle = -Wide::PI - angle;
            cosine_sign = -1;
        }
        let (sin, cos) = cordic::rotate::<Wide>(angle);
        (sin, cos * cosine_sign)
    }

    /// Guarded acos in Q96, using compile-time-selected vectoring. The common
    /// Q62 square root retains endpoint resolution before working-state rounding.
    fn acos_wide_with<const WORK_BITS: u32>(self) -> i128 {
        const { Self::validate_cordic::<WORK_BITS>() };
        let raw = self.bits.widen();
        let x = raw.abs() << (62 - F);
        let y = integer_sqrt((1u128 << 124).wrapping_sub((x * x) as u128)) as i128;
        if WORK_BITS == 64 {
            let frac = Narrow::<F>::FRAC;
            let x = div(x, 1i128 << (62 - frac)) as i64;
            let y = div(y, 1i128 << (62 - frac)) as i64;
            let angle = cordic::vectoring::<Narrow<F>>(x, y);
            let angle = if raw < 0 {
                Narrow::<F>::PI - angle
            } else {
                angle
            };
            (angle as i128) << (96 - frac)
        } else {
            let angle = cordic::vectoring::<Wide>(x << 34, y << 34);
            if raw < 0 {
                Wide::PI - angle
            } else {
                angle
            }
        }
    }

    /// Integer-only sine, rounded to this format's quantum.
    pub fn checked_sin(self) -> Result<Self, MathError> {
        self.checked_sin_cos().map(|pair| pair.0)
    }
    /// Integer-only cosine, rounded to this format's quantum.
    pub fn checked_cos(self) -> Result<Self, MathError> {
        self.checked_sin_cos().map(|pair| pair.1)
    }
    /// Divides guarded sine/cosine before output quantization. A cosine rounded
    /// to zero in the destination format therefore does not create a fake pole.
    pub fn checked_tan(self) -> Result<Self, MathError> {
        self.checked_tan_with::<128>()
    }
    /// Const-selected tangent with destination range checking. Checks reject
    /// overflow and zero divisors; they do not recover lost guard precision or
    /// bound near-pole error. Width restrictions match [`Self::sin_cos_with`].
    pub fn checked_tan_with<const WORK_BITS: u32>(self) -> Result<Self, MathError> {
        let (sin, cos) = self.sin_cos_wide_with::<WORK_BITS>();
        // Both components fit Q96; first reduce to Q64 so scaling the quotient
        // cannot exceed i128 even for the largest supported fractional width.
        let sin = round_div(sin, 1i128 << 32)?;
        let cos = round_div(cos, 1i128 << 32)?;
        Self::raw(round_div(sin * Self::scale(), cos)?)
    }

    /// Arc cosine on [-1, 1]. Guarded `sqrt(1-x*x)` and CORDIC vectoring avoid
    /// the endpoint precision loss caused by squaring directly in this format.
    pub fn checked_acos(self) -> Result<Self, MathError> {
        self.checked_acos_with::<128>()
    }
    /// Const-selected arc cosine with domain and destination range validation.
    /// The experimental i64 precision policy matches [`Self::sin_cos_with`].
    pub fn checked_acos_with<const WORK_BITS: u32>(self) -> Result<Self, MathError> {
        const { Self::validate_cordic::<WORK_BITS>() };
        let raw = self.bits.widen();
        if raw.abs() > Self::scale() {
            return Err(MathError::Domain);
        }
        if raw == Self::scale() {
            return Ok(Self::ZERO);
        }
        if raw == -Self::scale() {
            return Self::from_q96(Wide::PI);
        }
        Self::from_q96(self.acos_wide_with::<WORK_BITS>())
    }
}

impl<R: ScalarFormat<F>, const F: u32> Scalar for Fixed<R, F> {
    fn is_finite(self) -> bool {
        true
    }
    fn checked_abs(self) -> Result<Self, MathError> {
        Self::checked_abs(self)
    }
    fn checked_rem(self, rhs: Self) -> Result<Self, MathError> {
        Self::checked_rem(self, rhs)
    }
    #[inline(always)]
    fn sum_products(terms: &[(Self, Self, bool)]) -> Self {
        Self::sum_products(terms)
    }
    fn two() -> Self {
        Self::wrapping_raw(2 * Self::scale())
    }
    fn min(a: Self, b: Self) -> Self {
        core::cmp::min(a, b)
    }
    fn max(a: Self, b: Self) -> Self {
        core::cmp::max(a, b)
    }
    fn tabs(self) -> Self {
        self.abs()
    }
    fn checked_add(self, rhs: Self) -> Result<Self, MathError> {
        Self::checked_add(self, rhs)
    }
    fn checked_sub(self, rhs: Self) -> Result<Self, MathError> {
        Self::checked_sub(self, rhs)
    }
    fn checked_mul(self, rhs: Self) -> Result<Self, MathError> {
        Self::checked_mul(self, rhs)
    }
    fn checked_div(self, rhs: Self) -> Result<Self, MathError> {
        Self::checked_div(self, rhs)
    }
    fn checked_neg(self) -> Result<Self, MathError> {
        Self::checked_neg(self)
    }
    #[inline(always)]
    fn checked_sum_products(terms: &[(Self, Self, bool)]) -> Result<Self, MathError> {
        Self::checked_sum_products(terms)
    }
}

impl<R: RealFormat<F>, const F: u32> RealScalar for Fixed<R, F> {
    fn reciprocal(self) -> Self::Reciprocal {
        super::FixedReciprocal::new(self)
    }
    fn mul_reciprocal(self, reciprocal: Self::Reciprocal) -> Self {
        reciprocal.apply(self)
    }
    #[inline(always)]
    fn norm(values: &[Self]) -> Self {
        let mut squared = 0u128;
        unroll_short_slice!(values, &value => {
            let raw = value.bits.widen();
            squared = squared.wrapping_add((raw*raw) as u128);
        });
        Self::wrapping_raw(rounded_sqrt(squared) as i128)
    }
    type Reciprocal = super::FixedReciprocal<R, F>;
    fn checked_reciprocal(self) -> Result<Self::Reciprocal, MathError> {
        super::FixedReciprocal::checked_new(self)
    }
    fn checked_mul_reciprocal(self, reciprocal: Self::Reciprocal) -> Result<Self, MathError> {
        reciprocal.checked_apply(self)
    }
    fn epsilon() -> Self {
        Self::wrapping_raw(4)
    }
    fn half() -> Self {
        Self::wrapping_raw(Self::scale() / 2)
    }
    fn quarter() -> Self {
        Self::wrapping_raw(Self::scale() / 4)
    }
    fn tsqrt(self) -> Self {
        self.sqrt()
    }
    fn tsin(self) -> Self {
        self.sin()
    }
    fn tcos(self) -> Self {
        self.cos()
    }
    fn ttan(self) -> Self {
        self.tan()
    }
    fn tacos(self) -> Self {
        self.acos()
    }
    fn checked_sqrt(self) -> Result<Self, MathError> {
        Self::checked_sqrt(self)
    }
    fn checked_sin(self) -> Result<Self, MathError> {
        Self::checked_sin(self)
    }
    fn checked_cos(self) -> Result<Self, MathError> {
        Self::checked_cos(self)
    }
    fn checked_tan(self) -> Result<Self, MathError> {
        Self::checked_tan(self)
    }
    fn checked_acos(self) -> Result<Self, MathError> {
        Self::checked_acos(self)
    }
    #[inline(always)]
    fn checked_norm(values: &[Self]) -> Result<Self, MathError> {
        let mut squared = 0u128;
        unroll_short_slice!(values, &value => {
            let raw = value.bits.widen();
            squared = squared
                .checked_add((raw * raw) as u128)
                .ok_or(MathError::Overflow)?;
        });
        Self::raw(rounded_sqrt(squared) as i128)
    }
    #[inline(always)]
    fn checked_norm_exceeds(values: &[Self], epsilon: Self) -> Result<bool, MathError> {
        let epsilon = epsilon.bits.widen();
        if epsilon < 0 {
            return Err(MathError::InvalidTolerance);
        }
        let threshold = (epsilon * epsilon) as u128;
        let mut squared = 0u128;
        unroll_short_slice!(values, &value => {
            let raw = value.bits.widen();
            // Exceeding a u128 sum already exceeds every representable epsilon.
            match squared.checked_add((raw * raw) as u128) {
                None => return Ok(true),
                Some(value) => squared = value,
            }
        });
        Ok(squared > threshold)
    }
}
