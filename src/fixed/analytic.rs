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

//! Integer-only analytic functions. Rotation/vectoring use Q96 guard precision;
//! full-range input reduction uses Q64 to fit every signed storage magnitude.

use super::{
    arithmetic::{integer_sqrt, round_div},
    constants, Fixed, RealFormat, ScalarFormat,
};
use crate::{MathError, RealScalar, Scalar};

/// Rounds a nonnegative integer square root to the nearest integer. Midpoint
/// ties cannot occur because `(n + 1/2)^2` is never an integer.
fn rounded_sqrt(value: u128) -> u128 {
    let root = integer_sqrt(value);
    root + (value - root * root > root) as u128
}

impl<R: RealFormat<F>, const F: u32> Fixed<R, F> {
    /// Computes a correctly rounded fixed square root without a float backend.
    pub fn try_sqrt(self) -> Result<Self, MathError> {
        if self.bits.widen() < 0 {
            return Err(MathError::Domain);
        }
        let radicand = self.bits.widen() as u128 * Self::scale() as u128;
        Self::raw(rounded_sqrt(radicand) as i128)
    }

    /// Computes sine/cosine together, retaining Q96 precision until narrowing.
    /// Range reduction's error is bounded relative to this format's quantum.
    pub fn try_sin_cos(self) -> Result<(Self, Self), MathError> {
        let (sin, cos) = self.sin_cos_wide();
        Ok((Self::from_q96(sin)?, Self::from_q96(cos)?))
    }

    /// Converts a guarded Q96 analytic result with one ties-to-even rounding.
    fn from_q96(value: i128) -> Result<Self, MathError> {
        Self::raw(round_div(value, 1i128 << (96 - F))?)
    }

    /// Rotates the gain-corrected CORDIC seed into the reduced angle.
    fn sin_cos_wide(self) -> (i128, i128) {
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
        let mut angle = reduced << 32;
        let mut cosine_sign = 1;
        if angle > constants::PI / 2 {
            angle = constants::PI - angle;
            cosine_sign = -1;
        } else if angle < -constants::PI / 2 {
            angle = -constants::PI - angle;
            cosine_sign = -1;
        }
        let mut x = constants::GAIN;
        let mut y = 0;
        for (i, &step) in constants::ANGLES.iter().enumerate() {
            let direction = if angle >= 0 { 1 } else { -1 };
            let next_x = x - direction * (y >> i);
            y += direction * (x >> i);
            x = next_x;
            angle -= direction * step;
        }
        (y, x * cosine_sign)
    }

    /// Integer-only sine, rounded to this format's quantum.
    pub fn try_sin(self) -> Result<Self, MathError> {
        self.try_sin_cos().map(|pair| pair.0)
    }
    /// Integer-only cosine, rounded to this format's quantum.
    pub fn try_cos(self) -> Result<Self, MathError> {
        self.try_sin_cos().map(|pair| pair.1)
    }
    /// Divides guarded sine/cosine before output quantization. A cosine rounded
    /// to zero in the destination format therefore does not create a fake pole.
    pub fn try_tan(self) -> Result<Self, MathError> {
        let (sin, cos) = self.sin_cos_wide();
        // Both components fit Q96; first reduce to Q64 so scaling the quotient
        // cannot exceed i128 even for the largest supported fractional width.
        let sin = round_div(sin, 1i128 << 32)?;
        let cos = round_div(cos, 1i128 << 32)?;
        Self::raw(round_div(sin * Self::scale(), cos)?)
    }

    /// Arc cosine on [-1, 1]. Guarded `sqrt(1-x*x)` and CORDIC vectoring avoid
    /// the endpoint precision loss caused by squaring directly in this format.
    pub fn try_acos(self) -> Result<Self, MathError> {
        let raw = self.bits.widen();
        if raw.abs() > Self::scale() {
            return Err(MathError::Domain);
        }
        if raw == Self::scale() {
            return Ok(Self::ZERO);
        }
        if raw == -Self::scale() {
            return Self::from_q96(constants::PI);
        }
        let x = raw.abs() << (62 - F);
        let y = integer_sqrt((1u128 << 124) - (x * x) as u128) as i128;
        let mut x = x << 34;
        let mut y = y << 34;
        let mut angle = 0;
        for (i, &step) in constants::ANGLES.iter().enumerate() {
            if y == 0 {
                break;
            }
            let direction = if y > 0 { 1 } else { -1 };
            let next_x = x + direction * (y >> i);
            y -= direction * (x >> i);
            x = next_x;
            angle += direction * step;
        }
        Self::from_q96(if raw < 0 {
            constants::PI - angle
        } else {
            angle
        })
    }
}

impl<R: ScalarFormat<F>, const F: u32> Scalar for Fixed<R, F> {
    fn two() -> Self {
        Self::try_from_int(2).expect("scalar format represents two")
    }
    fn min(a: Self, b: Self) -> Self {
        core::cmp::min(a, b)
    }
    fn max(a: Self, b: Self) -> Self {
        core::cmp::max(a, b)
    }
    fn tabs(self) -> Self {
        self.try_abs().expect("fixed absolute value overflow")
    }
    fn try_add(self, rhs: Self) -> Result<Self, MathError> {
        Self::try_add(self, rhs)
    }
    fn try_sub(self, rhs: Self) -> Result<Self, MathError> {
        Self::try_sub(self, rhs)
    }
    fn try_mul(self, rhs: Self) -> Result<Self, MathError> {
        Self::try_mul(self, rhs)
    }
    fn try_div(self, rhs: Self) -> Result<Self, MathError> {
        Self::try_div(self, rhs)
    }
    fn try_neg(self) -> Result<Self, MathError> {
        Self::try_neg(self)
    }
    fn try_sum_products(terms: &[(Self, Self, bool)]) -> Result<Self, MathError> {
        Self::sum_products(terms)
    }
}

impl<R: RealFormat<F>, const F: u32> RealScalar for Fixed<R, F> {
    type Reciprocal = super::FixedReciprocal<R, F>;
    fn try_reciprocal(self) -> Result<Self::Reciprocal, MathError> {
        super::FixedReciprocal::new(self)
    }
    fn try_mul_reciprocal(self, reciprocal: Self::Reciprocal) -> Result<Self, MathError> {
        reciprocal.apply(self)
    }
    fn epsilon() -> Self {
        Self::raw(4).expect("real format represents four quanta")
    }
    fn half() -> Self {
        Self::raw(Self::scale() / 2).expect("real format represents half")
    }
    fn quarter() -> Self {
        Self::raw(Self::scale() / 4).expect("real format represents quarter")
    }
    fn is_finite(self) -> bool {
        true
    }
    fn try_abs(self) -> Result<Self, MathError> {
        Self::try_abs(self)
    }
    fn tsqrt(self) -> Self {
        self.try_sqrt().expect("fixed square root failed")
    }
    fn tsin(self) -> Self {
        self.try_sin().expect("fixed sine failed")
    }
    fn tcos(self) -> Self {
        self.try_cos().expect("fixed cosine failed")
    }
    fn ttan(self) -> Self {
        self.try_tan().expect("fixed tangent failed")
    }
    fn tacos(self) -> Self {
        self.try_acos().expect("fixed arc cosine failed")
    }
    fn try_sqrt(self) -> Result<Self, MathError> {
        Self::try_sqrt(self)
    }
    fn try_sin(self) -> Result<Self, MathError> {
        Self::try_sin(self)
    }
    fn try_cos(self) -> Result<Self, MathError> {
        Self::try_cos(self)
    }
    fn try_tan(self) -> Result<Self, MathError> {
        Self::try_tan(self)
    }
    fn try_acos(self) -> Result<Self, MathError> {
        Self::try_acos(self)
    }
    fn try_norm(values: &[Self]) -> Result<Self, MathError> {
        let mut squared = 0u128;
        for &value in values {
            let raw = value.bits.widen();
            squared = squared
                .checked_add((raw * raw) as u128)
                .ok_or(MathError::Overflow)?;
        }
        Self::raw(rounded_sqrt(squared) as i128)
    }
    fn norm_exceeds(values: &[Self], epsilon: Self) -> Result<bool, MathError> {
        let epsilon = epsilon.bits.widen();
        if epsilon < 0 {
            return Err(MathError::InvalidTolerance);
        }
        let threshold = (epsilon * epsilon) as u128;
        let mut squared = 0u128;
        for &value in values {
            let raw = value.bits.widen();
            // Exceeding a u128 sum already exceeds every representable epsilon.
            match squared.checked_add((raw * raw) as u128) {
                None => return Ok(true),
                Some(value) => squared = value,
            }
        }
        Ok(squared > threshold)
    }
}
