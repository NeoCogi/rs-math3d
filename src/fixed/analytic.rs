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

/// Guarded CORDIC working state, owned by each expanded chunk and passed to
/// the next. All fields use Q96; rotation starts with reciprocal gain and
/// vectoring starts with guarded first-quadrant coordinates.
struct CordicState {
    /// Current horizontal coordinate.
    x: i128,
    /// Current vertical coordinate; exact zero terminates vectoring.
    y: i128,
    /// Residual rotation angle or accumulated vectoring angle.
    angle: i128,
}

// Seven literal chunk starts cover exactly the generated 97 entries.
const _: [(); 97] = [(); constants::ANGLES.len()];

/// Expands at most sixteen literal-index rotation steps. Non-inlining keeps
/// optimizer work bounded and shares code across all destination formats;
/// chunk calls themselves are also explicitly expanded, with no control loop.
#[inline(never)]
fn cordic_rotate_chunk<const START: usize>(mut state: CordicState) -> CordicState {
    unroll_indices!(offset in [0,1,2,3,4,5,6,7,8,9,10,11,12,13,14,15] {
        let i = START+offset;
        if i < constants::ANGLES.len() {
            let direction = if state.angle >= 0 {1} else {-1};
            let next_x = state.x-direction*(state.y >> i);
            state.y += direction*(state.x >> i);
            state.x = next_x;
            state.angle -= direction*constants::ANGLES[i];
        }
    });
    state
}

/// Expands a vectoring chunk while preserving the exact residual-zero exit.
/// `Break` is terminal and bypasses every remaining chunk; `Continue` owns the
/// next working state. Literal out-of-table steps in the last chunk disappear
/// at compile time. Non-inlining avoids optimizing a single huge branch graph.
#[inline(never)]
fn cordic_vectoring_chunk<const START: usize>(
    mut state: CordicState,
) -> core::ops::ControlFlow<i128, CordicState> {
    unroll_indices!(offset in [0,1,2,3,4,5,6,7,8,9,10,11,12,13,14,15] {
        let i = START+offset;
        if i < constants::ANGLES.len() {
            if state.y == 0 { return core::ops::ControlFlow::Break(state.angle); }
            let direction = if state.y > 0 {1} else {-1};
            let next_x = state.x+direction*(state.y >> i);
            state.y -= direction*(state.x >> i);
            state.x = next_x;
            state.angle += direction*constants::ANGLES[i];
        }
    });
    core::ops::ControlFlow::Continue(state)
}

/// Runs all 97 rotation steps through seven explicit chunk calls. Q96 work is
/// independent of storage width/fractional precision and shared across formats.
fn cordic_rotate(angle: i128) -> (i128, i128) {
    let state = CordicState {
        x: constants::GAIN,
        y: 0,
        angle,
    };
    let state = cordic_rotate_chunk::<0>(state);
    let state = cordic_rotate_chunk::<16>(state);
    let state = cordic_rotate_chunk::<32>(state);
    let state = cordic_rotate_chunk::<48>(state);
    let state = cordic_rotate_chunk::<64>(state);
    let state = cordic_rotate_chunk::<80>(state);
    let state = cordic_rotate_chunk::<96>(state);
    (state.y, state.x)
}

/// Runs the vectoring schedule with a terminal check at each chunk boundary.
/// Exact-zero residuals return the same accumulated angle as the iterative
/// algorithm without executing arithmetic or calls for later chunks.
fn cordic_vectoring(x: i128, y: i128) -> i128 {
    let mut state = CordicState { x, y, angle: 0 };
    /// Propagates a terminal angle without hiding it in a boolean flag.
    macro_rules! chunk {
        ($start:literal) => {
            match cordic_vectoring_chunk::<$start>(state) {
                core::ops::ControlFlow::Continue(next) => state = next,
                core::ops::ControlFlow::Break(angle) => return angle,
            }
        };
    }
    chunk!(0);
    chunk!(16);
    chunk!(32);
    chunk!(48);
    chunk!(64);
    chunk!(80);
    chunk!(96);
    state.angle
}

use super::arithmetic::div;

impl<R: RealFormat<F>, const F: u32> Fixed<R, F> {
    /// Integer square root without domain validation. Supply a nonnegative input.
    pub fn sqrt(self) -> Self {
        let radicand = (self.bits.widen() as u128).wrapping_mul(Self::scale() as u128);
        Self::wrapping_raw(rounded_sqrt(radicand) as i128)
    }
    /// Computes sine and cosine without destination range validation.
    pub fn sin_cos(self) -> (Self, Self) {
        let (sin, cos) = self.sin_cos_wide();
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
        let (sin, cos) = self.sin_cos_wide();
        Self::wrapping_raw(div(
            div(sin, 1i128 << 32) * Self::scale(),
            div(cos, 1i128 << 32),
        ))
    }
    /// Arc cosine without domain validation. Supply a value in [-1, 1].
    pub fn acos(self) -> Self {
        let raw = self.bits.widen();
        let x = raw.abs() << (62 - F);
        let y = integer_sqrt((1u128 << 124).wrapping_sub((x * x) as u128)) as i128;
        let angle = cordic_vectoring(x << 34, y << 34);
        Self::wrapping_raw(div(
            if raw < 0 {
                constants::PI - angle
            } else {
                angle
            },
            1i128 << (96 - F),
        ))
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
        let (sin, cos) = cordic_rotate(angle);
        (sin, cos * cosine_sign)
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
        let (sin, cos) = self.sin_cos_wide();
        // Both components fit Q96; first reduce to Q64 so scaling the quotient
        // cannot exceed i128 even for the largest supported fractional width.
        let sin = round_div(sin, 1i128 << 32)?;
        let cos = round_div(cos, 1i128 << 32)?;
        Self::raw(round_div(sin * Self::scale(), cos)?)
    }

    /// Arc cosine on [-1, 1]. Guarded `sqrt(1-x*x)` and CORDIC vectoring avoid
    /// the endpoint precision loss caused by squaring directly in this format.
    pub fn checked_acos(self) -> Result<Self, MathError> {
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
        let angle = cordic_vectoring(x << 34, y << 34);
        Self::from_q96(if raw < 0 {
            constants::PI - angle
        } else {
            angle
        })
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

#[cfg(test)]
mod unrolling_tests {
    //! Compare guarded internal states, not only rounded destination values,
    //! so even an omitted final table step is observable in these regressions.
    use super::*;

    /// Original iterative rotation schedule retained as an independent oracle.
    fn reference_rotate(mut angle: i128) -> (i128, i128) {
        let mut x = constants::GAIN;
        let mut y = 0;
        for (i, &step) in constants::ANGLES.iter().enumerate() {
            let direction = if angle >= 0 { 1 } else { -1 };
            let next_x = x - direction * (y >> i);
            y += direction * (x >> i);
            x = next_x;
            angle -= direction * step;
        }
        (y, x)
    }

    /// Original vectoring schedule preserves the exact residual-zero exit.
    fn reference_vectoring(mut x: i128, mut y: i128) -> i128 {
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
        angle
    }

    #[test]
    fn expanded_rotation_preserves_every_guard_bit() {
        for raw in i16::MIN..=i16::MAX {
            let angle = raw as i128 * constants::PI / (2 * (1i128 << 15));
            assert_eq!(cordic_rotate(angle), reference_rotate(angle));
        }
        for angle in [-constants::PI / 2, -1, 0, 1, constants::PI / 2] {
            assert_eq!(cordic_rotate(angle), reference_rotate(angle));
        }
    }

    #[test]
    fn expanded_vectoring_preserves_guard_bits_and_early_exits() {
        for raw in -256i128..=256 {
            let x = raw.abs() << 54;
            let y = integer_sqrt((1u128 << 124) - (x * x) as u128) as i128;
            assert_eq!(
                cordic_vectoring(x << 34, y << 34),
                reference_vectoring(x << 34, y << 34)
            );
        }
        for (x, y) in [
            (0, 0),
            (1i128 << 96, 0),
            (1i128 << 95, 1i128 << 95),
            (0, 1i128 << 96),
            (1i128 << 96, 1),
            (1, 1i128 << 96),
        ] {
            assert_eq!(cordic_vectoring(x, y), reference_vectoring(x, y));
        }
    }
}
