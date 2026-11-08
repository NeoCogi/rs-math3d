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

//! Guarded reusable fixed reciprocals for the default matrix inverse path.

use super::{
    arithmetic::{signed, signed_wrapping},
    Fixed, RealFormat,
};
use crate::MathError;

/// A prepared signed fixed divisor with 127 reciprocal fraction bits.
///
/// Preparing it performs one integer division. Applying it uses widened
/// multiplication and one remainder correction, preserving truncating
/// division without quantizing the reciprocal to the destination format.
#[derive(Clone, Copy, Debug)]
pub struct FixedReciprocal<R: RealFormat<F>, const F: u32> {
    /// Original divisor retains scale and sign without additional metadata.
    denominator: Fixed<R, F>,
    /// Floor(2^127 / abs(denominator_raw)), computed exactly once.
    multiplier: u128,
}

impl<R: RealFormat<F>, const F: u32> FixedReciprocal<R, F> {
    /// Prepares a nonzero represented divisor; both signed extrema are valid.
    pub(crate) fn checked_new(denominator: Fixed<R, F>) -> Result<Self, MathError> {
        let magnitude = denominator.to_bits().widen().unsigned_abs();
        if magnitude == 0 {
            return Err(MathError::DivisionByZero);
        }
        Ok(Self {
            denominator,
            multiplier: (1u128 << 127) / magnitude,
        })
    }

    /// Applies the reciprocal with exact truncation toward zero and range checks.
    pub(crate) fn checked_apply(self, numerator: Fixed<R, F>) -> Result<Fixed<R, F>, MathError> {
        let (quotient, negative) = self.quotient(numerator);
        Fixed::raw(signed(quotient, negative)?)
    }

    /// Prepares a divisor without checking it. A zero divisor panics.
    pub(crate) fn new(denominator: Fixed<R, F>) -> Self {
        Self {
            denominator,
            multiplier: (1u128 << 127) / denominator.to_bits().widen().unsigned_abs(),
        }
    }

    /// Applies a prepared reciprocal and wraps the final storage bits.
    pub(crate) fn apply(self, numerator: Fixed<R, F>) -> Fixed<R, F> {
        let (quotient, negative) = self.quotient(numerator);
        Fixed::wrapping_raw(signed_wrapping(quotient, negative))
    }

    /// Exact quotient magnitude shared by both public arithmetic contracts.
    fn quotient(self, numerator: Fixed<R, F>) -> (u128, bool) {
        let n = numerator.to_bits().widen();
        let d = self.denominator.to_bits().widen();
        let a = n.unsigned_abs();
        let b = d.unsigned_abs();
        // a has at most 64 magnitude bits. Two partial products therefore fit
        // u128 and together encode the complete 192-bit product without loss.
        let lower = a * (self.multiplier & u64::MAX as u128);
        let upper = a * (self.multiplier >> 64);
        let (low, carry) = lower.overflowing_add(upper << 64);
        let high = (upper >> 64) + carry as u128;
        let shift = 127 - F;
        let mut quotient = (low >> shift) | (high << (128 - shift));
        let scaled = a << F;
        let remainder = scaled - quotient * b;
        // The floor reciprocal underestimates by less than one quotient unit:
        // a * 2^F < 2^125 for every supported RealFormat.
        if remainder >= b {
            quotient += 1;
        }
        (quotient, (n < 0) != (d < 0))
    }
}
