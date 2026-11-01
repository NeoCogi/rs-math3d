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

//! Exact integer rounding and accumulation used by fixed arithmetic.

use crate::MathError;

/// Rounds a widened quotient without checking domain or destination range.
/// Callers supply a nonzero divisor and narrow the low bits at their boundary.
#[inline]
pub(super) fn div(n: i128, d: i128) -> i128 {
    let a = n.unsigned_abs();
    let b = d.unsigned_abs();
    let q = a / b;
    let r = a % b;
    let rounded = q.wrapping_add((r > b - r || (r == b - r && q & 1 != 0)) as u128);
    signed_wrapping(rounded, (n < 0) != (d < 0))
}

/// Restores a sign modulo 2^128. Narrowing later preserves the low storage bits.
#[inline]
pub(super) fn signed_wrapping(magnitude: u128, negative: bool) -> i128 {
    if negative {
        (magnitude as i128).wrapping_neg()
    } else {
        magnitude as i128
    }
}

/// Divides signed integers, rounding to nearest with ties to an even quotient.
/// Magnitudes avoid negating a signed minimum; the caller checks output range.
pub(super) fn round_div(n: i128, d: i128) -> Result<i128, MathError> {
    if d == 0 {
        return Err(MathError::DivisionByZero);
    }
    let negative = (n < 0) != (d < 0);
    let a = n.unsigned_abs();
    let b = d.unsigned_abs();
    let q = a / b;
    let r = a % b;
    let increment = r > b - r || (r == b - r && q & 1 != 0);
    signed(
        q.checked_add(increment as u128)
            .ok_or(MathError::Overflow)?,
        negative,
    )
}

/// Reassembles a signed magnitude, checking both asymmetric signed limits.
#[inline]
pub(super) fn signed(magnitude: u128, negative: bool) -> Result<i128, MathError> {
    if negative && magnitude == 1u128 << 127 {
        return Ok(i128::MIN);
    }
    let value = i128::try_from(magnitude).map_err(|_| MathError::Overflow)?;
    Ok(if negative { -value } else { value })
}

/// Rounds an unsigned integer divided by a power of two, including huge shifts.
#[inline]
pub(super) fn round_shift(value: u128, shift: u32) -> u128 {
    if shift == 0 {
        return value;
    }
    if shift > 128 {
        return 0;
    }
    if shift == 128 {
        return (value > (1u128 << 127)) as u128;
    }
    let q = value >> shift;
    let r = value & ((1u128 << shift) - 1);
    let half = 1u128 << (shift - 1);
    q + (r > half || (r == half && q & 1 != 0)) as u128
}

/// A signed-magnitude 256-bit accumulator for sums of stored integer products.
///
/// Keeping sign separate permits exact cancellation even when a partial sum
/// exceeds `i128`. Four `i64` products need more than a signed 128-bit sum.
#[derive(Default)]
pub(super) struct Accumulator {
    /// Sign of the current magnitude; zero is canonicalized to positive.
    negative: bool,
    /// Most significant 128 magnitude bits.
    high: u128,
    /// Least significant 128 magnitude bits.
    low: u128,
}

impl Accumulator {
    /// Adds or subtracts one widened product without quantizing it.
    pub(super) fn add(&mut self, value: i128, subtract: bool) -> Result<(), MathError> {
        let magnitude = value.unsigned_abs();
        let negative = (value < 0) != subtract;
        if self.negative == negative {
            let (low, carry) = self.low.overflowing_add(magnitude);
            self.high = self
                .high
                .checked_add(carry as u128)
                .ok_or(MathError::Overflow)?;
            self.low = low;
        } else if self.high != 0 || self.low >= magnitude {
            let (low, borrow) = self.low.overflowing_sub(magnitude);
            self.high -= borrow as u128;
            self.low = low;
        } else {
            self.low = magnitude - self.low;
            self.negative = negative;
        }
        if self.high == 0 && self.low == 0 {
            self.negative = false;
        }
        Ok(())
    }

    /// Quantizes once at the scalar boundary. A surviving high limb cannot fit
    /// any supported stored result after a shift of at most 63 fractional bits.
    pub(super) fn finish(self, frac: u32) -> Result<i128, MathError> {
        if self.high != 0 {
            return Err(MathError::Overflow);
        }
        signed(round_shift(self.low, frac), self.negative)
    }
}

/// Computes floor(sqrt(value)) without multiplication overflow or allocation.
pub(super) fn integer_sqrt(value: u128) -> u128 {
    if value < 2 {
        return value;
    }
    let mut x = 1u128 << ((128 - value.leading_zeros()).div_ceil(2));
    loop {
        let next = (x + value / x) / 2;
        if next >= x {
            return x;
        }
        x = next;
    }
}

use core::convert::TryFrom;
