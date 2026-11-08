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

//! Experimental i64 CORDIC with format-derived const precision and schedules.
//!
//! Working precision is clamp(F + 16, 32, 60), giving at least twelve guard
//! bits for F <= 48. Values have at least three signed integer headroom bits:
//! reduced angles fit pi, and vectoring starts with first-quadrant unit inputs.

use super::constants;

/// A const-only policy shared by conversion, tables and expanded step selection.
pub(super) struct Precision<const F: u32>;

impl<const F: u32> Precision<F> {
    /// Fractional working bits, including guard bits beyond the destination.
    pub(super) const FRAC: u32 = if F < 16 {
        32
    } else if F > 44 {
        60
    } else {
        F + 16
    };
    /// One final step beyond the fractional working width resolves its last bit.
    const STEPS: usize = Self::FRAC as usize + 1;
    /// Pi rounded to the working scale at compile time.
    pub(super) const PI: i64 = rescale(constants::PI_Q60, 60 - Self::FRAC);
    /// Reciprocal gain rounded once for the selected working scale. The omitted
    /// tail of any shorter schedule starts after step 32 and changes gain by
    /// less than 2^-64, far below the smallest Q32 working quantum.
    const GAIN: i64 = rescale(constants::GAIN_Q60, 60 - Self::FRAC);
    /// The full fixed-size backing table; unused entries are removed with their
    /// const-false steps. This avoids unstable generic array-length expressions.
    const ANGLES: [i64; 61] = {
        let mut angles = [0; 61];
        let mut i = 0;
        while i < Self::STEPS {
            angles[i] = rescale(constants::ANGLES_Q60[i], 60 - Self::FRAC);
            i += 1;
        }
        angles
    };
}

/// Const ties-to-even downscaling of a positive Q60 constant. The minimum
/// working width is Q32, so the shift is at most 28 and additions fit i64.
const fn rescale(value: i64, shift: u32) -> i64 {
    if shift == 0 {
        return value;
    }
    let quotient = value >> shift;
    let remainder = value & ((1i64 << shift) - 1);
    let half = 1i64 << (shift - 1);
    quotient + (remainder > half || (remainder == half && quotient & 1 != 0)) as i64
}

/// Owned working state. Rotation owns a residual angle; vectoring accumulates
/// an angle and terminates when its vertical residual becomes exactly zero.
struct State {
    /// Horizontal coordinate in the selected working scale.
    x: i64,
    /// Vertical coordinate in the same scale.
    y: i64,
    /// Residual or accumulated radian angle in that scale.
    angle: i64,
}

/// Up to sixteen expanded steps, selected entirely by const expressions.
/// Separate chunks bound optimizer work; each selected format has its own
/// schedule rather than executing runtime shifts or table index loops.
#[inline(never)]
fn cordic_rotate64_chunk<const F: u32, const START: usize>(mut state: State) -> State {
    unroll_indices!(offset in [0,1,2,3,4,5,6,7,8,9,10,11,12,13,14,15] {
        let i = START + offset;
        if i < Precision::<F>::STEPS {
            let direction = if state.angle >= 0 { 1 } else { -1 };
            let next_x = state.x - direction * (state.y >> i);
            state.y += direction * (state.x >> i);
            state.x = next_x;
            state.angle -= direction * Precision::<F>::ANGLES[i];
        }
    });
    state
}

/// Expanded vectoring with an explicit terminal residual-zero state.
#[inline(never)]
fn cordic_vectoring64_chunk<const F: u32, const START: usize>(
    mut state: State,
) -> core::ops::ControlFlow<i64, State> {
    unroll_indices!(offset in [0,1,2,3,4,5,6,7,8,9,10,11,12,13,14,15] {
        let i = START + offset;
        if i < Precision::<F>::STEPS {
            if state.y == 0 { return core::ops::ControlFlow::Break(state.angle); }
            let direction = if state.y > 0 { 1 } else { -1 };
            let next_x = state.x + direction * (state.y >> i);
            state.y -= direction * (state.x >> i);
            state.x = next_x;
            state.angle += direction * Precision::<F>::ANGLES[i];
        }
    });
    core::ops::ControlFlow::Continue(state)
}

/// Rotates a gain-corrected seed through a reduced angle in [-pi/2, pi/2].
/// Returns (sine, cosine) in the format-derived working scale.
pub(super) fn rotate<const F: u32>(angle: i64) -> (i64, i64) {
    let state = State {
        x: Precision::<F>::GAIN,
        y: 0,
        angle,
    };
    let state = cordic_rotate64_chunk::<F, 0>(state);
    let state = cordic_rotate64_chunk::<F, 16>(state);
    let state = cordic_rotate64_chunk::<F, 32>(state);
    let state = if Precision::<F>::STEPS > 48 {
        cordic_rotate64_chunk::<F, 48>(state)
    } else {
        state
    };
    (state.y, state.x)
}

/// Returns a first-quadrant angle for unit-circle coordinates. Gain growth is
/// below two, so every selected working scale retains sufficient headroom.
pub(super) fn vectoring<const F: u32>(x: i64, y: i64) -> i64 {
    let mut state = State { x, y, angle: 0 };
    macro_rules! chunk {
        ($start:literal) => {
            match cordic_vectoring64_chunk::<F, $start>(state) {
                core::ops::ControlFlow::Continue(next) => state = next,
                core::ops::ControlFlow::Break(angle) => return angle,
            }
        };
    }
    chunk!(0);
    chunk!(16);
    chunk!(32);
    if Precision::<F>::STEPS > 48 {
        chunk!(48);
    }
    state.angle
}
