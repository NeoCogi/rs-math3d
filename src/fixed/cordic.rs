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

//! One integer CORDIC engine with sealed, compile-time precision backends.
//!
//! Backends own the word type and a coherent table/gain/scale policy. The engine
//! owns rotation, vectoring and expanded chunk sequencing. Reduced angles and
//! unit-circle coordinates must fit the selected backend's documented headroom.

use super::constants;
use core::ops::{Add, AddAssign, Mul, Neg, Shr, Sub, SubAssign};

/// Prevents unrelated numeric implementations from weakening engine invariants.
mod sealed {
    /// Marker implemented only by this module's two precision policies.
    pub trait Sealed {}
}

/// A coherent numeric policy for the shared CORDIC engine. Implementations must
/// provide enough headroom for pi and gain-expanded first-quadrant unit inputs,
/// a reciprocal gain matching the selected schedule, and at least STEPS angles.
/// The sealed implementations use signed primitive words, so each monomorphized
/// step performs arithmetic directly in i64 or i128 without conversion/dispatch.
pub(super) trait Backend: sealed::Sealed {
    /// Signed working word; all state fields and step operations use this type.
    type Word: Copy
        + 'static
        + PartialOrd
        + Add<Output = Self::Word>
        + Sub<Output = Self::Word>
        + Mul<Output = Self::Word>
        + Neg<Output = Self::Word>
        + Shr<usize, Output = Self::Word>
        + AddAssign
        + SubAssign;
    /// Exact working zero, for initialization and residual termination.
    const ZERO: Self::Word;
    /// An integer step direction, not the scaled numeric value one.
    const ONE: Self::Word;
    /// Fractional working bits, including guards beyond the destination scale.
    const FRAC: u32;
    /// One final step beyond the fractional width resolves its last bit.
    const STEPS: usize = Self::FRAC as usize + 1;
    /// Pi encoded in the working scale, used by analytic input folding.
    const PI: Self::Word;
    /// Reciprocal rotation gain encoded in the working scale.
    const GAIN: Self::Word;
    /// atan(2^-i) in the working scale. Unused tail entries are never evaluated.
    const ANGLES: &'static [Self::Word];
}

/// Default Q96/i128 policy, independent of destination storage and precision.
/// A single concrete backend keeps its chunks shared across all fixed formats.
pub(super) struct Wide;

impl sealed::Sealed for Wide {}

impl Backend for Wide {
    type Word = i128;
    const ZERO: i128 = 0;
    const ONE: i128 = 1;
    const FRAC: u32 = 96;
    const PI: i128 = constants::PI;
    const GAIN: i128 = constants::GAIN;
    const ANGLES: &'static [i128] = &constants::ANGLES;
}

/// Experimental i64 policy. For supported destination F<=48, clamp(F+16,32,60)
/// retains at least twelve guard bits and three integer magnitude headroom bits.
/// Analytic APIs validate the destination restriction before entering the engine.
pub(super) struct Narrow<const F: u32>;

impl<const F: u32> sealed::Sealed for Narrow<F> {}

impl<const F: u32> Backend for Narrow<F> {
    type Word = i64;
    const ZERO: i64 = 0;
    const ONE: i64 = 1;
    const FRAC: u32 = if F < 16 {
        32
    } else if F > 44 {
        60
    } else {
        F + 16
    };
    const PI: i64 = rescale(constants::PI_Q60, 60 - Self::FRAC);
    // The omitted tail for the shortest (33-step) schedule changes gain by less
    // than 2^-64, far below its Q32 working quantum. Preserve the existing seed.
    const GAIN: i64 = rescale(constants::GAIN_Q60, 60 - Self::FRAC);
    const ANGLES: &'static [i64] = &{
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

/// Owned engine state, passed between chunks without erasing the backend word.
/// Rotation consumes a residual angle; vectoring accumulates an angle and can
/// terminate on an exact zero vertical residual.
struct State<B: Backend> {
    /// Horizontal coordinate in the selected working scale.
    x: B::Word,
    /// Vertical coordinate in that scale; zero terminates vectoring.
    y: B::Word,
    /// Residual or accumulated radian angle in that scale.
    angle: B::Word,
}

/// Expands up to sixteen rotation steps. Literal starts, backend constants and
/// monomorphized primitive operators remove unused steps and width dispatch.
/// Non-inlining bounds optimizer work and shares each chunk across callers.
#[inline(never)]
fn cordic_rotate_chunk<B: Backend, const START: usize>(mut state: State<B>) -> State<B> {
    unroll_indices!(offset in [0,1,2,3,4,5,6,7,8,9,10,11,12,13,14,15] {
        let i = START + offset;
        if i < B::STEPS {
            let direction = if state.angle >= B::ZERO { B::ONE } else { -B::ONE };
            let next_x = state.x - direction * (state.y >> i);
            state.y += direction * (state.x >> i);
            state.x = next_x;
            state.angle -= direction * B::ANGLES[i];
        }
    });
    state
}

/// Expanded vectoring steps with an explicit terminal state. Break owns the
/// final angle; Continue owns the state for the next statically selected chunk.
#[inline(never)]
fn cordic_vectoring_chunk<B: Backend, const START: usize>(
    mut state: State<B>,
) -> core::ops::ControlFlow<B::Word, State<B>> {
    unroll_indices!(offset in [0,1,2,3,4,5,6,7,8,9,10,11,12,13,14,15] {
        let i = START + offset;
        if i < B::STEPS {
            if state.y == B::ZERO { return core::ops::ControlFlow::Break(state.angle); }
            let direction = if state.y > B::ZERO { B::ONE } else { -B::ONE };
            let next_x = state.x + direction * (state.y >> i);
            state.y -= direction * (state.x >> i);
            state.x = next_x;
            state.angle += direction * B::ANGLES[i];
        }
    });
    core::ops::ControlFlow::Continue(state)
}

/// Rotates the gain-corrected seed through a reduced angle in [-pi/2,pi/2],
/// returning (sine, cosine) in the backend scale. The same schedule covers both
/// backends: i64 selects three/four chunks and Q96 selects all seven.
pub(super) fn rotate<B: Backend>(angle: B::Word) -> (B::Word, B::Word) {
    const {
        assert!(B::STEPS <= 97 && B::STEPS <= B::ANGLES.len());
    }
    let state = State::<B> {
        x: B::GAIN,
        y: B::ZERO,
        angle,
    };
    let state = cordic_rotate_chunk::<B, 0>(state);
    let state = cordic_rotate_chunk::<B, 16>(state);
    let state = cordic_rotate_chunk::<B, 32>(state);
    let state = if B::STEPS > 48 {
        cordic_rotate_chunk::<B, 48>(state)
    } else {
        state
    };
    let state = if B::STEPS > 64 {
        cordic_rotate_chunk::<B, 64>(state)
    } else {
        state
    };
    let state = if B::STEPS > 80 {
        cordic_rotate_chunk::<B, 80>(state)
    } else {
        state
    };
    let state = if B::STEPS > 96 {
        cordic_rotate_chunk::<B, 96>(state)
    } else {
        state
    };
    (state.y, state.x)
}

/// Returns a first-quadrant angle for unit-circle coordinates. The statically
/// expanded schedule stops immediately on a terminal residual, including at
/// chunk boundaries; no later chunk executes after Break.
pub(super) fn vectoring<B: Backend>(x: B::Word, y: B::Word) -> B::Word {
    const {
        assert!(B::STEPS <= 97 && B::STEPS <= B::ANGLES.len());
    }
    let mut state = State::<B> {
        x,
        y,
        angle: B::ZERO,
    };
    macro_rules! chunk {
        ($start:literal) => {
            match cordic_vectoring_chunk::<B, $start>(state) {
                core::ops::ControlFlow::Continue(next) => state = next,
                core::ops::ControlFlow::Break(angle) => return angle,
            }
        };
    }
    chunk!(0);
    chunk!(16);
    chunk!(32);
    if B::STEPS > 48 {
        chunk!(48);
    }
    if B::STEPS > 64 {
        chunk!(64);
    }
    if B::STEPS > 80 {
        chunk!(80);
    }
    if B::STEPS > 96 {
        chunk!(96);
    }
    state.angle
}

#[cfg(test)]
mod tests;
