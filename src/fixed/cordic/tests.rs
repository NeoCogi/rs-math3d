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

//! Guard-bit equivalence against the original typed iterative schedules.

use super::*;
use crate::fixed::arithmetic::integer_sqrt;
use crate::fixed::arithmetic::round_div_wrapping as div;

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
        assert_eq!(rotate::<Wide>(angle), reference_rotate(angle));
    }
    for angle in [-constants::PI / 2, -1, 0, 1, constants::PI / 2] {
        assert_eq!(rotate::<Wide>(angle), reference_rotate(angle));
    }
}

#[test]
fn expanded_vectoring_preserves_guard_bits_and_early_exits() {
    for raw in -256i128..=256 {
        let x = raw.abs() << 54;
        let y = integer_sqrt((1u128 << 124) - (x * x) as u128) as i128;
        assert_eq!(
            vectoring::<Wide>(x << 34, y << 34),
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
        assert_eq!(vectoring::<Wide>(x, y), reference_vectoring(x, y));
    }
}

/// Original i64 rotation expressed as a typed iterative oracle. The table and
/// seed come from independently rescaled generated constants, not the backend.
fn reference_rotate64(mut angle: i64, gain: i64, angles: &[i64]) -> (i64, i64) {
    let mut x = gain;
    let mut y = 0;
    for (i, &step) in angles.iter().enumerate() {
        let direction = if angle >= 0 { 1 } else { -1 };
        let next_x = x - direction * (y >> i);
        y += direction * (x >> i);
        x = next_x;
        angle -= direction * step;
    }
    (y, x)
}

/// Original i64 vectoring oracle retains its exact residual-zero exit.
fn reference_vectoring64(mut x: i64, mut y: i64, angles: &[i64]) -> i64 {
    let mut angle = 0;
    for (i, &step) in angles.iter().enumerate() {
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

/// Checks policy constants and every output guard bit, including partial last
/// chunks, all distinct working widths and both clamped precision intervals.
fn narrow_equivalence<const F: u32>() {
    let frac = (F + 16).clamp(32, 60);
    let steps = frac as usize + 1;
    let scale = 1i128 << (60 - frac);
    let gain = div(constants::GAIN_Q60 as i128, scale) as i64;
    let pi = div(constants::PI_Q60 as i128, scale) as i64;
    let angles: [i64; 61] =
        core::array::from_fn(|i| div(constants::ANGLES_Q60[i] as i128, scale) as i64);
    assert_eq!(Narrow::<F>::FRAC, frac);
    assert_eq!(Narrow::<F>::STEPS, steps);
    assert_eq!(Narrow::<F>::PI, pi);
    assert_eq!(Narrow::<F>::GAIN, gain);
    assert_eq!(&Narrow::<F>::ANGLES[..steps], &angles[..steps]);
    let angles = &angles[..steps];
    for raw in -1024i128..=1024 {
        let angle = (raw * pi as i128 / 2048) as i64;
        assert_eq!(
            rotate::<Narrow<F>>(angle),
            reference_rotate64(angle, gain, angles),
            "F={} raw={}",
            F,
            raw
        );
    }
    let one = 1i64 << frac;
    for raw in 0..=256 {
        let x = raw * (one >> 8);
        let y = integer_sqrt((one as u128).pow(2) - (x as u128).pow(2)) as i64;
        assert_eq!(
            vectoring::<Narrow<F>>(x, y),
            reference_vectoring64(x, y, angles)
        );
    }
    for (x, y) in [
        (0, 0),
        (one, 0),
        (one / 2, one / 2),
        (0, one),
        (one, 1),
        (1, one),
    ] {
        assert_eq!(
            vectoring::<Narrow<F>>(x, y),
            reference_vectoring64(x, y, angles)
        );
    }
}

/// Every supported destination scale must preserve the original i64 policy,
/// rotation state and vectoring angle before any destination quantization.
#[test]
fn generic_i64_engine_preserves_all_supported_guard_precisions() {
    macro_rules! check {
        ($($f:literal),*) => { $(narrow_equivalence::<$f>();)* };
    }
    check!(
        2, 3, 4, 5, 6, 7, 8, 9, 10, 11, 12, 13, 14, 15, 16, 17, 18, 19, 20, 21, 22, 23, 24, 25, 26,
        27, 28, 29, 30, 31, 32, 33, 34, 35, 36, 37, 38, 39, 40, 41, 42, 43, 44, 45, 46, 47, 48
    );
}
