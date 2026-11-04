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

//! Exhaustive small-format reference checks and wide conversion boundaries.

use super::*;
use crate::{RealScalar, RealVector, Scalar, Vector, Vector3};
use std::{format, panic::catch_unwind};

/// Converts the independent floating reference, whose tested dyadic operands
/// and products are exact in `f64`, into the expected checked small-format raw.
fn expected(value: f64) -> Result<i8, MathError> {
    let raw = value.round_ties_even();
    if raw < i8::MIN as f64 || raw > i8::MAX as f64 {
        Err(MathError::Overflow)
    } else {
        Ok(raw as i8)
    }
}

#[test]
fn exhaustive_i8_arithmetic() {
    type F = Fixed<i8, 4>;
    for a in i8::MIN..=i8::MAX {
        for b in i8::MIN..=i8::MAX {
            let x = F::from_bits(a);
            let y = F::from_bits(b);
            assert_eq!((x + y).to_bits(), a.wrapping_add(b));
            assert_eq!((x - y).to_bits(), a.wrapping_sub(b));
            assert_eq!((-x).to_bits(), a.wrapping_neg());
            assert_eq!(
                (x * y).to_bits(),
                (a as f64 * b as f64 / 16.0).round_ties_even() as i16 as i8
            );
            if b != 0 {
                assert_eq!(
                    (x / y).to_bits(),
                    (a as f64 * 16.0 / b as f64).round_ties_even() as i16 as i8
                );
                assert_eq!((x % y).to_bits(), (a as i16 % b as i16) as i8);
            }
            assert_eq!(
                x.checked_add(y).map(F::to_bits),
                expected(a as f64 + b as f64)
            );
            assert_eq!(
                x.checked_sub(y).map(F::to_bits),
                expected(a as f64 - b as f64)
            );
            assert_eq!(
                x.checked_mul(y).map(F::to_bits),
                expected(a as f64 * b as f64 / 16.0)
            );
            if b == 0 {
                assert_eq!(x.checked_div(y), Err(MathError::DivisionByZero));
                assert_eq!(x.checked_rem(y), Err(MathError::DivisionByZero));
            } else {
                assert_eq!(
                    x.checked_div(y).map(F::to_bits),
                    expected(a as f64 * 16.0 / b as f64)
                );
                assert_eq!(
                    x.checked_rem(y).unwrap().to_bits() as i16,
                    a as i16 % b as i16
                );
            }
        }
    }
}

#[test]
fn from_into_float_both_widths_and_directions() {
    type F = Fixed<i32, 8>;
    let single: F = 1.5f32.into();
    let double: F = (-2.25f64).into();
    assert_eq!(single.to_bits(), 384);
    assert_eq!(double.to_bits(), -576);
    let a: f32 = single.into();
    let b: f64 = single.into();
    let c: f32 = double.into();
    let d: f64 = double.into();
    assert_eq!((a, b, c, d), (1.5, 1.5, -2.25, -2.25));
    for bits in i8::MIN..=i8::MAX {
        let value = Fixed::<i8, 7>::from_bits(bits);
        let a: f32 = value.into();
        let b: f64 = value.into();
        assert_eq!(Fixed::<i8, 7>::try_from_f32(a).unwrap(), value);
        assert_eq!(Fixed::<i8, 7>::try_from_f64(b).unwrap(), value);
    }
}

#[test]
fn float_boundaries_and_signed_rounding_ties() {
    type F = Fixed<i8, 4>;
    for (raw, expected) in [
        (0.5, 0),
        (1.5, 2),
        (2.5, 2),
        (-0.5, 0),
        (-1.5, -2),
        (-2.5, -2),
    ] {
        assert_eq!(F::try_from_f64(raw / 16.0).unwrap().to_bits(), expected);
    }
    assert_eq!(F::try_from_f64(-8.0).unwrap().to_bits(), -128);
    assert_eq!(F::try_from_f64(8.0), Err(MathError::Overflow));
    assert_eq!(F::try_from_f64(f64::NAN), Err(MathError::NonFinite));
    assert_eq!(F::try_from_f32(f32::INFINITY), Err(MathError::NonFinite));
    assert_eq!(F::try_from_f64(f64::from_bits(1)).unwrap(), F::ZERO);
    assert_eq!(F::try_from_f64(-0.0).unwrap(), F::ZERO);
    assert_eq!(
        Fixed::<i64, 0>::try_from_f64(i64::MAX as f64),
        Err(MathError::Overflow)
    );
    assert_eq!(
        Fixed::<i64, 0>::try_from_f64(i64::MIN as f64)
            .unwrap()
            .to_bits(),
        i64::MIN
    );
    assert!(catch_unwind(|| {
        let _: F = f64::NAN.into();
    })
    .is_err());
    assert!(catch_unwind(|| {
        let _: F = 8.0f32.into();
    })
    .is_err());
    assert_eq!(F::max_value() + F::DELTA, F::min_value());
}

#[test]
fn exact_rescaling_rounding_and_layout() {
    type F = Fixed<i32, 8>;
    let value = F::try_from_ratio(-3, 2).unwrap();
    let wide: Fixed<i64, 32> = value.try_rescale().unwrap();
    assert_eq!(wide.to_bits(), -3i64 << 31);
    assert_eq!(wide.try_rescale::<i32, 8>().unwrap(), value);
    assert_eq!(F::from_bits(3).try_rescale::<i8, 7>().unwrap().to_bits(), 2);
    assert_eq!(value.trunc().to_f64(), -1.0);
    assert_eq!(value.floor().to_f64(), -2.0);
    assert_eq!(value.checked_ceil().unwrap().to_f64(), -1.0);
    assert_eq!(value.checked_round().unwrap().to_f64(), -2.0);
    assert_eq!(F::min_value().checked_neg(), Err(MathError::Overflow));
    assert_eq!(F::max_value().checked_ceil(), Err(MathError::Overflow));
    assert_eq!(core::mem::size_of::<F>(), core::mem::size_of::<i32>());
    assert_eq!(core::mem::align_of::<F>(), core::mem::align_of::<i32>());
}

#[test]
fn parsing_is_exact_and_round_trips_all_small_values() {
    for raw in i8::MIN..=i8::MAX {
        let value = Fixed::<i8, 7>::from_bits(raw);
        assert_eq!(format!("{}", value).parse::<Fixed<i8, 7>>().unwrap(), value);
    }
    type F = Fixed<i64, 63>;
    for raw in [i64::MIN, -1, 0, 1, i64::MAX] {
        let value = F::from_bits(raw);
        assert_eq!(format!("{}", value).parse::<F>().unwrap(), value);
    }
    type G = Fixed<i32, 8>;
    assert_eq!(G::from_str_radix("-a.8", 16).unwrap().to_f64(), -10.5);
    assert_eq!(G::from_str_radix("1.1", 2).unwrap().to_f64(), 1.5);
    assert_eq!(G::from_str_radix(".001953125", 10).unwrap().to_bits(), 0);
    assert_eq!(G::from_str_radix(".005859375", 10).unwrap().to_bits(), 2);
    for bad in ["", ".", "+", " 1", "1e2", "1.2.3", "nan"] {
        assert_eq!(bad.parse::<G>(), Err(MathError::InvalidNumber));
    }
    assert_eq!(G::from_str_radix("1", 1), Err(MathError::InvalidNumber));
    assert_eq!(
        G::from_str_radix("999999999999999999999999999999999999999999999", 10),
        Err(MathError::Overflow)
    );
}

#[test]
fn accumulation_keeps_cancellation_and_subquantum_products() {
    type F = Fixed<i64, 8>;
    let a = F::min_value();
    let terms = [
        (a, a, false),
        (a, a, false),
        (a, a, false),
        (a, a, true),
        (a, a, true),
        (a, a, true),
    ];
    assert_eq!(F::checked_sum_products(&terms).unwrap(), F::ZERO);
    assert_eq!(
        F::checked_sum_products(&[(F::from_bits(8), F::from_bits(8), false); 4]).unwrap(),
        F::DELTA
    );
}

#[test]
fn exhaustive_i16_analytic_reference_bounds() {
    type F = Fixed<i16, 8>;
    for raw in i16::MIN..=i16::MAX {
        let value = F::from_bits(raw);
        let angle = value.to_f64();
        let (sin, cos) = value.checked_sin_cos().unwrap();
        assert!(
            (sin.to_bits() as f64 - (angle.sin() * 256.0).round_ties_even()).abs() <= 1.0,
            "sin({})",
            angle
        );
        assert!(
            (cos.to_bits() as f64 - (angle.cos() * 256.0).round_ties_even()).abs() <= 1.0,
            "cos({})",
            angle
        );
        if raw >= 0 {
            assert_eq!(
                value.checked_sqrt().unwrap().to_bits(),
                (angle.sqrt() * 256.0).round_ties_even() as i16
            );
        } else {
            assert_eq!(value.checked_sqrt(), Err(MathError::Domain));
        }
        if (-256..=256).contains(&raw) {
            assert!(
                (value.checked_acos().unwrap().to_bits() as f64
                    - (angle.acos() * 256.0).round_ties_even())
                .abs()
                    <= 1.0
            );
        }
        let expected = angle.tan() * 256.0;
        if expected.abs() < 32760.0 {
            assert!(
                (value.checked_tan().unwrap().to_bits() as f64 - expected.round_ties_even()).abs()
                    <= 1.0
            );
        }
    }
}

#[test]
fn vector_fixed_accumulation_and_tiny_normalization() {
    type F = Fixed<i32, 8>;
    let tiny = Vector3::new(F::DELTA, F::ZERO, F::ZERO);
    assert_eq!(tiny.checked_length().unwrap(), F::DELTA);
    let unit = tiny.checked_normalize(F::ZERO).unwrap();
    assert_eq!(unit.x, F::ONE);
    assert!(tiny.checked_normalize(F::DELTA).is_err());
    assert_eq!(
        tiny.checked_normalize(-F::DELTA).unwrap_err(),
        MathError::InvalidTolerance
    );
    let ordinary = Vector3::new(
        F::try_from_int(3).unwrap(),
        F::try_from_int(4).unwrap(),
        F::ZERO,
    );
    assert_eq!(ordinary.checked_length().unwrap().to_f64(), 5.0);
    let normalized = ordinary.checked_normalize(F::epsilon()).unwrap();
    assert!((normalized.x.to_f64() - 0.6).abs() <= 1.0 / 256.0);
    assert!((normalized.y.to_f64() - 0.8).abs() <= 1.0 / 256.0);
    let left = F::try_from_int(2).unwrap() * ordinary;
    assert_eq!(left.x.to_f64(), 6.0);
    let fraction = F::from_bits(8);
    let v = Vector3::new(fraction, fraction, fraction);
    assert_eq!(Vector3::dot(&v, &v).to_bits(), 1);
    // A large represented direction can normalize even when its length cannot
    // fit its scalar format; normalization never constructs the raw magnitude.
    let large = Vector3::new(F::max_value(), F::max_value(), F::ZERO);
    assert_eq!(large.checked_length(), Err(MathError::Overflow));
    assert!(large.checked_normalize(F::epsilon()).is_ok());
}

#[test]
fn analytic_domains_and_high_precision_formats() {
    type F = Fixed<i64, 61>;
    assert_eq!(F::ONE.checked_sqrt().unwrap(), F::ONE);
    assert_eq!(F::ZERO.checked_sin().unwrap(), F::ZERO);
    assert_eq!(F::ZERO.checked_cos().unwrap(), F::ONE);
    assert_eq!(F::ONE.checked_acos().unwrap(), F::ZERO);
    assert_eq!((F::ONE + F::DELTA).checked_acos(), Err(MathError::Domain));
    assert_eq!(
        F::DELTA.checked_sqrt().unwrap().to_bits(),
        ((2.0f64).powi(61)).sqrt().round_ties_even() as i64
    );
    let quarter = F::quarter();
    assert!((quarter.checked_sin().unwrap().to_f64() - 0.25f64.sin()).abs() <= 1e-16);
    assert!(((-quarter).checked_acos().unwrap().to_f64() - (-0.25f64).acos()).abs() <= 4e-16);
    // Low fractional precision and the largest integer magnitude exercise
    // range reduction before increasing CORDIC's internal scale.
    type G = Fixed<i64, 2>;
    assert!(G::max_value().checked_sin_cos().is_ok());
    assert!(G::min_value().checked_sin_cos().is_ok());
    let minimum = F::min_value();
    assert_eq!(Scalar::checked_abs(minimum), Err(MathError::Overflow));
    assert_eq!(
        <F as Scalar>::checked_div(F::ONE, F::ZERO),
        Err(MathError::DivisionByZero)
    );
}

/// Checks guarded CORDIC against independent 120-digit Decimal Taylor values,
/// including magnitudes where a binary float reference loses the input bits.
fn check_wide_reference<const F: u32>()
where
    i64: RealFormat<F>,
{
    for line in include_str!("reference.csv")
        .lines()
        .filter(|s| !s.starts_with('#') && !s.is_empty())
    {
        let mut cells = line.split(',');
        if cells.next().unwrap().parse::<u32>().unwrap() != F {
            continue;
        }
        let raw = cells.next().unwrap().parse::<i64>().unwrap();
        let sine = cells.next().unwrap().parse::<i64>().unwrap();
        let cosine = cells.next().unwrap().parse::<i64>().unwrap();
        let (actual_sine, actual_cosine) =
            Fixed::<i64, F>::from_bits(raw).checked_sin_cos().unwrap();
        assert!(
            (actual_sine.to_bits() as i128 - sine as i128).abs() <= 1,
            "sin S{}.{} raw={}",
            63 - F,
            F,
            raw
        );
        assert!(
            (actual_cosine.to_bits() as i128 - cosine as i128).abs() <= 1,
            "cos S{}.{} raw={}",
            63 - F,
            F,
            raw
        );
    }
}

#[test]
fn full_range_cordic_matches_decimal_reference() {
    check_wide_reference::<2>();
    check_wide_reference::<8>();
    check_wide_reference::<16>();
    check_wide_reference::<32>();
    check_wide_reference::<61>();
}

#[test]
fn prepared_reciprocals_match_exhaustive_checked_division() {
    type F = Fixed<i8, 4>;
    for denominator in i8::MIN..=i8::MAX {
        let divisor = F::from_bits(denominator);
        if denominator == 0 {
            assert!(matches!(
                divisor.checked_reciprocal(),
                Err(MathError::DivisionByZero)
            ));
            continue;
        }
        let reciprocal = divisor.checked_reciprocal().unwrap();
        for numerator in i8::MIN..=i8::MAX {
            let value = F::from_bits(numerator);
            assert_eq!(value.mul_reciprocal(divisor.reciprocal()), value / divisor);
            assert_eq!(
                value.checked_mul_reciprocal(reciprocal),
                value.checked_div(divisor)
            );
        }
    }
    type Wide = Fixed<i64, 61>;
    for denominator in [
        i64::MIN,
        -(1i64 << 62) + 1,
        -3,
        -1,
        1,
        3,
        (1i64 << 62) - 1,
        i64::MAX,
    ] {
        let divisor = Wide::from_bits(denominator);
        let reciprocal = divisor.checked_reciprocal().unwrap();
        for numerator in [i64::MIN, -3, -1, 0, 1, 3, i64::MAX] {
            let value = Wide::from_bits(numerator);
            assert_eq!(value.mul_reciprocal(divisor.reciprocal()), value / divisor);
            assert_eq!(
                value.checked_mul_reciprocal(reciprocal),
                value.checked_div(divisor)
            );
        }
    }
}

/// Intermediate sums exceed i128 while exact cancellation leaves a small
/// result. The expected answers follow integer identities, independently of
/// either accumulator implementation.
#[test]
fn wrapping_wide_accumulation_preserves_final_bits_and_cancellation() {
    fn check<const F: u32>(expected_four_max: i64)
    where
        i64: Format<F>,
    {
        type R = i64;
        let min = Fixed::<R, F>::from_bits(i64::MIN);
        let max = Fixed::<R, F>::from_bits(i64::MAX);
        assert_eq!(
            Fixed::sum_products(&[(max, max, false); 4]).to_bits(),
            expected_four_max
        );
        // 2*MIN^2 - 2*MAX^2 = 2^65-2, regardless of an overflowing partial sum.
        let terms = [
            (min, min, false),
            (min, min, false),
            (max, max, true),
            (max, max, true),
        ];
        let numerator = (1u128 << 65) - 2;
        let quotient = if F == 0 {
            numerator
        } else {
            // At the only halfway case (F=2), the truncated quotient is
            // odd, so ties-to-even also rounds upward.
            (numerator + (1u128 << (F - 1))) >> F
        };
        assert_eq!(Fixed::sum_products(&terms).to_bits(), quotient as i64);
        if quotient <= i64::MAX as u128 {
            assert_eq!(
                Fixed::checked_sum_products(&terms).unwrap().to_bits(),
                quotient as i64
            );
        }
    }
    check::<0>(4);
    check::<2>(1);
    check::<8>(-(1i64 << 58));
    check::<32>(-(1i64 << 34));
    check::<61>(-32);
    check::<63>(-8);
}

#[test]
fn ordinary_analytic_operations_match_checked_valid_results() {
    type F = Fixed<i16, 8>;
    for bits in i16::MIN..=i16::MAX {
        let x = F::from_bits(bits);
        assert_eq!(x.sin_cos(), x.checked_sin_cos().unwrap());
        if let Ok(value) = x.checked_sqrt() {
            assert_eq!(x.sqrt(), value);
        }
        if let Ok(value) = x.checked_tan() {
            assert_eq!(x.tan(), value);
        }
        if let Ok(value) = x.checked_acos() {
            assert_eq!(x.acos(), value);
        }
    }
}

/// Pair specialization must match the original exact accumulator across edge values and signs.
#[test]
fn two_product_kernel_matches_exact_accumulator_at_storage_boundaries() {
    fn verify<const F: u32>()
    where
        i64: Format<F>,
    {
        let cases = [
            i64::MIN,
            i64::MIN + 1,
            -(1i64 << 40),
            -8193,
            -4096,
            -3,
            -1,
            0,
            1,
            3,
            4096,
            8193,
            1i64 << 40,
            i64::MAX - 1,
            i64::MAX,
        ];
        for &a in &cases {
            for &b in &cases {
                for &c in &cases {
                    for &d in &cases {
                        for subtract_a in [false, true] {
                            for subtract_b in [false, true] {
                                let mut exact = Accumulator::default();
                                exact.add(a as i128 * b as i128, subtract_a).unwrap();
                                exact.add(c as i128 * d as i128, subtract_b).unwrap();
                                let expected = exact.finish(F).and_then(Fixed::<i64, F>::raw);
                                let terms = [
                                    (Fixed::from_bits(a), Fixed::from_bits(b), subtract_a),
                                    (Fixed::from_bits(c), Fixed::from_bits(d), subtract_b),
                                ];
                                assert_eq!(Fixed::<i64, F>::checked_sum_products(&terms), expected);
                            }
                        }
                    }
                }
            }
        }
    }
    verify::<0>();
    verify::<6>();
    verify::<12>();
    verify::<32>();
    verify::<63>();
}

/// Storage-width overflow flags must agree with independent widened range checks at every scale.
#[test]
fn checked_storage_arithmetic_matches_widened_reference() {
    fn check<R: Format<F>, const F: u32>(cases: &[R]) {
        for &a in cases {
            for &b in cases {
                let left = Fixed::<R, F>::from_bits(a);
                let right = Fixed::<R, F>::from_bits(b);
                let expected = |bits| {
                    R::narrow(bits)
                        .map(Fixed::<R, F>::from_bits)
                        .ok_or(MathError::Overflow)
                };
                assert_eq!(left.checked_add(right), expected(a.widen() + b.widen()));
                assert_eq!(left.checked_sub(right), expected(a.widen() - b.widen()));
            }
        }
    }
    let small: [i8; 256] = core::array::from_fn(|index| index as i8);
    check::<i8, 0>(&small);
    check::<i8, 7>(&small);
    check::<i16, 8>(&[i16::MIN, i16::MIN + 1, -1, 0, 1, i16::MAX - 1, i16::MAX]);
    check::<i32, 6>(&[i32::MIN, i32::MIN + 1, -1, 0, 1, i32::MAX - 1, i32::MAX]);
    let wide = [i64::MIN, i64::MIN + 1, -1, 0, 1, i64::MAX - 1, i64::MAX];
    check::<i64, 0>(&wide);
    check::<i64, 12>(&wide);
    check::<i64, 63>(&wide);
}

/// Direct f32 decoding must equal the independent full-width f64 decoder after exact widening,
/// across exponent/sign boundaries, halfway mantissas, extreme formats and deterministic raw bits.
#[test]
fn single_precision_conversion_matches_exact_widened_decoder() {
    fn check<R: Format<F>, const F: u32>(bits: u32) {
        let value = f32::from_bits(bits);
        assert_eq!(
            Fixed::<R, F>::try_from_f32(value),
            Fixed::<R, F>::try_from_f64(value as f64),
            "bits={bits:08x}, F={F}"
        );
    }
    let verify = |bits| {
        check::<i8, 0>(bits);
        check::<i8, 7>(bits);
        check::<i16, 8>(bits);
        check::<i32, 6>(bits);
        check::<i64, 0>(bits);
        check::<i64, 12>(bits);
        check::<i64, 63>(bits);
    };
    for exponent in 0..=255 {
        for fraction in [0, 1, 2, 0x3f_ffff, 0x40_0000, 0x7f_fffe, 0x7f_ffff] {
            for sign in [0, 1 << 31] {
                verify(sign | (exponent << 23) | fraction);
            }
        }
    }
    let mut state = 0x9e37_79b9_u32;
    for _ in 0..100_000 {
        state ^= state << 13;
        state ^= state >> 17;
        state ^= state << 5;
        verify(state);
    }
    type Pixel = Fixed<i32, 6>;
    for (value, expected) in [
        (1. / 128., 0),
        (3. / 128., 2),
        (5. / 128., 2),
        (-1. / 128., 0),
        (-3. / 128., -2),
    ] {
        assert_eq!(Pixel::try_from_f32(value).unwrap().to_bits(), expected);
    }
    assert_eq!(
        Fixed::<i64, 63>::try_from_f32(-1.).unwrap().to_bits(),
        i64::MIN
    );
    assert_eq!(Fixed::<i64, 63>::try_from_f32(1.), Err(MathError::Overflow));
}
