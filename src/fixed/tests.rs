//! Exhaustive small-format reference checks and wide conversion boundaries.

use super::*;
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
            assert_eq!(x.try_add(y).map(F::to_bits), expected(a as f64 + b as f64));
            assert_eq!(x.try_sub(y).map(F::to_bits), expected(a as f64 - b as f64));
            assert_eq!(
                x.try_mul(y).map(F::to_bits),
                expected(a as f64 * b as f64 / 16.0)
            );
            if b == 0 {
                assert_eq!(x.try_div(y), Err(MathError::DivisionByZero));
                assert_eq!(x.try_rem(y), Err(MathError::DivisionByZero));
            } else {
                assert_eq!(
                    x.try_div(y).map(F::to_bits),
                    expected(a as f64 * 16.0 / b as f64)
                );
                assert_eq!(x.try_rem(y).unwrap().to_bits() as i16, a as i16 % b as i16);
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
    assert!(catch_unwind(|| F::max_value() + F::DELTA).is_err());
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
    assert_eq!(value.try_ceil().unwrap().to_f64(), -1.0);
    assert_eq!(value.try_round().unwrap().to_f64(), -2.0);
    assert_eq!(F::min_value().try_neg(), Err(MathError::Overflow));
    assert_eq!(F::max_value().try_ceil(), Err(MathError::Overflow));
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
    assert_eq!(F::sum_products(&terms).unwrap(), F::ZERO);
    assert_eq!(
        F::sum_products(&[(F::from_bits(8), F::from_bits(8), false); 4]).unwrap(),
        F::DELTA
    );
}
