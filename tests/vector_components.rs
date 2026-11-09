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

//! Shared component reductions, independent of vector length and float backends.

use core::fmt::Debug;
use rs_math3d::{Fixed, MathError, RealVector, Scalar, Vector, Vector2, Vector3, Vector4};

/// Compare both arithmetic policies to an independently supplied component bound.
/// Unrepresentable absolutes are checked only; ordinary arithmetic requires fit.
fn assert_reduction<T: Scalar + Debug, V: Vector<T>>(v: V, expected: Result<T, MathError>) {
    assert_eq!(v.checked_max_abs_component(), expected);
    if let Ok(bound) = expected {
        assert_eq!(v.max_abs_component(), bound);
    }
}

/// Every coordinate in every supported dimension must participate in reduction
/// and validation, including coordinates following a larger, already-seen value.
fn assert_each_coordinate<T: Scalar + Debug>(value: T, expected: Result<T, MathError>) {
    let z = T::zero();
    for v in [Vector2::new(value, z), Vector2::new(z, value)] {
        assert_reduction(v, expected);
    }
    for v in [
        Vector3::new(value, z, z),
        Vector3::new(z, value, z),
        Vector3::new(z, z, value),
    ] {
        assert_reduction(v, expected);
    }
    for v in [
        Vector4::new(value, z, z, z),
        Vector4::new(z, value, z, z),
        Vector4::new(z, z, value, z),
        Vector4::new(z, z, z, value),
    ] {
        assert_reduction(v, expected);
    }
}

/// All raw i8 fixed values exercise signs, zero, every extremum position and the
/// asymmetric minimum against native checked integer absolute value as oracle.
#[test]
fn fixed_component_reduction_covers_every_i8_value_and_coordinate() {
    type F = Fixed<i8, 4>;
    for raw in i8::MIN..=i8::MAX {
        let expected = raw
            .checked_abs()
            .map(F::from_bits)
            .ok_or(MathError::Overflow);
        assert_each_coordinate(F::from_bits(raw), expected);
    }
}

/// Mixed components/ties distinguish the largest component from Euclidean length.
#[test]
fn integer_component_reductions_keep_signs_ties_and_range_boundaries() {
    assert_reduction(Vector2::new(-3i32, 4), Ok(4));
    assert_reduction(Vector3::new(-3i32, 4, -5), Ok(5));
    assert_reduction(Vector4::new(-5i64, 4, -5, 2), Ok(5));
    assert_each_coordinate(i32::MIN, Err(MathError::Overflow));
    assert_each_coordinate(i64::MIN, Err(MathError::Overflow));
    assert_each_coordinate(i32::MIN + 1, Ok(i32::MAX));
    assert_each_coordinate(i64::MIN + 1, Ok(i64::MAX));
}

/// Scalar component reductions need no floating analytic backend. Subnormals,
/// MAX and every IEEE nonfinite category must be handled in every coordinate.
#[test]
fn float_component_reductions_preserve_extremes_and_reject_nonfinite() {
    macro_rules! check_float {
        ($ty:ty) => {{
            let tiny = <$ty>::from_bits(1);
            for value in [0.0, -0.0, tiny, -tiny, <$ty>::MAX, -<$ty>::MAX] {
                assert_each_coordinate(value, Ok(value.abs()));
            }
            for value in [<$ty>::NAN, <$ty>::INFINITY, <$ty>::NEG_INFINITY] {
                assert_each_coordinate(value, Err(MathError::NonFinite));
            }
            let zeros = Vector4::<$ty>::new(-0.0, 0.0, -0.0, -0.0);
            assert_eq!(zeros.max_abs_component().to_bits(), 0);
            assert_eq!(zeros.checked_max_abs_component().unwrap().to_bits(), 0);
            // A preceding maximum cannot hide a later invalid component.
            assert_eq!(
                Vector2::new(<$ty>::MAX, <$ty>::NAN).checked_max_abs_component(),
                Err(MathError::NonFinite)
            );
        }};
    }
    check_float!(f32);
    check_float!(f64);
}

/// Neither quantized squares nor an overflowing Euclidean norm may influence
/// the largest represented component, on both replayed fixed storage widths.
#[test]
fn fixed_component_bounds_survive_squared_underflow_and_norm_overflow() {
    macro_rules! check_fixed {
        ($ty:ty) => {{
            let tiny = Vector3::new(<$ty>::DELTA, <$ty>::ZERO, <$ty>::ZERO);
            assert_eq!(tiny.length_squared(), <$ty>::ZERO);
            assert_reduction(tiny, Ok(<$ty>::DELTA));
            let large = Vector4::new(
                <$ty>::max_value(),
                <$ty>::max_value(),
                <$ty>::max_value(),
                <$ty>::max_value(),
            );
            assert_eq!(large.checked_length(), Err(MathError::Overflow));
            assert_reduction(large, Ok(<$ty>::max_value()));
            assert_each_coordinate(<$ty>::min_value(), Err(MathError::Overflow));
        }};
    }
    check_fixed!(Fixed<i32, 16>);
    check_fixed!(Fixed<i64, 32>);
}
