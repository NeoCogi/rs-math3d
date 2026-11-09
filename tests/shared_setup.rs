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

//! Shared conversion, component mapping, clamping and column packing contracts.

use rs_math3d::{
    Fixed, MathError, Matrix2, Matrix3, Matrix4, Scalar, TryCastFrom, Vector2, Vector3, Vector4,
};

/// Panicking setup conversion must delegate quantization and range to the
/// existing checked trait, including exact fixed-to-fixed rescaling.
#[test]
fn setup_casts_keep_checked_conversion_results_and_errors() {
    type F = Fixed<i32, 16>;
    for x in [0., -0., 0.1, -0.1, 0.5, 1.5 / 65536., 2.5 / 65536., 32767.] {
        assert_eq!(F::cast_from(x), F::try_cast_from(x).unwrap());
    }
    for x in [f64::NAN, f64::INFINITY, -f64::INFINITY, 32768.] {
        assert!(F::try_cast_from(x).is_err());
        assert!(std::panic::catch_unwind(|| F::cast_from(x)).is_err());
    }
    // Primitive destinations retain their existing NumCast exceptional-value policy.
    assert!(f32::cast_from(f64::NAN).is_nan());
    assert_eq!(f64::cast_from(f32::INFINITY), f64::INFINITY);
    type Wide = Fixed<i64, 16>;
    let source = Wide::from_bits((1i64 << 60) + 1);
    assert_eq!(Wide::cast_from(source).to_bits(), source.to_bits());
    assert_eq!(i32::cast_from(-3.75f64), -3);
}

/// Mapping owns components and invokes a stateful callback exactly once per
/// coordinate. Non-Copy values must not require numeric or clone capabilities.
#[test]
fn vector_arrays_and_maps_preserve_ownership_and_coordinate_order() {
    let source = Vector3::from_array([String::from("x"), String::from("yy"), String::from("zzz")]);
    let mut seen = Vec::new();
    let mapped = source.map(|s| {
        seen.push(s.len());
        s.into_bytes()
    });
    assert_eq!(seen, [1, 2, 3]);
    assert_eq!(
        mapped.to_array(),
        [vec![b'x'], vec![b'y', b'y'], vec![b'z', b'z', b'z']]
    );
    assert_eq!(
        Vector2::from_array([7, -9]).map(|x| x * 2).to_array(),
        [14, -18]
    );
    assert_eq!(
        Vector4::from_array([1, 2, 3, 4]).map(|x| x + 1).to_array(),
        [2, 3, 4, 5]
    );
    type F = Fixed<i64, 16>;
    let raw = [(1i64 << 60) + 1, i64::MIN, i64::MAX, -1];
    assert_eq!(
        Vector4::from_array(raw)
            .map(F::from_bits)
            .map(F::to_bits)
            .to_array(),
        raw
    );
    let bits = [0x8000_0000u32, 0x7fc0_1234];
    assert_eq!(
        Vector2::from_array(bits)
            .map(f32::from_bits)
            .map(f32::to_bits)
            .to_array(),
        bits
    );
}

/// Matrix packing follows constructor column order for each supported size,
/// and does not quantize wide raw fixed values through a float representation.
#[test]
fn matrix_packing_is_column_major_and_preserves_raw_components() {
    assert_eq!(Matrix2::new(1, 2, 3, 4).to_cols_array(), [1, 2, 3, 4]);
    assert_eq!(
        Matrix3::new(1, 2, 3, 4, 5, 6, 7, 8, 9).to_cols_array(),
        [1, 2, 3, 4, 5, 6, 7, 8, 9]
    );
    let m = Matrix4::new(1i64, 2, 3, 4, 5, 6, 7, 8, 9, 10, 11, 12, 13, 14, 15, 16);
    assert_eq!(
        m.to_cols_array(),
        core::array::from_fn::<_, 16, _>(|i| i as i64 + 1)
    );
    type F = Fixed<i64, 16>;
    let fixed = m.try_cast::<F>().unwrap();
    assert_eq!(
        fixed.to_cols_array().map(F::to_bits),
        m.to_cols_array().map(|x| x << 16)
    );
    let wide = F::from_bits((1i64 << 60) + 1);
    let fixed = Matrix2::new(wide, F::min_value(), F::max_value(), F::DELTA);
    assert_eq!(
        fixed.to_cols_array().map(F::to_bits),
        [wide.to_bits(), i64::MIN, i64::MAX, 1]
    );
}

/// Closed endpoints and a zero-width interval are valid; reversed or exceptional
/// bounds are errors, and the unchecked path delegates only ordinary arithmetic.
#[test]
fn scalar_clamping_has_shared_closed_interval_and_validation_contracts() {
    for (x, expected) in [(-5, -2), (-2, -2), (0, 0), (3, 3), (5, 3)] {
        assert_eq!(Scalar::clamp(x, -2, 3), expected);
        assert_eq!(x.checked_clamp(-2, 3), Ok(expected));
    }
    assert_eq!(7i32.checked_clamp(3, 3), Ok(3));
    assert_eq!(0i32.checked_clamp(3, -2), Err(MathError::Domain));
    for invalid in [f64::NAN, f64::INFINITY, f64::NEG_INFINITY] {
        assert_eq!(invalid.checked_clamp(-1., 1.), Err(MathError::NonFinite));
        assert_eq!(0f64.checked_clamp(invalid, 1.), Err(MathError::NonFinite));
        assert_eq!(0f64.checked_clamp(-1., invalid), Err(MathError::NonFinite));
    }
    type F = Fixed<i32, 16>;
    assert_eq!(F::DELTA.checked_clamp(F::ZERO, F::ONE), Ok(F::DELTA));
    assert_eq!(Scalar::clamp(F::min_value(), -F::ONE, F::ONE), -F::ONE);
}
