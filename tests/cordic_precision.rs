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

//! Precision and contract checks for the opt-in const-selected CORDIC path.

use rs_math3d::{fixed::RealFormat, Fixed, MathError};

/// Exercises every i16 input at one scale against independent host analytics.
/// The input and reference resolution here are comfortably within f64 precision.
fn exhaustive<const F: u32>()
where
    i16: RealFormat<F>,
{
    let scale = (1u64 << F) as f64;
    for raw in i16::MIN..=i16::MAX {
        let input = Fixed::<i16, F>::from_bits(raw);
        let x = raw as f64 / scale;
        let result = input.checked_sin_cos_with::<64>().unwrap();
        assert_eq!(result, input.sin_cos_with::<64>());
        assert!((result.0.to_bits() as f64 - x.sin() * scale).abs() <= 1.0);
        assert!((result.1.to_bits() as f64 - x.cos() * scale).abs() <= 1.0);
        assert_eq!(input.sin_cos(), input.sin_cos_with::<128>());
        if raw as i64 >= -(1i64 << F) && raw as i64 <= 1i64 << F {
            let angle = input.checked_acos_with::<64>().unwrap();
            assert_eq!(angle, input.acos_with::<64>());
            assert!((angle.to_bits() as f64 - x.acos() * scale).abs() <= 1.0);
        } else {
            assert_eq!(input.checked_acos_with::<64>(), Err(MathError::Domain));
        }
        if let Ok(tangent) = input.checked_tan_with::<64>() {
            assert_eq!(tangent, input.tan_with::<64>());
        }
    }
}

/// Exhaustive coverage includes minimum F, every i16 format, and the largest
/// i16 scale. This runs without selecting a floating backend for the crate.
#[test]
fn all_i16_formats_meet_one_quantum_sine_cosine_acos_bounds() {
    exhaustive::<2>();
    exhaustive::<3>();
    exhaustive::<4>();
    exhaustive::<5>();
    exhaustive::<6>();
    exhaustive::<7>();
    exhaustive::<8>();
    exhaustive::<9>();
    exhaustive::<10>();
    exhaustive::<11>();
    exhaustive::<12>();
    exhaustive::<13>();
}

/// Full-range i32/i64 references use high-precision integer-plus-residual values,
/// avoiding loss of input bits in a float conversion and preserving ULP metrics.
fn high_precision<const F: u32>()
where
    i64: RealFormat<F>,
{
    let mut count = 0;
    for row in include_str!("../src/fixed/cordic_reference.csv")
        .lines()
        .filter(|s| !s.starts_with('#'))
    {
        let cells: Vec<_> = row.split(',').collect();
        if cells[1].parse::<u32>().unwrap() != F || cells[2] == "tan" {
            continue;
        }
        let raw = cells[3].parse::<i64>().unwrap();
        let expected = cells[4].parse::<i128>().unwrap();
        let residual = cells[5].parse::<f64>().unwrap();
        let input = Fixed::<i64, F>::from_bits(raw);
        let result = match cells[2] {
            "sin" => input.checked_sin_cos_with::<64>().unwrap().0,
            "cos" => input.checked_sin_cos_with::<64>().unwrap().1,
            "acos" => input.checked_acos_with::<64>().unwrap(),
            _ => panic!("unknown reference operation"),
        };
        let error = ((result.to_bits() as i128 - expected) as f64 - residual).abs();
        assert!(
            error <= 1.0,
            "F={} op={} raw={} error={}",
            F,
            cells[2],
            raw,
            error
        );
        count += 1;
    }
    assert!(count > 700);
}

/// Includes all distinct working precisions in the benchmark, with F=44/48
/// testing the Q60 cap and F=48 the minimum twelve guard bits.
#[test]
fn i64_core_matches_independent_high_precision_references() {
    high_precision::<8>();
    high_precision::<16>();
    high_precision::<29>();
    high_precision::<32>();
    high_precision::<44>();
    high_precision::<48>();
}

/// Validates endpoint identities and domain checking independently of the bulk
/// reference statistics, including the exact-zero vectoring terminal state.
#[test]
fn selected_paths_preserve_endpoints_and_checked_domains() {
    type F = Fixed<i32, 16>;
    assert_eq!(F::ZERO.sin_cos_with::<64>(), (F::ZERO, F::ONE));
    assert_eq!(F::ZERO.tan_with::<64>(), F::ZERO);
    assert_eq!(F::ONE.acos_with::<64>(), F::ZERO);
    assert_eq!(F::ONE.checked_acos_with::<64>(), Ok(F::ZERO));
    assert_eq!(
        (F::ONE + F::DELTA).checked_acos_with::<64>(),
        Err(MathError::Domain)
    );
    assert_eq!(
        (-F::ONE - F::DELTA).checked_acos_with::<64>(),
        Err(MathError::Domain)
    );
    for input in [-F::ONE, -F::DELTA, F::ZERO, F::DELTA, F::ONE] {
        assert_eq!(input.acos_with::<128>(), input.acos());
        assert_eq!(input.tan_with::<128>(), input.tan());
        assert_eq!(input.checked_acos_with::<128>(), input.checked_acos());
        assert_eq!(input.checked_tan_with::<128>(), input.checked_tan());
    }
}
