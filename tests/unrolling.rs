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

//! Independent iterative references for explicitly expanded numeric kernels.

use core::fmt::Debug;
use rs_math3d::{Fixed, MathError, Matrix2, Matrix3, Matrix4, RealScalar, Scalar, TryCastFrom};

/// Retains the iterative Gauss-Jordan algorithm as a numerical/error-order
/// reference. Production kernels expand each supported dimension separately.
fn reference_rows<T: RealScalar, const N: usize>(
    mut values: [[T; N]; N],
) -> Result<[[T; N]; N], MathError> {
    let zero = T::zero();
    let one = T::one();
    let mut inverse = [[zero; N]; N];
    for (i, row) in inverse.iter_mut().enumerate() {
        row[i] = one;
    }
    for row in &values {
        for &value in row {
            if !value.is_finite() {
                return Err(MathError::NonFinite);
            }
        }
    }
    for column in 0..N {
        let mut pivot = column;
        for row in column + 1..N {
            if values[row][column].checked_abs()? > values[pivot][column].checked_abs()? {
                pivot = row;
            }
        }
        let divisor = values[pivot][column];
        if divisor == zero {
            return Err(MathError::Degenerate);
        }
        values.swap(column, pivot);
        inverse.swap(column, pivot);
        for entry in 0..N {
            values[column][entry] = values[column][entry].checked_div(divisor)?;
            inverse[column][entry] = inverse[column][entry].checked_div(divisor)?;
        }
        values[column][column] = one;
        for row in 0..N {
            if row == column {
                continue;
            }
            let factor = values[row][column];
            for entry in 0..N {
                values[row][entry] = T::checked_sum_products(&[
                    (values[row][entry], one, false),
                    (factor, values[column][entry], true),
                ])?;
                inverse[row][entry] = T::checked_sum_products(&[
                    (inverse[row][entry], one, false),
                    (factor, inverse[column][entry], true),
                ])?;
            }
            values[row][column] = zero;
        }
    }
    Ok(inverse)
}

/// Defines dimension-specific adapters so the reference can exercise the real
/// public matrix API without depending on production expansion helpers.
macro_rules! reference_cases {
    ($check:ident, $cases:ident, $matrix:ident, $n:literal, [$($i:literal => $field:ident),+]) => {
        /// Compares every output entry or exact error with the iterative reference.
        fn $check<T: RealScalar + Debug>(rows: [[T;$n];$n]) {
            let mut matrix = $matrix::<T>::identity();
            for column in 0..$n { $(matrix.col[column].$field = rows[$i][column];)+ }
            match (reference_rows(rows),matrix.checked_inverse_rows()) {
                (Ok(expected),Ok(actual)) => {
                    for column in 0..$n { $(assert_eq!(actual.col[column].$field,expected[$i][column]);)+ }
                }
                (Err(expected),Err(actual)) => assert_eq!(actual,expected),
                (expected,actual) => panic!("row inversion diverged: {:?} / {:?}",expected,actual),
            }
        }

        /// Covers every row permutation of signed scaled diagonal matrices,
        /// deterministic dense inputs, singular inputs and bounded error paths.
        fn $cases<T: RealScalar + TryCastFrom<i32> + Debug>() {
            let zero = T::zero();
            let mut count = 0;
            // Base-N enumeration deliberately stays iterative and independent:
            // accept only arrangements with distinct row indices.
            for encoding in 0..($n as usize).pow($n) {
                let mut remaining = encoding;
                let mut permutation = [0;$n];
                for row in &mut permutation { *row = remaining % $n; remaining /= $n; }
                let mut distinct = true;
                for i in 0..$n { for j in 0..i { distinct &= permutation[i]!=permutation[j]; } }
                if !distinct { continue; }
                count += 1;
                let mut values = [[zero;$n];$n];
                for row in 0..$n {
                    let value = (row as i32 + 1) * if row % 2 == 0 { 1 } else { -1 };
                    values[row][permutation[row]] = T::try_cast_from(value).unwrap();
                }
                $check(values);
            }
            assert_eq!(count,(1..=$n).product::<usize>());
            let mut state = 0x734d_1903u32;
            for _ in 0..64 {
                let mut values = [[zero;$n];$n];
                for row in &mut values {
                    for value in row {
                        state = state.wrapping_mul(1664525).wrapping_add(1013904223);
                        *value = T::try_cast_from((state % 11) as i32-5).unwrap();
                    }
                }
                $check(values);
            }
            $check([[zero;$n];$n]);
            let mut repeated = [[zero;$n];$n];
            repeated[0][0] = T::one();
            repeated[1] = repeated[0];
            $check(repeated);
        }
    };
}
reference_cases!(check2,cases2,Matrix2,2,[0=>x,1=>y]);
reference_cases!(check3,cases3,Matrix3,3,[0=>x,1=>y,2=>z]);
reference_cases!(check4,cases4,Matrix4,4,[0=>x,1=>y,2=>z,3=>w]);

#[test]
fn expanded_rows_match_fixed_reference_all_dimensions() {
    cases2::<Fixed<i32, 8>>();
    cases3::<Fixed<i32, 8>>();
    cases4::<Fixed<i32, 8>>();
    cases2::<Fixed<i64, 16>>();
    cases3::<Fixed<i64, 16>>();
    cases4::<Fixed<i64, 16>>();
}

#[cfg(any(feature = "std", feature = "libm", feature = "system-libm"))]
#[test]
fn expanded_rows_match_float_reference_all_dimensions() {
    cases2::<f32>();
    cases3::<f32>();
    cases4::<f32>();
    cases2::<f64>();
    cases3::<f64>();
    cases4::<f64>();
    let mut values = [[0.0f32; 3]; 3];
    values[2][1] = f32::NAN;
    check3(values);
    values[2][1] = f32::INFINITY;
    check3(values);
}

#[test]
fn short_scalar_dispatch_preserves_rounding_errors_and_fallback() {
    type F = Fixed<i64, 8>;
    let terms = [(F::from_bits(8), F::from_bits(8), false); 8];
    for n in 0..=terms.len() {
        let expected = F::from_bits(match n {
            0 | 1 | 2 => 0,
            3 | 4 | 5 => 1,
            _ => 2,
        });
        assert_eq!(
            <F as Scalar>::checked_sum_products(&terms[..n]).unwrap(),
            expected
        );
    }
    let min = F::min_value();
    let cancellation = [
        (min, min, false),
        (min, min, false),
        (min, min, true),
        (min, min, true),
    ];
    assert_eq!(
        <F as Scalar>::checked_sum_products(&cancellation).unwrap(),
        F::ZERO
    );
    for n in 0..=8 {
        let values = [F::DELTA; 8];
        assert_eq!(
            F::checked_norm_exceeds(&values[..n], F::ZERO).unwrap(),
            n > 0
        );
    }
    assert_eq!(
        F::checked_norm_exceeds(&[F::ONE], -F::DELTA),
        Err(MathError::InvalidTolerance)
    );
    assert_eq!(
        <i32 as Scalar>::checked_sum_products(&[(i32::MAX, 2, false), (i32::MAX, 2, true)]),
        Err(MathError::Overflow)
    );
}

#[cfg(any(feature = "std", feature = "libm", feature = "system-libm"))]
#[test]
fn short_float_dispatch_preserves_left_to_right_evaluation() {
    let terms = [
        (1e16f64, 1.0, false),
        (1.0, 1.0, false),
        (1e16, 1.0, true),
        (1.0, 1.0, false),
        (2.0, 1.0, false),
    ];
    for n in 0..=terms.len() {
        let expected =
            terms[..n].iter().fold(
                0.0,
                |sum, &(a, b, subtract)| if subtract { sum - a * b } else { sum + a * b },
            );
        assert_eq!(
            <f64 as Scalar>::sum_products(&terms[..n]).to_bits(),
            expected.to_bits()
        );
        assert_eq!(
            <f64 as Scalar>::checked_sum_products(&terms[..n])
                .unwrap()
                .to_bits(),
            expected.to_bits()
        );
    }
    assert_eq!(<f64 as Scalar>::sum_products(&terms[..4]), 1.0);
    assert_eq!(
        f32::checked_norm_exceeds(&[f32::NAN], 0.0),
        Err(MathError::NonFinite)
    );
    assert_eq!(
        f32::checked_norm_exceeds(&[f32::MAX, f32::MAX], 1.0),
        Ok(true)
    );
}

#[test]
fn expanded_octants_preserve_public_corner_order() {
    use rs_math3d::{Box3, Vector3};
    type F = Fixed<i32, 8>;
    let scalar = |n| F::try_from_int(n).unwrap();
    let low = Vector3::new(scalar(-2), scalar(-4), scalar(-6));
    let high = Vector3::new(scalar(2), scalar(4), scalar(6));
    let bounds = Box3::new(&low, &high);
    let octants = bounds.subdivide();
    let corners = [
        [0, 1, 0],
        [1, 1, 0],
        [1, 1, 1],
        [0, 1, 1],
        [0, 0, 0],
        [1, 0, 0],
        [1, 0, 1],
        [0, 0, 1],
    ];
    for (octant, corner) in octants.iter().zip(corners) {
        assert_eq!(octant.min.x, if corner[0] == 0 { low.x } else { F::ZERO });
        assert_eq!(octant.min.y, if corner[1] == 0 { low.y } else { F::ZERO });
        assert_eq!(octant.min.z, if corner[2] == 0 { low.z } else { F::ZERO });
        assert_eq!(octant.max.x, if corner[0] == 1 { high.x } else { F::ZERO });
        assert_eq!(octant.max.y, if corner[1] == 1 { high.y } else { F::ZERO });
        assert_eq!(octant.max.z, if corner[2] == 1 { high.z } else { F::ZERO });
    }
}

#[test]
fn expanded_queries_cover_each_axis_and_triangle_edge() {
    use rs_math3d::{Box3, Intersect, Sphere3, Tri3, Vector3};
    type F = Fixed<i32, 8>;
    let scalar = |n| F::try_from_int(n).unwrap();
    let origin = Vector3::new(F::ZERO, F::ZERO, F::ZERO);
    let one = Vector3::new(F::ONE, F::ONE, F::ONE);
    let bounds = Box3::new(&origin, &one);
    let half = F::try_from_ratio(1, 2).unwrap();
    for center in [
        Vector3::new(scalar(2), half, half),
        Vector3::new(half, scalar(2), half),
        Vector3::new(half, half, scalar(2)),
    ] {
        assert!(bounds
            .checked_intersect(&Sphere3::new(center, F::ONE))
            .unwrap());
    }
    let corner = Vector3::new(scalar(2), scalar(2), scalar(2));
    assert!(!bounds
        .checked_intersect(&Sphere3::new(corner, F::try_from_ratio(3, 2).unwrap()))
        .unwrap());
    assert!(bounds
        .checked_intersect(&Sphere3::new(corner, F::try_from_ratio(7, 4).unwrap()))
        .unwrap());
    let triangle = Tri3::new([
        origin,
        Vector3::new(scalar(4), F::ZERO, F::ZERO),
        Vector3::new(F::ZERO, scalar(4), F::ZERO),
    ]);
    for (center, radius) in [
        (Vector3::new(scalar(2), scalar(-1), F::ZERO), F::ONE),
        (
            Vector3::new(scalar(3), scalar(3), F::ZERO),
            F::try_from_ratio(3, 2).unwrap(),
        ),
        (Vector3::new(scalar(-1), scalar(2), F::ZERO), F::ONE),
    ] {
        assert!(Sphere3::new(center, radius)
            .checked_intersect(&triangle)
            .unwrap());
    }
    assert!(
        !Sphere3::new(Vector3::new(scalar(2), scalar(-2), F::ZERO), F::ONE)
            .checked_intersect(&triangle)
            .unwrap()
    );
}
