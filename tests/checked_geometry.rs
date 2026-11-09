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

//! Consumer regressions for represented structure, parameter and input contracts.
use core::fmt::Debug;
use rs_math3d::*;

/// Every fixed quantum is meaningful structure, including terms accepted by a
/// deliberately approximate classifier. Both affine inverse modes must reject it.
#[test]
fn fixed_projective_rows_are_not_discarded() {
    type F = Fixed<i32, 8>;
    for column in 0..4 {
        for sign in [-F::ONE, F::ONE] {
            let mut m = Matrix4::<F>::identity();
            m.col[column].w = m.col[column].w + sign * F::DELTA;
            assert!(m.checked_is_affine(F::DELTA).unwrap());
            assert!(!m.checked_is_affine(F::ZERO).unwrap());
            for mode in [InverseMode::Reciprocal, InverseMode::Rows] {
                assert!(matches!(
                    m.checked_inverse_affine_with(mode),
                    Err(MathError::Domain)
                ));
                let inv = m.checked_inverse_with(mode).unwrap();
                let product = Matrix4::checked_mul_matrix_matrix(&m, &inv).unwrap();
                for (i, c) in product.col.iter().enumerate() {
                    for (j, value) in [c.x, c.y, c.z, c.w].iter().copied().enumerate() {
                        let expected = if i == j { F::ONE } else { F::ZERO };
                        assert!(
                            (value - expected).tabs() <= F::DELTA * F::try_from_int(2).unwrap()
                        );
                    }
                }
            }
            assert!(matches!(
                transforms::checked_decompose_affine(&m),
                Err(MathError::Domain)
            ));
        }
        let mut extreme = Matrix4::<F>::identity();
        extreme.col[column].w = F::min_value();
        assert!(!extreme.is_affine(F::ZERO));
        assert_eq!(extreme.checked_is_affine(F::ZERO), Ok(false));
        assert!(matches!(
            extreme.checked_inverse_affine(),
            Err(MathError::Domain)
        ));
    }
}

/// Valid exactly affine matrices retain their optimized inverses and decomposition.
#[test]
fn exact_fixed_affine_inputs_keep_both_inverse_modes() {
    type F = Fixed<i32, 16>;
    let n = |x| F::try_from_int(x).unwrap();
    let m = transforms::translate(Vector3::new(n(3), n(-2), n(1)))
        * transforms::scale(Vector3::new(n(2), n(4), F::half()));
    for mode in [InverseMode::Reciprocal, InverseMode::Rows] {
        let inverse = m.checked_inverse_affine_with(mode).unwrap();
        let product = Matrix4::checked_mul_matrix_matrix(&m, &inverse).unwrap();
        for (i, c) in product.col.iter().enumerate() {
            for (j, value) in [c.x, c.y, c.z, c.w].iter().copied().enumerate() {
                assert_eq!(value, if i == j { F::ONE } else { F::ZERO });
            }
        }
    }
    assert!(transforms::checked_decompose_affine(&m).is_ok());
}

/// Small projective coefficients can cause visible error even inside a viewport.
#[cfg(any(feature = "std", feature = "libm", feature = "system-libm"))]
#[test]
fn checked_unproject_preserves_small_projective_coefficients() {
    let world = Matrix4::<f32>::identity();
    let mut projection = transforms::scale(Vector3::new(1. / 1024., 1. / 1024., 1. / 1024.));
    projection.col[0].w = EPS_F32;
    let (lb, rt) = (Vector2::new(0., 0.), Vector2::new(1., 1.));
    let p = Vector3::new(1024., 0., 0.);
    let screen = transforms::checked_project3(&world, &projection, &lb, &rt, &p).unwrap();
    assert!(screen.x > 0. && screen.x < 1.);
    let restored = transforms::checked_unproject3(&world, &projection, &lb, &rt, &screen).unwrap();
    assert!((restored.x - p.x).abs() < 0.001);
    assert_eq!(restored.y, 0.);
    assert_eq!(restored.z, 0.);

    let world = Matrix4::<f64>::identity();
    let mut projection = world;
    projection.col[0].w = 5e-13;
    let (lb, rt) = (Vector2::new(-1., -1.), Vector2::new(1., 1.));
    let p = Vector3::new(2e12, 0., 0.);
    let screen = transforms::checked_project3(&world, &projection, &lb, &rt, &p).unwrap();
    let restored = transforms::checked_unproject3(&world, &projection, &lb, &rt, &screen).unwrap();
    assert!((restored.x - p.x).abs() < 1.);

    projection.col[1].x = f64::NAN;
    assert!(matches!(
        projection.checked_inverse_affine(),
        Err(MathError::NonFinite)
    ));
    assert!(matches!(
        transforms::checked_decompose_affine(&projection),
        Err(MathError::NonFinite)
    ));
}

/// The zero/nonzero invariant is dimension-independent. Scalar division may
/// quantize an interior parameter to zero; geometric result tuples may not.
fn fixed_closest_contract<V>(origin: V, direction: V, target: V)
where
    V: RealVector<Fixed<i32, 8>> + Debug,
{
    type F = Fixed<i32, 8>;
    let line = Line::checked_new(&origin, &direction, F::ZERO).unwrap();
    let segment = Segment::new(&origin, &direction);
    for p in [target, -target] {
        assert!(matches!(
            line.checked_closest_point_on_line(&p, F::ZERO),
            Err(MathError::Overflow)
        ));
    }
    assert!(matches!(
        segment.checked_closest_point_on_segment(&target, F::ZERO),
        Err(MathError::Overflow)
    ));
    let check = |actual: (F, V), expected: (F, V)| {
        assert_eq!(actual.0, expected.0);
        assert_eq!((actual.1 - expected.1).checked_length().unwrap(), F::ZERO);
    };
    check(
        line.checked_closest_point_on_line(&origin, F::ZERO)
            .unwrap(),
        (F::ZERO, origin),
    );
    check(
        segment
            .checked_closest_point_on_segment(&(-target), F::ZERO)
            .unwrap(),
        (F::ZERO, origin),
    );
    check(
        segment
            .checked_closest_point_on_segment(&(direction + direction), F::ZERO)
            .unwrap(),
        (F::ONE, direction),
    );
    let unit = direction.checked_normalize(F::ZERO).unwrap();
    let short = Line::checked_new(&origin, &unit, F::ZERO).unwrap();
    check(
        short
            .checked_closest_point_on_line(&target, F::ZERO)
            .unwrap(),
        (F::DELTA, target),
    );
}
#[test]
fn fixed_closest_parameters_preserve_nonzero_travel_in_all_dimensions() {
    type F = Fixed<i32, 8>;
    let (z, d, p) = (F::ZERO, F::try_from_int(256).unwrap(), F::DELTA);
    fixed_closest_contract(Vector2::new(z, z), Vector2::new(d, z), Vector2::new(p, z));
    fixed_closest_contract(
        Vector3::new(z, z, z),
        Vector3::new(d, z, z),
        Vector3::new(p, z, z),
    );
    fixed_closest_contract(
        Vector4::new(z, z, z, z),
        Vector4::new(d, z, z, z),
        Vector4::new(p, z, z, z),
    );
}

/// Exercise finite floating underflow and minimum-subnormal success separately.
#[cfg(any(feature = "std", feature = "libm", feature = "system-libm"))]
#[test]
fn float_closest_parameters_reject_lost_travel_but_keep_subnormals() {
    macro_rules! check {
        ($t:ty,$large:expr,$tiny:expr) => {{
            let zero = Vector3::<$t>::new(0., 0., 0.);
            let direction = Vector3::new($large, 0., 0.);
            let p = Vector3::new($tiny, 0., 0.);
            let line = Line::checked_new(&zero, &direction, 0.).unwrap();
            let segment = Segment::new(&zero, &direction);
            assert!(matches!(
                line.checked_closest_point_on_line(&p, 0.),
                Err(MathError::Overflow)
            ));
            assert!(matches!(
                segment.checked_closest_point_on_segment(&p, 0.),
                Err(MathError::Overflow)
            ));
            let unit = Vector3::new(1., 0., 0.);
            let line = Line::checked_new(&zero, &unit, 0.).unwrap();
            let segment = Segment::new(&zero, &unit);
            let tiny = <$t>::from_bits(1);
            let point = Vector3::new(tiny, 0., 0.);
            for (t, p) in [
                line.checked_closest_point_on_line(&point, 0.).unwrap(),
                segment
                    .checked_closest_point_on_segment(&point, 0.)
                    .unwrap(),
            ] {
                assert_eq!(t, tiny);
                assert_eq!([p.x, p.y, p.z], [tiny, 0., 0.]);
            }
            let (t, p) = line.checked_closest_point_on_line(&zero, 0.).unwrap();
            assert_eq!(t, 0.);
            assert_eq!([p.x, p.y, p.z], [0., 0., 0.]);
        }};
    }
    check!(f32, 1e30f32, 1e-30f32);
    check!(f64, 1e300f64, 1e-200f64);
}

/// Invalid plane storage is diagnosed before orientation-based misses through
/// both owning entry points and the symmetric ray forwarding entry point.
#[cfg(any(feature = "std", feature = "libm", feature = "system-libm"))]
#[test]
fn checked_plane_queries_validate_constants_before_parallel_misses() {
    let zero = Vector3::new(0.0f64, 0., 0.);
    let normal = Vector3::new(0., 0., 1.);
    for value in [f64::NAN, f64::INFINITY, f64::NEG_INFINITY] {
        let plane = Plane::new(&normal, &Vector3::new(0., 0., value));
        for origin in [zero, normal, -normal] {
            for direction in [
                Vector3::new(1., 0., 0.),
                Vector3::new(0., 1., 0.),
                normal,
                -normal,
            ] {
                let ray = Ray::checked_new(&origin, &direction, 0.).unwrap();
                let line = Line::checked_new(&origin, &direction, 0.).unwrap();
                assert!(matches!(
                    ray.checked_intersect_plane(&plane, 0.),
                    Err(MathError::NonFinite)
                ));
                assert!(matches!(
                    plane.checked_intersect_ray(&ray, 0.),
                    Err(MathError::NonFinite)
                ));
                assert!(matches!(
                    plane.checked_intersect_line(&line, 0.),
                    Err(MathError::NonFinite)
                ));
            }
        }
    }
    // A finite constant need not be used in an actual solve for a parallel miss.
    for height in [1., f64::MAX] {
        let plane = Plane::new(&normal, &Vector3::new(0., 0., height));
        let direction = Vector3::new(1., 0., 0.);
        let ray = Ray::checked_new(&zero, &direction, 0.).unwrap();
        let line = Line::checked_new(&zero, &direction, 0.).unwrap();
        assert!(matches!(ray.checked_intersect_plane(&plane, 0.), Ok(None)));
        assert!(matches!(plane.checked_intersect_line(&line, 0.), Ok(None)));
    }
}
