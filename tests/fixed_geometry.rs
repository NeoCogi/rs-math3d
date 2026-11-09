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

//! Consumer-level coverage of signed fixed scalars across every geometry module.

use rs_math3d::*;

/// Shared S23.8 fixture scalar, with enough integer headroom for scene geometry.
type F = Fixed<i32, 8>;

/// Constructs exact integer fixtures without a float intermediary.
fn f(value: i64) -> F {
    F::try_from_int(value).unwrap()
}

/// Checks a numeric answer within an explicit number of destination quanta.
fn close(value: F, expected: f64, quanta: f64) {
    assert!(
        (value.to_f64() - expected).abs() <= quanta / 256.0,
        "{} != {}",
        value,
        expected
    );
}

#[test]
fn matrix_modes_and_default_preserve_small_reciprocals() {
    assert_eq!(InverseMode::default(), InverseMode::Reciprocal);
    // Scalar 1/det rounds to zero, but inverse diagonal 1/32 is representable.
    let m2 = Matrix2::new(f(32), f(0), f(0), f(32));
    assert_eq!((F::ONE / m2.determinant()).to_bits(), 0);
    for mode in [InverseMode::Reciprocal, InverseMode::Rows] {
        let inverse = m2.checked_inverse_with(mode).unwrap();
        close(inverse.col[0].x, 1.0 / 32.0, 0.0);
        close((m2 * inverse).col[1].y, 1.0, 0.0);
        let m3 = Matrix3::new(f(8), f(0), f(0), f(0), f(8), f(0), f(0), f(0), f(8));
        let inverse = m3.checked_inverse_with(mode).unwrap();
        close((m3 * inverse).col[2].z, 1.0, 0.0);
        let m4 = transforms::scale(Vector3::new(f(8), f(8), f(8)));
        let inverse = m4.checked_inverse_with(mode).unwrap();
        close((m4 * inverse).col[2].z, 1.0, 0.0);
    }
    assert_eq!(
        m2.inverse().col[0].x,
        m2.inverse_with(InverseMode::Reciprocal).col[0].x
    );
    let singular = Matrix2::new(f(1), f(1), f(1), f(1));
    for mode in [InverseMode::Reciprocal, InverseMode::Rows] {
        assert!(matches!(
            singular.checked_inverse_with(mode),
            Err(MathError::Degenerate)
        ));
    }
    // Row mode deliberately offers a different intermediate-range tradeoff.
    type Narrow = Fixed<i16, 8>;
    let eight = Narrow::try_from_int(8).unwrap();
    let zero = Narrow::ZERO;
    let matrix = Matrix3::new(eight, zero, zero, zero, eight, zero, zero, zero, eight);
    assert!(matches!(matrix.checked_inverse(), Err(MathError::Overflow)));
    assert!(matrix.checked_inverse_with(InverseMode::Rows).is_ok());
}

#[test]
fn matrix_products_and_casts_preserve_raw_precision() {
    let value = F::from_bits(8);
    let m = Matrix4::new(
        value, value, value, value, value, value, value, value, value, value, value, value, value,
        value, value, value,
    );
    let product = Matrix4::checked_mul_matrix_matrix(&m, &m).unwrap();
    assert_eq!(product.col[0].x, F::DELTA);
    type Wide = Fixed<i64, 8>;
    let value = Wide::from_bits((1i64 << 60) + 1);
    let vector = Vector3::new(value, Wide::ZERO, Wide::ONE);
    assert_eq!(
        vector.try_cast::<Wide>().unwrap().x.to_bits(),
        value.to_bits()
    );
    let matrix = Matrix2::new(value, Wide::ZERO, Wide::ZERO, Wide::ONE);
    assert_eq!(
        matrix.try_cast::<Wide>().unwrap().col[0].x.to_bits(),
        value.to_bits()
    );
    let decimal = Vector3::new(1.5f64, -0.25, 2.0).try_cast::<F>().unwrap();
    assert_eq!(decimal.x.to_bits(), 384);
    assert_eq!(decimal.y.to_bits(), -64);
}

#[test]
fn transforms_quaternions_and_affine_decomposition() {
    let zero = f(0);
    let one = f(1);
    let axis = Vector3::new(zero, zero, one);
    let angle: F = core::f64::consts::FRAC_PI_2.into();
    let q = Quat::checked_of_axis_angle(&axis, angle, F::DELTA)
        .ok()
        .unwrap();
    let rotated = q.mat3() * Vector3::new(one, zero, zero);
    close(rotated.x, 0.0, 4.0);
    close(rotated.y, 1.0, 4.0);
    let doubled = f(2) * q;
    close(doubled.w, q.w.to_f64() * 2.0, 0.0);
    let translation = transforms::translate(Vector3::new(f(2), f(3), f(4)));
    let scale = transforms::scale(Vector3::new(f(2), f(4), f(8)));
    let matrix = translation * scale;
    let point = Vector3::new(one, one, one);
    let result = transforms::checked_transform_vec3(&matrix, &point).unwrap();
    assert_eq!((result.x, result.y, result.z), (f(4), f(7), f(12)));
    let inverse = matrix
        .checked_inverse_affine_with(InverseMode::Rows)
        .unwrap();
    let restored = transforms::checked_transform_vec3(&inverse, &result).unwrap();
    assert_eq!((restored.x, restored.y, restored.z), (one, one, one));
    let parts = transforms::checked_decompose_affine(&matrix).ok().unwrap();
    assert_eq!(
        (parts.scale.x, parts.scale.y, parts.scale.z),
        (f(2), f(4), f(8))
    );
    assert_eq!(parts.translation.x, f(2));
    let basis = Basis::of_mat4(&matrix);
    assert_eq!(basis.to_mat4().col[3].y, f(3));
    let orthogonal = checked_basis_from_unit_rh(&axis, F::DELTA).ok().unwrap();
    close(Vector3::dot(&orthogonal[0], &orthogonal[1]), 0.0, 1.0);
    // S2.61 can represent an identity quaternion but cannot represent trace 4.
    type Precision = Fixed<i64, 61>;
    assert_eq!(
        Quat::of_matrix3(&Matrix3::<Precision>::identity()).w,
        Precision::ONE
    );
}

#[test]
fn checked_projection_and_camera_domains() {
    let projection =
        transforms::checked_perspective(F::from(1.0f32), f(1), f(1), f(10000)).unwrap();
    close(projection.col[3].z, -2.0, 2.0);
    let eye = Vector3::new(f(0), f(0), f(2));
    let target = Vector3::new(f(0), f(0), f(0));
    let up = Vector3::new(f(0), f(1), f(0));
    let view = transforms::checked_lookat(&eye, &target, &up).unwrap();
    close(view.col[3].z, -2.0, 0.0);
    assert!(matches!(
        transforms::checked_lookat(&eye, &eye, &up),
        Err(MathError::Degenerate)
    ));
    assert!(matches!(
        transforms::checked_perspective(f(1), f(0), f(1), f(2)),
        Err(MathError::Domain)
    ));
    let ortho = transforms::checked_ortho4(f(-1), f(1), f(-1), f(1), f(1), f(3)).unwrap();
    let lo = Vector2::new(f(0), f(0));
    let hi = Vector2::new(f(100), f(100));
    let identity = Matrix4::identity();
    let point = Vector3::new(f(0), f(0), f(-2));
    let screen = transforms::checked_project3(&identity, &ortho, &lo, &hi, &point).unwrap();
    let world = transforms::checked_unproject3(&identity, &ortho, &lo, &hi, &screen).unwrap();
    assert_eq!((world.x, world.y, world.z), (point.x, point.y, point.z));
    assert!(transforms::checked_frustum(
        &Vector3::new(f(-1), f(-1), f(1)),
        &Vector3::new(f(1), f(1), f(10))
    )
    .is_ok());
    let mut invalid = Matrix4::<F>::identity();
    invalid.col[0].w = F::min_value();
    assert!(matches!(
        invalid.checked_inverse_affine(),
        Err(MathError::Domain)
    ));
    assert!(transforms::checked_decompose_affine(&invalid)
        .ok()
        .is_none());
    invalid.col[0].w = f(0);
    invalid.col[3].w = f(0);
    assert!(matches!(
        transforms::checked_transform_vec3(&invalid, &point),
        Err(MathError::DivisionByZero)
    ));
}

#[test]
fn primitive_construction_coordinates_and_distance() {
    let dimension = Dimension::new(f(2), f(3));
    assert_eq!(dimension.width, f(2));
    let rect = Rect::new(f(0), f(0), f(2), f(3));
    assert!(rect.contains(&Vector2::new(f(1), f(1))));
    let zero = Vector3::new(f(0), f(0), f(0));
    let x = Vector3::new(f(1), f(0), f(0));
    let y = Vector3::new(f(0), f(1), f(0));
    let triangle = Tri3::new([zero, x, y]);
    let quarter = F::try_from_ratio(1, 4).unwrap();
    let barycentric = triangle
        .checked_barycentric_coordinates(&Vector3::new(quarter, quarter, f(0)))
        .unwrap();
    assert_eq!(
        (barycentric.x, barycentric.y, barycentric.z),
        (F::try_from_ratio(1, 2).unwrap(), quarter, quarter)
    );
    let plane = Plane::new(&Vector3::new(f(0), f(0), f(1)), &zero);
    assert_eq!(
        plane
            .checked_distance(&Vector3::new(f(0), f(0), f(3)))
            .ok()
            .unwrap(),
        f(3)
    );
    let parametrized = ParametricPlane::new(&zero, &x, &Vector3::new(f(1), f(1), f(0)));
    let coordinates = parametrized
        .checked_project(&Vector3::new(f(2), f(3), f(0)))
        .unwrap();
    // The scaled Gram solve quantizes several intermediates. Directed native
    // arithmetic no longer gets nearest-even's accidental cancellation here;
    // allow eight S23.8 quanta against the independent exact coordinates.
    close(coordinates.x, -1.0, 8.0);
    close(coordinates.y, 3.0, 8.0);
    let segment = Segment::new(&zero, &x);
    assert_eq!(
        segment
            .checked_distance(&Vector3::new(f(0), f(2), f(0)))
            .ok()
            .unwrap(),
        f(2)
    );
    let tiny = Vector3::new(F::DELTA, f(0), f(0));
    let tiny_line = Line::checked_new(&zero, &tiny, F::ZERO).ok().unwrap();
    assert_eq!(tiny_line.checked_normalize(F::ZERO).unwrap().d.x, F::ONE);
    assert!(Matrix3::checked_of_axis_angle(&tiny, F::ONE, F::ZERO)
        .ok()
        .is_some());
    let normalized = tiny
        .checked_normalize_with_inv_len(F::ZERO, f(256), F::ZERO)
        .unwrap();
    assert_eq!(normalized.x, F::ONE);
    assert_eq!(
        Ray::checked_new(&zero, &tiny, F::ZERO)
            .ok()
            .unwrap()
            .direction
            .x,
        F::ONE
    );
    let bounds = Box3::new(&zero, &Vector3::new(f(2), f(2), f(2)));
    assert_eq!(bounds.center().x, f(1));
    assert_eq!(bounds.subdivide().len(), 8);
    let sphere = Sphere3::new(zero, f(1));
    assert!(bounds.checked_intersect(&sphere).unwrap());
    assert!(sphere.checked_intersect(&triangle).unwrap());
}

#[test]
fn checked_queries_distinguish_misses_and_range_errors() {
    let triangle = Tri3::new([
        Vector3::new(f(0), f(0), f(0)),
        Vector3::new(f(1), f(0), f(0)),
        Vector3::new(f(0), f(1), f(0)),
    ]);
    let quarter = F::try_from_ratio(1, 4).unwrap();
    let start = Vector3::new(quarter, quarter, f(1));
    let mut ray = Ray::checked_new(&start, &Vector3::new(f(0), f(0), f(-1)), F::DELTA)
        .ok()
        .unwrap();
    let hit = ray.checked_intersection(&triangle).unwrap().unwrap();
    assert_eq!(hit.0, f(1));
    assert_eq!(hit.1.z, f(0));
    ray.direction.z = f(1);
    assert!(ray.checked_intersection(&triangle).unwrap().is_none());
    ray.direction.z = f(0);
    assert!(matches!(
        ray.checked_intersection(&triangle),
        Err(MathError::Degenerate)
    ));
    let origin = Vector3::new(f(0), f(0), f(0));
    let mut ray = Ray::checked_new(&origin, &Vector3::new(f(1), f(0), f(0)), F::DELTA)
        .ok()
        .unwrap();
    let bounds = Box3::new(
        &Vector3::new(f(1), f(-1), f(-1)),
        &Vector3::new(f(2), f(1), f(1)),
    );
    assert!(bounds.checked_intersect(&ray).unwrap());
    ray.start.y = f(2);
    assert!(!bounds.checked_intersect(&ray).unwrap());
    ray.start.y = f(0);
    ray.direction.x = F::DELTA;
    let distant = Box3::new(
        &Vector3::new(F::max_value() - f(1), f(-1), f(-1)),
        &Vector3::new(F::max_value(), f(1), f(1)),
    );
    assert_eq!(distant.checked_intersect(&ray), Err(MathError::Overflow));
}

/// Exercises one identical consumer algorithm across multiple format families.
fn format_consumer<R: fixed::RealFormat<G>, const G: u32>() {
    let zero = Fixed::<R, G>::ZERO;
    let one = Fixed::<R, G>::ONE;
    let v = Vector3::new(
        Fixed::<R, G>::try_from_int(3).unwrap(),
        Fixed::<R, G>::try_from_int(4).unwrap(),
        zero,
    );
    assert_eq!(
        v.checked_length().unwrap(),
        Fixed::<R, G>::try_from_int(5).unwrap()
    );
    assert_eq!((Matrix3::<Fixed<R, G>>::identity() * v).x, v.x);
    assert_eq!(Quat::<Fixed<R, G>>::default().w, one);
}

#[test]
fn supported_scene_format_families() {
    format_consumer::<i32, 8>();
    format_consumer::<i32, 16>();
    format_consumer::<i64, 16>();
}

#[test]
fn fixed_line_parameters_and_subquantum_angular_squares() {
    let origin = Vector3::new(f(0), f(0), f(0));
    let normal = Vector3::new(f(0), f(0), f(1));
    let plane = Plane::new(&normal, &origin);
    let start = Vector3::new(f(0), f(0), f(2));
    let mut line = Line::checked_new(&start, &Vector3::new(f(0), f(0), f(-2)), F::DELTA)
        .ok()
        .unwrap();
    let first = plane
        .checked_intersect_line(&line, F::DELTA)
        .ok()
        .flatten()
        .unwrap();
    assert_eq!(first.0, f(1));
    assert_eq!(first.1.z, f(0));
    line.d = line.d * f(2);
    let second = plane
        .checked_intersect_line(&line, F::DELTA)
        .ok()
        .flatten()
        .unwrap();
    assert_eq!(second.0, F::try_from_ratio(1, 2).unwrap());
    assert_eq!(second.1.z, first.1.z);
    let x = Vector3::new(f(1), f(0), f(0));
    let near_x = Vector3::new(f(1), F::DELTA, f(0));
    assert_eq!(F::DELTA * F::DELTA, F::ZERO);
    let unit = checked_tri_normal(&origin, &x, &near_x, F::ZERO)
        .ok()
        .unwrap();
    assert_eq!(unit.z, F::ONE);
    assert!(checked_tri_normal(&origin, &x, &near_x, F::DELTA)
        .ok()
        .is_none());
    let tiny_x = Vector3::new(F::DELTA, f(0), f(0));
    let tiny_y = Vector3::new(f(0), F::DELTA, f(0));
    assert_eq!(
        checked_tri_normal(&origin, &tiny_x, &tiny_y, F::ZERO)
            .ok()
            .unwrap()
            .z,
        F::ONE
    );
    let line0 = Line::checked_new(&origin, &x, F::DELTA).ok().unwrap();
    let line1 = Line::checked_new(&normal, &Vector3::new(f(0), f(1), f(0)), F::DELTA)
        .ok()
        .unwrap();
    let closest = checked_shortest_segment3d_between_lines3d(&line0, &line1, F::DELTA)
        .ok()
        .flatten()
        .unwrap();
    assert_eq!(closest.e.z - closest.s.z, f(1));
}

#[test]
fn guarded_tangent_does_not_create_a_quantized_cosine_pole() {
    let angle = F::try_from_f64(core::f64::consts::FRAC_PI_2).unwrap();
    assert_eq!(angle.checked_cos().unwrap(), F::ZERO);
    assert!(angle.checked_tan().unwrap() > f(1000));
}
