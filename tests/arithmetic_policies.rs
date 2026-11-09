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

//! Contract regressions for explicit arithmetic policies and their call graphs.

use core::ops::*;
use num_traits::{Num, One, Zero};
use rs_math3d::*;

/// Scalar whose checked capabilities panic. A successful ordinary operation
/// therefore proves it never calls checked arithmetic or finite validation.
#[derive(Clone, Copy, Debug, Default, PartialEq, PartialOrd)]
struct Unchecked(f64);

macro_rules! arithmetic {
    ($($op:ident,$method:ident,$assign:ident,$assign_method:ident,$token:tt);*) => {$(
        impl $op for Unchecked { type Output=Self; fn $method(self,r:Self)->Self {Self(self.0 $token r.0)} }
        impl $assign for Unchecked { fn $assign_method(&mut self,r:Self) {*self=(*self).$method(r);} }
    )*};
}
arithmetic!(Add,add,AddAssign,add_assign,+; Sub,sub,SubAssign,sub_assign,-;
    Mul,mul,MulAssign,mul_assign,*; Div,div,DivAssign,div_assign,/; Rem,rem,RemAssign,rem_assign,%);
impl Neg for Unchecked {
    type Output = Self;
    fn neg(self) -> Self {
        Self(-self.0)
    }
}
impl Zero for Unchecked {
    fn zero() -> Self {
        Self(0.0)
    }
    fn is_zero(&self) -> bool {
        self.0 == 0.0
    }
}
impl One for Unchecked {
    fn one() -> Self {
        Self(1.0)
    }
}
impl Num for Unchecked {
    type FromStrRadixErr = <f64 as Num>::FromStrRadixErr;
    fn from_str_radix(s: &str, r: u32) -> Result<Self, Self::FromStrRadixErr> {
        f64::from_str_radix(s, r).map(Self)
    }
}
impl Scalar for Unchecked {
    fn checked_add(self, _: Self) -> Result<Self, MathError> {
        panic!("checked add")
    }
    fn checked_sub(self, _: Self) -> Result<Self, MathError> {
        panic!("checked sub")
    }
    fn checked_mul(self, _: Self) -> Result<Self, MathError> {
        panic!("checked mul")
    }
    fn checked_div(self, _: Self) -> Result<Self, MathError> {
        panic!("checked div")
    }
    fn checked_rem(self, _: Self) -> Result<Self, MathError> {
        panic!("checked rem")
    }
    fn checked_neg(self) -> Result<Self, MathError> {
        panic!("checked neg")
    }
    fn checked_abs(self) -> Result<Self, MathError> {
        panic!("checked abs")
    }
    fn is_finite(self) -> bool {
        panic!("finite validation")
    }
    fn two() -> Self {
        Self(2.0)
    }
    fn min(a: Self, b: Self) -> Self {
        if a < b {
            a
        } else {
            b
        }
    }
    fn max(a: Self, b: Self) -> Self {
        if a > b {
            a
        } else {
            b
        }
    }
    fn tabs(self) -> Self {
        Self(self.0.abs())
    }
}
impl RealScalar for Unchecked {
    type Reciprocal = Self;
    fn reciprocal(self) -> Self {
        Self(1.0 / self.0)
    }
    fn mul_reciprocal(self, r: Self) -> Self {
        self * r
    }
    fn checked_reciprocal(self) -> Result<Self, MathError> {
        panic!("checked reciprocal")
    }
    fn checked_mul_reciprocal(self, _: Self) -> Result<Self, MathError> {
        panic!("checked reciprocal multiplication")
    }
    fn epsilon() -> Self {
        Self(1e-9)
    }
    fn half() -> Self {
        Self(0.5)
    }
    fn quarter() -> Self {
        Self(0.25)
    }
    fn tsqrt(self) -> Self {
        Self(self.0.sqrt())
    }
    fn tsin(self) -> Self {
        Self(self.0.sin())
    }
    fn tcos(self) -> Self {
        Self(self.0.cos())
    }
    fn ttan(self) -> Self {
        Self(self.0.tan())
    }
    fn tacos(self) -> Self {
        Self(self.0.acos())
    }
}

#[test]
fn ordinary_vectors_matrices_and_quaternions_never_validate() {
    let a = Vector3::from_array([1.0, 2.0, 3.0]).map(Unchecked);
    let b = Vector3::from_array([3.0, 1.0, 2.0]).map(Unchecked);
    let _ = (
        a + b,
        a - b,
        a * b,
        a / b,
        a % b,
        -a,
        a * Unchecked(2.0),
        a / Unchecked(2.0),
    );
    let _ = (
        Vector3::dot(&a, &b),
        Vector3::cross(&a, &b),
        a.length(),
        a.normalize(),
        Vector3::distance(&a, &b),
        a.max_abs_component(),
        Unchecked(2.0).clamp(Unchecked(0.0), Unchecked(1.0)),
    );
    let m2 = Matrix2::new(
        Unchecked(2.0),
        Unchecked(1.0),
        Unchecked(1.0),
        Unchecked(3.0),
    );
    let m3 = Matrix3::new(
        Unchecked(2.0),
        Unchecked(1.0),
        Unchecked(0.0),
        Unchecked(0.0),
        Unchecked(3.0),
        Unchecked(1.0),
        Unchecked(1.0),
        Unchecked(0.0),
        Unchecked(4.0),
    );
    let m4 = transforms::translate(a) * transforms::scale(b);
    for mode in [InverseMode::Reciprocal, InverseMode::Rows] {
        let p = m2 * m2.inverse_with(mode);
        assert!((p.col[0].x.0 - 1.0).abs() < 1e-10);
        let p = m3 * m3.inverse_with(mode);
        assert!((p.col[1].y.0 - 1.0).abs() < 1e-10);
        let p = m4 * m4.inverse_with(mode);
        assert!((p.col[2].z.0 - 1.0).abs() < 1e-10);
        let p = m4 * m4.inverse_affine_with(mode);
        assert!((p.col[3].w.0 - 1.0).abs() < 1e-10);
    }
    let _ = (
        m2.inverse(),
        m3.inverse(),
        m4.inverse(),
        m4.inverse_affine(),
        m3.determinant(),
        m4.determinant(),
        m4 + m4,
        m4 - m4,
        m4 * Vector4::new(a.x, a.y, a.z, Unchecked(1.0)),
    );
    let q = Quat::of_axis_angle(&a, Unchecked(0.7));
    let _ = (
        q + q,
        q - q,
        q * q,
        q * Unchecked(2.0),
        q / Unchecked(2.0),
        Quat::inverse(&q),
        Quat::normalize(&q),
        q.mat3(),
        q.mat4(),
        q.to_axis_angle(),
        Quat::of_matrix3(&q.mat3()),
    );
}

#[test]
fn ordinary_transforms_and_queries_never_validate() {
    let zero = Vector3::from_array([0.0, 0.0, 0.0]).map(Unchecked);
    let x = Vector3::from_array([1.0, 0.0, 0.0]).map(Unchecked);
    let y = Vector3::from_array([0.0, 1.0, 0.0]).map(Unchecked);
    let z = Vector3::from_array([0.0, 0.0, 1.0]).map(Unchecked);
    let m = transforms::translate(Vector3::from_array([1.0, 2.0, 3.0]).map(Unchecked));
    let p = transforms::perspective(
        Unchecked(0.8),
        Unchecked(1.5),
        Unchecked(1.0),
        Unchecked(10.0),
    );
    let _ = transforms::frustum(
        &Vector3::from_array([-1.0, -1.0, 1.0]).map(Unchecked),
        &Vector3::from_array([1.0, 1.0, 10.0]).map(Unchecked),
    );
    let _ = transforms::ortho4(
        Unchecked(-1.0),
        Unchecked(1.0),
        Unchecked(-1.0),
        Unchecked(1.0),
        Unchecked(1.0),
        Unchecked(10.0),
    );
    let _ = transforms::lookat(&z, &zero, &y);
    let _ = transforms::rotation_from_axis_angle(&x, Unchecked(0.5));
    let _ = transforms::rotation_from_quat(&Quat::<Unchecked>::identity());
    let lb = Vector2::new(Unchecked(0.0), Unchecked(0.0));
    let rt = Vector2::new(Unchecked(2.0), Unchecked(2.0));
    let screen = transforms::project3(&m, &p, &lb, &rt, &z);
    let _ = transforms::unproject3(&m, &p, &lb, &rt, &screen);
    let _ = (transforms::decompose_affine(&m), transforms::decompose(&m));
    let line = Line::new(&z, &(-z));
    let ray = Ray::new(&Vector3::from_array([0.2, 0.2, 1.0]).map(Unchecked), &(-z));
    let segment = Segment::new(&zero, &x);
    let plane = Plane::new(&z, &zero);
    let _ = (
        line.normalize(),
        Line::from_start_end(&zero, &x),
        line.closest_point_on_line(&x),
        segment.closest_point_on_segment(&y),
        segment.distance(&y),
        plane.distance(&z),
    );
    let eps = Unchecked(1e-9);
    assert!(plane.intersect_ray(&ray, eps).is_some());
    assert!(ray.intersect_plane(&plane, eps).is_some());
    assert!(plane.intersect_line(&line, eps).is_some());
    let tri = Tri3::new([zero, x, y]);
    assert!(ray.intersection(&tri).is_some());
    assert!(tri.intersection(&ray).is_some());
    assert!(line.intersection(&tri).is_some());
    assert!(tri.intersection(&line).is_some());
    let sphere = Sphere3::new(
        Vector3::from_array([0.25, 0.25, 0.25]).map(Unchecked),
        Unchecked(0.5),
    );
    assert!(sphere.intersect(&tri));
    let box3 = Box3::new(&(-x - y - z), &(x + y + z));
    assert!(box3.intersect(&sphere));
    assert!(box3.intersect(&ray));
    let _ = (box3.center(), box3.extent(), box3.subdivide());
    let pp = ParametricPlane::new(&zero, &x, &y);
    let _ = (
        pp.normal(),
        pp.plane(),
        pp.project(&x),
        pp.intersect_ray(&ray, eps),
        pp.intersect_line(&line, eps),
    );
    let _ = (
        tri_normal(&zero, &x, &y),
        quad_normal(&zero, &x, &(x + y), &y),
        Plane::from_tri(&zero, &x, &y),
        Plane::from_quad(&zero, &x, &(x + y), &y),
    );
    let _ = (
        basis_from_unit_rh(&z),
        basis_from_unit_lh(&z),
        shortest_segment3d_between_lines3d(&line, &Line::new(&x, &y), eps),
    );
}

#[test]
fn checked_fixed_vector_matrix_and_geometry_failures_are_explicit() {
    type F = Fixed<i8, 4>;
    let max = F::max_value();
    let one = F::ONE;
    let zero = F::ZERO;
    let a = Vector3::new(max, one, one);
    let b = Vector3::new(one, one, one);
    assert!(matches!(
        Vector3::checked_add_vv(&a, &b),
        Err(MathError::Overflow)
    ));
    assert!(matches!(
        Vector3::checked_mul_vv(&a, &a),
        Err(MathError::Overflow)
    ));
    assert!(matches!(
        Vector3::checked_div_vv(&a, &Vector3::zero()),
        Err(MathError::DivisionByZero)
    ));
    assert!(matches!(
        Vector3::checked_rem_vs(&a, zero),
        Err(MathError::DivisionByZero)
    ));
    assert!(matches!(
        Vector3::new(F::min_value(), zero, zero).checked_neg(),
        Err(MathError::Overflow)
    ));
    assert_eq!(
        (a + b).x.to_bits(),
        max.to_bits().wrapping_add(one.to_bits())
    );
    let m = Matrix2::new(max, zero, zero, max);
    assert!(matches!(m.checked_inverse(), Err(MathError::Overflow)));
    assert!(matches!(
        Matrix2::checked_mul_matrix_matrix(&m, &m),
        Err(MathError::Overflow)
    ));
    let singular = Matrix2::new(one, one, one, one);
    for mode in [InverseMode::Reciprocal, InverseMode::Rows] {
        assert!(matches!(
            singular.checked_inverse_with(mode),
            Err(MathError::Degenerate)
        ));
    }
    assert!(matches!(
        Vector3::<F>::zero().checked_normalize(zero),
        Err(MathError::Degenerate)
    ));
    assert!(matches!(
        Sphere3::checked_new(b, F::min_value()),
        Err(MathError::Overflow)
    ));
    assert!(matches!(
        Ray::checked_new(&b, &Vector3::<F>::zero(), zero),
        Err(MathError::Degenerate)
    ));
    assert!(matches!(
        transforms::checked_ortho4(zero, zero, zero, one, zero, one),
        Err(MathError::DivisionByZero)
    ));
}

#[cfg(any(feature = "std", feature = "libm", feature = "system-libm"))]
#[test]
fn ieee_defaults_and_checked_input_errors_are_distinct() {
    let zero = Vector3::<f32>::zero();
    assert!(zero.normalize().x.is_nan());
    assert!(matches!(
        zero.checked_normalize(0.0),
        Err(MathError::Degenerate)
    ));
    let singular = Matrix3::<f32>::new(0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0);
    assert!(!singular.inverse().col[0].x.is_finite());
    assert!(matches!(
        singular.checked_inverse(),
        Err(MathError::Degenerate)
    ));
    let nan = Vector3::new(f32::NAN, 0.0, 0.0);
    let x = Vector3::new(1.0, 0.0, 0.0);
    assert!(matches!(
        Line::checked_new(&nan, &x, 0.0),
        Err(MathError::NonFinite)
    ));
    assert!(matches!(
        Ray::checked_new(&zero, &x, -1.0),
        Err(MathError::InvalidTolerance)
    ));
    assert!(matches!(
        transforms::checked_translate(nan),
        Err(MathError::NonFinite)
    ));
    assert!(Quat::normalize(&Quat::<f32>::new(0.0, 0.0, 0.0, 0.0))
        .x
        .is_nan());
    assert!(matches!(
        Quat::checked_normalize(&Quat::<f32>::new(0.0, 0.0, 0.0, 0.0), 0.0),
        Err(MathError::Degenerate)
    ));
    let plane = Plane::new(&Vector3::new(0.0, 0.0, 1.0), &zero);
    let ray = Ray::new(&Vector3::new(0.0, 0.0, 1.0), &x);
    assert!(plane.intersect_ray(&ray, 0.0).is_none());
    assert!(matches!(plane.checked_intersect_ray(&ray, 0.0), Ok(None)));
    assert!(matches!(
        plane.checked_intersect_ray(&ray, -1.0),
        Err(MathError::InvalidTolerance)
    ));
}

/// Exercises off-diagonal minors, reflections and row pivoting, while comparing
/// ordinary/checked rounding and an independent matrix-times-inverse identity.
#[test]
fn matrix_inverse_policies_agree_on_representable_dense_inputs() {
    type F = Fixed<i64, 16>;
    let mut seed = 0x9e3779b9u32;
    for _ in 0..64 {
        let mut next = || {
            seed = seed.wrapping_mul(1664525).wrapping_add(1013904223);
            F::try_from_ratio(((seed >> 16) % 17) as i64 - 8, 16).unwrap()
        };
        let mut m = Matrix4::new(
            next(),
            next(),
            next(),
            next(),
            next(),
            next(),
            next(),
            next(),
            next(),
            next(),
            next(),
            next(),
            next(),
            next(),
            next(),
            next(),
        );
        m.col[0].x += F::try_from_int(2).unwrap();
        m.col[1].y += F::try_from_int(2).unwrap();
        m.col[2].z += F::try_from_int(2).unwrap();
        m.col[3].w += F::try_from_int(2).unwrap();
        for mode in [InverseMode::Reciprocal, InverseMode::Rows] {
            let raw = m.inverse_with(mode);
            let checked = m.checked_inverse_with(mode).unwrap();
            let identity = m * raw;
            for col in 0..4 {
                let r = raw.col[col];
                let c = checked.col[col];
                assert_eq!((r.x, r.y, r.z, r.w), (c.x, c.y, c.z, c.w));
                let v = identity.col[col];
                for (row, value) in [v.x, v.y, v.z, v.w].iter().enumerate() {
                    let expected = if row == col { F::ONE } else { F::ZERO };
                    assert!((*value - expected).abs().to_bits() <= 16);
                }
            }
        }
    }
}

#[cfg(any(feature = "std", feature = "libm", feature = "system-libm"))]
#[test]
fn checked_geometric_misses_do_not_hide_invalid_inputs() {
    let zero = Vector3::<f64>::zero();
    let z = Vector3::new(0.0, 0.0, 1.0);
    let x = Vector3::new(1.0, 0.0, 0.0);
    let plane = Plane::new(&z, &zero);
    let away = Ray::new(&z, &z);
    assert!(matches!(
        away.checked_intersect_plane(&plane, 0.0),
        Ok(None)
    ));
    let mut invalid = Ray::new(&z, &x);
    invalid.start.x = f64::NAN;
    assert!(matches!(
        invalid.checked_intersect_plane(&plane, 0.0),
        Err(MathError::NonFinite)
    ));
    let mut sphere = Sphere3::new(z, 1.0);
    sphere.radius = f64::NAN;
    assert!(matches!(
        Box3::new(&zero, &z).checked_intersect(&sphere),
        Err(MathError::NonFinite)
    ));
    let tri = Tri3::new([zero, x, z]);
    assert!(matches!(
        sphere.checked_intersect(&tri),
        Err(MathError::NonFinite)
    ));
}
