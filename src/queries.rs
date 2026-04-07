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
//! Intersection and distance query traits and implementations.
//!
//! Boolean intersection tests are boundary-inclusive unless stated otherwise:
//! touching counts as intersecting.
//!
//! The `Box3`/`Ray` query uses a slab-interval test. For ray directions that are
//! parallel to a slab plane, the implementation checks whether the ray origin is
//! already inside that slab instead of relying on IEEE `0/0` behaviour.

use crate::primitives::*;
use crate::scalar::*;
use crate::vector::*;
use num_traits::{One, Zero};

////////////////////////////////////////////////////////////////////////////////
/// Query Traits
////////////////////////////////////////////////////////////////////////////////
pub trait Distance<T, Other> {
    /// Computes the distance to another value, or `None` if undefined.
    fn distance(&self, other: &Other) -> Option<T>;
}

/// Trait for boolean intersection tests.
pub trait Intersect<T> {
    /// Returns `true` when two objects intersect.
    ///
    /// Unless an implementation documents otherwise, touching at the boundary
    /// counts as an intersection.
    fn intersect(&self, other: &T) -> bool;
}

/// Trait for producing intersection results.
pub trait Intersection<T, Other> {
    /// Returns the intersection result, or `None` if there is no intersection.
    fn intersection(&self, other: &Other) -> Option<T>;
}

////////////////////////////////////////////////////////////////////////////////
/// Distance Queries
////////////////////////////////////////////////////////////////////////////////
impl<T: FloatScalar> Distance<T, Vector3<T>> for Plane<T> {
    fn distance(&self, other: &Vector3<T>) -> Option<T> {
        let n = self.normal();
        let nom = Vector3::dot(other, &n) + self.constant();
        let denom = Vector3::dot(&n, &n);
        if denom <= T::epsilon() {
            return None;
        }
        Some(T::tabs(nom) / denom)
    }
}

impl<T: FloatScalar> Distance<T, Vector3<T>> for Segment<T, Vector3<T>> {
    fn distance(&self, other: &Vector3<T>) -> Option<T> {
        Segment::distance(self, other, T::epsilon())
    }
}

////////////////////////////////////////////////////////////////////////////////
/// Intersect Queries
////////////////////////////////////////////////////////////////////////////////
// Updates the running `[tmin, tmax]` interval for one axis-aligned slab.
//
// For nonzero directions this is the usual slab intersection update:
// intersect the current interval with the parameter interval where the ray lies
// between `slab_min` and `slab_max`.
//
// For zero directions the ray is parallel to the slab planes. In that case there
// is no finite entry/exit parameter, so the query reduces to whether the origin
// already lies inside the slab.
fn update_slab_interval<T: FloatScalar>(
    slab_min: T,
    slab_max: T,
    ray_start: T,
    ray_dir: T,
    tmin: &mut T,
    tmax: &mut T,
) -> bool {
    if ray_dir == <T as Zero>::zero() {
        return ray_start >= slab_min && ray_start <= slab_max;
    }

    let inv_dir = <T as One>::one() / ray_dir;
    let mut t0 = (slab_min - ray_start) * inv_dir;
    let mut t1 = (slab_max - ray_start) * inv_dir;
    if t0 > t1 {
        core::mem::swap(&mut t0, &mut t1);
    }

    *tmin = T::max(*tmin, t0);
    *tmax = T::min(*tmax, t1);
    *tmin <= *tmax
}

impl<T: FloatScalar> Intersect<Ray<T, Vector3<T>>> for Box3<T> {
    fn intersect(&self, other: &Ray<T, Vector3<T>>) -> bool {
        // Slab test for an axis-aligned box.
        //
        // The running interval `[tmin, tmax]` contains all ray parameters `t`
        // for which the point `other.start + t * other.direction` is inside the
        // slabs processed so far.
        //
        // This is in the same family as the Williams et al. ray-box test, but we
        // handle `direction == 0` explicitly instead of depending on direct
        // division for every axis. That avoids `0/0 -> NaN` when a ray starts on
        // a slab boundary and is parallel to that axis.
        let mut tmin = -T::infinity();
        let mut tmax = T::infinity();

        if !update_slab_interval(
            self.min.x,
            self.max.x,
            other.start.x,
            other.direction.x,
            &mut tmin,
            &mut tmax,
        ) {
            return false;
        }
        if !update_slab_interval(
            self.min.y,
            self.max.y,
            other.start.y,
            other.direction.y,
            &mut tmin,
            &mut tmax,
        ) {
            return false;
        }
        if !update_slab_interval(
            self.min.z,
            self.max.z,
            other.start.z,
            other.direction.z,
            &mut tmin,
            &mut tmax,
        ) {
            return false;
        }

        // Intersect the final slab interval with the forward-ray constraint
        // `t >= 0`. Touching the box boundary counts as an intersection.
        tmax >= T::max(tmin, <T as Zero>::zero())
    }
}

impl<T: FloatScalar> Intersect<Sphere3<T>> for Box3<T> {
    fn intersect(&self, other: &Sphere3<T>) -> bool {
        let mut dist = <T as Zero>::zero();

        let c = other.center;
        let r = other.radius;

        if c.x < self.min.x {
            dist += T::squared(c.x - self.min.x)
        } else if c.x > self.max.x {
            dist += T::squared(c.x - self.max.x)
        }

        if c.y < self.min.y {
            dist += T::squared(c.y - self.min.y)
        } else if c.y > self.max.y {
            dist += T::squared(c.y - self.max.y)
        }

        if c.z < self.min.z {
            dist += T::squared(c.z - self.min.z)
        } else if c.z > self.max.z {
            dist += T::squared(c.z - self.max.z)
        }

        r * r >= dist
    }
}

fn is_in_0_to_1_range<T: FloatScalar>(x: T) -> bool {
    x >= <T as Zero>::zero() && x <= <T as One>::one()
}

impl<T: FloatScalar> Intersect<Tri3<T>> for Sphere3<T> {
    fn intersect(&self, tri: &Tri3<T>) -> bool {
        let uvw = tri.barycentric_coordinates(&self.center);
        let verts = tri.vertices();
        let v0 = verts[0];
        let v1 = verts[1];
        let v2 = verts[2];
        if is_in_0_to_1_range(uvw.x) && is_in_0_to_1_range(uvw.y) && is_in_0_to_1_range(uvw.z) {
            Plane::try_from_tri(&v0, &v1, &v2, T::epsilon()).is_some_and(|p| {
                p.distance(&self.center)
                    .is_some_and(|dist| dist <= self.radius)
            })
        } else {
            let d0 = Segment::new(&v0, &v1).distance(&self.center, T::epsilon());
            let d1 = Segment::new(&v1, &v2).distance(&self.center, T::epsilon());
            let d2 = Segment::new(&v2, &v0).distance(&self.center, T::epsilon());

            match (d0, d1, d2) {
                (Some(d0), Some(d1), Some(d2)) => {
                    let m = T::min(d0, T::min(d1, d2));
                    m <= self.radius
                }
                _ => false,
            }
        }
    }
}

///
/// Ray/Line-Triangle Intersection Test Routines
/// Different optimizations of my and Ben Trumbore's
/// code from journals of graphics tools (JGT)
/// <http://www.acm.org/jgt/>
/// by Tomas Moller, May 2000
///
enum TriangleIntersectionKind {
    Ray,
    Line,
}

fn triangle_intersection_from_point_dir<T: FloatScalar>(
    start: &Vector3<T>,
    direction: &Vector3<T>,
    tri: &Tri3<T>,
    epsilon: T,
    kind: TriangleIntersectionKind,
) -> Option<(T, Vector3<T>)> {
    let verts = tri.vertices();
    let v0 = verts[0];
    let v1 = verts[1];
    let v2 = verts[2];
    let edge1 = v1 - v0;
    let edge2 = v2 - v0;

    let pvec = Vector3::cross(direction, &edge2);
    let det = Vector3::dot(&edge1, &pvec);
    if det > -epsilon && det < epsilon {
        return None;
    }

    let tvec = *start - v0;
    let qvec = Vector3::cross(&tvec, &edge1);

    let u = Vector3::dot(&tvec, &pvec) / det;
    if u < -epsilon || u > <T as One>::one() + epsilon {
        return None;
    }

    let v = Vector3::dot(direction, &qvec) / det;
    if v < -epsilon || u + v > <T as One>::one() + T::two() * epsilon {
        return None;
    }

    let t = Vector3::dot(&edge2, &qvec) / det;
    if matches!(kind, TriangleIntersectionKind::Ray) && t < -epsilon {
        return None;
    }

    Some((t, *start + (*direction * t)))
}

impl<T: FloatScalar> Intersection<(T, Vector3<T>), Tri3<T>> for Ray<T, Vector3<T>> {
    fn intersection(&self, tri: &Tri3<T>) -> Option<(T, Vector3<T>)> {
        triangle_intersection_from_point_dir(
            &self.start,
            &self.direction,
            tri,
            T::epsilon(),
            TriangleIntersectionKind::Ray,
        )
    }
}

impl<T: FloatScalar> Intersection<(T, Vector3<T>), Tri3<T>> for Line<T, Vector3<T>> {
    fn intersection(&self, tri: &Tri3<T>) -> Option<(T, Vector3<T>)> {
        triangle_intersection_from_point_dir(
            &self.p,
            &self.d,
            tri,
            T::epsilon(),
            TriangleIntersectionKind::Line,
        )
    }
}

impl<T: FloatScalar> Intersection<(T, Vector3<T>), Ray<T, Vector3<T>>> for Tri3<T> {
    fn intersection(&self, ray: &Ray<T, Vector3<T>>) -> Option<(T, Vector3<T>)> {
        ray.intersection(self)
    }
}

impl<T: FloatScalar> Intersection<(T, Vector3<T>), Line<T, Vector3<T>>> for Tri3<T> {
    fn intersection(&self, line: &Line<T, Vector3<T>>) -> Option<(T, Vector3<T>)> {
        line.intersection(self)
    }
}

///
/// Building an Orthonormal Basis from a Unit Vector
/// John. F. Hughes & Thomas Moller
/// Journal of Graphics Tools, 4:4, 33-35 (DOI: 10.1080/10867651.1999.10487513)
///
/// This implementation uses a symmetric variant of the Hughes-Moller
/// construction: instead of branching on two specific components, it zeros the
/// smallest-magnitude component and normalizes the resulting orthogonal helper
/// vector. The proof sketch below applies to this variant.
///
/// Proof sketch for the branch construction:
///
/// Let `u = (x, y, z)` be the normalized input, so `||u|| = 1`.
/// We choose the helper vector by zeroing the smallest-magnitude component:
///
/// - if `|x| <= |y|` and `|x| <= |z|`, choose `v0 = (0, -z, y)`
/// - else if `|y| <= |x|` and `|y| <= |z|`, choose `v0 = (-z, 0, x)`
/// - else choose `v0 = (-y, x, 0)`
///
/// In each branch, `u · v0 = 0`, so `v0` is orthogonal to `u`.
/// The squared length is also bounded away from zero:
///
/// - first branch: `||v0||² = y² + z² = 1 - x²`
/// - second branch: `||v0||² = x² + z² = 1 - y²`
/// - third branch: `||v0||² = x² + y² = 1 - z²`
///
/// Because the selected component has minimal magnitude, its square is at most
/// `1/3`. Therefore `||v0||² >= 2/3` in every branch, so `v0` is never zero for
/// non-zero `u`, and normalizing it is always safe.
///
/// Let `v = normalize(v0)`. Then `u` and `v` are orthonormal. Finally,
/// `w = normalize(u × v)` is orthogonal to both and has unit length, so the
/// returned triple is orthonormal.
///
pub fn try_basis_from_unit<T: FloatScalar>(
    unit: &Vector3<T>,
    epsilon: T,
) -> Option<[Vector3<T>; 3]> {
    let u = unit.try_normalize(epsilon)?;
    let x = u.x;
    let y = u.y;
    let z = u.z;

    let v = if x.tabs() <= y.tabs() && x.tabs() <= z.tabs() {
        Vector3::new(<T as Zero>::zero(), -z, y)
    } else if y.tabs() <= x.tabs() && y.tabs() <= z.tabs() {
        Vector3::new(-z, <T as Zero>::zero(), x)
    } else {
        // z.abs() < x.abs() && z.abs() <= y.abs()
        Vector3::new(-y, x, <T as Zero>::zero())
    };
    let v = Vector3::normalize(&v);
    let w = Vector3::normalize(&Vector3::cross(&u, &v));
    Some([u, w, v])
}

/// Builds an orthonormal basis from a non-zero input direction.
///
/// # Panics
/// Panics if `unit` is too small to normalize. Use
/// [`try_basis_from_unit`] when the input may be degenerate.
pub fn basis_from_unit<T: FloatScalar>(unit: &Vector3<T>) -> [Vector3<T>; 3] {
    try_basis_from_unit(unit, T::epsilon()).expect("basis direction must be non-zero")
}

#[cfg(test)]
mod tests {
    use super::{Intersect, Intersection};
    use crate::primitives::{Box3, Line, Ray, Sphere3, Tri3};
    use crate::vector::{FloatVector, Vector, Vector3};
    use crate::EPS_F32;

    fn assert_orthonormal_basis(basis: [Vector3<f32>; 3]) {
        let [u, w, v] = basis;
        assert!((u.length() - 1.0).abs() < 0.001);
        assert!((v.length() - 1.0).abs() < 0.001);
        assert!((w.length() - 1.0).abs() < 0.001);
        assert!(Vector3::dot(&u, &v).abs() < 0.001);
        assert!(Vector3::dot(&u, &w).abs() < 0.001);
        assert!(Vector3::dot(&v, &w).abs() < 0.001);
    }

    #[test]
    fn test_box3_sphere_intersect_z_axis() {
        let b = Box3::new(
            &Vector3::new(0.0f32, 0.0, 0.0),
            &Vector3::new(1.0, 1.0, 1.0),
        );

        let outside_z = Sphere3::new(Vector3::new(0.5, 0.5, 2.0), 0.25);
        assert!(!b.intersect(&outside_z));

        let touching_z = Sphere3::new(Vector3::new(0.5, 0.5, 2.0), 1.0);
        assert!(b.intersect(&touching_z));
    }

    #[test]
    fn test_box3_sphere_intersect_on_edge() {
        let b = Box3::new(
            &Vector3::new(0.0f32, 0.0, 0.0),
            &Vector3::new(1.0, 1.0, 1.0),
        );

        let touching = Sphere3::new(Vector3::new(2.0, 0.5, 0.5), 1.0);
        assert!(b.intersect(&touching));

        let just_inside = Sphere3::new(Vector3::new(2.0, 0.5, 0.5), 1.0001);
        assert!(b.intersect(&just_inside));
    }

    #[test]
    fn test_box3_ray_intersect_touching_boundary() {
        let b = Box3::new(
            &Vector3::new(0.0f32, 0.0, 0.0),
            &Vector3::new(1.0, 1.0, 1.0),
        );
        let ray = Ray::new(
            &Vector3::new(1.0f32, 0.5, 0.5),
            &Vector3::new(1.0, 0.0, 0.0),
            EPS_F32,
        )
        .expect("ray should be valid");
        assert!(b.intersect(&ray));
    }

    #[test]
    fn test_box3_ray_intersect_parallel_on_face() {
        let b = Box3::new(
            &Vector3::new(0.0f32, 0.0, 0.0),
            &Vector3::new(1.0, 1.0, 1.0),
        );
        let ray = Ray::new(
            &Vector3::new(0.0f32, 0.5, 0.5),
            &Vector3::new(0.0, 1.0, 0.0),
            EPS_F32,
        )
        .expect("ray should be valid");
        assert!(b.intersect(&ray));
    }

    #[test]
    fn test_box3_ray_intersect_parallel_outside_slab() {
        let b = Box3::new(
            &Vector3::new(0.0f32, 0.0, 0.0),
            &Vector3::new(1.0, 1.0, 1.0),
        );
        let ray = Ray::new(
            &Vector3::new(2.0f32, 0.5, 0.5),
            &Vector3::new(0.0, 1.0, 0.0),
            EPS_F32,
        )
        .expect("ray should be valid");
        assert!(!b.intersect(&ray));
    }

    #[test]
    fn test_try_basis_from_unit_zero_none() {
        let zero = Vector3::new(0.0f32, 0.0, 0.0);
        assert!(super::try_basis_from_unit(&zero, EPS_F32).is_none());
    }

    #[test]
    fn test_try_basis_from_unit_orthonormal() {
        let basis0 = super::try_basis_from_unit(&Vector3::new(1.0f32, 2.0, 3.0), EPS_F32)
            .expect("basis should exist");
        let basis1 = super::try_basis_from_unit(&Vector3::new(0.1f32, 4.0, 5.0), EPS_F32)
            .expect("basis should exist");
        let basis2 = super::try_basis_from_unit(&Vector3::new(4.0f32, 0.1, 5.0), EPS_F32)
            .expect("basis should exist");
        let basis3 = super::try_basis_from_unit(&Vector3::new(4.0f32, 5.0, 0.1), EPS_F32)
            .expect("basis should exist");

        assert_orthonormal_basis(basis0);
        assert_orthonormal_basis(basis1);
        assert_orthonormal_basis(basis2);
        assert_orthonormal_basis(basis3);
    }

    #[test]
    fn test_try_basis_from_unit_axes_and_ties() {
        let basis_x =
            super::try_basis_from_unit(&Vector3::new(1.0f32, 0.0, 0.0), EPS_F32).expect("basis");
        let basis_y =
            super::try_basis_from_unit(&Vector3::new(0.0f32, 1.0, 0.0), EPS_F32).expect("basis");
        let basis_z =
            super::try_basis_from_unit(&Vector3::new(0.0f32, 0.0, 1.0), EPS_F32).expect("basis");
        let basis_neg =
            super::try_basis_from_unit(&Vector3::new(-1.0f32, -1.0, -1.0), EPS_F32).expect("basis");

        assert_orthonormal_basis(basis_x);
        assert_orthonormal_basis(basis_y);
        assert_orthonormal_basis(basis_z);
        assert_orthonormal_basis(basis_neg);
    }

    #[test]
    fn test_ray_triangle_intersection_rejects_hits_behind_ray() {
        let tri = Tri3::new([
            Vector3::new(0.0f32, 0.0, 0.0),
            Vector3::new(1.0, 0.0, 0.0),
            Vector3::new(0.0, 1.0, 0.0),
        ]);
        let ray = Ray::new(
            &Vector3::new(0.25f32, 0.25, 1.0),
            &Vector3::new(0.0, 0.0, 1.0),
            EPS_F32,
        )
        .expect("ray should be valid");

        assert!(ray.intersection(&tri).is_none());
        assert!(tri.intersection(&ray).is_none());
    }

    #[test]
    fn test_ray_triangle_intersection_forward_hit() {
        let tri = Tri3::new([
            Vector3::new(0.0f32, 0.0, 0.0),
            Vector3::new(1.0, 0.0, 0.0),
            Vector3::new(0.0, 1.0, 0.0),
        ]);
        let ray = Ray::new(
            &Vector3::new(0.25f32, 0.25, 1.0),
            &Vector3::new(0.0, 0.0, -1.0),
            EPS_F32,
        )
        .expect("ray should be valid");

        let (t0, p0) = ray.intersection(&tri).expect("ray should hit triangle");
        let (t1, p1) = tri.intersection(&ray).expect("triangle should hit ray");

        assert!((t0 - 1.0).abs() < 0.001);
        assert!((p0.x - 0.25).abs() < 0.001);
        assert!((p0.y - 0.25).abs() < 0.001);
        assert!(p0.z.abs() < 0.001);

        assert!((t1 - t0).abs() < 0.001);
        assert!((p1.x - p0.x).abs() < 0.001);
        assert!((p1.y - p0.y).abs() < 0.001);
        assert!((p1.z - p0.z).abs() < 0.001);
    }

    #[test]
    fn test_line_triangle_intersection_accepts_hits_on_both_sides() {
        let tri = Tri3::new([
            Vector3::new(0.0f32, 0.0, 0.0),
            Vector3::new(1.0, 0.0, 0.0),
            Vector3::new(0.0, 1.0, 0.0),
        ]);
        let line = Line::new(
            &Vector3::new(0.25f32, 0.25, 1.0),
            &Vector3::new(0.0, 0.0, 1.0),
            EPS_F32,
        )
        .expect("line should be valid");

        let (t0, p0) = line.intersection(&tri).expect("line should hit triangle");
        let (t1, p1) = tri.intersection(&line).expect("triangle should hit line");

        assert!((t0 + 1.0).abs() < 0.001);
        assert!((p0.x - 0.25).abs() < 0.001);
        assert!((p0.y - 0.25).abs() < 0.001);
        assert!(p0.z.abs() < 0.001);

        assert!((t1 - t0).abs() < 0.001);
        assert!((p1.x - p0.x).abs() < 0.001);
        assert!((p1.y - p0.y).abs() < 0.001);
        assert!((p1.z - p0.z).abs() < 0.001);
    }

    #[test]
    fn test_box3_sphere_intersect_negative_radius_canonicalized() {
        let b = Box3::new(
            &Vector3::new(0.0f32, 0.0, 0.0),
            &Vector3::new(1.0, 1.0, 1.0),
        );
        let touching = Sphere3::new(Vector3::new(2.0f32, 0.5, 0.5), -1.0);
        assert!(b.intersect(&touching));
    }
}
