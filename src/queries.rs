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
//! Line/ray-to-triangle orientation checks use a dimensionless angular
//! tolerance. Triangle edges and query directions are normalized temporarily
//! with max-component scaling, so the parallelism decision is not coupled to
//! triangle area or direction magnitude. Positive rescaling of a finite,
//! nonzero stored direction preserves a ray's hit, while nonzero rescaling of a
//! line direction preserves its hit. Returned `t` values continue to
//! parameterize the original stored direction.
//!
//! The `Box3`/`Ray` query uses a slab-interval test. For ray directions that are
//! parallel to a slab plane, the implementation checks whether the ray origin is
//! already inside that slab instead of relying on IEEE `0/0` behaviour.

use crate::primitives::*;
use crate::scalar::*;
use crate::vector::*;
use crate::MathError;
use num_traits::{One, Zero};

////////////////////////////////////////////////////////////////////////////////
/// Query Traits
////////////////////////////////////////////////////////////////////////////////
pub trait Distance<T, Other> {
    /// Computes distance using ordinary arithmetic, without input or range validation.
    fn distance(&self, other: &Other) -> T;
    /// Computes distance without hiding the reason a calculation is undefined.
    fn checked_distance(&self, other: &Other) -> Result<T, MathError>;
}

/// Trait for boolean intersection tests.
pub trait Intersect<T> {
    /// Tests intersection with unchecked arithmetic. Boundary contact counts as a hit.
    fn intersect(&self, other: &T) -> bool;
    /// Distinguishes a valid boolean answer from unrepresentable arithmetic.
    fn checked_intersect(&self, other: &T) -> Result<bool, MathError>;
}

/// Trait for producing intersection results.
pub trait Intersection<T, Other> {
    /// Returns an unchecked geometric intersection, or `None` for a geometric miss.
    fn intersection(&self, other: &Other) -> Option<T>;
    /// Returns `Ok(None)` for a geometric miss, or an error for invalid or
    /// unrepresentable input/calculation. Prefer this method for diagnosis.
    fn checked_intersection(&self, other: &Other) -> Result<Option<T>, MathError>;
}

////////////////////////////////////////////////////////////////////////////////
/// Distance Queries
////////////////////////////////////////////////////////////////////////////////
impl<T: RealScalar> Distance<T, Vector3<T>> for Plane<T> {
    /// Computes distance using ordinary arithmetic, without input or range validation.
    fn distance(&self, other: &Vector3<T>) -> T {
        (Vector3::dot(&self.normal(), other) + self.constant()).tabs() / self.normal().length()
    }
    fn checked_distance(&self, other: &Vector3<T>) -> Result<T, MathError> {
        let metric = checked_normalized3(&self.normal())?;
        let constant = metric.checked_divide_by_length(self.constant())?;
        constant
            .checked_add(Vector3::checked_dot(&metric.unit(), other)?)?
            .checked_abs()
    }
}

impl<T: RealScalar> Distance<T, Vector3<T>> for Segment<T, Vector3<T>> {
    /// Computes distance using ordinary arithmetic, without input or range validation.
    fn distance(&self, other: &Vector3<T>) -> T {
        Segment::distance(self, other)
    }
    fn checked_distance(&self, other: &Vector3<T>) -> Result<T, MathError> {
        let (_, closest) = self.checked_closest_point_on_segment(other, T::epsilon())?;
        Vector3::checked_sub_vv(&closest, other)?.checked_length()
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
fn checked_update_slab_interval<T: RealScalar>(
    slab_min: T,
    slab_max: T,
    ray_start: T,
    ray_dir: T,
    tmin: &mut Option<T>,
    tmax: &mut Option<T>,
) -> Result<bool, MathError> {
    if ray_dir == <T as Zero>::zero() {
        return Ok(ray_start >= slab_min && ray_start <= slab_max);
    }

    let mut t0 = slab_min.checked_sub(ray_start)?.checked_div(ray_dir)?;
    let mut t1 = slab_max.checked_sub(ray_start)?.checked_div(ray_dir)?;
    if t0 > t1 {
        core::mem::swap(&mut t0, &mut t1);
    }

    *tmin = Some(tmin.map_or(t0, |old| T::max(old, t0)));
    *tmax = Some(tmax.map_or(t1, |old| T::min(old, t1)));
    Ok(tmin.unwrap() <= tmax.unwrap())
}

/// Updates a slab interval with ordinary arithmetic; parallel-axis handling is geometric.
fn update_slab_interval<T: RealScalar>(
    slab_min: T,
    slab_max: T,
    ray_start: T,
    ray_dir: T,
    tmin: &mut Option<T>,
    tmax: &mut Option<T>,
) -> bool {
    if ray_dir == <T as Zero>::zero() {
        return ray_start >= slab_min && ray_start <= slab_max;
    }

    let mut t0 = (slab_min - ray_start) / ray_dir;
    let mut t1 = (slab_max - ray_start) / ray_dir;
    if t0 > t1 {
        core::mem::swap(&mut t0, &mut t1);
    }

    *tmin = Some(tmin.map_or(t0, |old| T::max(old, t0)));
    *tmax = Some(tmax.map_or(t1, |old| T::min(old, t1)));
    tmin.unwrap() <= tmax.unwrap()
}

impl<T: RealScalar> Intersect<Ray<T, Vector3<T>>> for Box3<T> {
    /// Tests intersection with unchecked arithmetic. Boundary contact counts as a hit.
    fn intersect(&self, other: &Ray<T, Vector3<T>>) -> bool {
        let mut lo = None;
        let mut hi = None;
        if !update_slab_interval(
            self.min.x,
            self.max.x,
            other.start.x,
            other.direction.x,
            &mut lo,
            &mut hi,
        ) {
            return false;
        }
        if !update_slab_interval(
            self.min.y,
            self.max.y,
            other.start.y,
            other.direction.y,
            &mut lo,
            &mut hi,
        ) {
            return false;
        }
        if !update_slab_interval(
            self.min.z,
            self.max.z,
            other.start.z,
            other.direction.z,
            &mut lo,
            &mut hi,
        ) {
            return false;
        }
        hi.map_or(true, |exit| exit >= <T as Zero>::zero())
    }
    fn checked_intersect(&self, other: &Ray<T, Vector3<T>>) -> Result<bool, MathError> {
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
        if !other.start.is_finite()
            || !other.direction.is_finite()
            || !self.min.is_finite()
            || !self.max.is_finite()
        {
            return Err(MathError::NonFinite);
        }
        if !T::checked_norm_exceeds(
            &[other.direction.x, other.direction.y, other.direction.z],
            <T as Zero>::zero(),
        )? {
            return Err(MathError::Degenerate);
        }
        let mut tmin = None;
        let mut tmax = None;

        if !checked_update_slab_interval(
            self.min.x,
            self.max.x,
            other.start.x,
            other.direction.x,
            &mut tmin,
            &mut tmax,
        )? {
            return Ok(false);
        }
        if !checked_update_slab_interval(
            self.min.y,
            self.max.y,
            other.start.y,
            other.direction.y,
            &mut tmin,
            &mut tmax,
        )? {
            return Ok(false);
        }
        if !checked_update_slab_interval(
            self.min.z,
            self.max.z,
            other.start.z,
            other.direction.z,
            &mut tmin,
            &mut tmax,
        )? {
            return Ok(false);
        }

        // Intersect the final slab interval with the forward-ray constraint
        // `t >= 0`. Touching the box boundary counts as an intersection.
        Ok(tmax.map_or(true, |end| {
            end >= T::max(tmin.unwrap_or_else(<T as Zero>::zero), <T as Zero>::zero())
        }))
    }
}

impl<T: RealScalar> Intersect<Sphere3<T>> for Box3<T> {
    /// Tests intersection with unchecked arithmetic. Boundary contact counts as a hit.
    fn intersect(&self, other: &Sphere3<T>) -> bool {
        let distance = |c: T, lo: T, hi: T| {
            if c < lo {
                lo - c
            } else if c > hi {
                c - hi
            } else {
                <T as Zero>::zero()
            }
        };
        T::norm(&[
            distance(other.center.x, self.min.x, self.max.x),
            distance(other.center.y, self.min.y, self.max.y),
            distance(other.center.z, self.min.z, self.max.z),
        ]) <= other.radius
    }
    fn checked_intersect(&self, other: &Sphere3<T>) -> Result<bool, MathError> {
        if !other.radius.is_finite() {
            return Err(MathError::NonFinite);
        }
        if other.radius < <T as Zero>::zero() {
            return Err(MathError::Domain);
        }
        let center = other.center;
        if !center.is_finite() || !self.min.is_finite() || !self.max.is_finite() {
            return Err(MathError::NonFinite);
        }
        // The three axes are explicit; checked subtraction order stays x/y/z.
        let distance = |c: T, lo: T, hi: T| -> Result<T, MathError> {
            if c < lo {
                lo.checked_sub(c)
            } else if c > hi {
                c.checked_sub(hi)
            } else {
                Ok(<T as Zero>::zero())
            }
        };
        let distances = [
            distance(center.x, self.min.x, self.max.x)?,
            distance(center.y, self.min.y, self.max.y)?,
            distance(center.z, self.min.z, self.max.z)?,
        ];
        Ok(!T::checked_norm_exceeds(&distances, other.radius)?)
    }
}

fn is_in_0_to_1_range<T: RealScalar>(x: T) -> bool {
    x >= <T as Zero>::zero() && x <= <T as One>::one()
}

impl<T: RealScalar> Intersect<Tri3<T>> for Sphere3<T> {
    /// Tests intersection with unchecked arithmetic. Boundary contact counts as a hit.
    fn intersect(&self, tri: &Tri3<T>) -> bool {
        let uvw = tri.barycentric_coordinates(&self.center);
        let v = tri.vertices();
        if is_in_0_to_1_range(uvw.x) && is_in_0_to_1_range(uvw.y) && is_in_0_to_1_range(uvw.z) {
            Plane::from_tri(&v[0], &v[1], &v[2]).distance(&self.center) <= self.radius
        } else {
            let a = Segment::new(&v[0], &v[1]).distance(&self.center);
            let b = Segment::new(&v[1], &v[2]).distance(&self.center);
            let c = Segment::new(&v[2], &v[0]).distance(&self.center);
            T::min(a, T::min(b, c)) <= self.radius
        }
    }
    fn checked_intersect(&self, tri: &Tri3<T>) -> Result<bool, MathError> {
        if !self.radius.is_finite() {
            return Err(MathError::NonFinite);
        }
        if self.radius < <T as Zero>::zero() {
            return Err(MathError::Domain);
        }
        let uvw = tri.checked_barycentric_coordinates(&self.center)?;
        let verts = tri.vertices();
        if is_in_0_to_1_range(uvw.x) && is_in_0_to_1_range(uvw.y) && is_in_0_to_1_range(uvw.z) {
            let plane = Plane::checked_from_tri(&verts[0], &verts[1], &verts[2], T::epsilon())?;
            Ok(plane.checked_distance(&self.center)? <= self.radius)
        } else {
            let d01 = Segment::new(&verts[0], &verts[1]).checked_distance(&self.center)?;
            let d12 = Segment::new(&verts[1], &verts[2]).checked_distance(&self.center)?;
            let d20 = Segment::new(&verts[2], &verts[0]).checked_distance(&self.center)?;
            Ok(T::min(T::min(d01, d12), d20) <= self.radius)
        }
    }
}

/// Selects the parameter-domain rule applied by the shared triangle query.
///
/// Lines extend through their origin in both directions, whereas rays include
/// only points at non-negative parameters. Keeping this distinction explicit
/// lets the arithmetic and barycentric rules remain identical for both public
/// [`Intersection`] implementations.
enum TriangleIntersectionKind {
    /// Restrict the result to the forward half-line, including its origin.
    Ray,
    /// Accept intersections on either side of the infinite line's origin.
    Line,
}

/// Normalized Moller-Trumbore solve with ordinary arithmetic. Angular/barycentric
/// branches describe geometric misses; the caller owns input and range validity.
fn triangle_intersection_unchecked<T: RealScalar>(
    start: &Vector3<T>,
    direction: &Vector3<T>,
    tri: &Tri3<T>,
    epsilon: T,
    kind: TriangleIntersectionKind,
) -> Option<(T, Vector3<T>)> {
    let v = tri.vertices();
    let e1 = normalized3(&(v[1] - v[0]));
    let e2 = normalized3(&(v[2] - v[0]));
    let direction = normalized3(direction);
    let a = e1.unit();
    let b = e2.unit();
    let d = direction.unit();
    let cross = Vector3::cross(&a, &b);
    if cross.length() <= epsilon {
        return None;
    }
    if Vector3::dot(&cross.normalize(), &d).tabs() <= epsilon {
        return None;
    }
    let p = Vector3::cross(&d, &b);
    let det = Vector3::dot(&a, &p);
    let delta = *start - v[0];
    let u = e1.divide_by_length_unchecked(Vector3::dot(&delta, &p) / det);
    if u < -epsilon || u > <T as One>::one() + epsilon {
        return None;
    }
    let q = Vector3::cross(&delta, &a);
    let v = e2.divide_by_length_unchecked(Vector3::dot(&d, &q) / det);
    if v < -epsilon || u + v > <T as One>::one() + T::two() * epsilon {
        return None;
    }
    let travel = Vector3::dot(&b, &q) / det;
    if matches!(kind, TriangleIntersectionKind::Ray) && travel < <T as Zero>::zero() {
        return None;
    }
    Some((
        direction.divide_by_length_unchecked(travel),
        *start + d * travel,
    ))
}

/// Intersects a parameterized line or ray with a triangle.
///
/// This is a normalized form of Tomas Möller and Ben Trumbore's
/// Möller–Trumbore algorithm. The direction and both triangle edges are each
/// normalized exactly once with overflow-safe component scaling, and their
/// decompositions are reused for every subsequent orientation and parameter
/// calculation. The derived triangle normal is likewise normalized once.
/// Consequently, multiplying a valid direction or all triangle offsets by a
/// finite, positive scale does not turn the raw determinant into an absolute
/// length threshold, while the query pays only four stable normalization costs.
///
/// `epsilon` is dimensionless here: it is both the angular tolerance used to
/// reject parallel/degenerate orientations and the tolerance around the
/// triangle's barycentric boundary. The returned `t` still parameterizes the
/// original, unnormalized `direction`, preserving [`Line`] semantics.
///
/// `Ok(None)` is returned for a geometric miss or an orientation within the
/// rejected angular band. Invalid input, degenerate directions and checked
/// arithmetic failures return `MathError`. The ordinary public query uses its
/// separate unchecked solve; it does not collapse errors from this function.
fn checked_triangle_intersection_from_point_dir<T: RealScalar>(
    start: &Vector3<T>,
    direction: &Vector3<T>,
    tri: &Tri3<T>,
    epsilon: T,
    kind: TriangleIntersectionKind,
) -> Result<Option<(T, Vector3<T>)>, MathError> {
    if !start.is_finite() {
        return Err(MathError::NonFinite);
    }
    if !epsilon.is_finite() || epsilon < <T as Zero>::zero() || epsilon > <T as One>::one() {
        return Err(MathError::InvalidTolerance);
    }

    // Fetch and name each vertex once so all following vectors share `v0` as
    // their origin, matching the usual Möller–Trumbore derivation.
    let verts = tri.vertices();
    let v0 = verts[0];
    let v1 = verts[1];
    let v2 = verts[2];

    // Subtracting finite but extremely separated coordinates can itself
    // overflow. The normalization helpers below reject such a result instead
    // of allowing an infinite edge to contaminate the intersection tuple.
    let edge1 = Vector3::checked_sub_vv(&v1, &v0)?;
    let edge2 = Vector3::checked_sub_vv(&v2, &v0)?;

    // Normalize each independent source vector exactly once. Retaining each
    // decomposition's scale metadata both avoids repeated square roots in the
    // angular predicates and later converts physical distances back into the
    // barycentric coordinates and original line parameter without forming a
    // potentially overflowing Euclidean length.
    let normalized_edge1 = checked_normalized3(&edge1)?;
    let normalized_edge2 = checked_normalized3(&edge2)?;
    let normalized_direction = checked_normalized3(direction)?;
    let edge1_unit = normalized_edge1.unit();
    let edge2_unit = normalized_edge2.unit();
    let direction_unit = normalized_direction.unit();

    // Triangle degeneracy is an angular property: two non-zero edges are
    // degenerate when their normalized cross magnitude is within `epsilon`.
    // Supplying the cached decompositions lets the predicate compare their
    // unit directions without normalizing either edge for a second time. The
    // shared gate rejects degenerate or invalid classifications and allows only
    // a valid separated edge pair to proceed to the triangle normal.
    match classify_nearly_parallel_normalized3(&normalized_edge1, &normalized_edge2, epsilon) {
        AngularClassification::OutsideTolerance => (),
        AngularClassification::WithinTolerance => return Ok(None),
        AngularClassification::Invalid => return Err(MathError::InvalidTolerance),
    }

    // The cross of the unit edges has the triangle's normal direction and a
    // magnitude equal to the sine of their angle. Normalize that derived value
    // once after the degeneracy check, retaining the result for the following
    // direction-versus-plane orientation decision.
    let triangle_normal = Vector3::checked_cross(&edge1_unit, &edge2_unit)?;
    let normalized_triangle_normal = checked_normalized3(&triangle_normal)?;

    // A query direction perpendicular to the triangle normal lies parallel to
    // the triangle plane. Both operands are already normalized, so the cached
    // predicate reduces this decision to bounded unit-vector arithmetic. The
    // shared gate rejects in-plane or invalid classifications and lets only a
    // valid direction outside the tolerance band reach the intersection solve.
    match classify_nearly_perpendicular_normalized3(
        &normalized_direction,
        &normalized_triangle_normal,
        epsilon,
    ) {
        AngularClassification::OutsideTolerance => (),
        AngularClassification::WithinTolerance => return Ok(None),
        AngularClassification::Invalid => return Err(MathError::InvalidTolerance),
    }

    // Use only unit vectors in the determinant. Its magnitude now depends on
    // angles, not on direction length or triangle area, and remains bounded
    // for every finite input accepted by the normalization helpers.
    let pvec = Vector3::checked_cross(&direction_unit, &edge2_unit)?;
    let det = Vector3::checked_dot(&edge1_unit, &pvec)?;
    if det == <T as Zero>::zero() {
        return Err(MathError::Degenerate);
    }

    // Express the line origin relative to the triangle origin. Explicitly
    // reject overflow and invalid coordinates before using them in cross/dot
    // products, since NaN would otherwise bypass every ordered comparison.
    let tvec = Vector3::checked_sub_vv(start, &v0)?;
    if !is_finite_vector3(&tvec) {
        return Err(MathError::Overflow);
    }

    // Solving against unit edge 1 first yields a physical distance along that
    // edge. Divide by the original edge length to recover dimensionless `u`.
    let edge1_distance = Vector3::checked_dot(&tvec, &pvec)?.checked_div(det)?;
    if !is_finite_scalar(edge1_distance) {
        return Err(MathError::Overflow);
    }
    let u = normalized_edge1.checked_divide_by_length(edge1_distance)?;
    if u < -epsilon || u > <T as One>::one().checked_add(epsilon)? {
        return Ok(None);
    }

    // The second Cramer numerator is shared by the second barycentric
    // coordinate and the signed travel distance along the unit direction.
    let qvec = Vector3::checked_cross(&tvec, &edge1_unit)?;
    if !is_finite_vector3(&qvec) {
        return Err(MathError::Overflow);
    }

    // As for `u`, the normalized solve first produces a physical distance;
    // divide by edge 2's original length to obtain dimensionless `v`.
    let edge2_distance = Vector3::checked_dot(&direction_unit, &qvec)?.checked_div(det)?;
    if !is_finite_scalar(edge2_distance) {
        return Err(MathError::Overflow);
    }
    let v = normalized_edge2.checked_divide_by_length(edge2_distance)?;
    if v < -epsilon
        || u.checked_add(v)? > <T as One>::one().checked_add(T::two().checked_mul(epsilon)?)?
    {
        return Ok(None);
    }

    // With a unit direction this numerator produces signed physical travel
    // distance `tau`. Its sign is therefore the correct ray-domain decision,
    // independent of the magnitude of the caller's stored direction.
    let tau = Vector3::checked_dot(&edge2_unit, &qvec)?.checked_div(det)?;
    if !is_finite_scalar(tau) {
        return Err(MathError::Overflow);
    }
    if matches!(kind, TriangleIntersectionKind::Ray) && tau < <T as Zero>::zero() {
        return Ok(None);
    }

    // Convert unit-distance travel back to the parameter of the original
    // direction. Positive direction rescaling therefore changes `t`
    // inversely while leaving the represented point unchanged. Unlike internal
    // barycentric coordinates, this returned parameter must not round a
    // nonzero travel distance to zero: callers need it to reconstruct the hit.
    let t = normalized_direction.checked_divide_by_length(tau)?;
    if tau != <T as Zero>::zero() && t == <T as Zero>::zero() {
        return Err(MathError::Overflow);
    }

    // Construct the point from the unit direction and physical distance. This
    // avoids multiplying a huge direction by a tiny parameter (or vice versa),
    // either of which can overflow or underflow before their scales cancel.
    let point = Vector3::checked_add_vv(start, &Vector3::checked_mul_vs(&direction_unit, tau)?)?;
    if !is_finite_scalar(t) || !is_finite_vector3(&point) {
        return Err(MathError::Overflow);
    }

    // Staged division is ambiguous only at the zero/minimum-subnormal boundary:
    // a mathematical quotient on the zero side can double-round to the smallest
    // nonzero `t`. Keep the common path to one scalar magnitude check. For that
    // rare boundary alone, verify the API's parameter equation component by
    // component and reject a tuple whose `t` cannot reproduce its physical point.
    let zero = <T as Zero>::zero();
    if t != zero && t.tabs() * T::half() == zero {
        let parameterized_point =
            Vector3::checked_add_vv(start, &Vector3::checked_mul_vs(direction, t)?)?;
        if parameterized_point.x != point.x
            || parameterized_point.y != point.y
            || parameterized_point.z != point.z
        {
            return Err(MathError::Overflow);
        }
    }

    // All geometric, domain, and floating-point validity checks succeeded.
    Ok(Some((t, point)))
}

impl<T: RealScalar> Intersection<(T, Vector3<T>), Tri3<T>> for Ray<T, Vector3<T>> {
    /// Returns an unchecked geometric intersection, or `None` for a geometric miss.
    fn intersection(&self, tri: &Tri3<T>) -> Option<(T, Vector3<T>)> {
        triangle_intersection_unchecked(
            &self.start,
            &self.direction,
            tri,
            T::epsilon(),
            TriangleIntersectionKind::Ray,
        )
    }
    fn checked_intersection(&self, tri: &Tri3<T>) -> Result<Option<(T, Vector3<T>)>, MathError> {
        // Select the ray domain while sharing all normalized orientation and
        // barycentric arithmetic with the infinite-line implementation.
        checked_triangle_intersection_from_point_dir(
            &self.start,
            &self.direction,
            tri,
            T::epsilon(),
            TriangleIntersectionKind::Ray,
        )
    }
}

impl<T: RealScalar> Intersection<(T, Vector3<T>), Tri3<T>> for Line<T, Vector3<T>> {
    /// Returns an unchecked geometric intersection, or `None` for a geometric miss.
    fn intersection(&self, tri: &Tri3<T>) -> Option<(T, Vector3<T>)> {
        triangle_intersection_unchecked(
            &self.p,
            &self.d,
            tri,
            T::epsilon(),
            TriangleIntersectionKind::Line,
        )
    }
    fn checked_intersection(&self, tri: &Tri3<T>) -> Result<Option<(T, Vector3<T>)>, MathError> {
        // Select the unbounded line domain while sharing all normalized
        // orientation and barycentric arithmetic with the ray implementation.
        checked_triangle_intersection_from_point_dir(
            &self.p,
            &self.d,
            tri,
            T::epsilon(),
            TriangleIntersectionKind::Line,
        )
    }
}

impl<T: RealScalar> Intersection<(T, Vector3<T>), Ray<T, Vector3<T>>> for Tri3<T> {
    /// Returns an unchecked geometric intersection, or `None` for a geometric miss.
    fn intersection(&self, ray: &Ray<T, Vector3<T>>) -> Option<(T, Vector3<T>)> {
        ray.intersection(self)
    }
    fn checked_intersection(
        &self,
        ray: &Ray<T, Vector3<T>>,
    ) -> Result<Option<(T, Vector3<T>)>, MathError> {
        // Keep the symmetric trait surface a pure delegate so parameter,
        // boundary, and invalid-input semantics cannot diverge.
        ray.checked_intersection(self)
    }
}

impl<T: RealScalar> Intersection<(T, Vector3<T>), Line<T, Vector3<T>>> for Tri3<T> {
    /// Returns an unchecked geometric intersection, or `None` for a geometric miss.
    fn intersection(&self, line: &Line<T, Vector3<T>>) -> Option<(T, Vector3<T>)> {
        line.intersection(self)
    }
    fn checked_intersection(
        &self,
        line: &Line<T, Vector3<T>>,
    ) -> Result<Option<(T, Vector3<T>)>, MathError> {
        // Keep the symmetric trait surface a pure delegate so parameter,
        // boundary, and invalid-input semantics cannot diverge.
        line.checked_intersection(self)
    }
}

/// Prepares orthogonal unit axes with checked normalization and negation.
/// The finite nonnegative epsilon rejects a zero or too-small source direction.
fn checked_basis_uv<T: RealScalar>(
    unit: &Vector3<T>,
    epsilon: T,
) -> Result<(Vector3<T>, Vector3<T>), MathError> {
    let u = unit.checked_normalize(epsilon)?;
    let x = u.x;
    let y = u.y;
    let z = u.z;

    let v = if x.tabs() <= y.tabs() && x.tabs() <= z.tabs() {
        Vector3::new(<T as Zero>::zero(), z.checked_neg()?, y)
    } else if y.tabs() <= x.tabs() && y.tabs() <= z.tabs() {
        Vector3::new(z.checked_neg()?, <T as Zero>::zero(), x)
    } else {
        // z.abs() < x.abs() && z.abs() <= y.abs()
        Vector3::new(y.checked_neg()?, x, <T as Zero>::zero())
    };
    let v = v.checked_normalize(<T as Zero>::zero())?;
    Ok((u, v))
}

/// Builds `[u,v,w]` with `u × v = w` using checked arithmetic.
/// A finite nonnegative epsilon rejects an invalid or too-small source direction.
pub fn checked_basis_from_unit_rh<T: RealScalar>(
    unit: &Vector3<T>,
    epsilon: T,
) -> Result<[Vector3<T>; 3], MathError> {
    let (u, v) = checked_basis_uv(unit, epsilon)?;
    let w = Vector3::checked_cross(&u, &v)?.checked_normalize(<T as Zero>::zero())?;
    Ok([u, v, w])
}

/// Builds a right-handed orthonormal basis with unchecked arithmetic.
/// Requires a nonzero finite input and representable norm/cross intermediates.
pub fn basis_from_unit_rh<T: RealScalar>(unit: &Vector3<T>) -> [Vector3<T>; 3] {
    let u = unit.normalize();
    let z = <T as Zero>::zero();
    let v = if u.x.tabs() <= u.y.tabs() && u.x.tabs() <= u.z.tabs() {
        Vector3::new(z, -u.z, u.y)
    } else if u.y.tabs() <= u.z.tabs() {
        Vector3::new(-u.z, z, u.x)
    } else {
        Vector3::new(-u.y, u.x, z)
    };
    let v = v.normalize();
    let w = Vector3::cross(&u, &v).normalize();
    [u, v, w]
}

/// Builds `[u,v,w]` with `u × v = -w` using checked arithmetic.
/// A finite nonnegative epsilon rejects an invalid or too-small source direction.
pub fn checked_basis_from_unit_lh<T: RealScalar>(
    unit: &Vector3<T>,
    epsilon: T,
) -> Result<[Vector3<T>; 3], MathError> {
    let (u, v) = checked_basis_uv(unit, epsilon)?;
    // Negating only the cross-product axis reverses the ordered frame's
    // determinant without changing its first two orthonormal axes.
    let w = Vector3::checked_cross(&u, &v)?.checked_normalize(<T as Zero>::zero())?;
    Ok([u, v, w.checked_neg()?])
}

/// Builds a left-handed orthonormal basis with unchecked arithmetic.
/// Requires a nonzero finite input and representable norm/cross intermediates.
pub fn basis_from_unit_lh<T: RealScalar>(unit: &Vector3<T>) -> [Vector3<T>; 3] {
    let u = unit.normalize();
    let z = <T as Zero>::zero();
    let v = if u.x.tabs() <= u.y.tabs() && u.x.tabs() <= u.z.tabs() {
        Vector3::new(z, -u.z, u.y)
    } else if u.y.tabs() <= u.z.tabs() {
        Vector3::new(-u.z, z, u.x)
    } else {
        Vector3::new(-u.y, u.x, z)
    };
    let v = v.normalize();
    let w = Vector3::cross(&u, &v).normalize();
    [u, v, -w]
}

#[cfg(all(test, any(feature = "std", feature = "libm", feature = "system-libm")))]
mod tests {
    use super::{Intersect, Intersection};
    use crate::primitives::{Box3, Line, Ray, Sphere3, Tri3};
    use crate::scalar::{One, RealScalar, Zero};
    use crate::vector::{CrossProduct, RealVector, Vector, Vector3};
    use crate::EPS_F32;

    /// Verifies that uniformly scaling a well-shaped triangle and its ray
    /// origin does not change the query's dimensionless result.
    ///
    /// The supplied scales deliberately include values whose squared area
    /// would underflow or overflow in the old raw-determinant implementation.
    fn assert_triangle_world_scale_invariance<T>(scales: &[T], tolerance: T)
    where
        T: RealScalar + core::fmt::Debug,
    {
        // Use the default scalar tolerance so this helper exercises the same
        // path as the public `Intersection` implementations.
        let epsilon = T::epsilon();
        let zero = <T as Zero>::zero();
        let one = <T as One>::one();
        let quarter = T::quarter();

        for &scale in scales {
            // Scale every geometric displacement by the same positive factor;
            // the triangle retains its shape and the ray retains its incidence.
            let tri = Tri3::new([
                Vector3::new(zero, zero, zero),
                Vector3::new(scale, zero, zero),
                Vector3::new(zero, scale, zero),
            ]);
            let start = Vector3::new(scale * quarter, scale * quarter, scale);
            let direction = Vector3::new(zero, zero, -one);
            let ray = Ray::checked_new(&start, &direction, epsilon)
                .ok()
                .expect("the unit test direction must define a ray");

            // A valid hit must survive at every representable world scale.
            let (t, point) = ray
                .checked_intersection(&tri)
                .ok()
                .flatten()
                .expect("world scaling must not change a ray/triangle hit");

            // Divide by the fixture scale before comparing so the assertion is
            // meaningful for both extremely small and extremely large cases.
            assert!((t / scale - one).tabs() <= tolerance);
            assert!((point.x / scale - quarter).tabs() <= tolerance);
            assert!((point.y / scale - quarter).tabs() <= tolerance);
            assert!((point.z / scale).tabs() <= tolerance);

            // The symmetric trait implementation must remain a pure delegate
            // and therefore report the same parameter and point.
            let (symmetric_t, symmetric_point) = tri
                .checked_intersection(&ray)
                .ok()
                .flatten()
                .expect("symmetric triangle/ray query must also hit");
            assert!((symmetric_t / scale - one).tabs() <= tolerance);
            assert!(((symmetric_point.x - point.x) / scale).tabs() <= tolerance);
            assert!(((symmetric_point.y - point.y) / scale).tabs() <= tolerance);
            assert!(((symmetric_point.z - point.z) / scale).tabs() <= tolerance);
        }
    }

    /// Verifies that rescaling a stored line or ray direction preserves the
    /// hit point while inversely rescaling its returned parameter.
    fn assert_triangle_direction_scale_invariance<T>(scales: &[T], tolerance: T)
    where
        T: RealScalar + core::fmt::Debug,
    {
        // Keep the triangle and origin fixed so only direction magnitude varies.
        let epsilon = T::epsilon();
        let zero = <T as Zero>::zero();
        let one = <T as One>::one();
        let quarter = T::quarter();
        let tri = Tri3::new([
            Vector3::new(zero, zero, zero),
            Vector3::new(one, zero, zero),
            Vector3::new(zero, one, zero),
        ]);
        let start = Vector3::new(quarter, quarter, one);
        let unit_direction = Vector3::new(zero, zero, -one);

        for &scale in scales {
            // Construct valid public values first, then exercise their public
            // direction fields with magnitudes that constructor squaring could
            // not safely classify at the extremes.
            let mut line = Line::checked_new(&start, &unit_direction, epsilon)
                .ok()
                .expect("the unit test direction must define a line");
            line.d = unit_direction * scale;
            let mut ray = Ray::checked_new(&start, &unit_direction, epsilon)
                .ok()
                .expect("the unit test direction must define a ray");
            ray.direction = unit_direction * scale;

            // Both parameterized primitives represent the same forward hit;
            // multiplying the result by the direction scale recovers t = 1.
            let (line_t, line_point) = line
                .checked_intersection(&tri)
                .ok()
                .flatten()
                .expect("line hit must survive direction rescaling");
            let (ray_t, ray_point) = ray
                .checked_intersection(&tri)
                .ok()
                .flatten()
                .expect("ray hit must survive direction rescaling");
            assert!((line_t * scale - one).tabs() <= tolerance);
            assert!((ray_t * scale - one).tabs() <= tolerance);

            // The physical intersection is independent of parameterization.
            for point in [line_point, ray_point] {
                assert!((point.x - quarter).tabs() <= tolerance);
                assert!((point.y - quarter).tabs() <= tolerance);
                assert!(point.z.tabs() <= tolerance);
            }
        }
    }

    /// Verifies that degenerate and non-finite inputs fail closed instead of
    /// producing a tuple containing NaN or infinity.
    fn assert_triangle_invalid_inputs_return_none<T>(nan: T, infinity: T)
    where
        T: RealScalar + core::fmt::Debug,
    {
        // Build one ordinary triangle and line as the baseline for mutations.
        let epsilon = T::epsilon();
        let zero = <T as Zero>::zero();
        let one = <T as One>::one();
        let quarter = T::quarter();
        let tri = Tri3::new([
            Vector3::new(zero, zero, zero),
            Vector3::new(one, zero, zero),
            Vector3::new(zero, one, zero),
        ]);
        let start = Vector3::new(quarter, quarter, one);
        let direction = Vector3::new(zero, zero, -one);
        let valid_line = Line::checked_new(&start, &direction, epsilon)
            .ok()
            .expect("the unit test direction must define a line");

        // Duplicate and collinear vertices have no two-dimensional interior.
        let duplicate = Tri3::new([
            Vector3::new(zero, zero, zero),
            Vector3::new(zero, zero, zero),
            Vector3::new(zero, one, zero),
        ]);
        let collinear = Tri3::new([
            Vector3::new(zero, zero, zero),
            Vector3::new(one, zero, zero),
            Vector3::new(T::two(), zero, zero),
        ]);
        assert!(valid_line
            .checked_intersection(&duplicate)
            .ok()
            .flatten()
            .is_none());
        assert!(valid_line
            .checked_intersection(&collinear)
            .ok()
            .flatten()
            .is_none());

        // A publicly mutable direction can violate constructor invariants; the
        // query must still reject exact zero and non-finite directions safely.
        let mut invalid_line = valid_line;
        invalid_line.d = Vector3::new(zero, zero, zero);
        assert!(invalid_line
            .checked_intersection(&tri)
            .ok()
            .flatten()
            .is_none());
        invalid_line.d = Vector3::new(zero, zero, nan);
        assert!(invalid_line
            .checked_intersection(&tri)
            .ok()
            .flatten()
            .is_none());
        invalid_line.d = Vector3::new(zero, zero, infinity);
        assert!(invalid_line
            .checked_intersection(&tri)
            .ok()
            .flatten()
            .is_none());

        // Non-finite origins and vertices must likewise return `None`; in
        // particular, unordered NaN comparisons must never leak `Some`.
        let mut invalid_origin = valid_line;
        invalid_origin.p = Vector3::new(nan, quarter, one);
        assert!(invalid_origin
            .checked_intersection(&tri)
            .ok()
            .flatten()
            .is_none());
        let nan_triangle = Tri3::new([
            Vector3::new(zero, zero, zero),
            Vector3::new(nan, zero, zero),
            Vector3::new(zero, one, zero),
        ]);
        let infinite_triangle = Tri3::new([
            Vector3::new(zero, zero, zero),
            Vector3::new(infinity, zero, zero),
            Vector3::new(zero, one, zero),
        ]);
        assert!(valid_line
            .checked_intersection(&nan_triangle)
            .ok()
            .flatten()
            .is_none());
        assert!(valid_line
            .checked_intersection(&infinite_triangle)
            .ok()
            .flatten()
            .is_none());
    }

    fn assert_orthonormal_basis(basis: [Vector3<f32>; 3], handedness: f32) {
        let [u, v, w] = basis;
        assert!((u.length() - 1.0).abs() < 0.001);
        assert!((v.length() - 1.0).abs() < 0.001);
        assert!((w.length() - 1.0).abs() < 0.001);
        assert!(Vector3::dot(&u, &v).abs() < 0.001);
        assert!(Vector3::dot(&u, &w).abs() < 0.001);
        assert!(Vector3::dot(&v, &w).abs() < 0.001);

        let u_cross_v = Vector3::cross(&u, &v);
        assert!((u_cross_v.x - handedness * w.x).abs() < 0.001);
        assert!((u_cross_v.y - handedness * w.y).abs() < 0.001);
        assert!((u_cross_v.z - handedness * w.z).abs() < 0.001);

        let determinant = Vector3::dot(&u, &Vector3::cross(&v, &w));
        assert!((determinant - handedness).abs() < 0.001);
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
        let ray = Ray::checked_new(
            &Vector3::new(1.0f32, 0.5, 0.5),
            &Vector3::new(1.0, 0.0, 0.0),
            EPS_F32,
        )
        .ok()
        .expect("ray should be valid");
        assert!(b.intersect(&ray));
    }

    #[test]
    fn test_box3_ray_intersect_parallel_on_face() {
        let b = Box3::new(
            &Vector3::new(0.0f32, 0.0, 0.0),
            &Vector3::new(1.0, 1.0, 1.0),
        );
        let ray = Ray::checked_new(
            &Vector3::new(0.0f32, 0.5, 0.5),
            &Vector3::new(0.0, 1.0, 0.0),
            EPS_F32,
        )
        .ok()
        .expect("ray should be valid");
        assert!(b.intersect(&ray));
    }

    #[test]
    fn test_box3_ray_intersect_parallel_outside_slab() {
        let b = Box3::new(
            &Vector3::new(0.0f32, 0.0, 0.0),
            &Vector3::new(1.0, 1.0, 1.0),
        );
        let ray = Ray::checked_new(
            &Vector3::new(2.0f32, 0.5, 0.5),
            &Vector3::new(0.0, 1.0, 0.0),
            EPS_F32,
        )
        .ok()
        .expect("ray should be valid");
        assert!(!b.intersect(&ray));
    }

    #[test]
    fn test_try_basis_from_unit_handedness_zero_none() {
        let zero = Vector3::new(0.0f32, 0.0, 0.0);
        assert!(super::checked_basis_from_unit_rh(&zero, EPS_F32)
            .ok()
            .is_none());
        assert!(super::checked_basis_from_unit_lh(&zero, EPS_F32)
            .ok()
            .is_none());
    }

    #[test]
    fn test_try_basis_from_unit_handedness_orthonormal() {
        let directions = [
            Vector3::new(1.0f32, 2.0, 3.0),
            Vector3::new(0.1f32, 4.0, 5.0),
            Vector3::new(4.0f32, 0.1, 5.0),
            Vector3::new(4.0f32, 5.0, 0.1),
        ];

        for direction in directions {
            let expected_u = direction
                .checked_normalize(EPS_F32)
                .expect("direction should normalize");
            let right = super::checked_basis_from_unit_rh(&direction, EPS_F32)
                .ok()
                .expect("right-handed basis should exist");
            let left = super::checked_basis_from_unit_lh(&direction, EPS_F32)
                .ok()
                .expect("left-handed basis should exist");

            assert_orthonormal_basis(right, 1.0);
            assert_orthonormal_basis(left, -1.0);
            assert!((right[0] - expected_u).length() < 0.001);
            assert!((left[0] - expected_u).length() < 0.001);
        }
    }

    #[test]
    fn test_try_basis_from_unit_handedness_axes_and_ties() {
        let directions = [
            Vector3::new(1.0f32, 0.0, 0.0),
            Vector3::new(0.0f32, 1.0, 0.0),
            Vector3::new(0.0f32, 0.0, 1.0),
            Vector3::new(-1.0f32, -1.0, -1.0),
        ];

        for direction in directions {
            assert_orthonormal_basis(
                super::checked_basis_from_unit_rh(&direction, EPS_F32)
                    .ok()
                    .expect("basis"),
                1.0,
            );
            assert_orthonormal_basis(
                super::checked_basis_from_unit_lh(&direction, EPS_F32)
                    .ok()
                    .expect("basis"),
                -1.0,
            );
        }
    }

    #[test]
    fn test_checked_basis_from_unit_rh_zero_errors() {
        assert!(super::checked_basis_from_unit_rh(&Vector3::new(0.0f32, 0.0, 0.0), 0.0).is_err());
    }

    #[test]
    fn test_checked_basis_from_unit_lh_zero_errors() {
        assert!(super::checked_basis_from_unit_lh(&Vector3::new(0.0f32, 0.0, 0.0), 0.0).is_err());
    }

    /// Checks scale-metamorphic triangle intersections for `f32` geometry.
    #[test]
    fn test_triangle_intersection_world_scale_invariant_f32() {
        // These scales make the old area-dependent determinant respectively
        // tiny, ordinary, and enormous while preserving the same triangle.
        assert_triangle_world_scale_invariance(&[1.0e-18f32, 1.0, 1.0e18], 2.0e-5);
    }

    /// Checks scale-metamorphic triangle intersections for `f64` geometry.
    #[test]
    fn test_triangle_intersection_world_scale_invariant_f64() {
        // The wider range exercises underflow/overflow pressure specific to
        // double precision rather than merely repeating the f32 magnitudes.
        assert_triangle_world_scale_invariance(&[1.0e-150f64, 1.0, 1.0e150], 1.0e-12);
    }

    /// Checks direction-parameter invariance for `f32` lines and rays.
    #[test]
    fn test_triangle_intersection_direction_scale_invariant_f32() {
        // Squaring the extreme direction magnitudes would underflow or
        // overflow, while their represented line and ray remain unchanged.
        assert_triangle_direction_scale_invariance(&[1.0e-20f32, 1.0, 1.0e20], 2.0e-5);
    }

    /// Checks direction-parameter invariance for `f64` lines and rays.
    #[test]
    fn test_triangle_intersection_direction_scale_invariant_f64() {
        // Use exponents beyond the safe range of a raw squared norm.
        assert_triangle_direction_scale_invariance(&[1.0e-200f64, 1.0, 1.0e200], 1.0e-12);
    }

    /// Distinguishes an unrepresentable original-direction parameter from a
    /// reconstructable minimum-subnormal parameter in the public triangle query.
    #[test]
    fn test_ray_triangle_intersection_parameter_underflow_contract() {
        let triangle = Tri3::new([
            Vector3::new(-1.0f32, -1.0, 0.0),
            Vector3::new(1.0, -1.0, 0.0),
            Vector3::new(0.0, 1.0, 0.0),
        ]);
        let smallest_positive = f32::from_bits(1);

        // Physical travel of one minimum subnormal divided by the maximum
        // stored direction magnitude rounds to zero. Returning `t = 0` would
        // place the parameterized ray at its elevated origin instead of the
        // triangle, so the query must fail rather than return an inconsistent tuple.
        let mut underflowing_ray = Ray::checked_new(
            &Vector3::new(0.0f32, 0.0, smallest_positive),
            &Vector3::new(0.0, 0.0, -1.0),
            EPS_F32,
        )
        .ok()
        .expect("the unit direction must define a ray");
        underflowing_ray.direction = underflowing_ray.direction * f32::MAX;
        assert!(underflowing_ray
            .checked_intersection(&triangle)
            .ok()
            .flatten()
            .is_none());

        // A length-two stored direction and two-subnormal physical travel yield
        // the minimum positive `t` exactly. Its public ray equation reconstructs
        // the triangle hit, so the exponent-boundary validation must retain it.
        let mut minimum_parameter_ray = Ray::checked_new(
            &Vector3::new(0.0f32, 0.0, smallest_positive * 2.0),
            &Vector3::new(0.0, 0.0, -1.0),
            EPS_F32,
        )
        .ok()
        .expect("the unit direction must define a ray");
        minimum_parameter_ray.direction = minimum_parameter_ray.direction * 2.0;
        let (t, point) = minimum_parameter_ray
            .checked_intersection(&triangle)
            .ok()
            .flatten()
            .expect("a reconstructable minimum-subnormal parameter must be retained");
        assert_eq!(t.to_bits(), smallest_positive.to_bits());
        assert_eq!(point.x, 0.0);
        assert_eq!(point.y, 0.0);
        assert_eq!(point.z, 0.0);
    }

    /// Regresses the audited small-triangle miss caused by comparing its raw
    /// determinant, `0.0008²`, with a linear f32 epsilon.
    #[test]
    fn test_ray_triangle_intersection_accepts_well_shaped_small_triangle() {
        // Every edge is non-zero and the right angle is well outside the
        // angular degeneracy tolerance, regardless of its world-space area.
        let edge_length = 0.0008f32;
        let tri = Tri3::new([
            Vector3::new(0.0, 0.0, 0.0),
            Vector3::new(edge_length, 0.0, 0.0),
            Vector3::new(0.0, edge_length, 0.0),
        ]);
        let ray = Ray::checked_new(
            &Vector3::new(edge_length * 0.25, edge_length * 0.25, edge_length),
            &Vector3::new(0.0, 0.0, -1.0),
            EPS_F32,
        )
        .ok()
        .expect("the unit direction must define a ray");

        // The normalized algorithm must report the interior hit rather than
        // mistaking the small raw determinant for parallelism.
        let (t, point) = ray
            .checked_intersection(&tri)
            .ok()
            .flatten()
            .expect("a well-shaped small triangle must remain intersectable");
        assert!((t / edge_length - 1.0).abs() <= 2.0e-5);
        assert!((point.x / edge_length - 0.25).abs() <= 2.0e-5);
        assert!((point.y / edge_length - 0.25).abs() <= 2.0e-5);
        assert!((point.z / edge_length).abs() <= 2.0e-5);
    }

    /// Keeps a valid triangle hit when an internal positive barycentric
    /// coordinate is too small to remain nonzero in the scalar representation.
    #[test]
    fn test_ray_triangle_intersection_accepts_barycentric_underflow_to_boundary() {
        let smallest_positive = f32::from_bits(1);
        let a = Vector3::new(0.0f32, 0.0, 0.0);
        let b = Vector3::new(2.0, 0.0, 0.0);
        let c = Vector3::new(0.0, 1.0, 0.0);
        let ray = Ray::checked_new(
            &Vector3::new(smallest_positive, 0.25, 1.0),
            &Vector3::new(0.0, 0.0, -1.0),
            EPS_F32,
        )
        .ok()
        .expect("the unit direction must define a ray");

        // The exact x-axis barycentric contribution is positive, but dividing
        // the minimum subnormal x offset by the length-two edge rounds it to
        // zero. Zero is an inclusive boundary used only for containment, so it
        // must not trigger the stricter policy reserved for returned `t`.
        // Cycling the vertices moves that tiny contribution between `u`, `v`,
        // and the implicit third coordinate; ordering cannot change the hit.
        for vertices in [[a, b, c], [b, c, a], [c, a, b]] {
            let tri = Tri3::new(vertices);
            let (t, point) = ray
                .checked_intersection(&tri)
                .ok()
                .flatten()
                .expect("barycentric underflow to an inclusive edge must remain a hit");
            assert_eq!(t, 1.0);
            assert_eq!(point.x, smallest_positive);
            assert_eq!(point.y, 0.25);
            assert_eq!(point.z, 0.0);
        }
    }

    /// Confirms that translating the complete query does not disturb the
    /// normalized Möller–Trumbore solve or its returned world-space point.
    #[test]
    fn test_ray_triangle_intersection_translated_fixture() {
        // Place a two-unit right triangle well away from the origin so edge and
        // origin-relative calculations cannot accidentally rely on zero offsets.
        let tri = Tri3::new([
            Vector3::new(10.0f32, -20.0, 30.0),
            Vector3::new(12.0, -20.0, 30.0),
            Vector3::new(10.0, -18.0, 30.0),
        ]);
        let ray = Ray::checked_new(
            &Vector3::new(10.5f32, -19.5, 35.0),
            &Vector3::new(0.0, 0.0, -1.0),
            EPS_F32,
        )
        .ok()
        .expect("the unit direction must define a ray");

        // The ray travels five units to the translated triangle's interior.
        let (t, point) = ray
            .checked_intersection(&tri)
            .ok()
            .flatten()
            .expect("translation must preserve the represented hit");
        assert!((t - 5.0).abs() <= 1.0e-5);
        assert!((point.x - 10.5).abs() <= 1.0e-5);
        assert!((point.y + 19.5).abs() <= 1.0e-5);
        assert!((point.z - 30.0).abs() <= 1.0e-5);
    }

    /// Locks in inclusive edge/vertex handling and rejection outside the
    /// barycentric boundary after the normalized orientation rewrite.
    #[test]
    fn test_ray_triangle_intersection_barycentric_boundaries() {
        // Use one ordinary triangle and cast parallel rays through an edge, a
        // vertex, and a point beyond the hypotenuse respectively.
        let tri = Tri3::new([
            Vector3::new(0.0f32, 0.0, 0.0),
            Vector3::new(1.0, 0.0, 0.0),
            Vector3::new(0.0, 1.0, 0.0),
        ]);
        let direction = Vector3::new(0.0f32, 0.0, -1.0);
        let edge_ray = Ray::checked_new(&Vector3::new(0.5f32, 0.0, 1.0), &direction, EPS_F32)
            .ok()
            .expect("the edge ray direction must be valid");
        let vertex_ray = Ray::checked_new(&Vector3::new(0.0f32, 0.0, 1.0), &direction, EPS_F32)
            .ok()
            .expect("the vertex ray direction must be valid");
        let outside_ray = Ray::checked_new(&Vector3::new(0.75f32, 0.75, 1.0), &direction, EPS_F32)
            .ok()
            .expect("the outside ray direction must be valid");

        // Touching belongs to the triangle closure, while u + v = 1.5 is well
        // beyond the documented epsilon-expanded barycentric boundary.
        assert!(edge_ray.checked_intersection(&tri).ok().flatten().is_some());
        assert!(vertex_ray
            .checked_intersection(&tri)
            .ok()
            .flatten()
            .is_some());
        assert!(outside_ray
            .checked_intersection(&tri)
            .ok()
            .flatten()
            .is_none());
    }

    /// Exercises both sides of the dimensionless ray/plane and triangle-edge
    /// angular thresholds through the complete public triangle query.
    #[test]
    fn test_triangle_intersection_uses_angular_thresholds() {
        let triangle = Tri3::new([
            Vector3::new(0.0f32, 0.0, 0.0),
            Vector3::new(1.0, 0.0, 0.0),
            Vector3::new(0.0, 1.0, 0.0),
        ]);

        // A direction whose normal component is half epsilon is classified as
        // parallel at every stored magnitude. The chosen start would otherwise
        // reach the triangle interior after one half unit along +X.
        let near_parallel_direction = Vector3::new(1.0f32, 0.0, -EPS_F32 * 0.5);
        for scale in [1.0e-18f32, 1.0, 1.0e18] {
            let mut line = Line::checked_new(
                &Vector3::new(-0.25f32, 0.25, EPS_F32 * 0.25),
                &near_parallel_direction,
                EPS_F32,
            )
            .ok()
            .expect("the order-one base direction must define a line");
            line.d = near_parallel_direction * scale;
            assert!(line
                .checked_intersection(&triangle)
                .ok()
                .flatten()
                .is_none());
        }

        // Doubling the normal component places the direction beyond the
        // parallel band, so the otherwise identical geometry has a valid hit.
        let above_threshold_line = Line::checked_new(
            &Vector3::new(-0.25f32, 0.25, EPS_F32),
            &Vector3::new(1.0, 0.0, -EPS_F32 * 2.0),
            EPS_F32,
        )
        .ok()
        .expect("the above-threshold direction must define a line");
        let (_, above_threshold_point) = above_threshold_line
            .checked_intersection(&triangle)
            .ok()
            .flatten()
            .expect("a direction beyond the angular band must intersect");
        assert!((above_threshold_point.x - 0.25).abs() <= 2.0e-5);
        assert!((above_threshold_point.y - 0.25).abs() <= 2.0e-5);
        assert!(above_threshold_point.z.abs() <= 2.0e-5);

        // Apply the same threshold policy to triangle shape. Half-epsilon edge
        // separation is degenerate; twice-epsilon separation is a valid skinny
        // triangle at small, ordinary, and large representable world scales.
        for scale in [1.0e-18f32, 1.0, 1.0e18] {
            let near_collinear = Tri3::new([
                Vector3::new(0.0f32, 0.0, 0.0),
                Vector3::new(scale, 0.0, 0.0),
                Vector3::new(scale, scale * EPS_F32 * 0.5, 0.0),
            ]);
            let valid_skinny = Tri3::new([
                Vector3::new(0.0f32, 0.0, 0.0),
                Vector3::new(scale, 0.0, 0.0),
                Vector3::new(scale, scale * EPS_F32 * 2.0, 0.0),
            ]);
            let line = Line::checked_new(
                &Vector3::new(scale * 0.5, scale * EPS_F32 * 0.5, scale),
                &Vector3::new(0.0f32, 0.0, -1.0),
                EPS_F32,
            )
            .ok()
            .expect("the unit direction must define a line");

            assert!(line
                .checked_intersection(&near_collinear)
                .ok()
                .flatten()
                .is_none());
            assert!(line
                .checked_intersection(&valid_skinny)
                .ok()
                .flatten()
                .is_some());
        }
    }

    /// Checks that invalid triangle-query inputs fail closed for both public
    /// floating-point scalar implementations.
    #[test]
    fn test_triangle_intersection_rejects_degenerate_and_nonfinite_inputs() {
        // Exercise both scalar implementations because NaN propagation and
        // exponent ranges differ even though the generic algorithm is shared.
        assert_triangle_invalid_inputs_return_none::<f32>(f32::NAN, f32::INFINITY);
        assert_triangle_invalid_inputs_return_none::<f64>(f64::NAN, f64::INFINITY);
    }

    /// Confirms that the shared triangle solver maps an explicit invalid
    /// angular classification to `None` for both supported parameter domains.
    #[test]
    fn test_triangle_intersection_rejects_invalid_angular_tolerances() {
        let triangle = Tri3::new([
            Vector3::new(0.0f32, 0.0, 0.0),
            Vector3::new(1.0, 0.0, 0.0),
            Vector3::new(0.0, 1.0, 0.0),
        ]);
        let start = Vector3::new(0.25f32, 0.25, 1.0);
        let direction = Vector3::new(0.0f32, 0.0, -1.0);

        // Geometry is deliberately ordinary, isolating the enum's `Invalid`
        // state to the non-finite or out-of-range tolerance in each iteration.
        for epsilon in [-EPS_F32, 1.0 + EPS_F32, f32::NAN, f32::INFINITY] {
            assert!(super::checked_triangle_intersection_from_point_dir(
                &start,
                &direction,
                &triangle,
                epsilon,
                super::TriangleIntersectionKind::Ray,
            )
            .is_err());
            assert!(super::checked_triangle_intersection_from_point_dir(
                &start,
                &direction,
                &triangle,
                epsilon,
                super::TriangleIntersectionKind::Line,
            )
            .is_err());
        }
    }

    #[test]
    fn test_ray_triangle_intersection_rejects_hits_behind_ray() {
        let tri = Tri3::new([
            Vector3::new(0.0f32, 0.0, 0.0),
            Vector3::new(1.0, 0.0, 0.0),
            Vector3::new(0.0, 1.0, 0.0),
        ]);
        let ray = Ray::checked_new(
            &Vector3::new(0.25f32, 0.25, 1.0),
            &Vector3::new(0.0, 0.0, 1.0),
            EPS_F32,
        )
        .ok()
        .expect("ray should be valid");

        assert!(ray.checked_intersection(&tri).ok().flatten().is_none());
        assert!(tri.checked_intersection(&ray).ok().flatten().is_none());
    }

    #[test]
    fn test_ray_triangle_intersection_rejects_negative_epsilon_band() {
        let tri = Tri3::new([
            Vector3::new(0.0f32, 0.0, 0.0),
            Vector3::new(1.0, 0.0, 0.0),
            Vector3::new(0.0, 1.0, 0.0),
        ]);
        let start = Vector3::new(0.25f32, 0.25, EPS_F32 * 0.5);
        let direction = Vector3::new(0.0f32, 0.0, 1.0);
        let ray = Ray::checked_new(&start, &direction, EPS_F32)
            .ok()
            .expect("ray should be valid");
        let line = Line::checked_new(&start, &direction, EPS_F32)
            .ok()
            .expect("line should be valid");

        assert!(ray.checked_intersection(&tri).ok().flatten().is_none());
        assert!(tri.checked_intersection(&ray).ok().flatten().is_none());

        let (line_t, _) = line
            .checked_intersection(&tri)
            .ok()
            .flatten()
            .expect("infinite line should retain the negative hit");
        assert!(line_t < 0.0);
    }

    #[test]
    fn test_ray_triangle_intersection_accepts_origin_boundary() {
        let tri = Tri3::new([
            Vector3::new(0.0f32, 0.0, 0.0),
            Vector3::new(1.0, 0.0, 0.0),
            Vector3::new(0.0, 1.0, 0.0),
        ]);
        let ray = Ray::checked_new(
            &Vector3::new(0.25f32, 0.25, 0.0),
            &Vector3::new(0.0, 0.0, 1.0),
            EPS_F32,
        )
        .ok()
        .expect("ray should be valid");

        let (t, point) = ray
            .checked_intersection(&tri)
            .ok()
            .flatten()
            .expect("the ray origin lies on the triangle");
        assert!(t >= 0.0);
        assert!(point.z.abs() < EPS_F32);
    }

    #[test]
    fn test_ray_triangle_intersection_forward_hit() {
        let tri = Tri3::new([
            Vector3::new(0.0f32, 0.0, 0.0),
            Vector3::new(1.0, 0.0, 0.0),
            Vector3::new(0.0, 1.0, 0.0),
        ]);
        let ray = Ray::checked_new(
            &Vector3::new(0.25f32, 0.25, 1.0),
            &Vector3::new(0.0, 0.0, -1.0),
            EPS_F32,
        )
        .ok()
        .expect("ray should be valid");

        let (t0, p0) = ray
            .checked_intersection(&tri)
            .ok()
            .flatten()
            .expect("ray should hit triangle");
        let (t1, p1) = tri
            .checked_intersection(&ray)
            .ok()
            .flatten()
            .expect("triangle should hit ray");

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
        let line = Line::checked_new(
            &Vector3::new(0.25f32, 0.25, 1.0),
            &Vector3::new(0.0, 0.0, 1.0),
            EPS_F32,
        )
        .ok()
        .expect("line should be valid");

        let (t0, p0) = line
            .checked_intersection(&tri)
            .ok()
            .flatten()
            .expect("line should hit triangle");
        let (t1, p1) = tri
            .checked_intersection(&line)
            .ok()
            .flatten()
            .expect("triangle should hit line");

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
