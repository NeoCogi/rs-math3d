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

//! Geometric primitives for 3D graphics and collision detection.
//!
//! This module provides common geometric shapes and primitives used in
//! 3D graphics, physics simulations, and collision detection systems.
//!
//! Discrete storage types such as rectangles and boxes work with integer scalars.
//! Metric and analytic primitives such as lines, rays, planes, spheres, and
//! triangles are intended for floating-point use.
//!
//! Constructors that normalize directions or normals provide checked `try_`
//! variants. The unchecked variants assume non-degenerate input and panic when
//! that precondition is violated.
//!
//! Intersection routines interpret their epsilon as a dimensionless angular
//! tolerance when deciding whether directions are parallel or perpendicular.
//! They normalize temporary working vectors with component scaling, so changing
//! a valid line's direction magnitude does not change its geometric result.
//!
//! # Examples
//!
//! ```
//! # fn main() {
//! # #[cfg(any(feature = "std", feature = "libm", feature = "system-libm"))]
//! # {
//! use rs_math3d::primitives::{Ray, Plane, Tri3};
//! use rs_math3d::vector::Vector3;
//! use rs_math3d::EPS_F32;
//!
//! // Create a ray from origin pointing along +X axis
//! let ray = Ray::checked_new(
//!     &Vector3::new(0.0f32, 0.0, 0.0),
//!     &Vector3::new(1.0, 0.0, 0.0),
//!     EPS_F32,
//! ).unwrap();
//!
//! // Create a plane at z=5 facing down
//! let plane = Plane::new(
//!     &Vector3::new(0.0f32, 0.0, -1.0),
//!     &Vector3::new(0.0, 0.0, 5.0)
//! );
//!
//! let checked = Plane::checked_new(
//!     &Vector3::new(0.0f32, 0.0, -1.0),
//!     &Vector3::new(0.0, 0.0, 5.0),
//!     EPS_F32,
//! ).expect("plane normal should be valid");
//! assert_eq!(plane.constant(), checked.constant());
//! # }
//! # }
//! ```

use crate::scalar::*;
use crate::vector::*;
use crate::MathError;
use num_traits::{One, Zero};

/// A 2D dimension with width and height.
#[repr(C)]
#[derive(Debug, Clone, Copy, Default)]
pub struct Dimension<T: Scalar> {
    /// Width component.
    pub width: T,
    /// Height component.
    pub height: T,
}

impl<T: Scalar> Dimension<T> {
    /// Constructs the value using ordinary arithmetic, without validation.
    /// Callers supply valid finite coordinates and any required nonzero directions.
    pub fn new(width: T, height: T) -> Self {
        Self { width, height }
    }
}

/// A 2D axis-aligned rectangle.
///
/// Defined by its top-left corner position (x, y) and dimensions (width, height).
#[repr(C)]
#[derive(Debug, Clone, Copy, Default)]
pub struct Rect<T: Scalar> {
    /// X coordinate of the top-left corner.
    pub x: T,
    /// Y coordinate of the top-left corner.
    pub y: T,
    /// Rectangle width.
    pub width: T,
    /// Rectangle height.
    pub height: T,
}

impl<T: Scalar> Rect<T> {
    /// Computes the maximum corner with checked arithmetic.
    pub fn checked_max(&self) -> Result<Vector2<T>, MathError> {
        Ok(Vector2::new(
            self.x.checked_add(self.width)?,
            self.y.checked_add(self.height)?,
        ))
    }
    /// Builds sorted bounds, rejecting nonfinite or unrepresentable extents.
    pub fn checked_from(a: &Vector2<T>, b: &Vector2<T>) -> Result<Self, MathError> {
        let zero = <T as Zero>::zero();
        let a = Vector2::checked_add_vv(a, &Vector2::new(zero, zero))?;
        let b = Vector2::checked_add_vv(b, &Vector2::new(zero, zero))?;
        let min = Vector2::min(&a, &b);
        let max = Vector2::max(&a, &b);
        Ok(Self::new(
            min.x,
            min.y,
            max.x.checked_sub(min.x)?,
            max.y.checked_sub(min.y)?,
        ))
    }
    /// Checked rectangle overlap; a geometric miss is Ok(None).
    pub fn checked_intersect(&self, other: &Self) -> Result<Option<Self>, MathError> {
        let a = self.checked_max()?;
        let b = other.checked_max()?;
        let min = Vector2::max(&self.min(), &other.min());
        let max = Vector2::min(&a, &b);
        if max.x < min.x || max.y < min.y {
            Ok(None)
        } else {
            Ok(Some(Self::checked_from(&min, &max)?))
        }
    }
    /// Checked point containment, including representability of maximum bounds.
    pub fn checked_contains(&self, p: &Vector2<T>) -> Result<bool, MathError> {
        let max = self.checked_max()?;
        if !p.x.is_finite() || !p.y.is_finite() {
            return Err(MathError::NonFinite);
        }
        Ok(p.x >= self.x && p.y >= self.y && p.x <= max.x && p.y <= max.y)
    }

    /// Constructs the value using ordinary arithmetic, without validation.
    /// Callers supply valid finite coordinates and any required nonzero directions.
    pub fn new(x: T, y: T, width: T, height: T) -> Self {
        Self {
            x,
            y,
            width,
            height,
        }
    }
    /// Creates a rectangle from two corner points.
    ///
    /// The points don't need to be min/max - they will be sorted.
    pub fn from(min_vec: &Vector2<T>, max_vec: &Vector2<T>) -> Self {
        let min_x = T::min(min_vec.x, max_vec.x);
        let min_y = T::min(min_vec.y, max_vec.y);
        let max_x = T::max(min_vec.x, max_vec.x);
        let max_y = T::max(min_vec.y, max_vec.y);
        Self {
            x: min_x,
            y: min_y,
            width: max_x - min_x,
            height: max_y - min_y,
        }
    }

    /// Returns the minimum corner (top-left) of the rectangle.
    pub fn min(&self) -> Vector2<T> {
        Vector2::new(self.x, self.y)
    }
    /// Returns the maximum corner (bottom-right) of the rectangle.
    pub fn max(&self) -> Vector2<T> {
        Vector2::new(self.x + self.width, self.y + self.height)
    }
    /// Computes the intersection of two rectangles.
    ///
    /// Returns `None` if the rectangles don't overlap.
    pub fn intersect(&self, other: &Self) -> Option<Self> {
        let smx = self.max();
        let smn = self.min();
        let omx = other.max();
        let omn = other.min();

        if smx.x < omn.x || smx.y < omn.y || smn.x > omx.x || smn.y > omx.y {
            return None;
        }

        let min_vec = Vector2::max(&self.min(), &other.min());
        let max_vec = Vector2::min(&self.max(), &other.max());
        Some(Self::from(&min_vec, &max_vec))
    }

    /// Checks if a point is inside the rectangle.
    pub fn contains(&self, p: &Vector2<T>) -> bool {
        p.x >= self.x && p.y >= self.y && p.x <= self.x + self.width && p.y <= self.y + self.height
    }
}

/// A 3D axis-aligned bounding box (AABB).
///
/// Defined by its minimum and maximum corners.
#[repr(C)]
#[derive(Debug, Clone, Copy, Default)]
pub struct Box3<T: Scalar> {
    /// Minimum corner.
    pub min: Vector3<T>,
    /// Maximum corner.
    pub max: Vector3<T>,
}

impl<T: Scalar> Box3<T> {
    /// Constructs the value using ordinary arithmetic, without validation.
    /// Callers supply valid finite coordinates and any required nonzero directions.
    pub fn new(v0: &Vector3<T>, v1: &Vector3<T>) -> Self {
        Self {
            min: Vector3::min(v0, v1),
            max: Vector3::max(v0, v1),
        }
    }
    /// Checks if two boxes overlap.
    pub fn overlap(&self, other: &Self) -> bool {
        if self.max.x < other.min.x {
            return false;
        }
        if self.max.y < other.min.y {
            return false;
        }
        if self.max.z < other.min.z {
            return false;
        }

        if self.min.x > other.max.x {
            return false;
        }
        if self.min.y > other.max.y {
            return false;
        }
        if self.min.z > other.max.z {
            return false;
        }
        true
    }

    /// Expands the box to include a point.
    pub fn add(&self, p: &Vector3<T>) -> Self {
        Self {
            min: Vector3::min(p, &self.min),
            max: Vector3::max(p, &self.max),
        }
    }
}

impl<T: RealScalar> Box3<T> {
    /// Computes the center with checked single-rounding product accumulation.
    pub fn checked_center(&self) -> Result<Vector3<T>, MathError> {
        Ok(Vector3::new(
            T::checked_sum_products(&[
                (self.max.x, T::half(), false),
                (self.min.x, T::half(), false),
            ])?,
            T::checked_sum_products(&[
                (self.max.y, T::half(), false),
                (self.min.y, T::half(), false),
            ])?,
            T::checked_sum_products(&[
                (self.max.z, T::half(), false),
                (self.min.z, T::half(), false),
            ])?,
        ))
    }
    /// Computes half-extents with checked subtraction.
    pub fn checked_extent(&self) -> Result<Vector3<T>, MathError> {
        Vector3::checked_sub_vv(&self.max, &self.checked_center()?)
    }
    /// Subdivides with checked center arithmetic and the same octant ordering.
    pub fn checked_subdivide(&self) -> Result<[Self; 8], MathError> {
        let center = self.checked_center()?;
        // Each literal corner emits one octant directly, preserving the public
        // octant order without scratch corner arrays or runtime iteration.
        macro_rules! octant {
            ($x:ident,$y:ident,$z:ident) => {{
                let corner = Vector3::new(self.$x.x, self.$y.y, self.$z.z);
                Self::new(
                    &Vector3::min(&center, &corner),
                    &Vector3::max(&center, &corner),
                )
            }};
        }
        Ok([
            octant!(min, max, min),
            octant!(max, max, min),
            octant!(max, max, max),
            octant!(min, max, max),
            octant!(min, min, min),
            octant!(max, min, min),
            octant!(max, min, max),
            octant!(min, min, max),
        ])
    }
    /// Returns the center point of the box.
    pub fn center(&self) -> Vector3<T> {
        Vector3::new(
            T::sum_products(&[
                (self.max.x, T::half(), false),
                (self.min.x, T::half(), false),
            ]),
            T::sum_products(&[
                (self.max.y, T::half(), false),
                (self.min.y, T::half(), false),
            ]),
            T::sum_products(&[
                (self.max.z, T::half(), false),
                (self.min.z, T::half(), false),
            ]),
        )
    }
    /// Returns the half-extents of the box from center to max.
    pub fn extent(&self) -> Vector3<T> {
        self.max - self.center()
    }

    /// Subdivides the box into 8 octants.
    ///
    /// Returns an array of 8 boxes representing the octree subdivision.
    pub fn subdivide(&self) -> [Self; 8] {
        let center = self.center();
        // Each literal corner emits one octant directly, preserving the public
        // octant order without scratch corner arrays or runtime iteration.
        macro_rules! octant {
            ($x:ident,$y:ident,$z:ident) => {{
                let corner = Vector3::new(self.$x.x, self.$y.y, self.$z.z);
                Self::new(
                    &Vector3::min(&center, &corner),
                    &Vector3::max(&center, &corner),
                )
            }};
        }
        [
            octant!(min, max, min),
            octant!(max, max, min),
            octant!(max, max, max),
            octant!(min, max, max),
            octant!(min, min, min),
            octant!(max, min, min),
            octant!(max, min, max),
            octant!(min, min, max),
        ]
    }
}

/// An infinite line in 2D or 3D space.
///
/// The stored point and direction define the parameterization `p + d * t` for
/// every scalar `t`. The direction is not required to have unit length.
/// Multiplying `d` by a nonzero scalar `k` leaves the geometric line unchanged,
/// but representable parameters reported against the stored direction change
/// from `t` to `t / k`.
#[repr(C)]
#[derive(Debug, Clone, Copy, Default)]
pub struct Line<T: Scalar, V: Vector<T>> {
    /// Point corresponding to parameter `t = 0`.
    pub p: V,
    /// Direction in the stored parameterization `p + d * t`.
    ///
    /// This vector need not be normalized. Its magnitude and sign determine
    /// the scale and orientation of parameter values returned by line queries.
    pub d: V,
    /// Associates the scalar type with the line without adding stored data.
    ///
    /// The vector type carries the coordinates, while this marker retains `T`
    /// as part of the concrete line type and representation.
    t: core::marker::PhantomData<T>,
}

/// Gate checked angular classifications without collapsing invalid inputs into misses.
/// A normal needs an independent span; both collapsed and invalid spans are errors.
macro_rules! checked_span_gate {
    ($classification:expr) => {
        match $classification {
            AngularClassification::OutsideTolerance => {}
            AngularClassification::WithinTolerance | AngularClassification::Invalid => {
                return Err(MathError::Degenerate)
            }
        }
    };
}

macro_rules! checked_angular_gate {
    ($classification:expr) => {
        match $classification {
            AngularClassification::OutsideTolerance => {}
            AngularClassification::WithinTolerance => return Ok(None),
            AngularClassification::Invalid => return Err(MathError::Degenerate),
        }
    };
}

impl<T: RealScalar, V: RealVector<T>> Line<T, V> {
    /// Constructs the value using ordinary arithmetic, without validation.
    /// Callers supply valid finite coordinates and any required nonzero directions.
    pub fn new(p: &V, d: &V) -> Self {
        Self {
            p: *p,
            d: *d,
            t: core::marker::PhantomData,
        }
    }

    /// Constructs a line by unchecked endpoint subtraction. Requires distinct endpoints.
    pub fn from_start_end(s: &V, e: &V) -> Self {
        Self::new(s, &(*e - *s))
    }

    /// Computes `(t, point)` with unchecked arithmetic, where `point = p + d*t`.
    /// Requires a nonzero direction and representable dot products and divisions.
    pub fn closest_point_on_line(&self, p: &V) -> (T, V) {
        let t = V::dot(&(*p - self.p), &self.d) / V::dot(&self.d, &self.d);
        (t, self.p + self.d * t)
    }

    /// Normalizes direction with ordinary arithmetic. Requires a nonzero direction.
    pub fn normalize(&self) -> Self {
        Self::new(&self.p, &self.d.normalize())
    }

    /// Constructs with input and arithmetic validation.
    /// Returns `MathError` for nonfinite input, invalid tolerance, degeneracy or
    /// unrepresentable arithmetic, as applicable. Linear epsilon is nonnegative.
    pub fn checked_new(p: &V, d: &V, epsilon: T) -> Result<Self, MathError> {
        if !epsilon.is_finite() || epsilon < <T as Zero>::zero() {
            return Err(MathError::InvalidTolerance);
        }

        if !p.is_finite() {
            return Err(MathError::NonFinite);
        }
        d.checked_normalize(epsilon)?;
        Ok(Self {
            p: *p,
            d: *d,
            t: core::marker::PhantomData,
        })
    }

    /// Constructs a line with checked subtraction and a finite nonnegative length tolerance.
    /// Returns `Degenerate` when endpoints are too close; arithmetic failures propagate.
    pub fn checked_from_start_end(s: &V, e: &V, epsilon: T) -> Result<Self, MathError> {
        let dir = V::checked_sub_vv(e, s)?;
        Self::checked_new(s, &dir, epsilon)
    }

    /// Computes `(t, point)` with checked arithmetic and scale-aware normalization.
    /// A finite nonnegative epsilon rejects a direction at or below that length.
    pub fn checked_closest_point_on_line(&self, p: &V, epsilon: T) -> Result<(T, V), MathError> {
        if !epsilon.is_finite() || epsilon < <T as Zero>::zero() {
            return Err(MathError::InvalidTolerance);
        }

        let direction = self.d.checked_normalize(epsilon)?;
        let delta = V::checked_sub_vv(p, &self.p)?;
        let travel = V::checked_dot(&direction, &delta)?;
        let t = travel.checked_div(self.d.checked_length()?)?;
        let point = V::checked_add_vv(&self.p, &V::checked_mul_vs(&direction, travel)?)?;
        Ok((t, point))
    }

    /// Normalizes direction with checked arithmetic.
    /// Validates the origin and a finite nonnegative linear tolerance.
    pub fn checked_normalize(&self, epsilon: T) -> Result<Self, MathError> {
        if !self.p.is_finite() {
            return Err(MathError::NonFinite);
        }
        if !epsilon.is_finite() || epsilon < <T as Zero>::zero() {
            return Err(MathError::InvalidTolerance);
        }

        Ok(Self {
            p: self.p,
            d: self.d.checked_normalize(epsilon)?,
            t: core::marker::PhantomData,
        })
    }
}

/// Shortest connecting segment without input or arithmetic validation.
/// Returns None when directions are parallel within the angular tolerance.
pub fn shortest_segment3d_between_lines3d<T: RealScalar>(
    line0: &Line<T, Vector3<T>>,
    line1: &Line<T, Vector3<T>>,
    epsilon: T,
) -> Option<Segment<T, Vector3<T>>> {
    let a = line0.d.normalize();
    let b = line1.d.normalize();
    let cross = Vector3::cross(&a, &b);
    if cross.length() <= epsilon {
        return None;
    }
    let denominator = Vector3::dot(&cross, &cross);
    let separation = line0.p - line1.p;
    let ab = Vector3::dot(&a, &b);
    let p0 = Vector3::dot(&a, &separation);
    let p1 = Vector3::dot(&b, &separation);
    let d0 = (ab * p1 - p0) / denominator;
    let d1 = (p1 - ab * p0) / denominator;
    Some(Segment::new(&(line0.p + a * d0), &(line1.p + b * d1)))
}

/// Finds the shortest connecting segment with scale-aware checked arithmetic.
/// Angular epsilon must be finite in `0..=1`. Returns `Ok(None)` for directions
/// within the parallel tolerance band, and `MathError` for invalid origins,
/// directions, tolerances or unrepresentable arithmetic. Stored directions
/// and their parameterizations remain unchanged.
pub fn checked_shortest_segment3d_between_lines3d<T: RealScalar>(
    line0: &Line<T, Vector3<T>>,
    line1: &Line<T, Vector3<T>>,
    epsilon: T,
) -> Result<Option<Segment<T, Vector3<T>>>, MathError> {
    if !line0.p.is_finite() || !line1.p.is_finite() {
        return Err(MathError::NonFinite);
    }
    if !epsilon.is_finite() || epsilon < <T as Zero>::zero() || epsilon > <T as One>::one() {
        return Err(MathError::InvalidTolerance);
    }

    // Decompose each direction exactly once through max-component scaling so
    // valid very small or very large inputs do not underflow or overflow. The
    // same decompositions serve both angular classification and the solve.
    let direction0 = checked_normalized3(&line0.d)?;
    let direction1 = checked_normalized3(&line1.d)?;

    // Parallelism is an angular property. Classify the already-normalized
    // inputs instead of paying for a second pair of normalization passes. The
    // shared gate rejects parallel directions, which do not define a unique
    // closest segment, and invalid classifications; only a valid non-parallel
    // pair proceeds to the bounded line solve.
    checked_angular_gate!(classify_nearly_parallel_normalized3(
        &direction0,
        &direction1,
        epsilon,
    ));

    let unit0 = direction0.unit();
    let unit1 = direction1.unit();
    let separation = Vector3::checked_sub_vv(&line0.p, &line1.p)?;

    // For unit directions, the Gram determinant is |u0 x u1|^2. Computing it
    // from the cross product avoids the cancellation in 1 - dot(u0, u1)^2 for
    // nearly parallel lines.
    let cross = Vector3::checked_cross(&unit0, &unit1)?;
    let denominator = Vector3::checked_dot(&cross, &cross)?;
    if !is_finite_scalar(denominator) || denominator <= <T as Zero>::zero() {
        return Err(MathError::Overflow);
    }

    // Solve the two normal equations in distance-along-unit-direction space.
    // These parameters are independent of the magnitudes stored in line.d.
    let direction_dot = Vector3::checked_dot(&unit0, &unit1)?;
    let projection0 = Vector3::checked_dot(&unit0, &separation)?;
    let projection1 = Vector3::checked_dot(&unit1, &separation)?;
    let distance0 = direction_dot
        .checked_mul(projection1)?
        .checked_sub(projection0)?
        .checked_div(denominator)?;
    let distance1 = projection1
        .checked_sub(direction_dot.checked_mul(projection0)?)?
        .checked_div(denominator)?;
    if !is_finite_scalar(distance0) || !is_finite_scalar(distance1) {
        return Err(MathError::Overflow);
    }

    // Reconstruct the closest point on each line from unit directions. This
    // avoids multiplying an extreme stored direction by an inverse-scale t.
    let start = Vector3::checked_add_vv(&line0.p, &Vector3::checked_mul_vs(&unit0, distance0)?)?;
    let end = Vector3::checked_add_vv(&line1.p, &Vector3::checked_mul_vs(&unit1, distance1)?)?;
    if !is_finite_vector3(&start) || !is_finite_vector3(&end) {
        return Err(MathError::Overflow);
    }

    Ok(Some(Segment::new(&start, &end)))
}

/// A line segment with defined start and end points.
#[repr(C)]
#[derive(Clone, Copy, Default)]
pub struct Segment<T: Scalar, V: Vector<T>> {
    /// Start point.
    pub s: V,
    /// End point.
    pub e: V,
    t: core::marker::PhantomData<T>,
}

impl<T: Scalar, V: Vector<T>> Segment<T, V> {
    /// Constructs the value using ordinary arithmetic, without validation.
    /// Callers supply valid finite coordinates and any required nonzero directions.
    pub fn new(s: &V, e: &V) -> Self {
        Self {
            s: *s,
            e: *e,
            t: core::marker::PhantomData,
        }
    }
}

impl<T: RealScalar, V: RealVector<T>> Segment<T, V> {
    /// Computes `(t, point)` with unchecked arithmetic and clamps `t` to `[0,1]`.
    /// Requires distinct endpoints and representable intermediate arithmetic.
    pub fn closest_point_on_segment(&self, p: &V) -> (T, V) {
        let d = self.e - self.s;
        let t = T::min(
            <T as One>::one(),
            T::max(
                <T as Zero>::zero(),
                V::dot(&(*p - self.s), &d) / V::dot(&d, &d),
            ),
        );
        (t, self.s + d * t)
    }

    /// Computes point distance using unchecked arithmetic. Requires distinct endpoints.
    pub fn distance(&self, p: &V) -> T {
        let (_, closest) = self.closest_point_on_segment(p);
        (closest - *p).length()
    }

    /// Computes a clamped closest point with checked arithmetic.
    /// Returns `Degenerate` for length at or below finite nonnegative epsilon.
    pub fn checked_closest_point_on_segment(&self, p: &V, epsilon: T) -> Result<(T, V), MathError> {
        if !epsilon.is_finite() || epsilon < <T as Zero>::zero() {
            return Err(MathError::InvalidTolerance);
        }

        let delta = V::checked_sub_vv(&self.e, &self.s)?;
        let direction = delta.checked_normalize(epsilon)?;
        let length = delta.checked_length()?;
        let offset = V::checked_sub_vv(p, &self.s)?;
        let travel = V::checked_dot(&direction, &offset)?;
        if travel <= <T as Zero>::zero() {
            return Ok((<T as Zero>::zero(), self.s));
        }
        if travel >= length {
            return Ok((<T as One>::one(), self.e));
        }
        let t = travel.checked_div(length)?;
        let point = V::checked_add_vv(&self.s, &V::checked_mul_vs(&direction, travel)?)?;
        Ok((t, point))
    }

    /// Computes point distance with checked closest-point arithmetic.
    /// The finite nonnegative epsilon rejects degenerate segment length.
    pub fn checked_distance_with_epsilon(&self, p: &V, epsilon: T) -> Result<T, MathError> {
        let (_, closest) = self.checked_closest_point_on_segment(p, epsilon)?;
        V::checked_sub_vv(&closest, p)?.checked_length()
    }
}

/// A ray with an origin and direction.
///
/// Points on the ray follow `start + direction * t` for `t >= 0`. Positive
/// rescaling of `direction` preserves that point set; negative rescaling
/// reverses the ray and therefore changes its geometry.
#[repr(C)]
#[derive(Clone, Copy, Default)]
pub struct Ray<T: Scalar, V: Vector<T>> {
    /// Ray origin, corresponding to parameter `t = 0`.
    pub start: V,
    /// Forward ray direction.
    ///
    /// [`Ray::new`] stores a normalized direction, although the public field
    /// may subsequently be changed. Intersection queries therefore depend on
    /// its orientation rather than assuming that its magnitude remains one.
    pub direction: V,
    /// Associates the scalar type with the ray without adding stored data.
    ///
    /// The vector type carries the coordinates, while this marker retains `T`
    /// as part of the concrete ray type and representation.
    t: core::marker::PhantomData<T>,
}

impl<T: RealScalar, V: RealVector<T>> Ray<T, V> {
    /// Constructs the value using ordinary arithmetic, without validation.
    /// Callers supply valid finite coordinates and any required nonzero directions.
    pub fn new(start: &V, direction: &V) -> Self {
        Self {
            start: *start,
            direction: direction.normalize(),
            t: core::marker::PhantomData,
        }
    }

    /// Constructs with input and arithmetic validation.
    /// Returns `MathError` for nonfinite input, invalid tolerance, degeneracy or
    /// unrepresentable arithmetic, as applicable. Linear epsilon is nonnegative.
    pub fn checked_new(start: &V, direction: &V, epsilon: T) -> Result<Self, MathError> {
        if !epsilon.is_finite() || epsilon < <T as Zero>::zero() {
            return Err(MathError::InvalidTolerance);
        }

        if !start.is_finite() {
            return Err(MathError::NonFinite);
        }
        Ok(Self {
            start: *start,
            direction: direction.checked_normalize(epsilon)?,
            t: core::marker::PhantomData,
        })
    }
}

impl<T: RealScalar> Ray<T, Vector3<T>> {
    /// Solves a forward ray-plane hit using unchecked arithmetic.
    /// Returns `None` for an angularly parallel ray or a hit behind the origin.
    /// Requires valid nonzero directions, finite angular epsilon in `0..=1`,
    /// and representable arithmetic.
    pub fn intersect_plane(&self, p: &Plane<T>, epsilon: T) -> Option<Vector3<T>> {
        let n = p.normal();
        let unit = n.normalize();
        let direction = self.direction.normalize();
        let denominator = Vector3::dot(&unit, &direction);
        if denominator.tabs() <= epsilon {
            return None;
        }
        let distance =
            -(p.constant() / n.length() + Vector3::dot(&unit, &self.start)) / denominator;
        if distance < <T as Zero>::zero() {
            None
        } else {
            Some(self.start + direction * distance)
        }
    }

    /// Solves a forward ray-plane hit with checked arithmetic.
    /// Returns `Ok(None)` for a geometric miss; invalid input, angular tolerance
    /// or arithmetic returns `MathError`. Angular epsilon is finite in `0..=1`.
    pub fn checked_intersect_plane(
        &self,
        p: &Plane<T>,
        epsilon: T,
    ) -> Result<Option<Vector3<T>>, MathError> {
        if !self.start.is_finite() {
            return Err(MathError::NonFinite);
        }
        if !epsilon.is_finite() || epsilon < <T as Zero>::zero() || epsilon > <T as One>::one() {
            return Err(MathError::InvalidTolerance);
        }

        let normal = p.normal();

        // Decompose each operand once. Reusing these bounded representations
        // keeps both classification and later arithmetic independent of the
        // stored magnitudes without repeating normalization work.
        let normal_metric = checked_normalized3(&normal)?;
        let direction_metric = checked_normalized3(&self.direction)?;

        // A direction perpendicular to the normal is parallel to the plane.
        // The shared gate rejects that in-tolerance relationship and invalid
        // arithmetic, allowing only a valid non-parallel direction to reach
        // the intersection solve.
        checked_angular_gate!(classify_nearly_perpendicular_normalized3(
            &normal_metric,
            &direction_metric,
            epsilon,
        ));

        let unit_normal = normal_metric.unit();
        let unit_direction = direction_metric.unit();

        // Divide the complete plane equation by |normal|. Plane constructors
        // currently store a unit normal, but keeping this general also handles
        // internally constructed coefficient-scaled planes consistently.
        let unit_constant = normal_metric.checked_divide_by_length(p.constant())?;
        let numerator =
            unit_constant.checked_add(Vector3::checked_dot(&unit_normal, &self.start)?)?;
        let denominator = Vector3::checked_dot(&unit_normal, &unit_direction)?;
        if !is_finite_scalar(numerator) || !is_finite_scalar(denominator) {
            return Err(MathError::Overflow);
        }

        // `distance` is physical travel along the unit ray direction, so its
        // sign is unaffected by positive rescaling of the stored direction.
        let distance = numerator.checked_neg()?.checked_div(denominator)?;
        if distance < <T as Zero>::zero() {
            return Ok(None);
        }

        // Reconstruct with the temporary unit direction. Multiplying by the
        // stored direction would require first converting `distance` into its
        // possibly extreme parameter scale and would add avoidable rounding or
        // overflow risk. Positive direction rescaling leaves both operands in
        // this expression unchanged.
        let point = Vector3::checked_add_vv(
            &self.start,
            &Vector3::checked_mul_vs(&unit_direction, distance)?,
        )?;
        if !point.is_finite() {
            return Err(MathError::Overflow);
        }
        Ok(Some(point))
    }
}

////////////////////////////////////////////////////////////////////////////////
/// A sphere defined by center and radius.
////////////////////////////////////////////////////////////////////////////////
#[repr(C)]
#[derive(Clone, Copy, Default)]
pub struct Sphere3<T: RealScalar> {
    /// Sphere center.
    pub center: Vector3<T>,
    /// Sphere radius.
    pub radius: T,
}

impl<T: RealScalar> Sphere3<T> {
    /// Constructs with input and arithmetic validation.
    /// Returns `MathError` for nonfinite input, invalid tolerance, degeneracy or
    /// unrepresentable arithmetic, as applicable. Linear epsilon is nonnegative.
    pub fn checked_new(center: Vector3<T>, radius: T) -> Result<Self, MathError> {
        if !center.is_finite() || !radius.is_finite() {
            return Err(MathError::NonFinite);
        }
        Ok(Self {
            center,
            radius: radius.checked_abs()?,
        })
    }

    /// Constructs the value using ordinary arithmetic, without validation.
    /// Callers supply valid finite coordinates and any required nonzero directions.
    pub fn new(center: Vector3<T>, radius: T) -> Self {
        Self {
            center,
            radius: radius.tabs(),
        }
    }
}

////////////////////////////////////////////////////////////////////////////////
/// A triangle defined by three vertices.
////////////////////////////////////////////////////////////////////////////////
#[repr(C)]
#[derive(Clone, Copy, Default)]
pub struct Tri3<T: RealScalar> {
    vertices: [Vector3<T>; 3],
}

impl<T: RealScalar> Tri3<T> {
    /// Constructs the value using ordinary arithmetic, without validation.
    /// Callers supply valid finite coordinates and any required nonzero directions.
    pub fn new(vertices: [Vector3<T>; 3]) -> Self {
        Self { vertices }
    }
    /// Returns the triangle vertices.
    pub fn vertices(&self) -> &[Vector3<T>; 3] {
        &self.vertices
    }

    /// Solves barycentric coordinates with a scale-aware checked Gram system.
    /// Returns `Degenerate` for dependent edges and preserves arithmetic failures.
    pub fn checked_barycentric_coordinates(
        &self,
        point: &Vector3<T>,
    ) -> Result<Vector3<T>, MathError> {
        let x = Vector3::checked_sub_vv(&self.vertices[1], &self.vertices[0])?;
        let y = Vector3::checked_sub_vv(&self.vertices[2], &self.vertices[0])?;
        let delta = Vector3::checked_sub_vv(point, &self.vertices[0])?;
        let uv = checked_span_coordinates(&x, &y, &delta)?;
        Ok(Vector3::new(
            <T as One>::one().checked_sub(uv.x)?.checked_sub(uv.y)?,
            uv.x,
            uv.y,
        ))
    }

    /// Solves triangle coordinates using an unchecked Gram system.
    /// Requires independent edges and representable intermediates; components sum to one.
    pub fn barycentric_coordinates(&self, pt: &Vector3<T>) -> Vector3<T> {
        let uv = span_coordinates(
            &(self.vertices[1] - self.vertices[0]),
            &(self.vertices[2] - self.vertices[0]),
            &(*pt - self.vertices[0]),
        );
        Vector3::new(<T as One>::one() - uv.x - uv.y, uv.x, uv.y)
    }
}

/// Direct Gram solve without validating span independence or arithmetic range.
fn span_coordinates<T: RealScalar>(
    x: &Vector3<T>,
    y: &Vector3<T>,
    delta: &Vector3<T>,
) -> Vector2<T> {
    let a = Vector3::dot(x, x);
    let b = Vector3::dot(x, y);
    let c = Vector3::dot(y, y);
    let d = Vector3::dot(delta, x);
    let e = Vector3::dot(delta, y);
    let det = T::sum_products(&[(a, c, false), (b, b, true)]);
    Vector2::new(
        T::sum_products(&[(d, c, false), (e, b, true)]) / det,
        T::sum_products(&[(e, a, false), (d, b, true)]) / det,
    )
}

/// Solves coefficients of a point offset in two independent spanning axes.
/// Unit axes keep the Gram determinant dimensionless; cached lengths convert
/// physical coordinates back to each axis's represented parameterization.
fn checked_span_coordinates<T: RealScalar>(
    x: &Vector3<T>,
    y: &Vector3<T>,
    delta: &Vector3<T>,
) -> Result<Vector2<T>, MathError> {
    let x = checked_normalized3(x)?;
    let y = checked_normalized3(y)?;
    let ux = x.unit();
    let uy = y.unit();
    let a = Vector3::checked_dot(&ux, &ux)?;
    let b = Vector3::checked_dot(&ux, &uy)?;
    let c = Vector3::checked_dot(&uy, &uy)?;
    let d = Vector3::checked_dot(delta, &ux)?;
    let e = Vector3::checked_dot(delta, &uy)?;
    let det = T::checked_sum_products(&[(a, c, false), (b, b, true)])?;
    if det <= <T as Zero>::zero() {
        return Err(MathError::Degenerate);
    }
    let cx = T::checked_sum_products(&[(d, c, false), (e, b, true)])?.checked_div(det)?;
    let cy = T::checked_sum_products(&[(e, a, false), (d, b, true)])?.checked_div(det)?;
    Ok(Vector2::new(
        x.checked_divide_by_length(cx)?,
        y.checked_divide_by_length(cy)?,
    ))
}

////////////////////////////////////////////////////////////////////////////////
/// A plane defined by ax + by + cz + d = 0.
////////////////////////////////////////////////////////////////////////////////
#[repr(C)]
#[derive(Clone, Copy, Default)]
pub struct Plane<T: Scalar> {
    a: T,
    b: T,
    c: T,
    d: T,
}

impl<T: Scalar> Plane<T> {
    /// Returns the plane normal; computed normals use unchecked arithmetic.
    /// Computed normals require independent axes and representable intermediates.
    pub fn normal(&self) -> Vector3<T> {
        Vector3::new(self.a, self.b, self.c)
    }
    /// Returns the plane constant term (d).
    pub fn constant(&self) -> T {
        self.d
    }
}

impl<T: RealScalar> Plane<T> {
    /// Constructs the value using ordinary arithmetic, without validation.
    /// Callers supply valid finite coordinates and any required nonzero directions.
    pub fn new(n: &Vector3<T>, p: &Vector3<T>) -> Self {
        let n = n.normalize();
        Self {
            a: n.x,
            b: n.y,
            c: n.z,
            d: -Vector3::dot(&n, p),
        }
    }

    /// Constructs with input and arithmetic validation.
    /// Returns `MathError` for nonfinite input, invalid tolerance, degeneracy or
    /// unrepresentable arithmetic, as applicable. Linear epsilon is nonnegative.
    pub fn checked_new(n: &Vector3<T>, p: &Vector3<T>, epsilon: T) -> Result<Self, MathError> {
        if !epsilon.is_finite() || epsilon < <T as Zero>::zero() {
            return Err(MathError::InvalidTolerance);
        }

        let norm = n.checked_normalize(epsilon)?;
        let d = Vector3::checked_dot(&norm, p)?;
        Ok(Self {
            a: norm.x,
            b: norm.y,
            c: norm.z,
            d: d.checked_neg()?,
        })
    }

    /// Constructs a plane from the oriented triangle normal without validation.
    /// Requires independent edges and representable arithmetic.
    pub fn from_tri(v0: &Vector3<T>, v1: &Vector3<T>, v2: &Vector3<T>) -> Self {
        Self::new(&tri_normal(v0, v1, v2), v0)
    }

    /// Constructs a triangle plane with checked arithmetic and normalization.
    /// Angular epsilon must be finite in `0..=1`; collapsed spans are `Degenerate`.
    pub fn checked_from_tri(
        v0: &Vector3<T>,
        v1: &Vector3<T>,
        v2: &Vector3<T>,
        epsilon: T,
    ) -> Result<Self, MathError> {
        if !epsilon.is_finite() || epsilon < <T as Zero>::zero() || epsilon > <T as One>::one() {
            return Err(MathError::InvalidTolerance);
        }

        // Build the normal through the shared scale-stable edge-orientation
        // path, then anchor the normalized plane equation at the first vertex.
        let n = checked_tri_normal(v0, v1, v2, epsilon)?;
        Self::checked_new(&n, v0, epsilon)
    }

    /// Constructs a representative plane from the centroid and diagonal normal.
    /// Uses unchecked arithmetic and assumes independent diagonals. Does not validate coplanarity.
    pub fn from_quad(v0: &Vector3<T>, v1: &Vector3<T>, v2: &Vector3<T>, v3: &Vector3<T>) -> Self {
        let q = T::quarter();
        let center = Vector3::new(
            T::sum_products(&[
                (v0.x, q, false),
                (v1.x, q, false),
                (v2.x, q, false),
                (v3.x, q, false),
            ]),
            T::sum_products(&[
                (v0.y, q, false),
                (v1.y, q, false),
                (v2.y, q, false),
                (v3.y, q, false),
            ]),
            T::sum_products(&[
                (v0.z, q, false),
                (v1.z, q, false),
                (v2.z, q, false),
                (v3.z, q, false),
            ]),
        );
        Self::new(&quad_normal(v0, v1, v2, v3), &center)
    }

    /// Constructs a representative centroid/diagonal plane with checked arithmetic.
    /// Angular epsilon must be finite in `0..=1`. Does not validate coplanarity.
    pub fn checked_from_quad(
        v0: &Vector3<T>,
        v1: &Vector3<T>,
        v2: &Vector3<T>,
        v3: &Vector3<T>,
        epsilon: T,
    ) -> Result<Self, MathError> {
        if !epsilon.is_finite() || epsilon < <T as Zero>::zero() || epsilon > <T as One>::one() {
            return Err(MathError::InvalidTolerance);
        }

        // Derive a scale-stable unit normal from the diagonals before choosing
        // the centroid used to anchor the representative plane equation.
        let n = checked_quad_normal(v0, v1, v2, v3, epsilon)?;
        let q = T::quarter();
        let c = Vector3::new(
            T::checked_sum_products(&[
                (v0.x, q, false),
                (v1.x, q, false),
                (v2.x, q, false),
                (v3.x, q, false),
            ])?,
            T::checked_sum_products(&[
                (v0.y, q, false),
                (v1.y, q, false),
                (v2.y, q, false),
                (v3.y, q, false),
            ])?,
            T::checked_sum_products(&[
                (v0.z, q, false),
                (v1.z, q, false),
                (v2.z, q, false),
                (v3.z, q, false),
            ])?,
        );
        Self::checked_new(&n, &c, epsilon)
    }

    /// Returns a ray hit using unchecked plane arithmetic; `None` denotes a geometric miss.
    /// Requires valid spans/directions, representable arithmetic and angular epsilon in `0..=1`.
    pub fn intersect_ray(&self, r: &Ray<T, Vector3<T>>, epsilon: T) -> Option<Vector3<T>> {
        r.intersect_plane(self, epsilon)
    }

    /// Returns an unchecked line hit as `(t, point)` in the stored parameterization.
    /// `None` denotes angular parallelism. Requires valid inputs, representable
    /// arithmetic/parameters and finite angular epsilon in `0..=1`.
    pub fn intersect_line(
        &self,
        line: &Line<T, Vector3<T>>,
        epsilon: T,
    ) -> Option<(T, Vector3<T>)> {
        let n = self.normal();
        let unit = n.normalize();
        let direction = line.d.normalize();
        let denominator = Vector3::dot(&unit, &direction);
        if denominator.tabs() <= epsilon {
            return None;
        }
        let distance = -(self.constant() / n.length() + Vector3::dot(&unit, &line.p)) / denominator;
        Some((distance / line.d.length(), line.p + direction * distance))
    }

    /// Returns a checked ray hit. `Ok(None)` denotes a geometric miss;
    /// invalid geometry, arithmetic or angular epsilon outside `0..=1` is an error.
    pub fn checked_intersect_ray(
        &self,
        r: &Ray<T, Vector3<T>>,
        epsilon: T,
    ) -> Result<Option<Vector3<T>>, MathError> {
        if !epsilon.is_finite() || epsilon < <T as Zero>::zero() || epsilon > <T as One>::one() {
            return Err(MathError::InvalidTolerance);
        }

        // Keep one implementation of ray-domain and angular-tolerance policy;
        // the ray owns the actual normalized intersection calculation.
        r.checked_intersect_plane(self, epsilon)
    }

    /// Returns a checked line hit as `(t, point)` in its original parameterization.
    /// `Ok(None)` denotes angular parallelism. Invalid input/tolerance and
    /// unrepresentable arithmetic or parameters return `MathError`.
    pub fn checked_intersect_line(
        &self,
        line: &Line<T, Vector3<T>>,
        epsilon: T,
    ) -> Result<Option<(T, Vector3<T>)>, MathError> {
        if !line.p.is_finite() {
            return Err(MathError::NonFinite);
        }
        if !epsilon.is_finite() || epsilon < <T as Zero>::zero() || epsilon > <T as One>::one() {
            return Err(MathError::InvalidTolerance);
        }

        let normal = self.normal();

        // Decompose each operand once without forming raw squared lengths,
        // which may overflow or underflow even when both directions are valid.
        // The normalized forms are reused for classification and the solve.
        let normal_metric = checked_normalized3(&normal)?;
        let direction_metric = checked_normalized3(&line.d)?;

        // A direction perpendicular to the normal is parallel to the plane.
        // The shared gate rejects that in-tolerance relationship and invalid
        // arithmetic, allowing only a valid non-parallel direction to reach
        // the original-parameter intersection solve.
        checked_angular_gate!(classify_nearly_perpendicular_normalized3(
            &normal_metric,
            &direction_metric,
            epsilon,
        ));

        let unit_normal = normal_metric.unit();
        let unit_direction = direction_metric.unit();
        let unit_constant = normal_metric.checked_divide_by_length(self.constant())?;
        let numerator = unit_constant.checked_add(Vector3::checked_dot(&unit_normal, &line.p)?)?;
        let denominator = Vector3::checked_dot(&unit_normal, &unit_direction)?;
        if !is_finite_scalar(numerator) || !is_finite_scalar(denominator) {
            return Err(MathError::Overflow);
        }

        // Solve first in physical distance along a unit direction. Dividing by
        // the stored direction length then restores the caller's original line
        // parameterization without using an overflow-prone raw length.
        let distance = numerator.checked_neg()?.checked_div(denominator)?;
        if !is_finite_scalar(distance) {
            return Err(MathError::Overflow);
        }
        // This is `distance / |line.d|`, evaluated from the bounded length
        // decomposition. It restores the parameter of the caller's exact
        // equation `line.p + line.d * t`, including the sign change produced
        // when the stored direction is reversed.
        let t = direction_metric.checked_divide_by_length_preserving_nonzero(distance)?;

        // Build the geometric result from physical travel rather than from the
        // potentially extreme reciprocal pair `line.d * t`. Both expressions
        // represent the same point, but the unit-direction form remains bounded
        // over a much wider range of stored direction magnitudes.
        let point = Vector3::checked_add_vv(
            &line.p,
            &Vector3::checked_mul_vs(&unit_direction, distance)?,
        )?;
        if !is_finite_vector3(&point) {
            return Err(MathError::Overflow);
        }

        // Two staged divisions can round a value whose exact quotient lies on
        // either side of the zero/minimum-subnormal boundary to the same minimum
        // subnormal `t`. This ambiguity occurs only for that smallest nonzero
        // magnitude. In that rare case, directly exercise the public line
        // equation and fail closed unless it reproduces the physical-travel
        // point component for component. Ordinary parameters avoid this extra
        // vector multiply and comparison on the interactive-query hot path.
        let zero = <T as Zero>::zero();
        if t != zero && t.tabs() * T::half() == zero {
            let parameterized_point =
                Vector3::checked_add_vv(&line.p, &Vector3::checked_mul_vs(&line.d, t)?)?;
            if parameterized_point.x != point.x
                || parameterized_point.y != point.y
                || parameterized_point.z != point.z
            {
                return Err(MathError::Overflow);
            }
        }

        Ok(Some((t, point)))
    }
}

////////////////////////////////////////////////////////////////////////////////
/// A parametric plane defined by center and axis vectors.
////////////////////////////////////////////////////////////////////////////////
#[repr(C)]
#[derive(Clone, Copy, Default)]
pub struct ParametricPlane<T: Scalar> {
    /// Plane center point.
    pub center: Vector3<T>,
    /// Plane X axis direction.
    pub x_axis: Vector3<T>,
    /// Plane Y axis direction.
    pub y_axis: Vector3<T>,
}

impl<T: RealScalar> ParametricPlane<T> {
    /// Constructs the value using ordinary arithmetic, without validation.
    /// Callers supply valid finite coordinates and any required nonzero directions.
    pub fn new(center: &Vector3<T>, x_axis: &Vector3<T>, y_axis: &Vector3<T>) -> Self {
        Self {
            center: *center,
            x_axis: *x_axis,
            y_axis: *y_axis,
        }
    }

    /// Builds an infinite plane without validation. Requires independent spanning axes.
    pub fn plane(&self) -> Plane<T> {
        Plane::new(&self.normal(), &self.center)
    }

    /// Builds an infinite plane with checked span and coefficient arithmetic.
    /// Angular epsilon must be finite in `0..=1`; dependent axes are `Degenerate`.
    pub fn checked_plane(&self, epsilon: T) -> Result<Plane<T>, MathError> {
        if !epsilon.is_finite() || epsilon < <T as Zero>::zero() || epsilon > <T as One>::one() {
            return Err(MathError::InvalidTolerance);
        }

        // Build the normal with the same scale-invariant classification used by
        // intersection routines, then anchor the resulting plane at the center.
        let normal = self.checked_normal(epsilon)?;
        Plane::checked_new(&normal, &self.center, epsilon)
    }

    /// Returns the plane normal; computed normals use unchecked arithmetic.
    /// Computed normals require independent axes and representable intermediates.
    pub fn normal(&self) -> Vector3<T> {
        Vector3::cross(&self.x_axis, &self.y_axis).normalize()
    }

    /// Computes the unit normal with checked arithmetic and a dimensionless span test.
    /// Angular epsilon must be finite in `0..=1`; dependent axes are `Degenerate`.
    pub fn checked_normal(&self, epsilon: T) -> Result<Vector3<T>, MathError> {
        if !epsilon.is_finite() || epsilon < <T as Zero>::zero() || epsilon > <T as One>::one() {
            return Err(MathError::InvalidTolerance);
        }

        // Delegate to the shared spanning-vector path so tiny and huge but
        // well-conditioned axes receive identical treatment.
        checked_normal_from_spanning_vectors(&self.x_axis, &self.y_axis, epsilon)
    }

    /// Returns a ray hit using unchecked plane arithmetic; `None` denotes a geometric miss.
    /// Requires valid spans/directions, representable arithmetic and angular epsilon in `0..=1`.
    pub fn intersect_ray(&self, r: &Ray<T, Vector3<T>>, epsilon: T) -> Option<Vector3<T>> {
        self.plane().intersect_ray(r, epsilon)
    }

    /// Returns an unchecked line hit as `(t, point)` in the stored parameterization.
    /// `None` denotes angular parallelism. Requires valid inputs, representable
    /// arithmetic/parameters and finite angular epsilon in `0..=1`.
    pub fn intersect_line(
        &self,
        line: &Line<T, Vector3<T>>,
        epsilon: T,
    ) -> Option<(T, Vector3<T>)> {
        self.plane().intersect_line(line, epsilon)
    }

    /// Returns a checked ray hit. `Ok(None)` denotes a geometric miss;
    /// invalid geometry, arithmetic or angular epsilon outside `0..=1` is an error.
    pub fn checked_intersect_ray(
        &self,
        r: &Ray<T, Vector3<T>>,
        epsilon: T,
    ) -> Result<Option<Vector3<T>>, MathError> {
        if !epsilon.is_finite() || epsilon < <T as Zero>::zero() || epsilon > <T as One>::one() {
            return Err(MathError::InvalidTolerance);
        }

        // Validate and canonicalize the spanning axes once, then reuse the
        // ordinary plane/ray query and its forward-ray semantics.
        let plane = self.checked_plane(epsilon)?;
        r.checked_intersect_plane(&plane, epsilon)
    }

    /// Returns a checked line hit as `(t, point)` in its original parameterization.
    /// `Ok(None)` denotes angular parallelism. Invalid input/tolerance and
    /// unrepresentable arithmetic or parameters return `MathError`.
    pub fn checked_intersect_line(
        &self,
        line: &Line<T, Vector3<T>>,
        epsilon: T,
    ) -> Result<Option<(T, Vector3<T>)>, MathError> {
        if !epsilon.is_finite() || epsilon < <T as Zero>::zero() || epsilon > <T as One>::one() {
            return Err(MathError::InvalidTolerance);
        }

        // Convert through the checked unit-normal representation before
        // delegating to the single line/plane intersection implementation.
        let plane = self.checked_plane(epsilon)?;
        plane.checked_intersect_line(line, epsilon)
    }

    /// Solves oblique plane coordinates with a scale-aware checked Gram system.
    /// Dependent axes and arithmetic failures are explicit errors.
    pub fn checked_project(&self, value: &Vector3<T>) -> Result<Vector2<T>, MathError> {
        let delta = Vector3::checked_sub_vv(value, &self.center)?;
        checked_span_coordinates(&self.x_axis, &self.y_axis, &delta)
    }

    /// Solves oblique plane coordinates without validation.
    /// Requires independent axes and representable Gram-system intermediates.
    pub fn project(&self, v: &Vector3<T>) -> Vector2<T> {
        span_coordinates(&self.x_axis, &self.y_axis, &(*v - self.center))
    }
}

/// Computes the oriented triangle unit normal with unchecked arithmetic.
/// Requires independent edges and representable cross/norm intermediates.
pub fn tri_normal<T: RealScalar>(v0: &Vector3<T>, v1: &Vector3<T>, v2: &Vector3<T>) -> Vector3<T> {
    Vector3::cross(&(*v1 - *v0), &(*v2 - *v0)).normalize()
}

/// Computes a triangle unit normal with checked subtraction and normalization.
/// Angular epsilon is finite in `0..=1`; collapsed spans return `Degenerate`.
pub fn checked_tri_normal<T: RealScalar>(
    v0: &Vector3<T>,
    v1: &Vector3<T>,
    v2: &Vector3<T>,
    epsilon: T,
) -> Result<Vector3<T>, MathError> {
    if !epsilon.is_finite() || epsilon < <T as Zero>::zero() || epsilon > <T as One>::one() {
        return Err(MathError::InvalidTolerance);
    }

    // Translate vertices into edge directions before applying a dimensionless
    // shape test; the absolute triangle area is not a degeneracy criterion.
    let v10 = Vector3::checked_sub_vv(v1, v0)?;
    let v20 = Vector3::checked_sub_vv(v2, v0)?;
    checked_normal_from_spanning_vectors(&v10, &v20, epsilon)
}

/// Computes the diagonal-derived quad unit normal without validation.
/// Requires independent diagonals; does not establish coplanarity.
pub fn quad_normal<T: RealScalar>(
    v0: &Vector3<T>,
    v1: &Vector3<T>,
    v2: &Vector3<T>,
    v3: &Vector3<T>,
) -> Vector3<T> {
    Vector3::cross(&(*v2 - *v0), &(*v3 - *v1)).normalize()
}

/// Computes the diagonal-derived quad unit normal with checked arithmetic.
/// Angular epsilon is finite in `0..=1`; collapsed spans return `Degenerate`.
pub fn checked_quad_normal<T: RealScalar>(
    v0: &Vector3<T>,
    v1: &Vector3<T>,
    v2: &Vector3<T>,
    v3: &Vector3<T>,
    epsilon: T,
) -> Result<Vector3<T>, MathError> {
    if !epsilon.is_finite() || epsilon < <T as Zero>::zero() || epsilon > <T as One>::one() {
        return Err(MathError::InvalidTolerance);
    }

    // Diagonal magnitudes depend on the caller's coordinate scale, so only
    // their validity and relative orientation participate in this decision.
    let v20 = Vector3::checked_sub_vv(v2, v0)?;
    let v31 = Vector3::checked_sub_vv(v3, v1)?;
    checked_normal_from_spanning_vectors(&v20, &v31, epsilon)
}

/// Builds a checked unit normal from independent finite spanning vectors.
/// Normalizes each direction once, then classifies its sine-angle magnitude.
/// Invalid angular epsilon or collapsed spans are errors before the final normalization.
fn checked_normal_from_spanning_vectors<T: RealScalar>(
    left: &Vector3<T>,
    right: &Vector3<T>,
    epsilon: T,
) -> Result<Vector3<T>, MathError> {
    if !epsilon.is_finite() || epsilon < <T as Zero>::zero() || epsilon > <T as One>::one() {
        return Err(MathError::InvalidTolerance);
    }

    // Decompose each source vector exactly once. The resulting bounded forms
    // are reused for both angular classification and the cross product, while
    // the Option results reject zero and non-finite vectors up front.
    let left_metric = checked_normalized3(left)?;
    let right_metric = checked_normalized3(right)?;

    // Classify the prepared directions before normalizing their cross product.
    // The helper also validates the caller-provided angular tolerance. The
    // shared gate rejects in-tolerance spans, which cannot define a stable
    // plane normal, and invalid arithmetic; only a valid separated pair reaches
    // the unit-cross-product calculation.
    checked_span_gate!(classify_nearly_parallel_normalized3(
        &left_metric,
        &right_metric,
        epsilon,
    ));

    // Cross the unit directions so every intermediate stays bounded. A final
    // robust normalization removes the sine-angle magnitude and returns only
    // the orientation required by plane constructors.
    let left_unit = left_metric.unit();
    let right_unit = right_metric.unit();
    let cross = Vector3::checked_cross(&left_unit, &right_unit)?;
    Ok(checked_normalized3(&cross)?.unit())
}

#[cfg(all(test, any(feature = "std", feature = "libm", feature = "system-libm")))]
mod tests {
    use super::*;
    use crate::queries::Distance;
    #[test]
    pub fn test_barycentric() {
        let v0 = Vector3::new(0.0, 0.0, 0.0);
        let v1 = Vector3::new(0.0, 1.0, 0.0);
        let v2 = Vector3::new(0.0, 0.0, 1.0);

        let tri = Tri3::new([v0, v1, v2]);
        let pp0 = tri.barycentric_coordinates(&v0);
        assert!(f32::abs(pp0.x - 1.0) < 0.01);
        assert!(f32::abs(pp0.y) < 0.01);
        assert!(f32::abs(pp0.z) < 0.01);

        let pp1 = tri.barycentric_coordinates(&v1);
        assert!(f32::abs(pp1.x) < 0.01);
        assert!(f32::abs(pp1.y - 1.0) < 0.01);
        assert!(f32::abs(pp1.z) < 0.01);

        let pp2 = tri.barycentric_coordinates(&v2);
        assert!(f32::abs(pp2.x) < 0.01);
        assert!(f32::abs(pp2.y) < 0.01);
        assert!(f32::abs(pp2.z - 1.0) < 0.01);
    }

    #[test]
    pub fn test_barycentric_translated() {
        // Test with a triangle not at the origin
        let v0 = Vector3::new(1.0, 2.0, 3.0);
        let v1 = Vector3::new(4.0, 2.0, 3.0);
        let v2 = Vector3::new(1.0, 5.0, 3.0);

        let tri = Tri3::new([v0, v1, v2]);

        // Test vertices
        let pp0 = tri.barycentric_coordinates(&v0);
        assert!(f32::abs(pp0.x - 1.0) < 0.001);
        assert!(f32::abs(pp0.y) < 0.001);
        assert!(f32::abs(pp0.z) < 0.001);

        let pp1 = tri.barycentric_coordinates(&v1);
        assert!(f32::abs(pp1.x) < 0.001);
        assert!(f32::abs(pp1.y - 1.0) < 0.001);
        assert!(f32::abs(pp1.z) < 0.001);

        let pp2 = tri.barycentric_coordinates(&v2);
        assert!(f32::abs(pp2.x) < 0.001);
        assert!(f32::abs(pp2.y) < 0.001);
        assert!(f32::abs(pp2.z - 1.0) < 0.001);

        // Test center point
        let center = (v0 + v1 + v2) / 3.0;
        let pp_center = tri.barycentric_coordinates(&center);
        assert!(f32::abs(pp_center.x - 1.0 / 3.0) < 0.001);
        assert!(f32::abs(pp_center.y - 1.0 / 3.0) < 0.001);
        assert!(f32::abs(pp_center.z - 1.0 / 3.0) < 0.001);

        // Verify barycentric coordinates sum to 1
        assert!(f32::abs((pp_center.x + pp_center.y + pp_center.z) - 1.0) < 0.001);
    }

    #[test]
    pub fn test_barycentric_edge_midpoints() {
        let v0 = Vector3::new(0.0, 0.0, 0.0);
        let v1 = Vector3::new(2.0, 0.0, 0.0);
        let v2 = Vector3::new(0.0, 2.0, 0.0);

        let tri = Tri3::new([v0, v1, v2]);

        // Midpoint of edge v0-v1
        let mid01 = (v0 + v1) / 2.0;
        let pp_mid01 = tri.barycentric_coordinates(&mid01);
        assert!(f32::abs(pp_mid01.x - 0.5) < 0.001);
        assert!(f32::abs(pp_mid01.y - 0.5) < 0.001);
        assert!(f32::abs(pp_mid01.z) < 0.001);

        // Midpoint of edge v0-v2
        let mid02 = (v0 + v2) / 2.0;
        let pp_mid02 = tri.barycentric_coordinates(&mid02);
        assert!(f32::abs(pp_mid02.x - 0.5) < 0.001);
        assert!(f32::abs(pp_mid02.y) < 0.001);
        assert!(f32::abs(pp_mid02.z - 0.5) < 0.001);

        // Midpoint of edge v1-v2
        let mid12 = (v1 + v2) / 2.0;
        let pp_mid12 = tri.barycentric_coordinates(&mid12);
        assert!(f32::abs(pp_mid12.x) < 0.001);
        assert!(f32::abs(pp_mid12.y - 0.5) < 0.001);
        assert!(f32::abs(pp_mid12.z - 0.5) < 0.001);
    }

    #[test]
    pub fn test_barycentric_degenerate_triangle() {
        let v0 = Vector3::new(0.0f32, 0.0, 0.0);
        let v1 = Vector3::new(1.0f32, 0.0, 0.0);
        let v2 = Vector3::new(2.0f32, 0.0, 0.0);

        let tri = Tri3::new([v0, v1, v2]);
        let test_point = Vector3::new(0.5f32, 0.0, 0.0);
        assert!(matches!(
            tri.checked_barycentric_coordinates(&test_point),
            Err(MathError::Degenerate)
        ));
    }

    #[test]
    pub fn test_barycentric_interpolation() {
        // Test that we can reconstruct points using barycentric coordinates
        let v0 = Vector3::new(1.0, 0.0, 0.0);
        let v1 = Vector3::new(0.0, 1.0, 0.0);
        let v2 = Vector3::new(0.0, 0.0, 1.0);

        let tri = Tri3::new([v0, v1, v2]);

        // Test arbitrary point inside triangle
        let test_point = Vector3::new(0.2, 0.3, 0.5);
        let bary = tri.barycentric_coordinates(&test_point);

        // Reconstruct the point using barycentric coordinates
        let reconstructed = v0 * bary.x + v1 * bary.y + v2 * bary.z;

        assert!(f32::abs(reconstructed.x - test_point.x) < 0.001);
        assert!(f32::abs(reconstructed.y - test_point.y) < 0.001);
        assert!(f32::abs(reconstructed.z - test_point.z) < 0.001);
    }

    #[test]
    pub fn test_box3_overlap() {
        let a = Box3::new(&Vector3::new(0.0, 0.0, 0.0), &Vector3::new(1.0, 1.0, 1.0));
        let b = Box3::new(&Vector3::new(0.5, 0.5, 0.5), &Vector3::new(1.5, 1.5, 1.5));
        assert!(a.overlap(&b));

        let c = Box3::new(&Vector3::new(2.0, 0.0, 0.0), &Vector3::new(3.0, 1.0, 1.0));
        assert!(!a.overlap(&c));

        let d = Box3::new(&Vector3::new(0.0, 2.0, 0.0), &Vector3::new(1.0, 3.0, 1.0));
        assert!(!a.overlap(&d));

        let e = Box3::new(&Vector3::new(0.0, 0.0, 2.0), &Vector3::new(1.0, 1.0, 3.0));
        assert!(!a.overlap(&e));

        let f = Box3::new(&Vector3::new(1.0, 0.0, 0.0), &Vector3::new(2.0, 1.0, 1.0));
        assert!(a.overlap(&f));
    }

    #[test]
    fn test_line_new_zero_direction() {
        let p = Vector3::new(0.0f32, 0.0, 0.0);
        let d = Vector3::new(0.0f32, 0.0, 0.0);
        assert!(Line::checked_new(&p, &d, EPS_F32).ok().is_none());
    }

    #[test]
    fn test_line_from_start_end_zero_direction() {
        let p = Vector3::new(1.0f32, 2.0, 3.0);
        assert!(Line::checked_from_start_end(&p, &p, EPS_F32).ok().is_none());
    }

    #[test]
    fn test_line_closest_point_valid() {
        let p = Vector3::new(0.0f32, 0.0, 0.0);
        let d = Vector3::new(1.0f32, 0.0, 0.0);
        let line = Line::checked_new(&p, &d, EPS_F32)
            .ok()
            .expect("line should be valid");
        let target = Vector3::new(2.0f32, 1.0, 0.0);
        let (t, closest) = line
            .checked_closest_point_on_line(&target, EPS_F32)
            .ok()
            .expect("closest point should exist");
        assert!((t - 2.0).abs() < 0.001);
        assert!((closest.x - 2.0).abs() < 0.001);
        assert!(closest.y.abs() < 0.001);
        assert!(closest.z.abs() < 0.001);
    }

    #[test]
    fn test_line_normalize_zero_direction() {
        let line = Line {
            p: Vector3::new(0.0f32, 0.0, 0.0),
            d: Vector3::new(0.0f32, 0.0, 0.0),
            t: core::marker::PhantomData,
        };
        assert!(line.checked_normalize(EPS_F32).ok().is_none());
    }

    #[test]
    fn test_line_normalize_valid_direction() {
        let p = Vector3::new(0.0f32, 0.0, 0.0);
        let d = Vector3::new(2.0f32, 0.0, 0.0);
        let line = Line::checked_new(&p, &d, EPS_F32)
            .ok()
            .expect("line should be valid");
        let norm = line
            .checked_normalize(EPS_F32)
            .expect("normalize should succeed");
        assert!((norm.d.length() - 1.0).abs() < 0.001);
    }

    #[test]
    fn test_segment_distance_zero_length() {
        let p = Vector3::new(0.0f32, 0.0, 0.0);
        let seg = Segment::new(&p, &p);
        let target = Vector3::new(1.0f32, 0.0, 0.0);
        assert!(seg
            .checked_distance_with_epsilon(&target, EPS_F32)
            .ok()
            .is_none());
        assert!(seg
            .checked_closest_point_on_segment(&target, EPS_F32)
            .ok()
            .is_none());
    }

    #[test]
    fn test_ray_new_zero_direction() {
        let p = Vector3::new(0.0f32, 0.0, 0.0);
        let d = Vector3::new(0.0f32, 0.0, 0.0);
        assert!(Ray::checked_new(&p, &d, EPS_F32).ok().is_none());
    }

    #[test]
    fn test_sphere_new_abs_radius() {
        let sphere = Sphere3::new(Vector3::new(0.0f32, 0.0, 0.0), -2.5);
        assert!((sphere.radius - 2.5).abs() < 0.001);
    }

    #[test]
    fn test_shortest_segment_parallel_lines() {
        let p0 = Vector3::new(0.0f32, 0.0, 0.0);
        let p1 = Vector3::new(0.0f32, 1.0, 0.0);
        let d = Vector3::new(1.0f32, 0.0, 0.0);
        let l0 = Line::checked_new(&p0, &d, EPS_F32)
            .ok()
            .expect("line should be valid");
        let l1 = Line::checked_new(&p1, &d, EPS_F32)
            .ok()
            .expect("line should be valid");
        assert!(
            checked_shortest_segment3d_between_lines3d(&l0, &l1, EPS_F32)
                .ok()
                .flatten()
                .is_none()
        );
    }

    /// Verifies that every primitive consumer maps the classifier's explicit
    /// invalid state to its documented checked-query failure.
    #[test]
    fn test_angular_classification_invalid_state_fails_primitives_closed() {
        let origin = Vector3::new(0.0f32, 0.0, 0.0);
        let x = Vector3::new(1.0f32, 0.0, 0.0);
        let y = Vector3::new(0.0f32, 1.0, 0.0);
        let negative_z = Vector3::new(0.0f32, 0.0, -1.0);

        // The closest-lines consumer reaches its parallel classifier with two
        // valid, non-parallel directions; only the NaN tolerance is invalid.
        let line_x = Line::checked_new(&origin, &x, EPS_F32)
            .ok()
            .expect("x must define a line");
        let line_y = Line::checked_new(&origin, &y, EPS_F32)
            .ok()
            .expect("y must define a line");
        assert!(
            checked_shortest_segment3d_between_lines3d(&line_x, &line_y, f32::NAN)
                .ok()
                .flatten()
                .is_none()
        );

        // Ray/line-to-plane consumers use perpendicularity against the plane
        // normal. An invalid tolerance must fail before either valid solve can
        // be mistaken for an in- or out-of-tolerance angular result.
        let plane = Plane::new(&Vector3::new(0.0f32, 0.0, 1.0), &origin);
        let start = Vector3::new(0.0f32, 0.0, 1.0);
        let ray = Ray::checked_new(&start, &negative_z, EPS_F32)
            .ok()
            .expect("-z must define a ray");
        let line = Line::checked_new(&start, &negative_z, EPS_F32)
            .ok()
            .expect("-z must define a line");
        assert!(ray
            .checked_intersect_plane(&plane, f32::INFINITY)
            .ok()
            .flatten()
            .is_none());
        assert!(plane
            .checked_intersect_line(&line, 2.0)
            .ok()
            .flatten()
            .is_none());

        // Plane-normal construction consumes the parallel classifier. Its
        // valid right-angle edges still fail when policy itself is negative.
        assert!(checked_tri_normal(&origin, &x, &y, -EPS_F32).ok().is_none());
    }

    /// Verifies that closest-line classification depends on direction angle,
    /// not on either line's arbitrary parameter scale.
    #[test]
    fn test_shortest_segment_is_invariant_to_direction_scale() {
        // These skew perpendicular lines have nonzero closest-point parameters
        // and endpoints separated by one unit along Z. The first scale pair
        // reproduces the former 2*epsilon regression; the others exercise
        // extreme and negative line parameterizations.
        for (scale0, scale1) in [
            (EPS_F32 * 2.0, EPS_F32 * 2.0),
            (1.0e-18, 1.0e18),
            (-1.0e18, 1.0e-18),
        ] {
            let mut line0 = Line::checked_new(
                &Vector3::new(0.0f32, 0.0, 0.0),
                &Vector3::new(1.0, 0.0, 0.0),
                EPS_F32,
            )
            .ok()
            .expect("base line should be valid");
            let mut line1 = Line::checked_new(
                &Vector3::new(1.0f32, 1.0, 1.0),
                &Vector3::new(0.0, 1.0, 0.0),
                EPS_F32,
            )
            .ok()
            .expect("base line should be valid");
            line0.d = line0.d * scale0;
            line1.d = line1.d * scale1;

            let segment = checked_shortest_segment3d_between_lines3d(&line0, &line1, EPS_F32)
                .ok()
                .flatten()
                .expect("perpendicular lines must have a unique closest segment");
            assert!((segment.s.x - 1.0).abs() < 1.0e-5);
            assert!(segment.s.y.abs() < 1.0e-5);
            assert!(segment.s.z.abs() < 1.0e-5);
            assert!((segment.e.x - 1.0).abs() < 1.0e-5);
            assert!(segment.e.y.abs() < 1.0e-5);
            assert!((segment.e.z - 1.0).abs() < 1.0e-5);
        }
    }

    /// Checks that line-plane points remain fixed while the line parameter
    /// changes inversely with direction magnitude.
    #[test]
    fn test_line_plane_intersection_is_scale_invariant() {
        let plane = Plane::new(
            &Vector3::new(0.0f32, 0.0, 1.0),
            &Vector3::new(0.0, 0.0, 0.0),
        );

        // Every direction describes the same infinite line. The geometric hit
        // is (1, 0, 0), while t*scale remains the base parameter 1.
        for scale in [1.0e-18f32, 1.0, 1.0e18, -1.0e18] {
            let mut line = Line::checked_new(
                &Vector3::new(0.0f32, 0.0, 1.0),
                &Vector3::new(1.0, 0.0, -1.0),
                EPS_F32,
            )
            .ok()
            .expect("base line should be valid");
            line.d = line.d * scale;

            let (t, point) = plane
                .checked_intersect_line(&line, EPS_F32)
                .ok()
                .flatten()
                .expect("non-parallel line should intersect the plane");
            assert!((t * scale - 1.0).abs() < 1.0e-5);
            assert!((point.x - 1.0).abs() < 1.0e-5);
            assert!(point.y.abs() < 1.0e-5);
            assert!(point.z.abs() < 1.0e-5);

            // The returned parameter belongs to the stored, scaled direction,
            // not the temporary unit direction used internally. Reconstructing
            // through the public line equation must recover the same hit.
            let parameterized_point = line.p + line.d * t;
            assert!((parameterized_point.x - point.x).abs() < 1.0e-5);
            assert!((parameterized_point.y - point.y).abs() < 1.0e-5);
            assert!((parameterized_point.z - point.z).abs() < 1.0e-5);
        }
    }

    /// Exercises the same line-parameter rescaling contract at f64 exponent
    /// ranges that would overflow a raw squared-length calculation.
    #[test]
    fn test_line_plane_intersection_extreme_f64_scales() {
        let plane = Plane::new(
            &Vector3::new(0.0f64, 0.0, 1.0),
            &Vector3::new(0.0, 0.0, 0.0),
        );

        for scale in [1.0e-150f64, 1.0e150] {
            let mut line = Line::checked_new(
                &Vector3::new(0.0f64, 0.0, 1.0),
                &Vector3::new(1.0, 0.0, -1.0),
                EPS_F64,
            )
            .ok()
            .expect("base line should be valid");
            line.d = line.d * scale;

            let (t, point) = plane
                .checked_intersect_line(&line, EPS_F64)
                .ok()
                .flatten()
                .expect("non-parallel line should intersect the plane");
            assert!((t * scale - 1.0).abs() < 1.0e-12);
            assert!((point.x - 1.0).abs() < 1.0e-12);
            assert!(point.y.abs() < 1.0e-12);
            assert!(point.z.abs() < 1.0e-12);
        }
    }

    /// Rejects a line hit whose physical point is representable but whose
    /// original-direction parameter would round a nonzero value down to zero.
    #[test]
    fn test_line_plane_intersection_rejects_parameter_underflow() {
        let plane = Plane::new(
            &Vector3::new(0.0f32, 0.0, 1.0),
            &Vector3::new(0.0, 0.0, 0.0),
        );
        let smallest_positive = f32::from_bits(1);
        let maximum_downward_direction = Vector3::new(0.0f32, 0.0, -f32::MAX);

        // This line starts one minimum subnormal above the plane. Its physical
        // travel is representable, but dividing that distance by `f32::MAX`
        // rounds the mathematically positive parameter to zero. Returning that
        // zero beside the plane hit would violate `point = p + d * t`.
        let offset_line = Line::checked_new(
            &Vector3::new(0.0f32, 0.0, smallest_positive),
            &maximum_downward_direction,
            EPS_F32,
        )
        .ok()
        .expect("the maximum finite direction must define a line");
        assert!(plane
            .checked_intersect_line(&offset_line, EPS_F32)
            .ok()
            .flatten()
            .is_none());

        // A line that genuinely starts on the plane has physical distance zero,
        // so its exact stored parameter is also zero and remains a valid hit.
        // This distinguishes a real origin hit from the underflow rejected above.
        let on_plane_line = Line::checked_new(
            &Vector3::new(0.0f32, 0.0, 0.0),
            &maximum_downward_direction,
            EPS_F32,
        )
        .ok()
        .expect("the maximum finite direction must define a line");
        let (t, point) = plane
            .checked_intersect_line(&on_plane_line, EPS_F32)
            .ok()
            .flatten()
            .expect("an exactly on-plane line origin must remain a valid hit");
        assert_eq!(t, 0.0);
        assert_eq!(point.x, on_plane_line.p.x);
        assert_eq!(point.y, on_plane_line.p.y);
        assert_eq!(point.z, on_plane_line.p.z);

        // A minimum-subnormal parameter is not inherently invalid. Here the
        // stored direction times that parameter exactly reproduces the two-
        // subnormal physical travel, so the smallest positive `t` must survive.
        let minimum_parameter_line = Line::checked_new(
            &Vector3::new(0.0f32, 0.0, smallest_positive * 2.0),
            &Vector3::new(0.0, 0.0, -2.0),
            EPS_F32,
        )
        .ok()
        .expect("the length-two direction must define a line");
        let (t, point) = plane
            .checked_intersect_line(&minimum_parameter_line, EPS_F32)
            .ok()
            .flatten()
            .expect("a reconstructable minimum-subnormal parameter must be retained");
        assert_eq!(t.to_bits(), smallest_positive.to_bits());
        assert_eq!(point.z, 0.0);
    }

    /// Rejects a staged-division result at the minimum-subnormal boundary when
    /// its nonzero parameter cannot reconstruct the physical intersection point.
    #[test]
    fn test_line_plane_intersection_rejects_unreconstructable_minimum_parameter() {
        let smallest_positive = f32::from_bits(1);
        let direction = Vector3::new(1.6666666f32, 1.3844372, 0.0);
        let plane = Plane::new(&direction, &Vector3::new(0.0, 0.0, 0.0));
        let line = Line::checked_new(
            &Vector3::new(-smallest_positive, 0.0, 0.0),
            &direction,
            EPS_F32,
        )
        .ok()
        .expect("the ordinary finite direction must define a line");

        // Both safe division orderings round the original-scale parameter up to
        // the minimum subnormal even though the physical quotient belongs on
        // the zero side of that boundary. In floating arithmetic, the unit-
        // direction point and `line.p + line.d * t` then differ in x, so no
        // self-consistent `(t, point)` tuple can be returned.
        assert!(plane
            .checked_intersect_line(&line, EPS_F32)
            .ok()
            .flatten()
            .is_none());
    }

    /// Verifies that equivalent non-unit plane equations produce the same line
    /// intersection, including when every coefficient changes sign.
    #[test]
    fn test_line_plane_intersection_is_invariant_to_coefficient_scale() {
        // Both equations describe z = 2. Constructing them directly is possible
        // inside this module and exercises the general coefficient path that the
        // public normalized constructors normally make invisible.
        let plane = Plane {
            a: 0.0f32,
            b: 0.0,
            c: 2.0,
            d: -4.0,
        };
        let negative_scaled_plane = Plane {
            a: 0.0f32,
            b: 0.0,
            c: -6.0,
            d: 12.0,
        };
        let line = Line::checked_new(
            &Vector3::new(1.0f32, 2.0, 5.0),
            &Vector3::new(2.0, 0.0, -3.0),
            EPS_F32,
        )
        .ok()
        .expect("the nonzero direction must define a line");

        // The original line equation reaches (3, 2, 2) at t = 1. Scaling all
        // plane coefficients cannot change either the parameter or the point.
        for candidate in [plane, negative_scaled_plane] {
            let (t, point) = candidate
                .checked_intersect_line(&line, EPS_F32)
                .ok()
                .flatten()
                .expect("the line must intersect either plane representation");
            assert!((t - 1.0).abs() < 1.0e-5);
            assert!((point.x - 3.0).abs() < 1.0e-5);
            assert!((point.y - 2.0).abs() < 1.0e-5);
            assert!((point.z - 2.0).abs() < 1.0e-5);
        }
    }

    /// Ensures angular parallelism has one answer for every positive scaling of
    /// an otherwise identical line direction.
    #[test]
    fn test_line_plane_near_parallel_decision_is_scale_invariant() {
        let plane = Plane::new(
            &Vector3::new(0.0f32, 0.0, 1.0),
            &Vector3::new(0.0, 0.0, 0.0),
        );
        let base_direction = Vector3::new(1.0f32, 0.0, -EPS_F32 * 0.5);

        // The angle is inside the tolerance band. Multiplying the full
        // direction changes no geometry and must not turn the miss into a hit.
        for scale in [1.0f32, 4.0, 1.0e12] {
            let line = Line::checked_new(
                &Vector3::new(0.0f32, 0.0, 1.0),
                &(base_direction * scale),
                EPS_F32,
            )
            .ok()
            .expect("direction length should pass constructor validation");
            assert!(plane
                .checked_intersect_line(&line, EPS_F32)
                .ok()
                .flatten()
                .is_none());
        }
    }

    #[test]
    fn test_ray_intersect_plane_parallel() {
        let ray = Ray::checked_new(
            &Vector3::new(0.0f32, 0.0, 0.0),
            &Vector3::new(1.0, 0.0, 0.0),
            EPS_F32,
        )
        .ok()
        .expect("ray should be valid");
        let plane = Plane::new(
            &Vector3::new(0.0f32, 1.0, 0.0),
            &Vector3::new(0.0, 0.0, 0.0),
        );
        assert!(ray
            .checked_intersect_plane(&plane, EPS_F32)
            .ok()
            .flatten()
            .is_none());
        assert!(plane
            .checked_intersect_ray(&ray, EPS_F32)
            .ok()
            .flatten()
            .is_none());
    }

    #[test]
    fn test_ray_intersect_plane_hit() {
        let ray = Ray::checked_new(
            &Vector3::new(0.0f32, -1.0, 0.0),
            &Vector3::new(0.0, 1.0, 0.0),
            EPS_F32,
        )
        .ok()
        .expect("ray should be valid");
        let plane = Plane::new(
            &Vector3::new(0.0f32, 1.0, 0.0),
            &Vector3::new(0.0, 0.0, 0.0),
        );
        let hit = ray
            .checked_intersect_plane(&plane, EPS_F32)
            .ok()
            .flatten()
            .expect("should hit");
        assert!(hit.y.abs() < 0.001);
    }

    /// Confirms that ray-plane intersection uses direction rather than stored
    /// direction magnitude after construction.
    #[test]
    fn test_ray_plane_intersection_is_scale_invariant() {
        let plane = Plane::new(
            &Vector3::new(0.0f32, 0.0, 1.0),
            &Vector3::new(0.0, 0.0, 0.0),
        );

        // Mutating the public direction field models callers that retain the
        // same ray geometry but choose a different positive parameter scale.
        for scale in [1.0e-18f32, 1.0, 1.0e18] {
            let mut ray = Ray::checked_new(
                &Vector3::new(0.0f32, 0.0, 1.0),
                &Vector3::new(1.0, 0.0, -1.0),
                EPS_F32,
            )
            .ok()
            .expect("base ray should be valid");
            ray.direction = ray.direction * scale;

            let point = ray
                .checked_intersect_plane(&plane, EPS_F32)
                .ok()
                .flatten()
                .expect("forward ray should intersect the plane");
            assert!((point.x - 1.0).abs() < 1.0e-5);
            assert!(point.y.abs() < 1.0e-5);
            assert!(point.z.abs() < 1.0e-5);
        }
    }

    /// Confirms that the scale-invariance guarantee for a ray is intentionally
    /// limited to positive scales because a negative scale reverses its domain.
    #[test]
    fn test_ray_plane_negative_direction_scale_reverses_ray() {
        let plane = Plane::new(
            &Vector3::new(0.0f32, 0.0, 1.0),
            &Vector3::new(0.0, 0.0, 0.0),
        );
        let mut ray = Ray::checked_new(
            &Vector3::new(0.0f32, 0.0, 1.0),
            &Vector3::new(0.0, 0.0, -1.0),
            EPS_F32,
        )
        .ok()
        .expect("the downward direction must define a valid ray");

        // The original ray reaches the plane in its forward domain. Negating
        // the public direction points the ray upward, placing that same plane
        // behind the origin; this must be rejected rather than treated like an
        // infinite-line rescaling.
        assert!(ray
            .checked_intersect_plane(&plane, EPS_F32)
            .ok()
            .flatten()
            .is_some());
        ray.direction = -ray.direction;
        assert!(ray
            .checked_intersect_plane(&plane, EPS_F32)
            .ok()
            .flatten()
            .is_none());
    }

    /// Verifies that malformed direction data fails closed instead of returning
    /// an intersection containing NaN or infinity.
    #[test]
    fn test_plane_intersections_reject_non_finite_directions() {
        let plane = Plane::new(
            &Vector3::new(0.0f32, 0.0, 1.0),
            &Vector3::new(0.0, 0.0, 0.0),
        );
        let mut line = Line::checked_new(
            &Vector3::new(0.0f32, 0.0, 1.0),
            &Vector3::new(0.0, 0.0, -1.0),
            EPS_F32,
        )
        .ok()
        .expect("base line should be valid");
        let mut ray = Ray::checked_new(&line.p, &line.d, EPS_F32)
            .ok()
            .expect("base ray should be valid");

        // Each invalid component must be detected before orientation or division
        // can turn it into an apparently successful result.
        line.d.x = f32::NAN;
        ray.direction.z = f32::INFINITY;
        assert!(plane
            .checked_intersect_line(&line, EPS_F32)
            .ok()
            .flatten()
            .is_none());
        assert!(ray
            .checked_intersect_plane(&plane, EPS_F32)
            .ok()
            .flatten()
            .is_none());
    }

    #[test]
    fn test_plane_try_new_zero_normal_none() {
        let plane = Plane::checked_new(
            &Vector3::new(0.0f32, 0.0, 0.0),
            &Vector3::new(0.0, 0.0, 0.0),
            EPS_F32,
        )
        .ok();
        assert!(plane.is_none());
    }

    #[test]
    fn test_plane_distance_uses_general_equation() {
        let plane = Plane {
            a: 0.0f32,
            b: 2.0,
            c: 0.0,
            d: -4.0,
        };
        let scaled_plane = Plane {
            a: 0.0f32,
            b: -6.0,
            c: 0.0,
            d: 12.0,
        };

        for point in [
            Vector3::new(1.0f32, 5.0, -2.0),
            Vector3::new(1.0f32, -1.0, -2.0),
        ] {
            let distance = plane
                .checked_distance(&point)
                .ok()
                .expect("normal is non-zero");
            let scaled_distance = scaled_plane
                .checked_distance(&point)
                .ok()
                .expect("scaled normal is non-zero");
            assert!((distance - 3.0).abs() < EPS_F32);
            assert!((scaled_distance - distance).abs() < EPS_F32);
        }

        let on_plane = Vector3::new(3.0f32, 2.0, 7.0);
        assert!(
            plane
                .checked_distance(&on_plane)
                .ok()
                .expect("normal is non-zero")
                < EPS_F32
        );
    }

    #[test]
    fn test_plane_distance_rejects_zero_normal() {
        let zero_plane = Plane::<f32>::default();
        assert!(zero_plane
            .checked_distance(&Vector3::new(1.0, 2.0, 3.0))
            .ok()
            .is_none());
    }

    #[test]
    fn test_box3_center_extent_subdivide() {
        let b = Box3::new(
            &Vector3::new(0.0f32, 0.0, 0.0),
            &Vector3::new(2.0, 2.0, 2.0),
        );
        let center = b.center();
        let extent = b.extent();
        assert!((center.x - 1.0).abs() < 0.001);
        assert!((center.y - 1.0).abs() < 0.001);
        assert!((center.z - 1.0).abs() < 0.001);
        assert!((extent.x - 1.0).abs() < 0.001);
        assert!((extent.y - 1.0).abs() < 0.001);
        assert!((extent.z - 1.0).abs() < 0.001);

        let subs = b.subdivide();
        assert_eq!(subs.len(), 8);
        for sub in subs.iter() {
            assert!(sub.min.x >= 0.0 && sub.min.y >= 0.0 && sub.min.z >= 0.0);
            assert!(sub.max.x <= 2.0 && sub.max.y <= 2.0 && sub.max.z <= 2.0);
        }
    }

    #[test]
    fn test_parametric_plane_project() {
        let plane = ParametricPlane::new(
            &Vector3::new(1.0f32, 2.0, 3.0),
            &Vector3::new(1.0, 0.0, 0.0),
            &Vector3::new(0.0, 1.0, 0.0),
        );
        let point = Vector3::new(3.0f32, 6.0, 3.0);
        let uv = plane.project(&point);
        assert!((uv.x - 2.0).abs() < 0.001);
        assert!((uv.y - 4.0).abs() < 0.001);
    }

    #[test]
    fn test_parametric_plane_project_non_orthogonal_axes() {
        let plane = ParametricPlane::new(
            &Vector3::new(0.0f32, 0.0, 0.0),
            &Vector3::new(1.0, 0.0, 0.0),
            &Vector3::new(1.0, 1.0, 0.0),
        );
        let point = Vector3::new(2.0f32, 1.0, 0.0);
        let uv = plane.project(&point);
        assert!((uv.x - 1.0).abs() < 0.001);
        assert!((uv.y - 1.0).abs() < 0.001);
    }

    #[test]
    fn test_parametric_plane_try_normal_parallel_axes_none() {
        let plane = ParametricPlane::new(
            &Vector3::new(0.0f32, 0.0, 0.0),
            &Vector3::new(1.0, 0.0, 0.0),
            &Vector3::new(2.0, 0.0, 0.0),
        );
        assert!(plane.checked_normal(EPS_F32).ok().is_none());
        assert!(plane.checked_plane(EPS_F32).ok().is_none());
    }

    /// Checks that plane-producing cross products use axis orientation rather
    /// than raw area-like magnitudes.
    #[test]
    fn test_cross_derived_normals_are_scale_invariant() {
        let tiny = 1.0e-18f32;

        // The old cross-then-length-squared path underflowed for these vectors,
        // although they form a perfectly conditioned right angle.
        let triangle_normal = checked_tri_normal(
            &Vector3::new(0.0f32, 0.0, 0.0),
            &Vector3::new(tiny, 0.0, 0.0),
            &Vector3::new(0.0, tiny, 0.0),
            EPS_F32,
        )
        .ok()
        .expect("tiny right triangle should have a valid normal");
        assert!(triangle_normal.x.abs() < 1.0e-5);
        assert!(triangle_normal.y.abs() < 1.0e-5);
        assert!((triangle_normal.z - 1.0).abs() < 1.0e-5);

        let parametric = ParametricPlane::new(
            &Vector3::new(0.0f32, 0.0, 0.0),
            &Vector3::new(tiny, 0.0, 0.0),
            &Vector3::new(0.0, tiny, 0.0),
        );
        let normal = parametric
            .checked_normal(EPS_F32)
            .ok()
            .expect("tiny perpendicular axes should span a plane");
        assert!((normal.z - 1.0).abs() < 1.0e-5);
    }
}
