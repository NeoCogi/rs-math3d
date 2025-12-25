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
//! # Examples
//!
//! ```
//! use rs_math3d::primitives::{Ray, Plane, Tri3};
//! use rs_math3d::vector::Vector3;
//! use rs_math3d::EPS_F32;
//! 
//! // Create a ray from origin pointing along +X axis
//! let ray = Ray::new(
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
//! ```

use crate::matrix::*;
use crate::scalar::*;
use crate::vector::*;
use num_traits::{Zero, One};

/// A 2D dimension with width and height.
#[repr(C)]
#[derive(Debug, Clone, Copy, Default)]
pub struct Dimension<T: Scalar> {
    pub width: T,
    pub height: T,
}

impl<T: Scalar> Dimension<T> {
    /// Creates a new dimension with the specified width and height.
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
    pub x: T,
    pub y: T,
    pub width: T,
    pub height: T,
}

impl<T: Scalar> Rect<T> {
    /// Creates a new rectangle from position and dimensions.
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
    pub min: Vector3<T>,
    pub max: Vector3<T>,
}

impl<T: Scalar> Box3<T> {
    /// Creates a new box from two corner points.
    ///
    /// The points are automatically sorted to find min/max.
    pub fn new(v0: &Vector3<T>, v1: &Vector3<T>) -> Self {
        Self {
            min: Vector3::min(v0, v1),
            max: Vector3::max(v0, v1),
        }
    }
    /// Returns the center point of the box.
    pub fn center(&self) -> Vector3<T> {
        (self.max + self.min) * T::half()
    }
    /// Returns the half-extents of the box from center to max.
    pub fn extent(&self) -> Vector3<T> {
        self.max - self.center()
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
        return true;
    }

    /// Expands the box to include a point.
    pub fn add(&self, p: &Vector3<T>) -> Self {
        Self {
            min: Vector3::min(&p, &self.min),
            max: Vector3::max(&p, &self.max),
        }
    }

    /// Subdivides the box into 8 octants.
    ///
    /// Returns an array of 8 boxes representing the octree subdivision.
    pub fn subdivide(&self) -> [Self; 8] {
        let cube_table: [Vector3<i32>; 8] = [
            Vector3::new(0, 1, 0),
            Vector3::new(1, 1, 0),
            Vector3::new(1, 1, 1),
            Vector3::new(0, 1, 1),
            Vector3::new(0, 0, 0),
            Vector3::new(1, 0, 0),
            Vector3::new(1, 0, 1),
            Vector3::new(0, 0, 1),
        ];

        let ps: [Vector3<T>; 2] = [self.min, self.max];
        let mut vs = [Vector3::zero(); 8];
        for i in 0..8 {
            vs[i] = Vector3::new(
                ps[cube_table[i].x as usize].x,
                ps[cube_table[i].y as usize].y,
                ps[cube_table[i].z as usize].z,
            );
        }

        let c = self.center();
        let mut out = [Box3 {
            min: Vector3::zero(),
            max: Vector3::zero(),
        }; 8];
        for i in 0..8 {
            out[i] = Self::new(&Vector3::min(&c, &vs[i]), &Vector3::max(&c, &vs[i]));
        }
        out
    }
}

/// An infinite line in 2D or 3D space.
///
/// Defined by a point on the line and a direction vector.
#[repr(C)]
#[derive(Debug, Clone, Copy, Default)]
pub struct Line<T: Scalar, V: Vector<T>> {
    pub p: V, // point on the line
    pub d: V, // direction of the line
    t: core::marker::PhantomData<T>,
}

impl<T: Scalar, V: Vector<T>> Line<T, V> {
    /// Creates a new line from a point and direction.
    ///
    /// Returns `None` if the direction is too small.
    pub fn new(p: &V, d: &V, epsilon: T) -> Option<Self> {
        let d_ss = V::dot(d, d);
        if d_ss <= epsilon * epsilon {
            return None;
        }
        Some(Self {
            p: *p,
            d: *d,
            t: core::marker::PhantomData,
        })
    }
    /// Creates a line from two points.
    ///
    /// The line passes through both points, with direction from s to e.
    ///
    /// Returns `None` if the points are too close.
    pub fn from_start_end(s: &V, e: &V, epsilon: T) -> Option<Self> {
        let dir = *e - *s;
        Self::new(s, &dir, epsilon)
    }
    /// Finds the closest point on the line to a given point.
    ///
    /// Returns:
    /// - `t`: Parameter value where the closest point occurs
    /// - Point: The closest point on the line
    ///
    /// Returns `None` if the line direction is too small.
    pub fn closest_point_on_line(&self, p: &V, epsilon: T) -> Option<(T, V)> {
        let p_dir = *p - self.p;

        let d_sp = V::dot(&self.d, &p_dir);
        let d_ss = V::dot(&self.d, &self.d);

        if d_ss <= epsilon * epsilon {
            return None;
        }

        let t = d_sp / d_ss;

        Some((t, self.p + self.d * t))
    }
}

impl<T: FloatScalar, V: FloatVector<T>> Line<T, V> {
    /// Returns a line with normalized direction vector.
    ///
    /// Returns `None` if the direction is too small.
    pub fn normalize(&self, epsilon: T) -> Option<Self> {
        let d_ss = V::dot(&self.d, &self.d);
        if d_ss <= epsilon * epsilon {
            return None;
        }
        let inv_len = <T as One>::one() / d_ss.tsqrt();
        Some(Self {
            p: self.p,
            d: self.d * inv_len,
            t: core::marker::PhantomData,
        })
    }
}

/// Finds the shortest segment connecting two 3D lines.
///
/// Returns `None` if the lines are parallel (within epsilon tolerance).
pub fn shortest_segment3d_between_lines3d<T: FloatScalar>(
    line0: &Line<T, Vector3<T>>,
    line1: &Line<T, Vector3<T>>,
    epsilon: T,
) -> Option<Segment<T, Vector3<T>>> {
    let s0 = line0.p;
    let s1 = line1.p;

    let d1 = line1.d;
    let d0 = line0.d;

    let eps_sq = epsilon * epsilon;
    let d0_len_sq = Vector3::dot(&d0, &d0);
    let d1_len_sq = Vector3::dot(&d1, &d1);
    if d0_len_sq <= eps_sq || d1_len_sq <= eps_sq {
        return None;
    }

    let cross = Vector3::cross(&d1, &d0);
    let cross_len_sq = Vector3::dot(&cross, &cross);
    if cross_len_sq <= eps_sq {
        return None;
    }

    let normal = Vector3::normalize(&cross);
    let n0 = Vector3::normalize(&Vector3::cross(&normal, &d0));
    let n1 = Vector3::normalize(&Vector3::cross(&normal, &d1));

    let plane0 = Plane::new(&n0, &s0);
    let plane1 = Plane::new(&n1, &s1);

    let p1 = plane0.intersect_line(line1, epsilon);
    let p0 = plane1.intersect_line(line0, epsilon);

    match (p0, p1) {
        (Some((_, s)), Some((_, e))) => Some(Segment::new(&s, &e)),
        _ => None,
    }
}

/// A line segment with defined start and end points.
#[repr(C)]
#[derive(Clone, Copy, Default)]
pub struct Segment<T: Scalar, V: Vector<T>> {
    pub s: V, // start
    pub e: V, // end
    t: core::marker::PhantomData<T>,
}

impl<T: Scalar, V: Vector<T>> Segment<T, V> {
    /// Creates a new segment from start to end point.
    pub fn new(s: &V, e: &V) -> Self {
        Self {
            s: *s,
            e: *e,
            t: core::marker::PhantomData,
        }
    }
    /// Finds the closest point on the segment to a given point.
    ///
    /// Returns:
    /// - `t`: Parameter value \[0,1\] where 0=start, 1=end
    /// - Point: The closest point on the segment
    ///
    /// Returns `None` if the segment is too small.
    pub fn closest_point_on_segment(&self, p: &V, epsilon: T) -> Option<(T, V)> {
        let dir = self.e - self.s;
        let p_dir = *p - self.s;

        let d_sp = V::dot(&dir, &p_dir);
        let d_ss = V::dot(&dir, &dir);

        if d_ss <= epsilon * epsilon {
            return None;
        }

        if d_sp < <T as Zero>::zero() {
            return Some((<T as Zero>::zero(), self.s));
        } else if d_sp > d_ss {
            return Some((<T as One>::one(), self.e));
        }

        let t = d_sp / d_ss;

        Some((t, self.s + dir * t))
    }
}

impl<T: FloatScalar, V: FloatVector<T>> Segment<T, V> {
    /// Computes the distance from a point to the segment.
    ///
    /// Returns `None` if the segment is too small.
    pub fn distance(&self, p: &V, epsilon: T) -> Option<T> {
        self.closest_point_on_segment(p, epsilon)
            .map(|(_, p_on_seg)| V::length(&(p_on_seg - *p)))
    }
}

/// A ray with an origin and direction.
///
/// Commonly used for ray casting and intersection tests.
#[repr(C)]
#[derive(Clone, Copy, Default)]
pub struct Ray<T: Scalar, V: Vector<T>> {
    pub start: V,
    pub direction: V,
    t: core::marker::PhantomData<T>,
}

impl<T: FloatScalar, V: FloatVector<T>> Ray<T, V> {
    /// Creates a new ray with normalized direction.
    ///
    /// Returns `None` if the direction is too small.
    pub fn new(start: &V, direction: &V, epsilon: T) -> Option<Self> {
        let d_ss = V::dot(direction, direction);
        if d_ss <= epsilon * epsilon {
            return None;
        }
        let inv_len = <T as One>::one() / d_ss.tsqrt();
        Some(Self {
            start: *start,
            direction: *direction * inv_len,
            t: core::marker::PhantomData,
        })
    }
}

impl<T: Scalar> Ray<T, Vector3<T>> {
    /// Computes ray-plane intersection.
    ///
    /// Returns the intersection point, or `None` if the ray doesn't hit the plane
    /// (parallel or pointing away).
    pub fn intersect_plane(&self, p: &Plane<T>) -> Option<Vector3<T>> {
        let n = p.normal();
        let t: T = -(p.d + Vector3::dot(&n, &self.start)) / Vector3::dot(&n, &self.direction);
        if t < <T as Zero>::zero() {
            None
        } else {
            Some(self.direction * t + self.start)
        }
    }
}

////////////////////////////////////////////////////////////////////////////////
/// Sphere3
////////////////////////////////////////////////////////////////////////////////
#[repr(C)]
#[derive(Clone, Copy, Default)]
pub struct Sphere3<T: FloatScalar> {
    pub center: Vector3<T>,
    pub radius: T,
}

impl<T: FloatScalar> Sphere3<T> {
    pub fn new(center: Vector3<T>, radius: T) -> Self {
        Self {
            center: center,
            radius: radius,
        }
    }
}

////////////////////////////////////////////////////////////////////////////////
/// Triangle
////////////////////////////////////////////////////////////////////////////////
#[repr(C)]
#[derive(Clone, Copy, Default)]
pub struct Tri3<T: FloatScalar> {
    vertices: [Vector3<T>; 3],
}

impl<T: FloatScalar> Tri3<T> {
    pub fn new(vertices: [Vector3<T>; 3]) -> Self {
        Self { vertices: vertices }
    }
    pub fn vertices(&self) -> &[Vector3<T>; 3] {
        &self.vertices
    }

    pub fn barycentric_coordinates(&self, pt: &Vector3<T>) -> Vector3<T> {
        let v0 = self.vertices[0];
        let v1 = self.vertices[1];
        let v2 = self.vertices[2];
        let vv1 = v1 - v0;
        let vv2 = v2 - v0;
        let vvc = Vector3::cross(&vv1, &vv2);
        let m = Matrix3::new(
            vv1.x, vv1.y, vv1.z, vv2.x, vv2.y, vv2.z, vvc.x, vvc.y, vvc.z,
        );
        let lambda = m.inverse() * (*pt - v0);
        Vector3::new(<T as One>::one() - lambda.x - lambda.y, lambda.x, lambda.y)
    }
}

////////////////////////////////////////////////////////////////////////////////
/// Plane
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
    pub fn normal(&self) -> Vector3<T> {
        Vector3::new(self.a, self.b, self.c)
    }
    pub fn constant(&self) -> T {
        self.d
    }

    pub fn intersect_ray(&self, r: &Ray<T, Vector3<T>>) -> Option<Vector3<T>> {
        r.intersect_plane(self)
    }

    /// Computes line-plane intersection.
    ///
    /// Returns the parameter t and intersection point, or `None` if parallel.
    pub fn intersect_line(
        &self,
        line: &Line<T, Vector3<T>>,
        epsilon: T,
    ) -> Option<(T, Vector3<T>)> {
        let s = line.p;
        let dir = line.d;
        let n = self.normal();

        let denom = Vector3::dot(&n, &dir);
        if denom.tabs() < epsilon {
            None
        } else {
            let t = -(self.constant() + Vector3::dot(&n, &s)) / denom;
            Some((t, dir * t + s))
        }
    }
}

impl<T: FloatScalar> Plane<T> {
    pub fn new(n: &Vector3<T>, p: &Vector3<T>) -> Self {
        let norm = Vector3::normalize(n);
        let d = Vector3::dot(&norm, p);
        Self {
            a: norm.x,
            b: norm.y,
            c: norm.z,
            d: -d,
        }
    }

    pub fn from_tri(v0: &Vector3<T>, v1: &Vector3<T>, v2: &Vector3<T>) -> Self {
        let n = tri_normal(v0, v1, v2);
        Self::new(&n, v0)
    }
    pub fn from_quad(v0: &Vector3<T>, v1: &Vector3<T>, v2: &Vector3<T>, v3: &Vector3<T>) -> Self {
        let n = quad_normal(v0, v1, v2, v3);
        let c = (*v0 + *v1 + *v2 + *v3) * T::quarter();
        Self::new(&n, &c)
    }
}

////////////////////////////////////////////////////////////////////////////////
/// Parametric Plane
////////////////////////////////////////////////////////////////////////////////
#[repr(C)]
#[derive(Clone, Copy, Default)]
pub struct ParametricPlane<T: Scalar> {
    pub center: Vector3<T>,
    pub x_axis: Vector3<T>,
    pub y_axis: Vector3<T>,
}

impl<T: FloatScalar> ParametricPlane<T> {
    pub fn new(center: &Vector3<T>, x_axis: &Vector3<T>, y_axis: &Vector3<T>) -> Self {
        Self {
            center: *center,
            x_axis: *x_axis,
            y_axis: *y_axis,
        }
    }

    pub fn plane(&self) -> Plane<T> {
        Plane::new(&self.normal(), &self.center)
    }

    /// Computes the normal vector (cross product of axes).
    pub fn normal(&self) -> Vector3<T> {
        Vector3::cross(&self.x_axis, &self.y_axis).normalize()
    }

    pub fn intersect_ray(&self, r: &Ray<T, Vector3<T>>) -> Option<Vector3<T>> {
        r.intersect_plane(&self.plane())
    }

    pub fn intersect_line(
        &self,
        line: &Line<T, Vector3<T>>,
        epsilon: T,
    ) -> Option<(T, Vector3<T>)> {
        self.plane().intersect_line(line, epsilon)
    }

    pub fn project(&self, v: &Vector3<T>) -> Vector2<T> {
        let p = *v - self.center;
        let x_coord = dot(&p, &self.x_axis) / dot(&self.x_axis, &self.x_axis);
        let y_coord = dot(&p, &self.y_axis) / dot(&self.y_axis, &self.y_axis);
        Vector2::new(x_coord, y_coord)
    }
}

////////////////////////////////////////////////////////////////////////////////
/// Tri
////////////////////////////////////////////////////////////////////////////////
pub fn tri_normal<T: FloatScalar>(v0: &Vector3<T>, v1: &Vector3<T>, v2: &Vector3<T>) -> Vector3<T> {
    let v10 = *v1 - *v0;
    let v20 = *v2 - *v0;
    Vector3::normalize(&Vector3::cross(&v10, &v20))
}

/// Computes the normal vector of a quadrilateral.
///
/// Uses the cross product of the two diagonals for a more stable result.
pub fn quad_normal<T: FloatScalar>(
    v0: &Vector3<T>,
    v1: &Vector3<T>,
    v2: &Vector3<T>,
    v3: &Vector3<T>,
) -> Vector3<T> {
    let v20 = *v2 - *v0;
    let v31 = *v3 - *v1;
    Vector3::normalize(&Vector3::cross(&v20, &v31))
}

#[cfg(test)]
mod tests {
    use super::*;
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
        assert!(f32::abs(pp_center.x - 1.0/3.0) < 0.001);
        assert!(f32::abs(pp_center.y - 1.0/3.0) < 0.001);
        assert!(f32::abs(pp_center.z - 1.0/3.0) < 0.001);
        
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
        let a = Box3::new(
            &Vector3::new(0.0, 0.0, 0.0),
            &Vector3::new(1.0, 1.0, 1.0),
        );
        let b = Box3::new(
            &Vector3::new(0.5, 0.5, 0.5),
            &Vector3::new(1.5, 1.5, 1.5),
        );
        assert!(a.overlap(&b));

        let c = Box3::new(
            &Vector3::new(2.0, 0.0, 0.0),
            &Vector3::new(3.0, 1.0, 1.0),
        );
        assert!(!a.overlap(&c));

        let d = Box3::new(
            &Vector3::new(0.0, 2.0, 0.0),
            &Vector3::new(1.0, 3.0, 1.0),
        );
        assert!(!a.overlap(&d));

        let e = Box3::new(
            &Vector3::new(0.0, 0.0, 2.0),
            &Vector3::new(1.0, 1.0, 3.0),
        );
        assert!(!a.overlap(&e));

        let f = Box3::new(
            &Vector3::new(1.0, 0.0, 0.0),
            &Vector3::new(2.0, 1.0, 1.0),
        );
        assert!(a.overlap(&f));
    }

    #[test]
    fn test_line_new_zero_direction() {
        let p = Vector3::new(0.0f32, 0.0, 0.0);
        let d = Vector3::new(0.0f32, 0.0, 0.0);
        assert!(Line::new(&p, &d, EPS_F32).is_none());
    }

    #[test]
    fn test_line_from_start_end_zero_direction() {
        let p = Vector3::new(1.0f32, 2.0, 3.0);
        assert!(Line::from_start_end(&p, &p, EPS_F32).is_none());
    }

    #[test]
    fn test_line_closest_point_valid() {
        let p = Vector3::new(0.0f32, 0.0, 0.0);
        let d = Vector3::new(1.0f32, 0.0, 0.0);
        let line = Line::new(&p, &d, EPS_F32).expect("line should be valid");
        let target = Vector3::new(2.0f32, 1.0, 0.0);
        let (t, closest) = line
            .closest_point_on_line(&target, EPS_F32)
            .expect("closest point should exist");
        assert!((t - 2.0).abs() < 0.001);
        assert!((closest.x - 2.0).abs() < 0.001);
        assert!(closest.y.abs() < 0.001);
        assert!(closest.z.abs() < 0.001);
    }

    #[test]
    fn test_segment_distance_zero_length() {
        let p = Vector3::new(0.0f32, 0.0, 0.0);
        let seg = Segment::new(&p, &p);
        let target = Vector3::new(1.0f32, 0.0, 0.0);
        assert!(seg.distance(&target, EPS_F32).is_none());
        assert!(seg.closest_point_on_segment(&target, EPS_F32).is_none());
    }

    #[test]
    fn test_ray_new_zero_direction() {
        let p = Vector3::new(0.0f32, 0.0, 0.0);
        let d = Vector3::new(0.0f32, 0.0, 0.0);
        assert!(Ray::new(&p, &d, EPS_F32).is_none());
    }

    #[test]
    fn test_shortest_segment_parallel_lines() {
        let p0 = Vector3::new(0.0f32, 0.0, 0.0);
        let p1 = Vector3::new(0.0f32, 1.0, 0.0);
        let d = Vector3::new(1.0f32, 0.0, 0.0);
        let l0 = Line::new(&p0, &d, EPS_F32).expect("line should be valid");
        let l1 = Line::new(&p1, &d, EPS_F32).expect("line should be valid");
        assert!(shortest_segment3d_between_lines3d(&l0, &l1, EPS_F32).is_none());
    }
}
