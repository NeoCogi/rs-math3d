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
use crate::primitives::*;
use crate::scalar::*;
use crate::vector::*;
use num_traits::{Zero, One};

////////////////////////////////////////////////////////////////////////////////
/// Query Traits
////////////////////////////////////////////////////////////////////////////////
pub trait Distance<T, Other> {
    fn distance(&self, other: &Other) -> Option<T>;
}

pub trait Intersect<T> {
    fn intersect(&self, other: &T) -> bool;
}

pub trait Intersection<T, Other> {
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
impl<T: FloatScalar> Intersect<Ray<T, Vector3<T>>> for Box3<T> {
    fn intersect(&self, other: &Ray<T, Vector3<T>>) -> bool {
        // From the paper: An Efficient and Robust Ray–Box Intersection Algorithm by A. Williams et. al.
        // "... Note also that since IEEE arithmetic guarantees that a positive number divided by zero
        // is +\infinity and a negative number divided by zero is -\infinity, the code works for vertical
        // and horizontal line ..."

        let mut tmin = -T::infinity();
        let mut tmax = T::infinity();

        let mut t0 = (self.min.x - other.start.x) / other.direction.x;
        let mut t1 = (self.max.x - other.start.x) / other.direction.x;

        tmin = T::max(tmin, T::min(t0, t1));
        tmax = T::min(tmax, T::max(t0, t1));

        t0 = (self.min.y - other.start.y) / other.direction.y;
        t1 = (self.max.y - other.start.y) / other.direction.y;

        tmin = T::max(tmin, T::min(t0, t1));
        tmax = T::min(tmax, T::max(t0, t1));

        t0 = (self.min.z - other.start.z) / other.direction.z;
        t1 = (self.max.z - other.start.z) / other.direction.z;

        tmin = T::max(tmin, T::min(t0, t1));
        tmax = T::min(tmax, T::max(t0, t1));

        tmax > T::max(tmin, <T as Zero>::zero())
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

        r * r > dist
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
            let p = Plane::from_tri(&v0, &v1, &v2);
            if p.distance(&self.center)
                .map_or(false, |dist| dist <= self.radius)
            {
                true
            } else {
                false
            }
        } else {
            let d0 = Segment::new(&v0, &v1).distance(&self.center, T::epsilon());
            let d1 = Segment::new(&v1, &v2).distance(&self.center, T::epsilon());
            let d2 = Segment::new(&v2, &v0).distance(&self.center, T::epsilon());

            match (d0, d1, d2) {
                (Some(d0), Some(d1), Some(d2)) => {
                    let m = T::min(d0, T::min(d1, d2));
                    if m <= self.radius {
                        true
                    } else {
                        false
                    }
                }
                _ => false,
            }
        }
    }
}

///
/// Ray-Triangle Intersection Test Routines
/// Different optimizations of my and Ben Trumbore's
/// code from journals of graphics tools (JGT)
/// <http://www.acm.org/jgt/>
/// by Tomas Moller, May 2000
///
impl<T: FloatScalar> Intersection<(T, Vector3<T>), Tri3<T>> for Ray<T, Vector3<T>> {
    fn intersection(&self, tri: &Tri3<T>) -> Option<(T, Vector3<T>)> {
        let verts = tri.vertices();
        let v0 = verts[0];
        let v1 = verts[1];
        let v2 = verts[2];
        // find vectors for two edges sharing vert0
        let edge1 = v1 - v0;
        let edge2 = v2 - v0;

        // begin calculating determinant - also used to calculate U parameter
        let pvec = Vector3::cross(&self.direction, &edge2);

        // if determinant is near zero, ray lies in plane of triangle
        let det = Vector3::dot(&edge1, &pvec);

        if det > -T::epsilon() && det < T::epsilon() {
            return None; // Parallel
        }

        // calculate distance from vert0 to ray origin
        let tvec = self.start - v0;

        let qvec = Vector3::cross(&tvec, &edge1);

        let u = Vector3::dot(&tvec, &pvec) / det;

        if u < -T::epsilon() || u > <T as One>::one() + T::epsilon() {
            return None; // NoIntersection
        }

        // calculate V parameter and test bounds
        let v = Vector3::dot(&self.direction, &qvec) / det;
        if v < -T::epsilon() || u + v > <T as One>::one() + T::two() * T::epsilon() {
            return None; // NoIntersection
        }

        let t = Vector3::dot(&edge2, &qvec) / det;
        let out = self.start + (self.direction * t);
        Some((t, out)) //Intersect
    }
}

///
/// Building an Orthonormal Basis from a Unit Vector
/// John. F. Hughes & Thomas Moller
/// Journal of Graphics Tools, 4:4, 33-35 (DOI: 10.1080/10867651.1999.10487513)
///
/// The original implementation has issues around edge cases (any of x,y,z = 0)
/// I attempt to fix using the following without proof
/// TODO: do the proof one day!
///
pub fn basis_from_unit<T: FloatScalar>(unit: &Vector3<T>) -> [Vector3<T>; 3] {
    let u = Vector3::normalize(unit);
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
    [u, w, v]
}

#[cfg(test)]
mod tests {
    use super::Intersect;
    use crate::primitives::{Box3, Sphere3};
    use crate::vector::Vector3;

    #[test]
    fn test_box3_sphere_intersect_z_axis() {
        let b = Box3::new(
            &Vector3::new(0.0f32, 0.0, 0.0),
            &Vector3::new(1.0, 1.0, 1.0),
        );

        let outside_z = Sphere3::new(Vector3::new(0.5, 0.5, 2.0), 0.25);
        assert!(!b.intersect(&outside_z));

        let touching_z = Sphere3::new(Vector3::new(0.5, 0.5, 2.0), 1.1);
        assert!(b.intersect(&touching_z));
    }
}
