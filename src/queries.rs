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
use crate::scalar::*;
use crate::vector::*;
use crate::primitives::*;

////////////////////////////////////////////////////////////////////////////////
/// Query Traits
////////////////////////////////////////////////////////////////////////////////
pub trait Distance<T, Other> {
    fn distance(&self, other: &Other) -> T;
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
impl<T: FloatNum> Distance<T, Vector3<T>> for Plane<T> {
    fn distance(&self, other: &Vector3<T>) -> T {
        let n = self.normal();
        let nom = Vector3::dot(other, &n) + self.constant();
        let denom = Vector3::dot(&n, &n);
        T::tabs(nom) / denom
    }
}

impl<T: FloatNum> Distance<T, Vector3<T>> for Segment3<T> {
    fn distance(&self, other: &Vector3<T>) -> T {
        let segDir  = self.e - self.s;
        let ptDir   = *other - self.s;
        let dSP     = Vector3::dot(&segDir, &ptDir);
        let dSS     = Vector3::dot(&segDir, &segDir);

        if dSP < T::zero() {
            return Vector3::length(&ptDir);
        } else if dSP > dSS {
            return Vector3::length(&(*other - self.e));
        }

        let t = dSP / dSS;
        Vector3::length(&(*other - (self.s + segDir * t)))
    }
}

////////////////////////////////////////////////////////////////////////////////
/// Intersect Queries
////////////////////////////////////////////////////////////////////////////////
impl<T: FloatNum> Intersect<Ray3<T>> for Box3<T> {
    fn intersect(&self, other: &Ray3<T>) -> bool {
        // From the paper: An Efficient and Robust Ray–Box Intersection Algorithm by A. Williams et. al.
        // "... Note also that since IEEE arithmetic guarantees that a positive number divided by zero
        // is +\infinity and a negative number divided by zero is -\infinity, the code works for vertical
        // and horizontal line ..."

        let mut tmin = -T::INFINITY();
        let mut tmax = T::INFINITY();

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

        tmax > T::max(tmin, T::zero())
    }
}

impl<T: FloatNum> Intersect<Sphere3<T>> for Box3<T> {
    fn intersect(&self, other: &Sphere3<T>) -> bool {
        let mut dist = T::zero();

        let c = other.center;
        let r = other.radius;

        if      c.x < self.min.x { dist += T::squared(c.x - self.min.x) }
        else if c.x > self.max.x { dist += T::squared(c.x - self.max.x) }

        if      c.y < self.min.y { dist += T::squared(c.y - self.min.y) }
        else if c.y > self.max.y { dist += T::squared(c.y - self.max.y) }

        if      c.x < self.min.z { dist += T::squared(c.z - self.min.z) }
        else if c.x > self.max.z { dist += T::squared(c.z - self.max.z) }

        r * r > dist
    }
}


fn isIn0to1Range<T: FloatNum>(x: T) -> bool {
    x >= T::zero() && x <= T::one()
}

impl<T: FloatNum> Intersect<Tri3<T>> for Sphere3<T> {
    fn intersect(&self, tri: &Tri3<T>) -> bool {
        let uvw = tri.barycentricCoordinates(&self.center);
        let verts = tri.vertices();
        let v0 = verts[0];
        let v1 = verts[1];
        let v2 = verts[2];
        if isIn0to1Range(uvw.x) && isIn0to1Range(uvw.y) && isIn0to1Range(uvw.z) {
            let p = Plane::fromTri(&v0, &v1, &v2);
            if p.distance(&self.center) <= self.radius {
                true
            } else {
                false
            }
        } else {
            let d0 = Segment3::new(&v0, &v1).distance(&self.center);
            let d1 = Segment3::new(&v1, &v2).distance(&self.center);
            let d2 = Segment3::new(&v2, &v0).distance(&self.center);

            let m = T::min(d0, T::min(d1, d2));
            if m <= self.radius {
                true
            } else {
                false
            }
        }
    }
}

///
/// Ray-Triangle Intersection Test Routines
/// Different optimizations of my and Ben Trumbore's
/// code from journals of graphics tools (JGT)
/// http://www.acm.org/jgt/
/// by Tomas Moller, May 2000
///
impl<T: FloatNum> Intersection<Vector3<T>, Tri3<T>> for Ray3<T> {
    fn intersection(&self, tri: &Tri3<T>) -> Option<Vector3<T>> {
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
        let	tvec = self.start - v0;

        let qvec = Vector3::cross(&tvec, &edge1);

        let u = Vector3::dot(&tvec, &pvec) / det;

        if u < -T::epsilon() || u > T::one() + T::epsilon() {
            return None; // NoIntersection
        }

        // calculate V parameter and test bounds
        let v = Vector3::dot(&self.direction, &qvec) / det;
        if v < -T::epsilon() || u + v > T::one() + T::two() * T::epsilon() {
            return None; // NoIntersection
        }

        let t = Vector3::dot(&edge2, &qvec) / det;
        let out = self.start + (self.direction * t);
        Some(out) //Intersect
    }
}