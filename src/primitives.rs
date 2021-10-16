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
use crate::matrix::*;

////////////////////////////////////////////////////////////////////////////////
/// Rect
////////////////////////////////////////////////////////////////////////////////
#[repr(C)]
#[derive(Debug, Clone, Copy, Default)]
pub struct Dimension<T : Scalar> {
    pub width: T,
    pub height: T
}

impl<T: Scalar> Dimension<T> {
    pub fn new(width: T, height: T) -> Self { Self { width, height} }
}

////////////////////////////////////////////////////////////////////////////////
/// Rect
////////////////////////////////////////////////////////////////////////////////
#[repr(C)]
#[derive(Debug, Clone, Copy, Default)]
pub struct Rect<T : Scalar> {
    pub x: T,
    pub y: T,
    pub width: T,
    pub height: T
}

impl<T: Scalar> Rect<T> {
    pub fn new(x: T, y: T, width: T, height: T) -> Self { Self { x, y, width, height} }
    pub fn from(min_vec: &Vector2<T>, max_vec: &Vector2<T>) -> Self {
        let min_x = T::min(min_vec.x, max_vec.x);
        let min_y = T::min(min_vec.y, max_vec.y);
        let max_x = T::max(min_vec.x, max_vec.x);
        let max_y = T::max(min_vec.y, max_vec.y);
        Self {
            x: min_x,
            y: min_y,
            width:  max_x - min_x,
            height: max_y - min_y
        }
    }

    pub fn min(&self) -> Vector2<T> { Vector2::new(self.x, self.y) }
    pub fn max(&self) -> Vector2<T> { Vector2::new(self.x + self.width, self.y + self.height) }
    pub fn intersect(&self, other: &Self) -> Option<Self> {
        let smx = self.max();
        let smn = self.min();
        let omx = other.max();
        let omn = other.min();

        if smx.x < omn.x || smx.y < omn.y || smn.x > omx.x || smn.y > omx.y {
            return None
        }

        let min_vec = Vector2::max(&self.min(), &other.min());
        let max_vec = Vector2::min(&self.max(), &other.max());
        Some(Self::from(&min_vec, &max_vec))
    }

    pub fn contains(&self, p: &Vector2<T>) -> bool {
        p.x >= self.x && p.y >= self.y && p.x <= self.x + self.width && p.y <= self.y + self.height
    }
}

////////////////////////////////////////////////////////////////////////////////
/// Box3
////////////////////////////////////////////////////////////////////////////////
#[repr(C)]
#[derive(Debug, Clone, Copy, Default)]
pub struct Box3<T : Scalar> {
    pub min: Vector3<T>,
    pub max: Vector3<T>,
}

impl<T: Scalar> Box3<T> {
    pub fn new(v0: &Vector3<T>, v1: &Vector3<T>) -> Self { Self { min: Vector3::min(v0, v1), max: Vector3::max(v0, v1) } }
    pub fn center(&self) -> Vector3<T> { (self.max + self.min) * T::half() }
    pub fn extent(&self) -> Vector3<T> { self.max - self.center() }
    pub fn overlap(&self, other: &Self) -> bool {
        if self.max.x < other.min.x { return false }
        if self.max.y < other.min.y { return false }
        if self.max.z < other.min.z { return false }

        if self.min.x > other.max.x { return false }
        if self.min.x > other.max.x { return false }
        if self.min.x > other.max.x { return false }
        return true
    }

    pub fn add(&self, p: &Vector3<T>) -> Self {
        Self {
            min: Vector3::min(&p, &self.min),
            max: Vector3::max(&p, &self.max),
        }
    }

    pub fn subdivide(&self) -> [Self; 8] {
        let cube_table : [Vector3<i32>; 8] = [
            Vector3::new(0, 1, 0),
            Vector3::new(1, 1, 0),
            Vector3::new(1, 1, 1),
            Vector3::new(0, 1, 1),

            Vector3::new(0, 0, 0),
            Vector3::new(1, 0, 0),
            Vector3::new(1, 0, 1),
            Vector3::new(0, 0, 1)
        ];

        let ps: [Vector3<T>; 2] = [ self.min, self.max ];
        let mut vs = [Vector3::zero(); 8];
        for i in 0..8 {
            vs[i] = Vector3::new(ps[cube_table[i].x as usize].x, ps[cube_table[i].y as usize].y, ps[cube_table[i].z as usize].z);
        }

        let c = self.center();
        let mut out = [Box3 { min: Vector3::zero(), max:Vector3::zero() }; 8];
        for i in 0..8 {
            out[i] = Self::new(&Vector3::min(&c, &vs[i]), &Vector3::max(&c, &vs[i]));
        }
        out
    }
}

////////////////////////////////////////////////////////////////////////////////
/// Line
////////////////////////////////////////////////////////////////////////////////
#[repr(C)]
#[derive(Debug, Clone, Copy, Default)]
pub struct Line<T: Scalar, V: Vector<T>> {
    pub p: V,  // point on the line
    pub d: V,  // direction of the line
    t : core::marker::PhantomData<T>,
}

impl<T: Scalar, V: Vector<T>> Line<T, V> {
    pub fn new(p: &V, d: &V) -> Self { Self { p: *p, d: *d, t: core::marker::PhantomData } }
    pub fn from_start_end(s: &V, e: &V) -> Self { Self { p: *s, d: *e - *s, t: core::marker::PhantomData } }
    pub fn normalize(&self) -> Self { Self { p: self.p, d: Vector::normalize(&self.d), t: core::marker::PhantomData }}
    pub fn closest_point_on_line(&self, p: &V) -> (T, V) {
        let p_dir = *p - self.p;

        let d_sp = V::dot(&self.d, &p_dir);
        let d_ss = V::dot(&self.d, &self.d);

        let t = d_sp / d_ss;

        (t, self.p + self.d * t)
    }
}


pub fn shortest_segment3d_between_lines3d<T: Scalar>(line0: &Line<T, Vector3<T>>, line1: &Line<T, Vector3<T>>, epsilon: T) -> Option<Segment<T, Vector3<T>>> {
    let s0  = line0.p;
    let s1  = line1.p;

    let d1  = line1.d;
    let d0  = line0.d;

    let normal  = Vector3::normalize(&Vector3::cross(&d1, &d0));
    let n0      = Vector3::normalize(&Vector3::cross(&normal, &d0));
    let n1      = Vector3::normalize(&Vector3::cross(&normal, &d1));

    let plane0  = Plane::new(&n0, &s0);
    let plane1  = Plane::new(&n1, &s1);

    let p1      = plane0.intersect_line(line1, epsilon);
    let p0      = plane1.intersect_line(line0, epsilon);

    match (p0, p1) {
        (Some(s), Some(e)) => Some(Segment::new(&s, &e)),
        _ => None
    }
}

////////////////////////////////////////////////////////////////////////////////
/// Segment
////////////////////////////////////////////////////////////////////////////////
#[repr(C)]
#[derive(Clone, Copy, Default)]
pub struct Segment<T : Scalar, V: Vector<T>> {
    pub s: V,  // start
    pub e: V,  // end
    t : core::marker::PhantomData<T>,
}

impl<T: Scalar, V: Vector<T>> Segment<T, V> {
    pub fn new(s: &V, e: &V) -> Self { Self { s: *s, e: *e, t: core::marker::PhantomData } }
    pub fn closest_point_on_segment(&self, p: &V) -> (T, V) {
        let dir = self.e - self.s;
        let p_dir = *p - self.s;

        let d_sp = V::dot(&dir, &p_dir);
        let d_ss = V::dot(&dir, &dir);

        if d_sp < T::zero() {
            return (T::zero(), self.s)
        } else if d_sp > d_ss {
            return (T::one(), self.e)
        }

        let t = d_sp / d_ss;

        (t, self.s + dir * t)
    }

    pub fn distance(&self, p: &V) -> T {
        let (_, p_on_seg) = self.closest_point_on_segment(p);
        V::length(&(p_on_seg - *p))
    }
}



////////////////////////////////////////////////////////////////////////////////
/// Ray
////////////////////////////////////////////////////////////////////////////////
#[repr(C)]
#[derive(Clone, Copy, Default)]
pub struct Ray<T : Scalar, V: Vector<T>> {
    pub start: V,
    pub direction: V,
    t : core::marker::PhantomData<T>,
}

impl<T: Scalar, V: Vector<T>> Ray<T, V> {
    pub fn new(start: &V, direction: &V) -> Self {
        Self { start: *start, direction: V::normalize(direction), t: core::marker::PhantomData }
    }
}

impl<T: Scalar> Ray<T, Vector3<T>> {
    pub fn intersect_plane(&self, p: &Plane<T>) -> Option<Vector3<T>> {
        let n = p.normal();
        let t : T = -(p.d + Vector3::dot(&n, &self.start)) / Vector3::dot(&n, &self.direction);
        if t < T::zero() {
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
    pub center  : Vector3<T>,
    pub radius  : T,
}

impl<T: FloatScalar> Sphere3<T> {
    pub fn new(center: Vector3<T>, radius: T) -> Self {
        Self {
            center : center,
            radius : radius
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
    pub fn new(vertices: [Vector3<T>; 3]) -> Self { Self { vertices: vertices } }
    pub fn vertices(&self) -> &[Vector3<T>; 3] { &self.vertices }

    pub fn barycentric_coordinates(&self, pt: &Vector3<T>) -> Vector3<T> {
        let v0 = self.vertices[0];
        let v1 = self.vertices[1];
        let v2 = self.vertices[2];
        let m = Matrix3::new(v0.x, v0.y, v0.z, v1.x, v1.y, v1.z, v2.x, v2.y, v2.z);
        m.inverse() * *pt
    }
}

////////////////////////////////////////////////////////////////////////////////
/// Plane
////////////////////////////////////////////////////////////////////////////////
#[repr(C)]
#[derive(Clone, Copy, Default)]
pub struct Plane<T : Scalar> {
    a: T,
    b: T,
    c: T,
    d: T
}

impl<T: Scalar> Plane<T> {
    pub fn new(n: &Vector3<T>, p: &Vector3<T>) -> Self { let norm = Vector3::normalize(n); let d = Vector3::dot(&norm, p); Self { a: norm.x, b: norm.y, c: norm.z, d: -d } }
    pub fn normal(&self) -> Vector3<T> { Vector3::new(self.a, self.b, self.c) }
    pub fn constant(&self) -> T { self.d }
    pub fn from_tri(v0: &Vector3<T>, v1: &Vector3<T>, v2: &Vector3<T>) -> Self { let n = tri_normal(v0, v1, v2); Self::new(&n, v0) }
    pub fn from_quad(v0: &Vector3<T>, v1: &Vector3<T>, v2: &Vector3<T>, v3: &Vector3<T>) -> Self {
        let n = quad_normal(v0, v1, v2, v3);
        let c = (*v0 + *v1 + *v2 + *v3) * T::quarter();
        Self::new(&n, &c)
    }

    pub fn intersect_ray(&self, r: &Ray<T, Vector3<T>>) -> Option<Vector3<T>> {
        r.intersect_plane(self)
    }

    pub fn intersect_line(&self, line: &Line<T, Vector3<T>>, epsilon: T) -> Option<Vector3<T>> {
        let s = line.p;
        let dir = line.d;
        let n = self.normal();

        let denom = Vector3::dot(&n, &dir);
        if denom.tabs() < epsilon {
            None
        } else {
            let t = -(self.constant() + Vector3::dot(&n, &s)) / denom;
            Some(dir * t + s)
        }
    }
}


////////////////////////////////////////////////////////////////////////////////
/// Parametric Plane
////////////////////////////////////////////////////////////////////////////////
#[repr(C)]
#[derive(Clone, Copy, Default)]
pub struct ParametricPlane<T : Scalar> {
    pub center  : Vector3<T>,
    pub x_axis  : Vector3<T>,
    pub y_axis  : Vector3<T>,
}

impl<T: Scalar> ParametricPlane<T> {

    pub fn new(center: &Vector3<T>, x_axis: &Vector3<T>, y_axis: &Vector3<T>) -> Self {
        Self { center: *center, x_axis: *x_axis, y_axis: *y_axis }
    }

    pub fn plane(&self) -> Plane<T> {
        Plane::new(&self.normal(), &self.center)
    }

    pub fn normal(&self) -> Vector3<T> {
        Vector3::cross(&self.x_axis, &self.y_axis).normalize()
    }

    pub fn intersect_ray(&self, r: &Ray<T, Vector3<T>>) -> Option<Vector3<T>> {
        r.intersect_plane(&self.plane())
    }

    pub fn intersect_line(&self, line: &Line<T, Vector3<T>>, epsilon: T) -> Option<Vector3<T>> {
        self.plane().intersect_line(line, epsilon)
    }

    pub fn project(&self, v: &Vector3<T>) -> Vector2<T> {
        let p       = *v - self.center;
        let x_coord = dot(&p, &self.x_axis) / dot(&self.x_axis, &self.x_axis);
        let y_coord = dot(&p, &self.y_axis) / dot(&self.y_axis, &self.y_axis);
        Vector2::new(x_coord, y_coord)
    }
}

////////////////////////////////////////////////////////////////////////////////
/// Tri
////////////////////////////////////////////////////////////////////////////////
pub fn tri_normal<T: Scalar>(v0: &Vector3<T>, v1: &Vector3<T>, v2: &Vector3<T>) -> Vector3<T> {
    let v10 = *v1 - *v0;
    let v20 = *v2 - *v0;
    Vector3::normalize(&Vector3::cross(&v10, &v20))
}

////////////////////////////////////////////////////////////////////////////////
/// Tri
////////////////////////////////////////////////////////////////////////////////
pub fn quad_normal<T: Scalar>(v0: &Vector3<T>, v1: &Vector3<T>, v2: &Vector3<T>, v3: &Vector3<T>) -> Vector3<T> {
    let v20 = *v2 - *v0;
    let v31 = *v3 - *v1;
    Vector3::normalize(&Vector3::cross(&v20, &v31))
}
