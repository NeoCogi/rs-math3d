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

//! Example-only, allocation-free discrete overlap queries for a small physics engine.
//!
//! All ten unordered pairs of boxes, spheres, capsules and triangles are covered.
//! Arithmetic is ordinary (unchecked), and contact counts as overlap. Supply finite
//! coordinates, nonnegative sizes, orthonormal box axes and nondegenerate triangles.
//! Every intermediate must fit, including squared distances and products of dot
//! products. Fixed-point divisions and projections must retain enough precision;
//! rounding can change decisions near contact or parallelism. The contact module
//! adds bounded witness manifolds for the physics example; dynamics is owned by
//! that separate module. There is no CCD. The optional static soup tree allocates
//! during construction; its queries do not allocate.

// Consumers select different example workloads from this shared module.
#[allow(dead_code)]
pub mod aabb_tree;
/// Contact witnesses and manifolds used by the rigid-body example.
#[allow(dead_code)]
pub mod contact;

use rs_math3d::{CrossProduct, RealScalar, Sphere3, Tri3, Vector, Vector3};

/// An oriented box with three orthonormal world-space axes.
#[derive(Clone, Copy)]
pub struct Obb<T: RealScalar> {
    /// World-space center.
    pub center: Vector3<T>,
    /// Unit axes, in the same order as the half extents.
    pub axes: [Vector3<T>; 3],
    /// Nonnegative half sizes along the local axes.
    pub half: Vector3<T>,
}

/// A closed segment swept by a sphere. Coincident endpoints form a sphere.
#[derive(Clone, Copy)]
pub struct Capsule<T: RealScalar> {
    /// First centerline endpoint in world space.
    pub a: Vector3<T>,
    /// Second centerline endpoint in world space.
    pub b: Vector3<T>,
    /// Nonnegative sweep radius.
    pub radius: T,
}

/// Dot product with the library's scalar-specific accumulation and rounding.
fn dot<T: RealScalar>(a: Vector3<T>, b: Vector3<T>) -> T {
    Vector3::dot(&a, &b)
}

/// Squared length; avoids square roots in every overlap query.
fn squared<T: RealScalar>(v: Vector3<T>) -> T {
    dot(v, v)
}

/// Clamps a scalar to a closed interval.
fn clamp<T: RealScalar>(x: T, lo: T, hi: T) -> T {
    T::max(lo, T::min(x, hi))
}

/// Converts a world-space point to an orthonormal box's local coordinates.
fn local<T: RealScalar>(b: &Obb<T>, p: Vector3<T>) -> Vector3<T> {
    let d = p - b.center;
    Vector3::new(dot(d, b.axes[0]), dot(d, b.axes[1]), dot(d, b.axes[2]))
}

/// Squared distance from a point to an origin-centered axis-aligned box.
fn point_box_squared<T: RealScalar>(p: Vector3<T>, half: Vector3<T>) -> T {
    squared(Vector3::new(
        p.x - clamp(p.x, -half.x, half.x),
        p.y - clamp(p.y, -half.y, half.y),
        p.z - clamp(p.z, -half.z, half.z),
    ))
}

/// Squared distance to a closed segment, including a zero-length segment.
fn point_segment_squared<T: RealScalar>(p: Vector3<T>, a: Vector3<T>, b: Vector3<T>) -> T {
    let d = b - a;
    let length = squared(d);
    let t = if length > T::zero() {
        clamp(dot(p - a, d) / length, T::zero(), T::one())
    } else {
        T::zero()
    };
    squared(p - (a + d * t))
}

/// Squared distance between closed segments. Endpoint clamping recomputes the
/// other parameter, so parallel, point and endpoint-to-interior cases work.
fn segment_segment_squared<T: RealScalar>(
    p: Vector3<T>,
    q: Vector3<T>,
    a: Vector3<T>,
    b: Vector3<T>,
) -> T {
    let d = q - p;
    let e = b - a;
    let r = p - a;
    let dd = squared(d);
    let ee = squared(e);
    if dd == T::zero() {
        return point_segment_squared(p, a, b);
    }
    if ee == T::zero() {
        return point_segment_squared(a, p, q);
    }
    let de = dot(d, e);
    let dr = dot(d, r);
    let er = dot(e, r);
    let denominator = dd * ee - de * de;
    let mut s = if denominator > T::zero() {
        clamp((de * er - dr * ee) / denominator, T::zero(), T::one())
    } else {
        T::zero()
    };
    let mut t = (de * s + er) / ee;
    if t < T::zero() {
        t = T::zero();
        s = clamp(-dr / dd, T::zero(), T::one());
    } else if t > T::one() {
        t = T::one();
        s = clamp((de - dr) / dd, T::zero(), T::one());
    }
    squared((p + d * s) - (a + e * t))
}

/// Scales the represented face normal by its largest component. A nonzero
/// component becomes exactly +/-1, so subsequent squared-normal division cannot
/// lose its denominator to tiny-area quantization. No square root is required.
fn scaled_triangle_normal<T: RealScalar>(triangle: &Tri3<T>) -> Option<Vector3<T>> {
    let [a, b, c] = *triangle.vertices();
    let normal = Vector3::cross(&(b - a), &(c - a));
    let scale = T::max(normal.x.tabs(), T::max(normal.y.tabs(), normal.z.tabs()));
    if scale == T::zero() {
        None
    } else {
        Some(normal / scale)
    }
}

/// Squared distance to a triangle using a scaled-plane projection and its edges.
/// Unlike products of Gram terms, the face denominator stays at least one even
/// for small represented faces. A collapsed normal falls back to edge distances.
fn point_triangle_squared<T: RealScalar>(p: Vector3<T>, triangle: &Tri3<T>) -> T {
    point_triangle_with_normal(p, triangle, scaled_triangle_normal(triangle))
}

/// Reuses a prepared face for both capsule endpoints. An interior orthogonal
/// projection is closest on the face; otherwise the closest point is on one
/// of its three closed edges. Rounding can still alter boundary classifications.
fn point_triangle_with_normal<T: RealScalar>(
    p: Vector3<T>,
    triangle: &Tri3<T>,
    normal: Option<Vector3<T>>,
) -> T {
    let [a, b, c] = *triangle.vertices();
    if let Some(normal) = normal {
        let height = dot(p - a, normal) / squared(normal);
        let projected = p - normal * height;
        if inside_triangle(projected, triangle, normal) {
            return squared(p - projected);
        }
    }
    T::min(
        point_segment_squared(p, a, b),
        T::min(
            point_segment_squared(p, b, c),
            point_segment_squared(p, c, a),
        ),
    )
}

/// Tests the three oriented edge half-planes for a point on the triangle plane.
fn inside_triangle<T: RealScalar>(p: Vector3<T>, tri: &Tri3<T>, normal: Vector3<T>) -> bool {
    let [a, b, c] = *tri.vertices();
    [(a, b), (b, c), (c, a)]
        .iter()
        .all(|&(v, w)| dot(Vector3::cross(&(w - v), &(p - v)), normal) >= T::zero())
}

/// Squared segment-triangle distance. A face piercing must be tested explicitly:
/// endpoint-to-face and edge-to-edge distances alone miss interior crossings.
fn segment_triangle_squared<T: RealScalar>(a: Vector3<T>, b: Vector3<T>, tri: &Tri3<T>) -> T {
    let [u, v, w] = *tri.vertices();
    let normal = scaled_triangle_normal(tri);
    if let Some(normal) = normal {
        let da = dot(normal, a - u);
        let db = dot(normal, b - u);
        if ((da <= T::zero() && db >= T::zero()) || (db <= T::zero() && da >= T::zero()))
            && da != db
        {
            let t = clamp(da / (da - db), T::zero(), T::one());
            if inside_triangle(a + (b - a) * t, tri, normal) {
                return T::zero();
            }
        }
    }
    let mut result = T::min(
        point_triangle_with_normal(a, tri, normal),
        point_triangle_with_normal(b, tri, normal),
    );
    for (p, q) in [(u, v), (v, w), (w, u)] {
        result = T::min(result, segment_segment_squared(a, b, p, q));
    }
    result
}

/// Squared segment-AABB distance. Slab crossings split [0,1] into at most seven
/// intervals; within each, squared distance is quadratic. Evaluate its clamped
/// minimum, rather than treating an expanded box as a rounded capsule target.
fn segment_box_squared<T: RealScalar>(a: Vector3<T>, b: Vector3<T>, half: Vector3<T>) -> T {
    let d = b - a;
    let origin = [a.x, a.y, a.z];
    let direction = [d.x, d.y, d.z];
    let extent = [half.x, half.y, half.z];
    let mut breaks = [T::zero(); 8];
    breaks[1] = T::one();
    let mut count = 2;
    for axis in 0..3 {
        if direction[axis] != T::zero() {
            for bound in [-extent[axis], extent[axis]] {
                // Reject crossings outside the segment before division. Besides
                // saving work, this avoids huge parameters for tiny directions.
                let end = origin[axis] + direction[axis];
                if bound > T::min(origin[axis], end) && bound < T::max(origin[axis], end) {
                    let t = (bound - origin[axis]) / direction[axis];
                    let mut i = count;
                    while i > 0 && breaks[i - 1] > t {
                        breaks[i] = breaks[i - 1];
                        i -= 1;
                    }
                    breaks[i] = t;
                    count += 1;
                }
            }
        }
    }
    let mut result = T::min(point_box_squared(a, half), point_box_squared(b, half));
    for interval in breaks[..count].windows(2) {
        let lo = interval[0];
        let hi = interval[1];
        let mid = lo + (hi - lo) * T::half();
        let mut slope = [T::zero(); 3];
        let mut offset = [T::zero(); 3];
        for axis in 0..3 {
            let p = origin[axis] + direction[axis] * mid;
            if p < -extent[axis] || p > extent[axis] {
                slope[axis] = direction[axis];
                offset[axis] = origin[axis] - clamp(p, -extent[axis], extent[axis]);
            }
        }
        let slope = Vector3::new(slope[0], slope[1], slope[2]);
        let offset = Vector3::new(offset[0], offset[1], offset[2]);
        let quadratic = squared(slope);
        let t = if quadratic > T::zero() {
            clamp(-dot(slope, offset) / quadratic, lo, hi)
        } else {
            lo
        };
        result = T::min(result, point_box_squared(a + d * t, half));
    }
    result
}

/// Projects a box onto an arbitrary axis without normalizing it.
fn box_radius<T: RealScalar>(b: &Obb<T>, axis: Vector3<T>) -> T {
    b.half.x * dot(b.axes[0], axis).tabs()
        + b.half.y * dot(b.axes[1], axis).tabs()
        + b.half.z * dot(b.axes[2], axis).tabs()
}

/// Closed interval projection of a triangle onto an arbitrary SAT axis.
fn triangle_interval<T: RealScalar>(tri: &Tri3<T>, axis: Vector3<T>) -> (T, T) {
    let [a, b, c] = *tri.vertices();
    let pa = dot(a, axis);
    let pb = dot(b, axis);
    let pc = dot(c, axis);
    (T::min(pa, T::min(pb, pc)), T::max(pa, T::max(pb, pc)))
}

/// Triangle edge directions in winding order.
fn edges<T: RealScalar>(tri: &Tri3<T>) -> [Vector3<T>; 3] {
    let [a, b, c] = *tri.vertices();
    [b - a, c - b, a - c]
}

/// Sphere-sphere overlap from squared center distance.
pub fn sphere_sphere<T: RealScalar>(a: &Sphere3<T>, b: &Sphere3<T>) -> bool {
    let radius = a.radius + b.radius;
    squared(a.center - b.center) <= radius * radius
}

/// Sphere-capsule overlap from point-to-centerline distance.
pub fn sphere_capsule<T: RealScalar>(a: &Sphere3<T>, b: &Capsule<T>) -> bool {
    let radius = a.radius + b.radius;
    point_segment_squared(a.center, b.a, b.b) <= radius * radius
}

/// Sphere-box overlap from a clamped box-local point.
pub fn sphere_box<T: RealScalar>(a: &Sphere3<T>, b: &Obb<T>) -> bool {
    point_box_squared(local(b, a.center), b.half) <= a.radius * a.radius
}

/// Sphere-triangle overlap, including vertex and edge contacts.
pub fn sphere_triangle<T: RealScalar>(a: &Sphere3<T>, b: &Tri3<T>) -> bool {
    point_triangle_squared(a.center, b) <= a.radius * a.radius
}

/// Capsule-capsule overlap from the distance between closed centerlines.
pub fn capsule_capsule<T: RealScalar>(a: &Capsule<T>, b: &Capsule<T>) -> bool {
    let radius = a.radius + b.radius;
    segment_segment_squared(a.a, a.b, b.a, b.b) <= radius * radius
}

/// Capsule-box overlap with rounded edges/corners, not just expanded slabs.
pub fn capsule_box<T: RealScalar>(a: &Capsule<T>, b: &Obb<T>) -> bool {
    segment_box_squared(local(b, a.a), local(b, a.b), b.half) <= a.radius * a.radius
}

/// Capsule-triangle overlap, including centerlines piercing the triangle face.
pub fn capsule_triangle<T: RealScalar>(a: &Capsule<T>, b: &Tri3<T>) -> bool {
    segment_triangle_squared(a.a, a.b, b) <= a.radius * a.radius
}

/// Box-box separating-axis test: six face axes and nine edge cross products.
/// Zero cross products project both shapes to zero and cannot separate them.
pub fn box_box<T: RealScalar>(a: &Obb<T>, b: &Obb<T>) -> bool {
    let delta = b.center - a.center;
    let separated = |axis| dot(delta, axis).tabs() > box_radius(a, axis) + box_radius(b, axis);
    for axis in a.axes.iter().chain(b.axes.iter()) {
        if separated(*axis) {
            return false;
        }
    }
    for aa in a.axes {
        for bb in b.axes {
            if separated(Vector3::cross(&aa, &bb)) {
                return false;
            }
        }
    }
    true
}

/// Box-triangle SAT with three box normals, one triangle normal and nine
/// edge-cross axes. Triangle projections are centered to reduce cancellation.
pub fn box_triangle<T: RealScalar>(a: &Obb<T>, b: &Tri3<T>) -> bool {
    let vertices = b.vertices();
    let centered = Tri3::new([
        vertices[0] - a.center,
        vertices[1] - a.center,
        vertices[2] - a.center,
    ]);
    let edge = edges(&centered);
    let separated = |axis| {
        let (lo, hi) = triangle_interval(&centered, axis);
        let radius = box_radius(a, axis);
        lo > radius || hi < -radius
    };
    if separated(Vector3::cross(&edge[0], &edge[1])) {
        return false;
    }
    for axis in a.axes {
        if separated(axis) {
            return false;
        }
        for e in edge {
            if separated(Vector3::cross(&axis, &e)) {
                return false;
            }
        }
    }
    true
}

/// Triangle-triangle SAT. Face normals and edge crosses cover spatial pairs;
/// in-plane edge normals additionally separate disjoint coplanar triangles.
/// Those extra axes are valid separating directions for spatial pairs as well.
pub fn triangle_triangle<T: RealScalar>(a: &Tri3<T>, b: &Tri3<T>) -> bool {
    let ea = edges(a);
    let eb = edges(b);
    let na = Vector3::cross(&ea[0], &ea[1]);
    let nb = Vector3::cross(&eb[0], &eb[1]);
    let separated = |axis| {
        let (al, ah) = triangle_interval(a, axis);
        let (bl, bh) = triangle_interval(b, axis);
        al > bh || bl > ah
    };
    if separated(na) || separated(nb) {
        return false;
    }
    for edge in ea {
        if separated(Vector3::cross(&na, &edge)) {
            return false;
        }
        for other in eb {
            if separated(Vector3::cross(&edge, &other)) {
                return false;
            }
        }
    }
    for edge in eb {
        if separated(Vector3::cross(&nb, &edge)) {
            return false;
        }
    }
    true
}

#[cfg(test)]
mod tests;
