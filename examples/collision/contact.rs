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

//! Example contact generation. Normals point from B toward A; depth is the
//! positive displacement along that normal needed to separate A. Closed contact
//! is included. SAT manifolds have at most four spatially spread points. Inputs
//! and intermediate arithmetic must fit, as for the existing overlap kernels.

use super::*;
use rs_math3d::Box3;

/// A represented world-space convex primitive, borrowed by contact dispatch.
#[derive(Clone, Copy)]
pub enum Convex<T: RealScalar> {
    /// Sphere with a nonnegative radius.
    Sphere(Sphere3<T>),
    /// Swept segment with a nonnegative radius.
    Capsule(Capsule<T>),
    /// Oriented box with orthonormal axes and nonnegative half extents.
    Box(Obb<T>),
}
impl<T: RealScalar> Convex<T> {
    /// Conservative bounds in the represented arithmetic, including directed shifts.
    pub fn bounds(&self) -> Box3<T> {
        match self {
            Self::Sphere(s) => {
                let r = Vector3::new(s.radius, s.radius, s.radius);
                Box3::new(&(s.center - r), &(s.center + r))
            }
            Self::Capsule(c) => super::aabb_tree::capsule_bounds(c),
            Self::Box(b) => super::aabb_tree::obb_bounds(b),
        }
    }
}

/// One world contact, suitable for a normal impulse and a two-axis friction cone.
#[derive(Clone, Copy)]
pub struct Contact<T: RealScalar> {
    /// Shared world-space impulse location.
    pub point: Vector3<T>,
    /// Unit separating normal, B toward A.
    pub normal: Vector3<T>,
    /// Nonnegative represented interpenetration along normal.
    pub depth: T,
}
/// Fixed-capacity manifold; construction and traversal allocate no heap memory.
#[derive(Clone, Copy)]
pub struct Manifold<T: RealScalar> {
    /// Initialized contact storage; only the prefix `count` is observable.
    points: [Contact<T>; 4],
    /// Number of valid points.
    count: usize,
}
impl<T: RealScalar> Manifold<T> {
    /// Empty manifold, including represented misses.
    pub fn empty() -> Self {
        let z = T::zero();
        Self {
            points: [Contact {
                point: Vector3::zero(),
                normal: Vector3::new(z, T::one(), z),
                depth: z,
            }; 4],
            count: 0,
        }
    }
    /// Valid contact prefix.
    pub fn points(&self) -> &[Contact<T>] {
        &self.points[..self.count]
    }
    /// Adds a point when capacity permits. SAT reduction selects spread points first.
    fn push(&mut self, c: Contact<T>) {
        if self.points().iter().any(|old| {
            old.point.x == c.point.x
                && old.point.y == c.point.y
                && old.point.z == c.point.z
                && old.normal.x == c.normal.x
                && old.normal.y == c.normal.y
                && old.normal.z == c.normal.z
        }) {
            return;
        }
        if self.count < 4 {
            self.points[self.count] = c;
            self.count += 1;
        }
    }
    /// Swaps A/B ownership without moving the shared world point.
    fn flipped(mut self) -> Self {
        for p in &mut self.points[..self.count] {
            p.normal = -p.normal;
        }
        self
    }
}

/// Normalizes via the largest component so fixed small axes retain a denominator.
/// A represented zero direction has no unique normal and returns None.
/// The fused squared-length quantization is part of this collision experiment's
/// contact policy and can differ from the core norm-based normalizers.
pub fn unit<T: RealScalar>(v: Vector3<T>) -> Option<Vector3<T>> {
    let scale = v.max_abs_component();
    if scale == T::zero() {
        None
    } else {
        let v = v / scale;
        Some(v / v.length_squared().tsqrt())
    }
}
/// Deterministic fallback for coincident primitive centers.
fn up<T: RealScalar>() -> Vector3<T> {
    Vector3::new(T::zero(), T::one(), T::zero())
}
/// Zero-distance swept contacts escape perpendicular to both centerlines. A
/// parallel/coincident pair uses a deterministic perpendicular to its live line;
/// choosing an axis along the segment would underestimate containment depth.
fn swept_normal<T: RealScalar>(a: Vector3<T>, b: Vector3<T>) -> Vector3<T> {
    if let Some(n) = unit(Vector3::cross(&a, &b)) {
        return n;
    }
    let Some(line) = unit(a).or_else(|| unit(b)) else {
        return up();
    };
    let helper = if line.x.tabs() < line.y.tabs() {
        Vector3::new(T::one(), T::zero(), T::zero())
    } else {
        Vector3::new(T::zero(), T::one(), T::zero())
    };
    unit(Vector3::cross(&line, &helper)).unwrap_or(up())
}
/// World coordinate from box-local components.
fn world<T: RealScalar>(b: &Obb<T>, p: Vector3<T>) -> Vector3<T> {
    b.center + b.axes[0] * p.x + b.axes[1] * p.y + b.axes[2] * p.z
}
/// Closest point on a closed segment, including collapsed centerlines.
fn segment_point<T: RealScalar>(p: Vector3<T>, a: Vector3<T>, b: Vector3<T>) -> Vector3<T> {
    let d = b - a;
    let length = d.length_squared();
    a + d * if length > T::zero() {
        (Vector3::dot(&(p - a), &d) / length).clamp(T::zero(), T::one())
    } else {
        T::zero()
    }
}
/// Closest points between segments; retains the endpoint re-clamping used by
/// the existing distance predicate so parallel/end contacts are represented.
fn segments<T: RealScalar>(
    p: Vector3<T>,
    q: Vector3<T>,
    a: Vector3<T>,
    b: Vector3<T>,
) -> (Vector3<T>, Vector3<T>) {
    let d = q - p;
    let e = b - a;
    let r = p - a;
    let dd = d.length_squared();
    let ee = e.length_squared();
    if dd == T::zero() {
        return (p, segment_point(p, a, b));
    }
    if ee == T::zero() {
        return (segment_point(a, p, q), a);
    }
    let de = Vector3::dot(&d, &e);
    let dr = Vector3::dot(&d, &r);
    let er = Vector3::dot(&e, &r);
    let denom = dd * ee - de * de;
    let mut s = if denom > T::zero() {
        ((de * er - dr * ee) / denom).clamp(T::zero(), T::one())
    } else {
        T::zero()
    };
    let mut t = (de * s + er) / ee;
    if t < T::zero() {
        t = T::zero();
        s = (-dr / dd).clamp(T::zero(), T::one());
    } else if t > T::one() {
        t = T::one();
        s = ((de - dr) / dd).clamp(T::zero(), T::one());
    }
    (p + d * s, a + e * t)
}
/// Triangle closest point: scaled interior projection or one of the closed edges.
fn triangle_point<T: RealScalar>(p: Vector3<T>, tri: &Tri3<T>) -> Vector3<T> {
    let [a, b, c] = *tri.vertices();
    if let Some(normal) = scaled_triangle_normal(tri) {
        let projected = p - normal * (Vector3::dot(&(p - a), &normal) / normal.length_squared());
        if inside_triangle(projected, tri, normal) {
            return projected;
        }
    }
    let mut best = segment_point(p, a, b);
    for q in [segment_point(p, b, c), segment_point(p, c, a)] {
        if (p - q).length_squared() < (p - best).length_squared() {
            best = q;
        }
    }
    best
}
/// Segment/triangle witnesses, including a centerline piercing the interior face.
fn segment_triangle<T: RealScalar>(
    a: Vector3<T>,
    b: Vector3<T>,
    tri: &Tri3<T>,
) -> (Vector3<T>, Vector3<T>) {
    let [u, v, w] = *tri.vertices();
    if let Some(normal) = scaled_triangle_normal(tri) {
        let da = Vector3::dot(&normal, &(a - u));
        let db = Vector3::dot(&normal, &(b - u));
        if ((da <= T::zero() && db >= T::zero()) || (db <= T::zero() && da >= T::zero()))
            && da != db
        {
            let p = a + (b - a) * (da / (da - db)).clamp(T::zero(), T::one());
            if inside_triangle(p, tri, normal) {
                return (p, p);
            }
        }
    }
    let mut best = (a, triangle_point(a, tri));
    for candidate in [
        (b, triangle_point(b, tri)),
        segments(a, b, u, v),
        segments(a, b, v, w),
        segments(a, b, w, u),
    ] {
        if (candidate.0 - candidate.1).length_squared() < (best.0 - best.1).length_squared() {
            best = candidate;
        }
    }
    best
}
/// Spherical surface witnesses. Fallback normals resolve coincident centerlines.
fn rounded<T: RealScalar>(
    a: Vector3<T>,
    ra: T,
    b: Vector3<T>,
    rb: T,
    fallback: Vector3<T>,
) -> Manifold<T> {
    let mut result = Manifold::empty();
    let d = a - b;
    let distance = d.length_squared().tsqrt();
    let radius = ra + rb;
    if distance > radius {
        return result;
    }
    let normal = unit(d).unwrap_or(fallback);
    result.push(Contact {
        point: ((a - normal * ra) + (b + normal * rb)) * T::half(),
        normal,
        depth: T::max(radius - distance, T::zero()),
    });
    result
}
/// Sphere/box closest feature, including a center inside the box volume.
fn sphere_box_contact<T: RealScalar>(s: &Sphere3<T>, b: &Obb<T>) -> Manifold<T> {
    let p = local(b, s.center);
    let q = Vector3::new(
        p.x.clamp(-b.half.x, b.half.x),
        p.y.clamp(-b.half.y, b.half.y),
        p.z.clamp(-b.half.z, b.half.z),
    );
    if (p - q).length_squared() > T::zero() {
        return rounded(s.center, s.radius, world(b, q), T::zero(), up());
    }
    let mut axis = 0;
    let mut distance = b.half.x - p.x.tabs();
    for i in 1..3 {
        let d = b.half.to_array()[i] - p.to_array()[i].tabs();
        if d < distance {
            axis = i;
            distance = d;
        }
    }
    let normal = b.axes[axis]
        * if p.to_array()[axis] >= T::zero() {
            T::one()
        } else {
            -T::one()
        };
    let mut result = Manifold::empty();
    result.push(Contact {
        point: s.center + normal * distance,
        normal,
        depth: s.radius + distance,
    });
    result
}
/// Segment/AABB minimizer over the same slab-break intervals as the boolean
/// capsule/box predicate. Returns local centerline and closest box points.
fn segment_box<T: RealScalar>(
    a: Vector3<T>,
    b: Vector3<T>,
    half: Vector3<T>,
) -> (Vector3<T>, Vector3<T>) {
    let d = b - a;
    let mut breaks = [T::zero(); 8];
    breaks[1] = T::one();
    let mut count = 2;
    for i in 0..3 {
        let direction = d.to_array()[i];
        if direction != T::zero() {
            for bound in [-half.to_array()[i], half.to_array()[i]] {
                if bound > T::min(a.to_array()[i], b.to_array()[i])
                    && bound < T::max(a.to_array()[i], b.to_array()[i])
                {
                    let t = (bound - a.to_array()[i]) / direction;
                    let mut j = count;
                    while j > 0 && breaks[j - 1] > t {
                        breaks[j] = breaks[j - 1];
                        j -= 1;
                    }
                    breaks[j] = t;
                    count += 1;
                }
            }
        }
    }
    let closest = |p: Vector3<T>| {
        Vector3::new(
            p.x.clamp(-half.x, half.x),
            p.y.clamp(-half.y, half.y),
            p.z.clamp(-half.z, half.z),
        )
    };
    let mut best = (a, closest(a));
    for p in [b] {
        let q = closest(p);
        if (p - q).length_squared() < (best.0 - best.1).length_squared() {
            best = (p, q);
        }
    }
    for interval in breaks[..count].windows(2) {
        let lo = interval[0];
        let hi = interval[1];
        let mid = lo + (hi - lo) * T::half();
        let mut slope = [T::zero(); 3];
        let mut offset = slope;
        for i in 0..3 {
            let p = a.to_array()[i] + d.to_array()[i] * mid;
            let h = half.to_array()[i];
            if p < -h || p > h {
                slope[i] = d.to_array()[i];
                offset[i] = a.to_array()[i] - p.clamp(-h, h);
            }
        }
        let slope = Vector3::new(slope[0], slope[1], slope[2]);
        let offset = Vector3::new(offset[0], offset[1], offset[2]);
        let length = slope.length_squared();
        let t = if length > T::zero() {
            (-Vector3::dot(&slope, &offset) / length).clamp(lo, hi)
        } else {
            lo
        };
        let p = a + d * t;
        let q = closest(p);
        if (p - q).length_squared() < (best.0 - best.1).length_squared() {
            best = (p, q);
        }
    }
    best
}

/// SAT minimum translation; intervals account for containment, not merely the
/// intersection's length (which is zero along a triangle's face normal).
struct Separation<T: RealScalar> {
    /// Winning world-space unit axis pointing B to A.
    normal: Vector3<T>,
    /// Smallest translation distance seen so far.
    depth: Option<T>,
}
impl<T: RealScalar> Separation<T> {
    /// No usable axis has been considered.
    fn new() -> Self {
        Self {
            normal: up(),
            depth: None,
        }
    }
    /// Returns false on a separating interval; zero directions cannot separate.
    fn axis(
        &mut self,
        axis: Vector3<T>,
        a: impl Fn(Vector3<T>) -> (T, T),
        b: impl Fn(Vector3<T>) -> (T, T),
    ) -> bool {
        let Some(axis) = unit(axis) else {
            return true;
        };
        let (al, ah) = a(axis);
        let (bl, bh) = b(axis);
        if ah < bl || bh < al {
            return false;
        }
        let (negative, positive) = (ah - bl, bh - al);
        let (normal, depth) = if negative < positive {
            (-axis, negative)
        } else {
            (axis, positive)
        };
        if self.depth.is_none_or(|old| depth < old) {
            self.normal = normal;
            self.depth = Some(depth);
        }
        true
    }
}
/// Box interval on an arbitrary unit axis.
fn box_interval<T: RealScalar>(b: &Obb<T>, n: Vector3<T>) -> (T, T) {
    let c = Vector3::dot(&b.center, &n);
    let r = box_radius(b, n);
    (c - r, c + r)
}
/// Eight represented corners, evaluating both signed half-axis ends explicitly.
fn corners<T: RealScalar>(b: &Obb<T>) -> [Vector3<T>; 8] {
    core::array::from_fn(|i| {
        b.center
            + b.axes[0] * (b.half.x * if i & 1 == 0 { -T::one() } else { T::one() })
            + b.axes[1] * (b.half.y * if i & 2 == 0 { -T::one() } else { T::one() })
            + b.axes[2] * (b.half.z * if i & 4 == 0 { -T::one() } else { T::one() })
    })
}
/// Tests a world point against a box with a small represented boundary tolerance.
fn in_box<T: RealScalar>(b: &Obb<T>, p: Vector3<T>, epsilon: T) -> bool {
    let p = local(b, p);
    p.x.tabs() <= b.half.x + epsilon
        && p.y.tabs() <= b.half.y + epsilon
        && p.z.tabs() <= b.half.z + epsilon
}
/// Reduces a bounded polygon to four greedily spread contact points on the
/// common contact plane, deduplicating represented edge/triangle seam points.
fn polygon_manifold<T: RealScalar>(
    points: &[Vector3<T>],
    normal: Vector3<T>,
    depth: T,
    plane: T,
) -> Manifold<T> {
    let mut result = Manifold::empty();
    let mut used = [false; 32];
    for _ in 0..4 {
        let mut chosen = None;
        let mut score = T::zero();
        for (i, &p) in points.iter().enumerate() {
            if used[i] {
                continue;
            }
            let p = p + normal * (plane - Vector3::dot(&p, &normal));
            let distance = result
                .points()
                .iter()
                .map(|c| (p - c.point).length_squared())
                .reduce(T::min)
                .unwrap_or(T::one());
            if chosen.is_none() || distance > score {
                chosen = Some((i, p));
                score = distance;
            }
        }
        let Some((i, p)) = chosen else {
            break;
        };
        if result.count > 0 && score <= T::zero() {
            break;
        }
        used[i] = true;
        result.push(Contact {
            point: p,
            normal,
            depth,
        });
    }
    result
}
/// Clips a segment to a box, returning the entering/exiting world points.
fn clip_segment<T: RealScalar>(
    a: Vector3<T>,
    b: Vector3<T>,
    box_: &Obb<T>,
) -> Option<(Vector3<T>, Vector3<T>)> {
    let p = local(box_, a);
    let q = local(box_, b);
    let d = q - p;
    let (mut lo, mut hi) = (T::zero(), T::one());
    for i in 0..3 {
        let o = p.to_array()[i];
        let v = d.to_array()[i];
        let h = box_.half.to_array()[i];
        if v == T::zero() {
            if o < -h || o > h {
                return None;
            }
        } else {
            let x = (-h - o) / v;
            let y = (h - o) / v;
            lo = T::max(lo, T::min(x, y));
            hi = T::min(hi, T::max(x, y));
            if lo > hi {
                return None;
            }
        }
    }
    Some((a + (b - a) * lo, a + (b - a) * hi))
}
/// Box/box SAT plus vertices of the intersection polytope. The reduced face
/// manifold supports stacks; crossing-edge cases have a deterministic fallback.
fn boxes<T: RealScalar>(a: &Obb<T>, b: &Obb<T>) -> Manifold<T> {
    let mut sat = Separation::new();
    for axis in a.axes.iter().copied().chain(b.axes.iter().copied()) {
        if !sat.axis(axis, |n| box_interval(a, n), |n| box_interval(b, n)) {
            return Manifold::empty();
        }
    }
    for aa in a.axes {
        for bb in b.axes {
            if !sat.axis(
                Vector3::cross(&aa, &bb),
                |n| box_interval(a, n),
                |n| box_interval(b, n),
            ) {
                return Manifold::empty();
            }
        }
    }
    let Some(depth) = sat.depth else {
        return Manifold::empty();
    };
    let normal = sat.normal;
    let plane = (box_interval(a, normal).0 + box_interval(b, normal).1) * T::half();
    let ac = corners(a);
    let bc = corners(b);
    let mut points = [a.center; 32];
    let mut count = 0;
    for (vertices, other) in [(&ac, b), (&bc, a)] {
        for &p in vertices {
            if count < 32 && in_box(other, p, T::zero()) {
                points[count] = p;
                count += 1;
            }
        }
        for i in 0..8 {
            for bit in [1, 2, 4] {
                if i & bit == 0 {
                    if let Some((p, q)) = clip_segment(vertices[i], vertices[i | bit], other) {
                        for point in [p, q] {
                            if count < 32 {
                                points[count] = point;
                                count += 1;
                            }
                        }
                    }
                }
            }
        }
    }
    if count == 0 {
        points[0] = (a.center + b.center) * T::half();
        count = 1;
    }
    polygon_manifold(&points[..count], normal, depth, plane)
}
/// Capsule/box contact, with a face escape direction when the centerline is inside.
fn capsule_box_contact<T: RealScalar>(a: &Capsule<T>, b: &Obb<T>) -> Manifold<T> {
    let (p, q) = segment_box(local(b, a.a), local(b, a.b), b.half);
    if (p - q).length_squared() > T::zero() {
        return rounded(world(b, p), a.radius, world(b, q), T::zero(), up());
    }
    let mut sat = Separation::new();
    for axis in b.axes {
        if !sat.axis(
            axis,
            |n| {
                let x = Vector3::dot(&a.a, &n);
                let y = Vector3::dot(&a.b, &n);
                (T::min(x, y) - a.radius, T::max(x, y) + a.radius)
            },
            |n| box_interval(b, n),
        ) {
            return Manifold::empty();
        }
    }
    let mut result = Manifold::empty();
    if let Some(depth) = sat.depth {
        result.push(Contact {
            point: world(b, p),
            normal: sat.normal,
            depth,
        });
    }
    result
}

/// Generates all six unordered convex pair families. Existing collision kernels
/// remain the overlap gate; contact construction adds surface witnesses/depth.
pub fn convex<T: RealScalar>(a: &Convex<T>, b: &Convex<T>) -> Manifold<T> {
    match (a, b) {
        (Convex::Sphere(a), Convex::Sphere(b)) => {
            if sphere_sphere(a, b) {
                rounded(a.center, a.radius, b.center, b.radius, up())
            } else {
                Manifold::empty()
            }
        }
        (Convex::Sphere(a), Convex::Capsule(b)) => {
            if sphere_capsule(a, b) {
                rounded(
                    a.center,
                    a.radius,
                    segment_point(a.center, b.a, b.b),
                    b.radius,
                    swept_normal(b.b - b.a, b.b - b.a),
                )
            } else {
                Manifold::empty()
            }
        }
        (Convex::Capsule(_), Convex::Sphere(_)) => convex(b, a).flipped(),
        (Convex::Sphere(a), Convex::Box(b)) => {
            if sphere_box(a, b) {
                sphere_box_contact(a, b)
            } else {
                Manifold::empty()
            }
        }
        (Convex::Box(_), Convex::Sphere(_)) => convex(b, a).flipped(),
        (Convex::Capsule(a), Convex::Capsule(b)) => {
            if capsule_capsule(a, b) {
                let (p, q) = segments(a.a, a.b, b.a, b.b);
                rounded(p, a.radius, q, b.radius, swept_normal(a.b - a.a, b.b - b.a))
            } else {
                Manifold::empty()
            }
        }
        (Convex::Capsule(a), Convex::Box(b)) => {
            if capsule_box(a, b) {
                let mut result = capsule_box_contact(a, b);
                for endpoint in [a.a, a.b] {
                    let sphere = Sphere3::new(endpoint, a.radius);
                    if sphere_box(&sphere, b) {
                        for &c in sphere_box_contact(&sphere, b).points() {
                            if result
                                .points()
                                .first()
                                .is_some_and(|p| Vector3::dot(&p.normal, &c.normal) > T::half())
                            {
                                result.push(c);
                            }
                        }
                    }
                }
                result
            } else {
                Manifold::empty()
            }
        }
        (Convex::Box(_), Convex::Capsule(_)) => convex(b, a).flipped(),
        (Convex::Box(a), Convex::Box(b)) => {
            if box_box(a, b) {
                boxes(a, b)
            } else {
                Manifold::empty()
            }
        }
    }
}
/// Rounded shape/triangle witness, using a face normal when distance collapses.
fn capsule_triangle_contact<T: RealScalar>(a: &Capsule<T>, tri: &Tri3<T>) -> Manifold<T> {
    let (p, q) = segment_triangle(a.a, a.b, tri);
    if (p - q).length_squared() > T::zero() {
        let mut result = rounded(p, a.radius, q, T::zero(), up());
        for endpoint in [a.a, a.b] {
            let closest = triangle_point(endpoint, tri);
            let extra = rounded(endpoint, a.radius, closest, T::zero(), up());
            for &c in extra.points() {
                if result
                    .points()
                    .first()
                    .is_some_and(|first| Vector3::dot(&first.normal, &c.normal) > T::half())
                {
                    result.push(c);
                }
            }
        }
        return result;
    }
    let mut normal = unit(scaled_triangle_normal(tri).unwrap_or(up())).unwrap_or(up());
    let origin = tri.vertices()[0];
    if Vector3::dot(&((a.a + a.b) * T::half() - origin), &normal) < T::zero() {
        normal = -normal;
    }
    let depth = a.radius
        - T::min(
            Vector3::dot(&(a.a - origin), &normal),
            Vector3::dot(&(a.b - origin), &normal),
        );
    let mut result = Manifold::empty();
    result.push(Contact {
        point: q,
        normal,
        depth: T::max(depth, T::zero()),
    });
    result
}
/// Clips the triangle polygon against all six box half-spaces. The result is a
/// finite intersection polygon; native quantization may merge nearby vertices.
fn box_triangle_contact<T: RealScalar>(b: &Obb<T>, tri: &Tri3<T>) -> Manifold<T> {
    let mut sat = Separation::new();
    let edge = edges(tri);
    for axis in core::iter::once(Vector3::cross(&edge[0], &edge[1]))
        .chain(b.axes.iter().copied())
        .chain(
            b.axes
                .iter()
                .copied()
                .flat_map(|a| edge.map(|e| Vector3::cross(&a, &e))),
        )
    {
        if !sat.axis(axis, |n| box_interval(b, n), |n| triangle_interval(tri, n)) {
            return Manifold::empty();
        }
    }
    let Some(depth) = sat.depth else {
        return Manifold::empty();
    };
    let normal = sat.normal;
    let mut polygon = [b.center; 16];
    polygon[..3].copy_from_slice(tri.vertices());
    let mut count = 3;
    for axis in 0..3 {
        for sign in [-T::one(), T::one()] {
            let mut out = [b.center; 16];
            let mut used = 0;
            if count == 0 {
                return Manifold::empty();
            }
            let mut previous = polygon[count - 1];
            let mut pd = Vector3::dot(&(previous - b.center), &b.axes[axis]) * sign
                - b.half.to_array()[axis];
            for &current in &polygon[..count] {
                let cd = Vector3::dot(&(current - b.center), &b.axes[axis]) * sign
                    - b.half.to_array()[axis];
                if (pd <= T::zero()) != (cd <= T::zero()) {
                    let p = previous
                        + (current - previous) * (pd / (pd - cd)).clamp(T::zero(), T::one());
                    if used < 16 {
                        out[used] = p;
                        used += 1;
                    }
                }
                if cd <= T::zero() && used < 16 {
                    out[used] = current;
                    used += 1;
                }
                previous = current;
                pd = cd;
            }
            polygon = out;
            count = used;
        }
    }
    let plane = (box_interval(b, normal).0 + triangle_interval(tri, normal).1) * T::half();
    polygon_manifold(&polygon[..count], normal, depth, plane)
}
/// Convex/triangle contacts for an immutable triangle soup. Triangles are two
/// sided surfaces; volume containment without touching a surface is not inferred.
pub fn triangle<T: RealScalar>(shape: &Convex<T>, tri: &Tri3<T>) -> Manifold<T> {
    match shape {
        Convex::Sphere(s) => {
            if sphere_triangle(s, tri) {
                capsule_triangle_contact(
                    &Capsule {
                        a: s.center,
                        b: s.center,
                        radius: s.radius,
                    },
                    tri,
                )
            } else {
                Manifold::empty()
            }
        }
        Convex::Capsule(c) => {
            if capsule_triangle(c, tri) {
                capsule_triangle_contact(c, tri)
            } else {
                Manifold::empty()
            }
        }
        Convex::Box(b) => {
            if box_triangle(b, tri) {
                box_triangle_contact(b, tri)
            } else {
                Manifold::empty()
            }
        }
    }
}
