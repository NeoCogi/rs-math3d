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

//! Analytic contacts and independent distance references for the example kernels.
use super::*;
use rs_math3d::{Fixed, TryCastFrom};

/// Fixture conversion is outside the kernels under test.
fn n<T: TryCastFrom<f64>>(value: f64) -> T {
    T::try_cast_from(value).ok().unwrap()
}

/// A small exactly representable point shared across all test backends.
fn v<T: RealScalar + TryCastFrom<f64>>(x: f64, y: f64, z: f64) -> Vector3<T> {
    Vector3::new(n(x), n(y), n(z))
}

/// Unit axis-aligned box at the origin.
fn unit_box<T: RealScalar + TryCastFrom<f64>>() -> Obb<T> {
    Obb {
        center: v(0.0, 0.0, 0.0),
        axes: [v(1.0, 0.0, 0.0), v(0.0, 1.0, 0.0), v(0.0, 0.0, 1.0)],
        half: v(1.0, 1.0, 1.0),
    }
}

/// Right triangle in the xy plane with two-unit legs.
fn triangle<T: RealScalar + TryCastFrom<f64>>() -> Tri3<T> {
    Tri3::new([v(0.0, 0.0, 0.0), v(2.0, 0.0, 0.0), v(0.0, 2.0, 0.0)])
}

/// Sphere fixture without additional construction policy.
fn sphere<T: RealScalar + TryCastFrom<f64>>(x: f64, y: f64, z: f64, r: f64) -> Sphere3<T> {
    Sphere3::new(v(x, y, z), n(r))
}

/// Checks separated, overlapping and touching configurations of every pair,
/// plus degenerate centerlines, parallel segments and all triangle point regions.
fn contacts<T: RealScalar + TryCastFrom<f64> + core::fmt::Debug>() {
    let ball = sphere::<T>(0.0, 0.0, 0.0, 0.5);
    for (x, hit) in [(0.0, true), (1.0, true), (1.25, false)] {
        let other = sphere(x, 0.0, 0.0, 0.5);
        assert_eq!(sphere_sphere(&ball, &other), hit);
        assert_eq!(sphere_sphere(&other, &ball), hit);
    }
    let capsule = Capsule::<T> {
        a: v(0.0, -1.0, 0.0),
        b: v(0.0, 1.0, 0.0),
        radius: n(0.5),
    };
    for (x, hit) in [(0.5, true), (1.0, true), (1.25, false)] {
        let other = sphere(x, 0.0, 0.0, 0.5);
        assert_eq!(sphere_capsule(&other, &capsule), hit);
        let parallel = Capsule {
            a: v(x, -0.5, 0.0),
            b: v(x, 0.5, 0.0),
            radius: n(0.5),
        };
        assert_eq!(capsule_capsule(&capsule, &parallel), hit);
        assert_eq!(capsule_capsule(&parallel, &capsule), hit);
    }
    let point = Capsule {
        a: ball.center,
        b: ball.center,
        radius: ball.radius,
    };
    assert!(capsule_capsule(&point, &point));
    assert!(sphere_capsule(&ball, &point));
    assert_eq!(
        segment_segment_squared(
            v::<T>(-1.0, 0.0, 0.0),
            v(1.0, 0.0, 0.0),
            v(0.0, -1.0, 0.5),
            v(0.0, 1.0, 0.5)
        ),
        n(0.25)
    );
    assert_eq!(
        segment_segment_squared(
            v::<T>(0.0, 0.0, 0.0),
            v(1.0, 0.0, 0.0),
            v(2.0, -1.0, 0.0),
            v(2.0, 1.0, 0.0)
        ),
        n(1.0)
    );

    let b = unit_box::<T>();
    for (x, hit) in [(0.0, true), (1.5, true), (1.75, false)] {
        let ball = sphere(x, 0.0, 0.0, 0.5);
        assert_eq!(sphere_box(&ball, &b), hit);
        let capsule = Capsule {
            a: v(x, -0.5, 0.0),
            b: v(x, 0.5, 0.0),
            radius: n(0.5),
        };
        assert_eq!(capsule_box(&capsule, &b), hit);
    }
    // The expanded-slab shortcut would incorrectly report a hit at this corner.
    let corner = Capsule {
        a: v(1.5, 1.5, -0.5),
        b: v(1.5, 1.5, 0.5),
        radius: n(0.5),
    };
    assert!(!capsule_box(&corner, &b));
    let corner_contact = Capsule {
        a: v(1.375, 1.5, -0.5),
        b: v(1.375, 1.5, 0.5),
        radius: n(0.625),
    };
    assert!(capsule_box(&corner_contact, &b));
    let crossing = Capsule {
        a: v(-2.0, 0.0, 0.0),
        b: v(2.0, 0.0, 0.0),
        radius: n(0.0),
    };
    assert!(capsule_box(&crossing, &b));
    assert!(capsule_box(&point, &b));
    let diagonal = Capsule {
        a: v(-2.0, 0.5, 2.0),
        b: v(2.0, 2.5, 0.0),
        radius: n(0.25),
    };
    assert!(!capsule_box(&diagonal, &b));

    let tri = triangle::<T>();
    let tilted = Tri3::new([v::<T>(0.0, 0.0, 0.0), v(1.0, 0.0, 1.0), v(0.0, 1.0, 1.0)]);
    for (point, expected) in [
        (v(0.0, 0.0, 0.75), 0.1875),
        (v(0.5, 0.5, 0.25), 0.1875),
        (v(0.75, 0.75, 1.5), 0.375),
        (v(-0.5, -0.5, -1.0), 1.5),
    ] {
        assert_eq!(point_triangle_squared(point, &tilted), n(expected));
        let vertices = tilted.vertices();
        assert_eq!(
            point_triangle_squared(point, &Tri3::new([vertices[2], vertices[1], vertices[0]])),
            n(expected)
        );
    }
    for (p, expected) in [
        ((-0.5, -0.5, 0.0), 0.5), // vertex a
        ((2.5, 0.0, 0.0), 0.25),  // vertex b
        ((0.0, 2.5, 0.0), 0.25),  // vertex c
        ((0.5, -0.5, 0.0), 0.25), // edge ab
        ((-0.5, 0.5, 0.0), 0.25), // edge ac
        ((1.5, 1.5, 0.0), 0.5),   // edge bc
        ((0.5, 0.5, 0.5), 0.25),  // face
    ] {
        assert_eq!(
            point_triangle_squared(v::<T>(p.0, p.1, p.2), &tri),
            n(expected)
        );
    }
    for (z, hit) in [(0.0, true), (0.5, true), (0.75, false)] {
        assert_eq!(sphere_triangle(&sphere(0.5, 0.5, z, 0.5), &tri), hit);
        let cap = Capsule {
            a: v(0.25, 0.5, z),
            b: v(0.75, 0.5, z),
            radius: n(0.5),
        };
        assert_eq!(capsule_triangle(&cap, &tri), hit);
    }
    let piercing = Capsule {
        a: v(0.5, 0.5, -1.0),
        b: v(0.5, 0.5, 1.0),
        radius: n(0.0),
    };
    assert!(capsule_triangle(&piercing, &tri));
    let coplanar = Capsule {
        a: v(-1.0, 0.5, 0.0),
        b: v(2.0, 0.5, 0.0),
        radius: n(0.0),
    };
    assert!(capsule_triangle(&coplanar, &tri));
    assert!(capsule_triangle(
        &Capsule {
            a: v(0.5, 0.5, 0.0),
            b: v(0.5, 0.5, 0.0),
            radius: n(0.0)
        },
        &tri
    ));

    for (x, hit) in [(0.0, true), (2.0, true), (2.25, false)] {
        let other = Obb {
            center: v(x, 0.0, 0.0),
            ..b
        };
        assert_eq!(box_box(&b, &other), hit);
        assert_eq!(box_box(&other, &b), hit);
    }
    for (z, hit) in [(0.0, true), (1.0, true), (1.25, false)] {
        let other = Tri3::new([v(0.0, 0.0, z), v(0.5, 0.0, z), v(0.0, 0.5, z)]);
        assert_eq!(box_triangle(&b, &other), hit);
    }
    for (other, hit) in [
        (tri, true),
        (
            Tri3::new([v(2.0, 0.0, 0.0), v(3.0, 0.0, 0.0), v(2.0, 1.0, 0.0)]),
            true,
        ),
        // Overlapping coordinate bounds, but disjoint in the triangle plane.
        (
            Tri3::new([v(1.5, 1.5, 0.0), v(2.0, 1.5, 0.0), v(1.5, 2.0, 0.0)]),
            false,
        ),
        (
            Tri3::new([v(0.0, 0.0, 0.5), v(2.0, 0.0, 0.5), v(0.0, 2.0, 0.5)]),
            false,
        ),
        (
            Tri3::new([v(0.5, 0.5, -1.0), v(0.5, 0.5, 1.0), v(1.0, 0.5, 0.0)]),
            true,
        ),
        (
            Tri3::new([v(2.0, 2.0, -1.0), v(2.0, 2.0, 1.0), v(3.0, 2.0, 0.0)]),
            false,
        ),
    ] {
        assert_eq!(triangle_triangle(&tri, &other), hit);
        assert_eq!(triangle_triangle(&other, &tri), hit);
        let vertices = other.vertices();
        let reversed = Tri3::new([vertices[2], vertices[1], vertices[0]]);
        assert_eq!(triangle_triangle(&tri, &reversed), hit);
    }
}

/// A quarter turn with translation, exact in all supported test formats.
fn transform<T: RealScalar + TryCastFrom<f64>>(p: Vector3<T>) -> Vector3<T> {
    Vector3::new(-p.y + n::<T>(0.5), p.x - n::<T>(0.25), p.z + n::<T>(0.125))
}

/// Validates rotated-box coordinates and invariance under rigid motion, beyond
/// the axis-aligned contact fixtures. Triangle winding is preserved.
fn rotated<T: RealScalar + TryCastFrom<f64>>() {
    let base = unit_box::<T>();
    let b = Obb {
        center: transform(base.center),
        axes: [v(0.0, 1.0, 0.0), v(-1.0, 0.0, 0.0), v(0.0, 0.0, 1.0)],
        half: v(0.5, 1.0, 0.75),
    };
    let a = Obb {
        half: b.half,
        ..base
    };
    for x in [0.25, 0.75, 1.0, 1.5, 2.5] {
        let ball = sphere(x, 0.25, 0.0, 0.25);
        let moved = Sphere3::new(transform(ball.center), ball.radius);
        assert_eq!(sphere_box(&ball, &a), sphere_box(&moved, &b));
        let cap = Capsule {
            a: v(x, -0.5, -0.25),
            b: v(x, 0.5, 0.25),
            radius: n(0.25),
        };
        let moved = Capsule {
            a: transform(cap.a),
            b: transform(cap.b),
            radius: cap.radius,
        };
        assert_eq!(capsule_box(&cap, &a), capsule_box(&moved, &b));
        let tri = Tri3::new([v(x, -0.5, 0.0), v(x, 0.5, 0.0), v(x + 0.5, 0.0, 0.5)]);
        let verts = tri.vertices();
        let moved = Tri3::new([
            transform(verts[0]),
            transform(verts[1]),
            transform(verts[2]),
        ]);
        assert_eq!(box_triangle(&a, &tri), box_triangle(&b, &moved));
    }
}

/// Runs the analytic suite in each scalar configuration; fixed-only builds keep
/// all three fixed suites even when no floating analytic backend is available.
macro_rules! backend {
    ($name:ident, $t:ty) => {
        #[test]
        fn $name() {
            contacts::<$t>();
            rotated::<$t>();
        }
    };
}

#[cfg(any(feature = "std", feature = "libm", feature = "system-libm"))]
backend!(f32_contacts, f32);
#[cfg(any(feature = "std", feature = "libm", feature = "system-libm"))]
backend!(f64_contacts, f64);
backend!(s23_8_contacts, Fixed<i32, 8>);
backend!(s15_16_contacts, Fixed<i32, 16>);
backend!(s47_16_contacts, Fixed<i64, 16>);

/// Independent convex one-dimensional minimization checks the piecewise
/// segment-box solver against 1,000 varied f64 segments, including interior,
/// edge and corner minima. This oracle does not reuse slab splitting.
#[cfg(any(feature = "std", feature = "libm", feature = "system-libm"))]
#[test]
fn segment_box_matches_convex_distance_reference() {
    let mut state = 17u32;
    let mut coordinate = || {
        state = state.wrapping_mul(1664525).wrapping_add(1013904223);
        (state >> 16) as f64 / 8192.0 - 4.0
    };
    for _ in 0..1000 {
        let a = v::<f64>(coordinate(), coordinate(), coordinate());
        let b = v::<f64>(coordinate(), coordinate(), coordinate());
        let half = v(0.5, 0.75, 1.0);
        let distance = |t| {
            let p = a + (b - a) * t;
            [p.x.abs() - 0.5, p.y.abs() - 0.75, p.z.abs() - 1.0]
                .iter()
                .map(|x| x.max(0.0).powi(2))
                .sum::<f64>()
        };
        let (mut lo, mut hi) = (0.0, 1.0);
        for _ in 0..100 {
            let left = lo + (hi - lo) / 3.0;
            let right = hi - (hi - lo) / 3.0;
            if distance(left) < distance(right) {
                hi = right;
            } else {
                lo = left;
            }
        }
        let reference = distance((lo + hi) * 0.5);
        let result = segment_box_squared(a, b, half);
        assert!(
            (result - reference).abs() < 1e-10,
            "{} != {}",
            result,
            reference
        );
    }
}

/// A separate box-box oracle: clip all twelve edges of each box against the
/// other's slabs. This also detects containment and avoids SAT cross products.
#[cfg(any(feature = "std", feature = "libm", feature = "system-libm"))]
#[test]
fn oblique_boxes_match_edge_clipping_reference() {
    let mut state = 31u32;
    let mut random = || {
        state = state.wrapping_mul(1664525).wrapping_add(1013904223);
        (state >> 16) as f64 / 65536.0
    };
    let mut edge_only_misses = 0;
    for _ in 0..1000 {
        let mut make_box = || {
            let center = v::<f64>(
                random() * 3.0 - 1.5,
                random() * 3.0 - 1.5,
                random() * 3.0 - 1.5,
            );
            let (s, c) = (random() * 6.0).sin_cos();
            let (sy, cy) = (random() * 6.0).sin_cos();
            Obb {
                center,
                axes: [v(c * cy, s * cy, -sy), v(-s, c, 0.0), v(c * sy, s * sy, cy)],
                half: v(0.25 + random(), 0.25 + random(), 0.25 + random()),
            }
        };
        let a = make_box();
        let b = make_box();
        let edge_hits = |source: &Obb<f64>, target: &Obb<f64>| {
            let corners: [Vector3<f64>; 8] = core::array::from_fn(|i| {
                let signs: [f64; 3] =
                    core::array::from_fn(|axis| if i & (1 << axis) == 0 { -1.0 } else { 1.0 });
                source.center
                    + source.axes[0] * (source.half.x * signs[0])
                    + source.axes[1] * (source.half.y * signs[1])
                    + source.axes[2] * (source.half.z * signs[2])
            });
            for i in 0..8 {
                for axis in 0..3 {
                    let j = i ^ (1 << axis);
                    if j <= i {
                        continue;
                    }
                    let p = local(target, corners[i]);
                    let q = local(target, corners[j]);
                    let origins = [p.x, p.y, p.z];
                    let directions = [q.x - p.x, q.y - p.y, q.z - p.z];
                    let halves = [target.half.x, target.half.y, target.half.z];
                    let (mut enter, mut leave) = (0.0f64, 1.0f64);
                    for axis in 0..3 {
                        if directions[axis] == 0.0 {
                            if origins[axis].abs() > halves[axis] {
                                leave = -1.0;
                                break;
                            }
                        } else {
                            let t0 = (-halves[axis] - origins[axis]) / directions[axis];
                            let t1 = (halves[axis] - origins[axis]) / directions[axis];
                            enter = enter.max(t0.min(t1));
                            leave = leave.min(t0.max(t1));
                        }
                    }
                    if enter <= leave {
                        return true;
                    }
                }
            }
            false
        };
        let reference = edge_hits(&a, &b) || edge_hits(&b, &a);
        assert_eq!(box_box(&a, &b), reference);
        assert_eq!(box_box(&b, &a), reference);
        let face_separated = a.axes.iter().chain(b.axes.iter()).any(|&axis| {
            dot(b.center - a.center, axis).abs() > box_radius(&a, axis) + box_radius(&b, axis)
        });
        if !reference && !face_separated {
            edge_only_misses += 1;
        }
    }
    assert!(
        edge_only_misses > 0,
        "fixtures must require edge-cross separating axes"
    );
}
