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

//! Analytic contacts, solver invariants and deterministic fixed-step regressions.
use super::*;
use rs_math3d::{Fixed, Scalar, Tri3};

fn v<T: RealScalar + TryCastFrom<f64>>(x: f64, y: f64, z: f64) -> Vector3<T> {
    Vector3::new(scalar(x), scalar(y), scalar(z))
}
fn material<T: RealScalar + TryCastFrom<f64>>() -> Material<T> {
    Material {
        friction: scalar(0.6),
        restitution: scalar(0.0),
    }
}
fn floor<T: RealScalar + TryCastFrom<f64>>(world: &mut World<T>) {
    let mut triangles = Vec::new();
    for x in -2..2 {
        for z in -2..2 {
            let (x, z) = (x as f64 * 2.0, z as f64 * 2.0);
            let (a, b, c, d) = (
                v(x, 0.0, z),
                v(x + 2.0, 0.0, z),
                v(x + 2.0, 0.0, z + 2.0),
                v(x, 0.0, z + 2.0),
            );
            triangles.push(Tri3::new([a, c, b]));
            triangles.push(Tri3::new([a, d, c]));
        }
    }
    world
        .add_mesh(StaticMesh {
            tree: AabbTree::new(triangles, 4).unwrap(),
            material: material(),
        })
        .unwrap();
}
fn contacts<T: RealScalar + TryCastFrom<f64>>() {
    let shapes: [Shape<T>; 3] = [
        Shape::Sphere {
            radius: scalar(0.5),
        },
        Shape::Capsule {
            half_height: scalar(0.5),
            radius: scalar(0.5),
        },
        Shape::Box {
            half: v(0.5, 0.5, 0.5),
        },
    ];
    for a in shapes {
        for b in shapes {
            let a = Body::new(a, Pose::at(v(0.0, 0.0, 0.0)), scalar(1.0), material()).unwrap();
            let b = Body::new(b, Pose::at(v(0.75, 0.0, 0.0)), scalar(1.0), material()).unwrap();
            let manifold = contact::convex(&a.convex(), &b.convex());
            assert!(!manifold.points().is_empty());
            for c in manifold.points() {
                assert!(c.normal.x < scalar(-0.8));
                assert!(c.depth >= T::zero());
                assert!(finite(c.point));
            }
            let reverse = contact::convex(&b.convex(), &a.convex());
            assert!(!reverse.points().is_empty());
            assert!(reverse.points()[0].normal.x > scalar(0.8));
            let far =
                Body::new(b.shape, Pose::at(v(3.0, 0.0, 0.0)), scalar(1.0), material()).unwrap();
            assert!(contact::convex(&a.convex(), &far.convex())
                .points()
                .is_empty());
        }
    }
}

#[test]
fn coincident_capsule_centerlines_escape_perpendicularly() {
    type T = Fixed<i32, 16>;
    let capsule = Convex::Capsule(collision::Capsule {
        a: v::<T>(0., -1., 0.),
        b: v(0., 1., 0.),
        radius: scalar(0.5),
    });
    for a in [
        Convex::Sphere(Sphere3::new(v(0., 0., 0.), scalar(0.25))),
        Convex::Capsule(collision::Capsule {
            a: v(0., -0.5, 0.),
            b: v(0., 0.5, 0.),
            radius: scalar(0.25),
        }),
    ] {
        let manifold = contact::convex(&a, &capsule);
        let c = manifold.points()[0];
        assert!(c.normal.y.tabs() < scalar(0.001));
        assert!((c.depth - scalar::<T>(0.75)).tabs() < scalar(0.001));
        let offset = c.normal * (c.depth + scalar(0.01));
        let moved = match a {
            Convex::Sphere(s) => Convex::Sphere(Sphere3::new(s.center + offset, s.radius)),
            Convex::Capsule(s) => Convex::Capsule(collision::Capsule {
                a: s.a + offset,
                b: s.b + offset,
                radius: s.radius,
            }),
            Convex::Box(_) => unreachable!(),
        };
        assert!(contact::convex(&moved, &capsule).points().is_empty());
    }
}

#[test]
fn tree_visitor_returns_all_closed_candidates() {
    type T = Fixed<i32, 16>;
    let tri = |x: f64| Tri3::new([v::<T>(x, 0., 0.), v(x + 1., 0., 0.), v(x, 0., 1.)]);
    let tree = AabbTree::new(vec![tri(0.), tri(1.), tri(4.)], 1).unwrap();
    let bounds = Box3::new(&v(0.5, -1., 0.2), &v(1., 1., 0.8));
    let mut ids = Vec::new();
    let stats = tree.for_each_candidate(&bounds, |id, _| ids.push(id));
    ids.sort_unstable();
    assert_eq!(ids, vec![0, 1]);
    assert_eq!(stats.triangle_tests, 2);
    let empty = AabbTree::<T>::new(Vec::new(), 1).unwrap();
    assert_eq!(
        empty
            .for_each_candidate(&bounds, |_, _| panic!("empty visitor"))
            .bounds_tests,
        0
    );
}

#[test]
fn box_face_manifold_has_spread_and_depth() {
    type T = Fixed<i32, 16>;
    let shape = Shape::Box {
        half: v::<T>(0.5, 0.5, 0.5),
    };
    let a = Body::new(shape, Pose::at(v(0., 0., 0.)), scalar(1.), material()).unwrap();
    let b = Body::new(shape, Pose::at(v(0., -0.75, 0.)), scalar(1.), material()).unwrap();
    let m = contact::convex(&a.convex(), &b.convex());
    assert_eq!(m.points().len(), 4);
    for p in m.points() {
        assert!((p.depth - scalar::<T>(0.25)).tabs() < scalar(0.001));
        assert!(p.normal.y > scalar(0.99));
    }
    assert!(magnitude(m.points()[0].point - m.points()[1].point) > scalar(0.5));
}

#[test]
fn sleeping_body_wakes_on_external_impulse() {
    type T = Fixed<i64, 32>;
    let mut w = World::<T>::new();
    floor(&mut w);
    let b = w
        .add_body(
            Body::new(
                Shape::Sphere {
                    radius: scalar(0.4),
                },
                Pose::at(v(0., 0.4, 0.)),
                scalar(1.),
                material(),
            )
            .unwrap(),
        )
        .unwrap();
    for _ in 0..240 {
        w.step();
    }
    assert!(w.body(b).unwrap().sleeping());
    let p = w.body(b).unwrap().pose.position;
    w.body_mut(b)
        .unwrap()
        .apply_impulse(p, v(0., 2., 0.))
        .unwrap();
    assert!(!w.body(b).unwrap().sleeping());
    w.step();
    assert!(w.body(b).unwrap().velocity.y > scalar(1.8));
}

#[test]
fn friction_slows_a_sliding_box() {
    type T = Fixed<i64, 32>;
    let mut w = World::<T>::new();
    floor(&mut w);
    let mut body = Body::new(
        Shape::Box {
            half: v(0.5, 0.5, 0.5),
        },
        Pose::at(v(-2., 0.5, 0.)),
        scalar(1.),
        material(),
    )
    .unwrap();
    body.set_velocity(v(2., 0., 0.), zero()).unwrap();
    let b = w.add_body(body).unwrap();
    for _ in 0..240 {
        w.step();
    }
    assert!(w.body(b).unwrap().velocity.x.tabs() < scalar(0.2));
    assert!(w.body(b).unwrap().pose.position.y > scalar(0.45));
}

#[test]
fn off_center_impulse_changes_spin() {
    type T = Fixed<i64, 32>;
    let mut body = Body::new(
        Shape::Sphere {
            radius: scalar::<T>(1.),
        },
        Pose::at(zero()),
        scalar(1.),
        material(),
    )
    .unwrap();
    body.apply_impulse(v(0., 1., 0.), v(1., 0., 0.)).unwrap();
    assert!((body.angular.z - scalar::<T>(-2.5)).tabs() < scalar(0.0001));
    assert_eq!(body.velocity.x, T::ONE);
}

#[test]
fn fixed_ticks_repeat_bit_for_bit() {
    type T = Fixed<i32, 16>;
    let build = || {
        let mut w = World::<T>::new();
        floor(&mut w);
        w.add_body(
            Body::new(
                Shape::Box {
                    half: v(0.4, 0.4, 0.4),
                },
                Pose::at(v(0., 2., 0.)),
                scalar(1.),
                material(),
            )
            .unwrap(),
        )
        .unwrap();
        w
    };
    let mut a = build();
    let mut b = build();
    for _ in 0..240 {
        a.step();
        b.step();
    }
    let a = &a.bodies[0];
    let b = &b.bodies[0];
    assert_eq!(
        (a.pose.position.x, a.pose.position.y, a.pose.position.z),
        (b.pose.position.x, b.pose.position.y, b.pose.position.z)
    );
    assert_eq!(
        (
            a.pose.orientation.x,
            a.pose.orientation.y,
            a.pose.orientation.z,
            a.pose.orientation.w
        ),
        (
            b.pose.orientation.x,
            b.pose.orientation.y,
            b.pose.orientation.z,
            b.pose.orientation.w
        )
    );
}

/// An independent exhaustive AABB-pair oracle checks sweep pruning and stable
/// insertion order after bodies exchange X positions, without using narrow phase.
#[test]
fn sweep_matches_flat_pairs_after_reordering() {
    type T = Fixed<i32, 16>;
    let mut world = World::<T>::new();
    for i in 0..12 {
        world
            .add_body(
                Body::new(
                    Shape::Box {
                        half: v(0.5, 0.5, 0.5),
                    },
                    Pose::at(v((i % 4) as f64 * 0.8, (i / 4) as f64 * 0.8, 0.)),
                    scalar(1.),
                    material(),
                )
                .unwrap(),
            )
            .unwrap();
    }
    for _ in 0..3 {
        world.broad_phase();
        let mut actual: Vec<_> = world
            .candidates
            .iter()
            .filter_map(|c| match *c {
                Candidate::Bodies { a, b } => Some((a, b)),
                Candidate::Triangle { .. } => None,
            })
            .collect();
        actual.sort_unstable();
        let mut expected = Vec::new();
        for a in 0..world.bodies.len() {
            for b in a + 1..world.bodies.len() {
                if contact::overlaps(&world.bodies[a].bounds(), &world.bodies[b].bounds()) {
                    expected.push((a, b));
                }
            }
        }
        assert_eq!(actual, expected);
        for (i, body) in world.bodies.iter_mut().enumerate() {
            body.pose.position.x = scalar((11 - i) as f64 * 0.3);
        }
    }
}

#[test]
fn escaped_body_has_a_terminal_motion_state() {
    type T = Fixed<i32, 16>;
    let mut w = World::<T>::new();
    w.set_gravity(zero()).unwrap();
    let mut body = Body::new(
        Shape::Sphere {
            radius: scalar(0.4),
        },
        Pose::at(v(23.95, 2., 0.)),
        scalar(1.),
        material(),
    )
    .unwrap();
    body.set_velocity(v(15., 0., 0.), zero()).unwrap();
    let id = w.add_body(body).unwrap();
    w.step();
    assert!(!w.body(id).unwrap().enabled());
    assert_eq!(w.stats().disabled, 1);
    assert_eq!(
        w.body_mut(id).unwrap().set_velocity(zero(), zero()),
        Err(Error::InactiveBody)
    );
    assert_eq!(
        w.body_mut(id).unwrap().apply_impulse(zero(), v(1., 0., 0.)),
        Err(Error::InactiveBody)
    );
}
fn settling<T: RealScalar + TryCastFrom<f64>>() {
    let mut w = World::<T>::new();
    floor(&mut w);
    let shapes = [
        Shape::Sphere {
            radius: scalar(0.4),
        },
        Shape::Capsule {
            half_height: scalar(0.4),
            radius: scalar(0.3),
        },
        Shape::Box {
            half: v(0.4, 0.4, 0.4),
        },
    ];
    let mut ids = Vec::new();
    for (i, shape) in shapes.iter().enumerate() {
        ids.push(
            w.add_body(
                Body::new(
                    *shape,
                    Pose::at(v(-2.0 + i as f64 * 2.0, 2.0, 0.0)),
                    scalar(1.0),
                    material(),
                )
                .unwrap(),
            )
            .unwrap(),
        );
    }
    for _ in 0..480 {
        w.step();
    }
    for (id, height) in ids.into_iter().zip([0.4, 0.7, 0.4]) {
        let b = w.body(id).unwrap();
        assert!(b.enabled());
        assert!((b.pose.position.y - scalar::<T>(height)).tabs() < scalar(0.1));
        assert!(magnitude(b.velocity) < scalar::<T>(0.2));
    }
}
#[test]
fn fixed32_all_pairs() {
    contacts::<Fixed<i32, 16>>();
}
#[test]
fn fixed32_settles() {
    settling::<Fixed<i32, 16>>();
}
#[test]
fn fixed64_settles() {
    settling::<Fixed<i64, 32>>();
}
#[cfg(any(feature = "std", feature = "libm", feature = "system-libm"))]
#[test]
fn float_settles() {
    settling::<f32>();
}

#[test]
fn construction_and_capacity() {
    type T = Fixed<i32, 16>;
    let mut w = World::<T>::new();
    let shape = Shape::Sphere {
        radius: scalar(0.4),
    };
    assert!(matches!(
        Body::new(shape, Pose::at(v(0., 0., 0.)), T::ZERO, material()),
        Err(Error::InvalidBody)
    ));
    assert!(matches!(
        Body::new(
            shape,
            Pose {
                position: zero(),
                orientation: Quat::new(T::ZERO, T::ZERO, T::ZERO, T::ZERO)
            },
            T::ONE,
            material()
        ),
        Err(Error::InvalidPose)
    ));
    for _ in 0..128 {
        w.add_body(Body::new(shape, Pose::at(v(0., 2., 0.)), scalar(1.), material()).unwrap())
            .unwrap();
    }
    assert_eq!(
        w.add_body(Body::new(shape, Pose::at(v(0., 2., 0.)), scalar(1.), material()).unwrap()),
        Err(Error::Capacity)
    );
}

#[test]
fn two_box_stack_remains_supported() {
    type T = Fixed<i64, 32>;
    let mut w = World::<T>::new();
    floor(&mut w);
    let a = w
        .add_body(
            Body::new(
                Shape::Box {
                    half: v(0.4, 0.4, 0.4),
                },
                Pose::at(v(0., 0.4, 0.)),
                scalar(1.),
                material(),
            )
            .unwrap(),
        )
        .unwrap();
    let b = w
        .add_body(
            Body::new(
                Shape::Box {
                    half: v(0.4, 0.4, 0.4),
                },
                Pose::at(v(0., 1.2, 0.)),
                scalar(1.),
                material(),
            )
            .unwrap(),
        )
        .unwrap();
    for _ in 0..600 {
        w.step();
    }
    assert!((w.body(a).unwrap().pose.position.y - scalar::<T>(0.4)).abs() < scalar(0.08));
    assert!((w.body(b).unwrap().pose.position.y - scalar::<T>(1.2)).abs() < scalar(0.08));
    assert!(magnitude(w.body(b).unwrap().velocity) < scalar(0.2));
}

#[test]
fn head_on_impulses_preserve_linear_momentum() {
    type T = Fixed<i64, 32>;
    let mut w = World::<T>::new();
    w.set_gravity(zero()).unwrap();
    let shape = Shape::Sphere {
        radius: scalar(0.5),
    };
    let mut a = Body::new(
        shape,
        Pose::at(v(-0.49, 0., 0.)),
        scalar(1.),
        Material {
            friction: T::ZERO,
            restitution: T::ONE,
        },
    )
    .unwrap();
    a.set_velocity(v(2., 0., 0.), zero()).unwrap();
    let mut b = Body::new(
        shape,
        Pose::at(v(0.49, 0., 0.)),
        scalar(1.),
        Material {
            friction: T::ZERO,
            restitution: T::ONE,
        },
    )
    .unwrap();
    b.set_velocity(v(-2., 0., 0.), zero()).unwrap();
    let a = w.add_body(a).unwrap();
    let b = w.add_body(b).unwrap();
    w.step();
    assert!(w.body(a).unwrap().velocity.x < scalar(-1.9));
    assert!(w.body(b).unwrap().velocity.x > scalar(1.9));
    assert!(magnitude(w.body(a).unwrap().velocity + w.body(b).unwrap().velocity) < scalar(0.0001));
}
