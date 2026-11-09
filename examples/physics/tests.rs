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

fn material<T: RealScalar + TryCastFrom<f64>>() -> Material<T> {
    Material {
        friction: T::cast_from(0.6),
        restitution: T::cast_from(0.0),
    }
}
fn floor<T: RealScalar + TryCastFrom<f64>>(world: &mut World<T>) {
    let mut triangles = Vec::new();
    for x in -2..2 {
        for z in -2..2 {
            let (x, z) = (x as f64 * 2.0, z as f64 * 2.0);
            let (a, b, c, d) = (
                Vector3::from_array([x, 0.0, z]).map(T::cast_from),
                Vector3::from_array([x + 2.0, 0.0, z]).map(T::cast_from),
                Vector3::from_array([x + 2.0, 0.0, z + 2.0]).map(T::cast_from),
                Vector3::from_array([x, 0.0, z + 2.0]).map(T::cast_from),
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
            radius: T::cast_from(0.5),
        },
        Shape::Capsule {
            half_height: T::cast_from(0.5),
            radius: T::cast_from(0.5),
        },
        Shape::Box {
            half: Vector3::from_array([0.5, 0.5, 0.5]).map(T::cast_from),
        },
    ];
    for a in shapes {
        for b in shapes {
            let a = Body::new(
                a,
                Pose::at(Vector3::from_array([0.0, 0.0, 0.0]).map(T::cast_from)),
                T::cast_from(1.0),
                material(),
            )
            .unwrap();
            let b = Body::new(
                b,
                Pose::at(Vector3::from_array([0.75, 0.0, 0.0]).map(T::cast_from)),
                T::cast_from(1.0),
                material(),
            )
            .unwrap();
            let manifold = contact::convex(&a.convex(), &b.convex());
            assert!(!manifold.points().is_empty());
            for c in manifold.points() {
                assert!(c.normal.x < T::cast_from(-0.8));
                assert!(c.depth >= T::zero());
                assert!(c.point.is_finite());
            }
            let reverse = contact::convex(&b.convex(), &a.convex());
            assert!(!reverse.points().is_empty());
            assert!(reverse.points()[0].normal.x > T::cast_from(0.8));
            let far = Body::new(
                b.shape,
                Pose::at(Vector3::from_array([3.0, 0.0, 0.0]).map(T::cast_from)),
                T::cast_from(1.0),
                material(),
            )
            .unwrap();
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
        a: Vector3::from_array([0., -1., 0.]).map(<T>::cast_from),
        b: Vector3::from_array([0., 1., 0.]).map(T::cast_from),
        radius: T::cast_from(0.5),
    });
    for a in [
        Convex::Sphere(Sphere3::new(
            Vector3::from_array([0., 0., 0.]).map(T::cast_from),
            T::cast_from(0.25),
        )),
        Convex::Capsule(collision::Capsule {
            a: Vector3::from_array([0., -0.5, 0.]).map(T::cast_from),
            b: Vector3::from_array([0., 0.5, 0.]).map(T::cast_from),
            radius: T::cast_from(0.25),
        }),
    ] {
        let manifold = contact::convex(&a, &capsule);
        let c = manifold.points()[0];
        assert!(c.normal.y.tabs() < T::cast_from(0.001));
        assert!((c.depth - <T>::cast_from(0.75)).tabs() < T::cast_from(0.001));
        let offset = c.normal * (c.depth + T::cast_from(0.01));
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
    let tri = |x: f64| {
        Tri3::new([
            Vector3::from_array([x, 0., 0.]).map(<T>::cast_from),
            Vector3::from_array([x + 1., 0., 0.]).map(T::cast_from),
            Vector3::from_array([x, 0., 1.]).map(T::cast_from),
        ])
    };
    let tree = AabbTree::new(vec![tri(0.), tri(1.), tri(4.)], 1).unwrap();
    let bounds = Box3::new(
        &Vector3::from_array([0.5, -1., 0.2]).map(T::cast_from),
        &Vector3::from_array([1., 1., 0.8]).map(T::cast_from),
    );
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
        half: Vector3::from_array([0.5, 0.5, 0.5]).map(<T>::cast_from),
    };
    let a = Body::new(
        shape,
        Pose::at(Vector3::from_array([0., 0., 0.]).map(T::cast_from)),
        T::cast_from(1.),
        material(),
    )
    .unwrap();
    let b = Body::new(
        shape,
        Pose::at(Vector3::from_array([0., -0.75, 0.]).map(T::cast_from)),
        T::cast_from(1.),
        material(),
    )
    .unwrap();
    let m = contact::convex(&a.convex(), &b.convex());
    assert_eq!(m.points().len(), 4);
    for p in m.points() {
        assert!((p.depth - <T>::cast_from(0.25)).tabs() < T::cast_from(0.001));
        assert!(p.normal.y > T::cast_from(0.99));
    }
    assert!((m.points()[0].point - m.points()[1].point).max_abs_component() > T::cast_from(0.5));
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
                    radius: T::cast_from(0.4),
                },
                Pose::at(Vector3::from_array([0., 0.4, 0.]).map(T::cast_from)),
                T::cast_from(1.),
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
        .apply_impulse(p, Vector3::from_array([0., 2., 0.]).map(T::cast_from))
        .unwrap();
    assert!(!w.body(b).unwrap().sleeping());
    w.step();
    assert!(w.body(b).unwrap().velocity.y > T::cast_from(1.8));
}

#[test]
fn friction_slows_a_sliding_box() {
    type T = Fixed<i64, 32>;
    let mut w = World::<T>::new();
    floor(&mut w);
    let mut body = Body::new(
        Shape::Box {
            half: Vector3::from_array([0.5, 0.5, 0.5]).map(T::cast_from),
        },
        Pose::at(Vector3::from_array([-2., 0.5, 0.]).map(T::cast_from)),
        T::cast_from(1.),
        material(),
    )
    .unwrap();
    body.set_velocity(
        Vector3::from_array([2., 0., 0.]).map(T::cast_from),
        Vector3::zero(),
    )
    .unwrap();
    let b = w.add_body(body).unwrap();
    for _ in 0..240 {
        w.step();
    }
    assert!(w.body(b).unwrap().velocity.x.tabs() < T::cast_from(0.2));
    assert!(w.body(b).unwrap().pose.position.y > T::cast_from(0.45));
}

#[test]
fn off_center_impulse_changes_spin() {
    type T = Fixed<i64, 32>;
    let mut body = Body::new(
        Shape::Sphere {
            radius: <T>::cast_from(1.),
        },
        Pose::at(Vector3::zero()),
        T::cast_from(1.),
        material(),
    )
    .unwrap();
    body.apply_impulse(
        Vector3::from_array([0., 1., 0.]).map(T::cast_from),
        Vector3::from_array([1., 0., 0.]).map(T::cast_from),
    )
    .unwrap();
    assert!((body.angular.z - <T>::cast_from(-2.5)).tabs() < T::cast_from(0.0001));
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
                    half: Vector3::from_array([0.4, 0.4, 0.4]).map(T::cast_from),
                },
                Pose::at(Vector3::from_array([0., 2., 0.]).map(T::cast_from)),
                T::cast_from(1.),
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

/// Reversing/shuffling broad-phase output must not affect seam reduction or
/// sequential impulses. Dense overlapping bodies exercise both candidate kinds.
#[test]
fn solver_is_independent_of_candidate_visitation_order() {
    type T = Fixed<i32, 16>;
    let build = || {
        let mut w = World::<T>::new();
        floor(&mut w);
        for i in 0..6 {
            w.add_body(
                Body::new(
                    Shape::Box {
                        half: Vector3::from_array([0.45, 0.4, 0.45]).map(T::cast_from),
                    },
                    Pose::at(
                        Vector3::from_array([
                            (i % 3) as f64 * 0.75,
                            0.35 + (i / 3) as f64 * 0.7,
                            0.,
                        ])
                        .map(T::cast_from),
                    ),
                    T::cast_from(1.),
                    material(),
                )
                .unwrap(),
            )
            .unwrap();
        }
        w
    };
    let mut a = build();
    let mut b = build();
    for tick in 0..180 {
        a.step();
        b.stats = Stats::default();
        b.integrate();
        b.broad_phase();
        b.candidates.reverse();
        if !b.candidates.is_empty() {
            let offset = tick % b.candidates.len();
            b.candidates.rotate_left(offset);
        }
        b.narrow_phase();
        b.solve_velocity();
        b.solve_position();
        b.sleep();
        assert_eq!(a.contacts.len(), b.contacts.len());
        for (a, b) in a.bodies.iter().zip(&b.bodies) {
            let values = |b: &Body<T>| {
                [
                    b.pose.position.x,
                    b.pose.position.y,
                    b.pose.position.z,
                    b.pose.orientation.x,
                    b.pose.orientation.y,
                    b.pose.orientation.z,
                    b.pose.orientation.w,
                    b.velocity.x,
                    b.velocity.y,
                    b.velocity.z,
                    b.angular.x,
                    b.angular.y,
                    b.angular.z,
                ]
                .map(|x| x.to_bits())
            };
            assert_eq!(values(a), values(b), "tick {tick}");
        }
        for (a, b) in a.contacts.iter().zip(&b.contacts) {
            assert_eq!(a.impulse, b.impulse, "warm cache at tick {tick}");
        }
    }
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
                        half: Vector3::from_array([0.5, 0.5, 0.5]).map(T::cast_from),
                    },
                    Pose::at(
                        Vector3::from_array([(i % 4) as f64 * 0.8, (i / 4) as f64 * 0.8, 0.])
                            .map(T::cast_from),
                    ),
                    T::cast_from(1.),
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
                if world.bodies[a].bounds().overlap(&world.bodies[b].bounds()) {
                    expected.push((a, b));
                }
            }
        }
        assert_eq!(actual, expected);
        for (i, body) in world.bodies.iter_mut().enumerate() {
            body.pose.position.x = T::cast_from((11 - i) as f64 * 0.3);
        }
    }
}

#[test]
fn escaped_body_has_a_terminal_motion_state() {
    type T = Fixed<i32, 16>;
    let mut w = World::<T>::new();
    w.set_gravity(Vector3::zero()).unwrap();
    let mut body = Body::new(
        Shape::Sphere {
            radius: T::cast_from(0.4),
        },
        Pose::at(Vector3::from_array([23.95, 2., 0.]).map(T::cast_from)),
        T::cast_from(1.),
        material(),
    )
    .unwrap();
    body.set_velocity(
        Vector3::from_array([15., 0., 0.]).map(T::cast_from),
        Vector3::zero(),
    )
    .unwrap();
    let id = w.add_body(body).unwrap();
    w.step();
    assert!(!w.body(id).unwrap().enabled());
    assert_eq!(w.stats().disabled, 1);
    w.set_gravity(Vector3::from_array([0., 9.81, 0.]).map(T::cast_from))
        .unwrap();
    assert!(!w.body(id).unwrap().enabled());
    assert_eq!(
        w.body_mut(id)
            .unwrap()
            .set_velocity(Vector3::zero(), Vector3::zero()),
        Err(Error::InactiveBody)
    );
    assert_eq!(
        w.body_mut(id).unwrap().apply_impulse(
            Vector3::zero(),
            Vector3::from_array([1., 0., 0.]).map(T::cast_from)
        ),
        Err(Error::InactiveBody)
    );
}
fn settling<T: RealScalar + TryCastFrom<f64>>() {
    let mut w = World::<T>::new();
    floor(&mut w);
    let shapes = [
        Shape::Sphere {
            radius: T::cast_from(0.4),
        },
        Shape::Capsule {
            half_height: T::cast_from(0.4),
            radius: T::cast_from(0.3),
        },
        Shape::Box {
            half: Vector3::from_array([0.4, 0.4, 0.4]).map(T::cast_from),
        },
    ];
    let mut ids = Vec::new();
    for (i, shape) in shapes.iter().enumerate() {
        ids.push(
            w.add_body(
                Body::new(
                    *shape,
                    Pose::at(
                        Vector3::from_array([-2.0 + i as f64 * 2.0, 2.0, 0.0]).map(T::cast_from),
                    ),
                    T::cast_from(1.0),
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
        assert!((b.pose.position.y - <T>::cast_from(height)).tabs() < T::cast_from(0.1));
        assert!(b.velocity.max_abs_component() < <T>::cast_from(0.2));
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
        radius: T::cast_from(0.4),
    };
    assert!(matches!(
        Body::new(
            shape,
            Pose::at(Vector3::from_array([0., 0., 0.]).map(T::cast_from)),
            T::ZERO,
            material()
        ),
        Err(Error::InvalidBody)
    ));
    assert!(matches!(
        Body::new(
            shape,
            Pose {
                position: Vector3::zero(),
                orientation: Quat::new(T::ZERO, T::ZERO, T::ZERO, T::ZERO)
            },
            T::ONE,
            material()
        ),
        Err(Error::InvalidPose)
    ));
    for _ in 0..128 {
        w.add_body(
            Body::new(
                shape,
                Pose::at(Vector3::from_array([0., 2., 0.]).map(T::cast_from)),
                T::cast_from(1.),
                material(),
            )
            .unwrap(),
        )
        .unwrap();
    }
    assert_eq!(
        w.add_body(
            Body::new(
                shape,
                Pose::at(Vector3::from_array([0., 2., 0.]).map(T::cast_from)),
                T::cast_from(1.),
                material()
            )
            .unwrap()
        ),
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
                    half: Vector3::from_array([0.4, 0.4, 0.4]).map(T::cast_from),
                },
                Pose::at(Vector3::from_array([0., 0.4, 0.]).map(T::cast_from)),
                T::cast_from(1.),
                material(),
            )
            .unwrap(),
        )
        .unwrap();
    let b = w
        .add_body(
            Body::new(
                Shape::Box {
                    half: Vector3::from_array([0.4, 0.4, 0.4]).map(T::cast_from),
                },
                Pose::at(Vector3::from_array([0., 1.2, 0.]).map(T::cast_from)),
                T::cast_from(1.),
                material(),
            )
            .unwrap(),
        )
        .unwrap();
    for _ in 0..600 {
        w.step();
    }
    assert!((w.body(a).unwrap().pose.position.y - <T>::cast_from(0.4)).abs() < T::cast_from(0.08));
    assert!((w.body(b).unwrap().pose.position.y - <T>::cast_from(1.2)).abs() < T::cast_from(0.08));
    assert!(w.body(b).unwrap().velocity.max_abs_component() < T::cast_from(0.2));
}

#[test]
fn head_on_impulses_preserve_linear_momentum() {
    type T = Fixed<i64, 32>;
    let mut w = World::<T>::new();
    w.set_gravity(Vector3::zero()).unwrap();
    let shape = Shape::Sphere {
        radius: T::cast_from(0.5),
    };
    let mut a = Body::new(
        shape,
        Pose::at(Vector3::from_array([-0.49, 0., 0.]).map(T::cast_from)),
        T::cast_from(1.),
        Material {
            friction: T::ZERO,
            restitution: T::ONE,
        },
    )
    .unwrap();
    a.set_velocity(
        Vector3::from_array([2., 0., 0.]).map(T::cast_from),
        Vector3::zero(),
    )
    .unwrap();
    let mut b = Body::new(
        shape,
        Pose::at(Vector3::from_array([0.49, 0., 0.]).map(T::cast_from)),
        T::cast_from(1.),
        Material {
            friction: T::ZERO,
            restitution: T::ONE,
        },
    )
    .unwrap();
    b.set_velocity(
        Vector3::from_array([-2., 0., 0.]).map(T::cast_from),
        Vector3::zero(),
    )
    .unwrap();
    let a = w.add_body(a).unwrap();
    let b = w.add_body(b).unwrap();
    w.step();
    assert!(w.body(a).unwrap().velocity.x < T::cast_from(-1.9));
    assert!(w.body(b).unwrap().velocity.x > T::cast_from(1.9));
    assert!(
        (w.body(a).unwrap().velocity + w.body(b).unwrap().velocity).max_abs_component()
            < T::cast_from(0.0001)
    );
}

/// A slowly moving support must not leave a frozen dependent after separation.
/// Run far beyond the quiet window so one-time waking cannot satisfy the test.
fn slow_support_loss<T: RealScalar + TryCastFrom<f64>>() {
    let mut w = World::<T>::new();
    floor(&mut w);
    let surface = Material {
        friction: T::zero(),
        restitution: T::zero(),
    };
    let shape = Shape::Box {
        half: Vector3::from_array([0.4, 0.4, 0.4]).map(T::cast_from),
    };
    let lower = w
        .add_body(
            Body::new(
                shape,
                Pose::at(Vector3::from_array([0., 0.4, 0.]).map(T::cast_from)),
                T::one(),
                surface,
            )
            .unwrap(),
        )
        .unwrap();
    let upper = w
        .add_body(
            Body::new(
                shape,
                Pose::at(Vector3::from_array([0., 1.2, 0.]).map(T::cast_from)),
                T::one(),
                surface,
            )
            .unwrap(),
        )
        .unwrap();
    for _ in 0..600 {
        w.step();
    }
    assert!(w.body(upper).unwrap().pose.position.y > T::cast_from(1.1));
    assert!(!w.body(lower).unwrap().sleeping());
    assert!(!w.body(upper).unwrap().sleeping());
    w.body_mut(lower)
        .unwrap()
        .set_velocity(
            Vector3::from_array([0.14, 0., 0.]).map(T::cast_from),
            Vector3::zero(),
        )
        .unwrap();
    for _ in 0..1200 {
        w.step();
        for contact in &w.contacts {
            if let Other::Body(b) = contact.other {
                assert!(!w.bodies[contact.a].sleeping());
                assert!(!w.bodies[b].sleeping());
            }
        }
    }
    assert!(w.body(lower).unwrap().pose.position.x > T::cast_from(0.8));
    assert!(w.body(upper).unwrap().pose.position.y < T::cast_from(0.6));
}
#[test]
fn fixed64_slow_support_loss_remains_dynamic() {
    slow_support_loss::<Fixed<i64, 32>>();
}
#[cfg(any(feature = "std", feature = "libm", feature = "system-libm"))]
#[test]
fn float_slow_support_loss_remains_dynamic() {
    slow_support_loss::<f32>();
}

/// Body/body contact stays active even at zero relative speed and zero gravity;
/// without static geometry there is no immutable support eligible for sleeping.
#[test]
fn fixed32_body_contacts_do_not_sleep() {
    type T = Fixed<i32, 16>;
    let mut w = World::<T>::new();
    w.set_gravity(Vector3::zero()).unwrap();
    for x in [T::ZERO, <T>::cast_from(0.4) * T::two()] {
        w.add_body(
            Body::new(
                Shape::Sphere {
                    radius: T::cast_from(0.4),
                },
                Pose::at(Vector3::new(x, T::ONE, T::ZERO)),
                T::ONE,
                material(),
            )
            .unwrap(),
        )
        .unwrap();
    }
    for _ in 0..240 {
        w.step();
        assert!(w.contacts.iter().any(|c| matches!(c.other, Other::Body(_))));
    }
    for (_, body) in w.bodies() {
        assert!(!body.sleeping());
    }
}

/// Accepted force changes invalidate sleep immediately; equal gravity does not
/// restart the quiet window. Static floor sleeping remains available afterward.
fn gravity_wakes<T: RealScalar + TryCastFrom<f64>>() {
    let mut w = World::<T>::new();
    floor(&mut w);
    let id = w
        .add_body(
            Body::new(
                Shape::Sphere {
                    radius: T::cast_from(0.4),
                },
                Pose::at(Vector3::from_array([0., 0.4, 0.]).map(T::cast_from)),
                T::one(),
                material(),
            )
            .unwrap(),
        )
        .unwrap();
    for _ in 0..480 {
        w.step();
    }
    assert!(w.body(id).unwrap().sleeping());
    let before = w.body(id).unwrap().pose.position.y;
    w.set_gravity(w.config.gravity).unwrap();
    assert!(w.body(id).unwrap().sleeping());
    w.set_gravity(Vector3::from_array([0., 9.81, 0.]).map(T::cast_from))
        .unwrap();
    assert!(!w.body(id).unwrap().sleeping());
    w.step();
    assert!(w.body(id).unwrap().velocity.y > T::zero());
    for _ in 0..119 {
        w.step();
    }
    assert!(w.body(id).unwrap().pose.position.y > before + T::cast_from(1.));
    let mut quiet = World::<T>::new();
    floor(&mut quiet);
    quiet
        .add_body(
            Body::new(
                Shape::Sphere {
                    radius: T::cast_from(0.4),
                },
                Pose::at(Vector3::from_array([0., 0.4, 0.]).map(T::cast_from)),
                T::one(),
                material(),
            )
            .unwrap(),
        )
        .unwrap();
    for _ in 0..20 {
        quiet.step();
    }
    let Motion::Awake {
        quiet_ticks: before,
    } = quiet.bodies[0].motion
    else {
        panic!("quiet fixture must be awake")
    };
    assert!(before > 0);
    quiet.set_gravity(quiet.config.gravity).unwrap();
    assert!(matches!(quiet.bodies[0].motion,Motion::Awake{quiet_ticks} if quiet_ticks==before));
}
#[test]
fn fixed32_gravity_change_wakes_sleepers() {
    gravity_wakes::<Fixed<i32, 16>>();
}
#[test]
fn fixed64_gravity_change_wakes_sleepers() {
    gravity_wakes::<Fixed<i64, 32>>();
}
#[cfg(any(feature = "std", feature = "libm", feature = "system-libm"))]
#[test]
fn float_gravity_change_wakes_sleepers() {
    gravity_wakes::<f32>();
}

/// A quiet side-wall contact cannot support gravity parallel to that wall.
fn wall_is_not_support<T: RealScalar + TryCastFrom<f64>>() {
    let mut w = World::<T>::new();
    w.set_gravity(Vector3::from_array([0., -0.01, 0.]).map(T::cast_from))
        .unwrap();
    w.add_mesh(StaticMesh {
        tree: AabbTree::new(
            vec![
                Tri3::new([
                    Vector3::from_array([0., -2., -2.]).map(T::cast_from),
                    Vector3::from_array([0., 2., -2.]).map(T::cast_from),
                    Vector3::from_array([0., 2., 2.]).map(T::cast_from),
                ]),
                Tri3::new([
                    Vector3::from_array([0., -2., -2.]).map(T::cast_from),
                    Vector3::from_array([0., 2., 2.]).map(T::cast_from),
                    Vector3::from_array([0., -2., 2.]).map(T::cast_from),
                ]),
            ],
            4,
        )
        .unwrap(),
        material: material(),
    })
    .unwrap();
    let id = w
        .add_body(
            Body::new(
                Shape::Sphere {
                    radius: T::cast_from(0.4),
                },
                Pose::at(Vector3::from_array([0.4, 0., 0.]).map(T::cast_from)),
                T::one(),
                material(),
            )
            .unwrap(),
        )
        .unwrap();
    for _ in 0..180 {
        w.step();
    }
    assert!(!w.body(id).unwrap().sleeping());
    assert!(w.body(id).unwrap().pose.position.y < T::zero());
}
#[test]
fn fixed32_side_wall_is_not_sleep_support() {
    wall_is_not_support::<Fixed<i32, 16>>();
}
#[cfg(any(feature = "std", feature = "libm", feature = "system-libm"))]
#[test]
fn float_side_wall_is_not_sleep_support() {
    wall_is_not_support::<f32>();
}

#[cfg(any(feature = "std", feature = "libm", feature = "system-libm"))]
#[test]
fn rejected_gravity_preserves_sleep_and_force() {
    let mut w = World::<f32>::new();
    floor(&mut w);
    let id = w
        .add_body(
            Body::new(
                Shape::Sphere { radius: 0.4 },
                Pose::at(Vector3::from_array([0., 0.4, 0.]).map(f32::cast_from)),
                1.,
                material(),
            )
            .unwrap(),
        )
        .unwrap();
    for _ in 0..480 {
        w.step();
    }
    assert!(w.body(id).unwrap().sleeping());
    let gravity = w.config.gravity;
    for invalid in [f32::NAN, f32::INFINITY, f32::NEG_INFINITY] {
        assert_eq!(
            w.set_gravity(Vector3::new(invalid, 0., 0.)),
            Err(Error::InvalidPose)
        );
        assert!(w.body(id).unwrap().sleeping());
    }
    assert_eq!(
        [w.config.gravity.x, w.config.gravity.y, w.config.gravity.z],
        [gravity.x, gravity.y, gravity.z]
    );
    assert!(w.body(id).unwrap().sleeping());
}
