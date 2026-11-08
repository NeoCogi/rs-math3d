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

//! Structural, analytic and real-model acceleration regressions.

use super::*;
use core::convert::TryInto;
use rs_math3d::{Fixed, TryCastFrom};
#[path = "../fixtures.rs"]
mod fixtures;
#[path = "../shape_cases.rs"]
mod shape_cases;

/// Exact fixture conversion outside the algorithms under test.
fn n<T: TryCastFrom<f64>>(x: f64) -> T {
    T::try_cast_from(x).ok().unwrap()
}
/// Dyadic coordinates shared by every numeric backend.
fn p<T: RealScalar + TryCastFrom<f64>>(x: f64, y: f64, z: f64) -> Vector3<T> {
    Vector3::new(n(x), n(y), n(z))
}
/// Triangle with exactly representable edges and normal.
fn triangle<T: RealScalar + TryCastFrom<f64>>(z: f64) -> Tri3<T> {
    Tri3::new([p(-1.0, -1.0, z), p(1.0, -1.0, z), p(0.0, 1.0, z)])
}

/// Independently verifies leaf coverage, stable IDs, parent containment and
/// root-first topology on both coincident and spatially varied real geometry.
fn structure<T: RealScalar>(tree: &AabbTree<T>, leaf_size: usize) {
    let mut ids = tree.order.clone();
    ids.sort_unstable();
    assert_eq!(ids, (0..tree.entries.len()).collect::<Vec<_>>());
    let mut covered = vec![false; tree.order.len()];
    for (index, node) in tree.nodes.iter().enumerate() {
        let contained = |bound: &Box3<T>| {
            bound.min.x >= node.bounds.min.x
                && bound.max.x <= node.bounds.max.x
                && bound.min.y >= node.bounds.min.y
                && bound.max.y <= node.bounds.max.y
                && bound.min.z >= node.bounds.min.z
                && bound.max.z <= node.bounds.max.z
        };
        match node.kind {
            Kind::Leaf { start } => {
                assert!(node.count > 0 && node.count <= leaf_size);
                for slot in start..start + node.count {
                    assert!(!covered[slot]);
                    covered[slot] = true;
                    let entry = &tree.entries[tree.order[slot]];
                    assert!(contained(&entry.bounds));
                    for p in entry.triangle.vertices() {
                        assert!(p.x >= node.bounds.min.x && p.x <= node.bounds.max.x);
                        assert!(p.y >= node.bounds.min.y && p.y <= node.bounds.max.y);
                        assert!(p.z >= node.bounds.min.z && p.z <= node.bounds.max.z);
                    }
                }
            }
            Kind::Branch { left, right } => {
                assert!(left > index && right > left);
                assert_eq!(node.count, tree.nodes[left].count + tree.nodes[right].count);
                assert!(contained(&tree.nodes[left].bounds));
                assert!(contained(&tree.nodes[right].bounds));
            }
        }
    }
    assert!(covered.iter().all(|&x| x));
    if !tree.nodes.is_empty() {
        assert_eq!(tree.nodes[0].count, tree.entries.len());
    }
}

/// Tests exact geometry, nearest ordering, boundary inclusion, duplicate-ID
/// ties, empty input, containment semantics and tree enclosure/permutation.
fn analytic<T: RealScalar + TryCastFrom<f64> + core::fmt::Debug>() {
    let empty = AabbTree::<T>::new(Vec::new(), 4).unwrap();
    let ray = Ray::new(&p(0.0, 0.0, 3.0), &p(0.0, 0.0, -1.0));
    assert_eq!(empty.info().nodes, 0);
    assert_eq!(empty.info().depth, 0);
    assert!(empty.raycast(&ray).hit.is_none());
    assert!(matches!(
        AabbTree::<T>::new(Vec::new(), 0),
        Err(BuildError::InvalidLeafSize)
    ));
    let tree = AabbTree::new(vec![triangle::<T>(0.0), triangle(1.0)], 1).unwrap();
    let hit = tree.raycast(&ray).hit.unwrap();
    assert_eq!(hit.triangle, 1);
    assert_eq!(hit.parameter, n(2.0));
    assert_eq!(hit.point.z, n(1.0));
    let mut scaled = Ray::new(&p(0.0, 0.0, 3.0), &p(0.0, 0.0, -1.0));
    scaled.direction = p(0.0, 0.0, -2.0);
    let hit = tree.raycast(&scaled).hit.unwrap();
    assert_eq!(hit.parameter, n(1.0));
    assert_eq!(hit.point.z, n(1.0));
    for (origin, direction, expected) in [
        (p(0.0, 0.0, -3.0), p(0.0, 0.0, 1.0), true),
        (p(-1.0, -1.0, 3.0), p(0.0, 0.0, -1.0), true),
        (p(0.0, 0.0, 1.0), p(0.0, 0.0, 1.0), true),
        (p(0.0, 0.0, -3.0), p(0.0, 0.0, -1.0), false),
        (p(3.0, 0.0, 3.0), p(0.0, 0.0, -1.0), false),
        (p(0.0, 0.0, 0.0), p(1.0, 0.0, 0.0), false),
    ] {
        let ray = Ray::new(&origin, &direction);
        let result = tree.raycast(&ray);
        let reference = tree.brute_raycast(&ray);
        assert_eq!(result.hit.is_some(), expected);
        assert_eq!(
            result.hit.map(|h| (h.triangle, h.parameter)),
            reference.hit.map(|h| (h.triangle, h.parameter))
        );
    }
    let duplicates = AabbTree::new(vec![triangle::<T>(0.0); 64], 1).unwrap();
    assert_eq!(duplicates.info().depth, 7);
    assert_eq!(duplicates.info().nodes, 127);
    assert_eq!(duplicates.raycast(&ray).hit.unwrap().triangle, 0);
    structure(&duplicates, 1);
    let single = AabbTree::new(vec![triangle::<T>(0.0)], 1).unwrap();
    for (offset, expected) in [
        (p(0.0, 0.0, 0.0), true),
        (p(2.0, 0.0, 0.0), true),
        (p(2.25, 0.0, 0.0), false),
        (p(0.0, 0.0, 0.25), false),
    ] {
        assert_eq!(single.intersects(&single, offset).hit.is_some(), expected);
        assert_eq!(
            single.brute_intersects(&single, offset).hit.is_some(),
            expected
        );
    }
    assert!(empty.intersects(&single, p(0.0, 0.0, 0.0)).hit.is_none());
    assert!(single.intersects(&empty, p(0.0, 0.0, 0.0)).hit.is_none());
    let a = p::<T>(0.0, 0.0, 0.0);
    let b = p(4.0, 0.0, 0.0);
    let c = p(0.0, 4.0, 0.0);
    let d = p(0.0, 0.0, 4.0);
    let outer = AabbTree::new(
        vec![
            Tri3::new([a, b, c]),
            Tri3::new([a, b, d]),
            Tri3::new([a, c, d]),
            Tri3::new([b, c, d]),
        ],
        1,
    )
    .unwrap();
    let inner = AabbTree::new(
        vec![Tri3::new([
            p(0.5, 0.5, 0.5),
            p(0.75, 0.5, 0.5),
            p(0.5, 0.75, 0.5),
        ])],
        1,
    )
    .unwrap();
    assert!(outer.intersects(&inner, p(0.0, 0.0, 0.0)).hit.is_none());
    let capsule = Capsule {
        a: p(0.5, 0.5, 0.5),
        b: p(0.5, 0.5, 0.5),
        radius: n(0.125),
    };
    assert!(outer.intersects_capsule(&capsule).hit.is_none());
    let obb = Obb {
        center: p(0.5, 0.5, 0.5),
        axes: [p(1.0, 0.0, 0.0), p(0.0, 1.0, 0.0), p(0.0, 0.0, 1.0)],
        half: p(0.125, 0.125, 0.125),
    };
    assert!(outer.intersects_obb(&obb).hit.is_none());
    let bad = Tri3::new([p::<T>(0.0, 0.0, 0.0), p(1.0, 1.0, 1.0), p(2.0, 2.0, 2.0)]);
    assert!(matches!(
        AabbTree::new(vec![bad], 4),
        Err(BuildError::InvalidTriangle {
            index: 0,
            error: MathError::Degenerate
        })
    ));
}

/// Checks both real soups using identical represented vertices and ray grids,
/// plus intersecting and separated mesh translations and original-ID witnesses.
fn real_meshes<T: RealScalar + TryCastFrom<f64> + core::fmt::Debug>() {
    let bunny = AabbTree::<T>::new(fixtures::decode(fixtures::BUNNY).unwrap(), 4).unwrap();
    let teapot = AabbTree::<T>::new(fixtures::decode(fixtures::TEAPOT).unwrap(), 4).unwrap();
    assert_eq!(bunny.info().triangles, 3851);
    assert_eq!(teapot.info().triangles, 6320);
    for mesh in [&bunny, &teapot] {
        structure(mesh, 4);
        let (mut hits, mut misses) = (0, 0);
        for axis in 0..3 {
            for i in -3..=3 {
                for j in -3..=3 {
                    let mut origin = [i as f64 * 6.0, j as f64 * 6.0, 32.0];
                    origin.swap(axis, 2);
                    let mut direction = [0.0, 0.0, -1.0];
                    direction.swap(axis, 2);
                    let ray = Ray::new(
                        &p(origin[0], origin[1], origin[2]),
                        &p(direction[0], direction[1], direction[2]),
                    );
                    let actual = mesh.raycast(&ray);
                    let expected = mesh.brute_raycast(&ray);
                    assert_eq!(
                        actual.hit.map(|h| (h.triangle, h.parameter)),
                        expected.hit.map(|h| (h.triangle, h.parameter))
                    );
                    if actual.hit.is_some() {
                        hits += 1;
                    } else {
                        misses += 1;
                    }
                }
            }
        }
        assert!(hits > 0 && misses > 0);
        assert!(mesh.info().depth < 20);
    }
    let offset = p(0.0, 0.0, 0.0);
    let actual = teapot.intersects(&bunny, offset);
    let expected = teapot.brute_intersects(&bunny, offset);
    assert_eq!(actual.hit.is_some(), expected.hit.is_some());
    assert!(actual.hit.is_some());
    let (a, b) = actual.hit.unwrap();
    assert!(triangle_triangle(
        teapot.triangle(a).unwrap(),
        bunny.triangle(b).unwrap()
    ));
    assert!(teapot.intersects(&bunny, p(64.0, 0.0, 0.0)).hit.is_none());
}

/// Known shape/triangle contacts, misses, degenerate centerlines, tilted boxes
/// and envelope false positives. Each witness refers to original input geometry.
fn shape_contacts<T: RealScalar + TryCastFrom<f64> + core::fmt::Debug>() {
    let tree = AabbTree::new(vec![triangle::<T>(0.0)], 1).unwrap();
    let empty = AabbTree::<T>::new(Vec::new(), 4).unwrap();
    for (z, expected) in [(0.0, true), (0.25, true), (0.5, false)] {
        let capsule = Capsule {
            a: p(-0.5, 0.0, z),
            b: p(0.5, 0.0, z),
            radius: n(0.25),
        };
        assert_eq!(
            tree.intersects_capsule(&capsule).hit,
            if expected { Some(0) } else { None }
        );
        assert_eq!(
            tree.brute_intersects_capsule(&capsule).hit,
            tree.intersects_capsule(&capsule).hit
        );
        assert!(empty.intersects_capsule(&capsule).hit.is_none());
        let obb = Obb {
            center: p(0.0, 0.0, z),
            axes: [p(1.0, 0.0, 0.0), p(0.0, 1.0, 0.0), p(0.0, 0.0, 1.0)],
            half: p(0.5, 0.25, 0.25),
        };
        assert_eq!(
            tree.intersects_obb(&obb).hit,
            if expected { Some(0) } else { None }
        );
        assert_eq!(
            tree.brute_intersects_obb(&obb).hit,
            tree.intersects_obb(&obb).hit
        );
        assert!(empty.intersects_obb(&obb).hit.is_none());
    }
    let piercing = Capsule {
        a: p(0.0, 0.0, -1.0),
        b: p(0.0, 0.0, 1.0),
        radius: n(0.0),
    };
    assert_eq!(tree.intersects_capsule(&piercing).hit, Some(0));
    let sphere = Capsule {
        a: p(-1.0, -1.0, 0.25),
        b: p(-1.0, -1.0, 0.25),
        radius: n(0.25),
    };
    assert_eq!(tree.intersects_capsule(&sphere).hit, Some(0));
    let corner = Capsule {
        a: p(1.25, 1.25, 0.0),
        b: p(1.25, 1.25, 0.0),
        radius: n(0.25),
    };
    assert!(overlap(&capsule_bounds(&corner), &tree.nodes[0].bounds));
    assert!(tree.intersects_capsule(&corner).hit.is_none());
    let rotated = Obb {
        center: p(1.0, 1.0, 0.0),
        axes: [p(0.6, -0.8, 0.0), p(0.8, 0.6, 0.0), p(0.0, 0.0, 1.0)],
        half: p(1.0, 0.125, 0.125),
    };
    assert!(overlap(&obb_bounds(&rotated), &tree.nodes[0].bounds));
    assert!(tree.intersects_obb(&rotated).hit.is_none());
    let tilted = Obb {
        center: p(0.0, 0.0, 0.0),
        axes: [p(1.0, 0.0, 0.0), p(0.0, 0.6, 0.8), p(0.0, -0.8, 0.6)],
        half: p(0.5, 0.25, 0.25),
    };
    assert_eq!(tree.intersects_obb(&tilted).hit, Some(0));
    let contained_triangle = Obb {
        half: p(2.0, 2.0, 2.0),
        ..tilted
    };
    assert_eq!(tree.intersects_obb(&contained_triangle).hit, Some(0));
    let zero_size = Obb {
        axes: [p(1.0, 0.0, 0.0), p(0.0, 1.0, 0.0), p(0.0, 0.0, 1.0)],
        half: p(0.0, 0.0, 0.0),
        ..tilted
    };
    assert_eq!(tree.intersects_obb(&zero_size).hit, Some(0));
    assert!(tree
        .intersects_obb(&Obb {
            center: p(0.75, 0.75, 0.0),
            ..zero_size
        })
        .hit
        .is_none());
    let envelope = obb_bounds(&tilted);
    for sx in [-T::one(), T::one()] {
        for sy in [-T::one(), T::one()] {
            for sz in [-T::one(), T::one()] {
                let point = tilted.center
                    + tilted.axes[0] * (tilted.half.x * sx)
                    + tilted.axes[1] * (tilted.half.y * sy)
                    + tilted.axes[2] * (tilted.half.z * sz);
                assert!(point.x >= envelope.min.x && point.x <= envelope.max.x);
                assert!(point.y >= envelope.min.y && point.y <= envelope.max.y);
                assert!(point.z >= envelope.min.z && point.z <= envelope.max.z);
            }
        }
    }
}

/// A soup with one-cell Q8 triangles must support capsule distances without
/// dividing by a Gram determinant erased by fixed-point product quantization.
#[test]
fn capsule_handles_tiny_triangle_projection() {
    type F = Fixed<i32, 16>;
    let tree = AabbTree::new(
        vec![Tri3::new([
            p::<F>(0.0, 0.0, 0.0),
            p(1.0 / 256.0, 0.0, 0.0),
            p(0.0, 1.0 / 256.0, 0.0),
        ])],
        1,
    )
    .unwrap();
    let capsule = Capsule {
        a: p(1.0 / 512.0, 1.0 / 256.0, 0.0),
        b: p(1.0 / 512.0, 1.0 / 256.0, 0.0),
        radius: n(0.125),
    };
    assert_eq!(tree.intersects_capsule(&capsule).hit, Some(0));
}

/// Summing half extents before adding a negative center can round a positive
/// corner outside the envelope. Interval accumulation must contain that corner.
#[cfg(any(feature = "std", feature = "libm", feature = "system-libm"))]
#[test]
fn obb_envelope_contains_rounded_corner() {
    let obb = Obb::<f32> {
        center: p(-0.75, 0.0, 0.0),
        axes: [p(0.6, 0.8, 0.0), p(-0.8, 0.6, 0.0), p(0.0, 0.0, 1.0)],
        half: p(1.0, 0.5, 0.25),
    };
    let corner =
        obb.center + obb.axes[0] * obb.half.x - obb.axes[1] * obb.half.y + obb.axes[2] * obb.half.z;
    assert!(corner.x > 0.25);
    assert!(corner.x <= obb_bounds(&obb).max.x);
}

/// Both real fixtures use the exact same query corpus as the benchmark. Verify
/// acceleration, witness validity and a mix of hits/misses in each scalar.
fn model_shapes<T: RealScalar + TryCastFrom<f64> + core::fmt::Debug>()
where
    f64: TryCastFrom<T>,
{
    for data in [fixtures::TEAPOT, fixtures::BUNNY] {
        let tree = AabbTree::<T>::new(fixtures::decode(data).unwrap(), 4).unwrap();
        let cases = shape_cases::model_cases(&tree);
        let (mut caps, mut boxes) = (0, 0);
        for (i, capsule) in cases.capsules.iter().enumerate() {
            let actual = tree.intersects_capsule(capsule);
            let expected = tree.brute_intersects_capsule(capsule);
            assert_eq!(
                actual.hit.is_some(),
                expected.hit.is_some(),
                "capsule pose {}",
                i
            );
            if let Some(id) = actual.hit {
                caps += 1;
                assert!(capsule_triangle(capsule, tree.triangle(id).unwrap()));
            }
        }
        for (i, obb) in cases.obbs.iter().enumerate() {
            let actual = tree.intersects_obb(obb);
            let expected = tree.brute_intersects_obb(obb);
            assert_eq!(
                actual.hit.is_some(),
                expected.hit.is_some(),
                "OBB pose {}",
                i
            );
            if let Some(id) = actual.hit {
                boxes += 1;
                assert!(box_triangle(obb, tree.triangle(id).unwrap()));
            }
        }
        assert!(caps > 0 && caps < cases.capsules.len());
        assert!(boxes > 0 && boxes < cases.obbs.len());
    }
}

/// Shape suites retain fixed-only coverage; f32 runs when its backend is enabled.
macro_rules! shapes {
    ($name:ident,$t:ty) => {
        #[test]
        fn $name() {
            shape_contacts::<$t>();
            model_shapes::<$t>();
        }
    };
}
#[cfg(any(feature = "std", feature = "libm", feature = "system-libm"))]
shapes!(f32_capsule_obb_tree, f32);
shapes!(s15_16_capsule_obb_tree,Fixed<i32,16>);
shapes!(s31_32_capsule_obb_tree,Fixed<i64,32>);

/// The same analytic and real-mesh regressions run in fixed-only builds.
macro_rules! backend {
    ($name:ident,$scalar:ty) => {
        #[test]
        fn $name() {
            analytic::<$scalar>();
            real_meshes::<$scalar>();
        }
    };
}
#[cfg(any(feature = "std", feature = "libm", feature = "system-libm"))]
backend!(f32_tree_and_meshes, f32);
backend!(s15_16_tree_and_meshes,Fixed<i32,16>);
backend!(s31_32_tree_and_meshes,Fixed<i64,32>);

/// Checked construction rejects numeric failures before building a hierarchy.
#[test]
fn build_rejects_unrepresentable_span_and_cross_product() {
    type F = Fixed<i32, 16>;
    let remote = |x| Tri3::new([p::<F>(x, 0.0, 0.0), p(x + 1.0, 0.0, 0.0), p(x, 1.0, 0.0)]);
    assert!(matches!(
        AabbTree::new(vec![remote(-30000.0), remote(30000.0)], 4),
        Err(BuildError::InvalidSpan(MathError::Overflow))
    ));
    let huge = Tri3::new([
        p::<F>(0.0, 0.0, 0.0),
        p(200.0, 0.0, 0.0),
        p(0.0, 200.0, 0.0),
    ]);
    assert!(matches!(
        AabbTree::new(vec![huge], 4),
        Err(BuildError::InvalidTriangle {
            error: MathError::Overflow,
            ..
        })
    ));
}

/// Malformed packets fail before unbounded allocation or unchecked indexing.
#[test]
fn fixture_decoder_rejects_bad_lengths_and_indices() {
    type F = Fixed<i32, 16>;
    assert!(fixtures::decode::<F>(b"bad").is_err());
    let mut data = fixtures::BUNNY.to_vec();
    data.push(0);
    assert!(matches!(
        fixtures::decode::<F>(&data),
        Err(fixtures::DecodeError::InvalidPacket)
    ));
    let mut data = fixtures::BUNNY.to_vec();
    let vertices = u32::from_le_bytes(data[8..12].try_into().unwrap()) as usize;
    data[16 + vertices * 6..18 + vertices * 6].copy_from_slice(&u16::MAX.to_le_bytes());
    assert!(matches!(
        fixtures::decode::<F>(&data),
        Err(fixtures::DecodeError::InvalidIndex)
    ));
    let mut data = fixtures::BUNNY.to_vec();
    data[8..12].copy_from_slice(&u32::MAX.to_le_bytes());
    assert!(matches!(
        fixtures::decode::<F>(&data),
        Err(fixtures::DecodeError::InvalidPacket)
    ));
    assert!(matches!(
        fixtures::decode::<Fixed<i8, 4>>(fixtures::BUNNY),
        Err(fixtures::DecodeError::ScalarRange)
    ));
    let mut empty = b"M3SOUP01".to_vec();
    empty.extend_from_slice(&[0; 8]);
    assert!(fixtures::decode::<F>(&empty).unwrap().is_empty());
}

#[cfg(any(feature = "std", feature = "libm", feature = "system-libm"))]
#[test]
fn nonfinite_geometry_is_rejected() {
    let bad = Tri3::new([
        p::<f32>(0.0, 0.0, 0.0),
        p(1.0, 0.0, 0.0),
        Vector3::new(0.0, f32::NAN, 0.0),
    ]);
    assert!(matches!(
        AabbTree::new(vec![bad], 4),
        Err(BuildError::InvalidTriangle {
            error: MathError::NonFinite,
            ..
        })
    ));
}
