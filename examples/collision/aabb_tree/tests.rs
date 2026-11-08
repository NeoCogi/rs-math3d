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
