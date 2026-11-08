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

//! Static triangle-soup AABB tree performance and acceleration correctness.
//! Run `cargo run --release --example mesh_bench -- 20 5`.

#[allow(dead_code)] // This workload selects the tree and triangle-pair kernels.
mod collision;
#[path = "collision/fixtures.rs"]
mod fixtures;

use collision::aabb_tree::{AabbTree, QueryStats, RayHit};
use rs_math3d::{Fixed, Ray, RealScalar, Tri3, TryCastFrom, Vector3};
use std::{fmt::Debug, hint::black_box, time::Instant};

/// One exact dyadic input shared by f32 and both fixed backends.
struct RaySpec {
    /// World-space start; no source mesh conversion is timed.
    origin: [f64; 3],
    /// Stored direction, deliberately retaining its common parameterization.
    direction: [f64; 3],
}

/// Workload and f32 reference outcomes, all prepared outside timed loops.
struct Reference {
    /// Ray starts and directions for both meshes.
    rays: Vec<RaySpec>,
    /// Nearest f32 hit for each model and ray.
    hits: [Vec<Option<RayHit<f32>>>; 2],
    /// Relative translations of bunny into the teapot coordinate frame.
    offsets: Vec<[f64; 3]>,
    /// f32 surface-overlap classification at each translation.
    pairs: Vec<bool>,
}

/// Exact setup conversion for this bounded fixture grid.
fn n<T: TryCastFrom<f64>>(x: f64) -> T {
    T::try_cast_from(x).ok().unwrap()
}
/// Typed point/vector conversion outside timers.
fn point<T: RealScalar + TryCastFrom<f64>>(a: [f64; 3]) -> Vector3<T> {
    Vector3::new(n(a[0]), n(a[1]), n(a[2]))
}
/// Builds a ray with the original, common stored direction. Public Ray fields
/// support this parameterization, and both box/triangle solvers retain it.
fn ray<T: RealScalar + TryCastFrom<f64>>(spec: &RaySpec) -> Ray<T, Vector3<T>> {
    let mut ray = Ray::new(&point(spec.origin), &point([0.0, 0.0, 1.0]));
    ray.direction = point(spec.direction);
    ray
}

/// Six orthographic grids plus one oblique grid provide hits, misses, both
/// directions, zero components and a non-unit dyadic direction: 448 rays/model.
fn ray_specs() -> Vec<RaySpec> {
    let mut rays = Vec::new();
    for axis in 0..3 {
        for sign in [-1.0, 1.0] {
            for i in 0..8 {
                for j in 0..8 {
                    let mut origin = [
                        -18.375 + i as f64 * 5.25,
                        -18.375 + j as f64 * 5.25,
                        32.0 * sign,
                    ];
                    origin.swap(axis, 2);
                    let mut direction = [0.0, 0.0, -sign];
                    direction.swap(axis, 2);
                    rays.push(RaySpec { origin, direction });
                }
            }
        }
    }
    for i in 0..8 {
        for j in 0..8 {
            rays.push(RaySpec {
                origin: [-18.375 + i as f64 * 5.25, -18.375 + j as f64 * 5.25, 32.0],
                direction: [0.25, -0.125, -1.0],
            });
        }
    }
    rays
}

/// Prepares a scalar-independent workload and untimed f32 comparison baseline.
fn reference() -> Reference {
    let rays = ray_specs();
    let meshes: [AabbTree<f32>; 2] = [
        AabbTree::new(fixtures::decode(fixtures::TEAPOT).unwrap(), 4).unwrap(),
        AabbTree::new(fixtures::decode(fixtures::BUNNY).unwrap(), 4).unwrap(),
    ];
    let hits = core::array::from_fn(|model| {
        rays.iter()
            .map(|r| meshes[model].raycast(&ray(r)).hit)
            .collect()
    });
    let offsets = vec![
        [0.0, 0.0, 0.0],
        [6.0, 0.0, 0.0],
        [12.0, 0.0, 0.0],
        [18.0, 0.0, 0.0],
        [24.0, 0.0, 0.0],
        [28.0, 0.0, 0.0],
        [0.0, 8.0, 0.0],
        [0.0, 16.0, 0.0],
        [0.0, 24.0, 0.0],
        [0.0, 28.0, 0.0],
        [0.0, 0.0, 20.0],
        [48.0, 0.0, 0.0],
    ];
    let pairs = offsets
        .iter()
        .map(|&o| meshes[0].intersects(&meshes[1], point(o)).hit.is_some())
        .collect();
    Reference {
        rays,
        hits,
        offsets,
        pairs,
    }
}

/// Median/min/max throughput; queries include work counters, loop and black-box
/// overhead. Correctness passes warm both traversals before measurement.
fn measure<I, O>(
    label: &str,
    input: &[I],
    batches: u32,
    samples: usize,
    operation: impl Fn(&I) -> O,
) {
    let mut timings = Vec::with_capacity(samples);
    for _ in 0..samples {
        let start = Instant::now();
        for _ in 0..batches {
            for value in black_box(input) {
                black_box(operation(black_box(value)));
            }
        }
        timings.push(start.elapsed().as_secs_f64() * 1e9 / (batches as f64 * input.len() as f64));
    }
    timings.sort_by(f64::total_cmp);
    println!(
        "{label:25} {:12.2} [{:.2}, {:.2}] ns/query",
        timings[samples / 2],
        timings[0],
        timings[samples - 1]
    );
}

/// Constructor timing includes validation/partition/allocation; cloning the
/// canonical triangle inputs and dropping the completed tree are excluded.
fn build_time<T: RealScalar>(triangles: &[Tri3<T>], samples: usize) {
    let mut timings = Vec::new();
    for _ in 0..samples {
        let input = triangles.to_vec();
        let start = Instant::now();
        let tree = AabbTree::new(input, 4).unwrap();
        timings.push(start.elapsed().as_secs_f64() * 1e3);
        black_box(&tree);
    }
    timings.sort_by(f64::total_cmp);
    println!(
        "build median={:.3} [{:.3}, {:.3}] ms",
        timings[samples / 2],
        timings[0],
        timings[samples - 1]
    );
}

/// Aggregates traversal work separately from the timing loop.
fn accumulate(sum: &mut QueryStats, x: QueryStats) {
    sum.bounds_tests += x.bounds_tests;
    sum.triangle_tests += x.triangle_tests;
}

/// Validates acceleration against flat traversal, then reports hit/ID/parameter
/// differences from f32 without conflating quantization with a hierarchy defect.
fn backend<T: RealScalar + TryCastFrom<f64> + Debug>(
    label: &str,
    reference: &Reference,
    batches: u32,
    samples: usize,
) where
    f64: TryCastFrom<T>,
{
    println!("\n{label}");
    let triangles: [Vec<Tri3<T>>; 2] = [
        fixtures::decode(fixtures::TEAPOT).unwrap(),
        fixtures::decode(fixtures::BUNNY).unwrap(),
    ];
    let trees: [AabbTree<T>; 2] =
        core::array::from_fn(|i| AabbTree::new(triangles[i].clone(), 4).unwrap());
    let rays: Vec<_> = reference.rays.iter().map(ray::<T>).collect();
    for model in 0..2 {
        let tree = &trees[model];
        let info = tree.info();
        println!(
            "{}: triangles={} nodes={} depth={} reserved={} bytes",
            ["Utah teapot", "Stanford bunny"][model],
            info.triangles,
            info.nodes,
            info.depth,
            info.reserved_bytes
        );
        build_time(&triangles[model], samples);
        let (mut hits, mut classification, mut ids, mut max_parameter) = (0, 0, 0, 0.0f64);
        let (mut tree_work, mut flat_work) = (QueryStats::default(), QueryStats::default());
        for (i, ray) in rays.iter().enumerate() {
            let actual = tree.raycast(ray);
            let expected = tree.brute_raycast(ray);
            assert_eq!(
                actual.hit.map(|h| (h.triangle, h.parameter)),
                expected.hit.map(|h| (h.triangle, h.parameter)),
                "{label} ray {i}"
            );
            if actual.hit.is_some() {
                hits += 1;
            }
            let baseline = reference.hits[model][i];
            classification += (actual.hit.is_some() != baseline.is_some()) as usize;
            if let (Some(a), Some(b)) = (actual.hit, baseline) {
                ids += (a.triangle != b.triangle) as usize;
                let t = f64::try_cast_from(a.parameter).ok().unwrap();
                max_parameter = max_parameter.max((t - b.parameter as f64).abs());
            }
            accumulate(&mut tree_work, actual.stats);
            accumulate(&mut flat_work, expected.stats);
        }
        assert!(hits > 0 && hits < rays.len());
        println!("rays={} hits={} acceleration_mismatches=0 vs_f32: hit_changes={} id_changes={} max_parameter_delta={:.6e}",rays.len(),hits,classification,ids,max_parameter);
        println!(
            "mean work/ray: tree bounds={:.1} triangles={:.2}; flat bounds={:.1} triangles={:.2}",
            tree_work.bounds_tests as f64 / rays.len() as f64,
            tree_work.triangle_tests as f64 / rays.len() as f64,
            flat_work.bounds_tests as f64 / rays.len() as f64,
            flat_work.triangle_tests as f64 / rays.len() as f64
        );
        measure("tree nearest ray", &rays, batches, samples, |r| {
            tree.raycast(r)
        });
        measure("flat nearest ray", &rays, 1, samples, |r| {
            tree.brute_raycast(r)
        });
    }
    let offsets: Vec<_> = reference.offsets.iter().copied().map(point::<T>).collect();
    let (mut hits, mut changes, mut root_rejections) = (0, 0, 0);
    let (mut tree_work, mut flat_work) = (QueryStats::default(), QueryStats::default());
    for (i, &offset) in offsets.iter().enumerate() {
        let actual = trees[0].intersects(&trees[1], offset);
        let expected = trees[0].brute_intersects(&trees[1], offset);
        assert_eq!(
            actual.hit.is_some(),
            expected.hit.is_some(),
            "{label} mesh pair {i}"
        );
        hits += actual.hit.is_some() as usize;
        root_rejections += (actual.hit.is_none() && actual.stats.bounds_tests == 1) as usize;
        changes += (actual.hit.is_some() != reference.pairs[i]) as usize;
        accumulate(&mut tree_work, actual.stats);
        accumulate(&mut flat_work, expected.stats);
    }
    assert!(hits > 0 && hits < offsets.len());
    println!(
        "teapot/bunny poses={} hits={} root_rejections={} acceleration_mismatches=0 vs_f32_hit_changes={}",
        offsets.len(),
        hits,
        root_rejections,
        changes
    );
    println!(
        "mean work/pose: tree bounds={:.1} triangles={:.1}; flat bounds={:.1} triangles={:.1}",
        tree_work.bounds_tests as f64 / offsets.len() as f64,
        tree_work.triangle_tests as f64 / offsets.len() as f64,
        flat_work.bounds_tests as f64 / offsets.len() as f64,
        flat_work.triangle_tests as f64 / offsets.len() as f64
    );
    measure("tree mesh/mesh", &offsets, batches, samples, |&o| {
        trees[0].intersects(&trees[1], o)
    });
    // Exhaustive misses can test all 24 million pairs. One complete flat sample
    // keeps this reference timing bounded; its result is reported as one sample.
    measure("flat mesh/mesh (1 sample)", &offsets, 1, 1, |&o| {
        trees[0].brute_intersects(&trees[1], o)
    });
}

/// Positive bounded workload parameters, defaulting to twenty batches/five samples.
fn argument(args: &mut impl Iterator<Item = String>, default: u32) -> u32 {
    let value = args
        .next()
        .map(|s| s.parse::<u32>().expect("positive integer argument"))
        .unwrap_or(default);
    assert!(value > 0, "arguments must be positive");
    value
}

/// Runs static-build, nearest-ray and mesh/mesh workloads on the same geometry.
fn main() {
    let mut args = std::env::args().skip(1);
    let batches = argument(&mut args, 20);
    let samples = argument(&mut args, 5) as usize;
    assert!(
        args.next().is_none(),
        "usage: mesh_bench [batches] [samples]"
    );
    println!(
        "Static triangle-soup AABB tree; leaf_size=4; {batches} tree batches x {samples} samples"
    );
    println!("Flat ray timings use 1 batch/sample; flat mesh timing uses 1 batch/1 sample. Both traversals use a soup AABB prefilter; counters are included.");
    let reference = reference();
    backend::<f32>("f32", &reference, batches, samples);
    backend::<Fixed<i32, 16>>("S15.16 (i32)", &reference, batches, samples);
    backend::<Fixed<i64, 32>>("S31.32 (i64)", &reference, batches, samples);
}
