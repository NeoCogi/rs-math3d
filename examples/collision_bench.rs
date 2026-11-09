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

//! Target-local unchecked collision and scalar timings.
//! Run `cargo run --release --example collision_bench -- 200 5`.
//! Arguments are batches per sample and sample count; each batch has 128 pairs.

mod collision;

use collision::{Capsule, Obb};
use rs_math3d::{Fixed, RealScalar, Sphere3, Tri3, TryCastFrom, Vector, Vector3};
use std::{hint::black_box, time::Instant};

/// Number of varied pairs in each batch. Even entries hit, odd entries miss.
const PAIRS: usize = 128;

/// One instance of each primitive, colocated for controlled overlap workloads.
#[derive(Clone, Copy)]
struct Shapes<T: RealScalar> {
    /// A half-unit sphere.
    sphere: Sphere3<T>,
    /// A short vertical capsule.
    capsule: Capsule<T>,
    /// A box with an orientation prepared outside the timer.
    obb: Obb<T>,
    /// A horizontal or vertical triangle through the primitive cluster.
    triangle: Tri3<T>,
}

/// Builds a varied primitive cluster. Boxes rotate around two axes; triangles
/// use two intersecting planes so hits exercise a spatial triangle-triangle test.
fn shapes<T: RealScalar + TryCastFrom<f64>>(i: usize, second: bool) -> Shapes<T> {
    let n = i as f64;
    let shift = if second && i % 2 == 1 { 4.0 } else { 0.0 };
    let offset = if second { 0.125 } else { 0.0 };
    let center = Vector3::new(
        <T>::cast_from((n % 7.0 - 3.0) / 16.0 + offset + if i % 3 == 0 { shift } else { 0.0 }),
        <T>::cast_from(
            (n % 11.0 - 5.0) / 32.0 - offset * 0.5 + if i % 3 == 1 { shift } else { 0.0 },
        ),
        <T>::cast_from(
            (n % 13.0 - 6.0) / 32.0 + offset * 0.5 + if i % 3 == 2 { shift } else { 0.0 },
        ),
    );
    let angle = (n % 17.0 - 8.0) / 16.0 + if second { 0.25 } else { 0.0 };
    let (s, c) = angle.sin_cos();
    let (sy, cy) = (angle * 0.5).sin_cos();
    Shapes {
        sphere: Sphere3::new(center, T::cast_from(0.5)),
        capsule: Capsule {
            a: center
                + if second {
                    Vector3::from_array([-0.5, -0.125, 0.0]).map(T::cast_from)
                } else {
                    Vector3::from_array([-0.125, -0.5, 0.0]).map(T::cast_from)
                },
            b: center
                + if second {
                    Vector3::from_array([0.5, 0.125, 0.125]).map(T::cast_from)
                } else {
                    Vector3::from_array([0.125, 0.5, 0.125]).map(T::cast_from)
                },
            radius: T::cast_from(0.375),
        },
        obb: Obb {
            center,
            axes: [
                Vector3::from_array([c * cy, s * cy, -sy]).map(T::cast_from),
                Vector3::from_array([-s, c, 0.0]).map(T::cast_from),
                Vector3::from_array([c * sy, s * sy, cy]).map(T::cast_from),
            ],
            half: Vector3::from_array([0.625, 0.5, 0.375]).map(T::cast_from),
        },
        triangle: if second {
            Tri3::new([
                center + Vector3::from_array([-0.5, 0.0, -0.75]).map(T::cast_from),
                center + Vector3::from_array([0.5, 0.0, -0.75]).map(T::cast_from),
                center + Vector3::from_array([0.0, 0.0, 0.75]).map(T::cast_from),
            ])
        } else {
            Tri3::new([
                center + Vector3::from_array([-0.75, -0.5, 0.0]).map(T::cast_from),
                center + Vector3::from_array([0.75, -0.5, 0.0]).map(T::cast_from),
                center + Vector3::from_array([0.0, 0.75, 0.0]).map(T::cast_from),
            ])
        },
    }
}

/// Reports the median sample and range, with setup and validation excluded.
/// The function is monomorphized, so the timed loop has no dynamic dispatch.
fn measure<I, O>(
    name: &str,
    input: &[I],
    batches: u32,
    samples: usize,
    operation: impl Fn(&I) -> O,
) {
    for value in input {
        black_box(operation(black_box(value)));
    }
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
        "{name:23} {:10.2} [{:.2}, {:.2}]",
        timings[samples / 2],
        timings[0],
        timings[samples - 1]
    );
}

/// Runs all pair kernels on one scalar backend. Fixtures are independently
/// checked against the expected hit/miss pattern before any measurement.
fn collisions<T: RealScalar + TryCastFrom<f64>>(label: &str, batches: u32, samples: usize) {
    let input: Vec<_> = (0..PAIRS)
        .map(|i| (shapes::<T>(i, false), shapes::<T>(i, true)))
        .collect();
    println!("\n{label}: collision ns/query, median [min, max]; 64 hits + 64 misses/batch");
    macro_rules! pair {
        ($query:ident, $a:ident, $b:ident) => {{
            let operation = |(a, b): &(Shapes<T>, Shapes<T>)| collision::$query(&a.$a, &b.$b);
            for (i, fixture) in input.iter().enumerate() {
                assert_eq!(
                    operation(fixture),
                    i % 2 == 0,
                    "{} {} pair {}",
                    label,
                    stringify!($query),
                    i
                );
            }
            measure(stringify!($query), &input, batches, samples, operation);
        }};
    }
    pair!(sphere_sphere, sphere, sphere);
    pair!(sphere_capsule, sphere, capsule);
    pair!(sphere_box, sphere, obb);
    pair!(sphere_triangle, sphere, triangle);
    pair!(capsule_capsule, capsule, capsule);
    pair!(capsule_box, capsule, obb);
    pair!(capsule_triangle, capsule, triangle);
    pair!(box_box, obb, obb);
    pair!(box_triangle, obb, triangle);
    pair!(triangle_triangle, triangle, triangle);
}

/// Separately times arithmetic and analytics. Inputs vary and are black-boxed;
/// conversions and fixture creation are outside the measured region. sin+cos
/// intentionally uses the common trait's two calls, not fixed's fused sin_cos.
fn scalars<T: RealScalar + TryCastFrom<f64>>(label: &str, batches: u32, samples: usize) {
    let input: Vec<_> = (0..PAIRS)
        .map(|i| {
            (
                <T>::cast_from(0.25 + i as f64 / 256.0),
                <T>::cast_from(1.0 + i as f64 / 256.0),
            )
        })
        .collect();
    let vectors: Vec<_> = input
        .iter()
        .map(|&(a, b)| (Vector3::new(a, b, a), Vector3::new(b, a, b)))
        .collect();
    println!("\n{label}: scalar ns/op, median [min, max]");
    measure("add", &input, batches, samples, |&(a, b)| a + b);
    measure("multiply", &input, batches, samples, |&(a, b)| a * b);
    measure("divide", &input, batches, samples, |&(a, b)| a / b);
    measure("dot3", &vectors, batches, samples, |&(a, b)| {
        Vector3::dot(&a, &b)
    });
    measure("sqrt", &input, batches, samples, |&(a, _)| a.tsqrt());
    measure(
        "sin + cos (two calls)",
        &input,
        batches,
        samples,
        |&(a, _)| (a.tsin(), a.tcos()),
    );
}

/// Measures the fused fixed analytic API separately from the common trait.
fn fixed_sin_cos<R: rs_math3d::fixed::RealFormat<F>, const F: u32>(
    label: &str,
    batches: u32,
    samples: usize,
) {
    let input: Vec<_> = (0..PAIRS)
        .map(|i| Fixed::<R, F>::try_from_ratio(64 + i as i64, 256).unwrap())
        .collect();
    println!("\n{label}: fused analytic ns/op, median [min, max]");
    measure("sin_cos (one call)", &input, batches, samples, |&a| {
        a.sin_cos()
    });
}

/// Parses a positive bounded workload parameter, rejecting extra arguments.
fn argument(args: &mut impl Iterator<Item = String>, default: u32) -> u32 {
    let value = args
        .next()
        .map(|s| s.parse::<u32>().expect("positive integer argument"))
        .unwrap_or(default);
    assert!(value > 0, "arguments must be positive");
    value
}

/// Benchmarks floating baselines and three fixed formats without dependencies.
fn main() {
    let mut args = std::env::args().skip(1);
    let batches = argument(&mut args, 200);
    let samples = argument(&mut args, 5) as usize;
    assert!(
        args.next().is_none(),
        "usage: collision_bench [batches] [samples]"
    );
    println!("Unchecked discrete overlap; {PAIRS} pairs x {batches} batches x {samples} samples");
    println!(
        "Target {}-{}; run with --release; timings include loop/black_box overhead.",
        std::env::consts::ARCH,
        std::env::consts::OS
    );
    macro_rules! backend {
        ($t:ty, $label:literal) => {{
            collisions::<$t>($label, batches, samples);
            scalars::<$t>($label, batches, samples);
        }};
    }
    backend!(f32, "f32");
    backend!(f64, "f64");
    backend!(Fixed<i32, 8>, "S23.8 (i32)");
    backend!(Fixed<i32, 16>, "S15.16 (i32)");
    backend!(Fixed<i64, 16>, "S47.16 (i64)");
    fixed_sin_cos::<i32, 8>("S23.8 (i32)", batches, samples);
    fixed_sin_cos::<i32, 16>("S15.16 (i32)", batches, samples);
    fixed_sin_cos::<i64, 16>("S47.16 (i64)", batches, samples);
}
