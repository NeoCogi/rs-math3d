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

//! Numerical probes for the native fixed arithmetic contract.
//! References use exact bounded integer numerators;
//! floating error conversion happens only after subtracting the integer part.

#[allow(dead_code)]
#[path = "../../../examples/collision/mod.rs"]
mod collision;
#[path = "../../../examples/collision/fixtures.rs"]
mod fixtures;
#[path = "../../../examples/collision/shape_cases.rs"]
mod shape_cases;

use collision::aabb_tree::AabbTree;
use rs_math3d::{
    fixed::RealFormat, CrossProduct, Fixed, Ray, RealScalar, Tri3, TryCastFrom, Vector, Vector3,
};

/// Deterministic random bits, with identical seeds across policy processes.
struct Random(u64);
impl Random {
    /// Xorshift generator; all arithmetic is explicitly unsigned and wrapping.
    fn next(&mut self) -> u64 {
        self.0 ^= self.0 << 13;
        self.0 ^= self.0 >> 7;
        self.0 ^= self.0 << 17;
        self.0
    }
}

/// Errors in output quanta, including signed bias and worst absolute error.
#[derive(Default)]
struct Error {
    /// Number of independently compared outputs.
    count: u64,
    /// Sum of signed errors.
    sum: f64,
    /// Sum of absolute errors.
    absolute: f64,
    /// Sum of squared errors.
    squares: f64,
    /// Largest absolute error.
    maximum: f64,
}
impl Error {
    /// Incorporates one signed error measured in raw output units.
    fn add(&mut self, error: f64) {
        self.count += 1;
        self.sum += error;
        self.absolute += error.abs();
        self.squares += error * error;
        self.maximum = self.maximum.max(error.abs());
    }
    /// Compares against an exact rational without first rounding its large
    /// integer part through f64. All probe quotients fit the destination scalar.
    fn rational(&mut self, observed: i128, numerator: i128, denominator: i128) {
        let q = numerator / denominator;
        let r = numerator % denominator;
        self.add((observed - q) as f64 - r as f64 / denominator as f64);
    }
    /// Emits count, bias, mean absolute, RMS and maximum absolute error.
    fn print(&self, backend: &str, operation: &str) {
        let count = self.count as f64;
        println!(
            "error,{backend},{operation},{},{:.12e},{:.12e},{:.12e},{:.12e}",
            self.count,
            self.sum / count,
            self.absolute / count,
            (self.squares / count).sqrt(),
            self.maximum
        );
    }
}

/// Numeric setup is shared by every policy; conversions retain nearest-even.
fn n<T: TryCastFrom<f64>>(x: f64) -> T {
    T::try_cast_from(x).ok().expect("bounded probe input")
}
/// Constructs a vector outside any measured operation.
fn v<T: RealScalar + TryCastFrom<f64>>(a: [f64; 3]) -> Vector3<T> {
    Vector3::new(n(a[0]), n(a[1]), n(a[2]))
}

/// Samples full fractional resolution in [-4,4), keeping every multiplication,
/// dot/cross and division (|denominator| >= 0.5) well within the output range.
fn arithmetic<R: RealFormat<F>, const F: u32>(backend: &str) {
    let scale = 1i128 << F;
    let mut rng = Random(0x1234_5678_9abc_def0);
    let mut errors: [Error; 5] = core::array::from_fn(|_| Error::default());
    let mut antisymmetry = Error::default();
    for _ in 0..100_000 {
        let raw: [i128; 6] =
            core::array::from_fn(|_| (rng.next() as i128 & (8 * scale - 1)) - 4 * scale);
        let values = raw.map(|x| Fixed::<R, F>::from_bits(R::wrapping_narrow(x)));
        let a = Vector3::new(values[0], values[1], values[2]);
        let b = Vector3::new(values[3], values[4], values[5]);
        errors[0].rational(
            (values[0] * values[3]).to_bits().widen(),
            raw[0] * raw[3],
            scale,
        );
        if raw[3].abs() >= scale / 2 {
            errors[1].rational(
                (values[0] / values[3]).to_bits().widen(),
                raw[0] * scale,
                raw[3],
            );
        }
        errors[2].rational(
            Vector3::dot(&a, &b).to_bits().widen(),
            raw[0] * raw[3] + raw[1] * raw[4] + raw[2] * raw[5],
            scale,
        );
        let cross = Vector3::cross(&a, &b);
        let reverse = Vector3::cross(&b, &a);
        for (i, (j, k)) in [(1, 2), (2, 0), (0, 1)].iter().copied().enumerate() {
            let component = [cross.x, cross.y, cross.z][i];
            let opposite = [reverse.x, reverse.y, reverse.z][i];
            errors[3].rational(
                component.to_bits().widen(),
                raw[j] * raw[k + 3] - raw[k] * raw[j + 3],
                scale,
            );
            antisymmetry.add((component.to_bits().widen() + opposite.to_bits().widen()) as f64);
        }
        // Isolate negative products to expose the difference between arithmetic
        // floor shifts and truncation toward zero, independent of sample balance.
        if raw[0] * raw[3] < 0 {
            errors[4].rational(
                (values[0] * values[3]).to_bits().widen(),
                raw[0] * raw[3],
                scale,
            );
        }
    }
    for (error, label) in
        errors
            .iter()
            .zip(["multiply", "divide", "dot", "cross", "negative_multiply"])
    {
        error.print(backend, label);
    }
    antisymmetry.print(backend, "cross_antisymmetry");

    // A controlled integration illustration, not a rigid-body simulation.
    // Both policies receive the same represented dt and velocity.
    let dt = Fixed::<R, F>::try_from_ratio(1, 120).unwrap();
    let velocity = Fixed::<R, F>::try_from_ratio(1, 1000).unwrap();
    let mut positions = [Fixed::<R, F>::ZERO; 2];
    for _ in 0..10_000 {
        positions[0] += velocity * dt;
        positions[1] += (-velocity) * dt;
    }
    let exact = 10_000.0 * velocity.to_bits().widen() as f64 * dt.to_bits().widen() as f64
        / (scale * scale) as f64;
    println!(
        "drift,{backend},{:.12e},{:.12e},{:.12e}",
        positions[0].to_bits().widen() as f64 / scale as f64,
        positions[1].to_bits().widen() as f64 / scale as f64,
        exact
    );
}

/// Tests exact plane-distance contacts at +/- eight scalar quanta around three
/// radii. Known truth is k<=0 for both signs, independently of the kernels.
fn boundaries<R: RealFormat<F>, const F: u32>(backend: &str)
where
    Fixed<R, F>: TryCastFrom<f64>,
{
    let tri = Tri3::new([v([0.0, 0.0, 0.0]), v([4.0, 0.0, 0.0]), v([0.0, 4.0, 0.0])]);
    let q = 1.0 / (1u64 << F) as f64;
    for radius in [0.125, 0.375, 1.0] {
        for sign in [-1.0, 1.0] {
            for k in -8..=8 {
                let z = sign * (radius + k as f64 * q);
                let capsule = collision::Capsule::<Fixed<R, F>> {
                    a: v([0.5, 0.5, z]),
                    b: v([1.0, 0.5, z]),
                    radius: n(radius),
                };
                let obb = collision::Obb {
                    center: v([0.75, 0.75, z]),
                    axes: [v([1.0, 0.0, 0.0]), v([0.0, 1.0, 0.0]), v([0.0, 0.0, 1.0])],
                    half: v([0.25, 0.25, radius]),
                };
                for (shape, hit) in [
                    ("capsule", collision::capsule_triangle(&capsule, &tri)),
                    ("obb", collision::box_triangle(&obb, &tri)),
                ] {
                    println!(
                        "boundary,{backend},{shape},{radius},{sign},{k},{},{}",
                        k <= 0,
                        hit
                    );
                }
            }
        }
    }
}

/// The six orthographic and one oblique grids used by mesh_bench: 448 common
/// dyadic rays. Retain their original direction instead of normalizing it.
fn rays<T: RealScalar + TryCastFrom<f64>>() -> Vec<Ray<T, Vector3<T>>> {
    let mut specs = Vec::new();
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
                    specs.push((origin, direction));
                }
            }
        }
    }
    for i in 0..8 {
        for j in 0..8 {
            specs.push((
                [-18.375 + i as f64 * 5.25, -18.375 + j as f64 * 5.25, 32.0],
                [0.25, -0.125, -1.0],
            ));
        }
    }
    specs
        .into_iter()
        .map(|(origin, direction)| {
            let mut ray = Ray::new(&v(origin), &v([0.0, 0.0, 1.0]));
            ray.direction = v(direction);
            ray
        })
        .collect()
}

/// Emits individual outcomes for cross-policy comparisons, preserving misses
/// and original triangle witnesses. f64 is a floating reference, not exact truth.
fn geometry<T: RealScalar + TryCastFrom<f64>>(backend: &str)
where
    f64: TryCastFrom<T>,
{
    for (model, bytes) in [("teapot", fixtures::TEAPOT), ("bunny", fixtures::BUNNY)] {
        let tree = AabbTree::new(fixtures::decode::<T>(bytes).unwrap(), 4).unwrap();
        for (i, ray) in rays().iter().enumerate() {
            let actual = tree.raycast(ray).hit;
            let flat = tree.brute_raycast(ray).hit;
            assert!(
                actual.map(|h| (h.triangle, h.parameter))
                    == flat.map(|h| (h.triangle, h.parameter))
            );
            if let Some(hit) = actual {
                let t = f64::try_cast_from(hit.parameter).ok().unwrap();
                println!("query,{backend},{model},ray,{i},{},{t:.17e}", hit.triangle);
            } else {
                println!("query,{backend},{model},ray,{i},miss,");
            }
        }
        let cases = shape_cases::model_cases(&tree);
        for i in 0..cases.capsules.len() {
            let cap = tree.intersects_capsule(&cases.capsules[i]).hit;
            assert_eq!(
                cap.is_some(),
                tree.brute_intersects_capsule(&cases.capsules[i])
                    .hit
                    .is_some()
            );
            let obb = tree.intersects_obb(&cases.obbs[i]).hit;
            assert_eq!(
                obb.is_some(),
                tree.brute_intersects_obb(&cases.obbs[i]).hit.is_some()
            );
            for (shape, hit) in [("capsule", cap), ("obb", obb)] {
                let witness = hit
                    .map(|id| id.to_string())
                    .unwrap_or_else(|| "miss".to_owned());
                println!("query,{backend},{model},{shape},{i},{witness},");
            }
        }
    }
}

/// Isolates one tree-query loop for hardware counters. Process counters include
/// a small amortized setup/verification cost; the printed elapsed time excludes it.
fn profile<T: RealScalar + TryCastFrom<f64>>(shape: &str, batches: usize)
where
    f64: TryCastFrom<T>,
{
    let tree = AabbTree::new(fixtures::decode::<T>(fixtures::TEAPOT).unwrap(), 4).unwrap();
    let cases = shape_cases::model_cases(&tree);
    macro_rules! run {
        ($inputs:expr,$method:ident,$flat:ident) => {{
            let inputs = $inputs;
            for q in inputs {
                assert_eq!(tree.$method(q).hit.is_some(), tree.$flat(q).hit.is_some());
            }
            let start = std::time::Instant::now();
            for _ in 0..batches {
                for q in std::hint::black_box(inputs) {
                    std::hint::black_box(tree.$method(std::hint::black_box(q)));
                }
            }
            eprintln!(
                "queries={} ns/query={:.3}",
                batches * inputs.len(),
                start.elapsed().as_secs_f64() * 1e9 / (batches * inputs.len()) as f64
            );
        }};
    }
    match shape {
        "capsule" => run!(
            &cases.capsules,
            intersects_capsule,
            brute_intersects_capsule
        ),
        "obb" => run!(&cases.obbs, intersects_obb, brute_intersects_obb),
        _ => panic!("capsule or obb"),
    }
}

/// Runs untimed precision probes, or a selected loop for perf. Median timings
/// use the existing mesh_bench so its warm-up, batching and counters are retained.
fn main() {
    let args: Vec<_> = std::env::args().skip(1).collect();
    if !args.is_empty() {
        assert!(
            args.len() == 4 && args[0] == "profile",
            "profile scalar shape batches"
        );
        let batches = args[3].parse().unwrap();
        match args[1].as_str() {
            "f32" => profile::<f32>(&args[2], batches),
            "s15" => profile::<Fixed<i32, 16>>(&args[2], batches),
            "s31" => profile::<Fixed<i64, 32>>(&args[2], batches),
            _ => panic!("f32, s15 or s31"),
        }
        return;
    }
    arithmetic::<i32, 16>("s15");
    arithmetic::<i64, 32>("s31");
    boundaries::<i32, 16>("s15");
    boundaries::<i64, 32>("s31");
    geometry::<f64>("f64");
    geometry::<f32>("f32");
    geometry::<Fixed<i32, 16>>("s15");
    geometry::<Fixed<i64, 32>>("s31");
}

#[cfg(test)]
mod tests {
    use super::*;
    use rs_math3d::fixed::Format;

    /// Independent signed quotient/remainder reference; the native shift mode
    /// uses Euclidean division rather than the implementation's right shift.
    fn quotient(n: i128, d: i128, shift: bool) -> i128 {
        if shift {
            n.div_euclid(d)
        } else {
            n / d
        }
    }

    /// Exhausts every i8 operand pair, including wrapping destinations, for one
    /// fractional width. Zero division is covered separately by a panic test.
    fn exhaustive<const F: u32>()
    where
        i8: Format<F>,
    {
        let scale = 1i128 << F;
        for a in i8::MIN..=i8::MAX {
            for b in i8::MIN..=i8::MAX {
                let left = Fixed::<i8, F>::from_bits(a);
                let right = Fixed::<i8, F>::from_bits(b);
                assert_eq!(
                    (left * right).to_bits(),
                    quotient(a as i128 * b as i128, scale, true) as i8
                );
                if b != 0 {
                    assert_eq!(
                        (left / right).to_bits(),
                        quotient(a as i128 * scale, b as i128, false) as i8
                    );
                }
            }
        }
    }
    #[test]
    fn every_i8_format_and_operand_pair() {
        exhaustive::<0>();
        exhaustive::<1>();
        exhaustive::<2>();
        exhaustive::<3>();
        exhaustive::<4>();
        exhaustive::<5>();
        exhaustive::<6>();
        exhaustive::<7>();
    }

    /// Native floor shifting need not preserve cross-product antisymmetry;
    /// compare each component against exact widened integer differences instead.
    #[test]
    fn fused_dot_and_cross_match_integer_references() {
        let mut rng = Random(0x1234_5678_9abc_def0);
        for _ in 0..10000 {
            let raw: [i32; 6] = core::array::from_fn(|_| (rng.next() % 1000000) as i32 - 500000);
            let x = raw.map(Fixed::<i32, 16>::from_bits);
            let a = Vector3::new(x[0], x[1], x[2]);
            let b = Vector3::new(x[3], x[4], x[5]);
            let raw = raw.map(|x| x as i128);
            assert_eq!(
                Vector3::dot(&a, &b).to_bits(),
                quotient(
                    raw[0] * raw[3] + raw[1] * raw[4] + raw[2] * raw[5],
                    65536,
                    true
                ) as i32
            );
            let cross = Vector3::cross(&a, &b);
            for (observed, (j, k)) in
                [cross.x, cross.y, cross.z]
                    .iter()
                    .zip([(1, 2), (2, 0), (0, 1)])
            {
                assert_eq!(
                    observed.to_bits(),
                    quotient(raw[j] * raw[k + 3] - raw[k] * raw[j + 3], 65536, true) as i32
                );
            }
        }
    }

    /// Ordinary integer-based fixed division retains a native zero-divisor panic.
    #[test]
    #[should_panic]
    fn zero_divisor_panics() {
        let _ = std::hint::black_box(Fixed::<i32, 16>::ONE)
            / std::hint::black_box(Fixed::<i32, 16>::ZERO);
    }
}
