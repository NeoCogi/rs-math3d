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

//! Compare end-to-end const-selected CORDIC speed and Decimal-reference error.
//! Run `cargo run --release --example cordic_bench -- 1000 7`.

use rs_math3d::{fixed::RealFormat, Fixed, MathError};
use std::{hint::black_box, time::Instant};

/// One operation named in the generated independent reference fixture.
#[derive(Clone, Copy, PartialEq, Eq)]
enum Operation {
    /// First component of fused sine/cosine.
    Sin,
    /// Second component of fused sine/cosine.
    Cos,
    /// Guarded tangent quotient, sensitive to poles.
    Tan,
    /// Arc cosine, with endpoint-preserving root construction.
    Acos,
}

impl Operation {
    /// CSV spelling shared by the generator and report.
    fn name(self) -> &'static str {
        match self {
            Self::Sin => "sin",
            Self::Cos => "cos",
            Self::Tan => "tan",
            Self::Acos => "acos",
        }
    }
}

/// Reference results split into integer and residual parts to retain precision
/// at output magnitudes larger than f64 can represent one raw bit at a time.
struct Reference {
    /// Operation being evaluated.
    operation: Operation,
    /// Exactly represented scalar input, never converted through floating point.
    raw: i128,
    /// Nearest integer to the exact result measured in destination quanta.
    nearest: i128,
    /// Exact result minus nearest, independently computed with Decimal.
    residual: f64,
}

/// Per-operation precision statistics, kept separate for each working width.
#[derive(Default)]
struct Errors {
    /// Successfully evaluated reference results that fit the destination.
    count: usize,
    /// Maximum absolute error in destination quanta (not machine float ULPs).
    max: f64,
    /// Input raw bits attaining the reported maximum error.
    worst_raw: i128,
    /// Sum of squared errors, used to report RMS.
    squared: f64,
    /// Count with absolute error greater than one destination quantum.
    over_one: usize,
    /// In-range references rejected by a checked calculation.
    rejected: usize,
    /// Out-of-range reference results accepted by a checked calculation.
    accepted_outside: usize,
    /// Quantized results or error statuses differing from the Q96 baseline.
    different: usize,
}

/// Evaluates one operation with a compile-time working-width choice.
fn evaluate<R: RealFormat<F>, const F: u32, const W: u32>(
    input: Fixed<R, F>,
    operation: Operation,
) -> Result<i128, MathError> {
    match operation {
        Operation::Sin => input
            .checked_sin_cos_with::<W>()
            .map(|v| v.0.to_bits().widen()),
        Operation::Cos => input
            .checked_sin_cos_with::<W>()
            .map(|v| v.1.to_bits().widen()),
        Operation::Tan => input.checked_tan_with::<W>().map(|v| v.to_bits().widen()),
        Operation::Acos => input.checked_acos_with::<W>().map(|v| v.to_bits().widen()),
    }
}

/// Loads exactly the requested storage/scale distribution before reporting.
fn references<R: RealFormat<F>, const F: u32>() -> Vec<Reference> {
    include_str!("../src/fixed/cordic_reference.csv")
        .lines()
        .filter(|s| !s.starts_with('#'))
        .filter_map(|line| {
            let cells: Vec<_> = line.split(',').collect();
            if cells[0].parse::<u32>().unwrap() != R::BITS || cells[1].parse::<u32>().unwrap() != F
            {
                return None;
            }
            Some(Reference {
                operation: match cells[2] {
                    "sin" => Operation::Sin,
                    "cos" => Operation::Cos,
                    "tan" => Operation::Tan,
                    "acos" => Operation::Acos,
                    _ => panic!("reference operation"),
                },
                raw: cells[3].parse().unwrap(),
                nearest: cells[4].parse().unwrap(),
                residual: cells[5].parse().unwrap(),
            })
        })
        .collect()
}

/// Compares against the independent reference, including rejected valid results
/// and accepted overflows. Neither backend is used as the precision oracle.
fn precision<R: RealFormat<F>, const F: u32, const W: u32>(input: &[Reference]) {
    for operation in [
        Operation::Sin,
        Operation::Cos,
        Operation::Acos,
        Operation::Tan,
    ] {
        let mut error = Errors::default();
        for reference in input.iter().filter(|r| r.operation == operation) {
            let value = Fixed::<R, F>::from_bits(R::narrow(reference.raw).unwrap());
            let actual = evaluate::<R, F, W>(value, operation);
            let baseline = evaluate::<R, F, 128>(value, operation);
            error.different += (actual != baseline) as usize;
            let representable = reference.nearest >= R::MIN && reference.nearest <= R::MAX;
            match (representable, actual) {
                (true, Ok(raw)) => {
                    let difference = (raw - reference.nearest) as f64 - reference.residual;
                    error.count += 1;
                    if difference.abs() > error.max {
                        error.max = difference.abs();
                        error.worst_raw = reference.raw;
                    }
                    error.squared += difference * difference;
                    error.over_one += (difference.abs() > 1.0) as usize;
                }
                (true, Err(_)) => error.rejected += 1,
                (false, Ok(_)) => error.accepted_outside += 1,
                (false, Err(_)) => {}
            }
        }
        assert!(error.count > 0);
        println!("{} i{W:3}: n={:4} max={:.6e} RMS={:.6e} >1q={} rejected={} accepted_overflow={} different_Q96={} worst_input_raw={}",
            operation.name(), error.count, error.max, (error.squared / error.count as f64).sqrt(),
            error.over_one, error.rejected, error.accepted_outside, error.different, error.worst_raw);
    }
}

/// Reports median/min/max batch throughput with setup excluded. Black-boxed
/// inputs and results prevent constant folding and dead result elimination.
fn measure<I, O>(
    label: &str,
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
        "{label:24} {:9.2} [{:.2}, {:.2}] ns/call",
        timings[samples / 2],
        timings[0],
        timings[samples - 1]
    );
}

/// End-to-end ordinary API timings: bounded angles, full storage-range angles,
/// and unit-domain acos. All conversions, input generation and reporting are
/// excluded. Tangent timings stay away from poles, while precision tests do not.
fn performance<R: RealFormat<F>, const F: u32, const W: u32>(batches: u32, samples: usize) {
    let small: Vec<_> = (0..256)
        .map(|i| Fixed::<R, F>::try_from_ratio(i - 128, 128).unwrap())
        .collect();
    let unit: Vec<_> = (0..256)
        .map(|i| Fixed::<R, F>::try_from_ratio(i - 128, 128).unwrap())
        .collect();
    let mut state = 0x123456789abcdefu64;
    let full: Vec<_> = (0..256)
        .map(|_| {
            state = state
                .wrapping_mul(6364136223846793005)
                .wrapping_add(1442695040888963407);
            Fixed::<R, F>::from_bits(R::wrapping_narrow(state as i64 as i128))
        })
        .collect();
    measure(
        &format!("i{W} sin_cos bounded"),
        &small,
        batches,
        samples,
        |&x| x.sin_cos_with::<W>(),
    );
    measure(
        &format!("i{W} sin_cos full range"),
        &full,
        batches,
        samples,
        |&x| x.sin_cos_with::<W>(),
    );
    measure(&format!("i{W} acos"), &unit, batches, samples, |&x| {
        x.acos_with::<W>()
    });
    measure(
        &format!("i{W} tan bounded"),
        &small,
        batches,
        samples,
        |&x| x.tan_with::<W>(),
    );
}

/// Reports one storage/scale experiment, comparing both working widths against
/// the same timing inputs and the same independent precision fixture.
fn backend<R: RealFormat<F>, const F: u32>(batches: u32, samples: usize) {
    println!(
        "\nStorage i{}, F={F}; working fraction={} (i64), 96 (i128)",
        R::BITS,
        (F + 16).clamp(32, 60)
    );
    performance::<R, F, 128>(batches, samples);
    performance::<R, F, 64>(batches, samples);
    let input = references::<R, F>();
    println!("Precision in destination quanta; independent Decimal references:");
    precision::<R, F, 128>(&input);
    precision::<R, F, 64>(&input);
}

/// Parses positive workload parameters; seven samples is the default.
fn argument(args: &mut impl Iterator<Item = String>, default: u32) -> u32 {
    let value = args
        .next()
        .map(|s| s.parse::<u32>().expect("positive integer argument"))
        .unwrap_or(default);
    assert!(value > 0, "arguments must be positive");
    value
}

/// Runs lower-precision physics formats and the supported i64 accuracy boundary.
fn main() {
    let mut args = std::env::args().skip(1);
    let batches = argument(&mut args, 1000);
    let samples = argument(&mut args, 7) as usize;
    assert!(
        args.next().is_none(),
        "usage: cordic_bench [batches] [samples]"
    );
    println!("Const-selected CORDIC; 256 inputs x {batches} batches x {samples} samples");
    println!("Median [min, max]; includes loop/black_box overhead. Reduction/root/division retain wide arithmetic.");
    backend::<i32, 8>(batches, samples);
    backend::<i32, 16>(batches, samples);
    backend::<i32, 29>(batches, samples);
    backend::<i64, 16>(batches, samples);
    backend::<i64, 32>(batches, samples);
    backend::<i64, 44>(batches, samples);
    backend::<i64, 48>(batches, samples);
}
