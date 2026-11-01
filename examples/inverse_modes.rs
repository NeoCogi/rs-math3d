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

//! Small target-local timing harness for the two matrix inverse algorithms.
//! Run optimized: `cargo run --release --example inverse_modes -- 100000`.

use rs_math3d::{Fixed, InverseMode, Matrix4, RealScalar};
use std::{hint::black_box, time::Instant};

/// Times full inversion while preventing the optimizer from folding the input
/// matrix or discarding the result. This is a comparative harness, not a promise
/// about a particular machine, compiler, scalar type or matrix distribution.
fn measure<T: RealScalar>(name: &str, matrix: Matrix4<T>, mode: InverseMode, iterations: u32) {
    let start = Instant::now();
    for _ in 0..iterations {
        black_box(black_box(matrix).inverse_with(black_box(mode)));
    }
    println!(
        "{} {:?}: {:.1} ns/inverse",
        name,
        mode,
        start.elapsed().as_nanos() as f64 / iterations as f64
    );
}

/// Runs one dense, well-conditioned fixture for floating and fixed scalars.
fn main() {
    let iterations = std::env::args()
        .nth(1)
        .map(|value| value.parse::<u32>().expect("positive iteration count"))
        .unwrap_or(100000);
    assert!(iterations > 0, "positive iteration count");
    let matrix = Matrix4::new(
        4.0f32, 1.0, 0.5, 0.25, 1.0, 3.0, 1.0, 0.5, 0.5, 1.0, 2.0, 0.5, 0.25, 0.5, 0.5, 1.0,
    );
    let fixed = matrix
        .try_cast::<Fixed<i32, 16>>()
        .expect("fixture fits fixed scalar");
    measure("f32", matrix, InverseMode::Reciprocal, iterations);
    measure("S15.16", fixed, InverseMode::Reciprocal, iterations);
    measure("f32", matrix, InverseMode::Rows, iterations);
    measure("S15.16", fixed, InverseMode::Rows, iterations);
}
