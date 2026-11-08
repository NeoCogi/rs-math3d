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

//! Independent high-precision constants/reference generation using GMP on the
//! development host. Taylor/Machin/half-angle algorithms do not use CORDIC or
//! the crate's fixed arithmetic. --check performs read-only reproduction.

#[path = "../high_precision.rs"]
mod high_precision;
use high_precision::Float;
use rs_math3d_tools::root;
use std::{collections::BTreeSet, fs, io, path::PathBuf};

/// Precision context and independently computed pi; all operands share its bits.
struct Reference {
    /// Working mantissa precision shared by every independent reference value.
    bits: u32,
    /// Machin-formula pi at the selected working precision.
    pi: Float,
}
impl Reference {
    /// Converts an exact bounded integer at this context's precision.
    fn n(&self, n: i128) -> Float {
        Float::int(n, self.bits)
    }
    /// Alternating atan series at |x|<=1/2, stopped at reference convergence.
    fn atan_series(x: &Float, bits: u32) -> Float {
        let mut sum = x.clone();
        let mut term = x.clone();
        let minus_square = -&(x * x);
        for i in 1..10000 {
            term = &term * &minus_square;
            let candidate = &sum + &(&term / &Float::int(2 * i + 1, bits));
            if candidate == sum {
                return sum;
            }
            sum = candidate;
        }
        panic!("atan reference did not converge")
    }
    /// Machin's identity computes pi independently of platform float constants.
    fn new(bits: u32) -> Self {
        let n = |x| Float::int(x, bits);
        let a = Self::atan_series(&(&n(1) / &n(5)), bits);
        let b = Self::atan_series(&(&n(1) / &n(239)), bits);
        Self {
            bits,
            pi: &(&n(16) * &a) - &(&n(4) * &b),
        }
    }
    /// Independent Taylor sin/cos after high-precision full-range reduction.
    fn sin_cos(&self, x: &Float) -> (Float, Float) {
        let tau = self.pi.scale(1);
        let turns = (x / &tau).trunc_i128();
        let mut x = x - &(&self.n(turns) * &tau);
        if x > self.pi {
            x = &x - &tau;
        }
        if x < -&self.pi {
            x = &x + &tau;
        }
        let minus_square = -&(&x * &x);
        let (mut sine, mut cosine) = (x.clone(), self.n(1));
        let (mut st, mut ct) = (x, self.n(1));
        for i in 1..10000 {
            st = &(&st * &minus_square) / &self.n((2 * i) * (2 * i + 1));
            ct = &(&ct * &minus_square) / &self.n((2 * i - 1) * (2 * i));
            let s = &sine + &st;
            let c = &cosine + &ct;
            if s == sine && c == cosine {
                return (s, c);
            }
            sine = s;
            cosine = c;
        }
        panic!("trigonometric reference did not converge")
    }
    /// Reciprocal/half-angle reduction keeps the atan series away from one.
    fn atan_positive(&self, x: &Float) -> Float {
        if x > &self.n(1) {
            return &self.pi.scale(-1) - &self.atan_positive(&(&self.n(1) / x));
        }
        let mut x = x.clone();
        let mut doublings = 0;
        while x > self.n(1).scale(-2) {
            x = &x / &(&self.n(1) + &(&self.n(1) + &(&x * &x)).sqrt());
            doublings += 1;
        }
        Self::atan_series(&x, self.bits).scale(doublings)
    }
    /// acos via sqrt and atan, independent of the CORDIC implementation.
    fn acos(&self, x: &Float) -> Float {
        if x == &self.n(1) {
            return self.n(0);
        }
        if x == &self.n(-1) {
            return self.pi.clone();
        }
        if x == &self.n(0) {
            return self.pi.scale(-1);
        }
        let a = self.atan_positive(&(&(&self.n(1) - &(x * x)).sqrt() / &x.abs()));
        if x < &self.n(0) {
            &self.pi - &a
        } else {
            a
        }
    }
    /// CORDIC gain uses independent high-precision square roots.
    fn gain(&self, steps: i32) -> Float {
        let mut gain = self.n(1);
        for i in 0..steps {
            gain = &gain / &(&self.n(1) + &self.n(1).scale(-2 * i)).sqrt();
        }
        gain
    }
    /// Appends exact integer plus residual, avoiding large f64 reference values.
    fn record(&self, out: &mut String, bits: u32, frac: u32, op: &str, raw: i128, value: &Float) {
        let scaled = value.scale(frac as i32);
        let nearest = scaled.nearest_i128();
        let residual = &scaled - &self.n(nearest);
        out.push_str(&format!(
            "{bits},{frac},{op},{raw},{nearest},{}\n",
            residual.scientific()
        ));
    }
}

/// Produces deterministic constant Rust source and two independent CSV corpora.
fn generate(reference: &Reference, license: &str) -> [(String, String); 3] {
    let quant = |x: &Float, f: i32| x.scale(f).nearest_i128();
    let mut angles = vec![reference.pi.scale(-2)];
    for i in 1..97 {
        angles.push(Reference::atan_series(
            &reference.n(1).scale(-i),
            reference.bits,
        ));
    }
    let mut constants=format!("{license}\n\n//! Generated by `tools/src/bin/generate_fixed_constants.rs`; do not edit by hand.\n//! Constants use Q96 unless their name explicitly specifies another scale.\n\n/// Pi with 96 fractional bits.\npub(super) const PI: i128 = {};\n/// Tau with 64 fractional bits for bounded full-range input reduction.\npub(super) const TAU_Q64: i128 = {};\n/// Reciprocal CORDIC gain after all table iterations.\npub(super) const GAIN: i128 = {};\n/// atan(2^-i) in radians for rotation and vectoring iterations.\npub(super) const ANGLES: [i128; 97] = [\n",quant(&reference.pi,96),quant(&reference.pi.scale(1),64),quant(&reference.gain(97),96));
    for angle in &angles {
        constants += &format!("    {},\n", quant(angle, 96));
    }
    constants+=&format!("];\n\n/// Pi with 60 fractional bits for the experimental i64 path.\npub(super) const PI_Q60: i64 = {};\n/// Reciprocal gain for 61 steps; shorter schedules differ by less than 2^-64.\npub(super) const GAIN_Q60: i64 = {};\n/// atan(2^-i) with 60 fractional bits, rescaled at compile time.\npub(super) const ANGLES_Q60: [i64; 61] = [\n",quant(&reference.pi,60),quant(&reference.gain(61),60));
    for angle in &angles[..61] {
        constants += &format!("    {},\n", quant(angle, 60));
    }
    constants += "];\n";
    let notice = license.replace("//", "#");
    let mut basic =
        format!("{notice}\n# Independent high-precision references: FRAC,raw,sin_raw,cos_raw\n");
    for frac in [2, 8, 16, 32, 61] {
        for raw in [
            i64::MIN as i128,
            i64::MIN as i128 + 1,
            -1234567890123456789,
            -1,
            0,
            1,
            1234567890123456789,
            i64::MAX as i128 - 1,
            i64::MAX as i128,
        ] {
            let (s, c) = reference.sin_cos(&reference.n(raw).scale(-frac));
            basic += &format!("{frac},{raw},{},{}\n", quant(&s, frac), quant(&c, frac));
        }
    }
    let mut experiment=format!("{notice}\n# Independent high-precision reference: storage,frac,op,input_raw,nearest_raw,residual_raw\n");
    let mut state = 0x123456789abcdefu64;
    for (bits, frac) in [
        (32, 8),
        (32, 16),
        (32, 29),
        (64, 16),
        (64, 32),
        (64, 44),
        (64, 48),
    ] {
        let minimum = -(1i128 << (bits - 1));
        let maximum = (1i128 << (bits - 1)) - 1;
        let scale = 1i128 << frac;
        let mut inputs = BTreeSet::from([0, -1, 1]);
        for i in 0..32 {
            inputs.insert(minimum + i);
            inputs.insert(maximum - i);
        }
        for _ in 0..256 {
            state = state
                .wrapping_mul(6364136223846793005)
                .wrapping_add(1442695040888963407);
            inputs.insert(minimum + (state >> (64 - bits)) as i128);
        }
        let max_turn = (&reference.n(maximum) / &reference.pi.scale(frac - 1)).trunc_i128();
        for turns in [0, 1, -1, 2, -2, 3, 1001, max_turn, -max_turn] {
            let base = (&reference.n(turns) * &reference.pi.scale(frac - 1)).nearest_i128();
            for offset in -3..=3 {
                let raw = base + offset;
                if (minimum..=maximum).contains(&raw) {
                    inputs.insert(raw);
                }
            }
        }
        for raw in inputs {
            let (s, c) = reference.sin_cos(&reference.n(raw).scale(-frac));
            reference.record(&mut experiment, bits, frac as u32, "sin", raw, &s);
            reference.record(&mut experiment, bits, frac as u32, "cos", raw, &c);
            reference.record(&mut experiment, bits, frac as u32, "tan", raw, &(&s / &c));
        }
        let mut inputs = BTreeSet::from([0, -1, 1]);
        for i in 0..17 {
            inputs.insert(-scale + i);
            inputs.insert(scale - i);
        }
        for _ in 0..128 {
            state = state
                .wrapping_mul(6364136223846793005)
                .wrapping_add(1442695040888963407);
            inputs.insert(-scale + (state as i128 % (2 * scale + 1)));
        }
        for raw in inputs {
            reference.record(
                &mut experiment,
                bits,
                frac as u32,
                "acos",
                raw,
                &reference.acos(&reference.n(raw).scale(-frac)),
            );
        }
    }
    [
        ("constants.rs".to_owned(), constants),
        ("reference.csv".to_owned(), basic),
        ("cordic_reference.csv".to_owned(), experiment),
    ]
}

/// Defaults to 512-bit GMP precision. --bits 768 checks independent guard-width
/// stability, --check compares files without writing, and --output owns no cleanup.
fn main() -> io::Result<()> {
    let mut bits = 512;
    let mut check = false;
    let mut output = root().join("src/fixed");
    let mut args = std::env::args().skip(1);
    while let Some(arg) = args.next() {
        match arg.as_str() {
            "--check" => check = true,
            "--bits" => {
                bits = args
                    .next()
                    .expect("precision")
                    .parse()
                    .expect("integer precision")
            }
            "--output" => output = PathBuf::from(args.next().expect("output directory")),
            _ => {
                return Err(io::Error::other(
                    "usage: generate-fixed-constants [--check] [--bits 512..4096] [--output DIR]",
                ))
            }
        }
    }
    let source = fs::read_to_string(root().join("src/lib.rs"))?;
    let license = source.split("//! # rs-math3d").next().unwrap().trim_end();
    let reference = Reference::new(bits);
    let generated = generate(&reference, license);
    if !check {
        fs::create_dir_all(&output)?;
    }
    for (name, text) in generated {
        let path = output.join(name);
        if check {
            if fs::read_to_string(&path)? != text {
                return Err(io::Error::other(format!(
                    "generated reference differs: {}",
                    path.display()
                )));
            }
        } else {
            fs::write(path, text)?;
        }
    }
    println!(
        "Verified/generated constants and references with {bits}-bit independent GMP arithmetic."
    );
    Ok(())
}

#[cfg(test)]
mod tests {
    use super::*;
    #[test]
    fn known_constants_and_axes() {
        let r = Reference::new(512);
        assert_eq!(
            r.pi.scale(96).nearest_i128(),
            248902613312231085230521944622
        );
        let (s, c) = r.sin_cos(&r.n(0));
        assert_eq!(s.nearest_i128(), 0);
        assert_eq!(c.nearest_i128(), 1);
        assert_eq!(
            r.acos(&r.n(-1)).scale(96).nearest_i128(),
            r.pi.scale(96).nearest_i128()
        );
    }
}
