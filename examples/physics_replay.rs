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

//! Display-free raw replay recorder/verifier for architecture comparisons.
#[allow(dead_code)]
mod collision;
#[allow(dead_code)]
mod physics;
use physics::replay::{Frame, Replay, ReplayScalar};
use rs_math3d::Fixed;
use std::{
    fs,
    io::{BufWriter, Write},
    path::Path,
};

/// Writes every frame, including tick zero, in a platform-independent text form.
fn trace<T: ReplayScalar>(seed: &[u8], path: &Path) -> Result<(), String> {
    let mut replay = Replay::<T>::decode(seed).map_err(|e| format!("seed: {e:?}"))?;
    let mut file = BufWriter::new(fs::File::create(path).map_err(|e| e.to_string())?);
    let mut frame = replay.frame();
    loop {
        writeln!(file, "{} {:016x}", frame.tick, frame.hash).map_err(|e| e.to_string())?;
        match replay
            .step()
            .map_err(|e| format!("tick {}: {e:?}", frame.tick))?
        {
            Some(next) => frame = next,
            None => break,
        }
    }
    file.flush().map_err(|e| e.to_string())?;
    println!("recorded {} ticks; final {:016x}", frame.tick, frame.hash);
    Ok(())
}
/// Compares each expected frame at the first differing tick. Missing/extra or
/// malformed trace lines fail; a matching final state alone cannot pass.
fn check<T: ReplayScalar>(seed: &[u8], expected: &str) -> Result<(), String> {
    let mut replay = Replay::<T>::decode(seed).map_err(|e| format!("seed: {e:?}"))?;
    let mut actual = Some(replay.frame());
    let mut count = 0;
    for line in expected.lines() {
        let mut columns = line.split_whitespace();
        let tick = columns
            .next()
            .ok_or("missing trace tick")?
            .parse::<u32>()
            .map_err(|e| e.to_string())?;
        let hash = u64::from_str_radix(columns.next().ok_or("missing trace hash")?, 16)
            .map_err(|e| e.to_string())?;
        if columns.next().is_some() {
            return Err("extra trace column".into());
        }
        let frame = Frame { tick, hash };
        if actual != Some(frame) {
            return Err(format!(
                "first mismatch: expected {frame:?}, actual {actual:?}"
            ));
        }
        count += 1;
        actual = replay.step().map_err(|e| format!("tick {tick}: {e:?}"))?;
    }
    if actual.is_some() || count == 0 {
        return Err("trace ended before replay completion".into());
    }
    println!(
        "verified {count} states; final {:016x}",
        replay.frame().hash
    );
    Ok(())
}
/// Chooses a fixed format from the versioned packet, never host pointer size.
fn consume(seed: &[u8], path: &Path, verify: bool) -> Result<(), String> {
    let format = Replay::<Fixed<i32, 16>>::format(seed).map_err(|e| format!("header: {e:?}"))?;
    let expected = if verify {
        fs::read_to_string(path).map_err(|e| e.to_string())?
    } else {
        String::new()
    };
    match (format, verify) {
        ((32, 16), true) => check::<Fixed<i32, 16>>(seed, &expected),
        ((64, 32), true) => check::<Fixed<i64, 32>>(seed, &expected),
        ((32, 16), false) => trace::<Fixed<i32, 16>>(seed, path),
        ((64, 32), false) => trace::<Fixed<i64, 32>>(seed, path),
        _ => Err(format!("unsupported raw replay format {format:?}")),
    }
}
/// File generation is an explicit command; ordinary checks cannot rewrite their
/// golden input or expected trajectory as a side effect of a failed comparison.
fn main() -> Result<(), String> {
    let args: Vec<_> = std::env::args().skip(1).collect();
    match args.as_slice(){
        [command,scalar,seed_path,trace_path] if command=="record"=>{
            let seed=match scalar.as_str(){
                "fixed32"=>physics::replay::record_demo::<Fixed<i32,16>>()?,
                "fixed64"=>physics::replay::record_demo::<Fixed<i64,32>>()?,
                _=>return Err("record requires fixed32 or fixed64".into()),
            };
            fs::write(seed_path,&seed).map_err(|e|e.to_string())?;
            consume(&seed,Path::new(trace_path),false)
        },
        [command,seed_path,trace_path] if command=="check"||command=="trace"=>{
            let seed=fs::read(seed_path).map_err(|e|e.to_string())?;
            consume(&seed,Path::new(trace_path),command=="check")
        },
        _=>Err("usage: physics_replay record fixed32|fixed64 seed.bin trace.txt | check|trace seed.bin trace.txt".into()),
    }
}
