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

//! Deterministic display-free run of the shared rigid-body demo scene.
#[allow(dead_code)]
mod collision;
#[allow(dead_code)]
mod physics;
use rs_math3d::{Fixed, RealScalar, TryCastFrom};
fn run<T: RealScalar + TryCastFrom<f64>>(ticks: usize) -> Result<(), String>
where
    f64: TryCastFrom<T>,
{
    let mut world = physics::scene::demo::<T>()?;
    let start = std::time::Instant::now();
    let mut contacts = 0;
    for _ in 0..ticks {
        world.step();
        contacts += world.stats().contacts;
        for (_, body) in world.bodies() {
            let p = body.pose();
            let q = p.orientation;
            let v = body.velocity();
            let a = body.angular_velocity();
            if ![
                p.position.x,
                p.position.y,
                p.position.z,
                q.x,
                q.y,
                q.z,
                q.w,
                v.x,
                v.y,
                v.z,
                a.x,
                a.y,
                a.z,
            ]
            .iter()
            .all(|x| x.is_finite())
            {
                return Err("nonfinite simulation state".into());
            }
        }
    }
    println!(
        "ticks={ticks} bodies={} contacts_total={contacts} stats={:?} ms/tick={:.3}",
        world.bodies().count(),
        world.stats(),
        start.elapsed().as_secs_f64() * 1000.0 / ticks as f64
    );
    for (id, body) in world.bodies() {
        if !body.enabled() {
            let p = body.pose().position;
            let xyz = p.map(f64::cast_from).to_array();
            println!("disabled body={} position={xyz:?}", id.index());
        }
    }
    Ok(())
}
fn main() -> Result<(), String> {
    let mut args = std::env::args().skip(1);
    let scalar = args.next().unwrap_or_else(|| "f32".to_owned());
    let ticks = args
        .next()
        .map(|s| s.parse::<usize>().map_err(|e| e.to_string()))
        .transpose()?
        .unwrap_or(600);
    if ticks == 0 {
        return Err("ticks must be positive".into());
    }
    match scalar.as_str() {
        "f32" => run::<f32>(ticks),
        "fixed32" => run::<Fixed<i32, 16>>(ticks),
        "fixed64" => run::<Fixed<i64, 32>>(ticks),
        _ => Err("usage: physics_headless [f32|fixed32|fixed64] [ticks]".into()),
    }
}
