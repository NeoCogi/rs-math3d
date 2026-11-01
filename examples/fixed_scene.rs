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

//! A fixed-point scene/query that also runs with `--no-default-features`.

use rs_math3d::{transforms, Fixed, Intersection, MathError, Ray, Tri3, Vector3};

/// Builds a transformed triangle and intersects a forward ray using fixed math.
fn main() -> Result<(), MathError> {
    type F = Fixed<i32, 16>;
    let zero = F::ZERO;
    let one = F::ONE;
    let transform = transforms::translate(Vector3::new(one, zero, zero));
    let triangle = Tri3::new([
        transform * Vector3::new(zero, zero, zero),
        transform * Vector3::new(one, zero, zero),
        transform * Vector3::new(zero, one, zero),
    ]);
    let quarter = F::try_from_ratio(1, 4)?;
    let ray = Ray::new(
        &Vector3::new(one + quarter, quarter, one),
        &Vector3::new(zero, zero, -one),
        F::DELTA,
    )
    .ok_or(MathError::Degenerate)?;
    if let Some((parameter, point)) = ray.checked_intersection(&triangle)? {
        println!(
            "hit ({}, {}, {}) at t={}",
            point.x, point.y, point.z, parameter
        );
    }
    Ok(())
}
