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

//! Common model-relative capsule and OBB setup for regression tests and timings.
//! Floating setup is hosted and outside timers; all query inputs are snapped to
//! common binary grids so f32 and both fixed formats start from identical values.

use crate::collision::{aabb_tree::AabbTree, Capsule, Obb};
use rs_math3d::{RealScalar, TryCastFrom, Vector3};

/// Shared query corpus, with varied orientation, size and surface-normal offset.
pub struct ShapeCases<T: RealScalar> {
    /// Model-relative tilted capsules, plus explicit global misses.
    pub capsules: Vec<Capsule<T>>,
    /// Boxes rotated around two axes, plus explicit global misses.
    pub obbs: Vec<Obb<T>>,
}

/// Ties-to-even setup quantization, retaining a common exactly represented input.
fn snap(x: f64, scale: f64) -> f64 {
    (x * scale).round_ties_even() / scale
}

/// Copies prepared scalar data after the common-grid setup is complete.
fn scalar<T: TryCastFrom<f64>>(x: f64) -> T {
    T::try_cast_from(x).ok().expect("shape fixture fits scalar")
}

/// Encodes an exactly shared vector, using Q8 coordinates or Q16 unit axes.
fn vector<T: RealScalar + TryCastFrom<f64>>(v: [f64; 3], scale: f64) -> Vector3<T> {
    Vector3::new(
        scalar(snap(v[0], scale)),
        scalar(snap(v[1], scale)),
        scalar(snap(v[2], scale)),
    )
}

/// Builds 256 surface-neighborhood poses and eight remote poses per shape.
/// Normals/tangents are computed in setup from the exact shared model vertices;
/// axes are Q16 and centers/endpoints Q8, within every benchmark scalar's range.
pub fn model_cases<T: RealScalar + TryCastFrom<f64>>(tree: &AabbTree<T>) -> ShapeCases<T>
where
    f64: TryCastFrom<T>,
{
    let mut capsules = Vec::new();
    let mut obbs = Vec::new();
    let count = tree.info().triangles;
    if count == 0 {
        return ShapeCases { capsules, obbs };
    }
    for sample in 0..64 {
        let vertices = tree.triangle(sample * count / 64).unwrap().vertices();
        let points: [[f64; 3]; 3] = core::array::from_fn(|i| {
            let p = vertices[i];
            [p.x, p.y, p.z].map(|x| f64::try_cast_from(x).ok().unwrap())
        });
        let a = core::array::from_fn::<_, 3, _>(|i| points[1][i] - points[0][i]);
        let b = core::array::from_fn::<_, 3, _>(|i| points[2][i] - points[0][i]);
        let normal = [
            a[1] * b[2] - a[2] * b[1],
            a[2] * b[0] - a[0] * b[2],
            a[0] * b[1] - a[1] * b[0],
        ];
        let normal_length = normal.iter().map(|x| x * x).sum::<f64>().sqrt();
        let tangent_length = a.iter().map(|x| x * x).sum::<f64>().sqrt();
        assert!(normal_length > 0.0 && tangent_length > 0.0);
        let normal = normal.map(|x| x / normal_length);
        let tangent = a.map(|x| x / tangent_length);
        let center =
            core::array::from_fn::<_, 3, _>(|i| (points[0][i] + points[1][i] + points[2][i]) / 3.0);
        let angle = (sample as f64 - 31.5) / 32.0;
        let (s, c) = angle.sin_cos();
        let (sy, cy) = (angle * 0.625).sin_cos();
        let axes = [
            vector([c * cy, s * cy, -sy], 65536.0),
            vector([-s, c, 0.0], 65536.0),
            vector([c * sy, s * sy, cy], 65536.0),
        ];
        for offset in [-1.5, -0.25, 0.25, 1.5] {
            let center = core::array::from_fn::<_, 3, _>(|i| center[i] + normal[i] * offset);
            capsules.push(Capsule {
                a: vector(
                    core::array::from_fn(|i| center[i] - tangent[i] * 0.75 - normal[i] * 0.125),
                    256.0,
                ),
                b: vector(
                    core::array::from_fn(|i| center[i] + tangent[i] * 0.75 + normal[i] * 0.125),
                    256.0,
                ),
                radius: scalar(if sample % 2 == 0 { 0.375 } else { 0.5 }),
            });
            obbs.push(Obb {
                center: vector(center, 256.0),
                axes,
                half: Vector3::new(scalar(1.0), scalar(0.5), scalar(0.25)),
            });
        }
    }
    for i in 0..8 {
        let center = Vector3::new(scalar::<T>(40.0 + i as f64), scalar(40.0), scalar(40.0));
        capsules.push(Capsule {
            a: center,
            b: center,
            radius: scalar(0.5),
        });
        obbs.push(Obb {
            center,
            axes: [
                vector([1.0, 0.0, 0.0], 65536.0),
                vector([0.0, 1.0, 0.0], 65536.0),
                vector([0.0, 0.0, 1.0], 65536.0),
            ],
            half: Vector3::new(scalar(1.0), scalar(0.5), scalar(0.25)),
        });
    }
    ShapeCases { capsules, obbs }
}
