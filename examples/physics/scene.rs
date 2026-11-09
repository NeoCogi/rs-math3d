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

//! Shared demo/headless scene. Model decoding and transforms are hosted setup;
//! the actual World operates entirely in its selected scalar backend.

use super::*;
#[path = "../collision/fixtures.rs"]
mod fixtures;
use rs_math3d::Tri3;

/// Default friction/restitution for dynamic shapes and static triangle surfaces.
pub fn material<T: RealScalar + TryCastFrom<f64>>() -> Material<T> {
    Material {
        friction: T::cast_from(0.65),
        restitution: T::cast_from(0.15),
    }
}
/// Creates a sphere, capsule or box with a tilted initial orientation. Dimensions
/// are shared with rendering through Shape; no separate visible collider exists.
pub fn body<T: RealScalar + TryCastFrom<f64>>(
    kind: usize,
    position: [f64; 3],
    seed: usize,
) -> Result<Body<T>, Error> {
    let shape = match kind % 3 {
        0 => Shape::Sphere {
            radius: T::cast_from(0.42),
        },
        1 => Shape::Box {
            half: Vector3::from_array([0.45, 0.35, 0.4]).map(T::cast_from),
        },
        _ => Shape::Capsule {
            half_height: T::cast_from(0.45),
            radius: T::cast_from(0.27),
        },
    };
    let angle = <T>::cast_from((seed % 7) as f64 * 0.11);
    let axis = contact::unit(Vector3::from_array([0.7, 0.3, 1.0]).map(T::cast_from)).unwrap();
    let orientation = Quat::of_axis_angle(&axis, angle);
    Body::new(
        shape,
        Pose {
            position: Vector3::from_array(position).map(T::cast_from),
            orientation,
        },
        T::cast_from(1.0),
        material(),
    )
}
/// Builds a tiled floor, upright teapot and bunny, then three falling shape
/// groups. Floor edge lengths are bounded to avoid quartic S15.16 intermediates.
pub fn demo<T: RealScalar + TryCastFrom<f64>>() -> Result<World<T>, String> {
    let mut world = World::new();
    let mut ground = Vec::new();
    for x in 0..8 {
        for z in 0..8 {
            let x = -16.0 + x as f64 * 4.0;
            let z = -16.0 + z as f64 * 4.0;
            let a = Vector3::from_array([x, 0.0, z]).map(T::cast_from);
            let b = Vector3::from_array([x + 4.0, 0.0, z]).map(T::cast_from);
            let c = Vector3::from_array([x + 4.0, 0.0, z + 4.0]).map(T::cast_from);
            let d = Vector3::from_array([x, 0.0, z + 4.0]).map(T::cast_from);
            ground.push(Tri3::new([a, c, b]));
            ground.push(Tri3::new([a, d, c]));
        }
    }
    world
        .add_mesh(StaticMesh {
            tree: AabbTree::new(ground, 4).map_err(|e| format!("floor: {e:?}"))?,
            material: material(),
        })
        .map_err(|e| format!("floor material: {e:?}"))?;
    for (bytes, x) in [(fixtures::TEAPOT, -5.0), (fixtures::BUNNY, 4.0)] {
        let source = fixtures::decode::<f64>(bytes).map_err(|e| format!("fixture: {e:?}"))?;
        let min_y = source
            .iter()
            .flat_map(|t| t.vertices().iter().map(|p| p.y * 0.25))
            .fold(f64::INFINITY, f64::min);
        let triangles = source
            .iter()
            .map(|t| {
                Tri3::new(t.vertices().map(|p| {
                    Vector3::from_array([p.x * 0.25 + x, p.y * 0.25 - min_y, p.z * 0.25])
                        .map(T::cast_from)
                }))
            })
            .collect();
        world
            .add_mesh(StaticMesh {
                tree: AabbTree::new(triangles, 4).map_err(|e| format!("model: {e:?}"))?,
                material: material(),
            })
            .map_err(|e| format!("model material: {e:?}"))?;
    }
    for i in 0..24 {
        let group = i / 8;
        let x = [-5.0, 0.0, 4.0][group] + ((i % 2) as f64 - 0.5) * 1.0;
        let z = ((i / 2) % 2) as f64 * 1.2 - 0.6;
        let mut body =
            body(i, [x, 10.0 + (i % 8) as f64 * 0.8, z], i).map_err(|e| format!("body: {e:?}"))?;
        body.set_velocity(
            Vector3::from_array([if i % 2 == 0 { 0.4 } else { -0.4 }, 0.0, 0.2]).map(T::cast_from),
            Vector3::from_array([0.0, 0.2, 0.0]).map(T::cast_from),
        )
        .map_err(|e| format!("velocity: {e:?}"))?;
        world.add_body(body).map_err(|e| format!("spawn: {e:?}"))?;
    }
    Ok(world)
}
