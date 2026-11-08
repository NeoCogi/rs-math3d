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

//! Checked decoder for the pinned Q8 bunny and Utah teapot benchmark fixtures.

use core::convert::TryInto;
use rs_math3d::{RealScalar, Tri3, TryCastFrom, Vector3};

/// Model data remains separate from the crate's source-code licensing.
pub const BUNNY: &[u8] = include_bytes!("assets/bunny.soup");
/// Utah Teapot mesh converted to the common benchmark grid.
pub const TEAPOT: &[u8] = include_bytes!("assets/teapot.soup");

/// Errors in a fixture packet or an unsupported destination scalar.
#[derive(Clone, Copy, Debug, Eq, PartialEq)]
pub enum DecodeError {
    /// Header/version or exact packet length is invalid.
    InvalidPacket,
    /// A triangle references a vertex outside the packet.
    InvalidIndex,
    /// A coordinate cannot be represented by the requested scalar.
    ScalarRange,
}

/// Decodes exactly one packet with bounded indexing and checked byte counts.
/// Input conversion and allocation belong to fixture setup, outside query timers.
pub fn decode<T: RealScalar + TryCastFrom<f64>>(data: &[u8]) -> Result<Vec<Tri3<T>>, DecodeError> {
    if data.len() < 16 || &data[..8] != b"M3SOUP01" {
        return Err(DecodeError::InvalidPacket);
    }
    let count = |start| u32::from_le_bytes(data[start..start + 4].try_into().unwrap()) as usize;
    let vertices = count(8);
    let faces = count(12);
    let vertex_bytes = vertices.checked_mul(6).ok_or(DecodeError::InvalidPacket)?;
    let face_bytes = faces.checked_mul(6).ok_or(DecodeError::InvalidPacket)?;
    let end = 16usize
        .checked_add(vertex_bytes)
        .and_then(|n| n.checked_add(face_bytes))
        .ok_or(DecodeError::InvalidPacket)?;
    if end != data.len() || vertices > 65535 {
        return Err(DecodeError::InvalidPacket);
    }
    let mut points = Vec::with_capacity(vertices);
    for bytes in data[16..16 + vertex_bytes].chunks_exact(6) {
        let coordinate = |i| {
            let raw = i16::from_le_bytes([bytes[i], bytes[i + 1]]);
            T::try_cast_from(raw as f64 / 256.0).map_err(|_| DecodeError::ScalarRange)
        };
        points.push(Vector3::new(coordinate(0)?, coordinate(2)?, coordinate(4)?));
    }
    let mut triangles = Vec::with_capacity(faces);
    for bytes in data[16 + vertex_bytes..].chunks_exact(6) {
        let point = |i| {
            let index = u16::from_le_bytes([bytes[i], bytes[i + 1]]) as usize;
            points.get(index).copied().ok_or(DecodeError::InvalidIndex)
        };
        triangles.push(Tri3::new([point(0)?, point(2)?, point(4)?]));
    }
    Ok(triangles)
}
