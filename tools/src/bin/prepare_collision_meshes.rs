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

//! Prepare the pinned Stanford bunny/Utah teapot fixtures. curl, tar and
//! sha256sum are developer tools only; ordinary tests use checked-in packets.

use rs_math3d_tools::{checked, json_string, root, Scratch};
use std::{
    fs, io,
    path::{Path, PathBuf},
    process::Command,
};

/// One decoded triangle soup; all indices are validated before encoding.
struct Mesh {
    /// Source-coordinate vertices before normalization.
    points: Vec<[f64; 3]>,
    /// Original ordered triangular vertex indices.
    faces: Vec<[usize; 3]>,
}
/// Encoded fixture and normalization metadata for its manifest entry.
struct Encoded {
    /// Complete little-endian M3SOUP01 packet.
    bytes: Vec<u8>,
    /// Count of retained source vertices, including unreferenced ones.
    vertices: usize,
    /// Original number of triangular faces before collapse filtering.
    source_faces: usize,
    /// Number of retained nondegenerate faces.
    faces: usize,
    /// Source-coordinate midpoint used by the normalization.
    center: [f64; 3],
    /// Uniform source-to-fixture scale, before Q8 encoding.
    scale: f64,
}
/// Pinned source provenance, including a digest checked before parsing.
struct Source {
    /// Stable fixture basename.
    name: &'static str,
    /// Pinned upstream source location, used only with --download.
    url: &'static str,
    /// Mandatory full-source SHA256 before parsing.
    sha: &'static str,
}
const SOURCES: [Source; 2] = [
    Source {
        name: "bunny",
        url: "https://graphics.stanford.edu/pub/3Dscanrep/bunny.tar.gz",
        sha: "a5720bd96d158df403d153381b8411a727a1d73cff2f33dc9b212d6f75455b84",
    },
    Source {
        name: "teapot",
        url: "https://graphics.stanford.edu/courses/cs148-10-summer/as3/code/as3/teapot.obj",
        sha: "1b5396fedd74b577e32cef41146582c2f2e1a050d5b4915193c0ac1ad4187ed4",
    },
];

/// Creates a diagnostic for malformed or unsupported source data.
fn invalid(message: impl Into<String>) -> io::Error {
    io::Error::new(io::ErrorKind::InvalidData, message.into())
}
/// Parses one numeric field while preserving a useful error boundary.
fn number<T: std::str::FromStr>(s: &str) -> io::Result<T> {
    s.parse()
        .map_err(|_| invalid(format!("invalid numeric field {s}")))
}
/// Reads exactly three coordinates, ignoring allowed trailing vertex properties.
fn point(line: &str) -> io::Result<[f64; 3]> {
    let fields: Vec<_> = line.split_whitespace().collect();
    if fields.len() < 3 {
        return Err(invalid("missing point coordinates"));
    }
    Ok([number(fields[0])?, number(fields[1])?, number(fields[2])?])
}
/// Parses the pinned ASCII PLY member without unpacking arbitrary archive paths.
fn bunny(text: &str) -> io::Result<Mesh> {
    let lines: Vec<_> = text.lines().collect();
    let end = lines
        .iter()
        .position(|l| *l == "end_header")
        .ok_or_else(|| invalid("missing PLY header"))?
        + 1;
    if !lines[..end].contains(&"format ascii 1.0") {
        return Err(invalid("only ASCII PLY supported"));
    }
    let count = |prefix: &str| -> io::Result<usize> {
        number(
            lines[..end]
                .iter()
                .find_map(|l| l.strip_prefix(prefix))
                .ok_or_else(|| invalid("missing PLY element count"))?,
        )
    };
    let vertices = count("element vertex ")?;
    let faces = count("element face ")?;
    if lines.len() < end + vertices + faces {
        return Err(invalid("truncated PLY"));
    }
    let points = lines[end..end + vertices]
        .iter()
        .map(|l| point(l))
        .collect::<io::Result<_>>()?;
    let mut cells = Vec::with_capacity(faces);
    for line in &lines[end + vertices..end + vertices + faces] {
        let f: Vec<_> = line.split_whitespace().collect();
        if f.len() != 4 || f[0] != "3" {
            return Err(invalid("non-triangle PLY face"));
        }
        cells.push([number(f[1])?, number(f[2])?, number(f[3])?]);
    }
    Ok(Mesh {
        points,
        faces: cells,
    })
}
/// Parses the pinned triangular OBJ, rejecting relative/zero/out-of-range indices.
fn teapot(text: &str) -> io::Result<Mesh> {
    let mut mesh = Mesh {
        points: Vec::new(),
        faces: Vec::new(),
    };
    for line in text.lines() {
        let fields: Vec<_> = line.split_whitespace().collect();
        match fields.first().copied() {
            Some("v") => mesh.points.push(point(line.strip_prefix('v').unwrap())?),
            Some("f") => {
                if fields.len() != 4 {
                    return Err(invalid("non-triangle OBJ face"));
                }
                let mut cell = [0; 3];
                for i in 0..3 {
                    cell[i] = number::<usize>(fields[i + 1].split('/').next().unwrap())?
                        .checked_sub(1)
                        .ok_or_else(|| invalid("zero OBJ index"))?;
                }
                mesh.faces.push(cell);
            }
            _ => {}
        }
    }
    Ok(mesh)
}
/// Centers/scales to extent 32, snaps ties-to-even to Q8 and removes collapsed
/// faces with exact integer cross products. Face order/winding remain unchanged.
fn encode(mesh: Mesh) -> io::Result<Encoded> {
    if mesh.points.is_empty()
        || mesh.points.len() >= 65536
        || mesh.points.iter().flatten().any(|x| !x.is_finite())
    {
        return Err(invalid("invalid vertex count/coordinate"));
    }
    let minimum = core::array::from_fn::<_, 3, _>(|i| {
        mesh.points
            .iter()
            .map(|p| p[i])
            .fold(f64::INFINITY, f64::min)
    });
    let maximum = core::array::from_fn::<_, 3, _>(|i| {
        mesh.points
            .iter()
            .map(|p| p[i])
            .fold(f64::NEG_INFINITY, f64::max)
    });
    let center = core::array::from_fn(|i| (minimum[i] + maximum[i]) / 2.0);
    let extent = (0..3).map(|i| maximum[i] - minimum[i]).fold(0.0, f64::max);
    if !extent.is_finite() || extent <= 0.0 {
        return Err(invalid("invalid model extent"));
    }
    let scale = 32.0 / extent;
    if !scale.is_finite() || center.iter().any(|x| !x.is_finite()) {
        return Err(invalid("normalization exceeds host float range"));
    }
    let points: Vec<[i16; 3]> = mesh
        .points
        .iter()
        .map(|p| {
            core::array::from_fn(|i| ((p[i] - center[i]) * scale * 256.0).round_ties_even() as i16)
        })
        .collect();
    let mut faces = Vec::new();
    for face in &mesh.faces {
        if face.iter().any(|&i| i >= points.len()) {
            return Err(invalid("face index outside vertex table"));
        }
        let [a, b, c] = face.map(|i| points[i].map(i64::from));
        let u = core::array::from_fn::<_, 3, _>(|i| b[i] - a[i]);
        let v = core::array::from_fn::<_, 3, _>(|i| c[i] - a[i]);
        let cross = [
            u[1] * v[2] - u[2] * v[1],
            u[2] * v[0] - u[0] * v[2],
            u[0] * v[1] - u[1] * v[0],
        ];
        if cross != [0; 3] {
            faces.push(*face);
        }
    }
    let mut bytes = b"M3SOUP01".to_vec();
    bytes.extend((points.len() as u32).to_le_bytes());
    bytes.extend((faces.len() as u32).to_le_bytes());
    for p in &points {
        for v in p {
            bytes.extend(v.to_le_bytes());
        }
    }
    for f in &faces {
        for v in f {
            bytes.extend((*v as u16).to_le_bytes());
        }
    }
    Ok(Encoded {
        bytes,
        vertices: points.len(),
        source_faces: mesh.faces.len(),
        faces: faces.len(),
        center,
        scale,
    })
}
/// Uses the platform digest utility, never trusting a filename-derived digest.
fn sha(path: &Path) -> io::Result<String> {
    let output = checked(Command::new("sha256sum").arg(path))?;
    let text = String::from_utf8(output.stdout).map_err(io::Error::other)?;
    let digest = text
        .split_whitespace()
        .next()
        .ok_or_else(|| invalid("missing SHA256"))?;
    if digest.len() != 64 || !digest.bytes().all(|c| c.is_ascii_hexdigit()) {
        return Err(invalid("invalid SHA256 output"));
    }
    Ok(digest.to_owned())
}

/// Validates both sources before publishing any fixture. --check is read-only.
fn main() -> io::Result<()> {
    let mut download = false;
    let mut check = false;
    let mut local: [Option<PathBuf>; 2] = [None, None];
    let mut output = root().join("examples/collision/assets");
    let mut args = std::env::args().skip(1);
    while let Some(arg) = args.next() {
        match arg.as_str() {
            "--download"=>download=true,"--check"=>check=true,
            "--bunny-archive"=>local[0]=Some(args.next().ok_or_else(||invalid("missing archive"))?.into()),
            "--teapot-obj"=>local[1]=Some(args.next().ok_or_else(||invalid("missing OBJ"))?.into()),
            "--output"=>output=args.next().ok_or_else(||invalid("missing output"))?.into(),
            _=>return Err(invalid("usage: prepare-collision-meshes [--download | --bunny-archive PATH --teapot-obj PATH] [--output DIR] [--check]")),
        }
    }
    let scratch = Scratch::new("math3d-mesh")?;
    let mut prepared = Vec::new();
    let mut entries = Vec::new();
    for (i, source) in SOURCES.iter().enumerate() {
        let path = if download {
            let path = scratch.path().join(source.name);
            checked(
                Command::new("curl")
                    .args(["-fL", "--max-time", "60", "-o"])
                    .arg(&path)
                    .arg(source.url),
            )?;
            path
        } else {
            local[i]
                .clone()
                .ok_or_else(|| invalid("supply both local sources or --download"))?
        };
        if sha(&path)? != source.sha {
            return Err(invalid(format!("{} source SHA256 mismatch", source.name)));
        }
        let data = if i == 0 {
            checked(
                Command::new("tar")
                    .arg("-xOzf")
                    .arg(&path)
                    .arg("bunny/reconstruction/bun_zipper_res3.ply"),
            )?
            .stdout
        } else {
            fs::read(path)?
        };
        let text = String::from_utf8(data).map_err(io::Error::other)?;
        let encoded = encode(if i == 0 {
            bunny(&text)?
        } else {
            teapot(&text)?
        })?;
        let candidate = scratch.path().join(format!("{}.soup", source.name));
        fs::write(&candidate, &encoded.bytes)?;
        let digest = sha(&candidate)?;
        entries.push(format!("    {}: {{\n      \"collapsed_faces_removed\": {},\n      \"fixture_sha256\": {},\n      \"source_center\": [{}, {}, {}],\n      \"source_scale\": {},\n      \"source_sha256\": {},\n      \"source_triangles\": {},\n      \"source_url\": {},\n      \"triangles\": {},\n      \"vertices\": {}\n    }}",json_string(source.name),encoded.source_faces-encoded.faces,json_string(&digest),encoded.center[0],encoded.center[1],encoded.center[2],encoded.scale,json_string(source.sha),encoded.source_faces,json_string(source.url),encoded.faces,encoded.vertices));
        println!(
            "{}: {} triangles; {} collapsed faces removed; SHA256 {digest}",
            source.name,
            encoded.faces,
            encoded.source_faces - encoded.faces
        );
        prepared.push((format!("{}.soup", source.name), encoded.bytes));
    }
    let manifest=format!("{{\n  \"format\": \"M3SOUP01\",\n  \"grid\": \"1/256\",\n  \"longest_extent\": 32,\n  \"models\": {{\n{}\n  }}\n}}\n",entries.join(",\n"));
    prepared.push(("manifest.json".to_owned(), manifest.into_bytes()));
    if !check {
        fs::create_dir_all(&output)?;
    }
    for (name, bytes) in prepared {
        let path = output.join(name);
        if check {
            if fs::read(&path)? != bytes {
                return Err(invalid(format!("fixture differs: {}", path.display())));
            }
        } else {
            fs::write(path, bytes)?;
        }
    }
    Ok(())
}

#[cfg(test)]
mod tests {
    use super::*;
    #[test]
    fn encoding_preserves_order_and_removes_collapsed_faces() {
        let mesh = Mesh {
            points: vec![[0., 0., 0.], [1., 0., 0.], [0., 1., 0.]],
            faces: vec![[0, 0, 1], [0, 1, 2]],
        };
        let result = encode(mesh).unwrap();
        assert_eq!(result.faces, 1);
        assert_eq!(&result.bytes[..8], b"M3SOUP01");
        assert_eq!(&result.bytes[result.bytes.len() - 6..], &[0, 0, 1, 0, 2, 0]);
    }
    #[test]
    fn malformed_geometry_is_rejected() {
        assert!(teapot("v 0 0 0\nf 0 1 2").is_err());
        assert!(encode(Mesh {
            points: vec![[0.; 3]],
            faces: vec![]
        })
        .is_err());
        assert!(bunny("ply\nformat binary_little_endian 1.0\nend_header").is_err());
    }
}
