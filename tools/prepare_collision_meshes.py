#!/usr/bin/env python3
"""Prepare pinned bunny/Utah teapot fixtures using only Python's standard library.

Run with --download, or supply --bunny-archive and --teapot-obj local sources.
Each model is centered, uniformly scaled to a longest extent of 32, and snapped
to a 1/256 grid. Exact integer cross products remove collapsed faces once, so
every benchmark backend receives the same vertices and triangle ordering.
"""

# Copyright 2020-Present (c) Raja Lehtihet & Wael El Oraiby
#
# Redistribution and use in source and binary forms, with or without
# modification, are permitted provided that the following conditions are met:
#
# 1. Redistributions of source code must retain the above copyright notice,
# this list of conditions and the following disclaimer.
#
# 2. Redistributions in binary form must reproduce the above copyright notice,
# this list of conditions and the following disclaimer in the documentation
# and/or other materials provided with the distribution.
#
# 3. Neither the name of the copyright holder nor the names of its contributors
# may be used to endorse or promote products derived from this software without
# specific prior written permission.
#
# THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS "AS IS"
# AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE
# IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE
# ARE DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT HOLDER OR CONTRIBUTORS BE
# LIABLE FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR
# CONSEQUENTIAL DAMAGES (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF
# SUBSTITUTE GOODS OR SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS
# INTERRUPTION) HOWEVER CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN
# CONTRACT, STRICT LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE)
# ARISING IN ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE
# POSSIBILITY OF SUCH DAMAGE.

import argparse
import hashlib
import json
from pathlib import Path
import struct
import tarfile
import tempfile
import urllib.request


SOURCES = {
    "bunny": ("https://graphics.stanford.edu/pub/3Dscanrep/bunny.tar.gz",
              "a5720bd96d158df403d153381b8411a727a1d73cff2f33dc9b212d6f75455b84"),
    "teapot": ("https://graphics.stanford.edu/courses/cs148-10-summer/as3/code/as3/teapot.obj",
               "1b5396fedd74b577e32cef41146582c2f2e1a050d5b4915193c0ac1ad4187ed4"),
}


def read_bunny(path):
    """Read only the low-resolution reconstruction member, without extraction."""
    with tarfile.open(path) as archive:
        member = archive.extractfile("bunny/reconstruction/bun_zipper_res3.ply")
        lines = member.read().decode("ascii").splitlines()
    end = lines.index("end_header") + 1
    vertices = next(int(s.split()[2]) for s in lines[:end] if s.startswith("element vertex"))
    faces = next(int(s.split()[2]) for s in lines[:end] if s.startswith("element face"))
    points = [tuple(map(float, s.split()[:3])) for s in lines[end:end + vertices]]
    cells = [list(map(int, s.split())) for s in lines[end + vertices:end + vertices + faces]]
    assert all(cell[0] == 3 and len(cell) == 4 for cell in cells)
    return points, [tuple(cell[1:]) for cell in cells]


def read_teapot(path):
    """Read the pinned all-triangle OBJ; reject unexpected polygonal faces."""
    points, faces = [], []
    for line in path.read_text().splitlines():
        cells = line.split()
        if not cells:
            continue
        if cells[0] == "v":
            points.append(tuple(map(float, cells[1:4])))
        elif cells[0] == "f":
            assert len(cells) == 4
            faces.append(tuple(int(s.split("/")[0]) - 1 for s in cells[1:]))
    return points, faces


def encode(points, faces):
    """Encode little-endian signed Q8 vertices and unsigned 16-bit indices."""
    minimum = [min(p[i] for p in points) for i in range(3)]
    maximum = [max(p[i] for p in points) for i in range(3)]
    center = [(a + b) / 2 for a, b in zip(minimum, maximum)]
    scale = 32 / max(b - a for a, b in zip(minimum, maximum))
    points = [tuple(round((p[i] - center[i]) * scale * 256) for i in range(3)) for p in points]
    assert len(points) < 65536
    kept = []
    for face in faces:
        assert all(0 <= i < len(points) for i in face)
        a, b, c = (points[i] for i in face)
        u, v = [b[i] - a[i] for i in range(3)], [c[i] - a[i] for i in range(3)]
        cross = (u[1]*v[2]-u[2]*v[1], u[2]*v[0]-u[0]*v[2], u[0]*v[1]-u[1]*v[0])
        if cross != (0, 0, 0):
            kept.append(face)
    data = bytearray(b"M3SOUP01")
    data += struct.pack("<II", len(points), len(kept))
    for point in points:
        data += struct.pack("<hhh", *point)
    for face in kept:
        data += struct.pack("<HHH", *face)
    return bytes(data), {"vertices": len(points), "source_triangles": len(faces),
                         "triangles": len(kept), "collapsed_faces_removed": len(faces) - len(kept),
                         "source_center": center, "source_scale": scale}


def main():
    """Verify source hashes before writing deterministic, offline fixtures."""
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--download", action="store_true")
    parser.add_argument("--bunny-archive", type=Path)
    parser.add_argument("--teapot-obj", type=Path)
    args = parser.parse_args()
    destination = Path(__file__).resolve().parents[1] / "examples/collision/assets"
    destination.mkdir(parents=True, exist_ok=True)
    manifest = {"format": "M3SOUP01", "grid": "1/256", "longest_extent": 32, "models": {}}
    with tempfile.TemporaryDirectory(prefix="math3d-mesh-sources-") as temporary:
        for name, reader, supplied in (("bunny", read_bunny, args.bunny_archive),
                                       ("teapot", read_teapot, args.teapot_obj)):
            url, expected = SOURCES[name]
            if args.download:
                source = Path(temporary) / name
                source.write_bytes(urllib.request.urlopen(url, timeout=60).read())
            elif supplied is not None:
                source = supplied
            else:
                parser.error("use --download or supply both local source paths")
            digest = hashlib.sha256(source.read_bytes()).hexdigest()
            if digest != expected:
                raise ValueError(f"{name} source SHA256 mismatch: {digest}")
            data, metadata = encode(*reader(source))
            (destination / f"{name}.soup").write_bytes(data)
            metadata.update(source_url=url, source_sha256=expected,
                            fixture_sha256=hashlib.sha256(data).hexdigest())
            manifest["models"][name] = metadata
            print(name, metadata["triangles"], "triangles;", metadata["collapsed_faces_removed"], "collapsed faces removed")
    (destination / "manifest.json").write_text(json.dumps(manifest, indent=2, sort_keys=True) + "\n")


if __name__ == "__main__":
    main()
