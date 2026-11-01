#!/usr/bin/env python3
"""Check host release LLVM control flow for explicitly expanded math kernels.

Run with `python3 tools/check_unrolled_codegen.py`. The repository root is
resolved from this script, independently of the working directory. Cargo
and rustc run sequentially in the normal target directory; probe artifacts use
a temporary directory. No assembly syntax, external Python packages, nightly
compiler features or runtime timing thresholds are required. The check covers
the concrete scalar/dimension combinations below on the host compiler; it does
not claim an identical instruction selection on every target or compiler.
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

from collections import deque
import json
from pathlib import Path
import re
import subprocess
import tempfile


def run(arguments, root):
    """Run one compiler command to completion, propagating any failure."""
    subprocess.run(arguments, cwd=root, check=True)


def functions(path):
    """Read LLVM function definitions and their basic-block successor graphs."""
    result = {}
    graph = None
    name = None
    block = "entry"
    for line in path.read_text().splitlines():
        if line.startswith("define "):
            name = re.search(r'@("[^"]+"|[^\s(]+)\(', line).group(1).strip('"')
            graph = {"entry": set()}
            block = "entry"
        elif graph is not None and line == "}":
            result[name] = graph
            graph = None
        elif graph is not None:
            label = re.match(r'^([A-Za-z0-9_.$-]+):', line)
            if label:
                block = label.group(1)
                graph.setdefault(block, set())
            graph[block].update(re.findall(r'label %([A-Za-z0-9_.$-]+)', line))
    return result


def assert_acyclic(name, graph):
    """Topologically remove all blocks; a remainder indicates a control loop.

    Backward jumps alone are insufficient evidence of a loop because compiler
    block layout can place an acyclic successor earlier in the output.
    """
    indegree = {block: 0 for block in graph}
    for successors in graph.values():
        for successor in successors:
            if successor not in graph:
                raise AssertionError(f"Unparsed LLVM block in {name}: {successor}")
            indegree[successor] += 1
    ready = deque(block for block, count in indegree.items() if count == 0)
    visited = 0
    while ready:
        block = ready.popleft()
        visited += 1
        for successor in graph[block]:
            indegree[successor] -= 1
            if indegree[successor] == 0:
                ready.append(successor)
    if visited != len(graph):
        raise AssertionError(f"Runtime control-flow cycle in {name}")


def main():
    """Compile public consumer probes and inspect them plus shared CORDIC code."""
    root = Path(__file__).resolve().parents[1]
    metadata = json.loads(subprocess.check_output(
        ["cargo", "metadata", "--offline", "--no-deps", "--format-version", "1"],
        cwd=root, text=True,
    ))
    target = Path(metadata["target_directory"]) / "release"
    run(["cargo", "rustc", "--offline", "--release", "--lib", "--", "--emit=llvm-ir,link"], root)
    library_ir = max((target / "deps").glob("rs_math3d-*.ll"), key=lambda p: p.stat().st_mtime_ns)
    source = ["use rs_math3d::{Fixed, MathError, Matrix2, Matrix3, Matrix4, Vector, Vector4};"]
    names = []
    for label, scalar in [("f32", "f32"), ("f64", "f64"),
                          ("fixed32", "Fixed<i32,8>"), ("fixed64", "Fixed<i64,16>")]:
        for dimension in (2, 3, 4):
            name = f"unroll_rows_{label}_{dimension}"
            matrix = f"Matrix{dimension}<{scalar}>"
            source.append(f"#[no_mangle] pub fn {name}(m: &{matrix}) -> Result<{matrix},MathError> {{ m.checked_inverse_rows() }}")
            names.append(name)
        name = f"unroll_dot_{label}"
        source.append(f"#[no_mangle] pub fn {name}(a: &Vector4<{scalar}>, b: &Vector4<{scalar}>) -> Result<{scalar},MathError> {{ Vector4::checked_dot(a,b) }}")
        names.append(name)
    with tempfile.TemporaryDirectory(prefix="rs-math3d-unroll-codegen-") as temporary:
        temporary = Path(temporary)
        probe = temporary / "probes.rs"
        probe.write_text("\n".join(source))
        output = temporary / "probes.ll"
        run(["rustc", "--crate-type", "lib", "--edition=2018", "--crate-name", "unroll_probes",
             "-C", "opt-level=3", "--emit=llvm-ir", "-o", str(output),
             "--extern", f"rs_math3d={target / 'librs_math3d.rlib'}",
             "-L", f"dependency={target / 'deps'}", str(probe)], root)
        graphs = functions(output)
        for name in names:
            if name not in graphs:
                raise AssertionError(f"Missing consumer probe: {name}")
            assert_acyclic(name, graphs[name])
        # A thin probe could otherwise hide a loop in an outlined matrix kernel.
        # Check every emitted dimension-specific helper as well as its callers.
        numeric_helpers = [(name, graph) for name, graph in graphs.items()
                          if any(marker in name for marker in (
                              "invert_rows2", "invert_rows3", "invert_rows4",
                              "checked_inverse_rows", "checked_dot", "sum_products_pair"))]
        for name, graph in numeric_helpers:
            assert_acyclic(name, graph)
        library_graphs = functions(library_ir)
        cordic_count = 0
        for helper in ("cordic_rotate", "cordic_vectoring"):
            matches = [(name, graph) for name, graph in library_graphs.items() if helper in name]
            if not matches:
                raise AssertionError(f"Missing shared CORDIC kernel: {helper}")
            for name, graph in matches:
                assert_acyclic(name, graph)
            cordic_count += len(matches)
        print(f"Verified {len(names)} consumer probes, {len(numeric_helpers)} outlined numeric helpers "
              f"and {cordic_count} shared CORDIC functions: no LLVM control-flow cycles.")


if __name__ == "__main__":
    main()
