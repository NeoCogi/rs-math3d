#!/usr/bin/env python3
"""Compare current, branchless nearest-even and native-style fixed arithmetic.

This is an experiment, not a library configuration API. It copies the crate into
the output directory and changes only unchecked multiply/divide/sum-products at
asserted source sites. The native OBB envelope also handles asymmetric rounding.
Construction, checked arithmetic code and analytic kernels are not patched.
All copies share one sequential Cargo target directory.
Results and variant sources remain available; --keep-build also retains targets.

Requires the host Rust toolchain, Python 3 and Linux taskset; --perf additionally
requires perf and permission to count the selected user-space hardware events.
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
import csv
import json
import math
import os
from pathlib import Path
import re
import shutil
import subprocess
import tempfile


def replace_once(text, old, new):
    """Fail on changed source contracts instead of silently benchmarking a mix."""
    if text.count(old) != 1:
        raise ValueError(f"Expected exactly one arithmetic site: {old}")
    return text.replace(old, new)


def variant(root, destination, policy):
    """Own a source snapshot, linking unchanged hosted fixtures and docs."""
    destination.mkdir()
    for name in ("Cargo.toml", "Cargo.lock"):
        shutil.copy2(root / name, destination / name)
    shutil.copytree(root / "src", destination / "src")
    shutil.copytree(root / "examples", destination / "examples")
    (destination / "tools").mkdir()
    shutil.copy2(root / "tools/fixed_rounding_probe.rs", destination / "tools/fixed_rounding_probe.rs")
    for name in ("docs", "tests"):
        (destination / name).symlink_to(root / name, target_is_directory=True)
    if policy == "current":
        return
    module = destination / "src/fixed/mod.rs"
    text = module.read_text()
    nearest = policy == "nearest"
    text = replace_once(text,
        "Self::wrapping_raw(signed_wrapping(round_shift(sum.unsigned_abs(), F), sum < 0))",
        "Self::wrapping_raw(arithmetic::round_signed_wrapping(sum, F))" if nearest
        else "Self::wrapping_raw(sum >> F)")
    text = replace_once(text,
        "signed_wrapping(round_shift(product.unsigned_abs(), F), product < 0)",
        "arithmetic::round_signed_wrapping(product, F)" if nearest else "product >> F")
    text = replace_once(text, "round_shift, signed, signed_wrapping, Accumulator",
                        "round_shift, signed, Accumulator")
    if not nearest:
        text = replace_once(text, "|a,b| div(a * Self::scale(), b)",
                            "|a,b| (a * Self::scale()) / b")
    module.write_text(text)
    if not nearest:
        tree = destination / "examples/collision/aabb_tree.rs"
        text = tree.read_text()
        start = text.index("fn obb_bounds<T: RealScalar>")
        end = text.index("\n}\n", start) + 3
        text = text[:start] + '''fn obb_bounds<T: RealScalar>(obb: &Obb<T>) -> Box3<T> {
    // Directed rounding is not odd: the negative half axis cannot be recovered
    // by negating the rounded positive half axis. Bound both represented ends.
    let mut lo = obb.center;
    let mut hi = obb.center;
    for (axis, half) in obb.axes.iter().zip([obb.half.x, obb.half.y, obb.half.z]) {
        let a = *axis * half;
        let b = *axis * (-half);
        lo = lo + Vector3::new(T::min(a.x,b.x),T::min(a.y,b.y),T::min(a.z,b.z));
        hi = hi + Vector3::new(T::max(a.x,b.x),T::max(a.y,b.y),T::max(a.z,b.z));
    }
    Box3::new(&lo,&hi)
}
''' + text[end:]
        tree.write_text(text)
    if nearest:
        arithmetic = destination / "src/fixed/arithmetic.rs"
        arithmetic.write_text(arithmetic.read_text() + '''
/// Experimental branchless nearest-even for ordinary fixed products/sums.
/// All supported shifts are 0..63. Arithmetic floor and a nonnegative remainder
/// avoid abs/sign restoration and short-circuit rounding branches for both signs.
#[inline(always)]
pub(super) fn round_signed_wrapping(value: i128, shift: u32) -> i128 {
    if shift == 0 { return value; }
    let floor = value >> shift;
    let remainder = (value as u128) & ((1u128 << shift) - 1);
    let half = 1u128 << (shift - 1);
    let increment = (remainder + half - 1 + ((floor as u128) & 1)) >> shift;
    floor.wrapping_add(increment as i128)
}
''')


def run(command, log, root, env, check=True):
    """Run sequentially and retain diagnostics outside disposable Cargo targets."""
    with log.open("w") as output:
        result = subprocess.run(command, cwd=root, env=env, stdout=output,
                                stderr=subprocess.STDOUT)
    if check and result.returncode:
        raise RuntimeError(f"Command failed ({result.returncode}): {command}\nSee {log}")
    return result.returncode


def read_probe(path):
    """Parse exact-reference errors and individual geometric classifications."""
    errors, boundaries, queries, drift = {}, {}, {}, {}
    for row in csv.reader(path.read_text().splitlines()):
        if row[0] == "error":
            errors["/".join(row[1:3])] = dict(zip(
                ("count", "bias_quanta", "mae_quanta", "rms_quanta", "max_quanta"),
                [int(row[3])] + [float(x) for x in row[4:]]))
        elif row[0] == "boundary":
            key = "/".join(row[1:3])
            stats = boundaries.setdefault(key, {"count": 0, "false_positive": 0,
                                                 "false_negative": 0})
            expected, actual = row[6] == "true", row[7] == "true"
            stats["count"] += 1
            stats["false_positive"] += actual and not expected
            stats["false_negative"] += expected and not actual
        elif row[0] == "query":
            key = "/".join(row[1:5])
            assert key not in queries
            queries[key] = (row[5], float(row[6]) if row[6] else None)
        elif row[0] == "drift":
            drift[row[1]] = dict(zip(("positive", "negative", "reference_positive"),
                                    map(float, row[2:])))
        else:
            raise ValueError(f"Unrecognized probe record: {row}")
    assert len(queries) == 4 * 2 * (448 + 264 + 264)
    assert len(errors) == 12 and len(boundaries) == 4
    return {"errors": errors, "boundaries": boundaries, "drift": drift}, queries


def compare(actual, reference, cross_backend):
    """Compare classification, witnesses and ray parameters on matched hits."""
    groups = {}
    for key, (hit, parameter) in actual.items():
        backend, model, shape, index = key.split("/")
        if backend not in ("s15", "s31", "f32"):
            continue
        other = "/".join(("f64" if cross_backend else backend, model, shape, index))
        baseline, t = reference[other]
        stats = groups.setdefault("/".join((backend, model, shape)), {
            "count": 0, "hit_changes": 0, "witness_changes": 0,
            "paired_ray_hits": 0, "max_parameter_delta": 0.0,
            "parameter_square_sum": 0.0})
        stats["count"] += 1
        stats["hit_changes"] += (hit == "miss") != (baseline == "miss")
        if hit != "miss" and baseline != "miss":
            stats["witness_changes"] += hit != baseline
            if parameter is not None:
                difference = abs(parameter - t)
                stats["paired_ray_hits"] += 1
                stats["max_parameter_delta"] = max(stats["max_parameter_delta"], difference)
                stats["parameter_square_sum"] += difference * difference
    for stats in groups.values():
        count = stats["paired_ray_hits"]
        stats["rms_parameter_delta"] = math.sqrt(stats.pop("parameter_square_sum") / count) if count else 0.0
    return groups


def timings(path):
    """Extract tree medians/ranges; raw logs retain flat references and checks."""
    result, backend, model = {}, None, None
    for line in path.read_text().splitlines():
        if line in ("f32", "S15.16 (i32)", "S31.32 (i64)"):
            backend = {"f32": "f32", "S15.16 (i32)": "s15", "S31.32 (i64)": "s31"}[line]
        if line.startswith("Utah teapot:"):
            model = "teapot"
        elif line.startswith("Stanford bunny:"):
            model = "bunny"
        match = re.match(r"(tree (?:nearest ray|capsule/soup|OBB/soup|mesh/mesh))\s+([\d.]+) \[([\d.]+), ([\d.]+)\]", line)
        if match:
            query = match[1].removeprefix("tree ")
            key = "/".join((backend, "teapot_bunny" if query == "mesh/mesh" else model, query))
            result[key] = dict(zip(("median_ns", "min_ns", "max_ns"), map(float, match.groups()[1:])))
    assert len(result) == 21
    return result


def perf_events(cpu):
    """Select the pinned CPU's PMU on Intel hybrid hosts, avoiding the other
    core type's multiplexed counters. Generic events cover non-hybrid hosts.
    """
    names = ("cycles", "instructions", "branches", "branch-misses")
    for pmu in ("cpu_core", "cpu_atom"):
        path = Path("/sys/bus/event_source/devices") / pmu / "cpus"
        if not path.exists():
            continue
        for interval in path.read_text().strip().split(","):
            endpoints = [int(x) for x in interval.split("-")]
            if endpoints[0] <= cpu <= endpoints[-1]:
                return ",".join(f"{pmu}/{name}/u" for name in names)
    return ",".join(f"{name}:u" for name in names)


def main():
    """Build/run all policies, write machine-readable comparisons, clean targets."""
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--output", type=Path)
    parser.add_argument("--batches", type=int, default=200)
    parser.add_argument("--samples", type=int, default=7)
    parser.add_argument("--cpu", type=int, default=0)
    parser.add_argument("--perf", action="store_true")
    parser.add_argument("--keep-build", action="store_true")
    args = parser.parse_args()
    if args.batches <= 0 or args.samples <= 0:
        parser.error("batches and samples must be positive")
    root = Path(__file__).resolve().parent.parent
    output = (args.output or Path(tempfile.mkdtemp(prefix="math3d-rounding-"))).resolve()
    output.mkdir(parents=True, exist_ok=True)
    sources, target = output / "sources", output / "target"
    sources.mkdir()
    env = dict(os.environ, CARGO_TARGET_DIR=str(target), CARGO_PROFILE_RELEASE_DEBUG="2")
    if env.get("RUSTFLAGS") or env.get("CARGO_ENCODED_RUSTFLAGS"):
        parser.error("unset custom RUSTFLAGS for the controlled comparison")
    summary, results = {"config": vars(args) | {"output": str(output)}}, {}
    summary["config"]["output"] = str(output)
    summary["config"]["perf_events"] = perf_events(args.cpu) if args.perf else None
    summary["rustc"] = subprocess.check_output(["rustc", "-vV"], text=True)
    summary["revision"] = subprocess.check_output(["git", "rev-parse", "HEAD"], cwd=root, text=True).strip()
    for policy in ("current", "nearest", "native"):
        print(f"{policy}: building and validating", flush=True)
        source = sources / policy
        variant(root, source, policy)
        cargo = ["cargo", "--offline"]
        run(cargo + ["build", "--release", "--manifest-path", str(source / "Cargo.toml"),
                     "--lib", "--example", "mesh_bench"], output / f"{policy}-build.log", root, env)
        libraries = list((target / "release/deps").glob("librs_math3d-*.rlib"))
        assert len(libraries) == 1
        compiler = ["rustc", "--edition=2018", "-C", "opt-level=3", "-C", "debuginfo=2",
                    "--cfg", 'feature="std"', "--extern", f"rs_math3d={libraries[0]}",
                    "-L", f"dependency={target / 'release/deps'}", str(source / "tools/fixed_rounding_probe.rs")]
        if policy == "native":
            compiler += ["--cfg", "rounding_native"]
        probe = output / f"{policy}-probe"
        tests = output / f"{policy}-tests"
        run(compiler + ["-o", str(probe)], output / f"{policy}-probe-build.log", root, env)
        run(compiler + ["--test", "-o", str(tests)], output / f"{policy}-tests-build.log", root, env)
        passed = run([str(tests)], output / f"{policy}-tests.log", root, env, check=False) == 0
        if policy != "native":
            run(cargo + ["test", "--release", "--manifest-path", str(source / "Cargo.toml"), "--lib"],
                output / f"{policy}-unit-tests.log", root, env)
        run([str(probe)], output / f"{policy}-precision.csv", root, env)
        bench = output / f"{policy}-mesh_bench"
        shutil.copy2(target / "release/examples/mesh_bench", bench)
        print(f"{policy}: pinned timing and precision", flush=True)
        run(["taskset", "-c", str(args.cpu), str(bench), str(args.batches), str(args.samples)],
            output / f"{policy}-mesh.log", root, env)
        data, results[policy] = read_probe(output / f"{policy}-precision.csv")
        summary[policy] = data | {"contract_tests_pass": passed, "timings": timings(output / f"{policy}-mesh.log")}
        if args.perf:
            for scalar in ("f32", "s15", "s31"):
                for shape in ("capsule", "obb"):
                    tag = f"{policy}-{scalar}-{shape}"
                    events = perf_events(args.cpu)
                    run(["perf", "stat", "-x,", "-e", events, "-o", str(output / f"{tag}.stat"),
                         "--", "taskset", "-c", str(args.cpu), str(probe), "profile", scalar, shape, "10000"],
                        output / f"{tag}.perf.log", root, env)
    for policy in ("current", "nearest", "native"):
        summary[policy]["vs_f64"] = compare(results[policy], results[policy], True)
        summary[policy]["vs_nearest"] = compare(results[policy], results["nearest"], False)
    (output / "comparison.json").write_text(json.dumps(summary, indent=2) + "\n")
    if not args.keep_build:
        run(["cargo", "clean", "--manifest-path", str(sources / "native/Cargo.toml")], output / "clean.log", root, env)
    print(f"Results: {output / 'comparison.json'}", flush=True)
    if not all(summary[p]["contract_tests_pass"] for p in ("current", "nearest", "native")):
        print("Some contract/geometry tests failed; inspect policy test logs before adopting a policy.", flush=True)
        raise SystemExit(1)


if __name__ == "__main__":
    main()
