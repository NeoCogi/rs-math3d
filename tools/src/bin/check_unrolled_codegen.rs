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

//! Compile public math consumers and audit LLVM control-flow graphs. The check
//! covers explicit host specializations, not a target-independent codegen claim.

use rs_math3d_tools::{checked, root, target_directory, Scratch};
use std::{
    collections::{BTreeMap, BTreeSet, VecDeque},
    fs, io,
    path::Path,
    process::Command,
};

/// One LLVM definition and the successors of each explicitly named basic block.
#[derive(Default)]
struct Function {
    /// Complete definition text for call reachability and operand inspection.
    body: String,
    /// Named basic blocks and their branch/switch successors.
    blocks: BTreeMap<String, BTreeSet<String>>,
}

/// Reads quoted and unquoted LLVM global symbols without guessing mangling hashes.
fn symbols(line: &str) -> Vec<String> {
    let mut rest = line;
    let mut result = Vec::new();
    while let Some((_, tail)) = rest.split_once('@') {
        if let Some(tail) = tail.strip_prefix('"') {
            let end = tail.find('"').expect("LLVM symbol quote");
            result.push(tail[..end].to_owned());
            rest = &tail[end + 1..];
        } else {
            let end = tail
                .find(|c: char| c.is_whitespace() || "(,)=[]".contains(c))
                .unwrap_or(tail.len());
            result.push(tail[..end].to_owned());
            rest = &tail[end..];
        }
    }
    result
}

/// Parses definitions, branch/switch destinations and function aliases. A jump
/// laid out backwards is legal; only cycles in the successor graph are rejected.
fn parse(text: &str) -> (BTreeMap<String, Function>, BTreeMap<String, String>) {
    let (mut functions, mut aliases) = (BTreeMap::new(), BTreeMap::new());
    let mut current: Option<(String, Function, String)> = None;
    for line in text.lines() {
        if line.starts_with("define ") {
            let name = symbols(line).remove(0);
            let mut f = Function::default();
            f.body.push_str(line);
            f.body.push('\n');
            f.blocks.insert("entry".to_owned(), BTreeSet::new());
            current = Some((name, f, "entry".to_owned()));
        } else if line == "}" && current.is_some() {
            let (name, mut f, _) = current.take().unwrap();
            f.body.push_str("}\n");
            functions.insert(name, f);
        } else if let Some((_, f, block)) = &mut current {
            f.body.push_str(line);
            f.body.push('\n');
            let trimmed = line.trim_start();
            if let Some((label, _)) = trimmed.split_once(':') {
                if !label.is_empty()
                    && label
                        .chars()
                        .all(|c| c.is_ascii_alphanumeric() || "_.$-".contains(c))
                {
                    *block = label.to_owned();
                    f.blocks.entry(block.clone()).or_default();
                }
            }
            let mut rest = line;
            while let Some((_, tail)) = rest.split_once("label %") {
                let end = tail
                    .find(|c: char| !c.is_ascii_alphanumeric() && !"_.$-".contains(c))
                    .unwrap_or(tail.len());
                f.blocks
                    .get_mut(block)
                    .unwrap()
                    .insert(tail[..end].to_owned());
                rest = &tail[end..];
            }
        } else if line.starts_with('@') && line.contains(" alias ") {
            let names = symbols(line);
            aliases.insert(names[0].clone(), names.last().unwrap().clone());
        }
    }
    assert!(current.is_none(), "unterminated LLVM function");
    (functions, aliases)
}

/// Topologically removes all blocks; unresolved successors and cycles fail closed.
fn acyclic(name: &str, f: &Function) {
    let mut degrees: BTreeMap<_, usize> = f.blocks.keys().map(|k| (k.clone(), 0)).collect();
    for edges in f.blocks.values() {
        for edge in edges {
            *degrees
                .get_mut(edge)
                .unwrap_or_else(|| panic!("unparsed label {edge} in {name}")) += 1;
        }
    }
    let mut queue: VecDeque<_> = degrees
        .iter()
        .filter(|(_, v)| **v == 0)
        .map(|(k, _)| k.clone())
        .collect();
    let mut visited = 0;
    while let Some(block) = queue.pop_front() {
        visited += 1;
        for edge in &f.blocks[&block] {
            let count = degrees.get_mut(edge).unwrap();
            *count -= 1;
            if *count == 0 {
                queue.push_back(edge.clone());
            }
        }
    }
    assert_eq!(
        visited,
        degrees.len(),
        "runtime control-flow cycle in {name}"
    );
}

/// Finds only numeric rotation/vectoring chunks reached by the selected width.
fn chunks(
    roots: &[String],
    fs: &BTreeMap<String, Function>,
    aliases: &BTreeMap<String, String>,
) -> BTreeSet<String> {
    let mut pending = roots.to_vec();
    let mut visited = BTreeSet::new();
    let mut result = BTreeSet::new();
    while let Some(name) = pending.pop() {
        if !visited.insert(name.clone()) {
            continue;
        }
        if let Some(alias) = aliases.get(&name) {
            pending.push(alias.clone());
        } else if let Some(f) = fs.get(&name) {
            if name.contains("cordic_rotate_chunk") || name.contains("cordic_vectoring_chunk") {
                result.insert(name.clone());
            }
            for line in f
                .body
                .lines()
                .filter(|l| l.contains("call ") || l.contains("invoke "))
            {
                pending.extend(symbols(line));
            }
        }
    }
    result
}

/// Checks the arithmetic operand type, not incidental i128 text in SSA names.
fn wide_arithmetic(body: &str) -> bool {
    let operations = [
        "add", "sub", "mul", "ashr", "shl", "lshr", "sdiv", "udiv", "srem", "urem",
    ];
    body.lines().any(|line| {
        let tokens: Vec<_> = line.split_whitespace().collect();
        tokens.iter().enumerate().any(|(i, op)| {
            operations.contains(op)
                && tokens[i + 1..]
                    .iter()
                    .skip_while(|x| ["nuw", "nsw", "exact"].contains(x))
                    .next()
                    == Some(&"i128")
        })
    })
}

/// Chooses a newly emitted library IR file, without relying on artifact hashes.
fn latest_ir(path: &Path) -> io::Result<std::path::PathBuf> {
    fs::read_dir(path)?
        .filter_map(Result::ok)
        .filter(|e| {
            e.file_name().to_string_lossy().starts_with("rs_math3d-")
                && e.path().extension().is_some_and(|x| x == "ll")
        })
        .max_by_key(|e| e.metadata().and_then(|m| m.modified()).ok())
        .map(|e| e.path())
        .ok_or_else(|| io::Error::other("missing library IR"))
}

/// Builds probes sequentially in the root target and discards only owned scratch.
fn main() -> io::Result<()> {
    let root = root();
    let target = target_directory(&root)?.join("release");
    checked(Command::new("cargo").current_dir(&root).args([
        "rustc",
        "--offline",
        "--release",
        "--lib",
        "--",
        "--emit=llvm-ir,link",
    ]))?;
    let library = latest_ir(&target.join("deps"))?;
    let mut source = String::from(
        "use rs_math3d::{Fixed, MathError, Matrix2, Matrix3, Matrix4, Vector, Vector4};\n",
    );
    let (mut names, mut narrow, mut wide) = (Vec::new(), Vec::new(), Vec::new());
    for (label, scalar) in [
        ("f32", "f32"),
        ("f64", "f64"),
        ("fixed32", "Fixed<i32,8>"),
        ("fixed64", "Fixed<i64,16>"),
    ] {
        for checked in [false, true] {
            let prefix = if checked { "checked_" } else { "" };
            for dim in 2..=4 {
                let matrix = format!("Matrix{dim}<{scalar}>");
                let result = if checked {
                    format!("Result<{matrix},MathError>")
                } else {
                    matrix.clone()
                };
                for mode in ["rows", "reciprocal"] {
                    let name = format!("unroll_{prefix}{mode}_{label}_{dim}");
                    source+=&format!("#[no_mangle] pub fn {name}(m:&{matrix})->{result} {{m.{prefix}inverse_{mode}()}}\n");
                    names.push(name);
                }
            }
            let name = format!("unroll_{prefix}dot_{label}");
            let result = if checked {
                format!("Result<{scalar},MathError>")
            } else {
                scalar.to_owned()
            };
            source+=&format!("#[no_mangle] pub fn {name}(a:&Vector4<{scalar}>,b:&Vector4<{scalar}>)->{result} {{Vector4::{prefix}dot(a,b)}}\n");
            names.push(name);
        }
    }
    for (width, fractions) in [(64, &[8, 16, 29, 32, 44, 48][..]), (128, &[16, 48, 61][..])] {
        for frac in fractions {
            let scalar = format!("Fixed<i64,{frac}>");
            let name = format!("unroll_cordic{width}_f{frac}");
            source+=&format!("#[no_mangle] pub fn {name}(x:{scalar})->({scalar},{scalar}) {{x.sin_cos_with::<{width}>()}}\n");
            names.push(name.clone());
            let roots = if width == 64 { &mut narrow } else { &mut wide };
            roots.push(name);
            let name = format!("cordic{width}_acos_f{frac}");
            source += &format!(
                "#[no_mangle] pub fn {name}(x:{scalar})->{scalar} {{x.acos_with::<{width}>()}}\n"
            );
            roots.push(name);
        }
    }
    let scratch = Scratch::new("math3d-unroll")?;
    let probe = scratch.path().join("probes.rs");
    let output = scratch.path().join("probes.ll");
    fs::write(&probe, source)?;
    checked(
        Command::new("rustc")
            .current_dir(&root)
            .args([
                "--crate-type",
                "lib",
                "--edition=2018",
                "--crate-name",
                "unroll_probes",
                "-C",
                "opt-level=3",
                "--emit=llvm-ir",
                "-o",
            ])
            .arg(&output)
            .arg("--extern")
            .arg(format!(
                "rs_math3d={}",
                target.join("librs_math3d.rlib").display()
            ))
            .arg("-L")
            .arg(format!("dependency={}", target.join("deps").display()))
            .arg(probe),
    )?;
    let (mut functions, mut aliases) = parse(&fs::read_to_string(output)?);
    for name in &names {
        acyclic(
            name,
            functions
                .get(name)
                .unwrap_or_else(|| panic!("missing probe {name}")),
        );
    }
    let markers = [
        "invert_rows2",
        "invert_rows3",
        "invert_rows4",
        "inverse_rows",
        "inverse_reciprocal",
        "checked_dot",
        "sum_products_pair",
    ];
    let mut helpers = 0;
    for (name, f) in &functions {
        if markers.iter().any(|m| name.contains(m)) {
            acyclic(name, f);
            helpers += 1;
        }
    }
    let (library, library_aliases) = parse(&fs::read_to_string(library)?);
    functions.extend(library);
    aliases.extend(library_aliases);
    let narrow = chunks(&narrow, &functions, &aliases);
    let wide = chunks(&wide, &functions, &aliases);
    assert!(narrow.len() >= 6, "missing specialized narrow chunks");
    assert_eq!(wide.len(), 14, "Q96 requires seven chunks per operation");
    assert!(narrow.is_disjoint(&wide));
    for name in &narrow {
        acyclic(name, &functions[name]);
        assert!(
            !wide_arithmetic(&functions[name].body),
            "i128 arithmetic in narrow {name}"
        );
    }
    for name in &wide {
        acyclic(name, &functions[name]);
        assert!(
            wide_arithmetic(&functions[name].body),
            "missing wide arithmetic"
        );
    }
    println!("Verified {} consumers, {helpers} outlined helpers, {} shared wide chunks and {} narrow chunks: acyclic; narrow chunks contain no i128 arithmetic.",names.len(),wide.len(),narrow.len());
    Ok(())
}

#[cfg(test)]
mod tests {
    use super::*;
    #[test]
    fn graph_and_operand_parsing() {
        let(f,a)=parse("define i32 @f() {\nstart:\n br label %later\nlater:\n ret i32 1\n}\n@g = alias i32 (), ptr @f\n");
        acyclic("f", &f["f"]);
        assert_eq!(a["g"], "f");
        assert!(wide_arithmetic("%x = add nuw i128 %a, %b"));
        assert!(!wide_arithmetic("%name.i128 = add i64 %a, %b"));
    }
    #[test]
    #[should_panic(expected = "runtime control-flow cycle")]
    fn detects_actual_cycle() {
        let (f, _) = parse("define void @f() {\nstart:\n br label %start\n}\n");
        acyclic("f", &f["f"]);
    }
}
