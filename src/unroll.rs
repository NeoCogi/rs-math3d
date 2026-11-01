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

//! Explicit expansion of statically bounded numerical kernels.
//!
//! These private macros emit ordinary statements, including in debug builds;
//! they do not depend on an optimizer's loop-unrolling heuristics. Expansion
//! preserves left-to-right evaluation and propagates `?`/`return` in the caller.
//! Input-length and convergence loops keep their runtime termination rules.

/// Repeats a statement block for an ordered list of literal indices.
///
/// Each index has its own scope. Nested expansions may use independent index
/// names. Callers express conditional skipping with `if`, rather than `continue`;
/// an explicit outer block label can provide an early exit across all steps.
macro_rules! unroll_indices {
    ($index:ident in [$($value:literal),* $(,)?] $body:block) => {{
        $({
            let $index: usize = $value;
            $body;
        })*
    }};
}

/// Expands scalar slices of lengths zero through four without an iteration
/// loop, covering every vector/matrix component or product count in this crate.
///
/// Public scalar APIs also accept arbitrary-length slices; their fallback
/// remains a runtime loop. The input expression is evaluated exactly once.
/// Bindings borrow each element just as a slice iterator would, and both paths
/// retain identical evaluation/error order without allocating or copying data.
macro_rules! unroll_short_slice {
    ($items:expr, $binding:pat => $body:block) => {{
        let __unroll_slice: &[_] = $items;
        match __unroll_slice.len() {
            0 => (),
            1 => unroll_indices!(__unroll_index in [0] {
                let $binding = &__unroll_slice[__unroll_index];
                $body;
            }),
            2 => unroll_indices!(__unroll_index in [0, 1] {
                let $binding = &__unroll_slice[__unroll_index];
                $body;
            }),
            3 => unroll_indices!(__unroll_index in [0, 1, 2] {
                let $binding = &__unroll_slice[__unroll_index];
                $body;
            }),
            4 => unroll_indices!(__unroll_index in [0, 1, 2, 3] {
                let $binding = &__unroll_slice[__unroll_index];
                $body;
            }),
            _ => {
                for $binding in __unroll_slice {
                    $body;
                }
            }
        }
    }};
}
