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

//! Errors shared by checked scalar and geometry operations.

use core::fmt;

/// A calculation could not produce a valid, representable mathematical value.
#[derive(Clone, Copy, Debug, Eq, PartialEq)]
pub enum MathError {
    /// A result or required intermediate exceeds its representation's range.
    Overflow,
    /// A divisor is exactly zero.
    DivisionByZero,
    /// An analytic input lies outside the function's domain.
    Domain,
    /// A floating-point input or result is NaN or infinity.
    NonFinite,
    /// A tolerance is negative, non-finite, or outside its documented range.
    InvalidTolerance,
    /// The input does not define a unique result, such as a zero direction.
    Degenerate,
    /// Text is empty or contains an invalid digit, sign, or radix point.
    InvalidNumber,
}

impl fmt::Display for MathError {
    fn fmt(&self, f: &mut fmt::Formatter<'_>) -> fmt::Result {
        f.write_str(match self {
            Self::Overflow => "numeric range exceeded",
            Self::DivisionByZero => "division by zero",
            Self::Domain => "input outside mathematical domain",
            Self::NonFinite => "non-finite numeric value",
            Self::InvalidTolerance => "invalid tolerance",
            Self::Degenerate => "degenerate mathematical input",
            Self::InvalidNumber => "invalid numeric text",
        })
    }
}

impl core::error::Error for MathError {}
