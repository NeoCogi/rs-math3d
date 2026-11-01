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
