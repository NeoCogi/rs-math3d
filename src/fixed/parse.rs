//! Allocation-free exact decimal output and radix input. Fraction comparisons
//! use long division, so input length is not limited by a power-of-ten integer.

use super::{arithmetic::signed, Fixed, Format};
use crate::MathError;
use core::{cmp::Ordering, fmt};

/// Compares fractional input digits with `numerator / denominator` exactly.
/// Remainders remain below `2^64`, even for the widest supported format.
fn compare(digits: &str, radix: u32, mut numerator: u128, denominator: u128) -> Ordering {
    for ch in digits.chars() {
        numerator *= radix as u128;
        let expected = numerator / denominator;
        numerator %= denominator;
        let actual = ch.to_digit(radix).expect("validated fractional digit") as u128;
        match actual.cmp(&expected) {
            Ordering::Equal => (),
            order => return order,
        }
    }
    if numerator == 0 {
        Ordering::Equal
    } else {
        Ordering::Less
    }
}

/// Parses an arbitrary-length fraction with exact binary search and a final
/// midpoint comparison. Integer range is checked before any scaling/narrowing.
pub(super) fn parse<R: Format<F>, const F: u32>(
    text: &str,
    radix: u32,
) -> Result<Fixed<R, F>, MathError> {
    if !(2..=36).contains(&radix) || text.is_empty() {
        return Err(MathError::InvalidNumber);
    }
    let (negative, body) = if let Some(s) = text.strip_prefix('-') {
        (true, s)
    } else if let Some(s) = text.strip_prefix('+') {
        (false, s)
    } else {
        (false, text)
    };
    let mut pieces = body.split('.');
    let integer = pieces.next().unwrap_or("");
    let fraction = pieces.next().unwrap_or("");
    if pieces.next().is_some() || integer.is_empty() && fraction.is_empty() {
        return Err(MathError::InvalidNumber);
    }
    let mut whole = 0u128;
    for ch in integer.chars() {
        let digit = ch.to_digit(radix).ok_or(MathError::InvalidNumber)?;
        whole = whole
            .checked_mul(radix as u128)
            .and_then(|n| n.checked_add(digit as u128))
            .ok_or(MathError::Overflow)?;
    }
    for ch in fraction.chars() {
        ch.to_digit(radix).ok_or(MathError::InvalidNumber)?;
    }
    let scale = Fixed::<R, F>::scale() as u128;
    let base = whole.checked_mul(scale).ok_or(MathError::Overflow)?;
    let limit = if negative {
        R::MIN.unsigned_abs()
    } else {
        R::MAX as u128
    };
    if base > limit {
        return Err(MathError::Overflow);
    }
    let mut lo = 0;
    let mut hi = scale;
    while lo < hi {
        let mid = (lo + hi).div_ceil(2);
        if compare(fraction, radix, mid, scale) == Ordering::Less {
            hi = mid - 1;
        } else {
            lo = mid;
        }
    }
    let order = compare(fraction, radix, 2 * lo + 1, 2 * scale);
    let round_up = order == Ordering::Greater || order == Ordering::Equal && (base + lo) & 1 != 0;
    let raw = base
        .checked_add(lo)
        .and_then(|n| n.checked_add(round_up as u128))
        .ok_or(MathError::Overflow)?;
    Fixed::raw(signed(raw, negative)?)
}

/// Emits the exact terminating decimal expansion of the stored binary rational.
/// Precision flags do not truncate digits; this representation round-trips.
pub(super) fn display<R: Format<F>, const F: u32>(
    value: Fixed<R, F>,
    f: &mut fmt::Formatter<'_>,
) -> fmt::Result {
    let raw = value.to_bits().widen();
    if raw < 0 {
        f.write_str("-")?;
    }
    let magnitude = raw.unsigned_abs();
    let scale = Fixed::<R, F>::scale() as u128;
    write!(f, "{}", magnitude / scale)?;
    let mut remainder = magnitude % scale;
    if remainder != 0 {
        f.write_str(".")?;
    }
    while remainder != 0 {
        remainder *= 10;
        write!(f, "{}", remainder / scale)?;
        remainder %= scale;
    }
    Ok(())
}
