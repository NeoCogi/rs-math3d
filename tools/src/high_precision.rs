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

//! Small owned binding to GMP's mpf API for independent developer references.
//! The library itself never links GMP. The Unix ABI follows gmp.h: two int limb
//! counts, a long exponent and an unsigned-long limb pointer. Precision is at
//! least 512 bits, with far more guard precision than the stored 36-digit residuals.

use std::{
    cmp::Ordering,
    ffi::{c_char, c_int, c_long, c_ulong, CStr, CString},
    mem::MaybeUninit,
    ops::{Add, Div, Mul, Neg, Sub},
};

/// GMP-owned mantissa metadata. Only GMP initializes/mutates/frees its limbs.
#[repr(C)]
struct Raw {
    /// Allocated precision in limbs, as defined by gmp.h.
    precision: c_int,
    /// Signed count of occupied mantissa limbs.
    size: c_int,
    /// Exponent in radix 2^limb_bits.
    exponent: c_long,
    /// Allocation owned exclusively by this initialized mpf instance.
    limbs: *mut c_ulong,
}

#[link(name = "gmp")]
unsafe extern "C" {
    fn __gmpf_init2(x: *mut Raw, bits: c_ulong);
    fn __gmpf_clear(x: *mut Raw);
    fn __gmpf_set(x: *mut Raw, y: *const Raw);
    fn __gmpf_set_str(x: *mut Raw, text: *const c_char, base: c_int) -> c_int;
    fn __gmpf_get_str(
        text: *mut c_char,
        exponent: *mut c_long,
        base: c_int,
        digits: usize,
        x: *const Raw,
    ) -> *mut c_char;
    fn __gmpf_add(x: *mut Raw, a: *const Raw, b: *const Raw);
    fn __gmpf_sub(x: *mut Raw, a: *const Raw, b: *const Raw);
    fn __gmpf_mul(x: *mut Raw, a: *const Raw, b: *const Raw);
    fn __gmpf_div(x: *mut Raw, a: *const Raw, b: *const Raw);
    fn __gmpf_neg(x: *mut Raw, a: *const Raw);
    fn __gmpf_abs(x: *mut Raw, a: *const Raw);
    fn __gmpf_sqrt(x: *mut Raw, a: *const Raw);
    fn __gmpf_trunc(x: *mut Raw, a: *const Raw);
    fn __gmpf_cmp(a: *const Raw, b: *const Raw) -> c_int;
    fn __gmpf_mul_2exp(x: *mut Raw, a: *const Raw, n: c_ulong);
    fn __gmpf_div_2exp(x: *mut Raw, a: *const Raw, n: c_ulong);
}

/// One initialized GMP value. Arithmetic allocates a distinct output, borrowing
/// initialized operands; Drop clears exactly the allocation initialized here.
pub struct Float {
    /// Private live GMP allocation; never copied by value.
    raw: Raw,
    /// Requested precision used when allocating operation results.
    bits: u32,
}
impl Float {
    /// Initializes enough mantissa space. The supported precision bound also
    /// bounds get_str's caller-owned output buffer, including GMP's guard limb.
    fn zero(bits: u32) -> Self {
        assert!((512..=4096).contains(&bits));
        let mut raw = MaybeUninit::uninit();
        // SAFETY: GMP initializes all fields and owns its newly allocated limbs.
        unsafe {
            __gmpf_init2(raw.as_mut_ptr(), bits as c_ulong);
            Self {
                raw: raw.assume_init(),
                bits,
            }
        }
    }
    /// Sets an exact signed integer through a NUL-terminated decimal encoding.
    pub fn int(value: i128, bits: u32) -> Self {
        let mut result = Self::zero(bits);
        let text = CString::new(value.to_string()).unwrap();
        // SAFETY: output is initialized and the input is a live valid C string.
        assert_eq!(
            unsafe { __gmpf_set_str(&mut result.raw, text.as_ptr(), 10) },
            0
        );
        result
    }
    /// Exact power-of-two scaling; both operands have initialized owned storage.
    pub fn scale(&self, exponent: i32) -> Self {
        let mut result = Self::zero(self.bits);
        // SAFETY: distinct initialized output, live borrowed input; exponent fits.
        unsafe {
            if exponent >= 0 {
                __gmpf_mul_2exp(&mut result.raw, &self.raw, exponent as c_ulong)
            } else {
                __gmpf_div_2exp(&mut result.raw, &self.raw, (-exponent) as c_ulong)
            }
        }
        result
    }
    /// Absolute value, preserving the requested reference precision.
    pub fn abs(&self) -> Self {
        let mut r = Self::zero(self.bits);
        // SAFETY: initialized distinct output and live borrowed input.
        unsafe { __gmpf_abs(&mut r.raw, &self.raw) }
        r
    }
    /// Positive square root. The caller must supply a nonnegative reference.
    pub fn sqrt(&self) -> Self {
        assert!(self >= &Self::int(0, self.bits));
        let mut r = Self::zero(self.bits);
        // SAFETY: initialized output/input, validated nonnegative radicand.
        unsafe { __gmpf_sqrt(&mut r.raw, &self.raw) }
        r
    }
    /// Converts a bounded integral part without losing i128 bits through f64.
    pub fn trunc_i128(&self) -> i128 {
        let mut integral = Self::zero(self.bits);
        // SAFETY: distinct initialized output and borrowed initialized input.
        unsafe { __gmpf_trunc(&mut integral.raw, &self.raw) }
        let (digits, exponent) = integral.digits();
        if digits.is_empty() {
            return 0;
        }
        let negative = digits.starts_with('-');
        let digits = digits.trim_start_matches('-');
        let mut text = digits.to_owned();
        if exponent >= text.len() as i64 {
            text.push_str(&"0".repeat(exponent as usize - text.len()));
        } else {
            text.truncate(exponent.max(0) as usize);
        }
        if text.is_empty() {
            return 0;
        }
        if negative {
            text.insert(0, '-');
        }
        text.parse().expect("reference integer fits i128")
    }
    /// Nearest-even reference quantization, independent of the library's kernels.
    pub fn nearest_i128(&self) -> i128 {
        let integer = self.trunc_i128();
        let residual = (self - &Self::int(integer, self.bits)).abs();
        let half = Self::int(1, self.bits).scale(-1);
        let increment = residual > half || (residual == half && integer & 1 != 0);
        if increment {
            integer
                + if self < &Self::int(0, self.bits) {
                    -1
                } else {
                    1
                }
        } else {
            integer
        }
    }
    /// Returns GMP's mantissa digits and decimal exponent in a caller-owned
    /// buffer. At <=4096 bits, at most 1300 decimal digits plus sign/NUL are used.
    fn digits(&self) -> (String, i64) {
        let mut buffer = [0 as c_char; 2048];
        let mut exponent = 0;
        // SAFETY: buffer exceeds max supported mantissa decimal length; GMP
        // writes a NUL terminator. No GMP-owned result allocation is requested.
        let text = unsafe { __gmpf_get_str(buffer.as_mut_ptr(), &mut exponent, 10, 0, &self.raw) };
        assert_eq!(text, buffer.as_mut_ptr());
        // SAFETY: GMP returned the caller-owned buffer with a NUL terminator.
        (
            unsafe { CStr::from_ptr(text) }.to_str().unwrap().to_owned(),
            exponent as i64,
        )
    }
    /// Formats a residual with 36 places after the leading significant digit.
    /// An explicit decimal guard digit gives stable nearest-even text at 512/768 bits.
    pub fn scientific(&self) -> String {
        let (text, mut exponent) = self.digits();
        if text.is_empty() {
            return format!("0.{}E+0", "0".repeat(36));
        }
        let negative = text.starts_with('-');
        let mut digits = text.trim_start_matches('-').as_bytes().to_vec();
        digits.resize(digits.len().max(39), b'0');
        let up = digits[37] > b'5'
            || (digits[37] == b'5'
                && (digits[38..].iter().any(|x| *x != b'0') || (digits[36] - b'0') & 1 != 0));
        digits.truncate(37);
        if up {
            let mut carry = true;
            for digit in digits.iter_mut().rev() {
                if *digit == b'9' {
                    *digit = b'0';
                } else {
                    *digit += 1;
                    carry = false;
                    break;
                }
            }
            if carry {
                digits[0] = b'1';
                exponent += 1;
            }
        }
        let text = String::from_utf8(digits).unwrap();
        format!(
            "{}{}.{}E{:+}",
            if negative { "-" } else { "" },
            &text[..1],
            &text[1..],
            exponent - 1
        )
    }
}
impl Clone for Float {
    fn clone(&self) -> Self {
        let mut r = Self::zero(self.bits);
        // SAFETY: both initialized; this copies the value, never its limb owner.
        unsafe { __gmpf_set(&mut r.raw, &self.raw) }
        r
    }
}
impl Drop for Float {
    fn drop(&mut self) {
        // SAFETY: exactly the live allocation created by this owner's init2.
        unsafe { __gmpf_clear(&mut self.raw) }
    }
}
impl PartialEq for Float {
    fn eq(&self, other: &Self) -> bool {
        // SAFETY: both operands remain initialized and immutably borrowed.
        unsafe { __gmpf_cmp(&self.raw, &other.raw) == 0 }
    }
}
impl PartialOrd for Float {
    fn partial_cmp(&self, other: &Self) -> Option<Ordering> {
        // SAFETY: both operands remain initialized and immutably borrowed.
        Some(unsafe { __gmpf_cmp(&self.raw, &other.raw) }.cmp(&0))
    }
}
macro_rules! operation {
    ($trait:ident,$method:ident,$ffi:ident) => {
        impl $trait<&Float> for &Float {
            type Output = Float;
            fn $method(self, other: &Float) -> Float {
                assert_eq!(self.bits, other.bits);
                let mut r = Float::zero(self.bits);
                // SAFETY: all values initialized, output distinct, operands borrowed.
                unsafe { $ffi(&mut r.raw, &self.raw, &other.raw) }
                r
            }
        }
    };
}
operation!(Add, add, __gmpf_add);
operation!(Sub, sub, __gmpf_sub);
operation!(Mul, mul, __gmpf_mul);
impl Div<&Float> for &Float {
    type Output = Float;
    fn div(self, other: &Float) -> Float {
        assert_eq!(self.bits, other.bits);
        assert!(other != &Float::int(0, other.bits));
        let mut r = Float::zero(self.bits);
        // SAFETY: initialized distinct values and validated nonzero divisor.
        unsafe { __gmpf_div(&mut r.raw, &self.raw, &other.raw) }
        r
    }
}
impl Neg for &Float {
    type Output = Float;
    fn neg(self) -> Float {
        let mut r = Float::zero(self.bits);
        // SAFETY: initialized distinct output and live borrowed input.
        unsafe { __gmpf_neg(&mut r.raw, &self.raw) }
        r
    }
}

#[cfg(test)]
mod tests {
    use super::*;
    #[test]
    fn exact_integer_and_ties() {
        for x in [i128::MIN, -1, 0, 1, i128::MAX] {
            assert_eq!(Float::int(x, 512).trunc_i128(), x);
        }
        for (x, expected) in [(-5, -2), (-3, -2), (-1, 0), (1, 0), (3, 2), (5, 2)] {
            assert_eq!(Float::int(x, 512).scale(-1).nearest_i128(), expected);
        }
        assert_eq!(
            Float::int(1, 512).scale(-1).scientific(),
            format!("5.{}E-1", "0".repeat(36))
        );
    }
}
