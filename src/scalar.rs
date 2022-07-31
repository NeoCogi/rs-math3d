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
use core::ops::{Add, Sub, Mul, Div, Rem, Neg, AddAssign, SubAssign, DivAssign, MulAssign};
use core::cmp::{PartialOrd};

use crate::cmath::{CScalar};

pub trait Scalar<Rhs = Self, Output = Self> :
    Add<Rhs, Output = Output>
    + Sub<Rhs, Output = Output>
    + Mul<Rhs, Output = Output>
    + Div<Rhs, Output = Output>
    + Rem<Rhs, Output = Output>
    + DivAssign<Rhs>
    + MulAssign<Rhs>
    + Neg<Output = Output>
    + AddAssign<Rhs>
    + SubAssign<Rhs>
    + PartialOrd
    + Clone + Copy
{
    fn zero() -> Self;
    fn epsilon() -> Self;
    fn one() -> Self;
    fn two() -> Self;
    fn half() -> Self;
    fn quarter() -> Self;
    fn l8192() -> Self;
    fn min(l: Self, r: Self) -> Self;
    fn max(l: Self, r: Self) -> Self;
    fn squared(l: Self) -> Self;
    fn tabs(self) -> Self;
}

pub trait FloatScalar<Rhs = Self, Output = Self> : Scalar<Rhs, Output> {
    fn infinity() -> Self;
    fn tsin(self) -> Self;
    fn tcos(self) -> Self;
    fn ttan(self) -> Self;
    fn tacos(self) -> Self;
    fn tsqrt(self) -> Self;
}

trait Epsilon {
    fn epsilon() -> Self;
}

impl Epsilon for i32 {
    fn epsilon() -> Self { 0 }
}

impl Epsilon for i64 {
    fn epsilon() -> Self { 0 }
}

impl Epsilon for f32 {
    fn epsilon() -> Self { 1.0 / (1024.0 * 1024.0) }
}

impl Epsilon for f64 {
    fn epsilon() -> Self { 1.0 / (1024.0 * 1024.0 * 1024.0 * 1024.0) }
}

macro_rules! impl_scalar {
    ($scalar:ident, $float:ident) => {
        impl Scalar for $scalar {
            fn epsilon() -> $scalar { <$scalar as Epsilon>::epsilon() }
            fn zero() -> $scalar { 0 as $scalar }
            fn one() -> $scalar { 1 as $scalar }
            fn two() -> $scalar { 2 as $scalar }
            fn half() -> $scalar { 0.5 as $scalar }
            fn quarter() -> $scalar { 0.25 as $scalar }
            fn l8192() -> $scalar { 8192 as $scalar }
            fn min(l: Self, r: Self) -> Self { if l < r { l } else { r } }
            fn max(l: Self, r: Self) -> Self { if l > r { l } else { r } }
            fn squared(l: Self) -> Self { l * l }
            fn tabs(self) -> $scalar { self.abs() }
        }
    }
}

macro_rules! impl_float_scalar {
    ($scalar:ident) => {
        impl FloatScalar for $scalar {
            fn infinity() -> $scalar { core::$scalar::INFINITY }
            fn tsqrt(self) -> $scalar { self.sqrt() as $scalar }
            fn tsin(self) -> $scalar { self.sin() as $scalar }
            fn tcos(self) -> $scalar { self.cos() as $scalar }
            fn ttan(self) -> $scalar { self.tan() as $scalar }
            fn tacos(self) -> $scalar { self.acos() as $scalar }
        }
    }
}

impl_scalar!(i32, f32);
impl_scalar!(i64, f64);
impl_scalar!(f32, f32);
impl_scalar!(f64, f64);

impl_float_scalar!(f32);
impl_float_scalar!(f64);

#[cfg(feature = "fixedpoint")]
mod fixedpoint {
    macro_rules! impl_scalar_for_fixed {
        ($scalar:ident, $float:ident) => {
            impl super::Scalar for $scalar {
                fn epsilon() -> $scalar { Self::zero() }
                fn zero() -> $scalar    { $scalar::from_num(0   ) }
                fn one() -> $scalar     { $scalar::from_num(1   ) }
                fn two() -> $scalar     { $scalar::from_num(2   ) }
                fn half() -> $scalar    { $scalar::from_num(0.5 ) }
                fn quarter() -> $scalar { $scalar::from_num(0.25) }
                fn l8192() -> $scalar   { $scalar::from_num(8192) }
                fn min(l: Self, r: Self) -> Self { if l < r { l } else { r } }
                fn max(l: Self, r: Self) -> Self { if l > r { l } else { r } }
                fn squared(l: Self) -> Self { l * l }
                fn tabs(self) -> $scalar { self.abs() }
            }
        }
    }


    macro_rules! impl_fixed_scalar {
        ($scalar:ident, $max:path) => {
            impl super::FloatScalar for $scalar {
                fn infinity()  -> $scalar { $max }
                fn tsqrt(self) -> $scalar { $scalar::from_num((self.to_num::<f64>()).sqrt()) }
                fn tsin(self)  -> $scalar { $scalar::from_num((self.to_num::<f64>()).sin() ) }
                fn tcos(self)  -> $scalar { $scalar::from_num((self.to_num::<f64>()).cos() ) }
                fn ttan(self)  -> $scalar { $scalar::from_num((self.to_num::<f64>()).tan() ) }
                fn tacos(self) -> $scalar { $scalar::from_num((self.to_num::<f64>()).acos()) }
            }
        }
    }

    pub type Fixed112 = fixed::types::I112F16;
    pub type Fixed48  = fixed::types::I48F16;

    impl_scalar_for_fixed!(Fixed48, f64);
    impl_scalar_for_fixed!(Fixed112, f64);

    impl_fixed_scalar!(Fixed48, fixed::FixedI64::MAX);
    impl_fixed_scalar!(Fixed112, fixed::FixedI128::MAX);
}

#[cfg(feature="fixedpoint")]
pub use fixedpoint::*;

#[cfg(test)]
mod tests {
    use super::*;
    #[test]
    pub fn test() {
        let out = -1.0;
        let f = out.tabs();
        assert_eq!(f, 1.0);
    }
}