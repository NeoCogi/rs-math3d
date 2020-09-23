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
    fn tsin(self) -> Self;
    fn tcos(self) -> Self;
    fn ttan(self) -> Self;
    fn tacos(self) -> Self;
    fn tsqrt(self) -> Self;
    fn tabs(self) -> Self;
    fn min(l: Self, r: Self) -> Self;
    fn max(l: Self, r: Self) -> Self;
    fn squared(l: Self) -> Self;
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

macro_rules! implScalar {
    ($scalar:ident, $float:ident) => {
        impl Scalar for $scalar {
            fn epsilon() -> $scalar { <$scalar as Epsilon>::epsilon() }
            fn zero() -> $scalar { 0 as $scalar }
            fn one() -> $scalar { 1 as $scalar }
            fn two() -> $scalar { 2 as $scalar }
            fn half() -> $scalar { 0.5 as $scalar }
            fn quarter() -> $scalar { 0.25 as $scalar }
            fn tsqrt(self) -> $scalar { (self as $float).sqrt() as $scalar }
            fn tsin(self) -> $scalar { (self as $float).sin() as $scalar }
            fn tcos(self) -> $scalar { (self as $float).cos() as $scalar }
            fn ttan(self) -> $scalar { (self as $float).tan() as $scalar }
            fn tacos(self) -> $scalar { (self as $float).acos() as $scalar }
            fn tabs(self) -> $scalar { self.abs() }
            fn l8192() -> $scalar { 8192 as $scalar }
            fn min(l: Self, r: Self) -> Self { if l < r { l } else { r } }
            fn max(l: Self, r: Self) -> Self { if l > r { l } else { r } }
            fn squared(l: Self) -> Self { l * l }
        }
    }
}

implScalar!(i32, f32);
implScalar!(i64, f64);
implScalar!(f32, f32);
implScalar!(f64, f64);

pub trait FloatScalar : Scalar {
    fn INFINITY() -> Self;
}

impl FloatScalar for f32 {
    fn INFINITY() -> Self { core::f32::INFINITY }
}

impl FloatScalar for f64 {
    fn INFINITY() -> Self { core::f64::INFINITY }
}