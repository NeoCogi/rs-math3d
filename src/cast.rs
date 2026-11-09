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

//! Numeric conversion dispatch for vectors/matrices. Unlike `NumCast`, the
//! fixed-to-fixed path never passes through a floating-point significand.

use crate::{fixed::Format, Fixed, MathError};
use num_traits::ToPrimitive;

/// Converts a scalar numerically with checked range and explicit precision.
///
/// Fixed-to-fixed conversions rescale raw integers once. Float destinations
/// may round to their significand precision; integer destinations truncate
/// toward zero. This trait is independent of the geometry scalar capabilities.
pub trait TryCastFrom<Source>: Sized {
    /// Converts one scalar, returning an error when its value cannot fit.
    fn try_cast_from(source: Source) -> Result<Self, MathError>;

    /// Converts a trusted value using the checks and quantization of
    /// [`TryCastFrom::try_cast_from`]. Useful for representable setup constants.
    ///
    /// # Panics
    /// Panics if the checked conversion returns an error. Use `try_cast_from`
    /// for input whose range or representation has not been established.
    ///
    /// ```
    /// use rs_math3d::{Fixed, TryCastFrom};
    /// type F = Fixed<i32, 16>;
    /// assert_eq!(F::cast_from(0.5f64), F::ONE / F::cast_from(2));
    /// ```
    #[track_caller]
    fn cast_from(source: Source) -> Self {
        Self::try_cast_from(source).expect("numeric conversion must fit")
    }
}

impl<R: Format<F>, S: Format<G>, const F: u32, const G: u32> TryCastFrom<Fixed<R, F>>
    for Fixed<S, G>
{
    fn try_cast_from(source: Fixed<R, F>) -> Result<Self, MathError> {
        source.try_rescale()
    }
}

/// Generates integer-source paths without a float intermediate.
macro_rules! integers_to_fixed {
    ($($t:ty),*) => { $(impl<R: Format<F>, const F: u32> TryCastFrom<$t> for Fixed<R,F> {
        fn try_cast_from(source: $t) -> Result<Self, MathError> {
            Self::try_from_int(source.to_i64().ok_or(MathError::Overflow)?)
        }
    })* };
}
integers_to_fixed!(i8, i16, i32, i64, i128, isize, u8, u16, u32, u64, u128, usize);

/// Generates explicit float-to-fixed conversions with non-finite checks.
macro_rules! floats_to_fixed {
    ($($t:ty, $method:ident);*) => { $(impl<R: Format<F>, const F: u32> TryCastFrom<$t> for Fixed<R,F> {
        fn try_cast_from(source: $t) -> Result<Self, MathError> { Self::$method(source) }
    })* };
}
floats_to_fixed!(f32,try_from_f32;f64,try_from_f64);

/// Generates primitive destinations and exact fixed source dispatch. Primitive
/// source conversions retain num-traits' range-checked numeric cast semantics.
macro_rules! primitive_destination {
    ($destination:ty) => {
        impl<R: Format<F>, const F: u32> TryCastFrom<Fixed<R,F>> for $destination {
            fn try_cast_from(source: Fixed<R,F>) -> Result<Self, MathError> {
                num_traits::NumCast::from(source).ok_or(MathError::Overflow)
            }
        }
        primitive_sources!($destination; i8,i16,i32,i64,i128,isize,u8,u16,u32,u64,u128,usize,f32,f64);
    };
}
/// Expands the finite set of primitive-to-primitive conversions without a
/// blanket implementation that could overlap the exact fixed conversion.
macro_rules! primitive_sources {
    ($destination:ty; $($source:ty),*) => { $(impl TryCastFrom<$source> for $destination {
        fn try_cast_from(source: $source) -> Result<Self, MathError> {
            num_traits::NumCast::from(source).ok_or(MathError::Overflow)
        }
    })* };
}
primitive_destination!(i8);
primitive_destination!(i16);
primitive_destination!(i32);
primitive_destination!(i64);
primitive_destination!(i128);
primitive_destination!(isize);
primitive_destination!(u8);
primitive_destination!(u16);
primitive_destination!(u32);
primitive_destination!(u64);
primitive_destination!(u128);
primitive_destination!(usize);
primitive_destination!(f32);
primitive_destination!(f64);
