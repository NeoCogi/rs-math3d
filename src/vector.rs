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
//! Vector mathematics module providing 2D, 3D, and 4D vector operations.
//!
//! This module provides generic vector types and operations for computer graphics
//! and linear algebra applications. All vectors support standard arithmetic operations,
//! dot products, and component-wise operations.
//!
//! Integer vectors are supported for storage and discrete geometry. Operations that
//! require fractional results, such as normalization, are available only through
//! [`RealVector`].
//!
//! Checked 3D geometry uses crate-private, max-component-scaled normalization
//! helpers and reuses the prepared directions, subject to scalar quantization.
//! Its angular classifiers use stable scalar norms: floats scale components
//! before taking a norm, while fixed scalars compare unquantized squared
//! integers. They distinguish within-tolerance, outside-tolerance and invalid
//! results. Public checked queries preserve invalid input or arithmetic as an
//! error and reserve `Ok(None)` for valid geometric rejection. Public ordinary
//! and checked [`RealVector`] methods follow the crate's
//! [arithmetic contracts](crate#arithmetic-contracts).
//!
//! # Examples
//!
//! ```
//! # fn main() {
//! # #[cfg(any(feature = "std", feature = "libm", feature = "system-libm"))]
//! # {
//! use rs_math3d::vector::{RealVector, Vector, Vector3};
//!
//! let v1 = Vector3::new(1.0, 2.0, 3.0);
//! let v2 = Vector3::new(4.0, 5.0, 6.0);
//!
//! // Vector addition
//! let sum = v1 + v2;
//!
//! // Dot product
//! let dot = Vector3::dot(&v1, &v2);
//!
//! // Normalization
//! let normalized = v1.normalize();
//! # }
//! # }
//! ```

use crate::scalar::*;
use crate::MathError;
use core::ops::{Add, Div, Mul, Neg, Rem, Sub};
use num_traits::Zero;

/// Generic vector trait defining common vector operations.
///
/// This trait provides the foundation for all vector types, defining
/// operations like addition, subtraction, dot product, and component-wise operations.
///
/// # Mathematical Operations
///
/// For vectors **v** and **w**, and scalar `s`:
/// - Addition: **v** + **w** = (v₁ + w₁, v₂ + w₂, ...)
/// - Subtraction: **v** - **w** = (v₁ - w₁, v₂ - w₂, ...)
/// - Scalar multiplication: s**v** = (sv₁, sv₂, ...)
/// - Dot product: **v** · **w** = v₁w₁ + v₂w₂ + ...
pub trait Vector<T: Scalar, Rhs = Self, Output = Self>:
    Add<Rhs, Output = Output>
    + Sub<Rhs, Output = Output>
    + Mul<Rhs, Output = Output>
    + Mul<T, Output = Output>
    + Div<Rhs, Output = Output>
    + Div<T, Output = Output>
    + Neg<Output = Output>
    + Clone
    + Copy
{
    /// Returns the zero vector (all components are zero).
    fn zero() -> Self;

    /// Adds two vectors component-wise.
    fn add_vv(l: &Self, r: &Self) -> Self;

    /// Subtracts two vectors component-wise.
    fn sub_vv(l: &Self, r: &Self) -> Self;

    /// Multiplies two vectors component-wise (Hadamard product).
    fn mul_vv(l: &Self, r: &Self) -> Self;

    /// Divides two vectors component-wise.
    fn div_vv(l: &Self, r: &Self) -> Self;

    /// Multiplies a vector by a scalar.
    fn mul_vs(l: &Self, r: T) -> Self;

    /// Divides a vector by a scalar.
    fn div_vs(l: &Self, r: T) -> Self;

    /// Computes the remainder of component-wise division.
    fn rem_vv(l: &Self, r: &Self) -> Self;

    /// Computes the dot product of two vectors.
    ///
    /// For vectors **v** and **w**:
    /// ```text
    /// v · w = Σᵢ vᵢwᵢ
    /// ```
    fn dot(l: &Self, r: &Self) -> T;

    /// Adds components with checked arithmetic.
    fn checked_add_vv(l: &Self, r: &Self) -> Result<Self, MathError>;
    /// Subtracts components with checked arithmetic.
    fn checked_sub_vv(l: &Self, r: &Self) -> Result<Self, MathError>;
    /// Multiplies every component with checked arithmetic.
    fn checked_mul_vs(l: &Self, r: T) -> Result<Self, MathError>;
    /// Divides every component with zero/range checks.
    fn checked_div_vs(l: &Self, r: T) -> Result<Self, MathError>;
    /// Checked componentwise multiplication.
    fn checked_mul_vv(l: &Self, r: &Self) -> Result<Self, MathError>;
    /// Checked componentwise division.
    fn checked_div_vv(l: &Self, r: &Self) -> Result<Self, MathError>;
    /// Checked componentwise remainder.
    fn checked_rem_vv(l: &Self, r: &Self) -> Result<Self, MathError>;
    /// Checked scalar remainder.
    fn checked_rem_vs(l: &Self, r: T) -> Result<Self, MathError>;
    /// Checked componentwise negation.
    fn checked_neg(&self) -> Result<Self, MathError>;
    /// Computes a checked dot product with one fixed-point quantization.
    fn checked_dot(l: &Self, r: &Self) -> Result<T, MathError>;

    /// Returns a vector with the minimum components from both vectors.
    fn min(l: &Self, r: &Self) -> Self;

    /// Returns a vector with the maximum components from both vectors.
    fn max(l: &Self, r: &Self) -> Self;

    /// Returns the largest absolute component (the infinity norm).
    ///
    /// This reduction performs no products or square root, preserving small
    /// fixed-point components even when their squares would quantize to zero.
    /// Zero vectors return zero. Components must be finite and their absolute
    /// values representable; otherwise ordinary scalar arithmetic applies.
    /// Use [`Vector::checked_max_abs_component`] to validate those requirements.
    ///
    /// ```
    /// use rs_math3d::{Fixed, Vector, Vector3};
    /// type F = Fixed<i32, 16>;
    /// let v = Vector3::new(F::DELTA, -F::DELTA, F::ZERO);
    /// assert_eq!(v.max_abs_component(), F::DELTA);
    /// ```
    fn max_abs_component(&self) -> T;

    /// Returns the largest absolute component with checked scalar absolutes.
    ///
    /// Visits components in coordinate order and propagates the first error:
    /// [`MathError::NonFinite`] for exceptional floating-point values, or
    /// [`MathError::Overflow`] when a signed minimum has no positive counterpart.
    /// Zero vectors return zero; no squared length or norm must fit.
    fn checked_max_abs_component(&self) -> Result<T, MathError>;
}

/// Trait for vectors with fractional real components, including fixed point.
///
/// Extends the base `Vector` trait with operations that require
/// fractional arithmetic, such as length calculation and normalization.
pub trait RealVector<T: RealScalar>: Vector<T> {
    /// Computes the Euclidean length (magnitude) of the vector.
    ///
    /// For vector **v**:
    /// ```text
    /// ||v|| = √(v₁² + v₂² + ... + vₙ²)
    /// ```
    fn length(&self) -> T;

    /// Computes length with a checked, scale-aware scalar norm.
    fn checked_length(&self) -> Result<T, MathError>;

    /// Checks every component for IEEE exceptional values; fixed values are finite.
    fn is_finite(&self) -> bool;

    /// Normalizes through max-component scaling. Invalid tolerances, zero or
    /// near-zero vectors, and unrepresentable arithmetic are explicit errors.
    fn checked_normalize(&self, epsilon: T) -> Result<Self, MathError>;

    /// Returns a unit vector in the same direction.
    ///
    /// For vector **v**:
    /// ```text
    /// v̂ = v / ||v||
    /// ```
    ///
    /// # Note
    /// Floating zero vectors yield NaN/Inf; fixed overflow wraps and zero division panics.
    /// Prefer `checked_normalize` for explicit error handling.
    fn normalize(&self) -> Self;

    /// Computes the Euclidean distance between two vectors.
    ///
    /// For vectors **v** and **w**:
    /// ```text
    /// d(v, w) = ||v - w||
    /// ```
    fn distance(l: &Self, r: &Self) -> T;

    /// Computes the squared length (avoids a square root).
    ///
    /// ```text
    /// ||v||² = v · v
    /// ```
    fn length_squared(&self) -> T {
        Self::dot(self, self)
    }

    /// Computes squared length with checked product accumulation.
    fn checked_length_squared(&self) -> Result<T, MathError> {
        Self::checked_dot(self, self)
    }
    /// Computes distance with checked subtraction and a stable checked norm.
    fn checked_distance(l: &Self, r: &Self) -> Result<T, MathError> {
        Self::checked_sub_vv(l, r)?.checked_length()
    }
    /// Returns a normalized vector or the zero vector when too small.
    fn normalize_or_zero(&self, epsilon: T) -> Self {
        self.checked_normalize(epsilon)
            .unwrap_or_else(|_| Self::zero())
    }

    /// Normalizes using a precomputed inverse length (e.g., from rsqrt).
    fn normalize_with_inv_len(&self, inv_len: T) -> Self {
        *self * inv_len
    }

    /// Tests represented length against a finite nonnegative tolerance without
    /// requiring the unscaled norm or small squared tolerance to fit.
    fn checked_norm_exceeds(&self, epsilon: T) -> Result<bool, MathError>;

    /// Normalizes with precomputed length squared and inverse length.
    fn checked_normalize_with_inv_len(
        &self,
        len_sq: T,
        inv_len: T,
        epsilon: T,
    ) -> Result<Self, MathError> {
        if !len_sq.is_finite() || !inv_len.is_finite() {
            return Err(MathError::NonFinite);
        }
        if len_sq < <T as Zero>::zero() || inv_len <= <T as Zero>::zero() {
            return Err(MathError::Domain);
        }
        if !self.checked_norm_exceeds(epsilon)? {
            return Err(MathError::Degenerate);
        }
        Self::checked_mul_vs(self, inv_len)
    }
}

macro_rules! implVecScalar {
    ($vecName:ident, $scalar:ident) => {
        impl Mul<$vecName<$scalar>> for $scalar {
            type Output = $vecName<$scalar>;

            fn mul(self, rhs: $vecName<$scalar>) -> Self::Output {
                $vecName::mul_vs(&rhs, self)
            }
        }
    };
}

macro_rules! vector_field_doc {
    (x) => {
        "X component."
    };
    (y) => {
        "Y component."
    };
    (z) => {
        "Z component."
    };
    (w) => {
        "W component."
    };
}

macro_rules! implVector {
    ($(#[$meta:meta])* $vecName:ident, $dimension:literal, $($field:ident)*) => {
        $(#[$meta])*
        #[repr(C)]
        #[derive(Copy, Clone, Debug, Default)]
        pub struct $vecName<T> { $(#[doc = vector_field_doc!($field)] pub $field: T),* }

        impl<T> $vecName<T> {
            /// Moves array components into coordinate order, without conversion.
            pub fn from_array(values: [T; $dimension]) -> Self {
                let [$($field),*] = values;
                Self { $($field),* }
            }
            /// Moves components into an array in coordinate order, without conversion.
            pub fn to_array(self) -> [T; $dimension] { [$(self.$field),*] }
            /// Maps owned components once each, in coordinate order. No numeric
            /// policy is imposed; the callback owns conversion, checks or projection.
            pub fn map<U>(self, mut map: impl FnMut(T) -> U) -> $vecName<U> {
                $vecName { $($field: map(self.$field)),* }
            }
        }

        impl<T: Scalar> $vecName<T> {
            /// Creates a vector from components.
            pub fn new($($field:T),*) -> Self { Self { $($field: $field),* } }
        }

        impl<T> $vecName<T>
        where
            T: Copy,
        {
            /// Attempts to cast each component into another numeric type.
            pub fn try_cast<U>(&self) -> Option<$vecName<U>>
            where
                U: Scalar + crate::TryCastFrom<T>,
            {
                Some($vecName {
                    $($field: U::try_cast_from(self.$field).ok()?),*
                })
            }
        }

        impl<T: Scalar> Vector<T> for $vecName<T> {
            fn zero() -> Self { Self { $($field: <T as Zero>::zero()),* } }
            #[inline(always)]
            fn checked_add_vv(l: &Self, r: &Self) -> Result<Self, MathError> { Ok(Self::new($(l.$field.checked_add(r.$field)?),*)) }
            #[inline(always)]
            fn checked_sub_vv(l: &Self, r: &Self) -> Result<Self, MathError> { Ok(Self::new($(l.$field.checked_sub(r.$field)?),*)) }
            fn checked_mul_vs(l: &Self, r: T) -> Result<Self, MathError> { Ok(Self::new($(l.$field.checked_mul(r)?),*)) }
            fn checked_div_vs(l: &Self, r: T) -> Result<Self, MathError> { Ok(Self::new($(l.$field.checked_div(r)?),*)) }
            fn checked_mul_vv(l: &Self, r: &Self) -> Result<Self, MathError> { Ok(Self::new($(l.$field.checked_mul(r.$field)?),*)) }
            fn checked_div_vv(l: &Self, r: &Self) -> Result<Self, MathError> { Ok(Self::new($(l.$field.checked_div(r.$field)?),*)) }
            fn checked_rem_vv(l: &Self, r: &Self) -> Result<Self, MathError> { Ok(Self::new($(l.$field.checked_rem(r.$field)?),*)) }
            fn checked_rem_vs(l: &Self, r: T) -> Result<Self, MathError> { Ok(Self::new($(l.$field.checked_rem(r)?),*)) }
            fn checked_neg(&self) -> Result<Self, MathError> { Ok(Self::new($(self.$field.checked_neg()?),*)) }
            fn checked_dot(l: &Self, r: &Self) -> Result<T, MathError> { T::checked_sum_products(&[$((l.$field,r.$field,false)),*]) }
            fn dot  (l: &Self, r: &Self) -> T { T::sum_products(&[$((l.$field, r.$field, false)),*]) }
            fn add_vv(l: &Self, r: &Self) -> Self { Self::new($(l.$field + r.$field),*) }
            fn sub_vv(l: &Self, r: &Self) -> Self { Self::new($(l.$field - r.$field),*) }
            fn mul_vv(l: &Self, r: &Self) -> Self { Self::new($(l.$field * r.$field),*) }
            fn div_vv(l: &Self, r: &Self) -> Self { Self::new($(l.$field / r.$field),*) }
            fn mul_vs(l: &Self, r: T) -> Self { Self::new($(l.$field * r),*) }
            fn div_vs(l: &Self, r: T) -> Self { Self::new($(l.$field / r),*) }
            fn rem_vv(l: &Self, r: &Self) -> Self { Self::new($(l.$field % r.$field),*) }
            fn min(l: &Self, r: &Self) -> Self { Self::new($(T::min(l.$field, r.$field)),*) }
            fn max(l: &Self, r: &Self) -> Self { Self::new($(T::max(l.$field, r.$field)),*) }
            fn max_abs_component(&self) -> T {
                let mut maximum = <T as Zero>::zero();
                $(maximum = T::max(maximum, self.$field.tabs());)*
                maximum
            }
            fn checked_max_abs_component(&self) -> Result<T, MathError> {
                let mut maximum = <T as Zero>::zero();
                $(maximum = T::max(maximum, self.$field.checked_abs()?);)*
                Ok(maximum)
            }
        }

        impl<T> Add for $vecName<T> where T: Scalar {
            type Output = $vecName<T>;

            fn add(self, rhs: Self) -> Self::Output {
                Self { $($field: self.$field + rhs.$field),* }
            }
        }

        impl<T> Sub for $vecName<T> where T: Scalar {
            type Output = $vecName<T>;

            fn sub(self, rhs: Self) -> Self::Output {
                Self { $($field: self.$field - rhs.$field),* }
            }
        }

        impl<T> Mul for $vecName<T> where T: Scalar {
            type Output = $vecName<T>;

            fn mul(self, rhs: Self) -> Self::Output {
                Self { $($field: self.$field * rhs.$field),* }
            }
        }

        impl<T> Mul<T> for $vecName<T> where T:Scalar {
            type Output = $vecName<T>;

            fn mul(self, rhs: T) -> Self::Output {
                Self { $($field: self.$field * rhs),* }
            }
        }

        implVecScalar!($vecName, f32);
        implVecScalar!($vecName, f64);
        implVecScalar!($vecName, i32);
        implVecScalar!($vecName, i64);

        impl<R: crate::fixed::ScalarFormat<F>, const F: u32> Mul<$vecName<crate::Fixed<R,F>>> for crate::Fixed<R,F> {
            type Output = $vecName<Self>;
            fn mul(self, rhs: Self::Output) -> Self::Output { rhs * self }
        }

        impl<T> Div for $vecName<T> where T:Scalar {
            type Output = $vecName<T>;

            fn div(self, rhs: Self) -> Self::Output {
                Self { $($field: self.$field / rhs.$field),* }
            }
        }

        impl<T> Div<T> for $vecName<T> where T:Scalar {
            type Output = $vecName<T>;

            fn div(self, rhs: T) -> Self::Output {
                Self { $($field: self.$field / rhs),* }
            }
        }

        impl<T> Rem for $vecName<T> where T: Scalar {
            type Output = $vecName<T>;

            fn rem(self, rhs: $vecName<T>) -> Self::Output {
                Self { $($field: self.$field % rhs.$field),* }
            }
        }

        impl<T> Rem<T> for $vecName<T> where T:Scalar {
            type Output = $vecName<T>;

            fn rem(self, rhs: T) -> Self::Output {
                Self { $($field: self.$field % rhs),* }
            }
        }

        impl<T: Scalar> Neg for $vecName<T> {
            type Output = $vecName<T>;
            fn neg(self) -> Self::Output {
                Self { $($field: -self.$field),* }
            }
        }
    };
}

macro_rules! implRealVector {
    ($vecName:ident, $($field:ident),+) => {
        impl<T: RealScalar> RealVector<T> for $vecName<T> {
            fn length(&self) -> T {
                T::norm(&[$(self.$field),*])
            }
            fn checked_length(&self) -> Result<T, MathError> { T::checked_norm(&[$(self.$field),*]) }
            fn is_finite(&self) -> bool { $(self.$field.is_finite() &&)* true }
            fn checked_norm_exceeds(&self, epsilon: T) -> Result<bool, MathError> { T::checked_norm_exceeds(&[$(self.$field),*],epsilon) }
            fn checked_normalize(&self, epsilon: T) -> Result<Self, MathError> {
                let values = [$(self.$field),*];
                if !T::checked_norm_exceeds(&values, epsilon)? { return Err(MathError::Degenerate); }
                let scale = self.checked_max_abs_component()?;
                let scaled = Self::checked_div_vs(self, scale)?;
                let length = scaled.checked_length()?;
                Self::checked_div_vs(&scaled, length)
            }
            fn normalize(&self) -> Self {
                let len = Self::length(self);
                *self / len
            }
            fn distance(l: &Self, r: &Self) -> T {
                Self::length(&(*r - *l))
            }
        }
    };
}

/// Trait for computing the cross product of 3D vectors.
///
/// In exact arithmetic the 3D cross product produces a vector perpendicular
/// to both inputs. Scalar rounding can affect perpendicularity and algebraic
/// identities in a represented result.
pub trait CrossProduct {
    /// Computes the cross product of two vectors.
    ///
    /// For 3D vectors **v** and **w**:
    /// ```text
    /// v × w = (v₂w₃ - v₃w₂, v₃w₁ - v₁w₃, v₁w₂ - v₂w₁)
    /// ```
    ///
    /// In exact arithmetic the result is perpendicular to both input vectors,
    /// with magnitude ||v|| ||w|| sin(θ), where θ is the angle between them.
    /// Implementations follow their scalar's arithmetic and rounding rules.
    fn cross(l: &Self, r: &Self) -> Self;
}

implVector!(
    /// A 2D vector with x and y components.
    ///
    /// # Examples
    /// ```
    /// use rs_math3d::vector::Vector2;
    ///
    /// let v = Vector2::new(3.0, 4.0);
    /// assert_eq!(v.x, 3.0);
    /// assert_eq!(v.y, 4.0);
    /// ```
    Vector2, 2,
    x y
);

implVector!(
    /// A 3D vector with x, y, and z components.
    ///
    /// # Examples
    /// ```
    /// use rs_math3d::vector::{Vector3, CrossProduct};
    ///
    /// let v1 = Vector3::new(1.0, 0.0, 0.0);
    /// let v2 = Vector3::new(0.0, 1.0, 0.0);
    /// let cross = Vector3::cross(&v1, &v2);
    /// // Result is (0, 0, 1) - the z-axis
    /// ```
    Vector3, 3,
    x y z
);

implVector!(
    /// A 4D vector with x, y, z, and w components.
    ///
    /// Often used for homogeneous coordinates in 3D graphics.
    ///
    /// # Examples
    /// ```
    /// use rs_math3d::vector::Vector4;
    ///
    /// let v = Vector4::new(1.0, 2.0, 3.0, 1.0);
    /// // w=1 for positions, w=0 for directions
    /// ```
    Vector4, 4,
    x y z w
);

implRealVector!(Vector2, x, y);
implRealVector!(Vector3, x, y, z);
implRealVector!(Vector4, x, y, z, w);

impl<T: Scalar> Vector3<T> {
    /// Computes a cross product with checked, single-rounding differences of
    /// products, preserving cancellation below one stored quantum.
    /// Uses the same scalar quantization as [`CrossProduct::cross`]; checking
    /// range does not restore exact-arithmetic identities.
    pub fn checked_cross(l: &Self, r: &Self) -> Result<Self, MathError> {
        Ok(Self::new(
            T::checked_sum_products(&[(l.y, r.z, false), (l.z, r.y, true)])?,
            T::checked_sum_products(&[(l.z, r.x, false), (l.x, r.z, true)])?,
            T::checked_sum_products(&[(l.x, r.y, false), (l.y, r.x, true)])?,
        ))
    }
}

impl<T> CrossProduct for Vector3<T>
where
    T: Scalar,
{
    /// Computes the cross product of two 3D vectors.
    ///
    /// In exact arithmetic, **v** × **w** is perpendicular to both inputs and
    /// follows the right-hand rule.
    ///
    /// # Exact-arithmetic properties
    ///
    /// - Anti-commutative: **v** × **w** = -(**w** × **v**)
    /// - Distributive: **v** × (**w** + **u**) = **v** × **w** + **v** × **u**
    /// - **v** × **v** = **0**
    ///
    /// These identities and perpendicularity are subject to scalar arithmetic
    /// and rounding. Fixed components accumulate widened differences of products
    /// and floor-shift once, as described in the [fixed-point guide](crate::fixed).
    /// Reversing the operands can therefore differ from negating the result:
    ///
    /// ```
    /// use rs_math3d::{CrossProduct, Fixed, Vector3};
    /// type F = Fixed<i32, 16>;
    /// let half = F::from_bits(1 << 15);
    /// let a = Vector3::new(F::DELTA, F::ZERO, F::ZERO);
    /// let b = Vector3::new(F::ZERO, half, F::ZERO);
    /// assert_eq!(Vector3::cross(&a, &b).z, F::ZERO);
    /// assert_eq!(Vector3::cross(&b, &a).z, -F::DELTA);
    /// assert_eq!(Vector3::checked_cross(&b, &a).unwrap().z, -F::DELTA);
    /// ```
    fn cross(l: &Vector3<T>, r: &Vector3<T>) -> Vector3<T> {
        Vector3::new(
            T::sum_products(&[(l.y, r.z, false), (l.z, r.y, true)]),
            T::sum_products(&[(l.z, r.x, false), (l.x, r.z, true)]),
            T::sum_products(&[(l.x, r.y, false), (l.y, r.x, true)]),
        )
    }
}

/// Decomposes a nonzero direction without validation. Scaling is a numerical
/// algorithm choice, not an input check; zero/nonfinite input has native semantics.
pub(crate) fn normalized3<T: RealScalar>(vector: &Vector3<T>) -> Normalized3<T> {
    let scale = vector.max_abs_component();
    let scaled = *vector / scale;
    let scaled_length = scaled.length();
    Normalized3 {
        unit: scaled / scaled_length,
        scale,
        scaled_length,
    }
}

/// A finite, nonzero 3D vector split into a unit direction and safe length factors.
///
/// Directly computing `sqrt(x*x + y*y + z*z)` can overflow for a finite large
/// vector or underflow for a finite small vector. This representation first
/// divides by the largest absolute component. Its invariants are:
///
/// - `scale` is the original vector's finite, strictly positive largest
///   component magnitude;
/// - `scaled_length` is the length of `original / scale`, and is finite and in
///   the interval `[1, sqrt(3)]`;
/// - `unit` is `(original / scale) / scaled_length`, without requiring the
///   potentially overflowing original length to be formed.
///
/// Checked construction validates these invariants; unchecked construction
/// assumes valid inputs and representable arithmetic.
/// The type is crate-private because it supplies numerical machinery for
/// geometric queries rather than a second public vector-normalization API.
#[derive(Clone, Copy, Debug)]
pub(crate) struct Normalized3<T: RealScalar> {
    /// Unit-length direction of the original vector.
    unit: Vector3<T>,
    /// Largest absolute component of the original vector.
    scale: T,
    /// Length of the max-component-scaled vector `original / scale`.
    ///
    /// Retaining this bounded factor lets parameter conversions avoid forming
    /// the potentially overflowing product `scale * scaled_length`.
    scaled_length: T,
}

impl<T: RealScalar> Normalized3<T> {
    /// Converts physical travel to the stored parameter without validation.
    pub(crate) fn divide_by_length_unchecked(&self, value: T) -> T {
        (value / self.scale) / self.scaled_length
    }
    /// Returns the finite unit vector represented by this decomposition.
    pub(crate) fn unit(&self) -> Vector3<T> {
        // Checked construction establishes the invariants. Unchecked callers
        // are responsible for nonzero finite inputs and representable arithmetic.
        self.unit
    }

    /// Divides physical travel by length using checked arithmetic.
    /// Tries both division orderings so an overflowing first intermediate does not
    /// reject a representable quotient. Nonfinite inputs and range failures propagate.
    pub(crate) fn checked_divide_by_length(&self, value: T) -> Result<T, MathError> {
        value
            .checked_div(self.scale)
            .and_then(|q| q.checked_div(self.scaled_length))
            .or_else(|_| {
                value
                    .checked_div(self.scaled_length)?
                    .checked_div(self.scale)
            })
    }

    /// Converts physical travel to a stored parameter using checked division.
    /// Returns `Overflow` when nonzero travel rounds to zero; exact zero retains its sign.
    pub(crate) fn checked_divide_by_length_preserving_nonzero(
        &self,
        value: T,
    ) -> Result<T, MathError> {
        let quotient = self.checked_divide_by_length(value)?;
        checked_preserve_nonzero_parameter(value, quotient)
    }
}

/// Validates the cross-field invariant of a returned geometric parameter.
/// `travel` and `parameter` come from successful checked arithmetic; finite
/// scalar underflow is allowed there, but a nonzero travel must not become a
/// zero stored parameter beside a separately reconstructed non-origin point.
/// This policy is dimension-independent and does not alter scalar division.
pub(crate) fn checked_preserve_nonzero_parameter<T: RealScalar>(
    travel: T,
    parameter: T,
) -> Result<T, MathError> {
    if travel != <T as Zero>::zero() && parameter == <T as Zero>::zero() {
        Err(MathError::Overflow)
    } else {
        Ok(parameter)
    }
}

/// Result of comparing an angular relationship with a caller's tolerance.
///
/// Parallel classifiers compare `|sin(theta)|`, while perpendicular classifiers
/// compare `|cos(theta)|`. Keeping the comparison result separate from the
/// relationship being measured lets both hot paths share one explicit state
/// type without computing an unnecessary second angular measure.
///
/// This enum is crate-private because it coordinates geometric query internals;
/// public queries continue to report invalid or rejected geometry through their
/// documented `Result` results.
#[must_use = "angular classifications, including invalid states, must be handled"]
#[derive(Clone, Copy, Debug, Eq, PartialEq)]
pub(crate) enum AngularClassification {
    /// The selected sine or cosine magnitude is at most the inclusive tolerance.
    WithinTolerance,
    /// The selected sine or cosine magnitude is greater than the tolerance.
    OutsideTolerance,
    /// No classification is possible because input or derived arithmetic is invalid.
    Invalid,
}

/// Validates a dimensionless angular tolerance shared by orientation predicates.
///
/// Sine and cosine magnitudes both occupy the closed interval `[0, 1]`.
/// Centralizing this check keeps raw-vector and pre-normalized call paths
/// identical while rejecting NaN, infinity, and nonsensical negative or
/// greater-than-one tolerances before any orientation arithmetic.
fn is_valid_angular_epsilon<T: RealScalar>(angular_epsilon: T) -> bool {
    // Comparisons with NaN are unordered, but the explicit finiteness check
    // also makes that behavior clear for custom `RealScalar` implementations.
    angular_epsilon.is_finite()
        && angular_epsilon >= <T as Zero>::zero()
        && angular_epsilon <= <T as One>::one()
}

/// Prepares the same normalized representation while preserving the cause of
/// invalid input, degenerate direction or unrepresentable checked arithmetic.
pub(crate) fn checked_normalized3<T: RealScalar>(
    vector: &Vector3<T>,
) -> Result<Normalized3<T>, MathError> {
    if !vector.is_finite() {
        return Err(MathError::NonFinite);
    }
    let scale = vector.checked_max_abs_component()?;
    if scale == <T as Zero>::zero() {
        return Err(MathError::Degenerate);
    }
    let scaled = Vector3::checked_div_vs(vector, scale)?;
    let scaled_length = scaled.checked_length()?;
    let unit = if scaled_length == <T as One>::one() {
        scaled
    } else {
        Vector3::checked_div_vs(&scaled, scaled_length)?
    };
    Ok(Normalized3 {
        unit,
        scale,
        scaled_length,
    })
}

/// Classifies pre-normalized vectors for parallelism within an angular tolerance.
///
/// `angular_epsilon` bounds `|sin(theta)|`. The scalar norm comparison avoids
/// losing small represented directions/tolerances when their squares quantize
/// to zero. Fixed formats compare widened integer squares; float formats scale
/// components before computing a norm, preserving subnormal information.
///
/// Returns [`AngularClassification::WithinTolerance`] when the sine magnitude
/// is at most the inclusive tolerance, [`AngularClassification::OutsideTolerance`]
/// when it is greater, and [`AngularClassification::Invalid`] when the tolerance
/// or derived arithmetic is invalid. The operands already carry the finite,
/// nonzero invariants established by [`checked_normalized3`].
pub(crate) fn classify_nearly_parallel_normalized3<T: RealScalar>(
    left: &Normalized3<T>,
    right: &Normalized3<T>,
    angular_epsilon: T,
) -> AngularClassification {
    // Reject invalid policy values before doing the cross product. Callers can
    // distinguish an invalid query from either valid angular comparison.
    if !is_valid_angular_epsilon(angular_epsilon) {
        return AngularClassification::Invalid;
    }

    let cross = match Vector3::checked_cross(&left.unit(), &right.unit()) {
        Ok(value) => value,
        Err(_) => return AngularClassification::Invalid,
    };
    match T::checked_norm_exceeds(&[cross.x, cross.y, cross.z], angular_epsilon) {
        Ok(true) => AngularClassification::OutsideTolerance,
        Ok(false) => AngularClassification::WithinTolerance,
        Err(_) => AngularClassification::Invalid,
    }
}

/// Classifies pre-normalized vectors for perpendicularity within an angular tolerance.
///
/// `angular_epsilon` bounds `|cos(theta)|`. Since the dot product of unit
/// vectors is already the cosine, this path needs neither a square root nor any
/// additional normalization. The inclusive comparison preserves the public
/// orientation-policy boundary.
///
/// Returns [`AngularClassification::WithinTolerance`] when the cosine magnitude
/// is at most the inclusive tolerance, [`AngularClassification::OutsideTolerance`]
/// when it is greater, and [`AngularClassification::Invalid`] when the tolerance
/// or dot-product arithmetic is invalid.
pub(crate) fn classify_nearly_perpendicular_normalized3<T: RealScalar>(
    left: &Normalized3<T>,
    right: &Normalized3<T>,
    angular_epsilon: T,
) -> AngularClassification {
    // Cosine magnitudes use the same closed interval and validation rules as
    // sine magnitudes in the parallel predicate.
    if !is_valid_angular_epsilon(angular_epsilon) {
        return AngularClassification::Invalid;
    }

    // Stored units make the absolute dot product dimensionless and invariant
    // under independent nonzero scaling of either original vector.
    let cosine = match Vector3::checked_dot(&left.unit(), &right.unit()) {
        Ok(value) => value,
        Err(_) => return AngularClassification::Invalid,
    };
    if !cosine.is_finite() {
        return AngularClassification::Invalid;
    }

    // Compare the magnitude directly so the caller's inclusive tolerance is
    // neither widened nor squared into an underflow-prone representation.
    let cosine_magnitude = match cosine.checked_abs() {
        Ok(value) => value,
        Err(_) => return AngularClassification::Invalid,
    };
    if cosine_magnitude <= angular_epsilon {
        AngularClassification::WithinTolerance
    } else {
        AngularClassification::OutsideTolerance
    }
}

/// Classifies two raw vectors for parallelism within an angular tolerance.
///
/// `angular_epsilon` is a dimensionless upper bound on `|sin(theta)|`, where
/// `theta` is the angle between the vectors. Parallel and anti-parallel vectors
/// are treated identically. The inclusive comparison means a vector exactly on
/// the requested boundary is classified as nearly parallel.
///
/// [`AngularClassification::Invalid`] is returned if either vector is zero or
/// non-finite, or when the tolerance is non-finite or outside `[0, 1]`.
#[cfg(all(test, any(feature = "std", feature = "libm", feature = "system-libm")))]
pub(crate) fn classify_nearly_parallel3<T: RealScalar>(
    left: &Vector3<T>,
    right: &Vector3<T>,
    angular_epsilon: T,
) -> AngularClassification {
    // Each raw operand is normalized exactly once. The normalized-input helper
    // then performs only orientation arithmetic and can also be called directly
    // by queries that need these same decompositions for later calculations.
    let left_normalized = match checked_normalized3(left) {
        Ok(metric) => metric,
        Err(_) => return AngularClassification::Invalid,
    };
    let right_normalized = match checked_normalized3(right) {
        Ok(metric) => metric,
        Err(_) => return AngularClassification::Invalid,
    };
    classify_nearly_parallel_normalized3(&left_normalized, &right_normalized, angular_epsilon)
}

/// Classifies two raw vectors for perpendicularity within an angular tolerance.
///
/// `angular_epsilon` is a dimensionless upper bound on `|cos(theta)|`, where
/// `theta` is the angle between the vectors. The inclusive comparison means a
/// vector exactly on the requested boundary is classified as nearly
/// perpendicular.
///
/// [`AngularClassification::Invalid`] is returned if either vector is zero or
/// non-finite, or when the tolerance is non-finite or outside `[0, 1]`.
#[cfg(all(test, any(feature = "std", feature = "libm", feature = "system-libm")))]
pub(crate) fn classify_nearly_perpendicular3<T: RealScalar>(
    left: &Vector3<T>,
    right: &Vector3<T>,
    angular_epsilon: T,
) -> AngularClassification {
    // Normalize each operand once and delegate to the reusable fast path. This
    // wrapper remains useful to sites that do not otherwise need vector metrics.
    let left_normalized = match checked_normalized3(left) {
        Ok(metric) => metric,
        Err(_) => return AngularClassification::Invalid,
    };
    let right_normalized = match checked_normalized3(right) {
        Ok(metric) => metric,
        Err(_) => return AngularClassification::Invalid,
    };
    classify_nearly_perpendicular_normalized3(&left_normalized, &right_normalized, angular_epsilon)
}

/// Trait for 2D swizzle operations on vectors.
///
/// Provides methods to rearrange and duplicate vector components
/// into 2D vectors using familiar shader-style syntax.
pub trait Swizzle2<T: Scalar> {
    /// Returns a vector with components (x, x).
    fn xx(&self) -> Vector2<T>;
    /// Returns a vector with components (x, y).
    fn xy(&self) -> Vector2<T>;
    /// Returns a vector with components (x, z).
    fn xz(&self) -> Vector2<T>;
    /// Returns a vector with components (y, x).
    fn yx(&self) -> Vector2<T>;
    /// Returns a vector with components (y, y).
    fn yy(&self) -> Vector2<T>;
    /// Returns a vector with components (y, z).
    fn yz(&self) -> Vector2<T>;
    /// Returns a vector with components (z, x).
    fn zx(&self) -> Vector2<T>;
    /// Returns a vector with components (z, y).
    fn zy(&self) -> Vector2<T>;
    /// Returns a vector with components (z, z).
    fn zz(&self) -> Vector2<T>;
}

macro_rules! swizzle_field {
    ($s:ident, x) => {
        $s.x
    };
    ($s:ident, y) => {
        $s.y
    };
    ($s:ident, z) => {
        $s.z
    };
    ($s:ident, zero) => {
        <T as Zero>::zero()
    };
}

macro_rules! impl_swizzle2 {
    ($vec:ident, $x:tt, $y:tt, $z:tt) => {
        impl<T: Scalar> Swizzle2<T> for $vec<T> {
            fn xx(&self) -> Vector2<T> {
                Vector2::new(swizzle_field!(self, $x), swizzle_field!(self, $x))
            }
            fn xy(&self) -> Vector2<T> {
                Vector2::new(swizzle_field!(self, $x), swizzle_field!(self, $y))
            }
            fn xz(&self) -> Vector2<T> {
                Vector2::new(swizzle_field!(self, $x), swizzle_field!(self, $z))
            }
            fn yx(&self) -> Vector2<T> {
                Vector2::new(swizzle_field!(self, $y), swizzle_field!(self, $x))
            }
            fn yy(&self) -> Vector2<T> {
                Vector2::new(swizzle_field!(self, $y), swizzle_field!(self, $y))
            }
            fn yz(&self) -> Vector2<T> {
                Vector2::new(swizzle_field!(self, $y), swizzle_field!(self, $z))
            }
            fn zx(&self) -> Vector2<T> {
                Vector2::new(swizzle_field!(self, $z), swizzle_field!(self, $x))
            }
            fn zy(&self) -> Vector2<T> {
                Vector2::new(swizzle_field!(self, $z), swizzle_field!(self, $y))
            }
            fn zz(&self) -> Vector2<T> {
                Vector2::new(swizzle_field!(self, $z), swizzle_field!(self, $z))
            }
        }
    };
}

impl_swizzle2!(Vector2, x, y, zero);
impl_swizzle2!(Vector3, x, y, z);
impl_swizzle2!(Vector4, x, y, z);

/// Trait for 3D swizzle operations on vectors.
///
/// Provides methods to rearrange and duplicate vector components
/// into 3D vectors using familiar shader-style syntax.
pub trait Swizzle3<T: Scalar> {
    /// Returns a vector with components (x, x, x).
    fn xxx(&self) -> Vector3<T>;
    /// Returns a vector with components (x, x, y).
    fn xxy(&self) -> Vector3<T>;
    /// Returns a vector with components (x, x, z).
    fn xxz(&self) -> Vector3<T>;
    /// Returns a vector with components (x, y, x).
    fn xyx(&self) -> Vector3<T>;
    /// Returns a vector with components (x, y, y).
    fn xyy(&self) -> Vector3<T>;
    /// Returns a vector with components (x, y, z).
    fn xyz(&self) -> Vector3<T>;
    /// Returns a vector with components (x, z, x).
    fn xzx(&self) -> Vector3<T>;
    /// Returns a vector with components (x, z, y).
    fn xzy(&self) -> Vector3<T>;
    /// Returns a vector with components (x, z, z).
    fn xzz(&self) -> Vector3<T>;

    /// Returns a vector with components (y, x, x).
    fn yxx(&self) -> Vector3<T>;
    /// Returns a vector with components (y, x, y).
    fn yxy(&self) -> Vector3<T>;
    /// Returns a vector with components (y, x, z).
    fn yxz(&self) -> Vector3<T>;
    /// Returns a vector with components (y, y, x).
    fn yyx(&self) -> Vector3<T>;
    /// Returns a vector with components (y, y, y).
    fn yyy(&self) -> Vector3<T>;
    /// Returns a vector with components (y, y, z).
    fn yyz(&self) -> Vector3<T>;
    /// Returns a vector with components (y, z, x).
    fn yzx(&self) -> Vector3<T>;
    /// Returns a vector with components (y, z, y).
    fn yzy(&self) -> Vector3<T>;
    /// Returns a vector with components (y, z, z).
    fn yzz(&self) -> Vector3<T>;

    /// Returns a vector with components (z, x, x).
    fn zxx(&self) -> Vector3<T>;
    /// Returns a vector with components (z, x, y).
    fn zxy(&self) -> Vector3<T>;
    /// Returns a vector with components (z, x, z).
    fn zxz(&self) -> Vector3<T>;
    /// Returns a vector with components (z, y, x).
    fn zyx(&self) -> Vector3<T>;
    /// Returns a vector with components (z, y, y).
    fn zyy(&self) -> Vector3<T>;
    /// Returns a vector with components (z, y, z).
    fn zyz(&self) -> Vector3<T>;
    /// Returns a vector with components (z, z, x).
    fn zzx(&self) -> Vector3<T>;
    /// Returns a vector with components (z, z, y).
    fn zzy(&self) -> Vector3<T>;
    /// Returns a vector with components (z, z, z).
    fn zzz(&self) -> Vector3<T>;
}

macro_rules! impl_swizzle3 {
    ($vec:ident, $x:tt, $y:tt, $z:tt) => {
        impl<T: Scalar> Swizzle3<T> for $vec<T> {
            fn xxx(&self) -> Vector3<T> {
                Vector3::new(swizzle_field!(self, $x), swizzle_field!(self, $x), swizzle_field!(self, $x))
            }
            fn xxy(&self) -> Vector3<T> {
                Vector3::new(swizzle_field!(self, $x), swizzle_field!(self, $x), swizzle_field!(self, $y))
            }
            fn xxz(&self) -> Vector3<T> {
                Vector3::new(swizzle_field!(self, $x), swizzle_field!(self, $x), swizzle_field!(self, $z))
            }
            fn xyx(&self) -> Vector3<T> {
                Vector3::new(swizzle_field!(self, $x), swizzle_field!(self, $y), swizzle_field!(self, $x))
            }
            fn xyy(&self) -> Vector3<T> {
                Vector3::new(swizzle_field!(self, $x), swizzle_field!(self, $y), swizzle_field!(self, $y))
            }
            fn xyz(&self) -> Vector3<T> {
                Vector3::new(swizzle_field!(self, $x), swizzle_field!(self, $y), swizzle_field!(self, $z))
            }
            fn xzx(&self) -> Vector3<T> {
                Vector3::new(swizzle_field!(self, $x), swizzle_field!(self, $z), swizzle_field!(self, $x))
            }
            fn xzy(&self) -> Vector3<T> {
                Vector3::new(swizzle_field!(self, $x), swizzle_field!(self, $z), swizzle_field!(self, $y))
            }
            fn xzz(&self) -> Vector3<T> {
                Vector3::new(swizzle_field!(self, $x), swizzle_field!(self, $z), swizzle_field!(self, $z))
            }

            fn yxx(&self) -> Vector3<T> {
                Vector3::new(swizzle_field!(self, $y), swizzle_field!(self, $x), swizzle_field!(self, $x))
            }
            fn yxy(&self) -> Vector3<T> {
                Vector3::new(swizzle_field!(self, $y), swizzle_field!(self, $x), swizzle_field!(self, $y))
            }
            fn yxz(&self) -> Vector3<T> {
                Vector3::new(swizzle_field!(self, $y), swizzle_field!(self, $x), swizzle_field!(self, $z))
            }
            fn yyx(&self) -> Vector3<T> {
                Vector3::new(swizzle_field!(self, $y), swizzle_field!(self, $y), swizzle_field!(self, $x))
            }
            fn yyy(&self) -> Vector3<T> {
                Vector3::new(swizzle_field!(self, $y), swizzle_field!(self, $y), swizzle_field!(self, $y))
            }
            fn yyz(&self) -> Vector3<T> {
                Vector3::new(swizzle_field!(self, $y), swizzle_field!(self, $y), swizzle_field!(self, $z))
            }
            fn yzx(&self) -> Vector3<T> {
                Vector3::new(swizzle_field!(self, $y), swizzle_field!(self, $z), swizzle_field!(self, $x))
            }
            fn yzy(&self) -> Vector3<T> {
                Vector3::new(swizzle_field!(self, $y), swizzle_field!(self, $z), swizzle_field!(self, $y))
            }
            fn yzz(&self) -> Vector3<T> {
                Vector3::new(swizzle_field!(self, $y), swizzle_field!(self, $z), swizzle_field!(self, $z))
            }

            fn zxx(&self) -> Vector3<T> {
                Vector3::new(swizzle_field!(self, $z), swizzle_field!(self, $x), swizzle_field!(self, $x))
            }
            fn zxy(&self) -> Vector3<T> {
                Vector3::new(swizzle_field!(self, $z), swizzle_field!(self, $x), swizzle_field!(self, $y))
            }
            fn zxz(&self) -> Vector3<T> {
                Vector3::new(swizzle_field!(self, $z), swizzle_field!(self, $x), swizzle_field!(self, $z))
            }
            fn zyx(&self) -> Vector3<T> {
                Vector3::new(swizzle_field!(self, $z), swizzle_field!(self, $y), swizzle_field!(self, $x))
            }
            fn zyy(&self) -> Vector3<T> {
                Vector3::new(swizzle_field!(self, $z), swizzle_field!(self, $y), swizzle_field!(self, $y))
            }
            fn zyz(&self) -> Vector3<T> {
                Vector3::new(swizzle_field!(self, $z), swizzle_field!(self, $y), swizzle_field!(self, $z))
            }
            fn zzx(&self) -> Vector3<T> {
                Vector3::new(swizzle_field!(self, $z), swizzle_field!(self, $z), swizzle_field!(self, $x))
            }
            fn zzy(&self) -> Vector3<T> {
                Vector3::new(swizzle_field!(self, $z), swizzle_field!(self, $z), swizzle_field!(self, $y))
            }
            fn zzz(&self) -> Vector3<T> {
                Vector3::new(swizzle_field!(self, $z), swizzle_field!(self, $z), swizzle_field!(self, $z))
            }
        }
    };
}

impl_swizzle3!(Vector3, x, y, z);
impl_swizzle3!(Vector4, x, y, z);

#[cfg(all(test, any(feature = "std", feature = "libm", feature = "system-libm")))]
mod tests {
    use super::AngularClassification::{Invalid, OutsideTolerance, WithinTolerance};
    use super::*;
    use crate::scalar::{RealScalar, EPS_F32, EPS_F64};

    /// Asserts that two generic floating-point values differ by at most `tolerance`.
    fn assert_scalar_close<T>(actual: T, expected: T, tolerance: T)
    where
        T: RealScalar + core::fmt::Debug,
    {
        // Use an absolute comparison because these focused fixtures all have
        // normalized or otherwise order-one expected values.
        let difference = (actual - expected).tabs();
        assert!(
            difference <= tolerance,
            "expected {:?} to be within {:?} of {:?}",
            actual,
            tolerance,
            expected,
        );
    }

    /// Exercises stable normalization for one concrete floating-point scalar type.
    fn assert_stable_normalization_contract<T>(
        smallest_positive: T,
        largest_finite: T,
        tolerance: T,
        nan: T,
        infinity: T,
    ) where
        T: RealScalar + core::fmt::Debug,
    {
        // Build all ordinary fixture constants through the scalar traits so the
        // same assertions cover both supported floating-point widths.
        let zero = <T as Zero>::zero();
        let one = <T as One>::one();
        let two = T::two();
        let three = two + one;
        let four = two + two;
        let five = four + one;
        let ten = five * two;

        // Scalar and vector finiteness must accept the complete finite range and
        // reject every IEEE non-finite category.
        assert!(zero.is_finite());
        assert!(largest_finite.is_finite());
        assert!(!nan.is_finite());
        assert!(!infinity.is_finite());
        assert!(!(-infinity).is_finite());
        assert!(Vector3::new(smallest_positive, zero, largest_finite).is_finite());
        assert!(!Vector3::new(nan, zero, zero).is_finite());
        assert!(!Vector3::new(zero, infinity, zero).is_finite());

        // A 3-4-5 vector checks the ordinary unit direction and the method that
        // divides a caller's value by the original, unscaled length.
        let ordinary = checked_normalized3(&Vector3::new(three, four, zero))
            .expect("a finite nonzero vector must normalize");
        let ordinary_unit = ordinary.unit();
        assert_scalar_close(ordinary_unit.x, three / five, tolerance);
        assert_scalar_close(ordinary_unit.y, four / five, tolerance);
        assert_scalar_close(ordinary_unit.z, zero, tolerance);
        assert_scalar_close(ordinary_unit.length(), one, tolerance);
        assert_scalar_close(
            ordinary
                .checked_divide_by_length(ten)
                .expect("10 / length(3, 4, 0) is finite"),
            two,
            tolerance,
        );
        assert_eq!(ordinary.checked_divide_by_length(zero), Ok(zero));

        // A balanced vector with components 3/4 has scale below one but length
        // above one. Dividing MAX by scale first overflows even though dividing
        // MAX by the complete vector length is representable; the alternate
        // ordering must preserve that finite result.
        let three_quarters = three / four;
        let balanced = checked_normalized3(&Vector3::new(
            three_quarters,
            three_quarters,
            three_quarters,
        ))
        .expect("the balanced finite vector must normalize");
        let balanced_quotient = balanced
            .checked_divide_by_length(largest_finite)
            .expect("a representable quotient must survive intermediate overflow");
        assert!(balanced_quotient.is_finite());
        assert!(balanced_quotient > zero);

        // Axis vectors at both exponent extremes would underflow or overflow in
        // a raw sum-of-squares implementation, but max scaling keeps them unit.
        for magnitude in [smallest_positive, largest_finite] {
            let normalized = checked_normalized3(&Vector3::new(magnitude, zero, zero))
                .expect("a finite nonzero axis vector must normalize");
            assert_scalar_close(normalized.unit().x, one, tolerance);
            assert_scalar_close(normalized.unit().y, zero, tolerance);
            assert_scalar_close(normalized.unit().z, zero, tolerance);
            assert_scalar_close(
                normalized
                    .checked_divide_by_length(magnitude)
                    .expect("a magnitude divided by itself is finite"),
                one,
                tolerance,
            );
        }

        // Invalid source vectors and invalid numerators are reported through
        // errors; no NaN or infinity is allowed to escape the checked helpers.
        assert!(checked_normalized3(&Vector3::<T>::zero()).ok().is_none());
        assert!(checked_normalized3(&Vector3::new(nan, zero, zero))
            .ok()
            .is_none());
        assert!(checked_normalized3(&Vector3::new(infinity, zero, zero))
            .ok()
            .is_none());
        assert!(ordinary.checked_divide_by_length(nan).is_err());
        assert!(ordinary.checked_divide_by_length(infinity).is_err());

        // Dividing one by the smallest subnormal exceeds the scalar range. The
        // operation is valid mathematically but deliberately returns an error
        // because this API promises a finite result.
        let smallest = checked_normalized3(&Vector3::new(smallest_positive, zero, zero))
            .expect("the smallest positive axis vector must normalize");
        assert!(smallest.checked_divide_by_length(one).is_err());

        // The opposite exponent pairing has a mathematical nonzero quotient
        // below the scalar's representable range. Scalar checked division
        // permits finite underflow; the nonzero-preserving geometric parameter
        // helper reports an error to keep the parameter consistent with its point.
        let largest = checked_normalized3(&Vector3::new(largest_finite, zero, zero))
            .expect("the largest finite axis vector must normalize");
        assert_eq!(
            largest.checked_divide_by_length(smallest_positive),
            Ok(zero)
        );
        assert!(largest
            .checked_divide_by_length_preserving_nonzero(smallest_positive)
            .is_err());
    }

    /// Exercises dimensionless parallel/perpendicular classification for one scalar type.
    fn assert_orientation_contract<T>(
        smallest_positive: T,
        largest_finite: T,
        angular_epsilon: T,
        nan: T,
        infinity: T,
    ) where
        T: RealScalar + core::fmt::Debug,
    {
        // Define basis and oblique directions without concrete float literals.
        let zero = <T as Zero>::zero();
        let one = <T as One>::one();
        let two = T::two();
        let three = two + one;
        let four = two + two;
        let five = four + one;
        let x = Vector3::new(one, zero, zero);
        let y = Vector3::new(zero, one, zero);
        let negative_x = Vector3::new(-one, zero, zero);
        let oblique = Vector3::new(one, one, zero);

        // Exact basis relationships establish parallel, anti-parallel,
        // perpendicular, and clearly oblique classifications.
        assert_eq!(
            classify_nearly_parallel3(&x, &x, angular_epsilon),
            WithinTolerance
        );
        assert_eq!(
            classify_nearly_parallel3(&x, &negative_x, angular_epsilon),
            WithinTolerance
        );
        assert_eq!(
            classify_nearly_parallel3(&x, &y, angular_epsilon),
            OutsideTolerance
        );
        assert_eq!(
            classify_nearly_parallel3(&x, &oblique, angular_epsilon),
            OutsideTolerance
        );
        assert_eq!(
            classify_nearly_perpendicular3(&x, &y, angular_epsilon),
            WithinTolerance
        );
        assert_eq!(
            classify_nearly_perpendicular3(&x, &x, angular_epsilon),
            OutsideTolerance
        );
        assert_eq!(
            classify_nearly_perpendicular3(&x, &oblique, angular_epsilon),
            OutsideTolerance
        );

        // Perturbations on opposite sides of the tolerance verify that the
        // helpers measure angle rather than raw dot/cross magnitude.
        let half_epsilon = angular_epsilon * T::half();
        let twice_epsilon = angular_epsilon * two;
        let near_parallel = Vector3::new(one, half_epsilon, zero);
        let not_near_parallel = Vector3::new(one, twice_epsilon, zero);
        assert_eq!(
            classify_nearly_parallel3(&x, &near_parallel, angular_epsilon),
            WithinTolerance
        );
        assert_eq!(
            classify_nearly_parallel3(&x, &not_near_parallel, angular_epsilon),
            OutsideTolerance
        );
        let near_perpendicular = Vector3::new(half_epsilon, one, zero);
        let not_near_perpendicular = Vector3::new(twice_epsilon, one, zero);
        assert_eq!(
            classify_nearly_perpendicular3(&x, &near_perpendicular, angular_epsilon),
            WithinTolerance
        );
        assert_eq!(
            classify_nearly_perpendicular3(&x, &not_near_perpendicular, angular_epsilon),
            OutsideTolerance
        );

        // The 3-4-5 directions place sine or cosine exactly at 3/5. These
        // assertions lock in the documented inclusive boundary convention.
        let boundary = three / five;
        assert_eq!(
            classify_nearly_parallel3(&x, &Vector3::new(four, three, zero), boundary),
            WithinTolerance
        );
        assert_eq!(
            classify_nearly_parallel3(&x, &Vector3::new(four, three, zero), boundary * T::half(),),
            OutsideTolerance
        );
        assert_eq!(
            classify_nearly_perpendicular3(&x, &Vector3::new(three, four, zero), boundary),
            WithinTolerance
        );
        assert_eq!(
            classify_nearly_perpendicular3(
                &x,
                &Vector3::new(three, four, zero),
                boundary * T::half(),
            ),
            OutsideTolerance
        );

        // Independently rescale each operand across the scalar exponent range.
        // Classification must remain unchanged even for subnormal and maximum
        // finite axis magnitudes.
        for left_scale in [smallest_positive, one, largest_finite] {
            for right_scale in [smallest_positive, one, largest_finite] {
                let scaled_x = Vector3::new(left_scale, zero, zero);
                let scaled_negative_x = Vector3::new(-right_scale, zero, zero);
                let scaled_y = Vector3::new(zero, right_scale, zero);
                assert_eq!(
                    classify_nearly_parallel3(&scaled_x, &scaled_negative_x, angular_epsilon,),
                    WithinTolerance
                );
                assert_eq!(
                    classify_nearly_perpendicular3(&scaled_x, &scaled_y, angular_epsilon),
                    WithinTolerance
                );
            }
        }

        // Invalid tolerances and indeterminate operands must produce the named
        // invalid state instead of being silently assigned a geometric answer.
        for invalid_epsilon in [-angular_epsilon, one + angular_epsilon, nan, infinity] {
            assert_eq!(classify_nearly_parallel3(&x, &y, invalid_epsilon), Invalid);
            assert_eq!(
                classify_nearly_perpendicular3(&x, &y, invalid_epsilon),
                Invalid
            );
        }
        let zero_vector = Vector3::zero();
        let nan_vector = Vector3::new(nan, zero, zero);
        let infinite_vector = Vector3::new(zero, infinity, zero);
        for invalid_vector in [zero_vector, nan_vector, infinite_vector] {
            assert_eq!(
                classify_nearly_parallel3(&x, &invalid_vector, angular_epsilon),
                Invalid
            );
            assert_eq!(
                classify_nearly_perpendicular3(&x, &invalid_vector, angular_epsilon),
                Invalid
            );
        }
    }

    /// Verifies that squared comparisons retain subnormal angular information.
    fn assert_subnormal_angular_epsilon_contract<T>(smallest_positive: T)
    where
        T: RealScalar + core::fmt::Debug,
    {
        // A unit x direction crossed with `(1, delta, 0)` has a z component of
        // `delta`, but squaring a minimum subnormal delta produces zero. These
        // fixtures therefore force both exact-zero and positive-underflow
        // branches of the normalized parallel predicate.
        let zero = <T as Zero>::zero();
        let one = <T as One>::one();
        let two = T::two();
        let x = Vector3::new(one, zero, zero);
        let at_boundary = Vector3::new(one, smallest_positive, zero);
        let beyond_boundary = Vector3::new(one, smallest_positive * two, zero);

        // Prepare each input once to exercise the production fast path used by
        // intersection queries, rather than relying only on its raw test wrapper.
        let normalized_x = checked_normalized3(&x).expect("the x axis must normalize");
        let normalized_boundary =
            checked_normalized3(&at_boundary).expect("the subnormal direction must normalize");
        let normalized_beyond =
            checked_normalized3(&beyond_boundary).expect("the doubled direction must normalize");

        // The inclusive positive boundary remains true even though both the
        // cross magnitude squared and epsilon squared round to zero.
        assert_eq!(
            classify_nearly_parallel_normalized3(
                &normalized_x,
                &normalized_boundary,
                smallest_positive,
            ),
            WithinTolerance,
        );

        // Exact epsilon zero must observe the nonzero subnormal cross component
        // directly instead of confusing squared underflow with exact parallelism.
        assert_eq!(
            classify_nearly_parallel_normalized3(&normalized_x, &normalized_boundary, zero),
            OutsideTolerance,
        );
        assert_eq!(
            classify_nearly_parallel_normalized3(&normalized_x, &normalized_x, zero),
            WithinTolerance,
        );

        // Doubling the angular perturbation puts it outside the one-subnormal
        // tolerance and confirms that the fallback does not accept every value
        // whose square happens to underflow.
        assert_eq!(
            classify_nearly_parallel_normalized3(
                &normalized_x,
                &normalized_beyond,
                smallest_positive,
            ),
            OutsideTolerance,
        );

        // The raw-vector wrapper must retain the same decisions while preparing
        // each input exactly once internally.
        assert_eq!(
            classify_nearly_parallel3(&x, &at_boundary, smallest_positive),
            WithinTolerance,
        );
        assert_eq!(
            classify_nearly_parallel3(&x, &at_boundary, zero),
            OutsideTolerance
        );
        assert_eq!(
            classify_nearly_parallel3(&x, &beyond_boundary, smallest_positive),
            OutsideTolerance,
        );
    }

    /// Verifies max-component normalization across the full `f32` exponent range.
    #[test]
    fn test_stable_normalization_f32() {
        // The smallest positive subnormal specifically exercises the underflow
        // case that raw sum-of-squares normalization cannot represent.
        assert_stable_normalization_contract(
            f32::from_bits(1),
            f32::MAX,
            EPS_F32,
            f32::NAN,
            f32::INFINITY,
        );
    }

    /// Verifies max-component normalization across the full `f64` exponent range.
    #[test]
    fn test_stable_normalization_f64() {
        // Use the same bit-level subnormal fixture as the f32 test to ensure the
        // generic algorithm does not depend on a particular exponent width.
        assert_stable_normalization_contract(
            f64::from_bits(1),
            f64::MAX,
            EPS_F64,
            f64::NAN,
            f64::INFINITY,
        );
    }

    /// Verifies that both length-division policies retain the sign of an exact
    /// zero numerator for each supported floating-point scalar type.
    #[test]
    fn test_stable_length_division_preserves_signed_zero() {
        // Use a non-axis-aligned vector so preservation is checked across both
        // divisions in the ordinary path, not only its scaled-length-one shortcut.
        let normalized_f32 = checked_normalized3(&Vector3::new(3.0f32, 4.0, 0.0))
            .expect("the finite nonzero f32 vector must normalize");
        let negative_zero_f32 = -0.0f32;
        let divided_f32 = normalized_f32
            .checked_divide_by_length(negative_zero_f32)
            .expect("signed zero divided by a positive length must remain finite");
        let strict_f32 = normalized_f32
            .checked_divide_by_length_preserving_nonzero(negative_zero_f32)
            .expect("exact signed zero is valid under the strict policy");
        assert_eq!(divided_f32.to_bits(), negative_zero_f32.to_bits());
        assert_eq!(strict_f32.to_bits(), negative_zero_f32.to_bits());

        // Repeat at f64 so backend-independent generic code cannot accidentally
        // preserve the sign for only one concrete scalar implementation.
        let normalized_f64 = checked_normalized3(&Vector3::new(3.0f64, 4.0, 0.0))
            .expect("the finite nonzero f64 vector must normalize");
        let negative_zero_f64 = -0.0f64;
        let divided_f64 = normalized_f64
            .checked_divide_by_length(negative_zero_f64)
            .expect("signed zero divided by a positive length must remain finite");
        let strict_f64 = normalized_f64
            .checked_divide_by_length_preserving_nonzero(negative_zero_f64)
            .expect("exact signed zero is valid under the strict policy");
        assert_eq!(divided_f64.to_bits(), negative_zero_f64.to_bits());
        assert_eq!(strict_f64.to_bits(), negative_zero_f64.to_bits());
    }

    /// Verifies the shared checked-query gate's exhaustive three-state policy.

    /// Verifies scale-invariant angular classification for `f32` vectors.
    #[test]
    fn test_orientation_predicates_f32() {
        // A tolerance well above rounding noise gives stable fixtures on both
        // sides of the near-orientation boundary.
        assert_orientation_contract(
            f32::from_bits(1),
            f32::MAX,
            0.001f32,
            f32::NAN,
            f32::INFINITY,
        );
    }

    /// Verifies scale-invariant angular classification for `f64` vectors.
    #[test]
    fn test_orientation_predicates_f64() {
        // Match the f32 geometry exactly so both implementations exercise the
        // same semantic contract rather than type-specific special cases.
        assert_orientation_contract(
            f64::from_bits(1),
            f64::MAX,
            0.001f64,
            f64::NAN,
            f64::INFINITY,
        );
    }

    /// Verifies the squared-epsilon fallback at the bottom of the `f32` range.
    #[test]
    fn test_subnormal_angular_epsilon_f32() {
        // Bit pattern one is the smallest positive subnormal and therefore
        // guarantees that multiplying it by itself rounds to zero.
        assert_subnormal_angular_epsilon_contract(f32::from_bits(1));
    }

    /// Verifies the squared-epsilon fallback at the bottom of the `f64` range.
    #[test]
    fn test_subnormal_angular_epsilon_f64() {
        // Mirror the f32 fixture at f64 width so generic code cannot accidentally
        // depend on the exponent or mantissa range of one scalar type.
        assert_subnormal_angular_epsilon_contract(f64::from_bits(1));
    }

    #[test]
    pub fn test() {
        let f1 = Vector2 { x: 1.0, y: 2.0 };
        let f2 = Vector2 { x: 3.0, y: 4.0 };
        let out = f1 + f2;
        assert_eq!(out.x, 4.0);
        assert_eq!(out.y, 6.0);

        let f22: Vector2<f32> = 2.0 * f2;
        let f23: Vector2<f32> = f2 * 2.0;

        assert_eq!(f22.x, 6.0);
        assert_eq!(f22.y, 8.0);
        assert_eq!(f23.x, f22.x);
        assert_eq!(f23.y, f22.y);
    }

    #[test]
    fn test_vector_normalization() {
        // Test normal case
        let v = Vector3::<f32>::new(3.0, 4.0, 0.0);
        let nv = v.normalize();
        let len = nv.length();
        assert!((len - 1.0).abs() < f32::epsilon());

        // Test zero vector normalization (should handle gracefully)
        let v_zero = Vector3::<f32>::new(0.0, 0.0, 0.0);
        let nv_zero = v_zero.normalize();
        // When normalizing zero vector, we get NaN or Inf components
        // The current implementation divides by zero, resulting in inf/nan
        assert!(nv_zero.x.is_infinite() || nv_zero.x.is_nan());

        // Test already normalized vector
        let v_unit = Vector3::<f32>::new(1.0, 0.0, 0.0);
        let nv_unit = v_unit.normalize();
        assert!((nv_unit.x - 1.0).abs() < f32::epsilon());
        assert!((nv_unit.y).abs() < f32::epsilon());
        assert!((nv_unit.z).abs() < f32::epsilon());
    }

    #[test]
    fn test_vector_checked_normalize() {
        let v = Vector3::<f32>::new(3.0, 4.0, 0.0);
        let nv = v.checked_normalize(EPS_F32).expect("should normalize");
        assert!((nv.length() - 1.0).abs() < 0.001);

        let zero = Vector3::<f32>::zero();
        assert!(zero.checked_normalize(EPS_F32).ok().is_none());
        let zero_norm = zero.normalize_or_zero(EPS_F32);
        assert_eq!(zero_norm.x, 0.0);
        assert_eq!(zero_norm.y, 0.0);
        assert_eq!(zero_norm.z, 0.0);

        let len_sq = v.length_squared();
        let inv_len = 1.0f32 / len_sq.tsqrt();
        let nv_fast = v.normalize_with_inv_len(inv_len);
        assert!((nv_fast.length() - 1.0).abs() < 0.001);

        let nv_fast_try = v
            .checked_normalize_with_inv_len(len_sq, inv_len, EPS_F32)
            .expect("should normalize");
        assert!((nv_fast_try.length() - 1.0).abs() < 0.001);
        assert!(zero
            .checked_normalize_with_inv_len(0.0, 0.0, EPS_F32)
            .ok()
            .is_none());
    }

    #[test]
    fn test_vector_length() {
        let v2 = Vector2::<f32>::new(3.0, 4.0);
        assert!((v2.length() - 5.0).abs() < f32::epsilon());

        let v3 = Vector3::<f32>::new(2.0, 3.0, 6.0);
        assert!((v3.length() - 7.0).abs() < f32::epsilon());

        let v4 = Vector4::<f32>::new(1.0, 2.0, 2.0, 0.0);
        assert!((v4.length() - 3.0).abs() < f32::epsilon());

        // Test zero vector
        let v_zero = Vector3::<f32>::zero();
        assert_eq!(v_zero.length(), 0.0);
    }

    #[test]
    fn test_vector_dot_product() {
        let v1 = Vector3::<f32>::new(1.0, 2.0, 3.0);
        let v2 = Vector3::<f32>::new(4.0, 5.0, 6.0);
        let dot = Vector3::dot(&v1, &v2);
        assert_eq!(dot, 32.0); // 1*4 + 2*5 + 3*6 = 32

        // Test orthogonal vectors
        let v_ortho1 = Vector3::<f32>::new(1.0, 0.0, 0.0);
        let v_ortho2 = Vector3::<f32>::new(0.0, 1.0, 0.0);
        assert_eq!(Vector3::dot(&v_ortho1, &v_ortho2), 0.0);

        // Test dot product with itself equals length squared
        let self_dot = Vector3::dot(&v1, &v1);
        let len_squared = v1.length() * v1.length();
        assert!((self_dot - len_squared).abs() < 0.0001);
    }

    #[test]
    fn test_vector_cross_product() {
        // Test standard basis vectors
        let x = Vector3::<f32>::new(1.0, 0.0, 0.0);
        let y = Vector3::<f32>::new(0.0, 1.0, 0.0);
        let z = Vector3::<f32>::new(0.0, 0.0, 1.0);

        let x_cross_y = Vector3::cross(&x, &y);
        assert!((x_cross_y.x - z.x).abs() < f32::epsilon());
        assert!((x_cross_y.y - z.y).abs() < f32::epsilon());
        assert!((x_cross_y.z - z.z).abs() < f32::epsilon());

        let y_cross_z = Vector3::cross(&y, &z);
        assert!((y_cross_z.x - x.x).abs() < f32::epsilon());
        assert!((y_cross_z.y - x.y).abs() < f32::epsilon());
        assert!((y_cross_z.z - x.z).abs() < f32::epsilon());

        let z_cross_x = Vector3::cross(&z, &x);
        assert!((z_cross_x.x - y.x).abs() < f32::epsilon());
        assert!((z_cross_x.y - y.y).abs() < f32::epsilon());
        assert!((z_cross_x.z - y.z).abs() < f32::epsilon());

        // Test anti-commutativity: a × b = -(b × a)
        let a = Vector3::<f32>::new(1.0, 2.0, 3.0);
        let b = Vector3::<f32>::new(4.0, 5.0, 6.0);
        let a_cross_b = Vector3::cross(&a, &b);
        let b_cross_a = Vector3::cross(&b, &a);
        assert!((a_cross_b.x + b_cross_a.x).abs() < f32::epsilon());
        assert!((a_cross_b.y + b_cross_a.y).abs() < f32::epsilon());
        assert!((a_cross_b.z + b_cross_a.z).abs() < f32::epsilon());

        // Test cross product with itself is zero
        let self_cross = Vector3::cross(&a, &a);
        assert!(self_cross.x.abs() < f32::epsilon());
        assert!(self_cross.y.abs() < f32::epsilon());
        assert!(self_cross.z.abs() < f32::epsilon());
    }

    #[test]
    fn test_vector_distance() {
        let v1 = Vector3::<f32>::new(1.0, 2.0, 3.0);
        let v2 = Vector3::<f32>::new(4.0, 6.0, 3.0);
        let dist = Vector3::distance(&v1, &v2);
        assert!((dist - 5.0).abs() < f32::epsilon()); // sqrt(9 + 16 + 0) = 5

        // Distance to itself should be zero
        let self_dist = Vector3::distance(&v1, &v1);
        assert!(self_dist.abs() < f32::epsilon());
    }

    #[test]
    fn test_vector_min_max() {
        let v1 = Vector3::<f32>::new(1.0, 5.0, 3.0);
        let v2 = Vector3::<f32>::new(4.0, 2.0, 6.0);

        let v_min = Vector3::min(&v1, &v2);
        assert_eq!(v_min.x, 1.0);
        assert_eq!(v_min.y, 2.0);
        assert_eq!(v_min.z, 3.0);

        let v_max = Vector3::max(&v1, &v2);
        assert_eq!(v_max.x, 4.0);
        assert_eq!(v_max.y, 5.0);
        assert_eq!(v_max.z, 6.0);
    }

    #[test]
    fn test_vector_arithmetic() {
        let v1 = Vector3::<f32>::new(1.0, 2.0, 3.0);
        let v2 = Vector3::<f32>::new(4.0, 5.0, 6.0);

        // Addition
        let sum = v1 + v2;
        assert_eq!(sum.x, 5.0);
        assert_eq!(sum.y, 7.0);
        assert_eq!(sum.z, 9.0);

        // Subtraction
        let diff = v2 - v1;
        assert_eq!(diff.x, 3.0);
        assert_eq!(diff.y, 3.0);
        assert_eq!(diff.z, 3.0);

        // Component-wise multiplication
        let prod = v1 * v2;
        assert_eq!(prod.x, 4.0);
        assert_eq!(prod.y, 10.0);
        assert_eq!(prod.z, 18.0);

        // Component-wise division
        let div = v2 / v1;
        assert_eq!(div.x, 4.0);
        assert_eq!(div.y, 2.5);
        assert_eq!(div.z, 2.0);

        // Scalar multiplication
        let scaled = v1 * 2.0;
        assert_eq!(scaled.x, 2.0);
        assert_eq!(scaled.y, 4.0);
        assert_eq!(scaled.z, 6.0);

        // Scalar division
        let divided = v2 / 2.0;
        assert_eq!(divided.x, 2.0);
        assert_eq!(divided.y, 2.5);
        assert_eq!(divided.z, 3.0);

        // Negation
        let neg = -v1;
        assert_eq!(neg.x, -1.0);
        assert_eq!(neg.y, -2.0);
        assert_eq!(neg.z, -3.0);
    }

    #[test]
    fn test_swizzle_operations() {
        let v2 = Vector2::<f32>::new(1.0, 2.0);

        assert_eq!(v2.xx().x, 1.0);
        assert_eq!(v2.xx().y, 1.0);

        assert_eq!(v2.xy().x, 1.0);
        assert_eq!(v2.xy().y, 2.0);

        assert_eq!(v2.yx().x, 2.0);
        assert_eq!(v2.yx().y, 1.0);

        assert_eq!(v2.yy().x, 2.0);
        assert_eq!(v2.yy().y, 2.0);

        let v2_xz = v2.xz();
        assert_eq!(v2_xz.x, 1.0);
        assert_eq!(v2_xz.y, 0.0);

        let v3 = Vector3::<f32>::new(1.0, 2.0, 3.0);

        assert_eq!(v3.xz().x, 1.0);
        assert_eq!(v3.xz().y, 3.0);

        assert_eq!(v3.zy().x, 3.0);
        assert_eq!(v3.zy().y, 2.0);

        let v3_swizzle = v3.zyx();
        assert_eq!(v3_swizzle.x, 3.0);
        assert_eq!(v3_swizzle.y, 2.0);
        assert_eq!(v3_swizzle.z, 1.0);

        let v4 = Vector4::<f32>::new(1.0, 2.0, 3.0, 4.0);
        let v4_xz = v4.xz();
        assert_eq!(v4_xz.x, 1.0);
        assert_eq!(v4_xz.y, 3.0);
        let v4_swizzle = v4.zyx();
        assert_eq!(v4_swizzle.x, 3.0);
        assert_eq!(v4_swizzle.y, 2.0);
        assert_eq!(v4_swizzle.z, 1.0);
    }

    #[test]
    fn test_vector_rem_operation() {
        let v1 = Vector3::<i32>::new(10, 15, 20);
        let v2 = Vector3::<i32>::new(3, 4, 6);

        let rem = v1 % v2;
        assert_eq!(rem.x, 1); // 10 % 3 = 1
        assert_eq!(rem.y, 3); // 15 % 4 = 3
        assert_eq!(rem.z, 2); // 20 % 6 = 2

        let v3 = Vector3::<i32>::new(10, 15, 20);
        let rem_scalar = v3 % 7;
        assert_eq!(rem_scalar.x, 3); // 10 % 7 = 3
        assert_eq!(rem_scalar.y, 1); // 15 % 7 = 1
        assert_eq!(rem_scalar.z, 6); // 20 % 7 = 6
    }

    #[test]
    fn test_vector_try_cast() {
        let vi = Vector3::<i32>::new(1, 2, 3);
        let vf = vi
            .try_cast::<f32>()
            .expect("integer vector should cast to f32");
        assert_eq!(vf.x, 1.0);
        assert_eq!(vf.y, 2.0);
        assert_eq!(vf.z, 3.0);
    }
}
