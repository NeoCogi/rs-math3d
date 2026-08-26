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
//! [`FloatVector`].
//!
//! Intersection code also uses crate-private, max-component-scaled 3D
//! normalization helpers. Those helpers keep angular decisions independent of
//! a vector's magnitude without changing the behavior of the public
//! [`FloatVector`] normalization methods.
//!
//! # Examples
//!
//! ```
//! use rs_math3d::vector::{FloatVector, Vector, Vector3};
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
//! ```

use crate::scalar::*;
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

    /// Returns a vector with the minimum components from both vectors.
    fn min(l: &Self, r: &Self) -> Self;

    /// Returns a vector with the maximum components from both vectors.
    fn max(l: &Self, r: &Self) -> Self;
}

/// Trait for vectors with floating-point components.
///
/// Extends the base `Vector` trait with operations that require
/// floating-point arithmetic, such as length calculation and normalization.
pub trait FloatVector<T: FloatScalar>: Vector<T> {
    /// Computes the Euclidean length (magnitude) of the vector.
    ///
    /// For vector **v**:
    /// ```text
    /// ||v|| = √(v₁² + v₂² + ... + vₙ²)
    /// ```
    fn length(&self) -> T;

    /// Returns a unit vector in the same direction.
    ///
    /// For vector **v**:
    /// ```text
    /// v̂ = v / ||v||
    /// ```
    ///
    /// # Note
    /// Returns NaN or Inf components if the vector has zero length.
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

    /// Returns a normalized vector or `None` when the length is too small.
    fn try_normalize(&self, epsilon: T) -> Option<Self> {
        let len_sq = self.length_squared();
        if len_sq <= epsilon * epsilon {
            None
        } else {
            Some(*self / len_sq.tsqrt())
        }
    }

    /// Returns a normalized vector or the zero vector when too small.
    fn normalize_or_zero(&self, epsilon: T) -> Self {
        self.try_normalize(epsilon).unwrap_or_else(Self::zero)
    }

    /// Normalizes using a precomputed inverse length (e.g., from rsqrt).
    fn normalize_with_inv_len(&self, inv_len: T) -> Self {
        *self * inv_len
    }

    /// Normalizes with precomputed length squared and inverse length.
    fn try_normalize_with_inv_len(&self, len_sq: T, inv_len: T, epsilon: T) -> Option<Self> {
        if len_sq <= epsilon * epsilon {
            None
        } else {
            Some(*self * inv_len)
        }
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
    ($(#[$meta:meta])* $vecName:ident, $($field:ident)*) => {
        $(#[$meta])*
        #[repr(C)]
        #[derive(Copy, Clone, Debug, Default)]
        pub struct $vecName<T> { $(#[doc = vector_field_doc!($field)] pub $field: T),* }

        impl<T: Scalar> $vecName<T> {
            /// Creates a vector from components.
            pub fn new($($field:T),*) -> Self { Self { $($field: $field),* } }
        }

        impl<T> $vecName<T>
        where
            T: Copy + num_traits::ToPrimitive,
        {
            /// Attempts to cast each component into another numeric type.
            pub fn try_cast<U>(&self) -> Option<$vecName<U>>
            where
                U: Scalar + num_traits::NumCast,
            {
                Some($vecName {
                    $($field: num_traits::NumCast::from(self.$field)?),*
                })
            }
        }

        impl<T: Scalar> Vector<T> for $vecName<T> {
            fn zero() -> Self { Self { $($field: <T as Zero>::zero()),* } }
            fn dot  (l: &Self, r: &Self) -> T { $(l.$field * r.$field +)* <T as Zero>::zero() }
            fn add_vv(l: &Self, r: &Self) -> Self { Self::new($(l.$field + r.$field),*) }
            fn sub_vv(l: &Self, r: &Self) -> Self { Self::new($(l.$field - r.$field),*) }
            fn mul_vv(l: &Self, r: &Self) -> Self { Self::new($(l.$field * r.$field),*) }
            fn div_vv(l: &Self, r: &Self) -> Self { Self::new($(l.$field / r.$field),*) }
            fn mul_vs(l: &Self, r: T) -> Self { Self::new($(l.$field * r),*) }
            fn div_vs(l: &Self, r: T) -> Self { Self::new($(l.$field / r),*) }
            fn rem_vv(l: &Self, r: &Self) -> Self { Self::new($(l.$field % r.$field),*) }
            fn min(l: &Self, r: &Self) -> Self { Self::new($(T::min(l.$field, r.$field)),*) }
            fn max(l: &Self, r: &Self) -> Self { Self::new($(T::max(l.$field, r.$field)),*) }
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

macro_rules! implFloatVector {
    ($vecName:ident) => {
        impl<T: FloatScalar> FloatVector<T> for $vecName<T> {
            fn length(&self) -> T {
                Self::dot(self, self).tsqrt()
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
/// The cross product is only defined for 3D vectors and produces
/// a vector perpendicular to both input vectors.
pub trait CrossProduct {
    /// Computes the cross product of two vectors.
    ///
    /// For 3D vectors **v** and **w**:
    /// ```text
    /// v × w = (v₂w₃ - v₃w₂, v₃w₁ - v₁w₃, v₁w₂ - v₂w₁)
    /// ```
    ///
    /// The resulting vector is perpendicular to both input vectors,
    /// with magnitude ||v|| ||w|| sin(θ), where θ is the angle between them.
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
    Vector2,
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
    Vector3,
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
    Vector4,
    x y z w
);

implFloatVector!(Vector2);
implFloatVector!(Vector3);
implFloatVector!(Vector4);

impl<T> CrossProduct for Vector3<T>
where
    T: Scalar,
{
    /// Computes the cross product of two 3D vectors.
    ///
    /// The cross product **v** × **w** produces a vector perpendicular
    /// to both **v** and **w**, following the right-hand rule.
    ///
    /// # Properties
    /// - Anti-commutative: **v** × **w** = -(**w** × **v**)
    /// - Distributive: **v** × (**w** + **u**) = **v** × **w** + **v** × **u**
    /// - **v** × **v** = **0**
    fn cross(l: &Vector3<T>, r: &Vector3<T>) -> Vector3<T> {
        Vector3::new(
            l.y * r.z - l.z * r.y,
            l.z * r.x - l.x * r.z,
            l.x * r.y - l.y * r.x,
        )
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
/// - `scaled_length` is the length of `original / scale`, and is finite and at
///   least one;
/// - `unit` is `original / (scale * scaled_length)`, without requiring that
///   potentially overflowing product to be formed.
///
/// The type is crate-private because it supplies numerical machinery for
/// geometric queries rather than a second public vector-normalization API.
#[derive(Clone, Copy, Debug)]
pub(crate) struct Normalized3<T: FloatScalar> {
    /// Unit-length direction of the original vector.
    unit: Vector3<T>,
    /// Largest absolute component of the original vector.
    scale: T,
    /// Length of the max-component-scaled vector `original / scale`.
    scaled_length: T,
}

impl<T: FloatScalar> Normalized3<T> {
    /// Returns the finite unit vector represented by this decomposition.
    pub(crate) fn unit(&self) -> Vector3<T> {
        // `Normalized3` can only be constructed by `try_normalized3`, which
        // establishes that every stored unit-vector component is finite.
        self.unit
    }

    /// Divides a finite scalar by the original vector's Euclidean length.
    ///
    /// The calculation avoids constructing `scale * scaled_length`, which can
    /// overflow even when the requested quotient is representable. It first
    /// divides by `scale` to preserve ratios between similarly tiny values. If
    /// that intermediate overflows, it retries by dividing by the bounded
    /// `scaled_length` first. `None` is returned when `value` is non-finite or
    /// neither ordering produces a finite quotient.
    pub(crate) fn divide_by_length(&self, value: T) -> Option<T> {
        // Reject NaN and infinity before arithmetic so they cannot silently
        // contaminate an otherwise valid geometric query.
        if !is_finite_scalar(value) {
            return None;
        }

        // Divide quantities of comparable scale first. In particular, this
        // preserves ratios between equally tiny or equally large values.
        let scale_adjusted = value / self.scale;
        if is_finite_scalar(scale_adjusted) {
            // The remaining divisor is in [1, sqrt(3)], so this step cannot
            // enlarge a finite intermediate for the built-in float types.
            let quotient = scale_adjusted / self.scaled_length;
            if is_finite_scalar(quotient) {
                return Some(quotient);
            }
        }

        // A value/scale intermediate can overflow even when the final quotient
        // is finite: for example, when scale < 1 but the complete vector length
        // is > 1. Dividing by the bounded factor first recovers that case.
        let length_adjusted = value / self.scaled_length;
        if !is_finite_scalar(length_adjusted) {
            return None;
        }
        let quotient = length_adjusted / self.scale;
        is_finite_scalar(quotient).then_some(quotient)
    }
}

/// Returns whether a floating-point scalar is neither NaN nor infinity.
///
/// `FloatScalar` deliberately exposes only the analytic operations needed by
/// this crate, not an `is_finite` method. A finite magnitude is strictly below
/// the trait's positive-infinity value; comparisons with NaN evaluate to
/// `false`, so this single comparison rejects all three non-finite cases.
pub(crate) fn is_finite_scalar<T: FloatScalar>(value: T) -> bool {
    // Absolute value maps both infinities to positive infinity while leaving a
    // NaN unordered, making the strict comparison sufficient for both cases.
    value.tabs() < T::infinity()
}

/// Returns whether all three components of a 3D vector are finite.
pub(crate) fn is_finite_vector3<T: FloatScalar>(vector: &Vector3<T>) -> bool {
    // Test every component independently; accepting even one NaN or infinity
    // would make max-component scaling and orientation comparisons unordered.
    is_finite_scalar(vector.x) && is_finite_scalar(vector.y) && is_finite_scalar(vector.z)
}

/// Normalizes a finite, nonzero 3D vector without squaring its raw components.
///
/// The largest absolute component is removed before computing a squared
/// length. The scaled components lie in `[-1, 1]`, so their squared sum cannot
/// overflow and a nonzero subnormal input is not lost to underflow. `None` is
/// returned for zero vectors and vectors containing NaN or infinity.
pub(crate) fn try_normalized3<T: FloatScalar>(vector: &Vector3<T>) -> Option<Normalized3<T>> {
    // A non-finite component could be hidden by generic min/max ordering, so
    // reject the complete input before selecting its largest magnitude.
    if !is_finite_vector3(vector) {
        return None;
    }

    // Scaling by the largest component bounds every subsequent component and
    // guarantees that at least one scaled component has magnitude exactly one.
    let scale = T::max(vector.x.tabs(), T::max(vector.y.tabs(), vector.z.tabs()));
    if scale <= <T as Zero>::zero() {
        return None;
    }

    // Form the bounded vector before taking its squared length. This is the
    // operation that removes dependence on the original exponent range.
    let scaled = *vector / scale;
    let scaled_length_squared = Vector3::dot(&scaled, &scaled);
    if !is_finite_scalar(scaled_length_squared) || scaled_length_squared <= <T as Zero>::zero() {
        return None;
    }

    // The square root lies in [1, sqrt(3)] for an IEEE three-component input.
    // Validate it explicitly because `FloatScalar` is implementable outside
    // this crate and may use a different analytic backend.
    let scaled_length = scaled_length_squared.tsqrt();
    if !is_finite_scalar(scaled_length) || scaled_length <= <T as Zero>::zero() {
        return None;
    }

    // Dividing the already bounded vector produces the requested direction
    // without ever constructing the possibly unrepresentable original length.
    let unit = scaled / scaled_length;
    if !is_finite_vector3(&unit) {
        return None;
    }

    Some(Normalized3 {
        unit,
        scale,
        scaled_length,
    })
}

/// Tests whether two finite, nonzero vectors are parallel within an angular tolerance.
///
/// `angular_epsilon` is a dimensionless upper bound on `|sin(theta)|`, where
/// `theta` is the angle between the vectors. Parallel and anti-parallel vectors
/// are treated identically. The inclusive comparison means a vector exactly on
/// the requested boundary is classified as nearly parallel.
///
/// `None` is returned if either vector is zero or non-finite, or when the
/// tolerance is non-finite or outside the closed interval `[0, 1]`.
pub(crate) fn try_nearly_parallel3<T: FloatScalar>(
    left: &Vector3<T>,
    right: &Vector3<T>,
    angular_epsilon: T,
) -> Option<bool> {
    // Angular sine magnitudes live in [0, 1]. Rejecting any other tolerance
    // keeps the classification meaningful and prevents NaN comparisons.
    if !is_finite_scalar(angular_epsilon)
        || angular_epsilon < <T as Zero>::zero()
        || angular_epsilon > <T as One>::one()
    {
        return None;
    }

    // Normalize with max-component scaling so independently rescaling either
    // operand cannot change the classification through raw overflow/underflow.
    let left_unit = try_normalized3(left)?.unit();
    let right_unit = try_normalized3(right)?.unit();

    // For unit vectors, the cross-product length is `|sin(theta)|`. Compute the
    // length before comparison instead of squaring the caller's tolerance:
    // squaring a valid tiny epsilon could underflow to zero and accidentally
    // turn a near-parallel policy into an exact-parallel test.
    let cross = Vector3::cross(&left_unit, &right_unit);
    let sine_squared = Vector3::dot(&cross, &cross);
    if !is_finite_scalar(sine_squared) {
        return None;
    }
    let sine = sine_squared.tsqrt();
    if !is_finite_scalar(sine) {
        return None;
    }
    Some(sine <= angular_epsilon)
}

/// Tests whether two finite, nonzero vectors are perpendicular within an angular tolerance.
///
/// `angular_epsilon` is a dimensionless upper bound on `|cos(theta)|`, where
/// `theta` is the angle between the vectors. The inclusive comparison means a
/// vector exactly on the requested boundary is classified as nearly
/// perpendicular.
///
/// `None` is returned if either vector is zero or non-finite, or when the
/// tolerance is non-finite or outside the closed interval `[0, 1]`.
pub(crate) fn try_nearly_perpendicular3<T: FloatScalar>(
    left: &Vector3<T>,
    right: &Vector3<T>,
    angular_epsilon: T,
) -> Option<bool> {
    // Cosine magnitudes and their tolerance both live in [0, 1]. Validate the
    // caller-provided bound before performing any vector arithmetic.
    if !is_finite_scalar(angular_epsilon)
        || angular_epsilon < <T as Zero>::zero()
        || angular_epsilon > <T as One>::one()
    {
        return None;
    }

    // The stable unit representations remove both vectors' arbitrary positive
    // scales while retaining their directions and relative signs.
    let left_unit = try_normalized3(left)?.unit();
    let right_unit = try_normalized3(right)?.unit();

    // The absolute dot product of unit vectors is `|cos(theta)|`. Compare its
    // magnitude directly so a tiny valid tolerance is not lost by squaring it.
    let cosine = Vector3::dot(&left_unit, &right_unit);
    if !is_finite_scalar(cosine) {
        return None;
    }
    Some(cosine.tabs() <= angular_epsilon)
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

#[cfg(test)]
mod tests {
    use super::*;
    use crate::scalar::{FloatScalar, EPS_F32, EPS_F64};

    /// Asserts that two generic floating-point values differ by at most `tolerance`.
    fn assert_scalar_close<T>(actual: T, expected: T, tolerance: T)
    where
        T: FloatScalar + core::fmt::Debug,
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
        T: FloatScalar + core::fmt::Debug,
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
        assert!(is_finite_scalar(zero));
        assert!(is_finite_scalar(largest_finite));
        assert!(!is_finite_scalar(nan));
        assert!(!is_finite_scalar(infinity));
        assert!(!is_finite_scalar(-infinity));
        assert!(is_finite_vector3(&Vector3::new(
            smallest_positive,
            zero,
            largest_finite,
        )));
        assert!(!is_finite_vector3(&Vector3::new(nan, zero, zero)));
        assert!(!is_finite_vector3(&Vector3::new(zero, infinity, zero)));

        // A 3-4-5 vector checks the ordinary unit direction and the method that
        // divides a caller's value by the original, unscaled length.
        let ordinary = try_normalized3(&Vector3::new(three, four, zero))
            .expect("a finite nonzero vector must normalize");
        let ordinary_unit = ordinary.unit();
        assert_scalar_close(ordinary_unit.x, three / five, tolerance);
        assert_scalar_close(ordinary_unit.y, four / five, tolerance);
        assert_scalar_close(ordinary_unit.z, zero, tolerance);
        assert_scalar_close(ordinary_unit.length(), one, tolerance);
        assert_scalar_close(
            ordinary
                .divide_by_length(ten)
                .expect("10 / length(3, 4, 0) is finite"),
            two,
            tolerance,
        );
        assert_eq!(ordinary.divide_by_length(zero), Some(zero));

        // A balanced vector with components 3/4 has scale below one but length
        // above one. Dividing MAX by scale first overflows even though dividing
        // MAX by the complete vector length is representable; the alternate
        // ordering must preserve that finite result.
        let three_quarters = three / four;
        let balanced = try_normalized3(&Vector3::new(
            three_quarters,
            three_quarters,
            three_quarters,
        ))
        .expect("the balanced finite vector must normalize");
        let balanced_quotient = balanced
            .divide_by_length(largest_finite)
            .expect("a representable quotient must survive intermediate overflow");
        assert!(is_finite_scalar(balanced_quotient));
        assert!(balanced_quotient > zero);

        // Axis vectors at both exponent extremes would underflow or overflow in
        // a raw sum-of-squares implementation, but max scaling keeps them unit.
        for magnitude in [smallest_positive, largest_finite] {
            let normalized = try_normalized3(&Vector3::new(magnitude, zero, zero))
                .expect("a finite nonzero axis vector must normalize");
            assert_scalar_close(normalized.unit().x, one, tolerance);
            assert_scalar_close(normalized.unit().y, zero, tolerance);
            assert_scalar_close(normalized.unit().z, zero, tolerance);
            assert_scalar_close(
                normalized
                    .divide_by_length(magnitude)
                    .expect("a magnitude divided by itself is finite"),
                one,
                tolerance,
            );
        }

        // Invalid source vectors and invalid numerators are reported through
        // `None`; no NaN or infinity is allowed to escape the helper layer.
        assert!(try_normalized3(&Vector3::<T>::zero()).is_none());
        assert!(try_normalized3(&Vector3::new(nan, zero, zero)).is_none());
        assert!(try_normalized3(&Vector3::new(infinity, zero, zero)).is_none());
        assert!(ordinary.divide_by_length(nan).is_none());
        assert!(ordinary.divide_by_length(infinity).is_none());

        // Dividing one by the smallest subnormal exceeds the scalar range. The
        // operation is valid mathematically but deliberately returns `None`
        // because this API promises a finite result.
        let smallest = try_normalized3(&Vector3::new(smallest_positive, zero, zero))
            .expect("the smallest positive axis vector must normalize");
        assert!(smallest.divide_by_length(one).is_none());
    }

    /// Exercises dimensionless parallel/perpendicular classification for one scalar type.
    fn assert_orientation_contract<T>(
        smallest_positive: T,
        largest_finite: T,
        angular_epsilon: T,
        nan: T,
        infinity: T,
    ) where
        T: FloatScalar + core::fmt::Debug,
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
        assert_eq!(try_nearly_parallel3(&x, &x, angular_epsilon), Some(true));
        assert_eq!(
            try_nearly_parallel3(&x, &negative_x, angular_epsilon),
            Some(true)
        );
        assert_eq!(try_nearly_parallel3(&x, &y, angular_epsilon), Some(false));
        assert_eq!(
            try_nearly_parallel3(&x, &oblique, angular_epsilon),
            Some(false)
        );
        assert_eq!(
            try_nearly_perpendicular3(&x, &y, angular_epsilon),
            Some(true)
        );
        assert_eq!(
            try_nearly_perpendicular3(&x, &x, angular_epsilon),
            Some(false)
        );
        assert_eq!(
            try_nearly_perpendicular3(&x, &oblique, angular_epsilon),
            Some(false)
        );

        // Perturbations on opposite sides of the tolerance verify that the
        // helpers measure angle rather than raw dot/cross magnitude.
        let half_epsilon = angular_epsilon * T::half();
        let twice_epsilon = angular_epsilon * two;
        let near_parallel = Vector3::new(one, half_epsilon, zero);
        let not_near_parallel = Vector3::new(one, twice_epsilon, zero);
        assert_eq!(
            try_nearly_parallel3(&x, &near_parallel, angular_epsilon),
            Some(true)
        );
        assert_eq!(
            try_nearly_parallel3(&x, &not_near_parallel, angular_epsilon),
            Some(false)
        );
        let near_perpendicular = Vector3::new(half_epsilon, one, zero);
        let not_near_perpendicular = Vector3::new(twice_epsilon, one, zero);
        assert_eq!(
            try_nearly_perpendicular3(&x, &near_perpendicular, angular_epsilon),
            Some(true)
        );
        assert_eq!(
            try_nearly_perpendicular3(&x, &not_near_perpendicular, angular_epsilon),
            Some(false)
        );

        // The 3-4-5 directions place sine or cosine exactly at 3/5. These
        // assertions lock in the documented inclusive boundary convention.
        let boundary = three / five;
        assert_eq!(
            try_nearly_parallel3(&x, &Vector3::new(four, three, zero), boundary),
            Some(true)
        );
        assert_eq!(
            try_nearly_parallel3(&x, &Vector3::new(four, three, zero), boundary * T::half(),),
            Some(false)
        );
        assert_eq!(
            try_nearly_perpendicular3(&x, &Vector3::new(three, four, zero), boundary),
            Some(true)
        );
        assert_eq!(
            try_nearly_perpendicular3(&x, &Vector3::new(three, four, zero), boundary * T::half(),),
            Some(false)
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
                    try_nearly_parallel3(&scaled_x, &scaled_negative_x, angular_epsilon,),
                    Some(true)
                );
                assert_eq!(
                    try_nearly_perpendicular3(&scaled_x, &scaled_y, angular_epsilon),
                    Some(true)
                );
            }
        }

        // Invalid tolerances and indeterminate operands must return `None`
        // instead of being silently classified by unordered comparisons.
        for invalid_epsilon in [-angular_epsilon, one + angular_epsilon, nan, infinity] {
            assert_eq!(try_nearly_parallel3(&x, &y, invalid_epsilon), None);
            assert_eq!(try_nearly_perpendicular3(&x, &y, invalid_epsilon), None);
        }
        let zero_vector = Vector3::zero();
        let nan_vector = Vector3::new(nan, zero, zero);
        let infinite_vector = Vector3::new(zero, infinity, zero);
        for invalid_vector in [zero_vector, nan_vector, infinite_vector] {
            assert_eq!(
                try_nearly_parallel3(&x, &invalid_vector, angular_epsilon),
                None
            );
            assert_eq!(
                try_nearly_perpendicular3(&x, &invalid_vector, angular_epsilon),
                None
            );
        }
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
    fn test_vector_try_normalize() {
        let v = Vector3::<f32>::new(3.0, 4.0, 0.0);
        let nv = v.try_normalize(EPS_F32).expect("should normalize");
        assert!((nv.length() - 1.0).abs() < 0.001);

        let zero = Vector3::<f32>::zero();
        assert!(zero.try_normalize(EPS_F32).is_none());
        let zero_norm = zero.normalize_or_zero(EPS_F32);
        assert_eq!(zero_norm.x, 0.0);
        assert_eq!(zero_norm.y, 0.0);
        assert_eq!(zero_norm.z, 0.0);

        let len_sq = v.length_squared();
        let inv_len = 1.0f32 / len_sq.tsqrt();
        let nv_fast = v.normalize_with_inv_len(inv_len);
        assert!((nv_fast.length() - 1.0).abs() < 0.001);

        let nv_fast_try = v
            .try_normalize_with_inv_len(len_sq, inv_len, EPS_F32)
            .expect("should normalize");
        assert!((nv_fast_try.length() - 1.0).abs() < 0.001);
        assert!(zero.try_normalize_with_inv_len(0.0, 0.0, EPS_F32).is_none());
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
