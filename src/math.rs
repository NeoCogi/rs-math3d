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

//! Compile-time selection and isolation of floating-point math backends.
//!
//! The rest of the crate calls the private [`Math`] facade and therefore does
//! not need backend-specific conditional compilation. Backend precedence is
//! deterministic: `std`, then `libm`, then the Unix C math library. A build
//! that disables default features must select `libm` or `system-libm`
//! explicitly; silently importing undeclared C symbols is intentionally not a
//! supported fallback.

// A featureless build previously selected the C backend implicitly. That
// compiled as an rlib but could fail only when a downstream no_std executable
// was linked. Rejecting the configuration here makes the missing backend an
// immediate, actionable Cargo error instead.
#[cfg(all(
    not(feature = "std"),
    not(feature = "libm"),
    not(feature = "system-libm")
))]
compile_error!(
    "no floating-point math backend selected; enable the default `std` feature, or disable \
     default features and explicitly enable `libm` (std-free) or `system-libm` (Unix only)"
);

// The raw C backend is deliberately conservative: Unix targets conventionally
// provide these C99 functions through libm, while other target families use
// different runtime libraries or may have no C runtime at all. `std` and
// `libm` retain precedence, so merely unifying `system-libm` alongside either
// portable backend does not make an otherwise valid build target-dependent.
#[cfg(all(
    not(feature = "std"),
    not(feature = "libm"),
    feature = "system-libm",
    not(unix)
))]
compile_error!(
    "the selected `system-libm` backend is supported only on Unix targets; use the `libm` \
     feature for portable std-free builds"
);

#[cfg(feature = "std")]
mod backend {
    //! Standard-library implementation of every float operation used by the
    //! crate. This module is selected whenever the `std` feature is enabled;
    //! the parent crate remains source-level `#![no_std]` while explicitly
    //! importing and linking Rust's standard library.

    /// Computes the square root of an `f32` with the standard library.
    pub(crate) fn sqrt_f32(x: f32) -> f32 {
        // Calling the inherent method preserves the platform behavior selected
        // by Rust's standard library, including its IEEE exceptional values.
        x.sqrt()
    }

    /// Computes the sine of an `f32` angle, expressed in radians.
    pub(crate) fn sin_f32(x: f32) -> f32 {
        // Keep transcendental dispatch inside this module so scalar code stays
        // identical for all compile-time backends.
        x.sin()
    }

    /// Computes the cosine of an `f32` angle, expressed in radians.
    pub(crate) fn cos_f32(x: f32) -> f32 {
        // Delegate directly to the standard library's target implementation.
        x.cos()
    }

    /// Computes the tangent of an `f32` angle, expressed in radians.
    pub(crate) fn tan_f32(x: f32) -> f32 {
        // Delegate directly; poles and non-finite inputs retain std semantics.
        x.tan()
    }

    /// Computes the arc cosine of an `f32`, returning radians.
    pub(crate) fn acos_f32(x: f32) -> f32 {
        // The standard library defines the domain and exceptional-value
        // behavior used by the public FloatScalar contract.
        x.acos()
    }

    /// Computes the square root of an `f64` with the standard library.
    pub(crate) fn sqrt_f64(x: f64) -> f64 {
        // Use the matching f64 operation rather than narrowing through f32.
        x.sqrt()
    }

    /// Computes the sine of an `f64` angle, expressed in radians.
    pub(crate) fn sin_f64(x: f64) -> f64 {
        // Preserve full f64 precision through the selected std implementation.
        x.sin()
    }

    /// Computes the cosine of an `f64` angle, expressed in radians.
    pub(crate) fn cos_f64(x: f64) -> f64 {
        // Preserve full f64 precision through the selected std implementation.
        x.cos()
    }

    /// Computes the tangent of an `f64` angle, expressed in radians.
    pub(crate) fn tan_f64(x: f64) -> f64 {
        // Delegate directly; poles and non-finite inputs retain std semantics.
        x.tan()
    }

    /// Computes the arc cosine of an `f64`, returning radians.
    pub(crate) fn acos_f64(x: f64) -> f64 {
        // Use std's domain handling without a lossy intermediate conversion.
        x.acos()
    }
}

#[cfg(all(feature = "libm", not(feature = "std")))]
mod backend {
    //! Pure-Rust, std-free implementation backed by the optional `libm` crate.
    //! This is the portable backend for targets without a standard library.

    /// Computes the square root of an `f32` with pure-Rust libm.
    pub(crate) fn sqrt_f32(x: f32) -> f32 {
        // The suffixed entry point keeps the complete calculation in f32.
        libm::sqrtf(x)
    }

    /// Computes the sine of an `f32` angle, expressed in radians.
    pub(crate) fn sin_f32(x: f32) -> f32 {
        // Route through libm rather than declaring a target C math symbol.
        libm::sinf(x)
    }

    /// Computes the cosine of an `f32` angle, expressed in radians.
    pub(crate) fn cos_f32(x: f32) -> f32 {
        // Route through libm rather than declaring a target C math symbol.
        libm::cosf(x)
    }

    /// Computes the tangent of an `f32` angle, expressed in radians.
    pub(crate) fn tan_f32(x: f32) -> f32 {
        // Preserve libm's no_std implementation and exceptional-value behavior.
        libm::tanf(x)
    }

    /// Computes the arc cosine of an `f32`, returning radians.
    pub(crate) fn acos_f32(x: f32) -> f32 {
        // Use libm's f32-specific routine without widening the calculation.
        libm::acosf(x)
    }

    /// Computes the square root of an `f64` with pure-Rust libm.
    pub(crate) fn sqrt_f64(x: f64) -> f64 {
        // The unsuffixed libm API operates directly on f64 values.
        libm::sqrt(x)
    }

    /// Computes the sine of an `f64` angle, expressed in radians.
    pub(crate) fn sin_f64(x: f64) -> f64 {
        // Keep the std-free path independent of the system C math library.
        libm::sin(x)
    }

    /// Computes the cosine of an `f64` angle, expressed in radians.
    pub(crate) fn cos_f64(x: f64) -> f64 {
        // Keep the std-free path independent of the system C math library.
        libm::cos(x)
    }

    /// Computes the tangent of an `f64` angle, expressed in radians.
    pub(crate) fn tan_f64(x: f64) -> f64 {
        // Preserve libm's no_std implementation and exceptional-value behavior.
        libm::tan(x)
    }

    /// Computes the arc cosine of an `f64`, returning radians.
    pub(crate) fn acos_f64(x: f64) -> f64 {
        // Use the f64 routine directly so the backend does not lose precision.
        libm::acos(x)
    }
}

// Compile the foreign-function backend only for an explicit, effective
// `system-libm` selection on its supported target family. Keeping every part
// of the module behind this gate ensures unsupported targets never declare or
// attempt to link C math symbols.
#[cfg(all(
    feature = "system-libm",
    not(feature = "std"),
    not(feature = "libm"),
    unix
))]
mod backend {
    //! Unix system-libm implementation. The foreign declarations are kept in
    //! this one private module so unsafe calls and native-link requirements are
    //! easy to audit.

    // Unix linkers do not generally search libm unless it is named explicitly.
    // Attaching the native library to this extern block propagates `-lm` to
    // downstream binaries instead of requiring application-specific link flags.
    #[link(name = "m")]
    extern "C" {
        /// C99 single-precision square-root entry point.
        fn sqrtf(x: f32) -> f32;
        /// C99 single-precision sine entry point; the argument is in radians.
        fn sinf(x: f32) -> f32;
        /// C99 single-precision cosine entry point; the argument is in radians.
        fn cosf(x: f32) -> f32;
        /// C99 single-precision tangent entry point; the argument is in radians.
        fn tanf(x: f32) -> f32;
        /// C99 single-precision arc-cosine entry point; the result is in radians.
        fn acosf(x: f32) -> f32;

        /// C99 double-precision square-root entry point.
        fn sqrt(x: f64) -> f64;
        /// C99 double-precision sine entry point; the argument is in radians.
        fn sin(x: f64) -> f64;
        /// C99 double-precision cosine entry point; the argument is in radians.
        fn cos(x: f64) -> f64;
        /// C99 double-precision tangent entry point; the argument is in radians.
        fn tan(x: f64) -> f64;
        /// C99 double-precision arc-cosine entry point; the result is in radians.
        fn acos(x: f64) -> f64;
    }

    /// Computes the square root of an `f32` through Unix libm.
    pub(crate) fn sqrt_f32(x: f32) -> f32 {
        // SAFETY: Unix libm defines `sqrtf` with exactly this C signature. The
        // function accepts every f32 bit pattern and returns a value without
        // retaining pointers or accessing Rust-managed memory.
        unsafe { sqrtf(x) }
    }

    /// Computes the sine of an `f32` angle through Unix libm.
    pub(crate) fn sin_f32(x: f32) -> f32 {
        // SAFETY: The linked C99 `sinf(float) -> float` ABI matches this
        // declaration, and passing an f32 by value has no memory-safety
        // preconditions.
        unsafe { sinf(x) }
    }

    /// Computes the cosine of an `f32` angle through Unix libm.
    pub(crate) fn cos_f32(x: f32) -> f32 {
        // SAFETY: The linked C99 `cosf(float) -> float` ABI matches this
        // declaration; all finite and non-finite inputs are valid C values.
        unsafe { cosf(x) }
    }

    /// Computes the tangent of an `f32` angle through Unix libm.
    pub(crate) fn tan_f32(x: f32) -> f32 {
        // SAFETY: The linked C99 `tanf(float) -> float` ABI matches this
        // declaration and the call does not share memory with C.
        unsafe { tanf(x) }
    }

    /// Computes the arc cosine of an `f32` through Unix libm.
    pub(crate) fn acos_f32(x: f32) -> f32 {
        // SAFETY: The linked C99 `acosf(float) -> float` ABI matches this
        // declaration. Out-of-domain values affect only the numeric result.
        unsafe { acosf(x) }
    }

    /// Computes the square root of an `f64` through Unix libm.
    pub(crate) fn sqrt_f64(x: f64) -> f64 {
        // SAFETY: Unix libm defines `sqrt` with this double-precision C ABI;
        // the by-value call has no pointer or lifetime requirements.
        unsafe { sqrt(x) }
    }

    /// Computes the sine of an `f64` angle through Unix libm.
    pub(crate) fn sin_f64(x: f64) -> f64 {
        // SAFETY: The linked C99 `sin(double) -> double` ABI matches this
        // declaration and accepts every f64 representation.
        unsafe { sin(x) }
    }

    /// Computes the cosine of an `f64` angle through Unix libm.
    pub(crate) fn cos_f64(x: f64) -> f64 {
        // SAFETY: The linked C99 `cos(double) -> double` ABI matches this
        // declaration and performs no Rust-visible memory access.
        unsafe { cos(x) }
    }

    /// Computes the tangent of an `f64` angle through Unix libm.
    pub(crate) fn tan_f64(x: f64) -> f64 {
        // SAFETY: The linked C99 `tan(double) -> double` ABI matches this
        // declaration. Poles change the numeric result, not call safety.
        unsafe { tan(x) }
    }

    /// Computes the arc cosine of an `f64` through Unix libm.
    pub(crate) fn acos_f64(x: f64) -> f64 {
        // SAFETY: The linked C99 `acos(double) -> double` ABI matches this
        // declaration. Domain errors are represented numerically by libm.
        unsafe { acos(x) }
    }
}

/// Backend-neutral facade used by [`crate::scalar::FloatScalar`].
///
/// This zero-sized private type deliberately exposes only the operations the
/// crate currently needs. Keeping that surface small prevents backend details
/// and foreign functions from leaking into the public API.
pub(crate) struct Math;

impl Math {
    /// Computes an `f32` square root with the compile-time-selected backend.
    pub(crate) fn sqrt_f32(x: f32) -> f32 {
        // Forwarding here centralizes backend dispatch and leaves scalar trait
        // implementations free of repeated cfg expressions.
        backend::sqrt_f32(x)
    }

    /// Computes the sine of an `f32` radian angle with the selected backend.
    pub(crate) fn sin_f32(x: f32) -> f32 {
        // The active backend is resolved at compile time, so this abstraction
        // adds no runtime feature check.
        backend::sin_f32(x)
    }

    /// Computes the cosine of an `f32` radian angle with the selected backend.
    pub(crate) fn cos_f32(x: f32) -> f32 {
        // Keep public scalar behavior independent of how cosine is supplied.
        backend::cos_f32(x)
    }

    /// Computes the tangent of an `f32` radian angle with the selected backend.
    pub(crate) fn tan_f32(x: f32) -> f32 {
        // Forward directly; backend-specific exceptional values are preserved.
        backend::tan_f32(x)
    }

    /// Computes the arc cosine of an `f32` with the selected backend.
    pub(crate) fn acos_f32(x: f32) -> f32 {
        // Forward directly without widening or otherwise changing the input.
        backend::acos_f32(x)
    }

    /// Computes an `f64` square root with the compile-time-selected backend.
    pub(crate) fn sqrt_f64(x: f64) -> f64 {
        // Dispatch to the f64 backend entry point to retain input precision.
        backend::sqrt_f64(x)
    }

    /// Computes the sine of an `f64` radian angle with the selected backend.
    pub(crate) fn sin_f64(x: f64) -> f64 {
        // The backend choice is compile-time-only and introduces no branch.
        backend::sin_f64(x)
    }

    /// Computes the cosine of an `f64` radian angle with the selected backend.
    pub(crate) fn cos_f64(x: f64) -> f64 {
        // Keep the FloatScalar implementation unaware of backend modules.
        backend::cos_f64(x)
    }

    /// Computes the tangent of an `f64` radian angle with the selected backend.
    pub(crate) fn tan_f64(x: f64) -> f64 {
        // Forward directly; numeric edge behavior belongs to the backend.
        backend::tan_f64(x)
    }

    /// Computes the arc cosine of an `f64` with the selected backend.
    pub(crate) fn acos_f64(x: f64) -> f64 {
        // Forward directly without narrowing or altering domain handling.
        backend::acos_f64(x)
    }
}
