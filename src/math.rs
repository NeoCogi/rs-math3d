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

#[cfg(any(
    feature = "std",
    all(
        test,
        not(feature = "std"),
        not(feature = "libm"),
        not(feature = "system-libm")
    )
))]
mod backend {
    pub(crate) fn sqrt_f32(x: f32) -> f32 {
        x.sqrt()
    }

    pub(crate) fn sin_f32(x: f32) -> f32 {
        x.sin()
    }

    pub(crate) fn cos_f32(x: f32) -> f32 {
        x.cos()
    }

    pub(crate) fn tan_f32(x: f32) -> f32 {
        x.tan()
    }

    pub(crate) fn acos_f32(x: f32) -> f32 {
        x.acos()
    }

    pub(crate) fn sqrt_f64(x: f64) -> f64 {
        x.sqrt()
    }

    pub(crate) fn sin_f64(x: f64) -> f64 {
        x.sin()
    }

    pub(crate) fn cos_f64(x: f64) -> f64 {
        x.cos()
    }

    pub(crate) fn tan_f64(x: f64) -> f64 {
        x.tan()
    }

    pub(crate) fn acos_f64(x: f64) -> f64 {
        x.acos()
    }
}

#[cfg(all(feature = "libm", not(feature = "std")))]
mod backend {
    pub(crate) fn sqrt_f32(x: f32) -> f32 {
        libm::sqrtf(x)
    }

    pub(crate) fn sin_f32(x: f32) -> f32 {
        libm::sinf(x)
    }

    pub(crate) fn cos_f32(x: f32) -> f32 {
        libm::cosf(x)
    }

    pub(crate) fn tan_f32(x: f32) -> f32 {
        libm::tanf(x)
    }

    pub(crate) fn acos_f32(x: f32) -> f32 {
        libm::acosf(x)
    }

    pub(crate) fn sqrt_f64(x: f64) -> f64 {
        libm::sqrt(x)
    }

    pub(crate) fn sin_f64(x: f64) -> f64 {
        libm::sin(x)
    }

    pub(crate) fn cos_f64(x: f64) -> f64 {
        libm::cos(x)
    }

    pub(crate) fn tan_f64(x: f64) -> f64 {
        libm::tan(x)
    }

    pub(crate) fn acos_f64(x: f64) -> f64 {
        libm::acos(x)
    }
}

#[cfg(any(
    all(feature = "system-libm", not(feature = "std"), not(feature = "libm")),
    all(
        not(test),
        not(feature = "std"),
        not(feature = "libm"),
        not(feature = "system-libm")
    )
))]
mod backend {
    extern "C" {
        fn sqrtf(x: f32) -> f32;
        fn sinf(x: f32) -> f32;
        fn cosf(x: f32) -> f32;
        fn tanf(x: f32) -> f32;
        fn acosf(x: f32) -> f32;

        fn sqrt(x: f64) -> f64;
        fn sin(x: f64) -> f64;
        fn cos(x: f64) -> f64;
        fn tan(x: f64) -> f64;
        fn acos(x: f64) -> f64;
    }

    pub(crate) fn sqrt_f32(x: f32) -> f32 {
        unsafe { sqrtf(x) }
    }

    pub(crate) fn sin_f32(x: f32) -> f32 {
        unsafe { sinf(x) }
    }

    pub(crate) fn cos_f32(x: f32) -> f32 {
        unsafe { cosf(x) }
    }

    pub(crate) fn tan_f32(x: f32) -> f32 {
        unsafe { tanf(x) }
    }

    pub(crate) fn acos_f32(x: f32) -> f32 {
        unsafe { acosf(x) }
    }

    pub(crate) fn sqrt_f64(x: f64) -> f64 {
        unsafe { sqrt(x) }
    }

    pub(crate) fn sin_f64(x: f64) -> f64 {
        unsafe { sin(x) }
    }

    pub(crate) fn cos_f64(x: f64) -> f64 {
        unsafe { cos(x) }
    }

    pub(crate) fn tan_f64(x: f64) -> f64 {
        unsafe { tan(x) }
    }

    pub(crate) fn acos_f64(x: f64) -> f64 {
        unsafe { acos(x) }
    }
}

pub(crate) struct Math;

impl Math {
    pub(crate) fn sqrt_f32(x: f32) -> f32 {
        backend::sqrt_f32(x)
    }

    pub(crate) fn sin_f32(x: f32) -> f32 {
        backend::sin_f32(x)
    }

    pub(crate) fn cos_f32(x: f32) -> f32 {
        backend::cos_f32(x)
    }

    pub(crate) fn tan_f32(x: f32) -> f32 {
        backend::tan_f32(x)
    }

    pub(crate) fn acos_f32(x: f32) -> f32 {
        backend::acos_f32(x)
    }

    pub(crate) fn sqrt_f64(x: f64) -> f64 {
        backend::sqrt_f64(x)
    }

    pub(crate) fn sin_f64(x: f64) -> f64 {
        backend::sin_f64(x)
    }

    pub(crate) fn cos_f64(x: f64) -> f64 {
        backend::cos_f64(x)
    }

    pub(crate) fn tan_f64(x: f64) -> f64 {
        backend::tan_f64(x)
    }

    pub(crate) fn acos_f64(x: f64) -> f64 {
        backend::acos_f64(x)
    }
}
