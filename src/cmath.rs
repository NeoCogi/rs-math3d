#[cfg(any(unix, target_arch="wasm32"))]
pub(crate) mod implementation {
    #[link(name = "m")]
    extern "C" {
        pub fn sqrt(n: f64) -> f64;
        pub fn sqrtf(n: f32) -> f32;
        pub fn acos(n: f64) -> f64;
        pub fn acosf(n: f32) -> f32;
        pub fn asin(n: f64) -> f64;
        pub fn asinf(n: f32) -> f32;
        pub fn tan(n: f64) -> f64;
        pub fn tanf(n: f32) -> f32;
        pub fn cos(n: f64) -> f64;
        pub fn cosf(n: f32) -> f32;
        pub fn sin(n: f64) -> f64;
        pub fn sinf(n: f32) -> f32;
        pub fn abs(n: f64) -> f64;
        pub fn absf(n: f32) -> f32;
    }
}

#[cfg(windows)]
pub(crate) mod implementation {
    extern "C" {
        pub fn sqrt(n: f64) -> f64;
        pub fn sqrtf(n: f32) -> f32;
        pub fn acos(n: f64) -> f64;
        pub fn acosf(n: f32) -> f32;
        pub fn asin(n: f64) -> f64;
        pub fn asinf(n: f32) -> f32;
        pub fn tan(n: f64) -> f64;
        pub fn tanf(n: f32) -> f32;
        pub fn cos(n: f64) -> f64;
        pub fn cosf(n: f32) -> f32;
        pub fn sin(n: f64) -> f64;
        pub fn sinf(n: f32) -> f32;
        pub fn abs(n: f64) -> f64;
        pub fn absf(n: f32) -> f32;
    }
}

pub(crate) trait CScalar {
    fn sqrt(self) -> Self;
    fn acos(self) -> Self;
    fn asin(self) -> Self;
    fn tan (self) -> Self;
    fn cos (self) -> Self;
    fn sin (self) -> Self;
    fn abs (self) -> Self;
}

impl CScalar for f32 {
    fn sqrt(self) -> Self { unsafe { implementation::sqrtf(self) } }
    fn acos(self) -> Self { unsafe { implementation::acosf(self) } }
    fn asin(self) -> Self { unsafe { implementation::asinf(self) } }
    fn tan (self) -> Self { unsafe { implementation::tanf (self) } }
    fn cos (self) -> Self { unsafe { implementation::cosf (self) } }
    fn sin (self) -> Self { unsafe { implementation::sinf (self) } }
    fn abs (self) -> Self { unsafe { implementation::absf (self) } }
}

impl CScalar for f64 {
    fn sqrt(self) -> Self { unsafe { implementation::sqrt(self) } }
    fn acos(self) -> Self { unsafe { implementation::acos(self) } }
    fn asin(self) -> Self { unsafe { implementation::asin(self) } }
    fn tan (self) -> Self { unsafe { implementation::tan (self) } }
    fn cos (self) -> Self { unsafe { implementation::cos (self) } }
    fn sin (self) -> Self { unsafe { implementation::sin (self) } }
    fn abs (self) -> Self { unsafe { implementation::abs (self) } }
}