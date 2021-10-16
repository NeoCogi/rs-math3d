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
use core::ops::{Add, Sub, Mul, Div, Rem, Neg};
use crate::scalar::*;

pub trait Vector<T: Scalar, Rhs = Self, Output = Self> :
    Add<Rhs, Output = Output>
    + Sub<Rhs, Output = Output>
    + Mul<Rhs, Output = Output>
    + Mul<T, Output = Output>
    + Div<Rhs, Output = Output>
    + Div<T, Output = Output>
    + Neg<Output = Output>
    + Clone + Copy
{
    fn zero() -> Self;

    fn add_vv(l: &Self, r: &Self)    -> Self;
    fn sub_vv(l: &Self, r: &Self)    -> Self;
    fn mul_vv(l: &Self, r: &Self)    -> Self;
    fn div_vv(l: &Self, r: &Self)    -> Self;
    fn mul_vs(l: &Self, r: T)        -> Self;
    fn div_vs(l: &Self, r: T)        -> Self;
    fn rem_vv(l: &Self, r: &Self)    -> Self;

    fn length(&self) -> T { Self::dot(&self, &self).tsqrt() }
    fn dot  (l: &Self, r: &Self) -> T;
    fn normalize(v: &Self) -> Self;
    fn distance (l: &Self, r: &Self) -> T;

    fn min(l: &Self, r: &Self) -> Self;
    fn max(l: &Self, r: &Self) -> Self;
}

macro_rules! implVecScalar {
    ($vecName:ident, $scalar:ident) => {
        impl Mul<$vecName<$scalar>> for $scalar {
            type Output = $vecName<$scalar>;

            fn mul(self, rhs: $vecName<$scalar>) -> Self::Output {
                $vecName::mul_vs(&rhs, self)
            }
        }
    }
}

macro_rules! implVector {
    ($vecName:ident, $($field:ident)*) => {
        #[repr(C)]
        #[derive(Copy, Clone, Debug, Default)]
        pub struct $vecName<T> { $(pub $field: T),* }

        impl<T: Scalar> $vecName<T> {
            pub fn new($($field:T),*) -> Self { Self { $($field: $field),* } }
            pub fn normalize(&self) -> Self { <$vecName<T> as Vector<T>>::normalize(self) }
        }

        impl<T: Scalar> Vector<T> for $vecName<T> {
            fn zero() -> Self { Self { $($field: T::zero()),* } }
            fn length(&self) -> T { Self::dot(&self, &self).tsqrt() }
            fn dot  (l: &Self, r: &Self) -> T { $(l.$field * r.$field +)* T::zero() }
            fn add_vv(l: &Self, r: &Self) -> Self { Self::new($(l.$field + r.$field),*) }
            fn sub_vv(l: &Self, r: &Self) -> Self { Self::new($(l.$field - r.$field),*) }
            fn mul_vv(l: &Self, r: &Self) -> Self { Self::new($(l.$field * r.$field),*) }
            fn div_vv(l: &Self, r: &Self) -> Self { Self::new($(l.$field / r.$field),*) }
            fn mul_vs(l: &Self, r: T) -> Self { Self::new($(l.$field * r),*) }
            fn div_vs(l: &Self, r: T) -> Self { Self::new($(l.$field / r),*) }
            fn rem_vv(l: &Self, r: &Self) -> Self { Self::new($(l.$field % r.$field),*) }
            fn normalize(v: &Self) -> Self { let len = v.length(); *v / len }
            fn distance (l: &Self, r: &Self) -> T { (*r - *l).length() }
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

pub trait CrossProduct {
    fn cross(l: &Self, r: &Self) -> Self;
}

implVector!(Vector2, x y);
implVector!(Vector3, x y z);
implVector!(Vector4, x y z w);

impl<T> CrossProduct for Vector3<T> where T : Scalar {
    fn cross(l: &Vector3<T>, r: &Vector3<T>) -> Vector3<T> {
        Vector3::new(l.y * r.z - l.z * r.y, l.z * r.x - l.x * r.z, l.x * r.y - l.y * r.x)
    }
}

pub trait Swizzle2<T: Scalar> {
    fn xx(&self) -> Vector2<T>;
    fn xy(&self) -> Vector2<T>;
    fn xz(&self) -> Vector2<T>;
    fn yx(&self) -> Vector2<T>;
    fn yy(&self) -> Vector2<T>;
    fn yz(&self) -> Vector2<T>;
    fn zx(&self) -> Vector2<T>;
    fn zy(&self) -> Vector2<T>;
    fn zz(&self) -> Vector2<T>;
}

impl<T: Scalar> Swizzle2<T> for Vector2<T> {
    fn xx(&self) -> Vector2<T> { Vector2::new(self.x, self.x) }
    fn xy(&self) -> Vector2<T> { Vector2::new(self.x, self.y) }
    fn xz(&self) -> Vector2<T> { Vector2::new(self.x, T::zero()) }
    fn yx(&self) -> Vector2<T> { Vector2::new(self.y, self.x) }
    fn yy(&self) -> Vector2<T> { Vector2::new(self.y, self.y) }
    fn yz(&self) -> Vector2<T> { Vector2::new(self.y, T::zero()) }
    fn zx(&self) -> Vector2<T> { Vector2::new(T::zero(), self.x) }
    fn zy(&self) -> Vector2<T> { Vector2::new(T::zero(), self.y) }
    fn zz(&self) -> Vector2<T> { Vector2::new(T::zero(), T::zero()) }
}

impl<T: Scalar> Swizzle2<T> for Vector3<T> {
    fn xx(&self) -> Vector2<T> { Vector2::new(self.x, self.x) }
    fn xy(&self) -> Vector2<T> { Vector2::new(self.x, self.y) }
    fn xz(&self) -> Vector2<T> { Vector2::new(self.x, self.z) }
    fn yx(&self) -> Vector2<T> { Vector2::new(self.y, self.x) }
    fn yy(&self) -> Vector2<T> { Vector2::new(self.y, self.y) }
    fn yz(&self) -> Vector2<T> { Vector2::new(self.y, self.z) }
    fn zx(&self) -> Vector2<T> { Vector2::new(self.z, self.x) }
    fn zy(&self) -> Vector2<T> { Vector2::new(self.z, self.y) }
    fn zz(&self) -> Vector2<T> { Vector2::new(self.z, self.z) }
}

impl<T: Scalar> Swizzle2<T> for Vector4<T> {
    fn xx(&self) -> Vector2<T> { Vector2::new(self.x, self.x) }
    fn xy(&self) -> Vector2<T> { Vector2::new(self.x, self.y) }
    fn xz(&self) -> Vector2<T> { Vector2::new(self.x, self.z) }
    fn yx(&self) -> Vector2<T> { Vector2::new(self.y, self.x) }
    fn yy(&self) -> Vector2<T> { Vector2::new(self.y, self.y) }
    fn yz(&self) -> Vector2<T> { Vector2::new(self.y, self.z) }
    fn zx(&self) -> Vector2<T> { Vector2::new(self.z, self.x) }
    fn zy(&self) -> Vector2<T> { Vector2::new(self.z, self.y) }
    fn zz(&self) -> Vector2<T> { Vector2::new(self.z, self.z) }
}

pub trait Swizzle3<T: Scalar> {
    fn xxx(&self) -> Vector3<T>;
    fn xxy(&self) -> Vector3<T>;
    fn xxz(&self) -> Vector3<T>;
    fn xyx(&self) -> Vector3<T>;
    fn xyy(&self) -> Vector3<T>;
    fn xyz(&self) -> Vector3<T>;
    fn xzx(&self) -> Vector3<T>;
    fn xzy(&self) -> Vector3<T>;
    fn xzz(&self) -> Vector3<T>;

    fn yxx(&self) -> Vector3<T>;
    fn yxy(&self) -> Vector3<T>;
    fn yxz(&self) -> Vector3<T>;
    fn yyx(&self) -> Vector3<T>;
    fn yyy(&self) -> Vector3<T>;
    fn yyz(&self) -> Vector3<T>;
    fn yzx(&self) -> Vector3<T>;
    fn yzy(&self) -> Vector3<T>;
    fn yzz(&self) -> Vector3<T>;

    fn zxx(&self) -> Vector3<T>;
    fn zxy(&self) -> Vector3<T>;
    fn zxz(&self) -> Vector3<T>;
    fn zyx(&self) -> Vector3<T>;
    fn zyy(&self) -> Vector3<T>;
    fn zyz(&self) -> Vector3<T>;
    fn zzx(&self) -> Vector3<T>;
    fn zzy(&self) -> Vector3<T>;
    fn zzz(&self) -> Vector3<T>;
}

impl<T: Scalar> Swizzle3<T> for Vector3<T> {
    fn xxx(&self) -> Vector3<T> { Vector3::new(self.x, self.x, self.x) }
    fn xxy(&self) -> Vector3<T> { Vector3::new(self.x, self.x, self.y) }
    fn xxz(&self) -> Vector3<T> { Vector3::new(self.x, self.x, self.z) }
    fn xyx(&self) -> Vector3<T> { Vector3::new(self.x, self.y, self.x) }
    fn xyy(&self) -> Vector3<T> { Vector3::new(self.x, self.y, self.y) }
    fn xyz(&self) -> Vector3<T> { Vector3::new(self.x, self.y, self.z) }
    fn xzx(&self) -> Vector3<T> { Vector3::new(self.x, self.z, self.x) }
    fn xzy(&self) -> Vector3<T> { Vector3::new(self.x, self.z, self.y) }
    fn xzz(&self) -> Vector3<T> { Vector3::new(self.x, self.z, self.z) }

    fn yxx(&self) -> Vector3<T> { Vector3::new(self.y, self.x, self.x) }
    fn yxy(&self) -> Vector3<T> { Vector3::new(self.y, self.x, self.y) }
    fn yxz(&self) -> Vector3<T> { Vector3::new(self.y, self.x, self.z) }
    fn yyx(&self) -> Vector3<T> { Vector3::new(self.y, self.y, self.x) }
    fn yyy(&self) -> Vector3<T> { Vector3::new(self.y, self.y, self.y) }
    fn yyz(&self) -> Vector3<T> { Vector3::new(self.y, self.y, self.z) }
    fn yzx(&self) -> Vector3<T> { Vector3::new(self.y, self.z, self.x) }
    fn yzy(&self) -> Vector3<T> { Vector3::new(self.y, self.z, self.y) }
    fn yzz(&self) -> Vector3<T> { Vector3::new(self.y, self.z, self.z) }

    fn zxx(&self) -> Vector3<T> { Vector3::new(self.z, self.x, self.x) }
    fn zxy(&self) -> Vector3<T> { Vector3::new(self.z, self.x, self.y) }
    fn zxz(&self) -> Vector3<T> { Vector3::new(self.z, self.x, self.z) }
    fn zyx(&self) -> Vector3<T> { Vector3::new(self.z, self.y, self.x) }
    fn zyy(&self) -> Vector3<T> { Vector3::new(self.z, self.y, self.y) }
    fn zyz(&self) -> Vector3<T> { Vector3::new(self.z, self.y, self.z) }
    fn zzx(&self) -> Vector3<T> { Vector3::new(self.z, self.z, self.x) }
    fn zzy(&self) -> Vector3<T> { Vector3::new(self.z, self.z, self.y) }
    fn zzz(&self) -> Vector3<T> { Vector3::new(self.z, self.z, self.z) }
}

impl<T: Scalar> Swizzle3<T> for Vector4<T> {
    fn xxx(&self) -> Vector3<T> { Vector3::new(self.x, self.x, self.x) }
    fn xxy(&self) -> Vector3<T> { Vector3::new(self.x, self.x, self.y) }
    fn xxz(&self) -> Vector3<T> { Vector3::new(self.x, self.x, self.z) }
    fn xyx(&self) -> Vector3<T> { Vector3::new(self.x, self.y, self.x) }
    fn xyy(&self) -> Vector3<T> { Vector3::new(self.x, self.y, self.y) }
    fn xyz(&self) -> Vector3<T> { Vector3::new(self.x, self.y, self.z) }
    fn xzx(&self) -> Vector3<T> { Vector3::new(self.x, self.z, self.x) }
    fn xzy(&self) -> Vector3<T> { Vector3::new(self.x, self.z, self.y) }
    fn xzz(&self) -> Vector3<T> { Vector3::new(self.x, self.z, self.z) }

    fn yxx(&self) -> Vector3<T> { Vector3::new(self.y, self.x, self.x) }
    fn yxy(&self) -> Vector3<T> { Vector3::new(self.y, self.x, self.y) }
    fn yxz(&self) -> Vector3<T> { Vector3::new(self.y, self.x, self.z) }
    fn yyx(&self) -> Vector3<T> { Vector3::new(self.y, self.y, self.x) }
    fn yyy(&self) -> Vector3<T> { Vector3::new(self.y, self.y, self.y) }
    fn yyz(&self) -> Vector3<T> { Vector3::new(self.y, self.y, self.z) }
    fn yzx(&self) -> Vector3<T> { Vector3::new(self.y, self.z, self.x) }
    fn yzy(&self) -> Vector3<T> { Vector3::new(self.y, self.z, self.y) }
    fn yzz(&self) -> Vector3<T> { Vector3::new(self.y, self.z, self.z) }

    fn zxx(&self) -> Vector3<T> { Vector3::new(self.z, self.x, self.x) }
    fn zxy(&self) -> Vector3<T> { Vector3::new(self.z, self.x, self.y) }
    fn zxz(&self) -> Vector3<T> { Vector3::new(self.z, self.x, self.z) }
    fn zyx(&self) -> Vector3<T> { Vector3::new(self.z, self.y, self.x) }
    fn zyy(&self) -> Vector3<T> { Vector3::new(self.z, self.y, self.y) }
    fn zyz(&self) -> Vector3<T> { Vector3::new(self.z, self.y, self.z) }
    fn zzx(&self) -> Vector3<T> { Vector3::new(self.z, self.z, self.x) }
    fn zzy(&self) -> Vector3<T> { Vector3::new(self.z, self.z, self.y) }
    fn zzz(&self) -> Vector3<T> { Vector3::new(self.z, self.z, self.z) }
}


pub fn length   <T: Scalar, V: Vector<T>>   (v: &V) -> T { V::dot(v, v).tsqrt() }
pub fn dot      <T: Scalar, V: Vector<T>>   (l: &V, r: &V) -> T { V::dot(l, r)  }
pub fn normalize<T: Scalar, V: Vector<T>>   (v: &V) -> V { let len = v.length(); *v / len }
pub fn distance <T: Scalar, V: Vector<T>>   (l: &V, r: &V) -> T { (*r - *l).length() }
pub fn min      <T: Scalar, V: Vector<T>>   (l: &V, r: &V) -> V { V::min(l, r) }
pub fn max      <T: Scalar, V: Vector<T>>   (l: &V, r: &V) -> V { V::max(l, r) }

#[cfg(test)]
mod tests {
    use super::*;
    #[test]
    pub fn test() {
        let f1 = Vector2 { x: 1.0, y: 2.0 };
        let f2 = Vector2 { x: 3.0, y: 4.0 };
        let out = f1 + f2;
        assert_eq!(out.x, 4.0);
        assert_eq!(out.y, 6.0);

        let f22 : Vector2<f32> = 2.0 * f2;
        let f23 : Vector2<f32> = f2 * 2.0;

        assert_eq!(f22.x, 6.0);
        assert_eq!(f22.y, 8.0);
        assert_eq!(f23.x, f22.x);
        assert_eq!(f23.y, f22.y);
    }
}