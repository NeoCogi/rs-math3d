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
use crate::scalar::*;
use num_traits::{Zero, One};
use core::ops::{Add, Div, Mul, Neg, Rem, Sub};

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
    fn zero() -> Self;

    fn add_vv(l: &Self, r: &Self) -> Self;
    fn sub_vv(l: &Self, r: &Self) -> Self;
    fn mul_vv(l: &Self, r: &Self) -> Self;
    fn div_vv(l: &Self, r: &Self) -> Self;
    fn mul_vs(l: &Self, r: T) -> Self;
    fn div_vs(l: &Self, r: T) -> Self;
    fn rem_vv(l: &Self, r: &Self) -> Self;

    fn dot(l: &Self, r: &Self) -> T;

    fn min(l: &Self, r: &Self) -> Self;
    fn max(l: &Self, r: &Self) -> Self;
}

pub trait FloatVector<T: FloatScalar>: Vector<T> {
    fn length(&self) -> T;
    fn normalize(&self) -> Self;
    fn distance(l: &Self, r: &Self) -> T;
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

macro_rules! implVector {
    ($vecName:ident, $($field:ident)*) => {
        #[repr(C)]
        #[derive(Copy, Clone, Debug, Default)]
        pub struct $vecName<T> { $(pub $field: T),* }

        impl<T: Scalar> $vecName<T> {
            pub fn new($($field:T),*) -> Self { Self { $($field: $field),* } }
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

pub trait CrossProduct {
    fn cross(l: &Self, r: &Self) -> Self;
}

implVector!(Vector2, x y);
implVector!(Vector3, x y z);
implVector!(Vector4, x y z w);

implFloatVector!(Vector2);
implFloatVector!(Vector3);
implFloatVector!(Vector4);

impl<T> CrossProduct for Vector3<T>
where
    T: Scalar,
{
    fn cross(l: &Vector3<T>, r: &Vector3<T>) -> Vector3<T> {
        Vector3::new(
            l.y * r.z - l.z * r.y,
            l.z * r.x - l.x * r.z,
            l.x * r.y - l.y * r.x,
        )
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
    fn xx(&self) -> Vector2<T> {
        Vector2::new(self.x, self.x)
    }
    fn xy(&self) -> Vector2<T> {
        Vector2::new(self.x, self.y)
    }
    fn xz(&self) -> Vector2<T> {
        Vector2::new(self.x, <T as Zero>::zero())
    }
    fn yx(&self) -> Vector2<T> {
        Vector2::new(self.y, self.x)
    }
    fn yy(&self) -> Vector2<T> {
        Vector2::new(self.y, self.y)
    }
    fn yz(&self) -> Vector2<T> {
        Vector2::new(self.y, <T as Zero>::zero())
    }
    fn zx(&self) -> Vector2<T> {
        Vector2::new(<T as Zero>::zero(), self.x)
    }
    fn zy(&self) -> Vector2<T> {
        Vector2::new(<T as Zero>::zero(), self.y)
    }
    fn zz(&self) -> Vector2<T> {
        Vector2::new(<T as Zero>::zero(), <T as Zero>::zero())
    }
}

impl<T: Scalar> Swizzle2<T> for Vector3<T> {
    fn xx(&self) -> Vector2<T> {
        Vector2::new(self.x, self.x)
    }
    fn xy(&self) -> Vector2<T> {
        Vector2::new(self.x, self.y)
    }
    fn xz(&self) -> Vector2<T> {
        Vector2::new(self.x, self.z)
    }
    fn yx(&self) -> Vector2<T> {
        Vector2::new(self.y, self.x)
    }
    fn yy(&self) -> Vector2<T> {
        Vector2::new(self.y, self.y)
    }
    fn yz(&self) -> Vector2<T> {
        Vector2::new(self.y, self.z)
    }
    fn zx(&self) -> Vector2<T> {
        Vector2::new(self.z, self.x)
    }
    fn zy(&self) -> Vector2<T> {
        Vector2::new(self.z, self.y)
    }
    fn zz(&self) -> Vector2<T> {
        Vector2::new(self.z, self.z)
    }
}

impl<T: Scalar> Swizzle2<T> for Vector4<T> {
    fn xx(&self) -> Vector2<T> {
        Vector2::new(self.x, self.x)
    }
    fn xy(&self) -> Vector2<T> {
        Vector2::new(self.x, self.y)
    }
    fn xz(&self) -> Vector2<T> {
        Vector2::new(self.x, self.z)
    }
    fn yx(&self) -> Vector2<T> {
        Vector2::new(self.y, self.x)
    }
    fn yy(&self) -> Vector2<T> {
        Vector2::new(self.y, self.y)
    }
    fn yz(&self) -> Vector2<T> {
        Vector2::new(self.y, self.z)
    }
    fn zx(&self) -> Vector2<T> {
        Vector2::new(self.z, self.x)
    }
    fn zy(&self) -> Vector2<T> {
        Vector2::new(self.z, self.y)
    }
    fn zz(&self) -> Vector2<T> {
        Vector2::new(self.z, self.z)
    }
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
    fn xxx(&self) -> Vector3<T> {
        Vector3::new(self.x, self.x, self.x)
    }
    fn xxy(&self) -> Vector3<T> {
        Vector3::new(self.x, self.x, self.y)
    }
    fn xxz(&self) -> Vector3<T> {
        Vector3::new(self.x, self.x, self.z)
    }
    fn xyx(&self) -> Vector3<T> {
        Vector3::new(self.x, self.y, self.x)
    }
    fn xyy(&self) -> Vector3<T> {
        Vector3::new(self.x, self.y, self.y)
    }
    fn xyz(&self) -> Vector3<T> {
        Vector3::new(self.x, self.y, self.z)
    }
    fn xzx(&self) -> Vector3<T> {
        Vector3::new(self.x, self.z, self.x)
    }
    fn xzy(&self) -> Vector3<T> {
        Vector3::new(self.x, self.z, self.y)
    }
    fn xzz(&self) -> Vector3<T> {
        Vector3::new(self.x, self.z, self.z)
    }

    fn yxx(&self) -> Vector3<T> {
        Vector3::new(self.y, self.x, self.x)
    }
    fn yxy(&self) -> Vector3<T> {
        Vector3::new(self.y, self.x, self.y)
    }
    fn yxz(&self) -> Vector3<T> {
        Vector3::new(self.y, self.x, self.z)
    }
    fn yyx(&self) -> Vector3<T> {
        Vector3::new(self.y, self.y, self.x)
    }
    fn yyy(&self) -> Vector3<T> {
        Vector3::new(self.y, self.y, self.y)
    }
    fn yyz(&self) -> Vector3<T> {
        Vector3::new(self.y, self.y, self.z)
    }
    fn yzx(&self) -> Vector3<T> {
        Vector3::new(self.y, self.z, self.x)
    }
    fn yzy(&self) -> Vector3<T> {
        Vector3::new(self.y, self.z, self.y)
    }
    fn yzz(&self) -> Vector3<T> {
        Vector3::new(self.y, self.z, self.z)
    }

    fn zxx(&self) -> Vector3<T> {
        Vector3::new(self.z, self.x, self.x)
    }
    fn zxy(&self) -> Vector3<T> {
        Vector3::new(self.z, self.x, self.y)
    }
    fn zxz(&self) -> Vector3<T> {
        Vector3::new(self.z, self.x, self.z)
    }
    fn zyx(&self) -> Vector3<T> {
        Vector3::new(self.z, self.y, self.x)
    }
    fn zyy(&self) -> Vector3<T> {
        Vector3::new(self.z, self.y, self.y)
    }
    fn zyz(&self) -> Vector3<T> {
        Vector3::new(self.z, self.y, self.z)
    }
    fn zzx(&self) -> Vector3<T> {
        Vector3::new(self.z, self.z, self.x)
    }
    fn zzy(&self) -> Vector3<T> {
        Vector3::new(self.z, self.z, self.y)
    }
    fn zzz(&self) -> Vector3<T> {
        Vector3::new(self.z, self.z, self.z)
    }
}

impl<T: Scalar> Swizzle3<T> for Vector4<T> {
    fn xxx(&self) -> Vector3<T> {
        Vector3::new(self.x, self.x, self.x)
    }
    fn xxy(&self) -> Vector3<T> {
        Vector3::new(self.x, self.x, self.y)
    }
    fn xxz(&self) -> Vector3<T> {
        Vector3::new(self.x, self.x, self.z)
    }
    fn xyx(&self) -> Vector3<T> {
        Vector3::new(self.x, self.y, self.x)
    }
    fn xyy(&self) -> Vector3<T> {
        Vector3::new(self.x, self.y, self.y)
    }
    fn xyz(&self) -> Vector3<T> {
        Vector3::new(self.x, self.y, self.z)
    }
    fn xzx(&self) -> Vector3<T> {
        Vector3::new(self.x, self.z, self.x)
    }
    fn xzy(&self) -> Vector3<T> {
        Vector3::new(self.x, self.z, self.y)
    }
    fn xzz(&self) -> Vector3<T> {
        Vector3::new(self.x, self.z, self.z)
    }

    fn yxx(&self) -> Vector3<T> {
        Vector3::new(self.y, self.x, self.x)
    }
    fn yxy(&self) -> Vector3<T> {
        Vector3::new(self.y, self.x, self.y)
    }
    fn yxz(&self) -> Vector3<T> {
        Vector3::new(self.y, self.x, self.z)
    }
    fn yyx(&self) -> Vector3<T> {
        Vector3::new(self.y, self.y, self.x)
    }
    fn yyy(&self) -> Vector3<T> {
        Vector3::new(self.y, self.y, self.y)
    }
    fn yyz(&self) -> Vector3<T> {
        Vector3::new(self.y, self.y, self.z)
    }
    fn yzx(&self) -> Vector3<T> {
        Vector3::new(self.y, self.z, self.x)
    }
    fn yzy(&self) -> Vector3<T> {
        Vector3::new(self.y, self.z, self.y)
    }
    fn yzz(&self) -> Vector3<T> {
        Vector3::new(self.y, self.z, self.z)
    }

    fn zxx(&self) -> Vector3<T> {
        Vector3::new(self.z, self.x, self.x)
    }
    fn zxy(&self) -> Vector3<T> {
        Vector3::new(self.z, self.x, self.y)
    }
    fn zxz(&self) -> Vector3<T> {
        Vector3::new(self.z, self.x, self.z)
    }
    fn zyx(&self) -> Vector3<T> {
        Vector3::new(self.z, self.y, self.x)
    }
    fn zyy(&self) -> Vector3<T> {
        Vector3::new(self.z, self.y, self.y)
    }
    fn zyz(&self) -> Vector3<T> {
        Vector3::new(self.z, self.y, self.z)
    }
    fn zzx(&self) -> Vector3<T> {
        Vector3::new(self.z, self.z, self.x)
    }
    fn zzy(&self) -> Vector3<T> {
        Vector3::new(self.z, self.z, self.y)
    }
    fn zzz(&self) -> Vector3<T> {
        Vector3::new(self.z, self.z, self.z)
    }
}

pub fn length<T: FloatScalar, V: Vector<T>>(v: &V) -> T {
    V::dot(v, v).tsqrt()
}
pub fn dot<T: Scalar, V: Vector<T>>(l: &V, r: &V) -> T {
    V::dot(l, r)
}
pub fn normalize<T: FloatScalar, V: FloatVector<T>>(v: &V) -> V {
    let len = v.length();
    *v / len
}
pub fn distance<T: FloatScalar, V: FloatVector<T>>(l: &V, r: &V) -> T {
    (*r - *l).length()
}
pub fn min<T: Scalar, V: Vector<T>>(l: &V, r: &V) -> V {
    V::min(l, r)
}
pub fn max<T: Scalar, V: Vector<T>>(l: &V, r: &V) -> V {
    V::max(l, r)
}

#[cfg(test)]
mod tests {
    use super::*;
    use crate::scalar::FloatScalar;
    
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
        
        let v3 = Vector3::<f32>::new(1.0, 2.0, 3.0);
        
        assert_eq!(v3.xz().x, 1.0);
        assert_eq!(v3.xz().y, 3.0);
        
        assert_eq!(v3.zy().x, 3.0);
        assert_eq!(v3.zy().y, 2.0);
        
        let v3_swizzle = v3.zyx();
        assert_eq!(v3_swizzle.x, 3.0);
        assert_eq!(v3_swizzle.y, 2.0);
        assert_eq!(v3_swizzle.z, 1.0);
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
}
