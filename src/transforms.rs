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
use crate::vector::*;
use crate::matrix::*;
use crate::quaternion::*;

pub fn translate<T: Scalar>(trans: Vector3<T>) -> Matrix4<T> {
    Matrix4::new(T::one(), T::zero(), T::zero(), T::zero(),
                 T::zero(), T::one(), T::zero(), T::zero(),
                 T::zero(), T::zero(), T::one(), T::zero(),
                 trans.x, trans.y, trans.z, T::one())
}

pub fn scale<T: Scalar>(scale: Vector3<T>) -> Matrix4<T> {
    Matrix4::new(scale.x, T::zero(), T::zero(), T::zero(),
                 T::zero(), scale.y, T::zero(), T::zero(),
                 T::zero(), T::zero(), scale.z, T::zero(),
                 T::zero(), T::zero(), T::zero(), T::one())
}

pub fn rotation_from_quat<T: Scalar>(q: &Quat<T>) -> Matrix4<T> {
    Quat::mat4(q)
}

pub fn rotation_from_axis_angle<T: Scalar>(axis: &Vector3<T>, angle: T) -> Matrix4<T> {
    Quat::mat4(&Quat::of_axis_angle(axis, angle))
}

pub fn transform_vec3<T: Scalar>(m: &Matrix4<T>, v: &Vector3<T>) -> Vector3<T> {
    let v4 = Vector4::new(v.x, v.y, v.z, T::one());
    let vout = *m * v4;
    Vector3::new(vout.x / vout.w, vout.y / vout.w, vout.z / vout.w)
}

pub fn project3<T: Scalar>(world: &Matrix4<T>, persp: &Matrix4<T>, lb: &Vector2<T>, rt: &Vector2<T>, pt: &Vector3<T>) -> Vector3<T> {
    let	inp	= Vector4::new(pt.x, pt.y, pt.z, T::one());
    let	pw	= *persp * *world;
    let	mut out	= pw * inp;

    out.x	/= out.w;
    out.y	/= out.w;
    out.z	/= out.w;

    let out_x	= lb.x + ((rt.x - lb.x) * (out.x + T::one()) * T::half());
    let out_y	= lb.y + ((rt.y - lb.y) * (out.y + T::one()) * T::half());
    let out_z	= (out.z + T::one()) * T::half();
    Vector3::new(out_x, out_y, out_z)
}

pub fn unproject3<T: Scalar>(world: &Matrix4<T>, persp: &Matrix4<T>, lb: &Vector2<T>, rt: &Vector2<T>, pt: &Vector3<T>) -> Vector3<T> {
    let	pw	= *persp *  *world;
    let	inv	= pw.inverse();
    let in_x = (T::two() * (pt.x - lb.x) / (rt.x - lb.x)) - T::one();
    let in_y = (T::two() * (pt.y - lb.y) / (rt.y - lb.y)) - T::one();
    let in_z = (T::two() * pt.z) - T::one();
    let in_w = T::one();
    let inp = Vector4::new(in_x, in_y, in_z, in_w);
    let out = inv * inp;
    let out4 = out / out.w;
    Vector3::new(out4.x, out4.y, out4.z)
}

pub fn frustum<T: Scalar>(lbn: &Vector3<T>, rtf: &Vector3<T>) -> Matrix4<T>{
    let	width	= rtf.x - lbn.x;
    let	height	= rtf.y - lbn.y;
    let	depth	= rtf.z - lbn.z;
    let	a	= (rtf.x + lbn.x) / width;
    let	b	= (rtf.y + lbn.y) / height;
    let	c	= -(rtf.z + lbn.z) / depth;
    let	d	= -(T::two() * rtf.z * lbn.z) / depth;

    Matrix4::new(T::two() * lbn.z / width,  T::zero(),                  T::zero(),  T::zero(),
                 T::zero(),                 T::two() * lbn.z / height,  T::zero(),  T::zero(),
                 a,                         b,                          c,          -T::one(),
                 T::zero(),                 T::zero(),                  d,          T::zero())
}

pub fn ortho4<T: Scalar>(left: T, right: T, bottom: T, top: T, near: T, far: T) -> Matrix4<T> {
    let width   = right - left;
    let height  = top - bottom;
    let depth   = far - near;
    let r00	= T::two() / width;
    let r11	= T::two() / height;
    let r22	= -T::two() / depth;
    let r03	= -(right + left) / width;
    let r13	= -(top + bottom) / height;
    let r23	= -(far + near) / depth;
    Matrix4::new(r00,       T::zero(),  T::zero(),  T::zero(),
                T::zero(),  r11,        T::zero(),  T::zero(),
                T::zero(),  T::zero(),  r22,        T::zero(),
                r03,        r13,        r23,        T::one())
}

pub fn perspective<T: Scalar>(fovy: T, aspect: T, near: T, far: T) -> Matrix4<T> {
    let f       = T::one() / T::ttan(fovy * T::half());
    let denom   = near - far;
    let a       = (far + near) / denom;
    let b       = (T::two() * far * near) / denom;

    Matrix4::new(f / aspect, T::zero(), T::zero(),  T::zero(),
                 T::zero(), f,          T::zero(),  T::zero(),
                 T::zero(), T::zero(),  a,          -T::one(),
                 T::zero(), T::zero(),  b,          T::zero())
}

pub fn lookat<T: Scalar>(eye: &Vector3<T>, dest: &Vector3<T>, up: &Vector3<T>) -> Matrix4<T> {
    let f	= Vector3::normalize(&(*dest - *eye));
    let s	= Vector3::normalize(&Vector3::cross(&f, up));
    let u	= Vector3::normalize(&Vector3::cross(&s, &f));

    let trans   = translate(-*eye);

    let m = Matrix4::new(s.x, u.x, -f.x, T::zero(),
                         s.y, u.y, -f.y, T::zero(),
                         s.z, u.z, -f.z, T::zero(),
                         T::zero(), T::zero(), T::zero(), T::one());
    m * trans
}

// decompose a matrix into scale, rotation and translation
pub fn decompose<T: Scalar>(m: &Matrix4<T>) -> Option<(Vector3<T>, Quat<T>, Vector3<T>)> {

    let	mut col0	= Vector3::new(m.col[0].x, m.col[0].y, m.col[0].z);
    let	mut col1	= Vector3::new(m.col[1].x, m.col[1].y, m.col[1].z);
    let	mut col2	= Vector3::new(m.col[2].x, m.col[2].y, m.col[2].z);
    let det	        = m.determinant();

    // the scale needs to be tested
    let mut scale	= Vector3::new(col0.length(), col1.length(), col2.length());
    let trans	= Vector3::new(m.col[3].x, m.col[3].y, m.col[3].z);

    if det < T::zero() {
        scale	= -scale;
    }

    if scale.x != T::zero() {
        col0	= col0 / scale.x;
    } else {
        return Option::None;
    }

    if scale.y != T::zero() {
        col1	= col1 / scale.y;
    } else {
        return Option::None;
    }

    if scale.z != T::zero() {
        col2	= col2 / scale.z;
    } else {
        return Option::None;
    }

    let rot_matrix = Matrix3::new(col0.x, col0.y, col0.z,
                                 col1.x, col1.y, col1.z,
                                 col2.x, col2.y, col2.z);

    let rot	= Quat::of_matrix3(&rot_matrix);

    Some((scale, rot, trans))
}