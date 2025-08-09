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
//! Coordinate basis and reference frame utilities.
//!
//! This module provides types and functions for working with coordinate systems
//! and reference frames in 3D space. A basis represents a local coordinate system
//! with its own origin and axis orientations.
//!
//! # Examples
//!
//! ```
//! use rs_math3d::basis::{Basis, BasisPlane};
//! use rs_math3d::vector::Vector3;
//! 
//! // Create a basis at origin with default axes
//! let basis = Basis::<f32>::default();
//! 
//! // Create a basis with custom center
//! let center = Vector3::new(10.0, 5.0, 0.0);
//! let basis = Basis::default_with_center(&center);
//! ```

use crate::matrix::*;
use crate::scalar::*;
use crate::vector::*;
use num_traits::{Zero, One};

/// Represents one of the three coordinate planes.
#[derive(Debug, Clone, Copy)]
pub enum BasisPlane {
    YZ,
    ZX,
    XY,
}

impl BasisPlane {
    /// Converts the plane to a numeric ID (0=YZ, 1=ZX, 2=XY).
    pub fn to_id(&self) -> usize {
        match self {
            BasisPlane::YZ => 0,
            BasisPlane::ZX => 1,
            BasisPlane::XY => 2,
        }
    }

    /// Creates a plane from a numeric ID.
    ///
    /// # Panics
    /// Panics if id is not 0, 1, or 2.
    pub fn of_id(id: usize) -> Self {
        match id {
            0 => BasisPlane::YZ,
            1 => BasisPlane::ZX,
            2 => BasisPlane::XY,
            _ => panic!("invalid id"),
        }
    }
}

/// A 3D coordinate basis (reference frame).
///
/// Represents a local coordinate system with three orthogonal axes
/// and an origin point. This is useful for transformations between
/// different coordinate spaces.
#[derive(Debug, Clone, Copy)]
#[repr(C)]
pub struct Basis<T: Scalar> {
    pub x_axis: Vector3<T>,
    pub y_axis: Vector3<T>,
    pub z_axis: Vector3<T>,
    pub center: Vector3<T>,
}

impl<T: Scalar> Basis<T> {
    /// Returns the center (origin) of the basis.
    pub fn center(&self) -> &Vector3<T> {
        &self.center
    }

    /// Returns a mutable reference to the center.
    pub fn center_mut(&mut self) -> &mut Vector3<T> {
        &mut self.center
    }

    /// Converts the basis to a 4x4 transformation matrix.
    ///
    /// The resulting matrix transforms from local basis space to world space:
    /// - Columns 0-2: The basis axes (rotation/scale)
    /// - Column 3: The center point (translation)
    pub fn to_mat4(&self) -> Matrix4<T> {
        Matrix4::new(
            self.x_axis.x,
            self.x_axis.y,
            self.x_axis.z,
            <T as Zero>::zero(),
            self.y_axis.x,
            self.y_axis.y,
            self.y_axis.z,
            <T as Zero>::zero(),
            self.z_axis.x,
            self.z_axis.y,
            self.z_axis.z,
            <T as Zero>::zero(),
            self.center.x,
            self.center.y,
            self.center.z,
            <T as One>::one(),
        )
    }

    /// Creates a basis from a 4x4 transformation matrix.
    ///
    /// Extracts the axes from columns 0-2 and center from column 3.
    pub fn of_mat4(mat: &Matrix4<T>) -> Self {
        let col0 = mat.col[0];
        let col1 = mat.col[1];
        let col2 = mat.col[2];
        let col3 = mat.col[3];
        Self {
            center: col3.xyz(),
            x_axis: col0.xyz(),
            y_axis: col1.xyz(),
            z_axis: col2.xyz(),
        }
    }

    pub fn default() -> Self {
        Self {
            center: Vector3::new(<T as Zero>::zero(), <T as Zero>::zero(), <T as Zero>::zero()),
            x_axis: Vector3::new(<T as One>::one(), <T as Zero>::zero(), <T as Zero>::zero()),
            y_axis: Vector3::new(<T as Zero>::zero(), <T as One>::one(), <T as Zero>::zero()),
            z_axis: Vector3::new(<T as Zero>::zero(), <T as Zero>::zero(), <T as One>::one()),
        }
    }

    pub fn default_with_center(center: &Vector3<T>) -> Self {
        Self {
            center: *center,
            x_axis: Vector3::new(<T as One>::one(), <T as Zero>::zero(), <T as Zero>::zero()),
            y_axis: Vector3::new(<T as Zero>::zero(), <T as One>::one(), <T as Zero>::zero()),
            z_axis: Vector3::new(<T as Zero>::zero(), <T as Zero>::zero(), <T as One>::one()),
        }
    }

    pub fn plane_axis(&self, plane: BasisPlane) -> (&Vector3<T>, &Vector3<T>) {
        match plane {
            BasisPlane::YZ => (&self.y_axis, &self.z_axis),
            BasisPlane::ZX => (&self.z_axis, &self.x_axis),
            BasisPlane::XY => (&self.x_axis, &self.y_axis),
        }
    }

    pub fn plane_axis_mut(&mut self, plane: BasisPlane) -> (&mut Vector3<T>, &mut Vector3<T>) {
        match plane {
            BasisPlane::YZ => (&mut self.y_axis, &mut self.z_axis),
            BasisPlane::ZX => (&mut self.z_axis, &mut self.x_axis),
            BasisPlane::XY => (&mut self.x_axis, &mut self.y_axis),
        }
    }

    /// Converts the basis axes to a 3x3 rotation matrix.
    ///
    /// The matrix contains only the rotation part, without translation.
    pub fn to_mat3(&self) -> Matrix3<T> {
        Matrix3::new(
            self.x_axis.x,
            self.x_axis.y,
            self.x_axis.z,
            self.y_axis.x,
            self.y_axis.y,
            self.y_axis.z,
            self.z_axis.x,
            self.z_axis.y,
            self.z_axis.z,
        )
    }

    /// Creates a basis from a center point and a 3x3 rotation matrix.
    ///
    /// The matrix columns become the basis axes.
    pub fn of_center_mat3(center: &Vector3<T>, m: Matrix3<T>) -> Self {
        Self {
            center: *center,
            x_axis: m.col[0],
            y_axis: m.col[1],
            z_axis: m.col[2],
        }
    }
}
