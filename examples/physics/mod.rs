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

//! Small example rigid-body engine. Each fixed tick integrates poses, runs
//! sweep-and-prune/body-vs-tree broad phase, builds contact manifolds, then solves
//! accumulated normal/friction impulses and split position constraints. Bodies
//! are append-only; static triangle soups are immutable. All simulation arithmetic
//! uses T, including native fixed point; only scene setup/rendering convert floats.
//! This is discrete collision detection with bounded example-scale inputs.

#[cfg(any(feature = "std", feature = "libm", feature = "system-libm"))]
pub mod scene;
#[cfg(test)]
mod tests;

use crate::collision::{
    self,
    aabb_tree::AabbTree,
    contact::{self, Contact, Convex},
};
use rs_math3d::{Box3, CrossProduct, Quat, RealScalar, Sphere3, TryCastFrom, Vector, Vector3};

/// World-local stable handle. No individual deletion or index reuse is exposed.
#[derive(Clone, Copy, Debug, PartialEq, Eq)]
pub struct BodyId(usize);
impl BodyId {
    /// Stable array index, useful for deterministic rendering/debug colors.
    pub fn index(self) -> usize {
        self.0
    }
}
/// Local, centered shape dimensions. Capsules use the local Y centerline.
#[derive(Clone, Copy)]
pub enum Shape<T: RealScalar> {
    /// Sphere radius.
    Sphere { radius: T },
    /// Capsule centerline half-length and spherical sweep radius.
    Capsule { half_height: T, radius: T },
    /// Oriented box half extents in its body's frame.
    Box { half: Vector3<T> },
}
/// Owned position and unit orientation. Body construction validates input poses.
#[derive(Clone, Copy)]
pub struct Pose<T: RealScalar> {
    /// Center of mass in world space.
    pub position: Vector3<T>,
    /// Local-to-world rotation; normalized at every integration/correction.
    pub orientation: Quat<T>,
}
impl<T: RealScalar> Pose<T> {
    /// Identity orientation at a supplied center of mass.
    pub fn at(position: Vector3<T>) -> Self {
        Self {
            position,
            orientation: Quat::identity(),
        }
    }
}
/// Coulomb friction and impact restitution, both validated in [0,1].
#[derive(Clone, Copy)]
pub struct Material<T: RealScalar> {
    /// Friction coefficient, mixed by geometric mean.
    pub friction: T,
    /// Normal impact restitution, mixed by maximum.
    pub restitution: T,
}
/// Explicit motion lifecycle. Sleeping is reversible by an impulse/energetic
/// collision; out-of-bounds bodies remain disabled until the world is reset.
#[derive(Clone, Copy)]
enum Motion {
    /// Integrates gravity and participates with finite mass.
    Awake {
        /// Consecutive supported quiet ticks; ninety transitions to sleep.
        quiet_ticks: u16,
    },
    /// At rest; acts as a static support until awakened.
    Sleeping,
    /// Outside the demo's bounded simulation domain; no longer participates.
    Disabled,
}
/// Construction failures. Simulation steps assume the documented numeric range.
#[derive(Clone, Copy, Debug, PartialEq, Eq)]
pub enum Error {
    /// Nonfinite/nonpositive dimensions, mass or unrepresentable derived inertia.
    InvalidBody,
    /// Invalid normalized orientation, position or velocity.
    InvalidPose,
    /// Material coefficient outside [0,1].
    InvalidMaterial,
    /// The configured append-only body limit is reached.
    Capacity,
    /// Disabled bodies cannot be awakened through motion commands.
    InactiveBody,
}

/// One dynamic body. Pose, dimensions and mass are not publicly mutable, keeping
/// its orientation cache and inertia consistent. External impulses wake it.
pub struct Body<T: RealScalar> {
    /// Authoritative local dimensions.
    shape: Shape<T>,
    /// Authoritative represented pose.
    pose: Pose<T>,
    /// Derived world axes; refreshed with every pose rotation.
    axes: [Vector3<T>; 3],
    /// Linear center-of-mass velocity.
    velocity: Vector3<T>,
    /// World-space angular velocity in radians per second.
    angular: Vector3<T>,
    /// Inverse physical mass, independent of temporary sleep state.
    inverse_mass: T,
    /// Diagonal inverse inertia in the local principal frame.
    inverse_inertia: Vector3<T>,
    /// Validated surface coefficients.
    material: Material<T>,
    /// Motion lifecycle; no hidden interior mutability.
    motion: Motion,
}

/// Exact setup conversion; all constants are bounded by the example's formats.
pub fn scalar<T: TryCastFrom<f64>>(x: f64) -> T {
    T::try_cast_from(x)
        .ok()
        .expect("physics constant fits scalar")
}
/// Zero vector without a floating backend.
fn zero<T: RealScalar>() -> Vector3<T> {
    Vector3::new(T::zero(), T::zero(), T::zero())
}
/// Finite component validation at public mutation boundaries.
fn finite<T: RealScalar>(v: Vector3<T>) -> bool {
    v.x.is_finite() && v.y.is_finite() && v.z.is_finite()
}
/// Vector dot product with fused fixed accumulation.
fn dot<T: RealScalar>(a: Vector3<T>, b: Vector3<T>) -> T {
    Vector3::dot(&a, &b)
}
/// Maximum component magnitude, also valid for sub-quantum squared distances.
fn magnitude<T: RealScalar>(v: Vector3<T>) -> T {
    T::max(v.x.tabs(), T::max(v.y.tabs(), v.z.tabs()))
}
/// First-order world-angular quaternion update followed by normalization.
fn rotate<T: RealScalar>(q: Quat<T>, angle: Vector3<T>) -> Quat<T> {
    let derivative = Quat::new(angle.x, angle.y, angle.z, T::zero()) * q;
    Quat::normalize(&Quat::add(&q, &(derivative * T::half())))
}
impl<T: RealScalar + TryCastFrom<f64>> Body<T> {
    /// Validates bounded dimensions/mass and prepares shape-specific inertia.
    /// Dimensions are in [0.1,4], centerline half-length in [0,4], mass in [0.1,10].
    /// Positions must remain within the world's domain and intermediates must fit T.
    pub fn new(
        shape: Shape<T>,
        pose: Pose<T>,
        mass: T,
        material: Material<T>,
    ) -> Result<Self, Error> {
        let n = scalar::<T>;
        let positive = |x: T| x.is_finite() && x >= n(0.1) && x <= n(4.0);
        if !mass.is_finite() || mass < n(0.1) || mass > n(10.0) {
            return Err(Error::InvalidBody);
        }
        if !finite(pose.position)
            || ![
                pose.orientation.x,
                pose.orientation.y,
                pose.orientation.z,
                pose.orientation.w,
            ]
            .iter()
            .all(|x| x.is_finite())
        {
            return Err(Error::InvalidPose);
        }
        if !material.friction.is_finite()
            || !material.restitution.is_finite()
            || material.friction < T::zero()
            || material.friction > T::one()
            || material.restitution < T::zero()
            || material.restitution > T::one()
        {
            return Err(Error::InvalidMaterial);
        }
        let inverse_mass = T::one().checked_div(mass).map_err(|_| Error::InvalidBody)?;
        let orientation = Quat::checked_normalize(&pose.orientation, T::zero())
            .map_err(|_| Error::InvalidPose)?;
        let inertia = match shape {
            Shape::Sphere { radius: r } => {
                if !positive(r) {
                    return Err(Error::InvalidBody);
                }
                Vector3::new(n(0.4) * r * r, n(0.4) * r * r, n(0.4) * r * r)
            }
            Shape::Box { half: h } => {
                if !positive(h.x) || !positive(h.y) || !positive(h.z) {
                    return Err(Error::InvalidBody);
                }
                Vector3::new(
                    (h.y * h.y + h.z * h.z) / n(3.0),
                    (h.x * h.x + h.z * h.z) / n(3.0),
                    (h.x * h.x + h.y * h.y) / n(3.0),
                )
            }
            Shape::Capsule {
                half_height: h,
                radius: r,
            } => {
                if !positive(r) || !h.is_finite() || h < T::zero() || h > n(4.0) {
                    return Err(Error::InvalidBody);
                }
                let cylinder = (n(2.0) * h) / (n(2.0) * h + n(4.0 / 3.0) * r);
                let caps = T::one() - cylinder;
                let y = cylinder * r * r * T::half() + caps * n(0.4) * r * r;
                let x = cylinder * (r * r * n(0.25) + h * h / n(3.0))
                    + caps * (n(0.4) * r * r + h * h + n(0.75) * h * r);
                Vector3::new(x, y, x)
            }
        };
        if inertia.x <= T::zero() || inertia.y <= T::zero() || inertia.z <= T::zero() {
            return Err(Error::InvalidBody);
        }
        let inverse_inertia = Vector3::new(
            inverse_mass
                .checked_div(inertia.x)
                .map_err(|_| Error::InvalidBody)?,
            inverse_mass
                .checked_div(inertia.y)
                .map_err(|_| Error::InvalidBody)?,
            inverse_mass
                .checked_div(inertia.z)
                .map_err(|_| Error::InvalidBody)?,
        );
        let mut body = Self {
            shape,
            pose: Pose {
                position: pose.position,
                orientation,
            },
            axes: [zero(); 3],
            velocity: zero(),
            angular: zero(),
            inverse_mass,
            inverse_inertia,
            material,
            motion: Motion::Awake { quiet_ticks: 0 },
        };
        body.refresh();
        Ok(body)
    }
    /// Read-only pose snapshot; mutating the copy cannot invalidate body caches.
    pub fn pose(&self) -> Pose<T> {
        self.pose
    }
    /// Local collider dimensions, also the rendering geometry source.
    pub fn shape(&self) -> Shape<T> {
        self.shape
    }
    /// Current conservative world envelope, also used by debug rendering.
    pub fn bounds(&self) -> Box3<T> {
        self.convex().bounds()
    }
    /// Linear velocity snapshot.
    pub fn velocity(&self) -> Vector3<T> {
        self.velocity
    }
    /// Angular velocity snapshot.
    pub fn angular_velocity(&self) -> Vector3<T> {
        self.angular
    }
    /// Whether the body is at rest with gravity temporarily disabled.
    pub fn sleeping(&self) -> bool {
        matches!(self.motion, Motion::Sleeping)
    }
    /// Whether this body remains in the bounded simulation domain.
    pub fn enabled(&self) -> bool {
        !matches!(self.motion, Motion::Disabled)
    }
    /// Sets initial/interactive velocities. Finite, representable magnitudes are
    /// required; the world clamps travel/spin before its next discrete tick.
    pub fn set_velocity(&mut self, linear: Vector3<T>, angular: Vector3<T>) -> Result<(), Error> {
        if !self.enabled() {
            return Err(Error::InactiveBody);
        }
        if !finite(linear) || !finite(angular) {
            return Err(Error::InvalidPose);
        }
        self.velocity = linear;
        self.angular = angular;
        self.wake();
        Ok(())
    }
    /// Applies a represented world impulse at a world point and wakes this body.
    pub fn apply_impulse(&mut self, point: Vector3<T>, impulse: Vector3<T>) -> Result<(), Error> {
        if !self.enabled() {
            return Err(Error::InactiveBody);
        }
        if !finite(point) || !finite(impulse) {
            return Err(Error::InvalidPose);
        }
        self.wake();
        self.impulse(point, impulse);
        Ok(())
    }
    /// Rebuilds the derived axes after normalization.
    fn refresh(&mut self) {
        self.pose.orientation = Quat::normalize(&self.pose.orientation);
        self.axes = self.pose.orientation.mat3().col;
    }
    /// Returns the current represented convex shape without heap allocation.
    fn convex(&self) -> Convex<T> {
        match self.shape {
            Shape::Sphere { radius } => Convex::Sphere(Sphere3::new(self.pose.position, radius)),
            Shape::Capsule {
                half_height,
                radius,
            } => Convex::Capsule(collision::Capsule {
                a: self.pose.position - self.axes[1] * half_height,
                b: self.pose.position + self.axes[1] * half_height,
                radius,
            }),
            Shape::Box { half } => Convex::Box(collision::Obb {
                center: self.pose.position,
                axes: self.axes,
                half,
            }),
        }
    }
    /// Converts a world anchor into this body's principal frame.
    fn local_point(&self, p: Vector3<T>) -> Vector3<T> {
        let d = p - self.pose.position;
        Vector3::new(
            dot(d, self.axes[0]),
            dot(d, self.axes[1]),
            dot(d, self.axes[2]),
        )
    }
    /// Converts a local anchor to its current world position.
    fn world_point(&self, p: Vector3<T>) -> Vector3<T> {
        self.pose.position + self.axes[0] * p.x + self.axes[1] * p.y + self.axes[2] * p.z
    }
    /// Wakes only reversible sleep state; disabled bodies are terminal.
    fn wake(&mut self) {
        if self.enabled() {
            self.motion = Motion::Awake { quiet_ticks: 0 };
        }
    }
    /// Solver inverse mass, zero while sleeping/disabled.
    fn mass(&self) -> T {
        if matches!(self.motion, Motion::Awake { .. }) {
            self.inverse_mass
        } else {
            T::zero()
        }
    }
    /// World inverse inertia action, using the current represented principal axes.
    fn inertia(&self, v: Vector3<T>) -> Vector3<T> {
        if self.mass() == T::zero() {
            return zero();
        }
        self.axes[0] * (dot(v, self.axes[0]) * self.inverse_inertia.x)
            + self.axes[1] * (dot(v, self.axes[1]) * self.inverse_inertia.y)
            + self.axes[2] * (dot(v, self.axes[2]) * self.inverse_inertia.z)
    }
    /// Contact point velocity, including rotation around the center of mass.
    fn point_velocity(&self, p: Vector3<T>) -> Vector3<T> {
        self.velocity + Vector3::cross(&self.angular, &(p - self.pose.position))
    }
    /// Internal solver impulse; sleeping bodies must be awakened before solving.
    fn impulse(&mut self, p: Vector3<T>, impulse: Vector3<T>) {
        self.velocity = self.velocity + impulse * self.mass();
        self.angular =
            self.angular + self.inertia(Vector3::cross(&(p - self.pose.position), &impulse));
    }
    /// Split position impulse changes pose but never adds kinetic energy.
    fn position_impulse(&mut self, p: Vector3<T>, impulse: Vector3<T>) {
        if self.mass() == T::zero() {
            return;
        }
        let angle = self.inertia(Vector3::cross(&(p - self.pose.position), &impulse));
        self.pose.position = self.pose.position + impulse * self.mass();
        self.pose.orientation = rotate(self.pose.orientation, angle);
        self.refresh();
    }
}

/// Immutable static object; triangles are already in world coordinates.
pub struct StaticMesh<T: RealScalar> {
    /// Owned immutable acceleration structure and rendering geometry.
    pub tree: AabbTree<T>,
    /// Surface coefficients shared by its triangles.
    pub material: Material<T>,
}
/// Explicit collision ownership. Static triangle identity is used in candidates;
/// constraints group one mesh's contacts for seam reduction and warm starting.
#[derive(Clone, Copy, PartialEq, Eq)]
enum Other {
    /// Another append-only body ID.
    Body(usize),
    /// Immutable static object ID; triangle IDs live only in candidates.
    Mesh(usize),
}
/// Broad-phase identities: body pairs cannot carry a meaningless triangle ID.
#[derive(Clone, Copy)]
enum Candidate {
    /// Two stable dynamic IDs in ascending order.
    Bodies { a: usize, b: usize },
    /// One body and an immutable tree's stable original triangle ID.
    Triangle {
        body: usize,
        mesh: usize,
        triangle: usize,
    },
}
/// Solver state for one contact, retained to warm-start the next fixed tick.
#[derive(Clone, Copy)]
struct Constraint<T: RealScalar> {
    /// Body A and the body/static mesh it touches.
    a: usize,
    /// Concrete second collider; identifies static seam/warm-start groups.
    other: Other,
    /// Common contact normal and orthonormal friction basis.
    normal: Vector3<T>,
    /// Two orthogonal tangents defining the Coulomb disk.
    tangent: [Vector3<T>; 2],
    /// Body-local anchors, or a world anchor for static B.
    anchor_a: Vector3<T>,
    /// Body B local anchor, or world point for a static mesh.
    anchor_b: Vector3<T>,
    /// Contact generation depth, before split correction.
    depth: T,
    /// Inverse normal/tangent effective masses.
    mass: [T; 3],
    /// Pre-solve restitution target, friction coefficient and accumulated impulses.
    bounce: T,
    /// Mixed coefficient limiting total tangential impulse.
    friction: T,
    /// Accumulated normal and two tangential impulses, in current basis.
    impulse: [T; 3],
}
/// Work counts for one completed fixed tick, excluding rendering.
#[derive(Clone, Copy, Default, Debug)]
pub struct Stats {
    /// Convex body/body broad-phase pairs.
    pub body_pairs: usize,
    /// Static triangle candidates delivered by immutable trees.
    pub triangle_candidates: usize,
    /// Bounds tests during tree traversal.
    pub bounds_tests: usize,
    /// Reduced solver contacts.
    pub contacts: usize,
    /// Bodies currently integrating.
    pub awake: usize,
    /// Bodies at rest.
    pub sleeping: usize,
    /// Bodies disabled after leaving the bounded demo domain.
    pub disabled: usize,
}
/// Preconverted solver configuration; no float constants occur in contact loops.
struct Config<T: RealScalar> {
    /// Represented fixed timestep and gravity.
    dt: T,
    /// Constant world acceleration, applied only to awake bodies.
    gravity: Vector3<T>,
    /// Native quantization/contact tolerances and split-impulse parameters.
    slop: T,
    /// Fraction of remaining penetration corrected per position iteration.
    position_rate: T,
    /// Maximum raw penetration correction before effective-mass weighting.
    max_correction: T,
    /// Local-anchor matching distance for previous-frame contact impulses.
    match_distance: T,
    /// Local point tolerance for merging duplicate mesh-seam contacts.
    merge_distance: T,
    /// Damping and motion limits for this discrete example.
    linear_damping: T,
    /// Per-tick spin damping, also a bounded rolling-resistance approximation.
    angular_damping: T,
    /// Maximum component of linear velocity before discrete integration.
    speed_limit: T,
    /// Maximum angular velocity component before quaternion integration.
    spin_limit: T,
    /// Absolute center-coordinate limit beyond which a body is disabled.
    domain: T,
    /// Contact wake/sleep thresholds and warm-start strength.
    wake_speed: T,
    /// Maximum linear component considered quiet for sleep accounting.
    sleep_linear: T,
    /// Maximum angular component considered quiet for sleep accounting.
    sleep_angular: T,
    /// Retained fraction of previous contact impulse.
    warm: T,
    /// Minimum normal alignment when merging seam points.
    normal_merge: T,
    /// Minimum normal alignment for warm-start matching.
    normal_match: T,
}
/// Owned simulation and reusable scratch. At most 128 dynamic bodies; static
/// soups can contain arbitrary triangle counts within T's numeric range.
pub struct World<T: RealScalar + TryCastFrom<f64>> {
    /// Stable append-only dynamic bodies and immutable static colliders.
    bodies: Vec<Body<T>>,
    /// Immutable static geometry, retained for the entire world's lifetime.
    meshes: Vec<StaticMesh<T>>,
    /// Current derived convex colliders/bounds and temporally coherent sweep IDs.
    shapes: Vec<Convex<T>>,
    /// Per-body represented envelopes for the current broad phase.
    bounds: Vec<Box3<T>>,
    /// Retained sweep order, insertion-sorted after every integration.
    order: Vec<usize>,
    /// Broad-phase candidates and double-buffered solver/cache state.
    candidates: Vec<Candidate>,
    /// Current reduced contact solver state.
    contacts: Vec<Constraint<T>>,
    /// Previous solver state, reused as warm cache and next scratch buffer.
    previous: Vec<Constraint<T>>,
    /// Numeric configuration and last completed work counters.
    config: Config<T>,
    /// Last completed tick, rather than accumulated application-frame counters.
    stats: Stats,
}
impl<T: RealScalar + TryCastFrom<f64>> World<T> {
    /// Creates a 120 Hz world. Use bounded positions/dimensions and static mesh
    /// edges (the supplied scene tiles its floor to preserve S15.16 intermediates).
    pub fn new() -> Self {
        let n = scalar::<T>;
        Self {
            bodies: Vec::new(),
            meshes: Vec::new(),
            shapes: Vec::new(),
            bounds: Vec::new(),
            order: Vec::new(),
            candidates: Vec::new(),
            contacts: Vec::new(),
            previous: Vec::new(),
            stats: Stats::default(),
            config: Config {
                dt: n(1.0 / 120.0),
                gravity: Vector3::new(n(0.0), n(-9.81), n(0.0)),
                slop: n(0.004),
                position_rate: n(0.25),
                max_correction: n(0.15),
                match_distance: n(0.06),
                merge_distance: n(0.015),
                linear_damping: n(0.9998),
                angular_damping: n(0.997),
                speed_limit: n(15.0),
                spin_limit: n(15.0),
                domain: n(24.0),
                wake_speed: n(0.15),
                sleep_linear: n(0.06),
                sleep_angular: n(0.1),
                warm: n(0.8),
                normal_merge: n(0.98),
                normal_match: n(0.95),
            },
        }
    }
    /// Adds a validated body; its handle is stable until the entire world resets.
    pub fn add_body(&mut self, body: Body<T>) -> Result<BodyId, Error> {
        if self.bodies.len() >= 128 {
            return Err(Error::Capacity);
        }
        if magnitude(body.pose.position) > self.config.domain {
            return Err(Error::InvalidPose);
        }
        let id = BodyId(self.bodies.len());
        self.bodies.push(body);
        Ok(id)
    }
    /// Adds immutable world-space triangles and material. Validate material as
    /// for bodies; the tree's constructor has already validated triangle storage.
    pub fn add_mesh(&mut self, mesh: StaticMesh<T>) -> Result<usize, Error> {
        let m = mesh.material;
        if !m.friction.is_finite()
            || !m.restitution.is_finite()
            || m.friction < T::zero()
            || m.friction > T::one()
            || m.restitution < T::zero()
            || m.restitution > T::one()
        {
            return Err(Error::InvalidMaterial);
        }
        let id = self.meshes.len();
        self.meshes.push(mesh);
        Ok(id)
    }
    /// World-local body lookup; only controlled mutation methods are exposed.
    pub fn body(&self, id: BodyId) -> Option<&Body<T>> {
        self.bodies.get(id.0)
    }
    /// World-local mutable body lookup for impulses/velocity changes.
    pub fn body_mut(&mut self, id: BodyId) -> Option<&mut Body<T>> {
        self.bodies.get_mut(id.0)
    }
    /// Bodies in stable insertion order, including disabled ones for diagnostics.
    pub fn bodies(&self) -> impl Iterator<Item = (BodyId, &Body<T>)> {
        self.bodies.iter().enumerate().map(|(i, b)| (BodyId(i), b))
    }
    /// Immutable static geometry for rendering or query inspection.
    pub fn meshes(&self) -> &[StaticMesh<T>] {
        &self.meshes
    }
    /// Last completed tick's work counters.
    pub fn stats(&self) -> Stats {
        self.stats
    }
    /// Replaces gravity at a validated, finite mutation boundary.
    pub fn set_gravity(&mut self, gravity: Vector3<T>) -> Result<(), Error> {
        if !finite(gravity) {
            return Err(Error::InvalidPose);
        }
        self.config.gravity = gravity;
        Ok(())
    }
    /// Contact point/normal snapshots for debug rendering.
    pub fn contacts(&self) -> impl Iterator<Item = (Vector3<T>, Vector3<T>)> + '_ {
        self.contacts
            .iter()
            .map(move |c| (self.bodies[c.a].world_point(c.anchor_a), c.normal))
    }
    /// Integrates bounded motion before the two collision phases.
    fn integrate(&mut self) {
        let c = &self.config;
        for body in &mut self.bodies {
            if matches!(body.motion, Motion::Awake { .. }) {
                body.velocity = body.velocity + c.gravity * c.dt;
                for (v, limit) in [
                    (&mut body.velocity, c.speed_limit),
                    (&mut body.angular, c.spin_limit),
                ] {
                    let m = magnitude(*v);
                    if m > limit {
                        *v = *v * (limit / m);
                    }
                }
                body.pose.position = body.pose.position + body.velocity * c.dt;
                body.pose.orientation = rotate(body.pose.orientation, body.angular * c.dt);
                body.refresh();
                if magnitude(body.pose.position) > c.domain {
                    body.motion = Motion::Disabled;
                    body.velocity = zero();
                    body.angular = zero();
                }
            }
        }
    }
    /// Phase one: insertion-sorted X sweep with Y/Z rejection, then per-body
    /// static AABB-tree traversal. Candidates contain identities, never contacts.
    fn broad_phase(&mut self) {
        self.shapes.clear();
        self.bounds.clear();
        self.candidates.clear();
        let bodies = &self.bodies;
        self.order.retain(|&i| bodies[i].enabled());
        for (i, b) in self.bodies.iter().enumerate() {
            let shape = b.convex();
            self.bounds.push(shape.bounds());
            self.shapes.push(shape);
            if b.enabled() && !self.order.contains(&i) {
                self.order.push(i);
            }
        }
        for i in 1..self.order.len() {
            let id = self.order[i];
            let mut j = i;
            while j > 0 {
                let previous = self.order[j - 1];
                let order = self.bounds[id]
                    .min
                    .x
                    .partial_cmp(&self.bounds[previous].min.x)
                    .unwrap()
                    .then(id.cmp(&previous));
                if !order.is_lt() {
                    break;
                }
                self.order[j] = previous;
                j -= 1;
            }
            self.order[j] = id;
        }
        for (i, &a) in self.order.iter().enumerate() {
            for &b in &self.order[i + 1..] {
                if self.bounds[b].min.x > self.bounds[a].max.x {
                    break;
                }
                if self.bodies[a].sleeping() && self.bodies[b].sleeping() {
                    continue;
                }
                if contact::overlaps(&self.bounds[a], &self.bounds[b]) {
                    let (a, b) = (a.min(b), a.max(b));
                    self.candidates.push(Candidate::Bodies { a, b });
                    self.stats.body_pairs += 1;
                }
            }
        }
        for &a in &self.order {
            if self.bodies[a].sleeping() {
                continue;
            }
            for (mesh_id, mesh) in self.meshes.iter().enumerate() {
                let candidates = &mut self.candidates;
                let stats = mesh
                    .tree
                    .for_each_candidate(&self.bounds[a], |triangle, _| {
                        candidates.push(Candidate::Triangle {
                            body: a,
                            mesh: mesh_id,
                            triangle,
                        })
                    });
                self.stats.bounds_tests += stats.bounds_tests;
                self.stats.triangle_candidates += stats.triangle_tests;
            }
        }
    }
    /// Relative contact velocity, with immutable static surfaces at zero velocity.
    fn relative_velocity(&self, a: usize, b: Other, p: Vector3<T>) -> Vector3<T> {
        self.bodies[a].point_velocity(p)
            - match b {
                Other::Body(id) => self.bodies[id].point_velocity(p),
                Other::Mesh(_) => zero(),
            }
    }
    /// Effective inverse mass including rotational leverage at one world point.
    fn effective(&self, a: usize, b: Other, p: Vector3<T>, n: Vector3<T>) -> T {
        let body = &self.bodies[a];
        let r = p - body.pose.position;
        let mut mass =
            body.mass() + dot(n, Vector3::cross(&body.inertia(Vector3::cross(&r, &n)), &r));
        if let Other::Body(id) = b {
            let body = &self.bodies[id];
            let r = p - body.pose.position;
            mass = mass
                + body.mass()
                + dot(n, Vector3::cross(&body.inertia(Vector3::cross(&r, &n)), &r));
        }
        if mass > T::zero() {
            T::one() / mass
        } else {
            T::zero()
        }
    }
    /// Builds one reduced, warm-started constraint. Coincident seam points merge;
    /// each body/object pair retains at most eight deepest contacts.
    fn add_contact(&mut self, a: usize, other: Other, c: Contact<T>) {
        if let Other::Body(b) = other {
            let energetic = magnitude(self.bodies[a].velocity - self.bodies[b].velocity)
                > self.config.wake_speed;
            if energetic || c.depth > self.config.slop + self.config.slop {
                self.bodies[a].wake();
                self.bodies[b].wake();
            }
        }
        if self.bodies[a].mass() == T::zero()
            && match other {
                Other::Body(b) => self.bodies[b].mass() == T::zero(),
                Other::Mesh(_) => true,
            }
        {
            return;
        }
        let anchor_a = self.bodies[a].local_point(c.point);
        let anchor_b = match other {
            Other::Body(b) => self.bodies[b].local_point(c.point),
            Other::Mesh(_) => c.point,
        };
        let mut count = 0;
        let mut shallow = None;
        for (i, old) in self.contacts.iter().enumerate() {
            if old.a == a && old.other == other {
                if magnitude(old.anchor_a - anchor_a) < self.config.merge_distance
                    && dot(old.normal, c.normal) > self.config.normal_merge
                {
                    return;
                }
                count += 1;
                if shallow.is_none_or(|j: usize| old.depth < self.contacts[j].depth) {
                    shallow = Some(i);
                }
            }
        }
        if count >= 8 && shallow.is_some_and(|i| self.contacts[i].depth >= c.depth) {
            return;
        }
        let helper = if c.normal.x.tabs() < c.normal.y.tabs() {
            Vector3::new(T::one(), T::zero(), T::zero())
        } else {
            Vector3::new(T::zero(), T::one(), T::zero())
        };
        let t1 = contact::unit(Vector3::cross(&c.normal, &helper)).unwrap_or(Vector3::new(
            T::zero(),
            T::zero(),
            T::one(),
        ));
        let t2 = Vector3::cross(&c.normal, &t1);
        let material_b = match other {
            Other::Body(b) => self.bodies[b].material,
            Other::Mesh(m) => self.meshes[m].material,
        };
        let material_a = self.bodies[a].material;
        let vn = dot(self.relative_velocity(a, other, c.point), c.normal);
        let bounce = if vn < -T::one() {
            -vn * T::max(material_a.restitution, material_b.restitution)
        } else {
            T::zero()
        };
        let mut impulse = [T::zero(); 3];
        if let Some(old) = self.previous.iter().find(|old| {
            old.a == a
                && old.other == other
                && dot(old.normal, c.normal) > self.config.normal_match
                && magnitude(old.anchor_a - anchor_a) < self.config.match_distance
        }) {
            let friction = old.tangent[0] * old.impulse[1] + old.tangent[1] * old.impulse[2];
            impulse = [
                old.impulse[0] * self.config.warm,
                dot(friction, t1) * self.config.warm,
                dot(friction, t2) * self.config.warm,
            ];
        }
        let constraint = Constraint {
            a,
            other,
            normal: c.normal,
            tangent: [t1, t2],
            anchor_a,
            anchor_b,
            depth: c.depth,
            mass: [
                self.effective(a, other, c.point, c.normal),
                self.effective(a, other, c.point, t1),
                self.effective(a, other, c.point, t2),
            ],
            bounce,
            friction: (material_a.friction * material_b.friction).tsqrt(),
            impulse,
        };
        if count >= 8 {
            self.contacts[shallow.unwrap()] = constraint;
        } else {
            self.contacts.push(constraint);
        }
    }
    /// Phase two: only broad-phase candidates invoke the shared collision/contact
    /// module. Static triangle witnesses retain original tree IDs.
    fn narrow_phase(&mut self) {
        std::mem::swap(&mut self.contacts, &mut self.previous);
        self.contacts.clear();
        for i in 0..self.candidates.len() {
            let (a, other, manifold) = match self.candidates[i] {
                Candidate::Bodies { a, b } => (
                    a,
                    Other::Body(b),
                    contact::convex(&self.shapes[a], &self.shapes[b]),
                ),
                Candidate::Triangle {
                    body: a,
                    mesh: m,
                    triangle,
                } => (
                    a,
                    Other::Mesh(m),
                    contact::triangle(
                        &self.shapes[a],
                        self.meshes[m].tree.triangle(triangle).unwrap(),
                    ),
                ),
            };
            for &point in manifold.points() {
                self.add_contact(a, other, point);
            }
        }
        self.stats.contacts = self.contacts.len();
    }
    /// Applies equal/opposite impulses without aliasing mutable body references.
    fn impulse(&mut self, c: Constraint<T>, impulse: Vector3<T>) {
        let p = self.bodies[c.a].world_point(c.anchor_a);
        self.bodies[c.a].impulse(p, impulse);
        if let Other::Body(b) = c.other {
            let p = self.bodies[b].world_point(c.anchor_b);
            self.bodies[b].impulse(p, -impulse);
        }
    }
    /// Sequential normal impulses and a circular two-axis Coulomb friction cone.
    fn solve_velocity(&mut self) {
        for i in 0..self.contacts.len() {
            let c = self.contacts[i];
            self.impulse(
                c,
                c.normal * c.impulse[0] + c.tangent[0] * c.impulse[1] + c.tangent[1] * c.impulse[2],
            );
        }
        for _ in 0..12 {
            for i in 0..self.contacts.len() {
                let mut c = self.contacts[i];
                let p = self.bodies[c.a].world_point(c.anchor_a);
                let vn = dot(self.relative_velocity(c.a, c.other, p), c.normal);
                let old = c.impulse[0];
                c.impulse[0] = T::max(T::zero(), old + (c.bounce - vn) * c.mass[0]);
                self.impulse(c, c.normal * (c.impulse[0] - old));
                let velocity = self.relative_velocity(c.a, c.other, p);
                let old = [c.impulse[1], c.impulse[2]];
                let mut next = [
                    old[0] - dot(velocity, c.tangent[0]) * c.mass[1],
                    old[1] - dot(velocity, c.tangent[1]) * c.mass[2],
                ];
                let length = T::norm(&next);
                let limit = c.friction * c.impulse[0];
                if length > limit && length > T::zero() {
                    let factor = limit / length;
                    next[0] = next[0] * factor;
                    next[1] = next[1] * factor;
                }
                self.impulse(
                    c,
                    c.tangent[0] * (next[0] - old[0]) + c.tangent[1] * (next[1] - old[1]),
                );
                c.impulse[1] = next[0];
                c.impulse[2] = next[1];
                self.contacts[i] = c;
            }
        }
    }
    /// Split position impulses correct overlap without injecting bounce energy.
    fn solve_position(&mut self) {
        for _ in 0..4 {
            for i in 0..self.contacts.len() {
                let c = self.contacts[i];
                let pa = self.bodies[c.a].world_point(c.anchor_a);
                let pb = match c.other {
                    Other::Body(b) => self.bodies[b].world_point(c.anchor_b),
                    Other::Mesh(_) => c.anchor_b,
                };
                let depth = c.depth + dot(pb - pa, c.normal) - self.config.slop;
                if depth <= T::zero() {
                    continue;
                }
                let amount = T::min(
                    depth * self.config.position_rate,
                    self.config.max_correction,
                ) * self.effective(c.a, c.other, pa, c.normal);
                let impulse = c.normal * amount;
                self.bodies[c.a].position_impulse(pa, impulse);
                if let Other::Body(b) = c.other {
                    self.bodies[b].position_impulse(pb, -impulse);
                }
            }
        }
    }
    /// Completes sleep transitions and damping after contacts settle.
    fn sleep(&mut self) {
        for (i, b) in self.bodies.iter_mut().enumerate() {
            if let Motion::Awake { quiet_ticks } = b.motion {
                b.velocity = b.velocity * self.config.linear_damping;
                b.angular = b.angular * self.config.angular_damping;
                let supported = self
                    .contacts
                    .iter()
                    .any(|c| c.a == i || c.other == Other::Body(i));
                let quiet = supported
                    && magnitude(b.velocity) < self.config.sleep_linear
                    && magnitude(b.angular) < self.config.sleep_angular;
                let count = if quiet {
                    quiet_ticks.saturating_add(1)
                } else {
                    0
                };
                if count >= 90 {
                    b.motion = Motion::Sleeping;
                    b.velocity = zero();
                    b.angular = zero();
                } else {
                    b.motion = Motion::Awake { quiet_ticks: count };
                }
            }
            match b.motion {
                Motion::Awake { .. } => self.stats.awake += 1,
                Motion::Sleeping => self.stats.sleeping += 1,
                Motion::Disabled => self.stats.disabled += 1,
            };
        }
    }
    /// Advances exactly one represented 1/120-second tick. Frames and wall-clock
    /// catch-up belong to the application, never to simulation state.
    pub fn step(&mut self) {
        self.stats = Stats::default();
        self.integrate();
        self.broad_phase();
        self.narrow_phase();
        self.solve_velocity();
        self.solve_position();
        self.sleep();
    }
}
