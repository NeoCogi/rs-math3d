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

//! Versioned raw fixed-point scene inputs and tick-indexed command replay.
//! Seeds describe constructor inputs, not resumable mid-simulation snapshots.
//! Fingerprints include all body/cache/configuration values, using explicit
//! little-endian words and FNV-1a rather than architecture-dependent Hash APIs.

use super::*;
use core::convert::{TryFrom, TryInto};
use rs_math3d::{Fixed, Tri3};

#[cfg(test)]
mod tests;

/// Wire identity: changing field order, solver contracts or input semantics must
/// bump this version and deliberately regenerate the independently checked traces.
const MAGIC: &[u8; 8] = b"M3RPLY02";
/// Version 02 removes the obsolete dynamic-contact wake-speed threshold; the
/// remaining raw fields define static-support-only sleeping on every backend.
const CONFIG_SCALARS: usize = 19;
/// Replay resource limits bound allocation before untrusted lengths are used.
const MAX_TRIANGLES: usize = 32_768;
/// Upper bounds for scene resources and execution requested by a seed packet.
const MAX_MESHES: usize = 16;
/// Maximum replay duration; all tick indices fit equally on 32/64-bit hosts.
const MAX_TICKS: u32 = 100_000;
/// Maximum number of input commands, including commands sharing a tick.
const MAX_INPUTS: usize = 4096;

/// Prevents a float or platform-sized integer from becoming a replay backend.
mod sealed {
    /// Implemented only by the two fixed scalar formats with recorded fixtures.
    pub trait Backend {}
    impl Backend for rs_math3d::Fixed<i32, 16> {}
    impl Backend for rs_math3d::Fixed<i64, 32> {}
}
/// Exactly supported raw fixed formats. Every scalar occupies one sign-extended
/// i64 little-endian wire word; the header distinguishes storage/fraction widths.
pub trait ReplayScalar: sealed::Backend + RealScalar + TryCastFrom<f64> {
    /// Signed storage width, independent of pointer width.
    const BITS: u32;
    /// Binary fractional width.
    const FRAC: u32;
    /// Exact sign-extended storage bits.
    fn raw(self) -> i64;
    /// Rejects a wire word that does not fit this format's storage.
    fn from_raw(raw: i64) -> Option<Self>;
}
impl ReplayScalar for Fixed<i32, 16> {
    const BITS: u32 = 32;
    const FRAC: u32 = 16;
    fn raw(self) -> i64 {
        self.to_bits() as i64
    }
    fn from_raw(raw: i64) -> Option<Self> {
        i32::try_from(raw).ok().map(Self::from_bits)
    }
}
impl ReplayScalar for Fixed<i64, 32> {
    const BITS: u32 = 64;
    const FRAC: u32 = 32;
    fn raw(self) -> i64 {
        self.to_bits()
    }
    fn from_raw(raw: i64) -> Option<Self> {
        Some(Self::from_bits(raw))
    }
}

/// Structural, numeric or execution failures; failed playback cannot resume.
#[derive(Clone, Copy, Debug, PartialEq, Eq)]
pub enum ReplayError {
    /// Wrong version, truncated/trailing bytes, invalid tags/counts/input order.
    Packet,
    /// Requested scalar format does not match the seed header.
    Format,
    /// Raw scalar, configuration or resource count is outside supported limits.
    Range,
    /// A checked body/material/motion boundary rejected seed or scheduled input.
    Physics(Error),
    /// The checked triangle tree constructor rejected raw geometry.
    Tree(collision::aabb_tree::BuildError),
    /// A previous input failed; replay may only be restarted by loading its seed.
    Failed,
}

/// Concrete scheduled input. Same-tick commands execute in their file order,
/// immediately before that tick's integration. No wall clock or renderer enters.
#[derive(Clone, Copy)]
enum Input<T: ReplayScalar> {
    /// Replace linear/angular velocity and wake a reversible sleeping body.
    Velocity {
        body: BodyId,
        linear: Vector3<T>,
        angular: Vector3<T>,
    },
    /// World-space impulse/point, with ordinary body wake semantics.
    Impulse {
        body: BodyId,
        point: Vector3<T>,
        impulse: Vector3<T>,
    },
    /// Replace world gravity at this tick.
    Gravity(Vector3<T>),
}
/// File-order command identity and its zero-based, before-step tick index.
#[derive(Clone, Copy)]
struct Scheduled<T: ReplayScalar> {
    /// Simulation tick at which to apply this input.
    tick: u32,
    /// Owned typed input, with no deferred external lookup.
    input: Input<T>,
}
/// Legal playback lifecycle: completion and input failure are terminal.
#[derive(Clone, Copy)]
enum Status {
    /// Inputs and physics steps may advance until the validated tick limit.
    Running,
    /// All steps completed; subsequent step calls return None without mutation.
    Finished,
    /// An input was rejected; subsequent calls reject without retrying commands.
    Failed,
}
/// One deterministic completed state. Tick zero denotes constructor-prepared
/// initial state; subsequent ticks include input application and one full step.
#[derive(Clone, Copy, Debug, PartialEq, Eq)]
pub struct Frame {
    /// Number of completed simulation steps.
    pub tick: u32,
    /// Noncryptographic 64-bit diagnostic fingerprint.
    pub hash: u64,
}
/// Owns one raw-seeded world and the complete deterministic input timeline.
/// Static geometry is immutable; its identity is covered by the seed hash.
pub struct Replay<T: ReplayScalar> {
    /// Prepared world; no public mutable escape can bypass scheduled inputs.
    world: World<T>,
    /// Validated, nondecreasing command sequence.
    inputs: Vec<Scheduled<T>>,
    /// First command not yet applied.
    cursor: usize,
    /// Completed simulation steps.
    tick: u32,
    /// Validated total number of steps.
    ticks: u32,
    /// Identity of every seed byte, including immutable geometry and all inputs.
    seed_hash: u64,
    /// Terminal completion/failure contract.
    status: Status,
}

/// Fixed canonical byte hashing; wrapping is explicit in every build profile.
#[derive(Clone, Copy)]
struct Digest(u64);
impl Digest {
    /// Standard FNV-1a offset basis.
    fn new() -> Self {
        Self(0xcbf29ce484222325)
    }
    /// Hashes wire bytes in their specified order without native-memory casts.
    fn bytes(&mut self, bytes: &[u8]) {
        for &byte in bytes {
            self.0 = (self.0 ^ u64::from(byte)).wrapping_mul(0x100000001b3);
        }
    }
    /// All sizes, tags, IDs and counters use one portable unsigned word.
    fn word(&mut self, value: u64) {
        self.bytes(&value.to_le_bytes());
    }
    /// Every fixed component uses its exact sign-extended raw storage bits.
    fn scalar<T: ReplayScalar>(&mut self, value: T) {
        self.bytes(&value.raw().to_le_bytes());
    }
    /// Explicit XYZ component order.
    fn vector<T: ReplayScalar>(&mut self, value: Vector3<T>) {
        for x in [value.x, value.y, value.z] {
            self.scalar(x);
        }
    }
}
/// Canonical configuration field order, shared by seed encoding and hashing.
fn config_values<T: RealScalar>(c: &Config<T>) -> [T; CONFIG_SCALARS] {
    [
        c.dt,
        c.gravity.x,
        c.gravity.y,
        c.gravity.z,
        c.slop,
        c.position_rate,
        c.max_correction,
        c.match_distance,
        c.merge_distance,
        c.linear_damping,
        c.angular_damping,
        c.speed_limit,
        c.spin_limit,
        c.domain,
        c.sleep_linear,
        c.sleep_angular,
        c.warm,
        c.normal_merge,
        c.normal_match,
    ]
}
/// Reconstructs configuration only from quantized words. Every non-gravity
/// component is nonnegative; divisors, coefficients and domain are bounded.
fn config<T: ReplayScalar>(v: [T; CONFIG_SCALARS]) -> Result<Config<T>, ReplayError> {
    let one = T::one();
    let integer = |n: i64| T::from_raw(n << T::FRAC).unwrap();
    if v.iter()
        .enumerate()
        .any(|(i, x)| !(1..=3).contains(&i) && *x < T::zero())
        || v[0] <= T::zero()
        || v[0] > one
        || v[13] < one
        || v[13] > integer(24)
        || [5, 9, 10, 16, 17, 18].iter().any(|&i| v[i] > one)
        || v.iter()
            .any(|x| x.raw().unsigned_abs() > (24u64 << T::FRAC))
    {
        return Err(ReplayError::Range);
    }
    Ok(Config {
        dt: v[0],
        gravity: Vector3::new(v[1], v[2], v[3]),
        slop: v[4],
        position_rate: v[5],
        max_correction: v[6],
        match_distance: v[7],
        merge_distance: v[8],
        linear_damping: v[9],
        angular_damping: v[10],
        speed_limit: v[11],
        spin_limit: v[12],
        domain: v[13],
        sleep_linear: v[14],
        sleep_angular: v[15],
        warm: v[16],
        normal_merge: v[17],
        normal_match: v[18],
    })
}
/// Fingerprints every represented body value and both contact buffers. Tree
/// layout, sweep scratch, allocation capacities and work counters are excluded:
/// canonical candidates make them irrelevant to future simulated state. Static
/// triangles and input order are already covered by the immutable seed identity.
fn state_hash<T: ReplayScalar>(world: &World<T>, seed_hash: u64, tick: u32) -> u64 {
    let mut h = Digest::new();
    h.word(seed_hash);
    h.word(u64::from(tick));
    for value in config_values(&world.config) {
        h.scalar(value);
    }
    h.word(world.bodies.len() as u64);
    for b in &world.bodies {
        match b.shape {
            Shape::Sphere { radius } => {
                h.word(0);
                h.scalar(radius);
            }
            Shape::Capsule {
                half_height,
                radius,
            } => {
                h.word(1);
                h.scalar(half_height);
                h.scalar(radius);
            }
            Shape::Box { half } => {
                h.word(2);
                h.vector(half);
            }
        }
        h.vector(b.pose.position);
        for x in [
            b.pose.orientation.x,
            b.pose.orientation.y,
            b.pose.orientation.z,
            b.pose.orientation.w,
        ] {
            h.scalar(x);
        }
        for axis in b.axes {
            h.vector(axis);
        }
        h.vector(b.velocity);
        h.vector(b.angular);
        h.scalar(b.inverse_mass);
        h.vector(b.inverse_inertia);
        h.scalar(b.material.friction);
        h.scalar(b.material.restitution);
        match b.motion {
            Motion::Awake { quiet_ticks } => {
                h.word(0);
                h.word(u64::from(quiet_ticks));
            }
            Motion::Sleeping => h.word(1),
            Motion::Disabled => h.word(2),
        }
    }
    for buffer in [&world.contacts, &world.previous] {
        h.word(buffer.len() as u64);
        for c in buffer {
            h.word(c.a as u64);
            match c.other {
                Other::Body(id) => {
                    h.word(0);
                    h.word(id as u64);
                }
                Other::Mesh(id) => {
                    h.word(1);
                    h.word(id as u64);
                }
            }
            h.vector(c.normal);
            for t in c.tangent {
                h.vector(t);
            }
            h.vector(c.anchor_a);
            h.vector(c.anchor_b);
            h.scalar(c.depth);
            for m in c.mass {
                h.scalar(m);
            }
            h.scalar(c.bounce);
            h.scalar(c.friction);
            for impulse in c.impulse {
                h.scalar(impulse);
            }
        }
    }
    h.0
}
/// Checked forward-only decoder. Counts are validated before allocating; every
/// access rejects short input and every scalar rejects out-of-storage raw bits.
struct Reader<'a> {
    /// Unconsumed wire bytes; no raw pointers or native-layout reads.
    bytes: &'a [u8],
}
impl Reader<'_> {
    /// Consumes one fixed-size byte array or reports a truncated packet.
    fn take<const N: usize>(&mut self) -> Result<[u8; N], ReplayError> {
        let bytes = self.bytes.get(..N).ok_or(ReplayError::Packet)?;
        let result = bytes.try_into().unwrap();
        self.bytes = &self.bytes[N..];
        Ok(result)
    }
    /// Counts and tags use little-endian u32 in the seed wire format.
    fn word(&mut self) -> Result<u32, ReplayError> {
        Ok(u32::from_le_bytes(self.take()?))
    }
    /// Checks resource count and host width before allocation.
    fn count(&mut self, max: usize) -> Result<usize, ReplayError> {
        let n = usize::try_from(self.word()?).map_err(|_| ReplayError::Range)?;
        if n > max {
            Err(ReplayError::Range)
        } else {
            Ok(n)
        }
    }
    /// Scalar conversion preserves every stored bit without float arithmetic.
    fn scalar<T: ReplayScalar>(&mut self) -> Result<T, ReplayError> {
        T::from_raw(i64::from_le_bytes(self.take()?)).ok_or(ReplayError::Range)
    }
    /// XYZ storage order, shared with encoding.
    fn vector<T: ReplayScalar>(&mut self) -> Result<Vector3<T>, ReplayError> {
        Ok(Vector3::new(self.scalar()?, self.scalar()?, self.scalar()?))
    }
    /// Rejects unsupported mutation magnitudes, including the signed minimum
    /// whose wrapping fixed absolute value would remain negative.
    fn bounded_vector<T: ReplayScalar>(&mut self, limit: u64) -> Result<Vector3<T>, ReplayError> {
        let v = self.vector::<T>()?;
        if [v.x, v.y, v.z]
            .iter()
            .any(|x| x.raw().unsigned_abs() > (limit << T::FRAC))
        {
            return Err(ReplayError::Range);
        }
        Ok(v)
    }
    /// Body dimensions are checked by Body::new after decoding this typed shape.
    fn shape<T: ReplayScalar>(&mut self) -> Result<Shape<T>, ReplayError> {
        match self.word()? {
            0 => Ok(Shape::Sphere {
                radius: self.scalar()?,
            }),
            1 => Ok(Shape::Capsule {
                half_height: self.scalar()?,
                radius: self.scalar()?,
            }),
            2 => Ok(Shape::Box {
                half: self.vector()?,
            }),
            _ => Err(ReplayError::Packet),
        }
    }
    /// Surface coefficients are subsequently checked by body/mesh insertion.
    fn material<T: ReplayScalar>(&mut self) -> Result<Material<T>, ReplayError> {
        Ok(Material {
            friction: self.scalar()?,
            restitution: self.scalar()?,
        })
    }
}
impl<T: ReplayScalar> Replay<T> {
    /// Validates the version/header without committing a world. Consumers use
    /// the result to select exactly one supported backend before full decoding.
    pub fn format(bytes: &[u8]) -> Result<(u32, u32), ReplayError> {
        let mut r = Reader { bytes };
        if &r.take::<8>()? != MAGIC {
            return Err(ReplayError::Packet);
        }
        Ok((r.word()?, r.word()?))
    }
    /// Loads raw constructor inputs with fixed-point preparation, initially
    /// awake bodies and empty warm caches. Trailing bytes are an error. A failed
    /// load owns no external state and cannot leave a partially initialized world.
    pub fn decode(bytes: &[u8]) -> Result<Self, ReplayError> {
        Self::decode_with_leaf(bytes, 4)
    }
    /// The acceleration leaf size is deliberately outside simulated identity;
    /// tests vary it to verify candidate canonicalization and hash exclusions.
    fn decode_with_leaf(bytes: &[u8], leaf: usize) -> Result<Self, ReplayError> {
        let mut r = Reader { bytes };
        if &r.take::<8>()? != MAGIC {
            return Err(ReplayError::Packet);
        }
        if (r.word()?, r.word()?) != (T::BITS, T::FRAC) {
            return Err(ReplayError::Format);
        }
        let ticks = r.word()?;
        if ticks == 0 || ticks > MAX_TICKS {
            return Err(ReplayError::Range);
        }
        let bodies = r.count(128)?;
        let meshes = r.count(MAX_MESHES)?;
        let count = r.count(MAX_INPUTS)?;
        let mut values = [T::zero(); CONFIG_SCALARS];
        for v in &mut values {
            *v = r.scalar()?;
        }
        let mut world = World::<T>::new();
        world.config = config(values)?;
        for _ in 0..bodies {
            let shape = r.shape()?;
            let pose = Pose {
                position: r.bounded_vector(24)?,
                orientation: Quat::new(r.scalar()?, r.scalar()?, r.scalar()?, r.scalar()?),
            };
            let mass = r.scalar()?;
            let material = r.material()?;
            let linear = r.bounded_vector(15)?;
            let angular = r.bounded_vector(15)?;
            let mut body = Body::new(shape, pose, mass, material).map_err(ReplayError::Physics)?;
            body.set_velocity(linear, angular)
                .map_err(ReplayError::Physics)?;
            world.add_body(body).map_err(ReplayError::Physics)?;
        }
        let mut remaining = MAX_TRIANGLES;
        for _ in 0..meshes {
            let material = r.material()?;
            let n = r.count(remaining)?;
            remaining -= n;
            // Reject impossible lengths before reserving triangle storage.
            if n > r.bytes.len() / 72 {
                return Err(ReplayError::Packet);
            }
            let mut triangles = Vec::with_capacity(n);
            for _ in 0..n {
                triangles.push(Tri3::new([r.vector()?, r.vector()?, r.vector()?]));
            }
            world
                .add_mesh(StaticMesh {
                    tree: AabbTree::new(triangles, leaf).map_err(ReplayError::Tree)?,
                    material,
                })
                .map_err(ReplayError::Physics)?;
        }
        let mut inputs = Vec::with_capacity(count);
        for _ in 0..count {
            let tick = r.word()?;
            if tick >= ticks || inputs.last().is_some_and(|s: &Scheduled<T>| s.tick > tick) {
                return Err(ReplayError::Packet);
            }
            let input = match r.word()? {
                tag @ (0 | 1) => {
                    let body = BodyId(r.count(bodies.saturating_sub(1))?);
                    if bodies == 0 {
                        return Err(ReplayError::Packet);
                    }
                    let a = r.bounded_vector(if tag == 0 { 15 } else { 24 })?;
                    let b = r.bounded_vector(15)?;
                    if tag == 0 {
                        Input::Velocity {
                            body,
                            linear: a,
                            angular: b,
                        }
                    } else {
                        Input::Impulse {
                            body,
                            point: a,
                            impulse: b,
                        }
                    }
                }
                2 => Input::Gravity(r.bounded_vector(24)?),
                _ => return Err(ReplayError::Packet),
            };
            inputs.push(Scheduled { tick, input });
        }
        if !r.bytes.is_empty() {
            return Err(ReplayError::Packet);
        }
        let mut hash = Digest::new();
        hash.bytes(bytes);
        Ok(Self {
            world,
            inputs,
            cursor: 0,
            tick: 0,
            ticks,
            seed_hash: hash.0,
            status: Status::Running,
        })
    }
    /// Read-only world for diagnostics and regression coverage.
    pub fn world(&self) -> &World<T> {
        &self.world
    }
    /// Fingerprints the current state, including quiet counters, both cached
    /// impulse buffers, derived axes and all prepared mass/configuration values.
    pub fn frame(&self) -> Frame {
        Frame {
            tick: self.tick,
            hash: state_hash(&self.world, self.seed_hash, self.tick),
        }
    }
    /// Applies all same-tick commands in file order, then executes exactly one
    /// tick. Runtime input errors permanently fail playback; elapsed time never
    /// advances it and callers cannot retry a partially applied command group.
    pub fn step(&mut self) -> Result<Option<Frame>, ReplayError> {
        match self.status {
            Status::Finished => return Ok(None),
            Status::Failed => return Err(ReplayError::Failed),
            Status::Running => {}
        }
        while let Some(s) = self
            .inputs
            .get(self.cursor)
            .copied()
            .filter(|s| s.tick == self.tick)
        {
            let result = match s.input {
                Input::Velocity {
                    body,
                    linear,
                    angular,
                } => self
                    .world
                    .body_mut(body)
                    .unwrap()
                    .set_velocity(linear, angular),
                Input::Impulse {
                    body,
                    point,
                    impulse,
                } => self
                    .world
                    .body_mut(body)
                    .unwrap()
                    .apply_impulse(point, impulse),
                Input::Gravity(g) => self.world.set_gravity(g),
            };
            if let Err(e) = result {
                self.status = Status::Failed;
                return Err(ReplayError::Physics(e));
            }
            self.cursor += 1;
        }
        self.world.step();
        self.tick += 1;
        if self.tick == self.ticks {
            self.status = Status::Finished;
        }
        Ok(Some(self.frame()))
    }
}

/// Raw seed encoder, used only to record the versioned reference scenario.
#[cfg(any(feature = "std", feature = "libm", feature = "system-libm"))]
struct Writer(Vec<u8>);
#[cfg(any(feature = "std", feature = "libm", feature = "system-libm"))]
impl Writer {
    /// Writes a portable tag, count or tick.
    fn word(&mut self, n: u32) {
        self.0.extend(n.to_le_bytes());
    }
    /// Sign-extends storage to the wire's exact eight-byte scalar word.
    fn scalar<T: ReplayScalar>(&mut self, n: T) {
        self.0.extend(n.raw().to_le_bytes());
    }
    /// Writes components independently of Rust vector layout or endianness.
    fn vector<T: ReplayScalar>(&mut self, v: Vector3<T>) {
        for x in [v.x, v.y, v.z] {
            self.scalar(x);
        }
    }
    /// Fixed shape tags and dimension order, mirrored by Reader::shape.
    fn shape<T: ReplayScalar>(&mut self, s: Shape<T>) {
        match s {
            Shape::Sphere { radius } => {
                self.word(0);
                self.scalar(radius);
            }
            Shape::Capsule {
                half_height,
                radius,
            } => {
                self.word(1);
                self.scalar(half_height);
                self.scalar(radius);
            }
            Shape::Box { half } => {
                self.word(2);
                self.vector(half);
            }
        }
    }
    /// Friction then restitution, both already validated.
    fn material<T: ReplayScalar>(&mut self, m: Material<T>) {
        self.scalar(m.friction);
        self.scalar(m.restitution);
    }
}
/// Records the shared bunny/teapot scene as raw inputs, plus one guaranteed
/// sleeping/waking body and one domain-exit body. Recording may use hosted scene
/// setup; cross-target playback reads these bytes and executes fixed arithmetic.
#[cfg(any(feature = "std", feature = "libm", feature = "system-libm"))]
pub fn record_demo<T: ReplayScalar>() -> Result<Vec<u8>, String> {
    let mut world = scene::demo::<T>()?;
    let sleeper = world
        .add_body(
            Body::new(
                Shape::Sphere {
                    radius: scalar(0.42),
                },
                Pose::at(scene::vector([12., 0.42, 12.])),
                T::one(),
                scene::material(),
            )
            .map_err(|e| format!("{e:?}"))?,
        )
        .map_err(|e| format!("{e:?}"))?;
    let mut exit = Body::new(
        Shape::Sphere {
            radius: scalar(0.2),
        },
        Pose::at(scene::vector([23.95, 4., 0.])),
        T::one(),
        scene::material(),
    )
    .map_err(|e| format!("{e:?}"))?;
    exit.set_velocity(scene::vector([15., 0., 0.]), zero())
        .map_err(|e| format!("{e:?}"))?;
    world.add_body(exit).map_err(|e| format!("{e:?}"))?;
    let inputs: [Scheduled<T>; 5] = [
        Scheduled {
            tick: 320,
            input: Input::Impulse {
                body: sleeper,
                point: scene::vector([12., 0.42, 12.]),
                impulse: scene::vector([-1., 2., 0.5]),
            },
        },
        Scheduled {
            tick: 640,
            input: Input::Velocity {
                body: sleeper,
                linear: scene::vector([1., 0.5, -1.]),
                angular: scene::vector([0., 0.5, 0.]),
            },
        },
        Scheduled {
            tick: 640,
            input: Input::Impulse {
                body: BodyId(1),
                point: scene::vector([-5., 1., 0.]),
                impulse: scene::vector([0.1, 1., 0.]),
            },
        },
        Scheduled {
            tick: 900,
            input: Input::Gravity(scene::vector([0., 6., 0.])),
        },
        Scheduled {
            tick: 1100,
            input: Input::Gravity(scene::vector([0., -9.81, 0.])),
        },
    ];
    let mut w = Writer(MAGIC.to_vec());
    w.word(T::BITS);
    w.word(T::FRAC);
    w.word(1440);
    w.word(world.bodies.len() as u32);
    w.word(world.meshes.len() as u32);
    w.word(inputs.len() as u32);
    for v in config_values(&world.config) {
        w.scalar(v);
    }
    for body in &world.bodies {
        // This reference scene's constructor mass is exactly one; seed files
        // encode constructor inputs rather than reverse-engineering inertia.
        if body.inverse_mass != T::one() {
            return Err("reference scene requires unit masses".into());
        }
        w.shape(body.shape);
        w.vector(body.pose.position);
        for q in [
            body.pose.orientation.x,
            body.pose.orientation.y,
            body.pose.orientation.z,
            body.pose.orientation.w,
        ] {
            w.scalar(q);
        }
        w.scalar(T::one());
        w.material(body.material);
        w.vector(body.velocity);
        w.vector(body.angular);
    }
    for mesh in &world.meshes {
        w.material(mesh.material);
        w.word(mesh.tree.info().triangles as u32);
        for id in 0..mesh.tree.info().triangles {
            for &p in mesh.tree.triangle(id).unwrap().vertices() {
                w.vector(p);
            }
        }
    }
    for s in inputs {
        w.word(s.tick);
        match s.input {
            Input::Velocity {
                body,
                linear,
                angular,
            } => {
                w.word(0);
                w.word(body.0 as u32);
                w.vector(linear);
                w.vector(angular);
            }
            Input::Impulse {
                body,
                point,
                impulse,
            } => {
                w.word(1);
                w.word(body.0 as u32);
                w.vector(point);
                w.vector(impulse);
            }
            Input::Gravity(g) => {
                w.word(2);
                w.vector(g);
            }
        }
    }
    Ok(w.0)
}
