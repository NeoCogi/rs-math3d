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

//! Measures the actual World::step, with immutable soups prepared once.
//! Native runs retain each scalar's trajectory. Shared-state runs restore one
//! recorded S15.16 pre-step state, dequantized for f32, before every full tick.
//! Restoration/conversion and validation are untimed; only shared-state runs
//! include a clock interval per step. Canonical order and warm state are retained.
use super::*;
use rs_math3d::{Fixed, Tri3};
use std::{
    hint::black_box,
    time::{Duration, Instant},
};

/// Authoritative shared trace scalar; this is not a run-time arithmetic switch.
type F = Fixed<i32, 16>;
/// Rejects any trace component that f32 cannot represent exactly. This check is
/// untimed and ensures the shared comparison really starts from the same values.
fn dequantize(value: F) -> f32 {
    let result = value.to_f32();
    assert_eq!(
        f64::from(result),
        value.to_f64(),
        "shared-state component is not exactly representable by f32"
    );
    result
}
/// Component mapping preserves stored state without normalization/repreparation.
fn vector<T: RealScalar, U: RealScalar>(v: Vector3<T>, cast: impl Fn(T) -> U + Copy) -> Vector3<U> {
    Vector3::new(cast(v.x), cast(v.y), cast(v.z))
}
/// Maps local collider dimensions, retaining the exact primitive variant.
fn shape<T: RealScalar, U: RealScalar>(s: Shape<T>, cast: impl Fn(T) -> U + Copy) -> Shape<U> {
    match s {
        Shape::Sphere { radius } => Shape::Sphere {
            radius: cast(radius),
        },
        Shape::Capsule {
            half_height,
            radius,
        } => Shape::Capsule {
            half_height: cast(half_height),
            radius: cast(radius),
        },
        Shape::Box { half } => Shape::Box {
            half: vector(half, cast),
        },
    }
}
/// Maps validated surface coefficients without changing their mixing policy.
fn material<T: RealScalar, U: RealScalar>(
    m: Material<T>,
    cast: impl Fn(T) -> U + Copy,
) -> Material<U> {
    Material {
        friction: cast(m.friction),
        restitution: cast(m.restitution),
    }
}
/// Copies prepared values, including inverse inertia, axes and sleep counters.
/// Calling Body::new here would normalize/recompute a different starting state.
fn body<T: RealScalar, U: RealScalar>(b: &Body<T>, cast: impl Fn(T) -> U + Copy) -> Body<U> {
    Body {
        shape: shape(b.shape, cast),
        pose: Pose {
            position: vector(b.pose.position, cast),
            orientation: Quat::new(
                cast(b.pose.orientation.x),
                cast(b.pose.orientation.y),
                cast(b.pose.orientation.z),
                cast(b.pose.orientation.w),
            ),
        },
        axes: b.axes.map(|v| vector(v, cast)),
        velocity: vector(b.velocity, cast),
        angular: vector(b.angular, cast),
        inverse_mass: cast(b.inverse_mass),
        inverse_inertia: vector(b.inverse_inertia, cast),
        material: material(b.material, cast),
        motion: b.motion,
    }
}
/// Copies all numerical configuration, including the represented fixed timestep.
fn config<T: RealScalar, U: RealScalar>(c: &Config<T>, f: impl Fn(T) -> U + Copy) -> Config<U> {
    Config {
        dt: f(c.dt),
        gravity: vector(c.gravity, f),
        slop: f(c.slop),
        position_rate: f(c.position_rate),
        max_correction: f(c.max_correction),
        match_distance: f(c.match_distance),
        merge_distance: f(c.merge_distance),
        linear_damping: f(c.linear_damping),
        angular_damping: f(c.angular_damping),
        speed_limit: f(c.speed_limit),
        spin_limit: f(c.spin_limit),
        domain: f(c.domain),
        sleep_linear: f(c.sleep_linear),
        sleep_angular: f(c.sleep_angular),
        warm: f(c.warm),
        normal_merge: f(c.normal_merge),
        normal_match: f(c.normal_match),
    }
}
/// Copies both contact buffers without changing warm-start anchors or impulses.
fn constraint<T: RealScalar, U: RealScalar>(
    c: &Constraint<T>,
    f: impl Fn(T) -> U + Copy,
) -> Constraint<U> {
    Constraint {
        a: c.a,
        other: c.other,
        normal: vector(c.normal, f),
        tangent: c.tangent.map(|v| vector(v, f)),
        anchor_a: vector(c.anchor_a, f),
        anchor_b: vector(c.anchor_b, f),
        depth: f(c.depth),
        mass: c.mass.map(f),
        bounce: f(c.bounce),
        friction: f(c.friction),
        impulse: c.impulse.map(f),
    }
}
/// Owned pre-step state. Immutable geometry and rebuilt broad-phase scratch are
/// excluded; no benchmark state is exposed through the engine's public API.
struct Frame<T: RealScalar> {
    /// Stable body order and all prepared/mutable per-body values.
    bodies: Vec<Body<T>>,
    /// Previous coherent sweep order, to preserve insertion-sort work.
    order: Vec<usize>,
    /// Current contact/warm cache, before the next swap.
    contacts: Vec<Constraint<T>>,
    /// Other cache buffer, including the same initialized prefix.
    previous: Vec<Constraint<T>>,
    /// Preconverted configuration, preserving dt/tolerances/damping across types.
    config: Config<T>,
}
impl<T: RealScalar + TryCastFrom<f64>> Frame<T> {
    /// Takes a typed copy or numeric mapping of the complete relevant state.
    fn capture<S: RealScalar + TryCastFrom<f64>>(
        world: &World<S>,
        cast: impl Fn(S) -> T + Copy,
    ) -> Self {
        Self {
            bodies: world.bodies.iter().map(|b| body(b, cast)).collect(),
            order: world.order.clone(),
            contacts: world.contacts.iter().map(|c| constraint(c, cast)).collect(),
            previous: world.previous.iter().map(|c| constraint(c, cast)).collect(),
            config: config(&world.config, cast),
        }
    }
    /// Reuses allocated scratch/caches. Restoration is always outside timing;
    /// shapes/bounds/candidates/stats are rebuilt by the following World::step.
    fn restore(&self, world: &mut World<T>) {
        world.bodies.clear();
        world
            .bodies
            .extend(self.bodies.iter().map(|b| body(b, |x| x)));
        world.order.clear();
        world.order.extend_from_slice(&self.order);
        world.contacts.clear();
        world.contacts.extend_from_slice(&self.contacts);
        world.previous.clear();
        world.previous.extend_from_slice(&self.previous);
        world.config = config(&self.config, |x| x);
    }
}
/// Builds immutable trees from the same represented input triangle IDs once.
/// Source geometry/scales are the existing bounded demo's, not arbitrary input.
fn mapped_world<S: RealScalar + TryCastFrom<f64>, T: RealScalar + TryCastFrom<f64>>(
    source: &World<S>,
    cast: impl Fn(S) -> T + Copy,
) -> Result<World<T>, String> {
    let mut world = World::new();
    for mesh in &source.meshes {
        let triangles = (0..mesh.tree.info().triangles)
            .map(|i| {
                Tri3::new(
                    mesh.tree
                        .triangle(i)
                        .unwrap()
                        .vertices()
                        .map(|p| vector(p, cast)),
                )
            })
            .collect();
        world
            .add_mesh(StaticMesh {
                tree: AabbTree::new(triangles, 4).map_err(|e| format!("mapped tree: {e:?}"))?,
                material: material(mesh.material, cast),
            })
            .map_err(|e| format!("mapped material: {e:?}"))?;
    }
    Frame::capture(source, cast).restore(&mut world);
    Ok(world)
}
/// Measured work over an entire sample. Counts are semantic work, not timings;
/// divergent native trajectories must not be presented as identical workloads.
#[derive(Clone, Copy, Default, Debug, PartialEq, Eq)]
struct Work {
    /// Dynamic broad-phase pairs.
    pairs: u64,
    /// Static triangle candidates.
    triangles: u64,
    /// Static bound tests.
    bounds: u64,
    /// Reduced solver contacts.
    contacts: u64,
    /// Total awake-body ticks.
    awake: u64,
    /// Total sleeping-body ticks.
    sleeping: u64,
    /// Total disabled-body ticks.
    disabled: u64,
}
impl Work {
    /// Same counter bookkeeping is timed for both backends and workloads.
    fn add(&mut self, s: Stats) {
        self.pairs += s.body_pairs as u64;
        self.triangles += s.triangle_candidates as u64;
        self.bounds += s.bounds_tests as u64;
        self.contacts += s.contacts as u64;
        self.awake += s.awake as u64;
        self.sleeping += s.sleeping as u64;
        self.disabled += s.disabled as u64;
    }
}
/// Finite-state validation and repeated-sample end diagnostics, both untimed.
/// S15.16/f32 values convert exactly to f64 for these output comparisons.
fn end_values<T: RealScalar + TryCastFrom<f64>>(world: &World<T>) -> Result<Vec<[f64; 13]>, String>
where
    f64: TryCastFrom<T>,
{
    world
        .bodies
        .iter()
        .map(|b| {
            let q = b.pose.orientation;
            let p = b.pose.position;
            let v = b.velocity;
            let a = b.angular;
            let values = [
                p.x, p.y, p.z, q.x, q.y, q.z, q.w, v.x, v.y, v.z, a.x, a.y, a.z,
            ];
            if values.iter().any(|x| !x.is_finite()) {
                return Err("nonfinite simulation state".into());
            }
            let mut converted = [0.; 13];
            for (o, x) in converted.iter_mut().zip(values) {
                *o = f64::try_cast_from(x).map_err(|_| "end conversion")?;
            }
            Ok(converted)
        })
        .collect()
}
/// One measurement keeps wall time, work and output diagnostics separate.
struct Sample {
    /// Total measured time divided by actual complete steps.
    ns_per_tick: f64,
    /// Work counters, checked for repeatability within a workload/backend.
    work: Work,
    /// Final represented bodies, for same-backend repeatability checks.
    end: Vec<[f64; 13]>,
}
/// Full native trajectory, reset from identical prepared initial state each batch.
struct Native<T: RealScalar + TryCastFrom<f64>> {
    /// Reuses trees and peak scratch allocation across batches.
    world: World<T>,
    /// Backend-native setup and configuration, including its native dt.
    initial: Frame<T>,
}
impl<T: RealScalar + TryCastFrom<f64>> Native<T>
where
    f64: TryCastFrom<T>,
{
    /// Builds the ordinary scene and warms the entire trajectory before timing.
    fn new(ticks: usize) -> Result<Self, String> {
        let mut world = scene::demo::<T>()?;
        let initial = Frame::capture(&world, |x| x);
        for _ in 0..ticks {
            world.step();
        }
        end_values(&world)?;
        Ok(Self { world, initial })
    }
    /// Times complete contiguous trajectories. Reset, validation and rendering
    /// are excluded; integration, both collision phases and all solver passes run.
    fn measure(&mut self, ticks: usize, batches: usize) -> Result<Sample, String> {
        let mut elapsed = Duration::ZERO;
        let mut work = Work::default();
        for _ in 0..batches {
            self.initial.restore(&mut self.world);
            let world = black_box(&mut self.world);
            let start = Instant::now();
            for _ in 0..ticks {
                world.step();
                work.add(world.stats());
            }
            elapsed += start.elapsed();
            black_box(&self.world);
        }
        Ok(Sample {
            ns_per_tick: elapsed.as_secs_f64() * 1e9 / (ticks * batches) as f64,
            work,
            end: end_values(&self.world)?,
        })
    }
}
/// Complete steps from shared pre-step states, without accumulating cross-scalar
/// trajectory divergence. One clock interval per tick is included, not subtracted.
fn shared<T: RealScalar + TryCastFrom<f64>>(
    world: &mut World<T>,
    frames: &[Frame<T>],
    batches: usize,
) -> Result<Sample, String>
where
    f64: TryCastFrom<T>,
{
    let mut elapsed = Duration::ZERO;
    let mut work = Work::default();
    for _ in 0..batches {
        for frame in frames {
            frame.restore(world);
            let world = black_box(&mut *world);
            let start = Instant::now();
            world.step();
            work.add(world.stats());
            elapsed += start.elapsed();
            black_box(&*world);
        }
    }
    Ok(Sample {
        ns_per_tick: elapsed.as_secs_f64() * 1e9 / (frames.len() * batches) as f64,
        work,
        end: end_values(world)?,
    })
}
/// Verifies repeated inputs produce identical outputs/work before reporting time.
/// Samples are retained individually so medians/ranges do not hide noisy runs.
fn report(label: &str, samples: &[Sample], steps: usize) -> f64 {
    for sample in samples {
        assert_eq!(sample.work, samples[0].work);
        assert_eq!(sample.end, samples[0].end);
    }
    let mut times: Vec<_> = samples.iter().map(|s| s.ns_per_tick).collect();
    times.sort_by(f64::total_cmp);
    let n = times.len();
    let median = if n % 2 == 0 {
        (times[n / 2 - 1] + times[n / 2]) * 0.5
    } else {
        times[n / 2]
    };
    println!(
        "{label}: median={:.3} us/tick min={:.3} max={:.3}",
        median / 1000.,
        times[0] / 1000.,
        times[n - 1] / 1000.
    );
    println!(
        "  samples_us={:?}",
        samples
            .iter()
            .map(|s| s.ns_per_tick / 1000.)
            .collect::<Vec<_>>()
    );
    let w = samples[0].work;
    let d = steps as f64;
    println!("  per_tick pairs={:.3} triangles={:.3} bounds={:.3} contacts={:.3} awake={:.3} sleeping={:.3} disabled={:.3}",
        w.pairs as f64/d,w.triangles as f64/d,w.bounds as f64/d,w.contacts as f64/d,w.awake as f64/d,w.sleeping as f64/d,w.disabled as f64/d);
    println!(
        "  end_position_sum={:.9}",
        samples[0]
            .end
            .iter()
            .map(|v| v[0] + v[1] + v[2])
            .sum::<f64>()
    );
    median
}
/// Runs paired, alternately ordered samples for both complete-step workloads.
/// All frame conversion, construction, reset, warm-up and validation is untimed.
pub fn run(ticks: usize, batches: usize, samples: usize) -> Result<(), String> {
    let steps = ticks.checked_mul(batches).ok_or("step count overflow")?;
    if steps == 0 || samples == 0 {
        return Err("ticks, batches and samples must be positive".into());
    }
    println!("Full physics: 24 bodies, 6320 teapot + 3851 bunny + 128 floor triangles");
    println!("ticks={ticks} batches={batches} samples={samples}; warm scratch; alternating scalar order; setup/reset/validation excluded");
    let mut native_float = Native::<f32>::new(ticks)?;
    let mut native_fixed = Native::<F>::new(ticks)?;
    let mut source = scene::demo::<F>()?;
    let mut shared_fixed = mapped_world(&source, |x| x)?;
    let mut shared_float = mapped_world(&source, dequantize)?;
    let mut fixed_frames = Vec::with_capacity(ticks);
    let mut float_frames = Vec::with_capacity(ticks);
    for _ in 0..ticks {
        fixed_frames.push(Frame::capture(&source, |x| x));
        float_frames.push(Frame::capture(&source, dequantize));
        source.step();
    }
    // Warm all shared states/caches once, and validate outside measurements.
    for (a, b) in fixed_frames.iter().zip(&float_frames) {
        a.restore(&mut shared_fixed);
        shared_fixed.step();
        b.restore(&mut shared_float);
        shared_float.step();
    }
    end_values(&shared_fixed)?;
    end_values(&shared_float)?;
    let (mut nf, mut ni, mut sf, mut si) = (Vec::new(), Vec::new(), Vec::new(), Vec::new());
    for i in 0..samples {
        if i % 2 == 0 {
            nf.push(native_float.measure(ticks, batches)?);
            ni.push(native_fixed.measure(ticks, batches)?);
            sf.push(shared(&mut shared_float, &float_frames, batches)?);
            si.push(shared(&mut shared_fixed, &fixed_frames, batches)?);
        } else {
            ni.push(native_fixed.measure(ticks, batches)?);
            nf.push(native_float.measure(ticks, batches)?);
            si.push(shared(&mut shared_fixed, &fixed_frames, batches)?);
            sf.push(shared(&mut shared_float, &float_frames, batches)?);
        }
    }
    let a = report("native f32", &nf, steps);
    let b = report("native S15.16", &ni, steps);
    let c = report("shared-state f32", &sf, steps);
    let d = report("shared-state S15.16", &si, steps);
    println!(
        "S15.16/f32 time ratio: native={:.3} shared-state={:.3}",
        b / a,
        d / c
    );
    let mut overhead = Duration::ZERO;
    for _ in 0..20000 {
        let t = Instant::now();
        black_box(());
        overhead += t.elapsed();
    }
    println!(
        "empty per-step clock interval average={:.2} ns (included in shared-state times)",
        overhead.as_secs_f64() * 1e9 / 20000.
    );
    Ok(())
}

#[cfg(test)]
mod tests {
    use super::*;
    /// Actual uninterrupted execution is the reference for restored simulation,
    /// including both impulse buffers, orientation axes, counters and config.
    #[test]
    fn restoring_frames_preserves_complete_fixed_step() {
        let mut original = scene::demo::<F>().unwrap();
        let mut restored = mapped_world(&original, |x| x).unwrap();
        for tick in 0..1200 {
            Frame::capture(&original, |x| x).restore(&mut restored);
            original.step();
            restored.step();
            assert_eq!(
                replay::state_hash(&original, 0, tick),
                replay::state_hash(&restored, 0, tick)
            );
        }
    }
    /// The comparison shares represented dt, prepared inertia and cached axes;
    /// it must not silently create a new f32 constructor/normalization policy.
    #[test]
    fn shared_float_frame_preserves_quantized_preparation() {
        let mut original = scene::demo::<F>().unwrap();
        for _ in 0..240 {
            original.step();
        }
        let mapped = mapped_world(&original, dequantize).unwrap();
        assert_eq!(mapped.config.dt, original.config.dt.to_f32());
        for (a, b) in original.bodies.iter().zip(&mapped.bodies) {
            assert_eq!(a.inverse_inertia.x.to_f32(), b.inverse_inertia.x);
            assert_eq!(a.pose.orientation.w.to_f32(), b.pose.orientation.w);
            assert_eq!(a.axes[0].x.to_f32(), b.axes[0].x);
        }
        assert_eq!(mapped.contacts.len(), original.contacts.len());
        for (a, b) in original.contacts.iter().zip(&mapped.contacts) {
            assert_eq!(a.impulse.map(F::to_f32), b.impulse);
        }
    }
}
