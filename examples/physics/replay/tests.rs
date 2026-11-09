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

//! Raw packet boundaries, complete state hashing and checked golden trajectories.
use super::*;

/// Generated on the reference host; playback never regenerates its expectation.
const SEED32: &[u8] = include_bytes!(concat!(
    env!("CARGO_MANIFEST_DIR"),
    "/tests/fixtures/physics-fixed32.bin"
));
/// Identical wire schema, independent S31.32 quantization and reference trajectory.
const SEED64: &[u8] = include_bytes!(concat!(
    env!("CARGO_MANIFEST_DIR"),
    "/tests/fixtures/physics-fixed64.bin"
));
/// One hash per completed tick, including all represented initial state.
const TRACE32: &str = include_str!(concat!(
    env!("CARGO_MANIFEST_DIR"),
    "/tests/fixtures/physics-fixed32.hashes"
));
/// Separate format-specific trace; different formats need not agree numerically.
const TRACE64: &str = include_str!(concat!(
    env!("CARGO_MANIFEST_DIR"),
    "/tests/fixtures/physics-fixed64.hashes"
));

/// Header plus raw configuration precedes the first sphere constructor record.
/// Body offsets below are relative to that record, independent of config length.
const FIRST_BODY: usize = 32 + CONFIG_SCALARS * 8;

/// Confirms every frame, and independently requires the scenario to exercise
/// quiet/sleep/disabled states plus current and previous accumulated impulses.
fn golden<T: ReplayScalar>(seed: &[u8], trace: &str) {
    let mut replay = Replay::<T>::decode(seed).unwrap();
    let mut frame = Some(replay.frame());
    let (mut quiet, mut sleeping, mut disabled, mut warm, mut previous) =
        (false, false, false, false, false);
    let mut count = 0;
    for line in trace.lines() {
        let mut cols = line.split_whitespace();
        let expected = Frame {
            tick: cols.next().unwrap().parse().unwrap(),
            hash: u64::from_str_radix(cols.next().unwrap(), 16).unwrap(),
        };
        assert!(cols.next().is_none());
        assert_eq!(frame, Some(expected));
        for body in &replay.world.bodies {
            match body.motion {
                Motion::Awake { quiet_ticks } => quiet |= quiet_ticks > 0,
                Motion::Sleeping => sleeping = true,
                Motion::Disabled => disabled = true,
            }
        }
        let nonzero = |c: &Constraint<T>| c.impulse.iter().any(|x| *x != T::zero());
        warm |= replay.world.contacts.iter().any(nonzero);
        previous |= replay.world.previous.iter().any(nonzero);
        frame = replay.step().unwrap();
        count += 1;
    }
    assert_eq!(count, 1441);
    assert!(frame.is_none());
    assert!(replay.step().unwrap().is_none());
    assert!(
        quiet && sleeping && disabled && warm && previous,
        "missing scenario coverage: {:?}",
        (quiet, sleeping, disabled, warm, previous)
    );
}
#[test]
fn fixed32_matches_every_reference_tick() {
    golden::<Fixed<i32, 16>>(SEED32, TRACE32);
}
#[test]
fn fixed64_matches_every_reference_tick() {
    golden::<Fixed<i64, 32>>(SEED64, TRACE64);
}

/// Version 02's tick-900 upward force invalidates the actual settled states,
/// rather than merely proving that some sleeper and some gravity input occurred.
#[test]
fn gravity_command_wakes_reference_sleepers() {
    fn check<T: ReplayScalar>(seed: &[u8]) {
        let mut r = Replay::<T>::decode(seed).unwrap();
        for _ in 0..900 {
            r.step().unwrap();
        }
        let sleepers: Vec<_> = r
            .world
            .bodies
            .iter()
            .enumerate()
            .filter_map(|(i, b)| b.sleeping().then_some(i))
            .collect();
        assert!(!sleepers.is_empty());
        assert_eq!(r.step().unwrap().unwrap().tick, 901);
        for id in sleepers {
            assert!(!r.world.bodies[id].sleeping());
        }
    }
    check::<Fixed<i32, 16>>(SEED32);
    check::<Fixed<i64, 32>>(SEED64);
}

/// Different tree topologies/work counts must produce the same simulated state.
#[test]
fn leaf_sizes_do_not_change_the_replay() {
    let mut a = Replay::<Fixed<i32, 16>>::decode_with_leaf(SEED32, 1).unwrap();
    let mut b = Replay::<Fixed<i32, 16>>::decode_with_leaf(SEED32, 17).unwrap();
    assert_eq!(a.frame(), b.frame());
    for _ in 0..480 {
        assert_eq!(a.step().unwrap(), b.step().unwrap());
    }
    assert_ne!(a.world.stats.bounds_tests, b.world.stats.bounds_tests);
}

/// Hashing must observe hidden state that affects future ticks, even when the
/// visible pose is unchanged. Reversible test mutations isolate each field.
#[test]
fn fingerprints_cover_motion_and_solver_cache() {
    type T = Fixed<i32, 16>;
    let mut r = Replay::<T>::decode(SEED32).unwrap();
    for _ in 0..240 {
        r.step().unwrap();
    }
    let baseline = r.frame();
    let motion = r.world.bodies[0].motion;
    r.world.bodies[0].motion = Motion::Awake { quiet_ticks: 314 };
    assert_ne!(baseline, r.frame());
    r.world.bodies[0].motion = Motion::Sleeping;
    assert_ne!(baseline, r.frame());
    r.world.bodies[0].motion = Motion::Disabled;
    assert_ne!(baseline, r.frame());
    r.world.bodies[0].motion = motion;
    assert_eq!(baseline, r.frame());
    let q = r.world.bodies[0].pose.orientation.w;
    r.world.bodies[0].pose.orientation.w = q + T::DELTA;
    assert_ne!(baseline, r.frame());
    r.world.bodies[0].pose.orientation.w = q;
    let axis = r.world.bodies[0].axes[0].x;
    r.world.bodies[0].axes[0].x = axis + T::DELTA;
    assert_ne!(baseline, r.frame());
    r.world.bodies[0].axes[0].x = axis;
    let inertia = r.world.bodies[0].inverse_inertia.x;
    r.world.bodies[0].inverse_inertia.x = inertia + T::DELTA;
    assert_ne!(baseline, r.frame());
    r.world.bodies[0].inverse_inertia.x = inertia;
    let gravity = r.world.config.gravity.y;
    r.world.config.gravity.y = gravity + T::DELTA;
    assert_ne!(baseline, r.frame());
    r.world.config.gravity.y = gravity;
    for previous in [false, true] {
        for channel in 0..3 {
            let buffer = if previous {
                &mut r.world.previous
            } else {
                &mut r.world.contacts
            };
            let old = buffer[0].impulse[channel];
            buffer[0].impulse[channel] = old + T::DELTA;
            assert_ne!(baseline, r.frame());
            let buffer = if previous {
                &mut r.world.previous
            } else {
                &mut r.world.contacts
            };
            buffer[0].impulse[channel] = old;
        }
    }
    assert_eq!(baseline, r.frame());
    r.world.order.reverse();
    r.world.bounds.clear();
    r.world.shapes.clear();
    r.world.stats.bounds_tests += 10;
    assert_eq!(
        baseline,
        r.frame(),
        "acceleration/scratch is outside state identity"
    );
}

/// Malformed raw storage must fail at a checked boundary, before unchecked ticks.
#[test]
fn malformed_seeds_are_rejected() {
    type T = Fixed<i32, 16>;
    for end in (0..FIRST_BODY).chain(SEED32.len() - 100..SEED32.len()) {
        assert!(
            Replay::<T>::decode(&SEED32[..end]).is_err(),
            "prefix {}",
            end
        );
    }
    let rejects = |offset: usize, bytes: &[u8]| {
        let mut seed = SEED32.to_vec();
        seed[offset..offset + bytes.len()].copy_from_slice(bytes);
        assert!(
            Replay::<T>::decode(&seed).is_err(),
            "mutation at {}",
            offset
        );
    };
    rejects(0, b"badmagic");
    rejects(7, b"1"); // retired solver/schema version is not silently replayed
    rejects(8, &64u32.to_le_bytes());
    rejects(12, &8u32.to_le_bytes());
    for (offset, value) in [
        (16, 0u32),
        (16, MAX_TICKS + 1),
        (20, 129),
        (24, 17),
        (28, 4097),
    ] {
        rejects(offset, &value.to_le_bytes());
    }
    rejects(32, &i64::MAX.to_le_bytes()); // nonrepresentable raw S15.16 configuration
    rejects(FIRST_BODY, &9u32.to_le_bytes()); // invalid shape tag
    rejects(FIRST_BODY + 4, &0i64.to_le_bytes()); // zero sphere radius
    rejects(FIRST_BODY + 36, &[0; 32]); // zero quaternion
    rejects(FIRST_BODY + 68, &0i64.to_le_bytes()); // zero mass
    rejects(FIRST_BODY + 92, &i64::MIN.to_le_bytes()); // invalid initial velocity
    let commands = SEED32.len() - 244;
    rejects(commands + 4, &99u32.to_le_bytes());
    rejects(commands + 8, &128u32.to_le_bytes());
    rejects(commands + 12, &i64::MIN.to_le_bytes()); // invalid impulse point
    rejects(commands + 60, &0u32.to_le_bytes()); // decreasing command ticks
    rejects(commands, &1440u32.to_le_bytes()); // input after final step
    let mut trailing = SEED32.to_vec();
    trailing.push(0);
    assert!(Replay::<T>::decode(&trailing).is_err());
    assert!(matches!(
        Replay::<T>::decode(SEED64),
        Err(ReplayError::Format)
    ));
}

/// S31.32 accepts every i64 as storage, so vector bounds must explicitly reject
/// the signed minimum rather than relying on narrowing or wrapping absolute value.
#[test]
fn fixed64_signed_minimum_vectors_are_rejected() {
    for offset in [
        40,
        FIRST_BODY + 12,
        FIRST_BODY + 92,
        SEED64.len() - 244 + 12,
        SEED64.len() - 24,
    ] {
        let mut seed = SEED64.to_vec();
        seed[offset..offset + 8].copy_from_slice(&i64::MIN.to_le_bytes());
        assert!(matches!(
            Replay::<Fixed<i64, 32>>::decode(&seed),
            Err(ReplayError::Range)
        ));
    }
}

/// A runtime command can target a body that has since left the domain. Failure
/// is terminal, so callers cannot repeat a partially applied same-tick group.
#[test]
fn runtime_input_failure_is_terminal() {
    type T = Fixed<i32, 16>;
    let mut r = Replay::<T>::decode(SEED32).unwrap();
    r.inputs = vec![Scheduled {
        tick: 1,
        input: Input::Velocity {
            body: BodyId(25),
            linear: Vector3::zero(),
            angular: Vector3::zero(),
        },
    }];
    r.step().unwrap();
    assert!(!r.world.bodies[25].enabled());
    let before = r.frame();
    assert_eq!(r.step(), Err(ReplayError::Physics(Error::InactiveBody)));
    assert_eq!(r.step(), Err(ReplayError::Failed));
    assert_eq!(before, r.frame());
}

/// Same-tick command order is part of the seed's contract; it is never sorted by
/// opcode or body, unlike broad-phase candidates which have no input semantics.
#[test]
fn same_tick_inputs_preserve_file_order() {
    type T = Fixed<i32, 16>;
    let mut a = Replay::<T>::decode(SEED32).unwrap();
    let mut b = Replay::<T>::decode(SEED32).unwrap();
    let body = BodyId(24);
    let point = a.world.bodies[24].pose.position;
    a.inputs = vec![
        Scheduled {
            tick: 0,
            input: Input::Velocity {
                body,
                linear: Vector3::zero(),
                angular: Vector3::zero(),
            },
        },
        Scheduled {
            tick: 0,
            input: Input::Impulse {
                body,
                point,
                impulse: Vector3::new(T::ONE, T::ZERO, T::ZERO),
            },
        },
    ];
    b.inputs = a.inputs.iter().rev().copied().collect();
    a.step().unwrap();
    b.step().unwrap();
    assert_ne!(a.world.bodies[24].velocity.x, b.world.bodies[24].velocity.x);
}
