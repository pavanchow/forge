//! Gate 5: rollback netcode reproduction.
//!
//! Rollback only works on a deterministic engine. The property proven here:
//! rolling back to a recorded tick and replaying forward with the SAME inputs
//! reproduces the original final hash exactly, while replaying with DIFFERENT
//! inputs diverges, and the divergence itself is deterministic.
//!
//! `FORGE_FUZZ_OPS` above 40 scales the gate to hundreds of entities and
//! thousands of ticks.

use forge::math::Vec2;
use forge::rollback::{replay_to, SnapshotRing};
use forge::sim::{Command, ScriptEntry, SimConfig, Simulation};

fn fuzz_ops() -> u64 {
    std::env::var("FORGE_FUZZ_OPS")
        .ok()
        .and_then(|v| v.parse().ok())
        .unwrap_or(40)
}

fn build(seed: u64, balls: u32, steps: u64) -> (Simulation, Vec<ScriptEntry>) {
    let config = SimConfig::default();
    let mut sim = Simulation::new(config, seed);
    sim.seed_scene(config, balls);
    let script: Vec<ScriptEntry> = vec![
        (
            steps / 4,
            Command::Impulse {
                entity: sim.world.entities_with::<forge::components::Velocity>()[0],
                delta_v: Vec2::new(45.0, 25.0),
            },
        ),
        (steps / 2, Command::SetGravity(Vec2::new(0.0, 30.0))),
        (
            steps / 2 + 1,
            Command::SpawnBall {
                pos: Vec2::new(10.0, 90.0),
                vel: Vec2::new(-80.0, -20.0),
                radius: 1.5,
                mass: 2.0,
                restitution: 0.7,
            },
        ),
    ];
    (sim, script)
}

fn run_to_tick_recording(
    seed: u64,
    balls: u32,
    steps: u64,
    record_every: u64,
    ring: &mut SnapshotRing,
) -> u64 {
    let (mut sim, script) = build(seed, balls, steps);
    loop {
        let next_tick = sim.tick + record_every;
        replay_to(&mut sim, next_tick, &script);
        ring.record(&sim);
        if sim.tick >= steps {
            break;
        }
    }
    sim.hash()
}

#[test]
fn rollback_replay_same_inputs_reproduce_original_hash() {
    let ops = fuzz_ops();
    let (balls, steps, record_every) = if ops <= 40 {
        (30, 400, 10)
    } else {
        (250, 3_000, 25)
    };
    let mut ring = SnapshotRing::new(512);
    let original = run_to_tick_recording(123, balls, steps, record_every, &mut ring);

    let rollback_tick = (steps / 2) / record_every * record_every;
    assert!(ring.can_rollback(rollback_tick), "mid-run tick must be retained");

    // Replay from the rollback point with identical inputs: exact reproduction,
    // and repeatable.
    let (_, script) = build(123, balls, steps);
    let first = {
        let mut sim = ring.rollback(rollback_tick).unwrap();
        replay_to(&mut sim, steps, &script);
        sim.hash()
    };
    let second = {
        let mut sim = ring.rollback(rollback_tick).unwrap();
        replay_to(&mut sim, steps, &script);
        sim.hash()
    };
    assert_eq!(first, second, "same-input replay must be repeatable");
    assert_eq!(first, original, "same-input replay must reproduce the original hash");
}

#[test]
fn rollback_replay_divergent_inputs_diverge_deterministically() {
    let ops = fuzz_ops();
    let (balls, steps, record_every) = if ops <= 40 {
        (30, 400, 10)
    } else {
        (250, 3_000, 25)
    };
    let mut ring = SnapshotRing::new(512);
    let original = run_to_tick_recording(123, balls, steps, record_every, &mut ring);
    let (_, script) = build(123, balls, steps);

    let rollback_tick = (steps / 2) / record_every * record_every;
    let mut divergent_script = script.clone();
    divergent_script.push((
        rollback_tick + 10,
        Command::Impulse {
            entity: ring
                .rollback(rollback_tick)
                .unwrap()
                .world
                .entities_with::<forge::components::Velocity>()[1],
            delta_v: Vec2::new(-120.0, 60.0),
        },
    ));

    let run_divergent = || {
        let mut sim = ring.rollback(rollback_tick).unwrap();
        replay_to(&mut sim, steps, &divergent_script);
        sim.hash()
    };
    let d1 = run_divergent();
    let d2 = run_divergent();
    assert_eq!(d1, d2, "the divergence itself must be deterministic");
    assert_ne!(d1, original, "different inputs must diverge from the original");
}

#[test]
fn ring_window_is_bounded() {
    // Rollback beyond the retained window is a clean error, not a panic.
    let ops = fuzz_ops();
    let (balls, steps, record_every, cap) = if ops <= 40 {
        (20, 300, 5, 16)
    } else {
        (120, 2_000, 25, 24)
    };
    let mut ring = SnapshotRing::new(cap);
    let _ = run_to_tick_recording(7, balls, steps, record_every, &mut ring);
    assert_eq!(ring.len(), cap, "ring must hold exactly its capacity");
    assert!(ring.oldest_tick().unwrap() > 0, "oldest entries were evicted");
    let oldest = ring.oldest_tick().unwrap();
    assert!(ring.rollback(oldest - 1).is_err(), "evicted tick must be missing");
    let newest = ring.newest_tick().unwrap();
    assert!(ring.rollback(newest).is_ok(), "newest tick must roll back");
}
