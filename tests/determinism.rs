//! Gate 1: deterministic replay.
//!
//! Given a seed and an input script, the simulation produces an identical
//! world-state hash on every run, across many seeds. This is the core property
//! the whole engine is built to guarantee.
//!
//! `FORGE_FUZZ_OPS` is the stress dial. At the default (40 or below) the gate
//! runs the CI-sized workload. Above 40 the gate switches to max scale: hundreds
//! of entities and tens of thousands of steps per run, with fewer seeds so the
//! whole suite stays inside the stress time budget.

// Loop-counter-to-float casts in test arithmetic: every value here is a small
// spawn-wave index far below 2^53, so no precision is actually lost.
#![allow(clippy::cast_precision_loss)]

use forge::components::Velocity;
use forge::math::{Transform, Vec2};
use forge::sim::{Command, ScriptEntry, SimConfig, Simulation};

fn fuzz_ops() -> u64 {
    std::env::var("FORGE_FUZZ_OPS")
        .ok()
        .and_then(|v| v.parse().ok())
        .unwrap_or(40)
}

/// Max-scale workload parameters: (seed count, balls, steps).
/// Defaults are unchanged when `FORGE_FUZZ_OPS` is 40 or less.
fn scale(ops: u64, base_balls: u32, base_steps: u64) -> (u64, u32, u64) {
    if ops <= 40 {
        (ops.max(1), base_balls, base_steps)
    } else {
        let seeds = (ops / 200).clamp(2, 14);
        let balls = base_balls.max(200);
        let steps = base_steps.max(15_000);
        (seeds, balls, steps)
    }
}

/// Assert every transform and velocity in the world is finite. Non-finite
/// state must never leak into a world hash.
fn assert_world_finite(sim: &Simulation) {
    for (e, t) in sim.world.query::<Transform>() {
        assert!(
            t.position.x.is_finite() && t.position.y.is_finite(),
            "entity {e:?} has non-finite position {t:?}"
        );
    }
    for (e, v) in sim.world.query::<Velocity>() {
        assert!(
            v.0.x.is_finite() && v.0.y.is_finite(),
            "entity {e:?} has non-finite velocity {v:?}"
        );
    }
}

fn build(seed: u64, balls: u32, steps: u64) -> (Simulation, Vec<ScriptEntry>) {
    let config = SimConfig::default();
    let mut sim = Simulation::new(config, seed);
    sim.seed_scene(config, balls);
    let mut script: Vec<ScriptEntry> = Vec::new();
    if let Some(first) = sim
        .world
        .entities_with::<forge::components::Velocity>()
        .first()
    {
        script.push((
            steps / 3,
            Command::Impulse {
                entity: *first,
                delta_v: Vec2::new(35.0, 15.0),
            },
        ));
    }
    script.push((steps / 2, Command::SetGravity(Vec2::new(5.0, 20.0))));
    // A late spawn wave exercises entity-index growth deep into the run.
    for k in 0..(steps / 1000).min(50) {
        script.push((
            steps.saturating_sub(100 + k),
            Command::SpawnBall {
                pos: Vec2::new(10.0 + 2.0 * k as f64, 90.0),
                vel: Vec2::new(-10.0 + k as f64, -40.0),
                radius: 1.0,
                mass: 1.0,
                restitution: 0.8,
            },
        ));
    }
    (sim, script)
}

fn run(seed: u64, balls: u32, steps: u64) -> u64 {
    let (mut sim, script) = build(seed, balls, steps);
    sim.run(steps, &script);
    assert_world_finite(&sim);
    sim.hash()
}

#[test]
fn same_seed_identical_hash_across_many_seeds() {
    let (seeds, balls, steps) = scale(fuzz_ops(), 25, 200);
    for s in 0..seeds {
        let h1 = run(s, balls, steps);
        let h2 = run(s, balls, steps);
        assert_eq!(h1, h2, "seed {s} was not deterministic");
    }
}

#[test]
fn distinct_seeds_generally_diverge() {
    // Not a hard guarantee for every pair, but collisions should be vanishingly
    // rare. Confirm the set of hashes is highly diverse.
    let ops = fuzz_ops();
    let (seeds, balls, steps) = if ops <= 40 {
        (ops.max(8), 20, 150)
    } else {
        ((ops / 100).clamp(8, 24), 200, 2_000)
    };
    let mut hashes = std::collections::HashSet::new();
    for s in 0..seeds {
        hashes.insert(run(s, balls, steps));
    }
    // Expect almost all distinct. Allow a tiny slack in case of an accidental tie.
    assert!(
        hashes.len() as u64 >= seeds - 1,
        "too many identical hashes across seeds: {} unique of {}",
        hashes.len(),
        seeds
    );
}

#[test]
fn replay_is_stable_when_rerun_many_times() {
    let ops = fuzz_ops();
    let (_, balls, steps) = scale(ops, 30, 300);
    let reruns = if ops <= 40 { 5 } else { 2 };
    let reference = run(777, balls, steps);
    for _ in 0..reruns {
        assert_eq!(run(777, balls, steps), reference);
    }
}
