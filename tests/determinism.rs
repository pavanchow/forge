//! Gate 1: deterministic replay.
//!
//! Given a seed and an input script, the simulation produces an identical
//! world-state hash on every run, across many seeds. This is the core property
//! the whole engine is built to guarantee.

use forge::math::Vec2;
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
    (sim, script)
}

fn run(seed: u64, balls: u32, steps: u64) -> u64 {
    let (mut sim, script) = build(seed, balls, steps);
    sim.run(steps, &script);
    sim.hash()
}

#[test]
fn same_seed_identical_hash_across_many_seeds() {
    let seeds = fuzz_ops();
    let steps = 200;
    let balls = 25;
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
    let seeds = fuzz_ops().max(8);
    let mut hashes = std::collections::HashSet::new();
    for s in 0..seeds {
        hashes.insert(run(s, 20, 150));
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
    let steps = 300;
    let reference = run(777, 30, steps);
    for _ in 0..5 {
        assert_eq!(run(777, 30, steps), reference);
    }
}
