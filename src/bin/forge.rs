//! Forge headless CLI.
//!
//! Runs a scripted simulation from a seed, prints world-state hashes along the
//! way, then proves determinism by running the identical scenario a second time
//! and checking the hashes match bit-for-bit. Also verifies a mid-run
//! serialize/restore round-trip.
//!
//! Usage:
//!   forge [--seed N] [--steps N] [--balls N]
//!
//! Environment (CLI flags take precedence):
//!   FORGE_SEED, FORGE_STEPS, FORGE_BALLS, FORGE_FUZZ_OPS (alias for steps)

use forge::math::Vec2;
use forge::sim::{Command, ScriptEntry, SimConfig, Simulation};

fn env_u64(key: &str) -> Option<u64> {
    std::env::var(key).ok().and_then(|v| v.parse().ok())
}

fn parse_args() -> (u64, u64, u32) {
    let mut seed = env_u64("FORGE_SEED").unwrap_or(1_234_567);
    let mut steps = env_u64("FORGE_STEPS")
        .or_else(|| env_u64("FORGE_FUZZ_OPS"))
        .unwrap_or(600);
    let mut balls = env_u64("FORGE_BALLS").unwrap_or(40) as u32;

    let args: Vec<String> = std::env::args().collect();
    let mut i = 1;
    while i < args.len() {
        match args[i].as_str() {
            "--seed" => {
                if let Some(v) = args.get(i + 1).and_then(|s| s.parse().ok()) {
                    seed = v;
                    i += 1;
                }
            }
            "--steps" => {
                if let Some(v) = args.get(i + 1).and_then(|s| s.parse().ok()) {
                    steps = v;
                    i += 1;
                }
            }
            "--balls" => {
                if let Some(v) = args.get(i + 1).and_then(|s| s.parse().ok()) {
                    balls = v;
                    i += 1;
                }
            }
            "-h" | "--help" => {
                println!("usage: forge [--seed N] [--steps N] [--balls N]");
                std::process::exit(0);
            }
            _ => {}
        }
        i += 1;
    }
    (seed, steps, balls)
}

/// Build a fresh simulation and its input script for the given seed and size.
fn build(seed: u64, balls: u32, steps: u64) -> (Simulation, Vec<ScriptEntry>) {
    let config = SimConfig::default();
    let mut sim = Simulation::new(config, seed);
    sim.seed_scene(config, balls);

    // A deterministic script: nudge the first ball, flip gravity, then launch a
    // very fast ball straight at a wall to exercise continuous collision.
    let mut script: Vec<ScriptEntry> = Vec::new();
    if let Some(first) = sim.world.entities_with::<forge::components::Velocity>().first() {
        script.push((steps / 4, Command::Impulse {
            entity: *first,
            delta_v: Vec2::new(40.0, 60.0),
        }));
    }
    script.push((steps / 2, Command::SetGravity(Vec2::new(0.0, 30.0))));
    script.push((
        steps / 2 + 1,
        Command::SpawnBall {
            pos: Vec2::new(10.0, 50.0),
            vel: Vec2::new(5000.0, 0.0),
            radius: 2.0,
            mass: 4.0,
            restitution: 0.5,
        },
    ));
    (sim, script)
}

fn run_and_trace(seed: u64, balls: u32, steps: u64, trace: bool) -> u64 {
    let (mut sim, script) = build(seed, balls, steps);
    let interval = (steps / 6).max(1);
    if trace {
        println!("  tick {:>6}  entities {:>4}  hash 0x{:016x}", sim.tick, sim.world.entity_count(), sim.hash());
    }
    for _ in 0..steps {
        for &(tick, cmd) in &script {
            if tick == sim.tick {
                sim.apply(cmd);
            }
        }
        sim.step();
        if trace && sim.tick % interval == 0 {
            println!(
                "  tick {:>6}  entities {:>4}  hash 0x{:016x}",
                sim.tick,
                sim.world.entity_count(),
                sim.hash()
            );
        }
    }
    sim.hash()
}

/// Run to the halfway point, snapshot, restore, and confirm the continuation
/// hash matches an uninterrupted run.
fn roundtrip_check(seed: u64, balls: u32, steps: u64) -> bool {
    let (mut a, script) = build(seed, balls, steps);
    let half = steps / 2;

    let apply_due = |sim: &mut Simulation, script: &[ScriptEntry]| {
        for &(tick, cmd) in script {
            if tick == sim.tick {
                sim.apply(cmd);
            }
        }
    };

    for _ in 0..half {
        apply_due(&mut a, &script);
        a.step();
    }
    let snapshot = a.serialize();
    let mut b = Simulation::empty(SimConfig::default());
    if b.deserialize(&snapshot).is_err() {
        return false;
    }
    if a.hash() != b.hash() {
        return false;
    }
    for _ in half..steps {
        apply_due(&mut a, &script);
        a.step();
        apply_due(&mut b, &script);
        b.step();
    }
    a.hash() == b.hash()
}

fn main() {
    let (seed, steps, balls) = parse_args();

    println!("Forge headless determinism check");
    println!("seed={seed} steps={steps} balls={balls}");
    println!();
    println!("Run 1 (traced):");
    let h1 = run_and_trace(seed, balls, steps, true);
    println!();

    let h2 = run_and_trace(seed, balls, steps, false);
    println!("Run 1 final hash: 0x{h1:016x}");
    println!("Run 2 final hash: 0x{h2:016x}");

    let deterministic = h1 == h2;
    println!(
        "determinism (same seed same hash): {}",
        if deterministic { "PASS" } else { "FAIL" }
    );

    let roundtrip = roundtrip_check(seed, balls, steps);
    println!(
        "serialize/restore round-trip:      {}",
        if roundtrip { "PASS" } else { "FAIL" }
    );

    // A different seed must produce a different world.
    let h_other = run_and_trace(seed.wrapping_add(1), balls, steps, false);
    let distinct = h_other != h1;
    println!(
        "distinct seed distinct hash:       {}",
        if distinct { "PASS" } else { "FAIL" }
    );

    if deterministic && roundtrip && distinct {
        println!("\nALL CHECKS PASSED");
        std::process::exit(0);
    } else {
        eprintln!("\nCHECK FAILED");
        std::process::exit(1);
    }
}
