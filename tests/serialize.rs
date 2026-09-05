//! Gate 2: state serialize/deserialize round-trip.
//!
//! Serializing the whole world and deserializing it yields an equal world, and
//! continuing the simulation from the restored world matches continuing the
//! original bit-for-bit.

use forge::sim::{SimConfig, Simulation};

fn fuzz_ops() -> u64 {
    std::env::var("FORGE_FUZZ_OPS")
        .ok()
        .and_then(|v| v.parse().ok())
        .unwrap_or(40)
}

fn demo(seed: u64, balls: u32) -> Simulation {
    let config = SimConfig::default();
    let mut sim = Simulation::new(config, seed);
    sim.seed_scene(config, balls);
    sim
}

#[test]
fn roundtrip_bytes_are_stable() {
    let mut sim = demo(3, 25);
    sim.run(120, &[]);
    let bytes1 = sim.serialize();

    let mut restored = Simulation::empty(SimConfig::default());
    restored.deserialize(&bytes1).unwrap();

    // Restored state hashes identically and re-serializes to identical bytes.
    assert_eq!(sim.hash(), restored.hash());
    assert_eq!(bytes1, restored.serialize());
}

#[test]
fn continue_from_restored_matches_original_many_seeds() {
    let seeds = fuzz_ops();
    for s in 0..seeds {
        let mut original = demo(s, 20);
        original.run(100, &[]);

        let snapshot = original.serialize();
        let mut restored = Simulation::empty(SimConfig::default());
        restored.deserialize(&snapshot).unwrap();
        assert_eq!(original.hash(), restored.hash(), "seed {s} restore mismatch");

        // Continue both and confirm they never diverge.
        for _ in 0..100 {
            original.step();
            restored.step();
        }
        assert_eq!(
            original.hash(),
            restored.hash(),
            "seed {s} diverged after restore"
        );
    }
}

#[test]
fn deserialize_rejects_truncated_input() {
    let mut sim = demo(1, 5);
    sim.run(10, &[]);
    let mut bytes = sim.serialize();
    bytes.truncate(bytes.len() / 2);
    let mut restored = Simulation::empty(SimConfig::default());
    assert!(restored.deserialize(&bytes).is_err());
}
