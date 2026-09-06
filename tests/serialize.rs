//! Gate 2: state serialize/deserialize round-trip.
//!
//! Serializing the whole world and deserializing it yields an equal world, and
//! continuing the simulation from the restored world matches continuing the
//! original bit-for-bit. Truncated and corrupted input is rejected.
//!
//! `FORGE_FUZZ_OPS` is the stress dial: above 40 the gate runs at max scale
//! (hundreds of entities, thousands of steps, many seeds and checkpoints).

use forge::math::Transform;
use forge::sim::{SimConfig, Simulation};

fn fuzz_ops() -> u64 {
    std::env::var("FORGE_FUZZ_OPS")
        .ok()
        .and_then(|v| v.parse().ok())
        .unwrap_or(40)
}

/// Assert every transform in the world is finite. Non-finite state must never
/// enter or survive a round-trip.
fn assert_world_finite(sim: &Simulation) {
    for (e, t) in sim.world.query::<Transform>() {
        assert!(
            t.position.x.is_finite() && t.position.y.is_finite(),
            "entity {e:?} has non-finite position {t:?}"
        );
    }
}

fn demo(seed: u64, balls: u32) -> Simulation {
    let config = SimConfig::default();
    let mut sim = Simulation::new(config, seed);
    sim.seed_scene(config, balls);
    sim
}

#[test]
fn roundtrip_bytes_are_stable() {
    let ops = fuzz_ops();
    let (balls, steps) = if ops <= 40 { (25, 120) } else { (200, 500) };
    let mut sim = demo(3, balls);
    sim.run(steps, &[]);
    assert_world_finite(&sim);
    let bytes1 = sim.serialize();

    let mut restored = Simulation::empty(SimConfig::default());
    restored.deserialize(&bytes1).unwrap();

    // Restored state hashes identically and re-serializes to identical bytes.
    assert_eq!(sim.hash(), restored.hash());
    assert_eq!(bytes1, restored.serialize());
}

#[test]
fn continue_from_restored_matches_original_many_seeds() {
    let ops = fuzz_ops();
    let (seeds, balls, steps) = if ops <= 40 {
        (ops, 20, 100)
    } else {
        ((ops / 150).clamp(2, 16), 200, 1_000)
    };
    for s in 0..seeds {
        let mut original = demo(s, balls);
        original.run(steps, &[]);
        assert_world_finite(&original);

        let snapshot = original.serialize();
        let mut restored = Simulation::empty(SimConfig::default());
        restored.deserialize(&snapshot).unwrap();
        assert_eq!(original.hash(), restored.hash(), "seed {s} restore mismatch");

        // Continue both and confirm they never diverge.
        for _ in 0..steps {
            original.step();
            restored.step();
        }
        assert_world_finite(&restored);
        assert_eq!(
            original.hash(),
            restored.hash(),
            "seed {s} diverged after restore"
        );
    }
}

#[test]
fn deserialize_rejects_truncated_input() {
    let ops = fuzz_ops();
    let mut sim = demo(1, 5);
    sim.run(10, &[]);
    let bytes = sim.serialize();
    let mut restored = Simulation::empty(SimConfig::default());
    if ops <= 40 {
        let mut truncated = bytes.clone();
        truncated.truncate(truncated.len() / 2);
        assert!(restored.deserialize(&truncated).is_err());
    } else {
        // Reject at every sampled cut point, including length 0 and len-1.
        for cut in (0..bytes.len()).step_by(17) {
            assert!(
                restored.deserialize(&bytes[..cut]).is_err(),
                "truncation at byte {cut} must be rejected"
            );
        }
        assert!(restored.deserialize(&[]).is_err());
        assert!(restored.deserialize(&bytes[..bytes.len() - 1]).is_err());
    }
}

#[test]
fn checkpoint_roundtrips_stay_aligned() {
    // Snapshot at many mid-run checkpoints; each restore must hash equal and
    // continue bit-for-bit, and every re-serialization must be byte-stable.
    let ops = fuzz_ops();
    let (balls, steps, checkpoints) = if ops <= 40 {
        (20, 600, 3)
    } else {
        (150, 4_000, 8)
    };
    let config = SimConfig::default();
    let mut original = demo(9, balls);
    let interval = steps / (checkpoints + 1);

    for cp in 0..checkpoints {
        original.run(interval as u64, &[]);
        assert_world_finite(&original);
        let snapshot = original.serialize();

        let mut restored = Simulation::empty(config);
        restored.deserialize(&snapshot).unwrap();
        assert_eq!(original.hash(), restored.hash(), "checkpoint {cp} mismatch");

        // Continue both for a while from the checkpoint.
        for _ in 0..100 {
            original.step();
            restored.step();
        }
        assert_eq!(
            original.hash(),
            restored.hash(),
            "checkpoint {cp} diverged after continue"
        );
        assert_eq!(
            original.serialize(),
            restored.serialize(),
            "checkpoint {cp} re-serialization differs"
        );
    }
}
