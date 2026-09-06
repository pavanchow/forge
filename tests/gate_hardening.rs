//! Gate 4: adversarial and boundary inputs.
//!
//! Covers the cases the first three gates do not: zero-size bodies, bodies
//! exactly on broadphase cell boundaries, non-finite body state, corrupted
//! (not just truncated) serialized bytes, empty-world hash stability, corner
//! contacts, and entity reuse under spawn/despawn mid-script.

use forge::collision::{brute_force_pairs, collide, detect_pairs, BodyView};
use forge::components::{Collider, Shape, Velocity};
use forge::ecs::World;
use forge::math::{vec2, Transform, Vec2};
use forge::serialize::{ByteIo, Cursor, DecodeError};
use forge::sim::{Command, ScriptEntry, SimConfig, Simulation};
use forge::time::FixedTimestep;

fn aabb(center: Vec2, hx: f64, hy: f64) -> BodyView {
    BodyView {
        center,
        shape: Shape::Aabb { half: vec2(hx, hy) },
    }
}

fn circle(center: Vec2, r: f64) -> BodyView {
    BodyView {
        center,
        shape: Shape::Circle { radius: r },
    }
}

/// Assert every transform and velocity in the world is finite.
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

// ---------------------------------------------------------------------------
// Non-finite and pathological body bounds
// ---------------------------------------------------------------------------

#[test]
fn nonfinite_geometry_collides_with_nothing() {
    let solid = circle(vec2(0.0, 0.0), 1.0);
    let nan_center = circle(vec2(f64::NAN, 0.0), 1.0);
    let inf_center = circle(vec2(f64::INFINITY, 0.0), 1.0);
    let nan_radius = circle(vec2(0.0, 0.0), f64::NAN);
    for bad in [nan_center, inf_center, nan_radius] {
        assert!(collide(&solid, &bad).is_none(), "NaN/inf geometry must not collide");
        assert!(collide(&bad, &solid).is_none(), "NaN/inf geometry must not collide");
    }
}

#[test]
fn broadphase_pathological_bounds_match_bruteforce() {
    // One body whose finite bounds span the whole saturating cell range (the
    // old grid looped from i64::MIN to i64::MAX here and hung), one body with
    // an infinite center, one with a NaN center, plus ordinary bodies that do
    // not overlap anything. The overlap set must equal brute force exactly.
    let mut bodies = vec![
        circle(vec2(0.0, 0.0), 1.0),
        circle(vec2(10.0, 10.0), 1.0),
        aabb(vec2(-20.0, 5.0), 2.0, 2.0),
        aabb(vec2(1.5e306, 0.0), 2.0e306, 1.0),
        circle(vec2(f64::INFINITY, 0.0), 1.0),
        aabb(vec2(f64::NAN, f64::NAN), 1.0, 1.0),
    ];
    // And one that genuinely overlaps a normal body.
    bodies.push(circle(vec2(0.5, 0.0), 1.0));
    let brute = brute_force_pairs(&bodies);
    let grid = detect_pairs(&bodies, 4.0);
    assert_eq!(brute, grid, "pathological bounds changed the overlap set");
    assert!(grid.contains(&(0, 6)), "the real overlap must survive");
}

// ---------------------------------------------------------------------------
// Cursor hardening: absurd lengths must be errors, never panics
// ---------------------------------------------------------------------------

#[test]
fn cursor_rejects_absurd_lengths() {
    // A u8 payload followed by a String whose declared length is u64::MAX.
    let mut bytes = vec![0u8];
    bytes.extend_from_slice(&u64::MAX.to_le_bytes());
    let mut cur = Cursor::new(&bytes);
    assert_eq!(u8::read(&mut cur), Ok(0));
    let result = String::read(&mut cur);
    assert_eq!(result, Err(DecodeError::UnexpectedEof));

    // Same shape for a Vec.
    let mut cur = Cursor::new(&bytes);
    let result = Vec::<u64>::read(&mut cur);
    assert!(result.is_err(), "absurd vec length must be rejected");
}

fn sim_bytes() -> Vec<u8> {
    let config = SimConfig::default();
    let mut sim = Simulation::new(config, 3);
    sim.seed_scene(config, 5);
    sim.run(30, &[]);
    sim.serialize()
}

#[test]
fn corrupted_stream_is_rejected_or_safe_never_panics() {
    // Flip every byte of a real snapshot to a handful of values. Every
    // mutation must decode to Ok or Err, never panic, and an Ok decode must
    // step and hash safely.
    let original = sim_bytes();
    let variants = [0x00u8, 0xFF, 0x7F];
    for i in 0..original.len() {
        for v in variants {
            let mut bytes = original.clone();
            bytes[i] = v;
            let mut restored = Simulation::empty(SimConfig::default());
            if restored.deserialize(&bytes).is_ok() {
                restored.run(10, &[]);
                let _ = restored.hash();
            }
        }
    }
}

/// Hand-craft a serialized world byte stream with exact control over the
/// entity table. Layout matches `World::serialize` for a world with the single
/// registered type `Velocity`.
fn craft_world(slots: &[(u32, bool)], free: &[u32], items: &[Option<Vec2>]) -> Vec<u8> {
    let mut out = Vec::new();
    (slots.len() as u64).write(&mut out);
    for &(generation, alive) in slots {
        generation.write(&mut out);
        alive.write(&mut out);
    }
    free.to_vec().write(&mut out);
    (1u64).write(&mut out);
    "forge::components::Velocity".to_string().write(&mut out);
    items.to_vec().write(&mut out);
    out
}

#[test]
fn deserialize_rejects_invalid_free_lists() {
    let control = craft_world(&[(0, true), (1, false)], &[1], &[Some(Vec2::ONE), None]);
    let mut world = World::new();
    world.register::<Velocity>();
    world
        .deserialize(&mut Cursor::new(&control))
        .expect("control stream must decode");

    let cases: Vec<(&str, Vec<u32>)> = vec![
        ("out of range", vec![999]),
        ("u32 max", vec![u32::MAX]),
        ("duplicate entries", vec![1, 1]),
        ("points at alive slot", vec![0]),
        ("free entry with no slots", vec![0]),
    ];
    for (name, free) in cases {
        let (slots, items): (&[(u32, bool)], &[Option<Vec2>]) = if name == "empty slots nonempty free"
        {
            (&[], &[])
        } else {
            (&[(0, true), (1, false)], &[Some(Vec2::ONE), None])
        };
        let bytes = craft_world(slots, &free, items);
        let mut world = World::new();
        world.register::<Velocity>();
        let result = world.deserialize(&mut Cursor::new(&bytes));
        assert!(result.is_err(), "free list {name} must be rejected");
    }
}

#[test]
fn timestep_read_rejects_nonpositive_dt() {
    let craft = |dt: f64| {
        let mut out = Vec::new();
        dt.write(&mut out);
        0.0f64.write(&mut out);
        8u32.write(&mut out);
        out
    };
    for dt in [0.0, -1.0, f64::NAN, f64::INFINITY] {
        let bytes = craft(dt);
        let mut cur = Cursor::new(&bytes);
        assert!(
            FixedTimestep::read(&mut cur).is_err(),
            "dt={dt} must be rejected on read"
        );
    }
    let valid = craft(1.0 / 60.0);
    let mut cur = Cursor::new(&valid);
    assert!(FixedTimestep::read(&mut cur).is_ok(), "valid dt must decode");
}

// ---------------------------------------------------------------------------
// Boundary and degenerate geometry
// ---------------------------------------------------------------------------

#[test]
fn zero_size_bodies_match_oracle_and_stay_deterministic() {
    let mut bodies = vec![
        circle(vec2(1.0, 1.0), 0.0),
        circle(vec2(1.0, 1.0), 0.0),
        aabb(vec2(2.0, 1.0), 0.0, 0.0),
        circle(vec2(1.5, 1.0), 0.5),
        aabb(vec2(9.0, 9.0), 0.0, 0.0),
    ];
    bodies.push(circle(vec2(5.0, 5.0), 2.0));
    assert_eq!(brute_force_pairs(&bodies), detect_pairs(&bodies, 4.0));

    // Zero-radius balls inside walls: no NaN, deterministic hash.
    let config = SimConfig::default();
    let mut a = Simulation::new(config, 11);
    a.add_walls(config.bounds_min, config.bounds_max, 5.0, 0.9);
    a.spawn_ball(Vec2::new(50.0, 50.0), Vec2::new(10.0, 0.0), 0.0, 0.0, 0.5);
    a.spawn_ball(Vec2::new(20.0, 50.0), Vec2::ZERO, 0.0, 0.0, 0.5);
    let mut b = Simulation::new(config, 11);
    b.add_walls(config.bounds_min, config.bounds_max, 5.0, 0.9);
    b.spawn_ball(Vec2::new(50.0, 50.0), Vec2::new(10.0, 0.0), 0.0, 0.0, 0.5);
    b.spawn_ball(Vec2::new(20.0, 50.0), Vec2::ZERO, 0.0, 0.0, 0.5);
    a.run(400, &[]);
    b.run(400, &[]);
    assert_world_finite(&a);
    assert_eq!(a.hash(), b.hash(), "zero-size bodies must be deterministic");
}

#[test]
fn bodies_on_cell_boundaries_match_oracle() {
    // Cell size 4: bodies centered exactly on multiples of 4, with radii that
    // make them exactly touch and slightly overlap their neighbors.
    let mut bodies = Vec::new();
    for k in 0..12u32 {
        let x = f64::from(k) * 4.0;
        bodies.push(circle(vec2(x, 0.0), 2.0)); // exactly touching neighbors
        bodies.push(circle(vec2(x, 8.0), 2.5)); // overlapping neighbors
        bodies.push(aabb(vec2(x, 16.0), 2.0, 2.0)); // faces exactly on lines
    }
    let brute = brute_force_pairs(&bodies);
    let grid = detect_pairs(&bodies, 4.0);
    assert_eq!(brute, grid, "cell-boundary bodies changed the overlap set");
    // Touching (distance == sum of radii) is not an overlap.
    assert!(
        !brute.contains(&(0, 2)),
        "exactly touching bodies must not count as overlapping"
    );
}

// ---------------------------------------------------------------------------
// Multi-contact and containment corners
// ---------------------------------------------------------------------------

#[test]
fn ball_overlapping_a_wall_cannot_tunnel_through_it() {
    // The swept test skips pre-existing overlap by design, so a body that ends
    // up inside a wall (piled on by other bodies, or spawned there by a
    // caller) must not deepen the overlap with its own displacement. Before
    // the pre-overlap guard, a ball inside the bottom wall with a huge inward
    // velocity crossed the whole wall on the next step.
    let config = SimConfig::default();
    let min = config.bounds_min;
    let max = config.bounds_max;
    let build = || {
        let mut sim = Simulation::new(config, 5);
        sim.add_walls(min, max, 5.0, 0.9);
        // The bottom wall's center is (cx, min.y - 5) with half-height 5, so
        // this ball starts fully inside the wall moving straight down fast.
        sim.spawn_ball(
            Vec2::new(50.0, min.y - 5.0),
            Vec2::new(0.0, -3000.0),
            1.0,
            1.0,
            0.8,
        );
        sim
    };
    let mut a = build();
    let mut b = build();
    a.run(400, &[]);
    b.run(400, &[]);
    assert_world_finite(&a);
    assert_eq!(a.hash(), b.hash(), "pre-overlap escape must be deterministic");
    for (e, _) in a.world.query::<Velocity>() {
        let p = a.world.get::<Transform>(e).unwrap().position;
        assert!(
            p.y > min.y - 6.0 && p.y < max.y + 6.0,
            "ball tunneled through the wall it started inside: {p:?}"
        );
    }
}

#[test]
fn nonfinite_spawn_and_impulse_never_poison_the_world() {
    // Non-finite values can only enter through caller error, and the engine's
    // contract is that they are neutralized at the boundary, never propagated
    // into world state or the hash.
    let config = SimConfig::default();
    let build = || {
        let mut sim = Simulation::new(config, 9);
        sim.add_walls(config.bounds_min, config.bounds_max, 5.0, 0.9);
        sim.spawn_ball(
            Vec2::new(f64::NAN, 50.0),
            Vec2::new(f64::INFINITY, f64::NAN),
            f64::NAN,
            f64::INFINITY,
            f64::NAN,
        );
        sim.spawn_ball(Vec2::new(30.0, 50.0), Vec2::new(5.0, 5.0), 1.5, 2.0, 0.8);
        sim
    };
    let mut a = build();
    let mut b = build();
    let script: Vec<ScriptEntry> = vec![
        (
            10,
            Command::Impulse {
                entity: a.world.entities_with::<Velocity>()[1],
                delta_v: Vec2::new(f64::NAN, f64::INFINITY),
            },
        ),
        (
            20,
            Command::SpawnBall {
                pos: Vec2::new(f64::INFINITY, f64::NAN),
                vel: Vec2::new(f64::NAN, f64::NAN),
                radius: f64::INFINITY,
                mass: f64::NAN,
                restitution: f64::INFINITY,
            },
        ),
    ];
    let script2 = vec![
        (
            10,
            Command::Impulse {
                entity: b.world.entities_with::<Velocity>()[1],
                delta_v: Vec2::new(f64::NAN, f64::INFINITY),
            },
        ),
        (
            20,
            Command::SpawnBall {
                pos: Vec2::new(f64::INFINITY, f64::NAN),
                vel: Vec2::new(f64::NAN, f64::NAN),
                radius: f64::INFINITY,
                mass: f64::NAN,
                restitution: f64::INFINITY,
            },
        ),
    ];
    a.run(300, &script);
    b.run(300, &script2);
    assert_world_finite(&a);
    assert_eq!(a.hash(), b.hash(), "non-finite input must be handled deterministically");
}

#[test]
fn corner_slam_is_deterministic_and_contained() {
    // A ball fired into an exact corner where two walls meet: two simultaneous
    // contacts, resolved in a fixed order.
    let config = SimConfig::default();
    let build = || {
        let mut sim = Simulation::new(config, 5);
        sim.add_walls(config.bounds_min, config.bounds_max, 5.0, 0.9);
        sim.spawn_ball(
            Vec2::new(2.0, 2.0),
            Vec2::new(2500.0, 2500.0),
            1.5,
            2.0,
            0.7,
        );
        sim
    };
    let mut a = build();
    let mut b = build();
    a.run(600, &[]);
    b.run(600, &[]);
    assert_world_finite(&a);
    assert_eq!(a.hash(), b.hash(), "corner contact must be deterministic");
    let min = config.bounds_min;
    let max = config.bounds_max;
    for (e, _) in a.world.query::<Velocity>() {
        let p = a.world.get::<Transform>(e).unwrap().position;
        assert!(
            p.x > min.x - 6.0 && p.x < max.x + 6.0 && p.y > min.y - 6.0 && p.y < max.y + 6.0,
            "ball escaped through a corner: {p:?}"
        );
    }
}

// ---------------------------------------------------------------------------
// Empty world and restore chains
// ---------------------------------------------------------------------------

#[test]
fn empty_world_hash_is_stable() {
    let config = SimConfig::default();
    // Same seed: the RNG state is part of the serialized world, so seed is the
    // only allowed difference. Stability means identical worlds hash identically.
    let mut a = Simulation::new(config, 1);
    let mut b = Simulation::new(config, 1);
    let c = Simulation::new(config, 2);
    assert_eq!(a.hash(), b.hash(), "identical empty worlds must hash identically");
    // Different seeds differ only in RNG state, which is serialized too.
    assert_ne!(a.hash(), c.hash(), "rng state must be part of the hash");
    a.run(100, &[]);
    b.run(100, &[]);
    assert_eq!(a.hash(), b.hash(), "empty worlds must step identically");

    // Restore chain: empty -> restore -> restore of restore.
    let bytes = a.serialize();
    let mut r1 = Simulation::empty(config);
    r1.deserialize(&bytes).unwrap();
    let bytes2 = r1.serialize();
    let mut r2 = Simulation::empty(config);
    r2.deserialize(&bytes2).unwrap();
    assert_eq!(a.hash(), r2.hash(), "second-generation restore must match");
}

#[test]
fn restore_chain_matches_original_across_generations() {
    let config = SimConfig::default();
    let mut original = Simulation::new(config, 42);
    original.seed_scene(config, 12);
    original.run(200, &[]);

    let mut current = original.serialize();
    for generation in 0..5 {
        let mut restored = Simulation::empty(config);
        restored.deserialize(&current).unwrap();
        assert_eq!(
            original.hash(),
            restored.hash(),
            "generation {generation} restore mismatch"
        );
        original.run(50, &[]);
        restored.run(50, &[]);
        assert_eq!(
            original.hash(),
            restored.hash(),
            "generation {generation} diverged after continue"
        );
        current = restored.serialize();
    }
}

// ---------------------------------------------------------------------------
// Entity reuse under scripts
// ---------------------------------------------------------------------------

#[test]
fn mass_spawn_despawn_midscript_is_deterministic() {
    let config = SimConfig::default();
    let build = || {
        let mut sim = Simulation::new(config, 77);
        sim.seed_scene(config, 20);
        let mut script: Vec<ScriptEntry> = Vec::new();
        // Spawn a wave of balls every 40 ticks for 400 ticks.
        for wave in 0..10u64 {
            for k in 0..8u64 {
                let tick = 40 * wave + k / 4;
                script.push((
                    tick,
                    Command::SpawnBall {
                        pos: Vec2::new(10.0 + 5.0 * k as f64, 90.0),
                        vel: Vec2::new(-20.0 + 4.0 * k as f64, -50.0),
                        radius: 1.0 + 0.25 * (k % 4) as f64,
                        mass: 1.0 + (k % 3) as f64,
                        restitution: 0.8,
                    },
                ));
            }
        }
        (sim, script)
    };
    let (mut a, script) = build();
    let (mut b, script2) = build();
    assert_eq!(script.len(), script2.len());

    a.run(200, &script);
    b.run(200, &script2);
    // Mid-script: despawn every other dynamic body through direct world access,
    // then keep running the same script on both. Index reuse is exercised hard.
    let dynamics: Vec<_> = a
        .world
        .query::<Velocity>()
        .map(|(e, _)| e)
        .filter(|e| e.index % 2 == 0)
        .collect();
    for e in &dynamics {
        assert!(a.world.despawn(*e));
        assert!(b.world.despawn(*e));
    }
    a.run(300, &script);
    b.run(300, &script2);
    assert_world_finite(&a);
    assert_eq!(a.hash(), b.hash(), "spawn/despawn churn must be deterministic");

    // A stale handle from before the churn must be inert, not a panic.
    let stale = dynamics[0];
    a.run(
        10,
        &[(0, Command::Impulse { entity: stale, delta_v: Vec2::new(100.0, 0.0) })],
    );
    assert_world_finite(&a);
}

#[test]
fn collider_component_roundtrips_through_world_serialize() {
    // A world carrying Collider and Transform round-trips bit-for-bit.
    let mut w = World::new();
    w.register::<Transform>();
    w.register::<Collider>();
    let e = w.spawn();
    w.insert(e, Transform::from_position(vec2(1.0, 2.0)));
    w.insert(e, Collider::circle(0.5));
    let mut buf = Vec::new();
    w.serialize(&mut buf);
    let mut w2 = World::new();
    w2.register::<Transform>();
    w2.register::<Collider>();
    w2.deserialize(&mut Cursor::new(&buf)).unwrap();
    let mut buf2 = Vec::new();
    w2.serialize(&mut buf2);
    assert_eq!(buf, buf2);
}
