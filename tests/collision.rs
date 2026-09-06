//! Gate 3: collision correctness.
//!
//! Part A: the broadphase plus narrowphase overlap set exactly matches a
//! brute-force O(n^2) reference over many random entity sets.
//! Part B: the resolver prevents tunneling at the fixed timestep for fast bodies.

use forge::collision::{brute_force_pairs, detect_pairs, BodyView};
use forge::components::Shape;
use forge::math::{Transform, Vec2};
use forge::prng::Rng;
use forge::sim::{SimConfig, Simulation};

fn fuzz_ops() -> u64 {
    std::env::var("FORGE_FUZZ_OPS")
        .ok()
        .and_then(|v| v.parse().ok())
        .unwrap_or(60)
}

/// Assert every transform and velocity in the world is finite.
fn assert_world_finite(sim: &Simulation) {
    for (e, t) in sim.world.query::<Transform>() {
        assert!(
            t.position.x.is_finite() && t.position.y.is_finite(),
            "entity {e:?} has non-finite position {t:?}"
        );
    }
    for (e, v) in sim.world.query::<forge::components::Velocity>() {
        assert!(
            v.0.x.is_finite() && v.0.y.is_finite(),
            "entity {e:?} has non-finite velocity {v:?}"
        );
    }
}

fn random_bodies(rng: &mut Rng, n: usize, span: f64) -> Vec<BodyView> {
    let mut bodies = Vec::with_capacity(n);
    for _ in 0..n {
        let center = Vec2::new(rng.range_f64(0.0, span), rng.range_f64(0.0, span));
        let shape = if rng.next_f64() < 0.5 {
            Shape::Circle {
                radius: rng.range_f64(0.5, 4.0),
            }
        } else {
            Shape::Aabb {
                half: Vec2::new(rng.range_f64(0.5, 4.0), rng.range_f64(0.5, 4.0)),
            }
        };
        bodies.push(BodyView { center, shape });
    }
    bodies
}

#[test]
fn broadphase_matches_bruteforce() {
    let heavy = fuzz_ops() > 60;
    let rounds = if heavy { 200 } else { fuzz_ops() };
    let max_bodies = if heavy { 400 } else { 60 };
    let mut rng = Rng::new(0xDEADBEEF);
    for round in 0..rounds {
        let n = rng.range_u32(2, max_bodies) as usize;
        // Vary density: a tight span forces many overlaps, a loose one few.
        let span = rng.range_f64(15.0, 80.0);
        let bodies = random_bodies(&mut rng, n, span);

        let brute = brute_force_pairs(&bodies);
        let grid = detect_pairs(&bodies, 8.0);
        assert_eq!(
            brute, grid,
            "round {round}: broadphase overlap set != brute force (n={n}, span={span})"
        );
    }
}

#[test]
fn broadphase_correct_under_extreme_density() {
    // Everything piled into a tiny area: nearly all pairs overlap.
    let heavy = fuzz_ops() > 60;
    let (rounds, n) = if heavy { (200, 120) } else { (20, 30) };
    let mut rng = Rng::new(1);
    for _ in 0..rounds {
        let bodies = random_bodies(&mut rng, n, 3.0);
        assert_eq!(brute_force_pairs(&bodies), detect_pairs(&bodies, 8.0));
    }
}

/// One wall, one ball at `speed` aimed straight at it under the given dt. The
/// ball must never appear past the wall's left face.
fn assert_no_tunneling(dt: f64, speed: f64, steps: u64) {
    let config = SimConfig {
        dt,
        gravity: Vec2::ZERO,
        bounds_min: Vec2::new(-1000.0, -1000.0),
        bounds_max: Vec2::new(1000.0, 1000.0),
    };
    let mut sim = Simulation::new(config, 0);
    let wall_x = 50.0;
    let wall_half_x = 1.0;
    sim.spawn_static_box(Vec2::new(wall_x, 0.0), Vec2::new(wall_half_x, 30.0), 0.0);

    let ball_radius = 1.0;
    let ball = sim.spawn_ball(
        Vec2::new(0.0, 0.0),
        Vec2::new(speed, 0.0),
        ball_radius,
        1.0,
        0.0,
    );

    let wall_left = wall_x - wall_half_x;
    let mut moved = false;
    for _ in 0..steps {
        sim.step();
        let p = sim.world.get::<Transform>(ball).unwrap().position;
        if p.x > 1.0 {
            moved = true;
        }
        assert!(
            p.x <= wall_left + 0.001,
            "ball tunneled through the wall: x={} (wall left face {}, dt={dt}, speed={speed})",
            p.x,
            wall_left
        );
        assert!(
            p.x.is_finite(),
            "position went non-finite at dt={dt}, speed={speed}"
        );
    }
    assert!(moved, "sanity: the ball should have actually moved toward the wall");
}

#[test]
fn fast_body_does_not_tunnel_through_wall() {
    let heavy = fuzz_ops() > 60;
    if !heavy {
        // Per step the ball would move 6000/60 = 100 units, far more than the
        // wall is thick. Without continuous collision it would tunnel on step one.
        assert_no_tunneling(1.0 / 60.0, 6000.0, 300);
        return;
    }
    // Max scale: a matrix of dt and speed extremes, including a step
    // displacement of 500000/1000 = 500 units against a 2-unit wall.
    for dt in [1.0 / 60.0, 1.0 / 240.0, 1.0 / 1000.0] {
        for speed in [6000.0, 50_000.0, 500_000.0] {
            assert_no_tunneling(dt, speed, 300);
        }
    }
}

#[test]
fn many_fast_bodies_stay_contained() {
    // A box of walls, many fast balls. Long soak with periodic checkpoints:
    // containment and finiteness are asserted throughout, not only at the end.
    let heavy = fuzz_ops() > 60;
    let (balls, steps, checkpoint_every) = if heavy {
        (300, 24_000, 2_000)
    } else {
        (25, 800, 800)
    };
    let config = SimConfig::default();
    let mut sim = Simulation::new(config, 2024);
    sim.add_walls(config.bounds_min, config.bounds_max, 5.0, 0.9);
    let mut rng = Rng::new(2024);
    for _ in 0..balls {
        let r = rng.range_f64(1.0, 2.5);
        let pos = Vec2::new(
            rng.range_f64(config.bounds_min.x + 10.0, config.bounds_max.x - 10.0),
            rng.range_f64(config.bounds_min.y + 10.0, config.bounds_max.y - 10.0),
        );
        // Deliberately extreme speeds to stress continuous collision.
        let vel = Vec2::new(rng.range_f64(-3000.0, 3000.0), rng.range_f64(-3000.0, 3000.0));
        sim.spawn_ball(pos, vel, r, r * r, 0.8);
    }

    for chunk in 0..(steps / checkpoint_every) {
        sim.run(checkpoint_every, &[]);
        assert_world_finite(&sim);
        for (e, _) in sim.world.query::<forge::components::Velocity>() {
            let p = sim.world.get::<Transform>(e).unwrap().position;
            assert!(
                p.x > config.bounds_min.x - 6.0
                    && p.x < config.bounds_max.x + 6.0
                    && p.y > config.bounds_min.y - 6.0
                    && p.y < config.bounds_max.y + 6.0,
                "a fast ball escaped containment at step {}: {p:?}",
                (chunk + 1) * checkpoint_every
            );
        }
    }
}
