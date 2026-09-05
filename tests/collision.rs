//! Gate 3: collision correctness.
//!
//! Part A: the broadphase plus narrowphase overlap set exactly matches a
//! brute-force O(n^2) reference over many random entity sets.
//! Part B: the resolver prevents tunneling at the fixed timestep for fast bodies.

use forge::collision::{brute_force_pairs, detect_pairs, BodyView};
use forge::components::Shape;
use forge::math::Vec2;
use forge::prng::Rng;
use forge::sim::{SimConfig, Simulation};

fn fuzz_ops() -> u64 {
    std::env::var("FORGE_FUZZ_OPS")
        .ok()
        .and_then(|v| v.parse().ok())
        .unwrap_or(60)
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
    let rounds = fuzz_ops();
    let mut rng = Rng::new(0xDEADBEEF);
    for round in 0..rounds {
        let n = rng.range_u32(2, 60) as usize;
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
    let mut rng = Rng::new(1);
    for _ in 0..20 {
        let bodies = random_bodies(&mut rng, 30, 3.0);
        assert_eq!(brute_force_pairs(&bodies), detect_pairs(&bodies, 8.0));
    }
}

#[test]
fn fast_body_does_not_tunnel_through_wall() {
    // No gravity, one thin static wall, one very fast ball aimed straight at it.
    let config = SimConfig {
        dt: 1.0 / 60.0,
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
        Vec2::new(6000.0, 0.0),
        ball_radius,
        1.0,
        0.0,
    );

    // Per step the ball would move 6000/60 = 100 units, far more than the wall is
    // thick. Without continuous collision it would tunnel on step one.
    let wall_left = wall_x - wall_half_x;
    let mut moved = false;
    for _ in 0..300 {
        sim.step();
        let p = sim.world.get::<forge::math::Transform>(ball).unwrap().position;
        if p.x > 1.0 {
            moved = true;
        }
        assert!(
            p.x <= wall_left + 0.001,
            "ball tunneled through the wall: x={} (wall left face {})",
            p.x,
            wall_left
        );
    }
    assert!(moved, "sanity: the ball should have actually moved toward the wall");
}

#[test]
fn many_fast_bodies_stay_contained() {
    // A box of walls, many fast balls. After a long run none escapes.
    let config = SimConfig::default();
    let mut sim = Simulation::new(config, 2024);
    sim.add_walls(config.bounds_min, config.bounds_max, 5.0, 0.9);
    let mut rng = Rng::new(2024);
    for _ in 0..25 {
        let r = rng.range_f64(1.0, 2.5);
        let pos = Vec2::new(
            rng.range_f64(config.bounds_min.x + 10.0, config.bounds_max.x - 10.0),
            rng.range_f64(config.bounds_min.y + 10.0, config.bounds_max.y - 10.0),
        );
        // Deliberately extreme speeds to stress continuous collision.
        let vel = Vec2::new(rng.range_f64(-3000.0, 3000.0), rng.range_f64(-3000.0, 3000.0));
        sim.spawn_ball(pos, vel, r, r * r, 0.8);
    }

    sim.run(800, &[]);

    for (e, _) in sim.world.query::<forge::components::Velocity>() {
        let p = sim.world.get::<forge::math::Transform>(e).unwrap().position;
        assert!(
            p.x > config.bounds_min.x - 6.0
                && p.x < config.bounds_max.x + 6.0
                && p.y > config.bounds_min.y - 6.0
                && p.y < config.bounds_max.y + 6.0,
            "a fast ball escaped containment: {p:?}"
        );
    }
}
