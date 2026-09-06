//! The simulation: world plus systems, driven by a fixed timestep.
//!
//! A `Simulation` owns the ECS world, the seeded RNG, gravity, and the tick
//! counter. `step` runs one fixed slice of the world: integrate velocities,
//! sweep dynamic bodies against static geometry to prevent tunneling, then run
//! discrete collision detection and impulse resolution. Everything a step reads
//! or writes is part of the serialized state, so a restored simulation continues
//! bit-for-bit identically.

use crate::collision::{self, BodyView};
use crate::components::{Collider, Forces, RigidBody, Shape, Velocity};
use crate::ecs::{Entity, World};
use crate::hash::hash_bytes;
use crate::math::{Transform, Vec2};
use crate::prng::Rng;
use crate::serialize::{ByteIo, Cursor, DecodeError};
use crate::time::FixedTimestep;

/// Static configuration used to build a simulation.
#[derive(Clone, Copy, Debug)]
pub struct SimConfig {
    pub dt: f64,
    pub gravity: Vec2,
    pub bounds_min: Vec2,
    pub bounds_max: Vec2,
}

impl Default for SimConfig {
    fn default() -> Self {
        SimConfig {
            dt: 1.0 / 120.0,
            gravity: Vec2::new(0.0, -30.0),
            bounds_min: Vec2::new(0.0, 0.0),
            bounds_max: Vec2::new(100.0, 100.0),
        }
    }
}

/// A scripted command applied at the start of a specific tick.
#[derive(Clone, Copy, Debug)]
pub enum Command {
    SetGravity(Vec2),
    Impulse { entity: Entity, delta_v: Vec2 },
    SpawnBall { pos: Vec2, vel: Vec2, radius: f64, mass: f64, restitution: f64 },
}

/// One entry in an input script: run `command` when the sim reaches `tick`.
pub type ScriptEntry = (u64, Command);

const POSITION_CORRECTION: f64 = 0.8;
const PENETRATION_SLOP: f64 = 1e-4;

/// Non-finite scalars collapse to zero at state boundaries. Non-finite values
/// can only enter through caller error or corruption, and the engine's contract
/// is to neutralize them (narrowphase rejects non-finite geometry the same way)
/// rather than let a NaN reach the world hash.
fn finite_or_zero_f(v: f64) -> f64 {
    if v.is_finite() {
        v
    } else {
        0.0
    }
}

pub struct Simulation {
    pub world: World,
    pub rng: Rng,
    pub gravity: Vec2,
    pub timestep: FixedTimestep,
    pub tick: u64,
}

impl Simulation {
    fn register(world: &mut World) {
        // Registration order is part of the serialization contract.
        world.register::<Transform>();
        world.register::<Velocity>();
        world.register::<Forces>();
        world.register::<RigidBody>();
        world.register::<Collider>();
        world.register::<crate::components::Parent>();
    }

    /// An empty simulation with components registered. Used before deserializing.
    pub fn empty(config: SimConfig) -> Self {
        let mut world = World::new();
        Self::register(&mut world);
        Simulation {
            world,
            rng: Rng::new(0),
            gravity: config.gravity,
            timestep: FixedTimestep::new(config.dt),
            tick: 0,
        }
    }

    /// A new simulation seeded by `seed`, with no entities.
    pub fn new(config: SimConfig, seed: u64) -> Self {
        let mut sim = Self::empty(config);
        sim.rng = Rng::new(seed);
        sim
    }

    /// Spawn a dynamic circle body. Non-finite arguments are neutralized to
    /// zero at this boundary.
    pub fn spawn_ball(
        &mut self,
        pos: Vec2,
        vel: Vec2,
        radius: f64,
        mass: f64,
        restitution: f64,
    ) -> Entity {
        let e = self.world.spawn();
        self.world
            .insert(e, Transform::from_position(pos.finite_or_zero()));
        self.world.insert(e, Velocity(vel.finite_or_zero()));
        self.world.insert(e, Forces(Vec2::ZERO));
        self.world.insert(
            e,
            RigidBody::dynamic(finite_or_zero_f(mass), finite_or_zero_f(restitution)),
        );
        self.world
            .insert(e, Collider::circle(finite_or_zero_f(radius)));
        e
    }

    /// Spawn a static (immovable) axis-aligned box. Non-finite arguments are
    /// neutralized to zero at this boundary.
    pub fn spawn_static_box(&mut self, center: Vec2, half: Vec2, restitution: f64) -> Entity {
        let e = self.world.spawn();
        self.world
            .insert(e, Transform::from_position(center.finite_or_zero()));
        self.world
            .insert(e, RigidBody::fixed(finite_or_zero_f(restitution)));
        self.world
            .insert(e, Collider::aabb(half.finite_or_zero()));
        e
    }

    /// Build four static walls enclosing the configured bounds.
    pub fn add_walls(&mut self, min: Vec2, max: Vec2, thickness: f64, restitution: f64) {
        let w = max.x - min.x;
        let h = max.y - min.y;
        let cx = (min.x + max.x) * 0.5;
        let cy = (min.y + max.y) * 0.5;
        let t = thickness;
        // Bottom, top, left, right.
        self.spawn_static_box(Vec2::new(cx, min.y - t), Vec2::new(w * 0.5 + t, t), restitution);
        self.spawn_static_box(Vec2::new(cx, max.y + t), Vec2::new(w * 0.5 + t, t), restitution);
        self.spawn_static_box(Vec2::new(min.x - t, cy), Vec2::new(t, h * 0.5 + t), restitution);
        self.spawn_static_box(Vec2::new(max.x + t, cy), Vec2::new(t, h * 0.5 + t), restitution);
    }

    /// Populate a walled box with `count` randomly placed, randomly moving balls.
    pub fn seed_scene(&mut self, config: SimConfig, count: u32) {
        self.add_walls(config.bounds_min, config.bounds_max, 5.0, 0.9);
        let min = config.bounds_min;
        let max = config.bounds_max;
        for _ in 0..count {
            let r = self.rng.range_f64(1.0, 3.0);
            let pos = Vec2::new(
                self.rng.range_f64(min.x + r + 1.0, max.x - r - 1.0),
                self.rng.range_f64(min.y + r + 1.0, max.y - r - 1.0),
            );
            let vel = Vec2::new(
                self.rng.range_f64(-20.0, 20.0),
                self.rng.range_f64(-20.0, 20.0),
            );
            let mass = r * r;
            self.spawn_ball(pos, vel, r, mass, 0.85);
        }
    }

    /// Apply a scripted command immediately.
    pub fn apply(&mut self, command: Command) {
        match command {
            Command::SetGravity(g) => self.gravity = g,
            Command::Impulse { entity, delta_v } => {
                let delta_v = delta_v.finite_or_zero();
                if let Some(v) = self.world.get_mut::<Velocity>(entity) {
                    v.0 += delta_v;
                }
            }
            Command::SpawnBall {
                pos,
                vel,
                radius,
                mass,
                restitution,
            } => {
                self.spawn_ball(pos, vel, radius, mass, restitution);
            }
        }
    }

    /// The recommended broadphase cell size: the largest body diameter, so no
    /// pair is ever missed while cells stay as coarse as possible.
    fn cell_size(bodies: &[BodyView]) -> f64 {
        let mut max_extent = 1.0_f64;
        for b in bodies {
            let h = b.bounds_half();
            max_extent = max_extent.max(h.x.max(h.y) * 2.0);
        }
        max_extent
    }

    /// Advance the simulation by exactly one fixed timestep.
    pub fn step(&mut self) {
        let dt = self.timestep.dt();

        // 1. Integrate velocities under gravity and accumulated forces.
        crate::physics::integrate_velocities(&mut self.world, self.gravity, dt);

        // 2. Continuous movement: sweep dynamic bodies against static geometry so
        //    fast bodies cannot tunnel through thin walls.
        self.integrate_with_ccd(dt);

        // 3. Discrete detection and impulse resolution for the rest.
        self.resolve_collisions();

        self.tick += 1;
    }

    fn static_bodies(&self) -> Vec<(Vec2, Vec2, f64)> {
        let mut out = Vec::new();
        for (e, rb) in self.world.query::<RigidBody>() {
            if !rb.is_static {
                continue;
            }
            if let (Some(t), Some(c)) = (
                self.world.get::<Transform>(e),
                self.world.get::<Collider>(e),
            ) {
                out.push((t.position, c.shape.bounds_half(), rb.restitution));
            }
        }
        out
    }

    fn integrate_with_ccd(&mut self, dt: f64) {
        let statics = self.static_bodies();
        let dynamics: Vec<Entity> = self
            .world
            .query::<RigidBody>()
            .filter(|(_, rb)| !rb.is_static)
            .map(|(e, _)| e)
            .collect();

        for e in dynamics {
            let (pos, vel, moving_half, shape, rest) = match (
                self.world.get::<Transform>(e),
                self.world.get::<Velocity>(e),
                self.world.get::<Collider>(e),
                self.world.get::<RigidBody>(e),
            ) {
                (Some(t), Some(v), Some(c), Some(rb)) => {
                    (t.position, v.0, c.shape.bounds_half(), c.shape, rb.restitution)
                }
                _ => continue,
            };

            // A body that already overlaps a static must not deepen the overlap
            // with its own displacement. The swept test below skips pre-existing
            // overlap by design (t_enter <= 0 hands the contact to discrete
            // resolution), so without this guard a body pushed into a wall by a
            // pile, or spawned inside one, crosses the whole wall on the next
            // step at extreme speed. Cancel the inward velocity component
            // against every static currently touching the body, in deterministic
            // order, leaving tangential slide until positional correction
            // depenetrates the contact.
            let mut vel = vel;
            for &(sc, sh, srest) in &statics {
                let body = BodyView { center: pos, shape };
                let stat = BodyView {
                    center: sc,
                    shape: Shape::Aabb { half: sh },
                };
                if let Some(m) = collision::collide(&body, &stat) {
                    // The manifold normal points from the body toward the
                    // static, so a positive component means still moving in.
                    let vn = vel.dot(m.normal);
                    if vn > 0.0 {
                        let e_rest = rest.min(srest);
                        vel -= m.normal * ((1.0 + e_rest) * vn);
                    }
                }
            }

            let disp = vel * dt;
            let mut earliest: Option<(f64, Vec2, f64)> = None;
            for &(sc, sh, srest) in &statics {
                if let Some((toi, normal)) =
                    collision::swept_aabb(pos, moving_half, disp, sc, sh)
                {
                    if earliest.map(|(t, _, _)| toi < t).unwrap_or(true) {
                        earliest = Some((toi, normal, srest));
                    }
                }
            }

            let (new_pos, new_vel) = if let Some((toi, normal, srest)) = earliest {
                // Move to the contact point and reflect the normal velocity.
                let contact = pos + disp * toi;
                let vn = vel.dot(normal);
                let e_rest = rest.min(srest);
                let reflected = if vn < 0.0 {
                    vel - normal * ((1.0 + e_rest) * vn)
                } else {
                    vel
                };
                (contact, reflected)
            } else {
                (pos + disp, vel)
            };

            if let Some(t) = self.world.get_mut::<Transform>(e) {
                t.position = new_pos;
            }
            if let Some(v) = self.world.get_mut::<Velocity>(e) {
                v.0 = new_vel;
            }
        }
    }

    fn resolve_collisions(&mut self) {
        // Snapshot collidable bodies in deterministic index order.
        let mut ents: Vec<Entity> = Vec::new();
        let mut bodies: Vec<BodyView> = Vec::new();
        for e in self.world.entities_with::<Collider>() {
            let t = self.world.get::<Transform>(e).copied().unwrap_or_default();
            let c = *self.world.get::<Collider>(e).unwrap();
            ents.push(e);
            bodies.push(BodyView {
                center: t.position,
                shape: c.shape,
            });
        }

        let cell = Self::cell_size(&bodies);
        let mut pairs: Vec<(usize, usize)> =
            collision::detect_pairs(&bodies, cell).into_iter().collect();
        pairs.sort_unstable();

        for (i, j) in pairs {
            let ea = ents[i];
            let eb = ents[j];
            let rba = *self.world.get::<RigidBody>(ea).unwrap();
            let rbb = *self.world.get::<RigidBody>(eb).unwrap();
            let inv_sum = rba.inv_mass + rbb.inv_mass;
            if inv_sum == 0.0 {
                continue;
            }

            // Recompute the manifold from live positions.
            let ca = self.world.get::<Transform>(ea).unwrap().position;
            let cb = self.world.get::<Transform>(eb).unwrap().position;
            let a_view = BodyView {
                center: ca,
                shape: self.world.get::<Collider>(ea).unwrap().shape,
            };
            let b_view = BodyView {
                center: cb,
                shape: self.world.get::<Collider>(eb).unwrap().shape,
            };
            let manifold = match collision::collide(&a_view, &b_view) {
                Some(m) => m,
                None => continue,
            };
            let n = manifold.normal;

            let va = self
                .world
                .get::<Velocity>(ea)
                .map(|v| v.0)
                .unwrap_or(Vec2::ZERO);
            let vb = self
                .world
                .get::<Velocity>(eb)
                .map(|v| v.0)
                .unwrap_or(Vec2::ZERO);

            // Impulse along the contact normal.
            let rv = vb - va;
            let vn = rv.dot(n);
            if vn < 0.0 {
                let e_rest = rba.restitution.min(rbb.restitution);
                let jimp = -(1.0 + e_rest) * vn / inv_sum;
                let impulse = n * jimp;
                if let Some(v) = self.world.get_mut::<Velocity>(ea) {
                    v.0 -= impulse * rba.inv_mass;
                }
                if let Some(v) = self.world.get_mut::<Velocity>(eb) {
                    v.0 += impulse * rbb.inv_mass;
                }
            }

            // Positional correction to remove residual penetration.
            let corr_mag = (manifold.penetration - PENETRATION_SLOP).max(0.0) / inv_sum
                * POSITION_CORRECTION;
            let correction = n * corr_mag;
            if let Some(t) = self.world.get_mut::<Transform>(ea) {
                t.position -= correction * rba.inv_mass;
            }
            if let Some(t) = self.world.get_mut::<Transform>(eb) {
                t.position += correction * rbb.inv_mass;
            }
        }
    }

    /// Run `steps` fixed steps, applying any script commands scheduled for each
    /// tick before that tick runs. The script need not be sorted.
    pub fn run(&mut self, steps: u64, script: &[ScriptEntry]) {
        for _ in 0..steps {
            for &(tick, cmd) in script {
                if tick == self.tick {
                    self.apply(cmd);
                }
            }
            self.step();
        }
    }

    /// Canonical serialization of the entire simulation state.
    pub fn serialize(&self) -> Vec<u8> {
        let mut out = Vec::new();
        self.tick.write(&mut out);
        self.gravity.write(&mut out);
        self.timestep.write(&mut out);
        self.rng.write(&mut out);
        self.world.serialize(&mut out);
        out
    }

    /// Restore state written by [`Simulation::serialize`]. Components are already
    /// registered by [`Simulation::empty`].
    pub fn deserialize(&mut self, bytes: &[u8]) -> Result<(), DecodeError> {
        let mut cur = Cursor::new(bytes);
        self.tick = u64::read(&mut cur)?;
        self.gravity = Vec2::read(&mut cur)?;
        self.timestep = FixedTimestep::read(&mut cur)?;
        self.rng = Rng::read(&mut cur)?;
        self.world.deserialize(&mut cur)?;
        Ok(())
    }

    /// FNV-1a hash of the canonical serialization. Two simulations with the same
    /// hash are bit-for-bit identical.
    pub fn hash(&self) -> u64 {
        hash_bytes(&self.serialize())
    }
}

#[cfg(test)]
mod tests {
    use super::*;

    fn demo(seed: u64, balls: u32) -> Simulation {
        let config = SimConfig::default();
        let mut sim = Simulation::new(config, seed);
        sim.seed_scene(config, balls);
        sim
    }

    #[test]
    fn same_seed_same_hash_after_running() {
        let mut a = demo(42, 20);
        let mut b = demo(42, 20);
        a.run(300, &[]);
        b.run(300, &[]);
        assert_eq!(a.hash(), b.hash());
    }

    #[test]
    fn different_seed_diverges() {
        let mut a = demo(1, 20);
        let mut b = demo(2, 20);
        a.run(300, &[]);
        b.run(300, &[]);
        assert_ne!(a.hash(), b.hash());
    }

    #[test]
    fn serialize_then_continue_matches() {
        let mut original = demo(7, 15);
        original.run(120, &[]);

        // Snapshot, restore into a fresh sim, then run both forward.
        let snapshot = original.serialize();
        let mut restored = Simulation::empty(SimConfig::default());
        restored.deserialize(&snapshot).unwrap();
        assert_eq!(original.hash(), restored.hash());

        original.run(120, &[]);
        restored.run(120, &[]);
        assert_eq!(original.hash(), restored.hash());
    }

    #[test]
    fn balls_stay_inside_walls() {
        let mut sim = demo(99, 30);
        sim.run(600, &[]);
        let min = SimConfig::default().bounds_min;
        let max = SimConfig::default().bounds_max;
        for (e, _) in sim.world.query::<Velocity>() {
            let p = sim.world.get::<Transform>(e).unwrap().position;
            // Allow a small margin for penetration slop.
            assert!(p.x > min.x - 5.0 && p.x < max.x + 5.0, "x out of bounds: {p:?}");
            assert!(p.y > min.y - 5.0 && p.y < max.y + 5.0, "y out of bounds: {p:?}");
        }
    }

    #[test]
    fn scripted_command_changes_outcome() {
        let mut base = demo(5, 10);
        let mut nudged = demo(5, 10);
        let target = nudged.world.entities_with::<Velocity>()[0];
        base.run(200, &[]);
        nudged.run(
            200,
            &[(50, Command::Impulse { entity: target, delta_v: Vec2::new(100.0, 0.0) })],
        );
        assert_ne!(base.hash(), nudged.hash());
    }
}
