//! Semi-implicit Euler integration.
//!
//! Semi-implicit (a.k.a. symplectic) Euler updates velocity first, then uses the
//! new velocity to update position. It is stable for the spring-like restoring
//! behaviour of collision resolution and, unlike explicit Euler, does not pump
//! energy into a system under constant force.

use crate::components::{Forces, RigidBody, Velocity};
use crate::ecs::World;
use crate::math::{Transform, Vec2};

/// Pure semi-implicit Euler step for a single body. Returns the new
/// `(position, velocity)`.
pub fn semi_implicit(pos: Vec2, vel: Vec2, accel: Vec2, dt: f64) -> (Vec2, Vec2) {
    let new_vel = vel + accel * dt;
    let new_pos = pos + new_vel * dt;
    (new_pos, new_vel)
}

/// Integrate velocities for every dynamic body: apply gravity and accumulated
/// forces, then clear the force accumulator. Position is advanced separately so
/// that continuous collision can clamp the movement.
pub fn integrate_velocities(world: &mut World, gravity: Vec2, dt: f64) {
    let bodies: Vec<_> = world
        .query::<RigidBody>()
        .filter(|(_, rb)| !rb.is_static)
        .map(|(e, rb)| (e, *rb))
        .collect();

    for (e, rb) in bodies {
        let force = world.get::<Forces>(e).map_or(Vec2::ZERO, |f| f.0);
        let accel = gravity + force * rb.inv_mass;
        if let Some(vel) = world.get_mut::<Velocity>(e) {
            vel.0 += accel * dt;
        }
        if let Some(f) = world.get_mut::<Forces>(e) {
            f.0 = Vec2::ZERO;
        }
    }
}

/// Advance positions by `velocity * dt` for every dynamic body that has a
/// `Transform` and `Velocity`. Used when continuous collision is not needed.
pub fn integrate_positions(world: &mut World, dt: f64) {
    let moving: Vec<_> = world
        .query::<RigidBody>()
        .filter(|(_, rb)| !rb.is_static)
        .map(|(e, _)| e)
        .collect();
    for e in moving {
        let v = world.get::<Velocity>(e).map_or(Vec2::ZERO, |v| v.0);
        if let Some(t) = world.get_mut::<Transform>(e) {
            t.position += v * dt;
        }
    }
}

#[cfg(test)]
mod tests {
    use super::*;
    use crate::math::vec2;

    #[test]
    fn free_fall_matches_closed_form_reasonably() {
        // Under constant gravity, semi-implicit Euler tracks the analytic drop
        // closely for small dt.
        let g = vec2(0.0, -9.81);
        let dt = 1.0 / 240.0;
        let mut pos = Vec2::ZERO;
        let mut vel = Vec2::ZERO;
        let mut t = 0.0;
        for _ in 0..240 {
            let (p, v) = semi_implicit(pos, vel, g, dt);
            pos = p;
            vel = v;
            t += dt;
        }
        // Velocity after one second is exactly g * t for constant acceleration.
        assert!((vel.y - (-9.81 * t)).abs() < 1e-6);
        // Position is close to the analytic 0.5 g t^2, within one step of error.
        let analytic = 0.5 * -9.81 * t * t;
        assert!((pos.y - analytic).abs() < 0.05);
    }

    #[test]
    fn velocity_first_then_position() {
        // With zero initial velocity and unit accel over dt=1, position should
        // move by the NEW velocity (1), not the old (0). That is the semi-
        // implicit property.
        let (p, v) = semi_implicit(Vec2::ZERO, Vec2::ZERO, vec2(1.0, 0.0), 1.0);
        assert_eq!(v, vec2(1.0, 0.0));
        assert_eq!(p, vec2(1.0, 0.0));
    }
}
