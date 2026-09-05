//! Rendering abstraction.
//!
//! The engine core never talks to a graphics API. It emits draw commands through
//! the `Renderer` trait, so the same simulation can drive a real backend, a
//! no-op sink in headless CI, or a recorder in tests. Keeping rendering behind a
//! trait is what makes the whole engine headless-testable.

use crate::components::{Collider, Shape};
use crate::ecs::World;
use crate::math::{Transform, Vec2};

/// An RGBA color with components in [0, 1].
#[derive(Clone, Copy, Debug, PartialEq)]
pub struct Color {
    pub r: f64,
    pub g: f64,
    pub b: f64,
    pub a: f64,
}

impl Color {
    pub const WHITE: Color = Color {
        r: 1.0,
        g: 1.0,
        b: 1.0,
        a: 1.0,
    };
    pub fn rgb(r: f64, g: f64, b: f64) -> Self {
        Color { r, g, b, a: 1.0 }
    }
}

/// A single drawing instruction.
#[derive(Clone, Copy, Debug, PartialEq)]
pub enum DrawCommand {
    Circle {
        center: Vec2,
        radius: f64,
        color: Color,
    },
    Rect {
        center: Vec2,
        half: Vec2,
        color: Color,
    },
}

/// A rendering backend. Implement this to draw the world however you like.
pub trait Renderer {
    fn begin_frame(&mut self) {}
    fn submit(&mut self, cmd: DrawCommand);
    fn end_frame(&mut self) {}
}

/// Discards everything. Use in headless runs where nothing should be drawn.
#[derive(Clone, Copy, Debug, Default)]
pub struct NoopRenderer;

impl Renderer for NoopRenderer {
    fn submit(&mut self, _cmd: DrawCommand) {}
}

/// Records every draw command for inspection in tests.
#[derive(Clone, Debug, Default)]
pub struct RecordingRenderer {
    pub commands: Vec<DrawCommand>,
    pub frames: u32,
}

impl RecordingRenderer {
    pub fn new() -> Self {
        Self::default()
    }
}

impl Renderer for RecordingRenderer {
    fn begin_frame(&mut self) {
        self.commands.clear();
    }
    fn submit(&mut self, cmd: DrawCommand) {
        self.commands.push(cmd);
    }
    fn end_frame(&mut self) {
        self.frames += 1;
    }
}

/// Walk every entity that has a `Transform` and `Collider` and emit a draw
/// command for its shape, in deterministic entity order.
pub fn render_world<R: Renderer>(world: &World, renderer: &mut R, color: Color) {
    renderer.begin_frame();
    for e in world.entities_with::<Collider>() {
        let transform = world.get::<Transform>(e).copied().unwrap_or_default();
        let collider = world.get::<Collider>(e).copied().unwrap();
        match collider.shape {
            Shape::Circle { radius } => renderer.submit(DrawCommand::Circle {
                center: transform.position,
                radius,
                color,
            }),
            Shape::Aabb { half } => renderer.submit(DrawCommand::Rect {
                center: transform.position,
                half,
                color,
            }),
        }
    }
    renderer.end_frame();
}

#[cfg(test)]
mod tests {
    use super::*;
    use crate::math::vec2;

    fn make_world() -> World {
        let mut w = World::new();
        w.register::<Transform>();
        w.register::<Collider>();
        let a = w.spawn();
        w.insert(a, Transform::from_position(vec2(1.0, 2.0)));
        w.insert(a, Collider::circle(0.5));
        let b = w.spawn();
        w.insert(b, Transform::from_position(vec2(3.0, 4.0)));
        w.insert(b, Collider::aabb(vec2(1.0, 1.0)));
        w
    }

    #[test]
    fn recording_captures_all_shapes() {
        let w = make_world();
        let mut r = RecordingRenderer::new();
        render_world(&w, &mut r, Color::WHITE);
        assert_eq!(r.commands.len(), 2);
        assert_eq!(r.frames, 1);
        assert!(matches!(r.commands[0], DrawCommand::Circle { .. }));
        assert!(matches!(r.commands[1], DrawCommand::Rect { .. }));
    }

    #[test]
    fn noop_renderer_is_headless() {
        let w = make_world();
        let mut r = NoopRenderer;
        // Simply must not panic and must compile as a Renderer.
        render_world(&w, &mut r, Color::WHITE);
    }
}
