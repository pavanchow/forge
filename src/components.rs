//! The built-in simulation components.
//!
//! These are the concrete component types the physics and collision systems
//! operate on. They all implement [`ByteIo`] so the whole world can be
//! serialized and hashed.

use crate::math::Vec2;
use crate::serialize::{ByteIo, Cursor, DecodeError};

/// Linear velocity in units per second.
#[derive(Clone, Copy, Debug, PartialEq, Default)]
pub struct Velocity(pub Vec2);

impl ByteIo for Velocity {
    fn write(&self, out: &mut Vec<u8>) {
        self.0.write(out);
    }
    fn read(cur: &mut Cursor) -> Result<Self, DecodeError> {
        Ok(Velocity(Vec2::read(cur)?))
    }
}

/// A force accumulator, cleared each step after integration.
#[derive(Clone, Copy, Debug, PartialEq, Default)]
pub struct Forces(pub Vec2);

impl ByteIo for Forces {
    fn write(&self, out: &mut Vec<u8>) {
        self.0.write(out);
    }
    fn read(cur: &mut Cursor) -> Result<Self, DecodeError> {
        Ok(Forces(Vec2::read(cur)?))
    }
}

/// Physical body properties. A static body has `inv_mass == 0` and never moves.
#[derive(Clone, Copy, Debug, PartialEq)]
pub struct RigidBody {
    pub inv_mass: f64,
    pub restitution: f64,
    pub is_static: bool,
}

impl RigidBody {
    pub fn dynamic(mass: f64, restitution: f64) -> Self {
        RigidBody {
            inv_mass: if mass > 0.0 { 1.0 / mass } else { 0.0 },
            restitution,
            is_static: false,
        }
    }

    pub fn fixed(restitution: f64) -> Self {
        RigidBody {
            inv_mass: 0.0,
            restitution,
            is_static: true,
        }
    }
}

impl ByteIo for RigidBody {
    fn write(&self, out: &mut Vec<u8>) {
        self.inv_mass.write(out);
        self.restitution.write(out);
        self.is_static.write(out);
    }
    fn read(cur: &mut Cursor) -> Result<Self, DecodeError> {
        Ok(RigidBody {
            inv_mass: f64::read(cur)?,
            restitution: f64::read(cur)?,
            is_static: bool::read(cur)?,
        })
    }
}

/// Collision shape. Position comes from the entity's `Transform`.
#[derive(Clone, Copy, Debug, PartialEq)]
pub enum Shape {
    Aabb { half: Vec2 },
    Circle { radius: f64 },
}

impl Shape {
    /// Half-extents of the axis-aligned bounding box of this shape.
    pub fn bounds_half(&self) -> Vec2 {
        match self {
            Shape::Aabb { half } => *half,
            Shape::Circle { radius } => Vec2::splat(*radius),
        }
    }
}

impl ByteIo for Shape {
    fn write(&self, out: &mut Vec<u8>) {
        match self {
            Shape::Aabb { half } => {
                0u8.write(out);
                half.write(out);
            }
            Shape::Circle { radius } => {
                1u8.write(out);
                radius.write(out);
            }
        }
    }
    fn read(cur: &mut Cursor) -> Result<Self, DecodeError> {
        match u8::read(cur)? {
            0 => Ok(Shape::Aabb {
                half: Vec2::read(cur)?,
            }),
            1 => Ok(Shape::Circle {
                radius: f64::read(cur)?,
            }),
            t => Err(DecodeError::BadTag(t)),
        }
    }
}

/// A collider is a shape plus a layer tag for coarse filtering.
#[derive(Clone, Copy, Debug, PartialEq)]
pub struct Collider {
    pub shape: Shape,
}

impl Collider {
    pub fn aabb(half: Vec2) -> Self {
        Collider {
            shape: Shape::Aabb { half },
        }
    }
    pub fn circle(radius: f64) -> Self {
        Collider {
            shape: Shape::Circle { radius },
        }
    }
}

impl ByteIo for Collider {
    fn write(&self, out: &mut Vec<u8>) {
        self.shape.write(out);
    }
    fn read(cur: &mut Cursor) -> Result<Self, DecodeError> {
        Ok(Collider {
            shape: Shape::read(cur)?,
        })
    }
}

/// A parent link for the scene hierarchy. The entity's `Transform` is treated as
/// local to the parent.
#[derive(Clone, Copy, Debug, PartialEq)]
pub struct Parent(pub crate::ecs::Entity);

impl ByteIo for Parent {
    fn write(&self, out: &mut Vec<u8>) {
        self.0.index.write(out);
        self.0.generation.write(out);
    }
    fn read(cur: &mut Cursor) -> Result<Self, DecodeError> {
        Ok(Parent(crate::ecs::Entity {
            index: u32::read(cur)?,
            generation: u32::read(cur)?,
        }))
    }
}

#[cfg(test)]
mod tests {
    use super::*;
    use crate::math::vec2;

    #[test]
    fn rigidbody_helpers() {
        let d = RigidBody::dynamic(2.0, 0.5);
        assert_eq!(d.inv_mass, 0.5);
        assert!(!d.is_static);
        let s = RigidBody::fixed(1.0);
        assert_eq!(s.inv_mass, 0.0);
        assert!(s.is_static);
        // Zero mass is treated as infinite (static-like) mass.
        assert_eq!(RigidBody::dynamic(0.0, 0.0).inv_mass, 0.0);
    }

    #[test]
    fn shape_bounds() {
        assert_eq!(Shape::Circle { radius: 3.0 }.bounds_half(), vec2(3.0, 3.0));
        assert_eq!(
            Shape::Aabb { half: vec2(2.0, 5.0) }.bounds_half(),
            vec2(2.0, 5.0)
        );
    }
}
