//! 2D math: vectors and affine transforms.

use crate::serialize::{ByteIo, Cursor, DecodeError};
use std::ops::{Add, AddAssign, Div, Mul, Neg, Sub, SubAssign};

/// A 2D vector using f64 components.
#[derive(Clone, Copy, Debug, PartialEq, Default)]
pub struct Vec2 {
    pub x: f64,
    pub y: f64,
}

pub const fn vec2(x: f64, y: f64) -> Vec2 {
    Vec2 { x, y }
}

impl Vec2 {
    pub const ZERO: Vec2 = Vec2 { x: 0.0, y: 0.0 };
    pub const ONE: Vec2 = Vec2 { x: 1.0, y: 1.0 };

    pub const fn new(x: f64, y: f64) -> Self {
        Vec2 { x, y }
    }

    pub const fn splat(v: f64) -> Self {
        Vec2 { x: v, y: v }
    }

    pub fn dot(self, o: Vec2) -> f64 {
        self.x * o.x + self.y * o.y
    }

    /// 2D scalar cross product (z component of the 3D cross).
    pub fn cross(self, o: Vec2) -> f64 {
        self.x * o.y - self.y * o.x
    }

    pub fn length_sq(self) -> f64 {
        self.dot(self)
    }

    pub fn length(self) -> f64 {
        self.length_sq().sqrt()
    }

    pub fn distance_sq(self, o: Vec2) -> f64 {
        (self - o).length_sq()
    }

    pub fn distance(self, o: Vec2) -> f64 {
        (self - o).length()
    }

    /// Returns the unit vector, or zero if the length is zero.
    pub fn normalized(self) -> Vec2 {
        let len = self.length();
        if len == 0.0 {
            Vec2::ZERO
        } else {
            self / len
        }
    }

    /// Perpendicular vector rotated 90 degrees counter-clockwise.
    pub fn perp(self) -> Vec2 {
        Vec2::new(-self.y, self.x)
    }

    pub fn min(self, o: Vec2) -> Vec2 {
        Vec2::new(self.x.min(o.x), self.y.min(o.y))
    }

    pub fn max(self, o: Vec2) -> Vec2 {
        Vec2::new(self.x.max(o.x), self.y.max(o.y))
    }

    pub fn abs(self) -> Vec2 {
        Vec2::new(self.x.abs(), self.y.abs())
    }

    /// Linear interpolation. t is not clamped.
    pub fn lerp(self, o: Vec2, t: f64) -> Vec2 {
        self + (o - self) * t
    }
}

impl Add for Vec2 {
    type Output = Vec2;
    fn add(self, o: Vec2) -> Vec2 {
        Vec2::new(self.x + o.x, self.y + o.y)
    }
}

impl Sub for Vec2 {
    type Output = Vec2;
    fn sub(self, o: Vec2) -> Vec2 {
        Vec2::new(self.x - o.x, self.y - o.y)
    }
}

impl Neg for Vec2 {
    type Output = Vec2;
    fn neg(self) -> Vec2 {
        Vec2::new(-self.x, -self.y)
    }
}

impl Mul<f64> for Vec2 {
    type Output = Vec2;
    fn mul(self, s: f64) -> Vec2 {
        Vec2::new(self.x * s, self.y * s)
    }
}

impl Mul<Vec2> for f64 {
    type Output = Vec2;
    fn mul(self, v: Vec2) -> Vec2 {
        v * self
    }
}

impl Div<f64> for Vec2 {
    type Output = Vec2;
    fn div(self, s: f64) -> Vec2 {
        Vec2::new(self.x / s, self.y / s)
    }
}

impl AddAssign for Vec2 {
    fn add_assign(&mut self, o: Vec2) {
        *self = *self + o;
    }
}

impl SubAssign for Vec2 {
    fn sub_assign(&mut self, o: Vec2) {
        *self = *self - o;
    }
}

impl ByteIo for Vec2 {
    fn write(&self, out: &mut Vec<u8>) {
        self.x.write(out);
        self.y.write(out);
    }
    fn read(cur: &mut Cursor) -> Result<Self, DecodeError> {
        Ok(Vec2::new(f64::read(cur)?, f64::read(cur)?))
    }
}

/// An affine transform: translation, rotation (radians), and uniform-per-axis scale.
#[derive(Clone, Copy, Debug, PartialEq)]
pub struct Transform {
    pub position: Vec2,
    pub rotation: f64,
    pub scale: Vec2,
}

impl Default for Transform {
    fn default() -> Self {
        Transform {
            position: Vec2::ZERO,
            rotation: 0.0,
            scale: Vec2::ONE,
        }
    }
}

impl Transform {
    pub fn from_position(position: Vec2) -> Self {
        Transform {
            position,
            ..Default::default()
        }
    }

    /// Transform a point from local space into the space this transform lives in.
    pub fn apply(&self, local: Vec2) -> Vec2 {
        let scaled = Vec2::new(local.x * self.scale.x, local.y * self.scale.y);
        let (s, c) = self.rotation.sin_cos();
        let rotated = Vec2::new(scaled.x * c - scaled.y * s, scaled.x * s + scaled.y * c);
        rotated + self.position
    }

    /// Compose two transforms. `self` is the parent, `child` is expressed in the
    /// parent's local space. The result places the child in the parent's space.
    pub fn combine(&self, child: &Transform) -> Transform {
        let (s, c) = self.rotation.sin_cos();
        let cs = Vec2::new(child.position.x * self.scale.x, child.position.y * self.scale.y);
        let rotated = Vec2::new(cs.x * c - cs.y * s, cs.x * s + cs.y * c);
        Transform {
            position: self.position + rotated,
            rotation: self.rotation + child.rotation,
            scale: Vec2::new(self.scale.x * child.scale.x, self.scale.y * child.scale.y),
        }
    }
}

impl ByteIo for Transform {
    fn write(&self, out: &mut Vec<u8>) {
        self.position.write(out);
        self.rotation.write(out);
        self.scale.write(out);
    }
    fn read(cur: &mut Cursor) -> Result<Self, DecodeError> {
        Ok(Transform {
            position: Vec2::read(cur)?,
            rotation: f64::read(cur)?,
            scale: Vec2::read(cur)?,
        })
    }
}

#[cfg(test)]
mod tests {
    use super::*;

    const EPS: f64 = 1e-12;

    fn close(a: Vec2, b: Vec2) -> bool {
        a.distance(b) < 1e-9
    }

    #[test]
    fn arithmetic() {
        let a = vec2(1.0, 2.0);
        let b = vec2(3.0, 4.0);
        assert_eq!(a + b, vec2(4.0, 6.0));
        assert_eq!(b - a, vec2(2.0, 2.0));
        assert_eq!(a * 2.0, vec2(2.0, 4.0));
        assert_eq!(2.0 * a, vec2(2.0, 4.0));
        assert_eq!(-a, vec2(-1.0, -2.0));
    }

    #[test]
    fn dot_and_cross() {
        let a = vec2(1.0, 0.0);
        let b = vec2(0.0, 1.0);
        assert!((a.dot(b)).abs() < EPS);
        assert!((a.cross(b) - 1.0).abs() < EPS);
        assert!((a.dot(a) - 1.0).abs() < EPS);
    }

    #[test]
    fn length_and_normalize() {
        let v = vec2(3.0, 4.0);
        assert!((v.length() - 5.0).abs() < EPS);
        assert!((v.length_sq() - 25.0).abs() < EPS);
        assert!((v.normalized().length() - 1.0).abs() < EPS);
        assert_eq!(Vec2::ZERO.normalized(), Vec2::ZERO);
    }

    #[test]
    fn perp_is_orthogonal() {
        let v = vec2(2.0, -5.0);
        assert!(v.dot(v.perp()).abs() < EPS);
    }

    #[test]
    fn lerp_endpoints() {
        let a = vec2(0.0, 0.0);
        let b = vec2(10.0, 20.0);
        assert!(close(a.lerp(b, 0.0), a));
        assert!(close(a.lerp(b, 1.0), b));
        assert!(close(a.lerp(b, 0.5), vec2(5.0, 10.0)));
    }

    #[test]
    fn transform_identity_apply() {
        let t = Transform::default();
        assert!(close(t.apply(vec2(3.0, 7.0)), vec2(3.0, 7.0)));
    }

    #[test]
    fn transform_translation_and_rotation() {
        let t = Transform {
            position: vec2(1.0, 1.0),
            rotation: std::f64::consts::FRAC_PI_2,
            scale: Vec2::ONE,
        };
        // Rotating (1,0) by 90 degrees gives (0,1), then translate by (1,1).
        assert!(close(t.apply(vec2(1.0, 0.0)), vec2(1.0, 2.0)));
    }

    #[test]
    fn transform_combine_composes_translation() {
        let parent = Transform::from_position(vec2(10.0, 0.0));
        let child = Transform::from_position(vec2(0.0, 5.0));
        let world = parent.combine(&child);
        assert!(close(world.position, vec2(10.0, 5.0)));
    }
}
