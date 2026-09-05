//! A minimal scene hierarchy.
//!
//! An entity's `Transform` is treated as local to its `Parent`. The world-space
//! transform is the composition of every transform from the root down. Resolving
//! it walks parent links, which is deterministic because it depends only on the
//! stored data, not on iteration order.

use crate::components::Parent;
use crate::ecs::{Entity, World};
use crate::math::Transform;

/// Compute the world-space transform of `entity` by composing its local
/// transform with all of its ancestors. Cycles are guarded with a depth cap.
pub fn world_transform(world: &World, entity: Entity) -> Transform {
    // Collect the chain from the entity up to the root.
    let mut chain = Vec::new();
    let mut current = Some(entity);
    let mut guard = 0;
    while let Some(e) = current {
        let local = world.get::<Transform>(e).copied().unwrap_or_default();
        chain.push(local);
        current = world.get::<Parent>(e).map(|p| p.0);
        guard += 1;
        if guard > 4096 {
            break;
        }
    }
    // Fold from the root down: root.combine(child).combine(grandchild)...
    let mut acc = chain.pop().unwrap_or_default();
    while let Some(local) = chain.pop() {
        acc = acc.combine(&local);
    }
    acc
}

#[cfg(test)]
mod tests {
    use super::*;
    use crate::math::{vec2, Vec2};

    fn setup() -> World {
        let mut w = World::new();
        w.register::<Transform>();
        w.register::<Parent>();
        w
    }

    #[test]
    fn root_transform_is_local() {
        let mut w = setup();
        let e = w.spawn();
        w.insert(e, Transform::from_position(vec2(3.0, 4.0)));
        let wt = world_transform(&w, e);
        assert_eq!(wt.position, vec2(3.0, 4.0));
    }

    #[test]
    fn child_composes_with_parent() {
        let mut w = setup();
        let parent = w.spawn();
        w.insert(parent, Transform::from_position(vec2(10.0, 0.0)));
        let child = w.spawn();
        w.insert(child, Transform::from_position(vec2(0.0, 5.0)));
        w.insert(child, Parent(parent));
        let wt = world_transform(&w, child);
        assert_eq!(wt.position, vec2(10.0, 5.0));
    }

    #[test]
    fn deep_chain_composes() {
        let mut w = setup();
        let a = w.spawn();
        w.insert(a, Transform::from_position(vec2(1.0, 0.0)));
        let b = w.spawn();
        w.insert(b, Transform::from_position(vec2(1.0, 0.0)));
        w.insert(b, Parent(a));
        let c = w.spawn();
        w.insert(c, Transform::from_position(vec2(1.0, 0.0)));
        w.insert(c, Parent(b));
        let wt = world_transform(&w, c);
        assert_eq!(wt.position, vec2(3.0, 0.0));
    }

    #[test]
    fn rotation_propagates() {
        let mut w = setup();
        let parent = w.spawn();
        w.insert(
            parent,
            Transform {
                position: Vec2::ZERO,
                rotation: std::f64::consts::FRAC_PI_2,
                scale: Vec2::ONE,
            },
        );
        let child = w.spawn();
        w.insert(child, Transform::from_position(vec2(1.0, 0.0)));
        w.insert(child, Parent(parent));
        let wt = world_transform(&w, child);
        // The child's local +x becomes world +y under a 90 degree parent rotation.
        assert!(wt.position.distance(vec2(0.0, 1.0)) < 1e-9);
    }
}
