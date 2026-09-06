//! Collision detection and continuous (swept) tests.
//!
//! Narrowphase covers AABB and circle shapes in every pairing. Broadphase is a
//! uniform spatial grid that is guaranteed to surface every truly overlapping
//! pair (it inserts each body into every cell its bounding box touches), so the
//! candidate set is always a superset of the real overlaps. Gate 3 checks that
//! broadphase plus narrowphase produces exactly the brute-force O(n^2) overlap
//! set. Continuous collision uses a swept AABB slab test to stop fast bodies
//! before they tunnel through thin static geometry.

use crate::components::Shape;
use crate::math::Vec2;
use std::collections::HashSet;

/// A resolved contact between two shapes.
#[derive(Clone, Copy, Debug, PartialEq)]
pub struct Manifold {
    /// Unit separation axis pointing from A toward B.
    pub normal: Vec2,
    /// Overlap depth along `normal`, always non-negative.
    pub penetration: f64,
}

/// A body as the collision system sees it: a world-space center and a shape.
#[derive(Clone, Copy, Debug)]
pub struct BodyView {
    pub center: Vec2,
    pub shape: Shape,
}

impl BodyView {
    pub fn bounds_half(&self) -> Vec2 {
        self.shape.bounds_half()
    }
}

/// A body participates in collision only if its center is finite and its
/// extents are finite and non-negative. Non-finite state must never propagate
/// into a manifold, and an inverted (negative half-extent) box is not a body:
/// the broadphase already skips it because its cell range iterates zero cells,
/// so narrowphase and the sweep reject it too and all three entry points stay
/// consistent.
fn is_valid_view(b: &BodyView) -> bool {
    if !b.center.x.is_finite() || !b.center.y.is_finite() {
        return false;
    }
    match b.shape {
        Shape::Aabb { half } => {
            half.x.is_finite() && half.y.is_finite() && half.x >= 0.0 && half.y >= 0.0
        }
        Shape::Circle { radius } => radius.is_finite() && radius >= 0.0,
    }
}

fn aabb_vs_aabb(ca: Vec2, ha: Vec2, cb: Vec2, hb: Vec2) -> Option<Manifold> {
    let d = cb - ca;
    let ox = ha.x + hb.x - d.x.abs();
    if ox <= 0.0 {
        return None;
    }
    let oy = ha.y + hb.y - d.y.abs();
    if oy <= 0.0 {
        return None;
    }
    // Resolve along the axis of least penetration.
    if ox < oy {
        let sign = if d.x < 0.0 { -1.0 } else { 1.0 };
        Some(Manifold {
            normal: Vec2::new(sign, 0.0),
            penetration: ox,
        })
    } else {
        let sign = if d.y < 0.0 { -1.0 } else { 1.0 };
        Some(Manifold {
            normal: Vec2::new(0.0, sign),
            penetration: oy,
        })
    }
}

fn circle_vs_circle(ca: Vec2, ra: f64, cb: Vec2, rb: f64) -> Option<Manifold> {
    let d = cb - ca;
    let r = ra + rb;
    let dist_sq = d.length_sq();
    if dist_sq >= r * r {
        return None;
    }
    let dist = dist_sq.sqrt();
    let normal = if dist > 0.0 {
        d / dist
    } else {
        // Concentric circles: pick a stable, deterministic axis.
        Vec2::new(1.0, 0.0)
    };
    Some(Manifold {
        normal,
        penetration: r - dist,
    })
}

fn aabb_vs_circle(ca: Vec2, ha: Vec2, cc: Vec2, r: f64) -> Option<Manifold> {
    // Closest point on the box to the circle center.
    let clamped = Vec2::new(
        cc.x.clamp(ca.x - ha.x, ca.x + ha.x),
        cc.y.clamp(ca.y - ha.y, ca.y + ha.y),
    );
    let d = cc - clamped;
    let dist_sq = d.length_sq();
    if dist_sq > r * r {
        return None;
    }
    if dist_sq > 0.0 {
        // Circle center outside the box.
        let dist = dist_sq.sqrt();
        Some(Manifold {
            normal: d / dist,
            penetration: r - dist,
        })
    } else {
        // Center inside the box: push out along the nearest face.
        let dx = ha.x - (cc.x - ca.x).abs();
        let dy = ha.y - (cc.y - ca.y).abs();
        if dx < dy {
            let sign = if cc.x < ca.x { -1.0 } else { 1.0 };
            Some(Manifold {
                normal: Vec2::new(sign, 0.0),
                penetration: dx + r,
            })
        } else {
            let sign = if cc.y < ca.y { -1.0 } else { 1.0 };
            Some(Manifold {
                normal: Vec2::new(0.0, sign),
                penetration: dy + r,
            })
        }
    }
}

/// Narrowphase test between two bodies. Returns the contact manifold with the
/// normal pointing from `a` toward `b`, or `None` if they do not overlap.
///
/// Bodies with non-finite centers, non-finite extents, or negative extents
/// collide with nothing. There is no meaningful manifold at infinity, a NaN
/// manifold would poison resolution, and an inverted box is not a shape at
/// all, so the case is rejected before any arithmetic that could panic or
/// propagate it.
pub fn collide(a: &BodyView, b: &BodyView) -> Option<Manifold> {
    if !is_valid_view(a) || !is_valid_view(b) {
        return None;
    }
    match (a.shape, b.shape) {
        (Shape::Aabb { half: ha }, Shape::Aabb { half: hb }) => {
            aabb_vs_aabb(a.center, ha, b.center, hb)
        }
        (Shape::Circle { radius: ra }, Shape::Circle { radius: rb }) => {
            circle_vs_circle(a.center, ra, b.center, rb)
        }
        (Shape::Aabb { half }, Shape::Circle { radius }) => {
            aabb_vs_circle(a.center, half, b.center, radius)
        }
        (Shape::Circle { radius }, Shape::Aabb { half }) => {
            // Flip so the normal still points from a toward b.
            aabb_vs_circle(b.center, half, a.center, radius).map(|m| Manifold {
                normal: -m.normal,
                penetration: m.penetration,
            })
        }
    }
}

/// Brute-force reference: every overlapping pair, checked O(n^2). Used as the
/// ground truth the broadphase must match.
pub fn brute_force_pairs(bodies: &[BodyView]) -> HashSet<(usize, usize)> {
    let mut out = HashSet::new();
    for i in 0..bodies.len() {
        for j in (i + 1)..bodies.len() {
            if collide(&bodies[i], &bodies[j]).is_some() {
                out.insert((i, j));
            }
        }
    }
    out
}

/// Upper bound on how many grid cells one body may span per axis before it is
/// treated as giant and simply paired against every other body. A body this
/// large overlaps a huge fraction of the world anyway, so the fallback keeps
/// the candidate set a superset while bounding the loop.
const MAX_CELLS_PER_AXIS: i64 = 1 << 20;

/// Uniform-grid broadphase. Returns candidate index pairs (i < j), deduplicated.
/// Every overlapping pair is guaranteed to appear because a body is inserted
/// into every cell its bounding box touches.
///
/// Pathological bounds stay correct and bounded: a body with non-finite bounds
/// produces no candidates (narrowphase rejects non-finite geometry too), and a
/// finite body whose cell span exceeds [`MAX_CELLS_PER_AXIS`] per axis is paired
/// against everything instead of walking an unbounded cell range.
pub fn broadphase_pairs(bodies: &[BodyView], cell_size: f64) -> Vec<(usize, usize)> {
    use std::collections::HashMap;
    let cell = if cell_size > 0.0 { cell_size } else { 1.0 };
    let mut grid: HashMap<(i64, i64), Vec<usize>> = HashMap::new();
    let coord = |v: f64| (v / cell).floor() as i64;
    let mut giants: Vec<usize> = Vec::new();

    for (i, b) in bodies.iter().enumerate() {
        let h = b.bounds_half();
        let min = b.center - h;
        let max = b.center + h;
        if !(min.x.is_finite()
            && min.y.is_finite()
            && max.x.is_finite()
            && max.y.is_finite())
        {
            continue;
        }
        let (x0, x1) = (coord(min.x), coord(max.x));
        let (y0, y1) = (coord(min.y), coord(max.y));
        if x1.saturating_sub(x0) > MAX_CELLS_PER_AXIS || y1.saturating_sub(y0) > MAX_CELLS_PER_AXIS
        {
            giants.push(i);
            continue;
        }
        for cx in x0..=x1 {
            for cy in y0..=y1 {
                grid.entry((cx, cy)).or_default().push(i);
            }
        }
    }

    let mut pairs: HashSet<(usize, usize)> = HashSet::new();
    for members in grid.values() {
        for a in 0..members.len() {
            for b in (a + 1)..members.len() {
                let (i, j) = (members[a], members[b]);
                pairs.insert(if i < j { (i, j) } else { (j, i) });
            }
        }
    }
    for g in giants {
        for j in 0..bodies.len() {
            if j != g {
                pairs.insert(if g < j { (g, j) } else { (j, g) });
            }
        }
    }
    let mut out: Vec<(usize, usize)> = pairs.into_iter().collect();
    // Sort so downstream resolution order is deterministic.
    out.sort_unstable();
    out
}

/// The set of actually overlapping pairs, found via broadphase then narrowphase.
pub fn detect_pairs(bodies: &[BodyView], cell_size: f64) -> HashSet<(usize, usize)> {
    let mut out = HashSet::new();
    for (i, j) in broadphase_pairs(bodies, cell_size) {
        if collide(&bodies[i], &bodies[j]).is_some() {
            out.insert((i, j));
        }
    }
    out
}

/// Swept AABB against a static AABB. `disp` is the moving box's displacement
/// this step. Returns `(toi, normal)` where `toi` in [0, 1] is the fraction of
/// the displacement at first contact and `normal` is the surface normal to
/// cancel velocity against. Returns `None` if no contact occurs within the step.
pub fn swept_aabb(
    center: Vec2,
    half: Vec2,
    disp: Vec2,
    static_center: Vec2,
    static_half: Vec2,
) -> Option<(f64, Vec2)> {
    // An inverted or non-finite box is not a body: narrowphase rejects it and
    // the broadphase never emits it, so the sweep must not hit it either.
    let valid = |v: Vec2| v.x.is_finite() && v.y.is_finite() && v.x >= 0.0 && v.y >= 0.0;
    if !valid(half) || !valid(static_half) {
        return None;
    }

    // Minkowski-expand the static box by the moving half-extents, then treat the
    // moving box as a point (a ray from its center) against the expanded box.
    let bmin = static_center - (static_half + half);
    let bmax = static_center + (static_half + half);

    let mut t_enter = 0.0_f64;
    let mut t_exit = 1.0_f64;
    let mut normal = Vec2::ZERO;

    for axis in 0..2 {
        let (o, d, lo, hi) = if axis == 0 {
            (center.x, disp.x, bmin.x, bmax.x)
        } else {
            (center.y, disp.y, bmin.y, bmax.y)
        };

        if d.abs() < 1e-12 {
            // Parallel to this slab: if already outside it, no hit is possible.
            if o < lo || o > hi {
                return None;
            }
            continue;
        }

        let inv = 1.0 / d;
        let mut t1 = (lo - o) * inv;
        let mut t2 = (hi - o) * inv;
        let mut axis_normal = if axis == 0 {
            Vec2::new(-1.0, 0.0)
        } else {
            Vec2::new(0.0, -1.0)
        };
        if t1 > t2 {
            std::mem::swap(&mut t1, &mut t2);
            axis_normal = -axis_normal;
        }
        if t1 > t_enter {
            t_enter = t1;
            normal = axis_normal;
        }
        if t2 < t_exit {
            t_exit = t2;
        }
        if t_enter > t_exit {
            return None;
        }
    }

    if t_enter > 1.0 || t_exit < 0.0 {
        return None;
    }
    // Already overlapping before moving: let discrete resolution handle it.
    if t_enter <= 0.0 {
        return None;
    }
    Some((t_enter, normal))
}

#[cfg(test)]
mod tests {
    use super::*;
    use crate::math::vec2;
    use crate::prng::Rng;

    fn aabb(center: Vec2, hx: f64, hy: f64) -> BodyView {
        BodyView {
            center,
            shape: Shape::Aabb {
                half: vec2(hx, hy),
            },
        }
    }
    fn circle(center: Vec2, r: f64) -> BodyView {
        BodyView {
            center,
            shape: Shape::Circle { radius: r },
        }
    }

    #[test]
    fn aabb_overlap_and_separation() {
        let a = aabb(vec2(0.0, 0.0), 1.0, 1.0);
        let b = aabb(vec2(1.5, 0.0), 1.0, 1.0);
        let m = collide(&a, &b).unwrap();
        assert!((m.penetration - 0.5).abs() < 1e-9);
        assert_eq!(m.normal, vec2(1.0, 0.0));
        let far = aabb(vec2(5.0, 0.0), 1.0, 1.0);
        assert!(collide(&a, &far).is_none());
    }

    #[test]
    fn circle_overlap() {
        let a = circle(vec2(0.0, 0.0), 1.0);
        let b = circle(vec2(1.0, 0.0), 1.0);
        let m = collide(&a, &b).unwrap();
        assert!((m.penetration - 1.0).abs() < 1e-9);
        assert_eq!(m.normal, vec2(1.0, 0.0));
        assert!(collide(&a, &circle(vec2(3.0, 0.0), 1.0)).is_none());
    }

    #[test]
    fn aabb_circle_both_orders() {
        let box_ = aabb(vec2(0.0, 0.0), 1.0, 1.0);
        let c = circle(vec2(1.5, 0.0), 0.75);
        let m1 = collide(&box_, &c).unwrap();
        let m2 = collide(&c, &box_).unwrap();
        // Normals point opposite ways because a and b swap.
        assert!((m1.penetration - m2.penetration).abs() < 1e-9);
        assert_eq!(m1.normal, -m2.normal);
    }

    #[test]
    fn broadphase_matches_bruteforce_random() {
        let mut rng = Rng::new(0xB16B_00B5);
        for _ in 0..200 {
            let n = rng.range_u32(2, 40) as usize;
            let mut bodies = Vec::new();
            for _ in 0..n {
                let c = vec2(rng.range_f64(0.0, 50.0), rng.range_f64(0.0, 50.0));
                if rng.next_f64() < 0.5 {
                    bodies.push(circle(c, rng.range_f64(0.5, 3.0)));
                } else {
                    bodies.push(aabb(c, rng.range_f64(0.5, 3.0), rng.range_f64(0.5, 3.0)));
                }
            }
            let brute = brute_force_pairs(&bodies);
            let grid = detect_pairs(&bodies, 4.0);
            assert_eq!(brute, grid, "broadphase disagreed with brute force");
        }
    }

    #[test]
    fn swept_stops_fast_body() {
        // A box far to the left moving fast to the right hits a wall at x=10.
        let hit = swept_aabb(
            vec2(0.0, 0.0),
            vec2(0.5, 0.5),
            vec2(100.0, 0.0),
            vec2(10.0, 0.0),
            vec2(0.5, 5.0),
        );
        let (toi, normal) = hit.expect("should register a swept hit");
        assert!((0.0..=1.0).contains(&toi));
        assert_eq!(normal, vec2(-1.0, 0.0));
        // Contact happens when the right face of the mover meets the left face of
        // the wall: gap is 10 - 0.5 - 0.5 = 9 over a 100-unit sweep.
        assert!((toi - 0.09).abs() < 1e-9);
    }

    #[test]
    fn swept_misses_when_offset() {
        // Moving right but the wall is far above: no contact.
        let miss = swept_aabb(
            vec2(0.0, 0.0),
            vec2(0.5, 0.5),
            vec2(100.0, 0.0),
            vec2(10.0, 50.0),
            vec2(0.5, 1.0),
        );
        assert!(miss.is_none());
    }

    #[test]
    fn swept_ignores_preexisting_overlap() {
        // Already overlapping: swept returns None, discrete resolution handles it.
        let r = swept_aabb(
            vec2(10.0, 0.0),
            vec2(1.0, 1.0),
            vec2(1.0, 0.0),
            vec2(10.0, 0.0),
            vec2(1.0, 1.0),
        );
        assert!(r.is_none());
    }
}
