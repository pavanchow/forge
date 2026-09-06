//! # Forge
//!
//! A from-scratch, dependency-free 2D game engine built around one idea: the
//! simulation is deterministic and headless-testable. Given a seed and an input
//! script, the world evolves identically on every run and on every machine of
//! the same architecture, and the entire world can be serialized, restored, and
//! continued without any drift.
//!
//! ## Modules
//! - [`math`] 2D vectors and affine transforms.
//! - [`prng`] a seeded `SplitMix64` generator (the standard library has no RNG).
//! - [`ecs`] a deterministic entity component system.
//! - [`components`] the built-in simulation components.
//! - [`time`] the fixed-timestep accumulator.
//! - [`physics`] semi-implicit Euler integration.
//! - [`collision`] AABB and circle detection, a grid broadphase, and swept CCD.
//! - [`input`] input state and a double-buffered event queue.
//! - [`scene`] a parent/child transform hierarchy.
//! - [`render`] a headless-friendly `Renderer` trait.
//! - [`serialize`] canonical binary encoding used for saves and hashing.
//! - [`hash`] FNV-1a over the canonical encoding.
//! - [`sim`] the `Simulation` that ties it all together.
//! - [`rollback`] snapshot ring for rollback-netcode style rewinds.
//!
//! ## Quickstart
//! ```
//! use forge::sim::{SimConfig, Simulation};
//!
//! let config = SimConfig::default();
//! let mut sim = Simulation::new(config, 42);
//! sim.seed_scene(config, 20);
//! sim.run(300, &[]);
//! let h = sim.hash();
//!
//! // Same seed, same steps, same hash.
//! let mut again = Simulation::new(config, 42);
//! again.seed_scene(config, 20);
//! again.run(300, &[]);
//! assert_eq!(h, again.hash());
//! ```

// Pedantic lints that are wrong for this domain, not oversights:
// - must_use_candidate and return_self_not_must_use: the API is query-style and
//   callers legitimately discard results (step, record, insert, apply), so a
//   few dozen attributes would add churn without catching a real bug.
// - The cast family: the canonical binary format fixes widths (u32 entity
//   indexes, u64 stream lengths) and index arithmetic depends on explicit
//   widening and narrowing casts. Every narrowing value is bounded by
//   construction (entity count is capped by index width, decode lengths are
//   checked against the remaining input), so the truncation risk is nil.
// - float_cmp: exact bit equality is the determinism contract. Comparisons on
//   floats here are exact-zero checks after a sqrt and canonical-state
//   assertions in tests, both intentional.
#![allow(
    clippy::must_use_candidate,
    clippy::return_self_not_must_use,
    clippy::cast_precision_loss,
    clippy::cast_possible_truncation,
    clippy::cast_possible_wrap,
    clippy::cast_sign_loss,
    clippy::float_cmp
)]

pub mod collision;
pub mod components;
pub mod ecs;
pub mod hash;
pub mod input;
pub mod math;
pub mod physics;
pub mod prng;
pub mod render;
pub mod rollback;
pub mod scene;
pub mod serialize;
pub mod sim;
pub mod time;

pub use ecs::{Entity, World};
pub use math::{vec2, Transform, Vec2};
pub use rollback::{replay_to, RollbackError, SnapshotRing};
pub use sim::{Command, SimConfig, Simulation};
