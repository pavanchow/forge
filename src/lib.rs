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
//! - [`prng`] a seeded SplitMix64 generator (the standard library has no RNG).
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

pub mod collision;
pub mod components;
pub mod ecs;
pub mod hash;
pub mod input;
pub mod math;
pub mod physics;
pub mod prng;
pub mod render;
pub mod scene;
pub mod serialize;
pub mod sim;
pub mod time;

pub use ecs::{Entity, World};
pub use math::{vec2, Transform, Vec2};
pub use sim::{Command, SimConfig, Simulation};
