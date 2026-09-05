# Forge

A from-scratch, dependency-free 2D game engine in pure Rust, built around one
idea. The simulation is deterministic and headless-testable. Given a seed and an
input script the world evolves identically on every run, and the entire world
can be serialized, restored, and continued with no drift.

Zero external dependencies. Only the standard library. Edition 2021.

Live playground: https://pavanchow.github.io/forge/

## What it is

Forge is a compact but complete game engine core. It has an entity component
system, a fixed-timestep simulation loop, semi-implicit Euler physics, AABB and
circle collision with continuous (swept) resolution, an input and event system,
a scene hierarchy, and a rendering abstraction that keeps the core headless. It
ships a CLI that runs a headless simulation from a seed and proves determinism.

## The gap it fills

Most engines are hard to test because rendering, timing, and randomness are
tangled into the core, and floating-point results wobble from run to run. Forge
inverts that. Rendering sits behind a trait so the core never needs a screen.
Time advances in fixed slices so the result never depends on frame rate.
Randomness comes from a seeded generator that lives inside the world state. The
whole world hashes to a single number.

That matters for two kinds of user.

A person building a game gets reproducible bugs. A crash from a specific seed and
input replays exactly, every time, so it can be debugged instead of chased.
Networked lockstep and replay files become trivial because identical inputs
produce identical worlds.

An AI agent building or testing game logic gets a tight, verifiable feedback
loop. The engine runs headless in CI, a run collapses to one hash, and a
regression shows up as a changed number rather than a flaky pixel diff. The three
correctness gates below are machine-checkable properties, not eyeballed demos.

## Quickstart

```rust
use forge::sim::{SimConfig, Simulation};

let config = SimConfig::default();
let mut sim = Simulation::new(config, 42);
sim.seed_scene(config, 20);   // walls plus 20 seeded balls
sim.run(300, &[]);            // 300 fixed steps, empty input script
let h = sim.hash();

// Same seed, same steps, same hash.
let mut again = Simulation::new(config, 42);
again.seed_scene(config, 20);
again.run(300, &[]);
assert_eq!(h, again.hash());
```

Run the headless CLI and its self-check:

```
cargo run --release --bin forge -- --seed 42 --steps 600 --balls 40
```

It prints world-state hashes as the run proceeds, then runs the same scenario a
second time and confirms the hashes match, checks a serialize and restore round
trip, and confirms a different seed yields a different world.

## API tour

- `math`: `Vec2` and `Transform` (translation, rotation, scale).
- `prng`: `Rng`, a seeded SplitMix64 generator. The standard library has none.
- `ecs`: `World`, `Entity`. Register component types, spawn, insert, remove,
  query, and iterate in deterministic entity order.
- `components`: `Velocity`, `Forces`, `RigidBody`, `Collider`, `Shape`, `Parent`.
- `time`: `FixedTimestep`, the accumulator.
- `physics`: `semi_implicit`, `integrate_velocities`.
- `collision`: `collide`, `broadphase_pairs`, `detect_pairs`, `brute_force_pairs`,
  `swept_aabb`.
- `input`: `InputState`, `Events<T>`.
- `scene`: `world_transform` over a parent hierarchy.
- `render`: `Renderer` trait, `NoopRenderer`, `RecordingRenderer`.
- `serialize`, `hash`: canonical binary encoding and FNV-1a over it.
- `sim`: `Simulation`, `SimConfig`, `Command`. The whole thing tied together.

## The correctness gate

Three properties are enforced as tests. They are the point of the project. Each
is bounded for CI and scaled by the `FORGE_FUZZ_OPS` environment variable.

1. Deterministic replay (`tests/determinism.rs`). Given a seed and an input
   script, the simulation produces an identical world-state hash on every run,
   across many seeds. Same seed, same result, bit for bit.
2. Serialize round trip (`tests/serialize.rs`). Serializing the whole world and
   deserializing it yields an equal world, and continuing the simulation from
   the restored world matches continuing the original bit for bit.
3. Collision correctness (`tests/collision.rs`). The broadphase plus narrowphase
   overlap set exactly matches a brute-force reference over many random entity
   sets, and the resolver stops fast bodies from tunneling through thin static
   geometry at the fixed timestep.

Plus unit tests per module for vector math, ECS add, remove, and query, the
integrator, and PRNG determinism.

Run everything:

```
cargo test
FORGE_FUZZ_OPS=200 cargo test   # heavier fuzzing
cargo clippy --all-targets -- -D warnings
```

## License

MIT.
