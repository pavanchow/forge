# Forge design

This document explains how Forge is put together, why the architecture is shaped
the way it is, and why each correctness gate proves what it claims. The single
organizing goal is determinism. Every design choice serves the property that a
seed and an input script fully determine the world.

## Architecture overview

The engine is a set of small modules layered from pure data at the bottom to the
driving simulation at the top.

At the base sit `math` and `serialize`. `math` provides `Vec2` and `Transform`.
`serialize` defines a canonical little-endian byte encoding through the `ByteIo`
trait, which every stored type implements. Above them `prng` provides a seeded
generator and `hash` provides FNV-1a over bytes. The `ecs` module holds entities
and typed component storage. `components` defines the concrete component types.
`physics`, `collision`, `time`, `input`, `scene`, and `render` are the systems
and services. `rollback` layers a snapshot ring over the canonical encoding for
netcode-style rewinds. At the top `sim` owns a world plus the RNG, gravity, and
the tick counter, and defines the step order that turns all of it into a
simulation.

Nothing depends on a graphics library or the clock. Rendering is a trait and time
is a value that the caller advances. That is what keeps the core headless.

## The ECS design

The ECS separates identity, data, and behaviour. An `Entity` is a small handle,
an index paired with a generation. Component data lives in per-type storage. A
system is just code that queries components and acts on them.

Storage is dense by index. Each component type owns a `Vec<Option<T>>` addressed
by the entity index. This gives cheap lookup and, more importantly, a stable
iteration order. Every query walks the vector in ascending index order, so two
worlds with the same contents always iterate in the same sequence regardless of
how entities were inserted or removed.

The registry that maps a component type to its storage is a hash map keyed by
`TypeId`. A hash map has a nondeterministic iteration order, so the engine never
iterates it during simulation. It is used only for lookup by type. Serialization
walks a separate vector that records component types in registration order, which
is deterministic.

Handles carry a generation to make use-after-free safe. When an entity is
despawned its index is freed for reuse and its generation is bumped. A stale
handle to the old entity fails its generation check and reads as dead, so it can
never silently address a different entity that later took the same slot.

Systems that need two components at once read small `Copy` component values one
entity at a time, compute, and write back. This avoids aliasing two type-erased
storages and keeps the borrow checker satisfied without any unsafe code, which
suits an engine at this scale.

## The fixed-timestep determinism approach and why it matters

Real frames arrive at uneven intervals. If the simulation advanced by whatever
real time elapsed since the last frame, the result would depend on frame rate and
scheduling jitter, and it would never reproduce. Forge decouples simulation time
from real time with an accumulator.

Each frame the caller adds the elapsed real time to an accumulator. The engine
then runs as many fixed `dt` steps as the accumulator can pay for, subtracting
`dt` each step, and keeps the remainder for next frame. Every step advances the
world by exactly `dt`, so a given number of steps always represents the same
amount of simulated time. A slow machine that renders at half the frame rate runs
the same number of fixed steps over the same wall-clock span as a fast one. The
banked remainder can drive render interpolation, which is presentation only and
never feeds back into the state.

The accumulator also caps the steps per frame. Without a cap, one long stall
would demand a burst of steps, which takes even longer, which demands more steps.
That runaway is the spiral of death. The cap drops the backlog instead.

This is the foundation of determinism. Fixed steps remove time as a hidden input.
Combined with a seeded RNG that lives in the world state and iteration that is
ordered by entity index, the only inputs left are the seed and the scripted
commands. That is exactly the claim the replay gate checks.

## Physics

Integration is semi-implicit Euler, also called symplectic Euler. It updates
velocity first using the current acceleration, then updates position using the
new velocity. The order matters. Explicit Euler, which moves position by the old
velocity, injects energy under a constant force and makes stacks and bounces
grow unstable. Semi-implicit Euler does not, so resting contacts settle and
bouncing bodies keep sensible energy. Forces accumulate into a per-body
accumulator that is cleared after each integration, and gravity is applied as a
global acceleration to every dynamic body.

## Collision

Collision runs in three parts, broadphase, narrowphase, and resolution, with a
continuous check folded into movement.

Narrowphase tests two shapes and returns a manifold, a unit normal pointing from
the first body to the second and a non-negative penetration depth. It covers
every pairing of axis-aligned boxes and circles. Bodies with non-finite centers
or extents collide with nothing. There is no meaningful manifold at infinity, a
NaN manifold would silently poison resolution, and non-finite geometry can only
enter through caller error or corruption, so the case is rejected before any
arithmetic that could propagate it.

Broadphase is a uniform spatial grid. Each body is inserted into every grid cell
its bounding box touches, then bodies that share a cell become candidate pairs.
Because two overlapping bounding boxes must share at least one cell, the grid can
never miss a real overlap. It only ever produces a superset of the true pairs,
which narrowphase then filters. The grid keeps the cost near linear in the common
sparse case while staying correct at any density.

The grid is also bounded against pathological bounds. A body whose bounding box
extends beyond the finite range, for example after a caller injects an infinite
position, contributes no candidates, which matches narrowphase, which also
rejects it. A finite body whose bounds span more than 2^20 cells per axis is
treated as giant and simply paired against every other body, since a body that
size overlaps a huge fraction of the world anyway. Both fallbacks keep the
candidate set a superset of the true pairs and keep the per-body cell walk
bounded, so no input can hang the loop.

Resolution handles a contact in two moves. An impulse along the contact normal
corrects the relative velocity using the combined inverse mass and a restitution
coefficient, so bodies bounce or come to rest as their material dictates. A
positional correction then pushes the bodies apart along the normal to remove
residual penetration, scaled by inverse mass and softened by a small slop so
resting contacts do not jitter. Static bodies have zero inverse mass, so they
absorb impulses and corrections without moving.

Discrete resolution alone can tunnel. A body moving faster than its own size in
one step can start a step on one side of a thin wall and end on the other with no
overlap ever detected. Forge prevents this with a swept test against static
geometry. Before a dynamic body moves, its intended displacement is cast as a ray
against each static box expanded by the mover's half-extents, the Minkowski sum,
using the slab method. If the ray hits within the step, the body is advanced only
to the time of impact and its velocity along the surface normal is cancelled. Fast
bodies therefore stop at the wall instead of passing through it.

## Serialization and hashing

Serialization defines a canonical byte layout for the whole world, the entity
table, the free list, and every component storage in registration order, plus the
tick, gravity, timestep, and RNG state. Floating-point values are written as
their raw IEEE-754 bits, so the encoding is exact rather than rounded. The
world-state hash is simply FNV-1a over that canonical encoding. Because the bytes
capture every simulation-relevant value exactly, two worlds with the same hash
are bit-for-bit identical, and the same bytes are used for both saving and
hashing, so the two can never disagree.

Decoding is defensive because save data is untrusted input. The cursor uses
checked arithmetic, so a corrupt stream that declares an absurd length fails
with an end-of-input error instead of overflowing an index. The entity free
list is validated on read, every entry must point at an existing dead slot and
appear at most once, because an out-of-range entry would panic a later spawn
and a duplicate would alias two entities onto one handle. The timestep enforces
its positive-dt invariant on read, not just in its constructor. Rejection is
always an error value, never a panic.

## Rollback netcode snapshots

`rollback` provides the primitive a lockstep-with-rollback netcode needs. A
`SnapshotRing` holds a bounded window of world snapshots keyed by tick. Each
snapshot is the canonical serialization, so restoring one reproduces the world
at that tick exactly, RNG stream and all. The ring evicts its oldest entry when
full, mirroring how a client only needs the window of confirmed ticks the
remote peer may ask it to rewind to.

Rolling back returns a restored simulation, and `replay_to` advances it to a
target tick applying an input script along the way. The whole scheme stands on
determinism. Replaying the same inputs from the same snapshot must land on the
same final hash, and that is exactly what gate five asserts, along with the
contrapositive that different inputs diverge and the divergence itself is
reproducible.

## Why each gate proves what it claims

Gate one, deterministic replay, runs the same seed and script twice across many
seeds and asserts the final hashes are equal. Since the hash is a faithful
fingerprint of the entire state, equal hashes mean the two runs produced
identical worlds down to the last bit. Running across many seeds guards against a
determinism bug that only shows up for particular data, and rerunning the same
seed several times guards against nondeterminism that is intermittent.

Gate two, serialize round trip, does two things at once. It serializes a live
world, restores it into a fresh simulation, and asserts equal hashes, which proves
the encode and decode are lossless. It then steps the original and the restored
world forward in lockstep and asserts they stay equal, which proves the restored
world is a true continuation and not merely a snapshot that looks right for one
frame. Asserting equality at both ends of the transform chain is what makes the
round trip meaningful.

Gate three, collision correctness, splits into two claims. For detection it
compares the grid-based overlap set against a brute-force check of every pair over
many random scenes at varying density. The brute-force set is the ground truth,
so exact set equality proves the broadphase drops nothing and invents nothing.
For resolution it launches a body fast enough to cross a thin wall in a single
step and asserts that across the whole run the body never appears on the far side.
That is a direct test of the anti-tunneling behaviour rather than a proxy for it.
At max scale the tunneling claim runs across a matrix of timestep and speed
combinations, and a long soak with many fast balls asserts containment and
finiteness at periodic checkpoints instead of only at the end.

Gate four, adversarial and boundary inputs, covers the cases the first three
gates never touch. Zero-size bodies and bodies exactly on cell boundaries probe
the geometric edges of the broadphase oracle. Non-finite geometry asserts the
narrowphase rejects it and the grid stays bounded. A corrupted-stream test
flips every byte of a real snapshot and requires each mutation to decode to Ok
or Err and never panic, with any Ok result still stepping and hashing safely.
Hand-crafted free lists and timesteps verify the read-side validation directly.
Empty-world hash stability, multi-generation restore chains, corner contacts
with two simultaneous walls, and mass spawn and despawn churn verify that
degenerate but legal worlds stay deterministic and finite.

Gate five, rollback reproduction, records a snapshot ring during a run, rolls
back to a mid-run tick, and replays forward. Same inputs must reproduce the
original final hash exactly, twice in a row. Different inputs must diverge,
and the divergent replay must itself be repeatable. This proves the snapshot
restore is exact and that determinism survives a rewind, which is the property
rollback netcode is built on.

## Rendering as a trait

The core emits draw commands through the `Renderer` trait. A real backend can
draw them, a no-op sink can discard them in headless runs, and a recorder can
capture them for tests. Keeping rendering behind a trait is what lets the whole
engine run and be verified in continuous integration with no display at all.
