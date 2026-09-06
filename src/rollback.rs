//! Rollback-netcode primitives: a ring buffer of serialized world snapshots.
//!
//! Networked games with rollback rewind the simulation to the last confirmed
//! tick, reapply a corrected input script, and replay forward. That only works
//! on a fully deterministic engine: replaying the same inputs from the same
//! snapshot must reproduce the original world bit-for-bit. Snapshots are the
//! canonical serialization, so a world restored from a snapshot is exactly the
//! world that existed at that tick, RNG stream and all.
//!
//! The ring keeps the most recent `capacity` snapshots and silently evicts the
//! oldest, mirroring how a netcode client only needs a bounded window of
//! confirmed ticks.

use crate::serialize::DecodeError;
use crate::sim::{ScriptEntry, Simulation, SimConfig};

/// Why a rollback request can fail.
#[derive(Debug, PartialEq, Eq)]
pub enum RollbackError {
    /// No snapshot recorded at that tick is retained (evicted or never taken).
    MissingTick,
    /// The stored snapshot failed to decode.
    Corrupt(DecodeError),
}

/// One entry: the tick the snapshot was taken at, plus the canonical bytes.
type Slot = Option<(u64, Vec<u8>)>;

/// A fixed-capacity ring of world snapshots, one per recorded tick.
pub struct SnapshotRing {
    cap: usize,
    slots: Vec<Slot>,
    /// Index of the next write position.
    next: usize,
    len: usize,
}

impl SnapshotRing {
    /// A ring holding at most `capacity` snapshots. Capacity is floored at 1.
    pub fn new(capacity: usize) -> Self {
        let cap = capacity.max(1);
        SnapshotRing {
            cap,
            slots: (0..cap).map(|_| None).collect(),
            next: 0,
            len: 0,
        }
    }

    pub fn capacity(&self) -> usize {
        self.cap
    }

    pub fn len(&self) -> usize {
        self.len
    }

    pub fn is_empty(&self) -> bool {
        self.len == 0
    }

    /// Record the current state of `sim` under its current tick. Overwrites
    /// the oldest entry once the ring is full.
    pub fn record(&mut self, sim: &Simulation) {
        self.slots[self.next] = Some((sim.tick, sim.serialize()));
        self.next = (self.next + 1) % self.cap;
        self.len = (self.len + 1).min(self.cap);
    }

    /// Tick of the oldest retained snapshot, if any.
    pub fn oldest_tick(&self) -> Option<u64> {
        if self.len == 0 {
            return None;
        }
        let idx = if self.len == self.cap { self.next } else { 0 };
        self.slots[idx].as_ref().map(|(t, _)| *t)
    }

    /// Tick of the newest retained snapshot, if any.
    pub fn newest_tick(&self) -> Option<u64> {
        if self.len == 0 {
            return None;
        }
        let idx = (self.next + self.cap - 1) % self.cap;
        self.slots[idx].as_ref().map(|(t, _)| *t)
    }

    fn snapshot_bytes(&self, to_tick: u64) -> Option<&Vec<u8>> {
        self.slots
            .iter()
            .find_map(|s| s.as_ref().and_then(|(t, b)| (*t == to_tick).then_some(b)))
    }

    /// Whether a snapshot for exactly `to_tick` is retained.
    pub fn can_rollback(&self, to_tick: u64) -> bool {
        self.snapshot_bytes(to_tick).is_some()
    }

    /// Rewind to `to_tick`: return a simulation restored from the snapshot
    /// recorded at exactly that tick. The result is bit-for-bit the world as
    /// it was, ready to replay forward with [`replay_to`].
    pub fn rollback(&self, to_tick: u64) -> Result<Simulation, RollbackError> {
        let bytes = self
            .snapshot_bytes(to_tick)
            .ok_or(RollbackError::MissingTick)?;
        // deserialize overwrites every piece of simulation state (tick,
        // gravity, timestep, rng, world), so the bootstrap config is inert.
        let mut sim = Simulation::empty(SimConfig::default());
        sim.deserialize(bytes).map_err(RollbackError::Corrupt)?;
        Ok(sim)
    }
}

/// Replay `sim` forward until it reaches `target_tick`, applying `script`
/// entries as their ticks come up. If the sim is already at or past the
/// target this does nothing. The script need not be sorted, matching
/// [`Simulation::run`].
pub fn replay_to(sim: &mut Simulation, target_tick: u64, script: &[ScriptEntry]) {
    let remaining = target_tick.saturating_sub(sim.tick);
    sim.run(remaining, script);
}

#[cfg(test)]
mod tests {
    use super::*;
    use crate::math::Vec2;
    use crate::sim::{Command, SimConfig};

    fn scripted(balls: u32) -> (Simulation, Vec<ScriptEntry>) {
        let config = SimConfig::default();
        let mut sim = Simulation::new(config, 4242);
        sim.seed_scene(config, balls);
        let script: Vec<ScriptEntry> = vec![
            (
                40,
                Command::Impulse {
                    entity: sim.world.entities_with::<crate::components::Velocity>()[0],
                    delta_v: Vec2::new(30.0, 20.0),
                },
            ),
            (60, Command::SetGravity(Vec2::new(0.0, 25.0))),
        ];
        (sim, script)
    }

    #[test]
    fn ring_records_and_evicts_oldest() {
        let config = SimConfig::default();
        let mut sim = Simulation::new(config, 1);
        let mut ring = SnapshotRing::new(4);
        assert!(ring.is_empty());

        for _ in 0..6 {
            ring.record(&sim);
            sim.step();
        }
        // Recorded at ticks 0..5; the ring kept the last 4.
        assert_eq!(ring.len(), 4);
        assert_eq!(ring.capacity(), 4);
        assert_eq!(ring.oldest_tick(), Some(2));
        assert_eq!(ring.newest_tick(), Some(5));
        assert!(!ring.can_rollback(1));
        assert!(ring.can_rollback(3));
    }

    #[test]
    fn rollback_restores_exact_state() {
        let (mut sim, _) = scripted(12);
        for _ in 0..50 {
            sim.step();
        }
        let mut ring = SnapshotRing::new(64);
        ring.record(&sim);
        let bytes_at_50 = sim.serialize();

        for _ in 0..50 {
            sim.step();
        }

        let rolled = ring.rollback(50).unwrap();
        assert_eq!(rolled.tick, 50);
        assert_eq!(rolled.serialize(), bytes_at_50);
    }

    #[test]
    fn missing_tick_is_reported() {
        // An empty ring has nothing to roll back to, regardless of the tick.
        let ring = SnapshotRing::new(4);
        assert_eq!(ring.rollback(0).err(), Some(RollbackError::MissingTick));
    }

    #[test]
    fn replay_to_is_a_no_op_when_already_past() {
        let (mut sim, script) = scripted(8);
        replay_to(&mut sim, 10, &script);
        let at_ten = sim.tick;
        replay_to(&mut sim, 5, &script);
        assert_eq!(sim.tick, at_ten);
    }
}
