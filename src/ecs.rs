//! A small, deterministic entity component system.
//!
//! Design goals:
//! - Typed component storage. Each component type gets its own dense-by-index
//!   store, so a system that iterates one component touches only that data.
//! - Deterministic iteration. Every query walks entities in ascending index
//!   order regardless of insertion history, which is what makes replay and
//!   hashing reproducible. No `HashMap` is ever iterated during simulation.
//! - Generational handles. A reused entity index carries a new generation so a
//!   stale handle can never silently address a different entity.
//!
//! The component-type registry is a `HashMap<TypeId, _>` used only for lookup by
//! type, never iterated for anything that affects state. Serialization walks the
//! registry in registration order (a `Vec`), which is deterministic.

use crate::serialize::{ByteIo, Cursor, DecodeError};
use std::any::{Any, TypeId};
use std::collections::HashMap;

/// A handle to an entity. Cheap to copy. Invalidated when the entity is despawned.
#[derive(Clone, Copy, Debug, PartialEq, Eq, Hash)]
pub struct Entity {
    pub index: u32,
    pub generation: u32,
}

#[derive(Clone, Copy, Debug, PartialEq, Eq)]
struct Slot {
    generation: u32,
    alive: bool,
}

/// A dense-by-index store for one component type.
struct Storage<T> {
    items: Vec<Option<T>>,
}

impl<T> Storage<T> {
    fn new() -> Self {
        Storage { items: Vec::new() }
    }

    fn ensure(&mut self, index: usize) {
        if self.items.len() <= index {
            self.items.resize_with(index + 1, || None);
        }
    }

    fn present(&self) -> usize {
        self.items.iter().filter(|x| x.is_some()).count()
    }
}

/// Type-erased view of a `Storage<T>` so the world can hold many kinds at once.
trait AnyStorage {
    fn as_any(&self) -> &dyn Any;
    fn as_any_mut(&mut self) -> &mut dyn Any;
    fn clear_index(&mut self, index: usize);
    fn serialize(&self, out: &mut Vec<u8>);
    fn deserialize(&mut self, cur: &mut Cursor) -> Result<(), DecodeError>;
}

impl<T: ByteIo + 'static> AnyStorage for Storage<T> {
    fn as_any(&self) -> &dyn Any {
        self
    }
    fn as_any_mut(&mut self) -> &mut dyn Any {
        self
    }
    fn clear_index(&mut self, index: usize) {
        if index < self.items.len() {
            self.items[index] = None;
        }
    }
    fn serialize(&self, out: &mut Vec<u8>) {
        self.items.write(out);
    }
    fn deserialize(&mut self, cur: &mut Cursor) -> Result<(), DecodeError> {
        self.items = Vec::<Option<T>>::read(cur)?;
        Ok(())
    }
}

/// The world owns all entities and component storages.
pub struct World {
    slots: Vec<Slot>,
    free: Vec<u32>,
    storages: HashMap<TypeId, Box<dyn AnyStorage>>,
    order: Vec<TypeId>,
    names: Vec<&'static str>,
}

impl Default for World {
    fn default() -> Self {
        Self::new()
    }
}

impl World {
    pub fn new() -> Self {
        World {
            slots: Vec::new(),
            free: Vec::new(),
            storages: HashMap::new(),
            order: Vec::new(),
            names: Vec::new(),
        }
    }

    /// Register a component type. Must be called before use, and every world
    /// that will deserialize a byte stream must register the same types in the
    /// same order.
    pub fn register<T: ByteIo + 'static>(&mut self) {
        let id = TypeId::of::<T>();
        if self.storages.contains_key(&id) {
            return;
        }
        self.storages.insert(id, Box::new(Storage::<T>::new()));
        self.order.push(id);
        self.names.push(std::any::type_name::<T>());
    }

    fn store<T: 'static>(&self) -> &Storage<T> {
        self.storages
            .get(&TypeId::of::<T>())
            .and_then(|s| s.as_any().downcast_ref::<Storage<T>>())
            .expect("component type not registered")
    }

    fn store_mut<T: 'static>(&mut self) -> &mut Storage<T> {
        self.storages
            .get_mut(&TypeId::of::<T>())
            .and_then(|s| s.as_any_mut().downcast_mut::<Storage<T>>())
            .expect("component type not registered")
    }

    /// Create a new entity with no components.
    pub fn spawn(&mut self) -> Entity {
        if let Some(index) = self.free.pop() {
            let slot = &mut self.slots[index as usize];
            slot.alive = true;
            Entity {
                index,
                generation: slot.generation,
            }
        } else {
            let index = self.slots.len() as u32;
            self.slots.push(Slot {
                generation: 0,
                alive: true,
            });
            Entity {
                index,
                generation: 0,
            }
        }
    }

    /// Returns true if the handle refers to a live entity.
    pub fn is_alive(&self, e: Entity) -> bool {
        self.slots
            .get(e.index as usize)
            .is_some_and(|s| s.alive && s.generation == e.generation)
    }

    /// Remove an entity and all of its components. The index becomes reusable
    /// with a bumped generation.
    pub fn despawn(&mut self, e: Entity) -> bool {
        if !self.is_alive(e) {
            return false;
        }
        for s in self.storages.values_mut() {
            s.clear_index(e.index as usize);
        }
        let slot = &mut self.slots[e.index as usize];
        slot.alive = false;
        slot.generation = slot.generation.wrapping_add(1);
        self.free.push(e.index);
        true
    }

    /// Attach or overwrite a component on an entity.
    pub fn insert<T: ByteIo + 'static>(&mut self, e: Entity, value: T) {
        if !self.is_alive(e) {
            return;
        }
        let store = self.store_mut::<T>();
        store.ensure(e.index as usize);
        store.items[e.index as usize] = Some(value);
    }

    /// Remove a single component from an entity, returning it if present.
    pub fn remove<T: ByteIo + 'static>(&mut self, e: Entity) -> Option<T> {
        if !self.is_alive(e) {
            return None;
        }
        let store = self.store_mut::<T>();
        if (e.index as usize) < store.items.len() {
            store.items[e.index as usize].take()
        } else {
            None
        }
    }

    pub fn get<T: 'static>(&self, e: Entity) -> Option<&T> {
        if !self.is_alive(e) {
            return None;
        }
        self.store::<T>().items.get(e.index as usize)?.as_ref()
    }

    pub fn get_mut<T: 'static>(&mut self, e: Entity) -> Option<&mut T> {
        if !self.is_alive(e) {
            return None;
        }
        let index = e.index as usize;
        self.store_mut::<T>().items.get_mut(index)?.as_mut()
    }

    pub fn has<T: 'static>(&self, e: Entity) -> bool {
        self.get::<T>(e).is_some()
    }

    /// Iterate `(Entity, &T)` for every live entity that has component `T`, in
    /// ascending index order.
    pub fn query<T: 'static>(&self) -> impl Iterator<Item = (Entity, &T)> + '_ {
        let slots = &self.slots;
        self.store::<T>()
            .items
            .iter()
            .enumerate()
            .filter_map(move |(i, opt)| {
                let v = opt.as_ref()?;
                let slot = slots.get(i)?;
                if slot.alive {
                    Some((
                        Entity {
                            index: i as u32,
                            generation: slot.generation,
                        },
                        v,
                    ))
                } else {
                    None
                }
            })
    }

    /// Run a closure over every live `&mut T`, in ascending index order.
    pub fn for_each_mut<T: 'static>(&mut self, mut f: impl FnMut(Entity, &mut T)) {
        // Snapshot the (index, generation) of live entities up front to avoid a
        // borrow conflict, then mutate the storage.
        let live: Vec<(usize, u32)> = self
            .slots
            .iter()
            .enumerate()
            .filter(|(_, s)| s.alive)
            .map(|(i, s)| (i, s.generation))
            .collect();
        let store = self.store_mut::<T>();
        for (i, generation) in live {
            if let Some(Some(v)) = store.items.get_mut(i) {
                f(
                    Entity {
                        index: i as u32,
                        generation,
                    },
                    v,
                );
            }
        }
    }

    /// Collect the handles of every live entity that has component `T`.
    pub fn entities_with<T: 'static>(&self) -> Vec<Entity> {
        self.query::<T>().map(|(e, _)| e).collect()
    }

    /// Number of live entities.
    pub fn entity_count(&self) -> usize {
        self.slots.iter().filter(|s| s.alive).count()
    }

    /// Number of stored components of type `T`.
    pub fn component_count<T: 'static>(&self) -> usize {
        self.store::<T>().present()
    }

    /// Serialize the entity table and every registered component storage, in
    /// registration order.
    pub fn serialize(&self, out: &mut Vec<u8>) {
        (self.slots.len() as u64).write(out);
        for s in &self.slots {
            s.generation.write(out);
            s.alive.write(out);
        }
        self.free.write(out);
        (self.order.len() as u64).write(out);
        for (idx, id) in self.order.iter().enumerate() {
            self.names[idx].to_string().write(out);
            self.storages[id].serialize(out);
        }
    }

    /// Restore state written by [`World::serialize`]. Component types must have
    /// been registered in the same order first.
    pub fn deserialize(&mut self, cur: &mut Cursor) -> Result<(), DecodeError> {
        let n = u64::read(cur)? as usize;
        let mut slots = Vec::with_capacity(n.min(cur.remaining()));
        for _ in 0..n {
            slots.push(Slot {
                generation: u32::read(cur)?,
                alive: bool::read(cur)?,
            });
        }
        self.slots = slots;
        self.free = Vec::<u32>::read(cur)?;
        let count = u64::read(cur)? as usize;
        if count != self.order.len() {
            return Err(DecodeError::BadLayout);
        }
        for idx in 0..count {
            let name = String::read(cur)?;
            if name != self.names[idx] {
                return Err(DecodeError::BadLayout);
            }
            let id = self.order[idx];
            self.storages
                .get_mut(&id)
                .ok_or(DecodeError::BadLayout)?
                .deserialize(cur)?;
        }
        Ok(())
    }
}

#[cfg(test)]
mod tests {
    use super::*;

    #[derive(Clone, Copy, Debug, PartialEq)]
    struct Pos(f64, f64);
    impl ByteIo for Pos {
        fn write(&self, out: &mut Vec<u8>) {
            self.0.write(out);
            self.1.write(out);
        }
        fn read(cur: &mut Cursor) -> Result<Self, DecodeError> {
            Ok(Pos(f64::read(cur)?, f64::read(cur)?))
        }
    }

    #[derive(Clone, Copy, Debug, PartialEq)]
    struct Tag(u32);
    impl ByteIo for Tag {
        fn write(&self, out: &mut Vec<u8>) {
            self.0.write(out);
        }
        fn read(cur: &mut Cursor) -> Result<Self, DecodeError> {
            Ok(Tag(u32::read(cur)?))
        }
    }

    fn fresh() -> World {
        let mut w = World::new();
        w.register::<Pos>();
        w.register::<Tag>();
        w
    }

    #[test]
    fn spawn_insert_get() {
        let mut w = fresh();
        let e = w.spawn();
        w.insert(e, Pos(1.0, 2.0));
        assert_eq!(w.get::<Pos>(e), Some(&Pos(1.0, 2.0)));
        assert!(!w.has::<Tag>(e));
    }

    #[test]
    fn despawn_invalidates_and_reuses() {
        let mut w = fresh();
        let a = w.spawn();
        w.insert(a, Pos(9.0, 9.0));
        assert!(w.despawn(a));
        assert!(!w.is_alive(a));
        assert_eq!(w.get::<Pos>(a), None);
        // Reused index, new generation, old handle stays invalid.
        let b = w.spawn();
        assert_eq!(a.index, b.index);
        assert_ne!(a.generation, b.generation);
        assert!(w.is_alive(b));
        assert!(!w.is_alive(a));
    }

    #[test]
    fn remove_component() {
        let mut w = fresh();
        let e = w.spawn();
        w.insert(e, Tag(7));
        assert_eq!(w.remove::<Tag>(e), Some(Tag(7)));
        assert_eq!(w.remove::<Tag>(e), None);
    }

    #[test]
    fn query_is_index_ordered() {
        let mut w = fresh();
        let mut ents = Vec::new();
        for i in 0..5 {
            let e = w.spawn();
            w.insert(e, Pos(i as f64, 0.0));
            ents.push(e);
        }
        let seen: Vec<f64> = w.query::<Pos>().map(|(_, p)| p.0).collect();
        assert_eq!(seen, vec![0.0, 1.0, 2.0, 3.0, 4.0]);
    }

    #[test]
    fn for_each_mut_visits_all() {
        let mut w = fresh();
        for i in 0..4 {
            let e = w.spawn();
            w.insert(e, Pos(i as f64, i as f64));
        }
        w.for_each_mut::<Pos>(|_, p| p.0 += 10.0);
        let xs: Vec<f64> = w.query::<Pos>().map(|(_, p)| p.0).collect();
        assert_eq!(xs, vec![10.0, 11.0, 12.0, 13.0]);
    }

    #[test]
    fn counts() {
        let mut w = fresh();
        let a = w.spawn();
        let b = w.spawn();
        w.insert(a, Pos(0.0, 0.0));
        w.insert(b, Pos(0.0, 0.0));
        w.insert(a, Tag(1));
        assert_eq!(w.entity_count(), 2);
        assert_eq!(w.component_count::<Pos>(), 2);
        assert_eq!(w.component_count::<Tag>(), 1);
    }

    #[test]
    fn serialize_roundtrip() {
        let mut w = fresh();
        let a = w.spawn();
        let b = w.spawn();
        let c = w.spawn();
        w.insert(a, Pos(1.0, 2.0));
        w.insert(b, Pos(3.0, 4.0));
        w.insert(c, Tag(99));
        w.despawn(b);

        let mut buf = Vec::new();
        w.serialize(&mut buf);

        let mut w2 = fresh();
        let mut cur = Cursor::new(&buf);
        w2.deserialize(&mut cur).unwrap();

        assert_eq!(w2.get::<Pos>(a), Some(&Pos(1.0, 2.0)));
        assert_eq!(w2.get::<Tag>(c), Some(&Tag(99)));
        assert!(!w2.is_alive(b));
        assert_eq!(w2.entity_count(), 2);

        // Re-serializing the restored world yields identical bytes.
        let mut buf2 = Vec::new();
        w2.serialize(&mut buf2);
        assert_eq!(buf, buf2);
    }
}
