//! Input and events.
//!
//! `InputState` tracks which buttons are held and which changed this step, using
//! a sorted vector so the state is deterministic and serializable. `Events<T>`
//! is a simple double-buffered queue: events written this step are readable next
//! step and then cleared, which decouples producers from consumers.

use crate::serialize::{ByteIo, Cursor, DecodeError};

/// An opaque button or key identifier.
#[derive(Clone, Copy, Debug, PartialEq, Eq, PartialOrd, Ord)]
pub struct Button(pub u32);

pub mod keys {
    use super::Button;
    pub const LEFT: Button = Button(0);
    pub const RIGHT: Button = Button(1);
    pub const UP: Button = Button(2);
    pub const DOWN: Button = Button(3);
    pub const SPACE: Button = Button(4);
}

/// Which buttons are currently held, plus edges since the last `next_frame`.
#[derive(Clone, Debug, Default, PartialEq)]
pub struct InputState {
    held: Vec<u32>,
    pressed: Vec<u32>,
    released: Vec<u32>,
}

impl InputState {
    pub fn new() -> Self {
        Self::default()
    }

    pub fn press(&mut self, b: Button) {
        if let Err(i) = self.held.binary_search(&b.0) {
            self.held.insert(i, b.0);
            if let Err(j) = self.pressed.binary_search(&b.0) {
                self.pressed.insert(j, b.0);
            }
        }
    }

    pub fn release(&mut self, b: Button) {
        if let Ok(i) = self.held.binary_search(&b.0) {
            self.held.remove(i);
            if let Err(j) = self.released.binary_search(&b.0) {
                self.released.insert(j, b.0);
            }
        }
    }

    pub fn is_down(&self, b: Button) -> bool {
        self.held.binary_search(&b.0).is_ok()
    }

    pub fn just_pressed(&self, b: Button) -> bool {
        self.pressed.binary_search(&b.0).is_ok()
    }

    pub fn just_released(&self, b: Button) -> bool {
        self.released.binary_search(&b.0).is_ok()
    }

    /// Clear the per-step edge sets. Call once at the end of each step.
    pub fn next_frame(&mut self) {
        self.pressed.clear();
        self.released.clear();
    }
}

impl ByteIo for InputState {
    fn write(&self, out: &mut Vec<u8>) {
        self.held.write(out);
        self.pressed.write(out);
        self.released.write(out);
    }
    fn read(cur: &mut Cursor) -> Result<Self, DecodeError> {
        Ok(InputState {
            held: Vec::<u32>::read(cur)?,
            pressed: Vec::<u32>::read(cur)?,
            released: Vec::<u32>::read(cur)?,
        })
    }
}

/// A double-buffered event queue. Producers `send`; consumers `drain` the
/// events from the previous step. Call `next_frame` once per step to swap.
#[derive(Clone, Debug)]
pub struct Events<T> {
    current: Vec<T>,
    previous: Vec<T>,
}

impl<T> Default for Events<T> {
    fn default() -> Self {
        Events {
            current: Vec::new(),
            previous: Vec::new(),
        }
    }
}

impl<T> Events<T> {
    pub fn new() -> Self {
        Self::default()
    }

    pub fn send(&mut self, event: T) {
        self.current.push(event);
    }

    /// Read events produced during the previous step.
    pub fn read(&self) -> &[T] {
        &self.previous
    }

    /// Swap buffers: this step's events become readable, the old ones are dropped.
    pub fn next_frame(&mut self) {
        std::mem::swap(&mut self.current, &mut self.previous);
        self.current.clear();
    }

    pub fn len(&self) -> usize {
        self.previous.len()
    }

    pub fn is_empty(&self) -> bool {
        self.previous.is_empty()
    }
}

#[cfg(test)]
mod tests {
    use super::*;

    #[test]
    fn press_release_edges() {
        let mut input = InputState::new();
        input.press(keys::LEFT);
        assert!(input.is_down(keys::LEFT));
        assert!(input.just_pressed(keys::LEFT));
        assert!(!input.just_released(keys::LEFT));

        input.next_frame();
        assert!(input.is_down(keys::LEFT));
        assert!(!input.just_pressed(keys::LEFT));

        input.release(keys::LEFT);
        assert!(!input.is_down(keys::LEFT));
        assert!(input.just_released(keys::LEFT));
    }

    #[test]
    fn double_press_is_idempotent() {
        let mut input = InputState::new();
        input.press(keys::SPACE);
        input.press(keys::SPACE);
        // Held set contains exactly one entry.
        assert!(input.is_down(keys::SPACE));
        let mut buf = Vec::new();
        input.write(&mut buf);
        let mut cur = Cursor::new(&buf);
        let back = InputState::read(&mut cur).unwrap();
        assert_eq!(input, back);
    }

    #[test]
    fn events_are_double_buffered() {
        let mut events: Events<u32> = Events::new();
        events.send(1);
        events.send(2);
        // Not readable until the buffers swap.
        assert!(events.read().is_empty());
        events.next_frame();
        assert_eq!(events.read(), &[1, 2]);
        events.next_frame();
        // Cleared after one step.
        assert!(events.read().is_empty());
    }
}
