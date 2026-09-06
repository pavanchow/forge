//! Fixed-timestep accumulator.
//!
//! Real frames arrive at irregular intervals. The accumulator banks that real
//! time and releases it in fixed `dt` slices, so the simulation always advances
//! by the same amount per step no matter the frame rate. This is the single
//! most important ingredient for determinism: identical step counts produce
//! identical state.

use crate::serialize::{ByteIo, Cursor, DecodeError};

#[derive(Clone, Copy, Debug, PartialEq)]
pub struct FixedTimestep {
    dt: f64,
    accumulator: f64,
    max_steps: u32,
}

impl FixedTimestep {
    /// A timestep of `dt` seconds with the default step cap of 8.
    ///
    /// # Panics
    ///
    /// Panics if `dt` is zero or negative, which is a caller bug: a fixed
    /// timestep must advance.
    pub fn new(dt: f64) -> Self {
        assert!(dt > 0.0, "timestep must be positive");
        FixedTimestep {
            dt,
            accumulator: 0.0,
            // Guard against the spiral of death when a frame stalls.
            max_steps: 8,
        }
    }

    pub fn with_max_steps(mut self, max_steps: u32) -> Self {
        self.max_steps = max_steps.max(1);
        self
    }

    pub fn dt(&self) -> f64 {
        self.dt
    }

    /// Bank `frame_dt` seconds of real time and return how many fixed steps
    /// should run this frame.
    pub fn accumulate(&mut self, frame_dt: f64) -> u32 {
        if frame_dt > 0.0 {
            self.accumulator += frame_dt;
        }
        let mut steps = 0;
        while self.accumulator >= self.dt && steps < self.max_steps {
            self.accumulator -= self.dt;
            steps += 1;
        }
        if steps == self.max_steps {
            // Drop the backlog rather than trying to catch up forever.
            self.accumulator = 0.0;
        }
        steps
    }

    /// Fraction of a step already banked, for render interpolation in [0, 1).
    pub fn alpha(&self) -> f64 {
        self.accumulator / self.dt
    }
}

impl ByteIo for FixedTimestep {
    fn write(&self, out: &mut Vec<u8>) {
        self.dt.write(out);
        self.accumulator.write(out);
        self.max_steps.write(out);
    }
    fn read(cur: &mut Cursor) -> Result<Self, DecodeError> {
        let dt = f64::read(cur)?;
        let accumulator = f64::read(cur)?;
        let max_steps = u32::read(cur)?;
        // `new` guarantees a positive finite dt and max_steps >= 1; a restored
        // timestep must satisfy the same invariant or the stream is corrupt.
        if !dt.is_finite() || dt <= 0.0 || max_steps == 0 {
            return Err(DecodeError::BadLayout);
        }
        Ok(FixedTimestep {
            dt,
            accumulator,
            max_steps,
        })
    }
}

#[cfg(test)]
mod tests {
    use super::*;

    #[test]
    fn steps_are_fixed_regardless_of_frame() {
        let mut a = FixedTimestep::new(0.01).with_max_steps(16);
        let mut b = FixedTimestep::new(0.01).with_max_steps(16);
        // One big frame vs many small frames covering the same real time.
        let steps_big = a.accumulate(0.1);
        let mut steps_small = 0;
        for _ in 0..10 {
            steps_small += b.accumulate(0.01);
        }
        assert_eq!(steps_big, 10);
        assert_eq!(steps_small, 10);
    }

    #[test]
    fn leftover_is_banked() {
        let mut t = FixedTimestep::new(0.01);
        assert_eq!(t.accumulate(0.025), 2);
        assert!((t.alpha() - 0.5).abs() < 1e-9);
        // The remaining 0.005 plus 0.006 crosses one more boundary.
        assert_eq!(t.accumulate(0.006), 1);
    }

    #[test]
    fn max_steps_prevents_spiral() {
        let mut t = FixedTimestep::new(0.01).with_max_steps(4);
        assert_eq!(t.accumulate(10.0), 4);
        // Backlog was dropped.
        assert_eq!(t.alpha(), 0.0);
    }
}
