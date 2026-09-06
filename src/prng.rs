//! Seeded pseudo-random number generator.
//!
//! The standard library ships no RNG, so the engine carries its own. The
//! generator is `SplitMix64`, chosen because it is tiny, has no dependencies, and
//! is fully deterministic given a seed. It lives inside the world state and is
//! serialized with it, so a restored world continues the exact same stream.

use crate::serialize::{ByteIo, Cursor, DecodeError};

/// `SplitMix64` generator. Same seed produces the same sequence, always.
#[derive(Clone, Copy, Debug, PartialEq, Eq)]
pub struct Rng {
    state: u64,
}

impl Rng {
    pub fn new(seed: u64) -> Self {
        Rng { state: seed }
    }

    /// Raw 64-bit output.
    pub fn next_u64(&mut self) -> u64 {
        self.state = self.state.wrapping_add(0x9e37_79b9_7f4a_7c15);
        let mut z = self.state;
        z = (z ^ (z >> 30)).wrapping_mul(0xbf58_476d_1ce4_e5b9);
        z = (z ^ (z >> 27)).wrapping_mul(0x94d0_49bb_1331_11eb);
        z ^ (z >> 31)
    }

    pub fn next_u32(&mut self) -> u32 {
        (self.next_u64() >> 32) as u32
    }

    /// Uniform f64 in the half-open range [0, 1).
    pub fn next_f64(&mut self) -> f64 {
        // Use the top 53 bits, the mantissa width of f64.
        (self.next_u64() >> 11) as f64 * (1.0 / (1u64 << 53) as f64)
    }

    /// Uniform f64 in [lo, hi).
    pub fn range_f64(&mut self, lo: f64, hi: f64) -> f64 {
        lo + (hi - lo) * self.next_f64()
    }

    /// Uniform integer in [lo, hi).
    ///
    /// # Panics
    ///
    /// Panics if the range is empty (`hi <= lo`), which is a caller bug rather
    /// than a runtime condition.
    pub fn range_u32(&mut self, lo: u32, hi: u32) -> u32 {
        assert!(hi > lo, "empty range");
        lo + (self.next_u64() % u64::from(hi - lo)) as u32
    }
}

impl ByteIo for Rng {
    fn write(&self, out: &mut Vec<u8>) {
        self.state.write(out);
    }
    fn read(cur: &mut Cursor) -> Result<Self, DecodeError> {
        Ok(Rng {
            state: u64::read(cur)?,
        })
    }
}

#[cfg(test)]
mod tests {
    use super::*;
    use crate::serialize::Cursor;

    #[test]
    fn same_seed_same_sequence() {
        let mut a = Rng::new(12345);
        let mut b = Rng::new(12345);
        for _ in 0..1000 {
            assert_eq!(a.next_u64(), b.next_u64());
        }
    }

    #[test]
    fn different_seed_diverges() {
        let mut a = Rng::new(1);
        let mut b = Rng::new(2);
        let mut differed = false;
        for _ in 0..8 {
            if a.next_u64() != b.next_u64() {
                differed = true;
                break;
            }
        }
        assert!(differed);
    }

    #[test]
    fn f64_in_unit_range() {
        let mut r = Rng::new(99);
        for _ in 0..10000 {
            let x = r.next_f64();
            assert!((0.0..1.0).contains(&x));
        }
    }

    #[test]
    fn range_u32_bounds() {
        let mut r = Rng::new(7);
        for _ in 0..10000 {
            let x = r.range_u32(10, 20);
            assert!((10..20).contains(&x));
        }
    }

    #[test]
    fn serialize_preserves_stream() {
        let mut r = Rng::new(555);
        for _ in 0..17 {
            r.next_u64();
        }
        let mut buf = Vec::new();
        r.write(&mut buf);
        let mut cur = Cursor::new(&buf);
        let mut restored = Rng::read(&mut cur).unwrap();
        for _ in 0..50 {
            assert_eq!(r.next_u64(), restored.next_u64());
        }
    }
}
