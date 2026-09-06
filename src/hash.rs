//! FNV-1a hashing over canonical byte streams.
//!
//! The world-state hash is defined as the FNV-1a hash of the world's canonical
//! serialization. Because serialization is deterministic and captures every
//! simulation-relevant bit, two runs that produce the same hash are guaranteed
//! bit-for-bit identical.

const FNV_OFFSET: u64 = 0xcbf2_9ce4_8422_2325;
const FNV_PRIME: u64 = 0x0000_0100_0000_01b3;

/// Streaming FNV-1a hasher.
#[derive(Clone, Copy, Debug)]
pub struct Fnv1a {
    state: u64,
}

impl Default for Fnv1a {
    fn default() -> Self {
        Fnv1a {
            state: FNV_OFFSET,
        }
    }
}

impl Fnv1a {
    pub fn new() -> Self {
        Self::default()
    }

    pub fn write(&mut self, bytes: &[u8]) {
        for &b in bytes {
            self.state ^= u64::from(b);
            self.state = self.state.wrapping_mul(FNV_PRIME);
        }
    }

    pub fn finish(&self) -> u64 {
        self.state
    }
}

/// One-shot FNV-1a over a byte slice.
pub fn hash_bytes(bytes: &[u8]) -> u64 {
    let mut h = Fnv1a::new();
    h.write(bytes);
    h.finish()
}

#[cfg(test)]
mod tests {
    use super::*;

    #[test]
    fn known_vectors() {
        // FNV-1a 64-bit reference values.
        assert_eq!(hash_bytes(b""), FNV_OFFSET);
        assert_eq!(hash_bytes(b"a"), 0xaf63_dc4c_8601_ec8c);
        assert_eq!(hash_bytes(b"foobar"), 0x8594_4171_f739_67e8);
    }

    #[test]
    fn streaming_matches_oneshot() {
        let mut h = Fnv1a::new();
        h.write(b"foo");
        h.write(b"bar");
        assert_eq!(h.finish(), hash_bytes(b"foobar"));
    }

    #[test]
    fn different_input_differs() {
        assert_ne!(hash_bytes(b"forge"), hash_bytes(b"Forge"));
    }
}
