//! FNV-1a hashing over canonical byte streams.
//!
//! The world-state hash is defined as the FNV-1a hash of the world's canonical
//! serialization. Because serialization is deterministic and captures every
//! simulation-relevant bit, two runs that produce the same hash are guaranteed
//! bit-for-bit identical.

const FNV_OFFSET: u64 = 0xcbf29ce4_84222325;
const FNV_PRIME: u64 = 0x00000100_000001b3;

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
            self.state ^= b as u64;
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
        assert_eq!(hash_bytes(b"a"), 0xaf63dc4c8601ec8c);
        assert_eq!(hash_bytes(b"foobar"), 0x85944171f73967e8);
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
