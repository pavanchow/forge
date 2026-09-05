//! Canonical binary serialization used for both persistence and world hashing.
//!
//! Everything is written little-endian in a fixed order so that the byte
//! stream is a canonical fingerprint of the value. The determinism gate hashes
//! exactly these bytes, so any two worlds that serialize to identical bytes are
//! bit-for-bit identical.

/// A forward-only reader over a byte slice.
pub struct Cursor<'a> {
    data: &'a [u8],
    pos: usize,
}

impl<'a> Cursor<'a> {
    pub fn new(data: &'a [u8]) -> Self {
        Cursor { data, pos: 0 }
    }

    pub fn remaining(&self) -> usize {
        self.data.len() - self.pos
    }

    pub fn take(&mut self, n: usize) -> Result<&'a [u8], DecodeError> {
        if self.pos + n > self.data.len() {
            return Err(DecodeError::UnexpectedEof);
        }
        let out = &self.data[self.pos..self.pos + n];
        self.pos += n;
        Ok(out)
    }
}

/// Failure produced while decoding a byte stream.
#[derive(Debug, PartialEq, Eq)]
pub enum DecodeError {
    UnexpectedEof,
    BadTag(u8),
    BadLayout,
}

impl std::fmt::Display for DecodeError {
    fn fmt(&self, f: &mut std::fmt::Formatter<'_>) -> std::fmt::Result {
        match self {
            DecodeError::UnexpectedEof => write!(f, "unexpected end of input"),
            DecodeError::BadTag(t) => write!(f, "bad tag byte {t}"),
            DecodeError::BadLayout => write!(f, "bad layout"),
        }
    }
}

impl std::error::Error for DecodeError {}

/// Canonical little-endian byte serialization.
pub trait ByteIo: Sized {
    fn write(&self, out: &mut Vec<u8>);
    fn read(cur: &mut Cursor) -> Result<Self, DecodeError>;
}

macro_rules! impl_int {
    ($ty:ty, $n:expr) => {
        impl ByteIo for $ty {
            fn write(&self, out: &mut Vec<u8>) {
                out.extend_from_slice(&self.to_le_bytes());
            }
            fn read(cur: &mut Cursor) -> Result<Self, DecodeError> {
                let b = cur.take($n)?;
                let mut arr = [0u8; $n];
                arr.copy_from_slice(b);
                Ok(<$ty>::from_le_bytes(arr))
            }
        }
    };
}

impl_int!(u8, 1);
impl_int!(u16, 2);
impl_int!(u32, 4);
impl_int!(u64, 8);
impl_int!(i32, 4);
impl_int!(i64, 8);

impl ByteIo for f32 {
    fn write(&self, out: &mut Vec<u8>) {
        // Serialize the raw IEEE-754 bits so the byte stream is exact.
        out.extend_from_slice(&self.to_bits().to_le_bytes());
    }
    fn read(cur: &mut Cursor) -> Result<Self, DecodeError> {
        Ok(f32::from_bits(u32::read(cur)?))
    }
}

impl ByteIo for f64 {
    fn write(&self, out: &mut Vec<u8>) {
        out.extend_from_slice(&self.to_bits().to_le_bytes());
    }
    fn read(cur: &mut Cursor) -> Result<Self, DecodeError> {
        Ok(f64::from_bits(u64::read(cur)?))
    }
}

impl ByteIo for bool {
    fn write(&self, out: &mut Vec<u8>) {
        out.push(*self as u8);
    }
    fn read(cur: &mut Cursor) -> Result<Self, DecodeError> {
        match u8::read(cur)? {
            0 => Ok(false),
            1 => Ok(true),
            t => Err(DecodeError::BadTag(t)),
        }
    }
}

impl<T: ByteIo> ByteIo for Option<T> {
    fn write(&self, out: &mut Vec<u8>) {
        match self {
            None => out.push(0),
            Some(v) => {
                out.push(1);
                v.write(out);
            }
        }
    }
    fn read(cur: &mut Cursor) -> Result<Self, DecodeError> {
        match u8::read(cur)? {
            0 => Ok(None),
            1 => Ok(Some(T::read(cur)?)),
            t => Err(DecodeError::BadTag(t)),
        }
    }
}

impl<T: ByteIo> ByteIo for Vec<T> {
    fn write(&self, out: &mut Vec<u8>) {
        (self.len() as u64).write(out);
        for item in self {
            item.write(out);
        }
    }
    fn read(cur: &mut Cursor) -> Result<Self, DecodeError> {
        let len = u64::read(cur)? as usize;
        let mut v = Vec::with_capacity(len.min(cur.remaining()));
        for _ in 0..len {
            v.push(T::read(cur)?);
        }
        Ok(v)
    }
}

impl ByteIo for String {
    fn write(&self, out: &mut Vec<u8>) {
        let bytes = self.as_bytes();
        (bytes.len() as u64).write(out);
        out.extend_from_slice(bytes);
    }
    fn read(cur: &mut Cursor) -> Result<Self, DecodeError> {
        let len = u64::read(cur)? as usize;
        let bytes = cur.take(len)?;
        String::from_utf8(bytes.to_vec()).map_err(|_| DecodeError::BadLayout)
    }
}

#[cfg(test)]
mod tests {
    use super::*;

    fn roundtrip<T: ByteIo + PartialEq + std::fmt::Debug>(v: T) {
        let mut buf = Vec::new();
        v.write(&mut buf);
        let mut cur = Cursor::new(&buf);
        let back = T::read(&mut cur).unwrap();
        assert_eq!(v, back);
        assert_eq!(cur.remaining(), 0);
    }

    #[test]
    fn primitives_roundtrip() {
        roundtrip(42u8);
        roundtrip(40000u16);
        roundtrip(3_000_000_000u32);
        roundtrip(9_000_000_000_000u64);
        roundtrip(-12345i32);
        roundtrip(true);
        roundtrip(false);
    }

    #[test]
    fn float_bits_are_exact() {
        roundtrip(1.5f64);
        roundtrip(-0.0f64);
        roundtrip(f64::MAX);
        // NaN cannot use PartialEq, so compare bit patterns directly.
        let mut buf = Vec::new();
        f64::NAN.write(&mut buf);
        let mut cur = Cursor::new(&buf);
        let back = f64::read(&mut cur).unwrap();
        assert!(back.is_nan());
    }

    #[test]
    fn compound_roundtrip() {
        roundtrip(vec![1u32, 2, 3, 4]);
        roundtrip(Some(7u64));
        roundtrip(Option::<u64>::None);
        roundtrip("forge".to_string());
        roundtrip(vec![Some(1u8), None, Some(3u8)]);
    }

    #[test]
    fn eof_is_reported() {
        let buf = vec![0u8, 1];
        let mut cur = Cursor::new(&buf);
        assert_eq!(u64::read(&mut cur), Err(DecodeError::UnexpectedEof));
    }
}
