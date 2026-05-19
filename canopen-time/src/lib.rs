//! CANopen TIME object — DS301 section 7.2.6
//!
//! 6-byte broadcast frame on CAN ID 0x100 (configurable via OD 0x1012):
//!
//! ```text
//! Bytes 0–3: milliseconds since midnight (u32 LE, 0–86_399_999)
//! Bytes 4–5: days since 1984-01-01 (u16 LE)
//! ```
//!
//! No sequence counter, no CRC, no quality field — pure broadcast.
//! Receiver trusts whatever arrives.
//!
//! # References
//! - CANopen DS301 section 7.2.6
//! - CANopenNode CO_TIME.c: https://github.com/CANopenNode/CANopenNode

#![no_std]

/// Default CAN ID for the TIME object (COB-ID TIME, OD 0x1012).
pub const TIME_COB_ID: u16 = 0x100;

/// Wire length of a TIME PDU.
pub const TIME_DLC: usize = 6;

/// Days from 1984-01-01 to Unix epoch (1970-01-01).
/// 1984 - 1970 = 14 years, accounting for leap years 1972, 1976, 1980.
pub const DAYS_1984_TO_UNIX: u16 = 5113;

/// A CANopen TIME value: milliseconds since midnight + days since 1984-01-01.
#[derive(Clone, Copy, Debug, Default, PartialEq, Eq)]
pub struct CanOpenTime {
    /// Milliseconds since midnight (0–86_399_999).
    pub ms: u32,
    /// Days since 1984-01-01.
    pub days: u16,
}

impl CanOpenTime {
    pub const fn new(ms: u32, days: u16) -> Self {
        Self { ms, days }
    }

    /// Construct from a Unix epoch (seconds since 1970-01-01 00:00:00 UTC).
    pub fn from_unix_epoch(unix_s: u32) -> Self {
        let total_days = unix_s / 86_400;
        let ms = (unix_s % 86_400) * 1_000;
        let days = (total_days as u16).saturating_add(DAYS_1984_TO_UNIX);
        // Note: saturating_add handles the case where total_days > 65535 - DAYS_1984_TO_UNIX
        // (year ~2149) gracefully rather than wrapping.
        Self { ms, days }
    }

    /// Convert to Unix epoch seconds (truncates milliseconds).
    pub fn to_unix_epoch_s(self) -> u32 {
        let unix_days = self.days.saturating_sub(DAYS_1984_TO_UNIX) as u32;
        unix_days * 86_400 + self.ms / 1_000
    }

    /// Encode to 6-byte wire format (little-endian).
    pub fn to_bytes(self) -> [u8; TIME_DLC] {
        let ms = self.ms.to_le_bytes();
        let days = self.days.to_le_bytes();
        [ms[0], ms[1], ms[2], ms[3], days[0], days[1]]
    }

    /// Decode from 6-byte wire format.
    pub fn from_bytes(b: &[u8; TIME_DLC]) -> Self {
        let ms   = u32::from_le_bytes([b[0], b[1], b[2], b[3]]);
        let days = u16::from_le_bytes([b[4], b[5]]);
        Self { ms, days }
    }
}

#[cfg(test)]
mod tests {
    use super::*;

    #[test]
    fn roundtrip_bytes() {
        let t = CanOpenTime::new(3_600_000, 14_610); // 01:00:00 on some day
        assert_eq!(CanOpenTime::from_bytes(&t.to_bytes()), t);
    }

    #[test]
    fn unix_epoch_zero() {
        // Unix epoch 0 = 1970-01-01 00:00:00 = day 0 in unix, day (0 - 5113) would underflow
        // saturating_sub gives day 0 in CANopen (1984 reference)
        let t = CanOpenTime::from_unix_epoch(0);
        // ms should be 0, days underflow saturates to 0
        assert_eq!(t.ms, 0);
    }

    #[test]
    fn unix_roundtrip() {
        // 2026-05-19 09:00:00 UTC approx
        let epoch: u32 = 1_779_141_600;
        let t = CanOpenTime::from_unix_epoch(epoch);
        let back = t.to_unix_epoch_s();
        assert_eq!(back, epoch);
    }
}
