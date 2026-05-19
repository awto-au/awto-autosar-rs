//! AUTOSAR CanTSyn — Time Synchronisation over CAN
//!
//! Implements the master and slave state machines defined in
//! AUTOSAR SWS TimeSyncOverCAN R4.3.1 / R21-11.
//!
//! # Frame format (classic CAN, DLC=8)
//!
//! ```text
//! Byte 0: message type
//!   0x10 = SYNC  (no CRC)    0x20 = SYNC  (CRC)
//!   0x18 = FUP   (no CRC)    0x28 = FUP   (CRC)
//! Byte 1: CRC8H2F (0x00 if unsecured)
//! Byte 2: domain[7:4] | sequence_counter[3:0]
//! Byte 3: flags — bit2=SGW, bits[1:0]=OVS
//! Bytes 4–7: seconds (u32 big-endian)
//! ```
//!
//! The SYNC `seconds` field is approximate. The FUP `seconds` field carries
//! the nanosecond *correction offset* (not a full timestamp) between the
//! time placed in SYNC and the actual measured TX time. The slave applies:
//!   `corrected_ns = sync_seconds_ns + fup_correction_ns`
//!
//! # References
//! - AUTOSAR SWS_TimeSyncOverCAN R4.3.1
//! - autoas/as CanTSyn C reference: https://github.com/autoas/as

#![no_std]

/// Message type byte values (byte 0 of every CanTSyn PDU).
pub mod msg_type {
    pub const SYNC_NO_CRC:       u8 = 0x10;
    pub const FUP_NO_CRC:        u8 = 0x18;
    pub const SYNC_WITH_CRC:     u8 = 0x20;
    pub const FUP_WITH_CRC:      u8 = 0x28;
    pub const OFS_NO_CRC:        u8 = 0x34;
    pub const OFS_WITH_CRC:      u8 = 0x44;
    pub const EXT_OFS_NO_CRC:    u8 = 0x54;
    pub const EXT_OFS_WITH_CRC:  u8 = 0x64;
}

/// SGW flag in byte 3 — set when this master is itself a slave of a
/// higher-level time domain (gateway). Equivalent to NTP stratum+1.
pub const FLAG_SGW: u8 = 0x04;

/// Maximum sequence counter value (4-bit, wraps 0→15→0).
pub const SEQ_CTR_MAX: u8 = 15;

/// A 64-bit nanosecond timestamp (seconds × 1_000_000_000 + nanoseconds).
#[derive(Clone, Copy, Debug, Default, PartialEq, Eq, PartialOrd, Ord)]
pub struct Timestamp {
    pub seconds: u32,
    pub nanoseconds: u32,
}

impl Timestamp {
    pub const fn new(seconds: u32, nanoseconds: u32) -> Self {
        Self { seconds, nanoseconds }
    }

    pub fn as_nanos_u64(self) -> u64 {
        self.seconds as u64 * 1_000_000_000 + self.nanoseconds as u64
    }
}

/// A raw 8-byte CanTSyn PDU.
#[derive(Clone, Copy, Debug, Default)]
pub struct Pdu(pub [u8; 8]);

impl Pdu {
    pub fn msg_type(&self)        -> u8 { self.0[0] }
    pub fn crc(&self)             -> u8 { self.0[1] }
    pub fn domain(&self)          -> u8 { self.0[2] >> 4 }
    pub fn seq_ctr(&self)         -> u8 { self.0[2] & 0x0F }
    pub fn flags(&self)           -> u8 { self.0[3] }
    pub fn sgw(&self)             -> bool { self.0[3] & FLAG_SGW != 0 }

    /// Seconds field (bytes 4–7, big-endian).
    pub fn seconds_be(&self) -> u32 {
        u32::from_be_bytes([self.0[4], self.0[5], self.0[6], self.0[7]])
    }

    fn set_seconds_be(&mut self, s: u32) {
        let b = s.to_be_bytes();
        self.0[4..8].copy_from_slice(&b);
    }

    /// Build a SYNC PDU (unsecured).
    pub fn build_sync(domain: u8, seq: u8, seconds: u32, sgw: bool) -> Self {
        let mut p = Self::default();
        p.0[0] = msg_type::SYNC_NO_CRC;
        p.0[1] = 0x00;
        p.0[2] = (domain << 4) | (seq & 0x0F);
        p.0[3] = if sgw { FLAG_SGW } else { 0 };
        p.set_seconds_be(seconds);
        p
    }

    /// Build a FUP PDU carrying a nanosecond correction offset (unsecured).
    pub fn build_fup(domain: u8, seq: u8, correction_ns: u32, sgw: bool) -> Self {
        let mut p = Self::default();
        p.0[0] = msg_type::FUP_NO_CRC;
        p.0[1] = 0x00;
        p.0[2] = (domain << 4) | (seq & 0x0F);
        p.0[3] = if sgw { FLAG_SGW } else { 0 };
        p.set_seconds_be(correction_ns);
        p
    }
}

// ── Master ────────────────────────────────────────────────────────────────────

#[derive(Clone, Copy, Debug, PartialEq, Eq)]
pub enum MasterState {
    Idle,
    SyncSent { seq: u8, sync_time: Timestamp },
}

/// CanTSyn master — sends SYNC + FUP pairs on a configured interval.
pub struct Master {
    pub domain: u8,
    pub sgw: bool,
    seq: u8,
    state: MasterState,
}

impl Master {
    pub const fn new(domain: u8, sgw: bool) -> Self {
        Self { domain, sgw, seq: 0, state: MasterState::Idle }
    }

    /// Call on each sync interval. Returns the SYNC PDU to transmit.
    /// Record the returned `seq` and the actual TX timestamp, then call
    /// `build_fup()` once TX is confirmed.
    pub fn build_sync(&mut self, now: Timestamp) -> Pdu {
        let pdu = Pdu::build_sync(self.domain, self.seq, now.seconds, self.sgw);
        self.state = MasterState::SyncSent { seq: self.seq, sync_time: now };
        pdu
    }

    /// Call after the SYNC frame has been transmitted with the actual TX
    /// timestamp. Returns the FUP PDU to transmit immediately.
    pub fn build_fup(&mut self, actual_tx: Timestamp) -> Option<Pdu> {
        if let MasterState::SyncSent { seq, sync_time } = self.state {
            let correction_ns = actual_tx
                .as_nanos_u64()
                .saturating_sub(sync_time.as_nanos_u64()) as u32;
            let pdu = Pdu::build_fup(self.domain, seq, correction_ns, self.sgw);
            self.seq = self.seq.wrapping_add(1) & SEQ_CTR_MAX;
            self.state = MasterState::Idle;
            Some(pdu)
        } else {
            None
        }
    }
}

// ── Slave ─────────────────────────────────────────────────────────────────────

#[derive(Clone, Copy, Debug, PartialEq, Eq)]
pub enum SlaveState {
    Idle,
    WaitingFup { seq: u8, sync_seconds: u32, rx_time: Timestamp },
}

/// Error from slave processing.
#[derive(Clone, Copy, Debug, PartialEq, Eq)]
pub enum SlaveError {
    /// FUP arrived but no pending SYNC (or seq mismatch).
    UnexpectedFup,
    /// SYNC arrived while already waiting for a FUP — stale SYNC discarded.
    SyncOverrun,
    /// Domain mismatch.
    DomainMismatch,
}

/// Corrected time output from slave after a matched SYNC+FUP pair.
#[derive(Clone, Copy, Debug)]
pub struct CorrectedTime {
    pub time: Timestamp,
    pub sgw: bool,
    pub seq: u8,
}

/// CanTSyn slave — receives SYNC + FUP pairs and outputs corrected time.
pub struct Slave {
    pub domain: u8,
    state: SlaveState,
    last_seq: Option<u8>,
}

impl Slave {
    pub const fn new(domain: u8) -> Self {
        Self { domain, state: SlaveState::Idle, last_seq: None }
    }

    /// Process a received SYNC PDU. `rx_time` is the local timestamp at receipt.
    pub fn on_sync(&mut self, pdu: &Pdu, rx_time: Timestamp) -> Result<(), SlaveError> {
        if pdu.domain() != self.domain {
            return Err(SlaveError::DomainMismatch);
        }
        if matches!(self.state, SlaveState::WaitingFup { .. }) {
            // Previous SYNC had no FUP — discard and restart.
            self.state = SlaveState::Idle;
            return Err(SlaveError::SyncOverrun);
        }
        self.state = SlaveState::WaitingFup {
            seq: pdu.seq_ctr(),
            sync_seconds: pdu.seconds_be(),
            rx_time,
        };
        Ok(())
    }

    /// Process a received FUP PDU. Returns corrected time if SYNC+FUP matched.
    pub fn on_fup(&mut self, pdu: &Pdu) -> Result<CorrectedTime, SlaveError> {
        if pdu.domain() != self.domain {
            return Err(SlaveError::DomainMismatch);
        }
        match self.state {
            SlaveState::WaitingFup { seq, sync_seconds, rx_time } => {
                if pdu.seq_ctr() != seq {
                    self.state = SlaveState::Idle;
                    return Err(SlaveError::UnexpectedFup);
                }
                let correction_ns = pdu.seconds_be(); // FUP reuses the seconds field for correction
                let base_ns = sync_seconds as u64 * 1_000_000_000;
                let corrected_ns = base_ns + correction_ns as u64;
                let time = Timestamp::new(
                    (corrected_ns / 1_000_000_000) as u32,
                    (corrected_ns % 1_000_000_000) as u32,
                );
                self.last_seq = Some(seq);
                self.state = SlaveState::Idle;
                Ok(CorrectedTime { time, sgw: pdu.sgw(), seq })
            }
            SlaveState::Idle => Err(SlaveError::UnexpectedFup),
        }
    }
}
