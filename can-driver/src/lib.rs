//! Abstract CAN I/O trait + implementations.
//!
//! The `CanDriver` trait is the only interface `cantsyn` and `canopen-time`
//! need.  Two implementations ship here:
//!
//! - `LoopbackDriver` — `no_std`, in-process ring buffer.  Use for unit tests
//!   on any platform and as a stub when porting to a new MCU (replace the
//!   push/pop logic with calls to your bxCAN / FDCAN HAL).
//!
//! - `SocketcanDriver` — Linux only (`feature = "socketcan"`).  Wraps the
//!   `socketcan` crate so the same protocol code runs on a real CAN adapter
//!   (PEAK, Kvaser, gs_usb, etc.) with zero changes.

#![no_std]

/// A minimal CAN frame as seen by the protocol layer.
///
/// Deliberately simpler than `embedded-can::Frame` — no RTR, no error frames,
/// just the data path the time-sync protocols use.
#[derive(Clone, Copy, Debug, PartialEq, Eq)]
pub struct CanFrame {
    pub id: u32,
    pub dlc: u8,
    pub data: [u8; 8],
}

impl CanFrame {
    pub fn new(id: u32, data: &[u8]) -> Self {
        let mut f = Self { id, dlc: data.len() as u8, data: [0u8; 8] };
        f.data[..data.len()].copy_from_slice(data);
        f
    }

    pub fn payload(&self) -> &[u8] {
        &self.data[..self.dlc as usize]
    }
}

/// Driver errors.
#[derive(Clone, Copy, Debug, PartialEq, Eq)]
pub enum DriverError {
    /// TX queue full (loopback) or OS send error (socketcan).
    TxFull,
    /// No frame available (non-blocking recv).
    Empty,
    /// Underlying OS / HAL error.
    Io,
}

/// Abstract CAN send + receive.
///
/// Implement this for your platform:
/// - Embedded: wrap `bxcan::Can` or `fdcan::FdCan`.
/// - Linux: use `SocketcanDriver` from this crate (feature = "socketcan").
/// - Tests: use `LoopbackDriver` from this crate.
pub trait CanDriver {
    /// Transmit a frame.  May block or return `TxFull` if the queue is full.
    fn send(&mut self, frame: &CanFrame) -> Result<(), DriverError>;

    /// Receive the next available frame without blocking.
    /// Returns `Err(Empty)` when there is nothing to read.
    fn recv(&mut self) -> Result<CanFrame, DriverError>;
}

// ── Loopback ─────────────────────────────────────────────────────────────────

const LOOPBACK_CAP: usize = 16;

/// In-process loopback driver — `no_std`.
///
/// Frames sent via `send()` are placed in a ring buffer and returned by
/// `recv()`.  Useful for unit tests and as an MCU porting stub.
pub struct LoopbackDriver {
    buf: [CanFrame; LOOPBACK_CAP],
    head: usize,
    tail: usize,
    len: usize,
}

impl LoopbackDriver {
    pub const fn new() -> Self {
        const EMPTY: CanFrame = CanFrame { id: 0, dlc: 0, data: [0u8; 8] };
        Self { buf: [EMPTY; LOOPBACK_CAP], head: 0, tail: 0, len: 0 }
    }

    pub fn is_empty(&self) -> bool { self.len == 0 }

    /// Directly inject a frame (simulates a peer transmitting).
    pub fn inject(&mut self, frame: CanFrame) -> Result<(), DriverError> {
        self.push(frame)
    }

    fn push(&mut self, frame: CanFrame) -> Result<(), DriverError> {
        if self.len == LOOPBACK_CAP { return Err(DriverError::TxFull); }
        self.buf[self.tail] = frame;
        self.tail = (self.tail + 1) % LOOPBACK_CAP;
        self.len += 1;
        Ok(())
    }
}

impl Default for LoopbackDriver {
    fn default() -> Self { Self::new() }
}

impl CanDriver for LoopbackDriver {
    fn send(&mut self, frame: &CanFrame) -> Result<(), DriverError> {
        self.push(*frame)
    }

    fn recv(&mut self) -> Result<CanFrame, DriverError> {
        if self.len == 0 { return Err(DriverError::Empty); }
        let f = self.buf[self.head];
        self.head = (self.head + 1) % LOOPBACK_CAP;
        self.len -= 1;
        Ok(f)
    }
}

// ── SocketCAN (Linux) ─────────────────────────────────────────────────────────

#[cfg(feature = "socketcan")]
pub mod socketcan_driver {
    extern crate std;
    use std::io;
    use socketcan::{CanSocket, Socket, Frame as SockFrame, CanFrame as ScanFrame, EmbeddedFrame, StandardId};
    use super::{CanDriver, CanFrame, DriverError};

    /// SocketCAN driver — wraps a `socketcan::CanSocket` bound to a named
    /// interface (e.g. `"can0"`, `"vcan0"`).
    pub struct SocketcanDriver {
        sock: CanSocket,
    }

    impl SocketcanDriver {
        /// Open the named SocketCAN interface in non-blocking mode.
        pub fn open(iface: &str) -> io::Result<Self> {
            let sock = CanSocket::open(iface)?;
            sock.set_nonblocking(true)?;
            Ok(Self { sock })
        }
    }

    impl CanDriver for SocketcanDriver {
        fn send(&mut self, frame: &CanFrame) -> Result<(), DriverError> {
            let id = StandardId::new(frame.id as u16)
                .ok_or(DriverError::Io)?;
            let sf = ScanFrame::new(id, frame.payload()).ok_or(DriverError::Io)?;
            self.sock.write_frame(&sf).map_err(|_| DriverError::Io)
        }

        fn recv(&mut self) -> Result<CanFrame, DriverError> {
            match self.sock.read_frame() {
                Ok(sf) => {
                    let id = sf.raw_id() & 0x1FFF_FFFF; // strip RTR/EFF flags
                    let data = sf.data();
                    Ok(CanFrame::new(id, data))
                }
                Err(e) if e.kind() == io::ErrorKind::WouldBlock => Err(DriverError::Empty),
                Err(_) => Err(DriverError::Io),
            }
        }
    }
}

// ── Tests ─────────────────────────────────────────────────────────────────────

#[cfg(test)]
mod tests {
    use super::*;

    #[test]
    fn loopback_roundtrip() {
        let mut drv = LoopbackDriver::new();
        let f = CanFrame::new(0x100, &[1, 2, 3, 4, 5, 6]);
        drv.send(&f).unwrap();
        assert_eq!(drv.recv().unwrap(), f);
        assert!(drv.recv().is_err());
    }

    #[test]
    fn loopback_inject() {
        let mut drv = LoopbackDriver::new();
        let f = CanFrame::new(0x200, &[0xAB]);
        drv.inject(f).unwrap();
        assert_eq!(drv.recv().unwrap(), f);
    }

    #[test]
    fn loopback_full() {
        let mut drv = LoopbackDriver::new();
        for _ in 0..16 {
            drv.send(&CanFrame::new(0, &[])).unwrap();
        }
        assert_eq!(drv.send(&CanFrame::new(0, &[])), Err(DriverError::TxFull));
    }
}
