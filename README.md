# awto-autosar-rs

Rust `no_std` implementations of CAN-based time synchronisation protocols.

## Crates

| Crate | Description |
|-------|-------------|
| [`cantsyn`](cantsyn/) | AUTOSAR CanTSyn — SYNC/FUP master & slave state machines (DS SWS_TimeSyncOverCAN R4.3.1) |
| [`canopen-time`](canopen-time/) | CANopen TIME object — 6-byte ms-since-midnight + days-since-1984 (DS301 §7.2.6) |

## Goals

- `#![no_std]` — runs on bare-metal STM32, RP2040, etc.
- Zero dependencies in the core crates (optional `crc` feature for CRC8H2F)
- Abstract CAN layer — bring your own frame I/O (socketcan, bxCAN, FDCAN, loopback)
- Suitable for both Linux SocketCAN hosts and embedded targets

## Usage (CanTSyn master)

```rust
use cantsyn::{Master, Timestamp};

let mut master = Master::new(0, false); // domain 0, not a gateway
let sync_pdu = master.build_sync(Timestamp::new(now_s, now_ns));
// transmit sync_pdu.0 on CAN ID 0x000 (configured per node)
let fup_pdu = master.build_fup(actual_tx_time).unwrap();
// transmit fup_pdu.0
```

## Usage (CANopen TIME consumer)

```rust
use canopen_time::CanOpenTime;

let frame = [0x40, 0x77, 0x36, 0x00, 0x2A, 0x39]; // raw 6 bytes from CAN ID 0x100
let t = CanOpenTime::from_bytes(&frame);
let unix_s = t.to_unix_epoch_s();
```

## License

MIT OR Apache-2.0
