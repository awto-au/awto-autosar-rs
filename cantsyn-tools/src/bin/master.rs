//! cantsyn-master — transmit AUTOSAR CanTSyn SYNC+FUP pairs on a SocketCAN interface.
//!
//! Usage:  cantsyn-master [IFACE [INTERVAL_MS [CAN_ID_HEX]]]
//!   IFACE       CAN interface name (default: can0)
//!   INTERVAL_MS SYNC transmit interval in ms (default: 1000)
//!   CAN_ID_HEX  CAN frame ID in hex without 0x prefix (default: 700)
//!
//! 0x700 is chosen as default — clear of all AWTO L8 HTC IDs (max 0x500)
//! and the Dingo range (0x5FF-0x61E).
//!
//! The master sends SYNC immediately, records the actual TX instant, then
//! sends FUP carrying the nanosecond correction.  Time source is CLOCK_REALTIME.

use std::{env, time::{Duration, Instant, SystemTime, UNIX_EPOCH}};
use can_driver::{CanDriver, CanFrame};
use can_driver::socketcan_driver::SocketcanDriver;
use cantsyn::{Master, Timestamp};

fn now_ts() -> Timestamp {
    let d = SystemTime::now().duration_since(UNIX_EPOCH).unwrap_or_default();
    Timestamp::new(d.as_secs() as u32, d.subsec_nanos())
}

fn main() {
    let args: Vec<String> = env::args().collect();
    let iface    = args.get(1).map(|s| s.as_str()).unwrap_or("can0");
    let interval = args.get(2)
        .and_then(|s| s.parse::<u64>().ok())
        .unwrap_or(1000);
    let can_id: u32 = args.get(3)
        .and_then(|s| u32::from_str_radix(s, 16).ok())
        .unwrap_or(0x700);

    let mut drv = SocketcanDriver::open(iface)
        .unwrap_or_else(|e| { eprintln!("open {iface}: {e}"); std::process::exit(1) });

    let mut master = Master::new(0, false);
    let period = Duration::from_millis(interval);

    println!("cantsyn-master on {iface}, interval={interval}ms, CAN-ID=0x{can_id:03X}");

    loop {
        let t_before = now_ts();
        let sync_pdu = master.build_sync(t_before);
        let tx_start = Instant::now();

        if let Err(e) = drv.send(&CanFrame::new(can_id, &sync_pdu.0)) {
            eprintln!("SYNC send error: {e:?}");
        } else {
            // Record actual TX time — in a real system this comes from the CAN
            // controller's TX timestamp register (hardware timestamping).
            // Here we approximate with wall-clock immediately after the send call.
            let tx_elapsed_ns = tx_start.elapsed().subsec_nanos();
            let actual_tx = Timestamp::new(
                t_before.seconds,
                t_before.nanoseconds.saturating_add(tx_elapsed_ns),
            );

            if let Some(fup_pdu) = master.build_fup(actual_tx) {
                if let Err(e) = drv.send(&CanFrame::new(can_id, &fup_pdu.0)) {
                    eprintln!("FUP send error: {e:?}");
                } else {
                    println!(
                        "SYNC+FUP seq={} t={}.{:09}s correction={}ns",
                        fup_pdu.seq_ctr(),
                        actual_tx.seconds,
                        actual_tx.nanoseconds,
                        actual_tx.as_nanos_u64().saturating_sub(t_before.as_nanos_u64()),
                    );
                }
            }
        }

        std::thread::sleep(period);
    }
}
