//! cantsyn-slave — receive AUTOSAR CanTSyn SYNC+FUP pairs and print corrected time.
//!
//! Usage:  cantsyn-slave [IFACE]
//!   IFACE  CAN interface name (default: can0)
//!
//! Prints a line for each matched SYNC+FUP pair showing the corrected UTC time.

use std::{env, time::{Instant, SystemTime, UNIX_EPOCH}};
use can_driver::{CanDriver, DriverError};
use can_driver::socketcan_driver::SocketcanDriver;
use cantsyn::{Slave, Timestamp, msg_type};

const CANTSYN_CAN_ID: u32 = 0x640;

fn local_ts() -> Timestamp {
    let d = SystemTime::now().duration_since(UNIX_EPOCH).unwrap_or_default();
    Timestamp::new(d.as_secs() as u32, d.subsec_nanos())
}

fn main() {
    let args: Vec<String> = env::args().collect();
    let iface = args.get(1).map(|s| s.as_str()).unwrap_or("can0");

    let mut drv = SocketcanDriver::open(iface)
        .unwrap_or_else(|e| { eprintln!("open {iface}: {e}"); std::process::exit(1) });

    let mut slave = Slave::new(0);

    println!("cantsyn-slave on {iface}, listening for CAN-ID=0x{CANTSYN_CAN_ID:03X}");

    let mut last_print = Instant::now();

    loop {
        match drv.recv() {
            Ok(frame) if frame.id == CANTSYN_CAN_ID && frame.dlc == 8 => {
                use cantsyn::Pdu;
                let pdu = Pdu(frame.data);
                match pdu.msg_type() {
                    msg_type::SYNC_NO_CRC | msg_type::SYNC_WITH_CRC => {
                        let rx_time = local_ts();
                        match slave.on_sync(&pdu, rx_time) {
                            Ok(()) => {},
                            Err(e) => eprintln!("SYNC error: {e:?}"),
                        }
                    }
                    msg_type::FUP_NO_CRC | msg_type::FUP_WITH_CRC => {
                        match slave.on_fup(&pdu) {
                            Ok(ct) => {
                                let t = ct.time;
                                println!(
                                    "corrected UTC {}.{:09}s  seq={}  sgw={}",
                                    t.seconds, t.nanoseconds, ct.seq, ct.sgw,
                                );
                            }
                            Err(e) => eprintln!("FUP error: {e:?}"),
                        }
                    }
                    _ => {}
                }
            }
            Ok(_) => {}  // different CAN ID — ignore
            Err(DriverError::Empty) => {
                // Non-blocking poll — yield briefly to avoid busy-spinning.
                // 1 ms is fine: SYNC interval is 1000ms, we need sub-ms rx latency.
                std::thread::sleep(std::time::Duration::from_millis(1));

                // Heartbeat so the user knows we're alive.
                if last_print.elapsed().as_secs() >= 5 {
                    println!("(waiting for SYNC+FUP frames on {iface} ...)");
                    last_print = Instant::now();
                }
            }
            Err(e) => eprintln!("recv error: {e:?}"),
        }
    }
}
