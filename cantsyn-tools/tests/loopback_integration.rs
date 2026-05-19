//! Integration test: CanTSyn master → LoopbackDriver → slave, no hardware.
//!
//! Proves the full SYNC+FUP round trip through the abstract CAN layer.

use can_driver::{CanDriver, CanFrame, LoopbackDriver};
use cantsyn::{Master, Slave, Pdu, Timestamp, msg_type};

const CAN_ID: u32 = 0x640;

/// Encode a Pdu into a CanFrame and inject into the driver.
fn send(drv: &mut LoopbackDriver, id: u32, pdu: &Pdu) {
    drv.inject(CanFrame::new(id, &pdu.0)).unwrap();
}

#[test]
fn sync_fup_produces_corrected_time() {
    let mut master = Master::new(0, false);
    let mut slave  = Slave::new(0);
    let mut drv    = LoopbackDriver::new();

    // Simulate wall clock: 2026-05-19 09:00:00 UTC
    let t_sync = Timestamp::new(1_779_141_600, 0);
    let t_tx   = Timestamp::new(1_779_141_600, 500_000); // 500 µs TX latency

    // Master builds SYNC
    let sync_pdu = master.build_sync(t_sync);
    assert_eq!(sync_pdu.msg_type(), msg_type::SYNC_NO_CRC);
    assert_eq!(sync_pdu.domain(), 0);

    send(&mut drv, CAN_ID, &sync_pdu);

    // Master builds FUP with 500µs correction
    let fup_pdu = master.build_fup(t_tx).unwrap();
    assert_eq!(fup_pdu.msg_type(), msg_type::FUP_NO_CRC);
    assert_eq!(fup_pdu.seconds_be(), 500_000); // correction in ns

    send(&mut drv, CAN_ID, &fup_pdu);

    // Slave receives SYNC
    let frame = drv.recv().unwrap();
    assert_eq!(frame.id, CAN_ID);
    let rx_time = Timestamp::new(1_779_141_600, 1_000_000); // slave local clock
    slave.on_sync(&Pdu(frame.data), rx_time).unwrap();

    // Slave receives FUP
    let frame = drv.recv().unwrap();
    let ct = slave.on_fup(&Pdu(frame.data)).unwrap();

    // Corrected time = sync_seconds + correction_ns = 1779141600s + 500µs
    assert_eq!(ct.time.seconds, 1_779_141_600);
    assert_eq!(ct.time.nanoseconds, 500_000);
    assert!(!ct.sgw);
}

#[test]
fn seq_mismatch_returns_unexpected_fup() {
    let mut master = Master::new(0, false);
    let mut slave  = Slave::new(0);
    let mut drv    = LoopbackDriver::new();

    let t = Timestamp::new(1_000_000, 0);

    let sync_pdu = master.build_sync(t);
    send(&mut drv, CAN_ID, &sync_pdu);

    // Tamper: bump seq in FUP so it doesn't match SYNC
    let mut fup_pdu = master.build_fup(t).unwrap();
    fup_pdu.0[2] = (fup_pdu.0[2] & 0xF0) | ((fup_pdu.0[2] + 1) & 0x0F);

    send(&mut drv, CAN_ID, &sync_pdu); // first = SYNC (already consumed above, need fresh)
    drv.recv().unwrap(); // discard SYNC from first send
    slave.on_sync(&Pdu(sync_pdu.0), t).unwrap();

    let err = slave.on_fup(&Pdu(fup_pdu.0)).unwrap_err();
    assert_eq!(err, cantsyn::SlaveError::UnexpectedFup);
}

#[test]
fn domain_mismatch_rejected() {
    let mut slave = Slave::new(1); // domain 1
    let pdu = cantsyn::Pdu::build_sync(0, 0, 0, false); // domain 0
    let rx_time = Timestamp::new(0, 0);
    let err = slave.on_sync(&pdu, rx_time).unwrap_err();
    assert_eq!(err, cantsyn::SlaveError::DomainMismatch);
}
