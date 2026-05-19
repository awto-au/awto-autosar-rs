"""
CI test for cantsyn-master frame output.

Runs cantsyn-master against a vcan0 interface and validates:
  - SYNC frames: correct msg_type, DLC=8, domain, seq counter structure
  - FUP frames:  correct msg_type, matching seq counter, sane correction value
  - Pairs arrive in order: SYNC always before FUP, seq numbers increment
  - No stray frame types on the CAN ID

Prerequisites:
  sudo modprobe vcan
  sudo ip link add vcan0 type vcan
  sudo ip link set vcan0 up
  cargo build --release --package cantsyn-tools  (from repo root)

Run:
  pytest tests/test_cantsyn_frames.py -v
"""

import subprocess
import struct
import time
import threading
import os
import sys
import pytest
import can

# ── constants matching cantsyn crate ─────────────────────────────────────────

SYNC_NO_CRC    = 0x10
FUP_NO_CRC     = 0x18
SYNC_WITH_CRC  = 0x20
FUP_WITH_CRC   = 0x28
SYNC_TYPES     = {SYNC_NO_CRC, SYNC_WITH_CRC}
FUP_TYPES      = {FUP_NO_CRC, FUP_WITH_CRC}

DEFAULT_CAN_ID = 0x700
IFACE          = "vcan0"
INTERVAL_MS    = 200   # fast so tests finish quickly
N_PAIRS        = 5     # how many SYNC+FUP pairs to collect
COLLECT_TIMEOUT_S = (N_PAIRS * INTERVAL_MS / 1000) + 3  # generous headroom

REPO_ROOT = os.path.dirname(os.path.dirname(os.path.abspath(__file__)))
MASTER_BIN = os.path.join(REPO_ROOT, "target", "release", "cantsyn-master")


# ── helpers ──────────────────────────────────────────────────────────────────

def decode_pdu(data: bytes) -> dict:
    """Decode an 8-byte CanTSyn PDU into a dict."""
    assert len(data) == 8
    msg_type  = data[0]
    crc       = data[1]
    domain    = (data[2] >> 4) & 0x0F
    seq       = data[2] & 0x0F
    flags     = data[3]
    seconds   = struct.unpack_from(">I", data, 4)[0]  # big-endian u32
    return dict(msg_type=msg_type, crc=crc, domain=domain,
                seq=seq, flags=flags, seconds=seconds)


def collect_frames(n_pairs: int, timeout: float) -> list[dict]:
    """Receive CAN frames on vcan0, return decoded PDUs for our CAN ID."""
    frames = []
    bus = can.interface.Bus(channel=IFACE, interface="socketcan")
    deadline = time.monotonic() + timeout
    try:
        while len(frames) < n_pairs * 2:
            remaining = deadline - time.monotonic()
            if remaining <= 0:
                break
            msg = bus.recv(timeout=remaining)
            if msg is None:
                break
            if msg.arbitration_id != DEFAULT_CAN_ID:
                continue
            if msg.dlc != 8:
                continue
            frames.append(decode_pdu(bytes(msg.data)))
    finally:
        bus.shutdown()
    return frames


# ── fixtures ─────────────────────────────────────────────────────────────────

@pytest.fixture(scope="module")
def master_frames():
    """Start cantsyn-master, collect N_PAIRS SYNC+FUP pairs, stop it."""
    if not os.path.exists(MASTER_BIN):
        pytest.skip(f"Binary not found: {MASTER_BIN} — run: cargo build --release --package cantsyn-tools")

    proc = subprocess.Popen(
        [MASTER_BIN, IFACE, str(INTERVAL_MS), format(DEFAULT_CAN_ID, "X")],
        stdout=subprocess.PIPE, stderr=subprocess.STDOUT,
    )
    try:
        frames = collect_frames(N_PAIRS, COLLECT_TIMEOUT_S)
    finally:
        proc.terminate()
        proc.wait(timeout=3)

    assert len(frames) >= N_PAIRS * 2, (
        f"Expected {N_PAIRS * 2} frames, got {len(frames)}. "
        f"Is {IFACE} up? (ip link set {IFACE} up)"
    )
    return frames


# ── tests ─────────────────────────────────────────────────────────────────────

def test_frame_count(master_frames):
    """We got at least N_PAIRS×2 frames."""
    assert len(master_frames) >= N_PAIRS * 2


def test_alternating_sync_fup(master_frames):
    """Frames must alternate SYNC, FUP, SYNC, FUP, ..."""
    for i, f in enumerate(master_frames[: N_PAIRS * 2]):
        if i % 2 == 0:
            assert f["msg_type"] in SYNC_TYPES, f"Frame {i} should be SYNC, got 0x{f['msg_type']:02X}"
        else:
            assert f["msg_type"] in FUP_TYPES, f"Frame {i} should be FUP, got 0x{f['msg_type']:02X}"


def test_sync_fup_seq_match(master_frames):
    """Each SYNC and its following FUP carry the same sequence counter."""
    pairs = list(zip(master_frames[::2], master_frames[1::2]))
    for i, (sync, fup) in enumerate(pairs[:N_PAIRS]):
        assert sync["seq"] == fup["seq"], (
            f"Pair {i}: SYNC seq={sync['seq']} != FUP seq={fup['seq']}"
        )


def test_seq_counter_increments(master_frames):
    """Sequence counter increments (mod 16) across successive SYNCs."""
    syncs = [f for f in master_frames if f["msg_type"] in SYNC_TYPES]
    for i in range(1, min(len(syncs), N_PAIRS)):
        expected = (syncs[i - 1]["seq"] + 1) % 16
        assert syncs[i]["seq"] == expected, (
            f"Seq jump: {syncs[i-1]['seq']} -> {syncs[i]['seq']}, expected {expected}"
        )


def test_domain_zero(master_frames):
    """All frames are on domain 0 (master started with domain=0)."""
    for f in master_frames[:N_PAIRS * 2]:
        assert f["domain"] == 0, f"Unexpected domain {f['domain']}"


def test_sync_seconds_reasonable(master_frames):
    """SYNC seconds field looks like a real Unix timestamp (after 2020)."""
    syncs = [f for f in master_frames if f["msg_type"] in SYNC_TYPES]
    year_2020_s = 1_577_836_800
    year_2100_s = 4_102_444_800
    for f in syncs[:N_PAIRS]:
        assert year_2020_s < f["seconds"] < year_2100_s, (
            f"SYNC seconds {f['seconds']} out of expected range"
        )


def test_fup_correction_reasonable(master_frames):
    """FUP correction (seconds field) should be < 10ms — TX latency on vcan0 is tiny."""
    fups = [f for f in master_frames if f["msg_type"] in FUP_TYPES]
    max_correction_ns = 10_000_000  # 10 ms
    for f in fups[:N_PAIRS]:
        assert f["seconds"] < max_correction_ns, (
            f"FUP correction {f['seconds']} ns looks too large (> 10ms)"
        )


def test_no_crc_flag(master_frames):
    """Master started without CRC so msg_type should be NO_CRC variants."""
    for f in master_frames[:N_PAIRS * 2]:
        assert f["msg_type"] in (SYNC_NO_CRC, FUP_NO_CRC), (
            f"Unexpected msg_type 0x{f['msg_type']:02X} — CRC not enabled"
        )


def test_crc_byte_zero(master_frames):
    """CRC byte (byte 1) must be 0x00 when CRC is disabled."""
    for f in master_frames[:N_PAIRS * 2]:
        assert f["crc"] == 0x00, f"CRC byte should be 0, got {f['crc']}"
