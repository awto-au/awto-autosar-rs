#!/usr/bin/env bash
# Install cantsyn-master and/or cantsyn-slave as systemd services.
#
# Usage:
#   ./install.sh master          # install + enable cantsyn-master
#   ./install.sh slave           # install + enable cantsyn-slave
#   ./install.sh both            # both
#   ./install.sh status          # show service status
#   ./install.sh stop            # stop both
#
# Prerequisites:
#   cargo build --release --package cantsyn-tools  (run from repo root first)

set -euo pipefail

REPO_ROOT="$(cd "$(dirname "$0")"/../../.. && pwd)"
BIN_DIR=/usr/local/bin
UNIT_DIR=/etc/systemd/system

install_bin() {
    local name=$1
    local src="$REPO_ROOT/target/release/$name"
    if [[ ! -f "$src" ]]; then
        echo "ERROR: $src not found — run: cargo build --release --package cantsyn-tools"
        exit 1
    fi
    echo "Installing $src -> $BIN_DIR/$name"
    sudo install -m 755 "$src" "$BIN_DIR/$name"
}

install_service() {
    local name=$1
    echo "Installing $name.service -> $UNIT_DIR/"
    sudo install -m 644 "$REPO_ROOT/cantsyn-tools/systemd/$name.service" "$UNIT_DIR/"
    sudo systemctl daemon-reload
    sudo systemctl enable --now "$name.service"
    echo "$name started and enabled."
}

cmd="${1:-help}"

case "$cmd" in
    master)
        install_bin cantsyn-master
        install_service cantsyn-master
        ;;
    slave)
        install_bin cantsyn-slave
        install_service cantsyn-slave
        ;;
    both)
        install_bin cantsyn-master
        install_bin cantsyn-slave
        install_service cantsyn-master
        install_service cantsyn-slave
        ;;
    status)
        systemctl status cantsyn-master.service cantsyn-slave.service 2>/dev/null || true
        ;;
    stop)
        sudo systemctl stop cantsyn-master.service cantsyn-slave.service 2>/dev/null || true
        echo "Stopped."
        ;;
    *)
        echo "Usage: $0 {master|slave|both|status|stop}"
        exit 1
        ;;
esac
