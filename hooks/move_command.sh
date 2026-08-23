#!/usr/bin/env bash
# hooks/move_command.sh <direction> <magnitude> <reason>
# Called by Prototype1 when a QR fragment or off-center QR requires drone movement guidance.

DIRECTION="${1:-NONE}"
MAGNITUDE="${2:-0.0}"
REASON="${3:-unknown}"

echo "[MoveCmd] $(date '+%Y-%m-%d %H:%M:%S') - Direction: $DIRECTION | Magnitude: $MAGNITUDE | Reason: $REASON"
