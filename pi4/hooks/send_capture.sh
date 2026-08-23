#!/usr/bin/env bash
# Called by qr_scanner_pi4 exactly once per QR scan event:
#   $1 = absolute path to the captured full-resolution JPEG
#   $2 = decoded QR text
#
# PLACEHOLDER: replace this with the real hand-off to the other program
# once it exists, e.g.:
#   - launch it directly:      exec /path/to/other_program "$1" "$2"
#   - HTTP upload:              curl -s -F "image=@$1" -F "text=$2" http://host:port/upload
#   - copy into a watched dir:  cp "$1" /path/other_program/watches/
#
# This script runs detached (fire-and-forget) from the scanner, so it will
# never block the scan loop -- keep it fast, or launch your real program
# in the background from here.

IMAGE_PATH="$1"
DECODED_TEXT="$2"

echo "[send_capture.sh] image=$IMAGE_PATH text=$DECODED_TEXT" >> ./captures/hook.log
