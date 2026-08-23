#!/usr/bin/env bash
# Build qr_scanner_pi4 on a Raspberry Pi 4 (Raspberry Pi OS / Ubuntu, 64-bit recommended).
set -e

if ! pkg-config --exists opencv4 2>/dev/null; then
    echo "OpenCV not found. Installing (apt)..."
    sudo apt update
    sudo apt install -y build-essential cmake libopencv-dev
fi

mkdir -p build
cd build
cmake -DCMAKE_BUILD_TYPE=Release ..
make -j"$(nproc)"

echo ""
echo "Built: build/qr_scanner_pi4"
echo "Run from the pi4/ folder so it finds config.cfg and hooks/:"
echo "  cd .. && ./pi4/build/qr_scanner_pi4 ./pi4/config.cfg"
