# qr_scanner_pi4

Raspberry Pi 4 (4GB) build of the QR scanner. Scans continuously; on a
decoded QR, saves one full-resolution JPEG and hands it to an external
program via `hooks/send_capture.sh` (fork+exec, detached — never blocks
the scan loop). Wire in the real downstream program later by editing that
script.

## Build

```bash
cd pi4
./build.sh
```

Or manually:
```bash
sudo apt install -y build-essential cmake libopencv-dev
mkdir build && cd build
cmake -DCMAKE_BUILD_TYPE=Release ..
make -j4
```

## Run

```bash
cd pi4
./build/qr_scanner_pi4 config.cfg
```

Ctrl+C for clean shutdown. Auto-start on boot: see `systemd/qr-scanner.service`.

## Config (`config.cfg`)

| Key | Meaning |
|---|---|
| `camera_index` | `/dev/video<N>` |
| `req_width/height` | requested capture resolution (driver may pick nearest supported) |
| `detect_scale` | QR search runs on a downscaled copy (0.4–0.6 recommended); the saved/sent photo is always full resolution |
| `burst_count` | frames per sharpness-selected burst, reduced automatically under thermal throttle |
| `require_alignment` | if `true`, only send when the QR is centered and correctly sized for `reference_distance_m` (needs a real distance/altitude reading to be meaningful — leave `false` for a plain "scan → photo → send") |
| `cooldown_ms` | minimum gap before the same QR payload can trigger another send |
| `single_shot` | exit after the first successful send instead of continuing to scan |
| `hook_script` | receives `argv[1]=image_path argv[2]=decoded_text` |

## Hooking up the other program

Edit `hooks/send_capture.sh`. It gets exactly two arguments and runs
detached, so it won't slow down scanning. Options are in comments in the
script (direct launch, HTTP POST, drop into a watched folder, etc.) — swap
in the real one once you send it over.

## Performance notes (Pi 4, 4GB)

- QR search runs on a downscaled frame (`detect_scale`); detection cost is
  roughly quadratic in pixel count, so this is the biggest lever.
- Camera I/O runs on its own thread so a slow USB/CSI read never stalls
  detection.
- No artificial per-cycle delay — the loop runs as fast as the camera and
  CPU allow. It only backs off (smaller burst) when
  `/sys/class/thermal/thermal_zone0/temp` shows real throttling.
- MJPG capture + `CAP_PROP_BUFFERSIZE=1` keeps USB webcam latency down.
- Build with `-O3 -mcpu=cortex-a72` (set automatically by `CMakeLists.txt`
  on aarch64/arm).
- For a Pi Camera Module (CSI) rather than a USB webcam, `V4L2` still works
  via `libcamera`'s V4L2 compatibility layer on current Raspberry Pi OS;
  no code change needed, just confirm `/dev/video0` exists.

## What's NOT included on purpose

The uploaded zip's top-level folder contained ~700MB of Windows `.dll`
files, an MSYS2 installer, and a Windows `.exe` from what looks like a
local vcpkg/msys2 OpenCV build environment. None of that runs on Linux/ARM
and it isn't needed here — Pi 4 gets OpenCV from `apt` (`libopencv-dev`)
via `build.sh`. It was left out of this output to keep the deliverable to
just the relevant source. The original `V1.cpp`/`V2.cpp`/docs from the zip
are kept one level up for reference; `qr_scanner_pi4.cpp` in this folder is
the version to actually deploy. See `AUDIT_REPORT_PI4.md` for the full
list of bugs found and fixed in this pass.
