# Independent Audit — 3 passes over V2.cpp (the code the repo's own
AUDIT_REPORT*.md files call "done"), before extending it into
qr_scanner_pi4.cpp. Only reporting what those existing reports missed.

## Pass 1 — Logic / correctness

**[BUG-1] FrameBuffer write-index drift under thermal throttling (real bug)**
`FrameBuffer` is constructed once with a fixed capacity (5) and reused
across every detection cycle. Its internal `current_idx_` is never reset
between cycles. `captureBestOfBurst` also tracks a *separate* local counter
`frames_captured` (0..burst_count-1) and calls `buffer.getFrame(frames_captured)`
assuming that equals the slot just written by `addFrame`. That's only true
while `burst_count == buffer capacity`. The moment thermal throttling kicks
in (`adaptive_burst_count = 3` while the buffer stays sized 5), `current_idx_`
advances by 3 per cycle instead of wrapping at 5, so on the next cycle
`getFrame(0..2)` no longer points at the frames just written — it can read
frames left over from a *previous* cycle. The sharpness scorer then silently
scores/selects a stale frame. Never flagged in AUDIT_REPORT / DEEP / PERFORMANCE
/ V2_FIX_SUMMARY, and it only manifests under the exact condition (thermal
throttle) those docs were trying to handle.
Fix: `FrameBuffer::reset()` rewinds the cursor at the start of every burst;
`addFrame()` now returns the slot it wrote to, which is what the caller
uses directly — no more parallel counter that can drift.

**[BUG-2] Alignment math uses hardcoded resolution, not the camera's actual one**
`capture.set(CAP_PROP_FRAME_WIDTH/HEIGHT, ...)` is never called in V1 or V2 —
the camera keeps whatever resolution the driver defaults to. `main()` reads
that actual resolution into `actual_width/actual_height` purely for a log
line, then builds `CameraParams` from **hardcoded** `1280x720` regardless.
`isAligned()` and `drawOverlay()` compute `frame_center` from those hardcoded
values. If the camera's real default differs from 1280x720 (common — many
UVC webcams default to 640x480, Pi camera via V4L2 varies by mode), every
centering check is silently wrong. Present in V1, carried into V2, not
caught by any existing audit pass.
Fix: request a resolution explicitly, then populate `CameraParams` from
`capture.get()` (the actually-negotiated size) instead of a constant.

**[BUG-3] Missing zero-height guard on the aspect-ratio divide**
`V2_TEST.cpp`'s copy of `isAligned()` has `if (detected_box.height == 0) return false;`
before the aspect-ratio division. The real `V2.cpp` `isAligned()` does not —
the guard was added to the test stub but never ported back. Not a crash
(IEEE double div-by-zero yields `inf`, which correctly fails the
0.8–1.25 check), but it's fragile and was made explicit rather than relying
on IEEE semantics by accident.

## Pass 2 — Numerical / resource

- `SharpnessScorer` pre-allocates Sobel buffers at a fixed size in its
  constructor, but `cv::Sobel`'s destination `Mat` auto-reallocates if the
  source size differs anyway (`Mat::create` inside OpenCV), so the
  pre-allocation was cosmetic, not a real optimization. Rewritten to use
  `convertScaleAbs`/`add` into member Mats so there really are zero
  temporary allocations after the first call.
- `descent_rate_ms` in V2's `main()` is documented as "meters per second" —
  misleading name, not a functional bug (removed along with the simulated
  descent loop in the Pi 4 version — see below).
- Confirmed correct (no issues): NaN/Inf guard in `computeExpectedBoxSizePx`,
  parameter validation, thermal read graceful-degrade when sysfs paths don't
  exist, cleanup on exit, `MAX_CYCLES` bound.

## Pass 3 — Architecture fit for the new requirement

V1/V2's `main()` simulates a **fake altitude that decreases every loop
iteration** (no real sensor) purely to demo the alignment box shrinking —
not something that maps to "run on a Pi 4, scan a QR, take one picture,
send it." Reusing it as-is would mean the capture-and-send step is gated on
an alignment check against a number nobody is actually feeding in.
`qr_scanner_pi4.cpp` keeps the alignment/guide-box logic (useful, and now
bug-fixed per above) but makes it **optional** (`require_alignment=false`
by default) so the trigger is exactly what was asked for: QR found + decoded
→ capture → send. Flip `require_alignment=true` and set `reference_distance_m`
once a real distance/altitude source exists.

## What's new in qr_scanner_pi4.cpp (not bug fixes, added functionality)

- Single reference photo taken at full sensor resolution and handed to
  `hooks/send_capture.sh` (fork+exec, detached, SIGCHLD ignored so no
  zombies, no shell involved → no injection risk) exactly once per new QR
  payload (debounced via `cooldown_ms` so a code sitting in frame across
  many loop iterations doesn't spam sends).
- Dedicated camera-grab thread (decouples I/O from processing).
- QR search on a downscaled frame (`detect_scale`) while the saved/sent
  photo stays full resolution — detection cost drops roughly with the
  square of the scale factor.
- Removed V2's fixed 300ms artificial per-cycle sleep; loop now runs at
  full speed and only slows via the existing thermal-adaptive burst count.
- `-O3 -mcpu=cortex-a72` build flags for Pi 4 via CMake.
- Config file, systemd unit, clean SIGINT/SIGTERM shutdown.

## Verification

Compiled clean with `g++ -std=c++17 -O2 -Wall -Wextra -pthread` against
`libopencv-dev` 4.6 — zero warnings. Ran against a nonexistent camera index
to confirm the failure path is graceful (no crash, clear message, exit
code 255) since no physical camera/QR target was available in this
environment to test the live detection path — verify that end-to-end on
the actual Pi 4 + camera before relying on it.
