# Advanced Audit Report: V1.cpp
## Logic, Algorithm, and Performance Deep Dive

**Date:** 2026-08-18  
**Scope:** Logic correctness, algorithmic efficiency, edge cases, numerical stability  
**Verdict:** ⚠️ **MULTIPLE LOGIC FLAWS** — Code has design issues beyond syntax errors

---

## 🔴 LOGIC ERRORS (Will Produce Wrong Results)

### 1. **Sharpness Metric is Mathematically Flawed** ⚠️ CRITICAL
**Location:** `computeSharpnessScore()`, lines 45-53

```cpp
double computeSharpnessScore(const cv::Mat& frame) {
    cv::Mat gray, lap;
    // ... convert to gray ...
    cv::Laplacian(gray, lap, CV_64F);
    cv::Scalar mean, stddev;
    cv::meanStdDev(lap, mean, stddev);
    return stddev[0] * stddev[0]; // variance of Laplacian
}
```

**Problem:**
Laplacian computes the 2nd derivative (∇²f). In noise-free images, Laplacian = 0 almost everywhere. In noisy/textured images, Laplacian has high variance.

**Result:** The metric conflates "texture richness" with "sharpness":
- A blurry image WITH texture (e.g., blurry photo of a textured floor) scores HIGH
- A sharp image WITH NO texture (e.g., plain white wall) scores LOW
- A blurry QR code (pure black/white) might score lower than a sharp random noise pattern

**Real-world consequence on Pi 4 drone at 2-4m altitude:**
- Blurry QR code in outdoor environment (sky background) may score lower than a sharp motion-blurred image of leaves/grass in background
- **You select the WRONG frame as "best"** → Detection fails → Drone descends unnecessarily

**Better sharpness metric:**
```cpp
// Use Sobel edge density (detects actual edges, not just variance)
double computeSharpnessScore_Better(const cv::Mat& gray) {
    cv::Mat sobelx, sobely;
    cv::Sobel(gray, sobelx, CV_32F, 1, 0, 3);
    cv::Sobel(gray, sobely, CV_32F, 0, 1, 3);
    cv::Mat edges = cv::abs(sobelx) + cv::abs(sobely);
    return cv::mean(edges)[0];  // Average edge magnitude
}
```

**Or use Tenengrad (Laplacian + Gradient combo):**
```cpp
double computeSharpnessScore_Tenengrad(const cv::Mat& gray) {
    cv::Mat lap, sobelx, sobely;
    cv::Laplacian(gray, lap, CV_32F);
    cv::Sobel(gray, sobelx, CV_32F, 1, 0, 3);
    cv::Sobel(gray, sobely, CV_32F, 0, 1, 3);
    
    cv::Mat tenengrad = (sobelx.mul(sobelx) + sobely.mul(sobely)).mul(lap.mul(lap));
    return cv::mean(tenengrad)[0];
}
```

**Impact:** **CRITICAL** — Wrong frame selection cascades failures downstream

---

### 2. **Bounding Box Averaging is Non-Isotropic** 
**Location:** `isAligned()`, line 128

```cpp
bool isAligned(...) {
    // --- size check ---
    double detected_size = (detected_box.width + detected_box.height) / 2.0;
    //                      ^ Average width and height
    double size_diff_ratio = std::abs(detected_size - expected_size_px)
                              / static_cast<double>(expected_size_px);
```

**Problem:**
QR codes are square. A detected bounding box that's 200×100 pixels has average size = 150. But expected size for a square QR is 100×100.

**Wrong result:**
- Detected: 200×100 → average = 150
- Expected: 100×100 (for square QR)
- Difference: 50 / 100 = 0.5 (50% mismatch)
- With tolerance=0.10 (10%), this FAILS alignment ✗

**What actually happened:**
The QR code is ROTATED or SKEWED in the image. The detector found it but with perspective distortion. You should be flagging this as a perspective/rotation issue, NOT a size issue.

**Better approach:**
```cpp
// Check actual QR size, not average
// QR codes have equal width and height
double size_x = detected_box.width;
double size_y = detected_box.height;
double aspect_ratio = size_x / size_y;  // Should be ~1.0 for square QR

// Check if reasonably square
if (aspect_ratio < 0.8 || aspect_ratio > 1.25) {
    // QR is rotated/skewed; likely false positive or detection artifact
    return false;
}

// Use average size for comparison, but only for roughly square QRs
double detected_size = (size_x + size_y) / 2.0;
```

**Impact:** **MEDIUM** — Causes false negatives (rejects valid detections)

---

### 3. **Expected Size Calculation Ignores Camera Orientation** 
**Location:** `computeExpectedBoxSizePx()`, lines 109-119

```cpp
double size_px = (target.qr_real_size_m * cam.focal_length_px) / altitude_m;
```

**Problem:**
This is the **pinhole camera model**, but it assumes:
- QR code is perfectly parallel to the camera sensor (no pitch/roll of drone)
- No lens distortion
- Altitude is measured perpendicular to QR plane
- No perspective transform

**Real-world issues on drone:**
- Drone pitch/roll varies ±20° during descent
- At 2m altitude with ±20° pitch, the perpendicular distance can vary by 20-30%
- Expected size becomes inaccurate
- False rejections or missed detections

**Severity depends on your tolerance:**
- If tolerance=±10%, a ±20° pitch → FAIL alignment
- If tolerance=±50%, you might miss misaligned QRs

**Fix (with IMU data):**
```cpp
// If you have drone pitch/roll from IMU:
double perpendicular_altitude = altitude_m * cos(pitch_angle) * cos(roll_angle);
double size_px = (target.qr_real_size_m * cam.focal_length_px) / perpendicular_altitude;
```

**Or use adaptive tolerance:**
```cpp
// Increase tolerance when drone attitude uncertainty is high
double attitude_uncertainty_factor = 1.2;  // ±20° uncertainty
double expected_size_min = size_px / attitude_uncertainty_factor;
double expected_size_max = size_px * attitude_uncertainty_factor;
```

**Impact:** **MEDIUM** — Depends on your drone's pitch/roll stability

---

### 4. **Center-of-Frame Check is Too Strict for Tactical Cameras** 
**Location:** `isAligned()`, lines 131-140

```cpp
cv::Point frame_center(cam.frame_width_px / 2, cam.frame_height_px / 2);
double offset = cv::norm(detected_center - frame_center);
bool center_ok = offset <= target.center_tolerance_px;
```

With `center_tolerance_px = 40.0` and frame size 1280×720:
- Frame center: (640, 360)
- Acceptable region: radius 40px circle around center
- This is **3% of frame width**, extremely restrictive

**Real scenario:**
- Drone descends slightly off-center
- QR is detected but 50px from frame center
- Alignment fails, drone descends further unnecessarily
- On real descent, this causes oscillation around target

**Better approach:**
```cpp
// Define acceptable region as percentage of frame, not fixed pixels
double max_center_offset_fraction = 0.15;  // 15% of frame width
double max_center_offset_px = cam.frame_width_px * max_center_offset_fraction;
bool center_ok = offset <= max_center_offset_px;
```

Or make it adaptive to detected QR size:
```cpp
// Allow larger offset if QR is far away (small in frame)
double size_frac = detected_size / expected_size_px;  // How close is QR?
double adaptive_tolerance = target.center_tolerance_px * (2.0 - size_frac);
```

**Impact:** **MEDIUM** — Causes missed valid alignments

---

### 5. **Descent Rate is Fixed; No Acceleration Awareness** 
**Location:** Main loop, line 219

```cpp
altitude_m -= 0.25;  // Fixed 0.25m descent per cycle
```

**Problem:**
Descent rate doesn't account for:
- Cycle time variability (could be 100ms to 500ms on Pi 4)
- Drone's actual descent speed from autopilot
- If cycle takes 200ms and you descend 0.25m, real descent rate = 1.25 m/s
- If cycle takes 500ms (thermal throttle), you descend 0.5m per cycle but think it's 0.25m

**Result:** On a Pi 4 under thermal load:
- Early cycles: fast (200ms) → descent seems slow
- Later cycles: thermal throttle (500ms+) → descent actually accelerates
- Drone oscillates or misses landing zone

**Fix:**
```cpp
auto cycle_start = std::chrono::high_resolution_clock::now();

bool success = runDetectionCycle(capture, cam, target, altitude_m);

auto cycle_end = std::chrono::high_resolution_clock::now();
double cycle_time_s = std::chrono::duration<double>(cycle_end - cycle_start).count();

// Adjust descent based on actual cycle time
double target_descent_rate_ms = 0.25;  // m/s (25cm/sec)
double descent_this_cycle = target_descent_rate_ms * cycle_time_s;
altitude_m -= descent_this_cycle;
```

**Impact:** **MEDIUM** — Causes descent trajectory errors

---

## 🟠 NUMERICAL PRECISION ISSUES

### 6. **Integer Truncation in Box Calculation** 
**Location:** `computeExpectedBoxSizePx()`, line 118

```cpp
double size_px = (target.qr_real_size_m * cam.focal_length_px) / altitude_m;
return static_cast<int>(std::round(size_px));  // ← Rounds to nearest pixel
```

**Problem:**
- True size: 125.4 pixels
- Rounded to: 125 pixels
- But if altitude changes by 0.05m, true size becomes 124.8 → rounds to 125
- You lose precision in tracking size change vs. altitude change

**Better: Keep as double and round only for display:**
```cpp
// Return as double internally
double computeExpectedBoxSizePx_Double(const CameraParams& cam,
                                        const TargetParams& target,
                                        double altitude_m) {
    if (altitude_m <= 0.0) return 0.0;
    return (target.qr_real_size_m * cam.focal_length_px) / altitude_m;
}

// In isAligned(), use double comparison
bool isAligned(const cv::Rect& detected_box,
               double expected_size_px,  // ← Now double
               ...) {
    double detected_size = (detected_box.width + detected_box.height) / 2.0;
    double size_diff_ratio = std::abs(detected_size - expected_size_px)
                              / expected_size_px;
    // ... rest unchanged
}
```

**Impact:** **LOW** — For precision at 1-4m altitude, error is <1px. Matters for long-range (>10m) ops.

---

### 7. **Floating-Point Comparison Without Epsilon**
**Location:** `computeExpectedBoxSizePx()`, line 111

```cpp
if (altitude_m <= 0.0) {  // Direct float comparison to 0.0
```

**Problem:**
If altitude_m is from a sensor with noise or integration drift, it might be -0.0000001 or 0.0000001. 

```cpp
if (-0.00001 <= 0.0)  // TRUE → returns 0
double size_px = (0.2 * 800) / (-0.00001);  // NaN or huge negative number!
```

**Fix:**
```cpp
const double ALTITUDE_MIN = 0.01;  // 1cm minimum
if (altitude_m < ALTITUDE_MIN) {
    std::cerr << "[Warning] Altitude " << altitude_m << "m is below minimum\n";
    return 0;
}
```

**Impact:** **MEDIUM** — Unlikely in practice, but produces NaN/Inf silently

---

## 🟡 DESIGN & ARCHITECTURAL ISSUES

### 8. **No Support for Multi-Detection (Multiple QR Codes)**
**Location:** `detectQR()`, lines 86-102

```cpp
DetectionResult detectQR(const cv::Mat& frame) {
    DetectionResult result;
    cv::QRCodeDetector detector;
    std::vector<cv::Point> points;  // ← Single vector!
    
    std::string text = detector.detectAndDecode(frame, points);
    // ...
}
```

**Problem:**
- Returns only ONE QR code's bounding box
- If two QR codes in frame, only detects first one
- On a large landing pad with multiple markers, misses valid targets

**Real scenario:**
- Landing zone has 4 QR codes (for redundancy)
- Your detector only finds one
- If that one is partially occluded, you miss alignment
- Better: Find all QRs, pick best one

**Fix:**
```cpp
struct DetectionResult {
    bool found = false;
    std::vector<cv::Rect> detected_boxes;  // Multiple boxes
    std::vector<std::string> decoded_texts;
    int best_idx = -1;  // Index of best quality detection
};

DetectionResult detectQR_Multi(const cv::Mat& frame) {
    DetectionResult result;
    cv::QRCodeDetector detector;
    
    std::vector<std::vector<cv::Point>> points_list;
    std::vector<std::string> decoded_list;
    
    detector.detectMulti(frame, points_list);  // Detect ALL QRs
    
    for (auto& points : points_list) {
        result.detected_boxes.push_back(cv::boundingRect(points));
        // Try to decode each
        std::string text = detector.decode(frame, points);
        result.decoded_texts.push_back(text);
    }
    
    result.found = !result.detected_boxes.empty();
    return result;
}
```

**Impact:** **MEDIUM** — Depends on your landing zone design. Good for redundancy.

---

### 9. **No Handling of Partial QR Detection**
**Location:** `detectQR()` / `runDetectionCycle()`, lines 189-201

```cpp
bool success = runDetectionCycle(...);
// ...
return aligned && detection.found && !detection.decoded_text.empty();
```

**Problem:**
- QR detector can find a QR (bounding box) but fail to DECODE
- This is common at extreme angles, motion blur, or poor lighting
- Your code treats "found but not decoded" as failure
- But the geometry is still useful for alignment!

**Real scenario:**
- QR detected at correct size/position (ALIGNED)
- But text is unreadable (motion blur on frame edges)
- You return failure, drone descends
- Next frame same QR is readable
- You wasted a cycle

**Better:**
```cpp
// Separate "localized" from "decoded"
struct DetectionResult {
    bool localized = false;   // QR bounding box found
    bool decoded = false;     // QR text successfully read
    cv::Rect detected_box;
    std::string decoded_text;
};

// In alignment check: use localized, not decoded
bool canProceeed(DetectionResult detection) {
    return detection.localized;  // Localization is enough
}

// But require decoded for final confirmation
bool canLand(DetectionResult detection) {
    return detection.decoded;
}
```

**Impact:** **MEDIUM** — Improves robustness to motion blur / poor lighting

---

### 10. **No Retry Logic on Frame Drops**
**Location:** `captureBestOfBurst()`, lines 62-71

```cpp
for (int i = 0; i < burst_count; ++i) {
    cv::Mat frame;
    capture >> frame;
    if (frame.empty()) continue;  // ← Just skip, don't retry!
    frames.push_back(frame.clone());
    scores.push_back(computeSharpnessScore(frame));
}

if (frames.empty()) {
    return cv::Mat();  // Return empty if all 5 fail!
}
```

**Problem:**
- If camera drops 5 frames in a row (e.g., USB timeout, thermal shutdown on Pi 4), entire burst fails
- Returns empty frame
- Caller crashes or returns failure
- Drone loses one detection cycle

**Real scenario on Pi 4:**
- CPU throttles at 82°C
- Camera I/O is slower
- Frame drop rate increases to 5-10%
- Probability of all 5 frames in burst failing ≈ (0.05)^5 to (0.10)^5
- With 20 descent cycles before min altitude, risk of total failure becomes appreciable

**Fix:**
```cpp
cv::Mat captureBestOfBurst_Resilient(..., int burst_count = 5) {
    int max_attempts = burst_count * 3;  // Allow 3x retries
    int frames_captured = 0;
    
    std::vector<cv::Mat> frames_gray;
    std::vector<double> scores;
    
    while (frames_captured < burst_count && max_attempts-- > 0) {
        cv::Mat frame;
        capture >> frame;
        if (frame.empty()) {
            std::cerr << "[Retry] Frame dropped, attempting again...\n";
            std::this_thread::sleep_for(std::chrono::milliseconds(10));
            continue;
        }
        // ... process ...
        frames_captured++;
    }
    
    if (frames_captured < burst_count / 2) {
        // Less than half frames captured → bad state
        std::cerr << "[Error] Insufficient frames: " << frames_captured << " / " << burst_count << "\n";
        return cv::Mat();
    }
    
    // Return best of captured frames
}
```

**Impact:** **MEDIUM** — Improves reliability under thermal stress

---

## 📊 Summary Table: Logic Errors

| # | Issue | Type | Severity | Consequence |
|---|---|---|---|---|
| 1 | Sharpness metric conflates texture with blur | Algorithm | 🔴 CRITICAL | Wrong frame selection |
| 2 | Bounding box averaging is non-isotropic | Logic | 🟠 HIGH | False negatives on rotated QRs |
| 3 | Expected size ignores drone attitude | Physics | 🟠 HIGH | Fails on pitched/rolled drone |
| 4 | Center tolerance too strict | Design | 🟡 MEDIUM | Misses valid alignments |
| 5 | Fixed descent rate ignores cycle time | Control | 🟡 MEDIUM | Descent oscillation |
| 6 | Integer truncation loses precision | Numerical | 🟢 LOW | Matters only >10m altitude |
| 7 | Float comparison to 0.0 without epsilon | Numerical | 🟡 MEDIUM | NaN/Inf edge case |
| 8 | Only handles single QR detection | Design | 🟡 MEDIUM | Misses redundant markers |
| 9 | Treats partial detection as failure | Logic | 🟡 MEDIUM | Extra descent cycles |
| 10 | No retry on frame drops | Resilience | 🟡 MEDIUM | Fails under thermal stress |

---

## ✅ Recommended Fixes (Phased)

### Phase 1 (Critical — Do First)
1. **Replace sharpness metric** — Use Sobel edge density or Tenengrad
2. **Add aspect ratio check** — Validate QR is square before accepting
3. **Add altitude minimum guard** — Prevent NaN/Inf

### Phase 2 (High Priority)
4. **Add adaptive centering tolerance** — Use % of frame, not fixed pixels
5. **Make descent adaptive** — Base on actual cycle time
6. **Add drone attitude correction** — Use IMU data if available

### Phase 3 (Robustness)
7. **Add multi-QR detection** — Support redundant markers
8. **Separate localization from decoding** — Use geometry even if text fails
9. **Add frame drop retry logic** — Resilience to camera timeouts
10. **Use double precision internally** — Round only for display

---

## 🔍 Testing Checklist

Once fixed, test these scenarios:

- [ ] **Blurry background, sharp QR** → Selects sharp QR
- [ ] **Sharp background, blurry QR** → Correctly identifies blurry QR (doesn't pick background)
- [ ] **Rotated QR in frame** → Detects and accepts (aspect ratio check)
- [ ] **Drone pitched 20° down** → Corrects expected size or adapts tolerance
- [ ] **QR slightly off-center** → Still aligns (not <3% center requirement)
- [ ] **Camera drops 1-2 frames in burst** → Recovers with retry logic
- [ ] **Sustained load (CPU hot)** → Maintains consistent cycle time (no oscillation)
- [ ] **Multiple QRs in frame** → Picks best one

---

