// Drone QR Scanner — Altitude-Adaptive Bounding Box + Burst Sharpness Select
// Dependencies: OpenCV (core, imgproc, objdetect, highgui/videoio)
// NOTE ON CALIBRATION:
//   focal_length_px must come from your camera's intrinsic calibration
//   (cv::calibrateCamera), NOT guessed. If you only know the horizontal
//   field of view (HFOV) in degrees, use:
//       focal_length_px = (frame_width_px / 2.0) / tan(HFOV_radians / 2.0)

#include <opencv2/opencv.hpp>
#include <opencv2/objdetect.hpp>
#include <vector>
#include <string>
#include <iostream>
#include <algorithm>

// ---------------------------------------------------------------------------
// Camera intrinsic + physical target parameters
// ---------------------------------------------------------------------------
struct CameraParams {
    double focal_length_px;   // from calibration
    int frame_width_px;
    int frame_height_px;
};

struct TargetParams {
    double qr_real_size_m;    // physical side length of the printed QR code
    double size_tolerance;    // e.g. 0.10 = allow +/-10% size mismatch
    double center_tolerance_px; // allowed offset from frame center, in pixels
};

// ---------------------------------------------------------------------------
// Result of one detection cycle
// ---------------------------------------------------------------------------
struct DetectionResult {
    bool found = false;
    cv::Rect detected_box;
    std::string decoded_text;
};

// ---------------------------------------------------------------------------
// 1. Sharpness scoring (variance of Laplacian)
//    Higher score = sharper image = less motion blur
// ---------------------------------------------------------------------------
double computeSharpnessScore(const cv::Mat& frame) {
    cv::Mat gray, lap;
    if (frame.channels() == 3) {
        cv::cvtColor(frame, gray, cv::COLOR_BGR2GRAY);
    } else {
        gray = frame;
    }
    cv::Laplacian(gray, lap, CV_64F);
    cv::Scalar mean, stddev;
    cv::meanStdDev(lap, mean, stddev);
    return stddev[0] * stddev[0]; // variance
}

// ---------------------------------------------------------------------------
// 2. Burst capture + best-frame selection
//    Replace the capture.read() call with your actual drone camera source.
// ---------------------------------------------------------------------------
cv::Mat captureBestOfBurst(cv::VideoCapture& capture, int burst_count = 5) {
    std::vector<cv::Mat> frames;
    std::vector<double> scores;

    for (int i = 0; i < burst_count; ++i) {
        cv::Mat frame;
        capture >> frame;
        if (frame.empty()) continue;
        frames.push_back(frame.clone());
        scores.push_back(computeSharpnessScore(frame));
    }

    if (frames.empty()) {
        return cv::Mat(); // caller must handle empty result
    }

    int best_idx = static_cast<int>(
        std::max_element(scores.begin(), scores.end()) - scores.begin());

    std::cout << "[Burst] Selected frame " << best_idx
              << " with sharpness score " << scores[best_idx] << std::endl;

    return frames[best_idx];
}

// ---------------------------------------------------------------------------
// 3. QR detection (OpenCV's built-in detector; swap for ZBar if preferred)
// ---------------------------------------------------------------------------
DetectionResult detectQR(const cv::Mat& frame) {
    DetectionResult result;
    static cv::QRCodeDetector detector;
    std::vector<cv::Point> points;

    std::string text = detector.detectAndDecode(frame, points);

    if (!points.empty()) {
        cv::Rect box = cv::boundingRect(points);
        result.found = true;
        result.detected_box = box;
        result.decoded_text = text; // may be empty if detected but not decoded yet
    }
    return result;
}

// ---------------------------------------------------------------------------
// 4. Altitude-adaptive expected box size
//    expected_size_px = (real_size_m * focal_length_px) / altitude_m
// ---------------------------------------------------------------------------
int computeExpectedBoxSizePx(const CameraParams& cam,
                              const TargetParams& target,
                              double altitude_m) {
    if (altitude_m <= 0.0) {
        std::cerr << "[Warning] Invalid altitude, returning 0 box size\n";
        return 0;
    }
    double size_px = (target.qr_real_size_m * cam.focal_length_px) / altitude_m;
    return static_cast<int>(std::round(size_px));
}

// ---------------------------------------------------------------------------
// 5. Alignment check -> true (green) or false (red)
// ---------------------------------------------------------------------------
bool isAligned(const cv::Rect& detected_box,
               int expected_size_px,
               const CameraParams& cam,
               const TargetParams& target) {
    if (expected_size_px <= 0 || detected_box.height <= 0 || detected_box.width <= 0) {
        return false;
    }

    // --- size check ---
    double detected_size = (detected_box.width + detected_box.height) / 2.0;
    double size_diff_ratio = std::abs(detected_size - expected_size_px)
                              / static_cast<double>(expected_size_px);
    bool size_ok = size_diff_ratio <= target.size_tolerance;

    // --- centering check ---
    cv::Point detected_center(
        detected_box.x + detected_box.width / 2,
        detected_box.y + detected_box.height / 2);
    cv::Point frame_center(cam.frame_width_px / 2, cam.frame_height_px / 2);

    double offset = cv::norm(detected_center - frame_center);
    bool center_ok = offset <= target.center_tolerance_px;

    return size_ok && center_ok;
}

// ---------------------------------------------------------------------------
// 6. Drawing: expected box (guide) + detected box (green/red)
// ---------------------------------------------------------------------------
void drawOverlay(cv::Mat& frame,
                  int expected_size_px,
                  const DetectionResult& detection,
                  bool aligned) {
    cv::Point center(frame.cols / 2, frame.rows / 2);

    // Guide box: where the QR SHOULD be at current altitude (always drawn, gray)
    cv::Rect guide_box(
        center.x - expected_size_px / 2,
        center.y - expected_size_px / 2,
        expected_size_px, expected_size_px);
    cv::rectangle(frame, guide_box, cv::Scalar(180, 180, 180), 1);

    // Detected box: green if aligned, red if not
    if (detection.found) {
        cv::Scalar color = aligned ? cv::Scalar(0, 200, 0)   // green (BGR)
                                    : cv::Scalar(0, 0, 255);  // red
        cv::rectangle(frame, detection.detected_box, color, 3);

        std::string label = aligned ? "ALIGNED" : "ADJUST POSITION/ALTITUDE";
        cv::putText(frame, label, cv::Point(detection.detected_box.x,
                    detection.detected_box.y - 10),
                    cv::FONT_HERSHEY_SIMPLEX, 0.6, color, 2);
    }
}

// ---------------------------------------------------------------------------
// Main per-altitude-step cycle
// (call this once per altitude step from your flight control loop)
// ---------------------------------------------------------------------------
bool runDetectionCycle(cv::VideoCapture& capture,
                        const CameraParams& cam,
                        const TargetParams& target,
                        double current_altitude_m,
                        bool show_debug_window = true) {
    // Step 1-2: burst capture, pick sharpest
    cv::Mat best_frame = captureBestOfBurst(capture, /*burst_count=*/5);
    if (best_frame.empty()) {
        std::cerr << "[Error] No usable frame from burst\n";
        return false;
    }

    // Step 3: detect QR
    DetectionResult detection = detectQR(best_frame);

    // Step 4: expected box size for current altitude
    int expected_size = computeExpectedBoxSizePx(cam, target, current_altitude_m);

    // Step 5: alignment
    bool aligned = false;
    if (detection.found) {
        aligned = isAligned(detection.detected_box, expected_size, cam, target);
    }

    // Step 6: overlay
    drawOverlay(best_frame, expected_size, detection, aligned);

    if (show_debug_window) {
        cv::imshow("QR Scan - Altitude " + std::to_string(current_altitude_m) + "m",
                   best_frame);
        cv::waitKey(1);
    }

    std::cout << "[Altitude " << current_altitude_m << "m] "
              << "Expected box: " << expected_size << "px | "
              << "Detected: " << (detection.found ? "YES" : "NO") << " | "
              << "Aligned: " << (aligned ? "GREEN" : "RED") << std::endl;

    // Return whether we have a confident, aligned, decoded read
    return aligned && detection.found && !detection.decoded_text.empty();
}

// ---------------------------------------------------------------------------
// Example usage (wire actual altitude sensor + descent logic here)
// ---------------------------------------------------------------------------
int main() {
    cv::VideoCapture capture(0); // replace 0 with your drone camera source
    if (!capture.isOpened()) {
        std::cerr << "Failed to open camera\n";
        return -1;
    }

    // --- Fill these in from calibration / spec sheet ---
    CameraParams cam;
    cam.focal_length_px = 800.0;   // PLACEHOLDER - calibrate this
    cam.frame_width_px  = 1280;
    cam.frame_height_px = 720;

    TargetParams target;
    target.qr_real_size_m      = 0.20; // e.g. 20cm x 20cm printed QR
    target.size_tolerance      = 0.10; // +/-10%
    target.center_tolerance_px = 40.0;

    double altitude_m = 2.0; // PLACEHOLDER - replace with live sensor reading

    while (true) {
        bool success = runDetectionCycle(capture, cam, target, altitude_m);

        if (success) {
            std::cout << "QR successfully scanned and aligned. Stopping descent.\n";
            break;
        } else {
            // Example descent logic: if not aligned/readable, step down 0.25m
            altitude_m -= 0.25;
            std::cout << "Descending to " << altitude_m << "m and retrying...\n";
            if (altitude_m <= 0.0) {
                std::cerr << "Reached minimum altitude without a valid scan.\n";
                break;
            }
        }
    }

    return 0;
}