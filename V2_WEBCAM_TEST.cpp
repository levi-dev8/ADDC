// V2_WEBCAM_TEST.cpp
// Same logic as V2_TEST.cpp, but wired to a REAL webcam feed via OpenCV
// instead of the stub cv::Mat. Use this to sanity-check detection +
// alignment logic on your laptop before deploying to the Pi 4.
//
// Requires a real OpenCV install (this is NOT the stub build anymore).
//
// ---- Build instructions ----
//
// Windows (vcpkg):
//   vcpkg install opencv4
//   cl V2_WEBCAM_TEST.cpp /EHsc /I <vcpkg>\installed\x64-windows\include ^
//       /link /LIBPATH:<vcpkg>\installed\x64-windows\lib opencv_world4*.lib
//   (or open in Visual Studio and link against opencv_world4XX.lib)
//
// Linux / Pi 4 / WSL:
//   sudo apt install libopencv-dev
//   g++ -O2 V2_WEBCAM_TEST.cpp -o v2webcam `pkg-config --cflags --libs opencv4`
//   ./v2webcam
//
// macOS (Homebrew):
//   brew install opencv
//   g++ -O2 V2_WEBCAM_TEST.cpp -o v2webcam `pkg-config --cflags --libs opencv4`
//
// Controls: press 'q' or ESC in the video window to quit.

#include <iostream>
#include <string>
#include <cmath>
#include <opencv2/opencv.hpp>

// ---------------------------------------------------------------------------
// Camera intrinsic + physical target parameters
// ---------------------------------------------------------------------------
struct CameraParams {
    double focal_length_px;
    int frame_width_px;
    int frame_height_px;

    bool validate() const {
        if (focal_length_px <= 0 || frame_width_px <= 0 || frame_height_px <= 0) {
            std::cerr << "[Error] Invalid camera params\n";
            return false;
        }
        return true;
    }
};

struct TargetParams {
    double qr_real_size_m;
    double size_tolerance;
    double center_tolerance_px;

    bool validate() const {
        if (qr_real_size_m <= 0 || size_tolerance < 0 || center_tolerance_px < 0) {
            std::cerr << "[Error] Invalid target params\n";
            return false;
        }
        return true;
    }
};

// ---------------------------------------------------------------------------
// Expected box size given altitude (same math as V2_TEST.cpp)
// ---------------------------------------------------------------------------
bool computeExpectedBoxSizePx(const CameraParams& cam,
                               const TargetParams& target,
                               double altitude_m,
                               double& out_size_px) {
    const double ALTITUDE_MIN = 0.01;
    const double ALTITUDE_MAX = 100.0;

    if (altitude_m < ALTITUDE_MIN || altitude_m > ALTITUDE_MAX) {
        std::cerr << "[Error] Altitude out of range: " << altitude_m << "m\n";
        return false;
    }

    double size_px = (target.qr_real_size_m * cam.focal_length_px) / altitude_m;

    if (!std::isfinite(size_px)) {
        std::cerr << "[Error] Size calculation produced non-finite result\n";
        return false;
    }

    out_size_px = size_px;
    return true;
}

// ---------------------------------------------------------------------------
// Alignment check against the detected QR bounding box
// ---------------------------------------------------------------------------
bool isAligned(const cv::Rect& detected_box,
               double expected_size_px,
               const TargetParams& target,
               double& out_size_diff_ratio) {
    if (expected_size_px <= 0.0 || detected_box.height == 0) return false;

    double aspect_ratio = static_cast<double>(detected_box.width) / detected_box.height;
    if (aspect_ratio < 0.8 || aspect_ratio > 1.25) {
        return false;
    }

    double detected_size = (detected_box.width + detected_box.height) / 2.0;
    out_size_diff_ratio = std::abs(detected_size - expected_size_px) / expected_size_px;
    return out_size_diff_ratio <= target.size_tolerance;
}

int main() {
    std::cout << "=== Drone QR Scanner - LIVE WEBCAM TEST ===\n\n";

    // --- Open the default webcam (index 0). Change to 1/2 if you have
    //     multiple cameras and the wrong one opens. ---
    cv::VideoCapture cap(0);
    if (!cap.isOpened()) {
        std::cerr << "[Error] Could not open webcam (index 0). "
                     "Try index 1 if you have an external camera.\n";
        return 1;
    }

    int frame_w = static_cast<int>(cap.get(cv::CAP_PROP_FRAME_WIDTH));
    int frame_h = static_cast<int>(cap.get(cv::CAP_PROP_FRAME_HEIGHT));
    if (frame_w <= 0) frame_w = 1280;
    if (frame_h <= 0) frame_h = 720;

    // --- Fill these in for YOUR webcam / target once you know them ---
    CameraParams cam;
    cam.focal_length_px = 800.0;   // placeholder — calibrate for real use
    cam.frame_width_px = frame_w;
    cam.frame_height_px = frame_h;

    TargetParams target;
    target.qr_real_size_m = 0.20;      // physical QR code side length, meters
    target.size_tolerance = 0.15;
    target.center_tolerance_px = 100.0;

    if (!cam.validate() || !target.validate()) {
        return 1;
    }

    // Assume a fixed test altitude since a laptop webcam has no altimeter.
    // Change this to whatever distance (in meters) you hold the QR code from
    // the camera during testing.
    double test_altitude_m = 0.5;
    double expected_size_px = 0.0;
    if (!computeExpectedBoxSizePx(cam, target, test_altitude_m, expected_size_px)) {
        return 1;
    }
    std::cout << "Expected QR box size at " << test_altitude_m << "m: "
              << expected_size_px << " px\n";
    std::cout << "Press 'q' or ESC to quit.\n\n";

    cv::QRCodeDetector qr_detector;
    cv::Mat frame;
    const std::string window_name = "V2 Webcam Test - QR Alignment";

    while (true) {
        cap >> frame;
        if (frame.empty()) {
            std::cerr << "[Warn] Empty frame grabbed, skipping\n";
            continue;
        }

        std::string decoded_text;
        std::vector<cv::Point> points;
        bool found = qr_detector.detect(frame, points);

        if (found && !points.empty()) {
            decoded_text = qr_detector.decode(frame, points);

            cv::Rect box = cv::boundingRect(points);
            double size_diff_ratio = 0.0;
            bool aligned = isAligned(box, expected_size_px, target, size_diff_ratio);

            cv::rectangle(frame, box, aligned ? cv::Scalar(0, 255, 0) : cv::Scalar(0, 0, 255), 2);

            std::string status = aligned ? "ALIGNED" : "NOT ALIGNED";
            cv::putText(frame, status, cv::Point(box.x, box.y - 10),
                        cv::FONT_HERSHEY_SIMPLEX, 0.7,
                        aligned ? cv::Scalar(0, 255, 0) : cv::Scalar(0, 0, 255), 2);

            if (!decoded_text.empty()) {
                cv::putText(frame, "Data: " + decoded_text, cv::Point(10, 30),
                            cv::FONT_HERSHEY_SIMPLEX, 0.6, cv::Scalar(255, 255, 0), 2);
            }

            std::cout << "[Frame] found=1 aligned=" << aligned
                      << " box=" << box.width << "x" << box.height
                      << " size_diff=" << size_diff_ratio
                      << " text=\"" << decoded_text << "\"\n";
        } else {
            cv::putText(frame, "No QR detected", cv::Point(10, 30),
                        cv::FONT_HERSHEY_SIMPLEX, 0.6, cv::Scalar(0, 0, 255), 2);
        }

        cv::imshow(window_name, frame);
        int key = cv::waitKey(1) & 0xFF;
        if (key == 'q' || key == 27) break; // 'q' or ESC
    }

    cap.release();
    cv::destroyAllWindows();
    std::cout << "\n=== Webcam test ended ===\n";
    return 0;
}
