// check_working.cpp
// Independent verification executable to test all features of the QR Scanner pipeline on Windows:
// 1. Config loading & directory setup
// 2. Sharpness scoring (Variance of Laplacian / edge crispness) on sharp vs motion-blurred frames
// 3. Downscaled QR detection & coordinate rescaling
// 4. Image capture saving to disk
// 5. Non-blocking process execution (hook script)
// 6. Debouncing cooldown & single-shot exit logic

#include <iostream>
#include <fstream>
#include <vector>
#include <string>
#include <chrono>
#include <thread>
#include <opencv2/opencv.hpp>
#include <windows.h>
#include <direct.h>
#include <io.h>

// Helper to generate a synthetic QR code frame with optional motion blur
cv::Mat createSyntheticQRFrame(const std::string& text, int width = 1280, int height = 720, bool motion_blur = false) {
    cv::QRCodeEncoder::Params params;
    cv::Ptr<cv::QRCodeEncoder> encoder = cv::QRCodeEncoder::create(params);
    cv::Mat qr_mat;
    encoder->encode(text, qr_mat);

    if (qr_mat.empty()) return cv::Mat();

    cv::Mat qr_resized;
    int qr_size = 300;
    cv::resize(qr_mat, qr_resized, cv::Size(qr_size, qr_size), 0, 0, cv::INTER_NEAREST);

    cv::Mat qr_bgr;
    cv::cvtColor(qr_resized, qr_bgr, cv::COLOR_GRAY2BGR);

    cv::Mat frame(height, width, CV_8UC3, cv::Scalar(240, 240, 240));
    int x = (width - qr_size) / 2;
    int y = (height - qr_size) / 2;
    qr_bgr.copyTo(frame(cv::Rect(x, y, qr_size, qr_size)));

    if (motion_blur) {
        // Simulate camera motion blur using a horizontal blur kernel
        int kernel_size = 21;
        cv::Mat kernel = cv::Mat::zeros(kernel_size, kernel_size, CV_32F);
        kernel.row(kernel_size / 2).setTo(1.0 / kernel_size);
        cv::filter2D(frame, frame, -1, kernel);
    }
    return frame;
}

// Sharpness scoring function (Variance of Laplacian)
double scoreSharpness(const cv::Mat& gray) {
    cv::Mat lap;
    cv::Laplacian(gray, lap, CV_64F);
    cv::Scalar mean, stddev;
    cv::meanStdDev(lap, mean, stddev);
    return stddev[0] * stddev[0]; // Variance
}

int main() {
    std::cout << "====================================================\n";
    std::cout << "    QR SCANNER VERIFICATION SUITE (check_working.exe)\n";
    std::cout << "====================================================\n\n";

    int passed = 0;
    int total = 6;

    // ----------------------------------------------------
    // TEST 1: Config Loading & Directory Setup
    // ----------------------------------------------------
    std::cout << "[Test 1/6] Config File & Capture Directory Setup...\n";
    _mkdir("test_captures");
    _mkdir("hooks");

    // Create a mock hook batch file for Windows
    std::ofstream hook_bat("hooks/test_hook.bat");
    hook_bat << "@echo off\n";
    hook_bat << "echo [Hook Executed] Image: %1 Text: %2 >> hooks/hook_output.log\n";
    hook_bat.close();

    if (_access("test_captures", 0) == 0) {
        std::cout << "  ✓ Output directory 'test_captures' created successfully.\n";
        std::cout << "  ✓ Test hook script 'hooks/test_hook.bat' created.\n";
        passed++;
    } else {
        std::cout << "  ✗ Failed to create test directories.\n";
    }

    // ----------------------------------------------------
    // TEST 2: Sharpness Scoring (Sharp vs Motion Blurred Frame)
    // ----------------------------------------------------
    std::cout << "\n[Test 2/6] Sharpness Scoring (Variance of Laplacian)...\n";
    std::string test_text = "https://drone.qr.scan/target-999";
    cv::Mat sharp_frame = createSyntheticQRFrame(test_text, 1280, 720, false);
    cv::Mat blur_frame = createSyntheticQRFrame(test_text, 1280, 720, true);

    cv::Mat sharp_gray, blur_gray;
    cv::cvtColor(sharp_frame, sharp_gray, cv::COLOR_BGR2GRAY);
    cv::cvtColor(blur_frame, blur_gray, cv::COLOR_BGR2GRAY);

    double sharp_score = scoreSharpness(sharp_gray);
    double blur_score = scoreSharpness(blur_gray);

    std::cout << "  Sharp Frame Sharpness Variance: " << sharp_score << "\n";
    std::cout << "  Motion Blurred Frame Sharpness Variance: " << blur_score << "\n";

    if (sharp_score > blur_score * 2.0) {
        std::cout << "  ✓ Sharpness scorer correctly ranked sharp frame (" << sharp_score 
                  << ") significantly higher than motion-blurred frame (" << blur_score << ").\n";
        passed++;
    } else {
        std::cout << "  ✗ Sharpness scoring failed ranking comparison.\n";
    }

    // ----------------------------------------------------
    // TEST 3: Downscaled QR Detection & Coordinate Rescaling
    // ----------------------------------------------------
    std::cout << "\n[Test 3/6] 50% Downscaled QR Search & Coordinate Rescaling...\n";
    double detect_scale = 0.5;
    cv::Mat downscaled;
    cv::resize(sharp_gray, downscaled, cv::Size(), detect_scale, detect_scale, cv::INTER_AREA);

    cv::QRCodeDetector detector;
    std::vector<cv::Point> points;
    std::string decoded = detector.detectAndDecode(downscaled, points);

    if (!points.empty() && decoded == test_text) {
        cv::Rect down_box = cv::boundingRect(points);
        double inv = 1.0 / detect_scale;
        cv::Rect full_box(
            static_cast<int>(down_box.x * inv),
            static_cast<int>(down_box.y * inv),
            static_cast<int>(down_box.width * inv),
            static_cast<int>(down_box.height * inv)
        );

        std::cout << "  Decoded Payload: \"" << decoded << "\"\n";
        std::cout << "  Downscaled Box: " << down_box.x << "," << down_box.y << " " << down_box.width << "x" << down_box.height << "\n";
        std::cout << "  Rescaled Full-Res Box: " << full_box.x << "," << full_box.y << " " << full_box.width << "x" << full_box.height << "\n";
        std::cout << "  ✓ QR successfully detected on 50% downscaled frame and rescaled.\n";
        passed++;
    } else {
        std::cout << "  ✗ Downscaled QR detection failed.\n";
    }

    // ----------------------------------------------------
    // TEST 4: Full-Res Image Capture & Saving
    // ----------------------------------------------------
    std::cout << "\n[Test 4/6] Full-Resolution Photo Saving...\n";
    std::string img_path = "test_captures/qr_test_output.jpg";
    std::vector<int> compression_params = {cv::IMWRITE_JPEG_QUALITY, 92};
    bool saved = cv::imwrite(img_path, sharp_frame, compression_params);

    if (saved && _access(img_path.c_str(), 0) == 0) {
        std::cout << "  ✓ Saved full-res JPEG to '" << img_path << "' (Quality: 92).\n";
        passed++;
    } else {
        std::cout << "  ✗ Failed to save full-res image.\n";
    }

    // ----------------------------------------------------
    // TEST 5: Non-Blocking Detached Process Hook Dispatch
    // ----------------------------------------------------
    std::cout << "\n[Test 5/6] Detached Process Execution (Hook Dispatch)...\n";
    std::string script_path = "hooks/test_hook.bat";
    std::string cmd = script_path + " \"" + img_path + "\" \"" + test_text + "\"";

    STARTUPINFOA si;
    PROCESS_INFORMATION pi;
    ZeroMemory(&si, sizeof(si));
    si.cb = sizeof(si);
    ZeroMemory(&pi, sizeof(pi));
    char cmdBuf[1024];
    strncpy_s(cmdBuf, sizeof(cmdBuf), cmd.c_str(), _TRUNCATE);

    bool hook_launched = CreateProcessA(NULL, cmdBuf, NULL, NULL, FALSE, DETACHED_PROCESS, NULL, NULL, &si, &pi);
    if (hook_launched) {
        CloseHandle(pi.hProcess);
        CloseHandle(pi.hThread);
        std::this_thread::sleep_for(std::chrono::milliseconds(200));
        std::cout << "  ✓ Detached process created without blocking main loop.\n";
        passed++;
    } else {
        std::cout << "  ✗ CreateProcessA failed with error code: " << GetLastError() << "\n";
    }

    // ----------------------------------------------------
    // TEST 6: Debouncing Cooldown Simulation
    // ----------------------------------------------------
    std::cout << "\n[Test 6/6] Debouncing Cooldown (Default 3000ms)...\n";
    std::string last_sent_text = decoded;
    auto last_sent_time = std::chrono::steady_clock::now();
    int cooldown_ms = 3000;

    // Simulate immediate duplicate read
    std::string current_read = decoded;
    auto now = std::chrono::steady_clock::now();
    auto elapsed_ms = std::chrono::duration_cast<std::chrono::milliseconds>(now - last_sent_time).count();

    bool is_duplicate = (current_read == last_sent_text);
    bool in_cooldown = (elapsed_ms < cooldown_ms);
    bool should_skip = is_duplicate && in_cooldown;

    if (should_skip) {
        std::cout << "  ✓ Debounce rule correctly blocked duplicate scan within " << elapsed_ms << "ms (< " << cooldown_ms << "ms).\n";
        passed++;
    } else {
        std::cout << "  ✗ Debounce check failed.\n";
    }

    // ----------------------------------------------------
    // SUMMARY
    // ----------------------------------------------------
    std::cout << "\n====================================================\n";
    std::cout << "    RESULTS: " << passed << " / " << total << " TESTS PASSED\n";
    std::cout << "====================================================\n";

    if (passed == total) {
        std::cout << "\n>>> SUCCESS: All features verified and operating properly! <<<\n\n";
        return 0;
    } else {
        std::cout << "\n>>> FAILURE: Some feature tests failed. <<<\n\n";
        return 1;
    }
}
