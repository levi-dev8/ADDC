// PARTIAL_QR_TEST.cpp
// Test suite to evaluate OpenCV QR detector performance on complete, partially occluded,
// and damaged QR code images.

#include <iostream>
#include <vector>
#include <string>
#include <opencv2/opencv.hpp>

// Create a synthetic image containing a QR code pattern or mock bounding box
cv::Mat createSyntheticQR(const std::string& text, int size_px = 300) {
    cv::QRCodeEncoder::Params params;
    cv::Ptr<cv::QRCodeEncoder> encoder = cv::QRCodeEncoder::create(params);
    cv::Mat qr_mat;
    encoder->encode(text, qr_mat);

    if (qr_mat.empty()) {
        std::cerr << "[Error] Failed to encode QR text: " << text << "\n";
        return cv::Mat();
    }

    cv::Mat render;
    cv::resize(qr_mat, render, cv::Size(size_px, size_px), 0, 0, cv::INTER_NEAREST);

    // Add white border padding around QR code (quiet zone)
    int border = 40;
    cv::Mat padded;
    cv::copyMakeBorder(render, padded, border, border, border, border, cv::BORDER_CONSTANT, cv::Scalar(255, 255, 255));
    return padded;
}

// Apply rectangular occlusion/cutoff to simulate a partial QR code
cv::Mat applyOcclusion(const cv::Mat& src, double crop_ratio_x, double crop_ratio_y) {
    cv::Mat result = src.clone();
    int cut_w = static_cast<int>(src.cols * crop_ratio_x);
    int cut_h = static_cast<int>(src.rows * crop_ratio_y);

    if (cut_w > 0 && cut_h > 0) {
        cv::Rect occlude_area(src.cols - cut_w, src.rows - cut_h, cut_w, cut_h);
        cv::rectangle(result, occlude_area, cv::Scalar(255, 255, 255), -1); // cover with white
    }
    return result;
}

int main() {
    std::cout << "=== Partial / Occluded QR Code Detection Test ===\n\n";

    std::string test_payload = "https://drone.qr.scanner.test/target-12345";
    std::cout << "[Info] Generating synthetic QR code for payload: \"" << test_payload << "\"\n";

    cv::Mat full_qr = createSyntheticQR(test_payload, 300);
    if (full_qr.empty()) {
        std::cerr << "[Fatal] Synthetic QR creation failed.\n";
        return 1;
    }

    cv::QRCodeDetector detector;

    // Test 1: Intact QR Code
    std::cout << "\n[Test 1] Intact Full QR Code\n";
    std::vector<cv::Point> points;
    std::string decoded = detector.detectAndDecode(full_qr, points);
    bool pass1 = (!points.empty() && decoded == test_payload);
    std::cout << "  Status: " << (pass1 ? "PASS" : "FAIL")
              << " | Points: " << points.size()
              << " | Text: \"" << decoded << "\"\n";

    // Test 2: Partial QR Code (10% Corner Occlusion)
    std::cout << "\n[Test 2] Partial QR Code (10% Corner Occluded)\n";
    cv::Mat partial_10 = applyOcclusion(full_qr, 0.10, 0.10);
    points.clear();
    decoded = detector.detectAndDecode(partial_10, points);
    std::cout << "  Status: " << (!points.empty() ? "DETECTED" : "NOT DETECTED")
              << " | Points: " << points.size()
              << " | Text: \"" << decoded << "\"\n";

    // Test 3: Partial QR Code (25% Corner Occlusion - Exceeds Error Correction capacity)
    std::cout << "\n[Test 3] Partial QR Code (25% Corner Occluded)\n";
    cv::Mat partial_25 = applyOcclusion(full_qr, 0.25, 0.25);
    points.clear();
    decoded = detector.detectAndDecode(partial_25, points);
    std::cout << "  Status: " << (!points.empty() ? "DETECTED" : "NOT DETECTED")
              << " | Points: " << points.size()
              << " | Text: \"" << decoded << "\"\n";

    std::cout << "\n=== Partial QR Code Test Complete ===\n";
    return 0;
}
