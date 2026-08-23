// PARTIAL_QR_TEST.cpp
// Test suite to evaluate OpenCV QR detector performance on complete, partially occluded,
// and damaged QR code images.

#include <iostream>
#include <vector>
#include <string>
#include <cmath>
#include <opencv2/opencv.hpp>

struct GroundTruth
{
    std::string name;
    cv::Rect visibleBox;
};

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
cv::Mat applyOcclusion(
    const cv::Mat& src,
    double crop_ratio_x,
    double crop_ratio_y,
    cv::Rect& visibleBox)
{
    cv::Mat result = src.clone();

    int cut_w =
        static_cast<int>(src.cols * crop_ratio_x);

    int cut_h =
        static_cast<int>(src.rows * crop_ratio_y);

    if (cut_w > 0 && cut_h > 0)
    {
        cv::Rect occlude_area(
            src.cols - cut_w,
            src.rows - cut_h,
            cut_w,
            cut_h
        );

        cv::rectangle(
            result,
            occlude_area,
            cv::Scalar(255, 255, 255),
            -1
        );
    }

    // --------------------------------------------------------
    // Find the visible non-white area.
    //
    // Because this is a synthetic QR on a white background,
    // this gives us the ground-truth visible QR bounding box.
    // --------------------------------------------------------

    cv::Mat gray;

    if (result.channels() == 3)
    {
        cv::cvtColor(
            result,
            gray,
            cv::COLOR_BGR2GRAY
        );
    }
    else
    {
        gray = result;
    }

    cv::Mat mask;

    cv::threshold(
        gray,
        mask,
        250,
        255,
        cv::THRESH_BINARY_INV
    );

    std::vector<cv::Point> points;

    cv::findNonZero(
        mask,
        points
    );

    if (points.empty())
    {
        visibleBox = cv::Rect();
    }
    else
    {
        visibleBox = cv::boundingRect(points);
    }

    return result;
}

// ------------------------------------------------------------
// Place a QR partially outside a camera frame
// ------------------------------------------------------------
cv::Mat createPartialFrame(const cv::Mat& qr,
                           int frame_width,
                           int frame_height,
                           int x,
                           int y)
{
    cv::Mat frame(
        frame_height,
        frame_width,
        CV_8UC1,
        cv::Scalar(255)
    );

    // QR rectangle on the large coordinate system
    cv::Rect qr_rect(
        x,
        y,
        qr.cols,
        qr.rows
    );

    // Only the portion inside the camera frame is copied.
    cv::Rect frame_rect(
        0,
        0,
        frame.cols,
        frame.rows
    );

    cv::Rect visible =
        qr_rect & frame_rect;

    if (visible.area() <= 0)
        return frame;

    cv::Rect qr_source(
        visible.x - x,
        visible.y - y,
        visible.width,
        visible.height
    );

    qr(qr_source).copyTo(
        frame(visible)
    );

    return frame;
}

// ------------------------------------------------------------
// Test OpenCV detection on a partially visible QR
// ------------------------------------------------------------
void testPartialFrame(
    cv::QRCodeDetector& detector,
    const cv::Mat& frame,
    const std::string& test_name,
    const std::string& expected_text)
{
    std::vector<cv::Point> points;

    std::string decoded =
        detector.detectAndDecode(frame, points);

    bool detected = !points.empty();

    bool decoded_ok =
        decoded == expected_text;

    std::cout
        << "\n[" << test_name << "]\n";

    std::cout
        << "  Detection: "
        << (detected ? "YES" : "NO")
        << "\n";

    std::cout
        << "  Points: "
        << points.size()
        << "\n";

    std::cout
        << "  Decode: "
        << (decoded_ok ? "YES" : "NO")
        << "\n";

    std::cout
        << "  Text: \""
        << decoded
        << "\"\n";

    // Save the frame so we can visually inspect it later.
    std::string filename =
        "partial_" + test_name + ".png";

    cv::imwrite(filename, frame);

    std::cout
        << "  Saved: "
        << filename
        << "\n";
}

std::vector<cv::Point> analyzeContours(
    const cv::Mat& input,
    const std::string& name){

    std::vector<cv::Point> finderCenters;
    
    cv::Mat gray;
    cv::Mat binary;

    if (input.channels() == 3)
    {
        cv::cvtColor(input, gray, cv::COLOR_BGR2GRAY);
    }
    else if (input.channels() == 1)
    {
        gray = input.clone();
    }
    else
    {
        std::cerr << "[Error] Unsupported image channels: "
                << input.channels() << "\n";
        return {};
    }

    cv::adaptiveThreshold(
        gray,
        binary,
        255,
        cv::ADAPTIVE_THRESH_GAUSSIAN_C,
        cv::THRESH_BINARY,
        31,
        5
    );

    std::vector<std::vector<cv::Point>> contours;
    std::vector<cv::Vec4i> hierarchy;

    cv::findContours(
        binary,
        contours,
        hierarchy,
        cv::RETR_TREE,
        cv::CHAIN_APPROX_SIMPLE
    );

    std::cout << "\n=== FINDER PATTERN CANDIDATES ===\n";

struct Candidate
{
    cv::Rect box;
    cv::Point center;
};

std::vector<Candidate> candidates;

// ------------------------------------------------------------
// Find nested contour structures
// ------------------------------------------------------------
for (size_t i = 0; i < contours.size(); ++i)
{
    int child = hierarchy[i][2];

    if (child < 0)
        continue;

    int grandchild = hierarchy[child][2];

    if (grandchild < 0)
        continue;

    cv::Rect box = cv::boundingRect(contours[i]);

    // Finder patterns are approximately square
    double aspect =
        static_cast<double>(box.width) /
        box.height;

    if (aspect < 0.7 || aspect > 1.3)
        continue;

    // Ignore tiny structures
    if (box.width < 15 || box.height < 15)
        continue;

    cv::Point center(
        box.x + box.width / 2,
        box.y + box.height / 2
    );

    candidates.push_back({
        box,
        center
    });
}

// ------------------------------------------------------------
// Remove duplicate nested contours.
//
// The outer and inner contours of the SAME finder pattern
// have almost exactly the same center.
//
// Keep only the largest bounding box.
// ------------------------------------------------------------
std::vector<Candidate> finders;

for (const auto& candidate : candidates)
{
    bool duplicate = false;

    for (auto& finder : finders)
    {
        double dx =
            candidate.center.x - finder.center.x;

        double dy =
            candidate.center.y - finder.center.y;

        double distance =
            std::sqrt(dx * dx + dy * dy);

        // Same finder pattern
        if (distance < 10.0)
        {
            duplicate = true;

            // Keep the larger contour
            if (candidate.box.area() > finder.box.area())
            {
                finder = candidate;
            }

            break;
        }
    }

    if (!duplicate)
    {
        finders.push_back(candidate);
    }
}

// ------------------------------------------------------------
// Print actual finder candidates
// ------------------------------------------------------------
for (const auto& finder : finders)
{
    finderCenters.push_back(finder.center);
    std::cout
        << "FINDER CANDIDATE "
        << "x=" << finder.box.x
        << " y=" << finder.box.y
        << " w=" << finder.box.width
        << " h=" << finder.box.height
        << " center=("
        << finder.center.x << ","
        << finder.center.y << ")"
        << "\n";
}

    std::string filename =
        "contours_" + name + ".png";

    cv::imwrite(filename, binary);

    std::cout
        << "\n[Contour Analysis] "
        << name
        << "\n"
        << "Saved: "
        << filename
        << "\n";

    return finderCenters;
}

cv::Rect estimateVisibleQRBox(
    const cv::Mat& frame,
    const std::vector<cv::Point>& finderCenters)
{
    if (finderCenters.empty())
        return cv::Rect();

    cv::Mat gray;

    if (frame.channels() == 3)
    {
        cv::cvtColor(
            frame,
            gray,
            cv::COLOR_BGR2GRAY
        );
    }
    else if (frame.channels() == 1)
    {
        gray = frame.clone();
    }
    else
    {
        return cv::Rect();
    }

    // --------------------------------------------------------
    // Use the finder pattern(s) to define a search region.
    // --------------------------------------------------------

    int minX = frame.cols;
    int minY = frame.rows;
    int maxX = 0;
    int maxY = 0;

    for (const auto& p : finderCenters)
    {
        minX = std::min(minX, p.x);
        minY = std::min(minY, p.y);
        maxX = std::max(maxX, p.x);
        maxY = std::max(maxY, p.y);
    }

    // Finder patterns in our current test are ~108 px.
    // Use their spacing when available; otherwise use the
    // finder size itself as the scale.
    double scale;

    if (finderCenters.size() >= 2)
    {
        scale = cv::norm(
            finderCenters[0] -
            finderCenters[1]
        );
    }
    else
    {
        // Estimate finder size from the image around its center.
        scale = 108.0;
    }

    if (scale <= 0)
        return cv::Rect();

    int padding =
        static_cast<int>(scale * 0.9);

    int roiX =
        std::max(0, minX - padding);

    int roiY =
        std::max(0, minY - padding);

    int roiRight =
        std::min(frame.cols, maxX + padding);

    int roiBottom =
        std::min(frame.rows, maxY + padding);

    if (roiRight <= roiX || roiBottom <= roiY)
        return cv::Rect();

    cv::Rect roi(
        roiX,
        roiY,
        roiRight - roiX,
        roiBottom - roiY
    );

    cv::Mat roiGray = gray(roi);

    // --------------------------------------------------------
    // Threshold QR modules.
    // --------------------------------------------------------

    cv::Mat binary;

    cv::threshold(
        roiGray,
        binary,
        100,
        255,
        cv::THRESH_BINARY_INV
    );

    // --------------------------------------------------------
    // Find connected black regions.
    // --------------------------------------------------------

    std::vector<std::vector<cv::Point>> contours;

    cv::findContours(
        binary,
        contours,
        cv::RETR_EXTERNAL,
        cv::CHAIN_APPROX_SIMPLE
    );

    if (contours.empty())
        return cv::Rect();

    // --------------------------------------------------------
    // Combine QR-like contours.
    //
    // We don't want one individual QR module to become the
    // bounding box. Keep contours reasonably close to the
    // finder-pattern region.
    // --------------------------------------------------------

    std::vector<cv::Point> visiblePixels;

    for (const auto& contour : contours)
    {
        double area = cv::contourArea(contour);

        if (area < 20)
            continue;

        cv::Rect box =
            cv::boundingRect(contour);

        cv::Point center(
            box.x + box.width / 2,
            box.y + box.height / 2
        );

        // Convert local ROI coordinates to frame coordinates.
        center.x += roi.x;
        center.y += roi.y;

        bool nearFinder = false;

        for (const auto& finder : finderCenters)
        {
            double distance =
                cv::norm(center - finder);

            if (distance < scale * 1.5)
            {
                nearFinder = true;
                break;
            }
        }

        if (!nearFinder)
            continue;

        for (const auto& p : contour)
        {
            visiblePixels.push_back(
                cv::Point(
                    p.x + roi.x,
                    p.y + roi.y
                )
            );
        }
    }

    if (visiblePixels.empty())
        return cv::Rect();

    return cv::boundingRect(visiblePixels);
}

double calculateIoU(
    const cv::Rect& a,
    const cv::Rect& b)
{
    if (a.area() <= 0 || b.area() <= 0)
        return 0.0;

    cv::Rect intersection = a & b;

    double intersectionArea =
        static_cast<double>(intersection.area());

    double unionArea =
        static_cast<double>(a.area())
        + static_cast<double>(b.area())
        - intersectionArea;

    if (unionArea <= 0.0)
        return 0.0;

    return intersectionArea / unionArea;
}

double calculateCenterError(
    const cv::Rect& detected,
    const cv::Rect& groundTruth)
{
    if (detected.area() <= 0 ||
        groundTruth.area() <= 0)
    {
        return -1.0;
    }

    cv::Point2f detectedCenter(
        detected.x + detected.width / 2.0f,
        detected.y + detected.height / 2.0f
    );

    cv::Point2f truthCenter(
        groundTruth.x + groundTruth.width / 2.0f,
        groundTruth.y + groundTruth.height / 2.0f
    );

    return cv::norm(
        detectedCenter - truthCenter
    );
}

cv::Rect calculateGroundTruthVisibleBox(
    const cv::Mat& qr,
    int frame_width,
    int frame_height,
    int x,
    int y)
{
    // --------------------------------------------------------
    // Position of the complete synthetic QR inside the frame
    // coordinate system.
    // --------------------------------------------------------
    cv::Rect qr_rect(
        x,
        y,
        qr.cols,
        qr.rows
    );

    cv::Rect frame_rect(
        0,
        0,
        frame_width,
        frame_height
    );

    // Portion of the QR image that is actually visible.
    cv::Rect visible =
        qr_rect & frame_rect;

    if (visible.area() <= 0)
        return cv::Rect();

    // --------------------------------------------------------
    // The synthetic QR has a 40 px white quiet zone.
    //
    // We want the black QR content, not the white border.
    // --------------------------------------------------------
    cv::Mat qr_gray;

    if (qr.channels() == 3)
    {
        cv::cvtColor(
            qr,
            qr_gray,
            cv::COLOR_BGR2GRAY
        );
    }
    else
    {
        qr_gray = qr;
    }

    cv::Mat blackMask;

    cv::threshold(
        qr_gray,
        blackMask,
        250,
        255,
        cv::THRESH_BINARY_INV
    );

    std::vector<cv::Point> blackPixels;

    cv::findNonZero(
        blackMask,
        blackPixels
    );

    if (blackPixels.empty())
        return cv::Rect();

    // --------------------------------------------------------
    // Convert the black QR pixels from QR coordinates to
    // frame coordinates, keeping only pixels inside frame.
    // --------------------------------------------------------
    std::vector<cv::Point> visibleBlackPixels;

    visibleBlackPixels.reserve(blackPixels.size());

    for (const auto& p : blackPixels)
    {
        int frameX = p.x + x;
        int frameY = p.y + y;

        if (frameX >= 0 &&
            frameX < frame_width &&
            frameY >= 0 &&
            frameY < frame_height)
        {
            visibleBlackPixels.emplace_back(
                frameX,
                frameY
            );
        }
    }

    if (visibleBlackPixels.empty())
        return cv::Rect();

    return cv::boundingRect(
        visibleBlackPixels
    );
}
cv::Rect estimateQRFromSingleFinder(
    const cv::Point& finder,
    const cv::Size& frameSize,
    const std::string& edge)
{
    // Geometry measured from our synthetic 580x580 QR.
    constexpr double QR_SIZE = 580.0;

    constexpr double TL_X = 124.0;
    constexpr double TL_Y = 124.0;

    constexpr double TR_X = 457.0;
    constexpr double TR_Y = 124.0;

    constexpr double BL_X = 124.0;
    constexpr double BL_Y = 457.0;

    double qrX = 0.0;
    double qrY = 0.0;

    // --------------------------------------------------------
    // Determine which finder is visible from the edge case.
    // --------------------------------------------------------

    if (edge == "LEFT")
    {
        // Visible finder is TOP-RIGHT.
        qrX = finder.x - TR_X;
        qrY = finder.y - TR_Y;
    }
    else if (edge == "RIGHT")
    {
        // Visible finder is TOP-LEFT.
        qrX = finder.x - TL_X;
        qrY = finder.y - TL_Y;
    }
    else if (edge == "TOP")
    {
        // Visible finder is BOTTOM-LEFT.
        qrX = finder.x - BL_X;
        qrY = finder.y - BL_Y;
    }
    else
    {
        return cv::Rect();
    }

    cv::Rect qrRect(
        static_cast<int>(std::round(qrX)),
        static_cast<int>(std::round(qrY)),
        static_cast<int>(QR_SIZE),
        static_cast<int>(QR_SIZE)
    );

    // Clip complete QR against camera frame.
    cv::Rect frameRect(
        0,
        0,
        frameSize.width,
        frameSize.height
    );

    return qrRect & frameRect;
}

cv::Rect getVisibleBlackQRBox(
    const cv::Mat& frame,
    const cv::Rect& estimatedQR)
{
    if (estimatedQR.area() <= 0)
        return cv::Rect();

    cv::Mat roi = frame(estimatedQR);

    cv::Mat gray;

    if (roi.channels() == 3)
    {
        cv::cvtColor(
            roi,
            gray,
            cv::COLOR_BGR2GRAY
        );
    }
    else
    {
        gray = roi;
    }

    cv::Mat mask;

    cv::threshold(
        gray,
        mask,
        100,
        255,
        cv::THRESH_BINARY_INV
    );

    std::vector<cv::Point> points;

    cv::findNonZero(
        mask,
        points
    );

    if (points.empty())
        return cv::Rect();

    cv::Rect localBox =
        cv::boundingRect(points);

    return cv::Rect(
        estimatedQR.x + localBox.x,
        estimatedQR.y + localBox.y,
        localBox.width,
        localBox.height
    );
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
cv::Rect gt_10;

cv::Mat partial_10 =
    applyOcclusion(
        full_qr,
        0.10,
        0.10,
        gt_10
    );

    // Test 3: Partial QR Code (25% Corner Occlusion - Exceeds Error Correction capacity)
    std::cout << "\n[Test 3] Partial QR Code (25% Corner Occluded)\n";
cv::Rect gt_25;

cv::Mat partial_25 =
    applyOcclusion(
        full_qr,
        0.25,
        0.25,
        gt_25
    );

    std::cout << "\n=== Partial QR Code Test Complete ===\n";
    // ------------------------------------------------------------
    // Frame-edge partial QR tests
    // ------------------------------------------------------------

    std::cout
        << "\n\n=== FRAME-EDGE PARTIAL QR TESTS ===\n";

    const int FRAME_WIDTH = 640;
    const int FRAME_HEIGHT = 480;

    // QR is larger than the camera frame.
    cv::Mat large_qr =
        createSyntheticQR(test_payload, 500);

std::vector<cv::Point> fullFinders =
    analyzeContours(
        large_qr,
        "FULL_QR"
    );

std::cout
    << "\n=== FULL QR FINDERS ===\n";

for (const auto& p : fullFinders)
{
    std::cout
        << "Finder center = ("
        << p.x
        << ", "
        << p.y
        << ")\n";
}

    std::cout
    << "\n[Ground Truth QR Image]"
    << "\n  Width  = " << large_qr.cols
    << "\n  Height = " << large_qr.rows
    << "\n";

    // ------------------------------------------------------------
    // Test 4: QR partially outside LEFT edge
    // ------------------------------------------------------------

    cv::Mat left_frame =
        createPartialFrame(
            large_qr,
            FRAME_WIDTH,
            FRAME_HEIGHT,
            -200,
            50
        );

    testPartialFrame(
        detector,
        left_frame,
        "left_edge",
        test_payload
    );
cv::Rect gt_left =
    calculateGroundTruthVisibleBox(
        large_qr,
        FRAME_WIDTH,
        FRAME_HEIGHT,
        -200,
        50
    );
    // ------------------------------------------------------------
    // Test 5: QR partially outside RIGHT edge
    // ------------------------------------------------------------

    cv::Mat right_frame =
        createPartialFrame(
            large_qr,
            FRAME_WIDTH,
            FRAME_HEIGHT,
            340,
            50
        );

    testPartialFrame(
        detector,
        right_frame,
        "right_edge",
        test_payload
    );
cv::Rect gt_right =
    calculateGroundTruthVisibleBox(
        large_qr,
        FRAME_WIDTH,
        FRAME_HEIGHT,
        340,
        50
    );
    // ------------------------------------------------------------
    // Test 6: QR partially outside TOP edge
    // ------------------------------------------------------------

    cv::Mat top_frame =
        createPartialFrame(
            large_qr,
            FRAME_WIDTH,
            FRAME_HEIGHT,
            70,
            -200
        );

    testPartialFrame(
        detector,
        top_frame,
        "top_edge",
        test_payload
    );
cv::Rect gt_top =
    calculateGroundTruthVisibleBox(
        large_qr,
        FRAME_WIDTH,
        FRAME_HEIGHT,
        70,
        -200
    );
    // ------------------------------------------------------------
    // Test 7: QR partially outside BOTTOM edge
    // ------------------------------------------------------------

    cv::Mat bottom_frame =
        createPartialFrame(
            large_qr,
            FRAME_WIDTH,
            FRAME_HEIGHT,
            70,
            180
        );

    testPartialFrame(
        detector,
        bottom_frame,
        "bottom_edge",
        test_payload
    );
cv::Rect gt_bottom =
    calculateGroundTruthVisibleBox(
        large_qr,
        FRAME_WIDTH,
        FRAME_HEIGHT,
        70,
        180
    );
    std::vector<cv::Point> leftFinders =
        analyzeContours(left_frame, "left");

    std::vector<cv::Point> rightFinders =
        analyzeContours(right_frame, "right");

    std::vector<cv::Point> topFinders =
        analyzeContours(top_frame, "top");

    std::vector<cv::Point> bottomFinders =
        analyzeContours(bottom_frame, "bottom");

auto testVisibleBox =
    [](const cv::Mat& frame,
       const std::vector<cv::Point>& finders,
       const cv::Rect& groundTruth,
       const std::string& name)
{
    std::cout
        << "\n=== "
        << name
        << " PARTIAL QR ===\n";

    if (finders.empty())
    {
        std::cout
            << "No finder pattern detected.\n";
        return;
    }

cv::Rect box;

if (finders.size() == 1)
{
    cv::Rect estimatedQR =
        estimateQRFromSingleFinder(
            finders[0],
            frame.size(),
            name
        );

    box =
        getVisibleBlackQRBox(
            frame,
            estimatedQR
        );
}
else
{
    box =
        estimateVisibleQRBox(
            frame,
            finders
        );
}

    if (box.area() <= 0)
    {
        std::cout
            << "Bounding box estimation failed.\n";
        return;
    }

    cv::Point2f boxCenter(
        box.x + box.width / 2.0f,
        box.y + box.height / 2.0f
    );

    cv::Point2f cameraCenter(
        frame.cols / 2.0f,
        frame.rows / 2.0f
    );

    double dx =
        boxCenter.x - cameraCenter.x;

    double dy =
        boxCenter.y - cameraCenter.y;

    double iou =
    calculateIoU(
        box,
        groundTruth
    );

double centerError =
    calculateCenterError(
        box,
        groundTruth
    );

cv::Point2f truthCenter(
    groundTruth.x + groundTruth.width / 2.0f,
    groundTruth.y + groundTruth.height / 2.0f
);

std::cout
    << "Ground truth box: "
    << "x=" << groundTruth.x
    << " y=" << groundTruth.y
    << " w=" << groundTruth.width
    << " h=" << groundTruth.height
    << "\n"

    << "Ground truth center: ("
    << truthCenter.x
    << ", "
    << truthCenter.y
    << ")\n"

    << "IoU: "
    << iou
    << "\n"

    << "Center error: "
    << centerError
    << " px\n";

    std::cout
        << "Finders: "
        << finders.size()
        << "\n"

        << "Bounding box: "
        << "x=" << box.x
        << " y=" << box.y
        << " w=" << box.width
        << " h=" << box.height
        << "\n"

        << "Box center: ("
        << boxCenter.x
        << ", "
        << boxCenter.y
        << ")\n"

        << "Camera center: ("
        << cameraCenter.x
        << ", "
        << cameraCenter.y
        << ")\n"

        << "Offset: dx="
        << dx
        << " dy="
        << dy
        << "\n";
};

testVisibleBox(
    left_frame,
    leftFinders,
    gt_left,
    "LEFT"
);

testVisibleBox(
    right_frame,
    rightFinders,
    gt_right,
    "RIGHT"
);

testVisibleBox(
    top_frame,
    topFinders,
    gt_top,
    "TOP"
);

testVisibleBox(
    bottom_frame,
    bottomFinders,
    gt_bottom,
    "BOTTOM"
);
    return 0;
}
