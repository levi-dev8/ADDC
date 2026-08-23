// BW_PATTERN_TEST.cpp
//
// Experimental black/white structure detector.
// IMPORTANT:
// This does NOT use QRCodeDetector.
// It only analyzes black/white transitions and connected regions.
//
// Goal:
// Determine whether a partially visible QR can be located purely
// from its black/white visual structure.

#include <iostream>
#include <vector>
#include <string>
#include <cmath>
#include <algorithm>

#include <opencv2/opencv.hpp>


// ============================================================
// Convert image into a black/white mask
// ============================================================

cv::Mat makeBWMask(const cv::Mat& input)
{
    cv::Mat gray;

    if (input.channels() == 3)
    {
        cv::cvtColor(
            input,
            gray,
            cv::COLOR_BGR2GRAY
        );
    }
    else if (input.channels() == 4)
    {
        cv::cvtColor(
            input,
            gray,
            cv::COLOR_BGRA2GRAY
        );
    }
    else
    {
        gray = input.clone();
    }

    cv::Mat bw;

    // Black becomes 255.
    // White becomes 0.
    cv::threshold(
        gray,
        bw,
        128,
        255,
        cv::THRESH_BINARY_INV
    );

    return bw;
}


// ============================================================
// Find black connected regions
// ============================================================

std::vector<cv::Rect> findBlackRegions(
    const cv::Mat& bw)
{
    std::vector<std::vector<cv::Point>> contours;

    cv::findContours(
        bw,
        contours,
        cv::RETR_EXTERNAL,
        cv::CHAIN_APPROX_SIMPLE
    );

    std::vector<cv::Rect> regions;

    for (const auto& contour : contours)
    {
        double area = cv::contourArea(contour);

        if (area < 20)
            continue;

        cv::Rect box =
            cv::boundingRect(contour);

        if (box.width < 3 || box.height < 3)
            continue;

        regions.push_back(box);
    }

    return regions;
}


// ============================================================
// Detect black-white-black transitions.
//
// A QR finder pattern contains:
//
// BLACK
// WHITE
// BLACK
// WHITE
// BLACK
//
// We scan horizontal and vertical lines looking for this
// alternating structure.
// ============================================================

bool hasFinderLikePattern(
    const cv::Mat& bw,
    const cv::Point& center,
    int radius)
{
    if (center.x - radius < 0 ||
        center.x + radius >= bw.cols ||
        center.y - radius < 0 ||
        center.y + radius >= bw.rows)
    {
        return false;
    }

    // --------------------------------------------------------
    // Horizontal scan
    // --------------------------------------------------------

    std::vector<int> horizontal;

    for (int x = center.x - radius;
         x <= center.x + radius;
         ++x)
    {
        uchar pixel =
            bw.at<uchar>(center.y, x);

        horizontal.push_back(
            pixel > 128 ? 1 : 0
        );
    }

    // --------------------------------------------------------
    // Vertical scan
    // --------------------------------------------------------

    std::vector<int> vertical;

    for (int y = center.y - radius;
         y <= center.y + radius;
         ++y)
    {
        uchar pixel =
            bw.at<uchar>(y, center.x);

        vertical.push_back(
            pixel > 128 ? 1 : 0
        );
    }

    // --------------------------------------------------------
    // Count transitions.
    //
    // A finder pattern should contain several alternating
    // black/white regions.
    // --------------------------------------------------------

    auto countTransitions =
        [](const std::vector<int>& values)
        {
            int transitions = 0;

            for (size_t i = 1;
                 i < values.size();
                 ++i)
            {
                if (values[i] != values[i - 1])
                    transitions++;
            }

            return transitions;
        };

    int horizontalTransitions =
        countTransitions(horizontal);

    int verticalTransitions =
        countTransitions(vertical);

    // Require sufficient black/white alternation.
    return
        horizontalTransitions >= 4 &&
        verticalTransitions >= 4;
}


// ============================================================
// Analyze black regions and search for finder-like structure
// ============================================================

std::vector<cv::Point> findBWPatternCandidates(
    const cv::Mat& bw)
{
    std::vector<cv::Rect> regions =
        findBlackRegions(bw);

    std::vector<cv::Point> candidates;

    for (const auto& box : regions)
    {
        cv::Point center(
            box.x + box.width / 2,
            box.y + box.height / 2
        );

        int radius =
            std::max(
                5,
                std::min(
                    box.width,
                    box.height
                ) / 2
            );

    if (hasFinderLikePattern(
            bw,
            center,
            radius))
    {
        std::cout
            << "\nCandidate:"
            << " x=" << center.x
            << " y=" << center.y
            << " radius=" << radius;

        candidates.push_back(center);
    }
    }

    return candidates;
}


// ============================================================
// Draw detected candidates
// ============================================================

cv::Mat drawCandidates(
    const cv::Mat& original,
    const std::vector<cv::Point>& candidates)
{
    cv::Mat output;

    if (original.channels() == 1)
        cv::cvtColor(
            original,
            output,
            cv::COLOR_GRAY2BGR
        );
    else
        output = original.clone();

    for (const auto& p : candidates)
    {
        cv::circle(
            output,
            p,
            15,
            cv::Scalar(0, 0, 255),
            2
        );

        cv::putText(
            output,
            "BW",
            cv::Point(
                p.x + 10,
                p.y
            ),
            cv::FONT_HERSHEY_SIMPLEX,
            0.5,
            cv::Scalar(0, 0, 255),
            1
        );
    }

    return output;
}
cv::Rect makeCandidateBoundingBox(
    const std::vector<cv::Point>& candidates,
    const cv::Size& frameSize)
{
    if (candidates.empty())
        return cv::Rect();

    int minX = frameSize.width;
    int minY = frameSize.height;
    int maxX = 0;
    int maxY = 0;

    for (const auto& p : candidates)
    {
        minX = std::min(minX, p.x);
        minY = std::min(minY, p.y);
        maxX = std::max(maxX, p.x);
        maxY = std::max(maxY, p.y);
    }

    // Padding around the detected pattern group.
    int padding = 100;

    minX = std::max(0, minX - padding);
    minY = std::max(0, minY - padding);
    maxX = std::min(frameSize.width - 1, maxX + padding);
    maxY = std::min(frameSize.height - 1, maxY + padding);

    return cv::Rect(
        minX,
        minY,
        maxX - minX + 1,
        maxY - minY + 1
    );
}

cv::Rect estimateQRFromFinder(
    const cv::Mat& bw,
    const cv::Point& finderCenter)
{
    // --------------------------------------------------------
    // 1. Estimate finder size
    // --------------------------------------------------------

    const int SEARCH_RADIUS = 120;

    int x0 = std::max(
        0,
        finderCenter.x - SEARCH_RADIUS
    );

    int y0 = std::max(
        0,
        finderCenter.y - SEARCH_RADIUS
    );

    int x1 = std::min(
        bw.cols,
        finderCenter.x + SEARCH_RADIUS
    );

    int y1 = std::min(
        bw.rows,
        finderCenter.y + SEARCH_RADIUS
    );

    cv::Rect searchRect(
        x0,
        y0,
        x1 - x0,
        y1 - y0
    );

    cv::Mat local = bw(searchRect).clone();

    // --------------------------------------------------------
    // 2. Dilate black modules.
    //
    // This groups nearby QR modules into larger regions.
    // --------------------------------------------------------

    int kernelSize = 7;

    cv::Mat kernel =
        cv::getStructuringElement(
            cv::MORPH_RECT,
            cv::Size(
                kernelSize,
                kernelSize
            )
        );

    cv::Mat dilated;

    cv::dilate(
        local,
        dilated,
        kernel
    );

    // --------------------------------------------------------
    // 3. Find connected regions.
    // --------------------------------------------------------

    std::vector<std::vector<cv::Point>> contours;

    cv::findContours(
        dilated,
        contours,
        cv::RETR_EXTERNAL,
        cv::CHAIN_APPROX_SIMPLE
    );

    if (contours.empty())
        return cv::Rect();

    // --------------------------------------------------------
    // 4. Convert finder position to local coordinates.
    // --------------------------------------------------------

    cv::Point localFinder(
        finderCenter.x - searchRect.x,
        finderCenter.y - searchRect.y
    );

    // --------------------------------------------------------
    // 5. Find regions near the finder.
    // --------------------------------------------------------

    std::vector<cv::Rect> nearby;

    for (const auto& contour : contours)
    {
        cv::Rect r =
            cv::boundingRect(contour);

        cv::Point center(
            r.x + r.width / 2,
            r.y + r.height / 2
        );

        double distance =
            cv::norm(
                center -
                localFinder
            );

        // Ignore tiny noise.
        if (r.area() < 50)
            continue;

        // Keep regions reasonably close
        // to the finder.
        if (distance < 180)
        {
            nearby.push_back(r);
        }
    }

    if (nearby.empty())
        return cv::Rect();

    // --------------------------------------------------------
    // 6. Combine nearby QR structure.
    // --------------------------------------------------------

    int minX = bw.cols;
    int minY = bw.rows;
    int maxX = 0;
    int maxY = 0;

    for (const auto& r : nearby)
    {
        int globalX =
            r.x + searchRect.x;

        int globalY =
            r.y + searchRect.y;

        minX =
            std::min(
                minX,
                globalX
            );

        minY =
            std::min(
                minY,
                globalY
            );

        maxX =
            std::max(
                maxX,
                globalX + r.width
            );

        maxY =
            std::max(
                maxY,
                globalY + r.height
            );
    }

    if (maxX <= minX ||
        maxY <= minY)
    {
        return cv::Rect();
    }

    // --------------------------------------------------------
    // 7. Add a small safety margin.
    // --------------------------------------------------------

    int padding = 20;

    minX =
        std::max(
            0,
            minX - padding
        );

    minY =
        std::max(
            0,
            minY - padding
        );

    maxX =
        std::min(
            bw.cols,
            maxX + padding
        );

    maxY =
        std::min(
            bw.rows,
            maxY + padding
        );

    return cv::Rect(
        minX,
        minY,
        maxX - minX,
        maxY - minY
    );
}

// ============================================================
// Main
// ============================================================

int main()
{
    std::cout
        << "========================================\n"
        << " BLACK / WHITE PATTERN EXPERIMENT\n"
        << "========================================\n\n";

    cv::VideoCapture cap(0);

    if (!cap.isOpened())
    {
        std::cerr
            << "[ERROR] Cannot open camera.\n";

        return 1;
    }

    std::cout
        << "Camera opened.\n"
        << "Press Q to quit.\n\n";

    while (true)
    {
        cv::Mat frame;

        cap >> frame;

        if (frame.empty())
            break;

        // ----------------------------------------------------
        // Create pure black/white representation.
        // ----------------------------------------------------

        cv::Mat bw =
            makeBWMask(frame);

        // ----------------------------------------------------
        // Find visual black/white pattern candidates.
        // ----------------------------------------------------

std::vector<cv::Point> candidates =
    findBWPatternCandidates(bw);

cv::Mat debug =
    drawCandidates(
        frame,
        candidates
    );

// --------------------------------------------------------
// Estimate QR region from the first detected finder.
// --------------------------------------------------------

if (!candidates.empty())
{
    cv::Point finder =
        candidates[0];

    cv::Rect qrBox =
        estimateQRFromFinder(
            bw,
            finder
        );

    if (qrBox.area() > 0)
    {
        cv::rectangle(
            debug,
            qrBox,
            cv::Scalar(0, 255, 0),
            3
        );

        // --------------------------------------------------------
        // Mark center of detected QR region
        // --------------------------------------------------------

        cv::Point qrCenter(
            qrBox.x + qrBox.width / 2,
            qrBox.y + qrBox.height / 2
        );

        // Draw center point
        cv::circle(
            debug,
            qrCenter,
            7,
            cv::Scalar(255, 0, 0),
            -1
        );

        // Print center coordinates
        std::cout
            << "\nQR Center: ("
            << qrCenter.x
            << ", "
            << qrCenter.y
            << ")"
            << std::endl;

        cv::putText(
            debug,
            "QR REGION",
            cv::Point(
                qrBox.x,
                std::max(
                    25,
                    qrBox.y - 10
                )
            ),
            cv::FONT_HERSHEY_SIMPLEX,
            0.7,
            cv::Scalar(0, 255, 0),
            2
        );

        std::cout
            << "\nQR region: "
            << "x=" << qrBox.x
            << " y=" << qrBox.y
            << " w=" << qrBox.width
            << " h=" << qrBox.height
            << std::endl;
    }
}

// ----------------------------------------------------
// Draw bounding box around detected BW structure
// ----------------------------------------------------

cv::Rect candidateBox =
    makeCandidateBoundingBox(
        candidates,
        frame.size()
    );

if (candidateBox.area() > 0)
{
    cv::rectangle(
        debug,
        candidateBox,
        cv::Scalar(0, 255, 0),
        3
    );

    cv::putText(
        debug,
        "BW REGION",
        cv::Point(
            candidateBox.x,
            std::max(25, candidateBox.y - 10)
        ),
        cv::FONT_HERSHEY_SIMPLEX,
        0.7,
        cv::Scalar(0, 255, 0),
        2
    );
};

        // ----------------------------------------------------
        // Display information.
        // ----------------------------------------------------

        std::cout
            << "\rBW candidates: "
            << candidates.size()
            << "        "
            << std::flush;

        // ----------------------------------------------------
        // Display windows.
        // ----------------------------------------------------

        cv::imshow(
            "Original",
            frame
        );

        cv::imshow(
            "Black White Mask",
            bw
        );

        cv::imshow(
            "BW Pattern Candidates",
            debug
        );

        char key =
            static_cast<char>(
                cv::waitKey(1)
            );

        if (key == 'q' ||
            key == 'Q' ||
            key == 27)
        {
            break;
        }
    }

    cap.release();

    cv::destroyAllWindows();

    std::cout << "\n\nDone.\n";

    return 0;
}
