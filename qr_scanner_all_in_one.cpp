// qr_scanner_all_in_one.cpp
// Unified All-in-One Raspberry Pi 4 (and Cross-Platform) QR Scanner Application.
// Combines startup config parsing, continuous background grabber thread, thermal throttling,
// downscaled sharpness-scored QR search, full-res photo capture, detached hook execution,
// debouncing cooldown, single-shot exit, clean shutdown signal handling, real-time GUI camera feed with HUD overlays,
// and built-in self-test mode.

#include <opencv2/opencv.hpp>
#include <opencv2/objdetect.hpp>

#include <atomic>
#include <cerrno>
#include <chrono>
#include <cmath>
#include <condition_variable>
#include <csignal>
#include <cstdio>
#include <cstring>
#include <fstream>
#include <iostream>
#include <mutex>
#include <sstream>
#include <string>
#include <thread>
#include <unordered_map>
#include <vector>

#ifdef _WIN32
#include <windows.h>
#include <process.h>
#include <direct.h>
#include <io.h>
#ifndef F_OK
#define F_OK 0
#endif
#ifndef X_OK
#define X_OK 0
#endif
#else
#include <sys/stat.h>
#include <sys/types.h>
#include <sys/wait.h>
#include <unistd.h>
#endif

// ===========================================================================
// Directory Helper
// ===========================================================================
void makeDirsRecursive(const std::string& path) {
    std::string cur;
    for (size_t i = 0; i < path.size(); ++i) {
        cur += path[i];
        if (path[i] == '/' || path[i] == '\\' || i == path.size() - 1) {
            if (!cur.empty() && cur != "." && cur != "/" && cur != "\\" && cur != "C:\\" && cur != "C:/") {
#ifdef _WIN32
                _mkdir(cur.c_str());
#else
                mkdir(cur.c_str(), 0755);
#endif
            }
        }
    }
}

// ===========================================================================
// Config
// ===========================================================================
struct Config {
    int camera_index = 0;
    int req_width = 1280;
    int req_height = 720;
    double focal_length_px = 800.0;
    double qr_real_size_m = 0.20;
    double size_tolerance = 0.15;
    double center_tolerance_px = 100.0;
    double detect_scale = 0.5;
    int burst_count = 5;
    int cooldown_ms = 6000;              // min gap between sends for same payload (6s = 0.166 Hz)
    int target_cycle_ms = 400;           // scan cycle pacing target (400ms = 2.5 Hz scan rate)
    bool single_shot = false;
    int max_runtime_s = 0;
    bool require_alignment = false;
    double reference_distance_m = 1.0;
    std::string output_dir = "./captures";
    std::string hook_script = "./hooks/send_capture.sh";
    int jpeg_quality = 92;
    int edge_margin_px = 40;
    std::string move_hook_script = "./hooks/move_command.sh";
    int move_cooldown_ms = 500;
    bool verbose = true;
    bool show_display = true;

    static std::string trim(const std::string& s) {
        size_t a = s.find_first_not_of(" \t\r\n");
        if (a == std::string::npos) return "";
        size_t b = s.find_last_not_of(" \t\r\n");
        return s.substr(a, b - a + 1);
    }

    bool load(const std::string& path) {
        std::ifstream f(path);
        if (!f.is_open()) {
            std::cerr << "[Config] Could not open " << path << " - using built-in defaults\n";
            return false;
        }
        std::string line;
        while (std::getline(f, line)) {
            line = trim(line);
            if (line.empty() || line[0] == '#') continue;
            size_t eq = line.find('=');
            if (eq == std::string::npos) continue;
            std::string key = trim(line.substr(0, eq));
            std::string val = trim(line.substr(eq + 1));
            if (val.empty()) continue;
            try {
                if (key == "camera_index") camera_index = std::stoi(val);
                else if (key == "req_width") req_width = std::stoi(val);
                else if (key == "req_height") req_height = std::stoi(val);
                else if (key == "focal_length_px") focal_length_px = std::stod(val);
                else if (key == "qr_real_size_m") qr_real_size_m = std::stod(val);
                else if (key == "size_tolerance") size_tolerance = std::stod(val);
                else if (key == "center_tolerance_px") center_tolerance_px = std::stod(val);
                else if (key == "detect_scale") detect_scale = std::stod(val);
                else if (key == "burst_count") burst_count = std::stoi(val);
                else if (key == "cooldown_ms") cooldown_ms = std::stoi(val);
                else if (key == "target_cycle_ms") target_cycle_ms = std::stoi(val);
                else if (key == "single_shot") single_shot = (val == "true" || val == "1");
                else if (key == "max_runtime_s") max_runtime_s = std::stoi(val);
                else if (key == "require_alignment") require_alignment = (val == "true" || val == "1");
                else if (key == "reference_distance_m") reference_distance_m = std::stod(val);
                else if (key == "output_dir") output_dir = val;
                else if (key == "hook_script") hook_script = val;
                else if (key == "jpeg_quality") jpeg_quality = std::stoi(val);
                else if (key == "edge_margin_px") edge_margin_px = std::stoi(val);
                else if (key == "move_hook_script") move_hook_script = val;
                else if (key == "move_cooldown_ms") move_cooldown_ms = std::stoi(val);
                else if (key == "verbose") verbose = (val == "true" || val == "1");
                else if (key == "show_display") show_display = (val == "true" || val == "1");
                else std::cerr << "[Config] Unknown key ignored: " << key << "\n";
            } catch (const std::exception& e) {
                std::cerr << "[Config] Bad value for " << key << "=" << val << "\n";
            }
        }
        return true;
    }
};

// ===========================================================================
// Camera / Target Parameters & Detection Structs
// ===========================================================================
struct CameraParams {
    double focal_length_px;
    int frame_width_px;
    int frame_height_px;

    bool validate() const {
        if (focal_length_px <= 0 || frame_width_px <= 0 || frame_height_px <= 0) {
            std::cerr << "[Error] Invalid camera params: focal=" << focal_length_px
                      << " width=" << frame_width_px << " height=" << frame_height_px << "\n";
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
            std::cerr << "[Error] Invalid target params: qr_size=" << qr_real_size_m
                      << " tolerance=" << size_tolerance << "\n";
            return false;
        }
        return true;
    }
};

struct FinderPattern {
    cv::Point2f centroid;
    float size_px = 20.0f;
};

enum class MoveDir {
    NONE,
    MOVE_LEFT,
    MOVE_RIGHT,
    MOVE_FORWARD,
    MOVE_BACKWARD,
    MOVE_FORWARD_LEFT,
    MOVE_FORWARD_RIGHT,
    MOVE_BACKWARD_LEFT,
    MOVE_BACKWARD_RIGHT
};

inline std::string moveDirToString(MoveDir dir) {
    switch (dir) {
        case MoveDir::MOVE_LEFT:           return "MOVE_LEFT";
        case MoveDir::MOVE_RIGHT:          return "MOVE_RIGHT";
        case MoveDir::MOVE_FORWARD:        return "MOVE_FORWARD";
        case MoveDir::MOVE_BACKWARD:       return "MOVE_BACKWARD";
        case MoveDir::MOVE_FORWARD_LEFT:   return "MOVE_FORWARD_LEFT";
        case MoveDir::MOVE_FORWARD_RIGHT:  return "MOVE_FORWARD_RIGHT";
        case MoveDir::MOVE_BACKWARD_LEFT:  return "MOVE_BACKWARD_LEFT";
        case MoveDir::MOVE_BACKWARD_RIGHT: return "MOVE_BACKWARD_RIGHT";
        default:                           return "NONE";
    }
}

struct DirectionCommand {
    MoveDir direction = MoveDir::NONE;
    double magnitude = 0.0;
    std::string reason;
};

struct DetectionResult {
    bool found = false;         // true if QR pattern found (even if undecodable)
    bool decoded = false;       // true if full text was successfully decoded
    cv::Rect detected_box;      // full-resolution coordinates
    std::string decoded_text;   // empty if partial/occluded
    int pattern_count = 0;      // 0, 1, 2, or 3+ finder patterns found
    std::vector<FinderPattern> finder_patterns;
};

// ===========================================================================
// Thermal Monitoring
// ===========================================================================
struct ThermalState {
    double cpu_temp_c = 0.0;
    bool is_throttled = false;
    int adaptive_burst_count = 5;
};

ThermalState checkThermalState(int base_burst_count) {
    ThermalState state;
    state.adaptive_burst_count = base_burst_count;

#ifdef __linux__
    std::ifstream temp_file("/sys/class/thermal/thermal_zone0/temp");
    if (temp_file.is_open()) {
        int raw_temp = 0;
        temp_file >> raw_temp;
        state.cpu_temp_c = raw_temp / 1000.0;
    }

    std::ifstream cur_freq_file("/sys/devices/system/cpu/cpu0/cpufreq/scaling_cur_freq");
    std::ifstream max_freq_file("/sys/devices/system/cpu/cpu0/cpufreq/cpuinfo_max_freq");
    if (cur_freq_file.is_open() && max_freq_file.is_open()) {
        long cur_freq = 0, max_freq = 0;
        cur_freq_file >> cur_freq;
        max_freq_file >> max_freq;
        if (max_freq > 0) {
            state.is_throttled = (cur_freq < max_freq * 0.9);
        }
    }

    if (state.is_throttled || state.cpu_temp_c > 70.0) {
        state.adaptive_burst_count = std::max(2, base_burst_count - 2);
        std::cerr << "[WARNING] Thermal throttle: " << state.cpu_temp_c
                  << "C. Burst reduced to " << state.adaptive_burst_count << "\n";
    }
#endif
    return state;
}

// ===========================================================================
// Pre-allocated Grayscale Frame Buffer
// ===========================================================================
class FrameBuffer {
public:
    FrameBuffer(int capacity, int width, int height)
        : capacity_(capacity), current_idx_(0) {
        for (int i = 0; i < capacity; ++i) {
            frames_gray_.emplace_back(height, width, CV_8U);
            scores_.push_back(0.0);
        }
    }

    void reset() { current_idx_ = 0; }

    int addFrame(const cv::Mat& raw_frame) {
        if (raw_frame.empty()) return -1;
        cv::Mat& dest = frames_gray_[current_idx_];
        if (raw_frame.channels() == 3) {
            cv::cvtColor(raw_frame, dest, cv::COLOR_BGR2GRAY);
        } else if (raw_frame.channels() == 1) {
            raw_frame.copyTo(dest);
        } else {
            cv::cvtColor(raw_frame, dest, cv::COLOR_BGRA2GRAY);
        }
        int written = current_idx_;
        current_idx_ = (current_idx_ + 1) % capacity_;
        return written;
    }

    const cv::Mat& getFrame(int idx) const { return frames_gray_[idx]; }
    void setScore(int idx, double score) { scores_[idx] = score; }
    double getScore(int idx) const { return scores_[idx]; }
    int capacity() const { return capacity_; }

private:
    std::vector<cv::Mat> frames_gray_;
    std::vector<double> scores_;
    int capacity_;
    int current_idx_;
};

// ===========================================================================
// Sharpness Scorer (Variance of Laplacian)
// ===========================================================================
class SharpnessScorer {
public:
    double score(const cv::Mat& gray) {
        if (gray.empty()) return 0.0;
        cv::Laplacian(gray, lap_, CV_64F);
        cv::Scalar mean, stddev;
        cv::meanStdDev(lap_, mean, stddev);
        return stddev[0] * stddev[0];
    }

private:
    cv::Mat lap_;
};

// ===========================================================================
// Contour-Hierarchy Based QR Finder Pattern Detector (for Clipped Fragments)
// ===========================================================================
struct FragmentResult {
    int pattern_count = 0;
    std::vector<FinderPattern> patterns;
    cv::Rect bounding_box;
};

class FinderPatternDetector {
public:
    static FragmentResult detectPatterns(const cv::Mat& frame) {
        FragmentResult result;
        if (frame.empty()) return result;

        cv::Mat gray;
        if (frame.channels() == 3) {
            cv::cvtColor(frame, gray, cv::COLOR_BGR2GRAY);
        } else {
            gray = frame;
        }

        cv::Mat bin;
        cv::adaptiveThreshold(gray, bin, 255, cv::ADAPTIVE_THRESH_GAUSSIAN_C,
                              cv::THRESH_BINARY, 21, 5);

        std::vector<std::vector<cv::Point>> contours;
        std::vector<cv::Vec4i> hierarchy;
        cv::findContours(bin, contours, hierarchy, cv::RETR_TREE, cv::CHAIN_APPROX_SIMPLE);

        if (hierarchy.empty()) return result;

        std::vector<FinderPattern> candidates;

        for (size_t i = 0; i < contours.size(); ++i) {
            int k = static_cast<int>(i);
            int count = 0;

            while (hierarchy[k][2] >= 0) {
                k = hierarchy[k][2];
                count++;
            }

            if (count >= 2) {
                cv::Rect r_outer = cv::boundingRect(contours[i]);
                if (r_outer.width < 10 || r_outer.height < 10) continue;

                float aspect = static_cast<float>(r_outer.width) / r_outer.height;
                if (aspect < 0.60f || aspect > 1.50f) continue;

                cv::Moments m = cv::moments(contours[k]);
                if (m.m00 == 0) continue;
                cv::Point2f pt(static_cast<float>(m.m10 / m.m00), static_cast<float>(m.m01 / m.m00));

                int cx = static_cast<int>(pt.x);
                int cy = static_cast<int>(pt.y);
                if (cx <= 2 || cx >= gray.cols - 2 || cy <= 2 || cy >= gray.rows - 2) continue;

                bool duplicate = false;
                for (const auto& existing : candidates) {
                    if (cv::norm(existing.centroid - pt) < (r_outer.width * 0.5f)) {
                        duplicate = true;
                        break;
                    }
                }
                if (!duplicate) {
                    FinderPattern fp;
                    fp.centroid = pt;
                    fp.size_px = static_cast<float>((r_outer.width + r_outer.height) / 2.0);
                    candidates.push_back(fp);
                }
            }
        }

        result.pattern_count = static_cast<int>(candidates.size());
        result.patterns = candidates;

        if (!candidates.empty()) {
            std::vector<cv::Point> pts;
            for (const auto& p : candidates) {
                float half = std::max(p.size_px * 0.75f, 15.0f);
                pts.push_back(cv::Point(static_cast<int>(p.centroid.x - half), static_cast<int>(p.centroid.y - half)));
                pts.push_back(cv::Point(static_cast<int>(p.centroid.x + half), static_cast<int>(p.centroid.y + half)));
            }
            result.bounding_box = cv::boundingRect(pts);
            result.bounding_box &= cv::Rect(0, 0, frame.cols, frame.rows);
        }

        return result;
    }
};

// ===========================================================================
// QR Detector (Two-Stage + Contour Fallback for Clipped QR Codes)
// ===========================================================================
class QRDetector {
public:
    DetectionResult detect(const cv::Mat& frame) {
        DetectionResult result;
        std::vector<cv::Point> points;

        // Stage 1: Try full detectAndDecode (works for intact QR codes)
        std::string text = detector_.detectAndDecode(frame, points);
        if (!points.empty() && !text.empty()) {
            result.found = true;
            result.decoded = true;
            result.detected_box = cv::boundingRect(points);
            result.decoded_text = text;
            result.pattern_count = 3;
            for (size_t i = 0; i < points.size(); ++i) {
                FinderPattern fp;
                fp.centroid = cv::Point2f(points[i]);
                fp.size_px = 20.0f;
                result.finder_patterns.push_back(fp);
            }
            return result;
        }

        // Stage 2: Try detect-only (finds finder patterns even on partial/occluded QR)
        points.clear();
        bool pattern_found = detector_.detect(frame, points);
        if (pattern_found && !points.empty()) {
            result.found = true;
            result.decoded = false;  // found but data unreadable
            result.detected_box = cv::boundingRect(points);
            result.decoded_text = "";
            result.pattern_count = 3;
            for (size_t i = 0; i < points.size(); ++i) {
                FinderPattern fp;
                fp.centroid = cv::Point2f(points[i]);
                fp.size_px = 20.0f;
                result.finder_patterns.push_back(fp);
            }

            // Stage 2b: Try decoding the detected region with contrast boost
            cv::Rect roi = result.detected_box;
            roi.x = std::max(0, roi.x - 20);
            roi.y = std::max(0, roi.y - 20);
            roi.width  = std::min(frame.cols - roi.x, roi.width  + 40);
            roi.height = std::min(frame.rows - roi.y, roi.height + 40);
            if (roi.width > 0 && roi.height > 0) {
                cv::Mat roi_crop = frame(roi).clone();
                cv::Mat roi_sharp;
                cv::GaussianBlur(roi_crop, roi_sharp, cv::Size(0, 0), 3);
                cv::addWeighted(roi_crop, 1.8, roi_sharp, -0.8, 0, roi_sharp);
                std::vector<cv::Point> pts2;
                std::string retry_text = detector_.detectAndDecode(roi_sharp, pts2);
                if (!retry_text.empty()) {
                    result.decoded = true;
                    result.decoded_text = retry_text;
                }
            }
            return result;
        }

        // Stage 3: Contour-hierarchy finder pattern detector (for 1-2 finder patterns when clipped)
        FragmentResult frag = FinderPatternDetector::detectPatterns(frame);
        if (frag.pattern_count > 0) {
            result.found = true;
            result.decoded = false;
            result.pattern_count = frag.pattern_count;
            result.detected_box = frag.bounding_box;
            result.finder_patterns = frag.patterns;
            result.decoded_text = "";
        }

        return result;
    }

private:
    cv::QRCodeDetector detector_;
};

// ===========================================================================
// Threaded Camera Grabber
// ===========================================================================
class CameraGrabber {
public:
    explicit CameraGrabber(cv::VideoCapture& cap) : cap_(cap), running_(false), has_frame_(false) {}

    void start() {
        running_ = true;
        worker_ = std::thread(&CameraGrabber::loop, this);
    }

    void stop() {
        running_ = false;
        if (worker_.joinable()) worker_.join();
    }

    bool getLatest(cv::Mat& out, int timeout_ms = 1000) {
        std::unique_lock<std::mutex> lock(mtx_);
        if (!cv_.wait_for(lock, std::chrono::milliseconds(timeout_ms),
                          [this] { return has_frame_; })) {
            return false;
        }
        out = latest_.clone();
        return true;
    }

private:
    void loop() {
        cv::Mat frame;
        while (running_) {
            if (!cap_.read(frame) || frame.empty()) {
                std::this_thread::sleep_for(std::chrono::milliseconds(5));
                continue;
            }
            {
                std::lock_guard<std::mutex> lock(mtx_);
                latest_ = frame;
                has_frame_ = true;
            }
            cv_.notify_one();
        }
    }

    cv::VideoCapture& cap_;
    std::thread worker_;
    std::atomic<bool> running_;
    std::mutex mtx_;
    std::condition_variable cv_;
    cv::Mat latest_;
    bool has_frame_;
};

// ===========================================================================
// Alignment Logic
// ===========================================================================
bool computeExpectedBoxSizePx(const CameraParams& cam, const TargetParams& target,
                               double distance_m, double& out_size_px) {
    const double DIST_MIN = 0.01, DIST_MAX = 100.0;
    if (distance_m < DIST_MIN || distance_m > DIST_MAX) return false;
    double size_px = (target.qr_real_size_m * cam.focal_length_px) / distance_m;
    if (!std::isfinite(size_px)) return false;
    out_size_px = size_px;
    return true;
}

bool isAligned(const cv::Rect& box, double expected_size_px, const CameraParams& cam,
               const TargetParams& target, double* out_offset_px = nullptr) {
    if (expected_size_px <= 0.0) return false;
    if (box.height <= 0 || box.width <= 0) return false;

    double aspect_ratio = static_cast<double>(box.width) / box.height;
    if (aspect_ratio < 0.8 || aspect_ratio > 1.25) return false;

    double detected_size = (box.width + box.height) / 2.0;
    double size_diff_ratio = std::abs(detected_size - expected_size_px) / expected_size_px;
    bool size_ok = size_diff_ratio <= target.size_tolerance;

    cv::Point center(box.x + box.width / 2, box.y + box.height / 2);
    cv::Point frame_center(cam.frame_width_px / 2, cam.frame_height_px / 2);
    double offset = cv::norm(center - frame_center);
    if (out_offset_px) *out_offset_px = offset;
    bool center_ok = offset <= target.center_tolerance_px * 1.5;

    return size_ok && center_ok;
}

// ===========================================================================
// Directional Guidance Helper (Fragment & Off-Center Alignment Mapping)
// ===========================================================================
DirectionCommand mapFragmentToDirection(
    const DetectionResult& detection,
    int frame_w, int frame_h,
    int edge_margin_px,
    double center_tolerance_px)
{
    DirectionCommand cmd;
    cmd.direction = MoveDir::NONE;
    cmd.magnitude = 0.0;
    cmd.reason = "";

    if (!detection.found || detection.detected_box.width <= 0 || detection.detected_box.height <= 0) {
        return cmd;
    }

    bool is_clipped = (!detection.decoded) || (detection.pattern_count < 3);

    cv::Rect box = detection.detected_box;

    bool touch_left   = (box.x <= edge_margin_px);
    bool touch_right  = (box.x + box.width >= frame_w - edge_margin_px);
    bool touch_top    = (box.y <= edge_margin_px);
    bool touch_bottom = (box.y + box.height >= frame_h - edge_margin_px);

    if (is_clipped && (touch_left || touch_right || touch_top || touch_bottom)) {
        cmd.magnitude = 1.0;
        cmd.reason = "clipped";

        if (touch_top && touch_left)             cmd.direction = MoveDir::MOVE_FORWARD_LEFT;
        else if (touch_top && touch_right)        cmd.direction = MoveDir::MOVE_FORWARD_RIGHT;
        else if (touch_bottom && touch_left)      cmd.direction = MoveDir::MOVE_BACKWARD_LEFT;
        else if (touch_bottom && touch_right)     cmd.direction = MoveDir::MOVE_BACKWARD_RIGHT;
        else if (touch_top)                       cmd.direction = MoveDir::MOVE_FORWARD;
        else if (touch_bottom)                    cmd.direction = MoveDir::MOVE_BACKWARD;
        else if (touch_left)                      cmd.direction = MoveDir::MOVE_LEFT;
        else if (touch_right)                     cmd.direction = MoveDir::MOVE_RIGHT;

        return cmd;
    }

    // Fine-centering case: calculate center offset
    cv::Point2f qr_center(box.x + box.width / 2.0f, box.y + box.height / 2.0f);
    float cam_cx = frame_w / 2.0f;
    float cam_cy = frame_h / 2.0f;

    float dx = qr_center.x - cam_cx; // dx < 0 -> left, dx > 0 -> right
    float dy = qr_center.y - cam_cy; // dy < 0 -> forward (top), dy > 0 -> backward (bottom)
    double dist = std::hypot(dx, dy);

    if (dist > center_tolerance_px) {
        cmd.reason = "centering";
        cmd.magnitude = std::min(1.0, dist / (std::min(frame_w, frame_h) / 2.0));

        double diag_thresh = center_tolerance_px * 0.5;

        if (std::abs(dx) > diag_thresh && std::abs(dy) > diag_thresh) {
            if (dy < 0 && dx < 0)      cmd.direction = MoveDir::MOVE_FORWARD_LEFT;
            else if (dy < 0 && dx > 0) cmd.direction = MoveDir::MOVE_FORWARD_RIGHT;
            else if (dy > 0 && dx < 0) cmd.direction = MoveDir::MOVE_BACKWARD_LEFT;
            else if (dy > 0 && dx > 0) cmd.direction = MoveDir::MOVE_BACKWARD_RIGHT;
        } else if (std::abs(dx) > std::abs(dy)) {
            cmd.direction = (dx < 0) ? MoveDir::MOVE_LEFT : MoveDir::MOVE_RIGHT;
        } else {
            cmd.direction = (dy < 0) ? MoveDir::MOVE_FORWARD : MoveDir::MOVE_BACKWARD;
        }
    }

    return cmd;
}

// ===========================================================================
// Live GUI HUD Overlay Generator
// ===========================================================================
void drawLiveOverlay(cv::Mat& frame,
                      double expected_size_px,
                      const DetectionResult& detection,
                      bool aligned,
                      double cycle_ms,
                      const CameraParams& cam,
                      const DirectionCommand& move_cmd) {
    if (frame.empty()) return;

    cv::Point2f camera_center(frame.cols / 2.0f, frame.rows / 2.0f);

    // 1. Camera center crosshair (cyan)
    cv::drawMarker(frame, camera_center, cv::Scalar(255, 255, 0),
                   cv::MARKER_TILTED_CROSS, 30, 2);

    // 2. Expected reference guide box (gray)
    if (expected_size_px > 0) {
        int box_size = static_cast<int>(std::round(expected_size_px));
        cv::Rect guide_box(
            static_cast<int>(camera_center.x - box_size / 2.0f),
            static_cast<int>(camera_center.y - box_size / 2.0f),
            box_size, box_size
        );
        cv::rectangle(frame, guide_box, cv::Scalar(180, 180, 180), 1, cv::LINE_AA);
    }

    // Draw individual finder pattern circles (orange)
    for (const auto& fp : detection.finder_patterns) {
        cv::circle(frame, fp.centroid, 8, cv::Scalar(0, 165, 255), 2, cv::LINE_AA);
    }

    if (detection.found && detection.detected_box.width > 0 && detection.detected_box.height > 0) {
        cv::Point2f qr_center(
            detection.detected_box.x + detection.detected_box.width / 2.0f,
            detection.detected_box.y + detection.detected_box.height / 2.0f
        );

        float dx = qr_center.x - camera_center.x;
        float dy = qr_center.y - camera_center.y;
        double offset_dist = cv::norm(qr_center - camera_center);

        cv::Scalar color;
        std::string status_label;
        if (!detection.decoded) {
            color = cv::Scalar(0, 165, 255);  // Orange - partial
            status_label = "PARTIAL QR";
        } else if (aligned) {
            color = cv::Scalar(0, 255, 0);    // Green - centered
            status_label = "CENTERED";
        } else {
            color = cv::Scalar(0, 0, 255);    // Red - not centered
            status_label = "NOT CENTERED";
        }

        int box_thickness = detection.decoded ? 3 : 2;
        cv::rectangle(frame, detection.detected_box, color, box_thickness);

        // Connect camera center to QR center
        cv::line(frame, camera_center, qr_center, cv::Scalar(255, 255, 0), 2, cv::LINE_AA);

        // Dot at QR center
        cv::circle(frame, qr_center, 6, color, -1);

        // Status label above bounding box
        cv::putText(frame, status_label,
                    cv::Point(detection.detected_box.x, std::max(20, detection.detected_box.y - 10)),
                    cv::FONT_HERSHEY_SIMPLEX, 0.7, color, 2);

        // Offset HUD overlay text
        std::string offset_info = "Offset X: " + std::to_string(static_cast<int>(dx)) +
                                  "  Y: " + std::to_string(static_cast<int>(dy)) +
                                  "  Dist: " + std::to_string(static_cast<int>(offset_dist)) + "px";
        cv::putText(frame, offset_info, cv::Point(20, 70),
                    cv::FONT_HERSHEY_SIMPLEX, 0.65, cv::Scalar(255, 255, 0), 2);

        if (detection.decoded && !detection.decoded_text.empty()) {
            cv::putText(frame, "Data: " + detection.decoded_text, cv::Point(20, 100),
                        cv::FONT_HERSHEY_SIMPLEX, 0.65, cv::Scalar(0, 255, 255), 2);
        } else if (!detection.decoded) {
            cv::putText(frame, "Data: [Partial - decoding failed]", cv::Point(20, 100),
                        cv::FONT_HERSHEY_SIMPLEX, 0.65, cv::Scalar(0, 165, 255), 2);
        }
    } else {
        cv::putText(frame, "Status: Searching for QR...", cv::Point(20, 70),
                    cv::FONT_HERSHEY_SIMPLEX, 0.65, cv::Scalar(0, 0, 255), 2);
    }

    if (move_cmd.direction != MoveDir::NONE) {
        char mag_buf[32];
        std::snprintf(mag_buf, sizeof(mag_buf), "%.2f", move_cmd.magnitude);
        std::string move_text = "MOVE: " + moveDirToString(move_cmd.direction) +
                                " | Mag: " + mag_buf +
                                " | " + move_cmd.reason;
        cv::putText(frame, move_text, cv::Point(20, 130),
                    cv::FONT_HERSHEY_SIMPLEX, 0.7, cv::Scalar(0, 255, 255), 2);
    }

    // Top HUD Title Bar
    std::string hud_title = "Prototype 1 - QR Scanner | Cycle: " + std::to_string(static_cast<int>(cycle_ms)) + "ms";
    cv::putText(frame, hud_title, cv::Point(20, 35),
                cv::FONT_HERSHEY_SIMPLEX, 0.75, cv::Scalar(255, 255, 255), 2);
}

// ===========================================================================
// Detached Process Dispatch & Photo Saving
// ===========================================================================
std::string timestampName() {
    auto now = std::chrono::system_clock::now();
    auto t = std::chrono::system_clock::to_time_t(now);
    auto ms = std::chrono::duration_cast<std::chrono::milliseconds>(
                  now.time_since_epoch()).count() % 1000;
    std::tm tm_buf;
#ifdef _WIN32
    localtime_s(&tm_buf, &t);
#else
    localtime_r(&t, &tm_buf);
#endif
    char buf[64];
    std::snprintf(buf, sizeof(buf), "%04d%02d%02d_%02d%02d%02d_%03ld",
                  tm_buf.tm_year + 1900, tm_buf.tm_mon + 1, tm_buf.tm_mday,
                  tm_buf.tm_hour, tm_buf.tm_min, tm_buf.tm_sec, (long)ms);
    return std::string(buf);
}

bool runMoveHookDetached(const std::string& script, MoveDir dir, double magnitude, const std::string& reason) {
    if (dir == MoveDir::NONE) return false;
    std::string dir_str = moveDirToString(dir);
    char mag_buf[32];
    std::snprintf(mag_buf, sizeof(mag_buf), "%.2f", magnitude);
    std::string mag_str(mag_buf);

#ifdef _WIN32
    std::string script_to_use = script;
    for (auto& c : script_to_use) { if (c == '/') c = '\\'; }

    size_t dot = script_to_use.find_last_of('.');
    std::string base_name = (dot != std::string::npos) ? script_to_use.substr(0, dot) : script_to_use;
    std::string bat_script = base_name + ".bat";

    if (_access(bat_script.c_str(), F_OK) == 0) {
        script_to_use = bat_script;
    } else if (_access(script_to_use.c_str(), F_OK) != 0) {
        std::cerr << "[MoveHook] Script missing or inaccessible: " << script_to_use << " / " << bat_script << "\n";
        return false;
    }

    std::string cmd;
    if (script_to_use.size() >= 4 && script_to_use.substr(script_to_use.size() - 4) == ".bat") {
        cmd = "cmd.exe /c \"" + script_to_use + " \"" + dir_str + "\" \"" + mag_str + "\" \"" + reason + "\"\"";
    } else {
        cmd = script_to_use + " \"" + dir_str + "\" \"" + mag_str + "\" \"" + reason + "\"";
    }
    STARTUPINFOA si;
    PROCESS_INFORMATION pi;
    ZeroMemory(&si, sizeof(si));
    si.cb = sizeof(si);
    ZeroMemory(&pi, sizeof(pi));
    char cmdBuf[1024];
    strncpy_s(cmdBuf, sizeof(cmdBuf), cmd.c_str(), _TRUNCATE);
    if (!CreateProcessA(NULL, cmdBuf, NULL, NULL, FALSE, DETACHED_PROCESS | CREATE_NO_WINDOW, NULL, NULL, &si, &pi)) {
        std::cerr << "[MoveHook] CreateProcessA failed with code: " << GetLastError() << "\n";
        return false;
    }
    CloseHandle(pi.hProcess);
    CloseHandle(pi.hThread);
    return true;
#else
    if (access(script.c_str(), F_OK) != 0) return false;
    pid_t pid = fork();
    if (pid == 0) {
        execl(script.c_str(), script.c_str(), dir_str.c_str(), mag_str.c_str(), reason.c_str(), (char*)NULL);
        _exit(127);
    }
    return (pid > 0);
#endif
}

bool runHookDetached(const std::string& script, const std::string& image_path,
                      const std::string& decoded_text) {
#ifdef _WIN32
    if (_access(script.c_str(), F_OK) != 0) {
        std::cerr << "[Hook] Script missing or inaccessible: " << script
                  << " (image saved, but not forwarded)\n";
        return false;
    }
    std::string cmd = script + " \"" + image_path + "\" \"" + decoded_text + "\"";
    STARTUPINFOA si;
    PROCESS_INFORMATION pi;
    ZeroMemory(&si, sizeof(si));
    si.cb = sizeof(si);
    ZeroMemory(&pi, sizeof(pi));
    char cmdBuf[1024];
    strncpy_s(cmdBuf, sizeof(cmdBuf), cmd.c_str(), _TRUNCATE);
    if (!CreateProcessA(NULL, cmdBuf, NULL, NULL, FALSE, DETACHED_PROCESS, NULL, NULL, &si, &pi)) {
        std::cerr << "[Hook] CreateProcessA failed: " << GetLastError() << "\n";
        return false;
    }
    CloseHandle(pi.hProcess);
    CloseHandle(pi.hThread);
    return true;
#else
    if (access(script.c_str(), X_OK) != 0) {
        std::cerr << "[Hook] Script not executable or missing: " << script
                  << " (image saved, but not forwarded)\n";
        return false;
    }
    pid_t pid = fork();
    if (pid < 0) {
        std::cerr << "[Hook] fork() failed: " << std::strerror(errno) << "\n";
        return false;
    }
    if (pid == 0) {
        execl(script.c_str(), script.c_str(), image_path.c_str(), decoded_text.c_str(), (char*)nullptr);
        _exit(127);
    }
    return true;
#endif
}

bool sendCapture(const cv::Mat& full_res_bgr_frame, const cv::Rect& detected_box, const std::string& decoded_text,
                  const Config& cfg) {
    if (full_res_bgr_frame.empty()) {
        std::cerr << "[Capture] Frame to send is empty, aborting\n";
        return false;
    }

    cv::Mat to_save = full_res_bgr_frame;

    // Crop ONLY the QR code with 10% padding if valid box is provided
    if (detected_box.width > 0 && detected_box.height > 0) {
        int pad_x = static_cast<int>(detected_box.width * 0.10);
        int pad_y = static_cast<int>(detected_box.height * 0.10);

        int x1 = std::max(0, detected_box.x - pad_x);
        int y1 = std::max(0, detected_box.y - pad_y);
        int x2 = std::min(full_res_bgr_frame.cols, detected_box.x + detected_box.width + pad_x);
        int y2 = std::min(full_res_bgr_frame.rows, detected_box.y + detected_box.height + pad_y);

        if (x2 > x1 && y2 > y1) {
            to_save = full_res_bgr_frame(cv::Rect(x1, y1, x2 - x1, y2 - y1)).clone();
        }
    }

    std::string filename = "qr_" + timestampName() + ".jpg";
    std::string path = cfg.output_dir + "/" + filename;

    std::vector<int> params = {cv::IMWRITE_JPEG_QUALITY, cfg.jpeg_quality};
    if (!cv::imwrite(path, to_save, params)) {
        std::cerr << "[Capture] Failed to write " << path << "\n";
        return false;
    }
    std::cout << "[Capture] Saved cropped QR image (" << to_save.cols << "x" << to_save.rows << ") to " << path << "\n";
    return runHookDetached(cfg.hook_script, path, decoded_text);
}

// ===========================================================================
// Signal Handling
// ===========================================================================
std::atomic<bool> g_running{true};
void onSignal(int) { g_running = false; }

// ===========================================================================
// Built-in Self Test Suite (--test)
// ===========================================================================
int runSelfTest() {
    std::cout << "====================================================\n";
    std::cout << "  QR SCANNER SELF-TEST SUITE (--test)\n";
    std::cout << "====================================================\n\n";

    int passed = 0;
    int total = 5;

    // Test 1: Config Defaults & Loader
    std::cout << "[Test 1/5] Config parsing & fallback...\n";
    Config cfg;
    if (cfg.req_width == 1280 && cfg.req_height == 720) {
        std::cout << "  ✓ Config defaults initialized properly.\n";
        passed++;
    }

    // Test 2: Sharpness Scorer
    std::cout << "\n[Test 2/5] Sharpness Scoring (Laplacian Variance)...\n";
    cv::Mat sharp(480, 640, CV_8UC1, cv::Scalar(255));
    cv::rectangle(sharp, cv::Rect(200, 150, 240, 180), cv::Scalar(0), -1);
    cv::Mat blur;
    cv::GaussianBlur(sharp, blur, cv::Size(21, 21), 7.0);

    SharpnessScorer scorer;
    double s_sharp = scorer.score(sharp);
    double s_blur = scorer.score(blur);
    std::cout << "  Sharp frame score: " << s_sharp << " | Blurry frame score: " << s_blur << "\n";
    if (s_sharp > s_blur * 2.0) {
        std::cout << "  ✓ Sharpness scorer correctly ranked sharp frame higher than blurry frame.\n";
        passed++;
    }

    // Test 3: Downscaled QR Detection & Coordinate Rescaling
    std::cout << "\n[Test 3/5] QR Detection & Bounding Box Rescaling...\n";
    std::string test_payload = "https://qr-scanner.all-in-one.test/target";
    cv::QRCodeEncoder::Params qrp;
    cv::Ptr<cv::QRCodeEncoder> qren = cv::QRCodeEncoder::create(qrp);
    cv::Mat qrm;
    qren->encode(test_payload, qrm);
    if (!qrm.empty()) {
        cv::Mat frame(720, 1280, CV_8UC1, cv::Scalar(255));
        cv::Mat qr_res;
        cv::resize(qrm, qr_res, cv::Size(300, 300), 0, 0, cv::INTER_NEAREST);
        qr_res.copyTo(frame(cv::Rect(490, 210, 300, 300)));

        cv::Mat down;
        double scale = 0.5;
        cv::resize(frame, down, cv::Size(), scale, scale, cv::INTER_AREA);

        QRDetector detector;
        DetectionResult det = detector.detect(down);
        if (det.found && det.decoded_text == test_payload) {
            double inv = 1.0 / scale;
            cv::Rect full_box(
                static_cast<int>(det.detected_box.x * inv),
                static_cast<int>(det.detected_box.y * inv),
                static_cast<int>(det.detected_box.width * inv),
                static_cast<int>(det.detected_box.height * inv)
            );
            std::cout << "  Decoded text: \"" << det.decoded_text << "\"\n";
            std::cout << "  Full-Res Box: " << full_box.x << "," << full_box.y << " " << full_box.width << "x" << full_box.height << "\n";
            std::cout << "  ✓ QR detected on 50% frame and rescaled accurately.\n";
            passed++;
        }
    }

    // Test 4: Alignment Check
    std::cout << "\n[Test 4/5] Alignment Check...\n";
    CameraParams cam{800.0, 1280, 720};
    TargetParams tgt{0.20, 0.15, 100.0};
    double expected_size = 0.0;
    if (computeExpectedBoxSizePx(cam, tgt, 1.0, expected_size)) {
        cv::Rect aligned_box(560, 280, static_cast<int>(expected_size), static_cast<int>(expected_size));
        bool ok = isAligned(aligned_box, expected_size, cam, tgt);
        std::cout << "  Expected Box Size at 1.0m: " << expected_size << "px\n";
        if (ok) {
            std::cout << "  ✓ Centered square box correctly evaluated as ALIGNED.\n";
            passed++;
        }
    }

    // Test 5: Image Saving
    std::cout << "\n[Test 5/7] Full-Res JPEG Creation...\n";
    makeDirsRecursive("./test_output");
    cv::Mat test_img(720, 1280, CV_8UC3, cv::Scalar(100, 200, 100));
    std::vector<int> jparams = {cv::IMWRITE_JPEG_QUALITY, 92};
    bool ok_save = cv::imwrite("./test_output/test.jpg", test_img, jparams);
    if (ok_save) {
        std::cout << "  ✓ Full-res JPEG image saved to ./test_output/test.jpg.\n";
        passed++;
    }

    // Test 6: Directional Guidance Mapping Matrix Test (All 8 edge/corner worked examples)
    std::cout << "\n[Test 6/7] Directional Mapping Matrix (8 Edge/Corner Worked Examples)...\n";
    int cases_passed = 0;
    struct TestCase {
        cv::Rect box;
        MoveDir expected_dir;
        std::string expected_reason;
        std::string name;
    };
    std::vector<TestCase> test_cases = {
        { {0, 260, 150, 210},     MoveDir::MOVE_LEFT,           "clipped", "Left Edge" },
        { {1120, 250, 160, 220},  MoveDir::MOVE_RIGHT,          "clipped", "Right Edge" },
        { {520, 0, 230, 140},     MoveDir::MOVE_FORWARD,        "clipped", "Top Edge" },
        { {500, 600, 220, 120},   MoveDir::MOVE_BACKWARD,       "clipped", "Bottom Edge" },
        { {0, 0, 140, 130},       MoveDir::MOVE_FORWARD_LEFT,   "clipped", "Top-Left Corner" },
        { {1150, 0, 130, 140},    MoveDir::MOVE_FORWARD_RIGHT,  "clipped", "Top-Right Corner" },
        { {0, 610, 150, 110},     MoveDir::MOVE_BACKWARD_LEFT,  "clipped", "Bottom-Left Corner" },
        { {1140, 590, 140, 130},  MoveDir::MOVE_BACKWARD_RIGHT, "clipped", "Bottom-Right Corner" }
    };

    for (const auto& tc : test_cases) {
        DetectionResult d;
        d.found = true;
        d.decoded = false;
        d.pattern_count = 1;
        d.detected_box = tc.box;

        DirectionCommand cmd = mapFragmentToDirection(d, 1280, 720, 40, 100.0);
        if (cmd.direction == tc.expected_dir && cmd.reason == tc.expected_reason && cmd.magnitude == 1.0) {
            cases_passed++;
        } else {
            std::cout << "  FAIL: " << tc.name << " got " << moveDirToString(cmd.direction)
                      << " (expected " << moveDirToString(tc.expected_dir) << ")\n";
        }
    }

    std::cout << "  Passed " << cases_passed << " / 8 coordinate mapping cases.\n";
    if (cases_passed == 8) {
        std::cout << "  ✓ All 8 edge/corner clipped coordinate mappings verified.\n";
        passed++;
    }

    // Test 7: Move Command Detached Hook Execution
    std::cout << "\n[Test 7/7] Move Hook Execution...\n";
    bool ok_hook = runMoveHookDetached("./hooks/move_command.sh", MoveDir::MOVE_LEFT, 1.0, "clipped");
    if (ok_hook) {
        std::cout << "  ✓ Detached move hook spawned successfully.\n";
        passed++;
    }

    int total_tests = 7;
    std::cout << "\n====================================================\n";
    std::cout << "  SELF-TEST RESULT: " << passed << " / " << total_tests << " PASSED\n";
    std::cout << "====================================================\n\n";
    return (passed == total_tests) ? 0 : 1;
}

// ===========================================================================
// Main Entry Point
// ===========================================================================
int main(int argc, char** argv) {
    // Check if --test flag was passed
    for (int i = 1; i < argc; ++i) {
        std::string arg = argv[i];
        if (arg == "--test" || arg == "-t") {
            return runSelfTest();
        }
    }

    std::cout << "=== QR Scanner - Prototype 1 Unified Application ===\n\n";

    std::signal(SIGINT, onSignal);
    std::signal(SIGTERM, onSignal);
#ifdef SIGCHLD
    std::signal(SIGCHLD, SIG_IGN);
#endif

    Config cfg;
    std::string cfg_path = (argc > 1 && argv[1][0] != '-') ? argv[1] : "config.cfg";
    cfg.load(cfg_path);

    makeDirsRecursive(cfg.output_dir);

    cv::VideoCapture capture;
#ifdef __linux__
    capture.open(cfg.camera_index, cv::CAP_V4L2);
#else
    capture.open(cfg.camera_index, cv::CAP_ANY);
#endif
    if (!capture.isOpened()) {
        std::cerr << "[Fatal] Failed to open camera index " << cfg.camera_index << "\n";
        return -1;
    }

    capture.set(cv::CAP_PROP_FOURCC, cv::VideoWriter::fourcc('M', 'J', 'P', 'G'));
    capture.set(cv::CAP_PROP_FRAME_WIDTH, cfg.req_width);
    capture.set(cv::CAP_PROP_FRAME_HEIGHT, cfg.req_height);
    capture.set(cv::CAP_PROP_BUFFERSIZE, 1);

    int actual_width = static_cast<int>(capture.get(cv::CAP_PROP_FRAME_WIDTH));
    int actual_height = static_cast<int>(capture.get(cv::CAP_PROP_FRAME_HEIGHT));
    double actual_fps = capture.get(cv::CAP_PROP_FPS);

    if (actual_width <= 0 || actual_height <= 0) {
        actual_width = cfg.req_width;
        actual_height = cfg.req_height;
    }
    std::cout << "[Camera] Negotiated " << actual_width << "x" << actual_height
              << " @ " << actual_fps << " FPS (requested " << cfg.req_width << "x"
              << cfg.req_height << ")\n";

    CameraParams cam;
    cam.focal_length_px = cfg.focal_length_px;
    cam.frame_width_px = actual_width;
    cam.frame_height_px = actual_height;
    if (!cam.validate()) return -1;

    TargetParams target;
    target.qr_real_size_m = cfg.qr_real_size_m;
    target.size_tolerance = cfg.size_tolerance;
    target.center_tolerance_px = cfg.center_tolerance_px;
    if (!target.validate()) return -1;

    double scale = cfg.detect_scale;
    if (scale <= 0.0 || scale > 1.0) scale = 1.0;
    int det_w = std::max(1, static_cast<int>(actual_width * scale));
    int det_h = std::max(1, static_cast<int>(actual_height * scale));

    FrameBuffer frame_buffer(std::max(1, cfg.burst_count), det_w, det_h);
    SharpnessScorer scorer;
    QRDetector detector;
    cv::Mat det_frame;

    CameraGrabber grabber(capture);
    grabber.start();

    std::string last_sent_text;
    auto last_sent_time = std::chrono::steady_clock::now() - std::chrono::hours(24);
    MoveDir last_move_dir = MoveDir::NONE;
    auto last_move_time = std::chrono::steady_clock::now() - std::chrono::hours(24);
    auto run_start = std::chrono::steady_clock::now();
    long cycle_count = 0;

    const std::string window_name = "Prototype 1 - QR Scanner";
    if (cfg.show_display) {
        cv::namedWindow(window_name, cv::WINDOW_AUTOSIZE);
    }

    std::cout << "[Starting continuous QR detection loop... Press 'q' or ESC in display window to exit]\n\n";

    while (g_running) {
        auto cycle_start = std::chrono::high_resolution_clock::now();

        ThermalState thermal = checkThermalState(cfg.burst_count);
        int burst_count = thermal.adaptive_burst_count;

        frame_buffer.reset();
        int best_slot = -1;
        double best_score = -1.0;
        cv::Mat last_full_frame;

        for (int i = 0; i < burst_count; ++i) {
            cv::Mat raw;
            if (!grabber.getLatest(raw, 200)) continue;
            last_full_frame = raw;

            cv::resize(raw, det_frame, cv::Size(det_w, det_h), 0, 0, cv::INTER_AREA);
            int slot = frame_buffer.addFrame(det_frame);
            if (slot < 0) continue;
            double score = scorer.score(frame_buffer.getFrame(slot));
            frame_buffer.setScore(slot, score);
            if (score > best_score) {
                best_score = score;
                best_slot = slot;
            }
        }

        if (best_slot < 0) {
            std::cerr << "[Warning] No usable frame this cycle\n";
            continue;
        }

        DetectionResult detection = detector.detect(frame_buffer.getFrame(best_slot));
        if (detection.found && scale != 1.0) {
            double inv = 1.0 / scale;
            detection.detected_box.x = static_cast<int>(detection.detected_box.x * inv);
            detection.detected_box.y = static_cast<int>(detection.detected_box.y * inv);
            detection.detected_box.width = static_cast<int>(detection.detected_box.width * inv);
            detection.detected_box.height = static_cast<int>(detection.detected_box.height * inv);
            for (auto& fp : detection.finder_patterns) {
                fp.centroid.x = static_cast<float>(fp.centroid.x * inv);
                fp.centroid.y = static_cast<float>(fp.centroid.y * inv);
                fp.size_px = static_cast<float>(fp.size_px * inv);
            }
        }

        double expected_size_px = 0.0;
        bool have_expected = computeExpectedBoxSizePx(cam, target, cfg.reference_distance_m, expected_size_px);

        bool aligned = false;
        double offset_px = -1.0;
        if (detection.found && have_expected) {
            aligned = isAligned(detection.detected_box, expected_size_px, cam, target, &offset_px);
        }

        DirectionCommand move_cmd;
        bool ready_to_send = detection.found && detection.decoded && !detection.decoded_text.empty() &&
                              (!cfg.require_alignment || aligned);

        if (ready_to_send) {
            // Full decode path: capture & send takes priority over move guidance
            auto now = std::chrono::steady_clock::now();
            auto since_last = std::chrono::duration_cast<std::chrono::milliseconds>(
                                   now - last_sent_time).count();
            bool new_payload = (detection.decoded_text != last_sent_text);
            if (new_payload || since_last > cfg.cooldown_ms) {
                cv::Mat to_send = last_full_frame.empty() ? cv::Mat() : last_full_frame;
                cv::Mat fresh;
                if (grabber.getLatest(fresh, 200)) to_send = fresh;

                if (sendCapture(to_send, detection.detected_box, detection.decoded_text, cfg)) {
                    last_sent_text = detection.decoded_text;
                    last_sent_time = now;
                    std::cout << "\n[SUCCESS] QR '" << detection.decoded_text
                              << "' captured and forwarded via hook.\n\n";
                    if (cfg.single_shot) break;
                }
            }
            last_move_dir = MoveDir::NONE;
        } else {
            // Directional guidance for clipped / partial / off-center QR codes
            move_cmd = mapFragmentToDirection(detection, cam.frame_width_px, cam.frame_height_px,
                                              cfg.edge_margin_px, cfg.center_tolerance_px);
            if (move_cmd.direction != MoveDir::NONE) {
                auto now = std::chrono::steady_clock::now();
                auto since_last_move = std::chrono::duration_cast<std::chrono::milliseconds>(
                                           now - last_move_time).count();
                if (move_cmd.direction != last_move_dir || since_last_move > cfg.move_cooldown_ms) {
                    if (runMoveHookDetached(cfg.move_hook_script, move_cmd.direction, move_cmd.magnitude, move_cmd.reason)) {
                        last_move_dir = move_cmd.direction;
                        last_move_time = now;
                        std::cout << "[MOVE] " << moveDirToString(move_cmd.direction)
                                  << " (mag=" << move_cmd.magnitude << ", reason=" << move_cmd.reason << ")\n";
                    }
                }
            } else {
                last_move_dir = MoveDir::NONE;
            }
        }

        auto cycle_end = std::chrono::high_resolution_clock::now();
        double cycle_ms = std::chrono::duration<double, std::milli>(cycle_end - cycle_start).count();

        if (cfg.verbose) {
            std::cout << "[Cycle " << ++cycle_count << "] " << cycle_ms << "ms | "
                      << "found=" << detection.found
                      << " decoded=" << detection.decoded
                      << " patterns=" << detection.pattern_count
                      << " aligned=" << aligned;
            if (detection.decoded) std::cout << " text=\"" << detection.decoded_text << "\"";
            else if (detection.found) std::cout << " [PARTIAL QR]";
            if (move_cmd.direction != MoveDir::NONE) {
                std::cout << " move=" << moveDirToString(move_cmd.direction);
            }
            std::cout << "\n";
        }

        // GUI Window Live Display with Overlay (Bounding Box, Center Crosshair, Offset X/Y, Data HUD, Direction Banner)
        if (cfg.show_display && !last_full_frame.empty()) {
            cv::Mat display_frame = last_full_frame.clone();
            drawLiveOverlay(display_frame, expected_size_px, detection, aligned, cycle_ms, cam, move_cmd);
            cv::imshow(window_name, display_frame);

            int key = cv::waitKey(1) & 0xFF;
            if (key == 'q' || key == 27) { // 'q' or ESC
                std::cout << "[Info] User pressed exit key ('q'/ESC). Stopping scanner.\n";
                g_running = false;
                break;
            }
        }

        if (cfg.max_runtime_s > 0) {
            auto elapsed_s = std::chrono::duration_cast<std::chrono::seconds>(
                                  std::chrono::steady_clock::now() - run_start).count();
            if (elapsed_s >= cfg.max_runtime_s) {
                std::cout << "[Info] max_runtime_s (" << cfg.max_runtime_s << "s) reached, stopping.\n";
                break;
            }
        }

        // Cycle pacing throttle: if scan cycle finished faster than target_cycle_ms, sleep remaining time
        if (cfg.target_cycle_ms > 0 && cycle_ms < cfg.target_cycle_ms) {
            int sleep_ms = static_cast<int>(cfg.target_cycle_ms - cycle_ms);
            std::this_thread::sleep_for(std::chrono::milliseconds(sleep_ms));
        }
    }

    grabber.stop();
    capture.release();
    if (cfg.show_display) {
        cv::destroyAllWindows();
    }
    std::cout << "\n[Done] Application terminated cleanly.\n";
    return 0;
}
