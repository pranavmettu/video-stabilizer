// main.cpp: command-line entry point.
//
//   ./stabilize <video> -o <out.mp4> [--smoother avg] [--radius 30]
//               [--side-by-side] [--trajectory-csv <file.csv>]   stabilize
//   ./stabilize <video> --motion-csv <file.csv>                  per-frame motion
//   ./stabilize <video> -o <out.mp4> --debug-tracks              draw tracked points

#include <fstream>
#include <iostream>
#include <string>
#include <vector>

#include <opencv2/core.hpp>
#include <opencv2/imgproc.hpp>
#include <opencv2/videoio.hpp>

#include "feature_tracker.hpp"
#include "frame_warper.hpp"
#include "metrics.hpp"
#include "motion_estimator.hpp"
#include "trajectory.hpp"

// Tracking runs on a shrunken frame because 1080x1920 is slow and not needed.
const int kTrackingWidth = 480;

struct Options {
    std::string input_path;
    std::string output_path = "results/stable.mp4";
    bool output_given = false;        // did the user pass -o?
    std::string motion_csv_path;      // empty = not requested
    std::string trajectory_csv_path;  // empty = not requested
    std::string smoother = "avg";
    int radius = 30;                  // moving-average window radius, in frames
    bool side_by_side = false;
    bool debug_tracks = false;
};

// Returns false if the arguments are unusable.
bool parse_args(int argc, char** argv, Options& options) {
    if (argc < 2) {
        return false;
    }
    options.input_path = argv[1];
    for (int i = 2; i < argc; ++i) {
        const std::string arg = argv[i];
        const bool has_value = i + 1 < argc;
        if (arg == "-o" && has_value) {
            options.output_path = argv[++i];
            options.output_given = true;
        } else if (arg == "--motion-csv" && has_value) {
            options.motion_csv_path = argv[++i];
        } else if (arg == "--trajectory-csv" && has_value) {
            options.trajectory_csv_path = argv[++i];
        } else if (arg == "--smoother" && has_value) {
            options.smoother = argv[++i];
        } else if (arg == "--radius" && has_value) {
            options.radius = std::stoi(argv[++i]);
        } else if (arg == "--side-by-side") {
            options.side_by_side = true;
        } else if (arg == "--debug-tracks") {
            options.debug_tracks = true;
        } else {
            std::cerr << "Unknown or incomplete argument: " << arg << "\n";
            return false;
        }
    }
    return true;
}

cv::Mat shrink_to_tracking_size(const cv::Mat& frame) {
    cv::Mat small_frame;
    const int small_height = frame.rows * kTrackingWidth / frame.cols;
    cv::resize(frame, small_frame, cv::Size(kTrackingWidth, small_height));
    return small_frame;
}

// Tracks features between consecutive frames and draws them.
// Green dot = where a feature is now. Orange trail = where it has been.
int draw_tracks(cv::VideoCapture& cap, cv::VideoWriter& writer, int out_width,
                int out_height) {
    FeatureTracker tracker;
    cv::Mat frame, gray, previous_gray;
    cv::Mat trails = cv::Mat::zeros(out_height, out_width, CV_8UC3);
    int written = 0;

    while (cap.read(frame)) {
        cv::Mat small_frame = shrink_to_tracking_size(frame);
        cv::cvtColor(small_frame, gray, cv::COLOR_BGR2GRAY);

        if (!previous_gray.empty()) {
            TrackedPoints points = tracker.track(previous_gray, gray);

            trails *= 0.9;  // fade old trails a little each frame
            for (size_t i = 0; i < points.current.size(); ++i) {
                cv::line(trails, points.previous[i], points.current[i],
                         cv::Scalar(0, 200, 255), 2);  // BGR: orange
            }
            cv::add(small_frame, trails, small_frame);  // overlay trails
            for (const cv::Point2f& p : points.current) {
                cv::circle(small_frame, p, 3, cv::Scalar(0, 255, 0), -1);
            }
        }

        writer.write(small_frame);
        ++written;
        gray.copyTo(previous_gray);  // copy, because `gray` is overwritten next loop
    }
    return written;
}

// Estimates the camera motion between every pair of consecutive frames.
// Result has one Motion per frame: motions[i] is the motion from frame i-1 to
// frame i, and motions[0] is all zeros (the first frame has no previous frame).
// dx and dy are converted to FULL-resolution pixels (we track on a shrunken
// frame but warp the full-size frame) and to rotation about the frame center.
std::vector<Motion> estimate_all_motions(cv::VideoCapture& cap, int full_width) {
    const double to_full_res = static_cast<double>(full_width) / kTrackingWidth;
    FeatureTracker tracker;
    MotionEstimator estimator;
    std::vector<Motion> motions;
    cv::Mat frame, gray, previous_gray;

    while (cap.read(frame)) {
        cv::cvtColor(shrink_to_tracking_size(frame), gray, cv::COLOR_BGR2GRAY);

        Motion m;  // frame 0 stays "no motion"
        if (!previous_gray.empty()) {
            m = estimator.estimate(tracker.track(previous_gray, gray));
            m.dx *= to_full_res;
            m.dy *= to_full_res;
            m = motion_about_point(m, frame.cols / 2.0, frame.rows / 2.0);
        }
        motions.push_back(m);
        gray.copyTo(previous_gray);
    }
    return motions;
}

bool write_motion_csv(const std::vector<Motion>& motions, const std::string& path) {
    std::ofstream csv(path);  // like open(path, "w") in Python
    if (!csv.is_open()) {
        std::cerr << "Could not open " << path << " for writing\n";
        return false;
    }
    csv << "frame,dx,dy,da,scale,inliers,valid\n";
    for (size_t i = 1; i < motions.size(); ++i) {  // skip frame 0 (no motion)
        const Motion& m = motions[i];
        csv << i << ',' << m.dx << ',' << m.dy << ',' << m.da << ',' << m.scale << ','
            << m.inliers << ',' << (m.valid ? 1 : 0) << '\n';
    }
    return true;
}

bool write_trajectory_csv(const std::vector<TrajectoryPoint>& raw,
                          const std::vector<TrajectoryPoint>& smooth,
                          const std::string& path) {
    std::ofstream csv(path);
    if (!csv.is_open()) {
        std::cerr << "Could not open " << path << " for writing\n";
        return false;
    }
    csv << "frame,raw_x,raw_y,raw_a,smooth_x,smooth_y,smooth_a\n";
    for (size_t i = 0; i < raw.size(); ++i) {
        csv << i << ',' << raw[i].x << ',' << raw[i].y << ',' << raw[i].angle << ','
            << smooth[i].x << ',' << smooth[i].y << ',' << smooth[i].angle << '\n';
    }
    return true;
}

// Pass 2: re-read the video, shift/rotate each frame by (smooth - raw), write it.
// With side_by_side, each output frame is [original | stabilized], both at half size.
int write_stabilized_video(cv::VideoCapture& cap, cv::VideoWriter& writer,
                          const std::vector<TrajectoryPoint>& raw,
                          const std::vector<TrajectoryPoint>& smooth,
                          bool side_by_side) {
    cv::Mat frame;
    size_t i = 0;
    InterframeFidelity itf_before, itf_after;  // measured in memory, before encoding
    while (i < raw.size() && cap.read(frame)) {
        // Correction = where the camera SHOULD have been minus where it was.
        const double dx = smooth[i].x - raw[i].x;
        const double dy = smooth[i].y - raw[i].y;
        const double da = smooth[i].angle - raw[i].angle;
        cv::Mat stable = warp_frame(frame, dx, dy, da);
        itf_before.add(frame);
        itf_after.add(stable);

        if (side_by_side) {
            const cv::Size half(frame.cols / 2, frame.rows / 2);
            cv::Mat left, right, both;
            cv::resize(frame, left, half);
            cv::resize(stable, right, half);
            cv::hconcat(left, right, both);
            writer.write(both);
        } else {
            writer.write(stable);
        }
        ++i;
    }
    std::cout << "ITF (higher = steadier): original " << itf_before.average()
              << " dB, stabilized " << itf_after.average() << " dB\n";
    return static_cast<int>(i);
}

int run_stabilize(const Options& options, cv::VideoCapture& cap, int width, int height,
                  double fps) {
    if (options.smoother != "avg") {
        std::cerr << "Smoother '" << options.smoother << "' is not available yet "
                  << "(only 'avg'; 'kalman' comes in Week 4)\n";
        return 1;
    }

    std::cout << "Pass 1: estimating motion...\n";
    const std::vector<Motion> motions = estimate_all_motions(cap, width);
    if (!options.motion_csv_path.empty() &&
        !write_motion_csv(motions, options.motion_csv_path)) {
        return 1;
    }

    const std::vector<TrajectoryPoint> raw = accumulate_trajectory(motions);
    const std::vector<TrajectoryPoint> smooth =
        smooth_moving_average(raw, options.radius);
    if (!options.trajectory_csv_path.empty() &&
        !write_trajectory_csv(raw, smooth, options.trajectory_csv_path)) {
        return 1;
    }

    const int out_width = options.side_by_side ? (width / 2) * 2 : width;
    const int out_height = options.side_by_side ? height / 2 : height;
    cv::VideoWriter writer(options.output_path,
                           cv::VideoWriter::fourcc('m', 'p', '4', 'v'), fps,
                           cv::Size(out_width, out_height));
    if (!writer.isOpened()) {
        std::cerr << "Could not open " << options.output_path << " for writing\n";
        return 1;
    }

    std::cout << "Pass 2: warping frames...\n";
    cv::VideoCapture cap_again(options.input_path);  // start over from frame 0
    const int written = write_stabilized_video(cap_again, writer, raw, smooth,
                                               options.side_by_side);
    std::cout << "Wrote " << written << " frames to " << options.output_path << "\n";
    return 0;
}

int main(int argc, char** argv) {
    Options options;
    if (!parse_args(argc, argv, options)) {
        std::cerr << "Usage: " << argv[0] << " <input video> [-o <out.mp4>]"
                  << " [--smoother avg] [--radius N] [--side-by-side]"
                  << " [--trajectory-csv <file>] [--motion-csv <file>]"
                  << " [--debug-tracks]\n";
        return 1;
    }

    cv::VideoCapture cap(options.input_path);
    if (!cap.isOpened()) {
        std::cerr << "Could not open " << options.input_path << "\n";
        return 1;
    }
    const int width = static_cast<int>(cap.get(cv::CAP_PROP_FRAME_WIDTH));
    const int height = static_cast<int>(cap.get(cv::CAP_PROP_FRAME_HEIGHT));
    const double fps = cap.get(cv::CAP_PROP_FPS);
    std::cout << "Resolution: " << width << "x" << height << "  FPS: " << fps
              << "  Frames: " << static_cast<int>(cap.get(cv::CAP_PROP_FRAME_COUNT))
              << "\n";

    if (options.debug_tracks) {
        const int out_height = height * kTrackingWidth / width;
        cv::VideoWriter writer(options.output_path,
                               cv::VideoWriter::fourcc('m', 'p', '4', 'v'), fps,
                               cv::Size(kTrackingWidth, out_height));
        if (!writer.isOpened()) {
            std::cerr << "Could not open " << options.output_path << " for writing\n";
            return 1;
        }
        const int written = draw_tracks(cap, writer, kTrackingWidth, out_height);
        std::cout << "Wrote " << written << " frames to " << options.output_path << "\n";
        return 0;
    }

    // Motion CSV only (no -o video): handy for plotting.
    if (!options.motion_csv_path.empty() && !options.output_given) {
        const std::vector<Motion> motions = estimate_all_motions(cap, width);
        if (!write_motion_csv(motions, options.motion_csv_path)) {
            return 1;
        }
        std::cout << "Wrote " << motions.size() - 1 << " motion rows to "
                  << options.motion_csv_path << "\n";
        return 0;
    }

    return run_stabilize(options, cap, width, height, fps);
}
