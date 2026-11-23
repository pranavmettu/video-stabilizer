// main.cpp: command-line entry point.
//   ./stabilize <video> -o <out.mp4>                  copies the video (Week 0)
//   ./stabilize <video> -o <out.mp4> --debug-tracks   draws tracked features (Week 1)
//   ./stabilize <video> --motion-csv <out.csv>        writes per-frame motion (Week 2)

#include <fstream>
#include <iostream>
#include <string>

#include <opencv2/core.hpp>
#include <opencv2/imgproc.hpp>
#include <opencv2/videoio.hpp>

#include "feature_tracker.hpp"
#include "motion_estimator.hpp"

// Tracking runs on a shrunken frame because 1080x1920 is slow and not needed.
const int kTrackingWidth = 480;

// Reads every frame and writes it back out unchanged.
int copy_video(cv::VideoCapture& cap, cv::VideoWriter& writer) {
    cv::Mat frame;
    int written = 0;
    while (cap.read(frame)) {
        writer.write(frame);
        ++written;
    }
    return written;
}

// Tracks features between consecutive frames and draws them.
// Green dot = where a feature is now. Fading trail = where it has been.
// `cap` is passed by reference (&): the function uses the caller's object
// directly instead of making a copy.
int draw_tracks(cv::VideoCapture& cap, cv::VideoWriter& writer, int out_width,
                int out_height) {
    FeatureTracker tracker;
    cv::Mat frame, small_frame, gray, previous_gray;
    cv::Mat trails = cv::Mat::zeros(out_height, out_width, CV_8UC3);
    int written = 0;

    while (cap.read(frame)) {
        cv::resize(frame, small_frame, cv::Size(out_width, out_height));
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

// Estimates the camera motion between every pair of consecutive frames and
// writes one CSV row per pair: frame, dx, dy, da, scale, inliers, valid.
// Frame i's row describes the motion from frame i-1 to frame i.
// dx and dy are converted back to FULL-resolution pixels, because we track on a
// shrunken frame but will warp the full-size frame later.
int write_motion_csv(cv::VideoCapture& cap, const std::string& csv_path,
                     int full_width) {
    std::ofstream csv(csv_path);  // like open(csv_path, "w") in Python
    if (!csv.is_open()) {
        std::cerr << "Could not open " << csv_path << " for writing\n";
        return -1;
    }
    csv << "frame,dx,dy,da,scale,inliers,valid\n";

    const double to_full_res = static_cast<double>(full_width) / kTrackingWidth;
    FeatureTracker tracker;
    MotionEstimator estimator;
    cv::Mat frame, small_frame, gray, previous_gray;
    int frame_index = 0;
    int rows = 0;

    while (cap.read(frame)) {
        const int small_height = frame.rows * kTrackingWidth / frame.cols;
        cv::resize(frame, small_frame, cv::Size(kTrackingWidth, small_height));
        cv::cvtColor(small_frame, gray, cv::COLOR_BGR2GRAY);

        if (!previous_gray.empty()) {
            Motion m = estimator.estimate(tracker.track(previous_gray, gray));
            csv << frame_index << ',' << m.dx * to_full_res << ','
                << m.dy * to_full_res << ',' << m.da << ',' << m.scale << ','
                << m.inliers << ',' << (m.valid ? 1 : 0) << '\n';
            ++rows;
        }
        gray.copyTo(previous_gray);
        ++frame_index;
    }
    return rows;
}

int main(int argc, char** argv) {
    if (argc < 2) {
        std::cerr << "Usage: " << argv[0]
                  << " <input video> [-o <output video>] [--debug-tracks]"
                     " [--motion-csv <file.csv>]\n";
        return 1;
    }

    const std::string input_path = argv[1];
    std::string output_path = "results/copy.mp4";
    std::string motion_csv_path;  // empty = not requested
    bool debug_tracks = false;
    for (int i = 2; i < argc; ++i) {
        const std::string arg = argv[i];
        if (arg == "-o" && i + 1 < argc) {
            output_path = argv[++i];
        } else if (arg == "--debug-tracks") {
            debug_tracks = true;
        } else if (arg == "--motion-csv" && i + 1 < argc) {
            motion_csv_path = argv[++i];
        }
    }

    cv::VideoCapture cap(input_path);
    if (!cap.isOpened()) {
        std::cerr << "Could not open " << input_path << "\n";
        return 1;
    }

    const int width = static_cast<int>(cap.get(cv::CAP_PROP_FRAME_WIDTH));
    const int height = static_cast<int>(cap.get(cv::CAP_PROP_FRAME_HEIGHT));
    const double fps = cap.get(cv::CAP_PROP_FPS);
    const int frame_count = static_cast<int>(cap.get(cv::CAP_PROP_FRAME_COUNT));

    std::cout << "Resolution: " << width << "x" << height << "\n"
              << "FPS:        " << fps << "\n"
              << "Frames:     " << frame_count << "\n";

    if (!motion_csv_path.empty()) {
        const int rows = write_motion_csv(cap, motion_csv_path, width);
        if (rows < 0) {
            return 1;
        }
        std::cout << "Wrote " << rows << " motion rows to " << motion_csv_path << "\n";
        return 0;
    }

    // The debug video is the small tracking size; the copy keeps full size.
    const int out_width = debug_tracks ? kTrackingWidth : width;
    const int out_height = debug_tracks ? height * kTrackingWidth / width : height;

    cv::VideoWriter writer(output_path,
                           cv::VideoWriter::fourcc('m', 'p', '4', 'v'), fps,
                           cv::Size(out_width, out_height));
    if (!writer.isOpened()) {
        std::cerr << "Could not open " << output_path << " for writing\n";
        return 1;
    }

    const int written = debug_tracks
                            ? draw_tracks(cap, writer, out_width, out_height)
                            : copy_video(cap, writer);

    std::cout << "Wrote " << written << " frames to " << output_path << "\n";
    return 0;
}
