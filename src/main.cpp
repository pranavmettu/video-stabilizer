// main.cpp: command-line entry point.
//   ./stabilize <video> -o <out.mp4>                  copies the video (Week 0)
//   ./stabilize <video> -o <out.mp4> --debug-tracks   draws tracked features (Week 1)

#include <iostream>
#include <string>

#include <opencv2/core.hpp>
#include <opencv2/imgproc.hpp>
#include <opencv2/videoio.hpp>

#include "feature_tracker.hpp"

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

int main(int argc, char** argv) {
    if (argc < 2) {
        std::cerr << "Usage: " << argv[0]
                  << " <input video> [-o <output video>] [--debug-tracks]\n";
        return 1;
    }

    const std::string input_path = argv[1];
    std::string output_path = "results/copy.mp4";
    bool debug_tracks = false;
    for (int i = 2; i < argc; ++i) {
        const std::string arg = argv[i];
        if (arg == "-o" && i + 1 < argc) {
            output_path = argv[++i];
        } else if (arg == "--debug-tracks") {
            debug_tracks = true;
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
