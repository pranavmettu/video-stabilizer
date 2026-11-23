// main.cpp: Week 0. Open a video, print its info, write a copy.

#include <iostream>
#include <string>

#include <opencv2/core.hpp>
#include <opencv2/videoio.hpp>

int main(int argc, char** argv) {
    if (argc < 2) {
        std::cerr << "Usage: " << argv[0] << " <input video> [-o <output video>]\n";
        return 1;
    }

    const std::string input_path = argv[1];
    std::string output_path = "results/copy.mp4";
    for (int i = 2; i + 1 < argc; ++i) {
        if (std::string(argv[i]) == "-o") {
            output_path = argv[i + 1];
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

    cv::VideoWriter writer(output_path,
                           cv::VideoWriter::fourcc('m', 'p', '4', 'v'),
                           fps, cv::Size(width, height));
    if (!writer.isOpened()) {
        std::cerr << "Could not open " << output_path << " for writing\n";
        return 1;
    }

    cv::Mat frame;
    int written = 0;
    while (cap.read(frame)) {
        writer.write(frame);
        ++written;
    }

    std::cout << "Wrote " << written << " frames to " << output_path << "\n";
    return 0;
}
