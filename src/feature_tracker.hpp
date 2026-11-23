// feature_tracker.hpp: the DECLARATION of our feature tracker.
//
// A header (.hpp) says WHAT exists: the names and types other files can use.
// The matching .cpp file says HOW it works. Other files #include this header,
// so they know FeatureTracker exists without seeing its inner workings.
// (Python has no direct equivalent: a class definition and its body live
// together. Here we split them so each .cpp compiles on its own.)

#pragma once  // "include this header only once, even if several files include it"

#include <vector>

#include <opencv2/core.hpp>

// The result of tracking: point i in `previous` moved to point i in `current`.
// A struct is a class where everything is public. Think of a Python dataclass.
struct TrackedPoints {
    std::vector<cv::Point2f> previous;  // positions in the earlier frame
    std::vector<cv::Point2f> current;   // positions in the later frame
};

class FeatureTracker {
public:
    // Constructor: runs when you write `FeatureTracker tracker(500);`
    explicit FeatureTracker(int max_corners = 500);

    // Finds corners in `previous_gray`, follows them into `current_gray`,
    // and returns only the points that were tracked successfully.
    // Both images must be single-channel (grayscale) and the same size.
    // `const cv::Mat&` means "a read-only reference": no copy of the image is made.
    TrackedPoints track(const cv::Mat& previous_gray,
                        const cv::Mat& current_gray) const;

private:
    int max_corners_;  // trailing underscore = member variable (a naming convention)
};
