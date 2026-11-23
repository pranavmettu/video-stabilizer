// feature_tracker.cpp: HOW the feature tracker works.

#include "feature_tracker.hpp"

#include <opencv2/features.hpp>  // goodFeaturesToTrack (moved here in OpenCV 5)
#include <opencv2/video.hpp>     // calcOpticalFlowPyrLK

FeatureTracker::FeatureTracker(int max_corners) : max_corners_(max_corners) {}

TrackedPoints FeatureTracker::track(const cv::Mat& previous_gray,
                                    const cv::Mat& current_gray) const {
    TrackedPoints result;

    // Step 1: find good corners in the previous frame.
    // Corners are points whose neighborhood looks different in EVERY direction,
    // so they can be located precisely. (A point on a plain wall or along an
    // edge can slide without the image changing, so it can't be pinned down.)
    std::vector<cv::Point2f> corners;
    const double quality_level = 0.01;  // keep corners at least 1% as strong as the best
    const double min_distance = 15;     // pixels between corners, so they spread out
    cv::goodFeaturesToTrack(previous_gray, corners, max_corners_,
                            quality_level, min_distance);
    if (corners.empty()) {
        return result;
    }

    // Step 2: optical flow. For each corner, find where it moved in the current
    // frame. Lucas-Kanade assumes brightness stays constant, and that nearby
    // pixels move together. "Pyr" = it works on a pyramid of shrunken images
    // so it can follow bigger motions.
    std::vector<cv::Point2f> moved;
    std::vector<unsigned char> status;  // 1 = tracked OK, 0 = lost
    std::vector<float> error;
    cv::calcOpticalFlowPyrLK(previous_gray, current_gray, corners, moved,
                             status, error);

    // Step 3: keep only the points that were tracked successfully.
    for (size_t i = 0; i < corners.size(); ++i) {
        if (status[i]) {
            result.previous.push_back(corners[i]);
            result.current.push_back(moved[i]);
        }
    }
    return result;
}
