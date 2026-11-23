// motion_estimator.hpp: turns matched points into ONE camera motion per frame pair.

#pragma once

#include "feature_tracker.hpp"  // for TrackedPoints

// How the camera moved from one frame to the next.
struct Motion {
    double dx = 0.0;     // slide left/right, in pixels (m02 of the matrix)
    double dy = 0.0;     // slide up/down, in pixels (m12 of the matrix)
    double da = 0.0;     // rotation angle, in radians
    double scale = 1.0;  // zoom factor (1.0 = no zoom)
    int inliers = 0;     // how many points agreed with this motion
    bool valid = false;  // false if we could not estimate (too few points, etc.)
};

class MotionEstimator {
public:
    // Fits a similarity transform (slide + rotate + zoom) to the point pairs.
    // RANSAC ignores outliers such as points on a person walking through the frame.
    // If the fit fails, returns a Motion with valid = false and "no motion" values.
    Motion estimate(const TrackedPoints& points) const;
};
