// motion_estimator.cpp

#include "motion_estimator.hpp"

#include <cmath>

#include <opencv2/geometry.hpp>  // estimateAffinePartial2D (OpenCV 5 location)

Motion MotionEstimator::estimate(const TrackedPoints& points) const {
    Motion motion;  // starts as "no motion, not valid"

    // A similarity transform has 4 unknowns and each point pair gives 2 equations,
    // so 2 pairs is the bare minimum. Require a few more so the fit means something.
    const size_t kMinPoints = 6;
    if (points.previous.size() < kMinPoints) {
        return motion;
    }

    // RANSAC fit. `inlier_mask` gets one entry per point: 1 = agreed with the fit.
    // 3.0 = a point counts as an inlier if it lands within 3 pixels of the prediction.
    cv::Mat inlier_mask;
    cv::Mat m = cv::estimateAffinePartial2D(points.previous, points.current,
                                            inlier_mask, cv::RANSAC, 3.0);
    if (m.empty()) {
        return motion;  // RANSAC could not find a good fit
    }

    // `m` is a 2x3 matrix of doubles. m.at<double>(row, col) reads one entry,
    // like m[row, col] in NumPy.
    const double m00 = m.at<double>(0, 0);
    const double m10 = m.at<double>(1, 0);

    motion.dx = m.at<double>(0, 2);
    motion.dy = m.at<double>(1, 2);
    // m00 = s*cos(a) and m10 = s*sin(a), so atan2 recovers the angle a
    // (the scale s cancels out) and the length of (m00, m10) is the scale s.
    motion.da = std::atan2(m10, m00);
    motion.scale = std::sqrt(m00 * m00 + m10 * m10);
    motion.inliers = cv::countNonZero(inlier_mask);
    motion.valid = true;
    return motion;
}
