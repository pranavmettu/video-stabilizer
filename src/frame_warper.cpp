// frame_warper.cpp

#include "frame_warper.hpp"

#include <cmath>

#include <opencv2/imgproc.hpp>  // warpAffine

cv::Mat warp_frame(const cv::Mat& frame, double dx, double dy, double angle) {
    const double c = std::cos(angle);
    const double s = std::sin(angle);
    const double cx = frame.cols / 2.0;
    const double cy = frame.rows / 2.0;

    // Rotation about the frame center, then a slide by (dx, dy).
    //   p' = R * (p - center) + center + (dx, dy)
    // Expanding gives the 2x3 matrix [R | t] with t = center - R*center + (dx, dy).
    // R = [c -s; s c] is the same matrix form we read the angle from in Week 2.
    const double tx = cx - (c * cx - s * cy) + dx;
    const double ty = cy - (s * cx + c * cy) + dy;
    double values[6] = {c, -s, tx,
                        s,  c, ty};
    cv::Mat m(2, 3, CV_64F, values);

    cv::Mat warped;
    cv::warpAffine(frame, warped, m, frame.size());
    return warped;
}
