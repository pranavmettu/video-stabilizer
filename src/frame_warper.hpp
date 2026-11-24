// frame_warper.hpp: shifts and rotates a frame to cancel camera shake.

#pragma once

#include <opencv2/core.hpp>

// Moves the frame by (dx, dy) pixels and rotates it by `angle` radians about the
// frame's center. Uncovered edges become black. The output has the same size.
cv::Mat warp_frame(const cv::Mat& frame, double dx, double dy, double angle);
