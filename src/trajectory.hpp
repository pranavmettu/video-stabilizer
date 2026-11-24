// trajectory.hpp: declarations for building and smoothing the camera path.
//
// The bodies are in src/trajectory.cpp. This file only says what exists.

#pragma once

#include <vector>

#include "motion_estimator.hpp"  // for Motion

// Where the camera "is" at one moment, measured from where it started.
// Same three numbers as a Motion, but TOTALS instead of per-frame changes.
struct TrajectoryPoint {
    double x = 0.0;      // total slide left/right so far (pixels)
    double y = 0.0;      // total slide up/down so far (pixels)
    double angle = 0.0;  // total rotation so far (radians)
};

// Turns per-frame motions into a camera path with a running sum.
//   result[i] = motions[0] + motions[1] + ... + motions[i]  (for x, y and angle)
// The result has the SAME length as `motions`.
// A Motion with valid == false has all-zero values (see MotionEstimator), so it
// adds nothing to the running sum: the path just stays put for that frame.
std::vector<TrajectoryPoint> accumulate_trajectory(const std::vector<Motion>& motions);

// Smooths the path with a moving average. Each output point is the average of
// the input points within `radius` frames on either side (a window of
// 2*radius + 1 frames). The result has the same length as `trajectory`.
// Near the start and end, where the window would run off the ends of the vector,
// the window is shrunk to the frames that exist (so the first point is the
// average of frames 0..radius).
std::vector<TrajectoryPoint> smooth_moving_average(
    const std::vector<TrajectoryPoint>& trajectory, int radius);
