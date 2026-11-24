// trajectory.cpp: camera path = running sum of per-frame motion, then smoothing.

#include "trajectory.hpp"

#include <algorithm>

std::vector<TrajectoryPoint> accumulate_trajectory(const std::vector<Motion>& motions) {
    std::vector<TrajectoryPoint> trajectory;
    trajectory.reserve(motions.size());

    // Running totals. Per-frame motion is a speed; adding it up gives a position.
    // (Adding the angles and slides separately is only an approximation of
    // composing the transforms, but it is accurate for the small motions between
    // two video frames.)
    TrajectoryPoint total;
    for (const Motion& m : motions) {
        total.x += m.dx;
        total.y += m.dy;
        total.angle += m.da;
        trajectory.push_back(total);
    }
    return trajectory;
}

std::vector<TrajectoryPoint> smooth_moving_average(
    const std::vector<TrajectoryPoint>& trajectory, int radius) {
    const int n = static_cast<int>(trajectory.size());
    std::vector<TrajectoryPoint> smoothed(trajectory.size());

    for (int i = 0; i < n; ++i) {
        // Window of frames [i - radius, i + radius], clipped to the frames that exist.
        const int first = std::max(0, i - radius);
        const int last = std::min(n - 1, i + radius);

        TrajectoryPoint sum;
        for (int j = first; j <= last; ++j) {
            sum.x += trajectory[j].x;
            sum.y += trajectory[j].y;
            sum.angle += trajectory[j].angle;
        }
        const double count = last - first + 1;
        smoothed[i].x = sum.x / count;
        smoothed[i].y = sum.y / count;
        smoothed[i].angle = sum.angle / count;
    }
    return smoothed;
}
