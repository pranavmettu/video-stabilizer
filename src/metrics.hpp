// metrics.hpp: numbers that say how steady a video is.

#pragma once

#include <opencv2/core.hpp>

// Peak signal-to-noise ratio between two same-size images, in dB.
// Higher = the images are more alike.
double psnr(const cv::Mat& a, const cv::Mat& b);

// ITF (interframe transformation fidelity): the average PSNR between each frame
// and the previous one. A steadier video has frames that look more alike, so a
// higher ITF means steadier. Only the center of each frame is compared, so black
// borders created by warping don't count.
class InterframeFidelity {
public:
    void add(const cv::Mat& frame);  // call once per frame, in order
    double average() const;          // average PSNR so far (0 if fewer than 2 frames)

private:
    cv::Mat previous_;
    double total_psnr_ = 0.0;
    int pairs_ = 0;
};
