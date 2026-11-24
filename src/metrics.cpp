// metrics.cpp

#include "metrics.hpp"

#include <cmath>

double psnr(const cv::Mat& a, const cv::Mat& b) {
    // Mean squared error over every pixel and channel.
    const double squared_error = cv::norm(a, b, cv::NORM_L2SQR);
    const double mse = squared_error / (static_cast<double>(a.total()) * a.channels());
    if (mse < 1e-10) {
        return 100.0;  // identical images: cap instead of dividing by zero
    }
    // PSNR = 10 * log10(255^2 / MSE) for 8-bit images.
    return 10.0 * std::log10(255.0 * 255.0 / mse);
}

void InterframeFidelity::add(const cv::Mat& frame) {
    // Keep the middle 50% of the frame (a view, then clone so we own the pixels).
    const cv::Rect middle(frame.cols / 4, frame.rows / 4, frame.cols / 2, frame.rows / 2);
    cv::Mat center = frame(middle).clone();

    if (!previous_.empty()) {
        total_psnr_ += psnr(previous_, center);
        ++pairs_;
    }
    previous_ = center;
}

double InterframeFidelity::average() const {
    return pairs_ > 0 ? total_psnr_ / pairs_ : 0.0;
}
